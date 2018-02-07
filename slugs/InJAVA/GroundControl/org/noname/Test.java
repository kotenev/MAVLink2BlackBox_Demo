
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
            long id = id__g(src);
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
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED));
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_GENERIC);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_UNINIT);
            assert(pack.custom_mode_GET() == 508460802L);
            assert(pack.mavlink_version_GET() == (char)61);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.mavlink_version_SET((char)61) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID) ;
        p0.custom_mode_SET(508460802L) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_UNINIT) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED)) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_GENERIC) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.current_battery_GET() == (short)8755);
            assert(pack.voltage_battery_GET() == (char)3842);
            assert(pack.errors_comm_GET() == (char)34384);
            assert(pack.errors_count4_GET() == (char)21213);
            assert(pack.battery_remaining_GET() == (byte)4);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.errors_count1_GET() == (char)30508);
            assert(pack.load_GET() == (char)64052);
            assert(pack.errors_count2_GET() == (char)1319);
            assert(pack.drop_rate_comm_GET() == (char)48784);
            assert(pack.errors_count3_GET() == (char)29129);
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY));
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.errors_count4_SET((char)21213) ;
        p1.battery_remaining_SET((byte)4) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY)) ;
        p1.errors_count3_SET((char)29129) ;
        p1.load_SET((char)64052) ;
        p1.errors_comm_SET((char)34384) ;
        p1.errors_count1_SET((char)30508) ;
        p1.current_battery_SET((short)8755) ;
        p1.errors_count2_SET((char)1319) ;
        p1.voltage_battery_SET((char)3842) ;
        p1.drop_rate_comm_SET((char)48784) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1464309453L);
            assert(pack.time_unix_usec_GET() == 4290134447994489756L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(1464309453L) ;
        p2.time_unix_usec_SET(4290134447994489756L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -2.711284E38F);
            assert(pack.vz_GET() == -4.9877695E37F);
            assert(pack.yaw_rate_GET() == 1.5740962E38F);
            assert(pack.afy_GET() == 7.612912E37F);
            assert(pack.x_GET() == -2.4579289E38F);
            assert(pack.vy_GET() == 2.3423333E38F);
            assert(pack.afz_GET() == -1.7831025E38F);
            assert(pack.type_mask_GET() == (char)23500);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.vx_GET() == -4.378727E37F);
            assert(pack.afx_GET() == -2.9015952E38F);
            assert(pack.y_GET() == -2.7801452E38F);
            assert(pack.z_GET() == 2.0702038E38F);
            assert(pack.time_boot_ms_GET() == 2661995311L);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.x_SET(-2.4579289E38F) ;
        p3.yaw_rate_SET(1.5740962E38F) ;
        p3.z_SET(2.0702038E38F) ;
        p3.vx_SET(-4.378727E37F) ;
        p3.vz_SET(-4.9877695E37F) ;
        p3.y_SET(-2.7801452E38F) ;
        p3.time_boot_ms_SET(2661995311L) ;
        p3.vy_SET(2.3423333E38F) ;
        p3.yaw_SET(-2.711284E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p3.afx_SET(-2.9015952E38F) ;
        p3.afz_SET(-1.7831025E38F) ;
        p3.type_mask_SET((char)23500) ;
        p3.afy_SET(7.612912E37F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)114);
            assert(pack.target_system_GET() == (char)142);
            assert(pack.time_usec_GET() == 7148232720982248323L);
            assert(pack.seq_GET() == 52256519L);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.target_system_SET((char)142) ;
        p4.time_usec_SET(7148232720982248323L) ;
        p4.target_component_SET((char)114) ;
        p4.seq_SET(52256519L) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.passkey_LEN(ph) == 9);
            assert(pack.passkey_TRY(ph).equals("meemxrgos"));
            assert(pack.version_GET() == (char)52);
            assert(pack.target_system_GET() == (char)3);
            assert(pack.control_request_GET() == (char)243);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)3) ;
        p5.passkey_SET("meemxrgos", PH) ;
        p5.version_SET((char)52) ;
        p5.control_request_SET((char)243) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.ack_GET() == (char)89);
            assert(pack.control_request_GET() == (char)78);
            assert(pack.gcs_system_id_GET() == (char)113);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.control_request_SET((char)78) ;
        p6.ack_SET((char)89) ;
        p6.gcs_system_id_SET((char)113) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 30);
            assert(pack.key_TRY(ph).equals("qGhwjAljvssquyCytgqcgjtnqzhsIz"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("qGhwjAljvssquyCytgqcgjtnqzhsIz", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_GUIDED_ARMED);
            assert(pack.custom_mode_GET() == 1633968730L);
            assert(pack.target_system_GET() == (char)201);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED) ;
        p11.target_system_SET((char)201) ;
        p11.custom_mode_SET(1633968730L) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short) -13522);
            assert(pack.target_component_GET() == (char)115);
            assert(pack.target_system_GET() == (char)168);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("ly"));
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_component_SET((char)115) ;
        p20.param_index_SET((short) -13522) ;
        p20.target_system_SET((char)168) ;
        p20.param_id_SET("ly", PH) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)232);
            assert(pack.target_component_GET() == (char)177);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)232) ;
        p21.target_component_SET((char)177) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_GET() == 2.9032896E38F);
            assert(pack.param_count_GET() == (char)33174);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("maeGuxhldqtS"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
            assert(pack.param_index_GET() == (char)39369);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_count_SET((char)33174) ;
        p22.param_value_SET(2.9032896E38F) ;
        p22.param_id_SET("maeGuxhldqtS", PH) ;
        p22.param_index_SET((char)39369) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)206);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("vooktonilbiw"));
            assert(pack.target_system_GET() == (char)176);
            assert(pack.param_value_GET() == -1.2661069E38F);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16) ;
        p23.param_value_SET(-1.2661069E38F) ;
        p23.target_system_SET((char)176) ;
        p23.param_id_SET("vooktonilbiw", PH) ;
        p23.target_component_SET((char)206) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.alt_ellipsoid_TRY(ph) == -367120056);
            assert(pack.epv_GET() == (char)11940);
            assert(pack.eph_GET() == (char)4576);
            assert(pack.cog_GET() == (char)16438);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
            assert(pack.lon_GET() == -608843538);
            assert(pack.vel_GET() == (char)61642);
            assert(pack.time_usec_GET() == 3805686137329084277L);
            assert(pack.h_acc_TRY(ph) == 2065807482L);
            assert(pack.lat_GET() == -1734526514);
            assert(pack.hdg_acc_TRY(ph) == 2462042188L);
            assert(pack.v_acc_TRY(ph) == 2935968579L);
            assert(pack.vel_acc_TRY(ph) == 437768785L);
            assert(pack.satellites_visible_GET() == (char)202);
            assert(pack.alt_GET() == 528316780);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.h_acc_SET(2065807482L, PH) ;
        p24.alt_SET(528316780) ;
        p24.cog_SET((char)16438) ;
        p24.satellites_visible_SET((char)202) ;
        p24.lon_SET(-608843538) ;
        p24.vel_SET((char)61642) ;
        p24.v_acc_SET(2935968579L, PH) ;
        p24.epv_SET((char)11940) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED) ;
        p24.time_usec_SET(3805686137329084277L) ;
        p24.hdg_acc_SET(2462042188L, PH) ;
        p24.lat_SET(-1734526514) ;
        p24.vel_acc_SET(437768785L, PH) ;
        p24.eph_SET((char)4576) ;
        p24.alt_ellipsoid_SET(-367120056, PH) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)11, (char)61, (char)36, (char)227, (char)64, (char)76, (char)61, (char)45, (char)56, (char)30, (char)202, (char)119, (char)220, (char)33, (char)219, (char)152, (char)87, (char)243, (char)148, (char)101}));
            assert(pack.satellites_visible_GET() == (char)176);
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)142, (char)18, (char)182, (char)53, (char)128, (char)136, (char)85, (char)98, (char)2, (char)89, (char)187, (char)106, (char)201, (char)106, (char)226, (char)150, (char)165, (char)54, (char)241, (char)20}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)37, (char)246, (char)101, (char)131, (char)185, (char)190, (char)25, (char)108, (char)132, (char)214, (char)20, (char)14, (char)235, (char)140, (char)105, (char)115, (char)39, (char)112, (char)38, (char)31}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)16, (char)156, (char)77, (char)37, (char)173, (char)221, (char)93, (char)35, (char)36, (char)93, (char)222, (char)196, (char)228, (char)25, (char)101, (char)174, (char)58, (char)177, (char)35, (char)25}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)37, (char)117, (char)153, (char)46, (char)132, (char)57, (char)217, (char)249, (char)23, (char)126, (char)57, (char)67, (char)137, (char)101, (char)154, (char)240, (char)59, (char)216, (char)49, (char)31}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_used_SET(new char[] {(char)142, (char)18, (char)182, (char)53, (char)128, (char)136, (char)85, (char)98, (char)2, (char)89, (char)187, (char)106, (char)201, (char)106, (char)226, (char)150, (char)165, (char)54, (char)241, (char)20}, 0) ;
        p25.satellites_visible_SET((char)176) ;
        p25.satellite_snr_SET(new char[] {(char)37, (char)117, (char)153, (char)46, (char)132, (char)57, (char)217, (char)249, (char)23, (char)126, (char)57, (char)67, (char)137, (char)101, (char)154, (char)240, (char)59, (char)216, (char)49, (char)31}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)11, (char)61, (char)36, (char)227, (char)64, (char)76, (char)61, (char)45, (char)56, (char)30, (char)202, (char)119, (char)220, (char)33, (char)219, (char)152, (char)87, (char)243, (char)148, (char)101}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)37, (char)246, (char)101, (char)131, (char)185, (char)190, (char)25, (char)108, (char)132, (char)214, (char)20, (char)14, (char)235, (char)140, (char)105, (char)115, (char)39, (char)112, (char)38, (char)31}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)16, (char)156, (char)77, (char)37, (char)173, (char)221, (char)93, (char)35, (char)36, (char)93, (char)222, (char)196, (char)228, (char)25, (char)101, (char)174, (char)58, (char)177, (char)35, (char)25}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3461386403L);
            assert(pack.zgyro_GET() == (short)32223);
            assert(pack.xgyro_GET() == (short)23102);
            assert(pack.zmag_GET() == (short)8362);
            assert(pack.ygyro_GET() == (short) -13122);
            assert(pack.yacc_GET() == (short)7372);
            assert(pack.xacc_GET() == (short) -13284);
            assert(pack.zacc_GET() == (short) -11719);
            assert(pack.ymag_GET() == (short)23641);
            assert(pack.xmag_GET() == (short) -21119);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.ygyro_SET((short) -13122) ;
        p26.ymag_SET((short)23641) ;
        p26.time_boot_ms_SET(3461386403L) ;
        p26.yacc_SET((short)7372) ;
        p26.zmag_SET((short)8362) ;
        p26.xmag_SET((short) -21119) ;
        p26.xacc_SET((short) -13284) ;
        p26.zgyro_SET((short)32223) ;
        p26.zacc_SET((short) -11719) ;
        p26.xgyro_SET((short)23102) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short) -29774);
            assert(pack.xmag_GET() == (short)27448);
            assert(pack.yacc_GET() == (short)10188);
            assert(pack.time_usec_GET() == 2476796271378940254L);
            assert(pack.ymag_GET() == (short)4972);
            assert(pack.xacc_GET() == (short)32745);
            assert(pack.zmag_GET() == (short) -15516);
            assert(pack.zgyro_GET() == (short)6245);
            assert(pack.xgyro_GET() == (short)25179);
            assert(pack.zacc_GET() == (short)950);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.zgyro_SET((short)6245) ;
        p27.xacc_SET((short)32745) ;
        p27.xmag_SET((short)27448) ;
        p27.ygyro_SET((short) -29774) ;
        p27.xgyro_SET((short)25179) ;
        p27.ymag_SET((short)4972) ;
        p27.yacc_SET((short)10188) ;
        p27.zmag_SET((short) -15516) ;
        p27.time_usec_SET(2476796271378940254L) ;
        p27.zacc_SET((short)950) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff1_GET() == (short) -25978);
            assert(pack.temperature_GET() == (short) -32740);
            assert(pack.press_diff2_GET() == (short) -15955);
            assert(pack.time_usec_GET() == 3183142677771297132L);
            assert(pack.press_abs_GET() == (short)31974);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.temperature_SET((short) -32740) ;
        p28.time_usec_SET(3183142677771297132L) ;
        p28.press_diff2_SET((short) -15955) ;
        p28.press_diff1_SET((short) -25978) ;
        p28.press_abs_SET((short)31974) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == -1.5067735E38F);
            assert(pack.press_abs_GET() == 2.7633704E38F);
            assert(pack.temperature_GET() == (short)9000);
            assert(pack.time_boot_ms_GET() == 3403807812L);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_diff_SET(-1.5067735E38F) ;
        p29.temperature_SET((short)9000) ;
        p29.press_abs_SET(2.7633704E38F) ;
        p29.time_boot_ms_SET(3403807812L) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == -2.0593091E38F);
            assert(pack.pitch_GET() == 2.7141132E38F);
            assert(pack.rollspeed_GET() == 2.0173323E38F);
            assert(pack.roll_GET() == -2.6244026E38F);
            assert(pack.yaw_GET() == 2.877523E38F);
            assert(pack.time_boot_ms_GET() == 2686827188L);
            assert(pack.pitchspeed_GET() == -2.7770485E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.pitch_SET(2.7141132E38F) ;
        p30.roll_SET(-2.6244026E38F) ;
        p30.pitchspeed_SET(-2.7770485E38F) ;
        p30.yawspeed_SET(-2.0593091E38F) ;
        p30.time_boot_ms_SET(2686827188L) ;
        p30.rollspeed_SET(2.0173323E38F) ;
        p30.yaw_SET(2.877523E38F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 1.9715238E38F);
            assert(pack.pitchspeed_GET() == 2.6637467E38F);
            assert(pack.rollspeed_GET() == -2.9022903E38F);
            assert(pack.q2_GET() == 2.2790478E38F);
            assert(pack.q3_GET() == -2.8224326E38F);
            assert(pack.time_boot_ms_GET() == 2936761692L);
            assert(pack.q4_GET() == 2.7634765E38F);
            assert(pack.q1_GET() == -5.108628E37F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q3_SET(-2.8224326E38F) ;
        p31.q1_SET(-5.108628E37F) ;
        p31.yawspeed_SET(1.9715238E38F) ;
        p31.rollspeed_SET(-2.9022903E38F) ;
        p31.pitchspeed_SET(2.6637467E38F) ;
        p31.q4_SET(2.7634765E38F) ;
        p31.time_boot_ms_SET(2936761692L) ;
        p31.q2_SET(2.2790478E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == 1.1884174E38F);
            assert(pack.time_boot_ms_GET() == 1249864508L);
            assert(pack.vx_GET() == 9.122547E36F);
            assert(pack.z_GET() == -2.5245199E38F);
            assert(pack.x_GET() == -1.7458135E38F);
            assert(pack.vy_GET() == -9.68491E37F);
            assert(pack.y_GET() == -9.352311E37F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.time_boot_ms_SET(1249864508L) ;
        p32.vx_SET(9.122547E36F) ;
        p32.x_SET(-1.7458135E38F) ;
        p32.y_SET(-9.352311E37F) ;
        p32.z_SET(-2.5245199E38F) ;
        p32.vz_SET(1.1884174E38F) ;
        p32.vy_SET(-9.68491E37F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.hdg_GET() == (char)59586);
            assert(pack.relative_alt_GET() == 1647227884);
            assert(pack.vy_GET() == (short)20504);
            assert(pack.lon_GET() == 1074353135);
            assert(pack.vx_GET() == (short) -7465);
            assert(pack.alt_GET() == 1456251296);
            assert(pack.time_boot_ms_GET() == 2909774818L);
            assert(pack.vz_GET() == (short)21572);
            assert(pack.lat_GET() == -1903467264);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.lon_SET(1074353135) ;
        p33.vz_SET((short)21572) ;
        p33.time_boot_ms_SET(2909774818L) ;
        p33.hdg_SET((char)59586) ;
        p33.lat_SET(-1903467264) ;
        p33.vx_SET((short) -7465) ;
        p33.relative_alt_SET(1647227884) ;
        p33.alt_SET(1456251296) ;
        p33.vy_SET((short)20504) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan1_scaled_GET() == (short) -1387);
            assert(pack.chan6_scaled_GET() == (short) -16558);
            assert(pack.chan4_scaled_GET() == (short)16733);
            assert(pack.rssi_GET() == (char)176);
            assert(pack.chan7_scaled_GET() == (short)4519);
            assert(pack.chan3_scaled_GET() == (short)27882);
            assert(pack.chan8_scaled_GET() == (short) -29709);
            assert(pack.chan2_scaled_GET() == (short)6947);
            assert(pack.chan5_scaled_GET() == (short)27671);
            assert(pack.port_GET() == (char)221);
            assert(pack.time_boot_ms_GET() == 137624753L);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan3_scaled_SET((short)27882) ;
        p34.chan6_scaled_SET((short) -16558) ;
        p34.port_SET((char)221) ;
        p34.chan2_scaled_SET((short)6947) ;
        p34.time_boot_ms_SET(137624753L) ;
        p34.rssi_SET((char)176) ;
        p34.chan7_scaled_SET((short)4519) ;
        p34.chan4_scaled_SET((short)16733) ;
        p34.chan8_scaled_SET((short) -29709) ;
        p34.chan5_scaled_SET((short)27671) ;
        p34.chan1_scaled_SET((short) -1387) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan8_raw_GET() == (char)60394);
            assert(pack.chan2_raw_GET() == (char)27013);
            assert(pack.chan5_raw_GET() == (char)44967);
            assert(pack.chan7_raw_GET() == (char)3340);
            assert(pack.rssi_GET() == (char)155);
            assert(pack.time_boot_ms_GET() == 1407712405L);
            assert(pack.chan4_raw_GET() == (char)3699);
            assert(pack.chan1_raw_GET() == (char)53611);
            assert(pack.port_GET() == (char)237);
            assert(pack.chan6_raw_GET() == (char)45227);
            assert(pack.chan3_raw_GET() == (char)49703);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan6_raw_SET((char)45227) ;
        p35.time_boot_ms_SET(1407712405L) ;
        p35.chan2_raw_SET((char)27013) ;
        p35.chan8_raw_SET((char)60394) ;
        p35.port_SET((char)237) ;
        p35.chan7_raw_SET((char)3340) ;
        p35.chan3_raw_SET((char)49703) ;
        p35.chan4_raw_SET((char)3699) ;
        p35.chan1_raw_SET((char)53611) ;
        p35.chan5_raw_SET((char)44967) ;
        p35.rssi_SET((char)155) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo15_raw_TRY(ph) == (char)34506);
            assert(pack.servo14_raw_TRY(ph) == (char)45221);
            assert(pack.time_usec_GET() == 4064470229L);
            assert(pack.port_GET() == (char)40);
            assert(pack.servo11_raw_TRY(ph) == (char)39915);
            assert(pack.servo4_raw_GET() == (char)44891);
            assert(pack.servo13_raw_TRY(ph) == (char)42942);
            assert(pack.servo2_raw_GET() == (char)48426);
            assert(pack.servo3_raw_GET() == (char)13892);
            assert(pack.servo5_raw_GET() == (char)22312);
            assert(pack.servo12_raw_TRY(ph) == (char)7105);
            assert(pack.servo6_raw_GET() == (char)57841);
            assert(pack.servo10_raw_TRY(ph) == (char)10527);
            assert(pack.servo1_raw_GET() == (char)63826);
            assert(pack.servo16_raw_TRY(ph) == (char)18427);
            assert(pack.servo9_raw_TRY(ph) == (char)59631);
            assert(pack.servo7_raw_GET() == (char)59782);
            assert(pack.servo8_raw_GET() == (char)35903);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo13_raw_SET((char)42942, PH) ;
        p36.servo2_raw_SET((char)48426) ;
        p36.servo16_raw_SET((char)18427, PH) ;
        p36.servo6_raw_SET((char)57841) ;
        p36.servo8_raw_SET((char)35903) ;
        p36.servo9_raw_SET((char)59631, PH) ;
        p36.servo4_raw_SET((char)44891) ;
        p36.servo14_raw_SET((char)45221, PH) ;
        p36.servo1_raw_SET((char)63826) ;
        p36.servo5_raw_SET((char)22312) ;
        p36.time_usec_SET(4064470229L) ;
        p36.servo11_raw_SET((char)39915, PH) ;
        p36.servo15_raw_SET((char)34506, PH) ;
        p36.servo7_raw_SET((char)59782) ;
        p36.servo12_raw_SET((char)7105, PH) ;
        p36.port_SET((char)40) ;
        p36.servo3_raw_SET((char)13892) ;
        p36.servo10_raw_SET((char)10527, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)122);
            assert(pack.start_index_GET() == (short)26691);
            assert(pack.end_index_GET() == (short) -19742);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)142);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_component_SET((char)142) ;
        p37.end_index_SET((short) -19742) ;
        p37.start_index_SET((short)26691) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p37.target_system_SET((char)122) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)152);
            assert(pack.target_component_GET() == (char)34);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.end_index_GET() == (short) -3948);
            assert(pack.start_index_GET() == (short) -14519);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.end_index_SET((short) -3948) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p38.start_index_SET((short) -14519) ;
        p38.target_system_SET((char)152) ;
        p38.target_component_SET((char)34) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 2.7112345E38F);
            assert(pack.target_component_GET() == (char)138);
            assert(pack.y_GET() == -1.8724692E38F);
            assert(pack.target_system_GET() == (char)12);
            assert(pack.x_GET() == 2.5721328E38F);
            assert(pack.current_GET() == (char)184);
            assert(pack.param2_GET() == -7.444173E37F);
            assert(pack.param1_GET() == 5.7767675E37F);
            assert(pack.autocontinue_GET() == (char)128);
            assert(pack.seq_GET() == (char)27770);
            assert(pack.param3_GET() == 1.9974176E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.param4_GET() == -3.3153438E38F);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.param1_SET(5.7767675E37F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST) ;
        p39.autocontinue_SET((char)128) ;
        p39.current_SET((char)184) ;
        p39.y_SET(-1.8724692E38F) ;
        p39.seq_SET((char)27770) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p39.param3_SET(1.9974176E38F) ;
        p39.z_SET(2.7112345E38F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p39.param2_SET(-7.444173E37F) ;
        p39.param4_SET(-3.3153438E38F) ;
        p39.target_component_SET((char)138) ;
        p39.x_SET(2.5721328E38F) ;
        p39.target_system_SET((char)12) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)58314);
            assert(pack.target_component_GET() == (char)205);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)52);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.seq_SET((char)58314) ;
        p40.target_component_SET((char)205) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p40.target_system_SET((char)52) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)32);
            assert(pack.target_component_GET() == (char)133);
            assert(pack.seq_GET() == (char)42280);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)32) ;
        p41.target_component_SET((char)133) ;
        p41.seq_SET((char)42280) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)16539);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)16539) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)13);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)118);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)118) ;
        p43.target_component_SET((char)13) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)130);
            assert(pack.target_system_GET() == (char)132);
            assert(pack.count_GET() == (char)15491);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_component_SET((char)130) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p44.target_system_SET((char)132) ;
        p44.count_SET((char)15491) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)95);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)201);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)95) ;
        p45.target_component_SET((char)201) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)27960);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)27960) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED_FRAME);
            assert(pack.target_component_GET() == (char)140);
            assert(pack.target_system_GET() == (char)99);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED_FRAME) ;
        p47.target_system_SET((char)99) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p47.target_component_SET((char)140) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 2513742839858188355L);
            assert(pack.longitude_GET() == -1688688487);
            assert(pack.altitude_GET() == -901137482);
            assert(pack.target_system_GET() == (char)246);
            assert(pack.latitude_GET() == 1761153081);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.latitude_SET(1761153081) ;
        p48.time_usec_SET(2513742839858188355L, PH) ;
        p48.longitude_SET(-1688688487) ;
        p48.target_system_SET((char)246) ;
        p48.altitude_SET(-901137482) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == -757274998);
            assert(pack.time_usec_TRY(ph) == 5957820634802183089L);
            assert(pack.altitude_GET() == -118017528);
            assert(pack.latitude_GET() == -1721196818);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.longitude_SET(-757274998) ;
        p49.latitude_SET(-1721196818) ;
        p49.altitude_SET(-118017528) ;
        p49.time_usec_SET(5957820634802183089L, PH) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)133);
            assert(pack.parameter_rc_channel_index_GET() == (char)175);
            assert(pack.param_value_min_GET() == -1.1355999E38F);
            assert(pack.param_value_max_GET() == 1.9309629E38F);
            assert(pack.param_value0_GET() == -1.6860926E38F);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("hubcwbgd"));
            assert(pack.param_index_GET() == (short) -31196);
            assert(pack.target_component_GET() == (char)78);
            assert(pack.scale_GET() == 3.1571501E38F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.param_value_max_SET(1.9309629E38F) ;
        p50.param_id_SET("hubcwbgd", PH) ;
        p50.param_value0_SET(-1.6860926E38F) ;
        p50.target_component_SET((char)78) ;
        p50.target_system_SET((char)133) ;
        p50.parameter_rc_channel_index_SET((char)175) ;
        p50.param_index_SET((short) -31196) ;
        p50.param_value_min_SET(-1.1355999E38F) ;
        p50.scale_SET(3.1571501E38F) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)60339);
            assert(pack.target_component_GET() == (char)206);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)63);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p51.seq_SET((char)60339) ;
        p51.target_component_SET((char)206) ;
        p51.target_system_SET((char)63) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.p2z_GET() == -8.606757E37F);
            assert(pack.p1z_GET() == -3.0794833E38F);
            assert(pack.p1x_GET() == 2.2431541E37F);
            assert(pack.p2y_GET() == -2.846305E38F);
            assert(pack.target_system_GET() == (char)164);
            assert(pack.p2x_GET() == 1.6568496E38F);
            assert(pack.p1y_GET() == -6.2601465E37F);
            assert(pack.target_component_GET() == (char)42);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p2z_SET(-8.606757E37F) ;
        p54.p1z_SET(-3.0794833E38F) ;
        p54.p2x_SET(1.6568496E38F) ;
        p54.p1x_SET(2.2431541E37F) ;
        p54.p1y_SET(-6.2601465E37F) ;
        p54.p2y_SET(-2.846305E38F) ;
        p54.target_component_SET((char)42) ;
        p54.target_system_SET((char)164) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1y_GET() == -3.3706875E37F);
            assert(pack.p1x_GET() == 2.7571226E37F);
            assert(pack.p1z_GET() == 2.2908927E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.p2x_GET() == -2.2593004E38F);
            assert(pack.p2z_GET() == 1.615149E38F);
            assert(pack.p2y_GET() == 3.99977E37F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2x_SET(-2.2593004E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p55.p1x_SET(2.7571226E37F) ;
        p55.p1y_SET(-3.3706875E37F) ;
        p55.p2z_SET(1.615149E38F) ;
        p55.p1z_SET(2.2908927E38F) ;
        p55.p2y_SET(3.99977E37F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == 1.9809856E38F);
            assert(pack.pitchspeed_GET() == 5.7489704E37F);
            assert(pack.time_usec_GET() == 3079143292651727490L);
            assert(pack.yawspeed_GET() == 3.2838325E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.1957065E38F, 3.071863E38F, 1.781535E38F, 1.7548923E38F}));
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.6368215E37F, 2.8049092E38F, 1.2882377E38F, -2.9781427E38F, -2.3332369E38F, 3.9837155E37F, -2.45282E38F, -2.460799E38F, -2.4276126E38F}));
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.q_SET(new float[] {1.1957065E38F, 3.071863E38F, 1.781535E38F, 1.7548923E38F}, 0) ;
        p61.pitchspeed_SET(5.7489704E37F) ;
        p61.time_usec_SET(3079143292651727490L) ;
        p61.yawspeed_SET(3.2838325E38F) ;
        p61.rollspeed_SET(1.9809856E38F) ;
        p61.covariance_SET(new float[] {-2.6368215E37F, 2.8049092E38F, 1.2882377E38F, -2.9781427E38F, -2.3332369E38F, 3.9837155E37F, -2.45282E38F, -2.460799E38F, -2.4276126E38F}, 0) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_bearing_GET() == (short)10647);
            assert(pack.nav_roll_GET() == -2.8770257E38F);
            assert(pack.aspd_error_GET() == 1.2180923E38F);
            assert(pack.wp_dist_GET() == (char)43999);
            assert(pack.xtrack_error_GET() == -1.1897405E38F);
            assert(pack.nav_pitch_GET() == 2.099815E38F);
            assert(pack.alt_error_GET() == 1.3881499E38F);
            assert(pack.target_bearing_GET() == (short) -3667);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.xtrack_error_SET(-1.1897405E38F) ;
        p62.nav_pitch_SET(2.099815E38F) ;
        p62.nav_bearing_SET((short)10647) ;
        p62.wp_dist_SET((char)43999) ;
        p62.nav_roll_SET(-2.8770257E38F) ;
        p62.target_bearing_SET((short) -3667) ;
        p62.aspd_error_SET(1.2180923E38F) ;
        p62.alt_error_SET(1.3881499E38F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.0790147E38F, -2.0546105E38F, -1.5562989E38F, -3.1583302E38F, 5.27769E37F, 1.2003029E38F, -2.1745163E38F, -1.0607708E38F, 5.2938063E37F, 5.438033E37F, 6.6422934E37F, -8.862434E36F, 7.364815E37F, 3.7155404E37F, 1.587158E38F, 3.0519152E37F, 1.5022364E38F, 6.325295E36F, 2.5245955E38F, -1.3330039E38F, 2.734946E38F, 2.5894218E38F, 1.2953505E38F, 3.0453466E38F, -1.5576832E37F, -3.1933979E38F, 1.1545475E38F, -3.3975204E38F, -7.349843E37F, 8.748824E36F, -2.3462963E38F, -3.3240955E38F, 3.0068374E38F, 6.730905E37F, 1.3982613E38F, -1.9147155E37F}));
            assert(pack.vz_GET() == 3.169164E38F);
            assert(pack.time_usec_GET() == 6326925911365885753L);
            assert(pack.vx_GET() == -1.9622678E38F);
            assert(pack.vy_GET() == -3.4438827E37F);
            assert(pack.lat_GET() == -792283881);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
            assert(pack.lon_GET() == 1210698968);
            assert(pack.alt_GET() == 1333410813);
            assert(pack.relative_alt_GET() == -193313783);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.vx_SET(-1.9622678E38F) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS) ;
        p63.lat_SET(-792283881) ;
        p63.alt_SET(1333410813) ;
        p63.lon_SET(1210698968) ;
        p63.vz_SET(3.169164E38F) ;
        p63.vy_SET(-3.4438827E37F) ;
        p63.covariance_SET(new float[] {2.0790147E38F, -2.0546105E38F, -1.5562989E38F, -3.1583302E38F, 5.27769E37F, 1.2003029E38F, -2.1745163E38F, -1.0607708E38F, 5.2938063E37F, 5.438033E37F, 6.6422934E37F, -8.862434E36F, 7.364815E37F, 3.7155404E37F, 1.587158E38F, 3.0519152E37F, 1.5022364E38F, 6.325295E36F, 2.5245955E38F, -1.3330039E38F, 2.734946E38F, 2.5894218E38F, 1.2953505E38F, 3.0453466E38F, -1.5576832E37F, -3.1933979E38F, 1.1545475E38F, -3.3975204E38F, -7.349843E37F, 8.748824E36F, -2.3462963E38F, -3.3240955E38F, 3.0068374E38F, 6.730905E37F, 1.3982613E38F, -1.9147155E37F}, 0) ;
        p63.relative_alt_SET(-193313783) ;
        p63.time_usec_SET(6326925911365885753L) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 1.0391264E38F);
            assert(pack.y_GET() == 1.6175163E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.3816605E38F, -1.5104086E38F, 1.7530014E37F, 3.475434E36F, -2.8702782E38F, 2.6450658E37F, -3.2658482E37F, -4.953861E37F, -1.4888785E38F, 3.2985989E38F, -1.9336754E38F, -1.5434052E38F, -5.8794294E37F, 2.5262676E38F, 3.3260807E38F, -2.3529405E37F, -1.1451824E37F, -1.0133177E38F, 7.3197305E37F, -3.070289E38F, -1.608449E38F, 2.823654E38F, -3.0046842E38F, -9.026328E37F, -2.200317E38F, -1.9603878E38F, 2.4987227E38F, 6.647485E36F, 4.7530483E37F, 1.6145817E38F, 2.8863715E38F, 1.8066761E38F, 5.7159856E37F, -3.4228105E37F, 1.1808255E38F, -3.1503225E38F, 1.0630935E38F, 3.0483095E38F, -3.1678154E38F, 3.984613E37F, 1.4468673E38F, 1.6615004E38F, 6.403443E37F, -2.530666E38F, 3.3888087E38F}));
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
            assert(pack.time_usec_GET() == 1437243516974687503L);
            assert(pack.vy_GET() == -2.10325E38F);
            assert(pack.vz_GET() == -1.2029842E38F);
            assert(pack.vx_GET() == 2.3349654E38F);
            assert(pack.ay_GET() == 5.56126E37F);
            assert(pack.ax_GET() == -1.3020851E38F);
            assert(pack.az_GET() == 1.733058E38F);
            assert(pack.x_GET() == 1.403527E37F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.vx_SET(2.3349654E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS) ;
        p64.ay_SET(5.56126E37F) ;
        p64.ax_SET(-1.3020851E38F) ;
        p64.covariance_SET(new float[] {-2.3816605E38F, -1.5104086E38F, 1.7530014E37F, 3.475434E36F, -2.8702782E38F, 2.6450658E37F, -3.2658482E37F, -4.953861E37F, -1.4888785E38F, 3.2985989E38F, -1.9336754E38F, -1.5434052E38F, -5.8794294E37F, 2.5262676E38F, 3.3260807E38F, -2.3529405E37F, -1.1451824E37F, -1.0133177E38F, 7.3197305E37F, -3.070289E38F, -1.608449E38F, 2.823654E38F, -3.0046842E38F, -9.026328E37F, -2.200317E38F, -1.9603878E38F, 2.4987227E38F, 6.647485E36F, 4.7530483E37F, 1.6145817E38F, 2.8863715E38F, 1.8066761E38F, 5.7159856E37F, -3.4228105E37F, 1.1808255E38F, -3.1503225E38F, 1.0630935E38F, 3.0483095E38F, -3.1678154E38F, 3.984613E37F, 1.4468673E38F, 1.6615004E38F, 6.403443E37F, -2.530666E38F, 3.3888087E38F}, 0) ;
        p64.x_SET(1.403527E37F) ;
        p64.z_SET(1.0391264E38F) ;
        p64.vy_SET(-2.10325E38F) ;
        p64.az_SET(1.733058E38F) ;
        p64.y_SET(1.6175163E38F) ;
        p64.time_usec_SET(1437243516974687503L) ;
        p64.vz_SET(-1.2029842E38F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)13419);
            assert(pack.time_boot_ms_GET() == 292512815L);
            assert(pack.chan1_raw_GET() == (char)42998);
            assert(pack.chan18_raw_GET() == (char)36154);
            assert(pack.chan10_raw_GET() == (char)28603);
            assert(pack.chan13_raw_GET() == (char)32570);
            assert(pack.chan15_raw_GET() == (char)50939);
            assert(pack.chan3_raw_GET() == (char)11077);
            assert(pack.rssi_GET() == (char)231);
            assert(pack.chan16_raw_GET() == (char)12105);
            assert(pack.chan11_raw_GET() == (char)10277);
            assert(pack.chancount_GET() == (char)51);
            assert(pack.chan14_raw_GET() == (char)8973);
            assert(pack.chan6_raw_GET() == (char)17425);
            assert(pack.chan5_raw_GET() == (char)9796);
            assert(pack.chan9_raw_GET() == (char)16911);
            assert(pack.chan8_raw_GET() == (char)9079);
            assert(pack.chan12_raw_GET() == (char)33267);
            assert(pack.chan2_raw_GET() == (char)63165);
            assert(pack.chan17_raw_GET() == (char)7564);
            assert(pack.chan7_raw_GET() == (char)47398);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan17_raw_SET((char)7564) ;
        p65.chan11_raw_SET((char)10277) ;
        p65.chan1_raw_SET((char)42998) ;
        p65.chan10_raw_SET((char)28603) ;
        p65.chan15_raw_SET((char)50939) ;
        p65.chan13_raw_SET((char)32570) ;
        p65.time_boot_ms_SET(292512815L) ;
        p65.chan7_raw_SET((char)47398) ;
        p65.chan5_raw_SET((char)9796) ;
        p65.rssi_SET((char)231) ;
        p65.chan6_raw_SET((char)17425) ;
        p65.chan14_raw_SET((char)8973) ;
        p65.chan9_raw_SET((char)16911) ;
        p65.chan2_raw_SET((char)63165) ;
        p65.chan3_raw_SET((char)11077) ;
        p65.chan18_raw_SET((char)36154) ;
        p65.chan12_raw_SET((char)33267) ;
        p65.chan4_raw_SET((char)13419) ;
        p65.chan8_raw_SET((char)9079) ;
        p65.chan16_raw_SET((char)12105) ;
        p65.chancount_SET((char)51) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)18);
            assert(pack.req_stream_id_GET() == (char)242);
            assert(pack.req_message_rate_GET() == (char)27005);
            assert(pack.start_stop_GET() == (char)135);
            assert(pack.target_system_GET() == (char)147);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_component_SET((char)18) ;
        p66.req_message_rate_SET((char)27005) ;
        p66.req_stream_id_SET((char)242) ;
        p66.target_system_SET((char)147) ;
        p66.start_stop_SET((char)135) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.on_off_GET() == (char)222);
            assert(pack.stream_id_GET() == (char)175);
            assert(pack.message_rate_GET() == (char)56579);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)56579) ;
        p67.on_off_SET((char)222) ;
        p67.stream_id_SET((char)175) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == (short)19524);
            assert(pack.r_GET() == (short) -7534);
            assert(pack.target_GET() == (char)229);
            assert(pack.x_GET() == (short)15917);
            assert(pack.buttons_GET() == (char)27131);
            assert(pack.z_GET() == (short) -4940);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.y_SET((short)19524) ;
        p69.target_SET((char)229) ;
        p69.buttons_SET((char)27131) ;
        p69.x_SET((short)15917) ;
        p69.r_SET((short) -7534) ;
        p69.z_SET((short) -4940) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan1_raw_GET() == (char)39818);
            assert(pack.chan7_raw_GET() == (char)15624);
            assert(pack.chan3_raw_GET() == (char)34832);
            assert(pack.target_component_GET() == (char)48);
            assert(pack.chan8_raw_GET() == (char)50636);
            assert(pack.chan4_raw_GET() == (char)47760);
            assert(pack.chan2_raw_GET() == (char)60593);
            assert(pack.chan6_raw_GET() == (char)57612);
            assert(pack.target_system_GET() == (char)242);
            assert(pack.chan5_raw_GET() == (char)20069);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.target_system_SET((char)242) ;
        p70.chan2_raw_SET((char)60593) ;
        p70.chan1_raw_SET((char)39818) ;
        p70.chan7_raw_SET((char)15624) ;
        p70.chan4_raw_SET((char)47760) ;
        p70.chan6_raw_SET((char)57612) ;
        p70.target_component_SET((char)48) ;
        p70.chan8_raw_SET((char)50636) ;
        p70.chan5_raw_SET((char)20069) ;
        p70.chan3_raw_SET((char)34832) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 530229174);
            assert(pack.seq_GET() == (char)18146);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_SERVO);
            assert(pack.param2_GET() == 7.9302913E37F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.param4_GET() == -3.3382992E38F);
            assert(pack.target_component_GET() == (char)227);
            assert(pack.autocontinue_GET() == (char)138);
            assert(pack.target_system_GET() == (char)232);
            assert(pack.z_GET() == -6.4500755E37F);
            assert(pack.param1_GET() == -1.1478976E38F);
            assert(pack.param3_GET() == -2.7003246E38F);
            assert(pack.current_GET() == (char)197);
            assert(pack.y_GET() == 1805857859);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.z_SET(-6.4500755E37F) ;
        p73.param2_SET(7.9302913E37F) ;
        p73.current_SET((char)197) ;
        p73.param4_SET(-3.3382992E38F) ;
        p73.autocontinue_SET((char)138) ;
        p73.param1_SET(-1.1478976E38F) ;
        p73.y_SET(1805857859) ;
        p73.x_SET(530229174) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p73.command_SET(MAV_CMD.MAV_CMD_DO_SET_SERVO) ;
        p73.param3_SET(-2.7003246E38F) ;
        p73.target_system_SET((char)232) ;
        p73.seq_SET((char)18146) ;
        p73.target_component_SET((char)227) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.climb_GET() == 1.8351135E38F);
            assert(pack.alt_GET() == -7.182042E37F);
            assert(pack.heading_GET() == (short)24859);
            assert(pack.airspeed_GET() == 1.0757481E38F);
            assert(pack.throttle_GET() == (char)32124);
            assert(pack.groundspeed_GET() == 1.5640127E38F);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.alt_SET(-7.182042E37F) ;
        p74.climb_SET(1.8351135E38F) ;
        p74.heading_SET((short)24859) ;
        p74.groundspeed_SET(1.5640127E38F) ;
        p74.throttle_SET((char)32124) ;
        p74.airspeed_SET(1.0757481E38F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.5631595E38F);
            assert(pack.param2_GET() == 3.2285704E38F);
            assert(pack.target_component_GET() == (char)199);
            assert(pack.target_system_GET() == (char)54);
            assert(pack.param3_GET() == 2.4645233E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.current_GET() == (char)45);
            assert(pack.autocontinue_GET() == (char)45);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS);
            assert(pack.x_GET() == -34868255);
            assert(pack.y_GET() == -96677947);
            assert(pack.param4_GET() == -2.1888993E38F);
            assert(pack.param1_GET() == 1.9570953E38F);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.param1_SET(1.9570953E38F) ;
        p75.param2_SET(3.2285704E38F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p75.y_SET(-96677947) ;
        p75.current_SET((char)45) ;
        p75.autocontinue_SET((char)45) ;
        p75.command_SET(MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS) ;
        p75.z_SET(-1.5631595E38F) ;
        p75.target_component_SET((char)199) ;
        p75.target_system_SET((char)54) ;
        p75.param4_SET(-2.1888993E38F) ;
        p75.x_SET(-34868255) ;
        p75.param3_SET(2.4645233E38F) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param2_GET() == -1.9930508E38F);
            assert(pack.param6_GET() == -7.5342656E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH);
            assert(pack.param1_GET() == 2.8977813E38F);
            assert(pack.param7_GET() == 3.031796E37F);
            assert(pack.param5_GET() == -3.1562944E38F);
            assert(pack.param3_GET() == -1.0837075E38F);
            assert(pack.target_component_GET() == (char)107);
            assert(pack.confirmation_GET() == (char)177);
            assert(pack.target_system_GET() == (char)1);
            assert(pack.param4_GET() == -1.1739806E38F);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.target_component_SET((char)107) ;
        p76.param4_SET(-1.1739806E38F) ;
        p76.param5_SET(-3.1562944E38F) ;
        p76.param1_SET(2.8977813E38F) ;
        p76.param6_SET(-7.5342656E37F) ;
        p76.confirmation_SET((char)177) ;
        p76.command_SET(MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH) ;
        p76.param3_SET(-1.0837075E38F) ;
        p76.param2_SET(-1.9930508E38F) ;
        p76.param7_SET(3.031796E37F) ;
        p76.target_system_SET((char)1) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.result_param2_TRY(ph) == 1874157447);
            assert(pack.target_component_TRY(ph) == (char)118);
            assert(pack.progress_TRY(ph) == (char)21);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_USER_2);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_ACCEPTED);
            assert(pack.target_system_TRY(ph) == (char)30);
        });
        COMMAND_ACK p77 = new COMMAND_ACK();
        PH.setPack(p77);
        p77.target_component_SET((char)118, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_USER_2) ;
        p77.result_param2_SET(1874157447, PH) ;
        p77.target_system_SET((char)30, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_ACCEPTED) ;
        p77.progress_SET((char)21, PH) ;
        TestChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -9.18366E37F);
            assert(pack.yaw_GET() == -2.4466746E38F);
            assert(pack.mode_switch_GET() == (char)19);
            assert(pack.manual_override_switch_GET() == (char)39);
            assert(pack.thrust_GET() == 2.602742E38F);
            assert(pack.time_boot_ms_GET() == 525009428L);
            assert(pack.pitch_GET() == 1.7663249E38F);
        });
        MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.yaw_SET(-2.4466746E38F) ;
        p81.manual_override_switch_SET((char)39) ;
        p81.time_boot_ms_SET(525009428L) ;
        p81.pitch_SET(1.7663249E38F) ;
        p81.mode_switch_SET((char)19) ;
        p81.thrust_SET(2.602742E38F) ;
        p81.roll_SET(-9.18366E37F) ;
        TestChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1480562047L);
            assert(pack.body_roll_rate_GET() == 1.6466837E38F);
            assert(pack.body_yaw_rate_GET() == 1.4088849E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.0328297E38F, 3.0052765E38F, -3.3551148E38F, -5.224989E37F}));
            assert(pack.body_pitch_rate_GET() == 4.149558E37F);
            assert(pack.target_system_GET() == (char)201);
            assert(pack.type_mask_GET() == (char)102);
            assert(pack.thrust_GET() == 2.9194085E38F);
            assert(pack.target_component_GET() == (char)219);
        });
        SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.target_component_SET((char)219) ;
        p82.time_boot_ms_SET(1480562047L) ;
        p82.q_SET(new float[] {3.0328297E38F, 3.0052765E38F, -3.3551148E38F, -5.224989E37F}, 0) ;
        p82.body_yaw_rate_SET(1.4088849E38F) ;
        p82.thrust_SET(2.9194085E38F) ;
        p82.body_roll_rate_SET(1.6466837E38F) ;
        p82.target_system_SET((char)201) ;
        p82.type_mask_SET((char)102) ;
        p82.body_pitch_rate_SET(4.149558E37F) ;
        TestChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_pitch_rate_GET() == -2.5402568E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.1437675E38F, -1.5388308E38F, 1.3904913E38F, -3.781234E37F}));
            assert(pack.thrust_GET() == 2.9468735E38F);
            assert(pack.body_roll_rate_GET() == 8.224542E37F);
            assert(pack.time_boot_ms_GET() == 1366020712L);
            assert(pack.body_yaw_rate_GET() == 1.8448156E38F);
            assert(pack.type_mask_GET() == (char)121);
        });
        ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.time_boot_ms_SET(1366020712L) ;
        p83.q_SET(new float[] {-2.1437675E38F, -1.5388308E38F, 1.3904913E38F, -3.781234E37F}, 0) ;
        p83.body_roll_rate_SET(8.224542E37F) ;
        p83.body_pitch_rate_SET(-2.5402568E38F) ;
        p83.body_yaw_rate_SET(1.8448156E38F) ;
        p83.type_mask_SET((char)121) ;
        p83.thrust_SET(2.9468735E38F) ;
        TestChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)109);
            assert(pack.afz_GET() == 2.6312058E38F);
            assert(pack.vy_GET() == -2.1407463E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.yaw_rate_GET() == 2.0275944E38F);
            assert(pack.yaw_GET() == -1.0615192E37F);
            assert(pack.vx_GET() == 5.2954106E37F);
            assert(pack.afy_GET() == -2.26474E38F);
            assert(pack.afx_GET() == 1.217639E38F);
            assert(pack.target_system_GET() == (char)41);
            assert(pack.vz_GET() == -2.9143082E38F);
            assert(pack.z_GET() == 3.2340207E38F);
            assert(pack.y_GET() == -1.1896177E38F);
            assert(pack.x_GET() == 3.367726E38F);
            assert(pack.type_mask_GET() == (char)33769);
            assert(pack.time_boot_ms_GET() == 2919206022L);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.yaw_rate_SET(2.0275944E38F) ;
        p84.afz_SET(2.6312058E38F) ;
        p84.yaw_SET(-1.0615192E37F) ;
        p84.x_SET(3.367726E38F) ;
        p84.y_SET(-1.1896177E38F) ;
        p84.vx_SET(5.2954106E37F) ;
        p84.afx_SET(1.217639E38F) ;
        p84.afy_SET(-2.26474E38F) ;
        p84.vz_SET(-2.9143082E38F) ;
        p84.target_system_SET((char)41) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p84.time_boot_ms_SET(2919206022L) ;
        p84.type_mask_SET((char)33769) ;
        p84.vy_SET(-2.1407463E38F) ;
        p84.target_component_SET((char)109) ;
        p84.z_SET(3.2340207E38F) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.afz_GET() == 1.0702709E38F);
            assert(pack.lon_int_GET() == -58523721);
            assert(pack.target_system_GET() == (char)9);
            assert(pack.afy_GET() == 2.434228E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.yaw_rate_GET() == -1.9437718E38F);
            assert(pack.lat_int_GET() == -1074270595);
            assert(pack.vx_GET() == -1.3434195E38F);
            assert(pack.vy_GET() == -3.2950196E38F);
            assert(pack.time_boot_ms_GET() == 2443827409L);
            assert(pack.target_component_GET() == (char)189);
            assert(pack.type_mask_GET() == (char)42638);
            assert(pack.vz_GET() == 2.4372287E38F);
            assert(pack.yaw_GET() == 1.7833128E38F);
            assert(pack.alt_GET() == 5.8369083E37F);
            assert(pack.afx_GET() == 2.1590775E38F);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.vy_SET(-3.2950196E38F) ;
        p86.afx_SET(2.1590775E38F) ;
        p86.type_mask_SET((char)42638) ;
        p86.target_system_SET((char)9) ;
        p86.lon_int_SET(-58523721) ;
        p86.yaw_rate_SET(-1.9437718E38F) ;
        p86.yaw_SET(1.7833128E38F) ;
        p86.afz_SET(1.0702709E38F) ;
        p86.afy_SET(2.434228E38F) ;
        p86.vx_SET(-1.3434195E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p86.alt_SET(5.8369083E37F) ;
        p86.time_boot_ms_SET(2443827409L) ;
        p86.lat_int_SET(-1074270595) ;
        p86.vz_SET(2.4372287E38F) ;
        p86.target_component_SET((char)189) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.afz_GET() == 2.5380742E38F);
            assert(pack.afy_GET() == -9.328492E37F);
            assert(pack.afx_GET() == -3.232551E38F);
            assert(pack.vx_GET() == -2.549602E38F);
            assert(pack.type_mask_GET() == (char)53741);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.lat_int_GET() == 1652390947);
            assert(pack.alt_GET() == -9.764067E37F);
            assert(pack.vy_GET() == 2.776035E38F);
            assert(pack.lon_int_GET() == -1652484574);
            assert(pack.time_boot_ms_GET() == 1701578057L);
            assert(pack.yaw_GET() == 3.2306163E38F);
            assert(pack.vz_GET() == -1.8654854E38F);
            assert(pack.yaw_rate_GET() == -2.2524132E36F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(1701578057L) ;
        p87.afz_SET(2.5380742E38F) ;
        p87.vz_SET(-1.8654854E38F) ;
        p87.alt_SET(-9.764067E37F) ;
        p87.lon_int_SET(-1652484574) ;
        p87.afx_SET(-3.232551E38F) ;
        p87.vx_SET(-2.549602E38F) ;
        p87.type_mask_SET((char)53741) ;
        p87.yaw_SET(3.2306163E38F) ;
        p87.afy_SET(-9.328492E37F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p87.yaw_rate_SET(-2.2524132E36F) ;
        p87.vy_SET(2.776035E38F) ;
        p87.lat_int_SET(1652390947) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1230070298L);
            assert(pack.x_GET() == -7.640525E37F);
            assert(pack.y_GET() == -3.0407242E38F);
            assert(pack.pitch_GET() == 1.8133917E37F);
            assert(pack.z_GET() == -3.268099E38F);
            assert(pack.yaw_GET() == -4.698798E37F);
            assert(pack.roll_GET() == 2.2059857E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.x_SET(-7.640525E37F) ;
        p89.z_SET(-3.268099E38F) ;
        p89.roll_SET(2.2059857E38F) ;
        p89.y_SET(-3.0407242E38F) ;
        p89.pitch_SET(1.8133917E37F) ;
        p89.yaw_SET(-4.698798E37F) ;
        p89.time_boot_ms_SET(1230070298L) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == 1.8432891E38F);
            assert(pack.lat_GET() == 293834178);
            assert(pack.vx_GET() == (short) -17017);
            assert(pack.lon_GET() == -687513136);
            assert(pack.vy_GET() == (short)31322);
            assert(pack.alt_GET() == -373184423);
            assert(pack.yacc_GET() == (short) -30977);
            assert(pack.pitch_GET() == 1.8804716E37F);
            assert(pack.zacc_GET() == (short)24780);
            assert(pack.yawspeed_GET() == 1.5837268E38F);
            assert(pack.pitchspeed_GET() == -2.1243236E38F);
            assert(pack.time_usec_GET() == 2061237338417459097L);
            assert(pack.vz_GET() == (short) -3085);
            assert(pack.xacc_GET() == (short) -4030);
            assert(pack.yaw_GET() == -2.1853074E37F);
            assert(pack.roll_GET() == -1.9169074E38F);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.vx_SET((short) -17017) ;
        p90.time_usec_SET(2061237338417459097L) ;
        p90.zacc_SET((short)24780) ;
        p90.pitch_SET(1.8804716E37F) ;
        p90.yawspeed_SET(1.5837268E38F) ;
        p90.rollspeed_SET(1.8432891E38F) ;
        p90.yaw_SET(-2.1853074E37F) ;
        p90.yacc_SET((short) -30977) ;
        p90.alt_SET(-373184423) ;
        p90.vz_SET((short) -3085) ;
        p90.roll_SET(-1.9169074E38F) ;
        p90.vy_SET((short)31322) ;
        p90.xacc_SET((short) -4030) ;
        p90.lon_SET(-687513136) ;
        p90.lat_SET(293834178) ;
        p90.pitchspeed_SET(-2.1243236E38F) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux2_GET() == 1.2328036E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_ARMED);
            assert(pack.nav_mode_GET() == (char)79);
            assert(pack.roll_ailerons_GET() == 4.7889495E36F);
            assert(pack.aux3_GET() == -3.1438903E38F);
            assert(pack.throttle_GET() == 2.6831902E38F);
            assert(pack.aux4_GET() == 3.0974673E38F);
            assert(pack.aux1_GET() == -1.7042528E38F);
            assert(pack.time_usec_GET() == 8612363510482051679L);
            assert(pack.yaw_rudder_GET() == 1.333705E38F);
            assert(pack.pitch_elevator_GET() == 2.7444181E38F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.pitch_elevator_SET(2.7444181E38F) ;
        p91.aux2_SET(1.2328036E38F) ;
        p91.yaw_rudder_SET(1.333705E38F) ;
        p91.nav_mode_SET((char)79) ;
        p91.aux3_SET(-3.1438903E38F) ;
        p91.roll_ailerons_SET(4.7889495E36F) ;
        p91.aux4_SET(3.0974673E38F) ;
        p91.aux1_SET(-1.7042528E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED) ;
        p91.time_usec_SET(8612363510482051679L) ;
        p91.throttle_SET(2.6831902E38F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan3_raw_GET() == (char)10518);
            assert(pack.chan10_raw_GET() == (char)61788);
            assert(pack.chan8_raw_GET() == (char)55759);
            assert(pack.rssi_GET() == (char)176);
            assert(pack.chan2_raw_GET() == (char)28189);
            assert(pack.chan5_raw_GET() == (char)54490);
            assert(pack.chan1_raw_GET() == (char)7142);
            assert(pack.chan6_raw_GET() == (char)17671);
            assert(pack.chan12_raw_GET() == (char)11369);
            assert(pack.chan4_raw_GET() == (char)11889);
            assert(pack.chan9_raw_GET() == (char)10378);
            assert(pack.chan11_raw_GET() == (char)16165);
            assert(pack.chan7_raw_GET() == (char)25376);
            assert(pack.time_usec_GET() == 5183570460082004572L);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan9_raw_SET((char)10378) ;
        p92.chan2_raw_SET((char)28189) ;
        p92.chan4_raw_SET((char)11889) ;
        p92.time_usec_SET(5183570460082004572L) ;
        p92.chan1_raw_SET((char)7142) ;
        p92.chan7_raw_SET((char)25376) ;
        p92.chan3_raw_SET((char)10518) ;
        p92.chan12_raw_SET((char)11369) ;
        p92.chan5_raw_SET((char)54490) ;
        p92.rssi_SET((char)176) ;
        p92.chan8_raw_SET((char)55759) ;
        p92.chan11_raw_SET((char)16165) ;
        p92.chan10_raw_SET((char)61788) ;
        p92.chan6_raw_SET((char)17671) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.5544654E38F, -2.6423254E38F, -2.0900192E38F, -2.427443E37F, 2.2456055E38F, 1.9179616E37F, -6.949166E37F, 4.4729797E37F, 3.0392365E38F, 3.0251794E38F, -1.8815346E38F, 1.5911694E37F, 3.33277E38F, -2.207755E38F, -2.333228E38F, -1.9721394E38F}));
            assert(pack.flags_GET() == 9169603611994112980L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_DISARMED);
            assert(pack.time_usec_GET() == 8372936770314343460L);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.flags_SET(9169603611994112980L) ;
        p93.controls_SET(new float[] {-1.5544654E38F, -2.6423254E38F, -2.0900192E38F, -2.427443E37F, 2.2456055E38F, 1.9179616E37F, -6.949166E37F, 4.4729797E37F, 3.0392365E38F, 3.0251794E38F, -1.8815346E38F, 1.5911694E37F, 3.33277E38F, -2.207755E38F, -2.333228E38F, -1.9721394E38F}, 0) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED) ;
        p93.time_usec_SET(8372936770314343460L) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3770485487765368839L);
            assert(pack.flow_rate_x_TRY(ph) == -7.945498E37F);
            assert(pack.flow_x_GET() == (short)32755);
            assert(pack.quality_GET() == (char)194);
            assert(pack.flow_rate_y_TRY(ph) == 2.3555242E38F);
            assert(pack.sensor_id_GET() == (char)178);
            assert(pack.ground_distance_GET() == 5.2985407E37F);
            assert(pack.flow_comp_m_x_GET() == -1.7185753E37F);
            assert(pack.flow_comp_m_y_GET() == -1.1464862E38F);
            assert(pack.flow_y_GET() == (short) -27043);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_rate_x_SET(-7.945498E37F, PH) ;
        p100.flow_rate_y_SET(2.3555242E38F, PH) ;
        p100.time_usec_SET(3770485487765368839L) ;
        p100.quality_SET((char)194) ;
        p100.flow_comp_m_x_SET(-1.7185753E37F) ;
        p100.flow_comp_m_y_SET(-1.1464862E38F) ;
        p100.flow_y_SET((short) -27043) ;
        p100.flow_x_SET((short)32755) ;
        p100.sensor_id_SET((char)178) ;
        p100.ground_distance_SET(5.2985407E37F) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -7.6322875E37F);
            assert(pack.pitch_GET() == 2.741426E38F);
            assert(pack.y_GET() == -8.766608E37F);
            assert(pack.usec_GET() == 2806101619322124545L);
            assert(pack.yaw_GET() == -2.5003968E38F);
            assert(pack.x_GET() == -2.1510793E38F);
            assert(pack.roll_GET() == 2.4485787E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.z_SET(-7.6322875E37F) ;
        p101.roll_SET(2.4485787E38F) ;
        p101.pitch_SET(2.741426E38F) ;
        p101.y_SET(-8.766608E37F) ;
        p101.yaw_SET(-2.5003968E38F) ;
        p101.usec_SET(2806101619322124545L) ;
        p101.x_SET(-2.1510793E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 6316156172377073013L);
            assert(pack.x_GET() == 2.0329177E38F);
            assert(pack.pitch_GET() == 8.830114E37F);
            assert(pack.yaw_GET() == -2.5668069E38F);
            assert(pack.roll_GET() == 2.395417E38F);
            assert(pack.z_GET() == 2.7952337E38F);
            assert(pack.y_GET() == 2.0078657E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.yaw_SET(-2.5668069E38F) ;
        p102.pitch_SET(8.830114E37F) ;
        p102.x_SET(2.0329177E38F) ;
        p102.roll_SET(2.395417E38F) ;
        p102.z_SET(2.7952337E38F) ;
        p102.usec_SET(6316156172377073013L) ;
        p102.y_SET(2.0078657E38F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 1.4171818E38F);
            assert(pack.x_GET() == -2.7894248E38F);
            assert(pack.z_GET() == 2.9338305E38F);
            assert(pack.usec_GET() == 5051856144520064236L);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.z_SET(2.9338305E38F) ;
        p103.x_SET(-2.7894248E38F) ;
        p103.usec_SET(5051856144520064236L) ;
        p103.y_SET(1.4171818E38F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 6.993525E37F);
            assert(pack.z_GET() == -2.4482893E38F);
            assert(pack.x_GET() == 8.638682E37F);
            assert(pack.yaw_GET() == 2.6355975E38F);
            assert(pack.pitch_GET() == -1.2491787E38F);
            assert(pack.usec_GET() == 1615734816237419881L);
            assert(pack.roll_GET() == 2.4293439E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.yaw_SET(2.6355975E38F) ;
        p104.pitch_SET(-1.2491787E38F) ;
        p104.z_SET(-2.4482893E38F) ;
        p104.y_SET(6.993525E37F) ;
        p104.usec_SET(1615734816237419881L) ;
        p104.x_SET(8.638682E37F) ;
        p104.roll_SET(2.4293439E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.abs_pressure_GET() == 1.4068878E38F);
            assert(pack.zgyro_GET() == 4.90215E37F);
            assert(pack.yacc_GET() == -4.12234E37F);
            assert(pack.diff_pressure_GET() == -7.7736614E37F);
            assert(pack.ygyro_GET() == 3.7145423E37F);
            assert(pack.temperature_GET() == -2.933549E38F);
            assert(pack.zacc_GET() == 9.058222E37F);
            assert(pack.time_usec_GET() == 5750773074985390008L);
            assert(pack.xacc_GET() == 3.0697275E38F);
            assert(pack.xgyro_GET() == -9.081872E37F);
            assert(pack.zmag_GET() == 2.5131112E38F);
            assert(pack.pressure_alt_GET() == 2.3850852E38F);
            assert(pack.ymag_GET() == 3.1284053E38F);
            assert(pack.xmag_GET() == -2.7050735E38F);
            assert(pack.fields_updated_GET() == (char)30088);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.diff_pressure_SET(-7.7736614E37F) ;
        p105.zacc_SET(9.058222E37F) ;
        p105.fields_updated_SET((char)30088) ;
        p105.temperature_SET(-2.933549E38F) ;
        p105.xgyro_SET(-9.081872E37F) ;
        p105.ymag_SET(3.1284053E38F) ;
        p105.ygyro_SET(3.7145423E37F) ;
        p105.zmag_SET(2.5131112E38F) ;
        p105.pressure_alt_SET(2.3850852E38F) ;
        p105.yacc_SET(-4.12234E37F) ;
        p105.xacc_SET(3.0697275E38F) ;
        p105.time_usec_SET(5750773074985390008L) ;
        p105.zgyro_SET(4.90215E37F) ;
        p105.xmag_SET(-2.7050735E38F) ;
        p105.abs_pressure_SET(1.4068878E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_y_GET() == -1.5120061E37F);
            assert(pack.integrated_ygyro_GET() == 2.4454962E38F);
            assert(pack.integration_time_us_GET() == 2893058228L);
            assert(pack.sensor_id_GET() == (char)173);
            assert(pack.time_delta_distance_us_GET() == 4204893159L);
            assert(pack.integrated_zgyro_GET() == -2.926978E37F);
            assert(pack.temperature_GET() == (short) -15565);
            assert(pack.time_usec_GET() == 2541309732573983976L);
            assert(pack.integrated_xgyro_GET() == -2.8626358E38F);
            assert(pack.quality_GET() == (char)136);
            assert(pack.integrated_x_GET() == -1.2098181E38F);
            assert(pack.distance_GET() == -1.5004255E37F);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.sensor_id_SET((char)173) ;
        p106.integrated_xgyro_SET(-2.8626358E38F) ;
        p106.temperature_SET((short) -15565) ;
        p106.integrated_zgyro_SET(-2.926978E37F) ;
        p106.integrated_x_SET(-1.2098181E38F) ;
        p106.integration_time_us_SET(2893058228L) ;
        p106.time_delta_distance_us_SET(4204893159L) ;
        p106.integrated_y_SET(-1.5120061E37F) ;
        p106.quality_SET((char)136) ;
        p106.integrated_ygyro_SET(2.4454962E38F) ;
        p106.time_usec_SET(2541309732573983976L) ;
        p106.distance_SET(-1.5004255E37F) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == -2.9782887E38F);
            assert(pack.diff_pressure_GET() == 1.2365829E37F);
            assert(pack.xacc_GET() == 5.1440795E37F);
            assert(pack.ygyro_GET() == 1.05705764E37F);
            assert(pack.ymag_GET() == 1.7467597E38F);
            assert(pack.pressure_alt_GET() == -7.949538E37F);
            assert(pack.zmag_GET() == 2.8432417E38F);
            assert(pack.fields_updated_GET() == 3030193367L);
            assert(pack.temperature_GET() == -2.069606E38F);
            assert(pack.yacc_GET() == -3.261293E37F);
            assert(pack.zacc_GET() == -7.0957985E37F);
            assert(pack.zgyro_GET() == -2.9068161E38F);
            assert(pack.time_usec_GET() == 7766111641739340187L);
            assert(pack.xgyro_GET() == 2.109148E38F);
            assert(pack.abs_pressure_GET() == -1.3207089E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.xacc_SET(5.1440795E37F) ;
        p107.temperature_SET(-2.069606E38F) ;
        p107.yacc_SET(-3.261293E37F) ;
        p107.zgyro_SET(-2.9068161E38F) ;
        p107.abs_pressure_SET(-1.3207089E38F) ;
        p107.ymag_SET(1.7467597E38F) ;
        p107.time_usec_SET(7766111641739340187L) ;
        p107.zmag_SET(2.8432417E38F) ;
        p107.xmag_SET(-2.9782887E38F) ;
        p107.xgyro_SET(2.109148E38F) ;
        p107.diff_pressure_SET(1.2365829E37F) ;
        p107.fields_updated_SET(3030193367L) ;
        p107.pressure_alt_SET(-7.949538E37F) ;
        p107.zacc_SET(-7.0957985E37F) ;
        p107.ygyro_SET(1.05705764E37F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 6.7144243E37F);
            assert(pack.zacc_GET() == -6.3342153E37F);
            assert(pack.pitch_GET() == -3.1758197E38F);
            assert(pack.q4_GET() == -2.6810579E38F);
            assert(pack.zgyro_GET() == 3.1657032E38F);
            assert(pack.ygyro_GET() == -1.8101073E38F);
            assert(pack.roll_GET() == 2.3661678E38F);
            assert(pack.ve_GET() == 2.2561088E38F);
            assert(pack.yaw_GET() == -2.4764707E38F);
            assert(pack.lon_GET() == 1.6071098E38F);
            assert(pack.q1_GET() == -6.199587E37F);
            assert(pack.std_dev_vert_GET() == -3.261461E38F);
            assert(pack.yacc_GET() == 3.0139983E38F);
            assert(pack.q2_GET() == -2.8400462E38F);
            assert(pack.vn_GET() == -3.1187452E38F);
            assert(pack.vd_GET() == -1.0078057E38F);
            assert(pack.q3_GET() == -3.0147193E37F);
            assert(pack.xgyro_GET() == 8.826499E37F);
            assert(pack.alt_GET() == 8.757432E37F);
            assert(pack.xacc_GET() == -2.8422207E38F);
            assert(pack.std_dev_horz_GET() == -6.5220137E37F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.yaw_SET(-2.4764707E38F) ;
        p108.lat_SET(6.7144243E37F) ;
        p108.xgyro_SET(8.826499E37F) ;
        p108.roll_SET(2.3661678E38F) ;
        p108.pitch_SET(-3.1758197E38F) ;
        p108.std_dev_vert_SET(-3.261461E38F) ;
        p108.q4_SET(-2.6810579E38F) ;
        p108.zgyro_SET(3.1657032E38F) ;
        p108.vn_SET(-3.1187452E38F) ;
        p108.vd_SET(-1.0078057E38F) ;
        p108.q2_SET(-2.8400462E38F) ;
        p108.zacc_SET(-6.3342153E37F) ;
        p108.std_dev_horz_SET(-6.5220137E37F) ;
        p108.q1_SET(-6.199587E37F) ;
        p108.yacc_SET(3.0139983E38F) ;
        p108.ve_SET(2.2561088E38F) ;
        p108.alt_SET(8.757432E37F) ;
        p108.ygyro_SET(-1.8101073E38F) ;
        p108.xacc_SET(-2.8422207E38F) ;
        p108.q3_SET(-3.0147193E37F) ;
        p108.lon_SET(1.6071098E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.remnoise_GET() == (char)183);
            assert(pack.txbuf_GET() == (char)68);
            assert(pack.remrssi_GET() == (char)229);
            assert(pack.rxerrors_GET() == (char)32211);
            assert(pack.fixed__GET() == (char)42582);
            assert(pack.noise_GET() == (char)148);
            assert(pack.rssi_GET() == (char)68);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.txbuf_SET((char)68) ;
        p109.rssi_SET((char)68) ;
        p109.remnoise_SET((char)183) ;
        p109.rxerrors_SET((char)32211) ;
        p109.remrssi_SET((char)229) ;
        p109.fixed__SET((char)42582) ;
        p109.noise_SET((char)148) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)75, (char)77, (char)73, (char)88, (char)95, (char)156, (char)167, (char)53, (char)1, (char)150, (char)52, (char)216, (char)131, (char)127, (char)238, (char)228, (char)70, (char)12, (char)115, (char)136, (char)95, (char)75, (char)229, (char)178, (char)0, (char)232, (char)100, (char)187, (char)98, (char)160, (char)13, (char)139, (char)57, (char)195, (char)93, (char)54, (char)206, (char)76, (char)24, (char)75, (char)162, (char)64, (char)155, (char)60, (char)31, (char)223, (char)157, (char)112, (char)234, (char)251, (char)51, (char)81, (char)178, (char)4, (char)233, (char)28, (char)51, (char)227, (char)55, (char)69, (char)108, (char)34, (char)152, (char)82, (char)249, (char)177, (char)141, (char)60, (char)77, (char)153, (char)87, (char)43, (char)116, (char)64, (char)106, (char)178, (char)149, (char)155, (char)68, (char)228, (char)4, (char)21, (char)7, (char)82, (char)178, (char)69, (char)103, (char)2, (char)184, (char)4, (char)4, (char)6, (char)238, (char)233, (char)170, (char)130, (char)233, (char)136, (char)99, (char)243, (char)108, (char)51, (char)248, (char)185, (char)193, (char)125, (char)33, (char)173, (char)223, (char)65, (char)88, (char)204, (char)91, (char)240, (char)116, (char)145, (char)144, (char)181, (char)76, (char)29, (char)208, (char)107, (char)163, (char)178, (char)245, (char)123, (char)10, (char)69, (char)163, (char)170, (char)223, (char)104, (char)215, (char)64, (char)82, (char)7, (char)125, (char)138, (char)226, (char)231, (char)144, (char)16, (char)5, (char)237, (char)36, (char)127, (char)136, (char)42, (char)68, (char)234, (char)174, (char)245, (char)229, (char)8, (char)201, (char)211, (char)12, (char)141, (char)2, (char)19, (char)97, (char)142, (char)229, (char)59, (char)25, (char)78, (char)205, (char)236, (char)221, (char)10, (char)20, (char)203, (char)93, (char)178, (char)97, (char)245, (char)113, (char)235, (char)93, (char)216, (char)78, (char)42, (char)67, (char)8, (char)144, (char)186, (char)36, (char)67, (char)136, (char)137, (char)31, (char)17, (char)195, (char)39, (char)194, (char)102, (char)100, (char)134, (char)144, (char)32, (char)154, (char)237, (char)167, (char)175, (char)92, (char)145, (char)102, (char)166, (char)192, (char)39, (char)246, (char)63, (char)60, (char)139, (char)85, (char)57, (char)171, (char)182, (char)241, (char)92, (char)171, (char)232, (char)128, (char)116, (char)109, (char)170, (char)133, (char)87, (char)22, (char)83, (char)96, (char)230, (char)136, (char)46, (char)102, (char)57, (char)167, (char)91, (char)218, (char)221, (char)71, (char)222, (char)150, (char)235, (char)165, (char)145, (char)103, (char)154, (char)72, (char)29, (char)169}));
            assert(pack.target_network_GET() == (char)97);
            assert(pack.target_component_GET() == (char)17);
            assert(pack.target_system_GET() == (char)144);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.payload_SET(new char[] {(char)75, (char)77, (char)73, (char)88, (char)95, (char)156, (char)167, (char)53, (char)1, (char)150, (char)52, (char)216, (char)131, (char)127, (char)238, (char)228, (char)70, (char)12, (char)115, (char)136, (char)95, (char)75, (char)229, (char)178, (char)0, (char)232, (char)100, (char)187, (char)98, (char)160, (char)13, (char)139, (char)57, (char)195, (char)93, (char)54, (char)206, (char)76, (char)24, (char)75, (char)162, (char)64, (char)155, (char)60, (char)31, (char)223, (char)157, (char)112, (char)234, (char)251, (char)51, (char)81, (char)178, (char)4, (char)233, (char)28, (char)51, (char)227, (char)55, (char)69, (char)108, (char)34, (char)152, (char)82, (char)249, (char)177, (char)141, (char)60, (char)77, (char)153, (char)87, (char)43, (char)116, (char)64, (char)106, (char)178, (char)149, (char)155, (char)68, (char)228, (char)4, (char)21, (char)7, (char)82, (char)178, (char)69, (char)103, (char)2, (char)184, (char)4, (char)4, (char)6, (char)238, (char)233, (char)170, (char)130, (char)233, (char)136, (char)99, (char)243, (char)108, (char)51, (char)248, (char)185, (char)193, (char)125, (char)33, (char)173, (char)223, (char)65, (char)88, (char)204, (char)91, (char)240, (char)116, (char)145, (char)144, (char)181, (char)76, (char)29, (char)208, (char)107, (char)163, (char)178, (char)245, (char)123, (char)10, (char)69, (char)163, (char)170, (char)223, (char)104, (char)215, (char)64, (char)82, (char)7, (char)125, (char)138, (char)226, (char)231, (char)144, (char)16, (char)5, (char)237, (char)36, (char)127, (char)136, (char)42, (char)68, (char)234, (char)174, (char)245, (char)229, (char)8, (char)201, (char)211, (char)12, (char)141, (char)2, (char)19, (char)97, (char)142, (char)229, (char)59, (char)25, (char)78, (char)205, (char)236, (char)221, (char)10, (char)20, (char)203, (char)93, (char)178, (char)97, (char)245, (char)113, (char)235, (char)93, (char)216, (char)78, (char)42, (char)67, (char)8, (char)144, (char)186, (char)36, (char)67, (char)136, (char)137, (char)31, (char)17, (char)195, (char)39, (char)194, (char)102, (char)100, (char)134, (char)144, (char)32, (char)154, (char)237, (char)167, (char)175, (char)92, (char)145, (char)102, (char)166, (char)192, (char)39, (char)246, (char)63, (char)60, (char)139, (char)85, (char)57, (char)171, (char)182, (char)241, (char)92, (char)171, (char)232, (char)128, (char)116, (char)109, (char)170, (char)133, (char)87, (char)22, (char)83, (char)96, (char)230, (char)136, (char)46, (char)102, (char)57, (char)167, (char)91, (char)218, (char)221, (char)71, (char)222, (char)150, (char)235, (char)165, (char)145, (char)103, (char)154, (char)72, (char)29, (char)169}, 0) ;
        p110.target_system_SET((char)144) ;
        p110.target_component_SET((char)17) ;
        p110.target_network_SET((char)97) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == 8855057062381959083L);
            assert(pack.ts1_GET() == -2639617878052716305L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(-2639617878052716305L) ;
        p111.tc1_SET(8855057062381959083L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5790839736432924143L);
            assert(pack.seq_GET() == 961350163L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(961350163L) ;
        p112.time_usec_SET(5790839736432924143L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.ve_GET() == (short) -9755);
            assert(pack.vel_GET() == (char)37483);
            assert(pack.satellites_visible_GET() == (char)102);
            assert(pack.fix_type_GET() == (char)130);
            assert(pack.eph_GET() == (char)30247);
            assert(pack.lat_GET() == -455386764);
            assert(pack.epv_GET() == (char)33157);
            assert(pack.vn_GET() == (short)31209);
            assert(pack.lon_GET() == 1850731360);
            assert(pack.time_usec_GET() == 1368422557026449733L);
            assert(pack.alt_GET() == 1048698789);
            assert(pack.vd_GET() == (short) -1535);
            assert(pack.cog_GET() == (char)24013);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.vd_SET((short) -1535) ;
        p113.fix_type_SET((char)130) ;
        p113.vn_SET((short)31209) ;
        p113.lon_SET(1850731360) ;
        p113.epv_SET((char)33157) ;
        p113.time_usec_SET(1368422557026449733L) ;
        p113.cog_SET((char)24013) ;
        p113.vel_SET((char)37483) ;
        p113.eph_SET((char)30247) ;
        p113.satellites_visible_SET((char)102) ;
        p113.ve_SET((short) -9755) ;
        p113.lat_SET(-455386764) ;
        p113.alt_SET(1048698789) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.distance_GET() == 2.053583E37F);
            assert(pack.time_usec_GET() == 3158371985666348145L);
            assert(pack.integrated_xgyro_GET() == 8.854904E37F);
            assert(pack.integrated_y_GET() == 4.824849E37F);
            assert(pack.sensor_id_GET() == (char)6);
            assert(pack.integrated_x_GET() == -2.4623133E38F);
            assert(pack.quality_GET() == (char)75);
            assert(pack.integration_time_us_GET() == 4119524394L);
            assert(pack.temperature_GET() == (short) -7890);
            assert(pack.integrated_zgyro_GET() == 2.773509E38F);
            assert(pack.integrated_ygyro_GET() == -1.2565126E38F);
            assert(pack.time_delta_distance_us_GET() == 4183158472L);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_ygyro_SET(-1.2565126E38F) ;
        p114.distance_SET(2.053583E37F) ;
        p114.integration_time_us_SET(4119524394L) ;
        p114.integrated_zgyro_SET(2.773509E38F) ;
        p114.integrated_xgyro_SET(8.854904E37F) ;
        p114.temperature_SET((short) -7890) ;
        p114.integrated_x_SET(-2.4623133E38F) ;
        p114.time_delta_distance_us_SET(4183158472L) ;
        p114.time_usec_SET(3158371985666348145L) ;
        p114.sensor_id_SET((char)6) ;
        p114.quality_SET((char)75) ;
        p114.integrated_y_SET(4.824849E37F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short) -20323);
            assert(pack.rollspeed_GET() == 2.5158954E38F);
            assert(pack.yacc_GET() == (short) -21049);
            assert(pack.lat_GET() == -856259686);
            assert(pack.vx_GET() == (short) -27340);
            assert(pack.ind_airspeed_GET() == (char)32488);
            assert(pack.true_airspeed_GET() == (char)1990);
            assert(pack.alt_GET() == 3758122);
            assert(pack.zacc_GET() == (short) -8680);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {1.5603146E38F, 9.174992E37F, 6.9446595E37F, 1.3090958E37F}));
            assert(pack.yawspeed_GET() == -2.3361204E38F);
            assert(pack.pitchspeed_GET() == -3.4561302E37F);
            assert(pack.vy_GET() == (short) -30358);
            assert(pack.time_usec_GET() == 1203965353078288533L);
            assert(pack.vz_GET() == (short) -21615);
            assert(pack.lon_GET() == -433059962);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.rollspeed_SET(2.5158954E38F) ;
        p115.lat_SET(-856259686) ;
        p115.attitude_quaternion_SET(new float[] {1.5603146E38F, 9.174992E37F, 6.9446595E37F, 1.3090958E37F}, 0) ;
        p115.pitchspeed_SET(-3.4561302E37F) ;
        p115.vy_SET((short) -30358) ;
        p115.lon_SET(-433059962) ;
        p115.yacc_SET((short) -21049) ;
        p115.true_airspeed_SET((char)1990) ;
        p115.ind_airspeed_SET((char)32488) ;
        p115.yawspeed_SET(-2.3361204E38F) ;
        p115.vz_SET((short) -21615) ;
        p115.zacc_SET((short) -8680) ;
        p115.xacc_SET((short) -20323) ;
        p115.vx_SET((short) -27340) ;
        p115.alt_SET(3758122) ;
        p115.time_usec_SET(1203965353078288533L) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short)8208);
            assert(pack.xgyro_GET() == (short)23015);
            assert(pack.zgyro_GET() == (short) -14074);
            assert(pack.xacc_GET() == (short)28189);
            assert(pack.ymag_GET() == (short)4963);
            assert(pack.xmag_GET() == (short)24234);
            assert(pack.zmag_GET() == (short) -7917);
            assert(pack.time_boot_ms_GET() == 1845617357L);
            assert(pack.ygyro_GET() == (short) -19402);
            assert(pack.zacc_GET() == (short)5909);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.zacc_SET((short)5909) ;
        p116.xacc_SET((short)28189) ;
        p116.xgyro_SET((short)23015) ;
        p116.time_boot_ms_SET(1845617357L) ;
        p116.zgyro_SET((short) -14074) ;
        p116.xmag_SET((short)24234) ;
        p116.ymag_SET((short)4963) ;
        p116.yacc_SET((short)8208) ;
        p116.zmag_SET((short) -7917) ;
        p116.ygyro_SET((short) -19402) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)198);
            assert(pack.start_GET() == (char)9447);
            assert(pack.target_system_GET() == (char)174);
            assert(pack.end_GET() == (char)47037);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_component_SET((char)198) ;
        p117.start_SET((char)9447) ;
        p117.end_SET((char)47037) ;
        p117.target_system_SET((char)174) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.num_logs_GET() == (char)31358);
            assert(pack.size_GET() == 1518008017L);
            assert(pack.time_utc_GET() == 3291539404L);
            assert(pack.id_GET() == (char)10918);
            assert(pack.last_log_num_GET() == (char)62755);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.size_SET(1518008017L) ;
        p118.id_SET((char)10918) ;
        p118.time_utc_SET(3291539404L) ;
        p118.num_logs_SET((char)31358) ;
        p118.last_log_num_SET((char)62755) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)29);
            assert(pack.count_GET() == 1066201944L);
            assert(pack.id_GET() == (char)5023);
            assert(pack.ofs_GET() == 2639504530L);
            assert(pack.target_system_GET() == (char)52);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.id_SET((char)5023) ;
        p119.ofs_SET(2639504530L) ;
        p119.target_component_SET((char)29) ;
        p119.target_system_SET((char)52) ;
        p119.count_SET(1066201944L) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)57823);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)89, (char)49, (char)168, (char)152, (char)157, (char)2, (char)198, (char)112, (char)196, (char)252, (char)51, (char)165, (char)100, (char)225, (char)120, (char)240, (char)220, (char)237, (char)26, (char)60, (char)102, (char)156, (char)58, (char)220, (char)167, (char)111, (char)148, (char)42, (char)152, (char)77, (char)208, (char)37, (char)143, (char)0, (char)23, (char)24, (char)220, (char)131, (char)174, (char)109, (char)7, (char)143, (char)221, (char)52, (char)70, (char)164, (char)248, (char)240, (char)112, (char)232, (char)156, (char)10, (char)106, (char)123, (char)160, (char)237, (char)100, (char)197, (char)7, (char)84, (char)100, (char)107, (char)85, (char)78, (char)113, (char)69, (char)52, (char)178, (char)1, (char)51, (char)101, (char)67, (char)26, (char)238, (char)81, (char)1, (char)72, (char)232, (char)48, (char)15, (char)125, (char)249, (char)70, (char)69, (char)135, (char)57, (char)180, (char)187, (char)39, (char)191}));
            assert(pack.ofs_GET() == 201399103L);
            assert(pack.count_GET() == (char)111);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.data__SET(new char[] {(char)89, (char)49, (char)168, (char)152, (char)157, (char)2, (char)198, (char)112, (char)196, (char)252, (char)51, (char)165, (char)100, (char)225, (char)120, (char)240, (char)220, (char)237, (char)26, (char)60, (char)102, (char)156, (char)58, (char)220, (char)167, (char)111, (char)148, (char)42, (char)152, (char)77, (char)208, (char)37, (char)143, (char)0, (char)23, (char)24, (char)220, (char)131, (char)174, (char)109, (char)7, (char)143, (char)221, (char)52, (char)70, (char)164, (char)248, (char)240, (char)112, (char)232, (char)156, (char)10, (char)106, (char)123, (char)160, (char)237, (char)100, (char)197, (char)7, (char)84, (char)100, (char)107, (char)85, (char)78, (char)113, (char)69, (char)52, (char)178, (char)1, (char)51, (char)101, (char)67, (char)26, (char)238, (char)81, (char)1, (char)72, (char)232, (char)48, (char)15, (char)125, (char)249, (char)70, (char)69, (char)135, (char)57, (char)180, (char)187, (char)39, (char)191}, 0) ;
        p120.count_SET((char)111) ;
        p120.ofs_SET(201399103L) ;
        p120.id_SET((char)57823) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)27);
            assert(pack.target_component_GET() == (char)85);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)85) ;
        p121.target_system_SET((char)27) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)94);
            assert(pack.target_component_GET() == (char)98);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_component_SET((char)98) ;
        p122.target_system_SET((char)94) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)125);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)237, (char)73, (char)171, (char)24, (char)80, (char)54, (char)206, (char)248, (char)14, (char)153, (char)217, (char)129, (char)121, (char)14, (char)53, (char)72, (char)167, (char)139, (char)214, (char)36, (char)140, (char)2, (char)42, (char)67, (char)90, (char)65, (char)39, (char)76, (char)242, (char)185, (char)171, (char)10, (char)68, (char)167, (char)171, (char)27, (char)23, (char)118, (char)163, (char)203, (char)164, (char)35, (char)168, (char)1, (char)202, (char)175, (char)138, (char)55, (char)207, (char)189, (char)140, (char)114, (char)142, (char)143, (char)229, (char)91, (char)131, (char)245, (char)50, (char)88, (char)21, (char)48, (char)56, (char)11, (char)44, (char)162, (char)98, (char)233, (char)169, (char)81, (char)31, (char)210, (char)253, (char)93, (char)241, (char)17, (char)161, (char)102, (char)191, (char)234, (char)131, (char)102, (char)122, (char)62, (char)89, (char)169, (char)241, (char)51, (char)232, (char)127, (char)15, (char)119, (char)85, (char)16, (char)80, (char)130, (char)130, (char)191, (char)75, (char)250, (char)74, (char)116, (char)69, (char)126, (char)205, (char)128, (char)148, (char)200, (char)67, (char)64}));
            assert(pack.len_GET() == (char)69);
            assert(pack.target_system_GET() == (char)26);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_component_SET((char)125) ;
        p123.data__SET(new char[] {(char)237, (char)73, (char)171, (char)24, (char)80, (char)54, (char)206, (char)248, (char)14, (char)153, (char)217, (char)129, (char)121, (char)14, (char)53, (char)72, (char)167, (char)139, (char)214, (char)36, (char)140, (char)2, (char)42, (char)67, (char)90, (char)65, (char)39, (char)76, (char)242, (char)185, (char)171, (char)10, (char)68, (char)167, (char)171, (char)27, (char)23, (char)118, (char)163, (char)203, (char)164, (char)35, (char)168, (char)1, (char)202, (char)175, (char)138, (char)55, (char)207, (char)189, (char)140, (char)114, (char)142, (char)143, (char)229, (char)91, (char)131, (char)245, (char)50, (char)88, (char)21, (char)48, (char)56, (char)11, (char)44, (char)162, (char)98, (char)233, (char)169, (char)81, (char)31, (char)210, (char)253, (char)93, (char)241, (char)17, (char)161, (char)102, (char)191, (char)234, (char)131, (char)102, (char)122, (char)62, (char)89, (char)169, (char)241, (char)51, (char)232, (char)127, (char)15, (char)119, (char)85, (char)16, (char)80, (char)130, (char)130, (char)191, (char)75, (char)250, (char)74, (char)116, (char)69, (char)126, (char)205, (char)128, (char)148, (char)200, (char)67, (char)64}, 0) ;
        p123.target_system_SET((char)26) ;
        p123.len_SET((char)69) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.epv_GET() == (char)64632);
            assert(pack.eph_GET() == (char)35925);
            assert(pack.dgps_age_GET() == 1197873987L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
            assert(pack.cog_GET() == (char)29486);
            assert(pack.dgps_numch_GET() == (char)96);
            assert(pack.alt_GET() == -478295778);
            assert(pack.time_usec_GET() == 3141507046772124559L);
            assert(pack.lat_GET() == -154487812);
            assert(pack.lon_GET() == 544512227);
            assert(pack.vel_GET() == (char)19122);
            assert(pack.satellites_visible_GET() == (char)129);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.lon_SET(544512227) ;
        p124.satellites_visible_SET((char)129) ;
        p124.lat_SET(-154487812) ;
        p124.vel_SET((char)19122) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX) ;
        p124.epv_SET((char)64632) ;
        p124.dgps_age_SET(1197873987L) ;
        p124.cog_SET((char)29486) ;
        p124.time_usec_SET(3141507046772124559L) ;
        p124.eph_SET((char)35925) ;
        p124.dgps_numch_SET((char)96) ;
        p124.alt_SET(-478295778) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vservo_GET() == (char)12197);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID));
            assert(pack.Vcc_GET() == (char)51962);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)12197) ;
        p125.Vcc_SET((char)51962) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID)) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.timeout_GET() == (char)46604);
            assert(pack.count_GET() == (char)202);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI));
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)172, (char)178, (char)73, (char)14, (char)182, (char)63, (char)160, (char)97, (char)82, (char)140, (char)96, (char)19, (char)79, (char)147, (char)19, (char)118, (char)123, (char)204, (char)102, (char)131, (char)13, (char)203, (char)253, (char)34, (char)114, (char)136, (char)42, (char)233, (char)231, (char)59, (char)67, (char)32, (char)163, (char)14, (char)9, (char)124, (char)155, (char)53, (char)121, (char)215, (char)67, (char)70, (char)212, (char)29, (char)102, (char)106, (char)254, (char)8, (char)87, (char)38, (char)50, (char)80, (char)80, (char)38, (char)151, (char)50, (char)144, (char)146, (char)4, (char)131, (char)252, (char)82, (char)255, (char)155, (char)110, (char)20, (char)143, (char)134, (char)20, (char)48}));
            assert(pack.baudrate_GET() == 3622076566L);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.baudrate_SET(3622076566L) ;
        p126.data__SET(new char[] {(char)172, (char)178, (char)73, (char)14, (char)182, (char)63, (char)160, (char)97, (char)82, (char)140, (char)96, (char)19, (char)79, (char)147, (char)19, (char)118, (char)123, (char)204, (char)102, (char)131, (char)13, (char)203, (char)253, (char)34, (char)114, (char)136, (char)42, (char)233, (char)231, (char)59, (char)67, (char)32, (char)163, (char)14, (char)9, (char)124, (char)155, (char)53, (char)121, (char)215, (char)67, (char)70, (char)212, (char)29, (char)102, (char)106, (char)254, (char)8, (char)87, (char)38, (char)50, (char)80, (char)80, (char)38, (char)151, (char)50, (char)144, (char)146, (char)4, (char)131, (char)252, (char)82, (char)255, (char)155, (char)110, (char)20, (char)143, (char)134, (char)20, (char)48}, 0) ;
        p126.count_SET((char)202) ;
        p126.timeout_SET((char)46604) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI)) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_c_mm_GET() == 230182295);
            assert(pack.tow_GET() == 663151204L);
            assert(pack.nsats_GET() == (char)76);
            assert(pack.iar_num_hypotheses_GET() == -877415176);
            assert(pack.rtk_receiver_id_GET() == (char)204);
            assert(pack.rtk_health_GET() == (char)68);
            assert(pack.wn_GET() == (char)29032);
            assert(pack.rtk_rate_GET() == (char)51);
            assert(pack.baseline_coords_type_GET() == (char)2);
            assert(pack.accuracy_GET() == 322867363L);
            assert(pack.time_last_baseline_ms_GET() == 1832757586L);
            assert(pack.baseline_a_mm_GET() == 159292486);
            assert(pack.baseline_b_mm_GET() == -1114609415);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(1832757586L) ;
        p127.baseline_coords_type_SET((char)2) ;
        p127.rtk_health_SET((char)68) ;
        p127.iar_num_hypotheses_SET(-877415176) ;
        p127.baseline_c_mm_SET(230182295) ;
        p127.accuracy_SET(322867363L) ;
        p127.wn_SET((char)29032) ;
        p127.rtk_receiver_id_SET((char)204) ;
        p127.nsats_SET((char)76) ;
        p127.baseline_a_mm_SET(159292486) ;
        p127.tow_SET(663151204L) ;
        p127.baseline_b_mm_SET(-1114609415) ;
        p127.rtk_rate_SET((char)51) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_c_mm_GET() == 630308584);
            assert(pack.rtk_rate_GET() == (char)32);
            assert(pack.baseline_a_mm_GET() == -560189642);
            assert(pack.tow_GET() == 1095058463L);
            assert(pack.rtk_receiver_id_GET() == (char)23);
            assert(pack.wn_GET() == (char)1724);
            assert(pack.time_last_baseline_ms_GET() == 1968760634L);
            assert(pack.nsats_GET() == (char)14);
            assert(pack.baseline_b_mm_GET() == 2020257805);
            assert(pack.iar_num_hypotheses_GET() == -835592020);
            assert(pack.accuracy_GET() == 2718689517L);
            assert(pack.rtk_health_GET() == (char)57);
            assert(pack.baseline_coords_type_GET() == (char)242);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_coords_type_SET((char)242) ;
        p128.nsats_SET((char)14) ;
        p128.rtk_receiver_id_SET((char)23) ;
        p128.iar_num_hypotheses_SET(-835592020) ;
        p128.rtk_rate_SET((char)32) ;
        p128.time_last_baseline_ms_SET(1968760634L) ;
        p128.baseline_c_mm_SET(630308584) ;
        p128.baseline_b_mm_SET(2020257805) ;
        p128.wn_SET((char)1724) ;
        p128.tow_SET(1095058463L) ;
        p128.rtk_health_SET((char)57) ;
        p128.accuracy_SET(2718689517L) ;
        p128.baseline_a_mm_SET(-560189642) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == (short)20807);
            assert(pack.yacc_GET() == (short) -16531);
            assert(pack.zacc_GET() == (short) -24868);
            assert(pack.zmag_GET() == (short)21085);
            assert(pack.xacc_GET() == (short)10766);
            assert(pack.zgyro_GET() == (short)22993);
            assert(pack.time_boot_ms_GET() == 3653278139L);
            assert(pack.xmag_GET() == (short)13638);
            assert(pack.ygyro_GET() == (short) -21306);
            assert(pack.xgyro_GET() == (short) -1000);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.zgyro_SET((short)22993) ;
        p129.ymag_SET((short)20807) ;
        p129.zacc_SET((short) -24868) ;
        p129.time_boot_ms_SET(3653278139L) ;
        p129.yacc_SET((short) -16531) ;
        p129.xacc_SET((short)10766) ;
        p129.zmag_SET((short)21085) ;
        p129.xgyro_SET((short) -1000) ;
        p129.xmag_SET((short)13638) ;
        p129.ygyro_SET((short) -21306) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)171);
            assert(pack.width_GET() == (char)44797);
            assert(pack.jpg_quality_GET() == (char)78);
            assert(pack.height_GET() == (char)49988);
            assert(pack.payload_GET() == (char)253);
            assert(pack.packets_GET() == (char)10759);
            assert(pack.size_GET() == 1623347117L);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.height_SET((char)49988) ;
        p130.payload_SET((char)253) ;
        p130.jpg_quality_SET((char)78) ;
        p130.packets_SET((char)10759) ;
        p130.width_SET((char)44797) ;
        p130.type_SET((char)171) ;
        p130.size_SET(1623347117L) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)8907);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)77, (char)49, (char)88, (char)18, (char)242, (char)239, (char)46, (char)178, (char)161, (char)164, (char)175, (char)27, (char)32, (char)130, (char)22, (char)242, (char)45, (char)67, (char)42, (char)87, (char)161, (char)137, (char)190, (char)161, (char)245, (char)115, (char)235, (char)12, (char)2, (char)92, (char)84, (char)56, (char)2, (char)242, (char)125, (char)130, (char)20, (char)119, (char)201, (char)156, (char)160, (char)69, (char)10, (char)48, (char)65, (char)106, (char)156, (char)151, (char)27, (char)236, (char)5, (char)7, (char)88, (char)76, (char)64, (char)158, (char)222, (char)64, (char)181, (char)154, (char)122, (char)251, (char)222, (char)8, (char)118, (char)130, (char)25, (char)80, (char)30, (char)244, (char)181, (char)124, (char)40, (char)6, (char)104, (char)66, (char)95, (char)59, (char)210, (char)8, (char)129, (char)29, (char)52, (char)200, (char)172, (char)217, (char)130, (char)220, (char)37, (char)30, (char)125, (char)171, (char)61, (char)153, (char)218, (char)250, (char)43, (char)65, (char)135, (char)96, (char)251, (char)26, (char)68, (char)213, (char)199, (char)155, (char)176, (char)139, (char)74, (char)167, (char)197, (char)64, (char)42, (char)116, (char)150, (char)7, (char)12, (char)15, (char)206, (char)78, (char)247, (char)113, (char)121, (char)137, (char)30, (char)9, (char)252, (char)89, (char)79, (char)109, (char)25, (char)105, (char)22, (char)252, (char)128, (char)72, (char)228, (char)38, (char)180, (char)78, (char)89, (char)220, (char)226, (char)129, (char)209, (char)77, (char)160, (char)169, (char)206, (char)148, (char)171, (char)146, (char)145, (char)245, (char)186, (char)131, (char)68, (char)42, (char)42, (char)207, (char)55, (char)148, (char)189, (char)137, (char)243, (char)92, (char)73, (char)233, (char)68, (char)200, (char)1, (char)106, (char)219, (char)120, (char)122, (char)238, (char)38, (char)229, (char)117, (char)160, (char)231, (char)207, (char)61, (char)146, (char)95, (char)10, (char)216, (char)96, (char)146, (char)245, (char)24, (char)84, (char)89, (char)9, (char)110, (char)182, (char)167, (char)28, (char)216, (char)41, (char)22, (char)176, (char)255, (char)78, (char)111, (char)49, (char)166, (char)151, (char)20, (char)87, (char)129, (char)85, (char)160, (char)216, (char)76, (char)141, (char)59, (char)174, (char)214, (char)122, (char)203, (char)145, (char)176, (char)71, (char)145, (char)44, (char)21, (char)227, (char)106, (char)201, (char)252, (char)34, (char)245, (char)249, (char)212, (char)160, (char)158, (char)163, (char)183, (char)240, (char)41, (char)85, (char)140, (char)225, (char)155, (char)184, (char)14, (char)16, (char)4, (char)17, (char)41, (char)153, (char)162}));
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.data__SET(new char[] {(char)77, (char)49, (char)88, (char)18, (char)242, (char)239, (char)46, (char)178, (char)161, (char)164, (char)175, (char)27, (char)32, (char)130, (char)22, (char)242, (char)45, (char)67, (char)42, (char)87, (char)161, (char)137, (char)190, (char)161, (char)245, (char)115, (char)235, (char)12, (char)2, (char)92, (char)84, (char)56, (char)2, (char)242, (char)125, (char)130, (char)20, (char)119, (char)201, (char)156, (char)160, (char)69, (char)10, (char)48, (char)65, (char)106, (char)156, (char)151, (char)27, (char)236, (char)5, (char)7, (char)88, (char)76, (char)64, (char)158, (char)222, (char)64, (char)181, (char)154, (char)122, (char)251, (char)222, (char)8, (char)118, (char)130, (char)25, (char)80, (char)30, (char)244, (char)181, (char)124, (char)40, (char)6, (char)104, (char)66, (char)95, (char)59, (char)210, (char)8, (char)129, (char)29, (char)52, (char)200, (char)172, (char)217, (char)130, (char)220, (char)37, (char)30, (char)125, (char)171, (char)61, (char)153, (char)218, (char)250, (char)43, (char)65, (char)135, (char)96, (char)251, (char)26, (char)68, (char)213, (char)199, (char)155, (char)176, (char)139, (char)74, (char)167, (char)197, (char)64, (char)42, (char)116, (char)150, (char)7, (char)12, (char)15, (char)206, (char)78, (char)247, (char)113, (char)121, (char)137, (char)30, (char)9, (char)252, (char)89, (char)79, (char)109, (char)25, (char)105, (char)22, (char)252, (char)128, (char)72, (char)228, (char)38, (char)180, (char)78, (char)89, (char)220, (char)226, (char)129, (char)209, (char)77, (char)160, (char)169, (char)206, (char)148, (char)171, (char)146, (char)145, (char)245, (char)186, (char)131, (char)68, (char)42, (char)42, (char)207, (char)55, (char)148, (char)189, (char)137, (char)243, (char)92, (char)73, (char)233, (char)68, (char)200, (char)1, (char)106, (char)219, (char)120, (char)122, (char)238, (char)38, (char)229, (char)117, (char)160, (char)231, (char)207, (char)61, (char)146, (char)95, (char)10, (char)216, (char)96, (char)146, (char)245, (char)24, (char)84, (char)89, (char)9, (char)110, (char)182, (char)167, (char)28, (char)216, (char)41, (char)22, (char)176, (char)255, (char)78, (char)111, (char)49, (char)166, (char)151, (char)20, (char)87, (char)129, (char)85, (char)160, (char)216, (char)76, (char)141, (char)59, (char)174, (char)214, (char)122, (char)203, (char)145, (char)176, (char)71, (char)145, (char)44, (char)21, (char)227, (char)106, (char)201, (char)252, (char)34, (char)245, (char)249, (char)212, (char)160, (char)158, (char)163, (char)183, (char)240, (char)41, (char)85, (char)140, (char)225, (char)155, (char)184, (char)14, (char)16, (char)4, (char)17, (char)41, (char)153, (char)162}, 0) ;
        p131.seqnr_SET((char)8907) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)54);
            assert(pack.current_distance_GET() == (char)29524);
            assert(pack.min_distance_GET() == (char)31442);
            assert(pack.max_distance_GET() == (char)59019);
            assert(pack.time_boot_ms_GET() == 4156786617L);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_180_YAW_90);
            assert(pack.covariance_GET() == (char)62);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.id_SET((char)54) ;
        p132.covariance_SET((char)62) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_180_YAW_90) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p132.max_distance_SET((char)59019) ;
        p132.time_boot_ms_SET(4156786617L) ;
        p132.current_distance_SET((char)29524) ;
        p132.min_distance_SET((char)31442) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mask_GET() == 6833415422671952049L);
            assert(pack.grid_spacing_GET() == (char)42608);
            assert(pack.lon_GET() == 1591895243);
            assert(pack.lat_GET() == -2004456015);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(-2004456015) ;
        p133.grid_spacing_SET((char)42608) ;
        p133.mask_SET(6833415422671952049L) ;
        p133.lon_SET(1591895243) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.gridbit_GET() == (char)212);
            assert(pack.lat_GET() == -1707503022);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -9385, (short)21959, (short) -12873, (short) -423, (short) -15411, (short)11582, (short) -12246, (short)941, (short)4554, (short)1292, (short) -18966, (short)18811, (short) -29121, (short)28318, (short) -13014, (short)20498}));
            assert(pack.grid_spacing_GET() == (char)17496);
            assert(pack.lon_GET() == -36663554);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.gridbit_SET((char)212) ;
        p134.data__SET(new short[] {(short) -9385, (short)21959, (short) -12873, (short) -423, (short) -15411, (short)11582, (short) -12246, (short)941, (short)4554, (short)1292, (short) -18966, (short)18811, (short) -29121, (short)28318, (short) -13014, (short)20498}, 0) ;
        p134.lat_SET(-1707503022) ;
        p134.grid_spacing_SET((char)17496) ;
        p134.lon_SET(-36663554) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -589770364);
            assert(pack.lat_GET() == -73749092);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(-589770364) ;
        p135.lat_SET(-73749092) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.spacing_GET() == (char)54371);
            assert(pack.current_height_GET() == -2.5267288E38F);
            assert(pack.lon_GET() == -867334801);
            assert(pack.lat_GET() == 2024653130);
            assert(pack.loaded_GET() == (char)8072);
            assert(pack.pending_GET() == (char)58517);
            assert(pack.terrain_height_GET() == -1.8253043E38F);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(2024653130) ;
        p136.loaded_SET((char)8072) ;
        p136.terrain_height_SET(-1.8253043E38F) ;
        p136.lon_SET(-867334801) ;
        p136.pending_SET((char)58517) ;
        p136.current_height_SET(-2.5267288E38F) ;
        p136.spacing_SET((char)54371) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -22531);
            assert(pack.press_abs_GET() == -2.293542E38F);
            assert(pack.time_boot_ms_GET() == 3923800299L);
            assert(pack.press_diff_GET() == -5.155567E37F);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(3923800299L) ;
        p137.temperature_SET((short) -22531) ;
        p137.press_diff_SET(-5.155567E37F) ;
        p137.press_abs_SET(-2.293542E38F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.3022223E38F, -8.046825E37F, -2.192296E38F, -2.637354E37F}));
            assert(pack.y_GET() == 4.315932E37F);
            assert(pack.x_GET() == -1.1730649E38F);
            assert(pack.time_usec_GET() == 2453667873072306230L);
            assert(pack.z_GET() == -8.3961184E37F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.z_SET(-8.3961184E37F) ;
        p138.x_SET(-1.1730649E38F) ;
        p138.q_SET(new float[] {-1.3022223E38F, -8.046825E37F, -2.192296E38F, -2.637354E37F}, 0) ;
        p138.y_SET(4.315932E37F) ;
        p138.time_usec_SET(2453667873072306230L) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)101);
            assert(pack.target_system_GET() == (char)41);
            assert(pack.time_usec_GET() == 7026974956379692206L);
            assert(pack.target_component_GET() == (char)80);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-5.896302E37F, -1.1289203E38F, 2.5344873E38F, -3.0855453E38F, 3.3052046E38F, 2.6719296E38F, 2.3049873E37F, -1.3190109E38F}));
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.controls_SET(new float[] {-5.896302E37F, -1.1289203E38F, 2.5344873E38F, -3.0855453E38F, 3.3052046E38F, 2.6719296E38F, 2.3049873E37F, -1.3190109E38F}, 0) ;
        p139.group_mlx_SET((char)101) ;
        p139.target_system_SET((char)41) ;
        p139.target_component_SET((char)80) ;
        p139.time_usec_SET(7026974956379692206L) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.690159E38F, -2.0039477E38F, -1.4006221E38F, 2.8236146E38F, 2.954769E38F, -2.9915481E38F, -3.0726402E38F, 2.1109708E37F}));
            assert(pack.time_usec_GET() == 2113027070005077432L);
            assert(pack.group_mlx_GET() == (char)136);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)136) ;
        p140.time_usec_SET(2113027070005077432L) ;
        p140.controls_SET(new float[] {2.690159E38F, -2.0039477E38F, -1.4006221E38F, 2.8236146E38F, 2.954769E38F, -2.9915481E38F, -3.0726402E38F, 2.1109708E37F}, 0) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_local_GET() == -8.787191E37F);
            assert(pack.altitude_monotonic_GET() == 8.554276E36F);
            assert(pack.time_usec_GET() == 4878640204141341432L);
            assert(pack.bottom_clearance_GET() == -4.4747833E37F);
            assert(pack.altitude_amsl_GET() == 2.5270108E38F);
            assert(pack.altitude_terrain_GET() == 1.5291507E38F);
            assert(pack.altitude_relative_GET() == -3.046542E38F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.bottom_clearance_SET(-4.4747833E37F) ;
        p141.altitude_relative_SET(-3.046542E38F) ;
        p141.altitude_local_SET(-8.787191E37F) ;
        p141.time_usec_SET(4878640204141341432L) ;
        p141.altitude_monotonic_SET(8.554276E36F) ;
        p141.altitude_terrain_SET(1.5291507E38F) ;
        p141.altitude_amsl_SET(2.5270108E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.transfer_type_GET() == (char)198);
            assert(pack.request_id_GET() == (char)101);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)48, (char)140, (char)238, (char)80, (char)141, (char)239, (char)225, (char)53, (char)65, (char)255, (char)114, (char)3, (char)251, (char)194, (char)127, (char)104, (char)19, (char)195, (char)253, (char)173, (char)107, (char)200, (char)226, (char)146, (char)132, (char)107, (char)138, (char)50, (char)179, (char)33, (char)103, (char)52, (char)223, (char)110, (char)104, (char)51, (char)130, (char)121, (char)14, (char)232, (char)24, (char)145, (char)89, (char)28, (char)206, (char)74, (char)154, (char)220, (char)133, (char)170, (char)63, (char)207, (char)163, (char)35, (char)87, (char)208, (char)216, (char)218, (char)113, (char)206, (char)56, (char)172, (char)44, (char)139, (char)248, (char)11, (char)39, (char)83, (char)249, (char)138, (char)43, (char)201, (char)128, (char)128, (char)43, (char)239, (char)55, (char)224, (char)26, (char)86, (char)201, (char)215, (char)40, (char)236, (char)138, (char)177, (char)191, (char)145, (char)244, (char)95, (char)216, (char)140, (char)224, (char)84, (char)177, (char)131, (char)236, (char)83, (char)128, (char)71, (char)77, (char)174, (char)137, (char)199, (char)78, (char)100, (char)133, (char)36, (char)7, (char)182, (char)248, (char)174, (char)18, (char)202, (char)122, (char)66, (char)35, (char)254, (char)102, (char)213}));
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)70, (char)225, (char)207, (char)40, (char)101, (char)220, (char)194, (char)233, (char)135, (char)215, (char)189, (char)241, (char)155, (char)169, (char)61, (char)119, (char)121, (char)163, (char)248, (char)61, (char)10, (char)182, (char)210, (char)40, (char)39, (char)23, (char)218, (char)189, (char)119, (char)246, (char)127, (char)127, (char)163, (char)210, (char)110, (char)136, (char)218, (char)227, (char)31, (char)34, (char)251, (char)56, (char)11, (char)84, (char)42, (char)144, (char)147, (char)15, (char)164, (char)54, (char)102, (char)197, (char)78, (char)162, (char)227, (char)16, (char)23, (char)63, (char)111, (char)21, (char)162, (char)142, (char)27, (char)210, (char)132, (char)204, (char)19, (char)97, (char)71, (char)108, (char)174, (char)109, (char)122, (char)57, (char)146, (char)159, (char)137, (char)245, (char)148, (char)144, (char)191, (char)146, (char)133, (char)108, (char)194, (char)152, (char)51, (char)78, (char)236, (char)215, (char)235, (char)167, (char)200, (char)88, (char)213, (char)128, (char)68, (char)251, (char)216, (char)80, (char)12, (char)96, (char)101, (char)16, (char)106, (char)155, (char)251, (char)71, (char)124, (char)126, (char)62, (char)125, (char)32, (char)33, (char)186, (char)141, (char)40, (char)149, (char)163, (char)55}));
            assert(pack.uri_type_GET() == (char)179);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_SET(new char[] {(char)70, (char)225, (char)207, (char)40, (char)101, (char)220, (char)194, (char)233, (char)135, (char)215, (char)189, (char)241, (char)155, (char)169, (char)61, (char)119, (char)121, (char)163, (char)248, (char)61, (char)10, (char)182, (char)210, (char)40, (char)39, (char)23, (char)218, (char)189, (char)119, (char)246, (char)127, (char)127, (char)163, (char)210, (char)110, (char)136, (char)218, (char)227, (char)31, (char)34, (char)251, (char)56, (char)11, (char)84, (char)42, (char)144, (char)147, (char)15, (char)164, (char)54, (char)102, (char)197, (char)78, (char)162, (char)227, (char)16, (char)23, (char)63, (char)111, (char)21, (char)162, (char)142, (char)27, (char)210, (char)132, (char)204, (char)19, (char)97, (char)71, (char)108, (char)174, (char)109, (char)122, (char)57, (char)146, (char)159, (char)137, (char)245, (char)148, (char)144, (char)191, (char)146, (char)133, (char)108, (char)194, (char)152, (char)51, (char)78, (char)236, (char)215, (char)235, (char)167, (char)200, (char)88, (char)213, (char)128, (char)68, (char)251, (char)216, (char)80, (char)12, (char)96, (char)101, (char)16, (char)106, (char)155, (char)251, (char)71, (char)124, (char)126, (char)62, (char)125, (char)32, (char)33, (char)186, (char)141, (char)40, (char)149, (char)163, (char)55}, 0) ;
        p142.request_id_SET((char)101) ;
        p142.transfer_type_SET((char)198) ;
        p142.storage_SET(new char[] {(char)48, (char)140, (char)238, (char)80, (char)141, (char)239, (char)225, (char)53, (char)65, (char)255, (char)114, (char)3, (char)251, (char)194, (char)127, (char)104, (char)19, (char)195, (char)253, (char)173, (char)107, (char)200, (char)226, (char)146, (char)132, (char)107, (char)138, (char)50, (char)179, (char)33, (char)103, (char)52, (char)223, (char)110, (char)104, (char)51, (char)130, (char)121, (char)14, (char)232, (char)24, (char)145, (char)89, (char)28, (char)206, (char)74, (char)154, (char)220, (char)133, (char)170, (char)63, (char)207, (char)163, (char)35, (char)87, (char)208, (char)216, (char)218, (char)113, (char)206, (char)56, (char)172, (char)44, (char)139, (char)248, (char)11, (char)39, (char)83, (char)249, (char)138, (char)43, (char)201, (char)128, (char)128, (char)43, (char)239, (char)55, (char)224, (char)26, (char)86, (char)201, (char)215, (char)40, (char)236, (char)138, (char)177, (char)191, (char)145, (char)244, (char)95, (char)216, (char)140, (char)224, (char)84, (char)177, (char)131, (char)236, (char)83, (char)128, (char)71, (char)77, (char)174, (char)137, (char)199, (char)78, (char)100, (char)133, (char)36, (char)7, (char)182, (char)248, (char)174, (char)18, (char)202, (char)122, (char)66, (char)35, (char)254, (char)102, (char)213}, 0) ;
        p142.uri_type_SET((char)179) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2770973016L);
            assert(pack.press_diff_GET() == -3.3132744E38F);
            assert(pack.press_abs_GET() == 2.442781E38F);
            assert(pack.temperature_GET() == (short)30296);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.press_diff_SET(-3.3132744E38F) ;
        p143.time_boot_ms_SET(2770973016L) ;
        p143.press_abs_SET(2.442781E38F) ;
        p143.temperature_SET((short)30296) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.rates_GET(),  new float[] {2.5496049E37F, -3.210991E38F, -3.3442688E38F}));
            assert(pack.alt_GET() == 1.2877342E37F);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-9.012616E36F, -3.5871662E37F, -2.8170332E38F}));
            assert(pack.lon_GET() == 265898292);
            assert(pack.est_capabilities_GET() == (char)43);
            assert(pack.lat_GET() == -365463031);
            assert(pack.custom_state_GET() == 6956356897636710235L);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-3.3619104E38F, 2.913689E38F, 1.7501077E38F, -6.6415166E37F}));
            assert(pack.timestamp_GET() == 5624286054397873046L);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {3.2746062E38F, 2.3760025E38F, -2.5499593E38F}));
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {2.8963194E38F, -1.0124698E38F, -5.1896505E37F}));
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.attitude_q_SET(new float[] {-3.3619104E38F, 2.913689E38F, 1.7501077E38F, -6.6415166E37F}, 0) ;
        p144.est_capabilities_SET((char)43) ;
        p144.timestamp_SET(5624286054397873046L) ;
        p144.acc_SET(new float[] {-9.012616E36F, -3.5871662E37F, -2.8170332E38F}, 0) ;
        p144.custom_state_SET(6956356897636710235L) ;
        p144.position_cov_SET(new float[] {2.8963194E38F, -1.0124698E38F, -5.1896505E37F}, 0) ;
        p144.lat_SET(-365463031) ;
        p144.vel_SET(new float[] {3.2746062E38F, 2.3760025E38F, -2.5499593E38F}, 0) ;
        p144.lon_SET(265898292) ;
        p144.rates_SET(new float[] {2.5496049E37F, -3.210991E38F, -3.3442688E38F}, 0) ;
        p144.alt_SET(1.2877342E37F) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.4384762E38F, -1.052359E38F, 3.3870066E38F, 2.0424283E38F}));
            assert(pack.x_pos_GET() == 2.142332E38F);
            assert(pack.z_vel_GET() == -2.981082E38F);
            assert(pack.time_usec_GET() == 5703541911813900932L);
            assert(pack.y_vel_GET() == 2.0654152E37F);
            assert(pack.z_acc_GET() == -2.6451144E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {-2.1693775E38F, -2.2006893E38F, -2.4741556E38F}));
            assert(pack.y_acc_GET() == 6.301934E37F);
            assert(pack.airspeed_GET() == 2.7309044E38F);
            assert(pack.z_pos_GET() == 4.0836292E37F);
            assert(pack.x_vel_GET() == 2.2248156E38F);
            assert(pack.y_pos_GET() == -2.4717595E38F);
            assert(pack.pitch_rate_GET() == 8.370063E37F);
            assert(pack.x_acc_GET() == 2.6393124E38F);
            assert(pack.yaw_rate_GET() == 2.898767E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-1.5339567E38F, 2.6501599E36F, 2.0679363E38F}));
            assert(pack.roll_rate_GET() == 1.4700873E38F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.z_vel_SET(-2.981082E38F) ;
        p146.pitch_rate_SET(8.370063E37F) ;
        p146.time_usec_SET(5703541911813900932L) ;
        p146.z_pos_SET(4.0836292E37F) ;
        p146.vel_variance_SET(new float[] {-1.5339567E38F, 2.6501599E36F, 2.0679363E38F}, 0) ;
        p146.pos_variance_SET(new float[] {-2.1693775E38F, -2.2006893E38F, -2.4741556E38F}, 0) ;
        p146.x_pos_SET(2.142332E38F) ;
        p146.x_acc_SET(2.6393124E38F) ;
        p146.x_vel_SET(2.2248156E38F) ;
        p146.q_SET(new float[] {1.4384762E38F, -1.052359E38F, 3.3870066E38F, 2.0424283E38F}, 0) ;
        p146.y_vel_SET(2.0654152E37F) ;
        p146.y_pos_SET(-2.4717595E38F) ;
        p146.y_acc_SET(6.301934E37F) ;
        p146.airspeed_SET(2.7309044E38F) ;
        p146.z_acc_SET(-2.6451144E38F) ;
        p146.roll_rate_SET(1.4700873E38F) ;
        p146.yaw_rate_SET(2.898767E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
            assert(pack.id_GET() == (char)87);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)13600, (char)59364, (char)50853, (char)20184, (char)62085, (char)8228, (char)5728, (char)47093, (char)58308, (char)51106}));
            assert(pack.temperature_GET() == (short)29906);
            assert(pack.battery_remaining_GET() == (byte)85);
            assert(pack.current_consumed_GET() == 224518125);
            assert(pack.current_battery_GET() == (short)31161);
            assert(pack.energy_consumed_GET() == 1587471559);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.energy_consumed_SET(1587471559) ;
        p147.current_consumed_SET(224518125) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL) ;
        p147.id_SET((char)87) ;
        p147.battery_remaining_SET((byte)85) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE) ;
        p147.current_battery_SET((short)31161) ;
        p147.voltages_SET(new char[] {(char)13600, (char)59364, (char)50853, (char)20184, (char)62085, (char)8228, (char)5728, (char)47093, (char)58308, (char)51106}, 0) ;
        p147.temperature_SET((short)29906) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.vendor_id_GET() == (char)38013);
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
            assert(pack.board_version_GET() == 1960762667L);
            assert(pack.middleware_sw_version_GET() == 600926988L);
            assert(pack.product_id_GET() == (char)17097);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)28, (char)233, (char)89, (char)105, (char)72, (char)167, (char)233, (char)61}));
            assert(pack.uid_GET() == 1111858583603189577L);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)19, (char)212, (char)24, (char)48, (char)26, (char)97, (char)51, (char)94}));
            assert(pack.flight_sw_version_GET() == 1503442781L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)149, (char)247, (char)31, (char)161, (char)27, (char)157, (char)33, (char)216}));
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)168, (char)180, (char)28, (char)17, (char)155, (char)254, (char)106, (char)74, (char)150, (char)24, (char)155, (char)38, (char)131, (char)16, (char)114, (char)153, (char)143, (char)108}));
            assert(pack.os_sw_version_GET() == 2193695269L);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.flight_custom_version_SET(new char[] {(char)149, (char)247, (char)31, (char)161, (char)27, (char)157, (char)33, (char)216}, 0) ;
        p148.flight_sw_version_SET(1503442781L) ;
        p148.uid_SET(1111858583603189577L) ;
        p148.product_id_SET((char)17097) ;
        p148.vendor_id_SET((char)38013) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE)) ;
        p148.os_sw_version_SET(2193695269L) ;
        p148.middleware_sw_version_SET(600926988L) ;
        p148.board_version_SET(1960762667L) ;
        p148.uid2_SET(new char[] {(char)168, (char)180, (char)28, (char)17, (char)155, (char)254, (char)106, (char)74, (char)150, (char)24, (char)155, (char)38, (char)131, (char)16, (char)114, (char)153, (char)143, (char)108}, 0, PH) ;
        p148.os_custom_version_SET(new char[] {(char)28, (char)233, (char)89, (char)105, (char)72, (char)167, (char)233, (char)61}, 0) ;
        p148.middleware_custom_version_SET(new char[] {(char)19, (char)212, (char)24, (char)48, (char)26, (char)97, (char)51, (char)94}, 0) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.position_valid_TRY(ph) == (char)96);
            assert(pack.angle_x_GET() == -2.4750975E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.target_num_GET() == (char)22);
            assert(pack.size_y_GET() == 1.9152996E38F);
            assert(pack.distance_GET() == -1.827815E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {2.931474E38F, -2.1093706E38F, 2.6314752E36F, 2.0000018E38F}));
            assert(pack.time_usec_GET() == 4977001315700285818L);
            assert(pack.z_TRY(ph) == 2.3847228E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
            assert(pack.y_TRY(ph) == 3.2973283E37F);
            assert(pack.x_TRY(ph) == 5.9598765E37F);
            assert(pack.angle_y_GET() == -1.4884984E37F);
            assert(pack.size_x_GET() == 2.5886875E38F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL) ;
        p149.x_SET(5.9598765E37F, PH) ;
        p149.z_SET(2.3847228E38F, PH) ;
        p149.y_SET(3.2973283E37F, PH) ;
        p149.position_valid_SET((char)96, PH) ;
        p149.size_x_SET(2.5886875E38F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p149.distance_SET(-1.827815E38F) ;
        p149.time_usec_SET(4977001315700285818L) ;
        p149.angle_x_SET(-2.4750975E38F) ;
        p149.angle_y_SET(-1.4884984E37F) ;
        p149.size_y_SET(1.9152996E38F) ;
        p149.q_SET(new float[] {2.931474E38F, -2.1093706E38F, 2.6314752E36F, 2.0000018E38F}, 0, PH) ;
        p149.target_num_SET((char)22) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_CPU_LOAD.add((src, ph, pack) ->
        {
            assert(pack.ctrlLoad_GET() == (char)71);
            assert(pack.sensLoad_GET() == (char)94);
            assert(pack.batVolt_GET() == (char)31150);
        });
        GroundControl.CPU_LOAD p170 = CommunicationChannel.new_CPU_LOAD();
        PH.setPack(p170);
        p170.batVolt_SET((char)31150) ;
        p170.sensLoad_SET((char)94) ;
        p170.ctrlLoad_SET((char)71) ;
        CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SENSOR_BIAS.add((src, ph, pack) ->
        {
            assert(pack.gxBias_GET() == -1.0610313E37F);
            assert(pack.azBias_GET() == 9.86951E37F);
            assert(pack.gzBias_GET() == 2.3216498E37F);
            assert(pack.axBias_GET() == 2.9425069E38F);
            assert(pack.ayBias_GET() == 5.9492186E37F);
            assert(pack.gyBias_GET() == -2.7831774E38F);
        });
        GroundControl.SENSOR_BIAS p172 = CommunicationChannel.new_SENSOR_BIAS();
        PH.setPack(p172);
        p172.axBias_SET(2.9425069E38F) ;
        p172.gzBias_SET(2.3216498E37F) ;
        p172.azBias_SET(9.86951E37F) ;
        p172.ayBias_SET(5.9492186E37F) ;
        p172.gyBias_SET(-2.7831774E38F) ;
        p172.gxBias_SET(-1.0610313E37F) ;
        CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DIAGNOSTIC.add((src, ph, pack) ->
        {
            assert(pack.diagSh2_GET() == (short) -30694);
            assert(pack.diagSh3_GET() == (short) -18078);
            assert(pack.diagFl3_GET() == -1.5659883E38F);
            assert(pack.diagFl1_GET() == -7.547675E37F);
            assert(pack.diagSh1_GET() == (short)19224);
            assert(pack.diagFl2_GET() == -2.0460655E38F);
        });
        GroundControl.DIAGNOSTIC p173 = CommunicationChannel.new_DIAGNOSTIC();
        PH.setPack(p173);
        p173.diagFl3_SET(-1.5659883E38F) ;
        p173.diagFl2_SET(-2.0460655E38F) ;
        p173.diagSh1_SET((short)19224) ;
        p173.diagFl1_SET(-7.547675E37F) ;
        p173.diagSh3_SET((short) -18078) ;
        p173.diagSh2_SET((short) -30694) ;
        CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SLUGS_NAVIGATION.add((src, ph, pack) ->
        {
            assert(pack.u_m_GET() == 2.5629287E38F);
            assert(pack.totalDist_GET() == -2.9772707E38F);
            assert(pack.psiDot_c_GET() == -2.549789E38F);
            assert(pack.theta_c_GET() == 1.0028218E38F);
            assert(pack.toWP_GET() == (char)53);
            assert(pack.fromWP_GET() == (char)149);
            assert(pack.phi_c_GET() == 2.2079472E38F);
            assert(pack.ay_body_GET() == -2.0795305E38F);
            assert(pack.dist2Go_GET() == 3.3593357E38F);
            assert(pack.h_c_GET() == (char)49431);
        });
        GroundControl.SLUGS_NAVIGATION p176 = CommunicationChannel.new_SLUGS_NAVIGATION();
        PH.setPack(p176);
        p176.toWP_SET((char)53) ;
        p176.fromWP_SET((char)149) ;
        p176.theta_c_SET(1.0028218E38F) ;
        p176.totalDist_SET(-2.9772707E38F) ;
        p176.h_c_SET((char)49431) ;
        p176.dist2Go_SET(3.3593357E38F) ;
        p176.ay_body_SET(-2.0795305E38F) ;
        p176.u_m_SET(2.5629287E38F) ;
        p176.psiDot_c_SET(-2.549789E38F) ;
        p176.phi_c_SET(2.2079472E38F) ;
        CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA_LOG.add((src, ph, pack) ->
        {
            assert(pack.fl_6_GET() == -5.6829907E37F);
            assert(pack.fl_2_GET() == 3.2067753E38F);
            assert(pack.fl_5_GET() == 2.560935E38F);
            assert(pack.fl_4_GET() == -1.6708069E38F);
            assert(pack.fl_1_GET() == -3.330013E38F);
            assert(pack.fl_3_GET() == -3.7894422E37F);
        });
        GroundControl.DATA_LOG p177 = CommunicationChannel.new_DATA_LOG();
        PH.setPack(p177);
        p177.fl_4_SET(-1.6708069E38F) ;
        p177.fl_3_SET(-3.7894422E37F) ;
        p177.fl_2_SET(3.2067753E38F) ;
        p177.fl_6_SET(-5.6829907E37F) ;
        p177.fl_1_SET(-3.330013E38F) ;
        p177.fl_5_SET(2.560935E38F) ;
        CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_DATE_TIME.add((src, ph, pack) ->
        {
            assert(pack.min_GET() == (char)57);
            assert(pack.day_GET() == (char)227);
            assert(pack.clockStat_GET() == (char)215);
            assert(pack.useSat_GET() == (char)193);
            assert(pack.year_GET() == (char)126);
            assert(pack.GppGl_GET() == (char)126);
            assert(pack.sigUsedMask_GET() == (char)111);
            assert(pack.hour_GET() == (char)188);
            assert(pack.sec_GET() == (char)226);
            assert(pack.month_GET() == (char)222);
            assert(pack.percentUsed_GET() == (char)100);
            assert(pack.visSat_GET() == (char)158);
        });
        GroundControl.GPS_DATE_TIME p179 = CommunicationChannel.new_GPS_DATE_TIME();
        PH.setPack(p179);
        p179.hour_SET((char)188) ;
        p179.sec_SET((char)226) ;
        p179.year_SET((char)126) ;
        p179.month_SET((char)222) ;
        p179.percentUsed_SET((char)100) ;
        p179.clockStat_SET((char)215) ;
        p179.visSat_SET((char)158) ;
        p179.min_SET((char)57) ;
        p179.day_SET((char)227) ;
        p179.GppGl_SET((char)126) ;
        p179.useSat_SET((char)193) ;
        p179.sigUsedMask_SET((char)111) ;
        CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MID_LVL_CMDS.add((src, ph, pack) ->
        {
            assert(pack.rCommand_GET() == 9.344874E37F);
            assert(pack.target_GET() == (char)131);
            assert(pack.hCommand_GET() == 1.6624344E38F);
            assert(pack.uCommand_GET() == 3.119433E37F);
        });
        GroundControl.MID_LVL_CMDS p180 = CommunicationChannel.new_MID_LVL_CMDS();
        PH.setPack(p180);
        p180.rCommand_SET(9.344874E37F) ;
        p180.uCommand_SET(3.119433E37F) ;
        p180.hCommand_SET(1.6624344E38F) ;
        p180.target_SET((char)131) ;
        CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CTRL_SRFC_PT.add((src, ph, pack) ->
        {
            assert(pack.target_GET() == (char)107);
            assert(pack.bitfieldPt_GET() == (char)28014);
        });
        GroundControl.CTRL_SRFC_PT p181 = CommunicationChannel.new_CTRL_SRFC_PT();
        PH.setPack(p181);
        p181.bitfieldPt_SET((char)28014) ;
        p181.target_SET((char)107) ;
        CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SLUGS_CAMERA_ORDER.add((src, ph, pack) ->
        {
            assert(pack.moveHome_GET() == (byte) - 44);
            assert(pack.target_GET() == (char)173);
            assert(pack.pan_GET() == (byte) - 9);
            assert(pack.zoom_GET() == (byte)97);
            assert(pack.tilt_GET() == (byte) - 118);
        });
        GroundControl.SLUGS_CAMERA_ORDER p184 = CommunicationChannel.new_SLUGS_CAMERA_ORDER();
        PH.setPack(p184);
        p184.tilt_SET((byte) - 118) ;
        p184.moveHome_SET((byte) - 44) ;
        p184.zoom_SET((byte)97) ;
        p184.pan_SET((byte) - 9) ;
        p184.target_SET((char)173) ;
        CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SURFACE.add((src, ph, pack) ->
        {
            assert(pack.mControl_GET() == 2.012172E37F);
            assert(pack.target_GET() == (char)18);
            assert(pack.bControl_GET() == -1.7136064E38F);
            assert(pack.idSurface_GET() == (char)108);
        });
        GroundControl.CONTROL_SURFACE p185 = CommunicationChannel.new_CONTROL_SURFACE();
        PH.setPack(p185);
        p185.idSurface_SET((char)108) ;
        p185.bControl_SET(-1.7136064E38F) ;
        p185.target_SET((char)18) ;
        p185.mControl_SET(2.012172E37F) ;
        CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SLUGS_MOBILE_LOCATION.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == 6.5778745E37F);
            assert(pack.latitude_GET() == 4.754845E37F);
            assert(pack.target_GET() == (char)67);
        });
        GroundControl.SLUGS_MOBILE_LOCATION p186 = CommunicationChannel.new_SLUGS_MOBILE_LOCATION();
        PH.setPack(p186);
        p186.longitude_SET(6.5778745E37F) ;
        p186.latitude_SET(4.754845E37F) ;
        p186.target_SET((char)67) ;
        CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SLUGS_CONFIGURATION_CAMERA.add((src, ph, pack) ->
        {
            assert(pack.order_GET() == (char)140);
            assert(pack.idOrder_GET() == (char)246);
            assert(pack.target_GET() == (char)207);
        });
        GroundControl.SLUGS_CONFIGURATION_CAMERA p188 = CommunicationChannel.new_SLUGS_CONFIGURATION_CAMERA();
        PH.setPack(p188);
        p188.idOrder_SET((char)246) ;
        p188.target_SET((char)207) ;
        p188.order_SET((char)140) ;
        CommunicationChannel.instance.send(p188);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ISR_LOCATION.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 5.248634E37F);
            assert(pack.target_GET() == (char)119);
            assert(pack.height_GET() == 3.0478708E38F);
            assert(pack.option2_GET() == (char)229);
            assert(pack.option1_GET() == (char)109);
            assert(pack.option3_GET() == (char)129);
            assert(pack.longitude_GET() == -2.8985612E38F);
        });
        GroundControl.ISR_LOCATION p189 = CommunicationChannel.new_ISR_LOCATION();
        PH.setPack(p189);
        p189.longitude_SET(-2.8985612E38F) ;
        p189.option1_SET((char)109) ;
        p189.option2_SET((char)229) ;
        p189.latitude_SET(5.248634E37F) ;
        p189.height_SET(3.0478708E38F) ;
        p189.target_SET((char)119) ;
        p189.option3_SET((char)129) ;
        CommunicationChannel.instance.send(p189);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VOLT_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.reading2_GET() == (char)6806);
            assert(pack.r2Type_GET() == (char)138);
            assert(pack.voltage_GET() == (char)46585);
        });
        GroundControl.VOLT_SENSOR p191 = CommunicationChannel.new_VOLT_SENSOR();
        PH.setPack(p191);
        p191.voltage_SET((char)46585) ;
        p191.r2Type_SET((char)138) ;
        p191.reading2_SET((char)6806) ;
        CommunicationChannel.instance.send(p191);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PTZ_STATUS.add((src, ph, pack) ->
        {
            assert(pack.zoom_GET() == (char)30);
            assert(pack.pan_GET() == (short)8786);
            assert(pack.tilt_GET() == (short)25110);
        });
        GroundControl.PTZ_STATUS p192 = CommunicationChannel.new_PTZ_STATUS();
        PH.setPack(p192);
        p192.pan_SET((short)8786) ;
        p192.zoom_SET((char)30) ;
        p192.tilt_SET((short)25110) ;
        CommunicationChannel.instance.send(p192);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAV_STATUS.add((src, ph, pack) ->
        {
            assert(pack.target_GET() == (char)118);
            assert(pack.longitude_GET() == -1.750718E38F);
            assert(pack.altitude_GET() == 1.6425827E38F);
            assert(pack.course_GET() == 3.7298398E37F);
            assert(pack.latitude_GET() == 2.0832655E38F);
            assert(pack.speed_GET() == 3.3032074E38F);
        });
        GroundControl.UAV_STATUS p193 = CommunicationChannel.new_UAV_STATUS();
        PH.setPack(p193);
        p193.longitude_SET(-1.750718E38F) ;
        p193.target_SET((char)118) ;
        p193.altitude_SET(1.6425827E38F) ;
        p193.speed_SET(3.3032074E38F) ;
        p193.course_SET(3.7298398E37F) ;
        p193.latitude_SET(2.0832655E38F) ;
        CommunicationChannel.instance.send(p193);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUS_GPS.add((src, ph, pack) ->
        {
            assert(pack.magDir_GET() == (byte) - 68);
            assert(pack.msgsType_GET() == (char)30);
            assert(pack.posStatus_GET() == (char)236);
            assert(pack.gpsQuality_GET() == (char)78);
            assert(pack.magVar_GET() == -1.4530497E38F);
            assert(pack.csFails_GET() == (char)9079);
            assert(pack.modeInd_GET() == (char)170);
        });
        GroundControl.STATUS_GPS p194 = CommunicationChannel.new_STATUS_GPS();
        PH.setPack(p194);
        p194.magDir_SET((byte) - 68) ;
        p194.modeInd_SET((char)170) ;
        p194.posStatus_SET((char)236) ;
        p194.magVar_SET(-1.4530497E38F) ;
        p194.msgsType_SET((char)30) ;
        p194.csFails_SET((char)9079) ;
        p194.gpsQuality_SET((char)78) ;
        CommunicationChannel.instance.send(p194);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NOVATEL_DIAG.add((src, ph, pack) ->
        {
            assert(pack.csFails_GET() == (char)45521);
            assert(pack.solStatus_GET() == (char)103);
            assert(pack.velType_GET() == (char)85);
            assert(pack.receiverStatus_GET() == 1799156418L);
            assert(pack.posType_GET() == (char)221);
            assert(pack.posSolAge_GET() == -1.2567302E38F);
            assert(pack.timeStatus_GET() == (char)136);
        });
        GroundControl.NOVATEL_DIAG p195 = CommunicationChannel.new_NOVATEL_DIAG();
        PH.setPack(p195);
        p195.posSolAge_SET(-1.2567302E38F) ;
        p195.csFails_SET((char)45521) ;
        p195.solStatus_SET((char)103) ;
        p195.velType_SET((char)85) ;
        p195.posType_SET((char)221) ;
        p195.timeStatus_SET((char)136) ;
        p195.receiverStatus_SET(1799156418L) ;
        CommunicationChannel.instance.send(p195);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SENSOR_DIAG.add((src, ph, pack) ->
        {
            assert(pack.char1_GET() == (byte)39);
            assert(pack.float1_GET() == 2.2868228E38F);
            assert(pack.float2_GET() == -7.2008253E37F);
            assert(pack.int1_GET() == (short) -3780);
        });
        GroundControl.SENSOR_DIAG p196 = CommunicationChannel.new_SENSOR_DIAG();
        PH.setPack(p196);
        p196.char1_SET((byte)39) ;
        p196.int1_SET((short) -3780) ;
        p196.float1_SET(2.2868228E38F) ;
        p196.float2_SET(-7.2008253E37F) ;
        CommunicationChannel.instance.send(p196);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BOOT.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == 382484654L);
        });
        GroundControl.BOOT p197 = CommunicationChannel.new_BOOT();
        PH.setPack(p197);
        p197.version_SET(382484654L) ;
        CommunicationChannel.instance.send(p197);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        CommunicationChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pos_horiz_accuracy_GET() == -6.496892E37F);
            assert(pack.time_usec_GET() == 842134790173930532L);
            assert(pack.pos_horiz_ratio_GET() == 2.1267891E38F);
            assert(pack.mag_ratio_GET() == -2.31188E38F);
            assert(pack.pos_vert_ratio_GET() == 2.0317697E38F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS));
            assert(pack.hagl_ratio_GET() == -1.2554872E38F);
            assert(pack.vel_ratio_GET() == 3.3371663E37F);
            assert(pack.tas_ratio_GET() == -2.4433754E38F);
            assert(pack.pos_vert_accuracy_GET() == -6.3613126E37F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.vel_ratio_SET(3.3371663E37F) ;
        p230.tas_ratio_SET(-2.4433754E38F) ;
        p230.time_usec_SET(842134790173930532L) ;
        p230.hagl_ratio_SET(-1.2554872E38F) ;
        p230.pos_horiz_ratio_SET(2.1267891E38F) ;
        p230.pos_vert_ratio_SET(2.0317697E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS)) ;
        p230.pos_horiz_accuracy_SET(-6.496892E37F) ;
        p230.pos_vert_accuracy_SET(-6.3613126E37F) ;
        p230.mag_ratio_SET(-2.31188E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_y_GET() == -1.2271441E38F);
            assert(pack.horiz_accuracy_GET() == -2.1263813E38F);
            assert(pack.vert_accuracy_GET() == 3.0219837E38F);
            assert(pack.var_horiz_GET() == -2.8377746E38F);
            assert(pack.time_usec_GET() == 7119851686645500645L);
            assert(pack.wind_x_GET() == -1.2112703E37F);
            assert(pack.wind_alt_GET() == 1.6219422E38F);
            assert(pack.var_vert_GET() == 1.5650613E38F);
            assert(pack.wind_z_GET() == -5.938276E36F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_z_SET(-5.938276E36F) ;
        p231.wind_alt_SET(1.6219422E38F) ;
        p231.var_vert_SET(1.5650613E38F) ;
        p231.var_horiz_SET(-2.8377746E38F) ;
        p231.wind_x_SET(-1.2112703E37F) ;
        p231.time_usec_SET(7119851686645500645L) ;
        p231.horiz_accuracy_SET(-2.1263813E38F) ;
        p231.vert_accuracy_SET(3.0219837E38F) ;
        p231.wind_y_SET(-1.2271441E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.gps_id_GET() == (char)101);
            assert(pack.time_week_GET() == (char)52442);
            assert(pack.alt_GET() == 3.3767778E38F);
            assert(pack.vd_GET() == 1.4649709E38F);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ));
            assert(pack.hdop_GET() == 1.1836443E38F);
            assert(pack.lon_GET() == 1046329564);
            assert(pack.time_week_ms_GET() == 932219551L);
            assert(pack.vdop_GET() == 5.0514766E37F);
            assert(pack.fix_type_GET() == (char)163);
            assert(pack.speed_accuracy_GET() == 2.8600082E38F);
            assert(pack.satellites_visible_GET() == (char)66);
            assert(pack.lat_GET() == -1894640345);
            assert(pack.time_usec_GET() == 8805865948106421876L);
            assert(pack.vert_accuracy_GET() == -1.538054E38F);
            assert(pack.vn_GET() == 3.27705E38F);
            assert(pack.horiz_accuracy_GET() == -3.3606306E38F);
            assert(pack.ve_GET() == 1.6757352E38F);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.horiz_accuracy_SET(-3.3606306E38F) ;
        p232.vert_accuracy_SET(-1.538054E38F) ;
        p232.alt_SET(3.3767778E38F) ;
        p232.gps_id_SET((char)101) ;
        p232.vd_SET(1.4649709E38F) ;
        p232.satellites_visible_SET((char)66) ;
        p232.vdop_SET(5.0514766E37F) ;
        p232.hdop_SET(1.1836443E38F) ;
        p232.lon_SET(1046329564) ;
        p232.fix_type_SET((char)163) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ)) ;
        p232.ve_SET(1.6757352E38F) ;
        p232.vn_SET(3.27705E38F) ;
        p232.speed_accuracy_SET(2.8600082E38F) ;
        p232.lat_SET(-1894640345) ;
        p232.time_week_ms_SET(932219551L) ;
        p232.time_usec_SET(8805865948106421876L) ;
        p232.time_week_SET((char)52442) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)235, (char)38, (char)121, (char)95, (char)171, (char)203, (char)48, (char)223, (char)3, (char)170, (char)76, (char)235, (char)156, (char)243, (char)73, (char)66, (char)156, (char)87, (char)122, (char)158, (char)237, (char)62, (char)188, (char)129, (char)35, (char)214, (char)46, (char)106, (char)142, (char)253, (char)202, (char)1, (char)50, (char)148, (char)213, (char)9, (char)66, (char)250, (char)72, (char)237, (char)77, (char)218, (char)185, (char)53, (char)4, (char)14, (char)28, (char)115, (char)111, (char)217, (char)203, (char)20, (char)134, (char)156, (char)165, (char)174, (char)189, (char)60, (char)203, (char)24, (char)95, (char)38, (char)229, (char)63, (char)237, (char)122, (char)232, (char)176, (char)172, (char)230, (char)133, (char)169, (char)90, (char)151, (char)104, (char)79, (char)173, (char)155, (char)101, (char)239, (char)180, (char)162, (char)137, (char)204, (char)8, (char)207, (char)221, (char)226, (char)117, (char)183, (char)253, (char)163, (char)234, (char)76, (char)5, (char)21, (char)186, (char)236, (char)130, (char)170, (char)42, (char)176, (char)242, (char)210, (char)109, (char)86, (char)232, (char)157, (char)181, (char)102, (char)126, (char)152, (char)251, (char)241, (char)17, (char)146, (char)233, (char)240, (char)156, (char)245, (char)201, (char)52, (char)177, (char)215, (char)189, (char)101, (char)74, (char)198, (char)85, (char)32, (char)233, (char)162, (char)73, (char)149, (char)91, (char)138, (char)55, (char)97, (char)80, (char)89, (char)231, (char)3, (char)95, (char)59, (char)111, (char)204, (char)100, (char)90, (char)175, (char)134, (char)232, (char)143, (char)201, (char)128, (char)249, (char)50, (char)106, (char)244, (char)195, (char)46, (char)183, (char)160, (char)123, (char)145, (char)175, (char)231, (char)124, (char)0, (char)112, (char)224, (char)252, (char)177, (char)71, (char)85, (char)89, (char)164, (char)35, (char)26, (char)80, (char)1}));
            assert(pack.len_GET() == (char)220);
            assert(pack.flags_GET() == (char)132);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.len_SET((char)220) ;
        p233.data__SET(new char[] {(char)235, (char)38, (char)121, (char)95, (char)171, (char)203, (char)48, (char)223, (char)3, (char)170, (char)76, (char)235, (char)156, (char)243, (char)73, (char)66, (char)156, (char)87, (char)122, (char)158, (char)237, (char)62, (char)188, (char)129, (char)35, (char)214, (char)46, (char)106, (char)142, (char)253, (char)202, (char)1, (char)50, (char)148, (char)213, (char)9, (char)66, (char)250, (char)72, (char)237, (char)77, (char)218, (char)185, (char)53, (char)4, (char)14, (char)28, (char)115, (char)111, (char)217, (char)203, (char)20, (char)134, (char)156, (char)165, (char)174, (char)189, (char)60, (char)203, (char)24, (char)95, (char)38, (char)229, (char)63, (char)237, (char)122, (char)232, (char)176, (char)172, (char)230, (char)133, (char)169, (char)90, (char)151, (char)104, (char)79, (char)173, (char)155, (char)101, (char)239, (char)180, (char)162, (char)137, (char)204, (char)8, (char)207, (char)221, (char)226, (char)117, (char)183, (char)253, (char)163, (char)234, (char)76, (char)5, (char)21, (char)186, (char)236, (char)130, (char)170, (char)42, (char)176, (char)242, (char)210, (char)109, (char)86, (char)232, (char)157, (char)181, (char)102, (char)126, (char)152, (char)251, (char)241, (char)17, (char)146, (char)233, (char)240, (char)156, (char)245, (char)201, (char)52, (char)177, (char)215, (char)189, (char)101, (char)74, (char)198, (char)85, (char)32, (char)233, (char)162, (char)73, (char)149, (char)91, (char)138, (char)55, (char)97, (char)80, (char)89, (char)231, (char)3, (char)95, (char)59, (char)111, (char)204, (char)100, (char)90, (char)175, (char)134, (char)232, (char)143, (char)201, (char)128, (char)249, (char)50, (char)106, (char)244, (char)195, (char)46, (char)183, (char)160, (char)123, (char)145, (char)175, (char)231, (char)124, (char)0, (char)112, (char)224, (char)252, (char)177, (char)71, (char)85, (char)89, (char)164, (char)35, (char)26, (char)80, (char)1}, 0) ;
        p233.flags_SET((char)132) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            assert(pack.wp_distance_GET() == (char)13956);
            assert(pack.climb_rate_GET() == (byte)90);
            assert(pack.temperature_GET() == (byte)17);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.gps_nsat_GET() == (char)108);
            assert(pack.altitude_sp_GET() == (short) -17630);
            assert(pack.throttle_GET() == (byte) - 59);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED));
            assert(pack.altitude_amsl_GET() == (short) -6471);
            assert(pack.battery_remaining_GET() == (char)244);
            assert(pack.pitch_GET() == (short)11937);
            assert(pack.custom_mode_GET() == 632019649L);
            assert(pack.airspeed_sp_GET() == (char)202);
            assert(pack.heading_sp_GET() == (short)17044);
            assert(pack.heading_GET() == (char)48104);
            assert(pack.airspeed_GET() == (char)142);
            assert(pack.roll_GET() == (short)15922);
            assert(pack.wp_num_GET() == (char)250);
            assert(pack.failsafe_GET() == (char)121);
            assert(pack.groundspeed_GET() == (char)124);
            assert(pack.latitude_GET() == -921970020);
            assert(pack.temperature_air_GET() == (byte)78);
            assert(pack.longitude_GET() == 799225579);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.temperature_air_SET((byte)78) ;
        p234.climb_rate_SET((byte)90) ;
        p234.altitude_amsl_SET((short) -6471) ;
        p234.pitch_SET((short)11937) ;
        p234.airspeed_sp_SET((char)202) ;
        p234.temperature_SET((byte)17) ;
        p234.roll_SET((short)15922) ;
        p234.custom_mode_SET(632019649L) ;
        p234.altitude_sp_SET((short) -17630) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND) ;
        p234.wp_distance_SET((char)13956) ;
        p234.groundspeed_SET((char)124) ;
        p234.longitude_SET(799225579) ;
        p234.throttle_SET((byte) - 59) ;
        p234.gps_nsat_SET((char)108) ;
        p234.wp_num_SET((char)250) ;
        p234.heading_SET((char)48104) ;
        p234.heading_sp_SET((short)17044) ;
        p234.airspeed_SET((char)142) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED)) ;
        p234.battery_remaining_SET((char)244) ;
        p234.latitude_SET(-921970020) ;
        p234.failsafe_SET((char)121) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_z_GET() == -2.4724831E38F);
            assert(pack.vibration_y_GET() == 5.1231004E37F);
            assert(pack.vibration_x_GET() == 3.1340024E38F);
            assert(pack.clipping_0_GET() == 1975577122L);
            assert(pack.time_usec_GET() == 589975720634882684L);
            assert(pack.clipping_2_GET() == 491570075L);
            assert(pack.clipping_1_GET() == 758090315L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_2_SET(491570075L) ;
        p241.vibration_z_SET(-2.4724831E38F) ;
        p241.vibration_y_SET(5.1231004E37F) ;
        p241.time_usec_SET(589975720634882684L) ;
        p241.clipping_1_SET(758090315L) ;
        p241.clipping_0_SET(1975577122L) ;
        p241.vibration_x_SET(3.1340024E38F) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == -907022364);
            assert(pack.approach_x_GET() == 1.3428065E38F);
            assert(pack.altitude_GET() == 550090951);
            assert(pack.y_GET() == 1.6006573E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.0297778E38F, -2.3446332E38F, -3.2565875E38F, -2.1699836E38F}));
            assert(pack.time_usec_TRY(ph) == 8261287003795598942L);
            assert(pack.approach_y_GET() == -2.062537E38F);
            assert(pack.z_GET() == -8.2148003E37F);
            assert(pack.approach_z_GET() == -3.3770956E38F);
            assert(pack.latitude_GET() == 1605210455);
            assert(pack.x_GET() == 2.7669166E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.y_SET(1.6006573E38F) ;
        p242.approach_y_SET(-2.062537E38F) ;
        p242.altitude_SET(550090951) ;
        p242.longitude_SET(-907022364) ;
        p242.x_SET(2.7669166E38F) ;
        p242.q_SET(new float[] {3.0297778E38F, -2.3446332E38F, -3.2565875E38F, -2.1699836E38F}, 0) ;
        p242.time_usec_SET(8261287003795598942L, PH) ;
        p242.approach_z_SET(-3.3770956E38F) ;
        p242.latitude_SET(1605210455) ;
        p242.approach_x_SET(1.3428065E38F) ;
        p242.z_SET(-8.2148003E37F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 1183265542406686901L);
            assert(pack.altitude_GET() == -110577394);
            assert(pack.target_system_GET() == (char)127);
            assert(pack.approach_z_GET() == 3.9537302E37F);
            assert(pack.longitude_GET() == -87290811);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.2044059E38F, 1.370282E38F, -8.4035763E37F, 3.1107065E38F}));
            assert(pack.y_GET() == 2.6156698E38F);
            assert(pack.z_GET() == -3.0087058E38F);
            assert(pack.x_GET() == 2.7770584E38F);
            assert(pack.approach_y_GET() == -1.2136687E38F);
            assert(pack.latitude_GET() == 2111977436);
            assert(pack.approach_x_GET() == 1.1771499E37F);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.longitude_SET(-87290811) ;
        p243.y_SET(2.6156698E38F) ;
        p243.time_usec_SET(1183265542406686901L, PH) ;
        p243.approach_z_SET(3.9537302E37F) ;
        p243.approach_x_SET(1.1771499E37F) ;
        p243.approach_y_SET(-1.2136687E38F) ;
        p243.target_system_SET((char)127) ;
        p243.z_SET(-3.0087058E38F) ;
        p243.x_SET(2.7770584E38F) ;
        p243.q_SET(new float[] {-2.2044059E38F, 1.370282E38F, -8.4035763E37F, 3.1107065E38F}, 0) ;
        p243.altitude_SET(-110577394) ;
        p243.latitude_SET(2111977436) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)23628);
            assert(pack.interval_us_GET() == -1177521670);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)23628) ;
        p244.interval_us_SET(-1177521670) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING) ;
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.tslc_GET() == (char)221);
            assert(pack.lon_GET() == -1797962013);
            assert(pack.callsign_LEN(ph) == 5);
            assert(pack.callsign_TRY(ph).equals("qXvRe"));
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.hor_velocity_GET() == (char)22737);
            assert(pack.lat_GET() == -1765704520);
            assert(pack.ICAO_address_GET() == 3317311426L);
            assert(pack.heading_GET() == (char)22935);
            assert(pack.squawk_GET() == (char)41178);
            assert(pack.altitude_GET() == -599406717);
            assert(pack.ver_velocity_GET() == (short) -32106);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGHLY_MANUV);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED));
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.tslc_SET((char)221) ;
        p246.hor_velocity_SET((char)22737) ;
        p246.callsign_SET("qXvRe", PH) ;
        p246.squawk_SET((char)41178) ;
        p246.ICAO_address_SET(3317311426L) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED)) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.lon_SET(-1797962013) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGHLY_MANUV) ;
        p246.altitude_SET(-599406717) ;
        p246.ver_velocity_SET((short) -32106) ;
        p246.lat_SET(-1765704520) ;
        p246.heading_SET((char)22935) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.time_to_minimum_delta_GET() == 1.1319588E38F);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            assert(pack.id_GET() == 2065038786L);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
            assert(pack.horizontal_minimum_delta_GET() == 2.6036485E38F);
            assert(pack.altitude_minimum_delta_GET() == -1.7165608E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE) ;
        p247.altitude_minimum_delta_SET(-1.7165608E38F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.time_to_minimum_delta_SET(1.1319588E38F) ;
        p247.id_SET(2065038786L) ;
        p247.horizontal_minimum_delta_SET(2.6036485E38F) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)20);
            assert(pack.target_network_GET() == (char)76);
            assert(pack.target_component_GET() == (char)145);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)79, (char)117, (char)190, (char)195, (char)145, (char)226, (char)235, (char)232, (char)221, (char)27, (char)12, (char)10, (char)190, (char)254, (char)157, (char)247, (char)184, (char)189, (char)106, (char)13, (char)22, (char)218, (char)196, (char)146, (char)37, (char)55, (char)54, (char)84, (char)69, (char)128, (char)10, (char)241, (char)123, (char)96, (char)61, (char)184, (char)221, (char)125, (char)8, (char)79, (char)239, (char)63, (char)82, (char)178, (char)74, (char)245, (char)60, (char)128, (char)29, (char)92, (char)14, (char)212, (char)71, (char)27, (char)44, (char)88, (char)217, (char)70, (char)124, (char)224, (char)252, (char)91, (char)229, (char)63, (char)216, (char)92, (char)171, (char)113, (char)10, (char)179, (char)205, (char)183, (char)213, (char)42, (char)120, (char)149, (char)140, (char)59, (char)89, (char)43, (char)231, (char)78, (char)121, (char)64, (char)126, (char)149, (char)77, (char)108, (char)4, (char)108, (char)251, (char)144, (char)19, (char)52, (char)116, (char)223, (char)241, (char)218, (char)143, (char)217, (char)146, (char)234, (char)229, (char)106, (char)229, (char)158, (char)16, (char)106, (char)26, (char)62, (char)85, (char)2, (char)160, (char)190, (char)243, (char)28, (char)185, (char)137, (char)213, (char)183, (char)107, (char)242, (char)245, (char)147, (char)247, (char)40, (char)181, (char)65, (char)71, (char)127, (char)216, (char)108, (char)230, (char)144, (char)230, (char)75, (char)21, (char)142, (char)74, (char)61, (char)63, (char)230, (char)71, (char)71, (char)43, (char)175, (char)242, (char)138, (char)62, (char)183, (char)8, (char)164, (char)153, (char)221, (char)36, (char)209, (char)210, (char)168, (char)241, (char)164, (char)188, (char)76, (char)94, (char)141, (char)115, (char)122, (char)58, (char)134, (char)136, (char)153, (char)170, (char)150, (char)179, (char)105, (char)204, (char)80, (char)171, (char)162, (char)150, (char)46, (char)169, (char)153, (char)145, (char)117, (char)7, (char)19, (char)37, (char)121, (char)173, (char)69, (char)23, (char)20, (char)104, (char)3, (char)196, (char)23, (char)157, (char)116, (char)105, (char)170, (char)126, (char)150, (char)74, (char)14, (char)243, (char)20, (char)172, (char)188, (char)220, (char)47, (char)237, (char)64, (char)138, (char)86, (char)181, (char)147, (char)32, (char)205, (char)6, (char)46, (char)241, (char)137, (char)80, (char)45, (char)2, (char)97, (char)236, (char)155, (char)162, (char)74, (char)94, (char)22, (char)1, (char)52, (char)53, (char)234, (char)18, (char)91, (char)130, (char)146, (char)77, (char)77, (char)212, (char)111, (char)182, (char)96, (char)8, (char)144, (char)111}));
            assert(pack.message_type_GET() == (char)50379);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_system_SET((char)20) ;
        p248.target_component_SET((char)145) ;
        p248.payload_SET(new char[] {(char)79, (char)117, (char)190, (char)195, (char)145, (char)226, (char)235, (char)232, (char)221, (char)27, (char)12, (char)10, (char)190, (char)254, (char)157, (char)247, (char)184, (char)189, (char)106, (char)13, (char)22, (char)218, (char)196, (char)146, (char)37, (char)55, (char)54, (char)84, (char)69, (char)128, (char)10, (char)241, (char)123, (char)96, (char)61, (char)184, (char)221, (char)125, (char)8, (char)79, (char)239, (char)63, (char)82, (char)178, (char)74, (char)245, (char)60, (char)128, (char)29, (char)92, (char)14, (char)212, (char)71, (char)27, (char)44, (char)88, (char)217, (char)70, (char)124, (char)224, (char)252, (char)91, (char)229, (char)63, (char)216, (char)92, (char)171, (char)113, (char)10, (char)179, (char)205, (char)183, (char)213, (char)42, (char)120, (char)149, (char)140, (char)59, (char)89, (char)43, (char)231, (char)78, (char)121, (char)64, (char)126, (char)149, (char)77, (char)108, (char)4, (char)108, (char)251, (char)144, (char)19, (char)52, (char)116, (char)223, (char)241, (char)218, (char)143, (char)217, (char)146, (char)234, (char)229, (char)106, (char)229, (char)158, (char)16, (char)106, (char)26, (char)62, (char)85, (char)2, (char)160, (char)190, (char)243, (char)28, (char)185, (char)137, (char)213, (char)183, (char)107, (char)242, (char)245, (char)147, (char)247, (char)40, (char)181, (char)65, (char)71, (char)127, (char)216, (char)108, (char)230, (char)144, (char)230, (char)75, (char)21, (char)142, (char)74, (char)61, (char)63, (char)230, (char)71, (char)71, (char)43, (char)175, (char)242, (char)138, (char)62, (char)183, (char)8, (char)164, (char)153, (char)221, (char)36, (char)209, (char)210, (char)168, (char)241, (char)164, (char)188, (char)76, (char)94, (char)141, (char)115, (char)122, (char)58, (char)134, (char)136, (char)153, (char)170, (char)150, (char)179, (char)105, (char)204, (char)80, (char)171, (char)162, (char)150, (char)46, (char)169, (char)153, (char)145, (char)117, (char)7, (char)19, (char)37, (char)121, (char)173, (char)69, (char)23, (char)20, (char)104, (char)3, (char)196, (char)23, (char)157, (char)116, (char)105, (char)170, (char)126, (char)150, (char)74, (char)14, (char)243, (char)20, (char)172, (char)188, (char)220, (char)47, (char)237, (char)64, (char)138, (char)86, (char)181, (char)147, (char)32, (char)205, (char)6, (char)46, (char)241, (char)137, (char)80, (char)45, (char)2, (char)97, (char)236, (char)155, (char)162, (char)74, (char)94, (char)22, (char)1, (char)52, (char)53, (char)234, (char)18, (char)91, (char)130, (char)146, (char)77, (char)77, (char)212, (char)111, (char)182, (char)96, (char)8, (char)144, (char)111}, 0) ;
        p248.message_type_SET((char)50379) ;
        p248.target_network_SET((char)76) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.address_GET() == (char)39972);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)111, (byte)11, (byte) - 109, (byte)107, (byte)46, (byte) - 114, (byte) - 30, (byte) - 9, (byte)11, (byte)78, (byte) - 76, (byte) - 69, (byte)15, (byte) - 47, (byte) - 18, (byte)94, (byte) - 101, (byte) - 119, (byte) - 46, (byte) - 59, (byte)125, (byte) - 102, (byte)52, (byte) - 99, (byte) - 101, (byte) - 108, (byte)83, (byte) - 47, (byte)82, (byte) - 62, (byte) - 82, (byte) - 41}));
            assert(pack.ver_GET() == (char)208);
            assert(pack.type_GET() == (char)252);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.type_SET((char)252) ;
        p249.address_SET((char)39972) ;
        p249.value_SET(new byte[] {(byte)111, (byte)11, (byte) - 109, (byte)107, (byte)46, (byte) - 114, (byte) - 30, (byte) - 9, (byte)11, (byte)78, (byte) - 76, (byte) - 69, (byte)15, (byte) - 47, (byte) - 18, (byte)94, (byte) - 101, (byte) - 119, (byte) - 46, (byte) - 59, (byte)125, (byte) - 102, (byte)52, (byte) - 99, (byte) - 101, (byte) - 108, (byte)83, (byte) - 47, (byte)82, (byte) - 62, (byte) - 82, (byte) - 41}, 0) ;
        p249.ver_SET((char)208) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("dyiovkoJty"));
            assert(pack.time_usec_GET() == 9163342808561346399L);
            assert(pack.x_GET() == -2.5923106E38F);
            assert(pack.y_GET() == 1.688532E38F);
            assert(pack.z_GET() == -1.1131676E38F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.y_SET(1.688532E38F) ;
        p250.x_SET(-2.5923106E38F) ;
        p250.z_SET(-1.1131676E38F) ;
        p250.name_SET("dyiovkoJty", PH) ;
        p250.time_usec_SET(9163342808561346399L) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 550805857L);
            assert(pack.value_GET() == -3.389445E38F);
            assert(pack.name_LEN(ph) == 1);
            assert(pack.name_TRY(ph).equals("d"));
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("d", PH) ;
        p251.time_boot_ms_SET(550805857L) ;
        p251.value_SET(-3.389445E38F) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3239415273L);
            assert(pack.value_GET() == 841316638);
            assert(pack.name_LEN(ph) == 9);
            assert(pack.name_TRY(ph).equals("vyxdzClju"));
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.value_SET(841316638) ;
        p252.name_SET("vyxdzClju", PH) ;
        p252.time_boot_ms_SET(3239415273L) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 18);
            assert(pack.text_TRY(ph).equals("spcybflmchuvfkKbsl"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_NOTICE);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("spcybflmchuvfkKbsl", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_NOTICE) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2484456099L);
            assert(pack.value_GET() == 2.8914632E38F);
            assert(pack.ind_GET() == (char)175);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.value_SET(2.8914632E38F) ;
        p254.time_boot_ms_SET(2484456099L) ;
        p254.ind_SET((char)175) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)180, (char)233, (char)142, (char)178, (char)218, (char)176, (char)113, (char)92, (char)237, (char)13, (char)104, (char)73, (char)112, (char)6, (char)224, (char)8, (char)220, (char)0, (char)214, (char)133, (char)255, (char)66, (char)17, (char)201, (char)236, (char)181, (char)52, (char)206, (char)197, (char)142, (char)234, (char)3}));
            assert(pack.initial_timestamp_GET() == 527727543181066163L);
            assert(pack.target_system_GET() == (char)163);
            assert(pack.target_component_GET() == (char)95);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)163) ;
        p256.secret_key_SET(new char[] {(char)180, (char)233, (char)142, (char)178, (char)218, (char)176, (char)113, (char)92, (char)237, (char)13, (char)104, (char)73, (char)112, (char)6, (char)224, (char)8, (char)220, (char)0, (char)214, (char)133, (char)255, (char)66, (char)17, (char)201, (char)236, (char)181, (char)52, (char)206, (char)197, (char)142, (char)234, (char)3}, 0) ;
        p256.target_component_SET((char)95) ;
        p256.initial_timestamp_SET(527727543181066163L) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.state_GET() == (char)167);
            assert(pack.time_boot_ms_GET() == 1000615876L);
            assert(pack.last_change_ms_GET() == 301417662L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.last_change_ms_SET(301417662L) ;
        p257.state_SET((char)167) ;
        p257.time_boot_ms_SET(1000615876L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.tune_LEN(ph) == 20);
            assert(pack.tune_TRY(ph).equals("blbkupzkpjyZwqvodwsq"));
            assert(pack.target_component_GET() == (char)251);
            assert(pack.target_system_GET() == (char)130);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.tune_SET("blbkupzkpjyZwqvodwsq", PH) ;
        p258.target_system_SET((char)130) ;
        p258.target_component_SET((char)251) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.sensor_size_v_GET() == 2.3502943E36F);
            assert(pack.cam_definition_version_GET() == (char)19464);
            assert(pack.resolution_h_GET() == (char)25329);
            assert(pack.time_boot_ms_GET() == 2922735871L);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)157, (char)139, (char)224, (char)61, (char)216, (char)57, (char)163, (char)167, (char)39, (char)197, (char)234, (char)151, (char)116, (char)127, (char)237, (char)92, (char)52, (char)25, (char)52, (char)118, (char)31, (char)251, (char)220, (char)26, (char)7, (char)208, (char)50, (char)54, (char)55, (char)222, (char)228, (char)34}));
            assert(pack.lens_id_GET() == (char)59);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)204, (char)211, (char)169, (char)59, (char)69, (char)107, (char)128, (char)143, (char)194, (char)215, (char)145, (char)93, (char)151, (char)81, (char)182, (char)27, (char)94, (char)198, (char)133, (char)177, (char)227, (char)209, (char)37, (char)93, (char)233, (char)148, (char)168, (char)119, (char)141, (char)145, (char)101, (char)16}));
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE));
            assert(pack.focal_length_GET() == -1.4707415E38F);
            assert(pack.sensor_size_h_GET() == 2.1184642E38F);
            assert(pack.resolution_v_GET() == (char)24322);
            assert(pack.cam_definition_uri_LEN(ph) == 5);
            assert(pack.cam_definition_uri_TRY(ph).equals("fbzab"));
            assert(pack.firmware_version_GET() == 4114837297L);
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.lens_id_SET((char)59) ;
        p259.cam_definition_version_SET((char)19464) ;
        p259.time_boot_ms_SET(2922735871L) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE)) ;
        p259.firmware_version_SET(4114837297L) ;
        p259.sensor_size_h_SET(2.1184642E38F) ;
        p259.model_name_SET(new char[] {(char)204, (char)211, (char)169, (char)59, (char)69, (char)107, (char)128, (char)143, (char)194, (char)215, (char)145, (char)93, (char)151, (char)81, (char)182, (char)27, (char)94, (char)198, (char)133, (char)177, (char)227, (char)209, (char)37, (char)93, (char)233, (char)148, (char)168, (char)119, (char)141, (char)145, (char)101, (char)16}, 0) ;
        p259.focal_length_SET(-1.4707415E38F) ;
        p259.resolution_h_SET((char)25329) ;
        p259.resolution_v_SET((char)24322) ;
        p259.vendor_name_SET(new char[] {(char)157, (char)139, (char)224, (char)61, (char)216, (char)57, (char)163, (char)167, (char)39, (char)197, (char)234, (char)151, (char)116, (char)127, (char)237, (char)92, (char)52, (char)25, (char)52, (char)118, (char)31, (char)251, (char)220, (char)26, (char)7, (char)208, (char)50, (char)54, (char)55, (char)222, (char)228, (char)34}, 0) ;
        p259.cam_definition_uri_SET("fbzab", PH) ;
        p259.sensor_size_v_SET(2.3502943E36F) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE);
            assert(pack.time_boot_ms_GET() == 1176607185L);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(1176607185L) ;
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.storage_count_GET() == (char)207);
            assert(pack.write_speed_GET() == 1.523192E38F);
            assert(pack.time_boot_ms_GET() == 253856688L);
            assert(pack.read_speed_GET() == 1.1427582E38F);
            assert(pack.status_GET() == (char)18);
            assert(pack.total_capacity_GET() == -2.259767E38F);
            assert(pack.used_capacity_GET() == 8.3719256E37F);
            assert(pack.available_capacity_GET() == 8.4113723E37F);
            assert(pack.storage_id_GET() == (char)218);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.status_SET((char)18) ;
        p261.storage_count_SET((char)207) ;
        p261.total_capacity_SET(-2.259767E38F) ;
        p261.write_speed_SET(1.523192E38F) ;
        p261.available_capacity_SET(8.4113723E37F) ;
        p261.storage_id_SET((char)218) ;
        p261.time_boot_ms_SET(253856688L) ;
        p261.used_capacity_SET(8.3719256E37F) ;
        p261.read_speed_SET(1.1427582E38F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3522442158L);
            assert(pack.recording_time_ms_GET() == 1529379639L);
            assert(pack.image_status_GET() == (char)57);
            assert(pack.image_interval_GET() == -2.9342166E38F);
            assert(pack.available_capacity_GET() == 1.06263575E37F);
            assert(pack.video_status_GET() == (char)103);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.image_interval_SET(-2.9342166E38F) ;
        p262.recording_time_ms_SET(1529379639L) ;
        p262.image_status_SET((char)57) ;
        p262.available_capacity_SET(1.06263575E37F) ;
        p262.time_boot_ms_SET(3522442158L) ;
        p262.video_status_SET((char)103) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1354677405L);
            assert(pack.file_url_LEN(ph) == 46);
            assert(pack.file_url_TRY(ph).equals("svRxNxouurpnWjwloncSbbgdpilnHtiwuPopqrbqFsRpgw"));
            assert(pack.relative_alt_GET() == 2119930497);
            assert(pack.time_utc_GET() == 3455935181559681888L);
            assert(pack.lon_GET() == 391626647);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.1702856E38F, 1.2210791E38F, 7.5992773E37F, 5.1353525E37F}));
            assert(pack.alt_GET() == 581289705);
            assert(pack.image_index_GET() == -1334545068);
            assert(pack.camera_id_GET() == (char)161);
            assert(pack.capture_result_GET() == (byte)46);
            assert(pack.lat_GET() == 153876129);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.relative_alt_SET(2119930497) ;
        p263.image_index_SET(-1334545068) ;
        p263.file_url_SET("svRxNxouurpnWjwloncSbbgdpilnHtiwuPopqrbqFsRpgw", PH) ;
        p263.q_SET(new float[] {-3.1702856E38F, 1.2210791E38F, 7.5992773E37F, 5.1353525E37F}, 0) ;
        p263.alt_SET(581289705) ;
        p263.capture_result_SET((byte)46) ;
        p263.time_boot_ms_SET(1354677405L) ;
        p263.lat_SET(153876129) ;
        p263.time_utc_SET(3455935181559681888L) ;
        p263.camera_id_SET((char)161) ;
        p263.lon_SET(391626647) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3185219739L);
            assert(pack.takeoff_time_utc_GET() == 4421103957465599340L);
            assert(pack.arming_time_utc_GET() == 8498290126874078641L);
            assert(pack.flight_uuid_GET() == 8476028044582260170L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.takeoff_time_utc_SET(4421103957465599340L) ;
        p264.arming_time_utc_SET(8498290126874078641L) ;
        p264.flight_uuid_SET(8476028044582260170L) ;
        p264.time_boot_ms_SET(3185219739L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 3.1884558E38F);
            assert(pack.roll_GET() == 1.2866681E37F);
            assert(pack.yaw_GET() == 3.3810974E38F);
            assert(pack.time_boot_ms_GET() == 1639301450L);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(1639301450L) ;
        p265.pitch_SET(3.1884558E38F) ;
        p265.yaw_SET(3.3810974E38F) ;
        p265.roll_SET(1.2866681E37F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.first_message_offset_GET() == (char)73);
            assert(pack.target_system_GET() == (char)205);
            assert(pack.length_GET() == (char)15);
            assert(pack.sequence_GET() == (char)45242);
            assert(pack.target_component_GET() == (char)89);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)103, (char)97, (char)54, (char)185, (char)117, (char)159, (char)164, (char)50, (char)17, (char)136, (char)91, (char)43, (char)0, (char)242, (char)139, (char)133, (char)198, (char)99, (char)43, (char)66, (char)199, (char)144, (char)89, (char)95, (char)250, (char)254, (char)119, (char)145, (char)209, (char)228, (char)92, (char)208, (char)76, (char)72, (char)92, (char)228, (char)137, (char)205, (char)163, (char)8, (char)16, (char)58, (char)4, (char)55, (char)83, (char)228, (char)240, (char)28, (char)219, (char)3, (char)60, (char)127, (char)217, (char)142, (char)168, (char)77, (char)117, (char)119, (char)246, (char)235, (char)17, (char)85, (char)50, (char)220, (char)230, (char)87, (char)152, (char)49, (char)91, (char)109, (char)175, (char)74, (char)13, (char)11, (char)60, (char)54, (char)18, (char)207, (char)66, (char)110, (char)126, (char)144, (char)71, (char)245, (char)153, (char)179, (char)251, (char)24, (char)195, (char)88, (char)96, (char)185, (char)30, (char)161, (char)90, (char)230, (char)70, (char)101, (char)249, (char)240, (char)126, (char)200, (char)111, (char)22, (char)231, (char)62, (char)79, (char)252, (char)71, (char)247, (char)128, (char)127, (char)215, (char)168, (char)150, (char)89, (char)131, (char)38, (char)89, (char)253, (char)124, (char)202, (char)182, (char)237, (char)32, (char)208, (char)128, (char)136, (char)54, (char)127, (char)247, (char)210, (char)81, (char)162, (char)182, (char)242, (char)239, (char)192, (char)189, (char)15, (char)46, (char)231, (char)94, (char)66, (char)151, (char)55, (char)125, (char)67, (char)92, (char)159, (char)181, (char)149, (char)204, (char)231, (char)28, (char)222, (char)202, (char)192, (char)36, (char)247, (char)217, (char)191, (char)70, (char)5, (char)151, (char)182, (char)153, (char)167, (char)47, (char)47, (char)156, (char)249, (char)223, (char)10, (char)164, (char)129, (char)151, (char)250, (char)65, (char)80, (char)236, (char)215, (char)152, (char)41, (char)78, (char)82, (char)163, (char)215, (char)143, (char)57, (char)253, (char)136, (char)41, (char)76, (char)11, (char)33, (char)140, (char)142, (char)135, (char)11, (char)5, (char)54, (char)163, (char)188, (char)12, (char)219, (char)134, (char)105, (char)240, (char)63, (char)173, (char)171, (char)152, (char)167, (char)248, (char)164, (char)170, (char)30, (char)16, (char)54, (char)220, (char)161, (char)7, (char)38, (char)81, (char)18, (char)46, (char)97, (char)231, (char)187, (char)101, (char)41, (char)213, (char)230, (char)232, (char)139, (char)166, (char)218, (char)81, (char)87, (char)154, (char)157, (char)162, (char)96, (char)227, (char)182, (char)87, (char)131, (char)3}));
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.sequence_SET((char)45242) ;
        p266.first_message_offset_SET((char)73) ;
        p266.length_SET((char)15) ;
        p266.data__SET(new char[] {(char)103, (char)97, (char)54, (char)185, (char)117, (char)159, (char)164, (char)50, (char)17, (char)136, (char)91, (char)43, (char)0, (char)242, (char)139, (char)133, (char)198, (char)99, (char)43, (char)66, (char)199, (char)144, (char)89, (char)95, (char)250, (char)254, (char)119, (char)145, (char)209, (char)228, (char)92, (char)208, (char)76, (char)72, (char)92, (char)228, (char)137, (char)205, (char)163, (char)8, (char)16, (char)58, (char)4, (char)55, (char)83, (char)228, (char)240, (char)28, (char)219, (char)3, (char)60, (char)127, (char)217, (char)142, (char)168, (char)77, (char)117, (char)119, (char)246, (char)235, (char)17, (char)85, (char)50, (char)220, (char)230, (char)87, (char)152, (char)49, (char)91, (char)109, (char)175, (char)74, (char)13, (char)11, (char)60, (char)54, (char)18, (char)207, (char)66, (char)110, (char)126, (char)144, (char)71, (char)245, (char)153, (char)179, (char)251, (char)24, (char)195, (char)88, (char)96, (char)185, (char)30, (char)161, (char)90, (char)230, (char)70, (char)101, (char)249, (char)240, (char)126, (char)200, (char)111, (char)22, (char)231, (char)62, (char)79, (char)252, (char)71, (char)247, (char)128, (char)127, (char)215, (char)168, (char)150, (char)89, (char)131, (char)38, (char)89, (char)253, (char)124, (char)202, (char)182, (char)237, (char)32, (char)208, (char)128, (char)136, (char)54, (char)127, (char)247, (char)210, (char)81, (char)162, (char)182, (char)242, (char)239, (char)192, (char)189, (char)15, (char)46, (char)231, (char)94, (char)66, (char)151, (char)55, (char)125, (char)67, (char)92, (char)159, (char)181, (char)149, (char)204, (char)231, (char)28, (char)222, (char)202, (char)192, (char)36, (char)247, (char)217, (char)191, (char)70, (char)5, (char)151, (char)182, (char)153, (char)167, (char)47, (char)47, (char)156, (char)249, (char)223, (char)10, (char)164, (char)129, (char)151, (char)250, (char)65, (char)80, (char)236, (char)215, (char)152, (char)41, (char)78, (char)82, (char)163, (char)215, (char)143, (char)57, (char)253, (char)136, (char)41, (char)76, (char)11, (char)33, (char)140, (char)142, (char)135, (char)11, (char)5, (char)54, (char)163, (char)188, (char)12, (char)219, (char)134, (char)105, (char)240, (char)63, (char)173, (char)171, (char)152, (char)167, (char)248, (char)164, (char)170, (char)30, (char)16, (char)54, (char)220, (char)161, (char)7, (char)38, (char)81, (char)18, (char)46, (char)97, (char)231, (char)187, (char)101, (char)41, (char)213, (char)230, (char)232, (char)139, (char)166, (char)218, (char)81, (char)87, (char)154, (char)157, (char)162, (char)96, (char)227, (char)182, (char)87, (char)131, (char)3}, 0) ;
        p266.target_system_SET((char)205) ;
        p266.target_component_SET((char)89) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)77, (char)134, (char)204, (char)58, (char)180, (char)11, (char)128, (char)141, (char)144, (char)213, (char)30, (char)229, (char)51, (char)95, (char)7, (char)136, (char)149, (char)113, (char)210, (char)118, (char)228, (char)36, (char)69, (char)39, (char)167, (char)182, (char)248, (char)226, (char)142, (char)36, (char)55, (char)47, (char)162, (char)253, (char)223, (char)178, (char)213, (char)184, (char)178, (char)147, (char)52, (char)241, (char)218, (char)122, (char)23, (char)200, (char)80, (char)9, (char)81, (char)34, (char)114, (char)60, (char)20, (char)233, (char)21, (char)156, (char)158, (char)106, (char)118, (char)235, (char)232, (char)91, (char)135, (char)88, (char)222, (char)20, (char)95, (char)241, (char)182, (char)189, (char)254, (char)49, (char)204, (char)95, (char)92, (char)242, (char)74, (char)92, (char)118, (char)175, (char)161, (char)178, (char)39, (char)218, (char)233, (char)134, (char)28, (char)89, (char)150, (char)177, (char)155, (char)63, (char)106, (char)217, (char)74, (char)192, (char)244, (char)164, (char)84, (char)136, (char)193, (char)10, (char)18, (char)64, (char)56, (char)91, (char)53, (char)172, (char)153, (char)70, (char)163, (char)218, (char)243, (char)213, (char)229, (char)212, (char)177, (char)62, (char)171, (char)196, (char)62, (char)18, (char)20, (char)214, (char)23, (char)201, (char)173, (char)206, (char)84, (char)67, (char)239, (char)164, (char)79, (char)225, (char)79, (char)148, (char)167, (char)33, (char)168, (char)223, (char)28, (char)105, (char)209, (char)179, (char)254, (char)231, (char)162, (char)157, (char)1, (char)107, (char)158, (char)252, (char)132, (char)160, (char)63, (char)23, (char)89, (char)163, (char)221, (char)220, (char)238, (char)47, (char)101, (char)26, (char)214, (char)92, (char)100, (char)126, (char)152, (char)171, (char)115, (char)3, (char)0, (char)244, (char)19, (char)173, (char)100, (char)181, (char)42, (char)162, (char)114, (char)95, (char)93, (char)39, (char)211, (char)103, (char)98, (char)81, (char)106, (char)252, (char)33, (char)54, (char)125, (char)198, (char)94, (char)116, (char)249, (char)170, (char)74, (char)103, (char)115, (char)30, (char)131, (char)115, (char)16, (char)12, (char)221, (char)63, (char)169, (char)43, (char)15, (char)153, (char)30, (char)5, (char)127, (char)27, (char)161, (char)202, (char)247, (char)245, (char)213, (char)190, (char)227, (char)13, (char)143, (char)251, (char)119, (char)207, (char)245, (char)239, (char)44, (char)1, (char)170, (char)118, (char)49, (char)44, (char)120, (char)61, (char)12, (char)254, (char)248, (char)4, (char)72, (char)178, (char)78, (char)74, (char)167, (char)215, (char)155}));
            assert(pack.target_system_GET() == (char)23);
            assert(pack.first_message_offset_GET() == (char)167);
            assert(pack.length_GET() == (char)105);
            assert(pack.sequence_GET() == (char)14237);
            assert(pack.target_component_GET() == (char)42);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.data__SET(new char[] {(char)77, (char)134, (char)204, (char)58, (char)180, (char)11, (char)128, (char)141, (char)144, (char)213, (char)30, (char)229, (char)51, (char)95, (char)7, (char)136, (char)149, (char)113, (char)210, (char)118, (char)228, (char)36, (char)69, (char)39, (char)167, (char)182, (char)248, (char)226, (char)142, (char)36, (char)55, (char)47, (char)162, (char)253, (char)223, (char)178, (char)213, (char)184, (char)178, (char)147, (char)52, (char)241, (char)218, (char)122, (char)23, (char)200, (char)80, (char)9, (char)81, (char)34, (char)114, (char)60, (char)20, (char)233, (char)21, (char)156, (char)158, (char)106, (char)118, (char)235, (char)232, (char)91, (char)135, (char)88, (char)222, (char)20, (char)95, (char)241, (char)182, (char)189, (char)254, (char)49, (char)204, (char)95, (char)92, (char)242, (char)74, (char)92, (char)118, (char)175, (char)161, (char)178, (char)39, (char)218, (char)233, (char)134, (char)28, (char)89, (char)150, (char)177, (char)155, (char)63, (char)106, (char)217, (char)74, (char)192, (char)244, (char)164, (char)84, (char)136, (char)193, (char)10, (char)18, (char)64, (char)56, (char)91, (char)53, (char)172, (char)153, (char)70, (char)163, (char)218, (char)243, (char)213, (char)229, (char)212, (char)177, (char)62, (char)171, (char)196, (char)62, (char)18, (char)20, (char)214, (char)23, (char)201, (char)173, (char)206, (char)84, (char)67, (char)239, (char)164, (char)79, (char)225, (char)79, (char)148, (char)167, (char)33, (char)168, (char)223, (char)28, (char)105, (char)209, (char)179, (char)254, (char)231, (char)162, (char)157, (char)1, (char)107, (char)158, (char)252, (char)132, (char)160, (char)63, (char)23, (char)89, (char)163, (char)221, (char)220, (char)238, (char)47, (char)101, (char)26, (char)214, (char)92, (char)100, (char)126, (char)152, (char)171, (char)115, (char)3, (char)0, (char)244, (char)19, (char)173, (char)100, (char)181, (char)42, (char)162, (char)114, (char)95, (char)93, (char)39, (char)211, (char)103, (char)98, (char)81, (char)106, (char)252, (char)33, (char)54, (char)125, (char)198, (char)94, (char)116, (char)249, (char)170, (char)74, (char)103, (char)115, (char)30, (char)131, (char)115, (char)16, (char)12, (char)221, (char)63, (char)169, (char)43, (char)15, (char)153, (char)30, (char)5, (char)127, (char)27, (char)161, (char)202, (char)247, (char)245, (char)213, (char)190, (char)227, (char)13, (char)143, (char)251, (char)119, (char)207, (char)245, (char)239, (char)44, (char)1, (char)170, (char)118, (char)49, (char)44, (char)120, (char)61, (char)12, (char)254, (char)248, (char)4, (char)72, (char)178, (char)78, (char)74, (char)167, (char)215, (char)155}, 0) ;
        p267.first_message_offset_SET((char)167) ;
        p267.length_SET((char)105) ;
        p267.sequence_SET((char)14237) ;
        p267.target_system_SET((char)23) ;
        p267.target_component_SET((char)42) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)31190);
            assert(pack.target_component_GET() == (char)101);
            assert(pack.target_system_GET() == (char)165);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)165) ;
        p268.target_component_SET((char)101) ;
        p268.sequence_SET((char)31190) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.rotation_GET() == (char)55020);
            assert(pack.camera_id_GET() == (char)162);
            assert(pack.status_GET() == (char)172);
            assert(pack.resolution_v_GET() == (char)63829);
            assert(pack.framerate_GET() == -3.0805526E38F);
            assert(pack.bitrate_GET() == 2899304451L);
            assert(pack.resolution_h_GET() == (char)48770);
            assert(pack.uri_LEN(ph) == 105);
            assert(pack.uri_TRY(ph).equals("NieguofhTytmCypvfnagkmtvynqqctPckqzofxkwvkecmpkysjdodezjobubpquweyjsQsxfzrQnwYhedkkwlsbnnxButpAywzitjzckw"));
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.resolution_v_SET((char)63829) ;
        p269.resolution_h_SET((char)48770) ;
        p269.bitrate_SET(2899304451L) ;
        p269.uri_SET("NieguofhTytmCypvfnagkmtvynqqctPckqzofxkwvkecmpkysjdodezjobubpquweyjsQsxfzrQnwYhedkkwlsbnnxButpAywzitjzckw", PH) ;
        p269.rotation_SET((char)55020) ;
        p269.framerate_SET(-3.0805526E38F) ;
        p269.camera_id_SET((char)162) ;
        p269.status_SET((char)172) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.framerate_GET() == 2.2145625E38F);
            assert(pack.bitrate_GET() == 2566954151L);
            assert(pack.resolution_v_GET() == (char)31875);
            assert(pack.resolution_h_GET() == (char)1198);
            assert(pack.target_component_GET() == (char)14);
            assert(pack.camera_id_GET() == (char)60);
            assert(pack.rotation_GET() == (char)33399);
            assert(pack.target_system_GET() == (char)212);
            assert(pack.uri_LEN(ph) == 4);
            assert(pack.uri_TRY(ph).equals("nhlq"));
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.bitrate_SET(2566954151L) ;
        p270.framerate_SET(2.2145625E38F) ;
        p270.rotation_SET((char)33399) ;
        p270.resolution_h_SET((char)1198) ;
        p270.uri_SET("nhlq", PH) ;
        p270.target_system_SET((char)212) ;
        p270.resolution_v_SET((char)31875) ;
        p270.camera_id_SET((char)60) ;
        p270.target_component_SET((char)14) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 18);
            assert(pack.ssid_TRY(ph).equals("wotjmyGvibiyfoPfok"));
            assert(pack.password_LEN(ph) == 50);
            assert(pack.password_TRY(ph).equals("psektjhtehqvqjjvtfEjjyttTdEepwniktaonywwzjEjywyxrx"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("psektjhtehqvqjjvtfEjjyttTdEepwniktaonywwzjEjywyxrx", PH) ;
        p299.ssid_SET("wotjmyGvibiyfoPfok", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == (char)3797);
            assert(pack.max_version_GET() == (char)288);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)70, (char)117, (char)52, (char)151, (char)219, (char)129, (char)25, (char)32}));
            assert(pack.min_version_GET() == (char)5590);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)158, (char)238, (char)90, (char)231, (char)26, (char)94, (char)84, (char)126}));
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.max_version_SET((char)288) ;
        p300.library_version_hash_SET(new char[] {(char)158, (char)238, (char)90, (char)231, (char)26, (char)94, (char)84, (char)126}, 0) ;
        p300.spec_version_hash_SET(new char[] {(char)70, (char)117, (char)52, (char)151, (char)219, (char)129, (char)25, (char)32}, 0) ;
        p300.version_SET((char)3797) ;
        p300.min_version_SET((char)5590) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.sub_mode_GET() == (char)250);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
            assert(pack.uptime_sec_GET() == 3227537012L);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
            assert(pack.time_usec_GET() == 8927874724103074166L);
            assert(pack.vendor_specific_status_code_GET() == (char)41337);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE) ;
        p310.vendor_specific_status_code_SET((char)41337) ;
        p310.uptime_sec_SET(3227537012L) ;
        p310.sub_mode_SET((char)250) ;
        p310.time_usec_SET(8927874724103074166L) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 57);
            assert(pack.name_TRY(ph).equals("XraffutcOjscdqeoromksylziGhLycdfidngwyTocnewimxvxxyAwbXmY"));
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)161, (char)200, (char)95, (char)125, (char)241, (char)63, (char)170, (char)200, (char)186, (char)83, (char)198, (char)183, (char)119, (char)210, (char)101, (char)88}));
            assert(pack.sw_vcs_commit_GET() == 2845746273L);
            assert(pack.sw_version_minor_GET() == (char)111);
            assert(pack.sw_version_major_GET() == (char)100);
            assert(pack.hw_version_major_GET() == (char)226);
            assert(pack.hw_version_minor_GET() == (char)107);
            assert(pack.time_usec_GET() == 3523949313059687285L);
            assert(pack.uptime_sec_GET() == 3662514994L);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_version_minor_SET((char)107) ;
        p311.hw_unique_id_SET(new char[] {(char)161, (char)200, (char)95, (char)125, (char)241, (char)63, (char)170, (char)200, (char)186, (char)83, (char)198, (char)183, (char)119, (char)210, (char)101, (char)88}, 0) ;
        p311.time_usec_SET(3523949313059687285L) ;
        p311.sw_version_minor_SET((char)111) ;
        p311.hw_version_major_SET((char)226) ;
        p311.name_SET("XraffutcOjscdqeoromksylziGhLycdfidngwyTocnewimxvxxyAwbXmY", PH) ;
        p311.uptime_sec_SET(3662514994L) ;
        p311.sw_vcs_commit_SET(2845746273L) ;
        p311.sw_version_major_SET((char)100) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short) -25385);
            assert(pack.target_system_GET() == (char)26);
            assert(pack.target_component_GET() == (char)29);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("uZvkhosh"));
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_index_SET((short) -25385) ;
        p320.target_component_SET((char)29) ;
        p320.param_id_SET("uZvkhosh", PH) ;
        p320.target_system_SET((char)26) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)115);
            assert(pack.target_system_GET() == (char)238);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)115) ;
        p321.target_system_SET((char)238) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 85);
            assert(pack.param_value_TRY(ph).equals("wqkKmzprusgfjnyrzanrgzbIodvysbewZgpdiqOraumadrqgvnilvdpgwokmIkcMgwcdkyUihckcjuwvhbqqr"));
            assert(pack.param_count_GET() == (char)7729);
            assert(pack.param_index_GET() == (char)28286);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("u"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_index_SET((char)28286) ;
        p322.param_id_SET("u", PH) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8) ;
        p322.param_count_SET((char)7729) ;
        p322.param_value_SET("wqkKmzprusgfjnyrzanrgzbIodvysbewZgpdiqOraumadrqgvnilvdpgwokmIkcMgwcdkyUihckcjuwvhbqqr", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)144);
            assert(pack.target_system_GET() == (char)225);
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("qaoeflayj"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
            assert(pack.param_value_LEN(ph) == 4);
            assert(pack.param_value_TRY(ph).equals("vuvm"));
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_value_SET("vuvm", PH) ;
        p323.param_id_SET("qaoeflayj", PH) ;
        p323.target_component_SET((char)144) ;
        p323.target_system_SET((char)225) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("CMPxr"));
            assert(pack.param_value_LEN(ph) == 50);
            assert(pack.param_value_TRY(ph).equals("xHqhscpkjIeaxmDgsncpUhpgclkglsskwmanzHqfsFrifqMigp"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_IN_PROGRESS);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("CMPxr", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_IN_PROGRESS) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64) ;
        p324.param_value_SET("xHqhscpkjIeaxmDgsncpUhpgclkglsskwmanzHqfsFrifqMigp", PH) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.min_distance_GET() == (char)64374);
            assert(pack.time_usec_GET() == 2289086108042488053L);
            assert(pack.max_distance_GET() == (char)9753);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)33465, (char)60447, (char)33311, (char)10103, (char)50415, (char)7308, (char)36448, (char)58847, (char)36005, (char)21576, (char)19056, (char)24587, (char)65090, (char)8353, (char)30580, (char)41348, (char)8867, (char)6395, (char)4832, (char)9444, (char)22562, (char)64764, (char)14919, (char)40940, (char)42307, (char)27134, (char)8909, (char)65126, (char)26581, (char)33920, (char)28520, (char)20201, (char)24999, (char)13686, (char)21350, (char)6384, (char)62962, (char)57654, (char)333, (char)31910, (char)23151, (char)32718, (char)36226, (char)32140, (char)11995, (char)47221, (char)39618, (char)25891, (char)48635, (char)2076, (char)5064, (char)28752, (char)1908, (char)18200, (char)64904, (char)35617, (char)42603, (char)13157, (char)62334, (char)13215, (char)42264, (char)19719, (char)2365, (char)11627, (char)16368, (char)2679, (char)47161, (char)1456, (char)24513, (char)3388, (char)26982, (char)27233}));
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.increment_GET() == (char)109);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.distances_SET(new char[] {(char)33465, (char)60447, (char)33311, (char)10103, (char)50415, (char)7308, (char)36448, (char)58847, (char)36005, (char)21576, (char)19056, (char)24587, (char)65090, (char)8353, (char)30580, (char)41348, (char)8867, (char)6395, (char)4832, (char)9444, (char)22562, (char)64764, (char)14919, (char)40940, (char)42307, (char)27134, (char)8909, (char)65126, (char)26581, (char)33920, (char)28520, (char)20201, (char)24999, (char)13686, (char)21350, (char)6384, (char)62962, (char)57654, (char)333, (char)31910, (char)23151, (char)32718, (char)36226, (char)32140, (char)11995, (char)47221, (char)39618, (char)25891, (char)48635, (char)2076, (char)5064, (char)28752, (char)1908, (char)18200, (char)64904, (char)35617, (char)42603, (char)13157, (char)62334, (char)13215, (char)42264, (char)19719, (char)2365, (char)11627, (char)16368, (char)2679, (char)47161, (char)1456, (char)24513, (char)3388, (char)26982, (char)27233}, 0) ;
        p330.max_distance_SET((char)9753) ;
        p330.min_distance_SET((char)64374) ;
        p330.time_usec_SET(2289086108042488053L) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p330.increment_SET((char)109) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
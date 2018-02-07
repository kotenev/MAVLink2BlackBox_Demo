
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
            long id = id__v(src);
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
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_RESET_MPPT:
                    id = 127;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL:
                    id = 128;
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
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_RESET_MPPT:
                    id = 127;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL:
                    id = 128;
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
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_RESET_MPPT:
                    id = 127;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL:
                    id = 128;
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
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_RESET_MPPT:
                    id = 127;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL:
                    id = 128;
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
                case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                    id = 110;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                    id = 111;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                    id = 112;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                    id = 113;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                    id = 114;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                    id = 115;
                    break;
                case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                    id = 116;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                    id = 117;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                    id = 118;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                    id = 119;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                    id = 120;
                    break;
                case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                    id = 121;
                    break;
                case MAV_CMD.MAV_CMD_USER_1:
                    id = 122;
                    break;
                case MAV_CMD.MAV_CMD_USER_2:
                    id = 123;
                    break;
                case MAV_CMD.MAV_CMD_USER_3:
                    id = 124;
                    break;
                case MAV_CMD.MAV_CMD_USER_4:
                    id = 125;
                    break;
                case MAV_CMD.MAV_CMD_USER_5:
                    id = 126;
                    break;
                case MAV_CMD.MAV_CMD_RESET_MPPT:
                    id = 127;
                    break;
                case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL:
                    id = 128;
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
        *	should allow to identify the commit using the main version number even for very large code bases*/
        public char[] flight_custom_version_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 28, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
        *	should allow to identify the commit using the main version number even for very large code bases*/
        public char[] flight_custom_version_GET()
        {return flight_custom_version_GET(new char[8], 0);}/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	should allow to identify the commit using the main version number even for very large code bases*/
        public char[] middleware_custom_version_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 36, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
        *	should allow to identify the commit using the main version number even for very large code bases*/
        public char[] middleware_custom_version_GET()
        {return middleware_custom_version_GET(new char[8], 0);}/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	should allow to identify the commit using the main version number even for very large code bases*/
        public char[] os_custom_version_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 44, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
        *	should allow to identify the commit using the main version number even for very large code bases*/
        public char[] os_custom_version_GET()
        {return os_custom_version_GET(new char[8], 0);} public @MAV_PROTOCOL_CAPABILITY int capabilities_GET()//bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
        {  return  1 + (int)get_bits(data, 416, 17); }
        /**
        *UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
        *	use uid*/
        public char[]  uid2_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  433 && !try_visit_field(ph, 433)) return null;
            return uid2_GET(ph, new char[ph.items], 0);
        }
        /**
        *UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
        *	use uid*/
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
*	the landing targe*/
        public char  position_valid_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  242 && !try_visit_field(ph, 242)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 1));
        }
    }
    public static class SENS_POWER extends GroundControl.SENS_POWER
    {
        public float adc121_vspb_volt_GET()//Power board voltage sensor reading in volts
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float adc121_cspb_amp_GET()//Power board current sensor reading in amps
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float adc121_cs1_amp_GET()//Board current sensor 1 reading in amps
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float adc121_cs2_amp_GET()//Board current sensor 2 reading in amps
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
    }
    public static class SENS_MPPT extends GroundControl.SENS_MPPT
    {
        public char mppt1_pwm_GET()//MPPT1 pwm
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char mppt2_pwm_GET()//MPPT2 pwm
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char mppt3_pwm_GET()//MPPT3 pwm
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public long mppt_timestamp_GET()//MPPT last timestamp
        {  return (get_bytes(data,  6, 8)); }
        public float mppt1_volt_GET()//MPPT1 voltage
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public float mppt1_amp_GET()//MPPT1 current
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public char mppt1_status_GET()//MPPT1 status
        {  return (char)((char) get_bytes(data,  22, 1)); }
        public float mppt2_volt_GET()//MPPT2 voltage
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  23, 4))); }
        public float mppt2_amp_GET()//MPPT2 current
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  27, 4))); }
        public char mppt2_status_GET()//MPPT2 status
        {  return (char)((char) get_bytes(data,  31, 1)); }
        public float mppt3_volt_GET()//MPPT3 voltage
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public float mppt3_amp_GET()//MPPT3 current
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public char mppt3_status_GET()//MPPT3 status
        {  return (char)((char) get_bytes(data,  40, 1)); }
    }
    public static class ASLCTRL_DATA extends GroundControl.ASLCTRL_DATA
    {
        public long timestamp_GET()//Timestamp
        {  return (get_bytes(data,  0, 8)); }
        public char aslctrl_mode_GET()//ASLCTRL control-mode (manual, stabilized, auto, etc...)
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public float h_GET()//See sourcecode for a description of these values...
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  9, 4))); }
        public float hRef_GET()//null
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  13, 4))); }
        public float hRef_t_GET()//null
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  17, 4))); }
        public float PitchAngle_GET()//Pitch angle [deg]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  21, 4))); }
        public float PitchAngleRef_GET()//Pitch angle reference[deg]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  25, 4))); }
        public float q_GET()//null
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  29, 4))); }
        public float qRef_GET()//null
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  33, 4))); }
        public float uElev_GET()//null
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  37, 4))); }
        public float uThrot_GET()//null
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  41, 4))); }
        public float uThrot2_GET()//null
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  45, 4))); }
        public float nZ_GET()//null
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  49, 4))); }
        public float AirspeedRef_GET()//Airspeed reference [m/s]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  53, 4))); }
        public char SpoilersEngaged_GET()//null
        {  return (char)((char) get_bytes(data,  57, 1)); }
        public float YawAngle_GET()//Yaw angle [deg]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  58, 4))); }
        public float YawAngleRef_GET()//Yaw angle reference[deg]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  62, 4))); }
        public float RollAngle_GET()//Roll angle [deg]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  66, 4))); }
        public float RollAngleRef_GET()//Roll angle reference[deg]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  70, 4))); }
        public float p_GET()//null
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  74, 4))); }
        public float pRef_GET()//null
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  78, 4))); }
        public float r_GET()//null
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  82, 4))); }
        public float rRef_GET()//null
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  86, 4))); }
        public float uAil_GET()//null
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  90, 4))); }
        public float uRud_GET()//null
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  94, 4))); }
    }
    public static class ASLCTRL_DEBUG extends GroundControl.ASLCTRL_DEBUG
    {
        public long i32_1_GET()//Debug data
        {  return (get_bytes(data,  0, 4)); }
        public char i8_1_GET()//Debug data
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char i8_2_GET()//Debug data
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public float f_1_GET()//Debug data
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public float f_2_GET()//Debug data
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public float f_3_GET()//Debug data
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public float f_4_GET()//Debug data
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public float f_5_GET()//Debug data
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public float f_6_GET()//Debug data
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  26, 4))); }
        public float f_7_GET()//Debug data
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  30, 4))); }
        public float f_8_GET()//Debug data
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  34, 4))); }
    }
    public static class ASLUAV_STATUS extends GroundControl.ASLUAV_STATUS
    {
        public char LED_status_GET()//Status of the position-indicator LEDs
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char SATCOM_status_GET()//Status of the IRIDIUM satellite communication system
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char[] Servo_status_GET(char[]  dst_ch, int pos)  //Status vector for up to 8 servos
        {
            for(int BYTE = 2, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] Servo_status_GET()//Status vector for up to 8 servos
        {return Servo_status_GET(new char[8], 0);} public float Motor_rpm_GET()//Motor RPM
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
    }
    public static class EKF_EXT extends GroundControl.EKF_EXT
    {
        public long timestamp_GET()//Time since system start [us]
        {  return (get_bytes(data,  0, 8)); }
        public float Windspeed_GET()//Magnitude of wind velocity (in lateral inertial plane) [m/s]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float WindDir_GET()//Wind heading angle from North [rad]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float WindZ_GET()//Z (Down) component of inertial wind velocity [m/s]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float Airspeed_GET()//Magnitude of air velocity [m/s]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float beta_GET()//Sideslip angle [rad]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float alpha_GET()//Angle of attack [rad]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
    }
    public static class ASL_OBCTRL extends GroundControl.ASL_OBCTRL
    {
        public long timestamp_GET()//Time since system start [us]
        {  return (get_bytes(data,  0, 8)); }
        public float uElev_GET()//Elevator command [~]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float uThrot_GET()//Throttle command [~]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float uThrot2_GET()//Throttle 2 command [~]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float uAilL_GET()//Left aileron command [~]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float uAilR_GET()//Right aileron command [~]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float uRud_GET()//Rudder command [~]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public char obctrl_status_GET()//Off-board computer status
        {  return (char)((char) get_bytes(data,  32, 1)); }
    }
    public static class SENS_ATMOS extends GroundControl.SENS_ATMOS
    {
        public float TempAmbient_GET()//Ambient temperature [degrees Celsius]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float Humidity_GET()//Relative humidity [%]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
    }
    public static class SENS_BATMON extends GroundControl.SENS_BATMON
    {
        public char voltage_GET()//Battery pack voltage in [mV]
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char batterystatus_GET()//Battery monitor status report bits in Hex
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char serialnumber_GET()//Battery monitor serial number in Hex
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public char hostfetcontrol_GET()//Battery monitor sensor host FET control in Hex
        {  return (char)((char) get_bytes(data,  6, 2)); }
        public char cellvoltage1_GET()//Battery pack cell 1 voltage in [mV]
        {  return (char)((char) get_bytes(data,  8, 2)); }
        public char cellvoltage2_GET()//Battery pack cell 2 voltage in [mV]
        {  return (char)((char) get_bytes(data,  10, 2)); }
        public char cellvoltage3_GET()//Battery pack cell 3 voltage in [mV]
        {  return (char)((char) get_bytes(data,  12, 2)); }
        public char cellvoltage4_GET()//Battery pack cell 4 voltage in [mV]
        {  return (char)((char) get_bytes(data,  14, 2)); }
        public char cellvoltage5_GET()//Battery pack cell 5 voltage in [mV]
        {  return (char)((char) get_bytes(data,  16, 2)); }
        public char cellvoltage6_GET()//Battery pack cell 6 voltage in [mV]
        {  return (char)((char) get_bytes(data,  18, 2)); }
        public float temperature_GET()//Battery pack temperature in [deg C]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public short current_GET()//Battery pack current in [mA]
        {  return (short)((short) get_bytes(data,  24, 2)); }
        public char SoC_GET()//Battery pack state-of-charge
        {  return (char)((char) get_bytes(data,  26, 1)); }
    }
    public static class FW_SOARING_DATA extends GroundControl.FW_SOARING_DATA
    {
        public long timestamp_GET()//Timestamp [ms]
        {  return (get_bytes(data,  0, 8)); }
        public long timestampModeChanged_GET()//Timestamp since last mode change[ms]
        {  return (get_bytes(data,  8, 8)); }
        public float xW_GET()//Thermal core updraft strength [m/s]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float xR_GET()//Thermal radius [m]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float xLat_GET()//Thermal center latitude [deg]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float xLon_GET()//Thermal center longitude [deg]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public float VarW_GET()//Variance W
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public float VarR_GET()//Variance R
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public float VarLat_GET()//Variance Lat
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public float VarLon_GET()//Variance Lon
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public float LoiterRadius_GET()//Suggested loiter radius [m]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  48, 4))); }
        public float LoiterDirection_GET()//Suggested loiter direction
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  52, 4))); }
        public float DistToSoarPoint_GET()//Distance to soar point [m]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  56, 4))); }
        public float vSinkExp_GET()//Expected sink rate at current airspeed, roll and throttle [m/s]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  60, 4))); }
        public float z1_LocalUpdraftSpeed_GET()//Measurement / updraft speed at current/local airplane position [m/s]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  64, 4))); }
        public float z2_DeltaRoll_GET()//Measurement / roll angle tracking error [deg]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  68, 4))); }
        public float z1_exp_GET()//Expected measurement 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  72, 4))); }
        public float z2_exp_GET()//Expected measurement 2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  76, 4))); }
        public float ThermalGSNorth_GET()//Thermal drift (from estimator prediction step only) [m/s]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  80, 4))); }
        public float ThermalGSEast_GET()//Thermal drift (from estimator prediction step only) [m/s]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  84, 4))); }
        public float TSE_dot_GET()//Total specific energy change (filtered) [m/s]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  88, 4))); }
        public float DebugVar1_GET()//Debug variable 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  92, 4))); }
        public float DebugVar2_GET()//Debug variable 2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  96, 4))); }
        public char ControlMode_GET()//Control Mode [-]
        {  return (char)((char) get_bytes(data,  100, 1)); }
        public char valid_GET()//Data valid [-]
        {  return (char)((char) get_bytes(data,  101, 1)); }
    }
    public static class SENSORPOD_STATUS extends GroundControl.SENSORPOD_STATUS
    {
        public char free_space_GET()//Free space available in recordings directory in [Gb] * 1e2
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public long timestamp_GET()//Timestamp in linuxtime [ms] (since 1.1.1970)
        {  return (get_bytes(data,  2, 8)); }
        public char visensor_rate_1_GET()//Rate of ROS topic 1
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public char visensor_rate_2_GET()//Rate of ROS topic 2
        {  return (char)((char) get_bytes(data,  11, 1)); }
        public char visensor_rate_3_GET()//Rate of ROS topic 3
        {  return (char)((char) get_bytes(data,  12, 1)); }
        public char visensor_rate_4_GET()//Rate of ROS topic 4
        {  return (char)((char) get_bytes(data,  13, 1)); }
        public char recording_nodes_count_GET()//Number of recording nodes
        {  return (char)((char) get_bytes(data,  14, 1)); }
        public char cpu_temp_GET()//Temperature of sensorpod CPU in [deg C]
        {  return (char)((char) get_bytes(data,  15, 1)); }
    }
    public static class SENS_POWER_BOARD extends GroundControl.SENS_POWER_BOARD
    {
        public long timestamp_GET()//Timestamp
        {  return (get_bytes(data,  0, 8)); }
        public char pwr_brd_status_GET()//Power board status register
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public char pwr_brd_led_status_GET()//Power board leds status
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public float pwr_brd_system_volt_GET()//Power board system voltage
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public float pwr_brd_servo_volt_GET()//Power board servo voltage
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public float pwr_brd_mot_l_amp_GET()//Power board left motor current sensor
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public float pwr_brd_mot_r_amp_GET()//Power board right motor current sensor
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public float pwr_brd_servo_1_amp_GET()//Power board servo1 current sensor
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  26, 4))); }
        public float pwr_brd_servo_2_amp_GET()//Power board servo1 current sensor
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  30, 4))); }
        public float pwr_brd_servo_3_amp_GET()//Power board servo1 current sensor
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  34, 4))); }
        public float pwr_brd_servo_4_amp_GET()//Power board servo1 current sensor
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  38, 4))); }
        public float pwr_brd_aux_amp_GET()//Power board aux current sensor
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  42, 4))); }
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

        static final Collection<OnReceive.Handler<AUTOPILOT_VERSION, Channel>> on_AUTOPILOT_VERSION = new OnReceive<>();
        static final Collection<OnReceive.Handler<LANDING_TARGET, Channel>> on_LANDING_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<SENS_POWER, Channel>> on_SENS_POWER = new OnReceive<>();
        static final Collection<OnReceive.Handler<SENS_MPPT, Channel>> on_SENS_MPPT = new OnReceive<>();
        static final Collection<OnReceive.Handler<ASLCTRL_DATA, Channel>> on_ASLCTRL_DATA = new OnReceive<>();
        static final Collection<OnReceive.Handler<ASLCTRL_DEBUG, Channel>> on_ASLCTRL_DEBUG = new OnReceive<>();
        static final Collection<OnReceive.Handler<ASLUAV_STATUS, Channel>> on_ASLUAV_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<EKF_EXT, Channel>> on_EKF_EXT = new OnReceive<>();
        static final Collection<OnReceive.Handler<ASL_OBCTRL, Channel>> on_ASL_OBCTRL = new OnReceive<>();
        static final Collection<OnReceive.Handler<SENS_ATMOS, Channel>> on_SENS_ATMOS = new OnReceive<>();
        static final Collection<OnReceive.Handler<SENS_BATMON, Channel>> on_SENS_BATMON = new OnReceive<>();
        static final Collection<OnReceive.Handler<FW_SOARING_DATA, Channel>> on_FW_SOARING_DATA = new OnReceive<>();
        static final Collection<OnReceive.Handler<SENSORPOD_STATUS, Channel>> on_SENSORPOD_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<SENS_POWER_BOARD, Channel>> on_SENS_POWER_BOARD = new OnReceive<>();
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
                case 148:
                    if(pack == null) return new AUTOPILOT_VERSION();
                    ((OnReceive) on_AUTOPILOT_VERSION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 149:
                    if(pack == null) return new LANDING_TARGET();
                    ((OnReceive) on_LANDING_TARGET).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 201:
                    if(pack == null) return new SENS_POWER();
                    ((OnReceive) on_SENS_POWER).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 202:
                    if(pack == null) return new SENS_MPPT();
                    ((OnReceive) on_SENS_MPPT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 203:
                    if(pack == null) return new ASLCTRL_DATA();
                    ((OnReceive) on_ASLCTRL_DATA).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 204:
                    if(pack == null) return new ASLCTRL_DEBUG();
                    ((OnReceive) on_ASLCTRL_DEBUG).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 205:
                    if(pack == null) return new ASLUAV_STATUS();
                    ((OnReceive) on_ASLUAV_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 206:
                    if(pack == null) return new EKF_EXT();
                    ((OnReceive) on_EKF_EXT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 207:
                    if(pack == null) return new ASL_OBCTRL();
                    ((OnReceive) on_ASL_OBCTRL).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 208:
                    if(pack == null) return new SENS_ATMOS();
                    ((OnReceive) on_SENS_ATMOS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 209:
                    if(pack == null) return new SENS_BATMON();
                    ((OnReceive) on_SENS_BATMON).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 210:
                    if(pack == null) return new FW_SOARING_DATA();
                    ((OnReceive) on_FW_SOARING_DATA).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 211:
                    if(pack == null) return new SENSORPOD_STATUS();
                    ((OnReceive) on_SENSORPOD_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 212:
                    if(pack == null) return new SENS_POWER_BOARD();
                    ((OnReceive) on_SENS_POWER_BOARD).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
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
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_SMARTAP);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED));
            assert(pack.mavlink_version_GET() == (char)7);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_POWEROFF);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_GENERIC);
            assert(pack.custom_mode_GET() == 1688898347L);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED)) ;
        p0.custom_mode_SET(1688898347L) ;
        p0.mavlink_version_SET((char)7) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_GENERIC) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_SMARTAP) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_POWEROFF) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_count2_GET() == (char)7449);
            assert(pack.voltage_battery_GET() == (char)21880);
            assert(pack.drop_rate_comm_GET() == (char)31796);
            assert(pack.load_GET() == (char)60528);
            assert(pack.errors_count3_GET() == (char)29258);
            assert(pack.current_battery_GET() == (short)13619);
            assert(pack.battery_remaining_GET() == (byte)98);
            assert(pack.errors_count1_GET() == (char)33168);
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.errors_comm_GET() == (char)59653);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
            assert(pack.errors_count4_GET() == (char)41550);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.errors_count3_SET((char)29258) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.errors_comm_SET((char)59653) ;
        p1.drop_rate_comm_SET((char)31796) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.battery_remaining_SET((byte)98) ;
        p1.load_SET((char)60528) ;
        p1.errors_count2_SET((char)7449) ;
        p1.voltage_battery_SET((char)21880) ;
        p1.current_battery_SET((short)13619) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW)) ;
        p1.errors_count1_SET((char)33168) ;
        p1.errors_count4_SET((char)41550) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 4312287163482396694L);
            assert(pack.time_boot_ms_GET() == 3461934066L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(3461934066L) ;
        p2.time_unix_usec_SET(4312287163482396694L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)53358);
            assert(pack.z_GET() == 8.1567515E37F);
            assert(pack.vy_GET() == -7.432974E37F);
            assert(pack.yaw_GET() == 2.9046607E38F);
            assert(pack.afz_GET() == 1.177488E38F);
            assert(pack.afx_GET() == 2.7478363E38F);
            assert(pack.vz_GET() == -1.1248683E38F);
            assert(pack.y_GET() == -1.6836402E38F);
            assert(pack.afy_GET() == -2.3540053E38F);
            assert(pack.yaw_rate_GET() == -1.0793327E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.time_boot_ms_GET() == 334930744L);
            assert(pack.vx_GET() == -1.55527E38F);
            assert(pack.x_GET() == -1.5329831E38F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.z_SET(8.1567515E37F) ;
        p3.yaw_SET(2.9046607E38F) ;
        p3.yaw_rate_SET(-1.0793327E38F) ;
        p3.y_SET(-1.6836402E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p3.afx_SET(2.7478363E38F) ;
        p3.x_SET(-1.5329831E38F) ;
        p3.time_boot_ms_SET(334930744L) ;
        p3.vz_SET(-1.1248683E38F) ;
        p3.vy_SET(-7.432974E37F) ;
        p3.type_mask_SET((char)53358) ;
        p3.afy_SET(-2.3540053E38F) ;
        p3.vx_SET(-1.55527E38F) ;
        p3.afz_SET(1.177488E38F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7884757818150194079L);
            assert(pack.target_component_GET() == (char)222);
            assert(pack.seq_GET() == 2665757123L);
            assert(pack.target_system_GET() == (char)113);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.target_component_SET((char)222) ;
        p4.time_usec_SET(7884757818150194079L) ;
        p4.seq_SET(2665757123L) ;
        p4.target_system_SET((char)113) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)35);
            assert(pack.passkey_LEN(ph) == 11);
            assert(pack.passkey_TRY(ph).equals("kNIacgzrhnq"));
            assert(pack.target_system_GET() == (char)226);
            assert(pack.version_GET() == (char)209);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.passkey_SET("kNIacgzrhnq", PH) ;
        p5.control_request_SET((char)35) ;
        p5.version_SET((char)209) ;
        p5.target_system_SET((char)226) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)165);
            assert(pack.control_request_GET() == (char)195);
            assert(pack.ack_GET() == (char)222);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)165) ;
        p6.ack_SET((char)222) ;
        p6.control_request_SET((char)195) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 15);
            assert(pack.key_TRY(ph).equals("yzdCwxdbfpkubvg"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("yzdCwxdbfpkubvg", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.custom_mode_GET() == 2644920774L);
            assert(pack.target_system_GET() == (char)16);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_ARMED);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.custom_mode_SET(2644920774L) ;
        p11.target_system_SET((char)16) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_STABILIZE_ARMED) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("ncvviiolymmhumj"));
            assert(pack.param_index_GET() == (short)2566);
            assert(pack.target_component_GET() == (char)103);
            assert(pack.target_system_GET() == (char)191);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_index_SET((short)2566) ;
        p20.target_component_SET((char)103) ;
        p20.target_system_SET((char)191) ;
        p20.param_id_SET("ncvviiolymmhumj", PH) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)164);
            assert(pack.target_component_GET() == (char)7);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)7) ;
        p21.target_system_SET((char)164) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (char)36556);
            assert(pack.param_count_GET() == (char)40055);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
            assert(pack.param_value_GET() == 3.12529E37F);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("ppnz"));
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_count_SET((char)40055) ;
        p22.param_id_SET("ppnz", PH) ;
        p22.param_index_SET((char)36556) ;
        p22.param_value_SET(3.12529E37F) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)122);
            assert(pack.target_component_GET() == (char)78);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
            assert(pack.param_value_GET() == 4.7430085E37F);
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("zhplEbcMrtqpuTo"));
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.param_id_SET("zhplEbcMrtqpuTo", PH) ;
        p23.target_component_SET((char)78) ;
        p23.target_system_SET((char)122) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32) ;
        p23.param_value_SET(4.7430085E37F) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.alt_ellipsoid_TRY(ph) == -1360008860);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.lon_GET() == 1757890294);
            assert(pack.eph_GET() == (char)37957);
            assert(pack.lat_GET() == 2050638335);
            assert(pack.satellites_visible_GET() == (char)86);
            assert(pack.alt_GET() == -784721346);
            assert(pack.epv_GET() == (char)49733);
            assert(pack.vel_acc_TRY(ph) == 811558454L);
            assert(pack.hdg_acc_TRY(ph) == 3443671270L);
            assert(pack.time_usec_GET() == 5047948131297590036L);
            assert(pack.v_acc_TRY(ph) == 2453465706L);
            assert(pack.cog_GET() == (char)32202);
            assert(pack.vel_GET() == (char)26012);
            assert(pack.h_acc_TRY(ph) == 2974377126L);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.alt_SET(-784721346) ;
        p24.h_acc_SET(2974377126L, PH) ;
        p24.vel_acc_SET(811558454L, PH) ;
        p24.lon_SET(1757890294) ;
        p24.lat_SET(2050638335) ;
        p24.time_usec_SET(5047948131297590036L) ;
        p24.vel_SET((char)26012) ;
        p24.hdg_acc_SET(3443671270L, PH) ;
        p24.cog_SET((char)32202) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p24.epv_SET((char)49733) ;
        p24.v_acc_SET(2453465706L, PH) ;
        p24.satellites_visible_SET((char)86) ;
        p24.alt_ellipsoid_SET(-1360008860, PH) ;
        p24.eph_SET((char)37957) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)193, (char)39, (char)131, (char)118, (char)155, (char)230, (char)219, (char)125, (char)98, (char)110, (char)73, (char)61, (char)10, (char)102, (char)15, (char)145, (char)183, (char)145, (char)86, (char)36}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)40, (char)228, (char)65, (char)65, (char)36, (char)63, (char)147, (char)229, (char)120, (char)177, (char)70, (char)244, (char)219, (char)128, (char)65, (char)223, (char)63, (char)122, (char)244, (char)81}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)101, (char)67, (char)41, (char)50, (char)116, (char)231, (char)188, (char)214, (char)170, (char)209, (char)187, (char)183, (char)184, (char)6, (char)102, (char)89, (char)185, (char)226, (char)84, (char)139}));
            assert(pack.satellites_visible_GET() == (char)97);
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)217, (char)44, (char)83, (char)51, (char)61, (char)245, (char)112, (char)239, (char)50, (char)234, (char)193, (char)43, (char)19, (char)192, (char)243, (char)222, (char)12, (char)254, (char)183, (char)16}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)183, (char)127, (char)135, (char)254, (char)240, (char)185, (char)159, (char)247, (char)23, (char)46, (char)185, (char)129, (char)9, (char)46, (char)3, (char)197, (char)33, (char)26, (char)183, (char)174}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)97) ;
        p25.satellite_prn_SET(new char[] {(char)183, (char)127, (char)135, (char)254, (char)240, (char)185, (char)159, (char)247, (char)23, (char)46, (char)185, (char)129, (char)9, (char)46, (char)3, (char)197, (char)33, (char)26, (char)183, (char)174}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)101, (char)67, (char)41, (char)50, (char)116, (char)231, (char)188, (char)214, (char)170, (char)209, (char)187, (char)183, (char)184, (char)6, (char)102, (char)89, (char)185, (char)226, (char)84, (char)139}, 0) ;
        p25.satellite_used_SET(new char[] {(char)217, (char)44, (char)83, (char)51, (char)61, (char)245, (char)112, (char)239, (char)50, (char)234, (char)193, (char)43, (char)19, (char)192, (char)243, (char)222, (char)12, (char)254, (char)183, (char)16}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)193, (char)39, (char)131, (char)118, (char)155, (char)230, (char)219, (char)125, (char)98, (char)110, (char)73, (char)61, (char)10, (char)102, (char)15, (char)145, (char)183, (char)145, (char)86, (char)36}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)40, (char)228, (char)65, (char)65, (char)36, (char)63, (char)147, (char)229, (char)120, (char)177, (char)70, (char)244, (char)219, (char)128, (char)65, (char)223, (char)63, (char)122, (char)244, (char)81}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short) -17550);
            assert(pack.zmag_GET() == (short)4839);
            assert(pack.xmag_GET() == (short) -19799);
            assert(pack.xacc_GET() == (short)12362);
            assert(pack.ymag_GET() == (short)19421);
            assert(pack.time_boot_ms_GET() == 3685326834L);
            assert(pack.ygyro_GET() == (short)14199);
            assert(pack.zacc_GET() == (short) -938);
            assert(pack.yacc_GET() == (short)16020);
            assert(pack.xgyro_GET() == (short) -19658);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.time_boot_ms_SET(3685326834L) ;
        p26.xacc_SET((short)12362) ;
        p26.ygyro_SET((short)14199) ;
        p26.zacc_SET((short) -938) ;
        p26.xmag_SET((short) -19799) ;
        p26.xgyro_SET((short) -19658) ;
        p26.zgyro_SET((short) -17550) ;
        p26.zmag_SET((short)4839) ;
        p26.yacc_SET((short)16020) ;
        p26.ymag_SET((short)19421) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short)13779);
            assert(pack.xacc_GET() == (short)28862);
            assert(pack.xmag_GET() == (short)7304);
            assert(pack.zacc_GET() == (short) -7537);
            assert(pack.ymag_GET() == (short)18096);
            assert(pack.ygyro_GET() == (short) -9098);
            assert(pack.zmag_GET() == (short) -15481);
            assert(pack.yacc_GET() == (short)12296);
            assert(pack.time_usec_GET() == 1383060145910996302L);
            assert(pack.xgyro_GET() == (short) -12203);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.yacc_SET((short)12296) ;
        p27.xmag_SET((short)7304) ;
        p27.ygyro_SET((short) -9098) ;
        p27.zgyro_SET((short)13779) ;
        p27.zacc_SET((short) -7537) ;
        p27.time_usec_SET(1383060145910996302L) ;
        p27.xacc_SET((short)28862) ;
        p27.zmag_SET((short) -15481) ;
        p27.ymag_SET((short)18096) ;
        p27.xgyro_SET((short) -12203) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff2_GET() == (short)12161);
            assert(pack.press_abs_GET() == (short)11525);
            assert(pack.time_usec_GET() == 1744734186971511300L);
            assert(pack.temperature_GET() == (short)20340);
            assert(pack.press_diff1_GET() == (short)11936);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_diff1_SET((short)11936) ;
        p28.press_diff2_SET((short)12161) ;
        p28.press_abs_SET((short)11525) ;
        p28.temperature_SET((short)20340) ;
        p28.time_usec_SET(1744734186971511300L) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 4197514213L);
            assert(pack.temperature_GET() == (short) -5894);
            assert(pack.press_diff_GET() == -1.2256574E38F);
            assert(pack.press_abs_GET() == 1.91219E38F);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_abs_SET(1.91219E38F) ;
        p29.press_diff_SET(-1.2256574E38F) ;
        p29.time_boot_ms_SET(4197514213L) ;
        p29.temperature_SET((short) -5894) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.pitchspeed_GET() == 4.2067145E37F);
            assert(pack.time_boot_ms_GET() == 1532784537L);
            assert(pack.yaw_GET() == -2.208505E38F);
            assert(pack.pitch_GET() == -1.343817E38F);
            assert(pack.yawspeed_GET() == 1.6551368E38F);
            assert(pack.rollspeed_GET() == 2.959127E38F);
            assert(pack.roll_GET() == 2.7653936E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.time_boot_ms_SET(1532784537L) ;
        p30.rollspeed_SET(2.959127E38F) ;
        p30.roll_SET(2.7653936E38F) ;
        p30.pitch_SET(-1.343817E38F) ;
        p30.yaw_SET(-2.208505E38F) ;
        p30.yawspeed_SET(1.6551368E38F) ;
        p30.pitchspeed_SET(4.2067145E37F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q3_GET() == 1.7956843E38F);
            assert(pack.time_boot_ms_GET() == 323415388L);
            assert(pack.q4_GET() == -1.2605166E37F);
            assert(pack.rollspeed_GET() == 7.78547E37F);
            assert(pack.yawspeed_GET() == 1.0546604E38F);
            assert(pack.q1_GET() == -5.010882E37F);
            assert(pack.q2_GET() == 3.3454784E38F);
            assert(pack.pitchspeed_GET() == -9.64427E37F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q3_SET(1.7956843E38F) ;
        p31.q2_SET(3.3454784E38F) ;
        p31.q4_SET(-1.2605166E37F) ;
        p31.time_boot_ms_SET(323415388L) ;
        p31.pitchspeed_SET(-9.64427E37F) ;
        p31.yawspeed_SET(1.0546604E38F) ;
        p31.q1_SET(-5.010882E37F) ;
        p31.rollspeed_SET(7.78547E37F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == 2.614442E38F);
            assert(pack.x_GET() == -1.8539134E38F);
            assert(pack.z_GET() == 3.2659405E38F);
            assert(pack.vy_GET() == 7.1433693E37F);
            assert(pack.y_GET() == -1.1410059E38F);
            assert(pack.vx_GET() == -5.94835E37F);
            assert(pack.time_boot_ms_GET() == 3992173459L);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.time_boot_ms_SET(3992173459L) ;
        p32.x_SET(-1.8539134E38F) ;
        p32.vz_SET(2.614442E38F) ;
        p32.z_SET(3.2659405E38F) ;
        p32.vy_SET(7.1433693E37F) ;
        p32.vx_SET(-5.94835E37F) ;
        p32.y_SET(-1.1410059E38F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 1399243627);
            assert(pack.lon_GET() == 1437972661);
            assert(pack.hdg_GET() == (char)27259);
            assert(pack.vx_GET() == (short) -31666);
            assert(pack.lat_GET() == -1586112532);
            assert(pack.vz_GET() == (short) -29497);
            assert(pack.vy_GET() == (short) -4890);
            assert(pack.relative_alt_GET() == -1274635786);
            assert(pack.time_boot_ms_GET() == 3591261830L);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.relative_alt_SET(-1274635786) ;
        p33.alt_SET(1399243627) ;
        p33.vz_SET((short) -29497) ;
        p33.vy_SET((short) -4890) ;
        p33.time_boot_ms_SET(3591261830L) ;
        p33.vx_SET((short) -31666) ;
        p33.lon_SET(1437972661) ;
        p33.hdg_SET((char)27259) ;
        p33.lat_SET(-1586112532) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan3_scaled_GET() == (short) -17229);
            assert(pack.chan1_scaled_GET() == (short) -26753);
            assert(pack.chan5_scaled_GET() == (short) -15887);
            assert(pack.chan4_scaled_GET() == (short) -4447);
            assert(pack.chan6_scaled_GET() == (short)21871);
            assert(pack.rssi_GET() == (char)65);
            assert(pack.chan8_scaled_GET() == (short) -7110);
            assert(pack.chan7_scaled_GET() == (short)7797);
            assert(pack.time_boot_ms_GET() == 1595759708L);
            assert(pack.chan2_scaled_GET() == (short)30182);
            assert(pack.port_GET() == (char)196);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan5_scaled_SET((short) -15887) ;
        p34.chan6_scaled_SET((short)21871) ;
        p34.chan1_scaled_SET((short) -26753) ;
        p34.port_SET((char)196) ;
        p34.chan3_scaled_SET((short) -17229) ;
        p34.chan7_scaled_SET((short)7797) ;
        p34.chan2_scaled_SET((short)30182) ;
        p34.time_boot_ms_SET(1595759708L) ;
        p34.chan4_scaled_SET((short) -4447) ;
        p34.chan8_scaled_SET((short) -7110) ;
        p34.rssi_SET((char)65) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan3_raw_GET() == (char)14091);
            assert(pack.chan1_raw_GET() == (char)22940);
            assert(pack.chan8_raw_GET() == (char)63166);
            assert(pack.port_GET() == (char)85);
            assert(pack.chan4_raw_GET() == (char)39562);
            assert(pack.rssi_GET() == (char)88);
            assert(pack.chan6_raw_GET() == (char)64029);
            assert(pack.chan2_raw_GET() == (char)56482);
            assert(pack.time_boot_ms_GET() == 3576814729L);
            assert(pack.chan7_raw_GET() == (char)41961);
            assert(pack.chan5_raw_GET() == (char)41553);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan5_raw_SET((char)41553) ;
        p35.chan4_raw_SET((char)39562) ;
        p35.chan8_raw_SET((char)63166) ;
        p35.chan7_raw_SET((char)41961) ;
        p35.time_boot_ms_SET(3576814729L) ;
        p35.chan2_raw_SET((char)56482) ;
        p35.chan3_raw_SET((char)14091) ;
        p35.chan1_raw_SET((char)22940) ;
        p35.chan6_raw_SET((char)64029) ;
        p35.port_SET((char)85) ;
        p35.rssi_SET((char)88) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo9_raw_TRY(ph) == (char)20357);
            assert(pack.servo11_raw_TRY(ph) == (char)53130);
            assert(pack.servo1_raw_GET() == (char)28725);
            assert(pack.port_GET() == (char)56);
            assert(pack.time_usec_GET() == 280737925L);
            assert(pack.servo15_raw_TRY(ph) == (char)29517);
            assert(pack.servo5_raw_GET() == (char)42237);
            assert(pack.servo7_raw_GET() == (char)51086);
            assert(pack.servo14_raw_TRY(ph) == (char)24389);
            assert(pack.servo3_raw_GET() == (char)18888);
            assert(pack.servo12_raw_TRY(ph) == (char)55922);
            assert(pack.servo10_raw_TRY(ph) == (char)57383);
            assert(pack.servo6_raw_GET() == (char)16051);
            assert(pack.servo4_raw_GET() == (char)17692);
            assert(pack.servo2_raw_GET() == (char)36834);
            assert(pack.servo13_raw_TRY(ph) == (char)54826);
            assert(pack.servo16_raw_TRY(ph) == (char)6906);
            assert(pack.servo8_raw_GET() == (char)45012);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo4_raw_SET((char)17692) ;
        p36.servo2_raw_SET((char)36834) ;
        p36.servo8_raw_SET((char)45012) ;
        p36.servo1_raw_SET((char)28725) ;
        p36.servo12_raw_SET((char)55922, PH) ;
        p36.servo16_raw_SET((char)6906, PH) ;
        p36.servo13_raw_SET((char)54826, PH) ;
        p36.servo6_raw_SET((char)16051) ;
        p36.servo15_raw_SET((char)29517, PH) ;
        p36.servo7_raw_SET((char)51086) ;
        p36.port_SET((char)56) ;
        p36.servo3_raw_SET((char)18888) ;
        p36.servo11_raw_SET((char)53130, PH) ;
        p36.servo14_raw_SET((char)24389, PH) ;
        p36.servo10_raw_SET((char)57383, PH) ;
        p36.servo9_raw_SET((char)20357, PH) ;
        p36.servo5_raw_SET((char)42237) ;
        p36.time_usec_SET(280737925L) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)41);
            assert(pack.end_index_GET() == (short)9029);
            assert(pack.start_index_GET() == (short) -15937);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_component_GET() == (char)229);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p37.end_index_SET((short)9029) ;
        p37.target_system_SET((char)41) ;
        p37.target_component_SET((char)229) ;
        p37.start_index_SET((short) -15937) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.end_index_GET() == (short) -5419);
            assert(pack.target_system_GET() == (char)193);
            assert(pack.start_index_GET() == (short)23213);
            assert(pack.target_component_GET() == (char)184);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.start_index_SET((short)23213) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p38.target_component_SET((char)184) ;
        p38.target_system_SET((char)193) ;
        p38.end_index_SET((short) -5419) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)34170);
            assert(pack.target_system_GET() == (char)109);
            assert(pack.y_GET() == 9.498059E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.param4_GET() == 2.2815224E38F);
            assert(pack.param3_GET() == -1.1428061E38F);
            assert(pack.param2_GET() == 2.319721E38F);
            assert(pack.autocontinue_GET() == (char)84);
            assert(pack.z_GET() == -2.1455356E38F);
            assert(pack.current_GET() == (char)77);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH);
            assert(pack.param1_GET() == 3.2513275E38F);
            assert(pack.x_GET() == 2.4971934E38F);
            assert(pack.target_component_GET() == (char)18);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.param4_SET(2.2815224E38F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p39.y_SET(9.498059E37F) ;
        p39.z_SET(-2.1455356E38F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p39.current_SET((char)77) ;
        p39.target_component_SET((char)18) ;
        p39.seq_SET((char)34170) ;
        p39.param3_SET(-1.1428061E38F) ;
        p39.x_SET(2.4971934E38F) ;
        p39.param1_SET(3.2513275E38F) ;
        p39.param2_SET(2.319721E38F) ;
        p39.autocontinue_SET((char)84) ;
        p39.target_system_SET((char)109) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)60229);
            assert(pack.target_component_GET() == (char)131);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)81);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p40.seq_SET((char)60229) ;
        p40.target_system_SET((char)81) ;
        p40.target_component_SET((char)131) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)40589);
            assert(pack.target_component_GET() == (char)12);
            assert(pack.target_system_GET() == (char)151);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_component_SET((char)12) ;
        p41.target_system_SET((char)151) ;
        p41.seq_SET((char)40589) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)15762);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)15762) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)24);
            assert(pack.target_system_GET() == (char)90);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)90) ;
        p43.target_component_SET((char)24) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)3882);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_component_GET() == (char)237);
            assert(pack.target_system_GET() == (char)174);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p44.count_SET((char)3882) ;
        p44.target_system_SET((char)174) ;
        p44.target_component_SET((char)237) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)68);
            assert(pack.target_component_GET() == (char)219);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)68) ;
        p45.target_component_SET((char)219) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)57545);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)57545) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED);
            assert(pack.target_component_GET() == (char)25);
            assert(pack.target_system_GET() == (char)86);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.target_component_SET((char)25) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p47.target_system_SET((char)86) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == -674461068);
            assert(pack.altitude_GET() == -1305454713);
            assert(pack.target_system_GET() == (char)145);
            assert(pack.time_usec_TRY(ph) == 2850347735975848769L);
            assert(pack.longitude_GET() == -1557799170);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.latitude_SET(-674461068) ;
        p48.time_usec_SET(2850347735975848769L, PH) ;
        p48.altitude_SET(-1305454713) ;
        p48.target_system_SET((char)145) ;
        p48.longitude_SET(-1557799170) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 2081696262);
            assert(pack.latitude_GET() == -84940977);
            assert(pack.longitude_GET() == 1473888080);
            assert(pack.time_usec_TRY(ph) == 6420275641249452486L);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(-84940977) ;
        p49.altitude_SET(2081696262) ;
        p49.time_usec_SET(6420275641249452486L, PH) ;
        p49.longitude_SET(1473888080) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_value0_GET() == -8.934704E37F);
            assert(pack.param_value_min_GET() == -5.278598E37F);
            assert(pack.scale_GET() == -1.2874739E38F);
            assert(pack.target_component_GET() == (char)112);
            assert(pack.param_value_max_GET() == -3.589083E37F);
            assert(pack.param_index_GET() == (short)10183);
            assert(pack.target_system_GET() == (char)92);
            assert(pack.parameter_rc_channel_index_GET() == (char)76);
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("qzYyjircgjs"));
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_system_SET((char)92) ;
        p50.param_value_min_SET(-5.278598E37F) ;
        p50.parameter_rc_channel_index_SET((char)76) ;
        p50.param_value0_SET(-8.934704E37F) ;
        p50.param_id_SET("qzYyjircgjs", PH) ;
        p50.param_value_max_SET(-3.589083E37F) ;
        p50.scale_SET(-1.2874739E38F) ;
        p50.target_component_SET((char)112) ;
        p50.param_index_SET((short)10183) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)112);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.seq_GET() == (char)16580);
            assert(pack.target_component_GET() == (char)117);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_component_SET((char)117) ;
        p51.target_system_SET((char)112) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p51.seq_SET((char)16580) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1x_GET() == 2.4578723E38F);
            assert(pack.target_component_GET() == (char)87);
            assert(pack.p1y_GET() == 2.8396495E38F);
            assert(pack.p1z_GET() == -1.6573253E37F);
            assert(pack.p2y_GET() == -7.6277584E37F);
            assert(pack.p2z_GET() == -3.6099505E37F);
            assert(pack.target_system_GET() == (char)4);
            assert(pack.p2x_GET() == -2.9973706E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p1z_SET(-1.6573253E37F) ;
        p54.target_component_SET((char)87) ;
        p54.p2y_SET(-7.6277584E37F) ;
        p54.p1x_SET(2.4578723E38F) ;
        p54.p2z_SET(-3.6099505E37F) ;
        p54.p1y_SET(2.8396495E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p54.target_system_SET((char)4) ;
        p54.p2x_SET(-2.9973706E38F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1y_GET() == -2.055702E38F);
            assert(pack.p1x_GET() == -2.2000342E38F);
            assert(pack.p2x_GET() == 1.9970117E38F);
            assert(pack.p1z_GET() == -3.0614537E38F);
            assert(pack.p2y_GET() == 1.9054486E38F);
            assert(pack.p2z_GET() == -1.5617905E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2x_SET(1.9970117E38F) ;
        p55.p2y_SET(1.9054486E38F) ;
        p55.p1x_SET(-2.2000342E38F) ;
        p55.p1y_SET(-2.055702E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p55.p1z_SET(-3.0614537E38F) ;
        p55.p2z_SET(-1.5617905E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == -2.2742347E38F);
            assert(pack.pitchspeed_GET() == -1.764855E37F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.5589785E38F, -2.3650007E36F, 1.5118218E36F, -3.0081324E38F, -1.936023E38F, -9.27893E37F, -3.1075805E38F, -1.3136087E38F, 1.6653551E38F}));
            assert(pack.rollspeed_GET() == 2.102159E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.6247554E38F, -2.1347471E38F, -3.2885887E38F, -2.4569255E38F}));
            assert(pack.time_usec_GET() == 7435654135091024088L);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.rollspeed_SET(2.102159E38F) ;
        p61.covariance_SET(new float[] {1.5589785E38F, -2.3650007E36F, 1.5118218E36F, -3.0081324E38F, -1.936023E38F, -9.27893E37F, -3.1075805E38F, -1.3136087E38F, 1.6653551E38F}, 0) ;
        p61.yawspeed_SET(-2.2742347E38F) ;
        p61.pitchspeed_SET(-1.764855E37F) ;
        p61.time_usec_SET(7435654135091024088L) ;
        p61.q_SET(new float[] {-1.6247554E38F, -2.1347471E38F, -3.2885887E38F, -2.4569255E38F}, 0) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.alt_error_GET() == 2.0635478E38F);
            assert(pack.wp_dist_GET() == (char)44190);
            assert(pack.aspd_error_GET() == 2.5629335E38F);
            assert(pack.nav_roll_GET() == -2.7718837E38F);
            assert(pack.target_bearing_GET() == (short)29356);
            assert(pack.nav_bearing_GET() == (short)7932);
            assert(pack.nav_pitch_GET() == 1.337207E38F);
            assert(pack.xtrack_error_GET() == 1.673975E38F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.alt_error_SET(2.0635478E38F) ;
        p62.xtrack_error_SET(1.673975E38F) ;
        p62.nav_bearing_SET((short)7932) ;
        p62.wp_dist_SET((char)44190) ;
        p62.target_bearing_SET((short)29356) ;
        p62.nav_roll_SET(-2.7718837E38F) ;
        p62.aspd_error_SET(2.5629335E38F) ;
        p62.nav_pitch_SET(1.337207E38F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == 8.1753454E37F);
            assert(pack.lat_GET() == -756384944);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
            assert(pack.lon_GET() == 1353958493);
            assert(pack.relative_alt_GET() == 1720126443);
            assert(pack.time_usec_GET() == 108998952166864002L);
            assert(pack.vy_GET() == 3.3824356E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {4.9846187E37F, 2.6767538E38F, 1.441571E38F, 2.0109768E38F, 3.0444206E37F, 7.008948E36F, 9.418397E36F, -1.7143563E38F, 1.8432127E38F, 1.7892927E38F, 3.1180231E38F, 1.5098297E38F, 2.0060743E38F, 2.4774207E38F, -2.1649199E38F, -2.8260915E38F, -3.3258974E38F, -5.7033005E37F, 2.7177706E37F, -7.2684226E37F, 8.529142E37F, -1.7092916E38F, -9.270114E37F, 5.433829E37F, -2.7840915E38F, 3.2087028E38F, 2.4817179E37F, 2.8757739E38F, -2.8544056E38F, 1.9457956E38F, 2.5430122E38F, -2.2896244E38F, -3.0341888E38F, 2.9304259E38F, -1.0127723E38F, -1.3190206E38F}));
            assert(pack.vz_GET() == -8.536814E37F);
            assert(pack.alt_GET() == 1314307963);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.vx_SET(8.1753454E37F) ;
        p63.vy_SET(3.3824356E38F) ;
        p63.time_usec_SET(108998952166864002L) ;
        p63.alt_SET(1314307963) ;
        p63.covariance_SET(new float[] {4.9846187E37F, 2.6767538E38F, 1.441571E38F, 2.0109768E38F, 3.0444206E37F, 7.008948E36F, 9.418397E36F, -1.7143563E38F, 1.8432127E38F, 1.7892927E38F, 3.1180231E38F, 1.5098297E38F, 2.0060743E38F, 2.4774207E38F, -2.1649199E38F, -2.8260915E38F, -3.3258974E38F, -5.7033005E37F, 2.7177706E37F, -7.2684226E37F, 8.529142E37F, -1.7092916E38F, -9.270114E37F, 5.433829E37F, -2.7840915E38F, 3.2087028E38F, 2.4817179E37F, 2.8757739E38F, -2.8544056E38F, 1.9457956E38F, 2.5430122E38F, -2.2896244E38F, -3.0341888E38F, 2.9304259E38F, -1.0127723E38F, -1.3190206E38F}, 0) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS) ;
        p63.lon_SET(1353958493) ;
        p63.lat_SET(-756384944) ;
        p63.vz_SET(-8.536814E37F) ;
        p63.relative_alt_SET(1720126443) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.ax_GET() == 1.6819922E38F);
            assert(pack.ay_GET() == -1.9018736E38F);
            assert(pack.az_GET() == -3.1262039E38F);
            assert(pack.vx_GET() == 3.3895253E38F);
            assert(pack.y_GET() == 1.2459815E38F);
            assert(pack.time_usec_GET() == 8952121005195713108L);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.4512495E38F, 9.265889E37F, 1.7237736E38F, -1.6126894E38F, 2.1815806E38F, -2.0005255E38F, -9.962175E37F, 2.474006E38F, 1.77719E38F, -2.6220677E38F, -2.9529278E38F, 1.4070759E38F, 9.333322E37F, 5.9325795E36F, 1.3514066E38F, -6.35346E37F, -1.2476213E38F, 2.883281E38F, 2.8234621E38F, -1.7577442E38F, 2.9247734E38F, 3.065005E38F, -1.0414198E38F, -2.0574931E36F, -2.1170065E37F, 1.2124366E38F, -7.836436E37F, -4.0504276E37F, 1.1988447E37F, 2.4415003E38F, -2.7867954E38F, 2.5941798E37F, -1.1129107E38F, 2.735415E38F, -2.9521205E38F, 1.3726495E38F, -2.9340008E38F, -2.3605283E38F, 2.9385865E38F, -2.8147418E37F, -1.646977E38F, -2.2966618E38F, 1.3576418E38F, 1.4939947E38F, -2.8332313E38F}));
            assert(pack.vz_GET() == 2.9799529E38F);
            assert(pack.vy_GET() == -5.0496233E37F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(pack.z_GET() == 2.2805223E38F);
            assert(pack.x_GET() == -2.3132277E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.vx_SET(3.3895253E38F) ;
        p64.time_usec_SET(8952121005195713108L) ;
        p64.ax_SET(1.6819922E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p64.y_SET(1.2459815E38F) ;
        p64.ay_SET(-1.9018736E38F) ;
        p64.vy_SET(-5.0496233E37F) ;
        p64.vz_SET(2.9799529E38F) ;
        p64.x_SET(-2.3132277E38F) ;
        p64.covariance_SET(new float[] {2.4512495E38F, 9.265889E37F, 1.7237736E38F, -1.6126894E38F, 2.1815806E38F, -2.0005255E38F, -9.962175E37F, 2.474006E38F, 1.77719E38F, -2.6220677E38F, -2.9529278E38F, 1.4070759E38F, 9.333322E37F, 5.9325795E36F, 1.3514066E38F, -6.35346E37F, -1.2476213E38F, 2.883281E38F, 2.8234621E38F, -1.7577442E38F, 2.9247734E38F, 3.065005E38F, -1.0414198E38F, -2.0574931E36F, -2.1170065E37F, 1.2124366E38F, -7.836436E37F, -4.0504276E37F, 1.1988447E37F, 2.4415003E38F, -2.7867954E38F, 2.5941798E37F, -1.1129107E38F, 2.735415E38F, -2.9521205E38F, 1.3726495E38F, -2.9340008E38F, -2.3605283E38F, 2.9385865E38F, -2.8147418E37F, -1.646977E38F, -2.2966618E38F, 1.3576418E38F, 1.4939947E38F, -2.8332313E38F}, 0) ;
        p64.az_SET(-3.1262039E38F) ;
        p64.z_SET(2.2805223E38F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan6_raw_GET() == (char)33738);
            assert(pack.chan11_raw_GET() == (char)23268);
            assert(pack.chan16_raw_GET() == (char)14697);
            assert(pack.chan18_raw_GET() == (char)47211);
            assert(pack.chan2_raw_GET() == (char)15413);
            assert(pack.chan7_raw_GET() == (char)58478);
            assert(pack.rssi_GET() == (char)138);
            assert(pack.chan12_raw_GET() == (char)44302);
            assert(pack.chan17_raw_GET() == (char)57069);
            assert(pack.chan14_raw_GET() == (char)17777);
            assert(pack.chancount_GET() == (char)226);
            assert(pack.chan4_raw_GET() == (char)53165);
            assert(pack.chan8_raw_GET() == (char)34320);
            assert(pack.chan10_raw_GET() == (char)49238);
            assert(pack.time_boot_ms_GET() == 1382049818L);
            assert(pack.chan15_raw_GET() == (char)8756);
            assert(pack.chan13_raw_GET() == (char)35386);
            assert(pack.chan9_raw_GET() == (char)35117);
            assert(pack.chan1_raw_GET() == (char)9564);
            assert(pack.chan3_raw_GET() == (char)54240);
            assert(pack.chan5_raw_GET() == (char)31360);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan15_raw_SET((char)8756) ;
        p65.chan3_raw_SET((char)54240) ;
        p65.chan2_raw_SET((char)15413) ;
        p65.chancount_SET((char)226) ;
        p65.chan11_raw_SET((char)23268) ;
        p65.chan10_raw_SET((char)49238) ;
        p65.chan17_raw_SET((char)57069) ;
        p65.chan9_raw_SET((char)35117) ;
        p65.chan13_raw_SET((char)35386) ;
        p65.time_boot_ms_SET(1382049818L) ;
        p65.chan6_raw_SET((char)33738) ;
        p65.chan5_raw_SET((char)31360) ;
        p65.chan8_raw_SET((char)34320) ;
        p65.chan18_raw_SET((char)47211) ;
        p65.chan14_raw_SET((char)17777) ;
        p65.rssi_SET((char)138) ;
        p65.chan12_raw_SET((char)44302) ;
        p65.chan4_raw_SET((char)53165) ;
        p65.chan1_raw_SET((char)9564) ;
        p65.chan16_raw_SET((char)14697) ;
        p65.chan7_raw_SET((char)58478) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.req_message_rate_GET() == (char)40851);
            assert(pack.start_stop_GET() == (char)109);
            assert(pack.target_system_GET() == (char)81);
            assert(pack.target_component_GET() == (char)135);
            assert(pack.req_stream_id_GET() == (char)133);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_stream_id_SET((char)133) ;
        p66.target_component_SET((char)135) ;
        p66.req_message_rate_SET((char)40851) ;
        p66.target_system_SET((char)81) ;
        p66.start_stop_SET((char)109) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.message_rate_GET() == (char)62064);
            assert(pack.stream_id_GET() == (char)91);
            assert(pack.on_off_GET() == (char)236);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)91) ;
        p67.message_rate_SET((char)62064) ;
        p67.on_off_SET((char)236) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_GET() == (char)231);
            assert(pack.z_GET() == (short) -20711);
            assert(pack.y_GET() == (short) -17285);
            assert(pack.buttons_GET() == (char)39282);
            assert(pack.x_GET() == (short)24557);
            assert(pack.r_GET() == (short) -23062);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.buttons_SET((char)39282) ;
        p69.target_SET((char)231) ;
        p69.z_SET((short) -20711) ;
        p69.y_SET((short) -17285) ;
        p69.x_SET((short)24557) ;
        p69.r_SET((short) -23062) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan6_raw_GET() == (char)41404);
            assert(pack.chan1_raw_GET() == (char)13200);
            assert(pack.chan3_raw_GET() == (char)35416);
            assert(pack.target_system_GET() == (char)216);
            assert(pack.chan5_raw_GET() == (char)44089);
            assert(pack.chan7_raw_GET() == (char)56480);
            assert(pack.chan2_raw_GET() == (char)9831);
            assert(pack.chan4_raw_GET() == (char)29946);
            assert(pack.target_component_GET() == (char)110);
            assert(pack.chan8_raw_GET() == (char)59852);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan6_raw_SET((char)41404) ;
        p70.target_component_SET((char)110) ;
        p70.chan1_raw_SET((char)13200) ;
        p70.chan4_raw_SET((char)29946) ;
        p70.chan8_raw_SET((char)59852) ;
        p70.chan7_raw_SET((char)56480) ;
        p70.chan2_raw_SET((char)9831) ;
        p70.chan3_raw_SET((char)35416) ;
        p70.target_system_SET((char)216) ;
        p70.chan5_raw_SET((char)44089) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param2_GET() == -1.428521E38F);
            assert(pack.current_GET() == (char)117);
            assert(pack.seq_GET() == (char)62262);
            assert(pack.param1_GET() == -4.1056303E37F);
            assert(pack.x_GET() == 89017821);
            assert(pack.param3_GET() == -2.0618552E38F);
            assert(pack.y_GET() == -1416085874);
            assert(pack.z_GET() == -2.0379074E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_GO_AROUND);
            assert(pack.param4_GET() == 9.00727E37F);
            assert(pack.target_system_GET() == (char)240);
            assert(pack.autocontinue_GET() == (char)149);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)156);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.param3_SET(-2.0618552E38F) ;
        p73.command_SET(MAV_CMD.MAV_CMD_DO_GO_AROUND) ;
        p73.param4_SET(9.00727E37F) ;
        p73.seq_SET((char)62262) ;
        p73.z_SET(-2.0379074E38F) ;
        p73.autocontinue_SET((char)149) ;
        p73.target_component_SET((char)156) ;
        p73.x_SET(89017821) ;
        p73.param1_SET(-4.1056303E37F) ;
        p73.param2_SET(-1.428521E38F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p73.target_system_SET((char)240) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p73.current_SET((char)117) ;
        p73.y_SET(-1416085874) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.throttle_GET() == (char)24087);
            assert(pack.climb_GET() == 1.6967905E38F);
            assert(pack.airspeed_GET() == 3.2028886E37F);
            assert(pack.heading_GET() == (short)362);
            assert(pack.groundspeed_GET() == 6.9223174E37F);
            assert(pack.alt_GET() == -5.228009E37F);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.heading_SET((short)362) ;
        p74.alt_SET(-5.228009E37F) ;
        p74.climb_SET(1.6967905E38F) ;
        p74.groundspeed_SET(6.9223174E37F) ;
        p74.throttle_SET((char)24087) ;
        p74.airspeed_SET(3.2028886E37F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.z_GET() == 1.5612524E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL);
            assert(pack.param4_GET() == 1.9601643E38F);
            assert(pack.param2_GET() == -1.332202E36F);
            assert(pack.y_GET() == -175194589);
            assert(pack.current_GET() == (char)148);
            assert(pack.target_component_GET() == (char)219);
            assert(pack.param1_GET() == -2.0519527E38F);
            assert(pack.target_system_GET() == (char)108);
            assert(pack.x_GET() == 1027693947);
            assert(pack.param3_GET() == 1.2907967E38F);
            assert(pack.autocontinue_GET() == (char)97);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.autocontinue_SET((char)97) ;
        p75.x_SET(1027693947) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p75.command_SET(MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL) ;
        p75.param4_SET(1.9601643E38F) ;
        p75.target_system_SET((char)108) ;
        p75.target_component_SET((char)219) ;
        p75.param3_SET(1.2907967E38F) ;
        p75.current_SET((char)148) ;
        p75.y_SET(-175194589) ;
        p75.z_SET(1.5612524E38F) ;
        p75.param1_SET(-2.0519527E38F) ;
        p75.param2_SET(-1.332202E36F) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param5_GET() == 1.1223306E38F);
            assert(pack.param2_GET() == -2.4321E38F);
            assert(pack.target_component_GET() == (char)112);
            assert(pack.param6_GET() == -2.9506852E38F);
            assert(pack.param3_GET() == 4.4554822E36F);
            assert(pack.param4_GET() == 2.308132E38F);
            assert(pack.param1_GET() == -2.0851037E38F);
            assert(pack.target_system_GET() == (char)161);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_PATHPLANNING);
            assert(pack.confirmation_GET() == (char)69);
            assert(pack.param7_GET() == 3.3580544E37F);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.target_system_SET((char)161) ;
        p76.param1_SET(-2.0851037E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_NAV_PATHPLANNING) ;
        p76.target_component_SET((char)112) ;
        p76.param4_SET(2.308132E38F) ;
        p76.param2_SET(-2.4321E38F) ;
        p76.param5_SET(1.1223306E38F) ;
        p76.param3_SET(4.4554822E36F) ;
        p76.param7_SET(3.3580544E37F) ;
        p76.confirmation_SET((char)69) ;
        p76.param6_SET(-2.9506852E38F) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.result_param2_TRY(ph) == -840080912);
            assert(pack.target_system_TRY(ph) == (char)135);
            assert(pack.progress_TRY(ph) == (char)46);
            assert(pack.target_component_TRY(ph) == (char)18);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_IN_PROGRESS);
        });
        COMMAND_ACK p77 = new COMMAND_ACK();
        PH.setPack(p77);
        p77.result_param2_SET(-840080912, PH) ;
        p77.target_component_SET((char)18, PH) ;
        p77.target_system_SET((char)135, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_IN_PROGRESS) ;
        p77.command_SET(MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION) ;
        p77.progress_SET((char)46, PH) ;
        TestChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -6.260516E37F);
            assert(pack.thrust_GET() == -6.467213E37F);
            assert(pack.pitch_GET() == 1.0793541E38F);
            assert(pack.manual_override_switch_GET() == (char)108);
            assert(pack.time_boot_ms_GET() == 460054067L);
            assert(pack.yaw_GET() == 2.2606107E37F);
            assert(pack.mode_switch_GET() == (char)114);
        });
        MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.mode_switch_SET((char)114) ;
        p81.thrust_SET(-6.467213E37F) ;
        p81.manual_override_switch_SET((char)108) ;
        p81.pitch_SET(1.0793541E38F) ;
        p81.time_boot_ms_SET(460054067L) ;
        p81.yaw_SET(2.2606107E37F) ;
        p81.roll_SET(-6.260516E37F) ;
        TestChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.0193154E38F, 2.461973E38F, -4.4747676E37F, -8.759562E37F}));
            assert(pack.thrust_GET() == 4.7711265E37F);
            assert(pack.target_component_GET() == (char)83);
            assert(pack.type_mask_GET() == (char)196);
            assert(pack.target_system_GET() == (char)203);
            assert(pack.time_boot_ms_GET() == 2330946122L);
            assert(pack.body_yaw_rate_GET() == -3.1623047E38F);
            assert(pack.body_roll_rate_GET() == -8.0944916E37F);
            assert(pack.body_pitch_rate_GET() == 3.1568924E37F);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.body_yaw_rate_SET(-3.1623047E38F) ;
        p82.type_mask_SET((char)196) ;
        p82.time_boot_ms_SET(2330946122L) ;
        p82.q_SET(new float[] {1.0193154E38F, 2.461973E38F, -4.4747676E37F, -8.759562E37F}, 0) ;
        p82.body_pitch_rate_SET(3.1568924E37F) ;
        p82.thrust_SET(4.7711265E37F) ;
        p82.target_component_SET((char)83) ;
        p82.body_roll_rate_SET(-8.0944916E37F) ;
        p82.target_system_SET((char)203) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_roll_rate_GET() == -1.3472276E38F);
            assert(pack.type_mask_GET() == (char)76);
            assert(pack.body_pitch_rate_GET() == 2.5534286E38F);
            assert(pack.thrust_GET() == -1.7276816E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.7241026E38F, -2.8476039E38F, 6.569035E37F, -8.939354E37F}));
            assert(pack.body_yaw_rate_GET() == -1.637528E38F);
            assert(pack.time_boot_ms_GET() == 3206094424L);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.thrust_SET(-1.7276816E38F) ;
        p83.type_mask_SET((char)76) ;
        p83.body_yaw_rate_SET(-1.637528E38F) ;
        p83.time_boot_ms_SET(3206094424L) ;
        p83.body_pitch_rate_SET(2.5534286E38F) ;
        p83.body_roll_rate_SET(-1.3472276E38F) ;
        p83.q_SET(new float[] {-2.7241026E38F, -2.8476039E38F, 6.569035E37F, -8.939354E37F}, 0) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1180492731L);
            assert(pack.z_GET() == 2.6653687E37F);
            assert(pack.afy_GET() == 7.324847E37F);
            assert(pack.y_GET() == -4.739509E37F);
            assert(pack.yaw_GET() == -2.9609048E38F);
            assert(pack.vz_GET() == 1.5807754E38F);
            assert(pack.type_mask_GET() == (char)58349);
            assert(pack.afz_GET() == -7.012191E37F);
            assert(pack.target_component_GET() == (char)86);
            assert(pack.afx_GET() == -3.3772843E38F);
            assert(pack.yaw_rate_GET() == 7.5445985E37F);
            assert(pack.x_GET() == -9.166268E37F);
            assert(pack.vy_GET() == -1.5340956E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.vx_GET() == -1.8840087E38F);
            assert(pack.target_system_GET() == (char)70);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.afx_SET(-3.3772843E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p84.yaw_SET(-2.9609048E38F) ;
        p84.time_boot_ms_SET(1180492731L) ;
        p84.vy_SET(-1.5340956E38F) ;
        p84.target_component_SET((char)86) ;
        p84.afz_SET(-7.012191E37F) ;
        p84.vz_SET(1.5807754E38F) ;
        p84.afy_SET(7.324847E37F) ;
        p84.y_SET(-4.739509E37F) ;
        p84.x_SET(-9.166268E37F) ;
        p84.target_system_SET((char)70) ;
        p84.vx_SET(-1.8840087E38F) ;
        p84.type_mask_SET((char)58349) ;
        p84.z_SET(2.6653687E37F) ;
        p84.yaw_rate_SET(7.5445985E37F) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == 6.585743E37F);
            assert(pack.afx_GET() == -2.7145575E38F);
            assert(pack.time_boot_ms_GET() == 3127152208L);
            assert(pack.afz_GET() == 1.4671378E38F);
            assert(pack.vz_GET() == 1.5777108E38F);
            assert(pack.alt_GET() == 1.1640142E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.vy_GET() == -1.9722261E37F);
            assert(pack.target_component_GET() == (char)248);
            assert(pack.lon_int_GET() == 182531551);
            assert(pack.afy_GET() == -1.0442272E38F);
            assert(pack.type_mask_GET() == (char)46278);
            assert(pack.lat_int_GET() == -1267974607);
            assert(pack.yaw_GET() == -1.7790306E38F);
            assert(pack.target_system_GET() == (char)155);
            assert(pack.vx_GET() == -2.111329E38F);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.vx_SET(-2.111329E38F) ;
        p86.vz_SET(1.5777108E38F) ;
        p86.yaw_rate_SET(6.585743E37F) ;
        p86.lat_int_SET(-1267974607) ;
        p86.time_boot_ms_SET(3127152208L) ;
        p86.afz_SET(1.4671378E38F) ;
        p86.alt_SET(1.1640142E38F) ;
        p86.afx_SET(-2.7145575E38F) ;
        p86.vy_SET(-1.9722261E37F) ;
        p86.target_component_SET((char)248) ;
        p86.target_system_SET((char)155) ;
        p86.lon_int_SET(182531551) ;
        p86.yaw_SET(-1.7790306E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p86.afy_SET(-1.0442272E38F) ;
        p86.type_mask_SET((char)46278) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 2.3750698E38F);
            assert(pack.afz_GET() == -2.0017288E38F);
            assert(pack.vz_GET() == 1.5733039E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.alt_GET() == 1.968836E38F);
            assert(pack.type_mask_GET() == (char)16379);
            assert(pack.lon_int_GET() == 655980732);
            assert(pack.yaw_rate_GET() == -9.778877E37F);
            assert(pack.time_boot_ms_GET() == 3328853456L);
            assert(pack.vx_GET() == 1.7574961E38F);
            assert(pack.lat_int_GET() == 2117746329);
            assert(pack.afy_GET() == -1.1636228E38F);
            assert(pack.yaw_GET() == 3.3997754E38F);
            assert(pack.afx_GET() == 2.560276E38F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.vy_SET(2.3750698E38F) ;
        p87.yaw_rate_SET(-9.778877E37F) ;
        p87.afy_SET(-1.1636228E38F) ;
        p87.yaw_SET(3.3997754E38F) ;
        p87.type_mask_SET((char)16379) ;
        p87.alt_SET(1.968836E38F) ;
        p87.vx_SET(1.7574961E38F) ;
        p87.afx_SET(2.560276E38F) ;
        p87.vz_SET(1.5733039E38F) ;
        p87.lon_int_SET(655980732) ;
        p87.time_boot_ms_SET(3328853456L) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p87.afz_SET(-2.0017288E38F) ;
        p87.lat_int_SET(2117746329) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1770289280L);
            assert(pack.y_GET() == -5.55518E37F);
            assert(pack.z_GET() == 1.7153625E38F);
            assert(pack.pitch_GET() == 1.9791768E38F);
            assert(pack.yaw_GET() == -2.7904125E38F);
            assert(pack.x_GET() == -3.0465652E38F);
            assert(pack.roll_GET() == 1.6212195E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.z_SET(1.7153625E38F) ;
        p89.y_SET(-5.55518E37F) ;
        p89.time_boot_ms_SET(1770289280L) ;
        p89.x_SET(-3.0465652E38F) ;
        p89.pitch_SET(1.9791768E38F) ;
        p89.roll_SET(1.6212195E38F) ;
        p89.yaw_SET(-2.7904125E38F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == (short)23098);
            assert(pack.yawspeed_GET() == -1.1522883E38F);
            assert(pack.zacc_GET() == (short)20509);
            assert(pack.lat_GET() == 289274886);
            assert(pack.vy_GET() == (short)24407);
            assert(pack.pitch_GET() == -2.5359736E38F);
            assert(pack.lon_GET() == -912907701);
            assert(pack.rollspeed_GET() == 1.1231597E37F);
            assert(pack.time_usec_GET() == 2227591394120182880L);
            assert(pack.vz_GET() == (short)6062);
            assert(pack.yacc_GET() == (short)30517);
            assert(pack.pitchspeed_GET() == 3.8339863E35F);
            assert(pack.xacc_GET() == (short) -23949);
            assert(pack.yaw_GET() == -1.3405189E38F);
            assert(pack.roll_GET() == 1.1697346E38F);
            assert(pack.alt_GET() == 1660187716);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.zacc_SET((short)20509) ;
        p90.lat_SET(289274886) ;
        p90.vz_SET((short)6062) ;
        p90.time_usec_SET(2227591394120182880L) ;
        p90.lon_SET(-912907701) ;
        p90.roll_SET(1.1697346E38F) ;
        p90.pitchspeed_SET(3.8339863E35F) ;
        p90.yacc_SET((short)30517) ;
        p90.xacc_SET((short) -23949) ;
        p90.pitch_SET(-2.5359736E38F) ;
        p90.yawspeed_SET(-1.1522883E38F) ;
        p90.alt_SET(1660187716) ;
        p90.rollspeed_SET(1.1231597E37F) ;
        p90.vx_SET((short)23098) ;
        p90.vy_SET((short)24407) ;
        p90.yaw_SET(-1.3405189E38F) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.nav_mode_GET() == (char)108);
            assert(pack.aux3_GET() == -1.2544781E38F);
            assert(pack.pitch_elevator_GET() == -1.8251564E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_AUTO_DISARMED);
            assert(pack.roll_ailerons_GET() == -7.440653E37F);
            assert(pack.aux1_GET() == -2.152968E38F);
            assert(pack.aux4_GET() == -2.3469277E38F);
            assert(pack.time_usec_GET() == 3936328336277473352L);
            assert(pack.throttle_GET() == -2.435987E38F);
            assert(pack.aux2_GET() == 6.7945626E37F);
            assert(pack.yaw_rudder_GET() == 1.9673511E38F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.nav_mode_SET((char)108) ;
        p91.roll_ailerons_SET(-7.440653E37F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED) ;
        p91.pitch_elevator_SET(-1.8251564E38F) ;
        p91.aux4_SET(-2.3469277E38F) ;
        p91.throttle_SET(-2.435987E38F) ;
        p91.aux3_SET(-1.2544781E38F) ;
        p91.aux2_SET(6.7945626E37F) ;
        p91.aux1_SET(-2.152968E38F) ;
        p91.yaw_rudder_SET(1.9673511E38F) ;
        p91.time_usec_SET(3936328336277473352L) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan6_raw_GET() == (char)46235);
            assert(pack.chan7_raw_GET() == (char)19818);
            assert(pack.chan3_raw_GET() == (char)60647);
            assert(pack.chan2_raw_GET() == (char)40461);
            assert(pack.rssi_GET() == (char)63);
            assert(pack.chan5_raw_GET() == (char)48371);
            assert(pack.chan8_raw_GET() == (char)18425);
            assert(pack.chan1_raw_GET() == (char)44921);
            assert(pack.chan9_raw_GET() == (char)38083);
            assert(pack.chan10_raw_GET() == (char)2463);
            assert(pack.time_usec_GET() == 5863770766034197318L);
            assert(pack.chan4_raw_GET() == (char)24062);
            assert(pack.chan12_raw_GET() == (char)7275);
            assert(pack.chan11_raw_GET() == (char)13057);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.rssi_SET((char)63) ;
        p92.chan8_raw_SET((char)18425) ;
        p92.chan10_raw_SET((char)2463) ;
        p92.chan5_raw_SET((char)48371) ;
        p92.chan1_raw_SET((char)44921) ;
        p92.chan11_raw_SET((char)13057) ;
        p92.chan3_raw_SET((char)60647) ;
        p92.chan12_raw_SET((char)7275) ;
        p92.chan6_raw_SET((char)46235) ;
        p92.time_usec_SET(5863770766034197318L) ;
        p92.chan7_raw_SET((char)19818) ;
        p92.chan9_raw_SET((char)38083) ;
        p92.chan2_raw_SET((char)40461) ;
        p92.chan4_raw_SET((char)24062) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.0976713E37F, -4.98987E37F, 3.2713977E38F, -2.2954086E38F, -1.7867103E38F, -7.4476364E37F, -2.9186047E38F, 8.019071E37F, 1.4812882E38F, -2.3879501E38F, 3.362199E38F, -5.590714E37F, -2.1661297E38F, 2.5272935E37F, 1.567545E38F, 1.7285907E38F}));
            assert(pack.time_usec_GET() == 6607750706020316426L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_ARMED);
            assert(pack.flags_GET() == 5229340716857476284L);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.mode_SET(MAV_MODE.MAV_MODE_STABILIZE_ARMED) ;
        p93.flags_SET(5229340716857476284L) ;
        p93.controls_SET(new float[] {-2.0976713E37F, -4.98987E37F, 3.2713977E38F, -2.2954086E38F, -1.7867103E38F, -7.4476364E37F, -2.9186047E38F, 8.019071E37F, 1.4812882E38F, -2.3879501E38F, 3.362199E38F, -5.590714E37F, -2.1661297E38F, 2.5272935E37F, 1.567545E38F, 1.7285907E38F}, 0) ;
        p93.time_usec_SET(6607750706020316426L) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_comp_m_x_GET() == 3.3457847E38F);
            assert(pack.flow_x_GET() == (short) -14475);
            assert(pack.flow_y_GET() == (short) -30778);
            assert(pack.time_usec_GET() == 4286819627384689210L);
            assert(pack.flow_comp_m_y_GET() == -7.004144E37F);
            assert(pack.ground_distance_GET() == -2.4311928E38F);
            assert(pack.quality_GET() == (char)250);
            assert(pack.flow_rate_y_TRY(ph) == -9.908159E36F);
            assert(pack.sensor_id_GET() == (char)197);
            assert(pack.flow_rate_x_TRY(ph) == 7.06442E37F);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(4286819627384689210L) ;
        p100.flow_x_SET((short) -14475) ;
        p100.flow_rate_x_SET(7.06442E37F, PH) ;
        p100.flow_comp_m_x_SET(3.3457847E38F) ;
        p100.flow_comp_m_y_SET(-7.004144E37F) ;
        p100.quality_SET((char)250) ;
        p100.sensor_id_SET((char)197) ;
        p100.flow_rate_y_SET(-9.908159E36F, PH) ;
        p100.ground_distance_SET(-2.4311928E38F) ;
        p100.flow_y_SET((short) -30778) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -3.0859341E38F);
            assert(pack.z_GET() == -3.366597E38F);
            assert(pack.yaw_GET() == 2.6849377E37F);
            assert(pack.usec_GET() == 1554223296284700265L);
            assert(pack.y_GET() == -1.9874068E38F);
            assert(pack.pitch_GET() == -1.1496673E38F);
            assert(pack.x_GET() == -3.2120228E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.yaw_SET(2.6849377E37F) ;
        p101.pitch_SET(-1.1496673E38F) ;
        p101.roll_SET(-3.0859341E38F) ;
        p101.x_SET(-3.2120228E38F) ;
        p101.z_SET(-3.366597E38F) ;
        p101.usec_SET(1554223296284700265L) ;
        p101.y_SET(-1.9874068E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -1.346074E38F);
            assert(pack.y_GET() == -4.73667E37F);
            assert(pack.roll_GET() == 9.453444E37F);
            assert(pack.pitch_GET() == 1.3519388E38F);
            assert(pack.usec_GET() == 3600164374757560829L);
            assert(pack.yaw_GET() == 1.4544106E38F);
            assert(pack.z_GET() == -1.6699971E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.roll_SET(9.453444E37F) ;
        p102.z_SET(-1.6699971E38F) ;
        p102.usec_SET(3600164374757560829L) ;
        p102.x_SET(-1.346074E38F) ;
        p102.y_SET(-4.73667E37F) ;
        p102.pitch_SET(1.3519388E38F) ;
        p102.yaw_SET(1.4544106E38F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -1.9586952E38F);
            assert(pack.y_GET() == 2.1848295E38F);
            assert(pack.z_GET() == -2.2778345E38F);
            assert(pack.usec_GET() == 7808667014485316243L);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.y_SET(2.1848295E38F) ;
        p103.usec_SET(7808667014485316243L) ;
        p103.x_SET(-1.9586952E38F) ;
        p103.z_SET(-2.2778345E38F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 2.903156E38F);
            assert(pack.yaw_GET() == 8.525304E37F);
            assert(pack.z_GET() == 9.025754E37F);
            assert(pack.usec_GET() == 8349286996170636769L);
            assert(pack.roll_GET() == 4.693543E37F);
            assert(pack.x_GET() == 2.2213046E38F);
            assert(pack.y_GET() == 1.5794382E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.pitch_SET(2.903156E38F) ;
        p104.z_SET(9.025754E37F) ;
        p104.y_SET(1.5794382E38F) ;
        p104.x_SET(2.2213046E38F) ;
        p104.yaw_SET(8.525304E37F) ;
        p104.roll_SET(4.693543E37F) ;
        p104.usec_SET(8349286996170636769L) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == -2.0508222E38F);
            assert(pack.zacc_GET() == 1.5104558E38F);
            assert(pack.abs_pressure_GET() == -1.1753579E38F);
            assert(pack.xacc_GET() == -2.403253E38F);
            assert(pack.fields_updated_GET() == (char)32823);
            assert(pack.ygyro_GET() == -3.0225311E38F);
            assert(pack.diff_pressure_GET() == 2.2979212E38F);
            assert(pack.xgyro_GET() == -2.027355E38F);
            assert(pack.temperature_GET() == -7.385497E37F);
            assert(pack.ymag_GET() == 3.2002584E38F);
            assert(pack.yacc_GET() == 1.6467127E38F);
            assert(pack.time_usec_GET() == 1987476409743003389L);
            assert(pack.pressure_alt_GET() == -1.2858161E38F);
            assert(pack.zmag_GET() == 3.3764784E38F);
            assert(pack.zgyro_GET() == -2.5715049E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.zacc_SET(1.5104558E38F) ;
        p105.zmag_SET(3.3764784E38F) ;
        p105.time_usec_SET(1987476409743003389L) ;
        p105.xacc_SET(-2.403253E38F) ;
        p105.ymag_SET(3.2002584E38F) ;
        p105.zgyro_SET(-2.5715049E38F) ;
        p105.xmag_SET(-2.0508222E38F) ;
        p105.diff_pressure_SET(2.2979212E38F) ;
        p105.temperature_SET(-7.385497E37F) ;
        p105.xgyro_SET(-2.027355E38F) ;
        p105.ygyro_SET(-3.0225311E38F) ;
        p105.pressure_alt_SET(-1.2858161E38F) ;
        p105.abs_pressure_SET(-1.1753579E38F) ;
        p105.fields_updated_SET((char)32823) ;
        p105.yacc_SET(1.6467127E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 8389483599634864606L);
            assert(pack.integrated_y_GET() == -8.534898E37F);
            assert(pack.integration_time_us_GET() == 175965068L);
            assert(pack.integrated_x_GET() == 1.1779266E38F);
            assert(pack.temperature_GET() == (short)18757);
            assert(pack.time_delta_distance_us_GET() == 2858707296L);
            assert(pack.integrated_xgyro_GET() == 3.080029E38F);
            assert(pack.sensor_id_GET() == (char)138);
            assert(pack.distance_GET() == 5.4791723E37F);
            assert(pack.integrated_zgyro_GET() == -2.6835295E38F);
            assert(pack.integrated_ygyro_GET() == 1.96061E38F);
            assert(pack.quality_GET() == (char)89);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.sensor_id_SET((char)138) ;
        p106.integration_time_us_SET(175965068L) ;
        p106.temperature_SET((short)18757) ;
        p106.time_usec_SET(8389483599634864606L) ;
        p106.integrated_xgyro_SET(3.080029E38F) ;
        p106.distance_SET(5.4791723E37F) ;
        p106.integrated_x_SET(1.1779266E38F) ;
        p106.time_delta_distance_us_SET(2858707296L) ;
        p106.integrated_zgyro_SET(-2.6835295E38F) ;
        p106.integrated_y_SET(-8.534898E37F) ;
        p106.integrated_ygyro_SET(1.96061E38F) ;
        p106.quality_SET((char)89) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.pressure_alt_GET() == -9.349723E37F);
            assert(pack.xmag_GET() == 2.465911E38F);
            assert(pack.diff_pressure_GET() == -1.8094625E38F);
            assert(pack.ymag_GET() == -1.4015112E38F);
            assert(pack.xgyro_GET() == 3.402714E38F);
            assert(pack.abs_pressure_GET() == 5.766581E37F);
            assert(pack.temperature_GET() == -2.247262E35F);
            assert(pack.xacc_GET() == 2.6864694E38F);
            assert(pack.yacc_GET() == -2.8287086E38F);
            assert(pack.zmag_GET() == -2.8389942E38F);
            assert(pack.zgyro_GET() == -1.1145009E38F);
            assert(pack.fields_updated_GET() == 3699890815L);
            assert(pack.zacc_GET() == -1.2787546E38F);
            assert(pack.time_usec_GET() == 607600292501663699L);
            assert(pack.ygyro_GET() == 2.6545007E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.abs_pressure_SET(5.766581E37F) ;
        p107.xacc_SET(2.6864694E38F) ;
        p107.zacc_SET(-1.2787546E38F) ;
        p107.zmag_SET(-2.8389942E38F) ;
        p107.yacc_SET(-2.8287086E38F) ;
        p107.pressure_alt_SET(-9.349723E37F) ;
        p107.xmag_SET(2.465911E38F) ;
        p107.zgyro_SET(-1.1145009E38F) ;
        p107.diff_pressure_SET(-1.8094625E38F) ;
        p107.ymag_SET(-1.4015112E38F) ;
        p107.xgyro_SET(3.402714E38F) ;
        p107.ygyro_SET(2.6545007E38F) ;
        p107.time_usec_SET(607600292501663699L) ;
        p107.temperature_SET(-2.247262E35F) ;
        p107.fields_updated_SET(3699890815L) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.std_dev_horz_GET() == 1.7951853E38F);
            assert(pack.vn_GET() == -1.0218531E38F);
            assert(pack.alt_GET() == 3.1917684E38F);
            assert(pack.yacc_GET() == -1.4745467E38F);
            assert(pack.xacc_GET() == -2.600352E38F);
            assert(pack.q2_GET() == -5.2295054E37F);
            assert(pack.ve_GET() == 2.2850722E38F);
            assert(pack.ygyro_GET() == 1.5843645E38F);
            assert(pack.lat_GET() == -2.7473869E38F);
            assert(pack.q4_GET() == -1.3853797E38F);
            assert(pack.xgyro_GET() == -3.0838077E38F);
            assert(pack.zacc_GET() == 1.5361724E38F);
            assert(pack.vd_GET() == -2.5672963E38F);
            assert(pack.pitch_GET() == -4.1300275E37F);
            assert(pack.q3_GET() == 3.074506E38F);
            assert(pack.std_dev_vert_GET() == 2.7435257E38F);
            assert(pack.zgyro_GET() == 1.0960515E38F);
            assert(pack.yaw_GET() == -1.8431901E38F);
            assert(pack.q1_GET() == -7.2222887E37F);
            assert(pack.roll_GET() == -2.7498526E36F);
            assert(pack.lon_GET() == -3.0393493E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.q4_SET(-1.3853797E38F) ;
        p108.xacc_SET(-2.600352E38F) ;
        p108.xgyro_SET(-3.0838077E38F) ;
        p108.lon_SET(-3.0393493E38F) ;
        p108.yacc_SET(-1.4745467E38F) ;
        p108.ygyro_SET(1.5843645E38F) ;
        p108.yaw_SET(-1.8431901E38F) ;
        p108.zacc_SET(1.5361724E38F) ;
        p108.ve_SET(2.2850722E38F) ;
        p108.q1_SET(-7.2222887E37F) ;
        p108.std_dev_horz_SET(1.7951853E38F) ;
        p108.vn_SET(-1.0218531E38F) ;
        p108.lat_SET(-2.7473869E38F) ;
        p108.q2_SET(-5.2295054E37F) ;
        p108.roll_SET(-2.7498526E36F) ;
        p108.alt_SET(3.1917684E38F) ;
        p108.pitch_SET(-4.1300275E37F) ;
        p108.q3_SET(3.074506E38F) ;
        p108.std_dev_vert_SET(2.7435257E38F) ;
        p108.zgyro_SET(1.0960515E38F) ;
        p108.vd_SET(-2.5672963E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.remrssi_GET() == (char)112);
            assert(pack.fixed__GET() == (char)47477);
            assert(pack.rssi_GET() == (char)87);
            assert(pack.txbuf_GET() == (char)117);
            assert(pack.noise_GET() == (char)138);
            assert(pack.rxerrors_GET() == (char)5609);
            assert(pack.remnoise_GET() == (char)167);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.remrssi_SET((char)112) ;
        p109.fixed__SET((char)47477) ;
        p109.remnoise_SET((char)167) ;
        p109.noise_SET((char)138) ;
        p109.rssi_SET((char)87) ;
        p109.txbuf_SET((char)117) ;
        p109.rxerrors_SET((char)5609) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)212);
            assert(pack.target_network_GET() == (char)170);
            assert(pack.target_system_GET() == (char)30);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)118, (char)172, (char)251, (char)101, (char)0, (char)80, (char)248, (char)155, (char)41, (char)209, (char)181, (char)241, (char)100, (char)5, (char)43, (char)172, (char)174, (char)239, (char)151, (char)186, (char)130, (char)45, (char)139, (char)154, (char)116, (char)127, (char)203, (char)20, (char)230, (char)163, (char)207, (char)225, (char)221, (char)14, (char)88, (char)139, (char)13, (char)54, (char)182, (char)87, (char)125, (char)102, (char)198, (char)22, (char)255, (char)209, (char)39, (char)137, (char)213, (char)75, (char)47, (char)155, (char)31, (char)162, (char)189, (char)245, (char)40, (char)40, (char)63, (char)40, (char)246, (char)92, (char)33, (char)211, (char)165, (char)34, (char)64, (char)91, (char)61, (char)211, (char)220, (char)178, (char)102, (char)86, (char)137, (char)246, (char)36, (char)143, (char)212, (char)211, (char)94, (char)22, (char)27, (char)178, (char)108, (char)229, (char)71, (char)129, (char)219, (char)127, (char)16, (char)142, (char)142, (char)14, (char)173, (char)32, (char)169, (char)6, (char)226, (char)148, (char)39, (char)177, (char)1, (char)99, (char)79, (char)89, (char)164, (char)175, (char)194, (char)49, (char)188, (char)34, (char)193, (char)97, (char)154, (char)241, (char)99, (char)2, (char)68, (char)187, (char)50, (char)31, (char)50, (char)32, (char)227, (char)140, (char)101, (char)65, (char)18, (char)79, (char)17, (char)232, (char)172, (char)40, (char)246, (char)58, (char)62, (char)112, (char)254, (char)73, (char)67, (char)213, (char)170, (char)249, (char)149, (char)126, (char)37, (char)104, (char)16, (char)192, (char)132, (char)102, (char)18, (char)201, (char)109, (char)51, (char)225, (char)45, (char)233, (char)19, (char)166, (char)43, (char)6, (char)74, (char)135, (char)20, (char)48, (char)200, (char)143, (char)12, (char)52, (char)12, (char)236, (char)41, (char)99, (char)210, (char)250, (char)116, (char)141, (char)238, (char)65, (char)142, (char)165, (char)40, (char)252, (char)93, (char)179, (char)52, (char)253, (char)127, (char)86, (char)157, (char)217, (char)79, (char)34, (char)180, (char)211, (char)134, (char)83, (char)228, (char)159, (char)132, (char)6, (char)58, (char)69, (char)110, (char)45, (char)187, (char)116, (char)121, (char)225, (char)237, (char)95, (char)141, (char)76, (char)195, (char)131, (char)88, (char)4, (char)156, (char)29, (char)10, (char)188, (char)48, (char)241, (char)211, (char)168, (char)87, (char)13, (char)103, (char)56, (char)216, (char)224, (char)130, (char)80, (char)107, (char)157, (char)51, (char)36, (char)78, (char)37, (char)252, (char)224, (char)224, (char)46, (char)199, (char)52, (char)204, (char)230, (char)206, (char)5}));
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.payload_SET(new char[] {(char)118, (char)172, (char)251, (char)101, (char)0, (char)80, (char)248, (char)155, (char)41, (char)209, (char)181, (char)241, (char)100, (char)5, (char)43, (char)172, (char)174, (char)239, (char)151, (char)186, (char)130, (char)45, (char)139, (char)154, (char)116, (char)127, (char)203, (char)20, (char)230, (char)163, (char)207, (char)225, (char)221, (char)14, (char)88, (char)139, (char)13, (char)54, (char)182, (char)87, (char)125, (char)102, (char)198, (char)22, (char)255, (char)209, (char)39, (char)137, (char)213, (char)75, (char)47, (char)155, (char)31, (char)162, (char)189, (char)245, (char)40, (char)40, (char)63, (char)40, (char)246, (char)92, (char)33, (char)211, (char)165, (char)34, (char)64, (char)91, (char)61, (char)211, (char)220, (char)178, (char)102, (char)86, (char)137, (char)246, (char)36, (char)143, (char)212, (char)211, (char)94, (char)22, (char)27, (char)178, (char)108, (char)229, (char)71, (char)129, (char)219, (char)127, (char)16, (char)142, (char)142, (char)14, (char)173, (char)32, (char)169, (char)6, (char)226, (char)148, (char)39, (char)177, (char)1, (char)99, (char)79, (char)89, (char)164, (char)175, (char)194, (char)49, (char)188, (char)34, (char)193, (char)97, (char)154, (char)241, (char)99, (char)2, (char)68, (char)187, (char)50, (char)31, (char)50, (char)32, (char)227, (char)140, (char)101, (char)65, (char)18, (char)79, (char)17, (char)232, (char)172, (char)40, (char)246, (char)58, (char)62, (char)112, (char)254, (char)73, (char)67, (char)213, (char)170, (char)249, (char)149, (char)126, (char)37, (char)104, (char)16, (char)192, (char)132, (char)102, (char)18, (char)201, (char)109, (char)51, (char)225, (char)45, (char)233, (char)19, (char)166, (char)43, (char)6, (char)74, (char)135, (char)20, (char)48, (char)200, (char)143, (char)12, (char)52, (char)12, (char)236, (char)41, (char)99, (char)210, (char)250, (char)116, (char)141, (char)238, (char)65, (char)142, (char)165, (char)40, (char)252, (char)93, (char)179, (char)52, (char)253, (char)127, (char)86, (char)157, (char)217, (char)79, (char)34, (char)180, (char)211, (char)134, (char)83, (char)228, (char)159, (char)132, (char)6, (char)58, (char)69, (char)110, (char)45, (char)187, (char)116, (char)121, (char)225, (char)237, (char)95, (char)141, (char)76, (char)195, (char)131, (char)88, (char)4, (char)156, (char)29, (char)10, (char)188, (char)48, (char)241, (char)211, (char)168, (char)87, (char)13, (char)103, (char)56, (char)216, (char)224, (char)130, (char)80, (char)107, (char)157, (char)51, (char)36, (char)78, (char)37, (char)252, (char)224, (char)224, (char)46, (char)199, (char)52, (char)204, (char)230, (char)206, (char)5}, 0) ;
        p110.target_network_SET((char)170) ;
        p110.target_component_SET((char)212) ;
        p110.target_system_SET((char)30) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -3525575858305899458L);
            assert(pack.ts1_GET() == 5295143252766679171L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(5295143252766679171L) ;
        p111.tc1_SET(-3525575858305899458L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 1209965450804617588L);
            assert(pack.seq_GET() == 4046378615L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(4046378615L) ;
        p112.time_usec_SET(1209965450804617588L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.epv_GET() == (char)22318);
            assert(pack.ve_GET() == (short) -23528);
            assert(pack.lat_GET() == -507788699);
            assert(pack.fix_type_GET() == (char)115);
            assert(pack.vn_GET() == (short) -24811);
            assert(pack.eph_GET() == (char)9929);
            assert(pack.vd_GET() == (short)18750);
            assert(pack.time_usec_GET() == 5855431219137714313L);
            assert(pack.vel_GET() == (char)8693);
            assert(pack.cog_GET() == (char)12408);
            assert(pack.satellites_visible_GET() == (char)20);
            assert(pack.alt_GET() == -513882560);
            assert(pack.lon_GET() == -394910142);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.vd_SET((short)18750) ;
        p113.cog_SET((char)12408) ;
        p113.vel_SET((char)8693) ;
        p113.lat_SET(-507788699) ;
        p113.fix_type_SET((char)115) ;
        p113.time_usec_SET(5855431219137714313L) ;
        p113.lon_SET(-394910142) ;
        p113.eph_SET((char)9929) ;
        p113.ve_SET((short) -23528) ;
        p113.alt_SET(-513882560) ;
        p113.satellites_visible_SET((char)20) ;
        p113.vn_SET((short) -24811) ;
        p113.epv_SET((char)22318) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_zgyro_GET() == -3.330185E38F);
            assert(pack.integrated_ygyro_GET() == -3.3632847E38F);
            assert(pack.sensor_id_GET() == (char)143);
            assert(pack.integrated_x_GET() == -7.313084E37F);
            assert(pack.distance_GET() == 3.2611685E38F);
            assert(pack.integration_time_us_GET() == 2475536971L);
            assert(pack.integrated_y_GET() == -2.5375836E38F);
            assert(pack.temperature_GET() == (short)24536);
            assert(pack.integrated_xgyro_GET() == -2.626271E38F);
            assert(pack.quality_GET() == (char)146);
            assert(pack.time_delta_distance_us_GET() == 3142933606L);
            assert(pack.time_usec_GET() == 7803116523133569356L);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_xgyro_SET(-2.626271E38F) ;
        p114.integration_time_us_SET(2475536971L) ;
        p114.integrated_y_SET(-2.5375836E38F) ;
        p114.quality_SET((char)146) ;
        p114.sensor_id_SET((char)143) ;
        p114.integrated_zgyro_SET(-3.330185E38F) ;
        p114.integrated_x_SET(-7.313084E37F) ;
        p114.integrated_ygyro_SET(-3.3632847E38F) ;
        p114.distance_SET(3.2611685E38F) ;
        p114.time_usec_SET(7803116523133569356L) ;
        p114.time_delta_distance_us_SET(3142933606L) ;
        p114.temperature_SET((short)24536) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.true_airspeed_GET() == (char)46702);
            assert(pack.alt_GET() == -1754874224);
            assert(pack.time_usec_GET() == 7566960063809434968L);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {-3.161862E38F, 8.87611E37F, 2.8778715E38F, -1.3333876E38F}));
            assert(pack.yacc_GET() == (short) -18969);
            assert(pack.xacc_GET() == (short)3157);
            assert(pack.vx_GET() == (short) -23421);
            assert(pack.vz_GET() == (short) -7853);
            assert(pack.ind_airspeed_GET() == (char)55245);
            assert(pack.pitchspeed_GET() == 4.679657E37F);
            assert(pack.vy_GET() == (short) -10314);
            assert(pack.yawspeed_GET() == -2.499825E38F);
            assert(pack.rollspeed_GET() == 5.6371043E37F);
            assert(pack.zacc_GET() == (short)12841);
            assert(pack.lat_GET() == 2031570950);
            assert(pack.lon_GET() == 1317087097);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.vz_SET((short) -7853) ;
        p115.yawspeed_SET(-2.499825E38F) ;
        p115.time_usec_SET(7566960063809434968L) ;
        p115.lon_SET(1317087097) ;
        p115.lat_SET(2031570950) ;
        p115.rollspeed_SET(5.6371043E37F) ;
        p115.alt_SET(-1754874224) ;
        p115.vx_SET((short) -23421) ;
        p115.attitude_quaternion_SET(new float[] {-3.161862E38F, 8.87611E37F, 2.8778715E38F, -1.3333876E38F}, 0) ;
        p115.ind_airspeed_SET((char)55245) ;
        p115.zacc_SET((short)12841) ;
        p115.vy_SET((short) -10314) ;
        p115.pitchspeed_SET(4.679657E37F) ;
        p115.xacc_SET((short)3157) ;
        p115.yacc_SET((short) -18969) ;
        p115.true_airspeed_SET((char)46702) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short)20638);
            assert(pack.ygyro_GET() == (short)5480);
            assert(pack.xgyro_GET() == (short) -9517);
            assert(pack.xacc_GET() == (short) -20888);
            assert(pack.time_boot_ms_GET() == 1640558775L);
            assert(pack.ymag_GET() == (short)21478);
            assert(pack.yacc_GET() == (short)1643);
            assert(pack.zacc_GET() == (short)19740);
            assert(pack.xmag_GET() == (short) -24614);
            assert(pack.zmag_GET() == (short) -4786);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.ygyro_SET((short)5480) ;
        p116.zgyro_SET((short)20638) ;
        p116.ymag_SET((short)21478) ;
        p116.xacc_SET((short) -20888) ;
        p116.zacc_SET((short)19740) ;
        p116.xgyro_SET((short) -9517) ;
        p116.zmag_SET((short) -4786) ;
        p116.xmag_SET((short) -24614) ;
        p116.time_boot_ms_SET(1640558775L) ;
        p116.yacc_SET((short)1643) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_GET() == (char)6438);
            assert(pack.start_GET() == (char)38424);
            assert(pack.target_system_GET() == (char)44);
            assert(pack.target_component_GET() == (char)117);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)44) ;
        p117.start_SET((char)38424) ;
        p117.target_component_SET((char)117) ;
        p117.end_SET((char)6438) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.size_GET() == 3119814526L);
            assert(pack.last_log_num_GET() == (char)3385);
            assert(pack.id_GET() == (char)7777);
            assert(pack.num_logs_GET() == (char)38550);
            assert(pack.time_utc_GET() == 1699431914L);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.last_log_num_SET((char)3385) ;
        p118.size_SET(3119814526L) ;
        p118.time_utc_SET(1699431914L) ;
        p118.num_logs_SET((char)38550) ;
        p118.id_SET((char)7777) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.ofs_GET() == 2897383420L);
            assert(pack.target_system_GET() == (char)172);
            assert(pack.count_GET() == 2816443189L);
            assert(pack.id_GET() == (char)42897);
            assert(pack.target_component_GET() == (char)42);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.id_SET((char)42897) ;
        p119.ofs_SET(2897383420L) ;
        p119.count_SET(2816443189L) ;
        p119.target_component_SET((char)42) ;
        p119.target_system_SET((char)172) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)13857);
            assert(pack.ofs_GET() == 3123029074L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)203, (char)114, (char)119, (char)207, (char)99, (char)139, (char)218, (char)90, (char)172, (char)166, (char)196, (char)134, (char)50, (char)174, (char)171, (char)114, (char)154, (char)27, (char)8, (char)208, (char)219, (char)69, (char)134, (char)232, (char)69, (char)193, (char)68, (char)31, (char)167, (char)76, (char)108, (char)53, (char)48, (char)107, (char)131, (char)152, (char)156, (char)83, (char)86, (char)76, (char)133, (char)253, (char)233, (char)125, (char)68, (char)135, (char)54, (char)144, (char)61, (char)230, (char)231, (char)209, (char)107, (char)235, (char)196, (char)32, (char)89, (char)252, (char)178, (char)172, (char)92, (char)193, (char)137, (char)237, (char)224, (char)118, (char)5, (char)174, (char)35, (char)22, (char)238, (char)6, (char)26, (char)254, (char)211, (char)123, (char)131, (char)167, (char)172, (char)104, (char)159, (char)119, (char)23, (char)166, (char)184, (char)77, (char)222, (char)78, (char)167, (char)66}));
            assert(pack.count_GET() == (char)92);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.ofs_SET(3123029074L) ;
        p120.data__SET(new char[] {(char)203, (char)114, (char)119, (char)207, (char)99, (char)139, (char)218, (char)90, (char)172, (char)166, (char)196, (char)134, (char)50, (char)174, (char)171, (char)114, (char)154, (char)27, (char)8, (char)208, (char)219, (char)69, (char)134, (char)232, (char)69, (char)193, (char)68, (char)31, (char)167, (char)76, (char)108, (char)53, (char)48, (char)107, (char)131, (char)152, (char)156, (char)83, (char)86, (char)76, (char)133, (char)253, (char)233, (char)125, (char)68, (char)135, (char)54, (char)144, (char)61, (char)230, (char)231, (char)209, (char)107, (char)235, (char)196, (char)32, (char)89, (char)252, (char)178, (char)172, (char)92, (char)193, (char)137, (char)237, (char)224, (char)118, (char)5, (char)174, (char)35, (char)22, (char)238, (char)6, (char)26, (char)254, (char)211, (char)123, (char)131, (char)167, (char)172, (char)104, (char)159, (char)119, (char)23, (char)166, (char)184, (char)77, (char)222, (char)78, (char)167, (char)66}, 0) ;
        p120.count_SET((char)92) ;
        p120.id_SET((char)13857) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)194);
            assert(pack.target_system_GET() == (char)52);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)52) ;
        p121.target_component_SET((char)194) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)12);
            assert(pack.target_component_GET() == (char)120);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)12) ;
        p122.target_component_SET((char)120) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)98);
            assert(pack.target_system_GET() == (char)20);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)240, (char)255, (char)251, (char)23, (char)3, (char)72, (char)131, (char)63, (char)86, (char)30, (char)45, (char)205, (char)64, (char)244, (char)9, (char)231, (char)50, (char)152, (char)106, (char)20, (char)29, (char)214, (char)101, (char)81, (char)73, (char)169, (char)180, (char)238, (char)20, (char)63, (char)158, (char)170, (char)209, (char)220, (char)76, (char)242, (char)211, (char)192, (char)32, (char)249, (char)65, (char)78, (char)100, (char)139, (char)125, (char)39, (char)227, (char)42, (char)14, (char)172, (char)2, (char)207, (char)229, (char)231, (char)181, (char)48, (char)233, (char)176, (char)112, (char)193, (char)208, (char)246, (char)51, (char)183, (char)189, (char)67, (char)104, (char)91, (char)56, (char)28, (char)137, (char)66, (char)189, (char)64, (char)244, (char)152, (char)190, (char)167, (char)136, (char)21, (char)143, (char)171, (char)92, (char)30, (char)252, (char)104, (char)149, (char)43, (char)254, (char)146, (char)232, (char)161, (char)252, (char)143, (char)21, (char)187, (char)85, (char)156, (char)10, (char)244, (char)210, (char)37, (char)201, (char)160, (char)30, (char)170, (char)112, (char)172, (char)61, (char)152}));
            assert(pack.len_GET() == (char)22);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.data__SET(new char[] {(char)240, (char)255, (char)251, (char)23, (char)3, (char)72, (char)131, (char)63, (char)86, (char)30, (char)45, (char)205, (char)64, (char)244, (char)9, (char)231, (char)50, (char)152, (char)106, (char)20, (char)29, (char)214, (char)101, (char)81, (char)73, (char)169, (char)180, (char)238, (char)20, (char)63, (char)158, (char)170, (char)209, (char)220, (char)76, (char)242, (char)211, (char)192, (char)32, (char)249, (char)65, (char)78, (char)100, (char)139, (char)125, (char)39, (char)227, (char)42, (char)14, (char)172, (char)2, (char)207, (char)229, (char)231, (char)181, (char)48, (char)233, (char)176, (char)112, (char)193, (char)208, (char)246, (char)51, (char)183, (char)189, (char)67, (char)104, (char)91, (char)56, (char)28, (char)137, (char)66, (char)189, (char)64, (char)244, (char)152, (char)190, (char)167, (char)136, (char)21, (char)143, (char)171, (char)92, (char)30, (char)252, (char)104, (char)149, (char)43, (char)254, (char)146, (char)232, (char)161, (char)252, (char)143, (char)21, (char)187, (char)85, (char)156, (char)10, (char)244, (char)210, (char)37, (char)201, (char)160, (char)30, (char)170, (char)112, (char)172, (char)61, (char)152}, 0) ;
        p123.target_system_SET((char)20) ;
        p123.len_SET((char)22) ;
        p123.target_component_SET((char)98) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
            assert(pack.satellites_visible_GET() == (char)228);
            assert(pack.lon_GET() == -65497678);
            assert(pack.vel_GET() == (char)5532);
            assert(pack.dgps_age_GET() == 3994319498L);
            assert(pack.cog_GET() == (char)48936);
            assert(pack.time_usec_GET() == 2464019623762205164L);
            assert(pack.alt_GET() == 1984905650);
            assert(pack.dgps_numch_GET() == (char)127);
            assert(pack.epv_GET() == (char)13688);
            assert(pack.eph_GET() == (char)5827);
            assert(pack.lat_GET() == 854669545);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.lon_SET(-65497678) ;
        p124.eph_SET((char)5827) ;
        p124.dgps_age_SET(3994319498L) ;
        p124.alt_SET(1984905650) ;
        p124.time_usec_SET(2464019623762205164L) ;
        p124.epv_SET((char)13688) ;
        p124.cog_SET((char)48936) ;
        p124.dgps_numch_SET((char)127) ;
        p124.satellites_visible_SET((char)228) ;
        p124.lat_SET(854669545) ;
        p124.vel_SET((char)5532) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vcc_GET() == (char)10254);
            assert(pack.Vservo_GET() == (char)3237);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT));
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT)) ;
        p125.Vcc_SET((char)10254) ;
        p125.Vservo_SET((char)3237) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.baudrate_GET() == 518941100L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)33, (char)122, (char)128, (char)148, (char)147, (char)48, (char)72, (char)193, (char)1, (char)164, (char)205, (char)160, (char)92, (char)150, (char)249, (char)175, (char)94, (char)87, (char)138, (char)89, (char)36, (char)145, (char)158, (char)212, (char)7, (char)27, (char)254, (char)139, (char)156, (char)225, (char)27, (char)67, (char)176, (char)9, (char)29, (char)230, (char)47, (char)196, (char)173, (char)110, (char)157, (char)16, (char)113, (char)67, (char)126, (char)13, (char)72, (char)16, (char)2, (char)80, (char)209, (char)53, (char)224, (char)173, (char)255, (char)47, (char)144, (char)165, (char)49, (char)116, (char)221, (char)44, (char)32, (char)7, (char)234, (char)147, (char)138, (char)253, (char)161, (char)228}));
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY));
            assert(pack.timeout_GET() == (char)50402);
            assert(pack.count_GET() == (char)207);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY)) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2) ;
        p126.baudrate_SET(518941100L) ;
        p126.count_SET((char)207) ;
        p126.data__SET(new char[] {(char)33, (char)122, (char)128, (char)148, (char)147, (char)48, (char)72, (char)193, (char)1, (char)164, (char)205, (char)160, (char)92, (char)150, (char)249, (char)175, (char)94, (char)87, (char)138, (char)89, (char)36, (char)145, (char)158, (char)212, (char)7, (char)27, (char)254, (char)139, (char)156, (char)225, (char)27, (char)67, (char)176, (char)9, (char)29, (char)230, (char)47, (char)196, (char)173, (char)110, (char)157, (char)16, (char)113, (char)67, (char)126, (char)13, (char)72, (char)16, (char)2, (char)80, (char)209, (char)53, (char)224, (char)173, (char)255, (char)47, (char)144, (char)165, (char)49, (char)116, (char)221, (char)44, (char)32, (char)7, (char)234, (char)147, (char)138, (char)253, (char)161, (char)228}, 0) ;
        p126.timeout_SET((char)50402) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_coords_type_GET() == (char)139);
            assert(pack.tow_GET() == 2906113526L);
            assert(pack.baseline_b_mm_GET() == 36513383);
            assert(pack.rtk_health_GET() == (char)242);
            assert(pack.rtk_rate_GET() == (char)168);
            assert(pack.accuracy_GET() == 2054633856L);
            assert(pack.time_last_baseline_ms_GET() == 389261687L);
            assert(pack.rtk_receiver_id_GET() == (char)149);
            assert(pack.nsats_GET() == (char)123);
            assert(pack.iar_num_hypotheses_GET() == -659269127);
            assert(pack.baseline_c_mm_GET() == 244392173);
            assert(pack.baseline_a_mm_GET() == 1086714478);
            assert(pack.wn_GET() == (char)11207);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.nsats_SET((char)123) ;
        p127.iar_num_hypotheses_SET(-659269127) ;
        p127.baseline_coords_type_SET((char)139) ;
        p127.tow_SET(2906113526L) ;
        p127.rtk_rate_SET((char)168) ;
        p127.rtk_health_SET((char)242) ;
        p127.wn_SET((char)11207) ;
        p127.baseline_a_mm_SET(1086714478) ;
        p127.time_last_baseline_ms_SET(389261687L) ;
        p127.accuracy_SET(2054633856L) ;
        p127.rtk_receiver_id_SET((char)149) ;
        p127.baseline_b_mm_SET(36513383) ;
        p127.baseline_c_mm_SET(244392173) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_c_mm_GET() == 839631816);
            assert(pack.rtk_rate_GET() == (char)93);
            assert(pack.rtk_health_GET() == (char)183);
            assert(pack.iar_num_hypotheses_GET() == -336587750);
            assert(pack.wn_GET() == (char)3282);
            assert(pack.rtk_receiver_id_GET() == (char)212);
            assert(pack.baseline_a_mm_GET() == 85549897);
            assert(pack.nsats_GET() == (char)216);
            assert(pack.time_last_baseline_ms_GET() == 370908321L);
            assert(pack.baseline_coords_type_GET() == (char)239);
            assert(pack.accuracy_GET() == 2419007159L);
            assert(pack.tow_GET() == 3229914345L);
            assert(pack.baseline_b_mm_GET() == -821763848);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.rtk_rate_SET((char)93) ;
        p128.iar_num_hypotheses_SET(-336587750) ;
        p128.time_last_baseline_ms_SET(370908321L) ;
        p128.baseline_coords_type_SET((char)239) ;
        p128.rtk_receiver_id_SET((char)212) ;
        p128.nsats_SET((char)216) ;
        p128.wn_SET((char)3282) ;
        p128.rtk_health_SET((char)183) ;
        p128.baseline_a_mm_SET(85549897) ;
        p128.baseline_c_mm_SET(839631816) ;
        p128.baseline_b_mm_SET(-821763848) ;
        p128.tow_SET(3229914345L) ;
        p128.accuracy_SET(2419007159L) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == (short)9639);
            assert(pack.xgyro_GET() == (short)7069);
            assert(pack.zacc_GET() == (short) -925);
            assert(pack.yacc_GET() == (short)8549);
            assert(pack.ygyro_GET() == (short) -2202);
            assert(pack.xmag_GET() == (short)30713);
            assert(pack.time_boot_ms_GET() == 3150765017L);
            assert(pack.xacc_GET() == (short)32131);
            assert(pack.zgyro_GET() == (short)1538);
            assert(pack.ymag_GET() == (short) -13398);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.ymag_SET((short) -13398) ;
        p129.zgyro_SET((short)1538) ;
        p129.ygyro_SET((short) -2202) ;
        p129.yacc_SET((short)8549) ;
        p129.time_boot_ms_SET(3150765017L) ;
        p129.zacc_SET((short) -925) ;
        p129.zmag_SET((short)9639) ;
        p129.xgyro_SET((short)7069) ;
        p129.xmag_SET((short)30713) ;
        p129.xacc_SET((short)32131) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)36);
            assert(pack.packets_GET() == (char)2950);
            assert(pack.payload_GET() == (char)251);
            assert(pack.size_GET() == 3910476153L);
            assert(pack.height_GET() == (char)58807);
            assert(pack.jpg_quality_GET() == (char)112);
            assert(pack.width_GET() == (char)57774);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.width_SET((char)57774) ;
        p130.height_SET((char)58807) ;
        p130.jpg_quality_SET((char)112) ;
        p130.payload_SET((char)251) ;
        p130.packets_SET((char)2950) ;
        p130.size_SET(3910476153L) ;
        p130.type_SET((char)36) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)49595);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)108, (char)115, (char)186, (char)107, (char)8, (char)254, (char)146, (char)210, (char)23, (char)62, (char)17, (char)14, (char)210, (char)168, (char)204, (char)92, (char)114, (char)223, (char)209, (char)5, (char)244, (char)151, (char)149, (char)194, (char)237, (char)23, (char)8, (char)46, (char)35, (char)8, (char)99, (char)64, (char)58, (char)214, (char)102, (char)166, (char)35, (char)175, (char)169, (char)58, (char)50, (char)211, (char)163, (char)80, (char)199, (char)196, (char)91, (char)25, (char)148, (char)246, (char)210, (char)53, (char)243, (char)46, (char)95, (char)115, (char)172, (char)3, (char)78, (char)211, (char)248, (char)162, (char)95, (char)194, (char)114, (char)181, (char)191, (char)194, (char)166, (char)162, (char)130, (char)109, (char)199, (char)42, (char)97, (char)65, (char)248, (char)71, (char)162, (char)173, (char)73, (char)134, (char)120, (char)57, (char)97, (char)28, (char)53, (char)28, (char)141, (char)145, (char)158, (char)16, (char)28, (char)171, (char)97, (char)30, (char)167, (char)26, (char)198, (char)41, (char)11, (char)74, (char)108, (char)189, (char)116, (char)51, (char)106, (char)162, (char)35, (char)2, (char)255, (char)104, (char)98, (char)199, (char)154, (char)67, (char)152, (char)89, (char)50, (char)132, (char)12, (char)87, (char)77, (char)119, (char)72, (char)162, (char)160, (char)56, (char)65, (char)48, (char)20, (char)255, (char)206, (char)35, (char)86, (char)89, (char)226, (char)8, (char)41, (char)52, (char)252, (char)51, (char)70, (char)23, (char)255, (char)96, (char)134, (char)188, (char)141, (char)173, (char)169, (char)222, (char)93, (char)131, (char)0, (char)82, (char)220, (char)249, (char)118, (char)153, (char)252, (char)71, (char)200, (char)99, (char)51, (char)117, (char)44, (char)226, (char)151, (char)76, (char)153, (char)77, (char)105, (char)117, (char)184, (char)190, (char)176, (char)77, (char)63, (char)138, (char)235, (char)67, (char)141, (char)81, (char)203, (char)58, (char)142, (char)138, (char)232, (char)244, (char)198, (char)104, (char)126, (char)242, (char)154, (char)53, (char)176, (char)202, (char)100, (char)211, (char)14, (char)155, (char)142, (char)34, (char)223, (char)135, (char)236, (char)53, (char)197, (char)174, (char)13, (char)232, (char)84, (char)102, (char)127, (char)120, (char)24, (char)80, (char)129, (char)224, (char)255, (char)95, (char)214, (char)56, (char)24, (char)29, (char)176, (char)82, (char)199, (char)80, (char)2, (char)238, (char)150, (char)45, (char)174, (char)56, (char)41, (char)214, (char)85, (char)75, (char)146, (char)90, (char)192, (char)168, (char)75, (char)176, (char)15, (char)250, (char)108, (char)133, (char)159, (char)96, (char)6}));
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.data__SET(new char[] {(char)108, (char)115, (char)186, (char)107, (char)8, (char)254, (char)146, (char)210, (char)23, (char)62, (char)17, (char)14, (char)210, (char)168, (char)204, (char)92, (char)114, (char)223, (char)209, (char)5, (char)244, (char)151, (char)149, (char)194, (char)237, (char)23, (char)8, (char)46, (char)35, (char)8, (char)99, (char)64, (char)58, (char)214, (char)102, (char)166, (char)35, (char)175, (char)169, (char)58, (char)50, (char)211, (char)163, (char)80, (char)199, (char)196, (char)91, (char)25, (char)148, (char)246, (char)210, (char)53, (char)243, (char)46, (char)95, (char)115, (char)172, (char)3, (char)78, (char)211, (char)248, (char)162, (char)95, (char)194, (char)114, (char)181, (char)191, (char)194, (char)166, (char)162, (char)130, (char)109, (char)199, (char)42, (char)97, (char)65, (char)248, (char)71, (char)162, (char)173, (char)73, (char)134, (char)120, (char)57, (char)97, (char)28, (char)53, (char)28, (char)141, (char)145, (char)158, (char)16, (char)28, (char)171, (char)97, (char)30, (char)167, (char)26, (char)198, (char)41, (char)11, (char)74, (char)108, (char)189, (char)116, (char)51, (char)106, (char)162, (char)35, (char)2, (char)255, (char)104, (char)98, (char)199, (char)154, (char)67, (char)152, (char)89, (char)50, (char)132, (char)12, (char)87, (char)77, (char)119, (char)72, (char)162, (char)160, (char)56, (char)65, (char)48, (char)20, (char)255, (char)206, (char)35, (char)86, (char)89, (char)226, (char)8, (char)41, (char)52, (char)252, (char)51, (char)70, (char)23, (char)255, (char)96, (char)134, (char)188, (char)141, (char)173, (char)169, (char)222, (char)93, (char)131, (char)0, (char)82, (char)220, (char)249, (char)118, (char)153, (char)252, (char)71, (char)200, (char)99, (char)51, (char)117, (char)44, (char)226, (char)151, (char)76, (char)153, (char)77, (char)105, (char)117, (char)184, (char)190, (char)176, (char)77, (char)63, (char)138, (char)235, (char)67, (char)141, (char)81, (char)203, (char)58, (char)142, (char)138, (char)232, (char)244, (char)198, (char)104, (char)126, (char)242, (char)154, (char)53, (char)176, (char)202, (char)100, (char)211, (char)14, (char)155, (char)142, (char)34, (char)223, (char)135, (char)236, (char)53, (char)197, (char)174, (char)13, (char)232, (char)84, (char)102, (char)127, (char)120, (char)24, (char)80, (char)129, (char)224, (char)255, (char)95, (char)214, (char)56, (char)24, (char)29, (char)176, (char)82, (char)199, (char)80, (char)2, (char)238, (char)150, (char)45, (char)174, (char)56, (char)41, (char)214, (char)85, (char)75, (char)146, (char)90, (char)192, (char)168, (char)75, (char)176, (char)15, (char)250, (char)108, (char)133, (char)159, (char)96, (char)6}, 0) ;
        p131.seqnr_SET((char)49595) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.max_distance_GET() == (char)51102);
            assert(pack.current_distance_GET() == (char)26916);
            assert(pack.id_GET() == (char)210);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_270);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.covariance_GET() == (char)194);
            assert(pack.min_distance_GET() == (char)20875);
            assert(pack.time_boot_ms_GET() == 838722873L);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(838722873L) ;
        p132.max_distance_SET((char)51102) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_270) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p132.current_distance_SET((char)26916) ;
        p132.covariance_SET((char)194) ;
        p132.min_distance_SET((char)20875) ;
        p132.id_SET((char)210) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mask_GET() == 7152584307236886661L);
            assert(pack.lon_GET() == 1837986199);
            assert(pack.lat_GET() == -736795307);
            assert(pack.grid_spacing_GET() == (char)52555);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.grid_spacing_SET((char)52555) ;
        p133.mask_SET(7152584307236886661L) ;
        p133.lon_SET(1837986199) ;
        p133.lat_SET(-736795307) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1190500756);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -19848, (short)10716, (short)7889, (short)32398, (short)23298, (short)21947, (short)19518, (short) -25120, (short)25013, (short)24525, (short)31652, (short) -28217, (short)19287, (short)29833, (short)5732, (short)16727}));
            assert(pack.gridbit_GET() == (char)68);
            assert(pack.grid_spacing_GET() == (char)28761);
            assert(pack.lon_GET() == -753793141);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.grid_spacing_SET((char)28761) ;
        p134.gridbit_SET((char)68) ;
        p134.lon_SET(-753793141) ;
        p134.data__SET(new short[] {(short) -19848, (short)10716, (short)7889, (short)32398, (short)23298, (short)21947, (short)19518, (short) -25120, (short)25013, (short)24525, (short)31652, (short) -28217, (short)19287, (short)29833, (short)5732, (short)16727}, 0) ;
        p134.lat_SET(-1190500756) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1316978873);
            assert(pack.lon_GET() == 1973852113);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-1316978873) ;
        p135.lon_SET(1973852113) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 248793146);
            assert(pack.current_height_GET() == 5.34867E37F);
            assert(pack.loaded_GET() == (char)63084);
            assert(pack.lon_GET() == 343945506);
            assert(pack.spacing_GET() == (char)214);
            assert(pack.pending_GET() == (char)39902);
            assert(pack.terrain_height_GET() == 5.463944E37F);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.current_height_SET(5.34867E37F) ;
        p136.pending_SET((char)39902) ;
        p136.lon_SET(343945506) ;
        p136.loaded_SET((char)63084) ;
        p136.lat_SET(248793146) ;
        p136.spacing_SET((char)214) ;
        p136.terrain_height_SET(5.463944E37F) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -1.8674872E38F);
            assert(pack.press_diff_GET() == -1.0611177E38F);
            assert(pack.time_boot_ms_GET() == 1893962225L);
            assert(pack.temperature_GET() == (short) -20251);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(1893962225L) ;
        p137.press_diff_SET(-1.0611177E38F) ;
        p137.press_abs_SET(-1.8674872E38F) ;
        p137.temperature_SET((short) -20251) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -1.0009347E38F);
            assert(pack.z_GET() == -1.2472793E38F);
            assert(pack.x_GET() == -2.0493193E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.2984986E37F, -2.2797106E38F, -8.610804E37F, 4.2903827E37F}));
            assert(pack.time_usec_GET() == 2988151587911621053L);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(2988151587911621053L) ;
        p138.z_SET(-1.2472793E38F) ;
        p138.q_SET(new float[] {1.2984986E37F, -2.2797106E38F, -8.610804E37F, 4.2903827E37F}, 0) ;
        p138.y_SET(-1.0009347E38F) ;
        p138.x_SET(-2.0493193E38F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 225514506009409363L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.0832509E38F, 1.7121915E38F, -2.4687267E37F, -1.5062688E38F, 3.1979078E38F, -1.0199112E38F, -2.2678462E38F, -8.4784716E37F}));
            assert(pack.target_system_GET() == (char)247);
            assert(pack.group_mlx_GET() == (char)32);
            assert(pack.target_component_GET() == (char)207);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.group_mlx_SET((char)32) ;
        p139.controls_SET(new float[] {-1.0832509E38F, 1.7121915E38F, -2.4687267E37F, -1.5062688E38F, 3.1979078E38F, -1.0199112E38F, -2.2678462E38F, -8.4784716E37F}, 0) ;
        p139.target_component_SET((char)207) ;
        p139.time_usec_SET(225514506009409363L) ;
        p139.target_system_SET((char)247) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)166);
            assert(pack.time_usec_GET() == 5694159360729416742L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {3.5635176E36F, -2.4440249E38F, -1.1953209E37F, -2.0580894E38F, 6.8649055E37F, 2.984925E38F, -3.6839085E37F, -2.3267429E38F}));
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(5694159360729416742L) ;
        p140.group_mlx_SET((char)166) ;
        p140.controls_SET(new float[] {3.5635176E36F, -2.4440249E38F, -1.1953209E37F, -2.0580894E38F, 6.8649055E37F, 2.984925E38F, -3.6839085E37F, -2.3267429E38F}, 0) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_monotonic_GET() == -2.2891788E38F);
            assert(pack.bottom_clearance_GET() == -3.0040119E38F);
            assert(pack.altitude_local_GET() == 2.8680362E37F);
            assert(pack.altitude_terrain_GET() == -2.0556788E38F);
            assert(pack.altitude_amsl_GET() == 2.103304E38F);
            assert(pack.altitude_relative_GET() == 5.5836044E37F);
            assert(pack.time_usec_GET() == 2720971493182082580L);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_monotonic_SET(-2.2891788E38F) ;
        p141.bottom_clearance_SET(-3.0040119E38F) ;
        p141.altitude_relative_SET(5.5836044E37F) ;
        p141.altitude_local_SET(2.8680362E37F) ;
        p141.altitude_terrain_SET(-2.0556788E38F) ;
        p141.altitude_amsl_SET(2.103304E38F) ;
        p141.time_usec_SET(2720971493182082580L) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)172, (char)195, (char)19, (char)184, (char)194, (char)73, (char)212, (char)2, (char)164, (char)164, (char)206, (char)183, (char)240, (char)39, (char)98, (char)209, (char)175, (char)81, (char)166, (char)203, (char)180, (char)175, (char)93, (char)111, (char)246, (char)14, (char)130, (char)238, (char)205, (char)208, (char)39, (char)206, (char)31, (char)253, (char)14, (char)66, (char)129, (char)64, (char)243, (char)98, (char)99, (char)55, (char)241, (char)3, (char)59, (char)166, (char)19, (char)102, (char)206, (char)129, (char)49, (char)69, (char)132, (char)245, (char)122, (char)238, (char)173, (char)154, (char)152, (char)97, (char)143, (char)15, (char)224, (char)31, (char)70, (char)85, (char)15, (char)252, (char)195, (char)17, (char)108, (char)122, (char)34, (char)58, (char)169, (char)50, (char)137, (char)91, (char)31, (char)58, (char)10, (char)94, (char)205, (char)100, (char)174, (char)26, (char)118, (char)106, (char)219, (char)163, (char)88, (char)209, (char)95, (char)233, (char)184, (char)176, (char)221, (char)143, (char)208, (char)146, (char)36, (char)164, (char)147, (char)33, (char)126, (char)236, (char)85, (char)132, (char)101, (char)134, (char)195, (char)136, (char)172, (char)198, (char)177, (char)120, (char)205, (char)70, (char)96, (char)121}));
            assert(pack.transfer_type_GET() == (char)56);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)1, (char)143, (char)34, (char)197, (char)80, (char)127, (char)191, (char)25, (char)133, (char)43, (char)14, (char)133, (char)29, (char)82, (char)225, (char)234, (char)7, (char)155, (char)217, (char)158, (char)154, (char)179, (char)169, (char)156, (char)32, (char)143, (char)226, (char)201, (char)243, (char)253, (char)253, (char)53, (char)22, (char)47, (char)28, (char)232, (char)223, (char)194, (char)242, (char)188, (char)174, (char)150, (char)95, (char)41, (char)67, (char)105, (char)134, (char)51, (char)115, (char)197, (char)51, (char)125, (char)75, (char)227, (char)61, (char)96, (char)72, (char)82, (char)180, (char)101, (char)199, (char)2, (char)1, (char)76, (char)241, (char)99, (char)211, (char)147, (char)228, (char)251, (char)51, (char)33, (char)135, (char)149, (char)59, (char)86, (char)207, (char)103, (char)231, (char)143, (char)156, (char)206, (char)122, (char)142, (char)77, (char)179, (char)219, (char)5, (char)239, (char)222, (char)132, (char)28, (char)132, (char)135, (char)208, (char)125, (char)176, (char)198, (char)170, (char)86, (char)109, (char)215, (char)223, (char)74, (char)17, (char)216, (char)55, (char)92, (char)150, (char)210, (char)98, (char)133, (char)151, (char)26, (char)21, (char)121, (char)108, (char)72, (char)143, (char)118}));
            assert(pack.request_id_GET() == (char)109);
            assert(pack.uri_type_GET() == (char)214);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.transfer_type_SET((char)56) ;
        p142.request_id_SET((char)109) ;
        p142.uri_type_SET((char)214) ;
        p142.storage_SET(new char[] {(char)1, (char)143, (char)34, (char)197, (char)80, (char)127, (char)191, (char)25, (char)133, (char)43, (char)14, (char)133, (char)29, (char)82, (char)225, (char)234, (char)7, (char)155, (char)217, (char)158, (char)154, (char)179, (char)169, (char)156, (char)32, (char)143, (char)226, (char)201, (char)243, (char)253, (char)253, (char)53, (char)22, (char)47, (char)28, (char)232, (char)223, (char)194, (char)242, (char)188, (char)174, (char)150, (char)95, (char)41, (char)67, (char)105, (char)134, (char)51, (char)115, (char)197, (char)51, (char)125, (char)75, (char)227, (char)61, (char)96, (char)72, (char)82, (char)180, (char)101, (char)199, (char)2, (char)1, (char)76, (char)241, (char)99, (char)211, (char)147, (char)228, (char)251, (char)51, (char)33, (char)135, (char)149, (char)59, (char)86, (char)207, (char)103, (char)231, (char)143, (char)156, (char)206, (char)122, (char)142, (char)77, (char)179, (char)219, (char)5, (char)239, (char)222, (char)132, (char)28, (char)132, (char)135, (char)208, (char)125, (char)176, (char)198, (char)170, (char)86, (char)109, (char)215, (char)223, (char)74, (char)17, (char)216, (char)55, (char)92, (char)150, (char)210, (char)98, (char)133, (char)151, (char)26, (char)21, (char)121, (char)108, (char)72, (char)143, (char)118}, 0) ;
        p142.uri_SET(new char[] {(char)172, (char)195, (char)19, (char)184, (char)194, (char)73, (char)212, (char)2, (char)164, (char)164, (char)206, (char)183, (char)240, (char)39, (char)98, (char)209, (char)175, (char)81, (char)166, (char)203, (char)180, (char)175, (char)93, (char)111, (char)246, (char)14, (char)130, (char)238, (char)205, (char)208, (char)39, (char)206, (char)31, (char)253, (char)14, (char)66, (char)129, (char)64, (char)243, (char)98, (char)99, (char)55, (char)241, (char)3, (char)59, (char)166, (char)19, (char)102, (char)206, (char)129, (char)49, (char)69, (char)132, (char)245, (char)122, (char)238, (char)173, (char)154, (char)152, (char)97, (char)143, (char)15, (char)224, (char)31, (char)70, (char)85, (char)15, (char)252, (char)195, (char)17, (char)108, (char)122, (char)34, (char)58, (char)169, (char)50, (char)137, (char)91, (char)31, (char)58, (char)10, (char)94, (char)205, (char)100, (char)174, (char)26, (char)118, (char)106, (char)219, (char)163, (char)88, (char)209, (char)95, (char)233, (char)184, (char)176, (char)221, (char)143, (char)208, (char)146, (char)36, (char)164, (char)147, (char)33, (char)126, (char)236, (char)85, (char)132, (char)101, (char)134, (char)195, (char)136, (char)172, (char)198, (char)177, (char)120, (char)205, (char)70, (char)96, (char)121}, 0) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)9748);
            assert(pack.time_boot_ms_GET() == 811366883L);
            assert(pack.press_abs_GET() == -1.2461801E38F);
            assert(pack.press_diff_GET() == 2.5581932E37F);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.press_diff_SET(2.5581932E37F) ;
        p143.time_boot_ms_SET(811366883L) ;
        p143.temperature_SET((short)9748) ;
        p143.press_abs_SET(-1.2461801E38F) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {5.5152197E37F, -2.2654001E38F, -2.0958974E38F, 1.8661287E37F}));
            assert(pack.lat_GET() == 1864732373);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-2.5770231E38F, -2.2090479E38F, -5.836206E37F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-1.1171722E37F, 1.8668512E38F, 2.9383956E38F}));
            assert(pack.est_capabilities_GET() == (char)98);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-1.1732519E38F, -4.441951E37F, -1.740594E38F}));
            assert(pack.timestamp_GET() == 9120994598643305711L);
            assert(pack.alt_GET() == 7.432544E37F);
            assert(pack.lon_GET() == -813677727);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {2.9882664E38F, 2.3136706E38F, 4.2574204E36F}));
            assert(pack.custom_state_GET() == 5367477399763397283L);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.alt_SET(7.432544E37F) ;
        p144.lat_SET(1864732373) ;
        p144.position_cov_SET(new float[] {-1.1732519E38F, -4.441951E37F, -1.740594E38F}, 0) ;
        p144.rates_SET(new float[] {-1.1171722E37F, 1.8668512E38F, 2.9383956E38F}, 0) ;
        p144.vel_SET(new float[] {2.9882664E38F, 2.3136706E38F, 4.2574204E36F}, 0) ;
        p144.acc_SET(new float[] {-2.5770231E38F, -2.2090479E38F, -5.836206E37F}, 0) ;
        p144.custom_state_SET(5367477399763397283L) ;
        p144.timestamp_SET(9120994598643305711L) ;
        p144.lon_SET(-813677727) ;
        p144.est_capabilities_SET((char)98) ;
        p144.attitude_q_SET(new float[] {5.5152197E37F, -2.2654001E38F, -2.0958974E38F, 1.8661287E37F}, 0) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.z_acc_GET() == 2.3554082E38F);
            assert(pack.time_usec_GET() == 4585415838165494426L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.459532E35F, 1.8744514E38F, -2.0033615E38F, 1.8701123E37F}));
            assert(pack.airspeed_GET() == -1.2667473E38F);
            assert(pack.z_vel_GET() == 2.5545788E38F);
            assert(pack.yaw_rate_GET() == -1.9717987E38F);
            assert(pack.pitch_rate_GET() == 6.967939E37F);
            assert(pack.roll_rate_GET() == 5.0431126E37F);
            assert(pack.z_pos_GET() == -5.615545E37F);
            assert(pack.y_acc_GET() == -3.0270498E38F);
            assert(pack.y_vel_GET() == -3.294239E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {-5.7575864E37F, 1.1953021E38F, -2.983641E38F}));
            assert(pack.y_pos_GET() == -1.966684E38F);
            assert(pack.x_pos_GET() == -2.8221517E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {3.0678017E38F, 1.1516353E38F, 2.2537098E38F}));
            assert(pack.x_vel_GET() == 8.814522E37F);
            assert(pack.x_acc_GET() == 2.2418095E38F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.vel_variance_SET(new float[] {3.0678017E38F, 1.1516353E38F, 2.2537098E38F}, 0) ;
        p146.q_SET(new float[] {-3.459532E35F, 1.8744514E38F, -2.0033615E38F, 1.8701123E37F}, 0) ;
        p146.time_usec_SET(4585415838165494426L) ;
        p146.x_vel_SET(8.814522E37F) ;
        p146.yaw_rate_SET(-1.9717987E38F) ;
        p146.y_pos_SET(-1.966684E38F) ;
        p146.x_acc_SET(2.2418095E38F) ;
        p146.z_vel_SET(2.5545788E38F) ;
        p146.pos_variance_SET(new float[] {-5.7575864E37F, 1.1953021E38F, -2.983641E38F}, 0) ;
        p146.y_acc_SET(-3.0270498E38F) ;
        p146.z_acc_SET(2.3554082E38F) ;
        p146.y_vel_SET(-3.294239E38F) ;
        p146.airspeed_SET(-1.2667473E38F) ;
        p146.pitch_rate_SET(6.967939E37F) ;
        p146.z_pos_SET(-5.615545E37F) ;
        p146.roll_rate_SET(5.0431126E37F) ;
        p146.x_pos_SET(-2.8221517E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -18390);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
            assert(pack.battery_remaining_GET() == (byte)42);
            assert(pack.current_battery_GET() == (short)26121);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)63978, (char)10482, (char)65305, (char)33713, (char)26713, (char)27504, (char)21253, (char)35793, (char)64228, (char)4717}));
            assert(pack.id_GET() == (char)242);
            assert(pack.energy_consumed_GET() == -1685602952);
            assert(pack.current_consumed_GET() == 934591410);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL) ;
        p147.current_battery_SET((short)26121) ;
        p147.current_consumed_SET(934591410) ;
        p147.voltages_SET(new char[] {(char)63978, (char)10482, (char)65305, (char)33713, (char)26713, (char)27504, (char)21253, (char)35793, (char)64228, (char)4717}, 0) ;
        p147.id_SET((char)242) ;
        p147.battery_remaining_SET((byte)42) ;
        p147.temperature_SET((short) -18390) ;
        p147.energy_consumed_SET(-1685602952) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.uid_GET() == 7048102782121323357L);
            assert(pack.middleware_sw_version_GET() == 3361266936L);
            assert(pack.flight_sw_version_GET() == 1039194553L);
            assert(pack.vendor_id_GET() == (char)48038);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)189, (char)187, (char)167, (char)62, (char)33, (char)162, (char)196, (char)212}));
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)168, (char)141, (char)215, (char)229, (char)12, (char)107, (char)16, (char)202}));
            assert(pack.product_id_GET() == (char)18009);
            assert(pack.board_version_GET() == 1218709757L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)242, (char)175, (char)51, (char)56, (char)207, (char)245, (char)157, (char)122, (char)128, (char)172, (char)22, (char)220, (char)77, (char)180, (char)207, (char)136, (char)145, (char)235}));
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION));
            assert(pack.os_sw_version_GET() == 596610539L);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)66, (char)30, (char)87, (char)56, (char)36, (char)208, (char)152, (char)47}));
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.flight_custom_version_SET(new char[] {(char)189, (char)187, (char)167, (char)62, (char)33, (char)162, (char)196, (char)212}, 0) ;
        p148.flight_sw_version_SET(1039194553L) ;
        p148.vendor_id_SET((char)48038) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION)) ;
        p148.uid_SET(7048102782121323357L) ;
        p148.middleware_sw_version_SET(3361266936L) ;
        p148.product_id_SET((char)18009) ;
        p148.os_custom_version_SET(new char[] {(char)168, (char)141, (char)215, (char)229, (char)12, (char)107, (char)16, (char)202}, 0) ;
        p148.board_version_SET(1218709757L) ;
        p148.middleware_custom_version_SET(new char[] {(char)66, (char)30, (char)87, (char)56, (char)36, (char)208, (char)152, (char)47}, 0) ;
        p148.os_sw_version_SET(596610539L) ;
        p148.uid2_SET(new char[] {(char)242, (char)175, (char)51, (char)56, (char)207, (char)245, (char)157, (char)122, (char)128, (char)172, (char)22, (char)220, (char)77, (char)180, (char)207, (char)136, (char)145, (char)235}, 0, PH) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.angle_y_GET() == -2.9331165E38F);
            assert(pack.angle_x_GET() == -1.937792E38F);
            assert(pack.size_y_GET() == 4.550028E37F);
            assert(pack.distance_GET() == 1.2928309E38F);
            assert(pack.size_x_GET() == -3.3985811E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.target_num_GET() == (char)24);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {2.1020279E37F, 2.5236508E38F, 2.6100849E38F, -1.4838538E38F}));
            assert(pack.time_usec_GET() == 7421318615830937382L);
            assert(pack.z_TRY(ph) == 9.1588935E36F);
            assert(pack.y_TRY(ph) == 1.9935941E38F);
            assert(pack.position_valid_TRY(ph) == (char)114);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
            assert(pack.x_TRY(ph) == 8.128408E37F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.z_SET(9.1588935E36F, PH) ;
        p149.y_SET(1.9935941E38F, PH) ;
        p149.angle_y_SET(-2.9331165E38F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p149.size_x_SET(-3.3985811E37F) ;
        p149.time_usec_SET(7421318615830937382L) ;
        p149.x_SET(8.128408E37F, PH) ;
        p149.position_valid_SET((char)114, PH) ;
        p149.size_y_SET(4.550028E37F) ;
        p149.target_num_SET((char)24) ;
        p149.q_SET(new float[] {2.1020279E37F, 2.5236508E38F, 2.6100849E38F, -1.4838538E38F}, 0, PH) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL) ;
        p149.angle_x_SET(-1.937792E38F) ;
        p149.distance_SET(1.2928309E38F) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENS_POWER.add((src, ph, pack) ->
        {
            assert(pack.adc121_cspb_amp_GET() == -2.6793175E37F);
            assert(pack.adc121_cs2_amp_GET() == -2.3492379E38F);
            assert(pack.adc121_vspb_volt_GET() == 3.1453054E38F);
            assert(pack.adc121_cs1_amp_GET() == 2.7374548E38F);
        });
        GroundControl.SENS_POWER p201 = CommunicationChannel.new_SENS_POWER();
        PH.setPack(p201);
        p201.adc121_cs2_amp_SET(-2.3492379E38F) ;
        p201.adc121_cs1_amp_SET(2.7374548E38F) ;
        p201.adc121_cspb_amp_SET(-2.6793175E37F) ;
        p201.adc121_vspb_volt_SET(3.1453054E38F) ;
        CommunicationChannel.instance.send(p201);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENS_MPPT.add((src, ph, pack) ->
        {
            assert(pack.mppt1_pwm_GET() == (char)50458);
            assert(pack.mppt2_status_GET() == (char)72);
            assert(pack.mppt1_amp_GET() == 2.1734874E38F);
            assert(pack.mppt1_volt_GET() == -2.3383444E38F);
            assert(pack.mppt_timestamp_GET() == 4147670304787249367L);
            assert(pack.mppt1_status_GET() == (char)112);
            assert(pack.mppt2_amp_GET() == -6.1280045E37F);
            assert(pack.mppt3_status_GET() == (char)51);
            assert(pack.mppt3_amp_GET() == -1.1946191E38F);
            assert(pack.mppt2_pwm_GET() == (char)39914);
            assert(pack.mppt3_volt_GET() == 2.1989424E38F);
            assert(pack.mppt3_pwm_GET() == (char)55148);
            assert(pack.mppt2_volt_GET() == 3.2254183E38F);
        });
        GroundControl.SENS_MPPT p202 = CommunicationChannel.new_SENS_MPPT();
        PH.setPack(p202);
        p202.mppt3_amp_SET(-1.1946191E38F) ;
        p202.mppt1_status_SET((char)112) ;
        p202.mppt1_volt_SET(-2.3383444E38F) ;
        p202.mppt3_status_SET((char)51) ;
        p202.mppt1_amp_SET(2.1734874E38F) ;
        p202.mppt3_pwm_SET((char)55148) ;
        p202.mppt_timestamp_SET(4147670304787249367L) ;
        p202.mppt3_volt_SET(2.1989424E38F) ;
        p202.mppt2_status_SET((char)72) ;
        p202.mppt2_amp_SET(-6.1280045E37F) ;
        p202.mppt2_pwm_SET((char)39914) ;
        p202.mppt2_volt_SET(3.2254183E38F) ;
        p202.mppt1_pwm_SET((char)50458) ;
        CommunicationChannel.instance.send(p202);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ASLCTRL_DATA.add((src, ph, pack) ->
        {
            assert(pack.qRef_GET() == 8.0283887E37F);
            assert(pack.uThrot2_GET() == -9.853886E37F);
            assert(pack.uAil_GET() == 1.15841754E36F);
            assert(pack.aslctrl_mode_GET() == (char)10);
            assert(pack.RollAngleRef_GET() == 2.7524364E38F);
            assert(pack.PitchAngle_GET() == 1.0930142E38F);
            assert(pack.YawAngleRef_GET() == -9.944373E37F);
            assert(pack.PitchAngleRef_GET() == 1.2182969E38F);
            assert(pack.uRud_GET() == 2.4845672E38F);
            assert(pack.r_GET() == -2.7735787E38F);
            assert(pack.uThrot_GET() == 1.1161433E38F);
            assert(pack.YawAngle_GET() == -1.8629253E38F);
            assert(pack.pRef_GET() == -1.7735381E38F);
            assert(pack.q_GET() == -4.5717946E37F);
            assert(pack.h_GET() == 1.6775976E38F);
            assert(pack.hRef_t_GET() == 2.6587556E38F);
            assert(pack.RollAngle_GET() == -2.876613E38F);
            assert(pack.nZ_GET() == 1.6031862E38F);
            assert(pack.hRef_GET() == 1.1526174E38F);
            assert(pack.AirspeedRef_GET() == 1.5589866E38F);
            assert(pack.timestamp_GET() == 7544299846963523059L);
            assert(pack.p_GET() == -2.3772146E38F);
            assert(pack.rRef_GET() == 3.0049037E38F);
            assert(pack.uElev_GET() == 1.2143712E38F);
            assert(pack.SpoilersEngaged_GET() == (char)30);
        });
        GroundControl.ASLCTRL_DATA p203 = CommunicationChannel.new_ASLCTRL_DATA();
        PH.setPack(p203);
        p203.uRud_SET(2.4845672E38F) ;
        p203.pRef_SET(-1.7735381E38F) ;
        p203.RollAngleRef_SET(2.7524364E38F) ;
        p203.PitchAngle_SET(1.0930142E38F) ;
        p203.timestamp_SET(7544299846963523059L) ;
        p203.aslctrl_mode_SET((char)10) ;
        p203.YawAngle_SET(-1.8629253E38F) ;
        p203.uAil_SET(1.15841754E36F) ;
        p203.uThrot2_SET(-9.853886E37F) ;
        p203.rRef_SET(3.0049037E38F) ;
        p203.hRef_SET(1.1526174E38F) ;
        p203.h_SET(1.6775976E38F) ;
        p203.nZ_SET(1.6031862E38F) ;
        p203.YawAngleRef_SET(-9.944373E37F) ;
        p203.uThrot_SET(1.1161433E38F) ;
        p203.PitchAngleRef_SET(1.2182969E38F) ;
        p203.q_SET(-4.5717946E37F) ;
        p203.qRef_SET(8.0283887E37F) ;
        p203.r_SET(-2.7735787E38F) ;
        p203.RollAngle_SET(-2.876613E38F) ;
        p203.AirspeedRef_SET(1.5589866E38F) ;
        p203.SpoilersEngaged_SET((char)30) ;
        p203.hRef_t_SET(2.6587556E38F) ;
        p203.p_SET(-2.3772146E38F) ;
        p203.uElev_SET(1.2143712E38F) ;
        CommunicationChannel.instance.send(p203);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ASLCTRL_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.i32_1_GET() == 3283554897L);
            assert(pack.f_8_GET() == -3.3448358E37F);
            assert(pack.f_2_GET() == -3.210802E38F);
            assert(pack.i8_2_GET() == (char)37);
            assert(pack.f_1_GET() == 6.131907E37F);
            assert(pack.f_3_GET() == -2.8910259E38F);
            assert(pack.f_7_GET() == -2.5827902E38F);
            assert(pack.i8_1_GET() == (char)195);
            assert(pack.f_5_GET() == -1.8016272E38F);
            assert(pack.f_4_GET() == 1.0499898E38F);
            assert(pack.f_6_GET() == -1.256435E38F);
        });
        GroundControl.ASLCTRL_DEBUG p204 = CommunicationChannel.new_ASLCTRL_DEBUG();
        PH.setPack(p204);
        p204.f_6_SET(-1.256435E38F) ;
        p204.f_4_SET(1.0499898E38F) ;
        p204.f_3_SET(-2.8910259E38F) ;
        p204.f_5_SET(-1.8016272E38F) ;
        p204.i8_1_SET((char)195) ;
        p204.f_8_SET(-3.3448358E37F) ;
        p204.f_1_SET(6.131907E37F) ;
        p204.f_7_SET(-2.5827902E38F) ;
        p204.i32_1_SET(3283554897L) ;
        p204.i8_2_SET((char)37) ;
        p204.f_2_SET(-3.210802E38F) ;
        CommunicationChannel.instance.send(p204);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ASLUAV_STATUS.add((src, ph, pack) ->
        {
            assert(pack.SATCOM_status_GET() == (char)232);
            assert(pack.LED_status_GET() == (char)215);
            assert(pack.Motor_rpm_GET() == 3.3470566E38F);
            assert(Arrays.equals(pack.Servo_status_GET(),  new char[] {(char)70, (char)191, (char)62, (char)55, (char)71, (char)102, (char)92, (char)241}));
        });
        GroundControl.ASLUAV_STATUS p205 = CommunicationChannel.new_ASLUAV_STATUS();
        PH.setPack(p205);
        p205.Servo_status_SET(new char[] {(char)70, (char)191, (char)62, (char)55, (char)71, (char)102, (char)92, (char)241}, 0) ;
        p205.SATCOM_status_SET((char)232) ;
        p205.Motor_rpm_SET(3.3470566E38F) ;
        p205.LED_status_SET((char)215) ;
        CommunicationChannel.instance.send(p205);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_EKF_EXT.add((src, ph, pack) ->
        {
            assert(pack.WindDir_GET() == 2.9770233E38F);
            assert(pack.alpha_GET() == -2.9286242E38F);
            assert(pack.timestamp_GET() == 1468011756477393155L);
            assert(pack.WindZ_GET() == 2.2105364E38F);
            assert(pack.Windspeed_GET() == -2.9022869E38F);
            assert(pack.Airspeed_GET() == 2.54985E38F);
            assert(pack.beta_GET() == -1.2023302E37F);
        });
        GroundControl.EKF_EXT p206 = CommunicationChannel.new_EKF_EXT();
        PH.setPack(p206);
        p206.timestamp_SET(1468011756477393155L) ;
        p206.beta_SET(-1.2023302E37F) ;
        p206.alpha_SET(-2.9286242E38F) ;
        p206.WindDir_SET(2.9770233E38F) ;
        p206.Airspeed_SET(2.54985E38F) ;
        p206.Windspeed_SET(-2.9022869E38F) ;
        p206.WindZ_SET(2.2105364E38F) ;
        CommunicationChannel.instance.send(p206);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ASL_OBCTRL.add((src, ph, pack) ->
        {
            assert(pack.uAilL_GET() == -1.6550846E38F);
            assert(pack.uThrot_GET() == 8.385985E37F);
            assert(pack.timestamp_GET() == 3374880152201372084L);
            assert(pack.uElev_GET() == -1.0188358E37F);
            assert(pack.uRud_GET() == 1.711403E38F);
            assert(pack.uAilR_GET() == -2.4625744E38F);
            assert(pack.uThrot2_GET() == 1.7320636E38F);
            assert(pack.obctrl_status_GET() == (char)73);
        });
        GroundControl.ASL_OBCTRL p207 = CommunicationChannel.new_ASL_OBCTRL();
        PH.setPack(p207);
        p207.uAilR_SET(-2.4625744E38F) ;
        p207.uAilL_SET(-1.6550846E38F) ;
        p207.uThrot2_SET(1.7320636E38F) ;
        p207.uThrot_SET(8.385985E37F) ;
        p207.uRud_SET(1.711403E38F) ;
        p207.timestamp_SET(3374880152201372084L) ;
        p207.obctrl_status_SET((char)73) ;
        p207.uElev_SET(-1.0188358E37F) ;
        CommunicationChannel.instance.send(p207);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENS_ATMOS.add((src, ph, pack) ->
        {
            assert(pack.Humidity_GET() == 9.513547E36F);
            assert(pack.TempAmbient_GET() == -4.7905094E37F);
        });
        GroundControl.SENS_ATMOS p208 = CommunicationChannel.new_SENS_ATMOS();
        PH.setPack(p208);
        p208.Humidity_SET(9.513547E36F) ;
        p208.TempAmbient_SET(-4.7905094E37F) ;
        CommunicationChannel.instance.send(p208);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENS_BATMON.add((src, ph, pack) ->
        {
            assert(pack.cellvoltage1_GET() == (char)50418);
            assert(pack.cellvoltage5_GET() == (char)35185);
            assert(pack.current_GET() == (short) -32236);
            assert(pack.voltage_GET() == (char)10509);
            assert(pack.cellvoltage2_GET() == (char)30537);
            assert(pack.hostfetcontrol_GET() == (char)59685);
            assert(pack.batterystatus_GET() == (char)34013);
            assert(pack.SoC_GET() == (char)190);
            assert(pack.temperature_GET() == -5.7195366E37F);
            assert(pack.cellvoltage6_GET() == (char)1302);
            assert(pack.cellvoltage4_GET() == (char)41900);
            assert(pack.serialnumber_GET() == (char)44345);
            assert(pack.cellvoltage3_GET() == (char)24501);
        });
        GroundControl.SENS_BATMON p209 = CommunicationChannel.new_SENS_BATMON();
        PH.setPack(p209);
        p209.cellvoltage3_SET((char)24501) ;
        p209.current_SET((short) -32236) ;
        p209.cellvoltage6_SET((char)1302) ;
        p209.hostfetcontrol_SET((char)59685) ;
        p209.voltage_SET((char)10509) ;
        p209.SoC_SET((char)190) ;
        p209.serialnumber_SET((char)44345) ;
        p209.batterystatus_SET((char)34013) ;
        p209.cellvoltage5_SET((char)35185) ;
        p209.temperature_SET(-5.7195366E37F) ;
        p209.cellvoltage4_SET((char)41900) ;
        p209.cellvoltage1_SET((char)50418) ;
        p209.cellvoltage2_SET((char)30537) ;
        CommunicationChannel.instance.send(p209);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FW_SOARING_DATA.add((src, ph, pack) ->
        {
            assert(pack.vSinkExp_GET() == 7.7554423E37F);
            assert(pack.z2_exp_GET() == -2.8032414E38F);
            assert(pack.z1_exp_GET() == -2.4679745E36F);
            assert(pack.ThermalGSNorth_GET() == 3.7472528E37F);
            assert(pack.ControlMode_GET() == (char)44);
            assert(pack.xR_GET() == 1.8488236E38F);
            assert(pack.VarLat_GET() == 2.8125932E38F);
            assert(pack.VarW_GET() == -2.0244914E38F);
            assert(pack.VarR_GET() == 2.0744043E38F);
            assert(pack.xLat_GET() == -2.6918088E38F);
            assert(pack.z2_DeltaRoll_GET() == 2.3185524E38F);
            assert(pack.DebugVar2_GET() == 2.5418644E38F);
            assert(pack.z1_LocalUpdraftSpeed_GET() == 2.1325341E38F);
            assert(pack.xW_GET() == -2.3068385E38F);
            assert(pack.LoiterDirection_GET() == 1.6881369E38F);
            assert(pack.VarLon_GET() == -1.3999638E38F);
            assert(pack.DebugVar1_GET() == 3.27091E38F);
            assert(pack.xLon_GET() == -1.3367994E38F);
            assert(pack.ThermalGSEast_GET() == -3.9913617E37F);
            assert(pack.TSE_dot_GET() == -1.7360146E38F);
            assert(pack.DistToSoarPoint_GET() == -2.068616E38F);
            assert(pack.valid_GET() == (char)237);
            assert(pack.timestamp_GET() == 3911860844324464673L);
            assert(pack.timestampModeChanged_GET() == 7100399346659361612L);
            assert(pack.LoiterRadius_GET() == -2.1207532E37F);
        });
        GroundControl.FW_SOARING_DATA p210 = CommunicationChannel.new_FW_SOARING_DATA();
        PH.setPack(p210);
        p210.valid_SET((char)237) ;
        p210.DistToSoarPoint_SET(-2.068616E38F) ;
        p210.ThermalGSNorth_SET(3.7472528E37F) ;
        p210.xR_SET(1.8488236E38F) ;
        p210.xLat_SET(-2.6918088E38F) ;
        p210.VarLat_SET(2.8125932E38F) ;
        p210.VarR_SET(2.0744043E38F) ;
        p210.timestampModeChanged_SET(7100399346659361612L) ;
        p210.DebugVar2_SET(2.5418644E38F) ;
        p210.z1_exp_SET(-2.4679745E36F) ;
        p210.VarLon_SET(-1.3999638E38F) ;
        p210.LoiterRadius_SET(-2.1207532E37F) ;
        p210.LoiterDirection_SET(1.6881369E38F) ;
        p210.vSinkExp_SET(7.7554423E37F) ;
        p210.ControlMode_SET((char)44) ;
        p210.DebugVar1_SET(3.27091E38F) ;
        p210.xW_SET(-2.3068385E38F) ;
        p210.TSE_dot_SET(-1.7360146E38F) ;
        p210.z2_exp_SET(-2.8032414E38F) ;
        p210.z1_LocalUpdraftSpeed_SET(2.1325341E38F) ;
        p210.xLon_SET(-1.3367994E38F) ;
        p210.VarW_SET(-2.0244914E38F) ;
        p210.z2_DeltaRoll_SET(2.3185524E38F) ;
        p210.ThermalGSEast_SET(-3.9913617E37F) ;
        p210.timestamp_SET(3911860844324464673L) ;
        CommunicationChannel.instance.send(p210);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENSORPOD_STATUS.add((src, ph, pack) ->
        {
            assert(pack.visensor_rate_1_GET() == (char)63);
            assert(pack.recording_nodes_count_GET() == (char)138);
            assert(pack.visensor_rate_4_GET() == (char)248);
            assert(pack.visensor_rate_3_GET() == (char)181);
            assert(pack.timestamp_GET() == 8833065796640162190L);
            assert(pack.cpu_temp_GET() == (char)86);
            assert(pack.visensor_rate_2_GET() == (char)200);
            assert(pack.free_space_GET() == (char)33360);
        });
        GroundControl.SENSORPOD_STATUS p211 = CommunicationChannel.new_SENSORPOD_STATUS();
        PH.setPack(p211);
        p211.timestamp_SET(8833065796640162190L) ;
        p211.visensor_rate_2_SET((char)200) ;
        p211.free_space_SET((char)33360) ;
        p211.visensor_rate_3_SET((char)181) ;
        p211.visensor_rate_1_SET((char)63) ;
        p211.cpu_temp_SET((char)86) ;
        p211.visensor_rate_4_SET((char)248) ;
        p211.recording_nodes_count_SET((char)138) ;
        CommunicationChannel.instance.send(p211);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENS_POWER_BOARD.add((src, ph, pack) ->
        {
            assert(pack.timestamp_GET() == 782625782661960960L);
            assert(pack.pwr_brd_servo_3_amp_GET() == 2.5837583E38F);
            assert(pack.pwr_brd_servo_4_amp_GET() == -1.253417E38F);
            assert(pack.pwr_brd_servo_1_amp_GET() == -1.5168036E38F);
            assert(pack.pwr_brd_mot_r_amp_GET() == -2.5305474E36F);
            assert(pack.pwr_brd_mot_l_amp_GET() == 3.0876687E38F);
            assert(pack.pwr_brd_servo_volt_GET() == -7.2671934E37F);
            assert(pack.pwr_brd_system_volt_GET() == 1.8108072E38F);
            assert(pack.pwr_brd_aux_amp_GET() == -1.755512E37F);
            assert(pack.pwr_brd_servo_2_amp_GET() == 2.77648E37F);
            assert(pack.pwr_brd_status_GET() == (char)148);
            assert(pack.pwr_brd_led_status_GET() == (char)140);
        });
        GroundControl.SENS_POWER_BOARD p212 = CommunicationChannel.new_SENS_POWER_BOARD();
        PH.setPack(p212);
        p212.pwr_brd_system_volt_SET(1.8108072E38F) ;
        p212.pwr_brd_servo_3_amp_SET(2.5837583E38F) ;
        p212.pwr_brd_servo_1_amp_SET(-1.5168036E38F) ;
        p212.pwr_brd_mot_r_amp_SET(-2.5305474E36F) ;
        p212.pwr_brd_servo_4_amp_SET(-1.253417E38F) ;
        p212.pwr_brd_servo_volt_SET(-7.2671934E37F) ;
        p212.pwr_brd_led_status_SET((char)140) ;
        p212.pwr_brd_aux_amp_SET(-1.755512E37F) ;
        p212.pwr_brd_mot_l_amp_SET(3.0876687E38F) ;
        p212.pwr_brd_servo_2_amp_SET(2.77648E37F) ;
        p212.timestamp_SET(782625782661960960L) ;
        p212.pwr_brd_status_SET((char)148) ;
        CommunicationChannel.instance.send(p212);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pos_horiz_accuracy_GET() == 2.8457492E38F);
            assert(pack.pos_horiz_ratio_GET() == 1.1291786E38F);
            assert(pack.mag_ratio_GET() == -2.806065E38F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE));
            assert(pack.pos_vert_ratio_GET() == 2.7653914E37F);
            assert(pack.time_usec_GET() == 3612683872273272187L);
            assert(pack.hagl_ratio_GET() == -2.4720906E37F);
            assert(pack.tas_ratio_GET() == 1.3983754E38F);
            assert(pack.vel_ratio_GET() == -3.752625E37F);
            assert(pack.pos_vert_accuracy_GET() == -1.5124735E38F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.tas_ratio_SET(1.3983754E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE)) ;
        p230.hagl_ratio_SET(-2.4720906E37F) ;
        p230.pos_horiz_ratio_SET(1.1291786E38F) ;
        p230.pos_vert_ratio_SET(2.7653914E37F) ;
        p230.mag_ratio_SET(-2.806065E38F) ;
        p230.vel_ratio_SET(-3.752625E37F) ;
        p230.pos_horiz_accuracy_SET(2.8457492E38F) ;
        p230.pos_vert_accuracy_SET(-1.5124735E38F) ;
        p230.time_usec_SET(3612683872273272187L) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_x_GET() == 3.3971912E38F);
            assert(pack.var_horiz_GET() == -2.681713E38F);
            assert(pack.horiz_accuracy_GET() == -5.115364E37F);
            assert(pack.wind_z_GET() == -1.4877333E38F);
            assert(pack.wind_alt_GET() == -2.7600404E38F);
            assert(pack.var_vert_GET() == -2.0157985E38F);
            assert(pack.wind_y_GET() == -5.8661444E37F);
            assert(pack.vert_accuracy_GET() == 2.0820092E38F);
            assert(pack.time_usec_GET() == 5814095644237490199L);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.horiz_accuracy_SET(-5.115364E37F) ;
        p231.wind_y_SET(-5.8661444E37F) ;
        p231.vert_accuracy_SET(2.0820092E38F) ;
        p231.wind_x_SET(3.3971912E38F) ;
        p231.time_usec_SET(5814095644237490199L) ;
        p231.var_vert_SET(-2.0157985E38F) ;
        p231.wind_z_SET(-1.4877333E38F) ;
        p231.var_horiz_SET(-2.681713E38F) ;
        p231.wind_alt_SET(-2.7600404E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2955352394338329660L);
            assert(pack.horiz_accuracy_GET() == 2.1295295E38F);
            assert(pack.ve_GET() == 3.1349314E38F);
            assert(pack.lon_GET() == -798994702);
            assert(pack.alt_GET() == 3.1487761E38F);
            assert(pack.speed_accuracy_GET() == 1.83497E38F);
            assert(pack.fix_type_GET() == (char)71);
            assert(pack.vd_GET() == -2.5322308E38F);
            assert(pack.vert_accuracy_GET() == -1.2636917E38F);
            assert(pack.time_week_ms_GET() == 1305515592L);
            assert(pack.time_week_GET() == (char)63255);
            assert(pack.vdop_GET() == 2.3272173E38F);
            assert(pack.hdop_GET() == 2.1288896E37F);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY));
            assert(pack.lat_GET() == -1230666206);
            assert(pack.vn_GET() == -2.469154E38F);
            assert(pack.satellites_visible_GET() == (char)37);
            assert(pack.gps_id_GET() == (char)127);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.lon_SET(-798994702) ;
        p232.ve_SET(3.1349314E38F) ;
        p232.horiz_accuracy_SET(2.1295295E38F) ;
        p232.time_week_ms_SET(1305515592L) ;
        p232.fix_type_SET((char)71) ;
        p232.vdop_SET(2.3272173E38F) ;
        p232.alt_SET(3.1487761E38F) ;
        p232.vn_SET(-2.469154E38F) ;
        p232.satellites_visible_SET((char)37) ;
        p232.gps_id_SET((char)127) ;
        p232.time_usec_SET(2955352394338329660L) ;
        p232.hdop_SET(2.1288896E37F) ;
        p232.lat_SET(-1230666206) ;
        p232.vd_SET(-2.5322308E38F) ;
        p232.speed_accuracy_SET(1.83497E38F) ;
        p232.time_week_SET((char)63255) ;
        p232.vert_accuracy_SET(-1.2636917E38F) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY)) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)253, (char)251, (char)180, (char)176, (char)160, (char)98, (char)142, (char)234, (char)191, (char)165, (char)50, (char)137, (char)234, (char)181, (char)254, (char)198, (char)29, (char)232, (char)217, (char)14, (char)10, (char)98, (char)85, (char)42, (char)174, (char)144, (char)165, (char)163, (char)136, (char)70, (char)100, (char)126, (char)254, (char)187, (char)220, (char)25, (char)81, (char)192, (char)7, (char)177, (char)223, (char)194, (char)143, (char)183, (char)222, (char)83, (char)98, (char)184, (char)181, (char)242, (char)15, (char)181, (char)169, (char)254, (char)147, (char)69, (char)201, (char)166, (char)180, (char)196, (char)167, (char)73, (char)222, (char)249, (char)56, (char)163, (char)214, (char)193, (char)108, (char)15, (char)225, (char)70, (char)179, (char)88, (char)115, (char)194, (char)237, (char)133, (char)32, (char)119, (char)30, (char)93, (char)87, (char)210, (char)245, (char)38, (char)214, (char)76, (char)252, (char)178, (char)3, (char)122, (char)18, (char)145, (char)178, (char)122, (char)255, (char)104, (char)207, (char)81, (char)177, (char)39, (char)214, (char)67, (char)192, (char)111, (char)52, (char)240, (char)205, (char)218, (char)202, (char)199, (char)17, (char)241, (char)81, (char)250, (char)181, (char)36, (char)44, (char)168, (char)43, (char)75, (char)118, (char)176, (char)1, (char)195, (char)81, (char)5, (char)92, (char)16, (char)172, (char)106, (char)77, (char)202, (char)3, (char)23, (char)115, (char)59, (char)130, (char)63, (char)221, (char)190, (char)42, (char)180, (char)44, (char)139, (char)190, (char)42, (char)212, (char)162, (char)241, (char)29, (char)177, (char)36, (char)170, (char)113, (char)38, (char)119, (char)144, (char)6, (char)140, (char)48, (char)217, (char)52, (char)5, (char)79, (char)87, (char)241, (char)49, (char)68, (char)194, (char)99, (char)215, (char)167, (char)253, (char)146, (char)167, (char)30, (char)198, (char)175}));
            assert(pack.flags_GET() == (char)65);
            assert(pack.len_GET() == (char)193);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.len_SET((char)193) ;
        p233.flags_SET((char)65) ;
        p233.data__SET(new char[] {(char)253, (char)251, (char)180, (char)176, (char)160, (char)98, (char)142, (char)234, (char)191, (char)165, (char)50, (char)137, (char)234, (char)181, (char)254, (char)198, (char)29, (char)232, (char)217, (char)14, (char)10, (char)98, (char)85, (char)42, (char)174, (char)144, (char)165, (char)163, (char)136, (char)70, (char)100, (char)126, (char)254, (char)187, (char)220, (char)25, (char)81, (char)192, (char)7, (char)177, (char)223, (char)194, (char)143, (char)183, (char)222, (char)83, (char)98, (char)184, (char)181, (char)242, (char)15, (char)181, (char)169, (char)254, (char)147, (char)69, (char)201, (char)166, (char)180, (char)196, (char)167, (char)73, (char)222, (char)249, (char)56, (char)163, (char)214, (char)193, (char)108, (char)15, (char)225, (char)70, (char)179, (char)88, (char)115, (char)194, (char)237, (char)133, (char)32, (char)119, (char)30, (char)93, (char)87, (char)210, (char)245, (char)38, (char)214, (char)76, (char)252, (char)178, (char)3, (char)122, (char)18, (char)145, (char)178, (char)122, (char)255, (char)104, (char)207, (char)81, (char)177, (char)39, (char)214, (char)67, (char)192, (char)111, (char)52, (char)240, (char)205, (char)218, (char)202, (char)199, (char)17, (char)241, (char)81, (char)250, (char)181, (char)36, (char)44, (char)168, (char)43, (char)75, (char)118, (char)176, (char)1, (char)195, (char)81, (char)5, (char)92, (char)16, (char)172, (char)106, (char)77, (char)202, (char)3, (char)23, (char)115, (char)59, (char)130, (char)63, (char)221, (char)190, (char)42, (char)180, (char)44, (char)139, (char)190, (char)42, (char)212, (char)162, (char)241, (char)29, (char)177, (char)36, (char)170, (char)113, (char)38, (char)119, (char)144, (char)6, (char)140, (char)48, (char)217, (char)52, (char)5, (char)79, (char)87, (char)241, (char)49, (char)68, (char)194, (char)99, (char)215, (char)167, (char)253, (char)146, (char)167, (char)30, (char)198, (char)175}, 0) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.wp_num_GET() == (char)248);
            assert(pack.temperature_GET() == (byte) - 31);
            assert(pack.groundspeed_GET() == (char)28);
            assert(pack.heading_GET() == (char)19659);
            assert(pack.altitude_sp_GET() == (short)29048);
            assert(pack.battery_remaining_GET() == (char)241);
            assert(pack.longitude_GET() == 1652590568);
            assert(pack.pitch_GET() == (short) -12649);
            assert(pack.throttle_GET() == (byte)74);
            assert(pack.airspeed_GET() == (char)216);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED));
            assert(pack.wp_distance_GET() == (char)26606);
            assert(pack.latitude_GET() == -412812085);
            assert(pack.custom_mode_GET() == 2264389364L);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.airspeed_sp_GET() == (char)244);
            assert(pack.heading_sp_GET() == (short) -13276);
            assert(pack.altitude_amsl_GET() == (short) -9413);
            assert(pack.gps_nsat_GET() == (char)219);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            assert(pack.failsafe_GET() == (char)61);
            assert(pack.climb_rate_GET() == (byte)66);
            assert(pack.temperature_air_GET() == (byte)32);
            assert(pack.roll_GET() == (short) -27166);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.airspeed_sp_SET((char)244) ;
        p234.temperature_SET((byte) - 31) ;
        p234.heading_sp_SET((short) -13276) ;
        p234.battery_remaining_SET((char)241) ;
        p234.failsafe_SET((char)61) ;
        p234.longitude_SET(1652590568) ;
        p234.gps_nsat_SET((char)219) ;
        p234.heading_SET((char)19659) ;
        p234.throttle_SET((byte)74) ;
        p234.latitude_SET(-412812085) ;
        p234.temperature_air_SET((byte)32) ;
        p234.wp_num_SET((char)248) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED)) ;
        p234.custom_mode_SET(2264389364L) ;
        p234.roll_SET((short) -27166) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND) ;
        p234.climb_rate_SET((byte)66) ;
        p234.pitch_SET((short) -12649) ;
        p234.altitude_sp_SET((short)29048) ;
        p234.airspeed_SET((char)216) ;
        p234.altitude_amsl_SET((short) -9413) ;
        p234.groundspeed_SET((char)28) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p234.wp_distance_SET((char)26606) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 1358895990709505405L);
            assert(pack.vibration_y_GET() == 1.984925E38F);
            assert(pack.clipping_0_GET() == 3485782502L);
            assert(pack.vibration_z_GET() == 1.0184796E38F);
            assert(pack.vibration_x_GET() == 9.806263E37F);
            assert(pack.clipping_2_GET() == 2784899235L);
            assert(pack.clipping_1_GET() == 1374206059L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_1_SET(1374206059L) ;
        p241.vibration_z_SET(1.0184796E38F) ;
        p241.vibration_x_SET(9.806263E37F) ;
        p241.clipping_0_SET(3485782502L) ;
        p241.vibration_y_SET(1.984925E38F) ;
        p241.clipping_2_SET(2784899235L) ;
        p241.time_usec_SET(1358895990709505405L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == -1602599636);
            assert(pack.approach_x_GET() == -4.323202E37F);
            assert(pack.approach_z_GET() == 1.7288925E38F);
            assert(pack.time_usec_TRY(ph) == 5638260895917466781L);
            assert(pack.latitude_GET() == -454697411);
            assert(pack.approach_y_GET() == 1.9561147E38F);
            assert(pack.altitude_GET() == -114680421);
            assert(pack.z_GET() == -2.949742E38F);
            assert(pack.x_GET() == 2.4608278E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-7.8232124E37F, 1.8036372E38F, 2.9177055E38F, -3.0909506E38F}));
            assert(pack.y_GET() == 1.1908892E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.x_SET(2.4608278E38F) ;
        p242.latitude_SET(-454697411) ;
        p242.q_SET(new float[] {-7.8232124E37F, 1.8036372E38F, 2.9177055E38F, -3.0909506E38F}, 0) ;
        p242.approach_x_SET(-4.323202E37F) ;
        p242.y_SET(1.1908892E38F) ;
        p242.altitude_SET(-114680421) ;
        p242.time_usec_SET(5638260895917466781L, PH) ;
        p242.approach_z_SET(1.7288925E38F) ;
        p242.z_SET(-2.949742E38F) ;
        p242.longitude_SET(-1602599636) ;
        p242.approach_y_SET(1.9561147E38F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -3.1579316E38F);
            assert(pack.x_GET() == 8.224589E36F);
            assert(pack.time_usec_TRY(ph) == 9055640153673849713L);
            assert(pack.latitude_GET() == -600816263);
            assert(pack.longitude_GET() == -1373974355);
            assert(pack.z_GET() == -2.5302874E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.9449003E38F, 6.5846625E37F, 2.689491E38F, -1.6067648E38F}));
            assert(pack.target_system_GET() == (char)90);
            assert(pack.approach_y_GET() == 1.0997232E38F);
            assert(pack.approach_x_GET() == -2.4588846E38F);
            assert(pack.approach_z_GET() == -2.883118E38F);
            assert(pack.altitude_GET() == -132868379);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.longitude_SET(-1373974355) ;
        p243.approach_y_SET(1.0997232E38F) ;
        p243.altitude_SET(-132868379) ;
        p243.approach_z_SET(-2.883118E38F) ;
        p243.x_SET(8.224589E36F) ;
        p243.latitude_SET(-600816263) ;
        p243.y_SET(-3.1579316E38F) ;
        p243.target_system_SET((char)90) ;
        p243.approach_x_SET(-2.4588846E38F) ;
        p243.z_SET(-2.5302874E38F) ;
        p243.time_usec_SET(9055640153673849713L, PH) ;
        p243.q_SET(new float[] {-1.9449003E38F, 6.5846625E37F, 2.689491E38F, -1.6067648E38F}, 0) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)28554);
            assert(pack.interval_us_GET() == -1847222572);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)28554) ;
        p244.interval_us_SET(-1847222572) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.heading_GET() == (char)40263);
            assert(pack.lon_GET() == 1704351402);
            assert(pack.squawk_GET() == (char)12433);
            assert(pack.hor_velocity_GET() == (char)37604);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHTER_AIR);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK));
            assert(pack.callsign_LEN(ph) == 8);
            assert(pack.callsign_TRY(ph).equals("sjrmtdnh"));
            assert(pack.ICAO_address_GET() == 2255300167L);
            assert(pack.ver_velocity_GET() == (short) -767);
            assert(pack.altitude_GET() == -560369998);
            assert(pack.lat_GET() == -2027060642);
            assert(pack.tslc_GET() == (char)123);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.squawk_SET((char)12433) ;
        p246.ICAO_address_SET(2255300167L) ;
        p246.ver_velocity_SET((short) -767) ;
        p246.altitude_SET(-560369998) ;
        p246.lat_SET(-2027060642) ;
        p246.lon_SET(1704351402) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.tslc_SET((char)123) ;
        p246.hor_velocity_SET((char)37604) ;
        p246.heading_SET((char)40263) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK)) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHTER_AIR) ;
        p246.callsign_SET("sjrmtdnh", PH) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == 1594467851L);
            assert(pack.time_to_minimum_delta_GET() == -2.435067E38F);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
            assert(pack.altitude_minimum_delta_GET() == 2.9907518E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            assert(pack.horizontal_minimum_delta_GET() == -2.3940431E38F);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.time_to_minimum_delta_SET(-2.435067E38F) ;
        p247.id_SET(1594467851L) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH) ;
        p247.horizontal_minimum_delta_SET(-2.3940431E38F) ;
        p247.altitude_minimum_delta_SET(2.9907518E38F) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.message_type_GET() == (char)38983);
            assert(pack.target_system_GET() == (char)137);
            assert(pack.target_network_GET() == (char)239);
            assert(pack.target_component_GET() == (char)70);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)22, (char)39, (char)107, (char)230, (char)108, (char)53, (char)21, (char)199, (char)123, (char)9, (char)178, (char)14, (char)133, (char)206, (char)53, (char)76, (char)11, (char)145, (char)117, (char)193, (char)107, (char)212, (char)124, (char)114, (char)164, (char)7, (char)214, (char)188, (char)68, (char)202, (char)90, (char)131, (char)128, (char)150, (char)123, (char)241, (char)236, (char)145, (char)83, (char)56, (char)67, (char)64, (char)230, (char)91, (char)40, (char)99, (char)89, (char)102, (char)101, (char)133, (char)127, (char)44, (char)22, (char)238, (char)186, (char)3, (char)26, (char)163, (char)227, (char)36, (char)230, (char)196, (char)93, (char)219, (char)119, (char)34, (char)200, (char)158, (char)179, (char)91, (char)196, (char)240, (char)179, (char)100, (char)74, (char)20, (char)123, (char)202, (char)61, (char)15, (char)65, (char)163, (char)131, (char)13, (char)10, (char)115, (char)135, (char)44, (char)156, (char)74, (char)82, (char)116, (char)56, (char)60, (char)179, (char)9, (char)223, (char)37, (char)186, (char)246, (char)100, (char)99, (char)80, (char)145, (char)250, (char)141, (char)219, (char)204, (char)69, (char)122, (char)196, (char)109, (char)22, (char)191, (char)54, (char)126, (char)27, (char)115, (char)211, (char)242, (char)112, (char)162, (char)82, (char)139, (char)4, (char)46, (char)123, (char)57, (char)23, (char)175, (char)124, (char)72, (char)50, (char)9, (char)251, (char)107, (char)51, (char)252, (char)2, (char)239, (char)25, (char)142, (char)120, (char)178, (char)193, (char)130, (char)30, (char)198, (char)190, (char)83, (char)21, (char)55, (char)239, (char)194, (char)155, (char)28, (char)250, (char)236, (char)140, (char)20, (char)237, (char)17, (char)91, (char)14, (char)84, (char)215, (char)18, (char)120, (char)201, (char)177, (char)219, (char)60, (char)143, (char)6, (char)181, (char)173, (char)63, (char)99, (char)168, (char)49, (char)86, (char)138, (char)91, (char)6, (char)145, (char)87, (char)68, (char)193, (char)215, (char)95, (char)253, (char)123, (char)161, (char)210, (char)187, (char)189, (char)92, (char)145, (char)153, (char)228, (char)109, (char)119, (char)47, (char)171, (char)189, (char)142, (char)206, (char)43, (char)173, (char)95, (char)239, (char)156, (char)155, (char)137, (char)3, (char)111, (char)216, (char)29, (char)114, (char)220, (char)248, (char)121, (char)8, (char)86, (char)247, (char)189, (char)221, (char)116, (char)230, (char)113, (char)182, (char)201, (char)129, (char)118, (char)173, (char)4, (char)120, (char)129, (char)136, (char)41, (char)163, (char)219, (char)20, (char)126, (char)234, (char)52, (char)206, (char)22, (char)24}));
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_system_SET((char)137) ;
        p248.message_type_SET((char)38983) ;
        p248.target_component_SET((char)70) ;
        p248.payload_SET(new char[] {(char)22, (char)39, (char)107, (char)230, (char)108, (char)53, (char)21, (char)199, (char)123, (char)9, (char)178, (char)14, (char)133, (char)206, (char)53, (char)76, (char)11, (char)145, (char)117, (char)193, (char)107, (char)212, (char)124, (char)114, (char)164, (char)7, (char)214, (char)188, (char)68, (char)202, (char)90, (char)131, (char)128, (char)150, (char)123, (char)241, (char)236, (char)145, (char)83, (char)56, (char)67, (char)64, (char)230, (char)91, (char)40, (char)99, (char)89, (char)102, (char)101, (char)133, (char)127, (char)44, (char)22, (char)238, (char)186, (char)3, (char)26, (char)163, (char)227, (char)36, (char)230, (char)196, (char)93, (char)219, (char)119, (char)34, (char)200, (char)158, (char)179, (char)91, (char)196, (char)240, (char)179, (char)100, (char)74, (char)20, (char)123, (char)202, (char)61, (char)15, (char)65, (char)163, (char)131, (char)13, (char)10, (char)115, (char)135, (char)44, (char)156, (char)74, (char)82, (char)116, (char)56, (char)60, (char)179, (char)9, (char)223, (char)37, (char)186, (char)246, (char)100, (char)99, (char)80, (char)145, (char)250, (char)141, (char)219, (char)204, (char)69, (char)122, (char)196, (char)109, (char)22, (char)191, (char)54, (char)126, (char)27, (char)115, (char)211, (char)242, (char)112, (char)162, (char)82, (char)139, (char)4, (char)46, (char)123, (char)57, (char)23, (char)175, (char)124, (char)72, (char)50, (char)9, (char)251, (char)107, (char)51, (char)252, (char)2, (char)239, (char)25, (char)142, (char)120, (char)178, (char)193, (char)130, (char)30, (char)198, (char)190, (char)83, (char)21, (char)55, (char)239, (char)194, (char)155, (char)28, (char)250, (char)236, (char)140, (char)20, (char)237, (char)17, (char)91, (char)14, (char)84, (char)215, (char)18, (char)120, (char)201, (char)177, (char)219, (char)60, (char)143, (char)6, (char)181, (char)173, (char)63, (char)99, (char)168, (char)49, (char)86, (char)138, (char)91, (char)6, (char)145, (char)87, (char)68, (char)193, (char)215, (char)95, (char)253, (char)123, (char)161, (char)210, (char)187, (char)189, (char)92, (char)145, (char)153, (char)228, (char)109, (char)119, (char)47, (char)171, (char)189, (char)142, (char)206, (char)43, (char)173, (char)95, (char)239, (char)156, (char)155, (char)137, (char)3, (char)111, (char)216, (char)29, (char)114, (char)220, (char)248, (char)121, (char)8, (char)86, (char)247, (char)189, (char)221, (char)116, (char)230, (char)113, (char)182, (char)201, (char)129, (char)118, (char)173, (char)4, (char)120, (char)129, (char)136, (char)41, (char)163, (char)219, (char)20, (char)126, (char)234, (char)52, (char)206, (char)22, (char)24}, 0) ;
        p248.target_network_SET((char)239) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.ver_GET() == (char)37);
            assert(pack.address_GET() == (char)51202);
            assert(pack.type_GET() == (char)69);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)47, (byte)47, (byte)82, (byte) - 15, (byte) - 63, (byte)69, (byte)30, (byte)109, (byte) - 66, (byte) - 92, (byte)123, (byte)20, (byte)54, (byte) - 6, (byte) - 92, (byte) - 37, (byte)41, (byte) - 10, (byte) - 14, (byte) - 97, (byte)101, (byte) - 79, (byte) - 17, (byte)36, (byte)11, (byte) - 84, (byte)78, (byte) - 99, (byte)24, (byte) - 90, (byte)62, (byte) - 55}));
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)51202) ;
        p249.type_SET((char)69) ;
        p249.ver_SET((char)37) ;
        p249.value_SET(new byte[] {(byte)47, (byte)47, (byte)82, (byte) - 15, (byte) - 63, (byte)69, (byte)30, (byte)109, (byte) - 66, (byte) - 92, (byte)123, (byte)20, (byte)54, (byte) - 6, (byte) - 92, (byte) - 37, (byte)41, (byte) - 10, (byte) - 14, (byte) - 97, (byte)101, (byte) - 79, (byte) - 17, (byte)36, (byte)11, (byte) - 84, (byte)78, (byte) - 99, (byte)24, (byte) - 90, (byte)62, (byte) - 55}, 0) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 1.3087452E38F);
            assert(pack.x_GET() == 3.2756322E37F);
            assert(pack.time_usec_GET() == 3613420763089008725L);
            assert(pack.y_GET() == 2.9096384E37F);
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("qomfewcxaf"));
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.x_SET(3.2756322E37F) ;
        p250.name_SET("qomfewcxaf", PH) ;
        p250.z_SET(1.3087452E38F) ;
        p250.y_SET(2.9096384E37F) ;
        p250.time_usec_SET(3613420763089008725L) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2766083160L);
            assert(pack.name_LEN(ph) == 5);
            assert(pack.name_TRY(ph).equals("lygcm"));
            assert(pack.value_GET() == 2.8674011E38F);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(2.8674011E38F) ;
        p251.name_SET("lygcm", PH) ;
        p251.time_boot_ms_SET(2766083160L) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2326195772L);
            assert(pack.value_GET() == 1789232796);
            assert(pack.name_LEN(ph) == 6);
            assert(pack.name_TRY(ph).equals("wjtvnr"));
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.value_SET(1789232796) ;
        p252.name_SET("wjtvnr", PH) ;
        p252.time_boot_ms_SET(2326195772L) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 10);
            assert(pack.text_TRY(ph).equals("ologptivha"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_INFO);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("ologptivha", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_INFO) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.ind_GET() == (char)231);
            assert(pack.value_GET() == 2.420494E38F);
            assert(pack.time_boot_ms_GET() == 1700740556L);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.ind_SET((char)231) ;
        p254.value_SET(2.420494E38F) ;
        p254.time_boot_ms_SET(1700740556L) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)0);
            assert(pack.target_system_GET() == (char)14);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)60, (char)151, (char)126, (char)32, (char)197, (char)234, (char)94, (char)230, (char)40, (char)80, (char)86, (char)47, (char)168, (char)84, (char)95, (char)171, (char)112, (char)220, (char)45, (char)175, (char)71, (char)126, (char)42, (char)201, (char)140, (char)248, (char)96, (char)143, (char)245, (char)133, (char)19, (char)19}));
            assert(pack.initial_timestamp_GET() == 242961320518400211L);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)14) ;
        p256.target_component_SET((char)0) ;
        p256.secret_key_SET(new char[] {(char)60, (char)151, (char)126, (char)32, (char)197, (char)234, (char)94, (char)230, (char)40, (char)80, (char)86, (char)47, (char)168, (char)84, (char)95, (char)171, (char)112, (char)220, (char)45, (char)175, (char)71, (char)126, (char)42, (char)201, (char)140, (char)248, (char)96, (char)143, (char)245, (char)133, (char)19, (char)19}, 0) ;
        p256.initial_timestamp_SET(242961320518400211L) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1643477853L);
            assert(pack.state_GET() == (char)154);
            assert(pack.last_change_ms_GET() == 1917208146L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.last_change_ms_SET(1917208146L) ;
        p257.time_boot_ms_SET(1643477853L) ;
        p257.state_SET((char)154) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.tune_LEN(ph) == 2);
            assert(pack.tune_TRY(ph).equals("ov"));
            assert(pack.target_component_GET() == (char)213);
            assert(pack.target_system_GET() == (char)214);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_component_SET((char)213) ;
        p258.target_system_SET((char)214) ;
        p258.tune_SET("ov", PH) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.sensor_size_v_GET() == -3.0363309E38F);
            assert(pack.firmware_version_GET() == 369372794L);
            assert(pack.cam_definition_uri_LEN(ph) == 5);
            assert(pack.cam_definition_uri_TRY(ph).equals("vonlm"));
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
            assert(pack.time_boot_ms_GET() == 1216791392L);
            assert(pack.resolution_h_GET() == (char)44241);
            assert(pack.lens_id_GET() == (char)174);
            assert(pack.resolution_v_GET() == (char)61719);
            assert(pack.sensor_size_h_GET() == 5.7739127E37F);
            assert(pack.cam_definition_version_GET() == (char)19482);
            assert(pack.focal_length_GET() == -2.9847598E38F);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)223, (char)28, (char)75, (char)113, (char)61, (char)89, (char)68, (char)32, (char)10, (char)24, (char)72, (char)170, (char)158, (char)159, (char)124, (char)57, (char)186, (char)25, (char)107, (char)111, (char)86, (char)138, (char)19, (char)55, (char)14, (char)235, (char)192, (char)198, (char)244, (char)10, (char)27, (char)76}));
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)125, (char)237, (char)144, (char)118, (char)200, (char)45, (char)154, (char)172, (char)192, (char)122, (char)217, (char)87, (char)1, (char)250, (char)240, (char)129, (char)230, (char)70, (char)12, (char)102, (char)90, (char)127, (char)8, (char)219, (char)220, (char)92, (char)11, (char)228, (char)26, (char)20, (char)19, (char)225}));
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.vendor_name_SET(new char[] {(char)223, (char)28, (char)75, (char)113, (char)61, (char)89, (char)68, (char)32, (char)10, (char)24, (char)72, (char)170, (char)158, (char)159, (char)124, (char)57, (char)186, (char)25, (char)107, (char)111, (char)86, (char)138, (char)19, (char)55, (char)14, (char)235, (char)192, (char)198, (char)244, (char)10, (char)27, (char)76}, 0) ;
        p259.focal_length_SET(-2.9847598E38F) ;
        p259.cam_definition_uri_SET("vonlm", PH) ;
        p259.resolution_h_SET((char)44241) ;
        p259.cam_definition_version_SET((char)19482) ;
        p259.time_boot_ms_SET(1216791392L) ;
        p259.firmware_version_SET(369372794L) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE)) ;
        p259.lens_id_SET((char)174) ;
        p259.sensor_size_h_SET(5.7739127E37F) ;
        p259.sensor_size_v_SET(-3.0363309E38F) ;
        p259.model_name_SET(new char[] {(char)125, (char)237, (char)144, (char)118, (char)200, (char)45, (char)154, (char)172, (char)192, (char)122, (char)217, (char)87, (char)1, (char)250, (char)240, (char)129, (char)230, (char)70, (char)12, (char)102, (char)90, (char)127, (char)8, (char)219, (char)220, (char)92, (char)11, (char)228, (char)26, (char)20, (char)19, (char)225}, 0) ;
        p259.resolution_v_SET((char)61719) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2192774088L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_VIDEO);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO) ;
        p260.time_boot_ms_SET(2192774088L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.status_GET() == (char)154);
            assert(pack.write_speed_GET() == 2.7743957E38F);
            assert(pack.storage_count_GET() == (char)16);
            assert(pack.storage_id_GET() == (char)68);
            assert(pack.time_boot_ms_GET() == 3534801456L);
            assert(pack.total_capacity_GET() == 1.2144493E38F);
            assert(pack.available_capacity_GET() == 3.3771265E38F);
            assert(pack.read_speed_GET() == 3.185238E38F);
            assert(pack.used_capacity_GET() == 1.6233678E38F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.available_capacity_SET(3.3771265E38F) ;
        p261.write_speed_SET(2.7743957E38F) ;
        p261.status_SET((char)154) ;
        p261.used_capacity_SET(1.6233678E38F) ;
        p261.storage_id_SET((char)68) ;
        p261.total_capacity_SET(1.2144493E38F) ;
        p261.read_speed_SET(3.185238E38F) ;
        p261.time_boot_ms_SET(3534801456L) ;
        p261.storage_count_SET((char)16) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.available_capacity_GET() == -3.0593747E38F);
            assert(pack.video_status_GET() == (char)45);
            assert(pack.time_boot_ms_GET() == 4088730276L);
            assert(pack.recording_time_ms_GET() == 2349759288L);
            assert(pack.image_interval_GET() == -1.494597E38F);
            assert(pack.image_status_GET() == (char)203);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.image_status_SET((char)203) ;
        p262.recording_time_ms_SET(2349759288L) ;
        p262.image_interval_SET(-1.494597E38F) ;
        p262.video_status_SET((char)45) ;
        p262.available_capacity_SET(-3.0593747E38F) ;
        p262.time_boot_ms_SET(4088730276L) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.image_index_GET() == 1240111356);
            assert(pack.time_utc_GET() == 1091640838501048366L);
            assert(pack.lat_GET() == -1454845050);
            assert(pack.relative_alt_GET() == -2055757516);
            assert(pack.camera_id_GET() == (char)10);
            assert(pack.time_boot_ms_GET() == 1113318781L);
            assert(pack.alt_GET() == 1602319191);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.086598E38F, -1.6424981E38F, 7.565693E37F, 1.3233117E37F}));
            assert(pack.lon_GET() == -1505063727);
            assert(pack.file_url_LEN(ph) == 72);
            assert(pack.file_url_TRY(ph).equals("qeqjlbcobxmxAmfaxezohbdzqzworstjfmbfbtaqbzexkLkdajevNnoaqqlhucvcjtgxNdlf"));
            assert(pack.capture_result_GET() == (byte)0);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.lon_SET(-1505063727) ;
        p263.lat_SET(-1454845050) ;
        p263.q_SET(new float[] {-1.086598E38F, -1.6424981E38F, 7.565693E37F, 1.3233117E37F}, 0) ;
        p263.file_url_SET("qeqjlbcobxmxAmfaxezohbdzqzworstjfmbfbtaqbzexkLkdajevNnoaqqlhucvcjtgxNdlf", PH) ;
        p263.camera_id_SET((char)10) ;
        p263.capture_result_SET((byte)0) ;
        p263.image_index_SET(1240111356) ;
        p263.time_boot_ms_SET(1113318781L) ;
        p263.time_utc_SET(1091640838501048366L) ;
        p263.relative_alt_SET(-2055757516) ;
        p263.alt_SET(1602319191) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.flight_uuid_GET() == 1194562848594468931L);
            assert(pack.time_boot_ms_GET() == 3906167398L);
            assert(pack.arming_time_utc_GET() == 8530654195267031974L);
            assert(pack.takeoff_time_utc_GET() == 4696507978873873106L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(3906167398L) ;
        p264.arming_time_utc_SET(8530654195267031974L) ;
        p264.takeoff_time_utc_SET(4696507978873873106L) ;
        p264.flight_uuid_SET(1194562848594468931L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -2.6564998E38F);
            assert(pack.roll_GET() == -4.3346785E37F);
            assert(pack.time_boot_ms_GET() == 1229102004L);
            assert(pack.pitch_GET() == -1.976831E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(-4.3346785E37F) ;
        p265.time_boot_ms_SET(1229102004L) ;
        p265.yaw_SET(-2.6564998E38F) ;
        p265.pitch_SET(-1.976831E38F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)189);
            assert(pack.sequence_GET() == (char)4522);
            assert(pack.first_message_offset_GET() == (char)225);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)46, (char)42, (char)129, (char)68, (char)3, (char)167, (char)1, (char)148, (char)176, (char)192, (char)144, (char)13, (char)111, (char)18, (char)96, (char)232, (char)3, (char)58, (char)105, (char)180, (char)189, (char)101, (char)43, (char)132, (char)96, (char)221, (char)156, (char)240, (char)101, (char)124, (char)144, (char)98, (char)77, (char)35, (char)67, (char)220, (char)221, (char)80, (char)252, (char)129, (char)215, (char)111, (char)55, (char)43, (char)255, (char)230, (char)99, (char)166, (char)116, (char)204, (char)41, (char)216, (char)61, (char)90, (char)231, (char)129, (char)171, (char)170, (char)26, (char)215, (char)108, (char)208, (char)125, (char)104, (char)191, (char)171, (char)72, (char)104, (char)37, (char)133, (char)219, (char)102, (char)138, (char)144, (char)244, (char)226, (char)41, (char)13, (char)181, (char)171, (char)2, (char)25, (char)81, (char)153, (char)109, (char)230, (char)150, (char)2, (char)81, (char)247, (char)36, (char)56, (char)107, (char)156, (char)85, (char)64, (char)50, (char)95, (char)59, (char)254, (char)231, (char)208, (char)135, (char)142, (char)128, (char)69, (char)104, (char)103, (char)82, (char)171, (char)224, (char)194, (char)242, (char)222, (char)209, (char)134, (char)237, (char)41, (char)127, (char)84, (char)105, (char)42, (char)166, (char)55, (char)248, (char)193, (char)202, (char)76, (char)30, (char)216, (char)76, (char)231, (char)123, (char)26, (char)11, (char)0, (char)237, (char)97, (char)22, (char)160, (char)4, (char)123, (char)210, (char)68, (char)84, (char)164, (char)169, (char)36, (char)75, (char)17, (char)59, (char)234, (char)92, (char)167, (char)97, (char)69, (char)171, (char)35, (char)121, (char)46, (char)214, (char)58, (char)80, (char)184, (char)60, (char)7, (char)167, (char)133, (char)57, (char)52, (char)103, (char)177, (char)9, (char)230, (char)251, (char)20, (char)194, (char)206, (char)201, (char)13, (char)65, (char)252, (char)58, (char)61, (char)10, (char)2, (char)121, (char)118, (char)183, (char)221, (char)128, (char)42, (char)222, (char)188, (char)233, (char)233, (char)120, (char)71, (char)99, (char)112, (char)150, (char)142, (char)122, (char)173, (char)215, (char)85, (char)156, (char)249, (char)191, (char)35, (char)56, (char)246, (char)35, (char)188, (char)106, (char)181, (char)38, (char)100, (char)243, (char)82, (char)207, (char)43, (char)2, (char)80, (char)142, (char)123, (char)54, (char)12, (char)76, (char)126, (char)244, (char)144, (char)63, (char)192, (char)155, (char)34, (char)252, (char)217, (char)146, (char)153, (char)87, (char)156, (char)10, (char)11, (char)60, (char)129, (char)133, (char)1, (char)138}));
            assert(pack.length_GET() == (char)106);
            assert(pack.target_system_GET() == (char)252);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.length_SET((char)106) ;
        p266.first_message_offset_SET((char)225) ;
        p266.target_component_SET((char)189) ;
        p266.data__SET(new char[] {(char)46, (char)42, (char)129, (char)68, (char)3, (char)167, (char)1, (char)148, (char)176, (char)192, (char)144, (char)13, (char)111, (char)18, (char)96, (char)232, (char)3, (char)58, (char)105, (char)180, (char)189, (char)101, (char)43, (char)132, (char)96, (char)221, (char)156, (char)240, (char)101, (char)124, (char)144, (char)98, (char)77, (char)35, (char)67, (char)220, (char)221, (char)80, (char)252, (char)129, (char)215, (char)111, (char)55, (char)43, (char)255, (char)230, (char)99, (char)166, (char)116, (char)204, (char)41, (char)216, (char)61, (char)90, (char)231, (char)129, (char)171, (char)170, (char)26, (char)215, (char)108, (char)208, (char)125, (char)104, (char)191, (char)171, (char)72, (char)104, (char)37, (char)133, (char)219, (char)102, (char)138, (char)144, (char)244, (char)226, (char)41, (char)13, (char)181, (char)171, (char)2, (char)25, (char)81, (char)153, (char)109, (char)230, (char)150, (char)2, (char)81, (char)247, (char)36, (char)56, (char)107, (char)156, (char)85, (char)64, (char)50, (char)95, (char)59, (char)254, (char)231, (char)208, (char)135, (char)142, (char)128, (char)69, (char)104, (char)103, (char)82, (char)171, (char)224, (char)194, (char)242, (char)222, (char)209, (char)134, (char)237, (char)41, (char)127, (char)84, (char)105, (char)42, (char)166, (char)55, (char)248, (char)193, (char)202, (char)76, (char)30, (char)216, (char)76, (char)231, (char)123, (char)26, (char)11, (char)0, (char)237, (char)97, (char)22, (char)160, (char)4, (char)123, (char)210, (char)68, (char)84, (char)164, (char)169, (char)36, (char)75, (char)17, (char)59, (char)234, (char)92, (char)167, (char)97, (char)69, (char)171, (char)35, (char)121, (char)46, (char)214, (char)58, (char)80, (char)184, (char)60, (char)7, (char)167, (char)133, (char)57, (char)52, (char)103, (char)177, (char)9, (char)230, (char)251, (char)20, (char)194, (char)206, (char)201, (char)13, (char)65, (char)252, (char)58, (char)61, (char)10, (char)2, (char)121, (char)118, (char)183, (char)221, (char)128, (char)42, (char)222, (char)188, (char)233, (char)233, (char)120, (char)71, (char)99, (char)112, (char)150, (char)142, (char)122, (char)173, (char)215, (char)85, (char)156, (char)249, (char)191, (char)35, (char)56, (char)246, (char)35, (char)188, (char)106, (char)181, (char)38, (char)100, (char)243, (char)82, (char)207, (char)43, (char)2, (char)80, (char)142, (char)123, (char)54, (char)12, (char)76, (char)126, (char)244, (char)144, (char)63, (char)192, (char)155, (char)34, (char)252, (char)217, (char)146, (char)153, (char)87, (char)156, (char)10, (char)11, (char)60, (char)129, (char)133, (char)1, (char)138}, 0) ;
        p266.target_system_SET((char)252) ;
        p266.sequence_SET((char)4522) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)250);
            assert(pack.first_message_offset_GET() == (char)118);
            assert(pack.length_GET() == (char)84);
            assert(pack.sequence_GET() == (char)26455);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)15, (char)6, (char)46, (char)170, (char)141, (char)1, (char)226, (char)177, (char)248, (char)104, (char)235, (char)121, (char)59, (char)158, (char)245, (char)37, (char)86, (char)139, (char)175, (char)174, (char)8, (char)119, (char)84, (char)111, (char)9, (char)219, (char)61, (char)135, (char)169, (char)80, (char)238, (char)108, (char)24, (char)179, (char)13, (char)36, (char)201, (char)136, (char)199, (char)164, (char)72, (char)243, (char)98, (char)235, (char)211, (char)122, (char)1, (char)231, (char)199, (char)128, (char)239, (char)122, (char)58, (char)130, (char)104, (char)111, (char)63, (char)115, (char)127, (char)122, (char)132, (char)78, (char)27, (char)47, (char)31, (char)157, (char)39, (char)168, (char)3, (char)34, (char)35, (char)183, (char)114, (char)136, (char)162, (char)35, (char)137, (char)120, (char)158, (char)229, (char)112, (char)26, (char)145, (char)83, (char)150, (char)136, (char)72, (char)254, (char)62, (char)229, (char)217, (char)76, (char)93, (char)88, (char)40, (char)189, (char)166, (char)1, (char)154, (char)241, (char)236, (char)67, (char)168, (char)210, (char)108, (char)10, (char)45, (char)243, (char)206, (char)119, (char)226, (char)69, (char)37, (char)109, (char)123, (char)157, (char)193, (char)212, (char)105, (char)130, (char)21, (char)13, (char)138, (char)221, (char)190, (char)40, (char)57, (char)244, (char)111, (char)88, (char)146, (char)70, (char)102, (char)139, (char)65, (char)181, (char)58, (char)100, (char)109, (char)80, (char)112, (char)127, (char)219, (char)247, (char)62, (char)91, (char)192, (char)111, (char)252, (char)126, (char)26, (char)191, (char)162, (char)212, (char)159, (char)221, (char)101, (char)31, (char)243, (char)15, (char)26, (char)173, (char)198, (char)200, (char)138, (char)44, (char)96, (char)213, (char)116, (char)98, (char)192, (char)96, (char)115, (char)81, (char)175, (char)54, (char)104, (char)136, (char)245, (char)229, (char)128, (char)46, (char)49, (char)74, (char)31, (char)115, (char)25, (char)203, (char)46, (char)198, (char)130, (char)139, (char)171, (char)169, (char)94, (char)23, (char)26, (char)65, (char)137, (char)146, (char)179, (char)159, (char)70, (char)217, (char)27, (char)77, (char)177, (char)187, (char)155, (char)113, (char)156, (char)89, (char)254, (char)151, (char)248, (char)172, (char)10, (char)50, (char)150, (char)228, (char)169, (char)250, (char)153, (char)160, (char)95, (char)68, (char)99, (char)179, (char)193, (char)144, (char)89, (char)191, (char)6, (char)6, (char)242, (char)110, (char)145, (char)48, (char)38, (char)19, (char)158, (char)184, (char)198, (char)171, (char)120, (char)251, (char)227, (char)250, (char)36}));
            assert(pack.target_component_GET() == (char)133);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)250) ;
        p267.data__SET(new char[] {(char)15, (char)6, (char)46, (char)170, (char)141, (char)1, (char)226, (char)177, (char)248, (char)104, (char)235, (char)121, (char)59, (char)158, (char)245, (char)37, (char)86, (char)139, (char)175, (char)174, (char)8, (char)119, (char)84, (char)111, (char)9, (char)219, (char)61, (char)135, (char)169, (char)80, (char)238, (char)108, (char)24, (char)179, (char)13, (char)36, (char)201, (char)136, (char)199, (char)164, (char)72, (char)243, (char)98, (char)235, (char)211, (char)122, (char)1, (char)231, (char)199, (char)128, (char)239, (char)122, (char)58, (char)130, (char)104, (char)111, (char)63, (char)115, (char)127, (char)122, (char)132, (char)78, (char)27, (char)47, (char)31, (char)157, (char)39, (char)168, (char)3, (char)34, (char)35, (char)183, (char)114, (char)136, (char)162, (char)35, (char)137, (char)120, (char)158, (char)229, (char)112, (char)26, (char)145, (char)83, (char)150, (char)136, (char)72, (char)254, (char)62, (char)229, (char)217, (char)76, (char)93, (char)88, (char)40, (char)189, (char)166, (char)1, (char)154, (char)241, (char)236, (char)67, (char)168, (char)210, (char)108, (char)10, (char)45, (char)243, (char)206, (char)119, (char)226, (char)69, (char)37, (char)109, (char)123, (char)157, (char)193, (char)212, (char)105, (char)130, (char)21, (char)13, (char)138, (char)221, (char)190, (char)40, (char)57, (char)244, (char)111, (char)88, (char)146, (char)70, (char)102, (char)139, (char)65, (char)181, (char)58, (char)100, (char)109, (char)80, (char)112, (char)127, (char)219, (char)247, (char)62, (char)91, (char)192, (char)111, (char)252, (char)126, (char)26, (char)191, (char)162, (char)212, (char)159, (char)221, (char)101, (char)31, (char)243, (char)15, (char)26, (char)173, (char)198, (char)200, (char)138, (char)44, (char)96, (char)213, (char)116, (char)98, (char)192, (char)96, (char)115, (char)81, (char)175, (char)54, (char)104, (char)136, (char)245, (char)229, (char)128, (char)46, (char)49, (char)74, (char)31, (char)115, (char)25, (char)203, (char)46, (char)198, (char)130, (char)139, (char)171, (char)169, (char)94, (char)23, (char)26, (char)65, (char)137, (char)146, (char)179, (char)159, (char)70, (char)217, (char)27, (char)77, (char)177, (char)187, (char)155, (char)113, (char)156, (char)89, (char)254, (char)151, (char)248, (char)172, (char)10, (char)50, (char)150, (char)228, (char)169, (char)250, (char)153, (char)160, (char)95, (char)68, (char)99, (char)179, (char)193, (char)144, (char)89, (char)191, (char)6, (char)6, (char)242, (char)110, (char)145, (char)48, (char)38, (char)19, (char)158, (char)184, (char)198, (char)171, (char)120, (char)251, (char)227, (char)250, (char)36}, 0) ;
        p267.sequence_SET((char)26455) ;
        p267.first_message_offset_SET((char)118) ;
        p267.length_SET((char)84) ;
        p267.target_component_SET((char)133) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)40);
            assert(pack.target_system_GET() == (char)243);
            assert(pack.sequence_GET() == (char)63132);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.sequence_SET((char)63132) ;
        p268.target_component_SET((char)40) ;
        p268.target_system_SET((char)243) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.resolution_h_GET() == (char)60237);
            assert(pack.uri_LEN(ph) == 142);
            assert(pack.uri_TRY(ph).equals("bmquEoktcbtvPaznnLujxgbuzyedmaecjdhehjnDfpqwmtkexbyHpsendjlvohecivnmwIizvhbzwqceouazHeiIzFjrmkRwyqgypgmsssfashmxrlpwbnnKpXtcgqcaziqhroFaCUrgfv"));
            assert(pack.rotation_GET() == (char)20301);
            assert(pack.resolution_v_GET() == (char)35810);
            assert(pack.bitrate_GET() == 1505107197L);
            assert(pack.framerate_GET() == 1.27035E38F);
            assert(pack.status_GET() == (char)224);
            assert(pack.camera_id_GET() == (char)99);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)99) ;
        p269.rotation_SET((char)20301) ;
        p269.bitrate_SET(1505107197L) ;
        p269.resolution_h_SET((char)60237) ;
        p269.status_SET((char)224) ;
        p269.resolution_v_SET((char)35810) ;
        p269.framerate_SET(1.27035E38F) ;
        p269.uri_SET("bmquEoktcbtvPaznnLujxgbuzyedmaecjdhehjnDfpqwmtkexbyHpsendjlvohecivnmwIizvhbzwqceouazHeiIzFjrmkRwyqgypgmsssfashmxrlpwbnnKpXtcgqcaziqhroFaCUrgfv", PH) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.bitrate_GET() == 744235153L);
            assert(pack.resolution_v_GET() == (char)26863);
            assert(pack.resolution_h_GET() == (char)32684);
            assert(pack.target_component_GET() == (char)92);
            assert(pack.target_system_GET() == (char)27);
            assert(pack.rotation_GET() == (char)18502);
            assert(pack.camera_id_GET() == (char)254);
            assert(pack.framerate_GET() == 2.1787867E38F);
            assert(pack.uri_LEN(ph) == 183);
            assert(pack.uri_TRY(ph).equals("TcsrpYnJDvdhebywvfcjgrdrcyhyjwlotvwibzfupfnaldysqwBoiyafhmvodrdrszkdcgiawlxwipmueryDwxdftvcnciuxnqcoEolSjoyPtAiZqszouyvanesmsrwufitnkwzqSUfrdbXyysewydimPSspimxbiniJCxeccrombimwfeqKnnb"));
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)27) ;
        p270.target_component_SET((char)92) ;
        p270.resolution_h_SET((char)32684) ;
        p270.resolution_v_SET((char)26863) ;
        p270.camera_id_SET((char)254) ;
        p270.bitrate_SET(744235153L) ;
        p270.uri_SET("TcsrpYnJDvdhebywvfcjgrdrcyhyjwlotvwibzfupfnaldysqwBoiyafhmvodrdrszkdcgiawlxwipmueryDwxdftvcnciuxnqcoEolSjoyPtAiZqszouyvanesmsrwufitnkwzqSUfrdbXyysewydimPSspimxbiniJCxeccrombimwfeqKnnb", PH) ;
        p270.rotation_SET((char)18502) ;
        p270.framerate_SET(2.1787867E38F) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 3);
            assert(pack.ssid_TRY(ph).equals("xrw"));
            assert(pack.password_LEN(ph) == 61);
            assert(pack.password_TRY(ph).equals("qevpalvqxmlQWppoJsaijmfjevEdicpjvwsKjafcMqocenvyhksejVsHMtpkk"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("qevpalvqxmlQWppoJsaijmfjevEdicpjvwsKjafcMqocenvyhksejVsHMtpkk", PH) ;
        p299.ssid_SET("xrw", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.max_version_GET() == (char)22175);
            assert(pack.min_version_GET() == (char)57050);
            assert(pack.version_GET() == (char)30345);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)254, (char)87, (char)25, (char)153, (char)148, (char)79, (char)107, (char)178}));
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)10, (char)56, (char)225, (char)231, (char)212, (char)68, (char)48, (char)21}));
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.library_version_hash_SET(new char[] {(char)10, (char)56, (char)225, (char)231, (char)212, (char)68, (char)48, (char)21}, 0) ;
        p300.min_version_SET((char)57050) ;
        p300.version_SET((char)30345) ;
        p300.max_version_SET((char)22175) ;
        p300.spec_version_hash_SET(new char[] {(char)254, (char)87, (char)25, (char)153, (char)148, (char)79, (char)107, (char)178}, 0) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
            assert(pack.uptime_sec_GET() == 3870827405L);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
            assert(pack.sub_mode_GET() == (char)224);
            assert(pack.vendor_specific_status_code_GET() == (char)18827);
            assert(pack.time_usec_GET() == 2578090548958560701L);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK) ;
        p310.vendor_specific_status_code_SET((char)18827) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE) ;
        p310.uptime_sec_SET(3870827405L) ;
        p310.time_usec_SET(2578090548958560701L) ;
        p310.sub_mode_SET((char)224) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.uptime_sec_GET() == 1203227622L);
            assert(pack.hw_version_major_GET() == (char)56);
            assert(pack.name_LEN(ph) == 33);
            assert(pack.name_TRY(ph).equals("fePytmpcyNgfhdKreLduecuqetFCjljhb"));
            assert(pack.time_usec_GET() == 5066212985778920522L);
            assert(pack.sw_vcs_commit_GET() == 2143310640L);
            assert(pack.hw_version_minor_GET() == (char)189);
            assert(pack.sw_version_major_GET() == (char)239);
            assert(pack.sw_version_minor_GET() == (char)5);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)123, (char)158, (char)197, (char)76, (char)146, (char)109, (char)208, (char)44, (char)87, (char)250, (char)249, (char)194, (char)7, (char)0, (char)251, (char)69}));
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.sw_version_minor_SET((char)5) ;
        p311.sw_vcs_commit_SET(2143310640L) ;
        p311.sw_version_major_SET((char)239) ;
        p311.uptime_sec_SET(1203227622L) ;
        p311.hw_version_major_SET((char)56) ;
        p311.hw_unique_id_SET(new char[] {(char)123, (char)158, (char)197, (char)76, (char)146, (char)109, (char)208, (char)44, (char)87, (char)250, (char)249, (char)194, (char)7, (char)0, (char)251, (char)69}, 0) ;
        p311.name_SET("fePytmpcyNgfhdKreLduecuqetFCjljhb", PH) ;
        p311.time_usec_SET(5066212985778920522L) ;
        p311.hw_version_minor_SET((char)189) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short) -14256);
            assert(pack.target_system_GET() == (char)111);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("R"));
            assert(pack.target_component_GET() == (char)76);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_component_SET((char)76) ;
        p320.target_system_SET((char)111) ;
        p320.param_index_SET((short) -14256) ;
        p320.param_id_SET("R", PH) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)122);
            assert(pack.target_system_GET() == (char)198);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)122) ;
        p321.target_system_SET((char)198) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)51920);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
            assert(pack.param_index_GET() == (char)60590);
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("jprjm"));
            assert(pack.param_value_LEN(ph) == 103);
            assert(pack.param_value_TRY(ph).equals("lorkqxtWReqnhsemaismjdzthtbvnfjFbefVjcfcabkkorncnyachoqulttkbkdqfjhqwokjevlocyogkeeitmcplIDmphucnfuwxrs"));
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("jprjm", PH) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32) ;
        p322.param_value_SET("lorkqxtWReqnhsemaismjdzthtbvnfjFbefVjcfcabkkorncnyachoqulttkbkdqfjhqwokjevlocyogkeeitmcplIDmphucnfuwxrs", PH) ;
        p322.param_count_SET((char)51920) ;
        p322.param_index_SET((char)60590) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
            assert(pack.target_component_GET() == (char)29);
            assert(pack.param_value_LEN(ph) == 9);
            assert(pack.param_value_TRY(ph).equals("miaeqxuvb"));
            assert(pack.target_system_GET() == (char)202);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("jfag"));
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_value_SET("miaeqxuvb", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8) ;
        p323.target_system_SET((char)202) ;
        p323.param_id_SET("jfag", PH) ;
        p323.target_component_SET((char)29) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 82);
            assert(pack.param_value_TRY(ph).equals("dvxNzVmsjkxzmussizdJtaufrecmJxsgnzcjqFpxywDeacvhxeakokggcmlkrkiaunXhtuoKqpdmnzeuuo"));
            assert(pack.param_id_LEN(ph) == 6);
            assert(pack.param_id_TRY(ph).equals("jmpfjq"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("jmpfjq", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED) ;
        p324.param_value_SET("dvxNzVmsjkxzmussizdJtaufrecmJxsgnzcjqFpxywDeacvhxeakokggcmlkrkiaunXhtuoKqpdmnzeuuo", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.max_distance_GET() == (char)18551);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)56249, (char)49606, (char)4810, (char)31297, (char)18452, (char)14662, (char)4, (char)4127, (char)41764, (char)8385, (char)26651, (char)4012, (char)18761, (char)19984, (char)29622, (char)28738, (char)44307, (char)50563, (char)5572, (char)9524, (char)10802, (char)40783, (char)57552, (char)13335, (char)9326, (char)28886, (char)5548, (char)4281, (char)13190, (char)43583, (char)6018, (char)48537, (char)39199, (char)5950, (char)35560, (char)3743, (char)6335, (char)62553, (char)43044, (char)38590, (char)54445, (char)45244, (char)33476, (char)25000, (char)56367, (char)16432, (char)46336, (char)62891, (char)41394, (char)45360, (char)17023, (char)44380, (char)44079, (char)54255, (char)39773, (char)49051, (char)38919, (char)57360, (char)20766, (char)26353, (char)8629, (char)22485, (char)38123, (char)58008, (char)38880, (char)62301, (char)22884, (char)51832, (char)54100, (char)62149, (char)50675, (char)1600}));
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.min_distance_GET() == (char)12458);
            assert(pack.time_usec_GET() == 7365412027265280695L);
            assert(pack.increment_GET() == (char)223);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.distances_SET(new char[] {(char)56249, (char)49606, (char)4810, (char)31297, (char)18452, (char)14662, (char)4, (char)4127, (char)41764, (char)8385, (char)26651, (char)4012, (char)18761, (char)19984, (char)29622, (char)28738, (char)44307, (char)50563, (char)5572, (char)9524, (char)10802, (char)40783, (char)57552, (char)13335, (char)9326, (char)28886, (char)5548, (char)4281, (char)13190, (char)43583, (char)6018, (char)48537, (char)39199, (char)5950, (char)35560, (char)3743, (char)6335, (char)62553, (char)43044, (char)38590, (char)54445, (char)45244, (char)33476, (char)25000, (char)56367, (char)16432, (char)46336, (char)62891, (char)41394, (char)45360, (char)17023, (char)44380, (char)44079, (char)54255, (char)39773, (char)49051, (char)38919, (char)57360, (char)20766, (char)26353, (char)8629, (char)22485, (char)38123, (char)58008, (char)38880, (char)62301, (char)22884, (char)51832, (char)54100, (char)62149, (char)50675, (char)1600}, 0) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p330.time_usec_SET(7365412027265280695L) ;
        p330.max_distance_SET((char)18551) ;
        p330.min_distance_SET((char)12458) ;
        p330.increment_SET((char)223) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
    }

}
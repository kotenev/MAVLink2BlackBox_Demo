
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
            long id = id__p(src);
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
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_EMERGENCY);
            assert(pack.custom_mode_GET() == 1092478833L);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_PX4);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
            assert(pack.mavlink_version_GET() == (char)128);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_TRICOPTER);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.custom_mode_SET(1092478833L) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_EMERGENCY) ;
        p0.mavlink_version_SET((char)128) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_PX4) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_TRICOPTER) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.errors_comm_GET() == (char)60558);
            assert(pack.drop_rate_comm_GET() == (char)15792);
            assert(pack.load_GET() == (char)16861);
            assert(pack.errors_count3_GET() == (char)31987);
            assert(pack.current_battery_GET() == (short)22188);
            assert(pack.voltage_battery_GET() == (char)41286);
            assert(pack.battery_remaining_GET() == (byte)9);
            assert(pack.errors_count4_GET() == (char)9078);
            assert(pack.errors_count1_GET() == (char)54342);
            assert(pack.errors_count2_GET() == (char)17088);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY));
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.errors_comm_SET((char)60558) ;
        p1.errors_count1_SET((char)54342) ;
        p1.drop_rate_comm_SET((char)15792) ;
        p1.errors_count3_SET((char)31987) ;
        p1.errors_count2_SET((char)17088) ;
        p1.battery_remaining_SET((byte)9) ;
        p1.current_battery_SET((short)22188) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY)) ;
        p1.voltage_battery_SET((char)41286) ;
        p1.errors_count4_SET((char)9078) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.load_SET((char)16861) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 5629108293081778098L);
            assert(pack.time_boot_ms_GET() == 1725990807L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(1725990807L) ;
        p2.time_unix_usec_SET(5629108293081778098L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == 1.9030259E38F);
            assert(pack.type_mask_GET() == (char)12439);
            assert(pack.afx_GET() == 1.9973987E38F);
            assert(pack.vx_GET() == -7.903185E36F);
            assert(pack.y_GET() == -1.0061171E38F);
            assert(pack.vz_GET() == 1.9772986E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.x_GET() == 1.1663513E38F);
            assert(pack.time_boot_ms_GET() == 1131336914L);
            assert(pack.afy_GET() == 1.3272045E38F);
            assert(pack.yaw_GET() == 1.8698856E38F);
            assert(pack.z_GET() == 2.5127595E38F);
            assert(pack.afz_GET() == 1.98027E38F);
            assert(pack.vy_GET() == -2.9006856E38F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.afx_SET(1.9973987E38F) ;
        p3.vz_SET(1.9772986E38F) ;
        p3.time_boot_ms_SET(1131336914L) ;
        p3.x_SET(1.1663513E38F) ;
        p3.yaw_SET(1.8698856E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p3.y_SET(-1.0061171E38F) ;
        p3.afy_SET(1.3272045E38F) ;
        p3.afz_SET(1.98027E38F) ;
        p3.vx_SET(-7.903185E36F) ;
        p3.z_SET(2.5127595E38F) ;
        p3.vy_SET(-2.9006856E38F) ;
        p3.yaw_rate_SET(1.9030259E38F) ;
        p3.type_mask_SET((char)12439) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)6);
            assert(pack.seq_GET() == 1245114387L);
            assert(pack.time_usec_GET() == 105662807971210344L);
            assert(pack.target_system_GET() == (char)145);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.seq_SET(1245114387L) ;
        p4.time_usec_SET(105662807971210344L) ;
        p4.target_system_SET((char)145) ;
        p4.target_component_SET((char)6) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)38);
            assert(pack.passkey_LEN(ph) == 7);
            assert(pack.passkey_TRY(ph).equals("mfwbfwg"));
            assert(pack.version_GET() == (char)70);
            assert(pack.target_system_GET() == (char)130);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)130) ;
        p5.control_request_SET((char)38) ;
        p5.passkey_SET("mfwbfwg", PH) ;
        p5.version_SET((char)70) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)90);
            assert(pack.control_request_GET() == (char)248);
            assert(pack.ack_GET() == (char)234);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.ack_SET((char)234) ;
        p6.control_request_SET((char)248) ;
        p6.gcs_system_id_SET((char)90) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 15);
            assert(pack.key_TRY(ph).equals("nwoygsexmlnrpyh"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("nwoygsexmlnrpyh", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_AUTO_DISARMED);
            assert(pack.custom_mode_GET() == 3316212425L);
            assert(pack.target_system_GET() == (char)51);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED) ;
        p11.custom_mode_SET(3316212425L) ;
        p11.target_system_SET((char)51) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short) -27946);
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("jplzbxceokvsnjpg"));
            assert(pack.target_component_GET() == (char)128);
            assert(pack.target_system_GET() == (char)146);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_index_SET((short) -27946) ;
        p20.target_system_SET((char)146) ;
        p20.param_id_SET("jplzbxceokvsnjpg", PH) ;
        p20.target_component_SET((char)128) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)176);
            assert(pack.target_component_GET() == (char)46);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)46) ;
        p21.target_system_SET((char)176) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_GET() == 6.9840084E37F);
            assert(pack.param_count_GET() == (char)63481);
            assert(pack.param_index_GET() == (char)49793);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("gcnfklrmspuxru"));
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_index_SET((char)49793) ;
        p22.param_count_SET((char)63481) ;
        p22.param_value_SET(6.9840084E37F) ;
        p22.param_id_SET("gcnfklrmspuxru", PH) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
            assert(pack.param_value_GET() == 1.821551E38F);
            assert(pack.target_component_GET() == (char)62);
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("cdihigopk"));
            assert(pack.target_system_GET() == (char)35);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.param_value_SET(1.821551E38F) ;
        p23.target_system_SET((char)35) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16) ;
        p23.target_component_SET((char)62) ;
        p23.param_id_SET("cdihigopk", PH) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
            assert(pack.satellites_visible_GET() == (char)196);
            assert(pack.time_usec_GET() == 3120969404784463522L);
            assert(pack.cog_GET() == (char)61036);
            assert(pack.lon_GET() == 1840257923);
            assert(pack.vel_GET() == (char)31538);
            assert(pack.vel_acc_TRY(ph) == 1872454728L);
            assert(pack.epv_GET() == (char)21179);
            assert(pack.eph_GET() == (char)55549);
            assert(pack.lat_GET() == -1815416942);
            assert(pack.hdg_acc_TRY(ph) == 2732761316L);
            assert(pack.alt_ellipsoid_TRY(ph) == -1358418432);
            assert(pack.v_acc_TRY(ph) == 1282777028L);
            assert(pack.h_acc_TRY(ph) == 937122051L);
            assert(pack.alt_GET() == -1753219096);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.satellites_visible_SET((char)196) ;
        p24.epv_SET((char)21179) ;
        p24.cog_SET((char)61036) ;
        p24.alt_SET(-1753219096) ;
        p24.vel_SET((char)31538) ;
        p24.hdg_acc_SET(2732761316L, PH) ;
        p24.h_acc_SET(937122051L, PH) ;
        p24.v_acc_SET(1282777028L, PH) ;
        p24.alt_ellipsoid_SET(-1358418432, PH) ;
        p24.lat_SET(-1815416942) ;
        p24.time_usec_SET(3120969404784463522L) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT) ;
        p24.eph_SET((char)55549) ;
        p24.lon_SET(1840257923) ;
        p24.vel_acc_SET(1872454728L, PH) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)110, (char)136, (char)27, (char)204, (char)57, (char)126, (char)153, (char)140, (char)182, (char)147, (char)238, (char)43, (char)224, (char)114, (char)182, (char)131, (char)231, (char)212, (char)201, (char)243}));
            assert(pack.satellites_visible_GET() == (char)221);
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)32, (char)235, (char)152, (char)106, (char)192, (char)190, (char)176, (char)180, (char)65, (char)163, (char)169, (char)18, (char)193, (char)182, (char)142, (char)72, (char)119, (char)6, (char)63, (char)61}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)138, (char)119, (char)249, (char)26, (char)1, (char)171, (char)166, (char)52, (char)31, (char)106, (char)225, (char)78, (char)223, (char)159, (char)249, (char)228, (char)10, (char)34, (char)225, (char)210}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)237, (char)249, (char)215, (char)158, (char)80, (char)163, (char)185, (char)94, (char)4, (char)193, (char)226, (char)186, (char)221, (char)154, (char)179, (char)26, (char)90, (char)229, (char)164, (char)41}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)239, (char)151, (char)219, (char)2, (char)42, (char)133, (char)150, (char)121, (char)89, (char)43, (char)190, (char)28, (char)128, (char)82, (char)105, (char)140, (char)8, (char)17, (char)248, (char)59}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)221) ;
        p25.satellite_azimuth_SET(new char[] {(char)237, (char)249, (char)215, (char)158, (char)80, (char)163, (char)185, (char)94, (char)4, (char)193, (char)226, (char)186, (char)221, (char)154, (char)179, (char)26, (char)90, (char)229, (char)164, (char)41}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)239, (char)151, (char)219, (char)2, (char)42, (char)133, (char)150, (char)121, (char)89, (char)43, (char)190, (char)28, (char)128, (char)82, (char)105, (char)140, (char)8, (char)17, (char)248, (char)59}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)110, (char)136, (char)27, (char)204, (char)57, (char)126, (char)153, (char)140, (char)182, (char)147, (char)238, (char)43, (char)224, (char)114, (char)182, (char)131, (char)231, (char)212, (char)201, (char)243}, 0) ;
        p25.satellite_used_SET(new char[] {(char)32, (char)235, (char)152, (char)106, (char)192, (char)190, (char)176, (char)180, (char)65, (char)163, (char)169, (char)18, (char)193, (char)182, (char)142, (char)72, (char)119, (char)6, (char)63, (char)61}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)138, (char)119, (char)249, (char)26, (char)1, (char)171, (char)166, (char)52, (char)31, (char)106, (char)225, (char)78, (char)223, (char)159, (char)249, (char)228, (char)10, (char)34, (char)225, (char)210}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.xgyro_GET() == (short)15390);
            assert(pack.zacc_GET() == (short) -915);
            assert(pack.zgyro_GET() == (short)23137);
            assert(pack.xmag_GET() == (short)6712);
            assert(pack.time_boot_ms_GET() == 818069442L);
            assert(pack.yacc_GET() == (short) -2584);
            assert(pack.zmag_GET() == (short)24621);
            assert(pack.xacc_GET() == (short) -6459);
            assert(pack.ygyro_GET() == (short) -14762);
            assert(pack.ymag_GET() == (short)26827);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.zmag_SET((short)24621) ;
        p26.xacc_SET((short) -6459) ;
        p26.ymag_SET((short)26827) ;
        p26.zacc_SET((short) -915) ;
        p26.time_boot_ms_SET(818069442L) ;
        p26.yacc_SET((short) -2584) ;
        p26.ygyro_SET((short) -14762) ;
        p26.zgyro_SET((short)23137) ;
        p26.xmag_SET((short)6712) ;
        p26.xgyro_SET((short)15390) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.xgyro_GET() == (short)14667);
            assert(pack.ygyro_GET() == (short)20132);
            assert(pack.time_usec_GET() == 8863678335376808508L);
            assert(pack.zacc_GET() == (short)3722);
            assert(pack.ymag_GET() == (short) -17320);
            assert(pack.zgyro_GET() == (short)195);
            assert(pack.xmag_GET() == (short)16439);
            assert(pack.xacc_GET() == (short)16548);
            assert(pack.yacc_GET() == (short)22207);
            assert(pack.zmag_GET() == (short)29904);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.ymag_SET((short) -17320) ;
        p27.xacc_SET((short)16548) ;
        p27.yacc_SET((short)22207) ;
        p27.zgyro_SET((short)195) ;
        p27.time_usec_SET(8863678335376808508L) ;
        p27.xgyro_SET((short)14667) ;
        p27.xmag_SET((short)16439) ;
        p27.zmag_SET((short)29904) ;
        p27.ygyro_SET((short)20132) ;
        p27.zacc_SET((short)3722) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5004723068707121548L);
            assert(pack.press_diff1_GET() == (short)4878);
            assert(pack.temperature_GET() == (short)4775);
            assert(pack.press_diff2_GET() == (short)14250);
            assert(pack.press_abs_GET() == (short)14593);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.time_usec_SET(5004723068707121548L) ;
        p28.temperature_SET((short)4775) ;
        p28.press_diff1_SET((short)4878) ;
        p28.press_diff2_SET((short)14250) ;
        p28.press_abs_SET((short)14593) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -2.4991214E38F);
            assert(pack.time_boot_ms_GET() == 4010745874L);
            assert(pack.press_diff_GET() == 1.9150223E38F);
            assert(pack.temperature_GET() == (short) -10940);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_diff_SET(1.9150223E38F) ;
        p29.time_boot_ms_SET(4010745874L) ;
        p29.press_abs_SET(-2.4991214E38F) ;
        p29.temperature_SET((short) -10940) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == -1.970581E38F);
            assert(pack.yaw_GET() == 1.9318164E38F);
            assert(pack.pitch_GET() == -2.3413344E38F);
            assert(pack.roll_GET() == -2.7334824E37F);
            assert(pack.yawspeed_GET() == -4.3296353E37F);
            assert(pack.time_boot_ms_GET() == 2002149175L);
            assert(pack.pitchspeed_GET() == 1.5416934E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.yaw_SET(1.9318164E38F) ;
        p30.pitchspeed_SET(1.5416934E38F) ;
        p30.time_boot_ms_SET(2002149175L) ;
        p30.rollspeed_SET(-1.970581E38F) ;
        p30.yawspeed_SET(-4.3296353E37F) ;
        p30.roll_SET(-2.7334824E37F) ;
        p30.pitch_SET(-2.3413344E38F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3536294336L);
            assert(pack.q2_GET() == -2.4177373E38F);
            assert(pack.q3_GET() == -1.4338488E38F);
            assert(pack.rollspeed_GET() == 2.0164862E37F);
            assert(pack.q4_GET() == -2.2066584E38F);
            assert(pack.yawspeed_GET() == 8.2421217E37F);
            assert(pack.pitchspeed_GET() == 1.4509362E38F);
            assert(pack.q1_GET() == 2.3377404E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.rollspeed_SET(2.0164862E37F) ;
        p31.q4_SET(-2.2066584E38F) ;
        p31.time_boot_ms_SET(3536294336L) ;
        p31.q1_SET(2.3377404E38F) ;
        p31.yawspeed_SET(8.2421217E37F) ;
        p31.q3_SET(-1.4338488E38F) ;
        p31.pitchspeed_SET(1.4509362E38F) ;
        p31.q2_SET(-2.4177373E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -3.078362E38F);
            assert(pack.vx_GET() == 2.9877902E38F);
            assert(pack.time_boot_ms_GET() == 3490533156L);
            assert(pack.y_GET() == 2.1455487E38F);
            assert(pack.x_GET() == -2.2492227E38F);
            assert(pack.vy_GET() == -2.9063829E38F);
            assert(pack.vz_GET() == 2.509042E38F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.x_SET(-2.2492227E38F) ;
        p32.vz_SET(2.509042E38F) ;
        p32.vx_SET(2.9877902E38F) ;
        p32.vy_SET(-2.9063829E38F) ;
        p32.z_SET(-3.078362E38F) ;
        p32.y_SET(2.1455487E38F) ;
        p32.time_boot_ms_SET(3490533156L) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 198452889);
            assert(pack.alt_GET() == -877488841);
            assert(pack.lon_GET() == 346519364);
            assert(pack.vx_GET() == (short) -7743);
            assert(pack.time_boot_ms_GET() == 128798694L);
            assert(pack.hdg_GET() == (char)9507);
            assert(pack.vy_GET() == (short)5281);
            assert(pack.relative_alt_GET() == -53735225);
            assert(pack.vz_GET() == (short) -13027);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.lon_SET(346519364) ;
        p33.alt_SET(-877488841) ;
        p33.vy_SET((short)5281) ;
        p33.vz_SET((short) -13027) ;
        p33.hdg_SET((char)9507) ;
        p33.relative_alt_SET(-53735225) ;
        p33.time_boot_ms_SET(128798694L) ;
        p33.lat_SET(198452889) ;
        p33.vx_SET((short) -7743) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan6_scaled_GET() == (short) -5212);
            assert(pack.port_GET() == (char)162);
            assert(pack.chan8_scaled_GET() == (short)18190);
            assert(pack.chan2_scaled_GET() == (short) -17727);
            assert(pack.chan3_scaled_GET() == (short)6563);
            assert(pack.rssi_GET() == (char)223);
            assert(pack.chan1_scaled_GET() == (short)25962);
            assert(pack.chan5_scaled_GET() == (short)10186);
            assert(pack.chan4_scaled_GET() == (short)22872);
            assert(pack.chan7_scaled_GET() == (short) -15057);
            assert(pack.time_boot_ms_GET() == 1051339490L);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan5_scaled_SET((short)10186) ;
        p34.chan2_scaled_SET((short) -17727) ;
        p34.chan6_scaled_SET((short) -5212) ;
        p34.time_boot_ms_SET(1051339490L) ;
        p34.chan8_scaled_SET((short)18190) ;
        p34.chan4_scaled_SET((short)22872) ;
        p34.chan1_scaled_SET((short)25962) ;
        p34.chan3_scaled_SET((short)6563) ;
        p34.rssi_SET((char)223) ;
        p34.chan7_scaled_SET((short) -15057) ;
        p34.port_SET((char)162) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan3_raw_GET() == (char)9998);
            assert(pack.port_GET() == (char)29);
            assert(pack.chan8_raw_GET() == (char)55052);
            assert(pack.chan7_raw_GET() == (char)56943);
            assert(pack.chan1_raw_GET() == (char)29894);
            assert(pack.chan4_raw_GET() == (char)60077);
            assert(pack.chan2_raw_GET() == (char)20845);
            assert(pack.chan6_raw_GET() == (char)55765);
            assert(pack.chan5_raw_GET() == (char)36505);
            assert(pack.time_boot_ms_GET() == 283927074L);
            assert(pack.rssi_GET() == (char)248);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan5_raw_SET((char)36505) ;
        p35.port_SET((char)29) ;
        p35.chan7_raw_SET((char)56943) ;
        p35.rssi_SET((char)248) ;
        p35.chan4_raw_SET((char)60077) ;
        p35.chan6_raw_SET((char)55765) ;
        p35.chan8_raw_SET((char)55052) ;
        p35.chan2_raw_SET((char)20845) ;
        p35.chan1_raw_SET((char)29894) ;
        p35.chan3_raw_SET((char)9998) ;
        p35.time_boot_ms_SET(283927074L) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo6_raw_GET() == (char)59645);
            assert(pack.servo1_raw_GET() == (char)3338);
            assert(pack.servo15_raw_TRY(ph) == (char)46775);
            assert(pack.servo11_raw_TRY(ph) == (char)38457);
            assert(pack.servo10_raw_TRY(ph) == (char)17154);
            assert(pack.servo8_raw_GET() == (char)39536);
            assert(pack.servo9_raw_TRY(ph) == (char)26609);
            assert(pack.servo7_raw_GET() == (char)4104);
            assert(pack.servo14_raw_TRY(ph) == (char)7461);
            assert(pack.servo16_raw_TRY(ph) == (char)24202);
            assert(pack.servo3_raw_GET() == (char)19859);
            assert(pack.servo2_raw_GET() == (char)44910);
            assert(pack.servo13_raw_TRY(ph) == (char)24451);
            assert(pack.servo5_raw_GET() == (char)54048);
            assert(pack.port_GET() == (char)224);
            assert(pack.servo4_raw_GET() == (char)38605);
            assert(pack.time_usec_GET() == 279957128L);
            assert(pack.servo12_raw_TRY(ph) == (char)33873);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo9_raw_SET((char)26609, PH) ;
        p36.servo10_raw_SET((char)17154, PH) ;
        p36.servo5_raw_SET((char)54048) ;
        p36.servo4_raw_SET((char)38605) ;
        p36.servo1_raw_SET((char)3338) ;
        p36.time_usec_SET(279957128L) ;
        p36.servo12_raw_SET((char)33873, PH) ;
        p36.port_SET((char)224) ;
        p36.servo8_raw_SET((char)39536) ;
        p36.servo3_raw_SET((char)19859) ;
        p36.servo2_raw_SET((char)44910) ;
        p36.servo6_raw_SET((char)59645) ;
        p36.servo16_raw_SET((char)24202, PH) ;
        p36.servo7_raw_SET((char)4104) ;
        p36.servo13_raw_SET((char)24451, PH) ;
        p36.servo15_raw_SET((char)46775, PH) ;
        p36.servo11_raw_SET((char)38457, PH) ;
        p36.servo14_raw_SET((char)7461, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short)10294);
            assert(pack.end_index_GET() == (short) -16623);
            assert(pack.target_component_GET() == (char)119);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)91);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_component_SET((char)119) ;
        p37.target_system_SET((char)91) ;
        p37.end_index_SET((short) -16623) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p37.start_index_SET((short)10294) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short) -28321);
            assert(pack.end_index_GET() == (short)25017);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)1);
            assert(pack.target_component_GET() == (char)140);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_component_SET((char)140) ;
        p38.start_index_SET((short) -28321) ;
        p38.target_system_SET((char)1) ;
        p38.end_index_SET((short)25017) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.param1_GET() == 5.0626583E37F);
            assert(pack.param4_GET() == -3.270765E38F);
            assert(pack.target_component_GET() == (char)233);
            assert(pack.current_GET() == (char)201);
            assert(pack.z_GET() == -3.2850987E38F);
            assert(pack.autocontinue_GET() == (char)14);
            assert(pack.param2_GET() == 2.3927227E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_USER_1);
            assert(pack.param3_GET() == 2.537987E38F);
            assert(pack.seq_GET() == (char)61572);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.x_GET() == 2.9523773E38F);
            assert(pack.y_GET() == 1.8972032E38F);
            assert(pack.target_system_GET() == (char)20);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.param1_SET(5.0626583E37F) ;
        p39.param3_SET(2.537987E38F) ;
        p39.seq_SET((char)61572) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p39.autocontinue_SET((char)14) ;
        p39.param4_SET(-3.270765E38F) ;
        p39.param2_SET(2.3927227E37F) ;
        p39.x_SET(2.9523773E38F) ;
        p39.current_SET((char)201) ;
        p39.target_system_SET((char)20) ;
        p39.target_component_SET((char)233) ;
        p39.command_SET(MAV_CMD.MAV_CMD_USER_1) ;
        p39.z_SET(-3.2850987E38F) ;
        p39.y_SET(1.8972032E38F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)235);
            assert(pack.target_component_GET() == (char)107);
            assert(pack.seq_GET() == (char)56917);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p40.target_component_SET((char)107) ;
        p40.seq_SET((char)56917) ;
        p40.target_system_SET((char)235) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)173);
            assert(pack.target_system_GET() == (char)126);
            assert(pack.seq_GET() == (char)37815);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_component_SET((char)173) ;
        p41.target_system_SET((char)126) ;
        p41.seq_SET((char)37815) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)46000);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)46000) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)111);
            assert(pack.target_system_GET() == (char)124);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p43.target_system_SET((char)124) ;
        p43.target_component_SET((char)111) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)190);
            assert(pack.count_GET() == (char)16188);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)216);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)216) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p44.count_SET((char)16188) ;
        p44.target_component_SET((char)190) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)112);
            assert(pack.target_system_GET() == (char)181);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_component_SET((char)112) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p45.target_system_SET((char)181) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)51654);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)51654) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)7);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)205);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p47.target_component_SET((char)7) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE) ;
        p47.target_system_SET((char)205) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == 1279639056);
            assert(pack.latitude_GET() == 645032327);
            assert(pack.altitude_GET() == 627857468);
            assert(pack.target_system_GET() == (char)166);
            assert(pack.time_usec_TRY(ph) == 3454990120398475628L);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.time_usec_SET(3454990120398475628L, PH) ;
        p48.altitude_SET(627857468) ;
        p48.longitude_SET(1279639056) ;
        p48.target_system_SET((char)166) ;
        p48.latitude_SET(645032327) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 1604528644);
            assert(pack.time_usec_TRY(ph) == 6675472325360617421L);
            assert(pack.longitude_GET() == -969722191);
            assert(pack.altitude_GET() == -1269933762);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.altitude_SET(-1269933762) ;
        p49.longitude_SET(-969722191) ;
        p49.time_usec_SET(6675472325360617421L, PH) ;
        p49.latitude_SET(1604528644) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.parameter_rc_channel_index_GET() == (char)125);
            assert(pack.target_system_GET() == (char)168);
            assert(pack.param_value_max_GET() == -2.8080846E38F);
            assert(pack.param_index_GET() == (short)18679);
            assert(pack.scale_GET() == 1.9927601E38F);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("rbzs"));
            assert(pack.param_value_min_GET() == -1.4326565E38F);
            assert(pack.target_component_GET() == (char)165);
            assert(pack.param_value0_GET() == -1.0805667E38F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.param_index_SET((short)18679) ;
        p50.target_system_SET((char)168) ;
        p50.param_value_max_SET(-2.8080846E38F) ;
        p50.param_value_min_SET(-1.4326565E38F) ;
        p50.parameter_rc_channel_index_SET((char)125) ;
        p50.param_value0_SET(-1.0805667E38F) ;
        p50.target_component_SET((char)165) ;
        p50.param_id_SET("rbzs", PH) ;
        p50.scale_SET(1.9927601E38F) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.seq_GET() == (char)23883);
            assert(pack.target_system_GET() == (char)19);
            assert(pack.target_component_GET() == (char)167);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_component_SET((char)167) ;
        p51.seq_SET((char)23883) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p51.target_system_SET((char)19) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2z_GET() == 2.6415427E38F);
            assert(pack.p2y_GET() == 3.1513046E38F);
            assert(pack.target_component_GET() == (char)183);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.p2x_GET() == -1.822711E38F);
            assert(pack.p1z_GET() == -1.3395383E38F);
            assert(pack.p1y_GET() == 8.932357E37F);
            assert(pack.target_system_GET() == (char)117);
            assert(pack.p1x_GET() == 8.616586E37F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p54.p1y_SET(8.932357E37F) ;
        p54.p2z_SET(2.6415427E38F) ;
        p54.target_component_SET((char)183) ;
        p54.p1x_SET(8.616586E37F) ;
        p54.p2y_SET(3.1513046E38F) ;
        p54.target_system_SET((char)117) ;
        p54.p1z_SET(-1.3395383E38F) ;
        p54.p2x_SET(-1.822711E38F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2y_GET() == 3.2902058E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.p1y_GET() == -1.6084877E38F);
            assert(pack.p1z_GET() == -2.0733655E38F);
            assert(pack.p2x_GET() == 1.3973486E38F);
            assert(pack.p1x_GET() == 1.3698951E38F);
            assert(pack.p2z_GET() == -2.1187725E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2y_SET(3.2902058E38F) ;
        p55.p2x_SET(1.3973486E38F) ;
        p55.p2z_SET(-2.1187725E38F) ;
        p55.p1z_SET(-2.0733655E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p55.p1y_SET(-1.6084877E38F) ;
        p55.p1x_SET(1.3698951E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == -3.2101033E38F);
            assert(pack.rollspeed_GET() == -2.0965527E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.4931497E38F, -1.8952066E38F, -1.8336456E38F, 2.1442997E38F}));
            assert(pack.pitchspeed_GET() == 1.3661339E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-9.61128E37F, 5.6956596E37F, -8.3788967E36F, 1.9914606E38F, -2.79295E38F, 1.980484E38F, 9.363485E37F, 2.0489E38F, 3.047531E38F}));
            assert(pack.time_usec_GET() == 4689735467166713356L);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.rollspeed_SET(-2.0965527E38F) ;
        p61.yawspeed_SET(-3.2101033E38F) ;
        p61.pitchspeed_SET(1.3661339E38F) ;
        p61.time_usec_SET(4689735467166713356L) ;
        p61.q_SET(new float[] {-2.4931497E38F, -1.8952066E38F, -1.8336456E38F, 2.1442997E38F}, 0) ;
        p61.covariance_SET(new float[] {-9.61128E37F, 5.6956596E37F, -8.3788967E36F, 1.9914606E38F, -2.79295E38F, 1.980484E38F, 9.363485E37F, 2.0489E38F, 3.047531E38F}, 0) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.wp_dist_GET() == (char)3739);
            assert(pack.nav_roll_GET() == 9.617222E37F);
            assert(pack.xtrack_error_GET() == 2.4979678E38F);
            assert(pack.target_bearing_GET() == (short)23530);
            assert(pack.alt_error_GET() == -5.260869E37F);
            assert(pack.nav_pitch_GET() == -2.04453E38F);
            assert(pack.nav_bearing_GET() == (short)9679);
            assert(pack.aspd_error_GET() == 8.932724E37F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.alt_error_SET(-5.260869E37F) ;
        p62.nav_pitch_SET(-2.04453E38F) ;
        p62.target_bearing_SET((short)23530) ;
        p62.wp_dist_SET((char)3739) ;
        p62.nav_roll_SET(9.617222E37F) ;
        p62.aspd_error_SET(8.932724E37F) ;
        p62.nav_bearing_SET((short)9679) ;
        p62.xtrack_error_SET(2.4979678E38F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2425598315572961788L);
            assert(pack.alt_GET() == 1979707413);
            assert(pack.vx_GET() == 1.5243827E38F);
            assert(pack.relative_alt_GET() == -603878663);
            assert(pack.lon_GET() == 1241595709);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.4483308E38F, 7.1662535E37F, 1.8569175E36F, 6.8325774E37F, 8.8164695E35F, -2.3257675E38F, 8.464959E37F, -1.7163032E38F, 2.2360229E38F, 5.777807E36F, -1.0517369E38F, -4.9765955E37F, 2.6072727E38F, -2.735448E38F, 2.7710073E38F, -3.4383664E37F, 1.8720052E38F, -2.8207828E38F, -6.739212E37F, -2.2594961E38F, -1.2935297E38F, 2.7810563E38F, -1.1476475E38F, 2.4015326E37F, 1.0421856E38F, -8.709278E37F, -3.3372415E38F, 7.819345E37F, 1.9258675E38F, 2.6458592E38F, -1.2912251E38F, -1.3361414E38F, 1.345609E38F, 3.1567516E38F, 2.328769E38F, 1.1985801E38F}));
            assert(pack.vz_GET() == -1.206128E38F);
            assert(pack.vy_GET() == 3.3061028E36F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.lat_GET() == 1160816326);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.relative_alt_SET(-603878663) ;
        p63.vy_SET(3.3061028E36F) ;
        p63.covariance_SET(new float[] {2.4483308E38F, 7.1662535E37F, 1.8569175E36F, 6.8325774E37F, 8.8164695E35F, -2.3257675E38F, 8.464959E37F, -1.7163032E38F, 2.2360229E38F, 5.777807E36F, -1.0517369E38F, -4.9765955E37F, 2.6072727E38F, -2.735448E38F, 2.7710073E38F, -3.4383664E37F, 1.8720052E38F, -2.8207828E38F, -6.739212E37F, -2.2594961E38F, -1.2935297E38F, 2.7810563E38F, -1.1476475E38F, 2.4015326E37F, 1.0421856E38F, -8.709278E37F, -3.3372415E38F, 7.819345E37F, 1.9258675E38F, 2.6458592E38F, -1.2912251E38F, -1.3361414E38F, 1.345609E38F, 3.1567516E38F, 2.328769E38F, 1.1985801E38F}, 0) ;
        p63.lat_SET(1160816326) ;
        p63.time_usec_SET(2425598315572961788L) ;
        p63.vz_SET(-1.206128E38F) ;
        p63.lon_SET(1241595709) ;
        p63.vx_SET(1.5243827E38F) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p63.alt_SET(1979707413) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.az_GET() == 8.293878E37F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {4.9561777E37F, 8.848764E37F, 2.8884382E37F, 7.7805727E37F, -3.3996635E38F, 2.5824014E38F, 2.1703398E37F, 3.382268E38F, -5.2194956E37F, -2.4973727E38F, 1.5889387E38F, 6.152042E37F, 3.1791923E38F, 2.4170789E38F, -3.3238606E38F, 2.7779504E38F, 1.1416736E38F, -2.570271E38F, 3.302599E38F, 3.1414264E38F, -4.666743E37F, 1.6958454E38F, 7.341282E37F, -1.3760195E38F, -1.0258549E38F, -2.04602E38F, 3.589729E36F, 2.5907032E38F, -3.0400336E38F, -2.4745144E38F, -7.886427E37F, 2.2109112E38F, 3.1125528E38F, -1.7000195E38F, 1.7826958E38F, -5.336221E37F, -2.9326046E38F, 8.69186E37F, -1.3111445E38F, -1.2121229E38F, 1.4611624E38F, -6.0954523E37F, 2.0980642E38F, -3.2247104E38F, 9.056971E36F}));
            assert(pack.vy_GET() == -1.6678688E38F);
            assert(pack.z_GET() == 4.500361E37F);
            assert(pack.x_GET() == 1.1038824E37F);
            assert(pack.ax_GET() == -2.6645227E38F);
            assert(pack.vx_GET() == 5.7189565E37F);
            assert(pack.time_usec_GET() == 1356676580369223402L);
            assert(pack.y_GET() == 2.4541032E38F);
            assert(pack.vz_GET() == 3.267893E38F);
            assert(pack.ay_GET() == 1.1117759E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.ay_SET(1.1117759E38F) ;
        p64.x_SET(1.1038824E37F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p64.vy_SET(-1.6678688E38F) ;
        p64.y_SET(2.4541032E38F) ;
        p64.z_SET(4.500361E37F) ;
        p64.covariance_SET(new float[] {4.9561777E37F, 8.848764E37F, 2.8884382E37F, 7.7805727E37F, -3.3996635E38F, 2.5824014E38F, 2.1703398E37F, 3.382268E38F, -5.2194956E37F, -2.4973727E38F, 1.5889387E38F, 6.152042E37F, 3.1791923E38F, 2.4170789E38F, -3.3238606E38F, 2.7779504E38F, 1.1416736E38F, -2.570271E38F, 3.302599E38F, 3.1414264E38F, -4.666743E37F, 1.6958454E38F, 7.341282E37F, -1.3760195E38F, -1.0258549E38F, -2.04602E38F, 3.589729E36F, 2.5907032E38F, -3.0400336E38F, -2.4745144E38F, -7.886427E37F, 2.2109112E38F, 3.1125528E38F, -1.7000195E38F, 1.7826958E38F, -5.336221E37F, -2.9326046E38F, 8.69186E37F, -1.3111445E38F, -1.2121229E38F, 1.4611624E38F, -6.0954523E37F, 2.0980642E38F, -3.2247104E38F, 9.056971E36F}, 0) ;
        p64.vz_SET(3.267893E38F) ;
        p64.vx_SET(5.7189565E37F) ;
        p64.time_usec_SET(1356676580369223402L) ;
        p64.ax_SET(-2.6645227E38F) ;
        p64.az_SET(8.293878E37F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan10_raw_GET() == (char)882);
            assert(pack.chan11_raw_GET() == (char)29794);
            assert(pack.chan18_raw_GET() == (char)29355);
            assert(pack.rssi_GET() == (char)85);
            assert(pack.chan9_raw_GET() == (char)2458);
            assert(pack.chan5_raw_GET() == (char)18116);
            assert(pack.chan2_raw_GET() == (char)17156);
            assert(pack.chan4_raw_GET() == (char)25689);
            assert(pack.chan16_raw_GET() == (char)14610);
            assert(pack.chan7_raw_GET() == (char)43596);
            assert(pack.chan13_raw_GET() == (char)1422);
            assert(pack.chancount_GET() == (char)137);
            assert(pack.chan8_raw_GET() == (char)55734);
            assert(pack.chan1_raw_GET() == (char)38464);
            assert(pack.chan17_raw_GET() == (char)44216);
            assert(pack.chan6_raw_GET() == (char)14526);
            assert(pack.time_boot_ms_GET() == 4214622170L);
            assert(pack.chan3_raw_GET() == (char)16743);
            assert(pack.chan12_raw_GET() == (char)3626);
            assert(pack.chan14_raw_GET() == (char)37644);
            assert(pack.chan15_raw_GET() == (char)16660);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan12_raw_SET((char)3626) ;
        p65.chan17_raw_SET((char)44216) ;
        p65.chan7_raw_SET((char)43596) ;
        p65.chan6_raw_SET((char)14526) ;
        p65.chan1_raw_SET((char)38464) ;
        p65.chan16_raw_SET((char)14610) ;
        p65.chan9_raw_SET((char)2458) ;
        p65.chan10_raw_SET((char)882) ;
        p65.chan15_raw_SET((char)16660) ;
        p65.chancount_SET((char)137) ;
        p65.chan2_raw_SET((char)17156) ;
        p65.chan14_raw_SET((char)37644) ;
        p65.chan3_raw_SET((char)16743) ;
        p65.chan18_raw_SET((char)29355) ;
        p65.chan4_raw_SET((char)25689) ;
        p65.chan8_raw_SET((char)55734) ;
        p65.chan13_raw_SET((char)1422) ;
        p65.chan5_raw_SET((char)18116) ;
        p65.rssi_SET((char)85) ;
        p65.chan11_raw_SET((char)29794) ;
        p65.time_boot_ms_SET(4214622170L) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.start_stop_GET() == (char)170);
            assert(pack.req_message_rate_GET() == (char)64548);
            assert(pack.target_component_GET() == (char)136);
            assert(pack.req_stream_id_GET() == (char)0);
            assert(pack.target_system_GET() == (char)246);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.start_stop_SET((char)170) ;
        p66.target_component_SET((char)136) ;
        p66.req_stream_id_SET((char)0) ;
        p66.target_system_SET((char)246) ;
        p66.req_message_rate_SET((char)64548) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.message_rate_GET() == (char)54076);
            assert(pack.on_off_GET() == (char)109);
            assert(pack.stream_id_GET() == (char)19);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)54076) ;
        p67.stream_id_SET((char)19) ;
        p67.on_off_SET((char)109) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.buttons_GET() == (char)24648);
            assert(pack.target_GET() == (char)103);
            assert(pack.r_GET() == (short)15395);
            assert(pack.z_GET() == (short)26868);
            assert(pack.y_GET() == (short)12991);
            assert(pack.x_GET() == (short) -8907);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.buttons_SET((char)24648) ;
        p69.z_SET((short)26868) ;
        p69.r_SET((short)15395) ;
        p69.x_SET((short) -8907) ;
        p69.target_SET((char)103) ;
        p69.y_SET((short)12991) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)25536);
            assert(pack.target_system_GET() == (char)59);
            assert(pack.chan3_raw_GET() == (char)1375);
            assert(pack.chan7_raw_GET() == (char)31275);
            assert(pack.chan6_raw_GET() == (char)31482);
            assert(pack.chan5_raw_GET() == (char)34329);
            assert(pack.target_component_GET() == (char)216);
            assert(pack.chan8_raw_GET() == (char)34298);
            assert(pack.chan1_raw_GET() == (char)47666);
            assert(pack.chan2_raw_GET() == (char)48312);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.target_component_SET((char)216) ;
        p70.target_system_SET((char)59) ;
        p70.chan2_raw_SET((char)48312) ;
        p70.chan3_raw_SET((char)1375) ;
        p70.chan6_raw_SET((char)31482) ;
        p70.chan1_raw_SET((char)47666) ;
        p70.chan4_raw_SET((char)25536) ;
        p70.chan5_raw_SET((char)34329) ;
        p70.chan7_raw_SET((char)31275) ;
        p70.chan8_raw_SET((char)34298) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param4_GET() == -1.6452752E38F);
            assert(pack.param2_GET() == -2.3210238E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.param3_GET() == -3.0782463E38F);
            assert(pack.seq_GET() == (char)64638);
            assert(pack.y_GET() == -1275186362);
            assert(pack.autocontinue_GET() == (char)154);
            assert(pack.x_GET() == 2012987857);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL);
            assert(pack.target_component_GET() == (char)249);
            assert(pack.current_GET() == (char)110);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.param1_GET() == -2.8707179E38F);
            assert(pack.target_system_GET() == (char)65);
            assert(pack.z_GET() == 3.1878553E38F);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.target_component_SET((char)249) ;
        p73.target_system_SET((char)65) ;
        p73.x_SET(2012987857) ;
        p73.seq_SET((char)64638) ;
        p73.param2_SET(-2.3210238E38F) ;
        p73.param4_SET(-1.6452752E38F) ;
        p73.current_SET((char)110) ;
        p73.z_SET(3.1878553E38F) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p73.param3_SET(-3.0782463E38F) ;
        p73.y_SET(-1275186362) ;
        p73.command_SET(MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL) ;
        p73.param1_SET(-2.8707179E38F) ;
        p73.autocontinue_SET((char)154) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.airspeed_GET() == -1.2167243E37F);
            assert(pack.groundspeed_GET() == -5.868225E37F);
            assert(pack.throttle_GET() == (char)51845);
            assert(pack.climb_GET() == -2.3720136E38F);
            assert(pack.heading_GET() == (short) -28802);
            assert(pack.alt_GET() == 2.4603007E37F);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.airspeed_SET(-1.2167243E37F) ;
        p74.groundspeed_SET(-5.868225E37F) ;
        p74.climb_SET(-2.3720136E38F) ;
        p74.throttle_SET((char)51845) ;
        p74.alt_SET(2.4603007E37F) ;
        p74.heading_SET((short) -28802) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.current_GET() == (char)18);
            assert(pack.target_component_GET() == (char)108);
            assert(pack.autocontinue_GET() == (char)219);
            assert(pack.y_GET() == -53209672);
            assert(pack.param2_GET() == -2.0129162E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.z_GET() == -1.3483346E38F);
            assert(pack.target_system_GET() == (char)151);
            assert(pack.param4_GET() == -1.224591E37F);
            assert(pack.param1_GET() == 3.1747504E38F);
            assert(pack.x_GET() == -498672896);
            assert(pack.param3_GET() == -1.0323866E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.autocontinue_SET((char)219) ;
        p75.current_SET((char)18) ;
        p75.command_SET(MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION) ;
        p75.param2_SET(-2.0129162E38F) ;
        p75.target_component_SET((char)108) ;
        p75.y_SET(-53209672) ;
        p75.param4_SET(-1.224591E37F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p75.x_SET(-498672896) ;
        p75.param3_SET(-1.0323866E38F) ;
        p75.z_SET(-1.3483346E38F) ;
        p75.param1_SET(3.1747504E38F) ;
        p75.target_system_SET((char)151) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param3_GET() == 2.5075916E38F);
            assert(pack.param4_GET() == 2.0008341E37F);
            assert(pack.param2_GET() == 3.0233065E38F);
            assert(pack.confirmation_GET() == (char)233);
            assert(pack.target_component_GET() == (char)251);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE);
            assert(pack.param7_GET() == 1.03233827E37F);
            assert(pack.param5_GET() == 3.1879908E38F);
            assert(pack.param6_GET() == -2.0235056E38F);
            assert(pack.param1_GET() == -2.4645148E38F);
            assert(pack.target_system_GET() == (char)80);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.param1_SET(-2.4645148E38F) ;
        p76.param2_SET(3.0233065E38F) ;
        p76.target_component_SET((char)251) ;
        p76.target_system_SET((char)80) ;
        p76.param3_SET(2.5075916E38F) ;
        p76.param7_SET(1.03233827E37F) ;
        p76.param6_SET(-2.0235056E38F) ;
        p76.param4_SET(2.0008341E37F) ;
        p76.confirmation_SET((char)233) ;
        p76.command_SET(MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE) ;
        p76.param5_SET(3.1879908E38F) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_TRY(ph) == (char)135);
            assert(pack.progress_TRY(ph) == (char)110);
            assert(pack.result_param2_TRY(ph) == -565891085);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_ROI);
            assert(pack.target_system_TRY(ph) == (char)155);
        });
        COMMAND_ACK p77 = new COMMAND_ACK();
        PH.setPack(p77);
        p77.result_param2_SET(-565891085, PH) ;
        p77.target_system_SET((char)155, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED) ;
        p77.command_SET(MAV_CMD.MAV_CMD_NAV_ROI) ;
        p77.target_component_SET((char)135, PH) ;
        p77.progress_SET((char)110, PH) ;
        TestChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 2.1047302E38F);
            assert(pack.time_boot_ms_GET() == 1847530103L);
            assert(pack.manual_override_switch_GET() == (char)120);
            assert(pack.thrust_GET() == -2.310724E38F);
            assert(pack.mode_switch_GET() == (char)224);
            assert(pack.roll_GET() == -9.80431E37F);
            assert(pack.pitch_GET() == 3.126145E38F);
        });
        MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.yaw_SET(2.1047302E38F) ;
        p81.mode_switch_SET((char)224) ;
        p81.roll_SET(-9.80431E37F) ;
        p81.time_boot_ms_SET(1847530103L) ;
        p81.thrust_SET(-2.310724E38F) ;
        p81.pitch_SET(3.126145E38F) ;
        p81.manual_override_switch_SET((char)120) ;
        TestChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_roll_rate_GET() == -1.3354662E38F);
            assert(pack.body_pitch_rate_GET() == 1.266226E38F);
            assert(pack.thrust_GET() == -7.0853165E37F);
            assert(pack.time_boot_ms_GET() == 2250605945L);
            assert(pack.target_system_GET() == (char)28);
            assert(pack.body_yaw_rate_GET() == 1.227464E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.139163E37F, 3.1783372E37F, -2.9172908E38F, 2.19817E38F}));
            assert(pack.type_mask_GET() == (char)213);
            assert(pack.target_component_GET() == (char)52);
        });
        SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.q_SET(new float[] {3.139163E37F, 3.1783372E37F, -2.9172908E38F, 2.19817E38F}, 0) ;
        p82.body_pitch_rate_SET(1.266226E38F) ;
        p82.time_boot_ms_SET(2250605945L) ;
        p82.target_component_SET((char)52) ;
        p82.type_mask_SET((char)213) ;
        p82.thrust_SET(-7.0853165E37F) ;
        p82.body_yaw_rate_SET(1.227464E38F) ;
        p82.body_roll_rate_SET(-1.3354662E38F) ;
        p82.target_system_SET((char)28) ;
        TestChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)66);
            assert(pack.thrust_GET() == -3.5299463E37F);
            assert(pack.body_yaw_rate_GET() == -2.2664104E38F);
            assert(pack.body_roll_rate_GET() == -3.3726994E38F);
            assert(pack.body_pitch_rate_GET() == 1.7284576E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.6658428E38F, -7.51662E37F, -7.467898E37F, -2.317121E38F}));
            assert(pack.time_boot_ms_GET() == 4255718998L);
        });
        ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_yaw_rate_SET(-2.2664104E38F) ;
        p83.thrust_SET(-3.5299463E37F) ;
        p83.body_roll_rate_SET(-3.3726994E38F) ;
        p83.body_pitch_rate_SET(1.7284576E38F) ;
        p83.time_boot_ms_SET(4255718998L) ;
        p83.type_mask_SET((char)66) ;
        p83.q_SET(new float[] {2.6658428E38F, -7.51662E37F, -7.467898E37F, -2.317121E38F}, 0) ;
        TestChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)149);
            assert(pack.target_component_GET() == (char)164);
            assert(pack.x_GET() == 1.0802725E38F);
            assert(pack.yaw_rate_GET() == -6.715317E37F);
            assert(pack.vz_GET() == 2.7388824E38F);
            assert(pack.z_GET() == 2.921448E38F);
            assert(pack.type_mask_GET() == (char)57788);
            assert(pack.afz_GET() == -2.6586414E38F);
            assert(pack.y_GET() == 2.1634814E38F);
            assert(pack.yaw_GET() == -2.6852148E38F);
            assert(pack.time_boot_ms_GET() == 2769292930L);
            assert(pack.afx_GET() == 4.5871007E37F);
            assert(pack.vx_GET() == -1.3841928E38F);
            assert(pack.vy_GET() == 8.355805E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.afy_GET() == 1.0892353E38F);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.yaw_rate_SET(-6.715317E37F) ;
        p84.vx_SET(-1.3841928E38F) ;
        p84.afx_SET(4.5871007E37F) ;
        p84.target_component_SET((char)164) ;
        p84.x_SET(1.0802725E38F) ;
        p84.afz_SET(-2.6586414E38F) ;
        p84.target_system_SET((char)149) ;
        p84.vy_SET(8.355805E37F) ;
        p84.afy_SET(1.0892353E38F) ;
        p84.time_boot_ms_SET(2769292930L) ;
        p84.yaw_SET(-2.6852148E38F) ;
        p84.vz_SET(2.7388824E38F) ;
        p84.y_SET(2.1634814E38F) ;
        p84.type_mask_SET((char)57788) ;
        p84.z_SET(2.921448E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 1.0767618E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.lat_int_GET() == -365373415);
            assert(pack.type_mask_GET() == (char)44624);
            assert(pack.target_component_GET() == (char)164);
            assert(pack.target_system_GET() == (char)43);
            assert(pack.time_boot_ms_GET() == 565704347L);
            assert(pack.afy_GET() == 2.4348173E37F);
            assert(pack.afx_GET() == -3.0816359E38F);
            assert(pack.afz_GET() == 2.4494766E38F);
            assert(pack.yaw_GET() == 3.1090806E38F);
            assert(pack.lon_int_GET() == -849127296);
            assert(pack.alt_GET() == 2.596295E38F);
            assert(pack.vz_GET() == -1.2004549E38F);
            assert(pack.yaw_rate_GET() == 7.5624384E37F);
            assert(pack.vx_GET() == 2.9040606E38F);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p86.vy_SET(1.0767618E37F) ;
        p86.lon_int_SET(-849127296) ;
        p86.afx_SET(-3.0816359E38F) ;
        p86.target_system_SET((char)43) ;
        p86.lat_int_SET(-365373415) ;
        p86.type_mask_SET((char)44624) ;
        p86.yaw_SET(3.1090806E38F) ;
        p86.alt_SET(2.596295E38F) ;
        p86.target_component_SET((char)164) ;
        p86.time_boot_ms_SET(565704347L) ;
        p86.yaw_rate_SET(7.5624384E37F) ;
        p86.vz_SET(-1.2004549E38F) ;
        p86.vx_SET(2.9040606E38F) ;
        p86.afy_SET(2.4348173E37F) ;
        p86.afz_SET(2.4494766E38F) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1662760645L);
            assert(pack.vx_GET() == -2.5859364E38F);
            assert(pack.yaw_GET() == -2.014334E38F);
            assert(pack.vz_GET() == 3.2477634E38F);
            assert(pack.afz_GET() == -3.296172E37F);
            assert(pack.alt_GET() == -1.2169561E37F);
            assert(pack.afy_GET() == 1.4735161E38F);
            assert(pack.lat_int_GET() == 1425044630);
            assert(pack.type_mask_GET() == (char)5930);
            assert(pack.yaw_rate_GET() == -1.1337162E38F);
            assert(pack.lon_int_GET() == 589767772);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.afx_GET() == -1.0947735E38F);
            assert(pack.vy_GET() == -2.3155642E38F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.lon_int_SET(589767772) ;
        p87.vy_SET(-2.3155642E38F) ;
        p87.yaw_rate_SET(-1.1337162E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p87.vz_SET(3.2477634E38F) ;
        p87.alt_SET(-1.2169561E37F) ;
        p87.afz_SET(-3.296172E37F) ;
        p87.lat_int_SET(1425044630) ;
        p87.afx_SET(-1.0947735E38F) ;
        p87.afy_SET(1.4735161E38F) ;
        p87.yaw_SET(-2.014334E38F) ;
        p87.vx_SET(-2.5859364E38F) ;
        p87.type_mask_SET((char)5930) ;
        p87.time_boot_ms_SET(1662760645L) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 2.8590369E38F);
            assert(pack.y_GET() == -2.7809834E38F);
            assert(pack.pitch_GET() == 1.7345614E38F);
            assert(pack.time_boot_ms_GET() == 2716464376L);
            assert(pack.roll_GET() == -3.3818677E38F);
            assert(pack.x_GET() == -2.5517878E38F);
            assert(pack.z_GET() == 9.627635E37F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.z_SET(9.627635E37F) ;
        p89.yaw_SET(2.8590369E38F) ;
        p89.time_boot_ms_SET(2716464376L) ;
        p89.pitch_SET(1.7345614E38F) ;
        p89.x_SET(-2.5517878E38F) ;
        p89.roll_SET(-3.3818677E38F) ;
        p89.y_SET(-2.7809834E38F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == -2.518767E38F);
            assert(pack.lon_GET() == -1895514793);
            assert(pack.roll_GET() == 1.7751285E38F);
            assert(pack.vz_GET() == (short) -17147);
            assert(pack.lat_GET() == -687932327);
            assert(pack.yacc_GET() == (short) -1972);
            assert(pack.yaw_GET() == 3.1956117E38F);
            assert(pack.alt_GET() == 672634992);
            assert(pack.vx_GET() == (short) -23755);
            assert(pack.rollspeed_GET() == -6.708137E37F);
            assert(pack.vy_GET() == (short)16351);
            assert(pack.pitchspeed_GET() == 3.1649617E38F);
            assert(pack.xacc_GET() == (short)28616);
            assert(pack.pitch_GET() == 3.2834696E38F);
            assert(pack.time_usec_GET() == 3311318984575321953L);
            assert(pack.zacc_GET() == (short) -4437);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.time_usec_SET(3311318984575321953L) ;
        p90.lon_SET(-1895514793) ;
        p90.pitchspeed_SET(3.1649617E38F) ;
        p90.vy_SET((short)16351) ;
        p90.yawspeed_SET(-2.518767E38F) ;
        p90.vx_SET((short) -23755) ;
        p90.yacc_SET((short) -1972) ;
        p90.lat_SET(-687932327) ;
        p90.xacc_SET((short)28616) ;
        p90.pitch_SET(3.2834696E38F) ;
        p90.rollspeed_SET(-6.708137E37F) ;
        p90.vz_SET((short) -17147) ;
        p90.roll_SET(1.7751285E38F) ;
        p90.yaw_SET(3.1956117E38F) ;
        p90.alt_SET(672634992) ;
        p90.zacc_SET((short) -4437) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_PREFLIGHT);
            assert(pack.time_usec_GET() == 5116157069740608237L);
            assert(pack.aux3_GET() == 1.236613E38F);
            assert(pack.pitch_elevator_GET() == -1.7072864E38F);
            assert(pack.roll_ailerons_GET() == 2.0951581E38F);
            assert(pack.aux1_GET() == -1.3772236E38F);
            assert(pack.throttle_GET() == 2.8528414E38F);
            assert(pack.nav_mode_GET() == (char)145);
            assert(pack.yaw_rudder_GET() == -2.9786106E38F);
            assert(pack.aux2_GET() == 3.3423344E38F);
            assert(pack.aux4_GET() == -1.7961222E38F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.throttle_SET(2.8528414E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_PREFLIGHT) ;
        p91.pitch_elevator_SET(-1.7072864E38F) ;
        p91.aux4_SET(-1.7961222E38F) ;
        p91.aux3_SET(1.236613E38F) ;
        p91.nav_mode_SET((char)145) ;
        p91.roll_ailerons_SET(2.0951581E38F) ;
        p91.aux2_SET(3.3423344E38F) ;
        p91.time_usec_SET(5116157069740608237L) ;
        p91.yaw_rudder_SET(-2.9786106E38F) ;
        p91.aux1_SET(-1.3772236E38F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6296201848135250077L);
            assert(pack.chan11_raw_GET() == (char)60490);
            assert(pack.chan10_raw_GET() == (char)17072);
            assert(pack.chan1_raw_GET() == (char)10840);
            assert(pack.chan12_raw_GET() == (char)60963);
            assert(pack.chan9_raw_GET() == (char)11901);
            assert(pack.chan5_raw_GET() == (char)16972);
            assert(pack.rssi_GET() == (char)200);
            assert(pack.chan4_raw_GET() == (char)45862);
            assert(pack.chan2_raw_GET() == (char)18728);
            assert(pack.chan7_raw_GET() == (char)24157);
            assert(pack.chan3_raw_GET() == (char)13346);
            assert(pack.chan8_raw_GET() == (char)8796);
            assert(pack.chan6_raw_GET() == (char)48233);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan5_raw_SET((char)16972) ;
        p92.chan11_raw_SET((char)60490) ;
        p92.chan2_raw_SET((char)18728) ;
        p92.rssi_SET((char)200) ;
        p92.chan9_raw_SET((char)11901) ;
        p92.chan7_raw_SET((char)24157) ;
        p92.chan1_raw_SET((char)10840) ;
        p92.chan10_raw_SET((char)17072) ;
        p92.time_usec_SET(6296201848135250077L) ;
        p92.chan8_raw_SET((char)8796) ;
        p92.chan6_raw_SET((char)48233) ;
        p92.chan12_raw_SET((char)60963) ;
        p92.chan3_raw_SET((char)13346) ;
        p92.chan4_raw_SET((char)45862) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_AUTO_ARMED);
            assert(pack.time_usec_GET() == 7024712996174053946L);
            assert(pack.flags_GET() == 8786051419074844739L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-3.120245E38F, -2.8438514E38F, 2.1338097E38F, -9.294375E37F, -6.330892E37F, 3.0539384E38F, 1.4500595E38F, -1.769931E38F, 1.5357447E38F, -3.2813166E38F, 3.2079069E38F, -1.6662708E38F, 1.7546473E38F, -1.770291E38F, -2.9813158E38F, -2.6270907E38F}));
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.controls_SET(new float[] {-3.120245E38F, -2.8438514E38F, 2.1338097E38F, -9.294375E37F, -6.330892E37F, 3.0539384E38F, 1.4500595E38F, -1.769931E38F, 1.5357447E38F, -3.2813166E38F, 3.2079069E38F, -1.6662708E38F, 1.7546473E38F, -1.770291E38F, -2.9813158E38F, -2.6270907E38F}, 0) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED) ;
        p93.time_usec_SET(7024712996174053946L) ;
        p93.flags_SET(8786051419074844739L) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_y_GET() == (short) -30426);
            assert(pack.sensor_id_GET() == (char)28);
            assert(pack.quality_GET() == (char)27);
            assert(pack.flow_comp_m_x_GET() == -3.0817805E38F);
            assert(pack.flow_x_GET() == (short) -14948);
            assert(pack.flow_rate_x_TRY(ph) == 3.8241403E37F);
            assert(pack.ground_distance_GET() == 3.173074E38F);
            assert(pack.flow_rate_y_TRY(ph) == 1.0686828E38F);
            assert(pack.flow_comp_m_y_GET() == -3.2472988E38F);
            assert(pack.time_usec_GET() == 6809336327937104225L);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.quality_SET((char)27) ;
        p100.flow_rate_y_SET(1.0686828E38F, PH) ;
        p100.flow_x_SET((short) -14948) ;
        p100.time_usec_SET(6809336327937104225L) ;
        p100.sensor_id_SET((char)28) ;
        p100.flow_comp_m_x_SET(-3.0817805E38F) ;
        p100.flow_comp_m_y_SET(-3.2472988E38F) ;
        p100.flow_y_SET((short) -30426) ;
        p100.flow_rate_x_SET(3.8241403E37F, PH) ;
        p100.ground_distance_SET(3.173074E38F) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 2.5317165E38F);
            assert(pack.y_GET() == 3.7039262E37F);
            assert(pack.usec_GET() == 1827433076345743180L);
            assert(pack.x_GET() == 1.2855225E38F);
            assert(pack.z_GET() == -2.5226208E38F);
            assert(pack.yaw_GET() == 2.15818E38F);
            assert(pack.roll_GET() == -1.959524E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.roll_SET(-1.959524E38F) ;
        p101.yaw_SET(2.15818E38F) ;
        p101.usec_SET(1827433076345743180L) ;
        p101.pitch_SET(2.5317165E38F) ;
        p101.y_SET(3.7039262E37F) ;
        p101.z_SET(-2.5226208E38F) ;
        p101.x_SET(1.2855225E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 2713817052217804382L);
            assert(pack.y_GET() == 2.453179E38F);
            assert(pack.z_GET() == -1.815305E38F);
            assert(pack.roll_GET() == -1.3162891E38F);
            assert(pack.yaw_GET() == 4.0019669E37F);
            assert(pack.x_GET() == 3.1929778E38F);
            assert(pack.pitch_GET() == -1.2856207E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.x_SET(3.1929778E38F) ;
        p102.roll_SET(-1.3162891E38F) ;
        p102.usec_SET(2713817052217804382L) ;
        p102.z_SET(-1.815305E38F) ;
        p102.yaw_SET(4.0019669E37F) ;
        p102.pitch_SET(-1.2856207E38F) ;
        p102.y_SET(2.453179E38F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 4034141633696990023L);
            assert(pack.x_GET() == 1.3408797E38F);
            assert(pack.z_GET() == -2.7433523E37F);
            assert(pack.y_GET() == -4.4622173E36F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(4034141633696990023L) ;
        p103.x_SET(1.3408797E38F) ;
        p103.z_SET(-2.7433523E37F) ;
        p103.y_SET(-4.4622173E36F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 1.6614463E38F);
            assert(pack.pitch_GET() == -2.2275065E38F);
            assert(pack.roll_GET() == -2.2853974E38F);
            assert(pack.y_GET() == 1.8329808E38F);
            assert(pack.z_GET() == -1.2927618E38F);
            assert(pack.yaw_GET() == -1.5604013E38F);
            assert(pack.usec_GET() == 4111008102947767915L);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.roll_SET(-2.2853974E38F) ;
        p104.z_SET(-1.2927618E38F) ;
        p104.y_SET(1.8329808E38F) ;
        p104.x_SET(1.6614463E38F) ;
        p104.yaw_SET(-1.5604013E38F) ;
        p104.usec_SET(4111008102947767915L) ;
        p104.pitch_SET(-2.2275065E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.xgyro_GET() == -2.2237967E38F);
            assert(pack.zgyro_GET() == -1.551763E38F);
            assert(pack.zacc_GET() == 4.1544275E37F);
            assert(pack.yacc_GET() == 2.8678812E38F);
            assert(pack.ygyro_GET() == 6.24999E37F);
            assert(pack.temperature_GET() == -2.3633167E38F);
            assert(pack.xmag_GET() == -1.9609194E38F);
            assert(pack.xacc_GET() == 3.3253351E38F);
            assert(pack.zmag_GET() == 3.3145808E38F);
            assert(pack.time_usec_GET() == 5497697816623499812L);
            assert(pack.abs_pressure_GET() == 3.1228037E38F);
            assert(pack.ymag_GET() == 4.4926404E37F);
            assert(pack.fields_updated_GET() == (char)35441);
            assert(pack.pressure_alt_GET() == -2.1941127E38F);
            assert(pack.diff_pressure_GET() == -2.878567E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.zmag_SET(3.3145808E38F) ;
        p105.diff_pressure_SET(-2.878567E38F) ;
        p105.fields_updated_SET((char)35441) ;
        p105.time_usec_SET(5497697816623499812L) ;
        p105.pressure_alt_SET(-2.1941127E38F) ;
        p105.xmag_SET(-1.9609194E38F) ;
        p105.xgyro_SET(-2.2237967E38F) ;
        p105.temperature_SET(-2.3633167E38F) ;
        p105.yacc_SET(2.8678812E38F) ;
        p105.ymag_SET(4.4926404E37F) ;
        p105.zacc_SET(4.1544275E37F) ;
        p105.xacc_SET(3.3253351E38F) ;
        p105.zgyro_SET(-1.551763E38F) ;
        p105.ygyro_SET(6.24999E37F) ;
        p105.abs_pressure_SET(3.1228037E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_y_GET() == -2.843998E38F);
            assert(pack.integration_time_us_GET() == 2748403031L);
            assert(pack.integrated_ygyro_GET() == -1.0948469E38F);
            assert(pack.integrated_xgyro_GET() == 1.7959598E37F);
            assert(pack.integrated_x_GET() == 2.076196E38F);
            assert(pack.distance_GET() == 6.2683847E37F);
            assert(pack.quality_GET() == (char)96);
            assert(pack.integrated_zgyro_GET() == -7.7140535E37F);
            assert(pack.sensor_id_GET() == (char)214);
            assert(pack.temperature_GET() == (short) -16194);
            assert(pack.time_usec_GET() == 1261732250174174735L);
            assert(pack.time_delta_distance_us_GET() == 3951149737L);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.distance_SET(6.2683847E37F) ;
        p106.integrated_zgyro_SET(-7.7140535E37F) ;
        p106.temperature_SET((short) -16194) ;
        p106.integrated_ygyro_SET(-1.0948469E38F) ;
        p106.integrated_x_SET(2.076196E38F) ;
        p106.integration_time_us_SET(2748403031L) ;
        p106.integrated_y_SET(-2.843998E38F) ;
        p106.integrated_xgyro_SET(1.7959598E37F) ;
        p106.quality_SET((char)96) ;
        p106.time_delta_distance_us_SET(3951149737L) ;
        p106.time_usec_SET(1261732250174174735L) ;
        p106.sensor_id_SET((char)214) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == -3.2997247E38F);
            assert(pack.xgyro_GET() == -1.9572357E38F);
            assert(pack.ygyro_GET() == -5.778122E37F);
            assert(pack.yacc_GET() == -2.3758942E38F);
            assert(pack.ymag_GET() == 2.7311997E38F);
            assert(pack.pressure_alt_GET() == -2.2358357E38F);
            assert(pack.fields_updated_GET() == 3905012173L);
            assert(pack.temperature_GET() == -2.8672415E38F);
            assert(pack.xmag_GET() == -1.1478422E38F);
            assert(pack.zgyro_GET() == 1.4171167E38F);
            assert(pack.xacc_GET() == 1.9554969E38F);
            assert(pack.zacc_GET() == 5.582025E37F);
            assert(pack.diff_pressure_GET() == 4.509869E37F);
            assert(pack.time_usec_GET() == 2464003444563680520L);
            assert(pack.abs_pressure_GET() == 1.6437454E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.zgyro_SET(1.4171167E38F) ;
        p107.temperature_SET(-2.8672415E38F) ;
        p107.xgyro_SET(-1.9572357E38F) ;
        p107.zacc_SET(5.582025E37F) ;
        p107.diff_pressure_SET(4.509869E37F) ;
        p107.pressure_alt_SET(-2.2358357E38F) ;
        p107.yacc_SET(-2.3758942E38F) ;
        p107.time_usec_SET(2464003444563680520L) ;
        p107.ymag_SET(2.7311997E38F) ;
        p107.ygyro_SET(-5.778122E37F) ;
        p107.xacc_SET(1.9554969E38F) ;
        p107.abs_pressure_SET(1.6437454E38F) ;
        p107.zmag_SET(-3.2997247E38F) ;
        p107.xmag_SET(-1.1478422E38F) ;
        p107.fields_updated_SET(3905012173L) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == 1.3306063E38F);
            assert(pack.xacc_GET() == 4.154646E37F);
            assert(pack.q1_GET() == -1.0937071E38F);
            assert(pack.lon_GET() == -2.9317925E38F);
            assert(pack.yacc_GET() == 9.1403484E36F);
            assert(pack.q3_GET() == 2.8744205E38F);
            assert(pack.ve_GET() == -7.849882E37F);
            assert(pack.q4_GET() == 2.0220177E38F);
            assert(pack.vn_GET() == -4.412289E37F);
            assert(pack.zgyro_GET() == 2.9445455E38F);
            assert(pack.vd_GET() == 3.1065548E37F);
            assert(pack.xgyro_GET() == -2.268841E38F);
            assert(pack.std_dev_horz_GET() == 3.9401278E37F);
            assert(pack.std_dev_vert_GET() == -9.030443E37F);
            assert(pack.pitch_GET() == 1.9163486E38F);
            assert(pack.alt_GET() == 4.80663E37F);
            assert(pack.ygyro_GET() == 1.5858541E38F);
            assert(pack.lat_GET() == 1.4533542E38F);
            assert(pack.q2_GET() == -4.923383E37F);
            assert(pack.roll_GET() == -2.9683398E38F);
            assert(pack.yaw_GET() == 3.707205E37F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.yacc_SET(9.1403484E36F) ;
        p108.lon_SET(-2.9317925E38F) ;
        p108.zacc_SET(1.3306063E38F) ;
        p108.alt_SET(4.80663E37F) ;
        p108.yaw_SET(3.707205E37F) ;
        p108.ygyro_SET(1.5858541E38F) ;
        p108.ve_SET(-7.849882E37F) ;
        p108.q2_SET(-4.923383E37F) ;
        p108.vd_SET(3.1065548E37F) ;
        p108.std_dev_vert_SET(-9.030443E37F) ;
        p108.std_dev_horz_SET(3.9401278E37F) ;
        p108.vn_SET(-4.412289E37F) ;
        p108.zgyro_SET(2.9445455E38F) ;
        p108.q1_SET(-1.0937071E38F) ;
        p108.q3_SET(2.8744205E38F) ;
        p108.xgyro_SET(-2.268841E38F) ;
        p108.roll_SET(-2.9683398E38F) ;
        p108.lat_SET(1.4533542E38F) ;
        p108.pitch_SET(1.9163486E38F) ;
        p108.xacc_SET(4.154646E37F) ;
        p108.q4_SET(2.0220177E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.txbuf_GET() == (char)3);
            assert(pack.rxerrors_GET() == (char)38843);
            assert(pack.remrssi_GET() == (char)111);
            assert(pack.fixed__GET() == (char)30577);
            assert(pack.noise_GET() == (char)196);
            assert(pack.rssi_GET() == (char)103);
            assert(pack.remnoise_GET() == (char)80);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rxerrors_SET((char)38843) ;
        p109.noise_SET((char)196) ;
        p109.txbuf_SET((char)3) ;
        p109.remnoise_SET((char)80) ;
        p109.fixed__SET((char)30577) ;
        p109.rssi_SET((char)103) ;
        p109.remrssi_SET((char)111) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)232);
            assert(pack.target_system_GET() == (char)226);
            assert(pack.target_component_GET() == (char)224);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)28, (char)247, (char)120, (char)65, (char)90, (char)18, (char)92, (char)100, (char)239, (char)2, (char)203, (char)118, (char)190, (char)21, (char)169, (char)138, (char)13, (char)139, (char)57, (char)228, (char)50, (char)144, (char)130, (char)225, (char)5, (char)192, (char)18, (char)250, (char)220, (char)244, (char)98, (char)16, (char)90, (char)144, (char)184, (char)231, (char)252, (char)5, (char)19, (char)145, (char)117, (char)42, (char)156, (char)184, (char)157, (char)21, (char)220, (char)234, (char)13, (char)206, (char)231, (char)213, (char)7, (char)169, (char)2, (char)103, (char)243, (char)135, (char)11, (char)1, (char)189, (char)174, (char)139, (char)24, (char)184, (char)201, (char)42, (char)107, (char)70, (char)227, (char)251, (char)61, (char)161, (char)116, (char)100, (char)64, (char)208, (char)249, (char)135, (char)144, (char)183, (char)181, (char)2, (char)117, (char)183, (char)132, (char)84, (char)98, (char)247, (char)165, (char)176, (char)101, (char)129, (char)125, (char)175, (char)135, (char)42, (char)8, (char)109, (char)35, (char)142, (char)190, (char)138, (char)32, (char)103, (char)14, (char)155, (char)148, (char)187, (char)126, (char)49, (char)235, (char)6, (char)159, (char)227, (char)139, (char)219, (char)141, (char)145, (char)24, (char)109, (char)119, (char)115, (char)20, (char)15, (char)192, (char)73, (char)203, (char)104, (char)215, (char)249, (char)219, (char)70, (char)38, (char)77, (char)5, (char)255, (char)134, (char)43, (char)225, (char)249, (char)237, (char)83, (char)178, (char)121, (char)24, (char)172, (char)120, (char)215, (char)12, (char)232, (char)130, (char)206, (char)23, (char)42, (char)217, (char)202, (char)125, (char)54, (char)172, (char)147, (char)21, (char)186, (char)36, (char)180, (char)147, (char)21, (char)43, (char)64, (char)229, (char)56, (char)53, (char)93, (char)103, (char)215, (char)102, (char)138, (char)165, (char)209, (char)124, (char)124, (char)205, (char)114, (char)37, (char)101, (char)122, (char)204, (char)96, (char)165, (char)172, (char)6, (char)77, (char)229, (char)131, (char)186, (char)167, (char)248, (char)124, (char)16, (char)36, (char)226, (char)24, (char)75, (char)4, (char)31, (char)128, (char)40, (char)148, (char)128, (char)141, (char)181, (char)79, (char)66, (char)200, (char)78, (char)148, (char)96, (char)101, (char)130, (char)199, (char)249, (char)57, (char)104, (char)52, (char)90, (char)125, (char)240, (char)249, (char)98, (char)183, (char)180, (char)61, (char)118, (char)142, (char)50, (char)231, (char)68, (char)145, (char)182, (char)74, (char)29, (char)23, (char)202, (char)102, (char)104, (char)248, (char)17, (char)208, (char)111, (char)59, (char)133}));
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.payload_SET(new char[] {(char)28, (char)247, (char)120, (char)65, (char)90, (char)18, (char)92, (char)100, (char)239, (char)2, (char)203, (char)118, (char)190, (char)21, (char)169, (char)138, (char)13, (char)139, (char)57, (char)228, (char)50, (char)144, (char)130, (char)225, (char)5, (char)192, (char)18, (char)250, (char)220, (char)244, (char)98, (char)16, (char)90, (char)144, (char)184, (char)231, (char)252, (char)5, (char)19, (char)145, (char)117, (char)42, (char)156, (char)184, (char)157, (char)21, (char)220, (char)234, (char)13, (char)206, (char)231, (char)213, (char)7, (char)169, (char)2, (char)103, (char)243, (char)135, (char)11, (char)1, (char)189, (char)174, (char)139, (char)24, (char)184, (char)201, (char)42, (char)107, (char)70, (char)227, (char)251, (char)61, (char)161, (char)116, (char)100, (char)64, (char)208, (char)249, (char)135, (char)144, (char)183, (char)181, (char)2, (char)117, (char)183, (char)132, (char)84, (char)98, (char)247, (char)165, (char)176, (char)101, (char)129, (char)125, (char)175, (char)135, (char)42, (char)8, (char)109, (char)35, (char)142, (char)190, (char)138, (char)32, (char)103, (char)14, (char)155, (char)148, (char)187, (char)126, (char)49, (char)235, (char)6, (char)159, (char)227, (char)139, (char)219, (char)141, (char)145, (char)24, (char)109, (char)119, (char)115, (char)20, (char)15, (char)192, (char)73, (char)203, (char)104, (char)215, (char)249, (char)219, (char)70, (char)38, (char)77, (char)5, (char)255, (char)134, (char)43, (char)225, (char)249, (char)237, (char)83, (char)178, (char)121, (char)24, (char)172, (char)120, (char)215, (char)12, (char)232, (char)130, (char)206, (char)23, (char)42, (char)217, (char)202, (char)125, (char)54, (char)172, (char)147, (char)21, (char)186, (char)36, (char)180, (char)147, (char)21, (char)43, (char)64, (char)229, (char)56, (char)53, (char)93, (char)103, (char)215, (char)102, (char)138, (char)165, (char)209, (char)124, (char)124, (char)205, (char)114, (char)37, (char)101, (char)122, (char)204, (char)96, (char)165, (char)172, (char)6, (char)77, (char)229, (char)131, (char)186, (char)167, (char)248, (char)124, (char)16, (char)36, (char)226, (char)24, (char)75, (char)4, (char)31, (char)128, (char)40, (char)148, (char)128, (char)141, (char)181, (char)79, (char)66, (char)200, (char)78, (char)148, (char)96, (char)101, (char)130, (char)199, (char)249, (char)57, (char)104, (char)52, (char)90, (char)125, (char)240, (char)249, (char)98, (char)183, (char)180, (char)61, (char)118, (char)142, (char)50, (char)231, (char)68, (char)145, (char)182, (char)74, (char)29, (char)23, (char)202, (char)102, (char)104, (char)248, (char)17, (char)208, (char)111, (char)59, (char)133}, 0) ;
        p110.target_component_SET((char)224) ;
        p110.target_network_SET((char)232) ;
        p110.target_system_SET((char)226) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -1399428810186905754L);
            assert(pack.ts1_GET() == -5870608332257767136L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(-5870608332257767136L) ;
        p111.tc1_SET(-1399428810186905754L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 2434782659L);
            assert(pack.time_usec_GET() == 3367635749060987658L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(3367635749060987658L) ;
        p112.seq_SET(2434782659L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.epv_GET() == (char)63226);
            assert(pack.fix_type_GET() == (char)216);
            assert(pack.lon_GET() == -621155272);
            assert(pack.cog_GET() == (char)39567);
            assert(pack.vn_GET() == (short) -28129);
            assert(pack.lat_GET() == 131129698);
            assert(pack.time_usec_GET() == 4335383951205951776L);
            assert(pack.eph_GET() == (char)17897);
            assert(pack.vd_GET() == (short)15616);
            assert(pack.alt_GET() == -111476819);
            assert(pack.ve_GET() == (short) -25644);
            assert(pack.vel_GET() == (char)25435);
            assert(pack.satellites_visible_GET() == (char)113);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.cog_SET((char)39567) ;
        p113.alt_SET(-111476819) ;
        p113.ve_SET((short) -25644) ;
        p113.eph_SET((char)17897) ;
        p113.satellites_visible_SET((char)113) ;
        p113.lat_SET(131129698) ;
        p113.fix_type_SET((char)216) ;
        p113.vel_SET((char)25435) ;
        p113.vn_SET((short) -28129) ;
        p113.time_usec_SET(4335383951205951776L) ;
        p113.epv_SET((char)63226) ;
        p113.vd_SET((short)15616) ;
        p113.lon_SET(-621155272) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.distance_GET() == -3.1918507E38F);
            assert(pack.integrated_x_GET() == -1.6927308E38F);
            assert(pack.integrated_ygyro_GET() == 1.4273165E38F);
            assert(pack.time_usec_GET() == 4726504397242229791L);
            assert(pack.temperature_GET() == (short) -30695);
            assert(pack.integrated_zgyro_GET() == 2.360577E38F);
            assert(pack.sensor_id_GET() == (char)162);
            assert(pack.integration_time_us_GET() == 2950567975L);
            assert(pack.time_delta_distance_us_GET() == 3171074269L);
            assert(pack.quality_GET() == (char)212);
            assert(pack.integrated_xgyro_GET() == -1.8144938E38F);
            assert(pack.integrated_y_GET() == -2.0543288E38F);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_ygyro_SET(1.4273165E38F) ;
        p114.distance_SET(-3.1918507E38F) ;
        p114.sensor_id_SET((char)162) ;
        p114.integrated_zgyro_SET(2.360577E38F) ;
        p114.temperature_SET((short) -30695) ;
        p114.time_delta_distance_us_SET(3171074269L) ;
        p114.quality_SET((char)212) ;
        p114.integrated_y_SET(-2.0543288E38F) ;
        p114.integrated_xgyro_SET(-1.8144938E38F) ;
        p114.integrated_x_SET(-1.6927308E38F) ;
        p114.integration_time_us_SET(2950567975L) ;
        p114.time_usec_SET(4726504397242229791L) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1316832420);
            assert(pack.pitchspeed_GET() == -1.9775514E38F);
            assert(pack.rollspeed_GET() == 2.7995308E37F);
            assert(pack.zacc_GET() == (short)5925);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {-1.6969446E38F, 2.6109767E38F, -1.2347957E38F, -3.0938565E38F}));
            assert(pack.yacc_GET() == (short)1313);
            assert(pack.ind_airspeed_GET() == (char)24183);
            assert(pack.vy_GET() == (short)4938);
            assert(pack.true_airspeed_GET() == (char)55120);
            assert(pack.time_usec_GET() == 370350155950909233L);
            assert(pack.vz_GET() == (short)22059);
            assert(pack.xacc_GET() == (short) -9653);
            assert(pack.vx_GET() == (short) -17526);
            assert(pack.lat_GET() == 1813100616);
            assert(pack.yawspeed_GET() == -1.9773709E38F);
            assert(pack.alt_GET() == -901929971);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.zacc_SET((short)5925) ;
        p115.ind_airspeed_SET((char)24183) ;
        p115.attitude_quaternion_SET(new float[] {-1.6969446E38F, 2.6109767E38F, -1.2347957E38F, -3.0938565E38F}, 0) ;
        p115.yacc_SET((short)1313) ;
        p115.time_usec_SET(370350155950909233L) ;
        p115.true_airspeed_SET((char)55120) ;
        p115.lat_SET(1813100616) ;
        p115.rollspeed_SET(2.7995308E37F) ;
        p115.vy_SET((short)4938) ;
        p115.vx_SET((short) -17526) ;
        p115.alt_SET(-901929971) ;
        p115.lon_SET(-1316832420) ;
        p115.pitchspeed_SET(-1.9775514E38F) ;
        p115.xacc_SET((short) -9653) ;
        p115.yawspeed_SET(-1.9773709E38F) ;
        p115.vz_SET((short)22059) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.xgyro_GET() == (short)29933);
            assert(pack.xmag_GET() == (short) -5730);
            assert(pack.xacc_GET() == (short)23374);
            assert(pack.time_boot_ms_GET() == 1566821774L);
            assert(pack.zmag_GET() == (short)24789);
            assert(pack.ymag_GET() == (short) -5080);
            assert(pack.ygyro_GET() == (short)17289);
            assert(pack.zacc_GET() == (short) -11227);
            assert(pack.zgyro_GET() == (short) -21464);
            assert(pack.yacc_GET() == (short)29339);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.xacc_SET((short)23374) ;
        p116.xmag_SET((short) -5730) ;
        p116.ymag_SET((short) -5080) ;
        p116.ygyro_SET((short)17289) ;
        p116.zmag_SET((short)24789) ;
        p116.xgyro_SET((short)29933) ;
        p116.time_boot_ms_SET(1566821774L) ;
        p116.yacc_SET((short)29339) ;
        p116.zacc_SET((short) -11227) ;
        p116.zgyro_SET((short) -21464) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_GET() == (char)26120);
            assert(pack.target_system_GET() == (char)45);
            assert(pack.target_component_GET() == (char)70);
            assert(pack.start_GET() == (char)705);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)45) ;
        p117.target_component_SET((char)70) ;
        p117.end_SET((char)26120) ;
        p117.start_SET((char)705) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.num_logs_GET() == (char)44709);
            assert(pack.last_log_num_GET() == (char)41145);
            assert(pack.time_utc_GET() == 188976807L);
            assert(pack.id_GET() == (char)6072);
            assert(pack.size_GET() == 973697655L);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.size_SET(973697655L) ;
        p118.id_SET((char)6072) ;
        p118.time_utc_SET(188976807L) ;
        p118.last_log_num_SET((char)41145) ;
        p118.num_logs_SET((char)44709) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == 412566873L);
            assert(pack.ofs_GET() == 800207088L);
            assert(pack.target_system_GET() == (char)231);
            assert(pack.id_GET() == (char)37892);
            assert(pack.target_component_GET() == (char)34);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)231) ;
        p119.id_SET((char)37892) ;
        p119.target_component_SET((char)34) ;
        p119.count_SET(412566873L) ;
        p119.ofs_SET(800207088L) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)235, (char)233, (char)198, (char)22, (char)103, (char)137, (char)234, (char)218, (char)47, (char)236, (char)18, (char)205, (char)16, (char)112, (char)249, (char)248, (char)250, (char)65, (char)147, (char)26, (char)89, (char)106, (char)63, (char)53, (char)174, (char)246, (char)34, (char)205, (char)80, (char)177, (char)228, (char)69, (char)38, (char)32, (char)86, (char)141, (char)50, (char)133, (char)244, (char)125, (char)48, (char)6, (char)154, (char)41, (char)47, (char)153, (char)195, (char)118, (char)155, (char)165, (char)53, (char)105, (char)49, (char)19, (char)167, (char)115, (char)208, (char)35, (char)212, (char)145, (char)147, (char)165, (char)82, (char)44, (char)149, (char)60, (char)40, (char)219, (char)163, (char)95, (char)73, (char)13, (char)161, (char)71, (char)172, (char)74, (char)12, (char)115, (char)0, (char)100, (char)244, (char)200, (char)56, (char)82, (char)174, (char)144, (char)165, (char)8, (char)235, (char)148}));
            assert(pack.ofs_GET() == 2299754058L);
            assert(pack.id_GET() == (char)156);
            assert(pack.count_GET() == (char)212);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.ofs_SET(2299754058L) ;
        p120.data__SET(new char[] {(char)235, (char)233, (char)198, (char)22, (char)103, (char)137, (char)234, (char)218, (char)47, (char)236, (char)18, (char)205, (char)16, (char)112, (char)249, (char)248, (char)250, (char)65, (char)147, (char)26, (char)89, (char)106, (char)63, (char)53, (char)174, (char)246, (char)34, (char)205, (char)80, (char)177, (char)228, (char)69, (char)38, (char)32, (char)86, (char)141, (char)50, (char)133, (char)244, (char)125, (char)48, (char)6, (char)154, (char)41, (char)47, (char)153, (char)195, (char)118, (char)155, (char)165, (char)53, (char)105, (char)49, (char)19, (char)167, (char)115, (char)208, (char)35, (char)212, (char)145, (char)147, (char)165, (char)82, (char)44, (char)149, (char)60, (char)40, (char)219, (char)163, (char)95, (char)73, (char)13, (char)161, (char)71, (char)172, (char)74, (char)12, (char)115, (char)0, (char)100, (char)244, (char)200, (char)56, (char)82, (char)174, (char)144, (char)165, (char)8, (char)235, (char)148}, 0) ;
        p120.id_SET((char)156) ;
        p120.count_SET((char)212) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)246);
            assert(pack.target_component_GET() == (char)251);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)251) ;
        p121.target_system_SET((char)246) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)92);
            assert(pack.target_system_GET() == (char)203);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_component_SET((char)92) ;
        p122.target_system_SET((char)203) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)19, (char)146, (char)162, (char)242, (char)146, (char)83, (char)6, (char)168, (char)103, (char)94, (char)63, (char)23, (char)213, (char)68, (char)246, (char)100, (char)235, (char)21, (char)45, (char)152, (char)100, (char)32, (char)40, (char)203, (char)247, (char)249, (char)41, (char)219, (char)49, (char)126, (char)103, (char)139, (char)3, (char)19, (char)76, (char)199, (char)113, (char)189, (char)95, (char)89, (char)160, (char)209, (char)137, (char)97, (char)67, (char)111, (char)158, (char)136, (char)5, (char)45, (char)147, (char)223, (char)26, (char)214, (char)184, (char)140, (char)115, (char)3, (char)68, (char)68, (char)44, (char)171, (char)49, (char)147, (char)60, (char)246, (char)150, (char)120, (char)170, (char)245, (char)31, (char)200, (char)158, (char)82, (char)181, (char)63, (char)0, (char)19, (char)9, (char)46, (char)145, (char)250, (char)14, (char)255, (char)249, (char)204, (char)50, (char)78, (char)217, (char)94, (char)153, (char)68, (char)212, (char)32, (char)62, (char)193, (char)223, (char)231, (char)168, (char)154, (char)157, (char)247, (char)62, (char)125, (char)77, (char)6, (char)247, (char)16, (char)84, (char)157}));
            assert(pack.len_GET() == (char)58);
            assert(pack.target_system_GET() == (char)3);
            assert(pack.target_component_GET() == (char)133);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.data__SET(new char[] {(char)19, (char)146, (char)162, (char)242, (char)146, (char)83, (char)6, (char)168, (char)103, (char)94, (char)63, (char)23, (char)213, (char)68, (char)246, (char)100, (char)235, (char)21, (char)45, (char)152, (char)100, (char)32, (char)40, (char)203, (char)247, (char)249, (char)41, (char)219, (char)49, (char)126, (char)103, (char)139, (char)3, (char)19, (char)76, (char)199, (char)113, (char)189, (char)95, (char)89, (char)160, (char)209, (char)137, (char)97, (char)67, (char)111, (char)158, (char)136, (char)5, (char)45, (char)147, (char)223, (char)26, (char)214, (char)184, (char)140, (char)115, (char)3, (char)68, (char)68, (char)44, (char)171, (char)49, (char)147, (char)60, (char)246, (char)150, (char)120, (char)170, (char)245, (char)31, (char)200, (char)158, (char)82, (char)181, (char)63, (char)0, (char)19, (char)9, (char)46, (char)145, (char)250, (char)14, (char)255, (char)249, (char)204, (char)50, (char)78, (char)217, (char)94, (char)153, (char)68, (char)212, (char)32, (char)62, (char)193, (char)223, (char)231, (char)168, (char)154, (char)157, (char)247, (char)62, (char)125, (char)77, (char)6, (char)247, (char)16, (char)84, (char)157}, 0) ;
        p123.target_system_SET((char)3) ;
        p123.target_component_SET((char)133) ;
        p123.len_SET((char)58) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
            assert(pack.time_usec_GET() == 4630301126263165379L);
            assert(pack.epv_GET() == (char)44965);
            assert(pack.cog_GET() == (char)41269);
            assert(pack.eph_GET() == (char)26544);
            assert(pack.dgps_numch_GET() == (char)195);
            assert(pack.dgps_age_GET() == 2448262764L);
            assert(pack.lat_GET() == -1520592331);
            assert(pack.alt_GET() == 639824112);
            assert(pack.vel_GET() == (char)4486);
            assert(pack.satellites_visible_GET() == (char)119);
            assert(pack.lon_GET() == -1749902845);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.lon_SET(-1749902845) ;
        p124.cog_SET((char)41269) ;
        p124.time_usec_SET(4630301126263165379L) ;
        p124.epv_SET((char)44965) ;
        p124.alt_SET(639824112) ;
        p124.dgps_numch_SET((char)195) ;
        p124.dgps_age_SET(2448262764L) ;
        p124.satellites_visible_SET((char)119) ;
        p124.eph_SET((char)26544) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX) ;
        p124.lat_SET(-1520592331) ;
        p124.vel_SET((char)4486) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vservo_GET() == (char)60711);
            assert(pack.Vcc_GET() == (char)62391);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID));
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)60711) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID)) ;
        p125.Vcc_SET((char)62391) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)22);
            assert(pack.timeout_GET() == (char)51705);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)19, (char)198, (char)196, (char)174, (char)195, (char)254, (char)164, (char)61, (char)46, (char)7, (char)106, (char)183, (char)9, (char)186, (char)198, (char)223, (char)204, (char)214, (char)94, (char)216, (char)70, (char)167, (char)161, (char)220, (char)94, (char)159, (char)149, (char)167, (char)97, (char)145, (char)96, (char)175, (char)240, (char)241, (char)127, (char)249, (char)102, (char)243, (char)8, (char)213, (char)203, (char)36, (char)54, (char)73, (char)203, (char)183, (char)218, (char)151, (char)2, (char)199, (char)231, (char)38, (char)4, (char)152, (char)38, (char)90, (char)218, (char)44, (char)122, (char)100, (char)135, (char)24, (char)160, (char)215, (char)74, (char)97, (char)169, (char)72, (char)41, (char)211}));
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
            assert(pack.baudrate_GET() == 2580586187L);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING)) ;
        p126.data__SET(new char[] {(char)19, (char)198, (char)196, (char)174, (char)195, (char)254, (char)164, (char)61, (char)46, (char)7, (char)106, (char)183, (char)9, (char)186, (char)198, (char)223, (char)204, (char)214, (char)94, (char)216, (char)70, (char)167, (char)161, (char)220, (char)94, (char)159, (char)149, (char)167, (char)97, (char)145, (char)96, (char)175, (char)240, (char)241, (char)127, (char)249, (char)102, (char)243, (char)8, (char)213, (char)203, (char)36, (char)54, (char)73, (char)203, (char)183, (char)218, (char)151, (char)2, (char)199, (char)231, (char)38, (char)4, (char)152, (char)38, (char)90, (char)218, (char)44, (char)122, (char)100, (char)135, (char)24, (char)160, (char)215, (char)74, (char)97, (char)169, (char)72, (char)41, (char)211}, 0) ;
        p126.baudrate_SET(2580586187L) ;
        p126.count_SET((char)22) ;
        p126.timeout_SET((char)51705) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_a_mm_GET() == 1978599813);
            assert(pack.baseline_b_mm_GET() == 154567831);
            assert(pack.rtk_receiver_id_GET() == (char)51);
            assert(pack.rtk_health_GET() == (char)152);
            assert(pack.baseline_coords_type_GET() == (char)172);
            assert(pack.tow_GET() == 761717041L);
            assert(pack.time_last_baseline_ms_GET() == 275585673L);
            assert(pack.nsats_GET() == (char)249);
            assert(pack.rtk_rate_GET() == (char)210);
            assert(pack.iar_num_hypotheses_GET() == -1162828623);
            assert(pack.wn_GET() == (char)5090);
            assert(pack.baseline_c_mm_GET() == -1693204615);
            assert(pack.accuracy_GET() == 3799507130L);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.rtk_rate_SET((char)210) ;
        p127.baseline_coords_type_SET((char)172) ;
        p127.rtk_health_SET((char)152) ;
        p127.wn_SET((char)5090) ;
        p127.baseline_b_mm_SET(154567831) ;
        p127.baseline_c_mm_SET(-1693204615) ;
        p127.nsats_SET((char)249) ;
        p127.baseline_a_mm_SET(1978599813) ;
        p127.time_last_baseline_ms_SET(275585673L) ;
        p127.tow_SET(761717041L) ;
        p127.rtk_receiver_id_SET((char)51) ;
        p127.iar_num_hypotheses_SET(-1162828623) ;
        p127.accuracy_SET(3799507130L) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.accuracy_GET() == 2091273456L);
            assert(pack.baseline_b_mm_GET() == 2021936481);
            assert(pack.iar_num_hypotheses_GET() == 1890492800);
            assert(pack.rtk_health_GET() == (char)2);
            assert(pack.baseline_a_mm_GET() == 52653500);
            assert(pack.baseline_c_mm_GET() == 1422004505);
            assert(pack.tow_GET() == 174839106L);
            assert(pack.nsats_GET() == (char)154);
            assert(pack.baseline_coords_type_GET() == (char)139);
            assert(pack.wn_GET() == (char)64132);
            assert(pack.rtk_receiver_id_GET() == (char)17);
            assert(pack.time_last_baseline_ms_GET() == 464880771L);
            assert(pack.rtk_rate_GET() == (char)228);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_b_mm_SET(2021936481) ;
        p128.tow_SET(174839106L) ;
        p128.baseline_a_mm_SET(52653500) ;
        p128.rtk_rate_SET((char)228) ;
        p128.baseline_coords_type_SET((char)139) ;
        p128.time_last_baseline_ms_SET(464880771L) ;
        p128.nsats_SET((char)154) ;
        p128.wn_SET((char)64132) ;
        p128.accuracy_SET(2091273456L) ;
        p128.rtk_receiver_id_SET((char)17) ;
        p128.baseline_c_mm_SET(1422004505) ;
        p128.rtk_health_SET((char)2) ;
        p128.iar_num_hypotheses_SET(1890492800) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == (short)5474);
            assert(pack.time_boot_ms_GET() == 4053898861L);
            assert(pack.ygyro_GET() == (short)28720);
            assert(pack.xacc_GET() == (short) -11114);
            assert(pack.zacc_GET() == (short)9353);
            assert(pack.xgyro_GET() == (short) -14915);
            assert(pack.zmag_GET() == (short) -670);
            assert(pack.yacc_GET() == (short)25620);
            assert(pack.ymag_GET() == (short) -2179);
            assert(pack.zgyro_GET() == (short) -29077);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.xgyro_SET((short) -14915) ;
        p129.ygyro_SET((short)28720) ;
        p129.yacc_SET((short)25620) ;
        p129.xacc_SET((short) -11114) ;
        p129.time_boot_ms_SET(4053898861L) ;
        p129.xmag_SET((short)5474) ;
        p129.zacc_SET((short)9353) ;
        p129.ymag_SET((short) -2179) ;
        p129.zgyro_SET((short) -29077) ;
        p129.zmag_SET((short) -670) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.size_GET() == 1386292371L);
            assert(pack.type_GET() == (char)145);
            assert(pack.width_GET() == (char)15401);
            assert(pack.payload_GET() == (char)55);
            assert(pack.height_GET() == (char)42079);
            assert(pack.jpg_quality_GET() == (char)247);
            assert(pack.packets_GET() == (char)24281);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.width_SET((char)15401) ;
        p130.type_SET((char)145) ;
        p130.packets_SET((char)24281) ;
        p130.height_SET((char)42079) ;
        p130.jpg_quality_SET((char)247) ;
        p130.size_SET(1386292371L) ;
        p130.payload_SET((char)55) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)7377);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)197, (char)226, (char)158, (char)95, (char)127, (char)106, (char)47, (char)239, (char)120, (char)82, (char)84, (char)244, (char)50, (char)11, (char)86, (char)72, (char)153, (char)10, (char)235, (char)26, (char)200, (char)216, (char)190, (char)73, (char)153, (char)5, (char)135, (char)17, (char)180, (char)207, (char)36, (char)159, (char)2, (char)207, (char)241, (char)172, (char)201, (char)8, (char)3, (char)129, (char)178, (char)38, (char)27, (char)120, (char)166, (char)117, (char)200, (char)30, (char)111, (char)245, (char)18, (char)238, (char)54, (char)235, (char)121, (char)36, (char)80, (char)80, (char)247, (char)166, (char)10, (char)106, (char)192, (char)180, (char)82, (char)31, (char)72, (char)180, (char)156, (char)227, (char)83, (char)94, (char)22, (char)33, (char)81, (char)195, (char)91, (char)9, (char)190, (char)94, (char)207, (char)196, (char)91, (char)193, (char)155, (char)236, (char)193, (char)167, (char)211, (char)191, (char)225, (char)5, (char)100, (char)19, (char)133, (char)253, (char)64, (char)242, (char)57, (char)97, (char)122, (char)134, (char)215, (char)238, (char)8, (char)248, (char)51, (char)106, (char)117, (char)80, (char)199, (char)13, (char)175, (char)20, (char)154, (char)215, (char)9, (char)136, (char)173, (char)196, (char)233, (char)184, (char)192, (char)176, (char)126, (char)125, (char)118, (char)195, (char)183, (char)104, (char)205, (char)161, (char)145, (char)23, (char)251, (char)250, (char)75, (char)51, (char)83, (char)139, (char)154, (char)26, (char)238, (char)171, (char)53, (char)24, (char)111, (char)122, (char)224, (char)41, (char)213, (char)74, (char)60, (char)72, (char)68, (char)41, (char)12, (char)64, (char)240, (char)48, (char)217, (char)8, (char)129, (char)3, (char)48, (char)87, (char)216, (char)114, (char)219, (char)155, (char)220, (char)200, (char)178, (char)199, (char)58, (char)139, (char)40, (char)162, (char)227, (char)19, (char)161, (char)212, (char)5, (char)223, (char)77, (char)102, (char)190, (char)80, (char)14, (char)78, (char)81, (char)71, (char)175, (char)137, (char)30, (char)99, (char)246, (char)104, (char)29, (char)241, (char)200, (char)135, (char)182, (char)67, (char)12, (char)103, (char)101, (char)125, (char)41, (char)41, (char)49, (char)13, (char)238, (char)169, (char)12, (char)176, (char)72, (char)126, (char)7, (char)124, (char)219, (char)228, (char)140, (char)205, (char)72, (char)75, (char)139, (char)119, (char)134, (char)140, (char)104, (char)15, (char)107, (char)139, (char)207, (char)152, (char)151, (char)110, (char)107, (char)89, (char)161, (char)98, (char)219, (char)204, (char)44, (char)102, (char)138, (char)236, (char)238, (char)20, (char)76, (char)206, (char)126}));
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)7377) ;
        p131.data__SET(new char[] {(char)197, (char)226, (char)158, (char)95, (char)127, (char)106, (char)47, (char)239, (char)120, (char)82, (char)84, (char)244, (char)50, (char)11, (char)86, (char)72, (char)153, (char)10, (char)235, (char)26, (char)200, (char)216, (char)190, (char)73, (char)153, (char)5, (char)135, (char)17, (char)180, (char)207, (char)36, (char)159, (char)2, (char)207, (char)241, (char)172, (char)201, (char)8, (char)3, (char)129, (char)178, (char)38, (char)27, (char)120, (char)166, (char)117, (char)200, (char)30, (char)111, (char)245, (char)18, (char)238, (char)54, (char)235, (char)121, (char)36, (char)80, (char)80, (char)247, (char)166, (char)10, (char)106, (char)192, (char)180, (char)82, (char)31, (char)72, (char)180, (char)156, (char)227, (char)83, (char)94, (char)22, (char)33, (char)81, (char)195, (char)91, (char)9, (char)190, (char)94, (char)207, (char)196, (char)91, (char)193, (char)155, (char)236, (char)193, (char)167, (char)211, (char)191, (char)225, (char)5, (char)100, (char)19, (char)133, (char)253, (char)64, (char)242, (char)57, (char)97, (char)122, (char)134, (char)215, (char)238, (char)8, (char)248, (char)51, (char)106, (char)117, (char)80, (char)199, (char)13, (char)175, (char)20, (char)154, (char)215, (char)9, (char)136, (char)173, (char)196, (char)233, (char)184, (char)192, (char)176, (char)126, (char)125, (char)118, (char)195, (char)183, (char)104, (char)205, (char)161, (char)145, (char)23, (char)251, (char)250, (char)75, (char)51, (char)83, (char)139, (char)154, (char)26, (char)238, (char)171, (char)53, (char)24, (char)111, (char)122, (char)224, (char)41, (char)213, (char)74, (char)60, (char)72, (char)68, (char)41, (char)12, (char)64, (char)240, (char)48, (char)217, (char)8, (char)129, (char)3, (char)48, (char)87, (char)216, (char)114, (char)219, (char)155, (char)220, (char)200, (char)178, (char)199, (char)58, (char)139, (char)40, (char)162, (char)227, (char)19, (char)161, (char)212, (char)5, (char)223, (char)77, (char)102, (char)190, (char)80, (char)14, (char)78, (char)81, (char)71, (char)175, (char)137, (char)30, (char)99, (char)246, (char)104, (char)29, (char)241, (char)200, (char)135, (char)182, (char)67, (char)12, (char)103, (char)101, (char)125, (char)41, (char)41, (char)49, (char)13, (char)238, (char)169, (char)12, (char)176, (char)72, (char)126, (char)7, (char)124, (char)219, (char)228, (char)140, (char)205, (char)72, (char)75, (char)139, (char)119, (char)134, (char)140, (char)104, (char)15, (char)107, (char)139, (char)207, (char)152, (char)151, (char)110, (char)107, (char)89, (char)161, (char)98, (char)219, (char)204, (char)44, (char)102, (char)138, (char)236, (char)238, (char)20, (char)76, (char)206, (char)126}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2162538630L);
            assert(pack.covariance_GET() == (char)56);
            assert(pack.max_distance_GET() == (char)38940);
            assert(pack.current_distance_GET() == (char)12791);
            assert(pack.id_GET() == (char)0);
            assert(pack.min_distance_GET() == (char)3910);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_90);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.max_distance_SET((char)38940) ;
        p132.current_distance_SET((char)12791) ;
        p132.covariance_SET((char)56) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_90) ;
        p132.min_distance_SET((char)3910) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p132.time_boot_ms_SET(2162538630L) ;
        p132.id_SET((char)0) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 608233542);
            assert(pack.lon_GET() == -1458235880);
            assert(pack.grid_spacing_GET() == (char)17123);
            assert(pack.mask_GET() == 5838617926816985366L);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.grid_spacing_SET((char)17123) ;
        p133.mask_SET(5838617926816985366L) ;
        p133.lon_SET(-1458235880) ;
        p133.lat_SET(608233542) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1847271143);
            assert(pack.gridbit_GET() == (char)210);
            assert(pack.grid_spacing_GET() == (char)55432);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -25277, (short) -27909, (short)14238, (short) -10285, (short)31694, (short) -19790, (short) -27684, (short)22123, (short) -4915, (short)27139, (short) -20362, (short) -15947, (short) -20153, (short) -23474, (short)19551, (short)19484}));
            assert(pack.lon_GET() == -139265758);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lon_SET(-139265758) ;
        p134.lat_SET(1847271143) ;
        p134.gridbit_SET((char)210) ;
        p134.data__SET(new short[] {(short) -25277, (short) -27909, (short)14238, (short) -10285, (short)31694, (short) -19790, (short) -27684, (short)22123, (short) -4915, (short)27139, (short) -20362, (short) -15947, (short) -20153, (short) -23474, (short)19551, (short)19484}, 0) ;
        p134.grid_spacing_SET((char)55432) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1821837410);
            assert(pack.lon_GET() == 258796369);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(1821837410) ;
        p135.lon_SET(258796369) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -2086076854);
            assert(pack.terrain_height_GET() == -1.6724265E38F);
            assert(pack.spacing_GET() == (char)27714);
            assert(pack.current_height_GET() == 3.892765E37F);
            assert(pack.loaded_GET() == (char)39228);
            assert(pack.lat_GET() == 1190547869);
            assert(pack.pending_GET() == (char)53635);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.current_height_SET(3.892765E37F) ;
        p136.loaded_SET((char)39228) ;
        p136.spacing_SET((char)27714) ;
        p136.terrain_height_SET(-1.6724265E38F) ;
        p136.pending_SET((char)53635) ;
        p136.lon_SET(-2086076854) ;
        p136.lat_SET(1190547869) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == -2.2949155E38F);
            assert(pack.time_boot_ms_GET() == 1255262732L);
            assert(pack.temperature_GET() == (short) -9748);
            assert(pack.press_abs_GET() == 4.111363E37F);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_diff_SET(-2.2949155E38F) ;
        p137.temperature_SET((short) -9748) ;
        p137.time_boot_ms_SET(1255262732L) ;
        p137.press_abs_SET(4.111363E37F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.3369442E37F, -1.9674122E38F, 2.5765278E38F, 2.3811861E38F}));
            assert(pack.y_GET() == 1.517087E38F);
            assert(pack.x_GET() == -8.4809356E36F);
            assert(pack.z_GET() == -2.3799351E38F);
            assert(pack.time_usec_GET() == 5405431168661024694L);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.y_SET(1.517087E38F) ;
        p138.x_SET(-8.4809356E36F) ;
        p138.q_SET(new float[] {3.3369442E37F, -1.9674122E38F, 2.5765278E38F, 2.3811861E38F}, 0) ;
        p138.time_usec_SET(5405431168661024694L) ;
        p138.z_SET(-2.3799351E38F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)65);
            assert(pack.target_component_GET() == (char)50);
            assert(pack.group_mlx_GET() == (char)90);
            assert(pack.time_usec_GET() == 2424739528631436400L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.3821865E38F, -4.845828E37F, 9.673184E37F, -2.9409291E38F, -3.2607326E38F, -3.241698E38F, 3.3476277E38F, -1.3472419E38F}));
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.target_component_SET((char)50) ;
        p139.target_system_SET((char)65) ;
        p139.controls_SET(new float[] {-2.3821865E38F, -4.845828E37F, 9.673184E37F, -2.9409291E38F, -3.2607326E38F, -3.241698E38F, 3.3476277E38F, -1.3472419E38F}, 0) ;
        p139.group_mlx_SET((char)90) ;
        p139.time_usec_SET(2424739528631436400L) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.3635509E38F, 2.4060329E38F, -2.6882758E38F, -2.4621649E38F, -2.8900629E38F, -1.3725265E38F, -2.674424E37F, -1.848034E38F}));
            assert(pack.time_usec_GET() == 6467758444934910870L);
            assert(pack.group_mlx_GET() == (char)65);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)65) ;
        p140.time_usec_SET(6467758444934910870L) ;
        p140.controls_SET(new float[] {-1.3635509E38F, 2.4060329E38F, -2.6882758E38F, -2.4621649E38F, -2.8900629E38F, -1.3725265E38F, -2.674424E37F, -1.848034E38F}, 0) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_terrain_GET() == 1.0900809E38F);
            assert(pack.altitude_relative_GET() == 7.889674E37F);
            assert(pack.time_usec_GET() == 56804639088700647L);
            assert(pack.altitude_amsl_GET() == 2.7237E38F);
            assert(pack.altitude_local_GET() == -1.5244163E38F);
            assert(pack.bottom_clearance_GET() == -1.5018833E38F);
            assert(pack.altitude_monotonic_GET() == 3.282762E38F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_terrain_SET(1.0900809E38F) ;
        p141.altitude_monotonic_SET(3.282762E38F) ;
        p141.altitude_local_SET(-1.5244163E38F) ;
        p141.time_usec_SET(56804639088700647L) ;
        p141.altitude_amsl_SET(2.7237E38F) ;
        p141.altitude_relative_SET(7.889674E37F) ;
        p141.bottom_clearance_SET(-1.5018833E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.uri_type_GET() == (char)90);
            assert(pack.request_id_GET() == (char)123);
            assert(pack.transfer_type_GET() == (char)100);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)119, (char)119, (char)221, (char)60, (char)214, (char)240, (char)112, (char)142, (char)37, (char)178, (char)99, (char)179, (char)124, (char)234, (char)108, (char)141, (char)219, (char)156, (char)155, (char)3, (char)108, (char)50, (char)87, (char)228, (char)199, (char)175, (char)196, (char)45, (char)212, (char)156, (char)181, (char)94, (char)237, (char)76, (char)1, (char)250, (char)181, (char)202, (char)205, (char)117, (char)51, (char)62, (char)111, (char)49, (char)73, (char)170, (char)129, (char)117, (char)210, (char)237, (char)77, (char)33, (char)17, (char)45, (char)179, (char)102, (char)198, (char)47, (char)79, (char)113, (char)241, (char)160, (char)54, (char)139, (char)226, (char)162, (char)33, (char)21, (char)106, (char)200, (char)35, (char)72, (char)70, (char)166, (char)219, (char)224, (char)210, (char)150, (char)255, (char)159, (char)158, (char)23, (char)103, (char)22, (char)189, (char)169, (char)239, (char)235, (char)247, (char)39, (char)146, (char)73, (char)44, (char)156, (char)32, (char)128, (char)16, (char)97, (char)208, (char)218, (char)92, (char)227, (char)222, (char)114, (char)230, (char)221, (char)221, (char)142, (char)79, (char)174, (char)122, (char)242, (char)247, (char)58, (char)112, (char)155, (char)183, (char)72, (char)141, (char)68}));
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)25, (char)28, (char)228, (char)89, (char)194, (char)74, (char)45, (char)112, (char)224, (char)67, (char)68, (char)173, (char)184, (char)88, (char)126, (char)7, (char)178, (char)226, (char)106, (char)90, (char)139, (char)162, (char)251, (char)134, (char)198, (char)13, (char)23, (char)56, (char)189, (char)99, (char)221, (char)204, (char)127, (char)15, (char)74, (char)154, (char)230, (char)149, (char)119, (char)119, (char)64, (char)111, (char)9, (char)250, (char)26, (char)77, (char)47, (char)23, (char)120, (char)58, (char)164, (char)48, (char)142, (char)171, (char)147, (char)76, (char)86, (char)106, (char)100, (char)3, (char)172, (char)134, (char)61, (char)41, (char)203, (char)2, (char)55, (char)173, (char)7, (char)92, (char)150, (char)172, (char)202, (char)192, (char)104, (char)77, (char)44, (char)3, (char)239, (char)143, (char)54, (char)197, (char)212, (char)9, (char)157, (char)65, (char)168, (char)233, (char)187, (char)243, (char)234, (char)245, (char)54, (char)70, (char)64, (char)56, (char)10, (char)161, (char)118, (char)226, (char)254, (char)1, (char)34, (char)245, (char)139, (char)74, (char)160, (char)2, (char)154, (char)53, (char)233, (char)229, (char)14, (char)194, (char)26, (char)4, (char)98, (char)68, (char)80, (char)246}));
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.transfer_type_SET((char)100) ;
        p142.uri_SET(new char[] {(char)25, (char)28, (char)228, (char)89, (char)194, (char)74, (char)45, (char)112, (char)224, (char)67, (char)68, (char)173, (char)184, (char)88, (char)126, (char)7, (char)178, (char)226, (char)106, (char)90, (char)139, (char)162, (char)251, (char)134, (char)198, (char)13, (char)23, (char)56, (char)189, (char)99, (char)221, (char)204, (char)127, (char)15, (char)74, (char)154, (char)230, (char)149, (char)119, (char)119, (char)64, (char)111, (char)9, (char)250, (char)26, (char)77, (char)47, (char)23, (char)120, (char)58, (char)164, (char)48, (char)142, (char)171, (char)147, (char)76, (char)86, (char)106, (char)100, (char)3, (char)172, (char)134, (char)61, (char)41, (char)203, (char)2, (char)55, (char)173, (char)7, (char)92, (char)150, (char)172, (char)202, (char)192, (char)104, (char)77, (char)44, (char)3, (char)239, (char)143, (char)54, (char)197, (char)212, (char)9, (char)157, (char)65, (char)168, (char)233, (char)187, (char)243, (char)234, (char)245, (char)54, (char)70, (char)64, (char)56, (char)10, (char)161, (char)118, (char)226, (char)254, (char)1, (char)34, (char)245, (char)139, (char)74, (char)160, (char)2, (char)154, (char)53, (char)233, (char)229, (char)14, (char)194, (char)26, (char)4, (char)98, (char)68, (char)80, (char)246}, 0) ;
        p142.storage_SET(new char[] {(char)119, (char)119, (char)221, (char)60, (char)214, (char)240, (char)112, (char)142, (char)37, (char)178, (char)99, (char)179, (char)124, (char)234, (char)108, (char)141, (char)219, (char)156, (char)155, (char)3, (char)108, (char)50, (char)87, (char)228, (char)199, (char)175, (char)196, (char)45, (char)212, (char)156, (char)181, (char)94, (char)237, (char)76, (char)1, (char)250, (char)181, (char)202, (char)205, (char)117, (char)51, (char)62, (char)111, (char)49, (char)73, (char)170, (char)129, (char)117, (char)210, (char)237, (char)77, (char)33, (char)17, (char)45, (char)179, (char)102, (char)198, (char)47, (char)79, (char)113, (char)241, (char)160, (char)54, (char)139, (char)226, (char)162, (char)33, (char)21, (char)106, (char)200, (char)35, (char)72, (char)70, (char)166, (char)219, (char)224, (char)210, (char)150, (char)255, (char)159, (char)158, (char)23, (char)103, (char)22, (char)189, (char)169, (char)239, (char)235, (char)247, (char)39, (char)146, (char)73, (char)44, (char)156, (char)32, (char)128, (char)16, (char)97, (char)208, (char)218, (char)92, (char)227, (char)222, (char)114, (char)230, (char)221, (char)221, (char)142, (char)79, (char)174, (char)122, (char)242, (char)247, (char)58, (char)112, (char)155, (char)183, (char)72, (char)141, (char)68}, 0) ;
        p142.uri_type_SET((char)90) ;
        p142.request_id_SET((char)123) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -3.2301048E38F);
            assert(pack.time_boot_ms_GET() == 2331634568L);
            assert(pack.press_diff_GET() == -3.0111588E38F);
            assert(pack.temperature_GET() == (short) -31086);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(2331634568L) ;
        p143.press_diff_SET(-3.0111588E38F) ;
        p143.press_abs_SET(-3.2301048E38F) ;
        p143.temperature_SET((short) -31086) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.custom_state_GET() == 4860627155637563120L);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {1.9287028E38F, 2.7375168E38F, -9.905615E37F}));
            assert(pack.alt_GET() == 2.7321783E38F);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-2.274313E38F, -2.2885671E38F, 2.648355E38F}));
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {9.964275E37F, 3.0332394E38F, 1.5357546E38F}));
            assert(pack.timestamp_GET() == 3954796471349821684L);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {2.0198921E38F, -2.4991746E38F, -8.904184E37F}));
            assert(pack.lat_GET() == 1641971952);
            assert(pack.est_capabilities_GET() == (char)114);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-2.3855357E38F, 1.8398782E38F, 2.8199508E38F, -3.2610519E38F}));
            assert(pack.lon_GET() == 1119570828);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.alt_SET(2.7321783E38F) ;
        p144.attitude_q_SET(new float[] {-2.3855357E38F, 1.8398782E38F, 2.8199508E38F, -3.2610519E38F}, 0) ;
        p144.position_cov_SET(new float[] {9.964275E37F, 3.0332394E38F, 1.5357546E38F}, 0) ;
        p144.timestamp_SET(3954796471349821684L) ;
        p144.lat_SET(1641971952) ;
        p144.lon_SET(1119570828) ;
        p144.vel_SET(new float[] {1.9287028E38F, 2.7375168E38F, -9.905615E37F}, 0) ;
        p144.acc_SET(new float[] {2.0198921E38F, -2.4991746E38F, -8.904184E37F}, 0) ;
        p144.custom_state_SET(4860627155637563120L) ;
        p144.est_capabilities_SET((char)114) ;
        p144.rates_SET(new float[] {-2.274313E38F, -2.2885671E38F, 2.648355E38F}, 0) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == 2.0832716E38F);
            assert(pack.pitch_rate_GET() == -3.1077E38F);
            assert(pack.roll_rate_GET() == -2.992232E38F);
            assert(pack.time_usec_GET() == 1827608146328038562L);
            assert(pack.x_pos_GET() == 3.3827575E38F);
            assert(pack.airspeed_GET() == 1.6750215E38F);
            assert(pack.x_acc_GET() == 2.5819305E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-3.4022845E37F, 3.0357989E38F, -2.7201571E38F}));
            assert(pack.y_pos_GET() == -3.1751881E38F);
            assert(pack.y_acc_GET() == 2.1432213E38F);
            assert(pack.z_pos_GET() == 2.8775721E38F);
            assert(pack.y_vel_GET() == 3.0625988E38F);
            assert(pack.z_vel_GET() == -2.2753584E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {-1.1442277E38F, -8.4617594E37F, -8.87068E37F}));
            assert(pack.x_vel_GET() == 3.1463745E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.4606007E38F, -2.4130707E38F, -2.8974242E38F, 1.9433284E38F}));
            assert(pack.z_acc_GET() == -2.3681352E38F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.vel_variance_SET(new float[] {-3.4022845E37F, 3.0357989E38F, -2.7201571E38F}, 0) ;
        p146.z_vel_SET(-2.2753584E38F) ;
        p146.y_pos_SET(-3.1751881E38F) ;
        p146.pos_variance_SET(new float[] {-1.1442277E38F, -8.4617594E37F, -8.87068E37F}, 0) ;
        p146.z_pos_SET(2.8775721E38F) ;
        p146.roll_rate_SET(-2.992232E38F) ;
        p146.pitch_rate_SET(-3.1077E38F) ;
        p146.airspeed_SET(1.6750215E38F) ;
        p146.y_vel_SET(3.0625988E38F) ;
        p146.x_vel_SET(3.1463745E38F) ;
        p146.x_pos_SET(3.3827575E38F) ;
        p146.z_acc_SET(-2.3681352E38F) ;
        p146.time_usec_SET(1827608146328038562L) ;
        p146.x_acc_SET(2.5819305E38F) ;
        p146.q_SET(new float[] {2.4606007E38F, -2.4130707E38F, -2.8974242E38F, 1.9433284E38F}, 0) ;
        p146.yaw_rate_SET(2.0832716E38F) ;
        p146.y_acc_SET(2.1432213E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)192);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
            assert(pack.temperature_GET() == (short)30590);
            assert(pack.battery_remaining_GET() == (byte)51);
            assert(pack.current_consumed_GET() == -186432095);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)52288, (char)60380, (char)26817, (char)3216, (char)26014, (char)51976, (char)30501, (char)9975, (char)1679, (char)58051}));
            assert(pack.current_battery_GET() == (short) -4086);
            assert(pack.energy_consumed_GET() == 1159067561);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)192) ;
        p147.temperature_SET((short)30590) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS) ;
        p147.current_consumed_SET(-186432095) ;
        p147.battery_remaining_SET((byte)51) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO) ;
        p147.energy_consumed_SET(1159067561) ;
        p147.current_battery_SET((short) -4086) ;
        p147.voltages_SET(new char[] {(char)52288, (char)60380, (char)26817, (char)3216, (char)26014, (char)51976, (char)30501, (char)9975, (char)1679, (char)58051}, 0) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)243, (char)70, (char)188, (char)56, (char)149, (char)103, (char)180, (char)204}));
            assert(pack.vendor_id_GET() == (char)59588);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)252, (char)115, (char)122, (char)203, (char)78, (char)199, (char)159, (char)143, (char)68, (char)80, (char)166, (char)17, (char)173, (char)74, (char)82, (char)250, (char)29, (char)212}));
            assert(pack.os_sw_version_GET() == 1113490361L);
            assert(pack.flight_sw_version_GET() == 2211412649L);
            assert(pack.board_version_GET() == 3260984774L);
            assert(pack.uid_GET() == 5925820199597615642L);
            assert(pack.product_id_GET() == (char)57766);
            assert(pack.middleware_sw_version_GET() == 2134247512L);
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT));
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)177, (char)43, (char)113, (char)67, (char)171, (char)162, (char)54, (char)35}));
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)112, (char)235, (char)37, (char)58, (char)217, (char)90, (char)63, (char)80}));
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.middleware_custom_version_SET(new char[] {(char)177, (char)43, (char)113, (char)67, (char)171, (char)162, (char)54, (char)35}, 0) ;
        p148.os_sw_version_SET(1113490361L) ;
        p148.uid_SET(5925820199597615642L) ;
        p148.uid2_SET(new char[] {(char)252, (char)115, (char)122, (char)203, (char)78, (char)199, (char)159, (char)143, (char)68, (char)80, (char)166, (char)17, (char)173, (char)74, (char)82, (char)250, (char)29, (char)212}, 0, PH) ;
        p148.board_version_SET(3260984774L) ;
        p148.middleware_sw_version_SET(2134247512L) ;
        p148.flight_custom_version_SET(new char[] {(char)243, (char)70, (char)188, (char)56, (char)149, (char)103, (char)180, (char)204}, 0) ;
        p148.vendor_id_SET((char)59588) ;
        p148.flight_sw_version_SET(2211412649L) ;
        p148.product_id_SET((char)57766) ;
        p148.os_custom_version_SET(new char[] {(char)112, (char)235, (char)37, (char)58, (char)217, (char)90, (char)63, (char)80}, 0) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT)) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.x_TRY(ph) == -2.6935271E38F);
            assert(pack.size_y_GET() == 1.6505343E37F);
            assert(pack.y_TRY(ph) == 2.8526578E37F);
            assert(pack.size_x_GET() == -2.5929801E38F);
            assert(pack.z_TRY(ph) == 1.2252844E38F);
            assert(pack.distance_GET() == 1.5745908E38F);
            assert(pack.target_num_GET() == (char)213);
            assert(pack.angle_x_GET() == 3.4010041E38F);
            assert(pack.position_valid_TRY(ph) == (char)198);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {1.4336772E37F, -1.0066689E38F, 1.4232976E37F, -2.2352745E38F}));
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
            assert(pack.angle_y_GET() == 1.4510885E37F);
            assert(pack.time_usec_GET() == 8879486765325297412L);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.angle_y_SET(1.4510885E37F) ;
        p149.position_valid_SET((char)198, PH) ;
        p149.distance_SET(1.5745908E38F) ;
        p149.size_y_SET(1.6505343E37F) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON) ;
        p149.z_SET(1.2252844E38F, PH) ;
        p149.angle_x_SET(3.4010041E38F) ;
        p149.y_SET(2.8526578E37F, PH) ;
        p149.time_usec_SET(8879486765325297412L) ;
        p149.x_SET(-2.6935271E38F, PH) ;
        p149.size_x_SET(-2.5929801E38F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p149.target_num_SET((char)213) ;
        p149.q_SET(new float[] {1.4336772E37F, -1.0066689E38F, 1.4232976E37F, -2.2352745E38F}, 0, PH) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_CPU_LOAD.add((src, ph, pack) ->
        {
            assert(pack.ctrlLoad_GET() == (char)91);
            assert(pack.batVolt_GET() == (char)58833);
            assert(pack.sensLoad_GET() == (char)160);
        });
        GroundControl.CPU_LOAD p170 = CommunicationChannel.new_CPU_LOAD();
        PH.setPack(p170);
        p170.batVolt_SET((char)58833) ;
        p170.ctrlLoad_SET((char)91) ;
        p170.sensLoad_SET((char)160) ;
        CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SENSOR_BIAS.add((src, ph, pack) ->
        {
            assert(pack.gxBias_GET() == -1.6159986E38F);
            assert(pack.axBias_GET() == -2.0952646E37F);
            assert(pack.gzBias_GET() == -1.674551E38F);
            assert(pack.ayBias_GET() == 2.398847E38F);
            assert(pack.gyBias_GET() == 2.7397763E38F);
            assert(pack.azBias_GET() == -2.4325807E38F);
        });
        GroundControl.SENSOR_BIAS p172 = CommunicationChannel.new_SENSOR_BIAS();
        PH.setPack(p172);
        p172.gzBias_SET(-1.674551E38F) ;
        p172.ayBias_SET(2.398847E38F) ;
        p172.azBias_SET(-2.4325807E38F) ;
        p172.axBias_SET(-2.0952646E37F) ;
        p172.gxBias_SET(-1.6159986E38F) ;
        p172.gyBias_SET(2.7397763E38F) ;
        CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DIAGNOSTIC.add((src, ph, pack) ->
        {
            assert(pack.diagFl1_GET() == 3.2784343E38F);
            assert(pack.diagFl3_GET() == -6.2469365E37F);
            assert(pack.diagSh3_GET() == (short) -6794);
            assert(pack.diagSh1_GET() == (short)8599);
            assert(pack.diagSh2_GET() == (short)8826);
            assert(pack.diagFl2_GET() == 8.616891E37F);
        });
        GroundControl.DIAGNOSTIC p173 = CommunicationChannel.new_DIAGNOSTIC();
        PH.setPack(p173);
        p173.diagSh3_SET((short) -6794) ;
        p173.diagSh1_SET((short)8599) ;
        p173.diagFl3_SET(-6.2469365E37F) ;
        p173.diagSh2_SET((short)8826) ;
        p173.diagFl2_SET(8.616891E37F) ;
        p173.diagFl1_SET(3.2784343E38F) ;
        CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SLUGS_NAVIGATION.add((src, ph, pack) ->
        {
            assert(pack.fromWP_GET() == (char)213);
            assert(pack.totalDist_GET() == 4.8035175E37F);
            assert(pack.psiDot_c_GET() == -2.1385106E37F);
            assert(pack.u_m_GET() == 1.8307163E38F);
            assert(pack.phi_c_GET() == 1.2945739E38F);
            assert(pack.dist2Go_GET() == -1.5865679E37F);
            assert(pack.h_c_GET() == (char)42578);
            assert(pack.theta_c_GET() == 2.2955745E38F);
            assert(pack.ay_body_GET() == -2.2786923E36F);
            assert(pack.toWP_GET() == (char)249);
        });
        GroundControl.SLUGS_NAVIGATION p176 = CommunicationChannel.new_SLUGS_NAVIGATION();
        PH.setPack(p176);
        p176.dist2Go_SET(-1.5865679E37F) ;
        p176.toWP_SET((char)249) ;
        p176.ay_body_SET(-2.2786923E36F) ;
        p176.fromWP_SET((char)213) ;
        p176.psiDot_c_SET(-2.1385106E37F) ;
        p176.h_c_SET((char)42578) ;
        p176.phi_c_SET(1.2945739E38F) ;
        p176.theta_c_SET(2.2955745E38F) ;
        p176.totalDist_SET(4.8035175E37F) ;
        p176.u_m_SET(1.8307163E38F) ;
        CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DATA_LOG.add((src, ph, pack) ->
        {
            assert(pack.fl_1_GET() == 1.2812209E38F);
            assert(pack.fl_2_GET() == 2.1031208E38F);
            assert(pack.fl_3_GET() == 9.0379926E36F);
            assert(pack.fl_5_GET() == 1.4386997E38F);
            assert(pack.fl_6_GET() == 3.1824285E38F);
            assert(pack.fl_4_GET() == 2.115598E38F);
        });
        GroundControl.DATA_LOG p177 = CommunicationChannel.new_DATA_LOG();
        PH.setPack(p177);
        p177.fl_6_SET(3.1824285E38F) ;
        p177.fl_1_SET(1.2812209E38F) ;
        p177.fl_5_SET(1.4386997E38F) ;
        p177.fl_4_SET(2.115598E38F) ;
        p177.fl_2_SET(2.1031208E38F) ;
        p177.fl_3_SET(9.0379926E36F) ;
        CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_DATE_TIME.add((src, ph, pack) ->
        {
            assert(pack.clockStat_GET() == (char)187);
            assert(pack.min_GET() == (char)189);
            assert(pack.day_GET() == (char)243);
            assert(pack.percentUsed_GET() == (char)21);
            assert(pack.useSat_GET() == (char)9);
            assert(pack.year_GET() == (char)47);
            assert(pack.GppGl_GET() == (char)115);
            assert(pack.sigUsedMask_GET() == (char)205);
            assert(pack.visSat_GET() == (char)170);
            assert(pack.sec_GET() == (char)60);
            assert(pack.hour_GET() == (char)127);
            assert(pack.month_GET() == (char)5);
        });
        GroundControl.GPS_DATE_TIME p179 = CommunicationChannel.new_GPS_DATE_TIME();
        PH.setPack(p179);
        p179.percentUsed_SET((char)21) ;
        p179.hour_SET((char)127) ;
        p179.day_SET((char)243) ;
        p179.min_SET((char)189) ;
        p179.visSat_SET((char)170) ;
        p179.GppGl_SET((char)115) ;
        p179.sec_SET((char)60) ;
        p179.year_SET((char)47) ;
        p179.month_SET((char)5) ;
        p179.sigUsedMask_SET((char)205) ;
        p179.clockStat_SET((char)187) ;
        p179.useSat_SET((char)9) ;
        CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MID_LVL_CMDS.add((src, ph, pack) ->
        {
            assert(pack.target_GET() == (char)50);
            assert(pack.hCommand_GET() == 2.407729E38F);
            assert(pack.uCommand_GET() == 1.5785112E38F);
            assert(pack.rCommand_GET() == -3.2471111E38F);
        });
        GroundControl.MID_LVL_CMDS p180 = CommunicationChannel.new_MID_LVL_CMDS();
        PH.setPack(p180);
        p180.uCommand_SET(1.5785112E38F) ;
        p180.rCommand_SET(-3.2471111E38F) ;
        p180.hCommand_SET(2.407729E38F) ;
        p180.target_SET((char)50) ;
        CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CTRL_SRFC_PT.add((src, ph, pack) ->
        {
            assert(pack.bitfieldPt_GET() == (char)18809);
            assert(pack.target_GET() == (char)71);
        });
        GroundControl.CTRL_SRFC_PT p181 = CommunicationChannel.new_CTRL_SRFC_PT();
        PH.setPack(p181);
        p181.bitfieldPt_SET((char)18809) ;
        p181.target_SET((char)71) ;
        CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SLUGS_CAMERA_ORDER.add((src, ph, pack) ->
        {
            assert(pack.pan_GET() == (byte)59);
            assert(pack.moveHome_GET() == (byte) - 24);
            assert(pack.target_GET() == (char)249);
            assert(pack.tilt_GET() == (byte) - 34);
            assert(pack.zoom_GET() == (byte) - 49);
        });
        GroundControl.SLUGS_CAMERA_ORDER p184 = CommunicationChannel.new_SLUGS_CAMERA_ORDER();
        PH.setPack(p184);
        p184.target_SET((char)249) ;
        p184.zoom_SET((byte) - 49) ;
        p184.tilt_SET((byte) - 34) ;
        p184.pan_SET((byte)59) ;
        p184.moveHome_SET((byte) - 24) ;
        CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SURFACE.add((src, ph, pack) ->
        {
            assert(pack.target_GET() == (char)39);
            assert(pack.idSurface_GET() == (char)136);
            assert(pack.mControl_GET() == 1.5837986E38F);
            assert(pack.bControl_GET() == 2.5356432E37F);
        });
        GroundControl.CONTROL_SURFACE p185 = CommunicationChannel.new_CONTROL_SURFACE();
        PH.setPack(p185);
        p185.idSurface_SET((char)136) ;
        p185.bControl_SET(2.5356432E37F) ;
        p185.mControl_SET(1.5837986E38F) ;
        p185.target_SET((char)39) ;
        CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SLUGS_MOBILE_LOCATION.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 1.1241602E38F);
            assert(pack.target_GET() == (char)182);
            assert(pack.longitude_GET() == -3.136991E38F);
        });
        GroundControl.SLUGS_MOBILE_LOCATION p186 = CommunicationChannel.new_SLUGS_MOBILE_LOCATION();
        PH.setPack(p186);
        p186.latitude_SET(1.1241602E38F) ;
        p186.target_SET((char)182) ;
        p186.longitude_SET(-3.136991E38F) ;
        CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SLUGS_CONFIGURATION_CAMERA.add((src, ph, pack) ->
        {
            assert(pack.idOrder_GET() == (char)226);
            assert(pack.order_GET() == (char)223);
            assert(pack.target_GET() == (char)80);
        });
        GroundControl.SLUGS_CONFIGURATION_CAMERA p188 = CommunicationChannel.new_SLUGS_CONFIGURATION_CAMERA();
        PH.setPack(p188);
        p188.order_SET((char)223) ;
        p188.target_SET((char)80) ;
        p188.idOrder_SET((char)226) ;
        CommunicationChannel.instance.send(p188);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ISR_LOCATION.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == -1.7458443E38F);
            assert(pack.option2_GET() == (char)36);
            assert(pack.option3_GET() == (char)34);
            assert(pack.longitude_GET() == -1.0689502E38F);
            assert(pack.option1_GET() == (char)35);
            assert(pack.target_GET() == (char)220);
            assert(pack.height_GET() == -3.393048E38F);
        });
        GroundControl.ISR_LOCATION p189 = CommunicationChannel.new_ISR_LOCATION();
        PH.setPack(p189);
        p189.height_SET(-3.393048E38F) ;
        p189.target_SET((char)220) ;
        p189.option3_SET((char)34) ;
        p189.option2_SET((char)36) ;
        p189.longitude_SET(-1.0689502E38F) ;
        p189.option1_SET((char)35) ;
        p189.latitude_SET(-1.7458443E38F) ;
        CommunicationChannel.instance.send(p189);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VOLT_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.r2Type_GET() == (char)29);
            assert(pack.voltage_GET() == (char)56778);
            assert(pack.reading2_GET() == (char)4494);
        });
        GroundControl.VOLT_SENSOR p191 = CommunicationChannel.new_VOLT_SENSOR();
        PH.setPack(p191);
        p191.reading2_SET((char)4494) ;
        p191.voltage_SET((char)56778) ;
        p191.r2Type_SET((char)29) ;
        CommunicationChannel.instance.send(p191);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PTZ_STATUS.add((src, ph, pack) ->
        {
            assert(pack.zoom_GET() == (char)40);
            assert(pack.pan_GET() == (short) -784);
            assert(pack.tilt_GET() == (short) -12053);
        });
        GroundControl.PTZ_STATUS p192 = CommunicationChannel.new_PTZ_STATUS();
        PH.setPack(p192);
        p192.pan_SET((short) -784) ;
        p192.zoom_SET((char)40) ;
        p192.tilt_SET((short) -12053) ;
        CommunicationChannel.instance.send(p192);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAV_STATUS.add((src, ph, pack) ->
        {
            assert(pack.course_GET() == 1.0235544E38F);
            assert(pack.altitude_GET() == -1.2266169E38F);
            assert(pack.target_GET() == (char)162);
            assert(pack.latitude_GET() == 1.6858604E38F);
            assert(pack.speed_GET() == 2.1711129E38F);
            assert(pack.longitude_GET() == -3.022509E38F);
        });
        GroundControl.UAV_STATUS p193 = CommunicationChannel.new_UAV_STATUS();
        PH.setPack(p193);
        p193.latitude_SET(1.6858604E38F) ;
        p193.altitude_SET(-1.2266169E38F) ;
        p193.speed_SET(2.1711129E38F) ;
        p193.target_SET((char)162) ;
        p193.longitude_SET(-3.022509E38F) ;
        p193.course_SET(1.0235544E38F) ;
        CommunicationChannel.instance.send(p193);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUS_GPS.add((src, ph, pack) ->
        {
            assert(pack.posStatus_GET() == (char)207);
            assert(pack.modeInd_GET() == (char)155);
            assert(pack.magVar_GET() == -2.6133217E38F);
            assert(pack.csFails_GET() == (char)13559);
            assert(pack.gpsQuality_GET() == (char)136);
            assert(pack.magDir_GET() == (byte) - 69);
            assert(pack.msgsType_GET() == (char)115);
        });
        GroundControl.STATUS_GPS p194 = CommunicationChannel.new_STATUS_GPS();
        PH.setPack(p194);
        p194.csFails_SET((char)13559) ;
        p194.modeInd_SET((char)155) ;
        p194.msgsType_SET((char)115) ;
        p194.gpsQuality_SET((char)136) ;
        p194.magVar_SET(-2.6133217E38F) ;
        p194.posStatus_SET((char)207) ;
        p194.magDir_SET((byte) - 69) ;
        CommunicationChannel.instance.send(p194);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NOVATEL_DIAG.add((src, ph, pack) ->
        {
            assert(pack.receiverStatus_GET() == 363716375L);
            assert(pack.velType_GET() == (char)242);
            assert(pack.solStatus_GET() == (char)91);
            assert(pack.posSolAge_GET() == -2.7378375E38F);
            assert(pack.posType_GET() == (char)186);
            assert(pack.timeStatus_GET() == (char)32);
            assert(pack.csFails_GET() == (char)22086);
        });
        GroundControl.NOVATEL_DIAG p195 = CommunicationChannel.new_NOVATEL_DIAG();
        PH.setPack(p195);
        p195.csFails_SET((char)22086) ;
        p195.receiverStatus_SET(363716375L) ;
        p195.velType_SET((char)242) ;
        p195.timeStatus_SET((char)32) ;
        p195.posType_SET((char)186) ;
        p195.posSolAge_SET(-2.7378375E38F) ;
        p195.solStatus_SET((char)91) ;
        CommunicationChannel.instance.send(p195);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SENSOR_DIAG.add((src, ph, pack) ->
        {
            assert(pack.char1_GET() == (byte) - 103);
            assert(pack.int1_GET() == (short) -32527);
            assert(pack.float1_GET() == -1.5159292E38F);
            assert(pack.float2_GET() == -4.7919997E37F);
        });
        GroundControl.SENSOR_DIAG p196 = CommunicationChannel.new_SENSOR_DIAG();
        PH.setPack(p196);
        p196.char1_SET((byte) - 103) ;
        p196.int1_SET((short) -32527) ;
        p196.float2_SET(-4.7919997E37F) ;
        p196.float1_SET(-1.5159292E38F) ;
        CommunicationChannel.instance.send(p196);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BOOT.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == 150980370L);
        });
        GroundControl.BOOT p197 = CommunicationChannel.new_BOOT();
        PH.setPack(p197);
        p197.version_SET(150980370L) ;
        CommunicationChannel.instance.send(p197);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        CommunicationChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pos_vert_ratio_GET() == -1.53551E38F);
            assert(pack.time_usec_GET() == 5851700751588333980L);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE));
            assert(pack.vel_ratio_GET() == 2.1312882E38F);
            assert(pack.pos_horiz_ratio_GET() == 1.4560416E38F);
            assert(pack.pos_vert_accuracy_GET() == -2.6872515E38F);
            assert(pack.tas_ratio_GET() == 1.1895127E38F);
            assert(pack.mag_ratio_GET() == 1.2656042E37F);
            assert(pack.hagl_ratio_GET() == -2.0855077E38F);
            assert(pack.pos_horiz_accuracy_GET() == 2.0506865E38F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.pos_horiz_accuracy_SET(2.0506865E38F) ;
        p230.hagl_ratio_SET(-2.0855077E38F) ;
        p230.pos_horiz_ratio_SET(1.4560416E38F) ;
        p230.mag_ratio_SET(1.2656042E37F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE)) ;
        p230.pos_vert_ratio_SET(-1.53551E38F) ;
        p230.time_usec_SET(5851700751588333980L) ;
        p230.vel_ratio_SET(2.1312882E38F) ;
        p230.tas_ratio_SET(1.1895127E38F) ;
        p230.pos_vert_accuracy_SET(-2.6872515E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_x_GET() == 1.5539482E38F);
            assert(pack.horiz_accuracy_GET() == -1.6051531E38F);
            assert(pack.vert_accuracy_GET() == 2.4473104E38F);
            assert(pack.var_vert_GET() == 3.0994039E38F);
            assert(pack.var_horiz_GET() == 2.446843E38F);
            assert(pack.wind_alt_GET() == -2.5503272E38F);
            assert(pack.time_usec_GET() == 606124400584931969L);
            assert(pack.wind_y_GET() == 3.4909815E37F);
            assert(pack.wind_z_GET() == -1.8121962E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_alt_SET(-2.5503272E38F) ;
        p231.var_horiz_SET(2.446843E38F) ;
        p231.vert_accuracy_SET(2.4473104E38F) ;
        p231.time_usec_SET(606124400584931969L) ;
        p231.horiz_accuracy_SET(-1.6051531E38F) ;
        p231.wind_z_SET(-1.8121962E38F) ;
        p231.var_vert_SET(3.0994039E38F) ;
        p231.wind_y_SET(3.4909815E37F) ;
        p231.wind_x_SET(1.5539482E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -7.602332E37F);
            assert(pack.vert_accuracy_GET() == 3.130388E37F);
            assert(pack.hdop_GET() == -3.0428732E38F);
            assert(pack.speed_accuracy_GET() == -2.624877E38F);
            assert(pack.horiz_accuracy_GET() == 2.629168E38F);
            assert(pack.vdop_GET() == 8.3129094E37F);
            assert(pack.lon_GET() == -172274116);
            assert(pack.time_usec_GET() == 1911915219321948915L);
            assert(pack.time_week_ms_GET() == 2706528090L);
            assert(pack.ve_GET() == 2.2448705E38F);
            assert(pack.time_week_GET() == (char)55512);
            assert(pack.satellites_visible_GET() == (char)221);
            assert(pack.vd_GET() == 2.2378457E38F);
            assert(pack.vn_GET() == -1.7896484E38F);
            assert(pack.lat_GET() == 583257090);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY));
            assert(pack.gps_id_GET() == (char)150);
            assert(pack.fix_type_GET() == (char)200);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.speed_accuracy_SET(-2.624877E38F) ;
        p232.lat_SET(583257090) ;
        p232.fix_type_SET((char)200) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY)) ;
        p232.vdop_SET(8.3129094E37F) ;
        p232.satellites_visible_SET((char)221) ;
        p232.vd_SET(2.2378457E38F) ;
        p232.alt_SET(-7.602332E37F) ;
        p232.horiz_accuracy_SET(2.629168E38F) ;
        p232.time_week_ms_SET(2706528090L) ;
        p232.gps_id_SET((char)150) ;
        p232.lon_SET(-172274116) ;
        p232.vert_accuracy_SET(3.130388E37F) ;
        p232.time_week_SET((char)55512) ;
        p232.vn_SET(-1.7896484E38F) ;
        p232.ve_SET(2.2448705E38F) ;
        p232.time_usec_SET(1911915219321948915L) ;
        p232.hdop_SET(-3.0428732E38F) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)49);
            assert(pack.flags_GET() == (char)140);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)39, (char)14, (char)203, (char)192, (char)154, (char)253, (char)133, (char)4, (char)20, (char)216, (char)39, (char)238, (char)163, (char)3, (char)127, (char)105, (char)140, (char)54, (char)75, (char)221, (char)28, (char)149, (char)223, (char)154, (char)26, (char)88, (char)105, (char)63, (char)2, (char)58, (char)221, (char)13, (char)111, (char)143, (char)153, (char)137, (char)159, (char)222, (char)239, (char)174, (char)249, (char)37, (char)229, (char)151, (char)23, (char)176, (char)170, (char)215, (char)74, (char)205, (char)85, (char)68, (char)196, (char)146, (char)53, (char)113, (char)230, (char)194, (char)24, (char)111, (char)39, (char)172, (char)14, (char)117, (char)19, (char)192, (char)123, (char)235, (char)130, (char)109, (char)230, (char)206, (char)75, (char)225, (char)25, (char)89, (char)154, (char)178, (char)147, (char)232, (char)33, (char)120, (char)139, (char)225, (char)9, (char)43, (char)186, (char)237, (char)242, (char)38, (char)84, (char)82, (char)253, (char)239, (char)30, (char)217, (char)88, (char)45, (char)87, (char)125, (char)187, (char)225, (char)168, (char)119, (char)255, (char)149, (char)151, (char)173, (char)104, (char)104, (char)242, (char)34, (char)169, (char)191, (char)105, (char)29, (char)94, (char)111, (char)69, (char)92, (char)222, (char)36, (char)49, (char)128, (char)131, (char)215, (char)54, (char)224, (char)198, (char)154, (char)27, (char)116, (char)48, (char)220, (char)7, (char)195, (char)101, (char)182, (char)199, (char)99, (char)10, (char)60, (char)229, (char)166, (char)61, (char)135, (char)212, (char)134, (char)96, (char)96, (char)82, (char)251, (char)65, (char)65, (char)13, (char)68, (char)22, (char)242, (char)96, (char)192, (char)84, (char)237, (char)174, (char)157, (char)198, (char)58, (char)47, (char)118, (char)209, (char)158, (char)87, (char)143, (char)91, (char)117, (char)130, (char)188, (char)104, (char)232, (char)199, (char)53}));
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)140) ;
        p233.len_SET((char)49) ;
        p233.data__SET(new char[] {(char)39, (char)14, (char)203, (char)192, (char)154, (char)253, (char)133, (char)4, (char)20, (char)216, (char)39, (char)238, (char)163, (char)3, (char)127, (char)105, (char)140, (char)54, (char)75, (char)221, (char)28, (char)149, (char)223, (char)154, (char)26, (char)88, (char)105, (char)63, (char)2, (char)58, (char)221, (char)13, (char)111, (char)143, (char)153, (char)137, (char)159, (char)222, (char)239, (char)174, (char)249, (char)37, (char)229, (char)151, (char)23, (char)176, (char)170, (char)215, (char)74, (char)205, (char)85, (char)68, (char)196, (char)146, (char)53, (char)113, (char)230, (char)194, (char)24, (char)111, (char)39, (char)172, (char)14, (char)117, (char)19, (char)192, (char)123, (char)235, (char)130, (char)109, (char)230, (char)206, (char)75, (char)225, (char)25, (char)89, (char)154, (char)178, (char)147, (char)232, (char)33, (char)120, (char)139, (char)225, (char)9, (char)43, (char)186, (char)237, (char)242, (char)38, (char)84, (char)82, (char)253, (char)239, (char)30, (char)217, (char)88, (char)45, (char)87, (char)125, (char)187, (char)225, (char)168, (char)119, (char)255, (char)149, (char)151, (char)173, (char)104, (char)104, (char)242, (char)34, (char)169, (char)191, (char)105, (char)29, (char)94, (char)111, (char)69, (char)92, (char)222, (char)36, (char)49, (char)128, (char)131, (char)215, (char)54, (char)224, (char)198, (char)154, (char)27, (char)116, (char)48, (char)220, (char)7, (char)195, (char)101, (char)182, (char)199, (char)99, (char)10, (char)60, (char)229, (char)166, (char)61, (char)135, (char)212, (char)134, (char)96, (char)96, (char)82, (char)251, (char)65, (char)65, (char)13, (char)68, (char)22, (char)242, (char)96, (char)192, (char)84, (char)237, (char)174, (char)157, (char)198, (char)58, (char)47, (char)118, (char)209, (char)158, (char)87, (char)143, (char)91, (char)117, (char)130, (char)188, (char)104, (char)232, (char)199, (char)53}, 0) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.temperature_air_GET() == (byte) - 15);
            assert(pack.temperature_GET() == (byte)54);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
            assert(pack.heading_GET() == (char)32916);
            assert(pack.heading_sp_GET() == (short)32178);
            assert(pack.roll_GET() == (short) -31326);
            assert(pack.pitch_GET() == (short) -30212);
            assert(pack.latitude_GET() == -85853193);
            assert(pack.gps_nsat_GET() == (char)96);
            assert(pack.custom_mode_GET() == 4074837746L);
            assert(pack.wp_distance_GET() == (char)16571);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED));
            assert(pack.failsafe_GET() == (char)243);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.longitude_GET() == -989957229);
            assert(pack.battery_remaining_GET() == (char)174);
            assert(pack.wp_num_GET() == (char)146);
            assert(pack.climb_rate_GET() == (byte) - 14);
            assert(pack.groundspeed_GET() == (char)195);
            assert(pack.airspeed_GET() == (char)139);
            assert(pack.altitude_amsl_GET() == (short)19241);
            assert(pack.altitude_sp_GET() == (short)23963);
            assert(pack.airspeed_sp_GET() == (char)99);
            assert(pack.throttle_GET() == (byte)86);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.throttle_SET((byte)86) ;
        p234.latitude_SET(-85853193) ;
        p234.temperature_SET((byte)54) ;
        p234.wp_distance_SET((char)16571) ;
        p234.gps_nsat_SET((char)96) ;
        p234.wp_num_SET((char)146) ;
        p234.failsafe_SET((char)243) ;
        p234.climb_rate_SET((byte) - 14) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS) ;
        p234.heading_SET((char)32916) ;
        p234.longitude_SET(-989957229) ;
        p234.temperature_air_SET((byte) - 15) ;
        p234.pitch_SET((short) -30212) ;
        p234.airspeed_sp_SET((char)99) ;
        p234.groundspeed_SET((char)195) ;
        p234.roll_SET((short) -31326) ;
        p234.altitude_amsl_SET((short)19241) ;
        p234.heading_sp_SET((short)32178) ;
        p234.altitude_sp_SET((short)23963) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        p234.airspeed_SET((char)139) ;
        p234.battery_remaining_SET((char)174) ;
        p234.custom_mode_SET(4074837746L) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 8079941033317226849L);
            assert(pack.clipping_1_GET() == 3207673153L);
            assert(pack.clipping_2_GET() == 3412326637L);
            assert(pack.vibration_y_GET() == 8.969219E37F);
            assert(pack.vibration_z_GET() == -2.3466529E38F);
            assert(pack.vibration_x_GET() == 1.6927134E38F);
            assert(pack.clipping_0_GET() == 319658197L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.vibration_z_SET(-2.3466529E38F) ;
        p241.clipping_0_SET(319658197L) ;
        p241.vibration_y_SET(8.969219E37F) ;
        p241.vibration_x_SET(1.6927134E38F) ;
        p241.clipping_1_SET(3207673153L) ;
        p241.time_usec_SET(8079941033317226849L) ;
        p241.clipping_2_SET(3412326637L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 1620506055);
            assert(pack.approach_y_GET() == 1.7078604E38F);
            assert(pack.approach_z_GET() == -9.6686646E36F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.5394078E38F, 9.901969E37F, 3.341649E38F, 5.581661E36F}));
            assert(pack.altitude_GET() == -1412694542);
            assert(pack.y_GET() == -2.349587E38F);
            assert(pack.longitude_GET() == -1349109929);
            assert(pack.x_GET() == -2.467146E38F);
            assert(pack.approach_x_GET() == -1.548952E38F);
            assert(pack.z_GET() == 2.4070809E38F);
            assert(pack.time_usec_TRY(ph) == 4986521853447120734L);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(1620506055) ;
        p242.approach_y_SET(1.7078604E38F) ;
        p242.time_usec_SET(4986521853447120734L, PH) ;
        p242.y_SET(-2.349587E38F) ;
        p242.x_SET(-2.467146E38F) ;
        p242.altitude_SET(-1412694542) ;
        p242.longitude_SET(-1349109929) ;
        p242.q_SET(new float[] {-2.5394078E38F, 9.901969E37F, 3.341649E38F, 5.581661E36F}, 0) ;
        p242.approach_z_SET(-9.6686646E36F) ;
        p242.z_SET(2.4070809E38F) ;
        p242.approach_x_SET(-1.548952E38F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.189335E38F, 2.9498077E38F, -1.5662979E38F, -8.1134395E37F}));
            assert(pack.approach_z_GET() == -4.8964576E37F);
            assert(pack.approach_x_GET() == -2.7688562E38F);
            assert(pack.z_GET() == 2.7335569E38F);
            assert(pack.longitude_GET() == 1321732958);
            assert(pack.approach_y_GET() == 2.5336689E38F);
            assert(pack.latitude_GET() == -1278770835);
            assert(pack.target_system_GET() == (char)76);
            assert(pack.time_usec_TRY(ph) == 5523811849161008149L);
            assert(pack.x_GET() == 3.0403523E38F);
            assert(pack.altitude_GET() == -2054477623);
            assert(pack.y_GET() == -1.1723758E38F);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.x_SET(3.0403523E38F) ;
        p243.approach_y_SET(2.5336689E38F) ;
        p243.altitude_SET(-2054477623) ;
        p243.time_usec_SET(5523811849161008149L, PH) ;
        p243.longitude_SET(1321732958) ;
        p243.approach_z_SET(-4.8964576E37F) ;
        p243.latitude_SET(-1278770835) ;
        p243.y_SET(-1.1723758E38F) ;
        p243.q_SET(new float[] {-3.189335E38F, 2.9498077E38F, -1.5662979E38F, -8.1134395E37F}, 0) ;
        p243.target_system_SET((char)76) ;
        p243.z_SET(2.7335569E38F) ;
        p243.approach_x_SET(-2.7688562E38F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)45168);
            assert(pack.interval_us_GET() == -1240469841);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-1240469841) ;
        p244.message_id_SET((char)45168) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_FW) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 698994400);
            assert(pack.heading_GET() == (char)15271);
            assert(pack.altitude_GET() == -558608744);
            assert(pack.ICAO_address_GET() == 1457795630L);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.ver_velocity_GET() == (short) -23426);
            assert(pack.tslc_GET() == (char)77);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_EMERGENCY_SURFACE);
            assert(pack.lat_GET() == 715054558);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK));
            assert(pack.hor_velocity_GET() == (char)60994);
            assert(pack.squawk_GET() == (char)57932);
            assert(pack.callsign_LEN(ph) == 9);
            assert(pack.callsign_TRY(ph).equals("dhegdyHkc"));
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.heading_SET((char)15271) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK)) ;
        p246.hor_velocity_SET((char)60994) ;
        p246.squawk_SET((char)57932) ;
        p246.callsign_SET("dhegdyHkc", PH) ;
        p246.ICAO_address_SET(1457795630L) ;
        p246.lon_SET(698994400) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.ver_velocity_SET((short) -23426) ;
        p246.tslc_SET((char)77) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_EMERGENCY_SURFACE) ;
        p246.altitude_SET(-558608744) ;
        p246.lat_SET(715054558) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
            assert(pack.id_GET() == 4289374356L);
            assert(pack.altitude_minimum_delta_GET() == 3.243139E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
            assert(pack.time_to_minimum_delta_GET() == -2.8526599E38F);
            assert(pack.horizontal_minimum_delta_GET() == 3.3284377E38F);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.id_SET(4289374356L) ;
        p247.altitude_minimum_delta_SET(3.243139E38F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.horizontal_minimum_delta_SET(3.3284377E38F) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER) ;
        p247.time_to_minimum_delta_SET(-2.8526599E38F) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)64);
            assert(pack.message_type_GET() == (char)10713);
            assert(pack.target_system_GET() == (char)61);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)17, (char)99, (char)30, (char)2, (char)177, (char)233, (char)16, (char)168, (char)57, (char)87, (char)128, (char)243, (char)129, (char)154, (char)202, (char)202, (char)86, (char)114, (char)116, (char)127, (char)253, (char)40, (char)17, (char)201, (char)43, (char)209, (char)14, (char)138, (char)89, (char)178, (char)9, (char)161, (char)167, (char)62, (char)67, (char)146, (char)172, (char)15, (char)49, (char)144, (char)114, (char)170, (char)152, (char)82, (char)49, (char)252, (char)223, (char)102, (char)159, (char)166, (char)0, (char)161, (char)99, (char)135, (char)169, (char)242, (char)72, (char)211, (char)112, (char)10, (char)129, (char)201, (char)147, (char)235, (char)43, (char)199, (char)14, (char)146, (char)150, (char)170, (char)208, (char)53, (char)95, (char)71, (char)19, (char)25, (char)87, (char)183, (char)6, (char)181, (char)192, (char)143, (char)251, (char)211, (char)250, (char)229, (char)222, (char)198, (char)76, (char)143, (char)93, (char)114, (char)234, (char)35, (char)17, (char)193, (char)133, (char)72, (char)254, (char)73, (char)70, (char)101, (char)241, (char)229, (char)44, (char)70, (char)196, (char)196, (char)39, (char)255, (char)163, (char)96, (char)76, (char)2, (char)209, (char)249, (char)141, (char)7, (char)94, (char)33, (char)177, (char)5, (char)3, (char)232, (char)115, (char)229, (char)16, (char)64, (char)134, (char)12, (char)84, (char)92, (char)68, (char)211, (char)7, (char)171, (char)217, (char)81, (char)9, (char)106, (char)34, (char)144, (char)57, (char)106, (char)126, (char)143, (char)59, (char)178, (char)62, (char)86, (char)191, (char)92, (char)142, (char)194, (char)224, (char)57, (char)163, (char)63, (char)240, (char)55, (char)46, (char)214, (char)112, (char)217, (char)128, (char)151, (char)140, (char)99, (char)76, (char)89, (char)119, (char)94, (char)193, (char)105, (char)226, (char)170, (char)130, (char)123, (char)240, (char)246, (char)38, (char)108, (char)27, (char)215, (char)153, (char)232, (char)55, (char)129, (char)64, (char)112, (char)9, (char)209, (char)162, (char)170, (char)139, (char)220, (char)230, (char)86, (char)79, (char)220, (char)45, (char)166, (char)133, (char)129, (char)8, (char)142, (char)253, (char)131, (char)209, (char)154, (char)163, (char)155, (char)118, (char)198, (char)225, (char)1, (char)59, (char)107, (char)5, (char)191, (char)237, (char)70, (char)142, (char)113, (char)22, (char)20, (char)153, (char)110, (char)22, (char)128, (char)0, (char)231, (char)157, (char)27, (char)217, (char)245, (char)133, (char)220, (char)251, (char)35, (char)32, (char)84, (char)113, (char)130, (char)208, (char)107, (char)171, (char)177, (char)179}));
            assert(pack.target_network_GET() == (char)247);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.payload_SET(new char[] {(char)17, (char)99, (char)30, (char)2, (char)177, (char)233, (char)16, (char)168, (char)57, (char)87, (char)128, (char)243, (char)129, (char)154, (char)202, (char)202, (char)86, (char)114, (char)116, (char)127, (char)253, (char)40, (char)17, (char)201, (char)43, (char)209, (char)14, (char)138, (char)89, (char)178, (char)9, (char)161, (char)167, (char)62, (char)67, (char)146, (char)172, (char)15, (char)49, (char)144, (char)114, (char)170, (char)152, (char)82, (char)49, (char)252, (char)223, (char)102, (char)159, (char)166, (char)0, (char)161, (char)99, (char)135, (char)169, (char)242, (char)72, (char)211, (char)112, (char)10, (char)129, (char)201, (char)147, (char)235, (char)43, (char)199, (char)14, (char)146, (char)150, (char)170, (char)208, (char)53, (char)95, (char)71, (char)19, (char)25, (char)87, (char)183, (char)6, (char)181, (char)192, (char)143, (char)251, (char)211, (char)250, (char)229, (char)222, (char)198, (char)76, (char)143, (char)93, (char)114, (char)234, (char)35, (char)17, (char)193, (char)133, (char)72, (char)254, (char)73, (char)70, (char)101, (char)241, (char)229, (char)44, (char)70, (char)196, (char)196, (char)39, (char)255, (char)163, (char)96, (char)76, (char)2, (char)209, (char)249, (char)141, (char)7, (char)94, (char)33, (char)177, (char)5, (char)3, (char)232, (char)115, (char)229, (char)16, (char)64, (char)134, (char)12, (char)84, (char)92, (char)68, (char)211, (char)7, (char)171, (char)217, (char)81, (char)9, (char)106, (char)34, (char)144, (char)57, (char)106, (char)126, (char)143, (char)59, (char)178, (char)62, (char)86, (char)191, (char)92, (char)142, (char)194, (char)224, (char)57, (char)163, (char)63, (char)240, (char)55, (char)46, (char)214, (char)112, (char)217, (char)128, (char)151, (char)140, (char)99, (char)76, (char)89, (char)119, (char)94, (char)193, (char)105, (char)226, (char)170, (char)130, (char)123, (char)240, (char)246, (char)38, (char)108, (char)27, (char)215, (char)153, (char)232, (char)55, (char)129, (char)64, (char)112, (char)9, (char)209, (char)162, (char)170, (char)139, (char)220, (char)230, (char)86, (char)79, (char)220, (char)45, (char)166, (char)133, (char)129, (char)8, (char)142, (char)253, (char)131, (char)209, (char)154, (char)163, (char)155, (char)118, (char)198, (char)225, (char)1, (char)59, (char)107, (char)5, (char)191, (char)237, (char)70, (char)142, (char)113, (char)22, (char)20, (char)153, (char)110, (char)22, (char)128, (char)0, (char)231, (char)157, (char)27, (char)217, (char)245, (char)133, (char)220, (char)251, (char)35, (char)32, (char)84, (char)113, (char)130, (char)208, (char)107, (char)171, (char)177, (char)179}, 0) ;
        p248.target_component_SET((char)64) ;
        p248.message_type_SET((char)10713) ;
        p248.target_system_SET((char)61) ;
        p248.target_network_SET((char)247) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.address_GET() == (char)25819);
            assert(pack.type_GET() == (char)202);
            assert(pack.ver_GET() == (char)189);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)126, (byte) - 5, (byte)35, (byte) - 97, (byte)97, (byte) - 62, (byte) - 88, (byte) - 67, (byte) - 47, (byte) - 39, (byte) - 79, (byte)45, (byte)45, (byte) - 61, (byte) - 55, (byte)124, (byte) - 94, (byte)35, (byte)104, (byte)10, (byte) - 25, (byte) - 11, (byte) - 71, (byte) - 28, (byte)120, (byte) - 92, (byte) - 38, (byte) - 73, (byte) - 16, (byte) - 115, (byte) - 43, (byte)16}));
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)25819) ;
        p249.value_SET(new byte[] {(byte)126, (byte) - 5, (byte)35, (byte) - 97, (byte)97, (byte) - 62, (byte) - 88, (byte) - 67, (byte) - 47, (byte) - 39, (byte) - 79, (byte)45, (byte)45, (byte) - 61, (byte) - 55, (byte)124, (byte) - 94, (byte)35, (byte)104, (byte)10, (byte) - 25, (byte) - 11, (byte) - 71, (byte) - 28, (byte)120, (byte) - 92, (byte) - 38, (byte) - 73, (byte) - 16, (byte) - 115, (byte) - 43, (byte)16}, 0) ;
        p249.ver_SET((char)189) ;
        p249.type_SET((char)202) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 3.0815984E36F);
            assert(pack.name_LEN(ph) == 7);
            assert(pack.name_TRY(ph).equals("nbtfnkh"));
            assert(pack.x_GET() == -2.9838538E38F);
            assert(pack.z_GET() == 1.9257815E38F);
            assert(pack.time_usec_GET() == 5403261818879584350L);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.y_SET(3.0815984E36F) ;
        p250.name_SET("nbtfnkh", PH) ;
        p250.z_SET(1.9257815E38F) ;
        p250.x_SET(-2.9838538E38F) ;
        p250.time_usec_SET(5403261818879584350L) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2934200204L);
            assert(pack.name_LEN(ph) == 7);
            assert(pack.name_TRY(ph).equals("ydwkdmq"));
            assert(pack.value_GET() == -1.60929E38F);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("ydwkdmq", PH) ;
        p251.value_SET(-1.60929E38F) ;
        p251.time_boot_ms_SET(2934200204L) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1786407035L);
            assert(pack.name_LEN(ph) == 1);
            assert(pack.name_TRY(ph).equals("o"));
            assert(pack.value_GET() == 445999534);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.name_SET("o", PH) ;
        p252.value_SET(445999534) ;
        p252.time_boot_ms_SET(1786407035L) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 31);
            assert(pack.text_TRY(ph).equals("vlyscQdprybcqsxehseRlOifuschvOo"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_WARNING);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_WARNING) ;
        p253.text_SET("vlyscQdprybcqsxehseRlOifuschvOo", PH) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 3.454131E37F);
            assert(pack.time_boot_ms_GET() == 2223931988L);
            assert(pack.ind_GET() == (char)59);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(2223931988L) ;
        p254.value_SET(3.454131E37F) ;
        p254.ind_SET((char)59) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)222, (char)34, (char)56, (char)236, (char)3, (char)107, (char)189, (char)152, (char)55, (char)178, (char)31, (char)192, (char)195, (char)172, (char)128, (char)136, (char)180, (char)148, (char)175, (char)49, (char)75, (char)41, (char)17, (char)201, (char)29, (char)58, (char)238, (char)53, (char)33, (char)213, (char)224, (char)24}));
            assert(pack.initial_timestamp_GET() == 4314874006643521077L);
            assert(pack.target_system_GET() == (char)184);
            assert(pack.target_component_GET() == (char)24);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.secret_key_SET(new char[] {(char)222, (char)34, (char)56, (char)236, (char)3, (char)107, (char)189, (char)152, (char)55, (char)178, (char)31, (char)192, (char)195, (char)172, (char)128, (char)136, (char)180, (char)148, (char)175, (char)49, (char)75, (char)41, (char)17, (char)201, (char)29, (char)58, (char)238, (char)53, (char)33, (char)213, (char)224, (char)24}, 0) ;
        p256.initial_timestamp_SET(4314874006643521077L) ;
        p256.target_system_SET((char)184) ;
        p256.target_component_SET((char)24) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.state_GET() == (char)193);
            assert(pack.last_change_ms_GET() == 3867771521L);
            assert(pack.time_boot_ms_GET() == 886598651L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(886598651L) ;
        p257.last_change_ms_SET(3867771521L) ;
        p257.state_SET((char)193) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)35);
            assert(pack.tune_LEN(ph) == 26);
            assert(pack.tune_TRY(ph).equals("zyjYbwrxomcrxnohkjwjqcfdqg"));
            assert(pack.target_system_GET() == (char)88);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)88) ;
        p258.tune_SET("zyjYbwrxomcrxnohkjwjqcfdqg", PH) ;
        p258.target_component_SET((char)35) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.firmware_version_GET() == 1919828566L);
            assert(pack.sensor_size_v_GET() == 2.9115036E38F);
            assert(pack.resolution_h_GET() == (char)36017);
            assert(pack.sensor_size_h_GET() == 1.0743836E38F);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)222, (char)12, (char)12, (char)186, (char)20, (char)69, (char)110, (char)252, (char)172, (char)89, (char)196, (char)168, (char)254, (char)175, (char)248, (char)108, (char)214, (char)151, (char)97, (char)43, (char)250, (char)6, (char)161, (char)5, (char)100, (char)87, (char)135, (char)62, (char)68, (char)231, (char)178, (char)74}));
            assert(pack.cam_definition_uri_LEN(ph) == 110);
            assert(pack.cam_definition_uri_TRY(ph).equals("ydqbtrluvntiHanrglhgmctosqnzflrrRbumHIhIwsobaqyfpagXqxwalnorgntesvcvbqkacusiroedPjsgpwkthuHnnNhdwwonfustxvgqjo"));
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO));
            assert(pack.cam_definition_version_GET() == (char)33372);
            assert(pack.focal_length_GET() == 2.2096582E38F);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)62, (char)202, (char)179, (char)33, (char)205, (char)42, (char)63, (char)202, (char)105, (char)220, (char)171, (char)9, (char)54, (char)234, (char)177, (char)53, (char)208, (char)54, (char)161, (char)147, (char)3, (char)111, (char)110, (char)14, (char)128, (char)116, (char)221, (char)191, (char)189, (char)249, (char)14, (char)90}));
            assert(pack.resolution_v_GET() == (char)18199);
            assert(pack.lens_id_GET() == (char)42);
            assert(pack.time_boot_ms_GET() == 1579442030L);
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.sensor_size_v_SET(2.9115036E38F) ;
        p259.resolution_h_SET((char)36017) ;
        p259.time_boot_ms_SET(1579442030L) ;
        p259.sensor_size_h_SET(1.0743836E38F) ;
        p259.model_name_SET(new char[] {(char)62, (char)202, (char)179, (char)33, (char)205, (char)42, (char)63, (char)202, (char)105, (char)220, (char)171, (char)9, (char)54, (char)234, (char)177, (char)53, (char)208, (char)54, (char)161, (char)147, (char)3, (char)111, (char)110, (char)14, (char)128, (char)116, (char)221, (char)191, (char)189, (char)249, (char)14, (char)90}, 0) ;
        p259.cam_definition_uri_SET("ydqbtrluvntiHanrglhgmctosqnzflrrRbumHIhIwsobaqyfpagXqxwalnorgntesvcvbqkacusiroedPjsgpwkthuHnnNhdwwonfustxvgqjo", PH) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO)) ;
        p259.firmware_version_SET(1919828566L) ;
        p259.resolution_v_SET((char)18199) ;
        p259.lens_id_SET((char)42) ;
        p259.cam_definition_version_SET((char)33372) ;
        p259.vendor_name_SET(new char[] {(char)222, (char)12, (char)12, (char)186, (char)20, (char)69, (char)110, (char)252, (char)172, (char)89, (char)196, (char)168, (char)254, (char)175, (char)248, (char)108, (char)214, (char)151, (char)97, (char)43, (char)250, (char)6, (char)161, (char)5, (char)100, (char)87, (char)135, (char)62, (char)68, (char)231, (char)178, (char)74}, 0) ;
        p259.focal_length_SET(2.2096582E38F) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 240231653L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY) ;
        p260.time_boot_ms_SET(240231653L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2185911289L);
            assert(pack.used_capacity_GET() == 1.361692E38F);
            assert(pack.read_speed_GET() == -2.7301326E38F);
            assert(pack.write_speed_GET() == -2.1209888E38F);
            assert(pack.total_capacity_GET() == -3.2979997E38F);
            assert(pack.status_GET() == (char)171);
            assert(pack.available_capacity_GET() == 5.1010575E37F);
            assert(pack.storage_count_GET() == (char)39);
            assert(pack.storage_id_GET() == (char)223);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.status_SET((char)171) ;
        p261.read_speed_SET(-2.7301326E38F) ;
        p261.storage_id_SET((char)223) ;
        p261.write_speed_SET(-2.1209888E38F) ;
        p261.available_capacity_SET(5.1010575E37F) ;
        p261.time_boot_ms_SET(2185911289L) ;
        p261.used_capacity_SET(1.361692E38F) ;
        p261.storage_count_SET((char)39) ;
        p261.total_capacity_SET(-3.2979997E38F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.video_status_GET() == (char)166);
            assert(pack.time_boot_ms_GET() == 3275800892L);
            assert(pack.available_capacity_GET() == 1.545282E38F);
            assert(pack.recording_time_ms_GET() == 2315335420L);
            assert(pack.image_status_GET() == (char)244);
            assert(pack.image_interval_GET() == 2.868259E38F);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.recording_time_ms_SET(2315335420L) ;
        p262.time_boot_ms_SET(3275800892L) ;
        p262.image_interval_SET(2.868259E38F) ;
        p262.video_status_SET((char)166) ;
        p262.available_capacity_SET(1.545282E38F) ;
        p262.image_status_SET((char)244) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.camera_id_GET() == (char)28);
            assert(Arrays.equals(pack.q_GET(),  new float[] {9.822461E37F, -1.1471111E38F, -2.0897845E38F, -2.4314413E38F}));
            assert(pack.lat_GET() == 1751159638);
            assert(pack.alt_GET() == 1798598127);
            assert(pack.capture_result_GET() == (byte)124);
            assert(pack.time_boot_ms_GET() == 1736549249L);
            assert(pack.image_index_GET() == 312329347);
            assert(pack.time_utc_GET() == 4738591397817713044L);
            assert(pack.relative_alt_GET() == -188409989);
            assert(pack.file_url_LEN(ph) == 120);
            assert(pack.file_url_TRY(ph).equals("ozttplFywtbrXlxgsrvActcseFzvqjgberipgtuNlrCPkvoiexgumbvvdhjyjmpfzcdokzzFevxanAgthmVforjtllgerzynxqkVtigjrttnZmhkzwgpTycl"));
            assert(pack.lon_GET() == -879478517);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.image_index_SET(312329347) ;
        p263.capture_result_SET((byte)124) ;
        p263.q_SET(new float[] {9.822461E37F, -1.1471111E38F, -2.0897845E38F, -2.4314413E38F}, 0) ;
        p263.time_utc_SET(4738591397817713044L) ;
        p263.file_url_SET("ozttplFywtbrXlxgsrvActcseFzvqjgberipgtuNlrCPkvoiexgumbvvdhjyjmpfzcdokzzFevxanAgthmVforjtllgerzynxqkVtigjrttnZmhkzwgpTycl", PH) ;
        p263.time_boot_ms_SET(1736549249L) ;
        p263.lat_SET(1751159638) ;
        p263.camera_id_SET((char)28) ;
        p263.relative_alt_SET(-188409989) ;
        p263.alt_SET(1798598127) ;
        p263.lon_SET(-879478517) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.arming_time_utc_GET() == 3730335909894974938L);
            assert(pack.flight_uuid_GET() == 4535035658279176582L);
            assert(pack.takeoff_time_utc_GET() == 3742571873824876639L);
            assert(pack.time_boot_ms_GET() == 3994367623L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.takeoff_time_utc_SET(3742571873824876639L) ;
        p264.arming_time_utc_SET(3730335909894974938L) ;
        p264.time_boot_ms_SET(3994367623L) ;
        p264.flight_uuid_SET(4535035658279176582L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 1.5970939E38F);
            assert(pack.pitch_GET() == 2.6144251E38F);
            assert(pack.time_boot_ms_GET() == 3798517739L);
            assert(pack.yaw_GET() == 2.7429444E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(3798517739L) ;
        p265.pitch_SET(2.6144251E38F) ;
        p265.roll_SET(1.5970939E38F) ;
        p265.yaw_SET(2.7429444E38F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)251, (char)79, (char)25, (char)202, (char)233, (char)0, (char)186, (char)185, (char)198, (char)210, (char)195, (char)226, (char)251, (char)53, (char)26, (char)125, (char)78, (char)76, (char)185, (char)83, (char)161, (char)188, (char)85, (char)234, (char)114, (char)58, (char)74, (char)127, (char)103, (char)212, (char)156, (char)203, (char)195, (char)47, (char)66, (char)180, (char)68, (char)174, (char)205, (char)19, (char)206, (char)103, (char)245, (char)87, (char)180, (char)136, (char)23, (char)216, (char)194, (char)84, (char)187, (char)184, (char)37, (char)9, (char)111, (char)199, (char)70, (char)139, (char)55, (char)214, (char)236, (char)225, (char)164, (char)108, (char)90, (char)47, (char)116, (char)59, (char)118, (char)63, (char)110, (char)125, (char)0, (char)200, (char)19, (char)161, (char)201, (char)125, (char)66, (char)33, (char)125, (char)36, (char)202, (char)91, (char)113, (char)33, (char)178, (char)39, (char)198, (char)47, (char)24, (char)56, (char)113, (char)204, (char)185, (char)166, (char)176, (char)67, (char)111, (char)159, (char)178, (char)3, (char)126, (char)110, (char)209, (char)193, (char)34, (char)126, (char)248, (char)157, (char)216, (char)22, (char)111, (char)253, (char)102, (char)192, (char)92, (char)52, (char)48, (char)61, (char)236, (char)222, (char)181, (char)196, (char)225, (char)146, (char)83, (char)8, (char)98, (char)20, (char)249, (char)89, (char)2, (char)61, (char)147, (char)185, (char)35, (char)127, (char)55, (char)139, (char)80, (char)199, (char)31, (char)78, (char)244, (char)127, (char)246, (char)167, (char)9, (char)121, (char)185, (char)31, (char)20, (char)69, (char)143, (char)116, (char)212, (char)181, (char)38, (char)158, (char)167, (char)146, (char)112, (char)207, (char)196, (char)82, (char)246, (char)27, (char)10, (char)211, (char)178, (char)6, (char)204, (char)245, (char)183, (char)23, (char)117, (char)204, (char)236, (char)0, (char)129, (char)50, (char)89, (char)110, (char)124, (char)139, (char)215, (char)140, (char)82, (char)245, (char)164, (char)112, (char)82, (char)13, (char)173, (char)17, (char)33, (char)28, (char)12, (char)147, (char)58, (char)140, (char)23, (char)139, (char)106, (char)184, (char)36, (char)244, (char)227, (char)140, (char)197, (char)163, (char)63, (char)129, (char)108, (char)234, (char)211, (char)90, (char)233, (char)27, (char)188, (char)28, (char)190, (char)84, (char)229, (char)181, (char)66, (char)255, (char)147, (char)243, (char)49, (char)131, (char)58, (char)78, (char)202, (char)188, (char)241, (char)255, (char)42, (char)220, (char)75, (char)3, (char)53, (char)216, (char)25, (char)150, (char)97, (char)255, (char)235}));
            assert(pack.first_message_offset_GET() == (char)190);
            assert(pack.target_system_GET() == (char)255);
            assert(pack.length_GET() == (char)10);
            assert(pack.sequence_GET() == (char)21314);
            assert(pack.target_component_GET() == (char)164);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.length_SET((char)10) ;
        p266.target_system_SET((char)255) ;
        p266.target_component_SET((char)164) ;
        p266.sequence_SET((char)21314) ;
        p266.first_message_offset_SET((char)190) ;
        p266.data__SET(new char[] {(char)251, (char)79, (char)25, (char)202, (char)233, (char)0, (char)186, (char)185, (char)198, (char)210, (char)195, (char)226, (char)251, (char)53, (char)26, (char)125, (char)78, (char)76, (char)185, (char)83, (char)161, (char)188, (char)85, (char)234, (char)114, (char)58, (char)74, (char)127, (char)103, (char)212, (char)156, (char)203, (char)195, (char)47, (char)66, (char)180, (char)68, (char)174, (char)205, (char)19, (char)206, (char)103, (char)245, (char)87, (char)180, (char)136, (char)23, (char)216, (char)194, (char)84, (char)187, (char)184, (char)37, (char)9, (char)111, (char)199, (char)70, (char)139, (char)55, (char)214, (char)236, (char)225, (char)164, (char)108, (char)90, (char)47, (char)116, (char)59, (char)118, (char)63, (char)110, (char)125, (char)0, (char)200, (char)19, (char)161, (char)201, (char)125, (char)66, (char)33, (char)125, (char)36, (char)202, (char)91, (char)113, (char)33, (char)178, (char)39, (char)198, (char)47, (char)24, (char)56, (char)113, (char)204, (char)185, (char)166, (char)176, (char)67, (char)111, (char)159, (char)178, (char)3, (char)126, (char)110, (char)209, (char)193, (char)34, (char)126, (char)248, (char)157, (char)216, (char)22, (char)111, (char)253, (char)102, (char)192, (char)92, (char)52, (char)48, (char)61, (char)236, (char)222, (char)181, (char)196, (char)225, (char)146, (char)83, (char)8, (char)98, (char)20, (char)249, (char)89, (char)2, (char)61, (char)147, (char)185, (char)35, (char)127, (char)55, (char)139, (char)80, (char)199, (char)31, (char)78, (char)244, (char)127, (char)246, (char)167, (char)9, (char)121, (char)185, (char)31, (char)20, (char)69, (char)143, (char)116, (char)212, (char)181, (char)38, (char)158, (char)167, (char)146, (char)112, (char)207, (char)196, (char)82, (char)246, (char)27, (char)10, (char)211, (char)178, (char)6, (char)204, (char)245, (char)183, (char)23, (char)117, (char)204, (char)236, (char)0, (char)129, (char)50, (char)89, (char)110, (char)124, (char)139, (char)215, (char)140, (char)82, (char)245, (char)164, (char)112, (char)82, (char)13, (char)173, (char)17, (char)33, (char)28, (char)12, (char)147, (char)58, (char)140, (char)23, (char)139, (char)106, (char)184, (char)36, (char)244, (char)227, (char)140, (char)197, (char)163, (char)63, (char)129, (char)108, (char)234, (char)211, (char)90, (char)233, (char)27, (char)188, (char)28, (char)190, (char)84, (char)229, (char)181, (char)66, (char)255, (char)147, (char)243, (char)49, (char)131, (char)58, (char)78, (char)202, (char)188, (char)241, (char)255, (char)42, (char)220, (char)75, (char)3, (char)53, (char)216, (char)25, (char)150, (char)97, (char)255, (char)235}, 0) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)168, (char)18, (char)175, (char)100, (char)173, (char)16, (char)69, (char)204, (char)157, (char)94, (char)255, (char)102, (char)124, (char)94, (char)211, (char)170, (char)253, (char)14, (char)84, (char)75, (char)255, (char)59, (char)90, (char)166, (char)12, (char)159, (char)230, (char)44, (char)49, (char)78, (char)192, (char)57, (char)20, (char)241, (char)221, (char)78, (char)43, (char)202, (char)183, (char)240, (char)125, (char)220, (char)202, (char)198, (char)57, (char)231, (char)251, (char)77, (char)250, (char)118, (char)195, (char)69, (char)66, (char)184, (char)111, (char)40, (char)169, (char)172, (char)66, (char)70, (char)31, (char)168, (char)95, (char)229, (char)129, (char)195, (char)61, (char)250, (char)48, (char)167, (char)2, (char)224, (char)188, (char)69, (char)12, (char)99, (char)3, (char)120, (char)147, (char)59, (char)68, (char)48, (char)143, (char)15, (char)151, (char)58, (char)54, (char)88, (char)228, (char)225, (char)162, (char)207, (char)48, (char)242, (char)39, (char)15, (char)16, (char)125, (char)45, (char)80, (char)252, (char)71, (char)58, (char)53, (char)196, (char)86, (char)23, (char)158, (char)196, (char)184, (char)109, (char)130, (char)198, (char)68, (char)160, (char)165, (char)66, (char)109, (char)42, (char)28, (char)109, (char)118, (char)236, (char)55, (char)185, (char)153, (char)74, (char)254, (char)44, (char)221, (char)113, (char)75, (char)209, (char)99, (char)115, (char)171, (char)110, (char)168, (char)73, (char)170, (char)133, (char)9, (char)135, (char)211, (char)66, (char)174, (char)196, (char)100, (char)128, (char)34, (char)104, (char)75, (char)21, (char)231, (char)123, (char)53, (char)182, (char)225, (char)246, (char)41, (char)16, (char)111, (char)34, (char)5, (char)244, (char)200, (char)0, (char)160, (char)254, (char)170, (char)92, (char)215, (char)165, (char)34, (char)37, (char)216, (char)236, (char)161, (char)10, (char)26, (char)67, (char)1, (char)109, (char)110, (char)101, (char)252, (char)3, (char)221, (char)109, (char)216, (char)72, (char)171, (char)43, (char)178, (char)67, (char)57, (char)131, (char)8, (char)205, (char)10, (char)30, (char)33, (char)12, (char)213, (char)225, (char)100, (char)73, (char)239, (char)96, (char)63, (char)234, (char)221, (char)217, (char)189, (char)15, (char)204, (char)6, (char)20, (char)194, (char)171, (char)171, (char)75, (char)217, (char)83, (char)252, (char)91, (char)155, (char)28, (char)35, (char)160, (char)71, (char)130, (char)120, (char)52, (char)143, (char)220, (char)36, (char)104, (char)149, (char)11, (char)141, (char)72, (char)218, (char)33, (char)211, (char)7, (char)75, (char)49, (char)178}));
            assert(pack.sequence_GET() == (char)3083);
            assert(pack.first_message_offset_GET() == (char)130);
            assert(pack.target_component_GET() == (char)51);
            assert(pack.target_system_GET() == (char)55);
            assert(pack.length_GET() == (char)206);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.length_SET((char)206) ;
        p267.data__SET(new char[] {(char)168, (char)18, (char)175, (char)100, (char)173, (char)16, (char)69, (char)204, (char)157, (char)94, (char)255, (char)102, (char)124, (char)94, (char)211, (char)170, (char)253, (char)14, (char)84, (char)75, (char)255, (char)59, (char)90, (char)166, (char)12, (char)159, (char)230, (char)44, (char)49, (char)78, (char)192, (char)57, (char)20, (char)241, (char)221, (char)78, (char)43, (char)202, (char)183, (char)240, (char)125, (char)220, (char)202, (char)198, (char)57, (char)231, (char)251, (char)77, (char)250, (char)118, (char)195, (char)69, (char)66, (char)184, (char)111, (char)40, (char)169, (char)172, (char)66, (char)70, (char)31, (char)168, (char)95, (char)229, (char)129, (char)195, (char)61, (char)250, (char)48, (char)167, (char)2, (char)224, (char)188, (char)69, (char)12, (char)99, (char)3, (char)120, (char)147, (char)59, (char)68, (char)48, (char)143, (char)15, (char)151, (char)58, (char)54, (char)88, (char)228, (char)225, (char)162, (char)207, (char)48, (char)242, (char)39, (char)15, (char)16, (char)125, (char)45, (char)80, (char)252, (char)71, (char)58, (char)53, (char)196, (char)86, (char)23, (char)158, (char)196, (char)184, (char)109, (char)130, (char)198, (char)68, (char)160, (char)165, (char)66, (char)109, (char)42, (char)28, (char)109, (char)118, (char)236, (char)55, (char)185, (char)153, (char)74, (char)254, (char)44, (char)221, (char)113, (char)75, (char)209, (char)99, (char)115, (char)171, (char)110, (char)168, (char)73, (char)170, (char)133, (char)9, (char)135, (char)211, (char)66, (char)174, (char)196, (char)100, (char)128, (char)34, (char)104, (char)75, (char)21, (char)231, (char)123, (char)53, (char)182, (char)225, (char)246, (char)41, (char)16, (char)111, (char)34, (char)5, (char)244, (char)200, (char)0, (char)160, (char)254, (char)170, (char)92, (char)215, (char)165, (char)34, (char)37, (char)216, (char)236, (char)161, (char)10, (char)26, (char)67, (char)1, (char)109, (char)110, (char)101, (char)252, (char)3, (char)221, (char)109, (char)216, (char)72, (char)171, (char)43, (char)178, (char)67, (char)57, (char)131, (char)8, (char)205, (char)10, (char)30, (char)33, (char)12, (char)213, (char)225, (char)100, (char)73, (char)239, (char)96, (char)63, (char)234, (char)221, (char)217, (char)189, (char)15, (char)204, (char)6, (char)20, (char)194, (char)171, (char)171, (char)75, (char)217, (char)83, (char)252, (char)91, (char)155, (char)28, (char)35, (char)160, (char)71, (char)130, (char)120, (char)52, (char)143, (char)220, (char)36, (char)104, (char)149, (char)11, (char)141, (char)72, (char)218, (char)33, (char)211, (char)7, (char)75, (char)49, (char)178}, 0) ;
        p267.target_system_SET((char)55) ;
        p267.target_component_SET((char)51) ;
        p267.first_message_offset_SET((char)130) ;
        p267.sequence_SET((char)3083) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)10);
            assert(pack.sequence_GET() == (char)37696);
            assert(pack.target_component_GET() == (char)239);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_component_SET((char)239) ;
        p268.sequence_SET((char)37696) ;
        p268.target_system_SET((char)10) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.camera_id_GET() == (char)7);
            assert(pack.resolution_v_GET() == (char)41539);
            assert(pack.bitrate_GET() == 668505039L);
            assert(pack.framerate_GET() == -9.516261E37F);
            assert(pack.rotation_GET() == (char)36412);
            assert(pack.uri_LEN(ph) == 141);
            assert(pack.uri_TRY(ph).equals("xmqmsgqputklkeqxpcwepmjwezaBnrejnebsciwjtbvehyczyfcMmvcjwcezzyoglxxzHroqngVwCzfinynfIylXstsvkRpebvSzfMmreigxdirzCrlvquilzsbchSuymoquYuxqntaud"));
            assert(pack.status_GET() == (char)46);
            assert(pack.resolution_h_GET() == (char)28668);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.framerate_SET(-9.516261E37F) ;
        p269.camera_id_SET((char)7) ;
        p269.rotation_SET((char)36412) ;
        p269.resolution_h_SET((char)28668) ;
        p269.bitrate_SET(668505039L) ;
        p269.status_SET((char)46) ;
        p269.resolution_v_SET((char)41539) ;
        p269.uri_SET("xmqmsgqputklkeqxpcwepmjwezaBnrejnebsciwjtbvehyczyfcMmvcjwcezzyoglxxzHroqngVwCzfinynfIylXstsvkRpebvSzfMmreigxdirzCrlvquilzsbchSuymoquYuxqntaud", PH) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.rotation_GET() == (char)44010);
            assert(pack.framerate_GET() == 3.2259793E38F);
            assert(pack.resolution_v_GET() == (char)51397);
            assert(pack.resolution_h_GET() == (char)56357);
            assert(pack.target_component_GET() == (char)65);
            assert(pack.camera_id_GET() == (char)93);
            assert(pack.bitrate_GET() == 1486456693L);
            assert(pack.uri_LEN(ph) == 190);
            assert(pack.uri_TRY(ph).equals("uvYekciicomjnEsppiietshprhgqjzPmvvigpoKcxgklogqxBzsUnwmkxjzyvkwqsrrjcjxejxajppavnyumytuTpoLwmogovBhiatuzgmzasfWcejulzeqxvvsKmgrjexvodbadnfznYWpcgctCfdrrPcgfkmcxzkZazoqjgvfqxApQvadwfeoumpnutP"));
            assert(pack.target_system_GET() == (char)96);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.bitrate_SET(1486456693L) ;
        p270.target_system_SET((char)96) ;
        p270.framerate_SET(3.2259793E38F) ;
        p270.resolution_h_SET((char)56357) ;
        p270.uri_SET("uvYekciicomjnEsppiietshprhgqjzPmvvigpoKcxgklogqxBzsUnwmkxjzyvkwqsrrjcjxejxajppavnyumytuTpoLwmogovBhiatuzgmzasfWcejulzeqxvvsKmgrjexvodbadnfznYWpcgctCfdrrPcgfkmcxzkZazoqjgvfqxApQvadwfeoumpnutP", PH) ;
        p270.target_component_SET((char)65) ;
        p270.camera_id_SET((char)93) ;
        p270.rotation_SET((char)44010) ;
        p270.resolution_v_SET((char)51397) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 45);
            assert(pack.password_TRY(ph).equals("uglqipQsyqnkyyZsllsPanzGubfabmaSefxdbrniqtPdd"));
            assert(pack.ssid_LEN(ph) == 20);
            assert(pack.ssid_TRY(ph).equals("kwommqypchbIeqYfskGB"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("kwommqypchbIeqYfskGB", PH) ;
        p299.password_SET("uglqipQsyqnkyyZsllsPanzGubfabmaSefxdbrniqtPdd", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == (char)9826);
            assert(pack.max_version_GET() == (char)542);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)18, (char)46, (char)216, (char)225, (char)7, (char)196, (char)76, (char)147}));
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)177, (char)217, (char)57, (char)232, (char)121, (char)255, (char)120, (char)5}));
            assert(pack.min_version_GET() == (char)37446);
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.min_version_SET((char)37446) ;
        p300.spec_version_hash_SET(new char[] {(char)18, (char)46, (char)216, (char)225, (char)7, (char)196, (char)76, (char)147}, 0) ;
        p300.version_SET((char)9826) ;
        p300.library_version_hash_SET(new char[] {(char)177, (char)217, (char)57, (char)232, (char)121, (char)255, (char)120, (char)5}, 0) ;
        p300.max_version_SET((char)542) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.uptime_sec_GET() == 72883334L);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
            assert(pack.sub_mode_GET() == (char)124);
            assert(pack.vendor_specific_status_code_GET() == (char)6947);
            assert(pack.time_usec_GET() == 3494095871784578854L);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.uptime_sec_SET(72883334L) ;
        p310.vendor_specific_status_code_SET((char)6947) ;
        p310.sub_mode_SET((char)124) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE) ;
        p310.time_usec_SET(3494095871784578854L) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3566399234458296327L);
            assert(pack.uptime_sec_GET() == 2508438120L);
            assert(pack.name_LEN(ph) == 13);
            assert(pack.name_TRY(ph).equals("ubudzeZmnvoSt"));
            assert(pack.sw_vcs_commit_GET() == 3009853931L);
            assert(pack.sw_version_minor_GET() == (char)93);
            assert(pack.hw_version_minor_GET() == (char)214);
            assert(pack.hw_version_major_GET() == (char)60);
            assert(pack.sw_version_major_GET() == (char)116);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)145, (char)247, (char)54, (char)18, (char)148, (char)244, (char)72, (char)183, (char)111, (char)104, (char)184, (char)38, (char)240, (char)186, (char)217, (char)74}));
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.sw_vcs_commit_SET(3009853931L) ;
        p311.sw_version_major_SET((char)116) ;
        p311.name_SET("ubudzeZmnvoSt", PH) ;
        p311.hw_version_major_SET((char)60) ;
        p311.uptime_sec_SET(2508438120L) ;
        p311.hw_unique_id_SET(new char[] {(char)145, (char)247, (char)54, (char)18, (char)148, (char)244, (char)72, (char)183, (char)111, (char)104, (char)184, (char)38, (char)240, (char)186, (char)217, (char)74}, 0) ;
        p311.time_usec_SET(3566399234458296327L) ;
        p311.hw_version_minor_SET((char)214) ;
        p311.sw_version_minor_SET((char)93) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short) -13614);
            assert(pack.target_component_GET() == (char)167);
            assert(pack.target_system_GET() == (char)28);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("xmcu"));
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)28) ;
        p320.param_index_SET((short) -13614) ;
        p320.param_id_SET("xmcu", PH) ;
        p320.target_component_SET((char)167) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)12);
            assert(pack.target_component_GET() == (char)19);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)12) ;
        p321.target_component_SET((char)19) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)31908);
            assert(pack.param_value_LEN(ph) == 124);
            assert(pack.param_value_TRY(ph).equals("mqgtlusklrpWVbgdlsjxstmenmmuhhQksrycNsymucanixzahoihhiPcrbibjvwvtbzxioskcczmnmsEqmqjmvapemsAnWEymwqmkgtusswmtmuupDLkjTrfpgns"));
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("k"));
            assert(pack.param_index_GET() == (char)30272);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM) ;
        p322.param_id_SET("k", PH) ;
        p322.param_count_SET((char)31908) ;
        p322.param_index_SET((char)30272) ;
        p322.param_value_SET("mqgtlusklrpWVbgdlsjxstmenmmuhhQksrycNsymucanixzahoihhiPcrbibjvwvtbzxioskcczmnmsEqmqjmvapemsAnWEymwqmkgtusswmtmuupDLkjTrfpgns", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 45);
            assert(pack.param_value_TRY(ph).equals("fteplfvhviVjbbmPfvlypelsmoVftcmrzoeCblhuekuee"));
            assert(pack.target_system_GET() == (char)250);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
            assert(pack.target_component_GET() == (char)142);
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("wpJfCCdymjtsxwJ"));
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)250) ;
        p323.param_value_SET("fteplfvhviVjbbmPfvlypelsmoVftcmrzoeCblhuekuee", PH) ;
        p323.param_id_SET("wpJfCCdymjtsxwJ", PH) ;
        p323.target_component_SET((char)142) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_FAILED);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("bu"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
            assert(pack.param_value_LEN(ph) == 72);
            assert(pack.param_value_TRY(ph).equals("flediOMsiMqfglpzpxzyZxuxsxylxasydibsjszmtyasqhcdhzCxUufarzyzbvsdDtnmyvcq"));
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32) ;
        p324.param_value_SET("flediOMsiMqfglpzpxzyZxuxsxylxasydibsjszmtyasqhcdhzCxUufarzyzbvsdDtnmyvcq", PH) ;
        p324.param_id_SET("bu", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.min_distance_GET() == (char)46793);
            assert(pack.increment_GET() == (char)9);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(pack.time_usec_GET() == 593709196842447290L);
            assert(pack.max_distance_GET() == (char)9427);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)60284, (char)11941, (char)18175, (char)34833, (char)46709, (char)28394, (char)60177, (char)37030, (char)46137, (char)41528, (char)3090, (char)37701, (char)3537, (char)51146, (char)19251, (char)40334, (char)34631, (char)17542, (char)19809, (char)65465, (char)17172, (char)48599, (char)44152, (char)17519, (char)46850, (char)20332, (char)48550, (char)42131, (char)51217, (char)63505, (char)35647, (char)20937, (char)59472, (char)33827, (char)16703, (char)39629, (char)43803, (char)42918, (char)54051, (char)32057, (char)50983, (char)61441, (char)41889, (char)21805, (char)23011, (char)37884, (char)43815, (char)385, (char)620, (char)139, (char)62951, (char)24795, (char)18782, (char)52286, (char)48025, (char)63348, (char)36344, (char)55665, (char)15160, (char)44265, (char)26905, (char)50148, (char)14778, (char)23890, (char)42780, (char)26408, (char)34950, (char)63454, (char)34664, (char)49123, (char)40636, (char)49854}));
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(593709196842447290L) ;
        p330.distances_SET(new char[] {(char)60284, (char)11941, (char)18175, (char)34833, (char)46709, (char)28394, (char)60177, (char)37030, (char)46137, (char)41528, (char)3090, (char)37701, (char)3537, (char)51146, (char)19251, (char)40334, (char)34631, (char)17542, (char)19809, (char)65465, (char)17172, (char)48599, (char)44152, (char)17519, (char)46850, (char)20332, (char)48550, (char)42131, (char)51217, (char)63505, (char)35647, (char)20937, (char)59472, (char)33827, (char)16703, (char)39629, (char)43803, (char)42918, (char)54051, (char)32057, (char)50983, (char)61441, (char)41889, (char)21805, (char)23011, (char)37884, (char)43815, (char)385, (char)620, (char)139, (char)62951, (char)24795, (char)18782, (char)52286, (char)48025, (char)63348, (char)36344, (char)55665, (char)15160, (char)44265, (char)26905, (char)50148, (char)14778, (char)23890, (char)42780, (char)26408, (char)34950, (char)63454, (char)34664, (char)49123, (char)40636, (char)49854}, 0) ;
        p330.increment_SET((char)9) ;
        p330.max_distance_SET((char)9427) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p330.min_distance_SET((char)46793) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
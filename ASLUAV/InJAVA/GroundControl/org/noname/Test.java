
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
            long id = id__j(src);
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
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
            assert(pack.mavlink_version_GET() == (char)30);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_BOOT);
            assert(pack.custom_mode_GET() == 3781671082L);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_COAXIAL);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID) ;
        p0.custom_mode_SET(3781671082L) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_BOOT) ;
        p0.mavlink_version_SET((char)30) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_COAXIAL) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.current_battery_GET() == (short)26988);
            assert(pack.errors_count3_GET() == (char)25464);
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.load_GET() == (char)3852);
            assert(pack.errors_count2_GET() == (char)45639);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL));
            assert(pack.battery_remaining_GET() == (byte) - 122);
            assert(pack.errors_count1_GET() == (char)59150);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2));
            assert(pack.errors_comm_GET() == (char)2354);
            assert(pack.errors_count4_GET() == (char)42754);
            assert(pack.drop_rate_comm_GET() == (char)58290);
            assert(pack.voltage_battery_GET() == (char)5128);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.errors_count1_SET((char)59150) ;
        p1.load_SET((char)3852) ;
        p1.drop_rate_comm_SET((char)58290) ;
        p1.current_battery_SET((short)26988) ;
        p1.errors_count3_SET((char)25464) ;
        p1.errors_comm_SET((char)2354) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2)) ;
        p1.errors_count4_SET((char)42754) ;
        p1.voltage_battery_SET((char)5128) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL)) ;
        p1.battery_remaining_SET((byte) - 122) ;
        p1.errors_count2_SET((char)45639) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 6888342003828326024L);
            assert(pack.time_boot_ms_GET() == 157428803L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(6888342003828326024L) ;
        p2.time_boot_ms_SET(157428803L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == 3.054879E38F);
            assert(pack.afx_GET() == 2.990669E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.z_GET() == 1.0903005E38F);
            assert(pack.vz_GET() == 2.3318926E38F);
            assert(pack.vy_GET() == 3.0118833E38F);
            assert(pack.afy_GET() == -1.0565735E38F);
            assert(pack.type_mask_GET() == (char)61907);
            assert(pack.y_GET() == -6.78445E36F);
            assert(pack.afz_GET() == -2.5201273E38F);
            assert(pack.time_boot_ms_GET() == 1241450016L);
            assert(pack.yaw_GET() == -3.320738E38F);
            assert(pack.vx_GET() == 9.487548E37F);
            assert(pack.x_GET() == -2.3537878E38F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.time_boot_ms_SET(1241450016L) ;
        p3.afx_SET(2.990669E37F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p3.vy_SET(3.0118833E38F) ;
        p3.afz_SET(-2.5201273E38F) ;
        p3.yaw_SET(-3.320738E38F) ;
        p3.yaw_rate_SET(3.054879E38F) ;
        p3.vx_SET(9.487548E37F) ;
        p3.x_SET(-2.3537878E38F) ;
        p3.z_SET(1.0903005E38F) ;
        p3.afy_SET(-1.0565735E38F) ;
        p3.y_SET(-6.78445E36F) ;
        p3.type_mask_SET((char)61907) ;
        p3.vz_SET(2.3318926E38F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 3753108961L);
            assert(pack.time_usec_GET() == 708758144509607107L);
            assert(pack.target_component_GET() == (char)174);
            assert(pack.target_system_GET() == (char)47);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.seq_SET(3753108961L) ;
        p4.target_component_SET((char)174) ;
        p4.target_system_SET((char)47) ;
        p4.time_usec_SET(708758144509607107L) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.passkey_LEN(ph) == 24);
            assert(pack.passkey_TRY(ph).equals("BefjrsMorLlTJNpHlqbpszcg"));
            assert(pack.version_GET() == (char)100);
            assert(pack.target_system_GET() == (char)10);
            assert(pack.control_request_GET() == (char)127);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.passkey_SET("BefjrsMorLlTJNpHlqbpszcg", PH) ;
        p5.version_SET((char)100) ;
        p5.control_request_SET((char)127) ;
        p5.target_system_SET((char)10) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.ack_GET() == (char)173);
            assert(pack.gcs_system_id_GET() == (char)41);
            assert(pack.control_request_GET() == (char)82);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.control_request_SET((char)82) ;
        p6.gcs_system_id_SET((char)41) ;
        p6.ack_SET((char)173) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 17);
            assert(pack.key_TRY(ph).equals("reacpaxmMdpyqKsLn"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("reacpaxmMdpyqKsLn", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)131);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_PREFLIGHT);
            assert(pack.custom_mode_GET() == 3472798519L);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)131) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_PREFLIGHT) ;
        p11.custom_mode_SET(3472798519L) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)253);
            assert(pack.target_system_GET() == (char)36);
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("eBkEhxbko"));
            assert(pack.param_index_GET() == (short)22416);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)36) ;
        p20.param_index_SET((short)22416) ;
        p20.param_id_SET("eBkEhxbko", PH) ;
        p20.target_component_SET((char)253) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)217);
            assert(pack.target_component_GET() == (char)136);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)217) ;
        p21.target_component_SET((char)136) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)5051);
            assert(pack.param_index_GET() == (char)26774);
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("mMxpDnIjl"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
            assert(pack.param_value_GET() == -3.3765657E38F);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_index_SET((char)26774) ;
        p22.param_count_SET((char)5051) ;
        p22.param_id_SET("mMxpDnIjl", PH) ;
        p22.param_value_SET(-3.3765657E38F) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("Juau"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8);
            assert(pack.target_component_GET() == (char)10);
            assert(pack.param_value_GET() == 3.0406208E38F);
            assert(pack.target_system_GET() == (char)83);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.param_value_SET(3.0406208E38F) ;
        p23.target_component_SET((char)10) ;
        p23.param_id_SET("Juau", PH) ;
        p23.target_system_SET((char)83) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -1571212542);
            assert(pack.vel_acc_TRY(ph) == 3090199939L);
            assert(pack.h_acc_TRY(ph) == 2619713980L);
            assert(pack.alt_ellipsoid_TRY(ph) == 508103395);
            assert(pack.cog_GET() == (char)52624);
            assert(pack.v_acc_TRY(ph) == 2807711396L);
            assert(pack.lat_GET() == -258794771);
            assert(pack.satellites_visible_GET() == (char)49);
            assert(pack.time_usec_GET() == 4328932985492483060L);
            assert(pack.vel_GET() == (char)19733);
            assert(pack.epv_GET() == (char)4939);
            assert(pack.eph_GET() == (char)42544);
            assert(pack.lon_GET() == 1318435641);
            assert(pack.hdg_acc_TRY(ph) == 3847330421L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.h_acc_SET(2619713980L, PH) ;
        p24.time_usec_SET(4328932985492483060L) ;
        p24.lat_SET(-258794771) ;
        p24.v_acc_SET(2807711396L, PH) ;
        p24.satellites_visible_SET((char)49) ;
        p24.hdg_acc_SET(3847330421L, PH) ;
        p24.vel_SET((char)19733) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT) ;
        p24.alt_SET(-1571212542) ;
        p24.epv_SET((char)4939) ;
        p24.cog_SET((char)52624) ;
        p24.alt_ellipsoid_SET(508103395, PH) ;
        p24.vel_acc_SET(3090199939L, PH) ;
        p24.lon_SET(1318435641) ;
        p24.eph_SET((char)42544) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)123, (char)178, (char)252, (char)97, (char)186, (char)148, (char)198, (char)127, (char)159, (char)80, (char)146, (char)17, (char)112, (char)22, (char)52, (char)216, (char)116, (char)255, (char)16, (char)201}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)116, (char)213, (char)254, (char)222, (char)45, (char)127, (char)239, (char)98, (char)173, (char)34, (char)89, (char)140, (char)99, (char)0, (char)74, (char)234, (char)0, (char)138, (char)244, (char)225}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)82, (char)86, (char)197, (char)30, (char)80, (char)145, (char)129, (char)63, (char)151, (char)235, (char)202, (char)5, (char)190, (char)211, (char)5, (char)36, (char)217, (char)101, (char)31, (char)84}));
            assert(pack.satellites_visible_GET() == (char)151);
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)42, (char)247, (char)250, (char)106, (char)206, (char)84, (char)179, (char)80, (char)9, (char)217, (char)209, (char)241, (char)244, (char)163, (char)72, (char)197, (char)48, (char)102, (char)19, (char)9}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)140, (char)25, (char)51, (char)42, (char)2, (char)152, (char)5, (char)103, (char)3, (char)238, (char)200, (char)213, (char)62, (char)137, (char)196, (char)168, (char)150, (char)244, (char)97, (char)38}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_azimuth_SET(new char[] {(char)42, (char)247, (char)250, (char)106, (char)206, (char)84, (char)179, (char)80, (char)9, (char)217, (char)209, (char)241, (char)244, (char)163, (char)72, (char)197, (char)48, (char)102, (char)19, (char)9}, 0) ;
        p25.satellite_used_SET(new char[] {(char)140, (char)25, (char)51, (char)42, (char)2, (char)152, (char)5, (char)103, (char)3, (char)238, (char)200, (char)213, (char)62, (char)137, (char)196, (char)168, (char)150, (char)244, (char)97, (char)38}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)123, (char)178, (char)252, (char)97, (char)186, (char)148, (char)198, (char)127, (char)159, (char)80, (char)146, (char)17, (char)112, (char)22, (char)52, (char)216, (char)116, (char)255, (char)16, (char)201}, 0) ;
        p25.satellites_visible_SET((char)151) ;
        p25.satellite_prn_SET(new char[] {(char)82, (char)86, (char)197, (char)30, (char)80, (char)145, (char)129, (char)63, (char)151, (char)235, (char)202, (char)5, (char)190, (char)211, (char)5, (char)36, (char)217, (char)101, (char)31, (char)84}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)116, (char)213, (char)254, (char)222, (char)45, (char)127, (char)239, (char)98, (char)173, (char)34, (char)89, (char)140, (char)99, (char)0, (char)74, (char)234, (char)0, (char)138, (char)244, (char)225}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short)6187);
            assert(pack.xacc_GET() == (short)13971);
            assert(pack.zacc_GET() == (short)21975);
            assert(pack.xgyro_GET() == (short)27331);
            assert(pack.ygyro_GET() == (short)19106);
            assert(pack.zgyro_GET() == (short)6243);
            assert(pack.zmag_GET() == (short)24229);
            assert(pack.time_boot_ms_GET() == 453562644L);
            assert(pack.ymag_GET() == (short)12854);
            assert(pack.xmag_GET() == (short)4978);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.ymag_SET((short)12854) ;
        p26.xgyro_SET((short)27331) ;
        p26.zgyro_SET((short)6243) ;
        p26.zmag_SET((short)24229) ;
        p26.zacc_SET((short)21975) ;
        p26.xacc_SET((short)13971) ;
        p26.yacc_SET((short)6187) ;
        p26.xmag_SET((short)4978) ;
        p26.time_boot_ms_SET(453562644L) ;
        p26.ygyro_SET((short)19106) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short)31586);
            assert(pack.zmag_GET() == (short)31522);
            assert(pack.zacc_GET() == (short) -1043);
            assert(pack.xacc_GET() == (short)28320);
            assert(pack.zgyro_GET() == (short)24791);
            assert(pack.xgyro_GET() == (short) -25333);
            assert(pack.xmag_GET() == (short)32599);
            assert(pack.time_usec_GET() == 6621558590897765834L);
            assert(pack.ygyro_GET() == (short)13032);
            assert(pack.ymag_GET() == (short) -9186);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.xacc_SET((short)28320) ;
        p27.xgyro_SET((short) -25333) ;
        p27.yacc_SET((short)31586) ;
        p27.time_usec_SET(6621558590897765834L) ;
        p27.zacc_SET((short) -1043) ;
        p27.xmag_SET((short)32599) ;
        p27.ygyro_SET((short)13032) ;
        p27.zgyro_SET((short)24791) ;
        p27.ymag_SET((short) -9186) ;
        p27.zmag_SET((short)31522) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == (short) -21611);
            assert(pack.press_diff1_GET() == (short) -15588);
            assert(pack.time_usec_GET() == 7069502739240003670L);
            assert(pack.temperature_GET() == (short) -19614);
            assert(pack.press_diff2_GET() == (short)18760);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_diff2_SET((short)18760) ;
        p28.time_usec_SET(7069502739240003670L) ;
        p28.temperature_SET((short) -19614) ;
        p28.press_abs_SET((short) -21611) ;
        p28.press_diff1_SET((short) -15588) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 461421761L);
            assert(pack.press_abs_GET() == -9.133428E37F);
            assert(pack.press_diff_GET() == -1.3645098E38F);
            assert(pack.temperature_GET() == (short)10321);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.temperature_SET((short)10321) ;
        p29.time_boot_ms_SET(461421761L) ;
        p29.press_abs_SET(-9.133428E37F) ;
        p29.press_diff_SET(-1.3645098E38F) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -2.3089759E38F);
            assert(pack.yawspeed_GET() == -4.4265497E37F);
            assert(pack.pitch_GET() == 3.3376007E38F);
            assert(pack.yaw_GET() == -5.30886E37F);
            assert(pack.time_boot_ms_GET() == 1774107147L);
            assert(pack.pitchspeed_GET() == 2.5928487E38F);
            assert(pack.rollspeed_GET() == 1.8410278E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.pitch_SET(3.3376007E38F) ;
        p30.roll_SET(-2.3089759E38F) ;
        p30.yawspeed_SET(-4.4265497E37F) ;
        p30.time_boot_ms_SET(1774107147L) ;
        p30.rollspeed_SET(1.8410278E38F) ;
        p30.yaw_SET(-5.30886E37F) ;
        p30.pitchspeed_SET(2.5928487E38F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q1_GET() == -1.8507384E38F);
            assert(pack.q3_GET() == 2.8594488E38F);
            assert(pack.q2_GET() == 2.01617E38F);
            assert(pack.q4_GET() == 2.0882726E38F);
            assert(pack.time_boot_ms_GET() == 4093123960L);
            assert(pack.rollspeed_GET() == -2.294879E38F);
            assert(pack.pitchspeed_GET() == -3.3845768E38F);
            assert(pack.yawspeed_GET() == -1.3222305E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q4_SET(2.0882726E38F) ;
        p31.time_boot_ms_SET(4093123960L) ;
        p31.rollspeed_SET(-2.294879E38F) ;
        p31.q3_SET(2.8594488E38F) ;
        p31.pitchspeed_SET(-3.3845768E38F) ;
        p31.yawspeed_SET(-1.3222305E38F) ;
        p31.q2_SET(2.01617E38F) ;
        p31.q1_SET(-1.8507384E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 3.0830025E38F);
            assert(pack.vz_GET() == -1.7398657E38F);
            assert(pack.vy_GET() == -3.2793892E38F);
            assert(pack.vx_GET() == -2.462331E38F);
            assert(pack.y_GET() == -1.5561719E38F);
            assert(pack.x_GET() == 2.881696E38F);
            assert(pack.time_boot_ms_GET() == 2705388228L);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.vy_SET(-3.2793892E38F) ;
        p32.time_boot_ms_SET(2705388228L) ;
        p32.z_SET(3.0830025E38F) ;
        p32.y_SET(-1.5561719E38F) ;
        p32.x_SET(2.881696E38F) ;
        p32.vx_SET(-2.462331E38F) ;
        p32.vz_SET(-1.7398657E38F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.relative_alt_GET() == -1740661496);
            assert(pack.lat_GET() == 1584964447);
            assert(pack.vz_GET() == (short)28002);
            assert(pack.hdg_GET() == (char)24458);
            assert(pack.time_boot_ms_GET() == 4264438856L);
            assert(pack.vy_GET() == (short) -27034);
            assert(pack.vx_GET() == (short) -7731);
            assert(pack.alt_GET() == 1550415006);
            assert(pack.lon_GET() == -1058421047);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.vy_SET((short) -27034) ;
        p33.vx_SET((short) -7731) ;
        p33.lat_SET(1584964447) ;
        p33.vz_SET((short)28002) ;
        p33.hdg_SET((char)24458) ;
        p33.alt_SET(1550415006) ;
        p33.relative_alt_SET(-1740661496) ;
        p33.time_boot_ms_SET(4264438856L) ;
        p33.lon_SET(-1058421047) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan6_scaled_GET() == (short) -21882);
            assert(pack.chan5_scaled_GET() == (short) -15524);
            assert(pack.chan3_scaled_GET() == (short)13690);
            assert(pack.time_boot_ms_GET() == 3867240680L);
            assert(pack.rssi_GET() == (char)79);
            assert(pack.port_GET() == (char)227);
            assert(pack.chan1_scaled_GET() == (short)24673);
            assert(pack.chan2_scaled_GET() == (short)26825);
            assert(pack.chan7_scaled_GET() == (short) -18516);
            assert(pack.chan4_scaled_GET() == (short)9647);
            assert(pack.chan8_scaled_GET() == (short) -14028);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan4_scaled_SET((short)9647) ;
        p34.chan7_scaled_SET((short) -18516) ;
        p34.chan2_scaled_SET((short)26825) ;
        p34.rssi_SET((char)79) ;
        p34.chan3_scaled_SET((short)13690) ;
        p34.time_boot_ms_SET(3867240680L) ;
        p34.chan5_scaled_SET((short) -15524) ;
        p34.chan6_scaled_SET((short) -21882) ;
        p34.chan8_scaled_SET((short) -14028) ;
        p34.chan1_scaled_SET((short)24673) ;
        p34.port_SET((char)227) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 935666766L);
            assert(pack.rssi_GET() == (char)49);
            assert(pack.port_GET() == (char)62);
            assert(pack.chan6_raw_GET() == (char)28800);
            assert(pack.chan1_raw_GET() == (char)63531);
            assert(pack.chan7_raw_GET() == (char)31575);
            assert(pack.chan5_raw_GET() == (char)54773);
            assert(pack.chan2_raw_GET() == (char)54209);
            assert(pack.chan3_raw_GET() == (char)5524);
            assert(pack.chan4_raw_GET() == (char)14755);
            assert(pack.chan8_raw_GET() == (char)48703);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan2_raw_SET((char)54209) ;
        p35.chan3_raw_SET((char)5524) ;
        p35.chan6_raw_SET((char)28800) ;
        p35.chan7_raw_SET((char)31575) ;
        p35.chan1_raw_SET((char)63531) ;
        p35.port_SET((char)62) ;
        p35.chan8_raw_SET((char)48703) ;
        p35.chan4_raw_SET((char)14755) ;
        p35.rssi_SET((char)49) ;
        p35.time_boot_ms_SET(935666766L) ;
        p35.chan5_raw_SET((char)54773) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo6_raw_GET() == (char)18910);
            assert(pack.time_usec_GET() == 4151781311L);
            assert(pack.servo11_raw_TRY(ph) == (char)64337);
            assert(pack.port_GET() == (char)59);
            assert(pack.servo12_raw_TRY(ph) == (char)57740);
            assert(pack.servo13_raw_TRY(ph) == (char)23789);
            assert(pack.servo10_raw_TRY(ph) == (char)52183);
            assert(pack.servo14_raw_TRY(ph) == (char)6713);
            assert(pack.servo8_raw_GET() == (char)17076);
            assert(pack.servo5_raw_GET() == (char)161);
            assert(pack.servo9_raw_TRY(ph) == (char)15963);
            assert(pack.servo7_raw_GET() == (char)5244);
            assert(pack.servo15_raw_TRY(ph) == (char)31395);
            assert(pack.servo2_raw_GET() == (char)24784);
            assert(pack.servo4_raw_GET() == (char)39898);
            assert(pack.servo3_raw_GET() == (char)35823);
            assert(pack.servo1_raw_GET() == (char)15255);
            assert(pack.servo16_raw_TRY(ph) == (char)15057);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo8_raw_SET((char)17076) ;
        p36.servo6_raw_SET((char)18910) ;
        p36.servo11_raw_SET((char)64337, PH) ;
        p36.servo15_raw_SET((char)31395, PH) ;
        p36.servo9_raw_SET((char)15963, PH) ;
        p36.servo7_raw_SET((char)5244) ;
        p36.servo16_raw_SET((char)15057, PH) ;
        p36.time_usec_SET(4151781311L) ;
        p36.servo13_raw_SET((char)23789, PH) ;
        p36.servo12_raw_SET((char)57740, PH) ;
        p36.servo5_raw_SET((char)161) ;
        p36.servo1_raw_SET((char)15255) ;
        p36.servo2_raw_SET((char)24784) ;
        p36.servo14_raw_SET((char)6713, PH) ;
        p36.port_SET((char)59) ;
        p36.servo4_raw_SET((char)39898) ;
        p36.servo3_raw_SET((char)35823) ;
        p36.servo10_raw_SET((char)52183, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short)6079);
            assert(pack.target_system_GET() == (char)5);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)203);
            assert(pack.start_index_GET() == (short) -28910);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.start_index_SET((short) -28910) ;
        p37.target_component_SET((char)203) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p37.target_system_SET((char)5) ;
        p37.end_index_SET((short)6079) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short)30878);
            assert(pack.target_component_GET() == (char)182);
            assert(pack.end_index_GET() == (short)29354);
            assert(pack.target_system_GET() == (char)106);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p38.target_system_SET((char)106) ;
        p38.end_index_SET((short)29354) ;
        p38.target_component_SET((char)182) ;
        p38.start_index_SET((short)30878) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 2.1245976E38F);
            assert(pack.param2_GET() == -1.0121441E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.param4_GET() == 1.6642785E38F);
            assert(pack.param3_GET() == 5.93982E37F);
            assert(pack.x_GET() == -3.3954995E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT);
            assert(pack.current_GET() == (char)73);
            assert(pack.z_GET() == 3.0513966E38F);
            assert(pack.autocontinue_GET() == (char)121);
            assert(pack.target_system_GET() == (char)213);
            assert(pack.seq_GET() == (char)9329);
            assert(pack.param1_GET() == 3.8222465E37F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)171);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.x_SET(-3.3954995E38F) ;
        p39.target_component_SET((char)171) ;
        p39.target_system_SET((char)213) ;
        p39.param1_SET(3.8222465E37F) ;
        p39.y_SET(2.1245976E38F) ;
        p39.current_SET((char)73) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p39.autocontinue_SET((char)121) ;
        p39.param4_SET(1.6642785E38F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p39.param3_SET(5.93982E37F) ;
        p39.seq_SET((char)9329) ;
        p39.param2_SET(-1.0121441E38F) ;
        p39.z_SET(3.0513966E38F) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)5379);
            assert(pack.target_system_GET() == (char)223);
            assert(pack.target_component_GET() == (char)20);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_component_SET((char)20) ;
        p40.seq_SET((char)5379) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p40.target_system_SET((char)223) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)106);
            assert(pack.target_system_GET() == (char)107);
            assert(pack.seq_GET() == (char)40347);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.seq_SET((char)40347) ;
        p41.target_system_SET((char)107) ;
        p41.target_component_SET((char)106) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)53143);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)53143) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)142);
            assert(pack.target_system_GET() == (char)90);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)90) ;
        p43.target_component_SET((char)142) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)254);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.count_GET() == (char)5878);
            assert(pack.target_component_GET() == (char)70);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_component_SET((char)70) ;
        p44.target_system_SET((char)254) ;
        p44.count_SET((char)5878) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)244);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)126);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p45.target_system_SET((char)244) ;
        p45.target_component_SET((char)126) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)9363);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)9363) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)32);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED_FRAME);
            assert(pack.target_system_GET() == (char)252);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED_FRAME) ;
        p47.target_system_SET((char)252) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p47.target_component_SET((char)32) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 2612609940859557691L);
            assert(pack.longitude_GET() == -1875576396);
            assert(pack.altitude_GET() == -971068558);
            assert(pack.target_system_GET() == (char)254);
            assert(pack.latitude_GET() == -1394870312);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.target_system_SET((char)254) ;
        p48.altitude_SET(-971068558) ;
        p48.longitude_SET(-1875576396) ;
        p48.time_usec_SET(2612609940859557691L, PH) ;
        p48.latitude_SET(-1394870312) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 531205397);
            assert(pack.time_usec_TRY(ph) == 2365863775667376793L);
            assert(pack.longitude_GET() == -1791431889);
            assert(pack.latitude_GET() == -548134013);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.time_usec_SET(2365863775667376793L, PH) ;
        p49.latitude_SET(-548134013) ;
        p49.altitude_SET(531205397) ;
        p49.longitude_SET(-1791431889) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_value0_GET() == -2.4288382E38F);
            assert(pack.scale_GET() == -1.2164125E38F);
            assert(pack.target_component_GET() == (char)151);
            assert(pack.param_value_min_GET() == -1.5627057E38F);
            assert(pack.param_value_max_GET() == -2.5914894E38F);
            assert(pack.param_index_GET() == (short) -21684);
            assert(pack.parameter_rc_channel_index_GET() == (char)160);
            assert(pack.target_system_GET() == (char)93);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("p"));
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.parameter_rc_channel_index_SET((char)160) ;
        p50.param_value_min_SET(-1.5627057E38F) ;
        p50.param_value0_SET(-2.4288382E38F) ;
        p50.scale_SET(-1.2164125E38F) ;
        p50.target_component_SET((char)151) ;
        p50.target_system_SET((char)93) ;
        p50.param_index_SET((short) -21684) ;
        p50.param_value_max_SET(-2.5914894E38F) ;
        p50.param_id_SET("p", PH) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)27);
            assert(pack.seq_GET() == (char)63516);
            assert(pack.target_component_GET() == (char)105);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.seq_SET((char)63516) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p51.target_system_SET((char)27) ;
        p51.target_component_SET((char)105) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2x_GET() == -3.071586E36F);
            assert(pack.target_component_GET() == (char)121);
            assert(pack.p1z_GET() == -2.9481966E37F);
            assert(pack.p2y_GET() == -2.2743568E38F);
            assert(pack.p1y_GET() == -2.6738493E38F);
            assert(pack.p2z_GET() == -2.9524578E38F);
            assert(pack.target_system_GET() == (char)248);
            assert(pack.p1x_GET() == 1.4604622E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p2x_SET(-3.071586E36F) ;
        p54.p2y_SET(-2.2743568E38F) ;
        p54.p1x_SET(1.4604622E38F) ;
        p54.target_component_SET((char)121) ;
        p54.p1z_SET(-2.9481966E37F) ;
        p54.p1y_SET(-2.6738493E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p54.target_system_SET((char)248) ;
        p54.p2z_SET(-2.9524578E38F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2y_GET() == -2.9378614E38F);
            assert(pack.p1z_GET() == 3.2120303E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.p2z_GET() == -2.9164535E38F);
            assert(pack.p1y_GET() == -7.5021885E37F);
            assert(pack.p1x_GET() == 1.21999E37F);
            assert(pack.p2x_GET() == -2.8510186E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2z_SET(-2.9164535E38F) ;
        p55.p1z_SET(3.2120303E38F) ;
        p55.p2y_SET(-2.9378614E38F) ;
        p55.p2x_SET(-2.8510186E38F) ;
        p55.p1x_SET(1.21999E37F) ;
        p55.p1y_SET(-7.5021885E37F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == -2.1060855E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.1942624E38F, 1.8045718E38F, -1.4862074E38F, -1.005003E38F}));
            assert(pack.time_usec_GET() == 8237530597712640922L);
            assert(pack.yawspeed_GET() == 6.520367E37F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.6271367E38F, 2.9936167E38F, 7.9132866E37F, 2.0303457E38F, -4.7208145E37F, 1.3110544E38F, 2.6200515E38F, -5.0677385E37F, -3.9674434E37F}));
            assert(pack.pitchspeed_GET() == 3.3165812E38F);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.covariance_SET(new float[] {1.6271367E38F, 2.9936167E38F, 7.9132866E37F, 2.0303457E38F, -4.7208145E37F, 1.3110544E38F, 2.6200515E38F, -5.0677385E37F, -3.9674434E37F}, 0) ;
        p61.q_SET(new float[] {2.1942624E38F, 1.8045718E38F, -1.4862074E38F, -1.005003E38F}, 0) ;
        p61.time_usec_SET(8237530597712640922L) ;
        p61.rollspeed_SET(-2.1060855E37F) ;
        p61.pitchspeed_SET(3.3165812E38F) ;
        p61.yawspeed_SET(6.520367E37F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.xtrack_error_GET() == -3.2901036E38F);
            assert(pack.wp_dist_GET() == (char)21005);
            assert(pack.alt_error_GET() == -2.432234E38F);
            assert(pack.target_bearing_GET() == (short) -13341);
            assert(pack.nav_bearing_GET() == (short) -15719);
            assert(pack.aspd_error_GET() == 9.947135E37F);
            assert(pack.nav_pitch_GET() == 9.507311E37F);
            assert(pack.nav_roll_GET() == -1.8238782E38F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_pitch_SET(9.507311E37F) ;
        p62.alt_error_SET(-2.432234E38F) ;
        p62.xtrack_error_SET(-3.2901036E38F) ;
        p62.wp_dist_SET((char)21005) ;
        p62.nav_roll_SET(-1.8238782E38F) ;
        p62.target_bearing_SET((short) -13341) ;
        p62.nav_bearing_SET((short) -15719) ;
        p62.aspd_error_SET(9.947135E37F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 1711430734);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(pack.vy_GET() == 2.0565717E36F);
            assert(pack.vz_GET() == -3.3428606E37F);
            assert(pack.vx_GET() == 3.5507836E37F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {5.7789676E37F, 2.0243988E37F, -9.87114E37F, 2.9200348E38F, -1.2103943E38F, 1.4817308E38F, 8.557455E37F, 7.869328E37F, -1.406539E38F, -3.0156907E38F, -1.9929457E38F, 1.5551314E38F, 8.300119E37F, -1.1303254E38F, -8.012082E37F, -2.1795735E38F, -3.5288023E37F, -1.9008666E38F, -1.7283286E38F, -2.9797584E38F, 2.8375272E38F, 2.079117E38F, -2.0567404E38F, -1.9091007E38F, -1.5591115E38F, -2.7701236E37F, -6.6266644E36F, 1.1923501E38F, -1.1790909E38F, -3.29563E38F, 2.9192227E38F, 3.29195E38F, -1.2334624E38F, -2.7936616E38F, 2.7710589E38F, 3.3894188E38F}));
            assert(pack.alt_GET() == -929908905);
            assert(pack.lat_GET() == -1691765944);
            assert(pack.relative_alt_GET() == -1522736273);
            assert(pack.time_usec_GET() == 8330999618143749506L);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.lat_SET(-1691765944) ;
        p63.covariance_SET(new float[] {5.7789676E37F, 2.0243988E37F, -9.87114E37F, 2.9200348E38F, -1.2103943E38F, 1.4817308E38F, 8.557455E37F, 7.869328E37F, -1.406539E38F, -3.0156907E38F, -1.9929457E38F, 1.5551314E38F, 8.300119E37F, -1.1303254E38F, -8.012082E37F, -2.1795735E38F, -3.5288023E37F, -1.9008666E38F, -1.7283286E38F, -2.9797584E38F, 2.8375272E38F, 2.079117E38F, -2.0567404E38F, -1.9091007E38F, -1.5591115E38F, -2.7701236E37F, -6.6266644E36F, 1.1923501E38F, -1.1790909E38F, -3.29563E38F, 2.9192227E38F, 3.29195E38F, -1.2334624E38F, -2.7936616E38F, 2.7710589E38F, 3.3894188E38F}, 0) ;
        p63.vy_SET(2.0565717E36F) ;
        p63.alt_SET(-929908905) ;
        p63.vz_SET(-3.3428606E37F) ;
        p63.lon_SET(1711430734) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p63.vx_SET(3.5507836E37F) ;
        p63.relative_alt_SET(-1522736273) ;
        p63.time_usec_SET(8330999618143749506L) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.az_GET() == 4.3539767E37F);
            assert(pack.z_GET() == -2.253682E38F);
            assert(pack.x_GET() == 5.9214165E37F);
            assert(pack.vy_GET() == -1.4718452E38F);
            assert(pack.vx_GET() == -3.0148966E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.1620601E38F, 1.0547993E38F, -2.1058944E38F, 2.5237015E38F, -1.0066293E38F, 2.3523417E38F, 1.9548531E38F, 7.4959045E37F, 1.8766829E37F, 2.9894749E38F, 1.2751277E38F, -3.359829E38F, -1.9604592E38F, 2.161042E38F, 1.391368E38F, 1.8417698E38F, -8.711711E37F, -2.153542E38F, 1.4574193E38F, 3.2611932E38F, 2.0314845E38F, -2.456967E38F, 4.1959833E37F, -2.5361581E38F, -1.2355243E38F, 1.0948799E38F, 3.0752157E38F, -5.682626E37F, -5.5219104E37F, 1.5136235E38F, -1.0313717E38F, 6.652226E37F, -3.305049E38F, 9.740341E37F, -2.2593694E37F, 9.717156E37F, -1.9484118E38F, -6.277699E37F, 2.7437303E38F, -4.147419E37F, 2.2070777E38F, 2.8680678E38F, -7.234106E36F, 2.407796E38F, -3.0841957E38F}));
            assert(pack.ay_GET() == 2.7645314E38F);
            assert(pack.vz_GET() == 2.4077847E38F);
            assert(pack.ax_GET() == -3.380895E38F);
            assert(pack.y_GET() == -6.5287317E37F);
            assert(pack.time_usec_GET() == 6853132635322957533L);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.ax_SET(-3.380895E38F) ;
        p64.vy_SET(-1.4718452E38F) ;
        p64.z_SET(-2.253682E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p64.vx_SET(-3.0148966E38F) ;
        p64.time_usec_SET(6853132635322957533L) ;
        p64.ay_SET(2.7645314E38F) ;
        p64.y_SET(-6.5287317E37F) ;
        p64.az_SET(4.3539767E37F) ;
        p64.x_SET(5.9214165E37F) ;
        p64.vz_SET(2.4077847E38F) ;
        p64.covariance_SET(new float[] {-1.1620601E38F, 1.0547993E38F, -2.1058944E38F, 2.5237015E38F, -1.0066293E38F, 2.3523417E38F, 1.9548531E38F, 7.4959045E37F, 1.8766829E37F, 2.9894749E38F, 1.2751277E38F, -3.359829E38F, -1.9604592E38F, 2.161042E38F, 1.391368E38F, 1.8417698E38F, -8.711711E37F, -2.153542E38F, 1.4574193E38F, 3.2611932E38F, 2.0314845E38F, -2.456967E38F, 4.1959833E37F, -2.5361581E38F, -1.2355243E38F, 1.0948799E38F, 3.0752157E38F, -5.682626E37F, -5.5219104E37F, 1.5136235E38F, -1.0313717E38F, 6.652226E37F, -3.305049E38F, 9.740341E37F, -2.2593694E37F, 9.717156E37F, -1.9484118E38F, -6.277699E37F, 2.7437303E38F, -4.147419E37F, 2.2070777E38F, 2.8680678E38F, -7.234106E36F, 2.407796E38F, -3.0841957E38F}, 0) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan10_raw_GET() == (char)10151);
            assert(pack.time_boot_ms_GET() == 2074509347L);
            assert(pack.chan1_raw_GET() == (char)1970);
            assert(pack.chan18_raw_GET() == (char)54076);
            assert(pack.chan11_raw_GET() == (char)38009);
            assert(pack.chan13_raw_GET() == (char)35256);
            assert(pack.chan17_raw_GET() == (char)39587);
            assert(pack.chan12_raw_GET() == (char)8813);
            assert(pack.chan9_raw_GET() == (char)42840);
            assert(pack.chan14_raw_GET() == (char)22952);
            assert(pack.chan8_raw_GET() == (char)5219);
            assert(pack.chan4_raw_GET() == (char)21606);
            assert(pack.chan5_raw_GET() == (char)55626);
            assert(pack.chan3_raw_GET() == (char)9320);
            assert(pack.chan6_raw_GET() == (char)5114);
            assert(pack.chan16_raw_GET() == (char)52571);
            assert(pack.chan2_raw_GET() == (char)1664);
            assert(pack.rssi_GET() == (char)91);
            assert(pack.chancount_GET() == (char)71);
            assert(pack.chan7_raw_GET() == (char)34026);
            assert(pack.chan15_raw_GET() == (char)40769);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan17_raw_SET((char)39587) ;
        p65.chan8_raw_SET((char)5219) ;
        p65.chancount_SET((char)71) ;
        p65.chan4_raw_SET((char)21606) ;
        p65.time_boot_ms_SET(2074509347L) ;
        p65.chan12_raw_SET((char)8813) ;
        p65.chan13_raw_SET((char)35256) ;
        p65.chan15_raw_SET((char)40769) ;
        p65.chan1_raw_SET((char)1970) ;
        p65.chan3_raw_SET((char)9320) ;
        p65.chan2_raw_SET((char)1664) ;
        p65.chan6_raw_SET((char)5114) ;
        p65.rssi_SET((char)91) ;
        p65.chan10_raw_SET((char)10151) ;
        p65.chan14_raw_SET((char)22952) ;
        p65.chan18_raw_SET((char)54076) ;
        p65.chan9_raw_SET((char)42840) ;
        p65.chan5_raw_SET((char)55626) ;
        p65.chan7_raw_SET((char)34026) ;
        p65.chan11_raw_SET((char)38009) ;
        p65.chan16_raw_SET((char)52571) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)114);
            assert(pack.start_stop_GET() == (char)213);
            assert(pack.req_message_rate_GET() == (char)2123);
            assert(pack.target_component_GET() == (char)190);
            assert(pack.req_stream_id_GET() == (char)153);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_message_rate_SET((char)2123) ;
        p66.target_system_SET((char)114) ;
        p66.req_stream_id_SET((char)153) ;
        p66.target_component_SET((char)190) ;
        p66.start_stop_SET((char)213) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.message_rate_GET() == (char)14668);
            assert(pack.on_off_GET() == (char)196);
            assert(pack.stream_id_GET() == (char)196);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)196) ;
        p67.on_off_SET((char)196) ;
        p67.message_rate_SET((char)14668) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.r_GET() == (short)17203);
            assert(pack.buttons_GET() == (char)65063);
            assert(pack.target_GET() == (char)226);
            assert(pack.y_GET() == (short) -30275);
            assert(pack.z_GET() == (short)17224);
            assert(pack.x_GET() == (short)21633);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.r_SET((short)17203) ;
        p69.target_SET((char)226) ;
        p69.buttons_SET((char)65063) ;
        p69.x_SET((short)21633) ;
        p69.y_SET((short) -30275) ;
        p69.z_SET((short)17224) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan5_raw_GET() == (char)17632);
            assert(pack.chan1_raw_GET() == (char)50609);
            assert(pack.chan2_raw_GET() == (char)47245);
            assert(pack.chan8_raw_GET() == (char)11565);
            assert(pack.target_component_GET() == (char)133);
            assert(pack.chan4_raw_GET() == (char)13594);
            assert(pack.chan3_raw_GET() == (char)4797);
            assert(pack.chan6_raw_GET() == (char)8642);
            assert(pack.target_system_GET() == (char)42);
            assert(pack.chan7_raw_GET() == (char)55554);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan8_raw_SET((char)11565) ;
        p70.chan6_raw_SET((char)8642) ;
        p70.chan4_raw_SET((char)13594) ;
        p70.target_component_SET((char)133) ;
        p70.chan3_raw_SET((char)4797) ;
        p70.chan2_raw_SET((char)47245) ;
        p70.target_system_SET((char)42) ;
        p70.chan1_raw_SET((char)50609) ;
        p70.chan5_raw_SET((char)17632) ;
        p70.chan7_raw_SET((char)55554) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param3_GET() == -2.9520422E38F);
            assert(pack.y_GET() == -382060789);
            assert(pack.autocontinue_GET() == (char)137);
            assert(pack.target_system_GET() == (char)164);
            assert(pack.z_GET() == -1.3013925E38F);
            assert(pack.current_GET() == (char)250);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_VIDEO_START_STREAMING);
            assert(pack.param1_GET() == 2.845294E38F);
            assert(pack.x_GET() == -463512213);
            assert(pack.target_component_GET() == (char)222);
            assert(pack.seq_GET() == (char)40976);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.param2_GET() == 9.240065E37F);
            assert(pack.param4_GET() == -1.2576755E38F);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.z_SET(-1.3013925E38F) ;
        p73.command_SET(MAV_CMD.MAV_CMD_VIDEO_START_STREAMING) ;
        p73.target_system_SET((char)164) ;
        p73.seq_SET((char)40976) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p73.x_SET(-463512213) ;
        p73.param4_SET(-1.2576755E38F) ;
        p73.current_SET((char)250) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p73.autocontinue_SET((char)137) ;
        p73.param2_SET(9.240065E37F) ;
        p73.y_SET(-382060789) ;
        p73.param3_SET(-2.9520422E38F) ;
        p73.target_component_SET((char)222) ;
        p73.param1_SET(2.845294E38F) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.groundspeed_GET() == -2.1656612E38F);
            assert(pack.alt_GET() == -1.06315574E37F);
            assert(pack.throttle_GET() == (char)39546);
            assert(pack.climb_GET() == 1.1417978E38F);
            assert(pack.airspeed_GET() == -1.1594144E38F);
            assert(pack.heading_GET() == (short) -2789);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.throttle_SET((char)39546) ;
        p74.heading_SET((short) -2789) ;
        p74.airspeed_SET(-1.1594144E38F) ;
        p74.climb_SET(1.1417978E38F) ;
        p74.groundspeed_SET(-2.1656612E38F) ;
        p74.alt_SET(-1.06315574E37F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 235042163);
            assert(pack.y_GET() == -1031163306);
            assert(pack.target_component_GET() == (char)67);
            assert(pack.target_system_GET() == (char)172);
            assert(pack.param1_GET() == -4.11579E37F);
            assert(pack.autocontinue_GET() == (char)125);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS);
            assert(pack.z_GET() == 1.3803261E38F);
            assert(pack.current_GET() == (char)100);
            assert(pack.param3_GET() == -2.8031917E38F);
            assert(pack.param2_GET() == -1.6967697E37F);
            assert(pack.param4_GET() == 1.6420213E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.z_SET(1.3803261E38F) ;
        p75.target_component_SET((char)67) ;
        p75.current_SET((char)100) ;
        p75.command_SET(MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p75.param2_SET(-1.6967697E37F) ;
        p75.autocontinue_SET((char)125) ;
        p75.param4_SET(1.6420213E38F) ;
        p75.y_SET(-1031163306) ;
        p75.param1_SET(-4.11579E37F) ;
        p75.x_SET(235042163) ;
        p75.target_system_SET((char)172) ;
        p75.param3_SET(-2.8031917E38F) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)0);
            assert(pack.param7_GET() == -2.8307505E37F);
            assert(pack.param2_GET() == -9.198711E37F);
            assert(pack.param6_GET() == -4.865446E37F);
            assert(pack.confirmation_GET() == (char)245);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_HOME);
            assert(pack.param4_GET() == 6.07207E37F);
            assert(pack.param1_GET() == -2.3338825E38F);
            assert(pack.param3_GET() == -2.1038469E38F);
            assert(pack.target_component_GET() == (char)33);
            assert(pack.param5_GET() == -2.9194005E38F);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.param6_SET(-4.865446E37F) ;
        p76.param1_SET(-2.3338825E38F) ;
        p76.target_system_SET((char)0) ;
        p76.confirmation_SET((char)245) ;
        p76.param7_SET(-2.8307505E37F) ;
        p76.param4_SET(6.07207E37F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_DO_SET_HOME) ;
        p76.param3_SET(-2.1038469E38F) ;
        p76.param2_SET(-9.198711E37F) ;
        p76.target_component_SET((char)33) ;
        p76.param5_SET(-2.9194005E38F) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_TRY(ph) == (char)174);
            assert(pack.result_param2_TRY(ph) == 1185638436);
            assert(pack.progress_TRY(ph) == (char)212);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_ACCEPTED);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS);
            assert(pack.target_component_TRY(ph) == (char)36);
        });
        COMMAND_ACK p77 = new COMMAND_ACK();
        PH.setPack(p77);
        p77.progress_SET((char)212, PH) ;
        p77.target_component_SET((char)36, PH) ;
        p77.result_param2_SET(1185638436, PH) ;
        p77.target_system_SET((char)174, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_ACCEPTED) ;
        p77.command_SET(MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS) ;
        TestChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.mode_switch_GET() == (char)234);
            assert(pack.pitch_GET() == 2.8202308E35F);
            assert(pack.time_boot_ms_GET() == 1603989533L);
            assert(pack.roll_GET() == 1.1818006E38F);
            assert(pack.manual_override_switch_GET() == (char)151);
            assert(pack.yaw_GET() == 3.1094443E38F);
            assert(pack.thrust_GET() == -9.012845E37F);
        });
        MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.manual_override_switch_SET((char)151) ;
        p81.time_boot_ms_SET(1603989533L) ;
        p81.yaw_SET(3.1094443E38F) ;
        p81.pitch_SET(2.8202308E35F) ;
        p81.thrust_SET(-9.012845E37F) ;
        p81.roll_SET(1.1818006E38F) ;
        p81.mode_switch_SET((char)234) ;
        TestChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-8.978124E37F, -3.4017394E38F, -1.9333935E38F, 5.643354E37F}));
            assert(pack.thrust_GET() == -2.7500935E38F);
            assert(pack.target_system_GET() == (char)229);
            assert(pack.body_roll_rate_GET() == -3.3640443E38F);
            assert(pack.type_mask_GET() == (char)211);
            assert(pack.body_yaw_rate_GET() == 1.6090844E38F);
            assert(pack.body_pitch_rate_GET() == -2.5815212E38F);
            assert(pack.target_component_GET() == (char)16);
            assert(pack.time_boot_ms_GET() == 761273301L);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.body_yaw_rate_SET(1.6090844E38F) ;
        p82.body_roll_rate_SET(-3.3640443E38F) ;
        p82.q_SET(new float[] {-8.978124E37F, -3.4017394E38F, -1.9333935E38F, 5.643354E37F}, 0) ;
        p82.type_mask_SET((char)211) ;
        p82.body_pitch_rate_SET(-2.5815212E38F) ;
        p82.time_boot_ms_SET(761273301L) ;
        p82.target_system_SET((char)229) ;
        p82.target_component_SET((char)16) ;
        p82.thrust_SET(-2.7500935E38F) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_yaw_rate_GET() == 2.134433E38F);
            assert(pack.type_mask_GET() == (char)154);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.2762447E38F, 8.233567E36F, -6.070316E37F, 1.3800882E38F}));
            assert(pack.body_roll_rate_GET() == 1.3396571E38F);
            assert(pack.thrust_GET() == -7.9673224E37F);
            assert(pack.time_boot_ms_GET() == 562902675L);
            assert(pack.body_pitch_rate_GET() == -9.218282E37F);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_yaw_rate_SET(2.134433E38F) ;
        p83.body_pitch_rate_SET(-9.218282E37F) ;
        p83.q_SET(new float[] {-2.2762447E38F, 8.233567E36F, -6.070316E37F, 1.3800882E38F}, 0) ;
        p83.thrust_SET(-7.9673224E37F) ;
        p83.body_roll_rate_SET(1.3396571E38F) ;
        p83.time_boot_ms_SET(562902675L) ;
        p83.type_mask_SET((char)154) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1968867325L);
            assert(pack.afx_GET() == -7.936904E37F);
            assert(pack.target_component_GET() == (char)199);
            assert(pack.target_system_GET() == (char)233);
            assert(pack.vx_GET() == 3.0191636E38F);
            assert(pack.afy_GET() == 1.2103468E38F);
            assert(pack.y_GET() == 1.8106308E38F);
            assert(pack.x_GET() == 1.4323459E38F);
            assert(pack.yaw_rate_GET() == 1.3195135E38F);
            assert(pack.vz_GET() == 3.2882646E37F);
            assert(pack.z_GET() == 1.9176633E38F);
            assert(pack.afz_GET() == -2.9065845E38F);
            assert(pack.yaw_GET() == 1.3208589E38F);
            assert(pack.type_mask_GET() == (char)18436);
            assert(pack.vy_GET() == 1.7448584E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.vx_SET(3.0191636E38F) ;
        p84.target_system_SET((char)233) ;
        p84.z_SET(1.9176633E38F) ;
        p84.yaw_rate_SET(1.3195135E38F) ;
        p84.afz_SET(-2.9065845E38F) ;
        p84.afy_SET(1.2103468E38F) ;
        p84.time_boot_ms_SET(1968867325L) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p84.target_component_SET((char)199) ;
        p84.type_mask_SET((char)18436) ;
        p84.vz_SET(3.2882646E37F) ;
        p84.yaw_SET(1.3208589E38F) ;
        p84.x_SET(1.4323459E38F) ;
        p84.y_SET(1.8106308E38F) ;
        p84.afx_SET(-7.936904E37F) ;
        p84.vy_SET(1.7448584E38F) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)142);
            assert(pack.vx_GET() == -1.0088332E38F);
            assert(pack.vy_GET() == 2.854914E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.lon_int_GET() == 1207296761);
            assert(pack.time_boot_ms_GET() == 3224869021L);
            assert(pack.yaw_GET() == -2.792192E38F);
            assert(pack.afy_GET() == -2.984206E38F);
            assert(pack.yaw_rate_GET() == -1.7810619E38F);
            assert(pack.type_mask_GET() == (char)24958);
            assert(pack.vz_GET() == -1.3998475E38F);
            assert(pack.afx_GET() == -2.4785364E38F);
            assert(pack.lat_int_GET() == -1253789565);
            assert(pack.target_component_GET() == (char)225);
            assert(pack.afz_GET() == 1.3286049E38F);
            assert(pack.alt_GET() == 5.5751405E37F);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.afx_SET(-2.4785364E38F) ;
        p86.target_component_SET((char)225) ;
        p86.lat_int_SET(-1253789565) ;
        p86.type_mask_SET((char)24958) ;
        p86.yaw_SET(-2.792192E38F) ;
        p86.vz_SET(-1.3998475E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p86.alt_SET(5.5751405E37F) ;
        p86.lon_int_SET(1207296761) ;
        p86.afz_SET(1.3286049E38F) ;
        p86.afy_SET(-2.984206E38F) ;
        p86.yaw_rate_SET(-1.7810619E38F) ;
        p86.time_boot_ms_SET(3224869021L) ;
        p86.vx_SET(-1.0088332E38F) ;
        p86.target_system_SET((char)142) ;
        p86.vy_SET(2.854914E37F) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -6.5876953E35F);
            assert(pack.afy_GET() == 2.051192E38F);
            assert(pack.vz_GET() == -1.7115703E38F);
            assert(pack.lon_int_GET() == -1000161646);
            assert(pack.afx_GET() == -5.2956286E37F);
            assert(pack.yaw_GET() == 3.1067855E37F);
            assert(pack.vx_GET() == 4.260918E37F);
            assert(pack.type_mask_GET() == (char)3956);
            assert(pack.time_boot_ms_GET() == 2524323739L);
            assert(pack.vy_GET() == -2.4316066E38F);
            assert(pack.yaw_rate_GET() == 1.4880831E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.afz_GET() == 4.459009E36F);
            assert(pack.lat_int_GET() == 1519580233);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p87.alt_SET(-6.5876953E35F) ;
        p87.lon_int_SET(-1000161646) ;
        p87.afy_SET(2.051192E38F) ;
        p87.vx_SET(4.260918E37F) ;
        p87.type_mask_SET((char)3956) ;
        p87.lat_int_SET(1519580233) ;
        p87.time_boot_ms_SET(2524323739L) ;
        p87.yaw_SET(3.1067855E37F) ;
        p87.afx_SET(-5.2956286E37F) ;
        p87.yaw_rate_SET(1.4880831E37F) ;
        p87.vz_SET(-1.7115703E38F) ;
        p87.vy_SET(-2.4316066E38F) ;
        p87.afz_SET(4.459009E36F) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 6.585513E36F);
            assert(pack.time_boot_ms_GET() == 4180701792L);
            assert(pack.pitch_GET() == 9.848037E37F);
            assert(pack.z_GET() == 1.372645E38F);
            assert(pack.x_GET() == -1.961268E38F);
            assert(pack.y_GET() == 2.6801744E38F);
            assert(pack.roll_GET() == -1.8520274E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.x_SET(-1.961268E38F) ;
        p89.z_SET(1.372645E38F) ;
        p89.y_SET(2.6801744E38F) ;
        p89.time_boot_ms_SET(4180701792L) ;
        p89.roll_SET(-1.8520274E38F) ;
        p89.pitch_SET(9.848037E37F) ;
        p89.yaw_SET(6.585513E36F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2805747898747510801L);
            assert(pack.lat_GET() == 1671107534);
            assert(pack.roll_GET() == 2.996982E38F);
            assert(pack.pitchspeed_GET() == 8.536627E37F);
            assert(pack.yacc_GET() == (short) -18438);
            assert(pack.alt_GET() == 2131884479);
            assert(pack.zacc_GET() == (short)26418);
            assert(pack.vz_GET() == (short)2258);
            assert(pack.yawspeed_GET() == -1.8539713E38F);
            assert(pack.xacc_GET() == (short) -32710);
            assert(pack.rollspeed_GET() == 1.1053044E38F);
            assert(pack.vx_GET() == (short) -12357);
            assert(pack.vy_GET() == (short)29820);
            assert(pack.yaw_GET() == -1.0220232E38F);
            assert(pack.pitch_GET() == 2.9705396E38F);
            assert(pack.lon_GET() == 79932483);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.alt_SET(2131884479) ;
        p90.xacc_SET((short) -32710) ;
        p90.roll_SET(2.996982E38F) ;
        p90.pitch_SET(2.9705396E38F) ;
        p90.yaw_SET(-1.0220232E38F) ;
        p90.yawspeed_SET(-1.8539713E38F) ;
        p90.vz_SET((short)2258) ;
        p90.lat_SET(1671107534) ;
        p90.yacc_SET((short) -18438) ;
        p90.rollspeed_SET(1.1053044E38F) ;
        p90.vx_SET((short) -12357) ;
        p90.lon_SET(79932483) ;
        p90.zacc_SET((short)26418) ;
        p90.vy_SET((short)29820) ;
        p90.pitchspeed_SET(8.536627E37F) ;
        p90.time_usec_SET(2805747898747510801L) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_PREFLIGHT);
            assert(pack.aux1_GET() == -2.9107436E38F);
            assert(pack.yaw_rudder_GET() == 7.3278754E37F);
            assert(pack.time_usec_GET() == 2663528276453066444L);
            assert(pack.pitch_elevator_GET() == -2.5447005E38F);
            assert(pack.aux2_GET() == -6.681108E37F);
            assert(pack.aux4_GET() == 1.3977437E38F);
            assert(pack.aux3_GET() == 7.603271E37F);
            assert(pack.nav_mode_GET() == (char)112);
            assert(pack.throttle_GET() == -1.1959251E38F);
            assert(pack.roll_ailerons_GET() == -1.2528073E38F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.nav_mode_SET((char)112) ;
        p91.throttle_SET(-1.1959251E38F) ;
        p91.aux1_SET(-2.9107436E38F) ;
        p91.time_usec_SET(2663528276453066444L) ;
        p91.yaw_rudder_SET(7.3278754E37F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_PREFLIGHT) ;
        p91.aux3_SET(7.603271E37F) ;
        p91.aux4_SET(1.3977437E38F) ;
        p91.pitch_elevator_SET(-2.5447005E38F) ;
        p91.roll_ailerons_SET(-1.2528073E38F) ;
        p91.aux2_SET(-6.681108E37F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)175);
            assert(pack.chan10_raw_GET() == (char)1411);
            assert(pack.chan12_raw_GET() == (char)67);
            assert(pack.chan8_raw_GET() == (char)43000);
            assert(pack.chan1_raw_GET() == (char)47451);
            assert(pack.chan11_raw_GET() == (char)39167);
            assert(pack.chan9_raw_GET() == (char)36081);
            assert(pack.chan5_raw_GET() == (char)31444);
            assert(pack.chan3_raw_GET() == (char)63235);
            assert(pack.chan6_raw_GET() == (char)12466);
            assert(pack.chan4_raw_GET() == (char)44094);
            assert(pack.chan2_raw_GET() == (char)30222);
            assert(pack.chan7_raw_GET() == (char)36744);
            assert(pack.time_usec_GET() == 5824899145517704704L);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(5824899145517704704L) ;
        p92.chan4_raw_SET((char)44094) ;
        p92.chan12_raw_SET((char)67) ;
        p92.chan6_raw_SET((char)12466) ;
        p92.chan2_raw_SET((char)30222) ;
        p92.chan8_raw_SET((char)43000) ;
        p92.chan7_raw_SET((char)36744) ;
        p92.chan11_raw_SET((char)39167) ;
        p92.chan5_raw_SET((char)31444) ;
        p92.chan10_raw_SET((char)1411) ;
        p92.rssi_SET((char)175) ;
        p92.chan3_raw_SET((char)63235) ;
        p92.chan9_raw_SET((char)36081) ;
        p92.chan1_raw_SET((char)47451) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == 8512664200891991815L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_AUTO_ARMED);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.1824547E38F, -1.9255122E38F, -8.755881E37F, 2.5813402E38F, 1.2452029E38F, -9.637134E37F, -2.7275484E38F, -2.8199097E38F, -1.5900347E38F, 1.777883E38F, 1.0921016E38F, 2.063272E38F, 3.3203889E38F, -2.5345136E38F, -1.7565146E37F, 1.1134265E38F}));
            assert(pack.time_usec_GET() == 2151710287918060334L);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.flags_SET(8512664200891991815L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED) ;
        p93.controls_SET(new float[] {-1.1824547E38F, -1.9255122E38F, -8.755881E37F, 2.5813402E38F, 1.2452029E38F, -9.637134E37F, -2.7275484E38F, -2.8199097E38F, -1.5900347E38F, 1.777883E38F, 1.0921016E38F, 2.063272E38F, 3.3203889E38F, -2.5345136E38F, -1.7565146E37F, 1.1134265E38F}, 0) ;
        p93.time_usec_SET(2151710287918060334L) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_rate_x_TRY(ph) == 3.26973E37F);
            assert(pack.time_usec_GET() == 7939489545313688547L);
            assert(pack.flow_x_GET() == (short) -28567);
            assert(pack.quality_GET() == (char)68);
            assert(pack.flow_comp_m_x_GET() == -2.6366806E38F);
            assert(pack.sensor_id_GET() == (char)85);
            assert(pack.ground_distance_GET() == 1.4540832E38F);
            assert(pack.flow_comp_m_y_GET() == -2.689217E38F);
            assert(pack.flow_rate_y_TRY(ph) == 1.2155417E38F);
            assert(pack.flow_y_GET() == (short) -15053);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.sensor_id_SET((char)85) ;
        p100.quality_SET((char)68) ;
        p100.flow_rate_x_SET(3.26973E37F, PH) ;
        p100.flow_comp_m_x_SET(-2.6366806E38F) ;
        p100.flow_x_SET((short) -28567) ;
        p100.flow_y_SET((short) -15053) ;
        p100.ground_distance_SET(1.4540832E38F) ;
        p100.flow_comp_m_y_SET(-2.689217E38F) ;
        p100.time_usec_SET(7939489545313688547L) ;
        p100.flow_rate_y_SET(1.2155417E38F, PH) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 3.3412142E38F);
            assert(pack.pitch_GET() == 1.583683E38F);
            assert(pack.y_GET() == 3.1532726E38F);
            assert(pack.z_GET() == -2.8065281E38F);
            assert(pack.yaw_GET() == 9.736543E37F);
            assert(pack.usec_GET() == 1256779870822069063L);
            assert(pack.roll_GET() == 2.5284713E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.pitch_SET(1.583683E38F) ;
        p101.roll_SET(2.5284713E38F) ;
        p101.usec_SET(1256779870822069063L) ;
        p101.yaw_SET(9.736543E37F) ;
        p101.z_SET(-2.8065281E38F) ;
        p101.x_SET(3.3412142E38F) ;
        p101.y_SET(3.1532726E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 8739809619359399166L);
            assert(pack.yaw_GET() == 2.5414758E38F);
            assert(pack.z_GET() == 2.8041494E38F);
            assert(pack.roll_GET() == -2.650073E38F);
            assert(pack.pitch_GET() == -3.396467E38F);
            assert(pack.x_GET() == -2.4376783E38F);
            assert(pack.y_GET() == -2.936054E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.yaw_SET(2.5414758E38F) ;
        p102.roll_SET(-2.650073E38F) ;
        p102.z_SET(2.8041494E38F) ;
        p102.usec_SET(8739809619359399166L) ;
        p102.x_SET(-2.4376783E38F) ;
        p102.y_SET(-2.936054E38F) ;
        p102.pitch_SET(-3.396467E38F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 2.3661408E37F);
            assert(pack.y_GET() == -2.5455882E38F);
            assert(pack.x_GET() == 1.8968276E38F);
            assert(pack.usec_GET() == 1692261809250909048L);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.z_SET(2.3661408E37F) ;
        p103.y_SET(-2.5455882E38F) ;
        p103.x_SET(1.8968276E38F) ;
        p103.usec_SET(1692261809250909048L) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -1.4387698E38F);
            assert(pack.z_GET() == 2.2577405E38F);
            assert(pack.pitch_GET() == 2.2227255E38F);
            assert(pack.usec_GET() == 6861672245584420909L);
            assert(pack.x_GET() == 1.444084E38F);
            assert(pack.y_GET() == 2.2224059E38F);
            assert(pack.roll_GET() == 2.8470332E37F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.x_SET(1.444084E38F) ;
        p104.pitch_SET(2.2227255E38F) ;
        p104.yaw_SET(-1.4387698E38F) ;
        p104.roll_SET(2.8470332E37F) ;
        p104.usec_SET(6861672245584420909L) ;
        p104.y_SET(2.2224059E38F) ;
        p104.z_SET(2.2577405E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.xgyro_GET() == 2.9520354E38F);
            assert(pack.abs_pressure_GET() == 2.3777075E38F);
            assert(pack.fields_updated_GET() == (char)3242);
            assert(pack.diff_pressure_GET() == -1.4645272E38F);
            assert(pack.zacc_GET() == 2.4186074E38F);
            assert(pack.yacc_GET() == -2.9757615E38F);
            assert(pack.xacc_GET() == -2.779097E38F);
            assert(pack.temperature_GET() == 3.9836643E37F);
            assert(pack.ygyro_GET() == 1.2630694E38F);
            assert(pack.xmag_GET() == 1.0177485E38F);
            assert(pack.ymag_GET() == -2.6177768E38F);
            assert(pack.zmag_GET() == -2.8975552E38F);
            assert(pack.zgyro_GET() == 2.2189364E38F);
            assert(pack.time_usec_GET() == 8491742162289848914L);
            assert(pack.pressure_alt_GET() == -2.0809055E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.xgyro_SET(2.9520354E38F) ;
        p105.temperature_SET(3.9836643E37F) ;
        p105.abs_pressure_SET(2.3777075E38F) ;
        p105.yacc_SET(-2.9757615E38F) ;
        p105.zgyro_SET(2.2189364E38F) ;
        p105.time_usec_SET(8491742162289848914L) ;
        p105.zacc_SET(2.4186074E38F) ;
        p105.xmag_SET(1.0177485E38F) ;
        p105.xacc_SET(-2.779097E38F) ;
        p105.diff_pressure_SET(-1.4645272E38F) ;
        p105.ygyro_SET(1.2630694E38F) ;
        p105.fields_updated_SET((char)3242) ;
        p105.zmag_SET(-2.8975552E38F) ;
        p105.ymag_SET(-2.6177768E38F) ;
        p105.pressure_alt_SET(-2.0809055E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_ygyro_GET() == 8.247877E35F);
            assert(pack.integrated_zgyro_GET() == 1.030951E38F);
            assert(pack.temperature_GET() == (short) -14505);
            assert(pack.time_delta_distance_us_GET() == 810554416L);
            assert(pack.integrated_x_GET() == -1.4696426E38F);
            assert(pack.time_usec_GET() == 2222953783251925837L);
            assert(pack.sensor_id_GET() == (char)230);
            assert(pack.distance_GET() == 1.386438E38F);
            assert(pack.integrated_y_GET() == 1.2872093E38F);
            assert(pack.integration_time_us_GET() == 4284011333L);
            assert(pack.quality_GET() == (char)253);
            assert(pack.integrated_xgyro_GET() == 3.3624641E38F);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_zgyro_SET(1.030951E38F) ;
        p106.quality_SET((char)253) ;
        p106.time_usec_SET(2222953783251925837L) ;
        p106.temperature_SET((short) -14505) ;
        p106.integrated_y_SET(1.2872093E38F) ;
        p106.integration_time_us_SET(4284011333L) ;
        p106.time_delta_distance_us_SET(810554416L) ;
        p106.integrated_x_SET(-1.4696426E38F) ;
        p106.sensor_id_SET((char)230) ;
        p106.integrated_xgyro_SET(3.3624641E38F) ;
        p106.distance_SET(1.386438E38F) ;
        p106.integrated_ygyro_SET(8.247877E35F) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == -1.1874261E38F);
            assert(pack.time_usec_GET() == 2798540717141752367L);
            assert(pack.zacc_GET() == 2.046838E38F);
            assert(pack.xmag_GET() == 6.0405544E37F);
            assert(pack.temperature_GET() == 1.4513083E38F);
            assert(pack.diff_pressure_GET() == 1.6798932E38F);
            assert(pack.abs_pressure_GET() == -2.544241E38F);
            assert(pack.xgyro_GET() == 1.3450281E38F);
            assert(pack.ymag_GET() == -1.4084624E38F);
            assert(pack.pressure_alt_GET() == -3.1940956E38F);
            assert(pack.xacc_GET() == 4.157716E36F);
            assert(pack.fields_updated_GET() == 3811311736L);
            assert(pack.zmag_GET() == -1.1801563E37F);
            assert(pack.ygyro_GET() == -1.3728945E38F);
            assert(pack.yacc_GET() == -1.8785205E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(2798540717141752367L) ;
        p107.xacc_SET(4.157716E36F) ;
        p107.ymag_SET(-1.4084624E38F) ;
        p107.yacc_SET(-1.8785205E38F) ;
        p107.zacc_SET(2.046838E38F) ;
        p107.diff_pressure_SET(1.6798932E38F) ;
        p107.pressure_alt_SET(-3.1940956E38F) ;
        p107.fields_updated_SET(3811311736L) ;
        p107.xgyro_SET(1.3450281E38F) ;
        p107.abs_pressure_SET(-2.544241E38F) ;
        p107.ygyro_SET(-1.3728945E38F) ;
        p107.zgyro_SET(-1.1874261E38F) ;
        p107.zmag_SET(-1.1801563E37F) ;
        p107.xmag_SET(6.0405544E37F) ;
        p107.temperature_SET(1.4513083E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == -1.1227977E38F);
            assert(pack.q4_GET() == 1.3918312E38F);
            assert(pack.q3_GET() == 3.1537137E38F);
            assert(pack.yacc_GET() == 4.4190046E37F);
            assert(pack.yaw_GET() == -4.821671E37F);
            assert(pack.ve_GET() == 1.6192421E38F);
            assert(pack.std_dev_vert_GET() == -1.6381315E38F);
            assert(pack.ygyro_GET() == 3.1072084E37F);
            assert(pack.lon_GET() == -3.3752528E38F);
            assert(pack.vd_GET() == -1.809665E38F);
            assert(pack.alt_GET() == -4.3907736E37F);
            assert(pack.q1_GET() == -1.458659E37F);
            assert(pack.lat_GET() == -2.991232E38F);
            assert(pack.roll_GET() == -2.9492997E38F);
            assert(pack.pitch_GET() == 2.0907436E38F);
            assert(pack.std_dev_horz_GET() == -3.2675656E38F);
            assert(pack.vn_GET() == -1.5064458E38F);
            assert(pack.zacc_GET() == 2.7146614E38F);
            assert(pack.xgyro_GET() == 5.3826356E37F);
            assert(pack.zgyro_GET() == 1.86019E38F);
            assert(pack.q2_GET() == 1.9260156E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.q2_SET(1.9260156E38F) ;
        p108.xgyro_SET(5.3826356E37F) ;
        p108.yacc_SET(4.4190046E37F) ;
        p108.xacc_SET(-1.1227977E38F) ;
        p108.roll_SET(-2.9492997E38F) ;
        p108.q4_SET(1.3918312E38F) ;
        p108.alt_SET(-4.3907736E37F) ;
        p108.std_dev_horz_SET(-3.2675656E38F) ;
        p108.vn_SET(-1.5064458E38F) ;
        p108.zgyro_SET(1.86019E38F) ;
        p108.pitch_SET(2.0907436E38F) ;
        p108.q3_SET(3.1537137E38F) ;
        p108.q1_SET(-1.458659E37F) ;
        p108.ve_SET(1.6192421E38F) ;
        p108.std_dev_vert_SET(-1.6381315E38F) ;
        p108.vd_SET(-1.809665E38F) ;
        p108.zacc_SET(2.7146614E38F) ;
        p108.lat_SET(-2.991232E38F) ;
        p108.yaw_SET(-4.821671E37F) ;
        p108.lon_SET(-3.3752528E38F) ;
        p108.ygyro_SET(3.1072084E37F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.txbuf_GET() == (char)39);
            assert(pack.remrssi_GET() == (char)210);
            assert(pack.rssi_GET() == (char)163);
            assert(pack.fixed__GET() == (char)64816);
            assert(pack.remnoise_GET() == (char)245);
            assert(pack.rxerrors_GET() == (char)40832);
            assert(pack.noise_GET() == (char)116);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.remrssi_SET((char)210) ;
        p109.rssi_SET((char)163) ;
        p109.noise_SET((char)116) ;
        p109.rxerrors_SET((char)40832) ;
        p109.fixed__SET((char)64816) ;
        p109.txbuf_SET((char)39) ;
        p109.remnoise_SET((char)245) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)34);
            assert(pack.target_network_GET() == (char)240);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)62, (char)38, (char)61, (char)235, (char)112, (char)131, (char)35, (char)0, (char)227, (char)157, (char)158, (char)156, (char)184, (char)123, (char)21, (char)19, (char)28, (char)201, (char)69, (char)178, (char)1, (char)178, (char)81, (char)196, (char)82, (char)45, (char)61, (char)36, (char)95, (char)242, (char)57, (char)108, (char)114, (char)77, (char)143, (char)180, (char)102, (char)249, (char)117, (char)83, (char)253, (char)167, (char)118, (char)156, (char)148, (char)24, (char)42, (char)40, (char)27, (char)218, (char)102, (char)222, (char)116, (char)217, (char)33, (char)134, (char)150, (char)103, (char)60, (char)28, (char)161, (char)179, (char)150, (char)157, (char)106, (char)178, (char)37, (char)34, (char)81, (char)117, (char)209, (char)100, (char)142, (char)218, (char)136, (char)151, (char)114, (char)150, (char)254, (char)62, (char)199, (char)21, (char)26, (char)173, (char)226, (char)46, (char)3, (char)91, (char)252, (char)85, (char)165, (char)172, (char)7, (char)239, (char)127, (char)68, (char)166, (char)249, (char)188, (char)173, (char)109, (char)211, (char)18, (char)190, (char)32, (char)52, (char)210, (char)119, (char)224, (char)180, (char)164, (char)40, (char)130, (char)95, (char)149, (char)117, (char)31, (char)151, (char)216, (char)188, (char)81, (char)168, (char)194, (char)89, (char)225, (char)155, (char)11, (char)225, (char)96, (char)219, (char)248, (char)50, (char)116, (char)217, (char)82, (char)160, (char)173, (char)190, (char)6, (char)123, (char)226, (char)105, (char)189, (char)215, (char)217, (char)242, (char)125, (char)94, (char)63, (char)228, (char)45, (char)185, (char)181, (char)117, (char)126, (char)46, (char)130, (char)187, (char)157, (char)198, (char)167, (char)214, (char)188, (char)11, (char)86, (char)111, (char)137, (char)227, (char)81, (char)115, (char)37, (char)6, (char)64, (char)162, (char)225, (char)45, (char)60, (char)142, (char)244, (char)153, (char)233, (char)161, (char)164, (char)166, (char)134, (char)5, (char)106, (char)193, (char)117, (char)169, (char)79, (char)112, (char)109, (char)122, (char)194, (char)118, (char)184, (char)158, (char)106, (char)101, (char)111, (char)219, (char)13, (char)239, (char)240, (char)97, (char)200, (char)10, (char)184, (char)52, (char)210, (char)67, (char)240, (char)39, (char)254, (char)155, (char)138, (char)26, (char)25, (char)47, (char)115, (char)204, (char)107, (char)6, (char)114, (char)205, (char)232, (char)154, (char)141, (char)110, (char)57, (char)147, (char)5, (char)252, (char)1, (char)129, (char)148, (char)5, (char)85, (char)128, (char)142, (char)239, (char)137, (char)178, (char)46, (char)74, (char)183, (char)121, (char)8, (char)207, (char)156}));
            assert(pack.target_component_GET() == (char)147);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_system_SET((char)34) ;
        p110.target_component_SET((char)147) ;
        p110.payload_SET(new char[] {(char)62, (char)38, (char)61, (char)235, (char)112, (char)131, (char)35, (char)0, (char)227, (char)157, (char)158, (char)156, (char)184, (char)123, (char)21, (char)19, (char)28, (char)201, (char)69, (char)178, (char)1, (char)178, (char)81, (char)196, (char)82, (char)45, (char)61, (char)36, (char)95, (char)242, (char)57, (char)108, (char)114, (char)77, (char)143, (char)180, (char)102, (char)249, (char)117, (char)83, (char)253, (char)167, (char)118, (char)156, (char)148, (char)24, (char)42, (char)40, (char)27, (char)218, (char)102, (char)222, (char)116, (char)217, (char)33, (char)134, (char)150, (char)103, (char)60, (char)28, (char)161, (char)179, (char)150, (char)157, (char)106, (char)178, (char)37, (char)34, (char)81, (char)117, (char)209, (char)100, (char)142, (char)218, (char)136, (char)151, (char)114, (char)150, (char)254, (char)62, (char)199, (char)21, (char)26, (char)173, (char)226, (char)46, (char)3, (char)91, (char)252, (char)85, (char)165, (char)172, (char)7, (char)239, (char)127, (char)68, (char)166, (char)249, (char)188, (char)173, (char)109, (char)211, (char)18, (char)190, (char)32, (char)52, (char)210, (char)119, (char)224, (char)180, (char)164, (char)40, (char)130, (char)95, (char)149, (char)117, (char)31, (char)151, (char)216, (char)188, (char)81, (char)168, (char)194, (char)89, (char)225, (char)155, (char)11, (char)225, (char)96, (char)219, (char)248, (char)50, (char)116, (char)217, (char)82, (char)160, (char)173, (char)190, (char)6, (char)123, (char)226, (char)105, (char)189, (char)215, (char)217, (char)242, (char)125, (char)94, (char)63, (char)228, (char)45, (char)185, (char)181, (char)117, (char)126, (char)46, (char)130, (char)187, (char)157, (char)198, (char)167, (char)214, (char)188, (char)11, (char)86, (char)111, (char)137, (char)227, (char)81, (char)115, (char)37, (char)6, (char)64, (char)162, (char)225, (char)45, (char)60, (char)142, (char)244, (char)153, (char)233, (char)161, (char)164, (char)166, (char)134, (char)5, (char)106, (char)193, (char)117, (char)169, (char)79, (char)112, (char)109, (char)122, (char)194, (char)118, (char)184, (char)158, (char)106, (char)101, (char)111, (char)219, (char)13, (char)239, (char)240, (char)97, (char)200, (char)10, (char)184, (char)52, (char)210, (char)67, (char)240, (char)39, (char)254, (char)155, (char)138, (char)26, (char)25, (char)47, (char)115, (char)204, (char)107, (char)6, (char)114, (char)205, (char)232, (char)154, (char)141, (char)110, (char)57, (char)147, (char)5, (char)252, (char)1, (char)129, (char)148, (char)5, (char)85, (char)128, (char)142, (char)239, (char)137, (char)178, (char)46, (char)74, (char)183, (char)121, (char)8, (char)207, (char)156}, 0) ;
        p110.target_network_SET((char)240) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -6005829663247719890L);
            assert(pack.ts1_GET() == 7037285633454748139L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(7037285633454748139L) ;
        p111.tc1_SET(-6005829663247719890L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4333609588457910639L);
            assert(pack.seq_GET() == 3479234396L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(4333609588457910639L) ;
        p112.seq_SET(3479234396L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -793522474);
            assert(pack.alt_GET() == 171992358);
            assert(pack.vd_GET() == (short) -22521);
            assert(pack.vn_GET() == (short) -15311);
            assert(pack.satellites_visible_GET() == (char)114);
            assert(pack.ve_GET() == (short) -10238);
            assert(pack.vel_GET() == (char)25685);
            assert(pack.lat_GET() == -1680953751);
            assert(pack.time_usec_GET() == 7093634573880999927L);
            assert(pack.eph_GET() == (char)2991);
            assert(pack.epv_GET() == (char)48181);
            assert(pack.cog_GET() == (char)24067);
            assert(pack.fix_type_GET() == (char)63);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.vn_SET((short) -15311) ;
        p113.time_usec_SET(7093634573880999927L) ;
        p113.ve_SET((short) -10238) ;
        p113.lat_SET(-1680953751) ;
        p113.vel_SET((char)25685) ;
        p113.eph_SET((char)2991) ;
        p113.lon_SET(-793522474) ;
        p113.alt_SET(171992358) ;
        p113.fix_type_SET((char)63) ;
        p113.epv_SET((char)48181) ;
        p113.satellites_visible_SET((char)114) ;
        p113.vd_SET((short) -22521) ;
        p113.cog_SET((char)24067) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integration_time_us_GET() == 1252533099L);
            assert(pack.time_delta_distance_us_GET() == 2033751915L);
            assert(pack.quality_GET() == (char)253);
            assert(pack.sensor_id_GET() == (char)168);
            assert(pack.integrated_ygyro_GET() == -8.4818745E37F);
            assert(pack.integrated_x_GET() == 1.0519704E38F);
            assert(pack.temperature_GET() == (short) -20313);
            assert(pack.integrated_y_GET() == 2.3392829E38F);
            assert(pack.time_usec_GET() == 8391397974986539638L);
            assert(pack.distance_GET() == -1.8665076E38F);
            assert(pack.integrated_zgyro_GET() == 1.2113529E38F);
            assert(pack.integrated_xgyro_GET() == 2.3368382E38F);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_xgyro_SET(2.3368382E38F) ;
        p114.integration_time_us_SET(1252533099L) ;
        p114.integrated_y_SET(2.3392829E38F) ;
        p114.time_usec_SET(8391397974986539638L) ;
        p114.quality_SET((char)253) ;
        p114.integrated_zgyro_SET(1.2113529E38F) ;
        p114.integrated_x_SET(1.0519704E38F) ;
        p114.integrated_ygyro_SET(-8.4818745E37F) ;
        p114.time_delta_distance_us_SET(2033751915L) ;
        p114.temperature_SET((short) -20313) ;
        p114.sensor_id_SET((char)168) ;
        p114.distance_SET(-1.8665076E38F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.true_airspeed_GET() == (char)5741);
            assert(pack.vz_GET() == (short)3683);
            assert(pack.lon_GET() == 258215679);
            assert(pack.vx_GET() == (short) -6854);
            assert(pack.zacc_GET() == (short)20904);
            assert(pack.xacc_GET() == (short)23714);
            assert(pack.yawspeed_GET() == 2.0390172E38F);
            assert(pack.pitchspeed_GET() == 2.384443E38F);
            assert(pack.lat_GET() == 1516421823);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {3.1804025E37F, -8.276683E37F, 6.4736746E37F, -2.9407857E38F}));
            assert(pack.alt_GET() == 1243260946);
            assert(pack.vy_GET() == (short)25522);
            assert(pack.time_usec_GET() == 7308934625313404094L);
            assert(pack.rollspeed_GET() == 9.617014E37F);
            assert(pack.yacc_GET() == (short)6982);
            assert(pack.ind_airspeed_GET() == (char)7877);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.vx_SET((short) -6854) ;
        p115.ind_airspeed_SET((char)7877) ;
        p115.true_airspeed_SET((char)5741) ;
        p115.time_usec_SET(7308934625313404094L) ;
        p115.yacc_SET((short)6982) ;
        p115.lat_SET(1516421823) ;
        p115.pitchspeed_SET(2.384443E38F) ;
        p115.xacc_SET((short)23714) ;
        p115.attitude_quaternion_SET(new float[] {3.1804025E37F, -8.276683E37F, 6.4736746E37F, -2.9407857E38F}, 0) ;
        p115.rollspeed_SET(9.617014E37F) ;
        p115.alt_SET(1243260946) ;
        p115.lon_SET(258215679) ;
        p115.zacc_SET((short)20904) ;
        p115.vz_SET((short)3683) ;
        p115.yawspeed_SET(2.0390172E38F) ;
        p115.vy_SET((short)25522) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 4135326181L);
            assert(pack.ymag_GET() == (short) -5903);
            assert(pack.xacc_GET() == (short) -13309);
            assert(pack.yacc_GET() == (short) -6292);
            assert(pack.zmag_GET() == (short)17435);
            assert(pack.ygyro_GET() == (short) -1863);
            assert(pack.xgyro_GET() == (short)5462);
            assert(pack.zgyro_GET() == (short) -11587);
            assert(pack.zacc_GET() == (short)18535);
            assert(pack.xmag_GET() == (short) -28773);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.ygyro_SET((short) -1863) ;
        p116.zmag_SET((short)17435) ;
        p116.yacc_SET((short) -6292) ;
        p116.xgyro_SET((short)5462) ;
        p116.zgyro_SET((short) -11587) ;
        p116.xmag_SET((short) -28773) ;
        p116.time_boot_ms_SET(4135326181L) ;
        p116.ymag_SET((short) -5903) ;
        p116.zacc_SET((short)18535) ;
        p116.xacc_SET((short) -13309) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)163);
            assert(pack.target_component_GET() == (char)99);
            assert(pack.end_GET() == (char)41098);
            assert(pack.start_GET() == (char)11960);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)163) ;
        p117.start_SET((char)11960) ;
        p117.target_component_SET((char)99) ;
        p117.end_SET((char)41098) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.num_logs_GET() == (char)35564);
            assert(pack.id_GET() == (char)63294);
            assert(pack.time_utc_GET() == 3089637566L);
            assert(pack.size_GET() == 19180273L);
            assert(pack.last_log_num_GET() == (char)2970);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.last_log_num_SET((char)2970) ;
        p118.size_SET(19180273L) ;
        p118.time_utc_SET(3089637566L) ;
        p118.num_logs_SET((char)35564) ;
        p118.id_SET((char)63294) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)59);
            assert(pack.id_GET() == (char)45358);
            assert(pack.ofs_GET() == 3375524511L);
            assert(pack.count_GET() == 2861437648L);
            assert(pack.target_system_GET() == (char)200);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_component_SET((char)59) ;
        p119.ofs_SET(3375524511L) ;
        p119.id_SET((char)45358) ;
        p119.target_system_SET((char)200) ;
        p119.count_SET(2861437648L) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.ofs_GET() == 2974606784L);
            assert(pack.count_GET() == (char)130);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)110, (char)250, (char)44, (char)118, (char)158, (char)100, (char)220, (char)141, (char)180, (char)99, (char)51, (char)253, (char)187, (char)246, (char)24, (char)180, (char)235, (char)154, (char)145, (char)209, (char)39, (char)53, (char)5, (char)199, (char)234, (char)190, (char)165, (char)72, (char)231, (char)137, (char)83, (char)15, (char)17, (char)108, (char)84, (char)29, (char)248, (char)172, (char)57, (char)227, (char)80, (char)93, (char)97, (char)36, (char)30, (char)167, (char)120, (char)246, (char)144, (char)55, (char)237, (char)52, (char)221, (char)127, (char)122, (char)89, (char)244, (char)98, (char)216, (char)193, (char)156, (char)120, (char)62, (char)82, (char)8, (char)49, (char)190, (char)38, (char)100, (char)85, (char)115, (char)32, (char)28, (char)103, (char)3, (char)136, (char)98, (char)217, (char)191, (char)184, (char)48, (char)204, (char)144, (char)237, (char)47, (char)47, (char)38, (char)53, (char)237, (char)96}));
            assert(pack.id_GET() == (char)9817);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.data__SET(new char[] {(char)110, (char)250, (char)44, (char)118, (char)158, (char)100, (char)220, (char)141, (char)180, (char)99, (char)51, (char)253, (char)187, (char)246, (char)24, (char)180, (char)235, (char)154, (char)145, (char)209, (char)39, (char)53, (char)5, (char)199, (char)234, (char)190, (char)165, (char)72, (char)231, (char)137, (char)83, (char)15, (char)17, (char)108, (char)84, (char)29, (char)248, (char)172, (char)57, (char)227, (char)80, (char)93, (char)97, (char)36, (char)30, (char)167, (char)120, (char)246, (char)144, (char)55, (char)237, (char)52, (char)221, (char)127, (char)122, (char)89, (char)244, (char)98, (char)216, (char)193, (char)156, (char)120, (char)62, (char)82, (char)8, (char)49, (char)190, (char)38, (char)100, (char)85, (char)115, (char)32, (char)28, (char)103, (char)3, (char)136, (char)98, (char)217, (char)191, (char)184, (char)48, (char)204, (char)144, (char)237, (char)47, (char)47, (char)38, (char)53, (char)237, (char)96}, 0) ;
        p120.count_SET((char)130) ;
        p120.id_SET((char)9817) ;
        p120.ofs_SET(2974606784L) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)109);
            assert(pack.target_component_GET() == (char)74);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)74) ;
        p121.target_system_SET((char)109) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)84);
            assert(pack.target_system_GET() == (char)63);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)63) ;
        p122.target_component_SET((char)84) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)100);
            assert(pack.target_system_GET() == (char)66);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)254, (char)89, (char)168, (char)133, (char)32, (char)255, (char)182, (char)139, (char)53, (char)240, (char)29, (char)237, (char)88, (char)215, (char)158, (char)102, (char)106, (char)109, (char)26, (char)88, (char)3, (char)171, (char)160, (char)159, (char)126, (char)84, (char)63, (char)149, (char)89, (char)16, (char)30, (char)234, (char)135, (char)30, (char)135, (char)224, (char)146, (char)7, (char)40, (char)6, (char)184, (char)5, (char)168, (char)116, (char)212, (char)25, (char)203, (char)4, (char)248, (char)29, (char)67, (char)150, (char)207, (char)204, (char)243, (char)48, (char)216, (char)40, (char)176, (char)195, (char)247, (char)104, (char)127, (char)65, (char)193, (char)76, (char)5, (char)16, (char)201, (char)55, (char)234, (char)58, (char)200, (char)154, (char)44, (char)63, (char)102, (char)92, (char)226, (char)235, (char)122, (char)35, (char)37, (char)12, (char)111, (char)255, (char)207, (char)105, (char)125, (char)175, (char)246, (char)118, (char)25, (char)238, (char)84, (char)217, (char)31, (char)247, (char)7, (char)108, (char)37, (char)128, (char)65, (char)16, (char)183, (char)88, (char)14, (char)203, (char)165, (char)208}));
            assert(pack.len_GET() == (char)213);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.len_SET((char)213) ;
        p123.target_system_SET((char)66) ;
        p123.data__SET(new char[] {(char)254, (char)89, (char)168, (char)133, (char)32, (char)255, (char)182, (char)139, (char)53, (char)240, (char)29, (char)237, (char)88, (char)215, (char)158, (char)102, (char)106, (char)109, (char)26, (char)88, (char)3, (char)171, (char)160, (char)159, (char)126, (char)84, (char)63, (char)149, (char)89, (char)16, (char)30, (char)234, (char)135, (char)30, (char)135, (char)224, (char)146, (char)7, (char)40, (char)6, (char)184, (char)5, (char)168, (char)116, (char)212, (char)25, (char)203, (char)4, (char)248, (char)29, (char)67, (char)150, (char)207, (char)204, (char)243, (char)48, (char)216, (char)40, (char)176, (char)195, (char)247, (char)104, (char)127, (char)65, (char)193, (char)76, (char)5, (char)16, (char)201, (char)55, (char)234, (char)58, (char)200, (char)154, (char)44, (char)63, (char)102, (char)92, (char)226, (char)235, (char)122, (char)35, (char)37, (char)12, (char)111, (char)255, (char)207, (char)105, (char)125, (char)175, (char)246, (char)118, (char)25, (char)238, (char)84, (char)217, (char)31, (char)247, (char)7, (char)108, (char)37, (char)128, (char)65, (char)16, (char)183, (char)88, (char)14, (char)203, (char)165, (char)208}, 0) ;
        p123.target_component_SET((char)100) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -627196510);
            assert(pack.vel_GET() == (char)1098);
            assert(pack.epv_GET() == (char)28730);
            assert(pack.dgps_age_GET() == 4125470509L);
            assert(pack.lon_GET() == -1363551316);
            assert(pack.satellites_visible_GET() == (char)123);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
            assert(pack.dgps_numch_GET() == (char)169);
            assert(pack.time_usec_GET() == 367003671084679538L);
            assert(pack.eph_GET() == (char)10727);
            assert(pack.lat_GET() == 1510069027);
            assert(pack.cog_GET() == (char)25606);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.alt_SET(-627196510) ;
        p124.eph_SET((char)10727) ;
        p124.dgps_numch_SET((char)169) ;
        p124.satellites_visible_SET((char)123) ;
        p124.vel_SET((char)1098) ;
        p124.time_usec_SET(367003671084679538L) ;
        p124.lat_SET(1510069027) ;
        p124.dgps_age_SET(4125470509L) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX) ;
        p124.cog_SET((char)25606) ;
        p124.lon_SET(-1363551316) ;
        p124.epv_SET((char)28730) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vservo_GET() == (char)44653);
            assert(pack.Vcc_GET() == (char)9065);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED));
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)9065) ;
        p125.Vservo_SET((char)44653) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED)) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.baudrate_GET() == 2245213778L);
            assert(pack.timeout_GET() == (char)32069);
            assert(pack.count_GET() == (char)185);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)142, (char)252, (char)145, (char)105, (char)222, (char)113, (char)211, (char)45, (char)53, (char)247, (char)24, (char)236, (char)45, (char)228, (char)22, (char)133, (char)148, (char)56, (char)45, (char)66, (char)248, (char)204, (char)101, (char)142, (char)170, (char)128, (char)4, (char)161, (char)156, (char)101, (char)133, (char)197, (char)197, (char)133, (char)14, (char)94, (char)11, (char)199, (char)76, (char)102, (char)181, (char)54, (char)210, (char)184, (char)220, (char)217, (char)243, (char)131, (char)197, (char)84, (char)254, (char)185, (char)60, (char)193, (char)81, (char)13, (char)115, (char)102, (char)165, (char)32, (char)82, (char)58, (char)7, (char)192, (char)244, (char)223, (char)109, (char)224, (char)145, (char)15}));
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING)) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1) ;
        p126.baudrate_SET(2245213778L) ;
        p126.data__SET(new char[] {(char)142, (char)252, (char)145, (char)105, (char)222, (char)113, (char)211, (char)45, (char)53, (char)247, (char)24, (char)236, (char)45, (char)228, (char)22, (char)133, (char)148, (char)56, (char)45, (char)66, (char)248, (char)204, (char)101, (char)142, (char)170, (char)128, (char)4, (char)161, (char)156, (char)101, (char)133, (char)197, (char)197, (char)133, (char)14, (char)94, (char)11, (char)199, (char)76, (char)102, (char)181, (char)54, (char)210, (char)184, (char)220, (char)217, (char)243, (char)131, (char)197, (char)84, (char)254, (char)185, (char)60, (char)193, (char)81, (char)13, (char)115, (char)102, (char)165, (char)32, (char)82, (char)58, (char)7, (char)192, (char)244, (char)223, (char)109, (char)224, (char)145, (char)15}, 0) ;
        p126.count_SET((char)185) ;
        p126.timeout_SET((char)32069) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.rtk_receiver_id_GET() == (char)126);
            assert(pack.baseline_coords_type_GET() == (char)34);
            assert(pack.baseline_a_mm_GET() == 528450812);
            assert(pack.rtk_health_GET() == (char)244);
            assert(pack.nsats_GET() == (char)193);
            assert(pack.wn_GET() == (char)22364);
            assert(pack.tow_GET() == 3365194237L);
            assert(pack.rtk_rate_GET() == (char)95);
            assert(pack.accuracy_GET() == 1420346425L);
            assert(pack.baseline_c_mm_GET() == -59691242);
            assert(pack.iar_num_hypotheses_GET() == -1956107029);
            assert(pack.time_last_baseline_ms_GET() == 121053344L);
            assert(pack.baseline_b_mm_GET() == -1106928177);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.baseline_coords_type_SET((char)34) ;
        p127.baseline_a_mm_SET(528450812) ;
        p127.accuracy_SET(1420346425L) ;
        p127.time_last_baseline_ms_SET(121053344L) ;
        p127.baseline_b_mm_SET(-1106928177) ;
        p127.rtk_health_SET((char)244) ;
        p127.nsats_SET((char)193) ;
        p127.rtk_rate_SET((char)95) ;
        p127.baseline_c_mm_SET(-59691242) ;
        p127.iar_num_hypotheses_SET(-1956107029) ;
        p127.tow_SET(3365194237L) ;
        p127.wn_SET((char)22364) ;
        p127.rtk_receiver_id_SET((char)126) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.iar_num_hypotheses_GET() == 536856360);
            assert(pack.nsats_GET() == (char)152);
            assert(pack.tow_GET() == 1622110632L);
            assert(pack.baseline_c_mm_GET() == 1847291315);
            assert(pack.rtk_rate_GET() == (char)229);
            assert(pack.rtk_health_GET() == (char)35);
            assert(pack.time_last_baseline_ms_GET() == 2252074013L);
            assert(pack.baseline_a_mm_GET() == 270422651);
            assert(pack.baseline_coords_type_GET() == (char)207);
            assert(pack.rtk_receiver_id_GET() == (char)104);
            assert(pack.accuracy_GET() == 159477032L);
            assert(pack.wn_GET() == (char)42710);
            assert(pack.baseline_b_mm_GET() == 1234484949);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(2252074013L) ;
        p128.iar_num_hypotheses_SET(536856360) ;
        p128.accuracy_SET(159477032L) ;
        p128.rtk_rate_SET((char)229) ;
        p128.baseline_a_mm_SET(270422651) ;
        p128.rtk_receiver_id_SET((char)104) ;
        p128.baseline_b_mm_SET(1234484949) ;
        p128.baseline_c_mm_SET(1847291315) ;
        p128.baseline_coords_type_SET((char)207) ;
        p128.rtk_health_SET((char)35) ;
        p128.tow_SET(1622110632L) ;
        p128.nsats_SET((char)152) ;
        p128.wn_SET((char)42710) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short) -24809);
            assert(pack.time_boot_ms_GET() == 2184389870L);
            assert(pack.zmag_GET() == (short)18177);
            assert(pack.xgyro_GET() == (short)16234);
            assert(pack.ymag_GET() == (short) -14239);
            assert(pack.xacc_GET() == (short)23257);
            assert(pack.xmag_GET() == (short)6764);
            assert(pack.zgyro_GET() == (short)29499);
            assert(pack.zacc_GET() == (short) -26184);
            assert(pack.ygyro_GET() == (short)24880);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.zmag_SET((short)18177) ;
        p129.yacc_SET((short) -24809) ;
        p129.time_boot_ms_SET(2184389870L) ;
        p129.xmag_SET((short)6764) ;
        p129.zgyro_SET((short)29499) ;
        p129.xgyro_SET((short)16234) ;
        p129.ygyro_SET((short)24880) ;
        p129.zacc_SET((short) -26184) ;
        p129.ymag_SET((short) -14239) ;
        p129.xacc_SET((short)23257) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.height_GET() == (char)4841);
            assert(pack.size_GET() == 4199959424L);
            assert(pack.width_GET() == (char)43710);
            assert(pack.type_GET() == (char)46);
            assert(pack.payload_GET() == (char)34);
            assert(pack.jpg_quality_GET() == (char)149);
            assert(pack.packets_GET() == (char)27534);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.packets_SET((char)27534) ;
        p130.height_SET((char)4841) ;
        p130.width_SET((char)43710) ;
        p130.jpg_quality_SET((char)149) ;
        p130.payload_SET((char)34) ;
        p130.size_SET(4199959424L) ;
        p130.type_SET((char)46) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)7919);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)74, (char)255, (char)95, (char)149, (char)65, (char)52, (char)171, (char)3, (char)252, (char)51, (char)145, (char)249, (char)49, (char)102, (char)170, (char)223, (char)10, (char)152, (char)8, (char)198, (char)29, (char)138, (char)156, (char)11, (char)192, (char)146, (char)104, (char)208, (char)92, (char)178, (char)207, (char)59, (char)142, (char)150, (char)193, (char)255, (char)45, (char)163, (char)202, (char)228, (char)229, (char)223, (char)70, (char)233, (char)127, (char)58, (char)36, (char)228, (char)26, (char)175, (char)94, (char)26, (char)121, (char)156, (char)139, (char)4, (char)97, (char)186, (char)186, (char)141, (char)217, (char)94, (char)192, (char)45, (char)14, (char)174, (char)77, (char)87, (char)228, (char)22, (char)61, (char)11, (char)65, (char)91, (char)87, (char)119, (char)233, (char)33, (char)8, (char)62, (char)188, (char)62, (char)89, (char)17, (char)81, (char)111, (char)13, (char)173, (char)231, (char)203, (char)105, (char)17, (char)183, (char)200, (char)11, (char)79, (char)207, (char)22, (char)139, (char)67, (char)152, (char)89, (char)235, (char)143, (char)223, (char)126, (char)112, (char)216, (char)119, (char)115, (char)164, (char)25, (char)136, (char)72, (char)219, (char)199, (char)248, (char)31, (char)167, (char)27, (char)129, (char)55, (char)50, (char)222, (char)240, (char)231, (char)39, (char)205, (char)46, (char)122, (char)59, (char)241, (char)25, (char)58, (char)218, (char)233, (char)121, (char)151, (char)100, (char)8, (char)67, (char)127, (char)36, (char)253, (char)128, (char)186, (char)62, (char)184, (char)91, (char)37, (char)208, (char)155, (char)121, (char)66, (char)107, (char)44, (char)214, (char)142, (char)65, (char)55, (char)191, (char)73, (char)115, (char)111, (char)251, (char)51, (char)91, (char)186, (char)202, (char)130, (char)144, (char)27, (char)191, (char)116, (char)100, (char)184, (char)243, (char)54, (char)120, (char)10, (char)203, (char)47, (char)116, (char)94, (char)244, (char)245, (char)6, (char)98, (char)132, (char)203, (char)215, (char)253, (char)118, (char)91, (char)17, (char)209, (char)67, (char)239, (char)176, (char)179, (char)105, (char)86, (char)204, (char)60, (char)231, (char)203, (char)208, (char)38, (char)95, (char)84, (char)126, (char)38, (char)234, (char)156, (char)24, (char)76, (char)21, (char)62, (char)133, (char)56, (char)36, (char)123, (char)255, (char)65, (char)180, (char)229, (char)146, (char)161, (char)75, (char)44, (char)98, (char)212, (char)11, (char)153, (char)40, (char)253, (char)20, (char)151, (char)231, (char)125, (char)154, (char)10, (char)106, (char)16, (char)130, (char)165, (char)75, (char)194, (char)68, (char)77, (char)124, (char)247, (char)229}));
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)7919) ;
        p131.data__SET(new char[] {(char)74, (char)255, (char)95, (char)149, (char)65, (char)52, (char)171, (char)3, (char)252, (char)51, (char)145, (char)249, (char)49, (char)102, (char)170, (char)223, (char)10, (char)152, (char)8, (char)198, (char)29, (char)138, (char)156, (char)11, (char)192, (char)146, (char)104, (char)208, (char)92, (char)178, (char)207, (char)59, (char)142, (char)150, (char)193, (char)255, (char)45, (char)163, (char)202, (char)228, (char)229, (char)223, (char)70, (char)233, (char)127, (char)58, (char)36, (char)228, (char)26, (char)175, (char)94, (char)26, (char)121, (char)156, (char)139, (char)4, (char)97, (char)186, (char)186, (char)141, (char)217, (char)94, (char)192, (char)45, (char)14, (char)174, (char)77, (char)87, (char)228, (char)22, (char)61, (char)11, (char)65, (char)91, (char)87, (char)119, (char)233, (char)33, (char)8, (char)62, (char)188, (char)62, (char)89, (char)17, (char)81, (char)111, (char)13, (char)173, (char)231, (char)203, (char)105, (char)17, (char)183, (char)200, (char)11, (char)79, (char)207, (char)22, (char)139, (char)67, (char)152, (char)89, (char)235, (char)143, (char)223, (char)126, (char)112, (char)216, (char)119, (char)115, (char)164, (char)25, (char)136, (char)72, (char)219, (char)199, (char)248, (char)31, (char)167, (char)27, (char)129, (char)55, (char)50, (char)222, (char)240, (char)231, (char)39, (char)205, (char)46, (char)122, (char)59, (char)241, (char)25, (char)58, (char)218, (char)233, (char)121, (char)151, (char)100, (char)8, (char)67, (char)127, (char)36, (char)253, (char)128, (char)186, (char)62, (char)184, (char)91, (char)37, (char)208, (char)155, (char)121, (char)66, (char)107, (char)44, (char)214, (char)142, (char)65, (char)55, (char)191, (char)73, (char)115, (char)111, (char)251, (char)51, (char)91, (char)186, (char)202, (char)130, (char)144, (char)27, (char)191, (char)116, (char)100, (char)184, (char)243, (char)54, (char)120, (char)10, (char)203, (char)47, (char)116, (char)94, (char)244, (char)245, (char)6, (char)98, (char)132, (char)203, (char)215, (char)253, (char)118, (char)91, (char)17, (char)209, (char)67, (char)239, (char)176, (char)179, (char)105, (char)86, (char)204, (char)60, (char)231, (char)203, (char)208, (char)38, (char)95, (char)84, (char)126, (char)38, (char)234, (char)156, (char)24, (char)76, (char)21, (char)62, (char)133, (char)56, (char)36, (char)123, (char)255, (char)65, (char)180, (char)229, (char)146, (char)161, (char)75, (char)44, (char)98, (char)212, (char)11, (char)153, (char)40, (char)253, (char)20, (char)151, (char)231, (char)125, (char)154, (char)10, (char)106, (char)16, (char)130, (char)165, (char)75, (char)194, (char)68, (char)77, (char)124, (char)247, (char)229}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.max_distance_GET() == (char)9109);
            assert(pack.current_distance_GET() == (char)619);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_PITCH_90);
            assert(pack.time_boot_ms_GET() == 2757054305L);
            assert(pack.covariance_GET() == (char)91);
            assert(pack.id_GET() == (char)170);
            assert(pack.min_distance_GET() == (char)39228);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.current_distance_SET((char)619) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p132.covariance_SET((char)91) ;
        p132.min_distance_SET((char)39228) ;
        p132.max_distance_SET((char)9109) ;
        p132.id_SET((char)170) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_PITCH_90) ;
        p132.time_boot_ms_SET(2757054305L) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.grid_spacing_GET() == (char)23220);
            assert(pack.lat_GET() == 1819123477);
            assert(pack.mask_GET() == 3255755033095146524L);
            assert(pack.lon_GET() == -40107161);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lon_SET(-40107161) ;
        p133.grid_spacing_SET((char)23220) ;
        p133.lat_SET(1819123477) ;
        p133.mask_SET(3255755033095146524L) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 1320728830);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -6633, (short)13532, (short)18535, (short)30548, (short)14525, (short) -5436, (short)24313, (short)4627, (short)25212, (short) -19339, (short)11395, (short)27702, (short)29245, (short) -19873, (short)16605, (short) -27315}));
            assert(pack.gridbit_GET() == (char)84);
            assert(pack.lat_GET() == -1900251642);
            assert(pack.grid_spacing_GET() == (char)62197);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.gridbit_SET((char)84) ;
        p134.grid_spacing_SET((char)62197) ;
        p134.lat_SET(-1900251642) ;
        p134.data__SET(new short[] {(short) -6633, (short)13532, (short)18535, (short)30548, (short)14525, (short) -5436, (short)24313, (short)4627, (short)25212, (short) -19339, (short)11395, (short)27702, (short)29245, (short) -19873, (short)16605, (short) -27315}, 0) ;
        p134.lon_SET(1320728830) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -281793939);
            assert(pack.lon_GET() == -855158879);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(-855158879) ;
        p135.lat_SET(-281793939) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.terrain_height_GET() == 8.627244E37F);
            assert(pack.loaded_GET() == (char)30173);
            assert(pack.lat_GET() == 1580520964);
            assert(pack.lon_GET() == -1383797206);
            assert(pack.spacing_GET() == (char)28499);
            assert(pack.current_height_GET() == -3.2758241E38F);
            assert(pack.pending_GET() == (char)34990);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lon_SET(-1383797206) ;
        p136.spacing_SET((char)28499) ;
        p136.pending_SET((char)34990) ;
        p136.terrain_height_SET(8.627244E37F) ;
        p136.loaded_SET((char)30173) ;
        p136.current_height_SET(-3.2758241E38F) ;
        p136.lat_SET(1580520964) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -2.6255539E38F);
            assert(pack.press_diff_GET() == -8.511785E37F);
            assert(pack.time_boot_ms_GET() == 2359572895L);
            assert(pack.temperature_GET() == (short) -7068);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_diff_SET(-8.511785E37F) ;
        p137.time_boot_ms_SET(2359572895L) ;
        p137.temperature_SET((short) -7068) ;
        p137.press_abs_SET(-2.6255539E38F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -6.867384E37F);
            assert(pack.z_GET() == -6.358424E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.8625408E38F, 2.1991436E38F, 3.9957473E37F, -2.9245959E38F}));
            assert(pack.x_GET() == 1.8637642E38F);
            assert(pack.time_usec_GET() == 1070026239586581011L);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.q_SET(new float[] {-2.8625408E38F, 2.1991436E38F, 3.9957473E37F, -2.9245959E38F}, 0) ;
        p138.y_SET(-6.867384E37F) ;
        p138.time_usec_SET(1070026239586581011L) ;
        p138.x_SET(1.8637642E38F) ;
        p138.z_SET(-6.358424E37F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)157);
            assert(pack.target_system_GET() == (char)130);
            assert(pack.time_usec_GET() == 1610694681682041670L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.7112431E38F, 1.5381475E38F, -2.2034995E37F, 1.80231E38F, 3.2272297E38F, -4.184397E37F, 3.3764837E38F, -2.6302176E38F}));
            assert(pack.group_mlx_GET() == (char)143);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.controls_SET(new float[] {1.7112431E38F, 1.5381475E38F, -2.2034995E37F, 1.80231E38F, 3.2272297E38F, -4.184397E37F, 3.3764837E38F, -2.6302176E38F}, 0) ;
        p139.target_component_SET((char)157) ;
        p139.target_system_SET((char)130) ;
        p139.time_usec_SET(1610694681682041670L) ;
        p139.group_mlx_SET((char)143) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 8385938220311681286L);
            assert(pack.group_mlx_GET() == (char)28);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.3804026E38F, 4.9188266E37F, 8.0664345E37F, 3.0730015E38F, -3.948888E37F, 2.9257905E38F, 3.085378E37F, -3.284177E38F}));
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.controls_SET(new float[] {-2.3804026E38F, 4.9188266E37F, 8.0664345E37F, 3.0730015E38F, -3.948888E37F, 2.9257905E38F, 3.085378E37F, -3.284177E38F}, 0) ;
        p140.group_mlx_SET((char)28) ;
        p140.time_usec_SET(8385938220311681286L) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_monotonic_GET() == -3.6823911E37F);
            assert(pack.altitude_relative_GET() == -7.0468636E37F);
            assert(pack.altitude_local_GET() == -2.9351379E38F);
            assert(pack.time_usec_GET() == 519139507125745213L);
            assert(pack.bottom_clearance_GET() == -2.4995508E38F);
            assert(pack.altitude_terrain_GET() == 3.0133209E38F);
            assert(pack.altitude_amsl_GET() == 6.0765526E37F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_monotonic_SET(-3.6823911E37F) ;
        p141.bottom_clearance_SET(-2.4995508E38F) ;
        p141.altitude_amsl_SET(6.0765526E37F) ;
        p141.altitude_local_SET(-2.9351379E38F) ;
        p141.altitude_relative_SET(-7.0468636E37F) ;
        p141.altitude_terrain_SET(3.0133209E38F) ;
        p141.time_usec_SET(519139507125745213L) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)37, (char)42, (char)162, (char)46, (char)242, (char)55, (char)245, (char)35, (char)25, (char)29, (char)209, (char)170, (char)60, (char)131, (char)180, (char)254, (char)75, (char)249, (char)168, (char)193, (char)206, (char)0, (char)15, (char)152, (char)161, (char)188, (char)10, (char)166, (char)158, (char)109, (char)229, (char)23, (char)211, (char)46, (char)207, (char)15, (char)217, (char)243, (char)163, (char)244, (char)150, (char)3, (char)232, (char)239, (char)181, (char)220, (char)67, (char)56, (char)68, (char)114, (char)153, (char)200, (char)54, (char)185, (char)161, (char)254, (char)229, (char)51, (char)170, (char)18, (char)12, (char)59, (char)160, (char)111, (char)68, (char)68, (char)98, (char)2, (char)160, (char)76, (char)22, (char)4, (char)103, (char)192, (char)96, (char)227, (char)176, (char)64, (char)7, (char)126, (char)250, (char)153, (char)7, (char)187, (char)127, (char)129, (char)190, (char)114, (char)45, (char)199, (char)172, (char)52, (char)199, (char)44, (char)132, (char)201, (char)16, (char)181, (char)180, (char)101, (char)58, (char)180, (char)161, (char)10, (char)70, (char)186, (char)224, (char)15, (char)80, (char)147, (char)243, (char)61, (char)249, (char)191, (char)38, (char)37, (char)144, (char)186, (char)167, (char)137}));
            assert(pack.transfer_type_GET() == (char)163);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)81, (char)170, (char)38, (char)9, (char)92, (char)62, (char)48, (char)131, (char)58, (char)80, (char)95, (char)201, (char)80, (char)60, (char)125, (char)242, (char)100, (char)60, (char)120, (char)76, (char)73, (char)195, (char)72, (char)134, (char)171, (char)201, (char)79, (char)217, (char)154, (char)200, (char)180, (char)185, (char)229, (char)112, (char)11, (char)246, (char)99, (char)20, (char)106, (char)253, (char)120, (char)7, (char)237, (char)84, (char)59, (char)61, (char)156, (char)63, (char)184, (char)126, (char)130, (char)216, (char)66, (char)156, (char)170, (char)255, (char)182, (char)96, (char)239, (char)26, (char)133, (char)78, (char)54, (char)205, (char)125, (char)141, (char)64, (char)26, (char)105, (char)103, (char)232, (char)231, (char)159, (char)34, (char)183, (char)115, (char)220, (char)152, (char)26, (char)143, (char)154, (char)42, (char)228, (char)68, (char)75, (char)117, (char)233, (char)187, (char)180, (char)251, (char)125, (char)92, (char)183, (char)137, (char)219, (char)196, (char)4, (char)102, (char)130, (char)244, (char)207, (char)154, (char)202, (char)3, (char)140, (char)4, (char)26, (char)92, (char)245, (char)168, (char)24, (char)39, (char)163, (char)227, (char)147, (char)64, (char)22, (char)139, (char)134, (char)175}));
            assert(pack.request_id_GET() == (char)174);
            assert(pack.uri_type_GET() == (char)200);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)174) ;
        p142.storage_SET(new char[] {(char)37, (char)42, (char)162, (char)46, (char)242, (char)55, (char)245, (char)35, (char)25, (char)29, (char)209, (char)170, (char)60, (char)131, (char)180, (char)254, (char)75, (char)249, (char)168, (char)193, (char)206, (char)0, (char)15, (char)152, (char)161, (char)188, (char)10, (char)166, (char)158, (char)109, (char)229, (char)23, (char)211, (char)46, (char)207, (char)15, (char)217, (char)243, (char)163, (char)244, (char)150, (char)3, (char)232, (char)239, (char)181, (char)220, (char)67, (char)56, (char)68, (char)114, (char)153, (char)200, (char)54, (char)185, (char)161, (char)254, (char)229, (char)51, (char)170, (char)18, (char)12, (char)59, (char)160, (char)111, (char)68, (char)68, (char)98, (char)2, (char)160, (char)76, (char)22, (char)4, (char)103, (char)192, (char)96, (char)227, (char)176, (char)64, (char)7, (char)126, (char)250, (char)153, (char)7, (char)187, (char)127, (char)129, (char)190, (char)114, (char)45, (char)199, (char)172, (char)52, (char)199, (char)44, (char)132, (char)201, (char)16, (char)181, (char)180, (char)101, (char)58, (char)180, (char)161, (char)10, (char)70, (char)186, (char)224, (char)15, (char)80, (char)147, (char)243, (char)61, (char)249, (char)191, (char)38, (char)37, (char)144, (char)186, (char)167, (char)137}, 0) ;
        p142.uri_SET(new char[] {(char)81, (char)170, (char)38, (char)9, (char)92, (char)62, (char)48, (char)131, (char)58, (char)80, (char)95, (char)201, (char)80, (char)60, (char)125, (char)242, (char)100, (char)60, (char)120, (char)76, (char)73, (char)195, (char)72, (char)134, (char)171, (char)201, (char)79, (char)217, (char)154, (char)200, (char)180, (char)185, (char)229, (char)112, (char)11, (char)246, (char)99, (char)20, (char)106, (char)253, (char)120, (char)7, (char)237, (char)84, (char)59, (char)61, (char)156, (char)63, (char)184, (char)126, (char)130, (char)216, (char)66, (char)156, (char)170, (char)255, (char)182, (char)96, (char)239, (char)26, (char)133, (char)78, (char)54, (char)205, (char)125, (char)141, (char)64, (char)26, (char)105, (char)103, (char)232, (char)231, (char)159, (char)34, (char)183, (char)115, (char)220, (char)152, (char)26, (char)143, (char)154, (char)42, (char)228, (char)68, (char)75, (char)117, (char)233, (char)187, (char)180, (char)251, (char)125, (char)92, (char)183, (char)137, (char)219, (char)196, (char)4, (char)102, (char)130, (char)244, (char)207, (char)154, (char)202, (char)3, (char)140, (char)4, (char)26, (char)92, (char)245, (char)168, (char)24, (char)39, (char)163, (char)227, (char)147, (char)64, (char)22, (char)139, (char)134, (char)175}, 0) ;
        p142.transfer_type_SET((char)163) ;
        p142.uri_type_SET((char)200) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -6500);
            assert(pack.press_diff_GET() == -5.2184546E37F);
            assert(pack.time_boot_ms_GET() == 4061680879L);
            assert(pack.press_abs_GET() == 2.672388E38F);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.temperature_SET((short) -6500) ;
        p143.press_diff_SET(-5.2184546E37F) ;
        p143.time_boot_ms_SET(4061680879L) ;
        p143.press_abs_SET(2.672388E38F) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.acc_GET(),  new float[] {2.8508772E38F, -1.8812162E38F, 2.7763672E38F}));
            assert(Arrays.equals(pack.vel_GET(),  new float[] {-1.7847663E38F, 2.1859063E38F, 3.1876356E38F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-1.0378628E37F, -3.591334E37F, -2.784142E38F}));
            assert(pack.timestamp_GET() == 8657179753394966792L);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-1.6122789E38F, 1.6255794E38F, -3.1698289E38F, -1.0423411E38F}));
            assert(pack.lat_GET() == -1618380105);
            assert(pack.est_capabilities_GET() == (char)16);
            assert(pack.alt_GET() == 5.565396E37F);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-1.7218462E38F, 1.3393246E38F, -1.0389212E38F}));
            assert(pack.custom_state_GET() == 1898507030684004997L);
            assert(pack.lon_GET() == 1815706008);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.lon_SET(1815706008) ;
        p144.acc_SET(new float[] {2.8508772E38F, -1.8812162E38F, 2.7763672E38F}, 0) ;
        p144.lat_SET(-1618380105) ;
        p144.timestamp_SET(8657179753394966792L) ;
        p144.position_cov_SET(new float[] {-1.7218462E38F, 1.3393246E38F, -1.0389212E38F}, 0) ;
        p144.alt_SET(5.565396E37F) ;
        p144.rates_SET(new float[] {-1.0378628E37F, -3.591334E37F, -2.784142E38F}, 0) ;
        p144.attitude_q_SET(new float[] {-1.6122789E38F, 1.6255794E38F, -3.1698289E38F, -1.0423411E38F}, 0) ;
        p144.vel_SET(new float[] {-1.7847663E38F, 2.1859063E38F, 3.1876356E38F}, 0) ;
        p144.est_capabilities_SET((char)16) ;
        p144.custom_state_SET(1898507030684004997L) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.x_acc_GET() == 1.9392306E37F);
            assert(pack.z_pos_GET() == -8.513947E37F);
            assert(pack.z_acc_GET() == 3.0005878E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-1.1254765E38F, -2.5437099E38F, 2.2540193E38F}));
            assert(pack.x_pos_GET() == 8.458483E37F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {2.0877416E38F, 1.802541E38F, 7.241753E37F}));
            assert(pack.y_vel_GET() == 1.3093638E38F);
            assert(pack.z_vel_GET() == 5.833084E37F);
            assert(pack.time_usec_GET() == 5289208721437533239L);
            assert(pack.y_acc_GET() == 1.085471E38F);
            assert(pack.pitch_rate_GET() == -2.8030596E38F);
            assert(pack.yaw_rate_GET() == -1.4692983E38F);
            assert(pack.roll_rate_GET() == 1.6604862E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.5395206E38F, -2.3356752E38F, 7.5876783E37F, -1.028478E37F}));
            assert(pack.y_pos_GET() == -1.4109945E38F);
            assert(pack.x_vel_GET() == 2.4091728E38F);
            assert(pack.airspeed_GET() == 9.049085E37F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.x_acc_SET(1.9392306E37F) ;
        p146.pitch_rate_SET(-2.8030596E38F) ;
        p146.y_vel_SET(1.3093638E38F) ;
        p146.z_vel_SET(5.833084E37F) ;
        p146.airspeed_SET(9.049085E37F) ;
        p146.x_vel_SET(2.4091728E38F) ;
        p146.time_usec_SET(5289208721437533239L) ;
        p146.y_acc_SET(1.085471E38F) ;
        p146.q_SET(new float[] {2.5395206E38F, -2.3356752E38F, 7.5876783E37F, -1.028478E37F}, 0) ;
        p146.yaw_rate_SET(-1.4692983E38F) ;
        p146.x_pos_SET(8.458483E37F) ;
        p146.roll_rate_SET(1.6604862E38F) ;
        p146.vel_variance_SET(new float[] {-1.1254765E38F, -2.5437099E38F, 2.2540193E38F}, 0) ;
        p146.y_pos_SET(-1.4109945E38F) ;
        p146.pos_variance_SET(new float[] {2.0877416E38F, 1.802541E38F, 7.241753E37F}, 0) ;
        p146.z_acc_SET(3.0005878E38F) ;
        p146.z_pos_SET(-8.513947E37F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.current_consumed_GET() == 191473578);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)32586, (char)24048, (char)27952, (char)62836, (char)48429, (char)27134, (char)34714, (char)9803, (char)48470, (char)32306}));
            assert(pack.battery_remaining_GET() == (byte) - 88);
            assert(pack.current_battery_GET() == (short) -4419);
            assert(pack.temperature_GET() == (short) -26225);
            assert(pack.energy_consumed_GET() == -680801975);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
            assert(pack.id_GET() == (char)235);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.energy_consumed_SET(-680801975) ;
        p147.battery_remaining_SET((byte) - 88) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION) ;
        p147.id_SET((char)235) ;
        p147.voltages_SET(new char[] {(char)32586, (char)24048, (char)27952, (char)62836, (char)48429, (char)27134, (char)34714, (char)9803, (char)48470, (char)32306}, 0) ;
        p147.current_consumed_SET(191473578) ;
        p147.temperature_SET((short) -26225) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN) ;
        p147.current_battery_SET((short) -4419) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.flight_sw_version_GET() == 612508184L);
            assert(pack.uid_GET() == 981202907113431522L);
            assert(pack.product_id_GET() == (char)36389);
            assert(pack.board_version_GET() == 8026487L);
            assert(pack.vendor_id_GET() == (char)53991);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)39, (char)76, (char)144, (char)11, (char)90, (char)86, (char)161, (char)126, (char)149, (char)71, (char)80, (char)99, (char)156, (char)80, (char)217, (char)117, (char)250, (char)114}));
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT));
            assert(pack.middleware_sw_version_GET() == 320056843L);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)254, (char)171, (char)41, (char)9, (char)47, (char)42, (char)220, (char)199}));
            assert(pack.os_sw_version_GET() == 3586379544L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)36, (char)191, (char)28, (char)123, (char)18, (char)214, (char)63, (char)164}));
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)46, (char)142, (char)145, (char)54, (char)16, (char)148, (char)199, (char)103}));
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.os_sw_version_SET(3586379544L) ;
        p148.uid_SET(981202907113431522L) ;
        p148.flight_custom_version_SET(new char[] {(char)46, (char)142, (char)145, (char)54, (char)16, (char)148, (char)199, (char)103}, 0) ;
        p148.board_version_SET(8026487L) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT)) ;
        p148.vendor_id_SET((char)53991) ;
        p148.flight_sw_version_SET(612508184L) ;
        p148.middleware_sw_version_SET(320056843L) ;
        p148.middleware_custom_version_SET(new char[] {(char)254, (char)171, (char)41, (char)9, (char)47, (char)42, (char)220, (char)199}, 0) ;
        p148.os_custom_version_SET(new char[] {(char)36, (char)191, (char)28, (char)123, (char)18, (char)214, (char)63, (char)164}, 0) ;
        p148.uid2_SET(new char[] {(char)39, (char)76, (char)144, (char)11, (char)90, (char)86, (char)161, (char)126, (char)149, (char)71, (char)80, (char)99, (char)156, (char)80, (char)217, (char)117, (char)250, (char)114}, 0, PH) ;
        p148.product_id_SET((char)36389) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.size_y_GET() == 3.2160874E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
            assert(pack.target_num_GET() == (char)110);
            assert(pack.x_TRY(ph) == 2.2827781E38F);
            assert(pack.distance_GET() == 1.5920577E38F);
            assert(pack.z_TRY(ph) == 2.3999664E38F);
            assert(pack.angle_x_GET() == -3.2564041E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {-3.1264963E38F, -2.459825E36F, -2.2560167E38F, 1.0441635E38F}));
            assert(pack.size_x_GET() == -2.1576602E38F);
            assert(pack.y_TRY(ph) == 2.6876387E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.time_usec_GET() == 4892465928149448787L);
            assert(pack.angle_y_GET() == 2.7886146E37F);
            assert(pack.position_valid_TRY(ph) == (char)24);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.distance_SET(1.5920577E38F) ;
        p149.time_usec_SET(4892465928149448787L) ;
        p149.target_num_SET((char)110) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON) ;
        p149.size_y_SET(3.2160874E38F) ;
        p149.angle_y_SET(2.7886146E37F) ;
        p149.y_SET(2.6876387E38F, PH) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p149.z_SET(2.3999664E38F, PH) ;
        p149.q_SET(new float[] {-3.1264963E38F, -2.459825E36F, -2.2560167E38F, 1.0441635E38F}, 0, PH) ;
        p149.position_valid_SET((char)24, PH) ;
        p149.angle_x_SET(-3.2564041E38F) ;
        p149.size_x_SET(-2.1576602E38F) ;
        p149.x_SET(2.2827781E38F, PH) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENS_POWER.add((src, ph, pack) ->
        {
            assert(pack.adc121_cs2_amp_GET() == -2.859592E38F);
            assert(pack.adc121_cspb_amp_GET() == 4.0539147E37F);
            assert(pack.adc121_vspb_volt_GET() == 2.3039058E37F);
            assert(pack.adc121_cs1_amp_GET() == 1.2716578E38F);
        });
        GroundControl.SENS_POWER p201 = CommunicationChannel.new_SENS_POWER();
        PH.setPack(p201);
        p201.adc121_cs1_amp_SET(1.2716578E38F) ;
        p201.adc121_cspb_amp_SET(4.0539147E37F) ;
        p201.adc121_vspb_volt_SET(2.3039058E37F) ;
        p201.adc121_cs2_amp_SET(-2.859592E38F) ;
        CommunicationChannel.instance.send(p201);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENS_MPPT.add((src, ph, pack) ->
        {
            assert(pack.mppt_timestamp_GET() == 7522802669594951528L);
            assert(pack.mppt1_pwm_GET() == (char)24695);
            assert(pack.mppt3_volt_GET() == 1.513369E38F);
            assert(pack.mppt3_amp_GET() == 2.1452591E38F);
            assert(pack.mppt2_volt_GET() == 2.7825497E38F);
            assert(pack.mppt1_amp_GET() == 1.5737799E38F);
            assert(pack.mppt2_pwm_GET() == (char)36368);
            assert(pack.mppt2_status_GET() == (char)184);
            assert(pack.mppt1_volt_GET() == -1.0991926E38F);
            assert(pack.mppt2_amp_GET() == 7.374594E37F);
            assert(pack.mppt1_status_GET() == (char)232);
            assert(pack.mppt3_status_GET() == (char)212);
            assert(pack.mppt3_pwm_GET() == (char)25004);
        });
        GroundControl.SENS_MPPT p202 = CommunicationChannel.new_SENS_MPPT();
        PH.setPack(p202);
        p202.mppt3_volt_SET(1.513369E38F) ;
        p202.mppt2_volt_SET(2.7825497E38F) ;
        p202.mppt2_status_SET((char)184) ;
        p202.mppt3_pwm_SET((char)25004) ;
        p202.mppt1_amp_SET(1.5737799E38F) ;
        p202.mppt1_pwm_SET((char)24695) ;
        p202.mppt1_volt_SET(-1.0991926E38F) ;
        p202.mppt3_amp_SET(2.1452591E38F) ;
        p202.mppt2_amp_SET(7.374594E37F) ;
        p202.mppt2_pwm_SET((char)36368) ;
        p202.mppt3_status_SET((char)212) ;
        p202.mppt_timestamp_SET(7522802669594951528L) ;
        p202.mppt1_status_SET((char)232) ;
        CommunicationChannel.instance.send(p202);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ASLCTRL_DATA.add((src, ph, pack) ->
        {
            assert(pack.p_GET() == -7.54995E37F);
            assert(pack.r_GET() == -1.7263949E38F);
            assert(pack.uThrot2_GET() == -1.7035792E38F);
            assert(pack.RollAngle_GET() == 1.2517305E38F);
            assert(pack.nZ_GET() == 1.8678057E38F);
            assert(pack.aslctrl_mode_GET() == (char)202);
            assert(pack.RollAngleRef_GET() == -6.2298445E37F);
            assert(pack.YawAngleRef_GET() == -4.7801426E37F);
            assert(pack.qRef_GET() == 7.427208E37F);
            assert(pack.PitchAngle_GET() == 1.1155595E38F);
            assert(pack.uThrot_GET() == -3.2332094E38F);
            assert(pack.uAil_GET() == -1.676811E38F);
            assert(pack.SpoilersEngaged_GET() == (char)188);
            assert(pack.hRef_GET() == -1.891688E38F);
            assert(pack.hRef_t_GET() == -1.4972751E38F);
            assert(pack.uElev_GET() == 9.139967E37F);
            assert(pack.PitchAngleRef_GET() == -2.8662178E38F);
            assert(pack.timestamp_GET() == 8204657160159613939L);
            assert(pack.uRud_GET() == -2.0426437E38F);
            assert(pack.pRef_GET() == 2.5512462E38F);
            assert(pack.rRef_GET() == -2.6707814E38F);
            assert(pack.h_GET() == -9.424568E37F);
            assert(pack.AirspeedRef_GET() == -2.8494881E37F);
            assert(pack.YawAngle_GET() == -5.904194E37F);
            assert(pack.q_GET() == 3.0298527E38F);
        });
        GroundControl.ASLCTRL_DATA p203 = CommunicationChannel.new_ASLCTRL_DATA();
        PH.setPack(p203);
        p203.h_SET(-9.424568E37F) ;
        p203.RollAngle_SET(1.2517305E38F) ;
        p203.nZ_SET(1.8678057E38F) ;
        p203.SpoilersEngaged_SET((char)188) ;
        p203.RollAngleRef_SET(-6.2298445E37F) ;
        p203.qRef_SET(7.427208E37F) ;
        p203.rRef_SET(-2.6707814E38F) ;
        p203.q_SET(3.0298527E38F) ;
        p203.timestamp_SET(8204657160159613939L) ;
        p203.aslctrl_mode_SET((char)202) ;
        p203.PitchAngle_SET(1.1155595E38F) ;
        p203.YawAngle_SET(-5.904194E37F) ;
        p203.uThrot2_SET(-1.7035792E38F) ;
        p203.uAil_SET(-1.676811E38F) ;
        p203.AirspeedRef_SET(-2.8494881E37F) ;
        p203.uElev_SET(9.139967E37F) ;
        p203.uThrot_SET(-3.2332094E38F) ;
        p203.hRef_SET(-1.891688E38F) ;
        p203.r_SET(-1.7263949E38F) ;
        p203.uRud_SET(-2.0426437E38F) ;
        p203.p_SET(-7.54995E37F) ;
        p203.hRef_t_SET(-1.4972751E38F) ;
        p203.YawAngleRef_SET(-4.7801426E37F) ;
        p203.PitchAngleRef_SET(-2.8662178E38F) ;
        p203.pRef_SET(2.5512462E38F) ;
        CommunicationChannel.instance.send(p203);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ASLCTRL_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.f_8_GET() == -1.863925E38F);
            assert(pack.f_6_GET() == -5.2374702E36F);
            assert(pack.f_1_GET() == 2.478094E37F);
            assert(pack.i32_1_GET() == 1299347293L);
            assert(pack.i8_1_GET() == (char)243);
            assert(pack.f_5_GET() == 1.5814053E37F);
            assert(pack.f_4_GET() == 1.6261522E38F);
            assert(pack.f_7_GET() == -1.6287452E38F);
            assert(pack.f_3_GET() == -2.2399135E38F);
            assert(pack.f_2_GET() == -1.0824205E38F);
            assert(pack.i8_2_GET() == (char)221);
        });
        GroundControl.ASLCTRL_DEBUG p204 = CommunicationChannel.new_ASLCTRL_DEBUG();
        PH.setPack(p204);
        p204.f_2_SET(-1.0824205E38F) ;
        p204.f_3_SET(-2.2399135E38F) ;
        p204.i8_1_SET((char)243) ;
        p204.f_1_SET(2.478094E37F) ;
        p204.f_5_SET(1.5814053E37F) ;
        p204.f_4_SET(1.6261522E38F) ;
        p204.f_6_SET(-5.2374702E36F) ;
        p204.f_8_SET(-1.863925E38F) ;
        p204.f_7_SET(-1.6287452E38F) ;
        p204.i8_2_SET((char)221) ;
        p204.i32_1_SET(1299347293L) ;
        CommunicationChannel.instance.send(p204);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ASLUAV_STATUS.add((src, ph, pack) ->
        {
            assert(pack.SATCOM_status_GET() == (char)223);
            assert(Arrays.equals(pack.Servo_status_GET(),  new char[] {(char)209, (char)151, (char)136, (char)75, (char)243, (char)186, (char)205, (char)131}));
            assert(pack.Motor_rpm_GET() == -2.2428293E38F);
            assert(pack.LED_status_GET() == (char)80);
        });
        GroundControl.ASLUAV_STATUS p205 = CommunicationChannel.new_ASLUAV_STATUS();
        PH.setPack(p205);
        p205.Motor_rpm_SET(-2.2428293E38F) ;
        p205.LED_status_SET((char)80) ;
        p205.SATCOM_status_SET((char)223) ;
        p205.Servo_status_SET(new char[] {(char)209, (char)151, (char)136, (char)75, (char)243, (char)186, (char)205, (char)131}, 0) ;
        CommunicationChannel.instance.send(p205);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_EKF_EXT.add((src, ph, pack) ->
        {
            assert(pack.Airspeed_GET() == 1.6354839E38F);
            assert(pack.beta_GET() == 1.1988985E38F);
            assert(pack.WindDir_GET() == -1.0468603E38F);
            assert(pack.timestamp_GET() == 4336894038406092037L);
            assert(pack.WindZ_GET() == -3.5431438E37F);
            assert(pack.alpha_GET() == -2.781612E38F);
            assert(pack.Windspeed_GET() == -4.625103E36F);
        });
        GroundControl.EKF_EXT p206 = CommunicationChannel.new_EKF_EXT();
        PH.setPack(p206);
        p206.alpha_SET(-2.781612E38F) ;
        p206.Airspeed_SET(1.6354839E38F) ;
        p206.beta_SET(1.1988985E38F) ;
        p206.WindZ_SET(-3.5431438E37F) ;
        p206.WindDir_SET(-1.0468603E38F) ;
        p206.Windspeed_SET(-4.625103E36F) ;
        p206.timestamp_SET(4336894038406092037L) ;
        CommunicationChannel.instance.send(p206);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ASL_OBCTRL.add((src, ph, pack) ->
        {
            assert(pack.uThrot_GET() == -1.9617436E37F);
            assert(pack.timestamp_GET() == 1464981332191850226L);
            assert(pack.obctrl_status_GET() == (char)193);
            assert(pack.uElev_GET() == 1.1777562E38F);
            assert(pack.uRud_GET() == 3.0895888E38F);
            assert(pack.uAilL_GET() == -2.7513606E38F);
            assert(pack.uThrot2_GET() == -6.056595E37F);
            assert(pack.uAilR_GET() == 3.3973369E38F);
        });
        GroundControl.ASL_OBCTRL p207 = CommunicationChannel.new_ASL_OBCTRL();
        PH.setPack(p207);
        p207.uRud_SET(3.0895888E38F) ;
        p207.uThrot2_SET(-6.056595E37F) ;
        p207.obctrl_status_SET((char)193) ;
        p207.uThrot_SET(-1.9617436E37F) ;
        p207.uAilR_SET(3.3973369E38F) ;
        p207.uAilL_SET(-2.7513606E38F) ;
        p207.timestamp_SET(1464981332191850226L) ;
        p207.uElev_SET(1.1777562E38F) ;
        CommunicationChannel.instance.send(p207);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENS_ATMOS.add((src, ph, pack) ->
        {
            assert(pack.TempAmbient_GET() == -9.377936E37F);
            assert(pack.Humidity_GET() == 2.2056153E38F);
        });
        GroundControl.SENS_ATMOS p208 = CommunicationChannel.new_SENS_ATMOS();
        PH.setPack(p208);
        p208.TempAmbient_SET(-9.377936E37F) ;
        p208.Humidity_SET(2.2056153E38F) ;
        CommunicationChannel.instance.send(p208);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENS_BATMON.add((src, ph, pack) ->
        {
            assert(pack.cellvoltage4_GET() == (char)60784);
            assert(pack.cellvoltage5_GET() == (char)36584);
            assert(pack.cellvoltage6_GET() == (char)17099);
            assert(pack.cellvoltage1_GET() == (char)33621);
            assert(pack.SoC_GET() == (char)157);
            assert(pack.voltage_GET() == (char)20813);
            assert(pack.cellvoltage3_GET() == (char)61886);
            assert(pack.serialnumber_GET() == (char)41751);
            assert(pack.current_GET() == (short) -1456);
            assert(pack.hostfetcontrol_GET() == (char)15522);
            assert(pack.temperature_GET() == 1.6165481E36F);
            assert(pack.cellvoltage2_GET() == (char)46007);
            assert(pack.batterystatus_GET() == (char)42257);
        });
        GroundControl.SENS_BATMON p209 = CommunicationChannel.new_SENS_BATMON();
        PH.setPack(p209);
        p209.cellvoltage6_SET((char)17099) ;
        p209.current_SET((short) -1456) ;
        p209.SoC_SET((char)157) ;
        p209.batterystatus_SET((char)42257) ;
        p209.cellvoltage2_SET((char)46007) ;
        p209.cellvoltage4_SET((char)60784) ;
        p209.hostfetcontrol_SET((char)15522) ;
        p209.cellvoltage3_SET((char)61886) ;
        p209.serialnumber_SET((char)41751) ;
        p209.cellvoltage1_SET((char)33621) ;
        p209.temperature_SET(1.6165481E36F) ;
        p209.cellvoltage5_SET((char)36584) ;
        p209.voltage_SET((char)20813) ;
        CommunicationChannel.instance.send(p209);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FW_SOARING_DATA.add((src, ph, pack) ->
        {
            assert(pack.timestampModeChanged_GET() == 8357259663963329636L);
            assert(pack.VarR_GET() == -1.3764237E38F);
            assert(pack.TSE_dot_GET() == -9.504114E37F);
            assert(pack.vSinkExp_GET() == 3.1582083E38F);
            assert(pack.z2_DeltaRoll_GET() == -1.9979153E38F);
            assert(pack.VarLat_GET() == 1.0964684E38F);
            assert(pack.ThermalGSEast_GET() == -1.00570936E37F);
            assert(pack.z1_LocalUpdraftSpeed_GET() == -5.14245E37F);
            assert(pack.DebugVar2_GET() == -2.4967817E38F);
            assert(pack.xLat_GET() == -1.3163641E38F);
            assert(pack.xR_GET() == 2.1281706E38F);
            assert(pack.xLon_GET() == 9.264494E37F);
            assert(pack.LoiterRadius_GET() == 2.2488022E38F);
            assert(pack.DistToSoarPoint_GET() == -1.8409019E38F);
            assert(pack.LoiterDirection_GET() == 1.4057164E38F);
            assert(pack.ControlMode_GET() == (char)191);
            assert(pack.xW_GET() == 3.7513693E37F);
            assert(pack.valid_GET() == (char)193);
            assert(pack.timestamp_GET() == 2754796731181041151L);
            assert(pack.DebugVar1_GET() == 2.6312798E38F);
            assert(pack.VarW_GET() == -1.8328997E37F);
            assert(pack.z1_exp_GET() == 1.7991615E38F);
            assert(pack.VarLon_GET() == 6.517959E37F);
            assert(pack.z2_exp_GET() == 1.9609979E38F);
            assert(pack.ThermalGSNorth_GET() == 5.6962285E37F);
        });
        GroundControl.FW_SOARING_DATA p210 = CommunicationChannel.new_FW_SOARING_DATA();
        PH.setPack(p210);
        p210.xR_SET(2.1281706E38F) ;
        p210.vSinkExp_SET(3.1582083E38F) ;
        p210.ThermalGSNorth_SET(5.6962285E37F) ;
        p210.xLat_SET(-1.3163641E38F) ;
        p210.timestampModeChanged_SET(8357259663963329636L) ;
        p210.DistToSoarPoint_SET(-1.8409019E38F) ;
        p210.VarW_SET(-1.8328997E37F) ;
        p210.timestamp_SET(2754796731181041151L) ;
        p210.LoiterRadius_SET(2.2488022E38F) ;
        p210.DebugVar2_SET(-2.4967817E38F) ;
        p210.VarLon_SET(6.517959E37F) ;
        p210.DebugVar1_SET(2.6312798E38F) ;
        p210.LoiterDirection_SET(1.4057164E38F) ;
        p210.valid_SET((char)193) ;
        p210.ControlMode_SET((char)191) ;
        p210.z1_LocalUpdraftSpeed_SET(-5.14245E37F) ;
        p210.z2_DeltaRoll_SET(-1.9979153E38F) ;
        p210.VarR_SET(-1.3764237E38F) ;
        p210.xW_SET(3.7513693E37F) ;
        p210.z2_exp_SET(1.9609979E38F) ;
        p210.z1_exp_SET(1.7991615E38F) ;
        p210.VarLat_SET(1.0964684E38F) ;
        p210.xLon_SET(9.264494E37F) ;
        p210.ThermalGSEast_SET(-1.00570936E37F) ;
        p210.TSE_dot_SET(-9.504114E37F) ;
        CommunicationChannel.instance.send(p210);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENSORPOD_STATUS.add((src, ph, pack) ->
        {
            assert(pack.free_space_GET() == (char)2497);
            assert(pack.visensor_rate_2_GET() == (char)108);
            assert(pack.recording_nodes_count_GET() == (char)201);
            assert(pack.visensor_rate_3_GET() == (char)105);
            assert(pack.visensor_rate_4_GET() == (char)126);
            assert(pack.timestamp_GET() == 8946706193122828497L);
            assert(pack.cpu_temp_GET() == (char)54);
            assert(pack.visensor_rate_1_GET() == (char)47);
        });
        GroundControl.SENSORPOD_STATUS p211 = CommunicationChannel.new_SENSORPOD_STATUS();
        PH.setPack(p211);
        p211.cpu_temp_SET((char)54) ;
        p211.timestamp_SET(8946706193122828497L) ;
        p211.recording_nodes_count_SET((char)201) ;
        p211.visensor_rate_3_SET((char)105) ;
        p211.free_space_SET((char)2497) ;
        p211.visensor_rate_4_SET((char)126) ;
        p211.visensor_rate_2_SET((char)108) ;
        p211.visensor_rate_1_SET((char)47) ;
        CommunicationChannel.instance.send(p211);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENS_POWER_BOARD.add((src, ph, pack) ->
        {
            assert(pack.pwr_brd_mot_r_amp_GET() == -2.3047539E38F);
            assert(pack.pwr_brd_system_volt_GET() == -7.356852E37F);
            assert(pack.pwr_brd_servo_volt_GET() == 1.3441442E37F);
            assert(pack.pwr_brd_servo_1_amp_GET() == -6.513678E37F);
            assert(pack.pwr_brd_status_GET() == (char)196);
            assert(pack.pwr_brd_servo_4_amp_GET() == -3.2604487E38F);
            assert(pack.pwr_brd_mot_l_amp_GET() == 2.0461931E38F);
            assert(pack.pwr_brd_servo_2_amp_GET() == -3.0140395E38F);
            assert(pack.pwr_brd_servo_3_amp_GET() == -3.2795778E38F);
            assert(pack.pwr_brd_aux_amp_GET() == -8.2993217E37F);
            assert(pack.pwr_brd_led_status_GET() == (char)34);
            assert(pack.timestamp_GET() == 655349404593994540L);
        });
        GroundControl.SENS_POWER_BOARD p212 = CommunicationChannel.new_SENS_POWER_BOARD();
        PH.setPack(p212);
        p212.pwr_brd_mot_r_amp_SET(-2.3047539E38F) ;
        p212.pwr_brd_servo_1_amp_SET(-6.513678E37F) ;
        p212.pwr_brd_aux_amp_SET(-8.2993217E37F) ;
        p212.pwr_brd_status_SET((char)196) ;
        p212.pwr_brd_mot_l_amp_SET(2.0461931E38F) ;
        p212.pwr_brd_servo_volt_SET(1.3441442E37F) ;
        p212.pwr_brd_servo_2_amp_SET(-3.0140395E38F) ;
        p212.pwr_brd_led_status_SET((char)34) ;
        p212.timestamp_SET(655349404593994540L) ;
        p212.pwr_brd_servo_3_amp_SET(-3.2795778E38F) ;
        p212.pwr_brd_system_volt_SET(-7.356852E37F) ;
        p212.pwr_brd_servo_4_amp_SET(-3.2604487E38F) ;
        CommunicationChannel.instance.send(p212);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pos_vert_accuracy_GET() == 4.7352054E37F);
            assert(pack.pos_horiz_accuracy_GET() == 2.6795807E38F);
            assert(pack.hagl_ratio_GET() == 1.146647E38F);
            assert(pack.mag_ratio_GET() == 3.1801792E38F);
            assert(pack.pos_horiz_ratio_GET() == -2.4953854E37F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE));
            assert(pack.pos_vert_ratio_GET() == 1.0076357E38F);
            assert(pack.vel_ratio_GET() == 2.0782736E38F);
            assert(pack.tas_ratio_GET() == -1.0662927E38F);
            assert(pack.time_usec_GET() == 5694776719230487346L);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.vel_ratio_SET(2.0782736E38F) ;
        p230.pos_horiz_accuracy_SET(2.6795807E38F) ;
        p230.mag_ratio_SET(3.1801792E38F) ;
        p230.pos_vert_accuracy_SET(4.7352054E37F) ;
        p230.tas_ratio_SET(-1.0662927E38F) ;
        p230.time_usec_SET(5694776719230487346L) ;
        p230.pos_horiz_ratio_SET(-2.4953854E37F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE)) ;
        p230.pos_vert_ratio_SET(1.0076357E38F) ;
        p230.hagl_ratio_SET(1.146647E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.var_horiz_GET() == 1.9415138E38F);
            assert(pack.wind_y_GET() == -2.3716351E38F);
            assert(pack.wind_alt_GET() == 2.4576636E38F);
            assert(pack.wind_z_GET() == -3.2690342E38F);
            assert(pack.time_usec_GET() == 8411232352780474730L);
            assert(pack.var_vert_GET() == 1.4600803E38F);
            assert(pack.wind_x_GET() == -1.2947543E38F);
            assert(pack.horiz_accuracy_GET() == -8.188701E36F);
            assert(pack.vert_accuracy_GET() == 9.695745E37F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_x_SET(-1.2947543E38F) ;
        p231.wind_z_SET(-3.2690342E38F) ;
        p231.horiz_accuracy_SET(-8.188701E36F) ;
        p231.var_vert_SET(1.4600803E38F) ;
        p231.wind_y_SET(-2.3716351E38F) ;
        p231.vert_accuracy_SET(9.695745E37F) ;
        p231.wind_alt_SET(2.4576636E38F) ;
        p231.time_usec_SET(8411232352780474730L) ;
        p231.var_horiz_SET(1.9415138E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.vert_accuracy_GET() == 1.3961772E38F);
            assert(pack.hdop_GET() == 8.3597627E37F);
            assert(pack.gps_id_GET() == (char)42);
            assert(pack.time_week_ms_GET() == 3735146122L);
            assert(pack.lon_GET() == -315547892);
            assert(pack.alt_GET() == -2.6548138E37F);
            assert(pack.time_usec_GET() == 7901980036535738625L);
            assert(pack.vdop_GET() == 9.902329E37F);
            assert(pack.fix_type_GET() == (char)66);
            assert(pack.ve_GET() == 3.0931017E38F);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT));
            assert(pack.time_week_GET() == (char)26243);
            assert(pack.satellites_visible_GET() == (char)115);
            assert(pack.lat_GET() == -1100581224);
            assert(pack.speed_accuracy_GET() == -2.8536249E38F);
            assert(pack.horiz_accuracy_GET() == 8.399415E37F);
            assert(pack.vd_GET() == 6.875074E37F);
            assert(pack.vn_GET() == 2.512141E38F);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT)) ;
        p232.fix_type_SET((char)66) ;
        p232.gps_id_SET((char)42) ;
        p232.time_week_SET((char)26243) ;
        p232.vd_SET(6.875074E37F) ;
        p232.time_week_ms_SET(3735146122L) ;
        p232.vn_SET(2.512141E38F) ;
        p232.lon_SET(-315547892) ;
        p232.horiz_accuracy_SET(8.399415E37F) ;
        p232.lat_SET(-1100581224) ;
        p232.ve_SET(3.0931017E38F) ;
        p232.vert_accuracy_SET(1.3961772E38F) ;
        p232.time_usec_SET(7901980036535738625L) ;
        p232.speed_accuracy_SET(-2.8536249E38F) ;
        p232.vdop_SET(9.902329E37F) ;
        p232.hdop_SET(8.3597627E37F) ;
        p232.alt_SET(-2.6548138E37F) ;
        p232.satellites_visible_SET((char)115) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (char)73);
            assert(pack.len_GET() == (char)179);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)150, (char)132, (char)165, (char)179, (char)81, (char)124, (char)246, (char)244, (char)226, (char)241, (char)113, (char)181, (char)16, (char)210, (char)19, (char)210, (char)34, (char)213, (char)93, (char)41, (char)173, (char)145, (char)27, (char)177, (char)114, (char)98, (char)241, (char)188, (char)229, (char)180, (char)130, (char)46, (char)173, (char)86, (char)85, (char)252, (char)33, (char)195, (char)220, (char)45, (char)197, (char)184, (char)153, (char)199, (char)204, (char)170, (char)16, (char)120, (char)162, (char)41, (char)197, (char)64, (char)192, (char)46, (char)186, (char)14, (char)44, (char)29, (char)64, (char)136, (char)157, (char)80, (char)8, (char)153, (char)189, (char)234, (char)186, (char)130, (char)66, (char)38, (char)112, (char)167, (char)206, (char)211, (char)51, (char)103, (char)221, (char)42, (char)228, (char)147, (char)216, (char)215, (char)7, (char)131, (char)11, (char)207, (char)5, (char)102, (char)186, (char)231, (char)58, (char)154, (char)35, (char)198, (char)117, (char)219, (char)253, (char)133, (char)76, (char)186, (char)99, (char)89, (char)163, (char)81, (char)139, (char)163, (char)45, (char)71, (char)71, (char)70, (char)238, (char)51, (char)8, (char)71, (char)163, (char)36, (char)21, (char)100, (char)255, (char)128, (char)112, (char)40, (char)198, (char)186, (char)176, (char)129, (char)59, (char)54, (char)151, (char)0, (char)72, (char)138, (char)19, (char)77, (char)179, (char)15, (char)78, (char)217, (char)97, (char)75, (char)124, (char)25, (char)136, (char)55, (char)77, (char)110, (char)122, (char)157, (char)149, (char)90, (char)196, (char)78, (char)115, (char)84, (char)254, (char)104, (char)192, (char)131, (char)180, (char)49, (char)56, (char)43, (char)52, (char)104, (char)19, (char)101, (char)166, (char)196, (char)68, (char)22, (char)8, (char)231, (char)249, (char)103, (char)46, (char)26, (char)243, (char)174, (char)223, (char)213}));
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.data__SET(new char[] {(char)150, (char)132, (char)165, (char)179, (char)81, (char)124, (char)246, (char)244, (char)226, (char)241, (char)113, (char)181, (char)16, (char)210, (char)19, (char)210, (char)34, (char)213, (char)93, (char)41, (char)173, (char)145, (char)27, (char)177, (char)114, (char)98, (char)241, (char)188, (char)229, (char)180, (char)130, (char)46, (char)173, (char)86, (char)85, (char)252, (char)33, (char)195, (char)220, (char)45, (char)197, (char)184, (char)153, (char)199, (char)204, (char)170, (char)16, (char)120, (char)162, (char)41, (char)197, (char)64, (char)192, (char)46, (char)186, (char)14, (char)44, (char)29, (char)64, (char)136, (char)157, (char)80, (char)8, (char)153, (char)189, (char)234, (char)186, (char)130, (char)66, (char)38, (char)112, (char)167, (char)206, (char)211, (char)51, (char)103, (char)221, (char)42, (char)228, (char)147, (char)216, (char)215, (char)7, (char)131, (char)11, (char)207, (char)5, (char)102, (char)186, (char)231, (char)58, (char)154, (char)35, (char)198, (char)117, (char)219, (char)253, (char)133, (char)76, (char)186, (char)99, (char)89, (char)163, (char)81, (char)139, (char)163, (char)45, (char)71, (char)71, (char)70, (char)238, (char)51, (char)8, (char)71, (char)163, (char)36, (char)21, (char)100, (char)255, (char)128, (char)112, (char)40, (char)198, (char)186, (char)176, (char)129, (char)59, (char)54, (char)151, (char)0, (char)72, (char)138, (char)19, (char)77, (char)179, (char)15, (char)78, (char)217, (char)97, (char)75, (char)124, (char)25, (char)136, (char)55, (char)77, (char)110, (char)122, (char)157, (char)149, (char)90, (char)196, (char)78, (char)115, (char)84, (char)254, (char)104, (char)192, (char)131, (char)180, (char)49, (char)56, (char)43, (char)52, (char)104, (char)19, (char)101, (char)166, (char)196, (char)68, (char)22, (char)8, (char)231, (char)249, (char)103, (char)46, (char)26, (char)243, (char)174, (char)223, (char)213}, 0) ;
        p233.len_SET((char)179) ;
        p233.flags_SET((char)73) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.wp_num_GET() == (char)112);
            assert(pack.longitude_GET() == 1793684364);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
            assert(pack.throttle_GET() == (byte) - 108);
            assert(pack.wp_distance_GET() == (char)26327);
            assert(pack.roll_GET() == (short) -17932);
            assert(pack.temperature_air_GET() == (byte) - 123);
            assert(pack.pitch_GET() == (short) -23606);
            assert(pack.airspeed_GET() == (char)51);
            assert(pack.altitude_sp_GET() == (short) -3044);
            assert(pack.groundspeed_GET() == (char)136);
            assert(pack.altitude_amsl_GET() == (short) -27670);
            assert(pack.temperature_GET() == (byte) - 93);
            assert(pack.failsafe_GET() == (char)79);
            assert(pack.battery_remaining_GET() == (char)58);
            assert(pack.custom_mode_GET() == 463006158L);
            assert(pack.climb_rate_GET() == (byte) - 69);
            assert(pack.airspeed_sp_GET() == (char)179);
            assert(pack.gps_nsat_GET() == (char)186);
            assert(pack.heading_sp_GET() == (short) -16552);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.heading_GET() == (char)62401);
            assert(pack.latitude_GET() == -1655691591);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.airspeed_SET((char)51) ;
        p234.throttle_SET((byte) - 108) ;
        p234.temperature_air_SET((byte) - 123) ;
        p234.groundspeed_SET((char)136) ;
        p234.wp_num_SET((char)112) ;
        p234.longitude_SET(1793684364) ;
        p234.heading_SET((char)62401) ;
        p234.climb_rate_SET((byte) - 69) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p234.airspeed_sp_SET((char)179) ;
        p234.battery_remaining_SET((char)58) ;
        p234.pitch_SET((short) -23606) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        p234.roll_SET((short) -17932) ;
        p234.wp_distance_SET((char)26327) ;
        p234.latitude_SET(-1655691591) ;
        p234.altitude_amsl_SET((short) -27670) ;
        p234.gps_nsat_SET((char)186) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED)) ;
        p234.custom_mode_SET(463006158L) ;
        p234.temperature_SET((byte) - 93) ;
        p234.altitude_sp_SET((short) -3044) ;
        p234.failsafe_SET((char)79) ;
        p234.heading_sp_SET((short) -16552) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_x_GET() == -3.2029697E38F);
            assert(pack.clipping_0_GET() == 924910922L);
            assert(pack.time_usec_GET() == 1087281118643243045L);
            assert(pack.vibration_y_GET() == -2.3834977E38F);
            assert(pack.clipping_1_GET() == 2641255257L);
            assert(pack.clipping_2_GET() == 1775150819L);
            assert(pack.vibration_z_GET() == -2.5238747E38F);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.vibration_z_SET(-2.5238747E38F) ;
        p241.clipping_2_SET(1775150819L) ;
        p241.clipping_0_SET(924910922L) ;
        p241.vibration_x_SET(-3.2029697E38F) ;
        p241.vibration_y_SET(-2.3834977E38F) ;
        p241.clipping_1_SET(2641255257L) ;
        p241.time_usec_SET(1087281118643243045L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == -1060320623);
            assert(pack.x_GET() == -8.2714917E37F);
            assert(pack.altitude_GET() == 333412636);
            assert(pack.y_GET() == 2.3724537E38F);
            assert(pack.time_usec_TRY(ph) == 6252979750540881868L);
            assert(pack.latitude_GET() == -330306254);
            assert(pack.approach_x_GET() == 3.7842828E36F);
            assert(pack.approach_z_GET() == 4.612117E37F);
            assert(pack.approach_y_GET() == -2.2779884E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {6.667957E37F, 1.3410415E38F, -2.0822798E38F, -1.0876376E38F}));
            assert(pack.z_GET() == 4.90892E37F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.y_SET(2.3724537E38F) ;
        p242.approach_x_SET(3.7842828E36F) ;
        p242.q_SET(new float[] {6.667957E37F, 1.3410415E38F, -2.0822798E38F, -1.0876376E38F}, 0) ;
        p242.x_SET(-8.2714917E37F) ;
        p242.latitude_SET(-330306254) ;
        p242.approach_y_SET(-2.2779884E38F) ;
        p242.longitude_SET(-1060320623) ;
        p242.altitude_SET(333412636) ;
        p242.z_SET(4.90892E37F) ;
        p242.time_usec_SET(6252979750540881868L, PH) ;
        p242.approach_z_SET(4.612117E37F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == 1620778741);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.6956352E37F, -1.6842945E38F, 2.3021695E38F, -1.8905216E38F}));
            assert(pack.target_system_GET() == (char)185);
            assert(pack.x_GET() == 2.0945087E38F);
            assert(pack.time_usec_TRY(ph) == 4867417343370514199L);
            assert(pack.altitude_GET() == 253320047);
            assert(pack.approach_y_GET() == -3.1139878E38F);
            assert(pack.y_GET() == -2.486648E38F);
            assert(pack.approach_z_GET() == 8.811261E36F);
            assert(pack.approach_x_GET() == 2.4477226E38F);
            assert(pack.z_GET() == 2.0332613E38F);
            assert(pack.latitude_GET() == -1233272836);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.y_SET(-2.486648E38F) ;
        p243.q_SET(new float[] {1.6956352E37F, -1.6842945E38F, 2.3021695E38F, -1.8905216E38F}, 0) ;
        p243.approach_z_SET(8.811261E36F) ;
        p243.target_system_SET((char)185) ;
        p243.approach_y_SET(-3.1139878E38F) ;
        p243.z_SET(2.0332613E38F) ;
        p243.approach_x_SET(2.4477226E38F) ;
        p243.altitude_SET(253320047) ;
        p243.time_usec_SET(4867417343370514199L, PH) ;
        p243.latitude_SET(-1233272836) ;
        p243.x_SET(2.0945087E38F) ;
        p243.longitude_SET(1620778741) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)53023);
            assert(pack.interval_us_GET() == -843326997);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-843326997) ;
        p244.message_id_SET((char)53023) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_MC) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.heading_GET() == (char)20713);
            assert(pack.altitude_GET() == -631775124);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.lon_GET() == 1862883118);
            assert(pack.hor_velocity_GET() == (char)19730);
            assert(pack.ver_velocity_GET() == (short) -1054);
            assert(pack.tslc_GET() == (char)98);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY));
            assert(pack.lat_GET() == -23762460);
            assert(pack.callsign_LEN(ph) == 8);
            assert(pack.callsign_TRY(ph).equals("wimtnIka"));
            assert(pack.squawk_GET() == (char)18131);
            assert(pack.ICAO_address_GET() == 3648272775L);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SPACE);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY)) ;
        p246.lon_SET(1862883118) ;
        p246.callsign_SET("wimtnIka", PH) ;
        p246.altitude_SET(-631775124) ;
        p246.tslc_SET((char)98) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SPACE) ;
        p246.ver_velocity_SET((short) -1054) ;
        p246.lat_SET(-23762460) ;
        p246.hor_velocity_SET((char)19730) ;
        p246.heading_SET((char)20713) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.squawk_SET((char)18131) ;
        p246.ICAO_address_SET(3648272775L) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            assert(pack.altitude_minimum_delta_GET() == 3.311698E37F);
            assert(pack.id_GET() == 4216174881L);
            assert(pack.horizontal_minimum_delta_GET() == -9.208719E37F);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT);
            assert(pack.time_to_minimum_delta_GET() == -1.3626057E38F);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.id_SET(4216174881L) ;
        p247.horizontal_minimum_delta_SET(-9.208719E37F) ;
        p247.altitude_minimum_delta_SET(3.311698E37F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.time_to_minimum_delta_SET(-1.3626057E38F) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.message_type_GET() == (char)36520);
            assert(pack.target_network_GET() == (char)115);
            assert(pack.target_component_GET() == (char)190);
            assert(pack.target_system_GET() == (char)209);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)10, (char)19, (char)232, (char)63, (char)80, (char)118, (char)207, (char)86, (char)178, (char)167, (char)166, (char)134, (char)225, (char)140, (char)21, (char)117, (char)59, (char)41, (char)61, (char)39, (char)218, (char)124, (char)146, (char)35, (char)81, (char)179, (char)152, (char)12, (char)242, (char)209, (char)181, (char)222, (char)20, (char)198, (char)234, (char)10, (char)89, (char)129, (char)71, (char)234, (char)69, (char)166, (char)160, (char)59, (char)28, (char)131, (char)144, (char)65, (char)21, (char)252, (char)87, (char)203, (char)85, (char)99, (char)136, (char)21, (char)137, (char)5, (char)48, (char)2, (char)159, (char)95, (char)133, (char)202, (char)243, (char)34, (char)97, (char)194, (char)69, (char)105, (char)92, (char)147, (char)135, (char)98, (char)38, (char)56, (char)170, (char)171, (char)213, (char)109, (char)30, (char)2, (char)184, (char)130, (char)117, (char)173, (char)167, (char)166, (char)185, (char)128, (char)58, (char)255, (char)28, (char)62, (char)97, (char)154, (char)223, (char)55, (char)209, (char)13, (char)211, (char)109, (char)62, (char)30, (char)137, (char)21, (char)45, (char)72, (char)253, (char)157, (char)196, (char)54, (char)92, (char)253, (char)114, (char)165, (char)193, (char)116, (char)239, (char)139, (char)74, (char)80, (char)119, (char)255, (char)188, (char)164, (char)181, (char)108, (char)121, (char)46, (char)180, (char)85, (char)176, (char)142, (char)252, (char)170, (char)29, (char)205, (char)34, (char)48, (char)162, (char)151, (char)25, (char)15, (char)153, (char)50, (char)109, (char)237, (char)86, (char)77, (char)209, (char)35, (char)15, (char)44, (char)109, (char)194, (char)122, (char)91, (char)98, (char)225, (char)30, (char)59, (char)224, (char)241, (char)173, (char)79, (char)13, (char)25, (char)243, (char)79, (char)238, (char)239, (char)188, (char)69, (char)72, (char)94, (char)1, (char)208, (char)85, (char)216, (char)163, (char)162, (char)85, (char)214, (char)86, (char)171, (char)19, (char)129, (char)130, (char)54, (char)60, (char)165, (char)100, (char)1, (char)128, (char)134, (char)136, (char)212, (char)4, (char)15, (char)17, (char)139, (char)185, (char)226, (char)201, (char)125, (char)171, (char)204, (char)32, (char)19, (char)111, (char)156, (char)139, (char)111, (char)4, (char)92, (char)14, (char)94, (char)89, (char)203, (char)101, (char)224, (char)156, (char)106, (char)18, (char)55, (char)223, (char)201, (char)13, (char)137, (char)8, (char)229, (char)102, (char)27, (char)193, (char)70, (char)156, (char)200, (char)72, (char)183, (char)34, (char)217, (char)232, (char)174, (char)218, (char)36, (char)144, (char)128, (char)28}));
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_component_SET((char)190) ;
        p248.target_system_SET((char)209) ;
        p248.target_network_SET((char)115) ;
        p248.message_type_SET((char)36520) ;
        p248.payload_SET(new char[] {(char)10, (char)19, (char)232, (char)63, (char)80, (char)118, (char)207, (char)86, (char)178, (char)167, (char)166, (char)134, (char)225, (char)140, (char)21, (char)117, (char)59, (char)41, (char)61, (char)39, (char)218, (char)124, (char)146, (char)35, (char)81, (char)179, (char)152, (char)12, (char)242, (char)209, (char)181, (char)222, (char)20, (char)198, (char)234, (char)10, (char)89, (char)129, (char)71, (char)234, (char)69, (char)166, (char)160, (char)59, (char)28, (char)131, (char)144, (char)65, (char)21, (char)252, (char)87, (char)203, (char)85, (char)99, (char)136, (char)21, (char)137, (char)5, (char)48, (char)2, (char)159, (char)95, (char)133, (char)202, (char)243, (char)34, (char)97, (char)194, (char)69, (char)105, (char)92, (char)147, (char)135, (char)98, (char)38, (char)56, (char)170, (char)171, (char)213, (char)109, (char)30, (char)2, (char)184, (char)130, (char)117, (char)173, (char)167, (char)166, (char)185, (char)128, (char)58, (char)255, (char)28, (char)62, (char)97, (char)154, (char)223, (char)55, (char)209, (char)13, (char)211, (char)109, (char)62, (char)30, (char)137, (char)21, (char)45, (char)72, (char)253, (char)157, (char)196, (char)54, (char)92, (char)253, (char)114, (char)165, (char)193, (char)116, (char)239, (char)139, (char)74, (char)80, (char)119, (char)255, (char)188, (char)164, (char)181, (char)108, (char)121, (char)46, (char)180, (char)85, (char)176, (char)142, (char)252, (char)170, (char)29, (char)205, (char)34, (char)48, (char)162, (char)151, (char)25, (char)15, (char)153, (char)50, (char)109, (char)237, (char)86, (char)77, (char)209, (char)35, (char)15, (char)44, (char)109, (char)194, (char)122, (char)91, (char)98, (char)225, (char)30, (char)59, (char)224, (char)241, (char)173, (char)79, (char)13, (char)25, (char)243, (char)79, (char)238, (char)239, (char)188, (char)69, (char)72, (char)94, (char)1, (char)208, (char)85, (char)216, (char)163, (char)162, (char)85, (char)214, (char)86, (char)171, (char)19, (char)129, (char)130, (char)54, (char)60, (char)165, (char)100, (char)1, (char)128, (char)134, (char)136, (char)212, (char)4, (char)15, (char)17, (char)139, (char)185, (char)226, (char)201, (char)125, (char)171, (char)204, (char)32, (char)19, (char)111, (char)156, (char)139, (char)111, (char)4, (char)92, (char)14, (char)94, (char)89, (char)203, (char)101, (char)224, (char)156, (char)106, (char)18, (char)55, (char)223, (char)201, (char)13, (char)137, (char)8, (char)229, (char)102, (char)27, (char)193, (char)70, (char)156, (char)200, (char)72, (char)183, (char)34, (char)217, (char)232, (char)174, (char)218, (char)36, (char)144, (char)128, (char)28}, 0) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 8, (byte) - 107, (byte) - 9, (byte)65, (byte)70, (byte) - 5, (byte)64, (byte)107, (byte)21, (byte)36, (byte) - 18, (byte)117, (byte)12, (byte)113, (byte)89, (byte) - 40, (byte) - 30, (byte) - 28, (byte)23, (byte) - 19, (byte) - 37, (byte)97, (byte)119, (byte)12, (byte) - 83, (byte)95, (byte) - 25, (byte)80, (byte)16, (byte)17, (byte)3, (byte) - 68}));
            assert(pack.ver_GET() == (char)86);
            assert(pack.address_GET() == (char)56260);
            assert(pack.type_GET() == (char)236);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)56260) ;
        p249.value_SET(new byte[] {(byte) - 8, (byte) - 107, (byte) - 9, (byte)65, (byte)70, (byte) - 5, (byte)64, (byte)107, (byte)21, (byte)36, (byte) - 18, (byte)117, (byte)12, (byte)113, (byte)89, (byte) - 40, (byte) - 30, (byte) - 28, (byte)23, (byte) - 19, (byte) - 37, (byte)97, (byte)119, (byte)12, (byte) - 83, (byte)95, (byte) - 25, (byte)80, (byte)16, (byte)17, (byte)3, (byte) - 68}, 0) ;
        p249.ver_SET((char)86) ;
        p249.type_SET((char)236) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -7.112488E35F);
            assert(pack.x_GET() == -2.2100728E38F);
            assert(pack.z_GET() == 4.225205E37F);
            assert(pack.name_LEN(ph) == 1);
            assert(pack.name_TRY(ph).equals("b"));
            assert(pack.time_usec_GET() == 2592416814900898848L);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.y_SET(-7.112488E35F) ;
        p250.x_SET(-2.2100728E38F) ;
        p250.time_usec_SET(2592416814900898848L) ;
        p250.z_SET(4.225205E37F) ;
        p250.name_SET("b", PH) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 2);
            assert(pack.name_TRY(ph).equals("ad"));
            assert(pack.time_boot_ms_GET() == 99172519L);
            assert(pack.value_GET() == 1.4059052E38F);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(1.4059052E38F) ;
        p251.time_boot_ms_SET(99172519L) ;
        p251.name_SET("ad", PH) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 6);
            assert(pack.name_TRY(ph).equals("hiDraq"));
            assert(pack.time_boot_ms_GET() == 890169496L);
            assert(pack.value_GET() == -1639998809);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.name_SET("hiDraq", PH) ;
        p252.value_SET(-1639998809) ;
        p252.time_boot_ms_SET(890169496L) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 20);
            assert(pack.text_TRY(ph).equals("urhrwjrdqoXwocziibrn"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_INFO);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("urhrwjrdqoXwocziibrn", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_INFO) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.ind_GET() == (char)120);
            assert(pack.value_GET() == 7.054009E37F);
            assert(pack.time_boot_ms_GET() == 4117141149L);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(4117141149L) ;
        p254.value_SET(7.054009E37F) ;
        p254.ind_SET((char)120) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)166);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)9, (char)236, (char)225, (char)129, (char)191, (char)16, (char)158, (char)59, (char)26, (char)129, (char)159, (char)196, (char)96, (char)110, (char)235, (char)194, (char)186, (char)45, (char)32, (char)229, (char)117, (char)235, (char)191, (char)47, (char)47, (char)96, (char)42, (char)68, (char)123, (char)84, (char)132, (char)55}));
            assert(pack.initial_timestamp_GET() == 5879928158067415822L);
            assert(pack.target_component_GET() == (char)172);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)166) ;
        p256.initial_timestamp_SET(5879928158067415822L) ;
        p256.target_component_SET((char)172) ;
        p256.secret_key_SET(new char[] {(char)9, (char)236, (char)225, (char)129, (char)191, (char)16, (char)158, (char)59, (char)26, (char)129, (char)159, (char)196, (char)96, (char)110, (char)235, (char)194, (char)186, (char)45, (char)32, (char)229, (char)117, (char)235, (char)191, (char)47, (char)47, (char)96, (char)42, (char)68, (char)123, (char)84, (char)132, (char)55}, 0) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.state_GET() == (char)52);
            assert(pack.last_change_ms_GET() == 3415617125L);
            assert(pack.time_boot_ms_GET() == 2951331368L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(2951331368L) ;
        p257.last_change_ms_SET(3415617125L) ;
        p257.state_SET((char)52) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.tune_LEN(ph) == 29);
            assert(pack.tune_TRY(ph).equals("cnxsrujbddnwlznbKcbduhdvxaWlq"));
            assert(pack.target_system_GET() == (char)223);
            assert(pack.target_component_GET() == (char)21);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)223) ;
        p258.target_component_SET((char)21) ;
        p258.tune_SET("cnxsrujbddnwlznbKcbduhdvxaWlq", PH) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.cam_definition_uri_LEN(ph) == 29);
            assert(pack.cam_definition_uri_TRY(ph).equals("zhojgNVbatzwlroyjyMarwnnngbsu"));
            assert(pack.sensor_size_v_GET() == -7.8673654E36F);
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
            assert(pack.firmware_version_GET() == 2343892306L);
            assert(pack.sensor_size_h_GET() == -3.4438626E37F);
            assert(pack.resolution_h_GET() == (char)62433);
            assert(pack.cam_definition_version_GET() == (char)45258);
            assert(pack.time_boot_ms_GET() == 4043031227L);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)11, (char)249, (char)150, (char)163, (char)174, (char)183, (char)144, (char)210, (char)244, (char)241, (char)194, (char)158, (char)136, (char)7, (char)57, (char)91, (char)230, (char)140, (char)173, (char)147, (char)210, (char)242, (char)204, (char)59, (char)63, (char)141, (char)185, (char)79, (char)139, (char)16, (char)203, (char)31}));
            assert(pack.focal_length_GET() == 1.7971278E38F);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)90, (char)13, (char)201, (char)54, (char)28, (char)219, (char)108, (char)26, (char)16, (char)162, (char)32, (char)160, (char)202, (char)197, (char)235, (char)21, (char)21, (char)237, (char)248, (char)193, (char)130, (char)105, (char)209, (char)57, (char)75, (char)30, (char)199, (char)108, (char)207, (char)206, (char)244, (char)88}));
            assert(pack.resolution_v_GET() == (char)56625);
            assert(pack.lens_id_GET() == (char)122);
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.vendor_name_SET(new char[] {(char)90, (char)13, (char)201, (char)54, (char)28, (char)219, (char)108, (char)26, (char)16, (char)162, (char)32, (char)160, (char)202, (char)197, (char)235, (char)21, (char)21, (char)237, (char)248, (char)193, (char)130, (char)105, (char)209, (char)57, (char)75, (char)30, (char)199, (char)108, (char)207, (char)206, (char)244, (char)88}, 0) ;
        p259.resolution_h_SET((char)62433) ;
        p259.firmware_version_SET(2343892306L) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE)) ;
        p259.resolution_v_SET((char)56625) ;
        p259.lens_id_SET((char)122) ;
        p259.sensor_size_v_SET(-7.8673654E36F) ;
        p259.focal_length_SET(1.7971278E38F) ;
        p259.sensor_size_h_SET(-3.4438626E37F) ;
        p259.time_boot_ms_SET(4043031227L) ;
        p259.cam_definition_uri_SET("zhojgNVbatzwlroyjyMarwnnngbsu", PH) ;
        p259.model_name_SET(new char[] {(char)11, (char)249, (char)150, (char)163, (char)174, (char)183, (char)144, (char)210, (char)244, (char)241, (char)194, (char)158, (char)136, (char)7, (char)57, (char)91, (char)230, (char)140, (char)173, (char)147, (char)210, (char)242, (char)204, (char)59, (char)63, (char)141, (char)185, (char)79, (char)139, (char)16, (char)203, (char)31}, 0) ;
        p259.cam_definition_version_SET((char)45258) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1944177923L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(1944177923L) ;
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.available_capacity_GET() == -2.642876E36F);
            assert(pack.used_capacity_GET() == -3.018148E38F);
            assert(pack.status_GET() == (char)138);
            assert(pack.storage_id_GET() == (char)143);
            assert(pack.read_speed_GET() == 1.8593426E38F);
            assert(pack.write_speed_GET() == 3.1163557E38F);
            assert(pack.storage_count_GET() == (char)89);
            assert(pack.time_boot_ms_GET() == 4243046451L);
            assert(pack.total_capacity_GET() == -1.7145776E38F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.used_capacity_SET(-3.018148E38F) ;
        p261.storage_count_SET((char)89) ;
        p261.write_speed_SET(3.1163557E38F) ;
        p261.status_SET((char)138) ;
        p261.storage_id_SET((char)143) ;
        p261.total_capacity_SET(-1.7145776E38F) ;
        p261.time_boot_ms_SET(4243046451L) ;
        p261.available_capacity_SET(-2.642876E36F) ;
        p261.read_speed_SET(1.8593426E38F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.image_status_GET() == (char)122);
            assert(pack.recording_time_ms_GET() == 60696145L);
            assert(pack.time_boot_ms_GET() == 1082655671L);
            assert(pack.available_capacity_GET() == 1.5955704E38F);
            assert(pack.video_status_GET() == (char)83);
            assert(pack.image_interval_GET() == 2.1591818E38F);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.available_capacity_SET(1.5955704E38F) ;
        p262.image_interval_SET(2.1591818E38F) ;
        p262.image_status_SET((char)122) ;
        p262.recording_time_ms_SET(60696145L) ;
        p262.time_boot_ms_SET(1082655671L) ;
        p262.video_status_SET((char)83) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.file_url_LEN(ph) == 23);
            assert(pack.file_url_TRY(ph).equals("WztffcLBwvKbjsncwlrhzgs"));
            assert(pack.relative_alt_GET() == 1261832811);
            assert(pack.time_utc_GET() == 5489117823380153710L);
            assert(pack.alt_GET() == -1313438194);
            assert(pack.time_boot_ms_GET() == 2658738213L);
            assert(pack.lat_GET() == -246973275);
            assert(pack.lon_GET() == -1675114848);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.4970597E38F, 2.6002832E38F, 3.0476416E38F, -2.9660905E38F}));
            assert(pack.image_index_GET() == 1720421842);
            assert(pack.capture_result_GET() == (byte) - 36);
            assert(pack.camera_id_GET() == (char)152);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_utc_SET(5489117823380153710L) ;
        p263.lat_SET(-246973275) ;
        p263.alt_SET(-1313438194) ;
        p263.q_SET(new float[] {-1.4970597E38F, 2.6002832E38F, 3.0476416E38F, -2.9660905E38F}, 0) ;
        p263.image_index_SET(1720421842) ;
        p263.camera_id_SET((char)152) ;
        p263.capture_result_SET((byte) - 36) ;
        p263.file_url_SET("WztffcLBwvKbjsncwlrhzgs", PH) ;
        p263.lon_SET(-1675114848) ;
        p263.relative_alt_SET(1261832811) ;
        p263.time_boot_ms_SET(2658738213L) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.arming_time_utc_GET() == 4811460415013859511L);
            assert(pack.flight_uuid_GET() == 19801956178089247L);
            assert(pack.time_boot_ms_GET() == 1021239583L);
            assert(pack.takeoff_time_utc_GET() == 2656056614713492820L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.arming_time_utc_SET(4811460415013859511L) ;
        p264.flight_uuid_SET(19801956178089247L) ;
        p264.time_boot_ms_SET(1021239583L) ;
        p264.takeoff_time_utc_SET(2656056614713492820L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 340217481L);
            assert(pack.yaw_GET() == 2.210382E38F);
            assert(pack.roll_GET() == -9.392273E37F);
            assert(pack.pitch_GET() == -1.2355601E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(-9.392273E37F) ;
        p265.yaw_SET(2.210382E38F) ;
        p265.time_boot_ms_SET(340217481L) ;
        p265.pitch_SET(-1.2355601E38F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.first_message_offset_GET() == (char)94);
            assert(pack.sequence_GET() == (char)61180);
            assert(pack.length_GET() == (char)43);
            assert(pack.target_component_GET() == (char)6);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)140, (char)28, (char)16, (char)149, (char)153, (char)230, (char)27, (char)9, (char)240, (char)192, (char)118, (char)187, (char)62, (char)145, (char)145, (char)159, (char)54, (char)104, (char)87, (char)135, (char)254, (char)82, (char)3, (char)152, (char)134, (char)1, (char)27, (char)164, (char)136, (char)253, (char)113, (char)210, (char)177, (char)193, (char)16, (char)39, (char)189, (char)136, (char)59, (char)128, (char)152, (char)91, (char)115, (char)132, (char)14, (char)120, (char)84, (char)199, (char)10, (char)165, (char)54, (char)170, (char)44, (char)82, (char)39, (char)180, (char)117, (char)177, (char)79, (char)225, (char)199, (char)202, (char)145, (char)194, (char)203, (char)158, (char)251, (char)24, (char)120, (char)128, (char)110, (char)63, (char)156, (char)249, (char)218, (char)213, (char)75, (char)85, (char)161, (char)255, (char)183, (char)254, (char)219, (char)115, (char)179, (char)128, (char)245, (char)147, (char)201, (char)14, (char)242, (char)189, (char)161, (char)77, (char)79, (char)254, (char)152, (char)223, (char)255, (char)21, (char)190, (char)15, (char)105, (char)245, (char)137, (char)85, (char)205, (char)212, (char)69, (char)30, (char)255, (char)19, (char)49, (char)160, (char)250, (char)97, (char)90, (char)184, (char)16, (char)111, (char)78, (char)249, (char)41, (char)41, (char)161, (char)72, (char)123, (char)18, (char)37, (char)125, (char)232, (char)92, (char)169, (char)77, (char)233, (char)247, (char)211, (char)99, (char)21, (char)245, (char)153, (char)151, (char)158, (char)182, (char)24, (char)181, (char)52, (char)95, (char)138, (char)67, (char)55, (char)34, (char)6, (char)224, (char)252, (char)99, (char)44, (char)243, (char)131, (char)106, (char)234, (char)38, (char)205, (char)12, (char)76, (char)23, (char)182, (char)196, (char)54, (char)226, (char)186, (char)3, (char)51, (char)227, (char)42, (char)33, (char)151, (char)188, (char)8, (char)4, (char)60, (char)30, (char)66, (char)193, (char)114, (char)210, (char)196, (char)58, (char)41, (char)6, (char)219, (char)187, (char)69, (char)191, (char)219, (char)246, (char)248, (char)66, (char)214, (char)29, (char)185, (char)176, (char)145, (char)90, (char)62, (char)153, (char)200, (char)21, (char)222, (char)195, (char)164, (char)0, (char)165, (char)235, (char)248, (char)247, (char)178, (char)116, (char)191, (char)56, (char)51, (char)93, (char)152, (char)124, (char)116, (char)93, (char)120, (char)147, (char)37, (char)188, (char)203, (char)168, (char)99, (char)151, (char)171, (char)66, (char)0, (char)193, (char)18, (char)254, (char)83, (char)107, (char)66, (char)235, (char)225, (char)246, (char)3, (char)11, (char)227}));
            assert(pack.target_system_GET() == (char)95);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.first_message_offset_SET((char)94) ;
        p266.length_SET((char)43) ;
        p266.target_system_SET((char)95) ;
        p266.target_component_SET((char)6) ;
        p266.data__SET(new char[] {(char)140, (char)28, (char)16, (char)149, (char)153, (char)230, (char)27, (char)9, (char)240, (char)192, (char)118, (char)187, (char)62, (char)145, (char)145, (char)159, (char)54, (char)104, (char)87, (char)135, (char)254, (char)82, (char)3, (char)152, (char)134, (char)1, (char)27, (char)164, (char)136, (char)253, (char)113, (char)210, (char)177, (char)193, (char)16, (char)39, (char)189, (char)136, (char)59, (char)128, (char)152, (char)91, (char)115, (char)132, (char)14, (char)120, (char)84, (char)199, (char)10, (char)165, (char)54, (char)170, (char)44, (char)82, (char)39, (char)180, (char)117, (char)177, (char)79, (char)225, (char)199, (char)202, (char)145, (char)194, (char)203, (char)158, (char)251, (char)24, (char)120, (char)128, (char)110, (char)63, (char)156, (char)249, (char)218, (char)213, (char)75, (char)85, (char)161, (char)255, (char)183, (char)254, (char)219, (char)115, (char)179, (char)128, (char)245, (char)147, (char)201, (char)14, (char)242, (char)189, (char)161, (char)77, (char)79, (char)254, (char)152, (char)223, (char)255, (char)21, (char)190, (char)15, (char)105, (char)245, (char)137, (char)85, (char)205, (char)212, (char)69, (char)30, (char)255, (char)19, (char)49, (char)160, (char)250, (char)97, (char)90, (char)184, (char)16, (char)111, (char)78, (char)249, (char)41, (char)41, (char)161, (char)72, (char)123, (char)18, (char)37, (char)125, (char)232, (char)92, (char)169, (char)77, (char)233, (char)247, (char)211, (char)99, (char)21, (char)245, (char)153, (char)151, (char)158, (char)182, (char)24, (char)181, (char)52, (char)95, (char)138, (char)67, (char)55, (char)34, (char)6, (char)224, (char)252, (char)99, (char)44, (char)243, (char)131, (char)106, (char)234, (char)38, (char)205, (char)12, (char)76, (char)23, (char)182, (char)196, (char)54, (char)226, (char)186, (char)3, (char)51, (char)227, (char)42, (char)33, (char)151, (char)188, (char)8, (char)4, (char)60, (char)30, (char)66, (char)193, (char)114, (char)210, (char)196, (char)58, (char)41, (char)6, (char)219, (char)187, (char)69, (char)191, (char)219, (char)246, (char)248, (char)66, (char)214, (char)29, (char)185, (char)176, (char)145, (char)90, (char)62, (char)153, (char)200, (char)21, (char)222, (char)195, (char)164, (char)0, (char)165, (char)235, (char)248, (char)247, (char)178, (char)116, (char)191, (char)56, (char)51, (char)93, (char)152, (char)124, (char)116, (char)93, (char)120, (char)147, (char)37, (char)188, (char)203, (char)168, (char)99, (char)151, (char)171, (char)66, (char)0, (char)193, (char)18, (char)254, (char)83, (char)107, (char)66, (char)235, (char)225, (char)246, (char)3, (char)11, (char)227}, 0) ;
        p266.sequence_SET((char)61180) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)235, (char)33, (char)11, (char)40, (char)40, (char)232, (char)230, (char)10, (char)233, (char)205, (char)210, (char)170, (char)20, (char)152, (char)148, (char)154, (char)5, (char)116, (char)13, (char)194, (char)200, (char)176, (char)30, (char)184, (char)198, (char)221, (char)113, (char)164, (char)154, (char)187, (char)204, (char)127, (char)146, (char)183, (char)154, (char)188, (char)200, (char)55, (char)142, (char)161, (char)225, (char)34, (char)17, (char)164, (char)91, (char)143, (char)31, (char)246, (char)18, (char)87, (char)170, (char)217, (char)118, (char)135, (char)162, (char)180, (char)96, (char)139, (char)201, (char)152, (char)11, (char)168, (char)55, (char)125, (char)24, (char)94, (char)218, (char)127, (char)135, (char)71, (char)140, (char)41, (char)54, (char)146, (char)220, (char)101, (char)183, (char)249, (char)236, (char)211, (char)97, (char)114, (char)52, (char)195, (char)38, (char)245, (char)250, (char)55, (char)137, (char)240, (char)168, (char)71, (char)230, (char)65, (char)95, (char)173, (char)221, (char)186, (char)11, (char)49, (char)120, (char)171, (char)49, (char)155, (char)125, (char)142, (char)126, (char)252, (char)86, (char)195, (char)206, (char)200, (char)37, (char)190, (char)40, (char)115, (char)163, (char)47, (char)45, (char)78, (char)242, (char)192, (char)128, (char)173, (char)217, (char)57, (char)34, (char)153, (char)43, (char)233, (char)252, (char)17, (char)176, (char)87, (char)23, (char)236, (char)12, (char)220, (char)96, (char)204, (char)35, (char)156, (char)1, (char)117, (char)52, (char)195, (char)149, (char)200, (char)162, (char)173, (char)120, (char)127, (char)95, (char)252, (char)77, (char)161, (char)92, (char)157, (char)66, (char)180, (char)5, (char)177, (char)188, (char)69, (char)118, (char)42, (char)212, (char)124, (char)114, (char)196, (char)215, (char)56, (char)71, (char)140, (char)182, (char)47, (char)135, (char)182, (char)37, (char)120, (char)172, (char)53, (char)42, (char)27, (char)47, (char)87, (char)46, (char)0, (char)106, (char)32, (char)177, (char)12, (char)50, (char)74, (char)153, (char)242, (char)252, (char)205, (char)83, (char)46, (char)24, (char)72, (char)242, (char)123, (char)91, (char)52, (char)142, (char)95, (char)127, (char)74, (char)167, (char)94, (char)175, (char)49, (char)21, (char)68, (char)7, (char)94, (char)131, (char)202, (char)239, (char)222, (char)69, (char)143, (char)236, (char)233, (char)126, (char)69, (char)246, (char)212, (char)70, (char)138, (char)101, (char)248, (char)218, (char)136, (char)193, (char)156, (char)35, (char)188, (char)199, (char)74, (char)205, (char)176, (char)150, (char)144, (char)126, (char)117, (char)225}));
            assert(pack.target_system_GET() == (char)189);
            assert(pack.first_message_offset_GET() == (char)52);
            assert(pack.sequence_GET() == (char)61832);
            assert(pack.target_component_GET() == (char)12);
            assert(pack.length_GET() == (char)125);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.sequence_SET((char)61832) ;
        p267.length_SET((char)125) ;
        p267.first_message_offset_SET((char)52) ;
        p267.target_component_SET((char)12) ;
        p267.target_system_SET((char)189) ;
        p267.data__SET(new char[] {(char)235, (char)33, (char)11, (char)40, (char)40, (char)232, (char)230, (char)10, (char)233, (char)205, (char)210, (char)170, (char)20, (char)152, (char)148, (char)154, (char)5, (char)116, (char)13, (char)194, (char)200, (char)176, (char)30, (char)184, (char)198, (char)221, (char)113, (char)164, (char)154, (char)187, (char)204, (char)127, (char)146, (char)183, (char)154, (char)188, (char)200, (char)55, (char)142, (char)161, (char)225, (char)34, (char)17, (char)164, (char)91, (char)143, (char)31, (char)246, (char)18, (char)87, (char)170, (char)217, (char)118, (char)135, (char)162, (char)180, (char)96, (char)139, (char)201, (char)152, (char)11, (char)168, (char)55, (char)125, (char)24, (char)94, (char)218, (char)127, (char)135, (char)71, (char)140, (char)41, (char)54, (char)146, (char)220, (char)101, (char)183, (char)249, (char)236, (char)211, (char)97, (char)114, (char)52, (char)195, (char)38, (char)245, (char)250, (char)55, (char)137, (char)240, (char)168, (char)71, (char)230, (char)65, (char)95, (char)173, (char)221, (char)186, (char)11, (char)49, (char)120, (char)171, (char)49, (char)155, (char)125, (char)142, (char)126, (char)252, (char)86, (char)195, (char)206, (char)200, (char)37, (char)190, (char)40, (char)115, (char)163, (char)47, (char)45, (char)78, (char)242, (char)192, (char)128, (char)173, (char)217, (char)57, (char)34, (char)153, (char)43, (char)233, (char)252, (char)17, (char)176, (char)87, (char)23, (char)236, (char)12, (char)220, (char)96, (char)204, (char)35, (char)156, (char)1, (char)117, (char)52, (char)195, (char)149, (char)200, (char)162, (char)173, (char)120, (char)127, (char)95, (char)252, (char)77, (char)161, (char)92, (char)157, (char)66, (char)180, (char)5, (char)177, (char)188, (char)69, (char)118, (char)42, (char)212, (char)124, (char)114, (char)196, (char)215, (char)56, (char)71, (char)140, (char)182, (char)47, (char)135, (char)182, (char)37, (char)120, (char)172, (char)53, (char)42, (char)27, (char)47, (char)87, (char)46, (char)0, (char)106, (char)32, (char)177, (char)12, (char)50, (char)74, (char)153, (char)242, (char)252, (char)205, (char)83, (char)46, (char)24, (char)72, (char)242, (char)123, (char)91, (char)52, (char)142, (char)95, (char)127, (char)74, (char)167, (char)94, (char)175, (char)49, (char)21, (char)68, (char)7, (char)94, (char)131, (char)202, (char)239, (char)222, (char)69, (char)143, (char)236, (char)233, (char)126, (char)69, (char)246, (char)212, (char)70, (char)138, (char)101, (char)248, (char)218, (char)136, (char)193, (char)156, (char)35, (char)188, (char)199, (char)74, (char)205, (char)176, (char)150, (char)144, (char)126, (char)117, (char)225}, 0) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)61);
            assert(pack.sequence_GET() == (char)37492);
            assert(pack.target_component_GET() == (char)199);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.sequence_SET((char)37492) ;
        p268.target_component_SET((char)199) ;
        p268.target_system_SET((char)61) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.bitrate_GET() == 3336936326L);
            assert(pack.camera_id_GET() == (char)253);
            assert(pack.resolution_h_GET() == (char)33365);
            assert(pack.resolution_v_GET() == (char)63337);
            assert(pack.rotation_GET() == (char)32334);
            assert(pack.uri_LEN(ph) == 68);
            assert(pack.uri_TRY(ph).equals("nfzyKhpoaqizbppamfsblFfFylQwqhwLmbbgxfcnjOufqmqUpomfcusnogbruYtpjvHh"));
            assert(pack.framerate_GET() == -2.9652412E38F);
            assert(pack.status_GET() == (char)10);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.resolution_h_SET((char)33365) ;
        p269.camera_id_SET((char)253) ;
        p269.resolution_v_SET((char)63337) ;
        p269.status_SET((char)10) ;
        p269.uri_SET("nfzyKhpoaqizbppamfsblFfFylQwqhwLmbbgxfcnjOufqmqUpomfcusnogbruYtpjvHh", PH) ;
        p269.framerate_SET(-2.9652412E38F) ;
        p269.rotation_SET((char)32334) ;
        p269.bitrate_SET(3336936326L) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.camera_id_GET() == (char)232);
            assert(pack.bitrate_GET() == 1798770074L);
            assert(pack.resolution_v_GET() == (char)34860);
            assert(pack.target_system_GET() == (char)123);
            assert(pack.uri_LEN(ph) == 130);
            assert(pack.uri_TRY(ph).equals("shzaibvckcgsyqeebcksQakAdjuaumsmwzglhgnbmcekomlzrpaluytznxydtaXinxfnaicvnmruygftgsnreyqlgkwTcaixwhohlpvuqaQqyjSwhjNwpedbnvgjvcwqce"));
            assert(pack.resolution_h_GET() == (char)41384);
            assert(pack.framerate_GET() == -1.760826E38F);
            assert(pack.rotation_GET() == (char)21835);
            assert(pack.target_component_GET() == (char)194);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.bitrate_SET(1798770074L) ;
        p270.resolution_v_SET((char)34860) ;
        p270.rotation_SET((char)21835) ;
        p270.uri_SET("shzaibvckcgsyqeebcksQakAdjuaumsmwzglhgnbmcekomlzrpaluytznxydtaXinxfnaicvnmruygftgsnreyqlgkwTcaixwhohlpvuqaQqyjSwhjNwpedbnvgjvcwqce", PH) ;
        p270.framerate_SET(-1.760826E38F) ;
        p270.camera_id_SET((char)232) ;
        p270.target_system_SET((char)123) ;
        p270.target_component_SET((char)194) ;
        p270.resolution_h_SET((char)41384) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 14);
            assert(pack.ssid_TRY(ph).equals("tdCKvrhpaoVgxg"));
            assert(pack.password_LEN(ph) == 43);
            assert(pack.password_TRY(ph).equals("CfqttfbrrdnrdusstjzzsuxlryrdfDzymnvhuwxxlan"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("CfqttfbrrdnrdusstjzzsuxlryrdfDzymnvhuwxxlan", PH) ;
        p299.ssid_SET("tdCKvrhpaoVgxg", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)7, (char)224, (char)75, (char)107, (char)131, (char)94, (char)67, (char)50}));
            assert(pack.max_version_GET() == (char)34570);
            assert(pack.version_GET() == (char)44141);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)156, (char)157, (char)125, (char)108, (char)114, (char)110, (char)176, (char)27}));
            assert(pack.min_version_GET() == (char)48579);
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.max_version_SET((char)34570) ;
        p300.min_version_SET((char)48579) ;
        p300.spec_version_hash_SET(new char[] {(char)7, (char)224, (char)75, (char)107, (char)131, (char)94, (char)67, (char)50}, 0) ;
        p300.version_SET((char)44141) ;
        p300.library_version_hash_SET(new char[] {(char)156, (char)157, (char)125, (char)108, (char)114, (char)110, (char)176, (char)27}, 0) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.vendor_specific_status_code_GET() == (char)55860);
            assert(pack.time_usec_GET() == 2951496575421007360L);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
            assert(pack.sub_mode_GET() == (char)127);
            assert(pack.uptime_sec_GET() == 3321736118L);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.vendor_specific_status_code_SET((char)55860) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL) ;
        p310.uptime_sec_SET(3321736118L) ;
        p310.time_usec_SET(2951496575421007360L) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK) ;
        p310.sub_mode_SET((char)127) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.sw_version_major_GET() == (char)143);
            assert(pack.sw_vcs_commit_GET() == 3956437033L);
            assert(pack.hw_version_major_GET() == (char)213);
            assert(pack.uptime_sec_GET() == 923609372L);
            assert(pack.hw_version_minor_GET() == (char)114);
            assert(pack.time_usec_GET() == 6050133057314405794L);
            assert(pack.name_LEN(ph) == 58);
            assert(pack.name_TRY(ph).equals("hanjrlaskvqxeJpqsndhncpdvnotgizgipktZivqadamryubznvzxBxnii"));
            assert(pack.sw_version_minor_GET() == (char)89);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)66, (char)186, (char)174, (char)158, (char)250, (char)163, (char)106, (char)237, (char)73, (char)115, (char)54, (char)35, (char)250, (char)189, (char)132, (char)95}));
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_unique_id_SET(new char[] {(char)66, (char)186, (char)174, (char)158, (char)250, (char)163, (char)106, (char)237, (char)73, (char)115, (char)54, (char)35, (char)250, (char)189, (char)132, (char)95}, 0) ;
        p311.sw_version_minor_SET((char)89) ;
        p311.hw_version_minor_SET((char)114) ;
        p311.uptime_sec_SET(923609372L) ;
        p311.time_usec_SET(6050133057314405794L) ;
        p311.name_SET("hanjrlaskvqxeJpqsndhncpdvnotgizgipktZivqadamryubznvzxBxnii", PH) ;
        p311.sw_vcs_commit_SET(3956437033L) ;
        p311.hw_version_major_SET((char)213) ;
        p311.sw_version_major_SET((char)143) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("wwqGbQEh"));
            assert(pack.target_component_GET() == (char)75);
            assert(pack.param_index_GET() == (short) -23367);
            assert(pack.target_system_GET() == (char)227);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_index_SET((short) -23367) ;
        p320.target_component_SET((char)75) ;
        p320.param_id_SET("wwqGbQEh", PH) ;
        p320.target_system_SET((char)227) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)191);
            assert(pack.target_component_GET() == (char)21);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)191) ;
        p321.target_component_SET((char)21) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
            assert(pack.param_index_GET() == (char)47598);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("rbpo"));
            assert(pack.param_value_LEN(ph) == 19);
            assert(pack.param_value_TRY(ph).equals("lAMRofhxorpypphpghY"));
            assert(pack.param_count_GET() == (char)54327);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("rbpo", PH) ;
        p322.param_index_SET((char)47598) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64) ;
        p322.param_count_SET((char)54327) ;
        p322.param_value_SET("lAMRofhxorpypphpghY", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 125);
            assert(pack.param_value_TRY(ph).equals("touzzpbahdSbbprhsgsnvgqxnrxxaeiksozivecwSXgvpovirjjqyinGAkxyjwrauuotbdplAYnovNfsgfeyrfxsvdjoVfwpxhqvVfwdsrpouhOtwepxkhmgwetaQ"));
            assert(pack.target_component_GET() == (char)208);
            assert(pack.param_id_LEN(ph) == 13);
            assert(pack.param_id_TRY(ph).equals("ayjIafmiEsaFa"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
            assert(pack.target_system_GET() == (char)114);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)114) ;
        p323.target_component_SET((char)208) ;
        p323.param_value_SET("touzzpbahdSbbprhsgsnvgqxnrxxaeiksozivecwSXgvpovirjjqyinGAkxyjwrauuotbdplAYnovNfsgfeyrfxsvdjoVfwpxhqvVfwdsrpouhOtwepxkhmgwetaQ", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16) ;
        p323.param_id_SET("ayjIafmiEsaFa", PH) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 123);
            assert(pack.param_value_TRY(ph).equals("bGhmktqbmamBaGoxtbyaygeapmvzmkrKqahLlahbrwzevJpvEinntcpmmdfuowcwyWlirtnjdblNskqsoslTgyDmjrvpvydzzllAgxcwtcyrfzmuepjhuhmOkrk"));
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("shFlVmxzs"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_ACCEPTED);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_ACCEPTED) ;
        p324.param_value_SET("bGhmktqbmamBaGoxtbyaygeapmvzmkrKqahLlahbrwzevJpvEinntcpmmdfuowcwyWlirtnjdblNskqsoslTgyDmjrvpvydzzllAgxcwtcyrfzmuepjhuhmOkrk", PH) ;
        p324.param_id_SET("shFlVmxzs", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.increment_GET() == (char)252);
            assert(pack.max_distance_GET() == (char)23940);
            assert(pack.min_distance_GET() == (char)20455);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)16593, (char)15091, (char)34899, (char)24213, (char)25061, (char)37540, (char)53518, (char)12580, (char)64442, (char)34261, (char)19200, (char)60168, (char)9617, (char)19281, (char)8731, (char)56165, (char)36813, (char)60733, (char)65482, (char)21267, (char)3147, (char)52750, (char)60902, (char)3464, (char)12308, (char)21804, (char)29275, (char)57785, (char)29664, (char)40079, (char)26354, (char)13051, (char)61546, (char)21430, (char)59355, (char)3497, (char)55023, (char)55345, (char)40665, (char)46385, (char)59310, (char)18976, (char)57156, (char)15438, (char)19259, (char)64231, (char)60794, (char)7733, (char)40784, (char)22898, (char)51582, (char)2904, (char)47173, (char)40288, (char)59870, (char)23963, (char)36452, (char)10280, (char)13136, (char)38913, (char)40875, (char)59882, (char)62597, (char)64211, (char)52934, (char)25015, (char)52239, (char)37437, (char)17498, (char)45267, (char)59468, (char)48839}));
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.time_usec_GET() == 7381717494226400837L);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.increment_SET((char)252) ;
        p330.max_distance_SET((char)23940) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p330.min_distance_SET((char)20455) ;
        p330.distances_SET(new char[] {(char)16593, (char)15091, (char)34899, (char)24213, (char)25061, (char)37540, (char)53518, (char)12580, (char)64442, (char)34261, (char)19200, (char)60168, (char)9617, (char)19281, (char)8731, (char)56165, (char)36813, (char)60733, (char)65482, (char)21267, (char)3147, (char)52750, (char)60902, (char)3464, (char)12308, (char)21804, (char)29275, (char)57785, (char)29664, (char)40079, (char)26354, (char)13051, (char)61546, (char)21430, (char)59355, (char)3497, (char)55023, (char)55345, (char)40665, (char)46385, (char)59310, (char)18976, (char)57156, (char)15438, (char)19259, (char)64231, (char)60794, (char)7733, (char)40784, (char)22898, (char)51582, (char)2904, (char)47173, (char)40288, (char)59870, (char)23963, (char)36452, (char)10280, (char)13136, (char)38913, (char)40875, (char)59882, (char)62597, (char)64211, (char)52934, (char)25015, (char)52239, (char)37437, (char)17498, (char)45267, (char)59468, (char)48839}, 0) ;
        p330.time_usec_SET(7381717494226400837L) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
    }

}
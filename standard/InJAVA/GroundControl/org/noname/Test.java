
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
            long id = id__J(src);
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
            long id = id__D(src);
            set_bits(id, 7, data, 276);
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
            set_bits(id, 3, data, 283);
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
            long id = id__D(src);
            set_bits(id, 7, data, 276);
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
            set_bits(id, 3, data, 283);
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
    public static class SET_ACTUATOR_CONTROL_TARGET extends GroundControl.SET_ACTUATOR_CONTROL_TARGET
    {
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        /**
        *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
        *	this field to difference between instances*/
        public char group_mlx_GET()
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  10, 1)); }
        /**
        *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
        *	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
        *	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
        *	mixer to repurpose them as generic outputs*/
        public float[] controls_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 11, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        /**
        *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
        *	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
        *	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
        *	mixer to repurpose them as generic outputs*/
        public float[] controls_GET()
        {return controls_GET(new float[8], 0);}
    }
    public static class ACTUATOR_CONTROL_TARGET extends GroundControl.ACTUATOR_CONTROL_TARGET
    {
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        /**
        *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
        *	this field to difference between instances*/
        public char group_mlx_GET()
        {  return (char)((char) get_bytes(data,  8, 1)); }
        /**
        *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
        *	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
        *	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
        *	mixer to repurpose them as generic outputs*/
        public float[] controls_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 9, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        /**
        *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
        *	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
        *	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
        *	mixer to repurpose them as generic outputs*/
        public float[] controls_GET()
        {return controls_GET(new float[8], 0);}
    }
    public static class ALTITUDE extends GroundControl.ALTITUDE
    {
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        /**
        *This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
        *	local altitude change). The only guarantee on this field is that it will never be reset and is consistent
        *	within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
        *	time. This altitude will also drift and vary between flights*/
        public float altitude_monotonic_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        /**
        *This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
        *	like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
        *	are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
        *	by default and not the WGS84 altitude*/
        public float altitude_amsl_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        /**
        *This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
        *	to the coordinate origin (0, 0, 0). It is up-positive*/
        public float altitude_local_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float altitude_relative_GET()//This is the altitude above the home position. It resets on each change of the current home position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        /**
        *This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
        *	than -1000 should be interpreted as unknown*/
        public float altitude_terrain_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        /**
        *This is not the altitude, but the clear space below the system according to the fused clearance estimate.
        *	It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
        *	target. A negative value indicates no measurement available*/
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
        *	on the URI type enum*/
        public char[] uri_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 2, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
        *	on the URI type enum*/
        public char[] uri_GET()
        {return uri_GET(new char[120], 0);} public char transfer_type_GET()//The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
        {  return (char)((char) get_bytes(data,  122, 1)); }
        /**
        *The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
        *	has a storage associated (e.g. MAVLink FTP)*/
        public char[] storage_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 123, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
        *	has a storage associated (e.g. MAVLink FTP)*/
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
        *	should have the UINT16_MAX value*/
        public char[] voltages_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 0, dst_max = pos + 10; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        /**
        *Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
        *	should have the UINT16_MAX value*/
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
        *	energy consumption estimat*/
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

        static final Collection<OnReceive.Handler<SET_ACTUATOR_CONTROL_TARGET, Channel>> on_SET_ACTUATOR_CONTROL_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<ACTUATOR_CONTROL_TARGET, Channel>> on_ACTUATOR_CONTROL_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<ALTITUDE, Channel>> on_ALTITUDE = new OnReceive<>();
        static final Collection<OnReceive.Handler<RESOURCE_REQUEST, Channel>> on_RESOURCE_REQUEST = new OnReceive<>();
        static final Collection<OnReceive.Handler<SCALED_PRESSURE3, Channel>> on_SCALED_PRESSURE3 = new OnReceive<>();
        static final Collection<OnReceive.Handler<FOLLOW_TARGET, Channel>> on_FOLLOW_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<CONTROL_SYSTEM_STATE, Channel>> on_CONTROL_SYSTEM_STATE = new OnReceive<>();
        static final Collection<OnReceive.Handler<BATTERY_STATUS, Channel>> on_BATTERY_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<AUTOPILOT_VERSION, Channel>> on_AUTOPILOT_VERSION = new OnReceive<>();
        static final Collection<OnReceive.Handler<LANDING_TARGET, Channel>> on_LANDING_TARGET = new OnReceive<>();
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
                case 139:
                    if(pack == null) return new SET_ACTUATOR_CONTROL_TARGET();
                    ((OnReceive) on_SET_ACTUATOR_CONTROL_TARGET).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
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
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_UNINIT);
            assert(pack.mavlink_version_GET() == (char)116);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_FREE_BALLOON);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_PPZ);
            assert(pack.custom_mode_GET() == 3083077432L);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.system_status_SET(MAV_STATE.MAV_STATE_UNINIT) ;
        p0.mavlink_version_SET((char)116) ;
        p0.custom_mode_SET(3083077432L) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_PPZ) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_FREE_BALLOON) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_count3_GET() == (char)40185);
            assert(pack.voltage_battery_GET() == (char)47115);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.load_GET() == (char)13898);
            assert(pack.battery_remaining_GET() == (byte) - 83);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
            assert(pack.errors_count2_GET() == (char)5665);
            assert(pack.errors_comm_GET() == (char)56777);
            assert(pack.errors_count1_GET() == (char)9437);
            assert(pack.errors_count4_GET() == (char)12257);
            assert(pack.current_battery_GET() == (short) -10395);
            assert(pack.drop_rate_comm_GET() == (char)39513);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.errors_count2_SET((char)5665) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW)) ;
        p1.errors_count1_SET((char)9437) ;
        p1.voltage_battery_SET((char)47115) ;
        p1.load_SET((char)13898) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW)) ;
        p1.errors_comm_SET((char)56777) ;
        p1.errors_count4_SET((char)12257) ;
        p1.battery_remaining_SET((byte) - 83) ;
        p1.drop_rate_comm_SET((char)39513) ;
        p1.errors_count3_SET((char)40185) ;
        p1.current_battery_SET((short) -10395) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 1800509161808200294L);
            assert(pack.time_boot_ms_GET() == 3893562241L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(1800509161808200294L) ;
        p2.time_boot_ms_SET(3893562241L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.afy_GET() == 1.3527064E38F);
            assert(pack.vy_GET() == -2.9910267E38F);
            assert(pack.type_mask_GET() == (char)62623);
            assert(pack.afz_GET() == -1.959689E38F);
            assert(pack.yaw_GET() == 2.519167E38F);
            assert(pack.time_boot_ms_GET() == 2195163359L);
            assert(pack.yaw_rate_GET() == -9.430176E37F);
            assert(pack.z_GET() == 2.1162386E37F);
            assert(pack.vx_GET() == 1.0572577E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.x_GET() == 2.0583023E38F);
            assert(pack.vz_GET() == -1.5469629E38F);
            assert(pack.afx_GET() == 7.6305133E37F);
            assert(pack.y_GET() == -1.1856988E38F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.vz_SET(-1.5469629E38F) ;
        p3.yaw_rate_SET(-9.430176E37F) ;
        p3.vy_SET(-2.9910267E38F) ;
        p3.yaw_SET(2.519167E38F) ;
        p3.type_mask_SET((char)62623) ;
        p3.vx_SET(1.0572577E38F) ;
        p3.afy_SET(1.3527064E38F) ;
        p3.x_SET(2.0583023E38F) ;
        p3.afz_SET(-1.959689E38F) ;
        p3.y_SET(-1.1856988E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p3.z_SET(2.1162386E37F) ;
        p3.afx_SET(7.6305133E37F) ;
        p3.time_boot_ms_SET(2195163359L) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)212);
            assert(pack.time_usec_GET() == 7430232672977828015L);
            assert(pack.seq_GET() == 4019799777L);
            assert(pack.target_component_GET() == (char)123);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.target_system_SET((char)212) ;
        p4.time_usec_SET(7430232672977828015L) ;
        p4.target_component_SET((char)123) ;
        p4.seq_SET(4019799777L) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == (char)163);
            assert(pack.passkey_LEN(ph) == 12);
            assert(pack.passkey_TRY(ph).equals("baMrbdhyFhui"));
            assert(pack.target_system_GET() == (char)196);
            assert(pack.control_request_GET() == (char)112);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)196) ;
        p5.control_request_SET((char)112) ;
        p5.version_SET((char)163) ;
        p5.passkey_SET("baMrbdhyFhui", PH) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.ack_GET() == (char)69);
            assert(pack.gcs_system_id_GET() == (char)90);
            assert(pack.control_request_GET() == (char)142);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.ack_SET((char)69) ;
        p6.gcs_system_id_SET((char)90) ;
        p6.control_request_SET((char)142) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 21);
            assert(pack.key_TRY(ph).equals("gcrfeamgcvnaokvrfxpOo"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("gcrfeamgcvnaokvrfxpOo", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_AUTO_DISARMED);
            assert(pack.target_system_GET() == (char)77);
            assert(pack.custom_mode_GET() == 1039414816L);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)77) ;
        p11.custom_mode_SET(1039414816L) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)140);
            assert(pack.param_index_GET() == (short) -29063);
            assert(pack.target_system_GET() == (char)120);
            assert(pack.param_id_LEN(ph) == 6);
            assert(pack.param_id_TRY(ph).equals("ycazJH"));
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_component_SET((char)140) ;
        p20.param_index_SET((short) -29063) ;
        p20.target_system_SET((char)120) ;
        p20.param_id_SET("ycazJH", PH) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)201);
            assert(pack.target_system_GET() == (char)23);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)23) ;
        p21.target_component_SET((char)201) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (char)50070);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
            assert(pack.param_value_GET() == -1.3498483E38F);
            assert(pack.param_count_GET() == (char)47266);
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("yqfpstjvwywArk"));
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_id_SET("yqfpstjvwywArk", PH) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16) ;
        p22.param_count_SET((char)47266) ;
        p22.param_value_SET(-1.3498483E38F) ;
        p22.param_index_SET((char)50070) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)145);
            assert(pack.param_value_GET() == -2.1692607E38F);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("osaj"));
            assert(pack.target_system_GET() == (char)143);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8) ;
        p23.target_system_SET((char)143) ;
        p23.param_value_SET(-2.1692607E38F) ;
        p23.param_id_SET("osaj", PH) ;
        p23.target_component_SET((char)145) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.vel_GET() == (char)55083);
            assert(pack.lon_GET() == -498598293);
            assert(pack.time_usec_GET() == 7193686650565132342L);
            assert(pack.v_acc_TRY(ph) == 813829925L);
            assert(pack.hdg_acc_TRY(ph) == 3080192920L);
            assert(pack.cog_GET() == (char)23414);
            assert(pack.eph_GET() == (char)4484);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
            assert(pack.satellites_visible_GET() == (char)76);
            assert(pack.h_acc_TRY(ph) == 2350690035L);
            assert(pack.vel_acc_TRY(ph) == 1523100442L);
            assert(pack.alt_GET() == -867518607);
            assert(pack.epv_GET() == (char)11471);
            assert(pack.lat_GET() == -1799387167);
            assert(pack.alt_ellipsoid_TRY(ph) == -2025111173);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.time_usec_SET(7193686650565132342L) ;
        p24.satellites_visible_SET((char)76) ;
        p24.alt_ellipsoid_SET(-2025111173, PH) ;
        p24.lat_SET(-1799387167) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS) ;
        p24.vel_acc_SET(1523100442L, PH) ;
        p24.eph_SET((char)4484) ;
        p24.v_acc_SET(813829925L, PH) ;
        p24.epv_SET((char)11471) ;
        p24.lon_SET(-498598293) ;
        p24.h_acc_SET(2350690035L, PH) ;
        p24.hdg_acc_SET(3080192920L, PH) ;
        p24.vel_SET((char)55083) ;
        p24.alt_SET(-867518607) ;
        p24.cog_SET((char)23414) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)134);
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)81, (char)28, (char)245, (char)198, (char)41, (char)86, (char)213, (char)88, (char)97, (char)133, (char)232, (char)110, (char)160, (char)146, (char)229, (char)60, (char)177, (char)81, (char)155, (char)34}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)167, (char)187, (char)159, (char)169, (char)208, (char)147, (char)205, (char)139, (char)172, (char)97, (char)54, (char)37, (char)167, (char)93, (char)244, (char)4, (char)49, (char)68, (char)233, (char)198}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)25, (char)73, (char)31, (char)238, (char)101, (char)251, (char)67, (char)71, (char)181, (char)152, (char)28, (char)117, (char)87, (char)183, (char)214, (char)102, (char)106, (char)200, (char)165, (char)6}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)17, (char)87, (char)131, (char)85, (char)252, (char)18, (char)143, (char)222, (char)139, (char)27, (char)77, (char)142, (char)7, (char)4, (char)64, (char)3, (char)153, (char)75, (char)73, (char)97}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)31, (char)120, (char)59, (char)120, (char)124, (char)206, (char)124, (char)38, (char)128, (char)101, (char)248, (char)62, (char)25, (char)76, (char)183, (char)159, (char)163, (char)75, (char)51, (char)96}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_prn_SET(new char[] {(char)31, (char)120, (char)59, (char)120, (char)124, (char)206, (char)124, (char)38, (char)128, (char)101, (char)248, (char)62, (char)25, (char)76, (char)183, (char)159, (char)163, (char)75, (char)51, (char)96}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)25, (char)73, (char)31, (char)238, (char)101, (char)251, (char)67, (char)71, (char)181, (char)152, (char)28, (char)117, (char)87, (char)183, (char)214, (char)102, (char)106, (char)200, (char)165, (char)6}, 0) ;
        p25.satellites_visible_SET((char)134) ;
        p25.satellite_snr_SET(new char[] {(char)81, (char)28, (char)245, (char)198, (char)41, (char)86, (char)213, (char)88, (char)97, (char)133, (char)232, (char)110, (char)160, (char)146, (char)229, (char)60, (char)177, (char)81, (char)155, (char)34}, 0) ;
        p25.satellite_used_SET(new char[] {(char)17, (char)87, (char)131, (char)85, (char)252, (char)18, (char)143, (char)222, (char)139, (char)27, (char)77, (char)142, (char)7, (char)4, (char)64, (char)3, (char)153, (char)75, (char)73, (char)97}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)167, (char)187, (char)159, (char)169, (char)208, (char)147, (char)205, (char)139, (char)172, (char)97, (char)54, (char)37, (char)167, (char)93, (char)244, (char)4, (char)49, (char)68, (char)233, (char)198}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == (short) -32608);
            assert(pack.time_boot_ms_GET() == 2493263659L);
            assert(pack.ymag_GET() == (short) -32698);
            assert(pack.zacc_GET() == (short) -13055);
            assert(pack.xacc_GET() == (short) -20891);
            assert(pack.xmag_GET() == (short) -20647);
            assert(pack.zgyro_GET() == (short) -28174);
            assert(pack.yacc_GET() == (short)23242);
            assert(pack.xgyro_GET() == (short) -6326);
            assert(pack.ygyro_GET() == (short) -10562);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.xgyro_SET((short) -6326) ;
        p26.zmag_SET((short) -32608) ;
        p26.time_boot_ms_SET(2493263659L) ;
        p26.xmag_SET((short) -20647) ;
        p26.xacc_SET((short) -20891) ;
        p26.ygyro_SET((short) -10562) ;
        p26.zacc_SET((short) -13055) ;
        p26.ymag_SET((short) -32698) ;
        p26.zgyro_SET((short) -28174) ;
        p26.yacc_SET((short)23242) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == (short) -1242);
            assert(pack.xmag_GET() == (short)29892);
            assert(pack.time_usec_GET() == 6238004681607568960L);
            assert(pack.zmag_GET() == (short) -3995);
            assert(pack.yacc_GET() == (short)29770);
            assert(pack.ygyro_GET() == (short) -1856);
            assert(pack.xacc_GET() == (short)30436);
            assert(pack.zgyro_GET() == (short) -11425);
            assert(pack.xgyro_GET() == (short)23873);
            assert(pack.zacc_GET() == (short)26956);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.ygyro_SET((short) -1856) ;
        p27.xmag_SET((short)29892) ;
        p27.zmag_SET((short) -3995) ;
        p27.ymag_SET((short) -1242) ;
        p27.xgyro_SET((short)23873) ;
        p27.zgyro_SET((short) -11425) ;
        p27.yacc_SET((short)29770) ;
        p27.xacc_SET((short)30436) ;
        p27.time_usec_SET(6238004681607568960L) ;
        p27.zacc_SET((short)26956) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff2_GET() == (short) -29441);
            assert(pack.temperature_GET() == (short) -13430);
            assert(pack.time_usec_GET() == 3420915488609225969L);
            assert(pack.press_abs_GET() == (short)15341);
            assert(pack.press_diff1_GET() == (short)22420);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_diff2_SET((short) -29441) ;
        p28.time_usec_SET(3420915488609225969L) ;
        p28.press_diff1_SET((short)22420) ;
        p28.temperature_SET((short) -13430) ;
        p28.press_abs_SET((short)15341) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 8.020656E37F);
            assert(pack.press_diff_GET() == 1.800628E38F);
            assert(pack.time_boot_ms_GET() == 1238341203L);
            assert(pack.temperature_GET() == (short) -10494);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_abs_SET(8.020656E37F) ;
        p29.press_diff_SET(1.800628E38F) ;
        p29.temperature_SET((short) -10494) ;
        p29.time_boot_ms_SET(1238341203L) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.pitchspeed_GET() == 2.5451027E38F);
            assert(pack.rollspeed_GET() == -8.602238E37F);
            assert(pack.yawspeed_GET() == 2.7872205E38F);
            assert(pack.time_boot_ms_GET() == 653902118L);
            assert(pack.yaw_GET() == 2.5924258E38F);
            assert(pack.roll_GET() == -2.054243E38F);
            assert(pack.pitch_GET() == 3.3815618E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.yaw_SET(2.5924258E38F) ;
        p30.rollspeed_SET(-8.602238E37F) ;
        p30.time_boot_ms_SET(653902118L) ;
        p30.pitchspeed_SET(2.5451027E38F) ;
        p30.pitch_SET(3.3815618E38F) ;
        p30.roll_SET(-2.054243E38F) ;
        p30.yawspeed_SET(2.7872205E38F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q1_GET() == -1.6387666E38F);
            assert(pack.q2_GET() == -1.6974944E38F);
            assert(pack.pitchspeed_GET() == -2.460825E38F);
            assert(pack.q3_GET() == -2.0838983E38F);
            assert(pack.rollspeed_GET() == 2.8277604E38F);
            assert(pack.yawspeed_GET() == 1.0751779E38F);
            assert(pack.q4_GET() == -3.2712215E38F);
            assert(pack.time_boot_ms_GET() == 3866536447L);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.rollspeed_SET(2.8277604E38F) ;
        p31.q1_SET(-1.6387666E38F) ;
        p31.pitchspeed_SET(-2.460825E38F) ;
        p31.yawspeed_SET(1.0751779E38F) ;
        p31.q3_SET(-2.0838983E38F) ;
        p31.time_boot_ms_SET(3866536447L) ;
        p31.q4_SET(-3.2712215E38F) ;
        p31.q2_SET(-1.6974944E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 2.7592147E37F);
            assert(pack.vx_GET() == 1.404371E38F);
            assert(pack.y_GET() == 1.1287942E38F);
            assert(pack.vy_GET() == -2.3478015E37F);
            assert(pack.x_GET() == 3.7810103E37F);
            assert(pack.vz_GET() == 3.1612257E38F);
            assert(pack.time_boot_ms_GET() == 973605487L);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.y_SET(1.1287942E38F) ;
        p32.vz_SET(3.1612257E38F) ;
        p32.x_SET(3.7810103E37F) ;
        p32.vx_SET(1.404371E38F) ;
        p32.z_SET(2.7592147E37F) ;
        p32.time_boot_ms_SET(973605487L) ;
        p32.vy_SET(-2.3478015E37F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.hdg_GET() == (char)54224);
            assert(pack.vy_GET() == (short)19650);
            assert(pack.relative_alt_GET() == 487309049);
            assert(pack.time_boot_ms_GET() == 4188968952L);
            assert(pack.alt_GET() == -873952416);
            assert(pack.lon_GET() == -452588707);
            assert(pack.vz_GET() == (short)28076);
            assert(pack.lat_GET() == -1507528253);
            assert(pack.vx_GET() == (short)22780);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.lat_SET(-1507528253) ;
        p33.hdg_SET((char)54224) ;
        p33.lon_SET(-452588707) ;
        p33.vy_SET((short)19650) ;
        p33.time_boot_ms_SET(4188968952L) ;
        p33.vx_SET((short)22780) ;
        p33.vz_SET((short)28076) ;
        p33.relative_alt_SET(487309049) ;
        p33.alt_SET(-873952416) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 21683174L);
            assert(pack.chan7_scaled_GET() == (short)19757);
            assert(pack.chan8_scaled_GET() == (short)29927);
            assert(pack.chan2_scaled_GET() == (short)8980);
            assert(pack.chan5_scaled_GET() == (short)28466);
            assert(pack.chan1_scaled_GET() == (short)11026);
            assert(pack.chan3_scaled_GET() == (short) -27189);
            assert(pack.chan4_scaled_GET() == (short) -9228);
            assert(pack.chan6_scaled_GET() == (short)4295);
            assert(pack.rssi_GET() == (char)199);
            assert(pack.port_GET() == (char)164);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan2_scaled_SET((short)8980) ;
        p34.time_boot_ms_SET(21683174L) ;
        p34.chan5_scaled_SET((short)28466) ;
        p34.chan6_scaled_SET((short)4295) ;
        p34.chan7_scaled_SET((short)19757) ;
        p34.rssi_SET((char)199) ;
        p34.port_SET((char)164) ;
        p34.chan4_scaled_SET((short) -9228) ;
        p34.chan1_scaled_SET((short)11026) ;
        p34.chan8_scaled_SET((short)29927) ;
        p34.chan3_scaled_SET((short) -27189) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)100);
            assert(pack.time_boot_ms_GET() == 530749945L);
            assert(pack.port_GET() == (char)214);
            assert(pack.chan3_raw_GET() == (char)13716);
            assert(pack.chan4_raw_GET() == (char)9930);
            assert(pack.chan5_raw_GET() == (char)5943);
            assert(pack.chan6_raw_GET() == (char)21174);
            assert(pack.chan7_raw_GET() == (char)32848);
            assert(pack.chan1_raw_GET() == (char)19888);
            assert(pack.chan8_raw_GET() == (char)55204);
            assert(pack.chan2_raw_GET() == (char)49096);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.port_SET((char)214) ;
        p35.chan8_raw_SET((char)55204) ;
        p35.chan5_raw_SET((char)5943) ;
        p35.time_boot_ms_SET(530749945L) ;
        p35.chan1_raw_SET((char)19888) ;
        p35.chan4_raw_SET((char)9930) ;
        p35.rssi_SET((char)100) ;
        p35.chan6_raw_SET((char)21174) ;
        p35.chan7_raw_SET((char)32848) ;
        p35.chan2_raw_SET((char)49096) ;
        p35.chan3_raw_SET((char)13716) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo9_raw_TRY(ph) == (char)16679);
            assert(pack.servo12_raw_TRY(ph) == (char)24239);
            assert(pack.port_GET() == (char)130);
            assert(pack.servo15_raw_TRY(ph) == (char)30984);
            assert(pack.servo11_raw_TRY(ph) == (char)54252);
            assert(pack.servo1_raw_GET() == (char)53702);
            assert(pack.servo16_raw_TRY(ph) == (char)58708);
            assert(pack.servo6_raw_GET() == (char)38656);
            assert(pack.servo3_raw_GET() == (char)47317);
            assert(pack.servo10_raw_TRY(ph) == (char)26112);
            assert(pack.servo2_raw_GET() == (char)38919);
            assert(pack.servo14_raw_TRY(ph) == (char)31467);
            assert(pack.servo13_raw_TRY(ph) == (char)37056);
            assert(pack.time_usec_GET() == 2879767155L);
            assert(pack.servo7_raw_GET() == (char)20863);
            assert(pack.servo8_raw_GET() == (char)22352);
            assert(pack.servo4_raw_GET() == (char)52846);
            assert(pack.servo5_raw_GET() == (char)61184);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo11_raw_SET((char)54252, PH) ;
        p36.servo5_raw_SET((char)61184) ;
        p36.time_usec_SET(2879767155L) ;
        p36.servo6_raw_SET((char)38656) ;
        p36.servo9_raw_SET((char)16679, PH) ;
        p36.servo12_raw_SET((char)24239, PH) ;
        p36.servo13_raw_SET((char)37056, PH) ;
        p36.servo4_raw_SET((char)52846) ;
        p36.servo15_raw_SET((char)30984, PH) ;
        p36.servo14_raw_SET((char)31467, PH) ;
        p36.servo3_raw_SET((char)47317) ;
        p36.port_SET((char)130) ;
        p36.servo2_raw_SET((char)38919) ;
        p36.servo1_raw_SET((char)53702) ;
        p36.servo8_raw_SET((char)22352) ;
        p36.servo7_raw_SET((char)20863) ;
        p36.servo16_raw_SET((char)58708, PH) ;
        p36.servo10_raw_SET((char)26112, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short)12286);
            assert(pack.target_system_GET() == (char)12);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.start_index_GET() == (short) -2920);
            assert(pack.target_component_GET() == (char)248);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.end_index_SET((short)12286) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p37.target_component_SET((char)248) ;
        p37.target_system_SET((char)12) ;
        p37.start_index_SET((short) -2920) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short)5427);
            assert(pack.target_component_GET() == (char)61);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.end_index_GET() == (short)23953);
            assert(pack.target_system_GET() == (char)49);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.end_index_SET((short)23953) ;
        p38.start_index_SET((short)5427) ;
        p38.target_component_SET((char)61) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p38.target_system_SET((char)49) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)45);
            assert(pack.param4_GET() == 2.0536575E36F);
            assert(pack.param2_GET() == 1.4535389E38F);
            assert(pack.target_system_GET() == (char)78);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.autocontinue_GET() == (char)28);
            assert(pack.seq_GET() == (char)7984);
            assert(pack.param1_GET() == 3.0686355E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF);
            assert(pack.param3_GET() == -5.0159276E37F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.x_GET() == -2.3986288E38F);
            assert(pack.current_GET() == (char)24);
            assert(pack.y_GET() == -2.5601437E38F);
            assert(pack.z_GET() == -1.0353815E38F);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.x_SET(-2.3986288E38F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF) ;
        p39.param4_SET(2.0536575E36F) ;
        p39.seq_SET((char)7984) ;
        p39.y_SET(-2.5601437E38F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p39.target_system_SET((char)78) ;
        p39.param1_SET(3.0686355E38F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p39.param2_SET(1.4535389E38F) ;
        p39.target_component_SET((char)45) ;
        p39.z_SET(-1.0353815E38F) ;
        p39.autocontinue_SET((char)28) ;
        p39.current_SET((char)24) ;
        p39.param3_SET(-5.0159276E37F) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)14);
            assert(pack.seq_GET() == (char)49567);
            assert(pack.target_component_GET() == (char)111);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p40.seq_SET((char)49567) ;
        p40.target_component_SET((char)111) ;
        p40.target_system_SET((char)14) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)215);
            assert(pack.target_system_GET() == (char)60);
            assert(pack.seq_GET() == (char)16299);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)60) ;
        p41.seq_SET((char)16299) ;
        p41.target_component_SET((char)215) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)21116);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)21116) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)95);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)252);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_component_SET((char)95) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p43.target_system_SET((char)252) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)200);
            assert(pack.target_component_GET() == (char)132);
            assert(pack.count_GET() == (char)48399);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p44.target_component_SET((char)132) ;
        p44.target_system_SET((char)200) ;
        p44.count_SET((char)48399) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)104);
            assert(pack.target_component_GET() == (char)30);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)104) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p45.target_component_SET((char)30) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)63433);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)63433) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED);
            assert(pack.target_component_GET() == (char)210);
            assert(pack.target_system_GET() == (char)87);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p47.target_component_SET((char)210) ;
        p47.target_system_SET((char)87) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == -1528894603);
            assert(pack.target_system_GET() == (char)152);
            assert(pack.altitude_GET() == -2099051153);
            assert(pack.time_usec_TRY(ph) == 5248154447052564377L);
            assert(pack.latitude_GET() == -2015881768);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.time_usec_SET(5248154447052564377L, PH) ;
        p48.target_system_SET((char)152) ;
        p48.latitude_SET(-2015881768) ;
        p48.altitude_SET(-2099051153) ;
        p48.longitude_SET(-1528894603) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 59311285);
            assert(pack.altitude_GET() == -2112401291);
            assert(pack.time_usec_TRY(ph) == 6178259181007635223L);
            assert(pack.longitude_GET() == 1337902378);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(59311285) ;
        p49.longitude_SET(1337902378) ;
        p49.time_usec_SET(6178259181007635223L, PH) ;
        p49.altitude_SET(-2112401291) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.scale_GET() == 3.3835708E38F);
            assert(pack.parameter_rc_channel_index_GET() == (char)130);
            assert(pack.target_system_GET() == (char)88);
            assert(pack.param_value0_GET() == -1.689206E38F);
            assert(pack.target_component_GET() == (char)91);
            assert(pack.param_value_min_GET() == -2.1827728E38F);
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("sUtpyzanhyzfkvf"));
            assert(pack.param_index_GET() == (short)799);
            assert(pack.param_value_max_GET() == -1.6634364E38F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.param_id_SET("sUtpyzanhyzfkvf", PH) ;
        p50.target_system_SET((char)88) ;
        p50.parameter_rc_channel_index_SET((char)130) ;
        p50.param_value_max_SET(-1.6634364E38F) ;
        p50.param_value0_SET(-1.689206E38F) ;
        p50.target_component_SET((char)91) ;
        p50.param_value_min_SET(-2.1827728E38F) ;
        p50.param_index_SET((short)799) ;
        p50.scale_SET(3.3835708E38F) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)27396);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)48);
            assert(pack.target_system_GET() == (char)229);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p51.target_component_SET((char)48) ;
        p51.seq_SET((char)27396) ;
        p51.target_system_SET((char)229) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1z_GET() == -2.6860741E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.p1x_GET() == 3.1723242E38F);
            assert(pack.p2z_GET() == 1.9161202E38F);
            assert(pack.p2y_GET() == 2.2766974E38F);
            assert(pack.p1y_GET() == -9.663114E37F);
            assert(pack.target_component_GET() == (char)234);
            assert(pack.p2x_GET() == -1.9987183E38F);
            assert(pack.target_system_GET() == (char)33);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p1z_SET(-2.6860741E38F) ;
        p54.p2x_SET(-1.9987183E38F) ;
        p54.p1y_SET(-9.663114E37F) ;
        p54.p2y_SET(2.2766974E38F) ;
        p54.p1x_SET(3.1723242E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p54.target_component_SET((char)234) ;
        p54.p2z_SET(1.9161202E38F) ;
        p54.target_system_SET((char)33) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2x_GET() == -1.9251418E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.p1x_GET() == -1.4607393E38F);
            assert(pack.p1y_GET() == -7.9854514E37F);
            assert(pack.p2z_GET() == -2.5713702E37F);
            assert(pack.p2y_GET() == -1.5684369E38F);
            assert(pack.p1z_GET() == -2.3527695E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p1y_SET(-7.9854514E37F) ;
        p55.p2x_SET(-1.9251418E38F) ;
        p55.p1x_SET(-1.4607393E38F) ;
        p55.p2y_SET(-1.5684369E38F) ;
        p55.p2z_SET(-2.5713702E37F) ;
        p55.p1z_SET(-2.3527695E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == -2.0892121E38F);
            assert(pack.pitchspeed_GET() == 2.6322963E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.106107E38F, -5.602736E37F, -3.130355E38F, 5.451862E37F}));
            assert(pack.time_usec_GET() == 6642308271514697211L);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.0631262E38F, 2.2584615E38F, 4.7965774E37F, -1.6154107E38F, 1.8525243E38F, 1.38673E38F, 1.909629E38F, 1.0953037E38F, -4.1392904E37F}));
            assert(pack.yawspeed_GET() == 7.926543E37F);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.covariance_SET(new float[] {-1.0631262E38F, 2.2584615E38F, 4.7965774E37F, -1.6154107E38F, 1.8525243E38F, 1.38673E38F, 1.909629E38F, 1.0953037E38F, -4.1392904E37F}, 0) ;
        p61.q_SET(new float[] {1.106107E38F, -5.602736E37F, -3.130355E38F, 5.451862E37F}, 0) ;
        p61.time_usec_SET(6642308271514697211L) ;
        p61.yawspeed_SET(7.926543E37F) ;
        p61.rollspeed_SET(-2.0892121E38F) ;
        p61.pitchspeed_SET(2.6322963E38F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_bearing_GET() == (short) -18372);
            assert(pack.nav_roll_GET() == 1.9860022E38F);
            assert(pack.xtrack_error_GET() == -2.422796E38F);
            assert(pack.wp_dist_GET() == (char)5798);
            assert(pack.alt_error_GET() == -1.682907E37F);
            assert(pack.nav_pitch_GET() == 3.8377093E37F);
            assert(pack.target_bearing_GET() == (short) -14094);
            assert(pack.aspd_error_GET() == 7.022869E37F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.target_bearing_SET((short) -14094) ;
        p62.nav_roll_SET(1.9860022E38F) ;
        p62.xtrack_error_SET(-2.422796E38F) ;
        p62.nav_pitch_SET(3.8377093E37F) ;
        p62.nav_bearing_SET((short) -18372) ;
        p62.aspd_error_SET(7.022869E37F) ;
        p62.wp_dist_SET((char)5798) ;
        p62.alt_error_SET(-1.682907E37F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 454460299);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.5057964E38F, -3.215846E36F, 1.6755746E38F, 1.8820127E38F, 2.1035465E38F, -2.427496E38F, 3.126686E38F, -2.832409E38F, 3.3115396E38F, -2.2691191E38F, 2.6747809E38F, 1.8750623E38F, -3.338288E38F, -2.070715E38F, -2.3039787E38F, -1.8872228E38F, -1.4164737E38F, -1.8619992E38F, -1.3767309E38F, -2.8439161E38F, -1.365272E38F, -2.9019534E38F, -2.9097893E38F, -2.5836157E38F, -3.0539133E38F, 2.177077E38F, 2.290275E38F, 2.946862E38F, -1.5705427E38F, -6.9569765E37F, 1.808688E38F, 2.1963635E38F, -3.3346222E38F, -1.0581763E38F, -3.3779893E38F, -3.2393836E38F}));
            assert(pack.time_usec_GET() == 3002608802353973031L);
            assert(pack.relative_alt_GET() == -903302474);
            assert(pack.alt_GET() == -1851499519);
            assert(pack.vy_GET() == -1.391746E37F);
            assert(pack.vx_GET() == -9.973864E37F);
            assert(pack.lon_GET() == -1987416588);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
            assert(pack.vz_GET() == 3.117933E38F);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.lon_SET(-1987416588) ;
        p63.relative_alt_SET(-903302474) ;
        p63.lat_SET(454460299) ;
        p63.alt_SET(-1851499519) ;
        p63.vy_SET(-1.391746E37F) ;
        p63.covariance_SET(new float[] {-1.5057964E38F, -3.215846E36F, 1.6755746E38F, 1.8820127E38F, 2.1035465E38F, -2.427496E38F, 3.126686E38F, -2.832409E38F, 3.3115396E38F, -2.2691191E38F, 2.6747809E38F, 1.8750623E38F, -3.338288E38F, -2.070715E38F, -2.3039787E38F, -1.8872228E38F, -1.4164737E38F, -1.8619992E38F, -1.3767309E38F, -2.8439161E38F, -1.365272E38F, -2.9019534E38F, -2.9097893E38F, -2.5836157E38F, -3.0539133E38F, 2.177077E38F, 2.290275E38F, 2.946862E38F, -1.5705427E38F, -6.9569765E37F, 1.808688E38F, 2.1963635E38F, -3.3346222E38F, -1.0581763E38F, -3.3779893E38F, -3.2393836E38F}, 0) ;
        p63.vx_SET(-9.973864E37F) ;
        p63.time_usec_SET(3002608802353973031L) ;
        p63.vz_SET(3.117933E38F) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 5.7793606E37F);
            assert(pack.z_GET() == -5.7366904E37F);
            assert(pack.ay_GET() == 3.1971195E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {4.595084E37F, 4.0094006E37F, 4.4413325E37F, 1.0852639E38F, -1.3199344E38F, 2.6767962E38F, 6.942974E36F, 3.3155235E38F, 1.8784895E37F, -1.256479E38F, -5.153939E37F, -9.792349E37F, -6.3600434E37F, 1.2838264E38F, 1.893936E38F, 1.5101823E38F, -2.5411182E38F, 4.0136267E37F, -1.9900741E38F, 5.9967865E37F, 1.9762283E38F, 7.9520616E36F, -3.0288016E38F, 2.958512E38F, 9.733248E37F, -7.3034574E37F, -1.3680356E38F, -1.6198553E38F, -1.3378376E38F, 2.3883602E38F, -1.7584233E38F, -1.3953216E38F, -2.62889E38F, -2.4214173E38F, -3.2231856E38F, 4.0340912E37F, -3.221327E38F, -1.3273164E38F, -2.6108238E38F, 1.4058863E38F, -2.131535E38F, 1.6938075E38F, -3.2156172E38F, -2.8222904E38F, -3.6477462E37F}));
            assert(pack.ax_GET() == 2.0290758E38F);
            assert(pack.y_GET() == 2.256919E37F);
            assert(pack.az_GET() == 7.1703176E37F);
            assert(pack.vx_GET() == 1.920771E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.vy_GET() == -6.9167575E37F);
            assert(pack.time_usec_GET() == 3626116929985129070L);
            assert(pack.vz_GET() == 2.1285805E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.vx_SET(1.920771E38F) ;
        p64.ax_SET(2.0290758E38F) ;
        p64.x_SET(5.7793606E37F) ;
        p64.vz_SET(2.1285805E38F) ;
        p64.az_SET(7.1703176E37F) ;
        p64.vy_SET(-6.9167575E37F) ;
        p64.y_SET(2.256919E37F) ;
        p64.z_SET(-5.7366904E37F) ;
        p64.time_usec_SET(3626116929985129070L) ;
        p64.covariance_SET(new float[] {4.595084E37F, 4.0094006E37F, 4.4413325E37F, 1.0852639E38F, -1.3199344E38F, 2.6767962E38F, 6.942974E36F, 3.3155235E38F, 1.8784895E37F, -1.256479E38F, -5.153939E37F, -9.792349E37F, -6.3600434E37F, 1.2838264E38F, 1.893936E38F, 1.5101823E38F, -2.5411182E38F, 4.0136267E37F, -1.9900741E38F, 5.9967865E37F, 1.9762283E38F, 7.9520616E36F, -3.0288016E38F, 2.958512E38F, 9.733248E37F, -7.3034574E37F, -1.3680356E38F, -1.6198553E38F, -1.3378376E38F, 2.3883602E38F, -1.7584233E38F, -1.3953216E38F, -2.62889E38F, -2.4214173E38F, -3.2231856E38F, 4.0340912E37F, -3.221327E38F, -1.3273164E38F, -2.6108238E38F, 1.4058863E38F, -2.131535E38F, 1.6938075E38F, -3.2156172E38F, -2.8222904E38F, -3.6477462E37F}, 0) ;
        p64.ay_SET(3.1971195E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan15_raw_GET() == (char)20246);
            assert(pack.chan12_raw_GET() == (char)63697);
            assert(pack.chancount_GET() == (char)160);
            assert(pack.chan2_raw_GET() == (char)26592);
            assert(pack.chan17_raw_GET() == (char)50349);
            assert(pack.chan16_raw_GET() == (char)56540);
            assert(pack.chan4_raw_GET() == (char)55805);
            assert(pack.chan13_raw_GET() == (char)14014);
            assert(pack.chan3_raw_GET() == (char)26408);
            assert(pack.chan6_raw_GET() == (char)61371);
            assert(pack.chan1_raw_GET() == (char)32037);
            assert(pack.chan10_raw_GET() == (char)1531);
            assert(pack.chan8_raw_GET() == (char)47467);
            assert(pack.chan18_raw_GET() == (char)40992);
            assert(pack.rssi_GET() == (char)99);
            assert(pack.chan9_raw_GET() == (char)2750);
            assert(pack.time_boot_ms_GET() == 2083697863L);
            assert(pack.chan14_raw_GET() == (char)10138);
            assert(pack.chan5_raw_GET() == (char)60383);
            assert(pack.chan11_raw_GET() == (char)35960);
            assert(pack.chan7_raw_GET() == (char)470);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan14_raw_SET((char)10138) ;
        p65.chan7_raw_SET((char)470) ;
        p65.chan3_raw_SET((char)26408) ;
        p65.chan15_raw_SET((char)20246) ;
        p65.chan1_raw_SET((char)32037) ;
        p65.chan6_raw_SET((char)61371) ;
        p65.chan18_raw_SET((char)40992) ;
        p65.chan11_raw_SET((char)35960) ;
        p65.chan2_raw_SET((char)26592) ;
        p65.chan5_raw_SET((char)60383) ;
        p65.time_boot_ms_SET(2083697863L) ;
        p65.chan10_raw_SET((char)1531) ;
        p65.chan16_raw_SET((char)56540) ;
        p65.chan4_raw_SET((char)55805) ;
        p65.chancount_SET((char)160) ;
        p65.chan13_raw_SET((char)14014) ;
        p65.chan12_raw_SET((char)63697) ;
        p65.rssi_SET((char)99) ;
        p65.chan9_raw_SET((char)2750) ;
        p65.chan17_raw_SET((char)50349) ;
        p65.chan8_raw_SET((char)47467) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.req_message_rate_GET() == (char)4604);
            assert(pack.target_system_GET() == (char)198);
            assert(pack.req_stream_id_GET() == (char)254);
            assert(pack.start_stop_GET() == (char)72);
            assert(pack.target_component_GET() == (char)241);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_message_rate_SET((char)4604) ;
        p66.target_component_SET((char)241) ;
        p66.start_stop_SET((char)72) ;
        p66.req_stream_id_SET((char)254) ;
        p66.target_system_SET((char)198) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.stream_id_GET() == (char)241);
            assert(pack.on_off_GET() == (char)140);
            assert(pack.message_rate_GET() == (char)4298);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)241) ;
        p67.message_rate_SET((char)4298) ;
        p67.on_off_SET((char)140) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.r_GET() == (short)11006);
            assert(pack.y_GET() == (short)30808);
            assert(pack.target_GET() == (char)160);
            assert(pack.x_GET() == (short) -6317);
            assert(pack.buttons_GET() == (char)4512);
            assert(pack.z_GET() == (short) -17598);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.r_SET((short)11006) ;
        p69.z_SET((short) -17598) ;
        p69.x_SET((short) -6317) ;
        p69.target_SET((char)160) ;
        p69.y_SET((short)30808) ;
        p69.buttons_SET((char)4512) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)4184);
            assert(pack.chan2_raw_GET() == (char)44050);
            assert(pack.chan7_raw_GET() == (char)11549);
            assert(pack.chan3_raw_GET() == (char)51265);
            assert(pack.chan8_raw_GET() == (char)29160);
            assert(pack.chan5_raw_GET() == (char)41141);
            assert(pack.target_component_GET() == (char)158);
            assert(pack.chan1_raw_GET() == (char)53743);
            assert(pack.chan6_raw_GET() == (char)46549);
            assert(pack.target_system_GET() == (char)16);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan4_raw_SET((char)4184) ;
        p70.chan2_raw_SET((char)44050) ;
        p70.chan6_raw_SET((char)46549) ;
        p70.chan3_raw_SET((char)51265) ;
        p70.chan1_raw_SET((char)53743) ;
        p70.chan7_raw_SET((char)11549) ;
        p70.target_system_SET((char)16) ;
        p70.chan8_raw_SET((char)29160) ;
        p70.chan5_raw_SET((char)41141) ;
        p70.target_component_SET((char)158) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param1_GET() == 1.1462142E38F);
            assert(pack.param2_GET() == 2.1558757E38F);
            assert(pack.x_GET() == -141153420);
            assert(pack.current_GET() == (char)248);
            assert(pack.target_system_GET() == (char)2);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL);
            assert(pack.target_component_GET() == (char)113);
            assert(pack.param3_GET() == -4.2251616E37F);
            assert(pack.y_GET() == -1565659435);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.seq_GET() == (char)11529);
            assert(pack.autocontinue_GET() == (char)4);
            assert(pack.param4_GET() == 1.5993268E38F);
            assert(pack.z_GET() == -5.0622283E37F);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.param2_SET(2.1558757E38F) ;
        p73.param3_SET(-4.2251616E37F) ;
        p73.target_component_SET((char)113) ;
        p73.seq_SET((char)11529) ;
        p73.y_SET(-1565659435) ;
        p73.autocontinue_SET((char)4) ;
        p73.param1_SET(1.1462142E38F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p73.target_system_SET((char)2) ;
        p73.z_SET(-5.0622283E37F) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p73.command_SET(MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL) ;
        p73.current_SET((char)248) ;
        p73.param4_SET(1.5993268E38F) ;
        p73.x_SET(-141153420) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.heading_GET() == (short)19821);
            assert(pack.climb_GET() == -2.0004356E38F);
            assert(pack.groundspeed_GET() == -3.5290792E37F);
            assert(pack.airspeed_GET() == 2.377336E38F);
            assert(pack.alt_GET() == -5.2250737E37F);
            assert(pack.throttle_GET() == (char)18235);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.airspeed_SET(2.377336E38F) ;
        p74.groundspeed_SET(-3.5290792E37F) ;
        p74.throttle_SET((char)18235) ;
        p74.climb_SET(-2.0004356E38F) ;
        p74.heading_SET((short)19821) ;
        p74.alt_SET(-5.2250737E37F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 1965677234);
            assert(pack.target_component_GET() == (char)87);
            assert(pack.param2_GET() == -1.5425372E38F);
            assert(pack.target_system_GET() == (char)238);
            assert(pack.current_GET() == (char)6);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS);
            assert(pack.y_GET() == -1256072335);
            assert(pack.param4_GET() == 2.2207156E38F);
            assert(pack.param3_GET() == 2.4136045E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.z_GET() == -3.3805193E38F);
            assert(pack.autocontinue_GET() == (char)120);
            assert(pack.param1_GET() == -3.2672777E38F);
        });
        GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.current_SET((char)6) ;
        p75.x_SET(1965677234) ;
        p75.autocontinue_SET((char)120) ;
        p75.z_SET(-3.3805193E38F) ;
        p75.target_component_SET((char)87) ;
        p75.param3_SET(2.4136045E38F) ;
        p75.command_SET(MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS) ;
        p75.param2_SET(-1.5425372E38F) ;
        p75.param1_SET(-3.2672777E38F) ;
        p75.y_SET(-1256072335) ;
        p75.param4_SET(2.2207156E38F) ;
        p75.target_system_SET((char)238) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param6_GET() == -1.4167111E38F);
            assert(pack.param5_GET() == -3.0441128E38F);
            assert(pack.param3_GET() == -6.3990074E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT);
            assert(pack.target_system_GET() == (char)41);
            assert(pack.confirmation_GET() == (char)218);
            assert(pack.param1_GET() == 1.1704384E38F);
            assert(pack.param2_GET() == -2.7962208E36F);
            assert(pack.param4_GET() == -2.7779423E38F);
            assert(pack.target_component_GET() == (char)126);
            assert(pack.param7_GET() == -2.0031457E38F);
        });
        GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param1_SET(1.1704384E38F) ;
        p76.confirmation_SET((char)218) ;
        p76.param6_SET(-1.4167111E38F) ;
        p76.param4_SET(-2.7779423E38F) ;
        p76.param5_SET(-3.0441128E38F) ;
        p76.param3_SET(-6.3990074E37F) ;
        p76.param7_SET(-2.0031457E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT) ;
        p76.param2_SET(-2.7962208E36F) ;
        p76.target_component_SET((char)126) ;
        p76.target_system_SET((char)41) ;
        CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.result_param2_TRY(ph) == 1944140886);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_IN_PROGRESS);
            assert(pack.target_system_TRY(ph) == (char)210);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION);
            assert(pack.target_component_TRY(ph) == (char)23);
            assert(pack.progress_TRY(ph) == (char)204);
        });
        GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.target_system_SET((char)210, PH) ;
        p77.result_param2_SET(1944140886, PH) ;
        p77.progress_SET((char)204, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_IN_PROGRESS) ;
        p77.target_component_SET((char)23, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION) ;
        CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 1.8540143E38F);
            assert(pack.thrust_GET() == 1.3456668E38F);
            assert(pack.time_boot_ms_GET() == 3709203591L);
            assert(pack.manual_override_switch_GET() == (char)240);
            assert(pack.mode_switch_GET() == (char)30);
            assert(pack.yaw_GET() == -2.9481304E38F);
            assert(pack.pitch_GET() == 1.4307857E38F);
        });
        GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.time_boot_ms_SET(3709203591L) ;
        p81.thrust_SET(1.3456668E38F) ;
        p81.mode_switch_SET((char)30) ;
        p81.yaw_SET(-2.9481304E38F) ;
        p81.roll_SET(1.8540143E38F) ;
        p81.pitch_SET(1.4307857E38F) ;
        p81.manual_override_switch_SET((char)240) ;
        CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)177);
            assert(pack.thrust_GET() == 1.901646E38F);
            assert(pack.body_yaw_rate_GET() == -1.5903899E38F);
            assert(pack.body_roll_rate_GET() == -2.8528757E38F);
            assert(pack.type_mask_GET() == (char)10);
            assert(pack.body_pitch_rate_GET() == 9.080124E37F);
            assert(pack.time_boot_ms_GET() == 2845811647L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-8.936244E35F, -3.160689E37F, 6.6300896E36F, 2.2046226E37F}));
            assert(pack.target_component_GET() == (char)20);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.body_roll_rate_SET(-2.8528757E38F) ;
        p82.time_boot_ms_SET(2845811647L) ;
        p82.thrust_SET(1.901646E38F) ;
        p82.q_SET(new float[] {-8.936244E35F, -3.160689E37F, 6.6300896E36F, 2.2046226E37F}, 0) ;
        p82.type_mask_SET((char)10) ;
        p82.target_component_SET((char)20) ;
        p82.body_yaw_rate_SET(-1.5903899E38F) ;
        p82.body_pitch_rate_SET(9.080124E37F) ;
        p82.target_system_SET((char)177) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 291590773L);
            assert(pack.thrust_GET() == -4.0178525E36F);
            assert(pack.body_roll_rate_GET() == -2.7310905E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.4381462E37F, 2.0946774E38F, -8.139454E37F, -1.28051E38F}));
            assert(pack.body_yaw_rate_GET() == -2.9377285E37F);
            assert(pack.body_pitch_rate_GET() == -2.9248762E38F);
            assert(pack.type_mask_GET() == (char)221);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.thrust_SET(-4.0178525E36F) ;
        p83.body_yaw_rate_SET(-2.9377285E37F) ;
        p83.type_mask_SET((char)221) ;
        p83.time_boot_ms_SET(291590773L) ;
        p83.body_roll_rate_SET(-2.7310905E38F) ;
        p83.body_pitch_rate_SET(-2.9248762E38F) ;
        p83.q_SET(new float[] {-2.4381462E37F, 2.0946774E38F, -8.139454E37F, -1.28051E38F}, 0) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)65);
            assert(pack.x_GET() == -1.1750473E38F);
            assert(pack.yaw_GET() == -3.3401668E38F);
            assert(pack.vy_GET() == 7.06209E37F);
            assert(pack.afz_GET() == 3.2858017E38F);
            assert(pack.target_system_GET() == (char)41);
            assert(pack.afy_GET() == -1.1176919E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.y_GET() == 2.354725E38F);
            assert(pack.z_GET() == 1.2287018E38F);
            assert(pack.afx_GET() == -4.8553335E37F);
            assert(pack.time_boot_ms_GET() == 4114302332L);
            assert(pack.yaw_rate_GET() == -1.2227833E38F);
            assert(pack.type_mask_GET() == (char)36130);
            assert(pack.vx_GET() == 2.6927882E38F);
            assert(pack.vz_GET() == 2.29959E38F);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.afy_SET(-1.1176919E38F) ;
        p84.target_system_SET((char)41) ;
        p84.afz_SET(3.2858017E38F) ;
        p84.yaw_SET(-3.3401668E38F) ;
        p84.afx_SET(-4.8553335E37F) ;
        p84.yaw_rate_SET(-1.2227833E38F) ;
        p84.vx_SET(2.6927882E38F) ;
        p84.type_mask_SET((char)36130) ;
        p84.vz_SET(2.29959E38F) ;
        p84.vy_SET(7.06209E37F) ;
        p84.x_SET(-1.1750473E38F) ;
        p84.y_SET(2.354725E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p84.z_SET(1.2287018E38F) ;
        p84.time_boot_ms_SET(4114302332L) ;
        p84.target_component_SET((char)65) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.lat_int_GET() == -1344000278);
            assert(pack.type_mask_GET() == (char)21859);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.vy_GET() == -1.5297192E38F);
            assert(pack.vz_GET() == -8.364434E37F);
            assert(pack.target_component_GET() == (char)220);
            assert(pack.alt_GET() == -2.6082702E38F);
            assert(pack.yaw_rate_GET() == 2.5652332E37F);
            assert(pack.vx_GET() == 2.1330262E38F);
            assert(pack.target_system_GET() == (char)207);
            assert(pack.afy_GET() == 1.1827963E38F);
            assert(pack.afx_GET() == 3.3354181E38F);
            assert(pack.time_boot_ms_GET() == 2560697719L);
            assert(pack.yaw_GET() == -3.3352356E38F);
            assert(pack.afz_GET() == -1.3879805E38F);
            assert(pack.lon_int_GET() == -2044236837);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.afx_SET(3.3354181E38F) ;
        p86.afy_SET(1.1827963E38F) ;
        p86.yaw_SET(-3.3352356E38F) ;
        p86.type_mask_SET((char)21859) ;
        p86.vz_SET(-8.364434E37F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p86.afz_SET(-1.3879805E38F) ;
        p86.target_component_SET((char)220) ;
        p86.alt_SET(-2.6082702E38F) ;
        p86.vy_SET(-1.5297192E38F) ;
        p86.target_system_SET((char)207) ;
        p86.time_boot_ms_SET(2560697719L) ;
        p86.vx_SET(2.1330262E38F) ;
        p86.lat_int_SET(-1344000278) ;
        p86.yaw_rate_SET(2.5652332E37F) ;
        p86.lon_int_SET(-2044236837) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.lon_int_GET() == 160267827);
            assert(pack.alt_GET() == -2.1832685E38F);
            assert(pack.vx_GET() == 3.289419E38F);
            assert(pack.lat_int_GET() == -1113601474);
            assert(pack.type_mask_GET() == (char)47651);
            assert(pack.afx_GET() == -1.7564083E35F);
            assert(pack.yaw_GET() == 6.508713E37F);
            assert(pack.time_boot_ms_GET() == 2703651977L);
            assert(pack.afz_GET() == 2.6146348E38F);
            assert(pack.vy_GET() == -1.1854726E37F);
            assert(pack.vz_GET() == 2.7963474E38F);
            assert(pack.yaw_rate_GET() == -1.2915105E38F);
            assert(pack.afy_GET() == 1.7541889E38F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.afz_SET(2.6146348E38F) ;
        p87.yaw_SET(6.508713E37F) ;
        p87.lat_int_SET(-1113601474) ;
        p87.afx_SET(-1.7564083E35F) ;
        p87.afy_SET(1.7541889E38F) ;
        p87.alt_SET(-2.1832685E38F) ;
        p87.lon_int_SET(160267827) ;
        p87.time_boot_ms_SET(2703651977L) ;
        p87.type_mask_SET((char)47651) ;
        p87.yaw_rate_SET(-1.2915105E38F) ;
        p87.vy_SET(-1.1854726E37F) ;
        p87.vz_SET(2.7963474E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p87.vx_SET(3.289419E38F) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 1.6886115E38F);
            assert(pack.yaw_GET() == 2.0470825E38F);
            assert(pack.pitch_GET() == -3.8625304E37F);
            assert(pack.time_boot_ms_GET() == 353709295L);
            assert(pack.roll_GET() == 2.8557635E38F);
            assert(pack.x_GET() == -1.7000626E37F);
            assert(pack.y_GET() == -4.5910517E37F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.z_SET(1.6886115E38F) ;
        p89.time_boot_ms_SET(353709295L) ;
        p89.y_SET(-4.5910517E37F) ;
        p89.roll_SET(2.8557635E38F) ;
        p89.yaw_SET(2.0470825E38F) ;
        p89.pitch_SET(-3.8625304E37F) ;
        p89.x_SET(-1.7000626E37F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == (short)2092);
            assert(pack.lon_GET() == 615691026);
            assert(pack.yacc_GET() == (short) -20238);
            assert(pack.roll_GET() == 2.2406732E38F);
            assert(pack.yawspeed_GET() == -1.9564467E38F);
            assert(pack.zacc_GET() == (short)5340);
            assert(pack.time_usec_GET() == 5349413478225160497L);
            assert(pack.vy_GET() == (short) -22989);
            assert(pack.lat_GET() == -26570049);
            assert(pack.pitchspeed_GET() == -1.5645262E38F);
            assert(pack.yaw_GET() == 2.9278859E38F);
            assert(pack.vz_GET() == (short) -25071);
            assert(pack.xacc_GET() == (short)30822);
            assert(pack.pitch_GET() == -3.1886791E38F);
            assert(pack.rollspeed_GET() == -8.2201417E37F);
            assert(pack.alt_GET() == 1657190296);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.yawspeed_SET(-1.9564467E38F) ;
        p90.roll_SET(2.2406732E38F) ;
        p90.lat_SET(-26570049) ;
        p90.vx_SET((short)2092) ;
        p90.pitchspeed_SET(-1.5645262E38F) ;
        p90.yacc_SET((short) -20238) ;
        p90.pitch_SET(-3.1886791E38F) ;
        p90.rollspeed_SET(-8.2201417E37F) ;
        p90.vz_SET((short) -25071) ;
        p90.lon_SET(615691026) ;
        p90.xacc_SET((short)30822) ;
        p90.zacc_SET((short)5340) ;
        p90.time_usec_SET(5349413478225160497L) ;
        p90.vy_SET((short) -22989) ;
        p90.yaw_SET(2.9278859E38F) ;
        p90.alt_SET(1657190296) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3706474344282279224L);
            assert(pack.aux1_GET() == 1.9288734E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_PREFLIGHT);
            assert(pack.aux4_GET() == 1.177654E38F);
            assert(pack.nav_mode_GET() == (char)119);
            assert(pack.yaw_rudder_GET() == -3.3009492E38F);
            assert(pack.aux3_GET() == 2.0385485E38F);
            assert(pack.throttle_GET() == 1.271307E38F);
            assert(pack.aux2_GET() == -2.2764979E38F);
            assert(pack.pitch_elevator_GET() == -1.7739169E37F);
            assert(pack.roll_ailerons_GET() == 3.0663553E38F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.yaw_rudder_SET(-3.3009492E38F) ;
        p91.roll_ailerons_SET(3.0663553E38F) ;
        p91.nav_mode_SET((char)119) ;
        p91.aux4_SET(1.177654E38F) ;
        p91.aux3_SET(2.0385485E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_PREFLIGHT) ;
        p91.throttle_SET(1.271307E38F) ;
        p91.pitch_elevator_SET(-1.7739169E37F) ;
        p91.time_usec_SET(3706474344282279224L) ;
        p91.aux2_SET(-2.2764979E38F) ;
        p91.aux1_SET(1.9288734E38F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan12_raw_GET() == (char)9852);
            assert(pack.chan2_raw_GET() == (char)44235);
            assert(pack.chan7_raw_GET() == (char)48483);
            assert(pack.chan4_raw_GET() == (char)46903);
            assert(pack.rssi_GET() == (char)156);
            assert(pack.chan3_raw_GET() == (char)20179);
            assert(pack.time_usec_GET() == 8100434737309926475L);
            assert(pack.chan1_raw_GET() == (char)5353);
            assert(pack.chan10_raw_GET() == (char)39975);
            assert(pack.chan5_raw_GET() == (char)17530);
            assert(pack.chan6_raw_GET() == (char)43685);
            assert(pack.chan9_raw_GET() == (char)30308);
            assert(pack.chan8_raw_GET() == (char)40080);
            assert(pack.chan11_raw_GET() == (char)10317);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan4_raw_SET((char)46903) ;
        p92.chan8_raw_SET((char)40080) ;
        p92.chan9_raw_SET((char)30308) ;
        p92.chan10_raw_SET((char)39975) ;
        p92.chan1_raw_SET((char)5353) ;
        p92.rssi_SET((char)156) ;
        p92.chan11_raw_SET((char)10317) ;
        p92.time_usec_SET(8100434737309926475L) ;
        p92.chan3_raw_SET((char)20179) ;
        p92.chan12_raw_SET((char)9852) ;
        p92.chan2_raw_SET((char)44235) ;
        p92.chan7_raw_SET((char)48483) ;
        p92.chan5_raw_SET((char)17530) ;
        p92.chan6_raw_SET((char)43685) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.1038265E38F, 2.8236077E38F, 1.4294115E38F, -8.741108E37F, -2.1517272E38F, 5.823214E37F, 2.6443098E38F, -1.4015167E38F, 1.4089651E38F, -1.7633598E38F, 2.9480768E38F, 2.4529913E38F, 1.9146073E38F, -1.6050182E38F, 1.7980533E38F, 2.5449102E38F}));
            assert(pack.flags_GET() == 609004549223183055L);
            assert(pack.time_usec_GET() == 1047813837003683359L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(1047813837003683359L) ;
        p93.flags_SET(609004549223183055L) ;
        p93.controls_SET(new float[] {1.1038265E38F, 2.8236077E38F, 1.4294115E38F, -8.741108E37F, -2.1517272E38F, 5.823214E37F, 2.6443098E38F, -1.4015167E38F, 1.4089651E38F, -1.7633598E38F, 2.9480768E38F, 2.4529913E38F, 1.9146073E38F, -1.6050182E38F, 1.7980533E38F, 2.5449102E38F}, 0) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_comp_m_x_GET() == 1.0410011E38F);
            assert(pack.flow_rate_x_TRY(ph) == -2.446805E38F);
            assert(pack.ground_distance_GET() == -3.246142E37F);
            assert(pack.sensor_id_GET() == (char)93);
            assert(pack.flow_comp_m_y_GET() == 9.268212E37F);
            assert(pack.time_usec_GET() == 2431034754394014155L);
            assert(pack.flow_rate_y_TRY(ph) == -3.0850908E38F);
            assert(pack.flow_y_GET() == (short)12872);
            assert(pack.quality_GET() == (char)71);
            assert(pack.flow_x_GET() == (short) -26350);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.sensor_id_SET((char)93) ;
        p100.time_usec_SET(2431034754394014155L) ;
        p100.flow_y_SET((short)12872) ;
        p100.flow_comp_m_y_SET(9.268212E37F) ;
        p100.flow_comp_m_x_SET(1.0410011E38F) ;
        p100.flow_rate_x_SET(-2.446805E38F, PH) ;
        p100.flow_x_SET((short) -26350) ;
        p100.flow_rate_y_SET(-3.0850908E38F, PH) ;
        p100.quality_SET((char)71) ;
        p100.ground_distance_SET(-3.246142E37F) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -4.9811747E37F);
            assert(pack.y_GET() == -2.4225509E38F);
            assert(pack.usec_GET() == 376888955337163373L);
            assert(pack.yaw_GET() == -3.3695354E38F);
            assert(pack.roll_GET() == -2.2342425E38F);
            assert(pack.x_GET() == -2.7260313E38F);
            assert(pack.z_GET() == 2.4658753E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.z_SET(2.4658753E38F) ;
        p101.pitch_SET(-4.9811747E37F) ;
        p101.usec_SET(376888955337163373L) ;
        p101.yaw_SET(-3.3695354E38F) ;
        p101.roll_SET(-2.2342425E38F) ;
        p101.x_SET(-2.7260313E38F) ;
        p101.y_SET(-2.4225509E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 2.9996816E38F);
            assert(pack.y_GET() == -2.7553743E38F);
            assert(pack.usec_GET() == 3009122684186038952L);
            assert(pack.yaw_GET() == 1.595057E37F);
            assert(pack.z_GET() == -2.854461E38F);
            assert(pack.roll_GET() == 9.088418E37F);
            assert(pack.x_GET() == -3.2507886E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.roll_SET(9.088418E37F) ;
        p102.pitch_SET(2.9996816E38F) ;
        p102.x_SET(-3.2507886E38F) ;
        p102.usec_SET(3009122684186038952L) ;
        p102.z_SET(-2.854461E38F) ;
        p102.yaw_SET(1.595057E37F) ;
        p102.y_SET(-2.7553743E38F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 3.1274161E38F);
            assert(pack.z_GET() == 2.7219515E38F);
            assert(pack.x_GET() == -1.5346524E38F);
            assert(pack.usec_GET() == 8477828846147961782L);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.z_SET(2.7219515E38F) ;
        p103.y_SET(3.1274161E38F) ;
        p103.x_SET(-1.5346524E38F) ;
        p103.usec_SET(8477828846147961782L) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 6541606164234340701L);
            assert(pack.roll_GET() == -9.606289E37F);
            assert(pack.x_GET() == -4.8958116E37F);
            assert(pack.z_GET() == -1.5554227E38F);
            assert(pack.y_GET() == 2.0853848E38F);
            assert(pack.yaw_GET() == -1.5089937E38F);
            assert(pack.pitch_GET() == 3.3885696E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.roll_SET(-9.606289E37F) ;
        p104.y_SET(2.0853848E38F) ;
        p104.yaw_SET(-1.5089937E38F) ;
        p104.usec_SET(6541606164234340701L) ;
        p104.x_SET(-4.8958116E37F) ;
        p104.z_SET(-1.5554227E38F) ;
        p104.pitch_SET(3.3885696E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == -2.9618322E37F);
            assert(pack.xmag_GET() == -1.4443546E38F);
            assert(pack.xacc_GET() == -3.3081652E37F);
            assert(pack.diff_pressure_GET() == 1.0075323E38F);
            assert(pack.yacc_GET() == -2.8732836E38F);
            assert(pack.zmag_GET() == 1.425529E37F);
            assert(pack.fields_updated_GET() == (char)20181);
            assert(pack.abs_pressure_GET() == -2.3675127E38F);
            assert(pack.time_usec_GET() == 2612459345053694192L);
            assert(pack.temperature_GET() == 1.2286636E37F);
            assert(pack.ymag_GET() == 6.8507667E37F);
            assert(pack.pressure_alt_GET() == -2.4331468E38F);
            assert(pack.zacc_GET() == -3.1449618E38F);
            assert(pack.ygyro_GET() == -2.2536732E38F);
            assert(pack.xgyro_GET() == 2.7741694E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.zgyro_SET(-2.9618322E37F) ;
        p105.ymag_SET(6.8507667E37F) ;
        p105.time_usec_SET(2612459345053694192L) ;
        p105.diff_pressure_SET(1.0075323E38F) ;
        p105.xmag_SET(-1.4443546E38F) ;
        p105.zmag_SET(1.425529E37F) ;
        p105.pressure_alt_SET(-2.4331468E38F) ;
        p105.abs_pressure_SET(-2.3675127E38F) ;
        p105.temperature_SET(1.2286636E37F) ;
        p105.fields_updated_SET((char)20181) ;
        p105.xacc_SET(-3.3081652E37F) ;
        p105.ygyro_SET(-2.2536732E38F) ;
        p105.xgyro_SET(2.7741694E38F) ;
        p105.yacc_SET(-2.8732836E38F) ;
        p105.zacc_SET(-3.1449618E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_ygyro_GET() == -1.9280091E38F);
            assert(pack.time_delta_distance_us_GET() == 2468771211L);
            assert(pack.distance_GET() == 2.1274039E38F);
            assert(pack.integrated_zgyro_GET() == -2.572651E38F);
            assert(pack.temperature_GET() == (short) -28143);
            assert(pack.quality_GET() == (char)76);
            assert(pack.time_usec_GET() == 3097069461505308696L);
            assert(pack.integration_time_us_GET() == 3319812280L);
            assert(pack.integrated_y_GET() == -6.8103083E37F);
            assert(pack.integrated_x_GET() == -1.2922381E36F);
            assert(pack.integrated_xgyro_GET() == -1.763381E38F);
            assert(pack.sensor_id_GET() == (char)195);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_y_SET(-6.8103083E37F) ;
        p106.integrated_ygyro_SET(-1.9280091E38F) ;
        p106.integrated_zgyro_SET(-2.572651E38F) ;
        p106.integrated_xgyro_SET(-1.763381E38F) ;
        p106.time_delta_distance_us_SET(2468771211L) ;
        p106.sensor_id_SET((char)195) ;
        p106.integration_time_us_SET(3319812280L) ;
        p106.distance_SET(2.1274039E38F) ;
        p106.temperature_SET((short) -28143) ;
        p106.time_usec_SET(3097069461505308696L) ;
        p106.quality_SET((char)76) ;
        p106.integrated_x_SET(-1.2922381E36F) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.diff_pressure_GET() == 2.9398355E38F);
            assert(pack.temperature_GET() == 1.6792558E38F);
            assert(pack.yacc_GET() == 2.540162E38F);
            assert(pack.ymag_GET() == 1.3218676E38F);
            assert(pack.abs_pressure_GET() == 9.223283E37F);
            assert(pack.time_usec_GET() == 3512800161004697039L);
            assert(pack.xacc_GET() == -3.0467319E38F);
            assert(pack.pressure_alt_GET() == 6.024652E37F);
            assert(pack.xmag_GET() == -3.0908098E38F);
            assert(pack.zmag_GET() == 3.1387102E38F);
            assert(pack.xgyro_GET() == 3.1315909E38F);
            assert(pack.zacc_GET() == 2.9180787E38F);
            assert(pack.fields_updated_GET() == 1062827432L);
            assert(pack.zgyro_GET() == 1.849511E38F);
            assert(pack.ygyro_GET() == 2.0609394E37F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(3512800161004697039L) ;
        p107.xmag_SET(-3.0908098E38F) ;
        p107.temperature_SET(1.6792558E38F) ;
        p107.ygyro_SET(2.0609394E37F) ;
        p107.yacc_SET(2.540162E38F) ;
        p107.zacc_SET(2.9180787E38F) ;
        p107.ymag_SET(1.3218676E38F) ;
        p107.zmag_SET(3.1387102E38F) ;
        p107.xgyro_SET(3.1315909E38F) ;
        p107.fields_updated_SET(1062827432L) ;
        p107.pressure_alt_SET(6.024652E37F) ;
        p107.xacc_SET(-3.0467319E38F) ;
        p107.abs_pressure_SET(9.223283E37F) ;
        p107.diff_pressure_SET(2.9398355E38F) ;
        p107.zgyro_SET(1.849511E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.vn_GET() == -1.1818204E38F);
            assert(pack.lat_GET() == -2.9203895E38F);
            assert(pack.xgyro_GET() == -5.581833E37F);
            assert(pack.std_dev_vert_GET() == -1.3099854E37F);
            assert(pack.yacc_GET() == 8.465513E37F);
            assert(pack.alt_GET() == -3.401022E38F);
            assert(pack.zacc_GET() == -3.3688095E38F);
            assert(pack.ygyro_GET() == -3.3475754E38F);
            assert(pack.pitch_GET() == -9.073312E37F);
            assert(pack.yaw_GET() == -2.064126E38F);
            assert(pack.ve_GET() == 9.688561E37F);
            assert(pack.std_dev_horz_GET() == -2.3546874E37F);
            assert(pack.q1_GET() == -2.9819878E38F);
            assert(pack.q2_GET() == -2.9683596E38F);
            assert(pack.lon_GET() == -2.9202504E38F);
            assert(pack.vd_GET() == -1.6273139E38F);
            assert(pack.q4_GET() == -2.63507E38F);
            assert(pack.zgyro_GET() == -3.3349792E38F);
            assert(pack.q3_GET() == -3.26245E38F);
            assert(pack.roll_GET() == 3.2880187E38F);
            assert(pack.xacc_GET() == 2.145389E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.vd_SET(-1.6273139E38F) ;
        p108.vn_SET(-1.1818204E38F) ;
        p108.alt_SET(-3.401022E38F) ;
        p108.yaw_SET(-2.064126E38F) ;
        p108.zgyro_SET(-3.3349792E38F) ;
        p108.lat_SET(-2.9203895E38F) ;
        p108.q2_SET(-2.9683596E38F) ;
        p108.roll_SET(3.2880187E38F) ;
        p108.q1_SET(-2.9819878E38F) ;
        p108.zacc_SET(-3.3688095E38F) ;
        p108.std_dev_horz_SET(-2.3546874E37F) ;
        p108.q4_SET(-2.63507E38F) ;
        p108.yacc_SET(8.465513E37F) ;
        p108.q3_SET(-3.26245E38F) ;
        p108.ygyro_SET(-3.3475754E38F) ;
        p108.pitch_SET(-9.073312E37F) ;
        p108.std_dev_vert_SET(-1.3099854E37F) ;
        p108.xacc_SET(2.145389E38F) ;
        p108.ve_SET(9.688561E37F) ;
        p108.lon_SET(-2.9202504E38F) ;
        p108.xgyro_SET(-5.581833E37F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.fixed__GET() == (char)38384);
            assert(pack.remnoise_GET() == (char)156);
            assert(pack.rxerrors_GET() == (char)15763);
            assert(pack.noise_GET() == (char)109);
            assert(pack.rssi_GET() == (char)69);
            assert(pack.txbuf_GET() == (char)178);
            assert(pack.remrssi_GET() == (char)232);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.remnoise_SET((char)156) ;
        p109.rxerrors_SET((char)15763) ;
        p109.remrssi_SET((char)232) ;
        p109.rssi_SET((char)69) ;
        p109.noise_SET((char)109) ;
        p109.txbuf_SET((char)178) ;
        p109.fixed__SET((char)38384) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)145);
            assert(pack.target_system_GET() == (char)16);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)222, (char)179, (char)152, (char)127, (char)81, (char)10, (char)46, (char)151, (char)53, (char)145, (char)211, (char)139, (char)61, (char)1, (char)4, (char)70, (char)50, (char)173, (char)33, (char)176, (char)172, (char)28, (char)116, (char)143, (char)125, (char)228, (char)115, (char)200, (char)25, (char)12, (char)96, (char)182, (char)255, (char)110, (char)166, (char)229, (char)23, (char)31, (char)77, (char)122, (char)249, (char)252, (char)54, (char)112, (char)180, (char)202, (char)103, (char)18, (char)207, (char)139, (char)183, (char)105, (char)133, (char)30, (char)136, (char)150, (char)158, (char)2, (char)81, (char)144, (char)101, (char)83, (char)196, (char)8, (char)62, (char)217, (char)151, (char)183, (char)16, (char)118, (char)83, (char)162, (char)122, (char)31, (char)170, (char)76, (char)232, (char)235, (char)179, (char)145, (char)151, (char)211, (char)239, (char)35, (char)108, (char)232, (char)202, (char)124, (char)184, (char)59, (char)18, (char)255, (char)68, (char)249, (char)184, (char)243, (char)94, (char)239, (char)4, (char)80, (char)107, (char)28, (char)222, (char)255, (char)44, (char)51, (char)156, (char)105, (char)132, (char)178, (char)108, (char)2, (char)92, (char)65, (char)103, (char)202, (char)79, (char)219, (char)169, (char)173, (char)23, (char)9, (char)80, (char)95, (char)122, (char)241, (char)21, (char)150, (char)52, (char)94, (char)94, (char)97, (char)196, (char)135, (char)32, (char)68, (char)17, (char)130, (char)120, (char)158, (char)215, (char)172, (char)252, (char)71, (char)128, (char)148, (char)13, (char)245, (char)90, (char)194, (char)117, (char)250, (char)197, (char)155, (char)201, (char)159, (char)106, (char)223, (char)203, (char)119, (char)189, (char)191, (char)106, (char)122, (char)73, (char)94, (char)13, (char)190, (char)73, (char)8, (char)212, (char)33, (char)248, (char)56, (char)132, (char)119, (char)147, (char)232, (char)222, (char)60, (char)85, (char)19, (char)238, (char)155, (char)160, (char)114, (char)231, (char)223, (char)31, (char)37, (char)162, (char)14, (char)172, (char)109, (char)134, (char)164, (char)124, (char)141, (char)185, (char)13, (char)58, (char)109, (char)142, (char)214, (char)40, (char)228, (char)150, (char)43, (char)56, (char)39, (char)144, (char)90, (char)130, (char)33, (char)123, (char)115, (char)38, (char)8, (char)244, (char)185, (char)132, (char)62, (char)152, (char)52, (char)233, (char)149, (char)26, (char)178, (char)234, (char)7, (char)135, (char)7, (char)147, (char)67, (char)172, (char)137, (char)35, (char)107, (char)252, (char)230, (char)156, (char)20, (char)161, (char)185, (char)32, (char)152, (char)127, (char)158, (char)216, (char)185, (char)193}));
            assert(pack.target_network_GET() == (char)161);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_system_SET((char)16) ;
        p110.payload_SET(new char[] {(char)222, (char)179, (char)152, (char)127, (char)81, (char)10, (char)46, (char)151, (char)53, (char)145, (char)211, (char)139, (char)61, (char)1, (char)4, (char)70, (char)50, (char)173, (char)33, (char)176, (char)172, (char)28, (char)116, (char)143, (char)125, (char)228, (char)115, (char)200, (char)25, (char)12, (char)96, (char)182, (char)255, (char)110, (char)166, (char)229, (char)23, (char)31, (char)77, (char)122, (char)249, (char)252, (char)54, (char)112, (char)180, (char)202, (char)103, (char)18, (char)207, (char)139, (char)183, (char)105, (char)133, (char)30, (char)136, (char)150, (char)158, (char)2, (char)81, (char)144, (char)101, (char)83, (char)196, (char)8, (char)62, (char)217, (char)151, (char)183, (char)16, (char)118, (char)83, (char)162, (char)122, (char)31, (char)170, (char)76, (char)232, (char)235, (char)179, (char)145, (char)151, (char)211, (char)239, (char)35, (char)108, (char)232, (char)202, (char)124, (char)184, (char)59, (char)18, (char)255, (char)68, (char)249, (char)184, (char)243, (char)94, (char)239, (char)4, (char)80, (char)107, (char)28, (char)222, (char)255, (char)44, (char)51, (char)156, (char)105, (char)132, (char)178, (char)108, (char)2, (char)92, (char)65, (char)103, (char)202, (char)79, (char)219, (char)169, (char)173, (char)23, (char)9, (char)80, (char)95, (char)122, (char)241, (char)21, (char)150, (char)52, (char)94, (char)94, (char)97, (char)196, (char)135, (char)32, (char)68, (char)17, (char)130, (char)120, (char)158, (char)215, (char)172, (char)252, (char)71, (char)128, (char)148, (char)13, (char)245, (char)90, (char)194, (char)117, (char)250, (char)197, (char)155, (char)201, (char)159, (char)106, (char)223, (char)203, (char)119, (char)189, (char)191, (char)106, (char)122, (char)73, (char)94, (char)13, (char)190, (char)73, (char)8, (char)212, (char)33, (char)248, (char)56, (char)132, (char)119, (char)147, (char)232, (char)222, (char)60, (char)85, (char)19, (char)238, (char)155, (char)160, (char)114, (char)231, (char)223, (char)31, (char)37, (char)162, (char)14, (char)172, (char)109, (char)134, (char)164, (char)124, (char)141, (char)185, (char)13, (char)58, (char)109, (char)142, (char)214, (char)40, (char)228, (char)150, (char)43, (char)56, (char)39, (char)144, (char)90, (char)130, (char)33, (char)123, (char)115, (char)38, (char)8, (char)244, (char)185, (char)132, (char)62, (char)152, (char)52, (char)233, (char)149, (char)26, (char)178, (char)234, (char)7, (char)135, (char)7, (char)147, (char)67, (char)172, (char)137, (char)35, (char)107, (char)252, (char)230, (char)156, (char)20, (char)161, (char)185, (char)32, (char)152, (char)127, (char)158, (char)216, (char)185, (char)193}, 0) ;
        p110.target_network_SET((char)161) ;
        p110.target_component_SET((char)145) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == 1320609305743918098L);
            assert(pack.tc1_GET() == 7670377019637167925L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(1320609305743918098L) ;
        p111.tc1_SET(7670377019637167925L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 2516299402L);
            assert(pack.time_usec_GET() == 2650508662357949767L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(2650508662357949767L) ;
        p112.seq_SET(2516299402L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1915366269);
            assert(pack.cog_GET() == (char)13363);
            assert(pack.eph_GET() == (char)13301);
            assert(pack.vn_GET() == (short) -30732);
            assert(pack.vel_GET() == (char)21034);
            assert(pack.epv_GET() == (char)13600);
            assert(pack.ve_GET() == (short)27174);
            assert(pack.fix_type_GET() == (char)186);
            assert(pack.time_usec_GET() == 1101652258237894973L);
            assert(pack.alt_GET() == -1424769634);
            assert(pack.satellites_visible_GET() == (char)161);
            assert(pack.vd_GET() == (short)12247);
            assert(pack.lon_GET() == 483020059);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.eph_SET((char)13301) ;
        p113.time_usec_SET(1101652258237894973L) ;
        p113.alt_SET(-1424769634) ;
        p113.ve_SET((short)27174) ;
        p113.vd_SET((short)12247) ;
        p113.vel_SET((char)21034) ;
        p113.satellites_visible_SET((char)161) ;
        p113.epv_SET((char)13600) ;
        p113.lon_SET(483020059) ;
        p113.cog_SET((char)13363) ;
        p113.fix_type_SET((char)186) ;
        p113.lat_SET(-1915366269) ;
        p113.vn_SET((short) -30732) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.time_delta_distance_us_GET() == 3828646682L);
            assert(pack.distance_GET() == -2.6163037E38F);
            assert(pack.temperature_GET() == (short)32219);
            assert(pack.integrated_ygyro_GET() == 1.0690948E38F);
            assert(pack.integrated_xgyro_GET() == 2.1277982E37F);
            assert(pack.quality_GET() == (char)44);
            assert(pack.integrated_x_GET() == 3.665909E37F);
            assert(pack.time_usec_GET() == 7068286818582053087L);
            assert(pack.integrated_y_GET() == -2.5791796E38F);
            assert(pack.integration_time_us_GET() == 2254837860L);
            assert(pack.sensor_id_GET() == (char)221);
            assert(pack.integrated_zgyro_GET() == 2.742258E38F);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.distance_SET(-2.6163037E38F) ;
        p114.time_delta_distance_us_SET(3828646682L) ;
        p114.integration_time_us_SET(2254837860L) ;
        p114.integrated_y_SET(-2.5791796E38F) ;
        p114.integrated_x_SET(3.665909E37F) ;
        p114.time_usec_SET(7068286818582053087L) ;
        p114.temperature_SET((short)32219) ;
        p114.integrated_ygyro_SET(1.0690948E38F) ;
        p114.quality_SET((char)44) ;
        p114.integrated_xgyro_SET(2.1277982E37F) ;
        p114.sensor_id_SET((char)221) ;
        p114.integrated_zgyro_SET(2.742258E38F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == (short) -4653);
            assert(pack.pitchspeed_GET() == -1.8891884E38F);
            assert(pack.lon_GET() == -2009658675);
            assert(pack.true_airspeed_GET() == (char)18340);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {3.9464704E37F, 2.708834E38F, -2.613886E38F, -2.415867E38F}));
            assert(pack.yawspeed_GET() == 3.034016E38F);
            assert(pack.xacc_GET() == (short)24484);
            assert(pack.zacc_GET() == (short)16404);
            assert(pack.ind_airspeed_GET() == (char)61155);
            assert(pack.vy_GET() == (short) -216);
            assert(pack.alt_GET() == -921155547);
            assert(pack.rollspeed_GET() == 7.3012806E37F);
            assert(pack.time_usec_GET() == 2491103564701928794L);
            assert(pack.lat_GET() == 1073817144);
            assert(pack.vx_GET() == (short)30931);
            assert(pack.yacc_GET() == (short)20973);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.vz_SET((short) -4653) ;
        p115.rollspeed_SET(7.3012806E37F) ;
        p115.alt_SET(-921155547) ;
        p115.ind_airspeed_SET((char)61155) ;
        p115.vy_SET((short) -216) ;
        p115.attitude_quaternion_SET(new float[] {3.9464704E37F, 2.708834E38F, -2.613886E38F, -2.415867E38F}, 0) ;
        p115.time_usec_SET(2491103564701928794L) ;
        p115.lon_SET(-2009658675) ;
        p115.yacc_SET((short)20973) ;
        p115.vx_SET((short)30931) ;
        p115.lat_SET(1073817144) ;
        p115.zacc_SET((short)16404) ;
        p115.yawspeed_SET(3.034016E38F) ;
        p115.pitchspeed_SET(-1.8891884E38F) ;
        p115.true_airspeed_SET((char)18340) ;
        p115.xacc_SET((short)24484) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == (short)10021);
            assert(pack.zgyro_GET() == (short) -10896);
            assert(pack.xgyro_GET() == (short)28796);
            assert(pack.ygyro_GET() == (short)7951);
            assert(pack.xacc_GET() == (short) -3425);
            assert(pack.zacc_GET() == (short)31316);
            assert(pack.time_boot_ms_GET() == 1214688999L);
            assert(pack.zmag_GET() == (short)2499);
            assert(pack.yacc_GET() == (short)20823);
            assert(pack.xmag_GET() == (short) -25318);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.ygyro_SET((short)7951) ;
        p116.zmag_SET((short)2499) ;
        p116.ymag_SET((short)10021) ;
        p116.time_boot_ms_SET(1214688999L) ;
        p116.yacc_SET((short)20823) ;
        p116.xmag_SET((short) -25318) ;
        p116.zacc_SET((short)31316) ;
        p116.xgyro_SET((short)28796) ;
        p116.xacc_SET((short) -3425) ;
        p116.zgyro_SET((short) -10896) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_GET() == (char)53040);
            assert(pack.target_system_GET() == (char)62);
            assert(pack.target_component_GET() == (char)193);
            assert(pack.start_GET() == (char)37296);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.end_SET((char)53040) ;
        p117.target_component_SET((char)193) ;
        p117.start_SET((char)37296) ;
        p117.target_system_SET((char)62) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.time_utc_GET() == 3580172056L);
            assert(pack.last_log_num_GET() == (char)24464);
            assert(pack.id_GET() == (char)18694);
            assert(pack.size_GET() == 2648173969L);
            assert(pack.num_logs_GET() == (char)12169);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.last_log_num_SET((char)24464) ;
        p118.time_utc_SET(3580172056L) ;
        p118.num_logs_SET((char)12169) ;
        p118.size_SET(2648173969L) ;
        p118.id_SET((char)18694) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)43514);
            assert(pack.target_component_GET() == (char)114);
            assert(pack.ofs_GET() == 3736117097L);
            assert(pack.target_system_GET() == (char)3);
            assert(pack.count_GET() == 649512092L);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)3) ;
        p119.ofs_SET(3736117097L) ;
        p119.target_component_SET((char)114) ;
        p119.count_SET(649512092L) ;
        p119.id_SET((char)43514) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)18147);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)93, (char)209, (char)118, (char)176, (char)2, (char)159, (char)38, (char)131, (char)172, (char)46, (char)206, (char)60, (char)163, (char)98, (char)169, (char)230, (char)191, (char)223, (char)202, (char)190, (char)91, (char)114, (char)182, (char)148, (char)174, (char)9, (char)111, (char)132, (char)218, (char)255, (char)79, (char)236, (char)16, (char)174, (char)235, (char)232, (char)127, (char)97, (char)93, (char)179, (char)224, (char)150, (char)179, (char)153, (char)91, (char)62, (char)76, (char)5, (char)176, (char)44, (char)124, (char)55, (char)207, (char)182, (char)200, (char)20, (char)141, (char)124, (char)145, (char)92, (char)91, (char)35, (char)222, (char)72, (char)138, (char)122, (char)100, (char)18, (char)6, (char)0, (char)114, (char)96, (char)28, (char)211, (char)216, (char)156, (char)158, (char)25, (char)44, (char)40, (char)222, (char)117, (char)87, (char)121, (char)84, (char)184, (char)137, (char)167, (char)129, (char)206}));
            assert(pack.ofs_GET() == 3981015240L);
            assert(pack.count_GET() == (char)214);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.ofs_SET(3981015240L) ;
        p120.id_SET((char)18147) ;
        p120.count_SET((char)214) ;
        p120.data__SET(new char[] {(char)93, (char)209, (char)118, (char)176, (char)2, (char)159, (char)38, (char)131, (char)172, (char)46, (char)206, (char)60, (char)163, (char)98, (char)169, (char)230, (char)191, (char)223, (char)202, (char)190, (char)91, (char)114, (char)182, (char)148, (char)174, (char)9, (char)111, (char)132, (char)218, (char)255, (char)79, (char)236, (char)16, (char)174, (char)235, (char)232, (char)127, (char)97, (char)93, (char)179, (char)224, (char)150, (char)179, (char)153, (char)91, (char)62, (char)76, (char)5, (char)176, (char)44, (char)124, (char)55, (char)207, (char)182, (char)200, (char)20, (char)141, (char)124, (char)145, (char)92, (char)91, (char)35, (char)222, (char)72, (char)138, (char)122, (char)100, (char)18, (char)6, (char)0, (char)114, (char)96, (char)28, (char)211, (char)216, (char)156, (char)158, (char)25, (char)44, (char)40, (char)222, (char)117, (char)87, (char)121, (char)84, (char)184, (char)137, (char)167, (char)129, (char)206}, 0) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)219);
            assert(pack.target_component_GET() == (char)28);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)28) ;
        p121.target_system_SET((char)219) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)0);
            assert(pack.target_component_GET() == (char)11);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_component_SET((char)11) ;
        p122.target_system_SET((char)0) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)6);
            assert(pack.target_component_GET() == (char)79);
            assert(pack.target_system_GET() == (char)2);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)110, (char)105, (char)181, (char)146, (char)245, (char)206, (char)175, (char)114, (char)226, (char)50, (char)100, (char)37, (char)1, (char)28, (char)130, (char)183, (char)11, (char)16, (char)45, (char)88, (char)141, (char)177, (char)121, (char)163, (char)35, (char)255, (char)171, (char)255, (char)211, (char)81, (char)202, (char)117, (char)241, (char)91, (char)243, (char)8, (char)234, (char)18, (char)59, (char)140, (char)27, (char)41, (char)9, (char)0, (char)39, (char)126, (char)199, (char)99, (char)224, (char)107, (char)25, (char)225, (char)73, (char)56, (char)112, (char)235, (char)151, (char)98, (char)129, (char)116, (char)104, (char)109, (char)121, (char)239, (char)191, (char)41, (char)252, (char)95, (char)44, (char)111, (char)72, (char)167, (char)60, (char)126, (char)249, (char)161, (char)102, (char)56, (char)186, (char)226, (char)153, (char)173, (char)98, (char)131, (char)38, (char)91, (char)84, (char)242, (char)115, (char)148, (char)92, (char)116, (char)49, (char)93, (char)222, (char)34, (char)153, (char)226, (char)92, (char)173, (char)195, (char)167, (char)46, (char)164, (char)245, (char)91, (char)38, (char)151, (char)199, (char)251}));
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.len_SET((char)6) ;
        p123.target_component_SET((char)79) ;
        p123.data__SET(new char[] {(char)110, (char)105, (char)181, (char)146, (char)245, (char)206, (char)175, (char)114, (char)226, (char)50, (char)100, (char)37, (char)1, (char)28, (char)130, (char)183, (char)11, (char)16, (char)45, (char)88, (char)141, (char)177, (char)121, (char)163, (char)35, (char)255, (char)171, (char)255, (char)211, (char)81, (char)202, (char)117, (char)241, (char)91, (char)243, (char)8, (char)234, (char)18, (char)59, (char)140, (char)27, (char)41, (char)9, (char)0, (char)39, (char)126, (char)199, (char)99, (char)224, (char)107, (char)25, (char)225, (char)73, (char)56, (char)112, (char)235, (char)151, (char)98, (char)129, (char)116, (char)104, (char)109, (char)121, (char)239, (char)191, (char)41, (char)252, (char)95, (char)44, (char)111, (char)72, (char)167, (char)60, (char)126, (char)249, (char)161, (char)102, (char)56, (char)186, (char)226, (char)153, (char)173, (char)98, (char)131, (char)38, (char)91, (char)84, (char)242, (char)115, (char)148, (char)92, (char)116, (char)49, (char)93, (char)222, (char)34, (char)153, (char)226, (char)92, (char)173, (char)195, (char)167, (char)46, (char)164, (char)245, (char)91, (char)38, (char)151, (char)199, (char)251}, 0) ;
        p123.target_system_SET((char)2) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -609043754);
            assert(pack.cog_GET() == (char)46820);
            assert(pack.dgps_numch_GET() == (char)192);
            assert(pack.vel_GET() == (char)29319);
            assert(pack.dgps_age_GET() == 641863571L);
            assert(pack.eph_GET() == (char)61981);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
            assert(pack.time_usec_GET() == 1782781240940622695L);
            assert(pack.epv_GET() == (char)37118);
            assert(pack.lat_GET() == -1520253931);
            assert(pack.satellites_visible_GET() == (char)230);
            assert(pack.lon_GET() == -22901357);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.dgps_numch_SET((char)192) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX) ;
        p124.vel_SET((char)29319) ;
        p124.time_usec_SET(1782781240940622695L) ;
        p124.satellites_visible_SET((char)230) ;
        p124.lon_SET(-22901357) ;
        p124.eph_SET((char)61981) ;
        p124.lat_SET(-1520253931) ;
        p124.cog_SET((char)46820) ;
        p124.epv_SET((char)37118) ;
        p124.dgps_age_SET(641863571L) ;
        p124.alt_SET(-609043754) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vcc_GET() == (char)54794);
            assert(pack.Vservo_GET() == (char)24648);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED));
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED)) ;
        p125.Vcc_SET((char)54794) ;
        p125.Vservo_SET((char)24648) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)136, (char)165, (char)144, (char)191, (char)129, (char)43, (char)61, (char)99, (char)136, (char)251, (char)217, (char)254, (char)111, (char)44, (char)212, (char)99, (char)139, (char)196, (char)255, (char)109, (char)75, (char)213, (char)153, (char)86, (char)3, (char)101, (char)119, (char)174, (char)249, (char)126, (char)59, (char)47, (char)212, (char)14, (char)110, (char)184, (char)28, (char)198, (char)101, (char)244, (char)15, (char)3, (char)241, (char)186, (char)152, (char)66, (char)105, (char)44, (char)249, (char)71, (char)234, (char)31, (char)40, (char)190, (char)100, (char)106, (char)235, (char)117, (char)153, (char)77, (char)205, (char)224, (char)13, (char)68, (char)154, (char)24, (char)12, (char)45, (char)60, (char)200}));
            assert(pack.count_GET() == (char)139);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI));
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
            assert(pack.baudrate_GET() == 2654477688L);
            assert(pack.timeout_GET() == (char)8567);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.data__SET(new char[] {(char)136, (char)165, (char)144, (char)191, (char)129, (char)43, (char)61, (char)99, (char)136, (char)251, (char)217, (char)254, (char)111, (char)44, (char)212, (char)99, (char)139, (char)196, (char)255, (char)109, (char)75, (char)213, (char)153, (char)86, (char)3, (char)101, (char)119, (char)174, (char)249, (char)126, (char)59, (char)47, (char)212, (char)14, (char)110, (char)184, (char)28, (char)198, (char)101, (char)244, (char)15, (char)3, (char)241, (char)186, (char)152, (char)66, (char)105, (char)44, (char)249, (char)71, (char)234, (char)31, (char)40, (char)190, (char)100, (char)106, (char)235, (char)117, (char)153, (char)77, (char)205, (char)224, (char)13, (char)68, (char)154, (char)24, (char)12, (char)45, (char)60, (char)200}, 0) ;
        p126.baudrate_SET(2654477688L) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI)) ;
        p126.timeout_SET((char)8567) ;
        p126.count_SET((char)139) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_b_mm_GET() == 2140816832);
            assert(pack.tow_GET() == 613105462L);
            assert(pack.time_last_baseline_ms_GET() == 3955188712L);
            assert(pack.rtk_receiver_id_GET() == (char)33);
            assert(pack.rtk_health_GET() == (char)6);
            assert(pack.iar_num_hypotheses_GET() == 1360445259);
            assert(pack.wn_GET() == (char)57229);
            assert(pack.rtk_rate_GET() == (char)8);
            assert(pack.baseline_coords_type_GET() == (char)132);
            assert(pack.baseline_c_mm_GET() == 405770908);
            assert(pack.baseline_a_mm_GET() == -647403842);
            assert(pack.nsats_GET() == (char)66);
            assert(pack.accuracy_GET() == 1966575496L);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.wn_SET((char)57229) ;
        p127.baseline_b_mm_SET(2140816832) ;
        p127.baseline_c_mm_SET(405770908) ;
        p127.rtk_rate_SET((char)8) ;
        p127.baseline_a_mm_SET(-647403842) ;
        p127.time_last_baseline_ms_SET(3955188712L) ;
        p127.baseline_coords_type_SET((char)132) ;
        p127.iar_num_hypotheses_SET(1360445259) ;
        p127.accuracy_SET(1966575496L) ;
        p127.rtk_health_SET((char)6) ;
        p127.tow_SET(613105462L) ;
        p127.rtk_receiver_id_SET((char)33) ;
        p127.nsats_SET((char)66) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.tow_GET() == 2195030048L);
            assert(pack.rtk_rate_GET() == (char)186);
            assert(pack.baseline_a_mm_GET() == 79936618);
            assert(pack.wn_GET() == (char)13729);
            assert(pack.baseline_b_mm_GET() == -1313139601);
            assert(pack.rtk_health_GET() == (char)35);
            assert(pack.baseline_coords_type_GET() == (char)253);
            assert(pack.nsats_GET() == (char)230);
            assert(pack.rtk_receiver_id_GET() == (char)31);
            assert(pack.baseline_c_mm_GET() == 1028294997);
            assert(pack.iar_num_hypotheses_GET() == -627642222);
            assert(pack.time_last_baseline_ms_GET() == 2119748075L);
            assert(pack.accuracy_GET() == 1834763078L);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_b_mm_SET(-1313139601) ;
        p128.baseline_a_mm_SET(79936618) ;
        p128.nsats_SET((char)230) ;
        p128.iar_num_hypotheses_SET(-627642222) ;
        p128.wn_SET((char)13729) ;
        p128.rtk_rate_SET((char)186) ;
        p128.accuracy_SET(1834763078L) ;
        p128.rtk_receiver_id_SET((char)31) ;
        p128.tow_SET(2195030048L) ;
        p128.time_last_baseline_ms_SET(2119748075L) ;
        p128.baseline_c_mm_SET(1028294997) ;
        p128.rtk_health_SET((char)35) ;
        p128.baseline_coords_type_SET((char)253) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short)22448);
            assert(pack.xgyro_GET() == (short) -18337);
            assert(pack.zmag_GET() == (short) -5331);
            assert(pack.xacc_GET() == (short) -8539);
            assert(pack.yacc_GET() == (short)11087);
            assert(pack.time_boot_ms_GET() == 1593223914L);
            assert(pack.ygyro_GET() == (short)13905);
            assert(pack.xmag_GET() == (short)10490);
            assert(pack.ymag_GET() == (short) -1749);
            assert(pack.zacc_GET() == (short) -30236);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.zmag_SET((short) -5331) ;
        p129.xgyro_SET((short) -18337) ;
        p129.zacc_SET((short) -30236) ;
        p129.xmag_SET((short)10490) ;
        p129.ygyro_SET((short)13905) ;
        p129.zgyro_SET((short)22448) ;
        p129.ymag_SET((short) -1749) ;
        p129.time_boot_ms_SET(1593223914L) ;
        p129.yacc_SET((short)11087) ;
        p129.xacc_SET((short) -8539) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.height_GET() == (char)12846);
            assert(pack.payload_GET() == (char)43);
            assert(pack.packets_GET() == (char)59526);
            assert(pack.type_GET() == (char)112);
            assert(pack.jpg_quality_GET() == (char)5);
            assert(pack.size_GET() == 2886702190L);
            assert(pack.width_GET() == (char)7070);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.size_SET(2886702190L) ;
        p130.height_SET((char)12846) ;
        p130.type_SET((char)112) ;
        p130.payload_SET((char)43) ;
        p130.packets_SET((char)59526) ;
        p130.jpg_quality_SET((char)5) ;
        p130.width_SET((char)7070) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)29587);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)202, (char)84, (char)106, (char)181, (char)221, (char)9, (char)143, (char)91, (char)160, (char)110, (char)122, (char)67, (char)204, (char)154, (char)75, (char)252, (char)173, (char)60, (char)225, (char)169, (char)73, (char)165, (char)45, (char)92, (char)55, (char)130, (char)186, (char)79, (char)142, (char)106, (char)89, (char)17, (char)66, (char)194, (char)52, (char)94, (char)34, (char)126, (char)138, (char)204, (char)31, (char)233, (char)232, (char)205, (char)59, (char)83, (char)219, (char)226, (char)9, (char)197, (char)138, (char)153, (char)185, (char)219, (char)128, (char)37, (char)185, (char)2, (char)152, (char)78, (char)255, (char)43, (char)139, (char)81, (char)149, (char)142, (char)44, (char)251, (char)178, (char)116, (char)60, (char)110, (char)134, (char)242, (char)8, (char)209, (char)181, (char)106, (char)182, (char)104, (char)217, (char)157, (char)228, (char)111, (char)159, (char)19, (char)196, (char)185, (char)149, (char)74, (char)104, (char)155, (char)210, (char)56, (char)241, (char)76, (char)75, (char)240, (char)133, (char)212, (char)119, (char)197, (char)77, (char)4, (char)35, (char)100, (char)55, (char)28, (char)130, (char)103, (char)188, (char)45, (char)198, (char)134, (char)84, (char)87, (char)87, (char)104, (char)3, (char)53, (char)204, (char)74, (char)198, (char)228, (char)49, (char)43, (char)254, (char)16, (char)41, (char)121, (char)91, (char)144, (char)219, (char)141, (char)222, (char)110, (char)96, (char)85, (char)124, (char)161, (char)200, (char)204, (char)183, (char)161, (char)105, (char)183, (char)181, (char)59, (char)99, (char)147, (char)141, (char)44, (char)124, (char)134, (char)151, (char)172, (char)125, (char)151, (char)175, (char)246, (char)38, (char)77, (char)49, (char)204, (char)99, (char)92, (char)139, (char)144, (char)109, (char)252, (char)136, (char)167, (char)242, (char)155, (char)36, (char)155, (char)160, (char)14, (char)11, (char)67, (char)18, (char)226, (char)244, (char)22, (char)15, (char)161, (char)177, (char)105, (char)1, (char)104, (char)67, (char)211, (char)22, (char)200, (char)255, (char)29, (char)29, (char)252, (char)170, (char)165, (char)172, (char)106, (char)57, (char)123, (char)193, (char)212, (char)94, (char)37, (char)69, (char)43, (char)104, (char)165, (char)55, (char)79, (char)150, (char)168, (char)86, (char)49, (char)199, (char)240, (char)45, (char)210, (char)164, (char)47, (char)40, (char)227, (char)90, (char)83, (char)26, (char)32, (char)98, (char)118, (char)66, (char)217, (char)234, (char)71, (char)145, (char)192, (char)116, (char)51, (char)85, (char)139, (char)146, (char)83, (char)91, (char)34, (char)81, (char)183, (char)232, (char)251, (char)185, (char)238, (char)6}));
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.data__SET(new char[] {(char)202, (char)84, (char)106, (char)181, (char)221, (char)9, (char)143, (char)91, (char)160, (char)110, (char)122, (char)67, (char)204, (char)154, (char)75, (char)252, (char)173, (char)60, (char)225, (char)169, (char)73, (char)165, (char)45, (char)92, (char)55, (char)130, (char)186, (char)79, (char)142, (char)106, (char)89, (char)17, (char)66, (char)194, (char)52, (char)94, (char)34, (char)126, (char)138, (char)204, (char)31, (char)233, (char)232, (char)205, (char)59, (char)83, (char)219, (char)226, (char)9, (char)197, (char)138, (char)153, (char)185, (char)219, (char)128, (char)37, (char)185, (char)2, (char)152, (char)78, (char)255, (char)43, (char)139, (char)81, (char)149, (char)142, (char)44, (char)251, (char)178, (char)116, (char)60, (char)110, (char)134, (char)242, (char)8, (char)209, (char)181, (char)106, (char)182, (char)104, (char)217, (char)157, (char)228, (char)111, (char)159, (char)19, (char)196, (char)185, (char)149, (char)74, (char)104, (char)155, (char)210, (char)56, (char)241, (char)76, (char)75, (char)240, (char)133, (char)212, (char)119, (char)197, (char)77, (char)4, (char)35, (char)100, (char)55, (char)28, (char)130, (char)103, (char)188, (char)45, (char)198, (char)134, (char)84, (char)87, (char)87, (char)104, (char)3, (char)53, (char)204, (char)74, (char)198, (char)228, (char)49, (char)43, (char)254, (char)16, (char)41, (char)121, (char)91, (char)144, (char)219, (char)141, (char)222, (char)110, (char)96, (char)85, (char)124, (char)161, (char)200, (char)204, (char)183, (char)161, (char)105, (char)183, (char)181, (char)59, (char)99, (char)147, (char)141, (char)44, (char)124, (char)134, (char)151, (char)172, (char)125, (char)151, (char)175, (char)246, (char)38, (char)77, (char)49, (char)204, (char)99, (char)92, (char)139, (char)144, (char)109, (char)252, (char)136, (char)167, (char)242, (char)155, (char)36, (char)155, (char)160, (char)14, (char)11, (char)67, (char)18, (char)226, (char)244, (char)22, (char)15, (char)161, (char)177, (char)105, (char)1, (char)104, (char)67, (char)211, (char)22, (char)200, (char)255, (char)29, (char)29, (char)252, (char)170, (char)165, (char)172, (char)106, (char)57, (char)123, (char)193, (char)212, (char)94, (char)37, (char)69, (char)43, (char)104, (char)165, (char)55, (char)79, (char)150, (char)168, (char)86, (char)49, (char)199, (char)240, (char)45, (char)210, (char)164, (char)47, (char)40, (char)227, (char)90, (char)83, (char)26, (char)32, (char)98, (char)118, (char)66, (char)217, (char)234, (char)71, (char)145, (char)192, (char)116, (char)51, (char)85, (char)139, (char)146, (char)83, (char)91, (char)34, (char)81, (char)183, (char)232, (char)251, (char)185, (char)238, (char)6}, 0) ;
        p131.seqnr_SET((char)29587) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)45);
            assert(pack.time_boot_ms_GET() == 750332699L);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(pack.min_distance_GET() == (char)22840);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_90);
            assert(pack.covariance_GET() == (char)123);
            assert(pack.max_distance_GET() == (char)22828);
            assert(pack.current_distance_GET() == (char)37630);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.min_distance_SET((char)22840) ;
        p132.covariance_SET((char)123) ;
        p132.max_distance_SET((char)22828) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_90) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p132.time_boot_ms_SET(750332699L) ;
        p132.current_distance_SET((char)37630) ;
        p132.id_SET((char)45) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1761314321);
            assert(pack.mask_GET() == 1657033037637787859L);
            assert(pack.lon_GET() == 1784614785);
            assert(pack.grid_spacing_GET() == (char)26118);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lon_SET(1784614785) ;
        p133.lat_SET(1761314321) ;
        p133.grid_spacing_SET((char)26118) ;
        p133.mask_SET(1657033037637787859L) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1299062623);
            assert(pack.gridbit_GET() == (char)216);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)21791, (short)6498, (short)30282, (short)29339, (short)8071, (short)23163, (short)4984, (short) -4282, (short) -18287, (short) -15114, (short)31586, (short) -18273, (short)32215, (short) -21328, (short) -14460, (short) -7532}));
            assert(pack.grid_spacing_GET() == (char)4499);
            assert(pack.lon_GET() == 2035293889);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.grid_spacing_SET((char)4499) ;
        p134.lon_SET(2035293889) ;
        p134.lat_SET(-1299062623) ;
        p134.data__SET(new short[] {(short)21791, (short)6498, (short)30282, (short)29339, (short)8071, (short)23163, (short)4984, (short) -4282, (short) -18287, (short) -15114, (short)31586, (short) -18273, (short)32215, (short) -21328, (short) -14460, (short) -7532}, 0) ;
        p134.gridbit_SET((char)216) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1027770690);
            assert(pack.lon_GET() == -785787989);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-1027770690) ;
        p135.lon_SET(-785787989) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.loaded_GET() == (char)28187);
            assert(pack.lon_GET() == 465597107);
            assert(pack.lat_GET() == -1917052983);
            assert(pack.current_height_GET() == 1.909656E38F);
            assert(pack.terrain_height_GET() == 2.85602E38F);
            assert(pack.spacing_GET() == (char)23773);
            assert(pack.pending_GET() == (char)15081);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.spacing_SET((char)23773) ;
        p136.current_height_SET(1.909656E38F) ;
        p136.lon_SET(465597107) ;
        p136.lat_SET(-1917052983) ;
        p136.terrain_height_SET(2.85602E38F) ;
        p136.pending_SET((char)15081) ;
        p136.loaded_SET((char)28187) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -2.6427493E38F);
            assert(pack.time_boot_ms_GET() == 1696628358L);
            assert(pack.temperature_GET() == (short) -17590);
            assert(pack.press_diff_GET() == 5.019493E37F);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(1696628358L) ;
        p137.press_abs_SET(-2.6427493E38F) ;
        p137.temperature_SET((short) -17590) ;
        p137.press_diff_SET(5.019493E37F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 1.2622888E38F);
            assert(pack.x_GET() == -2.5341932E38F);
            assert(pack.time_usec_GET() == 6423697273472259089L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.5308875E38F, -1.7801662E38F, 2.0763802E38F, 2.8463021E38F}));
            assert(pack.y_GET() == 2.227481E38F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.x_SET(-2.5341932E38F) ;
        p138.time_usec_SET(6423697273472259089L) ;
        p138.q_SET(new float[] {-2.5308875E38F, -1.7801662E38F, 2.0763802E38F, 2.8463021E38F}, 0) ;
        p138.z_SET(1.2622888E38F) ;
        p138.y_SET(2.227481E38F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)172);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.0245347E38F, 5.9335246E37F, -2.1059776E38F, -2.9679402E38F, -5.862587E37F, 2.0634273E38F, 1.8932857E38F, -7.062549E37F}));
            assert(pack.time_usec_GET() == 8490891681199585586L);
            assert(pack.target_component_GET() == (char)243);
            assert(pack.group_mlx_GET() == (char)236);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.target_component_SET((char)243) ;
        p139.controls_SET(new float[] {1.0245347E38F, 5.9335246E37F, -2.1059776E38F, -2.9679402E38F, -5.862587E37F, 2.0634273E38F, 1.8932857E38F, -7.062549E37F}, 0) ;
        p139.group_mlx_SET((char)236) ;
        p139.target_system_SET((char)172) ;
        p139.time_usec_SET(8490891681199585586L) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5145361741916865997L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.5376694E38F, -8.596032E37F, -2.5062304E38F, -1.5362702E38F, 3.8683742E37F, -3.4712213E36F, 2.5015357E38F, -2.9394726E38F}));
            assert(pack.group_mlx_GET() == (char)89);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)89) ;
        p140.controls_SET(new float[] {-1.5376694E38F, -8.596032E37F, -2.5062304E38F, -1.5362702E38F, 3.8683742E37F, -3.4712213E36F, 2.5015357E38F, -2.9394726E38F}, 0) ;
        p140.time_usec_SET(5145361741916865997L) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_monotonic_GET() == 2.8783327E38F);
            assert(pack.bottom_clearance_GET() == 3.02363E38F);
            assert(pack.altitude_relative_GET() == 1.0141595E38F);
            assert(pack.time_usec_GET() == 5410815880962279961L);
            assert(pack.altitude_local_GET() == 2.421186E38F);
            assert(pack.altitude_amsl_GET() == 2.670422E38F);
            assert(pack.altitude_terrain_GET() == 2.821648E38F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_terrain_SET(2.821648E38F) ;
        p141.altitude_local_SET(2.421186E38F) ;
        p141.altitude_monotonic_SET(2.8783327E38F) ;
        p141.bottom_clearance_SET(3.02363E38F) ;
        p141.altitude_relative_SET(1.0141595E38F) ;
        p141.time_usec_SET(5410815880962279961L) ;
        p141.altitude_amsl_SET(2.670422E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.uri_type_GET() == (char)24);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)197, (char)73, (char)197, (char)114, (char)251, (char)8, (char)119, (char)57, (char)245, (char)161, (char)135, (char)82, (char)73, (char)229, (char)154, (char)49, (char)155, (char)60, (char)216, (char)215, (char)18, (char)123, (char)37, (char)167, (char)84, (char)105, (char)242, (char)249, (char)5, (char)133, (char)76, (char)207, (char)253, (char)242, (char)143, (char)226, (char)185, (char)37, (char)212, (char)75, (char)155, (char)131, (char)26, (char)3, (char)28, (char)4, (char)26, (char)56, (char)183, (char)1, (char)70, (char)55, (char)78, (char)59, (char)251, (char)211, (char)37, (char)141, (char)182, (char)251, (char)207, (char)133, (char)201, (char)171, (char)205, (char)125, (char)173, (char)54, (char)170, (char)172, (char)174, (char)68, (char)178, (char)53, (char)87, (char)211, (char)216, (char)144, (char)203, (char)234, (char)85, (char)93, (char)101, (char)173, (char)69, (char)216, (char)15, (char)114, (char)63, (char)218, (char)169, (char)195, (char)68, (char)23, (char)228, (char)153, (char)30, (char)206, (char)66, (char)136, (char)206, (char)227, (char)136, (char)41, (char)161, (char)139, (char)148, (char)105, (char)143, (char)204, (char)168, (char)27, (char)95, (char)182, (char)114, (char)110, (char)223, (char)191, (char)203, (char)110}));
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)188, (char)30, (char)246, (char)152, (char)76, (char)99, (char)207, (char)109, (char)106, (char)92, (char)74, (char)240, (char)179, (char)212, (char)89, (char)244, (char)244, (char)45, (char)51, (char)36, (char)202, (char)184, (char)45, (char)36, (char)231, (char)156, (char)213, (char)134, (char)60, (char)178, (char)88, (char)123, (char)223, (char)23, (char)155, (char)81, (char)250, (char)156, (char)34, (char)133, (char)141, (char)124, (char)41, (char)94, (char)241, (char)41, (char)80, (char)156, (char)222, (char)95, (char)49, (char)32, (char)121, (char)166, (char)2, (char)206, (char)43, (char)54, (char)110, (char)106, (char)241, (char)178, (char)210, (char)224, (char)25, (char)77, (char)24, (char)202, (char)154, (char)147, (char)101, (char)19, (char)66, (char)219, (char)222, (char)37, (char)84, (char)244, (char)39, (char)128, (char)22, (char)232, (char)210, (char)35, (char)150, (char)229, (char)1, (char)59, (char)182, (char)23, (char)76, (char)244, (char)155, (char)97, (char)179, (char)90, (char)95, (char)147, (char)222, (char)85, (char)233, (char)35, (char)123, (char)9, (char)149, (char)193, (char)75, (char)15, (char)31, (char)207, (char)165, (char)33, (char)166, (char)148, (char)77, (char)237, (char)146, (char)43, (char)119, (char)232}));
            assert(pack.transfer_type_GET() == (char)210);
            assert(pack.request_id_GET() == (char)108);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_type_SET((char)24) ;
        p142.uri_SET(new char[] {(char)188, (char)30, (char)246, (char)152, (char)76, (char)99, (char)207, (char)109, (char)106, (char)92, (char)74, (char)240, (char)179, (char)212, (char)89, (char)244, (char)244, (char)45, (char)51, (char)36, (char)202, (char)184, (char)45, (char)36, (char)231, (char)156, (char)213, (char)134, (char)60, (char)178, (char)88, (char)123, (char)223, (char)23, (char)155, (char)81, (char)250, (char)156, (char)34, (char)133, (char)141, (char)124, (char)41, (char)94, (char)241, (char)41, (char)80, (char)156, (char)222, (char)95, (char)49, (char)32, (char)121, (char)166, (char)2, (char)206, (char)43, (char)54, (char)110, (char)106, (char)241, (char)178, (char)210, (char)224, (char)25, (char)77, (char)24, (char)202, (char)154, (char)147, (char)101, (char)19, (char)66, (char)219, (char)222, (char)37, (char)84, (char)244, (char)39, (char)128, (char)22, (char)232, (char)210, (char)35, (char)150, (char)229, (char)1, (char)59, (char)182, (char)23, (char)76, (char)244, (char)155, (char)97, (char)179, (char)90, (char)95, (char)147, (char)222, (char)85, (char)233, (char)35, (char)123, (char)9, (char)149, (char)193, (char)75, (char)15, (char)31, (char)207, (char)165, (char)33, (char)166, (char)148, (char)77, (char)237, (char)146, (char)43, (char)119, (char)232}, 0) ;
        p142.storage_SET(new char[] {(char)197, (char)73, (char)197, (char)114, (char)251, (char)8, (char)119, (char)57, (char)245, (char)161, (char)135, (char)82, (char)73, (char)229, (char)154, (char)49, (char)155, (char)60, (char)216, (char)215, (char)18, (char)123, (char)37, (char)167, (char)84, (char)105, (char)242, (char)249, (char)5, (char)133, (char)76, (char)207, (char)253, (char)242, (char)143, (char)226, (char)185, (char)37, (char)212, (char)75, (char)155, (char)131, (char)26, (char)3, (char)28, (char)4, (char)26, (char)56, (char)183, (char)1, (char)70, (char)55, (char)78, (char)59, (char)251, (char)211, (char)37, (char)141, (char)182, (char)251, (char)207, (char)133, (char)201, (char)171, (char)205, (char)125, (char)173, (char)54, (char)170, (char)172, (char)174, (char)68, (char)178, (char)53, (char)87, (char)211, (char)216, (char)144, (char)203, (char)234, (char)85, (char)93, (char)101, (char)173, (char)69, (char)216, (char)15, (char)114, (char)63, (char)218, (char)169, (char)195, (char)68, (char)23, (char)228, (char)153, (char)30, (char)206, (char)66, (char)136, (char)206, (char)227, (char)136, (char)41, (char)161, (char)139, (char)148, (char)105, (char)143, (char)204, (char)168, (char)27, (char)95, (char)182, (char)114, (char)110, (char)223, (char)191, (char)203, (char)110}, 0) ;
        p142.transfer_type_SET((char)210) ;
        p142.request_id_SET((char)108) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == -3.3995909E38F);
            assert(pack.temperature_GET() == (short) -25487);
            assert(pack.time_boot_ms_GET() == 2489721653L);
            assert(pack.press_abs_GET() == -4.656691E37F);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.press_diff_SET(-3.3995909E38F) ;
        p143.time_boot_ms_SET(2489721653L) ;
        p143.press_abs_SET(-4.656691E37F) ;
        p143.temperature_SET((short) -25487) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-1.2531642E38F, 1.966021E38F, -3.005648E38F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-1.7368967E38F, -1.00328264E37F, 7.6389584E37F, -6.255125E37F}));
            assert(Arrays.equals(pack.vel_GET(),  new float[] {2.0381858E38F, -2.924179E38F, -9.364218E36F}));
            assert(pack.timestamp_GET() == 8028439428377499764L);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-5.5165203E37F, -1.6182622E37F, 3.3559397E38F}));
            assert(pack.lat_GET() == 1857884621);
            assert(pack.est_capabilities_GET() == (char)40);
            assert(pack.custom_state_GET() == 8575555686341947725L);
            assert(pack.lon_GET() == 153667101);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-1.4568999E38F, -1.3964615E38F, -3.0248437E38F}));
            assert(pack.alt_GET() == 1.8367507E38F);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.custom_state_SET(8575555686341947725L) ;
        p144.est_capabilities_SET((char)40) ;
        p144.vel_SET(new float[] {2.0381858E38F, -2.924179E38F, -9.364218E36F}, 0) ;
        p144.acc_SET(new float[] {-5.5165203E37F, -1.6182622E37F, 3.3559397E38F}, 0) ;
        p144.timestamp_SET(8028439428377499764L) ;
        p144.lat_SET(1857884621) ;
        p144.attitude_q_SET(new float[] {-1.7368967E38F, -1.00328264E37F, 7.6389584E37F, -6.255125E37F}, 0) ;
        p144.rates_SET(new float[] {-1.2531642E38F, 1.966021E38F, -3.005648E38F}, 0) ;
        p144.lon_SET(153667101) ;
        p144.position_cov_SET(new float[] {-1.4568999E38F, -1.3964615E38F, -3.0248437E38F}, 0) ;
        p144.alt_SET(1.8367507E38F) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.z_acc_GET() == 3.0595747E38F);
            assert(pack.y_acc_GET() == 5.896498E37F);
            assert(pack.x_pos_GET() == -1.9098785E38F);
            assert(pack.y_vel_GET() == -1.0177422E38F);
            assert(pack.y_pos_GET() == -2.8884934E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-2.9769326E38F, -1.5340616E38F, -8.29746E37F}));
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.6046265E38F, 3.3803493E38F, 3.3764967E38F, 1.0655549E38F}));
            assert(pack.z_vel_GET() == 1.8738732E38F);
            assert(pack.time_usec_GET() == 8611309491952932800L);
            assert(pack.z_pos_GET() == -2.0739847E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {2.6104248E38F, -1.4037593E38F, 1.1297006E38F}));
            assert(pack.x_acc_GET() == -2.6897966E38F);
            assert(pack.airspeed_GET() == -1.5793087E38F);
            assert(pack.yaw_rate_GET() == -3.4614926E37F);
            assert(pack.pitch_rate_GET() == 3.2598992E38F);
            assert(pack.x_vel_GET() == 3.8044332E37F);
            assert(pack.roll_rate_GET() == -2.3127926E38F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.time_usec_SET(8611309491952932800L) ;
        p146.x_pos_SET(-1.9098785E38F) ;
        p146.pos_variance_SET(new float[] {2.6104248E38F, -1.4037593E38F, 1.1297006E38F}, 0) ;
        p146.z_acc_SET(3.0595747E38F) ;
        p146.y_acc_SET(5.896498E37F) ;
        p146.z_pos_SET(-2.0739847E38F) ;
        p146.vel_variance_SET(new float[] {-2.9769326E38F, -1.5340616E38F, -8.29746E37F}, 0) ;
        p146.x_acc_SET(-2.6897966E38F) ;
        p146.airspeed_SET(-1.5793087E38F) ;
        p146.pitch_rate_SET(3.2598992E38F) ;
        p146.yaw_rate_SET(-3.4614926E37F) ;
        p146.roll_rate_SET(-2.3127926E38F) ;
        p146.x_vel_SET(3.8044332E37F) ;
        p146.z_vel_SET(1.8738732E38F) ;
        p146.y_vel_SET(-1.0177422E38F) ;
        p146.q_SET(new float[] {-2.6046265E38F, 3.3803493E38F, 3.3764967E38F, 1.0655549E38F}, 0) ;
        p146.y_pos_SET(-2.8884934E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
            assert(pack.battery_remaining_GET() == (byte) - 12);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
            assert(pack.current_consumed_GET() == -262607537);
            assert(pack.id_GET() == (char)18);
            assert(pack.energy_consumed_GET() == -421657989);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)47939, (char)28550, (char)14606, (char)60521, (char)56432, (char)476, (char)5944, (char)13956, (char)17686, (char)18302}));
            assert(pack.current_battery_GET() == (short) -3915);
            assert(pack.temperature_GET() == (short)3689);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)18) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL) ;
        p147.current_battery_SET((short) -3915) ;
        p147.temperature_SET((short)3689) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION) ;
        p147.voltages_SET(new char[] {(char)47939, (char)28550, (char)14606, (char)60521, (char)56432, (char)476, (char)5944, (char)13956, (char)17686, (char)18302}, 0) ;
        p147.battery_remaining_SET((byte) - 12) ;
        p147.energy_consumed_SET(-421657989) ;
        p147.current_consumed_SET(-262607537) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)113, (char)140, (char)170, (char)10, (char)19, (char)10, (char)23, (char)243, (char)87, (char)80, (char)118, (char)168, (char)10, (char)58, (char)84, (char)11, (char)148, (char)2}));
            assert(pack.vendor_id_GET() == (char)48895);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)235, (char)229, (char)1, (char)140, (char)149, (char)30, (char)251, (char)67}));
            assert(pack.board_version_GET() == 222724184L);
            assert(pack.product_id_GET() == (char)33248);
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
            assert(pack.middleware_sw_version_GET() == 1207659968L);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)177, (char)177, (char)65, (char)86, (char)76, (char)123, (char)61, (char)227}));
            assert(pack.flight_sw_version_GET() == 4139446251L);
            assert(pack.uid_GET() == 6572097904081709399L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)169, (char)138, (char)192, (char)191, (char)114, (char)34, (char)13, (char)202}));
            assert(pack.os_sw_version_GET() == 3994929879L);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.product_id_SET((char)33248) ;
        p148.os_custom_version_SET(new char[] {(char)235, (char)229, (char)1, (char)140, (char)149, (char)30, (char)251, (char)67}, 0) ;
        p148.os_sw_version_SET(3994929879L) ;
        p148.flight_sw_version_SET(4139446251L) ;
        p148.vendor_id_SET((char)48895) ;
        p148.middleware_custom_version_SET(new char[] {(char)177, (char)177, (char)65, (char)86, (char)76, (char)123, (char)61, (char)227}, 0) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE)) ;
        p148.uid2_SET(new char[] {(char)113, (char)140, (char)170, (char)10, (char)19, (char)10, (char)23, (char)243, (char)87, (char)80, (char)118, (char)168, (char)10, (char)58, (char)84, (char)11, (char)148, (char)2}, 0, PH) ;
        p148.uid_SET(6572097904081709399L) ;
        p148.middleware_sw_version_SET(1207659968L) ;
        p148.board_version_SET(222724184L) ;
        p148.flight_custom_version_SET(new char[] {(char)169, (char)138, (char)192, (char)191, (char)114, (char)34, (char)13, (char)202}, 0) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.angle_y_GET() == -1.8451109E38F);
            assert(pack.target_num_GET() == (char)216);
            assert(pack.time_usec_GET() == 8868637803502835334L);
            assert(pack.size_y_GET() == 1.893319E38F);
            assert(pack.size_x_GET() == 9.88771E37F);
            assert(pack.y_TRY(ph) == 1.0841911E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.x_TRY(ph) == 1.2016716E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
            assert(pack.position_valid_TRY(ph) == (char)118);
            assert(pack.angle_x_GET() == 1.7766225E38F);
            assert(pack.z_TRY(ph) == 2.600592E38F);
            assert(pack.distance_GET() == -5.3953395E37F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {-1.7411049E37F, -1.7107874E37F, -2.0591868E38F, -6.8382387E37F}));
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.distance_SET(-5.3953395E37F) ;
        p149.time_usec_SET(8868637803502835334L) ;
        p149.x_SET(1.2016716E38F, PH) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p149.position_valid_SET((char)118, PH) ;
        p149.size_x_SET(9.88771E37F) ;
        p149.size_y_SET(1.893319E38F) ;
        p149.angle_y_SET(-1.8451109E38F) ;
        p149.q_SET(new float[] {-1.7411049E37F, -1.7107874E37F, -2.0591868E38F, -6.8382387E37F}, 0, PH) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON) ;
        p149.z_SET(2.600592E38F, PH) ;
        p149.y_SET(1.0841911E38F, PH) ;
        p149.target_num_SET((char)216) ;
        p149.angle_x_SET(1.7766225E38F) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS));
            assert(pack.time_usec_GET() == 5034194418444577570L);
            assert(pack.vel_ratio_GET() == -3.0553945E38F);
            assert(pack.pos_vert_accuracy_GET() == -3.2062297E38F);
            assert(pack.pos_vert_ratio_GET() == 2.0233282E38F);
            assert(pack.mag_ratio_GET() == -3.0565891E38F);
            assert(pack.hagl_ratio_GET() == 1.085968E38F);
            assert(pack.tas_ratio_GET() == -2.5370317E38F);
            assert(pack.pos_horiz_accuracy_GET() == -2.5061645E37F);
            assert(pack.pos_horiz_ratio_GET() == -1.2805685E38F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.mag_ratio_SET(-3.0565891E38F) ;
        p230.pos_horiz_accuracy_SET(-2.5061645E37F) ;
        p230.pos_horiz_ratio_SET(-1.2805685E38F) ;
        p230.hagl_ratio_SET(1.085968E38F) ;
        p230.tas_ratio_SET(-2.5370317E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS)) ;
        p230.time_usec_SET(5034194418444577570L) ;
        p230.pos_vert_ratio_SET(2.0233282E38F) ;
        p230.pos_vert_accuracy_SET(-3.2062297E38F) ;
        p230.vel_ratio_SET(-3.0553945E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_alt_GET() == -3.4744427E37F);
            assert(pack.vert_accuracy_GET() == 1.9362122E37F);
            assert(pack.time_usec_GET() == 8841307127819389228L);
            assert(pack.horiz_accuracy_GET() == -1.0775093E37F);
            assert(pack.var_vert_GET() == -2.0633942E38F);
            assert(pack.wind_z_GET() == -2.1616627E38F);
            assert(pack.wind_y_GET() == 8.2158697E37F);
            assert(pack.wind_x_GET() == 5.3219527E37F);
            assert(pack.var_horiz_GET() == -2.5002737E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.horiz_accuracy_SET(-1.0775093E37F) ;
        p231.var_horiz_SET(-2.5002737E38F) ;
        p231.wind_x_SET(5.3219527E37F) ;
        p231.wind_y_SET(8.2158697E37F) ;
        p231.vert_accuracy_SET(1.9362122E37F) ;
        p231.wind_alt_SET(-3.4744427E37F) ;
        p231.wind_z_SET(-2.1616627E38F) ;
        p231.time_usec_SET(8841307127819389228L) ;
        p231.var_vert_SET(-2.0633942E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -38511076);
            assert(pack.speed_accuracy_GET() == -1.1830837E38F);
            assert(pack.gps_id_GET() == (char)5);
            assert(pack.vd_GET() == -2.622158E38F);
            assert(pack.vert_accuracy_GET() == 1.7519337E38F);
            assert(pack.satellites_visible_GET() == (char)119);
            assert(pack.vn_GET() == 1.2754849E38F);
            assert(pack.fix_type_GET() == (char)150);
            assert(pack.lat_GET() == -602849118);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP));
            assert(pack.vdop_GET() == 2.4833389E38F);
            assert(pack.time_usec_GET() == 6200648779330191456L);
            assert(pack.alt_GET() == -1.5698105E38F);
            assert(pack.ve_GET() == 1.540816E38F);
            assert(pack.hdop_GET() == 7.36112E37F);
            assert(pack.horiz_accuracy_GET() == 3.6583022E37F);
            assert(pack.time_week_GET() == (char)21783);
            assert(pack.time_week_ms_GET() == 1024841341L);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.lat_SET(-602849118) ;
        p232.ve_SET(1.540816E38F) ;
        p232.time_week_SET((char)21783) ;
        p232.lon_SET(-38511076) ;
        p232.gps_id_SET((char)5) ;
        p232.hdop_SET(7.36112E37F) ;
        p232.satellites_visible_SET((char)119) ;
        p232.alt_SET(-1.5698105E38F) ;
        p232.speed_accuracy_SET(-1.1830837E38F) ;
        p232.vn_SET(1.2754849E38F) ;
        p232.horiz_accuracy_SET(3.6583022E37F) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP)) ;
        p232.fix_type_SET((char)150) ;
        p232.vert_accuracy_SET(1.7519337E38F) ;
        p232.time_usec_SET(6200648779330191456L) ;
        p232.vd_SET(-2.622158E38F) ;
        p232.time_week_ms_SET(1024841341L) ;
        p232.vdop_SET(2.4833389E38F) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)118);
            assert(pack.flags_GET() == (char)182);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)0, (char)119, (char)136, (char)16, (char)145, (char)142, (char)45, (char)151, (char)8, (char)38, (char)131, (char)152, (char)116, (char)144, (char)229, (char)12, (char)237, (char)164, (char)142, (char)20, (char)109, (char)207, (char)74, (char)59, (char)223, (char)251, (char)200, (char)44, (char)80, (char)59, (char)190, (char)58, (char)182, (char)239, (char)132, (char)118, (char)71, (char)159, (char)64, (char)47, (char)36, (char)198, (char)80, (char)79, (char)233, (char)2, (char)31, (char)71, (char)23, (char)22, (char)142, (char)244, (char)248, (char)248, (char)191, (char)202, (char)68, (char)201, (char)239, (char)74, (char)177, (char)237, (char)54, (char)125, (char)199, (char)89, (char)223, (char)198, (char)185, (char)38, (char)146, (char)246, (char)43, (char)111, (char)227, (char)186, (char)106, (char)69, (char)215, (char)76, (char)237, (char)99, (char)152, (char)224, (char)156, (char)172, (char)168, (char)95, (char)81, (char)112, (char)190, (char)254, (char)175, (char)84, (char)244, (char)180, (char)26, (char)208, (char)69, (char)130, (char)59, (char)150, (char)142, (char)187, (char)179, (char)173, (char)6, (char)67, (char)100, (char)28, (char)122, (char)91, (char)81, (char)31, (char)1, (char)145, (char)126, (char)77, (char)87, (char)8, (char)179, (char)41, (char)73, (char)205, (char)225, (char)248, (char)228, (char)196, (char)6, (char)196, (char)228, (char)153, (char)76, (char)198, (char)163, (char)81, (char)106, (char)25, (char)192, (char)112, (char)142, (char)234, (char)75, (char)51, (char)125, (char)252, (char)222, (char)242, (char)22, (char)62, (char)14, (char)160, (char)108, (char)94, (char)151, (char)55, (char)76, (char)211, (char)161, (char)85, (char)99, (char)141, (char)158, (char)71, (char)63, (char)99, (char)95, (char)74, (char)73, (char)40, (char)78, (char)78, (char)17, (char)204, (char)56, (char)39, (char)45, (char)174, (char)215, (char)196}));
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.len_SET((char)118) ;
        p233.data__SET(new char[] {(char)0, (char)119, (char)136, (char)16, (char)145, (char)142, (char)45, (char)151, (char)8, (char)38, (char)131, (char)152, (char)116, (char)144, (char)229, (char)12, (char)237, (char)164, (char)142, (char)20, (char)109, (char)207, (char)74, (char)59, (char)223, (char)251, (char)200, (char)44, (char)80, (char)59, (char)190, (char)58, (char)182, (char)239, (char)132, (char)118, (char)71, (char)159, (char)64, (char)47, (char)36, (char)198, (char)80, (char)79, (char)233, (char)2, (char)31, (char)71, (char)23, (char)22, (char)142, (char)244, (char)248, (char)248, (char)191, (char)202, (char)68, (char)201, (char)239, (char)74, (char)177, (char)237, (char)54, (char)125, (char)199, (char)89, (char)223, (char)198, (char)185, (char)38, (char)146, (char)246, (char)43, (char)111, (char)227, (char)186, (char)106, (char)69, (char)215, (char)76, (char)237, (char)99, (char)152, (char)224, (char)156, (char)172, (char)168, (char)95, (char)81, (char)112, (char)190, (char)254, (char)175, (char)84, (char)244, (char)180, (char)26, (char)208, (char)69, (char)130, (char)59, (char)150, (char)142, (char)187, (char)179, (char)173, (char)6, (char)67, (char)100, (char)28, (char)122, (char)91, (char)81, (char)31, (char)1, (char)145, (char)126, (char)77, (char)87, (char)8, (char)179, (char)41, (char)73, (char)205, (char)225, (char)248, (char)228, (char)196, (char)6, (char)196, (char)228, (char)153, (char)76, (char)198, (char)163, (char)81, (char)106, (char)25, (char)192, (char)112, (char)142, (char)234, (char)75, (char)51, (char)125, (char)252, (char)222, (char)242, (char)22, (char)62, (char)14, (char)160, (char)108, (char)94, (char)151, (char)55, (char)76, (char)211, (char)161, (char)85, (char)99, (char)141, (char)158, (char)71, (char)63, (char)99, (char)95, (char)74, (char)73, (char)40, (char)78, (char)78, (char)17, (char)204, (char)56, (char)39, (char)45, (char)174, (char)215, (char)196}, 0) ;
        p233.flags_SET((char)182) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            assert(pack.altitude_sp_GET() == (short) -20608);
            assert(pack.groundspeed_GET() == (char)114);
            assert(pack.wp_num_GET() == (char)235);
            assert(pack.temperature_air_GET() == (byte) - 49);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
            assert(pack.heading_sp_GET() == (short) -20503);
            assert(pack.heading_GET() == (char)51646);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.airspeed_GET() == (char)145);
            assert(pack.gps_nsat_GET() == (char)37);
            assert(pack.temperature_GET() == (byte)21);
            assert(pack.custom_mode_GET() == 3317291514L);
            assert(pack.pitch_GET() == (short) -11653);
            assert(pack.wp_distance_GET() == (char)59221);
            assert(pack.roll_GET() == (short)24522);
            assert(pack.longitude_GET() == -741736821);
            assert(pack.latitude_GET() == -583003162);
            assert(pack.airspeed_sp_GET() == (char)46);
            assert(pack.altitude_amsl_GET() == (short)13226);
            assert(pack.failsafe_GET() == (char)232);
            assert(pack.battery_remaining_GET() == (char)166);
            assert(pack.climb_rate_GET() == (byte)4);
            assert(pack.throttle_GET() == (byte) - 18);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.throttle_SET((byte) - 18) ;
        p234.airspeed_sp_SET((char)46) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED)) ;
        p234.heading_sp_SET((short) -20503) ;
        p234.wp_num_SET((char)235) ;
        p234.climb_rate_SET((byte)4) ;
        p234.roll_SET((short)24522) ;
        p234.heading_SET((char)51646) ;
        p234.longitude_SET(-741736821) ;
        p234.airspeed_SET((char)145) ;
        p234.altitude_sp_SET((short) -20608) ;
        p234.failsafe_SET((char)232) ;
        p234.altitude_amsl_SET((short)13226) ;
        p234.temperature_SET((byte)21) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND) ;
        p234.battery_remaining_SET((char)166) ;
        p234.gps_nsat_SET((char)37) ;
        p234.temperature_air_SET((byte) - 49) ;
        p234.custom_mode_SET(3317291514L) ;
        p234.pitch_SET((short) -11653) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p234.wp_distance_SET((char)59221) ;
        p234.groundspeed_SET((char)114) ;
        p234.latitude_SET(-583003162) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_z_GET() == 8.590307E37F);
            assert(pack.vibration_y_GET() == 8.4668564E37F);
            assert(pack.vibration_x_GET() == -2.6578218E38F);
            assert(pack.clipping_0_GET() == 4206081138L);
            assert(pack.clipping_1_GET() == 3492500606L);
            assert(pack.clipping_2_GET() == 1728793733L);
            assert(pack.time_usec_GET() == 3339034759990746029L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_1_SET(3492500606L) ;
        p241.time_usec_SET(3339034759990746029L) ;
        p241.vibration_x_SET(-2.6578218E38F) ;
        p241.clipping_2_SET(1728793733L) ;
        p241.vibration_z_SET(8.590307E37F) ;
        p241.clipping_0_SET(4206081138L) ;
        p241.vibration_y_SET(8.4668564E37F) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -2.0437757E38F);
            assert(pack.latitude_GET() == -389095817);
            assert(pack.approach_z_GET() == -3.1751883E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {9.154541E37F, -1.7792877E37F, -1.6736959E38F, -7.091107E37F}));
            assert(pack.altitude_GET() == -2114122798);
            assert(pack.longitude_GET() == 1719906160);
            assert(pack.approach_x_GET() == -1.6347505E38F);
            assert(pack.y_GET() == -3.2826638E38F);
            assert(pack.time_usec_TRY(ph) == 8382815989127989632L);
            assert(pack.approach_y_GET() == -1.6356892E37F);
            assert(pack.z_GET() == -2.3239818E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.time_usec_SET(8382815989127989632L, PH) ;
        p242.approach_y_SET(-1.6356892E37F) ;
        p242.altitude_SET(-2114122798) ;
        p242.approach_x_SET(-1.6347505E38F) ;
        p242.q_SET(new float[] {9.154541E37F, -1.7792877E37F, -1.6736959E38F, -7.091107E37F}, 0) ;
        p242.longitude_SET(1719906160) ;
        p242.z_SET(-2.3239818E38F) ;
        p242.latitude_SET(-389095817) ;
        p242.y_SET(-3.2826638E38F) ;
        p242.approach_z_SET(-3.1751883E38F) ;
        p242.x_SET(-2.0437757E38F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.2096155E37F, 2.957871E38F, -9.47356E36F, -3.0286075E38F}));
            assert(pack.y_GET() == -4.64716E37F);
            assert(pack.z_GET() == 2.3116454E38F);
            assert(pack.longitude_GET() == 1414905106);
            assert(pack.approach_z_GET() == 2.7841363E38F);
            assert(pack.latitude_GET() == -652197695);
            assert(pack.approach_y_GET() == 2.9857279E38F);
            assert(pack.approach_x_GET() == 1.0372714E38F);
            assert(pack.target_system_GET() == (char)245);
            assert(pack.x_GET() == -7.760448E37F);
            assert(pack.time_usec_TRY(ph) == 3350746376344939518L);
            assert(pack.altitude_GET() == 193907917);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.approach_x_SET(1.0372714E38F) ;
        p243.q_SET(new float[] {-3.2096155E37F, 2.957871E38F, -9.47356E36F, -3.0286075E38F}, 0) ;
        p243.approach_z_SET(2.7841363E38F) ;
        p243.approach_y_SET(2.9857279E38F) ;
        p243.target_system_SET((char)245) ;
        p243.altitude_SET(193907917) ;
        p243.y_SET(-4.64716E37F) ;
        p243.time_usec_SET(3350746376344939518L, PH) ;
        p243.latitude_SET(-652197695) ;
        p243.z_SET(2.3116454E38F) ;
        p243.longitude_SET(1414905106) ;
        p243.x_SET(-7.760448E37F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == 2019000769);
            assert(pack.message_id_GET() == (char)21659);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)21659) ;
        p244.interval_us_SET(2019000769) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED) ;
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SPACE);
            assert(pack.heading_GET() == (char)27622);
            assert(pack.altitude_GET() == -1823941162);
            assert(pack.ICAO_address_GET() == 710081008L);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN));
            assert(pack.hor_velocity_GET() == (char)31909);
            assert(pack.squawk_GET() == (char)13472);
            assert(pack.lon_GET() == -473780211);
            assert(pack.callsign_LEN(ph) == 6);
            assert(pack.callsign_TRY(ph).equals("ktCqsM"));
            assert(pack.lat_GET() == 1089084269);
            assert(pack.tslc_GET() == (char)200);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.ver_velocity_GET() == (short) -5950);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.heading_SET((char)27622) ;
        p246.callsign_SET("ktCqsM", PH) ;
        p246.lat_SET(1089084269) ;
        p246.squawk_SET((char)13472) ;
        p246.ICAO_address_SET(710081008L) ;
        p246.hor_velocity_SET((char)31909) ;
        p246.lon_SET(-473780211) ;
        p246.altitude_SET(-1823941162) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.tslc_SET((char)200) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN)) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SPACE) ;
        p246.ver_velocity_SET((short) -5950) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.horizontal_minimum_delta_GET() == 6.5285684E37F);
            assert(pack.time_to_minimum_delta_GET() == -2.4600829E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.altitude_minimum_delta_GET() == 3.3639498E38F);
            assert(pack.id_GET() == 1524925716L);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.time_to_minimum_delta_SET(-2.4600829E38F) ;
        p247.horizontal_minimum_delta_SET(6.5285684E37F) ;
        p247.id_SET(1524925716L) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW) ;
        p247.altitude_minimum_delta_SET(3.3639498E38F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)104);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)201, (char)235, (char)62, (char)56, (char)180, (char)133, (char)34, (char)247, (char)23, (char)151, (char)161, (char)203, (char)14, (char)80, (char)230, (char)100, (char)24, (char)146, (char)136, (char)133, (char)59, (char)3, (char)189, (char)245, (char)240, (char)241, (char)205, (char)190, (char)220, (char)189, (char)69, (char)236, (char)244, (char)108, (char)69, (char)48, (char)197, (char)162, (char)235, (char)213, (char)171, (char)50, (char)80, (char)118, (char)189, (char)84, (char)245, (char)217, (char)240, (char)74, (char)194, (char)148, (char)51, (char)167, (char)236, (char)218, (char)104, (char)96, (char)235, (char)161, (char)186, (char)71, (char)145, (char)67, (char)66, (char)123, (char)177, (char)56, (char)75, (char)234, (char)227, (char)131, (char)198, (char)178, (char)245, (char)201, (char)53, (char)159, (char)150, (char)227, (char)82, (char)108, (char)118, (char)244, (char)105, (char)230, (char)150, (char)219, (char)207, (char)4, (char)117, (char)46, (char)119, (char)172, (char)165, (char)205, (char)169, (char)145, (char)212, (char)16, (char)27, (char)55, (char)221, (char)74, (char)208, (char)135, (char)90, (char)100, (char)164, (char)83, (char)187, (char)220, (char)18, (char)112, (char)207, (char)194, (char)20, (char)88, (char)75, (char)206, (char)236, (char)95, (char)66, (char)149, (char)149, (char)32, (char)99, (char)249, (char)217, (char)167, (char)212, (char)58, (char)150, (char)149, (char)183, (char)41, (char)186, (char)3, (char)67, (char)245, (char)33, (char)231, (char)235, (char)216, (char)46, (char)171, (char)45, (char)216, (char)25, (char)86, (char)176, (char)175, (char)216, (char)159, (char)10, (char)37, (char)33, (char)141, (char)43, (char)164, (char)123, (char)113, (char)61, (char)229, (char)205, (char)61, (char)145, (char)134, (char)203, (char)101, (char)245, (char)247, (char)145, (char)29, (char)122, (char)167, (char)174, (char)122, (char)173, (char)91, (char)169, (char)44, (char)215, (char)245, (char)139, (char)100, (char)17, (char)252, (char)117, (char)176, (char)134, (char)67, (char)207, (char)52, (char)12, (char)221, (char)79, (char)17, (char)73, (char)24, (char)179, (char)242, (char)181, (char)71, (char)155, (char)122, (char)246, (char)9, (char)236, (char)74, (char)236, (char)215, (char)175, (char)25, (char)71, (char)45, (char)238, (char)238, (char)9, (char)71, (char)29, (char)165, (char)131, (char)245, (char)170, (char)72, (char)200, (char)134, (char)57, (char)78, (char)60, (char)192, (char)136, (char)0, (char)239, (char)53, (char)253, (char)65, (char)59, (char)161, (char)253, (char)133, (char)225, (char)2, (char)249, (char)40, (char)35, (char)15, (char)46}));
            assert(pack.target_network_GET() == (char)3);
            assert(pack.message_type_GET() == (char)12957);
            assert(pack.target_system_GET() == (char)117);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_component_SET((char)104) ;
        p248.payload_SET(new char[] {(char)201, (char)235, (char)62, (char)56, (char)180, (char)133, (char)34, (char)247, (char)23, (char)151, (char)161, (char)203, (char)14, (char)80, (char)230, (char)100, (char)24, (char)146, (char)136, (char)133, (char)59, (char)3, (char)189, (char)245, (char)240, (char)241, (char)205, (char)190, (char)220, (char)189, (char)69, (char)236, (char)244, (char)108, (char)69, (char)48, (char)197, (char)162, (char)235, (char)213, (char)171, (char)50, (char)80, (char)118, (char)189, (char)84, (char)245, (char)217, (char)240, (char)74, (char)194, (char)148, (char)51, (char)167, (char)236, (char)218, (char)104, (char)96, (char)235, (char)161, (char)186, (char)71, (char)145, (char)67, (char)66, (char)123, (char)177, (char)56, (char)75, (char)234, (char)227, (char)131, (char)198, (char)178, (char)245, (char)201, (char)53, (char)159, (char)150, (char)227, (char)82, (char)108, (char)118, (char)244, (char)105, (char)230, (char)150, (char)219, (char)207, (char)4, (char)117, (char)46, (char)119, (char)172, (char)165, (char)205, (char)169, (char)145, (char)212, (char)16, (char)27, (char)55, (char)221, (char)74, (char)208, (char)135, (char)90, (char)100, (char)164, (char)83, (char)187, (char)220, (char)18, (char)112, (char)207, (char)194, (char)20, (char)88, (char)75, (char)206, (char)236, (char)95, (char)66, (char)149, (char)149, (char)32, (char)99, (char)249, (char)217, (char)167, (char)212, (char)58, (char)150, (char)149, (char)183, (char)41, (char)186, (char)3, (char)67, (char)245, (char)33, (char)231, (char)235, (char)216, (char)46, (char)171, (char)45, (char)216, (char)25, (char)86, (char)176, (char)175, (char)216, (char)159, (char)10, (char)37, (char)33, (char)141, (char)43, (char)164, (char)123, (char)113, (char)61, (char)229, (char)205, (char)61, (char)145, (char)134, (char)203, (char)101, (char)245, (char)247, (char)145, (char)29, (char)122, (char)167, (char)174, (char)122, (char)173, (char)91, (char)169, (char)44, (char)215, (char)245, (char)139, (char)100, (char)17, (char)252, (char)117, (char)176, (char)134, (char)67, (char)207, (char)52, (char)12, (char)221, (char)79, (char)17, (char)73, (char)24, (char)179, (char)242, (char)181, (char)71, (char)155, (char)122, (char)246, (char)9, (char)236, (char)74, (char)236, (char)215, (char)175, (char)25, (char)71, (char)45, (char)238, (char)238, (char)9, (char)71, (char)29, (char)165, (char)131, (char)245, (char)170, (char)72, (char)200, (char)134, (char)57, (char)78, (char)60, (char)192, (char)136, (char)0, (char)239, (char)53, (char)253, (char)65, (char)59, (char)161, (char)253, (char)133, (char)225, (char)2, (char)249, (char)40, (char)35, (char)15, (char)46}, 0) ;
        p248.target_system_SET((char)117) ;
        p248.message_type_SET((char)12957) ;
        p248.target_network_SET((char)3) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 65, (byte) - 75, (byte) - 33, (byte) - 51, (byte) - 41, (byte)107, (byte) - 6, (byte)125, (byte)88, (byte)92, (byte)125, (byte)125, (byte)127, (byte)12, (byte) - 99, (byte)109, (byte)26, (byte) - 89, (byte) - 1, (byte) - 91, (byte)97, (byte) - 32, (byte) - 11, (byte)39, (byte)36, (byte) - 27, (byte)0, (byte) - 72, (byte) - 123, (byte) - 108, (byte)69, (byte) - 96}));
            assert(pack.ver_GET() == (char)170);
            assert(pack.address_GET() == (char)3715);
            assert(pack.type_GET() == (char)229);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.type_SET((char)229) ;
        p249.ver_SET((char)170) ;
        p249.value_SET(new byte[] {(byte) - 65, (byte) - 75, (byte) - 33, (byte) - 51, (byte) - 41, (byte)107, (byte) - 6, (byte)125, (byte)88, (byte)92, (byte)125, (byte)125, (byte)127, (byte)12, (byte) - 99, (byte)109, (byte)26, (byte) - 89, (byte) - 1, (byte) - 91, (byte)97, (byte) - 32, (byte) - 11, (byte)39, (byte)36, (byte) - 27, (byte)0, (byte) - 72, (byte) - 123, (byte) - 108, (byte)69, (byte) - 96}, 0) ;
        p249.address_SET((char)3715) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 2);
            assert(pack.name_TRY(ph).equals("hp"));
            assert(pack.x_GET() == -2.0095901E37F);
            assert(pack.y_GET() == -2.583919E38F);
            assert(pack.z_GET() == -1.4891067E38F);
            assert(pack.time_usec_GET() == 6864475369185731126L);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.z_SET(-1.4891067E38F) ;
        p250.name_SET("hp", PH) ;
        p250.x_SET(-2.0095901E37F) ;
        p250.y_SET(-2.583919E38F) ;
        p250.time_usec_SET(6864475369185731126L) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -2.7653786E38F);
            assert(pack.time_boot_ms_GET() == 1629860507L);
            assert(pack.name_LEN(ph) == 8);
            assert(pack.name_TRY(ph).equals("iVctsxaB"));
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(-2.7653786E38F) ;
        p251.name_SET("iVctsxaB", PH) ;
        p251.time_boot_ms_SET(1629860507L) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1619463718L);
            assert(pack.name_LEN(ph) == 8);
            assert(pack.name_TRY(ph).equals("qvgcnket"));
            assert(pack.value_GET() == 181102615);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(1619463718L) ;
        p252.name_SET("qvgcnket", PH) ;
        p252.value_SET(181102615) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 8);
            assert(pack.text_TRY(ph).equals("fcdbptiT"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_ERROR);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("fcdbptiT", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_ERROR) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 785050334L);
            assert(pack.ind_GET() == (char)20);
            assert(pack.value_GET() == -8.4390524E36F);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(785050334L) ;
        p254.value_SET(-8.4390524E36F) ;
        p254.ind_SET((char)20) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.initial_timestamp_GET() == 8201856904584487065L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)180, (char)45, (char)78, (char)116, (char)149, (char)50, (char)84, (char)135, (char)98, (char)176, (char)98, (char)136, (char)124, (char)133, (char)133, (char)134, (char)3, (char)65, (char)11, (char)190, (char)69, (char)246, (char)139, (char)196, (char)0, (char)173, (char)214, (char)37, (char)254, (char)97, (char)65, (char)34}));
            assert(pack.target_system_GET() == (char)23);
            assert(pack.target_component_GET() == (char)87);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.initial_timestamp_SET(8201856904584487065L) ;
        p256.secret_key_SET(new char[] {(char)180, (char)45, (char)78, (char)116, (char)149, (char)50, (char)84, (char)135, (char)98, (char)176, (char)98, (char)136, (char)124, (char)133, (char)133, (char)134, (char)3, (char)65, (char)11, (char)190, (char)69, (char)246, (char)139, (char)196, (char)0, (char)173, (char)214, (char)37, (char)254, (char)97, (char)65, (char)34}, 0) ;
        p256.target_component_SET((char)87) ;
        p256.target_system_SET((char)23) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.last_change_ms_GET() == 4257698425L);
            assert(pack.state_GET() == (char)44);
            assert(pack.time_boot_ms_GET() == 4079639423L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.last_change_ms_SET(4257698425L) ;
        p257.time_boot_ms_SET(4079639423L) ;
        p257.state_SET((char)44) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.tune_LEN(ph) == 29);
            assert(pack.tune_TRY(ph).equals("gatauxgwxqzmaccqsmxOiiswfksfN"));
            assert(pack.target_system_GET() == (char)109);
            assert(pack.target_component_GET() == (char)247);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.tune_SET("gatauxgwxqzmaccqsmxOiiswfksfN", PH) ;
        p258.target_component_SET((char)247) ;
        p258.target_system_SET((char)109) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.focal_length_GET() == 5.435821E37F);
            assert(pack.cam_definition_version_GET() == (char)55254);
            assert(pack.resolution_h_GET() == (char)17368);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)154, (char)232, (char)253, (char)31, (char)236, (char)142, (char)205, (char)121, (char)169, (char)47, (char)29, (char)29, (char)182, (char)227, (char)178, (char)131, (char)51, (char)37, (char)139, (char)120, (char)168, (char)6, (char)101, (char)155, (char)180, (char)178, (char)47, (char)131, (char)100, (char)42, (char)220, (char)197}));
            assert(pack.lens_id_GET() == (char)174);
            assert(pack.time_boot_ms_GET() == 1346089911L);
            assert(pack.sensor_size_v_GET() == 2.4965442E38F);
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES));
            assert(pack.resolution_v_GET() == (char)3294);
            assert(pack.sensor_size_h_GET() == -4.9341474E37F);
            assert(pack.cam_definition_uri_LEN(ph) == 54);
            assert(pack.cam_definition_uri_TRY(ph).equals("fabxrloedknkDcRfkuXccfovtvpRfnwKkPnhvpqmmjjhhTsIsoLvec"));
            assert(pack.firmware_version_GET() == 1949818066L);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)246, (char)184, (char)29, (char)206, (char)94, (char)43, (char)89, (char)141, (char)47, (char)119, (char)209, (char)118, (char)83, (char)171, (char)53, (char)9, (char)162, (char)172, (char)69, (char)91, (char)187, (char)179, (char)29, (char)88, (char)83, (char)181, (char)50, (char)135, (char)54, (char)222, (char)205, (char)118}));
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.sensor_size_v_SET(2.4965442E38F) ;
        p259.lens_id_SET((char)174) ;
        p259.sensor_size_h_SET(-4.9341474E37F) ;
        p259.time_boot_ms_SET(1346089911L) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES)) ;
        p259.cam_definition_uri_SET("fabxrloedknkDcRfkuXccfovtvpRfnwKkPnhvpqmmjjhhTsIsoLvec", PH) ;
        p259.firmware_version_SET(1949818066L) ;
        p259.resolution_v_SET((char)3294) ;
        p259.resolution_h_SET((char)17368) ;
        p259.cam_definition_version_SET((char)55254) ;
        p259.vendor_name_SET(new char[] {(char)154, (char)232, (char)253, (char)31, (char)236, (char)142, (char)205, (char)121, (char)169, (char)47, (char)29, (char)29, (char)182, (char)227, (char)178, (char)131, (char)51, (char)37, (char)139, (char)120, (char)168, (char)6, (char)101, (char)155, (char)180, (char)178, (char)47, (char)131, (char)100, (char)42, (char)220, (char)197}, 0) ;
        p259.focal_length_SET(5.435821E37F) ;
        p259.model_name_SET(new char[] {(char)246, (char)184, (char)29, (char)206, (char)94, (char)43, (char)89, (char)141, (char)47, (char)119, (char)209, (char)118, (char)83, (char)171, (char)53, (char)9, (char)162, (char)172, (char)69, (char)91, (char)187, (char)179, (char)29, (char)88, (char)83, (char)181, (char)50, (char)135, (char)54, (char)222, (char)205, (char)118}, 0) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_VIDEO);
            assert(pack.time_boot_ms_GET() == 1466081309L);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO) ;
        p260.time_boot_ms_SET(1466081309L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2291866688L);
            assert(pack.storage_id_GET() == (char)89);
            assert(pack.write_speed_GET() == 3.328764E38F);
            assert(pack.used_capacity_GET() == 5.776746E36F);
            assert(pack.status_GET() == (char)27);
            assert(pack.read_speed_GET() == 1.5386001E37F);
            assert(pack.total_capacity_GET() == 6.592216E37F);
            assert(pack.storage_count_GET() == (char)93);
            assert(pack.available_capacity_GET() == -1.0383046E38F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.storage_id_SET((char)89) ;
        p261.status_SET((char)27) ;
        p261.read_speed_SET(1.5386001E37F) ;
        p261.total_capacity_SET(6.592216E37F) ;
        p261.available_capacity_SET(-1.0383046E38F) ;
        p261.write_speed_SET(3.328764E38F) ;
        p261.used_capacity_SET(5.776746E36F) ;
        p261.storage_count_SET((char)93) ;
        p261.time_boot_ms_SET(2291866688L) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.recording_time_ms_GET() == 135163083L);
            assert(pack.available_capacity_GET() == 1.6326258E37F);
            assert(pack.image_interval_GET() == 3.0029349E38F);
            assert(pack.video_status_GET() == (char)95);
            assert(pack.time_boot_ms_GET() == 2893164781L);
            assert(pack.image_status_GET() == (char)169);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(2893164781L) ;
        p262.recording_time_ms_SET(135163083L) ;
        p262.video_status_SET((char)95) ;
        p262.image_interval_SET(3.0029349E38F) ;
        p262.image_status_SET((char)169) ;
        p262.available_capacity_SET(1.6326258E37F) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.camera_id_GET() == (char)134);
            assert(pack.time_utc_GET() == 593586305240384182L);
            assert(pack.lat_GET() == 626267831);
            assert(pack.time_boot_ms_GET() == 2827301322L);
            assert(pack.file_url_LEN(ph) == 192);
            assert(pack.file_url_TRY(ph).equals("lkSucsvdHgcMwzgxugyaaisEnTbzsyoblmgCHmjvfjnyjhNGmeotjqwikQqvyueyJponxvceisWvzkznoefwskxdzkyKtehscsszguaasruuakhqvmqbpuzrcirrafirpuoxcgxqielkfpgbfieEthwbtqnngctqbvwqoePtogclafjvmvvcmBijbewztubZ"));
            assert(pack.lon_GET() == -1521178506);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.1154459E38F, 3.2387584E38F, -1.3249108E38F, 2.8221113E38F}));
            assert(pack.capture_result_GET() == (byte)3);
            assert(pack.relative_alt_GET() == 1896993106);
            assert(pack.image_index_GET() == 1783665368);
            assert(pack.alt_GET() == -559015807);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(2827301322L) ;
        p263.camera_id_SET((char)134) ;
        p263.file_url_SET("lkSucsvdHgcMwzgxugyaaisEnTbzsyoblmgCHmjvfjnyjhNGmeotjqwikQqvyueyJponxvceisWvzkznoefwskxdzkyKtehscsszguaasruuakhqvmqbpuzrcirrafirpuoxcgxqielkfpgbfieEthwbtqnngctqbvwqoePtogclafjvmvvcmBijbewztubZ", PH) ;
        p263.lon_SET(-1521178506) ;
        p263.capture_result_SET((byte)3) ;
        p263.relative_alt_SET(1896993106) ;
        p263.time_utc_SET(593586305240384182L) ;
        p263.q_SET(new float[] {3.1154459E38F, 3.2387584E38F, -1.3249108E38F, 2.8221113E38F}, 0) ;
        p263.alt_SET(-559015807) ;
        p263.image_index_SET(1783665368) ;
        p263.lat_SET(626267831) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.arming_time_utc_GET() == 303160805191883330L);
            assert(pack.flight_uuid_GET() == 5941057509830349092L);
            assert(pack.takeoff_time_utc_GET() == 4787846259028983074L);
            assert(pack.time_boot_ms_GET() == 3249815418L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(3249815418L) ;
        p264.arming_time_utc_SET(303160805191883330L) ;
        p264.flight_uuid_SET(5941057509830349092L) ;
        p264.takeoff_time_utc_SET(4787846259028983074L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 2.9089105E37F);
            assert(pack.time_boot_ms_GET() == 1253790686L);
            assert(pack.roll_GET() == -5.122563E37F);
            assert(pack.pitch_GET() == -2.533159E37F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.yaw_SET(2.9089105E37F) ;
        p265.pitch_SET(-2.533159E37F) ;
        p265.time_boot_ms_SET(1253790686L) ;
        p265.roll_SET(-5.122563E37F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)10288);
            assert(pack.first_message_offset_GET() == (char)148);
            assert(pack.length_GET() == (char)159);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)239, (char)59, (char)142, (char)79, (char)218, (char)249, (char)42, (char)171, (char)70, (char)94, (char)161, (char)175, (char)112, (char)35, (char)186, (char)176, (char)176, (char)118, (char)140, (char)180, (char)191, (char)195, (char)19, (char)16, (char)187, (char)111, (char)110, (char)203, (char)2, (char)142, (char)225, (char)17, (char)5, (char)26, (char)224, (char)181, (char)204, (char)181, (char)29, (char)209, (char)73, (char)36, (char)18, (char)122, (char)247, (char)175, (char)84, (char)230, (char)140, (char)148, (char)221, (char)158, (char)243, (char)240, (char)24, (char)203, (char)46, (char)254, (char)247, (char)224, (char)234, (char)54, (char)230, (char)128, (char)101, (char)74, (char)83, (char)68, (char)212, (char)229, (char)128, (char)111, (char)111, (char)194, (char)111, (char)193, (char)62, (char)63, (char)78, (char)193, (char)231, (char)138, (char)31, (char)111, (char)132, (char)3, (char)123, (char)184, (char)65, (char)124, (char)154, (char)165, (char)229, (char)73, (char)207, (char)211, (char)177, (char)138, (char)17, (char)39, (char)111, (char)40, (char)252, (char)197, (char)39, (char)132, (char)108, (char)188, (char)17, (char)61, (char)48, (char)179, (char)200, (char)204, (char)57, (char)136, (char)210, (char)162, (char)213, (char)214, (char)232, (char)250, (char)49, (char)210, (char)213, (char)88, (char)60, (char)66, (char)124, (char)158, (char)104, (char)117, (char)248, (char)50, (char)56, (char)108, (char)141, (char)155, (char)218, (char)204, (char)67, (char)86, (char)130, (char)37, (char)157, (char)159, (char)116, (char)249, (char)91, (char)7, (char)169, (char)17, (char)89, (char)33, (char)68, (char)180, (char)67, (char)202, (char)83, (char)230, (char)98, (char)171, (char)95, (char)120, (char)42, (char)12, (char)117, (char)199, (char)173, (char)164, (char)111, (char)53, (char)208, (char)119, (char)25, (char)29, (char)166, (char)175, (char)54, (char)77, (char)217, (char)217, (char)129, (char)197, (char)166, (char)118, (char)73, (char)218, (char)164, (char)39, (char)46, (char)155, (char)10, (char)229, (char)207, (char)62, (char)79, (char)90, (char)241, (char)67, (char)240, (char)101, (char)243, (char)149, (char)160, (char)199, (char)149, (char)146, (char)48, (char)244, (char)32, (char)204, (char)164, (char)26, (char)15, (char)17, (char)165, (char)191, (char)132, (char)193, (char)226, (char)73, (char)65, (char)160, (char)58, (char)87, (char)194, (char)186, (char)208, (char)204, (char)162, (char)189, (char)74, (char)137, (char)237, (char)143, (char)160, (char)46, (char)117, (char)181, (char)33, (char)211, (char)57, (char)110, (char)161, (char)112, (char)167, (char)244, (char)124}));
            assert(pack.target_component_GET() == (char)179);
            assert(pack.target_system_GET() == (char)179);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.data__SET(new char[] {(char)239, (char)59, (char)142, (char)79, (char)218, (char)249, (char)42, (char)171, (char)70, (char)94, (char)161, (char)175, (char)112, (char)35, (char)186, (char)176, (char)176, (char)118, (char)140, (char)180, (char)191, (char)195, (char)19, (char)16, (char)187, (char)111, (char)110, (char)203, (char)2, (char)142, (char)225, (char)17, (char)5, (char)26, (char)224, (char)181, (char)204, (char)181, (char)29, (char)209, (char)73, (char)36, (char)18, (char)122, (char)247, (char)175, (char)84, (char)230, (char)140, (char)148, (char)221, (char)158, (char)243, (char)240, (char)24, (char)203, (char)46, (char)254, (char)247, (char)224, (char)234, (char)54, (char)230, (char)128, (char)101, (char)74, (char)83, (char)68, (char)212, (char)229, (char)128, (char)111, (char)111, (char)194, (char)111, (char)193, (char)62, (char)63, (char)78, (char)193, (char)231, (char)138, (char)31, (char)111, (char)132, (char)3, (char)123, (char)184, (char)65, (char)124, (char)154, (char)165, (char)229, (char)73, (char)207, (char)211, (char)177, (char)138, (char)17, (char)39, (char)111, (char)40, (char)252, (char)197, (char)39, (char)132, (char)108, (char)188, (char)17, (char)61, (char)48, (char)179, (char)200, (char)204, (char)57, (char)136, (char)210, (char)162, (char)213, (char)214, (char)232, (char)250, (char)49, (char)210, (char)213, (char)88, (char)60, (char)66, (char)124, (char)158, (char)104, (char)117, (char)248, (char)50, (char)56, (char)108, (char)141, (char)155, (char)218, (char)204, (char)67, (char)86, (char)130, (char)37, (char)157, (char)159, (char)116, (char)249, (char)91, (char)7, (char)169, (char)17, (char)89, (char)33, (char)68, (char)180, (char)67, (char)202, (char)83, (char)230, (char)98, (char)171, (char)95, (char)120, (char)42, (char)12, (char)117, (char)199, (char)173, (char)164, (char)111, (char)53, (char)208, (char)119, (char)25, (char)29, (char)166, (char)175, (char)54, (char)77, (char)217, (char)217, (char)129, (char)197, (char)166, (char)118, (char)73, (char)218, (char)164, (char)39, (char)46, (char)155, (char)10, (char)229, (char)207, (char)62, (char)79, (char)90, (char)241, (char)67, (char)240, (char)101, (char)243, (char)149, (char)160, (char)199, (char)149, (char)146, (char)48, (char)244, (char)32, (char)204, (char)164, (char)26, (char)15, (char)17, (char)165, (char)191, (char)132, (char)193, (char)226, (char)73, (char)65, (char)160, (char)58, (char)87, (char)194, (char)186, (char)208, (char)204, (char)162, (char)189, (char)74, (char)137, (char)237, (char)143, (char)160, (char)46, (char)117, (char)181, (char)33, (char)211, (char)57, (char)110, (char)161, (char)112, (char)167, (char)244, (char)124}, 0) ;
        p266.sequence_SET((char)10288) ;
        p266.length_SET((char)159) ;
        p266.target_system_SET((char)179) ;
        p266.target_component_SET((char)179) ;
        p266.first_message_offset_SET((char)148) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.length_GET() == (char)52);
            assert(pack.first_message_offset_GET() == (char)68);
            assert(pack.target_system_GET() == (char)234);
            assert(pack.sequence_GET() == (char)21212);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)243, (char)38, (char)216, (char)137, (char)252, (char)184, (char)248, (char)106, (char)229, (char)101, (char)100, (char)77, (char)60, (char)21, (char)27, (char)36, (char)118, (char)186, (char)181, (char)197, (char)2, (char)144, (char)244, (char)79, (char)198, (char)16, (char)247, (char)160, (char)224, (char)132, (char)6, (char)186, (char)201, (char)128, (char)203, (char)212, (char)10, (char)179, (char)11, (char)132, (char)234, (char)240, (char)8, (char)71, (char)190, (char)192, (char)217, (char)197, (char)192, (char)220, (char)133, (char)34, (char)179, (char)166, (char)174, (char)179, (char)197, (char)168, (char)227, (char)209, (char)135, (char)244, (char)250, (char)214, (char)51, (char)31, (char)97, (char)195, (char)33, (char)141, (char)116, (char)85, (char)47, (char)165, (char)52, (char)60, (char)132, (char)23, (char)11, (char)213, (char)70, (char)234, (char)71, (char)197, (char)84, (char)56, (char)222, (char)163, (char)7, (char)127, (char)250, (char)15, (char)78, (char)58, (char)159, (char)85, (char)102, (char)33, (char)230, (char)88, (char)216, (char)150, (char)53, (char)109, (char)89, (char)59, (char)188, (char)147, (char)233, (char)98, (char)111, (char)102, (char)121, (char)226, (char)98, (char)53, (char)156, (char)145, (char)206, (char)171, (char)95, (char)93, (char)181, (char)206, (char)187, (char)81, (char)138, (char)209, (char)150, (char)245, (char)6, (char)21, (char)45, (char)171, (char)20, (char)5, (char)67, (char)134, (char)18, (char)143, (char)12, (char)171, (char)43, (char)80, (char)248, (char)58, (char)74, (char)5, (char)136, (char)11, (char)113, (char)134, (char)18, (char)80, (char)242, (char)52, (char)92, (char)121, (char)205, (char)91, (char)20, (char)97, (char)7, (char)29, (char)208, (char)149, (char)185, (char)182, (char)248, (char)218, (char)17, (char)189, (char)215, (char)124, (char)255, (char)95, (char)24, (char)252, (char)169, (char)248, (char)171, (char)129, (char)234, (char)29, (char)200, (char)84, (char)129, (char)59, (char)93, (char)254, (char)228, (char)224, (char)147, (char)74, (char)34, (char)3, (char)103, (char)235, (char)16, (char)242, (char)98, (char)65, (char)194, (char)3, (char)161, (char)64, (char)121, (char)106, (char)2, (char)155, (char)154, (char)174, (char)179, (char)125, (char)167, (char)142, (char)229, (char)119, (char)45, (char)101, (char)115, (char)127, (char)169, (char)208, (char)204, (char)238, (char)41, (char)189, (char)141, (char)204, (char)109, (char)252, (char)33, (char)213, (char)165, (char)67, (char)9, (char)136, (char)223, (char)89, (char)68, (char)182, (char)70, (char)126, (char)167, (char)249, (char)110, (char)20, (char)49}));
            assert(pack.target_component_GET() == (char)138);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_component_SET((char)138) ;
        p267.data__SET(new char[] {(char)243, (char)38, (char)216, (char)137, (char)252, (char)184, (char)248, (char)106, (char)229, (char)101, (char)100, (char)77, (char)60, (char)21, (char)27, (char)36, (char)118, (char)186, (char)181, (char)197, (char)2, (char)144, (char)244, (char)79, (char)198, (char)16, (char)247, (char)160, (char)224, (char)132, (char)6, (char)186, (char)201, (char)128, (char)203, (char)212, (char)10, (char)179, (char)11, (char)132, (char)234, (char)240, (char)8, (char)71, (char)190, (char)192, (char)217, (char)197, (char)192, (char)220, (char)133, (char)34, (char)179, (char)166, (char)174, (char)179, (char)197, (char)168, (char)227, (char)209, (char)135, (char)244, (char)250, (char)214, (char)51, (char)31, (char)97, (char)195, (char)33, (char)141, (char)116, (char)85, (char)47, (char)165, (char)52, (char)60, (char)132, (char)23, (char)11, (char)213, (char)70, (char)234, (char)71, (char)197, (char)84, (char)56, (char)222, (char)163, (char)7, (char)127, (char)250, (char)15, (char)78, (char)58, (char)159, (char)85, (char)102, (char)33, (char)230, (char)88, (char)216, (char)150, (char)53, (char)109, (char)89, (char)59, (char)188, (char)147, (char)233, (char)98, (char)111, (char)102, (char)121, (char)226, (char)98, (char)53, (char)156, (char)145, (char)206, (char)171, (char)95, (char)93, (char)181, (char)206, (char)187, (char)81, (char)138, (char)209, (char)150, (char)245, (char)6, (char)21, (char)45, (char)171, (char)20, (char)5, (char)67, (char)134, (char)18, (char)143, (char)12, (char)171, (char)43, (char)80, (char)248, (char)58, (char)74, (char)5, (char)136, (char)11, (char)113, (char)134, (char)18, (char)80, (char)242, (char)52, (char)92, (char)121, (char)205, (char)91, (char)20, (char)97, (char)7, (char)29, (char)208, (char)149, (char)185, (char)182, (char)248, (char)218, (char)17, (char)189, (char)215, (char)124, (char)255, (char)95, (char)24, (char)252, (char)169, (char)248, (char)171, (char)129, (char)234, (char)29, (char)200, (char)84, (char)129, (char)59, (char)93, (char)254, (char)228, (char)224, (char)147, (char)74, (char)34, (char)3, (char)103, (char)235, (char)16, (char)242, (char)98, (char)65, (char)194, (char)3, (char)161, (char)64, (char)121, (char)106, (char)2, (char)155, (char)154, (char)174, (char)179, (char)125, (char)167, (char)142, (char)229, (char)119, (char)45, (char)101, (char)115, (char)127, (char)169, (char)208, (char)204, (char)238, (char)41, (char)189, (char)141, (char)204, (char)109, (char)252, (char)33, (char)213, (char)165, (char)67, (char)9, (char)136, (char)223, (char)89, (char)68, (char)182, (char)70, (char)126, (char)167, (char)249, (char)110, (char)20, (char)49}, 0) ;
        p267.target_system_SET((char)234) ;
        p267.sequence_SET((char)21212) ;
        p267.length_SET((char)52) ;
        p267.first_message_offset_SET((char)68) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)148);
            assert(pack.sequence_GET() == (char)60853);
            assert(pack.target_component_GET() == (char)96);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)148) ;
        p268.target_component_SET((char)96) ;
        p268.sequence_SET((char)60853) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.status_GET() == (char)116);
            assert(pack.rotation_GET() == (char)19851);
            assert(pack.uri_LEN(ph) == 138);
            assert(pack.uri_TRY(ph).equals("pvdtpobOqtpnmoxrluCybjyqoqtrwkniFbpImdrymxwMtjxQcngAregndDjmppvwzgadIyraflemSsttYsvounsxtquehyaNwfdivsfhdoqaecrvvxmWckzzrejmWpdtxdidcbnokc"));
            assert(pack.bitrate_GET() == 1705201319L);
            assert(pack.framerate_GET() == -1.996276E38F);
            assert(pack.camera_id_GET() == (char)207);
            assert(pack.resolution_v_GET() == (char)54153);
            assert(pack.resolution_h_GET() == (char)2073);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)207) ;
        p269.uri_SET("pvdtpobOqtpnmoxrluCybjyqoqtrwkniFbpImdrymxwMtjxQcngAregndDjmppvwzgadIyraflemSsttYsvounsxtquehyaNwfdivsfhdoqaecrvvxmWckzzrejmWpdtxdidcbnokc", PH) ;
        p269.framerate_SET(-1.996276E38F) ;
        p269.bitrate_SET(1705201319L) ;
        p269.status_SET((char)116) ;
        p269.rotation_SET((char)19851) ;
        p269.resolution_h_SET((char)2073) ;
        p269.resolution_v_SET((char)54153) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.bitrate_GET() == 3915977814L);
            assert(pack.resolution_h_GET() == (char)53507);
            assert(pack.target_system_GET() == (char)22);
            assert(pack.resolution_v_GET() == (char)55959);
            assert(pack.target_component_GET() == (char)222);
            assert(pack.camera_id_GET() == (char)132);
            assert(pack.uri_LEN(ph) == 139);
            assert(pack.uri_TRY(ph).equals("LvbkmvvykppftzxltUbogbOuAsbhaMkukeirClpyfdkaXhsbemqharilrhvibzeoIdrupvsghdspaadujUrKvnvuhdmgtmplmmvviovTmvydwqmsutrwxaopykoGipZyzshcyjltmfh"));
            assert(pack.framerate_GET() == -2.4135944E38F);
            assert(pack.rotation_GET() == (char)62674);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.uri_SET("LvbkmvvykppftzxltUbogbOuAsbhaMkukeirClpyfdkaXhsbemqharilrhvibzeoIdrupvsghdspaadujUrKvnvuhdmgtmplmmvviovTmvydwqmsutrwxaopykoGipZyzshcyjltmfh", PH) ;
        p270.bitrate_SET(3915977814L) ;
        p270.camera_id_SET((char)132) ;
        p270.rotation_SET((char)62674) ;
        p270.target_component_SET((char)222) ;
        p270.resolution_v_SET((char)55959) ;
        p270.framerate_SET(-2.4135944E38F) ;
        p270.resolution_h_SET((char)53507) ;
        p270.target_system_SET((char)22) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 31);
            assert(pack.password_TRY(ph).equals("dOtnnwhdrZcUhiJDyewQyskozihldjo"));
            assert(pack.ssid_LEN(ph) == 15);
            assert(pack.ssid_TRY(ph).equals("ihofpbWdnwxDwhz"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("ihofpbWdnwxDwhz", PH) ;
        p299.password_SET("dOtnnwhdrZcUhiJDyewQyskozihldjo", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.max_version_GET() == (char)12747);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)14, (char)240, (char)139, (char)11, (char)74, (char)4, (char)166, (char)65}));
            assert(pack.min_version_GET() == (char)51520);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)35, (char)70, (char)151, (char)20, (char)69, (char)83, (char)67, (char)1}));
            assert(pack.version_GET() == (char)41507);
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)41507) ;
        p300.min_version_SET((char)51520) ;
        p300.max_version_SET((char)12747) ;
        p300.library_version_hash_SET(new char[] {(char)35, (char)70, (char)151, (char)20, (char)69, (char)83, (char)67, (char)1}, 0) ;
        p300.spec_version_hash_SET(new char[] {(char)14, (char)240, (char)139, (char)11, (char)74, (char)4, (char)166, (char)65}, 0) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
            assert(pack.vendor_specific_status_code_GET() == (char)60271);
            assert(pack.time_usec_GET() == 4504238197676545782L);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
            assert(pack.sub_mode_GET() == (char)197);
            assert(pack.uptime_sec_GET() == 1061817062L);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.sub_mode_SET((char)197) ;
        p310.time_usec_SET(4504238197676545782L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL) ;
        p310.vendor_specific_status_code_SET((char)60271) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL) ;
        p310.uptime_sec_SET(1061817062L) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.hw_version_minor_GET() == (char)185);
            assert(pack.time_usec_GET() == 525020484166236795L);
            assert(pack.name_LEN(ph) == 40);
            assert(pack.name_TRY(ph).equals("nzPlybzGsymvWIwgSzjbfgotgiimzqmvfnaatbde"));
            assert(pack.hw_version_major_GET() == (char)120);
            assert(pack.uptime_sec_GET() == 555680650L);
            assert(pack.sw_version_minor_GET() == (char)54);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)235, (char)125, (char)39, (char)81, (char)133, (char)136, (char)110, (char)81, (char)109, (char)70, (char)76, (char)12, (char)236, (char)222, (char)13, (char)163}));
            assert(pack.sw_version_major_GET() == (char)167);
            assert(pack.sw_vcs_commit_GET() == 1304204364L);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.name_SET("nzPlybzGsymvWIwgSzjbfgotgiimzqmvfnaatbde", PH) ;
        p311.sw_version_major_SET((char)167) ;
        p311.time_usec_SET(525020484166236795L) ;
        p311.uptime_sec_SET(555680650L) ;
        p311.hw_version_major_SET((char)120) ;
        p311.sw_vcs_commit_SET(1304204364L) ;
        p311.sw_version_minor_SET((char)54) ;
        p311.hw_version_minor_SET((char)185) ;
        p311.hw_unique_id_SET(new char[] {(char)235, (char)125, (char)39, (char)81, (char)133, (char)136, (char)110, (char)81, (char)109, (char)70, (char)76, (char)12, (char)236, (char)222, (char)13, (char)163}, 0) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)0);
            assert(pack.param_index_GET() == (short) -16837);
            assert(pack.target_component_GET() == (char)191);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("hHqerHlp"));
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)0) ;
        p320.target_component_SET((char)191) ;
        p320.param_index_SET((short) -16837) ;
        p320.param_id_SET("hHqerHlp", PH) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)231);
            assert(pack.target_system_GET() == (char)44);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)44) ;
        p321.target_component_SET((char)231) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)2632);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("qcjv"));
            assert(pack.param_index_GET() == (char)28836);
            assert(pack.param_value_LEN(ph) == 61);
            assert(pack.param_value_TRY(ph).equals("qUuhpipteaOYmufftzPhwsffprzhfbPcpndjtmaykmwvbdswjgSavxtqqzpeo"));
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_index_SET((char)28836) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64) ;
        p322.param_value_SET("qUuhpipteaOYmufftzPhwsffprzhfbPcpndjtmaykmwvbdswjgSavxtqqzpeo", PH) ;
        p322.param_count_SET((char)2632) ;
        p322.param_id_SET("qcjv", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)14);
            assert(pack.param_value_LEN(ph) == 108);
            assert(pack.param_value_TRY(ph).equals("latsssglyuwpaajdnzVfpaqwPlqpsmptaDbvgmcBjxdawIlfoshzioekioBykCgdmsammhmiaqwbovcqKiRgbukktsyVxtqcxnxuohparkOh"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
            assert(pack.param_id_LEN(ph) == 13);
            assert(pack.param_id_TRY(ph).equals("aoaxmcmxspxLd"));
            assert(pack.target_system_GET() == (char)134);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_value_SET("latsssglyuwpaajdnzVfpaqwPlqpsmptaDbvgmcBjxdawIlfoshzioekioBykCgdmsammhmiaqwbovcqKiRgbukktsyVxtqcxnxuohparkOh", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32) ;
        p323.param_id_SET("aoaxmcmxspxLd", PH) ;
        p323.target_component_SET((char)14) ;
        p323.target_system_SET((char)134) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_FAILED);
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("xaksxfaoy"));
            assert(pack.param_value_LEN(ph) == 83);
            assert(pack.param_value_TRY(ph).equals("kdyrxurebhurpvCzechlxibhkybqvhkgcbpxvkwparxptyxrninetvqwwJtbhjZJxwwxktbqclstgomvjpz"));
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED) ;
        p324.param_value_SET("kdyrxurebhurpvCzechlxibhkybqvhkgcbpxvkwparxptyxrninetvqwwJtbhjZJxwwxktbqclstgomvjpz", PH) ;
        p324.param_id_SET("xaksxfaoy", PH) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.min_distance_GET() == (char)29061);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)52907, (char)4790, (char)42742, (char)60010, (char)63642, (char)51817, (char)47074, (char)50692, (char)11556, (char)48925, (char)62696, (char)11198, (char)30403, (char)42507, (char)63429, (char)35681, (char)44994, (char)48548, (char)11585, (char)3648, (char)64583, (char)2629, (char)29979, (char)53322, (char)2798, (char)59013, (char)38538, (char)6149, (char)10993, (char)22984, (char)54492, (char)47483, (char)16554, (char)16531, (char)8338, (char)28534, (char)42172, (char)4638, (char)18985, (char)63002, (char)61974, (char)61044, (char)3438, (char)22015, (char)7228, (char)13712, (char)36696, (char)44660, (char)46381, (char)53880, (char)28163, (char)1032, (char)48744, (char)13857, (char)58062, (char)54496, (char)6508, (char)19852, (char)57816, (char)14311, (char)40970, (char)33875, (char)125, (char)13498, (char)49174, (char)4428, (char)7031, (char)38539, (char)28317, (char)52770, (char)2727, (char)49558}));
            assert(pack.max_distance_GET() == (char)22526);
            assert(pack.increment_GET() == (char)208);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
            assert(pack.time_usec_GET() == 776027227804523156L);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.max_distance_SET((char)22526) ;
        p330.time_usec_SET(776027227804523156L) ;
        p330.increment_SET((char)208) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED) ;
        p330.distances_SET(new char[] {(char)52907, (char)4790, (char)42742, (char)60010, (char)63642, (char)51817, (char)47074, (char)50692, (char)11556, (char)48925, (char)62696, (char)11198, (char)30403, (char)42507, (char)63429, (char)35681, (char)44994, (char)48548, (char)11585, (char)3648, (char)64583, (char)2629, (char)29979, (char)53322, (char)2798, (char)59013, (char)38538, (char)6149, (char)10993, (char)22984, (char)54492, (char)47483, (char)16554, (char)16531, (char)8338, (char)28534, (char)42172, (char)4638, (char)18985, (char)63002, (char)61974, (char)61044, (char)3438, (char)22015, (char)7228, (char)13712, (char)36696, (char)44660, (char)46381, (char)53880, (char)28163, (char)1032, (char)48744, (char)13857, (char)58062, (char)54496, (char)6508, (char)19852, (char)57816, (char)14311, (char)40970, (char)33875, (char)125, (char)13498, (char)49174, (char)4428, (char)7031, (char)38539, (char)28317, (char)52770, (char)2727, (char)49558}, 0) ;
        p330.min_distance_SET((char)29061) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
    }

}
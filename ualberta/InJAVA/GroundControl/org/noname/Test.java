
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
            long id = id__x(src);
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
            long id = id__i(src);
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
            long id = id__i(src);
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
            long id = id__i(src);
            set_bits(id, 7, data, 260);
        }
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
    public static class NAV_FILTER_BIAS extends GroundControl.NAV_FILTER_BIAS
    {
        public long usec_GET()//Timestamp (microseconds)
        {  return (get_bytes(data,  0, 8)); }
        public float accel_0_GET()//b_f[0]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float accel_1_GET()//b_f[1]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float accel_2_GET()//b_f[2]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float gyro_0_GET()//b_f[0]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float gyro_1_GET()//b_f[1]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float gyro_2_GET()//b_f[2]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
    }
    public static class RADIO_CALIBRATION extends GroundControl.RADIO_CALIBRATION
    {
        public char[] aileron_GET(char[]  dst_ch, int pos)  //Aileron setpoints: left, center, right
        {
            for(int BYTE = 0, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] aileron_GET()//Aileron setpoints: left, center, right
        {return aileron_GET(new char[3], 0);} public char[] elevator_GET(char[]  dst_ch, int pos)  //Elevator setpoints: nose down, center, nose up
        {
            for(int BYTE = 6, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] elevator_GET()//Elevator setpoints: nose down, center, nose up
        {return elevator_GET(new char[3], 0);} public char[] rudder_GET(char[]  dst_ch, int pos)  //Rudder setpoints: nose left, center, nose right
        {
            for(int BYTE = 12, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] rudder_GET()//Rudder setpoints: nose left, center, nose right
        {return rudder_GET(new char[3], 0);} public char[] gyro_GET(char[]  dst_ch, int pos)  //Tail gyro mode/gain setpoints: heading hold, rate mode
        {
            for(int BYTE = 18, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] gyro_GET()//Tail gyro mode/gain setpoints: heading hold, rate mode
        {return gyro_GET(new char[2], 0);} public char[] pitch_GET(char[]  dst_ch, int pos)  //Pitch curve setpoints (every 25%)
        {
            for(int BYTE = 22, dst_max = pos + 5; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] pitch_GET()//Pitch curve setpoints (every 25%)
        {return pitch_GET(new char[5], 0);} public char[] throttle_GET(char[]  dst_ch, int pos)  //Throttle curve setpoints (every 25%)
        {
            for(int BYTE = 32, dst_max = pos + 5; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] throttle_GET()//Throttle curve setpoints (every 25%)
        {return throttle_GET(new char[5], 0);}
    }
    public static class UALBERTA_SYS_STATUS extends GroundControl.UALBERTA_SYS_STATUS
    {
        public char mode_GET()//System mode, see UALBERTA_AUTOPILOT_MODE ENUM
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char nav_mode_GET()//Navigation mode, see UALBERTA_NAV_MODE ENUM
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char pilot_GET()//Pilot mode, see UALBERTA_PILOT_MODE
        {  return (char)((char) get_bytes(data,  2, 1)); }
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

        static final Collection<OnReceive.Handler<ALTITUDE, Channel>> on_ALTITUDE = new OnReceive<>();
        static final Collection<OnReceive.Handler<RESOURCE_REQUEST, Channel>> on_RESOURCE_REQUEST = new OnReceive<>();
        static final Collection<OnReceive.Handler<SCALED_PRESSURE3, Channel>> on_SCALED_PRESSURE3 = new OnReceive<>();
        static final Collection<OnReceive.Handler<FOLLOW_TARGET, Channel>> on_FOLLOW_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<CONTROL_SYSTEM_STATE, Channel>> on_CONTROL_SYSTEM_STATE = new OnReceive<>();
        static final Collection<OnReceive.Handler<BATTERY_STATUS, Channel>> on_BATTERY_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<AUTOPILOT_VERSION, Channel>> on_AUTOPILOT_VERSION = new OnReceive<>();
        static final Collection<OnReceive.Handler<LANDING_TARGET, Channel>> on_LANDING_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<NAV_FILTER_BIAS, Channel>> on_NAV_FILTER_BIAS = new OnReceive<>();
        static final Collection<OnReceive.Handler<RADIO_CALIBRATION, Channel>> on_RADIO_CALIBRATION = new OnReceive<>();
        static final Collection<OnReceive.Handler<UALBERTA_SYS_STATUS, Channel>> on_UALBERTA_SYS_STATUS = new OnReceive<>();
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
                case 220:
                    if(pack == null) return new NAV_FILTER_BIAS();
                    ((OnReceive) on_NAV_FILTER_BIAS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 221:
                    if(pack == null) return new RADIO_CALIBRATION();
                    ((OnReceive) on_RADIO_CALIBRATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 222:
                    if(pack == null) return new UALBERTA_SYS_STATUS();
                    ((OnReceive) on_UALBERTA_SYS_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
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
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_VTOL_RESERVED5);
            assert(pack.custom_mode_GET() == 187709506L);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_UNINIT);
            assert(pack.mavlink_version_GET() == (char)78);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_OPENPILOT);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED)) ;
        p0.mavlink_version_SET((char)78) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_OPENPILOT) ;
        p0.custom_mode_SET(187709506L) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_UNINIT) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_VTOL_RESERVED5) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_comm_GET() == (char)22055);
            assert(pack.errors_count1_GET() == (char)20497);
            assert(pack.voltage_battery_GET() == (char)59758);
            assert(pack.errors_count4_GET() == (char)43129);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.errors_count2_GET() == (char)24161);
            assert(pack.load_GET() == (char)32880);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.battery_remaining_GET() == (byte) - 56);
            assert(pack.current_battery_GET() == (short)30136);
            assert(pack.drop_rate_comm_GET() == (char)40989);
            assert(pack.errors_count3_GET() == (char)7518);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.errors_count2_SET((char)24161) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW)) ;
        p1.battery_remaining_SET((byte) - 56) ;
        p1.load_SET((char)32880) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.errors_count4_SET((char)43129) ;
        p1.errors_count1_SET((char)20497) ;
        p1.drop_rate_comm_SET((char)40989) ;
        p1.voltage_battery_SET((char)59758) ;
        p1.current_battery_SET((short)30136) ;
        p1.errors_comm_SET((char)22055) ;
        p1.errors_count3_SET((char)7518) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1485549635L);
            assert(pack.time_unix_usec_GET() == 4681807395735168619L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(1485549635L) ;
        p2.time_unix_usec_SET(4681807395735168619L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == 2.0599237E38F);
            assert(pack.afz_GET() == -1.2630246E37F);
            assert(pack.vx_GET() == -2.7267734E38F);
            assert(pack.y_GET() == 9.854286E36F);
            assert(pack.type_mask_GET() == (char)60393);
            assert(pack.vy_GET() == 1.2640623E38F);
            assert(pack.afx_GET() == 9.528124E37F);
            assert(pack.time_boot_ms_GET() == 1717004827L);
            assert(pack.x_GET() == -5.155309E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.z_GET() == -3.1307968E38F);
            assert(pack.afy_GET() == -1.488037E38F);
            assert(pack.yaw_rate_GET() == -1.8280812E38F);
            assert(pack.yaw_GET() == -1.6942902E38F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.afy_SET(-1.488037E38F) ;
        p3.afz_SET(-1.2630246E37F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p3.type_mask_SET((char)60393) ;
        p3.x_SET(-5.155309E37F) ;
        p3.vy_SET(1.2640623E38F) ;
        p3.y_SET(9.854286E36F) ;
        p3.time_boot_ms_SET(1717004827L) ;
        p3.yaw_rate_SET(-1.8280812E38F) ;
        p3.afx_SET(9.528124E37F) ;
        p3.vz_SET(2.0599237E38F) ;
        p3.z_SET(-3.1307968E38F) ;
        p3.vx_SET(-2.7267734E38F) ;
        p3.yaw_SET(-1.6942902E38F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)234);
            assert(pack.seq_GET() == 1883645296L);
            assert(pack.target_component_GET() == (char)167);
            assert(pack.time_usec_GET() == 3411870851356903671L);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.seq_SET(1883645296L) ;
        p4.target_component_SET((char)167) ;
        p4.target_system_SET((char)234) ;
        p4.time_usec_SET(3411870851356903671L) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == (char)238);
            assert(pack.control_request_GET() == (char)175);
            assert(pack.passkey_LEN(ph) == 20);
            assert(pack.passkey_TRY(ph).equals("cmvDNbwdvbUsSwopoywr"));
            assert(pack.target_system_GET() == (char)135);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)135) ;
        p5.passkey_SET("cmvDNbwdvbUsSwopoywr", PH) ;
        p5.version_SET((char)238) ;
        p5.control_request_SET((char)175) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)111);
            assert(pack.control_request_GET() == (char)253);
            assert(pack.ack_GET() == (char)103);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)111) ;
        p6.control_request_SET((char)253) ;
        p6.ack_SET((char)103) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 30);
            assert(pack.key_TRY(ph).equals("HHhcjkqnksaosixbqZykgggaeismoo"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("HHhcjkqnksaosixbqZykgggaeismoo", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_MANUAL_DISARMED);
            assert(pack.target_system_GET() == (char)229);
            assert(pack.custom_mode_GET() == 3841152429L);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)229) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_MANUAL_DISARMED) ;
        p11.custom_mode_SET(3841152429L) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)229);
            assert(pack.target_system_GET() == (char)217);
            assert(pack.param_index_GET() == (short) -2037);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("z"));
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_component_SET((char)229) ;
        p20.param_index_SET((short) -2037) ;
        p20.target_system_SET((char)217) ;
        p20.param_id_SET("z", PH) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)26);
            assert(pack.target_system_GET() == (char)142);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)26) ;
        p21.target_system_SET((char)142) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_GET() == 2.7494242E38F);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("tav"));
            assert(pack.param_count_GET() == (char)53768);
            assert(pack.param_index_GET() == (char)19428);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_count_SET((char)53768) ;
        p22.param_index_SET((char)19428) ;
        p22.param_id_SET("tav", PH) ;
        p22.param_value_SET(2.7494242E38F) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)74);
            assert(pack.target_system_GET() == (char)199);
            assert(pack.param_value_GET() == -2.7013287E38F);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("c"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.target_component_SET((char)74) ;
        p23.target_system_SET((char)199) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16) ;
        p23.param_value_SET(-2.7013287E38F) ;
        p23.param_id_SET("c", PH) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.hdg_acc_TRY(ph) == 210689830L);
            assert(pack.v_acc_TRY(ph) == 2617341495L);
            assert(pack.vel_GET() == (char)2045);
            assert(pack.vel_acc_TRY(ph) == 2062572900L);
            assert(pack.epv_GET() == (char)3669);
            assert(pack.eph_GET() == (char)48185);
            assert(pack.lon_GET() == 1703566892);
            assert(pack.h_acc_TRY(ph) == 409629170L);
            assert(pack.satellites_visible_GET() == (char)76);
            assert(pack.time_usec_GET() == 379465026186406325L);
            assert(pack.alt_GET() == -63914628);
            assert(pack.alt_ellipsoid_TRY(ph) == -578034587);
            assert(pack.lat_GET() == 1203397749);
            assert(pack.cog_GET() == (char)64701);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.lon_SET(1703566892) ;
        p24.hdg_acc_SET(210689830L, PH) ;
        p24.vel_acc_SET(2062572900L, PH) ;
        p24.satellites_visible_SET((char)76) ;
        p24.cog_SET((char)64701) ;
        p24.eph_SET((char)48185) ;
        p24.alt_SET(-63914628) ;
        p24.h_acc_SET(409629170L, PH) ;
        p24.v_acc_SET(2617341495L, PH) ;
        p24.epv_SET((char)3669) ;
        p24.time_usec_SET(379465026186406325L) ;
        p24.vel_SET((char)2045) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP) ;
        p24.lat_SET(1203397749) ;
        p24.alt_ellipsoid_SET(-578034587, PH) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)107, (char)94, (char)142, (char)231, (char)173, (char)212, (char)175, (char)166, (char)128, (char)177, (char)105, (char)236, (char)48, (char)206, (char)40, (char)57, (char)7, (char)238, (char)134, (char)72}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)72, (char)212, (char)200, (char)130, (char)51, (char)168, (char)45, (char)109, (char)190, (char)189, (char)15, (char)253, (char)87, (char)23, (char)8, (char)115, (char)182, (char)97, (char)81, (char)187}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)69, (char)236, (char)142, (char)169, (char)251, (char)103, (char)0, (char)214, (char)129, (char)234, (char)33, (char)99, (char)64, (char)194, (char)248, (char)125, (char)183, (char)185, (char)212, (char)120}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)153, (char)73, (char)229, (char)20, (char)110, (char)8, (char)75, (char)188, (char)248, (char)221, (char)4, (char)236, (char)233, (char)169, (char)85, (char)40, (char)182, (char)113, (char)10, (char)125}));
            assert(pack.satellites_visible_GET() == (char)49);
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)49, (char)123, (char)51, (char)213, (char)174, (char)141, (char)77, (char)7, (char)212, (char)85, (char)34, (char)48, (char)149, (char)197, (char)105, (char)11, (char)191, (char)158, (char)15, (char)182}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_snr_SET(new char[] {(char)72, (char)212, (char)200, (char)130, (char)51, (char)168, (char)45, (char)109, (char)190, (char)189, (char)15, (char)253, (char)87, (char)23, (char)8, (char)115, (char)182, (char)97, (char)81, (char)187}, 0) ;
        p25.satellite_used_SET(new char[] {(char)107, (char)94, (char)142, (char)231, (char)173, (char)212, (char)175, (char)166, (char)128, (char)177, (char)105, (char)236, (char)48, (char)206, (char)40, (char)57, (char)7, (char)238, (char)134, (char)72}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)49, (char)123, (char)51, (char)213, (char)174, (char)141, (char)77, (char)7, (char)212, (char)85, (char)34, (char)48, (char)149, (char)197, (char)105, (char)11, (char)191, (char)158, (char)15, (char)182}, 0) ;
        p25.satellites_visible_SET((char)49) ;
        p25.satellite_azimuth_SET(new char[] {(char)153, (char)73, (char)229, (char)20, (char)110, (char)8, (char)75, (char)188, (char)248, (char)221, (char)4, (char)236, (char)233, (char)169, (char)85, (char)40, (char)182, (char)113, (char)10, (char)125}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)69, (char)236, (char)142, (char)169, (char)251, (char)103, (char)0, (char)214, (char)129, (char)234, (char)33, (char)99, (char)64, (char)194, (char)248, (char)125, (char)183, (char)185, (char)212, (char)120}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == (short)32527);
            assert(pack.ygyro_GET() == (short)17190);
            assert(pack.time_boot_ms_GET() == 2512162434L);
            assert(pack.ymag_GET() == (short)10830);
            assert(pack.xacc_GET() == (short)3957);
            assert(pack.xgyro_GET() == (short) -1205);
            assert(pack.zmag_GET() == (short)19659);
            assert(pack.zacc_GET() == (short) -8135);
            assert(pack.yacc_GET() == (short)7182);
            assert(pack.zgyro_GET() == (short)4328);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.yacc_SET((short)7182) ;
        p26.zacc_SET((short) -8135) ;
        p26.zmag_SET((short)19659) ;
        p26.ymag_SET((short)10830) ;
        p26.xmag_SET((short)32527) ;
        p26.xgyro_SET((short) -1205) ;
        p26.xacc_SET((short)3957) ;
        p26.ygyro_SET((short)17190) ;
        p26.time_boot_ms_SET(2512162434L) ;
        p26.zgyro_SET((short)4328) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == (short) -14072);
            assert(pack.ygyro_GET() == (short)18874);
            assert(pack.yacc_GET() == (short)26624);
            assert(pack.time_usec_GET() == 7423213995739003848L);
            assert(pack.xacc_GET() == (short) -1860);
            assert(pack.zgyro_GET() == (short)1327);
            assert(pack.zmag_GET() == (short)19437);
            assert(pack.ymag_GET() == (short) -3139);
            assert(pack.zacc_GET() == (short) -19896);
            assert(pack.xgyro_GET() == (short) -191);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.xacc_SET((short) -1860) ;
        p27.xmag_SET((short) -14072) ;
        p27.zacc_SET((short) -19896) ;
        p27.ygyro_SET((short)18874) ;
        p27.zgyro_SET((short)1327) ;
        p27.yacc_SET((short)26624) ;
        p27.zmag_SET((short)19437) ;
        p27.xgyro_SET((short) -191) ;
        p27.ymag_SET((short) -3139) ;
        p27.time_usec_SET(7423213995739003848L) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff2_GET() == (short) -32517);
            assert(pack.press_abs_GET() == (short)20576);
            assert(pack.temperature_GET() == (short)14647);
            assert(pack.press_diff1_GET() == (short) -11611);
            assert(pack.time_usec_GET() == 7961428127898903454L);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.temperature_SET((short)14647) ;
        p28.press_diff1_SET((short) -11611) ;
        p28.time_usec_SET(7961428127898903454L) ;
        p28.press_abs_SET((short)20576) ;
        p28.press_diff2_SET((short) -32517) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -22584);
            assert(pack.press_diff_GET() == 2.9205347E38F);
            assert(pack.time_boot_ms_GET() == 4001187480L);
            assert(pack.press_abs_GET() == -2.0954504E38F);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_abs_SET(-2.0954504E38F) ;
        p29.time_boot_ms_SET(4001187480L) ;
        p29.press_diff_SET(2.9205347E38F) ;
        p29.temperature_SET((short) -22584) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 3.3489449E38F);
            assert(pack.yawspeed_GET() == -2.444448E37F);
            assert(pack.rollspeed_GET() == 3.3464374E38F);
            assert(pack.pitch_GET() == -2.3486862E38F);
            assert(pack.yaw_GET() == -3.3620725E38F);
            assert(pack.pitchspeed_GET() == -2.0941728E38F);
            assert(pack.time_boot_ms_GET() == 1959792872L);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.pitch_SET(-2.3486862E38F) ;
        p30.rollspeed_SET(3.3464374E38F) ;
        p30.yaw_SET(-3.3620725E38F) ;
        p30.roll_SET(3.3489449E38F) ;
        p30.time_boot_ms_SET(1959792872L) ;
        p30.pitchspeed_SET(-2.0941728E38F) ;
        p30.yawspeed_SET(-2.444448E37F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q4_GET() == 2.4782823E38F);
            assert(pack.q1_GET() == 1.4779456E37F);
            assert(pack.yawspeed_GET() == 1.2156413E38F);
            assert(pack.q3_GET() == -2.0956435E38F);
            assert(pack.q2_GET() == -1.16072205E36F);
            assert(pack.time_boot_ms_GET() == 3535548302L);
            assert(pack.pitchspeed_GET() == 2.7418976E38F);
            assert(pack.rollspeed_GET() == 1.3448164E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q4_SET(2.4782823E38F) ;
        p31.q1_SET(1.4779456E37F) ;
        p31.pitchspeed_SET(2.7418976E38F) ;
        p31.time_boot_ms_SET(3535548302L) ;
        p31.q2_SET(-1.16072205E36F) ;
        p31.yawspeed_SET(1.2156413E38F) ;
        p31.q3_SET(-2.0956435E38F) ;
        p31.rollspeed_SET(1.3448164E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -6.316719E36F);
            assert(pack.z_GET() == 3.2098781E38F);
            assert(pack.vz_GET() == -1.1996153E38F);
            assert(pack.x_GET() == 1.1573226E38F);
            assert(pack.vx_GET() == -8.544961E37F);
            assert(pack.vy_GET() == -2.1617175E38F);
            assert(pack.time_boot_ms_GET() == 1665191727L);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.vz_SET(-1.1996153E38F) ;
        p32.x_SET(1.1573226E38F) ;
        p32.z_SET(3.2098781E38F) ;
        p32.y_SET(-6.316719E36F) ;
        p32.time_boot_ms_SET(1665191727L) ;
        p32.vx_SET(-8.544961E37F) ;
        p32.vy_SET(-2.1617175E38F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1341506746);
            assert(pack.time_boot_ms_GET() == 2459894882L);
            assert(pack.vy_GET() == (short)343);
            assert(pack.vz_GET() == (short) -3199);
            assert(pack.hdg_GET() == (char)18093);
            assert(pack.vx_GET() == (short)21040);
            assert(pack.lon_GET() == 1045847497);
            assert(pack.relative_alt_GET() == -1526839357);
            assert(pack.alt_GET() == -2114349982);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.vy_SET((short)343) ;
        p33.lon_SET(1045847497) ;
        p33.time_boot_ms_SET(2459894882L) ;
        p33.lat_SET(1341506746) ;
        p33.vx_SET((short)21040) ;
        p33.alt_SET(-2114349982) ;
        p33.relative_alt_SET(-1526839357) ;
        p33.hdg_SET((char)18093) ;
        p33.vz_SET((short) -3199) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan3_scaled_GET() == (short) -31446);
            assert(pack.chan8_scaled_GET() == (short)1641);
            assert(pack.chan7_scaled_GET() == (short)17313);
            assert(pack.rssi_GET() == (char)205);
            assert(pack.port_GET() == (char)63);
            assert(pack.chan5_scaled_GET() == (short) -11185);
            assert(pack.chan2_scaled_GET() == (short) -21664);
            assert(pack.chan6_scaled_GET() == (short) -15288);
            assert(pack.time_boot_ms_GET() == 762599839L);
            assert(pack.chan4_scaled_GET() == (short) -6615);
            assert(pack.chan1_scaled_GET() == (short)2739);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan7_scaled_SET((short)17313) ;
        p34.rssi_SET((char)205) ;
        p34.time_boot_ms_SET(762599839L) ;
        p34.chan3_scaled_SET((short) -31446) ;
        p34.port_SET((char)63) ;
        p34.chan8_scaled_SET((short)1641) ;
        p34.chan5_scaled_SET((short) -11185) ;
        p34.chan2_scaled_SET((short) -21664) ;
        p34.chan4_scaled_SET((short) -6615) ;
        p34.chan6_scaled_SET((short) -15288) ;
        p34.chan1_scaled_SET((short)2739) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.port_GET() == (char)123);
            assert(pack.time_boot_ms_GET() == 2697297420L);
            assert(pack.chan6_raw_GET() == (char)36938);
            assert(pack.chan2_raw_GET() == (char)14991);
            assert(pack.chan4_raw_GET() == (char)50258);
            assert(pack.chan8_raw_GET() == (char)52779);
            assert(pack.chan1_raw_GET() == (char)20887);
            assert(pack.rssi_GET() == (char)157);
            assert(pack.chan5_raw_GET() == (char)51476);
            assert(pack.chan7_raw_GET() == (char)27352);
            assert(pack.chan3_raw_GET() == (char)5906);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan3_raw_SET((char)5906) ;
        p35.chan2_raw_SET((char)14991) ;
        p35.chan4_raw_SET((char)50258) ;
        p35.time_boot_ms_SET(2697297420L) ;
        p35.chan5_raw_SET((char)51476) ;
        p35.chan1_raw_SET((char)20887) ;
        p35.rssi_SET((char)157) ;
        p35.port_SET((char)123) ;
        p35.chan7_raw_SET((char)27352) ;
        p35.chan6_raw_SET((char)36938) ;
        p35.chan8_raw_SET((char)52779) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo5_raw_GET() == (char)12391);
            assert(pack.servo9_raw_TRY(ph) == (char)57436);
            assert(pack.servo8_raw_GET() == (char)57474);
            assert(pack.servo4_raw_GET() == (char)47198);
            assert(pack.servo13_raw_TRY(ph) == (char)2825);
            assert(pack.time_usec_GET() == 2968197960L);
            assert(pack.servo6_raw_GET() == (char)60945);
            assert(pack.servo12_raw_TRY(ph) == (char)50775);
            assert(pack.servo10_raw_TRY(ph) == (char)3090);
            assert(pack.servo11_raw_TRY(ph) == (char)44090);
            assert(pack.port_GET() == (char)249);
            assert(pack.servo14_raw_TRY(ph) == (char)13538);
            assert(pack.servo3_raw_GET() == (char)25821);
            assert(pack.servo7_raw_GET() == (char)49961);
            assert(pack.servo2_raw_GET() == (char)7850);
            assert(pack.servo1_raw_GET() == (char)53649);
            assert(pack.servo15_raw_TRY(ph) == (char)60696);
            assert(pack.servo16_raw_TRY(ph) == (char)49485);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo6_raw_SET((char)60945) ;
        p36.servo7_raw_SET((char)49961) ;
        p36.servo3_raw_SET((char)25821) ;
        p36.servo2_raw_SET((char)7850) ;
        p36.servo12_raw_SET((char)50775, PH) ;
        p36.servo14_raw_SET((char)13538, PH) ;
        p36.servo1_raw_SET((char)53649) ;
        p36.servo9_raw_SET((char)57436, PH) ;
        p36.servo10_raw_SET((char)3090, PH) ;
        p36.servo16_raw_SET((char)49485, PH) ;
        p36.time_usec_SET(2968197960L) ;
        p36.servo5_raw_SET((char)12391) ;
        p36.port_SET((char)249) ;
        p36.servo8_raw_SET((char)57474) ;
        p36.servo13_raw_SET((char)2825, PH) ;
        p36.servo4_raw_SET((char)47198) ;
        p36.servo15_raw_SET((char)60696, PH) ;
        p36.servo11_raw_SET((char)44090, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)100);
            assert(pack.end_index_GET() == (short)20727);
            assert(pack.target_system_GET() == (char)84);
            assert(pack.start_index_GET() == (short) -826);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_component_SET((char)100) ;
        p37.target_system_SET((char)84) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p37.end_index_SET((short)20727) ;
        p37.start_index_SET((short) -826) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short) -20616);
            assert(pack.target_component_GET() == (char)201);
            assert(pack.target_system_GET() == (char)125);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.start_index_GET() == (short) -15723);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.end_index_SET((short) -20616) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p38.start_index_SET((short) -15723) ;
        p38.target_component_SET((char)201) ;
        p38.target_system_SET((char)125) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_DELAY);
            assert(pack.param2_GET() == 3.213961E38F);
            assert(pack.param1_GET() == 1.4583487E38F);
            assert(pack.param4_GET() == -2.4045792E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.param3_GET() == -1.2488444E38F);
            assert(pack.seq_GET() == (char)11312);
            assert(pack.target_component_GET() == (char)108);
            assert(pack.current_GET() == (char)120);
            assert(pack.y_GET() == -1.0908136E38F);
            assert(pack.z_GET() == 3.1518487E38F);
            assert(pack.x_GET() == -5.3285997E37F);
            assert(pack.autocontinue_GET() == (char)119);
            assert(pack.target_system_GET() == (char)22);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.target_component_SET((char)108) ;
        p39.y_SET(-1.0908136E38F) ;
        p39.z_SET(3.1518487E38F) ;
        p39.target_system_SET((char)22) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p39.param2_SET(3.213961E38F) ;
        p39.param3_SET(-1.2488444E38F) ;
        p39.param4_SET(-2.4045792E38F) ;
        p39.param1_SET(1.4583487E38F) ;
        p39.x_SET(-5.3285997E37F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_NAV_DELAY) ;
        p39.seq_SET((char)11312) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p39.current_SET((char)120) ;
        p39.autocontinue_SET((char)119) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)106);
            assert(pack.target_component_GET() == (char)140);
            assert(pack.seq_GET() == (char)51423);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_component_SET((char)140) ;
        p40.seq_SET((char)51423) ;
        p40.target_system_SET((char)106) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)210);
            assert(pack.seq_GET() == (char)53478);
            assert(pack.target_system_GET() == (char)122);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.seq_SET((char)53478) ;
        p41.target_system_SET((char)122) ;
        p41.target_component_SET((char)210) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)57562);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)57562) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)229);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)105);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p43.target_system_SET((char)229) ;
        p43.target_component_SET((char)105) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)52);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)81);
            assert(pack.count_GET() == (char)34926);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)81) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p44.count_SET((char)34926) ;
        p44.target_component_SET((char)52) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)71);
            assert(pack.target_component_GET() == (char)214);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p45.target_system_SET((char)71) ;
        p45.target_component_SET((char)214) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)32557);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)32557) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE);
            assert(pack.target_system_GET() == (char)100);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)16);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p47.target_component_SET((char)16) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE) ;
        p47.target_system_SET((char)100) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 313901382);
            assert(pack.longitude_GET() == 910366701);
            assert(pack.target_system_GET() == (char)32);
            assert(pack.time_usec_TRY(ph) == 8523605791142354451L);
            assert(pack.altitude_GET() == 355527469);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.time_usec_SET(8523605791142354451L, PH) ;
        p48.target_system_SET((char)32) ;
        p48.latitude_SET(313901382) ;
        p48.altitude_SET(355527469) ;
        p48.longitude_SET(910366701) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == -937855224);
            assert(pack.altitude_GET() == -654307384);
            assert(pack.time_usec_TRY(ph) == 3190905389254338741L);
            assert(pack.longitude_GET() == -795818607);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.altitude_SET(-654307384) ;
        p49.latitude_SET(-937855224) ;
        p49.longitude_SET(-795818607) ;
        p49.time_usec_SET(3190905389254338741L, PH) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_value_max_GET() == -1.4137429E37F);
            assert(pack.scale_GET() == -2.6154005E38F);
            assert(pack.target_component_GET() == (char)137);
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("nhhlgjueYIzcqppz"));
            assert(pack.param_index_GET() == (short) -7438);
            assert(pack.target_system_GET() == (char)60);
            assert(pack.param_value0_GET() == -5.072315E37F);
            assert(pack.param_value_min_GET() == 2.4569203E37F);
            assert(pack.parameter_rc_channel_index_GET() == (char)55);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.param_value_min_SET(2.4569203E37F) ;
        p50.param_index_SET((short) -7438) ;
        p50.param_id_SET("nhhlgjueYIzcqppz", PH) ;
        p50.parameter_rc_channel_index_SET((char)55) ;
        p50.scale_SET(-2.6154005E38F) ;
        p50.param_value0_SET(-5.072315E37F) ;
        p50.param_value_max_SET(-1.4137429E37F) ;
        p50.target_system_SET((char)60) ;
        p50.target_component_SET((char)137) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)148);
            assert(pack.target_system_GET() == (char)50);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.seq_GET() == (char)27596);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_system_SET((char)50) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p51.target_component_SET((char)148) ;
        p51.seq_SET((char)27596) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.p2x_GET() == 2.4174298E38F);
            assert(pack.p1z_GET() == 1.1734229E38F);
            assert(pack.target_component_GET() == (char)230);
            assert(pack.p2z_GET() == -8.965946E37F);
            assert(pack.target_system_GET() == (char)251);
            assert(pack.p1y_GET() == -1.4939782E38F);
            assert(pack.p1x_GET() == -1.4126863E38F);
            assert(pack.p2y_GET() == 2.2008534E37F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p1y_SET(-1.4939782E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p54.p1x_SET(-1.4126863E38F) ;
        p54.target_component_SET((char)230) ;
        p54.p2y_SET(2.2008534E37F) ;
        p54.p1z_SET(1.1734229E38F) ;
        p54.p2z_SET(-8.965946E37F) ;
        p54.p2x_SET(2.4174298E38F) ;
        p54.target_system_SET((char)251) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2y_GET() == -2.581821E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.p2z_GET() == 2.727835E38F);
            assert(pack.p1y_GET() == -1.5076542E38F);
            assert(pack.p1x_GET() == 2.9367674E38F);
            assert(pack.p2x_GET() == -4.827833E37F);
            assert(pack.p1z_GET() == 1.2328819E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p1x_SET(2.9367674E38F) ;
        p55.p2y_SET(-2.581821E38F) ;
        p55.p1y_SET(-1.5076542E38F) ;
        p55.p2x_SET(-4.827833E37F) ;
        p55.p1z_SET(1.2328819E38F) ;
        p55.p2z_SET(2.727835E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-3.294296E38F, 2.1399926E38F, -1.5216041E38F, 2.4778484E38F, 3.2358386E38F, 1.2782908E38F, -9.799993E37F, -7.049939E37F, -2.0628813E38F}));
            assert(pack.yawspeed_GET() == -6.0613976E37F);
            assert(pack.rollspeed_GET() == 4.3740213E37F);
            assert(pack.pitchspeed_GET() == 5.612413E37F);
            assert(pack.time_usec_GET() == 7815097602570086053L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.582581E38F, 1.6405295E38F, 1.6647652E38F, -3.2420844E38F}));
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.time_usec_SET(7815097602570086053L) ;
        p61.covariance_SET(new float[] {-3.294296E38F, 2.1399926E38F, -1.5216041E38F, 2.4778484E38F, 3.2358386E38F, 1.2782908E38F, -9.799993E37F, -7.049939E37F, -2.0628813E38F}, 0) ;
        p61.pitchspeed_SET(5.612413E37F) ;
        p61.rollspeed_SET(4.3740213E37F) ;
        p61.yawspeed_SET(-6.0613976E37F) ;
        p61.q_SET(new float[] {2.582581E38F, 1.6405295E38F, 1.6647652E38F, -3.2420844E38F}, 0) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_pitch_GET() == -8.4051786E37F);
            assert(pack.nav_bearing_GET() == (short) -21502);
            assert(pack.nav_roll_GET() == -1.4540134E38F);
            assert(pack.aspd_error_GET() == 6.512605E37F);
            assert(pack.target_bearing_GET() == (short) -3245);
            assert(pack.alt_error_GET() == -1.8388897E38F);
            assert(pack.wp_dist_GET() == (char)36055);
            assert(pack.xtrack_error_GET() == -7.557984E37F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.aspd_error_SET(6.512605E37F) ;
        p62.nav_roll_SET(-1.4540134E38F) ;
        p62.wp_dist_SET((char)36055) ;
        p62.nav_pitch_SET(-8.4051786E37F) ;
        p62.target_bearing_SET((short) -3245) ;
        p62.alt_error_SET(-1.8388897E38F) ;
        p62.nav_bearing_SET((short) -21502) ;
        p62.xtrack_error_SET(-7.557984E37F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1355858045);
            assert(pack.alt_GET() == -2143425727);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-9.580497E37F, -1.1356961E38F, -2.1372944E37F, -2.1633017E38F, -1.6730327E38F, -1.9568947E38F, -9.568215E37F, 2.5192043E38F, 6.461632E37F, -3.3280102E38F, -1.4572233E38F, -3.2651148E38F, 2.7028694E38F, -2.7467808E38F, -3.068697E38F, -1.1492615E37F, -3.0621078E38F, -2.2954964E38F, 3.1664506E38F, 8.46125E35F, -2.3728083E38F, 5.3530786E37F, 3.3322293E38F, -7.716906E37F, 1.6058639E38F, -3.2742963E38F, -1.5482518E38F, 5.8755296E37F, 3.2859142E38F, 3.2241636E38F, 1.6294292E38F, -7.699693E37F, 2.5901757E38F, -4.5529953E37F, 2.2766376E37F, -3.3829982E37F}));
            assert(pack.relative_alt_GET() == 1058047156);
            assert(pack.lon_GET() == 966036724);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.vx_GET() == 2.3427368E38F);
            assert(pack.time_usec_GET() == 3469619635538710802L);
            assert(pack.vz_GET() == -2.7186326E37F);
            assert(pack.vy_GET() == -2.959826E38F);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.relative_alt_SET(1058047156) ;
        p63.covariance_SET(new float[] {-9.580497E37F, -1.1356961E38F, -2.1372944E37F, -2.1633017E38F, -1.6730327E38F, -1.9568947E38F, -9.568215E37F, 2.5192043E38F, 6.461632E37F, -3.3280102E38F, -1.4572233E38F, -3.2651148E38F, 2.7028694E38F, -2.7467808E38F, -3.068697E38F, -1.1492615E37F, -3.0621078E38F, -2.2954964E38F, 3.1664506E38F, 8.46125E35F, -2.3728083E38F, 5.3530786E37F, 3.3322293E38F, -7.716906E37F, 1.6058639E38F, -3.2742963E38F, -1.5482518E38F, 5.8755296E37F, 3.2859142E38F, 3.2241636E38F, 1.6294292E38F, -7.699693E37F, 2.5901757E38F, -4.5529953E37F, 2.2766376E37F, -3.3829982E37F}, 0) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p63.lon_SET(966036724) ;
        p63.time_usec_SET(3469619635538710802L) ;
        p63.alt_SET(-2143425727) ;
        p63.lat_SET(1355858045) ;
        p63.vy_SET(-2.959826E38F) ;
        p63.vx_SET(2.3427368E38F) ;
        p63.vz_SET(-2.7186326E37F) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7647586685529798900L);
            assert(pack.vx_GET() == -1.7129996E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.3986702E38F, 3.3338724E38F, -1.3264975E38F, -5.551603E37F, 2.5064146E38F, 2.9748695E38F, 7.272243E36F, -1.3480523E38F, 2.2505674E38F, -2.8963695E38F, 3.26813E38F, 1.5573603E38F, 2.9346507E37F, 2.8557986E38F, 1.6277749E38F, 8.447123E35F, -2.807161E38F, -2.8479525E38F, 2.8277667E38F, 1.726867E38F, 1.3565293E38F, -1.7089457E38F, -2.4844356E38F, -2.8523465E38F, -2.194832E38F, 6.794202E37F, 2.3850692E38F, -3.0997408E38F, 3.6694223E37F, -3.3335E38F, 2.466358E38F, -2.3648273E38F, -2.545322E38F, 3.22699E38F, -2.4202914E38F, 1.3704001E38F, 1.231894E38F, -2.888992E38F, 2.2374082E38F, 1.4421854E38F, -2.9198455E38F, -1.8107085E36F, -1.2371836E38F, -7.8156237E37F, 1.414535E38F}));
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.vz_GET() == 6.256975E36F);
            assert(pack.z_GET() == 7.706024E37F);
            assert(pack.az_GET() == 4.4101564E37F);
            assert(pack.vy_GET() == 1.340287E38F);
            assert(pack.y_GET() == 1.3330531E38F);
            assert(pack.ay_GET() == 2.8820105E38F);
            assert(pack.ax_GET() == 1.3176547E38F);
            assert(pack.x_GET() == -3.0767008E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.covariance_SET(new float[] {-1.3986702E38F, 3.3338724E38F, -1.3264975E38F, -5.551603E37F, 2.5064146E38F, 2.9748695E38F, 7.272243E36F, -1.3480523E38F, 2.2505674E38F, -2.8963695E38F, 3.26813E38F, 1.5573603E38F, 2.9346507E37F, 2.8557986E38F, 1.6277749E38F, 8.447123E35F, -2.807161E38F, -2.8479525E38F, 2.8277667E38F, 1.726867E38F, 1.3565293E38F, -1.7089457E38F, -2.4844356E38F, -2.8523465E38F, -2.194832E38F, 6.794202E37F, 2.3850692E38F, -3.0997408E38F, 3.6694223E37F, -3.3335E38F, 2.466358E38F, -2.3648273E38F, -2.545322E38F, 3.22699E38F, -2.4202914E38F, 1.3704001E38F, 1.231894E38F, -2.888992E38F, 2.2374082E38F, 1.4421854E38F, -2.9198455E38F, -1.8107085E36F, -1.2371836E38F, -7.8156237E37F, 1.414535E38F}, 0) ;
        p64.vx_SET(-1.7129996E38F) ;
        p64.z_SET(7.706024E37F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p64.x_SET(-3.0767008E38F) ;
        p64.ay_SET(2.8820105E38F) ;
        p64.vz_SET(6.256975E36F) ;
        p64.time_usec_SET(7647586685529798900L) ;
        p64.az_SET(4.4101564E37F) ;
        p64.vy_SET(1.340287E38F) ;
        p64.y_SET(1.3330531E38F) ;
        p64.ax_SET(1.3176547E38F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan11_raw_GET() == (char)51517);
            assert(pack.chan15_raw_GET() == (char)48731);
            assert(pack.chan1_raw_GET() == (char)27861);
            assert(pack.chancount_GET() == (char)243);
            assert(pack.time_boot_ms_GET() == 4237480272L);
            assert(pack.chan7_raw_GET() == (char)6348);
            assert(pack.chan2_raw_GET() == (char)52440);
            assert(pack.chan3_raw_GET() == (char)14442);
            assert(pack.chan4_raw_GET() == (char)57228);
            assert(pack.chan8_raw_GET() == (char)25410);
            assert(pack.chan6_raw_GET() == (char)27659);
            assert(pack.chan10_raw_GET() == (char)43705);
            assert(pack.chan16_raw_GET() == (char)20849);
            assert(pack.chan13_raw_GET() == (char)31522);
            assert(pack.chan14_raw_GET() == (char)45422);
            assert(pack.chan9_raw_GET() == (char)35257);
            assert(pack.rssi_GET() == (char)139);
            assert(pack.chan5_raw_GET() == (char)45665);
            assert(pack.chan12_raw_GET() == (char)46372);
            assert(pack.chan18_raw_GET() == (char)12723);
            assert(pack.chan17_raw_GET() == (char)30992);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan18_raw_SET((char)12723) ;
        p65.chan3_raw_SET((char)14442) ;
        p65.chan11_raw_SET((char)51517) ;
        p65.chan6_raw_SET((char)27659) ;
        p65.chan12_raw_SET((char)46372) ;
        p65.time_boot_ms_SET(4237480272L) ;
        p65.chan17_raw_SET((char)30992) ;
        p65.chan5_raw_SET((char)45665) ;
        p65.chan1_raw_SET((char)27861) ;
        p65.chancount_SET((char)243) ;
        p65.chan4_raw_SET((char)57228) ;
        p65.rssi_SET((char)139) ;
        p65.chan13_raw_SET((char)31522) ;
        p65.chan9_raw_SET((char)35257) ;
        p65.chan8_raw_SET((char)25410) ;
        p65.chan15_raw_SET((char)48731) ;
        p65.chan7_raw_SET((char)6348) ;
        p65.chan14_raw_SET((char)45422) ;
        p65.chan16_raw_SET((char)20849) ;
        p65.chan10_raw_SET((char)43705) ;
        p65.chan2_raw_SET((char)52440) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.req_stream_id_GET() == (char)82);
            assert(pack.req_message_rate_GET() == (char)46593);
            assert(pack.start_stop_GET() == (char)203);
            assert(pack.target_component_GET() == (char)23);
            assert(pack.target_system_GET() == (char)86);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_system_SET((char)86) ;
        p66.req_message_rate_SET((char)46593) ;
        p66.target_component_SET((char)23) ;
        p66.start_stop_SET((char)203) ;
        p66.req_stream_id_SET((char)82) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.on_off_GET() == (char)88);
            assert(pack.message_rate_GET() == (char)45130);
            assert(pack.stream_id_GET() == (char)247);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)247) ;
        p67.message_rate_SET((char)45130) ;
        p67.on_off_SET((char)88) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.r_GET() == (short)19237);
            assert(pack.z_GET() == (short)21977);
            assert(pack.target_GET() == (char)222);
            assert(pack.x_GET() == (short) -30653);
            assert(pack.buttons_GET() == (char)58962);
            assert(pack.y_GET() == (short)30705);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.target_SET((char)222) ;
        p69.z_SET((short)21977) ;
        p69.y_SET((short)30705) ;
        p69.buttons_SET((char)58962) ;
        p69.r_SET((short)19237) ;
        p69.x_SET((short) -30653) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan7_raw_GET() == (char)10396);
            assert(pack.chan4_raw_GET() == (char)56918);
            assert(pack.chan1_raw_GET() == (char)51121);
            assert(pack.chan3_raw_GET() == (char)39827);
            assert(pack.target_component_GET() == (char)235);
            assert(pack.chan6_raw_GET() == (char)44897);
            assert(pack.chan2_raw_GET() == (char)32064);
            assert(pack.target_system_GET() == (char)61);
            assert(pack.chan8_raw_GET() == (char)27746);
            assert(pack.chan5_raw_GET() == (char)3987);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan2_raw_SET((char)32064) ;
        p70.chan1_raw_SET((char)51121) ;
        p70.chan5_raw_SET((char)3987) ;
        p70.target_system_SET((char)61) ;
        p70.chan4_raw_SET((char)56918) ;
        p70.chan8_raw_SET((char)27746) ;
        p70.chan6_raw_SET((char)44897) ;
        p70.target_component_SET((char)235) ;
        p70.chan7_raw_SET((char)10396) ;
        p70.chan3_raw_SET((char)39827) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param3_GET() == -8.044516E37F);
            assert(pack.target_system_GET() == (char)212);
            assert(pack.current_GET() == (char)213);
            assert(pack.z_GET() == -2.682772E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_REPEAT_SERVO);
            assert(pack.autocontinue_GET() == (char)46);
            assert(pack.x_GET() == 1813042968);
            assert(pack.param1_GET() == 2.4715783E38F);
            assert(pack.target_component_GET() == (char)83);
            assert(pack.y_GET() == -581254669);
            assert(pack.seq_GET() == (char)46301);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.param4_GET() == 1.9457887E38F);
            assert(pack.param2_GET() == -2.6935565E38F);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.x_SET(1813042968) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p73.param4_SET(1.9457887E38F) ;
        p73.seq_SET((char)46301) ;
        p73.target_component_SET((char)83) ;
        p73.autocontinue_SET((char)46) ;
        p73.param1_SET(2.4715783E38F) ;
        p73.param3_SET(-8.044516E37F) ;
        p73.target_system_SET((char)212) ;
        p73.y_SET(-581254669) ;
        p73.command_SET(MAV_CMD.MAV_CMD_DO_REPEAT_SERVO) ;
        p73.current_SET((char)213) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p73.param2_SET(-2.6935565E38F) ;
        p73.z_SET(-2.682772E38F) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.climb_GET() == 7.4605477E37F);
            assert(pack.throttle_GET() == (char)20894);
            assert(pack.airspeed_GET() == 2.873988E38F);
            assert(pack.groundspeed_GET() == 1.2598829E38F);
            assert(pack.heading_GET() == (short)21455);
            assert(pack.alt_GET() == 2.054122E38F);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.climb_SET(7.4605477E37F) ;
        p74.throttle_SET((char)20894) ;
        p74.alt_SET(2.054122E38F) ;
        p74.airspeed_SET(2.873988E38F) ;
        p74.heading_SET((short)21455) ;
        p74.groundspeed_SET(1.2598829E38F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.autocontinue_GET() == (char)175);
            assert(pack.target_component_GET() == (char)1);
            assert(pack.current_GET() == (char)53);
            assert(pack.param1_GET() == -1.6325177E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.y_GET() == 1019026730);
            assert(pack.param3_GET() == 3.2744015E38F);
            assert(pack.param4_GET() == 1.4887847E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION);
            assert(pack.z_GET() == -2.100223E38F);
            assert(pack.target_system_GET() == (char)243);
            assert(pack.x_GET() == 222702915);
            assert(pack.param2_GET() == 1.590971E38F);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.param4_SET(1.4887847E38F) ;
        p75.target_system_SET((char)243) ;
        p75.current_SET((char)53) ;
        p75.param3_SET(3.2744015E38F) ;
        p75.y_SET(1019026730) ;
        p75.target_component_SET((char)1) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p75.x_SET(222702915) ;
        p75.param1_SET(-1.6325177E38F) ;
        p75.autocontinue_SET((char)175) ;
        p75.z_SET(-2.100223E38F) ;
        p75.param2_SET(1.590971E38F) ;
        p75.command_SET(MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param6_GET() == -9.449051E37F);
            assert(pack.param5_GET() == 2.2902047E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_PARACHUTE);
            assert(pack.param3_GET() == 2.4190274E38F);
            assert(pack.param1_GET() == -3.0261404E38F);
            assert(pack.confirmation_GET() == (char)54);
            assert(pack.target_system_GET() == (char)59);
            assert(pack.param2_GET() == -3.350375E38F);
            assert(pack.param4_GET() == 1.1218056E38F);
            assert(pack.param7_GET() == -1.9242293E38F);
            assert(pack.target_component_GET() == (char)64);
        });
        GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param1_SET(-3.0261404E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_DO_PARACHUTE) ;
        p76.target_system_SET((char)59) ;
        p76.param5_SET(2.2902047E38F) ;
        p76.param6_SET(-9.449051E37F) ;
        p76.confirmation_SET((char)54) ;
        p76.target_component_SET((char)64) ;
        p76.param3_SET(2.4190274E38F) ;
        p76.param7_SET(-1.9242293E38F) ;
        p76.param4_SET(1.1218056E38F) ;
        p76.param2_SET(-3.350375E38F) ;
        CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.result_param2_TRY(ph) == -1619644681);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_IN_PROGRESS);
            assert(pack.target_component_TRY(ph) == (char)10);
            assert(pack.progress_TRY(ph) == (char)136);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_JUMP);
            assert(pack.target_system_TRY(ph) == (char)111);
        });
        GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.target_system_SET((char)111, PH) ;
        p77.progress_SET((char)136, PH) ;
        p77.result_param2_SET(-1619644681, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_DO_JUMP) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_IN_PROGRESS) ;
        p77.target_component_SET((char)10, PH) ;
        CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -1.0374458E38F);
            assert(pack.manual_override_switch_GET() == (char)1);
            assert(pack.yaw_GET() == -2.970655E38F);
            assert(pack.mode_switch_GET() == (char)110);
            assert(pack.time_boot_ms_GET() == 4294305794L);
            assert(pack.pitch_GET() == -1.7203749E38F);
            assert(pack.thrust_GET() == 2.6335778E38F);
        });
        GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.thrust_SET(2.6335778E38F) ;
        p81.yaw_SET(-2.970655E38F) ;
        p81.mode_switch_SET((char)110) ;
        p81.roll_SET(-1.0374458E38F) ;
        p81.manual_override_switch_SET((char)1) ;
        p81.pitch_SET(-1.7203749E38F) ;
        p81.time_boot_ms_SET(4294305794L) ;
        CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_yaw_rate_GET() == -6.9276816E37F);
            assert(pack.body_roll_rate_GET() == -1.5506438E38F);
            assert(pack.target_component_GET() == (char)231);
            assert(pack.type_mask_GET() == (char)232);
            assert(pack.time_boot_ms_GET() == 2099457666L);
            assert(pack.body_pitch_rate_GET() == -2.4949767E38F);
            assert(pack.thrust_GET() == -5.971251E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {6.953127E37F, 2.7846304E38F, 1.0586492E38F, 1.968203E38F}));
            assert(pack.target_system_GET() == (char)67);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.thrust_SET(-5.971251E37F) ;
        p82.body_yaw_rate_SET(-6.9276816E37F) ;
        p82.q_SET(new float[] {6.953127E37F, 2.7846304E38F, 1.0586492E38F, 1.968203E38F}, 0) ;
        p82.time_boot_ms_SET(2099457666L) ;
        p82.type_mask_SET((char)232) ;
        p82.body_roll_rate_SET(-1.5506438E38F) ;
        p82.target_component_SET((char)231) ;
        p82.target_system_SET((char)67) ;
        p82.body_pitch_rate_SET(-2.4949767E38F) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3275022329L);
            assert(pack.body_roll_rate_GET() == 1.1879413E38F);
            assert(pack.body_yaw_rate_GET() == 1.679357E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.3294068E38F, -8.820342E37F, -1.6428874E38F, -2.7673078E38F}));
            assert(pack.body_pitch_rate_GET() == 1.1846234E38F);
            assert(pack.thrust_GET() == 1.282084E38F);
            assert(pack.type_mask_GET() == (char)74);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.time_boot_ms_SET(3275022329L) ;
        p83.body_roll_rate_SET(1.1879413E38F) ;
        p83.thrust_SET(1.282084E38F) ;
        p83.body_yaw_rate_SET(1.679357E38F) ;
        p83.q_SET(new float[] {3.3294068E38F, -8.820342E37F, -1.6428874E38F, -2.7673078E38F}, 0) ;
        p83.body_pitch_rate_SET(1.1846234E38F) ;
        p83.type_mask_SET((char)74) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.vz_GET() == 1.3355991E38F);
            assert(pack.afz_GET() == 2.4170205E38F);
            assert(pack.vx_GET() == -3.3790255E38F);
            assert(pack.vy_GET() == 1.1448795E38F);
            assert(pack.type_mask_GET() == (char)6868);
            assert(pack.z_GET() == 9.246183E37F);
            assert(pack.afy_GET() == -1.0584476E38F);
            assert(pack.afx_GET() == 1.3204069E38F);
            assert(pack.target_system_GET() == (char)243);
            assert(pack.yaw_GET() == 2.0570807E38F);
            assert(pack.y_GET() == -1.9943888E38F);
            assert(pack.x_GET() == 2.407692E37F);
            assert(pack.time_boot_ms_GET() == 938896303L);
            assert(pack.target_component_GET() == (char)104);
            assert(pack.yaw_rate_GET() == 2.5235804E38F);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.vx_SET(-3.3790255E38F) ;
        p84.time_boot_ms_SET(938896303L) ;
        p84.type_mask_SET((char)6868) ;
        p84.afz_SET(2.4170205E38F) ;
        p84.vy_SET(1.1448795E38F) ;
        p84.yaw_rate_SET(2.5235804E38F) ;
        p84.target_component_SET((char)104) ;
        p84.vz_SET(1.3355991E38F) ;
        p84.x_SET(2.407692E37F) ;
        p84.afy_SET(-1.0584476E38F) ;
        p84.afx_SET(1.3204069E38F) ;
        p84.z_SET(9.246183E37F) ;
        p84.target_system_SET((char)243) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p84.yaw_SET(2.0570807E38F) ;
        p84.y_SET(-1.9943888E38F) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == -1.0892614E38F);
            assert(pack.alt_GET() == -1.6919368E38F);
            assert(pack.vy_GET() == -3.1060837E38F);
            assert(pack.afz_GET() == -1.7825741E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.time_boot_ms_GET() == 4042044727L);
            assert(pack.target_component_GET() == (char)13);
            assert(pack.vx_GET() == -3.0910437E38F);
            assert(pack.type_mask_GET() == (char)62068);
            assert(pack.afy_GET() == 5.636562E36F);
            assert(pack.yaw_rate_GET() == 1.7936057E38F);
            assert(pack.yaw_GET() == -7.3572387E37F);
            assert(pack.target_system_GET() == (char)79);
            assert(pack.lat_int_GET() == -114033559);
            assert(pack.lon_int_GET() == 1482797622);
            assert(pack.afx_GET() == 3.259292E38F);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.vz_SET(-1.0892614E38F) ;
        p86.target_system_SET((char)79) ;
        p86.yaw_rate_SET(1.7936057E38F) ;
        p86.time_boot_ms_SET(4042044727L) ;
        p86.type_mask_SET((char)62068) ;
        p86.afz_SET(-1.7825741E37F) ;
        p86.alt_SET(-1.6919368E38F) ;
        p86.vx_SET(-3.0910437E38F) ;
        p86.target_component_SET((char)13) ;
        p86.lon_int_SET(1482797622) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p86.afy_SET(5.636562E36F) ;
        p86.lat_int_SET(-114033559) ;
        p86.yaw_SET(-7.3572387E37F) ;
        p86.afx_SET(3.259292E38F) ;
        p86.vy_SET(-3.1060837E38F) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.afz_GET() == 3.0519029E38F);
            assert(pack.afy_GET() == 7.90172E37F);
            assert(pack.vx_GET() == 6.2311076E37F);
            assert(pack.alt_GET() == -1.4261293E38F);
            assert(pack.afx_GET() == 1.7032062E38F);
            assert(pack.lat_int_GET() == -1181606286);
            assert(pack.type_mask_GET() == (char)54694);
            assert(pack.vz_GET() == -5.6336467E37F);
            assert(pack.vy_GET() == 3.285717E37F);
            assert(pack.yaw_GET() == 1.836996E38F);
            assert(pack.yaw_rate_GET() == -2.9197123E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.lon_int_GET() == 56237256);
            assert(pack.time_boot_ms_GET() == 2318857491L);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.yaw_rate_SET(-2.9197123E38F) ;
        p87.afy_SET(7.90172E37F) ;
        p87.lon_int_SET(56237256) ;
        p87.type_mask_SET((char)54694) ;
        p87.vy_SET(3.285717E37F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p87.lat_int_SET(-1181606286) ;
        p87.afx_SET(1.7032062E38F) ;
        p87.afz_SET(3.0519029E38F) ;
        p87.alt_SET(-1.4261293E38F) ;
        p87.time_boot_ms_SET(2318857491L) ;
        p87.vz_SET(-5.6336467E37F) ;
        p87.yaw_SET(1.836996E38F) ;
        p87.vx_SET(6.2311076E37F) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 3.0366114E38F);
            assert(pack.z_GET() == -3.1626189E38F);
            assert(pack.y_GET() == -1.2105455E38F);
            assert(pack.time_boot_ms_GET() == 2688567336L);
            assert(pack.yaw_GET() == 1.7106671E38F);
            assert(pack.roll_GET() == -2.6737654E38F);
            assert(pack.x_GET() == -1.880166E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.yaw_SET(1.7106671E38F) ;
        p89.pitch_SET(3.0366114E38F) ;
        p89.z_SET(-3.1626189E38F) ;
        p89.roll_SET(-2.6737654E38F) ;
        p89.time_boot_ms_SET(2688567336L) ;
        p89.y_SET(-1.2105455E38F) ;
        p89.x_SET(-1.880166E38F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 9.324308E37F);
            assert(pack.vy_GET() == (short)10551);
            assert(pack.yacc_GET() == (short)20221);
            assert(pack.yaw_GET() == 3.0647317E38F);
            assert(pack.xacc_GET() == (short) -31771);
            assert(pack.vx_GET() == (short) -16467);
            assert(pack.pitch_GET() == 7.649784E37F);
            assert(pack.zacc_GET() == (short)16447);
            assert(pack.alt_GET() == 1782784380);
            assert(pack.pitchspeed_GET() == -2.3823995E37F);
            assert(pack.roll_GET() == -1.2206759E38F);
            assert(pack.rollspeed_GET() == 2.50722E38F);
            assert(pack.vz_GET() == (short)2659);
            assert(pack.time_usec_GET() == 9157433196386858191L);
            assert(pack.lat_GET() == 515663512);
            assert(pack.lon_GET() == 2133260660);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.time_usec_SET(9157433196386858191L) ;
        p90.pitchspeed_SET(-2.3823995E37F) ;
        p90.lon_SET(2133260660) ;
        p90.rollspeed_SET(2.50722E38F) ;
        p90.pitch_SET(7.649784E37F) ;
        p90.lat_SET(515663512) ;
        p90.yawspeed_SET(9.324308E37F) ;
        p90.zacc_SET((short)16447) ;
        p90.yaw_SET(3.0647317E38F) ;
        p90.vx_SET((short) -16467) ;
        p90.vz_SET((short)2659) ;
        p90.yacc_SET((short)20221) ;
        p90.roll_SET(-1.2206759E38F) ;
        p90.alt_SET(1782784380) ;
        p90.xacc_SET((short) -31771) ;
        p90.vy_SET((short)10551) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux2_GET() == -9.373904E37F);
            assert(pack.aux1_GET() == 1.2697163E38F);
            assert(pack.throttle_GET() == 1.4440809E38F);
            assert(pack.aux4_GET() == -3.0357853E38F);
            assert(pack.pitch_elevator_GET() == 8.2765283E37F);
            assert(pack.time_usec_GET() == 1358438998591670668L);
            assert(pack.yaw_rudder_GET() == 3.2856175E38F);
            assert(pack.nav_mode_GET() == (char)148);
            assert(pack.roll_ailerons_GET() == -2.6294294E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_DISARMED);
            assert(pack.aux3_GET() == 1.4897847E37F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED) ;
        p91.nav_mode_SET((char)148) ;
        p91.time_usec_SET(1358438998591670668L) ;
        p91.yaw_rudder_SET(3.2856175E38F) ;
        p91.aux1_SET(1.2697163E38F) ;
        p91.aux4_SET(-3.0357853E38F) ;
        p91.roll_ailerons_SET(-2.6294294E38F) ;
        p91.throttle_SET(1.4440809E38F) ;
        p91.pitch_elevator_SET(8.2765283E37F) ;
        p91.aux2_SET(-9.373904E37F) ;
        p91.aux3_SET(1.4897847E37F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan9_raw_GET() == (char)31366);
            assert(pack.chan3_raw_GET() == (char)27209);
            assert(pack.chan1_raw_GET() == (char)39506);
            assert(pack.chan10_raw_GET() == (char)18621);
            assert(pack.chan4_raw_GET() == (char)28145);
            assert(pack.chan5_raw_GET() == (char)64213);
            assert(pack.time_usec_GET() == 5720890907321733375L);
            assert(pack.chan8_raw_GET() == (char)1489);
            assert(pack.chan2_raw_GET() == (char)19783);
            assert(pack.chan12_raw_GET() == (char)30569);
            assert(pack.chan6_raw_GET() == (char)63077);
            assert(pack.rssi_GET() == (char)17);
            assert(pack.chan11_raw_GET() == (char)22371);
            assert(pack.chan7_raw_GET() == (char)53209);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan5_raw_SET((char)64213) ;
        p92.chan2_raw_SET((char)19783) ;
        p92.chan7_raw_SET((char)53209) ;
        p92.chan10_raw_SET((char)18621) ;
        p92.chan9_raw_SET((char)31366) ;
        p92.chan8_raw_SET((char)1489) ;
        p92.chan12_raw_SET((char)30569) ;
        p92.chan1_raw_SET((char)39506) ;
        p92.chan4_raw_SET((char)28145) ;
        p92.time_usec_SET(5720890907321733375L) ;
        p92.chan3_raw_SET((char)27209) ;
        p92.chan11_raw_SET((char)22371) ;
        p92.rssi_SET((char)17) ;
        p92.chan6_raw_SET((char)63077) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == 7514560153695277415L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_DISARMED);
            assert(pack.time_usec_GET() == 8724667290361997467L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-8.0875104E37F, -3.2505732E38F, 1.8460536E38F, -1.3212677E38F, 2.1182125E38F, 1.4453311E38F, -2.0948358E38F, -3.2830664E38F, -2.7069922E38F, -2.790697E38F, -2.5413176E38F, 3.8214202E36F, 1.03489506E37F, 3.021498E37F, -1.7984855E38F, -2.9934918E38F}));
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED) ;
        p93.flags_SET(7514560153695277415L) ;
        p93.controls_SET(new float[] {-8.0875104E37F, -3.2505732E38F, 1.8460536E38F, -1.3212677E38F, 2.1182125E38F, 1.4453311E38F, -2.0948358E38F, -3.2830664E38F, -2.7069922E38F, -2.790697E38F, -2.5413176E38F, 3.8214202E36F, 1.03489506E37F, 3.021498E37F, -1.7984855E38F, -2.9934918E38F}, 0) ;
        p93.time_usec_SET(8724667290361997467L) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.ground_distance_GET() == 2.295661E38F);
            assert(pack.flow_x_GET() == (short)13082);
            assert(pack.time_usec_GET() == 7448407993557279045L);
            assert(pack.flow_y_GET() == (short)3630);
            assert(pack.flow_rate_y_TRY(ph) == 3.2827482E38F);
            assert(pack.flow_comp_m_x_GET() == -3.0279127E38F);
            assert(pack.flow_rate_x_TRY(ph) == 2.1769538E38F);
            assert(pack.sensor_id_GET() == (char)174);
            assert(pack.quality_GET() == (char)125);
            assert(pack.flow_comp_m_y_GET() == 9.9582125E36F);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(7448407993557279045L) ;
        p100.sensor_id_SET((char)174) ;
        p100.quality_SET((char)125) ;
        p100.flow_x_SET((short)13082) ;
        p100.flow_y_SET((short)3630) ;
        p100.ground_distance_SET(2.295661E38F) ;
        p100.flow_rate_y_SET(3.2827482E38F, PH) ;
        p100.flow_comp_m_y_SET(9.9582125E36F) ;
        p100.flow_rate_x_SET(2.1769538E38F, PH) ;
        p100.flow_comp_m_x_SET(-3.0279127E38F) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -9.131515E37F);
            assert(pack.x_GET() == -2.3442249E38F);
            assert(pack.pitch_GET() == -1.4961155E38F);
            assert(pack.roll_GET() == 5.1596614E37F);
            assert(pack.z_GET() == -1.4618902E38F);
            assert(pack.y_GET() == -2.0205115E38F);
            assert(pack.usec_GET() == 5215830192194546971L);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.roll_SET(5.1596614E37F) ;
        p101.yaw_SET(-9.131515E37F) ;
        p101.pitch_SET(-1.4961155E38F) ;
        p101.x_SET(-2.3442249E38F) ;
        p101.usec_SET(5215830192194546971L) ;
        p101.z_SET(-1.4618902E38F) ;
        p101.y_SET(-2.0205115E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -2.6012943E38F);
            assert(pack.x_GET() == -2.1593193E37F);
            assert(pack.yaw_GET() == -2.8259733E38F);
            assert(pack.roll_GET() == 2.878385E38F);
            assert(pack.usec_GET() == 1669550999854988630L);
            assert(pack.y_GET() == 2.4595498E38F);
            assert(pack.z_GET() == 2.2384442E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.z_SET(2.2384442E38F) ;
        p102.roll_SET(2.878385E38F) ;
        p102.pitch_SET(-2.6012943E38F) ;
        p102.y_SET(2.4595498E38F) ;
        p102.yaw_SET(-2.8259733E38F) ;
        p102.usec_SET(1669550999854988630L) ;
        p102.x_SET(-2.1593193E37F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 2.9400724E38F);
            assert(pack.x_GET() == -2.0765058E38F);
            assert(pack.usec_GET() == 7035767407969267948L);
            assert(pack.z_GET() == 2.1262157E38F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.y_SET(2.9400724E38F) ;
        p103.z_SET(2.1262157E38F) ;
        p103.usec_SET(7035767407969267948L) ;
        p103.x_SET(-2.0765058E38F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -1.0457382E38F);
            assert(pack.z_GET() == -7.432624E37F);
            assert(pack.yaw_GET() == -3.1414564E38F);
            assert(pack.x_GET() == -1.5624965E38F);
            assert(pack.usec_GET() == 7491910421878831915L);
            assert(pack.y_GET() == -8.2542536E37F);
            assert(pack.pitch_GET() == 1.9683628E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(7491910421878831915L) ;
        p104.x_SET(-1.5624965E38F) ;
        p104.pitch_SET(1.9683628E38F) ;
        p104.yaw_SET(-3.1414564E38F) ;
        p104.y_SET(-8.2542536E37F) ;
        p104.z_SET(-7.432624E37F) ;
        p104.roll_SET(-1.0457382E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == -1.9734359E38F);
            assert(pack.xgyro_GET() == -1.6569857E38F);
            assert(pack.pressure_alt_GET() == -7.969519E37F);
            assert(pack.diff_pressure_GET() == 6.931701E37F);
            assert(pack.ygyro_GET() == 4.5530577E37F);
            assert(pack.xacc_GET() == 1.1563509E38F);
            assert(pack.zacc_GET() == -1.4270756E38F);
            assert(pack.fields_updated_GET() == (char)57);
            assert(pack.ymag_GET() == -1.0241989E38F);
            assert(pack.time_usec_GET() == 8528603395931460018L);
            assert(pack.abs_pressure_GET() == -1.8849709E38F);
            assert(pack.zgyro_GET() == 1.494138E38F);
            assert(pack.zmag_GET() == 1.4756433E38F);
            assert(pack.xmag_GET() == 3.0891373E38F);
            assert(pack.yacc_GET() == -1.8509648E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.zacc_SET(-1.4270756E38F) ;
        p105.pressure_alt_SET(-7.969519E37F) ;
        p105.ygyro_SET(4.5530577E37F) ;
        p105.zgyro_SET(1.494138E38F) ;
        p105.xmag_SET(3.0891373E38F) ;
        p105.diff_pressure_SET(6.931701E37F) ;
        p105.fields_updated_SET((char)57) ;
        p105.xgyro_SET(-1.6569857E38F) ;
        p105.ymag_SET(-1.0241989E38F) ;
        p105.time_usec_SET(8528603395931460018L) ;
        p105.temperature_SET(-1.9734359E38F) ;
        p105.yacc_SET(-1.8509648E38F) ;
        p105.zmag_SET(1.4756433E38F) ;
        p105.abs_pressure_SET(-1.8849709E38F) ;
        p105.xacc_SET(1.1563509E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_xgyro_GET() == -3.1523104E38F);
            assert(pack.distance_GET() == -3.5515428E36F);
            assert(pack.integrated_x_GET() == 1.0720272E38F);
            assert(pack.temperature_GET() == (short)8502);
            assert(pack.sensor_id_GET() == (char)47);
            assert(pack.quality_GET() == (char)69);
            assert(pack.integrated_y_GET() == 1.6774629E37F);
            assert(pack.integrated_zgyro_GET() == 2.1091706E38F);
            assert(pack.time_delta_distance_us_GET() == 1445805402L);
            assert(pack.time_usec_GET() == 8330327106770786951L);
            assert(pack.integration_time_us_GET() == 2028673786L);
            assert(pack.integrated_ygyro_GET() == -5.760798E37F);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(8330327106770786951L) ;
        p106.temperature_SET((short)8502) ;
        p106.time_delta_distance_us_SET(1445805402L) ;
        p106.integrated_y_SET(1.6774629E37F) ;
        p106.integrated_x_SET(1.0720272E38F) ;
        p106.integration_time_us_SET(2028673786L) ;
        p106.integrated_xgyro_SET(-3.1523104E38F) ;
        p106.integrated_ygyro_SET(-5.760798E37F) ;
        p106.sensor_id_SET((char)47) ;
        p106.integrated_zgyro_SET(2.1091706E38F) ;
        p106.distance_SET(-3.5515428E36F) ;
        p106.quality_SET((char)69) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.xgyro_GET() == 2.3247712E38F);
            assert(pack.abs_pressure_GET() == 2.1453056E38F);
            assert(pack.ymag_GET() == 2.5635004E38F);
            assert(pack.temperature_GET() == 1.2463093E38F);
            assert(pack.time_usec_GET() == 5233699439936461967L);
            assert(pack.zgyro_GET() == -2.192513E38F);
            assert(pack.ygyro_GET() == -3.0648354E38F);
            assert(pack.zmag_GET() == -5.115903E37F);
            assert(pack.fields_updated_GET() == 227428790L);
            assert(pack.zacc_GET() == -1.7258667E38F);
            assert(pack.pressure_alt_GET() == -1.5720365E38F);
            assert(pack.yacc_GET() == -1.5033202E38F);
            assert(pack.xmag_GET() == -4.748265E35F);
            assert(pack.xacc_GET() == -1.2699863E38F);
            assert(pack.diff_pressure_GET() == 5.1599855E37F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.zmag_SET(-5.115903E37F) ;
        p107.ymag_SET(2.5635004E38F) ;
        p107.fields_updated_SET(227428790L) ;
        p107.pressure_alt_SET(-1.5720365E38F) ;
        p107.abs_pressure_SET(2.1453056E38F) ;
        p107.zacc_SET(-1.7258667E38F) ;
        p107.yacc_SET(-1.5033202E38F) ;
        p107.xgyro_SET(2.3247712E38F) ;
        p107.zgyro_SET(-2.192513E38F) ;
        p107.temperature_SET(1.2463093E38F) ;
        p107.diff_pressure_SET(5.1599855E37F) ;
        p107.ygyro_SET(-3.0648354E38F) ;
        p107.xmag_SET(-4.748265E35F) ;
        p107.xacc_SET(-1.2699863E38F) ;
        p107.time_usec_SET(5233699439936461967L) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.std_dev_vert_GET() == 8.0523047E37F);
            assert(pack.lon_GET() == 6.8609074E37F);
            assert(pack.zacc_GET() == -8.0204613E37F);
            assert(pack.vn_GET() == 1.0192356E38F);
            assert(pack.yacc_GET() == -2.9896136E38F);
            assert(pack.zgyro_GET() == 1.0486326E38F);
            assert(pack.ve_GET() == -2.7978523E38F);
            assert(pack.q3_GET() == 1.5400943E38F);
            assert(pack.q2_GET() == 2.343645E37F);
            assert(pack.yaw_GET() == -2.846403E38F);
            assert(pack.std_dev_horz_GET() == -7.127488E36F);
            assert(pack.alt_GET() == -9.598004E37F);
            assert(pack.pitch_GET() == -6.0534347E37F);
            assert(pack.roll_GET() == 1.2282653E38F);
            assert(pack.vd_GET() == -1.1912743E38F);
            assert(pack.q4_GET() == 4.8204863E37F);
            assert(pack.lat_GET() == -3.359529E38F);
            assert(pack.ygyro_GET() == -3.1656221E38F);
            assert(pack.xgyro_GET() == 3.1571844E38F);
            assert(pack.xacc_GET() == -2.218231E38F);
            assert(pack.q1_GET() == 1.8035715E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.yacc_SET(-2.9896136E38F) ;
        p108.alt_SET(-9.598004E37F) ;
        p108.q2_SET(2.343645E37F) ;
        p108.std_dev_horz_SET(-7.127488E36F) ;
        p108.lat_SET(-3.359529E38F) ;
        p108.q3_SET(1.5400943E38F) ;
        p108.ve_SET(-2.7978523E38F) ;
        p108.yaw_SET(-2.846403E38F) ;
        p108.q1_SET(1.8035715E38F) ;
        p108.vn_SET(1.0192356E38F) ;
        p108.ygyro_SET(-3.1656221E38F) ;
        p108.zgyro_SET(1.0486326E38F) ;
        p108.xacc_SET(-2.218231E38F) ;
        p108.q4_SET(4.8204863E37F) ;
        p108.lon_SET(6.8609074E37F) ;
        p108.pitch_SET(-6.0534347E37F) ;
        p108.std_dev_vert_SET(8.0523047E37F) ;
        p108.vd_SET(-1.1912743E38F) ;
        p108.roll_SET(1.2282653E38F) ;
        p108.xgyro_SET(3.1571844E38F) ;
        p108.zacc_SET(-8.0204613E37F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)210);
            assert(pack.noise_GET() == (char)22);
            assert(pack.rxerrors_GET() == (char)14678);
            assert(pack.fixed__GET() == (char)31171);
            assert(pack.txbuf_GET() == (char)17);
            assert(pack.remnoise_GET() == (char)93);
            assert(pack.remrssi_GET() == (char)148);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)210) ;
        p109.remnoise_SET((char)93) ;
        p109.rxerrors_SET((char)14678) ;
        p109.remrssi_SET((char)148) ;
        p109.noise_SET((char)22) ;
        p109.txbuf_SET((char)17) ;
        p109.fixed__SET((char)31171) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)188);
            assert(pack.target_system_GET() == (char)71);
            assert(pack.target_component_GET() == (char)83);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)76, (char)100, (char)103, (char)73, (char)89, (char)168, (char)166, (char)60, (char)24, (char)20, (char)238, (char)80, (char)144, (char)99, (char)231, (char)208, (char)54, (char)245, (char)230, (char)160, (char)158, (char)197, (char)27, (char)162, (char)40, (char)62, (char)201, (char)146, (char)115, (char)205, (char)160, (char)25, (char)131, (char)73, (char)232, (char)30, (char)143, (char)71, (char)60, (char)243, (char)170, (char)46, (char)97, (char)183, (char)129, (char)218, (char)53, (char)203, (char)19, (char)212, (char)167, (char)53, (char)53, (char)17, (char)122, (char)78, (char)10, (char)114, (char)229, (char)248, (char)149, (char)142, (char)29, (char)177, (char)24, (char)18, (char)177, (char)168, (char)114, (char)147, (char)123, (char)191, (char)114, (char)195, (char)161, (char)44, (char)74, (char)147, (char)34, (char)12, (char)29, (char)218, (char)120, (char)234, (char)240, (char)98, (char)189, (char)22, (char)60, (char)103, (char)59, (char)78, (char)38, (char)169, (char)121, (char)214, (char)114, (char)58, (char)175, (char)11, (char)185, (char)227, (char)57, (char)165, (char)160, (char)81, (char)85, (char)68, (char)175, (char)64, (char)76, (char)6, (char)146, (char)206, (char)0, (char)158, (char)38, (char)252, (char)126, (char)97, (char)50, (char)151, (char)8, (char)230, (char)238, (char)15, (char)157, (char)55, (char)185, (char)45, (char)237, (char)253, (char)61, (char)91, (char)198, (char)60, (char)56, (char)235, (char)184, (char)207, (char)233, (char)14, (char)43, (char)71, (char)163, (char)181, (char)10, (char)29, (char)160, (char)72, (char)32, (char)142, (char)9, (char)143, (char)117, (char)234, (char)133, (char)200, (char)195, (char)20, (char)142, (char)243, (char)5, (char)23, (char)179, (char)189, (char)106, (char)214, (char)18, (char)179, (char)68, (char)23, (char)151, (char)27, (char)242, (char)157, (char)146, (char)28, (char)150, (char)1, (char)227, (char)208, (char)233, (char)38, (char)215, (char)148, (char)138, (char)22, (char)78, (char)218, (char)62, (char)245, (char)199, (char)129, (char)84, (char)141, (char)130, (char)108, (char)90, (char)251, (char)86, (char)128, (char)125, (char)162, (char)69, (char)178, (char)27, (char)186, (char)10, (char)45, (char)252, (char)110, (char)124, (char)95, (char)55, (char)164, (char)227, (char)118, (char)137, (char)116, (char)158, (char)38, (char)119, (char)206, (char)131, (char)163, (char)119, (char)27, (char)177, (char)25, (char)247, (char)122, (char)238, (char)131, (char)53, (char)211, (char)170, (char)131, (char)92, (char)218, (char)191, (char)132, (char)100, (char)144, (char)86, (char)154, (char)191, (char)226, (char)48, (char)241, (char)1}));
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)188) ;
        p110.target_system_SET((char)71) ;
        p110.target_component_SET((char)83) ;
        p110.payload_SET(new char[] {(char)76, (char)100, (char)103, (char)73, (char)89, (char)168, (char)166, (char)60, (char)24, (char)20, (char)238, (char)80, (char)144, (char)99, (char)231, (char)208, (char)54, (char)245, (char)230, (char)160, (char)158, (char)197, (char)27, (char)162, (char)40, (char)62, (char)201, (char)146, (char)115, (char)205, (char)160, (char)25, (char)131, (char)73, (char)232, (char)30, (char)143, (char)71, (char)60, (char)243, (char)170, (char)46, (char)97, (char)183, (char)129, (char)218, (char)53, (char)203, (char)19, (char)212, (char)167, (char)53, (char)53, (char)17, (char)122, (char)78, (char)10, (char)114, (char)229, (char)248, (char)149, (char)142, (char)29, (char)177, (char)24, (char)18, (char)177, (char)168, (char)114, (char)147, (char)123, (char)191, (char)114, (char)195, (char)161, (char)44, (char)74, (char)147, (char)34, (char)12, (char)29, (char)218, (char)120, (char)234, (char)240, (char)98, (char)189, (char)22, (char)60, (char)103, (char)59, (char)78, (char)38, (char)169, (char)121, (char)214, (char)114, (char)58, (char)175, (char)11, (char)185, (char)227, (char)57, (char)165, (char)160, (char)81, (char)85, (char)68, (char)175, (char)64, (char)76, (char)6, (char)146, (char)206, (char)0, (char)158, (char)38, (char)252, (char)126, (char)97, (char)50, (char)151, (char)8, (char)230, (char)238, (char)15, (char)157, (char)55, (char)185, (char)45, (char)237, (char)253, (char)61, (char)91, (char)198, (char)60, (char)56, (char)235, (char)184, (char)207, (char)233, (char)14, (char)43, (char)71, (char)163, (char)181, (char)10, (char)29, (char)160, (char)72, (char)32, (char)142, (char)9, (char)143, (char)117, (char)234, (char)133, (char)200, (char)195, (char)20, (char)142, (char)243, (char)5, (char)23, (char)179, (char)189, (char)106, (char)214, (char)18, (char)179, (char)68, (char)23, (char)151, (char)27, (char)242, (char)157, (char)146, (char)28, (char)150, (char)1, (char)227, (char)208, (char)233, (char)38, (char)215, (char)148, (char)138, (char)22, (char)78, (char)218, (char)62, (char)245, (char)199, (char)129, (char)84, (char)141, (char)130, (char)108, (char)90, (char)251, (char)86, (char)128, (char)125, (char)162, (char)69, (char)178, (char)27, (char)186, (char)10, (char)45, (char)252, (char)110, (char)124, (char)95, (char)55, (char)164, (char)227, (char)118, (char)137, (char)116, (char)158, (char)38, (char)119, (char)206, (char)131, (char)163, (char)119, (char)27, (char)177, (char)25, (char)247, (char)122, (char)238, (char)131, (char)53, (char)211, (char)170, (char)131, (char)92, (char)218, (char)191, (char)132, (char)100, (char)144, (char)86, (char)154, (char)191, (char)226, (char)48, (char)241, (char)1}, 0) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -7333016818544497616L);
            assert(pack.ts1_GET() == -457233222378171498L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(-457233222378171498L) ;
        p111.tc1_SET(-7333016818544497616L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 605673627L);
            assert(pack.time_usec_GET() == 8993136367533486604L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(8993136367533486604L) ;
        p112.seq_SET(605673627L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.ve_GET() == (short) -9119);
            assert(pack.time_usec_GET() == 4376063117923479645L);
            assert(pack.vel_GET() == (char)12106);
            assert(pack.satellites_visible_GET() == (char)243);
            assert(pack.vn_GET() == (short)27183);
            assert(pack.eph_GET() == (char)484);
            assert(pack.cog_GET() == (char)3246);
            assert(pack.vd_GET() == (short) -17659);
            assert(pack.alt_GET() == -815993937);
            assert(pack.lat_GET() == -1148052637);
            assert(pack.epv_GET() == (char)37744);
            assert(pack.fix_type_GET() == (char)67);
            assert(pack.lon_GET() == 569591979);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.lat_SET(-1148052637) ;
        p113.time_usec_SET(4376063117923479645L) ;
        p113.eph_SET((char)484) ;
        p113.satellites_visible_SET((char)243) ;
        p113.fix_type_SET((char)67) ;
        p113.cog_SET((char)3246) ;
        p113.vn_SET((short)27183) ;
        p113.epv_SET((char)37744) ;
        p113.lon_SET(569591979) ;
        p113.vd_SET((short) -17659) ;
        p113.alt_SET(-815993937) ;
        p113.ve_SET((short) -9119) ;
        p113.vel_SET((char)12106) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_ygyro_GET() == 2.2238689E38F);
            assert(pack.sensor_id_GET() == (char)165);
            assert(pack.quality_GET() == (char)42);
            assert(pack.time_delta_distance_us_GET() == 3638875080L);
            assert(pack.integrated_y_GET() == -1.9161275E38F);
            assert(pack.integration_time_us_GET() == 861043628L);
            assert(pack.temperature_GET() == (short)16458);
            assert(pack.integrated_xgyro_GET() == -2.48283E38F);
            assert(pack.distance_GET() == -2.0753552E38F);
            assert(pack.integrated_x_GET() == 1.0701596E38F);
            assert(pack.integrated_zgyro_GET() == -2.83352E38F);
            assert(pack.time_usec_GET() == 7898809717905198504L);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_x_SET(1.0701596E38F) ;
        p114.sensor_id_SET((char)165) ;
        p114.quality_SET((char)42) ;
        p114.integrated_xgyro_SET(-2.48283E38F) ;
        p114.time_usec_SET(7898809717905198504L) ;
        p114.integrated_zgyro_SET(-2.83352E38F) ;
        p114.temperature_SET((short)16458) ;
        p114.integration_time_us_SET(861043628L) ;
        p114.integrated_ygyro_SET(2.2238689E38F) ;
        p114.time_delta_distance_us_SET(3638875080L) ;
        p114.integrated_y_SET(-1.9161275E38F) ;
        p114.distance_SET(-2.0753552E38F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 635979250);
            assert(pack.rollspeed_GET() == -4.981605E37F);
            assert(pack.alt_GET() == 1805414399);
            assert(pack.xacc_GET() == (short)25492);
            assert(pack.pitchspeed_GET() == -1.7583104E35F);
            assert(pack.ind_airspeed_GET() == (char)46834);
            assert(pack.zacc_GET() == (short) -7061);
            assert(pack.true_airspeed_GET() == (char)23880);
            assert(pack.lat_GET() == -1144262403);
            assert(pack.yawspeed_GET() == 1.3869418E38F);
            assert(pack.yacc_GET() == (short) -7492);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {7.748289E37F, -1.6640501E38F, -2.3004496E38F, -3.2343117E38F}));
            assert(pack.vy_GET() == (short)27202);
            assert(pack.vx_GET() == (short) -9698);
            assert(pack.time_usec_GET() == 1217904781986482969L);
            assert(pack.vz_GET() == (short)12693);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.zacc_SET((short) -7061) ;
        p115.vx_SET((short) -9698) ;
        p115.true_airspeed_SET((char)23880) ;
        p115.lon_SET(635979250) ;
        p115.lat_SET(-1144262403) ;
        p115.alt_SET(1805414399) ;
        p115.time_usec_SET(1217904781986482969L) ;
        p115.yawspeed_SET(1.3869418E38F) ;
        p115.attitude_quaternion_SET(new float[] {7.748289E37F, -1.6640501E38F, -2.3004496E38F, -3.2343117E38F}, 0) ;
        p115.ind_airspeed_SET((char)46834) ;
        p115.vz_SET((short)12693) ;
        p115.pitchspeed_SET(-1.7583104E35F) ;
        p115.vy_SET((short)27202) ;
        p115.yacc_SET((short) -7492) ;
        p115.rollspeed_SET(-4.981605E37F) ;
        p115.xacc_SET((short)25492) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short) -30596);
            assert(pack.yacc_GET() == (short) -14199);
            assert(pack.time_boot_ms_GET() == 1828440056L);
            assert(pack.zmag_GET() == (short) -26561);
            assert(pack.zgyro_GET() == (short) -12805);
            assert(pack.xacc_GET() == (short)15378);
            assert(pack.ymag_GET() == (short) -20259);
            assert(pack.xmag_GET() == (short)7743);
            assert(pack.xgyro_GET() == (short)23040);
            assert(pack.ygyro_GET() == (short)24062);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.xmag_SET((short)7743) ;
        p116.zacc_SET((short) -30596) ;
        p116.ymag_SET((short) -20259) ;
        p116.yacc_SET((short) -14199) ;
        p116.time_boot_ms_SET(1828440056L) ;
        p116.xgyro_SET((short)23040) ;
        p116.zmag_SET((short) -26561) ;
        p116.xacc_SET((short)15378) ;
        p116.zgyro_SET((short) -12805) ;
        p116.ygyro_SET((short)24062) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)35);
            assert(pack.target_component_GET() == (char)177);
            assert(pack.start_GET() == (char)32025);
            assert(pack.end_GET() == (char)28636);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)35) ;
        p117.target_component_SET((char)177) ;
        p117.start_SET((char)32025) ;
        p117.end_SET((char)28636) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)49639);
            assert(pack.size_GET() == 3325082159L);
            assert(pack.num_logs_GET() == (char)46718);
            assert(pack.time_utc_GET() == 2421786499L);
            assert(pack.last_log_num_GET() == (char)49572);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.last_log_num_SET((char)49572) ;
        p118.num_logs_SET((char)46718) ;
        p118.id_SET((char)49639) ;
        p118.size_SET(3325082159L) ;
        p118.time_utc_SET(2421786499L) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)166);
            assert(pack.ofs_GET() == 2525816790L);
            assert(pack.count_GET() == 3759622593L);
            assert(pack.id_GET() == (char)8888);
            assert(pack.target_system_GET() == (char)83);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)83) ;
        p119.ofs_SET(2525816790L) ;
        p119.id_SET((char)8888) ;
        p119.count_SET(3759622593L) ;
        p119.target_component_SET((char)166) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)48344);
            assert(pack.ofs_GET() == 3917178249L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)87, (char)53, (char)13, (char)238, (char)184, (char)118, (char)255, (char)180, (char)2, (char)167, (char)18, (char)107, (char)54, (char)242, (char)172, (char)220, (char)11, (char)168, (char)54, (char)1, (char)195, (char)230, (char)31, (char)171, (char)163, (char)61, (char)158, (char)68, (char)179, (char)1, (char)88, (char)27, (char)103, (char)136, (char)123, (char)156, (char)15, (char)233, (char)119, (char)213, (char)210, (char)225, (char)64, (char)68, (char)3, (char)249, (char)237, (char)89, (char)101, (char)77, (char)110, (char)78, (char)70, (char)251, (char)233, (char)36, (char)127, (char)118, (char)115, (char)241, (char)130, (char)45, (char)32, (char)15, (char)235, (char)248, (char)169, (char)116, (char)236, (char)225, (char)161, (char)60, (char)223, (char)85, (char)193, (char)4, (char)68, (char)57, (char)208, (char)61, (char)203, (char)231, (char)159, (char)230, (char)182, (char)43, (char)171, (char)59, (char)66, (char)164}));
            assert(pack.count_GET() == (char)205);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.count_SET((char)205) ;
        p120.data__SET(new char[] {(char)87, (char)53, (char)13, (char)238, (char)184, (char)118, (char)255, (char)180, (char)2, (char)167, (char)18, (char)107, (char)54, (char)242, (char)172, (char)220, (char)11, (char)168, (char)54, (char)1, (char)195, (char)230, (char)31, (char)171, (char)163, (char)61, (char)158, (char)68, (char)179, (char)1, (char)88, (char)27, (char)103, (char)136, (char)123, (char)156, (char)15, (char)233, (char)119, (char)213, (char)210, (char)225, (char)64, (char)68, (char)3, (char)249, (char)237, (char)89, (char)101, (char)77, (char)110, (char)78, (char)70, (char)251, (char)233, (char)36, (char)127, (char)118, (char)115, (char)241, (char)130, (char)45, (char)32, (char)15, (char)235, (char)248, (char)169, (char)116, (char)236, (char)225, (char)161, (char)60, (char)223, (char)85, (char)193, (char)4, (char)68, (char)57, (char)208, (char)61, (char)203, (char)231, (char)159, (char)230, (char)182, (char)43, (char)171, (char)59, (char)66, (char)164}, 0) ;
        p120.id_SET((char)48344) ;
        p120.ofs_SET(3917178249L) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)27);
            assert(pack.target_system_GET() == (char)148);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)148) ;
        p121.target_component_SET((char)27) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)109);
            assert(pack.target_component_GET() == (char)118);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)109) ;
        p122.target_component_SET((char)118) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)62);
            assert(pack.len_GET() == (char)93);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)158, (char)100, (char)158, (char)17, (char)85, (char)141, (char)11, (char)42, (char)10, (char)243, (char)151, (char)233, (char)251, (char)95, (char)116, (char)3, (char)206, (char)150, (char)131, (char)36, (char)170, (char)251, (char)205, (char)14, (char)23, (char)173, (char)181, (char)184, (char)208, (char)45, (char)71, (char)171, (char)158, (char)129, (char)212, (char)0, (char)105, (char)67, (char)174, (char)197, (char)187, (char)153, (char)80, (char)149, (char)249, (char)28, (char)207, (char)96, (char)246, (char)197, (char)138, (char)68, (char)225, (char)10, (char)164, (char)44, (char)29, (char)141, (char)105, (char)186, (char)251, (char)204, (char)221, (char)254, (char)83, (char)135, (char)106, (char)130, (char)198, (char)233, (char)210, (char)210, (char)182, (char)42, (char)67, (char)228, (char)94, (char)101, (char)67, (char)235, (char)226, (char)242, (char)90, (char)35, (char)55, (char)3, (char)64, (char)28, (char)247, (char)248, (char)247, (char)206, (char)218, (char)182, (char)123, (char)137, (char)127, (char)92, (char)186, (char)35, (char)153, (char)119, (char)115, (char)125, (char)52, (char)140, (char)175, (char)153, (char)92, (char)198}));
            assert(pack.target_system_GET() == (char)120);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.data__SET(new char[] {(char)158, (char)100, (char)158, (char)17, (char)85, (char)141, (char)11, (char)42, (char)10, (char)243, (char)151, (char)233, (char)251, (char)95, (char)116, (char)3, (char)206, (char)150, (char)131, (char)36, (char)170, (char)251, (char)205, (char)14, (char)23, (char)173, (char)181, (char)184, (char)208, (char)45, (char)71, (char)171, (char)158, (char)129, (char)212, (char)0, (char)105, (char)67, (char)174, (char)197, (char)187, (char)153, (char)80, (char)149, (char)249, (char)28, (char)207, (char)96, (char)246, (char)197, (char)138, (char)68, (char)225, (char)10, (char)164, (char)44, (char)29, (char)141, (char)105, (char)186, (char)251, (char)204, (char)221, (char)254, (char)83, (char)135, (char)106, (char)130, (char)198, (char)233, (char)210, (char)210, (char)182, (char)42, (char)67, (char)228, (char)94, (char)101, (char)67, (char)235, (char)226, (char)242, (char)90, (char)35, (char)55, (char)3, (char)64, (char)28, (char)247, (char)248, (char)247, (char)206, (char)218, (char)182, (char)123, (char)137, (char)127, (char)92, (char)186, (char)35, (char)153, (char)119, (char)115, (char)125, (char)52, (char)140, (char)175, (char)153, (char)92, (char)198}, 0) ;
        p123.target_system_SET((char)120) ;
        p123.len_SET((char)93) ;
        p123.target_component_SET((char)62) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -85737310);
            assert(pack.satellites_visible_GET() == (char)58);
            assert(pack.vel_GET() == (char)45545);
            assert(pack.epv_GET() == (char)24890);
            assert(pack.dgps_age_GET() == 2525328132L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
            assert(pack.time_usec_GET() == 2659561610932370422L);
            assert(pack.lat_GET() == -2099768399);
            assert(pack.dgps_numch_GET() == (char)123);
            assert(pack.lon_GET() == -32573002);
            assert(pack.cog_GET() == (char)47633);
            assert(pack.eph_GET() == (char)49259);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS) ;
        p124.satellites_visible_SET((char)58) ;
        p124.time_usec_SET(2659561610932370422L) ;
        p124.cog_SET((char)47633) ;
        p124.lon_SET(-32573002) ;
        p124.dgps_age_SET(2525328132L) ;
        p124.epv_SET((char)24890) ;
        p124.eph_SET((char)49259) ;
        p124.dgps_numch_SET((char)123) ;
        p124.vel_SET((char)45545) ;
        p124.lat_SET(-2099768399) ;
        p124.alt_SET(-85737310) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID));
            assert(pack.Vservo_GET() == (char)27118);
            assert(pack.Vcc_GET() == (char)1753);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)27118) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID)) ;
        p125.Vcc_SET((char)1753) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)199);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)175, (char)201, (char)209, (char)68, (char)62, (char)198, (char)207, (char)219, (char)143, (char)137, (char)29, (char)229, (char)168, (char)108, (char)29, (char)201, (char)199, (char)126, (char)54, (char)238, (char)122, (char)47, (char)68, (char)196, (char)53, (char)76, (char)215, (char)201, (char)214, (char)111, (char)119, (char)32, (char)185, (char)227, (char)115, (char)200, (char)134, (char)81, (char)242, (char)104, (char)240, (char)208, (char)189, (char)98, (char)71, (char)101, (char)121, (char)147, (char)42, (char)85, (char)133, (char)191, (char)218, (char)103, (char)229, (char)91, (char)94, (char)104, (char)172, (char)128, (char)60, (char)15, (char)250, (char)235, (char)54, (char)98, (char)154, (char)61, (char)12, (char)102}));
            assert(pack.baudrate_GET() == 3963775153L);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
            assert(pack.timeout_GET() == (char)45879);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND));
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.baudrate_SET(3963775153L) ;
        p126.data__SET(new char[] {(char)175, (char)201, (char)209, (char)68, (char)62, (char)198, (char)207, (char)219, (char)143, (char)137, (char)29, (char)229, (char)168, (char)108, (char)29, (char)201, (char)199, (char)126, (char)54, (char)238, (char)122, (char)47, (char)68, (char)196, (char)53, (char)76, (char)215, (char)201, (char)214, (char)111, (char)119, (char)32, (char)185, (char)227, (char)115, (char)200, (char)134, (char)81, (char)242, (char)104, (char)240, (char)208, (char)189, (char)98, (char)71, (char)101, (char)121, (char)147, (char)42, (char)85, (char)133, (char)191, (char)218, (char)103, (char)229, (char)91, (char)94, (char)104, (char)172, (char)128, (char)60, (char)15, (char)250, (char)235, (char)54, (char)98, (char)154, (char)61, (char)12, (char)102}, 0) ;
        p126.timeout_SET((char)45879) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND)) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2) ;
        p126.count_SET((char)199) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_c_mm_GET() == 2101939018);
            assert(pack.iar_num_hypotheses_GET() == 1410868100);
            assert(pack.baseline_coords_type_GET() == (char)3);
            assert(pack.rtk_rate_GET() == (char)57);
            assert(pack.rtk_receiver_id_GET() == (char)31);
            assert(pack.accuracy_GET() == 1691025263L);
            assert(pack.nsats_GET() == (char)199);
            assert(pack.wn_GET() == (char)48671);
            assert(pack.rtk_health_GET() == (char)239);
            assert(pack.tow_GET() == 1884239707L);
            assert(pack.baseline_a_mm_GET() == 693006751);
            assert(pack.time_last_baseline_ms_GET() == 1706663981L);
            assert(pack.baseline_b_mm_GET() == -617415898);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.baseline_coords_type_SET((char)3) ;
        p127.tow_SET(1884239707L) ;
        p127.nsats_SET((char)199) ;
        p127.time_last_baseline_ms_SET(1706663981L) ;
        p127.iar_num_hypotheses_SET(1410868100) ;
        p127.baseline_b_mm_SET(-617415898) ;
        p127.rtk_rate_SET((char)57) ;
        p127.rtk_health_SET((char)239) ;
        p127.baseline_a_mm_SET(693006751) ;
        p127.wn_SET((char)48671) ;
        p127.accuracy_SET(1691025263L) ;
        p127.baseline_c_mm_SET(2101939018) ;
        p127.rtk_receiver_id_SET((char)31) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.time_last_baseline_ms_GET() == 3326766404L);
            assert(pack.baseline_b_mm_GET() == -1524864192);
            assert(pack.iar_num_hypotheses_GET() == 1108000825);
            assert(pack.tow_GET() == 3627325149L);
            assert(pack.wn_GET() == (char)32569);
            assert(pack.rtk_rate_GET() == (char)47);
            assert(pack.baseline_a_mm_GET() == -1870210522);
            assert(pack.baseline_coords_type_GET() == (char)77);
            assert(pack.nsats_GET() == (char)5);
            assert(pack.baseline_c_mm_GET() == -1194717564);
            assert(pack.rtk_health_GET() == (char)39);
            assert(pack.rtk_receiver_id_GET() == (char)231);
            assert(pack.accuracy_GET() == 1331697708L);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.rtk_health_SET((char)39) ;
        p128.baseline_a_mm_SET(-1870210522) ;
        p128.rtk_receiver_id_SET((char)231) ;
        p128.accuracy_SET(1331697708L) ;
        p128.wn_SET((char)32569) ;
        p128.baseline_coords_type_SET((char)77) ;
        p128.baseline_c_mm_SET(-1194717564) ;
        p128.tow_SET(3627325149L) ;
        p128.iar_num_hypotheses_SET(1108000825) ;
        p128.nsats_SET((char)5) ;
        p128.rtk_rate_SET((char)47) ;
        p128.time_last_baseline_ms_SET(3326766404L) ;
        p128.baseline_b_mm_SET(-1524864192) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == (short) -2157);
            assert(pack.zgyro_GET() == (short) -9640);
            assert(pack.xacc_GET() == (short) -25348);
            assert(pack.time_boot_ms_GET() == 2216609976L);
            assert(pack.ymag_GET() == (short) -19853);
            assert(pack.zacc_GET() == (short)4132);
            assert(pack.ygyro_GET() == (short) -22650);
            assert(pack.yacc_GET() == (short)4018);
            assert(pack.xmag_GET() == (short) -10021);
            assert(pack.xgyro_GET() == (short) -2332);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.ygyro_SET((short) -22650) ;
        p129.time_boot_ms_SET(2216609976L) ;
        p129.zacc_SET((short)4132) ;
        p129.yacc_SET((short)4018) ;
        p129.zmag_SET((short) -2157) ;
        p129.xgyro_SET((short) -2332) ;
        p129.xacc_SET((short) -25348) ;
        p129.ymag_SET((short) -19853) ;
        p129.xmag_SET((short) -10021) ;
        p129.zgyro_SET((short) -9640) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.height_GET() == (char)37110);
            assert(pack.jpg_quality_GET() == (char)223);
            assert(pack.packets_GET() == (char)56276);
            assert(pack.type_GET() == (char)204);
            assert(pack.payload_GET() == (char)78);
            assert(pack.size_GET() == 1780619488L);
            assert(pack.width_GET() == (char)4639);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.width_SET((char)4639) ;
        p130.type_SET((char)204) ;
        p130.payload_SET((char)78) ;
        p130.size_SET(1780619488L) ;
        p130.packets_SET((char)56276) ;
        p130.height_SET((char)37110) ;
        p130.jpg_quality_SET((char)223) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)222, (char)225, (char)121, (char)238, (char)199, (char)67, (char)137, (char)4, (char)40, (char)88, (char)89, (char)118, (char)134, (char)125, (char)53, (char)109, (char)25, (char)16, (char)94, (char)174, (char)238, (char)192, (char)31, (char)82, (char)31, (char)224, (char)211, (char)200, (char)148, (char)46, (char)11, (char)28, (char)34, (char)202, (char)117, (char)11, (char)89, (char)91, (char)128, (char)1, (char)13, (char)122, (char)61, (char)213, (char)41, (char)185, (char)160, (char)198, (char)183, (char)98, (char)154, (char)153, (char)168, (char)251, (char)216, (char)51, (char)117, (char)113, (char)19, (char)91, (char)60, (char)118, (char)219, (char)69, (char)108, (char)132, (char)229, (char)20, (char)112, (char)179, (char)142, (char)156, (char)100, (char)92, (char)219, (char)236, (char)211, (char)98, (char)190, (char)192, (char)204, (char)133, (char)240, (char)231, (char)144, (char)176, (char)45, (char)141, (char)246, (char)17, (char)52, (char)250, (char)81, (char)69, (char)160, (char)55, (char)164, (char)182, (char)57, (char)111, (char)102, (char)78, (char)18, (char)3, (char)27, (char)37, (char)144, (char)76, (char)54, (char)221, (char)45, (char)28, (char)131, (char)229, (char)222, (char)63, (char)214, (char)63, (char)102, (char)69, (char)162, (char)56, (char)22, (char)247, (char)7, (char)146, (char)248, (char)190, (char)64, (char)96, (char)52, (char)98, (char)12, (char)249, (char)166, (char)219, (char)157, (char)199, (char)176, (char)211, (char)72, (char)237, (char)215, (char)183, (char)246, (char)237, (char)93, (char)110, (char)169, (char)201, (char)13, (char)125, (char)73, (char)72, (char)230, (char)135, (char)217, (char)224, (char)103, (char)134, (char)167, (char)195, (char)226, (char)52, (char)204, (char)252, (char)199, (char)126, (char)110, (char)143, (char)255, (char)114, (char)35, (char)255, (char)7, (char)198, (char)185, (char)25, (char)88, (char)190, (char)195, (char)12, (char)189, (char)28, (char)3, (char)237, (char)61, (char)23, (char)242, (char)133, (char)175, (char)120, (char)59, (char)219, (char)198, (char)190, (char)10, (char)148, (char)7, (char)78, (char)184, (char)133, (char)10, (char)28, (char)19, (char)0, (char)47, (char)179, (char)44, (char)44, (char)189, (char)136, (char)155, (char)91, (char)230, (char)38, (char)167, (char)19, (char)173, (char)173, (char)161, (char)134, (char)50, (char)237, (char)135, (char)126, (char)251, (char)252, (char)22, (char)79, (char)68, (char)88, (char)143, (char)246, (char)47, (char)52, (char)149, (char)179, (char)154, (char)96, (char)53, (char)43, (char)93, (char)189, (char)237, (char)125, (char)56, (char)61, (char)238, (char)79, (char)224, (char)250, (char)32}));
            assert(pack.seqnr_GET() == (char)64890);
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)64890) ;
        p131.data__SET(new char[] {(char)222, (char)225, (char)121, (char)238, (char)199, (char)67, (char)137, (char)4, (char)40, (char)88, (char)89, (char)118, (char)134, (char)125, (char)53, (char)109, (char)25, (char)16, (char)94, (char)174, (char)238, (char)192, (char)31, (char)82, (char)31, (char)224, (char)211, (char)200, (char)148, (char)46, (char)11, (char)28, (char)34, (char)202, (char)117, (char)11, (char)89, (char)91, (char)128, (char)1, (char)13, (char)122, (char)61, (char)213, (char)41, (char)185, (char)160, (char)198, (char)183, (char)98, (char)154, (char)153, (char)168, (char)251, (char)216, (char)51, (char)117, (char)113, (char)19, (char)91, (char)60, (char)118, (char)219, (char)69, (char)108, (char)132, (char)229, (char)20, (char)112, (char)179, (char)142, (char)156, (char)100, (char)92, (char)219, (char)236, (char)211, (char)98, (char)190, (char)192, (char)204, (char)133, (char)240, (char)231, (char)144, (char)176, (char)45, (char)141, (char)246, (char)17, (char)52, (char)250, (char)81, (char)69, (char)160, (char)55, (char)164, (char)182, (char)57, (char)111, (char)102, (char)78, (char)18, (char)3, (char)27, (char)37, (char)144, (char)76, (char)54, (char)221, (char)45, (char)28, (char)131, (char)229, (char)222, (char)63, (char)214, (char)63, (char)102, (char)69, (char)162, (char)56, (char)22, (char)247, (char)7, (char)146, (char)248, (char)190, (char)64, (char)96, (char)52, (char)98, (char)12, (char)249, (char)166, (char)219, (char)157, (char)199, (char)176, (char)211, (char)72, (char)237, (char)215, (char)183, (char)246, (char)237, (char)93, (char)110, (char)169, (char)201, (char)13, (char)125, (char)73, (char)72, (char)230, (char)135, (char)217, (char)224, (char)103, (char)134, (char)167, (char)195, (char)226, (char)52, (char)204, (char)252, (char)199, (char)126, (char)110, (char)143, (char)255, (char)114, (char)35, (char)255, (char)7, (char)198, (char)185, (char)25, (char)88, (char)190, (char)195, (char)12, (char)189, (char)28, (char)3, (char)237, (char)61, (char)23, (char)242, (char)133, (char)175, (char)120, (char)59, (char)219, (char)198, (char)190, (char)10, (char)148, (char)7, (char)78, (char)184, (char)133, (char)10, (char)28, (char)19, (char)0, (char)47, (char)179, (char)44, (char)44, (char)189, (char)136, (char)155, (char)91, (char)230, (char)38, (char)167, (char)19, (char)173, (char)173, (char)161, (char)134, (char)50, (char)237, (char)135, (char)126, (char)251, (char)252, (char)22, (char)79, (char)68, (char)88, (char)143, (char)246, (char)47, (char)52, (char)149, (char)179, (char)154, (char)96, (char)53, (char)43, (char)93, (char)189, (char)237, (char)125, (char)56, (char)61, (char)238, (char)79, (char)224, (char)250, (char)32}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.covariance_GET() == (char)94);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_135);
            assert(pack.current_distance_GET() == (char)54872);
            assert(pack.min_distance_GET() == (char)15218);
            assert(pack.id_GET() == (char)175);
            assert(pack.time_boot_ms_GET() == 3146772087L);
            assert(pack.max_distance_GET() == (char)36595);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_135) ;
        p132.min_distance_SET((char)15218) ;
        p132.id_SET((char)175) ;
        p132.current_distance_SET((char)54872) ;
        p132.time_boot_ms_SET(3146772087L) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p132.max_distance_SET((char)36595) ;
        p132.covariance_SET((char)94) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.grid_spacing_GET() == (char)17232);
            assert(pack.lon_GET() == 1354537611);
            assert(pack.lat_GET() == -1587763096);
            assert(pack.mask_GET() == 9073190316211745373L);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(-1587763096) ;
        p133.lon_SET(1354537611) ;
        p133.mask_SET(9073190316211745373L) ;
        p133.grid_spacing_SET((char)17232) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.gridbit_GET() == (char)43);
            assert(pack.lat_GET() == -1680927091);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -21765, (short)12325, (short) -16452, (short)30747, (short) -24896, (short)26003, (short) -16315, (short) -8592, (short) -4129, (short) -767, (short) -14278, (short)30091, (short)11646, (short)5352, (short) -19274, (short) -32083}));
            assert(pack.grid_spacing_GET() == (char)30012);
            assert(pack.lon_GET() == -95585887);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.data__SET(new short[] {(short) -21765, (short)12325, (short) -16452, (short)30747, (short) -24896, (short)26003, (short) -16315, (short) -8592, (short) -4129, (short) -767, (short) -14278, (short)30091, (short)11646, (short)5352, (short) -19274, (short) -32083}, 0) ;
        p134.lon_SET(-95585887) ;
        p134.lat_SET(-1680927091) ;
        p134.gridbit_SET((char)43) ;
        p134.grid_spacing_SET((char)30012) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 836376148);
            assert(pack.lat_GET() == -991294694);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-991294694) ;
        p135.lon_SET(836376148) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.terrain_height_GET() == 1.0854006E38F);
            assert(pack.spacing_GET() == (char)3768);
            assert(pack.loaded_GET() == (char)56968);
            assert(pack.lon_GET() == -281302751);
            assert(pack.pending_GET() == (char)14216);
            assert(pack.lat_GET() == -2120703843);
            assert(pack.current_height_GET() == 8.614115E37F);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.terrain_height_SET(1.0854006E38F) ;
        p136.lat_SET(-2120703843) ;
        p136.spacing_SET((char)3768) ;
        p136.lon_SET(-281302751) ;
        p136.loaded_SET((char)56968) ;
        p136.pending_SET((char)14216) ;
        p136.current_height_SET(8.614115E37F) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == 7.7554205E37F);
            assert(pack.time_boot_ms_GET() == 2327640026L);
            assert(pack.temperature_GET() == (short) -24666);
            assert(pack.press_abs_GET() == -1.9317516E37F);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.temperature_SET((short) -24666) ;
        p137.time_boot_ms_SET(2327640026L) ;
        p137.press_abs_SET(-1.9317516E37F) ;
        p137.press_diff_SET(7.7554205E37F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.8221285E38F, 8.406841E37F, 7.808994E36F, 5.2399757E37F}));
            assert(pack.y_GET() == -6.560352E37F);
            assert(pack.z_GET() == 2.8930965E38F);
            assert(pack.time_usec_GET() == 4248367623728444754L);
            assert(pack.x_GET() == -1.1742293E38F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.x_SET(-1.1742293E38F) ;
        p138.z_SET(2.8930965E38F) ;
        p138.y_SET(-6.560352E37F) ;
        p138.time_usec_SET(4248367623728444754L) ;
        p138.q_SET(new float[] {1.8221285E38F, 8.406841E37F, 7.808994E36F, 5.2399757E37F}, 0) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.3290273E38F, -4.5485033E37F, -9.613338E37F, 2.7616807E38F, 2.3134269E38F, -2.6908805E38F, -3.1747898E38F, -1.2759036E37F}));
            assert(pack.target_system_GET() == (char)114);
            assert(pack.group_mlx_GET() == (char)58);
            assert(pack.target_component_GET() == (char)105);
            assert(pack.time_usec_GET() == 2301001074083675368L);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.target_component_SET((char)105) ;
        p139.controls_SET(new float[] {2.3290273E38F, -4.5485033E37F, -9.613338E37F, 2.7616807E38F, 2.3134269E38F, -2.6908805E38F, -3.1747898E38F, -1.2759036E37F}, 0) ;
        p139.time_usec_SET(2301001074083675368L) ;
        p139.target_system_SET((char)114) ;
        p139.group_mlx_SET((char)58) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6546198502476881292L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.4180044E37F, -3.0182175E38F, 3.2429311E38F, 6.506052E37F, 1.6733222E38F, -1.9020627E38F, 8.931688E37F, -9.429813E37F}));
            assert(pack.group_mlx_GET() == (char)111);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.controls_SET(new float[] {1.4180044E37F, -3.0182175E38F, 3.2429311E38F, 6.506052E37F, 1.6733222E38F, -1.9020627E38F, 8.931688E37F, -9.429813E37F}, 0) ;
        p140.time_usec_SET(6546198502476881292L) ;
        p140.group_mlx_SET((char)111) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 1836835349172693656L);
            assert(pack.altitude_local_GET() == 1.5480776E38F);
            assert(pack.altitude_relative_GET() == -3.7662178E37F);
            assert(pack.altitude_amsl_GET() == 1.4999751E37F);
            assert(pack.altitude_monotonic_GET() == 2.7423917E38F);
            assert(pack.altitude_terrain_GET() == -1.0377995E38F);
            assert(pack.bottom_clearance_GET() == -3.0790232E37F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_relative_SET(-3.7662178E37F) ;
        p141.altitude_terrain_SET(-1.0377995E38F) ;
        p141.altitude_amsl_SET(1.4999751E37F) ;
        p141.bottom_clearance_SET(-3.0790232E37F) ;
        p141.altitude_monotonic_SET(2.7423917E38F) ;
        p141.time_usec_SET(1836835349172693656L) ;
        p141.altitude_local_SET(1.5480776E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)22, (char)173, (char)29, (char)195, (char)235, (char)142, (char)109, (char)146, (char)22, (char)1, (char)87, (char)81, (char)255, (char)159, (char)196, (char)174, (char)34, (char)157, (char)188, (char)143, (char)95, (char)113, (char)101, (char)95, (char)206, (char)148, (char)211, (char)18, (char)101, (char)111, (char)150, (char)158, (char)235, (char)79, (char)11, (char)93, (char)73, (char)140, (char)43, (char)71, (char)163, (char)181, (char)195, (char)91, (char)35, (char)144, (char)113, (char)93, (char)152, (char)239, (char)184, (char)210, (char)83, (char)11, (char)167, (char)100, (char)8, (char)164, (char)176, (char)3, (char)118, (char)161, (char)77, (char)19, (char)223, (char)123, (char)211, (char)73, (char)23, (char)224, (char)117, (char)80, (char)36, (char)223, (char)228, (char)137, (char)13, (char)236, (char)97, (char)0, (char)74, (char)7, (char)13, (char)129, (char)123, (char)203, (char)128, (char)27, (char)66, (char)182, (char)195, (char)196, (char)66, (char)4, (char)196, (char)143, (char)33, (char)221, (char)62, (char)48, (char)184, (char)95, (char)133, (char)127, (char)224, (char)102, (char)176, (char)53, (char)169, (char)57, (char)86, (char)192, (char)84, (char)197, (char)141, (char)172, (char)130, (char)222, (char)44, (char)4}));
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)116, (char)183, (char)255, (char)128, (char)205, (char)137, (char)67, (char)91, (char)160, (char)78, (char)92, (char)107, (char)27, (char)58, (char)8, (char)138, (char)87, (char)29, (char)214, (char)149, (char)69, (char)14, (char)220, (char)113, (char)195, (char)170, (char)83, (char)54, (char)100, (char)52, (char)222, (char)64, (char)235, (char)247, (char)40, (char)153, (char)94, (char)224, (char)236, (char)30, (char)82, (char)63, (char)57, (char)128, (char)57, (char)73, (char)6, (char)239, (char)102, (char)23, (char)154, (char)141, (char)231, (char)19, (char)179, (char)44, (char)67, (char)178, (char)226, (char)226, (char)76, (char)77, (char)248, (char)3, (char)235, (char)35, (char)183, (char)154, (char)76, (char)167, (char)118, (char)228, (char)24, (char)112, (char)229, (char)106, (char)121, (char)252, (char)226, (char)246, (char)103, (char)252, (char)133, (char)248, (char)102, (char)136, (char)126, (char)7, (char)200, (char)102, (char)161, (char)226, (char)78, (char)164, (char)87, (char)208, (char)8, (char)19, (char)55, (char)166, (char)118, (char)19, (char)88, (char)120, (char)123, (char)121, (char)179, (char)209, (char)47, (char)94, (char)13, (char)82, (char)109, (char)61, (char)104, (char)161, (char)175, (char)83, (char)104, (char)77}));
            assert(pack.uri_type_GET() == (char)9);
            assert(pack.transfer_type_GET() == (char)33);
            assert(pack.request_id_GET() == (char)212);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.storage_SET(new char[] {(char)116, (char)183, (char)255, (char)128, (char)205, (char)137, (char)67, (char)91, (char)160, (char)78, (char)92, (char)107, (char)27, (char)58, (char)8, (char)138, (char)87, (char)29, (char)214, (char)149, (char)69, (char)14, (char)220, (char)113, (char)195, (char)170, (char)83, (char)54, (char)100, (char)52, (char)222, (char)64, (char)235, (char)247, (char)40, (char)153, (char)94, (char)224, (char)236, (char)30, (char)82, (char)63, (char)57, (char)128, (char)57, (char)73, (char)6, (char)239, (char)102, (char)23, (char)154, (char)141, (char)231, (char)19, (char)179, (char)44, (char)67, (char)178, (char)226, (char)226, (char)76, (char)77, (char)248, (char)3, (char)235, (char)35, (char)183, (char)154, (char)76, (char)167, (char)118, (char)228, (char)24, (char)112, (char)229, (char)106, (char)121, (char)252, (char)226, (char)246, (char)103, (char)252, (char)133, (char)248, (char)102, (char)136, (char)126, (char)7, (char)200, (char)102, (char)161, (char)226, (char)78, (char)164, (char)87, (char)208, (char)8, (char)19, (char)55, (char)166, (char)118, (char)19, (char)88, (char)120, (char)123, (char)121, (char)179, (char)209, (char)47, (char)94, (char)13, (char)82, (char)109, (char)61, (char)104, (char)161, (char)175, (char)83, (char)104, (char)77}, 0) ;
        p142.request_id_SET((char)212) ;
        p142.uri_SET(new char[] {(char)22, (char)173, (char)29, (char)195, (char)235, (char)142, (char)109, (char)146, (char)22, (char)1, (char)87, (char)81, (char)255, (char)159, (char)196, (char)174, (char)34, (char)157, (char)188, (char)143, (char)95, (char)113, (char)101, (char)95, (char)206, (char)148, (char)211, (char)18, (char)101, (char)111, (char)150, (char)158, (char)235, (char)79, (char)11, (char)93, (char)73, (char)140, (char)43, (char)71, (char)163, (char)181, (char)195, (char)91, (char)35, (char)144, (char)113, (char)93, (char)152, (char)239, (char)184, (char)210, (char)83, (char)11, (char)167, (char)100, (char)8, (char)164, (char)176, (char)3, (char)118, (char)161, (char)77, (char)19, (char)223, (char)123, (char)211, (char)73, (char)23, (char)224, (char)117, (char)80, (char)36, (char)223, (char)228, (char)137, (char)13, (char)236, (char)97, (char)0, (char)74, (char)7, (char)13, (char)129, (char)123, (char)203, (char)128, (char)27, (char)66, (char)182, (char)195, (char)196, (char)66, (char)4, (char)196, (char)143, (char)33, (char)221, (char)62, (char)48, (char)184, (char)95, (char)133, (char)127, (char)224, (char)102, (char)176, (char)53, (char)169, (char)57, (char)86, (char)192, (char)84, (char)197, (char)141, (char)172, (char)130, (char)222, (char)44, (char)4}, 0) ;
        p142.uri_type_SET((char)9) ;
        p142.transfer_type_SET((char)33) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 4.1981644E37F);
            assert(pack.press_diff_GET() == 9.516283E37F);
            assert(pack.time_boot_ms_GET() == 3028285737L);
            assert(pack.temperature_GET() == (short)348);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.press_diff_SET(9.516283E37F) ;
        p143.press_abs_SET(4.1981644E37F) ;
        p143.temperature_SET((short)348) ;
        p143.time_boot_ms_SET(3028285737L) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1707747882);
            assert(pack.est_capabilities_GET() == (char)188);
            assert(pack.alt_GET() == 3.9479758E37F);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {3.3858255E37F, 1.0907446E38F, -2.1315074E38F}));
            assert(pack.custom_state_GET() == 6529624269576832799L);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-1.825273E38F, -1.2436378E38F, -1.9260551E38F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-4.598933E37F, -3.2671771E38F, 2.5891007E38F, 2.1848068E38F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {3.3739969E38F, -4.3470406E37F, 8.33073E37F}));
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-2.1432814E38F, -4.8813214E37F, -1.1150415E38F}));
            assert(pack.lat_GET() == -20806410);
            assert(pack.timestamp_GET() == 4921357914806976474L);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.lat_SET(-20806410) ;
        p144.est_capabilities_SET((char)188) ;
        p144.acc_SET(new float[] {-2.1432814E38F, -4.8813214E37F, -1.1150415E38F}, 0) ;
        p144.attitude_q_SET(new float[] {-4.598933E37F, -3.2671771E38F, 2.5891007E38F, 2.1848068E38F}, 0) ;
        p144.lon_SET(-1707747882) ;
        p144.timestamp_SET(4921357914806976474L) ;
        p144.vel_SET(new float[] {3.3858255E37F, 1.0907446E38F, -2.1315074E38F}, 0) ;
        p144.alt_SET(3.9479758E37F) ;
        p144.custom_state_SET(6529624269576832799L) ;
        p144.position_cov_SET(new float[] {-1.825273E38F, -1.2436378E38F, -1.9260551E38F}, 0) ;
        p144.rates_SET(new float[] {3.3739969E38F, -4.3470406E37F, 8.33073E37F}, 0) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.z_pos_GET() == 5.7400335E37F);
            assert(pack.y_pos_GET() == 6.136272E37F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {2.200344E38F, 2.6263287E38F, 1.7875853E38F}));
            assert(pack.z_acc_GET() == 9.793531E37F);
            assert(pack.time_usec_GET() == 1823683495072412713L);
            assert(pack.x_pos_GET() == 2.8637797E38F);
            assert(pack.yaw_rate_GET() == 1.4441108E38F);
            assert(pack.x_vel_GET() == 7.2013564E36F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-4.7312746E37F, -1.1557094E38F, 1.0289716E38F, 3.2365943E38F}));
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {1.90952E38F, -2.7775966E37F, -2.1335673E38F}));
            assert(pack.z_vel_GET() == -2.5425357E38F);
            assert(pack.airspeed_GET() == 1.7647823E36F);
            assert(pack.y_acc_GET() == -6.6001887E37F);
            assert(pack.y_vel_GET() == 3.1577661E38F);
            assert(pack.x_acc_GET() == -1.0995327E38F);
            assert(pack.pitch_rate_GET() == 2.90445E38F);
            assert(pack.roll_rate_GET() == -4.0784706E37F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.vel_variance_SET(new float[] {2.200344E38F, 2.6263287E38F, 1.7875853E38F}, 0) ;
        p146.y_vel_SET(3.1577661E38F) ;
        p146.time_usec_SET(1823683495072412713L) ;
        p146.airspeed_SET(1.7647823E36F) ;
        p146.yaw_rate_SET(1.4441108E38F) ;
        p146.x_acc_SET(-1.0995327E38F) ;
        p146.pitch_rate_SET(2.90445E38F) ;
        p146.roll_rate_SET(-4.0784706E37F) ;
        p146.pos_variance_SET(new float[] {1.90952E38F, -2.7775966E37F, -2.1335673E38F}, 0) ;
        p146.y_pos_SET(6.136272E37F) ;
        p146.x_pos_SET(2.8637797E38F) ;
        p146.x_vel_SET(7.2013564E36F) ;
        p146.z_vel_SET(-2.5425357E38F) ;
        p146.q_SET(new float[] {-4.7312746E37F, -1.1557094E38F, 1.0289716E38F, 3.2365943E38F}, 0) ;
        p146.z_pos_SET(5.7400335E37F) ;
        p146.z_acc_SET(9.793531E37F) ;
        p146.y_acc_SET(-6.6001887E37F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.battery_remaining_GET() == (byte)54);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
            assert(pack.temperature_GET() == (short)20345);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)11852, (char)29678, (char)14456, (char)3684, (char)62061, (char)13164, (char)34902, (char)15293, (char)59395, (char)40138}));
            assert(pack.current_battery_GET() == (short) -19917);
            assert(pack.energy_consumed_GET() == 279770055);
            assert(pack.id_GET() == (char)166);
            assert(pack.current_consumed_GET() == -486931450);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.current_battery_SET((short) -19917) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN) ;
        p147.current_consumed_SET(-486931450) ;
        p147.temperature_SET((short)20345) ;
        p147.energy_consumed_SET(279770055) ;
        p147.voltages_SET(new char[] {(char)11852, (char)29678, (char)14456, (char)3684, (char)62061, (char)13164, (char)34902, (char)15293, (char)59395, (char)40138}, 0) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL) ;
        p147.battery_remaining_SET((byte)54) ;
        p147.id_SET((char)166) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2));
            assert(pack.middleware_sw_version_GET() == 530581071L);
            assert(pack.vendor_id_GET() == (char)2226);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)156, (char)133, (char)67, (char)179, (char)254, (char)233, (char)221, (char)81, (char)218, (char)95, (char)212, (char)243, (char)40, (char)91, (char)89, (char)167, (char)224, (char)230}));
            assert(pack.uid_GET() == 2840398865259980986L);
            assert(pack.product_id_GET() == (char)2464);
            assert(pack.os_sw_version_GET() == 2868414730L);
            assert(pack.board_version_GET() == 3834303106L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)153, (char)191, (char)161, (char)78, (char)111, (char)9, (char)7, (char)254}));
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)24, (char)43, (char)232, (char)141, (char)209, (char)157, (char)109, (char)210}));
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)53, (char)127, (char)93, (char)143, (char)136, (char)236, (char)146, (char)155}));
            assert(pack.flight_sw_version_GET() == 2440755699L);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.uid_SET(2840398865259980986L) ;
        p148.middleware_sw_version_SET(530581071L) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2)) ;
        p148.middleware_custom_version_SET(new char[] {(char)24, (char)43, (char)232, (char)141, (char)209, (char)157, (char)109, (char)210}, 0) ;
        p148.board_version_SET(3834303106L) ;
        p148.vendor_id_SET((char)2226) ;
        p148.flight_custom_version_SET(new char[] {(char)53, (char)127, (char)93, (char)143, (char)136, (char)236, (char)146, (char)155}, 0) ;
        p148.flight_sw_version_SET(2440755699L) ;
        p148.uid2_SET(new char[] {(char)156, (char)133, (char)67, (char)179, (char)254, (char)233, (char)221, (char)81, (char)218, (char)95, (char)212, (char)243, (char)40, (char)91, (char)89, (char)167, (char)224, (char)230}, 0, PH) ;
        p148.os_custom_version_SET(new char[] {(char)153, (char)191, (char)161, (char)78, (char)111, (char)9, (char)7, (char)254}, 0) ;
        p148.product_id_SET((char)2464) ;
        p148.os_sw_version_SET(2868414730L) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.angle_x_GET() == -2.4841534E38F);
            assert(pack.target_num_GET() == (char)221);
            assert(pack.position_valid_TRY(ph) == (char)37);
            assert(pack.size_x_GET() == -1.5143925E38F);
            assert(pack.z_TRY(ph) == 2.392553E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {2.3567308E38F, 1.1238366E38F, 4.8804817E37F, -1.765801E38F}));
            assert(pack.x_TRY(ph) == 2.6176873E37F);
            assert(pack.angle_y_GET() == -1.060237E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.time_usec_GET() == 6186191261234371001L);
            assert(pack.size_y_GET() == -7.118271E37F);
            assert(pack.distance_GET() == 6.829801E37F);
            assert(pack.y_TRY(ph) == 1.9415387E38F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.y_SET(1.9415387E38F, PH) ;
        p149.angle_y_SET(-1.060237E38F) ;
        p149.size_x_SET(-1.5143925E38F) ;
        p149.size_y_SET(-7.118271E37F) ;
        p149.time_usec_SET(6186191261234371001L) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON) ;
        p149.q_SET(new float[] {2.3567308E38F, 1.1238366E38F, 4.8804817E37F, -1.765801E38F}, 0, PH) ;
        p149.x_SET(2.6176873E37F, PH) ;
        p149.angle_x_SET(-2.4841534E38F) ;
        p149.target_num_SET((char)221) ;
        p149.position_valid_SET((char)37, PH) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p149.distance_SET(6.829801E37F) ;
        p149.z_SET(2.392553E38F, PH) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAV_FILTER_BIAS.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 4986764857132413820L);
            assert(pack.gyro_2_GET() == -1.8723224E38F);
            assert(pack.accel_1_GET() == 7.182254E37F);
            assert(pack.gyro_0_GET() == -3.3811647E38F);
            assert(pack.gyro_1_GET() == -9.421146E37F);
            assert(pack.accel_0_GET() == -8.9372593E36F);
            assert(pack.accel_2_GET() == 2.158936E38F);
        });
        GroundControl.NAV_FILTER_BIAS p220 = CommunicationChannel.new_NAV_FILTER_BIAS();
        PH.setPack(p220);
        p220.gyro_2_SET(-1.8723224E38F) ;
        p220.accel_1_SET(7.182254E37F) ;
        p220.usec_SET(4986764857132413820L) ;
        p220.accel_0_SET(-8.9372593E36F) ;
        p220.gyro_0_SET(-3.3811647E38F) ;
        p220.accel_2_SET(2.158936E38F) ;
        p220.gyro_1_SET(-9.421146E37F) ;
        CommunicationChannel.instance.send(p220);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RADIO_CALIBRATION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.rudder_GET(),  new char[] {(char)13993, (char)16746, (char)4873}));
            assert(Arrays.equals(pack.throttle_GET(),  new char[] {(char)34044, (char)44950, (char)26269, (char)60154, (char)59096}));
            assert(Arrays.equals(pack.pitch_GET(),  new char[] {(char)61564, (char)57915, (char)41773, (char)15231, (char)39000}));
            assert(Arrays.equals(pack.gyro_GET(),  new char[] {(char)41637, (char)36248}));
            assert(Arrays.equals(pack.elevator_GET(),  new char[] {(char)53303, (char)49731, (char)37020}));
            assert(Arrays.equals(pack.aileron_GET(),  new char[] {(char)63070, (char)41487, (char)32731}));
        });
        GroundControl.RADIO_CALIBRATION p221 = CommunicationChannel.new_RADIO_CALIBRATION();
        PH.setPack(p221);
        p221.rudder_SET(new char[] {(char)13993, (char)16746, (char)4873}, 0) ;
        p221.gyro_SET(new char[] {(char)41637, (char)36248}, 0) ;
        p221.throttle_SET(new char[] {(char)34044, (char)44950, (char)26269, (char)60154, (char)59096}, 0) ;
        p221.aileron_SET(new char[] {(char)63070, (char)41487, (char)32731}, 0) ;
        p221.pitch_SET(new char[] {(char)61564, (char)57915, (char)41773, (char)15231, (char)39000}, 0) ;
        p221.elevator_SET(new char[] {(char)53303, (char)49731, (char)37020}, 0) ;
        CommunicationChannel.instance.send(p221);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UALBERTA_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pilot_GET() == (char)223);
            assert(pack.mode_GET() == (char)238);
            assert(pack.nav_mode_GET() == (char)213);
        });
        GroundControl.UALBERTA_SYS_STATUS p222 = CommunicationChannel.new_UALBERTA_SYS_STATUS();
        PH.setPack(p222);
        p222.nav_mode_SET((char)213) ;
        p222.pilot_SET((char)223) ;
        p222.mode_SET((char)238) ;
        CommunicationChannel.instance.send(p222);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.hagl_ratio_GET() == 2.7575287E38F);
            assert(pack.pos_vert_ratio_GET() == -1.7098092E38F);
            assert(pack.vel_ratio_GET() == 7.5803716E37F);
            assert(pack.pos_horiz_accuracy_GET() == -1.0611247E38F);
            assert(pack.time_usec_GET() == 3137448412569836211L);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS));
            assert(pack.mag_ratio_GET() == 2.8510237E38F);
            assert(pack.pos_vert_accuracy_GET() == -1.8256507E38F);
            assert(pack.pos_horiz_ratio_GET() == -1.8655397E38F);
            assert(pack.tas_ratio_GET() == 9.25592E37F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.mag_ratio_SET(2.8510237E38F) ;
        p230.pos_vert_accuracy_SET(-1.8256507E38F) ;
        p230.hagl_ratio_SET(2.7575287E38F) ;
        p230.pos_horiz_accuracy_SET(-1.0611247E38F) ;
        p230.tas_ratio_SET(9.25592E37F) ;
        p230.vel_ratio_SET(7.5803716E37F) ;
        p230.pos_horiz_ratio_SET(-1.8655397E38F) ;
        p230.pos_vert_ratio_SET(-1.7098092E38F) ;
        p230.time_usec_SET(3137448412569836211L) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS)) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.vert_accuracy_GET() == -1.9188614E38F);
            assert(pack.wind_y_GET() == -3.3376372E38F);
            assert(pack.var_vert_GET() == 3.1518215E37F);
            assert(pack.wind_x_GET() == -1.725231E38F);
            assert(pack.time_usec_GET() == 4327549437324649278L);
            assert(pack.wind_alt_GET() == -1.2991002E38F);
            assert(pack.wind_z_GET() == 2.8923428E38F);
            assert(pack.var_horiz_GET() == -7.5315686E37F);
            assert(pack.horiz_accuracy_GET() == 1.6466427E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(4327549437324649278L) ;
        p231.var_vert_SET(3.1518215E37F) ;
        p231.wind_z_SET(2.8923428E38F) ;
        p231.wind_y_SET(-3.3376372E38F) ;
        p231.vert_accuracy_SET(-1.9188614E38F) ;
        p231.horiz_accuracy_SET(1.6466427E38F) ;
        p231.wind_x_SET(-1.725231E38F) ;
        p231.wind_alt_SET(-1.2991002E38F) ;
        p231.var_horiz_SET(-7.5315686E37F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.vert_accuracy_GET() == 3.3488035E38F);
            assert(pack.satellites_visible_GET() == (char)150);
            assert(pack.fix_type_GET() == (char)3);
            assert(pack.time_week_ms_GET() == 3957896564L);
            assert(pack.gps_id_GET() == (char)208);
            assert(pack.lon_GET() == 1915224690);
            assert(pack.horiz_accuracy_GET() == -2.5208707E38F);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY));
            assert(pack.vd_GET() == 2.0730884E38F);
            assert(pack.hdop_GET() == 3.5674464E37F);
            assert(pack.time_week_GET() == (char)65531);
            assert(pack.ve_GET() == 1.00794E38F);
            assert(pack.time_usec_GET() == 5543564345776966512L);
            assert(pack.speed_accuracy_GET() == -7.1475353E37F);
            assert(pack.alt_GET() == 1.6159929E36F);
            assert(pack.lat_GET() == -454928609);
            assert(pack.vdop_GET() == -1.5719925E38F);
            assert(pack.vn_GET() == -5.489184E35F);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY)) ;
        p232.satellites_visible_SET((char)150) ;
        p232.horiz_accuracy_SET(-2.5208707E38F) ;
        p232.fix_type_SET((char)3) ;
        p232.gps_id_SET((char)208) ;
        p232.time_week_ms_SET(3957896564L) ;
        p232.time_week_SET((char)65531) ;
        p232.ve_SET(1.00794E38F) ;
        p232.time_usec_SET(5543564345776966512L) ;
        p232.vd_SET(2.0730884E38F) ;
        p232.vdop_SET(-1.5719925E38F) ;
        p232.vert_accuracy_SET(3.3488035E38F) ;
        p232.speed_accuracy_SET(-7.1475353E37F) ;
        p232.lon_SET(1915224690) ;
        p232.hdop_SET(3.5674464E37F) ;
        p232.lat_SET(-454928609) ;
        p232.vn_SET(-5.489184E35F) ;
        p232.alt_SET(1.6159929E36F) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (char)148);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)224, (char)142, (char)233, (char)181, (char)170, (char)31, (char)150, (char)69, (char)45, (char)217, (char)155, (char)133, (char)120, (char)33, (char)251, (char)153, (char)225, (char)20, (char)249, (char)9, (char)14, (char)10, (char)254, (char)254, (char)2, (char)37, (char)30, (char)125, (char)100, (char)166, (char)223, (char)62, (char)98, (char)225, (char)226, (char)189, (char)226, (char)23, (char)97, (char)189, (char)174, (char)210, (char)6, (char)142, (char)92, (char)135, (char)103, (char)79, (char)229, (char)197, (char)48, (char)77, (char)15, (char)28, (char)38, (char)178, (char)44, (char)45, (char)32, (char)129, (char)195, (char)23, (char)185, (char)238, (char)181, (char)154, (char)79, (char)208, (char)215, (char)209, (char)30, (char)213, (char)151, (char)162, (char)25, (char)204, (char)133, (char)167, (char)18, (char)175, (char)209, (char)153, (char)196, (char)207, (char)190, (char)63, (char)247, (char)175, (char)211, (char)59, (char)89, (char)120, (char)54, (char)104, (char)207, (char)226, (char)124, (char)37, (char)73, (char)210, (char)152, (char)48, (char)241, (char)162, (char)217, (char)196, (char)244, (char)229, (char)232, (char)70, (char)22, (char)213, (char)249, (char)116, (char)250, (char)166, (char)106, (char)231, (char)166, (char)190, (char)88, (char)11, (char)168, (char)192, (char)104, (char)33, (char)143, (char)213, (char)149, (char)54, (char)227, (char)87, (char)220, (char)251, (char)234, (char)125, (char)55, (char)0, (char)205, (char)173, (char)154, (char)136, (char)102, (char)244, (char)53, (char)214, (char)236, (char)109, (char)167, (char)218, (char)202, (char)228, (char)189, (char)239, (char)104, (char)83, (char)221, (char)202, (char)36, (char)212, (char)4, (char)203, (char)120, (char)252, (char)175, (char)60, (char)198, (char)126, (char)156, (char)204, (char)14, (char)210, (char)91, (char)44, (char)24, (char)165, (char)84, (char)135, (char)203, (char)234}));
            assert(pack.len_GET() == (char)62);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)148) ;
        p233.data__SET(new char[] {(char)224, (char)142, (char)233, (char)181, (char)170, (char)31, (char)150, (char)69, (char)45, (char)217, (char)155, (char)133, (char)120, (char)33, (char)251, (char)153, (char)225, (char)20, (char)249, (char)9, (char)14, (char)10, (char)254, (char)254, (char)2, (char)37, (char)30, (char)125, (char)100, (char)166, (char)223, (char)62, (char)98, (char)225, (char)226, (char)189, (char)226, (char)23, (char)97, (char)189, (char)174, (char)210, (char)6, (char)142, (char)92, (char)135, (char)103, (char)79, (char)229, (char)197, (char)48, (char)77, (char)15, (char)28, (char)38, (char)178, (char)44, (char)45, (char)32, (char)129, (char)195, (char)23, (char)185, (char)238, (char)181, (char)154, (char)79, (char)208, (char)215, (char)209, (char)30, (char)213, (char)151, (char)162, (char)25, (char)204, (char)133, (char)167, (char)18, (char)175, (char)209, (char)153, (char)196, (char)207, (char)190, (char)63, (char)247, (char)175, (char)211, (char)59, (char)89, (char)120, (char)54, (char)104, (char)207, (char)226, (char)124, (char)37, (char)73, (char)210, (char)152, (char)48, (char)241, (char)162, (char)217, (char)196, (char)244, (char)229, (char)232, (char)70, (char)22, (char)213, (char)249, (char)116, (char)250, (char)166, (char)106, (char)231, (char)166, (char)190, (char)88, (char)11, (char)168, (char)192, (char)104, (char)33, (char)143, (char)213, (char)149, (char)54, (char)227, (char)87, (char)220, (char)251, (char)234, (char)125, (char)55, (char)0, (char)205, (char)173, (char)154, (char)136, (char)102, (char)244, (char)53, (char)214, (char)236, (char)109, (char)167, (char)218, (char)202, (char)228, (char)189, (char)239, (char)104, (char)83, (char)221, (char)202, (char)36, (char)212, (char)4, (char)203, (char)120, (char)252, (char)175, (char)60, (char)198, (char)126, (char)156, (char)204, (char)14, (char)210, (char)91, (char)44, (char)24, (char)165, (char)84, (char)135, (char)203, (char)234}, 0) ;
        p233.len_SET((char)62) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
            assert(pack.groundspeed_GET() == (char)133);
            assert(pack.heading_sp_GET() == (short) -4603);
            assert(pack.wp_num_GET() == (char)119);
            assert(pack.roll_GET() == (short)12034);
            assert(pack.airspeed_GET() == (char)182);
            assert(pack.custom_mode_GET() == 1068953866L);
            assert(pack.temperature_GET() == (byte) - 31);
            assert(pack.throttle_GET() == (byte)99);
            assert(pack.wp_distance_GET() == (char)3183);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.failsafe_GET() == (char)135);
            assert(pack.battery_remaining_GET() == (char)73);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
            assert(pack.temperature_air_GET() == (byte) - 5);
            assert(pack.airspeed_sp_GET() == (char)66);
            assert(pack.pitch_GET() == (short)6287);
            assert(pack.altitude_amsl_GET() == (short) -16409);
            assert(pack.climb_rate_GET() == (byte)43);
            assert(pack.latitude_GET() == 1224577678);
            assert(pack.heading_GET() == (char)41770);
            assert(pack.altitude_sp_GET() == (short) -18032);
            assert(pack.longitude_GET() == 2106582504);
            assert(pack.gps_nsat_GET() == (char)23);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.wp_num_SET((char)119) ;
        p234.heading_SET((char)41770) ;
        p234.roll_SET((short)12034) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        p234.airspeed_SET((char)182) ;
        p234.altitude_amsl_SET((short) -16409) ;
        p234.latitude_SET(1224577678) ;
        p234.temperature_air_SET((byte) - 5) ;
        p234.heading_sp_SET((short) -4603) ;
        p234.climb_rate_SET((byte)43) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP) ;
        p234.groundspeed_SET((char)133) ;
        p234.gps_nsat_SET((char)23) ;
        p234.wp_distance_SET((char)3183) ;
        p234.custom_mode_SET(1068953866L) ;
        p234.throttle_SET((byte)99) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) ;
        p234.airspeed_sp_SET((char)66) ;
        p234.failsafe_SET((char)135) ;
        p234.temperature_SET((byte) - 31) ;
        p234.pitch_SET((short)6287) ;
        p234.altitude_sp_SET((short) -18032) ;
        p234.longitude_SET(2106582504) ;
        p234.battery_remaining_SET((char)73) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.clipping_0_GET() == 2979510363L);
            assert(pack.vibration_x_GET() == 3.1794853E38F);
            assert(pack.clipping_2_GET() == 2163294473L);
            assert(pack.clipping_1_GET() == 4078346068L);
            assert(pack.time_usec_GET() == 430276584151928746L);
            assert(pack.vibration_z_GET() == 2.4966987E38F);
            assert(pack.vibration_y_GET() == 5.4791776E36F);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_2_SET(2163294473L) ;
        p241.vibration_z_SET(2.4966987E38F) ;
        p241.vibration_y_SET(5.4791776E36F) ;
        p241.time_usec_SET(430276584151928746L) ;
        p241.vibration_x_SET(3.1794853E38F) ;
        p241.clipping_0_SET(2979510363L) ;
        p241.clipping_1_SET(4078346068L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 6628518556541816722L);
            assert(pack.x_GET() == -2.4949027E38F);
            assert(pack.altitude_GET() == 527217705);
            assert(pack.approach_x_GET() == 2.815528E38F);
            assert(pack.approach_y_GET() == -3.6394261E37F);
            assert(pack.latitude_GET() == 774391673);
            assert(pack.approach_z_GET() == 3.3436072E37F);
            assert(pack.y_GET() == -2.5884904E38F);
            assert(pack.longitude_GET() == -180216461);
            assert(pack.z_GET() == -4.483349E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.3670514E38F, 3.7328918E37F, 2.6042099E38F, -1.0057188E38F}));
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.y_SET(-2.5884904E38F) ;
        p242.q_SET(new float[] {3.3670514E38F, 3.7328918E37F, 2.6042099E38F, -1.0057188E38F}, 0) ;
        p242.altitude_SET(527217705) ;
        p242.approach_z_SET(3.3436072E37F) ;
        p242.latitude_SET(774391673) ;
        p242.longitude_SET(-180216461) ;
        p242.x_SET(-2.4949027E38F) ;
        p242.approach_x_SET(2.815528E38F) ;
        p242.z_SET(-4.483349E37F) ;
        p242.time_usec_SET(6628518556541816722L, PH) ;
        p242.approach_y_SET(-3.6394261E37F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == -1966529221);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.6995056E38F, 6.687265E37F, 2.210955E38F, -2.3391226E38F}));
            assert(pack.target_system_GET() == (char)17);
            assert(pack.time_usec_TRY(ph) == 235241897034299418L);
            assert(pack.approach_z_GET() == -2.127742E38F);
            assert(pack.x_GET() == 9.354868E37F);
            assert(pack.approach_y_GET() == -1.4627882E38F);
            assert(pack.z_GET() == -1.6944607E38F);
            assert(pack.approach_x_GET() == -3.3761209E38F);
            assert(pack.longitude_GET() == -1407931828);
            assert(pack.latitude_GET() == -1731192334);
            assert(pack.y_GET() == 1.5072059E38F);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.approach_z_SET(-2.127742E38F) ;
        p243.q_SET(new float[] {-2.6995056E38F, 6.687265E37F, 2.210955E38F, -2.3391226E38F}, 0) ;
        p243.y_SET(1.5072059E38F) ;
        p243.z_SET(-1.6944607E38F) ;
        p243.approach_x_SET(-3.3761209E38F) ;
        p243.target_system_SET((char)17) ;
        p243.longitude_SET(-1407931828) ;
        p243.time_usec_SET(235241897034299418L, PH) ;
        p243.altitude_SET(-1966529221) ;
        p243.latitude_SET(-1731192334) ;
        p243.approach_y_SET(-1.4627882E38F) ;
        p243.x_SET(9.354868E37F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == -1209897467);
            assert(pack.message_id_GET() == (char)31055);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-1209897467) ;
        p244.message_id_SET((char)31055) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING) ;
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.ICAO_address_GET() == 3748550732L);
            assert(pack.squawk_GET() == (char)32954);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT);
            assert(pack.altitude_GET() == -773148369);
            assert(pack.ver_velocity_GET() == (short)20881);
            assert(pack.hor_velocity_GET() == (char)34709);
            assert(pack.lat_GET() == 1283430153);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.callsign_LEN(ph) == 6);
            assert(pack.callsign_TRY(ph).equals("hityth"));
            assert(pack.lon_GET() == -192079400);
            assert(pack.tslc_GET() == (char)232);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS));
            assert(pack.heading_GET() == (char)4723);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.squawk_SET((char)32954) ;
        p246.lat_SET(1283430153) ;
        p246.ICAO_address_SET(3748550732L) ;
        p246.hor_velocity_SET((char)34709) ;
        p246.callsign_SET("hityth", PH) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.heading_SET((char)4723) ;
        p246.tslc_SET((char)232) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT) ;
        p246.lon_SET(-192079400) ;
        p246.altitude_SET(-773148369) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS)) ;
        p246.ver_velocity_SET((short)20881) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == 63636270L);
            assert(pack.time_to_minimum_delta_GET() == -2.191384E38F);
            assert(pack.altitude_minimum_delta_GET() == -4.1694103E37F);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.horizontal_minimum_delta_GET() == -2.0285817E38F);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY) ;
        p247.id_SET(63636270L) ;
        p247.time_to_minimum_delta_SET(-2.191384E38F) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE) ;
        p247.altitude_minimum_delta_SET(-4.1694103E37F) ;
        p247.horizontal_minimum_delta_SET(-2.0285817E38F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)85, (char)15, (char)205, (char)129, (char)122, (char)63, (char)207, (char)225, (char)65, (char)132, (char)232, (char)88, (char)213, (char)191, (char)113, (char)52, (char)214, (char)223, (char)88, (char)148, (char)39, (char)247, (char)56, (char)23, (char)149, (char)6, (char)89, (char)86, (char)19, (char)0, (char)60, (char)96, (char)69, (char)181, (char)106, (char)46, (char)243, (char)213, (char)3, (char)156, (char)73, (char)39, (char)105, (char)192, (char)4, (char)249, (char)183, (char)135, (char)227, (char)98, (char)218, (char)9, (char)157, (char)57, (char)236, (char)42, (char)118, (char)140, (char)116, (char)45, (char)32, (char)95, (char)64, (char)130, (char)195, (char)243, (char)248, (char)240, (char)143, (char)78, (char)47, (char)236, (char)96, (char)156, (char)153, (char)100, (char)126, (char)113, (char)188, (char)38, (char)224, (char)60, (char)17, (char)153, (char)120, (char)96, (char)14, (char)26, (char)250, (char)74, (char)123, (char)227, (char)132, (char)56, (char)135, (char)244, (char)57, (char)48, (char)218, (char)21, (char)70, (char)203, (char)0, (char)7, (char)201, (char)191, (char)31, (char)82, (char)219, (char)240, (char)206, (char)133, (char)81, (char)104, (char)52, (char)34, (char)227, (char)197, (char)117, (char)179, (char)77, (char)231, (char)103, (char)10, (char)223, (char)141, (char)133, (char)234, (char)16, (char)239, (char)185, (char)89, (char)153, (char)233, (char)156, (char)12, (char)185, (char)242, (char)53, (char)253, (char)53, (char)73, (char)239, (char)48, (char)69, (char)202, (char)182, (char)216, (char)6, (char)8, (char)254, (char)61, (char)235, (char)220, (char)224, (char)175, (char)24, (char)237, (char)241, (char)39, (char)34, (char)207, (char)160, (char)185, (char)142, (char)64, (char)71, (char)233, (char)207, (char)69, (char)8, (char)149, (char)87, (char)246, (char)77, (char)149, (char)240, (char)2, (char)106, (char)79, (char)125, (char)225, (char)86, (char)112, (char)155, (char)237, (char)169, (char)223, (char)61, (char)64, (char)240, (char)44, (char)34, (char)212, (char)186, (char)24, (char)226, (char)188, (char)193, (char)73, (char)140, (char)3, (char)161, (char)208, (char)210, (char)12, (char)59, (char)63, (char)240, (char)145, (char)39, (char)99, (char)13, (char)227, (char)162, (char)3, (char)129, (char)21, (char)64, (char)81, (char)108, (char)203, (char)207, (char)233, (char)230, (char)2, (char)211, (char)27, (char)134, (char)146, (char)146, (char)192, (char)142, (char)65, (char)161, (char)12, (char)158, (char)21, (char)44, (char)208, (char)247, (char)126, (char)158, (char)92, (char)76, (char)216, (char)92, (char)142, (char)125}));
            assert(pack.message_type_GET() == (char)34719);
            assert(pack.target_network_GET() == (char)38);
            assert(pack.target_component_GET() == (char)34);
            assert(pack.target_system_GET() == (char)203);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_system_SET((char)203) ;
        p248.target_component_SET((char)34) ;
        p248.message_type_SET((char)34719) ;
        p248.payload_SET(new char[] {(char)85, (char)15, (char)205, (char)129, (char)122, (char)63, (char)207, (char)225, (char)65, (char)132, (char)232, (char)88, (char)213, (char)191, (char)113, (char)52, (char)214, (char)223, (char)88, (char)148, (char)39, (char)247, (char)56, (char)23, (char)149, (char)6, (char)89, (char)86, (char)19, (char)0, (char)60, (char)96, (char)69, (char)181, (char)106, (char)46, (char)243, (char)213, (char)3, (char)156, (char)73, (char)39, (char)105, (char)192, (char)4, (char)249, (char)183, (char)135, (char)227, (char)98, (char)218, (char)9, (char)157, (char)57, (char)236, (char)42, (char)118, (char)140, (char)116, (char)45, (char)32, (char)95, (char)64, (char)130, (char)195, (char)243, (char)248, (char)240, (char)143, (char)78, (char)47, (char)236, (char)96, (char)156, (char)153, (char)100, (char)126, (char)113, (char)188, (char)38, (char)224, (char)60, (char)17, (char)153, (char)120, (char)96, (char)14, (char)26, (char)250, (char)74, (char)123, (char)227, (char)132, (char)56, (char)135, (char)244, (char)57, (char)48, (char)218, (char)21, (char)70, (char)203, (char)0, (char)7, (char)201, (char)191, (char)31, (char)82, (char)219, (char)240, (char)206, (char)133, (char)81, (char)104, (char)52, (char)34, (char)227, (char)197, (char)117, (char)179, (char)77, (char)231, (char)103, (char)10, (char)223, (char)141, (char)133, (char)234, (char)16, (char)239, (char)185, (char)89, (char)153, (char)233, (char)156, (char)12, (char)185, (char)242, (char)53, (char)253, (char)53, (char)73, (char)239, (char)48, (char)69, (char)202, (char)182, (char)216, (char)6, (char)8, (char)254, (char)61, (char)235, (char)220, (char)224, (char)175, (char)24, (char)237, (char)241, (char)39, (char)34, (char)207, (char)160, (char)185, (char)142, (char)64, (char)71, (char)233, (char)207, (char)69, (char)8, (char)149, (char)87, (char)246, (char)77, (char)149, (char)240, (char)2, (char)106, (char)79, (char)125, (char)225, (char)86, (char)112, (char)155, (char)237, (char)169, (char)223, (char)61, (char)64, (char)240, (char)44, (char)34, (char)212, (char)186, (char)24, (char)226, (char)188, (char)193, (char)73, (char)140, (char)3, (char)161, (char)208, (char)210, (char)12, (char)59, (char)63, (char)240, (char)145, (char)39, (char)99, (char)13, (char)227, (char)162, (char)3, (char)129, (char)21, (char)64, (char)81, (char)108, (char)203, (char)207, (char)233, (char)230, (char)2, (char)211, (char)27, (char)134, (char)146, (char)146, (char)192, (char)142, (char)65, (char)161, (char)12, (char)158, (char)21, (char)44, (char)208, (char)247, (char)126, (char)158, (char)92, (char)76, (char)216, (char)92, (char)142, (char)125}, 0) ;
        p248.target_network_SET((char)38) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.ver_GET() == (char)49);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)16, (byte) - 97, (byte)94, (byte) - 19, (byte) - 123, (byte)89, (byte)116, (byte)38, (byte) - 111, (byte) - 102, (byte) - 27, (byte) - 76, (byte) - 42, (byte) - 36, (byte)28, (byte) - 18, (byte) - 35, (byte) - 83, (byte) - 46, (byte) - 91, (byte) - 62, (byte)88, (byte) - 5, (byte) - 102, (byte)45, (byte) - 83, (byte)38, (byte)127, (byte) - 92, (byte) - 104, (byte) - 36, (byte)3}));
            assert(pack.address_GET() == (char)39831);
            assert(pack.type_GET() == (char)5);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.type_SET((char)5) ;
        p249.address_SET((char)39831) ;
        p249.ver_SET((char)49) ;
        p249.value_SET(new byte[] {(byte)16, (byte) - 97, (byte)94, (byte) - 19, (byte) - 123, (byte)89, (byte)116, (byte)38, (byte) - 111, (byte) - 102, (byte) - 27, (byte) - 76, (byte) - 42, (byte) - 36, (byte)28, (byte) - 18, (byte) - 35, (byte) - 83, (byte) - 46, (byte) - 91, (byte) - 62, (byte)88, (byte) - 5, (byte) - 102, (byte)45, (byte) - 83, (byte)38, (byte)127, (byte) - 92, (byte) - 104, (byte) - 36, (byte)3}, 0) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3567493191454685900L);
            assert(pack.name_LEN(ph) == 9);
            assert(pack.name_TRY(ph).equals("fxoKgbasg"));
            assert(pack.y_GET() == -2.7776936E38F);
            assert(pack.x_GET() == -1.9374581E38F);
            assert(pack.z_GET() == 1.2691881E38F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.x_SET(-1.9374581E38F) ;
        p250.name_SET("fxoKgbasg", PH) ;
        p250.y_SET(-2.7776936E38F) ;
        p250.z_SET(1.2691881E38F) ;
        p250.time_usec_SET(3567493191454685900L) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 2);
            assert(pack.name_TRY(ph).equals("Ae"));
            assert(pack.time_boot_ms_GET() == 3571734600L);
            assert(pack.value_GET() == 1.8171344E37F);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("Ae", PH) ;
        p251.time_boot_ms_SET(3571734600L) ;
        p251.value_SET(1.8171344E37F) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3730418359L);
            assert(pack.name_LEN(ph) == 5);
            assert(pack.name_TRY(ph).equals("kzobp"));
            assert(pack.value_GET() == -518038686);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(3730418359L) ;
        p252.name_SET("kzobp", PH) ;
        p252.value_SET(-518038686) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_NOTICE);
            assert(pack.text_LEN(ph) == 28);
            assert(pack.text_TRY(ph).equals("cpjxbgvarbhshcvbrObponbakbpt"));
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("cpjxbgvarbhshcvbrObponbakbpt", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_NOTICE) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.ind_GET() == (char)189);
            assert(pack.time_boot_ms_GET() == 2648344454L);
            assert(pack.value_GET() == -6.620747E37F);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(2648344454L) ;
        p254.ind_SET((char)189) ;
        p254.value_SET(-6.620747E37F) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)123, (char)210, (char)163, (char)215, (char)92, (char)97, (char)180, (char)34, (char)213, (char)133, (char)132, (char)21, (char)114, (char)119, (char)216, (char)207, (char)1, (char)214, (char)182, (char)12, (char)30, (char)226, (char)248, (char)208, (char)250, (char)199, (char)187, (char)252, (char)52, (char)202, (char)97, (char)204}));
            assert(pack.target_component_GET() == (char)17);
            assert(pack.initial_timestamp_GET() == 7563650315839713053L);
            assert(pack.target_system_GET() == (char)48);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_component_SET((char)17) ;
        p256.initial_timestamp_SET(7563650315839713053L) ;
        p256.target_system_SET((char)48) ;
        p256.secret_key_SET(new char[] {(char)123, (char)210, (char)163, (char)215, (char)92, (char)97, (char)180, (char)34, (char)213, (char)133, (char)132, (char)21, (char)114, (char)119, (char)216, (char)207, (char)1, (char)214, (char)182, (char)12, (char)30, (char)226, (char)248, (char)208, (char)250, (char)199, (char)187, (char)252, (char)52, (char)202, (char)97, (char)204}, 0) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.last_change_ms_GET() == 1144430811L);
            assert(pack.state_GET() == (char)44);
            assert(pack.time_boot_ms_GET() == 264645650L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.last_change_ms_SET(1144430811L) ;
        p257.time_boot_ms_SET(264645650L) ;
        p257.state_SET((char)44) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)80);
            assert(pack.tune_LEN(ph) == 30);
            assert(pack.tune_TRY(ph).equals("loegnlPloxIJPiFprghvmfltwimwrt"));
            assert(pack.target_system_GET() == (char)59);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)59) ;
        p258.target_component_SET((char)80) ;
        p258.tune_SET("loegnlPloxIJPiFprghvmfltwimwrt", PH) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.sensor_size_v_GET() == -3.3079134E38F);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)62, (char)1, (char)9, (char)157, (char)87, (char)166, (char)178, (char)60, (char)140, (char)63, (char)103, (char)137, (char)179, (char)108, (char)62, (char)205, (char)127, (char)88, (char)81, (char)143, (char)58, (char)8, (char)217, (char)223, (char)80, (char)43, (char)66, (char)133, (char)248, (char)144, (char)219, (char)2}));
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE));
            assert(pack.focal_length_GET() == -2.7391159E38F);
            assert(pack.cam_definition_uri_LEN(ph) == 125);
            assert(pack.cam_definition_uri_TRY(ph).equals("GapesrtkgvaovnsaqdpqwvMpdahpxoityxqwhiqjknuhchwfnioCeoHcdiUgnpbMoprmODaehqVvbfijzpcdkqkDwivebBpzobpdatjkramrcoyJfgipyyblnnLuS"));
            assert(pack.resolution_h_GET() == (char)33765);
            assert(pack.sensor_size_h_GET() == -1.2398136E38F);
            assert(pack.cam_definition_version_GET() == (char)40602);
            assert(pack.firmware_version_GET() == 3852862372L);
            assert(pack.lens_id_GET() == (char)189);
            assert(pack.time_boot_ms_GET() == 882762749L);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)121, (char)253, (char)100, (char)52, (char)176, (char)238, (char)143, (char)145, (char)123, (char)196, (char)137, (char)232, (char)136, (char)205, (char)243, (char)231, (char)66, (char)153, (char)171, (char)16, (char)169, (char)167, (char)214, (char)15, (char)163, (char)204, (char)9, (char)33, (char)133, (char)205, (char)153, (char)24}));
            assert(pack.resolution_v_GET() == (char)46506);
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.lens_id_SET((char)189) ;
        p259.vendor_name_SET(new char[] {(char)62, (char)1, (char)9, (char)157, (char)87, (char)166, (char)178, (char)60, (char)140, (char)63, (char)103, (char)137, (char)179, (char)108, (char)62, (char)205, (char)127, (char)88, (char)81, (char)143, (char)58, (char)8, (char)217, (char)223, (char)80, (char)43, (char)66, (char)133, (char)248, (char)144, (char)219, (char)2}, 0) ;
        p259.cam_definition_version_SET((char)40602) ;
        p259.time_boot_ms_SET(882762749L) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE)) ;
        p259.cam_definition_uri_SET("GapesrtkgvaovnsaqdpqwvMpdahpxoityxqwhiqjknuhchwfnioCeoHcdiUgnpbMoprmODaehqVvbfijzpcdkqkDwivebBpzobpdatjkramrcoyJfgipyyblnnLuS", PH) ;
        p259.sensor_size_v_SET(-3.3079134E38F) ;
        p259.resolution_v_SET((char)46506) ;
        p259.resolution_h_SET((char)33765) ;
        p259.model_name_SET(new char[] {(char)121, (char)253, (char)100, (char)52, (char)176, (char)238, (char)143, (char)145, (char)123, (char)196, (char)137, (char)232, (char)136, (char)205, (char)243, (char)231, (char)66, (char)153, (char)171, (char)16, (char)169, (char)167, (char)214, (char)15, (char)163, (char)204, (char)9, (char)33, (char)133, (char)205, (char)153, (char)24}, 0) ;
        p259.sensor_size_h_SET(-1.2398136E38F) ;
        p259.focal_length_SET(-2.7391159E38F) ;
        p259.firmware_version_SET(3852862372L) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1352873536L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(1352873536L) ;
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.status_GET() == (char)181);
            assert(pack.time_boot_ms_GET() == 2049069139L);
            assert(pack.storage_id_GET() == (char)112);
            assert(pack.available_capacity_GET() == 2.8673764E38F);
            assert(pack.total_capacity_GET() == 2.8436646E38F);
            assert(pack.read_speed_GET() == -2.478904E38F);
            assert(pack.storage_count_GET() == (char)26);
            assert(pack.write_speed_GET() == 2.6756013E38F);
            assert(pack.used_capacity_GET() == -3.2614096E38F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.used_capacity_SET(-3.2614096E38F) ;
        p261.total_capacity_SET(2.8436646E38F) ;
        p261.write_speed_SET(2.6756013E38F) ;
        p261.storage_count_SET((char)26) ;
        p261.available_capacity_SET(2.8673764E38F) ;
        p261.storage_id_SET((char)112) ;
        p261.time_boot_ms_SET(2049069139L) ;
        p261.status_SET((char)181) ;
        p261.read_speed_SET(-2.478904E38F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.video_status_GET() == (char)65);
            assert(pack.time_boot_ms_GET() == 3851689138L);
            assert(pack.image_status_GET() == (char)249);
            assert(pack.image_interval_GET() == -2.1582129E38F);
            assert(pack.available_capacity_GET() == 9.455785E37F);
            assert(pack.recording_time_ms_GET() == 2362761264L);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.recording_time_ms_SET(2362761264L) ;
        p262.available_capacity_SET(9.455785E37F) ;
        p262.video_status_SET((char)65) ;
        p262.image_interval_SET(-2.1582129E38F) ;
        p262.image_status_SET((char)249) ;
        p262.time_boot_ms_SET(3851689138L) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -596685687);
            assert(pack.image_index_GET() == -1833453173);
            assert(pack.file_url_LEN(ph) == 123);
            assert(pack.file_url_TRY(ph).equals("uebGybirfyYihisdxerdrjUdzxwbBUfemkiscWqejqanfquiqonlhodsnknluufnigqoeodsmhbceSwqzjjiwbdgdljaroivtSZnuifmggqxLygprJwrlpojhyp"));
            assert(pack.alt_GET() == -261859611);
            assert(pack.capture_result_GET() == (byte) - 42);
            assert(pack.camera_id_GET() == (char)205);
            assert(pack.time_utc_GET() == 2238981554851979891L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.3251923E38F, 8.828478E37F, -1.714999E38F, -2.2899613E38F}));
            assert(pack.relative_alt_GET() == 2146226590);
            assert(pack.lon_GET() == 307640353);
            assert(pack.time_boot_ms_GET() == 1142171695L);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.lon_SET(307640353) ;
        p263.image_index_SET(-1833453173) ;
        p263.lat_SET(-596685687) ;
        p263.file_url_SET("uebGybirfyYihisdxerdrjUdzxwbBUfemkiscWqejqanfquiqonlhodsnknluufnigqoeodsmhbceSwqzjjiwbdgdljaroivtSZnuifmggqxLygprJwrlpojhyp", PH) ;
        p263.q_SET(new float[] {3.3251923E38F, 8.828478E37F, -1.714999E38F, -2.2899613E38F}, 0) ;
        p263.camera_id_SET((char)205) ;
        p263.capture_result_SET((byte) - 42) ;
        p263.time_boot_ms_SET(1142171695L) ;
        p263.time_utc_SET(2238981554851979891L) ;
        p263.alt_SET(-261859611) ;
        p263.relative_alt_SET(2146226590) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.arming_time_utc_GET() == 921487381422240385L);
            assert(pack.flight_uuid_GET() == 9031757934382732945L);
            assert(pack.takeoff_time_utc_GET() == 4305386146254508817L);
            assert(pack.time_boot_ms_GET() == 3532279036L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.arming_time_utc_SET(921487381422240385L) ;
        p264.flight_uuid_SET(9031757934382732945L) ;
        p264.time_boot_ms_SET(3532279036L) ;
        p264.takeoff_time_utc_SET(4305386146254508817L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 9.012721E37F);
            assert(pack.time_boot_ms_GET() == 289965894L);
            assert(pack.pitch_GET() == -1.7943203E38F);
            assert(pack.roll_GET() == 1.4829729E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(1.4829729E38F) ;
        p265.pitch_SET(-1.7943203E38F) ;
        p265.time_boot_ms_SET(289965894L) ;
        p265.yaw_SET(9.012721E37F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)23, (char)194, (char)130, (char)45, (char)144, (char)136, (char)16, (char)227, (char)105, (char)56, (char)233, (char)155, (char)234, (char)221, (char)3, (char)45, (char)171, (char)186, (char)81, (char)192, (char)162, (char)33, (char)108, (char)77, (char)10, (char)234, (char)229, (char)175, (char)32, (char)172, (char)147, (char)174, (char)83, (char)78, (char)14, (char)233, (char)162, (char)232, (char)7, (char)219, (char)235, (char)151, (char)94, (char)100, (char)122, (char)132, (char)177, (char)192, (char)40, (char)172, (char)244, (char)211, (char)206, (char)174, (char)21, (char)48, (char)12, (char)126, (char)48, (char)46, (char)33, (char)66, (char)151, (char)180, (char)123, (char)104, (char)132, (char)126, (char)146, (char)201, (char)21, (char)91, (char)44, (char)174, (char)246, (char)211, (char)182, (char)74, (char)199, (char)173, (char)82, (char)113, (char)213, (char)12, (char)247, (char)172, (char)94, (char)151, (char)99, (char)175, (char)129, (char)224, (char)53, (char)154, (char)6, (char)217, (char)40, (char)48, (char)64, (char)217, (char)139, (char)46, (char)175, (char)26, (char)187, (char)223, (char)27, (char)127, (char)238, (char)195, (char)35, (char)171, (char)127, (char)250, (char)43, (char)97, (char)67, (char)59, (char)102, (char)205, (char)45, (char)77, (char)11, (char)57, (char)115, (char)141, (char)9, (char)91, (char)37, (char)27, (char)176, (char)4, (char)124, (char)100, (char)144, (char)196, (char)131, (char)145, (char)115, (char)225, (char)164, (char)253, (char)157, (char)140, (char)14, (char)16, (char)114, (char)216, (char)141, (char)156, (char)47, (char)83, (char)14, (char)67, (char)28, (char)139, (char)181, (char)81, (char)88, (char)114, (char)125, (char)157, (char)147, (char)131, (char)112, (char)45, (char)78, (char)70, (char)195, (char)66, (char)19, (char)255, (char)180, (char)217, (char)97, (char)72, (char)225, (char)99, (char)133, (char)10, (char)74, (char)34, (char)138, (char)97, (char)166, (char)173, (char)101, (char)201, (char)52, (char)179, (char)33, (char)82, (char)225, (char)224, (char)160, (char)247, (char)209, (char)53, (char)39, (char)112, (char)191, (char)124, (char)155, (char)79, (char)190, (char)173, (char)93, (char)233, (char)147, (char)168, (char)179, (char)228, (char)37, (char)110, (char)133, (char)211, (char)136, (char)171, (char)92, (char)182, (char)168, (char)123, (char)9, (char)217, (char)96, (char)39, (char)192, (char)13, (char)92, (char)165, (char)82, (char)175, (char)25, (char)39, (char)40, (char)97, (char)197, (char)244, (char)93, (char)62, (char)145, (char)63, (char)105, (char)69, (char)27, (char)164, (char)182, (char)184, (char)190}));
            assert(pack.target_system_GET() == (char)203);
            assert(pack.sequence_GET() == (char)44924);
            assert(pack.length_GET() == (char)196);
            assert(pack.target_component_GET() == (char)25);
            assert(pack.first_message_offset_GET() == (char)127);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.data__SET(new char[] {(char)23, (char)194, (char)130, (char)45, (char)144, (char)136, (char)16, (char)227, (char)105, (char)56, (char)233, (char)155, (char)234, (char)221, (char)3, (char)45, (char)171, (char)186, (char)81, (char)192, (char)162, (char)33, (char)108, (char)77, (char)10, (char)234, (char)229, (char)175, (char)32, (char)172, (char)147, (char)174, (char)83, (char)78, (char)14, (char)233, (char)162, (char)232, (char)7, (char)219, (char)235, (char)151, (char)94, (char)100, (char)122, (char)132, (char)177, (char)192, (char)40, (char)172, (char)244, (char)211, (char)206, (char)174, (char)21, (char)48, (char)12, (char)126, (char)48, (char)46, (char)33, (char)66, (char)151, (char)180, (char)123, (char)104, (char)132, (char)126, (char)146, (char)201, (char)21, (char)91, (char)44, (char)174, (char)246, (char)211, (char)182, (char)74, (char)199, (char)173, (char)82, (char)113, (char)213, (char)12, (char)247, (char)172, (char)94, (char)151, (char)99, (char)175, (char)129, (char)224, (char)53, (char)154, (char)6, (char)217, (char)40, (char)48, (char)64, (char)217, (char)139, (char)46, (char)175, (char)26, (char)187, (char)223, (char)27, (char)127, (char)238, (char)195, (char)35, (char)171, (char)127, (char)250, (char)43, (char)97, (char)67, (char)59, (char)102, (char)205, (char)45, (char)77, (char)11, (char)57, (char)115, (char)141, (char)9, (char)91, (char)37, (char)27, (char)176, (char)4, (char)124, (char)100, (char)144, (char)196, (char)131, (char)145, (char)115, (char)225, (char)164, (char)253, (char)157, (char)140, (char)14, (char)16, (char)114, (char)216, (char)141, (char)156, (char)47, (char)83, (char)14, (char)67, (char)28, (char)139, (char)181, (char)81, (char)88, (char)114, (char)125, (char)157, (char)147, (char)131, (char)112, (char)45, (char)78, (char)70, (char)195, (char)66, (char)19, (char)255, (char)180, (char)217, (char)97, (char)72, (char)225, (char)99, (char)133, (char)10, (char)74, (char)34, (char)138, (char)97, (char)166, (char)173, (char)101, (char)201, (char)52, (char)179, (char)33, (char)82, (char)225, (char)224, (char)160, (char)247, (char)209, (char)53, (char)39, (char)112, (char)191, (char)124, (char)155, (char)79, (char)190, (char)173, (char)93, (char)233, (char)147, (char)168, (char)179, (char)228, (char)37, (char)110, (char)133, (char)211, (char)136, (char)171, (char)92, (char)182, (char)168, (char)123, (char)9, (char)217, (char)96, (char)39, (char)192, (char)13, (char)92, (char)165, (char)82, (char)175, (char)25, (char)39, (char)40, (char)97, (char)197, (char)244, (char)93, (char)62, (char)145, (char)63, (char)105, (char)69, (char)27, (char)164, (char)182, (char)184, (char)190}, 0) ;
        p266.first_message_offset_SET((char)127) ;
        p266.length_SET((char)196) ;
        p266.target_component_SET((char)25) ;
        p266.target_system_SET((char)203) ;
        p266.sequence_SET((char)44924) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)21);
            assert(pack.first_message_offset_GET() == (char)69);
            assert(pack.target_system_GET() == (char)138);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)237, (char)43, (char)190, (char)123, (char)51, (char)98, (char)154, (char)22, (char)146, (char)192, (char)132, (char)89, (char)102, (char)128, (char)53, (char)83, (char)234, (char)134, (char)215, (char)40, (char)208, (char)122, (char)110, (char)44, (char)165, (char)67, (char)35, (char)37, (char)27, (char)113, (char)175, (char)244, (char)246, (char)35, (char)103, (char)21, (char)10, (char)30, (char)36, (char)107, (char)244, (char)219, (char)154, (char)38, (char)157, (char)137, (char)242, (char)195, (char)76, (char)48, (char)121, (char)220, (char)27, (char)202, (char)195, (char)101, (char)111, (char)100, (char)135, (char)24, (char)190, (char)29, (char)16, (char)66, (char)9, (char)239, (char)126, (char)90, (char)222, (char)217, (char)91, (char)113, (char)154, (char)237, (char)161, (char)55, (char)90, (char)65, (char)248, (char)132, (char)213, (char)134, (char)103, (char)118, (char)81, (char)148, (char)119, (char)245, (char)8, (char)171, (char)57, (char)201, (char)136, (char)22, (char)97, (char)69, (char)45, (char)223, (char)136, (char)78, (char)24, (char)57, (char)27, (char)110, (char)72, (char)105, (char)191, (char)86, (char)37, (char)166, (char)71, (char)155, (char)45, (char)178, (char)8, (char)157, (char)52, (char)254, (char)84, (char)136, (char)161, (char)137, (char)159, (char)252, (char)124, (char)183, (char)7, (char)7, (char)88, (char)125, (char)177, (char)46, (char)6, (char)35, (char)99, (char)117, (char)44, (char)134, (char)125, (char)35, (char)60, (char)179, (char)146, (char)197, (char)19, (char)134, (char)147, (char)72, (char)83, (char)138, (char)22, (char)189, (char)197, (char)100, (char)47, (char)158, (char)27, (char)32, (char)55, (char)215, (char)130, (char)25, (char)46, (char)158, (char)104, (char)202, (char)95, (char)126, (char)212, (char)57, (char)211, (char)148, (char)192, (char)100, (char)153, (char)183, (char)183, (char)221, (char)69, (char)138, (char)220, (char)130, (char)122, (char)213, (char)7, (char)52, (char)119, (char)101, (char)190, (char)174, (char)172, (char)172, (char)109, (char)86, (char)101, (char)94, (char)229, (char)76, (char)114, (char)19, (char)87, (char)99, (char)72, (char)165, (char)74, (char)248, (char)165, (char)221, (char)170, (char)123, (char)174, (char)182, (char)189, (char)32, (char)198, (char)208, (char)189, (char)236, (char)48, (char)121, (char)244, (char)131, (char)238, (char)250, (char)16, (char)17, (char)141, (char)204, (char)216, (char)37, (char)90, (char)113, (char)79, (char)252, (char)95, (char)234, (char)85, (char)16, (char)137, (char)101, (char)85, (char)210, (char)17, (char)10, (char)4, (char)222, (char)216, (char)250, (char)235}));
            assert(pack.length_GET() == (char)188);
            assert(pack.sequence_GET() == (char)30295);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.data__SET(new char[] {(char)237, (char)43, (char)190, (char)123, (char)51, (char)98, (char)154, (char)22, (char)146, (char)192, (char)132, (char)89, (char)102, (char)128, (char)53, (char)83, (char)234, (char)134, (char)215, (char)40, (char)208, (char)122, (char)110, (char)44, (char)165, (char)67, (char)35, (char)37, (char)27, (char)113, (char)175, (char)244, (char)246, (char)35, (char)103, (char)21, (char)10, (char)30, (char)36, (char)107, (char)244, (char)219, (char)154, (char)38, (char)157, (char)137, (char)242, (char)195, (char)76, (char)48, (char)121, (char)220, (char)27, (char)202, (char)195, (char)101, (char)111, (char)100, (char)135, (char)24, (char)190, (char)29, (char)16, (char)66, (char)9, (char)239, (char)126, (char)90, (char)222, (char)217, (char)91, (char)113, (char)154, (char)237, (char)161, (char)55, (char)90, (char)65, (char)248, (char)132, (char)213, (char)134, (char)103, (char)118, (char)81, (char)148, (char)119, (char)245, (char)8, (char)171, (char)57, (char)201, (char)136, (char)22, (char)97, (char)69, (char)45, (char)223, (char)136, (char)78, (char)24, (char)57, (char)27, (char)110, (char)72, (char)105, (char)191, (char)86, (char)37, (char)166, (char)71, (char)155, (char)45, (char)178, (char)8, (char)157, (char)52, (char)254, (char)84, (char)136, (char)161, (char)137, (char)159, (char)252, (char)124, (char)183, (char)7, (char)7, (char)88, (char)125, (char)177, (char)46, (char)6, (char)35, (char)99, (char)117, (char)44, (char)134, (char)125, (char)35, (char)60, (char)179, (char)146, (char)197, (char)19, (char)134, (char)147, (char)72, (char)83, (char)138, (char)22, (char)189, (char)197, (char)100, (char)47, (char)158, (char)27, (char)32, (char)55, (char)215, (char)130, (char)25, (char)46, (char)158, (char)104, (char)202, (char)95, (char)126, (char)212, (char)57, (char)211, (char)148, (char)192, (char)100, (char)153, (char)183, (char)183, (char)221, (char)69, (char)138, (char)220, (char)130, (char)122, (char)213, (char)7, (char)52, (char)119, (char)101, (char)190, (char)174, (char)172, (char)172, (char)109, (char)86, (char)101, (char)94, (char)229, (char)76, (char)114, (char)19, (char)87, (char)99, (char)72, (char)165, (char)74, (char)248, (char)165, (char)221, (char)170, (char)123, (char)174, (char)182, (char)189, (char)32, (char)198, (char)208, (char)189, (char)236, (char)48, (char)121, (char)244, (char)131, (char)238, (char)250, (char)16, (char)17, (char)141, (char)204, (char)216, (char)37, (char)90, (char)113, (char)79, (char)252, (char)95, (char)234, (char)85, (char)16, (char)137, (char)101, (char)85, (char)210, (char)17, (char)10, (char)4, (char)222, (char)216, (char)250, (char)235}, 0) ;
        p267.length_SET((char)188) ;
        p267.target_component_SET((char)21) ;
        p267.first_message_offset_SET((char)69) ;
        p267.sequence_SET((char)30295) ;
        p267.target_system_SET((char)138) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)100);
            assert(pack.sequence_GET() == (char)60159);
            assert(pack.target_system_GET() == (char)87);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.sequence_SET((char)60159) ;
        p268.target_component_SET((char)100) ;
        p268.target_system_SET((char)87) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.framerate_GET() == 1.2320638E37F);
            assert(pack.uri_LEN(ph) == 35);
            assert(pack.uri_TRY(ph).equals("BmduwctpozahybmjzkmAgirvntuijfitvbk"));
            assert(pack.camera_id_GET() == (char)122);
            assert(pack.resolution_v_GET() == (char)19656);
            assert(pack.rotation_GET() == (char)31469);
            assert(pack.status_GET() == (char)80);
            assert(pack.bitrate_GET() == 2564616862L);
            assert(pack.resolution_h_GET() == (char)22650);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.resolution_h_SET((char)22650) ;
        p269.status_SET((char)80) ;
        p269.framerate_SET(1.2320638E37F) ;
        p269.resolution_v_SET((char)19656) ;
        p269.bitrate_SET(2564616862L) ;
        p269.rotation_SET((char)31469) ;
        p269.uri_SET("BmduwctpozahybmjzkmAgirvntuijfitvbk", PH) ;
        p269.camera_id_SET((char)122) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.framerate_GET() == 1.1487772E38F);
            assert(pack.target_system_GET() == (char)72);
            assert(pack.rotation_GET() == (char)3878);
            assert(pack.uri_LEN(ph) == 88);
            assert(pack.uri_TRY(ph).equals("rDspipkqvzTiufsSonakimslangnfxcyskjlgKpvwdCrldNufwgvciynjaufnmkqbjFwcqcwpsfUimDxinfalarw"));
            assert(pack.bitrate_GET() == 684676453L);
            assert(pack.target_component_GET() == (char)115);
            assert(pack.resolution_h_GET() == (char)10520);
            assert(pack.camera_id_GET() == (char)137);
            assert(pack.resolution_v_GET() == (char)24822);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.bitrate_SET(684676453L) ;
        p270.rotation_SET((char)3878) ;
        p270.framerate_SET(1.1487772E38F) ;
        p270.target_component_SET((char)115) ;
        p270.camera_id_SET((char)137) ;
        p270.resolution_v_SET((char)24822) ;
        p270.resolution_h_SET((char)10520) ;
        p270.uri_SET("rDspipkqvzTiufsSonakimslangnfxcyskjlgKpvwdCrldNufwgvciynjaufnmkqbjFwcqcwpsfUimDxinfalarw", PH) ;
        p270.target_system_SET((char)72) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 28);
            assert(pack.password_TRY(ph).equals("qQuMdogxheltThyvgoZrivObunrw"));
            assert(pack.ssid_LEN(ph) == 31);
            assert(pack.ssid_TRY(ph).equals("pEmfefnbvZqbrzBkLhcpuwloetJdduY"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("pEmfefnbvZqbrzBkLhcpuwloetJdduY", PH) ;
        p299.password_SET("qQuMdogxheltThyvgoZrivObunrw", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)190, (char)88, (char)116, (char)213, (char)224, (char)193, (char)81, (char)30}));
            assert(pack.max_version_GET() == (char)20467);
            assert(pack.min_version_GET() == (char)52739);
            assert(pack.version_GET() == (char)3987);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)53, (char)11, (char)1, (char)4, (char)72, (char)101, (char)219, (char)200}));
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.max_version_SET((char)20467) ;
        p300.library_version_hash_SET(new char[] {(char)190, (char)88, (char)116, (char)213, (char)224, (char)193, (char)81, (char)30}, 0) ;
        p300.spec_version_hash_SET(new char[] {(char)53, (char)11, (char)1, (char)4, (char)72, (char)101, (char)219, (char)200}, 0) ;
        p300.min_version_SET((char)52739) ;
        p300.version_SET((char)3987) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.vendor_specific_status_code_GET() == (char)39185);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
            assert(pack.time_usec_GET() == 8569231476107492273L);
            assert(pack.uptime_sec_GET() == 1968039561L);
            assert(pack.sub_mode_GET() == (char)39);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.vendor_specific_status_code_SET((char)39185) ;
        p310.sub_mode_SET((char)39) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL) ;
        p310.time_usec_SET(8569231476107492273L) ;
        p310.uptime_sec_SET(1968039561L) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.sw_version_major_GET() == (char)14);
            assert(pack.uptime_sec_GET() == 3386717013L);
            assert(pack.sw_version_minor_GET() == (char)150);
            assert(pack.hw_version_minor_GET() == (char)189);
            assert(pack.name_LEN(ph) == 39);
            assert(pack.name_TRY(ph).equals("jxkawhasFfdqgdkudaBnracmutphnbylodyNmsx"));
            assert(pack.hw_version_major_GET() == (char)177);
            assert(pack.time_usec_GET() == 2868471147941820055L);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)92, (char)250, (char)218, (char)237, (char)46, (char)123, (char)209, (char)59, (char)80, (char)195, (char)208, (char)127, (char)49, (char)177, (char)208, (char)72}));
            assert(pack.sw_vcs_commit_GET() == 3151482848L);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_version_major_SET((char)177) ;
        p311.sw_version_minor_SET((char)150) ;
        p311.sw_vcs_commit_SET(3151482848L) ;
        p311.hw_unique_id_SET(new char[] {(char)92, (char)250, (char)218, (char)237, (char)46, (char)123, (char)209, (char)59, (char)80, (char)195, (char)208, (char)127, (char)49, (char)177, (char)208, (char)72}, 0) ;
        p311.name_SET("jxkawhasFfdqgdkudaBnracmutphnbylodyNmsx", PH) ;
        p311.sw_version_major_SET((char)14) ;
        p311.hw_version_minor_SET((char)189) ;
        p311.time_usec_SET(2868471147941820055L) ;
        p311.uptime_sec_SET(3386717013L) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("eyx"));
            assert(pack.target_system_GET() == (char)91);
            assert(pack.param_index_GET() == (short) -11543);
            assert(pack.target_component_GET() == (char)193);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_index_SET((short) -11543) ;
        p320.target_system_SET((char)91) ;
        p320.target_component_SET((char)193) ;
        p320.param_id_SET("eyx", PH) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)63);
            assert(pack.target_component_GET() == (char)173);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)173) ;
        p321.target_system_SET((char)63) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("ie"));
            assert(pack.param_index_GET() == (char)47180);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
            assert(pack.param_value_LEN(ph) == 115);
            assert(pack.param_value_TRY(ph).equals("cdOnupfqjrajnkqgcohQllcbbpayjoDfiBzJntjvdEzwKvrgwbljPlfloigldcFnVwipXouoottxxzkkhogunxwauphJnivbpkjdyJrgukkligzsuCf"));
            assert(pack.param_count_GET() == (char)24223);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("ie", PH) ;
        p322.param_count_SET((char)24223) ;
        p322.param_value_SET("cdOnupfqjrajnkqgcohQllcbbpayjoDfiBzJntjvdEzwKvrgwbljPlfloigldcFnVwipXouoottxxzkkhogunxwauphJnivbpkjdyJrgukkligzsuCf", PH) ;
        p322.param_index_SET((char)47180) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)159);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("wqwdNypn"));
            assert(pack.target_system_GET() == (char)227);
            assert(pack.param_value_LEN(ph) == 100);
            assert(pack.param_value_TRY(ph).equals("vvrvzbiosiGnJnmwaxuozjwkoqVAgarmVgnzrfdorfrumPyqbypfdrijdhclmbfledxjoerxpwgWztjuggjibsomndtcdyqdorit"));
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64) ;
        p323.target_system_SET((char)227) ;
        p323.target_component_SET((char)159) ;
        p323.param_id_SET("wqwdNypn", PH) ;
        p323.param_value_SET("vvrvzbiosiGnJnmwaxuozjwkoqVAgarmVgnzrfdorfrumPyqbypfdrijdhclmbfledxjoerxpwgWztjuggjibsomndtcdyqdorit", PH) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 33);
            assert(pack.param_value_TRY(ph).equals("itktaexirbpzhserkwmhEyulkqfpoupwo"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("cvzuy"));
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM) ;
        p324.param_id_SET("cvzuy", PH) ;
        p324.param_value_SET("itktaexirbpzhserkwmhEyulkqfpoupwo", PH) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)55105, (char)19417, (char)12618, (char)8658, (char)4841, (char)17373, (char)54583, (char)44748, (char)29670, (char)30762, (char)44249, (char)56910, (char)30215, (char)52059, (char)3611, (char)63556, (char)30290, (char)10936, (char)49186, (char)52867, (char)12242, (char)43382, (char)24806, (char)21380, (char)7466, (char)9250, (char)19748, (char)29594, (char)2386, (char)55871, (char)17544, (char)56566, (char)33344, (char)4622, (char)26376, (char)12927, (char)37150, (char)17405, (char)2077, (char)47672, (char)57123, (char)26062, (char)10527, (char)40387, (char)62279, (char)9782, (char)50534, (char)40434, (char)53594, (char)52282, (char)42008, (char)56695, (char)16638, (char)7358, (char)22725, (char)51701, (char)56375, (char)62708, (char)34779, (char)26988, (char)55866, (char)53459, (char)20551, (char)57154, (char)32204, (char)41825, (char)4653, (char)8869, (char)65297, (char)1352, (char)5224, (char)32003}));
            assert(pack.time_usec_GET() == 7690769932587955858L);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.max_distance_GET() == (char)64894);
            assert(pack.increment_GET() == (char)8);
            assert(pack.min_distance_GET() == (char)17221);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.min_distance_SET((char)17221) ;
        p330.max_distance_SET((char)64894) ;
        p330.distances_SET(new char[] {(char)55105, (char)19417, (char)12618, (char)8658, (char)4841, (char)17373, (char)54583, (char)44748, (char)29670, (char)30762, (char)44249, (char)56910, (char)30215, (char)52059, (char)3611, (char)63556, (char)30290, (char)10936, (char)49186, (char)52867, (char)12242, (char)43382, (char)24806, (char)21380, (char)7466, (char)9250, (char)19748, (char)29594, (char)2386, (char)55871, (char)17544, (char)56566, (char)33344, (char)4622, (char)26376, (char)12927, (char)37150, (char)17405, (char)2077, (char)47672, (char)57123, (char)26062, (char)10527, (char)40387, (char)62279, (char)9782, (char)50534, (char)40434, (char)53594, (char)52282, (char)42008, (char)56695, (char)16638, (char)7358, (char)22725, (char)51701, (char)56375, (char)62708, (char)34779, (char)26988, (char)55866, (char)53459, (char)20551, (char)57154, (char)32204, (char)41825, (char)4653, (char)8869, (char)65297, (char)1352, (char)5224, (char)32003}, 0) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p330.increment_SET((char)8) ;
        p330.time_usec_SET(7690769932587955858L) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
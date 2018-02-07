
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
            long id = id__U(src);
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
                default:
                    assert(false);//("Unknown enum" + id);
            }
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
                default:
                    assert(false);//("Unknown enum" + id);
            }
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
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 7, data, 260);
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
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 7, data, 248);
        }
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
    public static class ARRAY_TEST_0 extends GroundControl.ARRAY_TEST_0
    {
        public char[] ar_u16_GET(char[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] ar_u16_GET()//Value array
        {return ar_u16_GET(new char[4], 0);} public long[] ar_u32_GET(long[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 8, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (get_bytes(data,  BYTE, 4));
            return dst_ch;
        }
        public long[] ar_u32_GET()//Value array
        {return ar_u32_GET(new long[4], 0);} public char v1_GET()//Stub field
        {  return (char)((char) get_bytes(data,  24, 1)); }
        public byte[] ar_i8_GET(byte[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 25, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (byte)((byte) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public byte[] ar_i8_GET()//Value array
        {return ar_i8_GET(new byte[4], 0);} public char[] ar_u8_GET(char[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 29, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] ar_u8_GET()//Value array
        {return ar_u8_GET(new char[4], 0);}
    }
    public static class ARRAY_TEST_1 extends GroundControl.ARRAY_TEST_1
    {
        public long[] ar_u32_GET(long[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (get_bytes(data,  BYTE, 4));
            return dst_ch;
        }
        public long[] ar_u32_GET()//Value array
        {return ar_u32_GET(new long[4], 0);}
    }
    public static class ARRAY_TEST_3 extends GroundControl.ARRAY_TEST_3
    {
        public long[] ar_u32_GET(long[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (get_bytes(data,  BYTE, 4));
            return dst_ch;
        }
        public long[] ar_u32_GET()//Value array
        {return ar_u32_GET(new long[4], 0);} public char v_GET()//Stub field
        {  return (char)((char) get_bytes(data,  16, 1)); }
    }
    public static class ARRAY_TEST_4 extends GroundControl.ARRAY_TEST_4
    {
        public long[] ar_u32_GET(long[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (get_bytes(data,  BYTE, 4));
            return dst_ch;
        }
        public long[] ar_u32_GET()//Value array
        {return ar_u32_GET(new long[4], 0);} public char v_GET()//Stub field
        {  return (char)((char) get_bytes(data,  16, 1)); }
    }
    public static class ARRAY_TEST_5 extends GroundControl.ARRAY_TEST_5
    {
        public String c1_TRY(Bounds.Inside ph)//Value array
        {
            if(ph.field_bit !=  0 && !try_visit_field(ph, 0)  ||  !try_visit_item(ph, 0)) return null;
            return new String(c1_GET(ph, new char[ph.items], 0));
        }
        public char[] c1_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Value array
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int c1_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  0 && !try_visit_field(ph, 0)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public String c2_TRY(Bounds.Inside ph)//Value array
        {
            if(ph.field_bit !=  1 && !try_visit_field(ph, 1)  ||  !try_visit_item(ph, 0)) return null;
            return new String(c2_GET(ph, new char[ph.items], 0));
        }
        public char[] c2_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Value array
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int c2_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  1 && !try_visit_field(ph, 1)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class ARRAY_TEST_6 extends GroundControl.ARRAY_TEST_6
    {
        public char v2_GET()//Stub field
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char[] ar_u16_GET(char[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 2, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] ar_u16_GET()//Value array
        {return ar_u16_GET(new char[2], 0);} public long v3_GET()//Stub field
        {  return (get_bytes(data,  6, 4)); }
        public long[] ar_u32_GET(long[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 10, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (get_bytes(data,  BYTE, 4));
            return dst_ch;
        }
        public long[] ar_u32_GET()//Value array
        {return ar_u32_GET(new long[2], 0);} public char v1_GET()//Stub field
        {  return (char)((char) get_bytes(data,  18, 1)); }
        public int[] ar_i32_GET(int[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 19, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (int)((int) get_bytes(data,  BYTE, 4));
            return dst_ch;
        }
        public int[] ar_i32_GET()//Value array
        {return ar_i32_GET(new int[2], 0);} public short[] ar_i16_GET(short[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 27, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (short)((short) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public short[] ar_i16_GET()//Value array
        {return ar_i16_GET(new short[2], 0);} public char[] ar_u8_GET(char[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 31, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] ar_u8_GET()//Value array
        {return ar_u8_GET(new char[2], 0);} public byte[] ar_i8_GET(byte[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 33, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (byte)((byte) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public byte[] ar_i8_GET()//Value array
        {return ar_i8_GET(new byte[2], 0);} public double[] ar_d_GET(double[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 35, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 8)
                dst_ch[pos] = (double)(Double.longBitsToDouble(get_bytes(data, BYTE, 8)));
            return dst_ch;
        }
        public double[] ar_d_GET()//Value array
        {return ar_d_GET(new double[2], 0);} public float[] ar_f_GET(float[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 51, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] ar_f_GET()//Value array
        {return ar_f_GET(new float[2], 0);} public String ar_c_TRY(Bounds.Inside ph) //Value array
        {
            if(ph.field_bit !=  472 && !try_visit_field(ph, 472)  ||  !try_visit_item(ph, 0)) return null;
            return new String(ar_c_GET(ph, new char[ph.items], 0));
        }
        public char[] ar_c_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Value array
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int ar_c_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  472 && !try_visit_field(ph, 472)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class ARRAY_TEST_7 extends GroundControl.ARRAY_TEST_7
    {
        public char[] ar_u16_GET(char[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 0, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] ar_u16_GET()//Value array
        {return ar_u16_GET(new char[2], 0);} public long[] ar_u32_GET(long[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 4, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (get_bytes(data,  BYTE, 4));
            return dst_ch;
        }
        public long[] ar_u32_GET()//Value array
        {return ar_u32_GET(new long[2], 0);} public double[] ar_d_GET(double[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 12, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 8)
                dst_ch[pos] = (double)(Double.longBitsToDouble(get_bytes(data, BYTE, 8)));
            return dst_ch;
        }
        public double[] ar_d_GET()//Value array
        {return ar_d_GET(new double[2], 0);} public float[] ar_f_GET(float[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 28, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] ar_f_GET()//Value array
        {return ar_f_GET(new float[2], 0);} public int[] ar_i32_GET(int[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 36, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (int)((int) get_bytes(data,  BYTE, 4));
            return dst_ch;
        }
        public int[] ar_i32_GET()//Value array
        {return ar_i32_GET(new int[2], 0);} public short[] ar_i16_GET(short[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 44, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (short)((short) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public short[] ar_i16_GET()//Value array
        {return ar_i16_GET(new short[2], 0);} public char[] ar_u8_GET(char[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 48, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] ar_u8_GET()//Value array
        {return ar_u8_GET(new char[2], 0);} public byte[] ar_i8_GET(byte[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 50, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (byte)((byte) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public byte[] ar_i8_GET()//Value array
        {return ar_i8_GET(new byte[2], 0);} public String ar_c_TRY(Bounds.Inside ph) //Value array
        {
            if(ph.field_bit !=  416 && !try_visit_field(ph, 416)  ||  !try_visit_item(ph, 0)) return null;
            return new String(ar_c_GET(ph, new char[ph.items], 0));
        }
        public char[] ar_c_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Value array
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int ar_c_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  416 && !try_visit_field(ph, 416)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class ARRAY_TEST_8 extends GroundControl.ARRAY_TEST_8
    {
        public char[] ar_u16_GET(char[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 0, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] ar_u16_GET()//Value array
        {return ar_u16_GET(new char[2], 0);} public long v3_GET()//Stub field
        {  return (get_bytes(data,  4, 4)); }
        public double[] ar_d_GET(double[]  dst_ch, int pos)  //Value array
        {
            for(int BYTE = 8, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 8)
                dst_ch[pos] = (double)(Double.longBitsToDouble(get_bytes(data, BYTE, 8)));
            return dst_ch;
        }
        public double[] ar_d_GET()//Value array
        {return ar_d_GET(new double[2], 0);}
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

        static final Collection<OnReceive.Handler<FOLLOW_TARGET, Channel>> on_FOLLOW_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<CONTROL_SYSTEM_STATE, Channel>> on_CONTROL_SYSTEM_STATE = new OnReceive<>();
        static final Collection<OnReceive.Handler<BATTERY_STATUS, Channel>> on_BATTERY_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<AUTOPILOT_VERSION, Channel>> on_AUTOPILOT_VERSION = new OnReceive<>();
        static final Collection<OnReceive.Handler<LANDING_TARGET, Channel>> on_LANDING_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<ARRAY_TEST_0, Channel>> on_ARRAY_TEST_0 = new OnReceive<>();
        static final Collection<OnReceive.Handler<ARRAY_TEST_1, Channel>> on_ARRAY_TEST_1 = new OnReceive<>();
        static final Collection<OnReceive.Handler<ARRAY_TEST_3, Channel>> on_ARRAY_TEST_3 = new OnReceive<>();
        static final Collection<OnReceive.Handler<ARRAY_TEST_4, Channel>> on_ARRAY_TEST_4 = new OnReceive<>();
        static final Collection<OnReceive.Handler<ARRAY_TEST_5, Channel>> on_ARRAY_TEST_5 = new OnReceive<>();
        static final Collection<OnReceive.Handler<ARRAY_TEST_6, Channel>> on_ARRAY_TEST_6 = new OnReceive<>();
        static final Collection<OnReceive.Handler<ARRAY_TEST_7, Channel>> on_ARRAY_TEST_7 = new OnReceive<>();
        static final Collection<OnReceive.Handler<ARRAY_TEST_8, Channel>> on_ARRAY_TEST_8 = new OnReceive<>();
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
                    if(pack == null) return new ARRAY_TEST_0();
                    ((OnReceive) on_ARRAY_TEST_0).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 151:
                    if(pack == null) return new ARRAY_TEST_1();
                    ((OnReceive) on_ARRAY_TEST_1).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 153:
                    if(pack == null) return new ARRAY_TEST_3();
                    ((OnReceive) on_ARRAY_TEST_3).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 154:
                    if(pack == null) return new ARRAY_TEST_4();
                    ((OnReceive) on_ARRAY_TEST_4).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 155:
                    if(pack == null) return new ARRAY_TEST_5();
                    ((OnReceive) on_ARRAY_TEST_5).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 156:
                    if(pack == null) return new ARRAY_TEST_6();
                    ((OnReceive) on_ARRAY_TEST_6).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 157:
                    if(pack == null) return new ARRAY_TEST_7();
                    ((OnReceive) on_ARRAY_TEST_7).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 158:
                    if(pack == null) return new ARRAY_TEST_8();
                    ((OnReceive) on_ARRAY_TEST_8).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
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
            assert(pack.custom_mode_GET() == 69882143L);
            assert(pack.mavlink_version_GET() == (char)123);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED));
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_EMERGENCY);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_SUBMARINE);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_UDB);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED)) ;
        p0.custom_mode_SET(69882143L) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_SUBMARINE) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_EMERGENCY) ;
        p0.mavlink_version_SET((char)123) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_UDB) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_count2_GET() == (char)24373);
            assert(pack.errors_comm_GET() == (char)53535);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH));
            assert(pack.drop_rate_comm_GET() == (char)54089);
            assert(pack.errors_count1_GET() == (char)32780);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
            assert(pack.current_battery_GET() == (short) -1749);
            assert(pack.errors_count3_GET() == (char)19158);
            assert(pack.battery_remaining_GET() == (byte)47);
            assert(pack.voltage_battery_GET() == (char)28560);
            assert(pack.errors_count4_GET() == (char)9075);
            assert(pack.load_GET() == (char)46325);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.errors_count2_SET((char)24373) ;
        p1.errors_count1_SET((char)32780) ;
        p1.errors_count3_SET((char)19158) ;
        p1.current_battery_SET((short) -1749) ;
        p1.battery_remaining_SET((byte)47) ;
        p1.errors_comm_SET((char)53535) ;
        p1.voltage_battery_SET((char)28560) ;
        p1.errors_count4_SET((char)9075) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.drop_rate_comm_SET((char)54089) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.load_SET((char)46325) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH)) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 5852205543677792817L);
            assert(pack.time_boot_ms_GET() == 2033411960L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(5852205543677792817L) ;
        p2.time_boot_ms_SET(2033411960L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 1.2978214E38F);
            assert(pack.yaw_rate_GET() == 8.1526773E37F);
            assert(pack.afz_GET() == 7.479541E37F);
            assert(pack.afx_GET() == 1.616434E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.y_GET() == 1.9563252E38F);
            assert(pack.vz_GET() == -2.9592464E38F);
            assert(pack.yaw_GET() == -1.979036E38F);
            assert(pack.afy_GET() == -2.9427456E38F);
            assert(pack.type_mask_GET() == (char)25234);
            assert(pack.z_GET() == -1.596575E38F);
            assert(pack.time_boot_ms_GET() == 992197839L);
            assert(pack.vy_GET() == 5.153276E37F);
            assert(pack.vx_GET() == 5.6442716E37F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.type_mask_SET((char)25234) ;
        p3.time_boot_ms_SET(992197839L) ;
        p3.afx_SET(1.616434E38F) ;
        p3.yaw_rate_SET(8.1526773E37F) ;
        p3.afz_SET(7.479541E37F) ;
        p3.y_SET(1.9563252E38F) ;
        p3.vy_SET(5.153276E37F) ;
        p3.afy_SET(-2.9427456E38F) ;
        p3.vz_SET(-2.9592464E38F) ;
        p3.vx_SET(5.6442716E37F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p3.x_SET(1.2978214E38F) ;
        p3.yaw_SET(-1.979036E38F) ;
        p3.z_SET(-1.596575E38F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 141292654663751249L);
            assert(pack.seq_GET() == 3306856219L);
            assert(pack.target_component_GET() == (char)241);
            assert(pack.target_system_GET() == (char)92);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.seq_SET(3306856219L) ;
        p4.time_usec_SET(141292654663751249L) ;
        p4.target_component_SET((char)241) ;
        p4.target_system_SET((char)92) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)199);
            assert(pack.passkey_LEN(ph) == 5);
            assert(pack.passkey_TRY(ph).equals("hhxnn"));
            assert(pack.version_GET() == (char)174);
            assert(pack.target_system_GET() == (char)163);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.version_SET((char)174) ;
        p5.passkey_SET("hhxnn", PH) ;
        p5.target_system_SET((char)163) ;
        p5.control_request_SET((char)199) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)128);
            assert(pack.control_request_GET() == (char)145);
            assert(pack.ack_GET() == (char)144);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.ack_SET((char)144) ;
        p6.control_request_SET((char)145) ;
        p6.gcs_system_id_SET((char)128) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 28);
            assert(pack.key_TRY(ph).equals("igvsfsosogikmwpOgHzhsulntcxm"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("igvsfsosogikmwpOgHzhsulntcxm", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_GUIDED_ARMED);
            assert(pack.custom_mode_GET() == 3392285422L);
            assert(pack.target_system_GET() == (char)62);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.custom_mode_SET(3392285422L) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED) ;
        p11.target_system_SET((char)62) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short) -286);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("qGhrJlla"));
            assert(pack.target_component_GET() == (char)243);
            assert(pack.target_system_GET() == (char)25);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_id_SET("qGhrJlla", PH) ;
        p20.param_index_SET((short) -286) ;
        p20.target_system_SET((char)25) ;
        p20.target_component_SET((char)243) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)192);
            assert(pack.target_component_GET() == (char)148);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)148) ;
        p21.target_system_SET((char)192) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("jbdy"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
            assert(pack.param_index_GET() == (char)31642);
            assert(pack.param_count_GET() == (char)45053);
            assert(pack.param_value_GET() == 2.711207E38F);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16) ;
        p22.param_index_SET((char)31642) ;
        p22.param_value_SET(2.711207E38F) ;
        p22.param_count_SET((char)45053) ;
        p22.param_id_SET("jbdy", PH) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("zy"));
            assert(pack.target_system_GET() == (char)141);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
            assert(pack.param_value_GET() == -2.8118032E37F);
            assert(pack.target_component_GET() == (char)42);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64) ;
        p23.param_value_SET(-2.8118032E37F) ;
        p23.target_component_SET((char)42) ;
        p23.param_id_SET("zy", PH) ;
        p23.target_system_SET((char)141) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.h_acc_TRY(ph) == 2239304292L);
            assert(pack.time_usec_GET() == 8705021767288521963L);
            assert(pack.lon_GET() == 972055936);
            assert(pack.alt_ellipsoid_TRY(ph) == -461426806);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
            assert(pack.epv_GET() == (char)11402);
            assert(pack.satellites_visible_GET() == (char)24);
            assert(pack.vel_acc_TRY(ph) == 2356101151L);
            assert(pack.v_acc_TRY(ph) == 1221618890L);
            assert(pack.lat_GET() == -1250172536);
            assert(pack.cog_GET() == (char)28307);
            assert(pack.vel_GET() == (char)37747);
            assert(pack.alt_GET() == -882120408);
            assert(pack.hdg_acc_TRY(ph) == 2976063437L);
            assert(pack.eph_GET() == (char)33886);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.satellites_visible_SET((char)24) ;
        p24.epv_SET((char)11402) ;
        p24.v_acc_SET(1221618890L, PH) ;
        p24.h_acc_SET(2239304292L, PH) ;
        p24.eph_SET((char)33886) ;
        p24.vel_SET((char)37747) ;
        p24.cog_SET((char)28307) ;
        p24.lon_SET(972055936) ;
        p24.vel_acc_SET(2356101151L, PH) ;
        p24.alt_ellipsoid_SET(-461426806, PH) ;
        p24.alt_SET(-882120408) ;
        p24.lat_SET(-1250172536) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP) ;
        p24.time_usec_SET(8705021767288521963L) ;
        p24.hdg_acc_SET(2976063437L, PH) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)107, (char)121, (char)44, (char)62, (char)195, (char)225, (char)245, (char)173, (char)38, (char)15, (char)226, (char)80, (char)137, (char)160, (char)65, (char)232, (char)104, (char)61, (char)149, (char)109}));
            assert(pack.satellites_visible_GET() == (char)13);
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)54, (char)206, (char)160, (char)76, (char)107, (char)188, (char)129, (char)49, (char)247, (char)96, (char)19, (char)84, (char)202, (char)254, (char)228, (char)91, (char)76, (char)112, (char)117, (char)181}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)58, (char)240, (char)15, (char)30, (char)113, (char)150, (char)123, (char)192, (char)248, (char)85, (char)154, (char)81, (char)128, (char)61, (char)11, (char)54, (char)227, (char)19, (char)86, (char)107}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)58, (char)177, (char)29, (char)231, (char)181, (char)181, (char)119, (char)11, (char)141, (char)152, (char)35, (char)137, (char)230, (char)148, (char)218, (char)120, (char)197, (char)249, (char)189, (char)195}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)234, (char)73, (char)59, (char)52, (char)92, (char)149, (char)104, (char)105, (char)18, (char)78, (char)101, (char)211, (char)53, (char)239, (char)148, (char)181, (char)98, (char)121, (char)106, (char)131}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_elevation_SET(new char[] {(char)58, (char)177, (char)29, (char)231, (char)181, (char)181, (char)119, (char)11, (char)141, (char)152, (char)35, (char)137, (char)230, (char)148, (char)218, (char)120, (char)197, (char)249, (char)189, (char)195}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)234, (char)73, (char)59, (char)52, (char)92, (char)149, (char)104, (char)105, (char)18, (char)78, (char)101, (char)211, (char)53, (char)239, (char)148, (char)181, (char)98, (char)121, (char)106, (char)131}, 0) ;
        p25.satellites_visible_SET((char)13) ;
        p25.satellite_prn_SET(new char[] {(char)107, (char)121, (char)44, (char)62, (char)195, (char)225, (char)245, (char)173, (char)38, (char)15, (char)226, (char)80, (char)137, (char)160, (char)65, (char)232, (char)104, (char)61, (char)149, (char)109}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)54, (char)206, (char)160, (char)76, (char)107, (char)188, (char)129, (char)49, (char)247, (char)96, (char)19, (char)84, (char)202, (char)254, (char)228, (char)91, (char)76, (char)112, (char)117, (char)181}, 0) ;
        p25.satellite_used_SET(new char[] {(char)58, (char)240, (char)15, (char)30, (char)113, (char)150, (char)123, (char)192, (char)248, (char)85, (char)154, (char)81, (char)128, (char)61, (char)11, (char)54, (char)227, (char)19, (char)86, (char)107}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short)28889);
            assert(pack.time_boot_ms_GET() == 4144027993L);
            assert(pack.yacc_GET() == (short) -5831);
            assert(pack.xmag_GET() == (short)31727);
            assert(pack.ymag_GET() == (short)5589);
            assert(pack.ygyro_GET() == (short)17765);
            assert(pack.zmag_GET() == (short) -9122);
            assert(pack.zgyro_GET() == (short) -3200);
            assert(pack.xgyro_GET() == (short) -19317);
            assert(pack.xacc_GET() == (short)24575);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.zmag_SET((short) -9122) ;
        p26.xmag_SET((short)31727) ;
        p26.ygyro_SET((short)17765) ;
        p26.zacc_SET((short)28889) ;
        p26.time_boot_ms_SET(4144027993L) ;
        p26.xacc_SET((short)24575) ;
        p26.ymag_SET((short)5589) ;
        p26.xgyro_SET((short) -19317) ;
        p26.zgyro_SET((short) -3200) ;
        p26.yacc_SET((short) -5831) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == (short) -326);
            assert(pack.xacc_GET() == (short) -25370);
            assert(pack.time_usec_GET() == 4409350776005616032L);
            assert(pack.zgyro_GET() == (short)9417);
            assert(pack.ymag_GET() == (short) -567);
            assert(pack.zmag_GET() == (short) -13866);
            assert(pack.zacc_GET() == (short)22750);
            assert(pack.yacc_GET() == (short) -25285);
            assert(pack.ygyro_GET() == (short)28685);
            assert(pack.xgyro_GET() == (short) -21183);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.ygyro_SET((short)28685) ;
        p27.xgyro_SET((short) -21183) ;
        p27.xacc_SET((short) -25370) ;
        p27.ymag_SET((short) -567) ;
        p27.yacc_SET((short) -25285) ;
        p27.time_usec_SET(4409350776005616032L) ;
        p27.zgyro_SET((short)9417) ;
        p27.zacc_SET((short)22750) ;
        p27.zmag_SET((short) -13866) ;
        p27.xmag_SET((short) -326) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)2107);
            assert(pack.press_diff1_GET() == (short) -15496);
            assert(pack.press_abs_GET() == (short) -31235);
            assert(pack.time_usec_GET() == 8364517903119312957L);
            assert(pack.press_diff2_GET() == (short)11873);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.time_usec_SET(8364517903119312957L) ;
        p28.press_diff1_SET((short) -15496) ;
        p28.temperature_SET((short)2107) ;
        p28.press_abs_SET((short) -31235) ;
        p28.press_diff2_SET((short)11873) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -9.892402E35F);
            assert(pack.temperature_GET() == (short) -25342);
            assert(pack.time_boot_ms_GET() == 2482852069L);
            assert(pack.press_diff_GET() == 9.020716E37F);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.temperature_SET((short) -25342) ;
        p29.time_boot_ms_SET(2482852069L) ;
        p29.press_abs_SET(-9.892402E35F) ;
        p29.press_diff_SET(9.020716E37F) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 733479614L);
            assert(pack.rollspeed_GET() == -5.589794E37F);
            assert(pack.yawspeed_GET() == -6.2939E37F);
            assert(pack.pitchspeed_GET() == -2.831981E37F);
            assert(pack.yaw_GET() == 2.5483874E37F);
            assert(pack.roll_GET() == 3.0817908E36F);
            assert(pack.pitch_GET() == -2.41956E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.time_boot_ms_SET(733479614L) ;
        p30.rollspeed_SET(-5.589794E37F) ;
        p30.pitch_SET(-2.41956E38F) ;
        p30.yawspeed_SET(-6.2939E37F) ;
        p30.pitchspeed_SET(-2.831981E37F) ;
        p30.roll_SET(3.0817908E36F) ;
        p30.yaw_SET(2.5483874E37F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q2_GET() == -2.3723795E38F);
            assert(pack.yawspeed_GET() == 2.0416874E38F);
            assert(pack.q1_GET() == 1.8730655E38F);
            assert(pack.pitchspeed_GET() == -8.3525255E37F);
            assert(pack.q4_GET() == 1.4984477E38F);
            assert(pack.time_boot_ms_GET() == 4268924259L);
            assert(pack.rollspeed_GET() == 1.5395675E38F);
            assert(pack.q3_GET() == -1.5381697E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.time_boot_ms_SET(4268924259L) ;
        p31.rollspeed_SET(1.5395675E38F) ;
        p31.q1_SET(1.8730655E38F) ;
        p31.q4_SET(1.4984477E38F) ;
        p31.pitchspeed_SET(-8.3525255E37F) ;
        p31.q3_SET(-1.5381697E38F) ;
        p31.yawspeed_SET(2.0416874E38F) ;
        p31.q2_SET(-2.3723795E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == -1.741095E38F);
            assert(pack.vx_GET() == -2.1952323E38F);
            assert(pack.x_GET() == 3.1366817E37F);
            assert(pack.z_GET() == 2.5264694E38F);
            assert(pack.time_boot_ms_GET() == 4140872419L);
            assert(pack.vy_GET() == 9.023381E37F);
            assert(pack.y_GET() == -1.967842E38F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.vz_SET(-1.741095E38F) ;
        p32.x_SET(3.1366817E37F) ;
        p32.z_SET(2.5264694E38F) ;
        p32.vy_SET(9.023381E37F) ;
        p32.y_SET(-1.967842E38F) ;
        p32.time_boot_ms_SET(4140872419L) ;
        p32.vx_SET(-2.1952323E38F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == (short)294);
            assert(pack.lon_GET() == -224268980);
            assert(pack.alt_GET() == 1485848718);
            assert(pack.vx_GET() == (short)16030);
            assert(pack.relative_alt_GET() == -1942930858);
            assert(pack.time_boot_ms_GET() == 4092021253L);
            assert(pack.hdg_GET() == (char)897);
            assert(pack.vz_GET() == (short)19423);
            assert(pack.lat_GET() == 888502765);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.vy_SET((short)294) ;
        p33.hdg_SET((char)897) ;
        p33.lat_SET(888502765) ;
        p33.vx_SET((short)16030) ;
        p33.relative_alt_SET(-1942930858) ;
        p33.time_boot_ms_SET(4092021253L) ;
        p33.lon_SET(-224268980) ;
        p33.alt_SET(1485848718) ;
        p33.vz_SET((short)19423) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1305601334L);
            assert(pack.chan5_scaled_GET() == (short) -18607);
            assert(pack.chan1_scaled_GET() == (short) -15119);
            assert(pack.chan3_scaled_GET() == (short) -25576);
            assert(pack.chan8_scaled_GET() == (short) -22053);
            assert(pack.rssi_GET() == (char)39);
            assert(pack.chan6_scaled_GET() == (short)2883);
            assert(pack.chan4_scaled_GET() == (short) -28741);
            assert(pack.chan2_scaled_GET() == (short) -3288);
            assert(pack.port_GET() == (char)35);
            assert(pack.chan7_scaled_GET() == (short) -21112);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan2_scaled_SET((short) -3288) ;
        p34.chan1_scaled_SET((short) -15119) ;
        p34.chan7_scaled_SET((short) -21112) ;
        p34.port_SET((char)35) ;
        p34.time_boot_ms_SET(1305601334L) ;
        p34.chan5_scaled_SET((short) -18607) ;
        p34.chan3_scaled_SET((short) -25576) ;
        p34.chan4_scaled_SET((short) -28741) ;
        p34.chan8_scaled_SET((short) -22053) ;
        p34.chan6_scaled_SET((short)2883) ;
        p34.rssi_SET((char)39) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan8_raw_GET() == (char)35844);
            assert(pack.chan5_raw_GET() == (char)25797);
            assert(pack.chan7_raw_GET() == (char)3383);
            assert(pack.chan1_raw_GET() == (char)2400);
            assert(pack.port_GET() == (char)122);
            assert(pack.rssi_GET() == (char)71);
            assert(pack.chan3_raw_GET() == (char)49165);
            assert(pack.chan4_raw_GET() == (char)47538);
            assert(pack.time_boot_ms_GET() == 2186385265L);
            assert(pack.chan6_raw_GET() == (char)6128);
            assert(pack.chan2_raw_GET() == (char)21290);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan6_raw_SET((char)6128) ;
        p35.chan1_raw_SET((char)2400) ;
        p35.chan4_raw_SET((char)47538) ;
        p35.chan7_raw_SET((char)3383) ;
        p35.rssi_SET((char)71) ;
        p35.port_SET((char)122) ;
        p35.chan5_raw_SET((char)25797) ;
        p35.chan8_raw_SET((char)35844) ;
        p35.chan3_raw_SET((char)49165) ;
        p35.time_boot_ms_SET(2186385265L) ;
        p35.chan2_raw_SET((char)21290) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo4_raw_GET() == (char)40041);
            assert(pack.servo2_raw_GET() == (char)6673);
            assert(pack.servo14_raw_TRY(ph) == (char)813);
            assert(pack.servo6_raw_GET() == (char)29843);
            assert(pack.time_usec_GET() == 513702991L);
            assert(pack.servo11_raw_TRY(ph) == (char)10782);
            assert(pack.servo5_raw_GET() == (char)43253);
            assert(pack.servo16_raw_TRY(ph) == (char)36541);
            assert(pack.servo3_raw_GET() == (char)51430);
            assert(pack.servo15_raw_TRY(ph) == (char)30696);
            assert(pack.servo13_raw_TRY(ph) == (char)26992);
            assert(pack.servo1_raw_GET() == (char)5399);
            assert(pack.port_GET() == (char)6);
            assert(pack.servo10_raw_TRY(ph) == (char)58825);
            assert(pack.servo9_raw_TRY(ph) == (char)44079);
            assert(pack.servo8_raw_GET() == (char)22052);
            assert(pack.servo7_raw_GET() == (char)40440);
            assert(pack.servo12_raw_TRY(ph) == (char)8709);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo4_raw_SET((char)40041) ;
        p36.servo3_raw_SET((char)51430) ;
        p36.time_usec_SET(513702991L) ;
        p36.servo12_raw_SET((char)8709, PH) ;
        p36.servo11_raw_SET((char)10782, PH) ;
        p36.servo15_raw_SET((char)30696, PH) ;
        p36.port_SET((char)6) ;
        p36.servo10_raw_SET((char)58825, PH) ;
        p36.servo9_raw_SET((char)44079, PH) ;
        p36.servo14_raw_SET((char)813, PH) ;
        p36.servo2_raw_SET((char)6673) ;
        p36.servo8_raw_SET((char)22052) ;
        p36.servo1_raw_SET((char)5399) ;
        p36.servo16_raw_SET((char)36541, PH) ;
        p36.servo6_raw_SET((char)29843) ;
        p36.servo5_raw_SET((char)43253) ;
        p36.servo13_raw_SET((char)26992, PH) ;
        p36.servo7_raw_SET((char)40440) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short) -24995);
            assert(pack.end_index_GET() == (short)21021);
            assert(pack.target_component_GET() == (char)201);
            assert(pack.target_system_GET() == (char)102);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.start_index_SET((short) -24995) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p37.target_component_SET((char)201) ;
        p37.target_system_SET((char)102) ;
        p37.end_index_SET((short)21021) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)40);
            assert(pack.start_index_GET() == (short) -17452);
            assert(pack.target_system_GET() == (char)6);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.end_index_GET() == (short)2422);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p38.target_system_SET((char)6) ;
        p38.start_index_SET((short) -17452) ;
        p38.target_component_SET((char)40) ;
        p38.end_index_SET((short)2422) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 2.0494724E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.seq_GET() == (char)62678);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_USER_3);
            assert(pack.current_GET() == (char)52);
            assert(pack.x_GET() == 4.91088E37F);
            assert(pack.param2_GET() == -1.7123868E38F);
            assert(pack.z_GET() == -1.595102E38F);
            assert(pack.target_system_GET() == (char)137);
            assert(pack.param3_GET() == -4.8281363E37F);
            assert(pack.param1_GET() == -1.0421783E38F);
            assert(pack.param4_GET() == 1.1296256E38F);
            assert(pack.target_component_GET() == (char)129);
            assert(pack.autocontinue_GET() == (char)207);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.current_SET((char)52) ;
        p39.param4_SET(1.1296256E38F) ;
        p39.target_system_SET((char)137) ;
        p39.param2_SET(-1.7123868E38F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p39.param3_SET(-4.8281363E37F) ;
        p39.seq_SET((char)62678) ;
        p39.z_SET(-1.595102E38F) ;
        p39.y_SET(2.0494724E38F) ;
        p39.target_component_SET((char)129) ;
        p39.command_SET(MAV_CMD.MAV_CMD_USER_3) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p39.autocontinue_SET((char)207) ;
        p39.x_SET(4.91088E37F) ;
        p39.param1_SET(-1.0421783E38F) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)111);
            assert(pack.seq_GET() == (char)50346);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)142);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p40.target_system_SET((char)142) ;
        p40.target_component_SET((char)111) ;
        p40.seq_SET((char)50346) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)225);
            assert(pack.target_system_GET() == (char)18);
            assert(pack.seq_GET() == (char)1100);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.seq_SET((char)1100) ;
        p41.target_system_SET((char)18) ;
        p41.target_component_SET((char)225) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)47454);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)47454) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)184);
            assert(pack.target_system_GET() == (char)102);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)102) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p43.target_component_SET((char)184) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)124);
            assert(pack.count_GET() == (char)20823);
            assert(pack.target_system_GET() == (char)74);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_component_SET((char)124) ;
        p44.count_SET((char)20823) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p44.target_system_SET((char)74) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)163);
            assert(pack.target_system_GET() == (char)16);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p45.target_system_SET((char)16) ;
        p45.target_component_SET((char)163) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)30798);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)30798) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)163);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)120);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM6_Y);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM6_Y) ;
        p47.target_component_SET((char)120) ;
        p47.target_system_SET((char)163) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 1239491203);
            assert(pack.longitude_GET() == 1814903656);
            assert(pack.target_system_GET() == (char)57);
            assert(pack.time_usec_TRY(ph) == 2468766297454303899L);
            assert(pack.latitude_GET() == 1775718103);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.target_system_SET((char)57) ;
        p48.longitude_SET(1814903656) ;
        p48.time_usec_SET(2468766297454303899L, PH) ;
        p48.latitude_SET(1775718103) ;
        p48.altitude_SET(1239491203) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 1807504430);
            assert(pack.altitude_GET() == -1652773970);
            assert(pack.time_usec_TRY(ph) == 8624180904971592603L);
            assert(pack.longitude_GET() == -1964602641);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.altitude_SET(-1652773970) ;
        p49.time_usec_SET(8624180904971592603L, PH) ;
        p49.latitude_SET(1807504430) ;
        p49.longitude_SET(-1964602641) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_value_min_GET() == 1.0615857E38F);
            assert(pack.scale_GET() == -1.9626943E38F);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("N"));
            assert(pack.parameter_rc_channel_index_GET() == (char)157);
            assert(pack.target_component_GET() == (char)149);
            assert(pack.param_index_GET() == (short) -14174);
            assert(pack.target_system_GET() == (char)181);
            assert(pack.param_value_max_GET() == -2.5962397E38F);
            assert(pack.param_value0_GET() == -1.6893279E37F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.parameter_rc_channel_index_SET((char)157) ;
        p50.scale_SET(-1.9626943E38F) ;
        p50.param_value0_SET(-1.6893279E37F) ;
        p50.param_index_SET((short) -14174) ;
        p50.target_system_SET((char)181) ;
        p50.param_value_max_SET(-2.5962397E38F) ;
        p50.target_component_SET((char)149) ;
        p50.param_id_SET("N", PH) ;
        p50.param_value_min_SET(1.0615857E38F) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)203);
            assert(pack.target_system_GET() == (char)26);
            assert(pack.seq_GET() == (char)11376);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p51.target_component_SET((char)203) ;
        p51.seq_SET((char)11376) ;
        p51.target_system_SET((char)26) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1z_GET() == -1.6413832E38F);
            assert(pack.p2x_GET() == 1.1599266E38F);
            assert(pack.p1y_GET() == 2.8612452E38F);
            assert(pack.target_system_GET() == (char)149);
            assert(pack.p2y_GET() == -9.927143E37F);
            assert(pack.p2z_GET() == 1.6206799E38F);
            assert(pack.p1x_GET() == -1.834051E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.target_component_GET() == (char)113);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p1x_SET(-1.834051E38F) ;
        p54.p1y_SET(2.8612452E38F) ;
        p54.p2z_SET(1.6206799E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p54.target_system_SET((char)149) ;
        p54.p1z_SET(-1.6413832E38F) ;
        p54.p2y_SET(-9.927143E37F) ;
        p54.target_component_SET((char)113) ;
        p54.p2x_SET(1.1599266E38F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2z_GET() == 2.8599435E38F);
            assert(pack.p1y_GET() == -2.4897838E38F);
            assert(pack.p2x_GET() == 3.007905E38F);
            assert(pack.p2y_GET() == -2.4766714E38F);
            assert(pack.p1x_GET() == 2.6899807E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.p1z_GET() == 1.1567529E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2x_SET(3.007905E38F) ;
        p55.p2y_SET(-2.4766714E38F) ;
        p55.p1x_SET(2.6899807E38F) ;
        p55.p1z_SET(1.1567529E38F) ;
        p55.p2z_SET(2.8599435E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p55.p1y_SET(-2.4897838E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.pitchspeed_GET() == 1.6901017E38F);
            assert(pack.time_usec_GET() == 4886310394011585477L);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.452499E38F, -2.2521588E38F, 1.6379881E38F, 2.9726211E37F, -1.469508E38F, 1.7431938E38F, -2.2196485E38F, 1.64285E38F, 1.1931649E38F}));
            assert(pack.yawspeed_GET() == 2.8169225E38F);
            assert(pack.rollspeed_GET() == 3.993407E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {9.86056E37F, 2.8351186E38F, 1.5741645E38F, -3.322976E38F}));
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.yawspeed_SET(2.8169225E38F) ;
        p61.pitchspeed_SET(1.6901017E38F) ;
        p61.covariance_SET(new float[] {2.452499E38F, -2.2521588E38F, 1.6379881E38F, 2.9726211E37F, -1.469508E38F, 1.7431938E38F, -2.2196485E38F, 1.64285E38F, 1.1931649E38F}, 0) ;
        p61.time_usec_SET(4886310394011585477L) ;
        p61.rollspeed_SET(3.993407E37F) ;
        p61.q_SET(new float[] {9.86056E37F, 2.8351186E38F, 1.5741645E38F, -3.322976E38F}, 0) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_bearing_GET() == (short) -3503);
            assert(pack.xtrack_error_GET() == -3.233667E38F);
            assert(pack.alt_error_GET() == 9.021166E37F);
            assert(pack.nav_pitch_GET() == 2.8215466E38F);
            assert(pack.aspd_error_GET() == -6.771533E37F);
            assert(pack.wp_dist_GET() == (char)1252);
            assert(pack.target_bearing_GET() == (short)5154);
            assert(pack.nav_roll_GET() == 3.0653004E38F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.alt_error_SET(9.021166E37F) ;
        p62.wp_dist_SET((char)1252) ;
        p62.aspd_error_SET(-6.771533E37F) ;
        p62.xtrack_error_SET(-3.233667E38F) ;
        p62.target_bearing_SET((short)5154) ;
        p62.nav_bearing_SET((short) -3503) ;
        p62.nav_pitch_SET(2.8215466E38F) ;
        p62.nav_roll_SET(3.0653004E38F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == -2.1233588E38F);
            assert(pack.relative_alt_GET() == -284627369);
            assert(pack.vx_GET() == 8.3952154E37F);
            assert(pack.time_usec_GET() == 2144489533040591134L);
            assert(pack.lat_GET() == -1883098261);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.lon_GET() == -855090238);
            assert(pack.alt_GET() == 987464203);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {6.514089E36F, 6.0332674E37F, 8.6868134E35F, 4.385415E36F, 1.9737072E38F, 8.979336E37F, 1.1713667E38F, 1.3057266E38F, 2.435104E38F, 2.4771696E38F, 1.3455598E37F, 4.165147E37F, 2.719283E38F, -1.4868462E37F, 7.3439284E37F, 2.3682023E38F, -1.0951454E38F, -3.1737671E38F, 1.6467182E38F, 2.0625971E38F, -2.6944165E38F, -1.9052659E38F, -1.5278391E38F, -8.858167E37F, 1.04031E38F, 3.2236085E38F, -5.3651055E37F, 3.3685565E38F, -4.5417933E37F, 1.3334667E38F, -2.7358979E38F, -1.6943708E38F, 1.4770699E38F, 2.3333752E38F, 3.0284958E38F, 1.204008E36F}));
            assert(pack.vz_GET() == -1.8063658E38F);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.vy_SET(-2.1233588E38F) ;
        p63.vz_SET(-1.8063658E38F) ;
        p63.relative_alt_SET(-284627369) ;
        p63.alt_SET(987464203) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p63.lon_SET(-855090238) ;
        p63.time_usec_SET(2144489533040591134L) ;
        p63.covariance_SET(new float[] {6.514089E36F, 6.0332674E37F, 8.6868134E35F, 4.385415E36F, 1.9737072E38F, 8.979336E37F, 1.1713667E38F, 1.3057266E38F, 2.435104E38F, 2.4771696E38F, 1.3455598E37F, 4.165147E37F, 2.719283E38F, -1.4868462E37F, 7.3439284E37F, 2.3682023E38F, -1.0951454E38F, -3.1737671E38F, 1.6467182E38F, 2.0625971E38F, -2.6944165E38F, -1.9052659E38F, -1.5278391E38F, -8.858167E37F, 1.04031E38F, 3.2236085E38F, -5.3651055E37F, 3.3685565E38F, -4.5417933E37F, 1.3334667E38F, -2.7358979E38F, -1.6943708E38F, 1.4770699E38F, 2.3333752E38F, 3.0284958E38F, 1.204008E36F}, 0) ;
        p63.lat_SET(-1883098261) ;
        p63.vx_SET(8.3952154E37F) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == 5.8741935E37F);
            assert(pack.ax_GET() == -1.7758951E38F);
            assert(pack.z_GET() == -1.4965826E38F);
            assert(pack.vx_GET() == -8.730582E37F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.0139386E38F, 1.2782824E38F, -3.2696516E38F, -2.4561878E38F, -2.2614278E38F, -2.7903912E36F, -1.3088598E37F, 2.0578314E38F, -2.98672E38F, 2.3833896E38F, -3.2102032E38F, -2.0629062E38F, 5.5112844E37F, -2.066224E38F, 1.4040662E38F, -6.7091377E37F, 1.0400625E38F, 2.2780523E38F, 1.9074065E38F, -2.2895966E38F, -2.49026E38F, 1.0996701E38F, 7.728613E37F, -3.1351975E38F, -3.0786236E37F, 2.5243242E38F, 1.337849E38F, -2.1900593E38F, 3.53653E37F, 2.6080074E38F, -7.112864E37F, -2.0498809E38F, -1.0708717E38F, -1.2112375E38F, -4.441424E37F, 3.1528476E38F, 3.2833674E37F, 3.33504E38F, -2.54695E38F, -5.5585886E37F, 1.8878096E38F, -2.5915317E38F, -3.3761626E38F, -2.4059558E38F, -2.1434718E38F}));
            assert(pack.ay_GET() == -3.4018448E38F);
            assert(pack.time_usec_GET() == 1384068308905761940L);
            assert(pack.x_GET() == -2.5047782E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
            assert(pack.vy_GET() == 2.701135E38F);
            assert(pack.y_GET() == -3.1767582E38F);
            assert(pack.az_GET() == -2.7514665E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.vx_SET(-8.730582E37F) ;
        p64.x_SET(-2.5047782E38F) ;
        p64.ax_SET(-1.7758951E38F) ;
        p64.ay_SET(-3.4018448E38F) ;
        p64.z_SET(-1.4965826E38F) ;
        p64.covariance_SET(new float[] {1.0139386E38F, 1.2782824E38F, -3.2696516E38F, -2.4561878E38F, -2.2614278E38F, -2.7903912E36F, -1.3088598E37F, 2.0578314E38F, -2.98672E38F, 2.3833896E38F, -3.2102032E38F, -2.0629062E38F, 5.5112844E37F, -2.066224E38F, 1.4040662E38F, -6.7091377E37F, 1.0400625E38F, 2.2780523E38F, 1.9074065E38F, -2.2895966E38F, -2.49026E38F, 1.0996701E38F, 7.728613E37F, -3.1351975E38F, -3.0786236E37F, 2.5243242E38F, 1.337849E38F, -2.1900593E38F, 3.53653E37F, 2.6080074E38F, -7.112864E37F, -2.0498809E38F, -1.0708717E38F, -1.2112375E38F, -4.441424E37F, 3.1528476E38F, 3.2833674E37F, 3.33504E38F, -2.54695E38F, -5.5585886E37F, 1.8878096E38F, -2.5915317E38F, -3.3761626E38F, -2.4059558E38F, -2.1434718E38F}, 0) ;
        p64.time_usec_SET(1384068308905761940L) ;
        p64.y_SET(-3.1767582E38F) ;
        p64.az_SET(-2.7514665E38F) ;
        p64.vz_SET(5.8741935E37F) ;
        p64.vy_SET(2.701135E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)39677);
            assert(pack.chan12_raw_GET() == (char)47469);
            assert(pack.chan15_raw_GET() == (char)1458);
            assert(pack.chan9_raw_GET() == (char)58528);
            assert(pack.chan7_raw_GET() == (char)7907);
            assert(pack.chan8_raw_GET() == (char)9974);
            assert(pack.chancount_GET() == (char)86);
            assert(pack.chan6_raw_GET() == (char)8927);
            assert(pack.chan13_raw_GET() == (char)46857);
            assert(pack.chan14_raw_GET() == (char)65216);
            assert(pack.chan2_raw_GET() == (char)10622);
            assert(pack.chan10_raw_GET() == (char)43734);
            assert(pack.chan5_raw_GET() == (char)17111);
            assert(pack.chan3_raw_GET() == (char)42858);
            assert(pack.chan18_raw_GET() == (char)14616);
            assert(pack.chan1_raw_GET() == (char)39179);
            assert(pack.chan17_raw_GET() == (char)1891);
            assert(pack.chan16_raw_GET() == (char)11266);
            assert(pack.rssi_GET() == (char)54);
            assert(pack.chan11_raw_GET() == (char)14293);
            assert(pack.time_boot_ms_GET() == 1840444777L);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan1_raw_SET((char)39179) ;
        p65.chan3_raw_SET((char)42858) ;
        p65.chancount_SET((char)86) ;
        p65.chan18_raw_SET((char)14616) ;
        p65.chan2_raw_SET((char)10622) ;
        p65.chan14_raw_SET((char)65216) ;
        p65.rssi_SET((char)54) ;
        p65.chan8_raw_SET((char)9974) ;
        p65.chan5_raw_SET((char)17111) ;
        p65.chan10_raw_SET((char)43734) ;
        p65.chan17_raw_SET((char)1891) ;
        p65.chan15_raw_SET((char)1458) ;
        p65.chan4_raw_SET((char)39677) ;
        p65.chan13_raw_SET((char)46857) ;
        p65.chan9_raw_SET((char)58528) ;
        p65.chan12_raw_SET((char)47469) ;
        p65.chan16_raw_SET((char)11266) ;
        p65.time_boot_ms_SET(1840444777L) ;
        p65.chan7_raw_SET((char)7907) ;
        p65.chan11_raw_SET((char)14293) ;
        p65.chan6_raw_SET((char)8927) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.req_message_rate_GET() == (char)62320);
            assert(pack.req_stream_id_GET() == (char)115);
            assert(pack.target_system_GET() == (char)46);
            assert(pack.start_stop_GET() == (char)137);
            assert(pack.target_component_GET() == (char)15);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_component_SET((char)15) ;
        p66.target_system_SET((char)46) ;
        p66.req_stream_id_SET((char)115) ;
        p66.start_stop_SET((char)137) ;
        p66.req_message_rate_SET((char)62320) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.message_rate_GET() == (char)42334);
            assert(pack.stream_id_GET() == (char)124);
            assert(pack.on_off_GET() == (char)141);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)42334) ;
        p67.stream_id_SET((char)124) ;
        p67.on_off_SET((char)141) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == (short)1753);
            assert(pack.buttons_GET() == (char)14833);
            assert(pack.target_GET() == (char)14);
            assert(pack.r_GET() == (short)9359);
            assert(pack.x_GET() == (short) -28263);
            assert(pack.z_GET() == (short) -4659);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.buttons_SET((char)14833) ;
        p69.target_SET((char)14) ;
        p69.r_SET((short)9359) ;
        p69.y_SET((short)1753) ;
        p69.z_SET((short) -4659) ;
        p69.x_SET((short) -28263) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)64);
            assert(pack.target_component_GET() == (char)232);
            assert(pack.chan2_raw_GET() == (char)63522);
            assert(pack.chan1_raw_GET() == (char)14629);
            assert(pack.chan5_raw_GET() == (char)47330);
            assert(pack.chan4_raw_GET() == (char)55422);
            assert(pack.chan8_raw_GET() == (char)25762);
            assert(pack.chan3_raw_GET() == (char)64105);
            assert(pack.chan7_raw_GET() == (char)51752);
            assert(pack.chan6_raw_GET() == (char)18573);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.target_system_SET((char)64) ;
        p70.chan6_raw_SET((char)18573) ;
        p70.chan7_raw_SET((char)51752) ;
        p70.chan5_raw_SET((char)47330) ;
        p70.chan8_raw_SET((char)25762) ;
        p70.target_component_SET((char)232) ;
        p70.chan4_raw_SET((char)55422) ;
        p70.chan3_raw_SET((char)64105) ;
        p70.chan2_raw_SET((char)63522) ;
        p70.chan1_raw_SET((char)14629) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param4_GET() == -1.9382303E38F);
            assert(pack.target_system_GET() == (char)114);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN);
            assert(pack.param3_GET() == -1.1365066E38F);
            assert(pack.param1_GET() == 2.8504558E38F);
            assert(pack.seq_GET() == (char)27650);
            assert(pack.y_GET() == 648966653);
            assert(pack.autocontinue_GET() == (char)111);
            assert(pack.param2_GET() == -5.5730073E37F);
            assert(pack.z_GET() == -3.0514115E38F);
            assert(pack.current_GET() == (char)120);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)128);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.x_GET() == -415809600);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p73.current_SET((char)120) ;
        p73.x_SET(-415809600) ;
        p73.param4_SET(-1.9382303E38F) ;
        p73.param3_SET(-1.1365066E38F) ;
        p73.param1_SET(2.8504558E38F) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p73.y_SET(648966653) ;
        p73.seq_SET((char)27650) ;
        p73.command_SET(MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN) ;
        p73.z_SET(-3.0514115E38F) ;
        p73.param2_SET(-5.5730073E37F) ;
        p73.target_system_SET((char)114) ;
        p73.target_component_SET((char)128) ;
        p73.autocontinue_SET((char)111) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 2.4603632E38F);
            assert(pack.groundspeed_GET() == 1.6900239E38F);
            assert(pack.heading_GET() == (short)13985);
            assert(pack.airspeed_GET() == -6.5915296E37F);
            assert(pack.climb_GET() == 1.5255558E38F);
            assert(pack.throttle_GET() == (char)27983);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.alt_SET(2.4603632E38F) ;
        p74.airspeed_SET(-6.5915296E37F) ;
        p74.groundspeed_SET(1.6900239E38F) ;
        p74.throttle_SET((char)27983) ;
        p74.heading_SET((short)13985) ;
        p74.climb_SET(1.5255558E38F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.param1_GET() == 1.2785425E38F);
            assert(pack.y_GET() == -2000154932);
            assert(pack.x_GET() == -1960107442);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.param3_GET() == 1.1696318E38F);
            assert(pack.target_component_GET() == (char)221);
            assert(pack.param2_GET() == 3.0123124E38F);
            assert(pack.z_GET() == 1.1229177E38F);
            assert(pack.current_GET() == (char)31);
            assert(pack.target_system_GET() == (char)249);
            assert(pack.autocontinue_GET() == (char)178);
            assert(pack.param4_GET() == 3.2017595E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_STORAGE_FORMAT);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.param3_SET(1.1696318E38F) ;
        p75.param4_SET(3.2017595E38F) ;
        p75.command_SET(MAV_CMD.MAV_CMD_STORAGE_FORMAT) ;
        p75.x_SET(-1960107442) ;
        p75.z_SET(1.1229177E38F) ;
        p75.param1_SET(1.2785425E38F) ;
        p75.target_system_SET((char)249) ;
        p75.autocontinue_SET((char)178) ;
        p75.param2_SET(3.0123124E38F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p75.target_component_SET((char)221) ;
        p75.y_SET(-2000154932) ;
        p75.current_SET((char)31) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_LOGGING_START);
            assert(pack.target_component_GET() == (char)145);
            assert(pack.param6_GET() == -1.8968272E38F);
            assert(pack.param4_GET() == 1.2059118E38F);
            assert(pack.confirmation_GET() == (char)149);
            assert(pack.param3_GET() == 8.710061E37F);
            assert(pack.param7_GET() == 2.186824E38F);
            assert(pack.param2_GET() == 1.6499078E38F);
            assert(pack.param5_GET() == 1.4842693E38F);
            assert(pack.target_system_GET() == (char)124);
            assert(pack.param1_GET() == 2.0711873E38F);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.param5_SET(1.4842693E38F) ;
        p76.param1_SET(2.0711873E38F) ;
        p76.param7_SET(2.186824E38F) ;
        p76.target_component_SET((char)145) ;
        p76.target_system_SET((char)124) ;
        p76.param6_SET(-1.8968272E38F) ;
        p76.param3_SET(8.710061E37F) ;
        p76.param4_SET(1.2059118E38F) ;
        p76.param2_SET(1.6499078E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_LOGGING_START) ;
        p76.confirmation_SET((char)149) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.result_param2_TRY(ph) == 740637709);
            assert(pack.target_system_TRY(ph) == (char)217);
            assert(pack.progress_TRY(ph) == (char)143);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_DENIED);
            assert(pack.target_component_TRY(ph) == (char)67);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_USER_1);
        });
        GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.result_SET(MAV_RESULT.MAV_RESULT_DENIED) ;
        p77.target_component_SET((char)67, PH) ;
        p77.progress_SET((char)143, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_USER_1) ;
        p77.result_param2_SET(740637709, PH) ;
        p77.target_system_SET((char)217, PH) ;
        CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 3.3061003E38F);
            assert(pack.manual_override_switch_GET() == (char)92);
            assert(pack.roll_GET() == 1.5771772E38F);
            assert(pack.yaw_GET() == 2.2148304E38F);
            assert(pack.time_boot_ms_GET() == 214405301L);
            assert(pack.thrust_GET() == -2.2878954E38F);
            assert(pack.mode_switch_GET() == (char)105);
        });
        GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.roll_SET(1.5771772E38F) ;
        p81.yaw_SET(2.2148304E38F) ;
        p81.pitch_SET(3.3061003E38F) ;
        p81.manual_override_switch_SET((char)92) ;
        p81.thrust_SET(-2.2878954E38F) ;
        p81.time_boot_ms_SET(214405301L) ;
        p81.mode_switch_SET((char)105) ;
        CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)202);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.457311E38F, -2.3319287E38F, -2.2386229E38F, -1.2167246E38F}));
            assert(pack.target_component_GET() == (char)222);
            assert(pack.body_pitch_rate_GET() == -2.8396057E38F);
            assert(pack.thrust_GET() == -3.2009954E38F);
            assert(pack.body_roll_rate_GET() == -1.5608045E38F);
            assert(pack.time_boot_ms_GET() == 137816683L);
            assert(pack.body_yaw_rate_GET() == -2.5365871E38F);
            assert(pack.type_mask_GET() == (char)175);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.body_roll_rate_SET(-1.5608045E38F) ;
        p82.type_mask_SET((char)175) ;
        p82.target_component_SET((char)222) ;
        p82.q_SET(new float[] {1.457311E38F, -2.3319287E38F, -2.2386229E38F, -1.2167246E38F}, 0) ;
        p82.body_pitch_rate_SET(-2.8396057E38F) ;
        p82.target_system_SET((char)202) ;
        p82.time_boot_ms_SET(137816683L) ;
        p82.body_yaw_rate_SET(-2.5365871E38F) ;
        p82.thrust_SET(-3.2009954E38F) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_pitch_rate_GET() == 2.8183572E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-5.03633E37F, -1.8978965E38F, 2.0454784E38F, 7.2924587E37F}));
            assert(pack.time_boot_ms_GET() == 3893002817L);
            assert(pack.thrust_GET() == 2.807681E38F);
            assert(pack.type_mask_GET() == (char)199);
            assert(pack.body_roll_rate_GET() == -7.930968E37F);
            assert(pack.body_yaw_rate_GET() == 1.8376027E38F);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_yaw_rate_SET(1.8376027E38F) ;
        p83.body_roll_rate_SET(-7.930968E37F) ;
        p83.type_mask_SET((char)199) ;
        p83.body_pitch_rate_SET(2.8183572E38F) ;
        p83.q_SET(new float[] {-5.03633E37F, -1.8978965E38F, 2.0454784E38F, 7.2924587E37F}, 0) ;
        p83.thrust_SET(2.807681E38F) ;
        p83.time_boot_ms_SET(3893002817L) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == -3.4943324E37F);
            assert(pack.x_GET() == -2.0351796E38F);
            assert(pack.yaw_GET() == -5.2412545E37F);
            assert(pack.type_mask_GET() == (char)40284);
            assert(pack.z_GET() == -8.483043E37F);
            assert(pack.vy_GET() == -5.088556E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.afx_GET() == -2.5343082E38F);
            assert(pack.vz_GET() == 2.5616787E38F);
            assert(pack.yaw_rate_GET() == -5.5600727E37F);
            assert(pack.target_system_GET() == (char)246);
            assert(pack.y_GET() == -2.9565103E38F);
            assert(pack.afy_GET() == -2.7329535E38F);
            assert(pack.target_component_GET() == (char)115);
            assert(pack.time_boot_ms_GET() == 920154337L);
            assert(pack.afz_GET() == 1.0654782E38F);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.afy_SET(-2.7329535E38F) ;
        p84.yaw_rate_SET(-5.5600727E37F) ;
        p84.vz_SET(2.5616787E38F) ;
        p84.x_SET(-2.0351796E38F) ;
        p84.vy_SET(-5.088556E37F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p84.type_mask_SET((char)40284) ;
        p84.target_system_SET((char)246) ;
        p84.time_boot_ms_SET(920154337L) ;
        p84.afx_SET(-2.5343082E38F) ;
        p84.vx_SET(-3.4943324E37F) ;
        p84.z_SET(-8.483043E37F) ;
        p84.afz_SET(1.0654782E38F) ;
        p84.y_SET(-2.9565103E38F) ;
        p84.yaw_SET(-5.2412545E37F) ;
        p84.target_component_SET((char)115) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)67);
            assert(pack.target_system_GET() == (char)63);
            assert(pack.time_boot_ms_GET() == 3947173075L);
            assert(pack.type_mask_GET() == (char)33949);
            assert(pack.afy_GET() == 1.0287003E38F);
            assert(pack.yaw_GET() == -2.3793345E38F);
            assert(pack.vz_GET() == -1.7222348E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.afx_GET() == 4.466588E37F);
            assert(pack.afz_GET() == -1.3181797E38F);
            assert(pack.alt_GET() == -2.1738774E38F);
            assert(pack.lat_int_GET() == 773602770);
            assert(pack.vx_GET() == -1.8059155E38F);
            assert(pack.vy_GET() == -7.3505836E37F);
            assert(pack.yaw_rate_GET() == -1.8324267E38F);
            assert(pack.lon_int_GET() == -98688650);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.target_system_SET((char)63) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p86.type_mask_SET((char)33949) ;
        p86.lat_int_SET(773602770) ;
        p86.alt_SET(-2.1738774E38F) ;
        p86.lon_int_SET(-98688650) ;
        p86.yaw_SET(-2.3793345E38F) ;
        p86.time_boot_ms_SET(3947173075L) ;
        p86.target_component_SET((char)67) ;
        p86.yaw_rate_SET(-1.8324267E38F) ;
        p86.vy_SET(-7.3505836E37F) ;
        p86.afz_SET(-1.3181797E38F) ;
        p86.vx_SET(-1.8059155E38F) ;
        p86.afy_SET(1.0287003E38F) ;
        p86.afx_SET(4.466588E37F) ;
        p86.vz_SET(-1.7222348E38F) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -1.5816531E38F);
            assert(pack.type_mask_GET() == (char)6643);
            assert(pack.afy_GET() == 3.7341731E37F);
            assert(pack.lon_int_GET() == 201797005);
            assert(pack.afx_GET() == -1.0176481E38F);
            assert(pack.lat_int_GET() == -397364455);
            assert(pack.alt_GET() == 1.1089806E38F);
            assert(pack.time_boot_ms_GET() == 2207237062L);
            assert(pack.vx_GET() == 2.3160074E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.vz_GET() == 1.8517225E38F);
            assert(pack.vy_GET() == 2.164702E38F);
            assert(pack.yaw_rate_GET() == -2.5284715E38F);
            assert(pack.afz_GET() == 2.8866317E38F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.afx_SET(-1.0176481E38F) ;
        p87.afy_SET(3.7341731E37F) ;
        p87.lat_int_SET(-397364455) ;
        p87.lon_int_SET(201797005) ;
        p87.time_boot_ms_SET(2207237062L) ;
        p87.type_mask_SET((char)6643) ;
        p87.vy_SET(2.164702E38F) ;
        p87.vx_SET(2.3160074E38F) ;
        p87.afz_SET(2.8866317E38F) ;
        p87.alt_SET(1.1089806E38F) ;
        p87.yaw_rate_SET(-2.5284715E38F) ;
        p87.yaw_SET(-1.5816531E38F) ;
        p87.vz_SET(1.8517225E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -3.1509135E38F);
            assert(pack.time_boot_ms_GET() == 952754191L);
            assert(pack.y_GET() == 1.9298842E37F);
            assert(pack.yaw_GET() == 1.8936276E38F);
            assert(pack.z_GET() == 2.7227547E38F);
            assert(pack.x_GET() == 3.962195E35F);
            assert(pack.roll_GET() == 1.4672097E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(952754191L) ;
        p89.yaw_SET(1.8936276E38F) ;
        p89.y_SET(1.9298842E37F) ;
        p89.pitch_SET(-3.1509135E38F) ;
        p89.x_SET(3.962195E35F) ;
        p89.roll_SET(1.4672097E38F) ;
        p89.z_SET(2.7227547E38F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 668687621);
            assert(pack.time_usec_GET() == 6998046566655488486L);
            assert(pack.zacc_GET() == (short) -2035);
            assert(pack.lat_GET() == 1568693205);
            assert(pack.pitch_GET() == 2.6824122E38F);
            assert(pack.yawspeed_GET() == -2.6867719E38F);
            assert(pack.lon_GET() == -984579603);
            assert(pack.roll_GET() == 1.0670029E38F);
            assert(pack.vx_GET() == (short) -14684);
            assert(pack.pitchspeed_GET() == -3.800833E37F);
            assert(pack.xacc_GET() == (short) -15909);
            assert(pack.yaw_GET() == 9.227578E37F);
            assert(pack.vz_GET() == (short) -25769);
            assert(pack.vy_GET() == (short) -3744);
            assert(pack.yacc_GET() == (short)25542);
            assert(pack.rollspeed_GET() == -1.0381341E38F);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.lat_SET(1568693205) ;
        p90.vy_SET((short) -3744) ;
        p90.roll_SET(1.0670029E38F) ;
        p90.zacc_SET((short) -2035) ;
        p90.pitchspeed_SET(-3.800833E37F) ;
        p90.vx_SET((short) -14684) ;
        p90.yacc_SET((short)25542) ;
        p90.lon_SET(-984579603) ;
        p90.xacc_SET((short) -15909) ;
        p90.pitch_SET(2.6824122E38F) ;
        p90.alt_SET(668687621) ;
        p90.vz_SET((short) -25769) ;
        p90.yawspeed_SET(-2.6867719E38F) ;
        p90.rollspeed_SET(-1.0381341E38F) ;
        p90.yaw_SET(9.227578E37F) ;
        p90.time_usec_SET(6998046566655488486L) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.nav_mode_GET() == (char)57);
            assert(pack.aux1_GET() == 1.5384501E38F);
            assert(pack.throttle_GET() == 4.3557137E36F);
            assert(pack.yaw_rudder_GET() == -2.4116642E37F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_ARMED);
            assert(pack.roll_ailerons_GET() == 1.4392582E38F);
            assert(pack.aux2_GET() == 2.6882194E37F);
            assert(pack.aux3_GET() == 1.949382E38F);
            assert(pack.time_usec_GET() == 5420524043595349565L);
            assert(pack.pitch_elevator_GET() == -2.0952188E38F);
            assert(pack.aux4_GET() == -1.2876471E38F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.throttle_SET(4.3557137E36F) ;
        p91.yaw_rudder_SET(-2.4116642E37F) ;
        p91.nav_mode_SET((char)57) ;
        p91.aux1_SET(1.5384501E38F) ;
        p91.roll_ailerons_SET(1.4392582E38F) ;
        p91.aux3_SET(1.949382E38F) ;
        p91.time_usec_SET(5420524043595349565L) ;
        p91.pitch_elevator_SET(-2.0952188E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED) ;
        p91.aux4_SET(-1.2876471E38F) ;
        p91.aux2_SET(2.6882194E37F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan10_raw_GET() == (char)53522);
            assert(pack.chan5_raw_GET() == (char)35519);
            assert(pack.chan4_raw_GET() == (char)45496);
            assert(pack.time_usec_GET() == 5437618552400449166L);
            assert(pack.chan1_raw_GET() == (char)20690);
            assert(pack.chan7_raw_GET() == (char)4846);
            assert(pack.chan3_raw_GET() == (char)59409);
            assert(pack.chan12_raw_GET() == (char)652);
            assert(pack.chan6_raw_GET() == (char)2700);
            assert(pack.chan9_raw_GET() == (char)56525);
            assert(pack.chan8_raw_GET() == (char)38832);
            assert(pack.chan11_raw_GET() == (char)51208);
            assert(pack.rssi_GET() == (char)208);
            assert(pack.chan2_raw_GET() == (char)22811);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan4_raw_SET((char)45496) ;
        p92.chan3_raw_SET((char)59409) ;
        p92.chan1_raw_SET((char)20690) ;
        p92.time_usec_SET(5437618552400449166L) ;
        p92.chan12_raw_SET((char)652) ;
        p92.chan2_raw_SET((char)22811) ;
        p92.chan5_raw_SET((char)35519) ;
        p92.rssi_SET((char)208) ;
        p92.chan9_raw_SET((char)56525) ;
        p92.chan7_raw_SET((char)4846) ;
        p92.chan10_raw_SET((char)53522) ;
        p92.chan6_raw_SET((char)2700) ;
        p92.chan11_raw_SET((char)51208) ;
        p92.chan8_raw_SET((char)38832) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_AUTO_DISARMED);
            assert(pack.flags_GET() == 4472056870195623638L);
            assert(pack.time_usec_GET() == 7751646491879367634L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.0211013E38F, -1.9988004E38F, 9.125724E37F, -1.7465952E38F, 4.5475794E37F, 2.619494E37F, -9.749886E37F, 1.7394779E38F, 3.243302E38F, 1.28556E38F, 1.4487921E38F, 9.826209E37F, 1.3690763E38F, -2.2517057E38F, -3.1147348E38F, -8.3712487E37F}));
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(7751646491879367634L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED) ;
        p93.flags_SET(4472056870195623638L) ;
        p93.controls_SET(new float[] {1.0211013E38F, -1.9988004E38F, 9.125724E37F, -1.7465952E38F, 4.5475794E37F, 2.619494E37F, -9.749886E37F, 1.7394779E38F, 3.243302E38F, 1.28556E38F, 1.4487921E38F, 9.826209E37F, 1.3690763E38F, -2.2517057E38F, -3.1147348E38F, -8.3712487E37F}, 0) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_x_GET() == (short) -20482);
            assert(pack.flow_comp_m_y_GET() == 1.931466E38F);
            assert(pack.sensor_id_GET() == (char)249);
            assert(pack.flow_y_GET() == (short)28951);
            assert(pack.quality_GET() == (char)165);
            assert(pack.flow_rate_x_TRY(ph) == -2.9366169E38F);
            assert(pack.flow_comp_m_x_GET() == 1.6326377E38F);
            assert(pack.flow_rate_y_TRY(ph) == 3.2581038E38F);
            assert(pack.ground_distance_GET() == -1.9621524E38F);
            assert(pack.time_usec_GET() == 3492688192739416528L);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.ground_distance_SET(-1.9621524E38F) ;
        p100.flow_x_SET((short) -20482) ;
        p100.flow_rate_x_SET(-2.9366169E38F, PH) ;
        p100.time_usec_SET(3492688192739416528L) ;
        p100.quality_SET((char)165) ;
        p100.flow_rate_y_SET(3.2581038E38F, PH) ;
        p100.flow_y_SET((short)28951) ;
        p100.flow_comp_m_x_SET(1.6326377E38F) ;
        p100.sensor_id_SET((char)249) ;
        p100.flow_comp_m_y_SET(1.931466E38F) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 1.984408E38F);
            assert(pack.yaw_GET() == -2.589958E38F);
            assert(pack.roll_GET() == -1.7281514E38F);
            assert(pack.pitch_GET() == 2.3956335E38F);
            assert(pack.y_GET() == 5.876354E37F);
            assert(pack.usec_GET() == 5590292840657911911L);
            assert(pack.x_GET() == -1.7959434E37F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.y_SET(5.876354E37F) ;
        p101.pitch_SET(2.3956335E38F) ;
        p101.roll_SET(-1.7281514E38F) ;
        p101.yaw_SET(-2.589958E38F) ;
        p101.z_SET(1.984408E38F) ;
        p101.usec_SET(5590292840657911911L) ;
        p101.x_SET(-1.7959434E37F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 1.4107457E38F);
            assert(pack.roll_GET() == -3.341394E38F);
            assert(pack.y_GET() == 1.859923E38F);
            assert(pack.usec_GET() == 1068206023859110472L);
            assert(pack.z_GET() == 3.007475E37F);
            assert(pack.yaw_GET() == -2.7443188E37F);
            assert(pack.pitch_GET() == 3.8523154E37F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.yaw_SET(-2.7443188E37F) ;
        p102.y_SET(1.859923E38F) ;
        p102.x_SET(1.4107457E38F) ;
        p102.usec_SET(1068206023859110472L) ;
        p102.pitch_SET(3.8523154E37F) ;
        p102.roll_SET(-3.341394E38F) ;
        p102.z_SET(3.007475E37F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.8071112E38F);
            assert(pack.x_GET() == 2.3872915E38F);
            assert(pack.y_GET() == 2.6334007E38F);
            assert(pack.usec_GET() == 6511271792640935210L);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.y_SET(2.6334007E38F) ;
        p103.usec_SET(6511271792640935210L) ;
        p103.x_SET(2.3872915E38F) ;
        p103.z_SET(-1.8071112E38F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -2.8472481E38F);
            assert(pack.x_GET() == -1.6138025E38F);
            assert(pack.yaw_GET() == -6.409342E37F);
            assert(pack.usec_GET() == 7347809168253845791L);
            assert(pack.y_GET() == 7.4947377E37F);
            assert(pack.roll_GET() == -2.8214838E38F);
            assert(pack.pitch_GET() == 9.101899E37F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.x_SET(-1.6138025E38F) ;
        p104.z_SET(-2.8472481E38F) ;
        p104.yaw_SET(-6.409342E37F) ;
        p104.y_SET(7.4947377E37F) ;
        p104.roll_SET(-2.8214838E38F) ;
        p104.pitch_SET(9.101899E37F) ;
        p104.usec_SET(7347809168253845791L) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == -2.0079945E38F);
            assert(pack.ymag_GET() == -8.0102497E37F);
            assert(pack.temperature_GET() == -2.2325986E38F);
            assert(pack.diff_pressure_GET() == -2.1578005E38F);
            assert(pack.time_usec_GET() == 1868281731735983364L);
            assert(pack.zgyro_GET() == 2.78485E37F);
            assert(pack.pressure_alt_GET() == -1.2983905E38F);
            assert(pack.xgyro_GET() == 1.3799193E38F);
            assert(pack.zmag_GET() == 1.1749062E38F);
            assert(pack.zacc_GET() == 6.125583E37F);
            assert(pack.fields_updated_GET() == (char)25014);
            assert(pack.xmag_GET() == 3.2002712E38F);
            assert(pack.abs_pressure_GET() == 5.7531364E37F);
            assert(pack.ygyro_GET() == -1.955493E38F);
            assert(pack.yacc_GET() == 1.5320837E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.xmag_SET(3.2002712E38F) ;
        p105.time_usec_SET(1868281731735983364L) ;
        p105.zgyro_SET(2.78485E37F) ;
        p105.zacc_SET(6.125583E37F) ;
        p105.ygyro_SET(-1.955493E38F) ;
        p105.abs_pressure_SET(5.7531364E37F) ;
        p105.xacc_SET(-2.0079945E38F) ;
        p105.ymag_SET(-8.0102497E37F) ;
        p105.fields_updated_SET((char)25014) ;
        p105.xgyro_SET(1.3799193E38F) ;
        p105.yacc_SET(1.5320837E38F) ;
        p105.pressure_alt_SET(-1.2983905E38F) ;
        p105.diff_pressure_SET(-2.1578005E38F) ;
        p105.zmag_SET(1.1749062E38F) ;
        p105.temperature_SET(-2.2325986E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_x_GET() == -1.4174765E38F);
            assert(pack.quality_GET() == (char)18);
            assert(pack.time_delta_distance_us_GET() == 411619602L);
            assert(pack.integrated_ygyro_GET() == -2.657627E37F);
            assert(pack.distance_GET() == -1.795366E38F);
            assert(pack.integrated_zgyro_GET() == -2.9710195E38F);
            assert(pack.sensor_id_GET() == (char)200);
            assert(pack.integrated_xgyro_GET() == 1.3513725E38F);
            assert(pack.integrated_y_GET() == -6.0570414E37F);
            assert(pack.temperature_GET() == (short) -6858);
            assert(pack.time_usec_GET() == 4685646161357921773L);
            assert(pack.integration_time_us_GET() == 4127569260L);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.distance_SET(-1.795366E38F) ;
        p106.integrated_x_SET(-1.4174765E38F) ;
        p106.time_usec_SET(4685646161357921773L) ;
        p106.quality_SET((char)18) ;
        p106.sensor_id_SET((char)200) ;
        p106.integrated_zgyro_SET(-2.9710195E38F) ;
        p106.temperature_SET((short) -6858) ;
        p106.integrated_xgyro_SET(1.3513725E38F) ;
        p106.time_delta_distance_us_SET(411619602L) ;
        p106.integrated_y_SET(-6.0570414E37F) ;
        p106.integration_time_us_SET(4127569260L) ;
        p106.integrated_ygyro_SET(-2.657627E37F) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.diff_pressure_GET() == 9.292271E37F);
            assert(pack.fields_updated_GET() == 1347820119L);
            assert(pack.zacc_GET() == 2.1948342E38F);
            assert(pack.abs_pressure_GET() == -2.7705362E38F);
            assert(pack.temperature_GET() == -2.2346106E38F);
            assert(pack.ymag_GET() == 3.3274808E38F);
            assert(pack.pressure_alt_GET() == -9.117824E37F);
            assert(pack.xmag_GET() == 8.503375E37F);
            assert(pack.xgyro_GET() == 1.2260443E38F);
            assert(pack.xacc_GET() == 1.7757864E38F);
            assert(pack.zmag_GET() == 2.0493992E38F);
            assert(pack.ygyro_GET() == -3.0630314E38F);
            assert(pack.zgyro_GET() == -1.4975425E38F);
            assert(pack.yacc_GET() == -2.3854345E38F);
            assert(pack.time_usec_GET() == 622041337824459198L);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.temperature_SET(-2.2346106E38F) ;
        p107.xacc_SET(1.7757864E38F) ;
        p107.xmag_SET(8.503375E37F) ;
        p107.diff_pressure_SET(9.292271E37F) ;
        p107.pressure_alt_SET(-9.117824E37F) ;
        p107.zacc_SET(2.1948342E38F) ;
        p107.abs_pressure_SET(-2.7705362E38F) ;
        p107.ymag_SET(3.3274808E38F) ;
        p107.time_usec_SET(622041337824459198L) ;
        p107.ygyro_SET(-3.0630314E38F) ;
        p107.yacc_SET(-2.3854345E38F) ;
        p107.zmag_SET(2.0493992E38F) ;
        p107.xgyro_SET(1.2260443E38F) ;
        p107.fields_updated_SET(1347820119L) ;
        p107.zgyro_SET(-1.4975425E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == -1.5638352E38F);
            assert(pack.xacc_GET() == 7.05517E37F);
            assert(pack.q2_GET() == -4.912511E37F);
            assert(pack.ve_GET() == -1.7915712E38F);
            assert(pack.std_dev_vert_GET() == 1.5558276E38F);
            assert(pack.q4_GET() == 8.673766E37F);
            assert(pack.yaw_GET() == 1.8375451E38F);
            assert(pack.q1_GET() == -2.706939E38F);
            assert(pack.vd_GET() == -2.2055508E38F);
            assert(pack.zacc_GET() == 1.809816E37F);
            assert(pack.xgyro_GET() == -2.8712872E38F);
            assert(pack.pitch_GET() == -1.1373393E37F);
            assert(pack.lon_GET() == 3.1200601E38F);
            assert(pack.lat_GET() == 2.8059217E38F);
            assert(pack.zgyro_GET() == 6.8002016E37F);
            assert(pack.vn_GET() == 2.9785576E38F);
            assert(pack.std_dev_horz_GET() == -2.201477E37F);
            assert(pack.q3_GET() == 2.662944E38F);
            assert(pack.yacc_GET() == 1.4083015E38F);
            assert(pack.roll_GET() == 2.5919703E38F);
            assert(pack.alt_GET() == 2.5573776E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.xacc_SET(7.05517E37F) ;
        p108.yacc_SET(1.4083015E38F) ;
        p108.ve_SET(-1.7915712E38F) ;
        p108.zgyro_SET(6.8002016E37F) ;
        p108.ygyro_SET(-1.5638352E38F) ;
        p108.alt_SET(2.5573776E38F) ;
        p108.lon_SET(3.1200601E38F) ;
        p108.vd_SET(-2.2055508E38F) ;
        p108.roll_SET(2.5919703E38F) ;
        p108.q1_SET(-2.706939E38F) ;
        p108.std_dev_vert_SET(1.5558276E38F) ;
        p108.std_dev_horz_SET(-2.201477E37F) ;
        p108.q2_SET(-4.912511E37F) ;
        p108.xgyro_SET(-2.8712872E38F) ;
        p108.vn_SET(2.9785576E38F) ;
        p108.lat_SET(2.8059217E38F) ;
        p108.q3_SET(2.662944E38F) ;
        p108.q4_SET(8.673766E37F) ;
        p108.yaw_SET(1.8375451E38F) ;
        p108.pitch_SET(-1.1373393E37F) ;
        p108.zacc_SET(1.809816E37F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.noise_GET() == (char)181);
            assert(pack.fixed__GET() == (char)7215);
            assert(pack.remrssi_GET() == (char)61);
            assert(pack.txbuf_GET() == (char)4);
            assert(pack.remnoise_GET() == (char)244);
            assert(pack.rxerrors_GET() == (char)2979);
            assert(pack.rssi_GET() == (char)243);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)243) ;
        p109.txbuf_SET((char)4) ;
        p109.rxerrors_SET((char)2979) ;
        p109.noise_SET((char)181) ;
        p109.remnoise_SET((char)244) ;
        p109.remrssi_SET((char)61) ;
        p109.fixed__SET((char)7215) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)107);
            assert(pack.target_component_GET() == (char)120);
            assert(pack.target_network_GET() == (char)171);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)227, (char)47, (char)232, (char)106, (char)136, (char)207, (char)108, (char)164, (char)250, (char)225, (char)68, (char)133, (char)166, (char)202, (char)10, (char)29, (char)172, (char)106, (char)78, (char)13, (char)74, (char)42, (char)27, (char)96, (char)4, (char)16, (char)242, (char)145, (char)229, (char)90, (char)79, (char)47, (char)32, (char)22, (char)92, (char)73, (char)253, (char)23, (char)27, (char)163, (char)146, (char)74, (char)26, (char)140, (char)101, (char)170, (char)171, (char)2, (char)110, (char)48, (char)131, (char)253, (char)102, (char)122, (char)166, (char)156, (char)113, (char)10, (char)171, (char)118, (char)201, (char)245, (char)248, (char)238, (char)96, (char)7, (char)245, (char)157, (char)152, (char)114, (char)152, (char)174, (char)43, (char)109, (char)173, (char)118, (char)243, (char)52, (char)28, (char)36, (char)211, (char)155, (char)178, (char)1, (char)8, (char)3, (char)90, (char)28, (char)148, (char)136, (char)203, (char)217, (char)149, (char)58, (char)252, (char)168, (char)97, (char)150, (char)97, (char)235, (char)167, (char)48, (char)104, (char)215, (char)220, (char)182, (char)233, (char)79, (char)117, (char)175, (char)7, (char)2, (char)221, (char)158, (char)249, (char)192, (char)22, (char)117, (char)4, (char)179, (char)152, (char)113, (char)228, (char)27, (char)125, (char)25, (char)171, (char)137, (char)213, (char)118, (char)247, (char)41, (char)141, (char)171, (char)35, (char)199, (char)125, (char)67, (char)103, (char)46, (char)54, (char)171, (char)238, (char)56, (char)217, (char)217, (char)96, (char)30, (char)128, (char)244, (char)114, (char)0, (char)201, (char)44, (char)8, (char)9, (char)92, (char)137, (char)93, (char)185, (char)69, (char)136, (char)29, (char)94, (char)139, (char)43, (char)162, (char)66, (char)20, (char)172, (char)12, (char)214, (char)240, (char)202, (char)244, (char)186, (char)248, (char)27, (char)144, (char)185, (char)222, (char)189, (char)63, (char)112, (char)217, (char)2, (char)176, (char)158, (char)147, (char)206, (char)253, (char)167, (char)91, (char)150, (char)234, (char)111, (char)204, (char)125, (char)97, (char)19, (char)141, (char)243, (char)51, (char)11, (char)228, (char)113, (char)60, (char)118, (char)136, (char)236, (char)14, (char)41, (char)180, (char)238, (char)104, (char)207, (char)87, (char)32, (char)144, (char)233, (char)254, (char)224, (char)42, (char)11, (char)164, (char)224, (char)239, (char)235, (char)224, (char)200, (char)158, (char)65, (char)229, (char)83, (char)103, (char)71, (char)244, (char)83, (char)23, (char)42, (char)131, (char)66, (char)55, (char)206, (char)60, (char)94, (char)240, (char)42, (char)98, (char)100, (char)19}));
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_system_SET((char)107) ;
        p110.payload_SET(new char[] {(char)227, (char)47, (char)232, (char)106, (char)136, (char)207, (char)108, (char)164, (char)250, (char)225, (char)68, (char)133, (char)166, (char)202, (char)10, (char)29, (char)172, (char)106, (char)78, (char)13, (char)74, (char)42, (char)27, (char)96, (char)4, (char)16, (char)242, (char)145, (char)229, (char)90, (char)79, (char)47, (char)32, (char)22, (char)92, (char)73, (char)253, (char)23, (char)27, (char)163, (char)146, (char)74, (char)26, (char)140, (char)101, (char)170, (char)171, (char)2, (char)110, (char)48, (char)131, (char)253, (char)102, (char)122, (char)166, (char)156, (char)113, (char)10, (char)171, (char)118, (char)201, (char)245, (char)248, (char)238, (char)96, (char)7, (char)245, (char)157, (char)152, (char)114, (char)152, (char)174, (char)43, (char)109, (char)173, (char)118, (char)243, (char)52, (char)28, (char)36, (char)211, (char)155, (char)178, (char)1, (char)8, (char)3, (char)90, (char)28, (char)148, (char)136, (char)203, (char)217, (char)149, (char)58, (char)252, (char)168, (char)97, (char)150, (char)97, (char)235, (char)167, (char)48, (char)104, (char)215, (char)220, (char)182, (char)233, (char)79, (char)117, (char)175, (char)7, (char)2, (char)221, (char)158, (char)249, (char)192, (char)22, (char)117, (char)4, (char)179, (char)152, (char)113, (char)228, (char)27, (char)125, (char)25, (char)171, (char)137, (char)213, (char)118, (char)247, (char)41, (char)141, (char)171, (char)35, (char)199, (char)125, (char)67, (char)103, (char)46, (char)54, (char)171, (char)238, (char)56, (char)217, (char)217, (char)96, (char)30, (char)128, (char)244, (char)114, (char)0, (char)201, (char)44, (char)8, (char)9, (char)92, (char)137, (char)93, (char)185, (char)69, (char)136, (char)29, (char)94, (char)139, (char)43, (char)162, (char)66, (char)20, (char)172, (char)12, (char)214, (char)240, (char)202, (char)244, (char)186, (char)248, (char)27, (char)144, (char)185, (char)222, (char)189, (char)63, (char)112, (char)217, (char)2, (char)176, (char)158, (char)147, (char)206, (char)253, (char)167, (char)91, (char)150, (char)234, (char)111, (char)204, (char)125, (char)97, (char)19, (char)141, (char)243, (char)51, (char)11, (char)228, (char)113, (char)60, (char)118, (char)136, (char)236, (char)14, (char)41, (char)180, (char)238, (char)104, (char)207, (char)87, (char)32, (char)144, (char)233, (char)254, (char)224, (char)42, (char)11, (char)164, (char)224, (char)239, (char)235, (char)224, (char)200, (char)158, (char)65, (char)229, (char)83, (char)103, (char)71, (char)244, (char)83, (char)23, (char)42, (char)131, (char)66, (char)55, (char)206, (char)60, (char)94, (char)240, (char)42, (char)98, (char)100, (char)19}, 0) ;
        p110.target_network_SET((char)171) ;
        p110.target_component_SET((char)120) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == 7884362181842942956L);
            assert(pack.tc1_GET() == -455512213365773985L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-455512213365773985L) ;
        p111.ts1_SET(7884362181842942956L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3768346076226568997L);
            assert(pack.seq_GET() == 2608277250L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(3768346076226568997L) ;
        p112.seq_SET(2608277250L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.epv_GET() == (char)37829);
            assert(pack.cog_GET() == (char)6994);
            assert(pack.vd_GET() == (short) -25659);
            assert(pack.fix_type_GET() == (char)124);
            assert(pack.lon_GET() == 650657123);
            assert(pack.alt_GET() == -324358939);
            assert(pack.time_usec_GET() == 414334033208021790L);
            assert(pack.ve_GET() == (short)4994);
            assert(pack.satellites_visible_GET() == (char)108);
            assert(pack.lat_GET() == 1916419292);
            assert(pack.vel_GET() == (char)18087);
            assert(pack.vn_GET() == (short)12300);
            assert(pack.eph_GET() == (char)18707);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.cog_SET((char)6994) ;
        p113.alt_SET(-324358939) ;
        p113.fix_type_SET((char)124) ;
        p113.vd_SET((short) -25659) ;
        p113.vel_SET((char)18087) ;
        p113.time_usec_SET(414334033208021790L) ;
        p113.eph_SET((char)18707) ;
        p113.epv_SET((char)37829) ;
        p113.vn_SET((short)12300) ;
        p113.lon_SET(650657123) ;
        p113.ve_SET((short)4994) ;
        p113.lat_SET(1916419292) ;
        p113.satellites_visible_SET((char)108) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.distance_GET() == 2.6474954E38F);
            assert(pack.integration_time_us_GET() == 3211974093L);
            assert(pack.quality_GET() == (char)2);
            assert(pack.sensor_id_GET() == (char)114);
            assert(pack.integrated_x_GET() == -2.5713818E38F);
            assert(pack.integrated_zgyro_GET() == 1.9805475E38F);
            assert(pack.integrated_xgyro_GET() == 5.9041263E37F);
            assert(pack.time_usec_GET() == 2593294333245770276L);
            assert(pack.integrated_y_GET() == -7.562879E37F);
            assert(pack.integrated_ygyro_GET() == -2.7467194E38F);
            assert(pack.temperature_GET() == (short) -24364);
            assert(pack.time_delta_distance_us_GET() == 1868225958L);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.temperature_SET((short) -24364) ;
        p114.integrated_x_SET(-2.5713818E38F) ;
        p114.sensor_id_SET((char)114) ;
        p114.quality_SET((char)2) ;
        p114.time_delta_distance_us_SET(1868225958L) ;
        p114.integrated_ygyro_SET(-2.7467194E38F) ;
        p114.distance_SET(2.6474954E38F) ;
        p114.integrated_xgyro_SET(5.9041263E37F) ;
        p114.integrated_zgyro_SET(1.9805475E38F) ;
        p114.integration_time_us_SET(3211974093L) ;
        p114.time_usec_SET(2593294333245770276L) ;
        p114.integrated_y_SET(-7.562879E37F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 1850242230);
            assert(pack.vz_GET() == (short) -16381);
            assert(pack.yawspeed_GET() == -1.5613303E38F);
            assert(pack.pitchspeed_GET() == -1.8651692E38F);
            assert(pack.zacc_GET() == (short)30095);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {2.7653402E38F, -2.557694E38F, 4.7090416E37F, 2.754211E38F}));
            assert(pack.lat_GET() == 1844095870);
            assert(pack.alt_GET() == -1220969258);
            assert(pack.time_usec_GET() == 7894795894746565398L);
            assert(pack.true_airspeed_GET() == (char)8680);
            assert(pack.vy_GET() == (short)16056);
            assert(pack.rollspeed_GET() == 8.537379E37F);
            assert(pack.xacc_GET() == (short) -10364);
            assert(pack.vx_GET() == (short) -11871);
            assert(pack.yacc_GET() == (short)4076);
            assert(pack.ind_airspeed_GET() == (char)60113);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.vz_SET((short) -16381) ;
        p115.lat_SET(1844095870) ;
        p115.yacc_SET((short)4076) ;
        p115.vy_SET((short)16056) ;
        p115.rollspeed_SET(8.537379E37F) ;
        p115.attitude_quaternion_SET(new float[] {2.7653402E38F, -2.557694E38F, 4.7090416E37F, 2.754211E38F}, 0) ;
        p115.vx_SET((short) -11871) ;
        p115.ind_airspeed_SET((char)60113) ;
        p115.xacc_SET((short) -10364) ;
        p115.lon_SET(1850242230) ;
        p115.zacc_SET((short)30095) ;
        p115.true_airspeed_SET((char)8680) ;
        p115.pitchspeed_SET(-1.8651692E38F) ;
        p115.time_usec_SET(7894795894746565398L) ;
        p115.alt_SET(-1220969258) ;
        p115.yawspeed_SET(-1.5613303E38F) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2779057102L);
            assert(pack.zmag_GET() == (short)31722);
            assert(pack.zacc_GET() == (short) -2295);
            assert(pack.xacc_GET() == (short)26769);
            assert(pack.yacc_GET() == (short) -18819);
            assert(pack.xmag_GET() == (short)7081);
            assert(pack.ygyro_GET() == (short) -9939);
            assert(pack.ymag_GET() == (short)25925);
            assert(pack.zgyro_GET() == (short)26890);
            assert(pack.xgyro_GET() == (short) -16482);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.xmag_SET((short)7081) ;
        p116.time_boot_ms_SET(2779057102L) ;
        p116.zmag_SET((short)31722) ;
        p116.xacc_SET((short)26769) ;
        p116.zgyro_SET((short)26890) ;
        p116.ygyro_SET((short) -9939) ;
        p116.xgyro_SET((short) -16482) ;
        p116.ymag_SET((short)25925) ;
        p116.zacc_SET((short) -2295) ;
        p116.yacc_SET((short) -18819) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_GET() == (char)47773);
            assert(pack.target_component_GET() == (char)188);
            assert(pack.end_GET() == (char)29725);
            assert(pack.target_system_GET() == (char)182);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.start_SET((char)47773) ;
        p117.target_component_SET((char)188) ;
        p117.target_system_SET((char)182) ;
        p117.end_SET((char)29725) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.num_logs_GET() == (char)40871);
            assert(pack.last_log_num_GET() == (char)37777);
            assert(pack.id_GET() == (char)52904);
            assert(pack.size_GET() == 1120414039L);
            assert(pack.time_utc_GET() == 3643152071L);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.time_utc_SET(3643152071L) ;
        p118.num_logs_SET((char)40871) ;
        p118.id_SET((char)52904) ;
        p118.last_log_num_SET((char)37777) ;
        p118.size_SET(1120414039L) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)73);
            assert(pack.ofs_GET() == 3548794054L);
            assert(pack.target_component_GET() == (char)141);
            assert(pack.id_GET() == (char)27715);
            assert(pack.count_GET() == 4037283342L);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.count_SET(4037283342L) ;
        p119.target_system_SET((char)73) ;
        p119.id_SET((char)27715) ;
        p119.target_component_SET((char)141) ;
        p119.ofs_SET(3548794054L) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)42968);
            assert(pack.ofs_GET() == 4022468634L);
            assert(pack.count_GET() == (char)125);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)221, (char)18, (char)181, (char)28, (char)72, (char)154, (char)103, (char)1, (char)47, (char)81, (char)106, (char)28, (char)182, (char)188, (char)97, (char)60, (char)237, (char)254, (char)132, (char)26, (char)236, (char)168, (char)148, (char)182, (char)218, (char)87, (char)30, (char)20, (char)57, (char)160, (char)155, (char)8, (char)247, (char)172, (char)197, (char)104, (char)22, (char)52, (char)166, (char)153, (char)171, (char)48, (char)174, (char)11, (char)10, (char)161, (char)135, (char)106, (char)48, (char)57, (char)120, (char)190, (char)8, (char)60, (char)148, (char)66, (char)6, (char)41, (char)90, (char)211, (char)215, (char)175, (char)108, (char)155, (char)201, (char)105, (char)121, (char)59, (char)125, (char)97, (char)241, (char)61, (char)18, (char)6, (char)235, (char)205, (char)158, (char)125, (char)90, (char)121, (char)50, (char)170, (char)146, (char)106, (char)39, (char)240, (char)153, (char)66, (char)120, (char)208}));
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)42968) ;
        p120.data__SET(new char[] {(char)221, (char)18, (char)181, (char)28, (char)72, (char)154, (char)103, (char)1, (char)47, (char)81, (char)106, (char)28, (char)182, (char)188, (char)97, (char)60, (char)237, (char)254, (char)132, (char)26, (char)236, (char)168, (char)148, (char)182, (char)218, (char)87, (char)30, (char)20, (char)57, (char)160, (char)155, (char)8, (char)247, (char)172, (char)197, (char)104, (char)22, (char)52, (char)166, (char)153, (char)171, (char)48, (char)174, (char)11, (char)10, (char)161, (char)135, (char)106, (char)48, (char)57, (char)120, (char)190, (char)8, (char)60, (char)148, (char)66, (char)6, (char)41, (char)90, (char)211, (char)215, (char)175, (char)108, (char)155, (char)201, (char)105, (char)121, (char)59, (char)125, (char)97, (char)241, (char)61, (char)18, (char)6, (char)235, (char)205, (char)158, (char)125, (char)90, (char)121, (char)50, (char)170, (char)146, (char)106, (char)39, (char)240, (char)153, (char)66, (char)120, (char)208}, 0) ;
        p120.count_SET((char)125) ;
        p120.ofs_SET(4022468634L) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)50);
            assert(pack.target_component_GET() == (char)206);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)50) ;
        p121.target_component_SET((char)206) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)173);
            assert(pack.target_component_GET() == (char)157);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)173) ;
        p122.target_component_SET((char)157) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)48);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)175, (char)74, (char)61, (char)180, (char)196, (char)144, (char)166, (char)2, (char)219, (char)212, (char)26, (char)142, (char)69, (char)114, (char)140, (char)81, (char)249, (char)203, (char)82, (char)113, (char)217, (char)243, (char)141, (char)151, (char)195, (char)165, (char)17, (char)172, (char)163, (char)28, (char)161, (char)31, (char)165, (char)243, (char)36, (char)194, (char)162, (char)86, (char)78, (char)120, (char)34, (char)4, (char)222, (char)88, (char)98, (char)5, (char)182, (char)208, (char)120, (char)44, (char)51, (char)222, (char)197, (char)29, (char)47, (char)130, (char)45, (char)233, (char)216, (char)145, (char)32, (char)233, (char)40, (char)27, (char)171, (char)134, (char)39, (char)130, (char)100, (char)38, (char)44, (char)61, (char)101, (char)77, (char)88, (char)137, (char)201, (char)55, (char)195, (char)13, (char)225, (char)108, (char)37, (char)135, (char)105, (char)210, (char)66, (char)57, (char)119, (char)170, (char)43, (char)113, (char)125, (char)151, (char)173, (char)73, (char)224, (char)85, (char)65, (char)157, (char)86, (char)11, (char)205, (char)44, (char)49, (char)16, (char)14, (char)182, (char)14, (char)72}));
            assert(pack.target_system_GET() == (char)254);
            assert(pack.len_GET() == (char)177);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_component_SET((char)48) ;
        p123.target_system_SET((char)254) ;
        p123.data__SET(new char[] {(char)175, (char)74, (char)61, (char)180, (char)196, (char)144, (char)166, (char)2, (char)219, (char)212, (char)26, (char)142, (char)69, (char)114, (char)140, (char)81, (char)249, (char)203, (char)82, (char)113, (char)217, (char)243, (char)141, (char)151, (char)195, (char)165, (char)17, (char)172, (char)163, (char)28, (char)161, (char)31, (char)165, (char)243, (char)36, (char)194, (char)162, (char)86, (char)78, (char)120, (char)34, (char)4, (char)222, (char)88, (char)98, (char)5, (char)182, (char)208, (char)120, (char)44, (char)51, (char)222, (char)197, (char)29, (char)47, (char)130, (char)45, (char)233, (char)216, (char)145, (char)32, (char)233, (char)40, (char)27, (char)171, (char)134, (char)39, (char)130, (char)100, (char)38, (char)44, (char)61, (char)101, (char)77, (char)88, (char)137, (char)201, (char)55, (char)195, (char)13, (char)225, (char)108, (char)37, (char)135, (char)105, (char)210, (char)66, (char)57, (char)119, (char)170, (char)43, (char)113, (char)125, (char)151, (char)173, (char)73, (char)224, (char)85, (char)65, (char)157, (char)86, (char)11, (char)205, (char)44, (char)49, (char)16, (char)14, (char)182, (char)14, (char)72}, 0) ;
        p123.len_SET((char)177) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -1227712886);
            assert(pack.cog_GET() == (char)12901);
            assert(pack.lat_GET() == -567676970);
            assert(pack.eph_GET() == (char)50375);
            assert(pack.lon_GET() == 1387684517);
            assert(pack.dgps_age_GET() == 543552929L);
            assert(pack.vel_GET() == (char)5607);
            assert(pack.time_usec_GET() == 5278225177223104997L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
            assert(pack.satellites_visible_GET() == (char)254);
            assert(pack.epv_GET() == (char)34628);
            assert(pack.dgps_numch_GET() == (char)167);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.epv_SET((char)34628) ;
        p124.time_usec_SET(5278225177223104997L) ;
        p124.eph_SET((char)50375) ;
        p124.cog_SET((char)12901) ;
        p124.vel_SET((char)5607) ;
        p124.lon_SET(1387684517) ;
        p124.dgps_numch_SET((char)167) ;
        p124.lat_SET(-567676970) ;
        p124.alt_SET(-1227712886) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS) ;
        p124.satellites_visible_SET((char)254) ;
        p124.dgps_age_SET(543552929L) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vcc_GET() == (char)29448);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT));
            assert(pack.Vservo_GET() == (char)55177);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT)) ;
        p125.Vservo_SET((char)55177) ;
        p125.Vcc_SET((char)29448) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.timeout_GET() == (char)13539);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)47, (char)233, (char)59, (char)30, (char)236, (char)114, (char)242, (char)162, (char)251, (char)1, (char)134, (char)109, (char)198, (char)237, (char)250, (char)69, (char)253, (char)82, (char)22, (char)202, (char)231, (char)137, (char)117, (char)86, (char)182, (char)97, (char)84, (char)28, (char)126, (char)146, (char)10, (char)58, (char)170, (char)229, (char)179, (char)99, (char)14, (char)123, (char)182, (char)69, (char)174, (char)183, (char)51, (char)122, (char)167, (char)211, (char)121, (char)170, (char)112, (char)5, (char)73, (char)15, (char)137, (char)167, (char)7, (char)117, (char)204, (char)107, (char)91, (char)55, (char)116, (char)70, (char)227, (char)118, (char)139, (char)29, (char)69, (char)24, (char)229, (char)117}));
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
            assert(pack.baudrate_GET() == 3325448359L);
            assert(pack.count_GET() == (char)170);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.count_SET((char)170) ;
        p126.data__SET(new char[] {(char)47, (char)233, (char)59, (char)30, (char)236, (char)114, (char)242, (char)162, (char)251, (char)1, (char)134, (char)109, (char)198, (char)237, (char)250, (char)69, (char)253, (char)82, (char)22, (char)202, (char)231, (char)137, (char)117, (char)86, (char)182, (char)97, (char)84, (char)28, (char)126, (char)146, (char)10, (char)58, (char)170, (char)229, (char)179, (char)99, (char)14, (char)123, (char)182, (char)69, (char)174, (char)183, (char)51, (char)122, (char)167, (char)211, (char)121, (char)170, (char)112, (char)5, (char)73, (char)15, (char)137, (char)167, (char)7, (char)117, (char)204, (char)107, (char)91, (char)55, (char)116, (char)70, (char)227, (char)118, (char)139, (char)29, (char)69, (char)24, (char)229, (char)117}, 0) ;
        p126.timeout_SET((char)13539) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2) ;
        p126.baudrate_SET(3325448359L) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING)) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.rtk_receiver_id_GET() == (char)61);
            assert(pack.baseline_b_mm_GET() == 1029473490);
            assert(pack.baseline_c_mm_GET() == -1463712522);
            assert(pack.time_last_baseline_ms_GET() == 878169487L);
            assert(pack.tow_GET() == 3289712376L);
            assert(pack.baseline_a_mm_GET() == 1790057906);
            assert(pack.iar_num_hypotheses_GET() == -1188288664);
            assert(pack.baseline_coords_type_GET() == (char)226);
            assert(pack.nsats_GET() == (char)218);
            assert(pack.wn_GET() == (char)52771);
            assert(pack.rtk_health_GET() == (char)185);
            assert(pack.accuracy_GET() == 81122223L);
            assert(pack.rtk_rate_GET() == (char)170);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.baseline_c_mm_SET(-1463712522) ;
        p127.tow_SET(3289712376L) ;
        p127.baseline_b_mm_SET(1029473490) ;
        p127.rtk_receiver_id_SET((char)61) ;
        p127.rtk_health_SET((char)185) ;
        p127.accuracy_SET(81122223L) ;
        p127.baseline_coords_type_SET((char)226) ;
        p127.wn_SET((char)52771) ;
        p127.iar_num_hypotheses_SET(-1188288664) ;
        p127.time_last_baseline_ms_SET(878169487L) ;
        p127.rtk_rate_SET((char)170) ;
        p127.nsats_SET((char)218) ;
        p127.baseline_a_mm_SET(1790057906) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.wn_GET() == (char)44299);
            assert(pack.time_last_baseline_ms_GET() == 2058121509L);
            assert(pack.baseline_a_mm_GET() == -1263150785);
            assert(pack.baseline_b_mm_GET() == 2089758840);
            assert(pack.rtk_receiver_id_GET() == (char)64);
            assert(pack.tow_GET() == 2502531405L);
            assert(pack.iar_num_hypotheses_GET() == 2100020349);
            assert(pack.baseline_c_mm_GET() == -676744055);
            assert(pack.nsats_GET() == (char)35);
            assert(pack.rtk_rate_GET() == (char)81);
            assert(pack.accuracy_GET() == 170752130L);
            assert(pack.baseline_coords_type_GET() == (char)233);
            assert(pack.rtk_health_GET() == (char)82);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.iar_num_hypotheses_SET(2100020349) ;
        p128.baseline_c_mm_SET(-676744055) ;
        p128.baseline_b_mm_SET(2089758840) ;
        p128.nsats_SET((char)35) ;
        p128.tow_SET(2502531405L) ;
        p128.baseline_coords_type_SET((char)233) ;
        p128.time_last_baseline_ms_SET(2058121509L) ;
        p128.rtk_rate_SET((char)81) ;
        p128.baseline_a_mm_SET(-1263150785) ;
        p128.accuracy_SET(170752130L) ;
        p128.rtk_health_SET((char)82) ;
        p128.rtk_receiver_id_SET((char)64) ;
        p128.wn_SET((char)44299) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1734094018L);
            assert(pack.ymag_GET() == (short)9972);
            assert(pack.xacc_GET() == (short) -14158);
            assert(pack.xgyro_GET() == (short) -12073);
            assert(pack.zacc_GET() == (short)4189);
            assert(pack.zmag_GET() == (short) -29586);
            assert(pack.zgyro_GET() == (short)22407);
            assert(pack.ygyro_GET() == (short)6814);
            assert(pack.yacc_GET() == (short)17799);
            assert(pack.xmag_GET() == (short)966);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(1734094018L) ;
        p129.xacc_SET((short) -14158) ;
        p129.zmag_SET((short) -29586) ;
        p129.ygyro_SET((short)6814) ;
        p129.ymag_SET((short)9972) ;
        p129.yacc_SET((short)17799) ;
        p129.xgyro_SET((short) -12073) ;
        p129.zacc_SET((short)4189) ;
        p129.zgyro_SET((short)22407) ;
        p129.xmag_SET((short)966) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.payload_GET() == (char)29);
            assert(pack.type_GET() == (char)238);
            assert(pack.jpg_quality_GET() == (char)137);
            assert(pack.width_GET() == (char)1920);
            assert(pack.size_GET() == 2570162775L);
            assert(pack.height_GET() == (char)34942);
            assert(pack.packets_GET() == (char)43109);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.width_SET((char)1920) ;
        p130.type_SET((char)238) ;
        p130.payload_SET((char)29) ;
        p130.jpg_quality_SET((char)137) ;
        p130.height_SET((char)34942) ;
        p130.packets_SET((char)43109) ;
        p130.size_SET(2570162775L) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)28, (char)188, (char)185, (char)134, (char)27, (char)235, (char)195, (char)165, (char)167, (char)183, (char)219, (char)234, (char)31, (char)87, (char)168, (char)141, (char)178, (char)231, (char)39, (char)224, (char)254, (char)81, (char)229, (char)159, (char)192, (char)55, (char)29, (char)117, (char)44, (char)165, (char)53, (char)174, (char)40, (char)126, (char)140, (char)57, (char)103, (char)91, (char)209, (char)39, (char)244, (char)250, (char)164, (char)38, (char)16, (char)22, (char)153, (char)146, (char)194, (char)61, (char)46, (char)190, (char)98, (char)36, (char)4, (char)7, (char)203, (char)140, (char)59, (char)25, (char)113, (char)155, (char)28, (char)168, (char)60, (char)244, (char)21, (char)129, (char)50, (char)7, (char)120, (char)53, (char)187, (char)38, (char)3, (char)34, (char)98, (char)193, (char)179, (char)113, (char)69, (char)129, (char)17, (char)255, (char)241, (char)46, (char)186, (char)238, (char)14, (char)163, (char)140, (char)170, (char)32, (char)189, (char)182, (char)253, (char)180, (char)230, (char)74, (char)174, (char)175, (char)191, (char)1, (char)152, (char)76, (char)221, (char)204, (char)139, (char)211, (char)241, (char)100, (char)191, (char)44, (char)200, (char)149, (char)96, (char)126, (char)52, (char)96, (char)184, (char)38, (char)220, (char)190, (char)168, (char)59, (char)110, (char)176, (char)198, (char)139, (char)190, (char)79, (char)106, (char)181, (char)113, (char)220, (char)124, (char)5, (char)206, (char)203, (char)164, (char)111, (char)247, (char)230, (char)65, (char)52, (char)245, (char)32, (char)90, (char)246, (char)182, (char)227, (char)176, (char)238, (char)150, (char)3, (char)228, (char)223, (char)251, (char)13, (char)204, (char)142, (char)148, (char)14, (char)175, (char)6, (char)113, (char)77, (char)19, (char)126, (char)65, (char)183, (char)41, (char)168, (char)220, (char)141, (char)35, (char)234, (char)64, (char)197, (char)245, (char)18, (char)229, (char)237, (char)132, (char)128, (char)26, (char)140, (char)30, (char)99, (char)162, (char)208, (char)198, (char)240, (char)71, (char)33, (char)190, (char)111, (char)164, (char)51, (char)141, (char)177, (char)178, (char)77, (char)121, (char)234, (char)208, (char)244, (char)18, (char)202, (char)92, (char)104, (char)206, (char)70, (char)23, (char)207, (char)150, (char)226, (char)15, (char)251, (char)108, (char)1, (char)51, (char)173, (char)223, (char)42, (char)83, (char)193, (char)255, (char)213, (char)66, (char)171, (char)242, (char)180, (char)231, (char)100, (char)46, (char)225, (char)87, (char)58, (char)220, (char)28, (char)137, (char)97, (char)127, (char)124, (char)205, (char)222, (char)244, (char)213, (char)233, (char)95, (char)97, (char)64}));
            assert(pack.seqnr_GET() == (char)54832);
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.data__SET(new char[] {(char)28, (char)188, (char)185, (char)134, (char)27, (char)235, (char)195, (char)165, (char)167, (char)183, (char)219, (char)234, (char)31, (char)87, (char)168, (char)141, (char)178, (char)231, (char)39, (char)224, (char)254, (char)81, (char)229, (char)159, (char)192, (char)55, (char)29, (char)117, (char)44, (char)165, (char)53, (char)174, (char)40, (char)126, (char)140, (char)57, (char)103, (char)91, (char)209, (char)39, (char)244, (char)250, (char)164, (char)38, (char)16, (char)22, (char)153, (char)146, (char)194, (char)61, (char)46, (char)190, (char)98, (char)36, (char)4, (char)7, (char)203, (char)140, (char)59, (char)25, (char)113, (char)155, (char)28, (char)168, (char)60, (char)244, (char)21, (char)129, (char)50, (char)7, (char)120, (char)53, (char)187, (char)38, (char)3, (char)34, (char)98, (char)193, (char)179, (char)113, (char)69, (char)129, (char)17, (char)255, (char)241, (char)46, (char)186, (char)238, (char)14, (char)163, (char)140, (char)170, (char)32, (char)189, (char)182, (char)253, (char)180, (char)230, (char)74, (char)174, (char)175, (char)191, (char)1, (char)152, (char)76, (char)221, (char)204, (char)139, (char)211, (char)241, (char)100, (char)191, (char)44, (char)200, (char)149, (char)96, (char)126, (char)52, (char)96, (char)184, (char)38, (char)220, (char)190, (char)168, (char)59, (char)110, (char)176, (char)198, (char)139, (char)190, (char)79, (char)106, (char)181, (char)113, (char)220, (char)124, (char)5, (char)206, (char)203, (char)164, (char)111, (char)247, (char)230, (char)65, (char)52, (char)245, (char)32, (char)90, (char)246, (char)182, (char)227, (char)176, (char)238, (char)150, (char)3, (char)228, (char)223, (char)251, (char)13, (char)204, (char)142, (char)148, (char)14, (char)175, (char)6, (char)113, (char)77, (char)19, (char)126, (char)65, (char)183, (char)41, (char)168, (char)220, (char)141, (char)35, (char)234, (char)64, (char)197, (char)245, (char)18, (char)229, (char)237, (char)132, (char)128, (char)26, (char)140, (char)30, (char)99, (char)162, (char)208, (char)198, (char)240, (char)71, (char)33, (char)190, (char)111, (char)164, (char)51, (char)141, (char)177, (char)178, (char)77, (char)121, (char)234, (char)208, (char)244, (char)18, (char)202, (char)92, (char)104, (char)206, (char)70, (char)23, (char)207, (char)150, (char)226, (char)15, (char)251, (char)108, (char)1, (char)51, (char)173, (char)223, (char)42, (char)83, (char)193, (char)255, (char)213, (char)66, (char)171, (char)242, (char)180, (char)231, (char)100, (char)46, (char)225, (char)87, (char)58, (char)220, (char)28, (char)137, (char)97, (char)127, (char)124, (char)205, (char)222, (char)244, (char)213, (char)233, (char)95, (char)97, (char)64}, 0) ;
        p131.seqnr_SET((char)54832) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_225);
            assert(pack.min_distance_GET() == (char)50469);
            assert(pack.time_boot_ms_GET() == 2177943688L);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            assert(pack.covariance_GET() == (char)43);
            assert(pack.id_GET() == (char)197);
            assert(pack.max_distance_GET() == (char)29028);
            assert(pack.current_distance_GET() == (char)64327);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_225) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND) ;
        p132.id_SET((char)197) ;
        p132.min_distance_SET((char)50469) ;
        p132.time_boot_ms_SET(2177943688L) ;
        p132.current_distance_SET((char)64327) ;
        p132.max_distance_SET((char)29028) ;
        p132.covariance_SET((char)43) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mask_GET() == 7349643559164407766L);
            assert(pack.grid_spacing_GET() == (char)919);
            assert(pack.lat_GET() == 200134309);
            assert(pack.lon_GET() == -2052269744);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.grid_spacing_SET((char)919) ;
        p133.lon_SET(-2052269744) ;
        p133.mask_SET(7349643559164407766L) ;
        p133.lat_SET(200134309) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -593764659);
            assert(pack.grid_spacing_GET() == (char)62070);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -30916, (short)25870, (short) -15237, (short) -20884, (short)15549, (short)6388, (short) -24708, (short) -18718, (short) -30211, (short)16350, (short)24675, (short)24908, (short)4043, (short)31577, (short)21730, (short) -10845}));
            assert(pack.gridbit_GET() == (char)38);
            assert(pack.lon_GET() == 1948641178);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lon_SET(1948641178) ;
        p134.grid_spacing_SET((char)62070) ;
        p134.data__SET(new short[] {(short) -30916, (short)25870, (short) -15237, (short) -20884, (short)15549, (short)6388, (short) -24708, (short) -18718, (short) -30211, (short)16350, (short)24675, (short)24908, (short)4043, (short)31577, (short)21730, (short) -10845}, 0) ;
        p134.lat_SET(-593764659) ;
        p134.gridbit_SET((char)38) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -953613302);
            assert(pack.lon_GET() == 1594221451);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(1594221451) ;
        p135.lat_SET(-953613302) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.terrain_height_GET() == -2.8952184E37F);
            assert(pack.loaded_GET() == (char)57658);
            assert(pack.lat_GET() == -535670058);
            assert(pack.pending_GET() == (char)49321);
            assert(pack.current_height_GET() == -2.9370467E38F);
            assert(pack.lon_GET() == 680046855);
            assert(pack.spacing_GET() == (char)59743);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.terrain_height_SET(-2.8952184E37F) ;
        p136.pending_SET((char)49321) ;
        p136.lat_SET(-535670058) ;
        p136.current_height_SET(-2.9370467E38F) ;
        p136.spacing_SET((char)59743) ;
        p136.lon_SET(680046855) ;
        p136.loaded_SET((char)57658) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == 1.7394028E38F);
            assert(pack.time_boot_ms_GET() == 1416871556L);
            assert(pack.press_abs_GET() == 3.1889027E38F);
            assert(pack.temperature_GET() == (short)26581);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.temperature_SET((short)26581) ;
        p137.press_abs_SET(3.1889027E38F) ;
        p137.press_diff_SET(1.7394028E38F) ;
        p137.time_boot_ms_SET(1416871556L) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 2.1919973E38F);
            assert(pack.y_GET() == 3.139296E37F);
            assert(pack.z_GET() == -2.9886344E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.2115721E38F, -2.0174164E37F, 4.598266E37F, 9.910868E37F}));
            assert(pack.time_usec_GET() == 6357792532086386638L);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.y_SET(3.139296E37F) ;
        p138.q_SET(new float[] {-3.2115721E38F, -2.0174164E37F, 4.598266E37F, 9.910868E37F}, 0) ;
        p138.x_SET(2.1919973E38F) ;
        p138.time_usec_SET(6357792532086386638L) ;
        p138.z_SET(-2.9886344E38F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)122);
            assert(pack.time_usec_GET() == 3075214748045475786L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.9820013E38F, 1.2012879E38F, -1.4333007E38F, 3.2164955E38F, -1.888565E38F, 4.2024565E37F, -3.0713199E38F, 2.8684801E38F}));
            assert(pack.target_system_GET() == (char)218);
            assert(pack.group_mlx_GET() == (char)138);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.target_component_SET((char)122) ;
        p139.target_system_SET((char)218) ;
        p139.group_mlx_SET((char)138) ;
        p139.controls_SET(new float[] {-1.9820013E38F, 1.2012879E38F, -1.4333007E38F, 3.2164955E38F, -1.888565E38F, 4.2024565E37F, -3.0713199E38F, 2.8684801E38F}, 0) ;
        p139.time_usec_SET(3075214748045475786L) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)245);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.1161331E38F, -2.7961338E38F, -1.0810189E38F, 2.168162E38F, -1.636761E38F, 1.1890328E38F, 2.3017369E38F, -2.1027918E38F}));
            assert(pack.time_usec_GET() == 4743212096051001113L);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.controls_SET(new float[] {1.1161331E38F, -2.7961338E38F, -1.0810189E38F, 2.168162E38F, -1.636761E38F, 1.1890328E38F, 2.3017369E38F, -2.1027918E38F}, 0) ;
        p140.group_mlx_SET((char)245) ;
        p140.time_usec_SET(4743212096051001113L) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_monotonic_GET() == 5.611443E37F);
            assert(pack.altitude_relative_GET() == -7.9387603E37F);
            assert(pack.altitude_amsl_GET() == -6.7190897E37F);
            assert(pack.bottom_clearance_GET() == 6.463756E37F);
            assert(pack.altitude_terrain_GET() == -1.4512753E38F);
            assert(pack.altitude_local_GET() == -9.554386E37F);
            assert(pack.time_usec_GET() == 4501780163572056182L);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_amsl_SET(-6.7190897E37F) ;
        p141.bottom_clearance_SET(6.463756E37F) ;
        p141.altitude_local_SET(-9.554386E37F) ;
        p141.altitude_relative_SET(-7.9387603E37F) ;
        p141.time_usec_SET(4501780163572056182L) ;
        p141.altitude_monotonic_SET(5.611443E37F) ;
        p141.altitude_terrain_SET(-1.4512753E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.uri_type_GET() == (char)210);
            assert(pack.request_id_GET() == (char)103);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)199, (char)145, (char)100, (char)32, (char)42, (char)172, (char)64, (char)0, (char)207, (char)100, (char)71, (char)161, (char)52, (char)89, (char)104, (char)238, (char)201, (char)114, (char)8, (char)240, (char)246, (char)79, (char)197, (char)161, (char)201, (char)108, (char)23, (char)173, (char)79, (char)179, (char)19, (char)89, (char)162, (char)29, (char)149, (char)249, (char)130, (char)182, (char)64, (char)61, (char)150, (char)218, (char)157, (char)164, (char)32, (char)23, (char)157, (char)179, (char)112, (char)211, (char)185, (char)136, (char)219, (char)161, (char)246, (char)150, (char)114, (char)119, (char)207, (char)35, (char)97, (char)81, (char)252, (char)172, (char)128, (char)162, (char)28, (char)41, (char)30, (char)202, (char)120, (char)171, (char)88, (char)172, (char)193, (char)252, (char)127, (char)76, (char)79, (char)53, (char)145, (char)115, (char)126, (char)227, (char)197, (char)107, (char)197, (char)90, (char)241, (char)109, (char)151, (char)83, (char)247, (char)201, (char)154, (char)48, (char)0, (char)69, (char)113, (char)227, (char)254, (char)211, (char)250, (char)228, (char)84, (char)60, (char)243, (char)178, (char)173, (char)1, (char)56, (char)34, (char)32, (char)197, (char)241, (char)152, (char)135, (char)129, (char)101, (char)187}));
            assert(pack.transfer_type_GET() == (char)179);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)158, (char)13, (char)27, (char)185, (char)48, (char)176, (char)195, (char)234, (char)231, (char)194, (char)63, (char)174, (char)106, (char)234, (char)82, (char)120, (char)71, (char)149, (char)247, (char)178, (char)74, (char)62, (char)196, (char)108, (char)253, (char)12, (char)213, (char)100, (char)248, (char)215, (char)22, (char)103, (char)250, (char)212, (char)11, (char)116, (char)74, (char)186, (char)105, (char)130, (char)196, (char)249, (char)253, (char)127, (char)21, (char)173, (char)255, (char)58, (char)254, (char)22, (char)232, (char)139, (char)207, (char)71, (char)204, (char)231, (char)34, (char)76, (char)252, (char)34, (char)173, (char)233, (char)71, (char)42, (char)207, (char)231, (char)149, (char)58, (char)225, (char)166, (char)169, (char)96, (char)119, (char)94, (char)17, (char)30, (char)72, (char)37, (char)54, (char)46, (char)176, (char)106, (char)59, (char)138, (char)155, (char)135, (char)206, (char)127, (char)225, (char)230, (char)93, (char)67, (char)132, (char)198, (char)103, (char)255, (char)87, (char)213, (char)94, (char)65, (char)100, (char)169, (char)204, (char)190, (char)143, (char)246, (char)207, (char)165, (char)125, (char)194, (char)118, (char)1, (char)53, (char)124, (char)85, (char)59, (char)112, (char)192, (char)2, (char)61}));
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.storage_SET(new char[] {(char)158, (char)13, (char)27, (char)185, (char)48, (char)176, (char)195, (char)234, (char)231, (char)194, (char)63, (char)174, (char)106, (char)234, (char)82, (char)120, (char)71, (char)149, (char)247, (char)178, (char)74, (char)62, (char)196, (char)108, (char)253, (char)12, (char)213, (char)100, (char)248, (char)215, (char)22, (char)103, (char)250, (char)212, (char)11, (char)116, (char)74, (char)186, (char)105, (char)130, (char)196, (char)249, (char)253, (char)127, (char)21, (char)173, (char)255, (char)58, (char)254, (char)22, (char)232, (char)139, (char)207, (char)71, (char)204, (char)231, (char)34, (char)76, (char)252, (char)34, (char)173, (char)233, (char)71, (char)42, (char)207, (char)231, (char)149, (char)58, (char)225, (char)166, (char)169, (char)96, (char)119, (char)94, (char)17, (char)30, (char)72, (char)37, (char)54, (char)46, (char)176, (char)106, (char)59, (char)138, (char)155, (char)135, (char)206, (char)127, (char)225, (char)230, (char)93, (char)67, (char)132, (char)198, (char)103, (char)255, (char)87, (char)213, (char)94, (char)65, (char)100, (char)169, (char)204, (char)190, (char)143, (char)246, (char)207, (char)165, (char)125, (char)194, (char)118, (char)1, (char)53, (char)124, (char)85, (char)59, (char)112, (char)192, (char)2, (char)61}, 0) ;
        p142.request_id_SET((char)103) ;
        p142.transfer_type_SET((char)179) ;
        p142.uri_SET(new char[] {(char)199, (char)145, (char)100, (char)32, (char)42, (char)172, (char)64, (char)0, (char)207, (char)100, (char)71, (char)161, (char)52, (char)89, (char)104, (char)238, (char)201, (char)114, (char)8, (char)240, (char)246, (char)79, (char)197, (char)161, (char)201, (char)108, (char)23, (char)173, (char)79, (char)179, (char)19, (char)89, (char)162, (char)29, (char)149, (char)249, (char)130, (char)182, (char)64, (char)61, (char)150, (char)218, (char)157, (char)164, (char)32, (char)23, (char)157, (char)179, (char)112, (char)211, (char)185, (char)136, (char)219, (char)161, (char)246, (char)150, (char)114, (char)119, (char)207, (char)35, (char)97, (char)81, (char)252, (char)172, (char)128, (char)162, (char)28, (char)41, (char)30, (char)202, (char)120, (char)171, (char)88, (char)172, (char)193, (char)252, (char)127, (char)76, (char)79, (char)53, (char)145, (char)115, (char)126, (char)227, (char)197, (char)107, (char)197, (char)90, (char)241, (char)109, (char)151, (char)83, (char)247, (char)201, (char)154, (char)48, (char)0, (char)69, (char)113, (char)227, (char)254, (char)211, (char)250, (char)228, (char)84, (char)60, (char)243, (char)178, (char)173, (char)1, (char)56, (char)34, (char)32, (char)197, (char)241, (char)152, (char)135, (char)129, (char)101, (char)187}, 0) ;
        p142.uri_type_SET((char)210) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 540551440L);
            assert(pack.press_diff_GET() == -2.4896491E38F);
            assert(pack.press_abs_GET() == 1.2132653E38F);
            assert(pack.temperature_GET() == (short) -13013);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.press_diff_SET(-2.4896491E38F) ;
        p143.press_abs_SET(1.2132653E38F) ;
        p143.temperature_SET((short) -13013) ;
        p143.time_boot_ms_SET(540551440L) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.timestamp_GET() == 5521572121571885650L);
            assert(pack.custom_state_GET() == 6375258065408889722L);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {2.8192923E38F, 9.228745E37F, -2.563551E38F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-2.2693035E38F, 2.3228505E38F, 2.6900002E38F, -2.2077062E38F}));
            assert(pack.alt_GET() == -1.6307228E38F);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {1.1812889E38F, -2.2447267E38F, 2.5704725E38F}));
            assert(Arrays.equals(pack.acc_GET(),  new float[] {1.4734781E38F, 3.13727E38F, 5.4230326E37F}));
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {4.6646485E37F, 2.9163095E38F, 2.9316777E38F}));
            assert(pack.lon_GET() == -1051237217);
            assert(pack.est_capabilities_GET() == (char)141);
            assert(pack.lat_GET() == -852517247);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.rates_SET(new float[] {1.1812889E38F, -2.2447267E38F, 2.5704725E38F}, 0) ;
        p144.est_capabilities_SET((char)141) ;
        p144.lat_SET(-852517247) ;
        p144.acc_SET(new float[] {1.4734781E38F, 3.13727E38F, 5.4230326E37F}, 0) ;
        p144.timestamp_SET(5521572121571885650L) ;
        p144.alt_SET(-1.6307228E38F) ;
        p144.lon_SET(-1051237217) ;
        p144.position_cov_SET(new float[] {4.6646485E37F, 2.9163095E38F, 2.9316777E38F}, 0) ;
        p144.custom_state_SET(6375258065408889722L) ;
        p144.vel_SET(new float[] {2.8192923E38F, 9.228745E37F, -2.563551E38F}, 0) ;
        p144.attitude_q_SET(new float[] {-2.2693035E38F, 2.3228505E38F, 2.6900002E38F, -2.2077062E38F}, 0) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == 1.5420453E38F);
            assert(pack.z_pos_GET() == -1.2644764E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {-3.1894801E38F, -2.8250843E38F, -3.1449105E38F}));
            assert(pack.time_usec_GET() == 5452349904541746507L);
            assert(pack.airspeed_GET() == -1.3441315E37F);
            assert(pack.y_vel_GET() == 2.524304E38F);
            assert(pack.pitch_rate_GET() == 9.800968E37F);
            assert(pack.x_pos_GET() == -2.941332E38F);
            assert(pack.z_vel_GET() == -1.872533E38F);
            assert(pack.roll_rate_GET() == 1.2837283E38F);
            assert(pack.x_acc_GET() == -1.9843403E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.0431159E38F, 3.8517939E37F, 2.0102588E38F, -3.3282093E38F}));
            assert(pack.y_acc_GET() == -9.994859E37F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {2.5875793E38F, -2.3689863E38F, -2.5051443E38F}));
            assert(pack.y_pos_GET() == 2.5873351E38F);
            assert(pack.z_acc_GET() == 7.578732E37F);
            assert(pack.x_vel_GET() == -3.0853257E38F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.z_pos_SET(-1.2644764E38F) ;
        p146.y_acc_SET(-9.994859E37F) ;
        p146.roll_rate_SET(1.2837283E38F) ;
        p146.pos_variance_SET(new float[] {-3.1894801E38F, -2.8250843E38F, -3.1449105E38F}, 0) ;
        p146.y_pos_SET(2.5873351E38F) ;
        p146.vel_variance_SET(new float[] {2.5875793E38F, -2.3689863E38F, -2.5051443E38F}, 0) ;
        p146.z_vel_SET(-1.872533E38F) ;
        p146.airspeed_SET(-1.3441315E37F) ;
        p146.q_SET(new float[] {-1.0431159E38F, 3.8517939E37F, 2.0102588E38F, -3.3282093E38F}, 0) ;
        p146.x_pos_SET(-2.941332E38F) ;
        p146.time_usec_SET(5452349904541746507L) ;
        p146.z_acc_SET(7.578732E37F) ;
        p146.y_vel_SET(2.524304E38F) ;
        p146.pitch_rate_SET(9.800968E37F) ;
        p146.x_vel_SET(-3.0853257E38F) ;
        p146.x_acc_SET(-1.9843403E38F) ;
        p146.yaw_rate_SET(1.5420453E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -19155);
            assert(pack.energy_consumed_GET() == -673061755);
            assert(pack.id_GET() == (char)214);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
            assert(pack.current_battery_GET() == (short)11619);
            assert(pack.current_consumed_GET() == 1725559124);
            assert(pack.battery_remaining_GET() == (byte) - 39);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)32460, (char)20585, (char)11350, (char)6683, (char)733, (char)32214, (char)9241, (char)21116, (char)14556, (char)36267}));
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.current_battery_SET((short)11619) ;
        p147.current_consumed_SET(1725559124) ;
        p147.temperature_SET((short) -19155) ;
        p147.energy_consumed_SET(-673061755) ;
        p147.id_SET((char)214) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD) ;
        p147.battery_remaining_SET((byte) - 39) ;
        p147.voltages_SET(new char[] {(char)32460, (char)20585, (char)11350, (char)6683, (char)733, (char)32214, (char)9241, (char)21116, (char)14556, (char)36267}, 0) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.vendor_id_GET() == (char)47961);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)89, (char)122, (char)91, (char)23, (char)58, (char)135, (char)41, (char)8}));
            assert(pack.middleware_sw_version_GET() == 593800779L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)17, (char)185, (char)172, (char)196, (char)175, (char)152, (char)207, (char)142, (char)143, (char)109, (char)198, (char)212, (char)248, (char)125, (char)147, (char)71, (char)40, (char)133}));
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)68, (char)35, (char)79, (char)142, (char)99, (char)146, (char)191, (char)229}));
            assert(pack.product_id_GET() == (char)63597);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)6, (char)157, (char)201, (char)239, (char)235, (char)182, (char)227, (char)170}));
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT));
            assert(pack.os_sw_version_GET() == 2650884428L);
            assert(pack.board_version_GET() == 1083911391L);
            assert(pack.uid_GET() == 7690452610136008255L);
            assert(pack.flight_sw_version_GET() == 2124521726L);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.middleware_sw_version_SET(593800779L) ;
        p148.vendor_id_SET((char)47961) ;
        p148.uid2_SET(new char[] {(char)17, (char)185, (char)172, (char)196, (char)175, (char)152, (char)207, (char)142, (char)143, (char)109, (char)198, (char)212, (char)248, (char)125, (char)147, (char)71, (char)40, (char)133}, 0, PH) ;
        p148.middleware_custom_version_SET(new char[] {(char)68, (char)35, (char)79, (char)142, (char)99, (char)146, (char)191, (char)229}, 0) ;
        p148.os_sw_version_SET(2650884428L) ;
        p148.os_custom_version_SET(new char[] {(char)6, (char)157, (char)201, (char)239, (char)235, (char)182, (char)227, (char)170}, 0) ;
        p148.uid_SET(7690452610136008255L) ;
        p148.board_version_SET(1083911391L) ;
        p148.product_id_SET((char)63597) ;
        p148.flight_custom_version_SET(new char[] {(char)89, (char)122, (char)91, (char)23, (char)58, (char)135, (char)41, (char)8}, 0) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT)) ;
        p148.flight_sw_version_SET(2124521726L) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.z_TRY(ph) == 3.0805834E38F);
            assert(pack.size_y_GET() == -1.7548213E38F);
            assert(pack.size_x_GET() == -3.1140606E38F);
            assert(pack.target_num_GET() == (char)218);
            assert(pack.angle_y_GET() == 1.6101E38F);
            assert(pack.distance_GET() == 2.5805273E38F);
            assert(pack.angle_x_GET() == 1.6246923E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
            assert(pack.time_usec_GET() == 62430282589298048L);
            assert(pack.position_valid_TRY(ph) == (char)5);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {3.308737E38F, 3.2482098E38F, -2.8323138E38F, 2.1948356E37F}));
            assert(pack.x_TRY(ph) == 1.4036496E38F);
            assert(pack.y_TRY(ph) == 1.7246997E38F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.angle_y_SET(1.6101E38F) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER) ;
        p149.angle_x_SET(1.6246923E38F) ;
        p149.time_usec_SET(62430282589298048L) ;
        p149.y_SET(1.7246997E38F, PH) ;
        p149.distance_SET(2.5805273E38F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p149.size_y_SET(-1.7548213E38F) ;
        p149.z_SET(3.0805834E38F, PH) ;
        p149.size_x_SET(-3.1140606E38F) ;
        p149.x_SET(1.4036496E38F, PH) ;
        p149.q_SET(new float[] {3.308737E38F, 3.2482098E38F, -2.8323138E38F, 2.1948356E37F}, 0, PH) ;
        p149.position_valid_SET((char)5, PH) ;
        p149.target_num_SET((char)218) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_0.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_u8_GET(),  new char[] {(char)162, (char)142, (char)87, (char)15}));
            assert(pack.v1_GET() == (char)5);
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {1073988492L, 1726089072L, 1046515597L, 890869337L}));
            assert(Arrays.equals(pack.ar_u16_GET(),  new char[] {(char)11315, (char)46383, (char)38790, (char)51647}));
            assert(Arrays.equals(pack.ar_i8_GET(),  new byte[] {(byte) - 69, (byte) - 104, (byte)73, (byte) - 103}));
        });
        GroundControl.ARRAY_TEST_0 p150 = CommunicationChannel.new_ARRAY_TEST_0();
        PH.setPack(p150);
        p150.ar_u8_SET(new char[] {(char)162, (char)142, (char)87, (char)15}, 0) ;
        p150.ar_u16_SET(new char[] {(char)11315, (char)46383, (char)38790, (char)51647}, 0) ;
        p150.ar_u32_SET(new long[] {1073988492L, 1726089072L, 1046515597L, 890869337L}, 0) ;
        p150.v1_SET((char)5) ;
        p150.ar_i8_SET(new byte[] {(byte) - 69, (byte) - 104, (byte)73, (byte) - 103}, 0) ;
        CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_1.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {4163168488L, 495114155L, 3612206683L, 1131366079L}));
        });
        GroundControl.ARRAY_TEST_1 p151 = CommunicationChannel.new_ARRAY_TEST_1();
        PH.setPack(p151);
        p151.ar_u32_SET(new long[] {4163168488L, 495114155L, 3612206683L, 1131366079L}, 0) ;
        CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_3.add((src, ph, pack) ->
        {
            assert(pack.v_GET() == (char)51);
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {2050893360L, 2029640274L, 1023244907L, 519719296L}));
        });
        GroundControl.ARRAY_TEST_3 p153 = CommunicationChannel.new_ARRAY_TEST_3();
        PH.setPack(p153);
        p153.ar_u32_SET(new long[] {2050893360L, 2029640274L, 1023244907L, 519719296L}, 0) ;
        p153.v_SET((char)51) ;
        CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_4.add((src, ph, pack) ->
        {
            assert(pack.v_GET() == (char)176);
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {1597807905L, 4024629322L, 2440739204L, 3139311417L}));
        });
        GroundControl.ARRAY_TEST_4 p154 = CommunicationChannel.new_ARRAY_TEST_4();
        PH.setPack(p154);
        p154.v_SET((char)176) ;
        p154.ar_u32_SET(new long[] {1597807905L, 4024629322L, 2440739204L, 3139311417L}, 0) ;
        CommunicationChannel.instance.send(p154);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_5.add((src, ph, pack) ->
        {
            assert(pack.c2_LEN(ph) == 4);
            assert(pack.c2_TRY(ph).equals("lkdb"));
            assert(pack.c1_LEN(ph) == 5);
            assert(pack.c1_TRY(ph).equals("fkvuw"));
        });
        GroundControl.ARRAY_TEST_5 p155 = CommunicationChannel.new_ARRAY_TEST_5();
        PH.setPack(p155);
        p155.c2_SET("lkdb", PH) ;
        p155.c1_SET("fkvuw", PH) ;
        CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_6.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_u8_GET(),  new char[] {(char)242, (char)95}));
            assert(Arrays.equals(pack.ar_u16_GET(),  new char[] {(char)64999, (char)7875}));
            assert(pack.v1_GET() == (char)51);
            assert(pack.v3_GET() == 650854575L);
            assert(pack.v2_GET() == (char)63255);
            assert(pack.ar_c_LEN(ph) == 25);
            assert(pack.ar_c_TRY(ph).equals("beYfsZQPyyxrcjokzuzjswuzx"));
            assert(Arrays.equals(pack.ar_i16_GET(),  new short[] {(short)161, (short)25235}));
            assert(Arrays.equals(pack.ar_d_GET(),  new double[] {1.49040086668173E308, 1.3873586287594082E308}));
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {2713010343L, 786629628L}));
            assert(Arrays.equals(pack.ar_f_GET(),  new float[] {-2.5283356E38F, 1.617296E38F}));
            assert(Arrays.equals(pack.ar_i8_GET(),  new byte[] {(byte) - 36, (byte) - 69}));
            assert(Arrays.equals(pack.ar_i32_GET(),  new int[] {511745526, 2037800868}));
        });
        GroundControl.ARRAY_TEST_6 p156 = CommunicationChannel.new_ARRAY_TEST_6();
        PH.setPack(p156);
        p156.ar_i32_SET(new int[] {511745526, 2037800868}, 0) ;
        p156.ar_d_SET(new double[] {1.49040086668173E308, 1.3873586287594082E308}, 0) ;
        p156.ar_i16_SET(new short[] {(short)161, (short)25235}, 0) ;
        p156.ar_f_SET(new float[] {-2.5283356E38F, 1.617296E38F}, 0) ;
        p156.v2_SET((char)63255) ;
        p156.ar_c_SET("beYfsZQPyyxrcjokzuzjswuzx", PH) ;
        p156.v3_SET(650854575L) ;
        p156.ar_u8_SET(new char[] {(char)242, (char)95}, 0) ;
        p156.v1_SET((char)51) ;
        p156.ar_u32_SET(new long[] {2713010343L, 786629628L}, 0) ;
        p156.ar_i8_SET(new byte[] {(byte) - 36, (byte) - 69}, 0) ;
        p156.ar_u16_SET(new char[] {(char)64999, (char)7875}, 0) ;
        CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_7.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_f_GET(),  new float[] {4.5186263E37F, -1.0667236E38F}));
            assert(Arrays.equals(pack.ar_u16_GET(),  new char[] {(char)12173, (char)60566}));
            assert(Arrays.equals(pack.ar_u8_GET(),  new char[] {(char)141, (char)46}));
            assert(Arrays.equals(pack.ar_d_GET(),  new double[] {-4.4704376637110685E307, -1.5154872810026337E308}));
            assert(Arrays.equals(pack.ar_i8_GET(),  new byte[] {(byte) - 2, (byte) - 109}));
            assert(pack.ar_c_LEN(ph) == 28);
            assert(pack.ar_c_TRY(ph).equals("mxetxpsyZVunvkxqifwescgehknc"));
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {1986499080L, 2850840695L}));
            assert(Arrays.equals(pack.ar_i32_GET(),  new int[] {-1722508217, -1693867248}));
            assert(Arrays.equals(pack.ar_i16_GET(),  new short[] {(short) -29568, (short)9891}));
        });
        GroundControl.ARRAY_TEST_7 p157 = CommunicationChannel.new_ARRAY_TEST_7();
        PH.setPack(p157);
        p157.ar_i32_SET(new int[] {-1722508217, -1693867248}, 0) ;
        p157.ar_i16_SET(new short[] {(short) -29568, (short)9891}, 0) ;
        p157.ar_d_SET(new double[] {-4.4704376637110685E307, -1.5154872810026337E308}, 0) ;
        p157.ar_u16_SET(new char[] {(char)12173, (char)60566}, 0) ;
        p157.ar_f_SET(new float[] {4.5186263E37F, -1.0667236E38F}, 0) ;
        p157.ar_i8_SET(new byte[] {(byte) - 2, (byte) - 109}, 0) ;
        p157.ar_c_SET("mxetxpsyZVunvkxqifwescgehknc", PH) ;
        p157.ar_u8_SET(new char[] {(char)141, (char)46}, 0) ;
        p157.ar_u32_SET(new long[] {1986499080L, 2850840695L}, 0) ;
        CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_8.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_u16_GET(),  new char[] {(char)48397, (char)3942}));
            assert(pack.v3_GET() == 1416852425L);
            assert(Arrays.equals(pack.ar_d_GET(),  new double[] {8.746801646897157E307, -1.6413948726650925E308}));
        });
        GroundControl.ARRAY_TEST_8 p158 = CommunicationChannel.new_ARRAY_TEST_8();
        PH.setPack(p158);
        p158.v3_SET(1416852425L) ;
        p158.ar_u16_SET(new char[] {(char)48397, (char)3942}, 0) ;
        p158.ar_d_SET(new double[] {8.746801646897157E307, -1.6413948726650925E308}, 0) ;
        CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.vel_ratio_GET() == -2.7186085E38F);
            assert(pack.pos_horiz_accuracy_GET() == -1.4170544E38F);
            assert(pack.hagl_ratio_GET() == 2.1519821E38F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE));
            assert(pack.time_usec_GET() == 7843518712414581797L);
            assert(pack.pos_horiz_ratio_GET() == 2.9338828E38F);
            assert(pack.mag_ratio_GET() == -2.3385906E38F);
            assert(pack.pos_vert_accuracy_GET() == 1.1323175E38F);
            assert(pack.pos_vert_ratio_GET() == -2.8214639E38F);
            assert(pack.tas_ratio_GET() == 1.7931228E38F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.pos_vert_accuracy_SET(1.1323175E38F) ;
        p230.mag_ratio_SET(-2.3385906E38F) ;
        p230.vel_ratio_SET(-2.7186085E38F) ;
        p230.pos_horiz_ratio_SET(2.9338828E38F) ;
        p230.hagl_ratio_SET(2.1519821E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE)) ;
        p230.pos_horiz_accuracy_SET(-1.4170544E38F) ;
        p230.time_usec_SET(7843518712414581797L) ;
        p230.tas_ratio_SET(1.7931228E38F) ;
        p230.pos_vert_ratio_SET(-2.8214639E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_alt_GET() == 3.3303698E38F);
            assert(pack.vert_accuracy_GET() == -1.451188E37F);
            assert(pack.wind_z_GET() == -2.193842E37F);
            assert(pack.var_vert_GET() == -6.970674E37F);
            assert(pack.wind_x_GET() == -3.1101643E38F);
            assert(pack.var_horiz_GET() == 3.1624023E38F);
            assert(pack.time_usec_GET() == 6845877266863006425L);
            assert(pack.wind_y_GET() == 3.3127886E38F);
            assert(pack.horiz_accuracy_GET() == -7.78106E36F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_x_SET(-3.1101643E38F) ;
        p231.wind_alt_SET(3.3303698E38F) ;
        p231.time_usec_SET(6845877266863006425L) ;
        p231.horiz_accuracy_SET(-7.78106E36F) ;
        p231.wind_y_SET(3.3127886E38F) ;
        p231.wind_z_SET(-2.193842E37F) ;
        p231.var_horiz_SET(3.1624023E38F) ;
        p231.vert_accuracy_SET(-1.451188E37F) ;
        p231.var_vert_SET(-6.970674E37F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.horiz_accuracy_GET() == 2.2306775E38F);
            assert(pack.vn_GET() == -8.433187E37F);
            assert(pack.speed_accuracy_GET() == -3.3151284E38F);
            assert(pack.fix_type_GET() == (char)53);
            assert(pack.lon_GET() == -2028281610);
            assert(pack.lat_GET() == 1257934541);
            assert(pack.time_week_GET() == (char)62761);
            assert(pack.time_week_ms_GET() == 3661424125L);
            assert(pack.gps_id_GET() == (char)185);
            assert(pack.vd_GET() == 2.1161755E37F);
            assert(pack.alt_GET() == 2.5642136E38F);
            assert(pack.vert_accuracy_GET() == 1.2291967E38F);
            assert(pack.vdop_GET() == -9.632796E37F);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ));
            assert(pack.hdop_GET() == 3.3529876E38F);
            assert(pack.time_usec_GET() == 3991985663662348487L);
            assert(pack.ve_GET() == -3.2186407E38F);
            assert(pack.satellites_visible_GET() == (char)190);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.lon_SET(-2028281610) ;
        p232.satellites_visible_SET((char)190) ;
        p232.lat_SET(1257934541) ;
        p232.fix_type_SET((char)53) ;
        p232.ve_SET(-3.2186407E38F) ;
        p232.time_usec_SET(3991985663662348487L) ;
        p232.vert_accuracy_SET(1.2291967E38F) ;
        p232.horiz_accuracy_SET(2.2306775E38F) ;
        p232.alt_SET(2.5642136E38F) ;
        p232.vn_SET(-8.433187E37F) ;
        p232.speed_accuracy_SET(-3.3151284E38F) ;
        p232.gps_id_SET((char)185) ;
        p232.vdop_SET(-9.632796E37F) ;
        p232.time_week_ms_SET(3661424125L) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ)) ;
        p232.hdop_SET(3.3529876E38F) ;
        p232.time_week_SET((char)62761) ;
        p232.vd_SET(2.1161755E37F) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)181, (char)37, (char)87, (char)176, (char)97, (char)71, (char)126, (char)44, (char)54, (char)208, (char)136, (char)74, (char)21, (char)226, (char)254, (char)23, (char)152, (char)176, (char)221, (char)156, (char)253, (char)233, (char)36, (char)96, (char)61, (char)202, (char)36, (char)4, (char)95, (char)2, (char)122, (char)158, (char)187, (char)47, (char)0, (char)197, (char)110, (char)9, (char)61, (char)28, (char)50, (char)42, (char)18, (char)157, (char)119, (char)54, (char)163, (char)202, (char)73, (char)80, (char)196, (char)216, (char)78, (char)90, (char)21, (char)99, (char)125, (char)52, (char)103, (char)253, (char)197, (char)89, (char)202, (char)218, (char)171, (char)41, (char)161, (char)248, (char)194, (char)126, (char)226, (char)233, (char)84, (char)221, (char)95, (char)78, (char)190, (char)9, (char)19, (char)227, (char)45, (char)21, (char)173, (char)209, (char)35, (char)77, (char)174, (char)207, (char)96, (char)47, (char)200, (char)163, (char)165, (char)240, (char)77, (char)188, (char)19, (char)224, (char)41, (char)78, (char)16, (char)239, (char)147, (char)103, (char)199, (char)170, (char)75, (char)238, (char)151, (char)113, (char)108, (char)209, (char)237, (char)109, (char)82, (char)46, (char)57, (char)104, (char)161, (char)105, (char)152, (char)128, (char)25, (char)88, (char)210, (char)91, (char)183, (char)90, (char)118, (char)168, (char)235, (char)13, (char)189, (char)111, (char)198, (char)75, (char)243, (char)243, (char)220, (char)147, (char)15, (char)60, (char)130, (char)56, (char)176, (char)214, (char)32, (char)53, (char)151, (char)113, (char)223, (char)29, (char)115, (char)135, (char)217, (char)172, (char)167, (char)71, (char)174, (char)12, (char)110, (char)42, (char)188, (char)146, (char)92, (char)22, (char)177, (char)229, (char)23, (char)132, (char)195, (char)211, (char)55, (char)8, (char)86, (char)142, (char)143, (char)229, (char)227, (char)222}));
            assert(pack.len_GET() == (char)79);
            assert(pack.flags_GET() == (char)109);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.data__SET(new char[] {(char)181, (char)37, (char)87, (char)176, (char)97, (char)71, (char)126, (char)44, (char)54, (char)208, (char)136, (char)74, (char)21, (char)226, (char)254, (char)23, (char)152, (char)176, (char)221, (char)156, (char)253, (char)233, (char)36, (char)96, (char)61, (char)202, (char)36, (char)4, (char)95, (char)2, (char)122, (char)158, (char)187, (char)47, (char)0, (char)197, (char)110, (char)9, (char)61, (char)28, (char)50, (char)42, (char)18, (char)157, (char)119, (char)54, (char)163, (char)202, (char)73, (char)80, (char)196, (char)216, (char)78, (char)90, (char)21, (char)99, (char)125, (char)52, (char)103, (char)253, (char)197, (char)89, (char)202, (char)218, (char)171, (char)41, (char)161, (char)248, (char)194, (char)126, (char)226, (char)233, (char)84, (char)221, (char)95, (char)78, (char)190, (char)9, (char)19, (char)227, (char)45, (char)21, (char)173, (char)209, (char)35, (char)77, (char)174, (char)207, (char)96, (char)47, (char)200, (char)163, (char)165, (char)240, (char)77, (char)188, (char)19, (char)224, (char)41, (char)78, (char)16, (char)239, (char)147, (char)103, (char)199, (char)170, (char)75, (char)238, (char)151, (char)113, (char)108, (char)209, (char)237, (char)109, (char)82, (char)46, (char)57, (char)104, (char)161, (char)105, (char)152, (char)128, (char)25, (char)88, (char)210, (char)91, (char)183, (char)90, (char)118, (char)168, (char)235, (char)13, (char)189, (char)111, (char)198, (char)75, (char)243, (char)243, (char)220, (char)147, (char)15, (char)60, (char)130, (char)56, (char)176, (char)214, (char)32, (char)53, (char)151, (char)113, (char)223, (char)29, (char)115, (char)135, (char)217, (char)172, (char)167, (char)71, (char)174, (char)12, (char)110, (char)42, (char)188, (char)146, (char)92, (char)22, (char)177, (char)229, (char)23, (char)132, (char)195, (char)211, (char)55, (char)8, (char)86, (char)142, (char)143, (char)229, (char)227, (char)222}, 0) ;
        p233.len_SET((char)79) ;
        p233.flags_SET((char)109) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.wp_num_GET() == (char)250);
            assert(pack.latitude_GET() == -2126637406);
            assert(pack.airspeed_GET() == (char)245);
            assert(pack.groundspeed_GET() == (char)24);
            assert(pack.wp_distance_GET() == (char)56804);
            assert(pack.throttle_GET() == (byte)40);
            assert(pack.temperature_GET() == (byte)64);
            assert(pack.airspeed_sp_GET() == (char)10);
            assert(pack.heading_sp_GET() == (short)16150);
            assert(pack.heading_GET() == (char)38675);
            assert(pack.custom_mode_GET() == 2652903654L);
            assert(pack.gps_nsat_GET() == (char)200);
            assert(pack.altitude_amsl_GET() == (short)14575);
            assert(pack.failsafe_GET() == (char)223);
            assert(pack.longitude_GET() == -667345182);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
            assert(pack.temperature_air_GET() == (byte) - 127);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.roll_GET() == (short) -9105);
            assert(pack.climb_rate_GET() == (byte)36);
            assert(pack.battery_remaining_GET() == (char)190);
            assert(pack.altitude_sp_GET() == (short) -26990);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
            assert(pack.pitch_GET() == (short)30889);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS) ;
        p234.temperature_air_SET((byte) - 127) ;
        p234.battery_remaining_SET((char)190) ;
        p234.altitude_amsl_SET((short)14575) ;
        p234.temperature_SET((byte)64) ;
        p234.longitude_SET(-667345182) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED)) ;
        p234.throttle_SET((byte)40) ;
        p234.latitude_SET(-2126637406) ;
        p234.heading_SET((char)38675) ;
        p234.altitude_sp_SET((short) -26990) ;
        p234.pitch_SET((short)30889) ;
        p234.groundspeed_SET((char)24) ;
        p234.custom_mode_SET(2652903654L) ;
        p234.wp_distance_SET((char)56804) ;
        p234.gps_nsat_SET((char)200) ;
        p234.roll_SET((short) -9105) ;
        p234.airspeed_sp_SET((char)10) ;
        p234.wp_num_SET((char)250) ;
        p234.climb_rate_SET((byte)36) ;
        p234.airspeed_SET((char)245) ;
        p234.heading_sp_SET((short)16150) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        p234.failsafe_SET((char)223) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4864720117827791372L);
            assert(pack.clipping_0_GET() == 2579465468L);
            assert(pack.clipping_2_GET() == 1319875606L);
            assert(pack.clipping_1_GET() == 1394908367L);
            assert(pack.vibration_x_GET() == 2.5504084E38F);
            assert(pack.vibration_y_GET() == -1.9974135E38F);
            assert(pack.vibration_z_GET() == -1.7652763E38F);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.vibration_y_SET(-1.9974135E38F) ;
        p241.vibration_z_SET(-1.7652763E38F) ;
        p241.clipping_1_SET(1394908367L) ;
        p241.clipping_2_SET(1319875606L) ;
        p241.vibration_x_SET(2.5504084E38F) ;
        p241.time_usec_SET(4864720117827791372L) ;
        p241.clipping_0_SET(2579465468L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_y_GET() == -1.1252299E38F);
            assert(pack.approach_x_GET() == 1.8988126E38F);
            assert(pack.y_GET() == -8.948308E37F);
            assert(pack.time_usec_TRY(ph) == 2841436847872265809L);
            assert(pack.x_GET() == -2.2908316E38F);
            assert(pack.longitude_GET() == 1517527475);
            assert(pack.approach_z_GET() == 1.5389894E38F);
            assert(pack.z_GET() == -2.445175E37F);
            assert(pack.latitude_GET() == 1754942184);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.9395167E37F, 2.958826E38F, -2.701692E38F, 1.8208407E38F}));
            assert(pack.altitude_GET() == 1675498822);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(1754942184) ;
        p242.y_SET(-8.948308E37F) ;
        p242.approach_y_SET(-1.1252299E38F) ;
        p242.approach_z_SET(1.5389894E38F) ;
        p242.q_SET(new float[] {2.9395167E37F, 2.958826E38F, -2.701692E38F, 1.8208407E38F}, 0) ;
        p242.x_SET(-2.2908316E38F) ;
        p242.z_SET(-2.445175E37F) ;
        p242.time_usec_SET(2841436847872265809L, PH) ;
        p242.longitude_SET(1517527475) ;
        p242.altitude_SET(1675498822) ;
        p242.approach_x_SET(1.8988126E38F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 260208012);
            assert(pack.approach_z_GET() == 4.920661E37F);
            assert(pack.x_GET() == 3.017624E38F);
            assert(pack.target_system_GET() == (char)143);
            assert(pack.longitude_GET() == -296540086);
            assert(pack.z_GET() == -3.0635866E38F);
            assert(pack.approach_x_GET() == 4.4457024E37F);
            assert(pack.latitude_GET() == -1156700915);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.7290022E38F, 8.230715E37F, -1.0272017E38F, 1.78494E38F}));
            assert(pack.y_GET() == -3.061944E38F);
            assert(pack.approach_y_GET() == 2.5191692E38F);
            assert(pack.time_usec_TRY(ph) == 9176515746388012126L);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.time_usec_SET(9176515746388012126L, PH) ;
        p243.altitude_SET(260208012) ;
        p243.q_SET(new float[] {1.7290022E38F, 8.230715E37F, -1.0272017E38F, 1.78494E38F}, 0) ;
        p243.approach_y_SET(2.5191692E38F) ;
        p243.x_SET(3.017624E38F) ;
        p243.approach_x_SET(4.4457024E37F) ;
        p243.approach_z_SET(4.920661E37F) ;
        p243.latitude_SET(-1156700915) ;
        p243.target_system_SET((char)143) ;
        p243.z_SET(-3.0635866E38F) ;
        p243.y_SET(-3.061944E38F) ;
        p243.longitude_SET(-296540086) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)60317);
            assert(pack.interval_us_GET() == -869766481);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-869766481) ;
        p244.message_id_SET((char)60317) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF) ;
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_FW) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.callsign_LEN(ph) == 8);
            assert(pack.callsign_TRY(ph).equals("yvunfiqp"));
            assert(pack.heading_GET() == (char)51800);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.lat_GET() == -185799022);
            assert(pack.altitude_GET() == -8572066);
            assert(pack.hor_velocity_GET() == (char)28761);
            assert(pack.ICAO_address_GET() == 741601708L);
            assert(pack.squawk_GET() == (char)10715);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_POINT_OBSTACLE);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK));
            assert(pack.ver_velocity_GET() == (short) -9684);
            assert(pack.tslc_GET() == (char)177);
            assert(pack.lon_GET() == 1313463813);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.hor_velocity_SET((char)28761) ;
        p246.callsign_SET("yvunfiqp", PH) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK)) ;
        p246.ICAO_address_SET(741601708L) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.lat_SET(-185799022) ;
        p246.ver_velocity_SET((short) -9684) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_POINT_OBSTACLE) ;
        p246.tslc_SET((char)177) ;
        p246.squawk_SET((char)10715) ;
        p246.lon_SET(1313463813) ;
        p246.heading_SET((char)51800) ;
        p246.altitude_SET(-8572066) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
            assert(pack.id_GET() == 385966297L);
            assert(pack.altitude_minimum_delta_GET() == -3.1415787E38F);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
            assert(pack.horizontal_minimum_delta_GET() == 6.894172E37F);
            assert(pack.time_to_minimum_delta_GET() == -1.4300728E38F);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.time_to_minimum_delta_SET(-1.4300728E38F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER) ;
        p247.id_SET(385966297L) ;
        p247.horizontal_minimum_delta_SET(6.894172E37F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.altitude_minimum_delta_SET(-3.1415787E38F) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)138);
            assert(pack.target_component_GET() == (char)52);
            assert(pack.target_network_GET() == (char)99);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)238, (char)125, (char)4, (char)249, (char)70, (char)26, (char)116, (char)110, (char)86, (char)134, (char)173, (char)19, (char)31, (char)13, (char)60, (char)245, (char)108, (char)33, (char)87, (char)114, (char)212, (char)65, (char)208, (char)153, (char)152, (char)93, (char)240, (char)15, (char)185, (char)210, (char)29, (char)146, (char)200, (char)37, (char)255, (char)197, (char)130, (char)97, (char)161, (char)235, (char)154, (char)144, (char)11, (char)60, (char)71, (char)79, (char)165, (char)0, (char)187, (char)54, (char)234, (char)134, (char)158, (char)160, (char)148, (char)77, (char)31, (char)54, (char)132, (char)199, (char)120, (char)109, (char)77, (char)144, (char)13, (char)225, (char)109, (char)78, (char)47, (char)211, (char)177, (char)160, (char)102, (char)6, (char)217, (char)57, (char)51, (char)43, (char)10, (char)134, (char)74, (char)188, (char)233, (char)79, (char)126, (char)76, (char)123, (char)148, (char)30, (char)59, (char)247, (char)173, (char)212, (char)201, (char)187, (char)131, (char)106, (char)160, (char)125, (char)82, (char)153, (char)94, (char)7, (char)36, (char)0, (char)165, (char)201, (char)100, (char)124, (char)74, (char)16, (char)100, (char)17, (char)132, (char)60, (char)61, (char)248, (char)238, (char)162, (char)33, (char)87, (char)91, (char)109, (char)184, (char)176, (char)212, (char)103, (char)124, (char)196, (char)223, (char)56, (char)23, (char)5, (char)116, (char)23, (char)219, (char)182, (char)47, (char)4, (char)53, (char)239, (char)39, (char)10, (char)179, (char)18, (char)183, (char)47, (char)107, (char)229, (char)232, (char)196, (char)154, (char)40, (char)165, (char)13, (char)15, (char)217, (char)58, (char)75, (char)117, (char)63, (char)49, (char)209, (char)169, (char)41, (char)102, (char)159, (char)88, (char)232, (char)167, (char)185, (char)49, (char)74, (char)81, (char)190, (char)54, (char)196, (char)74, (char)48, (char)114, (char)245, (char)55, (char)156, (char)249, (char)99, (char)133, (char)85, (char)53, (char)192, (char)160, (char)103, (char)48, (char)159, (char)247, (char)152, (char)88, (char)177, (char)206, (char)155, (char)116, (char)181, (char)220, (char)141, (char)192, (char)102, (char)151, (char)179, (char)83, (char)61, (char)175, (char)250, (char)113, (char)147, (char)50, (char)215, (char)250, (char)55, (char)221, (char)149, (char)89, (char)239, (char)141, (char)90, (char)101, (char)217, (char)62, (char)105, (char)131, (char)174, (char)176, (char)114, (char)163, (char)58, (char)84, (char)59, (char)238, (char)170, (char)170, (char)21, (char)197, (char)59, (char)76, (char)60, (char)215, (char)111, (char)191, (char)56, (char)167, (char)87}));
            assert(pack.message_type_GET() == (char)60225);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.payload_SET(new char[] {(char)238, (char)125, (char)4, (char)249, (char)70, (char)26, (char)116, (char)110, (char)86, (char)134, (char)173, (char)19, (char)31, (char)13, (char)60, (char)245, (char)108, (char)33, (char)87, (char)114, (char)212, (char)65, (char)208, (char)153, (char)152, (char)93, (char)240, (char)15, (char)185, (char)210, (char)29, (char)146, (char)200, (char)37, (char)255, (char)197, (char)130, (char)97, (char)161, (char)235, (char)154, (char)144, (char)11, (char)60, (char)71, (char)79, (char)165, (char)0, (char)187, (char)54, (char)234, (char)134, (char)158, (char)160, (char)148, (char)77, (char)31, (char)54, (char)132, (char)199, (char)120, (char)109, (char)77, (char)144, (char)13, (char)225, (char)109, (char)78, (char)47, (char)211, (char)177, (char)160, (char)102, (char)6, (char)217, (char)57, (char)51, (char)43, (char)10, (char)134, (char)74, (char)188, (char)233, (char)79, (char)126, (char)76, (char)123, (char)148, (char)30, (char)59, (char)247, (char)173, (char)212, (char)201, (char)187, (char)131, (char)106, (char)160, (char)125, (char)82, (char)153, (char)94, (char)7, (char)36, (char)0, (char)165, (char)201, (char)100, (char)124, (char)74, (char)16, (char)100, (char)17, (char)132, (char)60, (char)61, (char)248, (char)238, (char)162, (char)33, (char)87, (char)91, (char)109, (char)184, (char)176, (char)212, (char)103, (char)124, (char)196, (char)223, (char)56, (char)23, (char)5, (char)116, (char)23, (char)219, (char)182, (char)47, (char)4, (char)53, (char)239, (char)39, (char)10, (char)179, (char)18, (char)183, (char)47, (char)107, (char)229, (char)232, (char)196, (char)154, (char)40, (char)165, (char)13, (char)15, (char)217, (char)58, (char)75, (char)117, (char)63, (char)49, (char)209, (char)169, (char)41, (char)102, (char)159, (char)88, (char)232, (char)167, (char)185, (char)49, (char)74, (char)81, (char)190, (char)54, (char)196, (char)74, (char)48, (char)114, (char)245, (char)55, (char)156, (char)249, (char)99, (char)133, (char)85, (char)53, (char)192, (char)160, (char)103, (char)48, (char)159, (char)247, (char)152, (char)88, (char)177, (char)206, (char)155, (char)116, (char)181, (char)220, (char)141, (char)192, (char)102, (char)151, (char)179, (char)83, (char)61, (char)175, (char)250, (char)113, (char)147, (char)50, (char)215, (char)250, (char)55, (char)221, (char)149, (char)89, (char)239, (char)141, (char)90, (char)101, (char)217, (char)62, (char)105, (char)131, (char)174, (char)176, (char)114, (char)163, (char)58, (char)84, (char)59, (char)238, (char)170, (char)170, (char)21, (char)197, (char)59, (char)76, (char)60, (char)215, (char)111, (char)191, (char)56, (char)167, (char)87}, 0) ;
        p248.target_system_SET((char)138) ;
        p248.message_type_SET((char)60225) ;
        p248.target_network_SET((char)99) ;
        p248.target_component_SET((char)52) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.ver_GET() == (char)41);
            assert(pack.address_GET() == (char)42440);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 119, (byte)36, (byte)14, (byte)90, (byte)69, (byte)23, (byte) - 2, (byte)125, (byte) - 127, (byte) - 73, (byte)78, (byte) - 96, (byte)74, (byte)47, (byte) - 64, (byte) - 92, (byte) - 116, (byte) - 115, (byte) - 111, (byte)107, (byte) - 64, (byte) - 1, (byte) - 27, (byte)52, (byte)6, (byte) - 40, (byte) - 123, (byte)11, (byte)79, (byte) - 109, (byte) - 66, (byte) - 110}));
            assert(pack.type_GET() == (char)147);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)42440) ;
        p249.ver_SET((char)41) ;
        p249.type_SET((char)147) ;
        p249.value_SET(new byte[] {(byte) - 119, (byte)36, (byte)14, (byte)90, (byte)69, (byte)23, (byte) - 2, (byte)125, (byte) - 127, (byte) - 73, (byte)78, (byte) - 96, (byte)74, (byte)47, (byte) - 64, (byte) - 92, (byte) - 116, (byte) - 115, (byte) - 111, (byte)107, (byte) - 64, (byte) - 1, (byte) - 27, (byte)52, (byte)6, (byte) - 40, (byte) - 123, (byte)11, (byte)79, (byte) - 109, (byte) - 66, (byte) - 110}, 0) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -3.113356E38F);
            assert(pack.z_GET() == -2.6909354E38F);
            assert(pack.time_usec_GET() == 4688837261244043391L);
            assert(pack.y_GET() == -2.0765557E38F);
            assert(pack.name_LEN(ph) == 2);
            assert(pack.name_TRY(ph).equals("cj"));
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.x_SET(-3.113356E38F) ;
        p250.time_usec_SET(4688837261244043391L) ;
        p250.y_SET(-2.0765557E38F) ;
        p250.z_SET(-2.6909354E38F) ;
        p250.name_SET("cj", PH) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 485633280L);
            assert(pack.value_GET() == 2.62826E38F);
            assert(pack.name_LEN(ph) == 1);
            assert(pack.name_TRY(ph).equals("b"));
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(485633280L) ;
        p251.name_SET("b", PH) ;
        p251.value_SET(2.62826E38F) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -564163310);
            assert(pack.time_boot_ms_GET() == 1854148086L);
            assert(pack.name_LEN(ph) == 6);
            assert(pack.name_TRY(ph).equals("oTrfbv"));
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.name_SET("oTrfbv", PH) ;
        p252.time_boot_ms_SET(1854148086L) ;
        p252.value_SET(-564163310) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
            assert(pack.text_LEN(ph) == 26);
            assert(pack.text_TRY(ph).equals("llgwshzjairgewtwzYlrkbafhk"));
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_EMERGENCY) ;
        p253.text_SET("llgwshzjairgewtwzYlrkbafhk", PH) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3872386787L);
            assert(pack.ind_GET() == (char)243);
            assert(pack.value_GET() == -1.0856189E38F);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.value_SET(-1.0856189E38F) ;
        p254.time_boot_ms_SET(3872386787L) ;
        p254.ind_SET((char)243) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)228);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)166, (char)219, (char)250, (char)195, (char)192, (char)236, (char)22, (char)244, (char)182, (char)227, (char)66, (char)130, (char)236, (char)218, (char)122, (char)184, (char)162, (char)188, (char)50, (char)76, (char)182, (char)183, (char)14, (char)110, (char)236, (char)123, (char)193, (char)92, (char)165, (char)79, (char)218, (char)85}));
            assert(pack.target_component_GET() == (char)247);
            assert(pack.initial_timestamp_GET() == 7372397887586776467L);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_component_SET((char)247) ;
        p256.target_system_SET((char)228) ;
        p256.secret_key_SET(new char[] {(char)166, (char)219, (char)250, (char)195, (char)192, (char)236, (char)22, (char)244, (char)182, (char)227, (char)66, (char)130, (char)236, (char)218, (char)122, (char)184, (char)162, (char)188, (char)50, (char)76, (char)182, (char)183, (char)14, (char)110, (char)236, (char)123, (char)193, (char)92, (char)165, (char)79, (char)218, (char)85}, 0) ;
        p256.initial_timestamp_SET(7372397887586776467L) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.last_change_ms_GET() == 1195206796L);
            assert(pack.state_GET() == (char)213);
            assert(pack.time_boot_ms_GET() == 1019519439L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.last_change_ms_SET(1195206796L) ;
        p257.time_boot_ms_SET(1019519439L) ;
        p257.state_SET((char)213) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.tune_LEN(ph) == 25);
            assert(pack.tune_TRY(ph).equals("jqKkrovkhdsqakshsZGmQzvsr"));
            assert(pack.target_system_GET() == (char)183);
            assert(pack.target_component_GET() == (char)108);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_component_SET((char)108) ;
        p258.tune_SET("jqKkrovkhdsqakshsZGmQzvsr", PH) ;
        p258.target_system_SET((char)183) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)69, (char)233, (char)175, (char)37, (char)218, (char)138, (char)222, (char)185, (char)183, (char)170, (char)180, (char)154, (char)41, (char)185, (char)246, (char)198, (char)141, (char)155, (char)237, (char)41, (char)252, (char)247, (char)187, (char)138, (char)209, (char)175, (char)18, (char)92, (char)49, (char)46, (char)98, (char)198}));
            assert(pack.cam_definition_version_GET() == (char)32724);
            assert(pack.cam_definition_uri_LEN(ph) == 78);
            assert(pack.cam_definition_uri_TRY(ph).equals("vplayizicrqxqfshyIrnojAjozqpypyrnewfstoliQpbeqowtfcxihqijlwbdtqkwmwwiesCvjgqwf"));
            assert(pack.firmware_version_GET() == 3135441638L);
            assert(pack.resolution_v_GET() == (char)20463);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)151, (char)45, (char)139, (char)22, (char)15, (char)0, (char)80, (char)153, (char)61, (char)190, (char)221, (char)167, (char)190, (char)213, (char)174, (char)177, (char)90, (char)225, (char)20, (char)223, (char)206, (char)13, (char)5, (char)131, (char)166, (char)202, (char)161, (char)146, (char)127, (char)104, (char)70, (char)216}));
            assert(pack.resolution_h_GET() == (char)54433);
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
            assert(pack.focal_length_GET() == -2.6062663E38F);
            assert(pack.time_boot_ms_GET() == 461966152L);
            assert(pack.lens_id_GET() == (char)115);
            assert(pack.sensor_size_h_GET() == 1.7297107E38F);
            assert(pack.sensor_size_v_GET() == 2.5571005E38F);
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.model_name_SET(new char[] {(char)151, (char)45, (char)139, (char)22, (char)15, (char)0, (char)80, (char)153, (char)61, (char)190, (char)221, (char)167, (char)190, (char)213, (char)174, (char)177, (char)90, (char)225, (char)20, (char)223, (char)206, (char)13, (char)5, (char)131, (char)166, (char)202, (char)161, (char)146, (char)127, (char)104, (char)70, (char)216}, 0) ;
        p259.firmware_version_SET(3135441638L) ;
        p259.cam_definition_version_SET((char)32724) ;
        p259.sensor_size_h_SET(1.7297107E38F) ;
        p259.cam_definition_uri_SET("vplayizicrqxqfshyIrnojAjozqpypyrnewfstoliQpbeqowtfcxihqijlwbdtqkwmwwiesCvjgqwf", PH) ;
        p259.lens_id_SET((char)115) ;
        p259.sensor_size_v_SET(2.5571005E38F) ;
        p259.resolution_h_SET((char)54433) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE)) ;
        p259.focal_length_SET(-2.6062663E38F) ;
        p259.resolution_v_SET((char)20463) ;
        p259.vendor_name_SET(new char[] {(char)69, (char)233, (char)175, (char)37, (char)218, (char)138, (char)222, (char)185, (char)183, (char)170, (char)180, (char)154, (char)41, (char)185, (char)246, (char)198, (char)141, (char)155, (char)237, (char)41, (char)252, (char)247, (char)187, (char)138, (char)209, (char)175, (char)18, (char)92, (char)49, (char)46, (char)98, (char)198}, 0) ;
        p259.time_boot_ms_SET(461966152L) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 570662542L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(570662542L) ;
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.available_capacity_GET() == -2.9461074E38F);
            assert(pack.storage_count_GET() == (char)240);
            assert(pack.used_capacity_GET() == -2.8659199E38F);
            assert(pack.storage_id_GET() == (char)210);
            assert(pack.read_speed_GET() == 2.642097E38F);
            assert(pack.total_capacity_GET() == -3.2332057E38F);
            assert(pack.write_speed_GET() == -2.5828637E38F);
            assert(pack.time_boot_ms_GET() == 3737952377L);
            assert(pack.status_GET() == (char)163);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.used_capacity_SET(-2.8659199E38F) ;
        p261.status_SET((char)163) ;
        p261.storage_count_SET((char)240) ;
        p261.read_speed_SET(2.642097E38F) ;
        p261.total_capacity_SET(-3.2332057E38F) ;
        p261.time_boot_ms_SET(3737952377L) ;
        p261.storage_id_SET((char)210) ;
        p261.write_speed_SET(-2.5828637E38F) ;
        p261.available_capacity_SET(-2.9461074E38F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1072537086L);
            assert(pack.available_capacity_GET() == -1.1068665E38F);
            assert(pack.image_status_GET() == (char)69);
            assert(pack.video_status_GET() == (char)236);
            assert(pack.image_interval_GET() == -3.7747316E37F);
            assert(pack.recording_time_ms_GET() == 3941003756L);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(1072537086L) ;
        p262.recording_time_ms_SET(3941003756L) ;
        p262.image_status_SET((char)69) ;
        p262.video_status_SET((char)236) ;
        p262.available_capacity_SET(-1.1068665E38F) ;
        p262.image_interval_SET(-3.7747316E37F) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.image_index_GET() == 1691729252);
            assert(pack.camera_id_GET() == (char)241);
            assert(pack.alt_GET() == 1368565254);
            assert(pack.capture_result_GET() == (byte) - 70);
            assert(pack.lon_GET() == 436368359);
            assert(pack.lat_GET() == 504434115);
            assert(pack.relative_alt_GET() == -1240159961);
            assert(pack.time_utc_GET() == 7637434784169890574L);
            assert(pack.time_boot_ms_GET() == 2963172311L);
            assert(pack.file_url_LEN(ph) == 134);
            assert(pack.file_url_TRY(ph).equals("wfpxvxrhkqGwmlzhsdxerbdlAhemekaicidujbffojpanfnTauieAxhauexgwltqhtxmtftrwrszapbBacriiddytbvpjljkMxketvhuxrlvstuvcnxbmnqCqzmarbdpBpqhkz"));
            assert(Arrays.equals(pack.q_GET(),  new float[] {-4.2458043E37F, 1.6965712E38F, 1.610973E38F, -1.2653565E38F}));
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.relative_alt_SET(-1240159961) ;
        p263.file_url_SET("wfpxvxrhkqGwmlzhsdxerbdlAhemekaicidujbffojpanfnTauieAxhauexgwltqhtxmtftrwrszapbBacriiddytbvpjljkMxketvhuxrlvstuvcnxbmnqCqzmarbdpBpqhkz", PH) ;
        p263.alt_SET(1368565254) ;
        p263.lat_SET(504434115) ;
        p263.capture_result_SET((byte) - 70) ;
        p263.time_boot_ms_SET(2963172311L) ;
        p263.camera_id_SET((char)241) ;
        p263.q_SET(new float[] {-4.2458043E37F, 1.6965712E38F, 1.610973E38F, -1.2653565E38F}, 0) ;
        p263.lon_SET(436368359) ;
        p263.time_utc_SET(7637434784169890574L) ;
        p263.image_index_SET(1691729252) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.flight_uuid_GET() == 2474349765654336093L);
            assert(pack.takeoff_time_utc_GET() == 8622960552255888148L);
            assert(pack.arming_time_utc_GET() == 7864633269402451603L);
            assert(pack.time_boot_ms_GET() == 1143412462L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.takeoff_time_utc_SET(8622960552255888148L) ;
        p264.arming_time_utc_SET(7864633269402451603L) ;
        p264.time_boot_ms_SET(1143412462L) ;
        p264.flight_uuid_SET(2474349765654336093L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2056149960L);
            assert(pack.yaw_GET() == 1.2592651E38F);
            assert(pack.roll_GET() == -6.05526E37F);
            assert(pack.pitch_GET() == -5.356967E37F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.yaw_SET(1.2592651E38F) ;
        p265.pitch_SET(-5.356967E37F) ;
        p265.time_boot_ms_SET(2056149960L) ;
        p265.roll_SET(-6.05526E37F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)87);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)227, (char)220, (char)189, (char)58, (char)45, (char)254, (char)3, (char)144, (char)65, (char)37, (char)43, (char)198, (char)200, (char)138, (char)5, (char)52, (char)87, (char)94, (char)49, (char)160, (char)235, (char)162, (char)85, (char)229, (char)146, (char)63, (char)154, (char)43, (char)186, (char)214, (char)46, (char)236, (char)168, (char)23, (char)228, (char)55, (char)21, (char)231, (char)245, (char)236, (char)77, (char)170, (char)55, (char)228, (char)6, (char)25, (char)147, (char)86, (char)76, (char)29, (char)26, (char)74, (char)13, (char)163, (char)178, (char)107, (char)102, (char)69, (char)209, (char)93, (char)111, (char)41, (char)242, (char)79, (char)126, (char)13, (char)63, (char)94, (char)28, (char)223, (char)156, (char)18, (char)113, (char)34, (char)24, (char)133, (char)109, (char)70, (char)159, (char)166, (char)99, (char)63, (char)137, (char)0, (char)47, (char)4, (char)253, (char)124, (char)191, (char)67, (char)70, (char)5, (char)82, (char)87, (char)239, (char)72, (char)37, (char)138, (char)100, (char)150, (char)144, (char)199, (char)3, (char)234, (char)221, (char)62, (char)216, (char)137, (char)32, (char)243, (char)36, (char)34, (char)234, (char)192, (char)75, (char)174, (char)209, (char)136, (char)93, (char)83, (char)17, (char)192, (char)191, (char)72, (char)217, (char)151, (char)253, (char)217, (char)92, (char)126, (char)6, (char)46, (char)120, (char)242, (char)215, (char)214, (char)202, (char)157, (char)126, (char)32, (char)57, (char)17, (char)108, (char)137, (char)40, (char)110, (char)63, (char)222, (char)84, (char)43, (char)255, (char)49, (char)127, (char)219, (char)207, (char)37, (char)221, (char)177, (char)192, (char)17, (char)76, (char)133, (char)59, (char)218, (char)243, (char)49, (char)31, (char)240, (char)189, (char)49, (char)154, (char)135, (char)66, (char)41, (char)26, (char)51, (char)106, (char)40, (char)211, (char)191, (char)56, (char)31, (char)54, (char)67, (char)1, (char)46, (char)198, (char)102, (char)84, (char)167, (char)18, (char)25, (char)27, (char)48, (char)40, (char)196, (char)94, (char)251, (char)117, (char)174, (char)110, (char)249, (char)68, (char)123, (char)170, (char)195, (char)94, (char)179, (char)115, (char)0, (char)245, (char)127, (char)249, (char)6, (char)171, (char)3, (char)204, (char)102, (char)107, (char)16, (char)21, (char)65, (char)200, (char)156, (char)27, (char)33, (char)179, (char)237, (char)87, (char)113, (char)175, (char)161, (char)192, (char)74, (char)57, (char)98, (char)207, (char)77, (char)187, (char)9, (char)20, (char)138, (char)21, (char)228, (char)183, (char)185, (char)76, (char)87, (char)124}));
            assert(pack.first_message_offset_GET() == (char)207);
            assert(pack.target_system_GET() == (char)7);
            assert(pack.length_GET() == (char)251);
            assert(pack.sequence_GET() == (char)8350);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.sequence_SET((char)8350) ;
        p266.data__SET(new char[] {(char)227, (char)220, (char)189, (char)58, (char)45, (char)254, (char)3, (char)144, (char)65, (char)37, (char)43, (char)198, (char)200, (char)138, (char)5, (char)52, (char)87, (char)94, (char)49, (char)160, (char)235, (char)162, (char)85, (char)229, (char)146, (char)63, (char)154, (char)43, (char)186, (char)214, (char)46, (char)236, (char)168, (char)23, (char)228, (char)55, (char)21, (char)231, (char)245, (char)236, (char)77, (char)170, (char)55, (char)228, (char)6, (char)25, (char)147, (char)86, (char)76, (char)29, (char)26, (char)74, (char)13, (char)163, (char)178, (char)107, (char)102, (char)69, (char)209, (char)93, (char)111, (char)41, (char)242, (char)79, (char)126, (char)13, (char)63, (char)94, (char)28, (char)223, (char)156, (char)18, (char)113, (char)34, (char)24, (char)133, (char)109, (char)70, (char)159, (char)166, (char)99, (char)63, (char)137, (char)0, (char)47, (char)4, (char)253, (char)124, (char)191, (char)67, (char)70, (char)5, (char)82, (char)87, (char)239, (char)72, (char)37, (char)138, (char)100, (char)150, (char)144, (char)199, (char)3, (char)234, (char)221, (char)62, (char)216, (char)137, (char)32, (char)243, (char)36, (char)34, (char)234, (char)192, (char)75, (char)174, (char)209, (char)136, (char)93, (char)83, (char)17, (char)192, (char)191, (char)72, (char)217, (char)151, (char)253, (char)217, (char)92, (char)126, (char)6, (char)46, (char)120, (char)242, (char)215, (char)214, (char)202, (char)157, (char)126, (char)32, (char)57, (char)17, (char)108, (char)137, (char)40, (char)110, (char)63, (char)222, (char)84, (char)43, (char)255, (char)49, (char)127, (char)219, (char)207, (char)37, (char)221, (char)177, (char)192, (char)17, (char)76, (char)133, (char)59, (char)218, (char)243, (char)49, (char)31, (char)240, (char)189, (char)49, (char)154, (char)135, (char)66, (char)41, (char)26, (char)51, (char)106, (char)40, (char)211, (char)191, (char)56, (char)31, (char)54, (char)67, (char)1, (char)46, (char)198, (char)102, (char)84, (char)167, (char)18, (char)25, (char)27, (char)48, (char)40, (char)196, (char)94, (char)251, (char)117, (char)174, (char)110, (char)249, (char)68, (char)123, (char)170, (char)195, (char)94, (char)179, (char)115, (char)0, (char)245, (char)127, (char)249, (char)6, (char)171, (char)3, (char)204, (char)102, (char)107, (char)16, (char)21, (char)65, (char)200, (char)156, (char)27, (char)33, (char)179, (char)237, (char)87, (char)113, (char)175, (char)161, (char)192, (char)74, (char)57, (char)98, (char)207, (char)77, (char)187, (char)9, (char)20, (char)138, (char)21, (char)228, (char)183, (char)185, (char)76, (char)87, (char)124}, 0) ;
        p266.target_component_SET((char)87) ;
        p266.target_system_SET((char)7) ;
        p266.length_SET((char)251) ;
        p266.first_message_offset_SET((char)207) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)58957);
            assert(pack.first_message_offset_GET() == (char)156);
            assert(pack.target_component_GET() == (char)41);
            assert(pack.length_GET() == (char)209);
            assert(pack.target_system_GET() == (char)186);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)200, (char)139, (char)81, (char)85, (char)112, (char)227, (char)42, (char)74, (char)112, (char)234, (char)67, (char)148, (char)134, (char)15, (char)227, (char)115, (char)172, (char)110, (char)189, (char)0, (char)52, (char)50, (char)189, (char)25, (char)18, (char)94, (char)215, (char)250, (char)225, (char)221, (char)4, (char)248, (char)216, (char)118, (char)182, (char)243, (char)167, (char)166, (char)242, (char)45, (char)48, (char)80, (char)109, (char)132, (char)236, (char)107, (char)67, (char)57, (char)45, (char)9, (char)141, (char)95, (char)254, (char)4, (char)206, (char)113, (char)101, (char)138, (char)20, (char)35, (char)252, (char)66, (char)176, (char)75, (char)6, (char)242, (char)158, (char)59, (char)73, (char)56, (char)21, (char)225, (char)108, (char)72, (char)45, (char)29, (char)31, (char)196, (char)100, (char)34, (char)91, (char)203, (char)253, (char)104, (char)106, (char)228, (char)18, (char)107, (char)199, (char)172, (char)4, (char)235, (char)135, (char)245, (char)185, (char)251, (char)82, (char)139, (char)121, (char)69, (char)196, (char)91, (char)23, (char)127, (char)40, (char)30, (char)156, (char)178, (char)99, (char)240, (char)43, (char)199, (char)122, (char)8, (char)138, (char)19, (char)75, (char)179, (char)101, (char)106, (char)228, (char)108, (char)130, (char)114, (char)31, (char)222, (char)83, (char)51, (char)140, (char)255, (char)233, (char)181, (char)55, (char)186, (char)33, (char)94, (char)24, (char)65, (char)239, (char)173, (char)47, (char)221, (char)254, (char)249, (char)211, (char)57, (char)252, (char)175, (char)15, (char)222, (char)117, (char)81, (char)227, (char)197, (char)158, (char)133, (char)134, (char)223, (char)3, (char)92, (char)91, (char)114, (char)169, (char)115, (char)122, (char)71, (char)252, (char)223, (char)23, (char)41, (char)243, (char)65, (char)38, (char)220, (char)133, (char)181, (char)60, (char)46, (char)144, (char)88, (char)75, (char)108, (char)230, (char)25, (char)255, (char)248, (char)116, (char)38, (char)163, (char)143, (char)120, (char)248, (char)212, (char)139, (char)230, (char)84, (char)238, (char)48, (char)200, (char)147, (char)8, (char)158, (char)216, (char)182, (char)201, (char)129, (char)172, (char)168, (char)194, (char)231, (char)214, (char)89, (char)165, (char)75, (char)125, (char)77, (char)124, (char)21, (char)54, (char)223, (char)64, (char)194, (char)68, (char)209, (char)74, (char)117, (char)192, (char)195, (char)216, (char)187, (char)210, (char)216, (char)241, (char)6, (char)245, (char)24, (char)140, (char)60, (char)101, (char)86, (char)4, (char)182, (char)20, (char)125, (char)124, (char)176, (char)60, (char)50, (char)59}));
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.data__SET(new char[] {(char)200, (char)139, (char)81, (char)85, (char)112, (char)227, (char)42, (char)74, (char)112, (char)234, (char)67, (char)148, (char)134, (char)15, (char)227, (char)115, (char)172, (char)110, (char)189, (char)0, (char)52, (char)50, (char)189, (char)25, (char)18, (char)94, (char)215, (char)250, (char)225, (char)221, (char)4, (char)248, (char)216, (char)118, (char)182, (char)243, (char)167, (char)166, (char)242, (char)45, (char)48, (char)80, (char)109, (char)132, (char)236, (char)107, (char)67, (char)57, (char)45, (char)9, (char)141, (char)95, (char)254, (char)4, (char)206, (char)113, (char)101, (char)138, (char)20, (char)35, (char)252, (char)66, (char)176, (char)75, (char)6, (char)242, (char)158, (char)59, (char)73, (char)56, (char)21, (char)225, (char)108, (char)72, (char)45, (char)29, (char)31, (char)196, (char)100, (char)34, (char)91, (char)203, (char)253, (char)104, (char)106, (char)228, (char)18, (char)107, (char)199, (char)172, (char)4, (char)235, (char)135, (char)245, (char)185, (char)251, (char)82, (char)139, (char)121, (char)69, (char)196, (char)91, (char)23, (char)127, (char)40, (char)30, (char)156, (char)178, (char)99, (char)240, (char)43, (char)199, (char)122, (char)8, (char)138, (char)19, (char)75, (char)179, (char)101, (char)106, (char)228, (char)108, (char)130, (char)114, (char)31, (char)222, (char)83, (char)51, (char)140, (char)255, (char)233, (char)181, (char)55, (char)186, (char)33, (char)94, (char)24, (char)65, (char)239, (char)173, (char)47, (char)221, (char)254, (char)249, (char)211, (char)57, (char)252, (char)175, (char)15, (char)222, (char)117, (char)81, (char)227, (char)197, (char)158, (char)133, (char)134, (char)223, (char)3, (char)92, (char)91, (char)114, (char)169, (char)115, (char)122, (char)71, (char)252, (char)223, (char)23, (char)41, (char)243, (char)65, (char)38, (char)220, (char)133, (char)181, (char)60, (char)46, (char)144, (char)88, (char)75, (char)108, (char)230, (char)25, (char)255, (char)248, (char)116, (char)38, (char)163, (char)143, (char)120, (char)248, (char)212, (char)139, (char)230, (char)84, (char)238, (char)48, (char)200, (char)147, (char)8, (char)158, (char)216, (char)182, (char)201, (char)129, (char)172, (char)168, (char)194, (char)231, (char)214, (char)89, (char)165, (char)75, (char)125, (char)77, (char)124, (char)21, (char)54, (char)223, (char)64, (char)194, (char)68, (char)209, (char)74, (char)117, (char)192, (char)195, (char)216, (char)187, (char)210, (char)216, (char)241, (char)6, (char)245, (char)24, (char)140, (char)60, (char)101, (char)86, (char)4, (char)182, (char)20, (char)125, (char)124, (char)176, (char)60, (char)50, (char)59}, 0) ;
        p267.first_message_offset_SET((char)156) ;
        p267.target_system_SET((char)186) ;
        p267.length_SET((char)209) ;
        p267.sequence_SET((char)58957) ;
        p267.target_component_SET((char)41) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)224);
            assert(pack.sequence_GET() == (char)51403);
            assert(pack.target_component_GET() == (char)176);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)224) ;
        p268.target_component_SET((char)176) ;
        p268.sequence_SET((char)51403) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.camera_id_GET() == (char)155);
            assert(pack.uri_LEN(ph) == 135);
            assert(pack.uri_TRY(ph).equals("hlrcnixnleonmejqtetsruhsOyhblsskxzqvvehvygarwqmxQbcpbtUsZXuPfbhinoQaFvzvitosLailiMbwnxjqrbvkwyiobkfsepfwxvvvahexkjtfhvlpumjptldsLvpGILs"));
            assert(pack.bitrate_GET() == 1510825370L);
            assert(pack.framerate_GET() == 2.7820728E38F);
            assert(pack.status_GET() == (char)57);
            assert(pack.rotation_GET() == (char)32050);
            assert(pack.resolution_h_GET() == (char)3452);
            assert(pack.resolution_v_GET() == (char)9196);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.uri_SET("hlrcnixnleonmejqtetsruhsOyhblsskxzqvvehvygarwqmxQbcpbtUsZXuPfbhinoQaFvzvitosLailiMbwnxjqrbvkwyiobkfsepfwxvvvahexkjtfhvlpumjptldsLvpGILs", PH) ;
        p269.resolution_h_SET((char)3452) ;
        p269.framerate_SET(2.7820728E38F) ;
        p269.bitrate_SET(1510825370L) ;
        p269.resolution_v_SET((char)9196) ;
        p269.status_SET((char)57) ;
        p269.camera_id_SET((char)155) ;
        p269.rotation_SET((char)32050) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)102);
            assert(pack.bitrate_GET() == 3656087209L);
            assert(pack.target_component_GET() == (char)200);
            assert(pack.camera_id_GET() == (char)200);
            assert(pack.framerate_GET() == -6.401143E37F);
            assert(pack.resolution_h_GET() == (char)52791);
            assert(pack.rotation_GET() == (char)14597);
            assert(pack.resolution_v_GET() == (char)26964);
            assert(pack.uri_LEN(ph) == 70);
            assert(pack.uri_TRY(ph).equals("lfoidattkpwikizdxncymClgohlfDGmukdtevwzfmEwbmeSYkqsyCkycelcilpbhjdipBp"));
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.rotation_SET((char)14597) ;
        p270.framerate_SET(-6.401143E37F) ;
        p270.camera_id_SET((char)200) ;
        p270.resolution_h_SET((char)52791) ;
        p270.bitrate_SET(3656087209L) ;
        p270.resolution_v_SET((char)26964) ;
        p270.uri_SET("lfoidattkpwikizdxncymClgohlfDGmukdtevwzfmEwbmeSYkqsyCkycelcilpbhjdipBp", PH) ;
        p270.target_system_SET((char)102) ;
        p270.target_component_SET((char)200) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 36);
            assert(pack.password_TRY(ph).equals("mytmLwcnzlgzdjuNnxtxgPzodHkoylkruiNn"));
            assert(pack.ssid_LEN(ph) == 12);
            assert(pack.ssid_TRY(ph).equals("IwfdxJwWaufy"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("mytmLwcnzlgzdjuNnxtxgPzodHkoylkruiNn", PH) ;
        p299.ssid_SET("IwfdxJwWaufy", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.max_version_GET() == (char)23484);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)80, (char)66, (char)77, (char)151, (char)49, (char)179, (char)146, (char)239}));
            assert(pack.min_version_GET() == (char)52768);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)149, (char)224, (char)75, (char)45, (char)136, (char)52, (char)229, (char)97}));
            assert(pack.version_GET() == (char)2011);
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)2011) ;
        p300.library_version_hash_SET(new char[] {(char)149, (char)224, (char)75, (char)45, (char)136, (char)52, (char)229, (char)97}, 0) ;
        p300.spec_version_hash_SET(new char[] {(char)80, (char)66, (char)77, (char)151, (char)49, (char)179, (char)146, (char)239}, 0) ;
        p300.min_version_SET((char)52768) ;
        p300.max_version_SET((char)23484) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
            assert(pack.time_usec_GET() == 6399325978152714302L);
            assert(pack.sub_mode_GET() == (char)106);
            assert(pack.uptime_sec_GET() == 2360895542L);
            assert(pack.vendor_specific_status_code_GET() == (char)20521);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.uptime_sec_SET(2360895542L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK) ;
        p310.sub_mode_SET((char)106) ;
        p310.vendor_specific_status_code_SET((char)20521) ;
        p310.time_usec_SET(6399325978152714302L) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.uptime_sec_GET() == 531260039L);
            assert(pack.hw_version_minor_GET() == (char)145);
            assert(pack.time_usec_GET() == 8938488665755087191L);
            assert(pack.sw_vcs_commit_GET() == 1262123822L);
            assert(pack.sw_version_minor_GET() == (char)223);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)249, (char)222, (char)73, (char)87, (char)135, (char)20, (char)209, (char)224, (char)123, (char)126, (char)227, (char)29, (char)222, (char)66, (char)85, (char)220}));
            assert(pack.sw_version_major_GET() == (char)160);
            assert(pack.hw_version_major_GET() == (char)225);
            assert(pack.name_LEN(ph) == 43);
            assert(pack.name_TRY(ph).equals("otswhOpkczFfMAtxirAbfyWtldgnZpRvASsvakDunwe"));
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.sw_version_minor_SET((char)223) ;
        p311.hw_version_minor_SET((char)145) ;
        p311.hw_unique_id_SET(new char[] {(char)249, (char)222, (char)73, (char)87, (char)135, (char)20, (char)209, (char)224, (char)123, (char)126, (char)227, (char)29, (char)222, (char)66, (char)85, (char)220}, 0) ;
        p311.sw_version_major_SET((char)160) ;
        p311.uptime_sec_SET(531260039L) ;
        p311.time_usec_SET(8938488665755087191L) ;
        p311.sw_vcs_commit_SET(1262123822L) ;
        p311.name_SET("otswhOpkczFfMAtxirAbfyWtldgnZpRvASsvakDunwe", PH) ;
        p311.hw_version_major_SET((char)225) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)223);
            assert(pack.target_system_GET() == (char)141);
            assert(pack.param_index_GET() == (short)11841);
            assert(pack.param_id_LEN(ph) == 13);
            assert(pack.param_id_TRY(ph).equals("DlWdalkEbtmzp"));
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_index_SET((short)11841) ;
        p320.target_component_SET((char)223) ;
        p320.target_system_SET((char)141) ;
        p320.param_id_SET("DlWdalkEbtmzp", PH) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)127);
            assert(pack.target_component_GET() == (char)185);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)185) ;
        p321.target_system_SET((char)127) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 113);
            assert(pack.param_value_TRY(ph).equals("lbuchflOvfcxhbtxkpvbwEvbowtseujqsjvvxqfcfdlgwmfzzcqoFveoypMqugDNqnBvkygoyjTrdcuhOFmbudONlHvchkpfonuffkjLbxwctsgaZ"));
            assert(pack.param_count_GET() == (char)49570);
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("Vyy"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
            assert(pack.param_index_GET() == (char)43841);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_count_SET((char)49570) ;
        p322.param_index_SET((char)43841) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32) ;
        p322.param_value_SET("lbuchflOvfcxhbtxkpvbwEvbowtseujqsjvvxqfcfdlgwmfzzcqoFveoypMqugDNqnBvkygoyjTrdcuhOFmbudONlHvchkpfonuffkjLbxwctsgaZ", PH) ;
        p322.param_id_SET("Vyy", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("o"));
            assert(pack.param_value_LEN(ph) == 126);
            assert(pack.param_value_TRY(ph).equals("dywzGotvVbdvdazwxWgHootekjoswJdfouxagEnlvfgjtcgevyrouyqioyBlomRguWnziryvfqkiufwxjevqlkkvtQijswjhaqfcavihidfhGifrityydkefqmfcdl"));
            assert(pack.target_system_GET() == (char)255);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
            assert(pack.target_component_GET() == (char)9);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_id_SET("o", PH) ;
        p323.target_component_SET((char)9) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64) ;
        p323.target_system_SET((char)255) ;
        p323.param_value_SET("dywzGotvVbdvdazwxWgHootekjoswJdfouxagEnlvfgjtcgevyrouyqioyBlomRguWnziryvfqkiufwxjevqlkkvtQijswjhaqfcavihidfhGifrityydkefqmfcdl", PH) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
            assert(pack.param_value_LEN(ph) == 51);
            assert(pack.param_value_TRY(ph).equals("uihsdqqmuhMFuewcpibIoiEcuyScatunhokqdqkfngYsfnbxbro"));
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("qNgklczunf"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_value_SET("uihsdqqmuhMFuewcpibIoiEcuyScatunhokqdqkfngYsfnbxbro", PH) ;
        p324.param_id_SET("qNgklczunf", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.increment_GET() == (char)41);
            assert(pack.time_usec_GET() == 124793215330725985L);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)62210, (char)10374, (char)44657, (char)62636, (char)7739, (char)26773, (char)42185, (char)45684, (char)5431, (char)5741, (char)45209, (char)56329, (char)31281, (char)42999, (char)62240, (char)10896, (char)516, (char)5700, (char)29055, (char)41452, (char)34929, (char)54995, (char)33447, (char)45299, (char)2725, (char)49888, (char)25961, (char)39641, (char)18436, (char)16006, (char)3611, (char)14235, (char)49012, (char)60161, (char)43300, (char)5262, (char)41497, (char)59863, (char)44587, (char)32999, (char)62386, (char)21094, (char)9742, (char)43018, (char)24445, (char)41020, (char)46612, (char)36294, (char)16069, (char)35801, (char)25991, (char)20920, (char)41068, (char)7841, (char)25668, (char)19904, (char)37355, (char)24766, (char)64872, (char)58478, (char)22709, (char)13586, (char)8824, (char)46798, (char)15893, (char)26662, (char)33976, (char)46991, (char)36472, (char)14290, (char)15684, (char)24253}));
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(pack.min_distance_GET() == (char)55040);
            assert(pack.max_distance_GET() == (char)49489);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.increment_SET((char)41) ;
        p330.distances_SET(new char[] {(char)62210, (char)10374, (char)44657, (char)62636, (char)7739, (char)26773, (char)42185, (char)45684, (char)5431, (char)5741, (char)45209, (char)56329, (char)31281, (char)42999, (char)62240, (char)10896, (char)516, (char)5700, (char)29055, (char)41452, (char)34929, (char)54995, (char)33447, (char)45299, (char)2725, (char)49888, (char)25961, (char)39641, (char)18436, (char)16006, (char)3611, (char)14235, (char)49012, (char)60161, (char)43300, (char)5262, (char)41497, (char)59863, (char)44587, (char)32999, (char)62386, (char)21094, (char)9742, (char)43018, (char)24445, (char)41020, (char)46612, (char)36294, (char)16069, (char)35801, (char)25991, (char)20920, (char)41068, (char)7841, (char)25668, (char)19904, (char)37355, (char)24766, (char)64872, (char)58478, (char)22709, (char)13586, (char)8824, (char)46798, (char)15893, (char)26662, (char)33976, (char)46991, (char)36472, (char)14290, (char)15684, (char)24253}, 0) ;
        p330.max_distance_SET((char)49489) ;
        p330.time_usec_SET(124793215330725985L) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p330.min_distance_SET((char)55040) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
    }

}
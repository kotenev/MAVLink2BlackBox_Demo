
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
            long id = id__a(src);
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
            long id = id__M(src);
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
            long id = id__M(src);
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
            long id = id__M(src);
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
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_STANDBY);
            assert(pack.mavlink_version_GET() == (char)114);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_RESERVED);
            assert(pack.custom_mode_GET() == 2171918699L);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED));
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_VTOL_RESERVED4);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.type_SET(MAV_TYPE.MAV_TYPE_VTOL_RESERVED4) ;
        p0.mavlink_version_SET((char)114) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_STANDBY) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_RESERVED) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED)) ;
        p0.custom_mode_SET(2171918699L) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_count1_GET() == (char)12961);
            assert(pack.battery_remaining_GET() == (byte) - 74);
            assert(pack.drop_rate_comm_GET() == (char)33700);
            assert(pack.errors_comm_GET() == (char)24344);
            assert(pack.errors_count2_GET() == (char)60676);
            assert(pack.voltage_battery_GET() == (char)18348);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY));
            assert(pack.load_GET() == (char)41492);
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
            assert(pack.current_battery_GET() == (short) -22269);
            assert(pack.errors_count3_GET() == (char)56377);
            assert(pack.errors_count4_GET() == (char)14734);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS));
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.load_SET((char)41492) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY)) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.voltage_battery_SET((char)18348) ;
        p1.errors_count2_SET((char)60676) ;
        p1.battery_remaining_SET((byte) - 74) ;
        p1.errors_count1_SET((char)12961) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS)) ;
        p1.drop_rate_comm_SET((char)33700) ;
        p1.errors_comm_SET((char)24344) ;
        p1.errors_count4_SET((char)14734) ;
        p1.current_battery_SET((short) -22269) ;
        p1.errors_count3_SET((char)56377) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 6428546210201420163L);
            assert(pack.time_boot_ms_GET() == 160529456L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(160529456L) ;
        p2.time_unix_usec_SET(6428546210201420163L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == -2.2319765E38F);
            assert(pack.afz_GET() == -2.8822082E37F);
            assert(pack.yaw_rate_GET() == 1.6059574E37F);
            assert(pack.afx_GET() == 4.372313E37F);
            assert(pack.vx_GET() == -2.1961625E38F);
            assert(pack.z_GET() == -3.008739E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.type_mask_GET() == (char)5695);
            assert(pack.time_boot_ms_GET() == 239973699L);
            assert(pack.vy_GET() == -1.1966642E38F);
            assert(pack.yaw_GET() == -1.0794448E38F);
            assert(pack.afy_GET() == 2.6332535E38F);
            assert(pack.x_GET() == 2.2562017E38F);
            assert(pack.y_GET() == 2.5851483E38F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.yaw_rate_SET(1.6059574E37F) ;
        p3.y_SET(2.5851483E38F) ;
        p3.vz_SET(-2.2319765E38F) ;
        p3.time_boot_ms_SET(239973699L) ;
        p3.afy_SET(2.6332535E38F) ;
        p3.vx_SET(-2.1961625E38F) ;
        p3.afz_SET(-2.8822082E37F) ;
        p3.vy_SET(-1.1966642E38F) ;
        p3.yaw_SET(-1.0794448E38F) ;
        p3.x_SET(2.2562017E38F) ;
        p3.afx_SET(4.372313E37F) ;
        p3.type_mask_SET((char)5695) ;
        p3.z_SET(-3.008739E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 2792007394L);
            assert(pack.time_usec_GET() == 1489123618525814853L);
            assert(pack.target_component_GET() == (char)143);
            assert(pack.target_system_GET() == (char)43);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.target_component_SET((char)143) ;
        p4.seq_SET(2792007394L) ;
        p4.target_system_SET((char)43) ;
        p4.time_usec_SET(1489123618525814853L) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.passkey_LEN(ph) == 6);
            assert(pack.passkey_TRY(ph).equals("owmrzh"));
            assert(pack.target_system_GET() == (char)219);
            assert(pack.version_GET() == (char)235);
            assert(pack.control_request_GET() == (char)23);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.control_request_SET((char)23) ;
        p5.passkey_SET("owmrzh", PH) ;
        p5.version_SET((char)235) ;
        p5.target_system_SET((char)219) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)67);
            assert(pack.control_request_GET() == (char)63);
            assert(pack.ack_GET() == (char)39);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)67) ;
        p6.ack_SET((char)39) ;
        p6.control_request_SET((char)63) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 11);
            assert(pack.key_TRY(ph).equals("hclicoghUtd"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("hclicoghUtd", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_ARMED);
            assert(pack.target_system_GET() == (char)208);
            assert(pack.custom_mode_GET() == 549766815L);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)208) ;
        p11.custom_mode_SET(549766815L) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_STABILIZE_ARMED) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)195);
            assert(pack.target_component_GET() == (char)18);
            assert(pack.param_index_GET() == (short)16183);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("g"));
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_component_SET((char)18) ;
        p20.param_id_SET("g", PH) ;
        p20.param_index_SET((short)16183) ;
        p20.target_system_SET((char)195) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)228);
            assert(pack.target_system_GET() == (char)31);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)228) ;
        p21.target_system_SET((char)31) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 13);
            assert(pack.param_id_TRY(ph).equals("hmybtzTkfjyva"));
            assert(pack.param_value_GET() == -5.231339E37F);
            assert(pack.param_count_GET() == (char)55236);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8);
            assert(pack.param_index_GET() == (char)10280);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_count_SET((char)55236) ;
        p22.param_id_SET("hmybtzTkfjyva", PH) ;
        p22.param_value_SET(-5.231339E37F) ;
        p22.param_index_SET((char)10280) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)177);
            assert(pack.param_value_GET() == 5.459324E37F);
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("txuornofbswjihqk"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
            assert(pack.target_component_GET() == (char)155);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.target_component_SET((char)155) ;
        p23.param_value_SET(5.459324E37F) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32) ;
        p23.target_system_SET((char)177) ;
        p23.param_id_SET("txuornofbswjihqk", PH) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6045731234491278956L);
            assert(pack.cog_GET() == (char)47181);
            assert(pack.vel_GET() == (char)10527);
            assert(pack.alt_ellipsoid_TRY(ph) == 609608877);
            assert(pack.hdg_acc_TRY(ph) == 3520555967L);
            assert(pack.lat_GET() == -59038118);
            assert(pack.vel_acc_TRY(ph) == 1079455747L);
            assert(pack.satellites_visible_GET() == (char)240);
            assert(pack.eph_GET() == (char)44800);
            assert(pack.v_acc_TRY(ph) == 571718310L);
            assert(pack.h_acc_TRY(ph) == 2788689920L);
            assert(pack.lon_GET() == 1489090929);
            assert(pack.alt_GET() == 688165591);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.epv_GET() == (char)2277);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.cog_SET((char)47181) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p24.satellites_visible_SET((char)240) ;
        p24.alt_SET(688165591) ;
        p24.lon_SET(1489090929) ;
        p24.eph_SET((char)44800) ;
        p24.vel_acc_SET(1079455747L, PH) ;
        p24.time_usec_SET(6045731234491278956L) ;
        p24.alt_ellipsoid_SET(609608877, PH) ;
        p24.lat_SET(-59038118) ;
        p24.v_acc_SET(571718310L, PH) ;
        p24.epv_SET((char)2277) ;
        p24.vel_SET((char)10527) ;
        p24.h_acc_SET(2788689920L, PH) ;
        p24.hdg_acc_SET(3520555967L, PH) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)10, (char)150, (char)212, (char)12, (char)81, (char)97, (char)27, (char)179, (char)239, (char)16, (char)25, (char)50, (char)105, (char)229, (char)105, (char)98, (char)49, (char)225, (char)52, (char)164}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)74, (char)212, (char)25, (char)146, (char)68, (char)170, (char)160, (char)99, (char)75, (char)22, (char)151, (char)101, (char)237, (char)216, (char)25, (char)28, (char)186, (char)16, (char)37, (char)219}));
            assert(pack.satellites_visible_GET() == (char)191);
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)23, (char)94, (char)184, (char)102, (char)81, (char)54, (char)29, (char)186, (char)235, (char)105, (char)178, (char)225, (char)106, (char)243, (char)158, (char)191, (char)144, (char)169, (char)205, (char)58}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)128, (char)71, (char)13, (char)10, (char)130, (char)44, (char)4, (char)155, (char)148, (char)133, (char)102, (char)102, (char)106, (char)111, (char)251, (char)67, (char)23, (char)89, (char)15, (char)192}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)14, (char)33, (char)246, (char)96, (char)18, (char)204, (char)51, (char)205, (char)190, (char)26, (char)104, (char)227, (char)156, (char)46, (char)95, (char)114, (char)16, (char)250, (char)95, (char)137}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_elevation_SET(new char[] {(char)14, (char)33, (char)246, (char)96, (char)18, (char)204, (char)51, (char)205, (char)190, (char)26, (char)104, (char)227, (char)156, (char)46, (char)95, (char)114, (char)16, (char)250, (char)95, (char)137}, 0) ;
        p25.satellite_used_SET(new char[] {(char)10, (char)150, (char)212, (char)12, (char)81, (char)97, (char)27, (char)179, (char)239, (char)16, (char)25, (char)50, (char)105, (char)229, (char)105, (char)98, (char)49, (char)225, (char)52, (char)164}, 0) ;
        p25.satellites_visible_SET((char)191) ;
        p25.satellite_azimuth_SET(new char[] {(char)23, (char)94, (char)184, (char)102, (char)81, (char)54, (char)29, (char)186, (char)235, (char)105, (char)178, (char)225, (char)106, (char)243, (char)158, (char)191, (char)144, (char)169, (char)205, (char)58}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)128, (char)71, (char)13, (char)10, (char)130, (char)44, (char)4, (char)155, (char)148, (char)133, (char)102, (char)102, (char)106, (char)111, (char)251, (char)67, (char)23, (char)89, (char)15, (char)192}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)74, (char)212, (char)25, (char)146, (char)68, (char)170, (char)160, (char)99, (char)75, (char)22, (char)151, (char)101, (char)237, (char)216, (char)25, (char)28, (char)186, (char)16, (char)37, (char)219}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short) -31436);
            assert(pack.time_boot_ms_GET() == 3173997558L);
            assert(pack.zgyro_GET() == (short) -20067);
            assert(pack.zmag_GET() == (short)26459);
            assert(pack.xmag_GET() == (short) -1618);
            assert(pack.ymag_GET() == (short)18760);
            assert(pack.xgyro_GET() == (short) -31701);
            assert(pack.ygyro_GET() == (short)32110);
            assert(pack.yacc_GET() == (short) -15013);
            assert(pack.xacc_GET() == (short) -25441);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.yacc_SET((short) -15013) ;
        p26.ymag_SET((short)18760) ;
        p26.xmag_SET((short) -1618) ;
        p26.zmag_SET((short)26459) ;
        p26.time_boot_ms_SET(3173997558L) ;
        p26.xgyro_SET((short) -31701) ;
        p26.ygyro_SET((short)32110) ;
        p26.xacc_SET((short) -25441) ;
        p26.zacc_SET((short) -31436) ;
        p26.zgyro_SET((short) -20067) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short) -3671);
            assert(pack.zmag_GET() == (short)20184);
            assert(pack.xacc_GET() == (short)23025);
            assert(pack.yacc_GET() == (short) -28656);
            assert(pack.xgyro_GET() == (short)13668);
            assert(pack.time_usec_GET() == 4306895772905384874L);
            assert(pack.ygyro_GET() == (short) -28979);
            assert(pack.zacc_GET() == (short) -4668);
            assert(pack.ymag_GET() == (short) -15999);
            assert(pack.xmag_GET() == (short) -24205);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.yacc_SET((short) -28656) ;
        p27.time_usec_SET(4306895772905384874L) ;
        p27.ygyro_SET((short) -28979) ;
        p27.ymag_SET((short) -15999) ;
        p27.xacc_SET((short)23025) ;
        p27.zgyro_SET((short) -3671) ;
        p27.xgyro_SET((short)13668) ;
        p27.xmag_SET((short) -24205) ;
        p27.zacc_SET((short) -4668) ;
        p27.zmag_SET((short)20184) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff2_GET() == (short)8641);
            assert(pack.press_diff1_GET() == (short)15429);
            assert(pack.temperature_GET() == (short)16161);
            assert(pack.press_abs_GET() == (short) -3256);
            assert(pack.time_usec_GET() == 915902885948432472L);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.temperature_SET((short)16161) ;
        p28.press_diff2_SET((short)8641) ;
        p28.time_usec_SET(915902885948432472L) ;
        p28.press_abs_SET((short) -3256) ;
        p28.press_diff1_SET((short)15429) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)19838);
            assert(pack.time_boot_ms_GET() == 1277163126L);
            assert(pack.press_diff_GET() == -9.716515E37F);
            assert(pack.press_abs_GET() == -2.6110494E37F);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_abs_SET(-2.6110494E37F) ;
        p29.temperature_SET((short)19838) ;
        p29.press_diff_SET(-9.716515E37F) ;
        p29.time_boot_ms_SET(1277163126L) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 4144783903L);
            assert(pack.rollspeed_GET() == -7.1335526E37F);
            assert(pack.roll_GET() == 2.9386145E38F);
            assert(pack.pitchspeed_GET() == -7.4668966E37F);
            assert(pack.yaw_GET() == 2.9936942E38F);
            assert(pack.yawspeed_GET() == -1.615204E38F);
            assert(pack.pitch_GET() == -2.3019505E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.roll_SET(2.9386145E38F) ;
        p30.pitchspeed_SET(-7.4668966E37F) ;
        p30.pitch_SET(-2.3019505E38F) ;
        p30.rollspeed_SET(-7.1335526E37F) ;
        p30.yaw_SET(2.9936942E38F) ;
        p30.time_boot_ms_SET(4144783903L) ;
        p30.yawspeed_SET(-1.615204E38F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q1_GET() == -6.0763037E37F);
            assert(pack.pitchspeed_GET() == -6.5640687E37F);
            assert(pack.q3_GET() == -2.4233934E38F);
            assert(pack.rollspeed_GET() == 1.3198876E38F);
            assert(pack.q4_GET() == -2.4234267E38F);
            assert(pack.q2_GET() == -2.4772961E38F);
            assert(pack.time_boot_ms_GET() == 2308962298L);
            assert(pack.yawspeed_GET() == 9.706311E37F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q4_SET(-2.4234267E38F) ;
        p31.q1_SET(-6.0763037E37F) ;
        p31.q3_SET(-2.4233934E38F) ;
        p31.time_boot_ms_SET(2308962298L) ;
        p31.q2_SET(-2.4772961E38F) ;
        p31.rollspeed_SET(1.3198876E38F) ;
        p31.yawspeed_SET(9.706311E37F) ;
        p31.pitchspeed_SET(-6.5640687E37F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 2.7822584E38F);
            assert(pack.vy_GET() == -2.8678773E38F);
            assert(pack.x_GET() == -2.774608E38F);
            assert(pack.y_GET() == 1.1843712E38F);
            assert(pack.vx_GET() == 1.0929455E37F);
            assert(pack.vz_GET() == 1.6464238E38F);
            assert(pack.time_boot_ms_GET() == 1319560913L);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.y_SET(1.1843712E38F) ;
        p32.vz_SET(1.6464238E38F) ;
        p32.vy_SET(-2.8678773E38F) ;
        p32.x_SET(-2.774608E38F) ;
        p32.vx_SET(1.0929455E37F) ;
        p32.time_boot_ms_SET(1319560913L) ;
        p32.z_SET(2.7822584E38F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == (short) -12025);
            assert(pack.lat_GET() == -1693510171);
            assert(pack.alt_GET() == 1507862373);
            assert(pack.vx_GET() == (short)3913);
            assert(pack.vy_GET() == (short)28945);
            assert(pack.hdg_GET() == (char)53169);
            assert(pack.time_boot_ms_GET() == 635441272L);
            assert(pack.lon_GET() == -334113630);
            assert(pack.relative_alt_GET() == 513270372);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.alt_SET(1507862373) ;
        p33.hdg_SET((char)53169) ;
        p33.time_boot_ms_SET(635441272L) ;
        p33.vx_SET((short)3913) ;
        p33.vy_SET((short)28945) ;
        p33.vz_SET((short) -12025) ;
        p33.lat_SET(-1693510171) ;
        p33.relative_alt_SET(513270372) ;
        p33.lon_SET(-334113630) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan6_scaled_GET() == (short)23549);
            assert(pack.rssi_GET() == (char)64);
            assert(pack.chan1_scaled_GET() == (short) -6531);
            assert(pack.chan5_scaled_GET() == (short) -14565);
            assert(pack.port_GET() == (char)99);
            assert(pack.chan2_scaled_GET() == (short)32229);
            assert(pack.chan4_scaled_GET() == (short) -3356);
            assert(pack.chan7_scaled_GET() == (short) -14578);
            assert(pack.chan3_scaled_GET() == (short) -4838);
            assert(pack.time_boot_ms_GET() == 3955003931L);
            assert(pack.chan8_scaled_GET() == (short) -599);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.rssi_SET((char)64) ;
        p34.port_SET((char)99) ;
        p34.chan1_scaled_SET((short) -6531) ;
        p34.chan2_scaled_SET((short)32229) ;
        p34.time_boot_ms_SET(3955003931L) ;
        p34.chan7_scaled_SET((short) -14578) ;
        p34.chan8_scaled_SET((short) -599) ;
        p34.chan3_scaled_SET((short) -4838) ;
        p34.chan5_scaled_SET((short) -14565) ;
        p34.chan6_scaled_SET((short)23549) ;
        p34.chan4_scaled_SET((short) -3356) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan1_raw_GET() == (char)57290);
            assert(pack.chan4_raw_GET() == (char)51870);
            assert(pack.rssi_GET() == (char)187);
            assert(pack.time_boot_ms_GET() == 4170748570L);
            assert(pack.chan6_raw_GET() == (char)52268);
            assert(pack.chan3_raw_GET() == (char)39199);
            assert(pack.chan8_raw_GET() == (char)53403);
            assert(pack.port_GET() == (char)165);
            assert(pack.chan2_raw_GET() == (char)5790);
            assert(pack.chan7_raw_GET() == (char)19908);
            assert(pack.chan5_raw_GET() == (char)44434);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan2_raw_SET((char)5790) ;
        p35.chan5_raw_SET((char)44434) ;
        p35.chan7_raw_SET((char)19908) ;
        p35.rssi_SET((char)187) ;
        p35.chan8_raw_SET((char)53403) ;
        p35.port_SET((char)165) ;
        p35.chan6_raw_SET((char)52268) ;
        p35.time_boot_ms_SET(4170748570L) ;
        p35.chan4_raw_SET((char)51870) ;
        p35.chan1_raw_SET((char)57290) ;
        p35.chan3_raw_SET((char)39199) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo10_raw_TRY(ph) == (char)64557);
            assert(pack.time_usec_GET() == 3562033608L);
            assert(pack.servo11_raw_TRY(ph) == (char)36278);
            assert(pack.servo12_raw_TRY(ph) == (char)38344);
            assert(pack.servo16_raw_TRY(ph) == (char)43613);
            assert(pack.servo3_raw_GET() == (char)41874);
            assert(pack.servo5_raw_GET() == (char)42434);
            assert(pack.servo14_raw_TRY(ph) == (char)46975);
            assert(pack.servo9_raw_TRY(ph) == (char)19036);
            assert(pack.servo15_raw_TRY(ph) == (char)12780);
            assert(pack.servo1_raw_GET() == (char)60348);
            assert(pack.servo8_raw_GET() == (char)30927);
            assert(pack.port_GET() == (char)161);
            assert(pack.servo6_raw_GET() == (char)28110);
            assert(pack.servo13_raw_TRY(ph) == (char)58344);
            assert(pack.servo2_raw_GET() == (char)48334);
            assert(pack.servo4_raw_GET() == (char)45552);
            assert(pack.servo7_raw_GET() == (char)8010);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo4_raw_SET((char)45552) ;
        p36.servo14_raw_SET((char)46975, PH) ;
        p36.servo8_raw_SET((char)30927) ;
        p36.servo15_raw_SET((char)12780, PH) ;
        p36.servo7_raw_SET((char)8010) ;
        p36.servo5_raw_SET((char)42434) ;
        p36.servo11_raw_SET((char)36278, PH) ;
        p36.servo6_raw_SET((char)28110) ;
        p36.servo16_raw_SET((char)43613, PH) ;
        p36.servo10_raw_SET((char)64557, PH) ;
        p36.servo1_raw_SET((char)60348) ;
        p36.servo13_raw_SET((char)58344, PH) ;
        p36.servo9_raw_SET((char)19036, PH) ;
        p36.servo12_raw_SET((char)38344, PH) ;
        p36.servo3_raw_SET((char)41874) ;
        p36.servo2_raw_SET((char)48334) ;
        p36.time_usec_SET(3562033608L) ;
        p36.port_SET((char)161) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short) -11816);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)234);
            assert(pack.target_component_GET() == (char)149);
            assert(pack.end_index_GET() == (short)11618);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p37.target_system_SET((char)234) ;
        p37.start_index_SET((short) -11816) ;
        p37.target_component_SET((char)149) ;
        p37.end_index_SET((short)11618) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)58);
            assert(pack.end_index_GET() == (short) -30989);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.start_index_GET() == (short)27138);
            assert(pack.target_component_GET() == (char)58);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_system_SET((char)58) ;
        p38.end_index_SET((short) -30989) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p38.target_component_SET((char)58) ;
        p38.start_index_SET((short)27138) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.x_GET() == -2.4535797E38F);
            assert(pack.autocontinue_GET() == (char)2);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.param3_GET() == 3.0345614E38F);
            assert(pack.current_GET() == (char)208);
            assert(pack.param1_GET() == 2.5755385E38F);
            assert(pack.seq_GET() == (char)345);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_RALLY_LAND);
            assert(pack.param2_GET() == 3.3894006E38F);
            assert(pack.z_GET() == -2.3198957E38F);
            assert(pack.y_GET() == -2.3999995E38F);
            assert(pack.param4_GET() == -8.739469E36F);
            assert(pack.target_system_GET() == (char)81);
            assert(pack.target_component_GET() == (char)209);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.target_component_SET((char)209) ;
        p39.current_SET((char)208) ;
        p39.seq_SET((char)345) ;
        p39.param4_SET(-8.739469E36F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_DO_RALLY_LAND) ;
        p39.param3_SET(3.0345614E38F) ;
        p39.autocontinue_SET((char)2) ;
        p39.y_SET(-2.3999995E38F) ;
        p39.param2_SET(3.3894006E38F) ;
        p39.z_SET(-2.3198957E38F) ;
        p39.x_SET(-2.4535797E38F) ;
        p39.param1_SET(2.5755385E38F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p39.target_system_SET((char)81) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)168);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.seq_GET() == (char)32072);
            assert(pack.target_component_GET() == (char)47);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_system_SET((char)168) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p40.target_component_SET((char)47) ;
        p40.seq_SET((char)32072) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)25128);
            assert(pack.target_system_GET() == (char)33);
            assert(pack.target_component_GET() == (char)100);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)33) ;
        p41.seq_SET((char)25128) ;
        p41.target_component_SET((char)100) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)56713);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)56713) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)224);
            assert(pack.target_system_GET() == (char)26);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)26) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p43.target_component_SET((char)224) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)186);
            assert(pack.target_component_GET() == (char)216);
            assert(pack.count_GET() == (char)36131);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p44.count_SET((char)36131) ;
        p44.target_component_SET((char)216) ;
        p44.target_system_SET((char)186) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)167);
            assert(pack.target_component_GET() == (char)49);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)167) ;
        p45.target_component_SET((char)49) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)62847);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)62847) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)216);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_DENIED);
            assert(pack.target_system_GET() == (char)31);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_DENIED) ;
        p47.target_component_SET((char)216) ;
        p47.target_system_SET((char)31) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == -1712288980);
            assert(pack.longitude_GET() == -2019347399);
            assert(pack.time_usec_TRY(ph) == 1820195911918829896L);
            assert(pack.target_system_GET() == (char)145);
            assert(pack.altitude_GET() == 2035658231);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.latitude_SET(-1712288980) ;
        p48.altitude_SET(2035658231) ;
        p48.longitude_SET(-2019347399) ;
        p48.time_usec_SET(1820195911918829896L, PH) ;
        p48.target_system_SET((char)145) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == -1882402680);
            assert(pack.latitude_GET() == 1697260495);
            assert(pack.time_usec_TRY(ph) == 8154806740146451593L);
            assert(pack.altitude_GET() == 420260596);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(1697260495) ;
        p49.longitude_SET(-1882402680) ;
        p49.time_usec_SET(8154806740146451593L, PH) ;
        p49.altitude_SET(420260596) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)139);
            assert(pack.target_system_GET() == (char)255);
            assert(pack.param_index_GET() == (short)22359);
            assert(pack.parameter_rc_channel_index_GET() == (char)133);
            assert(pack.param_id_LEN(ph) == 6);
            assert(pack.param_id_TRY(ph).equals("fLnWeG"));
            assert(pack.param_value_max_GET() == 4.379028E37F);
            assert(pack.param_value0_GET() == -2.5542703E37F);
            assert(pack.param_value_min_GET() == 2.150617E38F);
            assert(pack.scale_GET() == -1.2127474E38F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_system_SET((char)255) ;
        p50.parameter_rc_channel_index_SET((char)133) ;
        p50.param_index_SET((short)22359) ;
        p50.param_id_SET("fLnWeG", PH) ;
        p50.scale_SET(-1.2127474E38F) ;
        p50.param_value0_SET(-2.5542703E37F) ;
        p50.param_value_max_SET(4.379028E37F) ;
        p50.param_value_min_SET(2.150617E38F) ;
        p50.target_component_SET((char)139) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)154);
            assert(pack.seq_GET() == (char)14099);
            assert(pack.target_system_GET() == (char)253);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.seq_SET((char)14099) ;
        p51.target_system_SET((char)253) ;
        p51.target_component_SET((char)154) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2y_GET() == 1.4615996E38F);
            assert(pack.p1y_GET() == -2.1481114E38F);
            assert(pack.target_component_GET() == (char)72);
            assert(pack.p1z_GET() == 6.6673376E37F);
            assert(pack.target_system_GET() == (char)55);
            assert(pack.p2z_GET() == 1.995617E38F);
            assert(pack.p2x_GET() == 2.5633097E37F);
            assert(pack.p1x_GET() == 5.3767375E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.target_system_SET((char)55) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p54.p1z_SET(6.6673376E37F) ;
        p54.p1x_SET(5.3767375E37F) ;
        p54.target_component_SET((char)72) ;
        p54.p1y_SET(-2.1481114E38F) ;
        p54.p2y_SET(1.4615996E38F) ;
        p54.p2z_SET(1.995617E38F) ;
        p54.p2x_SET(2.5633097E37F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.p1y_GET() == -2.626513E38F);
            assert(pack.p1x_GET() == -2.7881277E38F);
            assert(pack.p1z_GET() == 2.9749105E38F);
            assert(pack.p2y_GET() == -3.1818764E38F);
            assert(pack.p2z_GET() == -6.022222E37F);
            assert(pack.p2x_GET() == -3.3964167E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p55.p2z_SET(-6.022222E37F) ;
        p55.p1z_SET(2.9749105E38F) ;
        p55.p2x_SET(-3.3964167E38F) ;
        p55.p2y_SET(-3.1818764E38F) ;
        p55.p1x_SET(-2.7881277E38F) ;
        p55.p1y_SET(-2.626513E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5590566490129487892L);
            assert(pack.yawspeed_GET() == -2.8800162E38F);
            assert(pack.rollspeed_GET() == -1.8264365E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.9707175E37F, 9.0519E37F, -3.381991E38F, 1.658546E38F}));
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.8969683E38F, 3.2321273E38F, 6.994119E37F, 2.1869277E38F, 1.9339172E38F, -5.438313E37F, 2.3122888E38F, -2.1403877E38F, -1.1454886E38F}));
            assert(pack.pitchspeed_GET() == 2.5406542E37F);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.time_usec_SET(5590566490129487892L) ;
        p61.covariance_SET(new float[] {-1.8969683E38F, 3.2321273E38F, 6.994119E37F, 2.1869277E38F, 1.9339172E38F, -5.438313E37F, 2.3122888E38F, -2.1403877E38F, -1.1454886E38F}, 0) ;
        p61.yawspeed_SET(-2.8800162E38F) ;
        p61.q_SET(new float[] {-3.9707175E37F, 9.0519E37F, -3.381991E38F, 1.658546E38F}, 0) ;
        p61.rollspeed_SET(-1.8264365E38F) ;
        p61.pitchspeed_SET(2.5406542E37F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_pitch_GET() == 1.5576436E38F);
            assert(pack.nav_roll_GET() == -1.4800498E38F);
            assert(pack.nav_bearing_GET() == (short)28396);
            assert(pack.target_bearing_GET() == (short)21427);
            assert(pack.wp_dist_GET() == (char)33107);
            assert(pack.alt_error_GET() == -3.2679503E38F);
            assert(pack.aspd_error_GET() == 1.3377314E38F);
            assert(pack.xtrack_error_GET() == -2.6382385E38F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.wp_dist_SET((char)33107) ;
        p62.aspd_error_SET(1.3377314E38F) ;
        p62.xtrack_error_SET(-2.6382385E38F) ;
        p62.nav_roll_SET(-1.4800498E38F) ;
        p62.alt_error_SET(-3.2679503E38F) ;
        p62.target_bearing_SET((short)21427) ;
        p62.nav_pitch_SET(1.5576436E38F) ;
        p62.nav_bearing_SET((short)28396) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -685581316);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.vy_GET() == -5.684096E37F);
            assert(pack.time_usec_GET() == 6824110808818469780L);
            assert(pack.lat_GET() == 1538697967);
            assert(pack.alt_GET() == 570147246);
            assert(pack.relative_alt_GET() == -992370242);
            assert(pack.vx_GET() == -1.5893995E38F);
            assert(pack.vz_GET() == -2.243514E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.4896406E38F, -1.050291E38F, 5.0452413E37F, 1.8470876E38F, 1.4376206E38F, -4.4300297E37F, 1.1970424E38F, -1.0427402E37F, -3.2021937E38F, 2.1491205E38F, 2.4595004E38F, 3.2954372E38F, 2.8125686E38F, -2.3003964E38F, -1.0446986E38F, -1.7356667E37F, -7.338523E36F, 2.8082112E38F, -1.2977822E38F, -8.89099E37F, -2.1422478E38F, -2.0971107E38F, -2.6265406E38F, 8.3716746E37F, 5.5434006E37F, -2.7260579E38F, 3.197795E38F, -2.9525597E38F, 2.0047665E38F, -1.9190932E38F, -1.5829866E38F, 3.3334146E38F, 3.0799518E38F, -1.4250255E38F, 6.0705703E37F, 2.0128497E38F}));
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.alt_SET(570147246) ;
        p63.relative_alt_SET(-992370242) ;
        p63.lon_SET(-685581316) ;
        p63.vz_SET(-2.243514E38F) ;
        p63.covariance_SET(new float[] {-2.4896406E38F, -1.050291E38F, 5.0452413E37F, 1.8470876E38F, 1.4376206E38F, -4.4300297E37F, 1.1970424E38F, -1.0427402E37F, -3.2021937E38F, 2.1491205E38F, 2.4595004E38F, 3.2954372E38F, 2.8125686E38F, -2.3003964E38F, -1.0446986E38F, -1.7356667E37F, -7.338523E36F, 2.8082112E38F, -1.2977822E38F, -8.89099E37F, -2.1422478E38F, -2.0971107E38F, -2.6265406E38F, 8.3716746E37F, 5.5434006E37F, -2.7260579E38F, 3.197795E38F, -2.9525597E38F, 2.0047665E38F, -1.9190932E38F, -1.5829866E38F, 3.3334146E38F, 3.0799518E38F, -1.4250255E38F, 6.0705703E37F, 2.0128497E38F}, 0) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p63.lat_SET(1538697967) ;
        p63.time_usec_SET(6824110808818469780L) ;
        p63.vy_SET(-5.684096E37F) ;
        p63.vx_SET(-1.5893995E38F) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 3.1856982E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
            assert(pack.vx_GET() == -2.0782004E38F);
            assert(pack.time_usec_GET() == 135306198427624786L);
            assert(pack.az_GET() == -1.0043369E38F);
            assert(pack.vz_GET() == 1.4729612E38F);
            assert(pack.ax_GET() == 1.114145E37F);
            assert(pack.z_GET() == 6.216213E37F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.485877E38F, -3.262434E38F, 1.6350768E38F, -4.462789E37F, -3.0027582E38F, -2.3942001E38F, 3.1178341E38F, 9.294284E37F, 1.0409385E38F, -2.7211127E37F, 2.697602E38F, -2.57934E38F, 2.483669E38F, -1.7168058E38F, 8.4842815E37F, -5.2214576E36F, -1.8797942E38F, -2.517057E38F, 2.8610268E38F, -8.563794E37F, -7.2628327E37F, -1.4148488E38F, 3.0696098E38F, 1.2245908E38F, 3.3545138E38F, -1.8456788E38F, 3.0550905E38F, -2.4632307E38F, 3.3467953E38F, 2.8198228E38F, -2.9834536E38F, 3.0900008E38F, -4.3354604E37F, 9.210244E37F, 1.4291604E38F, -5.408162E37F, 1.9134107E38F, -8.997749E37F, -1.1082777E38F, 1.4000302E38F, -3.2064674E38F, 1.5018453E38F, -2.0405426E38F, -2.2459625E38F, 2.5828316E38F}));
            assert(pack.ay_GET() == -2.0273475E37F);
            assert(pack.y_GET() == 7.9428056E37F);
            assert(pack.vy_GET() == -2.5219998E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.covariance_SET(new float[] {-1.485877E38F, -3.262434E38F, 1.6350768E38F, -4.462789E37F, -3.0027582E38F, -2.3942001E38F, 3.1178341E38F, 9.294284E37F, 1.0409385E38F, -2.7211127E37F, 2.697602E38F, -2.57934E38F, 2.483669E38F, -1.7168058E38F, 8.4842815E37F, -5.2214576E36F, -1.8797942E38F, -2.517057E38F, 2.8610268E38F, -8.563794E37F, -7.2628327E37F, -1.4148488E38F, 3.0696098E38F, 1.2245908E38F, 3.3545138E38F, -1.8456788E38F, 3.0550905E38F, -2.4632307E38F, 3.3467953E38F, 2.8198228E38F, -2.9834536E38F, 3.0900008E38F, -4.3354604E37F, 9.210244E37F, 1.4291604E38F, -5.408162E37F, 1.9134107E38F, -8.997749E37F, -1.1082777E38F, 1.4000302E38F, -3.2064674E38F, 1.5018453E38F, -2.0405426E38F, -2.2459625E38F, 2.5828316E38F}, 0) ;
        p64.vx_SET(-2.0782004E38F) ;
        p64.time_usec_SET(135306198427624786L) ;
        p64.x_SET(3.1856982E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE) ;
        p64.ay_SET(-2.0273475E37F) ;
        p64.vz_SET(1.4729612E38F) ;
        p64.vy_SET(-2.5219998E38F) ;
        p64.ax_SET(1.114145E37F) ;
        p64.z_SET(6.216213E37F) ;
        p64.az_SET(-1.0043369E38F) ;
        p64.y_SET(7.9428056E37F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan14_raw_GET() == (char)43793);
            assert(pack.chan13_raw_GET() == (char)42789);
            assert(pack.chan3_raw_GET() == (char)28317);
            assert(pack.chan18_raw_GET() == (char)32473);
            assert(pack.chan4_raw_GET() == (char)3970);
            assert(pack.time_boot_ms_GET() == 2023090105L);
            assert(pack.chan5_raw_GET() == (char)52844);
            assert(pack.chan2_raw_GET() == (char)36802);
            assert(pack.chan1_raw_GET() == (char)9279);
            assert(pack.chan15_raw_GET() == (char)22878);
            assert(pack.chan10_raw_GET() == (char)38343);
            assert(pack.chancount_GET() == (char)196);
            assert(pack.rssi_GET() == (char)141);
            assert(pack.chan7_raw_GET() == (char)12148);
            assert(pack.chan6_raw_GET() == (char)24556);
            assert(pack.chan16_raw_GET() == (char)48304);
            assert(pack.chan11_raw_GET() == (char)5779);
            assert(pack.chan17_raw_GET() == (char)49230);
            assert(pack.chan9_raw_GET() == (char)9132);
            assert(pack.chan8_raw_GET() == (char)14876);
            assert(pack.chan12_raw_GET() == (char)8323);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan12_raw_SET((char)8323) ;
        p65.chan13_raw_SET((char)42789) ;
        p65.chan2_raw_SET((char)36802) ;
        p65.chan1_raw_SET((char)9279) ;
        p65.chan16_raw_SET((char)48304) ;
        p65.chan18_raw_SET((char)32473) ;
        p65.chan17_raw_SET((char)49230) ;
        p65.chan5_raw_SET((char)52844) ;
        p65.chan10_raw_SET((char)38343) ;
        p65.chan8_raw_SET((char)14876) ;
        p65.chan3_raw_SET((char)28317) ;
        p65.time_boot_ms_SET(2023090105L) ;
        p65.chan14_raw_SET((char)43793) ;
        p65.chan11_raw_SET((char)5779) ;
        p65.rssi_SET((char)141) ;
        p65.chan9_raw_SET((char)9132) ;
        p65.chan7_raw_SET((char)12148) ;
        p65.chan4_raw_SET((char)3970) ;
        p65.chancount_SET((char)196) ;
        p65.chan6_raw_SET((char)24556) ;
        p65.chan15_raw_SET((char)22878) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)25);
            assert(pack.start_stop_GET() == (char)19);
            assert(pack.req_message_rate_GET() == (char)53235);
            assert(pack.target_component_GET() == (char)74);
            assert(pack.req_stream_id_GET() == (char)129);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_system_SET((char)25) ;
        p66.req_stream_id_SET((char)129) ;
        p66.start_stop_SET((char)19) ;
        p66.req_message_rate_SET((char)53235) ;
        p66.target_component_SET((char)74) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.message_rate_GET() == (char)10355);
            assert(pack.on_off_GET() == (char)99);
            assert(pack.stream_id_GET() == (char)161);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)10355) ;
        p67.on_off_SET((char)99) ;
        p67.stream_id_SET((char)161) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == (short) -20700);
            assert(pack.y_GET() == (short) -3288);
            assert(pack.r_GET() == (short)16771);
            assert(pack.buttons_GET() == (char)49931);
            assert(pack.target_GET() == (char)177);
            assert(pack.x_GET() == (short)30714);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.target_SET((char)177) ;
        p69.x_SET((short)30714) ;
        p69.y_SET((short) -3288) ;
        p69.buttons_SET((char)49931) ;
        p69.z_SET((short) -20700) ;
        p69.r_SET((short)16771) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)221);
            assert(pack.chan5_raw_GET() == (char)46646);
            assert(pack.chan1_raw_GET() == (char)31454);
            assert(pack.target_system_GET() == (char)233);
            assert(pack.chan2_raw_GET() == (char)17946);
            assert(pack.chan6_raw_GET() == (char)30982);
            assert(pack.chan8_raw_GET() == (char)63359);
            assert(pack.chan4_raw_GET() == (char)34269);
            assert(pack.chan7_raw_GET() == (char)49968);
            assert(pack.chan3_raw_GET() == (char)63666);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan6_raw_SET((char)30982) ;
        p70.chan3_raw_SET((char)63666) ;
        p70.chan4_raw_SET((char)34269) ;
        p70.chan1_raw_SET((char)31454) ;
        p70.chan5_raw_SET((char)46646) ;
        p70.chan7_raw_SET((char)49968) ;
        p70.chan8_raw_SET((char)63359) ;
        p70.target_system_SET((char)233) ;
        p70.target_component_SET((char)221) ;
        p70.chan2_raw_SET((char)17946) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param3_GET() == 4.891449E37F);
            assert(pack.autocontinue_GET() == (char)36);
            assert(pack.param2_GET() == 6.034447E36F);
            assert(pack.z_GET() == 1.8968097E38F);
            assert(pack.param1_GET() == 2.5365088E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH);
            assert(pack.y_GET() == -1461268425);
            assert(pack.target_system_GET() == (char)42);
            assert(pack.param4_GET() == 2.8891244E38F);
            assert(pack.target_component_GET() == (char)38);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.x_GET() == -596694020);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.seq_GET() == (char)17889);
            assert(pack.current_GET() == (char)245);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.param4_SET(2.8891244E38F) ;
        p73.target_system_SET((char)42) ;
        p73.param2_SET(6.034447E36F) ;
        p73.z_SET(1.8968097E38F) ;
        p73.x_SET(-596694020) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p73.param3_SET(4.891449E37F) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p73.seq_SET((char)17889) ;
        p73.command_SET(MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH) ;
        p73.autocontinue_SET((char)36) ;
        p73.target_component_SET((char)38) ;
        p73.y_SET(-1461268425) ;
        p73.param1_SET(2.5365088E37F) ;
        p73.current_SET((char)245) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.throttle_GET() == (char)16758);
            assert(pack.groundspeed_GET() == 1.9115559E38F);
            assert(pack.alt_GET() == -2.4124387E38F);
            assert(pack.climb_GET() == -1.544162E38F);
            assert(pack.heading_GET() == (short)6233);
            assert(pack.airspeed_GET() == -2.352662E38F);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.throttle_SET((char)16758) ;
        p74.heading_SET((short)6233) ;
        p74.alt_SET(-2.4124387E38F) ;
        p74.groundspeed_SET(1.9115559E38F) ;
        p74.climb_SET(-1.544162E38F) ;
        p74.airspeed_SET(-2.352662E38F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -1010414474);
            assert(pack.target_component_GET() == (char)240);
            assert(pack.param1_GET() == -2.8619072E38F);
            assert(pack.target_system_GET() == (char)54);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN);
            assert(pack.z_GET() == -2.874405E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.autocontinue_GET() == (char)100);
            assert(pack.param3_GET() == 5.288352E37F);
            assert(pack.param4_GET() == -3.0877249E38F);
            assert(pack.x_GET() == -773943118);
            assert(pack.param2_GET() == 1.6180401E38F);
            assert(pack.current_GET() == (char)137);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p75.target_system_SET((char)54) ;
        p75.param2_SET(1.6180401E38F) ;
        p75.command_SET(MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN) ;
        p75.param1_SET(-2.8619072E38F) ;
        p75.autocontinue_SET((char)100) ;
        p75.param3_SET(5.288352E37F) ;
        p75.x_SET(-773943118) ;
        p75.z_SET(-2.874405E38F) ;
        p75.y_SET(-1010414474) ;
        p75.current_SET((char)137) ;
        p75.param4_SET(-3.0877249E38F) ;
        p75.target_component_SET((char)240) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param5_GET() == 1.7326388E38F);
            assert(pack.param1_GET() == -1.5905909E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE);
            assert(pack.target_component_GET() == (char)206);
            assert(pack.target_system_GET() == (char)220);
            assert(pack.param2_GET() == -6.126919E36F);
            assert(pack.param3_GET() == -7.3414884E37F);
            assert(pack.param7_GET() == 2.3468594E37F);
            assert(pack.param4_GET() == -1.1622579E38F);
            assert(pack.confirmation_GET() == (char)67);
            assert(pack.param6_GET() == -2.2490197E38F);
        });
        GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param4_SET(-1.1622579E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE) ;
        p76.param1_SET(-1.5905909E38F) ;
        p76.param6_SET(-2.2490197E38F) ;
        p76.param2_SET(-6.126919E36F) ;
        p76.target_component_SET((char)206) ;
        p76.confirmation_SET((char)67) ;
        p76.target_system_SET((char)220) ;
        p76.param3_SET(-7.3414884E37F) ;
        p76.param5_SET(1.7326388E38F) ;
        p76.param7_SET(2.3468594E37F) ;
        CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.progress_TRY(ph) == (char)173);
            assert(pack.target_system_TRY(ph) == (char)223);
            assert(pack.result_param2_TRY(ph) == 663316159);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_IN_PROGRESS);
            assert(pack.target_component_TRY(ph) == (char)167);
        });
        GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.command_SET(MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) ;
        p77.target_system_SET((char)223, PH) ;
        p77.target_component_SET((char)167, PH) ;
        p77.progress_SET((char)173, PH) ;
        p77.result_param2_SET(663316159, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_IN_PROGRESS) ;
        CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.mode_switch_GET() == (char)124);
            assert(pack.pitch_GET() == -2.835037E38F);
            assert(pack.yaw_GET() == -1.6033119E38F);
            assert(pack.thrust_GET() == 1.0816884E38F);
            assert(pack.roll_GET() == 1.7578981E38F);
            assert(pack.time_boot_ms_GET() == 3062774383L);
            assert(pack.manual_override_switch_GET() == (char)224);
        });
        GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.mode_switch_SET((char)124) ;
        p81.manual_override_switch_SET((char)224) ;
        p81.thrust_SET(1.0816884E38F) ;
        p81.time_boot_ms_SET(3062774383L) ;
        p81.pitch_SET(-2.835037E38F) ;
        p81.yaw_SET(-1.6033119E38F) ;
        p81.roll_SET(1.7578981E38F) ;
        CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)93);
            assert(pack.time_boot_ms_GET() == 4155375278L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.9856833E37F, 6.3240447E37F, 2.9825707E38F, -1.4660091E38F}));
            assert(pack.body_pitch_rate_GET() == 9.280804E37F);
            assert(pack.thrust_GET() == 8.47723E37F);
            assert(pack.target_component_GET() == (char)59);
            assert(pack.body_roll_rate_GET() == 3.377336E38F);
            assert(pack.body_yaw_rate_GET() == 1.379162E38F);
            assert(pack.target_system_GET() == (char)115);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.q_SET(new float[] {2.9856833E37F, 6.3240447E37F, 2.9825707E38F, -1.4660091E38F}, 0) ;
        p82.target_component_SET((char)59) ;
        p82.body_yaw_rate_SET(1.379162E38F) ;
        p82.target_system_SET((char)115) ;
        p82.time_boot_ms_SET(4155375278L) ;
        p82.body_roll_rate_SET(3.377336E38F) ;
        p82.type_mask_SET((char)93) ;
        p82.thrust_SET(8.47723E37F) ;
        p82.body_pitch_rate_SET(9.280804E37F) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2135918409L);
            assert(pack.body_yaw_rate_GET() == -2.5474274E38F);
            assert(pack.thrust_GET() == 1.2487187E38F);
            assert(pack.body_roll_rate_GET() == 2.2512846E38F);
            assert(pack.body_pitch_rate_GET() == -4.163158E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.8982703E38F, 2.8872736E38F, -9.373184E37F, -2.8688639E38F}));
            assert(pack.type_mask_GET() == (char)236);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.time_boot_ms_SET(2135918409L) ;
        p83.thrust_SET(1.2487187E38F) ;
        p83.body_pitch_rate_SET(-4.163158E37F) ;
        p83.type_mask_SET((char)236) ;
        p83.body_roll_rate_SET(2.2512846E38F) ;
        p83.body_yaw_rate_SET(-2.5474274E38F) ;
        p83.q_SET(new float[] {2.8982703E38F, 2.8872736E38F, -9.373184E37F, -2.8688639E38F}, 0) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.afx_GET() == -8.256982E37F);
            assert(pack.afz_GET() == 1.2009062E38F);
            assert(pack.z_GET() == 6.418878E37F);
            assert(pack.yaw_rate_GET() == 2.8494139E38F);
            assert(pack.type_mask_GET() == (char)38929);
            assert(pack.x_GET() == 2.3954215E38F);
            assert(pack.target_component_GET() == (char)152);
            assert(pack.vx_GET() == -8.8949426E36F);
            assert(pack.y_GET() == -8.419122E37F);
            assert(pack.time_boot_ms_GET() == 188187484L);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.target_system_GET() == (char)38);
            assert(pack.afy_GET() == -2.997895E38F);
            assert(pack.vz_GET() == 2.6732368E38F);
            assert(pack.vy_GET() == 7.389312E37F);
            assert(pack.yaw_GET() == -6.5741313E37F);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.x_SET(2.3954215E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p84.time_boot_ms_SET(188187484L) ;
        p84.target_component_SET((char)152) ;
        p84.vx_SET(-8.8949426E36F) ;
        p84.vy_SET(7.389312E37F) ;
        p84.y_SET(-8.419122E37F) ;
        p84.vz_SET(2.6732368E38F) ;
        p84.type_mask_SET((char)38929) ;
        p84.target_system_SET((char)38) ;
        p84.yaw_rate_SET(2.8494139E38F) ;
        p84.z_SET(6.418878E37F) ;
        p84.afx_SET(-8.256982E37F) ;
        p84.yaw_SET(-6.5741313E37F) ;
        p84.afz_SET(1.2009062E38F) ;
        p84.afy_SET(-2.997895E38F) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == -2.2245157E38F);
            assert(pack.lon_int_GET() == -1828848880);
            assert(pack.yaw_GET() == 3.957748E37F);
            assert(pack.time_boot_ms_GET() == 798266575L);
            assert(pack.target_system_GET() == (char)20);
            assert(pack.vx_GET() == -3.2441453E38F);
            assert(pack.vy_GET() == -1.4229602E38F);
            assert(pack.afy_GET() == 1.4570422E38F);
            assert(pack.lat_int_GET() == 788776896);
            assert(pack.vz_GET() == -8.2880137E37F);
            assert(pack.alt_GET() == 2.0568801E38F);
            assert(pack.afx_GET() == -1.1382862E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.afz_GET() == 1.459877E38F);
            assert(pack.type_mask_GET() == (char)60971);
            assert(pack.target_component_GET() == (char)49);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.alt_SET(2.0568801E38F) ;
        p86.afy_SET(1.4570422E38F) ;
        p86.vx_SET(-3.2441453E38F) ;
        p86.lat_int_SET(788776896) ;
        p86.target_component_SET((char)49) ;
        p86.lon_int_SET(-1828848880) ;
        p86.yaw_SET(3.957748E37F) ;
        p86.afx_SET(-1.1382862E38F) ;
        p86.vy_SET(-1.4229602E38F) ;
        p86.time_boot_ms_SET(798266575L) ;
        p86.target_system_SET((char)20) ;
        p86.type_mask_SET((char)60971) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p86.afz_SET(1.459877E38F) ;
        p86.yaw_rate_SET(-2.2245157E38F) ;
        p86.vz_SET(-8.2880137E37F) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.afx_GET() == -3.2414163E38F);
            assert(pack.vy_GET() == 1.1672486E38F);
            assert(pack.type_mask_GET() == (char)8421);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.lat_int_GET() == 757100300);
            assert(pack.vz_GET() == -2.4163706E38F);
            assert(pack.yaw_GET() == 1.5571981E37F);
            assert(pack.alt_GET() == -1.1876038E38F);
            assert(pack.lon_int_GET() == 687611895);
            assert(pack.time_boot_ms_GET() == 554555303L);
            assert(pack.afy_GET() == -6.683528E37F);
            assert(pack.vx_GET() == -2.501436E38F);
            assert(pack.yaw_rate_GET() == -2.2904369E36F);
            assert(pack.afz_GET() == -6.531044E37F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(554555303L) ;
        p87.afx_SET(-3.2414163E38F) ;
        p87.vx_SET(-2.501436E38F) ;
        p87.afz_SET(-6.531044E37F) ;
        p87.afy_SET(-6.683528E37F) ;
        p87.vz_SET(-2.4163706E38F) ;
        p87.vy_SET(1.1672486E38F) ;
        p87.lon_int_SET(687611895) ;
        p87.alt_SET(-1.1876038E38F) ;
        p87.yaw_rate_SET(-2.2904369E36F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p87.lat_int_SET(757100300) ;
        p87.type_mask_SET((char)8421) ;
        p87.yaw_SET(1.5571981E37F) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -2.2693092E38F);
            assert(pack.z_GET() == -9.120249E37F);
            assert(pack.yaw_GET() == 2.9869002E38F);
            assert(pack.pitch_GET() == 2.310944E38F);
            assert(pack.x_GET() == -9.38188E37F);
            assert(pack.time_boot_ms_GET() == 2042669150L);
            assert(pack.y_GET() == 9.803509E37F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.pitch_SET(2.310944E38F) ;
        p89.yaw_SET(2.9869002E38F) ;
        p89.y_SET(9.803509E37F) ;
        p89.roll_SET(-2.2693092E38F) ;
        p89.x_SET(-9.38188E37F) ;
        p89.z_SET(-9.120249E37F) ;
        p89.time_boot_ms_SET(2042669150L) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -137722635);
            assert(pack.time_usec_GET() == 646201278968139668L);
            assert(pack.roll_GET() == 2.6349002E38F);
            assert(pack.vy_GET() == (short)15207);
            assert(pack.vz_GET() == (short)16544);
            assert(pack.vx_GET() == (short) -14519);
            assert(pack.yaw_GET() == 1.6565742E38F);
            assert(pack.pitchspeed_GET() == 7.387085E37F);
            assert(pack.yawspeed_GET() == -2.6831508E38F);
            assert(pack.yacc_GET() == (short) -7144);
            assert(pack.zacc_GET() == (short) -12020);
            assert(pack.lon_GET() == -991257310);
            assert(pack.lat_GET() == 1830593537);
            assert(pack.xacc_GET() == (short)3448);
            assert(pack.rollspeed_GET() == 2.7121518E38F);
            assert(pack.pitch_GET() == 2.1116004E38F);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.xacc_SET((short)3448) ;
        p90.yacc_SET((short) -7144) ;
        p90.rollspeed_SET(2.7121518E38F) ;
        p90.pitchspeed_SET(7.387085E37F) ;
        p90.lat_SET(1830593537) ;
        p90.roll_SET(2.6349002E38F) ;
        p90.vz_SET((short)16544) ;
        p90.time_usec_SET(646201278968139668L) ;
        p90.lon_SET(-991257310) ;
        p90.yawspeed_SET(-2.6831508E38F) ;
        p90.zacc_SET((short) -12020) ;
        p90.vy_SET((short)15207) ;
        p90.vx_SET((short) -14519) ;
        p90.yaw_SET(1.6565742E38F) ;
        p90.alt_SET(-137722635) ;
        p90.pitch_SET(2.1116004E38F) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux3_GET() == -2.098724E38F);
            assert(pack.time_usec_GET() == 5540506886801696349L);
            assert(pack.nav_mode_GET() == (char)150);
            assert(pack.throttle_GET() == 1.2578321E38F);
            assert(pack.yaw_rudder_GET() == -1.0587549E38F);
            assert(pack.aux1_GET() == 3.255548E38F);
            assert(pack.roll_ailerons_GET() == -1.9851437E38F);
            assert(pack.aux4_GET() == 1.9027861E38F);
            assert(pack.aux2_GET() == -7.461186E37F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_MANUAL_DISARMED);
            assert(pack.pitch_elevator_GET() == 2.4824406E38F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.roll_ailerons_SET(-1.9851437E38F) ;
        p91.aux4_SET(1.9027861E38F) ;
        p91.pitch_elevator_SET(2.4824406E38F) ;
        p91.throttle_SET(1.2578321E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_MANUAL_DISARMED) ;
        p91.aux1_SET(3.255548E38F) ;
        p91.aux2_SET(-7.461186E37F) ;
        p91.nav_mode_SET((char)150) ;
        p91.yaw_rudder_SET(-1.0587549E38F) ;
        p91.time_usec_SET(5540506886801696349L) ;
        p91.aux3_SET(-2.098724E38F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan12_raw_GET() == (char)42441);
            assert(pack.chan6_raw_GET() == (char)56279);
            assert(pack.chan4_raw_GET() == (char)28628);
            assert(pack.chan9_raw_GET() == (char)54051);
            assert(pack.chan5_raw_GET() == (char)51671);
            assert(pack.time_usec_GET() == 636376915890824691L);
            assert(pack.chan3_raw_GET() == (char)59315);
            assert(pack.chan2_raw_GET() == (char)51983);
            assert(pack.chan7_raw_GET() == (char)56306);
            assert(pack.chan1_raw_GET() == (char)52156);
            assert(pack.rssi_GET() == (char)90);
            assert(pack.chan11_raw_GET() == (char)53519);
            assert(pack.chan10_raw_GET() == (char)57177);
            assert(pack.chan8_raw_GET() == (char)38227);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan8_raw_SET((char)38227) ;
        p92.chan10_raw_SET((char)57177) ;
        p92.rssi_SET((char)90) ;
        p92.chan9_raw_SET((char)54051) ;
        p92.chan12_raw_SET((char)42441) ;
        p92.chan6_raw_SET((char)56279) ;
        p92.chan3_raw_SET((char)59315) ;
        p92.chan1_raw_SET((char)52156) ;
        p92.chan4_raw_SET((char)28628) ;
        p92.chan7_raw_SET((char)56306) ;
        p92.chan11_raw_SET((char)53519) ;
        p92.time_usec_SET(636376915890824691L) ;
        p92.chan5_raw_SET((char)51671) ;
        p92.chan2_raw_SET((char)51983) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4635493529075838348L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-3.3751043E38F, -5.668127E37F, -9.850763E37F, -8.003804E37F, -3.1760844E38F, 3.3720784E38F, 2.6938865E38F, 1.1431599E38F, 6.193453E37F, 3.2237464E38F, 7.2542223E37F, -1.6967012E38F, -3.4556962E37F, 1.4198028E38F, 2.4959487E38F, 2.212555E38F}));
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_DISARMED);
            assert(pack.flags_GET() == 9127664972410364253L);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.flags_SET(9127664972410364253L) ;
        p93.time_usec_SET(4635493529075838348L) ;
        p93.controls_SET(new float[] {-3.3751043E38F, -5.668127E37F, -9.850763E37F, -8.003804E37F, -3.1760844E38F, 3.3720784E38F, 2.6938865E38F, 1.1431599E38F, 6.193453E37F, 3.2237464E38F, 7.2542223E37F, -1.6967012E38F, -3.4556962E37F, 1.4198028E38F, 2.4959487E38F, 2.212555E38F}, 0) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_y_GET() == (short)8830);
            assert(pack.flow_comp_m_x_GET() == -2.6932182E38F);
            assert(pack.flow_rate_y_TRY(ph) == -2.881315E38F);
            assert(pack.sensor_id_GET() == (char)178);
            assert(pack.flow_x_GET() == (short)7);
            assert(pack.flow_comp_m_y_GET() == 1.5414979E37F);
            assert(pack.flow_rate_x_TRY(ph) == 3.3580202E38F);
            assert(pack.quality_GET() == (char)248);
            assert(pack.time_usec_GET() == 7035960236840174262L);
            assert(pack.ground_distance_GET() == -1.099822E38F);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.ground_distance_SET(-1.099822E38F) ;
        p100.flow_rate_y_SET(-2.881315E38F, PH) ;
        p100.flow_x_SET((short)7) ;
        p100.flow_comp_m_y_SET(1.5414979E37F) ;
        p100.time_usec_SET(7035960236840174262L) ;
        p100.flow_comp_m_x_SET(-2.6932182E38F) ;
        p100.flow_rate_x_SET(3.3580202E38F, PH) ;
        p100.sensor_id_SET((char)178) ;
        p100.flow_y_SET((short)8830) ;
        p100.quality_SET((char)248) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -3.0810266E38F);
            assert(pack.roll_GET() == 2.2485629E38F);
            assert(pack.x_GET() == -1.5512898E38F);
            assert(pack.pitch_GET() == -7.3509243E37F);
            assert(pack.usec_GET() == 1613573134859942213L);
            assert(pack.yaw_GET() == -4.9494743E37F);
            assert(pack.y_GET() == 1.5693593E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.x_SET(-1.5512898E38F) ;
        p101.pitch_SET(-7.3509243E37F) ;
        p101.y_SET(1.5693593E38F) ;
        p101.usec_SET(1613573134859942213L) ;
        p101.yaw_SET(-4.9494743E37F) ;
        p101.z_SET(-3.0810266E38F) ;
        p101.roll_SET(2.2485629E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -1.814871E38F);
            assert(pack.roll_GET() == -1.7535011E38F);
            assert(pack.usec_GET() == 8860461373953399274L);
            assert(pack.x_GET() == 2.1332568E38F);
            assert(pack.y_GET() == -2.0294735E38F);
            assert(pack.z_GET() == -1.324981E38F);
            assert(pack.pitch_GET() == -2.8509281E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.roll_SET(-1.7535011E38F) ;
        p102.usec_SET(8860461373953399274L) ;
        p102.pitch_SET(-2.8509281E38F) ;
        p102.x_SET(2.1332568E38F) ;
        p102.yaw_SET(-1.814871E38F) ;
        p102.y_SET(-2.0294735E38F) ;
        p102.z_SET(-1.324981E38F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 1.7315509E38F);
            assert(pack.x_GET() == 1.506398E37F);
            assert(pack.z_GET() == -5.4450177E37F);
            assert(pack.usec_GET() == 1991150978522557992L);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.z_SET(-5.4450177E37F) ;
        p103.x_SET(1.506398E37F) ;
        p103.y_SET(1.7315509E38F) ;
        p103.usec_SET(1991150978522557992L) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 2.3744114E38F);
            assert(pack.usec_GET() == 9063732357730849138L);
            assert(pack.pitch_GET() == 2.726104E38F);
            assert(pack.z_GET() == -6.9898274E37F);
            assert(pack.roll_GET() == -2.9921329E38F);
            assert(pack.y_GET() == -2.5824718E38F);
            assert(pack.x_GET() == 3.0853242E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.pitch_SET(2.726104E38F) ;
        p104.y_SET(-2.5824718E38F) ;
        p104.x_SET(3.0853242E38F) ;
        p104.roll_SET(-2.9921329E38F) ;
        p104.yaw_SET(2.3744114E38F) ;
        p104.usec_SET(9063732357730849138L) ;
        p104.z_SET(-6.9898274E37F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == 9.281746E37F);
            assert(pack.xacc_GET() == 3.3725572E38F);
            assert(pack.xmag_GET() == 1.1199242E38F);
            assert(pack.time_usec_GET() == 3140786929988995878L);
            assert(pack.fields_updated_GET() == (char)40022);
            assert(pack.ymag_GET() == 9.723253E37F);
            assert(pack.diff_pressure_GET() == -3.055459E38F);
            assert(pack.yacc_GET() == 1.312606E38F);
            assert(pack.zgyro_GET() == 3.9033725E37F);
            assert(pack.ygyro_GET() == -8.814621E37F);
            assert(pack.zmag_GET() == -2.7117395E38F);
            assert(pack.pressure_alt_GET() == -1.0737353E37F);
            assert(pack.abs_pressure_GET() == 1.5967827E38F);
            assert(pack.temperature_GET() == -1.9342208E38F);
            assert(pack.xgyro_GET() == -2.5841277E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.zmag_SET(-2.7117395E38F) ;
        p105.temperature_SET(-1.9342208E38F) ;
        p105.zacc_SET(9.281746E37F) ;
        p105.xmag_SET(1.1199242E38F) ;
        p105.xacc_SET(3.3725572E38F) ;
        p105.ymag_SET(9.723253E37F) ;
        p105.diff_pressure_SET(-3.055459E38F) ;
        p105.abs_pressure_SET(1.5967827E38F) ;
        p105.fields_updated_SET((char)40022) ;
        p105.xgyro_SET(-2.5841277E38F) ;
        p105.ygyro_SET(-8.814621E37F) ;
        p105.time_usec_SET(3140786929988995878L) ;
        p105.zgyro_SET(3.9033725E37F) ;
        p105.yacc_SET(1.312606E38F) ;
        p105.pressure_alt_SET(-1.0737353E37F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.time_delta_distance_us_GET() == 970055696L);
            assert(pack.quality_GET() == (char)156);
            assert(pack.integrated_x_GET() == -2.5602642E38F);
            assert(pack.time_usec_GET() == 6485078592801863270L);
            assert(pack.integrated_zgyro_GET() == -1.9240218E38F);
            assert(pack.temperature_GET() == (short)28422);
            assert(pack.integrated_y_GET() == 7.250934E37F);
            assert(pack.distance_GET() == -1.7981115E38F);
            assert(pack.integrated_ygyro_GET() == 6.218591E36F);
            assert(pack.sensor_id_GET() == (char)51);
            assert(pack.integration_time_us_GET() == 1654710371L);
            assert(pack.integrated_xgyro_GET() == -2.3523748E38F);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_y_SET(7.250934E37F) ;
        p106.integrated_zgyro_SET(-1.9240218E38F) ;
        p106.integration_time_us_SET(1654710371L) ;
        p106.integrated_xgyro_SET(-2.3523748E38F) ;
        p106.time_usec_SET(6485078592801863270L) ;
        p106.time_delta_distance_us_SET(970055696L) ;
        p106.temperature_SET((short)28422) ;
        p106.integrated_ygyro_SET(6.218591E36F) ;
        p106.distance_SET(-1.7981115E38F) ;
        p106.sensor_id_SET((char)51) ;
        p106.quality_SET((char)156) ;
        p106.integrated_x_SET(-2.5602642E38F) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == -2.6867857E38F);
            assert(pack.zacc_GET() == 2.4901903E38F);
            assert(pack.xmag_GET() == 3.7263738E37F);
            assert(pack.ygyro_GET() == 1.3582833E38F);
            assert(pack.pressure_alt_GET() == -2.1477731E38F);
            assert(pack.zgyro_GET() == 2.479429E38F);
            assert(pack.time_usec_GET() == 283980214650561306L);
            assert(pack.xacc_GET() == -1.3800133E38F);
            assert(pack.diff_pressure_GET() == -2.894675E38F);
            assert(pack.xgyro_GET() == 2.3756895E37F);
            assert(pack.abs_pressure_GET() == -3.3283091E38F);
            assert(pack.yacc_GET() == -3.306262E38F);
            assert(pack.temperature_GET() == -2.0565128E38F);
            assert(pack.fields_updated_GET() == 81038408L);
            assert(pack.ymag_GET() == 7.0995304E37F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.temperature_SET(-2.0565128E38F) ;
        p107.yacc_SET(-3.306262E38F) ;
        p107.xgyro_SET(2.3756895E37F) ;
        p107.ymag_SET(7.0995304E37F) ;
        p107.pressure_alt_SET(-2.1477731E38F) ;
        p107.diff_pressure_SET(-2.894675E38F) ;
        p107.zgyro_SET(2.479429E38F) ;
        p107.zmag_SET(-2.6867857E38F) ;
        p107.xmag_SET(3.7263738E37F) ;
        p107.abs_pressure_SET(-3.3283091E38F) ;
        p107.zacc_SET(2.4901903E38F) ;
        p107.fields_updated_SET(81038408L) ;
        p107.ygyro_SET(1.3582833E38F) ;
        p107.time_usec_SET(283980214650561306L) ;
        p107.xacc_SET(-1.3800133E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 7.8330286E36F);
            assert(pack.vn_GET() == -1.5918141E38F);
            assert(pack.ve_GET() == -3.7385762E37F);
            assert(pack.yacc_GET() == 3.1475036E38F);
            assert(pack.q2_GET() == -2.253835E38F);
            assert(pack.q1_GET() == -1.0789184E38F);
            assert(pack.std_dev_horz_GET() == 2.2751462E38F);
            assert(pack.q4_GET() == -2.2899727E38F);
            assert(pack.xacc_GET() == 7.0821433E37F);
            assert(pack.lon_GET() == 9.9198027E36F);
            assert(pack.std_dev_vert_GET() == -1.9464665E38F);
            assert(pack.zacc_GET() == -6.116757E37F);
            assert(pack.lat_GET() == 2.1537538E38F);
            assert(pack.vd_GET() == -1.7388613E37F);
            assert(pack.roll_GET() == 1.7770908E38F);
            assert(pack.ygyro_GET() == 2.2193434E38F);
            assert(pack.q3_GET() == -1.7116605E38F);
            assert(pack.zgyro_GET() == 9.418607E37F);
            assert(pack.alt_GET() == -1.8295107E38F);
            assert(pack.yaw_GET() == 9.06229E36F);
            assert(pack.xgyro_GET() == 6.877158E37F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.q2_SET(-2.253835E38F) ;
        p108.roll_SET(1.7770908E38F) ;
        p108.std_dev_vert_SET(-1.9464665E38F) ;
        p108.xgyro_SET(6.877158E37F) ;
        p108.xacc_SET(7.0821433E37F) ;
        p108.yaw_SET(9.06229E36F) ;
        p108.vn_SET(-1.5918141E38F) ;
        p108.vd_SET(-1.7388613E37F) ;
        p108.zgyro_SET(9.418607E37F) ;
        p108.lat_SET(2.1537538E38F) ;
        p108.pitch_SET(7.8330286E36F) ;
        p108.q3_SET(-1.7116605E38F) ;
        p108.yacc_SET(3.1475036E38F) ;
        p108.std_dev_horz_SET(2.2751462E38F) ;
        p108.alt_SET(-1.8295107E38F) ;
        p108.q4_SET(-2.2899727E38F) ;
        p108.zacc_SET(-6.116757E37F) ;
        p108.q1_SET(-1.0789184E38F) ;
        p108.ygyro_SET(2.2193434E38F) ;
        p108.ve_SET(-3.7385762E37F) ;
        p108.lon_SET(9.9198027E36F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.rxerrors_GET() == (char)17799);
            assert(pack.fixed__GET() == (char)64039);
            assert(pack.rssi_GET() == (char)234);
            assert(pack.remrssi_GET() == (char)154);
            assert(pack.noise_GET() == (char)180);
            assert(pack.txbuf_GET() == (char)50);
            assert(pack.remnoise_GET() == (char)201);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rxerrors_SET((char)17799) ;
        p109.txbuf_SET((char)50) ;
        p109.remrssi_SET((char)154) ;
        p109.remnoise_SET((char)201) ;
        p109.rssi_SET((char)234) ;
        p109.noise_SET((char)180) ;
        p109.fixed__SET((char)64039) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)219, (char)64, (char)36, (char)72, (char)202, (char)34, (char)154, (char)239, (char)119, (char)80, (char)241, (char)107, (char)109, (char)148, (char)76, (char)183, (char)170, (char)22, (char)231, (char)12, (char)29, (char)3, (char)225, (char)27, (char)124, (char)122, (char)188, (char)78, (char)136, (char)195, (char)6, (char)136, (char)50, (char)85, (char)74, (char)156, (char)25, (char)95, (char)52, (char)25, (char)109, (char)226, (char)226, (char)2, (char)130, (char)173, (char)235, (char)182, (char)54, (char)111, (char)146, (char)125, (char)253, (char)206, (char)29, (char)0, (char)237, (char)242, (char)86, (char)154, (char)133, (char)236, (char)20, (char)96, (char)231, (char)242, (char)72, (char)201, (char)75, (char)33, (char)212, (char)150, (char)88, (char)52, (char)248, (char)76, (char)191, (char)204, (char)118, (char)112, (char)86, (char)245, (char)58, (char)193, (char)208, (char)243, (char)141, (char)226, (char)143, (char)250, (char)171, (char)198, (char)1, (char)209, (char)47, (char)36, (char)127, (char)72, (char)34, (char)48, (char)9, (char)112, (char)169, (char)1, (char)27, (char)221, (char)218, (char)144, (char)186, (char)113, (char)79, (char)112, (char)251, (char)98, (char)151, (char)213, (char)142, (char)199, (char)230, (char)232, (char)167, (char)144, (char)218, (char)55, (char)75, (char)115, (char)84, (char)63, (char)158, (char)13, (char)66, (char)250, (char)181, (char)61, (char)184, (char)114, (char)142, (char)111, (char)135, (char)236, (char)181, (char)239, (char)125, (char)0, (char)252, (char)63, (char)251, (char)243, (char)224, (char)126, (char)177, (char)38, (char)129, (char)238, (char)106, (char)106, (char)112, (char)153, (char)86, (char)37, (char)198, (char)68, (char)112, (char)147, (char)222, (char)252, (char)67, (char)242, (char)93, (char)57, (char)34, (char)237, (char)35, (char)131, (char)25, (char)10, (char)12, (char)92, (char)104, (char)113, (char)115, (char)25, (char)182, (char)208, (char)167, (char)228, (char)68, (char)170, (char)40, (char)101, (char)11, (char)96, (char)184, (char)176, (char)52, (char)48, (char)189, (char)87, (char)66, (char)46, (char)147, (char)170, (char)232, (char)91, (char)133, (char)108, (char)250, (char)149, (char)151, (char)4, (char)75, (char)162, (char)214, (char)168, (char)193, (char)73, (char)170, (char)151, (char)82, (char)153, (char)73, (char)121, (char)203, (char)248, (char)70, (char)154, (char)52, (char)11, (char)92, (char)144, (char)169, (char)2, (char)208, (char)100, (char)27, (char)112, (char)162, (char)238, (char)231, (char)25, (char)181, (char)142, (char)231, (char)227, (char)218, (char)148, (char)46, (char)12, (char)162, (char)101, (char)72}));
            assert(pack.target_component_GET() == (char)121);
            assert(pack.target_network_GET() == (char)223);
            assert(pack.target_system_GET() == (char)33);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_system_SET((char)33) ;
        p110.payload_SET(new char[] {(char)219, (char)64, (char)36, (char)72, (char)202, (char)34, (char)154, (char)239, (char)119, (char)80, (char)241, (char)107, (char)109, (char)148, (char)76, (char)183, (char)170, (char)22, (char)231, (char)12, (char)29, (char)3, (char)225, (char)27, (char)124, (char)122, (char)188, (char)78, (char)136, (char)195, (char)6, (char)136, (char)50, (char)85, (char)74, (char)156, (char)25, (char)95, (char)52, (char)25, (char)109, (char)226, (char)226, (char)2, (char)130, (char)173, (char)235, (char)182, (char)54, (char)111, (char)146, (char)125, (char)253, (char)206, (char)29, (char)0, (char)237, (char)242, (char)86, (char)154, (char)133, (char)236, (char)20, (char)96, (char)231, (char)242, (char)72, (char)201, (char)75, (char)33, (char)212, (char)150, (char)88, (char)52, (char)248, (char)76, (char)191, (char)204, (char)118, (char)112, (char)86, (char)245, (char)58, (char)193, (char)208, (char)243, (char)141, (char)226, (char)143, (char)250, (char)171, (char)198, (char)1, (char)209, (char)47, (char)36, (char)127, (char)72, (char)34, (char)48, (char)9, (char)112, (char)169, (char)1, (char)27, (char)221, (char)218, (char)144, (char)186, (char)113, (char)79, (char)112, (char)251, (char)98, (char)151, (char)213, (char)142, (char)199, (char)230, (char)232, (char)167, (char)144, (char)218, (char)55, (char)75, (char)115, (char)84, (char)63, (char)158, (char)13, (char)66, (char)250, (char)181, (char)61, (char)184, (char)114, (char)142, (char)111, (char)135, (char)236, (char)181, (char)239, (char)125, (char)0, (char)252, (char)63, (char)251, (char)243, (char)224, (char)126, (char)177, (char)38, (char)129, (char)238, (char)106, (char)106, (char)112, (char)153, (char)86, (char)37, (char)198, (char)68, (char)112, (char)147, (char)222, (char)252, (char)67, (char)242, (char)93, (char)57, (char)34, (char)237, (char)35, (char)131, (char)25, (char)10, (char)12, (char)92, (char)104, (char)113, (char)115, (char)25, (char)182, (char)208, (char)167, (char)228, (char)68, (char)170, (char)40, (char)101, (char)11, (char)96, (char)184, (char)176, (char)52, (char)48, (char)189, (char)87, (char)66, (char)46, (char)147, (char)170, (char)232, (char)91, (char)133, (char)108, (char)250, (char)149, (char)151, (char)4, (char)75, (char)162, (char)214, (char)168, (char)193, (char)73, (char)170, (char)151, (char)82, (char)153, (char)73, (char)121, (char)203, (char)248, (char)70, (char)154, (char)52, (char)11, (char)92, (char)144, (char)169, (char)2, (char)208, (char)100, (char)27, (char)112, (char)162, (char)238, (char)231, (char)25, (char)181, (char)142, (char)231, (char)227, (char)218, (char)148, (char)46, (char)12, (char)162, (char)101, (char)72}, 0) ;
        p110.target_network_SET((char)223) ;
        p110.target_component_SET((char)121) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == 2710948293073803901L);
            assert(pack.tc1_GET() == 7381090142364432512L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(2710948293073803901L) ;
        p111.tc1_SET(7381090142364432512L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 3096803975L);
            assert(pack.time_usec_GET() == 6733393960425072477L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(3096803975L) ;
        p112.time_usec_SET(6733393960425072477L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.vn_GET() == (short)13148);
            assert(pack.cog_GET() == (char)15485);
            assert(pack.fix_type_GET() == (char)148);
            assert(pack.vel_GET() == (char)31679);
            assert(pack.ve_GET() == (short)17699);
            assert(pack.lat_GET() == -1194633934);
            assert(pack.alt_GET() == 787007018);
            assert(pack.eph_GET() == (char)40200);
            assert(pack.epv_GET() == (char)31019);
            assert(pack.satellites_visible_GET() == (char)226);
            assert(pack.lon_GET() == 569707253);
            assert(pack.vd_GET() == (short) -9753);
            assert(pack.time_usec_GET() == 3151668781799054794L);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.vd_SET((short) -9753) ;
        p113.satellites_visible_SET((char)226) ;
        p113.lat_SET(-1194633934) ;
        p113.alt_SET(787007018) ;
        p113.vel_SET((char)31679) ;
        p113.eph_SET((char)40200) ;
        p113.lon_SET(569707253) ;
        p113.epv_SET((char)31019) ;
        p113.fix_type_SET((char)148) ;
        p113.cog_SET((char)15485) ;
        p113.vn_SET((short)13148) ;
        p113.ve_SET((short)17699) ;
        p113.time_usec_SET(3151668781799054794L) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.distance_GET() == -3.2314789E38F);
            assert(pack.time_usec_GET() == 2386643516164435454L);
            assert(pack.quality_GET() == (char)83);
            assert(pack.sensor_id_GET() == (char)130);
            assert(pack.integration_time_us_GET() == 1509774889L);
            assert(pack.integrated_y_GET() == -2.4194688E38F);
            assert(pack.integrated_x_GET() == 1.874751E38F);
            assert(pack.integrated_zgyro_GET() == -2.0523186E38F);
            assert(pack.time_delta_distance_us_GET() == 3945278837L);
            assert(pack.integrated_xgyro_GET() == 8.987626E37F);
            assert(pack.temperature_GET() == (short)14281);
            assert(pack.integrated_ygyro_GET() == -1.5899574E37F);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_xgyro_SET(8.987626E37F) ;
        p114.integrated_zgyro_SET(-2.0523186E38F) ;
        p114.time_delta_distance_us_SET(3945278837L) ;
        p114.quality_SET((char)83) ;
        p114.integrated_ygyro_SET(-1.5899574E37F) ;
        p114.time_usec_SET(2386643516164435454L) ;
        p114.integration_time_us_SET(1509774889L) ;
        p114.sensor_id_SET((char)130) ;
        p114.integrated_y_SET(-2.4194688E38F) ;
        p114.distance_SET(-3.2314789E38F) ;
        p114.temperature_SET((short)14281) ;
        p114.integrated_x_SET(1.874751E38F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 1203222625);
            assert(pack.vx_GET() == (short)19852);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {-3.0585933E38F, 1.2470962E38F, 5.023833E37F, -1.1549924E38F}));
            assert(pack.vz_GET() == (short) -25886);
            assert(pack.true_airspeed_GET() == (char)17943);
            assert(pack.yawspeed_GET() == 4.307377E37F);
            assert(pack.lat_GET() == 1660003691);
            assert(pack.yacc_GET() == (short) -28475);
            assert(pack.alt_GET() == 269685959);
            assert(pack.time_usec_GET() == 1858540771222539197L);
            assert(pack.vy_GET() == (short) -3679);
            assert(pack.rollspeed_GET() == -3.172421E38F);
            assert(pack.pitchspeed_GET() == -2.188493E38F);
            assert(pack.ind_airspeed_GET() == (char)25991);
            assert(pack.xacc_GET() == (short) -819);
            assert(pack.zacc_GET() == (short)14392);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.vx_SET((short)19852) ;
        p115.lat_SET(1660003691) ;
        p115.vz_SET((short) -25886) ;
        p115.attitude_quaternion_SET(new float[] {-3.0585933E38F, 1.2470962E38F, 5.023833E37F, -1.1549924E38F}, 0) ;
        p115.lon_SET(1203222625) ;
        p115.true_airspeed_SET((char)17943) ;
        p115.zacc_SET((short)14392) ;
        p115.yawspeed_SET(4.307377E37F) ;
        p115.time_usec_SET(1858540771222539197L) ;
        p115.xacc_SET((short) -819) ;
        p115.rollspeed_SET(-3.172421E38F) ;
        p115.pitchspeed_SET(-2.188493E38F) ;
        p115.alt_SET(269685959) ;
        p115.ind_airspeed_SET((char)25991) ;
        p115.yacc_SET((short) -28475) ;
        p115.vy_SET((short) -3679) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short) -30103);
            assert(pack.ymag_GET() == (short) -28480);
            assert(pack.zmag_GET() == (short)16351);
            assert(pack.time_boot_ms_GET() == 2642501837L);
            assert(pack.xgyro_GET() == (short) -22639);
            assert(pack.xmag_GET() == (short) -30594);
            assert(pack.zgyro_GET() == (short) -29965);
            assert(pack.zacc_GET() == (short)22647);
            assert(pack.ygyro_GET() == (short)21450);
            assert(pack.yacc_GET() == (short) -2373);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.yacc_SET((short) -2373) ;
        p116.xgyro_SET((short) -22639) ;
        p116.ygyro_SET((short)21450) ;
        p116.xacc_SET((short) -30103) ;
        p116.zmag_SET((short)16351) ;
        p116.time_boot_ms_SET(2642501837L) ;
        p116.zgyro_SET((short) -29965) ;
        p116.zacc_SET((short)22647) ;
        p116.ymag_SET((short) -28480) ;
        p116.xmag_SET((short) -30594) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_GET() == (char)4779);
            assert(pack.target_component_GET() == (char)123);
            assert(pack.start_GET() == (char)38834);
            assert(pack.target_system_GET() == (char)217);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)217) ;
        p117.end_SET((char)4779) ;
        p117.target_component_SET((char)123) ;
        p117.start_SET((char)38834) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.last_log_num_GET() == (char)57161);
            assert(pack.size_GET() == 3203534947L);
            assert(pack.time_utc_GET() == 3953915728L);
            assert(pack.id_GET() == (char)51506);
            assert(pack.num_logs_GET() == (char)33348);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)51506) ;
        p118.size_SET(3203534947L) ;
        p118.last_log_num_SET((char)57161) ;
        p118.time_utc_SET(3953915728L) ;
        p118.num_logs_SET((char)33348) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)214);
            assert(pack.ofs_GET() == 4178182722L);
            assert(pack.count_GET() == 3438275935L);
            assert(pack.target_component_GET() == (char)92);
            assert(pack.id_GET() == (char)28103);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)214) ;
        p119.id_SET((char)28103) ;
        p119.count_SET(3438275935L) ;
        p119.target_component_SET((char)92) ;
        p119.ofs_SET(4178182722L) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.ofs_GET() == 3650511362L);
            assert(pack.id_GET() == (char)18814);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)109, (char)246, (char)97, (char)3, (char)247, (char)32, (char)61, (char)232, (char)56, (char)189, (char)85, (char)147, (char)150, (char)179, (char)222, (char)203, (char)218, (char)196, (char)37, (char)33, (char)197, (char)154, (char)92, (char)218, (char)191, (char)117, (char)84, (char)233, (char)135, (char)18, (char)162, (char)164, (char)88, (char)63, (char)95, (char)228, (char)93, (char)75, (char)89, (char)50, (char)12, (char)154, (char)89, (char)181, (char)31, (char)133, (char)25, (char)123, (char)212, (char)126, (char)6, (char)100, (char)147, (char)98, (char)10, (char)20, (char)230, (char)147, (char)76, (char)239, (char)131, (char)210, (char)13, (char)130, (char)99, (char)141, (char)217, (char)91, (char)138, (char)43, (char)179, (char)223, (char)85, (char)227, (char)229, (char)5, (char)44, (char)108, (char)213, (char)206, (char)151, (char)18, (char)10, (char)94, (char)154, (char)112, (char)105, (char)60, (char)103, (char)216}));
            assert(pack.count_GET() == (char)97);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.count_SET((char)97) ;
        p120.id_SET((char)18814) ;
        p120.data__SET(new char[] {(char)109, (char)246, (char)97, (char)3, (char)247, (char)32, (char)61, (char)232, (char)56, (char)189, (char)85, (char)147, (char)150, (char)179, (char)222, (char)203, (char)218, (char)196, (char)37, (char)33, (char)197, (char)154, (char)92, (char)218, (char)191, (char)117, (char)84, (char)233, (char)135, (char)18, (char)162, (char)164, (char)88, (char)63, (char)95, (char)228, (char)93, (char)75, (char)89, (char)50, (char)12, (char)154, (char)89, (char)181, (char)31, (char)133, (char)25, (char)123, (char)212, (char)126, (char)6, (char)100, (char)147, (char)98, (char)10, (char)20, (char)230, (char)147, (char)76, (char)239, (char)131, (char)210, (char)13, (char)130, (char)99, (char)141, (char)217, (char)91, (char)138, (char)43, (char)179, (char)223, (char)85, (char)227, (char)229, (char)5, (char)44, (char)108, (char)213, (char)206, (char)151, (char)18, (char)10, (char)94, (char)154, (char)112, (char)105, (char)60, (char)103, (char)216}, 0) ;
        p120.ofs_SET(3650511362L) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)187);
            assert(pack.target_component_GET() == (char)207);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)187) ;
        p121.target_component_SET((char)207) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)135);
            assert(pack.target_system_GET() == (char)52);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)52) ;
        p122.target_component_SET((char)135) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)188);
            assert(pack.len_GET() == (char)201);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)94, (char)101, (char)209, (char)242, (char)149, (char)24, (char)122, (char)150, (char)137, (char)239, (char)72, (char)166, (char)193, (char)247, (char)161, (char)238, (char)81, (char)191, (char)26, (char)86, (char)144, (char)94, (char)124, (char)163, (char)255, (char)151, (char)119, (char)100, (char)54, (char)137, (char)241, (char)252, (char)215, (char)92, (char)230, (char)254, (char)46, (char)92, (char)154, (char)17, (char)232, (char)90, (char)222, (char)190, (char)29, (char)248, (char)41, (char)208, (char)11, (char)236, (char)136, (char)3, (char)75, (char)10, (char)223, (char)148, (char)53, (char)6, (char)96, (char)255, (char)190, (char)153, (char)54, (char)204, (char)207, (char)83, (char)78, (char)104, (char)173, (char)12, (char)77, (char)188, (char)173, (char)7, (char)70, (char)178, (char)220, (char)146, (char)241, (char)16, (char)142, (char)2, (char)79, (char)249, (char)122, (char)64, (char)236, (char)50, (char)15, (char)192, (char)15, (char)213, (char)3, (char)5, (char)237, (char)217, (char)119, (char)159, (char)229, (char)112, (char)238, (char)148, (char)82, (char)32, (char)56, (char)183, (char)198, (char)152, (char)85, (char)220}));
            assert(pack.target_system_GET() == (char)245);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)245) ;
        p123.target_component_SET((char)188) ;
        p123.len_SET((char)201) ;
        p123.data__SET(new char[] {(char)94, (char)101, (char)209, (char)242, (char)149, (char)24, (char)122, (char)150, (char)137, (char)239, (char)72, (char)166, (char)193, (char)247, (char)161, (char)238, (char)81, (char)191, (char)26, (char)86, (char)144, (char)94, (char)124, (char)163, (char)255, (char)151, (char)119, (char)100, (char)54, (char)137, (char)241, (char)252, (char)215, (char)92, (char)230, (char)254, (char)46, (char)92, (char)154, (char)17, (char)232, (char)90, (char)222, (char)190, (char)29, (char)248, (char)41, (char)208, (char)11, (char)236, (char)136, (char)3, (char)75, (char)10, (char)223, (char)148, (char)53, (char)6, (char)96, (char)255, (char)190, (char)153, (char)54, (char)204, (char)207, (char)83, (char)78, (char)104, (char)173, (char)12, (char)77, (char)188, (char)173, (char)7, (char)70, (char)178, (char)220, (char)146, (char)241, (char)16, (char)142, (char)2, (char)79, (char)249, (char)122, (char)64, (char)236, (char)50, (char)15, (char)192, (char)15, (char)213, (char)3, (char)5, (char)237, (char)217, (char)119, (char)159, (char)229, (char)112, (char)238, (char)148, (char)82, (char)32, (char)56, (char)183, (char)198, (char)152, (char)85, (char)220}, 0) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5528868354590588398L);
            assert(pack.epv_GET() == (char)27646);
            assert(pack.vel_GET() == (char)50412);
            assert(pack.cog_GET() == (char)41381);
            assert(pack.satellites_visible_GET() == (char)1);
            assert(pack.eph_GET() == (char)63667);
            assert(pack.dgps_age_GET() == 2482414603L);
            assert(pack.alt_GET() == 611532979);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
            assert(pack.lat_GET() == -636254236);
            assert(pack.lon_GET() == 561828960);
            assert(pack.dgps_numch_GET() == (char)164);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.time_usec_SET(5528868354590588398L) ;
        p124.epv_SET((char)27646) ;
        p124.lat_SET(-636254236) ;
        p124.lon_SET(561828960) ;
        p124.cog_SET((char)41381) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX) ;
        p124.eph_SET((char)63667) ;
        p124.alt_SET(611532979) ;
        p124.satellites_visible_SET((char)1) ;
        p124.vel_SET((char)50412) ;
        p124.dgps_numch_SET((char)164) ;
        p124.dgps_age_SET(2482414603L) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vservo_GET() == (char)1183);
            assert(pack.Vcc_GET() == (char)53768);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID));
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)1183) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID)) ;
        p125.Vcc_SET((char)53768) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.baudrate_GET() == 2273230786L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)129, (char)136, (char)240, (char)130, (char)131, (char)122, (char)128, (char)131, (char)240, (char)202, (char)174, (char)121, (char)215, (char)130, (char)82, (char)53, (char)206, (char)217, (char)99, (char)131, (char)57, (char)245, (char)89, (char)241, (char)102, (char)136, (char)5, (char)234, (char)164, (char)126, (char)23, (char)67, (char)143, (char)244, (char)25, (char)232, (char)80, (char)230, (char)69, (char)255, (char)29, (char)254, (char)108, (char)135, (char)195, (char)203, (char)69, (char)188, (char)152, (char)160, (char)117, (char)24, (char)60, (char)56, (char)37, (char)136, (char)216, (char)232, (char)167, (char)27, (char)62, (char)106, (char)11, (char)169, (char)217, (char)31, (char)66, (char)141, (char)59, (char)165}));
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI));
            assert(pack.timeout_GET() == (char)40658);
            assert(pack.count_GET() == (char)194);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.data__SET(new char[] {(char)129, (char)136, (char)240, (char)130, (char)131, (char)122, (char)128, (char)131, (char)240, (char)202, (char)174, (char)121, (char)215, (char)130, (char)82, (char)53, (char)206, (char)217, (char)99, (char)131, (char)57, (char)245, (char)89, (char)241, (char)102, (char)136, (char)5, (char)234, (char)164, (char)126, (char)23, (char)67, (char)143, (char)244, (char)25, (char)232, (char)80, (char)230, (char)69, (char)255, (char)29, (char)254, (char)108, (char)135, (char)195, (char)203, (char)69, (char)188, (char)152, (char)160, (char)117, (char)24, (char)60, (char)56, (char)37, (char)136, (char)216, (char)232, (char)167, (char)27, (char)62, (char)106, (char)11, (char)169, (char)217, (char)31, (char)66, (char)141, (char)59, (char)165}, 0) ;
        p126.count_SET((char)194) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI)) ;
        p126.timeout_SET((char)40658) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1) ;
        p126.baudrate_SET(2273230786L) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.accuracy_GET() == 214459700L);
            assert(pack.wn_GET() == (char)8180);
            assert(pack.rtk_receiver_id_GET() == (char)164);
            assert(pack.tow_GET() == 483052559L);
            assert(pack.rtk_rate_GET() == (char)160);
            assert(pack.time_last_baseline_ms_GET() == 277261088L);
            assert(pack.nsats_GET() == (char)66);
            assert(pack.baseline_a_mm_GET() == -564007252);
            assert(pack.baseline_c_mm_GET() == 796654519);
            assert(pack.rtk_health_GET() == (char)206);
            assert(pack.iar_num_hypotheses_GET() == -1459236550);
            assert(pack.baseline_coords_type_GET() == (char)54);
            assert(pack.baseline_b_mm_GET() == 2038272279);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.baseline_coords_type_SET((char)54) ;
        p127.accuracy_SET(214459700L) ;
        p127.baseline_b_mm_SET(2038272279) ;
        p127.wn_SET((char)8180) ;
        p127.rtk_receiver_id_SET((char)164) ;
        p127.tow_SET(483052559L) ;
        p127.rtk_health_SET((char)206) ;
        p127.baseline_c_mm_SET(796654519) ;
        p127.time_last_baseline_ms_SET(277261088L) ;
        p127.baseline_a_mm_SET(-564007252) ;
        p127.nsats_SET((char)66) ;
        p127.rtk_rate_SET((char)160) ;
        p127.iar_num_hypotheses_SET(-1459236550) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.nsats_GET() == (char)196);
            assert(pack.wn_GET() == (char)21519);
            assert(pack.baseline_a_mm_GET() == 169930369);
            assert(pack.rtk_receiver_id_GET() == (char)217);
            assert(pack.baseline_b_mm_GET() == 1774294035);
            assert(pack.baseline_c_mm_GET() == -1293583928);
            assert(pack.iar_num_hypotheses_GET() == 1987384142);
            assert(pack.time_last_baseline_ms_GET() == 2773367903L);
            assert(pack.rtk_rate_GET() == (char)191);
            assert(pack.rtk_health_GET() == (char)190);
            assert(pack.tow_GET() == 317129292L);
            assert(pack.baseline_coords_type_GET() == (char)229);
            assert(pack.accuracy_GET() == 3228527268L);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.rtk_health_SET((char)190) ;
        p128.time_last_baseline_ms_SET(2773367903L) ;
        p128.baseline_coords_type_SET((char)229) ;
        p128.rtk_rate_SET((char)191) ;
        p128.rtk_receiver_id_SET((char)217) ;
        p128.baseline_b_mm_SET(1774294035) ;
        p128.iar_num_hypotheses_SET(1987384142) ;
        p128.baseline_a_mm_SET(169930369) ;
        p128.tow_SET(317129292L) ;
        p128.nsats_SET((char)196) ;
        p128.accuracy_SET(3228527268L) ;
        p128.wn_SET((char)21519) ;
        p128.baseline_c_mm_SET(-1293583928) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == (short)31032);
            assert(pack.ygyro_GET() == (short)25434);
            assert(pack.ymag_GET() == (short)29240);
            assert(pack.xgyro_GET() == (short)4105);
            assert(pack.time_boot_ms_GET() == 4201810654L);
            assert(pack.zmag_GET() == (short)29638);
            assert(pack.yacc_GET() == (short)11332);
            assert(pack.xacc_GET() == (short) -11928);
            assert(pack.zacc_GET() == (short)16503);
            assert(pack.zgyro_GET() == (short) -25064);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.ymag_SET((short)29240) ;
        p129.xmag_SET((short)31032) ;
        p129.xacc_SET((short) -11928) ;
        p129.zmag_SET((short)29638) ;
        p129.yacc_SET((short)11332) ;
        p129.xgyro_SET((short)4105) ;
        p129.time_boot_ms_SET(4201810654L) ;
        p129.zgyro_SET((short) -25064) ;
        p129.ygyro_SET((short)25434) ;
        p129.zacc_SET((short)16503) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.jpg_quality_GET() == (char)12);
            assert(pack.payload_GET() == (char)53);
            assert(pack.width_GET() == (char)53031);
            assert(pack.size_GET() == 1588447449L);
            assert(pack.height_GET() == (char)64513);
            assert(pack.packets_GET() == (char)63618);
            assert(pack.type_GET() == (char)46);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.payload_SET((char)53) ;
        p130.width_SET((char)53031) ;
        p130.type_SET((char)46) ;
        p130.height_SET((char)64513) ;
        p130.size_SET(1588447449L) ;
        p130.packets_SET((char)63618) ;
        p130.jpg_quality_SET((char)12) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)48707);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)216, (char)62, (char)166, (char)16, (char)220, (char)189, (char)164, (char)134, (char)79, (char)67, (char)106, (char)215, (char)150, (char)76, (char)67, (char)234, (char)14, (char)99, (char)12, (char)45, (char)199, (char)26, (char)116, (char)166, (char)209, (char)248, (char)149, (char)184, (char)73, (char)131, (char)49, (char)10, (char)35, (char)63, (char)75, (char)78, (char)43, (char)96, (char)23, (char)48, (char)180, (char)229, (char)52, (char)86, (char)2, (char)214, (char)156, (char)184, (char)195, (char)160, (char)186, (char)253, (char)203, (char)118, (char)75, (char)124, (char)8, (char)66, (char)104, (char)231, (char)74, (char)173, (char)191, (char)186, (char)232, (char)193, (char)178, (char)10, (char)36, (char)155, (char)117, (char)188, (char)165, (char)70, (char)136, (char)118, (char)143, (char)177, (char)53, (char)8, (char)250, (char)98, (char)51, (char)131, (char)168, (char)88, (char)132, (char)43, (char)77, (char)106, (char)233, (char)14, (char)95, (char)18, (char)19, (char)127, (char)240, (char)24, (char)77, (char)190, (char)16, (char)47, (char)116, (char)226, (char)102, (char)128, (char)194, (char)85, (char)194, (char)129, (char)37, (char)245, (char)23, (char)113, (char)223, (char)66, (char)239, (char)119, (char)65, (char)106, (char)173, (char)232, (char)104, (char)140, (char)23, (char)184, (char)122, (char)151, (char)86, (char)27, (char)128, (char)160, (char)226, (char)204, (char)209, (char)158, (char)143, (char)56, (char)159, (char)116, (char)60, (char)222, (char)145, (char)95, (char)69, (char)155, (char)7, (char)44, (char)223, (char)72, (char)151, (char)81, (char)177, (char)142, (char)72, (char)168, (char)66, (char)123, (char)199, (char)209, (char)123, (char)124, (char)202, (char)19, (char)42, (char)53, (char)43, (char)218, (char)10, (char)206, (char)165, (char)44, (char)153, (char)242, (char)221, (char)219, (char)146, (char)252, (char)193, (char)116, (char)231, (char)132, (char)180, (char)246, (char)220, (char)59, (char)41, (char)135, (char)167, (char)221, (char)236, (char)76, (char)243, (char)176, (char)178, (char)109, (char)95, (char)116, (char)231, (char)144, (char)198, (char)135, (char)90, (char)82, (char)93, (char)1, (char)32, (char)138, (char)21, (char)136, (char)124, (char)33, (char)118, (char)134, (char)169, (char)247, (char)23, (char)66, (char)39, (char)61, (char)168, (char)244, (char)176, (char)223, (char)147, (char)195, (char)177, (char)235, (char)100, (char)39, (char)1, (char)92, (char)247, (char)183, (char)68, (char)89, (char)97, (char)50, (char)203, (char)52, (char)39, (char)117, (char)35, (char)135, (char)161, (char)178, (char)57, (char)147, (char)154, (char)126, (char)76, (char)63, (char)251}));
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)48707) ;
        p131.data__SET(new char[] {(char)216, (char)62, (char)166, (char)16, (char)220, (char)189, (char)164, (char)134, (char)79, (char)67, (char)106, (char)215, (char)150, (char)76, (char)67, (char)234, (char)14, (char)99, (char)12, (char)45, (char)199, (char)26, (char)116, (char)166, (char)209, (char)248, (char)149, (char)184, (char)73, (char)131, (char)49, (char)10, (char)35, (char)63, (char)75, (char)78, (char)43, (char)96, (char)23, (char)48, (char)180, (char)229, (char)52, (char)86, (char)2, (char)214, (char)156, (char)184, (char)195, (char)160, (char)186, (char)253, (char)203, (char)118, (char)75, (char)124, (char)8, (char)66, (char)104, (char)231, (char)74, (char)173, (char)191, (char)186, (char)232, (char)193, (char)178, (char)10, (char)36, (char)155, (char)117, (char)188, (char)165, (char)70, (char)136, (char)118, (char)143, (char)177, (char)53, (char)8, (char)250, (char)98, (char)51, (char)131, (char)168, (char)88, (char)132, (char)43, (char)77, (char)106, (char)233, (char)14, (char)95, (char)18, (char)19, (char)127, (char)240, (char)24, (char)77, (char)190, (char)16, (char)47, (char)116, (char)226, (char)102, (char)128, (char)194, (char)85, (char)194, (char)129, (char)37, (char)245, (char)23, (char)113, (char)223, (char)66, (char)239, (char)119, (char)65, (char)106, (char)173, (char)232, (char)104, (char)140, (char)23, (char)184, (char)122, (char)151, (char)86, (char)27, (char)128, (char)160, (char)226, (char)204, (char)209, (char)158, (char)143, (char)56, (char)159, (char)116, (char)60, (char)222, (char)145, (char)95, (char)69, (char)155, (char)7, (char)44, (char)223, (char)72, (char)151, (char)81, (char)177, (char)142, (char)72, (char)168, (char)66, (char)123, (char)199, (char)209, (char)123, (char)124, (char)202, (char)19, (char)42, (char)53, (char)43, (char)218, (char)10, (char)206, (char)165, (char)44, (char)153, (char)242, (char)221, (char)219, (char)146, (char)252, (char)193, (char)116, (char)231, (char)132, (char)180, (char)246, (char)220, (char)59, (char)41, (char)135, (char)167, (char)221, (char)236, (char)76, (char)243, (char)176, (char)178, (char)109, (char)95, (char)116, (char)231, (char)144, (char)198, (char)135, (char)90, (char)82, (char)93, (char)1, (char)32, (char)138, (char)21, (char)136, (char)124, (char)33, (char)118, (char)134, (char)169, (char)247, (char)23, (char)66, (char)39, (char)61, (char)168, (char)244, (char)176, (char)223, (char)147, (char)195, (char)177, (char)235, (char)100, (char)39, (char)1, (char)92, (char)247, (char)183, (char)68, (char)89, (char)97, (char)50, (char)203, (char)52, (char)39, (char)117, (char)35, (char)135, (char)161, (char)178, (char)57, (char)147, (char)154, (char)126, (char)76, (char)63, (char)251}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.covariance_GET() == (char)227);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_45);
            assert(pack.max_distance_GET() == (char)59882);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.current_distance_GET() == (char)37522);
            assert(pack.min_distance_GET() == (char)41839);
            assert(pack.time_boot_ms_GET() == 839114750L);
            assert(pack.id_GET() == (char)223);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.max_distance_SET((char)59882) ;
        p132.covariance_SET((char)227) ;
        p132.min_distance_SET((char)41839) ;
        p132.current_distance_SET((char)37522) ;
        p132.time_boot_ms_SET(839114750L) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p132.id_SET((char)223) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_45) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mask_GET() == 123894420619970974L);
            assert(pack.grid_spacing_GET() == (char)60281);
            assert(pack.lon_GET() == 1904329643);
            assert(pack.lat_GET() == 1672947867);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.mask_SET(123894420619970974L) ;
        p133.lon_SET(1904329643) ;
        p133.lat_SET(1672947867) ;
        p133.grid_spacing_SET((char)60281) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1711433970);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)12708, (short)5877, (short)29645, (short) -30262, (short) -17828, (short) -14267, (short) -31730, (short)4755, (short)13481, (short)26247, (short) -31089, (short) -482, (short) -2012, (short) -1217, (short)20985, (short) -13586}));
            assert(pack.grid_spacing_GET() == (char)11434);
            assert(pack.lat_GET() == -1168084011);
            assert(pack.gridbit_GET() == (char)128);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.grid_spacing_SET((char)11434) ;
        p134.lat_SET(-1168084011) ;
        p134.gridbit_SET((char)128) ;
        p134.data__SET(new short[] {(short)12708, (short)5877, (short)29645, (short) -30262, (short) -17828, (short) -14267, (short) -31730, (short)4755, (short)13481, (short)26247, (short) -31089, (short) -482, (short) -2012, (short) -1217, (short)20985, (short) -13586}, 0) ;
        p134.lon_SET(-1711433970) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 1513372543);
            assert(pack.lat_GET() == -132009383);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-132009383) ;
        p135.lon_SET(1513372543) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.pending_GET() == (char)3416);
            assert(pack.lon_GET() == 1449727753);
            assert(pack.loaded_GET() == (char)48243);
            assert(pack.current_height_GET() == -3.9550904E37F);
            assert(pack.spacing_GET() == (char)25260);
            assert(pack.lat_GET() == -1551127848);
            assert(pack.terrain_height_GET() == 2.6707778E37F);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lon_SET(1449727753) ;
        p136.spacing_SET((char)25260) ;
        p136.pending_SET((char)3416) ;
        p136.lat_SET(-1551127848) ;
        p136.terrain_height_SET(2.6707778E37F) ;
        p136.loaded_SET((char)48243) ;
        p136.current_height_SET(-3.9550904E37F) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == 4.0525986E37F);
            assert(pack.time_boot_ms_GET() == 776086898L);
            assert(pack.temperature_GET() == (short) -31241);
            assert(pack.press_abs_GET() == 2.7671642E38F);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(776086898L) ;
        p137.temperature_SET((short) -31241) ;
        p137.press_diff_SET(4.0525986E37F) ;
        p137.press_abs_SET(2.7671642E38F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -2.4304925E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-4.2932324E37F, -3.0328956E38F, 6.993935E37F, -1.5430895E38F}));
            assert(pack.z_GET() == -2.3692714E38F);
            assert(pack.time_usec_GET() == 5634410304682159247L);
            assert(pack.y_GET() == -2.6882271E38F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.y_SET(-2.6882271E38F) ;
        p138.q_SET(new float[] {-4.2932324E37F, -3.0328956E38F, 6.993935E37F, -1.5430895E38F}, 0) ;
        p138.time_usec_SET(5634410304682159247L) ;
        p138.z_SET(-2.3692714E38F) ;
        p138.x_SET(-2.4304925E38F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)128);
            assert(pack.target_component_GET() == (char)64);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.3512379E38F, -1.7827303E38F, -3.31226E38F, -1.5440903E38F, -1.943348E38F, 6.330399E37F, 5.2902543E37F, 2.2345828E38F}));
            assert(pack.time_usec_GET() == 2198255002583630240L);
            assert(pack.group_mlx_GET() == (char)61);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.controls_SET(new float[] {-1.3512379E38F, -1.7827303E38F, -3.31226E38F, -1.5440903E38F, -1.943348E38F, 6.330399E37F, 5.2902543E37F, 2.2345828E38F}, 0) ;
        p139.target_system_SET((char)128) ;
        p139.group_mlx_SET((char)61) ;
        p139.time_usec_SET(2198255002583630240L) ;
        p139.target_component_SET((char)64) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {3.295501E37F, -2.7066685E38F, 9.225196E37F, 3.18105E38F, 5.351803E37F, 8.406116E37F, 1.7753654E38F, 8.61471E37F}));
            assert(pack.time_usec_GET() == 6862146754357810822L);
            assert(pack.group_mlx_GET() == (char)244);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)244) ;
        p140.controls_SET(new float[] {3.295501E37F, -2.7066685E38F, 9.225196E37F, 3.18105E38F, 5.351803E37F, 8.406116E37F, 1.7753654E38F, 8.61471E37F}, 0) ;
        p140.time_usec_SET(6862146754357810822L) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.bottom_clearance_GET() == -2.6090298E37F);
            assert(pack.altitude_amsl_GET() == 3.0340223E38F);
            assert(pack.altitude_terrain_GET() == 4.3751733E37F);
            assert(pack.altitude_local_GET() == -2.145891E37F);
            assert(pack.altitude_relative_GET() == 5.8586846E37F);
            assert(pack.altitude_monotonic_GET() == -1.217585E37F);
            assert(pack.time_usec_GET() == 7469036545806106613L);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_terrain_SET(4.3751733E37F) ;
        p141.bottom_clearance_SET(-2.6090298E37F) ;
        p141.altitude_monotonic_SET(-1.217585E37F) ;
        p141.altitude_relative_SET(5.8586846E37F) ;
        p141.altitude_local_SET(-2.145891E37F) ;
        p141.time_usec_SET(7469036545806106613L) ;
        p141.altitude_amsl_SET(3.0340223E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.transfer_type_GET() == (char)173);
            assert(pack.request_id_GET() == (char)202);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)28, (char)195, (char)64, (char)165, (char)218, (char)187, (char)219, (char)64, (char)58, (char)127, (char)165, (char)91, (char)255, (char)171, (char)174, (char)123, (char)161, (char)169, (char)250, (char)2, (char)170, (char)2, (char)9, (char)95, (char)137, (char)116, (char)246, (char)39, (char)202, (char)110, (char)73, (char)79, (char)234, (char)71, (char)84, (char)137, (char)179, (char)211, (char)226, (char)86, (char)238, (char)62, (char)214, (char)69, (char)206, (char)59, (char)30, (char)124, (char)7, (char)179, (char)122, (char)126, (char)214, (char)120, (char)56, (char)137, (char)48, (char)224, (char)8, (char)239, (char)107, (char)213, (char)50, (char)211, (char)167, (char)10, (char)67, (char)133, (char)47, (char)146, (char)181, (char)60, (char)149, (char)18, (char)143, (char)7, (char)134, (char)169, (char)231, (char)105, (char)73, (char)187, (char)17, (char)103, (char)203, (char)175, (char)104, (char)183, (char)33, (char)186, (char)240, (char)196, (char)47, (char)168, (char)125, (char)186, (char)131, (char)50, (char)237, (char)80, (char)69, (char)151, (char)164, (char)19, (char)247, (char)238, (char)115, (char)1, (char)111, (char)247, (char)169, (char)243, (char)153, (char)124, (char)106, (char)254, (char)23, (char)251, (char)88, (char)196}));
            assert(pack.uri_type_GET() == (char)164);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)190, (char)74, (char)183, (char)37, (char)49, (char)128, (char)173, (char)28, (char)96, (char)210, (char)221, (char)170, (char)138, (char)183, (char)115, (char)226, (char)187, (char)204, (char)100, (char)197, (char)239, (char)174, (char)102, (char)226, (char)140, (char)166, (char)219, (char)20, (char)244, (char)163, (char)87, (char)162, (char)174, (char)25, (char)188, (char)219, (char)113, (char)92, (char)38, (char)211, (char)42, (char)97, (char)21, (char)190, (char)195, (char)95, (char)0, (char)20, (char)157, (char)49, (char)190, (char)70, (char)158, (char)86, (char)71, (char)136, (char)123, (char)136, (char)244, (char)119, (char)11, (char)95, (char)18, (char)49, (char)12, (char)104, (char)163, (char)71, (char)90, (char)8, (char)227, (char)22, (char)1, (char)67, (char)195, (char)195, (char)95, (char)239, (char)186, (char)157, (char)164, (char)4, (char)86, (char)35, (char)156, (char)151, (char)207, (char)177, (char)112, (char)9, (char)155, (char)228, (char)38, (char)235, (char)4, (char)170, (char)157, (char)63, (char)206, (char)37, (char)167, (char)108, (char)116, (char)5, (char)172, (char)136, (char)98, (char)235, (char)126, (char)171, (char)16, (char)90, (char)142, (char)68, (char)143, (char)63, (char)136, (char)122, (char)72, (char)32}));
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_type_SET((char)164) ;
        p142.transfer_type_SET((char)173) ;
        p142.uri_SET(new char[] {(char)28, (char)195, (char)64, (char)165, (char)218, (char)187, (char)219, (char)64, (char)58, (char)127, (char)165, (char)91, (char)255, (char)171, (char)174, (char)123, (char)161, (char)169, (char)250, (char)2, (char)170, (char)2, (char)9, (char)95, (char)137, (char)116, (char)246, (char)39, (char)202, (char)110, (char)73, (char)79, (char)234, (char)71, (char)84, (char)137, (char)179, (char)211, (char)226, (char)86, (char)238, (char)62, (char)214, (char)69, (char)206, (char)59, (char)30, (char)124, (char)7, (char)179, (char)122, (char)126, (char)214, (char)120, (char)56, (char)137, (char)48, (char)224, (char)8, (char)239, (char)107, (char)213, (char)50, (char)211, (char)167, (char)10, (char)67, (char)133, (char)47, (char)146, (char)181, (char)60, (char)149, (char)18, (char)143, (char)7, (char)134, (char)169, (char)231, (char)105, (char)73, (char)187, (char)17, (char)103, (char)203, (char)175, (char)104, (char)183, (char)33, (char)186, (char)240, (char)196, (char)47, (char)168, (char)125, (char)186, (char)131, (char)50, (char)237, (char)80, (char)69, (char)151, (char)164, (char)19, (char)247, (char)238, (char)115, (char)1, (char)111, (char)247, (char)169, (char)243, (char)153, (char)124, (char)106, (char)254, (char)23, (char)251, (char)88, (char)196}, 0) ;
        p142.storage_SET(new char[] {(char)190, (char)74, (char)183, (char)37, (char)49, (char)128, (char)173, (char)28, (char)96, (char)210, (char)221, (char)170, (char)138, (char)183, (char)115, (char)226, (char)187, (char)204, (char)100, (char)197, (char)239, (char)174, (char)102, (char)226, (char)140, (char)166, (char)219, (char)20, (char)244, (char)163, (char)87, (char)162, (char)174, (char)25, (char)188, (char)219, (char)113, (char)92, (char)38, (char)211, (char)42, (char)97, (char)21, (char)190, (char)195, (char)95, (char)0, (char)20, (char)157, (char)49, (char)190, (char)70, (char)158, (char)86, (char)71, (char)136, (char)123, (char)136, (char)244, (char)119, (char)11, (char)95, (char)18, (char)49, (char)12, (char)104, (char)163, (char)71, (char)90, (char)8, (char)227, (char)22, (char)1, (char)67, (char)195, (char)195, (char)95, (char)239, (char)186, (char)157, (char)164, (char)4, (char)86, (char)35, (char)156, (char)151, (char)207, (char)177, (char)112, (char)9, (char)155, (char)228, (char)38, (char)235, (char)4, (char)170, (char)157, (char)63, (char)206, (char)37, (char)167, (char)108, (char)116, (char)5, (char)172, (char)136, (char)98, (char)235, (char)126, (char)171, (char)16, (char)90, (char)142, (char)68, (char)143, (char)63, (char)136, (char)122, (char)72, (char)32}, 0) ;
        p142.request_id_SET((char)202) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -969);
            assert(pack.time_boot_ms_GET() == 3793559804L);
            assert(pack.press_abs_GET() == 3.4028125E38F);
            assert(pack.press_diff_GET() == -1.5977574E38F);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.temperature_SET((short) -969) ;
        p143.press_abs_SET(3.4028125E38F) ;
        p143.press_diff_SET(-1.5977574E38F) ;
        p143.time_boot_ms_SET(3793559804L) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.timestamp_GET() == 8579106114492596176L);
            assert(pack.alt_GET() == -1.3313915E38F);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-6.508638E37F, 1.1766561E38F, 2.495332E38F, 1.6427333E38F}));
            assert(pack.lat_GET() == 592133245);
            assert(pack.lon_GET() == -1728555161);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-3.28796E38F, -1.2138914E38F, 6.7394725E37F}));
            assert(pack.est_capabilities_GET() == (char)114);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-2.0879583E38F, 3.3563784E38F, 5.031415E37F}));
            assert(Arrays.equals(pack.vel_GET(),  new float[] {3.896219E37F, -1.2118741E38F, 1.0284923E38F}));
            assert(pack.custom_state_GET() == 7353810106790539274L);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-1.4580159E38F, -1.7069439E38F, 1.4694908E38F}));
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.attitude_q_SET(new float[] {-6.508638E37F, 1.1766561E38F, 2.495332E38F, 1.6427333E38F}, 0) ;
        p144.position_cov_SET(new float[] {-1.4580159E38F, -1.7069439E38F, 1.4694908E38F}, 0) ;
        p144.est_capabilities_SET((char)114) ;
        p144.timestamp_SET(8579106114492596176L) ;
        p144.vel_SET(new float[] {3.896219E37F, -1.2118741E38F, 1.0284923E38F}, 0) ;
        p144.lon_SET(-1728555161) ;
        p144.alt_SET(-1.3313915E38F) ;
        p144.lat_SET(592133245) ;
        p144.rates_SET(new float[] {-2.0879583E38F, 3.3563784E38F, 5.031415E37F}, 0) ;
        p144.custom_state_SET(7353810106790539274L) ;
        p144.acc_SET(new float[] {-3.28796E38F, -1.2138914E38F, 6.7394725E37F}, 0) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.z_vel_GET() == -1.1883607E38F);
            assert(pack.yaw_rate_GET() == -7.3855773E37F);
            assert(pack.y_acc_GET() == 2.0019156E38F);
            assert(pack.z_pos_GET() == -1.0700571E38F);
            assert(pack.y_vel_GET() == -2.3227018E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {5.8550444E37F, 1.0422306E38F, -2.0860626E37F}));
            assert(pack.airspeed_GET() == -2.3264845E38F);
            assert(pack.x_acc_GET() == 1.0974309E38F);
            assert(pack.roll_rate_GET() == 1.0614658E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.2362382E38F, -1.9243567E38F, 1.452631E36F, 1.9009394E38F}));
            assert(pack.y_pos_GET() == -3.0143052E38F);
            assert(pack.pitch_rate_GET() == 2.6807048E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-2.3686751E38F, -1.3070193E38F, 3.1172283E38F}));
            assert(pack.z_acc_GET() == 1.2074472E38F);
            assert(pack.x_pos_GET() == 9.348773E37F);
            assert(pack.x_vel_GET() == -7.4874214E37F);
            assert(pack.time_usec_GET() == 998229624814479224L);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.pitch_rate_SET(2.6807048E38F) ;
        p146.yaw_rate_SET(-7.3855773E37F) ;
        p146.vel_variance_SET(new float[] {-2.3686751E38F, -1.3070193E38F, 3.1172283E38F}, 0) ;
        p146.z_pos_SET(-1.0700571E38F) ;
        p146.z_vel_SET(-1.1883607E38F) ;
        p146.q_SET(new float[] {1.2362382E38F, -1.9243567E38F, 1.452631E36F, 1.9009394E38F}, 0) ;
        p146.x_pos_SET(9.348773E37F) ;
        p146.x_vel_SET(-7.4874214E37F) ;
        p146.airspeed_SET(-2.3264845E38F) ;
        p146.pos_variance_SET(new float[] {5.8550444E37F, 1.0422306E38F, -2.0860626E37F}, 0) ;
        p146.y_vel_SET(-2.3227018E38F) ;
        p146.y_pos_SET(-3.0143052E38F) ;
        p146.y_acc_SET(2.0019156E38F) ;
        p146.time_usec_SET(998229624814479224L) ;
        p146.roll_rate_SET(1.0614658E38F) ;
        p146.x_acc_SET(1.0974309E38F) ;
        p146.z_acc_SET(1.2074472E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.current_consumed_GET() == 1202876575);
            assert(pack.id_GET() == (char)91);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
            assert(pack.battery_remaining_GET() == (byte) - 111);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
            assert(pack.energy_consumed_GET() == 1558678292);
            assert(pack.temperature_GET() == (short)20816);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)20630, (char)34609, (char)50216, (char)13318, (char)7774, (char)15110, (char)42228, (char)4641, (char)42556, (char)5033}));
            assert(pack.current_battery_GET() == (short)31033);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN) ;
        p147.current_battery_SET((short)31033) ;
        p147.temperature_SET((short)20816) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS) ;
        p147.voltages_SET(new char[] {(char)20630, (char)34609, (char)50216, (char)13318, (char)7774, (char)15110, (char)42228, (char)4641, (char)42556, (char)5033}, 0) ;
        p147.energy_consumed_SET(1558678292) ;
        p147.id_SET((char)91) ;
        p147.current_consumed_SET(1202876575) ;
        p147.battery_remaining_SET((byte) - 111) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)226, (char)51, (char)183, (char)6, (char)83, (char)77, (char)167, (char)150}));
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)146, (char)125, (char)193, (char)101, (char)171, (char)179, (char)214, (char)60, (char)172, (char)11, (char)57, (char)29, (char)57, (char)241, (char)167, (char)170, (char)198, (char)135}));
            assert(pack.flight_sw_version_GET() == 3138779207L);
            assert(pack.vendor_id_GET() == (char)20650);
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)250, (char)215, (char)134, (char)52, (char)228, (char)29, (char)214, (char)227}));
            assert(pack.os_sw_version_GET() == 3243672010L);
            assert(pack.board_version_GET() == 3942675842L);
            assert(pack.product_id_GET() == (char)16740);
            assert(pack.uid_GET() == 3911508858827636784L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)9, (char)87, (char)25, (char)189, (char)193, (char)193, (char)182, (char)141}));
            assert(pack.middleware_sw_version_GET() == 3269289390L);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.flight_custom_version_SET(new char[] {(char)250, (char)215, (char)134, (char)52, (char)228, (char)29, (char)214, (char)227}, 0) ;
        p148.os_custom_version_SET(new char[] {(char)9, (char)87, (char)25, (char)189, (char)193, (char)193, (char)182, (char)141}, 0) ;
        p148.product_id_SET((char)16740) ;
        p148.uid_SET(3911508858827636784L) ;
        p148.middleware_custom_version_SET(new char[] {(char)226, (char)51, (char)183, (char)6, (char)83, (char)77, (char)167, (char)150}, 0) ;
        p148.os_sw_version_SET(3243672010L) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE)) ;
        p148.vendor_id_SET((char)20650) ;
        p148.flight_sw_version_SET(3138779207L) ;
        p148.middleware_sw_version_SET(3269289390L) ;
        p148.board_version_SET(3942675842L) ;
        p148.uid2_SET(new char[] {(char)146, (char)125, (char)193, (char)101, (char)171, (char)179, (char)214, (char)60, (char)172, (char)11, (char)57, (char)29, (char)57, (char)241, (char)167, (char)170, (char)198, (char)135}, 0, PH) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {5.5031557E37F, -6.2978586E37F, -2.181825E38F, 7.747358E37F}));
            assert(pack.x_TRY(ph) == -2.536769E38F);
            assert(pack.angle_y_GET() == -3.1487654E38F);
            assert(pack.size_y_GET() == 1.1397422E38F);
            assert(pack.time_usec_GET() == 4896826064195921510L);
            assert(pack.y_TRY(ph) == 3.2965215E38F);
            assert(pack.distance_GET() == 9.820517E37F);
            assert(pack.size_x_GET() == 1.2164598E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
            assert(pack.target_num_GET() == (char)52);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.z_TRY(ph) == 2.5423278E38F);
            assert(pack.position_valid_TRY(ph) == (char)227);
            assert(pack.angle_x_GET() == -3.3555703E38F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.x_SET(-2.536769E38F, PH) ;
        p149.time_usec_SET(4896826064195921510L) ;
        p149.distance_SET(9.820517E37F) ;
        p149.size_x_SET(1.2164598E38F) ;
        p149.target_num_SET((char)52) ;
        p149.angle_y_SET(-3.1487654E38F) ;
        p149.z_SET(2.5423278E38F, PH) ;
        p149.y_SET(3.2965215E38F, PH) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON) ;
        p149.position_valid_SET((char)227, PH) ;
        p149.angle_x_SET(-3.3555703E38F) ;
        p149.size_y_SET(1.1397422E38F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p149.q_SET(new float[] {5.5031557E37F, -6.2978586E37F, -2.181825E38F, 7.747358E37F}, 0, PH) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAV_FILTER_BIAS.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 1621487636892931628L);
            assert(pack.accel_0_GET() == 1.8072292E38F);
            assert(pack.gyro_1_GET() == -1.5102249E38F);
            assert(pack.accel_2_GET() == 2.1891604E38F);
            assert(pack.gyro_2_GET() == -1.1682271E38F);
            assert(pack.accel_1_GET() == -1.4745982E38F);
            assert(pack.gyro_0_GET() == 5.2852344E37F);
        });
        GroundControl.NAV_FILTER_BIAS p220 = CommunicationChannel.new_NAV_FILTER_BIAS();
        PH.setPack(p220);
        p220.gyro_0_SET(5.2852344E37F) ;
        p220.accel_0_SET(1.8072292E38F) ;
        p220.gyro_2_SET(-1.1682271E38F) ;
        p220.gyro_1_SET(-1.5102249E38F) ;
        p220.accel_2_SET(2.1891604E38F) ;
        p220.usec_SET(1621487636892931628L) ;
        p220.accel_1_SET(-1.4745982E38F) ;
        CommunicationChannel.instance.send(p220);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RADIO_CALIBRATION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.elevator_GET(),  new char[] {(char)19106, (char)43545, (char)63261}));
            assert(Arrays.equals(pack.gyro_GET(),  new char[] {(char)8589, (char)50885}));
            assert(Arrays.equals(pack.pitch_GET(),  new char[] {(char)42987, (char)50954, (char)54796, (char)19262, (char)16225}));
            assert(Arrays.equals(pack.rudder_GET(),  new char[] {(char)6808, (char)36976, (char)3032}));
            assert(Arrays.equals(pack.throttle_GET(),  new char[] {(char)33723, (char)55808, (char)53430, (char)27894, (char)17855}));
            assert(Arrays.equals(pack.aileron_GET(),  new char[] {(char)11836, (char)36076, (char)49807}));
        });
        GroundControl.RADIO_CALIBRATION p221 = CommunicationChannel.new_RADIO_CALIBRATION();
        PH.setPack(p221);
        p221.throttle_SET(new char[] {(char)33723, (char)55808, (char)53430, (char)27894, (char)17855}, 0) ;
        p221.elevator_SET(new char[] {(char)19106, (char)43545, (char)63261}, 0) ;
        p221.gyro_SET(new char[] {(char)8589, (char)50885}, 0) ;
        p221.aileron_SET(new char[] {(char)11836, (char)36076, (char)49807}, 0) ;
        p221.pitch_SET(new char[] {(char)42987, (char)50954, (char)54796, (char)19262, (char)16225}, 0) ;
        p221.rudder_SET(new char[] {(char)6808, (char)36976, (char)3032}, 0) ;
        CommunicationChannel.instance.send(p221);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UALBERTA_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pilot_GET() == (char)99);
            assert(pack.nav_mode_GET() == (char)116);
            assert(pack.mode_GET() == (char)145);
        });
        GroundControl.UALBERTA_SYS_STATUS p222 = CommunicationChannel.new_UALBERTA_SYS_STATUS();
        PH.setPack(p222);
        p222.pilot_SET((char)99) ;
        p222.mode_SET((char)145) ;
        p222.nav_mode_SET((char)116) ;
        CommunicationChannel.instance.send(p222);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pos_horiz_accuracy_GET() == 3.3435723E38F);
            assert(pack.pos_horiz_ratio_GET() == 5.4667275E37F);
            assert(pack.time_usec_GET() == 3461598310889960397L);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL));
            assert(pack.pos_vert_ratio_GET() == -3.2911412E38F);
            assert(pack.mag_ratio_GET() == 3.3361485E38F);
            assert(pack.vel_ratio_GET() == 2.5313798E38F);
            assert(pack.hagl_ratio_GET() == 3.2667827E38F);
            assert(pack.tas_ratio_GET() == -7.6779934E37F);
            assert(pack.pos_vert_accuracy_GET() == 4.8414386E37F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.pos_vert_ratio_SET(-3.2911412E38F) ;
        p230.vel_ratio_SET(2.5313798E38F) ;
        p230.hagl_ratio_SET(3.2667827E38F) ;
        p230.pos_vert_accuracy_SET(4.8414386E37F) ;
        p230.time_usec_SET(3461598310889960397L) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL)) ;
        p230.tas_ratio_SET(-7.6779934E37F) ;
        p230.pos_horiz_accuracy_SET(3.3435723E38F) ;
        p230.pos_horiz_ratio_SET(5.4667275E37F) ;
        p230.mag_ratio_SET(3.3361485E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_z_GET() == -1.7683375E38F);
            assert(pack.wind_x_GET() == 4.438264E37F);
            assert(pack.wind_alt_GET() == -9.270132E37F);
            assert(pack.wind_y_GET() == -3.7397863E37F);
            assert(pack.horiz_accuracy_GET() == -2.9951103E38F);
            assert(pack.var_horiz_GET() == 2.6830018E37F);
            assert(pack.var_vert_GET() == -1.7350914E38F);
            assert(pack.time_usec_GET() == 1201992742640454073L);
            assert(pack.vert_accuracy_GET() == 3.085025E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(1201992742640454073L) ;
        p231.var_vert_SET(-1.7350914E38F) ;
        p231.wind_z_SET(-1.7683375E38F) ;
        p231.var_horiz_SET(2.6830018E37F) ;
        p231.horiz_accuracy_SET(-2.9951103E38F) ;
        p231.wind_y_SET(-3.7397863E37F) ;
        p231.vert_accuracy_SET(3.085025E38F) ;
        p231.wind_x_SET(4.438264E37F) ;
        p231.wind_alt_SET(-9.270132E37F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)132);
            assert(pack.time_usec_GET() == 5768199310271174919L);
            assert(pack.speed_accuracy_GET() == 6.5380956E37F);
            assert(pack.ve_GET() == 2.0854303E37F);
            assert(pack.fix_type_GET() == (char)85);
            assert(pack.time_week_ms_GET() == 2801896323L);
            assert(pack.vert_accuracy_GET() == 2.9577313E38F);
            assert(pack.gps_id_GET() == (char)180);
            assert(pack.hdop_GET() == -6.5551744E37F);
            assert(pack.horiz_accuracy_GET() == 2.8608513E38F);
            assert(pack.time_week_GET() == (char)63934);
            assert(pack.vn_GET() == -1.6536706E38F);
            assert(pack.vd_GET() == 1.2468402E38F);
            assert(pack.vdop_GET() == 1.2885128E38F);
            assert(pack.alt_GET() == 6.268733E37F);
            assert(pack.lon_GET() == -920514339);
            assert(pack.lat_GET() == 284182861);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY));
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.hdop_SET(-6.5551744E37F) ;
        p232.vdop_SET(1.2885128E38F) ;
        p232.fix_type_SET((char)85) ;
        p232.vn_SET(-1.6536706E38F) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY)) ;
        p232.time_usec_SET(5768199310271174919L) ;
        p232.satellites_visible_SET((char)132) ;
        p232.alt_SET(6.268733E37F) ;
        p232.gps_id_SET((char)180) ;
        p232.vert_accuracy_SET(2.9577313E38F) ;
        p232.lon_SET(-920514339) ;
        p232.time_week_ms_SET(2801896323L) ;
        p232.vd_SET(1.2468402E38F) ;
        p232.lat_SET(284182861) ;
        p232.ve_SET(2.0854303E37F) ;
        p232.speed_accuracy_SET(6.5380956E37F) ;
        p232.horiz_accuracy_SET(2.8608513E38F) ;
        p232.time_week_SET((char)63934) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (char)244);
            assert(pack.len_GET() == (char)27);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)78, (char)174, (char)28, (char)47, (char)5, (char)140, (char)154, (char)159, (char)145, (char)52, (char)11, (char)119, (char)37, (char)54, (char)119, (char)201, (char)221, (char)186, (char)145, (char)95, (char)141, (char)247, (char)47, (char)72, (char)120, (char)200, (char)151, (char)235, (char)112, (char)41, (char)153, (char)119, (char)96, (char)83, (char)42, (char)198, (char)222, (char)43, (char)202, (char)91, (char)178, (char)227, (char)9, (char)49, (char)118, (char)110, (char)55, (char)154, (char)89, (char)83, (char)131, (char)26, (char)158, (char)211, (char)32, (char)22, (char)178, (char)230, (char)45, (char)234, (char)14, (char)209, (char)221, (char)171, (char)108, (char)86, (char)208, (char)128, (char)102, (char)38, (char)198, (char)243, (char)168, (char)185, (char)140, (char)175, (char)105, (char)54, (char)105, (char)164, (char)137, (char)204, (char)167, (char)213, (char)62, (char)62, (char)78, (char)48, (char)181, (char)202, (char)233, (char)200, (char)245, (char)113, (char)56, (char)224, (char)103, (char)114, (char)35, (char)1, (char)41, (char)44, (char)4, (char)109, (char)123, (char)247, (char)145, (char)26, (char)54, (char)252, (char)160, (char)99, (char)125, (char)237, (char)68, (char)42, (char)176, (char)85, (char)238, (char)149, (char)204, (char)107, (char)111, (char)37, (char)224, (char)174, (char)20, (char)43, (char)92, (char)123, (char)126, (char)95, (char)123, (char)13, (char)237, (char)116, (char)238, (char)1, (char)236, (char)252, (char)158, (char)164, (char)20, (char)111, (char)200, (char)241, (char)20, (char)63, (char)101, (char)229, (char)40, (char)184, (char)93, (char)82, (char)44, (char)174, (char)130, (char)215, (char)40, (char)213, (char)131, (char)20, (char)116, (char)102, (char)163, (char)33, (char)79, (char)210, (char)82, (char)126, (char)44, (char)37, (char)228, (char)30, (char)212, (char)18, (char)182, (char)185, (char)16, (char)162}));
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.len_SET((char)27) ;
        p233.flags_SET((char)244) ;
        p233.data__SET(new char[] {(char)78, (char)174, (char)28, (char)47, (char)5, (char)140, (char)154, (char)159, (char)145, (char)52, (char)11, (char)119, (char)37, (char)54, (char)119, (char)201, (char)221, (char)186, (char)145, (char)95, (char)141, (char)247, (char)47, (char)72, (char)120, (char)200, (char)151, (char)235, (char)112, (char)41, (char)153, (char)119, (char)96, (char)83, (char)42, (char)198, (char)222, (char)43, (char)202, (char)91, (char)178, (char)227, (char)9, (char)49, (char)118, (char)110, (char)55, (char)154, (char)89, (char)83, (char)131, (char)26, (char)158, (char)211, (char)32, (char)22, (char)178, (char)230, (char)45, (char)234, (char)14, (char)209, (char)221, (char)171, (char)108, (char)86, (char)208, (char)128, (char)102, (char)38, (char)198, (char)243, (char)168, (char)185, (char)140, (char)175, (char)105, (char)54, (char)105, (char)164, (char)137, (char)204, (char)167, (char)213, (char)62, (char)62, (char)78, (char)48, (char)181, (char)202, (char)233, (char)200, (char)245, (char)113, (char)56, (char)224, (char)103, (char)114, (char)35, (char)1, (char)41, (char)44, (char)4, (char)109, (char)123, (char)247, (char)145, (char)26, (char)54, (char)252, (char)160, (char)99, (char)125, (char)237, (char)68, (char)42, (char)176, (char)85, (char)238, (char)149, (char)204, (char)107, (char)111, (char)37, (char)224, (char)174, (char)20, (char)43, (char)92, (char)123, (char)126, (char)95, (char)123, (char)13, (char)237, (char)116, (char)238, (char)1, (char)236, (char)252, (char)158, (char)164, (char)20, (char)111, (char)200, (char)241, (char)20, (char)63, (char)101, (char)229, (char)40, (char)184, (char)93, (char)82, (char)44, (char)174, (char)130, (char)215, (char)40, (char)213, (char)131, (char)20, (char)116, (char)102, (char)163, (char)33, (char)79, (char)210, (char)82, (char)126, (char)44, (char)37, (char)228, (char)30, (char)212, (char)18, (char)182, (char)185, (char)16, (char)162}, 0) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.battery_remaining_GET() == (char)241);
            assert(pack.wp_num_GET() == (char)241);
            assert(pack.heading_sp_GET() == (short) -11466);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            assert(pack.temperature_GET() == (byte)4);
            assert(pack.airspeed_sp_GET() == (char)15);
            assert(pack.altitude_amsl_GET() == (short)21243);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED));
            assert(pack.longitude_GET() == -1054424992);
            assert(pack.heading_GET() == (char)19736);
            assert(pack.airspeed_GET() == (char)234);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.latitude_GET() == -1173341711);
            assert(pack.altitude_sp_GET() == (short) -6658);
            assert(pack.temperature_air_GET() == (byte) - 63);
            assert(pack.custom_mode_GET() == 2329778779L);
            assert(pack.climb_rate_GET() == (byte)83);
            assert(pack.throttle_GET() == (byte)20);
            assert(pack.failsafe_GET() == (char)87);
            assert(pack.gps_nsat_GET() == (char)107);
            assert(pack.wp_distance_GET() == (char)22709);
            assert(pack.roll_GET() == (short)979);
            assert(pack.groundspeed_GET() == (char)216);
            assert(pack.pitch_GET() == (short) -2603);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.heading_SET((char)19736) ;
        p234.throttle_SET((byte)20) ;
        p234.pitch_SET((short) -2603) ;
        p234.altitude_amsl_SET((short)21243) ;
        p234.temperature_SET((byte)4) ;
        p234.wp_distance_SET((char)22709) ;
        p234.latitude_SET(-1173341711) ;
        p234.roll_SET((short)979) ;
        p234.airspeed_sp_SET((char)15) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND) ;
        p234.temperature_air_SET((byte) - 63) ;
        p234.custom_mode_SET(2329778779L) ;
        p234.heading_sp_SET((short) -11466) ;
        p234.airspeed_SET((char)234) ;
        p234.longitude_SET(-1054424992) ;
        p234.gps_nsat_SET((char)107) ;
        p234.groundspeed_SET((char)216) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED)) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p234.altitude_sp_SET((short) -6658) ;
        p234.wp_num_SET((char)241) ;
        p234.climb_rate_SET((byte)83) ;
        p234.battery_remaining_SET((char)241) ;
        p234.failsafe_SET((char)87) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.clipping_2_GET() == 2415357335L);
            assert(pack.vibration_z_GET() == 1.892821E38F);
            assert(pack.vibration_x_GET() == 1.3622836E38F);
            assert(pack.clipping_0_GET() == 1749828060L);
            assert(pack.clipping_1_GET() == 1262121957L);
            assert(pack.time_usec_GET() == 5140154743874379717L);
            assert(pack.vibration_y_GET() == 9.476906E37F);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.vibration_x_SET(1.3622836E38F) ;
        p241.clipping_1_SET(1262121957L) ;
        p241.clipping_2_SET(2415357335L) ;
        p241.vibration_y_SET(9.476906E37F) ;
        p241.time_usec_SET(5140154743874379717L) ;
        p241.clipping_0_SET(1749828060L) ;
        p241.vibration_z_SET(1.892821E38F) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_x_GET() == 1.5631005E38F);
            assert(pack.altitude_GET() == -819717303);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.6926087E38F, -2.7943723E38F, -1.0488837E38F, -1.1175195E38F}));
            assert(pack.time_usec_TRY(ph) == 2461055840119509302L);
            assert(pack.z_GET() == 6.868894E37F);
            assert(pack.x_GET() == 3.035137E38F);
            assert(pack.longitude_GET() == 2126024393);
            assert(pack.approach_z_GET() == -1.6427484E38F);
            assert(pack.approach_y_GET() == -2.200964E37F);
            assert(pack.latitude_GET() == -2029773962);
            assert(pack.y_GET() == 1.1356726E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.y_SET(1.1356726E38F) ;
        p242.q_SET(new float[] {2.6926087E38F, -2.7943723E38F, -1.0488837E38F, -1.1175195E38F}, 0) ;
        p242.approach_z_SET(-1.6427484E38F) ;
        p242.approach_y_SET(-2.200964E37F) ;
        p242.approach_x_SET(1.5631005E38F) ;
        p242.longitude_SET(2126024393) ;
        p242.latitude_SET(-2029773962) ;
        p242.altitude_SET(-819717303) ;
        p242.x_SET(3.035137E38F) ;
        p242.z_SET(6.868894E37F) ;
        p242.time_usec_SET(2461055840119509302L, PH) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 9.789778E37F);
            assert(pack.approach_y_GET() == 7.1224876E37F);
            assert(pack.approach_x_GET() == -9.192325E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.3218539E38F, 7.335898E37F, 3.9567944E37F, -3.2301147E38F}));
            assert(pack.y_GET() == 7.2317707E37F);
            assert(pack.z_GET() == 3.1793293E37F);
            assert(pack.target_system_GET() == (char)245);
            assert(pack.approach_z_GET() == -1.7779879E38F);
            assert(pack.latitude_GET() == -553533264);
            assert(pack.longitude_GET() == -60405919);
            assert(pack.altitude_GET() == -72156941);
            assert(pack.time_usec_TRY(ph) == 3481539929357038232L);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.z_SET(3.1793293E37F) ;
        p243.approach_y_SET(7.1224876E37F) ;
        p243.q_SET(new float[] {-3.3218539E38F, 7.335898E37F, 3.9567944E37F, -3.2301147E38F}, 0) ;
        p243.y_SET(7.2317707E37F) ;
        p243.altitude_SET(-72156941) ;
        p243.time_usec_SET(3481539929357038232L, PH) ;
        p243.x_SET(9.789778E37F) ;
        p243.latitude_SET(-553533264) ;
        p243.approach_x_SET(-9.192325E37F) ;
        p243.longitude_SET(-60405919) ;
        p243.target_system_SET((char)245) ;
        p243.approach_z_SET(-1.7779879E38F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == -1148580344);
            assert(pack.message_id_GET() == (char)22102);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)22102) ;
        p244.interval_us_SET(-1148580344) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.squawk_GET() == (char)8413);
            assert(pack.ver_velocity_GET() == (short)18713);
            assert(pack.tslc_GET() == (char)80);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL);
            assert(pack.altitude_GET() == -890174549);
            assert(pack.callsign_LEN(ph) == 7);
            assert(pack.callsign_TRY(ph).equals("zAmwxoz"));
            assert(pack.lat_GET() == 1322864562);
            assert(pack.lon_GET() == 1367998412);
            assert(pack.hor_velocity_GET() == (char)30112);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY));
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.heading_GET() == (char)20084);
            assert(pack.ICAO_address_GET() == 4089573958L);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.lon_SET(1367998412) ;
        p246.heading_SET((char)20084) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.callsign_SET("zAmwxoz", PH) ;
        p246.ICAO_address_SET(4089573958L) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY)) ;
        p246.tslc_SET((char)80) ;
        p246.hor_velocity_SET((char)30112) ;
        p246.lat_SET(1322864562) ;
        p246.squawk_SET((char)8413) ;
        p246.altitude_SET(-890174549) ;
        p246.ver_velocity_SET((short)18713) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
            assert(pack.altitude_minimum_delta_GET() == -1.8170414E38F);
            assert(pack.id_GET() == 4155286696L);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            assert(pack.horizontal_minimum_delta_GET() == 2.7540585E38F);
            assert(pack.time_to_minimum_delta_GET() == 1.4326377E38F);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH) ;
        p247.id_SET(4155286696L) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.time_to_minimum_delta_SET(1.4326377E38F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY) ;
        p247.altitude_minimum_delta_SET(-1.8170414E38F) ;
        p247.horizontal_minimum_delta_SET(2.7540585E38F) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.message_type_GET() == (char)14006);
            assert(pack.target_component_GET() == (char)124);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)218, (char)97, (char)55, (char)56, (char)161, (char)72, (char)20, (char)111, (char)163, (char)236, (char)222, (char)66, (char)155, (char)26, (char)235, (char)183, (char)184, (char)101, (char)185, (char)126, (char)235, (char)111, (char)133, (char)75, (char)246, (char)48, (char)183, (char)75, (char)231, (char)155, (char)21, (char)68, (char)250, (char)112, (char)62, (char)62, (char)49, (char)176, (char)121, (char)170, (char)89, (char)119, (char)220, (char)63, (char)32, (char)60, (char)43, (char)234, (char)172, (char)71, (char)225, (char)13, (char)170, (char)56, (char)94, (char)180, (char)207, (char)137, (char)152, (char)129, (char)22, (char)39, (char)219, (char)50, (char)130, (char)68, (char)90, (char)52, (char)197, (char)30, (char)22, (char)119, (char)135, (char)97, (char)33, (char)253, (char)19, (char)217, (char)43, (char)231, (char)85, (char)206, (char)74, (char)129, (char)160, (char)40, (char)173, (char)219, (char)95, (char)3, (char)142, (char)61, (char)41, (char)218, (char)204, (char)218, (char)148, (char)139, (char)5, (char)12, (char)29, (char)100, (char)13, (char)208, (char)86, (char)206, (char)59, (char)199, (char)200, (char)120, (char)56, (char)108, (char)62, (char)126, (char)230, (char)24, (char)48, (char)118, (char)79, (char)251, (char)19, (char)145, (char)107, (char)38, (char)138, (char)201, (char)160, (char)216, (char)133, (char)63, (char)119, (char)201, (char)215, (char)114, (char)252, (char)254, (char)198, (char)102, (char)209, (char)208, (char)64, (char)161, (char)18, (char)105, (char)119, (char)36, (char)103, (char)134, (char)54, (char)238, (char)143, (char)72, (char)185, (char)75, (char)37, (char)124, (char)39, (char)168, (char)52, (char)191, (char)240, (char)185, (char)220, (char)107, (char)54, (char)178, (char)57, (char)1, (char)29, (char)235, (char)252, (char)142, (char)246, (char)218, (char)246, (char)225, (char)112, (char)129, (char)192, (char)71, (char)17, (char)252, (char)178, (char)3, (char)77, (char)192, (char)132, (char)181, (char)236, (char)60, (char)180, (char)5, (char)27, (char)107, (char)248, (char)96, (char)72, (char)61, (char)61, (char)118, (char)85, (char)49, (char)154, (char)147, (char)15, (char)16, (char)34, (char)232, (char)245, (char)203, (char)206, (char)111, (char)30, (char)193, (char)122, (char)5, (char)45, (char)69, (char)147, (char)47, (char)120, (char)87, (char)19, (char)104, (char)255, (char)80, (char)204, (char)90, (char)74, (char)41, (char)125, (char)162, (char)206, (char)194, (char)165, (char)109, (char)117, (char)161, (char)49, (char)224, (char)246, (char)76, (char)168, (char)9, (char)229, (char)80, (char)132, (char)249, (char)195}));
            assert(pack.target_network_GET() == (char)31);
            assert(pack.target_system_GET() == (char)130);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.message_type_SET((char)14006) ;
        p248.payload_SET(new char[] {(char)218, (char)97, (char)55, (char)56, (char)161, (char)72, (char)20, (char)111, (char)163, (char)236, (char)222, (char)66, (char)155, (char)26, (char)235, (char)183, (char)184, (char)101, (char)185, (char)126, (char)235, (char)111, (char)133, (char)75, (char)246, (char)48, (char)183, (char)75, (char)231, (char)155, (char)21, (char)68, (char)250, (char)112, (char)62, (char)62, (char)49, (char)176, (char)121, (char)170, (char)89, (char)119, (char)220, (char)63, (char)32, (char)60, (char)43, (char)234, (char)172, (char)71, (char)225, (char)13, (char)170, (char)56, (char)94, (char)180, (char)207, (char)137, (char)152, (char)129, (char)22, (char)39, (char)219, (char)50, (char)130, (char)68, (char)90, (char)52, (char)197, (char)30, (char)22, (char)119, (char)135, (char)97, (char)33, (char)253, (char)19, (char)217, (char)43, (char)231, (char)85, (char)206, (char)74, (char)129, (char)160, (char)40, (char)173, (char)219, (char)95, (char)3, (char)142, (char)61, (char)41, (char)218, (char)204, (char)218, (char)148, (char)139, (char)5, (char)12, (char)29, (char)100, (char)13, (char)208, (char)86, (char)206, (char)59, (char)199, (char)200, (char)120, (char)56, (char)108, (char)62, (char)126, (char)230, (char)24, (char)48, (char)118, (char)79, (char)251, (char)19, (char)145, (char)107, (char)38, (char)138, (char)201, (char)160, (char)216, (char)133, (char)63, (char)119, (char)201, (char)215, (char)114, (char)252, (char)254, (char)198, (char)102, (char)209, (char)208, (char)64, (char)161, (char)18, (char)105, (char)119, (char)36, (char)103, (char)134, (char)54, (char)238, (char)143, (char)72, (char)185, (char)75, (char)37, (char)124, (char)39, (char)168, (char)52, (char)191, (char)240, (char)185, (char)220, (char)107, (char)54, (char)178, (char)57, (char)1, (char)29, (char)235, (char)252, (char)142, (char)246, (char)218, (char)246, (char)225, (char)112, (char)129, (char)192, (char)71, (char)17, (char)252, (char)178, (char)3, (char)77, (char)192, (char)132, (char)181, (char)236, (char)60, (char)180, (char)5, (char)27, (char)107, (char)248, (char)96, (char)72, (char)61, (char)61, (char)118, (char)85, (char)49, (char)154, (char)147, (char)15, (char)16, (char)34, (char)232, (char)245, (char)203, (char)206, (char)111, (char)30, (char)193, (char)122, (char)5, (char)45, (char)69, (char)147, (char)47, (char)120, (char)87, (char)19, (char)104, (char)255, (char)80, (char)204, (char)90, (char)74, (char)41, (char)125, (char)162, (char)206, (char)194, (char)165, (char)109, (char)117, (char)161, (char)49, (char)224, (char)246, (char)76, (char)168, (char)9, (char)229, (char)80, (char)132, (char)249, (char)195}, 0) ;
        p248.target_system_SET((char)130) ;
        p248.target_network_SET((char)31) ;
        p248.target_component_SET((char)124) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.ver_GET() == (char)229);
            assert(pack.type_GET() == (char)1);
            assert(pack.address_GET() == (char)2425);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 63, (byte) - 32, (byte) - 56, (byte)127, (byte) - 40, (byte)27, (byte) - 126, (byte) - 27, (byte)122, (byte) - 71, (byte) - 87, (byte)32, (byte)28, (byte)10, (byte) - 116, (byte)34, (byte) - 90, (byte)106, (byte)95, (byte)63, (byte) - 125, (byte)30, (byte)85, (byte) - 89, (byte) - 68, (byte)72, (byte) - 82, (byte)45, (byte) - 123, (byte) - 89, (byte)2, (byte) - 42}));
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.value_SET(new byte[] {(byte) - 63, (byte) - 32, (byte) - 56, (byte)127, (byte) - 40, (byte)27, (byte) - 126, (byte) - 27, (byte)122, (byte) - 71, (byte) - 87, (byte)32, (byte)28, (byte)10, (byte) - 116, (byte)34, (byte) - 90, (byte)106, (byte)95, (byte)63, (byte) - 125, (byte)30, (byte)85, (byte) - 89, (byte) - 68, (byte)72, (byte) - 82, (byte)45, (byte) - 123, (byte) - 89, (byte)2, (byte) - 42}, 0) ;
        p249.type_SET((char)1) ;
        p249.ver_SET((char)229) ;
        p249.address_SET((char)2425) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.0061022E36F);
            assert(pack.x_GET() == -1.6971837E38F);
            assert(pack.name_LEN(ph) == 5);
            assert(pack.name_TRY(ph).equals("vqoaN"));
            assert(pack.time_usec_GET() == 7889087444216155190L);
            assert(pack.y_GET() == -2.425069E38F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.z_SET(-1.0061022E36F) ;
        p250.y_SET(-2.425069E38F) ;
        p250.x_SET(-1.6971837E38F) ;
        p250.time_usec_SET(7889087444216155190L) ;
        p250.name_SET("vqoaN", PH) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -2.509817E38F);
            assert(pack.name_LEN(ph) == 4);
            assert(pack.name_TRY(ph).equals("cuPa"));
            assert(pack.time_boot_ms_GET() == 199057935L);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(-2.509817E38F) ;
        p251.time_boot_ms_SET(199057935L) ;
        p251.name_SET("cuPa", PH) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 314309980);
            assert(pack.name_LEN(ph) == 5);
            assert(pack.name_TRY(ph).equals("rtulh"));
            assert(pack.time_boot_ms_GET() == 1125930689L);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(1125930689L) ;
        p252.name_SET("rtulh", PH) ;
        p252.value_SET(314309980) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
            assert(pack.text_LEN(ph) == 19);
            assert(pack.text_TRY(ph).equals("MpVuYggajvzflslvycd"));
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_CRITICAL) ;
        p253.text_SET("MpVuYggajvzflslvycd", PH) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.ind_GET() == (char)145);
            assert(pack.time_boot_ms_GET() == 2237938433L);
            assert(pack.value_GET() == -2.5261845E38F);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.value_SET(-2.5261845E38F) ;
        p254.time_boot_ms_SET(2237938433L) ;
        p254.ind_SET((char)145) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)228);
            assert(pack.initial_timestamp_GET() == 1090662132588052413L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)40, (char)148, (char)5, (char)94, (char)160, (char)103, (char)19, (char)23, (char)145, (char)61, (char)60, (char)209, (char)114, (char)100, (char)61, (char)76, (char)14, (char)110, (char)54, (char)47, (char)83, (char)91, (char)103, (char)248, (char)193, (char)49, (char)16, (char)56, (char)38, (char)207, (char)130, (char)62}));
            assert(pack.target_system_GET() == (char)49);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.initial_timestamp_SET(1090662132588052413L) ;
        p256.target_system_SET((char)49) ;
        p256.target_component_SET((char)228) ;
        p256.secret_key_SET(new char[] {(char)40, (char)148, (char)5, (char)94, (char)160, (char)103, (char)19, (char)23, (char)145, (char)61, (char)60, (char)209, (char)114, (char)100, (char)61, (char)76, (char)14, (char)110, (char)54, (char)47, (char)83, (char)91, (char)103, (char)248, (char)193, (char)49, (char)16, (char)56, (char)38, (char)207, (char)130, (char)62}, 0) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.last_change_ms_GET() == 1798491025L);
            assert(pack.state_GET() == (char)184);
            assert(pack.time_boot_ms_GET() == 51098751L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.state_SET((char)184) ;
        p257.last_change_ms_SET(1798491025L) ;
        p257.time_boot_ms_SET(51098751L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)25);
            assert(pack.tune_LEN(ph) == 30);
            assert(pack.tune_TRY(ph).equals("nzoHcrjimlJhAnvbrdsDcvwpqIgWpj"));
            assert(pack.target_system_GET() == (char)41);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_component_SET((char)25) ;
        p258.target_system_SET((char)41) ;
        p258.tune_SET("nzoHcrjimlJhAnvbrdsDcvwpqIgWpj", PH) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)110, (char)54, (char)210, (char)242, (char)181, (char)39, (char)217, (char)228, (char)194, (char)144, (char)87, (char)162, (char)181, (char)245, (char)48, (char)2, (char)183, (char)245, (char)157, (char)221, (char)245, (char)145, (char)49, (char)239, (char)110, (char)90, (char)170, (char)242, (char)196, (char)137, (char)135, (char)164}));
            assert(pack.firmware_version_GET() == 773661032L);
            assert(pack.resolution_h_GET() == (char)54965);
            assert(pack.sensor_size_v_GET() == -1.2978638E38F);
            assert(pack.lens_id_GET() == (char)97);
            assert(pack.cam_definition_version_GET() == (char)32578);
            assert(pack.focal_length_GET() == 1.7357717E38F);
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
            assert(pack.cam_definition_uri_LEN(ph) == 10);
            assert(pack.cam_definition_uri_TRY(ph).equals("mfwnfzxgjo"));
            assert(pack.sensor_size_h_GET() == -9.449516E37F);
            assert(pack.time_boot_ms_GET() == 440645799L);
            assert(pack.resolution_v_GET() == (char)21908);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)229, (char)35, (char)179, (char)164, (char)142, (char)164, (char)174, (char)240, (char)37, (char)87, (char)169, (char)141, (char)63, (char)76, (char)20, (char)170, (char)142, (char)154, (char)127, (char)216, (char)208, (char)95, (char)167, (char)164, (char)109, (char)83, (char)53, (char)87, (char)117, (char)58, (char)120, (char)104}));
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.cam_definition_uri_SET("mfwnfzxgjo", PH) ;
        p259.sensor_size_h_SET(-9.449516E37F) ;
        p259.lens_id_SET((char)97) ;
        p259.time_boot_ms_SET(440645799L) ;
        p259.focal_length_SET(1.7357717E38F) ;
        p259.vendor_name_SET(new char[] {(char)110, (char)54, (char)210, (char)242, (char)181, (char)39, (char)217, (char)228, (char)194, (char)144, (char)87, (char)162, (char)181, (char)245, (char)48, (char)2, (char)183, (char)245, (char)157, (char)221, (char)245, (char)145, (char)49, (char)239, (char)110, (char)90, (char)170, (char)242, (char)196, (char)137, (char)135, (char)164}, 0) ;
        p259.model_name_SET(new char[] {(char)229, (char)35, (char)179, (char)164, (char)142, (char)164, (char)174, (char)240, (char)37, (char)87, (char)169, (char)141, (char)63, (char)76, (char)20, (char)170, (char)142, (char)154, (char)127, (char)216, (char)208, (char)95, (char)167, (char)164, (char)109, (char)83, (char)53, (char)87, (char)117, (char)58, (char)120, (char)104}, 0) ;
        p259.resolution_h_SET((char)54965) ;
        p259.sensor_size_v_SET(-1.2978638E38F) ;
        p259.resolution_v_SET((char)21908) ;
        p259.cam_definition_version_SET((char)32578) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE)) ;
        p259.firmware_version_SET(773661032L) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
            assert(pack.time_boot_ms_GET() == 3140333173L);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(3140333173L) ;
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.write_speed_GET() == -2.9154664E38F);
            assert(pack.used_capacity_GET() == -1.4272889E38F);
            assert(pack.storage_count_GET() == (char)173);
            assert(pack.storage_id_GET() == (char)86);
            assert(pack.read_speed_GET() == 1.7987122E38F);
            assert(pack.available_capacity_GET() == -1.3139751E38F);
            assert(pack.status_GET() == (char)241);
            assert(pack.total_capacity_GET() == -2.7816994E38F);
            assert(pack.time_boot_ms_GET() == 4206871002L);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.storage_id_SET((char)86) ;
        p261.used_capacity_SET(-1.4272889E38F) ;
        p261.storage_count_SET((char)173) ;
        p261.status_SET((char)241) ;
        p261.time_boot_ms_SET(4206871002L) ;
        p261.read_speed_SET(1.7987122E38F) ;
        p261.available_capacity_SET(-1.3139751E38F) ;
        p261.write_speed_SET(-2.9154664E38F) ;
        p261.total_capacity_SET(-2.7816994E38F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.video_status_GET() == (char)160);
            assert(pack.time_boot_ms_GET() == 2798640761L);
            assert(pack.image_status_GET() == (char)72);
            assert(pack.recording_time_ms_GET() == 1188154953L);
            assert(pack.image_interval_GET() == -1.5122767E38F);
            assert(pack.available_capacity_GET() == 3.3871229E38F);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.available_capacity_SET(3.3871229E38F) ;
        p262.recording_time_ms_SET(1188154953L) ;
        p262.time_boot_ms_SET(2798640761L) ;
        p262.video_status_SET((char)160) ;
        p262.image_interval_SET(-1.5122767E38F) ;
        p262.image_status_SET((char)72) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 733743518);
            assert(pack.camera_id_GET() == (char)205);
            assert(pack.alt_GET() == -341901192);
            assert(pack.time_boot_ms_GET() == 722667080L);
            assert(pack.capture_result_GET() == (byte) - 74);
            assert(pack.image_index_GET() == 205792567);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.6219257E38F, 1.3207871E38F, -1.9158239E38F, 1.9813111E38F}));
            assert(pack.relative_alt_GET() == -1095492943);
            assert(pack.time_utc_GET() == 1318954494334938195L);
            assert(pack.lon_GET() == -2123263163);
            assert(pack.file_url_LEN(ph) == 177);
            assert(pack.file_url_TRY(ph).equals("lWthoPeenyraeKWlsncljvfycIyzxahhvtxbipucpBaeumcWnvtufhgiiazsbkwwquvxsbgsmsbahgkcGtlxLqknaqxyfkebxgyzeGdplohwntxazjfOqbvfrjpjqdmeukHjijghvoqqeqtlvytwwgzbDDliawiEtBjwJghajuvxaoqWz"));
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.relative_alt_SET(-1095492943) ;
        p263.lat_SET(733743518) ;
        p263.q_SET(new float[] {-1.6219257E38F, 1.3207871E38F, -1.9158239E38F, 1.9813111E38F}, 0) ;
        p263.lon_SET(-2123263163) ;
        p263.time_boot_ms_SET(722667080L) ;
        p263.alt_SET(-341901192) ;
        p263.file_url_SET("lWthoPeenyraeKWlsncljvfycIyzxahhvtxbipucpBaeumcWnvtufhgiiazsbkwwquvxsbgsmsbahgkcGtlxLqknaqxyfkebxgyzeGdplohwntxazjfOqbvfrjpjqdmeukHjijghvoqqeqtlvytwwgzbDDliawiEtBjwJghajuvxaoqWz", PH) ;
        p263.capture_result_SET((byte) - 74) ;
        p263.time_utc_SET(1318954494334938195L) ;
        p263.image_index_SET(205792567) ;
        p263.camera_id_SET((char)205) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.takeoff_time_utc_GET() == 7883896090314171857L);
            assert(pack.time_boot_ms_GET() == 4214969308L);
            assert(pack.arming_time_utc_GET() == 6494365667482282988L);
            assert(pack.flight_uuid_GET() == 8822230676642043291L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.flight_uuid_SET(8822230676642043291L) ;
        p264.takeoff_time_utc_SET(7883896090314171857L) ;
        p264.arming_time_utc_SET(6494365667482282988L) ;
        p264.time_boot_ms_SET(4214969308L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 775804453L);
            assert(pack.roll_GET() == 1.2966556E38F);
            assert(pack.yaw_GET() == 2.9200968E38F);
            assert(pack.pitch_GET() == -8.601904E37F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.yaw_SET(2.9200968E38F) ;
        p265.pitch_SET(-8.601904E37F) ;
        p265.roll_SET(1.2966556E38F) ;
        p265.time_boot_ms_SET(775804453L) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)52007);
            assert(pack.target_system_GET() == (char)248);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)1, (char)166, (char)50, (char)244, (char)170, (char)36, (char)92, (char)67, (char)192, (char)78, (char)96, (char)121, (char)97, (char)192, (char)155, (char)235, (char)166, (char)74, (char)69, (char)82, (char)77, (char)122, (char)108, (char)222, (char)127, (char)171, (char)166, (char)181, (char)209, (char)235, (char)202, (char)132, (char)56, (char)139, (char)202, (char)61, (char)80, (char)156, (char)156, (char)215, (char)236, (char)100, (char)37, (char)213, (char)117, (char)25, (char)21, (char)186, (char)14, (char)135, (char)62, (char)251, (char)126, (char)123, (char)74, (char)102, (char)22, (char)169, (char)37, (char)7, (char)101, (char)153, (char)173, (char)156, (char)166, (char)176, (char)13, (char)225, (char)6, (char)207, (char)121, (char)212, (char)78, (char)82, (char)8, (char)215, (char)118, (char)54, (char)133, (char)145, (char)33, (char)0, (char)190, (char)209, (char)4, (char)95, (char)232, (char)144, (char)251, (char)97, (char)25, (char)27, (char)73, (char)111, (char)214, (char)98, (char)73, (char)54, (char)22, (char)50, (char)44, (char)41, (char)168, (char)75, (char)132, (char)54, (char)52, (char)34, (char)17, (char)34, (char)23, (char)167, (char)197, (char)118, (char)0, (char)245, (char)17, (char)194, (char)247, (char)122, (char)93, (char)105, (char)162, (char)5, (char)24, (char)205, (char)191, (char)13, (char)101, (char)17, (char)49, (char)211, (char)242, (char)70, (char)71, (char)124, (char)87, (char)46, (char)143, (char)210, (char)130, (char)105, (char)44, (char)224, (char)12, (char)175, (char)125, (char)87, (char)107, (char)92, (char)103, (char)216, (char)77, (char)140, (char)203, (char)45, (char)70, (char)147, (char)106, (char)132, (char)109, (char)194, (char)238, (char)254, (char)15, (char)167, (char)109, (char)158, (char)81, (char)187, (char)44, (char)61, (char)79, (char)221, (char)100, (char)102, (char)196, (char)137, (char)89, (char)138, (char)140, (char)187, (char)33, (char)195, (char)203, (char)170, (char)60, (char)25, (char)219, (char)32, (char)115, (char)185, (char)69, (char)124, (char)110, (char)13, (char)227, (char)123, (char)67, (char)72, (char)30, (char)158, (char)193, (char)201, (char)240, (char)101, (char)227, (char)197, (char)130, (char)178, (char)26, (char)226, (char)193, (char)217, (char)124, (char)24, (char)138, (char)225, (char)155, (char)135, (char)25, (char)56, (char)92, (char)85, (char)149, (char)155, (char)2, (char)55, (char)8, (char)64, (char)245, (char)173, (char)23, (char)90, (char)96, (char)166, (char)44, (char)176, (char)38, (char)73, (char)162, (char)120, (char)79, (char)48, (char)205, (char)13, (char)164, (char)40, (char)204}));
            assert(pack.length_GET() == (char)132);
            assert(pack.target_component_GET() == (char)243);
            assert(pack.first_message_offset_GET() == (char)237);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.sequence_SET((char)52007) ;
        p266.first_message_offset_SET((char)237) ;
        p266.length_SET((char)132) ;
        p266.target_system_SET((char)248) ;
        p266.data__SET(new char[] {(char)1, (char)166, (char)50, (char)244, (char)170, (char)36, (char)92, (char)67, (char)192, (char)78, (char)96, (char)121, (char)97, (char)192, (char)155, (char)235, (char)166, (char)74, (char)69, (char)82, (char)77, (char)122, (char)108, (char)222, (char)127, (char)171, (char)166, (char)181, (char)209, (char)235, (char)202, (char)132, (char)56, (char)139, (char)202, (char)61, (char)80, (char)156, (char)156, (char)215, (char)236, (char)100, (char)37, (char)213, (char)117, (char)25, (char)21, (char)186, (char)14, (char)135, (char)62, (char)251, (char)126, (char)123, (char)74, (char)102, (char)22, (char)169, (char)37, (char)7, (char)101, (char)153, (char)173, (char)156, (char)166, (char)176, (char)13, (char)225, (char)6, (char)207, (char)121, (char)212, (char)78, (char)82, (char)8, (char)215, (char)118, (char)54, (char)133, (char)145, (char)33, (char)0, (char)190, (char)209, (char)4, (char)95, (char)232, (char)144, (char)251, (char)97, (char)25, (char)27, (char)73, (char)111, (char)214, (char)98, (char)73, (char)54, (char)22, (char)50, (char)44, (char)41, (char)168, (char)75, (char)132, (char)54, (char)52, (char)34, (char)17, (char)34, (char)23, (char)167, (char)197, (char)118, (char)0, (char)245, (char)17, (char)194, (char)247, (char)122, (char)93, (char)105, (char)162, (char)5, (char)24, (char)205, (char)191, (char)13, (char)101, (char)17, (char)49, (char)211, (char)242, (char)70, (char)71, (char)124, (char)87, (char)46, (char)143, (char)210, (char)130, (char)105, (char)44, (char)224, (char)12, (char)175, (char)125, (char)87, (char)107, (char)92, (char)103, (char)216, (char)77, (char)140, (char)203, (char)45, (char)70, (char)147, (char)106, (char)132, (char)109, (char)194, (char)238, (char)254, (char)15, (char)167, (char)109, (char)158, (char)81, (char)187, (char)44, (char)61, (char)79, (char)221, (char)100, (char)102, (char)196, (char)137, (char)89, (char)138, (char)140, (char)187, (char)33, (char)195, (char)203, (char)170, (char)60, (char)25, (char)219, (char)32, (char)115, (char)185, (char)69, (char)124, (char)110, (char)13, (char)227, (char)123, (char)67, (char)72, (char)30, (char)158, (char)193, (char)201, (char)240, (char)101, (char)227, (char)197, (char)130, (char)178, (char)26, (char)226, (char)193, (char)217, (char)124, (char)24, (char)138, (char)225, (char)155, (char)135, (char)25, (char)56, (char)92, (char)85, (char)149, (char)155, (char)2, (char)55, (char)8, (char)64, (char)245, (char)173, (char)23, (char)90, (char)96, (char)166, (char)44, (char)176, (char)38, (char)73, (char)162, (char)120, (char)79, (char)48, (char)205, (char)13, (char)164, (char)40, (char)204}, 0) ;
        p266.target_component_SET((char)243) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.length_GET() == (char)195);
            assert(pack.target_system_GET() == (char)243);
            assert(pack.first_message_offset_GET() == (char)54);
            assert(pack.sequence_GET() == (char)59210);
            assert(pack.target_component_GET() == (char)34);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)11, (char)191, (char)73, (char)171, (char)160, (char)53, (char)222, (char)11, (char)149, (char)71, (char)163, (char)235, (char)237, (char)140, (char)69, (char)105, (char)222, (char)106, (char)24, (char)110, (char)196, (char)113, (char)226, (char)220, (char)7, (char)73, (char)131, (char)175, (char)44, (char)32, (char)236, (char)255, (char)52, (char)167, (char)127, (char)69, (char)201, (char)138, (char)59, (char)28, (char)113, (char)23, (char)135, (char)157, (char)242, (char)207, (char)20, (char)155, (char)63, (char)15, (char)73, (char)228, (char)52, (char)84, (char)4, (char)230, (char)68, (char)163, (char)51, (char)86, (char)105, (char)47, (char)158, (char)226, (char)90, (char)215, (char)190, (char)27, (char)104, (char)172, (char)93, (char)42, (char)98, (char)167, (char)153, (char)235, (char)172, (char)6, (char)37, (char)11, (char)25, (char)39, (char)99, (char)49, (char)94, (char)85, (char)184, (char)90, (char)76, (char)12, (char)124, (char)152, (char)141, (char)204, (char)220, (char)43, (char)174, (char)254, (char)121, (char)42, (char)197, (char)75, (char)100, (char)57, (char)41, (char)94, (char)101, (char)222, (char)63, (char)2, (char)30, (char)11, (char)156, (char)163, (char)213, (char)110, (char)170, (char)134, (char)156, (char)95, (char)67, (char)57, (char)68, (char)247, (char)186, (char)92, (char)79, (char)235, (char)76, (char)120, (char)87, (char)123, (char)181, (char)17, (char)133, (char)175, (char)203, (char)237, (char)167, (char)93, (char)232, (char)120, (char)181, (char)35, (char)55, (char)32, (char)173, (char)73, (char)32, (char)227, (char)195, (char)37, (char)138, (char)25, (char)182, (char)220, (char)119, (char)141, (char)109, (char)1, (char)194, (char)205, (char)198, (char)91, (char)45, (char)123, (char)209, (char)224, (char)185, (char)40, (char)60, (char)151, (char)193, (char)10, (char)69, (char)109, (char)69, (char)65, (char)128, (char)91, (char)179, (char)176, (char)27, (char)221, (char)34, (char)14, (char)29, (char)86, (char)94, (char)221, (char)197, (char)93, (char)74, (char)123, (char)38, (char)92, (char)2, (char)227, (char)91, (char)10, (char)102, (char)249, (char)241, (char)205, (char)233, (char)139, (char)217, (char)1, (char)202, (char)3, (char)140, (char)231, (char)168, (char)133, (char)69, (char)229, (char)210, (char)216, (char)74, (char)207, (char)190, (char)201, (char)169, (char)254, (char)151, (char)52, (char)163, (char)242, (char)117, (char)30, (char)126, (char)129, (char)82, (char)183, (char)177, (char)146, (char)225, (char)174, (char)72, (char)201, (char)46, (char)147, (char)43, (char)67, (char)147, (char)109, (char)180, (char)101, (char)220}));
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.sequence_SET((char)59210) ;
        p267.first_message_offset_SET((char)54) ;
        p267.target_system_SET((char)243) ;
        p267.target_component_SET((char)34) ;
        p267.data__SET(new char[] {(char)11, (char)191, (char)73, (char)171, (char)160, (char)53, (char)222, (char)11, (char)149, (char)71, (char)163, (char)235, (char)237, (char)140, (char)69, (char)105, (char)222, (char)106, (char)24, (char)110, (char)196, (char)113, (char)226, (char)220, (char)7, (char)73, (char)131, (char)175, (char)44, (char)32, (char)236, (char)255, (char)52, (char)167, (char)127, (char)69, (char)201, (char)138, (char)59, (char)28, (char)113, (char)23, (char)135, (char)157, (char)242, (char)207, (char)20, (char)155, (char)63, (char)15, (char)73, (char)228, (char)52, (char)84, (char)4, (char)230, (char)68, (char)163, (char)51, (char)86, (char)105, (char)47, (char)158, (char)226, (char)90, (char)215, (char)190, (char)27, (char)104, (char)172, (char)93, (char)42, (char)98, (char)167, (char)153, (char)235, (char)172, (char)6, (char)37, (char)11, (char)25, (char)39, (char)99, (char)49, (char)94, (char)85, (char)184, (char)90, (char)76, (char)12, (char)124, (char)152, (char)141, (char)204, (char)220, (char)43, (char)174, (char)254, (char)121, (char)42, (char)197, (char)75, (char)100, (char)57, (char)41, (char)94, (char)101, (char)222, (char)63, (char)2, (char)30, (char)11, (char)156, (char)163, (char)213, (char)110, (char)170, (char)134, (char)156, (char)95, (char)67, (char)57, (char)68, (char)247, (char)186, (char)92, (char)79, (char)235, (char)76, (char)120, (char)87, (char)123, (char)181, (char)17, (char)133, (char)175, (char)203, (char)237, (char)167, (char)93, (char)232, (char)120, (char)181, (char)35, (char)55, (char)32, (char)173, (char)73, (char)32, (char)227, (char)195, (char)37, (char)138, (char)25, (char)182, (char)220, (char)119, (char)141, (char)109, (char)1, (char)194, (char)205, (char)198, (char)91, (char)45, (char)123, (char)209, (char)224, (char)185, (char)40, (char)60, (char)151, (char)193, (char)10, (char)69, (char)109, (char)69, (char)65, (char)128, (char)91, (char)179, (char)176, (char)27, (char)221, (char)34, (char)14, (char)29, (char)86, (char)94, (char)221, (char)197, (char)93, (char)74, (char)123, (char)38, (char)92, (char)2, (char)227, (char)91, (char)10, (char)102, (char)249, (char)241, (char)205, (char)233, (char)139, (char)217, (char)1, (char)202, (char)3, (char)140, (char)231, (char)168, (char)133, (char)69, (char)229, (char)210, (char)216, (char)74, (char)207, (char)190, (char)201, (char)169, (char)254, (char)151, (char)52, (char)163, (char)242, (char)117, (char)30, (char)126, (char)129, (char)82, (char)183, (char)177, (char)146, (char)225, (char)174, (char)72, (char)201, (char)46, (char)147, (char)43, (char)67, (char)147, (char)109, (char)180, (char)101, (char)220}, 0) ;
        p267.length_SET((char)195) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)235);
            assert(pack.target_component_GET() == (char)22);
            assert(pack.sequence_GET() == (char)39072);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_component_SET((char)22) ;
        p268.sequence_SET((char)39072) ;
        p268.target_system_SET((char)235) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.framerate_GET() == -6.800592E37F);
            assert(pack.uri_LEN(ph) == 206);
            assert(pack.uri_TRY(ph).equals("uPzugtokrfdwNipkdPwXhcVOmpwkscQjtfgymjCwiyunuxngnwiogwbgujvvigrhddbwpfWsvogblpqqkzrniprfsqvaswntglnmmbraxWxvJdtEirsztpLtgywrqtSxyrkwqmawyouvgkvSasqqypcdiduposrsmbhgkoGhZlSvlKvulkbnhhzjpajxenfalFyeDxwngxhlxb"));
            assert(pack.camera_id_GET() == (char)109);
            assert(pack.rotation_GET() == (char)9851);
            assert(pack.bitrate_GET() == 1835343225L);
            assert(pack.resolution_v_GET() == (char)45523);
            assert(pack.resolution_h_GET() == (char)43848);
            assert(pack.status_GET() == (char)9);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.bitrate_SET(1835343225L) ;
        p269.camera_id_SET((char)109) ;
        p269.framerate_SET(-6.800592E37F) ;
        p269.resolution_h_SET((char)43848) ;
        p269.rotation_SET((char)9851) ;
        p269.uri_SET("uPzugtokrfdwNipkdPwXhcVOmpwkscQjtfgymjCwiyunuxngnwiogwbgujvvigrhddbwpfWsvogblpqqkzrniprfsqvaswntglnmmbraxWxvJdtEirsztpLtgywrqtSxyrkwqmawyouvgkvSasqqypcdiduposrsmbhgkoGhZlSvlKvulkbnhhzjpajxenfalFyeDxwngxhlxb", PH) ;
        p269.resolution_v_SET((char)45523) ;
        p269.status_SET((char)9) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.uri_LEN(ph) == 190);
            assert(pack.uri_TRY(ph).equals("viceoaqlpmiblrStfilcadVphlawtLftzviilnsftwickvgkykvxvezfSiqfqkjpldqQedpnxnsxbtlwuewwxNirgjwuveHgleqnardxuvpjsJhodvgtslcQewllydfdoXbdjpjewChdxdecedayocrphzllljusceyiakocvAglthZduESmagzqFlwsrq"));
            assert(pack.bitrate_GET() == 3991156841L);
            assert(pack.framerate_GET() == 1.7171072E38F);
            assert(pack.target_component_GET() == (char)23);
            assert(pack.resolution_v_GET() == (char)62389);
            assert(pack.rotation_GET() == (char)54037);
            assert(pack.target_system_GET() == (char)31);
            assert(pack.camera_id_GET() == (char)96);
            assert(pack.resolution_h_GET() == (char)37926);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.resolution_v_SET((char)62389) ;
        p270.bitrate_SET(3991156841L) ;
        p270.uri_SET("viceoaqlpmiblrStfilcadVphlawtLftzviilnsftwickvgkykvxvezfSiqfqkjpldqQedpnxnsxbtlwuewwxNirgjwuveHgleqnardxuvpjsJhodvgtslcQewllydfdoXbdjpjewChdxdecedayocrphzllljusceyiakocvAglthZduESmagzqFlwsrq", PH) ;
        p270.framerate_SET(1.7171072E38F) ;
        p270.rotation_SET((char)54037) ;
        p270.camera_id_SET((char)96) ;
        p270.resolution_h_SET((char)37926) ;
        p270.target_component_SET((char)23) ;
        p270.target_system_SET((char)31) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 17);
            assert(pack.password_TRY(ph).equals("lrXipuuvTpythfgrs"));
            assert(pack.ssid_LEN(ph) == 26);
            assert(pack.ssid_TRY(ph).equals("usozfwPmpuvifmYwmjdcruaykc"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("usozfwPmpuvifmYwmjdcruaykc", PH) ;
        p299.password_SET("lrXipuuvTpythfgrs", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == (char)55097);
            assert(pack.max_version_GET() == (char)12489);
            assert(pack.min_version_GET() == (char)2747);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)192, (char)61, (char)83, (char)157, (char)143, (char)106, (char)75, (char)55}));
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)87, (char)233, (char)72, (char)155, (char)183, (char)90, (char)73, (char)174}));
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.library_version_hash_SET(new char[] {(char)87, (char)233, (char)72, (char)155, (char)183, (char)90, (char)73, (char)174}, 0) ;
        p300.version_SET((char)55097) ;
        p300.min_version_SET((char)2747) ;
        p300.max_version_SET((char)12489) ;
        p300.spec_version_hash_SET(new char[] {(char)192, (char)61, (char)83, (char)157, (char)143, (char)106, (char)75, (char)55}, 0) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
            assert(pack.uptime_sec_GET() == 1328131686L);
            assert(pack.sub_mode_GET() == (char)240);
            assert(pack.vendor_specific_status_code_GET() == (char)25471);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
            assert(pack.time_usec_GET() == 2804262863574647617L);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.sub_mode_SET((char)240) ;
        p310.time_usec_SET(2804262863574647617L) ;
        p310.vendor_specific_status_code_SET((char)25471) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL) ;
        p310.uptime_sec_SET(1328131686L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.hw_version_minor_GET() == (char)102);
            assert(pack.uptime_sec_GET() == 595006959L);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)228, (char)17, (char)193, (char)120, (char)133, (char)31, (char)51, (char)97, (char)193, (char)250, (char)84, (char)110, (char)96, (char)36, (char)113, (char)191}));
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("RTivByuenR"));
            assert(pack.sw_vcs_commit_GET() == 4074461313L);
            assert(pack.sw_version_minor_GET() == (char)48);
            assert(pack.sw_version_major_GET() == (char)217);
            assert(pack.time_usec_GET() == 4890603137642975137L);
            assert(pack.hw_version_major_GET() == (char)71);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.uptime_sec_SET(595006959L) ;
        p311.sw_version_minor_SET((char)48) ;
        p311.sw_vcs_commit_SET(4074461313L) ;
        p311.name_SET("RTivByuenR", PH) ;
        p311.hw_version_minor_SET((char)102) ;
        p311.sw_version_major_SET((char)217) ;
        p311.hw_version_major_SET((char)71) ;
        p311.hw_unique_id_SET(new char[] {(char)228, (char)17, (char)193, (char)120, (char)133, (char)31, (char)51, (char)97, (char)193, (char)250, (char)84, (char)110, (char)96, (char)36, (char)113, (char)191}, 0) ;
        p311.time_usec_SET(4890603137642975137L) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("qcaroduUmSx"));
            assert(pack.target_component_GET() == (char)199);
            assert(pack.param_index_GET() == (short) -27151);
            assert(pack.target_system_GET() == (char)245);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_id_SET("qcaroduUmSx", PH) ;
        p320.target_system_SET((char)245) ;
        p320.param_index_SET((short) -27151) ;
        p320.target_component_SET((char)199) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)246);
            assert(pack.target_component_GET() == (char)219);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)246) ;
        p321.target_component_SET((char)219) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)894);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("hk"));
            assert(pack.param_index_GET() == (char)27361);
            assert(pack.param_value_LEN(ph) == 79);
            assert(pack.param_value_TRY(ph).equals("pczzthFxcirLtcvTHiFwzfuwsrthvdbuinybklrbkwxtdjwyclwjipjNavycobanmxtcaffWfkjvisu"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64) ;
        p322.param_index_SET((char)27361) ;
        p322.param_count_SET((char)894) ;
        p322.param_id_SET("hk", PH) ;
        p322.param_value_SET("pczzthFxcirLtcvTHiFwzfuwsrthvdbuinybklrbkwxtdjwyclwjipjNavycobanmxtcaffWfkjvisu", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)221);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("xo"));
            assert(pack.target_component_GET() == (char)101);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
            assert(pack.param_value_LEN(ph) == 104);
            assert(pack.param_value_TRY(ph).equals("UuiLRoihjpzfvVtimvljecgwwihnjgucKijivsyoxiaqnvbyghmpgvfDxybipZrgZqkppkmbncskXlXvnkLfmflwccKiksuvfdwceajN"));
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)221) ;
        p323.target_component_SET((char)101) ;
        p323.param_id_SET("xo", PH) ;
        p323.param_value_SET("UuiLRoihjpzfvVtimvljecgwwihnjgucKijivsyoxiaqnvbyghmpgvfDxybipZrgZqkppkmbncskXlXvnkLfmflwccKiksuvfdwceajN", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_FAILED);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("wooxjfwmfmcdvam"));
            assert(pack.param_value_LEN(ph) == 98);
            assert(pack.param_value_TRY(ph).equals("rohebdlbokikjmngvFydghpMrgbpqramUeDwlkxxoikadWsxpdbnhczdxrxoarxxgsjveorarmVbdwkskkwkpzrjhnxgplhmya"));
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64) ;
        p324.param_id_SET("wooxjfwmfmcdvam", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED) ;
        p324.param_value_SET("rohebdlbokikjmngvFydghpMrgbpqramUeDwlkxxoikadWsxpdbnhczdxrxoarxxgsjveorarmVbdwkskkwkpzrjhnxgplhmya", PH) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)42711, (char)12813, (char)64789, (char)33701, (char)23054, (char)39413, (char)43886, (char)61122, (char)65168, (char)41738, (char)61157, (char)6067, (char)31343, (char)8364, (char)50206, (char)41295, (char)64150, (char)28849, (char)1038, (char)10869, (char)23942, (char)25146, (char)45076, (char)47446, (char)64352, (char)46076, (char)44737, (char)19694, (char)3880, (char)64924, (char)38544, (char)25193, (char)34512, (char)64325, (char)54462, (char)41586, (char)60714, (char)399, (char)55882, (char)9948, (char)30579, (char)5995, (char)40372, (char)22585, (char)6078, (char)47998, (char)58534, (char)22163, (char)51733, (char)14443, (char)35237, (char)41475, (char)30943, (char)32395, (char)15529, (char)23334, (char)57630, (char)26850, (char)5125, (char)49380, (char)26589, (char)29253, (char)18474, (char)13403, (char)13337, (char)59177, (char)24033, (char)24123, (char)21621, (char)52773, (char)3083, (char)3182}));
            assert(pack.increment_GET() == (char)210);
            assert(pack.max_distance_GET() == (char)14720);
            assert(pack.time_usec_GET() == 2221429454414772100L);
            assert(pack.min_distance_GET() == (char)4647);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.increment_SET((char)210) ;
        p330.time_usec_SET(2221429454414772100L) ;
        p330.max_distance_SET((char)14720) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p330.distances_SET(new char[] {(char)42711, (char)12813, (char)64789, (char)33701, (char)23054, (char)39413, (char)43886, (char)61122, (char)65168, (char)41738, (char)61157, (char)6067, (char)31343, (char)8364, (char)50206, (char)41295, (char)64150, (char)28849, (char)1038, (char)10869, (char)23942, (char)25146, (char)45076, (char)47446, (char)64352, (char)46076, (char)44737, (char)19694, (char)3880, (char)64924, (char)38544, (char)25193, (char)34512, (char)64325, (char)54462, (char)41586, (char)60714, (char)399, (char)55882, (char)9948, (char)30579, (char)5995, (char)40372, (char)22585, (char)6078, (char)47998, (char)58534, (char)22163, (char)51733, (char)14443, (char)35237, (char)41475, (char)30943, (char)32395, (char)15529, (char)23334, (char)57630, (char)26850, (char)5125, (char)49380, (char)26589, (char)29253, (char)18474, (char)13403, (char)13337, (char)59177, (char)24033, (char)24123, (char)21621, (char)52773, (char)3083, (char)3182}, 0) ;
        p330.min_distance_SET((char)4647) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
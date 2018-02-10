
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
            long id = id__W(src);
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
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_GROUND_ROVER);
            assert(pack.mavlink_version_GET() == (char)231);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_SLUGS);
            assert(pack.custom_mode_GET() == 2910894931L);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_UNINIT);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_SLUGS) ;
        p0.mavlink_version_SET((char)231) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED)) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_UNINIT) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_GROUND_ROVER) ;
        p0.custom_mode_SET(2910894931L) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_comm_GET() == (char)31974);
            assert(pack.load_GET() == (char)14338);
            assert(pack.voltage_battery_GET() == (char)46643);
            assert(pack.current_battery_GET() == (short) -17783);
            assert(pack.battery_remaining_GET() == (byte)110);
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.drop_rate_comm_GET() == (char)12099);
            assert(pack.errors_count2_GET() == (char)33675);
            assert(pack.errors_count1_GET() == (char)23902);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
            assert(pack.errors_count4_GET() == (char)1935);
            assert(pack.errors_count3_GET() == (char)42474);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS));
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.load_SET((char)14338) ;
        p1.errors_count3_SET((char)42474) ;
        p1.current_battery_SET((short) -17783) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS)) ;
        p1.errors_count1_SET((char)23902) ;
        p1.errors_count2_SET((char)33675) ;
        p1.drop_rate_comm_SET((char)12099) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW)) ;
        p1.errors_count4_SET((char)1935) ;
        p1.voltage_battery_SET((char)46643) ;
        p1.battery_remaining_SET((byte)110) ;
        p1.errors_comm_SET((char)31974) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 3320947830263957004L);
            assert(pack.time_boot_ms_GET() == 4197515810L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(4197515810L) ;
        p2.time_unix_usec_SET(3320947830263957004L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.vy_GET() == -4.1338809E37F);
            assert(pack.afy_GET() == 3.2484315E38F);
            assert(pack.afz_GET() == 8.1348607E37F);
            assert(pack.yaw_rate_GET() == -2.9228238E37F);
            assert(pack.y_GET() == 2.0780032E38F);
            assert(pack.z_GET() == 3.3652284E38F);
            assert(pack.type_mask_GET() == (char)52185);
            assert(pack.vz_GET() == -6.0116636E37F);
            assert(pack.vx_GET() == -1.2800157E38F);
            assert(pack.time_boot_ms_GET() == 2627395272L);
            assert(pack.yaw_GET() == -9.663284E37F);
            assert(pack.x_GET() == -3.2022262E38F);
            assert(pack.afx_GET() == 1.5206349E38F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.x_SET(-3.2022262E38F) ;
        p3.y_SET(2.0780032E38F) ;
        p3.vy_SET(-4.1338809E37F) ;
        p3.vx_SET(-1.2800157E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p3.yaw_SET(-9.663284E37F) ;
        p3.type_mask_SET((char)52185) ;
        p3.vz_SET(-6.0116636E37F) ;
        p3.yaw_rate_SET(-2.9228238E37F) ;
        p3.z_SET(3.3652284E38F) ;
        p3.afx_SET(1.5206349E38F) ;
        p3.afy_SET(3.2484315E38F) ;
        p3.afz_SET(8.1348607E37F) ;
        p3.time_boot_ms_SET(2627395272L) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 2823680439L);
            assert(pack.target_component_GET() == (char)221);
            assert(pack.target_system_GET() == (char)104);
            assert(pack.time_usec_GET() == 4493198995174599512L);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.target_component_SET((char)221) ;
        p4.target_system_SET((char)104) ;
        p4.seq_SET(2823680439L) ;
        p4.time_usec_SET(4493198995174599512L) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.passkey_LEN(ph) == 21);
            assert(pack.passkey_TRY(ph).equals("jybOryRxetcOduzkjkxdt"));
            assert(pack.target_system_GET() == (char)113);
            assert(pack.control_request_GET() == (char)230);
            assert(pack.version_GET() == (char)80);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.passkey_SET("jybOryRxetcOduzkjkxdt", PH) ;
        p5.target_system_SET((char)113) ;
        p5.version_SET((char)80) ;
        p5.control_request_SET((char)230) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.ack_GET() == (char)80);
            assert(pack.gcs_system_id_GET() == (char)219);
            assert(pack.control_request_GET() == (char)72);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.control_request_SET((char)72) ;
        p6.ack_SET((char)80) ;
        p6.gcs_system_id_SET((char)219) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 19);
            assert(pack.key_TRY(ph).equals("nyzaenquwljSswisFkg"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("nyzaenquwljSswisFkg", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.custom_mode_GET() == 2731614285L);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_MANUAL_ARMED);
            assert(pack.target_system_GET() == (char)49);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.custom_mode_SET(2731614285L) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_MANUAL_ARMED) ;
        p11.target_system_SET((char)49) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("fvxaojuageLq"));
            assert(pack.target_system_GET() == (char)120);
            assert(pack.target_component_GET() == (char)68);
            assert(pack.param_index_GET() == (short)22901);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_component_SET((char)68) ;
        p20.target_system_SET((char)120) ;
        p20.param_id_SET("fvxaojuageLq", PH) ;
        p20.param_index_SET((short)22901) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)158);
            assert(pack.target_component_GET() == (char)24);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)24) ;
        p21.target_system_SET((char)158) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("DvuchGyplv"));
            assert(pack.param_index_GET() == (char)13537);
            assert(pack.param_value_GET() == 2.1307896E38F);
            assert(pack.param_count_GET() == (char)32616);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64) ;
        p22.param_id_SET("DvuchGyplv", PH) ;
        p22.param_index_SET((char)13537) ;
        p22.param_value_SET(2.1307896E38F) ;
        p22.param_count_SET((char)32616) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
            assert(pack.target_component_GET() == (char)216);
            assert(pack.param_value_GET() == 2.9017197E37F);
            assert(pack.target_system_GET() == (char)165);
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("wgoJoii"));
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.target_system_SET((char)165) ;
        p23.param_id_SET("wgoJoii", PH) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8) ;
        p23.target_component_SET((char)216) ;
        p23.param_value_SET(2.9017197E37F) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -1027907407);
            assert(pack.epv_GET() == (char)6927);
            assert(pack.hdg_acc_TRY(ph) == 4093610220L);
            assert(pack.time_usec_GET() == 9044486710287476649L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
            assert(pack.vel_acc_TRY(ph) == 2600658325L);
            assert(pack.alt_ellipsoid_TRY(ph) == 223301669);
            assert(pack.v_acc_TRY(ph) == 3392426695L);
            assert(pack.satellites_visible_GET() == (char)163);
            assert(pack.lat_GET() == -766047639);
            assert(pack.h_acc_TRY(ph) == 1441968215L);
            assert(pack.lon_GET() == -1454209564);
            assert(pack.eph_GET() == (char)55686);
            assert(pack.cog_GET() == (char)20967);
            assert(pack.vel_GET() == (char)13899);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.alt_ellipsoid_SET(223301669, PH) ;
        p24.h_acc_SET(1441968215L, PH) ;
        p24.v_acc_SET(3392426695L, PH) ;
        p24.vel_acc_SET(2600658325L, PH) ;
        p24.alt_SET(-1027907407) ;
        p24.lat_SET(-766047639) ;
        p24.cog_SET((char)20967) ;
        p24.lon_SET(-1454209564) ;
        p24.epv_SET((char)6927) ;
        p24.time_usec_SET(9044486710287476649L) ;
        p24.hdg_acc_SET(4093610220L, PH) ;
        p24.vel_SET((char)13899) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED) ;
        p24.eph_SET((char)55686) ;
        p24.satellites_visible_SET((char)163) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)9, (char)118, (char)159, (char)239, (char)188, (char)49, (char)59, (char)145, (char)157, (char)32, (char)121, (char)27, (char)81, (char)140, (char)242, (char)242, (char)97, (char)62, (char)12, (char)213}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)30, (char)149, (char)175, (char)37, (char)61, (char)182, (char)25, (char)72, (char)211, (char)125, (char)199, (char)132, (char)146, (char)151, (char)151, (char)232, (char)183, (char)73, (char)77, (char)229}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)224, (char)37, (char)92, (char)20, (char)180, (char)250, (char)188, (char)218, (char)137, (char)217, (char)186, (char)181, (char)210, (char)2, (char)123, (char)126, (char)192, (char)186, (char)215, (char)141}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)74, (char)195, (char)116, (char)124, (char)69, (char)0, (char)179, (char)114, (char)195, (char)221, (char)106, (char)235, (char)128, (char)253, (char)216, (char)155, (char)86, (char)2, (char)36, (char)30}));
            assert(pack.satellites_visible_GET() == (char)53);
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)196, (char)22, (char)132, (char)168, (char)72, (char)143, (char)49, (char)217, (char)170, (char)68, (char)247, (char)116, (char)84, (char)207, (char)240, (char)79, (char)226, (char)215, (char)27, (char)72}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_used_SET(new char[] {(char)196, (char)22, (char)132, (char)168, (char)72, (char)143, (char)49, (char)217, (char)170, (char)68, (char)247, (char)116, (char)84, (char)207, (char)240, (char)79, (char)226, (char)215, (char)27, (char)72}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)74, (char)195, (char)116, (char)124, (char)69, (char)0, (char)179, (char)114, (char)195, (char)221, (char)106, (char)235, (char)128, (char)253, (char)216, (char)155, (char)86, (char)2, (char)36, (char)30}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)9, (char)118, (char)159, (char)239, (char)188, (char)49, (char)59, (char)145, (char)157, (char)32, (char)121, (char)27, (char)81, (char)140, (char)242, (char)242, (char)97, (char)62, (char)12, (char)213}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)30, (char)149, (char)175, (char)37, (char)61, (char)182, (char)25, (char)72, (char)211, (char)125, (char)199, (char)132, (char)146, (char)151, (char)151, (char)232, (char)183, (char)73, (char)77, (char)229}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)224, (char)37, (char)92, (char)20, (char)180, (char)250, (char)188, (char)218, (char)137, (char)217, (char)186, (char)181, (char)210, (char)2, (char)123, (char)126, (char)192, (char)186, (char)215, (char)141}, 0) ;
        p25.satellites_visible_SET((char)53) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short) -19464);
            assert(pack.xgyro_GET() == (short)6241);
            assert(pack.zacc_GET() == (short)17771);
            assert(pack.time_boot_ms_GET() == 1895287737L);
            assert(pack.ygyro_GET() == (short) -716);
            assert(pack.zgyro_GET() == (short) -873);
            assert(pack.ymag_GET() == (short)26331);
            assert(pack.xacc_GET() == (short) -18549);
            assert(pack.zmag_GET() == (short)12097);
            assert(pack.xmag_GET() == (short)27533);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.zmag_SET((short)12097) ;
        p26.zacc_SET((short)17771) ;
        p26.ymag_SET((short)26331) ;
        p26.yacc_SET((short) -19464) ;
        p26.time_boot_ms_SET(1895287737L) ;
        p26.xgyro_SET((short)6241) ;
        p26.ygyro_SET((short) -716) ;
        p26.xacc_SET((short) -18549) ;
        p26.xmag_SET((short)27533) ;
        p26.zgyro_SET((short) -873) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short)14541);
            assert(pack.xmag_GET() == (short)30459);
            assert(pack.yacc_GET() == (short)4812);
            assert(pack.ygyro_GET() == (short)32750);
            assert(pack.xacc_GET() == (short)2497);
            assert(pack.ymag_GET() == (short)1498);
            assert(pack.time_usec_GET() == 826244408220166682L);
            assert(pack.xgyro_GET() == (short)30783);
            assert(pack.zgyro_GET() == (short) -10342);
            assert(pack.zmag_GET() == (short) -23489);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.ymag_SET((short)1498) ;
        p27.zacc_SET((short)14541) ;
        p27.xmag_SET((short)30459) ;
        p27.xgyro_SET((short)30783) ;
        p27.zmag_SET((short) -23489) ;
        p27.zgyro_SET((short) -10342) ;
        p27.ygyro_SET((short)32750) ;
        p27.yacc_SET((short)4812) ;
        p27.time_usec_SET(826244408220166682L) ;
        p27.xacc_SET((short)2497) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7460369694160685664L);
            assert(pack.press_diff1_GET() == (short)8600);
            assert(pack.temperature_GET() == (short) -8946);
            assert(pack.press_diff2_GET() == (short)10686);
            assert(pack.press_abs_GET() == (short) -16757);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_diff2_SET((short)10686) ;
        p28.press_diff1_SET((short)8600) ;
        p28.time_usec_SET(7460369694160685664L) ;
        p28.press_abs_SET((short) -16757) ;
        p28.temperature_SET((short) -8946) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -3907);
            assert(pack.press_diff_GET() == -8.2888387E37F);
            assert(pack.press_abs_GET() == -8.2909957E37F);
            assert(pack.time_boot_ms_GET() == 1411539781L);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_abs_SET(-8.2909957E37F) ;
        p29.press_diff_SET(-8.2888387E37F) ;
        p29.temperature_SET((short) -3907) ;
        p29.time_boot_ms_SET(1411539781L) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 3.9176973E36F);
            assert(pack.yaw_GET() == 5.446403E37F);
            assert(pack.pitchspeed_GET() == 2.9321238E37F);
            assert(pack.rollspeed_GET() == 5.825678E37F);
            assert(pack.pitch_GET() == 1.1653529E38F);
            assert(pack.roll_GET() == -1.808453E38F);
            assert(pack.time_boot_ms_GET() == 2976266694L);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.yawspeed_SET(3.9176973E36F) ;
        p30.pitch_SET(1.1653529E38F) ;
        p30.yaw_SET(5.446403E37F) ;
        p30.pitchspeed_SET(2.9321238E37F) ;
        p30.rollspeed_SET(5.825678E37F) ;
        p30.roll_SET(-1.808453E38F) ;
        p30.time_boot_ms_SET(2976266694L) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q2_GET() == -2.7676843E38F);
            assert(pack.q1_GET() == -1.3485436E38F);
            assert(pack.yawspeed_GET() == -9.304779E37F);
            assert(pack.q4_GET() == 9.437307E37F);
            assert(pack.q3_GET() == 2.6072533E38F);
            assert(pack.pitchspeed_GET() == -3.950643E37F);
            assert(pack.time_boot_ms_GET() == 3762996576L);
            assert(pack.rollspeed_GET() == 2.1457808E37F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q3_SET(2.6072533E38F) ;
        p31.time_boot_ms_SET(3762996576L) ;
        p31.q4_SET(9.437307E37F) ;
        p31.pitchspeed_SET(-3.950643E37F) ;
        p31.q2_SET(-2.7676843E38F) ;
        p31.yawspeed_SET(-9.304779E37F) ;
        p31.rollspeed_SET(2.1457808E37F) ;
        p31.q1_SET(-1.3485436E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 2.6562276E38F);
            assert(pack.x_GET() == 2.8272284E38F);
            assert(pack.vx_GET() == -1.5947594E38F);
            assert(pack.y_GET() == -1.357187E38F);
            assert(pack.z_GET() == 2.0638425E38F);
            assert(pack.time_boot_ms_GET() == 1761836979L);
            assert(pack.vz_GET() == 2.5166193E38F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.vy_SET(2.6562276E38F) ;
        p32.vx_SET(-1.5947594E38F) ;
        p32.x_SET(2.8272284E38F) ;
        p32.time_boot_ms_SET(1761836979L) ;
        p32.y_SET(-1.357187E38F) ;
        p32.vz_SET(2.5166193E38F) ;
        p32.z_SET(2.0638425E38F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 4024324559L);
            assert(pack.hdg_GET() == (char)7101);
            assert(pack.relative_alt_GET() == 675233442);
            assert(pack.vx_GET() == (short) -31048);
            assert(pack.vz_GET() == (short)2776);
            assert(pack.lat_GET() == 1360889989);
            assert(pack.alt_GET() == -75942729);
            assert(pack.lon_GET() == 787297627);
            assert(pack.vy_GET() == (short)14929);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.lat_SET(1360889989) ;
        p33.time_boot_ms_SET(4024324559L) ;
        p33.vy_SET((short)14929) ;
        p33.relative_alt_SET(675233442) ;
        p33.vx_SET((short) -31048) ;
        p33.hdg_SET((char)7101) ;
        p33.vz_SET((short)2776) ;
        p33.alt_SET(-75942729) ;
        p33.lon_SET(787297627) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan1_scaled_GET() == (short)12244);
            assert(pack.chan2_scaled_GET() == (short)32347);
            assert(pack.chan3_scaled_GET() == (short) -15198);
            assert(pack.port_GET() == (char)24);
            assert(pack.chan4_scaled_GET() == (short) -14208);
            assert(pack.time_boot_ms_GET() == 1429514109L);
            assert(pack.chan6_scaled_GET() == (short)3597);
            assert(pack.chan8_scaled_GET() == (short)16368);
            assert(pack.chan7_scaled_GET() == (short) -5923);
            assert(pack.chan5_scaled_GET() == (short)26769);
            assert(pack.rssi_GET() == (char)181);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan3_scaled_SET((short) -15198) ;
        p34.chan6_scaled_SET((short)3597) ;
        p34.rssi_SET((char)181) ;
        p34.chan4_scaled_SET((short) -14208) ;
        p34.time_boot_ms_SET(1429514109L) ;
        p34.chan7_scaled_SET((short) -5923) ;
        p34.chan5_scaled_SET((short)26769) ;
        p34.port_SET((char)24) ;
        p34.chan8_scaled_SET((short)16368) ;
        p34.chan2_scaled_SET((short)32347) ;
        p34.chan1_scaled_SET((short)12244) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)81);
            assert(pack.time_boot_ms_GET() == 3193062477L);
            assert(pack.chan8_raw_GET() == (char)62740);
            assert(pack.chan2_raw_GET() == (char)59556);
            assert(pack.port_GET() == (char)64);
            assert(pack.chan7_raw_GET() == (char)479);
            assert(pack.chan4_raw_GET() == (char)35904);
            assert(pack.chan3_raw_GET() == (char)27833);
            assert(pack.chan6_raw_GET() == (char)673);
            assert(pack.chan5_raw_GET() == (char)32459);
            assert(pack.chan1_raw_GET() == (char)24642);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan8_raw_SET((char)62740) ;
        p35.rssi_SET((char)81) ;
        p35.chan2_raw_SET((char)59556) ;
        p35.port_SET((char)64) ;
        p35.chan3_raw_SET((char)27833) ;
        p35.chan5_raw_SET((char)32459) ;
        p35.chan7_raw_SET((char)479) ;
        p35.time_boot_ms_SET(3193062477L) ;
        p35.chan6_raw_SET((char)673) ;
        p35.chan1_raw_SET((char)24642) ;
        p35.chan4_raw_SET((char)35904) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo12_raw_TRY(ph) == (char)31189);
            assert(pack.time_usec_GET() == 1061236357L);
            assert(pack.servo3_raw_GET() == (char)33796);
            assert(pack.servo16_raw_TRY(ph) == (char)6906);
            assert(pack.servo15_raw_TRY(ph) == (char)15892);
            assert(pack.servo13_raw_TRY(ph) == (char)62430);
            assert(pack.servo8_raw_GET() == (char)9351);
            assert(pack.servo6_raw_GET() == (char)25913);
            assert(pack.servo14_raw_TRY(ph) == (char)3155);
            assert(pack.servo7_raw_GET() == (char)23851);
            assert(pack.port_GET() == (char)120);
            assert(pack.servo10_raw_TRY(ph) == (char)15058);
            assert(pack.servo11_raw_TRY(ph) == (char)45090);
            assert(pack.servo2_raw_GET() == (char)60402);
            assert(pack.servo4_raw_GET() == (char)58847);
            assert(pack.servo9_raw_TRY(ph) == (char)25449);
            assert(pack.servo5_raw_GET() == (char)33435);
            assert(pack.servo1_raw_GET() == (char)25502);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo3_raw_SET((char)33796) ;
        p36.servo2_raw_SET((char)60402) ;
        p36.servo8_raw_SET((char)9351) ;
        p36.servo5_raw_SET((char)33435) ;
        p36.servo16_raw_SET((char)6906, PH) ;
        p36.servo6_raw_SET((char)25913) ;
        p36.servo14_raw_SET((char)3155, PH) ;
        p36.time_usec_SET(1061236357L) ;
        p36.servo13_raw_SET((char)62430, PH) ;
        p36.servo11_raw_SET((char)45090, PH) ;
        p36.servo7_raw_SET((char)23851) ;
        p36.servo10_raw_SET((char)15058, PH) ;
        p36.servo12_raw_SET((char)31189, PH) ;
        p36.servo1_raw_SET((char)25502) ;
        p36.port_SET((char)120) ;
        p36.servo15_raw_SET((char)15892, PH) ;
        p36.servo4_raw_SET((char)58847) ;
        p36.servo9_raw_SET((char)25449, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short)24361);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.end_index_GET() == (short) -31545);
            assert(pack.target_system_GET() == (char)41);
            assert(pack.target_component_GET() == (char)95);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_component_SET((char)95) ;
        p37.end_index_SET((short) -31545) ;
        p37.start_index_SET((short)24361) ;
        p37.target_system_SET((char)41) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)251);
            assert(pack.end_index_GET() == (short)20042);
            assert(pack.start_index_GET() == (short)29712);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)126);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.end_index_SET((short)20042) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p38.start_index_SET((short)29712) ;
        p38.target_system_SET((char)126) ;
        p38.target_component_SET((char)251) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_DELAY);
            assert(pack.y_GET() == -2.4518815E38F);
            assert(pack.param2_GET() == 3.3736987E38F);
            assert(pack.x_GET() == -4.4250975E37F);
            assert(pack.seq_GET() == (char)61697);
            assert(pack.param1_GET() == -3.0018382E38F);
            assert(pack.autocontinue_GET() == (char)239);
            assert(pack.target_component_GET() == (char)32);
            assert(pack.param4_GET() == -2.8777157E38F);
            assert(pack.target_system_GET() == (char)139);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.current_GET() == (char)129);
            assert(pack.param3_GET() == 2.5740372E38F);
            assert(pack.z_GET() == 2.1036271E37F);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.z_SET(2.1036271E37F) ;
        p39.seq_SET((char)61697) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p39.command_SET(MAV_CMD.MAV_CMD_NAV_DELAY) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p39.current_SET((char)129) ;
        p39.autocontinue_SET((char)239) ;
        p39.param2_SET(3.3736987E38F) ;
        p39.x_SET(-4.4250975E37F) ;
        p39.target_system_SET((char)139) ;
        p39.param1_SET(-3.0018382E38F) ;
        p39.param3_SET(2.5740372E38F) ;
        p39.target_component_SET((char)32) ;
        p39.y_SET(-2.4518815E38F) ;
        p39.param4_SET(-2.8777157E38F) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)42049);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)58);
            assert(pack.target_component_GET() == (char)99);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_component_SET((char)99) ;
        p40.target_system_SET((char)58) ;
        p40.seq_SET((char)42049) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)59897);
            assert(pack.target_component_GET() == (char)127);
            assert(pack.target_system_GET() == (char)146);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)146) ;
        p41.seq_SET((char)59897) ;
        p41.target_component_SET((char)127) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)38668);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)38668) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)168);
            assert(pack.target_component_GET() == (char)82);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p43.target_component_SET((char)82) ;
        p43.target_system_SET((char)168) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)92);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)53);
            assert(pack.count_GET() == (char)12419);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p44.target_component_SET((char)92) ;
        p44.count_SET((char)12419) ;
        p44.target_system_SET((char)53) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_component_GET() == (char)195);
            assert(pack.target_system_GET() == (char)138);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)138) ;
        p45.target_component_SET((char)195) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)18298);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)18298) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM5_X);
            assert(pack.target_component_GET() == (char)63);
            assert(pack.target_system_GET() == (char)152);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM5_X) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p47.target_component_SET((char)63) ;
        p47.target_system_SET((char)152) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == 1690152791);
            assert(pack.time_usec_TRY(ph) == 8815791099998684004L);
            assert(pack.target_system_GET() == (char)234);
            assert(pack.altitude_GET() == 87618183);
            assert(pack.latitude_GET() == 1097073395);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.longitude_SET(1690152791) ;
        p48.latitude_SET(1097073395) ;
        p48.time_usec_SET(8815791099998684004L, PH) ;
        p48.target_system_SET((char)234) ;
        p48.altitude_SET(87618183) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 8987830216015430224L);
            assert(pack.longitude_GET() == -66282179);
            assert(pack.latitude_GET() == 1024947643);
            assert(pack.altitude_GET() == -1124658387);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.altitude_SET(-1124658387) ;
        p49.latitude_SET(1024947643) ;
        p49.time_usec_SET(8987830216015430224L, PH) ;
        p49.longitude_SET(-66282179) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)54);
            assert(pack.param_value_max_GET() == -4.280792E37F);
            assert(pack.param_value_min_GET() == 1.6001662E38F);
            assert(pack.param_index_GET() == (short) -2306);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("cryx"));
            assert(pack.target_component_GET() == (char)142);
            assert(pack.parameter_rc_channel_index_GET() == (char)37);
            assert(pack.scale_GET() == 2.324554E38F);
            assert(pack.param_value0_GET() == 3.2719431E38F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.param_id_SET("cryx", PH) ;
        p50.param_value_min_SET(1.6001662E38F) ;
        p50.target_component_SET((char)142) ;
        p50.param_value0_SET(3.2719431E38F) ;
        p50.param_value_max_SET(-4.280792E37F) ;
        p50.target_system_SET((char)54) ;
        p50.param_index_SET((short) -2306) ;
        p50.parameter_rc_channel_index_SET((char)37) ;
        p50.scale_SET(2.324554E38F) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)98);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)59);
            assert(pack.seq_GET() == (char)53742);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_component_SET((char)98) ;
        p51.seq_SET((char)53742) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p51.target_system_SET((char)59) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1z_GET() == 2.7015358E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.p1y_GET() == -8.078363E37F);
            assert(pack.p2x_GET() == -3.0715022E38F);
            assert(pack.target_system_GET() == (char)170);
            assert(pack.target_component_GET() == (char)106);
            assert(pack.p1x_GET() == -1.014541E38F);
            assert(pack.p2y_GET() == 3.167536E38F);
            assert(pack.p2z_GET() == -8.762571E37F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p54.p1z_SET(2.7015358E38F) ;
        p54.target_component_SET((char)106) ;
        p54.target_system_SET((char)170) ;
        p54.p2y_SET(3.167536E38F) ;
        p54.p2x_SET(-3.0715022E38F) ;
        p54.p2z_SET(-8.762571E37F) ;
        p54.p1x_SET(-1.014541E38F) ;
        p54.p1y_SET(-8.078363E37F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.p2y_GET() == -1.3047162E37F);
            assert(pack.p1y_GET() == -6.3716145E37F);
            assert(pack.p1x_GET() == -5.763821E37F);
            assert(pack.p1z_GET() == -2.1505621E38F);
            assert(pack.p2z_GET() == -6.9160355E37F);
            assert(pack.p2x_GET() == 2.9260518E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2z_SET(-6.9160355E37F) ;
        p55.p2x_SET(2.9260518E38F) ;
        p55.p1x_SET(-5.763821E37F) ;
        p55.p1z_SET(-2.1505621E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p55.p2y_SET(-1.3047162E37F) ;
        p55.p1y_SET(-6.3716145E37F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == 1.5879225E37F);
            assert(pack.pitchspeed_GET() == -3.2303126E38F);
            assert(pack.yawspeed_GET() == 6.5746283E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {6.3881325E37F, 1.2566675E38F, -1.2999025E38F, 8.800348E37F}));
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {6.5053177E37F, -2.0997192E38F, -3.2263267E38F, 3.03272E38F, -3.859138E37F, -1.5526387E38F, 1.0270688E38F, -2.5684668E38F, -2.9905888E38F}));
            assert(pack.time_usec_GET() == 6179081229079240716L);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.pitchspeed_SET(-3.2303126E38F) ;
        p61.yawspeed_SET(6.5746283E37F) ;
        p61.time_usec_SET(6179081229079240716L) ;
        p61.q_SET(new float[] {6.3881325E37F, 1.2566675E38F, -1.2999025E38F, 8.800348E37F}, 0) ;
        p61.rollspeed_SET(1.5879225E37F) ;
        p61.covariance_SET(new float[] {6.5053177E37F, -2.0997192E38F, -3.2263267E38F, 3.03272E38F, -3.859138E37F, -1.5526387E38F, 1.0270688E38F, -2.5684668E38F, -2.9905888E38F}, 0) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.alt_error_GET() == 6.0635024E36F);
            assert(pack.nav_roll_GET() == 2.5635475E38F);
            assert(pack.wp_dist_GET() == (char)48539);
            assert(pack.nav_bearing_GET() == (short) -16274);
            assert(pack.nav_pitch_GET() == 9.676504E37F);
            assert(pack.aspd_error_GET() == 3.1729258E38F);
            assert(pack.xtrack_error_GET() == 2.9014151E38F);
            assert(pack.target_bearing_GET() == (short) -5938);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.xtrack_error_SET(2.9014151E38F) ;
        p62.aspd_error_SET(3.1729258E38F) ;
        p62.alt_error_SET(6.0635024E36F) ;
        p62.nav_pitch_SET(9.676504E37F) ;
        p62.wp_dist_SET((char)48539) ;
        p62.target_bearing_SET((short) -5938) ;
        p62.nav_bearing_SET((short) -16274) ;
        p62.nav_roll_SET(2.5635475E38F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {5.5559407E37F, -2.4726383E38F, 1.4914603E37F, -3.1575217E38F, -1.5081471E38F, -2.1206225E38F, -1.3237758E38F, -1.9477192E38F, -1.1764967E38F, -3.4671717E37F, 2.603572E38F, 1.8906895E37F, 8.24322E37F, -1.4133166E37F, 2.5600188E38F, 2.5107828E38F, 1.964545E38F, -1.5644162E38F, -2.3978155E38F, -2.343893E38F, -4.001548E37F, -5.3005933E37F, 2.4163696E38F, 8.4967273E37F, 2.5305184E38F, 1.0220453E38F, -1.4868058E38F, 2.8326468E38F, 1.4660273E38F, -3.1619999E38F, -3.1617574E37F, 6.0092495E37F, -3.0479888E38F, -6.794718E37F, -3.3872172E38F, -2.7675188E38F}));
            assert(pack.vy_GET() == -3.1890325E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
            assert(pack.lat_GET() == -913630680);
            assert(pack.relative_alt_GET() == 2011684418);
            assert(pack.alt_GET() == 2124354722);
            assert(pack.vz_GET() == -4.516618E37F);
            assert(pack.lon_GET() == -1515216216);
            assert(pack.vx_GET() == 1.7397288E38F);
            assert(pack.time_usec_GET() == 4405339564347263346L);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.vx_SET(1.7397288E38F) ;
        p63.time_usec_SET(4405339564347263346L) ;
        p63.lon_SET(-1515216216) ;
        p63.covariance_SET(new float[] {5.5559407E37F, -2.4726383E38F, 1.4914603E37F, -3.1575217E38F, -1.5081471E38F, -2.1206225E38F, -1.3237758E38F, -1.9477192E38F, -1.1764967E38F, -3.4671717E37F, 2.603572E38F, 1.8906895E37F, 8.24322E37F, -1.4133166E37F, 2.5600188E38F, 2.5107828E38F, 1.964545E38F, -1.5644162E38F, -2.3978155E38F, -2.343893E38F, -4.001548E37F, -5.3005933E37F, 2.4163696E38F, 8.4967273E37F, 2.5305184E38F, 1.0220453E38F, -1.4868058E38F, 2.8326468E38F, 1.4660273E38F, -3.1619999E38F, -3.1617574E37F, 6.0092495E37F, -3.0479888E38F, -6.794718E37F, -3.3872172E38F, -2.7675188E38F}, 0) ;
        p63.alt_SET(2124354722) ;
        p63.relative_alt_SET(2011684418) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE) ;
        p63.vz_SET(-4.516618E37F) ;
        p63.lat_SET(-913630680) ;
        p63.vy_SET(-3.1890325E38F) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 2.851545E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
            assert(pack.ax_GET() == 3.2108079E38F);
            assert(pack.time_usec_GET() == 6665736688911841854L);
            assert(pack.x_GET() == -1.8841588E38F);
            assert(pack.vz_GET() == -8.787882E37F);
            assert(pack.vx_GET() == 1.2258614E38F);
            assert(pack.z_GET() == -2.3951343E38F);
            assert(pack.ay_GET() == 2.525818E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.3535314E37F, 2.7115101E38F, -3.0673279E38F, -4.236042E37F, 2.0473922E38F, -2.7277498E38F, 9.399878E37F, 1.7668054E38F, -3.1807873E38F, -1.3446246E38F, 1.172499E38F, 2.1267425E38F, -2.2331148E38F, -3.9038707E37F, -1.397501E38F, 1.6402362E38F, 1.905611E38F, -2.466348E38F, -2.6837429E38F, 2.5154954E38F, -2.8438577E38F, -3.0315572E38F, -6.225533E37F, 2.62304E38F, -2.9109992E38F, 2.4582717E37F, 2.5112146E38F, -2.0671061E38F, 2.136885E38F, 1.9071645E38F, -7.7673886E37F, -1.8991008E38F, 2.6881304E38F, -1.9591648E38F, 3.0879721E38F, 6.007921E37F, 2.9845412E38F, 3.3062906E38F, -1.6669199E38F, 1.2440647E38F, 1.1689535E38F, 2.5981986E38F, -3.1572718E38F, 6.787485E37F, -1.9711162E38F}));
            assert(pack.az_GET() == -1.3308904E38F);
            assert(pack.vy_GET() == 4.1370416E37F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.vx_SET(1.2258614E38F) ;
        p64.time_usec_SET(6665736688911841854L) ;
        p64.vy_SET(4.1370416E37F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE) ;
        p64.vz_SET(-8.787882E37F) ;
        p64.covariance_SET(new float[] {1.3535314E37F, 2.7115101E38F, -3.0673279E38F, -4.236042E37F, 2.0473922E38F, -2.7277498E38F, 9.399878E37F, 1.7668054E38F, -3.1807873E38F, -1.3446246E38F, 1.172499E38F, 2.1267425E38F, -2.2331148E38F, -3.9038707E37F, -1.397501E38F, 1.6402362E38F, 1.905611E38F, -2.466348E38F, -2.6837429E38F, 2.5154954E38F, -2.8438577E38F, -3.0315572E38F, -6.225533E37F, 2.62304E38F, -2.9109992E38F, 2.4582717E37F, 2.5112146E38F, -2.0671061E38F, 2.136885E38F, 1.9071645E38F, -7.7673886E37F, -1.8991008E38F, 2.6881304E38F, -1.9591648E38F, 3.0879721E38F, 6.007921E37F, 2.9845412E38F, 3.3062906E38F, -1.6669199E38F, 1.2440647E38F, 1.1689535E38F, 2.5981986E38F, -3.1572718E38F, 6.787485E37F, -1.9711162E38F}, 0) ;
        p64.ax_SET(3.2108079E38F) ;
        p64.ay_SET(2.525818E38F) ;
        p64.z_SET(-2.3951343E38F) ;
        p64.az_SET(-1.3308904E38F) ;
        p64.x_SET(-1.8841588E38F) ;
        p64.y_SET(2.851545E38F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan7_raw_GET() == (char)35399);
            assert(pack.chan10_raw_GET() == (char)41450);
            assert(pack.chan5_raw_GET() == (char)28853);
            assert(pack.chan18_raw_GET() == (char)10992);
            assert(pack.chan9_raw_GET() == (char)35099);
            assert(pack.chancount_GET() == (char)162);
            assert(pack.chan4_raw_GET() == (char)21820);
            assert(pack.chan1_raw_GET() == (char)38284);
            assert(pack.chan3_raw_GET() == (char)16096);
            assert(pack.chan13_raw_GET() == (char)46342);
            assert(pack.time_boot_ms_GET() == 230178408L);
            assert(pack.chan2_raw_GET() == (char)41889);
            assert(pack.chan14_raw_GET() == (char)24355);
            assert(pack.chan15_raw_GET() == (char)52631);
            assert(pack.chan16_raw_GET() == (char)4706);
            assert(pack.rssi_GET() == (char)229);
            assert(pack.chan8_raw_GET() == (char)42521);
            assert(pack.chan17_raw_GET() == (char)51404);
            assert(pack.chan11_raw_GET() == (char)14528);
            assert(pack.chan12_raw_GET() == (char)54668);
            assert(pack.chan6_raw_GET() == (char)47458);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan16_raw_SET((char)4706) ;
        p65.chan17_raw_SET((char)51404) ;
        p65.rssi_SET((char)229) ;
        p65.chan1_raw_SET((char)38284) ;
        p65.chan14_raw_SET((char)24355) ;
        p65.chan18_raw_SET((char)10992) ;
        p65.chan8_raw_SET((char)42521) ;
        p65.chan3_raw_SET((char)16096) ;
        p65.chan2_raw_SET((char)41889) ;
        p65.chan10_raw_SET((char)41450) ;
        p65.chan6_raw_SET((char)47458) ;
        p65.chan13_raw_SET((char)46342) ;
        p65.time_boot_ms_SET(230178408L) ;
        p65.chancount_SET((char)162) ;
        p65.chan5_raw_SET((char)28853) ;
        p65.chan4_raw_SET((char)21820) ;
        p65.chan15_raw_SET((char)52631) ;
        p65.chan9_raw_SET((char)35099) ;
        p65.chan7_raw_SET((char)35399) ;
        p65.chan11_raw_SET((char)14528) ;
        p65.chan12_raw_SET((char)54668) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)209);
            assert(pack.req_stream_id_GET() == (char)100);
            assert(pack.start_stop_GET() == (char)4);
            assert(pack.req_message_rate_GET() == (char)11118);
            assert(pack.target_component_GET() == (char)67);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_stream_id_SET((char)100) ;
        p66.req_message_rate_SET((char)11118) ;
        p66.target_system_SET((char)209) ;
        p66.start_stop_SET((char)4) ;
        p66.target_component_SET((char)67) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.message_rate_GET() == (char)59537);
            assert(pack.on_off_GET() == (char)166);
            assert(pack.stream_id_GET() == (char)249);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)249) ;
        p67.message_rate_SET((char)59537) ;
        p67.on_off_SET((char)166) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.buttons_GET() == (char)53413);
            assert(pack.y_GET() == (short)30741);
            assert(pack.z_GET() == (short) -6302);
            assert(pack.r_GET() == (short)19227);
            assert(pack.target_GET() == (char)214);
            assert(pack.x_GET() == (short)1334);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.r_SET((short)19227) ;
        p69.y_SET((short)30741) ;
        p69.buttons_SET((char)53413) ;
        p69.z_SET((short) -6302) ;
        p69.x_SET((short)1334) ;
        p69.target_SET((char)214) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan6_raw_GET() == (char)21842);
            assert(pack.chan3_raw_GET() == (char)65039);
            assert(pack.chan1_raw_GET() == (char)60689);
            assert(pack.chan2_raw_GET() == (char)6023);
            assert(pack.chan7_raw_GET() == (char)55507);
            assert(pack.chan5_raw_GET() == (char)54575);
            assert(pack.target_component_GET() == (char)176);
            assert(pack.target_system_GET() == (char)24);
            assert(pack.chan8_raw_GET() == (char)2791);
            assert(pack.chan4_raw_GET() == (char)11898);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan7_raw_SET((char)55507) ;
        p70.chan2_raw_SET((char)6023) ;
        p70.chan4_raw_SET((char)11898) ;
        p70.target_component_SET((char)176) ;
        p70.chan6_raw_SET((char)21842) ;
        p70.chan8_raw_SET((char)2791) ;
        p70.chan1_raw_SET((char)60689) ;
        p70.target_system_SET((char)24) ;
        p70.chan5_raw_SET((char)54575) ;
        p70.chan3_raw_SET((char)65039) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)49396);
            assert(pack.param1_GET() == 1.7272967E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.autocontinue_GET() == (char)217);
            assert(pack.param3_GET() == -1.6210721E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH);
            assert(pack.z_GET() == -5.970446E37F);
            assert(pack.target_component_GET() == (char)17);
            assert(pack.y_GET() == 2002496817);
            assert(pack.current_GET() == (char)115);
            assert(pack.target_system_GET() == (char)210);
            assert(pack.param4_GET() == 1.0684583E38F);
            assert(pack.param2_GET() == -2.9606138E38F);
            assert(pack.x_GET() == 1539288752);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.z_SET(-5.970446E37F) ;
        p73.autocontinue_SET((char)217) ;
        p73.param2_SET(-2.9606138E38F) ;
        p73.param1_SET(1.7272967E38F) ;
        p73.param3_SET(-1.6210721E38F) ;
        p73.command_SET(MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH) ;
        p73.x_SET(1539288752) ;
        p73.target_system_SET((char)210) ;
        p73.current_SET((char)115) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p73.target_component_SET((char)17) ;
        p73.seq_SET((char)49396) ;
        p73.y_SET(2002496817) ;
        p73.param4_SET(1.0684583E38F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.groundspeed_GET() == -6.5065184E37F);
            assert(pack.airspeed_GET() == -2.270271E38F);
            assert(pack.throttle_GET() == (char)35064);
            assert(pack.alt_GET() == 1.8439402E38F);
            assert(pack.heading_GET() == (short) -30250);
            assert(pack.climb_GET() == -2.7918395E37F);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.airspeed_SET(-2.270271E38F) ;
        p74.groundspeed_SET(-6.5065184E37F) ;
        p74.heading_SET((short) -30250) ;
        p74.alt_SET(1.8439402E38F) ;
        p74.climb_SET(-2.7918395E37F) ;
        p74.throttle_SET((char)35064) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 59611926);
            assert(pack.param1_GET() == -1.902846E38F);
            assert(pack.z_GET() == 2.1195692E38F);
            assert(pack.x_GET() == 943201051);
            assert(pack.autocontinue_GET() == (char)97);
            assert(pack.param4_GET() == 1.1444069E38F);
            assert(pack.target_system_GET() == (char)7);
            assert(pack.param3_GET() == 2.8723667E38F);
            assert(pack.target_component_GET() == (char)15);
            assert(pack.param2_GET() == 1.5287021E37F);
            assert(pack.current_GET() == (char)157);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_RALLY_LAND);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p75.x_SET(943201051) ;
        p75.command_SET(MAV_CMD.MAV_CMD_DO_RALLY_LAND) ;
        p75.param4_SET(1.1444069E38F) ;
        p75.target_component_SET((char)15) ;
        p75.current_SET((char)157) ;
        p75.y_SET(59611926) ;
        p75.z_SET(2.1195692E38F) ;
        p75.target_system_SET((char)7) ;
        p75.param3_SET(2.8723667E38F) ;
        p75.autocontinue_SET((char)97) ;
        p75.param1_SET(-1.902846E38F) ;
        p75.param2_SET(1.5287021E37F) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param2_GET() == -3.003444E38F);
            assert(pack.param1_GET() == 2.027853E38F);
            assert(pack.param6_GET() == -1.2286859E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE);
            assert(pack.param5_GET() == 2.9539259E38F);
            assert(pack.param4_GET() == 1.539112E37F);
            assert(pack.target_component_GET() == (char)249);
            assert(pack.target_system_GET() == (char)75);
            assert(pack.confirmation_GET() == (char)46);
            assert(pack.param3_GET() == 7.584426E37F);
            assert(pack.param7_GET() == -8.448224E37F);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.target_system_SET((char)75) ;
        p76.command_SET(MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE) ;
        p76.param5_SET(2.9539259E38F) ;
        p76.param4_SET(1.539112E37F) ;
        p76.param3_SET(7.584426E37F) ;
        p76.param7_SET(-8.448224E37F) ;
        p76.param6_SET(-1.2286859E38F) ;
        p76.confirmation_SET((char)46) ;
        p76.param1_SET(2.027853E38F) ;
        p76.param2_SET(-3.003444E38F) ;
        p76.target_component_SET((char)249) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_TRY(ph) == (char)65);
            assert(pack.target_system_TRY(ph) == (char)235);
            assert(pack.result_param2_TRY(ph) == 1356830557);
            assert(pack.progress_TRY(ph) == (char)161);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_REPEAT_RELAY);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_FAILED);
        });
        GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.result_SET(MAV_RESULT.MAV_RESULT_FAILED) ;
        p77.progress_SET((char)161, PH) ;
        p77.target_component_SET((char)65, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_DO_REPEAT_RELAY) ;
        p77.result_param2_SET(1356830557, PH) ;
        p77.target_system_SET((char)235, PH) ;
        CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 4228268366L);
            assert(pack.thrust_GET() == 2.3500528E38F);
            assert(pack.yaw_GET() == 2.2114351E38F);
            assert(pack.pitch_GET() == 1.1278066E38F);
            assert(pack.mode_switch_GET() == (char)245);
            assert(pack.manual_override_switch_GET() == (char)80);
            assert(pack.roll_GET() == 2.8075147E38F);
        });
        GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.mode_switch_SET((char)245) ;
        p81.time_boot_ms_SET(4228268366L) ;
        p81.thrust_SET(2.3500528E38F) ;
        p81.pitch_SET(1.1278066E38F) ;
        p81.manual_override_switch_SET((char)80) ;
        p81.yaw_SET(2.2114351E38F) ;
        p81.roll_SET(2.8075147E38F) ;
        CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_pitch_rate_GET() == 3.585841E36F);
            assert(pack.body_roll_rate_GET() == 2.5781437E38F);
            assert(pack.type_mask_GET() == (char)234);
            assert(pack.time_boot_ms_GET() == 2655508716L);
            assert(pack.body_yaw_rate_GET() == -1.3509726E38F);
            assert(pack.thrust_GET() == 5.894005E35F);
            assert(pack.target_system_GET() == (char)134);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.8775089E38F, -1.6707801E38F, 3.1534332E38F, -1.9203193E38F}));
            assert(pack.target_component_GET() == (char)196);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.time_boot_ms_SET(2655508716L) ;
        p82.body_yaw_rate_SET(-1.3509726E38F) ;
        p82.q_SET(new float[] {-2.8775089E38F, -1.6707801E38F, 3.1534332E38F, -1.9203193E38F}, 0) ;
        p82.target_system_SET((char)134) ;
        p82.thrust_SET(5.894005E35F) ;
        p82.body_roll_rate_SET(2.5781437E38F) ;
        p82.target_component_SET((char)196) ;
        p82.type_mask_SET((char)234) ;
        p82.body_pitch_rate_SET(3.585841E36F) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_yaw_rate_GET() == -1.5432843E38F);
            assert(pack.body_roll_rate_GET() == -2.0049774E38F);
            assert(pack.type_mask_GET() == (char)112);
            assert(pack.body_pitch_rate_GET() == 7.3661493E37F);
            assert(pack.time_boot_ms_GET() == 1603007287L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {5.102021E37F, -7.30449E37F, 1.9988895E38F, 3.0887763E38F}));
            assert(pack.thrust_GET() == -8.896443E37F);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.type_mask_SET((char)112) ;
        p83.body_yaw_rate_SET(-1.5432843E38F) ;
        p83.body_pitch_rate_SET(7.3661493E37F) ;
        p83.time_boot_ms_SET(1603007287L) ;
        p83.thrust_SET(-8.896443E37F) ;
        p83.q_SET(new float[] {5.102021E37F, -7.30449E37F, 1.9988895E38F, 3.0887763E38F}, 0) ;
        p83.body_roll_rate_SET(-2.0049774E38F) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == 1.9996847E38F);
            assert(pack.target_system_GET() == (char)238);
            assert(pack.time_boot_ms_GET() == 476536601L);
            assert(pack.type_mask_GET() == (char)48037);
            assert(pack.target_component_GET() == (char)100);
            assert(pack.vy_GET() == 1.6794224E38F);
            assert(pack.afy_GET() == 1.0982825E38F);
            assert(pack.afx_GET() == -6.8200496E36F);
            assert(pack.vx_GET() == 1.9763133E38F);
            assert(pack.afz_GET() == -1.3256616E38F);
            assert(pack.x_GET() == -2.6059836E38F);
            assert(pack.yaw_rate_GET() == -9.686613E37F);
            assert(pack.yaw_GET() == -2.1224118E38F);
            assert(pack.z_GET() == 1.5511145E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.y_GET() == 3.0092824E37F);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.afy_SET(1.0982825E38F) ;
        p84.target_component_SET((char)100) ;
        p84.type_mask_SET((char)48037) ;
        p84.x_SET(-2.6059836E38F) ;
        p84.target_system_SET((char)238) ;
        p84.time_boot_ms_SET(476536601L) ;
        p84.y_SET(3.0092824E37F) ;
        p84.yaw_SET(-2.1224118E38F) ;
        p84.vx_SET(1.9763133E38F) ;
        p84.vy_SET(1.6794224E38F) ;
        p84.vz_SET(1.9996847E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p84.afx_SET(-6.8200496E36F) ;
        p84.yaw_rate_SET(-9.686613E37F) ;
        p84.afz_SET(-1.3256616E38F) ;
        p84.z_SET(1.5511145E38F) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == -5.823958E37F);
            assert(pack.yaw_GET() == -8.147649E37F);
            assert(pack.time_boot_ms_GET() == 2207287671L);
            assert(pack.target_system_GET() == (char)118);
            assert(pack.type_mask_GET() == (char)43647);
            assert(pack.afz_GET() == 5.411231E37F);
            assert(pack.vx_GET() == -2.9515715E38F);
            assert(pack.afy_GET() == 3.0172891E38F);
            assert(pack.lon_int_GET() == -643985718);
            assert(pack.yaw_rate_GET() == 2.5537696E38F);
            assert(pack.lat_int_GET() == -320611336);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.vy_GET() == 2.696793E38F);
            assert(pack.alt_GET() == 3.93498E37F);
            assert(pack.target_component_GET() == (char)224);
            assert(pack.afx_GET() == 7.936888E37F);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.yaw_SET(-8.147649E37F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p86.target_component_SET((char)224) ;
        p86.target_system_SET((char)118) ;
        p86.lon_int_SET(-643985718) ;
        p86.vz_SET(-5.823958E37F) ;
        p86.alt_SET(3.93498E37F) ;
        p86.afy_SET(3.0172891E38F) ;
        p86.lat_int_SET(-320611336) ;
        p86.afz_SET(5.411231E37F) ;
        p86.type_mask_SET((char)43647) ;
        p86.vy_SET(2.696793E38F) ;
        p86.vx_SET(-2.9515715E38F) ;
        p86.time_boot_ms_SET(2207287671L) ;
        p86.yaw_rate_SET(2.5537696E38F) ;
        p86.afx_SET(7.936888E37F) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == -2.0291437E38F);
            assert(pack.afy_GET() == 1.2906814E38F);
            assert(pack.lat_int_GET() == -1847505572);
            assert(pack.vz_GET() == 7.114821E36F);
            assert(pack.time_boot_ms_GET() == 945967015L);
            assert(pack.afz_GET() == 7.8160933E37F);
            assert(pack.yaw_rate_GET() == 6.501169E37F);
            assert(pack.afx_GET() == -2.4349732E38F);
            assert(pack.type_mask_GET() == (char)60604);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.lon_int_GET() == 2098650280);
            assert(pack.vx_GET() == -2.7130593E38F);
            assert(pack.alt_GET() == 5.8812685E37F);
            assert(pack.yaw_GET() == 2.4348868E38F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.lat_int_SET(-1847505572) ;
        p87.vx_SET(-2.7130593E38F) ;
        p87.vz_SET(7.114821E36F) ;
        p87.yaw_SET(2.4348868E38F) ;
        p87.afy_SET(1.2906814E38F) ;
        p87.yaw_rate_SET(6.501169E37F) ;
        p87.time_boot_ms_SET(945967015L) ;
        p87.lon_int_SET(2098650280) ;
        p87.type_mask_SET((char)60604) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p87.afz_SET(7.8160933E37F) ;
        p87.alt_SET(5.8812685E37F) ;
        p87.afx_SET(-2.4349732E38F) ;
        p87.vy_SET(-2.0291437E38F) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1089962292L);
            assert(pack.pitch_GET() == -1.5640601E38F);
            assert(pack.y_GET() == -3.3074286E38F);
            assert(pack.roll_GET() == 1.4767935E38F);
            assert(pack.z_GET() == -8.015752E36F);
            assert(pack.yaw_GET() == -2.8830283E38F);
            assert(pack.x_GET() == 3.0828092E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.yaw_SET(-2.8830283E38F) ;
        p89.pitch_SET(-1.5640601E38F) ;
        p89.y_SET(-3.3074286E38F) ;
        p89.time_boot_ms_SET(1089962292L) ;
        p89.x_SET(3.0828092E38F) ;
        p89.roll_SET(1.4767935E38F) ;
        p89.z_SET(-8.015752E36F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 3.1498808E37F);
            assert(pack.zacc_GET() == (short)27281);
            assert(pack.lon_GET() == -111436815);
            assert(pack.alt_GET() == 768437901);
            assert(pack.vx_GET() == (short) -29094);
            assert(pack.rollspeed_GET() == 4.875233E37F);
            assert(pack.roll_GET() == -2.195655E38F);
            assert(pack.yacc_GET() == (short)12226);
            assert(pack.yawspeed_GET() == 1.8758134E38F);
            assert(pack.xacc_GET() == (short)15348);
            assert(pack.pitch_GET() == 2.241845E38F);
            assert(pack.vy_GET() == (short) -18047);
            assert(pack.vz_GET() == (short) -11714);
            assert(pack.time_usec_GET() == 3799101991367737368L);
            assert(pack.lat_GET() == -1196351785);
            assert(pack.pitchspeed_GET() == 1.05388744E37F);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.time_usec_SET(3799101991367737368L) ;
        p90.vx_SET((short) -29094) ;
        p90.vz_SET((short) -11714) ;
        p90.yaw_SET(3.1498808E37F) ;
        p90.rollspeed_SET(4.875233E37F) ;
        p90.pitchspeed_SET(1.05388744E37F) ;
        p90.xacc_SET((short)15348) ;
        p90.lon_SET(-111436815) ;
        p90.roll_SET(-2.195655E38F) ;
        p90.vy_SET((short) -18047) ;
        p90.pitch_SET(2.241845E38F) ;
        p90.alt_SET(768437901) ;
        p90.yawspeed_SET(1.8758134E38F) ;
        p90.lat_SET(-1196351785) ;
        p90.yacc_SET((short)12226) ;
        p90.zacc_SET((short)27281) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux4_GET() == -2.2330178E38F);
            assert(pack.pitch_elevator_GET() == 1.816729E38F);
            assert(pack.throttle_GET() == -9.133102E37F);
            assert(pack.nav_mode_GET() == (char)205);
            assert(pack.roll_ailerons_GET() == -3.328013E38F);
            assert(pack.aux3_GET() == -1.7474555E38F);
            assert(pack.aux2_GET() == 1.0307961E38F);
            assert(pack.aux1_GET() == -3.3394044E38F);
            assert(pack.yaw_rudder_GET() == -3.0625284E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
            assert(pack.time_usec_GET() == 3012897743466071189L);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.roll_ailerons_SET(-3.328013E38F) ;
        p91.time_usec_SET(3012897743466071189L) ;
        p91.aux2_SET(1.0307961E38F) ;
        p91.aux1_SET(-3.3394044E38F) ;
        p91.yaw_rudder_SET(-3.0625284E38F) ;
        p91.aux4_SET(-2.2330178E38F) ;
        p91.pitch_elevator_SET(1.816729E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_STABILIZE_DISARMED) ;
        p91.throttle_SET(-9.133102E37F) ;
        p91.nav_mode_SET((char)205) ;
        p91.aux3_SET(-1.7474555E38F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan11_raw_GET() == (char)62027);
            assert(pack.chan10_raw_GET() == (char)15903);
            assert(pack.chan5_raw_GET() == (char)60801);
            assert(pack.chan12_raw_GET() == (char)57223);
            assert(pack.chan3_raw_GET() == (char)35854);
            assert(pack.chan2_raw_GET() == (char)54371);
            assert(pack.time_usec_GET() == 2412325108547330671L);
            assert(pack.chan8_raw_GET() == (char)25165);
            assert(pack.rssi_GET() == (char)36);
            assert(pack.chan6_raw_GET() == (char)34512);
            assert(pack.chan1_raw_GET() == (char)55836);
            assert(pack.chan9_raw_GET() == (char)25962);
            assert(pack.chan4_raw_GET() == (char)9210);
            assert(pack.chan7_raw_GET() == (char)63240);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan12_raw_SET((char)57223) ;
        p92.rssi_SET((char)36) ;
        p92.chan7_raw_SET((char)63240) ;
        p92.chan1_raw_SET((char)55836) ;
        p92.chan2_raw_SET((char)54371) ;
        p92.chan9_raw_SET((char)25962) ;
        p92.time_usec_SET(2412325108547330671L) ;
        p92.chan4_raw_SET((char)9210) ;
        p92.chan6_raw_SET((char)34512) ;
        p92.chan3_raw_SET((char)35854) ;
        p92.chan8_raw_SET((char)25165) ;
        p92.chan10_raw_SET((char)15903) ;
        p92.chan11_raw_SET((char)62027) ;
        p92.chan5_raw_SET((char)60801) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7292268923019744256L);
            assert(pack.flags_GET() == 4123315359866060521L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.2578768E38F, -5.617859E37F, 2.5980353E38F, -1.9326095E37F, 1.3385116E38F, 1.798464E38F, -3.2385112E38F, -3.0124112E38F, 2.2953433E38F, -1.8530506E38F, -2.6932938E37F, -1.9801944E38F, -3.0628477E38F, 2.7011383E38F, -1.9494507E38F, -1.014146E37F}));
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_MANUAL_ARMED);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(7292268923019744256L) ;
        p93.controls_SET(new float[] {-2.2578768E38F, -5.617859E37F, 2.5980353E38F, -1.9326095E37F, 1.3385116E38F, 1.798464E38F, -3.2385112E38F, -3.0124112E38F, 2.2953433E38F, -1.8530506E38F, -2.6932938E37F, -1.9801944E38F, -3.0628477E38F, 2.7011383E38F, -1.9494507E38F, -1.014146E37F}, 0) ;
        p93.flags_SET(4123315359866060521L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_MANUAL_ARMED) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_y_GET() == (short) -2157);
            assert(pack.quality_GET() == (char)109);
            assert(pack.flow_rate_x_TRY(ph) == -2.2048551E38F);
            assert(pack.flow_x_GET() == (short) -19544);
            assert(pack.sensor_id_GET() == (char)47);
            assert(pack.flow_rate_y_TRY(ph) == -9.150458E37F);
            assert(pack.time_usec_GET() == 2232770780096471661L);
            assert(pack.flow_comp_m_y_GET() == -2.6516322E38F);
            assert(pack.ground_distance_GET() == 1.9997807E38F);
            assert(pack.flow_comp_m_x_GET() == -1.5309767E38F);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.sensor_id_SET((char)47) ;
        p100.ground_distance_SET(1.9997807E38F) ;
        p100.flow_x_SET((short) -19544) ;
        p100.flow_rate_x_SET(-2.2048551E38F, PH) ;
        p100.flow_comp_m_y_SET(-2.6516322E38F) ;
        p100.flow_rate_y_SET(-9.150458E37F, PH) ;
        p100.flow_comp_m_x_SET(-1.5309767E38F) ;
        p100.time_usec_SET(2232770780096471661L) ;
        p100.quality_SET((char)109) ;
        p100.flow_y_SET((short) -2157) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -6.6496635E37F);
            assert(pack.roll_GET() == 3.1318442E38F);
            assert(pack.usec_GET() == 694695687070540186L);
            assert(pack.z_GET() == -2.9343012E38F);
            assert(pack.x_GET() == -2.9997179E38F);
            assert(pack.pitch_GET() == -6.252867E37F);
            assert(pack.yaw_GET() == 3.2914832E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.z_SET(-2.9343012E38F) ;
        p101.roll_SET(3.1318442E38F) ;
        p101.x_SET(-2.9997179E38F) ;
        p101.y_SET(-6.6496635E37F) ;
        p101.pitch_SET(-6.252867E37F) ;
        p101.yaw_SET(3.2914832E38F) ;
        p101.usec_SET(694695687070540186L) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -1.9060968E38F);
            assert(pack.roll_GET() == 3.1377808E38F);
            assert(pack.usec_GET() == 6203452239221091659L);
            assert(pack.pitch_GET() == 1.7355826E38F);
            assert(pack.x_GET() == -2.3459499E38F);
            assert(pack.z_GET() == 3.0573386E38F);
            assert(pack.yaw_GET() == 3.3085318E37F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.yaw_SET(3.3085318E37F) ;
        p102.usec_SET(6203452239221091659L) ;
        p102.z_SET(3.0573386E38F) ;
        p102.x_SET(-2.3459499E38F) ;
        p102.pitch_SET(1.7355826E38F) ;
        p102.roll_SET(3.1377808E38F) ;
        p102.y_SET(-1.9060968E38F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 2.7575913E38F);
            assert(pack.z_GET() == -2.1586921E38F);
            assert(pack.usec_GET() == 5501768867027626756L);
            assert(pack.x_GET() == 3.1795446E38F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.x_SET(3.1795446E38F) ;
        p103.usec_SET(5501768867027626756L) ;
        p103.z_SET(-2.1586921E38F) ;
        p103.y_SET(2.7575913E38F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -6.364091E37F);
            assert(pack.roll_GET() == 2.0385619E38F);
            assert(pack.x_GET() == 1.5278605E38F);
            assert(pack.yaw_GET() == -2.3775886E38F);
            assert(pack.usec_GET() == 4855686368767465274L);
            assert(pack.y_GET() == -8.2175146E36F);
            assert(pack.z_GET() == 2.9234443E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(4855686368767465274L) ;
        p104.z_SET(2.9234443E38F) ;
        p104.x_SET(1.5278605E38F) ;
        p104.pitch_SET(-6.364091E37F) ;
        p104.y_SET(-8.2175146E36F) ;
        p104.roll_SET(2.0385619E38F) ;
        p104.yaw_SET(-2.3775886E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == 3.5778569E37F);
            assert(pack.zacc_GET() == -2.1628296E38F);
            assert(pack.time_usec_GET() == 3074203409710940186L);
            assert(pack.zgyro_GET() == 2.3121324E38F);
            assert(pack.ymag_GET() == 3.1409887E38F);
            assert(pack.xacc_GET() == 7.326182E37F);
            assert(pack.xgyro_GET() == -1.7156775E38F);
            assert(pack.diff_pressure_GET() == 3.239374E37F);
            assert(pack.abs_pressure_GET() == 5.750811E37F);
            assert(pack.fields_updated_GET() == (char)44176);
            assert(pack.xmag_GET() == 2.2694797E38F);
            assert(pack.zmag_GET() == 9.120125E37F);
            assert(pack.yacc_GET() == 1.936926E38F);
            assert(pack.pressure_alt_GET() == -2.5041797E38F);
            assert(pack.temperature_GET() == -2.3435822E37F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.ymag_SET(3.1409887E38F) ;
        p105.yacc_SET(1.936926E38F) ;
        p105.ygyro_SET(3.5778569E37F) ;
        p105.xgyro_SET(-1.7156775E38F) ;
        p105.zmag_SET(9.120125E37F) ;
        p105.zacc_SET(-2.1628296E38F) ;
        p105.fields_updated_SET((char)44176) ;
        p105.time_usec_SET(3074203409710940186L) ;
        p105.abs_pressure_SET(5.750811E37F) ;
        p105.diff_pressure_SET(3.239374E37F) ;
        p105.temperature_SET(-2.3435822E37F) ;
        p105.zgyro_SET(2.3121324E38F) ;
        p105.xmag_SET(2.2694797E38F) ;
        p105.pressure_alt_SET(-2.5041797E38F) ;
        p105.xacc_SET(7.326182E37F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_ygyro_GET() == 2.874972E38F);
            assert(pack.sensor_id_GET() == (char)231);
            assert(pack.quality_GET() == (char)158);
            assert(pack.integrated_y_GET() == -1.418565E38F);
            assert(pack.time_usec_GET() == 993497362540817977L);
            assert(pack.time_delta_distance_us_GET() == 4120040212L);
            assert(pack.integrated_xgyro_GET() == 8.64791E37F);
            assert(pack.temperature_GET() == (short) -7357);
            assert(pack.integrated_zgyro_GET() == -1.0470934E38F);
            assert(pack.integration_time_us_GET() == 1485954806L);
            assert(pack.distance_GET() == -2.6893366E38F);
            assert(pack.integrated_x_GET() == -1.2142448E38F);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_y_SET(-1.418565E38F) ;
        p106.integration_time_us_SET(1485954806L) ;
        p106.time_usec_SET(993497362540817977L) ;
        p106.integrated_ygyro_SET(2.874972E38F) ;
        p106.time_delta_distance_us_SET(4120040212L) ;
        p106.integrated_zgyro_SET(-1.0470934E38F) ;
        p106.integrated_xgyro_SET(8.64791E37F) ;
        p106.quality_SET((char)158) ;
        p106.sensor_id_SET((char)231) ;
        p106.distance_SET(-2.6893366E38F) ;
        p106.temperature_SET((short) -7357) ;
        p106.integrated_x_SET(-1.2142448E38F) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.pressure_alt_GET() == 2.3349755E38F);
            assert(pack.zmag_GET() == 1.3604535E38F);
            assert(pack.zgyro_GET() == -1.6475256E38F);
            assert(pack.abs_pressure_GET() == -3.2908631E38F);
            assert(pack.xacc_GET() == 1.151251E38F);
            assert(pack.time_usec_GET() == 9009407941259635397L);
            assert(pack.diff_pressure_GET() == -2.1925782E38F);
            assert(pack.xgyro_GET() == 1.8559808E38F);
            assert(pack.temperature_GET() == -2.8251397E38F);
            assert(pack.fields_updated_GET() == 1270986139L);
            assert(pack.zacc_GET() == -2.115575E38F);
            assert(pack.ygyro_GET() == 2.0255755E37F);
            assert(pack.yacc_GET() == -2.4771837E37F);
            assert(pack.xmag_GET() == -1.1650461E38F);
            assert(pack.ymag_GET() == 1.5038929E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(9009407941259635397L) ;
        p107.zacc_SET(-2.115575E38F) ;
        p107.temperature_SET(-2.8251397E38F) ;
        p107.fields_updated_SET(1270986139L) ;
        p107.pressure_alt_SET(2.3349755E38F) ;
        p107.xacc_SET(1.151251E38F) ;
        p107.xgyro_SET(1.8559808E38F) ;
        p107.xmag_SET(-1.1650461E38F) ;
        p107.zmag_SET(1.3604535E38F) ;
        p107.ymag_SET(1.5038929E38F) ;
        p107.yacc_SET(-2.4771837E37F) ;
        p107.ygyro_SET(2.0255755E37F) ;
        p107.abs_pressure_SET(-3.2908631E38F) ;
        p107.diff_pressure_SET(-2.1925782E38F) ;
        p107.zgyro_SET(-1.6475256E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 1.989032E38F);
            assert(pack.q4_GET() == -3.270669E38F);
            assert(pack.alt_GET() == 2.2586268E38F);
            assert(pack.q1_GET() == 3.4161168E37F);
            assert(pack.std_dev_horz_GET() == 9.25007E37F);
            assert(pack.pitch_GET() == 3.2913996E38F);
            assert(pack.ygyro_GET() == 7.8082085E37F);
            assert(pack.lon_GET() == -4.7491805E37F);
            assert(pack.zacc_GET() == 3.3110433E38F);
            assert(pack.xgyro_GET() == -4.6290285E37F);
            assert(pack.ve_GET() == -5.985189E37F);
            assert(pack.lat_GET() == -2.7326012E38F);
            assert(pack.vn_GET() == -1.7542925E38F);
            assert(pack.zgyro_GET() == 1.851117E38F);
            assert(pack.yaw_GET() == -4.9956264E37F);
            assert(pack.xacc_GET() == -1.0952864E38F);
            assert(pack.q2_GET() == 1.3123783E38F);
            assert(pack.vd_GET() == -2.4761658E37F);
            assert(pack.std_dev_vert_GET() == 2.2184259E38F);
            assert(pack.q3_GET() == 3.1338189E38F);
            assert(pack.yacc_GET() == 3.3053843E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(3.4161168E37F) ;
        p108.lon_SET(-4.7491805E37F) ;
        p108.yaw_SET(-4.9956264E37F) ;
        p108.q3_SET(3.1338189E38F) ;
        p108.vn_SET(-1.7542925E38F) ;
        p108.ygyro_SET(7.8082085E37F) ;
        p108.alt_SET(2.2586268E38F) ;
        p108.std_dev_horz_SET(9.25007E37F) ;
        p108.xgyro_SET(-4.6290285E37F) ;
        p108.roll_SET(1.989032E38F) ;
        p108.q2_SET(1.3123783E38F) ;
        p108.yacc_SET(3.3053843E38F) ;
        p108.vd_SET(-2.4761658E37F) ;
        p108.q4_SET(-3.270669E38F) ;
        p108.std_dev_vert_SET(2.2184259E38F) ;
        p108.lat_SET(-2.7326012E38F) ;
        p108.ve_SET(-5.985189E37F) ;
        p108.xacc_SET(-1.0952864E38F) ;
        p108.zgyro_SET(1.851117E38F) ;
        p108.zacc_SET(3.3110433E38F) ;
        p108.pitch_SET(3.2913996E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.txbuf_GET() == (char)222);
            assert(pack.remnoise_GET() == (char)47);
            assert(pack.fixed__GET() == (char)48424);
            assert(pack.rssi_GET() == (char)196);
            assert(pack.rxerrors_GET() == (char)50513);
            assert(pack.remrssi_GET() == (char)34);
            assert(pack.noise_GET() == (char)109);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.txbuf_SET((char)222) ;
        p109.remrssi_SET((char)34) ;
        p109.remnoise_SET((char)47) ;
        p109.noise_SET((char)109) ;
        p109.fixed__SET((char)48424) ;
        p109.rssi_SET((char)196) ;
        p109.rxerrors_SET((char)50513) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)82);
            assert(pack.target_component_GET() == (char)202);
            assert(pack.target_network_GET() == (char)39);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)109, (char)79, (char)73, (char)191, (char)28, (char)152, (char)180, (char)244, (char)79, (char)58, (char)45, (char)14, (char)8, (char)64, (char)73, (char)39, (char)89, (char)230, (char)227, (char)198, (char)208, (char)198, (char)23, (char)68, (char)221, (char)57, (char)91, (char)237, (char)7, (char)105, (char)79, (char)179, (char)101, (char)175, (char)21, (char)153, (char)138, (char)242, (char)119, (char)175, (char)40, (char)27, (char)64, (char)42, (char)225, (char)10, (char)103, (char)216, (char)61, (char)90, (char)28, (char)24, (char)117, (char)115, (char)235, (char)89, (char)238, (char)58, (char)10, (char)216, (char)220, (char)230, (char)106, (char)223, (char)40, (char)145, (char)59, (char)230, (char)223, (char)133, (char)58, (char)51, (char)157, (char)18, (char)62, (char)180, (char)202, (char)187, (char)155, (char)232, (char)126, (char)33, (char)33, (char)195, (char)254, (char)193, (char)98, (char)214, (char)230, (char)57, (char)150, (char)33, (char)60, (char)149, (char)190, (char)56, (char)228, (char)126, (char)77, (char)209, (char)46, (char)109, (char)240, (char)251, (char)9, (char)70, (char)57, (char)63, (char)106, (char)123, (char)10, (char)229, (char)186, (char)76, (char)245, (char)77, (char)151, (char)73, (char)18, (char)168, (char)89, (char)11, (char)109, (char)23, (char)223, (char)69, (char)120, (char)222, (char)56, (char)83, (char)175, (char)217, (char)230, (char)172, (char)88, (char)85, (char)54, (char)0, (char)141, (char)94, (char)176, (char)167, (char)171, (char)56, (char)32, (char)74, (char)83, (char)138, (char)132, (char)26, (char)206, (char)18, (char)82, (char)92, (char)18, (char)124, (char)194, (char)238, (char)205, (char)235, (char)164, (char)17, (char)244, (char)181, (char)192, (char)67, (char)242, (char)73, (char)104, (char)209, (char)33, (char)161, (char)62, (char)154, (char)75, (char)2, (char)170, (char)64, (char)102, (char)214, (char)176, (char)132, (char)218, (char)0, (char)227, (char)87, (char)26, (char)82, (char)246, (char)136, (char)206, (char)3, (char)104, (char)64, (char)93, (char)52, (char)154, (char)137, (char)189, (char)115, (char)226, (char)10, (char)48, (char)205, (char)231, (char)241, (char)246, (char)235, (char)154, (char)18, (char)83, (char)43, (char)160, (char)33, (char)96, (char)82, (char)23, (char)57, (char)254, (char)84, (char)69, (char)29, (char)32, (char)89, (char)91, (char)227, (char)162, (char)94, (char)227, (char)51, (char)202, (char)138, (char)201, (char)70, (char)61, (char)183, (char)251, (char)193, (char)52, (char)196, (char)13, (char)1, (char)57, (char)145, (char)108, (char)111, (char)124, (char)245, (char)208, (char)64, (char)50}));
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)39) ;
        p110.target_component_SET((char)202) ;
        p110.target_system_SET((char)82) ;
        p110.payload_SET(new char[] {(char)109, (char)79, (char)73, (char)191, (char)28, (char)152, (char)180, (char)244, (char)79, (char)58, (char)45, (char)14, (char)8, (char)64, (char)73, (char)39, (char)89, (char)230, (char)227, (char)198, (char)208, (char)198, (char)23, (char)68, (char)221, (char)57, (char)91, (char)237, (char)7, (char)105, (char)79, (char)179, (char)101, (char)175, (char)21, (char)153, (char)138, (char)242, (char)119, (char)175, (char)40, (char)27, (char)64, (char)42, (char)225, (char)10, (char)103, (char)216, (char)61, (char)90, (char)28, (char)24, (char)117, (char)115, (char)235, (char)89, (char)238, (char)58, (char)10, (char)216, (char)220, (char)230, (char)106, (char)223, (char)40, (char)145, (char)59, (char)230, (char)223, (char)133, (char)58, (char)51, (char)157, (char)18, (char)62, (char)180, (char)202, (char)187, (char)155, (char)232, (char)126, (char)33, (char)33, (char)195, (char)254, (char)193, (char)98, (char)214, (char)230, (char)57, (char)150, (char)33, (char)60, (char)149, (char)190, (char)56, (char)228, (char)126, (char)77, (char)209, (char)46, (char)109, (char)240, (char)251, (char)9, (char)70, (char)57, (char)63, (char)106, (char)123, (char)10, (char)229, (char)186, (char)76, (char)245, (char)77, (char)151, (char)73, (char)18, (char)168, (char)89, (char)11, (char)109, (char)23, (char)223, (char)69, (char)120, (char)222, (char)56, (char)83, (char)175, (char)217, (char)230, (char)172, (char)88, (char)85, (char)54, (char)0, (char)141, (char)94, (char)176, (char)167, (char)171, (char)56, (char)32, (char)74, (char)83, (char)138, (char)132, (char)26, (char)206, (char)18, (char)82, (char)92, (char)18, (char)124, (char)194, (char)238, (char)205, (char)235, (char)164, (char)17, (char)244, (char)181, (char)192, (char)67, (char)242, (char)73, (char)104, (char)209, (char)33, (char)161, (char)62, (char)154, (char)75, (char)2, (char)170, (char)64, (char)102, (char)214, (char)176, (char)132, (char)218, (char)0, (char)227, (char)87, (char)26, (char)82, (char)246, (char)136, (char)206, (char)3, (char)104, (char)64, (char)93, (char)52, (char)154, (char)137, (char)189, (char)115, (char)226, (char)10, (char)48, (char)205, (char)231, (char)241, (char)246, (char)235, (char)154, (char)18, (char)83, (char)43, (char)160, (char)33, (char)96, (char)82, (char)23, (char)57, (char)254, (char)84, (char)69, (char)29, (char)32, (char)89, (char)91, (char)227, (char)162, (char)94, (char)227, (char)51, (char)202, (char)138, (char)201, (char)70, (char)61, (char)183, (char)251, (char)193, (char)52, (char)196, (char)13, (char)1, (char)57, (char)145, (char)108, (char)111, (char)124, (char)245, (char)208, (char)64, (char)50}, 0) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -2940724290694710826L);
            assert(pack.ts1_GET() == -5010082562646753422L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(-5010082562646753422L) ;
        p111.tc1_SET(-2940724290694710826L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2936663492963396748L);
            assert(pack.seq_GET() == 3335652334L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(3335652334L) ;
        p112.time_usec_SET(2936663492963396748L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.eph_GET() == (char)5049);
            assert(pack.time_usec_GET() == 5533653041270043990L);
            assert(pack.lon_GET() == 1600931992);
            assert(pack.epv_GET() == (char)28297);
            assert(pack.satellites_visible_GET() == (char)93);
            assert(pack.cog_GET() == (char)64765);
            assert(pack.alt_GET() == -333864923);
            assert(pack.vd_GET() == (short) -29828);
            assert(pack.vel_GET() == (char)27509);
            assert(pack.lat_GET() == 2065632422);
            assert(pack.fix_type_GET() == (char)224);
            assert(pack.vn_GET() == (short)7062);
            assert(pack.ve_GET() == (short) -18565);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.lat_SET(2065632422) ;
        p113.epv_SET((char)28297) ;
        p113.fix_type_SET((char)224) ;
        p113.time_usec_SET(5533653041270043990L) ;
        p113.lon_SET(1600931992) ;
        p113.satellites_visible_SET((char)93) ;
        p113.vel_SET((char)27509) ;
        p113.vd_SET((short) -29828) ;
        p113.alt_SET(-333864923) ;
        p113.eph_SET((char)5049) ;
        p113.ve_SET((short) -18565) ;
        p113.vn_SET((short)7062) ;
        p113.cog_SET((char)64765) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.time_delta_distance_us_GET() == 3708652752L);
            assert(pack.quality_GET() == (char)140);
            assert(pack.sensor_id_GET() == (char)114);
            assert(pack.time_usec_GET() == 716702264813138298L);
            assert(pack.integrated_ygyro_GET() == -7.1673107E37F);
            assert(pack.integrated_xgyro_GET() == 2.1291464E38F);
            assert(pack.integrated_x_GET() == 5.220199E37F);
            assert(pack.distance_GET() == 2.3686289E38F);
            assert(pack.integrated_zgyro_GET() == 7.1648043E37F);
            assert(pack.temperature_GET() == (short) -30550);
            assert(pack.integration_time_us_GET() == 4224559086L);
            assert(pack.integrated_y_GET() == -2.5249002E38F);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_delta_distance_us_SET(3708652752L) ;
        p114.integrated_xgyro_SET(2.1291464E38F) ;
        p114.sensor_id_SET((char)114) ;
        p114.time_usec_SET(716702264813138298L) ;
        p114.temperature_SET((short) -30550) ;
        p114.quality_SET((char)140) ;
        p114.integration_time_us_SET(4224559086L) ;
        p114.integrated_y_SET(-2.5249002E38F) ;
        p114.integrated_zgyro_SET(7.1648043E37F) ;
        p114.integrated_ygyro_SET(-7.1673107E37F) ;
        p114.integrated_x_SET(5.220199E37F) ;
        p114.distance_SET(2.3686289E38F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == 1.6705183E38F);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {2.4256233E37F, -3.133866E38F, -5.725568E37F, -2.7584582E38F}));
            assert(pack.yawspeed_GET() == -2.4256774E38F);
            assert(pack.true_airspeed_GET() == (char)10954);
            assert(pack.zacc_GET() == (short) -10153);
            assert(pack.lon_GET() == 906450900);
            assert(pack.vx_GET() == (short)21042);
            assert(pack.vy_GET() == (short)30824);
            assert(pack.lat_GET() == -879254464);
            assert(pack.yacc_GET() == (short) -56);
            assert(pack.pitchspeed_GET() == -2.8050007E38F);
            assert(pack.xacc_GET() == (short) -3761);
            assert(pack.time_usec_GET() == 2530926850961780644L);
            assert(pack.vz_GET() == (short)15730);
            assert(pack.ind_airspeed_GET() == (char)16147);
            assert(pack.alt_GET() == 392068437);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.attitude_quaternion_SET(new float[] {2.4256233E37F, -3.133866E38F, -5.725568E37F, -2.7584582E38F}, 0) ;
        p115.xacc_SET((short) -3761) ;
        p115.vz_SET((short)15730) ;
        p115.yawspeed_SET(-2.4256774E38F) ;
        p115.zacc_SET((short) -10153) ;
        p115.time_usec_SET(2530926850961780644L) ;
        p115.lon_SET(906450900) ;
        p115.lat_SET(-879254464) ;
        p115.vy_SET((short)30824) ;
        p115.true_airspeed_SET((char)10954) ;
        p115.alt_SET(392068437) ;
        p115.rollspeed_SET(1.6705183E38F) ;
        p115.pitchspeed_SET(-2.8050007E38F) ;
        p115.ind_airspeed_SET((char)16147) ;
        p115.yacc_SET((short) -56) ;
        p115.vx_SET((short)21042) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short) -23933);
            assert(pack.zacc_GET() == (short)18345);
            assert(pack.zmag_GET() == (short)31285);
            assert(pack.xmag_GET() == (short)31384);
            assert(pack.ygyro_GET() == (short) -25098);
            assert(pack.time_boot_ms_GET() == 1100143833L);
            assert(pack.xgyro_GET() == (short) -27308);
            assert(pack.xacc_GET() == (short)20533);
            assert(pack.ymag_GET() == (short)26020);
            assert(pack.yacc_GET() == (short) -7088);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.ygyro_SET((short) -25098) ;
        p116.xmag_SET((short)31384) ;
        p116.zmag_SET((short)31285) ;
        p116.xgyro_SET((short) -27308) ;
        p116.xacc_SET((short)20533) ;
        p116.yacc_SET((short) -7088) ;
        p116.ymag_SET((short)26020) ;
        p116.zacc_SET((short)18345) ;
        p116.zgyro_SET((short) -23933) ;
        p116.time_boot_ms_SET(1100143833L) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)251);
            assert(pack.end_GET() == (char)17231);
            assert(pack.start_GET() == (char)36132);
            assert(pack.target_system_GET() == (char)147);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.start_SET((char)36132) ;
        p117.target_system_SET((char)147) ;
        p117.end_SET((char)17231) ;
        p117.target_component_SET((char)251) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.last_log_num_GET() == (char)15666);
            assert(pack.num_logs_GET() == (char)57928);
            assert(pack.size_GET() == 3088142819L);
            assert(pack.id_GET() == (char)35813);
            assert(pack.time_utc_GET() == 268481125L);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)35813) ;
        p118.last_log_num_SET((char)15666) ;
        p118.size_SET(3088142819L) ;
        p118.time_utc_SET(268481125L) ;
        p118.num_logs_SET((char)57928) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == 902029374L);
            assert(pack.id_GET() == (char)63254);
            assert(pack.ofs_GET() == 4107367277L);
            assert(pack.target_system_GET() == (char)115);
            assert(pack.target_component_GET() == (char)63);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.id_SET((char)63254) ;
        p119.target_system_SET((char)115) ;
        p119.ofs_SET(4107367277L) ;
        p119.count_SET(902029374L) ;
        p119.target_component_SET((char)63) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)35742);
            assert(pack.ofs_GET() == 1897490744L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)51, (char)16, (char)210, (char)200, (char)37, (char)131, (char)119, (char)171, (char)95, (char)25, (char)247, (char)164, (char)117, (char)197, (char)162, (char)55, (char)217, (char)86, (char)144, (char)134, (char)144, (char)202, (char)209, (char)235, (char)12, (char)6, (char)94, (char)1, (char)250, (char)242, (char)65, (char)167, (char)47, (char)175, (char)71, (char)167, (char)22, (char)96, (char)76, (char)121, (char)163, (char)254, (char)81, (char)195, (char)225, (char)93, (char)9, (char)167, (char)114, (char)29, (char)252, (char)201, (char)101, (char)24, (char)96, (char)97, (char)122, (char)90, (char)220, (char)86, (char)155, (char)132, (char)44, (char)161, (char)140, (char)168, (char)55, (char)206, (char)212, (char)41, (char)178, (char)157, (char)214, (char)92, (char)153, (char)10, (char)68, (char)45, (char)23, (char)246, (char)171, (char)73, (char)111, (char)123, (char)22, (char)15, (char)140, (char)246, (char)117, (char)221}));
            assert(pack.count_GET() == (char)10);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)35742) ;
        p120.data__SET(new char[] {(char)51, (char)16, (char)210, (char)200, (char)37, (char)131, (char)119, (char)171, (char)95, (char)25, (char)247, (char)164, (char)117, (char)197, (char)162, (char)55, (char)217, (char)86, (char)144, (char)134, (char)144, (char)202, (char)209, (char)235, (char)12, (char)6, (char)94, (char)1, (char)250, (char)242, (char)65, (char)167, (char)47, (char)175, (char)71, (char)167, (char)22, (char)96, (char)76, (char)121, (char)163, (char)254, (char)81, (char)195, (char)225, (char)93, (char)9, (char)167, (char)114, (char)29, (char)252, (char)201, (char)101, (char)24, (char)96, (char)97, (char)122, (char)90, (char)220, (char)86, (char)155, (char)132, (char)44, (char)161, (char)140, (char)168, (char)55, (char)206, (char)212, (char)41, (char)178, (char)157, (char)214, (char)92, (char)153, (char)10, (char)68, (char)45, (char)23, (char)246, (char)171, (char)73, (char)111, (char)123, (char)22, (char)15, (char)140, (char)246, (char)117, (char)221}, 0) ;
        p120.count_SET((char)10) ;
        p120.ofs_SET(1897490744L) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)210);
            assert(pack.target_component_GET() == (char)123);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)123) ;
        p121.target_system_SET((char)210) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)54);
            assert(pack.target_system_GET() == (char)53);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)53) ;
        p122.target_component_SET((char)54) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)220);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)98, (char)249, (char)179, (char)72, (char)175, (char)195, (char)12, (char)96, (char)6, (char)110, (char)76, (char)189, (char)125, (char)120, (char)180, (char)254, (char)101, (char)194, (char)16, (char)194, (char)41, (char)138, (char)24, (char)127, (char)128, (char)2, (char)131, (char)133, (char)204, (char)191, (char)158, (char)94, (char)155, (char)206, (char)166, (char)86, (char)61, (char)208, (char)250, (char)125, (char)12, (char)188, (char)99, (char)158, (char)222, (char)198, (char)192, (char)192, (char)231, (char)232, (char)94, (char)43, (char)93, (char)184, (char)33, (char)114, (char)88, (char)226, (char)81, (char)178, (char)50, (char)7, (char)84, (char)15, (char)115, (char)101, (char)34, (char)254, (char)246, (char)14, (char)90, (char)90, (char)70, (char)121, (char)122, (char)145, (char)95, (char)196, (char)54, (char)95, (char)197, (char)33, (char)60, (char)216, (char)209, (char)108, (char)11, (char)10, (char)81, (char)73, (char)86, (char)51, (char)21, (char)39, (char)185, (char)202, (char)16, (char)222, (char)228, (char)227, (char)99, (char)181, (char)250, (char)7, (char)83, (char)250, (char)120, (char)235, (char)144, (char)245}));
            assert(pack.target_component_GET() == (char)113);
            assert(pack.len_GET() == (char)211);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_component_SET((char)113) ;
        p123.data__SET(new char[] {(char)98, (char)249, (char)179, (char)72, (char)175, (char)195, (char)12, (char)96, (char)6, (char)110, (char)76, (char)189, (char)125, (char)120, (char)180, (char)254, (char)101, (char)194, (char)16, (char)194, (char)41, (char)138, (char)24, (char)127, (char)128, (char)2, (char)131, (char)133, (char)204, (char)191, (char)158, (char)94, (char)155, (char)206, (char)166, (char)86, (char)61, (char)208, (char)250, (char)125, (char)12, (char)188, (char)99, (char)158, (char)222, (char)198, (char)192, (char)192, (char)231, (char)232, (char)94, (char)43, (char)93, (char)184, (char)33, (char)114, (char)88, (char)226, (char)81, (char)178, (char)50, (char)7, (char)84, (char)15, (char)115, (char)101, (char)34, (char)254, (char)246, (char)14, (char)90, (char)90, (char)70, (char)121, (char)122, (char)145, (char)95, (char)196, (char)54, (char)95, (char)197, (char)33, (char)60, (char)216, (char)209, (char)108, (char)11, (char)10, (char)81, (char)73, (char)86, (char)51, (char)21, (char)39, (char)185, (char)202, (char)16, (char)222, (char)228, (char)227, (char)99, (char)181, (char)250, (char)7, (char)83, (char)250, (char)120, (char)235, (char)144, (char)245}, 0) ;
        p123.len_SET((char)211) ;
        p123.target_system_SET((char)220) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.vel_GET() == (char)25073);
            assert(pack.dgps_age_GET() == 3277716801L);
            assert(pack.alt_GET() == 1711944253);
            assert(pack.lon_GET() == -2116170014);
            assert(pack.eph_GET() == (char)24915);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
            assert(pack.time_usec_GET() == 5039836915893299758L);
            assert(pack.lat_GET() == -1680427431);
            assert(pack.dgps_numch_GET() == (char)203);
            assert(pack.epv_GET() == (char)44208);
            assert(pack.cog_GET() == (char)9975);
            assert(pack.satellites_visible_GET() == (char)194);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.dgps_numch_SET((char)203) ;
        p124.vel_SET((char)25073) ;
        p124.epv_SET((char)44208) ;
        p124.lat_SET(-1680427431) ;
        p124.cog_SET((char)9975) ;
        p124.satellites_visible_SET((char)194) ;
        p124.lon_SET(-2116170014) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS) ;
        p124.dgps_age_SET(3277716801L) ;
        p124.alt_SET(1711944253) ;
        p124.eph_SET((char)24915) ;
        p124.time_usec_SET(5039836915893299758L) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vservo_GET() == (char)35373);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT));
            assert(pack.Vcc_GET() == (char)16884);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)16884) ;
        p125.Vservo_SET((char)35373) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT)) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.baudrate_GET() == 4053131186L);
            assert(pack.count_GET() == (char)141);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)80, (char)52, (char)112, (char)114, (char)92, (char)48, (char)224, (char)144, (char)180, (char)254, (char)101, (char)53, (char)41, (char)73, (char)224, (char)139, (char)166, (char)33, (char)36, (char)249, (char)23, (char)37, (char)207, (char)163, (char)34, (char)253, (char)82, (char)27, (char)100, (char)118, (char)254, (char)160, (char)204, (char)106, (char)131, (char)143, (char)54, (char)7, (char)119, (char)236, (char)130, (char)153, (char)126, (char)111, (char)197, (char)85, (char)7, (char)188, (char)95, (char)236, (char)101, (char)2, (char)229, (char)187, (char)197, (char)103, (char)63, (char)143, (char)34, (char)96, (char)109, (char)200, (char)112, (char)245, (char)16, (char)26, (char)172, (char)35, (char)105, (char)220}));
            assert(pack.timeout_GET() == (char)18099);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY));
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.count_SET((char)141) ;
        p126.baudrate_SET(4053131186L) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL) ;
        p126.timeout_SET((char)18099) ;
        p126.data__SET(new char[] {(char)80, (char)52, (char)112, (char)114, (char)92, (char)48, (char)224, (char)144, (char)180, (char)254, (char)101, (char)53, (char)41, (char)73, (char)224, (char)139, (char)166, (char)33, (char)36, (char)249, (char)23, (char)37, (char)207, (char)163, (char)34, (char)253, (char)82, (char)27, (char)100, (char)118, (char)254, (char)160, (char)204, (char)106, (char)131, (char)143, (char)54, (char)7, (char)119, (char)236, (char)130, (char)153, (char)126, (char)111, (char)197, (char)85, (char)7, (char)188, (char)95, (char)236, (char)101, (char)2, (char)229, (char)187, (char)197, (char)103, (char)63, (char)143, (char)34, (char)96, (char)109, (char)200, (char)112, (char)245, (char)16, (char)26, (char)172, (char)35, (char)105, (char)220}, 0) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY)) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.nsats_GET() == (char)66);
            assert(pack.baseline_coords_type_GET() == (char)13);
            assert(pack.rtk_rate_GET() == (char)168);
            assert(pack.wn_GET() == (char)84);
            assert(pack.rtk_health_GET() == (char)139);
            assert(pack.baseline_c_mm_GET() == -1005690239);
            assert(pack.time_last_baseline_ms_GET() == 393707655L);
            assert(pack.baseline_a_mm_GET() == -1782989467);
            assert(pack.rtk_receiver_id_GET() == (char)87);
            assert(pack.baseline_b_mm_GET() == 529833260);
            assert(pack.accuracy_GET() == 486792809L);
            assert(pack.iar_num_hypotheses_GET() == 1352730798);
            assert(pack.tow_GET() == 2438802559L);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.baseline_c_mm_SET(-1005690239) ;
        p127.nsats_SET((char)66) ;
        p127.rtk_rate_SET((char)168) ;
        p127.baseline_a_mm_SET(-1782989467) ;
        p127.accuracy_SET(486792809L) ;
        p127.rtk_health_SET((char)139) ;
        p127.iar_num_hypotheses_SET(1352730798) ;
        p127.baseline_b_mm_SET(529833260) ;
        p127.baseline_coords_type_SET((char)13) ;
        p127.rtk_receiver_id_SET((char)87) ;
        p127.tow_SET(2438802559L) ;
        p127.wn_SET((char)84) ;
        p127.time_last_baseline_ms_SET(393707655L) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_a_mm_GET() == -386469424);
            assert(pack.rtk_rate_GET() == (char)66);
            assert(pack.iar_num_hypotheses_GET() == -66344923);
            assert(pack.accuracy_GET() == 1354133481L);
            assert(pack.time_last_baseline_ms_GET() == 1515190074L);
            assert(pack.nsats_GET() == (char)227);
            assert(pack.baseline_coords_type_GET() == (char)17);
            assert(pack.rtk_receiver_id_GET() == (char)57);
            assert(pack.rtk_health_GET() == (char)225);
            assert(pack.wn_GET() == (char)65482);
            assert(pack.baseline_c_mm_GET() == -409092376);
            assert(pack.tow_GET() == 1997839857L);
            assert(pack.baseline_b_mm_GET() == 1644303765);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_coords_type_SET((char)17) ;
        p128.nsats_SET((char)227) ;
        p128.tow_SET(1997839857L) ;
        p128.rtk_health_SET((char)225) ;
        p128.baseline_c_mm_SET(-409092376) ;
        p128.time_last_baseline_ms_SET(1515190074L) ;
        p128.iar_num_hypotheses_SET(-66344923) ;
        p128.wn_SET((char)65482) ;
        p128.baseline_a_mm_SET(-386469424) ;
        p128.baseline_b_mm_SET(1644303765) ;
        p128.rtk_receiver_id_SET((char)57) ;
        p128.rtk_rate_SET((char)66) ;
        p128.accuracy_SET(1354133481L) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 107342999L);
            assert(pack.yacc_GET() == (short) -29906);
            assert(pack.zmag_GET() == (short) -2913);
            assert(pack.zacc_GET() == (short)9955);
            assert(pack.ymag_GET() == (short) -23099);
            assert(pack.ygyro_GET() == (short)21629);
            assert(pack.zgyro_GET() == (short) -23511);
            assert(pack.xacc_GET() == (short) -28596);
            assert(pack.xmag_GET() == (short) -10247);
            assert(pack.xgyro_GET() == (short)29466);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.zgyro_SET((short) -23511) ;
        p129.xmag_SET((short) -10247) ;
        p129.xacc_SET((short) -28596) ;
        p129.time_boot_ms_SET(107342999L) ;
        p129.zacc_SET((short)9955) ;
        p129.ygyro_SET((short)21629) ;
        p129.ymag_SET((short) -23099) ;
        p129.zmag_SET((short) -2913) ;
        p129.yacc_SET((short) -29906) ;
        p129.xgyro_SET((short)29466) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.packets_GET() == (char)26568);
            assert(pack.payload_GET() == (char)141);
            assert(pack.width_GET() == (char)5025);
            assert(pack.height_GET() == (char)48402);
            assert(pack.size_GET() == 1357811649L);
            assert(pack.type_GET() == (char)87);
            assert(pack.jpg_quality_GET() == (char)137);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.size_SET(1357811649L) ;
        p130.width_SET((char)5025) ;
        p130.type_SET((char)87) ;
        p130.jpg_quality_SET((char)137) ;
        p130.height_SET((char)48402) ;
        p130.payload_SET((char)141) ;
        p130.packets_SET((char)26568) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)230, (char)147, (char)197, (char)14, (char)232, (char)18, (char)93, (char)119, (char)185, (char)134, (char)8, (char)213, (char)158, (char)8, (char)46, (char)244, (char)9, (char)56, (char)3, (char)37, (char)5, (char)11, (char)164, (char)58, (char)25, (char)162, (char)96, (char)172, (char)230, (char)212, (char)136, (char)185, (char)37, (char)96, (char)138, (char)172, (char)73, (char)99, (char)4, (char)163, (char)151, (char)229, (char)234, (char)230, (char)47, (char)225, (char)204, (char)114, (char)49, (char)107, (char)60, (char)242, (char)115, (char)37, (char)203, (char)176, (char)238, (char)130, (char)42, (char)175, (char)242, (char)66, (char)230, (char)124, (char)152, (char)160, (char)36, (char)76, (char)173, (char)216, (char)197, (char)74, (char)151, (char)175, (char)112, (char)26, (char)132, (char)209, (char)123, (char)62, (char)221, (char)155, (char)60, (char)136, (char)203, (char)145, (char)226, (char)215, (char)195, (char)244, (char)67, (char)65, (char)87, (char)147, (char)192, (char)95, (char)84, (char)11, (char)191, (char)153, (char)4, (char)141, (char)125, (char)241, (char)10, (char)146, (char)241, (char)207, (char)47, (char)51, (char)235, (char)126, (char)135, (char)97, (char)57, (char)15, (char)20, (char)194, (char)31, (char)241, (char)99, (char)188, (char)11, (char)21, (char)116, (char)112, (char)138, (char)153, (char)253, (char)24, (char)211, (char)122, (char)74, (char)116, (char)155, (char)121, (char)113, (char)4, (char)172, (char)172, (char)166, (char)172, (char)24, (char)141, (char)92, (char)208, (char)200, (char)241, (char)80, (char)200, (char)119, (char)100, (char)98, (char)247, (char)215, (char)203, (char)221, (char)200, (char)92, (char)16, (char)160, (char)154, (char)159, (char)249, (char)226, (char)93, (char)57, (char)97, (char)43, (char)67, (char)184, (char)93, (char)121, (char)50, (char)171, (char)153, (char)130, (char)193, (char)195, (char)33, (char)144, (char)246, (char)214, (char)210, (char)139, (char)219, (char)47, (char)67, (char)41, (char)39, (char)105, (char)183, (char)180, (char)241, (char)13, (char)215, (char)15, (char)216, (char)23, (char)124, (char)157, (char)161, (char)241, (char)56, (char)65, (char)71, (char)11, (char)2, (char)1, (char)57, (char)3, (char)197, (char)83, (char)6, (char)254, (char)149, (char)186, (char)123, (char)161, (char)168, (char)233, (char)152, (char)157, (char)152, (char)36, (char)198, (char)246, (char)218, (char)114, (char)100, (char)27, (char)9, (char)31, (char)109, (char)173, (char)203, (char)238, (char)96, (char)24, (char)173, (char)45, (char)99, (char)21, (char)109, (char)210, (char)244, (char)119, (char)196, (char)101, (char)245, (char)10, (char)41, (char)3}));
            assert(pack.seqnr_GET() == (char)14467);
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)14467) ;
        p131.data__SET(new char[] {(char)230, (char)147, (char)197, (char)14, (char)232, (char)18, (char)93, (char)119, (char)185, (char)134, (char)8, (char)213, (char)158, (char)8, (char)46, (char)244, (char)9, (char)56, (char)3, (char)37, (char)5, (char)11, (char)164, (char)58, (char)25, (char)162, (char)96, (char)172, (char)230, (char)212, (char)136, (char)185, (char)37, (char)96, (char)138, (char)172, (char)73, (char)99, (char)4, (char)163, (char)151, (char)229, (char)234, (char)230, (char)47, (char)225, (char)204, (char)114, (char)49, (char)107, (char)60, (char)242, (char)115, (char)37, (char)203, (char)176, (char)238, (char)130, (char)42, (char)175, (char)242, (char)66, (char)230, (char)124, (char)152, (char)160, (char)36, (char)76, (char)173, (char)216, (char)197, (char)74, (char)151, (char)175, (char)112, (char)26, (char)132, (char)209, (char)123, (char)62, (char)221, (char)155, (char)60, (char)136, (char)203, (char)145, (char)226, (char)215, (char)195, (char)244, (char)67, (char)65, (char)87, (char)147, (char)192, (char)95, (char)84, (char)11, (char)191, (char)153, (char)4, (char)141, (char)125, (char)241, (char)10, (char)146, (char)241, (char)207, (char)47, (char)51, (char)235, (char)126, (char)135, (char)97, (char)57, (char)15, (char)20, (char)194, (char)31, (char)241, (char)99, (char)188, (char)11, (char)21, (char)116, (char)112, (char)138, (char)153, (char)253, (char)24, (char)211, (char)122, (char)74, (char)116, (char)155, (char)121, (char)113, (char)4, (char)172, (char)172, (char)166, (char)172, (char)24, (char)141, (char)92, (char)208, (char)200, (char)241, (char)80, (char)200, (char)119, (char)100, (char)98, (char)247, (char)215, (char)203, (char)221, (char)200, (char)92, (char)16, (char)160, (char)154, (char)159, (char)249, (char)226, (char)93, (char)57, (char)97, (char)43, (char)67, (char)184, (char)93, (char)121, (char)50, (char)171, (char)153, (char)130, (char)193, (char)195, (char)33, (char)144, (char)246, (char)214, (char)210, (char)139, (char)219, (char)47, (char)67, (char)41, (char)39, (char)105, (char)183, (char)180, (char)241, (char)13, (char)215, (char)15, (char)216, (char)23, (char)124, (char)157, (char)161, (char)241, (char)56, (char)65, (char)71, (char)11, (char)2, (char)1, (char)57, (char)3, (char)197, (char)83, (char)6, (char)254, (char)149, (char)186, (char)123, (char)161, (char)168, (char)233, (char)152, (char)157, (char)152, (char)36, (char)198, (char)246, (char)218, (char)114, (char)100, (char)27, (char)9, (char)31, (char)109, (char)173, (char)203, (char)238, (char)96, (char)24, (char)173, (char)45, (char)99, (char)21, (char)109, (char)210, (char)244, (char)119, (char)196, (char)101, (char)245, (char)10, (char)41, (char)3}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.covariance_GET() == (char)66);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_270);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.time_boot_ms_GET() == 2560845259L);
            assert(pack.current_distance_GET() == (char)10498);
            assert(pack.id_GET() == (char)184);
            assert(pack.min_distance_GET() == (char)21914);
            assert(pack.max_distance_GET() == (char)38332);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p132.min_distance_SET((char)21914) ;
        p132.id_SET((char)184) ;
        p132.max_distance_SET((char)38332) ;
        p132.time_boot_ms_SET(2560845259L) ;
        p132.current_distance_SET((char)10498) ;
        p132.covariance_SET((char)66) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_270) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mask_GET() == 8958505807094782777L);
            assert(pack.lon_GET() == 34774363);
            assert(pack.grid_spacing_GET() == (char)7476);
            assert(pack.lat_GET() == 1985570992);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lon_SET(34774363) ;
        p133.lat_SET(1985570992) ;
        p133.grid_spacing_SET((char)7476) ;
        p133.mask_SET(8958505807094782777L) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -492569717);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)21881, (short)25972, (short) -6231, (short) -6983, (short) -26260, (short) -23693, (short)4112, (short) -7404, (short)5166, (short)24283, (short) -32445, (short)26184, (short)942, (short)9177, (short) -4097, (short)22948}));
            assert(pack.gridbit_GET() == (char)61);
            assert(pack.grid_spacing_GET() == (char)4679);
            assert(pack.lat_GET() == 1079054327);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.data__SET(new short[] {(short)21881, (short)25972, (short) -6231, (short) -6983, (short) -26260, (short) -23693, (short)4112, (short) -7404, (short)5166, (short)24283, (short) -32445, (short)26184, (short)942, (short)9177, (short) -4097, (short)22948}, 0) ;
        p134.grid_spacing_SET((char)4679) ;
        p134.lat_SET(1079054327) ;
        p134.gridbit_SET((char)61) ;
        p134.lon_SET(-492569717) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 303789930);
            assert(pack.lat_GET() == -739265814);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-739265814) ;
        p135.lon_SET(303789930) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.spacing_GET() == (char)56471);
            assert(pack.lon_GET() == -1561052341);
            assert(pack.loaded_GET() == (char)30937);
            assert(pack.lat_GET() == 1113448927);
            assert(pack.pending_GET() == (char)29826);
            assert(pack.terrain_height_GET() == 2.17448E38F);
            assert(pack.current_height_GET() == -6.963521E37F);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lon_SET(-1561052341) ;
        p136.pending_SET((char)29826) ;
        p136.loaded_SET((char)30937) ;
        p136.lat_SET(1113448927) ;
        p136.current_height_SET(-6.963521E37F) ;
        p136.spacing_SET((char)56471) ;
        p136.terrain_height_SET(2.17448E38F) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)8611);
            assert(pack.press_abs_GET() == -2.1128389E38F);
            assert(pack.time_boot_ms_GET() == 3008467789L);
            assert(pack.press_diff_GET() == -1.246718E38F);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.temperature_SET((short)8611) ;
        p137.press_diff_SET(-1.246718E38F) ;
        p137.press_abs_SET(-2.1128389E38F) ;
        p137.time_boot_ms_SET(3008467789L) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 2.0646221E38F);
            assert(pack.time_usec_GET() == 955920356045587669L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.0734581E37F, 8.652857E37F, -4.7195963E36F, -2.8931653E37F}));
            assert(pack.z_GET() == -2.7082193E38F);
            assert(pack.x_GET() == 1.5919595E37F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.z_SET(-2.7082193E38F) ;
        p138.y_SET(2.0646221E38F) ;
        p138.q_SET(new float[] {-1.0734581E37F, 8.652857E37F, -4.7195963E36F, -2.8931653E37F}, 0) ;
        p138.time_usec_SET(955920356045587669L) ;
        p138.x_SET(1.5919595E37F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)13);
            assert(pack.time_usec_GET() == 8497824534736929345L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.2320344E38F, -1.9164835E38F, 7.8972097E37F, 1.7064043E38F, -2.984665E38F, 1.2102044E38F, -2.5998206E38F, 2.2715905E38F}));
            assert(pack.target_component_GET() == (char)80);
            assert(pack.target_system_GET() == (char)57);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.target_system_SET((char)57) ;
        p139.time_usec_SET(8497824534736929345L) ;
        p139.group_mlx_SET((char)13) ;
        p139.target_component_SET((char)80) ;
        p139.controls_SET(new float[] {-2.2320344E38F, -1.9164835E38F, 7.8972097E37F, 1.7064043E38F, -2.984665E38F, 1.2102044E38F, -2.5998206E38F, 2.2715905E38F}, 0) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.1384185E38F, 1.4329977E38F, 3.1759734E38F, 2.9459409E38F, -5.461743E37F, 2.2415267E37F, -1.1741024E38F, -2.6262226E38F}));
            assert(pack.time_usec_GET() == 103503526021022728L);
            assert(pack.group_mlx_GET() == (char)55);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(103503526021022728L) ;
        p140.group_mlx_SET((char)55) ;
        p140.controls_SET(new float[] {2.1384185E38F, 1.4329977E38F, 3.1759734E38F, 2.9459409E38F, -5.461743E37F, 2.2415267E37F, -1.1741024E38F, -2.6262226E38F}, 0) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_relative_GET() == -2.874701E38F);
            assert(pack.altitude_local_GET() == -2.4872633E38F);
            assert(pack.time_usec_GET() == 501377183786678144L);
            assert(pack.altitude_amsl_GET() == -1.4401276E38F);
            assert(pack.altitude_monotonic_GET() == -2.1933083E38F);
            assert(pack.altitude_terrain_GET() == 1.5689669E38F);
            assert(pack.bottom_clearance_GET() == 2.3712419E38F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(501377183786678144L) ;
        p141.altitude_amsl_SET(-1.4401276E38F) ;
        p141.altitude_relative_SET(-2.874701E38F) ;
        p141.altitude_monotonic_SET(-2.1933083E38F) ;
        p141.altitude_local_SET(-2.4872633E38F) ;
        p141.altitude_terrain_SET(1.5689669E38F) ;
        p141.bottom_clearance_SET(2.3712419E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.transfer_type_GET() == (char)195);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)20, (char)202, (char)104, (char)124, (char)170, (char)223, (char)110, (char)112, (char)128, (char)137, (char)237, (char)125, (char)155, (char)211, (char)35, (char)54, (char)237, (char)197, (char)37, (char)239, (char)197, (char)15, (char)56, (char)237, (char)233, (char)52, (char)91, (char)14, (char)246, (char)108, (char)152, (char)238, (char)144, (char)136, (char)94, (char)40, (char)239, (char)13, (char)41, (char)188, (char)66, (char)182, (char)138, (char)225, (char)90, (char)196, (char)37, (char)143, (char)102, (char)7, (char)59, (char)213, (char)33, (char)31, (char)234, (char)195, (char)32, (char)156, (char)135, (char)97, (char)68, (char)136, (char)240, (char)131, (char)248, (char)225, (char)78, (char)13, (char)45, (char)208, (char)28, (char)105, (char)80, (char)30, (char)100, (char)18, (char)80, (char)165, (char)230, (char)54, (char)21, (char)23, (char)112, (char)35, (char)136, (char)220, (char)230, (char)90, (char)53, (char)185, (char)129, (char)25, (char)114, (char)221, (char)117, (char)195, (char)29, (char)6, (char)95, (char)25, (char)90, (char)129, (char)60, (char)122, (char)41, (char)144, (char)212, (char)210, (char)242, (char)112, (char)16, (char)153, (char)232, (char)4, (char)221, (char)214, (char)240, (char)182, (char)73, (char)196}));
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)103, (char)81, (char)223, (char)205, (char)216, (char)204, (char)24, (char)90, (char)247, (char)14, (char)114, (char)36, (char)55, (char)118, (char)33, (char)179, (char)197, (char)145, (char)211, (char)14, (char)17, (char)78, (char)112, (char)157, (char)61, (char)170, (char)24, (char)100, (char)11, (char)8, (char)18, (char)99, (char)12, (char)211, (char)150, (char)250, (char)93, (char)232, (char)80, (char)248, (char)215, (char)104, (char)1, (char)46, (char)208, (char)125, (char)10, (char)54, (char)46, (char)104, (char)17, (char)42, (char)200, (char)236, (char)144, (char)237, (char)86, (char)190, (char)233, (char)41, (char)194, (char)35, (char)213, (char)69, (char)162, (char)177, (char)183, (char)86, (char)138, (char)51, (char)86, (char)91, (char)142, (char)72, (char)252, (char)248, (char)9, (char)85, (char)208, (char)245, (char)141, (char)94, (char)97, (char)179, (char)128, (char)10, (char)41, (char)15, (char)112, (char)123, (char)240, (char)116, (char)56, (char)104, (char)103, (char)15, (char)219, (char)151, (char)78, (char)212, (char)170, (char)220, (char)73, (char)67, (char)186, (char)243, (char)63, (char)220, (char)103, (char)139, (char)170, (char)4, (char)79, (char)31, (char)229, (char)3, (char)162, (char)148, (char)145, (char)200}));
            assert(pack.uri_type_GET() == (char)255);
            assert(pack.request_id_GET() == (char)110);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_SET(new char[] {(char)20, (char)202, (char)104, (char)124, (char)170, (char)223, (char)110, (char)112, (char)128, (char)137, (char)237, (char)125, (char)155, (char)211, (char)35, (char)54, (char)237, (char)197, (char)37, (char)239, (char)197, (char)15, (char)56, (char)237, (char)233, (char)52, (char)91, (char)14, (char)246, (char)108, (char)152, (char)238, (char)144, (char)136, (char)94, (char)40, (char)239, (char)13, (char)41, (char)188, (char)66, (char)182, (char)138, (char)225, (char)90, (char)196, (char)37, (char)143, (char)102, (char)7, (char)59, (char)213, (char)33, (char)31, (char)234, (char)195, (char)32, (char)156, (char)135, (char)97, (char)68, (char)136, (char)240, (char)131, (char)248, (char)225, (char)78, (char)13, (char)45, (char)208, (char)28, (char)105, (char)80, (char)30, (char)100, (char)18, (char)80, (char)165, (char)230, (char)54, (char)21, (char)23, (char)112, (char)35, (char)136, (char)220, (char)230, (char)90, (char)53, (char)185, (char)129, (char)25, (char)114, (char)221, (char)117, (char)195, (char)29, (char)6, (char)95, (char)25, (char)90, (char)129, (char)60, (char)122, (char)41, (char)144, (char)212, (char)210, (char)242, (char)112, (char)16, (char)153, (char)232, (char)4, (char)221, (char)214, (char)240, (char)182, (char)73, (char)196}, 0) ;
        p142.storage_SET(new char[] {(char)103, (char)81, (char)223, (char)205, (char)216, (char)204, (char)24, (char)90, (char)247, (char)14, (char)114, (char)36, (char)55, (char)118, (char)33, (char)179, (char)197, (char)145, (char)211, (char)14, (char)17, (char)78, (char)112, (char)157, (char)61, (char)170, (char)24, (char)100, (char)11, (char)8, (char)18, (char)99, (char)12, (char)211, (char)150, (char)250, (char)93, (char)232, (char)80, (char)248, (char)215, (char)104, (char)1, (char)46, (char)208, (char)125, (char)10, (char)54, (char)46, (char)104, (char)17, (char)42, (char)200, (char)236, (char)144, (char)237, (char)86, (char)190, (char)233, (char)41, (char)194, (char)35, (char)213, (char)69, (char)162, (char)177, (char)183, (char)86, (char)138, (char)51, (char)86, (char)91, (char)142, (char)72, (char)252, (char)248, (char)9, (char)85, (char)208, (char)245, (char)141, (char)94, (char)97, (char)179, (char)128, (char)10, (char)41, (char)15, (char)112, (char)123, (char)240, (char)116, (char)56, (char)104, (char)103, (char)15, (char)219, (char)151, (char)78, (char)212, (char)170, (char)220, (char)73, (char)67, (char)186, (char)243, (char)63, (char)220, (char)103, (char)139, (char)170, (char)4, (char)79, (char)31, (char)229, (char)3, (char)162, (char)148, (char)145, (char)200}, 0) ;
        p142.request_id_SET((char)110) ;
        p142.uri_type_SET((char)255) ;
        p142.transfer_type_SET((char)195) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -2.9984387E38F);
            assert(pack.press_diff_GET() == -3.0346606E38F);
            assert(pack.temperature_GET() == (short) -28162);
            assert(pack.time_boot_ms_GET() == 3440492800L);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(3440492800L) ;
        p143.temperature_SET((short) -28162) ;
        p143.press_abs_SET(-2.9984387E38F) ;
        p143.press_diff_SET(-3.0346606E38F) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.vel_GET(),  new float[] {2.7595462E38F, -2.7480247E38F, 2.815479E38F}));
            assert(pack.est_capabilities_GET() == (char)228);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-6.8305497E37F, 1.3497045E38F, -1.506547E38F, -2.2485525E38F}));
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-2.6977298E38F, -2.4432164E38F, 5.260071E37F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-5.822477E37F, 2.3399207E38F, -7.2633717E37F}));
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-2.4221667E38F, 1.5779798E38F, 2.1366701E38F}));
            assert(pack.timestamp_GET() == 8092308132878715934L);
            assert(pack.lon_GET() == -883178719);
            assert(pack.alt_GET() == -1.186018E38F);
            assert(pack.lat_GET() == 1135321785);
            assert(pack.custom_state_GET() == 2764374640213819912L);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.alt_SET(-1.186018E38F) ;
        p144.timestamp_SET(8092308132878715934L) ;
        p144.attitude_q_SET(new float[] {-6.8305497E37F, 1.3497045E38F, -1.506547E38F, -2.2485525E38F}, 0) ;
        p144.custom_state_SET(2764374640213819912L) ;
        p144.rates_SET(new float[] {-5.822477E37F, 2.3399207E38F, -7.2633717E37F}, 0) ;
        p144.est_capabilities_SET((char)228) ;
        p144.position_cov_SET(new float[] {-2.6977298E38F, -2.4432164E38F, 5.260071E37F}, 0) ;
        p144.acc_SET(new float[] {-2.4221667E38F, 1.5779798E38F, 2.1366701E38F}, 0) ;
        p144.lat_SET(1135321785) ;
        p144.lon_SET(-883178719) ;
        p144.vel_SET(new float[] {2.7595462E38F, -2.7480247E38F, 2.815479E38F}, 0) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.991415E38F, 1.3344728E38F, -3.957498E37F, -2.0668938E38F}));
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-3.2659643E38F, 2.6063065E38F, -3.3771573E38F}));
            assert(pack.pitch_rate_GET() == 2.5102786E38F);
            assert(pack.x_pos_GET() == -2.510962E38F);
            assert(pack.y_vel_GET() == -1.3083566E38F);
            assert(pack.airspeed_GET() == -2.6332474E38F);
            assert(pack.z_pos_GET() == 1.2589832E38F);
            assert(pack.y_acc_GET() == 3.1649347E38F);
            assert(pack.yaw_rate_GET() == -1.7579276E38F);
            assert(pack.y_pos_GET() == 4.0166014E37F);
            assert(pack.z_vel_GET() == -1.2857291E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {1.3475238E38F, -6.764506E37F, -1.4530221E38F}));
            assert(pack.time_usec_GET() == 8268327434329776009L);
            assert(pack.x_acc_GET() == -2.5561402E38F);
            assert(pack.z_acc_GET() == 2.527387E38F);
            assert(pack.roll_rate_GET() == -2.7832597E38F);
            assert(pack.x_vel_GET() == -1.6743944E37F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.pitch_rate_SET(2.5102786E38F) ;
        p146.roll_rate_SET(-2.7832597E38F) ;
        p146.y_vel_SET(-1.3083566E38F) ;
        p146.z_pos_SET(1.2589832E38F) ;
        p146.x_vel_SET(-1.6743944E37F) ;
        p146.x_acc_SET(-2.5561402E38F) ;
        p146.y_pos_SET(4.0166014E37F) ;
        p146.y_acc_SET(3.1649347E38F) ;
        p146.vel_variance_SET(new float[] {-3.2659643E38F, 2.6063065E38F, -3.3771573E38F}, 0) ;
        p146.airspeed_SET(-2.6332474E38F) ;
        p146.pos_variance_SET(new float[] {1.3475238E38F, -6.764506E37F, -1.4530221E38F}, 0) ;
        p146.z_vel_SET(-1.2857291E38F) ;
        p146.q_SET(new float[] {2.991415E38F, 1.3344728E38F, -3.957498E37F, -2.0668938E38F}, 0) ;
        p146.z_acc_SET(2.527387E38F) ;
        p146.yaw_rate_SET(-1.7579276E38F) ;
        p146.x_pos_SET(-2.510962E38F) ;
        p146.time_usec_SET(8268327434329776009L) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)61053, (char)62318, (char)64080, (char)53057, (char)16698, (char)57931, (char)52051, (char)35111, (char)26370, (char)19154}));
            assert(pack.energy_consumed_GET() == -1841991560);
            assert(pack.battery_remaining_GET() == (byte)44);
            assert(pack.id_GET() == (char)35);
            assert(pack.temperature_GET() == (short) -5452);
            assert(pack.current_battery_GET() == (short)32119);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
            assert(pack.current_consumed_GET() == -1376448333);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.temperature_SET((short) -5452) ;
        p147.current_consumed_SET(-1376448333) ;
        p147.id_SET((char)35) ;
        p147.energy_consumed_SET(-1841991560) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD) ;
        p147.current_battery_SET((short)32119) ;
        p147.battery_remaining_SET((byte)44) ;
        p147.voltages_SET(new char[] {(char)61053, (char)62318, (char)64080, (char)53057, (char)16698, (char)57931, (char)52051, (char)35111, (char)26370, (char)19154}, 0) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)25, (char)116, (char)16, (char)92, (char)166, (char)157, (char)135, (char)10}));
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)242, (char)164, (char)47, (char)86, (char)197, (char)57, (char)64, (char)8, (char)190, (char)232, (char)90, (char)139, (char)97, (char)45, (char)2, (char)9, (char)154, (char)139}));
            assert(pack.os_sw_version_GET() == 3295358232L);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)26, (char)189, (char)224, (char)196, (char)147, (char)40, (char)86, (char)186}));
            assert(pack.product_id_GET() == (char)12240);
            assert(pack.middleware_sw_version_GET() == 11511111L);
            assert(pack.board_version_GET() == 3586385569L);
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT));
            assert(pack.vendor_id_GET() == (char)15910);
            assert(pack.uid_GET() == 1688932272629620273L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)177, (char)212, (char)252, (char)123, (char)250, (char)148, (char)126, (char)43}));
            assert(pack.flight_sw_version_GET() == 3108448523L);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.board_version_SET(3586385569L) ;
        p148.uid_SET(1688932272629620273L) ;
        p148.product_id_SET((char)12240) ;
        p148.os_sw_version_SET(3295358232L) ;
        p148.flight_custom_version_SET(new char[] {(char)25, (char)116, (char)16, (char)92, (char)166, (char)157, (char)135, (char)10}, 0) ;
        p148.middleware_custom_version_SET(new char[] {(char)26, (char)189, (char)224, (char)196, (char)147, (char)40, (char)86, (char)186}, 0) ;
        p148.uid2_SET(new char[] {(char)242, (char)164, (char)47, (char)86, (char)197, (char)57, (char)64, (char)8, (char)190, (char)232, (char)90, (char)139, (char)97, (char)45, (char)2, (char)9, (char)154, (char)139}, 0, PH) ;
        p148.middleware_sw_version_SET(11511111L) ;
        p148.os_custom_version_SET(new char[] {(char)177, (char)212, (char)252, (char)123, (char)250, (char)148, (char)126, (char)43}, 0) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT)) ;
        p148.flight_sw_version_SET(3108448523L) ;
        p148.vendor_id_SET((char)15910) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.angle_y_GET() == -2.4312157E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
            assert(pack.x_TRY(ph) == 2.9507452E38F);
            assert(pack.y_TRY(ph) == 2.0139266E38F);
            assert(pack.angle_x_GET() == 1.8041463E38F);
            assert(pack.size_y_GET() == -2.2638713E37F);
            assert(pack.position_valid_TRY(ph) == (char)240);
            assert(pack.z_TRY(ph) == -1.722683E38F);
            assert(pack.distance_GET() == -3.2012482E38F);
            assert(pack.target_num_GET() == (char)41);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.size_x_GET() == 5.853397E37F);
            assert(pack.time_usec_GET() == 6942022055134091658L);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {-2.4871463E38F, 3.1009557E38F, 4.8769023E37F, -2.4495928E38F}));
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.z_SET(-1.722683E38F, PH) ;
        p149.size_x_SET(5.853397E37F) ;
        p149.time_usec_SET(6942022055134091658L) ;
        p149.size_y_SET(-2.2638713E37F) ;
        p149.angle_x_SET(1.8041463E38F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p149.y_SET(2.0139266E38F, PH) ;
        p149.angle_y_SET(-2.4312157E38F) ;
        p149.q_SET(new float[] {-2.4871463E38F, 3.1009557E38F, 4.8769023E37F, -2.4495928E38F}, 0, PH) ;
        p149.target_num_SET((char)41) ;
        p149.x_SET(2.9507452E38F, PH) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER) ;
        p149.position_valid_SET((char)240, PH) ;
        p149.distance_SET(-3.2012482E38F) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_0.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_u16_GET(),  new char[] {(char)53584, (char)15396, (char)60899, (char)7211}));
            assert(pack.v1_GET() == (char)200);
            assert(Arrays.equals(pack.ar_i8_GET(),  new byte[] {(byte) - 47, (byte) - 53, (byte)8, (byte)90}));
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {605580111L, 4104565718L, 1308763066L, 1032358122L}));
            assert(Arrays.equals(pack.ar_u8_GET(),  new char[] {(char)89, (char)101, (char)39, (char)251}));
        });
        GroundControl.ARRAY_TEST_0 p150 = CommunicationChannel.new_ARRAY_TEST_0();
        PH.setPack(p150);
        p150.ar_u8_SET(new char[] {(char)89, (char)101, (char)39, (char)251}, 0) ;
        p150.v1_SET((char)200) ;
        p150.ar_i8_SET(new byte[] {(byte) - 47, (byte) - 53, (byte)8, (byte)90}, 0) ;
        p150.ar_u32_SET(new long[] {605580111L, 4104565718L, 1308763066L, 1032358122L}, 0) ;
        p150.ar_u16_SET(new char[] {(char)53584, (char)15396, (char)60899, (char)7211}, 0) ;
        CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_1.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {592686171L, 2748399623L, 4044945753L, 366936358L}));
        });
        GroundControl.ARRAY_TEST_1 p151 = CommunicationChannel.new_ARRAY_TEST_1();
        PH.setPack(p151);
        p151.ar_u32_SET(new long[] {592686171L, 2748399623L, 4044945753L, 366936358L}, 0) ;
        CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_3.add((src, ph, pack) ->
        {
            assert(pack.v_GET() == (char)192);
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {913022044L, 2603483660L, 3223541632L, 1242650195L}));
        });
        GroundControl.ARRAY_TEST_3 p153 = CommunicationChannel.new_ARRAY_TEST_3();
        PH.setPack(p153);
        p153.ar_u32_SET(new long[] {913022044L, 2603483660L, 3223541632L, 1242650195L}, 0) ;
        p153.v_SET((char)192) ;
        CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_4.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {4061405562L, 2315484641L, 1214589056L, 4166980773L}));
            assert(pack.v_GET() == (char)60);
        });
        GroundControl.ARRAY_TEST_4 p154 = CommunicationChannel.new_ARRAY_TEST_4();
        PH.setPack(p154);
        p154.v_SET((char)60) ;
        p154.ar_u32_SET(new long[] {4061405562L, 2315484641L, 1214589056L, 4166980773L}, 0) ;
        CommunicationChannel.instance.send(p154);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_5.add((src, ph, pack) ->
        {
            assert(pack.c2_LEN(ph) == 4);
            assert(pack.c2_TRY(ph).equals("dvmf"));
            assert(pack.c1_LEN(ph) == 3);
            assert(pack.c1_TRY(ph).equals("ewQ"));
        });
        GroundControl.ARRAY_TEST_5 p155 = CommunicationChannel.new_ARRAY_TEST_5();
        PH.setPack(p155);
        p155.c2_SET("dvmf", PH) ;
        p155.c1_SET("ewQ", PH) ;
        CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_6.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_i8_GET(),  new byte[] {(byte)1, (byte)6}));
            assert(Arrays.equals(pack.ar_u16_GET(),  new char[] {(char)23855, (char)50904}));
            assert(Arrays.equals(pack.ar_i32_GET(),  new int[] {352928652, -955425202}));
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {988213927L, 3423193721L}));
            assert(pack.v3_GET() == 1526850637L);
            assert(Arrays.equals(pack.ar_u8_GET(),  new char[] {(char)204, (char)81}));
            assert(pack.v2_GET() == (char)35832);
            assert(Arrays.equals(pack.ar_f_GET(),  new float[] {1.9701641E38F, -5.500139E37F}));
            assert(Arrays.equals(pack.ar_d_GET(),  new double[] {3.476632073067498E307, -9.759647696334233E307}));
            assert(Arrays.equals(pack.ar_i16_GET(),  new short[] {(short)27274, (short)9029}));
            assert(pack.v1_GET() == (char)135);
            assert(pack.ar_c_LEN(ph) == 15);
            assert(pack.ar_c_TRY(ph).equals("lIkbhtdjpvztiuz"));
        });
        GroundControl.ARRAY_TEST_6 p156 = CommunicationChannel.new_ARRAY_TEST_6();
        PH.setPack(p156);
        p156.v3_SET(1526850637L) ;
        p156.ar_d_SET(new double[] {3.476632073067498E307, -9.759647696334233E307}, 0) ;
        p156.ar_u16_SET(new char[] {(char)23855, (char)50904}, 0) ;
        p156.ar_i32_SET(new int[] {352928652, -955425202}, 0) ;
        p156.ar_i16_SET(new short[] {(short)27274, (short)9029}, 0) ;
        p156.ar_u8_SET(new char[] {(char)204, (char)81}, 0) ;
        p156.v2_SET((char)35832) ;
        p156.ar_c_SET("lIkbhtdjpvztiuz", PH) ;
        p156.ar_u32_SET(new long[] {988213927L, 3423193721L}, 0) ;
        p156.v1_SET((char)135) ;
        p156.ar_i8_SET(new byte[] {(byte)1, (byte)6}, 0) ;
        p156.ar_f_SET(new float[] {1.9701641E38F, -5.500139E37F}, 0) ;
        CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_7.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_i8_GET(),  new byte[] {(byte) - 60, (byte)51}));
            assert(Arrays.equals(pack.ar_f_GET(),  new float[] {1.5707304E38F, -1.2134878E38F}));
            assert(pack.ar_c_LEN(ph) == 20);
            assert(pack.ar_c_TRY(ph).equals("aKmvafdavcjzrxfdnaHe"));
            assert(Arrays.equals(pack.ar_i32_GET(),  new int[] {-1987382390, 294135927}));
            assert(Arrays.equals(pack.ar_u8_GET(),  new char[] {(char)54, (char)255}));
            assert(Arrays.equals(pack.ar_u16_GET(),  new char[] {(char)6381, (char)39681}));
            assert(Arrays.equals(pack.ar_i16_GET(),  new short[] {(short)8321, (short)2391}));
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {2030812504L, 3848196817L}));
            assert(Arrays.equals(pack.ar_d_GET(),  new double[] {-1.4983250511879937E308, -1.7214362891547993E308}));
        });
        GroundControl.ARRAY_TEST_7 p157 = CommunicationChannel.new_ARRAY_TEST_7();
        PH.setPack(p157);
        p157.ar_c_SET("aKmvafdavcjzrxfdnaHe", PH) ;
        p157.ar_i32_SET(new int[] {-1987382390, 294135927}, 0) ;
        p157.ar_u8_SET(new char[] {(char)54, (char)255}, 0) ;
        p157.ar_d_SET(new double[] {-1.4983250511879937E308, -1.7214362891547993E308}, 0) ;
        p157.ar_f_SET(new float[] {1.5707304E38F, -1.2134878E38F}, 0) ;
        p157.ar_i8_SET(new byte[] {(byte) - 60, (byte)51}, 0) ;
        p157.ar_u16_SET(new char[] {(char)6381, (char)39681}, 0) ;
        p157.ar_i16_SET(new short[] {(short)8321, (short)2391}, 0) ;
        p157.ar_u32_SET(new long[] {2030812504L, 3848196817L}, 0) ;
        CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_8.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_u16_GET(),  new char[] {(char)32472, (char)46777}));
            assert(Arrays.equals(pack.ar_d_GET(),  new double[] {9.242084145908888E307, 1.5165963191609034E308}));
            assert(pack.v3_GET() == 1722238641L);
        });
        GroundControl.ARRAY_TEST_8 p158 = CommunicationChannel.new_ARRAY_TEST_8();
        PH.setPack(p158);
        p158.ar_d_SET(new double[] {9.242084145908888E307, 1.5165963191609034E308}, 0) ;
        p158.ar_u16_SET(new char[] {(char)32472, (char)46777}, 0) ;
        p158.v3_SET(1722238641L) ;
        CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.vel_ratio_GET() == 9.572602E37F);
            assert(pack.pos_vert_ratio_GET() == 5.7126294E37F);
            assert(pack.mag_ratio_GET() == -2.413776E37F);
            assert(pack.pos_horiz_accuracy_GET() == -1.2844718E38F);
            assert(pack.pos_vert_accuracy_GET() == 8.85101E37F);
            assert(pack.hagl_ratio_GET() == 2.408177E38F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL));
            assert(pack.tas_ratio_GET() == -3.277493E38F);
            assert(pack.pos_horiz_ratio_GET() == 1.3153424E37F);
            assert(pack.time_usec_GET() == 7147980232260402098L);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.hagl_ratio_SET(2.408177E38F) ;
        p230.vel_ratio_SET(9.572602E37F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL)) ;
        p230.mag_ratio_SET(-2.413776E37F) ;
        p230.time_usec_SET(7147980232260402098L) ;
        p230.pos_vert_ratio_SET(5.7126294E37F) ;
        p230.pos_horiz_ratio_SET(1.3153424E37F) ;
        p230.tas_ratio_SET(-3.277493E38F) ;
        p230.pos_horiz_accuracy_SET(-1.2844718E38F) ;
        p230.pos_vert_accuracy_SET(8.85101E37F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4493047032214985804L);
            assert(pack.var_vert_GET() == -1.2050744E38F);
            assert(pack.var_horiz_GET() == -1.4974551E38F);
            assert(pack.wind_y_GET() == 2.1937122E38F);
            assert(pack.wind_z_GET() == 1.8453942E38F);
            assert(pack.vert_accuracy_GET() == -1.7112825E38F);
            assert(pack.wind_alt_GET() == 1.1469885E38F);
            assert(pack.wind_x_GET() == 2.946904E38F);
            assert(pack.horiz_accuracy_GET() == -5.668053E37F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.vert_accuracy_SET(-1.7112825E38F) ;
        p231.wind_y_SET(2.1937122E38F) ;
        p231.horiz_accuracy_SET(-5.668053E37F) ;
        p231.wind_z_SET(1.8453942E38F) ;
        p231.wind_x_SET(2.946904E38F) ;
        p231.var_horiz_SET(-1.4974551E38F) ;
        p231.wind_alt_SET(1.1469885E38F) ;
        p231.time_usec_SET(4493047032214985804L) ;
        p231.var_vert_SET(-1.2050744E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.hdop_GET() == -1.8987491E38F);
            assert(pack.time_usec_GET() == 3528927007861829086L);
            assert(pack.ve_GET() == -2.783411E38F);
            assert(pack.speed_accuracy_GET() == -7.524521E37F);
            assert(pack.fix_type_GET() == (char)34);
            assert(pack.time_week_ms_GET() == 4081958938L);
            assert(pack.gps_id_GET() == (char)217);
            assert(pack.time_week_GET() == (char)62616);
            assert(pack.lat_GET() == -1471897889);
            assert(pack.vdop_GET() == 2.5995628E38F);
            assert(pack.vd_GET() == 9.234604E37F);
            assert(pack.satellites_visible_GET() == (char)172);
            assert(pack.vn_GET() == -6.2331354E37F);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT));
            assert(pack.lon_GET() == 851768490);
            assert(pack.alt_GET() == 2.6088383E38F);
            assert(pack.horiz_accuracy_GET() == -1.9705207E38F);
            assert(pack.vert_accuracy_GET() == -2.2315727E38F);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_week_SET((char)62616) ;
        p232.alt_SET(2.6088383E38F) ;
        p232.speed_accuracy_SET(-7.524521E37F) ;
        p232.lon_SET(851768490) ;
        p232.vn_SET(-6.2331354E37F) ;
        p232.vdop_SET(2.5995628E38F) ;
        p232.lat_SET(-1471897889) ;
        p232.time_week_ms_SET(4081958938L) ;
        p232.horiz_accuracy_SET(-1.9705207E38F) ;
        p232.fix_type_SET((char)34) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT)) ;
        p232.ve_SET(-2.783411E38F) ;
        p232.gps_id_SET((char)217) ;
        p232.vd_SET(9.234604E37F) ;
        p232.vert_accuracy_SET(-2.2315727E38F) ;
        p232.hdop_SET(-1.8987491E38F) ;
        p232.satellites_visible_SET((char)172) ;
        p232.time_usec_SET(3528927007861829086L) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)249);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)174, (char)221, (char)205, (char)187, (char)235, (char)162, (char)123, (char)143, (char)145, (char)199, (char)144, (char)242, (char)92, (char)19, (char)63, (char)87, (char)86, (char)188, (char)244, (char)106, (char)240, (char)72, (char)184, (char)27, (char)111, (char)32, (char)47, (char)49, (char)71, (char)228, (char)208, (char)104, (char)90, (char)124, (char)204, (char)159, (char)7, (char)159, (char)221, (char)169, (char)162, (char)5, (char)187, (char)132, (char)211, (char)28, (char)22, (char)163, (char)31, (char)83, (char)112, (char)38, (char)44, (char)197, (char)245, (char)241, (char)31, (char)142, (char)160, (char)21, (char)169, (char)69, (char)123, (char)242, (char)132, (char)234, (char)68, (char)25, (char)14, (char)97, (char)231, (char)102, (char)228, (char)164, (char)0, (char)218, (char)179, (char)54, (char)67, (char)113, (char)234, (char)30, (char)152, (char)210, (char)80, (char)229, (char)38, (char)206, (char)121, (char)90, (char)76, (char)162, (char)196, (char)153, (char)244, (char)53, (char)29, (char)154, (char)30, (char)119, (char)132, (char)76, (char)248, (char)240, (char)148, (char)247, (char)171, (char)48, (char)237, (char)163, (char)254, (char)80, (char)6, (char)114, (char)32, (char)243, (char)108, (char)50, (char)52, (char)99, (char)175, (char)160, (char)79, (char)200, (char)188, (char)103, (char)87, (char)4, (char)179, (char)140, (char)32, (char)26, (char)227, (char)163, (char)65, (char)99, (char)172, (char)139, (char)202, (char)71, (char)246, (char)189, (char)222, (char)70, (char)205, (char)15, (char)204, (char)126, (char)147, (char)133, (char)168, (char)122, (char)198, (char)37, (char)12, (char)120, (char)49, (char)204, (char)201, (char)161, (char)57, (char)251, (char)146, (char)99, (char)96, (char)249, (char)63, (char)49, (char)77, (char)154, (char)92, (char)176, (char)20, (char)108, (char)42, (char)54, (char)6, (char)127, (char)139, (char)121}));
            assert(pack.flags_GET() == (char)25);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.len_SET((char)249) ;
        p233.flags_SET((char)25) ;
        p233.data__SET(new char[] {(char)174, (char)221, (char)205, (char)187, (char)235, (char)162, (char)123, (char)143, (char)145, (char)199, (char)144, (char)242, (char)92, (char)19, (char)63, (char)87, (char)86, (char)188, (char)244, (char)106, (char)240, (char)72, (char)184, (char)27, (char)111, (char)32, (char)47, (char)49, (char)71, (char)228, (char)208, (char)104, (char)90, (char)124, (char)204, (char)159, (char)7, (char)159, (char)221, (char)169, (char)162, (char)5, (char)187, (char)132, (char)211, (char)28, (char)22, (char)163, (char)31, (char)83, (char)112, (char)38, (char)44, (char)197, (char)245, (char)241, (char)31, (char)142, (char)160, (char)21, (char)169, (char)69, (char)123, (char)242, (char)132, (char)234, (char)68, (char)25, (char)14, (char)97, (char)231, (char)102, (char)228, (char)164, (char)0, (char)218, (char)179, (char)54, (char)67, (char)113, (char)234, (char)30, (char)152, (char)210, (char)80, (char)229, (char)38, (char)206, (char)121, (char)90, (char)76, (char)162, (char)196, (char)153, (char)244, (char)53, (char)29, (char)154, (char)30, (char)119, (char)132, (char)76, (char)248, (char)240, (char)148, (char)247, (char)171, (char)48, (char)237, (char)163, (char)254, (char)80, (char)6, (char)114, (char)32, (char)243, (char)108, (char)50, (char)52, (char)99, (char)175, (char)160, (char)79, (char)200, (char)188, (char)103, (char)87, (char)4, (char)179, (char)140, (char)32, (char)26, (char)227, (char)163, (char)65, (char)99, (char)172, (char)139, (char)202, (char)71, (char)246, (char)189, (char)222, (char)70, (char)205, (char)15, (char)204, (char)126, (char)147, (char)133, (char)168, (char)122, (char)198, (char)37, (char)12, (char)120, (char)49, (char)204, (char)201, (char)161, (char)57, (char)251, (char)146, (char)99, (char)96, (char)249, (char)63, (char)49, (char)77, (char)154, (char)92, (char)176, (char)20, (char)108, (char)42, (char)54, (char)6, (char)127, (char)139, (char)121}, 0) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 742299646);
            assert(pack.battery_remaining_GET() == (char)6);
            assert(pack.wp_distance_GET() == (char)17278);
            assert(pack.pitch_GET() == (short) -16445);
            assert(pack.climb_rate_GET() == (byte)7);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.heading_sp_GET() == (short)14697);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
            assert(pack.longitude_GET() == -1914051020);
            assert(pack.temperature_GET() == (byte) - 85);
            assert(pack.wp_num_GET() == (char)192);
            assert(pack.groundspeed_GET() == (char)182);
            assert(pack.airspeed_sp_GET() == (char)24);
            assert(pack.altitude_sp_GET() == (short)11654);
            assert(pack.altitude_amsl_GET() == (short) -31473);
            assert(pack.temperature_air_GET() == (byte) - 70);
            assert(pack.throttle_GET() == (byte)30);
            assert(pack.heading_GET() == (char)27428);
            assert(pack.roll_GET() == (short) -24967);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
            assert(pack.airspeed_GET() == (char)51);
            assert(pack.failsafe_GET() == (char)254);
            assert(pack.custom_mode_GET() == 1023099024L);
            assert(pack.gps_nsat_GET() == (char)84);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.heading_sp_SET((short)14697) ;
        p234.airspeed_sp_SET((char)24) ;
        p234.wp_distance_SET((char)17278) ;
        p234.heading_SET((char)27428) ;
        p234.pitch_SET((short) -16445) ;
        p234.latitude_SET(742299646) ;
        p234.gps_nsat_SET((char)84) ;
        p234.climb_rate_SET((byte)7) ;
        p234.wp_num_SET((char)192) ;
        p234.battery_remaining_SET((char)6) ;
        p234.throttle_SET((byte)30) ;
        p234.airspeed_SET((char)51) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) ;
        p234.altitude_sp_SET((short)11654) ;
        p234.longitude_SET(-1914051020) ;
        p234.temperature_air_SET((byte) - 70) ;
        p234.altitude_amsl_SET((short) -31473) ;
        p234.groundspeed_SET((char)182) ;
        p234.custom_mode_SET(1023099024L) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        p234.temperature_SET((byte) - 85) ;
        p234.roll_SET((short) -24967) ;
        p234.failsafe_SET((char)254) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.clipping_1_GET() == 1256973029L);
            assert(pack.clipping_0_GET() == 3741332629L);
            assert(pack.clipping_2_GET() == 1912425793L);
            assert(pack.vibration_x_GET() == -3.2439885E38F);
            assert(pack.vibration_z_GET() == -3.0008667E38F);
            assert(pack.time_usec_GET() == 3605992971620005119L);
            assert(pack.vibration_y_GET() == 1.3639376E38F);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.vibration_z_SET(-3.0008667E38F) ;
        p241.time_usec_SET(3605992971620005119L) ;
        p241.vibration_y_SET(1.3639376E38F) ;
        p241.clipping_0_SET(3741332629L) ;
        p241.vibration_x_SET(-3.2439885E38F) ;
        p241.clipping_2_SET(1912425793L) ;
        p241.clipping_1_SET(1256973029L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 731975743);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.0218746E38F, -2.533776E38F, 3.3988108E37F, 2.2647702E38F}));
            assert(pack.x_GET() == -1.0074442E38F);
            assert(pack.altitude_GET() == 542970768);
            assert(pack.longitude_GET() == -641520809);
            assert(pack.approach_z_GET() == -8.127944E37F);
            assert(pack.time_usec_TRY(ph) == 2044688616913061246L);
            assert(pack.approach_y_GET() == 3.0198348E38F);
            assert(pack.y_GET() == 6.3880935E37F);
            assert(pack.z_GET() == -2.6627703E38F);
            assert(pack.approach_x_GET() == -2.5840413E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.approach_z_SET(-8.127944E37F) ;
        p242.x_SET(-1.0074442E38F) ;
        p242.time_usec_SET(2044688616913061246L, PH) ;
        p242.approach_x_SET(-2.5840413E38F) ;
        p242.latitude_SET(731975743) ;
        p242.altitude_SET(542970768) ;
        p242.z_SET(-2.6627703E38F) ;
        p242.approach_y_SET(3.0198348E38F) ;
        p242.longitude_SET(-641520809) ;
        p242.q_SET(new float[] {-1.0218746E38F, -2.533776E38F, 3.3988108E37F, 2.2647702E38F}, 0) ;
        p242.y_SET(6.3880935E37F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -2.451897E38F);
            assert(pack.time_usec_TRY(ph) == 2736543786358248062L);
            assert(pack.z_GET() == 1.103896E38F);
            assert(pack.altitude_GET() == 536966741);
            assert(pack.longitude_GET() == 737058102);
            assert(pack.latitude_GET() == -359210068);
            assert(pack.approach_z_GET() == -1.9712243E38F);
            assert(pack.x_GET() == 2.3248306E38F);
            assert(pack.approach_y_GET() == -3.0543103E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.2150274E37F, -1.2604446E38F, -2.736539E38F, -4.601489E36F}));
            assert(pack.approach_x_GET() == 8.515975E37F);
            assert(pack.target_system_GET() == (char)167);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.latitude_SET(-359210068) ;
        p243.q_SET(new float[] {1.2150274E37F, -1.2604446E38F, -2.736539E38F, -4.601489E36F}, 0) ;
        p243.approach_x_SET(8.515975E37F) ;
        p243.approach_z_SET(-1.9712243E38F) ;
        p243.x_SET(2.3248306E38F) ;
        p243.altitude_SET(536966741) ;
        p243.approach_y_SET(-3.0543103E37F) ;
        p243.time_usec_SET(2736543786358248062L, PH) ;
        p243.target_system_SET((char)167) ;
        p243.y_SET(-2.451897E38F) ;
        p243.longitude_SET(737058102) ;
        p243.z_SET(1.103896E38F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == -998911362);
            assert(pack.message_id_GET() == (char)63302);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-998911362) ;
        p244.message_id_SET((char)63302) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -112919116);
            assert(pack.tslc_GET() == (char)24);
            assert(pack.altitude_GET() == 597867071);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY));
            assert(pack.ver_velocity_GET() == (short) -19959);
            assert(pack.ICAO_address_GET() == 1976305131L);
            assert(pack.heading_GET() == (char)62773);
            assert(pack.lat_GET() == 688150014);
            assert(pack.squawk_GET() == (char)61992);
            assert(pack.callsign_LEN(ph) == 6);
            assert(pack.callsign_TRY(ph).equals("gvctpF"));
            assert(pack.hor_velocity_GET() == (char)3694);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_POINT_OBSTACLE);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY)) ;
        p246.ICAO_address_SET(1976305131L) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_POINT_OBSTACLE) ;
        p246.lon_SET(-112919116) ;
        p246.tslc_SET((char)24) ;
        p246.heading_SET((char)62773) ;
        p246.hor_velocity_SET((char)3694) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.ver_velocity_SET((short) -19959) ;
        p246.callsign_SET("gvctpF", PH) ;
        p246.squawk_SET((char)61992) ;
        p246.altitude_SET(597867071) ;
        p246.lat_SET(688150014) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.horizontal_minimum_delta_GET() == 1.2745509E38F);
            assert(pack.altitude_minimum_delta_GET() == -4.716318E36F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
            assert(pack.time_to_minimum_delta_GET() == 1.5134212E38F);
            assert(pack.id_GET() == 1329731886L);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.horizontal_minimum_delta_SET(1.2745509E38F) ;
        p247.altitude_minimum_delta_SET(-4.716318E36F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW) ;
        p247.time_to_minimum_delta_SET(1.5134212E38F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT) ;
        p247.id_SET(1329731886L) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)167);
            assert(pack.target_system_GET() == (char)95);
            assert(pack.target_network_GET() == (char)174);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)248, (char)64, (char)142, (char)174, (char)212, (char)193, (char)249, (char)160, (char)127, (char)184, (char)47, (char)96, (char)14, (char)146, (char)177, (char)19, (char)208, (char)27, (char)195, (char)183, (char)129, (char)239, (char)91, (char)153, (char)76, (char)255, (char)244, (char)128, (char)160, (char)255, (char)176, (char)120, (char)172, (char)199, (char)151, (char)181, (char)85, (char)20, (char)1, (char)156, (char)34, (char)142, (char)79, (char)163, (char)67, (char)250, (char)63, (char)249, (char)0, (char)105, (char)121, (char)4, (char)32, (char)107, (char)110, (char)82, (char)126, (char)42, (char)255, (char)218, (char)177, (char)187, (char)184, (char)212, (char)199, (char)4, (char)107, (char)42, (char)1, (char)95, (char)189, (char)12, (char)38, (char)100, (char)185, (char)211, (char)238, (char)244, (char)99, (char)194, (char)222, (char)131, (char)75, (char)128, (char)4, (char)16, (char)232, (char)3, (char)27, (char)213, (char)244, (char)209, (char)189, (char)209, (char)172, (char)64, (char)45, (char)99, (char)221, (char)14, (char)158, (char)158, (char)213, (char)127, (char)126, (char)145, (char)75, (char)118, (char)140, (char)83, (char)46, (char)62, (char)226, (char)108, (char)1, (char)39, (char)153, (char)134, (char)69, (char)66, (char)4, (char)3, (char)228, (char)96, (char)69, (char)118, (char)68, (char)60, (char)221, (char)201, (char)7, (char)153, (char)97, (char)233, (char)241, (char)99, (char)222, (char)106, (char)78, (char)132, (char)112, (char)182, (char)236, (char)31, (char)57, (char)76, (char)204, (char)80, (char)56, (char)201, (char)241, (char)241, (char)3, (char)138, (char)82, (char)245, (char)66, (char)53, (char)187, (char)141, (char)100, (char)178, (char)15, (char)116, (char)135, (char)69, (char)102, (char)40, (char)83, (char)254, (char)232, (char)48, (char)164, (char)113, (char)254, (char)122, (char)92, (char)235, (char)216, (char)255, (char)110, (char)33, (char)160, (char)2, (char)255, (char)204, (char)206, (char)18, (char)91, (char)220, (char)91, (char)92, (char)144, (char)254, (char)158, (char)47, (char)3, (char)254, (char)145, (char)204, (char)125, (char)26, (char)86, (char)44, (char)54, (char)224, (char)193, (char)20, (char)115, (char)17, (char)72, (char)126, (char)119, (char)190, (char)75, (char)31, (char)189, (char)34, (char)136, (char)93, (char)8, (char)178, (char)82, (char)79, (char)181, (char)17, (char)95, (char)10, (char)72, (char)144, (char)202, (char)236, (char)84, (char)164, (char)229, (char)70, (char)176, (char)76, (char)99, (char)224, (char)170, (char)25, (char)229, (char)94, (char)187, (char)2, (char)177, (char)170, (char)45}));
            assert(pack.message_type_GET() == (char)34482);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.message_type_SET((char)34482) ;
        p248.target_component_SET((char)167) ;
        p248.payload_SET(new char[] {(char)248, (char)64, (char)142, (char)174, (char)212, (char)193, (char)249, (char)160, (char)127, (char)184, (char)47, (char)96, (char)14, (char)146, (char)177, (char)19, (char)208, (char)27, (char)195, (char)183, (char)129, (char)239, (char)91, (char)153, (char)76, (char)255, (char)244, (char)128, (char)160, (char)255, (char)176, (char)120, (char)172, (char)199, (char)151, (char)181, (char)85, (char)20, (char)1, (char)156, (char)34, (char)142, (char)79, (char)163, (char)67, (char)250, (char)63, (char)249, (char)0, (char)105, (char)121, (char)4, (char)32, (char)107, (char)110, (char)82, (char)126, (char)42, (char)255, (char)218, (char)177, (char)187, (char)184, (char)212, (char)199, (char)4, (char)107, (char)42, (char)1, (char)95, (char)189, (char)12, (char)38, (char)100, (char)185, (char)211, (char)238, (char)244, (char)99, (char)194, (char)222, (char)131, (char)75, (char)128, (char)4, (char)16, (char)232, (char)3, (char)27, (char)213, (char)244, (char)209, (char)189, (char)209, (char)172, (char)64, (char)45, (char)99, (char)221, (char)14, (char)158, (char)158, (char)213, (char)127, (char)126, (char)145, (char)75, (char)118, (char)140, (char)83, (char)46, (char)62, (char)226, (char)108, (char)1, (char)39, (char)153, (char)134, (char)69, (char)66, (char)4, (char)3, (char)228, (char)96, (char)69, (char)118, (char)68, (char)60, (char)221, (char)201, (char)7, (char)153, (char)97, (char)233, (char)241, (char)99, (char)222, (char)106, (char)78, (char)132, (char)112, (char)182, (char)236, (char)31, (char)57, (char)76, (char)204, (char)80, (char)56, (char)201, (char)241, (char)241, (char)3, (char)138, (char)82, (char)245, (char)66, (char)53, (char)187, (char)141, (char)100, (char)178, (char)15, (char)116, (char)135, (char)69, (char)102, (char)40, (char)83, (char)254, (char)232, (char)48, (char)164, (char)113, (char)254, (char)122, (char)92, (char)235, (char)216, (char)255, (char)110, (char)33, (char)160, (char)2, (char)255, (char)204, (char)206, (char)18, (char)91, (char)220, (char)91, (char)92, (char)144, (char)254, (char)158, (char)47, (char)3, (char)254, (char)145, (char)204, (char)125, (char)26, (char)86, (char)44, (char)54, (char)224, (char)193, (char)20, (char)115, (char)17, (char)72, (char)126, (char)119, (char)190, (char)75, (char)31, (char)189, (char)34, (char)136, (char)93, (char)8, (char)178, (char)82, (char)79, (char)181, (char)17, (char)95, (char)10, (char)72, (char)144, (char)202, (char)236, (char)84, (char)164, (char)229, (char)70, (char)176, (char)76, (char)99, (char)224, (char)170, (char)25, (char)229, (char)94, (char)187, (char)2, (char)177, (char)170, (char)45}, 0) ;
        p248.target_network_SET((char)174) ;
        p248.target_system_SET((char)95) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)134);
            assert(pack.ver_GET() == (char)235);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 77, (byte) - 123, (byte)124, (byte) - 116, (byte) - 26, (byte) - 55, (byte)35, (byte) - 58, (byte)118, (byte)42, (byte)1, (byte)34, (byte) - 27, (byte) - 53, (byte)44, (byte)109, (byte)39, (byte)74, (byte)89, (byte)29, (byte)33, (byte)95, (byte) - 86, (byte)111, (byte)72, (byte)15, (byte) - 124, (byte) - 96, (byte) - 95, (byte)69, (byte)3, (byte)66}));
            assert(pack.address_GET() == (char)8371);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.ver_SET((char)235) ;
        p249.value_SET(new byte[] {(byte) - 77, (byte) - 123, (byte)124, (byte) - 116, (byte) - 26, (byte) - 55, (byte)35, (byte) - 58, (byte)118, (byte)42, (byte)1, (byte)34, (byte) - 27, (byte) - 53, (byte)44, (byte)109, (byte)39, (byte)74, (byte)89, (byte)29, (byte)33, (byte)95, (byte) - 86, (byte)111, (byte)72, (byte)15, (byte) - 124, (byte) - 96, (byte) - 95, (byte)69, (byte)3, (byte)66}, 0) ;
        p249.address_SET((char)8371) ;
        p249.type_SET((char)134) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6225656669939628452L);
            assert(pack.y_GET() == 3.0141007E38F);
            assert(pack.name_LEN(ph) == 8);
            assert(pack.name_TRY(ph).equals("apxwogpe"));
            assert(pack.x_GET() == -4.0625243E37F);
            assert(pack.z_GET() == 2.2796443E38F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.y_SET(3.0141007E38F) ;
        p250.x_SET(-4.0625243E37F) ;
        p250.name_SET("apxwogpe", PH) ;
        p250.time_usec_SET(6225656669939628452L) ;
        p250.z_SET(2.2796443E38F) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 4267916623L);
            assert(pack.value_GET() == 6.5068373E37F);
            assert(pack.name_LEN(ph) == 1);
            assert(pack.name_TRY(ph).equals("c"));
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(4267916623L) ;
        p251.value_SET(6.5068373E37F) ;
        p251.name_SET("c", PH) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2867829123L);
            assert(pack.name_LEN(ph) == 4);
            assert(pack.name_TRY(ph).equals("osdn"));
            assert(pack.value_GET() == 606110053);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(2867829123L) ;
        p252.value_SET(606110053) ;
        p252.name_SET("osdn", PH) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 40);
            assert(pack.text_TRY(ph).equals("vgyfcactsSivdzutvblhTwpgaxmfwkfocoieshcr"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("vgyfcactsSivdzutvblhTwpgaxmfwkfocoieshcr", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_EMERGENCY) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.ind_GET() == (char)97);
            assert(pack.time_boot_ms_GET() == 1130668316L);
            assert(pack.value_GET() == 1.397969E38F);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.value_SET(1.397969E38F) ;
        p254.time_boot_ms_SET(1130668316L) ;
        p254.ind_SET((char)97) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)204);
            assert(pack.initial_timestamp_GET() == 6185802749381844977L);
            assert(pack.target_component_GET() == (char)135);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)140, (char)1, (char)19, (char)20, (char)40, (char)135, (char)150, (char)149, (char)128, (char)85, (char)54, (char)72, (char)32, (char)4, (char)106, (char)133, (char)144, (char)132, (char)75, (char)93, (char)209, (char)104, (char)11, (char)88, (char)45, (char)152, (char)121, (char)242, (char)162, (char)183, (char)113, (char)237}));
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_component_SET((char)135) ;
        p256.secret_key_SET(new char[] {(char)140, (char)1, (char)19, (char)20, (char)40, (char)135, (char)150, (char)149, (char)128, (char)85, (char)54, (char)72, (char)32, (char)4, (char)106, (char)133, (char)144, (char)132, (char)75, (char)93, (char)209, (char)104, (char)11, (char)88, (char)45, (char)152, (char)121, (char)242, (char)162, (char)183, (char)113, (char)237}, 0) ;
        p256.initial_timestamp_SET(6185802749381844977L) ;
        p256.target_system_SET((char)204) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.last_change_ms_GET() == 1793475408L);
            assert(pack.state_GET() == (char)15);
            assert(pack.time_boot_ms_GET() == 2988438560L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.last_change_ms_SET(1793475408L) ;
        p257.time_boot_ms_SET(2988438560L) ;
        p257.state_SET((char)15) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)166);
            assert(pack.tune_LEN(ph) == 28);
            assert(pack.tune_TRY(ph).equals("fxrkpmbhruxWjaOqypwUjslpzkoY"));
            assert(pack.target_component_GET() == (char)83);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.tune_SET("fxrkpmbhruxWjaOqypwUjslpzkoY", PH) ;
        p258.target_system_SET((char)166) ;
        p258.target_component_SET((char)83) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.firmware_version_GET() == 4002424047L);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)20, (char)92, (char)210, (char)3, (char)210, (char)167, (char)205, (char)185, (char)136, (char)94, (char)218, (char)0, (char)65, (char)27, (char)212, (char)231, (char)34, (char)128, (char)6, (char)56, (char)23, (char)174, (char)82, (char)127, (char)188, (char)185, (char)253, (char)183, (char)124, (char)120, (char)2, (char)106}));
            assert(pack.sensor_size_v_GET() == -6.693818E37F);
            assert(pack.lens_id_GET() == (char)137);
            assert(pack.cam_definition_version_GET() == (char)21691);
            assert(pack.sensor_size_h_GET() == -2.638629E38F);
            assert(pack.time_boot_ms_GET() == 1391528627L);
            assert(pack.resolution_v_GET() == (char)58856);
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO));
            assert(pack.resolution_h_GET() == (char)54247);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)91, (char)211, (char)16, (char)9, (char)7, (char)226, (char)66, (char)168, (char)242, (char)223, (char)199, (char)115, (char)204, (char)169, (char)181, (char)75, (char)192, (char)231, (char)53, (char)122, (char)234, (char)34, (char)135, (char)211, (char)204, (char)141, (char)70, (char)209, (char)73, (char)61, (char)146, (char)6}));
            assert(pack.focal_length_GET() == 1.2892421E38F);
            assert(pack.cam_definition_uri_LEN(ph) == 121);
            assert(pack.cam_definition_uri_TRY(ph).equals("yobdoyrcyvfpthawildtoNnsioaJftlWcgtbbyfVjqwiyazzjniiymisjYildfjzsybmyuEolaYpazerkmjosYegxEucaeMzcabwzhCppgbRmovvfmtdrhgUO"));
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.firmware_version_SET(4002424047L) ;
        p259.vendor_name_SET(new char[] {(char)91, (char)211, (char)16, (char)9, (char)7, (char)226, (char)66, (char)168, (char)242, (char)223, (char)199, (char)115, (char)204, (char)169, (char)181, (char)75, (char)192, (char)231, (char)53, (char)122, (char)234, (char)34, (char)135, (char)211, (char)204, (char)141, (char)70, (char)209, (char)73, (char)61, (char)146, (char)6}, 0) ;
        p259.resolution_h_SET((char)54247) ;
        p259.sensor_size_v_SET(-6.693818E37F) ;
        p259.sensor_size_h_SET(-2.638629E38F) ;
        p259.focal_length_SET(1.2892421E38F) ;
        p259.cam_definition_uri_SET("yobdoyrcyvfpthawildtoNnsioaJftlWcgtbbyfVjqwiyazzjniiymisjYildfjzsybmyuEolaYpazerkmjosYegxEucaeMzcabwzhCppgbRmovvfmtdrhgUO", PH) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO)) ;
        p259.time_boot_ms_SET(1391528627L) ;
        p259.model_name_SET(new char[] {(char)20, (char)92, (char)210, (char)3, (char)210, (char)167, (char)205, (char)185, (char)136, (char)94, (char)218, (char)0, (char)65, (char)27, (char)212, (char)231, (char)34, (char)128, (char)6, (char)56, (char)23, (char)174, (char)82, (char)127, (char)188, (char)185, (char)253, (char)183, (char)124, (char)120, (char)2, (char)106}, 0) ;
        p259.cam_definition_version_SET((char)21691) ;
        p259.resolution_v_SET((char)58856) ;
        p259.lens_id_SET((char)137) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_VIDEO);
            assert(pack.time_boot_ms_GET() == 557161412L);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(557161412L) ;
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1888499927L);
            assert(pack.used_capacity_GET() == -7.970345E37F);
            assert(pack.total_capacity_GET() == -3.3244048E38F);
            assert(pack.status_GET() == (char)30);
            assert(pack.storage_id_GET() == (char)64);
            assert(pack.write_speed_GET() == -1.6011958E38F);
            assert(pack.available_capacity_GET() == 1.8954745E38F);
            assert(pack.storage_count_GET() == (char)70);
            assert(pack.read_speed_GET() == 1.7749295E38F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.available_capacity_SET(1.8954745E38F) ;
        p261.storage_count_SET((char)70) ;
        p261.used_capacity_SET(-7.970345E37F) ;
        p261.write_speed_SET(-1.6011958E38F) ;
        p261.storage_id_SET((char)64) ;
        p261.status_SET((char)30) ;
        p261.total_capacity_SET(-3.3244048E38F) ;
        p261.time_boot_ms_SET(1888499927L) ;
        p261.read_speed_SET(1.7749295E38F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.image_status_GET() == (char)203);
            assert(pack.video_status_GET() == (char)84);
            assert(pack.image_interval_GET() == -2.5498451E38F);
            assert(pack.available_capacity_GET() == -1.8546414E38F);
            assert(pack.recording_time_ms_GET() == 2312673626L);
            assert(pack.time_boot_ms_GET() == 356294280L);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.image_interval_SET(-2.5498451E38F) ;
        p262.available_capacity_SET(-1.8546414E38F) ;
        p262.image_status_SET((char)203) ;
        p262.time_boot_ms_SET(356294280L) ;
        p262.video_status_SET((char)84) ;
        p262.recording_time_ms_SET(2312673626L) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.image_index_GET() == -945028116);
            assert(Arrays.equals(pack.q_GET(),  new float[] {7.3246114E37F, -1.6876579E38F, 2.4508943E38F, 9.612493E37F}));
            assert(pack.file_url_LEN(ph) == 111);
            assert(pack.file_url_TRY(ph).equals("ekvdeaVzxgvdfOyjnvvzvflxaxpuumyqtiQljesfibolzblcnSqmocsusskrdrmnrvblyqklyfjecarNutlkpptgkoNypEnvwrUotxzillyxvQe"));
            assert(pack.alt_GET() == -56974184);
            assert(pack.camera_id_GET() == (char)92);
            assert(pack.lat_GET() == -496373895);
            assert(pack.time_boot_ms_GET() == 1147615052L);
            assert(pack.relative_alt_GET() == 2147178565);
            assert(pack.time_utc_GET() == 5188202445343826568L);
            assert(pack.capture_result_GET() == (byte) - 88);
            assert(pack.lon_GET() == -2121698173);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.lon_SET(-2121698173) ;
        p263.image_index_SET(-945028116) ;
        p263.alt_SET(-56974184) ;
        p263.camera_id_SET((char)92) ;
        p263.relative_alt_SET(2147178565) ;
        p263.time_boot_ms_SET(1147615052L) ;
        p263.q_SET(new float[] {7.3246114E37F, -1.6876579E38F, 2.4508943E38F, 9.612493E37F}, 0) ;
        p263.lat_SET(-496373895) ;
        p263.file_url_SET("ekvdeaVzxgvdfOyjnvvzvflxaxpuumyqtiQljesfibolzblcnSqmocsusskrdrmnrvblyqklyfjecarNutlkpptgkoNypEnvwrUotxzillyxvQe", PH) ;
        p263.capture_result_SET((byte) - 88) ;
        p263.time_utc_SET(5188202445343826568L) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.flight_uuid_GET() == 8233117350846925562L);
            assert(pack.takeoff_time_utc_GET() == 2766199005373223038L);
            assert(pack.time_boot_ms_GET() == 2281667189L);
            assert(pack.arming_time_utc_GET() == 236312862565608804L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.arming_time_utc_SET(236312862565608804L) ;
        p264.time_boot_ms_SET(2281667189L) ;
        p264.takeoff_time_utc_SET(2766199005373223038L) ;
        p264.flight_uuid_SET(8233117350846925562L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 85727672L);
            assert(pack.pitch_GET() == -1.8541804E38F);
            assert(pack.yaw_GET() == 3.060941E38F);
            assert(pack.roll_GET() == 1.7529884E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.yaw_SET(3.060941E38F) ;
        p265.time_boot_ms_SET(85727672L) ;
        p265.roll_SET(1.7529884E38F) ;
        p265.pitch_SET(-1.8541804E38F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)163);
            assert(pack.target_system_GET() == (char)143);
            assert(pack.sequence_GET() == (char)53627);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)79, (char)180, (char)48, (char)169, (char)191, (char)215, (char)172, (char)129, (char)199, (char)95, (char)18, (char)170, (char)233, (char)118, (char)1, (char)178, (char)44, (char)34, (char)242, (char)134, (char)215, (char)253, (char)116, (char)5, (char)197, (char)174, (char)170, (char)140, (char)31, (char)87, (char)230, (char)1, (char)27, (char)18, (char)68, (char)228, (char)48, (char)16, (char)248, (char)10, (char)69, (char)141, (char)235, (char)152, (char)72, (char)99, (char)54, (char)226, (char)19, (char)18, (char)44, (char)233, (char)192, (char)217, (char)205, (char)232, (char)178, (char)126, (char)48, (char)218, (char)166, (char)96, (char)63, (char)19, (char)87, (char)9, (char)103, (char)11, (char)235, (char)142, (char)236, (char)101, (char)143, (char)100, (char)3, (char)53, (char)60, (char)25, (char)139, (char)196, (char)177, (char)112, (char)32, (char)188, (char)40, (char)84, (char)206, (char)46, (char)22, (char)18, (char)44, (char)12, (char)86, (char)141, (char)110, (char)74, (char)194, (char)121, (char)140, (char)122, (char)133, (char)39, (char)240, (char)153, (char)241, (char)19, (char)176, (char)166, (char)93, (char)34, (char)163, (char)226, (char)33, (char)166, (char)22, (char)34, (char)66, (char)81, (char)185, (char)157, (char)204, (char)117, (char)246, (char)203, (char)107, (char)20, (char)102, (char)185, (char)22, (char)144, (char)149, (char)64, (char)147, (char)5, (char)117, (char)216, (char)229, (char)229, (char)44, (char)224, (char)118, (char)149, (char)42, (char)237, (char)62, (char)144, (char)80, (char)77, (char)202, (char)220, (char)220, (char)234, (char)2, (char)151, (char)24, (char)1, (char)68, (char)201, (char)119, (char)224, (char)112, (char)202, (char)26, (char)88, (char)255, (char)150, (char)58, (char)240, (char)142, (char)246, (char)43, (char)110, (char)209, (char)206, (char)255, (char)243, (char)147, (char)181, (char)48, (char)39, (char)210, (char)198, (char)215, (char)222, (char)161, (char)14, (char)211, (char)1, (char)222, (char)109, (char)71, (char)196, (char)255, (char)141, (char)120, (char)55, (char)214, (char)101, (char)111, (char)237, (char)175, (char)75, (char)38, (char)107, (char)89, (char)119, (char)14, (char)206, (char)65, (char)183, (char)131, (char)132, (char)205, (char)191, (char)81, (char)195, (char)213, (char)114, (char)108, (char)187, (char)196, (char)64, (char)241, (char)224, (char)131, (char)157, (char)21, (char)194, (char)25, (char)124, (char)67, (char)47, (char)245, (char)91, (char)114, (char)173, (char)141, (char)126, (char)125, (char)51, (char)177, (char)230, (char)241, (char)14, (char)68, (char)196, (char)133, (char)254, (char)211}));
            assert(pack.length_GET() == (char)192);
            assert(pack.first_message_offset_GET() == (char)128);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)143) ;
        p266.first_message_offset_SET((char)128) ;
        p266.length_SET((char)192) ;
        p266.data__SET(new char[] {(char)79, (char)180, (char)48, (char)169, (char)191, (char)215, (char)172, (char)129, (char)199, (char)95, (char)18, (char)170, (char)233, (char)118, (char)1, (char)178, (char)44, (char)34, (char)242, (char)134, (char)215, (char)253, (char)116, (char)5, (char)197, (char)174, (char)170, (char)140, (char)31, (char)87, (char)230, (char)1, (char)27, (char)18, (char)68, (char)228, (char)48, (char)16, (char)248, (char)10, (char)69, (char)141, (char)235, (char)152, (char)72, (char)99, (char)54, (char)226, (char)19, (char)18, (char)44, (char)233, (char)192, (char)217, (char)205, (char)232, (char)178, (char)126, (char)48, (char)218, (char)166, (char)96, (char)63, (char)19, (char)87, (char)9, (char)103, (char)11, (char)235, (char)142, (char)236, (char)101, (char)143, (char)100, (char)3, (char)53, (char)60, (char)25, (char)139, (char)196, (char)177, (char)112, (char)32, (char)188, (char)40, (char)84, (char)206, (char)46, (char)22, (char)18, (char)44, (char)12, (char)86, (char)141, (char)110, (char)74, (char)194, (char)121, (char)140, (char)122, (char)133, (char)39, (char)240, (char)153, (char)241, (char)19, (char)176, (char)166, (char)93, (char)34, (char)163, (char)226, (char)33, (char)166, (char)22, (char)34, (char)66, (char)81, (char)185, (char)157, (char)204, (char)117, (char)246, (char)203, (char)107, (char)20, (char)102, (char)185, (char)22, (char)144, (char)149, (char)64, (char)147, (char)5, (char)117, (char)216, (char)229, (char)229, (char)44, (char)224, (char)118, (char)149, (char)42, (char)237, (char)62, (char)144, (char)80, (char)77, (char)202, (char)220, (char)220, (char)234, (char)2, (char)151, (char)24, (char)1, (char)68, (char)201, (char)119, (char)224, (char)112, (char)202, (char)26, (char)88, (char)255, (char)150, (char)58, (char)240, (char)142, (char)246, (char)43, (char)110, (char)209, (char)206, (char)255, (char)243, (char)147, (char)181, (char)48, (char)39, (char)210, (char)198, (char)215, (char)222, (char)161, (char)14, (char)211, (char)1, (char)222, (char)109, (char)71, (char)196, (char)255, (char)141, (char)120, (char)55, (char)214, (char)101, (char)111, (char)237, (char)175, (char)75, (char)38, (char)107, (char)89, (char)119, (char)14, (char)206, (char)65, (char)183, (char)131, (char)132, (char)205, (char)191, (char)81, (char)195, (char)213, (char)114, (char)108, (char)187, (char)196, (char)64, (char)241, (char)224, (char)131, (char)157, (char)21, (char)194, (char)25, (char)124, (char)67, (char)47, (char)245, (char)91, (char)114, (char)173, (char)141, (char)126, (char)125, (char)51, (char)177, (char)230, (char)241, (char)14, (char)68, (char)196, (char)133, (char)254, (char)211}, 0) ;
        p266.sequence_SET((char)53627) ;
        p266.target_component_SET((char)163) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)23392);
            assert(pack.first_message_offset_GET() == (char)0);
            assert(pack.target_component_GET() == (char)184);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)120, (char)163, (char)251, (char)145, (char)8, (char)111, (char)212, (char)109, (char)106, (char)187, (char)53, (char)76, (char)214, (char)181, (char)37, (char)255, (char)218, (char)232, (char)203, (char)243, (char)214, (char)165, (char)76, (char)197, (char)186, (char)25, (char)64, (char)160, (char)66, (char)87, (char)63, (char)153, (char)138, (char)182, (char)83, (char)191, (char)32, (char)216, (char)90, (char)110, (char)177, (char)130, (char)213, (char)40, (char)10, (char)103, (char)152, (char)234, (char)158, (char)47, (char)156, (char)69, (char)142, (char)204, (char)211, (char)153, (char)213, (char)21, (char)36, (char)65, (char)223, (char)113, (char)105, (char)57, (char)197, (char)200, (char)29, (char)159, (char)100, (char)176, (char)16, (char)211, (char)114, (char)119, (char)172, (char)215, (char)25, (char)90, (char)91, (char)170, (char)36, (char)122, (char)221, (char)33, (char)170, (char)121, (char)78, (char)39, (char)197, (char)21, (char)221, (char)134, (char)102, (char)245, (char)88, (char)179, (char)20, (char)213, (char)176, (char)239, (char)154, (char)16, (char)152, (char)211, (char)208, (char)1, (char)107, (char)167, (char)187, (char)143, (char)196, (char)10, (char)211, (char)177, (char)116, (char)54, (char)167, (char)158, (char)116, (char)226, (char)128, (char)121, (char)73, (char)3, (char)88, (char)38, (char)131, (char)212, (char)56, (char)227, (char)177, (char)246, (char)201, (char)67, (char)89, (char)82, (char)74, (char)40, (char)111, (char)142, (char)55, (char)35, (char)106, (char)154, (char)93, (char)89, (char)118, (char)56, (char)75, (char)80, (char)118, (char)18, (char)52, (char)111, (char)133, (char)216, (char)205, (char)10, (char)31, (char)20, (char)140, (char)59, (char)187, (char)56, (char)26, (char)61, (char)203, (char)171, (char)102, (char)1, (char)70, (char)18, (char)3, (char)146, (char)217, (char)39, (char)116, (char)12, (char)0, (char)85, (char)209, (char)24, (char)104, (char)9, (char)155, (char)148, (char)244, (char)136, (char)98, (char)235, (char)213, (char)127, (char)169, (char)112, (char)225, (char)129, (char)85, (char)195, (char)215, (char)2, (char)253, (char)220, (char)211, (char)230, (char)94, (char)156, (char)164, (char)180, (char)71, (char)152, (char)212, (char)7, (char)186, (char)91, (char)150, (char)254, (char)152, (char)97, (char)20, (char)30, (char)42, (char)246, (char)34, (char)198, (char)219, (char)47, (char)67, (char)28, (char)177, (char)174, (char)103, (char)176, (char)227, (char)107, (char)35, (char)233, (char)139, (char)37, (char)227, (char)252, (char)22, (char)158, (char)135, (char)99, (char)97, (char)159, (char)73, (char)233, (char)133}));
            assert(pack.length_GET() == (char)160);
            assert(pack.target_system_GET() == (char)112);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.data__SET(new char[] {(char)120, (char)163, (char)251, (char)145, (char)8, (char)111, (char)212, (char)109, (char)106, (char)187, (char)53, (char)76, (char)214, (char)181, (char)37, (char)255, (char)218, (char)232, (char)203, (char)243, (char)214, (char)165, (char)76, (char)197, (char)186, (char)25, (char)64, (char)160, (char)66, (char)87, (char)63, (char)153, (char)138, (char)182, (char)83, (char)191, (char)32, (char)216, (char)90, (char)110, (char)177, (char)130, (char)213, (char)40, (char)10, (char)103, (char)152, (char)234, (char)158, (char)47, (char)156, (char)69, (char)142, (char)204, (char)211, (char)153, (char)213, (char)21, (char)36, (char)65, (char)223, (char)113, (char)105, (char)57, (char)197, (char)200, (char)29, (char)159, (char)100, (char)176, (char)16, (char)211, (char)114, (char)119, (char)172, (char)215, (char)25, (char)90, (char)91, (char)170, (char)36, (char)122, (char)221, (char)33, (char)170, (char)121, (char)78, (char)39, (char)197, (char)21, (char)221, (char)134, (char)102, (char)245, (char)88, (char)179, (char)20, (char)213, (char)176, (char)239, (char)154, (char)16, (char)152, (char)211, (char)208, (char)1, (char)107, (char)167, (char)187, (char)143, (char)196, (char)10, (char)211, (char)177, (char)116, (char)54, (char)167, (char)158, (char)116, (char)226, (char)128, (char)121, (char)73, (char)3, (char)88, (char)38, (char)131, (char)212, (char)56, (char)227, (char)177, (char)246, (char)201, (char)67, (char)89, (char)82, (char)74, (char)40, (char)111, (char)142, (char)55, (char)35, (char)106, (char)154, (char)93, (char)89, (char)118, (char)56, (char)75, (char)80, (char)118, (char)18, (char)52, (char)111, (char)133, (char)216, (char)205, (char)10, (char)31, (char)20, (char)140, (char)59, (char)187, (char)56, (char)26, (char)61, (char)203, (char)171, (char)102, (char)1, (char)70, (char)18, (char)3, (char)146, (char)217, (char)39, (char)116, (char)12, (char)0, (char)85, (char)209, (char)24, (char)104, (char)9, (char)155, (char)148, (char)244, (char)136, (char)98, (char)235, (char)213, (char)127, (char)169, (char)112, (char)225, (char)129, (char)85, (char)195, (char)215, (char)2, (char)253, (char)220, (char)211, (char)230, (char)94, (char)156, (char)164, (char)180, (char)71, (char)152, (char)212, (char)7, (char)186, (char)91, (char)150, (char)254, (char)152, (char)97, (char)20, (char)30, (char)42, (char)246, (char)34, (char)198, (char)219, (char)47, (char)67, (char)28, (char)177, (char)174, (char)103, (char)176, (char)227, (char)107, (char)35, (char)233, (char)139, (char)37, (char)227, (char)252, (char)22, (char)158, (char)135, (char)99, (char)97, (char)159, (char)73, (char)233, (char)133}, 0) ;
        p267.first_message_offset_SET((char)0) ;
        p267.target_component_SET((char)184) ;
        p267.length_SET((char)160) ;
        p267.sequence_SET((char)23392) ;
        p267.target_system_SET((char)112) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)254);
            assert(pack.target_component_GET() == (char)211);
            assert(pack.sequence_GET() == (char)40396);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.sequence_SET((char)40396) ;
        p268.target_system_SET((char)254) ;
        p268.target_component_SET((char)211) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.uri_LEN(ph) == 50);
            assert(pack.uri_TRY(ph).equals("qmnpwuwcxgdOiaoqzdbntfqfelnvtwfygetlvvbebkqdupnmcg"));
            assert(pack.resolution_h_GET() == (char)49616);
            assert(pack.camera_id_GET() == (char)157);
            assert(pack.framerate_GET() == 1.0448182E38F);
            assert(pack.bitrate_GET() == 2317459280L);
            assert(pack.resolution_v_GET() == (char)14683);
            assert(pack.status_GET() == (char)218);
            assert(pack.rotation_GET() == (char)5801);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.uri_SET("qmnpwuwcxgdOiaoqzdbntfqfelnvtwfygetlvvbebkqdupnmcg", PH) ;
        p269.resolution_h_SET((char)49616) ;
        p269.status_SET((char)218) ;
        p269.camera_id_SET((char)157) ;
        p269.framerate_SET(1.0448182E38F) ;
        p269.bitrate_SET(2317459280L) ;
        p269.resolution_v_SET((char)14683) ;
        p269.rotation_SET((char)5801) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.rotation_GET() == (char)23616);
            assert(pack.target_system_GET() == (char)118);
            assert(pack.target_component_GET() == (char)196);
            assert(pack.resolution_h_GET() == (char)44139);
            assert(pack.bitrate_GET() == 4246344354L);
            assert(pack.uri_LEN(ph) == 221);
            assert(pack.uri_TRY(ph).equals("wwftcpwnmtunvhnjqmgnftvzsoGopazRzzkcEqzgwSnmdajxauPhswzrgzmayxmcBTIeywcymizmgmMynpjkfxvfzPvWoCpqhtluqrqhnllibmfNhwsswFplvJgsfmsvzhyadsapIoaruzzwbcvfowvzdGnqjsrkzcdkwkoiyfddTmbUHjvysylrdiegscrrgdrunikkhqiwsfaVdroqyymeUqxjk"));
            assert(pack.framerate_GET() == 2.4742714E38F);
            assert(pack.resolution_v_GET() == (char)53019);
            assert(pack.camera_id_GET() == (char)226);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.bitrate_SET(4246344354L) ;
        p270.uri_SET("wwftcpwnmtunvhnjqmgnftvzsoGopazRzzkcEqzgwSnmdajxauPhswzrgzmayxmcBTIeywcymizmgmMynpjkfxvfzPvWoCpqhtluqrqhnllibmfNhwsswFplvJgsfmsvzhyadsapIoaruzzwbcvfowvzdGnqjsrkzcdkwkoiyfddTmbUHjvysylrdiegscrrgdrunikkhqiwsfaVdroqyymeUqxjk", PH) ;
        p270.camera_id_SET((char)226) ;
        p270.rotation_SET((char)23616) ;
        p270.resolution_v_SET((char)53019) ;
        p270.resolution_h_SET((char)44139) ;
        p270.target_system_SET((char)118) ;
        p270.framerate_SET(2.4742714E38F) ;
        p270.target_component_SET((char)196) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 28);
            assert(pack.ssid_TRY(ph).equals("ixWhfhrizylyrbdxzgdbmlacdsjf"));
            assert(pack.password_LEN(ph) == 33);
            assert(pack.password_TRY(ph).equals("txjgicedsrpcbbxvoEnObcktfzyfpxqlR"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("txjgicedsrpcbbxvoEnObcktfzyfpxqlR", PH) ;
        p299.ssid_SET("ixWhfhrizylyrbdxzgdbmlacdsjf", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.max_version_GET() == (char)50745);
            assert(pack.min_version_GET() == (char)56225);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)152, (char)243, (char)104, (char)161, (char)105, (char)43, (char)48, (char)173}));
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)6, (char)167, (char)208, (char)67, (char)76, (char)27, (char)148, (char)53}));
            assert(pack.version_GET() == (char)9536);
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.max_version_SET((char)50745) ;
        p300.spec_version_hash_SET(new char[] {(char)6, (char)167, (char)208, (char)67, (char)76, (char)27, (char)148, (char)53}, 0) ;
        p300.min_version_SET((char)56225) ;
        p300.library_version_hash_SET(new char[] {(char)152, (char)243, (char)104, (char)161, (char)105, (char)43, (char)48, (char)173}, 0) ;
        p300.version_SET((char)9536) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
            assert(pack.time_usec_GET() == 3963904532443089738L);
            assert(pack.vendor_specific_status_code_GET() == (char)17530);
            assert(pack.sub_mode_GET() == (char)103);
            assert(pack.uptime_sec_GET() == 1954247525L);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.uptime_sec_SET(1954247525L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE) ;
        p310.sub_mode_SET((char)103) ;
        p310.time_usec_SET(3963904532443089738L) ;
        p310.vendor_specific_status_code_SET((char)17530) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.uptime_sec_GET() == 1314719173L);
            assert(pack.hw_version_minor_GET() == (char)243);
            assert(pack.name_LEN(ph) == 59);
            assert(pack.name_TRY(ph).equals("vjldconzpesQdqmgLmfbfaumszndzgbyrbnaesqivbyzcahXuzvSfscbrug"));
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)29, (char)146, (char)202, (char)218, (char)69, (char)196, (char)16, (char)29, (char)131, (char)43, (char)103, (char)192, (char)8, (char)207, (char)230, (char)227}));
            assert(pack.sw_version_major_GET() == (char)82);
            assert(pack.time_usec_GET() == 8741655024081317697L);
            assert(pack.hw_version_major_GET() == (char)107);
            assert(pack.sw_version_minor_GET() == (char)196);
            assert(pack.sw_vcs_commit_GET() == 1927528985L);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_unique_id_SET(new char[] {(char)29, (char)146, (char)202, (char)218, (char)69, (char)196, (char)16, (char)29, (char)131, (char)43, (char)103, (char)192, (char)8, (char)207, (char)230, (char)227}, 0) ;
        p311.sw_version_major_SET((char)82) ;
        p311.uptime_sec_SET(1314719173L) ;
        p311.hw_version_major_SET((char)107) ;
        p311.name_SET("vjldconzpesQdqmgLmfbfaumszndzgbyrbnaesqivbyzcahXuzvSfscbrug", PH) ;
        p311.hw_version_minor_SET((char)243) ;
        p311.sw_version_minor_SET((char)196) ;
        p311.sw_vcs_commit_SET(1927528985L) ;
        p311.time_usec_SET(8741655024081317697L) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)224);
            assert(pack.param_index_GET() == (short) -15976);
            assert(pack.target_component_GET() == (char)185);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("WfKesjck"));
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_component_SET((char)185) ;
        p320.param_id_SET("WfKesjck", PH) ;
        p320.target_system_SET((char)224) ;
        p320.param_index_SET((short) -15976) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)112);
            assert(pack.target_system_GET() == (char)75);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)75) ;
        p321.target_component_SET((char)112) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)16404);
            assert(pack.param_value_LEN(ph) == 8);
            assert(pack.param_value_TRY(ph).equals("mamepwth"));
            assert(pack.param_index_GET() == (char)41463);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("yndfjif"));
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("yndfjif", PH) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16) ;
        p322.param_value_SET("mamepwth", PH) ;
        p322.param_count_SET((char)16404) ;
        p322.param_index_SET((char)41463) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)251);
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("KtFcjtRypuKxdv"));
            assert(pack.target_component_GET() == (char)90);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
            assert(pack.param_value_LEN(ph) == 43);
            assert(pack.param_value_TRY(ph).equals("yqajxrzdriqxbbAkdoadWgyfaecBivrofszpkavyecB"));
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32) ;
        p323.param_id_SET("KtFcjtRypuKxdv", PH) ;
        p323.target_system_SET((char)251) ;
        p323.param_value_SET("yqajxrzdriqxbbAkdoadWgyfaecBivrofszpkavyecB", PH) ;
        p323.target_component_SET((char)90) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
            assert(pack.param_id_LEN(ph) == 13);
            assert(pack.param_id_TRY(ph).equals("sktqiqydylniI"));
            assert(pack.param_value_LEN(ph) == 2);
            assert(pack.param_value_TRY(ph).equals("uc"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED) ;
        p324.param_value_SET("uc", PH) ;
        p324.param_id_SET("sktqiqydylniI", PH) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.min_distance_GET() == (char)36570);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.max_distance_GET() == (char)5210);
            assert(pack.increment_GET() == (char)182);
            assert(pack.time_usec_GET() == 8395266130025589746L);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)53564, (char)35137, (char)8754, (char)350, (char)18188, (char)42364, (char)3210, (char)18656, (char)56604, (char)46969, (char)61863, (char)57120, (char)26575, (char)22270, (char)56853, (char)8042, (char)37893, (char)41396, (char)38896, (char)49784, (char)57936, (char)56075, (char)26041, (char)4830, (char)42801, (char)24037, (char)13506, (char)61265, (char)55920, (char)44891, (char)50362, (char)23356, (char)4660, (char)43031, (char)1114, (char)6941, (char)23822, (char)30957, (char)4337, (char)38475, (char)44842, (char)3454, (char)20861, (char)53033, (char)49627, (char)55901, (char)13743, (char)35647, (char)36570, (char)1706, (char)53710, (char)27270, (char)62009, (char)2788, (char)62619, (char)62057, (char)47523, (char)9830, (char)36167, (char)46275, (char)37741, (char)1548, (char)6574, (char)36301, (char)16194, (char)1493, (char)63020, (char)50578, (char)29789, (char)33939, (char)32962, (char)55802}));
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.increment_SET((char)182) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p330.distances_SET(new char[] {(char)53564, (char)35137, (char)8754, (char)350, (char)18188, (char)42364, (char)3210, (char)18656, (char)56604, (char)46969, (char)61863, (char)57120, (char)26575, (char)22270, (char)56853, (char)8042, (char)37893, (char)41396, (char)38896, (char)49784, (char)57936, (char)56075, (char)26041, (char)4830, (char)42801, (char)24037, (char)13506, (char)61265, (char)55920, (char)44891, (char)50362, (char)23356, (char)4660, (char)43031, (char)1114, (char)6941, (char)23822, (char)30957, (char)4337, (char)38475, (char)44842, (char)3454, (char)20861, (char)53033, (char)49627, (char)55901, (char)13743, (char)35647, (char)36570, (char)1706, (char)53710, (char)27270, (char)62009, (char)2788, (char)62619, (char)62057, (char)47523, (char)9830, (char)36167, (char)46275, (char)37741, (char)1548, (char)6574, (char)36301, (char)16194, (char)1493, (char)63020, (char)50578, (char)29789, (char)33939, (char)32962, (char)55802}, 0) ;
        p330.min_distance_SET((char)36570) ;
        p330.max_distance_SET((char)5210) ;
        p330.time_usec_SET(8395266130025589746L) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
    }

}
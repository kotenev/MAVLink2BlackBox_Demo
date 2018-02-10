
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
            long id = id__F(src);
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
            long id = id__X(src);
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
            long id = id__X(src);
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
            long id = id__X(src);
            set_bits(id, 7, data, 260);
        }
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
    public static class SCRIPT_ITEM extends GroundControl.SCRIPT_ITEM
    {
        public char seq_GET()//Sequence
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public String name_TRY(Bounds.Inside ph)//The name of the mission script, NULL terminated.
        {
            if(ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) return null;
            return new String(name_GET(ph, new char[ph.items], 0));
        }
        public char[] name_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //The name of the mission script, NULL terminated.
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int name_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        }
    }
    public static class SCRIPT_REQUEST extends GroundControl.SCRIPT_REQUEST
    {
        public char seq_GET()//Sequence
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  3, 1)); }
    }
    public static class SCRIPT_REQUEST_LIST extends GroundControl.SCRIPT_REQUEST_LIST
    {
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
    }
    public static class SCRIPT_COUNT extends GroundControl.SCRIPT_COUNT
    {
        public char count_GET()//Number of script items in the sequence
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  3, 1)); }
    }
    public static class SCRIPT_CURRENT extends GroundControl.SCRIPT_CURRENT
    {
        public char seq_GET()//Active Sequence
        {  return (char)((char) get_bytes(data,  0, 2)); }
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

        static final Collection<OnReceive.Handler<RESOURCE_REQUEST, Channel>> on_RESOURCE_REQUEST = new OnReceive<>();
        static final Collection<OnReceive.Handler<SCALED_PRESSURE3, Channel>> on_SCALED_PRESSURE3 = new OnReceive<>();
        static final Collection<OnReceive.Handler<FOLLOW_TARGET, Channel>> on_FOLLOW_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<CONTROL_SYSTEM_STATE, Channel>> on_CONTROL_SYSTEM_STATE = new OnReceive<>();
        static final Collection<OnReceive.Handler<BATTERY_STATUS, Channel>> on_BATTERY_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<AUTOPILOT_VERSION, Channel>> on_AUTOPILOT_VERSION = new OnReceive<>();
        static final Collection<OnReceive.Handler<LANDING_TARGET, Channel>> on_LANDING_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<SCRIPT_ITEM, Channel>> on_SCRIPT_ITEM = new OnReceive<>();
        static final Collection<OnReceive.Handler<SCRIPT_REQUEST, Channel>> on_SCRIPT_REQUEST = new OnReceive<>();
        static final Collection<OnReceive.Handler<SCRIPT_REQUEST_LIST, Channel>> on_SCRIPT_REQUEST_LIST = new OnReceive<>();
        static final Collection<OnReceive.Handler<SCRIPT_COUNT, Channel>> on_SCRIPT_COUNT = new OnReceive<>();
        static final Collection<OnReceive.Handler<SCRIPT_CURRENT, Channel>> on_SCRIPT_CURRENT = new OnReceive<>();
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
                case 180:
                    if(pack == null) return new SCRIPT_ITEM();
                    ((OnReceive) on_SCRIPT_ITEM).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 181:
                    if(pack == null) return new SCRIPT_REQUEST();
                    ((OnReceive) on_SCRIPT_REQUEST).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 182:
                    if(pack == null) return new SCRIPT_REQUEST_LIST();
                    ((OnReceive) on_SCRIPT_REQUEST_LIST).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 183:
                    if(pack == null) return new SCRIPT_COUNT();
                    ((OnReceive) on_SCRIPT_COUNT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 184:
                    if(pack == null) return new SCRIPT_CURRENT();
                    ((OnReceive) on_SCRIPT_CURRENT).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
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
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED));
            assert(pack.custom_mode_GET() == 3107190397L);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_CRITICAL);
            assert(pack.mavlink_version_GET() == (char)43);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_VTOL_RESERVED3);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.type_SET(MAV_TYPE.MAV_TYPE_VTOL_RESERVED3) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED)) ;
        p0.custom_mode_SET(3107190397L) ;
        p0.mavlink_version_SET((char)43) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_CRITICAL) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.drop_rate_comm_GET() == (char)47212);
            assert(pack.errors_count1_GET() == (char)54044);
            assert(pack.errors_count4_GET() == (char)46088);
            assert(pack.errors_count2_GET() == (char)63957);
            assert(pack.load_GET() == (char)13315);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL));
            assert(pack.errors_comm_GET() == (char)30541);
            assert(pack.errors_count3_GET() == (char)41091);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS));
            assert(pack.voltage_battery_GET() == (char)13009);
            assert(pack.battery_remaining_GET() == (byte) - 105);
            assert(pack.current_battery_GET() == (short)28507);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.load_SET((char)13315) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL)) ;
        p1.voltage_battery_SET((char)13009) ;
        p1.errors_count3_SET((char)41091) ;
        p1.current_battery_SET((short)28507) ;
        p1.errors_count4_SET((char)46088) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW)) ;
        p1.errors_comm_SET((char)30541) ;
        p1.errors_count1_SET((char)54044) ;
        p1.battery_remaining_SET((byte) - 105) ;
        p1.errors_count2_SET((char)63957) ;
        p1.drop_rate_comm_SET((char)47212) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS)) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 3151535185726868401L);
            assert(pack.time_boot_ms_GET() == 1855725181L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(3151535185726868401L) ;
        p2.time_boot_ms_SET(1855725181L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2935622285L);
            assert(pack.afz_GET() == -2.440276E38F);
            assert(pack.afx_GET() == -8.463267E37F);
            assert(pack.afy_GET() == 1.1861774E38F);
            assert(pack.z_GET() == -1.4016263E38F);
            assert(pack.vy_GET() == -1.6710747E38F);
            assert(pack.yaw_GET() == 1.889637E38F);
            assert(pack.type_mask_GET() == (char)9902);
            assert(pack.vz_GET() == -2.9953146E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.y_GET() == -3.167221E38F);
            assert(pack.yaw_rate_GET() == 1.0701991E38F);
            assert(pack.vx_GET() == -1.4012108E38F);
            assert(pack.x_GET() == -6.465735E37F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.x_SET(-6.465735E37F) ;
        p3.afz_SET(-2.440276E38F) ;
        p3.vx_SET(-1.4012108E38F) ;
        p3.y_SET(-3.167221E38F) ;
        p3.z_SET(-1.4016263E38F) ;
        p3.yaw_rate_SET(1.0701991E38F) ;
        p3.afy_SET(1.1861774E38F) ;
        p3.type_mask_SET((char)9902) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p3.afx_SET(-8.463267E37F) ;
        p3.vy_SET(-1.6710747E38F) ;
        p3.yaw_SET(1.889637E38F) ;
        p3.vz_SET(-2.9953146E38F) ;
        p3.time_boot_ms_SET(2935622285L) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 8932107693125927315L);
            assert(pack.target_component_GET() == (char)37);
            assert(pack.seq_GET() == 417608054L);
            assert(pack.target_system_GET() == (char)216);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.seq_SET(417608054L) ;
        p4.target_component_SET((char)37) ;
        p4.target_system_SET((char)216) ;
        p4.time_usec_SET(8932107693125927315L) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)202);
            assert(pack.passkey_LEN(ph) == 10);
            assert(pack.passkey_TRY(ph).equals("ayymwfqGzn"));
            assert(pack.version_GET() == (char)86);
            assert(pack.target_system_GET() == (char)2);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.passkey_SET("ayymwfqGzn", PH) ;
        p5.control_request_SET((char)202) ;
        p5.target_system_SET((char)2) ;
        p5.version_SET((char)86) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)236);
            assert(pack.ack_GET() == (char)28);
            assert(pack.control_request_GET() == (char)200);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.ack_SET((char)28) ;
        p6.control_request_SET((char)200) ;
        p6.gcs_system_id_SET((char)236) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 7);
            assert(pack.key_TRY(ph).equals("cCtecvx"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("cCtecvx", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_GUIDED_ARMED);
            assert(pack.target_system_GET() == (char)3);
            assert(pack.custom_mode_GET() == 771484200L);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED) ;
        p11.custom_mode_SET(771484200L) ;
        p11.target_system_SET((char)3) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)197);
            assert(pack.target_component_GET() == (char)141);
            assert(pack.param_index_GET() == (short)12666);
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("nyvfshbyjdrqwvpX"));
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_component_SET((char)141) ;
        p20.param_id_SET("nyvfshbyjdrqwvpX", PH) ;
        p20.param_index_SET((short)12666) ;
        p20.target_system_SET((char)197) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)59);
            assert(pack.target_component_GET() == (char)43);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)43) ;
        p21.target_system_SET((char)59) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (char)18523);
            assert(pack.param_count_GET() == (char)1853);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
            assert(pack.param_value_GET() == 3.7314606E37F);
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("ponra"));
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_index_SET((char)18523) ;
        p22.param_id_SET("ponra", PH) ;
        p22.param_value_SET(3.7314606E37F) ;
        p22.param_count_SET((char)1853) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64);
            assert(pack.target_component_GET() == (char)215);
            assert(pack.param_value_GET() == -2.4565065E38F);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("YaygjtqvbLsm"));
            assert(pack.target_system_GET() == (char)2);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.target_component_SET((char)215) ;
        p23.param_id_SET("YaygjtqvbLsm", PH) ;
        p23.target_system_SET((char)2) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64) ;
        p23.param_value_SET(-2.4565065E38F) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1947582586);
            assert(pack.alt_ellipsoid_TRY(ph) == 1014502915);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
            assert(pack.eph_GET() == (char)8324);
            assert(pack.v_acc_TRY(ph) == 1700248293L);
            assert(pack.hdg_acc_TRY(ph) == 475217226L);
            assert(pack.vel_acc_TRY(ph) == 3139011636L);
            assert(pack.lat_GET() == 1439853790);
            assert(pack.satellites_visible_GET() == (char)126);
            assert(pack.time_usec_GET() == 3794049624361806678L);
            assert(pack.h_acc_TRY(ph) == 3808753593L);
            assert(pack.vel_GET() == (char)7567);
            assert(pack.epv_GET() == (char)55870);
            assert(pack.cog_GET() == (char)49678);
            assert(pack.alt_GET() == -1661580838);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.lat_SET(1439853790) ;
        p24.alt_ellipsoid_SET(1014502915, PH) ;
        p24.hdg_acc_SET(475217226L, PH) ;
        p24.eph_SET((char)8324) ;
        p24.time_usec_SET(3794049624361806678L) ;
        p24.satellites_visible_SET((char)126) ;
        p24.epv_SET((char)55870) ;
        p24.alt_SET(-1661580838) ;
        p24.h_acc_SET(3808753593L, PH) ;
        p24.vel_acc_SET(3139011636L, PH) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED) ;
        p24.vel_SET((char)7567) ;
        p24.cog_SET((char)49678) ;
        p24.lon_SET(-1947582586) ;
        p24.v_acc_SET(1700248293L, PH) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)196, (char)101, (char)89, (char)0, (char)80, (char)187, (char)207, (char)163, (char)229, (char)90, (char)6, (char)50, (char)188, (char)146, (char)204, (char)164, (char)44, (char)155, (char)77, (char)160}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)207, (char)163, (char)142, (char)198, (char)93, (char)85, (char)98, (char)160, (char)87, (char)198, (char)132, (char)89, (char)7, (char)185, (char)106, (char)173, (char)145, (char)245, (char)232, (char)126}));
            assert(pack.satellites_visible_GET() == (char)222);
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)198, (char)211, (char)181, (char)55, (char)159, (char)31, (char)112, (char)164, (char)12, (char)80, (char)163, (char)189, (char)71, (char)252, (char)81, (char)173, (char)3, (char)66, (char)157, (char)191}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)61, (char)126, (char)159, (char)179, (char)226, (char)199, (char)209, (char)18, (char)2, (char)86, (char)241, (char)156, (char)240, (char)227, (char)85, (char)243, (char)154, (char)238, (char)249, (char)127}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)67, (char)171, (char)65, (char)40, (char)111, (char)161, (char)69, (char)66, (char)183, (char)228, (char)3, (char)232, (char)170, (char)239, (char)189, (char)36, (char)145, (char)30, (char)198, (char)169}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_used_SET(new char[] {(char)207, (char)163, (char)142, (char)198, (char)93, (char)85, (char)98, (char)160, (char)87, (char)198, (char)132, (char)89, (char)7, (char)185, (char)106, (char)173, (char)145, (char)245, (char)232, (char)126}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)196, (char)101, (char)89, (char)0, (char)80, (char)187, (char)207, (char)163, (char)229, (char)90, (char)6, (char)50, (char)188, (char)146, (char)204, (char)164, (char)44, (char)155, (char)77, (char)160}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)61, (char)126, (char)159, (char)179, (char)226, (char)199, (char)209, (char)18, (char)2, (char)86, (char)241, (char)156, (char)240, (char)227, (char)85, (char)243, (char)154, (char)238, (char)249, (char)127}, 0) ;
        p25.satellites_visible_SET((char)222) ;
        p25.satellite_azimuth_SET(new char[] {(char)67, (char)171, (char)65, (char)40, (char)111, (char)161, (char)69, (char)66, (char)183, (char)228, (char)3, (char)232, (char)170, (char)239, (char)189, (char)36, (char)145, (char)30, (char)198, (char)169}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)198, (char)211, (char)181, (char)55, (char)159, (char)31, (char)112, (char)164, (char)12, (char)80, (char)163, (char)189, (char)71, (char)252, (char)81, (char)173, (char)3, (char)66, (char)157, (char)191}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == (short) -14411);
            assert(pack.zacc_GET() == (short) -2909);
            assert(pack.zgyro_GET() == (short)7335);
            assert(pack.xgyro_GET() == (short)5483);
            assert(pack.yacc_GET() == (short)31165);
            assert(pack.zmag_GET() == (short)19586);
            assert(pack.xacc_GET() == (short) -23319);
            assert(pack.time_boot_ms_GET() == 396566199L);
            assert(pack.xmag_GET() == (short)26309);
            assert(pack.ygyro_GET() == (short)24634);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.zmag_SET((short)19586) ;
        p26.xacc_SET((short) -23319) ;
        p26.zacc_SET((short) -2909) ;
        p26.xmag_SET((short)26309) ;
        p26.ymag_SET((short) -14411) ;
        p26.ygyro_SET((short)24634) ;
        p26.time_boot_ms_SET(396566199L) ;
        p26.zgyro_SET((short)7335) ;
        p26.yacc_SET((short)31165) ;
        p26.xgyro_SET((short)5483) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == (short)21343);
            assert(pack.xgyro_GET() == (short)23315);
            assert(pack.zacc_GET() == (short) -20149);
            assert(pack.ygyro_GET() == (short)8785);
            assert(pack.yacc_GET() == (short) -12243);
            assert(pack.ymag_GET() == (short) -24158);
            assert(pack.xacc_GET() == (short) -1840);
            assert(pack.time_usec_GET() == 3505623114699652728L);
            assert(pack.xmag_GET() == (short)9917);
            assert(pack.zgyro_GET() == (short) -18358);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.xacc_SET((short) -1840) ;
        p27.zgyro_SET((short) -18358) ;
        p27.ymag_SET((short) -24158) ;
        p27.ygyro_SET((short)8785) ;
        p27.zacc_SET((short) -20149) ;
        p27.yacc_SET((short) -12243) ;
        p27.zmag_SET((short)21343) ;
        p27.xmag_SET((short)9917) ;
        p27.xgyro_SET((short)23315) ;
        p27.time_usec_SET(3505623114699652728L) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == (short) -29964);
            assert(pack.time_usec_GET() == 8735357165278937282L);
            assert(pack.temperature_GET() == (short)11665);
            assert(pack.press_diff2_GET() == (short) -5565);
            assert(pack.press_diff1_GET() == (short)27620);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_abs_SET((short) -29964) ;
        p28.press_diff1_SET((short)27620) ;
        p28.temperature_SET((short)11665) ;
        p28.press_diff2_SET((short) -5565) ;
        p28.time_usec_SET(8735357165278937282L) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -29894);
            assert(pack.press_diff_GET() == 4.948969E37F);
            assert(pack.time_boot_ms_GET() == 4169552520L);
            assert(pack.press_abs_GET() == 8.85008E37F);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_abs_SET(8.85008E37F) ;
        p29.press_diff_SET(4.948969E37F) ;
        p29.time_boot_ms_SET(4169552520L) ;
        p29.temperature_SET((short) -29894) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3451084641L);
            assert(pack.pitchspeed_GET() == 2.6677324E38F);
            assert(pack.yaw_GET() == 2.1603417E38F);
            assert(pack.roll_GET() == 2.1890405E38F);
            assert(pack.pitch_GET() == 1.5881976E38F);
            assert(pack.rollspeed_GET() == 2.9256157E38F);
            assert(pack.yawspeed_GET() == 3.1100081E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.pitchspeed_SET(2.6677324E38F) ;
        p30.pitch_SET(1.5881976E38F) ;
        p30.yawspeed_SET(3.1100081E38F) ;
        p30.roll_SET(2.1890405E38F) ;
        p30.time_boot_ms_SET(3451084641L) ;
        p30.rollspeed_SET(2.9256157E38F) ;
        p30.yaw_SET(2.1603417E38F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1812387925L);
            assert(pack.q1_GET() == -2.1179476E38F);
            assert(pack.rollspeed_GET() == -3.3591642E38F);
            assert(pack.q4_GET() == 2.761711E38F);
            assert(pack.q3_GET() == 6.960719E37F);
            assert(pack.q2_GET() == 3.0293923E38F);
            assert(pack.yawspeed_GET() == -3.3892132E38F);
            assert(pack.pitchspeed_GET() == -3.2775755E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q1_SET(-2.1179476E38F) ;
        p31.rollspeed_SET(-3.3591642E38F) ;
        p31.time_boot_ms_SET(1812387925L) ;
        p31.pitchspeed_SET(-3.2775755E38F) ;
        p31.yawspeed_SET(-3.3892132E38F) ;
        p31.q3_SET(6.960719E37F) ;
        p31.q4_SET(2.761711E38F) ;
        p31.q2_SET(3.0293923E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == -3.1850678E36F);
            assert(pack.z_GET() == 2.8866647E38F);
            assert(pack.vy_GET() == 3.248205E37F);
            assert(pack.y_GET() == -2.3086621E38F);
            assert(pack.time_boot_ms_GET() == 3967749769L);
            assert(pack.x_GET() == -1.7139642E38F);
            assert(pack.vz_GET() == 5.2196913E37F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.time_boot_ms_SET(3967749769L) ;
        p32.vx_SET(-3.1850678E36F) ;
        p32.x_SET(-1.7139642E38F) ;
        p32.vz_SET(5.2196913E37F) ;
        p32.y_SET(-2.3086621E38F) ;
        p32.z_SET(2.8866647E38F) ;
        p32.vy_SET(3.248205E37F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == (short) -14158);
            assert(pack.lon_GET() == 310784849);
            assert(pack.vy_GET() == (short) -8313);
            assert(pack.vz_GET() == (short)16836);
            assert(pack.relative_alt_GET() == -672982254);
            assert(pack.alt_GET() == 710331690);
            assert(pack.hdg_GET() == (char)39567);
            assert(pack.time_boot_ms_GET() == 1052604816L);
            assert(pack.lat_GET() == 612526748);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.relative_alt_SET(-672982254) ;
        p33.time_boot_ms_SET(1052604816L) ;
        p33.vx_SET((short) -14158) ;
        p33.hdg_SET((char)39567) ;
        p33.vz_SET((short)16836) ;
        p33.lat_SET(612526748) ;
        p33.lon_SET(310784849) ;
        p33.vy_SET((short) -8313) ;
        p33.alt_SET(710331690) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan1_scaled_GET() == (short)29000);
            assert(pack.chan5_scaled_GET() == (short) -588);
            assert(pack.chan6_scaled_GET() == (short)12903);
            assert(pack.rssi_GET() == (char)157);
            assert(pack.time_boot_ms_GET() == 3457804516L);
            assert(pack.chan7_scaled_GET() == (short) -31802);
            assert(pack.chan2_scaled_GET() == (short) -26209);
            assert(pack.chan8_scaled_GET() == (short) -17401);
            assert(pack.chan3_scaled_GET() == (short) -25864);
            assert(pack.chan4_scaled_GET() == (short)26475);
            assert(pack.port_GET() == (char)220);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan8_scaled_SET((short) -17401) ;
        p34.rssi_SET((char)157) ;
        p34.chan3_scaled_SET((short) -25864) ;
        p34.chan1_scaled_SET((short)29000) ;
        p34.chan4_scaled_SET((short)26475) ;
        p34.chan2_scaled_SET((short) -26209) ;
        p34.port_SET((char)220) ;
        p34.chan5_scaled_SET((short) -588) ;
        p34.chan6_scaled_SET((short)12903) ;
        p34.chan7_scaled_SET((short) -31802) ;
        p34.time_boot_ms_SET(3457804516L) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan1_raw_GET() == (char)17128);
            assert(pack.time_boot_ms_GET() == 3969517058L);
            assert(pack.chan4_raw_GET() == (char)11071);
            assert(pack.port_GET() == (char)95);
            assert(pack.chan5_raw_GET() == (char)16786);
            assert(pack.chan8_raw_GET() == (char)39385);
            assert(pack.chan2_raw_GET() == (char)59122);
            assert(pack.rssi_GET() == (char)110);
            assert(pack.chan7_raw_GET() == (char)14485);
            assert(pack.chan6_raw_GET() == (char)12256);
            assert(pack.chan3_raw_GET() == (char)21310);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan2_raw_SET((char)59122) ;
        p35.rssi_SET((char)110) ;
        p35.chan8_raw_SET((char)39385) ;
        p35.chan4_raw_SET((char)11071) ;
        p35.chan7_raw_SET((char)14485) ;
        p35.chan3_raw_SET((char)21310) ;
        p35.chan1_raw_SET((char)17128) ;
        p35.time_boot_ms_SET(3969517058L) ;
        p35.chan5_raw_SET((char)16786) ;
        p35.chan6_raw_SET((char)12256) ;
        p35.port_SET((char)95) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.port_GET() == (char)165);
            assert(pack.servo13_raw_TRY(ph) == (char)29400);
            assert(pack.servo8_raw_GET() == (char)56836);
            assert(pack.servo15_raw_TRY(ph) == (char)15589);
            assert(pack.servo11_raw_TRY(ph) == (char)1060);
            assert(pack.servo10_raw_TRY(ph) == (char)29899);
            assert(pack.time_usec_GET() == 888875561L);
            assert(pack.servo9_raw_TRY(ph) == (char)51240);
            assert(pack.servo16_raw_TRY(ph) == (char)4005);
            assert(pack.servo5_raw_GET() == (char)1452);
            assert(pack.servo1_raw_GET() == (char)28060);
            assert(pack.servo4_raw_GET() == (char)15501);
            assert(pack.servo7_raw_GET() == (char)18544);
            assert(pack.servo3_raw_GET() == (char)25295);
            assert(pack.servo6_raw_GET() == (char)40190);
            assert(pack.servo12_raw_TRY(ph) == (char)10297);
            assert(pack.servo2_raw_GET() == (char)42170);
            assert(pack.servo14_raw_TRY(ph) == (char)20911);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.port_SET((char)165) ;
        p36.time_usec_SET(888875561L) ;
        p36.servo4_raw_SET((char)15501) ;
        p36.servo9_raw_SET((char)51240, PH) ;
        p36.servo8_raw_SET((char)56836) ;
        p36.servo14_raw_SET((char)20911, PH) ;
        p36.servo1_raw_SET((char)28060) ;
        p36.servo12_raw_SET((char)10297, PH) ;
        p36.servo3_raw_SET((char)25295) ;
        p36.servo16_raw_SET((char)4005, PH) ;
        p36.servo15_raw_SET((char)15589, PH) ;
        p36.servo2_raw_SET((char)42170) ;
        p36.servo6_raw_SET((char)40190) ;
        p36.servo11_raw_SET((char)1060, PH) ;
        p36.servo13_raw_SET((char)29400, PH) ;
        p36.servo5_raw_SET((char)1452) ;
        p36.servo10_raw_SET((char)29899, PH) ;
        p36.servo7_raw_SET((char)18544) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.end_index_GET() == (short)17374);
            assert(pack.target_component_GET() == (char)250);
            assert(pack.start_index_GET() == (short) -7062);
            assert(pack.target_system_GET() == (char)80);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_component_SET((char)250) ;
        p37.start_index_SET((short) -7062) ;
        p37.end_index_SET((short)17374) ;
        p37.target_system_SET((char)80) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short) -27963);
            assert(pack.start_index_GET() == (short)2547);
            assert(pack.target_system_GET() == (char)215);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)133);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_system_SET((char)215) ;
        p38.start_index_SET((short)2547) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p38.target_component_SET((char)133) ;
        p38.end_index_SET((short) -27963) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.param2_GET() == -3.8166919E37F);
            assert(pack.target_system_GET() == (char)40);
            assert(pack.current_GET() == (char)139);
            assert(pack.z_GET() == -4.0960783E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_SPATIAL_USER_2);
            assert(pack.param3_GET() == -4.158179E36F);
            assert(pack.x_GET() == 1.8736638E38F);
            assert(pack.autocontinue_GET() == (char)102);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.seq_GET() == (char)3125);
            assert(pack.target_component_GET() == (char)143);
            assert(pack.param1_GET() == -1.5658994E38F);
            assert(pack.y_GET() == -1.9879972E38F);
            assert(pack.param4_GET() == -4.5377105E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.x_SET(1.8736638E38F) ;
        p39.param2_SET(-3.8166919E37F) ;
        p39.z_SET(-4.0960783E37F) ;
        p39.seq_SET((char)3125) ;
        p39.command_SET(MAV_CMD.MAV_CMD_SPATIAL_USER_2) ;
        p39.target_system_SET((char)40) ;
        p39.target_component_SET((char)143) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p39.autocontinue_SET((char)102) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p39.current_SET((char)139) ;
        p39.param3_SET(-4.158179E36F) ;
        p39.param1_SET(-1.5658994E38F) ;
        p39.y_SET(-1.9879972E38F) ;
        p39.param4_SET(-4.5377105E37F) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)186);
            assert(pack.seq_GET() == (char)57606);
            assert(pack.target_component_GET() == (char)43);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.seq_SET((char)57606) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p40.target_system_SET((char)186) ;
        p40.target_component_SET((char)43) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)3);
            assert(pack.target_component_GET() == (char)58);
            assert(pack.seq_GET() == (char)64963);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.seq_SET((char)64963) ;
        p41.target_system_SET((char)3) ;
        p41.target_component_SET((char)58) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)1657);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)1657) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)140);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)210);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_component_SET((char)210) ;
        p43.target_system_SET((char)140) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.count_GET() == (char)47739);
            assert(pack.target_system_GET() == (char)81);
            assert(pack.target_component_GET() == (char)14);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)81) ;
        p44.target_component_SET((char)14) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p44.count_SET((char)47739) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)122);
            assert(pack.target_component_GET() == (char)254);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)122) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p45.target_component_SET((char)254) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)44721);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)44721) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)30);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_ERROR);
            assert(pack.target_component_GET() == (char)76);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_ERROR) ;
        p47.target_system_SET((char)30) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p47.target_component_SET((char)76) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)20);
            assert(pack.latitude_GET() == 1127001231);
            assert(pack.time_usec_TRY(ph) == 4728007214872316492L);
            assert(pack.longitude_GET() == -2025175438);
            assert(pack.altitude_GET() == -34249623);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.altitude_SET(-34249623) ;
        p48.time_usec_SET(4728007214872316492L, PH) ;
        p48.target_system_SET((char)20) ;
        p48.longitude_SET(-2025175438) ;
        p48.latitude_SET(1127001231) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 6229344662625154308L);
            assert(pack.altitude_GET() == 1235287926);
            assert(pack.latitude_GET() == 38506018);
            assert(pack.longitude_GET() == -154785435);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.longitude_SET(-154785435) ;
        p49.time_usec_SET(6229344662625154308L, PH) ;
        p49.latitude_SET(38506018) ;
        p49.altitude_SET(1235287926) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.parameter_rc_channel_index_GET() == (char)54);
            assert(pack.param_value0_GET() == 1.9509376E38F);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("Caocsuzg"));
            assert(pack.target_system_GET() == (char)47);
            assert(pack.param_value_max_GET() == 1.373786E38F);
            assert(pack.param_value_min_GET() == 2.6278154E37F);
            assert(pack.target_component_GET() == (char)150);
            assert(pack.param_index_GET() == (short)10532);
            assert(pack.scale_GET() == -1.8726735E38F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_system_SET((char)47) ;
        p50.param_index_SET((short)10532) ;
        p50.scale_SET(-1.8726735E38F) ;
        p50.param_value0_SET(1.9509376E38F) ;
        p50.param_value_max_SET(1.373786E38F) ;
        p50.param_id_SET("Caocsuzg", PH) ;
        p50.param_value_min_SET(2.6278154E37F) ;
        p50.parameter_rc_channel_index_SET((char)54) ;
        p50.target_component_SET((char)150) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.seq_GET() == (char)25984);
            assert(pack.target_component_GET() == (char)34);
            assert(pack.target_system_GET() == (char)33);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.seq_SET((char)25984) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p51.target_system_SET((char)33) ;
        p51.target_component_SET((char)34) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)54);
            assert(pack.p2x_GET() == -2.5775892E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.p1y_GET() == 9.412586E37F);
            assert(pack.p2z_GET() == 1.4286134E38F);
            assert(pack.p1z_GET() == 1.4041038E38F);
            assert(pack.p1x_GET() == 1.780466E36F);
            assert(pack.target_component_GET() == (char)169);
            assert(pack.p2y_GET() == 2.4973358E37F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.target_component_SET((char)169) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p54.p2z_SET(1.4286134E38F) ;
        p54.p2y_SET(2.4973358E37F) ;
        p54.p2x_SET(-2.5775892E38F) ;
        p54.p1y_SET(9.412586E37F) ;
        p54.target_system_SET((char)54) ;
        p54.p1x_SET(1.780466E36F) ;
        p54.p1z_SET(1.4041038E38F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2z_GET() == 1.5676821E38F);
            assert(pack.p2x_GET() == 1.1339279E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.p1z_GET() == -6.131187E37F);
            assert(pack.p1y_GET() == -1.4178933E38F);
            assert(pack.p2y_GET() == -1.7957897E38F);
            assert(pack.p1x_GET() == 3.0055034E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2z_SET(1.5676821E38F) ;
        p55.p1y_SET(-1.4178933E38F) ;
        p55.p2x_SET(1.1339279E38F) ;
        p55.p1x_SET(3.0055034E38F) ;
        p55.p1z_SET(-6.131187E37F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p55.p2y_SET(-1.7957897E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.119786E38F, -6.4997943E37F, -3.0698636E37F, 2.0302063E38F}));
            assert(pack.rollspeed_GET() == 2.0456832E38F);
            assert(pack.yawspeed_GET() == -1.3735256E38F);
            assert(pack.time_usec_GET() == 584394016978810713L);
            assert(pack.pitchspeed_GET() == -2.85447E36F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.804345E38F, 3.3384043E38F, 1.0668847E38F, 1.3085428E38F, 8.2905744E36F, 3.0458912E38F, 2.9594506E38F, -1.943527E38F, -1.4188249E38F}));
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.q_SET(new float[] {-3.119786E38F, -6.4997943E37F, -3.0698636E37F, 2.0302063E38F}, 0) ;
        p61.yawspeed_SET(-1.3735256E38F) ;
        p61.time_usec_SET(584394016978810713L) ;
        p61.rollspeed_SET(2.0456832E38F) ;
        p61.covariance_SET(new float[] {2.804345E38F, 3.3384043E38F, 1.0668847E38F, 1.3085428E38F, 8.2905744E36F, 3.0458912E38F, 2.9594506E38F, -1.943527E38F, -1.4188249E38F}, 0) ;
        p61.pitchspeed_SET(-2.85447E36F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.alt_error_GET() == -7.5048987E37F);
            assert(pack.nav_roll_GET() == 2.249005E38F);
            assert(pack.target_bearing_GET() == (short)20464);
            assert(pack.wp_dist_GET() == (char)52789);
            assert(pack.nav_pitch_GET() == 4.600361E37F);
            assert(pack.nav_bearing_GET() == (short) -12101);
            assert(pack.aspd_error_GET() == -1.849148E38F);
            assert(pack.xtrack_error_GET() == -1.762793E38F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.wp_dist_SET((char)52789) ;
        p62.nav_pitch_SET(4.600361E37F) ;
        p62.nav_bearing_SET((short) -12101) ;
        p62.aspd_error_SET(-1.849148E38F) ;
        p62.target_bearing_SET((short)20464) ;
        p62.nav_roll_SET(2.249005E38F) ;
        p62.xtrack_error_SET(-1.762793E38F) ;
        p62.alt_error_SET(-7.5048987E37F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.3044527E38F, 2.5721837E37F, -2.7451102E38F, 1.0493493E38F, 2.7623603E38F, 1.3160037E38F, -1.3754068E38F, 3.2011607E38F, 3.2032774E38F, -2.021344E38F, 1.4110548E38F, -2.0518247E38F, 1.5135234E37F, 4.0681463E37F, 6.1593495E37F, -9.297419E37F, -1.6420861E38F, 1.9968531E38F, 3.0120918E38F, 8.735197E37F, -1.1786597E38F, 5.0235934E37F, 3.199865E38F, -1.8489027E38F, 6.621483E37F, -1.0229179E38F, 1.5913172E38F, 1.0161583E38F, -2.1208456E38F, 1.390534E38F, 2.8183088E38F, 1.7137861E38F, 2.7425602E36F, 2.184179E38F, 4.5096275E37F, -1.82802E38F}));
            assert(pack.alt_GET() == 813951335);
            assert(pack.time_usec_GET() == 6502958880485153453L);
            assert(pack.relative_alt_GET() == 1021683073);
            assert(pack.vy_GET() == -3.3537707E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
            assert(pack.vz_GET() == -1.9529054E38F);
            assert(pack.vx_GET() == -1.3113377E38F);
            assert(pack.lon_GET() == -1297234949);
            assert(pack.lat_GET() == -1852025840);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE) ;
        p63.vy_SET(-3.3537707E38F) ;
        p63.alt_SET(813951335) ;
        p63.vx_SET(-1.3113377E38F) ;
        p63.lat_SET(-1852025840) ;
        p63.time_usec_SET(6502958880485153453L) ;
        p63.lon_SET(-1297234949) ;
        p63.relative_alt_SET(1021683073) ;
        p63.vz_SET(-1.9529054E38F) ;
        p63.covariance_SET(new float[] {-1.3044527E38F, 2.5721837E37F, -2.7451102E38F, 1.0493493E38F, 2.7623603E38F, 1.3160037E38F, -1.3754068E38F, 3.2011607E38F, 3.2032774E38F, -2.021344E38F, 1.4110548E38F, -2.0518247E38F, 1.5135234E37F, 4.0681463E37F, 6.1593495E37F, -9.297419E37F, -1.6420861E38F, 1.9968531E38F, 3.0120918E38F, 8.735197E37F, -1.1786597E38F, 5.0235934E37F, 3.199865E38F, -1.8489027E38F, 6.621483E37F, -1.0229179E38F, 1.5913172E38F, 1.0161583E38F, -2.1208456E38F, 1.390534E38F, 2.8183088E38F, 1.7137861E38F, 2.7425602E36F, 2.184179E38F, 4.5096275E37F, -1.82802E38F}, 0) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 3.1691064E38F);
            assert(pack.y_GET() == 1.2558012E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.4451754E38F, 5.3485475E37F, -1.4189175E38F, -2.5697499E38F, -2.7602818E37F, 1.683492E38F, 3.0090549E38F, -1.5197868E38F, -2.6313579E38F, -2.027421E38F, -4.449909E37F, 1.4657306E38F, -9.744543E37F, 9.606473E37F, -2.7103112E38F, 7.027482E37F, 4.4732667E37F, -1.6954358E37F, 2.0412483E38F, -3.0453458E38F, 2.503526E38F, -2.2172858E38F, 3.1510752E38F, 3.0350515E38F, -2.2429228E38F, 5.996268E37F, 2.7129321E38F, 1.0773377E38F, -4.1255428E37F, -8.5006475E36F, -1.9094773E38F, 2.0333596E38F, -3.2633746E38F, -1.201167E37F, 2.6558373E38F, -1.1977617E38F, 2.1195599E38F, -1.7071116E38F, 2.0123282E38F, -2.3475575E38F, -1.6246345E38F, -2.197864E38F, 1.9879465E38F, 1.8969243E38F, -3.3090946E38F}));
            assert(pack.ay_GET() == 6.0586057E37F);
            assert(pack.az_GET() == 1.062559E38F);
            assert(pack.vx_GET() == 1.409642E38F);
            assert(pack.vz_GET() == 2.3264832E38F);
            assert(pack.z_GET() == -2.3097565E38F);
            assert(pack.ax_GET() == 1.1496343E38F);
            assert(pack.x_GET() == 1.4965565E37F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
            assert(pack.time_usec_GET() == 5216244750886687871L);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.vy_SET(3.1691064E38F) ;
        p64.ax_SET(1.1496343E38F) ;
        p64.az_SET(1.062559E38F) ;
        p64.vz_SET(2.3264832E38F) ;
        p64.y_SET(1.2558012E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE) ;
        p64.ay_SET(6.0586057E37F) ;
        p64.time_usec_SET(5216244750886687871L) ;
        p64.x_SET(1.4965565E37F) ;
        p64.z_SET(-2.3097565E38F) ;
        p64.vx_SET(1.409642E38F) ;
        p64.covariance_SET(new float[] {1.4451754E38F, 5.3485475E37F, -1.4189175E38F, -2.5697499E38F, -2.7602818E37F, 1.683492E38F, 3.0090549E38F, -1.5197868E38F, -2.6313579E38F, -2.027421E38F, -4.449909E37F, 1.4657306E38F, -9.744543E37F, 9.606473E37F, -2.7103112E38F, 7.027482E37F, 4.4732667E37F, -1.6954358E37F, 2.0412483E38F, -3.0453458E38F, 2.503526E38F, -2.2172858E38F, 3.1510752E38F, 3.0350515E38F, -2.2429228E38F, 5.996268E37F, 2.7129321E38F, 1.0773377E38F, -4.1255428E37F, -8.5006475E36F, -1.9094773E38F, 2.0333596E38F, -3.2633746E38F, -1.201167E37F, 2.6558373E38F, -1.1977617E38F, 2.1195599E38F, -1.7071116E38F, 2.0123282E38F, -2.3475575E38F, -1.6246345E38F, -2.197864E38F, 1.9879465E38F, 1.8969243E38F, -3.3090946E38F}, 0) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)44331);
            assert(pack.chan12_raw_GET() == (char)33828);
            assert(pack.chan10_raw_GET() == (char)59782);
            assert(pack.chan7_raw_GET() == (char)58251);
            assert(pack.chan13_raw_GET() == (char)25660);
            assert(pack.rssi_GET() == (char)63);
            assert(pack.chan5_raw_GET() == (char)24544);
            assert(pack.chancount_GET() == (char)154);
            assert(pack.chan16_raw_GET() == (char)56302);
            assert(pack.chan2_raw_GET() == (char)7313);
            assert(pack.chan9_raw_GET() == (char)44943);
            assert(pack.time_boot_ms_GET() == 1581338323L);
            assert(pack.chan1_raw_GET() == (char)43325);
            assert(pack.chan8_raw_GET() == (char)22220);
            assert(pack.chan18_raw_GET() == (char)16904);
            assert(pack.chan15_raw_GET() == (char)53762);
            assert(pack.chan3_raw_GET() == (char)46683);
            assert(pack.chan11_raw_GET() == (char)10828);
            assert(pack.chan17_raw_GET() == (char)36707);
            assert(pack.chan14_raw_GET() == (char)61061);
            assert(pack.chan6_raw_GET() == (char)58174);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan17_raw_SET((char)36707) ;
        p65.chan13_raw_SET((char)25660) ;
        p65.chan8_raw_SET((char)22220) ;
        p65.chan9_raw_SET((char)44943) ;
        p65.chancount_SET((char)154) ;
        p65.chan5_raw_SET((char)24544) ;
        p65.chan14_raw_SET((char)61061) ;
        p65.chan7_raw_SET((char)58251) ;
        p65.chan16_raw_SET((char)56302) ;
        p65.rssi_SET((char)63) ;
        p65.chan4_raw_SET((char)44331) ;
        p65.chan10_raw_SET((char)59782) ;
        p65.time_boot_ms_SET(1581338323L) ;
        p65.chan3_raw_SET((char)46683) ;
        p65.chan15_raw_SET((char)53762) ;
        p65.chan12_raw_SET((char)33828) ;
        p65.chan11_raw_SET((char)10828) ;
        p65.chan18_raw_SET((char)16904) ;
        p65.chan6_raw_SET((char)58174) ;
        p65.chan1_raw_SET((char)43325) ;
        p65.chan2_raw_SET((char)7313) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)54);
            assert(pack.req_message_rate_GET() == (char)38437);
            assert(pack.start_stop_GET() == (char)226);
            assert(pack.req_stream_id_GET() == (char)28);
            assert(pack.target_system_GET() == (char)128);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_component_SET((char)54) ;
        p66.req_stream_id_SET((char)28) ;
        p66.req_message_rate_SET((char)38437) ;
        p66.start_stop_SET((char)226) ;
        p66.target_system_SET((char)128) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.message_rate_GET() == (char)33941);
            assert(pack.on_off_GET() == (char)70);
            assert(pack.stream_id_GET() == (char)49);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)33941) ;
        p67.stream_id_SET((char)49) ;
        p67.on_off_SET((char)70) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.buttons_GET() == (char)48556);
            assert(pack.z_GET() == (short) -4064);
            assert(pack.y_GET() == (short)12915);
            assert(pack.x_GET() == (short)15663);
            assert(pack.r_GET() == (short)6017);
            assert(pack.target_GET() == (char)96);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.buttons_SET((char)48556) ;
        p69.z_SET((short) -4064) ;
        p69.r_SET((short)6017) ;
        p69.y_SET((short)12915) ;
        p69.x_SET((short)15663) ;
        p69.target_SET((char)96) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan3_raw_GET() == (char)20739);
            assert(pack.target_component_GET() == (char)94);
            assert(pack.chan7_raw_GET() == (char)34156);
            assert(pack.chan6_raw_GET() == (char)44899);
            assert(pack.chan5_raw_GET() == (char)16917);
            assert(pack.target_system_GET() == (char)222);
            assert(pack.chan2_raw_GET() == (char)1054);
            assert(pack.chan1_raw_GET() == (char)9186);
            assert(pack.chan8_raw_GET() == (char)60319);
            assert(pack.chan4_raw_GET() == (char)42862);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan1_raw_SET((char)9186) ;
        p70.chan2_raw_SET((char)1054) ;
        p70.chan5_raw_SET((char)16917) ;
        p70.chan6_raw_SET((char)44899) ;
        p70.target_system_SET((char)222) ;
        p70.chan7_raw_SET((char)34156) ;
        p70.chan8_raw_SET((char)60319) ;
        p70.chan4_raw_SET((char)42862) ;
        p70.target_component_SET((char)94) ;
        p70.chan3_raw_SET((char)20739) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param4_GET() == 2.1292086E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.param2_GET() == 1.1663928E37F);
            assert(pack.y_GET() == -1383387696);
            assert(pack.autocontinue_GET() == (char)158);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.param3_GET() == 1.7372257E38F);
            assert(pack.z_GET() == 6.370026E37F);
            assert(pack.param1_GET() == -2.3131052E38F);
            assert(pack.target_component_GET() == (char)65);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_PANORAMA_CREATE);
            assert(pack.target_system_GET() == (char)69);
            assert(pack.seq_GET() == (char)47423);
            assert(pack.current_GET() == (char)19);
            assert(pack.x_GET() == 980204306);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p73.param1_SET(-2.3131052E38F) ;
        p73.target_component_SET((char)65) ;
        p73.z_SET(6.370026E37F) ;
        p73.autocontinue_SET((char)158) ;
        p73.x_SET(980204306) ;
        p73.command_SET(MAV_CMD.MAV_CMD_PANORAMA_CREATE) ;
        p73.param2_SET(1.1663928E37F) ;
        p73.param4_SET(2.1292086E38F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p73.target_system_SET((char)69) ;
        p73.y_SET(-1383387696) ;
        p73.seq_SET((char)47423) ;
        p73.current_SET((char)19) ;
        p73.param3_SET(1.7372257E38F) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.climb_GET() == 3.3128274E38F);
            assert(pack.groundspeed_GET() == -3.764543E37F);
            assert(pack.alt_GET() == 2.7483273E38F);
            assert(pack.heading_GET() == (short)3790);
            assert(pack.airspeed_GET() == -1.0794424E38F);
            assert(pack.throttle_GET() == (char)61509);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.climb_SET(3.3128274E38F) ;
        p74.throttle_SET((char)61509) ;
        p74.alt_SET(2.7483273E38F) ;
        p74.heading_SET((short)3790) ;
        p74.groundspeed_SET(-3.764543E37F) ;
        p74.airspeed_SET(-1.0794424E38F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.current_GET() == (char)125);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.y_GET() == 740024100);
            assert(pack.param4_GET() == -9.941262E37F);
            assert(pack.param1_GET() == 2.5721823E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_WAYPOINT_USER_1);
            assert(pack.x_GET() == -81063682);
            assert(pack.autocontinue_GET() == (char)49);
            assert(pack.param3_GET() == 9.832352E37F);
            assert(pack.target_system_GET() == (char)179);
            assert(pack.param2_GET() == -9.100697E37F);
            assert(pack.z_GET() == -2.3761944E38F);
            assert(pack.target_component_GET() == (char)22);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.param4_SET(-9.941262E37F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p75.x_SET(-81063682) ;
        p75.target_system_SET((char)179) ;
        p75.param1_SET(2.5721823E38F) ;
        p75.target_component_SET((char)22) ;
        p75.autocontinue_SET((char)49) ;
        p75.current_SET((char)125) ;
        p75.param3_SET(9.832352E37F) ;
        p75.param2_SET(-9.100697E37F) ;
        p75.z_SET(-2.3761944E38F) ;
        p75.y_SET(740024100) ;
        p75.command_SET(MAV_CMD.MAV_CMD_WAYPOINT_USER_1) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param2_GET() == -2.2675746E38F);
            assert(pack.param4_GET() == -1.0633061E38F);
            assert(pack.target_system_GET() == (char)151);
            assert(pack.target_component_GET() == (char)2);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_WAYPOINT_USER_5);
            assert(pack.param3_GET() == -1.9984388E38F);
            assert(pack.param7_GET() == 1.8771046E38F);
            assert(pack.param6_GET() == 3.4698419E37F);
            assert(pack.param5_GET() == 2.1128043E37F);
            assert(pack.param1_GET() == 2.9255504E38F);
            assert(pack.confirmation_GET() == (char)221);
        });
        GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param3_SET(-1.9984388E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_WAYPOINT_USER_5) ;
        p76.param2_SET(-2.2675746E38F) ;
        p76.target_system_SET((char)151) ;
        p76.param7_SET(1.8771046E38F) ;
        p76.param5_SET(2.1128043E37F) ;
        p76.target_component_SET((char)2) ;
        p76.confirmation_SET((char)221) ;
        p76.param4_SET(-1.0633061E38F) ;
        p76.param1_SET(2.9255504E38F) ;
        p76.param6_SET(3.4698419E37F) ;
        CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_SPATIAL_USER_1);
            assert(pack.result_param2_TRY(ph) == 720290555);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_ACCEPTED);
            assert(pack.progress_TRY(ph) == (char)151);
            assert(pack.target_system_TRY(ph) == (char)22);
            assert(pack.target_component_TRY(ph) == (char)90);
        });
        GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.target_component_SET((char)90, PH) ;
        p77.result_param2_SET(720290555, PH) ;
        p77.target_system_SET((char)22, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_SPATIAL_USER_1) ;
        p77.progress_SET((char)151, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_ACCEPTED) ;
        CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2815795790L);
            assert(pack.thrust_GET() == -8.893492E37F);
            assert(pack.roll_GET() == -2.1699842E38F);
            assert(pack.mode_switch_GET() == (char)49);
            assert(pack.yaw_GET() == -5.129815E37F);
            assert(pack.manual_override_switch_GET() == (char)169);
            assert(pack.pitch_GET() == 2.2657153E38F);
        });
        GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.manual_override_switch_SET((char)169) ;
        p81.pitch_SET(2.2657153E38F) ;
        p81.mode_switch_SET((char)49) ;
        p81.thrust_SET(-8.893492E37F) ;
        p81.time_boot_ms_SET(2815795790L) ;
        p81.roll_SET(-2.1699842E38F) ;
        p81.yaw_SET(-5.129815E37F) ;
        CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_roll_rate_GET() == 1.0541395E38F);
            assert(pack.type_mask_GET() == (char)211);
            assert(pack.body_pitch_rate_GET() == 1.1925752E38F);
            assert(pack.target_component_GET() == (char)208);
            assert(pack.target_system_GET() == (char)196);
            assert(pack.thrust_GET() == 3.9189152E37F);
            assert(pack.time_boot_ms_GET() == 1987742391L);
            assert(pack.body_yaw_rate_GET() == 2.7181911E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.711795E38F, 2.771925E38F, 2.5655072E38F, -2.5332348E38F}));
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.time_boot_ms_SET(1987742391L) ;
        p82.body_roll_rate_SET(1.0541395E38F) ;
        p82.type_mask_SET((char)211) ;
        p82.target_component_SET((char)208) ;
        p82.target_system_SET((char)196) ;
        p82.body_pitch_rate_SET(1.1925752E38F) ;
        p82.body_yaw_rate_SET(2.7181911E38F) ;
        p82.q_SET(new float[] {-2.711795E38F, 2.771925E38F, 2.5655072E38F, -2.5332348E38F}, 0) ;
        p82.thrust_SET(3.9189152E37F) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)137);
            assert(pack.thrust_GET() == 3.0746642E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.0357162E38F, -1.1061467E37F, 1.9326045E38F, -1.8963037E38F}));
            assert(pack.body_roll_rate_GET() == 3.925798E36F);
            assert(pack.body_pitch_rate_GET() == 1.2447991E38F);
            assert(pack.time_boot_ms_GET() == 3873318805L);
            assert(pack.body_yaw_rate_GET() == -2.735208E38F);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_roll_rate_SET(3.925798E36F) ;
        p83.thrust_SET(3.0746642E37F) ;
        p83.type_mask_SET((char)137) ;
        p83.body_yaw_rate_SET(-2.735208E38F) ;
        p83.q_SET(new float[] {2.0357162E38F, -1.1061467E37F, 1.9326045E38F, -1.8963037E38F}, 0) ;
        p83.body_pitch_rate_SET(1.2447991E38F) ;
        p83.time_boot_ms_SET(3873318805L) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)55);
            assert(pack.vx_GET() == -1.3457156E38F);
            assert(pack.x_GET() == -1.6799187E38F);
            assert(pack.afy_GET() == -3.2699203E38F);
            assert(pack.yaw_rate_GET() == -2.9077818E38F);
            assert(pack.target_component_GET() == (char)187);
            assert(pack.type_mask_GET() == (char)33847);
            assert(pack.vy_GET() == 2.8843838E38F);
            assert(pack.afz_GET() == -2.8840032E37F);
            assert(pack.z_GET() == 2.1440961E38F);
            assert(pack.y_GET() == -2.2768956E38F);
            assert(pack.vz_GET() == 2.0889519E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.time_boot_ms_GET() == 955825672L);
            assert(pack.yaw_GET() == -1.7016086E38F);
            assert(pack.afx_GET() == -2.8261163E38F);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.yaw_rate_SET(-2.9077818E38F) ;
        p84.afx_SET(-2.8261163E38F) ;
        p84.time_boot_ms_SET(955825672L) ;
        p84.afy_SET(-3.2699203E38F) ;
        p84.afz_SET(-2.8840032E37F) ;
        p84.x_SET(-1.6799187E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p84.vz_SET(2.0889519E38F) ;
        p84.vy_SET(2.8843838E38F) ;
        p84.target_component_SET((char)187) ;
        p84.type_mask_SET((char)33847) ;
        p84.vx_SET(-1.3457156E38F) ;
        p84.y_SET(-2.2768956E38F) ;
        p84.yaw_SET(-1.7016086E38F) ;
        p84.z_SET(2.1440961E38F) ;
        p84.target_system_SET((char)55) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 2.1400088E38F);
            assert(pack.yaw_rate_GET() == -3.3633843E38F);
            assert(pack.time_boot_ms_GET() == 115761556L);
            assert(pack.afx_GET() == -4.1478542E37F);
            assert(pack.vx_GET() == -4.5527145E36F);
            assert(pack.target_system_GET() == (char)198);
            assert(pack.target_component_GET() == (char)180);
            assert(pack.lon_int_GET() == 1570733804);
            assert(pack.type_mask_GET() == (char)22817);
            assert(pack.yaw_GET() == 1.3584447E38F);
            assert(pack.alt_GET() == 2.0492075E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.vz_GET() == -2.6078686E38F);
            assert(pack.lat_int_GET() == -1528698089);
            assert(pack.afy_GET() == -6.0711367E37F);
            assert(pack.afz_GET() == 2.9805068E38F);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.lon_int_SET(1570733804) ;
        p86.time_boot_ms_SET(115761556L) ;
        p86.afx_SET(-4.1478542E37F) ;
        p86.vz_SET(-2.6078686E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p86.yaw_rate_SET(-3.3633843E38F) ;
        p86.target_component_SET((char)180) ;
        p86.type_mask_SET((char)22817) ;
        p86.afz_SET(2.9805068E38F) ;
        p86.yaw_SET(1.3584447E38F) ;
        p86.vx_SET(-4.5527145E36F) ;
        p86.alt_SET(2.0492075E38F) ;
        p86.lat_int_SET(-1528698089) ;
        p86.target_system_SET((char)198) ;
        p86.afy_SET(-6.0711367E37F) ;
        p86.vy_SET(2.1400088E38F) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.afy_GET() == -1.5471949E38F);
            assert(pack.vx_GET() == -1.1101542E38F);
            assert(pack.afz_GET() == -5.957085E37F);
            assert(pack.lat_int_GET() == -177184748);
            assert(pack.afx_GET() == -1.9322942E38F);
            assert(pack.vz_GET() == -2.9354273E38F);
            assert(pack.vy_GET() == -4.1179356E37F);
            assert(pack.yaw_GET() == 2.3039775E38F);
            assert(pack.time_boot_ms_GET() == 2305562605L);
            assert(pack.lon_int_GET() == -1151014580);
            assert(pack.alt_GET() == -7.329276E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.type_mask_GET() == (char)2079);
            assert(pack.yaw_rate_GET() == -2.293282E38F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.lon_int_SET(-1151014580) ;
        p87.vy_SET(-4.1179356E37F) ;
        p87.yaw_rate_SET(-2.293282E38F) ;
        p87.vz_SET(-2.9354273E38F) ;
        p87.vx_SET(-1.1101542E38F) ;
        p87.afy_SET(-1.5471949E38F) ;
        p87.afx_SET(-1.9322942E38F) ;
        p87.lat_int_SET(-177184748) ;
        p87.afz_SET(-5.957085E37F) ;
        p87.type_mask_SET((char)2079) ;
        p87.alt_SET(-7.329276E37F) ;
        p87.time_boot_ms_SET(2305562605L) ;
        p87.yaw_SET(2.3039775E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3009931848L);
            assert(pack.x_GET() == 1.5347742E38F);
            assert(pack.yaw_GET() == 2.1156298E38F);
            assert(pack.pitch_GET() == -9.772337E37F);
            assert(pack.z_GET() == 3.0589202E38F);
            assert(pack.y_GET() == 8.883663E37F);
            assert(pack.roll_GET() == 2.7829306E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.pitch_SET(-9.772337E37F) ;
        p89.roll_SET(2.7829306E38F) ;
        p89.yaw_SET(2.1156298E38F) ;
        p89.x_SET(1.5347742E38F) ;
        p89.time_boot_ms_SET(3009931848L) ;
        p89.y_SET(8.883663E37F) ;
        p89.z_SET(3.0589202E38F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 1733015662);
            assert(pack.vy_GET() == (short)7997);
            assert(pack.vz_GET() == (short)24183);
            assert(pack.xacc_GET() == (short) -3574);
            assert(pack.lon_GET() == -1922496971);
            assert(pack.zacc_GET() == (short)555);
            assert(pack.time_usec_GET() == 8214909504196034891L);
            assert(pack.roll_GET() == 6.1935907E37F);
            assert(pack.yacc_GET() == (short) -15633);
            assert(pack.yaw_GET() == 7.331721E37F);
            assert(pack.rollspeed_GET() == -8.491353E37F);
            assert(pack.pitch_GET() == -2.31264E38F);
            assert(pack.yawspeed_GET() == 3.0020398E38F);
            assert(pack.lat_GET() == 900256309);
            assert(pack.vx_GET() == (short) -30874);
            assert(pack.pitchspeed_GET() == 2.2167009E38F);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.yacc_SET((short) -15633) ;
        p90.vz_SET((short)24183) ;
        p90.vx_SET((short) -30874) ;
        p90.lat_SET(900256309) ;
        p90.lon_SET(-1922496971) ;
        p90.time_usec_SET(8214909504196034891L) ;
        p90.pitchspeed_SET(2.2167009E38F) ;
        p90.pitch_SET(-2.31264E38F) ;
        p90.xacc_SET((short) -3574) ;
        p90.rollspeed_SET(-8.491353E37F) ;
        p90.vy_SET((short)7997) ;
        p90.yaw_SET(7.331721E37F) ;
        p90.alt_SET(1733015662) ;
        p90.zacc_SET((short)555) ;
        p90.yawspeed_SET(3.0020398E38F) ;
        p90.roll_SET(6.1935907E37F) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_ARMED);
            assert(pack.aux4_GET() == -1.9399583E38F);
            assert(pack.aux3_GET() == -2.3626176E38F);
            assert(pack.throttle_GET() == -9.1277815E36F);
            assert(pack.aux2_GET() == 2.190758E38F);
            assert(pack.roll_ailerons_GET() == -2.0097256E38F);
            assert(pack.nav_mode_GET() == (char)118);
            assert(pack.yaw_rudder_GET() == -7.335486E37F);
            assert(pack.aux1_GET() == -1.9473914E38F);
            assert(pack.pitch_elevator_GET() == 9.49703E37F);
            assert(pack.time_usec_GET() == 6765324872623006914L);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.time_usec_SET(6765324872623006914L) ;
        p91.nav_mode_SET((char)118) ;
        p91.aux3_SET(-2.3626176E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED) ;
        p91.yaw_rudder_SET(-7.335486E37F) ;
        p91.roll_ailerons_SET(-2.0097256E38F) ;
        p91.pitch_elevator_SET(9.49703E37F) ;
        p91.aux4_SET(-1.9399583E38F) ;
        p91.throttle_SET(-9.1277815E36F) ;
        p91.aux1_SET(-1.9473914E38F) ;
        p91.aux2_SET(2.190758E38F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan10_raw_GET() == (char)60240);
            assert(pack.chan4_raw_GET() == (char)13276);
            assert(pack.chan9_raw_GET() == (char)3144);
            assert(pack.rssi_GET() == (char)172);
            assert(pack.chan7_raw_GET() == (char)20154);
            assert(pack.time_usec_GET() == 3193251876007990161L);
            assert(pack.chan1_raw_GET() == (char)32982);
            assert(pack.chan11_raw_GET() == (char)28664);
            assert(pack.chan3_raw_GET() == (char)32566);
            assert(pack.chan12_raw_GET() == (char)41537);
            assert(pack.chan6_raw_GET() == (char)25537);
            assert(pack.chan2_raw_GET() == (char)21778);
            assert(pack.chan8_raw_GET() == (char)49853);
            assert(pack.chan5_raw_GET() == (char)32523);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan11_raw_SET((char)28664) ;
        p92.time_usec_SET(3193251876007990161L) ;
        p92.chan4_raw_SET((char)13276) ;
        p92.chan2_raw_SET((char)21778) ;
        p92.chan9_raw_SET((char)3144) ;
        p92.chan3_raw_SET((char)32566) ;
        p92.rssi_SET((char)172) ;
        p92.chan12_raw_SET((char)41537) ;
        p92.chan5_raw_SET((char)32523) ;
        p92.chan7_raw_SET((char)20154) ;
        p92.chan6_raw_SET((char)25537) ;
        p92.chan8_raw_SET((char)49853) ;
        p92.chan1_raw_SET((char)32982) ;
        p92.chan10_raw_SET((char)60240) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {9.1687076E36F, 1.6249254E38F, -9.980525E37F, 8.623264E37F, -2.8461626E38F, 2.1697091E38F, -1.2541567E38F, -3.1655195E38F, 2.4756633E37F, -1.4857402E38F, 1.1825969E38F, -1.2865671E38F, -1.0117913E38F, 2.203631E38F, -1.5769119E38F, -9.142047E37F}));
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_ARMED);
            assert(pack.time_usec_GET() == 8728112059250454554L);
            assert(pack.flags_GET() == 5105925787245393100L);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.controls_SET(new float[] {9.1687076E36F, 1.6249254E38F, -9.980525E37F, 8.623264E37F, -2.8461626E38F, 2.1697091E38F, -1.2541567E38F, -3.1655195E38F, 2.4756633E37F, -1.4857402E38F, 1.1825969E38F, -1.2865671E38F, -1.0117913E38F, 2.203631E38F, -1.5769119E38F, -9.142047E37F}, 0) ;
        p93.flags_SET(5105925787245393100L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED) ;
        p93.time_usec_SET(8728112059250454554L) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_rate_y_TRY(ph) == -1.0695719E38F);
            assert(pack.flow_comp_m_x_GET() == 2.642533E38F);
            assert(pack.quality_GET() == (char)71);
            assert(pack.time_usec_GET() == 9202422133877395226L);
            assert(pack.flow_comp_m_y_GET() == 2.0008918E38F);
            assert(pack.flow_y_GET() == (short) -21943);
            assert(pack.ground_distance_GET() == -3.4185147E37F);
            assert(pack.flow_rate_x_TRY(ph) == -1.7826504E38F);
            assert(pack.sensor_id_GET() == (char)219);
            assert(pack.flow_x_GET() == (short)25375);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.quality_SET((char)71) ;
        p100.time_usec_SET(9202422133877395226L) ;
        p100.ground_distance_SET(-3.4185147E37F) ;
        p100.flow_rate_x_SET(-1.7826504E38F, PH) ;
        p100.flow_y_SET((short) -21943) ;
        p100.sensor_id_SET((char)219) ;
        p100.flow_comp_m_y_SET(2.0008918E38F) ;
        p100.flow_x_SET((short)25375) ;
        p100.flow_rate_y_SET(-1.0695719E38F, PH) ;
        p100.flow_comp_m_x_SET(2.642533E38F) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -2.8944098E38F);
            assert(pack.x_GET() == 2.8560789E38F);
            assert(pack.yaw_GET() == -1.5976973E38F);
            assert(pack.pitch_GET() == 3.3527336E38F);
            assert(pack.roll_GET() == 5.354551E37F);
            assert(pack.z_GET() == 2.9925055E38F);
            assert(pack.usec_GET() == 3524702726395005637L);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.x_SET(2.8560789E38F) ;
        p101.roll_SET(5.354551E37F) ;
        p101.y_SET(-2.8944098E38F) ;
        p101.usec_SET(3524702726395005637L) ;
        p101.yaw_SET(-1.5976973E38F) ;
        p101.z_SET(2.9925055E38F) ;
        p101.pitch_SET(3.3527336E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 2.2797266E38F);
            assert(pack.yaw_GET() == 1.0914321E37F);
            assert(pack.pitch_GET() == -1.9955928E38F);
            assert(pack.roll_GET() == 2.5443703E38F);
            assert(pack.y_GET() == 2.3880748E38F);
            assert(pack.z_GET() == -2.2342395E38F);
            assert(pack.usec_GET() == 5211757554916255540L);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(5211757554916255540L) ;
        p102.y_SET(2.3880748E38F) ;
        p102.z_SET(-2.2342395E38F) ;
        p102.roll_SET(2.5443703E38F) ;
        p102.pitch_SET(-1.9955928E38F) ;
        p102.x_SET(2.2797266E38F) ;
        p102.yaw_SET(1.0914321E37F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 1.4305934E38F);
            assert(pack.x_GET() == -2.6557655E38F);
            assert(pack.usec_GET() == 3605291234233379308L);
            assert(pack.y_GET() == 1.5332963E38F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(3605291234233379308L) ;
        p103.z_SET(1.4305934E38F) ;
        p103.x_SET(-2.6557655E38F) ;
        p103.y_SET(1.5332963E38F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -1.7938094E38F);
            assert(pack.yaw_GET() == -1.0125172E38F);
            assert(pack.usec_GET() == 8755875737972756862L);
            assert(pack.x_GET() == 1.4681696E38F);
            assert(pack.pitch_GET() == 3.011917E38F);
            assert(pack.roll_GET() == -1.8736663E38F);
            assert(pack.z_GET() == -1.5566347E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.pitch_SET(3.011917E38F) ;
        p104.usec_SET(8755875737972756862L) ;
        p104.x_SET(1.4681696E38F) ;
        p104.y_SET(-1.7938094E38F) ;
        p104.roll_SET(-1.8736663E38F) ;
        p104.yaw_SET(-1.0125172E38F) ;
        p104.z_SET(-1.5566347E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.pressure_alt_GET() == 2.1941898E38F);
            assert(pack.diff_pressure_GET() == 1.2772046E38F);
            assert(pack.time_usec_GET() == 555536603121528535L);
            assert(pack.xmag_GET() == 3.3030326E38F);
            assert(pack.zacc_GET() == -2.7557179E38F);
            assert(pack.xacc_GET() == 1.6516469E38F);
            assert(pack.abs_pressure_GET() == 7.9241574E37F);
            assert(pack.ymag_GET() == 3.4486303E37F);
            assert(pack.fields_updated_GET() == (char)46365);
            assert(pack.zgyro_GET() == 9.515304E37F);
            assert(pack.ygyro_GET() == 3.3499342E38F);
            assert(pack.zmag_GET() == 2.2844139E38F);
            assert(pack.temperature_GET() == 1.8253889E38F);
            assert(pack.xgyro_GET() == 2.3286125E38F);
            assert(pack.yacc_GET() == -2.490437E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.zmag_SET(2.2844139E38F) ;
        p105.xacc_SET(1.6516469E38F) ;
        p105.ygyro_SET(3.3499342E38F) ;
        p105.pressure_alt_SET(2.1941898E38F) ;
        p105.zgyro_SET(9.515304E37F) ;
        p105.fields_updated_SET((char)46365) ;
        p105.yacc_SET(-2.490437E38F) ;
        p105.time_usec_SET(555536603121528535L) ;
        p105.ymag_SET(3.4486303E37F) ;
        p105.zacc_SET(-2.7557179E38F) ;
        p105.diff_pressure_SET(1.2772046E38F) ;
        p105.temperature_SET(1.8253889E38F) ;
        p105.xmag_SET(3.3030326E38F) ;
        p105.abs_pressure_SET(7.9241574E37F) ;
        p105.xgyro_SET(2.3286125E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_y_GET() == -1.7143068E38F);
            assert(pack.temperature_GET() == (short)26759);
            assert(pack.integrated_x_GET() == 2.5074305E38F);
            assert(pack.integrated_ygyro_GET() == 2.9343036E37F);
            assert(pack.distance_GET() == 1.1812033E38F);
            assert(pack.time_usec_GET() == 7568542673307806966L);
            assert(pack.sensor_id_GET() == (char)188);
            assert(pack.time_delta_distance_us_GET() == 2087961403L);
            assert(pack.integrated_xgyro_GET() == -1.58446E38F);
            assert(pack.integration_time_us_GET() == 742255790L);
            assert(pack.integrated_zgyro_GET() == -2.9028808E37F);
            assert(pack.quality_GET() == (char)76);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_y_SET(-1.7143068E38F) ;
        p106.sensor_id_SET((char)188) ;
        p106.quality_SET((char)76) ;
        p106.integration_time_us_SET(742255790L) ;
        p106.integrated_x_SET(2.5074305E38F) ;
        p106.time_usec_SET(7568542673307806966L) ;
        p106.integrated_ygyro_SET(2.9343036E37F) ;
        p106.distance_SET(1.1812033E38F) ;
        p106.integrated_zgyro_SET(-2.9028808E37F) ;
        p106.integrated_xgyro_SET(-1.58446E38F) ;
        p106.time_delta_distance_us_SET(2087961403L) ;
        p106.temperature_SET((short)26759) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.fields_updated_GET() == 780058560L);
            assert(pack.xgyro_GET() == 6.4138415E37F);
            assert(pack.ygyro_GET() == 2.96947E38F);
            assert(pack.pressure_alt_GET() == -2.5996684E38F);
            assert(pack.xmag_GET() == -2.248727E35F);
            assert(pack.xacc_GET() == -9.004371E37F);
            assert(pack.abs_pressure_GET() == 3.2070806E38F);
            assert(pack.zacc_GET() == 5.553476E37F);
            assert(pack.time_usec_GET() == 8000816367270196073L);
            assert(pack.ymag_GET() == -2.6600798E38F);
            assert(pack.yacc_GET() == 9.518738E36F);
            assert(pack.zmag_GET() == 2.6606536E38F);
            assert(pack.diff_pressure_GET() == 1.8893776E38F);
            assert(pack.zgyro_GET() == -3.767707E37F);
            assert(pack.temperature_GET() == 1.6511441E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.xmag_SET(-2.248727E35F) ;
        p107.diff_pressure_SET(1.8893776E38F) ;
        p107.ygyro_SET(2.96947E38F) ;
        p107.zacc_SET(5.553476E37F) ;
        p107.pressure_alt_SET(-2.5996684E38F) ;
        p107.time_usec_SET(8000816367270196073L) ;
        p107.abs_pressure_SET(3.2070806E38F) ;
        p107.temperature_SET(1.6511441E38F) ;
        p107.zmag_SET(2.6606536E38F) ;
        p107.xgyro_SET(6.4138415E37F) ;
        p107.xacc_SET(-9.004371E37F) ;
        p107.zgyro_SET(-3.767707E37F) ;
        p107.yacc_SET(9.518738E36F) ;
        p107.fields_updated_SET(780058560L) ;
        p107.ymag_SET(-2.6600798E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1.5271218E38F);
            assert(pack.std_dev_horz_GET() == -2.3732946E38F);
            assert(pack.pitch_GET() == 1.4396465E38F);
            assert(pack.vn_GET() == 3.8802554E37F);
            assert(pack.std_dev_vert_GET() == 2.2595587E36F);
            assert(pack.q2_GET() == -1.0531691E38F);
            assert(pack.ygyro_GET() == -1.9926147E38F);
            assert(pack.ve_GET() == -1.3601718E38F);
            assert(pack.yacc_GET() == -1.7202063E38F);
            assert(pack.xacc_GET() == 1.4201185E38F);
            assert(pack.zgyro_GET() == -1.7074479E38F);
            assert(pack.roll_GET() == 2.4679861E38F);
            assert(pack.alt_GET() == 2.04302E38F);
            assert(pack.xgyro_GET() == -3.3364598E38F);
            assert(pack.lon_GET() == -3.0794606E38F);
            assert(pack.zacc_GET() == 8.656432E37F);
            assert(pack.q3_GET() == -4.197407E37F);
            assert(pack.q1_GET() == -3.203418E38F);
            assert(pack.yaw_GET() == 2.8301736E38F);
            assert(pack.vd_GET() == 2.9869385E38F);
            assert(pack.q4_GET() == -2.245828E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.lat_SET(-1.5271218E38F) ;
        p108.ygyro_SET(-1.9926147E38F) ;
        p108.yaw_SET(2.8301736E38F) ;
        p108.pitch_SET(1.4396465E38F) ;
        p108.std_dev_horz_SET(-2.3732946E38F) ;
        p108.yacc_SET(-1.7202063E38F) ;
        p108.q2_SET(-1.0531691E38F) ;
        p108.lon_SET(-3.0794606E38F) ;
        p108.xacc_SET(1.4201185E38F) ;
        p108.zgyro_SET(-1.7074479E38F) ;
        p108.roll_SET(2.4679861E38F) ;
        p108.q4_SET(-2.245828E38F) ;
        p108.zacc_SET(8.656432E37F) ;
        p108.q1_SET(-3.203418E38F) ;
        p108.alt_SET(2.04302E38F) ;
        p108.vd_SET(2.9869385E38F) ;
        p108.xgyro_SET(-3.3364598E38F) ;
        p108.ve_SET(-1.3601718E38F) ;
        p108.vn_SET(3.8802554E37F) ;
        p108.std_dev_vert_SET(2.2595587E36F) ;
        p108.q3_SET(-4.197407E37F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.txbuf_GET() == (char)237);
            assert(pack.noise_GET() == (char)155);
            assert(pack.remnoise_GET() == (char)158);
            assert(pack.remrssi_GET() == (char)43);
            assert(pack.rxerrors_GET() == (char)33329);
            assert(pack.rssi_GET() == (char)228);
            assert(pack.fixed__GET() == (char)41257);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.remrssi_SET((char)43) ;
        p109.noise_SET((char)155) ;
        p109.remnoise_SET((char)158) ;
        p109.rxerrors_SET((char)33329) ;
        p109.fixed__SET((char)41257) ;
        p109.rssi_SET((char)228) ;
        p109.txbuf_SET((char)237) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)202);
            assert(pack.target_component_GET() == (char)159);
            assert(pack.target_system_GET() == (char)148);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)190, (char)19, (char)190, (char)189, (char)187, (char)122, (char)230, (char)70, (char)103, (char)63, (char)137, (char)81, (char)207, (char)236, (char)131, (char)241, (char)252, (char)157, (char)251, (char)250, (char)167, (char)12, (char)231, (char)162, (char)200, (char)147, (char)133, (char)180, (char)249, (char)119, (char)231, (char)113, (char)20, (char)189, (char)0, (char)37, (char)109, (char)109, (char)206, (char)229, (char)14, (char)82, (char)80, (char)184, (char)81, (char)30, (char)68, (char)107, (char)79, (char)16, (char)208, (char)219, (char)14, (char)84, (char)20, (char)11, (char)198, (char)56, (char)214, (char)176, (char)239, (char)98, (char)169, (char)196, (char)235, (char)23, (char)185, (char)1, (char)0, (char)45, (char)130, (char)159, (char)58, (char)252, (char)77, (char)246, (char)183, (char)91, (char)220, (char)141, (char)121, (char)158, (char)210, (char)31, (char)2, (char)66, (char)128, (char)203, (char)232, (char)104, (char)139, (char)27, (char)245, (char)84, (char)210, (char)148, (char)19, (char)205, (char)235, (char)64, (char)12, (char)205, (char)231, (char)15, (char)255, (char)119, (char)43, (char)61, (char)253, (char)60, (char)37, (char)187, (char)128, (char)48, (char)185, (char)24, (char)91, (char)35, (char)231, (char)34, (char)178, (char)53, (char)99, (char)41, (char)91, (char)116, (char)156, (char)151, (char)68, (char)232, (char)96, (char)245, (char)133, (char)53, (char)1, (char)135, (char)234, (char)93, (char)163, (char)32, (char)245, (char)190, (char)112, (char)119, (char)162, (char)101, (char)8, (char)98, (char)134, (char)10, (char)203, (char)187, (char)232, (char)54, (char)174, (char)136, (char)185, (char)102, (char)220, (char)205, (char)97, (char)136, (char)164, (char)139, (char)15, (char)102, (char)207, (char)114, (char)250, (char)199, (char)53, (char)14, (char)39, (char)252, (char)21, (char)212, (char)144, (char)112, (char)101, (char)33, (char)248, (char)156, (char)128, (char)46, (char)159, (char)245, (char)44, (char)193, (char)164, (char)69, (char)64, (char)7, (char)55, (char)57, (char)107, (char)209, (char)22, (char)140, (char)168, (char)50, (char)4, (char)144, (char)7, (char)95, (char)135, (char)123, (char)23, (char)123, (char)33, (char)61, (char)125, (char)239, (char)36, (char)241, (char)140, (char)135, (char)181, (char)213, (char)63, (char)104, (char)221, (char)239, (char)17, (char)107, (char)174, (char)27, (char)158, (char)180, (char)198, (char)193, (char)245, (char)114, (char)56, (char)128, (char)27, (char)120, (char)70, (char)161, (char)186, (char)184, (char)27, (char)90, (char)16, (char)226, (char)180, (char)181, (char)238, (char)193, (char)179, (char)125, (char)30}));
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)202) ;
        p110.target_system_SET((char)148) ;
        p110.target_component_SET((char)159) ;
        p110.payload_SET(new char[] {(char)190, (char)19, (char)190, (char)189, (char)187, (char)122, (char)230, (char)70, (char)103, (char)63, (char)137, (char)81, (char)207, (char)236, (char)131, (char)241, (char)252, (char)157, (char)251, (char)250, (char)167, (char)12, (char)231, (char)162, (char)200, (char)147, (char)133, (char)180, (char)249, (char)119, (char)231, (char)113, (char)20, (char)189, (char)0, (char)37, (char)109, (char)109, (char)206, (char)229, (char)14, (char)82, (char)80, (char)184, (char)81, (char)30, (char)68, (char)107, (char)79, (char)16, (char)208, (char)219, (char)14, (char)84, (char)20, (char)11, (char)198, (char)56, (char)214, (char)176, (char)239, (char)98, (char)169, (char)196, (char)235, (char)23, (char)185, (char)1, (char)0, (char)45, (char)130, (char)159, (char)58, (char)252, (char)77, (char)246, (char)183, (char)91, (char)220, (char)141, (char)121, (char)158, (char)210, (char)31, (char)2, (char)66, (char)128, (char)203, (char)232, (char)104, (char)139, (char)27, (char)245, (char)84, (char)210, (char)148, (char)19, (char)205, (char)235, (char)64, (char)12, (char)205, (char)231, (char)15, (char)255, (char)119, (char)43, (char)61, (char)253, (char)60, (char)37, (char)187, (char)128, (char)48, (char)185, (char)24, (char)91, (char)35, (char)231, (char)34, (char)178, (char)53, (char)99, (char)41, (char)91, (char)116, (char)156, (char)151, (char)68, (char)232, (char)96, (char)245, (char)133, (char)53, (char)1, (char)135, (char)234, (char)93, (char)163, (char)32, (char)245, (char)190, (char)112, (char)119, (char)162, (char)101, (char)8, (char)98, (char)134, (char)10, (char)203, (char)187, (char)232, (char)54, (char)174, (char)136, (char)185, (char)102, (char)220, (char)205, (char)97, (char)136, (char)164, (char)139, (char)15, (char)102, (char)207, (char)114, (char)250, (char)199, (char)53, (char)14, (char)39, (char)252, (char)21, (char)212, (char)144, (char)112, (char)101, (char)33, (char)248, (char)156, (char)128, (char)46, (char)159, (char)245, (char)44, (char)193, (char)164, (char)69, (char)64, (char)7, (char)55, (char)57, (char)107, (char)209, (char)22, (char)140, (char)168, (char)50, (char)4, (char)144, (char)7, (char)95, (char)135, (char)123, (char)23, (char)123, (char)33, (char)61, (char)125, (char)239, (char)36, (char)241, (char)140, (char)135, (char)181, (char)213, (char)63, (char)104, (char)221, (char)239, (char)17, (char)107, (char)174, (char)27, (char)158, (char)180, (char)198, (char)193, (char)245, (char)114, (char)56, (char)128, (char)27, (char)120, (char)70, (char)161, (char)186, (char)184, (char)27, (char)90, (char)16, (char)226, (char)180, (char)181, (char)238, (char)193, (char)179, (char)125, (char)30}, 0) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -2808071378317298162L);
            assert(pack.ts1_GET() == 1397566783820132797L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-2808071378317298162L) ;
        p111.ts1_SET(1397566783820132797L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 1317780653L);
            assert(pack.time_usec_GET() == 2525427965498549623L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(2525427965498549623L) ;
        p112.seq_SET(1317780653L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 682055684);
            assert(pack.lon_GET() == -342375576);
            assert(pack.satellites_visible_GET() == (char)172);
            assert(pack.eph_GET() == (char)20817);
            assert(pack.fix_type_GET() == (char)57);
            assert(pack.epv_GET() == (char)28099);
            assert(pack.time_usec_GET() == 7316831151987268822L);
            assert(pack.vd_GET() == (short)16539);
            assert(pack.cog_GET() == (char)34033);
            assert(pack.alt_GET() == 1430916243);
            assert(pack.ve_GET() == (short) -5392);
            assert(pack.vel_GET() == (char)62597);
            assert(pack.vn_GET() == (short) -17337);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.lon_SET(-342375576) ;
        p113.fix_type_SET((char)57) ;
        p113.ve_SET((short) -5392) ;
        p113.time_usec_SET(7316831151987268822L) ;
        p113.vn_SET((short) -17337) ;
        p113.lat_SET(682055684) ;
        p113.eph_SET((char)20817) ;
        p113.satellites_visible_SET((char)172) ;
        p113.alt_SET(1430916243) ;
        p113.vel_SET((char)62597) ;
        p113.cog_SET((char)34033) ;
        p113.vd_SET((short)16539) ;
        p113.epv_SET((char)28099) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_xgyro_GET() == -3.19912E38F);
            assert(pack.integration_time_us_GET() == 729887762L);
            assert(pack.integrated_ygyro_GET() == -1.9295672E38F);
            assert(pack.integrated_y_GET() == -3.293317E38F);
            assert(pack.integrated_zgyro_GET() == -3.3960976E38F);
            assert(pack.time_delta_distance_us_GET() == 2687301593L);
            assert(pack.time_usec_GET() == 3646957447068446278L);
            assert(pack.distance_GET() == -8.713121E36F);
            assert(pack.integrated_x_GET() == -2.3127934E38F);
            assert(pack.quality_GET() == (char)25);
            assert(pack.sensor_id_GET() == (char)234);
            assert(pack.temperature_GET() == (short) -19553);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_y_SET(-3.293317E38F) ;
        p114.integrated_x_SET(-2.3127934E38F) ;
        p114.sensor_id_SET((char)234) ;
        p114.quality_SET((char)25) ;
        p114.integrated_xgyro_SET(-3.19912E38F) ;
        p114.time_delta_distance_us_SET(2687301593L) ;
        p114.time_usec_SET(3646957447068446278L) ;
        p114.integrated_zgyro_SET(-3.3960976E38F) ;
        p114.distance_SET(-8.713121E36F) ;
        p114.integration_time_us_SET(729887762L) ;
        p114.temperature_SET((short) -19553) ;
        p114.integrated_ygyro_SET(-1.9295672E38F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 233171351);
            assert(pack.vz_GET() == (short) -7074);
            assert(pack.lon_GET() == 793923343);
            assert(pack.true_airspeed_GET() == (char)57232);
            assert(pack.xacc_GET() == (short) -10148);
            assert(pack.time_usec_GET() == 6383831274047917978L);
            assert(pack.pitchspeed_GET() == 2.665634E38F);
            assert(pack.alt_GET() == 424452368);
            assert(pack.vy_GET() == (short)17119);
            assert(pack.ind_airspeed_GET() == (char)3045);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {-3.0708992E38F, -3.1211793E38F, -1.49942E38F, 5.732021E36F}));
            assert(pack.rollspeed_GET() == -2.8348678E37F);
            assert(pack.vx_GET() == (short)5138);
            assert(pack.yawspeed_GET() == 1.2451276E38F);
            assert(pack.zacc_GET() == (short) -4807);
            assert(pack.yacc_GET() == (short)11802);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.lat_SET(233171351) ;
        p115.vz_SET((short) -7074) ;
        p115.pitchspeed_SET(2.665634E38F) ;
        p115.xacc_SET((short) -10148) ;
        p115.attitude_quaternion_SET(new float[] {-3.0708992E38F, -3.1211793E38F, -1.49942E38F, 5.732021E36F}, 0) ;
        p115.yacc_SET((short)11802) ;
        p115.time_usec_SET(6383831274047917978L) ;
        p115.rollspeed_SET(-2.8348678E37F) ;
        p115.lon_SET(793923343) ;
        p115.true_airspeed_SET((char)57232) ;
        p115.ind_airspeed_SET((char)3045) ;
        p115.alt_SET(424452368) ;
        p115.vx_SET((short)5138) ;
        p115.zacc_SET((short) -4807) ;
        p115.vy_SET((short)17119) ;
        p115.yawspeed_SET(1.2451276E38F) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short)2273);
            assert(pack.yacc_GET() == (short)2320);
            assert(pack.xmag_GET() == (short)30095);
            assert(pack.xacc_GET() == (short)28459);
            assert(pack.xgyro_GET() == (short)8596);
            assert(pack.ymag_GET() == (short) -22105);
            assert(pack.zgyro_GET() == (short)21237);
            assert(pack.zmag_GET() == (short) -26932);
            assert(pack.zacc_GET() == (short)25744);
            assert(pack.time_boot_ms_GET() == 1778030204L);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.xmag_SET((short)30095) ;
        p116.zacc_SET((short)25744) ;
        p116.time_boot_ms_SET(1778030204L) ;
        p116.xgyro_SET((short)8596) ;
        p116.zmag_SET((short) -26932) ;
        p116.ymag_SET((short) -22105) ;
        p116.ygyro_SET((short)2273) ;
        p116.zgyro_SET((short)21237) ;
        p116.yacc_SET((short)2320) ;
        p116.xacc_SET((short)28459) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)139);
            assert(pack.end_GET() == (char)21091);
            assert(pack.start_GET() == (char)38806);
            assert(pack.target_component_GET() == (char)10);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)139) ;
        p117.target_component_SET((char)10) ;
        p117.start_SET((char)38806) ;
        p117.end_SET((char)21091) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.last_log_num_GET() == (char)17125);
            assert(pack.id_GET() == (char)37432);
            assert(pack.size_GET() == 3079178440L);
            assert(pack.num_logs_GET() == (char)19040);
            assert(pack.time_utc_GET() == 1855515953L);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.num_logs_SET((char)19040) ;
        p118.time_utc_SET(1855515953L) ;
        p118.size_SET(3079178440L) ;
        p118.last_log_num_SET((char)17125) ;
        p118.id_SET((char)37432) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)24);
            assert(pack.count_GET() == 623931621L);
            assert(pack.ofs_GET() == 2791770688L);
            assert(pack.id_GET() == (char)18127);
            assert(pack.target_component_GET() == (char)130);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_component_SET((char)130) ;
        p119.target_system_SET((char)24) ;
        p119.ofs_SET(2791770688L) ;
        p119.count_SET(623931621L) ;
        p119.id_SET((char)18127) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)116, (char)176, (char)209, (char)234, (char)5, (char)110, (char)244, (char)84, (char)112, (char)197, (char)81, (char)80, (char)198, (char)44, (char)204, (char)99, (char)123, (char)25, (char)164, (char)240, (char)20, (char)97, (char)71, (char)207, (char)82, (char)23, (char)74, (char)208, (char)198, (char)232, (char)152, (char)115, (char)173, (char)115, (char)211, (char)41, (char)151, (char)240, (char)146, (char)138, (char)64, (char)193, (char)39, (char)247, (char)56, (char)195, (char)72, (char)172, (char)174, (char)163, (char)74, (char)222, (char)242, (char)41, (char)251, (char)210, (char)9, (char)88, (char)213, (char)232, (char)159, (char)132, (char)115, (char)70, (char)60, (char)121, (char)230, (char)90, (char)132, (char)216, (char)22, (char)73, (char)60, (char)35, (char)127, (char)81, (char)143, (char)219, (char)251, (char)195, (char)230, (char)140, (char)109, (char)146, (char)150, (char)182, (char)187, (char)224, (char)101, (char)232}));
            assert(pack.id_GET() == (char)49389);
            assert(pack.count_GET() == (char)133);
            assert(pack.ofs_GET() == 3399926350L);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.ofs_SET(3399926350L) ;
        p120.data__SET(new char[] {(char)116, (char)176, (char)209, (char)234, (char)5, (char)110, (char)244, (char)84, (char)112, (char)197, (char)81, (char)80, (char)198, (char)44, (char)204, (char)99, (char)123, (char)25, (char)164, (char)240, (char)20, (char)97, (char)71, (char)207, (char)82, (char)23, (char)74, (char)208, (char)198, (char)232, (char)152, (char)115, (char)173, (char)115, (char)211, (char)41, (char)151, (char)240, (char)146, (char)138, (char)64, (char)193, (char)39, (char)247, (char)56, (char)195, (char)72, (char)172, (char)174, (char)163, (char)74, (char)222, (char)242, (char)41, (char)251, (char)210, (char)9, (char)88, (char)213, (char)232, (char)159, (char)132, (char)115, (char)70, (char)60, (char)121, (char)230, (char)90, (char)132, (char)216, (char)22, (char)73, (char)60, (char)35, (char)127, (char)81, (char)143, (char)219, (char)251, (char)195, (char)230, (char)140, (char)109, (char)146, (char)150, (char)182, (char)187, (char)224, (char)101, (char)232}, 0) ;
        p120.id_SET((char)49389) ;
        p120.count_SET((char)133) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)71);
            assert(pack.target_component_GET() == (char)157);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)157) ;
        p121.target_system_SET((char)71) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)43);
            assert(pack.target_component_GET() == (char)124);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)43) ;
        p122.target_component_SET((char)124) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)133);
            assert(pack.len_GET() == (char)68);
            assert(pack.target_component_GET() == (char)244);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)142, (char)239, (char)100, (char)184, (char)230, (char)191, (char)107, (char)62, (char)176, (char)147, (char)89, (char)196, (char)86, (char)74, (char)122, (char)248, (char)169, (char)66, (char)158, (char)167, (char)189, (char)235, (char)59, (char)112, (char)41, (char)232, (char)187, (char)243, (char)71, (char)61, (char)4, (char)179, (char)39, (char)132, (char)189, (char)242, (char)205, (char)148, (char)143, (char)97, (char)11, (char)240, (char)210, (char)55, (char)87, (char)197, (char)161, (char)60, (char)126, (char)80, (char)201, (char)124, (char)103, (char)63, (char)184, (char)84, (char)46, (char)104, (char)27, (char)140, (char)123, (char)181, (char)80, (char)247, (char)106, (char)246, (char)24, (char)7, (char)236, (char)178, (char)138, (char)43, (char)74, (char)113, (char)41, (char)229, (char)91, (char)9, (char)141, (char)15, (char)18, (char)161, (char)127, (char)242, (char)178, (char)60, (char)97, (char)140, (char)96, (char)179, (char)131, (char)132, (char)133, (char)111, (char)242, (char)60, (char)116, (char)154, (char)6, (char)39, (char)109, (char)231, (char)183, (char)164, (char)227, (char)105, (char)196, (char)56, (char)139, (char)174}));
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.data__SET(new char[] {(char)142, (char)239, (char)100, (char)184, (char)230, (char)191, (char)107, (char)62, (char)176, (char)147, (char)89, (char)196, (char)86, (char)74, (char)122, (char)248, (char)169, (char)66, (char)158, (char)167, (char)189, (char)235, (char)59, (char)112, (char)41, (char)232, (char)187, (char)243, (char)71, (char)61, (char)4, (char)179, (char)39, (char)132, (char)189, (char)242, (char)205, (char)148, (char)143, (char)97, (char)11, (char)240, (char)210, (char)55, (char)87, (char)197, (char)161, (char)60, (char)126, (char)80, (char)201, (char)124, (char)103, (char)63, (char)184, (char)84, (char)46, (char)104, (char)27, (char)140, (char)123, (char)181, (char)80, (char)247, (char)106, (char)246, (char)24, (char)7, (char)236, (char)178, (char)138, (char)43, (char)74, (char)113, (char)41, (char)229, (char)91, (char)9, (char)141, (char)15, (char)18, (char)161, (char)127, (char)242, (char)178, (char)60, (char)97, (char)140, (char)96, (char)179, (char)131, (char)132, (char)133, (char)111, (char)242, (char)60, (char)116, (char)154, (char)6, (char)39, (char)109, (char)231, (char)183, (char)164, (char)227, (char)105, (char)196, (char)56, (char)139, (char)174}, 0) ;
        p123.len_SET((char)68) ;
        p123.target_system_SET((char)133) ;
        p123.target_component_SET((char)244) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.dgps_numch_GET() == (char)21);
            assert(pack.eph_GET() == (char)49015);
            assert(pack.alt_GET() == 2089365460);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.vel_GET() == (char)60167);
            assert(pack.dgps_age_GET() == 3933673652L);
            assert(pack.epv_GET() == (char)12025);
            assert(pack.lat_GET() == 478970232);
            assert(pack.satellites_visible_GET() == (char)223);
            assert(pack.time_usec_GET() == 8716483026046290781L);
            assert(pack.lon_GET() == -178538475);
            assert(pack.cog_GET() == (char)54962);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p124.eph_SET((char)49015) ;
        p124.dgps_numch_SET((char)21) ;
        p124.dgps_age_SET(3933673652L) ;
        p124.cog_SET((char)54962) ;
        p124.time_usec_SET(8716483026046290781L) ;
        p124.lon_SET(-178538475) ;
        p124.lat_SET(478970232) ;
        p124.epv_SET((char)12025) ;
        p124.alt_SET(2089365460) ;
        p124.satellites_visible_SET((char)223) ;
        p124.vel_SET((char)60167) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vservo_GET() == (char)61170);
            assert(pack.Vcc_GET() == (char)31890);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID));
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)61170) ;
        p125.Vcc_SET((char)31890) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID)) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)12, (char)118, (char)47, (char)103, (char)124, (char)222, (char)13, (char)190, (char)166, (char)209, (char)183, (char)202, (char)3, (char)75, (char)68, (char)15, (char)229, (char)201, (char)221, (char)185, (char)215, (char)252, (char)236, (char)99, (char)248, (char)107, (char)231, (char)71, (char)203, (char)17, (char)98, (char)31, (char)120, (char)212, (char)104, (char)22, (char)143, (char)156, (char)219, (char)165, (char)207, (char)233, (char)135, (char)251, (char)21, (char)142, (char)114, (char)5, (char)188, (char)131, (char)215, (char)201, (char)16, (char)132, (char)45, (char)248, (char)179, (char)73, (char)35, (char)98, (char)233, (char)130, (char)141, (char)12, (char)149, (char)3, (char)173, (char)123, (char)41, (char)120}));
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
            assert(pack.count_GET() == (char)251);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI));
            assert(pack.baudrate_GET() == 3531012477L);
            assert(pack.timeout_GET() == (char)60379);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI)) ;
        p126.baudrate_SET(3531012477L) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1) ;
        p126.timeout_SET((char)60379) ;
        p126.data__SET(new char[] {(char)12, (char)118, (char)47, (char)103, (char)124, (char)222, (char)13, (char)190, (char)166, (char)209, (char)183, (char)202, (char)3, (char)75, (char)68, (char)15, (char)229, (char)201, (char)221, (char)185, (char)215, (char)252, (char)236, (char)99, (char)248, (char)107, (char)231, (char)71, (char)203, (char)17, (char)98, (char)31, (char)120, (char)212, (char)104, (char)22, (char)143, (char)156, (char)219, (char)165, (char)207, (char)233, (char)135, (char)251, (char)21, (char)142, (char)114, (char)5, (char)188, (char)131, (char)215, (char)201, (char)16, (char)132, (char)45, (char)248, (char)179, (char)73, (char)35, (char)98, (char)233, (char)130, (char)141, (char)12, (char)149, (char)3, (char)173, (char)123, (char)41, (char)120}, 0) ;
        p126.count_SET((char)251) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_a_mm_GET() == 1542243756);
            assert(pack.iar_num_hypotheses_GET() == 110948425);
            assert(pack.nsats_GET() == (char)70);
            assert(pack.baseline_c_mm_GET() == -2014650756);
            assert(pack.wn_GET() == (char)37365);
            assert(pack.rtk_rate_GET() == (char)28);
            assert(pack.rtk_health_GET() == (char)146);
            assert(pack.baseline_coords_type_GET() == (char)136);
            assert(pack.tow_GET() == 3504970539L);
            assert(pack.accuracy_GET() == 3139378457L);
            assert(pack.time_last_baseline_ms_GET() == 2777246018L);
            assert(pack.rtk_receiver_id_GET() == (char)24);
            assert(pack.baseline_b_mm_GET() == -1329008775);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.tow_SET(3504970539L) ;
        p127.accuracy_SET(3139378457L) ;
        p127.baseline_a_mm_SET(1542243756) ;
        p127.rtk_health_SET((char)146) ;
        p127.wn_SET((char)37365) ;
        p127.iar_num_hypotheses_SET(110948425) ;
        p127.rtk_receiver_id_SET((char)24) ;
        p127.nsats_SET((char)70) ;
        p127.baseline_b_mm_SET(-1329008775) ;
        p127.time_last_baseline_ms_SET(2777246018L) ;
        p127.baseline_coords_type_SET((char)136) ;
        p127.baseline_c_mm_SET(-2014650756) ;
        p127.rtk_rate_SET((char)28) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_c_mm_GET() == -460270817);
            assert(pack.baseline_coords_type_GET() == (char)8);
            assert(pack.accuracy_GET() == 2341531476L);
            assert(pack.rtk_health_GET() == (char)195);
            assert(pack.baseline_b_mm_GET() == 1955965304);
            assert(pack.wn_GET() == (char)51316);
            assert(pack.iar_num_hypotheses_GET() == -1692030301);
            assert(pack.tow_GET() == 1301216005L);
            assert(pack.rtk_receiver_id_GET() == (char)5);
            assert(pack.rtk_rate_GET() == (char)139);
            assert(pack.nsats_GET() == (char)86);
            assert(pack.baseline_a_mm_GET() == 280944156);
            assert(pack.time_last_baseline_ms_GET() == 1576841259L);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.accuracy_SET(2341531476L) ;
        p128.baseline_coords_type_SET((char)8) ;
        p128.time_last_baseline_ms_SET(1576841259L) ;
        p128.rtk_receiver_id_SET((char)5) ;
        p128.tow_SET(1301216005L) ;
        p128.rtk_health_SET((char)195) ;
        p128.rtk_rate_SET((char)139) ;
        p128.iar_num_hypotheses_SET(-1692030301) ;
        p128.wn_SET((char)51316) ;
        p128.baseline_c_mm_SET(-460270817) ;
        p128.nsats_SET((char)86) ;
        p128.baseline_a_mm_SET(280944156) ;
        p128.baseline_b_mm_SET(1955965304) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == (short)1561);
            assert(pack.yacc_GET() == (short)12929);
            assert(pack.zmag_GET() == (short)22857);
            assert(pack.xgyro_GET() == (short)14574);
            assert(pack.xacc_GET() == (short)21854);
            assert(pack.zacc_GET() == (short) -7074);
            assert(pack.zgyro_GET() == (short)2950);
            assert(pack.time_boot_ms_GET() == 2382660312L);
            assert(pack.ygyro_GET() == (short)17154);
            assert(pack.xmag_GET() == (short) -27526);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.xacc_SET((short)21854) ;
        p129.xgyro_SET((short)14574) ;
        p129.ygyro_SET((short)17154) ;
        p129.ymag_SET((short)1561) ;
        p129.zmag_SET((short)22857) ;
        p129.zgyro_SET((short)2950) ;
        p129.time_boot_ms_SET(2382660312L) ;
        p129.zacc_SET((short) -7074) ;
        p129.xmag_SET((short) -27526) ;
        p129.yacc_SET((short)12929) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.height_GET() == (char)64215);
            assert(pack.jpg_quality_GET() == (char)246);
            assert(pack.type_GET() == (char)212);
            assert(pack.size_GET() == 2025543696L);
            assert(pack.width_GET() == (char)55789);
            assert(pack.payload_GET() == (char)195);
            assert(pack.packets_GET() == (char)718);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.payload_SET((char)195) ;
        p130.jpg_quality_SET((char)246) ;
        p130.width_SET((char)55789) ;
        p130.type_SET((char)212) ;
        p130.size_SET(2025543696L) ;
        p130.packets_SET((char)718) ;
        p130.height_SET((char)64215) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)248, (char)105, (char)212, (char)20, (char)201, (char)1, (char)141, (char)51, (char)103, (char)243, (char)240, (char)133, (char)224, (char)184, (char)53, (char)154, (char)99, (char)8, (char)49, (char)19, (char)171, (char)96, (char)122, (char)63, (char)252, (char)95, (char)158, (char)112, (char)39, (char)67, (char)37, (char)134, (char)154, (char)72, (char)227, (char)53, (char)39, (char)235, (char)87, (char)104, (char)23, (char)157, (char)125, (char)141, (char)187, (char)224, (char)120, (char)48, (char)143, (char)245, (char)141, (char)63, (char)243, (char)186, (char)9, (char)158, (char)36, (char)81, (char)67, (char)157, (char)33, (char)60, (char)205, (char)28, (char)107, (char)36, (char)72, (char)173, (char)124, (char)28, (char)231, (char)252, (char)21, (char)240, (char)166, (char)202, (char)105, (char)9, (char)92, (char)70, (char)11, (char)62, (char)91, (char)249, (char)189, (char)10, (char)138, (char)191, (char)226, (char)3, (char)206, (char)58, (char)225, (char)247, (char)59, (char)52, (char)229, (char)114, (char)133, (char)126, (char)37, (char)103, (char)182, (char)127, (char)125, (char)38, (char)223, (char)156, (char)21, (char)241, (char)191, (char)177, (char)54, (char)70, (char)68, (char)186, (char)239, (char)68, (char)120, (char)33, (char)134, (char)23, (char)75, (char)121, (char)134, (char)179, (char)155, (char)5, (char)174, (char)178, (char)25, (char)86, (char)0, (char)239, (char)184, (char)246, (char)209, (char)197, (char)152, (char)3, (char)251, (char)140, (char)41, (char)186, (char)181, (char)130, (char)133, (char)175, (char)6, (char)254, (char)72, (char)6, (char)77, (char)184, (char)64, (char)238, (char)199, (char)205, (char)70, (char)108, (char)233, (char)225, (char)251, (char)122, (char)119, (char)146, (char)99, (char)29, (char)218, (char)233, (char)22, (char)235, (char)183, (char)4, (char)1, (char)125, (char)13, (char)87, (char)147, (char)163, (char)42, (char)20, (char)84, (char)43, (char)149, (char)190, (char)131, (char)67, (char)203, (char)169, (char)222, (char)30, (char)153, (char)242, (char)145, (char)41, (char)97, (char)103, (char)67, (char)60, (char)120, (char)134, (char)255, (char)149, (char)85, (char)90, (char)246, (char)255, (char)74, (char)53, (char)6, (char)79, (char)161, (char)217, (char)112, (char)234, (char)51, (char)251, (char)49, (char)95, (char)70, (char)85, (char)32, (char)217, (char)114, (char)60, (char)23, (char)84, (char)221, (char)78, (char)132, (char)58, (char)167, (char)119, (char)206, (char)40, (char)22, (char)249, (char)87, (char)238, (char)6, (char)34, (char)100, (char)102, (char)6, (char)221, (char)37, (char)33, (char)241, (char)91, (char)139, (char)101, (char)204}));
            assert(pack.seqnr_GET() == (char)2321);
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.data__SET(new char[] {(char)248, (char)105, (char)212, (char)20, (char)201, (char)1, (char)141, (char)51, (char)103, (char)243, (char)240, (char)133, (char)224, (char)184, (char)53, (char)154, (char)99, (char)8, (char)49, (char)19, (char)171, (char)96, (char)122, (char)63, (char)252, (char)95, (char)158, (char)112, (char)39, (char)67, (char)37, (char)134, (char)154, (char)72, (char)227, (char)53, (char)39, (char)235, (char)87, (char)104, (char)23, (char)157, (char)125, (char)141, (char)187, (char)224, (char)120, (char)48, (char)143, (char)245, (char)141, (char)63, (char)243, (char)186, (char)9, (char)158, (char)36, (char)81, (char)67, (char)157, (char)33, (char)60, (char)205, (char)28, (char)107, (char)36, (char)72, (char)173, (char)124, (char)28, (char)231, (char)252, (char)21, (char)240, (char)166, (char)202, (char)105, (char)9, (char)92, (char)70, (char)11, (char)62, (char)91, (char)249, (char)189, (char)10, (char)138, (char)191, (char)226, (char)3, (char)206, (char)58, (char)225, (char)247, (char)59, (char)52, (char)229, (char)114, (char)133, (char)126, (char)37, (char)103, (char)182, (char)127, (char)125, (char)38, (char)223, (char)156, (char)21, (char)241, (char)191, (char)177, (char)54, (char)70, (char)68, (char)186, (char)239, (char)68, (char)120, (char)33, (char)134, (char)23, (char)75, (char)121, (char)134, (char)179, (char)155, (char)5, (char)174, (char)178, (char)25, (char)86, (char)0, (char)239, (char)184, (char)246, (char)209, (char)197, (char)152, (char)3, (char)251, (char)140, (char)41, (char)186, (char)181, (char)130, (char)133, (char)175, (char)6, (char)254, (char)72, (char)6, (char)77, (char)184, (char)64, (char)238, (char)199, (char)205, (char)70, (char)108, (char)233, (char)225, (char)251, (char)122, (char)119, (char)146, (char)99, (char)29, (char)218, (char)233, (char)22, (char)235, (char)183, (char)4, (char)1, (char)125, (char)13, (char)87, (char)147, (char)163, (char)42, (char)20, (char)84, (char)43, (char)149, (char)190, (char)131, (char)67, (char)203, (char)169, (char)222, (char)30, (char)153, (char)242, (char)145, (char)41, (char)97, (char)103, (char)67, (char)60, (char)120, (char)134, (char)255, (char)149, (char)85, (char)90, (char)246, (char)255, (char)74, (char)53, (char)6, (char)79, (char)161, (char)217, (char)112, (char)234, (char)51, (char)251, (char)49, (char)95, (char)70, (char)85, (char)32, (char)217, (char)114, (char)60, (char)23, (char)84, (char)221, (char)78, (char)132, (char)58, (char)167, (char)119, (char)206, (char)40, (char)22, (char)249, (char)87, (char)238, (char)6, (char)34, (char)100, (char)102, (char)6, (char)221, (char)37, (char)33, (char)241, (char)91, (char)139, (char)101, (char)204}, 0) ;
        p131.seqnr_SET((char)2321) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.covariance_GET() == (char)9);
            assert(pack.id_GET() == (char)10);
            assert(pack.min_distance_GET() == (char)11855);
            assert(pack.max_distance_GET() == (char)1352);
            assert(pack.current_distance_GET() == (char)47658);
            assert(pack.time_boot_ms_GET() == 2306380213L);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_180);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.current_distance_SET((char)47658) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_180) ;
        p132.time_boot_ms_SET(2306380213L) ;
        p132.id_SET((char)10) ;
        p132.min_distance_SET((char)11855) ;
        p132.covariance_SET((char)9) ;
        p132.max_distance_SET((char)1352) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 129251930);
            assert(pack.grid_spacing_GET() == (char)34489);
            assert(pack.lat_GET() == -1448890257);
            assert(pack.mask_GET() == 5280765450594851166L);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(-1448890257) ;
        p133.mask_SET(5280765450594851166L) ;
        p133.lon_SET(129251930) ;
        p133.grid_spacing_SET((char)34489) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)4199, (short) -8774, (short) -20426, (short)13067, (short) -16797, (short) -7453, (short)11233, (short)29632, (short) -6923, (short)27031, (short)16958, (short)26839, (short)10160, (short) -30005, (short)252, (short) -20713}));
            assert(pack.lon_GET() == 1992020475);
            assert(pack.grid_spacing_GET() == (char)62503);
            assert(pack.gridbit_GET() == (char)216);
            assert(pack.lat_GET() == 1955073006);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.grid_spacing_SET((char)62503) ;
        p134.data__SET(new short[] {(short)4199, (short) -8774, (short) -20426, (short)13067, (short) -16797, (short) -7453, (short)11233, (short)29632, (short) -6923, (short)27031, (short)16958, (short)26839, (short)10160, (short) -30005, (short)252, (short) -20713}, 0) ;
        p134.gridbit_SET((char)216) ;
        p134.lat_SET(1955073006) ;
        p134.lon_SET(1992020475) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 1750756929);
            assert(pack.lat_GET() == 1600938820);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(1750756929) ;
        p135.lat_SET(1600938820) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.terrain_height_GET() == 1.3569741E38F);
            assert(pack.loaded_GET() == (char)55237);
            assert(pack.lat_GET() == -1649105727);
            assert(pack.lon_GET() == 628505394);
            assert(pack.spacing_GET() == (char)36691);
            assert(pack.pending_GET() == (char)62179);
            assert(pack.current_height_GET() == 1.0889474E36F);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lon_SET(628505394) ;
        p136.pending_SET((char)62179) ;
        p136.terrain_height_SET(1.3569741E38F) ;
        p136.lat_SET(-1649105727) ;
        p136.spacing_SET((char)36691) ;
        p136.current_height_SET(1.0889474E36F) ;
        p136.loaded_SET((char)55237) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1888664079L);
            assert(pack.temperature_GET() == (short) -25404);
            assert(pack.press_abs_GET() == -1.7628703E37F);
            assert(pack.press_diff_GET() == -2.3847885E38F);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.temperature_SET((short) -25404) ;
        p137.press_diff_SET(-2.3847885E38F) ;
        p137.time_boot_ms_SET(1888664079L) ;
        p137.press_abs_SET(-1.7628703E37F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6520166234207834393L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.0087681E38F, -1.400362E38F, 6.2640316E37F, 7.554969E37F}));
            assert(pack.y_GET() == -9.934757E37F);
            assert(pack.z_GET() == -3.3155406E38F);
            assert(pack.x_GET() == -3.131577E38F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.x_SET(-3.131577E38F) ;
        p138.time_usec_SET(6520166234207834393L) ;
        p138.y_SET(-9.934757E37F) ;
        p138.z_SET(-3.3155406E38F) ;
        p138.q_SET(new float[] {-1.0087681E38F, -1.400362E38F, 6.2640316E37F, 7.554969E37F}, 0) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {4.229754E37F, -2.8526246E37F, -2.3640698E38F, -8.927082E37F, -1.2893439E37F, -3.4753536E37F, 1.7937063E38F, -1.1064238E38F}));
            assert(pack.time_usec_GET() == 6846897202882491831L);
            assert(pack.group_mlx_GET() == (char)91);
            assert(pack.target_system_GET() == (char)239);
            assert(pack.target_component_GET() == (char)46);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(6846897202882491831L) ;
        p139.target_component_SET((char)46) ;
        p139.target_system_SET((char)239) ;
        p139.group_mlx_SET((char)91) ;
        p139.controls_SET(new float[] {4.229754E37F, -2.8526246E37F, -2.3640698E38F, -8.927082E37F, -1.2893439E37F, -3.4753536E37F, 1.7937063E38F, -1.1064238E38F}, 0) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.4824554E38F, -2.00372E38F, 3.5709378E37F, -4.633374E37F, -1.740274E38F, -8.3123374E37F, 1.5978731E38F, 1.8505837E38F}));
            assert(pack.group_mlx_GET() == (char)56);
            assert(pack.time_usec_GET() == 3064416480114222793L);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)56) ;
        p140.controls_SET(new float[] {2.4824554E38F, -2.00372E38F, 3.5709378E37F, -4.633374E37F, -1.740274E38F, -8.3123374E37F, 1.5978731E38F, 1.8505837E38F}, 0) ;
        p140.time_usec_SET(3064416480114222793L) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2446628050814867536L);
            assert(pack.altitude_amsl_GET() == -1.8864393E38F);
            assert(pack.altitude_local_GET() == -1.3579722E38F);
            assert(pack.bottom_clearance_GET() == -2.9021126E38F);
            assert(pack.altitude_terrain_GET() == -1.4348402E38F);
            assert(pack.altitude_monotonic_GET() == -7.1801094E37F);
            assert(pack.altitude_relative_GET() == -8.3133044E37F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_local_SET(-1.3579722E38F) ;
        p141.altitude_relative_SET(-8.3133044E37F) ;
        p141.altitude_terrain_SET(-1.4348402E38F) ;
        p141.altitude_amsl_SET(-1.8864393E38F) ;
        p141.time_usec_SET(2446628050814867536L) ;
        p141.altitude_monotonic_SET(-7.1801094E37F) ;
        p141.bottom_clearance_SET(-2.9021126E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)85, (char)119, (char)67, (char)147, (char)200, (char)177, (char)196, (char)251, (char)221, (char)110, (char)119, (char)231, (char)26, (char)46, (char)235, (char)224, (char)56, (char)175, (char)93, (char)115, (char)15, (char)102, (char)141, (char)148, (char)32, (char)87, (char)82, (char)168, (char)133, (char)206, (char)177, (char)8, (char)66, (char)70, (char)81, (char)169, (char)228, (char)207, (char)248, (char)220, (char)157, (char)144, (char)49, (char)177, (char)97, (char)156, (char)94, (char)125, (char)28, (char)174, (char)208, (char)141, (char)195, (char)255, (char)154, (char)216, (char)63, (char)66, (char)16, (char)113, (char)41, (char)12, (char)149, (char)44, (char)167, (char)70, (char)75, (char)216, (char)62, (char)214, (char)227, (char)152, (char)1, (char)63, (char)208, (char)189, (char)230, (char)176, (char)9, (char)41, (char)19, (char)116, (char)29, (char)72, (char)45, (char)33, (char)172, (char)76, (char)46, (char)115, (char)244, (char)219, (char)252, (char)137, (char)121, (char)145, (char)229, (char)130, (char)112, (char)58, (char)172, (char)45, (char)217, (char)240, (char)195, (char)115, (char)77, (char)176, (char)82, (char)115, (char)132, (char)47, (char)244, (char)225, (char)113, (char)182, (char)116, (char)185, (char)185, (char)73}));
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)19, (char)144, (char)147, (char)183, (char)183, (char)144, (char)151, (char)72, (char)69, (char)114, (char)51, (char)46, (char)81, (char)154, (char)87, (char)113, (char)38, (char)135, (char)35, (char)80, (char)232, (char)51, (char)244, (char)33, (char)11, (char)153, (char)172, (char)179, (char)126, (char)187, (char)46, (char)143, (char)203, (char)103, (char)164, (char)104, (char)242, (char)32, (char)47, (char)151, (char)220, (char)93, (char)27, (char)34, (char)197, (char)247, (char)51, (char)104, (char)100, (char)98, (char)6, (char)121, (char)154, (char)154, (char)147, (char)38, (char)249, (char)195, (char)206, (char)224, (char)146, (char)244, (char)180, (char)186, (char)127, (char)12, (char)158, (char)118, (char)141, (char)79, (char)192, (char)186, (char)125, (char)129, (char)131, (char)162, (char)212, (char)72, (char)53, (char)35, (char)47, (char)137, (char)120, (char)171, (char)32, (char)179, (char)163, (char)92, (char)132, (char)212, (char)204, (char)159, (char)97, (char)214, (char)227, (char)215, (char)127, (char)107, (char)222, (char)209, (char)176, (char)132, (char)19, (char)23, (char)209, (char)84, (char)222, (char)235, (char)242, (char)19, (char)169, (char)149, (char)42, (char)125, (char)135, (char)221, (char)255, (char)195, (char)45, (char)201}));
            assert(pack.transfer_type_GET() == (char)238);
            assert(pack.request_id_GET() == (char)22);
            assert(pack.uri_type_GET() == (char)211);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.storage_SET(new char[] {(char)85, (char)119, (char)67, (char)147, (char)200, (char)177, (char)196, (char)251, (char)221, (char)110, (char)119, (char)231, (char)26, (char)46, (char)235, (char)224, (char)56, (char)175, (char)93, (char)115, (char)15, (char)102, (char)141, (char)148, (char)32, (char)87, (char)82, (char)168, (char)133, (char)206, (char)177, (char)8, (char)66, (char)70, (char)81, (char)169, (char)228, (char)207, (char)248, (char)220, (char)157, (char)144, (char)49, (char)177, (char)97, (char)156, (char)94, (char)125, (char)28, (char)174, (char)208, (char)141, (char)195, (char)255, (char)154, (char)216, (char)63, (char)66, (char)16, (char)113, (char)41, (char)12, (char)149, (char)44, (char)167, (char)70, (char)75, (char)216, (char)62, (char)214, (char)227, (char)152, (char)1, (char)63, (char)208, (char)189, (char)230, (char)176, (char)9, (char)41, (char)19, (char)116, (char)29, (char)72, (char)45, (char)33, (char)172, (char)76, (char)46, (char)115, (char)244, (char)219, (char)252, (char)137, (char)121, (char)145, (char)229, (char)130, (char)112, (char)58, (char)172, (char)45, (char)217, (char)240, (char)195, (char)115, (char)77, (char)176, (char)82, (char)115, (char)132, (char)47, (char)244, (char)225, (char)113, (char)182, (char)116, (char)185, (char)185, (char)73}, 0) ;
        p142.uri_SET(new char[] {(char)19, (char)144, (char)147, (char)183, (char)183, (char)144, (char)151, (char)72, (char)69, (char)114, (char)51, (char)46, (char)81, (char)154, (char)87, (char)113, (char)38, (char)135, (char)35, (char)80, (char)232, (char)51, (char)244, (char)33, (char)11, (char)153, (char)172, (char)179, (char)126, (char)187, (char)46, (char)143, (char)203, (char)103, (char)164, (char)104, (char)242, (char)32, (char)47, (char)151, (char)220, (char)93, (char)27, (char)34, (char)197, (char)247, (char)51, (char)104, (char)100, (char)98, (char)6, (char)121, (char)154, (char)154, (char)147, (char)38, (char)249, (char)195, (char)206, (char)224, (char)146, (char)244, (char)180, (char)186, (char)127, (char)12, (char)158, (char)118, (char)141, (char)79, (char)192, (char)186, (char)125, (char)129, (char)131, (char)162, (char)212, (char)72, (char)53, (char)35, (char)47, (char)137, (char)120, (char)171, (char)32, (char)179, (char)163, (char)92, (char)132, (char)212, (char)204, (char)159, (char)97, (char)214, (char)227, (char)215, (char)127, (char)107, (char)222, (char)209, (char)176, (char)132, (char)19, (char)23, (char)209, (char)84, (char)222, (char)235, (char)242, (char)19, (char)169, (char)149, (char)42, (char)125, (char)135, (char)221, (char)255, (char)195, (char)45, (char)201}, 0) ;
        p142.transfer_type_SET((char)238) ;
        p142.uri_type_SET((char)211) ;
        p142.request_id_SET((char)22) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == -2.8932442E38F);
            assert(pack.press_abs_GET() == -1.59388E37F);
            assert(pack.time_boot_ms_GET() == 4293605124L);
            assert(pack.temperature_GET() == (short) -15570);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.press_abs_SET(-1.59388E37F) ;
        p143.time_boot_ms_SET(4293605124L) ;
        p143.press_diff_SET(-2.8932442E38F) ;
        p143.temperature_SET((short) -15570) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.timestamp_GET() == 2160199515834250000L);
            assert(pack.lon_GET() == 646327212);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {1.0552917E38F, 3.0306792E38F, 2.4355464E38F}));
            assert(pack.lat_GET() == 1103391433);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-3.0792547E38F, 1.7794061E37F, 2.4007383E37F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {3.1145555E38F, -2.1566351E38F, 2.1136108E38F, -3.30757E38F}));
            assert(pack.alt_GET() == -1.2523596E38F);
            assert(pack.est_capabilities_GET() == (char)217);
            assert(pack.custom_state_GET() == 874254097415564131L);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {1.2121337E38F, 3.3239099E38F, -3.238603E38F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {2.6403333E38F, -1.9782276E38F, 5.532612E36F}));
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.attitude_q_SET(new float[] {3.1145555E38F, -2.1566351E38F, 2.1136108E38F, -3.30757E38F}, 0) ;
        p144.lat_SET(1103391433) ;
        p144.position_cov_SET(new float[] {1.2121337E38F, 3.3239099E38F, -3.238603E38F}, 0) ;
        p144.est_capabilities_SET((char)217) ;
        p144.alt_SET(-1.2523596E38F) ;
        p144.custom_state_SET(874254097415564131L) ;
        p144.acc_SET(new float[] {-3.0792547E38F, 1.7794061E37F, 2.4007383E37F}, 0) ;
        p144.rates_SET(new float[] {2.6403333E38F, -1.9782276E38F, 5.532612E36F}, 0) ;
        p144.timestamp_SET(2160199515834250000L) ;
        p144.vel_SET(new float[] {1.0552917E38F, 3.0306792E38F, 2.4355464E38F}, 0) ;
        p144.lon_SET(646327212) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.x_pos_GET() == -2.6267006E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {6.5752225E37F, 4.508039E37F, 8.254326E37F}));
            assert(pack.z_pos_GET() == -2.6674912E38F);
            assert(pack.z_acc_GET() == -2.5703518E38F);
            assert(pack.time_usec_GET() == 418392178431305465L);
            assert(pack.roll_rate_GET() == -2.5882685E38F);
            assert(pack.x_vel_GET() == -2.1010817E37F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {4.866002E37F, -2.1224199E38F, 2.3491437E37F}));
            assert(pack.y_vel_GET() == -3.1488646E38F);
            assert(pack.z_vel_GET() == 5.6390854E37F);
            assert(pack.y_pos_GET() == -1.6287404E38F);
            assert(pack.yaw_rate_GET() == -1.5768177E37F);
            assert(pack.airspeed_GET() == 2.9702636E38F);
            assert(pack.y_acc_GET() == 6.48792E37F);
            assert(pack.pitch_rate_GET() == -2.3924824E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.389267E38F, -2.8157428E38F, -1.5024009E38F, -2.5961403E38F}));
            assert(pack.x_acc_GET() == 1.1070136E38F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.y_acc_SET(6.48792E37F) ;
        p146.x_vel_SET(-2.1010817E37F) ;
        p146.z_pos_SET(-2.6674912E38F) ;
        p146.z_acc_SET(-2.5703518E38F) ;
        p146.vel_variance_SET(new float[] {4.866002E37F, -2.1224199E38F, 2.3491437E37F}, 0) ;
        p146.z_vel_SET(5.6390854E37F) ;
        p146.q_SET(new float[] {-3.389267E38F, -2.8157428E38F, -1.5024009E38F, -2.5961403E38F}, 0) ;
        p146.pos_variance_SET(new float[] {6.5752225E37F, 4.508039E37F, 8.254326E37F}, 0) ;
        p146.roll_rate_SET(-2.5882685E38F) ;
        p146.y_pos_SET(-1.6287404E38F) ;
        p146.yaw_rate_SET(-1.5768177E37F) ;
        p146.time_usec_SET(418392178431305465L) ;
        p146.x_pos_SET(-2.6267006E38F) ;
        p146.x_acc_SET(1.1070136E38F) ;
        p146.airspeed_SET(2.9702636E38F) ;
        p146.y_vel_SET(-3.1488646E38F) ;
        p146.pitch_rate_SET(-2.3924824E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -13623);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
            assert(pack.energy_consumed_GET() == 1872020099);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
            assert(pack.current_battery_GET() == (short) -8971);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)64670, (char)45932, (char)64436, (char)50229, (char)61111, (char)59638, (char)18392, (char)36061, (char)9720, (char)34811}));
            assert(pack.id_GET() == (char)162);
            assert(pack.current_consumed_GET() == 1665399894);
            assert(pack.battery_remaining_GET() == (byte) - 3);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.temperature_SET((short) -13623) ;
        p147.current_consumed_SET(1665399894) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN) ;
        p147.voltages_SET(new char[] {(char)64670, (char)45932, (char)64436, (char)50229, (char)61111, (char)59638, (char)18392, (char)36061, (char)9720, (char)34811}, 0) ;
        p147.id_SET((char)162) ;
        p147.energy_consumed_SET(1872020099) ;
        p147.battery_remaining_SET((byte) - 3) ;
        p147.current_battery_SET((short) -8971) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.product_id_GET() == (char)61052);
            assert(pack.middleware_sw_version_GET() == 3583214801L);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)6, (char)170, (char)19, (char)100, (char)216, (char)61, (char)144, (char)121}));
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)235, (char)98, (char)80, (char)44, (char)103, (char)42, (char)57, (char)57, (char)228, (char)84, (char)168, (char)230, (char)65, (char)2, (char)104, (char)216, (char)87, (char)227}));
            assert(pack.flight_sw_version_GET() == 2237954448L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)80, (char)75, (char)97, (char)76, (char)174, (char)112, (char)249, (char)13}));
            assert(pack.os_sw_version_GET() == 1262275960L);
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)212, (char)76, (char)202, (char)7, (char)21, (char)55, (char)12, (char)227}));
            assert(pack.uid_GET() == 8470779405973373467L);
            assert(pack.board_version_GET() == 4202260713L);
            assert(pack.vendor_id_GET() == (char)31442);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.uid2_SET(new char[] {(char)235, (char)98, (char)80, (char)44, (char)103, (char)42, (char)57, (char)57, (char)228, (char)84, (char)168, (char)230, (char)65, (char)2, (char)104, (char)216, (char)87, (char)227}, 0, PH) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE)) ;
        p148.middleware_custom_version_SET(new char[] {(char)6, (char)170, (char)19, (char)100, (char)216, (char)61, (char)144, (char)121}, 0) ;
        p148.os_sw_version_SET(1262275960L) ;
        p148.vendor_id_SET((char)31442) ;
        p148.board_version_SET(4202260713L) ;
        p148.middleware_sw_version_SET(3583214801L) ;
        p148.product_id_SET((char)61052) ;
        p148.flight_sw_version_SET(2237954448L) ;
        p148.uid_SET(8470779405973373467L) ;
        p148.os_custom_version_SET(new char[] {(char)212, (char)76, (char)202, (char)7, (char)21, (char)55, (char)12, (char)227}, 0) ;
        p148.flight_custom_version_SET(new char[] {(char)80, (char)75, (char)97, (char)76, (char)174, (char)112, (char)249, (char)13}, 0) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
            assert(pack.angle_y_GET() == 1.9551612E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.time_usec_GET() == 8432024784433054324L);
            assert(pack.angle_x_GET() == 2.3009258E38F);
            assert(pack.position_valid_TRY(ph) == (char)52);
            assert(pack.distance_GET() == 2.0714979E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {1.01092E38F, -1.466432E38F, 1.908876E38F, -3.3772419E38F}));
            assert(pack.y_TRY(ph) == -3.3285732E38F);
            assert(pack.x_TRY(ph) == -1.5709212E38F);
            assert(pack.target_num_GET() == (char)84);
            assert(pack.z_TRY(ph) == -2.4001206E38F);
            assert(pack.size_y_GET() == -8.193212E37F);
            assert(pack.size_x_GET() == 2.3434414E38F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.distance_SET(2.0714979E38F) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON) ;
        p149.time_usec_SET(8432024784433054324L) ;
        p149.q_SET(new float[] {1.01092E38F, -1.466432E38F, 1.908876E38F, -3.3772419E38F}, 0, PH) ;
        p149.x_SET(-1.5709212E38F, PH) ;
        p149.angle_x_SET(2.3009258E38F) ;
        p149.z_SET(-2.4001206E38F, PH) ;
        p149.size_x_SET(2.3434414E38F) ;
        p149.size_y_SET(-8.193212E37F) ;
        p149.angle_y_SET(1.9551612E38F) ;
        p149.position_valid_SET((char)52, PH) ;
        p149.target_num_SET((char)84) ;
        p149.y_SET(-3.3285732E38F, PH) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCRIPT_ITEM.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)93);
            assert(pack.seq_GET() == (char)64518);
            assert(pack.name_LEN(ph) == 18);
            assert(pack.name_TRY(ph).equals("npnBrdbrvdtGjufkaq"));
            assert(pack.target_system_GET() == (char)135);
        });
        GroundControl.SCRIPT_ITEM p180 = CommunicationChannel.new_SCRIPT_ITEM();
        PH.setPack(p180);
        p180.name_SET("npnBrdbrvdtGjufkaq", PH) ;
        p180.seq_SET((char)64518) ;
        p180.target_system_SET((char)135) ;
        p180.target_component_SET((char)93) ;
        CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCRIPT_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)40);
            assert(pack.seq_GET() == (char)1261);
            assert(pack.target_component_GET() == (char)11);
        });
        GroundControl.SCRIPT_REQUEST p181 = CommunicationChannel.new_SCRIPT_REQUEST();
        PH.setPack(p181);
        p181.target_system_SET((char)40) ;
        p181.seq_SET((char)1261) ;
        p181.target_component_SET((char)11) ;
        CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCRIPT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)20);
            assert(pack.target_component_GET() == (char)96);
        });
        GroundControl.SCRIPT_REQUEST_LIST p182 = CommunicationChannel.new_SCRIPT_REQUEST_LIST();
        PH.setPack(p182);
        p182.target_component_SET((char)96) ;
        p182.target_system_SET((char)20) ;
        CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCRIPT_COUNT.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)40526);
            assert(pack.target_system_GET() == (char)165);
            assert(pack.target_component_GET() == (char)171);
        });
        GroundControl.SCRIPT_COUNT p183 = CommunicationChannel.new_SCRIPT_COUNT();
        PH.setPack(p183);
        p183.target_component_SET((char)171) ;
        p183.target_system_SET((char)165) ;
        p183.count_SET((char)40526) ;
        CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCRIPT_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)52137);
        });
        GroundControl.SCRIPT_CURRENT p184 = CommunicationChannel.new_SCRIPT_CURRENT();
        PH.setPack(p184);
        p184.seq_SET((char)52137) ;
        CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pos_vert_ratio_GET() == -1.3921767E38F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT));
            assert(pack.pos_horiz_accuracy_GET() == 5.4087097E37F);
            assert(pack.tas_ratio_GET() == -2.7633428E38F);
            assert(pack.vel_ratio_GET() == -1.3769499E38F);
            assert(pack.pos_horiz_ratio_GET() == 1.1563505E38F);
            assert(pack.time_usec_GET() == 3555854548701158371L);
            assert(pack.pos_vert_accuracy_GET() == -5.2112356E37F);
            assert(pack.mag_ratio_GET() == 4.681421E37F);
            assert(pack.hagl_ratio_GET() == 2.5454627E37F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.tas_ratio_SET(-2.7633428E38F) ;
        p230.pos_vert_ratio_SET(-1.3921767E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT)) ;
        p230.pos_vert_accuracy_SET(-5.2112356E37F) ;
        p230.pos_horiz_ratio_SET(1.1563505E38F) ;
        p230.pos_horiz_accuracy_SET(5.4087097E37F) ;
        p230.vel_ratio_SET(-1.3769499E38F) ;
        p230.mag_ratio_SET(4.681421E37F) ;
        p230.hagl_ratio_SET(2.5454627E37F) ;
        p230.time_usec_SET(3555854548701158371L) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.vert_accuracy_GET() == 1.7236981E38F);
            assert(pack.wind_x_GET() == 8.645464E37F);
            assert(pack.wind_alt_GET() == -1.0577578E38F);
            assert(pack.wind_z_GET() == 1.0804475E38F);
            assert(pack.time_usec_GET() == 2401818378609216605L);
            assert(pack.wind_y_GET() == -3.5447742E37F);
            assert(pack.horiz_accuracy_GET() == 1.7441016E38F);
            assert(pack.var_vert_GET() == -1.2602317E38F);
            assert(pack.var_horiz_GET() == -2.1858495E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.vert_accuracy_SET(1.7236981E38F) ;
        p231.wind_y_SET(-3.5447742E37F) ;
        p231.var_vert_SET(-1.2602317E38F) ;
        p231.wind_x_SET(8.645464E37F) ;
        p231.horiz_accuracy_SET(1.7441016E38F) ;
        p231.time_usec_SET(2401818378609216605L) ;
        p231.wind_z_SET(1.0804475E38F) ;
        p231.var_horiz_SET(-2.1858495E38F) ;
        p231.wind_alt_SET(-1.0577578E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 949080629102974304L);
            assert(pack.vn_GET() == -9.211549E37F);
            assert(pack.lat_GET() == -720043338);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY));
            assert(pack.speed_accuracy_GET() == 6.5975535E37F);
            assert(pack.time_week_GET() == (char)8590);
            assert(pack.fix_type_GET() == (char)165);
            assert(pack.vert_accuracy_GET() == 1.4185953E38F);
            assert(pack.lon_GET() == -809879146);
            assert(pack.satellites_visible_GET() == (char)213);
            assert(pack.time_week_ms_GET() == 752517122L);
            assert(pack.horiz_accuracy_GET() == 2.048173E38F);
            assert(pack.vdop_GET() == -5.739751E37F);
            assert(pack.gps_id_GET() == (char)78);
            assert(pack.hdop_GET() == 1.7001964E38F);
            assert(pack.ve_GET() == -2.3808168E38F);
            assert(pack.vd_GET() == 7.0899754E37F);
            assert(pack.alt_GET() == 2.1579677E38F);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.lat_SET(-720043338) ;
        p232.lon_SET(-809879146) ;
        p232.gps_id_SET((char)78) ;
        p232.vdop_SET(-5.739751E37F) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY)) ;
        p232.time_week_SET((char)8590) ;
        p232.horiz_accuracy_SET(2.048173E38F) ;
        p232.speed_accuracy_SET(6.5975535E37F) ;
        p232.vert_accuracy_SET(1.4185953E38F) ;
        p232.fix_type_SET((char)165) ;
        p232.hdop_SET(1.7001964E38F) ;
        p232.time_usec_SET(949080629102974304L) ;
        p232.alt_SET(2.1579677E38F) ;
        p232.satellites_visible_SET((char)213) ;
        p232.vn_SET(-9.211549E37F) ;
        p232.time_week_ms_SET(752517122L) ;
        p232.ve_SET(-2.3808168E38F) ;
        p232.vd_SET(7.0899754E37F) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (char)21);
            assert(pack.len_GET() == (char)182);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)120, (char)135, (char)125, (char)184, (char)253, (char)192, (char)15, (char)231, (char)223, (char)53, (char)31, (char)56, (char)129, (char)141, (char)224, (char)41, (char)47, (char)85, (char)245, (char)182, (char)153, (char)58, (char)10, (char)252, (char)7, (char)104, (char)215, (char)1, (char)48, (char)34, (char)112, (char)248, (char)214, (char)17, (char)164, (char)137, (char)248, (char)115, (char)168, (char)138, (char)32, (char)218, (char)251, (char)249, (char)39, (char)119, (char)30, (char)31, (char)161, (char)178, (char)238, (char)166, (char)132, (char)167, (char)192, (char)245, (char)133, (char)3, (char)130, (char)252, (char)53, (char)220, (char)243, (char)154, (char)154, (char)100, (char)213, (char)41, (char)19, (char)255, (char)159, (char)166, (char)222, (char)216, (char)3, (char)188, (char)139, (char)81, (char)255, (char)117, (char)195, (char)15, (char)24, (char)104, (char)30, (char)243, (char)1, (char)229, (char)220, (char)253, (char)52, (char)194, (char)227, (char)248, (char)40, (char)174, (char)69, (char)209, (char)248, (char)136, (char)91, (char)246, (char)62, (char)101, (char)87, (char)10, (char)19, (char)108, (char)166, (char)46, (char)69, (char)125, (char)121, (char)15, (char)3, (char)72, (char)110, (char)174, (char)106, (char)215, (char)235, (char)12, (char)181, (char)176, (char)159, (char)224, (char)75, (char)208, (char)197, (char)62, (char)176, (char)223, (char)158, (char)54, (char)21, (char)198, (char)239, (char)139, (char)19, (char)160, (char)66, (char)115, (char)237, (char)131, (char)112, (char)255, (char)112, (char)220, (char)47, (char)212, (char)7, (char)16, (char)177, (char)138, (char)155, (char)59, (char)237, (char)186, (char)254, (char)204, (char)171, (char)51, (char)97, (char)241, (char)5, (char)97, (char)59, (char)13, (char)117, (char)34, (char)36, (char)77, (char)36, (char)38, (char)51, (char)155, (char)106, (char)21, (char)47, (char)30}));
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.len_SET((char)182) ;
        p233.flags_SET((char)21) ;
        p233.data__SET(new char[] {(char)120, (char)135, (char)125, (char)184, (char)253, (char)192, (char)15, (char)231, (char)223, (char)53, (char)31, (char)56, (char)129, (char)141, (char)224, (char)41, (char)47, (char)85, (char)245, (char)182, (char)153, (char)58, (char)10, (char)252, (char)7, (char)104, (char)215, (char)1, (char)48, (char)34, (char)112, (char)248, (char)214, (char)17, (char)164, (char)137, (char)248, (char)115, (char)168, (char)138, (char)32, (char)218, (char)251, (char)249, (char)39, (char)119, (char)30, (char)31, (char)161, (char)178, (char)238, (char)166, (char)132, (char)167, (char)192, (char)245, (char)133, (char)3, (char)130, (char)252, (char)53, (char)220, (char)243, (char)154, (char)154, (char)100, (char)213, (char)41, (char)19, (char)255, (char)159, (char)166, (char)222, (char)216, (char)3, (char)188, (char)139, (char)81, (char)255, (char)117, (char)195, (char)15, (char)24, (char)104, (char)30, (char)243, (char)1, (char)229, (char)220, (char)253, (char)52, (char)194, (char)227, (char)248, (char)40, (char)174, (char)69, (char)209, (char)248, (char)136, (char)91, (char)246, (char)62, (char)101, (char)87, (char)10, (char)19, (char)108, (char)166, (char)46, (char)69, (char)125, (char)121, (char)15, (char)3, (char)72, (char)110, (char)174, (char)106, (char)215, (char)235, (char)12, (char)181, (char)176, (char)159, (char)224, (char)75, (char)208, (char)197, (char)62, (char)176, (char)223, (char)158, (char)54, (char)21, (char)198, (char)239, (char)139, (char)19, (char)160, (char)66, (char)115, (char)237, (char)131, (char)112, (char)255, (char)112, (char)220, (char)47, (char)212, (char)7, (char)16, (char)177, (char)138, (char)155, (char)59, (char)237, (char)186, (char)254, (char)204, (char)171, (char)51, (char)97, (char)241, (char)5, (char)97, (char)59, (char)13, (char)117, (char)34, (char)36, (char)77, (char)36, (char)38, (char)51, (char)155, (char)106, (char)21, (char)47, (char)30}, 0) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == (short) -5957);
            assert(pack.airspeed_sp_GET() == (char)7);
            assert(pack.wp_num_GET() == (char)70);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            assert(pack.heading_sp_GET() == (short)22607);
            assert(pack.battery_remaining_GET() == (char)91);
            assert(pack.longitude_GET() == -2047228059);
            assert(pack.altitude_amsl_GET() == (short)191);
            assert(pack.throttle_GET() == (byte) - 122);
            assert(pack.latitude_GET() == -226056995);
            assert(pack.airspeed_GET() == (char)18);
            assert(pack.custom_mode_GET() == 2298740081L);
            assert(pack.altitude_sp_GET() == (short) -12258);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
            assert(pack.failsafe_GET() == (char)96);
            assert(pack.climb_rate_GET() == (byte)86);
            assert(pack.gps_nsat_GET() == (char)109);
            assert(pack.temperature_air_GET() == (byte) - 38);
            assert(pack.groundspeed_GET() == (char)190);
            assert(pack.pitch_GET() == (short) -1327);
            assert(pack.heading_GET() == (char)37458);
            assert(pack.temperature_GET() == (byte)9);
            assert(pack.wp_distance_GET() == (char)27239);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.wp_distance_SET((char)27239) ;
        p234.groundspeed_SET((char)190) ;
        p234.temperature_air_SET((byte) - 38) ;
        p234.heading_sp_SET((short)22607) ;
        p234.climb_rate_SET((byte)86) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP) ;
        p234.pitch_SET((short) -1327) ;
        p234.throttle_SET((byte) - 122) ;
        p234.custom_mode_SET(2298740081L) ;
        p234.altitude_sp_SET((short) -12258) ;
        p234.longitude_SET(-2047228059) ;
        p234.latitude_SET(-226056995) ;
        p234.temperature_SET((byte)9) ;
        p234.roll_SET((short) -5957) ;
        p234.airspeed_sp_SET((char)7) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING) ;
        p234.battery_remaining_SET((char)91) ;
        p234.wp_num_SET((char)70) ;
        p234.failsafe_SET((char)96) ;
        p234.gps_nsat_SET((char)109) ;
        p234.altitude_amsl_SET((short)191) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED)) ;
        p234.heading_SET((char)37458) ;
        p234.airspeed_SET((char)18) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.clipping_2_GET() == 408743496L);
            assert(pack.time_usec_GET() == 6801783511273986901L);
            assert(pack.vibration_y_GET() == 7.9499425E37F);
            assert(pack.clipping_1_GET() == 2277304732L);
            assert(pack.vibration_z_GET() == 2.4477053E38F);
            assert(pack.clipping_0_GET() == 2540802268L);
            assert(pack.vibration_x_GET() == 1.3164993E38F);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.vibration_x_SET(1.3164993E38F) ;
        p241.time_usec_SET(6801783511273986901L) ;
        p241.vibration_y_SET(7.9499425E37F) ;
        p241.clipping_1_SET(2277304732L) ;
        p241.clipping_0_SET(2540802268L) ;
        p241.vibration_z_SET(2.4477053E38F) ;
        p241.clipping_2_SET(408743496L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 2.9949795E38F);
            assert(pack.time_usec_TRY(ph) == 3257218310466995004L);
            assert(pack.approach_x_GET() == -2.1845216E38F);
            assert(pack.y_GET() == 4.112205E36F);
            assert(pack.longitude_GET() == 317124494);
            assert(pack.approach_y_GET() == 3.0730481E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.9155403E38F, -1.9938712E38F, -1.0962807E38F, 2.752162E38F}));
            assert(pack.altitude_GET() == 346571017);
            assert(pack.latitude_GET() == 542495716);
            assert(pack.x_GET() == 2.4230379E38F);
            assert(pack.approach_z_GET() == -2.3061943E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.x_SET(2.4230379E38F) ;
        p242.longitude_SET(317124494) ;
        p242.time_usec_SET(3257218310466995004L, PH) ;
        p242.y_SET(4.112205E36F) ;
        p242.q_SET(new float[] {-1.9155403E38F, -1.9938712E38F, -1.0962807E38F, 2.752162E38F}, 0) ;
        p242.z_SET(2.9949795E38F) ;
        p242.latitude_SET(542495716) ;
        p242.approach_z_SET(-2.3061943E38F) ;
        p242.altitude_SET(346571017) ;
        p242.approach_y_SET(3.0730481E38F) ;
        p242.approach_x_SET(-2.1845216E38F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_y_GET() == 1.3737894E38F);
            assert(pack.time_usec_TRY(ph) == 1982531281571075402L);
            assert(pack.y_GET() == 4.827073E37F);
            assert(pack.approach_x_GET() == 7.47068E37F);
            assert(pack.approach_z_GET() == -1.0565103E38F);
            assert(pack.longitude_GET() == 27589036);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.3823905E38F, 1.8565575E38F, -1.6828756E38F, 3.150314E38F}));
            assert(pack.target_system_GET() == (char)28);
            assert(pack.altitude_GET() == 1403888238);
            assert(pack.z_GET() == 2.1169193E38F);
            assert(pack.latitude_GET() == 141966599);
            assert(pack.x_GET() == 2.418212E37F);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.approach_x_SET(7.47068E37F) ;
        p243.approach_z_SET(-1.0565103E38F) ;
        p243.longitude_SET(27589036) ;
        p243.x_SET(2.418212E37F) ;
        p243.target_system_SET((char)28) ;
        p243.latitude_SET(141966599) ;
        p243.z_SET(2.1169193E38F) ;
        p243.altitude_SET(1403888238) ;
        p243.approach_y_SET(1.3737894E38F) ;
        p243.q_SET(new float[] {2.3823905E38F, 1.8565575E38F, -1.6828756E38F, 3.150314E38F}, 0) ;
        p243.time_usec_SET(1982531281571075402L, PH) ;
        p243.y_SET(4.827073E37F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)45311);
            assert(pack.interval_us_GET() == -601647164);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-601647164) ;
        p244.message_id_SET((char)45311) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING) ;
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING));
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.heading_GET() == (char)48937);
            assert(pack.tslc_GET() == (char)79);
            assert(pack.ICAO_address_GET() == 1015511085L);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE);
            assert(pack.altitude_GET() == -264695219);
            assert(pack.squawk_GET() == (char)12715);
            assert(pack.hor_velocity_GET() == (char)43919);
            assert(pack.callsign_LEN(ph) == 3);
            assert(pack.callsign_TRY(ph).equals("hoy"));
            assert(pack.ver_velocity_GET() == (short) -5249);
            assert(pack.lat_GET() == 1169889893);
            assert(pack.lon_GET() == -997710548);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.lon_SET(-997710548) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING)) ;
        p246.squawk_SET((char)12715) ;
        p246.callsign_SET("hoy", PH) ;
        p246.hor_velocity_SET((char)43919) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.altitude_SET(-264695219) ;
        p246.ver_velocity_SET((short) -5249) ;
        p246.lat_SET(1169889893) ;
        p246.ICAO_address_SET(1015511085L) ;
        p246.heading_SET((char)48937) ;
        p246.tslc_SET((char)79) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == 2571145152L);
            assert(pack.altitude_minimum_delta_GET() == 3.19864E38F);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL);
            assert(pack.time_to_minimum_delta_GET() == 3.1083434E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            assert(pack.horizontal_minimum_delta_GET() == -1.9013406E38F);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.id_SET(2571145152L) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE) ;
        p247.altitude_minimum_delta_SET(3.19864E38F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL) ;
        p247.horizontal_minimum_delta_SET(-1.9013406E38F) ;
        p247.time_to_minimum_delta_SET(3.1083434E38F) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)204);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)190, (char)24, (char)34, (char)55, (char)124, (char)244, (char)99, (char)33, (char)218, (char)3, (char)245, (char)147, (char)5, (char)105, (char)125, (char)88, (char)19, (char)99, (char)108, (char)37, (char)255, (char)60, (char)233, (char)35, (char)211, (char)130, (char)120, (char)21, (char)252, (char)217, (char)245, (char)186, (char)225, (char)205, (char)189, (char)43, (char)186, (char)144, (char)204, (char)48, (char)148, (char)94, (char)92, (char)125, (char)158, (char)99, (char)133, (char)15, (char)96, (char)147, (char)85, (char)4, (char)166, (char)88, (char)177, (char)104, (char)31, (char)188, (char)102, (char)188, (char)7, (char)212, (char)114, (char)78, (char)249, (char)247, (char)202, (char)110, (char)59, (char)80, (char)153, (char)63, (char)86, (char)61, (char)139, (char)11, (char)6, (char)246, (char)243, (char)93, (char)77, (char)209, (char)244, (char)49, (char)230, (char)221, (char)85, (char)198, (char)184, (char)155, (char)93, (char)73, (char)109, (char)114, (char)174, (char)47, (char)18, (char)109, (char)35, (char)51, (char)120, (char)178, (char)245, (char)10, (char)63, (char)208, (char)0, (char)117, (char)18, (char)224, (char)175, (char)7, (char)167, (char)139, (char)198, (char)28, (char)230, (char)122, (char)193, (char)141, (char)96, (char)23, (char)78, (char)35, (char)105, (char)243, (char)113, (char)254, (char)74, (char)22, (char)236, (char)50, (char)97, (char)83, (char)33, (char)79, (char)99, (char)111, (char)156, (char)14, (char)1, (char)135, (char)219, (char)45, (char)240, (char)130, (char)165, (char)96, (char)173, (char)100, (char)22, (char)57, (char)106, (char)154, (char)47, (char)93, (char)51, (char)67, (char)11, (char)127, (char)170, (char)97, (char)70, (char)77, (char)247, (char)175, (char)130, (char)33, (char)177, (char)23, (char)213, (char)142, (char)113, (char)254, (char)68, (char)143, (char)35, (char)143, (char)36, (char)138, (char)198, (char)248, (char)127, (char)182, (char)201, (char)199, (char)249, (char)63, (char)197, (char)230, (char)22, (char)19, (char)163, (char)30, (char)24, (char)92, (char)86, (char)164, (char)208, (char)33, (char)61, (char)161, (char)93, (char)89, (char)100, (char)238, (char)141, (char)11, (char)112, (char)122, (char)39, (char)207, (char)50, (char)68, (char)204, (char)49, (char)19, (char)52, (char)176, (char)216, (char)108, (char)156, (char)254, (char)201, (char)213, (char)204, (char)231, (char)10, (char)108, (char)51, (char)203, (char)232, (char)119, (char)231, (char)29, (char)106, (char)13, (char)4, (char)171, (char)89, (char)5, (char)153, (char)64, (char)237, (char)107, (char)101, (char)114, (char)6, (char)183}));
            assert(pack.target_system_GET() == (char)109);
            assert(pack.target_network_GET() == (char)76);
            assert(pack.message_type_GET() == (char)51715);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)76) ;
        p248.target_component_SET((char)204) ;
        p248.target_system_SET((char)109) ;
        p248.payload_SET(new char[] {(char)190, (char)24, (char)34, (char)55, (char)124, (char)244, (char)99, (char)33, (char)218, (char)3, (char)245, (char)147, (char)5, (char)105, (char)125, (char)88, (char)19, (char)99, (char)108, (char)37, (char)255, (char)60, (char)233, (char)35, (char)211, (char)130, (char)120, (char)21, (char)252, (char)217, (char)245, (char)186, (char)225, (char)205, (char)189, (char)43, (char)186, (char)144, (char)204, (char)48, (char)148, (char)94, (char)92, (char)125, (char)158, (char)99, (char)133, (char)15, (char)96, (char)147, (char)85, (char)4, (char)166, (char)88, (char)177, (char)104, (char)31, (char)188, (char)102, (char)188, (char)7, (char)212, (char)114, (char)78, (char)249, (char)247, (char)202, (char)110, (char)59, (char)80, (char)153, (char)63, (char)86, (char)61, (char)139, (char)11, (char)6, (char)246, (char)243, (char)93, (char)77, (char)209, (char)244, (char)49, (char)230, (char)221, (char)85, (char)198, (char)184, (char)155, (char)93, (char)73, (char)109, (char)114, (char)174, (char)47, (char)18, (char)109, (char)35, (char)51, (char)120, (char)178, (char)245, (char)10, (char)63, (char)208, (char)0, (char)117, (char)18, (char)224, (char)175, (char)7, (char)167, (char)139, (char)198, (char)28, (char)230, (char)122, (char)193, (char)141, (char)96, (char)23, (char)78, (char)35, (char)105, (char)243, (char)113, (char)254, (char)74, (char)22, (char)236, (char)50, (char)97, (char)83, (char)33, (char)79, (char)99, (char)111, (char)156, (char)14, (char)1, (char)135, (char)219, (char)45, (char)240, (char)130, (char)165, (char)96, (char)173, (char)100, (char)22, (char)57, (char)106, (char)154, (char)47, (char)93, (char)51, (char)67, (char)11, (char)127, (char)170, (char)97, (char)70, (char)77, (char)247, (char)175, (char)130, (char)33, (char)177, (char)23, (char)213, (char)142, (char)113, (char)254, (char)68, (char)143, (char)35, (char)143, (char)36, (char)138, (char)198, (char)248, (char)127, (char)182, (char)201, (char)199, (char)249, (char)63, (char)197, (char)230, (char)22, (char)19, (char)163, (char)30, (char)24, (char)92, (char)86, (char)164, (char)208, (char)33, (char)61, (char)161, (char)93, (char)89, (char)100, (char)238, (char)141, (char)11, (char)112, (char)122, (char)39, (char)207, (char)50, (char)68, (char)204, (char)49, (char)19, (char)52, (char)176, (char)216, (char)108, (char)156, (char)254, (char)201, (char)213, (char)204, (char)231, (char)10, (char)108, (char)51, (char)203, (char)232, (char)119, (char)231, (char)29, (char)106, (char)13, (char)4, (char)171, (char)89, (char)5, (char)153, (char)64, (char)237, (char)107, (char)101, (char)114, (char)6, (char)183}, 0) ;
        p248.message_type_SET((char)51715) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.address_GET() == (char)47957);
            assert(pack.type_GET() == (char)207);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 119, (byte)116, (byte) - 13, (byte)64, (byte) - 10, (byte)40, (byte)50, (byte) - 77, (byte) - 44, (byte) - 82, (byte) - 117, (byte) - 25, (byte) - 38, (byte) - 52, (byte)84, (byte) - 114, (byte) - 88, (byte)77, (byte) - 68, (byte) - 46, (byte)48, (byte) - 2, (byte) - 55, (byte) - 56, (byte)102, (byte)120, (byte) - 106, (byte)124, (byte)103, (byte) - 101, (byte)36, (byte) - 67}));
            assert(pack.ver_GET() == (char)104);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)47957) ;
        p249.value_SET(new byte[] {(byte) - 119, (byte)116, (byte) - 13, (byte)64, (byte) - 10, (byte)40, (byte)50, (byte) - 77, (byte) - 44, (byte) - 82, (byte) - 117, (byte) - 25, (byte) - 38, (byte) - 52, (byte)84, (byte) - 114, (byte) - 88, (byte)77, (byte) - 68, (byte) - 46, (byte)48, (byte) - 2, (byte) - 55, (byte) - 56, (byte)102, (byte)120, (byte) - 106, (byte)124, (byte)103, (byte) - 101, (byte)36, (byte) - 67}, 0) ;
        p249.ver_SET((char)104) ;
        p249.type_SET((char)207) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.8790348E38F);
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("FjowbVeexn"));
            assert(pack.y_GET() == -3.1818172E38F);
            assert(pack.time_usec_GET() == 3678729180835132389L);
            assert(pack.x_GET() == 2.651351E38F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("FjowbVeexn", PH) ;
        p250.z_SET(-1.8790348E38F) ;
        p250.y_SET(-3.1818172E38F) ;
        p250.x_SET(2.651351E38F) ;
        p250.time_usec_SET(3678729180835132389L) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 9);
            assert(pack.name_TRY(ph).equals("vlgysvtwj"));
            assert(pack.value_GET() == 1.670526E38F);
            assert(pack.time_boot_ms_GET() == 1684596916L);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(1.670526E38F) ;
        p251.time_boot_ms_SET(1684596916L) ;
        p251.name_SET("vlgysvtwj", PH) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3538694957L);
            assert(pack.value_GET() == -677903144);
            assert(pack.name_LEN(ph) == 2);
            assert(pack.name_TRY(ph).equals("xn"));
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(3538694957L) ;
        p252.value_SET(-677903144) ;
        p252.name_SET("xn", PH) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 38);
            assert(pack.text_TRY(ph).equals("ynvnctrjtnCukztrqimgzycigzepaaKdnuhtte"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_ERROR);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_ERROR) ;
        p253.text_SET("ynvnctrjtnCukztrqimgzycigzepaaKdnuhtte", PH) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.ind_GET() == (char)250);
            assert(pack.time_boot_ms_GET() == 2029391900L);
            assert(pack.value_GET() == -1.8470328E38F);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.ind_SET((char)250) ;
        p254.time_boot_ms_SET(2029391900L) ;
        p254.value_SET(-1.8470328E38F) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)171);
            assert(pack.target_system_GET() == (char)132);
            assert(pack.initial_timestamp_GET() == 2881419238394791355L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)37, (char)90, (char)134, (char)196, (char)241, (char)176, (char)114, (char)223, (char)65, (char)119, (char)16, (char)223, (char)234, (char)233, (char)148, (char)135, (char)239, (char)169, (char)223, (char)254, (char)47, (char)119, (char)140, (char)32, (char)236, (char)127, (char)244, (char)40, (char)1, (char)221, (char)110, (char)122}));
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.initial_timestamp_SET(2881419238394791355L) ;
        p256.target_component_SET((char)171) ;
        p256.secret_key_SET(new char[] {(char)37, (char)90, (char)134, (char)196, (char)241, (char)176, (char)114, (char)223, (char)65, (char)119, (char)16, (char)223, (char)234, (char)233, (char)148, (char)135, (char)239, (char)169, (char)223, (char)254, (char)47, (char)119, (char)140, (char)32, (char)236, (char)127, (char)244, (char)40, (char)1, (char)221, (char)110, (char)122}, 0) ;
        p256.target_system_SET((char)132) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.state_GET() == (char)213);
            assert(pack.last_change_ms_GET() == 1077873797L);
            assert(pack.time_boot_ms_GET() == 1900858921L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.state_SET((char)213) ;
        p257.time_boot_ms_SET(1900858921L) ;
        p257.last_change_ms_SET(1077873797L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)82);
            assert(pack.target_system_GET() == (char)94);
            assert(pack.tune_LEN(ph) == 23);
            assert(pack.tune_TRY(ph).equals("poQnAwarxagiuydbrcWsumb"));
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.tune_SET("poQnAwarxagiuydbrcWsumb", PH) ;
        p258.target_component_SET((char)82) ;
        p258.target_system_SET((char)94) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO));
            assert(pack.time_boot_ms_GET() == 2508643645L);
            assert(pack.cam_definition_version_GET() == (char)8064);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)198, (char)173, (char)168, (char)91, (char)254, (char)72, (char)165, (char)123, (char)79, (char)207, (char)231, (char)242, (char)3, (char)136, (char)76, (char)55, (char)24, (char)44, (char)215, (char)100, (char)8, (char)30, (char)189, (char)240, (char)157, (char)221, (char)234, (char)185, (char)113, (char)188, (char)90, (char)111}));
            assert(pack.firmware_version_GET() == 2799793628L);
            assert(pack.cam_definition_uri_LEN(ph) == 61);
            assert(pack.cam_definition_uri_TRY(ph).equals("evxiecfsnuehlpisunbzizzbeVpnymbpmzawafgntidmleomncxytrlwbhqws"));
            assert(pack.lens_id_GET() == (char)121);
            assert(pack.sensor_size_h_GET() == -3.0875216E38F);
            assert(pack.resolution_v_GET() == (char)3042);
            assert(pack.resolution_h_GET() == (char)13721);
            assert(pack.sensor_size_v_GET() == 1.1598656E37F);
            assert(pack.focal_length_GET() == -1.678936E38F);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)168, (char)95, (char)228, (char)44, (char)89, (char)194, (char)3, (char)151, (char)57, (char)243, (char)29, (char)25, (char)47, (char)124, (char)35, (char)142, (char)100, (char)51, (char)105, (char)22, (char)72, (char)48, (char)188, (char)170, (char)27, (char)31, (char)72, (char)216, (char)44, (char)215, (char)226, (char)253}));
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.model_name_SET(new char[] {(char)198, (char)173, (char)168, (char)91, (char)254, (char)72, (char)165, (char)123, (char)79, (char)207, (char)231, (char)242, (char)3, (char)136, (char)76, (char)55, (char)24, (char)44, (char)215, (char)100, (char)8, (char)30, (char)189, (char)240, (char)157, (char)221, (char)234, (char)185, (char)113, (char)188, (char)90, (char)111}, 0) ;
        p259.vendor_name_SET(new char[] {(char)168, (char)95, (char)228, (char)44, (char)89, (char)194, (char)3, (char)151, (char)57, (char)243, (char)29, (char)25, (char)47, (char)124, (char)35, (char)142, (char)100, (char)51, (char)105, (char)22, (char)72, (char)48, (char)188, (char)170, (char)27, (char)31, (char)72, (char)216, (char)44, (char)215, (char)226, (char)253}, 0) ;
        p259.firmware_version_SET(2799793628L) ;
        p259.focal_length_SET(-1.678936E38F) ;
        p259.sensor_size_h_SET(-3.0875216E38F) ;
        p259.cam_definition_version_SET((char)8064) ;
        p259.resolution_h_SET((char)13721) ;
        p259.lens_id_SET((char)121) ;
        p259.cam_definition_uri_SET("evxiecfsnuehlpisunbzizzbeVpnymbpmzawafgntidmleomncxytrlwbhqws", PH) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO)) ;
        p259.resolution_v_SET((char)3042) ;
        p259.sensor_size_v_SET(1.1598656E37F) ;
        p259.time_boot_ms_SET(2508643645L) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 437257297L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_VIDEO);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO) ;
        p260.time_boot_ms_SET(437257297L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.total_capacity_GET() == -2.762245E38F);
            assert(pack.write_speed_GET() == -3.2639983E38F);
            assert(pack.available_capacity_GET() == -2.9807771E38F);
            assert(pack.status_GET() == (char)234);
            assert(pack.storage_count_GET() == (char)141);
            assert(pack.used_capacity_GET() == -5.321957E37F);
            assert(pack.read_speed_GET() == 3.0159136E38F);
            assert(pack.time_boot_ms_GET() == 1744815606L);
            assert(pack.storage_id_GET() == (char)33);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.available_capacity_SET(-2.9807771E38F) ;
        p261.total_capacity_SET(-2.762245E38F) ;
        p261.write_speed_SET(-3.2639983E38F) ;
        p261.used_capacity_SET(-5.321957E37F) ;
        p261.storage_count_SET((char)141) ;
        p261.read_speed_SET(3.0159136E38F) ;
        p261.time_boot_ms_SET(1744815606L) ;
        p261.status_SET((char)234) ;
        p261.storage_id_SET((char)33) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.video_status_GET() == (char)106);
            assert(pack.image_status_GET() == (char)67);
            assert(pack.time_boot_ms_GET() == 2351287538L);
            assert(pack.recording_time_ms_GET() == 1925558743L);
            assert(pack.image_interval_GET() == -7.598816E37F);
            assert(pack.available_capacity_GET() == 4.407273E37F);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.image_interval_SET(-7.598816E37F) ;
        p262.available_capacity_SET(4.407273E37F) ;
        p262.time_boot_ms_SET(2351287538L) ;
        p262.image_status_SET((char)67) ;
        p262.recording_time_ms_SET(1925558743L) ;
        p262.video_status_SET((char)106) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.capture_result_GET() == (byte)79);
            assert(pack.file_url_LEN(ph) == 70);
            assert(pack.file_url_TRY(ph).equals("FfoywvwrgzopllwYymjoEpodlqTgdyfhojEjpwmxkxcdoxpekxjedqpjVuvvjeyexcSLgH"));
            assert(pack.camera_id_GET() == (char)64);
            assert(pack.relative_alt_GET() == 45364997);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-9.4381715E36F, -1.0384191E38F, 3.3125037E38F, -2.5774892E38F}));
            assert(pack.lon_GET() == -1201076794);
            assert(pack.lat_GET() == 756171875);
            assert(pack.time_boot_ms_GET() == 4203155072L);
            assert(pack.alt_GET() == -733990589);
            assert(pack.image_index_GET() == -744001711);
            assert(pack.time_utc_GET() == 5260835004924928840L);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(4203155072L) ;
        p263.relative_alt_SET(45364997) ;
        p263.file_url_SET("FfoywvwrgzopllwYymjoEpodlqTgdyfhojEjpwmxkxcdoxpekxjedqpjVuvvjeyexcSLgH", PH) ;
        p263.capture_result_SET((byte)79) ;
        p263.lon_SET(-1201076794) ;
        p263.image_index_SET(-744001711) ;
        p263.q_SET(new float[] {-9.4381715E36F, -1.0384191E38F, 3.3125037E38F, -2.5774892E38F}, 0) ;
        p263.time_utc_SET(5260835004924928840L) ;
        p263.camera_id_SET((char)64) ;
        p263.lat_SET(756171875) ;
        p263.alt_SET(-733990589) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3299789862L);
            assert(pack.flight_uuid_GET() == 2481707320678178058L);
            assert(pack.takeoff_time_utc_GET() == 8303269922391076229L);
            assert(pack.arming_time_utc_GET() == 8641399659127468536L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.takeoff_time_utc_SET(8303269922391076229L) ;
        p264.arming_time_utc_SET(8641399659127468536L) ;
        p264.time_boot_ms_SET(3299789862L) ;
        p264.flight_uuid_SET(2481707320678178058L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -2.4321193E38F);
            assert(pack.yaw_GET() == 4.933055E36F);
            assert(pack.pitch_GET() == -6.5014275E37F);
            assert(pack.time_boot_ms_GET() == 1475856970L);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(-2.4321193E38F) ;
        p265.yaw_SET(4.933055E36F) ;
        p265.pitch_SET(-6.5014275E37F) ;
        p265.time_boot_ms_SET(1475856970L) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)205);
            assert(pack.length_GET() == (char)241);
            assert(pack.sequence_GET() == (char)2931);
            assert(pack.first_message_offset_GET() == (char)167);
            assert(pack.target_system_GET() == (char)41);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)3, (char)248, (char)233, (char)27, (char)236, (char)53, (char)83, (char)19, (char)190, (char)235, (char)163, (char)46, (char)198, (char)141, (char)157, (char)199, (char)181, (char)120, (char)58, (char)47, (char)209, (char)61, (char)36, (char)21, (char)255, (char)86, (char)59, (char)119, (char)9, (char)114, (char)16, (char)67, (char)97, (char)169, (char)24, (char)183, (char)150, (char)96, (char)4, (char)54, (char)168, (char)142, (char)3, (char)226, (char)31, (char)186, (char)37, (char)245, (char)118, (char)201, (char)120, (char)209, (char)155, (char)97, (char)104, (char)47, (char)55, (char)91, (char)49, (char)100, (char)87, (char)107, (char)53, (char)16, (char)252, (char)27, (char)51, (char)144, (char)164, (char)103, (char)146, (char)149, (char)168, (char)114, (char)251, (char)87, (char)2, (char)130, (char)76, (char)162, (char)44, (char)11, (char)24, (char)118, (char)148, (char)83, (char)181, (char)90, (char)12, (char)79, (char)80, (char)113, (char)157, (char)78, (char)120, (char)173, (char)126, (char)236, (char)171, (char)248, (char)46, (char)46, (char)192, (char)131, (char)42, (char)57, (char)17, (char)47, (char)176, (char)93, (char)238, (char)18, (char)26, (char)228, (char)7, (char)73, (char)76, (char)166, (char)5, (char)112, (char)122, (char)108, (char)103, (char)126, (char)133, (char)29, (char)7, (char)136, (char)179, (char)140, (char)217, (char)160, (char)59, (char)214, (char)101, (char)55, (char)8, (char)87, (char)78, (char)14, (char)96, (char)61, (char)157, (char)194, (char)61, (char)128, (char)229, (char)0, (char)158, (char)35, (char)86, (char)112, (char)103, (char)82, (char)38, (char)56, (char)19, (char)179, (char)128, (char)109, (char)174, (char)238, (char)18, (char)211, (char)70, (char)32, (char)161, (char)167, (char)166, (char)234, (char)150, (char)193, (char)239, (char)247, (char)130, (char)133, (char)197, (char)80, (char)241, (char)110, (char)136, (char)75, (char)27, (char)18, (char)126, (char)212, (char)54, (char)147, (char)80, (char)166, (char)81, (char)47, (char)154, (char)147, (char)99, (char)81, (char)254, (char)198, (char)160, (char)69, (char)216, (char)7, (char)53, (char)99, (char)169, (char)173, (char)7, (char)141, (char)222, (char)115, (char)225, (char)164, (char)114, (char)139, (char)134, (char)56, (char)102, (char)57, (char)254, (char)253, (char)171, (char)203, (char)60, (char)253, (char)240, (char)23, (char)27, (char)222, (char)248, (char)126, (char)131, (char)227, (char)146, (char)37, (char)43, (char)200, (char)133, (char)68, (char)172, (char)177, (char)231, (char)74, (char)119, (char)156, (char)13, (char)170, (char)221, (char)34, (char)97}));
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.first_message_offset_SET((char)167) ;
        p266.length_SET((char)241) ;
        p266.target_system_SET((char)41) ;
        p266.data__SET(new char[] {(char)3, (char)248, (char)233, (char)27, (char)236, (char)53, (char)83, (char)19, (char)190, (char)235, (char)163, (char)46, (char)198, (char)141, (char)157, (char)199, (char)181, (char)120, (char)58, (char)47, (char)209, (char)61, (char)36, (char)21, (char)255, (char)86, (char)59, (char)119, (char)9, (char)114, (char)16, (char)67, (char)97, (char)169, (char)24, (char)183, (char)150, (char)96, (char)4, (char)54, (char)168, (char)142, (char)3, (char)226, (char)31, (char)186, (char)37, (char)245, (char)118, (char)201, (char)120, (char)209, (char)155, (char)97, (char)104, (char)47, (char)55, (char)91, (char)49, (char)100, (char)87, (char)107, (char)53, (char)16, (char)252, (char)27, (char)51, (char)144, (char)164, (char)103, (char)146, (char)149, (char)168, (char)114, (char)251, (char)87, (char)2, (char)130, (char)76, (char)162, (char)44, (char)11, (char)24, (char)118, (char)148, (char)83, (char)181, (char)90, (char)12, (char)79, (char)80, (char)113, (char)157, (char)78, (char)120, (char)173, (char)126, (char)236, (char)171, (char)248, (char)46, (char)46, (char)192, (char)131, (char)42, (char)57, (char)17, (char)47, (char)176, (char)93, (char)238, (char)18, (char)26, (char)228, (char)7, (char)73, (char)76, (char)166, (char)5, (char)112, (char)122, (char)108, (char)103, (char)126, (char)133, (char)29, (char)7, (char)136, (char)179, (char)140, (char)217, (char)160, (char)59, (char)214, (char)101, (char)55, (char)8, (char)87, (char)78, (char)14, (char)96, (char)61, (char)157, (char)194, (char)61, (char)128, (char)229, (char)0, (char)158, (char)35, (char)86, (char)112, (char)103, (char)82, (char)38, (char)56, (char)19, (char)179, (char)128, (char)109, (char)174, (char)238, (char)18, (char)211, (char)70, (char)32, (char)161, (char)167, (char)166, (char)234, (char)150, (char)193, (char)239, (char)247, (char)130, (char)133, (char)197, (char)80, (char)241, (char)110, (char)136, (char)75, (char)27, (char)18, (char)126, (char)212, (char)54, (char)147, (char)80, (char)166, (char)81, (char)47, (char)154, (char)147, (char)99, (char)81, (char)254, (char)198, (char)160, (char)69, (char)216, (char)7, (char)53, (char)99, (char)169, (char)173, (char)7, (char)141, (char)222, (char)115, (char)225, (char)164, (char)114, (char)139, (char)134, (char)56, (char)102, (char)57, (char)254, (char)253, (char)171, (char)203, (char)60, (char)253, (char)240, (char)23, (char)27, (char)222, (char)248, (char)126, (char)131, (char)227, (char)146, (char)37, (char)43, (char)200, (char)133, (char)68, (char)172, (char)177, (char)231, (char)74, (char)119, (char)156, (char)13, (char)170, (char)221, (char)34, (char)97}, 0) ;
        p266.target_component_SET((char)205) ;
        p266.sequence_SET((char)2931) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)138);
            assert(pack.sequence_GET() == (char)53635);
            assert(pack.length_GET() == (char)147);
            assert(pack.first_message_offset_GET() == (char)225);
            assert(pack.target_system_GET() == (char)29);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)167, (char)214, (char)54, (char)240, (char)167, (char)73, (char)146, (char)1, (char)92, (char)90, (char)135, (char)180, (char)123, (char)123, (char)136, (char)0, (char)58, (char)87, (char)118, (char)204, (char)91, (char)218, (char)109, (char)112, (char)137, (char)82, (char)4, (char)223, (char)172, (char)23, (char)151, (char)164, (char)153, (char)194, (char)148, (char)23, (char)203, (char)14, (char)81, (char)71, (char)48, (char)153, (char)196, (char)224, (char)111, (char)100, (char)233, (char)132, (char)230, (char)225, (char)37, (char)240, (char)93, (char)153, (char)31, (char)217, (char)42, (char)23, (char)239, (char)55, (char)221, (char)207, (char)194, (char)84, (char)183, (char)109, (char)48, (char)245, (char)159, (char)207, (char)130, (char)75, (char)97, (char)166, (char)138, (char)249, (char)17, (char)18, (char)195, (char)238, (char)239, (char)104, (char)103, (char)250, (char)220, (char)184, (char)55, (char)169, (char)115, (char)11, (char)2, (char)145, (char)131, (char)112, (char)73, (char)90, (char)223, (char)157, (char)148, (char)17, (char)31, (char)190, (char)25, (char)146, (char)98, (char)154, (char)59, (char)100, (char)91, (char)40, (char)50, (char)233, (char)109, (char)24, (char)185, (char)139, (char)234, (char)20, (char)179, (char)221, (char)197, (char)101, (char)139, (char)113, (char)78, (char)103, (char)19, (char)143, (char)237, (char)248, (char)246, (char)219, (char)74, (char)7, (char)100, (char)236, (char)196, (char)248, (char)90, (char)2, (char)9, (char)61, (char)11, (char)103, (char)6, (char)146, (char)121, (char)228, (char)178, (char)55, (char)66, (char)181, (char)130, (char)230, (char)18, (char)218, (char)26, (char)158, (char)52, (char)137, (char)62, (char)64, (char)48, (char)247, (char)222, (char)219, (char)115, (char)219, (char)69, (char)196, (char)31, (char)120, (char)112, (char)210, (char)136, (char)249, (char)223, (char)124, (char)78, (char)252, (char)82, (char)249, (char)252, (char)110, (char)120, (char)117, (char)99, (char)5, (char)77, (char)172, (char)196, (char)122, (char)121, (char)231, (char)222, (char)127, (char)3, (char)79, (char)124, (char)135, (char)154, (char)224, (char)135, (char)196, (char)182, (char)173, (char)161, (char)55, (char)182, (char)78, (char)113, (char)10, (char)179, (char)14, (char)122, (char)63, (char)3, (char)78, (char)35, (char)161, (char)119, (char)1, (char)83, (char)98, (char)154, (char)193, (char)64, (char)196, (char)234, (char)139, (char)116, (char)30, (char)63, (char)128, (char)64, (char)180, (char)233, (char)123, (char)40, (char)23, (char)61, (char)51, (char)113, (char)20, (char)245, (char)235, (char)5, (char)157, (char)102}));
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.sequence_SET((char)53635) ;
        p267.first_message_offset_SET((char)225) ;
        p267.data__SET(new char[] {(char)167, (char)214, (char)54, (char)240, (char)167, (char)73, (char)146, (char)1, (char)92, (char)90, (char)135, (char)180, (char)123, (char)123, (char)136, (char)0, (char)58, (char)87, (char)118, (char)204, (char)91, (char)218, (char)109, (char)112, (char)137, (char)82, (char)4, (char)223, (char)172, (char)23, (char)151, (char)164, (char)153, (char)194, (char)148, (char)23, (char)203, (char)14, (char)81, (char)71, (char)48, (char)153, (char)196, (char)224, (char)111, (char)100, (char)233, (char)132, (char)230, (char)225, (char)37, (char)240, (char)93, (char)153, (char)31, (char)217, (char)42, (char)23, (char)239, (char)55, (char)221, (char)207, (char)194, (char)84, (char)183, (char)109, (char)48, (char)245, (char)159, (char)207, (char)130, (char)75, (char)97, (char)166, (char)138, (char)249, (char)17, (char)18, (char)195, (char)238, (char)239, (char)104, (char)103, (char)250, (char)220, (char)184, (char)55, (char)169, (char)115, (char)11, (char)2, (char)145, (char)131, (char)112, (char)73, (char)90, (char)223, (char)157, (char)148, (char)17, (char)31, (char)190, (char)25, (char)146, (char)98, (char)154, (char)59, (char)100, (char)91, (char)40, (char)50, (char)233, (char)109, (char)24, (char)185, (char)139, (char)234, (char)20, (char)179, (char)221, (char)197, (char)101, (char)139, (char)113, (char)78, (char)103, (char)19, (char)143, (char)237, (char)248, (char)246, (char)219, (char)74, (char)7, (char)100, (char)236, (char)196, (char)248, (char)90, (char)2, (char)9, (char)61, (char)11, (char)103, (char)6, (char)146, (char)121, (char)228, (char)178, (char)55, (char)66, (char)181, (char)130, (char)230, (char)18, (char)218, (char)26, (char)158, (char)52, (char)137, (char)62, (char)64, (char)48, (char)247, (char)222, (char)219, (char)115, (char)219, (char)69, (char)196, (char)31, (char)120, (char)112, (char)210, (char)136, (char)249, (char)223, (char)124, (char)78, (char)252, (char)82, (char)249, (char)252, (char)110, (char)120, (char)117, (char)99, (char)5, (char)77, (char)172, (char)196, (char)122, (char)121, (char)231, (char)222, (char)127, (char)3, (char)79, (char)124, (char)135, (char)154, (char)224, (char)135, (char)196, (char)182, (char)173, (char)161, (char)55, (char)182, (char)78, (char)113, (char)10, (char)179, (char)14, (char)122, (char)63, (char)3, (char)78, (char)35, (char)161, (char)119, (char)1, (char)83, (char)98, (char)154, (char)193, (char)64, (char)196, (char)234, (char)139, (char)116, (char)30, (char)63, (char)128, (char)64, (char)180, (char)233, (char)123, (char)40, (char)23, (char)61, (char)51, (char)113, (char)20, (char)245, (char)235, (char)5, (char)157, (char)102}, 0) ;
        p267.target_system_SET((char)29) ;
        p267.length_SET((char)147) ;
        p267.target_component_SET((char)138) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)51);
            assert(pack.sequence_GET() == (char)60653);
            assert(pack.target_component_GET() == (char)130);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_component_SET((char)130) ;
        p268.sequence_SET((char)60653) ;
        p268.target_system_SET((char)51) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.framerate_GET() == 1.6501327E37F);
            assert(pack.camera_id_GET() == (char)243);
            assert(pack.uri_LEN(ph) == 216);
            assert(pack.uri_TRY(ph).equals("yxuimtbJFyyzrjcsxstcnjrturqhncsHmdnnffuzPzIgmbjcshcJwmqmzhnyphvohgdabjtnsirYjyetazuNofevqzffzJqtdtsymznpzmtczzxrsoyqztptlezqztuGctgaeyyfjGggmObflegzleHmxblwneaeprWadvatlusqRZlescgiamgJfmoddwkkZqwjsxwjokwXlboHwwnqudsL"));
            assert(pack.bitrate_GET() == 3445226255L);
            assert(pack.resolution_h_GET() == (char)20014);
            assert(pack.rotation_GET() == (char)53165);
            assert(pack.resolution_v_GET() == (char)35094);
            assert(pack.status_GET() == (char)24);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)243) ;
        p269.status_SET((char)24) ;
        p269.rotation_SET((char)53165) ;
        p269.uri_SET("yxuimtbJFyyzrjcsxstcnjrturqhncsHmdnnffuzPzIgmbjcshcJwmqmzhnyphvohgdabjtnsirYjyetazuNofevqzffzJqtdtsymznpzmtczzxrsoyqztptlezqztuGctgaeyyfjGggmObflegzleHmxblwneaeprWadvatlusqRZlescgiamgJfmoddwkkZqwjsxwjokwXlboHwwnqudsL", PH) ;
        p269.resolution_v_SET((char)35094) ;
        p269.bitrate_SET(3445226255L) ;
        p269.framerate_SET(1.6501327E37F) ;
        p269.resolution_h_SET((char)20014) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.bitrate_GET() == 3938781016L);
            assert(pack.camera_id_GET() == (char)230);
            assert(pack.framerate_GET() == -2.8895345E38F);
            assert(pack.uri_LEN(ph) == 177);
            assert(pack.uri_TRY(ph).equals("dwLmaootitiBgyUgQokzwrepIbmaaloxaezuqfhiMijexoqkqrlwycpXguflzfpuwrwfarvinandbvkveghphsgusVixaqtqslbyofhclmjmpiygVmebHumnkxfvNzfhufiUizBhpjdyouakjgbetslurkpbcsviaorrtxvnuvzstagjf"));
            assert(pack.target_component_GET() == (char)116);
            assert(pack.resolution_h_GET() == (char)64922);
            assert(pack.rotation_GET() == (char)33578);
            assert(pack.target_system_GET() == (char)61);
            assert(pack.resolution_v_GET() == (char)37535);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.uri_SET("dwLmaootitiBgyUgQokzwrepIbmaaloxaezuqfhiMijexoqkqrlwycpXguflzfpuwrwfarvinandbvkveghphsgusVixaqtqslbyofhclmjmpiygVmebHumnkxfvNzfhufiUizBhpjdyouakjgbetslurkpbcsviaorrtxvnuvzstagjf", PH) ;
        p270.bitrate_SET(3938781016L) ;
        p270.target_component_SET((char)116) ;
        p270.resolution_v_SET((char)37535) ;
        p270.resolution_h_SET((char)64922) ;
        p270.camera_id_SET((char)230) ;
        p270.target_system_SET((char)61) ;
        p270.framerate_SET(-2.8895345E38F) ;
        p270.rotation_SET((char)33578) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 14);
            assert(pack.password_TRY(ph).equals("usvflDphiKwibx"));
            assert(pack.ssid_LEN(ph) == 20);
            assert(pack.ssid_TRY(ph).equals("mrtcpvezmkddbzrUqwpc"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("usvflDphiKwibx", PH) ;
        p299.ssid_SET("mrtcpvezmkddbzrUqwpc", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.max_version_GET() == (char)15978);
            assert(pack.min_version_GET() == (char)739);
            assert(pack.version_GET() == (char)54851);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)155, (char)27, (char)176, (char)143, (char)34, (char)24, (char)40, (char)244}));
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)245, (char)119, (char)17, (char)54, (char)182, (char)173, (char)12, (char)181}));
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.library_version_hash_SET(new char[] {(char)155, (char)27, (char)176, (char)143, (char)34, (char)24, (char)40, (char)244}, 0) ;
        p300.min_version_SET((char)739) ;
        p300.max_version_SET((char)15978) ;
        p300.version_SET((char)54851) ;
        p300.spec_version_hash_SET(new char[] {(char)245, (char)119, (char)17, (char)54, (char)182, (char)173, (char)12, (char)181}, 0) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.vendor_specific_status_code_GET() == (char)57919);
            assert(pack.sub_mode_GET() == (char)226);
            assert(pack.uptime_sec_GET() == 3959955482L);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
            assert(pack.time_usec_GET() == 6003048116818872624L);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(6003048116818872624L) ;
        p310.vendor_specific_status_code_SET((char)57919) ;
        p310.uptime_sec_SET(3959955482L) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE) ;
        p310.sub_mode_SET((char)226) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 9);
            assert(pack.name_TRY(ph).equals("Ijgfyswlb"));
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)237, (char)242, (char)126, (char)244, (char)82, (char)43, (char)111, (char)136, (char)116, (char)197, (char)6, (char)185, (char)161, (char)190, (char)27, (char)250}));
            assert(pack.sw_version_minor_GET() == (char)5);
            assert(pack.uptime_sec_GET() == 3763500241L);
            assert(pack.sw_vcs_commit_GET() == 504611281L);
            assert(pack.hw_version_minor_GET() == (char)77);
            assert(pack.sw_version_major_GET() == (char)88);
            assert(pack.time_usec_GET() == 9135517068134991573L);
            assert(pack.hw_version_major_GET() == (char)20);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.name_SET("Ijgfyswlb", PH) ;
        p311.hw_version_minor_SET((char)77) ;
        p311.uptime_sec_SET(3763500241L) ;
        p311.hw_unique_id_SET(new char[] {(char)237, (char)242, (char)126, (char)244, (char)82, (char)43, (char)111, (char)136, (char)116, (char)197, (char)6, (char)185, (char)161, (char)190, (char)27, (char)250}, 0) ;
        p311.sw_version_major_SET((char)88) ;
        p311.time_usec_SET(9135517068134991573L) ;
        p311.sw_vcs_commit_SET(504611281L) ;
        p311.hw_version_major_SET((char)20) ;
        p311.sw_version_minor_SET((char)5) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 13);
            assert(pack.param_id_TRY(ph).equals("pjsgdoAoufxbm"));
            assert(pack.target_component_GET() == (char)122);
            assert(pack.param_index_GET() == (short)27556);
            assert(pack.target_system_GET() == (char)27);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_id_SET("pjsgdoAoufxbm", PH) ;
        p320.param_index_SET((short)27556) ;
        p320.target_component_SET((char)122) ;
        p320.target_system_SET((char)27) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)146);
            assert(pack.target_system_GET() == (char)48);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)146) ;
        p321.target_system_SET((char)48) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("heebljqTF"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
            assert(pack.param_index_GET() == (char)21760);
            assert(pack.param_count_GET() == (char)26177);
            assert(pack.param_value_LEN(ph) == 114);
            assert(pack.param_value_TRY(ph).equals("wmndjwgkffyxdmoFchcjTpmlyonVptxpbbztlIhjtWfuibOjnqfoueAlbxXOagJrdqpxkoDanmXvvjktjhawbCdWrwvftkhIeueKkdHzQjyDlpxhsx"));
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_count_SET((char)26177) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8) ;
        p322.param_value_SET("wmndjwgkffyxdmoFchcjTpmlyonVptxpbbztlIhjtWfuibOjnqfoueAlbxXOagJrdqpxkoDanmXvvjktjhawbCdWrwvftkhIeueKkdHzQjyDlpxhsx", PH) ;
        p322.param_index_SET((char)21760) ;
        p322.param_id_SET("heebljqTF", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)47);
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("Uhqskoy"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
            assert(pack.param_value_LEN(ph) == 116);
            assert(pack.param_value_TRY(ph).equals("kfSivxiutmgouoorVkxUmbFecdapdhqlmlsmjIJowsJswzwpsewcwtnjsiyofpsykmzocrhLbqmtalbxrhkgthbwboiewhivrhfsmlwpszuxrlnnRomo"));
            assert(pack.target_system_GET() == (char)157);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_id_SET("Uhqskoy", PH) ;
        p323.target_system_SET((char)157) ;
        p323.param_value_SET("kfSivxiutmgouoorVkxUmbFecdapdhqlmlsmjIJowsJswzwpsewcwtnjsiyofpsykmzocrhLbqmtalbxrhkgthbwboiewhivrhfsmlwpszuxrlnnRomo", PH) ;
        p323.target_component_SET((char)47) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("ipfuosovKtjmqp"));
            assert(pack.param_value_LEN(ph) == 94);
            assert(pack.param_value_TRY(ph).equals("XxNyqwfdavmFfthdweiAeoafikwhnntmmhbBbHhjhxnrqmyoaUhlgzmnregqscopOdklvznprTnyutplznkjycArdqblvi"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_IN_PROGRESS);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_IN_PROGRESS) ;
        p324.param_value_SET("XxNyqwfdavmFfthdweiAeoafikwhnntmmhbBbHhjhxnrqmyoaUhlgzmnregqscopOdklvznprTnyutplznkjycArdqblvi", PH) ;
        p324.param_id_SET("ipfuosovKtjmqp", PH) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)62528, (char)65479, (char)34516, (char)33982, (char)6814, (char)61822, (char)43345, (char)1356, (char)14643, (char)46704, (char)47544, (char)59241, (char)56523, (char)43963, (char)12307, (char)2136, (char)24488, (char)1020, (char)42396, (char)15225, (char)56935, (char)25910, (char)42307, (char)15660, (char)53334, (char)6271, (char)34042, (char)14210, (char)30093, (char)19087, (char)13680, (char)33969, (char)35739, (char)8866, (char)37125, (char)19468, (char)12419, (char)61899, (char)54215, (char)27314, (char)62106, (char)35911, (char)52275, (char)13493, (char)39052, (char)7733, (char)37270, (char)43990, (char)7582, (char)50186, (char)13148, (char)13729, (char)2249, (char)59845, (char)9045, (char)46278, (char)38639, (char)2314, (char)17093, (char)59000, (char)26184, (char)22308, (char)38786, (char)29466, (char)34949, (char)24198, (char)64970, (char)40642, (char)65518, (char)37839, (char)62443, (char)28027}));
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(pack.increment_GET() == (char)244);
            assert(pack.max_distance_GET() == (char)30399);
            assert(pack.min_distance_GET() == (char)36839);
            assert(pack.time_usec_GET() == 1543892516334643757L);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.increment_SET((char)244) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p330.max_distance_SET((char)30399) ;
        p330.time_usec_SET(1543892516334643757L) ;
        p330.min_distance_SET((char)36839) ;
        p330.distances_SET(new char[] {(char)62528, (char)65479, (char)34516, (char)33982, (char)6814, (char)61822, (char)43345, (char)1356, (char)14643, (char)46704, (char)47544, (char)59241, (char)56523, (char)43963, (char)12307, (char)2136, (char)24488, (char)1020, (char)42396, (char)15225, (char)56935, (char)25910, (char)42307, (char)15660, (char)53334, (char)6271, (char)34042, (char)14210, (char)30093, (char)19087, (char)13680, (char)33969, (char)35739, (char)8866, (char)37125, (char)19468, (char)12419, (char)61899, (char)54215, (char)27314, (char)62106, (char)35911, (char)52275, (char)13493, (char)39052, (char)7733, (char)37270, (char)43990, (char)7582, (char)50186, (char)13148, (char)13729, (char)2249, (char)59845, (char)9045, (char)46278, (char)38639, (char)2314, (char)17093, (char)59000, (char)26184, (char)22308, (char)38786, (char)29466, (char)34949, (char)24198, (char)64970, (char)40642, (char)65518, (char)37839, (char)62443, (char)28027}, 0) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
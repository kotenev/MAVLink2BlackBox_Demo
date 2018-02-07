
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
            long id = id__d(src);
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
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_EMERGENCY);
            assert(pack.mavlink_version_GET() == (char)155);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_MISSION_FULL);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_PARAFOIL);
            assert(pack.custom_mode_GET() == 3237720172L);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.mavlink_version_SET((char)155) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_PARAFOIL) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_EMERGENCY) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_MISSION_FULL) ;
        p0.custom_mode_SET(3237720172L) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_count4_GET() == (char)31258);
            assert(pack.errors_count2_GET() == (char)17799);
            assert(pack.drop_rate_comm_GET() == (char)38825);
            assert(pack.errors_comm_GET() == (char)25159);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
            assert(pack.errors_count3_GET() == (char)47393);
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
            assert(pack.load_GET() == (char)49758);
            assert(pack.current_battery_GET() == (short) -17110);
            assert(pack.voltage_battery_GET() == (char)41379);
            assert(pack.battery_remaining_GET() == (byte) - 1);
            assert(pack.errors_count1_GET() == (char)51914);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY));
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW)) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY)) ;
        p1.errors_count4_SET((char)31258) ;
        p1.errors_count3_SET((char)47393) ;
        p1.voltage_battery_SET((char)41379) ;
        p1.errors_count1_SET((char)51914) ;
        p1.load_SET((char)49758) ;
        p1.errors_count2_SET((char)17799) ;
        p1.errors_comm_SET((char)25159) ;
        p1.current_battery_SET((short) -17110) ;
        p1.drop_rate_comm_SET((char)38825) ;
        p1.battery_remaining_SET((byte) - 1) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2252549447L);
            assert(pack.time_unix_usec_GET() == 4573005934268706778L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(4573005934268706778L) ;
        p2.time_boot_ms_SET(2252549447L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.afy_GET() == 2.0724148E38F);
            assert(pack.vz_GET() == 1.0026589E38F);
            assert(pack.yaw_GET() == 2.2864772E38F);
            assert(pack.vx_GET() == 2.4715842E38F);
            assert(pack.type_mask_GET() == (char)6871);
            assert(pack.afz_GET() == -2.1153795E38F);
            assert(pack.afx_GET() == 1.3923785E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.z_GET() == -3.3053981E38F);
            assert(pack.x_GET() == -2.0180296E38F);
            assert(pack.yaw_rate_GET() == 1.604117E38F);
            assert(pack.time_boot_ms_GET() == 4165887842L);
            assert(pack.vy_GET() == 2.5404805E38F);
            assert(pack.y_GET() == -6.902183E37F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.type_mask_SET((char)6871) ;
        p3.z_SET(-3.3053981E38F) ;
        p3.time_boot_ms_SET(4165887842L) ;
        p3.afy_SET(2.0724148E38F) ;
        p3.yaw_rate_SET(1.604117E38F) ;
        p3.yaw_SET(2.2864772E38F) ;
        p3.vx_SET(2.4715842E38F) ;
        p3.y_SET(-6.902183E37F) ;
        p3.vz_SET(1.0026589E38F) ;
        p3.afz_SET(-2.1153795E38F) ;
        p3.x_SET(-2.0180296E38F) ;
        p3.afx_SET(1.3923785E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p3.vy_SET(2.5404805E38F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6544975698926556063L);
            assert(pack.target_system_GET() == (char)207);
            assert(pack.target_component_GET() == (char)237);
            assert(pack.seq_GET() == 2884480744L);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.seq_SET(2884480744L) ;
        p4.target_system_SET((char)207) ;
        p4.time_usec_SET(6544975698926556063L) ;
        p4.target_component_SET((char)237) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.passkey_LEN(ph) == 10);
            assert(pack.passkey_TRY(ph).equals("FbyyQCajqy"));
            assert(pack.target_system_GET() == (char)189);
            assert(pack.version_GET() == (char)43);
            assert(pack.control_request_GET() == (char)217);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.version_SET((char)43) ;
        p5.passkey_SET("FbyyQCajqy", PH) ;
        p5.target_system_SET((char)189) ;
        p5.control_request_SET((char)217) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.ack_GET() == (char)149);
            assert(pack.control_request_GET() == (char)102);
            assert(pack.gcs_system_id_GET() == (char)252);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.control_request_SET((char)102) ;
        p6.gcs_system_id_SET((char)252) ;
        p6.ack_SET((char)149) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 6);
            assert(pack.key_TRY(ph).equals("cidaey"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("cidaey", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_AUTO_ARMED);
            assert(pack.custom_mode_GET() == 3311626837L);
            assert(pack.target_system_GET() == (char)132);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED) ;
        p11.target_system_SET((char)132) ;
        p11.custom_mode_SET(3311626837L) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)83);
            assert(pack.param_index_GET() == (short) -26883);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("ehfp"));
            assert(pack.target_component_GET() == (char)107);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_id_SET("ehfp", PH) ;
        p20.param_index_SET((short) -26883) ;
        p20.target_system_SET((char)83) ;
        p20.target_component_SET((char)107) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)143);
            assert(pack.target_system_GET() == (char)83);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)83) ;
        p21.target_component_SET((char)143) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("wjsmp"));
            assert(pack.param_index_GET() == (char)4378);
            assert(pack.param_count_GET() == (char)45028);
            assert(pack.param_value_GET() == 3.237798E38F);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_id_SET("wjsmp", PH) ;
        p22.param_index_SET((char)4378) ;
        p22.param_value_SET(3.237798E38F) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32) ;
        p22.param_count_SET((char)45028) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)6);
            assert(pack.target_system_GET() == (char)210);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64);
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("cgribyg"));
            assert(pack.param_value_GET() == -1.788634E38F);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.param_id_SET("cgribyg", PH) ;
        p23.param_value_SET(-1.788634E38F) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64) ;
        p23.target_component_SET((char)6) ;
        p23.target_system_SET((char)210) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -493140742);
            assert(pack.cog_GET() == (char)13163);
            assert(pack.eph_GET() == (char)31252);
            assert(pack.vel_GET() == (char)16188);
            assert(pack.satellites_visible_GET() == (char)213);
            assert(pack.hdg_acc_TRY(ph) == 3741750366L);
            assert(pack.alt_GET() == 1889235737);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
            assert(pack.vel_acc_TRY(ph) == 1189623745L);
            assert(pack.v_acc_TRY(ph) == 2660542994L);
            assert(pack.alt_ellipsoid_TRY(ph) == -925956885);
            assert(pack.h_acc_TRY(ph) == 425392378L);
            assert(pack.epv_GET() == (char)42162);
            assert(pack.lat_GET() == 1405691432);
            assert(pack.time_usec_GET() == 6240242705363265418L);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.hdg_acc_SET(3741750366L, PH) ;
        p24.h_acc_SET(425392378L, PH) ;
        p24.alt_SET(1889235737) ;
        p24.time_usec_SET(6240242705363265418L) ;
        p24.eph_SET((char)31252) ;
        p24.cog_SET((char)13163) ;
        p24.vel_acc_SET(1189623745L, PH) ;
        p24.lon_SET(-493140742) ;
        p24.lat_SET(1405691432) ;
        p24.v_acc_SET(2660542994L, PH) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP) ;
        p24.epv_SET((char)42162) ;
        p24.vel_SET((char)16188) ;
        p24.satellites_visible_SET((char)213) ;
        p24.alt_ellipsoid_SET(-925956885, PH) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)133, (char)235, (char)149, (char)194, (char)113, (char)195, (char)222, (char)125, (char)166, (char)194, (char)32, (char)15, (char)16, (char)48, (char)212, (char)85, (char)95, (char)123, (char)60, (char)202}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)190, (char)242, (char)11, (char)72, (char)237, (char)230, (char)47, (char)89, (char)239, (char)74, (char)85, (char)106, (char)95, (char)27, (char)80, (char)2, (char)32, (char)174, (char)189, (char)253}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)91, (char)191, (char)177, (char)136, (char)25, (char)208, (char)226, (char)145, (char)5, (char)50, (char)236, (char)52, (char)4, (char)169, (char)82, (char)15, (char)129, (char)187, (char)178, (char)88}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)8, (char)78, (char)236, (char)168, (char)72, (char)167, (char)24, (char)167, (char)0, (char)131, (char)72, (char)54, (char)234, (char)74, (char)20, (char)141, (char)102, (char)122, (char)178, (char)218}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)201, (char)167, (char)169, (char)140, (char)117, (char)43, (char)14, (char)195, (char)45, (char)217, (char)30, (char)107, (char)243, (char)219, (char)92, (char)58, (char)16, (char)168, (char)41, (char)30}));
            assert(pack.satellites_visible_GET() == (char)0);
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)0) ;
        p25.satellite_elevation_SET(new char[] {(char)190, (char)242, (char)11, (char)72, (char)237, (char)230, (char)47, (char)89, (char)239, (char)74, (char)85, (char)106, (char)95, (char)27, (char)80, (char)2, (char)32, (char)174, (char)189, (char)253}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)91, (char)191, (char)177, (char)136, (char)25, (char)208, (char)226, (char)145, (char)5, (char)50, (char)236, (char)52, (char)4, (char)169, (char)82, (char)15, (char)129, (char)187, (char)178, (char)88}, 0) ;
        p25.satellite_used_SET(new char[] {(char)201, (char)167, (char)169, (char)140, (char)117, (char)43, (char)14, (char)195, (char)45, (char)217, (char)30, (char)107, (char)243, (char)219, (char)92, (char)58, (char)16, (char)168, (char)41, (char)30}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)8, (char)78, (char)236, (char)168, (char)72, (char)167, (char)24, (char)167, (char)0, (char)131, (char)72, (char)54, (char)234, (char)74, (char)20, (char)141, (char)102, (char)122, (char)178, (char)218}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)133, (char)235, (char)149, (char)194, (char)113, (char)195, (char)222, (char)125, (char)166, (char)194, (char)32, (char)15, (char)16, (char)48, (char)212, (char)85, (char)95, (char)123, (char)60, (char)202}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short)23204);
            assert(pack.ymag_GET() == (short) -28376);
            assert(pack.xmag_GET() == (short) -9933);
            assert(pack.zmag_GET() == (short) -57);
            assert(pack.xacc_GET() == (short) -5966);
            assert(pack.yacc_GET() == (short)3793);
            assert(pack.time_boot_ms_GET() == 2531095335L);
            assert(pack.xgyro_GET() == (short) -19544);
            assert(pack.zacc_GET() == (short) -29448);
            assert(pack.zgyro_GET() == (short) -4896);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.ymag_SET((short) -28376) ;
        p26.xacc_SET((short) -5966) ;
        p26.zmag_SET((short) -57) ;
        p26.yacc_SET((short)3793) ;
        p26.xmag_SET((short) -9933) ;
        p26.zacc_SET((short) -29448) ;
        p26.zgyro_SET((short) -4896) ;
        p26.ygyro_SET((short)23204) ;
        p26.time_boot_ms_SET(2531095335L) ;
        p26.xgyro_SET((short) -19544) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short) -23212);
            assert(pack.xmag_GET() == (short)15891);
            assert(pack.xacc_GET() == (short) -21764);
            assert(pack.xgyro_GET() == (short) -9903);
            assert(pack.yacc_GET() == (short) -778);
            assert(pack.time_usec_GET() == 8765270872712362611L);
            assert(pack.ymag_GET() == (short) -22486);
            assert(pack.zgyro_GET() == (short)8564);
            assert(pack.ygyro_GET() == (short)30231);
            assert(pack.zmag_GET() == (short)20801);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.xmag_SET((short)15891) ;
        p27.ymag_SET((short) -22486) ;
        p27.ygyro_SET((short)30231) ;
        p27.zacc_SET((short) -23212) ;
        p27.zmag_SET((short)20801) ;
        p27.time_usec_SET(8765270872712362611L) ;
        p27.zgyro_SET((short)8564) ;
        p27.yacc_SET((short) -778) ;
        p27.xacc_SET((short) -21764) ;
        p27.xgyro_SET((short) -9903) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff1_GET() == (short)7101);
            assert(pack.press_abs_GET() == (short)2754);
            assert(pack.press_diff2_GET() == (short) -29088);
            assert(pack.temperature_GET() == (short)18002);
            assert(pack.time_usec_GET() == 1974978652562487236L);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_abs_SET((short)2754) ;
        p28.temperature_SET((short)18002) ;
        p28.press_diff1_SET((short)7101) ;
        p28.press_diff2_SET((short) -29088) ;
        p28.time_usec_SET(1974978652562487236L) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -8781);
            assert(pack.time_boot_ms_GET() == 2774823677L);
            assert(pack.press_abs_GET() == 3.1850918E38F);
            assert(pack.press_diff_GET() == 3.3683316E38F);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.time_boot_ms_SET(2774823677L) ;
        p29.press_diff_SET(3.3683316E38F) ;
        p29.temperature_SET((short) -8781) ;
        p29.press_abs_SET(3.1850918E38F) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3108548670L);
            assert(pack.pitchspeed_GET() == -2.3787504E38F);
            assert(pack.rollspeed_GET() == -1.1830076E38F);
            assert(pack.roll_GET() == -2.744E38F);
            assert(pack.yaw_GET() == -1.8249487E38F);
            assert(pack.yawspeed_GET() == 1.5796891E38F);
            assert(pack.pitch_GET() == -2.8685998E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.pitchspeed_SET(-2.3787504E38F) ;
        p30.rollspeed_SET(-1.1830076E38F) ;
        p30.yawspeed_SET(1.5796891E38F) ;
        p30.roll_SET(-2.744E38F) ;
        p30.pitch_SET(-2.8685998E38F) ;
        p30.time_boot_ms_SET(3108548670L) ;
        p30.yaw_SET(-1.8249487E38F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == 9.551254E37F);
            assert(pack.q2_GET() == 1.425348E38F);
            assert(pack.q3_GET() == -3.1344856E38F);
            assert(pack.yawspeed_GET() == -1.8586455E38F);
            assert(pack.q1_GET() == 2.6744357E37F);
            assert(pack.time_boot_ms_GET() == 1435271141L);
            assert(pack.q4_GET() == -7.2500077E37F);
            assert(pack.pitchspeed_GET() == -2.9746484E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q1_SET(2.6744357E37F) ;
        p31.q4_SET(-7.2500077E37F) ;
        p31.rollspeed_SET(9.551254E37F) ;
        p31.pitchspeed_SET(-2.9746484E38F) ;
        p31.time_boot_ms_SET(1435271141L) ;
        p31.yawspeed_SET(-1.8586455E38F) ;
        p31.q3_SET(-3.1344856E38F) ;
        p31.q2_SET(1.425348E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == 9.54317E37F);
            assert(pack.y_GET() == 2.7758751E38F);
            assert(pack.time_boot_ms_GET() == 2367086150L);
            assert(pack.vx_GET() == 2.2038592E38F);
            assert(pack.x_GET() == -2.1406496E37F);
            assert(pack.z_GET() == -2.1716786E38F);
            assert(pack.vy_GET() == 2.542102E38F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.vy_SET(2.542102E38F) ;
        p32.vx_SET(2.2038592E38F) ;
        p32.vz_SET(9.54317E37F) ;
        p32.y_SET(2.7758751E38F) ;
        p32.x_SET(-2.1406496E37F) ;
        p32.z_SET(-2.1716786E38F) ;
        p32.time_boot_ms_SET(2367086150L) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == (short) -30986);
            assert(pack.vz_GET() == (short) -15679);
            assert(pack.time_boot_ms_GET() == 1420381410L);
            assert(pack.vy_GET() == (short) -16446);
            assert(pack.lat_GET() == -886670727);
            assert(pack.lon_GET() == 1560614284);
            assert(pack.relative_alt_GET() == 1445106894);
            assert(pack.alt_GET() == 225207866);
            assert(pack.hdg_GET() == (char)47690);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.lat_SET(-886670727) ;
        p33.vz_SET((short) -15679) ;
        p33.hdg_SET((char)47690) ;
        p33.vy_SET((short) -16446) ;
        p33.lon_SET(1560614284) ;
        p33.time_boot_ms_SET(1420381410L) ;
        p33.relative_alt_SET(1445106894) ;
        p33.vx_SET((short) -30986) ;
        p33.alt_SET(225207866) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan3_scaled_GET() == (short) -20734);
            assert(pack.rssi_GET() == (char)116);
            assert(pack.time_boot_ms_GET() == 1256599948L);
            assert(pack.chan8_scaled_GET() == (short)1355);
            assert(pack.chan5_scaled_GET() == (short)19667);
            assert(pack.chan2_scaled_GET() == (short) -4938);
            assert(pack.chan6_scaled_GET() == (short) -28875);
            assert(pack.chan1_scaled_GET() == (short) -5870);
            assert(pack.port_GET() == (char)168);
            assert(pack.chan7_scaled_GET() == (short)15602);
            assert(pack.chan4_scaled_GET() == (short)16985);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.rssi_SET((char)116) ;
        p34.chan3_scaled_SET((short) -20734) ;
        p34.chan8_scaled_SET((short)1355) ;
        p34.chan4_scaled_SET((short)16985) ;
        p34.chan1_scaled_SET((short) -5870) ;
        p34.chan5_scaled_SET((short)19667) ;
        p34.port_SET((char)168) ;
        p34.time_boot_ms_SET(1256599948L) ;
        p34.chan2_scaled_SET((short) -4938) ;
        p34.chan6_scaled_SET((short) -28875) ;
        p34.chan7_scaled_SET((short)15602) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)61836);
            assert(pack.chan7_raw_GET() == (char)57565);
            assert(pack.chan6_raw_GET() == (char)17290);
            assert(pack.chan2_raw_GET() == (char)39216);
            assert(pack.port_GET() == (char)139);
            assert(pack.chan8_raw_GET() == (char)22761);
            assert(pack.chan3_raw_GET() == (char)3591);
            assert(pack.chan1_raw_GET() == (char)6105);
            assert(pack.chan5_raw_GET() == (char)55191);
            assert(pack.rssi_GET() == (char)114);
            assert(pack.time_boot_ms_GET() == 1715901734L);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan6_raw_SET((char)17290) ;
        p35.port_SET((char)139) ;
        p35.chan5_raw_SET((char)55191) ;
        p35.chan4_raw_SET((char)61836) ;
        p35.chan3_raw_SET((char)3591) ;
        p35.chan2_raw_SET((char)39216) ;
        p35.time_boot_ms_SET(1715901734L) ;
        p35.rssi_SET((char)114) ;
        p35.chan7_raw_SET((char)57565) ;
        p35.chan8_raw_SET((char)22761) ;
        p35.chan1_raw_SET((char)6105) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo12_raw_TRY(ph) == (char)37146);
            assert(pack.servo2_raw_GET() == (char)57720);
            assert(pack.time_usec_GET() == 1707737960L);
            assert(pack.servo4_raw_GET() == (char)59789);
            assert(pack.servo10_raw_TRY(ph) == (char)14603);
            assert(pack.servo13_raw_TRY(ph) == (char)41290);
            assert(pack.port_GET() == (char)247);
            assert(pack.servo3_raw_GET() == (char)30901);
            assert(pack.servo11_raw_TRY(ph) == (char)48319);
            assert(pack.servo5_raw_GET() == (char)28912);
            assert(pack.servo7_raw_GET() == (char)18306);
            assert(pack.servo8_raw_GET() == (char)46824);
            assert(pack.servo6_raw_GET() == (char)12071);
            assert(pack.servo14_raw_TRY(ph) == (char)54978);
            assert(pack.servo15_raw_TRY(ph) == (char)64826);
            assert(pack.servo16_raw_TRY(ph) == (char)56041);
            assert(pack.servo9_raw_TRY(ph) == (char)5851);
            assert(pack.servo1_raw_GET() == (char)17481);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo3_raw_SET((char)30901) ;
        p36.servo14_raw_SET((char)54978, PH) ;
        p36.port_SET((char)247) ;
        p36.servo10_raw_SET((char)14603, PH) ;
        p36.servo7_raw_SET((char)18306) ;
        p36.servo15_raw_SET((char)64826, PH) ;
        p36.servo4_raw_SET((char)59789) ;
        p36.servo11_raw_SET((char)48319, PH) ;
        p36.time_usec_SET(1707737960L) ;
        p36.servo16_raw_SET((char)56041, PH) ;
        p36.servo8_raw_SET((char)46824) ;
        p36.servo6_raw_SET((char)12071) ;
        p36.servo1_raw_SET((char)17481) ;
        p36.servo5_raw_SET((char)28912) ;
        p36.servo9_raw_SET((char)5851, PH) ;
        p36.servo13_raw_SET((char)41290, PH) ;
        p36.servo2_raw_SET((char)57720) ;
        p36.servo12_raw_SET((char)37146, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short) -14629);
            assert(pack.target_system_GET() == (char)105);
            assert(pack.target_component_GET() == (char)186);
            assert(pack.start_index_GET() == (short) -25690);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_component_SET((char)186) ;
        p37.start_index_SET((short) -25690) ;
        p37.end_index_SET((short) -14629) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p37.target_system_SET((char)105) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)53);
            assert(pack.start_index_GET() == (short)6531);
            assert(pack.target_component_GET() == (char)115);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.end_index_GET() == (short)17743);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.start_index_SET((short)6531) ;
        p38.end_index_SET((short)17743) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p38.target_system_SET((char)53) ;
        p38.target_component_SET((char)115) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.param2_GET() == 1.6079138E38F);
            assert(pack.param1_GET() == -2.856523E38F);
            assert(pack.y_GET() == -5.29129E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_USER_4);
            assert(pack.current_GET() == (char)220);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.autocontinue_GET() == (char)27);
            assert(pack.seq_GET() == (char)52462);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)125);
            assert(pack.x_GET() == -1.9356828E37F);
            assert(pack.param3_GET() == 2.8622946E38F);
            assert(pack.target_system_GET() == (char)213);
            assert(pack.z_GET() == 2.1364288E38F);
            assert(pack.param4_GET() == -3.2171161E38F);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.autocontinue_SET((char)27) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p39.z_SET(2.1364288E38F) ;
        p39.seq_SET((char)52462) ;
        p39.param4_SET(-3.2171161E38F) ;
        p39.param1_SET(-2.856523E38F) ;
        p39.target_component_SET((char)125) ;
        p39.target_system_SET((char)213) ;
        p39.current_SET((char)220) ;
        p39.y_SET(-5.29129E37F) ;
        p39.x_SET(-1.9356828E37F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p39.param3_SET(2.8622946E38F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_USER_4) ;
        p39.param2_SET(1.6079138E38F) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)22);
            assert(pack.target_component_GET() == (char)18);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.seq_GET() == (char)48222);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p40.seq_SET((char)48222) ;
        p40.target_system_SET((char)22) ;
        p40.target_component_SET((char)18) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)29573);
            assert(pack.target_component_GET() == (char)113);
            assert(pack.target_system_GET() == (char)178);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_component_SET((char)113) ;
        p41.target_system_SET((char)178) ;
        p41.seq_SET((char)29573) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)46283);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)46283) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)126);
            assert(pack.target_component_GET() == (char)177);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p43.target_component_SET((char)177) ;
        p43.target_system_SET((char)126) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)203);
            assert(pack.target_component_GET() == (char)148);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.count_GET() == (char)41549);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_component_SET((char)148) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p44.count_SET((char)41549) ;
        p44.target_system_SET((char)203) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)1);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)154);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_component_SET((char)1) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p45.target_system_SET((char)154) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)58127);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)58127) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)104);
            assert(pack.target_system_GET() == (char)84);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.target_component_SET((char)104) ;
        p47.target_system_SET((char)84) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)125);
            assert(pack.time_usec_TRY(ph) == 9214506817666480969L);
            assert(pack.latitude_GET() == 75582533);
            assert(pack.altitude_GET() == -1752261518);
            assert(pack.longitude_GET() == -2002091179);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.target_system_SET((char)125) ;
        p48.latitude_SET(75582533) ;
        p48.altitude_SET(-1752261518) ;
        p48.time_usec_SET(9214506817666480969L, PH) ;
        p48.longitude_SET(-2002091179) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 3636749710630029375L);
            assert(pack.longitude_GET() == 1622825220);
            assert(pack.altitude_GET() == 841406695);
            assert(pack.latitude_GET() == 561870259);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.longitude_SET(1622825220) ;
        p49.time_usec_SET(3636749710630029375L, PH) ;
        p49.latitude_SET(561870259) ;
        p49.altitude_SET(841406695) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)33);
            assert(pack.scale_GET() == -3.232749E38F);
            assert(pack.param_id_LEN(ph) == 6);
            assert(pack.param_id_TRY(ph).equals("lkrogm"));
            assert(pack.param_value_min_GET() == 1.0274818E38F);
            assert(pack.param_index_GET() == (short) -3070);
            assert(pack.parameter_rc_channel_index_GET() == (char)152);
            assert(pack.param_value0_GET() == -1.4139467E38F);
            assert(pack.param_value_max_GET() == -1.3736557E38F);
            assert(pack.target_component_GET() == (char)131);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_system_SET((char)33) ;
        p50.param_value_min_SET(1.0274818E38F) ;
        p50.scale_SET(-3.232749E38F) ;
        p50.param_value_max_SET(-1.3736557E38F) ;
        p50.param_id_SET("lkrogm", PH) ;
        p50.param_index_SET((short) -3070) ;
        p50.target_component_SET((char)131) ;
        p50.param_value0_SET(-1.4139467E38F) ;
        p50.parameter_rc_channel_index_SET((char)152) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)54);
            assert(pack.seq_GET() == (char)9344);
            assert(pack.target_component_GET() == (char)39);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p51.target_component_SET((char)39) ;
        p51.seq_SET((char)9344) ;
        p51.target_system_SET((char)54) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.target_component_GET() == (char)13);
            assert(pack.p1x_GET() == 1.727585E38F);
            assert(pack.p2x_GET() == -3.2501983E38F);
            assert(pack.p1z_GET() == 5.0010266E37F);
            assert(pack.target_system_GET() == (char)27);
            assert(pack.p1y_GET() == -2.7253443E37F);
            assert(pack.p2y_GET() == 2.7811672E38F);
            assert(pack.p2z_GET() == -2.9104094E38F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.target_system_SET((char)27) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p54.p1z_SET(5.0010266E37F) ;
        p54.p1y_SET(-2.7253443E37F) ;
        p54.p1x_SET(1.727585E38F) ;
        p54.p2x_SET(-3.2501983E38F) ;
        p54.target_component_SET((char)13) ;
        p54.p2y_SET(2.7811672E38F) ;
        p54.p2z_SET(-2.9104094E38F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1y_GET() == -3.0901904E37F);
            assert(pack.p1z_GET() == -2.4569878E38F);
            assert(pack.p2y_GET() == -3.1087472E38F);
            assert(pack.p1x_GET() == -2.536967E38F);
            assert(pack.p2x_GET() == 3.1691633E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.p2z_GET() == 1.1847408E37F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p1x_SET(-2.536967E38F) ;
        p55.p2z_SET(1.1847408E37F) ;
        p55.p2x_SET(3.1691633E37F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p55.p1y_SET(-3.0901904E37F) ;
        p55.p2y_SET(-3.1087472E38F) ;
        p55.p1z_SET(-2.4569878E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.pitchspeed_GET() == -2.1282355E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.2551172E38F, 3.4180277E37F, -2.3944305E38F, -9.862985E37F}));
            assert(pack.yawspeed_GET() == 3.3912252E38F);
            assert(pack.time_usec_GET() == 6021198849802839904L);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.7794426E38F, -3.165594E38F, 1.6255593E38F, 1.02304956E37F, -2.2893784E38F, -3.4444693E37F, -2.2096923E38F, 1.5324408E38F, 1.3220274E38F}));
            assert(pack.rollspeed_GET() == -1.9005082E38F);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.pitchspeed_SET(-2.1282355E38F) ;
        p61.yawspeed_SET(3.3912252E38F) ;
        p61.covariance_SET(new float[] {-2.7794426E38F, -3.165594E38F, 1.6255593E38F, 1.02304956E37F, -2.2893784E38F, -3.4444693E37F, -2.2096923E38F, 1.5324408E38F, 1.3220274E38F}, 0) ;
        p61.rollspeed_SET(-1.9005082E38F) ;
        p61.q_SET(new float[] {-2.2551172E38F, 3.4180277E37F, -2.3944305E38F, -9.862985E37F}, 0) ;
        p61.time_usec_SET(6021198849802839904L) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.wp_dist_GET() == (char)6823);
            assert(pack.alt_error_GET() == 1.00776435E37F);
            assert(pack.aspd_error_GET() == -2.210822E38F);
            assert(pack.nav_pitch_GET() == -3.3718648E38F);
            assert(pack.target_bearing_GET() == (short)2869);
            assert(pack.nav_roll_GET() == -1.2681298E38F);
            assert(pack.nav_bearing_GET() == (short)19835);
            assert(pack.xtrack_error_GET() == -3.4009094E38F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_bearing_SET((short)19835) ;
        p62.aspd_error_SET(-2.210822E38F) ;
        p62.nav_pitch_SET(-3.3718648E38F) ;
        p62.xtrack_error_SET(-3.4009094E38F) ;
        p62.wp_dist_SET((char)6823) ;
        p62.nav_roll_SET(-1.2681298E38F) ;
        p62.alt_error_SET(1.00776435E37F) ;
        p62.target_bearing_SET((short)2869) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == 3.6661013E36F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
            assert(pack.vy_GET() == -3.043557E38F);
            assert(pack.relative_alt_GET() == -1524843643);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-3.1121738E36F, 1.015361E38F, -2.515325E38F, -3.3295249E38F, 3.2008099E38F, -3.1755408E38F, 1.6490259E38F, -2.7178707E38F, -1.4666724E38F, -3.209916E37F, 1.8886087E38F, -2.9695786E38F, -3.0636161E37F, 3.1305204E38F, -1.7910305E38F, 1.372666E38F, -1.1829859E38F, -2.9553063E37F, -1.0611753E38F, 1.981534E38F, 1.7639784E38F, -2.3131092E38F, 2.646708E35F, 2.8729415E38F, -1.732294E38F, -2.4579808E38F, -1.6193448E38F, 2.240282E38F, -2.9638602E38F, -3.2979673E38F, -2.1764524E38F, 4.691564E37F, 8.081847E37F, 2.0356102E38F, -1.4243605E38F, 2.4814834E38F}));
            assert(pack.lon_GET() == -143200726);
            assert(pack.alt_GET() == 620834934);
            assert(pack.lat_GET() == 1298726345);
            assert(pack.vz_GET() == -2.482372E38F);
            assert(pack.time_usec_GET() == 7780961462629480224L);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.vz_SET(-2.482372E38F) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION) ;
        p63.lat_SET(1298726345) ;
        p63.covariance_SET(new float[] {-3.1121738E36F, 1.015361E38F, -2.515325E38F, -3.3295249E38F, 3.2008099E38F, -3.1755408E38F, 1.6490259E38F, -2.7178707E38F, -1.4666724E38F, -3.209916E37F, 1.8886087E38F, -2.9695786E38F, -3.0636161E37F, 3.1305204E38F, -1.7910305E38F, 1.372666E38F, -1.1829859E38F, -2.9553063E37F, -1.0611753E38F, 1.981534E38F, 1.7639784E38F, -2.3131092E38F, 2.646708E35F, 2.8729415E38F, -1.732294E38F, -2.4579808E38F, -1.6193448E38F, 2.240282E38F, -2.9638602E38F, -3.2979673E38F, -2.1764524E38F, 4.691564E37F, 8.081847E37F, 2.0356102E38F, -1.4243605E38F, 2.4814834E38F}, 0) ;
        p63.alt_SET(620834934) ;
        p63.vy_SET(-3.043557E38F) ;
        p63.lon_SET(-143200726) ;
        p63.vx_SET(3.6661013E36F) ;
        p63.time_usec_SET(7780961462629480224L) ;
        p63.relative_alt_SET(-1524843643) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -8.601157E37F);
            assert(pack.vy_GET() == 2.9513808E38F);
            assert(pack.ay_GET() == 2.6795702E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.8978679E38F, -1.2979155E38F, -2.8823111E38F, 1.7322232E38F, 1.6225486E38F, -2.012695E38F, 1.7018678E38F, -8.4353593E37F, 2.08291E38F, -8.1995505E37F, 2.91664E38F, -1.2863646E38F, 1.1135834E38F, -7.4186797E37F, -5.4565964E37F, 1.2742753E38F, -3.3654345E38F, -2.3010574E38F, -1.3759402E37F, 1.4283832E38F, -7.274718E37F, 2.6449954E38F, 3.365451E38F, 2.9412465E38F, -3.1146368E38F, -2.767253E38F, -3.3101615E38F, 1.5535315E38F, -2.8362763E38F, 6.3418313E37F, 1.3771689E38F, -1.4135081E38F, 2.9241728E38F, 1.1486264E38F, 1.5031672E38F, 1.7466284E38F, -3.2413304E37F, -3.2950947E38F, 2.3568815E38F, 2.5252062E38F, 1.9964862E38F, -1.7671485E38F, 6.9004424E37F, 1.9904354E38F, 3.3237148E38F}));
            assert(pack.z_GET() == 3.3470515E38F);
            assert(pack.time_usec_GET() == 2599263915006298233L);
            assert(pack.y_GET() == -1.0191009E38F);
            assert(pack.az_GET() == -3.3567714E38F);
            assert(pack.vx_GET() == 1.4991845E38F);
            assert(pack.vz_GET() == 2.9358283E37F);
            assert(pack.ax_GET() == 2.3502569E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.vx_SET(1.4991845E38F) ;
        p64.az_SET(-3.3567714E38F) ;
        p64.vy_SET(2.9513808E38F) ;
        p64.vz_SET(2.9358283E37F) ;
        p64.covariance_SET(new float[] {1.8978679E38F, -1.2979155E38F, -2.8823111E38F, 1.7322232E38F, 1.6225486E38F, -2.012695E38F, 1.7018678E38F, -8.4353593E37F, 2.08291E38F, -8.1995505E37F, 2.91664E38F, -1.2863646E38F, 1.1135834E38F, -7.4186797E37F, -5.4565964E37F, 1.2742753E38F, -3.3654345E38F, -2.3010574E38F, -1.3759402E37F, 1.4283832E38F, -7.274718E37F, 2.6449954E38F, 3.365451E38F, 2.9412465E38F, -3.1146368E38F, -2.767253E38F, -3.3101615E38F, 1.5535315E38F, -2.8362763E38F, 6.3418313E37F, 1.3771689E38F, -1.4135081E38F, 2.9241728E38F, 1.1486264E38F, 1.5031672E38F, 1.7466284E38F, -3.2413304E37F, -3.2950947E38F, 2.3568815E38F, 2.5252062E38F, 1.9964862E38F, -1.7671485E38F, 6.9004424E37F, 1.9904354E38F, 3.3237148E38F}, 0) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p64.y_SET(-1.0191009E38F) ;
        p64.ax_SET(2.3502569E38F) ;
        p64.x_SET(-8.601157E37F) ;
        p64.time_usec_SET(2599263915006298233L) ;
        p64.z_SET(3.3470515E38F) ;
        p64.ay_SET(2.6795702E38F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan15_raw_GET() == (char)21558);
            assert(pack.chan2_raw_GET() == (char)25238);
            assert(pack.time_boot_ms_GET() == 441433777L);
            assert(pack.chan7_raw_GET() == (char)40743);
            assert(pack.chan6_raw_GET() == (char)21366);
            assert(pack.chan9_raw_GET() == (char)35925);
            assert(pack.chan16_raw_GET() == (char)40876);
            assert(pack.chan14_raw_GET() == (char)7998);
            assert(pack.chan1_raw_GET() == (char)46237);
            assert(pack.chancount_GET() == (char)59);
            assert(pack.chan3_raw_GET() == (char)2513);
            assert(pack.chan13_raw_GET() == (char)27953);
            assert(pack.chan18_raw_GET() == (char)61753);
            assert(pack.chan17_raw_GET() == (char)56226);
            assert(pack.chan4_raw_GET() == (char)28198);
            assert(pack.chan5_raw_GET() == (char)37870);
            assert(pack.chan8_raw_GET() == (char)34690);
            assert(pack.chan11_raw_GET() == (char)5777);
            assert(pack.chan10_raw_GET() == (char)1882);
            assert(pack.chan12_raw_GET() == (char)35282);
            assert(pack.rssi_GET() == (char)87);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan4_raw_SET((char)28198) ;
        p65.chan10_raw_SET((char)1882) ;
        p65.chan13_raw_SET((char)27953) ;
        p65.time_boot_ms_SET(441433777L) ;
        p65.chan18_raw_SET((char)61753) ;
        p65.chan6_raw_SET((char)21366) ;
        p65.chan8_raw_SET((char)34690) ;
        p65.chan15_raw_SET((char)21558) ;
        p65.chan1_raw_SET((char)46237) ;
        p65.chan11_raw_SET((char)5777) ;
        p65.chan2_raw_SET((char)25238) ;
        p65.chan12_raw_SET((char)35282) ;
        p65.chan16_raw_SET((char)40876) ;
        p65.chan7_raw_SET((char)40743) ;
        p65.chan3_raw_SET((char)2513) ;
        p65.chan5_raw_SET((char)37870) ;
        p65.rssi_SET((char)87) ;
        p65.chan17_raw_SET((char)56226) ;
        p65.chancount_SET((char)59) ;
        p65.chan14_raw_SET((char)7998) ;
        p65.chan9_raw_SET((char)35925) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)109);
            assert(pack.req_message_rate_GET() == (char)35599);
            assert(pack.target_component_GET() == (char)239);
            assert(pack.start_stop_GET() == (char)139);
            assert(pack.req_stream_id_GET() == (char)65);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_stream_id_SET((char)65) ;
        p66.start_stop_SET((char)139) ;
        p66.req_message_rate_SET((char)35599) ;
        p66.target_system_SET((char)109) ;
        p66.target_component_SET((char)239) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.message_rate_GET() == (char)63741);
            assert(pack.stream_id_GET() == (char)49);
            assert(pack.on_off_GET() == (char)62);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)49) ;
        p67.on_off_SET((char)62) ;
        p67.message_rate_SET((char)63741) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_GET() == (char)240);
            assert(pack.r_GET() == (short) -27132);
            assert(pack.buttons_GET() == (char)899);
            assert(pack.y_GET() == (short) -29273);
            assert(pack.z_GET() == (short)29142);
            assert(pack.x_GET() == (short) -29264);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.y_SET((short) -29273) ;
        p69.x_SET((short) -29264) ;
        p69.buttons_SET((char)899) ;
        p69.z_SET((short)29142) ;
        p69.target_SET((char)240) ;
        p69.r_SET((short) -27132) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan5_raw_GET() == (char)26879);
            assert(pack.chan2_raw_GET() == (char)19720);
            assert(pack.target_component_GET() == (char)203);
            assert(pack.chan3_raw_GET() == (char)51263);
            assert(pack.chan4_raw_GET() == (char)43148);
            assert(pack.chan8_raw_GET() == (char)37316);
            assert(pack.chan1_raw_GET() == (char)47251);
            assert(pack.chan7_raw_GET() == (char)21669);
            assert(pack.target_system_GET() == (char)22);
            assert(pack.chan6_raw_GET() == (char)43601);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.target_system_SET((char)22) ;
        p70.chan8_raw_SET((char)37316) ;
        p70.chan2_raw_SET((char)19720) ;
        p70.chan5_raw_SET((char)26879) ;
        p70.target_component_SET((char)203) ;
        p70.chan3_raw_SET((char)51263) ;
        p70.chan4_raw_SET((char)43148) ;
        p70.chan6_raw_SET((char)43601) ;
        p70.chan7_raw_SET((char)21669) ;
        p70.chan1_raw_SET((char)47251) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param3_GET() == -1.8174982E38F);
            assert(pack.z_GET() == -9.470966E37F);
            assert(pack.current_GET() == (char)207);
            assert(pack.x_GET() == -1684003199);
            assert(pack.autocontinue_GET() == (char)24);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST);
            assert(pack.seq_GET() == (char)25489);
            assert(pack.param2_GET() == -2.9424704E38F);
            assert(pack.target_component_GET() == (char)194);
            assert(pack.y_GET() == 647966989);
            assert(pack.target_system_GET() == (char)164);
            assert(pack.param4_GET() == 7.777031E37F);
            assert(pack.param1_GET() == -1.5677837E38F);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.param4_SET(7.777031E37F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p73.param3_SET(-1.8174982E38F) ;
        p73.command_SET(MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST) ;
        p73.param2_SET(-2.9424704E38F) ;
        p73.current_SET((char)207) ;
        p73.y_SET(647966989) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p73.param1_SET(-1.5677837E38F) ;
        p73.target_component_SET((char)194) ;
        p73.target_system_SET((char)164) ;
        p73.z_SET(-9.470966E37F) ;
        p73.autocontinue_SET((char)24) ;
        p73.x_SET(-1684003199) ;
        p73.seq_SET((char)25489) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.groundspeed_GET() == -6.597048E37F);
            assert(pack.airspeed_GET() == -4.7303523E37F);
            assert(pack.climb_GET() == 2.0128257E38F);
            assert(pack.alt_GET() == -1.5948731E38F);
            assert(pack.throttle_GET() == (char)56917);
            assert(pack.heading_GET() == (short)32057);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.airspeed_SET(-4.7303523E37F) ;
        p74.groundspeed_SET(-6.597048E37F) ;
        p74.alt_SET(-1.5948731E38F) ;
        p74.throttle_SET((char)56917) ;
        p74.heading_SET((short)32057) ;
        p74.climb_SET(2.0128257E38F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)11);
            assert(pack.z_GET() == 1.9505839E38F);
            assert(pack.param4_GET() == 5.47226E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.param3_GET() == 3.344978E38F);
            assert(pack.param2_GET() == 2.333693E38F);
            assert(pack.x_GET() == -446892107);
            assert(pack.param1_GET() == 9.780612E37F);
            assert(pack.autocontinue_GET() == (char)37);
            assert(pack.y_GET() == 381080539);
            assert(pack.target_system_GET() == (char)13);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_LAND_START);
            assert(pack.current_GET() == (char)177);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.param4_SET(5.47226E37F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p75.y_SET(381080539) ;
        p75.target_system_SET((char)13) ;
        p75.target_component_SET((char)11) ;
        p75.x_SET(-446892107) ;
        p75.current_SET((char)177) ;
        p75.z_SET(1.9505839E38F) ;
        p75.param1_SET(9.780612E37F) ;
        p75.command_SET(MAV_CMD.MAV_CMD_DO_LAND_START) ;
        p75.param3_SET(3.344978E38F) ;
        p75.param2_SET(2.333693E38F) ;
        p75.autocontinue_SET((char)37) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param7_GET() == 5.2040444E37F);
            assert(pack.param2_GET() == -4.810654E37F);
            assert(pack.param6_GET() == -8.250849E37F);
            assert(pack.confirmation_GET() == (char)75);
            assert(pack.target_system_GET() == (char)125);
            assert(pack.param5_GET() == 9.282425E37F);
            assert(pack.param1_GET() == -2.1910957E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION);
            assert(pack.param4_GET() == 3.3958715E38F);
            assert(pack.target_component_GET() == (char)193);
            assert(pack.param3_GET() == 2.5735216E38F);
        });
        GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param4_SET(3.3958715E38F) ;
        p76.target_system_SET((char)125) ;
        p76.param5_SET(9.282425E37F) ;
        p76.param7_SET(5.2040444E37F) ;
        p76.target_component_SET((char)193) ;
        p76.param1_SET(-2.1910957E38F) ;
        p76.param3_SET(2.5735216E38F) ;
        p76.confirmation_SET((char)75) ;
        p76.command_SET(MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION) ;
        p76.param6_SET(-8.250849E37F) ;
        p76.param2_SET(-4.810654E37F) ;
        CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_TRY(ph) == (char)224);
            assert(pack.target_system_TRY(ph) == (char)191);
            assert(pack.progress_TRY(ph) == (char)57);
            assert(pack.result_param2_TRY(ph) == 1315429359);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_FAILED);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_LAND);
        });
        GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.target_system_SET((char)191, PH) ;
        p77.target_component_SET((char)224, PH) ;
        p77.result_param2_SET(1315429359, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_FAILED) ;
        p77.progress_SET((char)57, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_NAV_LAND) ;
        CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3539226662L);
            assert(pack.manual_override_switch_GET() == (char)59);
            assert(pack.mode_switch_GET() == (char)82);
            assert(pack.roll_GET() == -1.4096167E38F);
            assert(pack.thrust_GET() == -2.0018154E38F);
            assert(pack.pitch_GET() == 9.520819E37F);
            assert(pack.yaw_GET() == -2.9946014E38F);
        });
        GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.mode_switch_SET((char)82) ;
        p81.yaw_SET(-2.9946014E38F) ;
        p81.time_boot_ms_SET(3539226662L) ;
        p81.roll_SET(-1.4096167E38F) ;
        p81.thrust_SET(-2.0018154E38F) ;
        p81.pitch_SET(9.520819E37F) ;
        p81.manual_override_switch_SET((char)59) ;
        CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_yaw_rate_GET() == -3.8536892E36F);
            assert(pack.thrust_GET() == -2.934387E38F);
            assert(pack.type_mask_GET() == (char)232);
            assert(pack.body_pitch_rate_GET() == 2.0513392E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-9.622697E37F, -2.7264589E37F, 3.2658693E38F, 1.2723728E38F}));
            assert(pack.target_system_GET() == (char)223);
            assert(pack.body_roll_rate_GET() == -7.5522947E37F);
            assert(pack.target_component_GET() == (char)64);
            assert(pack.time_boot_ms_GET() == 2123459695L);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.target_component_SET((char)64) ;
        p82.body_pitch_rate_SET(2.0513392E38F) ;
        p82.body_yaw_rate_SET(-3.8536892E36F) ;
        p82.target_system_SET((char)223) ;
        p82.body_roll_rate_SET(-7.5522947E37F) ;
        p82.q_SET(new float[] {-9.622697E37F, -2.7264589E37F, 3.2658693E38F, 1.2723728E38F}, 0) ;
        p82.type_mask_SET((char)232) ;
        p82.thrust_SET(-2.934387E38F) ;
        p82.time_boot_ms_SET(2123459695L) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_roll_rate_GET() == -1.5668601E38F);
            assert(pack.body_pitch_rate_GET() == -1.4756992E38F);
            assert(pack.thrust_GET() == -7.5798057E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.2556198E38F, 1.8619138E38F, -3.198359E38F, -1.0783788E37F}));
            assert(pack.type_mask_GET() == (char)106);
            assert(pack.time_boot_ms_GET() == 1959460641L);
            assert(pack.body_yaw_rate_GET() == 2.858595E38F);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.q_SET(new float[] {-3.2556198E38F, 1.8619138E38F, -3.198359E38F, -1.0783788E37F}, 0) ;
        p83.body_pitch_rate_SET(-1.4756992E38F) ;
        p83.body_roll_rate_SET(-1.5668601E38F) ;
        p83.time_boot_ms_SET(1959460641L) ;
        p83.thrust_SET(-7.5798057E37F) ;
        p83.body_yaw_rate_SET(2.858595E38F) ;
        p83.type_mask_SET((char)106) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 1.1807699E38F);
            assert(pack.type_mask_GET() == (char)48895);
            assert(pack.vz_GET() == 6.66162E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.time_boot_ms_GET() == 1631979687L);
            assert(pack.afz_GET() == -3.2795281E38F);
            assert(pack.afx_GET() == 3.4324682E37F);
            assert(pack.yaw_rate_GET() == 2.621014E38F);
            assert(pack.z_GET() == 3.0755254E38F);
            assert(pack.yaw_GET() == -7.588065E37F);
            assert(pack.vy_GET() == 8.560008E37F);
            assert(pack.vx_GET() == 3.3610948E38F);
            assert(pack.target_component_GET() == (char)70);
            assert(pack.afy_GET() == 9.764588E37F);
            assert(pack.target_system_GET() == (char)47);
            assert(pack.x_GET() == 2.160217E38F);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p84.vz_SET(6.66162E37F) ;
        p84.z_SET(3.0755254E38F) ;
        p84.yaw_rate_SET(2.621014E38F) ;
        p84.afy_SET(9.764588E37F) ;
        p84.vx_SET(3.3610948E38F) ;
        p84.time_boot_ms_SET(1631979687L) ;
        p84.vy_SET(8.560008E37F) ;
        p84.type_mask_SET((char)48895) ;
        p84.y_SET(1.1807699E38F) ;
        p84.afz_SET(-3.2795281E38F) ;
        p84.yaw_SET(-7.588065E37F) ;
        p84.x_SET(2.160217E38F) ;
        p84.afx_SET(3.4324682E37F) ;
        p84.target_component_SET((char)70) ;
        p84.target_system_SET((char)47) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.afy_GET() == -2.9561324E38F);
            assert(pack.lon_int_GET() == 760229451);
            assert(pack.target_component_GET() == (char)20);
            assert(pack.vz_GET() == 2.7536052E38F);
            assert(pack.target_system_GET() == (char)107);
            assert(pack.yaw_rate_GET() == 3.1381999E38F);
            assert(pack.time_boot_ms_GET() == 3767419389L);
            assert(pack.vy_GET() == 2.0023957E38F);
            assert(pack.vx_GET() == 1.6685228E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.afx_GET() == -2.7663874E38F);
            assert(pack.lat_int_GET() == 1524835279);
            assert(pack.yaw_GET() == -2.7814561E37F);
            assert(pack.alt_GET() == 3.031454E38F);
            assert(pack.afz_GET() == -3.7050556E36F);
            assert(pack.type_mask_GET() == (char)44046);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.vy_SET(2.0023957E38F) ;
        p86.afx_SET(-2.7663874E38F) ;
        p86.target_system_SET((char)107) ;
        p86.target_component_SET((char)20) ;
        p86.lat_int_SET(1524835279) ;
        p86.vx_SET(1.6685228E38F) ;
        p86.afy_SET(-2.9561324E38F) ;
        p86.alt_SET(3.031454E38F) ;
        p86.lon_int_SET(760229451) ;
        p86.afz_SET(-3.7050556E36F) ;
        p86.vz_SET(2.7536052E38F) ;
        p86.yaw_rate_SET(3.1381999E38F) ;
        p86.type_mask_SET((char)44046) ;
        p86.yaw_SET(-2.7814561E37F) ;
        p86.time_boot_ms_SET(3767419389L) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == -2.1545862E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.yaw_rate_GET() == 1.0132622E38F);
            assert(pack.time_boot_ms_GET() == 2830928718L);
            assert(pack.afz_GET() == 2.4067223E38F);
            assert(pack.alt_GET() == -1.5327429E38F);
            assert(pack.vz_GET() == 8.765641E37F);
            assert(pack.yaw_GET() == -3.3133914E38F);
            assert(pack.vy_GET() == 1.2863498E38F);
            assert(pack.lat_int_GET() == -506326103);
            assert(pack.lon_int_GET() == -703780924);
            assert(pack.afy_GET() == 1.6677727E38F);
            assert(pack.type_mask_GET() == (char)2525);
            assert(pack.afx_GET() == -1.7754315E38F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.vz_SET(8.765641E37F) ;
        p87.vy_SET(1.2863498E38F) ;
        p87.vx_SET(-2.1545862E38F) ;
        p87.yaw_SET(-3.3133914E38F) ;
        p87.afx_SET(-1.7754315E38F) ;
        p87.alt_SET(-1.5327429E38F) ;
        p87.lat_int_SET(-506326103) ;
        p87.time_boot_ms_SET(2830928718L) ;
        p87.type_mask_SET((char)2525) ;
        p87.lon_int_SET(-703780924) ;
        p87.afz_SET(2.4067223E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p87.yaw_rate_SET(1.0132622E38F) ;
        p87.afy_SET(1.6677727E38F) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -2.3273497E38F);
            assert(pack.yaw_GET() == 4.508316E37F);
            assert(pack.y_GET() == -1.1904033E37F);
            assert(pack.x_GET() == 1.323151E38F);
            assert(pack.time_boot_ms_GET() == 1099846848L);
            assert(pack.roll_GET() == -2.5737084E35F);
            assert(pack.z_GET() == -1.764623E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.roll_SET(-2.5737084E35F) ;
        p89.yaw_SET(4.508316E37F) ;
        p89.pitch_SET(-2.3273497E38F) ;
        p89.y_SET(-1.1904033E37F) ;
        p89.z_SET(-1.764623E38F) ;
        p89.time_boot_ms_SET(1099846848L) ;
        p89.x_SET(1.323151E38F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == -2.3780075E38F);
            assert(pack.zacc_GET() == (short)12476);
            assert(pack.alt_GET() == 1071157146);
            assert(pack.pitch_GET() == 2.3673598E38F);
            assert(pack.yacc_GET() == (short) -11104);
            assert(pack.vy_GET() == (short)5688);
            assert(pack.roll_GET() == -2.4150504E38F);
            assert(pack.lat_GET() == -617826826);
            assert(pack.time_usec_GET() == 4913964973626130412L);
            assert(pack.vx_GET() == (short) -9059);
            assert(pack.pitchspeed_GET() == 2.5806141E37F);
            assert(pack.yaw_GET() == -2.3892577E38F);
            assert(pack.lon_GET() == 411569005);
            assert(pack.vz_GET() == (short)29735);
            assert(pack.rollspeed_GET() == 3.3622767E38F);
            assert(pack.xacc_GET() == (short)30918);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.vz_SET((short)29735) ;
        p90.rollspeed_SET(3.3622767E38F) ;
        p90.vy_SET((short)5688) ;
        p90.time_usec_SET(4913964973626130412L) ;
        p90.alt_SET(1071157146) ;
        p90.pitchspeed_SET(2.5806141E37F) ;
        p90.yacc_SET((short) -11104) ;
        p90.vx_SET((short) -9059) ;
        p90.yawspeed_SET(-2.3780075E38F) ;
        p90.lon_SET(411569005) ;
        p90.yaw_SET(-2.3892577E38F) ;
        p90.lat_SET(-617826826) ;
        p90.pitch_SET(2.3673598E38F) ;
        p90.zacc_SET((short)12476) ;
        p90.roll_SET(-2.4150504E38F) ;
        p90.xacc_SET((short)30918) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.throttle_GET() == -1.5417928E38F);
            assert(pack.aux4_GET() == 1.3600551E38F);
            assert(pack.pitch_elevator_GET() == -2.6790538E38F);
            assert(pack.nav_mode_GET() == (char)74);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
            assert(pack.roll_ailerons_GET() == 5.228195E37F);
            assert(pack.aux3_GET() == 7.911772E37F);
            assert(pack.time_usec_GET() == 7002419411511388219L);
            assert(pack.aux1_GET() == -5.653355E37F);
            assert(pack.yaw_rudder_GET() == -2.0923322E38F);
            assert(pack.aux2_GET() == -2.6876373E38F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.aux4_SET(1.3600551E38F) ;
        p91.yaw_rudder_SET(-2.0923322E38F) ;
        p91.time_usec_SET(7002419411511388219L) ;
        p91.roll_ailerons_SET(5.228195E37F) ;
        p91.aux2_SET(-2.6876373E38F) ;
        p91.aux3_SET(7.911772E37F) ;
        p91.throttle_SET(-1.5417928E38F) ;
        p91.nav_mode_SET((char)74) ;
        p91.pitch_elevator_SET(-2.6790538E38F) ;
        p91.aux1_SET(-5.653355E37F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan1_raw_GET() == (char)42547);
            assert(pack.chan10_raw_GET() == (char)38082);
            assert(pack.chan11_raw_GET() == (char)24980);
            assert(pack.chan9_raw_GET() == (char)41041);
            assert(pack.chan2_raw_GET() == (char)57892);
            assert(pack.chan4_raw_GET() == (char)5760);
            assert(pack.chan12_raw_GET() == (char)41807);
            assert(pack.chan8_raw_GET() == (char)18291);
            assert(pack.chan7_raw_GET() == (char)61235);
            assert(pack.chan3_raw_GET() == (char)19908);
            assert(pack.rssi_GET() == (char)241);
            assert(pack.chan6_raw_GET() == (char)22127);
            assert(pack.time_usec_GET() == 7831500611993511907L);
            assert(pack.chan5_raw_GET() == (char)9156);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan9_raw_SET((char)41041) ;
        p92.chan2_raw_SET((char)57892) ;
        p92.time_usec_SET(7831500611993511907L) ;
        p92.chan5_raw_SET((char)9156) ;
        p92.chan12_raw_SET((char)41807) ;
        p92.chan6_raw_SET((char)22127) ;
        p92.chan11_raw_SET((char)24980) ;
        p92.chan7_raw_SET((char)61235) ;
        p92.rssi_SET((char)241) ;
        p92.chan4_raw_SET((char)5760) ;
        p92.chan10_raw_SET((char)38082) ;
        p92.chan1_raw_SET((char)42547) ;
        p92.chan3_raw_SET((char)19908) ;
        p92.chan8_raw_SET((char)18291) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 8431043187518589696L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_ARMED);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-9.20819E37F, 1.8230854E38F, -1.2192491E38F, -2.5867751E38F, 1.5984439E38F, -1.6836179E38F, 2.1289E38F, 7.4679863E37F, 3.0762932E37F, 1.3690653E37F, 2.093774E38F, -2.1284462E38F, 3.552697E37F, 2.9771267E38F, 2.2602269E37F, -1.3237284E38F}));
            assert(pack.flags_GET() == 6523636357086947628L);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(8431043187518589696L) ;
        p93.controls_SET(new float[] {-9.20819E37F, 1.8230854E38F, -1.2192491E38F, -2.5867751E38F, 1.5984439E38F, -1.6836179E38F, 2.1289E38F, 7.4679863E37F, 3.0762932E37F, 1.3690653E37F, 2.093774E38F, -2.1284462E38F, 3.552697E37F, 2.9771267E38F, 2.2602269E37F, -1.3237284E38F}, 0) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED) ;
        p93.flags_SET(6523636357086947628L) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_x_GET() == (short) -2996);
            assert(pack.quality_GET() == (char)0);
            assert(pack.flow_comp_m_x_GET() == 2.9069837E38F);
            assert(pack.flow_rate_y_TRY(ph) == 1.541094E38F);
            assert(pack.ground_distance_GET() == -4.7844034E37F);
            assert(pack.flow_comp_m_y_GET() == -1.2064784E38F);
            assert(pack.sensor_id_GET() == (char)82);
            assert(pack.flow_rate_x_TRY(ph) == -1.5624698E38F);
            assert(pack.flow_y_GET() == (short)30572);
            assert(pack.time_usec_GET() == 3893076697430450535L);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.quality_SET((char)0) ;
        p100.flow_comp_m_x_SET(2.9069837E38F) ;
        p100.flow_x_SET((short) -2996) ;
        p100.time_usec_SET(3893076697430450535L) ;
        p100.flow_y_SET((short)30572) ;
        p100.flow_comp_m_y_SET(-1.2064784E38F) ;
        p100.ground_distance_SET(-4.7844034E37F) ;
        p100.flow_rate_x_SET(-1.5624698E38F, PH) ;
        p100.sensor_id_SET((char)82) ;
        p100.flow_rate_y_SET(1.541094E38F, PH) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -1.0539442E38F);
            assert(pack.z_GET() == 1.1791302E38F);
            assert(pack.roll_GET() == -2.1409254E38F);
            assert(pack.usec_GET() == 5663429247602450148L);
            assert(pack.pitch_GET() == 3.0389047E38F);
            assert(pack.x_GET() == -2.181493E38F);
            assert(pack.y_GET() == 3.3684707E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.x_SET(-2.181493E38F) ;
        p101.pitch_SET(3.0389047E38F) ;
        p101.yaw_SET(-1.0539442E38F) ;
        p101.y_SET(3.3684707E38F) ;
        p101.roll_SET(-2.1409254E38F) ;
        p101.usec_SET(5663429247602450148L) ;
        p101.z_SET(1.1791302E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 1.927018E38F);
            assert(pack.usec_GET() == 1241238755957322254L);
            assert(pack.y_GET() == -1.2294805E37F);
            assert(pack.pitch_GET() == -2.915581E38F);
            assert(pack.z_GET() == 3.2165687E38F);
            assert(pack.x_GET() == 3.3690802E38F);
            assert(pack.roll_GET() == -5.7791907E37F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.roll_SET(-5.7791907E37F) ;
        p102.usec_SET(1241238755957322254L) ;
        p102.x_SET(3.3690802E38F) ;
        p102.yaw_SET(1.927018E38F) ;
        p102.y_SET(-1.2294805E37F) ;
        p102.z_SET(3.2165687E38F) ;
        p102.pitch_SET(-2.915581E38F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -2.921192E38F);
            assert(pack.z_GET() == -2.4325126E38F);
            assert(pack.x_GET() == 2.523833E38F);
            assert(pack.usec_GET() == 6973938736773951578L);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(6973938736773951578L) ;
        p103.y_SET(-2.921192E38F) ;
        p103.z_SET(-2.4325126E38F) ;
        p103.x_SET(2.523833E38F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -6.503465E37F);
            assert(pack.x_GET() == -3.0387642E38F);
            assert(pack.roll_GET() == -1.7542672E38F);
            assert(pack.y_GET() == -1.2355467E37F);
            assert(pack.usec_GET() == 5598991153192043974L);
            assert(pack.pitch_GET() == 2.820869E38F);
            assert(pack.yaw_GET() == 4.6973336E37F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.yaw_SET(4.6973336E37F) ;
        p104.usec_SET(5598991153192043974L) ;
        p104.x_SET(-3.0387642E38F) ;
        p104.pitch_SET(2.820869E38F) ;
        p104.y_SET(-1.2355467E37F) ;
        p104.z_SET(-6.503465E37F) ;
        p104.roll_SET(-1.7542672E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == 4.4096017E37F);
            assert(pack.ymag_GET() == 3.2461611E38F);
            assert(pack.zacc_GET() == 2.008988E36F);
            assert(pack.zgyro_GET() == 1.1999822E38F);
            assert(pack.zmag_GET() == -1.5672982E38F);
            assert(pack.abs_pressure_GET() == -2.3066746E37F);
            assert(pack.ygyro_GET() == -1.3157834E38F);
            assert(pack.temperature_GET() == -2.7982123E38F);
            assert(pack.diff_pressure_GET() == 5.3286996E37F);
            assert(pack.fields_updated_GET() == (char)6202);
            assert(pack.xgyro_GET() == -2.1481844E38F);
            assert(pack.time_usec_GET() == 8507069732431216627L);
            assert(pack.yacc_GET() == 1.922499E38F);
            assert(pack.pressure_alt_GET() == 2.7814913E38F);
            assert(pack.xacc_GET() == -1.4906567E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.xmag_SET(4.4096017E37F) ;
        p105.ygyro_SET(-1.3157834E38F) ;
        p105.zacc_SET(2.008988E36F) ;
        p105.zgyro_SET(1.1999822E38F) ;
        p105.xacc_SET(-1.4906567E38F) ;
        p105.xgyro_SET(-2.1481844E38F) ;
        p105.ymag_SET(3.2461611E38F) ;
        p105.abs_pressure_SET(-2.3066746E37F) ;
        p105.yacc_SET(1.922499E38F) ;
        p105.temperature_SET(-2.7982123E38F) ;
        p105.fields_updated_SET((char)6202) ;
        p105.zmag_SET(-1.5672982E38F) ;
        p105.time_usec_SET(8507069732431216627L) ;
        p105.diff_pressure_SET(5.3286996E37F) ;
        p105.pressure_alt_SET(2.7814913E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.time_delta_distance_us_GET() == 1038536783L);
            assert(pack.integrated_ygyro_GET() == -8.89513E37F);
            assert(pack.integration_time_us_GET() == 4171213379L);
            assert(pack.integrated_x_GET() == 9.595233E37F);
            assert(pack.distance_GET() == 1.640204E38F);
            assert(pack.sensor_id_GET() == (char)14);
            assert(pack.time_usec_GET() == 6955417974131067805L);
            assert(pack.temperature_GET() == (short) -14005);
            assert(pack.integrated_zgyro_GET() == 1.875471E38F);
            assert(pack.integrated_xgyro_GET() == -2.6549198E38F);
            assert(pack.quality_GET() == (char)87);
            assert(pack.integrated_y_GET() == 3.0402736E38F);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.quality_SET((char)87) ;
        p106.time_usec_SET(6955417974131067805L) ;
        p106.integrated_zgyro_SET(1.875471E38F) ;
        p106.integration_time_us_SET(4171213379L) ;
        p106.integrated_x_SET(9.595233E37F) ;
        p106.distance_SET(1.640204E38F) ;
        p106.integrated_xgyro_SET(-2.6549198E38F) ;
        p106.time_delta_distance_us_SET(1038536783L) ;
        p106.temperature_SET((short) -14005) ;
        p106.integrated_y_SET(3.0402736E38F) ;
        p106.sensor_id_SET((char)14) ;
        p106.integrated_ygyro_SET(-8.89513E37F) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.diff_pressure_GET() == -9.270301E37F);
            assert(pack.zacc_GET() == 4.776807E37F);
            assert(pack.zmag_GET() == -3.1815955E38F);
            assert(pack.xmag_GET() == 2.3836188E38F);
            assert(pack.yacc_GET() == -3.0370213E38F);
            assert(pack.zgyro_GET() == 1.0507308E38F);
            assert(pack.xgyro_GET() == -3.6358747E37F);
            assert(pack.xacc_GET() == -3.3662413E38F);
            assert(pack.time_usec_GET() == 3815494030307818668L);
            assert(pack.fields_updated_GET() == 3710215639L);
            assert(pack.temperature_GET() == 1.8140035E38F);
            assert(pack.pressure_alt_GET() == -2.309765E38F);
            assert(pack.ymag_GET() == 6.6351865E37F);
            assert(pack.ygyro_GET() == -3.3059912E38F);
            assert(pack.abs_pressure_GET() == -5.7402044E37F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.zacc_SET(4.776807E37F) ;
        p107.time_usec_SET(3815494030307818668L) ;
        p107.yacc_SET(-3.0370213E38F) ;
        p107.xacc_SET(-3.3662413E38F) ;
        p107.temperature_SET(1.8140035E38F) ;
        p107.zgyro_SET(1.0507308E38F) ;
        p107.ymag_SET(6.6351865E37F) ;
        p107.zmag_SET(-3.1815955E38F) ;
        p107.diff_pressure_SET(-9.270301E37F) ;
        p107.xgyro_SET(-3.6358747E37F) ;
        p107.abs_pressure_SET(-5.7402044E37F) ;
        p107.pressure_alt_SET(-2.309765E38F) ;
        p107.ygyro_SET(-3.3059912E38F) ;
        p107.fields_updated_SET(3710215639L) ;
        p107.xmag_SET(2.3836188E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.vn_GET() == -1.6178309E38F);
            assert(pack.zacc_GET() == -2.1947314E38F);
            assert(pack.q4_GET() == -1.8455155E38F);
            assert(pack.lon_GET() == -2.4679424E36F);
            assert(pack.yacc_GET() == -1.0330236E38F);
            assert(pack.vd_GET() == -9.691694E37F);
            assert(pack.ygyro_GET() == 1.1930245E38F);
            assert(pack.zgyro_GET() == -1.3074436E38F);
            assert(pack.q1_GET() == -3.2063687E38F);
            assert(pack.ve_GET() == 1.9577827E38F);
            assert(pack.std_dev_horz_GET() == -2.0387322E38F);
            assert(pack.q3_GET() == -4.6871386E37F);
            assert(pack.lat_GET() == -2.191306E37F);
            assert(pack.yaw_GET() == -3.215635E34F);
            assert(pack.pitch_GET() == -2.8138452E38F);
            assert(pack.alt_GET() == -1.4145703E38F);
            assert(pack.xgyro_GET() == 1.7163314E38F);
            assert(pack.std_dev_vert_GET() == -2.2840376E38F);
            assert(pack.roll_GET() == -5.345361E37F);
            assert(pack.xacc_GET() == -4.213271E37F);
            assert(pack.q2_GET() == -2.2219321E37F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.lon_SET(-2.4679424E36F) ;
        p108.yaw_SET(-3.215635E34F) ;
        p108.zgyro_SET(-1.3074436E38F) ;
        p108.q1_SET(-3.2063687E38F) ;
        p108.q2_SET(-2.2219321E37F) ;
        p108.vd_SET(-9.691694E37F) ;
        p108.xgyro_SET(1.7163314E38F) ;
        p108.std_dev_horz_SET(-2.0387322E38F) ;
        p108.vn_SET(-1.6178309E38F) ;
        p108.zacc_SET(-2.1947314E38F) ;
        p108.pitch_SET(-2.8138452E38F) ;
        p108.q3_SET(-4.6871386E37F) ;
        p108.std_dev_vert_SET(-2.2840376E38F) ;
        p108.xacc_SET(-4.213271E37F) ;
        p108.yacc_SET(-1.0330236E38F) ;
        p108.roll_SET(-5.345361E37F) ;
        p108.q4_SET(-1.8455155E38F) ;
        p108.alt_SET(-1.4145703E38F) ;
        p108.ve_SET(1.9577827E38F) ;
        p108.lat_SET(-2.191306E37F) ;
        p108.ygyro_SET(1.1930245E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.txbuf_GET() == (char)81);
            assert(pack.remrssi_GET() == (char)151);
            assert(pack.fixed__GET() == (char)59220);
            assert(pack.noise_GET() == (char)186);
            assert(pack.remnoise_GET() == (char)220);
            assert(pack.rxerrors_GET() == (char)40691);
            assert(pack.rssi_GET() == (char)172);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.txbuf_SET((char)81) ;
        p109.remrssi_SET((char)151) ;
        p109.rssi_SET((char)172) ;
        p109.noise_SET((char)186) ;
        p109.fixed__SET((char)59220) ;
        p109.rxerrors_SET((char)40691) ;
        p109.remnoise_SET((char)220) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)77);
            assert(pack.target_component_GET() == (char)127);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)16, (char)153, (char)109, (char)78, (char)243, (char)238, (char)181, (char)199, (char)178, (char)13, (char)19, (char)132, (char)254, (char)69, (char)208, (char)90, (char)182, (char)173, (char)195, (char)81, (char)245, (char)153, (char)79, (char)133, (char)174, (char)68, (char)237, (char)19, (char)96, (char)31, (char)157, (char)40, (char)234, (char)233, (char)156, (char)28, (char)2, (char)170, (char)172, (char)165, (char)179, (char)200, (char)127, (char)160, (char)244, (char)68, (char)110, (char)123, (char)22, (char)92, (char)34, (char)250, (char)101, (char)81, (char)119, (char)192, (char)11, (char)103, (char)161, (char)20, (char)255, (char)224, (char)200, (char)72, (char)103, (char)160, (char)188, (char)41, (char)186, (char)141, (char)54, (char)61, (char)188, (char)26, (char)90, (char)124, (char)190, (char)104, (char)130, (char)90, (char)245, (char)208, (char)6, (char)64, (char)219, (char)38, (char)255, (char)143, (char)187, (char)65, (char)244, (char)20, (char)154, (char)204, (char)14, (char)63, (char)204, (char)123, (char)238, (char)17, (char)117, (char)149, (char)61, (char)98, (char)119, (char)41, (char)242, (char)193, (char)41, (char)151, (char)124, (char)21, (char)140, (char)139, (char)234, (char)18, (char)84, (char)111, (char)149, (char)69, (char)112, (char)243, (char)180, (char)254, (char)91, (char)139, (char)120, (char)171, (char)101, (char)11, (char)170, (char)206, (char)226, (char)8, (char)33, (char)60, (char)37, (char)134, (char)196, (char)176, (char)79, (char)84, (char)245, (char)71, (char)3, (char)198, (char)218, (char)76, (char)200, (char)75, (char)159, (char)200, (char)35, (char)85, (char)94, (char)214, (char)177, (char)19, (char)172, (char)28, (char)196, (char)30, (char)134, (char)0, (char)240, (char)178, (char)238, (char)242, (char)236, (char)166, (char)104, (char)113, (char)146, (char)53, (char)102, (char)89, (char)115, (char)86, (char)214, (char)85, (char)253, (char)246, (char)117, (char)161, (char)196, (char)142, (char)62, (char)188, (char)108, (char)112, (char)119, (char)230, (char)101, (char)29, (char)4, (char)89, (char)141, (char)24, (char)232, (char)70, (char)18, (char)107, (char)245, (char)182, (char)63, (char)54, (char)34, (char)245, (char)219, (char)82, (char)220, (char)125, (char)254, (char)107, (char)77, (char)119, (char)146, (char)222, (char)25, (char)65, (char)1, (char)191, (char)93, (char)1, (char)78, (char)160, (char)137, (char)172, (char)226, (char)11, (char)141, (char)6, (char)159, (char)189, (char)92, (char)84, (char)111, (char)77, (char)55, (char)72, (char)163, (char)75, (char)237, (char)107, (char)240, (char)141, (char)49, (char)73, (char)240, (char)10, (char)170}));
            assert(pack.target_system_GET() == (char)254);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_component_SET((char)127) ;
        p110.payload_SET(new char[] {(char)16, (char)153, (char)109, (char)78, (char)243, (char)238, (char)181, (char)199, (char)178, (char)13, (char)19, (char)132, (char)254, (char)69, (char)208, (char)90, (char)182, (char)173, (char)195, (char)81, (char)245, (char)153, (char)79, (char)133, (char)174, (char)68, (char)237, (char)19, (char)96, (char)31, (char)157, (char)40, (char)234, (char)233, (char)156, (char)28, (char)2, (char)170, (char)172, (char)165, (char)179, (char)200, (char)127, (char)160, (char)244, (char)68, (char)110, (char)123, (char)22, (char)92, (char)34, (char)250, (char)101, (char)81, (char)119, (char)192, (char)11, (char)103, (char)161, (char)20, (char)255, (char)224, (char)200, (char)72, (char)103, (char)160, (char)188, (char)41, (char)186, (char)141, (char)54, (char)61, (char)188, (char)26, (char)90, (char)124, (char)190, (char)104, (char)130, (char)90, (char)245, (char)208, (char)6, (char)64, (char)219, (char)38, (char)255, (char)143, (char)187, (char)65, (char)244, (char)20, (char)154, (char)204, (char)14, (char)63, (char)204, (char)123, (char)238, (char)17, (char)117, (char)149, (char)61, (char)98, (char)119, (char)41, (char)242, (char)193, (char)41, (char)151, (char)124, (char)21, (char)140, (char)139, (char)234, (char)18, (char)84, (char)111, (char)149, (char)69, (char)112, (char)243, (char)180, (char)254, (char)91, (char)139, (char)120, (char)171, (char)101, (char)11, (char)170, (char)206, (char)226, (char)8, (char)33, (char)60, (char)37, (char)134, (char)196, (char)176, (char)79, (char)84, (char)245, (char)71, (char)3, (char)198, (char)218, (char)76, (char)200, (char)75, (char)159, (char)200, (char)35, (char)85, (char)94, (char)214, (char)177, (char)19, (char)172, (char)28, (char)196, (char)30, (char)134, (char)0, (char)240, (char)178, (char)238, (char)242, (char)236, (char)166, (char)104, (char)113, (char)146, (char)53, (char)102, (char)89, (char)115, (char)86, (char)214, (char)85, (char)253, (char)246, (char)117, (char)161, (char)196, (char)142, (char)62, (char)188, (char)108, (char)112, (char)119, (char)230, (char)101, (char)29, (char)4, (char)89, (char)141, (char)24, (char)232, (char)70, (char)18, (char)107, (char)245, (char)182, (char)63, (char)54, (char)34, (char)245, (char)219, (char)82, (char)220, (char)125, (char)254, (char)107, (char)77, (char)119, (char)146, (char)222, (char)25, (char)65, (char)1, (char)191, (char)93, (char)1, (char)78, (char)160, (char)137, (char)172, (char)226, (char)11, (char)141, (char)6, (char)159, (char)189, (char)92, (char)84, (char)111, (char)77, (char)55, (char)72, (char)163, (char)75, (char)237, (char)107, (char)240, (char)141, (char)49, (char)73, (char)240, (char)10, (char)170}, 0) ;
        p110.target_network_SET((char)77) ;
        p110.target_system_SET((char)254) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == 4212538060650692966L);
            assert(pack.tc1_GET() == -7662045182532424838L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-7662045182532424838L) ;
        p111.ts1_SET(4212538060650692966L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 1653921212790641238L);
            assert(pack.seq_GET() == 4204867676L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(4204867676L) ;
        p112.time_usec_SET(1653921212790641238L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 62327566034763052L);
            assert(pack.lat_GET() == 1099359028);
            assert(pack.fix_type_GET() == (char)149);
            assert(pack.ve_GET() == (short) -11908);
            assert(pack.cog_GET() == (char)16983);
            assert(pack.alt_GET() == -388106259);
            assert(pack.vel_GET() == (char)33080);
            assert(pack.lon_GET() == -1624985614);
            assert(pack.eph_GET() == (char)8673);
            assert(pack.satellites_visible_GET() == (char)199);
            assert(pack.epv_GET() == (char)7351);
            assert(pack.vn_GET() == (short) -15156);
            assert(pack.vd_GET() == (short) -19948);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.lat_SET(1099359028) ;
        p113.ve_SET((short) -11908) ;
        p113.vel_SET((char)33080) ;
        p113.fix_type_SET((char)149) ;
        p113.time_usec_SET(62327566034763052L) ;
        p113.lon_SET(-1624985614) ;
        p113.cog_SET((char)16983) ;
        p113.satellites_visible_SET((char)199) ;
        p113.vd_SET((short) -19948) ;
        p113.eph_SET((char)8673) ;
        p113.epv_SET((char)7351) ;
        p113.vn_SET((short) -15156) ;
        p113.alt_SET(-388106259) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_ygyro_GET() == -8.0607224E37F);
            assert(pack.integration_time_us_GET() == 1837327761L);
            assert(pack.sensor_id_GET() == (char)68);
            assert(pack.quality_GET() == (char)249);
            assert(pack.temperature_GET() == (short)11440);
            assert(pack.integrated_xgyro_GET() == -3.1803133E38F);
            assert(pack.integrated_x_GET() == -2.2248966E38F);
            assert(pack.integrated_y_GET() == -1.2295127E37F);
            assert(pack.time_delta_distance_us_GET() == 3047306726L);
            assert(pack.time_usec_GET() == 7305992438723447058L);
            assert(pack.distance_GET() == -2.8439356E38F);
            assert(pack.integrated_zgyro_GET() == -3.0454356E38F);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_ygyro_SET(-8.0607224E37F) ;
        p114.integration_time_us_SET(1837327761L) ;
        p114.integrated_y_SET(-1.2295127E37F) ;
        p114.time_delta_distance_us_SET(3047306726L) ;
        p114.time_usec_SET(7305992438723447058L) ;
        p114.integrated_xgyro_SET(-3.1803133E38F) ;
        p114.temperature_SET((short)11440) ;
        p114.quality_SET((char)249) ;
        p114.integrated_x_SET(-2.2248966E38F) ;
        p114.sensor_id_SET((char)68) ;
        p114.integrated_zgyro_SET(-3.0454356E38F) ;
        p114.distance_SET(-2.8439356E38F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 8555477608717312169L);
            assert(pack.vz_GET() == (short) -17945);
            assert(pack.true_airspeed_GET() == (char)57135);
            assert(pack.alt_GET() == 1453559405);
            assert(pack.xacc_GET() == (short) -23792);
            assert(pack.yacc_GET() == (short)11460);
            assert(pack.vx_GET() == (short) -8758);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {-1.6412915E38F, 2.0718417E38F, 3.3427563E38F, 3.3534031E38F}));
            assert(pack.yawspeed_GET() == -3.103706E38F);
            assert(pack.ind_airspeed_GET() == (char)54009);
            assert(pack.zacc_GET() == (short) -13449);
            assert(pack.vy_GET() == (short) -23057);
            assert(pack.lat_GET() == -1400793014);
            assert(pack.rollspeed_GET() == 3.052953E38F);
            assert(pack.pitchspeed_GET() == -1.6610774E38F);
            assert(pack.lon_GET() == -441435712);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.true_airspeed_SET((char)57135) ;
        p115.vy_SET((short) -23057) ;
        p115.vz_SET((short) -17945) ;
        p115.yawspeed_SET(-3.103706E38F) ;
        p115.lat_SET(-1400793014) ;
        p115.xacc_SET((short) -23792) ;
        p115.ind_airspeed_SET((char)54009) ;
        p115.lon_SET(-441435712) ;
        p115.zacc_SET((short) -13449) ;
        p115.time_usec_SET(8555477608717312169L) ;
        p115.yacc_SET((short)11460) ;
        p115.attitude_quaternion_SET(new float[] {-1.6412915E38F, 2.0718417E38F, 3.3427563E38F, 3.3534031E38F}, 0) ;
        p115.alt_SET(1453559405) ;
        p115.vx_SET((short) -8758) ;
        p115.rollspeed_SET(3.052953E38F) ;
        p115.pitchspeed_SET(-1.6610774E38F) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == (short)1253);
            assert(pack.xacc_GET() == (short) -20798);
            assert(pack.xgyro_GET() == (short) -23598);
            assert(pack.zmag_GET() == (short)24869);
            assert(pack.zgyro_GET() == (short)25662);
            assert(pack.zacc_GET() == (short)18109);
            assert(pack.ymag_GET() == (short)11064);
            assert(pack.ygyro_GET() == (short)13575);
            assert(pack.yacc_GET() == (short) -6605);
            assert(pack.time_boot_ms_GET() == 1324738221L);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.xmag_SET((short)1253) ;
        p116.ymag_SET((short)11064) ;
        p116.xacc_SET((short) -20798) ;
        p116.zmag_SET((short)24869) ;
        p116.zgyro_SET((short)25662) ;
        p116.ygyro_SET((short)13575) ;
        p116.time_boot_ms_SET(1324738221L) ;
        p116.zacc_SET((short)18109) ;
        p116.yacc_SET((short) -6605) ;
        p116.xgyro_SET((short) -23598) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_GET() == (char)16480);
            assert(pack.end_GET() == (char)60334);
            assert(pack.target_system_GET() == (char)200);
            assert(pack.target_component_GET() == (char)28);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.end_SET((char)60334) ;
        p117.target_system_SET((char)200) ;
        p117.start_SET((char)16480) ;
        p117.target_component_SET((char)28) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.size_GET() == 530973247L);
            assert(pack.time_utc_GET() == 62515911L);
            assert(pack.last_log_num_GET() == (char)17991);
            assert(pack.num_logs_GET() == (char)37184);
            assert(pack.id_GET() == (char)39698);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.num_logs_SET((char)37184) ;
        p118.time_utc_SET(62515911L) ;
        p118.size_SET(530973247L) ;
        p118.id_SET((char)39698) ;
        p118.last_log_num_SET((char)17991) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)44793);
            assert(pack.count_GET() == 1654040829L);
            assert(pack.target_system_GET() == (char)250);
            assert(pack.target_component_GET() == (char)26);
            assert(pack.ofs_GET() == 3076270689L);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.ofs_SET(3076270689L) ;
        p119.target_system_SET((char)250) ;
        p119.id_SET((char)44793) ;
        p119.target_component_SET((char)26) ;
        p119.count_SET(1654040829L) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.ofs_GET() == 1889314879L);
            assert(pack.count_GET() == (char)123);
            assert(pack.id_GET() == (char)6054);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)179, (char)117, (char)155, (char)251, (char)248, (char)133, (char)105, (char)111, (char)32, (char)193, (char)146, (char)127, (char)6, (char)153, (char)168, (char)39, (char)26, (char)131, (char)115, (char)234, (char)229, (char)255, (char)228, (char)187, (char)212, (char)29, (char)172, (char)89, (char)147, (char)147, (char)179, (char)159, (char)240, (char)73, (char)11, (char)64, (char)156, (char)38, (char)166, (char)98, (char)72, (char)46, (char)79, (char)138, (char)159, (char)212, (char)94, (char)211, (char)56, (char)0, (char)132, (char)249, (char)190, (char)131, (char)220, (char)143, (char)135, (char)242, (char)3, (char)125, (char)205, (char)60, (char)207, (char)205, (char)121, (char)124, (char)114, (char)205, (char)98, (char)226, (char)232, (char)136, (char)39, (char)122, (char)226, (char)140, (char)51, (char)94, (char)131, (char)57, (char)55, (char)178, (char)222, (char)130, (char)112, (char)136, (char)135, (char)51, (char)3, (char)36}));
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)6054) ;
        p120.ofs_SET(1889314879L) ;
        p120.count_SET((char)123) ;
        p120.data__SET(new char[] {(char)179, (char)117, (char)155, (char)251, (char)248, (char)133, (char)105, (char)111, (char)32, (char)193, (char)146, (char)127, (char)6, (char)153, (char)168, (char)39, (char)26, (char)131, (char)115, (char)234, (char)229, (char)255, (char)228, (char)187, (char)212, (char)29, (char)172, (char)89, (char)147, (char)147, (char)179, (char)159, (char)240, (char)73, (char)11, (char)64, (char)156, (char)38, (char)166, (char)98, (char)72, (char)46, (char)79, (char)138, (char)159, (char)212, (char)94, (char)211, (char)56, (char)0, (char)132, (char)249, (char)190, (char)131, (char)220, (char)143, (char)135, (char)242, (char)3, (char)125, (char)205, (char)60, (char)207, (char)205, (char)121, (char)124, (char)114, (char)205, (char)98, (char)226, (char)232, (char)136, (char)39, (char)122, (char)226, (char)140, (char)51, (char)94, (char)131, (char)57, (char)55, (char)178, (char)222, (char)130, (char)112, (char)136, (char)135, (char)51, (char)3, (char)36}, 0) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)62);
            assert(pack.target_component_GET() == (char)138);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)62) ;
        p121.target_component_SET((char)138) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)164);
            assert(pack.target_component_GET() == (char)188);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)164) ;
        p122.target_component_SET((char)188) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)103);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)128, (char)204, (char)230, (char)50, (char)86, (char)53, (char)247, (char)223, (char)14, (char)32, (char)189, (char)82, (char)65, (char)33, (char)250, (char)111, (char)218, (char)227, (char)97, (char)97, (char)76, (char)179, (char)213, (char)222, (char)49, (char)108, (char)240, (char)104, (char)39, (char)180, (char)47, (char)180, (char)129, (char)12, (char)77, (char)47, (char)98, (char)18, (char)9, (char)198, (char)157, (char)40, (char)247, (char)251, (char)208, (char)3, (char)61, (char)184, (char)133, (char)155, (char)244, (char)13, (char)222, (char)35, (char)253, (char)193, (char)196, (char)107, (char)39, (char)13, (char)72, (char)210, (char)159, (char)49, (char)148, (char)47, (char)39, (char)154, (char)229, (char)1, (char)171, (char)92, (char)93, (char)177, (char)86, (char)99, (char)113, (char)181, (char)49, (char)244, (char)204, (char)103, (char)8, (char)129, (char)225, (char)87, (char)74, (char)121, (char)142, (char)214, (char)88, (char)240, (char)122, (char)156, (char)99, (char)173, (char)107, (char)182, (char)30, (char)224, (char)183, (char)154, (char)214, (char)177, (char)155, (char)33, (char)164, (char)200, (char)146, (char)183}));
            assert(pack.len_GET() == (char)220);
            assert(pack.target_component_GET() == (char)196);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_component_SET((char)196) ;
        p123.len_SET((char)220) ;
        p123.target_system_SET((char)103) ;
        p123.data__SET(new char[] {(char)128, (char)204, (char)230, (char)50, (char)86, (char)53, (char)247, (char)223, (char)14, (char)32, (char)189, (char)82, (char)65, (char)33, (char)250, (char)111, (char)218, (char)227, (char)97, (char)97, (char)76, (char)179, (char)213, (char)222, (char)49, (char)108, (char)240, (char)104, (char)39, (char)180, (char)47, (char)180, (char)129, (char)12, (char)77, (char)47, (char)98, (char)18, (char)9, (char)198, (char)157, (char)40, (char)247, (char)251, (char)208, (char)3, (char)61, (char)184, (char)133, (char)155, (char)244, (char)13, (char)222, (char)35, (char)253, (char)193, (char)196, (char)107, (char)39, (char)13, (char)72, (char)210, (char)159, (char)49, (char)148, (char)47, (char)39, (char)154, (char)229, (char)1, (char)171, (char)92, (char)93, (char)177, (char)86, (char)99, (char)113, (char)181, (char)49, (char)244, (char)204, (char)103, (char)8, (char)129, (char)225, (char)87, (char)74, (char)121, (char)142, (char)214, (char)88, (char)240, (char)122, (char)156, (char)99, (char)173, (char)107, (char)182, (char)30, (char)224, (char)183, (char)154, (char)214, (char)177, (char)155, (char)33, (char)164, (char)200, (char)146, (char)183}, 0) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.epv_GET() == (char)2325);
            assert(pack.cog_GET() == (char)29132);
            assert(pack.time_usec_GET() == 6053221621366819932L);
            assert(pack.vel_GET() == (char)64834);
            assert(pack.lat_GET() == -1963971563);
            assert(pack.lon_GET() == 669985314);
            assert(pack.satellites_visible_GET() == (char)46);
            assert(pack.dgps_age_GET() == 3649932914L);
            assert(pack.eph_GET() == (char)40550);
            assert(pack.alt_GET() == -125689795);
            assert(pack.dgps_numch_GET() == (char)138);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.dgps_numch_SET((char)138) ;
        p124.satellites_visible_SET((char)46) ;
        p124.vel_SET((char)64834) ;
        p124.time_usec_SET(6053221621366819932L) ;
        p124.lon_SET(669985314) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p124.lat_SET(-1963971563) ;
        p124.cog_SET((char)29132) ;
        p124.epv_SET((char)2325) ;
        p124.alt_SET(-125689795) ;
        p124.dgps_age_SET(3649932914L) ;
        p124.eph_SET((char)40550) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID));
            assert(pack.Vcc_GET() == (char)23153);
            assert(pack.Vservo_GET() == (char)59004);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)59004) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID)) ;
        p125.Vcc_SET((char)23153) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)160, (char)31, (char)111, (char)13, (char)178, (char)190, (char)202, (char)112, (char)152, (char)172, (char)74, (char)110, (char)219, (char)51, (char)64, (char)85, (char)36, (char)182, (char)141, (char)26, (char)8, (char)132, (char)13, (char)162, (char)119, (char)67, (char)38, (char)188, (char)159, (char)224, (char)55, (char)107, (char)41, (char)88, (char)85, (char)184, (char)6, (char)102, (char)61, (char)188, (char)6, (char)20, (char)80, (char)60, (char)230, (char)227, (char)46, (char)164, (char)170, (char)149, (char)28, (char)232, (char)232, (char)187, (char)117, (char)176, (char)249, (char)120, (char)113, (char)53, (char)75, (char)89, (char)5, (char)103, (char)181, (char)6, (char)30, (char)139, (char)182, (char)195}));
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
            assert(pack.baudrate_GET() == 782548229L);
            assert(pack.timeout_GET() == (char)2840);
            assert(pack.count_GET() == (char)173);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI));
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.count_SET((char)173) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2) ;
        p126.baudrate_SET(782548229L) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI)) ;
        p126.timeout_SET((char)2840) ;
        p126.data__SET(new char[] {(char)160, (char)31, (char)111, (char)13, (char)178, (char)190, (char)202, (char)112, (char)152, (char)172, (char)74, (char)110, (char)219, (char)51, (char)64, (char)85, (char)36, (char)182, (char)141, (char)26, (char)8, (char)132, (char)13, (char)162, (char)119, (char)67, (char)38, (char)188, (char)159, (char)224, (char)55, (char)107, (char)41, (char)88, (char)85, (char)184, (char)6, (char)102, (char)61, (char)188, (char)6, (char)20, (char)80, (char)60, (char)230, (char)227, (char)46, (char)164, (char)170, (char)149, (char)28, (char)232, (char)232, (char)187, (char)117, (char)176, (char)249, (char)120, (char)113, (char)53, (char)75, (char)89, (char)5, (char)103, (char)181, (char)6, (char)30, (char)139, (char)182, (char)195}, 0) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.tow_GET() == 3879626397L);
            assert(pack.baseline_coords_type_GET() == (char)43);
            assert(pack.rtk_receiver_id_GET() == (char)47);
            assert(pack.accuracy_GET() == 203590772L);
            assert(pack.baseline_b_mm_GET() == -1061596120);
            assert(pack.rtk_health_GET() == (char)91);
            assert(pack.rtk_rate_GET() == (char)73);
            assert(pack.nsats_GET() == (char)83);
            assert(pack.baseline_a_mm_GET() == 807343457);
            assert(pack.iar_num_hypotheses_GET() == -2045890156);
            assert(pack.baseline_c_mm_GET() == 1845124355);
            assert(pack.wn_GET() == (char)45712);
            assert(pack.time_last_baseline_ms_GET() == 2672222570L);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.iar_num_hypotheses_SET(-2045890156) ;
        p127.rtk_receiver_id_SET((char)47) ;
        p127.baseline_b_mm_SET(-1061596120) ;
        p127.time_last_baseline_ms_SET(2672222570L) ;
        p127.accuracy_SET(203590772L) ;
        p127.rtk_health_SET((char)91) ;
        p127.tow_SET(3879626397L) ;
        p127.baseline_a_mm_SET(807343457) ;
        p127.baseline_c_mm_SET(1845124355) ;
        p127.rtk_rate_SET((char)73) ;
        p127.wn_SET((char)45712) ;
        p127.nsats_SET((char)83) ;
        p127.baseline_coords_type_SET((char)43) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_a_mm_GET() == -1373597815);
            assert(pack.nsats_GET() == (char)48);
            assert(pack.baseline_c_mm_GET() == 2110409163);
            assert(pack.tow_GET() == 559742168L);
            assert(pack.baseline_coords_type_GET() == (char)55);
            assert(pack.accuracy_GET() == 471474880L);
            assert(pack.wn_GET() == (char)55491);
            assert(pack.rtk_rate_GET() == (char)178);
            assert(pack.time_last_baseline_ms_GET() == 3709529976L);
            assert(pack.iar_num_hypotheses_GET() == -1743653297);
            assert(pack.baseline_b_mm_GET() == 713892316);
            assert(pack.rtk_receiver_id_GET() == (char)176);
            assert(pack.rtk_health_GET() == (char)196);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_a_mm_SET(-1373597815) ;
        p128.rtk_receiver_id_SET((char)176) ;
        p128.nsats_SET((char)48) ;
        p128.baseline_b_mm_SET(713892316) ;
        p128.rtk_rate_SET((char)178) ;
        p128.baseline_c_mm_SET(2110409163) ;
        p128.accuracy_SET(471474880L) ;
        p128.time_last_baseline_ms_SET(3709529976L) ;
        p128.iar_num_hypotheses_SET(-1743653297) ;
        p128.wn_SET((char)55491) ;
        p128.rtk_health_SET((char)196) ;
        p128.tow_SET(559742168L) ;
        p128.baseline_coords_type_SET((char)55) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short)7113);
            assert(pack.xgyro_GET() == (short)31741);
            assert(pack.ymag_GET() == (short)27392);
            assert(pack.yacc_GET() == (short) -2791);
            assert(pack.ygyro_GET() == (short)3761);
            assert(pack.xacc_GET() == (short) -13826);
            assert(pack.zmag_GET() == (short)10138);
            assert(pack.xmag_GET() == (short)14956);
            assert(pack.zacc_GET() == (short) -6864);
            assert(pack.time_boot_ms_GET() == 1037917292L);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.xmag_SET((short)14956) ;
        p129.xacc_SET((short) -13826) ;
        p129.yacc_SET((short) -2791) ;
        p129.ygyro_SET((short)3761) ;
        p129.xgyro_SET((short)31741) ;
        p129.zgyro_SET((short)7113) ;
        p129.zmag_SET((short)10138) ;
        p129.time_boot_ms_SET(1037917292L) ;
        p129.zacc_SET((short) -6864) ;
        p129.ymag_SET((short)27392) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.size_GET() == 3281263861L);
            assert(pack.payload_GET() == (char)172);
            assert(pack.width_GET() == (char)37269);
            assert(pack.type_GET() == (char)29);
            assert(pack.height_GET() == (char)60325);
            assert(pack.jpg_quality_GET() == (char)244);
            assert(pack.packets_GET() == (char)29227);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.packets_SET((char)29227) ;
        p130.payload_SET((char)172) ;
        p130.jpg_quality_SET((char)244) ;
        p130.size_SET(3281263861L) ;
        p130.type_SET((char)29) ;
        p130.width_SET((char)37269) ;
        p130.height_SET((char)60325) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)12255);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)215, (char)94, (char)1, (char)173, (char)238, (char)163, (char)164, (char)129, (char)41, (char)51, (char)186, (char)119, (char)126, (char)115, (char)6, (char)24, (char)105, (char)175, (char)18, (char)247, (char)112, (char)248, (char)157, (char)19, (char)6, (char)174, (char)223, (char)6, (char)139, (char)54, (char)151, (char)225, (char)99, (char)136, (char)116, (char)70, (char)8, (char)104, (char)147, (char)55, (char)104, (char)214, (char)74, (char)144, (char)68, (char)254, (char)53, (char)161, (char)39, (char)93, (char)95, (char)26, (char)144, (char)132, (char)119, (char)142, (char)119, (char)149, (char)73, (char)233, (char)236, (char)253, (char)128, (char)241, (char)163, (char)58, (char)42, (char)182, (char)207, (char)20, (char)18, (char)60, (char)246, (char)147, (char)247, (char)231, (char)181, (char)98, (char)18, (char)20, (char)1, (char)48, (char)29, (char)3, (char)206, (char)211, (char)115, (char)204, (char)181, (char)219, (char)140, (char)171, (char)20, (char)135, (char)254, (char)101, (char)197, (char)168, (char)0, (char)31, (char)148, (char)82, (char)197, (char)215, (char)233, (char)22, (char)24, (char)125, (char)113, (char)238, (char)192, (char)126, (char)217, (char)51, (char)235, (char)81, (char)152, (char)94, (char)245, (char)86, (char)16, (char)188, (char)212, (char)200, (char)84, (char)232, (char)250, (char)238, (char)136, (char)188, (char)205, (char)203, (char)17, (char)63, (char)171, (char)122, (char)56, (char)244, (char)137, (char)157, (char)186, (char)54, (char)173, (char)11, (char)155, (char)207, (char)197, (char)162, (char)230, (char)54, (char)193, (char)216, (char)148, (char)249, (char)203, (char)182, (char)235, (char)165, (char)234, (char)30, (char)85, (char)212, (char)18, (char)204, (char)171, (char)95, (char)66, (char)216, (char)203, (char)163, (char)204, (char)106, (char)94, (char)51, (char)3, (char)226, (char)20, (char)25, (char)197, (char)125, (char)54, (char)90, (char)196, (char)151, (char)133, (char)184, (char)195, (char)187, (char)21, (char)78, (char)163, (char)204, (char)12, (char)110, (char)11, (char)120, (char)38, (char)0, (char)56, (char)180, (char)198, (char)189, (char)152, (char)249, (char)229, (char)50, (char)52, (char)225, (char)158, (char)53, (char)58, (char)146, (char)236, (char)61, (char)8, (char)127, (char)90, (char)204, (char)128, (char)86, (char)123, (char)201, (char)118, (char)88, (char)37, (char)235, (char)42, (char)26, (char)48, (char)38, (char)27, (char)186, (char)33, (char)203, (char)20, (char)136, (char)143, (char)180, (char)180, (char)123, (char)163, (char)43, (char)218, (char)222, (char)140, (char)90, (char)155, (char)138, (char)54, (char)9, (char)91, (char)73, (char)238}));
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)12255) ;
        p131.data__SET(new char[] {(char)215, (char)94, (char)1, (char)173, (char)238, (char)163, (char)164, (char)129, (char)41, (char)51, (char)186, (char)119, (char)126, (char)115, (char)6, (char)24, (char)105, (char)175, (char)18, (char)247, (char)112, (char)248, (char)157, (char)19, (char)6, (char)174, (char)223, (char)6, (char)139, (char)54, (char)151, (char)225, (char)99, (char)136, (char)116, (char)70, (char)8, (char)104, (char)147, (char)55, (char)104, (char)214, (char)74, (char)144, (char)68, (char)254, (char)53, (char)161, (char)39, (char)93, (char)95, (char)26, (char)144, (char)132, (char)119, (char)142, (char)119, (char)149, (char)73, (char)233, (char)236, (char)253, (char)128, (char)241, (char)163, (char)58, (char)42, (char)182, (char)207, (char)20, (char)18, (char)60, (char)246, (char)147, (char)247, (char)231, (char)181, (char)98, (char)18, (char)20, (char)1, (char)48, (char)29, (char)3, (char)206, (char)211, (char)115, (char)204, (char)181, (char)219, (char)140, (char)171, (char)20, (char)135, (char)254, (char)101, (char)197, (char)168, (char)0, (char)31, (char)148, (char)82, (char)197, (char)215, (char)233, (char)22, (char)24, (char)125, (char)113, (char)238, (char)192, (char)126, (char)217, (char)51, (char)235, (char)81, (char)152, (char)94, (char)245, (char)86, (char)16, (char)188, (char)212, (char)200, (char)84, (char)232, (char)250, (char)238, (char)136, (char)188, (char)205, (char)203, (char)17, (char)63, (char)171, (char)122, (char)56, (char)244, (char)137, (char)157, (char)186, (char)54, (char)173, (char)11, (char)155, (char)207, (char)197, (char)162, (char)230, (char)54, (char)193, (char)216, (char)148, (char)249, (char)203, (char)182, (char)235, (char)165, (char)234, (char)30, (char)85, (char)212, (char)18, (char)204, (char)171, (char)95, (char)66, (char)216, (char)203, (char)163, (char)204, (char)106, (char)94, (char)51, (char)3, (char)226, (char)20, (char)25, (char)197, (char)125, (char)54, (char)90, (char)196, (char)151, (char)133, (char)184, (char)195, (char)187, (char)21, (char)78, (char)163, (char)204, (char)12, (char)110, (char)11, (char)120, (char)38, (char)0, (char)56, (char)180, (char)198, (char)189, (char)152, (char)249, (char)229, (char)50, (char)52, (char)225, (char)158, (char)53, (char)58, (char)146, (char)236, (char)61, (char)8, (char)127, (char)90, (char)204, (char)128, (char)86, (char)123, (char)201, (char)118, (char)88, (char)37, (char)235, (char)42, (char)26, (char)48, (char)38, (char)27, (char)186, (char)33, (char)203, (char)20, (char)136, (char)143, (char)180, (char)180, (char)123, (char)163, (char)43, (char)218, (char)222, (char)140, (char)90, (char)155, (char)138, (char)54, (char)9, (char)91, (char)73, (char)238}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3953497058L);
            assert(pack.max_distance_GET() == (char)4825);
            assert(pack.current_distance_GET() == (char)23603);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.id_GET() == (char)196);
            assert(pack.min_distance_GET() == (char)12802);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_225);
            assert(pack.covariance_GET() == (char)237);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p132.min_distance_SET((char)12802) ;
        p132.max_distance_SET((char)4825) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_225) ;
        p132.time_boot_ms_SET(3953497058L) ;
        p132.current_distance_SET((char)23603) ;
        p132.covariance_SET((char)237) ;
        p132.id_SET((char)196) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mask_GET() == 2235390026466615205L);
            assert(pack.grid_spacing_GET() == (char)41184);
            assert(pack.lon_GET() == -550137980);
            assert(pack.lat_GET() == 512797461);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(512797461) ;
        p133.grid_spacing_SET((char)41184) ;
        p133.lon_SET(-550137980) ;
        p133.mask_SET(2235390026466615205L) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.gridbit_GET() == (char)211);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -3757, (short) -18854, (short) -32427, (short)6746, (short)7042, (short)15283, (short) -12703, (short)5841, (short)30809, (short)5602, (short) -1607, (short)6003, (short)21090, (short) -25702, (short)20182, (short)21987}));
            assert(pack.lat_GET() == -1732397576);
            assert(pack.lon_GET() == 1628976487);
            assert(pack.grid_spacing_GET() == (char)8431);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.data__SET(new short[] {(short) -3757, (short) -18854, (short) -32427, (short)6746, (short)7042, (short)15283, (short) -12703, (short)5841, (short)30809, (short)5602, (short) -1607, (short)6003, (short)21090, (short) -25702, (short)20182, (short)21987}, 0) ;
        p134.lon_SET(1628976487) ;
        p134.lat_SET(-1732397576) ;
        p134.gridbit_SET((char)211) ;
        p134.grid_spacing_SET((char)8431) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1681063420);
            assert(pack.lon_GET() == 1801610869);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-1681063420) ;
        p135.lon_SET(1801610869) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -765789331);
            assert(pack.terrain_height_GET() == 1.6224986E38F);
            assert(pack.loaded_GET() == (char)20959);
            assert(pack.current_height_GET() == -3.247424E38F);
            assert(pack.pending_GET() == (char)36797);
            assert(pack.spacing_GET() == (char)51467);
            assert(pack.lon_GET() == 212544712);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(-765789331) ;
        p136.current_height_SET(-3.247424E38F) ;
        p136.lon_SET(212544712) ;
        p136.pending_SET((char)36797) ;
        p136.terrain_height_SET(1.6224986E38F) ;
        p136.loaded_SET((char)20959) ;
        p136.spacing_SET((char)51467) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == 3.3809717E37F);
            assert(pack.press_abs_GET() == 3.3880432E37F);
            assert(pack.temperature_GET() == (short)8973);
            assert(pack.time_boot_ms_GET() == 1923727513L);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_diff_SET(3.3809717E37F) ;
        p137.time_boot_ms_SET(1923727513L) ;
        p137.press_abs_SET(3.3880432E37F) ;
        p137.temperature_SET((short)8973) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -1.946211E38F);
            assert(pack.time_usec_GET() == 673340358781522543L);
            assert(pack.z_GET() == -2.2830085E38F);
            assert(pack.y_GET() == -3.1302821E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.1043692E38F, -2.5087882E38F, -3.2620974E38F, 2.4783926E37F}));
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.x_SET(-1.946211E38F) ;
        p138.q_SET(new float[] {2.1043692E38F, -2.5087882E38F, -3.2620974E38F, 2.4783926E37F}, 0) ;
        p138.y_SET(-3.1302821E37F) ;
        p138.time_usec_SET(673340358781522543L) ;
        p138.z_SET(-2.2830085E38F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)70);
            assert(pack.group_mlx_GET() == (char)22);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.6134256E38F, 2.3228385E38F, 2.908268E38F, -5.3663433E37F, 2.418344E38F, -8.2871193E37F, 3.3564382E38F, 2.090832E38F}));
            assert(pack.target_component_GET() == (char)148);
            assert(pack.time_usec_GET() == 1425108636435051933L);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.group_mlx_SET((char)22) ;
        p139.time_usec_SET(1425108636435051933L) ;
        p139.target_component_SET((char)148) ;
        p139.target_system_SET((char)70) ;
        p139.controls_SET(new float[] {-1.6134256E38F, 2.3228385E38F, 2.908268E38F, -5.3663433E37F, 2.418344E38F, -8.2871193E37F, 3.3564382E38F, 2.090832E38F}, 0) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3565929767072031354L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.711992E38F, -2.889844E38F, -3.3805952E37F, 8.2713933E37F, 3.5900438E37F, 5.0022106E37F, 2.0670893E37F, 1.709186E38F}));
            assert(pack.group_mlx_GET() == (char)155);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)155) ;
        p140.time_usec_SET(3565929767072031354L) ;
        p140.controls_SET(new float[] {-2.711992E38F, -2.889844E38F, -3.3805952E37F, 8.2713933E37F, 3.5900438E37F, 5.0022106E37F, 2.0670893E37F, 1.709186E38F}, 0) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7111814981960501380L);
            assert(pack.bottom_clearance_GET() == 1.8100059E38F);
            assert(pack.altitude_monotonic_GET() == 7.2384223E37F);
            assert(pack.altitude_relative_GET() == 5.5866336E37F);
            assert(pack.altitude_terrain_GET() == -2.3951514E38F);
            assert(pack.altitude_local_GET() == -2.1388807E38F);
            assert(pack.altitude_amsl_GET() == -1.5508909E37F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_local_SET(-2.1388807E38F) ;
        p141.altitude_monotonic_SET(7.2384223E37F) ;
        p141.bottom_clearance_SET(1.8100059E38F) ;
        p141.altitude_amsl_SET(-1.5508909E37F) ;
        p141.time_usec_SET(7111814981960501380L) ;
        p141.altitude_relative_SET(5.5866336E37F) ;
        p141.altitude_terrain_SET(-2.3951514E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.transfer_type_GET() == (char)183);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)82, (char)113, (char)222, (char)93, (char)69, (char)108, (char)144, (char)252, (char)249, (char)232, (char)240, (char)2, (char)25, (char)14, (char)91, (char)66, (char)147, (char)15, (char)197, (char)182, (char)136, (char)149, (char)125, (char)153, (char)181, (char)250, (char)137, (char)41, (char)62, (char)128, (char)139, (char)187, (char)94, (char)47, (char)178, (char)2, (char)215, (char)178, (char)132, (char)137, (char)203, (char)150, (char)145, (char)228, (char)6, (char)88, (char)137, (char)101, (char)19, (char)205, (char)48, (char)138, (char)49, (char)88, (char)134, (char)181, (char)63, (char)126, (char)54, (char)209, (char)241, (char)105, (char)206, (char)183, (char)250, (char)113, (char)255, (char)92, (char)67, (char)170, (char)211, (char)21, (char)204, (char)187, (char)145, (char)120, (char)64, (char)93, (char)17, (char)1, (char)80, (char)127, (char)7, (char)130, (char)153, (char)190, (char)138, (char)65, (char)10, (char)139, (char)251, (char)91, (char)221, (char)15, (char)198, (char)61, (char)32, (char)185, (char)163, (char)241, (char)212, (char)190, (char)219, (char)155, (char)21, (char)159, (char)209, (char)26, (char)16, (char)78, (char)34, (char)217, (char)92, (char)18, (char)165, (char)15, (char)2, (char)105, (char)232, (char)137}));
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)99, (char)221, (char)49, (char)87, (char)44, (char)137, (char)68, (char)37, (char)232, (char)55, (char)175, (char)160, (char)137, (char)61, (char)105, (char)165, (char)242, (char)212, (char)136, (char)226, (char)146, (char)248, (char)152, (char)92, (char)214, (char)86, (char)0, (char)152, (char)184, (char)178, (char)212, (char)27, (char)69, (char)199, (char)151, (char)50, (char)79, (char)141, (char)135, (char)237, (char)204, (char)161, (char)9, (char)49, (char)131, (char)75, (char)172, (char)188, (char)23, (char)106, (char)202, (char)106, (char)52, (char)13, (char)1, (char)146, (char)24, (char)150, (char)190, (char)158, (char)87, (char)73, (char)51, (char)220, (char)8, (char)33, (char)114, (char)41, (char)234, (char)124, (char)178, (char)203, (char)45, (char)244, (char)137, (char)106, (char)73, (char)200, (char)109, (char)21, (char)206, (char)211, (char)243, (char)146, (char)140, (char)192, (char)244, (char)105, (char)237, (char)109, (char)217, (char)67, (char)55, (char)133, (char)128, (char)170, (char)234, (char)26, (char)234, (char)152, (char)115, (char)146, (char)77, (char)73, (char)205, (char)8, (char)151, (char)113, (char)235, (char)236, (char)138, (char)61, (char)238, (char)159, (char)92, (char)73, (char)199, (char)154, (char)113, (char)88}));
            assert(pack.uri_type_GET() == (char)146);
            assert(pack.request_id_GET() == (char)239);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.transfer_type_SET((char)183) ;
        p142.storage_SET(new char[] {(char)99, (char)221, (char)49, (char)87, (char)44, (char)137, (char)68, (char)37, (char)232, (char)55, (char)175, (char)160, (char)137, (char)61, (char)105, (char)165, (char)242, (char)212, (char)136, (char)226, (char)146, (char)248, (char)152, (char)92, (char)214, (char)86, (char)0, (char)152, (char)184, (char)178, (char)212, (char)27, (char)69, (char)199, (char)151, (char)50, (char)79, (char)141, (char)135, (char)237, (char)204, (char)161, (char)9, (char)49, (char)131, (char)75, (char)172, (char)188, (char)23, (char)106, (char)202, (char)106, (char)52, (char)13, (char)1, (char)146, (char)24, (char)150, (char)190, (char)158, (char)87, (char)73, (char)51, (char)220, (char)8, (char)33, (char)114, (char)41, (char)234, (char)124, (char)178, (char)203, (char)45, (char)244, (char)137, (char)106, (char)73, (char)200, (char)109, (char)21, (char)206, (char)211, (char)243, (char)146, (char)140, (char)192, (char)244, (char)105, (char)237, (char)109, (char)217, (char)67, (char)55, (char)133, (char)128, (char)170, (char)234, (char)26, (char)234, (char)152, (char)115, (char)146, (char)77, (char)73, (char)205, (char)8, (char)151, (char)113, (char)235, (char)236, (char)138, (char)61, (char)238, (char)159, (char)92, (char)73, (char)199, (char)154, (char)113, (char)88}, 0) ;
        p142.uri_SET(new char[] {(char)82, (char)113, (char)222, (char)93, (char)69, (char)108, (char)144, (char)252, (char)249, (char)232, (char)240, (char)2, (char)25, (char)14, (char)91, (char)66, (char)147, (char)15, (char)197, (char)182, (char)136, (char)149, (char)125, (char)153, (char)181, (char)250, (char)137, (char)41, (char)62, (char)128, (char)139, (char)187, (char)94, (char)47, (char)178, (char)2, (char)215, (char)178, (char)132, (char)137, (char)203, (char)150, (char)145, (char)228, (char)6, (char)88, (char)137, (char)101, (char)19, (char)205, (char)48, (char)138, (char)49, (char)88, (char)134, (char)181, (char)63, (char)126, (char)54, (char)209, (char)241, (char)105, (char)206, (char)183, (char)250, (char)113, (char)255, (char)92, (char)67, (char)170, (char)211, (char)21, (char)204, (char)187, (char)145, (char)120, (char)64, (char)93, (char)17, (char)1, (char)80, (char)127, (char)7, (char)130, (char)153, (char)190, (char)138, (char)65, (char)10, (char)139, (char)251, (char)91, (char)221, (char)15, (char)198, (char)61, (char)32, (char)185, (char)163, (char)241, (char)212, (char)190, (char)219, (char)155, (char)21, (char)159, (char)209, (char)26, (char)16, (char)78, (char)34, (char)217, (char)92, (char)18, (char)165, (char)15, (char)2, (char)105, (char)232, (char)137}, 0) ;
        p142.request_id_SET((char)239) ;
        p142.uri_type_SET((char)146) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == -2.330458E38F);
            assert(pack.temperature_GET() == (short)8862);
            assert(pack.time_boot_ms_GET() == 1378276662L);
            assert(pack.press_abs_GET() == -2.3781614E38F);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.press_abs_SET(-2.3781614E38F) ;
        p143.time_boot_ms_SET(1378276662L) ;
        p143.press_diff_SET(-2.330458E38F) ;
        p143.temperature_SET((short)8862) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.acc_GET(),  new float[] {1.7361866E38F, -9.091104E37F, -2.6992068E38F}));
            assert(Arrays.equals(pack.vel_GET(),  new float[] {1.786742E38F, -2.4634796E38F, -2.1667207E38F}));
            assert(pack.est_capabilities_GET() == (char)119);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-5.645892E37F, -2.0650496E37F, 5.5800864E37F}));
            assert(pack.lat_GET() == 1608240167);
            assert(pack.lon_GET() == 1198528624);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {8.919059E37F, 2.423537E38F, -1.3803488E38F, 1.2732439E37F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-8.713884E37F, -2.436481E38F, 2.6399242E38F}));
            assert(pack.alt_GET() == -8.170643E36F);
            assert(pack.custom_state_GET() == 4734040213004381554L);
            assert(pack.timestamp_GET() == 6171816928595516228L);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.alt_SET(-8.170643E36F) ;
        p144.timestamp_SET(6171816928595516228L) ;
        p144.lon_SET(1198528624) ;
        p144.lat_SET(1608240167) ;
        p144.rates_SET(new float[] {-8.713884E37F, -2.436481E38F, 2.6399242E38F}, 0) ;
        p144.vel_SET(new float[] {1.786742E38F, -2.4634796E38F, -2.1667207E38F}, 0) ;
        p144.position_cov_SET(new float[] {-5.645892E37F, -2.0650496E37F, 5.5800864E37F}, 0) ;
        p144.est_capabilities_SET((char)119) ;
        p144.attitude_q_SET(new float[] {8.919059E37F, 2.423537E38F, -1.3803488E38F, 1.2732439E37F}, 0) ;
        p144.custom_state_SET(4734040213004381554L) ;
        p144.acc_SET(new float[] {1.7361866E38F, -9.091104E37F, -2.6992068E38F}, 0) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_rate_GET() == 1.1142165E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {7.2418587E37F, -1.527285E38F, 3.0761093E38F}));
            assert(pack.time_usec_GET() == 3103577874189748074L);
            assert(pack.yaw_rate_GET() == 2.0148581E37F);
            assert(pack.z_vel_GET() == -1.1456667E38F);
            assert(pack.x_vel_GET() == 2.9049958E38F);
            assert(pack.x_pos_GET() == 3.2725889E38F);
            assert(pack.x_acc_GET() == -1.321825E38F);
            assert(pack.y_vel_GET() == -2.7325144E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {-3.2432468E37F, 6.761411E37F, -3.3288168E38F}));
            assert(pack.z_pos_GET() == -3.132563E38F);
            assert(pack.z_acc_GET() == -3.320265E38F);
            assert(pack.y_pos_GET() == -3.0729025E38F);
            assert(pack.airspeed_GET() == -1.5237722E37F);
            assert(pack.y_acc_GET() == -2.35431E37F);
            assert(pack.roll_rate_GET() == 1.731979E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.218388E38F, -2.1129072E38F, -3.2407455E38F, 1.8861101E38F}));
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.y_acc_SET(-2.35431E37F) ;
        p146.airspeed_SET(-1.5237722E37F) ;
        p146.roll_rate_SET(1.731979E38F) ;
        p146.z_vel_SET(-1.1456667E38F) ;
        p146.time_usec_SET(3103577874189748074L) ;
        p146.pitch_rate_SET(1.1142165E38F) ;
        p146.x_vel_SET(2.9049958E38F) ;
        p146.vel_variance_SET(new float[] {7.2418587E37F, -1.527285E38F, 3.0761093E38F}, 0) ;
        p146.z_acc_SET(-3.320265E38F) ;
        p146.z_pos_SET(-3.132563E38F) ;
        p146.x_acc_SET(-1.321825E38F) ;
        p146.x_pos_SET(3.2725889E38F) ;
        p146.y_pos_SET(-3.0729025E38F) ;
        p146.q_SET(new float[] {1.218388E38F, -2.1129072E38F, -3.2407455E38F, 1.8861101E38F}, 0) ;
        p146.yaw_rate_SET(2.0148581E37F) ;
        p146.pos_variance_SET(new float[] {-3.2432468E37F, 6.761411E37F, -3.3288168E38F}, 0) ;
        p146.y_vel_SET(-2.7325144E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
            assert(pack.battery_remaining_GET() == (byte) - 122);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)35702, (char)26182, (char)57205, (char)50796, (char)29649, (char)54526, (char)3612, (char)11770, (char)17930, (char)10004}));
            assert(pack.energy_consumed_GET() == 792869846);
            assert(pack.current_battery_GET() == (short) -18850);
            assert(pack.temperature_GET() == (short) -7722);
            assert(pack.current_consumed_GET() == 1516429567);
            assert(pack.id_GET() == (char)13);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE) ;
        p147.current_consumed_SET(1516429567) ;
        p147.current_battery_SET((short) -18850) ;
        p147.battery_remaining_SET((byte) - 122) ;
        p147.temperature_SET((short) -7722) ;
        p147.id_SET((char)13) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL) ;
        p147.energy_consumed_SET(792869846) ;
        p147.voltages_SET(new char[] {(char)35702, (char)26182, (char)57205, (char)50796, (char)29649, (char)54526, (char)3612, (char)11770, (char)17930, (char)10004}, 0) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)50, (char)69, (char)115, (char)89, (char)40, (char)163, (char)233, (char)79}));
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT));
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)101, (char)178, (char)129, (char)190, (char)194, (char)188, (char)56, (char)143}));
            assert(pack.os_sw_version_GET() == 2653821106L);
            assert(pack.board_version_GET() == 3178861334L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)192, (char)79, (char)67, (char)59, (char)56, (char)234, (char)35, (char)60, (char)131, (char)141, (char)209, (char)120, (char)67, (char)156, (char)230, (char)129, (char)132, (char)7}));
            assert(pack.middleware_sw_version_GET() == 3871083321L);
            assert(pack.uid_GET() == 4544508087954472135L);
            assert(pack.vendor_id_GET() == (char)3914);
            assert(pack.flight_sw_version_GET() == 2607532028L);
            assert(pack.product_id_GET() == (char)16794);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)43, (char)241, (char)25, (char)192, (char)31, (char)106, (char)138, (char)243}));
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.middleware_custom_version_SET(new char[] {(char)43, (char)241, (char)25, (char)192, (char)31, (char)106, (char)138, (char)243}, 0) ;
        p148.os_custom_version_SET(new char[] {(char)50, (char)69, (char)115, (char)89, (char)40, (char)163, (char)233, (char)79}, 0) ;
        p148.uid_SET(4544508087954472135L) ;
        p148.product_id_SET((char)16794) ;
        p148.uid2_SET(new char[] {(char)192, (char)79, (char)67, (char)59, (char)56, (char)234, (char)35, (char)60, (char)131, (char)141, (char)209, (char)120, (char)67, (char)156, (char)230, (char)129, (char)132, (char)7}, 0, PH) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT)) ;
        p148.middleware_sw_version_SET(3871083321L) ;
        p148.flight_custom_version_SET(new char[] {(char)101, (char)178, (char)129, (char)190, (char)194, (char)188, (char)56, (char)143}, 0) ;
        p148.board_version_SET(3178861334L) ;
        p148.vendor_id_SET((char)3914) ;
        p148.os_sw_version_SET(2653821106L) ;
        p148.flight_sw_version_SET(2607532028L) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.size_x_GET() == -1.4625502E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
            assert(pack.angle_x_GET() == 8.836289E37F);
            assert(pack.distance_GET() == 3.3620114E38F);
            assert(pack.position_valid_TRY(ph) == (char)85);
            assert(pack.size_y_GET() == -1.3583769E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {1.638729E37F, 2.901396E37F, 2.9029256E38F, 2.4270692E38F}));
            assert(pack.angle_y_GET() == 2.8167395E38F);
            assert(pack.target_num_GET() == (char)218);
            assert(pack.z_TRY(ph) == 2.3989543E38F);
            assert(pack.time_usec_GET() == 3292439110274098724L);
            assert(pack.x_TRY(ph) == -1.2511766E38F);
            assert(pack.y_TRY(ph) == -2.514192E37F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p149.z_SET(2.3989543E38F, PH) ;
        p149.target_num_SET((char)218) ;
        p149.size_x_SET(-1.4625502E38F) ;
        p149.angle_y_SET(2.8167395E38F) ;
        p149.time_usec_SET(3292439110274098724L) ;
        p149.position_valid_SET((char)85, PH) ;
        p149.distance_SET(3.3620114E38F) ;
        p149.angle_x_SET(8.836289E37F) ;
        p149.y_SET(-2.514192E37F, PH) ;
        p149.x_SET(-1.2511766E38F, PH) ;
        p149.q_SET(new float[] {1.638729E37F, 2.901396E37F, 2.9029256E38F, 2.4270692E38F}, 0, PH) ;
        p149.size_y_SET(-1.3583769E38F) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCRIPT_ITEM.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)96);
            assert(pack.name_LEN(ph) == 11);
            assert(pack.name_TRY(ph).equals("fwcOrzthbdp"));
            assert(pack.target_component_GET() == (char)94);
            assert(pack.seq_GET() == (char)30134);
        });
        GroundControl.SCRIPT_ITEM p180 = CommunicationChannel.new_SCRIPT_ITEM();
        PH.setPack(p180);
        p180.target_component_SET((char)94) ;
        p180.seq_SET((char)30134) ;
        p180.name_SET("fwcOrzthbdp", PH) ;
        p180.target_system_SET((char)96) ;
        CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCRIPT_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)41878);
            assert(pack.target_component_GET() == (char)225);
            assert(pack.target_system_GET() == (char)127);
        });
        GroundControl.SCRIPT_REQUEST p181 = CommunicationChannel.new_SCRIPT_REQUEST();
        PH.setPack(p181);
        p181.target_system_SET((char)127) ;
        p181.seq_SET((char)41878) ;
        p181.target_component_SET((char)225) ;
        CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCRIPT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)133);
            assert(pack.target_system_GET() == (char)12);
        });
        GroundControl.SCRIPT_REQUEST_LIST p182 = CommunicationChannel.new_SCRIPT_REQUEST_LIST();
        PH.setPack(p182);
        p182.target_component_SET((char)133) ;
        p182.target_system_SET((char)12) ;
        CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCRIPT_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)193);
            assert(pack.count_GET() == (char)19570);
            assert(pack.target_system_GET() == (char)97);
        });
        GroundControl.SCRIPT_COUNT p183 = CommunicationChannel.new_SCRIPT_COUNT();
        PH.setPack(p183);
        p183.count_SET((char)19570) ;
        p183.target_component_SET((char)193) ;
        p183.target_system_SET((char)97) ;
        CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCRIPT_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)37479);
        });
        GroundControl.SCRIPT_CURRENT p184 = CommunicationChannel.new_SCRIPT_CURRENT();
        PH.setPack(p184);
        p184.seq_SET((char)37479) ;
        CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.hagl_ratio_GET() == -4.2963163E37F);
            assert(pack.mag_ratio_GET() == 7.5887624E37F);
            assert(pack.vel_ratio_GET() == 8.3903293E37F);
            assert(pack.pos_vert_accuracy_GET() == 2.2234871E38F);
            assert(pack.pos_vert_ratio_GET() == 5.1235577E37F);
            assert(pack.time_usec_GET() == 6043137934921245237L);
            assert(pack.tas_ratio_GET() == 4.904987E37F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE));
            assert(pack.pos_horiz_accuracy_GET() == -2.5377416E38F);
            assert(pack.pos_horiz_ratio_GET() == -2.854483E38F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.hagl_ratio_SET(-4.2963163E37F) ;
        p230.tas_ratio_SET(4.904987E37F) ;
        p230.mag_ratio_SET(7.5887624E37F) ;
        p230.pos_horiz_ratio_SET(-2.854483E38F) ;
        p230.vel_ratio_SET(8.3903293E37F) ;
        p230.pos_vert_ratio_SET(5.1235577E37F) ;
        p230.pos_vert_accuracy_SET(2.2234871E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE)) ;
        p230.pos_horiz_accuracy_SET(-2.5377416E38F) ;
        p230.time_usec_SET(6043137934921245237L) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_y_GET() == -2.2452798E38F);
            assert(pack.wind_alt_GET() == 2.3934986E38F);
            assert(pack.wind_x_GET() == 2.0638496E37F);
            assert(pack.var_horiz_GET() == 1.2611505E38F);
            assert(pack.vert_accuracy_GET() == -1.4353986E38F);
            assert(pack.time_usec_GET() == 1195936400718549015L);
            assert(pack.var_vert_GET() == -1.5604336E38F);
            assert(pack.wind_z_GET() == -4.1542528E37F);
            assert(pack.horiz_accuracy_GET() == -1.6452601E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_y_SET(-2.2452798E38F) ;
        p231.wind_z_SET(-4.1542528E37F) ;
        p231.time_usec_SET(1195936400718549015L) ;
        p231.vert_accuracy_SET(-1.4353986E38F) ;
        p231.horiz_accuracy_SET(-1.6452601E38F) ;
        p231.wind_x_SET(2.0638496E37F) ;
        p231.var_vert_SET(-1.5604336E38F) ;
        p231.wind_alt_SET(2.3934986E38F) ;
        p231.var_horiz_SET(1.2611505E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 1.8346675E38F);
            assert(pack.vert_accuracy_GET() == -3.9328908E37F);
            assert(pack.speed_accuracy_GET() == -1.3856218E38F);
            assert(pack.satellites_visible_GET() == (char)152);
            assert(pack.lat_GET() == 2097759624);
            assert(pack.vd_GET() == 8.57439E37F);
            assert(pack.gps_id_GET() == (char)79);
            assert(pack.time_usec_GET() == 6902556364188212449L);
            assert(pack.hdop_GET() == -2.8686363E38F);
            assert(pack.ve_GET() == 1.8192279E38F);
            assert(pack.vn_GET() == -3.1351314E38F);
            assert(pack.vdop_GET() == 3.2293995E38F);
            assert(pack.lon_GET() == -1840384874);
            assert(pack.horiz_accuracy_GET() == -1.3275927E38F);
            assert(pack.time_week_GET() == (char)53357);
            assert(pack.fix_type_GET() == (char)34);
            assert(pack.time_week_ms_GET() == 2888366166L);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP));
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.lat_SET(2097759624) ;
        p232.lon_SET(-1840384874) ;
        p232.vert_accuracy_SET(-3.9328908E37F) ;
        p232.horiz_accuracy_SET(-1.3275927E38F) ;
        p232.vd_SET(8.57439E37F) ;
        p232.time_week_SET((char)53357) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP)) ;
        p232.speed_accuracy_SET(-1.3856218E38F) ;
        p232.gps_id_SET((char)79) ;
        p232.fix_type_SET((char)34) ;
        p232.hdop_SET(-2.8686363E38F) ;
        p232.vn_SET(-3.1351314E38F) ;
        p232.time_week_ms_SET(2888366166L) ;
        p232.time_usec_SET(6902556364188212449L) ;
        p232.vdop_SET(3.2293995E38F) ;
        p232.satellites_visible_SET((char)152) ;
        p232.ve_SET(1.8192279E38F) ;
        p232.alt_SET(1.8346675E38F) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)184);
            assert(pack.flags_GET() == (char)73);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)180, (char)62, (char)106, (char)75, (char)46, (char)87, (char)85, (char)120, (char)30, (char)25, (char)98, (char)135, (char)141, (char)110, (char)192, (char)191, (char)66, (char)167, (char)70, (char)184, (char)74, (char)249, (char)102, (char)229, (char)16, (char)185, (char)9, (char)83, (char)194, (char)154, (char)193, (char)216, (char)227, (char)203, (char)3, (char)148, (char)143, (char)169, (char)187, (char)253, (char)115, (char)37, (char)79, (char)56, (char)113, (char)60, (char)61, (char)236, (char)66, (char)255, (char)82, (char)68, (char)183, (char)98, (char)35, (char)240, (char)152, (char)247, (char)77, (char)137, (char)16, (char)173, (char)159, (char)209, (char)66, (char)204, (char)180, (char)224, (char)153, (char)228, (char)202, (char)86, (char)10, (char)158, (char)8, (char)194, (char)95, (char)3, (char)48, (char)174, (char)101, (char)26, (char)176, (char)107, (char)138, (char)130, (char)167, (char)197, (char)101, (char)129, (char)107, (char)91, (char)53, (char)15, (char)229, (char)120, (char)86, (char)90, (char)202, (char)179, (char)67, (char)75, (char)167, (char)0, (char)84, (char)132, (char)241, (char)62, (char)133, (char)196, (char)54, (char)110, (char)55, (char)174, (char)249, (char)44, (char)196, (char)230, (char)155, (char)178, (char)0, (char)50, (char)4, (char)29, (char)19, (char)230, (char)142, (char)118, (char)69, (char)217, (char)35, (char)254, (char)3, (char)12, (char)245, (char)141, (char)103, (char)188, (char)45, (char)205, (char)75, (char)242, (char)72, (char)103, (char)194, (char)77, (char)47, (char)103, (char)51, (char)50, (char)115, (char)106, (char)8, (char)255, (char)55, (char)97, (char)212, (char)184, (char)248, (char)29, (char)218, (char)17, (char)39, (char)107, (char)41, (char)148, (char)83, (char)138, (char)99, (char)63, (char)202, (char)2, (char)215, (char)45, (char)53, (char)216, (char)238, (char)169, (char)81, (char)113}));
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)73) ;
        p233.data__SET(new char[] {(char)180, (char)62, (char)106, (char)75, (char)46, (char)87, (char)85, (char)120, (char)30, (char)25, (char)98, (char)135, (char)141, (char)110, (char)192, (char)191, (char)66, (char)167, (char)70, (char)184, (char)74, (char)249, (char)102, (char)229, (char)16, (char)185, (char)9, (char)83, (char)194, (char)154, (char)193, (char)216, (char)227, (char)203, (char)3, (char)148, (char)143, (char)169, (char)187, (char)253, (char)115, (char)37, (char)79, (char)56, (char)113, (char)60, (char)61, (char)236, (char)66, (char)255, (char)82, (char)68, (char)183, (char)98, (char)35, (char)240, (char)152, (char)247, (char)77, (char)137, (char)16, (char)173, (char)159, (char)209, (char)66, (char)204, (char)180, (char)224, (char)153, (char)228, (char)202, (char)86, (char)10, (char)158, (char)8, (char)194, (char)95, (char)3, (char)48, (char)174, (char)101, (char)26, (char)176, (char)107, (char)138, (char)130, (char)167, (char)197, (char)101, (char)129, (char)107, (char)91, (char)53, (char)15, (char)229, (char)120, (char)86, (char)90, (char)202, (char)179, (char)67, (char)75, (char)167, (char)0, (char)84, (char)132, (char)241, (char)62, (char)133, (char)196, (char)54, (char)110, (char)55, (char)174, (char)249, (char)44, (char)196, (char)230, (char)155, (char)178, (char)0, (char)50, (char)4, (char)29, (char)19, (char)230, (char)142, (char)118, (char)69, (char)217, (char)35, (char)254, (char)3, (char)12, (char)245, (char)141, (char)103, (char)188, (char)45, (char)205, (char)75, (char)242, (char)72, (char)103, (char)194, (char)77, (char)47, (char)103, (char)51, (char)50, (char)115, (char)106, (char)8, (char)255, (char)55, (char)97, (char)212, (char)184, (char)248, (char)29, (char)218, (char)17, (char)39, (char)107, (char)41, (char)148, (char)83, (char)138, (char)99, (char)63, (char)202, (char)2, (char)215, (char)45, (char)53, (char)216, (char)238, (char)169, (char)81, (char)113}, 0) ;
        p233.len_SET((char)184) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.wp_distance_GET() == (char)8286);
            assert(pack.latitude_GET() == -993171691);
            assert(pack.throttle_GET() == (byte)47);
            assert(pack.custom_mode_GET() == 2565200303L);
            assert(pack.heading_GET() == (char)10369);
            assert(pack.climb_rate_GET() == (byte)109);
            assert(pack.temperature_GET() == (byte)98);
            assert(pack.failsafe_GET() == (char)252);
            assert(pack.battery_remaining_GET() == (char)164);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            assert(pack.airspeed_sp_GET() == (char)103);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED));
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
            assert(pack.wp_num_GET() == (char)109);
            assert(pack.longitude_GET() == -413030698);
            assert(pack.airspeed_GET() == (char)50);
            assert(pack.pitch_GET() == (short) -20984);
            assert(pack.groundspeed_GET() == (char)60);
            assert(pack.gps_nsat_GET() == (char)83);
            assert(pack.heading_sp_GET() == (short)8613);
            assert(pack.altitude_amsl_GET() == (short)22184);
            assert(pack.altitude_sp_GET() == (short) -26740);
            assert(pack.temperature_air_GET() == (byte)38);
            assert(pack.roll_GET() == (short) -7751);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.gps_nsat_SET((char)83) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX) ;
        p234.failsafe_SET((char)252) ;
        p234.heading_SET((char)10369) ;
        p234.wp_distance_SET((char)8286) ;
        p234.latitude_SET(-993171691) ;
        p234.longitude_SET(-413030698) ;
        p234.groundspeed_SET((char)60) ;
        p234.pitch_SET((short) -20984) ;
        p234.roll_SET((short) -7751) ;
        p234.climb_rate_SET((byte)109) ;
        p234.battery_remaining_SET((char)164) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)) ;
        p234.heading_sp_SET((short)8613) ;
        p234.airspeed_SET((char)50) ;
        p234.throttle_SET((byte)47) ;
        p234.altitude_sp_SET((short) -26740) ;
        p234.custom_mode_SET(2565200303L) ;
        p234.wp_num_SET((char)109) ;
        p234.temperature_air_SET((byte)38) ;
        p234.altitude_amsl_SET((short)22184) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF) ;
        p234.airspeed_sp_SET((char)103) ;
        p234.temperature_SET((byte)98) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_z_GET() == 2.4165463E38F);
            assert(pack.vibration_y_GET() == 1.3423265E38F);
            assert(pack.clipping_2_GET() == 495151535L);
            assert(pack.clipping_1_GET() == 496690271L);
            assert(pack.vibration_x_GET() == 3.3941755E38F);
            assert(pack.time_usec_GET() == 3411482317409639462L);
            assert(pack.clipping_0_GET() == 3895317387L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(3411482317409639462L) ;
        p241.clipping_2_SET(495151535L) ;
        p241.vibration_x_SET(3.3941755E38F) ;
        p241.clipping_0_SET(3895317387L) ;
        p241.vibration_y_SET(1.3423265E38F) ;
        p241.vibration_z_SET(2.4165463E38F) ;
        p241.clipping_1_SET(496690271L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 5.8842495E37F);
            assert(pack.altitude_GET() == 1632715884);
            assert(pack.y_GET() == 3.1467272E38F);
            assert(pack.latitude_GET() == 732348341);
            assert(pack.approach_x_GET() == 2.012285E38F);
            assert(pack.approach_y_GET() == 3.0469694E38F);
            assert(pack.time_usec_TRY(ph) == 8157752870618267121L);
            assert(pack.longitude_GET() == 1187116226);
            assert(pack.approach_z_GET() == 7.992715E37F);
            assert(pack.x_GET() == -1.0028938E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.7878105E37F, -2.9468737E38F, 1.4151854E38F, -1.1500787E38F}));
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.approach_z_SET(7.992715E37F) ;
        p242.x_SET(-1.0028938E38F) ;
        p242.altitude_SET(1632715884) ;
        p242.z_SET(5.8842495E37F) ;
        p242.y_SET(3.1467272E38F) ;
        p242.latitude_SET(732348341) ;
        p242.approach_x_SET(2.012285E38F) ;
        p242.time_usec_SET(8157752870618267121L, PH) ;
        p242.approach_y_SET(3.0469694E38F) ;
        p242.q_SET(new float[] {3.7878105E37F, -2.9468737E38F, 1.4151854E38F, -1.1500787E38F}, 0) ;
        p242.longitude_SET(1187116226) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 2.9283023E38F);
            assert(pack.longitude_GET() == -1985473720);
            assert(pack.latitude_GET() == -1957016638);
            assert(pack.target_system_GET() == (char)31);
            assert(pack.approach_y_GET() == -6.539354E37F);
            assert(pack.time_usec_TRY(ph) == 5691376608126518924L);
            assert(pack.altitude_GET() == 565172177);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.2101828E38F, -2.9191383E38F, -7.198417E37F, 1.697541E38F}));
            assert(pack.y_GET() == -2.7250223E38F);
            assert(pack.x_GET() == -3.2484502E38F);
            assert(pack.approach_z_GET() == 2.5197874E38F);
            assert(pack.approach_x_GET() == 1.3148085E38F);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.y_SET(-2.7250223E38F) ;
        p243.latitude_SET(-1957016638) ;
        p243.target_system_SET((char)31) ;
        p243.longitude_SET(-1985473720) ;
        p243.altitude_SET(565172177) ;
        p243.approach_x_SET(1.3148085E38F) ;
        p243.q_SET(new float[] {3.2101828E38F, -2.9191383E38F, -7.198417E37F, 1.697541E38F}, 0) ;
        p243.z_SET(2.9283023E38F) ;
        p243.approach_y_SET(-6.539354E37F) ;
        p243.x_SET(-3.2484502E38F) ;
        p243.time_usec_SET(5691376608126518924L, PH) ;
        p243.approach_z_SET(2.5197874E38F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)51080);
            assert(pack.interval_us_GET() == -1908348241);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-1908348241) ;
        p244.message_id_SET((char)51080) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.callsign_LEN(ph) == 8);
            assert(pack.callsign_TRY(ph).equals("irfjsixd"));
            assert(pack.ICAO_address_GET() == 2485490097L);
            assert(pack.heading_GET() == (char)28146);
            assert(pack.lat_GET() == -297432522);
            assert(pack.squawk_GET() == (char)29166);
            assert(pack.altitude_GET() == 1413028842);
            assert(pack.tslc_GET() == (char)162);
            assert(pack.hor_velocity_GET() == (char)12095);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SPACE);
            assert(pack.ver_velocity_GET() == (short)10558);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS));
            assert(pack.lon_GET() == 705097337);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.lat_SET(-297432522) ;
        p246.heading_SET((char)28146) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SPACE) ;
        p246.altitude_SET(1413028842) ;
        p246.hor_velocity_SET((char)12095) ;
        p246.squawk_SET((char)29166) ;
        p246.tslc_SET((char)162) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.callsign_SET("irfjsixd", PH) ;
        p246.ICAO_address_SET(2485490097L) ;
        p246.ver_velocity_SET((short)10558) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS)) ;
        p246.lon_SET(705097337) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.altitude_minimum_delta_GET() == -2.6544728E37F);
            assert(pack.id_GET() == 231096162L);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
            assert(pack.time_to_minimum_delta_GET() == 2.4784409E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            assert(pack.horizontal_minimum_delta_GET() == 3.0075337E38F);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE) ;
        p247.horizontal_minimum_delta_SET(3.0075337E38F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.altitude_minimum_delta_SET(-2.6544728E37F) ;
        p247.id_SET(231096162L) ;
        p247.time_to_minimum_delta_SET(2.4784409E38F) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)20, (char)225, (char)141, (char)92, (char)99, (char)143, (char)166, (char)84, (char)31, (char)63, (char)62, (char)99, (char)145, (char)109, (char)53, (char)3, (char)195, (char)85, (char)72, (char)205, (char)197, (char)118, (char)188, (char)62, (char)224, (char)210, (char)172, (char)235, (char)16, (char)12, (char)162, (char)139, (char)66, (char)64, (char)232, (char)222, (char)184, (char)183, (char)73, (char)146, (char)116, (char)177, (char)197, (char)157, (char)138, (char)136, (char)8, (char)131, (char)219, (char)236, (char)87, (char)61, (char)217, (char)254, (char)182, (char)128, (char)196, (char)138, (char)61, (char)150, (char)9, (char)10, (char)51, (char)188, (char)71, (char)162, (char)208, (char)248, (char)235, (char)44, (char)183, (char)123, (char)231, (char)124, (char)219, (char)221, (char)131, (char)68, (char)206, (char)80, (char)165, (char)76, (char)126, (char)28, (char)133, (char)250, (char)187, (char)200, (char)89, (char)247, (char)83, (char)155, (char)210, (char)239, (char)163, (char)135, (char)215, (char)158, (char)211, (char)168, (char)17, (char)220, (char)135, (char)81, (char)10, (char)74, (char)16, (char)163, (char)116, (char)228, (char)183, (char)16, (char)66, (char)54, (char)108, (char)5, (char)135, (char)137, (char)181, (char)100, (char)36, (char)97, (char)238, (char)228, (char)14, (char)151, (char)45, (char)159, (char)245, (char)36, (char)54, (char)166, (char)204, (char)9, (char)64, (char)244, (char)243, (char)40, (char)159, (char)10, (char)225, (char)117, (char)205, (char)54, (char)58, (char)186, (char)112, (char)106, (char)102, (char)128, (char)83, (char)215, (char)87, (char)72, (char)221, (char)133, (char)136, (char)179, (char)39, (char)172, (char)163, (char)118, (char)223, (char)238, (char)34, (char)143, (char)220, (char)142, (char)170, (char)194, (char)154, (char)0, (char)7, (char)209, (char)97, (char)68, (char)2, (char)50, (char)33, (char)39, (char)120, (char)81, (char)159, (char)140, (char)100, (char)203, (char)11, (char)46, (char)120, (char)171, (char)137, (char)134, (char)102, (char)242, (char)207, (char)231, (char)252, (char)87, (char)2, (char)122, (char)44, (char)30, (char)37, (char)170, (char)68, (char)53, (char)17, (char)38, (char)68, (char)247, (char)101, (char)111, (char)29, (char)213, (char)232, (char)60, (char)239, (char)232, (char)240, (char)13, (char)148, (char)64, (char)207, (char)77, (char)108, (char)58, (char)35, (char)108, (char)141, (char)238, (char)30, (char)155, (char)217, (char)126, (char)200, (char)189, (char)170, (char)138, (char)83, (char)240, (char)26, (char)134, (char)179, (char)57, (char)28, (char)227, (char)157, (char)250, (char)54}));
            assert(pack.message_type_GET() == (char)60300);
            assert(pack.target_system_GET() == (char)84);
            assert(pack.target_network_GET() == (char)117);
            assert(pack.target_component_GET() == (char)144);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_component_SET((char)144) ;
        p248.message_type_SET((char)60300) ;
        p248.target_network_SET((char)117) ;
        p248.payload_SET(new char[] {(char)20, (char)225, (char)141, (char)92, (char)99, (char)143, (char)166, (char)84, (char)31, (char)63, (char)62, (char)99, (char)145, (char)109, (char)53, (char)3, (char)195, (char)85, (char)72, (char)205, (char)197, (char)118, (char)188, (char)62, (char)224, (char)210, (char)172, (char)235, (char)16, (char)12, (char)162, (char)139, (char)66, (char)64, (char)232, (char)222, (char)184, (char)183, (char)73, (char)146, (char)116, (char)177, (char)197, (char)157, (char)138, (char)136, (char)8, (char)131, (char)219, (char)236, (char)87, (char)61, (char)217, (char)254, (char)182, (char)128, (char)196, (char)138, (char)61, (char)150, (char)9, (char)10, (char)51, (char)188, (char)71, (char)162, (char)208, (char)248, (char)235, (char)44, (char)183, (char)123, (char)231, (char)124, (char)219, (char)221, (char)131, (char)68, (char)206, (char)80, (char)165, (char)76, (char)126, (char)28, (char)133, (char)250, (char)187, (char)200, (char)89, (char)247, (char)83, (char)155, (char)210, (char)239, (char)163, (char)135, (char)215, (char)158, (char)211, (char)168, (char)17, (char)220, (char)135, (char)81, (char)10, (char)74, (char)16, (char)163, (char)116, (char)228, (char)183, (char)16, (char)66, (char)54, (char)108, (char)5, (char)135, (char)137, (char)181, (char)100, (char)36, (char)97, (char)238, (char)228, (char)14, (char)151, (char)45, (char)159, (char)245, (char)36, (char)54, (char)166, (char)204, (char)9, (char)64, (char)244, (char)243, (char)40, (char)159, (char)10, (char)225, (char)117, (char)205, (char)54, (char)58, (char)186, (char)112, (char)106, (char)102, (char)128, (char)83, (char)215, (char)87, (char)72, (char)221, (char)133, (char)136, (char)179, (char)39, (char)172, (char)163, (char)118, (char)223, (char)238, (char)34, (char)143, (char)220, (char)142, (char)170, (char)194, (char)154, (char)0, (char)7, (char)209, (char)97, (char)68, (char)2, (char)50, (char)33, (char)39, (char)120, (char)81, (char)159, (char)140, (char)100, (char)203, (char)11, (char)46, (char)120, (char)171, (char)137, (char)134, (char)102, (char)242, (char)207, (char)231, (char)252, (char)87, (char)2, (char)122, (char)44, (char)30, (char)37, (char)170, (char)68, (char)53, (char)17, (char)38, (char)68, (char)247, (char)101, (char)111, (char)29, (char)213, (char)232, (char)60, (char)239, (char)232, (char)240, (char)13, (char)148, (char)64, (char)207, (char)77, (char)108, (char)58, (char)35, (char)108, (char)141, (char)238, (char)30, (char)155, (char)217, (char)126, (char)200, (char)189, (char)170, (char)138, (char)83, (char)240, (char)26, (char)134, (char)179, (char)57, (char)28, (char)227, (char)157, (char)250, (char)54}, 0) ;
        p248.target_system_SET((char)84) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.ver_GET() == (char)43);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)54, (byte) - 100, (byte) - 69, (byte) - 46, (byte) - 30, (byte) - 30, (byte)23, (byte)100, (byte)16, (byte)22, (byte) - 5, (byte)77, (byte)49, (byte) - 92, (byte)57, (byte)15, (byte)21, (byte)121, (byte) - 19, (byte)118, (byte) - 62, (byte) - 39, (byte) - 91, (byte) - 54, (byte)96, (byte)11, (byte) - 113, (byte)39, (byte)11, (byte)64, (byte) - 37, (byte)77}));
            assert(pack.type_GET() == (char)47);
            assert(pack.address_GET() == (char)25691);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)25691) ;
        p249.ver_SET((char)43) ;
        p249.type_SET((char)47) ;
        p249.value_SET(new byte[] {(byte)54, (byte) - 100, (byte) - 69, (byte) - 46, (byte) - 30, (byte) - 30, (byte)23, (byte)100, (byte)16, (byte)22, (byte) - 5, (byte)77, (byte)49, (byte) - 92, (byte)57, (byte)15, (byte)21, (byte)121, (byte) - 19, (byte)118, (byte) - 62, (byte) - 39, (byte) - 91, (byte) - 54, (byte)96, (byte)11, (byte) - 113, (byte)39, (byte)11, (byte)64, (byte) - 37, (byte)77}, 0) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4748192487967843822L);
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("koyyvgmRsv"));
            assert(pack.x_GET() == -1.3922331E37F);
            assert(pack.y_GET() == 8.855577E37F);
            assert(pack.z_GET() == -1.6869761E38F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("koyyvgmRsv", PH) ;
        p250.y_SET(8.855577E37F) ;
        p250.time_usec_SET(4748192487967843822L) ;
        p250.x_SET(-1.3922331E37F) ;
        p250.z_SET(-1.6869761E38F) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 6);
            assert(pack.name_TRY(ph).equals("ldnlgh"));
            assert(pack.value_GET() == 1.5346741E38F);
            assert(pack.time_boot_ms_GET() == 1971723646L);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(1971723646L) ;
        p251.value_SET(1.5346741E38F) ;
        p251.name_SET("ldnlgh", PH) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1835604040L);
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("cvtyfRnuhv"));
            assert(pack.value_GET() == 930478049);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(1835604040L) ;
        p252.name_SET("cvtyfRnuhv", PH) ;
        p252.value_SET(930478049) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_DEBUG);
            assert(pack.text_LEN(ph) == 27);
            assert(pack.text_TRY(ph).equals("jxgqcvdquzlupfXbkrqqyddmhyl"));
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_DEBUG) ;
        p253.text_SET("jxgqcvdquzlupfXbkrqqyddmhyl", PH) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -2.8717868E38F);
            assert(pack.ind_GET() == (char)233);
            assert(pack.time_boot_ms_GET() == 4165238944L);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.ind_SET((char)233) ;
        p254.value_SET(-2.8717868E38F) ;
        p254.time_boot_ms_SET(4165238944L) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)249);
            assert(pack.target_system_GET() == (char)59);
            assert(pack.initial_timestamp_GET() == 4604198855348119204L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)251, (char)196, (char)150, (char)89, (char)76, (char)206, (char)147, (char)205, (char)179, (char)108, (char)91, (char)91, (char)174, (char)10, (char)173, (char)37, (char)64, (char)142, (char)236, (char)193, (char)46, (char)253, (char)138, (char)149, (char)234, (char)43, (char)83, (char)155, (char)120, (char)92, (char)18, (char)67}));
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)59) ;
        p256.secret_key_SET(new char[] {(char)251, (char)196, (char)150, (char)89, (char)76, (char)206, (char)147, (char)205, (char)179, (char)108, (char)91, (char)91, (char)174, (char)10, (char)173, (char)37, (char)64, (char)142, (char)236, (char)193, (char)46, (char)253, (char)138, (char)149, (char)234, (char)43, (char)83, (char)155, (char)120, (char)92, (char)18, (char)67}, 0) ;
        p256.target_component_SET((char)249) ;
        p256.initial_timestamp_SET(4604198855348119204L) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.state_GET() == (char)128);
            assert(pack.time_boot_ms_GET() == 1141410363L);
            assert(pack.last_change_ms_GET() == 1928061128L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.last_change_ms_SET(1928061128L) ;
        p257.state_SET((char)128) ;
        p257.time_boot_ms_SET(1141410363L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)23);
            assert(pack.tune_LEN(ph) == 27);
            assert(pack.tune_TRY(ph).equals("tnjKCerwkteJJnxaBRjtnpDukhw"));
            assert(pack.target_component_GET() == (char)64);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.tune_SET("tnjKCerwkteJJnxaBRjtnpDukhw", PH) ;
        p258.target_component_SET((char)64) ;
        p258.target_system_SET((char)23) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.sensor_size_h_GET() == 2.916389E38F);
            assert(pack.lens_id_GET() == (char)220);
            assert(pack.resolution_h_GET() == (char)54998);
            assert(pack.cam_definition_uri_LEN(ph) == 101);
            assert(pack.cam_definition_uri_TRY(ph).equals("qxCwdmBqqpbwltvgisroycTjsjdaAutjgmtWaqQhfiitdqeryxzjsdztsYoRvgnlppjRjJnytgjlauRthiqlceUikbadhtYfpagsa"));
            assert(pack.focal_length_GET() == -2.8178897E38F);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)72, (char)13, (char)120, (char)81, (char)64, (char)237, (char)46, (char)134, (char)98, (char)228, (char)55, (char)143, (char)34, (char)0, (char)175, (char)134, (char)89, (char)221, (char)134, (char)117, (char)18, (char)184, (char)32, (char)239, (char)233, (char)19, (char)216, (char)36, (char)163, (char)186, (char)7, (char)220}));
            assert(pack.time_boot_ms_GET() == 366478898L);
            assert(pack.firmware_version_GET() == 2157305538L);
            assert(pack.cam_definition_version_GET() == (char)35968);
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE));
            assert(pack.resolution_v_GET() == (char)36734);
            assert(pack.sensor_size_v_GET() == 1.4429732E38F);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)181, (char)69, (char)64, (char)179, (char)245, (char)156, (char)16, (char)97, (char)153, (char)10, (char)108, (char)116, (char)134, (char)149, (char)42, (char)13, (char)105, (char)227, (char)79, (char)37, (char)254, (char)44, (char)94, (char)12, (char)18, (char)93, (char)221, (char)108, (char)164, (char)202, (char)23, (char)102}));
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.resolution_h_SET((char)54998) ;
        p259.time_boot_ms_SET(366478898L) ;
        p259.cam_definition_version_SET((char)35968) ;
        p259.lens_id_SET((char)220) ;
        p259.sensor_size_v_SET(1.4429732E38F) ;
        p259.firmware_version_SET(2157305538L) ;
        p259.cam_definition_uri_SET("qxCwdmBqqpbwltvgisroycTjsjdaAutjgmtWaqQhfiitdqeryxzjsdztsYoRvgnlppjRjJnytgjlauRthiqlceUikbadhtYfpagsa", PH) ;
        p259.focal_length_SET(-2.8178897E38F) ;
        p259.sensor_size_h_SET(2.916389E38F) ;
        p259.resolution_v_SET((char)36734) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE)) ;
        p259.model_name_SET(new char[] {(char)72, (char)13, (char)120, (char)81, (char)64, (char)237, (char)46, (char)134, (char)98, (char)228, (char)55, (char)143, (char)34, (char)0, (char)175, (char)134, (char)89, (char)221, (char)134, (char)117, (char)18, (char)184, (char)32, (char)239, (char)233, (char)19, (char)216, (char)36, (char)163, (char)186, (char)7, (char)220}, 0) ;
        p259.vendor_name_SET(new char[] {(char)181, (char)69, (char)64, (char)179, (char)245, (char)156, (char)16, (char)97, (char)153, (char)10, (char)108, (char)116, (char)134, (char)149, (char)42, (char)13, (char)105, (char)227, (char)79, (char)37, (char)254, (char)44, (char)94, (char)12, (char)18, (char)93, (char)221, (char)108, (char)164, (char)202, (char)23, (char)102}, 0) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 12210792L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE) ;
        p260.time_boot_ms_SET(12210792L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.storage_count_GET() == (char)204);
            assert(pack.write_speed_GET() == -9.530752E37F);
            assert(pack.status_GET() == (char)214);
            assert(pack.read_speed_GET() == 1.8164028E38F);
            assert(pack.time_boot_ms_GET() == 769761520L);
            assert(pack.total_capacity_GET() == -7.6268E36F);
            assert(pack.storage_id_GET() == (char)152);
            assert(pack.available_capacity_GET() == -1.2704848E37F);
            assert(pack.used_capacity_GET() == -2.019219E38F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.write_speed_SET(-9.530752E37F) ;
        p261.status_SET((char)214) ;
        p261.time_boot_ms_SET(769761520L) ;
        p261.storage_count_SET((char)204) ;
        p261.read_speed_SET(1.8164028E38F) ;
        p261.used_capacity_SET(-2.019219E38F) ;
        p261.storage_id_SET((char)152) ;
        p261.total_capacity_SET(-7.6268E36F) ;
        p261.available_capacity_SET(-1.2704848E37F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.image_interval_GET() == 1.4286433E38F);
            assert(pack.available_capacity_GET() == -9.770584E37F);
            assert(pack.video_status_GET() == (char)110);
            assert(pack.image_status_GET() == (char)42);
            assert(pack.time_boot_ms_GET() == 1648454760L);
            assert(pack.recording_time_ms_GET() == 685869717L);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.recording_time_ms_SET(685869717L) ;
        p262.available_capacity_SET(-9.770584E37F) ;
        p262.image_interval_SET(1.4286433E38F) ;
        p262.video_status_SET((char)110) ;
        p262.time_boot_ms_SET(1648454760L) ;
        p262.image_status_SET((char)42) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 784054054);
            assert(pack.time_utc_GET() == 7219116875422947357L);
            assert(pack.image_index_GET() == 208387856);
            assert(pack.capture_result_GET() == (byte)38);
            assert(pack.alt_GET() == -219192694);
            assert(pack.relative_alt_GET() == 1239537282);
            assert(pack.lon_GET() == -1212691453);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.8010478E38F, 2.6314648E38F, 4.221267E37F, -3.3765506E38F}));
            assert(pack.time_boot_ms_GET() == 805681312L);
            assert(pack.file_url_LEN(ph) == 96);
            assert(pack.file_url_TRY(ph).equals("bvziXxqrVfoebwjcuXklqqblbxyeflocenofafwwpgyidspiqcxgcdyadfvorfvzenzxhNyvmscxycqjqqvPxneylkekbRiQ"));
            assert(pack.camera_id_GET() == (char)121);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.image_index_SET(208387856) ;
        p263.capture_result_SET((byte)38) ;
        p263.q_SET(new float[] {1.8010478E38F, 2.6314648E38F, 4.221267E37F, -3.3765506E38F}, 0) ;
        p263.alt_SET(-219192694) ;
        p263.time_boot_ms_SET(805681312L) ;
        p263.file_url_SET("bvziXxqrVfoebwjcuXklqqblbxyeflocenofafwwpgyidspiqcxgcdyadfvorfvzenzxhNyvmscxycqjqqvPxneylkekbRiQ", PH) ;
        p263.relative_alt_SET(1239537282) ;
        p263.camera_id_SET((char)121) ;
        p263.time_utc_SET(7219116875422947357L) ;
        p263.lon_SET(-1212691453) ;
        p263.lat_SET(784054054) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1464365025L);
            assert(pack.takeoff_time_utc_GET() == 6311213147581970359L);
            assert(pack.flight_uuid_GET() == 8334704312222980171L);
            assert(pack.arming_time_utc_GET() == 968966016945253701L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.flight_uuid_SET(8334704312222980171L) ;
        p264.time_boot_ms_SET(1464365025L) ;
        p264.arming_time_utc_SET(968966016945253701L) ;
        p264.takeoff_time_utc_SET(6311213147581970359L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -1.4338164E38F);
            assert(pack.yaw_GET() == 5.5405783E37F);
            assert(pack.roll_GET() == -2.6138377E38F);
            assert(pack.time_boot_ms_GET() == 2241457654L);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.yaw_SET(5.5405783E37F) ;
        p265.time_boot_ms_SET(2241457654L) ;
        p265.roll_SET(-2.6138377E38F) ;
        p265.pitch_SET(-1.4338164E38F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)255, (char)44, (char)117, (char)65, (char)78, (char)231, (char)196, (char)134, (char)165, (char)67, (char)134, (char)33, (char)76, (char)7, (char)214, (char)10, (char)25, (char)150, (char)153, (char)23, (char)43, (char)229, (char)119, (char)224, (char)6, (char)14, (char)227, (char)120, (char)70, (char)83, (char)103, (char)127, (char)152, (char)208, (char)213, (char)72, (char)205, (char)109, (char)22, (char)66, (char)237, (char)113, (char)164, (char)185, (char)202, (char)78, (char)23, (char)30, (char)109, (char)188, (char)113, (char)42, (char)146, (char)198, (char)184, (char)151, (char)252, (char)223, (char)170, (char)210, (char)137, (char)237, (char)39, (char)189, (char)194, (char)9, (char)159, (char)110, (char)206, (char)39, (char)120, (char)82, (char)185, (char)112, (char)29, (char)125, (char)10, (char)212, (char)167, (char)10, (char)12, (char)136, (char)27, (char)226, (char)245, (char)30, (char)3, (char)162, (char)85, (char)119, (char)81, (char)1, (char)144, (char)64, (char)238, (char)136, (char)254, (char)220, (char)180, (char)53, (char)21, (char)16, (char)214, (char)182, (char)87, (char)38, (char)51, (char)214, (char)89, (char)1, (char)161, (char)107, (char)176, (char)232, (char)150, (char)89, (char)53, (char)141, (char)206, (char)113, (char)24, (char)101, (char)243, (char)207, (char)215, (char)56, (char)73, (char)82, (char)142, (char)145, (char)66, (char)27, (char)136, (char)193, (char)20, (char)43, (char)200, (char)110, (char)158, (char)6, (char)206, (char)223, (char)125, (char)122, (char)153, (char)83, (char)194, (char)63, (char)182, (char)254, (char)14, (char)206, (char)68, (char)170, (char)205, (char)140, (char)179, (char)248, (char)66, (char)240, (char)145, (char)112, (char)236, (char)188, (char)154, (char)50, (char)7, (char)79, (char)142, (char)206, (char)216, (char)129, (char)171, (char)33, (char)143, (char)168, (char)30, (char)22, (char)130, (char)128, (char)7, (char)118, (char)196, (char)121, (char)85, (char)149, (char)133, (char)155, (char)232, (char)148, (char)241, (char)252, (char)193, (char)209, (char)18, (char)104, (char)185, (char)102, (char)219, (char)205, (char)152, (char)36, (char)88, (char)145, (char)31, (char)208, (char)166, (char)223, (char)11, (char)150, (char)139, (char)8, (char)25, (char)243, (char)102, (char)247, (char)79, (char)209, (char)16, (char)135, (char)164, (char)6, (char)253, (char)10, (char)147, (char)83, (char)249, (char)43, (char)110, (char)30, (char)165, (char)197, (char)86, (char)150, (char)25, (char)237, (char)80, (char)149, (char)124, (char)40, (char)4, (char)213, (char)30, (char)103, (char)22, (char)42, (char)132, (char)22, (char)146}));
            assert(pack.length_GET() == (char)136);
            assert(pack.target_component_GET() == (char)116);
            assert(pack.first_message_offset_GET() == (char)89);
            assert(pack.sequence_GET() == (char)16409);
            assert(pack.target_system_GET() == (char)1);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.first_message_offset_SET((char)89) ;
        p266.sequence_SET((char)16409) ;
        p266.target_component_SET((char)116) ;
        p266.target_system_SET((char)1) ;
        p266.length_SET((char)136) ;
        p266.data__SET(new char[] {(char)255, (char)44, (char)117, (char)65, (char)78, (char)231, (char)196, (char)134, (char)165, (char)67, (char)134, (char)33, (char)76, (char)7, (char)214, (char)10, (char)25, (char)150, (char)153, (char)23, (char)43, (char)229, (char)119, (char)224, (char)6, (char)14, (char)227, (char)120, (char)70, (char)83, (char)103, (char)127, (char)152, (char)208, (char)213, (char)72, (char)205, (char)109, (char)22, (char)66, (char)237, (char)113, (char)164, (char)185, (char)202, (char)78, (char)23, (char)30, (char)109, (char)188, (char)113, (char)42, (char)146, (char)198, (char)184, (char)151, (char)252, (char)223, (char)170, (char)210, (char)137, (char)237, (char)39, (char)189, (char)194, (char)9, (char)159, (char)110, (char)206, (char)39, (char)120, (char)82, (char)185, (char)112, (char)29, (char)125, (char)10, (char)212, (char)167, (char)10, (char)12, (char)136, (char)27, (char)226, (char)245, (char)30, (char)3, (char)162, (char)85, (char)119, (char)81, (char)1, (char)144, (char)64, (char)238, (char)136, (char)254, (char)220, (char)180, (char)53, (char)21, (char)16, (char)214, (char)182, (char)87, (char)38, (char)51, (char)214, (char)89, (char)1, (char)161, (char)107, (char)176, (char)232, (char)150, (char)89, (char)53, (char)141, (char)206, (char)113, (char)24, (char)101, (char)243, (char)207, (char)215, (char)56, (char)73, (char)82, (char)142, (char)145, (char)66, (char)27, (char)136, (char)193, (char)20, (char)43, (char)200, (char)110, (char)158, (char)6, (char)206, (char)223, (char)125, (char)122, (char)153, (char)83, (char)194, (char)63, (char)182, (char)254, (char)14, (char)206, (char)68, (char)170, (char)205, (char)140, (char)179, (char)248, (char)66, (char)240, (char)145, (char)112, (char)236, (char)188, (char)154, (char)50, (char)7, (char)79, (char)142, (char)206, (char)216, (char)129, (char)171, (char)33, (char)143, (char)168, (char)30, (char)22, (char)130, (char)128, (char)7, (char)118, (char)196, (char)121, (char)85, (char)149, (char)133, (char)155, (char)232, (char)148, (char)241, (char)252, (char)193, (char)209, (char)18, (char)104, (char)185, (char)102, (char)219, (char)205, (char)152, (char)36, (char)88, (char)145, (char)31, (char)208, (char)166, (char)223, (char)11, (char)150, (char)139, (char)8, (char)25, (char)243, (char)102, (char)247, (char)79, (char)209, (char)16, (char)135, (char)164, (char)6, (char)253, (char)10, (char)147, (char)83, (char)249, (char)43, (char)110, (char)30, (char)165, (char)197, (char)86, (char)150, (char)25, (char)237, (char)80, (char)149, (char)124, (char)40, (char)4, (char)213, (char)30, (char)103, (char)22, (char)42, (char)132, (char)22, (char)146}, 0) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)4);
            assert(pack.target_system_GET() == (char)36);
            assert(pack.first_message_offset_GET() == (char)175);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)104, (char)178, (char)26, (char)36, (char)78, (char)153, (char)59, (char)201, (char)17, (char)186, (char)69, (char)177, (char)92, (char)246, (char)224, (char)164, (char)70, (char)4, (char)138, (char)55, (char)135, (char)48, (char)78, (char)45, (char)215, (char)122, (char)3, (char)89, (char)164, (char)220, (char)130, (char)244, (char)20, (char)101, (char)146, (char)250, (char)86, (char)40, (char)91, (char)234, (char)238, (char)153, (char)252, (char)109, (char)21, (char)51, (char)111, (char)124, (char)8, (char)102, (char)85, (char)217, (char)204, (char)100, (char)75, (char)195, (char)200, (char)56, (char)167, (char)145, (char)179, (char)141, (char)223, (char)146, (char)134, (char)241, (char)218, (char)64, (char)19, (char)56, (char)124, (char)240, (char)84, (char)129, (char)175, (char)24, (char)248, (char)98, (char)34, (char)59, (char)64, (char)39, (char)205, (char)93, (char)53, (char)174, (char)97, (char)235, (char)67, (char)221, (char)168, (char)196, (char)223, (char)130, (char)153, (char)20, (char)245, (char)137, (char)197, (char)140, (char)180, (char)170, (char)81, (char)132, (char)85, (char)150, (char)83, (char)23, (char)196, (char)196, (char)60, (char)202, (char)156, (char)46, (char)180, (char)38, (char)89, (char)118, (char)7, (char)199, (char)17, (char)247, (char)193, (char)186, (char)113, (char)70, (char)60, (char)129, (char)145, (char)6, (char)130, (char)110, (char)91, (char)10, (char)219, (char)66, (char)137, (char)151, (char)135, (char)47, (char)18, (char)76, (char)85, (char)187, (char)154, (char)15, (char)147, (char)25, (char)172, (char)224, (char)74, (char)62, (char)113, (char)78, (char)252, (char)252, (char)8, (char)75, (char)145, (char)120, (char)221, (char)98, (char)222, (char)190, (char)205, (char)210, (char)4, (char)138, (char)201, (char)235, (char)199, (char)196, (char)22, (char)54, (char)233, (char)120, (char)145, (char)179, (char)216, (char)51, (char)171, (char)34, (char)26, (char)53, (char)100, (char)255, (char)23, (char)197, (char)146, (char)211, (char)194, (char)88, (char)67, (char)121, (char)231, (char)207, (char)240, (char)17, (char)21, (char)42, (char)15, (char)138, (char)228, (char)38, (char)205, (char)64, (char)12, (char)117, (char)171, (char)92, (char)69, (char)100, (char)183, (char)165, (char)165, (char)102, (char)35, (char)202, (char)41, (char)102, (char)161, (char)54, (char)254, (char)239, (char)209, (char)9, (char)130, (char)63, (char)16, (char)75, (char)130, (char)197, (char)90, (char)51, (char)129, (char)154, (char)222, (char)107, (char)20, (char)210, (char)121, (char)243, (char)161, (char)154, (char)210, (char)76, (char)68, (char)250, (char)154}));
            assert(pack.length_GET() == (char)149);
            assert(pack.sequence_GET() == (char)2322);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.sequence_SET((char)2322) ;
        p267.target_system_SET((char)36) ;
        p267.data__SET(new char[] {(char)104, (char)178, (char)26, (char)36, (char)78, (char)153, (char)59, (char)201, (char)17, (char)186, (char)69, (char)177, (char)92, (char)246, (char)224, (char)164, (char)70, (char)4, (char)138, (char)55, (char)135, (char)48, (char)78, (char)45, (char)215, (char)122, (char)3, (char)89, (char)164, (char)220, (char)130, (char)244, (char)20, (char)101, (char)146, (char)250, (char)86, (char)40, (char)91, (char)234, (char)238, (char)153, (char)252, (char)109, (char)21, (char)51, (char)111, (char)124, (char)8, (char)102, (char)85, (char)217, (char)204, (char)100, (char)75, (char)195, (char)200, (char)56, (char)167, (char)145, (char)179, (char)141, (char)223, (char)146, (char)134, (char)241, (char)218, (char)64, (char)19, (char)56, (char)124, (char)240, (char)84, (char)129, (char)175, (char)24, (char)248, (char)98, (char)34, (char)59, (char)64, (char)39, (char)205, (char)93, (char)53, (char)174, (char)97, (char)235, (char)67, (char)221, (char)168, (char)196, (char)223, (char)130, (char)153, (char)20, (char)245, (char)137, (char)197, (char)140, (char)180, (char)170, (char)81, (char)132, (char)85, (char)150, (char)83, (char)23, (char)196, (char)196, (char)60, (char)202, (char)156, (char)46, (char)180, (char)38, (char)89, (char)118, (char)7, (char)199, (char)17, (char)247, (char)193, (char)186, (char)113, (char)70, (char)60, (char)129, (char)145, (char)6, (char)130, (char)110, (char)91, (char)10, (char)219, (char)66, (char)137, (char)151, (char)135, (char)47, (char)18, (char)76, (char)85, (char)187, (char)154, (char)15, (char)147, (char)25, (char)172, (char)224, (char)74, (char)62, (char)113, (char)78, (char)252, (char)252, (char)8, (char)75, (char)145, (char)120, (char)221, (char)98, (char)222, (char)190, (char)205, (char)210, (char)4, (char)138, (char)201, (char)235, (char)199, (char)196, (char)22, (char)54, (char)233, (char)120, (char)145, (char)179, (char)216, (char)51, (char)171, (char)34, (char)26, (char)53, (char)100, (char)255, (char)23, (char)197, (char)146, (char)211, (char)194, (char)88, (char)67, (char)121, (char)231, (char)207, (char)240, (char)17, (char)21, (char)42, (char)15, (char)138, (char)228, (char)38, (char)205, (char)64, (char)12, (char)117, (char)171, (char)92, (char)69, (char)100, (char)183, (char)165, (char)165, (char)102, (char)35, (char)202, (char)41, (char)102, (char)161, (char)54, (char)254, (char)239, (char)209, (char)9, (char)130, (char)63, (char)16, (char)75, (char)130, (char)197, (char)90, (char)51, (char)129, (char)154, (char)222, (char)107, (char)20, (char)210, (char)121, (char)243, (char)161, (char)154, (char)210, (char)76, (char)68, (char)250, (char)154}, 0) ;
        p267.target_component_SET((char)4) ;
        p267.first_message_offset_SET((char)175) ;
        p267.length_SET((char)149) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)81);
            assert(pack.sequence_GET() == (char)18122);
            assert(pack.target_component_GET() == (char)222);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_component_SET((char)222) ;
        p268.target_system_SET((char)81) ;
        p268.sequence_SET((char)18122) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.uri_LEN(ph) == 71);
            assert(pack.uri_TRY(ph).equals("ujzIsxbqxrpmwhmwzsborxqyozgnxbtmiiHofhlkrbcWingyDrpmdpzcholUbdobWpcdgem"));
            assert(pack.bitrate_GET() == 3675567998L);
            assert(pack.resolution_h_GET() == (char)10610);
            assert(pack.framerate_GET() == 1.0595254E38F);
            assert(pack.camera_id_GET() == (char)81);
            assert(pack.status_GET() == (char)105);
            assert(pack.resolution_v_GET() == (char)5904);
            assert(pack.rotation_GET() == (char)46736);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.status_SET((char)105) ;
        p269.uri_SET("ujzIsxbqxrpmwhmwzsborxqyozgnxbtmiiHofhlkrbcWingyDrpmdpzcholUbdobWpcdgem", PH) ;
        p269.resolution_h_SET((char)10610) ;
        p269.rotation_SET((char)46736) ;
        p269.resolution_v_SET((char)5904) ;
        p269.bitrate_SET(3675567998L) ;
        p269.camera_id_SET((char)81) ;
        p269.framerate_SET(1.0595254E38F) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.camera_id_GET() == (char)152);
            assert(pack.target_system_GET() == (char)26);
            assert(pack.framerate_GET() == 3.0342045E38F);
            assert(pack.bitrate_GET() == 1680985526L);
            assert(pack.resolution_v_GET() == (char)25255);
            assert(pack.resolution_h_GET() == (char)7338);
            assert(pack.rotation_GET() == (char)61728);
            assert(pack.target_component_GET() == (char)240);
            assert(pack.uri_LEN(ph) == 140);
            assert(pack.uri_TRY(ph).equals("liujqyumfynmdoirlokhbufizukevbvibtfvaoarocunzejkpGhQuaeaefoeadFvnurlzJkweginurocifotohLpklnwttuxxMbnypliIggtrrqqcgpybcuuxuicxFzrkuoplrOerdqI"));
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.camera_id_SET((char)152) ;
        p270.resolution_v_SET((char)25255) ;
        p270.uri_SET("liujqyumfynmdoirlokhbufizukevbvibtfvaoarocunzejkpGhQuaeaefoeadFvnurlzJkweginurocifotohLpklnwttuxxMbnypliIggtrrqqcgpybcuuxuicxFzrkuoplrOerdqI", PH) ;
        p270.framerate_SET(3.0342045E38F) ;
        p270.target_system_SET((char)26) ;
        p270.bitrate_SET(1680985526L) ;
        p270.target_component_SET((char)240) ;
        p270.rotation_SET((char)61728) ;
        p270.resolution_h_SET((char)7338) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 7);
            assert(pack.password_TRY(ph).equals("ijuuljg"));
            assert(pack.ssid_LEN(ph) == 10);
            assert(pack.ssid_TRY(ph).equals("eiequXsyan"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("eiequXsyan", PH) ;
        p299.password_SET("ijuuljg", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.max_version_GET() == (char)49838);
            assert(pack.min_version_GET() == (char)5078);
            assert(pack.version_GET() == (char)56933);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)148, (char)234, (char)158, (char)224, (char)39, (char)115, (char)11, (char)58}));
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)186, (char)43, (char)187, (char)82, (char)159, (char)219, (char)237, (char)144}));
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)56933) ;
        p300.min_version_SET((char)5078) ;
        p300.library_version_hash_SET(new char[] {(char)186, (char)43, (char)187, (char)82, (char)159, (char)219, (char)237, (char)144}, 0) ;
        p300.max_version_SET((char)49838) ;
        p300.spec_version_hash_SET(new char[] {(char)148, (char)234, (char)158, (char)224, (char)39, (char)115, (char)11, (char)58}, 0) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
            assert(pack.uptime_sec_GET() == 1935334974L);
            assert(pack.vendor_specific_status_code_GET() == (char)21937);
            assert(pack.time_usec_GET() == 6316667578067306576L);
            assert(pack.sub_mode_GET() == (char)88);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.uptime_sec_SET(1935334974L) ;
        p310.time_usec_SET(6316667578067306576L) ;
        p310.vendor_specific_status_code_SET((char)21937) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION) ;
        p310.sub_mode_SET((char)88) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 7);
            assert(pack.name_TRY(ph).equals("fSaphzx"));
            assert(pack.hw_version_minor_GET() == (char)137);
            assert(pack.sw_version_major_GET() == (char)243);
            assert(pack.sw_vcs_commit_GET() == 1012782515L);
            assert(pack.sw_version_minor_GET() == (char)189);
            assert(pack.hw_version_major_GET() == (char)109);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)73, (char)200, (char)94, (char)203, (char)215, (char)244, (char)112, (char)137, (char)232, (char)251, (char)126, (char)41, (char)49, (char)2, (char)137, (char)178}));
            assert(pack.uptime_sec_GET() == 2396192772L);
            assert(pack.time_usec_GET() == 3339905493798382282L);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.uptime_sec_SET(2396192772L) ;
        p311.sw_vcs_commit_SET(1012782515L) ;
        p311.hw_unique_id_SET(new char[] {(char)73, (char)200, (char)94, (char)203, (char)215, (char)244, (char)112, (char)137, (char)232, (char)251, (char)126, (char)41, (char)49, (char)2, (char)137, (char)178}, 0) ;
        p311.sw_version_minor_SET((char)189) ;
        p311.name_SET("fSaphzx", PH) ;
        p311.hw_version_minor_SET((char)137) ;
        p311.sw_version_major_SET((char)243) ;
        p311.time_usec_SET(3339905493798382282L) ;
        p311.hw_version_major_SET((char)109) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)109);
            assert(pack.param_index_GET() == (short)5269);
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("pjktxduaiy"));
            assert(pack.target_system_GET() == (char)157);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_index_SET((short)5269) ;
        p320.target_component_SET((char)109) ;
        p320.param_id_SET("pjktxduaiy", PH) ;
        p320.target_system_SET((char)157) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)79);
            assert(pack.target_component_GET() == (char)135);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)79) ;
        p321.target_component_SET((char)135) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)29169);
            assert(pack.param_value_LEN(ph) == 37);
            assert(pack.param_value_TRY(ph).equals("ThgsuwffkajwqnvtuzkbyqdrynRgiamazvaqx"));
            assert(pack.param_index_GET() == (char)61864);
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("cliozaeYilb"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_count_SET((char)29169) ;
        p322.param_value_SET("ThgsuwffkajwqnvtuzkbyqdrynRgiamazvaqx", PH) ;
        p322.param_index_SET((char)61864) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64) ;
        p322.param_id_SET("cliozaeYilb", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("YhikywZuhz"));
            assert(pack.target_system_GET() == (char)54);
            assert(pack.target_component_GET() == (char)55);
            assert(pack.param_value_LEN(ph) == 17);
            assert(pack.param_value_TRY(ph).equals("hPvjmqxbhxddrcmux"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_component_SET((char)55) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64) ;
        p323.target_system_SET((char)54) ;
        p323.param_value_SET("hPvjmqxbhxddrcmux", PH) ;
        p323.param_id_SET("YhikywZuhz", PH) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("kPzcses"));
            assert(pack.param_value_LEN(ph) == 26);
            assert(pack.param_value_TRY(ph).equals("cruhgzXurGacnzllhqVzynpxon"));
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_ACCEPTED);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("kPzcses", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_ACCEPTED) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16) ;
        p324.param_value_SET("cruhgzXurGacnzllhqVzynpxon", PH) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.min_distance_GET() == (char)46841);
            assert(pack.time_usec_GET() == 4425208447324225967L);
            assert(pack.increment_GET() == (char)44);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            assert(pack.max_distance_GET() == (char)62454);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)44990, (char)5051, (char)34637, (char)35385, (char)41572, (char)9857, (char)34353, (char)5499, (char)10260, (char)16279, (char)51130, (char)43555, (char)42583, (char)18763, (char)17386, (char)15252, (char)1143, (char)48214, (char)35384, (char)53586, (char)21189, (char)25306, (char)56723, (char)21171, (char)17320, (char)36854, (char)44738, (char)63689, (char)10468, (char)10554, (char)13412, (char)40219, (char)60136, (char)867, (char)46734, (char)31125, (char)38252, (char)62484, (char)42224, (char)3474, (char)46555, (char)64374, (char)59626, (char)22856, (char)23311, (char)11088, (char)45743, (char)6813, (char)4931, (char)22281, (char)11607, (char)9380, (char)24282, (char)6424, (char)13342, (char)10751, (char)57885, (char)49285, (char)44916, (char)52727, (char)20031, (char)8894, (char)41101, (char)26079, (char)14855, (char)1824, (char)23074, (char)33256, (char)27129, (char)1269, (char)60393, (char)56737}));
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND) ;
        p330.min_distance_SET((char)46841) ;
        p330.time_usec_SET(4425208447324225967L) ;
        p330.max_distance_SET((char)62454) ;
        p330.distances_SET(new char[] {(char)44990, (char)5051, (char)34637, (char)35385, (char)41572, (char)9857, (char)34353, (char)5499, (char)10260, (char)16279, (char)51130, (char)43555, (char)42583, (char)18763, (char)17386, (char)15252, (char)1143, (char)48214, (char)35384, (char)53586, (char)21189, (char)25306, (char)56723, (char)21171, (char)17320, (char)36854, (char)44738, (char)63689, (char)10468, (char)10554, (char)13412, (char)40219, (char)60136, (char)867, (char)46734, (char)31125, (char)38252, (char)62484, (char)42224, (char)3474, (char)46555, (char)64374, (char)59626, (char)22856, (char)23311, (char)11088, (char)45743, (char)6813, (char)4931, (char)22281, (char)11607, (char)9380, (char)24282, (char)6424, (char)13342, (char)10751, (char)57885, (char)49285, (char)44916, (char)52727, (char)20031, (char)8894, (char)41101, (char)26079, (char)14855, (char)1824, (char)23074, (char)33256, (char)27129, (char)1269, (char)60393, (char)56737}, 0) ;
        p330.increment_SET((char)44) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
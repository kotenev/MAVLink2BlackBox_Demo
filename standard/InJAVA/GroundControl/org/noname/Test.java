
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
            long id = id__t(src);
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
            long id = id__e(src);
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
            long id = id__e(src);
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
            assert(pack.custom_mode_GET() == 3267842051L);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_POWEROFF);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_OPENPILOT);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_KITE);
            assert(pack.mavlink_version_GET() == (char)255);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED));
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.mavlink_version_SET((char)255) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_POWEROFF) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_OPENPILOT) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_KITE) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED)) ;
        p0.custom_mode_SET(3267842051L) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.current_battery_GET() == (short)26519);
            assert(pack.errors_count3_GET() == (char)27055);
            assert(pack.errors_count1_GET() == (char)13);
            assert(pack.voltage_battery_GET() == (char)27991);
            assert(pack.errors_count4_GET() == (char)25441);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS));
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION));
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.drop_rate_comm_GET() == (char)22779);
            assert(pack.load_GET() == (char)21873);
            assert(pack.errors_comm_GET() == (char)54572);
            assert(pack.battery_remaining_GET() == (byte) - 101);
            assert(pack.errors_count2_GET() == (char)60800);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.load_SET((char)21873) ;
        p1.drop_rate_comm_SET((char)22779) ;
        p1.errors_count2_SET((char)60800) ;
        p1.current_battery_SET((short)26519) ;
        p1.voltage_battery_SET((char)27991) ;
        p1.errors_count4_SET((char)25441) ;
        p1.errors_count1_SET((char)13) ;
        p1.battery_remaining_SET((byte) - 101) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION)) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS)) ;
        p1.errors_count3_SET((char)27055) ;
        p1.errors_comm_SET((char)54572) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 426196687028036482L);
            assert(pack.time_boot_ms_GET() == 2056122459L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(426196687028036482L) ;
        p2.time_boot_ms_SET(2056122459L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -1.3661649E38F);
            assert(pack.time_boot_ms_GET() == 115958414L);
            assert(pack.afy_GET() == -8.7713E37F);
            assert(pack.vz_GET() == 1.683386E38F);
            assert(pack.yaw_rate_GET() == -2.9183497E38F);
            assert(pack.type_mask_GET() == (char)10749);
            assert(pack.vx_GET() == 3.3790608E37F);
            assert(pack.afx_GET() == -2.8833549E38F);
            assert(pack.yaw_GET() == -1.2442795E38F);
            assert(pack.x_GET() == -1.990697E38F);
            assert(pack.afz_GET() == 2.9364589E38F);
            assert(pack.vy_GET() == -1.4962619E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.z_GET() == 2.9136235E38F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.time_boot_ms_SET(115958414L) ;
        p3.yaw_rate_SET(-2.9183497E38F) ;
        p3.z_SET(2.9136235E38F) ;
        p3.afx_SET(-2.8833549E38F) ;
        p3.y_SET(-1.3661649E38F) ;
        p3.yaw_SET(-1.2442795E38F) ;
        p3.vz_SET(1.683386E38F) ;
        p3.afz_SET(2.9364589E38F) ;
        p3.x_SET(-1.990697E38F) ;
        p3.vy_SET(-1.4962619E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p3.type_mask_SET((char)10749) ;
        p3.afy_SET(-8.7713E37F) ;
        p3.vx_SET(3.3790608E37F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 4146864070L);
            assert(pack.target_system_GET() == (char)182);
            assert(pack.time_usec_GET() == 5218215604687089337L);
            assert(pack.target_component_GET() == (char)6);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.target_component_SET((char)6) ;
        p4.target_system_SET((char)182) ;
        p4.time_usec_SET(5218215604687089337L) ;
        p4.seq_SET(4146864070L) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)183);
            assert(pack.target_system_GET() == (char)54);
            assert(pack.version_GET() == (char)246);
            assert(pack.passkey_LEN(ph) == 16);
            assert(pack.passkey_TRY(ph).equals("gxiovzdQjCchjjnb"));
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.passkey_SET("gxiovzdQjCchjjnb", PH) ;
        p5.target_system_SET((char)54) ;
        p5.control_request_SET((char)183) ;
        p5.version_SET((char)246) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.ack_GET() == (char)107);
            assert(pack.gcs_system_id_GET() == (char)111);
            assert(pack.control_request_GET() == (char)160);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.control_request_SET((char)160) ;
        p6.gcs_system_id_SET((char)111) ;
        p6.ack_SET((char)107) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 7);
            assert(pack.key_TRY(ph).equals("lqwgjjD"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("lqwgjjD", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)24);
            assert(pack.custom_mode_GET() == 1827024858L);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_STABILIZE_DISARMED) ;
        p11.custom_mode_SET(1827024858L) ;
        p11.target_system_SET((char)24) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("tOxBmtfedoDZ"));
            assert(pack.param_index_GET() == (short) -28281);
            assert(pack.target_system_GET() == (char)100);
            assert(pack.target_component_GET() == (char)153);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_component_SET((char)153) ;
        p20.target_system_SET((char)100) ;
        p20.param_id_SET("tOxBmtfedoDZ", PH) ;
        p20.param_index_SET((short) -28281) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)48);
            assert(pack.target_component_GET() == (char)231);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)231) ;
        p21.target_system_SET((char)48) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)64101);
            assert(pack.param_value_GET() == -7.217219E37F);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("papbyfyl"));
            assert(pack.param_index_GET() == (char)5269);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_id_SET("papbyfyl", PH) ;
        p22.param_index_SET((char)5269) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32) ;
        p22.param_value_SET(-7.217219E37F) ;
        p22.param_count_SET((char)64101) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("Vajjh"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
            assert(pack.param_value_GET() == -3.1670163E38F);
            assert(pack.target_system_GET() == (char)169);
            assert(pack.target_component_GET() == (char)219);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.target_system_SET((char)169) ;
        p23.param_value_SET(-3.1670163E38F) ;
        p23.param_id_SET("Vajjh", PH) ;
        p23.target_component_SET((char)219) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.vel_GET() == (char)26998);
            assert(pack.lon_GET() == -1870038321);
            assert(pack.lat_GET() == 1428614817);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
            assert(pack.alt_ellipsoid_TRY(ph) == -1382472388);
            assert(pack.time_usec_GET() == 540522861222654713L);
            assert(pack.h_acc_TRY(ph) == 1572968620L);
            assert(pack.epv_GET() == (char)21140);
            assert(pack.alt_GET() == 2044036493);
            assert(pack.hdg_acc_TRY(ph) == 2190672543L);
            assert(pack.cog_GET() == (char)57293);
            assert(pack.satellites_visible_GET() == (char)139);
            assert(pack.eph_GET() == (char)12717);
            assert(pack.v_acc_TRY(ph) == 1902865318L);
            assert(pack.vel_acc_TRY(ph) == 2065495744L);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.hdg_acc_SET(2190672543L, PH) ;
        p24.alt_ellipsoid_SET(-1382472388, PH) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP) ;
        p24.lat_SET(1428614817) ;
        p24.epv_SET((char)21140) ;
        p24.vel_SET((char)26998) ;
        p24.alt_SET(2044036493) ;
        p24.h_acc_SET(1572968620L, PH) ;
        p24.eph_SET((char)12717) ;
        p24.v_acc_SET(1902865318L, PH) ;
        p24.cog_SET((char)57293) ;
        p24.time_usec_SET(540522861222654713L) ;
        p24.satellites_visible_SET((char)139) ;
        p24.lon_SET(-1870038321) ;
        p24.vel_acc_SET(2065495744L, PH) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)213, (char)34, (char)72, (char)114, (char)13, (char)146, (char)64, (char)228, (char)201, (char)180, (char)118, (char)174, (char)130, (char)100, (char)157, (char)156, (char)40, (char)184, (char)250, (char)91}));
            assert(pack.satellites_visible_GET() == (char)220);
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)96, (char)82, (char)77, (char)147, (char)137, (char)129, (char)212, (char)219, (char)45, (char)119, (char)190, (char)227, (char)178, (char)11, (char)87, (char)109, (char)148, (char)212, (char)153, (char)51}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)93, (char)122, (char)48, (char)83, (char)10, (char)201, (char)24, (char)131, (char)181, (char)74, (char)105, (char)194, (char)76, (char)171, (char)168, (char)96, (char)98, (char)30, (char)222, (char)101}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)30, (char)199, (char)23, (char)134, (char)65, (char)80, (char)100, (char)122, (char)252, (char)132, (char)215, (char)56, (char)62, (char)117, (char)81, (char)126, (char)173, (char)35, (char)100, (char)184}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)128, (char)208, (char)74, (char)75, (char)2, (char)19, (char)28, (char)226, (char)155, (char)49, (char)137, (char)131, (char)8, (char)57, (char)70, (char)199, (char)22, (char)231, (char)184, (char)84}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)220) ;
        p25.satellite_snr_SET(new char[] {(char)213, (char)34, (char)72, (char)114, (char)13, (char)146, (char)64, (char)228, (char)201, (char)180, (char)118, (char)174, (char)130, (char)100, (char)157, (char)156, (char)40, (char)184, (char)250, (char)91}, 0) ;
        p25.satellite_used_SET(new char[] {(char)30, (char)199, (char)23, (char)134, (char)65, (char)80, (char)100, (char)122, (char)252, (char)132, (char)215, (char)56, (char)62, (char)117, (char)81, (char)126, (char)173, (char)35, (char)100, (char)184}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)96, (char)82, (char)77, (char)147, (char)137, (char)129, (char)212, (char)219, (char)45, (char)119, (char)190, (char)227, (char)178, (char)11, (char)87, (char)109, (char)148, (char)212, (char)153, (char)51}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)128, (char)208, (char)74, (char)75, (char)2, (char)19, (char)28, (char)226, (char)155, (char)49, (char)137, (char)131, (char)8, (char)57, (char)70, (char)199, (char)22, (char)231, (char)184, (char)84}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)93, (char)122, (char)48, (char)83, (char)10, (char)201, (char)24, (char)131, (char)181, (char)74, (char)105, (char)194, (char)76, (char)171, (char)168, (char)96, (char)98, (char)30, (char)222, (char)101}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short)1944);
            assert(pack.yacc_GET() == (short)18147);
            assert(pack.xgyro_GET() == (short) -2059);
            assert(pack.ymag_GET() == (short)20596);
            assert(pack.zmag_GET() == (short)9498);
            assert(pack.zacc_GET() == (short) -4757);
            assert(pack.ygyro_GET() == (short)27187);
            assert(pack.xmag_GET() == (short) -8622);
            assert(pack.time_boot_ms_GET() == 1171188976L);
            assert(pack.xacc_GET() == (short) -29407);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.zgyro_SET((short)1944) ;
        p26.yacc_SET((short)18147) ;
        p26.xacc_SET((short) -29407) ;
        p26.ygyro_SET((short)27187) ;
        p26.ymag_SET((short)20596) ;
        p26.time_boot_ms_SET(1171188976L) ;
        p26.xmag_SET((short) -8622) ;
        p26.zacc_SET((short) -4757) ;
        p26.zmag_SET((short)9498) ;
        p26.xgyro_SET((short) -2059) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short)20554);
            assert(pack.ymag_GET() == (short)17730);
            assert(pack.zmag_GET() == (short) -29165);
            assert(pack.ygyro_GET() == (short) -8662);
            assert(pack.xgyro_GET() == (short) -16945);
            assert(pack.yacc_GET() == (short)7675);
            assert(pack.xacc_GET() == (short)1335);
            assert(pack.zgyro_GET() == (short)2611);
            assert(pack.xmag_GET() == (short)4411);
            assert(pack.time_usec_GET() == 4133234854305198650L);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.xacc_SET((short)1335) ;
        p27.xgyro_SET((short) -16945) ;
        p27.ygyro_SET((short) -8662) ;
        p27.zmag_SET((short) -29165) ;
        p27.zgyro_SET((short)2611) ;
        p27.xmag_SET((short)4411) ;
        p27.ymag_SET((short)17730) ;
        p27.time_usec_SET(4133234854305198650L) ;
        p27.zacc_SET((short)20554) ;
        p27.yacc_SET((short)7675) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -13011);
            assert(pack.time_usec_GET() == 1348450655603771401L);
            assert(pack.press_diff1_GET() == (short) -11465);
            assert(pack.press_abs_GET() == (short) -15117);
            assert(pack.press_diff2_GET() == (short)23793);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.temperature_SET((short) -13011) ;
        p28.time_usec_SET(1348450655603771401L) ;
        p28.press_abs_SET((short) -15117) ;
        p28.press_diff1_SET((short) -11465) ;
        p28.press_diff2_SET((short)23793) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -2.970287E38F);
            assert(pack.time_boot_ms_GET() == 3994101361L);
            assert(pack.temperature_GET() == (short)6845);
            assert(pack.press_diff_GET() == -1.7073235E38F);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_abs_SET(-2.970287E38F) ;
        p29.time_boot_ms_SET(3994101361L) ;
        p29.temperature_SET((short)6845) ;
        p29.press_diff_SET(-1.7073235E38F) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2379424066L);
            assert(pack.pitch_GET() == -7.9514707E37F);
            assert(pack.yaw_GET() == 6.329698E37F);
            assert(pack.roll_GET() == 5.203031E37F);
            assert(pack.rollspeed_GET() == -3.0227084E38F);
            assert(pack.yawspeed_GET() == -1.8070459E38F);
            assert(pack.pitchspeed_GET() == 7.0605563E37F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.yaw_SET(6.329698E37F) ;
        p30.time_boot_ms_SET(2379424066L) ;
        p30.roll_SET(5.203031E37F) ;
        p30.yawspeed_SET(-1.8070459E38F) ;
        p30.pitch_SET(-7.9514707E37F) ;
        p30.rollspeed_SET(-3.0227084E38F) ;
        p30.pitchspeed_SET(7.0605563E37F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 2.948832E38F);
            assert(pack.q4_GET() == 2.0962994E38F);
            assert(pack.q3_GET() == 8.90374E37F);
            assert(pack.rollspeed_GET() == 2.6063837E38F);
            assert(pack.pitchspeed_GET() == 1.9218415E38F);
            assert(pack.q2_GET() == -1.249444E38F);
            assert(pack.time_boot_ms_GET() == 1492807594L);
            assert(pack.q1_GET() == -1.714981E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.yawspeed_SET(2.948832E38F) ;
        p31.q4_SET(2.0962994E38F) ;
        p31.q3_SET(8.90374E37F) ;
        p31.q1_SET(-1.714981E38F) ;
        p31.rollspeed_SET(2.6063837E38F) ;
        p31.pitchspeed_SET(1.9218415E38F) ;
        p31.time_boot_ms_SET(1492807594L) ;
        p31.q2_SET(-1.249444E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == -1.6959548E38F);
            assert(pack.z_GET() == 1.8862507E38F);
            assert(pack.vy_GET() == 2.5076979E38F);
            assert(pack.y_GET() == -2.8143953E38F);
            assert(pack.time_boot_ms_GET() == 2769033209L);
            assert(pack.x_GET() == 2.6477236E38F);
            assert(pack.vz_GET() == -3.0265162E38F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.time_boot_ms_SET(2769033209L) ;
        p32.vz_SET(-3.0265162E38F) ;
        p32.vy_SET(2.5076979E38F) ;
        p32.x_SET(2.6477236E38F) ;
        p32.y_SET(-2.8143953E38F) ;
        p32.z_SET(1.8862507E38F) ;
        p32.vx_SET(-1.6959548E38F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == (short) -5823);
            assert(pack.vz_GET() == (short) -17469);
            assert(pack.vy_GET() == (short)31587);
            assert(pack.relative_alt_GET() == -116109902);
            assert(pack.hdg_GET() == (char)39855);
            assert(pack.time_boot_ms_GET() == 957578080L);
            assert(pack.lat_GET() == 510974803);
            assert(pack.alt_GET() == 691696736);
            assert(pack.lon_GET() == 1476671118);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.hdg_SET((char)39855) ;
        p33.vy_SET((short)31587) ;
        p33.lat_SET(510974803) ;
        p33.vx_SET((short) -5823) ;
        p33.lon_SET(1476671118) ;
        p33.relative_alt_SET(-116109902) ;
        p33.vz_SET((short) -17469) ;
        p33.alt_SET(691696736) ;
        p33.time_boot_ms_SET(957578080L) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan5_scaled_GET() == (short) -4093);
            assert(pack.chan3_scaled_GET() == (short) -8952);
            assert(pack.chan2_scaled_GET() == (short)3092);
            assert(pack.chan1_scaled_GET() == (short) -7087);
            assert(pack.rssi_GET() == (char)81);
            assert(pack.time_boot_ms_GET() == 3988980132L);
            assert(pack.chan7_scaled_GET() == (short) -11181);
            assert(pack.chan6_scaled_GET() == (short)12227);
            assert(pack.port_GET() == (char)65);
            assert(pack.chan4_scaled_GET() == (short) -16523);
            assert(pack.chan8_scaled_GET() == (short) -10185);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.time_boot_ms_SET(3988980132L) ;
        p34.chan2_scaled_SET((short)3092) ;
        p34.chan3_scaled_SET((short) -8952) ;
        p34.chan7_scaled_SET((short) -11181) ;
        p34.port_SET((char)65) ;
        p34.rssi_SET((char)81) ;
        p34.chan6_scaled_SET((short)12227) ;
        p34.chan4_scaled_SET((short) -16523) ;
        p34.chan5_scaled_SET((short) -4093) ;
        p34.chan1_scaled_SET((short) -7087) ;
        p34.chan8_scaled_SET((short) -10185) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3016142495L);
            assert(pack.chan7_raw_GET() == (char)6535);
            assert(pack.chan2_raw_GET() == (char)39854);
            assert(pack.chan3_raw_GET() == (char)4015);
            assert(pack.rssi_GET() == (char)152);
            assert(pack.chan5_raw_GET() == (char)928);
            assert(pack.chan1_raw_GET() == (char)62081);
            assert(pack.chan6_raw_GET() == (char)35913);
            assert(pack.chan4_raw_GET() == (char)50475);
            assert(pack.port_GET() == (char)221);
            assert(pack.chan8_raw_GET() == (char)59913);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.time_boot_ms_SET(3016142495L) ;
        p35.chan5_raw_SET((char)928) ;
        p35.rssi_SET((char)152) ;
        p35.chan2_raw_SET((char)39854) ;
        p35.port_SET((char)221) ;
        p35.chan7_raw_SET((char)6535) ;
        p35.chan8_raw_SET((char)59913) ;
        p35.chan3_raw_SET((char)4015) ;
        p35.chan6_raw_SET((char)35913) ;
        p35.chan1_raw_SET((char)62081) ;
        p35.chan4_raw_SET((char)50475) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo2_raw_GET() == (char)9576);
            assert(pack.servo11_raw_TRY(ph) == (char)13619);
            assert(pack.servo3_raw_GET() == (char)34998);
            assert(pack.servo1_raw_GET() == (char)13464);
            assert(pack.servo7_raw_GET() == (char)49787);
            assert(pack.servo14_raw_TRY(ph) == (char)22273);
            assert(pack.servo9_raw_TRY(ph) == (char)8899);
            assert(pack.servo13_raw_TRY(ph) == (char)26854);
            assert(pack.servo8_raw_GET() == (char)48048);
            assert(pack.servo4_raw_GET() == (char)44314);
            assert(pack.servo6_raw_GET() == (char)31158);
            assert(pack.servo5_raw_GET() == (char)31777);
            assert(pack.servo16_raw_TRY(ph) == (char)37061);
            assert(pack.time_usec_GET() == 1678202360L);
            assert(pack.servo10_raw_TRY(ph) == (char)35506);
            assert(pack.servo15_raw_TRY(ph) == (char)31120);
            assert(pack.servo12_raw_TRY(ph) == (char)34481);
            assert(pack.port_GET() == (char)133);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo16_raw_SET((char)37061, PH) ;
        p36.port_SET((char)133) ;
        p36.servo11_raw_SET((char)13619, PH) ;
        p36.servo13_raw_SET((char)26854, PH) ;
        p36.servo9_raw_SET((char)8899, PH) ;
        p36.servo7_raw_SET((char)49787) ;
        p36.servo10_raw_SET((char)35506, PH) ;
        p36.servo14_raw_SET((char)22273, PH) ;
        p36.time_usec_SET(1678202360L) ;
        p36.servo5_raw_SET((char)31777) ;
        p36.servo4_raw_SET((char)44314) ;
        p36.servo6_raw_SET((char)31158) ;
        p36.servo1_raw_SET((char)13464) ;
        p36.servo3_raw_SET((char)34998) ;
        p36.servo12_raw_SET((char)34481, PH) ;
        p36.servo8_raw_SET((char)48048) ;
        p36.servo2_raw_SET((char)9576) ;
        p36.servo15_raw_SET((char)31120, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)133);
            assert(pack.target_system_GET() == (char)242);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.start_index_GET() == (short) -3594);
            assert(pack.end_index_GET() == (short)2861);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p37.end_index_SET((short)2861) ;
        p37.target_component_SET((char)133) ;
        p37.start_index_SET((short) -3594) ;
        p37.target_system_SET((char)242) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)138);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)136);
            assert(pack.start_index_GET() == (short) -19948);
            assert(pack.end_index_GET() == (short)13208);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.start_index_SET((short) -19948) ;
        p38.end_index_SET((short)13208) ;
        p38.target_component_SET((char)138) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p38.target_system_SET((char)136) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.param2_GET() == -2.4993458E38F);
            assert(pack.y_GET() == 1.8524209E38F);
            assert(pack.target_component_GET() == (char)53);
            assert(pack.seq_GET() == (char)53180);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_SERVO);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.z_GET() == 2.079004E38F);
            assert(pack.target_system_GET() == (char)162);
            assert(pack.param3_GET() == -8.250136E37F);
            assert(pack.param4_GET() == 2.5373568E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.x_GET() == 2.0867085E38F);
            assert(pack.autocontinue_GET() == (char)12);
            assert(pack.param1_GET() == 3.0455898E38F);
            assert(pack.current_GET() == (char)44);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p39.command_SET(MAV_CMD.MAV_CMD_DO_SET_SERVO) ;
        p39.param4_SET(2.5373568E38F) ;
        p39.x_SET(2.0867085E38F) ;
        p39.param2_SET(-2.4993458E38F) ;
        p39.param1_SET(3.0455898E38F) ;
        p39.target_component_SET((char)53) ;
        p39.current_SET((char)44) ;
        p39.seq_SET((char)53180) ;
        p39.autocontinue_SET((char)12) ;
        p39.y_SET(1.8524209E38F) ;
        p39.target_system_SET((char)162) ;
        p39.param3_SET(-8.250136E37F) ;
        p39.z_SET(2.079004E38F) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)55269);
            assert(pack.target_system_GET() == (char)157);
            assert(pack.target_component_GET() == (char)238);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_component_SET((char)238) ;
        p40.seq_SET((char)55269) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p40.target_system_SET((char)157) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)112);
            assert(pack.seq_GET() == (char)52718);
            assert(pack.target_system_GET() == (char)58);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_component_SET((char)112) ;
        p41.seq_SET((char)52718) ;
        p41.target_system_SET((char)58) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)18230);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)18230) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)151);
            assert(pack.target_component_GET() == (char)194);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_component_SET((char)194) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p43.target_system_SET((char)151) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)57);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.count_GET() == (char)37600);
            assert(pack.target_system_GET() == (char)183);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p44.count_SET((char)37600) ;
        p44.target_component_SET((char)57) ;
        p44.target_system_SET((char)183) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)216);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)175);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)216) ;
        p45.target_component_SET((char)175) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)63464);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)63464) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)175);
            assert(pack.target_component_GET() == (char)47);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED) ;
        p47.target_system_SET((char)175) ;
        p47.target_component_SET((char)47) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)192);
            assert(pack.time_usec_TRY(ph) == 8034775186411186999L);
            assert(pack.altitude_GET() == 1033163495);
            assert(pack.latitude_GET() == 1361017691);
            assert(pack.longitude_GET() == -824214192);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.latitude_SET(1361017691) ;
        p48.longitude_SET(-824214192) ;
        p48.altitude_SET(1033163495) ;
        p48.time_usec_SET(8034775186411186999L, PH) ;
        p48.target_system_SET((char)192) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 3912380015626650530L);
            assert(pack.latitude_GET() == 958235731);
            assert(pack.altitude_GET() == -740002055);
            assert(pack.longitude_GET() == -428233400);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.time_usec_SET(3912380015626650530L, PH) ;
        p49.longitude_SET(-428233400) ;
        p49.latitude_SET(958235731) ;
        p49.altitude_SET(-740002055) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_value_max_GET() == 3.2719405E38F);
            assert(pack.scale_GET() == 2.3440188E37F);
            assert(pack.param_index_GET() == (short) -22837);
            assert(pack.target_component_GET() == (char)163);
            assert(pack.parameter_rc_channel_index_GET() == (char)91);
            assert(pack.param_value_min_GET() == 1.4237761E38F);
            assert(pack.target_system_GET() == (char)39);
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("zdqdWctxld"));
            assert(pack.param_value0_GET() == -8.293038E37F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.parameter_rc_channel_index_SET((char)91) ;
        p50.scale_SET(2.3440188E37F) ;
        p50.target_system_SET((char)39) ;
        p50.param_index_SET((short) -22837) ;
        p50.param_value_min_SET(1.4237761E38F) ;
        p50.param_value0_SET(-8.293038E37F) ;
        p50.param_value_max_SET(3.2719405E38F) ;
        p50.target_component_SET((char)163) ;
        p50.param_id_SET("zdqdWctxld", PH) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)241);
            assert(pack.seq_GET() == (char)59684);
            assert(pack.target_component_GET() == (char)186);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_system_SET((char)241) ;
        p51.target_component_SET((char)186) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p51.seq_SET((char)59684) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2x_GET() == -1.0094102E38F);
            assert(pack.p2z_GET() == -1.1722445E38F);
            assert(pack.target_system_GET() == (char)11);
            assert(pack.p1x_GET() == 3.0580846E38F);
            assert(pack.p2y_GET() == -5.6015857E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.target_component_GET() == (char)67);
            assert(pack.p1z_GET() == -1.0697036E38F);
            assert(pack.p1y_GET() == 1.509937E38F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p54.target_component_SET((char)67) ;
        p54.p2z_SET(-1.1722445E38F) ;
        p54.p1y_SET(1.509937E38F) ;
        p54.p1x_SET(3.0580846E38F) ;
        p54.p1z_SET(-1.0697036E38F) ;
        p54.target_system_SET((char)11) ;
        p54.p2y_SET(-5.6015857E37F) ;
        p54.p2x_SET(-1.0094102E38F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2z_GET() == 2.435974E37F);
            assert(pack.p1z_GET() == 1.0480711E38F);
            assert(pack.p2x_GET() == 3.0736069E38F);
            assert(pack.p1y_GET() == -5.450612E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.p2y_GET() == -1.8920744E38F);
            assert(pack.p1x_GET() == 6.0008663E37F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2y_SET(-1.8920744E38F) ;
        p55.p1z_SET(1.0480711E38F) ;
        p55.p2z_SET(2.435974E37F) ;
        p55.p1y_SET(-5.450612E37F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p55.p1x_SET(6.0008663E37F) ;
        p55.p2x_SET(3.0736069E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == -3.0751056E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.7799993E38F, -1.791864E38F, -1.6874232E38F, -2.2048141E38F}));
            assert(pack.yawspeed_GET() == 2.421138E38F);
            assert(pack.time_usec_GET() == 3887924535865840933L);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.1845911E38F, -1.7038757E38F, -1.6009357E38F, -1.999036E38F, -1.9811205E38F, -4.7793404E37F, -6.090539E37F, -2.7316207E38F, 2.023932E38F}));
            assert(pack.pitchspeed_GET() == -2.9633728E38F);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.rollspeed_SET(-3.0751056E38F) ;
        p61.time_usec_SET(3887924535865840933L) ;
        p61.q_SET(new float[] {-1.7799993E38F, -1.791864E38F, -1.6874232E38F, -2.2048141E38F}, 0) ;
        p61.covariance_SET(new float[] {1.1845911E38F, -1.7038757E38F, -1.6009357E38F, -1.999036E38F, -1.9811205E38F, -4.7793404E37F, -6.090539E37F, -2.7316207E38F, 2.023932E38F}, 0) ;
        p61.pitchspeed_SET(-2.9633728E38F) ;
        p61.yawspeed_SET(2.421138E38F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.aspd_error_GET() == 1.5211161E38F);
            assert(pack.nav_pitch_GET() == 1.791186E38F);
            assert(pack.nav_roll_GET() == -1.2193476E38F);
            assert(pack.target_bearing_GET() == (short)436);
            assert(pack.nav_bearing_GET() == (short) -28573);
            assert(pack.alt_error_GET() == -2.964014E38F);
            assert(pack.wp_dist_GET() == (char)32042);
            assert(pack.xtrack_error_GET() == 1.9221636E38F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_roll_SET(-1.2193476E38F) ;
        p62.nav_pitch_SET(1.791186E38F) ;
        p62.target_bearing_SET((short)436) ;
        p62.aspd_error_SET(1.5211161E38F) ;
        p62.wp_dist_SET((char)32042) ;
        p62.alt_error_SET(-2.964014E38F) ;
        p62.xtrack_error_SET(1.9221636E38F) ;
        p62.nav_bearing_SET((short) -28573) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.5013588E38F, -1.897098E38F, -2.4782135E38F, 9.265755E37F, -1.4408016E38F, -1.4581789E38F, -1.8070866E38F, -6.539831E37F, -4.552353E37F, -2.594182E38F, 6.855802E37F, -2.4607666E38F, -6.0968903E37F, -1.1187777E38F, -7.215617E36F, 1.0023222E38F, -3.520087E37F, 2.3951128E38F, -6.314458E37F, -2.8884747E38F, -9.5095985E36F, -3.2565694E38F, -1.5629556E38F, 2.2743966E38F, -1.2328195E38F, -2.1443397E38F, -2.842695E38F, 1.567021E38F, 2.8313323E38F, -5.0971953E35F, 2.0502833E38F, 1.6607833E38F, 8.57946E37F, -3.222743E38F, 1.0757838E38F, 1.1800956E38F}));
            assert(pack.lon_GET() == 1251390992);
            assert(pack.vz_GET() == -2.6098725E38F);
            assert(pack.vx_GET() == 2.5391784E38F);
            assert(pack.vy_GET() == -2.6797012E38F);
            assert(pack.lat_GET() == -320405902);
            assert(pack.alt_GET() == -1511812584);
            assert(pack.relative_alt_GET() == -1062209613);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
            assert(pack.time_usec_GET() == 2092301864454414143L);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.lat_SET(-320405902) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE) ;
        p63.covariance_SET(new float[] {-2.5013588E38F, -1.897098E38F, -2.4782135E38F, 9.265755E37F, -1.4408016E38F, -1.4581789E38F, -1.8070866E38F, -6.539831E37F, -4.552353E37F, -2.594182E38F, 6.855802E37F, -2.4607666E38F, -6.0968903E37F, -1.1187777E38F, -7.215617E36F, 1.0023222E38F, -3.520087E37F, 2.3951128E38F, -6.314458E37F, -2.8884747E38F, -9.5095985E36F, -3.2565694E38F, -1.5629556E38F, 2.2743966E38F, -1.2328195E38F, -2.1443397E38F, -2.842695E38F, 1.567021E38F, 2.8313323E38F, -5.0971953E35F, 2.0502833E38F, 1.6607833E38F, 8.57946E37F, -3.222743E38F, 1.0757838E38F, 1.1800956E38F}, 0) ;
        p63.time_usec_SET(2092301864454414143L) ;
        p63.lon_SET(1251390992) ;
        p63.alt_SET(-1511812584) ;
        p63.vx_SET(2.5391784E38F) ;
        p63.relative_alt_SET(-1062209613) ;
        p63.vy_SET(-2.6797012E38F) ;
        p63.vz_SET(-2.6098725E38F) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -3.2660225E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
            assert(pack.vx_GET() == -3.7169113E36F);
            assert(pack.ay_GET() == 3.0017715E38F);
            assert(pack.x_GET() == 7.7737933E37F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.6214073E38F, -1.1409364E38F, -2.284541E38F, 1.9488593E38F, -2.4966766E38F, 2.0135877E38F, 3.1660953E38F, 1.98476E38F, -3.1357982E38F, 2.7709173E38F, -3.2554267E38F, -2.594229E37F, 3.2639877E38F, -1.2214381E37F, -1.764104E38F, 1.3539129E38F, 1.8345421E38F, -7.9494273E37F, -1.5617947E38F, 2.222632E37F, 1.0656279E38F, -1.9846748E38F, 1.229482E38F, 2.3210305E37F, -3.070786E38F, -3.2694065E37F, -2.2518217E38F, -6.643905E37F, 2.9404715E38F, -2.4212037E38F, 1.0050255E38F, 1.6538025E38F, 2.2786226E38F, 2.5290613E38F, -2.4700403E38F, -6.4376814E37F, 1.8614735E38F, -3.3936647E37F, -2.0456595E38F, -1.5559804E38F, -2.4902302E38F, -3.2070228E38F, -1.5791848E38F, -6.272484E36F, -3.1387992E38F}));
            assert(pack.vy_GET() == -1.2827919E38F);
            assert(pack.time_usec_GET() == 2336017218096222038L);
            assert(pack.az_GET() == 1.7072657E38F);
            assert(pack.z_GET() == -2.2950305E38F);
            assert(pack.vz_GET() == 1.7630582E38F);
            assert(pack.ax_GET() == -1.4286545E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.y_SET(-3.2660225E38F) ;
        p64.ay_SET(3.0017715E38F) ;
        p64.vx_SET(-3.7169113E36F) ;
        p64.covariance_SET(new float[] {-1.6214073E38F, -1.1409364E38F, -2.284541E38F, 1.9488593E38F, -2.4966766E38F, 2.0135877E38F, 3.1660953E38F, 1.98476E38F, -3.1357982E38F, 2.7709173E38F, -3.2554267E38F, -2.594229E37F, 3.2639877E38F, -1.2214381E37F, -1.764104E38F, 1.3539129E38F, 1.8345421E38F, -7.9494273E37F, -1.5617947E38F, 2.222632E37F, 1.0656279E38F, -1.9846748E38F, 1.229482E38F, 2.3210305E37F, -3.070786E38F, -3.2694065E37F, -2.2518217E38F, -6.643905E37F, 2.9404715E38F, -2.4212037E38F, 1.0050255E38F, 1.6538025E38F, 2.2786226E38F, 2.5290613E38F, -2.4700403E38F, -6.4376814E37F, 1.8614735E38F, -3.3936647E37F, -2.0456595E38F, -1.5559804E38F, -2.4902302E38F, -3.2070228E38F, -1.5791848E38F, -6.272484E36F, -3.1387992E38F}, 0) ;
        p64.vz_SET(1.7630582E38F) ;
        p64.az_SET(1.7072657E38F) ;
        p64.vy_SET(-1.2827919E38F) ;
        p64.ax_SET(-1.4286545E38F) ;
        p64.x_SET(7.7737933E37F) ;
        p64.time_usec_SET(2336017218096222038L) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION) ;
        p64.z_SET(-2.2950305E38F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)41074);
            assert(pack.chan2_raw_GET() == (char)1384);
            assert(pack.chan6_raw_GET() == (char)24744);
            assert(pack.chan3_raw_GET() == (char)29010);
            assert(pack.chan1_raw_GET() == (char)58624);
            assert(pack.chan14_raw_GET() == (char)7398);
            assert(pack.chan9_raw_GET() == (char)52516);
            assert(pack.chan13_raw_GET() == (char)26795);
            assert(pack.time_boot_ms_GET() == 2890635947L);
            assert(pack.chan11_raw_GET() == (char)21334);
            assert(pack.chan5_raw_GET() == (char)47076);
            assert(pack.chan7_raw_GET() == (char)15044);
            assert(pack.chan16_raw_GET() == (char)64046);
            assert(pack.chan12_raw_GET() == (char)3632);
            assert(pack.chan17_raw_GET() == (char)56171);
            assert(pack.chan8_raw_GET() == (char)42233);
            assert(pack.chan10_raw_GET() == (char)42285);
            assert(pack.chancount_GET() == (char)61);
            assert(pack.chan15_raw_GET() == (char)51585);
            assert(pack.rssi_GET() == (char)117);
            assert(pack.chan18_raw_GET() == (char)20199);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan3_raw_SET((char)29010) ;
        p65.chan15_raw_SET((char)51585) ;
        p65.rssi_SET((char)117) ;
        p65.chan16_raw_SET((char)64046) ;
        p65.chan12_raw_SET((char)3632) ;
        p65.chan7_raw_SET((char)15044) ;
        p65.chan5_raw_SET((char)47076) ;
        p65.chan18_raw_SET((char)20199) ;
        p65.chancount_SET((char)61) ;
        p65.time_boot_ms_SET(2890635947L) ;
        p65.chan4_raw_SET((char)41074) ;
        p65.chan14_raw_SET((char)7398) ;
        p65.chan6_raw_SET((char)24744) ;
        p65.chan9_raw_SET((char)52516) ;
        p65.chan11_raw_SET((char)21334) ;
        p65.chan10_raw_SET((char)42285) ;
        p65.chan17_raw_SET((char)56171) ;
        p65.chan2_raw_SET((char)1384) ;
        p65.chan1_raw_SET((char)58624) ;
        p65.chan13_raw_SET((char)26795) ;
        p65.chan8_raw_SET((char)42233) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.start_stop_GET() == (char)129);
            assert(pack.target_system_GET() == (char)48);
            assert(pack.target_component_GET() == (char)127);
            assert(pack.req_stream_id_GET() == (char)143);
            assert(pack.req_message_rate_GET() == (char)52660);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.start_stop_SET((char)129) ;
        p66.target_system_SET((char)48) ;
        p66.target_component_SET((char)127) ;
        p66.req_stream_id_SET((char)143) ;
        p66.req_message_rate_SET((char)52660) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.on_off_GET() == (char)150);
            assert(pack.message_rate_GET() == (char)65376);
            assert(pack.stream_id_GET() == (char)189);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)189) ;
        p67.message_rate_SET((char)65376) ;
        p67.on_off_SET((char)150) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == (short)25563);
            assert(pack.target_GET() == (char)188);
            assert(pack.buttons_GET() == (char)23095);
            assert(pack.x_GET() == (short)11538);
            assert(pack.r_GET() == (short) -29012);
            assert(pack.y_GET() == (short)19042);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.y_SET((short)19042) ;
        p69.r_SET((short) -29012) ;
        p69.buttons_SET((char)23095) ;
        p69.target_SET((char)188) ;
        p69.x_SET((short)11538) ;
        p69.z_SET((short)25563) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan6_raw_GET() == (char)12139);
            assert(pack.chan8_raw_GET() == (char)39847);
            assert(pack.chan7_raw_GET() == (char)1725);
            assert(pack.target_system_GET() == (char)21);
            assert(pack.chan1_raw_GET() == (char)63445);
            assert(pack.chan4_raw_GET() == (char)58474);
            assert(pack.target_component_GET() == (char)9);
            assert(pack.chan3_raw_GET() == (char)61049);
            assert(pack.chan2_raw_GET() == (char)11346);
            assert(pack.chan5_raw_GET() == (char)20204);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan5_raw_SET((char)20204) ;
        p70.target_component_SET((char)9) ;
        p70.target_system_SET((char)21) ;
        p70.chan4_raw_SET((char)58474) ;
        p70.chan3_raw_SET((char)61049) ;
        p70.chan2_raw_SET((char)11346) ;
        p70.chan8_raw_SET((char)39847) ;
        p70.chan1_raw_SET((char)63445) ;
        p70.chan6_raw_SET((char)12139) ;
        p70.chan7_raw_SET((char)1725) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -6.4282947E37F);
            assert(pack.param4_GET() == -3.2542737E38F);
            assert(pack.x_GET() == -1801069978);
            assert(pack.target_system_GET() == (char)11);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_MOTOR_TEST);
            assert(pack.y_GET() == 402206736);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.param3_GET() == 6.5097834E37F);
            assert(pack.target_component_GET() == (char)39);
            assert(pack.param1_GET() == 1.9697953E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.param2_GET() == -1.5528188E38F);
            assert(pack.seq_GET() == (char)21036);
            assert(pack.autocontinue_GET() == (char)127);
            assert(pack.current_GET() == (char)8);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.y_SET(402206736) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p73.param2_SET(-1.5528188E38F) ;
        p73.autocontinue_SET((char)127) ;
        p73.seq_SET((char)21036) ;
        p73.param1_SET(1.9697953E37F) ;
        p73.param3_SET(6.5097834E37F) ;
        p73.current_SET((char)8) ;
        p73.x_SET(-1801069978) ;
        p73.target_system_SET((char)11) ;
        p73.z_SET(-6.4282947E37F) ;
        p73.command_SET(MAV_CMD.MAV_CMD_DO_MOTOR_TEST) ;
        p73.target_component_SET((char)39) ;
        p73.param4_SET(-3.2542737E38F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.heading_GET() == (short) -11318);
            assert(pack.alt_GET() == 1.6849422E37F);
            assert(pack.climb_GET() == -9.255014E37F);
            assert(pack.airspeed_GET() == -4.100508E37F);
            assert(pack.throttle_GET() == (char)31759);
            assert(pack.groundspeed_GET() == -8.309985E37F);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.alt_SET(1.6849422E37F) ;
        p74.groundspeed_SET(-8.309985E37F) ;
        p74.climb_SET(-9.255014E37F) ;
        p74.throttle_SET((char)31759) ;
        p74.airspeed_SET(-4.100508E37F) ;
        p74.heading_SET((short) -11318) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -4.9061496E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE);
            assert(pack.autocontinue_GET() == (char)48);
            assert(pack.target_system_GET() == (char)5);
            assert(pack.param3_GET() == -2.6710088E38F);
            assert(pack.param4_GET() == 3.3705485E38F);
            assert(pack.target_component_GET() == (char)134);
            assert(pack.param2_GET() == -2.5469086E38F);
            assert(pack.current_GET() == (char)41);
            assert(pack.x_GET() == 1764537462);
            assert(pack.param1_GET() == -1.1188463E38F);
            assert(pack.y_GET() == -111360836);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
        });
        GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p75.z_SET(-4.9061496E37F) ;
        p75.autocontinue_SET((char)48) ;
        p75.target_system_SET((char)5) ;
        p75.param4_SET(3.3705485E38F) ;
        p75.param2_SET(-2.5469086E38F) ;
        p75.current_SET((char)41) ;
        p75.command_SET(MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE) ;
        p75.x_SET(1764537462) ;
        p75.param3_SET(-2.6710088E38F) ;
        p75.param1_SET(-1.1188463E38F) ;
        p75.y_SET(-111360836) ;
        p75.target_component_SET((char)134) ;
        CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param6_GET() == -1.391116E38F);
            assert(pack.param5_GET() == -1.1943529E38F);
            assert(pack.param4_GET() == -1.4894984E38F);
            assert(pack.target_component_GET() == (char)116);
            assert(pack.confirmation_GET() == (char)76);
            assert(pack.target_system_GET() == (char)149);
            assert(pack.param3_GET() == -7.528803E37F);
            assert(pack.param1_GET() == 2.3117966E38F);
            assert(pack.param2_GET() == 7.74917E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_WAYPOINT_USER_5);
            assert(pack.param7_GET() == -2.8209426E38F);
        });
        GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param5_SET(-1.1943529E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_WAYPOINT_USER_5) ;
        p76.target_component_SET((char)116) ;
        p76.param6_SET(-1.391116E38F) ;
        p76.param3_SET(-7.528803E37F) ;
        p76.param4_SET(-1.4894984E38F) ;
        p76.confirmation_SET((char)76) ;
        p76.param7_SET(-2.8209426E38F) ;
        p76.param1_SET(2.3117966E38F) ;
        p76.target_system_SET((char)149) ;
        p76.param2_SET(7.74917E37F) ;
        CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE);
            assert(pack.target_system_TRY(ph) == (char)106);
            assert(pack.result_param2_TRY(ph) == -1057415783);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_IN_PROGRESS);
            assert(pack.target_component_TRY(ph) == (char)37);
            assert(pack.progress_TRY(ph) == (char)226);
        });
        GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.result_SET(MAV_RESULT.MAV_RESULT_IN_PROGRESS) ;
        p77.target_system_SET((char)106, PH) ;
        p77.result_param2_SET(-1057415783, PH) ;
        p77.target_component_SET((char)37, PH) ;
        p77.progress_SET((char)226, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE) ;
        CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.mode_switch_GET() == (char)14);
            assert(pack.yaw_GET() == -3.492703E37F);
            assert(pack.manual_override_switch_GET() == (char)33);
            assert(pack.time_boot_ms_GET() == 2425337680L);
            assert(pack.thrust_GET() == -1.6470069E38F);
            assert(pack.roll_GET() == -3.2885357E36F);
            assert(pack.pitch_GET() == 2.6889958E38F);
        });
        GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.pitch_SET(2.6889958E38F) ;
        p81.roll_SET(-3.2885357E36F) ;
        p81.yaw_SET(-3.492703E37F) ;
        p81.mode_switch_SET((char)14) ;
        p81.manual_override_switch_SET((char)33) ;
        p81.thrust_SET(-1.6470069E38F) ;
        p81.time_boot_ms_SET(2425337680L) ;
        CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.2945487E38F, -2.5337023E38F, 5.510411E37F, 1.4930307E38F}));
            assert(pack.target_system_GET() == (char)94);
            assert(pack.type_mask_GET() == (char)5);
            assert(pack.target_component_GET() == (char)124);
            assert(pack.time_boot_ms_GET() == 2487217178L);
            assert(pack.body_roll_rate_GET() == -2.8598798E38F);
            assert(pack.body_pitch_rate_GET() == -2.5749379E35F);
            assert(pack.thrust_GET() == 2.2139577E38F);
            assert(pack.body_yaw_rate_GET() == 2.2995565E38F);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.body_roll_rate_SET(-2.8598798E38F) ;
        p82.thrust_SET(2.2139577E38F) ;
        p82.time_boot_ms_SET(2487217178L) ;
        p82.target_system_SET((char)94) ;
        p82.q_SET(new float[] {-3.2945487E38F, -2.5337023E38F, 5.510411E37F, 1.4930307E38F}, 0) ;
        p82.body_yaw_rate_SET(2.2995565E38F) ;
        p82.type_mask_SET((char)5) ;
        p82.body_pitch_rate_SET(-2.5749379E35F) ;
        p82.target_component_SET((char)124) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2864397576L);
            assert(pack.thrust_GET() == -3.3436333E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.6822887E37F, -2.590478E38F, 1.0639656E38F, -2.1562635E38F}));
            assert(pack.body_yaw_rate_GET() == 1.4518097E38F);
            assert(pack.body_pitch_rate_GET() == -2.9089154E38F);
            assert(pack.body_roll_rate_GET() == 2.3327688E38F);
            assert(pack.type_mask_GET() == (char)126);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.q_SET(new float[] {3.6822887E37F, -2.590478E38F, 1.0639656E38F, -2.1562635E38F}, 0) ;
        p83.thrust_SET(-3.3436333E38F) ;
        p83.body_pitch_rate_SET(-2.9089154E38F) ;
        p83.body_yaw_rate_SET(1.4518097E38F) ;
        p83.time_boot_ms_SET(2864397576L) ;
        p83.type_mask_SET((char)126) ;
        p83.body_roll_rate_SET(2.3327688E38F) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == -3.3541416E37F);
            assert(pack.x_GET() == -2.8737797E38F);
            assert(pack.yaw_GET() == -2.7257814E37F);
            assert(pack.time_boot_ms_GET() == 1204997245L);
            assert(pack.afy_GET() == -1.7470126E38F);
            assert(pack.target_system_GET() == (char)181);
            assert(pack.afz_GET() == 3.007484E38F);
            assert(pack.afx_GET() == 2.819426E38F);
            assert(pack.type_mask_GET() == (char)17619);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.z_GET() == -3.3376794E38F);
            assert(pack.vz_GET() == 9.85561E37F);
            assert(pack.y_GET() == -2.6300065E38F);
            assert(pack.target_component_GET() == (char)45);
            assert(pack.vy_GET() == -2.6244256E38F);
            assert(pack.vx_GET() == -1.705224E38F);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.yaw_rate_SET(-3.3541416E37F) ;
        p84.time_boot_ms_SET(1204997245L) ;
        p84.vy_SET(-2.6244256E38F) ;
        p84.target_component_SET((char)45) ;
        p84.yaw_SET(-2.7257814E37F) ;
        p84.afz_SET(3.007484E38F) ;
        p84.z_SET(-3.3376794E38F) ;
        p84.vz_SET(9.85561E37F) ;
        p84.afy_SET(-1.7470126E38F) ;
        p84.x_SET(-2.8737797E38F) ;
        p84.target_system_SET((char)181) ;
        p84.y_SET(-2.6300065E38F) ;
        p84.type_mask_SET((char)17619) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p84.vx_SET(-1.705224E38F) ;
        p84.afx_SET(2.819426E38F) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.lon_int_GET() == -750606866);
            assert(pack.afy_GET() == -2.5184445E38F);
            assert(pack.vy_GET() == 2.2325238E38F);
            assert(pack.target_component_GET() == (char)219);
            assert(pack.time_boot_ms_GET() == 2127494784L);
            assert(pack.alt_GET() == 3.0418049E38F);
            assert(pack.lat_int_GET() == 1376786068);
            assert(pack.afx_GET() == -3.3143445E38F);
            assert(pack.type_mask_GET() == (char)13960);
            assert(pack.vz_GET() == -2.9904466E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.afz_GET() == -2.359041E38F);
            assert(pack.yaw_GET() == 7.466481E37F);
            assert(pack.yaw_rate_GET() == -2.909336E38F);
            assert(pack.target_system_GET() == (char)132);
            assert(pack.vx_GET() == -2.9531356E37F);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.time_boot_ms_SET(2127494784L) ;
        p86.vx_SET(-2.9531356E37F) ;
        p86.vz_SET(-2.9904466E38F) ;
        p86.afy_SET(-2.5184445E38F) ;
        p86.alt_SET(3.0418049E38F) ;
        p86.afx_SET(-3.3143445E38F) ;
        p86.vy_SET(2.2325238E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p86.target_system_SET((char)132) ;
        p86.lat_int_SET(1376786068) ;
        p86.yaw_rate_SET(-2.909336E38F) ;
        p86.lon_int_SET(-750606866) ;
        p86.type_mask_SET((char)13960) ;
        p86.yaw_SET(7.466481E37F) ;
        p86.target_component_SET((char)219) ;
        p86.afz_SET(-2.359041E38F) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == 1.8781878E38F);
            assert(pack.lon_int_GET() == 981556011);
            assert(pack.type_mask_GET() == (char)11958);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.yaw_rate_GET() == 2.2983501E38F);
            assert(pack.afx_GET() == 2.9445447E38F);
            assert(pack.time_boot_ms_GET() == 3514332529L);
            assert(pack.vx_GET() == 3.1185223E38F);
            assert(pack.alt_GET() == -2.3960081E38F);
            assert(pack.yaw_GET() == 1.2800904E37F);
            assert(pack.afz_GET() == 2.3699196E38F);
            assert(pack.lat_int_GET() == -2100016856);
            assert(pack.afy_GET() == 2.9638492E38F);
            assert(pack.vy_GET() == 2.6373951E38F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.yaw_SET(1.2800904E37F) ;
        p87.afy_SET(2.9638492E38F) ;
        p87.vx_SET(3.1185223E38F) ;
        p87.lat_int_SET(-2100016856) ;
        p87.yaw_rate_SET(2.2983501E38F) ;
        p87.afz_SET(2.3699196E38F) ;
        p87.type_mask_SET((char)11958) ;
        p87.vz_SET(1.8781878E38F) ;
        p87.lon_int_SET(981556011) ;
        p87.afx_SET(2.9445447E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p87.vy_SET(2.6373951E38F) ;
        p87.time_boot_ms_SET(3514332529L) ;
        p87.alt_SET(-2.3960081E38F) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.5931272E37F);
            assert(pack.roll_GET() == -4.6841027E36F);
            assert(pack.time_boot_ms_GET() == 510091391L);
            assert(pack.pitch_GET() == 1.4697345E38F);
            assert(pack.yaw_GET() == 2.4741925E38F);
            assert(pack.x_GET() == -1.7491896E37F);
            assert(pack.y_GET() == -1.7811874E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(510091391L) ;
        p89.z_SET(-1.5931272E37F) ;
        p89.pitch_SET(1.4697345E38F) ;
        p89.y_SET(-1.7811874E38F) ;
        p89.roll_SET(-4.6841027E36F) ;
        p89.x_SET(-1.7491896E37F) ;
        p89.yaw_SET(2.4741925E38F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 1.1335182E38F);
            assert(pack.alt_GET() == -1315995325);
            assert(pack.rollspeed_GET() == -3.3086182E38F);
            assert(pack.zacc_GET() == (short)826);
            assert(pack.pitchspeed_GET() == 9.895537E37F);
            assert(pack.xacc_GET() == (short)32294);
            assert(pack.vz_GET() == (short)2391);
            assert(pack.yaw_GET() == 3.2861945E38F);
            assert(pack.vx_GET() == (short)22568);
            assert(pack.roll_GET() == -3.1475728E38F);
            assert(pack.pitch_GET() == -1.0296824E38F);
            assert(pack.lon_GET() == -1544491077);
            assert(pack.lat_GET() == 349011768);
            assert(pack.yacc_GET() == (short)25293);
            assert(pack.time_usec_GET() == 190861166325505707L);
            assert(pack.vy_GET() == (short) -10153);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.zacc_SET((short)826) ;
        p90.roll_SET(-3.1475728E38F) ;
        p90.vz_SET((short)2391) ;
        p90.yawspeed_SET(1.1335182E38F) ;
        p90.alt_SET(-1315995325) ;
        p90.pitch_SET(-1.0296824E38F) ;
        p90.time_usec_SET(190861166325505707L) ;
        p90.lon_SET(-1544491077) ;
        p90.xacc_SET((short)32294) ;
        p90.vx_SET((short)22568) ;
        p90.vy_SET((short) -10153) ;
        p90.pitchspeed_SET(9.895537E37F) ;
        p90.lat_SET(349011768) ;
        p90.rollspeed_SET(-3.3086182E38F) ;
        p90.yacc_SET((short)25293) ;
        p90.yaw_SET(3.2861945E38F) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2529802512062725944L);
            assert(pack.yaw_rudder_GET() == -9.307173E37F);
            assert(pack.aux2_GET() == 3.2857745E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_AUTO_DISARMED);
            assert(pack.nav_mode_GET() == (char)98);
            assert(pack.aux1_GET() == -1.0636387E38F);
            assert(pack.pitch_elevator_GET() == -9.076817E37F);
            assert(pack.roll_ailerons_GET() == 2.0772568E37F);
            assert(pack.aux4_GET() == -2.1882442E38F);
            assert(pack.aux3_GET() == 2.6624232E38F);
            assert(pack.throttle_GET() == 3.2451344E38F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.aux4_SET(-2.1882442E38F) ;
        p91.roll_ailerons_SET(2.0772568E37F) ;
        p91.yaw_rudder_SET(-9.307173E37F) ;
        p91.aux1_SET(-1.0636387E38F) ;
        p91.pitch_elevator_SET(-9.076817E37F) ;
        p91.aux3_SET(2.6624232E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED) ;
        p91.aux2_SET(3.2857745E38F) ;
        p91.nav_mode_SET((char)98) ;
        p91.throttle_SET(3.2451344E38F) ;
        p91.time_usec_SET(2529802512062725944L) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)47177);
            assert(pack.rssi_GET() == (char)22);
            assert(pack.time_usec_GET() == 3568308542994796716L);
            assert(pack.chan12_raw_GET() == (char)49135);
            assert(pack.chan1_raw_GET() == (char)63947);
            assert(pack.chan8_raw_GET() == (char)62314);
            assert(pack.chan2_raw_GET() == (char)62116);
            assert(pack.chan10_raw_GET() == (char)64532);
            assert(pack.chan5_raw_GET() == (char)12959);
            assert(pack.chan11_raw_GET() == (char)2757);
            assert(pack.chan3_raw_GET() == (char)37358);
            assert(pack.chan7_raw_GET() == (char)583);
            assert(pack.chan9_raw_GET() == (char)58233);
            assert(pack.chan6_raw_GET() == (char)55317);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan7_raw_SET((char)583) ;
        p92.chan3_raw_SET((char)37358) ;
        p92.chan12_raw_SET((char)49135) ;
        p92.chan5_raw_SET((char)12959) ;
        p92.chan8_raw_SET((char)62314) ;
        p92.chan4_raw_SET((char)47177) ;
        p92.chan6_raw_SET((char)55317) ;
        p92.chan1_raw_SET((char)63947) ;
        p92.time_usec_SET(3568308542994796716L) ;
        p92.chan11_raw_SET((char)2757) ;
        p92.chan9_raw_SET((char)58233) ;
        p92.chan10_raw_SET((char)64532) ;
        p92.chan2_raw_SET((char)62116) ;
        p92.rssi_SET((char)22) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.2961076E38F, 1.9531543E38F, 3.3142613E38F, -1.643611E38F, 1.5328384E38F, -6.7129867E37F, -2.462755E38F, -7.125955E37F, -1.7634839E37F, -1.271204E38F, -2.959893E38F, 9.222938E37F, 2.2605709E38F, 1.4186855E38F, -3.1957567E38F, 1.5907373E38F}));
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_AUTO_ARMED);
            assert(pack.time_usec_GET() == 1214062028736194110L);
            assert(pack.flags_GET() == 3409829459955537963L);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.flags_SET(3409829459955537963L) ;
        p93.time_usec_SET(1214062028736194110L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED) ;
        p93.controls_SET(new float[] {-1.2961076E38F, 1.9531543E38F, 3.3142613E38F, -1.643611E38F, 1.5328384E38F, -6.7129867E37F, -2.462755E38F, -7.125955E37F, -1.7634839E37F, -1.271204E38F, -2.959893E38F, 9.222938E37F, 2.2605709E38F, 1.4186855E38F, -3.1957567E38F, 1.5907373E38F}, 0) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.ground_distance_GET() == 3.248946E38F);
            assert(pack.time_usec_GET() == 4527020667099177339L);
            assert(pack.flow_comp_m_x_GET() == -1.0420038E38F);
            assert(pack.flow_rate_y_TRY(ph) == -2.0244473E38F);
            assert(pack.flow_y_GET() == (short) -30468);
            assert(pack.quality_GET() == (char)77);
            assert(pack.flow_x_GET() == (short)30101);
            assert(pack.flow_comp_m_y_GET() == 1.3740056E38F);
            assert(pack.sensor_id_GET() == (char)176);
            assert(pack.flow_rate_x_TRY(ph) == -2.2690771E38F);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_comp_m_y_SET(1.3740056E38F) ;
        p100.sensor_id_SET((char)176) ;
        p100.flow_rate_x_SET(-2.2690771E38F, PH) ;
        p100.flow_comp_m_x_SET(-1.0420038E38F) ;
        p100.flow_x_SET((short)30101) ;
        p100.time_usec_SET(4527020667099177339L) ;
        p100.flow_rate_y_SET(-2.0244473E38F, PH) ;
        p100.flow_y_SET((short) -30468) ;
        p100.ground_distance_SET(3.248946E38F) ;
        p100.quality_SET((char)77) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 3.0480407E38F);
            assert(pack.yaw_GET() == 3.3028324E38F);
            assert(pack.z_GET() == -2.8273115E38F);
            assert(pack.usec_GET() == 5961289591256542356L);
            assert(pack.roll_GET() == -3.1172346E38F);
            assert(pack.x_GET() == -7.525352E37F);
            assert(pack.y_GET() == 9.109403E37F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(5961289591256542356L) ;
        p101.roll_SET(-3.1172346E38F) ;
        p101.y_SET(9.109403E37F) ;
        p101.z_SET(-2.8273115E38F) ;
        p101.x_SET(-7.525352E37F) ;
        p101.pitch_SET(3.0480407E38F) ;
        p101.yaw_SET(3.3028324E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -3.1765233E38F);
            assert(pack.yaw_GET() == 3.190503E37F);
            assert(pack.usec_GET() == 3672212810884478337L);
            assert(pack.z_GET() == 1.5725114E38F);
            assert(pack.roll_GET() == -5.7777415E37F);
            assert(pack.x_GET() == 3.2587161E38F);
            assert(pack.pitch_GET() == 2.4115704E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.y_SET(-3.1765233E38F) ;
        p102.pitch_SET(2.4115704E38F) ;
        p102.usec_SET(3672212810884478337L) ;
        p102.z_SET(1.5725114E38F) ;
        p102.yaw_SET(3.190503E37F) ;
        p102.roll_SET(-5.7777415E37F) ;
        p102.x_SET(3.2587161E38F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 7.8707604E37F);
            assert(pack.x_GET() == 1.8439953E38F);
            assert(pack.usec_GET() == 2092310263397975112L);
            assert(pack.y_GET() == 1.6454213E38F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.x_SET(1.8439953E38F) ;
        p103.z_SET(7.8707604E37F) ;
        p103.y_SET(1.6454213E38F) ;
        p103.usec_SET(2092310263397975112L) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -2.1480944E38F);
            assert(pack.y_GET() == -2.1190869E38F);
            assert(pack.pitch_GET() == 1.4120574E38F);
            assert(pack.z_GET() == -4.760988E37F);
            assert(pack.roll_GET() == 2.1762032E38F);
            assert(pack.usec_GET() == 7325106019388753687L);
            assert(pack.yaw_GET() == 8.80461E37F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.z_SET(-4.760988E37F) ;
        p104.roll_SET(2.1762032E38F) ;
        p104.yaw_SET(8.80461E37F) ;
        p104.pitch_SET(1.4120574E38F) ;
        p104.x_SET(-2.1480944E38F) ;
        p104.y_SET(-2.1190869E38F) ;
        p104.usec_SET(7325106019388753687L) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6649588562788749178L);
            assert(pack.zmag_GET() == 1.372277E37F);
            assert(pack.fields_updated_GET() == (char)41222);
            assert(pack.ygyro_GET() == 2.077764E38F);
            assert(pack.ymag_GET() == 2.653996E38F);
            assert(pack.zacc_GET() == 3.1603785E38F);
            assert(pack.temperature_GET() == 1.936429E38F);
            assert(pack.pressure_alt_GET() == 2.7163057E38F);
            assert(pack.zgyro_GET() == 2.7389185E38F);
            assert(pack.abs_pressure_GET() == 4.043745E37F);
            assert(pack.xacc_GET() == 1.6755445E38F);
            assert(pack.yacc_GET() == 6.2617254E37F);
            assert(pack.xgyro_GET() == 1.9529664E38F);
            assert(pack.xmag_GET() == 1.8650444E38F);
            assert(pack.diff_pressure_GET() == -1.1728076E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.abs_pressure_SET(4.043745E37F) ;
        p105.xacc_SET(1.6755445E38F) ;
        p105.yacc_SET(6.2617254E37F) ;
        p105.time_usec_SET(6649588562788749178L) ;
        p105.temperature_SET(1.936429E38F) ;
        p105.fields_updated_SET((char)41222) ;
        p105.ygyro_SET(2.077764E38F) ;
        p105.diff_pressure_SET(-1.1728076E38F) ;
        p105.zgyro_SET(2.7389185E38F) ;
        p105.ymag_SET(2.653996E38F) ;
        p105.zmag_SET(1.372277E37F) ;
        p105.xmag_SET(1.8650444E38F) ;
        p105.zacc_SET(3.1603785E38F) ;
        p105.xgyro_SET(1.9529664E38F) ;
        p105.pressure_alt_SET(2.7163057E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_ygyro_GET() == -1.4500501E38F);
            assert(pack.integrated_xgyro_GET() == 3.0110424E38F);
            assert(pack.quality_GET() == (char)208);
            assert(pack.sensor_id_GET() == (char)84);
            assert(pack.temperature_GET() == (short) -9822);
            assert(pack.time_delta_distance_us_GET() == 1648694659L);
            assert(pack.integrated_zgyro_GET() == -1.4848275E38F);
            assert(pack.distance_GET() == 1.7214278E37F);
            assert(pack.integrated_y_GET() == 1.0380756E38F);
            assert(pack.integration_time_us_GET() == 1228688473L);
            assert(pack.time_usec_GET() == 5468056660041021298L);
            assert(pack.integrated_x_GET() == -2.9965905E38F);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_ygyro_SET(-1.4500501E38F) ;
        p106.integrated_zgyro_SET(-1.4848275E38F) ;
        p106.distance_SET(1.7214278E37F) ;
        p106.temperature_SET((short) -9822) ;
        p106.integrated_x_SET(-2.9965905E38F) ;
        p106.integrated_y_SET(1.0380756E38F) ;
        p106.quality_SET((char)208) ;
        p106.sensor_id_SET((char)84) ;
        p106.integrated_xgyro_SET(3.0110424E38F) ;
        p106.time_usec_SET(5468056660041021298L) ;
        p106.time_delta_distance_us_SET(1648694659L) ;
        p106.integration_time_us_SET(1228688473L) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == 6.0924596E37F);
            assert(pack.xacc_GET() == 9.274855E37F);
            assert(pack.ymag_GET() == 1.7890769E38F);
            assert(pack.ygyro_GET() == 3.0797488E38F);
            assert(pack.zgyro_GET() == 4.7629725E37F);
            assert(pack.abs_pressure_GET() == -6.338604E37F);
            assert(pack.pressure_alt_GET() == 2.969889E38F);
            assert(pack.xgyro_GET() == -7.202428E37F);
            assert(pack.zmag_GET() == 7.016865E37F);
            assert(pack.diff_pressure_GET() == 2.0840575E38F);
            assert(pack.yacc_GET() == 2.6471183E38F);
            assert(pack.time_usec_GET() == 7665079409042081387L);
            assert(pack.zacc_GET() == 5.8244696E37F);
            assert(pack.temperature_GET() == 9.441555E37F);
            assert(pack.fields_updated_GET() == 3927919990L);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.fields_updated_SET(3927919990L) ;
        p107.yacc_SET(2.6471183E38F) ;
        p107.zgyro_SET(4.7629725E37F) ;
        p107.zacc_SET(5.8244696E37F) ;
        p107.temperature_SET(9.441555E37F) ;
        p107.xacc_SET(9.274855E37F) ;
        p107.abs_pressure_SET(-6.338604E37F) ;
        p107.xmag_SET(6.0924596E37F) ;
        p107.time_usec_SET(7665079409042081387L) ;
        p107.xgyro_SET(-7.202428E37F) ;
        p107.ygyro_SET(3.0797488E38F) ;
        p107.zmag_SET(7.016865E37F) ;
        p107.diff_pressure_SET(2.0840575E38F) ;
        p107.ymag_SET(1.7890769E38F) ;
        p107.pressure_alt_SET(2.969889E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == -2.2809083E38F);
            assert(pack.ve_GET() == 2.5049891E38F);
            assert(pack.ygyro_GET() == 1.0554429E38F);
            assert(pack.std_dev_horz_GET() == 2.9177191E38F);
            assert(pack.q2_GET() == -1.4122638E38F);
            assert(pack.lon_GET() == -2.9815087E38F);
            assert(pack.pitch_GET() == -1.5936922E38F);
            assert(pack.zgyro_GET() == -6.0361856E37F);
            assert(pack.q1_GET() == 1.0865022E38F);
            assert(pack.xgyro_GET() == -2.2504472E37F);
            assert(pack.q3_GET() == -2.638074E38F);
            assert(pack.vn_GET() == 8.968866E37F);
            assert(pack.vd_GET() == -3.538092E36F);
            assert(pack.yaw_GET() == 9.6912503E36F);
            assert(pack.yacc_GET() == 5.7075857E37F);
            assert(pack.zacc_GET() == 1.5652818E38F);
            assert(pack.roll_GET() == -2.1215958E38F);
            assert(pack.lat_GET() == -2.1401542E38F);
            assert(pack.alt_GET() == -3.2592437E38F);
            assert(pack.q4_GET() == -2.3985878E38F);
            assert(pack.std_dev_vert_GET() == -2.3588396E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.q3_SET(-2.638074E38F) ;
        p108.zacc_SET(1.5652818E38F) ;
        p108.alt_SET(-3.2592437E38F) ;
        p108.lat_SET(-2.1401542E38F) ;
        p108.std_dev_vert_SET(-2.3588396E38F) ;
        p108.q1_SET(1.0865022E38F) ;
        p108.roll_SET(-2.1215958E38F) ;
        p108.std_dev_horz_SET(2.9177191E38F) ;
        p108.yacc_SET(5.7075857E37F) ;
        p108.vn_SET(8.968866E37F) ;
        p108.pitch_SET(-1.5936922E38F) ;
        p108.yaw_SET(9.6912503E36F) ;
        p108.xacc_SET(-2.2809083E38F) ;
        p108.ve_SET(2.5049891E38F) ;
        p108.vd_SET(-3.538092E36F) ;
        p108.zgyro_SET(-6.0361856E37F) ;
        p108.q4_SET(-2.3985878E38F) ;
        p108.q2_SET(-1.4122638E38F) ;
        p108.lon_SET(-2.9815087E38F) ;
        p108.xgyro_SET(-2.2504472E37F) ;
        p108.ygyro_SET(1.0554429E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.rxerrors_GET() == (char)56687);
            assert(pack.rssi_GET() == (char)17);
            assert(pack.remnoise_GET() == (char)125);
            assert(pack.txbuf_GET() == (char)71);
            assert(pack.noise_GET() == (char)105);
            assert(pack.remrssi_GET() == (char)156);
            assert(pack.fixed__GET() == (char)59770);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.fixed__SET((char)59770) ;
        p109.remnoise_SET((char)125) ;
        p109.noise_SET((char)105) ;
        p109.rxerrors_SET((char)56687) ;
        p109.rssi_SET((char)17) ;
        p109.txbuf_SET((char)71) ;
        p109.remrssi_SET((char)156) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)51, (char)137, (char)172, (char)135, (char)157, (char)39, (char)101, (char)226, (char)132, (char)202, (char)192, (char)219, (char)255, (char)144, (char)196, (char)247, (char)147, (char)201, (char)231, (char)152, (char)109, (char)93, (char)146, (char)62, (char)153, (char)41, (char)31, (char)164, (char)164, (char)177, (char)177, (char)243, (char)135, (char)228, (char)93, (char)234, (char)252, (char)230, (char)147, (char)31, (char)51, (char)27, (char)172, (char)69, (char)69, (char)166, (char)156, (char)108, (char)44, (char)31, (char)5, (char)119, (char)86, (char)201, (char)223, (char)40, (char)139, (char)181, (char)2, (char)182, (char)206, (char)215, (char)164, (char)240, (char)231, (char)130, (char)13, (char)139, (char)98, (char)143, (char)2, (char)68, (char)223, (char)111, (char)65, (char)226, (char)125, (char)236, (char)96, (char)109, (char)84, (char)209, (char)233, (char)168, (char)145, (char)114, (char)159, (char)19, (char)164, (char)251, (char)44, (char)16, (char)117, (char)69, (char)45, (char)203, (char)122, (char)125, (char)58, (char)74, (char)80, (char)90, (char)28, (char)174, (char)63, (char)96, (char)47, (char)132, (char)208, (char)184, (char)120, (char)125, (char)135, (char)44, (char)1, (char)130, (char)41, (char)103, (char)221, (char)93, (char)90, (char)248, (char)13, (char)62, (char)162, (char)192, (char)15, (char)41, (char)12, (char)75, (char)233, (char)57, (char)119, (char)75, (char)95, (char)32, (char)119, (char)12, (char)240, (char)95, (char)24, (char)63, (char)100, (char)155, (char)97, (char)157, (char)116, (char)90, (char)72, (char)232, (char)32, (char)190, (char)30, (char)232, (char)79, (char)188, (char)219, (char)158, (char)212, (char)182, (char)85, (char)131, (char)26, (char)191, (char)203, (char)185, (char)111, (char)66, (char)191, (char)209, (char)157, (char)39, (char)140, (char)146, (char)70, (char)12, (char)135, (char)188, (char)39, (char)167, (char)158, (char)92, (char)161, (char)69, (char)111, (char)23, (char)55, (char)164, (char)196, (char)49, (char)54, (char)74, (char)148, (char)13, (char)163, (char)62, (char)61, (char)195, (char)77, (char)40, (char)196, (char)19, (char)130, (char)144, (char)124, (char)144, (char)53, (char)61, (char)158, (char)43, (char)42, (char)21, (char)224, (char)239, (char)199, (char)203, (char)23, (char)67, (char)132, (char)186, (char)78, (char)46, (char)248, (char)177, (char)238, (char)84, (char)194, (char)133, (char)88, (char)9, (char)87, (char)157, (char)54, (char)106, (char)93, (char)33, (char)200, (char)34, (char)65, (char)214, (char)179, (char)40, (char)137, (char)186, (char)44, (char)185, (char)10, (char)255, (char)165, (char)184, (char)108}));
            assert(pack.target_component_GET() == (char)237);
            assert(pack.target_network_GET() == (char)234);
            assert(pack.target_system_GET() == (char)163);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_system_SET((char)163) ;
        p110.target_component_SET((char)237) ;
        p110.target_network_SET((char)234) ;
        p110.payload_SET(new char[] {(char)51, (char)137, (char)172, (char)135, (char)157, (char)39, (char)101, (char)226, (char)132, (char)202, (char)192, (char)219, (char)255, (char)144, (char)196, (char)247, (char)147, (char)201, (char)231, (char)152, (char)109, (char)93, (char)146, (char)62, (char)153, (char)41, (char)31, (char)164, (char)164, (char)177, (char)177, (char)243, (char)135, (char)228, (char)93, (char)234, (char)252, (char)230, (char)147, (char)31, (char)51, (char)27, (char)172, (char)69, (char)69, (char)166, (char)156, (char)108, (char)44, (char)31, (char)5, (char)119, (char)86, (char)201, (char)223, (char)40, (char)139, (char)181, (char)2, (char)182, (char)206, (char)215, (char)164, (char)240, (char)231, (char)130, (char)13, (char)139, (char)98, (char)143, (char)2, (char)68, (char)223, (char)111, (char)65, (char)226, (char)125, (char)236, (char)96, (char)109, (char)84, (char)209, (char)233, (char)168, (char)145, (char)114, (char)159, (char)19, (char)164, (char)251, (char)44, (char)16, (char)117, (char)69, (char)45, (char)203, (char)122, (char)125, (char)58, (char)74, (char)80, (char)90, (char)28, (char)174, (char)63, (char)96, (char)47, (char)132, (char)208, (char)184, (char)120, (char)125, (char)135, (char)44, (char)1, (char)130, (char)41, (char)103, (char)221, (char)93, (char)90, (char)248, (char)13, (char)62, (char)162, (char)192, (char)15, (char)41, (char)12, (char)75, (char)233, (char)57, (char)119, (char)75, (char)95, (char)32, (char)119, (char)12, (char)240, (char)95, (char)24, (char)63, (char)100, (char)155, (char)97, (char)157, (char)116, (char)90, (char)72, (char)232, (char)32, (char)190, (char)30, (char)232, (char)79, (char)188, (char)219, (char)158, (char)212, (char)182, (char)85, (char)131, (char)26, (char)191, (char)203, (char)185, (char)111, (char)66, (char)191, (char)209, (char)157, (char)39, (char)140, (char)146, (char)70, (char)12, (char)135, (char)188, (char)39, (char)167, (char)158, (char)92, (char)161, (char)69, (char)111, (char)23, (char)55, (char)164, (char)196, (char)49, (char)54, (char)74, (char)148, (char)13, (char)163, (char)62, (char)61, (char)195, (char)77, (char)40, (char)196, (char)19, (char)130, (char)144, (char)124, (char)144, (char)53, (char)61, (char)158, (char)43, (char)42, (char)21, (char)224, (char)239, (char)199, (char)203, (char)23, (char)67, (char)132, (char)186, (char)78, (char)46, (char)248, (char)177, (char)238, (char)84, (char)194, (char)133, (char)88, (char)9, (char)87, (char)157, (char)54, (char)106, (char)93, (char)33, (char)200, (char)34, (char)65, (char)214, (char)179, (char)40, (char)137, (char)186, (char)44, (char)185, (char)10, (char)255, (char)165, (char)184, (char)108}, 0) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -7024297062148819865L);
            assert(pack.ts1_GET() == 7493380608462998721L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(7493380608462998721L) ;
        p111.tc1_SET(-7024297062148819865L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4908186725157713680L);
            assert(pack.seq_GET() == 496299885L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(496299885L) ;
        p112.time_usec_SET(4908186725157713680L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.epv_GET() == (char)58420);
            assert(pack.vn_GET() == (short)13319);
            assert(pack.vd_GET() == (short) -4672);
            assert(pack.lon_GET() == 2060026392);
            assert(pack.time_usec_GET() == 8883770423215424898L);
            assert(pack.lat_GET() == 468727444);
            assert(pack.fix_type_GET() == (char)170);
            assert(pack.ve_GET() == (short) -13207);
            assert(pack.vel_GET() == (char)4133);
            assert(pack.alt_GET() == -1323766170);
            assert(pack.cog_GET() == (char)31465);
            assert(pack.satellites_visible_GET() == (char)78);
            assert(pack.eph_GET() == (char)806);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.fix_type_SET((char)170) ;
        p113.alt_SET(-1323766170) ;
        p113.epv_SET((char)58420) ;
        p113.vd_SET((short) -4672) ;
        p113.vel_SET((char)4133) ;
        p113.vn_SET((short)13319) ;
        p113.lat_SET(468727444) ;
        p113.time_usec_SET(8883770423215424898L) ;
        p113.satellites_visible_SET((char)78) ;
        p113.cog_SET((char)31465) ;
        p113.ve_SET((short) -13207) ;
        p113.eph_SET((char)806) ;
        p113.lon_SET(2060026392) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_y_GET() == 1.1546026E38F);
            assert(pack.time_delta_distance_us_GET() == 4072711449L);
            assert(pack.time_usec_GET() == 3854930747259969176L);
            assert(pack.integrated_ygyro_GET() == 2.9875111E38F);
            assert(pack.integration_time_us_GET() == 2769525007L);
            assert(pack.quality_GET() == (char)60);
            assert(pack.integrated_x_GET() == 8.1220387E37F);
            assert(pack.temperature_GET() == (short)19386);
            assert(pack.distance_GET() == -2.3642254E37F);
            assert(pack.sensor_id_GET() == (char)125);
            assert(pack.integrated_xgyro_GET() == -5.702099E37F);
            assert(pack.integrated_zgyro_GET() == -7.171111E37F);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.distance_SET(-2.3642254E37F) ;
        p114.integrated_zgyro_SET(-7.171111E37F) ;
        p114.quality_SET((char)60) ;
        p114.temperature_SET((short)19386) ;
        p114.integrated_x_SET(8.1220387E37F) ;
        p114.integrated_ygyro_SET(2.9875111E38F) ;
        p114.time_usec_SET(3854930747259969176L) ;
        p114.sensor_id_SET((char)125) ;
        p114.integrated_y_SET(1.1546026E38F) ;
        p114.integrated_xgyro_SET(-5.702099E37F) ;
        p114.time_delta_distance_us_SET(4072711449L) ;
        p114.integration_time_us_SET(2769525007L) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == 2.2544468E38F);
            assert(pack.vx_GET() == (short)2872);
            assert(pack.alt_GET() == 381339229);
            assert(pack.true_airspeed_GET() == (char)44692);
            assert(pack.time_usec_GET() == 4355132116392479391L);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {1.7482559E38F, -2.9417086E38F, -2.528433E38F, 2.8504917E38F}));
            assert(pack.lat_GET() == 1729954994);
            assert(pack.yacc_GET() == (short)16101);
            assert(pack.ind_airspeed_GET() == (char)23189);
            assert(pack.pitchspeed_GET() == 1.5178702E38F);
            assert(pack.lon_GET() == 1625918116);
            assert(pack.vz_GET() == (short)27947);
            assert(pack.zacc_GET() == (short) -1590);
            assert(pack.vy_GET() == (short)9590);
            assert(pack.yawspeed_GET() == 2.9165385E38F);
            assert(pack.xacc_GET() == (short) -32126);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(4355132116392479391L) ;
        p115.ind_airspeed_SET((char)23189) ;
        p115.vz_SET((short)27947) ;
        p115.vy_SET((short)9590) ;
        p115.yacc_SET((short)16101) ;
        p115.lat_SET(1729954994) ;
        p115.alt_SET(381339229) ;
        p115.xacc_SET((short) -32126) ;
        p115.rollspeed_SET(2.2544468E38F) ;
        p115.lon_SET(1625918116) ;
        p115.true_airspeed_SET((char)44692) ;
        p115.attitude_quaternion_SET(new float[] {1.7482559E38F, -2.9417086E38F, -2.528433E38F, 2.8504917E38F}, 0) ;
        p115.zacc_SET((short) -1590) ;
        p115.pitchspeed_SET(1.5178702E38F) ;
        p115.yawspeed_SET(2.9165385E38F) ;
        p115.vx_SET((short)2872) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.xgyro_GET() == (short) -18049);
            assert(pack.time_boot_ms_GET() == 3606192688L);
            assert(pack.ymag_GET() == (short) -13088);
            assert(pack.xacc_GET() == (short)6743);
            assert(pack.zmag_GET() == (short) -23657);
            assert(pack.yacc_GET() == (short) -13436);
            assert(pack.zgyro_GET() == (short)13602);
            assert(pack.ygyro_GET() == (short)31545);
            assert(pack.xmag_GET() == (short) -31791);
            assert(pack.zacc_GET() == (short) -1140);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.ygyro_SET((short)31545) ;
        p116.ymag_SET((short) -13088) ;
        p116.yacc_SET((short) -13436) ;
        p116.xacc_SET((short)6743) ;
        p116.xgyro_SET((short) -18049) ;
        p116.time_boot_ms_SET(3606192688L) ;
        p116.zacc_SET((short) -1140) ;
        p116.xmag_SET((short) -31791) ;
        p116.zmag_SET((short) -23657) ;
        p116.zgyro_SET((short)13602) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_GET() == (char)64635);
            assert(pack.target_system_GET() == (char)181);
            assert(pack.start_GET() == (char)53844);
            assert(pack.target_component_GET() == (char)225);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_component_SET((char)225) ;
        p117.start_SET((char)53844) ;
        p117.end_SET((char)64635) ;
        p117.target_system_SET((char)181) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.last_log_num_GET() == (char)34080);
            assert(pack.time_utc_GET() == 3302121256L);
            assert(pack.size_GET() == 2914835862L);
            assert(pack.id_GET() == (char)49807);
            assert(pack.num_logs_GET() == (char)38923);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)49807) ;
        p118.size_SET(2914835862L) ;
        p118.time_utc_SET(3302121256L) ;
        p118.num_logs_SET((char)38923) ;
        p118.last_log_num_SET((char)34080) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == 1665485238L);
            assert(pack.target_component_GET() == (char)168);
            assert(pack.target_system_GET() == (char)108);
            assert(pack.id_GET() == (char)25898);
            assert(pack.ofs_GET() == 2742721581L);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.count_SET(1665485238L) ;
        p119.target_component_SET((char)168) ;
        p119.ofs_SET(2742721581L) ;
        p119.id_SET((char)25898) ;
        p119.target_system_SET((char)108) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.ofs_GET() == 296868647L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)202, (char)18, (char)64, (char)251, (char)3, (char)228, (char)29, (char)173, (char)61, (char)80, (char)221, (char)117, (char)50, (char)252, (char)49, (char)198, (char)21, (char)243, (char)152, (char)233, (char)128, (char)34, (char)252, (char)166, (char)44, (char)187, (char)199, (char)155, (char)146, (char)107, (char)168, (char)247, (char)125, (char)89, (char)191, (char)207, (char)233, (char)144, (char)120, (char)126, (char)211, (char)137, (char)145, (char)1, (char)19, (char)77, (char)25, (char)255, (char)36, (char)121, (char)129, (char)190, (char)178, (char)24, (char)228, (char)152, (char)42, (char)72, (char)153, (char)93, (char)101, (char)105, (char)151, (char)206, (char)17, (char)230, (char)183, (char)44, (char)176, (char)187, (char)70, (char)235, (char)248, (char)117, (char)241, (char)169, (char)242, (char)134, (char)75, (char)79, (char)82, (char)126, (char)141, (char)160, (char)252, (char)188, (char)62, (char)97, (char)106, (char)100}));
            assert(pack.id_GET() == (char)5714);
            assert(pack.count_GET() == (char)69);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.ofs_SET(296868647L) ;
        p120.data__SET(new char[] {(char)202, (char)18, (char)64, (char)251, (char)3, (char)228, (char)29, (char)173, (char)61, (char)80, (char)221, (char)117, (char)50, (char)252, (char)49, (char)198, (char)21, (char)243, (char)152, (char)233, (char)128, (char)34, (char)252, (char)166, (char)44, (char)187, (char)199, (char)155, (char)146, (char)107, (char)168, (char)247, (char)125, (char)89, (char)191, (char)207, (char)233, (char)144, (char)120, (char)126, (char)211, (char)137, (char)145, (char)1, (char)19, (char)77, (char)25, (char)255, (char)36, (char)121, (char)129, (char)190, (char)178, (char)24, (char)228, (char)152, (char)42, (char)72, (char)153, (char)93, (char)101, (char)105, (char)151, (char)206, (char)17, (char)230, (char)183, (char)44, (char)176, (char)187, (char)70, (char)235, (char)248, (char)117, (char)241, (char)169, (char)242, (char)134, (char)75, (char)79, (char)82, (char)126, (char)141, (char)160, (char)252, (char)188, (char)62, (char)97, (char)106, (char)100}, 0) ;
        p120.id_SET((char)5714) ;
        p120.count_SET((char)69) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)34);
            assert(pack.target_component_GET() == (char)204);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)204) ;
        p121.target_system_SET((char)34) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)142);
            assert(pack.target_component_GET() == (char)10);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_component_SET((char)10) ;
        p122.target_system_SET((char)142) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)130);
            assert(pack.len_GET() == (char)209);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)106, (char)38, (char)208, (char)107, (char)205, (char)126, (char)27, (char)108, (char)247, (char)251, (char)159, (char)4, (char)238, (char)5, (char)47, (char)139, (char)191, (char)110, (char)31, (char)255, (char)217, (char)238, (char)245, (char)73, (char)59, (char)57, (char)139, (char)103, (char)148, (char)175, (char)190, (char)100, (char)115, (char)250, (char)140, (char)98, (char)2, (char)222, (char)21, (char)245, (char)243, (char)114, (char)37, (char)94, (char)199, (char)248, (char)239, (char)160, (char)152, (char)68, (char)87, (char)9, (char)189, (char)138, (char)115, (char)2, (char)141, (char)86, (char)108, (char)176, (char)25, (char)103, (char)240, (char)71, (char)48, (char)248, (char)226, (char)60, (char)173, (char)219, (char)75, (char)112, (char)156, (char)254, (char)13, (char)51, (char)27, (char)127, (char)163, (char)64, (char)206, (char)46, (char)191, (char)10, (char)242, (char)210, (char)23, (char)177, (char)230, (char)135, (char)112, (char)117, (char)49, (char)89, (char)111, (char)228, (char)40, (char)31, (char)115, (char)39, (char)7, (char)143, (char)63, (char)143, (char)252, (char)24, (char)40, (char)112, (char)14, (char)233}));
            assert(pack.target_system_GET() == (char)169);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)169) ;
        p123.target_component_SET((char)130) ;
        p123.len_SET((char)209) ;
        p123.data__SET(new char[] {(char)106, (char)38, (char)208, (char)107, (char)205, (char)126, (char)27, (char)108, (char)247, (char)251, (char)159, (char)4, (char)238, (char)5, (char)47, (char)139, (char)191, (char)110, (char)31, (char)255, (char)217, (char)238, (char)245, (char)73, (char)59, (char)57, (char)139, (char)103, (char)148, (char)175, (char)190, (char)100, (char)115, (char)250, (char)140, (char)98, (char)2, (char)222, (char)21, (char)245, (char)243, (char)114, (char)37, (char)94, (char)199, (char)248, (char)239, (char)160, (char)152, (char)68, (char)87, (char)9, (char)189, (char)138, (char)115, (char)2, (char)141, (char)86, (char)108, (char)176, (char)25, (char)103, (char)240, (char)71, (char)48, (char)248, (char)226, (char)60, (char)173, (char)219, (char)75, (char)112, (char)156, (char)254, (char)13, (char)51, (char)27, (char)127, (char)163, (char)64, (char)206, (char)46, (char)191, (char)10, (char)242, (char)210, (char)23, (char)177, (char)230, (char)135, (char)112, (char)117, (char)49, (char)89, (char)111, (char)228, (char)40, (char)31, (char)115, (char)39, (char)7, (char)143, (char)63, (char)143, (char)252, (char)24, (char)40, (char)112, (char)14, (char)233}, 0) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.dgps_age_GET() == 2873550342L);
            assert(pack.epv_GET() == (char)19274);
            assert(pack.vel_GET() == (char)59795);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.cog_GET() == (char)26856);
            assert(pack.alt_GET() == 154919405);
            assert(pack.time_usec_GET() == 6790156789874066445L);
            assert(pack.satellites_visible_GET() == (char)216);
            assert(pack.dgps_numch_GET() == (char)226);
            assert(pack.eph_GET() == (char)2768);
            assert(pack.lat_GET() == -1967044918);
            assert(pack.lon_GET() == -1010016228);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.epv_SET((char)19274) ;
        p124.dgps_numch_SET((char)226) ;
        p124.time_usec_SET(6790156789874066445L) ;
        p124.satellites_visible_SET((char)216) ;
        p124.lat_SET(-1967044918) ;
        p124.vel_SET((char)59795) ;
        p124.eph_SET((char)2768) ;
        p124.cog_SET((char)26856) ;
        p124.dgps_age_SET(2873550342L) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p124.lon_SET(-1010016228) ;
        p124.alt_SET(154919405) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT));
            assert(pack.Vcc_GET() == (char)7601);
            assert(pack.Vservo_GET() == (char)4951);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)4951) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT)) ;
        p125.Vcc_SET((char)7601) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.baudrate_GET() == 1128090812L);
            assert(pack.timeout_GET() == (char)53403);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)16, (char)234, (char)20, (char)8, (char)60, (char)63, (char)226, (char)55, (char)126, (char)60, (char)46, (char)186, (char)221, (char)252, (char)85, (char)187, (char)129, (char)81, (char)99, (char)245, (char)198, (char)126, (char)132, (char)128, (char)112, (char)38, (char)51, (char)100, (char)141, (char)97, (char)155, (char)125, (char)16, (char)244, (char)201, (char)26, (char)83, (char)20, (char)96, (char)162, (char)232, (char)50, (char)3, (char)74, (char)206, (char)119, (char)90, (char)83, (char)162, (char)186, (char)206, (char)79, (char)175, (char)121, (char)254, (char)181, (char)199, (char)5, (char)94, (char)220, (char)205, (char)158, (char)71, (char)15, (char)49, (char)255, (char)186, (char)30, (char)131, (char)205}));
            assert(pack.count_GET() == (char)148);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE));
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.timeout_SET((char)53403) ;
        p126.baudrate_SET(1128090812L) ;
        p126.count_SET((char)148) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE)) ;
        p126.data__SET(new char[] {(char)16, (char)234, (char)20, (char)8, (char)60, (char)63, (char)226, (char)55, (char)126, (char)60, (char)46, (char)186, (char)221, (char)252, (char)85, (char)187, (char)129, (char)81, (char)99, (char)245, (char)198, (char)126, (char)132, (char)128, (char)112, (char)38, (char)51, (char)100, (char)141, (char)97, (char)155, (char)125, (char)16, (char)244, (char)201, (char)26, (char)83, (char)20, (char)96, (char)162, (char)232, (char)50, (char)3, (char)74, (char)206, (char)119, (char)90, (char)83, (char)162, (char)186, (char)206, (char)79, (char)175, (char)121, (char)254, (char)181, (char)199, (char)5, (char)94, (char)220, (char)205, (char)158, (char)71, (char)15, (char)49, (char)255, (char)186, (char)30, (char)131, (char)205}, 0) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_b_mm_GET() == 270158209);
            assert(pack.time_last_baseline_ms_GET() == 3931689709L);
            assert(pack.nsats_GET() == (char)188);
            assert(pack.rtk_receiver_id_GET() == (char)222);
            assert(pack.wn_GET() == (char)26559);
            assert(pack.baseline_c_mm_GET() == -1724206742);
            assert(pack.iar_num_hypotheses_GET() == 596290495);
            assert(pack.baseline_a_mm_GET() == -1895382365);
            assert(pack.rtk_health_GET() == (char)168);
            assert(pack.tow_GET() == 2600750563L);
            assert(pack.accuracy_GET() == 1699960472L);
            assert(pack.rtk_rate_GET() == (char)180);
            assert(pack.baseline_coords_type_GET() == (char)132);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.nsats_SET((char)188) ;
        p127.accuracy_SET(1699960472L) ;
        p127.baseline_c_mm_SET(-1724206742) ;
        p127.wn_SET((char)26559) ;
        p127.rtk_receiver_id_SET((char)222) ;
        p127.iar_num_hypotheses_SET(596290495) ;
        p127.rtk_rate_SET((char)180) ;
        p127.time_last_baseline_ms_SET(3931689709L) ;
        p127.baseline_a_mm_SET(-1895382365) ;
        p127.baseline_b_mm_SET(270158209) ;
        p127.rtk_health_SET((char)168) ;
        p127.baseline_coords_type_SET((char)132) ;
        p127.tow_SET(2600750563L) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.rtk_receiver_id_GET() == (char)185);
            assert(pack.iar_num_hypotheses_GET() == 1447955168);
            assert(pack.baseline_coords_type_GET() == (char)75);
            assert(pack.wn_GET() == (char)5411);
            assert(pack.baseline_c_mm_GET() == 1935612247);
            assert(pack.nsats_GET() == (char)194);
            assert(pack.rtk_rate_GET() == (char)134);
            assert(pack.baseline_a_mm_GET() == -633310092);
            assert(pack.rtk_health_GET() == (char)167);
            assert(pack.baseline_b_mm_GET() == -1194402755);
            assert(pack.tow_GET() == 313262107L);
            assert(pack.time_last_baseline_ms_GET() == 2576543080L);
            assert(pack.accuracy_GET() == 1180021644L);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(2576543080L) ;
        p128.wn_SET((char)5411) ;
        p128.baseline_b_mm_SET(-1194402755) ;
        p128.iar_num_hypotheses_SET(1447955168) ;
        p128.baseline_a_mm_SET(-633310092) ;
        p128.tow_SET(313262107L) ;
        p128.accuracy_SET(1180021644L) ;
        p128.rtk_health_SET((char)167) ;
        p128.baseline_c_mm_SET(1935612247) ;
        p128.rtk_rate_SET((char)134) ;
        p128.rtk_receiver_id_SET((char)185) ;
        p128.nsats_SET((char)194) ;
        p128.baseline_coords_type_SET((char)75) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short) -20740);
            assert(pack.zgyro_GET() == (short) -17041);
            assert(pack.ygyro_GET() == (short) -27157);
            assert(pack.zacc_GET() == (short)16861);
            assert(pack.xgyro_GET() == (short) -1553);
            assert(pack.time_boot_ms_GET() == 233044355L);
            assert(pack.xmag_GET() == (short) -756);
            assert(pack.ymag_GET() == (short) -5882);
            assert(pack.zmag_GET() == (short)9048);
            assert(pack.yacc_GET() == (short) -25805);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.xacc_SET((short) -20740) ;
        p129.xmag_SET((short) -756) ;
        p129.ymag_SET((short) -5882) ;
        p129.yacc_SET((short) -25805) ;
        p129.zacc_SET((short)16861) ;
        p129.time_boot_ms_SET(233044355L) ;
        p129.ygyro_SET((short) -27157) ;
        p129.zmag_SET((short)9048) ;
        p129.zgyro_SET((short) -17041) ;
        p129.xgyro_SET((short) -1553) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.size_GET() == 449751736L);
            assert(pack.packets_GET() == (char)46159);
            assert(pack.type_GET() == (char)10);
            assert(pack.payload_GET() == (char)239);
            assert(pack.height_GET() == (char)50667);
            assert(pack.width_GET() == (char)50131);
            assert(pack.jpg_quality_GET() == (char)163);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)10) ;
        p130.width_SET((char)50131) ;
        p130.jpg_quality_SET((char)163) ;
        p130.height_SET((char)50667) ;
        p130.size_SET(449751736L) ;
        p130.payload_SET((char)239) ;
        p130.packets_SET((char)46159) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)123, (char)7, (char)243, (char)1, (char)172, (char)68, (char)179, (char)44, (char)72, (char)253, (char)33, (char)55, (char)145, (char)248, (char)8, (char)72, (char)66, (char)222, (char)41, (char)67, (char)50, (char)80, (char)0, (char)107, (char)162, (char)209, (char)137, (char)221, (char)54, (char)80, (char)138, (char)204, (char)201, (char)191, (char)173, (char)178, (char)161, (char)161, (char)172, (char)178, (char)49, (char)202, (char)68, (char)164, (char)52, (char)188, (char)87, (char)98, (char)214, (char)230, (char)125, (char)115, (char)199, (char)103, (char)2, (char)17, (char)90, (char)10, (char)14, (char)93, (char)215, (char)191, (char)90, (char)219, (char)49, (char)226, (char)104, (char)45, (char)0, (char)109, (char)216, (char)189, (char)215, (char)40, (char)41, (char)67, (char)103, (char)8, (char)50, (char)56, (char)16, (char)176, (char)137, (char)190, (char)117, (char)2, (char)126, (char)123, (char)204, (char)15, (char)241, (char)198, (char)206, (char)59, (char)210, (char)247, (char)35, (char)10, (char)140, (char)173, (char)54, (char)47, (char)126, (char)230, (char)76, (char)136, (char)72, (char)101, (char)137, (char)154, (char)46, (char)12, (char)24, (char)255, (char)6, (char)161, (char)108, (char)162, (char)243, (char)100, (char)67, (char)70, (char)164, (char)55, (char)171, (char)163, (char)108, (char)238, (char)89, (char)31, (char)216, (char)13, (char)118, (char)196, (char)210, (char)1, (char)214, (char)216, (char)179, (char)147, (char)207, (char)162, (char)229, (char)93, (char)191, (char)1, (char)116, (char)87, (char)222, (char)127, (char)41, (char)143, (char)210, (char)93, (char)164, (char)71, (char)62, (char)227, (char)5, (char)93, (char)240, (char)23, (char)84, (char)246, (char)206, (char)189, (char)104, (char)149, (char)107, (char)156, (char)186, (char)118, (char)148, (char)74, (char)59, (char)128, (char)160, (char)8, (char)18, (char)246, (char)14, (char)236, (char)161, (char)116, (char)174, (char)55, (char)236, (char)49, (char)151, (char)189, (char)40, (char)66, (char)53, (char)150, (char)189, (char)194, (char)172, (char)177, (char)32, (char)35, (char)194, (char)106, (char)244, (char)69, (char)223, (char)35, (char)172, (char)208, (char)13, (char)254, (char)162, (char)5, (char)224, (char)11, (char)166, (char)25, (char)151, (char)102, (char)214, (char)73, (char)238, (char)173, (char)118, (char)113, (char)96, (char)253, (char)6, (char)3, (char)43, (char)16, (char)63, (char)130, (char)1, (char)16, (char)219, (char)164, (char)234, (char)118, (char)35, (char)220, (char)215, (char)164, (char)120, (char)17, (char)64, (char)6, (char)77, (char)132, (char)65, (char)114, (char)108, (char)235, (char)42}));
            assert(pack.seqnr_GET() == (char)43635);
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)43635) ;
        p131.data__SET(new char[] {(char)123, (char)7, (char)243, (char)1, (char)172, (char)68, (char)179, (char)44, (char)72, (char)253, (char)33, (char)55, (char)145, (char)248, (char)8, (char)72, (char)66, (char)222, (char)41, (char)67, (char)50, (char)80, (char)0, (char)107, (char)162, (char)209, (char)137, (char)221, (char)54, (char)80, (char)138, (char)204, (char)201, (char)191, (char)173, (char)178, (char)161, (char)161, (char)172, (char)178, (char)49, (char)202, (char)68, (char)164, (char)52, (char)188, (char)87, (char)98, (char)214, (char)230, (char)125, (char)115, (char)199, (char)103, (char)2, (char)17, (char)90, (char)10, (char)14, (char)93, (char)215, (char)191, (char)90, (char)219, (char)49, (char)226, (char)104, (char)45, (char)0, (char)109, (char)216, (char)189, (char)215, (char)40, (char)41, (char)67, (char)103, (char)8, (char)50, (char)56, (char)16, (char)176, (char)137, (char)190, (char)117, (char)2, (char)126, (char)123, (char)204, (char)15, (char)241, (char)198, (char)206, (char)59, (char)210, (char)247, (char)35, (char)10, (char)140, (char)173, (char)54, (char)47, (char)126, (char)230, (char)76, (char)136, (char)72, (char)101, (char)137, (char)154, (char)46, (char)12, (char)24, (char)255, (char)6, (char)161, (char)108, (char)162, (char)243, (char)100, (char)67, (char)70, (char)164, (char)55, (char)171, (char)163, (char)108, (char)238, (char)89, (char)31, (char)216, (char)13, (char)118, (char)196, (char)210, (char)1, (char)214, (char)216, (char)179, (char)147, (char)207, (char)162, (char)229, (char)93, (char)191, (char)1, (char)116, (char)87, (char)222, (char)127, (char)41, (char)143, (char)210, (char)93, (char)164, (char)71, (char)62, (char)227, (char)5, (char)93, (char)240, (char)23, (char)84, (char)246, (char)206, (char)189, (char)104, (char)149, (char)107, (char)156, (char)186, (char)118, (char)148, (char)74, (char)59, (char)128, (char)160, (char)8, (char)18, (char)246, (char)14, (char)236, (char)161, (char)116, (char)174, (char)55, (char)236, (char)49, (char)151, (char)189, (char)40, (char)66, (char)53, (char)150, (char)189, (char)194, (char)172, (char)177, (char)32, (char)35, (char)194, (char)106, (char)244, (char)69, (char)223, (char)35, (char)172, (char)208, (char)13, (char)254, (char)162, (char)5, (char)224, (char)11, (char)166, (char)25, (char)151, (char)102, (char)214, (char)73, (char)238, (char)173, (char)118, (char)113, (char)96, (char)253, (char)6, (char)3, (char)43, (char)16, (char)63, (char)130, (char)1, (char)16, (char)219, (char)164, (char)234, (char)118, (char)35, (char)220, (char)215, (char)164, (char)120, (char)17, (char)64, (char)6, (char)77, (char)132, (char)65, (char)114, (char)108, (char)235, (char)42}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)202);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_45);
            assert(pack.min_distance_GET() == (char)29708);
            assert(pack.time_boot_ms_GET() == 826046442L);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.max_distance_GET() == (char)46314);
            assert(pack.current_distance_GET() == (char)59085);
            assert(pack.covariance_GET() == (char)110);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.id_SET((char)202) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_45) ;
        p132.time_boot_ms_SET(826046442L) ;
        p132.current_distance_SET((char)59085) ;
        p132.covariance_SET((char)110) ;
        p132.max_distance_SET((char)46314) ;
        p132.min_distance_SET((char)29708) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mask_GET() == 2082553471801800528L);
            assert(pack.lon_GET() == -96860124);
            assert(pack.grid_spacing_GET() == (char)55116);
            assert(pack.lat_GET() == -485431340);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lon_SET(-96860124) ;
        p133.lat_SET(-485431340) ;
        p133.grid_spacing_SET((char)55116) ;
        p133.mask_SET(2082553471801800528L) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.gridbit_GET() == (char)136);
            assert(pack.grid_spacing_GET() == (char)8179);
            assert(pack.lon_GET() == -357064953);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -11991, (short)22913, (short) -7555, (short) -28516, (short) -22301, (short) -27944, (short)18663, (short) -6000, (short)26084, (short)3544, (short)30392, (short)1076, (short)8245, (short) -17078, (short) -10288, (short) -24941}));
            assert(pack.lat_GET() == -2003162050);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.data__SET(new short[] {(short) -11991, (short)22913, (short) -7555, (short) -28516, (short) -22301, (short) -27944, (short)18663, (short) -6000, (short)26084, (short)3544, (short)30392, (short)1076, (short)8245, (short) -17078, (short) -10288, (short) -24941}, 0) ;
        p134.lat_SET(-2003162050) ;
        p134.lon_SET(-357064953) ;
        p134.gridbit_SET((char)136) ;
        p134.grid_spacing_SET((char)8179) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -714220626);
            assert(pack.lat_GET() == 604231002);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(604231002) ;
        p135.lon_SET(-714220626) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.spacing_GET() == (char)19299);
            assert(pack.lat_GET() == -1551088789);
            assert(pack.terrain_height_GET() == 1.2124511E38F);
            assert(pack.pending_GET() == (char)19146);
            assert(pack.lon_GET() == -1264139138);
            assert(pack.current_height_GET() == -2.0568562E38F);
            assert(pack.loaded_GET() == (char)28159);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.spacing_SET((char)19299) ;
        p136.pending_SET((char)19146) ;
        p136.current_height_SET(-2.0568562E38F) ;
        p136.terrain_height_SET(1.2124511E38F) ;
        p136.lon_SET(-1264139138) ;
        p136.loaded_SET((char)28159) ;
        p136.lat_SET(-1551088789) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == 1.874822E38F);
            assert(pack.press_abs_GET() == 2.2264382E38F);
            assert(pack.temperature_GET() == (short)25427);
            assert(pack.time_boot_ms_GET() == 18415434L);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(18415434L) ;
        p137.temperature_SET((short)25427) ;
        p137.press_diff_SET(1.874822E38F) ;
        p137.press_abs_SET(2.2264382E38F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -2.2445407E38F);
            assert(pack.z_GET() == 8.512821E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.2457248E38F, 2.6409504E38F, -7.848507E37F, -1.9865752E38F}));
            assert(pack.time_usec_GET() == 212077581874160300L);
            assert(pack.y_GET() == -1.4432495E38F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.q_SET(new float[] {-2.2457248E38F, 2.6409504E38F, -7.848507E37F, -1.9865752E38F}, 0) ;
        p138.time_usec_SET(212077581874160300L) ;
        p138.y_SET(-1.4432495E38F) ;
        p138.z_SET(8.512821E37F) ;
        p138.x_SET(-2.2445407E38F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {4.3611034E37F, -6.891793E37F, -1.2179394E38F, 2.0192177E38F, 1.9332029E38F, 3.1195297E38F, 1.6652633E38F, 6.849108E37F}));
            assert(pack.time_usec_GET() == 4654608987932809248L);
            assert(pack.target_system_GET() == (char)119);
            assert(pack.group_mlx_GET() == (char)234);
            assert(pack.target_component_GET() == (char)165);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.group_mlx_SET((char)234) ;
        p139.target_system_SET((char)119) ;
        p139.target_component_SET((char)165) ;
        p139.controls_SET(new float[] {4.3611034E37F, -6.891793E37F, -1.2179394E38F, 2.0192177E38F, 1.9332029E38F, 3.1195297E38F, 1.6652633E38F, 6.849108E37F}, 0) ;
        p139.time_usec_SET(4654608987932809248L) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.4287446E36F, -2.3627928E38F, -3.1660564E38F, -9.804377E37F, 3.112612E38F, -2.504075E38F, 9.454373E36F, -1.1436194E38F}));
            assert(pack.group_mlx_GET() == (char)14);
            assert(pack.time_usec_GET() == 5904103106350628328L);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.controls_SET(new float[] {-2.4287446E36F, -2.3627928E38F, -3.1660564E38F, -9.804377E37F, 3.112612E38F, -2.504075E38F, 9.454373E36F, -1.1436194E38F}, 0) ;
        p140.time_usec_SET(5904103106350628328L) ;
        p140.group_mlx_SET((char)14) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_monotonic_GET() == -1.0509469E38F);
            assert(pack.bottom_clearance_GET() == 1.618204E38F);
            assert(pack.altitude_terrain_GET() == 2.8281648E38F);
            assert(pack.altitude_relative_GET() == 1.721214E38F);
            assert(pack.time_usec_GET() == 5224800920067009857L);
            assert(pack.altitude_local_GET() == -8.982417E37F);
            assert(pack.altitude_amsl_GET() == -3.7695697E37F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_terrain_SET(2.8281648E38F) ;
        p141.bottom_clearance_SET(1.618204E38F) ;
        p141.altitude_monotonic_SET(-1.0509469E38F) ;
        p141.altitude_amsl_SET(-3.7695697E37F) ;
        p141.altitude_relative_SET(1.721214E38F) ;
        p141.altitude_local_SET(-8.982417E37F) ;
        p141.time_usec_SET(5224800920067009857L) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)37, (char)165, (char)247, (char)69, (char)174, (char)166, (char)201, (char)56, (char)23, (char)62, (char)106, (char)14, (char)68, (char)107, (char)191, (char)31, (char)66, (char)175, (char)68, (char)18, (char)84, (char)143, (char)109, (char)234, (char)251, (char)189, (char)22, (char)28, (char)165, (char)11, (char)46, (char)96, (char)189, (char)21, (char)32, (char)149, (char)62, (char)74, (char)48, (char)141, (char)150, (char)36, (char)118, (char)156, (char)199, (char)242, (char)155, (char)146, (char)43, (char)248, (char)4, (char)224, (char)227, (char)236, (char)188, (char)146, (char)24, (char)240, (char)224, (char)5, (char)5, (char)15, (char)202, (char)232, (char)16, (char)240, (char)244, (char)203, (char)65, (char)232, (char)224, (char)99, (char)34, (char)68, (char)177, (char)60, (char)219, (char)199, (char)243, (char)145, (char)23, (char)144, (char)106, (char)32, (char)166, (char)78, (char)211, (char)207, (char)124, (char)111, (char)148, (char)13, (char)44, (char)254, (char)111, (char)127, (char)124, (char)97, (char)234, (char)119, (char)204, (char)187, (char)183, (char)63, (char)178, (char)89, (char)197, (char)126, (char)127, (char)99, (char)230, (char)205, (char)56, (char)89, (char)88, (char)217, (char)149, (char)24, (char)124, (char)208}));
            assert(pack.request_id_GET() == (char)156);
            assert(pack.transfer_type_GET() == (char)191);
            assert(pack.uri_type_GET() == (char)144);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)150, (char)0, (char)119, (char)174, (char)240, (char)246, (char)195, (char)33, (char)105, (char)106, (char)18, (char)162, (char)141, (char)244, (char)217, (char)87, (char)60, (char)73, (char)222, (char)217, (char)209, (char)23, (char)149, (char)91, (char)123, (char)1, (char)170, (char)137, (char)0, (char)205, (char)221, (char)104, (char)223, (char)240, (char)231, (char)39, (char)173, (char)54, (char)79, (char)183, (char)225, (char)170, (char)95, (char)140, (char)204, (char)24, (char)87, (char)13, (char)59, (char)202, (char)154, (char)181, (char)172, (char)172, (char)157, (char)67, (char)158, (char)248, (char)254, (char)110, (char)113, (char)106, (char)117, (char)138, (char)15, (char)124, (char)143, (char)52, (char)243, (char)254, (char)18, (char)206, (char)214, (char)121, (char)48, (char)52, (char)247, (char)194, (char)79, (char)160, (char)251, (char)32, (char)223, (char)232, (char)182, (char)172, (char)10, (char)73, (char)167, (char)107, (char)4, (char)128, (char)176, (char)29, (char)173, (char)70, (char)95, (char)79, (char)156, (char)24, (char)139, (char)59, (char)158, (char)227, (char)148, (char)55, (char)159, (char)161, (char)230, (char)15, (char)186, (char)135, (char)56, (char)137, (char)7, (char)100, (char)21, (char)63, (char)34, (char)21}));
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_SET(new char[] {(char)150, (char)0, (char)119, (char)174, (char)240, (char)246, (char)195, (char)33, (char)105, (char)106, (char)18, (char)162, (char)141, (char)244, (char)217, (char)87, (char)60, (char)73, (char)222, (char)217, (char)209, (char)23, (char)149, (char)91, (char)123, (char)1, (char)170, (char)137, (char)0, (char)205, (char)221, (char)104, (char)223, (char)240, (char)231, (char)39, (char)173, (char)54, (char)79, (char)183, (char)225, (char)170, (char)95, (char)140, (char)204, (char)24, (char)87, (char)13, (char)59, (char)202, (char)154, (char)181, (char)172, (char)172, (char)157, (char)67, (char)158, (char)248, (char)254, (char)110, (char)113, (char)106, (char)117, (char)138, (char)15, (char)124, (char)143, (char)52, (char)243, (char)254, (char)18, (char)206, (char)214, (char)121, (char)48, (char)52, (char)247, (char)194, (char)79, (char)160, (char)251, (char)32, (char)223, (char)232, (char)182, (char)172, (char)10, (char)73, (char)167, (char)107, (char)4, (char)128, (char)176, (char)29, (char)173, (char)70, (char)95, (char)79, (char)156, (char)24, (char)139, (char)59, (char)158, (char)227, (char)148, (char)55, (char)159, (char)161, (char)230, (char)15, (char)186, (char)135, (char)56, (char)137, (char)7, (char)100, (char)21, (char)63, (char)34, (char)21}, 0) ;
        p142.uri_type_SET((char)144) ;
        p142.storage_SET(new char[] {(char)37, (char)165, (char)247, (char)69, (char)174, (char)166, (char)201, (char)56, (char)23, (char)62, (char)106, (char)14, (char)68, (char)107, (char)191, (char)31, (char)66, (char)175, (char)68, (char)18, (char)84, (char)143, (char)109, (char)234, (char)251, (char)189, (char)22, (char)28, (char)165, (char)11, (char)46, (char)96, (char)189, (char)21, (char)32, (char)149, (char)62, (char)74, (char)48, (char)141, (char)150, (char)36, (char)118, (char)156, (char)199, (char)242, (char)155, (char)146, (char)43, (char)248, (char)4, (char)224, (char)227, (char)236, (char)188, (char)146, (char)24, (char)240, (char)224, (char)5, (char)5, (char)15, (char)202, (char)232, (char)16, (char)240, (char)244, (char)203, (char)65, (char)232, (char)224, (char)99, (char)34, (char)68, (char)177, (char)60, (char)219, (char)199, (char)243, (char)145, (char)23, (char)144, (char)106, (char)32, (char)166, (char)78, (char)211, (char)207, (char)124, (char)111, (char)148, (char)13, (char)44, (char)254, (char)111, (char)127, (char)124, (char)97, (char)234, (char)119, (char)204, (char)187, (char)183, (char)63, (char)178, (char)89, (char)197, (char)126, (char)127, (char)99, (char)230, (char)205, (char)56, (char)89, (char)88, (char)217, (char)149, (char)24, (char)124, (char)208}, 0) ;
        p142.transfer_type_SET((char)191) ;
        p142.request_id_SET((char)156) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)8440);
            assert(pack.press_abs_GET() == -2.9468674E38F);
            assert(pack.time_boot_ms_GET() == 2352013154L);
            assert(pack.press_diff_GET() == 2.485703E38F);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.temperature_SET((short)8440) ;
        p143.press_abs_SET(-2.9468674E38F) ;
        p143.time_boot_ms_SET(2352013154L) ;
        p143.press_diff_SET(2.485703E38F) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 3.3705008E38F);
            assert(pack.est_capabilities_GET() == (char)250);
            assert(pack.timestamp_GET() == 6960788947291082039L);
            assert(pack.lat_GET() == -312968037);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {2.5776334E38F, 1.2585407E38F, 1.8631955E38F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-1.0681784E38F, 3.293266E38F, 1.0073358E38F, 5.1233734E36F}));
            assert(pack.custom_state_GET() == 5101766946059482313L);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {8.670139E37F, -2.7079851E37F, -1.0113761E38F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {1.9183551E38F, 7.7406787E37F, 2.6009363E38F}));
            assert(pack.lon_GET() == -742100904);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-1.2072674E38F, 1.7799297E38F, -2.7208159E38F}));
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.alt_SET(3.3705008E38F) ;
        p144.attitude_q_SET(new float[] {-1.0681784E38F, 3.293266E38F, 1.0073358E38F, 5.1233734E36F}, 0) ;
        p144.timestamp_SET(6960788947291082039L) ;
        p144.custom_state_SET(5101766946059482313L) ;
        p144.lon_SET(-742100904) ;
        p144.rates_SET(new float[] {1.9183551E38F, 7.7406787E37F, 2.6009363E38F}, 0) ;
        p144.est_capabilities_SET((char)250) ;
        p144.lat_SET(-312968037) ;
        p144.acc_SET(new float[] {-1.2072674E38F, 1.7799297E38F, -2.7208159E38F}, 0) ;
        p144.position_cov_SET(new float[] {2.5776334E38F, 1.2585407E38F, 1.8631955E38F}, 0) ;
        p144.vel_SET(new float[] {8.670139E37F, -2.7079851E37F, -1.0113761E38F}, 0) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.y_pos_GET() == 3.2966526E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {1.5492688E38F, -2.2605453E38F, 1.2932602E38F}));
            assert(pack.z_vel_GET() == -9.408922E37F);
            assert(pack.time_usec_GET() == 463433578606334229L);
            assert(pack.airspeed_GET() == -1.3727938E38F);
            assert(pack.z_acc_GET() == -1.5650585E38F);
            assert(pack.pitch_rate_GET() == 1.7767117E38F);
            assert(pack.z_pos_GET() == -5.695066E37F);
            assert(pack.y_vel_GET() == -9.124433E37F);
            assert(pack.yaw_rate_GET() == 2.915018E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {-3.054723E38F, -1.0163998E38F, -2.9043916E38F}));
            assert(pack.x_pos_GET() == -1.8150459E38F);
            assert(pack.roll_rate_GET() == 1.9700493E38F);
            assert(pack.y_acc_GET() == -1.2923807E38F);
            assert(pack.x_vel_GET() == -8.326077E37F);
            assert(pack.x_acc_GET() == -2.7391585E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.3689688E37F, -3.570454E37F, 2.5843968E38F, -2.8514129E38F}));
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.vel_variance_SET(new float[] {1.5492688E38F, -2.2605453E38F, 1.2932602E38F}, 0) ;
        p146.airspeed_SET(-1.3727938E38F) ;
        p146.x_acc_SET(-2.7391585E38F) ;
        p146.x_pos_SET(-1.8150459E38F) ;
        p146.q_SET(new float[] {3.3689688E37F, -3.570454E37F, 2.5843968E38F, -2.8514129E38F}, 0) ;
        p146.pos_variance_SET(new float[] {-3.054723E38F, -1.0163998E38F, -2.9043916E38F}, 0) ;
        p146.y_vel_SET(-9.124433E37F) ;
        p146.y_acc_SET(-1.2923807E38F) ;
        p146.y_pos_SET(3.2966526E38F) ;
        p146.yaw_rate_SET(2.915018E38F) ;
        p146.z_vel_SET(-9.408922E37F) ;
        p146.z_pos_SET(-5.695066E37F) ;
        p146.time_usec_SET(463433578606334229L) ;
        p146.roll_rate_SET(1.9700493E38F) ;
        p146.pitch_rate_SET(1.7767117E38F) ;
        p146.x_vel_SET(-8.326077E37F) ;
        p146.z_acc_SET(-1.5650585E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.energy_consumed_GET() == -2072096624);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)23451, (char)8997, (char)15713, (char)17865, (char)49727, (char)45493, (char)38662, (char)5931, (char)52076, (char)58555}));
            assert(pack.temperature_GET() == (short) -21990);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
            assert(pack.battery_remaining_GET() == (byte) - 3);
            assert(pack.current_consumed_GET() == 1316925297);
            assert(pack.current_battery_GET() == (short)32055);
            assert(pack.id_GET() == (char)147);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.current_consumed_SET(1316925297) ;
        p147.voltages_SET(new char[] {(char)23451, (char)8997, (char)15713, (char)17865, (char)49727, (char)45493, (char)38662, (char)5931, (char)52076, (char)58555}, 0) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION) ;
        p147.energy_consumed_SET(-2072096624) ;
        p147.id_SET((char)147) ;
        p147.battery_remaining_SET((byte) - 3) ;
        p147.current_battery_SET((short)32055) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN) ;
        p147.temperature_SET((short) -21990) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)66, (char)247, (char)225, (char)156, (char)194, (char)241, (char)144, (char)103}));
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION));
            assert(pack.os_sw_version_GET() == 640836736L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)115, (char)37, (char)132, (char)32, (char)254, (char)144, (char)21, (char)32, (char)94, (char)0, (char)236, (char)200, (char)33, (char)209, (char)159, (char)201, (char)222, (char)182}));
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)254, (char)50, (char)208, (char)108, (char)205, (char)10, (char)109, (char)153}));
            assert(pack.flight_sw_version_GET() == 986418470L);
            assert(pack.vendor_id_GET() == (char)32833);
            assert(pack.product_id_GET() == (char)54166);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)36, (char)248, (char)59, (char)189, (char)67, (char)147, (char)61, (char)68}));
            assert(pack.board_version_GET() == 270295542L);
            assert(pack.uid_GET() == 3302854951895859012L);
            assert(pack.middleware_sw_version_GET() == 1875457303L);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.middleware_sw_version_SET(1875457303L) ;
        p148.board_version_SET(270295542L) ;
        p148.flight_custom_version_SET(new char[] {(char)254, (char)50, (char)208, (char)108, (char)205, (char)10, (char)109, (char)153}, 0) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION)) ;
        p148.flight_sw_version_SET(986418470L) ;
        p148.uid_SET(3302854951895859012L) ;
        p148.uid2_SET(new char[] {(char)115, (char)37, (char)132, (char)32, (char)254, (char)144, (char)21, (char)32, (char)94, (char)0, (char)236, (char)200, (char)33, (char)209, (char)159, (char)201, (char)222, (char)182}, 0, PH) ;
        p148.middleware_custom_version_SET(new char[] {(char)36, (char)248, (char)59, (char)189, (char)67, (char)147, (char)61, (char)68}, 0) ;
        p148.product_id_SET((char)54166) ;
        p148.vendor_id_SET((char)32833) ;
        p148.os_custom_version_SET(new char[] {(char)66, (char)247, (char)225, (char)156, (char)194, (char)241, (char)144, (char)103}, 0) ;
        p148.os_sw_version_SET(640836736L) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6501811619906630435L);
            assert(pack.y_TRY(ph) == -4.1898573E37F);
            assert(pack.z_TRY(ph) == 2.7324736E38F);
            assert(pack.angle_x_GET() == 1.9490599E38F);
            assert(pack.target_num_GET() == (char)115);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.angle_y_GET() == -3.242502E38F);
            assert(pack.position_valid_TRY(ph) == (char)114);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {-2.3879373E37F, 6.891795E36F, -5.313012E37F, -3.1852731E38F}));
            assert(pack.distance_GET() == 7.1081885E37F);
            assert(pack.size_y_GET() == 2.1813162E38F);
            assert(pack.x_TRY(ph) == -1.0539443E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
            assert(pack.size_x_GET() == -2.1227205E38F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.distance_SET(7.1081885E37F) ;
        p149.time_usec_SET(6501811619906630435L) ;
        p149.position_valid_SET((char)114, PH) ;
        p149.angle_x_SET(1.9490599E38F) ;
        p149.y_SET(-4.1898573E37F, PH) ;
        p149.target_num_SET((char)115) ;
        p149.size_y_SET(2.1813162E38F) ;
        p149.size_x_SET(-2.1227205E38F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p149.q_SET(new float[] {-2.3879373E37F, 6.891795E36F, -5.313012E37F, -3.1852731E38F}, 0, PH) ;
        p149.angle_y_SET(-3.242502E38F) ;
        p149.x_SET(-1.0539443E38F, PH) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON) ;
        p149.z_SET(2.7324736E38F, PH) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pos_horiz_accuracy_GET() == 2.5916715E38F);
            assert(pack.mag_ratio_GET() == -2.8936011E38F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL));
            assert(pack.pos_horiz_ratio_GET() == 5.6206715E37F);
            assert(pack.time_usec_GET() == 9175276354661076610L);
            assert(pack.tas_ratio_GET() == -1.3821513E38F);
            assert(pack.vel_ratio_GET() == 2.7285524E38F);
            assert(pack.hagl_ratio_GET() == 8.023717E37F);
            assert(pack.pos_vert_accuracy_GET() == -1.3986886E38F);
            assert(pack.pos_vert_ratio_GET() == -2.8129558E38F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.tas_ratio_SET(-1.3821513E38F) ;
        p230.hagl_ratio_SET(8.023717E37F) ;
        p230.mag_ratio_SET(-2.8936011E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL)) ;
        p230.vel_ratio_SET(2.7285524E38F) ;
        p230.pos_vert_accuracy_SET(-1.3986886E38F) ;
        p230.pos_horiz_accuracy_SET(2.5916715E38F) ;
        p230.time_usec_SET(9175276354661076610L) ;
        p230.pos_horiz_ratio_SET(5.6206715E37F) ;
        p230.pos_vert_ratio_SET(-2.8129558E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.var_horiz_GET() == -1.7452517E38F);
            assert(pack.time_usec_GET() == 230258356653570373L);
            assert(pack.wind_z_GET() == -2.2784898E38F);
            assert(pack.wind_y_GET() == 1.5440277E38F);
            assert(pack.vert_accuracy_GET() == -1.6650992E38F);
            assert(pack.wind_x_GET() == 3.3943657E38F);
            assert(pack.horiz_accuracy_GET() == 2.65669E38F);
            assert(pack.var_vert_GET() == -3.9938123E37F);
            assert(pack.wind_alt_GET() == 1.8868815E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.horiz_accuracy_SET(2.65669E38F) ;
        p231.var_horiz_SET(-1.7452517E38F) ;
        p231.var_vert_SET(-3.9938123E37F) ;
        p231.wind_z_SET(-2.2784898E38F) ;
        p231.time_usec_SET(230258356653570373L) ;
        p231.wind_alt_SET(1.8868815E38F) ;
        p231.vert_accuracy_SET(-1.6650992E38F) ;
        p231.wind_x_SET(3.3943657E38F) ;
        p231.wind_y_SET(1.5440277E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6930119824814075007L);
            assert(pack.vd_GET() == 1.8790302E38F);
            assert(pack.vn_GET() == -2.6505024E38F);
            assert(pack.gps_id_GET() == (char)191);
            assert(pack.vert_accuracy_GET() == -1.6145581E38F);
            assert(pack.hdop_GET() == 1.0047589E38F);
            assert(pack.satellites_visible_GET() == (char)146);
            assert(pack.vdop_GET() == 3.069644E38F);
            assert(pack.lat_GET() == 142407455);
            assert(pack.time_week_ms_GET() == 888529363L);
            assert(pack.speed_accuracy_GET() == -1.7952636E38F);
            assert(pack.lon_GET() == -81242049);
            assert(pack.alt_GET() == 1.3324122E38F);
            assert(pack.horiz_accuracy_GET() == -1.5320939E38F);
            assert(pack.time_week_GET() == (char)17740);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP));
            assert(pack.ve_GET() == 9.237871E37F);
            assert(pack.fix_type_GET() == (char)80);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.vd_SET(1.8790302E38F) ;
        p232.vn_SET(-2.6505024E38F) ;
        p232.lon_SET(-81242049) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP)) ;
        p232.time_usec_SET(6930119824814075007L) ;
        p232.fix_type_SET((char)80) ;
        p232.time_week_ms_SET(888529363L) ;
        p232.ve_SET(9.237871E37F) ;
        p232.alt_SET(1.3324122E38F) ;
        p232.gps_id_SET((char)191) ;
        p232.hdop_SET(1.0047589E38F) ;
        p232.horiz_accuracy_SET(-1.5320939E38F) ;
        p232.time_week_SET((char)17740) ;
        p232.vert_accuracy_SET(-1.6145581E38F) ;
        p232.satellites_visible_SET((char)146) ;
        p232.vdop_SET(3.069644E38F) ;
        p232.speed_accuracy_SET(-1.7952636E38F) ;
        p232.lat_SET(142407455) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)61, (char)77, (char)68, (char)57, (char)61, (char)107, (char)203, (char)103, (char)143, (char)177, (char)133, (char)54, (char)68, (char)184, (char)95, (char)73, (char)49, (char)183, (char)5, (char)93, (char)97, (char)66, (char)50, (char)49, (char)255, (char)92, (char)38, (char)105, (char)33, (char)184, (char)226, (char)117, (char)160, (char)183, (char)82, (char)93, (char)10, (char)167, (char)138, (char)212, (char)93, (char)103, (char)78, (char)26, (char)0, (char)92, (char)86, (char)158, (char)128, (char)101, (char)23, (char)214, (char)131, (char)162, (char)154, (char)138, (char)155, (char)79, (char)254, (char)111, (char)162, (char)18, (char)159, (char)245, (char)168, (char)62, (char)239, (char)205, (char)71, (char)62, (char)18, (char)122, (char)149, (char)129, (char)229, (char)216, (char)12, (char)185, (char)214, (char)81, (char)79, (char)21, (char)176, (char)179, (char)169, (char)1, (char)66, (char)185, (char)36, (char)240, (char)230, (char)197, (char)247, (char)222, (char)106, (char)185, (char)145, (char)221, (char)13, (char)117, (char)111, (char)7, (char)48, (char)205, (char)125, (char)1, (char)6, (char)20, (char)83, (char)229, (char)195, (char)237, (char)80, (char)68, (char)72, (char)10, (char)17, (char)37, (char)20, (char)38, (char)46, (char)52, (char)100, (char)37, (char)179, (char)201, (char)229, (char)202, (char)84, (char)104, (char)94, (char)158, (char)81, (char)143, (char)191, (char)235, (char)112, (char)30, (char)198, (char)55, (char)71, (char)143, (char)22, (char)94, (char)115, (char)252, (char)107, (char)166, (char)220, (char)99, (char)248, (char)199, (char)65, (char)67, (char)227, (char)89, (char)70, (char)112, (char)194, (char)32, (char)140, (char)121, (char)156, (char)207, (char)53, (char)185, (char)20, (char)196, (char)64, (char)209, (char)252, (char)161, (char)138, (char)202, (char)184, (char)2, (char)100, (char)22, (char)196, (char)238}));
            assert(pack.len_GET() == (char)241);
            assert(pack.flags_GET() == (char)54);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)54) ;
        p233.data__SET(new char[] {(char)61, (char)77, (char)68, (char)57, (char)61, (char)107, (char)203, (char)103, (char)143, (char)177, (char)133, (char)54, (char)68, (char)184, (char)95, (char)73, (char)49, (char)183, (char)5, (char)93, (char)97, (char)66, (char)50, (char)49, (char)255, (char)92, (char)38, (char)105, (char)33, (char)184, (char)226, (char)117, (char)160, (char)183, (char)82, (char)93, (char)10, (char)167, (char)138, (char)212, (char)93, (char)103, (char)78, (char)26, (char)0, (char)92, (char)86, (char)158, (char)128, (char)101, (char)23, (char)214, (char)131, (char)162, (char)154, (char)138, (char)155, (char)79, (char)254, (char)111, (char)162, (char)18, (char)159, (char)245, (char)168, (char)62, (char)239, (char)205, (char)71, (char)62, (char)18, (char)122, (char)149, (char)129, (char)229, (char)216, (char)12, (char)185, (char)214, (char)81, (char)79, (char)21, (char)176, (char)179, (char)169, (char)1, (char)66, (char)185, (char)36, (char)240, (char)230, (char)197, (char)247, (char)222, (char)106, (char)185, (char)145, (char)221, (char)13, (char)117, (char)111, (char)7, (char)48, (char)205, (char)125, (char)1, (char)6, (char)20, (char)83, (char)229, (char)195, (char)237, (char)80, (char)68, (char)72, (char)10, (char)17, (char)37, (char)20, (char)38, (char)46, (char)52, (char)100, (char)37, (char)179, (char)201, (char)229, (char)202, (char)84, (char)104, (char)94, (char)158, (char)81, (char)143, (char)191, (char)235, (char)112, (char)30, (char)198, (char)55, (char)71, (char)143, (char)22, (char)94, (char)115, (char)252, (char)107, (char)166, (char)220, (char)99, (char)248, (char)199, (char)65, (char)67, (char)227, (char)89, (char)70, (char)112, (char)194, (char)32, (char)140, (char)121, (char)156, (char)207, (char)53, (char)185, (char)20, (char)196, (char)64, (char)209, (char)252, (char)161, (char)138, (char)202, (char)184, (char)2, (char)100, (char)22, (char)196, (char)238}, 0) ;
        p233.len_SET((char)241) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.heading_GET() == (char)5432);
            assert(pack.altitude_amsl_GET() == (short) -7981);
            assert(pack.groundspeed_GET() == (char)164);
            assert(pack.longitude_GET() == -1925947038);
            assert(pack.gps_nsat_GET() == (char)137);
            assert(pack.altitude_sp_GET() == (short) -13295);
            assert(pack.pitch_GET() == (short)3205);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            assert(pack.heading_sp_GET() == (short) -30675);
            assert(pack.latitude_GET() == -1317414923);
            assert(pack.custom_mode_GET() == 686683385L);
            assert(pack.climb_rate_GET() == (byte) - 30);
            assert(pack.battery_remaining_GET() == (char)20);
            assert(pack.airspeed_GET() == (char)250);
            assert(pack.wp_distance_GET() == (char)11794);
            assert(pack.temperature_air_GET() == (byte)126);
            assert(pack.failsafe_GET() == (char)233);
            assert(pack.airspeed_sp_GET() == (char)149);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED));
            assert(pack.roll_GET() == (short)1829);
            assert(pack.wp_num_GET() == (char)132);
            assert(pack.temperature_GET() == (byte)100);
            assert(pack.throttle_GET() == (byte)105);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.custom_mode_SET(686683385L) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)) ;
        p234.wp_num_SET((char)132) ;
        p234.heading_sp_SET((short) -30675) ;
        p234.heading_SET((char)5432) ;
        p234.wp_distance_SET((char)11794) ;
        p234.airspeed_sp_SET((char)149) ;
        p234.throttle_SET((byte)105) ;
        p234.gps_nsat_SET((char)137) ;
        p234.altitude_sp_SET((short) -13295) ;
        p234.pitch_SET((short)3205) ;
        p234.longitude_SET(-1925947038) ;
        p234.airspeed_SET((char)250) ;
        p234.failsafe_SET((char)233) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING) ;
        p234.temperature_air_SET((byte)126) ;
        p234.battery_remaining_SET((char)20) ;
        p234.roll_SET((short)1829) ;
        p234.altitude_amsl_SET((short) -7981) ;
        p234.temperature_SET((byte)100) ;
        p234.climb_rate_SET((byte) - 30) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX) ;
        p234.groundspeed_SET((char)164) ;
        p234.latitude_SET(-1317414923) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_y_GET() == -2.377115E38F);
            assert(pack.vibration_z_GET() == 1.7901685E38F);
            assert(pack.time_usec_GET() == 5134909700481780905L);
            assert(pack.vibration_x_GET() == -2.1585723E38F);
            assert(pack.clipping_0_GET() == 2428065226L);
            assert(pack.clipping_2_GET() == 3507108397L);
            assert(pack.clipping_1_GET() == 1910236565L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_2_SET(3507108397L) ;
        p241.vibration_y_SET(-2.377115E38F) ;
        p241.vibration_z_SET(1.7901685E38F) ;
        p241.clipping_0_SET(2428065226L) ;
        p241.time_usec_SET(5134909700481780905L) ;
        p241.vibration_x_SET(-2.1585723E38F) ;
        p241.clipping_1_SET(1910236565L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 3.336547E38F);
            assert(pack.approach_x_GET() == 2.9672597E38F);
            assert(pack.altitude_GET() == 1332346002);
            assert(pack.time_usec_TRY(ph) == 8331281556547515486L);
            assert(pack.longitude_GET() == 1608603290);
            assert(pack.y_GET() == -1.874225E37F);
            assert(pack.approach_z_GET() == 3.3280781E38F);
            assert(pack.latitude_GET() == 1977947754);
            assert(pack.approach_y_GET() == 1.3872213E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {7.0083326E37F, -1.2173238E38F, 2.6481067E38F, 6.531055E37F}));
            assert(pack.z_GET() == 1.2216156E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.altitude_SET(1332346002) ;
        p242.q_SET(new float[] {7.0083326E37F, -1.2173238E38F, 2.6481067E38F, 6.531055E37F}, 0) ;
        p242.time_usec_SET(8331281556547515486L, PH) ;
        p242.z_SET(1.2216156E38F) ;
        p242.approach_z_SET(3.3280781E38F) ;
        p242.y_SET(-1.874225E37F) ;
        p242.approach_y_SET(1.3872213E38F) ;
        p242.approach_x_SET(2.9672597E38F) ;
        p242.longitude_SET(1608603290) ;
        p242.latitude_SET(1977947754) ;
        p242.x_SET(3.336547E38F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_y_GET() == -2.8533148E38F);
            assert(pack.y_GET() == -5.489249E37F);
            assert(pack.x_GET() == -6.122012E37F);
            assert(pack.time_usec_TRY(ph) == 3538387639605296183L);
            assert(pack.longitude_GET() == -1304101524);
            assert(pack.altitude_GET() == 1353282658);
            assert(pack.approach_z_GET() == -2.962228E38F);
            assert(pack.z_GET() == -2.1951139E38F);
            assert(pack.target_system_GET() == (char)136);
            assert(pack.approach_x_GET() == 1.7603176E38F);
            assert(pack.latitude_GET() == 474323633);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.6041597E38F, 2.5143343E38F, -3.3880045E38F, -1.3006154E38F}));
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.time_usec_SET(3538387639605296183L, PH) ;
        p243.target_system_SET((char)136) ;
        p243.altitude_SET(1353282658) ;
        p243.latitude_SET(474323633) ;
        p243.approach_y_SET(-2.8533148E38F) ;
        p243.y_SET(-5.489249E37F) ;
        p243.approach_x_SET(1.7603176E38F) ;
        p243.z_SET(-2.1951139E38F) ;
        p243.approach_z_SET(-2.962228E38F) ;
        p243.longitude_SET(-1304101524) ;
        p243.x_SET(-6.122012E37F) ;
        p243.q_SET(new float[] {-1.6041597E38F, 2.5143343E38F, -3.3880045E38F, -1.3006154E38F}, 0) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == -217928289);
            assert(pack.message_id_GET() == (char)45665);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)45665) ;
        p244.interval_us_SET(-217928289) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.ICAO_address_GET() == 3035474997L);
            assert(pack.squawk_GET() == (char)41737);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE);
            assert(pack.lon_GET() == -660675523);
            assert(pack.heading_GET() == (char)31282);
            assert(pack.tslc_GET() == (char)27);
            assert(pack.hor_velocity_GET() == (char)49249);
            assert(pack.callsign_LEN(ph) == 2);
            assert(pack.callsign_TRY(ph).equals("fx"));
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED));
            assert(pack.lat_GET() == 1650331852);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.altitude_GET() == -1009054006);
            assert(pack.ver_velocity_GET() == (short)32360);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.lat_SET(1650331852) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED)) ;
        p246.altitude_SET(-1009054006) ;
        p246.tslc_SET((char)27) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE) ;
        p246.ver_velocity_SET((short)32360) ;
        p246.squawk_SET((char)41737) ;
        p246.heading_SET((char)31282) ;
        p246.ICAO_address_SET(3035474997L) ;
        p246.lon_SET(-660675523) ;
        p246.callsign_SET("fx", PH) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.hor_velocity_SET((char)49249) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            assert(pack.id_GET() == 2262347098L);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            assert(pack.time_to_minimum_delta_GET() == 1.446704E38F);
            assert(pack.altitude_minimum_delta_GET() == -1.0712907E38F);
            assert(pack.horizontal_minimum_delta_GET() == -8.104209E36F);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.altitude_minimum_delta_SET(-1.0712907E38F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.id_SET(2262347098L) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE) ;
        p247.time_to_minimum_delta_SET(1.446704E38F) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE) ;
        p247.horizontal_minimum_delta_SET(-8.104209E36F) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)62);
            assert(pack.target_system_GET() == (char)162);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)40, (char)78, (char)63, (char)188, (char)150, (char)45, (char)209, (char)50, (char)228, (char)145, (char)19, (char)164, (char)115, (char)241, (char)156, (char)166, (char)87, (char)250, (char)209, (char)142, (char)222, (char)3, (char)254, (char)184, (char)103, (char)120, (char)51, (char)191, (char)95, (char)248, (char)73, (char)124, (char)74, (char)44, (char)34, (char)37, (char)43, (char)227, (char)69, (char)143, (char)72, (char)223, (char)237, (char)243, (char)55, (char)104, (char)69, (char)159, (char)187, (char)236, (char)250, (char)211, (char)37, (char)36, (char)182, (char)183, (char)155, (char)127, (char)235, (char)186, (char)222, (char)144, (char)183, (char)10, (char)75, (char)88, (char)19, (char)196, (char)139, (char)48, (char)124, (char)191, (char)84, (char)85, (char)141, (char)168, (char)68, (char)130, (char)127, (char)245, (char)4, (char)175, (char)151, (char)102, (char)252, (char)252, (char)137, (char)9, (char)159, (char)175, (char)128, (char)63, (char)207, (char)46, (char)30, (char)87, (char)184, (char)107, (char)191, (char)203, (char)221, (char)15, (char)79, (char)223, (char)106, (char)94, (char)20, (char)202, (char)1, (char)62, (char)40, (char)234, (char)57, (char)248, (char)81, (char)174, (char)241, (char)88, (char)238, (char)164, (char)53, (char)205, (char)18, (char)32, (char)74, (char)81, (char)154, (char)64, (char)32, (char)18, (char)200, (char)98, (char)53, (char)162, (char)250, (char)44, (char)92, (char)117, (char)13, (char)120, (char)28, (char)173, (char)10, (char)176, (char)193, (char)50, (char)128, (char)253, (char)62, (char)28, (char)254, (char)57, (char)46, (char)232, (char)193, (char)232, (char)77, (char)121, (char)208, (char)7, (char)140, (char)132, (char)138, (char)165, (char)74, (char)101, (char)221, (char)125, (char)250, (char)199, (char)199, (char)230, (char)116, (char)21, (char)161, (char)255, (char)48, (char)250, (char)8, (char)57, (char)216, (char)213, (char)187, (char)234, (char)230, (char)186, (char)229, (char)72, (char)68, (char)111, (char)243, (char)72, (char)209, (char)148, (char)126, (char)241, (char)80, (char)160, (char)84, (char)79, (char)74, (char)30, (char)180, (char)42, (char)156, (char)64, (char)247, (char)207, (char)182, (char)20, (char)182, (char)36, (char)131, (char)249, (char)46, (char)237, (char)22, (char)169, (char)27, (char)244, (char)219, (char)155, (char)221, (char)62, (char)95, (char)168, (char)164, (char)109, (char)77, (char)188, (char)91, (char)63, (char)130, (char)110, (char)206, (char)203, (char)20, (char)144, (char)252, (char)172, (char)15, (char)159, (char)11, (char)62, (char)21, (char)166, (char)211, (char)6, (char)170}));
            assert(pack.target_component_GET() == (char)185);
            assert(pack.message_type_GET() == (char)6453);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_system_SET((char)162) ;
        p248.payload_SET(new char[] {(char)40, (char)78, (char)63, (char)188, (char)150, (char)45, (char)209, (char)50, (char)228, (char)145, (char)19, (char)164, (char)115, (char)241, (char)156, (char)166, (char)87, (char)250, (char)209, (char)142, (char)222, (char)3, (char)254, (char)184, (char)103, (char)120, (char)51, (char)191, (char)95, (char)248, (char)73, (char)124, (char)74, (char)44, (char)34, (char)37, (char)43, (char)227, (char)69, (char)143, (char)72, (char)223, (char)237, (char)243, (char)55, (char)104, (char)69, (char)159, (char)187, (char)236, (char)250, (char)211, (char)37, (char)36, (char)182, (char)183, (char)155, (char)127, (char)235, (char)186, (char)222, (char)144, (char)183, (char)10, (char)75, (char)88, (char)19, (char)196, (char)139, (char)48, (char)124, (char)191, (char)84, (char)85, (char)141, (char)168, (char)68, (char)130, (char)127, (char)245, (char)4, (char)175, (char)151, (char)102, (char)252, (char)252, (char)137, (char)9, (char)159, (char)175, (char)128, (char)63, (char)207, (char)46, (char)30, (char)87, (char)184, (char)107, (char)191, (char)203, (char)221, (char)15, (char)79, (char)223, (char)106, (char)94, (char)20, (char)202, (char)1, (char)62, (char)40, (char)234, (char)57, (char)248, (char)81, (char)174, (char)241, (char)88, (char)238, (char)164, (char)53, (char)205, (char)18, (char)32, (char)74, (char)81, (char)154, (char)64, (char)32, (char)18, (char)200, (char)98, (char)53, (char)162, (char)250, (char)44, (char)92, (char)117, (char)13, (char)120, (char)28, (char)173, (char)10, (char)176, (char)193, (char)50, (char)128, (char)253, (char)62, (char)28, (char)254, (char)57, (char)46, (char)232, (char)193, (char)232, (char)77, (char)121, (char)208, (char)7, (char)140, (char)132, (char)138, (char)165, (char)74, (char)101, (char)221, (char)125, (char)250, (char)199, (char)199, (char)230, (char)116, (char)21, (char)161, (char)255, (char)48, (char)250, (char)8, (char)57, (char)216, (char)213, (char)187, (char)234, (char)230, (char)186, (char)229, (char)72, (char)68, (char)111, (char)243, (char)72, (char)209, (char)148, (char)126, (char)241, (char)80, (char)160, (char)84, (char)79, (char)74, (char)30, (char)180, (char)42, (char)156, (char)64, (char)247, (char)207, (char)182, (char)20, (char)182, (char)36, (char)131, (char)249, (char)46, (char)237, (char)22, (char)169, (char)27, (char)244, (char)219, (char)155, (char)221, (char)62, (char)95, (char)168, (char)164, (char)109, (char)77, (char)188, (char)91, (char)63, (char)130, (char)110, (char)206, (char)203, (char)20, (char)144, (char)252, (char)172, (char)15, (char)159, (char)11, (char)62, (char)21, (char)166, (char)211, (char)6, (char)170}, 0) ;
        p248.message_type_SET((char)6453) ;
        p248.target_network_SET((char)62) ;
        p248.target_component_SET((char)185) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)42, (byte) - 59, (byte) - 31, (byte)98, (byte) - 33, (byte) - 25, (byte)117, (byte) - 81, (byte)61, (byte) - 89, (byte) - 88, (byte) - 70, (byte)122, (byte)105, (byte)30, (byte) - 45, (byte)37, (byte) - 94, (byte)11, (byte)101, (byte)47, (byte) - 26, (byte) - 67, (byte) - 67, (byte) - 71, (byte) - 104, (byte) - 51, (byte) - 104, (byte)57, (byte) - 62, (byte) - 127, (byte)112}));
            assert(pack.ver_GET() == (char)140);
            assert(pack.address_GET() == (char)25420);
            assert(pack.type_GET() == (char)92);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)25420) ;
        p249.value_SET(new byte[] {(byte)42, (byte) - 59, (byte) - 31, (byte)98, (byte) - 33, (byte) - 25, (byte)117, (byte) - 81, (byte)61, (byte) - 89, (byte) - 88, (byte) - 70, (byte)122, (byte)105, (byte)30, (byte) - 45, (byte)37, (byte) - 94, (byte)11, (byte)101, (byte)47, (byte) - 26, (byte) - 67, (byte) - 67, (byte) - 71, (byte) - 104, (byte) - 51, (byte) - 104, (byte)57, (byte) - 62, (byte) - 127, (byte)112}, 0) ;
        p249.type_SET((char)92) ;
        p249.ver_SET((char)140) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6495196673480414292L);
            assert(pack.name_LEN(ph) == 6);
            assert(pack.name_TRY(ph).equals("fbcVwk"));
            assert(pack.y_GET() == 1.1550993E38F);
            assert(pack.x_GET() == -2.7084495E38F);
            assert(pack.z_GET() == 6.5249257E37F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.y_SET(1.1550993E38F) ;
        p250.time_usec_SET(6495196673480414292L) ;
        p250.name_SET("fbcVwk", PH) ;
        p250.x_SET(-2.7084495E38F) ;
        p250.z_SET(6.5249257E37F) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 1);
            assert(pack.name_TRY(ph).equals("h"));
            assert(pack.time_boot_ms_GET() == 71594614L);
            assert(pack.value_GET() == -2.2427648E38F);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("h", PH) ;
        p251.time_boot_ms_SET(71594614L) ;
        p251.value_SET(-2.2427648E38F) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 1884435502);
            assert(pack.time_boot_ms_GET() == 2298541383L);
            assert(pack.name_LEN(ph) == 1);
            assert(pack.name_TRY(ph).equals("s"));
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.name_SET("s", PH) ;
        p252.time_boot_ms_SET(2298541383L) ;
        p252.value_SET(1884435502) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_ALERT);
            assert(pack.text_LEN(ph) == 15);
            assert(pack.text_TRY(ph).equals("goeGhenDrzzkrkp"));
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("goeGhenDrzzkrkp", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_ALERT) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2350417868L);
            assert(pack.ind_GET() == (char)187);
            assert(pack.value_GET() == 1.7654718E38F);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.value_SET(1.7654718E38F) ;
        p254.time_boot_ms_SET(2350417868L) ;
        p254.ind_SET((char)187) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)124);
            assert(pack.target_component_GET() == (char)77);
            assert(pack.initial_timestamp_GET() == 5790930550281119106L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)56, (char)100, (char)25, (char)135, (char)27, (char)114, (char)107, (char)197, (char)109, (char)116, (char)83, (char)91, (char)38, (char)142, (char)74, (char)193, (char)91, (char)227, (char)242, (char)97, (char)25, (char)181, (char)210, (char)156, (char)220, (char)223, (char)168, (char)39, (char)155, (char)166, (char)173, (char)212}));
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)124) ;
        p256.initial_timestamp_SET(5790930550281119106L) ;
        p256.target_component_SET((char)77) ;
        p256.secret_key_SET(new char[] {(char)56, (char)100, (char)25, (char)135, (char)27, (char)114, (char)107, (char)197, (char)109, (char)116, (char)83, (char)91, (char)38, (char)142, (char)74, (char)193, (char)91, (char)227, (char)242, (char)97, (char)25, (char)181, (char)210, (char)156, (char)220, (char)223, (char)168, (char)39, (char)155, (char)166, (char)173, (char)212}, 0) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 545015312L);
            assert(pack.state_GET() == (char)66);
            assert(pack.last_change_ms_GET() == 2199613049L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.state_SET((char)66) ;
        p257.last_change_ms_SET(2199613049L) ;
        p257.time_boot_ms_SET(545015312L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)129);
            assert(pack.tune_LEN(ph) == 2);
            assert(pack.tune_TRY(ph).equals("fp"));
            assert(pack.target_component_GET() == (char)227);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_component_SET((char)227) ;
        p258.target_system_SET((char)129) ;
        p258.tune_SET("fp", PH) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)55, (char)219, (char)116, (char)119, (char)132, (char)172, (char)33, (char)13, (char)177, (char)129, (char)169, (char)209, (char)67, (char)160, (char)119, (char)73, (char)114, (char)193, (char)138, (char)73, (char)238, (char)129, (char)105, (char)217, (char)124, (char)101, (char)221, (char)233, (char)200, (char)35, (char)139, (char)73}));
            assert(pack.focal_length_GET() == -3.0117273E38F);
            assert(pack.lens_id_GET() == (char)133);
            assert(pack.resolution_h_GET() == (char)16821);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)240, (char)254, (char)32, (char)165, (char)218, (char)15, (char)138, (char)23, (char)214, (char)96, (char)166, (char)80, (char)101, (char)194, (char)135, (char)242, (char)252, (char)113, (char)114, (char)149, (char)125, (char)148, (char)210, (char)222, (char)152, (char)151, (char)115, (char)44, (char)189, (char)6, (char)145, (char)196}));
            assert(pack.firmware_version_GET() == 3051061398L);
            assert(pack.time_boot_ms_GET() == 1881402180L);
            assert(pack.sensor_size_h_GET() == -7.7172784E37F);
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
            assert(pack.cam_definition_version_GET() == (char)48470);
            assert(pack.sensor_size_v_GET() == 7.7938703E37F);
            assert(pack.resolution_v_GET() == (char)9101);
            assert(pack.cam_definition_uri_LEN(ph) == 124);
            assert(pack.cam_definition_uri_TRY(ph).equals("iShxqcrezudbsYRvosjxBelzylFwjimsirewlprjerlobumulkqmpeymxmgdzoihgttrvezvvnufjWbuzGmYkyebtwaeuxgosVorebywgimqcihvvtayonnwmmgg"));
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.sensor_size_v_SET(7.7938703E37F) ;
        p259.vendor_name_SET(new char[] {(char)240, (char)254, (char)32, (char)165, (char)218, (char)15, (char)138, (char)23, (char)214, (char)96, (char)166, (char)80, (char)101, (char)194, (char)135, (char)242, (char)252, (char)113, (char)114, (char)149, (char)125, (char)148, (char)210, (char)222, (char)152, (char)151, (char)115, (char)44, (char)189, (char)6, (char)145, (char)196}, 0) ;
        p259.model_name_SET(new char[] {(char)55, (char)219, (char)116, (char)119, (char)132, (char)172, (char)33, (char)13, (char)177, (char)129, (char)169, (char)209, (char)67, (char)160, (char)119, (char)73, (char)114, (char)193, (char)138, (char)73, (char)238, (char)129, (char)105, (char)217, (char)124, (char)101, (char)221, (char)233, (char)200, (char)35, (char)139, (char)73}, 0) ;
        p259.firmware_version_SET(3051061398L) ;
        p259.lens_id_SET((char)133) ;
        p259.resolution_v_SET((char)9101) ;
        p259.cam_definition_version_SET((char)48470) ;
        p259.resolution_h_SET((char)16821) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE)) ;
        p259.time_boot_ms_SET(1881402180L) ;
        p259.focal_length_SET(-3.0117273E38F) ;
        p259.sensor_size_h_SET(-7.7172784E37F) ;
        p259.cam_definition_uri_SET("iShxqcrezudbsYRvosjxBelzylFwjimsirewlprjerlobumulkqmpeymxmgdzoihgttrvezvvnufjWbuzGmYkyebtwaeuxgosVorebywgimqcihvvtayonnwmmgg", PH) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 8186985L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_VIDEO);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO) ;
        p260.time_boot_ms_SET(8186985L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.total_capacity_GET() == 2.652925E38F);
            assert(pack.available_capacity_GET() == -8.796687E37F);
            assert(pack.status_GET() == (char)186);
            assert(pack.time_boot_ms_GET() == 1283439581L);
            assert(pack.storage_id_GET() == (char)120);
            assert(pack.storage_count_GET() == (char)28);
            assert(pack.used_capacity_GET() == 1.7744802E38F);
            assert(pack.read_speed_GET() == -9.096338E37F);
            assert(pack.write_speed_GET() == 6.945819E37F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.write_speed_SET(6.945819E37F) ;
        p261.read_speed_SET(-9.096338E37F) ;
        p261.time_boot_ms_SET(1283439581L) ;
        p261.available_capacity_SET(-8.796687E37F) ;
        p261.storage_id_SET((char)120) ;
        p261.total_capacity_SET(2.652925E38F) ;
        p261.storage_count_SET((char)28) ;
        p261.status_SET((char)186) ;
        p261.used_capacity_SET(1.7744802E38F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.video_status_GET() == (char)237);
            assert(pack.available_capacity_GET() == -5.4483354E37F);
            assert(pack.image_status_GET() == (char)114);
            assert(pack.time_boot_ms_GET() == 1706426721L);
            assert(pack.image_interval_GET() == 1.4561761E38F);
            assert(pack.recording_time_ms_GET() == 2318015545L);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.available_capacity_SET(-5.4483354E37F) ;
        p262.recording_time_ms_SET(2318015545L) ;
        p262.image_status_SET((char)114) ;
        p262.image_interval_SET(1.4561761E38F) ;
        p262.video_status_SET((char)237) ;
        p262.time_boot_ms_SET(1706426721L) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 1274585225);
            assert(pack.time_boot_ms_GET() == 2399923895L);
            assert(pack.lon_GET() == -678674884);
            assert(pack.file_url_LEN(ph) == 57);
            assert(pack.file_url_TRY(ph).equals("kedwqkrokXijwrggxmvyDkZJdzpbpgqzwatpjsdqyxcpmznrrladwoeot"));
            assert(pack.relative_alt_GET() == 1937930375);
            assert(pack.lat_GET() == -1975526980);
            assert(pack.camera_id_GET() == (char)8);
            assert(pack.capture_result_GET() == (byte)35);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.5445166E38F, -1.8722796E38F, -1.2948174E38F, 2.6030081E38F}));
            assert(pack.image_index_GET() == 705507077);
            assert(pack.time_utc_GET() == 1110157010323244625L);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.capture_result_SET((byte)35) ;
        p263.time_utc_SET(1110157010323244625L) ;
        p263.time_boot_ms_SET(2399923895L) ;
        p263.alt_SET(1274585225) ;
        p263.camera_id_SET((char)8) ;
        p263.lon_SET(-678674884) ;
        p263.image_index_SET(705507077) ;
        p263.q_SET(new float[] {-1.5445166E38F, -1.8722796E38F, -1.2948174E38F, 2.6030081E38F}, 0) ;
        p263.lat_SET(-1975526980) ;
        p263.file_url_SET("kedwqkrokXijwrggxmvyDkZJdzpbpgqzwatpjsdqyxcpmznrrladwoeot", PH) ;
        p263.relative_alt_SET(1937930375) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1092488194L);
            assert(pack.flight_uuid_GET() == 6646321895898224797L);
            assert(pack.takeoff_time_utc_GET() == 3806265106280372517L);
            assert(pack.arming_time_utc_GET() == 828128460735067352L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.flight_uuid_SET(6646321895898224797L) ;
        p264.takeoff_time_utc_SET(3806265106280372517L) ;
        p264.arming_time_utc_SET(828128460735067352L) ;
        p264.time_boot_ms_SET(1092488194L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -2.8611893E37F);
            assert(pack.time_boot_ms_GET() == 4069843212L);
            assert(pack.yaw_GET() == -2.2368938E38F);
            assert(pack.pitch_GET() == -1.1784061E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.pitch_SET(-1.1784061E38F) ;
        p265.yaw_SET(-2.2368938E38F) ;
        p265.time_boot_ms_SET(4069843212L) ;
        p265.roll_SET(-2.8611893E37F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.length_GET() == (char)251);
            assert(pack.target_component_GET() == (char)71);
            assert(pack.target_system_GET() == (char)242);
            assert(pack.sequence_GET() == (char)23720);
            assert(pack.first_message_offset_GET() == (char)19);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)240, (char)23, (char)218, (char)169, (char)206, (char)151, (char)6, (char)121, (char)164, (char)112, (char)251, (char)57, (char)176, (char)233, (char)49, (char)157, (char)133, (char)185, (char)242, (char)199, (char)64, (char)130, (char)240, (char)99, (char)107, (char)193, (char)173, (char)75, (char)141, (char)72, (char)105, (char)58, (char)143, (char)70, (char)11, (char)97, (char)185, (char)50, (char)14, (char)98, (char)212, (char)92, (char)216, (char)83, (char)28, (char)175, (char)108, (char)229, (char)4, (char)239, (char)199, (char)130, (char)173, (char)80, (char)253, (char)239, (char)170, (char)247, (char)250, (char)100, (char)203, (char)53, (char)34, (char)185, (char)21, (char)39, (char)215, (char)60, (char)74, (char)130, (char)25, (char)147, (char)151, (char)201, (char)55, (char)16, (char)3, (char)92, (char)91, (char)41, (char)104, (char)102, (char)43, (char)198, (char)101, (char)200, (char)27, (char)15, (char)207, (char)207, (char)5, (char)248, (char)83, (char)231, (char)115, (char)231, (char)51, (char)166, (char)234, (char)102, (char)136, (char)11, (char)83, (char)178, (char)27, (char)113, (char)5, (char)26, (char)29, (char)169, (char)225, (char)183, (char)80, (char)77, (char)51, (char)25, (char)23, (char)61, (char)142, (char)57, (char)123, (char)252, (char)57, (char)45, (char)207, (char)213, (char)167, (char)4, (char)70, (char)233, (char)192, (char)201, (char)139, (char)240, (char)4, (char)169, (char)230, (char)247, (char)115, (char)207, (char)195, (char)225, (char)69, (char)245, (char)216, (char)144, (char)50, (char)226, (char)69, (char)119, (char)166, (char)122, (char)102, (char)252, (char)102, (char)127, (char)220, (char)140, (char)143, (char)188, (char)51, (char)94, (char)161, (char)190, (char)227, (char)102, (char)199, (char)86, (char)67, (char)234, (char)157, (char)36, (char)10, (char)67, (char)128, (char)49, (char)50, (char)210, (char)29, (char)255, (char)237, (char)104, (char)2, (char)255, (char)0, (char)95, (char)199, (char)109, (char)56, (char)224, (char)47, (char)38, (char)207, (char)218, (char)191, (char)64, (char)214, (char)53, (char)67, (char)75, (char)31, (char)55, (char)67, (char)174, (char)55, (char)194, (char)84, (char)86, (char)203, (char)2, (char)189, (char)68, (char)28, (char)218, (char)248, (char)126, (char)177, (char)214, (char)164, (char)78, (char)184, (char)47, (char)111, (char)171, (char)22, (char)23, (char)214, (char)98, (char)50, (char)62, (char)217, (char)222, (char)187, (char)65, (char)108, (char)245, (char)251, (char)74, (char)151, (char)82, (char)126, (char)149, (char)254, (char)212, (char)249, (char)133, (char)201, (char)9, (char)47}));
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)242) ;
        p266.first_message_offset_SET((char)19) ;
        p266.length_SET((char)251) ;
        p266.sequence_SET((char)23720) ;
        p266.target_component_SET((char)71) ;
        p266.data__SET(new char[] {(char)240, (char)23, (char)218, (char)169, (char)206, (char)151, (char)6, (char)121, (char)164, (char)112, (char)251, (char)57, (char)176, (char)233, (char)49, (char)157, (char)133, (char)185, (char)242, (char)199, (char)64, (char)130, (char)240, (char)99, (char)107, (char)193, (char)173, (char)75, (char)141, (char)72, (char)105, (char)58, (char)143, (char)70, (char)11, (char)97, (char)185, (char)50, (char)14, (char)98, (char)212, (char)92, (char)216, (char)83, (char)28, (char)175, (char)108, (char)229, (char)4, (char)239, (char)199, (char)130, (char)173, (char)80, (char)253, (char)239, (char)170, (char)247, (char)250, (char)100, (char)203, (char)53, (char)34, (char)185, (char)21, (char)39, (char)215, (char)60, (char)74, (char)130, (char)25, (char)147, (char)151, (char)201, (char)55, (char)16, (char)3, (char)92, (char)91, (char)41, (char)104, (char)102, (char)43, (char)198, (char)101, (char)200, (char)27, (char)15, (char)207, (char)207, (char)5, (char)248, (char)83, (char)231, (char)115, (char)231, (char)51, (char)166, (char)234, (char)102, (char)136, (char)11, (char)83, (char)178, (char)27, (char)113, (char)5, (char)26, (char)29, (char)169, (char)225, (char)183, (char)80, (char)77, (char)51, (char)25, (char)23, (char)61, (char)142, (char)57, (char)123, (char)252, (char)57, (char)45, (char)207, (char)213, (char)167, (char)4, (char)70, (char)233, (char)192, (char)201, (char)139, (char)240, (char)4, (char)169, (char)230, (char)247, (char)115, (char)207, (char)195, (char)225, (char)69, (char)245, (char)216, (char)144, (char)50, (char)226, (char)69, (char)119, (char)166, (char)122, (char)102, (char)252, (char)102, (char)127, (char)220, (char)140, (char)143, (char)188, (char)51, (char)94, (char)161, (char)190, (char)227, (char)102, (char)199, (char)86, (char)67, (char)234, (char)157, (char)36, (char)10, (char)67, (char)128, (char)49, (char)50, (char)210, (char)29, (char)255, (char)237, (char)104, (char)2, (char)255, (char)0, (char)95, (char)199, (char)109, (char)56, (char)224, (char)47, (char)38, (char)207, (char)218, (char)191, (char)64, (char)214, (char)53, (char)67, (char)75, (char)31, (char)55, (char)67, (char)174, (char)55, (char)194, (char)84, (char)86, (char)203, (char)2, (char)189, (char)68, (char)28, (char)218, (char)248, (char)126, (char)177, (char)214, (char)164, (char)78, (char)184, (char)47, (char)111, (char)171, (char)22, (char)23, (char)214, (char)98, (char)50, (char)62, (char)217, (char)222, (char)187, (char)65, (char)108, (char)245, (char)251, (char)74, (char)151, (char)82, (char)126, (char)149, (char)254, (char)212, (char)249, (char)133, (char)201, (char)9, (char)47}, 0) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.length_GET() == (char)7);
            assert(pack.sequence_GET() == (char)38657);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)20, (char)103, (char)229, (char)159, (char)61, (char)254, (char)141, (char)92, (char)226, (char)64, (char)91, (char)144, (char)100, (char)7, (char)163, (char)68, (char)76, (char)83, (char)213, (char)70, (char)129, (char)105, (char)164, (char)113, (char)167, (char)20, (char)215, (char)88, (char)56, (char)131, (char)190, (char)115, (char)94, (char)88, (char)26, (char)244, (char)189, (char)133, (char)215, (char)127, (char)90, (char)142, (char)158, (char)213, (char)22, (char)250, (char)35, (char)180, (char)246, (char)3, (char)192, (char)114, (char)223, (char)53, (char)122, (char)252, (char)129, (char)205, (char)142, (char)110, (char)176, (char)197, (char)204, (char)181, (char)221, (char)105, (char)33, (char)243, (char)101, (char)178, (char)196, (char)22, (char)142, (char)97, (char)93, (char)107, (char)197, (char)63, (char)171, (char)83, (char)19, (char)215, (char)247, (char)64, (char)231, (char)75, (char)121, (char)129, (char)110, (char)186, (char)177, (char)77, (char)199, (char)45, (char)159, (char)29, (char)42, (char)45, (char)128, (char)17, (char)58, (char)184, (char)144, (char)138, (char)52, (char)237, (char)32, (char)71, (char)169, (char)14, (char)1, (char)81, (char)108, (char)230, (char)96, (char)110, (char)245, (char)14, (char)22, (char)166, (char)6, (char)187, (char)251, (char)85, (char)15, (char)169, (char)91, (char)41, (char)14, (char)87, (char)218, (char)51, (char)176, (char)172, (char)103, (char)24, (char)45, (char)254, (char)24, (char)181, (char)16, (char)177, (char)237, (char)215, (char)174, (char)250, (char)65, (char)226, (char)44, (char)168, (char)58, (char)162, (char)6, (char)170, (char)46, (char)211, (char)81, (char)223, (char)171, (char)171, (char)64, (char)161, (char)164, (char)47, (char)134, (char)241, (char)177, (char)56, (char)49, (char)115, (char)209, (char)240, (char)184, (char)115, (char)15, (char)170, (char)152, (char)12, (char)96, (char)145, (char)218, (char)78, (char)175, (char)231, (char)178, (char)142, (char)55, (char)201, (char)230, (char)207, (char)39, (char)88, (char)199, (char)85, (char)81, (char)80, (char)72, (char)175, (char)148, (char)19, (char)248, (char)145, (char)113, (char)31, (char)234, (char)29, (char)31, (char)233, (char)4, (char)6, (char)36, (char)198, (char)38, (char)150, (char)131, (char)193, (char)189, (char)15, (char)54, (char)12, (char)94, (char)83, (char)192, (char)164, (char)245, (char)66, (char)155, (char)32, (char)94, (char)90, (char)22, (char)116, (char)182, (char)162, (char)149, (char)67, (char)114, (char)184, (char)166, (char)255, (char)47, (char)49, (char)133, (char)82, (char)56, (char)49, (char)98, (char)171, (char)166}));
            assert(pack.first_message_offset_GET() == (char)161);
            assert(pack.target_component_GET() == (char)30);
            assert(pack.target_system_GET() == (char)33);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.data__SET(new char[] {(char)20, (char)103, (char)229, (char)159, (char)61, (char)254, (char)141, (char)92, (char)226, (char)64, (char)91, (char)144, (char)100, (char)7, (char)163, (char)68, (char)76, (char)83, (char)213, (char)70, (char)129, (char)105, (char)164, (char)113, (char)167, (char)20, (char)215, (char)88, (char)56, (char)131, (char)190, (char)115, (char)94, (char)88, (char)26, (char)244, (char)189, (char)133, (char)215, (char)127, (char)90, (char)142, (char)158, (char)213, (char)22, (char)250, (char)35, (char)180, (char)246, (char)3, (char)192, (char)114, (char)223, (char)53, (char)122, (char)252, (char)129, (char)205, (char)142, (char)110, (char)176, (char)197, (char)204, (char)181, (char)221, (char)105, (char)33, (char)243, (char)101, (char)178, (char)196, (char)22, (char)142, (char)97, (char)93, (char)107, (char)197, (char)63, (char)171, (char)83, (char)19, (char)215, (char)247, (char)64, (char)231, (char)75, (char)121, (char)129, (char)110, (char)186, (char)177, (char)77, (char)199, (char)45, (char)159, (char)29, (char)42, (char)45, (char)128, (char)17, (char)58, (char)184, (char)144, (char)138, (char)52, (char)237, (char)32, (char)71, (char)169, (char)14, (char)1, (char)81, (char)108, (char)230, (char)96, (char)110, (char)245, (char)14, (char)22, (char)166, (char)6, (char)187, (char)251, (char)85, (char)15, (char)169, (char)91, (char)41, (char)14, (char)87, (char)218, (char)51, (char)176, (char)172, (char)103, (char)24, (char)45, (char)254, (char)24, (char)181, (char)16, (char)177, (char)237, (char)215, (char)174, (char)250, (char)65, (char)226, (char)44, (char)168, (char)58, (char)162, (char)6, (char)170, (char)46, (char)211, (char)81, (char)223, (char)171, (char)171, (char)64, (char)161, (char)164, (char)47, (char)134, (char)241, (char)177, (char)56, (char)49, (char)115, (char)209, (char)240, (char)184, (char)115, (char)15, (char)170, (char)152, (char)12, (char)96, (char)145, (char)218, (char)78, (char)175, (char)231, (char)178, (char)142, (char)55, (char)201, (char)230, (char)207, (char)39, (char)88, (char)199, (char)85, (char)81, (char)80, (char)72, (char)175, (char)148, (char)19, (char)248, (char)145, (char)113, (char)31, (char)234, (char)29, (char)31, (char)233, (char)4, (char)6, (char)36, (char)198, (char)38, (char)150, (char)131, (char)193, (char)189, (char)15, (char)54, (char)12, (char)94, (char)83, (char)192, (char)164, (char)245, (char)66, (char)155, (char)32, (char)94, (char)90, (char)22, (char)116, (char)182, (char)162, (char)149, (char)67, (char)114, (char)184, (char)166, (char)255, (char)47, (char)49, (char)133, (char)82, (char)56, (char)49, (char)98, (char)171, (char)166}, 0) ;
        p267.sequence_SET((char)38657) ;
        p267.target_system_SET((char)33) ;
        p267.first_message_offset_SET((char)161) ;
        p267.length_SET((char)7) ;
        p267.target_component_SET((char)30) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)43);
            assert(pack.target_component_GET() == (char)0);
            assert(pack.sequence_GET() == (char)64458);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.sequence_SET((char)64458) ;
        p268.target_system_SET((char)43) ;
        p268.target_component_SET((char)0) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.resolution_v_GET() == (char)51874);
            assert(pack.camera_id_GET() == (char)4);
            assert(pack.bitrate_GET() == 3230914315L);
            assert(pack.rotation_GET() == (char)39118);
            assert(pack.uri_LEN(ph) == 165);
            assert(pack.uri_TRY(ph).equals("NcZncaeoazwclkIusklulmheuazsvfoLdsAknlqalPrcvxfjevtmebiwgprervttmpPhghsjcywfzEjsvdemxcoakhMoAjmfsqjlkSZbkaonutuiwccbolsrcxdnVaNptZDlifnwagbndyiaasnfgwazpwOEwkUjrhwhk"));
            assert(pack.framerate_GET() == 1.130617E38F);
            assert(pack.status_GET() == (char)197);
            assert(pack.resolution_h_GET() == (char)42691);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.uri_SET("NcZncaeoazwclkIusklulmheuazsvfoLdsAknlqalPrcvxfjevtmebiwgprervttmpPhghsjcywfzEjsvdemxcoakhMoAjmfsqjlkSZbkaonutuiwccbolsrcxdnVaNptZDlifnwagbndyiaasnfgwazpwOEwkUjrhwhk", PH) ;
        p269.resolution_v_SET((char)51874) ;
        p269.resolution_h_SET((char)42691) ;
        p269.camera_id_SET((char)4) ;
        p269.status_SET((char)197) ;
        p269.framerate_SET(1.130617E38F) ;
        p269.rotation_SET((char)39118) ;
        p269.bitrate_SET(3230914315L) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.camera_id_GET() == (char)225);
            assert(pack.rotation_GET() == (char)56237);
            assert(pack.resolution_v_GET() == (char)17786);
            assert(pack.uri_LEN(ph) == 47);
            assert(pack.uri_TRY(ph).equals("cVfoindzvqxjxqtklospiggtxhhackypojRicknbmojInjV"));
            assert(pack.target_component_GET() == (char)222);
            assert(pack.framerate_GET() == 5.136567E37F);
            assert(pack.resolution_h_GET() == (char)41317);
            assert(pack.target_system_GET() == (char)19);
            assert(pack.bitrate_GET() == 2030691789L);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.resolution_v_SET((char)17786) ;
        p270.target_system_SET((char)19) ;
        p270.camera_id_SET((char)225) ;
        p270.target_component_SET((char)222) ;
        p270.framerate_SET(5.136567E37F) ;
        p270.bitrate_SET(2030691789L) ;
        p270.resolution_h_SET((char)41317) ;
        p270.rotation_SET((char)56237) ;
        p270.uri_SET("cVfoindzvqxjxqtklospiggtxhhackypojRicknbmojInjV", PH) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 29);
            assert(pack.ssid_TRY(ph).equals("MvxnxcJrgvtmiihuPwufalkPyarVg"));
            assert(pack.password_LEN(ph) == 5);
            assert(pack.password_TRY(ph).equals("qurrf"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("MvxnxcJrgvtmiihuPwufalkPyarVg", PH) ;
        p299.password_SET("qurrf", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)221, (char)32, (char)94, (char)223, (char)225, (char)61, (char)43, (char)125}));
            assert(pack.version_GET() == (char)49636);
            assert(pack.max_version_GET() == (char)31838);
            assert(pack.min_version_GET() == (char)34394);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)175, (char)94, (char)43, (char)111, (char)236, (char)250, (char)166, (char)233}));
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.min_version_SET((char)34394) ;
        p300.library_version_hash_SET(new char[] {(char)175, (char)94, (char)43, (char)111, (char)236, (char)250, (char)166, (char)233}, 0) ;
        p300.spec_version_hash_SET(new char[] {(char)221, (char)32, (char)94, (char)223, (char)225, (char)61, (char)43, (char)125}, 0) ;
        p300.version_SET((char)49636) ;
        p300.max_version_SET((char)31838) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
            assert(pack.time_usec_GET() == 347486908393858092L);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
            assert(pack.uptime_sec_GET() == 1229574376L);
            assert(pack.vendor_specific_status_code_GET() == (char)60693);
            assert(pack.sub_mode_GET() == (char)214);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.vendor_specific_status_code_SET((char)60693) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR) ;
        p310.time_usec_SET(347486908393858092L) ;
        p310.uptime_sec_SET(1229574376L) ;
        p310.sub_mode_SET((char)214) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.sw_version_major_GET() == (char)43);
            assert(pack.hw_version_major_GET() == (char)44);
            assert(pack.time_usec_GET() == 7118626631266325660L);
            assert(pack.uptime_sec_GET() == 3654870827L);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)241, (char)130, (char)127, (char)68, (char)191, (char)255, (char)129, (char)147, (char)63, (char)109, (char)117, (char)165, (char)172, (char)31, (char)53, (char)242}));
            assert(pack.sw_version_minor_GET() == (char)251);
            assert(pack.name_LEN(ph) == 12);
            assert(pack.name_TRY(ph).equals("ywxjullyajfe"));
            assert(pack.hw_version_minor_GET() == (char)130);
            assert(pack.sw_vcs_commit_GET() == 1641388781L);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.sw_version_minor_SET((char)251) ;
        p311.name_SET("ywxjullyajfe", PH) ;
        p311.time_usec_SET(7118626631266325660L) ;
        p311.hw_version_minor_SET((char)130) ;
        p311.uptime_sec_SET(3654870827L) ;
        p311.hw_unique_id_SET(new char[] {(char)241, (char)130, (char)127, (char)68, (char)191, (char)255, (char)129, (char)147, (char)63, (char)109, (char)117, (char)165, (char)172, (char)31, (char)53, (char)242}, 0) ;
        p311.sw_version_major_SET((char)43) ;
        p311.sw_vcs_commit_SET(1641388781L) ;
        p311.hw_version_major_SET((char)44) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("uoiwcvfhp"));
            assert(pack.target_component_GET() == (char)221);
            assert(pack.target_system_GET() == (char)119);
            assert(pack.param_index_GET() == (short)4881);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)119) ;
        p320.param_index_SET((short)4881) ;
        p320.param_id_SET("uoiwcvfhp", PH) ;
        p320.target_component_SET((char)221) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)253);
            assert(pack.target_component_GET() == (char)229);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)229) ;
        p321.target_system_SET((char)253) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("omdgjpfnptv"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
            assert(pack.param_count_GET() == (char)5487);
            assert(pack.param_index_GET() == (char)30044);
            assert(pack.param_value_LEN(ph) == 16);
            assert(pack.param_value_TRY(ph).equals("xnqEpgutiopizlkx"));
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8) ;
        p322.param_index_SET((char)30044) ;
        p322.param_count_SET((char)5487) ;
        p322.param_value_SET("xnqEpgutiopizlkx", PH) ;
        p322.param_id_SET("omdgjpfnptv", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 19);
            assert(pack.param_value_TRY(ph).equals("fmnyxpjHzafkQhiiAoq"));
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("KwpfrWmoyo"));
            assert(pack.target_system_GET() == (char)89);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
            assert(pack.target_component_GET() == (char)119);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_id_SET("KwpfrWmoyo", PH) ;
        p323.target_component_SET((char)119) ;
        p323.param_value_SET("fmnyxpjHzafkQhiiAoq", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64) ;
        p323.target_system_SET((char)89) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_ACCEPTED);
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("tXtrxdnobfq"));
            assert(pack.param_value_LEN(ph) == 79);
            assert(pack.param_value_TRY(ph).equals("LztturmFLryvnRefwvhozhaedofaypznpfhQgiveofgFizrLqkBmwikdkomiqkMtznrijzsdxpCroVJ"));
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("tXtrxdnobfq", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM) ;
        p324.param_value_SET("LztturmFLryvnRefwvhozhaedofaypznpfhQgiveofgFizrLqkBmwikdkomiqkMtznrijzsdxpCroVJ", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_ACCEPTED) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.min_distance_GET() == (char)53399);
            assert(pack.time_usec_GET() == 7090526815652299524L);
            assert(pack.increment_GET() == (char)186);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.max_distance_GET() == (char)59083);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)26109, (char)9580, (char)56141, (char)37540, (char)35988, (char)41139, (char)18727, (char)65414, (char)40404, (char)52471, (char)15941, (char)26789, (char)5219, (char)56691, (char)56804, (char)64785, (char)22387, (char)54179, (char)18966, (char)44377, (char)25716, (char)7189, (char)56018, (char)5570, (char)58553, (char)3694, (char)34112, (char)48165, (char)13851, (char)59877, (char)16302, (char)10695, (char)3818, (char)20767, (char)23199, (char)30889, (char)56949, (char)20445, (char)27405, (char)49087, (char)49255, (char)52924, (char)48832, (char)39418, (char)663, (char)57630, (char)32613, (char)27576, (char)16151, (char)48076, (char)37594, (char)26101, (char)59044, (char)27326, (char)45391, (char)6106, (char)41199, (char)42919, (char)31425, (char)41367, (char)60421, (char)37292, (char)29948, (char)39379, (char)44998, (char)23192, (char)41447, (char)61925, (char)46714, (char)3784, (char)43627, (char)31657}));
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.distances_SET(new char[] {(char)26109, (char)9580, (char)56141, (char)37540, (char)35988, (char)41139, (char)18727, (char)65414, (char)40404, (char)52471, (char)15941, (char)26789, (char)5219, (char)56691, (char)56804, (char)64785, (char)22387, (char)54179, (char)18966, (char)44377, (char)25716, (char)7189, (char)56018, (char)5570, (char)58553, (char)3694, (char)34112, (char)48165, (char)13851, (char)59877, (char)16302, (char)10695, (char)3818, (char)20767, (char)23199, (char)30889, (char)56949, (char)20445, (char)27405, (char)49087, (char)49255, (char)52924, (char)48832, (char)39418, (char)663, (char)57630, (char)32613, (char)27576, (char)16151, (char)48076, (char)37594, (char)26101, (char)59044, (char)27326, (char)45391, (char)6106, (char)41199, (char)42919, (char)31425, (char)41367, (char)60421, (char)37292, (char)29948, (char)39379, (char)44998, (char)23192, (char)41447, (char)61925, (char)46714, (char)3784, (char)43627, (char)31657}, 0) ;
        p330.time_usec_SET(7090526815652299524L) ;
        p330.min_distance_SET((char)53399) ;
        p330.max_distance_SET((char)59083) ;
        p330.increment_SET((char)186) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
    }

}

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
        *	 (packets that were corrupted on reception on the MAV*/
        public void drop_rate_comm_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        /**
        *Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
        *	 on reception on the MAV*/
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
        *	 present. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
        public void onboard_control_sensors_present_SET(@MAV_SYS_STATUS_SENSOR int  src)
        {  set_bits(- 1 +   src, 26, data, 152); }
        /**
        *Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of
        *	 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
        public void onboard_control_sensors_enabled_SET(@MAV_SYS_STATUS_SENSOR int  src)
        {  set_bits(- 1 +   src, 26, data, 178); }
        /**
        *Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not
        *	 enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
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
        *	 the system id of the requesting syste*/
        public void target_system_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  12); }
        /**
        *0: request ping from all receiving components, if greater than 0: message is a ping response and number
        *	 is the system id of the requesting syste*/
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
        *	 the safest mode possible initially and then gradually move down the encryption level if it gets a NACK
        *	 message indicating an encryption mismatch*/
        public void version_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        /**
        *Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
        *	 characters may involve A-Z, a-z, 0-9, and "!?,.-*/
        public void passkey_SET(String src, Bounds.Inside ph)
        {passkey_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
*	 characters may involve A-Z, a-z, 0-9, and "!?,.-*/
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
        *	 contro*/
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
            long id = id__K(src);
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
        *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	 storage if the ID is stored as strin*/
        public void param_id_SET(String src, Bounds.Inside ph)
        {param_id_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	 storage if the ID is stored as strin*/
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
        *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	 storage if the ID is stored as strin*/
        public void param_id_SET(String src, Bounds.Inside ph)
        {param_id_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	 storage if the ID is stored as strin*/
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
        *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	 storage if the ID is stored as strin*/
        public void param_id_SET(String src, Bounds.Inside ph)
        {param_id_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	 storage if the ID is stored as strin*/
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
        *	 unknown, set to: UINT16_MA*/
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
        *	 the AMSL altitude in addition to the WGS84 altitude*/
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
        *	 provide the AMSL as well*/
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
        *	 8 servos*/
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
        *	 8 servos*/
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
        *	 more than 8 servos*/
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
        *	 send -2 to disable any existing map for this rc_channel_index*/
        public void param_index_SET(short  src)
        {  set_bytes((short)(src) & -1L, 2, data,  2); }
        /**
        *Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob
        *	 on the RC*/
        public void parameter_rc_channel_index_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void param_value0_SET(float  src) //Initial parameter value
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 5); }
        public void scale_SET(float  src) //Scale, maps the RC range [-1, 1] to a parameter value
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 9); }
        /**
        *Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends
        *	 on implementation*/
        public void param_value_min_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 13); }
        /**
        *Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends
        *	 on implementation*/
        public void param_value_max_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 17); }
        /**
        *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
        *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	 storage if the ID is stored as strin*/
        public void param_id_SET(String src, Bounds.Inside ph)
        {param_id_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	 storage if the ID is stored as strin*/
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
        *	 with Z axis up or local, right handed, Z axis down*/
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
        *	 with Z axis up or local, right handed, Z axis down*/
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
        *	 the second row, etc.*/
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
        *	 are available but not given in this message. This value should be 0 when no RC channels are available*/
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
        *	 bit corresponds to Button 1*/
        public void buttons_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void target_SET(char  src) //The system to be controlled.
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        /**
        *X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
        *	 Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle*/
        public void x_SET(short  src)
        {  set_bytes((short)(src) & -1L, 2, data,  3); }
        /**
        *Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
        *	 Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle*/
        public void y_SET(short  src)
        {  set_bytes((short)(src) & -1L, 2, data,  5); }
        /**
        *Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
        *	 Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on
        *	 a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
        *	 thrust*/
        public void z_SET(short  src)
        {  set_bytes((short)(src) & -1L, 2, data,  7); }
        /**
        *R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
        *	 Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise
        *	 being -1000, and the yaw of a vehicle*/
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
        *	 sequence (0,1,2,3,4)*/
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
        *	 should have the UINT16_MAX value*/
        public char[] voltages_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 0, dst_max = pos + 10; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        /**
        *Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
        *	 should have the UINT16_MAX value*/
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
        *	 energy consumption estimat*/
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
        *	 should allow to identify the commit using the main version number even for very large code bases*/
        public char[] flight_custom_version_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 28, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
        *	 should allow to identify the commit using the main version number even for very large code bases*/
        public char[] flight_custom_version_GET()
        {return flight_custom_version_GET(new char[8], 0);}/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	 should allow to identify the commit using the main version number even for very large code bases*/
        public char[] middleware_custom_version_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 36, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
        *	 should allow to identify the commit using the main version number even for very large code bases*/
        public char[] middleware_custom_version_GET()
        {return middleware_custom_version_GET(new char[8], 0);}/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	 should allow to identify the commit using the main version number even for very large code bases*/
        public char[] os_custom_version_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 44, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
        *	 should allow to identify the commit using the main version number even for very large code bases*/
        public char[] os_custom_version_GET()
        {return os_custom_version_GET(new char[8], 0);} public @MAV_PROTOCOL_CAPABILITY int capabilities_GET()//bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
        {  return  1 + (int)get_bits(data, 416, 17); }
        /**
        *UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
        *	 use uid*/
        public char[]  uid2_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  433 && !try_visit_field(ph, 433)) return null;
            return uid2_GET(ph, new char[ph.items], 0);
        }
        /**
        *UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
        *	 use uid*/
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
*	 the landing targe*/
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
        *	 the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
        *	 on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
        *	 while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
        *	 fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
        *	 with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
        *	 corrupt RTCM data, and to recover from a unreliable transport delivery order*/
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
        *	 bit3:GCS, bit4:fence*/
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
        *	 and slope of the groun*/
        public float[] q_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 24, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        /**
        *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
        *	 and slope of the groun*/
        public float[] q_GET()
        {return q_GET(new float[4], 0);}/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	 from the threshold / touchdown zone*/
        public float approach_x_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        /**
        *Local Y position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
        public float approach_y_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        /**
        *Local Z position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
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
        *	 and slope of the groun*/
        public float[] q_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 25, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        /**
        *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
        *	 and slope of the groun*/
        public float[] q_GET()
        {return q_GET(new float[4], 0);}/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	 from the threshold / touchdown zone*/
        public float approach_x_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  41, 4))); }
        /**
        *Local Y position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
        public float approach_y_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  45, 4))); }
        /**
        *Local Z position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
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
        {  return  0 + (int)get_bits(data, 132, 3); }
    }
    public static class V2_EXTENSION extends GroundControl.V2_EXTENSION
    {
        /**
        *A code that identifies the software component that understands this message (analogous to usb device classes
        *	 or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
        *	 and the corresponding entry should be added to https:github.com/mavlink/mavlink/extension-message-ids.xml.
        *	 Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
        *	 Message_types greater than 32767 are considered local experiments and should not be checked in to any
        *	 widely distributed codebase*/
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
        *	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
        *	 message_type.  The particular encoding used can be extension specific and might not always be documented
        *	 as part of the mavlink specification*/
        public char[] payload_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 5, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *Variable length payload. The length is defined by the remaining message length when subtracting the header
        *	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
        *	 message_type.  The particular encoding used can be extension specific and might not always be documented
        *	 as part of the mavlink specification*/
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
        {  return  0 + (int)get_bits(data, 32, 3); }
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
        *	 set and capture in progress*/
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
        *	 lost (set to 255 if no start exists)*/
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
        *	 lost (set to 255 if no start exists)*/
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
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
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
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  38 && !try_visit_field(ph, 38)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
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
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  22 && !try_visit_field(ph, 22)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
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
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  8 && !try_visit_field(ph, 8)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
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
        *	 is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
        *	 for unknown/not used. In a array element, each unit corresponds to 1cm*/
        public char[] distances_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 0, dst_max = pos + 72; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        /**
        *Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
        *	 is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
        *	 for unknown/not used. In a array element, each unit corresponds to 1cm*/
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
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_POWEROFF);
            assert(pack.custom_mode_GET() == 3473269460L);
            assert(pack.mavlink_version_GET() == (char)102);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_FP);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_ANTENNA_TRACKER);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_POWEROFF) ;
        p0.mavlink_version_SET((char)102) ;
        p0.custom_mode_SET(3473269460L) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_FP) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_ANTENNA_TRACKER) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
            assert(pack.errors_count1_GET() == (char)23454);
            assert(pack.voltage_battery_GET() == (char)42157);
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.drop_rate_comm_GET() == (char)52932);
            assert(pack.errors_count2_GET() == (char)21936);
            assert(pack.load_GET() == (char)13562);
            assert(pack.errors_count3_GET() == (char)49874);
            assert(pack.errors_comm_GET() == (char)40934);
            assert(pack.current_battery_GET() == (short)3328);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION));
            assert(pack.battery_remaining_GET() == (byte)56);
            assert(pack.errors_count4_GET() == (char)36146);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION)) ;
        p1.errors_comm_SET((char)40934) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.current_battery_SET((short)3328) ;
        p1.errors_count3_SET((char)49874) ;
        p1.drop_rate_comm_SET((char)52932) ;
        p1.errors_count2_SET((char)21936) ;
        p1.load_SET((char)13562) ;
        p1.battery_remaining_SET((byte)56) ;
        p1.errors_count4_SET((char)36146) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW)) ;
        p1.voltage_battery_SET((char)42157) ;
        p1.errors_count1_SET((char)23454) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 8902231203336642358L);
            assert(pack.time_boot_ms_GET() == 3027737070L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(8902231203336642358L) ;
        p2.time_boot_ms_SET(3027737070L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -5.3204077E37F);
            assert(pack.type_mask_GET() == (char)39109);
            assert(pack.afy_GET() == 8.669935E37F);
            assert(pack.vy_GET() == 1.8822253E38F);
            assert(pack.afz_GET() == 2.553429E38F);
            assert(pack.afx_GET() == 9.259497E37F);
            assert(pack.time_boot_ms_GET() == 3038104671L);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.vx_GET() == 3.7636042E37F);
            assert(pack.yaw_GET() == 2.226066E38F);
            assert(pack.yaw_rate_GET() == 2.797446E38F);
            assert(pack.x_GET() == 3.1466903E38F);
            assert(pack.z_GET() == -2.4246868E38F);
            assert(pack.vz_GET() == -8.283649E36F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.afx_SET(9.259497E37F) ;
        p3.afz_SET(2.553429E38F) ;
        p3.time_boot_ms_SET(3038104671L) ;
        p3.vx_SET(3.7636042E37F) ;
        p3.x_SET(3.1466903E38F) ;
        p3.type_mask_SET((char)39109) ;
        p3.afy_SET(8.669935E37F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p3.y_SET(-5.3204077E37F) ;
        p3.vz_SET(-8.283649E36F) ;
        p3.yaw_rate_SET(2.797446E38F) ;
        p3.vy_SET(1.8822253E38F) ;
        p3.z_SET(-2.4246868E38F) ;
        p3.yaw_SET(2.226066E38F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)151);
            assert(pack.seq_GET() == 250790465L);
            assert(pack.target_component_GET() == (char)65);
            assert(pack.time_usec_GET() == 8347306262047844451L);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.target_system_SET((char)151) ;
        p4.time_usec_SET(8347306262047844451L) ;
        p4.target_component_SET((char)65) ;
        p4.seq_SET(250790465L) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)247);
            assert(pack.control_request_GET() == (char)119);
            assert(pack.version_GET() == (char)121);
            assert(pack.passkey_LEN(ph) == 8);
            assert(pack.passkey_TRY(ph).equals("ckKihebm"));
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.version_SET((char)121) ;
        p5.target_system_SET((char)247) ;
        p5.passkey_SET("ckKihebm", PH) ;
        p5.control_request_SET((char)119) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)112);
            assert(pack.ack_GET() == (char)137);
            assert(pack.control_request_GET() == (char)111);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.control_request_SET((char)111) ;
        p6.ack_SET((char)137) ;
        p6.gcs_system_id_SET((char)112) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 22);
            assert(pack.key_TRY(ph).equals("fjoldgquzqctoljylPlmda"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("fjoldgquzqctoljylPlmda", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_GUIDED_ARMED);
            assert(pack.custom_mode_GET() == 2664867261L);
            assert(pack.target_system_GET() == (char)40);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED) ;
        p11.custom_mode_SET(2664867261L) ;
        p11.target_system_SET((char)40) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("muggtfm"));
            assert(pack.target_component_GET() == (char)147);
            assert(pack.param_index_GET() == (short)32128);
            assert(pack.target_system_GET() == (char)77);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_index_SET((short)32128) ;
        p20.param_id_SET("muggtfm", PH) ;
        p20.target_component_SET((char)147) ;
        p20.target_system_SET((char)77) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)173);
            assert(pack.target_system_GET() == (char)196);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)173) ;
        p21.target_system_SET((char)196) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("kkb"));
            assert(pack.param_index_GET() == (char)26358);
            assert(pack.param_count_GET() == (char)30277);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
            assert(pack.param_value_GET() == 1.6230945E38F);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_value_SET(1.6230945E38F) ;
        p22.param_count_SET((char)30277) ;
        p22.param_index_SET((char)26358) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32) ;
        p22.param_id_SET("kkb", PH) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
            assert(pack.param_value_GET() == 1.3166969E38F);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("blwg"));
            assert(pack.target_component_GET() == (char)101);
            assert(pack.target_system_GET() == (char)108);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.param_id_SET("blwg", PH) ;
        p23.param_value_SET(1.3166969E38F) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32) ;
        p23.target_component_SET((char)101) ;
        p23.target_system_SET((char)108) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.alt_ellipsoid_TRY(ph) == -1614818101);
            assert(pack.lat_GET() == 1365071803);
            assert(pack.cog_GET() == (char)8181);
            assert(pack.alt_GET() == 605067800);
            assert(pack.eph_GET() == (char)8495);
            assert(pack.hdg_acc_TRY(ph) == 893517326L);
            assert(pack.vel_acc_TRY(ph) == 2592684698L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
            assert(pack.vel_GET() == (char)8658);
            assert(pack.epv_GET() == (char)44177);
            assert(pack.v_acc_TRY(ph) == 3831948880L);
            assert(pack.time_usec_GET() == 1982989207214188369L);
            assert(pack.lon_GET() == -197687497);
            assert(pack.h_acc_TRY(ph) == 487049436L);
            assert(pack.satellites_visible_GET() == (char)198);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.satellites_visible_SET((char)198) ;
        p24.lat_SET(1365071803) ;
        p24.vel_acc_SET(2592684698L, PH) ;
        p24.time_usec_SET(1982989207214188369L) ;
        p24.h_acc_SET(487049436L, PH) ;
        p24.v_acc_SET(3831948880L, PH) ;
        p24.hdg_acc_SET(893517326L, PH) ;
        p24.cog_SET((char)8181) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED) ;
        p24.alt_SET(605067800) ;
        p24.vel_SET((char)8658) ;
        p24.alt_ellipsoid_SET(-1614818101, PH) ;
        p24.epv_SET((char)44177) ;
        p24.eph_SET((char)8495) ;
        p24.lon_SET(-197687497) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)10, (char)209, (char)154, (char)45, (char)164, (char)100, (char)189, (char)226, (char)91, (char)105, (char)17, (char)61, (char)194, (char)209, (char)176, (char)74, (char)123, (char)121, (char)201, (char)118}));
            assert(pack.satellites_visible_GET() == (char)118);
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)227, (char)27, (char)66, (char)171, (char)116, (char)239, (char)75, (char)234, (char)74, (char)56, (char)82, (char)195, (char)19, (char)105, (char)213, (char)188, (char)47, (char)22, (char)146, (char)213}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)145, (char)153, (char)173, (char)30, (char)71, (char)238, (char)99, (char)111, (char)209, (char)240, (char)140, (char)25, (char)184, (char)29, (char)31, (char)168, (char)18, (char)39, (char)2, (char)103}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)173, (char)12, (char)88, (char)42, (char)182, (char)234, (char)126, (char)130, (char)150, (char)169, (char)180, (char)28, (char)195, (char)177, (char)54, (char)220, (char)118, (char)173, (char)233, (char)157}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)125, (char)164, (char)2, (char)204, (char)221, (char)228, (char)106, (char)80, (char)119, (char)148, (char)2, (char)47, (char)3, (char)91, (char)37, (char)97, (char)149, (char)62, (char)72, (char)84}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_prn_SET(new char[] {(char)10, (char)209, (char)154, (char)45, (char)164, (char)100, (char)189, (char)226, (char)91, (char)105, (char)17, (char)61, (char)194, (char)209, (char)176, (char)74, (char)123, (char)121, (char)201, (char)118}, 0) ;
        p25.satellites_visible_SET((char)118) ;
        p25.satellite_azimuth_SET(new char[] {(char)227, (char)27, (char)66, (char)171, (char)116, (char)239, (char)75, (char)234, (char)74, (char)56, (char)82, (char)195, (char)19, (char)105, (char)213, (char)188, (char)47, (char)22, (char)146, (char)213}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)125, (char)164, (char)2, (char)204, (char)221, (char)228, (char)106, (char)80, (char)119, (char)148, (char)2, (char)47, (char)3, (char)91, (char)37, (char)97, (char)149, (char)62, (char)72, (char)84}, 0) ;
        p25.satellite_used_SET(new char[] {(char)145, (char)153, (char)173, (char)30, (char)71, (char)238, (char)99, (char)111, (char)209, (char)240, (char)140, (char)25, (char)184, (char)29, (char)31, (char)168, (char)18, (char)39, (char)2, (char)103}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)173, (char)12, (char)88, (char)42, (char)182, (char)234, (char)126, (char)130, (char)150, (char)169, (char)180, (char)28, (char)195, (char)177, (char)54, (char)220, (char)118, (char)173, (char)233, (char)157}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short) -13687);
            assert(pack.zmag_GET() == (short)8322);
            assert(pack.time_boot_ms_GET() == 1780521212L);
            assert(pack.ymag_GET() == (short) -1627);
            assert(pack.ygyro_GET() == (short) -11052);
            assert(pack.xmag_GET() == (short)28407);
            assert(pack.xacc_GET() == (short) -3133);
            assert(pack.zgyro_GET() == (short) -11298);
            assert(pack.xgyro_GET() == (short)32511);
            assert(pack.zacc_GET() == (short) -12461);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.xgyro_SET((short)32511) ;
        p26.zgyro_SET((short) -11298) ;
        p26.yacc_SET((short) -13687) ;
        p26.xmag_SET((short)28407) ;
        p26.time_boot_ms_SET(1780521212L) ;
        p26.zacc_SET((short) -12461) ;
        p26.xacc_SET((short) -3133) ;
        p26.ymag_SET((short) -1627) ;
        p26.zmag_SET((short)8322) ;
        p26.ygyro_SET((short) -11052) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short)21491);
            assert(pack.xacc_GET() == (short)10170);
            assert(pack.yacc_GET() == (short) -6402);
            assert(pack.ymag_GET() == (short)16698);
            assert(pack.time_usec_GET() == 614910007264965340L);
            assert(pack.xgyro_GET() == (short)26887);
            assert(pack.zacc_GET() == (short)9191);
            assert(pack.zgyro_GET() == (short)32455);
            assert(pack.zmag_GET() == (short)7677);
            assert(pack.xmag_GET() == (short)17664);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.xmag_SET((short)17664) ;
        p27.ygyro_SET((short)21491) ;
        p27.xgyro_SET((short)26887) ;
        p27.time_usec_SET(614910007264965340L) ;
        p27.zmag_SET((short)7677) ;
        p27.xacc_SET((short)10170) ;
        p27.yacc_SET((short) -6402) ;
        p27.zgyro_SET((short)32455) ;
        p27.ymag_SET((short)16698) ;
        p27.zacc_SET((short)9191) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == (short) -28123);
            assert(pack.time_usec_GET() == 5401613942328253093L);
            assert(pack.press_diff2_GET() == (short)7890);
            assert(pack.press_diff1_GET() == (short)11028);
            assert(pack.temperature_GET() == (short)15429);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_abs_SET((short) -28123) ;
        p28.press_diff2_SET((short)7890) ;
        p28.press_diff1_SET((short)11028) ;
        p28.temperature_SET((short)15429) ;
        p28.time_usec_SET(5401613942328253093L) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -2.2825081E38F);
            assert(pack.time_boot_ms_GET() == 87908280L);
            assert(pack.temperature_GET() == (short)21940);
            assert(pack.press_diff_GET() == 9.659542E37F);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.time_boot_ms_SET(87908280L) ;
        p29.press_diff_SET(9.659542E37F) ;
        p29.temperature_SET((short)21940) ;
        p29.press_abs_SET(-2.2825081E38F) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 2.6419445E38F);
            assert(pack.roll_GET() == 6.93507E37F);
            assert(pack.pitch_GET() == -3.3470604E38F);
            assert(pack.time_boot_ms_GET() == 1336516802L);
            assert(pack.rollspeed_GET() == -2.6303533E38F);
            assert(pack.yaw_GET() == -2.4585029E38F);
            assert(pack.pitchspeed_GET() == -1.9172351E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.yawspeed_SET(2.6419445E38F) ;
        p30.yaw_SET(-2.4585029E38F) ;
        p30.pitchspeed_SET(-1.9172351E38F) ;
        p30.rollspeed_SET(-2.6303533E38F) ;
        p30.pitch_SET(-3.3470604E38F) ;
        p30.time_boot_ms_SET(1336516802L) ;
        p30.roll_SET(6.93507E37F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3666542314L);
            assert(pack.q1_GET() == -1.7937015E38F);
            assert(pack.q3_GET() == 3.0683134E38F);
            assert(pack.pitchspeed_GET() == 3.302635E38F);
            assert(pack.rollspeed_GET() == -2.6698865E38F);
            assert(pack.yawspeed_GET() == -9.737502E37F);
            assert(pack.q4_GET() == -2.0838579E37F);
            assert(pack.q2_GET() == 2.3648527E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q2_SET(2.3648527E38F) ;
        p31.q3_SET(3.0683134E38F) ;
        p31.pitchspeed_SET(3.302635E38F) ;
        p31.q1_SET(-1.7937015E38F) ;
        p31.rollspeed_SET(-2.6698865E38F) ;
        p31.time_boot_ms_SET(3666542314L) ;
        p31.yawspeed_SET(-9.737502E37F) ;
        p31.q4_SET(-2.0838579E37F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -3.2420842E38F);
            assert(pack.time_boot_ms_GET() == 1090564179L);
            assert(pack.x_GET() == -2.0051103E38F);
            assert(pack.vz_GET() == -6.3421206E36F);
            assert(pack.vx_GET() == 4.9562263E37F);
            assert(pack.z_GET() == 1.6885657E38F);
            assert(pack.vy_GET() == 1.6574595E38F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.x_SET(-2.0051103E38F) ;
        p32.vx_SET(4.9562263E37F) ;
        p32.y_SET(-3.2420842E38F) ;
        p32.time_boot_ms_SET(1090564179L) ;
        p32.vy_SET(1.6574595E38F) ;
        p32.vz_SET(-6.3421206E36F) ;
        p32.z_SET(1.6885657E38F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == (short) -719);
            assert(pack.lon_GET() == 504152420);
            assert(pack.vz_GET() == (short)24329);
            assert(pack.hdg_GET() == (char)3988);
            assert(pack.time_boot_ms_GET() == 3990488463L);
            assert(pack.lat_GET() == 754340775);
            assert(pack.vy_GET() == (short) -5457);
            assert(pack.relative_alt_GET() == 1424987301);
            assert(pack.alt_GET() == -2106771915);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.time_boot_ms_SET(3990488463L) ;
        p33.vy_SET((short) -5457) ;
        p33.hdg_SET((char)3988) ;
        p33.alt_SET(-2106771915) ;
        p33.relative_alt_SET(1424987301) ;
        p33.lon_SET(504152420) ;
        p33.lat_SET(754340775) ;
        p33.vx_SET((short) -719) ;
        p33.vz_SET((short)24329) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan2_scaled_GET() == (short)13213);
            assert(pack.chan6_scaled_GET() == (short)28531);
            assert(pack.port_GET() == (char)249);
            assert(pack.chan5_scaled_GET() == (short) -487);
            assert(pack.time_boot_ms_GET() == 1693301477L);
            assert(pack.chan8_scaled_GET() == (short)6574);
            assert(pack.chan1_scaled_GET() == (short) -5186);
            assert(pack.chan3_scaled_GET() == (short) -28351);
            assert(pack.chan4_scaled_GET() == (short) -14146);
            assert(pack.rssi_GET() == (char)67);
            assert(pack.chan7_scaled_GET() == (short)23014);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan5_scaled_SET((short) -487) ;
        p34.chan6_scaled_SET((short)28531) ;
        p34.chan3_scaled_SET((short) -28351) ;
        p34.chan8_scaled_SET((short)6574) ;
        p34.chan4_scaled_SET((short) -14146) ;
        p34.chan7_scaled_SET((short)23014) ;
        p34.port_SET((char)249) ;
        p34.chan2_scaled_SET((short)13213) ;
        p34.rssi_SET((char)67) ;
        p34.time_boot_ms_SET(1693301477L) ;
        p34.chan1_scaled_SET((short) -5186) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1670509673L);
            assert(pack.chan6_raw_GET() == (char)8701);
            assert(pack.chan8_raw_GET() == (char)9601);
            assert(pack.chan1_raw_GET() == (char)5656);
            assert(pack.chan7_raw_GET() == (char)56441);
            assert(pack.chan2_raw_GET() == (char)21962);
            assert(pack.port_GET() == (char)113);
            assert(pack.chan3_raw_GET() == (char)55378);
            assert(pack.chan4_raw_GET() == (char)32698);
            assert(pack.chan5_raw_GET() == (char)698);
            assert(pack.rssi_GET() == (char)42);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.time_boot_ms_SET(1670509673L) ;
        p35.chan5_raw_SET((char)698) ;
        p35.chan3_raw_SET((char)55378) ;
        p35.chan6_raw_SET((char)8701) ;
        p35.port_SET((char)113) ;
        p35.chan2_raw_SET((char)21962) ;
        p35.rssi_SET((char)42) ;
        p35.chan1_raw_SET((char)5656) ;
        p35.chan4_raw_SET((char)32698) ;
        p35.chan8_raw_SET((char)9601) ;
        p35.chan7_raw_SET((char)56441) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo7_raw_GET() == (char)20224);
            assert(pack.servo12_raw_TRY(ph) == (char)41045);
            assert(pack.servo3_raw_GET() == (char)60633);
            assert(pack.servo2_raw_GET() == (char)11107);
            assert(pack.servo9_raw_TRY(ph) == (char)22796);
            assert(pack.servo6_raw_GET() == (char)29042);
            assert(pack.servo11_raw_TRY(ph) == (char)22582);
            assert(pack.servo13_raw_TRY(ph) == (char)10806);
            assert(pack.time_usec_GET() == 33310140L);
            assert(pack.servo4_raw_GET() == (char)58671);
            assert(pack.servo8_raw_GET() == (char)11425);
            assert(pack.servo1_raw_GET() == (char)63348);
            assert(pack.servo16_raw_TRY(ph) == (char)54666);
            assert(pack.servo15_raw_TRY(ph) == (char)20425);
            assert(pack.port_GET() == (char)227);
            assert(pack.servo5_raw_GET() == (char)63680);
            assert(pack.servo14_raw_TRY(ph) == (char)33460);
            assert(pack.servo10_raw_TRY(ph) == (char)28668);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo6_raw_SET((char)29042) ;
        p36.servo2_raw_SET((char)11107) ;
        p36.servo9_raw_SET((char)22796, PH) ;
        p36.servo5_raw_SET((char)63680) ;
        p36.servo15_raw_SET((char)20425, PH) ;
        p36.time_usec_SET(33310140L) ;
        p36.servo3_raw_SET((char)60633) ;
        p36.port_SET((char)227) ;
        p36.servo11_raw_SET((char)22582, PH) ;
        p36.servo14_raw_SET((char)33460, PH) ;
        p36.servo7_raw_SET((char)20224) ;
        p36.servo4_raw_SET((char)58671) ;
        p36.servo12_raw_SET((char)41045, PH) ;
        p36.servo13_raw_SET((char)10806, PH) ;
        p36.servo1_raw_SET((char)63348) ;
        p36.servo8_raw_SET((char)11425) ;
        p36.servo10_raw_SET((char)28668, PH) ;
        p36.servo16_raw_SET((char)54666, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.start_index_GET() == (short)31978);
            assert(pack.target_component_GET() == (char)247);
            assert(pack.target_system_GET() == (char)161);
            assert(pack.end_index_GET() == (short) -23147);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.end_index_SET((short) -23147) ;
        p37.target_system_SET((char)161) ;
        p37.start_index_SET((short)31978) ;
        p37.target_component_SET((char)247) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)156);
            assert(pack.start_index_GET() == (short) -25435);
            assert(pack.target_system_GET() == (char)242);
            assert(pack.end_index_GET() == (short)9063);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.end_index_SET((short)9063) ;
        p38.target_component_SET((char)156) ;
        p38.target_system_SET((char)242) ;
        p38.start_index_SET((short) -25435) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.autocontinue_GET() == (char)193);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.target_system_GET() == (char)211);
            assert(pack.target_component_GET() == (char)82);
            assert(pack.z_GET() == -3.265007E38F);
            assert(pack.seq_GET() == (char)30355);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION);
            assert(pack.param4_GET() == -1.1778527E38F);
            assert(pack.y_GET() == -8.621599E37F);
            assert(pack.param1_GET() == 1.1844314E38F);
            assert(pack.param2_GET() == -1.2284858E37F);
            assert(pack.x_GET() == 5.293484E37F);
            assert(pack.current_GET() == (char)40);
            assert(pack.param3_GET() == 3.111094E38F);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.target_system_SET((char)211) ;
        p39.param1_SET(1.1844314E38F) ;
        p39.autocontinue_SET((char)193) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p39.x_SET(5.293484E37F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION) ;
        p39.param2_SET(-1.2284858E37F) ;
        p39.param4_SET(-1.1778527E38F) ;
        p39.target_component_SET((char)82) ;
        p39.z_SET(-3.265007E38F) ;
        p39.y_SET(-8.621599E37F) ;
        p39.param3_SET(3.111094E38F) ;
        p39.current_SET((char)40) ;
        p39.seq_SET((char)30355) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)11951);
            assert(pack.target_system_GET() == (char)72);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_component_GET() == (char)170);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p40.seq_SET((char)11951) ;
        p40.target_system_SET((char)72) ;
        p40.target_component_SET((char)170) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)5265);
            assert(pack.target_system_GET() == (char)237);
            assert(pack.target_component_GET() == (char)184);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.seq_SET((char)5265) ;
        p41.target_system_SET((char)237) ;
        p41.target_component_SET((char)184) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)62137);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)62137) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)12);
            assert(pack.target_component_GET() == (char)139);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)12) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p43.target_component_SET((char)139) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.count_GET() == (char)9381);
            assert(pack.target_component_GET() == (char)196);
            assert(pack.target_system_GET() == (char)180);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_component_SET((char)196) ;
        p44.target_system_SET((char)180) ;
        p44.count_SET((char)9381) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)204);
            assert(pack.target_component_GET() == (char)191);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_component_SET((char)191) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p45.target_system_SET((char)204) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)12301);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)12301) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)220);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)144);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p47.target_component_SET((char)220) ;
        p47.target_system_SET((char)144) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == 2083140074);
            assert(pack.target_system_GET() == (char)87);
            assert(pack.altitude_GET() == -1040362378);
            assert(pack.latitude_GET() == -195204489);
            assert(pack.time_usec_TRY(ph) == 1802465924211634192L);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.longitude_SET(2083140074) ;
        p48.altitude_SET(-1040362378) ;
        p48.time_usec_SET(1802465924211634192L, PH) ;
        p48.target_system_SET((char)87) ;
        p48.latitude_SET(-195204489) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 5946547139846109166L);
            assert(pack.latitude_GET() == 2092313297);
            assert(pack.altitude_GET() == 922554729);
            assert(pack.longitude_GET() == 1265161466);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(2092313297) ;
        p49.time_usec_SET(5946547139846109166L, PH) ;
        p49.longitude_SET(1265161466) ;
        p49.altitude_SET(922554729) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)168);
            assert(pack.param_value_min_GET() == -2.588537E38F);
            assert(pack.param_value0_GET() == -7.8758833E37F);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("uyphofliczxb"));
            assert(pack.param_index_GET() == (short) -6019);
            assert(pack.parameter_rc_channel_index_GET() == (char)223);
            assert(pack.target_component_GET() == (char)230);
            assert(pack.scale_GET() == -8.614239E37F);
            assert(pack.param_value_max_GET() == -2.2368364E38F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_system_SET((char)168) ;
        p50.param_value_min_SET(-2.588537E38F) ;
        p50.param_value_max_SET(-2.2368364E38F) ;
        p50.param_index_SET((short) -6019) ;
        p50.target_component_SET((char)230) ;
        p50.param_id_SET("uyphofliczxb", PH) ;
        p50.parameter_rc_channel_index_SET((char)223) ;
        p50.scale_SET(-8.614239E37F) ;
        p50.param_value0_SET(-7.8758833E37F) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)89);
            assert(pack.seq_GET() == (char)38162);
            assert(pack.target_system_GET() == (char)13);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.seq_SET((char)38162) ;
        p51.target_component_SET((char)89) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p51.target_system_SET((char)13) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.p1y_GET() == 2.2014195E38F);
            assert(pack.p2y_GET() == 6.194839E35F);
            assert(pack.target_system_GET() == (char)106);
            assert(pack.p2z_GET() == 2.4177444E36F);
            assert(pack.p1z_GET() == 1.0511702E38F);
            assert(pack.target_component_GET() == (char)163);
            assert(pack.p1x_GET() == 1.3579015E38F);
            assert(pack.p2x_GET() == -2.7201932E38F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p2x_SET(-2.7201932E38F) ;
        p54.p1x_SET(1.3579015E38F) ;
        p54.p1y_SET(2.2014195E38F) ;
        p54.target_system_SET((char)106) ;
        p54.p2y_SET(6.194839E35F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p54.target_component_SET((char)163) ;
        p54.p1z_SET(1.0511702E38F) ;
        p54.p2z_SET(2.4177444E36F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1y_GET() == 1.2420017E37F);
            assert(pack.p1z_GET() == -8.2672095E37F);
            assert(pack.p1x_GET() == 2.3850154E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.p2y_GET() == 1.2540718E38F);
            assert(pack.p2z_GET() == 4.34688E37F);
            assert(pack.p2x_GET() == 2.2790772E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2x_SET(2.2790772E38F) ;
        p55.p1x_SET(2.3850154E38F) ;
        p55.p1z_SET(-8.2672095E37F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p55.p2y_SET(1.2540718E38F) ;
        p55.p1y_SET(1.2420017E37F) ;
        p55.p2z_SET(4.34688E37F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == -2.0828201E38F);
            assert(pack.rollspeed_GET() == -3.153754E38F);
            assert(pack.pitchspeed_GET() == -3.372464E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-4.744128E37F, 2.0336278E38F, 1.8326674E38F, 2.795201E38F, 2.1547675E38F, 4.53486E37F, 4.4898577E37F, -1.1153228E38F, 1.6174625E38F}));
            assert(Arrays.equals(pack.q_GET(),  new float[] {-4.700752E37F, 3.3062946E38F, -3.1915814E38F, -1.8548596E38F}));
            assert(pack.time_usec_GET() == 8771810035441680171L);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.rollspeed_SET(-3.153754E38F) ;
        p61.covariance_SET(new float[] {-4.744128E37F, 2.0336278E38F, 1.8326674E38F, 2.795201E38F, 2.1547675E38F, 4.53486E37F, 4.4898577E37F, -1.1153228E38F, 1.6174625E38F}, 0) ;
        p61.yawspeed_SET(-2.0828201E38F) ;
        p61.pitchspeed_SET(-3.372464E38F) ;
        p61.time_usec_SET(8771810035441680171L) ;
        p61.q_SET(new float[] {-4.700752E37F, 3.3062946E38F, -3.1915814E38F, -1.8548596E38F}, 0) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_pitch_GET() == 8.932043E37F);
            assert(pack.target_bearing_GET() == (short) -7067);
            assert(pack.aspd_error_GET() == -1.1570591E38F);
            assert(pack.nav_roll_GET() == -2.4095586E38F);
            assert(pack.nav_bearing_GET() == (short) -25804);
            assert(pack.alt_error_GET() == -3.1926916E38F);
            assert(pack.wp_dist_GET() == (char)50670);
            assert(pack.xtrack_error_GET() == 9.915832E35F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.alt_error_SET(-3.1926916E38F) ;
        p62.nav_bearing_SET((short) -25804) ;
        p62.wp_dist_SET((char)50670) ;
        p62.target_bearing_SET((short) -7067) ;
        p62.nav_roll_SET(-2.4095586E38F) ;
        p62.nav_pitch_SET(8.932043E37F) ;
        p62.aspd_error_SET(-1.1570591E38F) ;
        p62.xtrack_error_SET(9.915832E35F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 228216488);
            assert(pack.alt_GET() == 631358310);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
            assert(pack.lat_GET() == 1046422390);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {8.201178E36F, 3.0875265E38F, -2.4935697E38F, 3.3266918E38F, 2.0694473E38F, 2.9348525E38F, -2.4136765E38F, -1.5371761E38F, 1.966045E38F, 2.0056311E38F, -7.9223584E37F, 3.3198745E38F, 3.282567E38F, -2.701592E38F, 1.4845585E38F, -3.38062E38F, 1.6431681E38F, -1.7405646E38F, 2.2618562E38F, 2.583756E38F, 3.2409502E38F, -2.6860571E38F, -1.0325236E37F, 1.5158631E38F, 2.5663791E38F, -1.5266089E38F, 2.1336557E38F, -2.8928775E37F, -3.821655E37F, 3.0414917E38F, 2.8346876E38F, -1.8109642E38F, 4.8811216E37F, 4.783116E37F, 1.0822219E38F, -2.4854235E38F}));
            assert(pack.vz_GET() == 8.786617E37F);
            assert(pack.vx_GET() == -6.027641E37F);
            assert(pack.relative_alt_GET() == -653968036);
            assert(pack.vy_GET() == -7.1819546E37F);
            assert(pack.time_usec_GET() == 6393302479230112468L);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.relative_alt_SET(-653968036) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION) ;
        p63.covariance_SET(new float[] {8.201178E36F, 3.0875265E38F, -2.4935697E38F, 3.3266918E38F, 2.0694473E38F, 2.9348525E38F, -2.4136765E38F, -1.5371761E38F, 1.966045E38F, 2.0056311E38F, -7.9223584E37F, 3.3198745E38F, 3.282567E38F, -2.701592E38F, 1.4845585E38F, -3.38062E38F, 1.6431681E38F, -1.7405646E38F, 2.2618562E38F, 2.583756E38F, 3.2409502E38F, -2.6860571E38F, -1.0325236E37F, 1.5158631E38F, 2.5663791E38F, -1.5266089E38F, 2.1336557E38F, -2.8928775E37F, -3.821655E37F, 3.0414917E38F, 2.8346876E38F, -1.8109642E38F, 4.8811216E37F, 4.783116E37F, 1.0822219E38F, -2.4854235E38F}, 0) ;
        p63.lat_SET(1046422390) ;
        p63.time_usec_SET(6393302479230112468L) ;
        p63.alt_SET(631358310) ;
        p63.vy_SET(-7.1819546E37F) ;
        p63.vz_SET(8.786617E37F) ;
        p63.lon_SET(228216488) ;
        p63.vx_SET(-6.027641E37F) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == -2.0464718E38F);
            assert(pack.x_GET() == -3.9322983E37F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
            assert(pack.z_GET() == -7.2281153E37F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {8.583216E37F, 8.508012E36F, 1.5110565E38F, 1.2206884E38F, -1.3477565E38F, 2.6158516E38F, 2.401741E38F, 2.2675539E37F, 7.4366875E37F, 3.056712E38F, 2.829456E37F, -2.0610048E37F, 1.1186257E38F, 6.0010716E37F, 1.6576556E38F, -5.913186E37F, 3.341852E38F, 6.407726E37F, 1.8297088E38F, 1.1463209E38F, -1.5861274E38F, -3.0877482E38F, 6.396866E37F, -2.628435E38F, -2.8623798E38F, 2.7901815E38F, 4.271753E37F, -3.352971E38F, -2.1052482E38F, -1.3412665E38F, -2.5228754E38F, 8.137566E37F, 1.838968E38F, -2.005282E38F, -1.320329E38F, -1.7020006E38F, 2.6433983E37F, -2.745661E37F, 1.2112569E38F, 1.1509043E38F, 1.0841506E38F, 1.9552097E38F, 1.3050235E38F, -1.119354E38F, 2.162486E38F}));
            assert(pack.vy_GET() == -2.9610347E38F);
            assert(pack.ay_GET() == -3.2737105E37F);
            assert(pack.az_GET() == -1.9913937E38F);
            assert(pack.vx_GET() == -2.6309575E38F);
            assert(pack.y_GET() == -8.562949E37F);
            assert(pack.ax_GET() == -4.355562E37F);
            assert(pack.time_usec_GET() == 2108791019396635503L);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.ax_SET(-4.355562E37F) ;
        p64.time_usec_SET(2108791019396635503L) ;
        p64.z_SET(-7.2281153E37F) ;
        p64.covariance_SET(new float[] {8.583216E37F, 8.508012E36F, 1.5110565E38F, 1.2206884E38F, -1.3477565E38F, 2.6158516E38F, 2.401741E38F, 2.2675539E37F, 7.4366875E37F, 3.056712E38F, 2.829456E37F, -2.0610048E37F, 1.1186257E38F, 6.0010716E37F, 1.6576556E38F, -5.913186E37F, 3.341852E38F, 6.407726E37F, 1.8297088E38F, 1.1463209E38F, -1.5861274E38F, -3.0877482E38F, 6.396866E37F, -2.628435E38F, -2.8623798E38F, 2.7901815E38F, 4.271753E37F, -3.352971E38F, -2.1052482E38F, -1.3412665E38F, -2.5228754E38F, 8.137566E37F, 1.838968E38F, -2.005282E38F, -1.320329E38F, -1.7020006E38F, 2.6433983E37F, -2.745661E37F, 1.2112569E38F, 1.1509043E38F, 1.0841506E38F, 1.9552097E38F, 1.3050235E38F, -1.119354E38F, 2.162486E38F}, 0) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION) ;
        p64.x_SET(-3.9322983E37F) ;
        p64.ay_SET(-3.2737105E37F) ;
        p64.vx_SET(-2.6309575E38F) ;
        p64.vz_SET(-2.0464718E38F) ;
        p64.az_SET(-1.9913937E38F) ;
        p64.y_SET(-8.562949E37F) ;
        p64.vy_SET(-2.9610347E38F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan6_raw_GET() == (char)15457);
            assert(pack.time_boot_ms_GET() == 2502968102L);
            assert(pack.chan10_raw_GET() == (char)10774);
            assert(pack.chan12_raw_GET() == (char)46131);
            assert(pack.chan2_raw_GET() == (char)44287);
            assert(pack.chan17_raw_GET() == (char)17844);
            assert(pack.rssi_GET() == (char)205);
            assert(pack.chan3_raw_GET() == (char)21590);
            assert(pack.chancount_GET() == (char)42);
            assert(pack.chan9_raw_GET() == (char)47305);
            assert(pack.chan4_raw_GET() == (char)27565);
            assert(pack.chan1_raw_GET() == (char)17458);
            assert(pack.chan16_raw_GET() == (char)3739);
            assert(pack.chan11_raw_GET() == (char)35307);
            assert(pack.chan15_raw_GET() == (char)39096);
            assert(pack.chan18_raw_GET() == (char)62087);
            assert(pack.chan5_raw_GET() == (char)20364);
            assert(pack.chan14_raw_GET() == (char)60466);
            assert(pack.chan7_raw_GET() == (char)58505);
            assert(pack.chan13_raw_GET() == (char)60950);
            assert(pack.chan8_raw_GET() == (char)61960);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.time_boot_ms_SET(2502968102L) ;
        p65.chan14_raw_SET((char)60466) ;
        p65.chan17_raw_SET((char)17844) ;
        p65.chan18_raw_SET((char)62087) ;
        p65.chan2_raw_SET((char)44287) ;
        p65.chan10_raw_SET((char)10774) ;
        p65.rssi_SET((char)205) ;
        p65.chan4_raw_SET((char)27565) ;
        p65.chancount_SET((char)42) ;
        p65.chan9_raw_SET((char)47305) ;
        p65.chan3_raw_SET((char)21590) ;
        p65.chan11_raw_SET((char)35307) ;
        p65.chan8_raw_SET((char)61960) ;
        p65.chan7_raw_SET((char)58505) ;
        p65.chan13_raw_SET((char)60950) ;
        p65.chan16_raw_SET((char)3739) ;
        p65.chan15_raw_SET((char)39096) ;
        p65.chan6_raw_SET((char)15457) ;
        p65.chan1_raw_SET((char)17458) ;
        p65.chan5_raw_SET((char)20364) ;
        p65.chan12_raw_SET((char)46131) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)109);
            assert(pack.req_message_rate_GET() == (char)30520);
            assert(pack.target_component_GET() == (char)33);
            assert(pack.req_stream_id_GET() == (char)226);
            assert(pack.start_stop_GET() == (char)205);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_system_SET((char)109) ;
        p66.req_stream_id_SET((char)226) ;
        p66.req_message_rate_SET((char)30520) ;
        p66.start_stop_SET((char)205) ;
        p66.target_component_SET((char)33) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.stream_id_GET() == (char)233);
            assert(pack.message_rate_GET() == (char)46803);
            assert(pack.on_off_GET() == (char)162);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.on_off_SET((char)162) ;
        p67.message_rate_SET((char)46803) ;
        p67.stream_id_SET((char)233) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == (short)1378);
            assert(pack.target_GET() == (char)31);
            assert(pack.r_GET() == (short)18684);
            assert(pack.z_GET() == (short) -16969);
            assert(pack.y_GET() == (short)13018);
            assert(pack.buttons_GET() == (char)7370);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.y_SET((short)13018) ;
        p69.x_SET((short)1378) ;
        p69.z_SET((short) -16969) ;
        p69.target_SET((char)31) ;
        p69.buttons_SET((char)7370) ;
        p69.r_SET((short)18684) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan3_raw_GET() == (char)56493);
            assert(pack.target_component_GET() == (char)196);
            assert(pack.chan1_raw_GET() == (char)32950);
            assert(pack.chan6_raw_GET() == (char)56104);
            assert(pack.chan4_raw_GET() == (char)31289);
            assert(pack.chan8_raw_GET() == (char)50698);
            assert(pack.chan5_raw_GET() == (char)20503);
            assert(pack.chan2_raw_GET() == (char)55059);
            assert(pack.chan7_raw_GET() == (char)17219);
            assert(pack.target_system_GET() == (char)47);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan4_raw_SET((char)31289) ;
        p70.chan7_raw_SET((char)17219) ;
        p70.chan5_raw_SET((char)20503) ;
        p70.chan6_raw_SET((char)56104) ;
        p70.target_component_SET((char)196) ;
        p70.target_system_SET((char)47) ;
        p70.chan2_raw_SET((char)55059) ;
        p70.chan3_raw_SET((char)56493) ;
        p70.chan8_raw_SET((char)50698) ;
        p70.chan1_raw_SET((char)32950) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.autocontinue_GET() == (char)130);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.x_GET() == -1583076578);
            assert(pack.param2_GET() == 3.283485E38F);
            assert(pack.seq_GET() == (char)49306);
            assert(pack.z_GET() == -2.3399116E38F);
            assert(pack.param3_GET() == 2.5415372E38F);
            assert(pack.param1_GET() == 7.937667E37F);
            assert(pack.target_system_GET() == (char)208);
            assert(pack.target_component_GET() == (char)10);
            assert(pack.current_GET() == (char)235);
            assert(pack.param4_GET() == -3.0714945E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_GUIDED_MASTER);
            assert(pack.y_GET() == -1202755505);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.param1_SET(7.937667E37F) ;
        p73.param2_SET(3.283485E38F) ;
        p73.command_SET(MAV_CMD.MAV_CMD_DO_GUIDED_MASTER) ;
        p73.y_SET(-1202755505) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p73.param4_SET(-3.0714945E38F) ;
        p73.seq_SET((char)49306) ;
        p73.current_SET((char)235) ;
        p73.x_SET(-1583076578) ;
        p73.param3_SET(2.5415372E38F) ;
        p73.autocontinue_SET((char)130) ;
        p73.z_SET(-2.3399116E38F) ;
        p73.target_component_SET((char)10) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p73.target_system_SET((char)208) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.groundspeed_GET() == -2.1633721E38F);
            assert(pack.airspeed_GET() == 3.8400952E37F);
            assert(pack.heading_GET() == (short) -1438);
            assert(pack.alt_GET() == 1.3943334E38F);
            assert(pack.climb_GET() == 1.7582411E38F);
            assert(pack.throttle_GET() == (char)53083);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.airspeed_SET(3.8400952E37F) ;
        p74.throttle_SET((char)53083) ;
        p74.heading_SET((short) -1438) ;
        p74.groundspeed_SET(-2.1633721E38F) ;
        p74.alt_SET(1.3943334E38F) ;
        p74.climb_SET(1.7582411E38F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.param3_GET() == -2.1402946E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.param1_GET() == 8.2280346E37F);
            assert(pack.y_GET() == 1485814159);
            assert(pack.current_GET() == (char)16);
            assert(pack.target_system_GET() == (char)98);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_WAYPOINT);
            assert(pack.x_GET() == 1476508560);
            assert(pack.z_GET() == 1.2679718E38F);
            assert(pack.target_component_GET() == (char)14);
            assert(pack.param4_GET() == -3.2149501E38F);
            assert(pack.param2_GET() == -1.0754863E36F);
            assert(pack.autocontinue_GET() == (char)143);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.x_SET(1476508560) ;
        p75.param1_SET(8.2280346E37F) ;
        p75.y_SET(1485814159) ;
        p75.target_system_SET((char)98) ;
        p75.z_SET(1.2679718E38F) ;
        p75.param2_SET(-1.0754863E36F) ;
        p75.command_SET(MAV_CMD.MAV_CMD_NAV_WAYPOINT) ;
        p75.target_component_SET((char)14) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p75.current_SET((char)16) ;
        p75.autocontinue_SET((char)143) ;
        p75.param4_SET(-3.2149501E38F) ;
        p75.param3_SET(-2.1402946E38F) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)74);
            assert(pack.target_component_GET() == (char)222);
            assert(pack.param1_GET() == 8.774555E37F);
            assert(pack.param5_GET() == -2.9491183E38F);
            assert(pack.param2_GET() == 5.8097466E36F);
            assert(pack.param6_GET() == 2.2925214E38F);
            assert(pack.param4_GET() == 3.1356988E37F);
            assert(pack.param3_GET() == -7.602568E36F);
            assert(pack.confirmation_GET() == (char)236);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_SPATIAL_USER_1);
            assert(pack.param7_GET() == 2.576101E38F);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.param1_SET(8.774555E37F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_SPATIAL_USER_1) ;
        p76.param3_SET(-7.602568E36F) ;
        p76.confirmation_SET((char)236) ;
        p76.param2_SET(5.8097466E36F) ;
        p76.param7_SET(2.576101E38F) ;
        p76.target_component_SET((char)222) ;
        p76.param5_SET(-2.9491183E38F) ;
        p76.param6_SET(2.2925214E38F) ;
        p76.param4_SET(3.1356988E37F) ;
        p76.target_system_SET((char)74) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_TRY(ph) == (char)253);
            assert(pack.target_component_TRY(ph) == (char)30);
            assert(pack.progress_TRY(ph) == (char)253);
            assert(pack.result_param2_TRY(ph) == 720370931);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_TAKEOFF);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_DENIED);
        });
        GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.result_SET(MAV_RESULT.MAV_RESULT_DENIED) ;
        p77.command_SET(MAV_CMD.MAV_CMD_NAV_TAKEOFF) ;
        p77.result_param2_SET(720370931, PH) ;
        p77.progress_SET((char)253, PH) ;
        p77.target_component_SET((char)30, PH) ;
        p77.target_system_SET((char)253, PH) ;
        CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 1.5603496E37F);
            assert(pack.time_boot_ms_GET() == 2421931555L);
            assert(pack.roll_GET() == 2.910928E38F);
            assert(pack.mode_switch_GET() == (char)147);
            assert(pack.yaw_GET() == -1.8873345E37F);
            assert(pack.thrust_GET() == -3.1366923E38F);
            assert(pack.manual_override_switch_GET() == (char)113);
        });
        GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.mode_switch_SET((char)147) ;
        p81.manual_override_switch_SET((char)113) ;
        p81.pitch_SET(1.5603496E37F) ;
        p81.thrust_SET(-3.1366923E38F) ;
        p81.roll_SET(2.910928E38F) ;
        p81.yaw_SET(-1.8873345E37F) ;
        p81.time_boot_ms_SET(2421931555L) ;
        CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2785673555L);
            assert(pack.target_system_GET() == (char)229);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.355781E38F, -6.371348E37F, -1.5802883E38F, -2.976057E37F}));
            assert(pack.body_pitch_rate_GET() == 2.379636E38F);
            assert(pack.body_yaw_rate_GET() == 1.459018E38F);
            assert(pack.target_component_GET() == (char)39);
            assert(pack.thrust_GET() == -1.7257003E37F);
            assert(pack.body_roll_rate_GET() == 2.8760378E38F);
            assert(pack.type_mask_GET() == (char)179);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.q_SET(new float[] {2.355781E38F, -6.371348E37F, -1.5802883E38F, -2.976057E37F}, 0) ;
        p82.target_component_SET((char)39) ;
        p82.body_pitch_rate_SET(2.379636E38F) ;
        p82.target_system_SET((char)229) ;
        p82.thrust_SET(-1.7257003E37F) ;
        p82.time_boot_ms_SET(2785673555L) ;
        p82.body_roll_rate_SET(2.8760378E38F) ;
        p82.body_yaw_rate_SET(1.459018E38F) ;
        p82.type_mask_SET((char)179) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.thrust_GET() == -2.6983464E38F);
            assert(pack.body_yaw_rate_GET() == 1.3572371E38F);
            assert(pack.body_roll_rate_GET() == -1.0072661E38F);
            assert(pack.time_boot_ms_GET() == 3553534386L);
            assert(pack.body_pitch_rate_GET() == 6.48078E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.241217E37F, 3.3309868E38F, -2.105657E38F, 1.1833255E38F}));
            assert(pack.type_mask_GET() == (char)131);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_yaw_rate_SET(1.3572371E38F) ;
        p83.thrust_SET(-2.6983464E38F) ;
        p83.type_mask_SET((char)131) ;
        p83.time_boot_ms_SET(3553534386L) ;
        p83.body_pitch_rate_SET(6.48078E37F) ;
        p83.body_roll_rate_SET(-1.0072661E38F) ;
        p83.q_SET(new float[] {-3.241217E37F, 3.3309868E38F, -2.105657E38F, 1.1833255E38F}, 0) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 2.1508068E38F);
            assert(pack.target_component_GET() == (char)38);
            assert(pack.target_system_GET() == (char)229);
            assert(pack.afy_GET() == -1.1212781E38F);
            assert(pack.afx_GET() == -1.8122112E38F);
            assert(pack.y_GET() == -2.6850012E38F);
            assert(pack.x_GET() == 1.251847E38F);
            assert(pack.vx_GET() == -1.4061476E38F);
            assert(pack.type_mask_GET() == (char)19834);
            assert(pack.yaw_GET() == -2.0393837E38F);
            assert(pack.afz_GET() == -1.9006226E38F);
            assert(pack.vy_GET() == -6.28606E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.yaw_rate_GET() == -3.1725144E37F);
            assert(pack.vz_GET() == 2.9731562E38F);
            assert(pack.time_boot_ms_GET() == 293318978L);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.yaw_rate_SET(-3.1725144E37F) ;
        p84.target_system_SET((char)229) ;
        p84.vy_SET(-6.28606E37F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p84.vz_SET(2.9731562E38F) ;
        p84.y_SET(-2.6850012E38F) ;
        p84.afx_SET(-1.8122112E38F) ;
        p84.x_SET(1.251847E38F) ;
        p84.yaw_SET(-2.0393837E38F) ;
        p84.z_SET(2.1508068E38F) ;
        p84.type_mask_SET((char)19834) ;
        p84.time_boot_ms_SET(293318978L) ;
        p84.vx_SET(-1.4061476E38F) ;
        p84.afz_SET(-1.9006226E38F) ;
        p84.afy_SET(-1.1212781E38F) ;
        p84.target_component_SET((char)38) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.lat_int_GET() == 8973446);
            assert(pack.yaw_rate_GET() == -1.6079824E38F);
            assert(pack.lon_int_GET() == 2053291282);
            assert(pack.afz_GET() == -2.6528207E38F);
            assert(pack.vz_GET() == 5.7220293E37F);
            assert(pack.time_boot_ms_GET() == 3456611119L);
            assert(pack.vx_GET() == 2.9225563E38F);
            assert(pack.type_mask_GET() == (char)23410);
            assert(pack.yaw_GET() == 1.8614223E37F);
            assert(pack.alt_GET() == -2.968697E38F);
            assert(pack.target_component_GET() == (char)87);
            assert(pack.vy_GET() == -1.5624769E38F);
            assert(pack.target_system_GET() == (char)135);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.afx_GET() == -1.5991617E38F);
            assert(pack.afy_GET() == 2.875535E38F);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.vy_SET(-1.5624769E38F) ;
        p86.type_mask_SET((char)23410) ;
        p86.yaw_SET(1.8614223E37F) ;
        p86.time_boot_ms_SET(3456611119L) ;
        p86.afx_SET(-1.5991617E38F) ;
        p86.target_component_SET((char)87) ;
        p86.target_system_SET((char)135) ;
        p86.lon_int_SET(2053291282) ;
        p86.vz_SET(5.7220293E37F) ;
        p86.alt_SET(-2.968697E38F) ;
        p86.lat_int_SET(8973446) ;
        p86.vx_SET(2.9225563E38F) ;
        p86.afy_SET(2.875535E38F) ;
        p86.yaw_rate_SET(-1.6079824E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p86.afz_SET(-2.6528207E38F) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == 1.7961066E38F);
            assert(pack.time_boot_ms_GET() == 118822720L);
            assert(pack.vy_GET() == 1.1782348E38F);
            assert(pack.vz_GET() == -1.8925766E37F);
            assert(pack.type_mask_GET() == (char)4667);
            assert(pack.afx_GET() == -3.276004E38F);
            assert(pack.afz_GET() == 8.1049614E37F);
            assert(pack.lon_int_GET() == -31053365);
            assert(pack.yaw_rate_GET() == 1.2393031E38F);
            assert(pack.alt_GET() == -2.3391115E38F);
            assert(pack.yaw_GET() == -1.5111523E38F);
            assert(pack.lat_int_GET() == -1899513376);
            assert(pack.afy_GET() == -1.6730076E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.yaw_SET(-1.5111523E38F) ;
        p87.alt_SET(-2.3391115E38F) ;
        p87.lon_int_SET(-31053365) ;
        p87.yaw_rate_SET(1.2393031E38F) ;
        p87.afz_SET(8.1049614E37F) ;
        p87.time_boot_ms_SET(118822720L) ;
        p87.vz_SET(-1.8925766E37F) ;
        p87.vy_SET(1.1782348E38F) ;
        p87.lat_int_SET(-1899513376) ;
        p87.afy_SET(-1.6730076E38F) ;
        p87.afx_SET(-3.276004E38F) ;
        p87.vx_SET(1.7961066E38F) ;
        p87.type_mask_SET((char)4667) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 2.3919094E38F);
            assert(pack.roll_GET() == 3.0248585E37F);
            assert(pack.pitch_GET() == -3.0765913E38F);
            assert(pack.y_GET() == 1.2755868E38F);
            assert(pack.z_GET() == -2.928025E38F);
            assert(pack.x_GET() == -2.2119103E38F);
            assert(pack.time_boot_ms_GET() == 4026418031L);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(4026418031L) ;
        p89.z_SET(-2.928025E38F) ;
        p89.roll_SET(3.0248585E37F) ;
        p89.yaw_SET(2.3919094E38F) ;
        p89.y_SET(1.2755868E38F) ;
        p89.pitch_SET(-3.0765913E38F) ;
        p89.x_SET(-2.2119103E38F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 5.9739556E37F);
            assert(pack.lat_GET() == 2045794825);
            assert(pack.lon_GET() == -976665903);
            assert(pack.vz_GET() == (short)13695);
            assert(pack.yacc_GET() == (short) -31016);
            assert(pack.roll_GET() == -1.3548689E38F);
            assert(pack.yawspeed_GET() == 8.891206E37F);
            assert(pack.pitchspeed_GET() == -2.2562905E37F);
            assert(pack.time_usec_GET() == 2012886350451720181L);
            assert(pack.rollspeed_GET() == -1.787954E38F);
            assert(pack.vx_GET() == (short) -15527);
            assert(pack.yaw_GET() == -1.9494176E38F);
            assert(pack.alt_GET() == -71757141);
            assert(pack.xacc_GET() == (short) -11835);
            assert(pack.vy_GET() == (short) -21104);
            assert(pack.zacc_GET() == (short) -8164);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.pitchspeed_SET(-2.2562905E37F) ;
        p90.vy_SET((short) -21104) ;
        p90.yacc_SET((short) -31016) ;
        p90.pitch_SET(5.9739556E37F) ;
        p90.vx_SET((short) -15527) ;
        p90.roll_SET(-1.3548689E38F) ;
        p90.alt_SET(-71757141) ;
        p90.xacc_SET((short) -11835) ;
        p90.yawspeed_SET(8.891206E37F) ;
        p90.lat_SET(2045794825) ;
        p90.vz_SET((short)13695) ;
        p90.time_usec_SET(2012886350451720181L) ;
        p90.lon_SET(-976665903) ;
        p90.zacc_SET((short) -8164) ;
        p90.yaw_SET(-1.9494176E38F) ;
        p90.rollspeed_SET(-1.787954E38F) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.pitch_elevator_GET() == -1.2270617E37F);
            assert(pack.roll_ailerons_GET() == -8.629844E37F);
            assert(pack.time_usec_GET() == 1583018274602163842L);
            assert(pack.yaw_rudder_GET() == -2.1233227E38F);
            assert(pack.aux2_GET() == 3.0836915E38F);
            assert(pack.aux4_GET() == -2.102313E38F);
            assert(pack.aux1_GET() == -1.2163927E38F);
            assert(pack.throttle_GET() == 2.5303367E38F);
            assert(pack.aux3_GET() == -2.0313632E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_ARMED);
            assert(pack.nav_mode_GET() == (char)83);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.aux4_SET(-2.102313E38F) ;
        p91.aux1_SET(-1.2163927E38F) ;
        p91.roll_ailerons_SET(-8.629844E37F) ;
        p91.nav_mode_SET((char)83) ;
        p91.yaw_rudder_SET(-2.1233227E38F) ;
        p91.throttle_SET(2.5303367E38F) ;
        p91.time_usec_SET(1583018274602163842L) ;
        p91.aux2_SET(3.0836915E38F) ;
        p91.pitch_elevator_SET(-1.2270617E37F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED) ;
        p91.aux3_SET(-2.0313632E38F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan1_raw_GET() == (char)24859);
            assert(pack.chan9_raw_GET() == (char)58761);
            assert(pack.chan10_raw_GET() == (char)46305);
            assert(pack.chan3_raw_GET() == (char)24937);
            assert(pack.chan7_raw_GET() == (char)33576);
            assert(pack.chan4_raw_GET() == (char)14914);
            assert(pack.time_usec_GET() == 2770843648423568443L);
            assert(pack.chan5_raw_GET() == (char)2702);
            assert(pack.chan12_raw_GET() == (char)2236);
            assert(pack.chan6_raw_GET() == (char)32143);
            assert(pack.chan8_raw_GET() == (char)54059);
            assert(pack.rssi_GET() == (char)237);
            assert(pack.chan2_raw_GET() == (char)62907);
            assert(pack.chan11_raw_GET() == (char)40149);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan2_raw_SET((char)62907) ;
        p92.chan10_raw_SET((char)46305) ;
        p92.chan11_raw_SET((char)40149) ;
        p92.chan12_raw_SET((char)2236) ;
        p92.chan8_raw_SET((char)54059) ;
        p92.chan6_raw_SET((char)32143) ;
        p92.time_usec_SET(2770843648423568443L) ;
        p92.chan7_raw_SET((char)33576) ;
        p92.chan4_raw_SET((char)14914) ;
        p92.rssi_SET((char)237) ;
        p92.chan9_raw_SET((char)58761) ;
        p92.chan1_raw_SET((char)24859) ;
        p92.chan3_raw_SET((char)24937) ;
        p92.chan5_raw_SET((char)2702) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 519493261858270601L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-3.864055E37F, 3.339838E37F, -2.5202287E38F, 6.1296733E37F, -2.569764E38F, 5.6757955E37F, 5.5741817E37F, 6.2261146E37F, 1.0211806E38F, 2.8826829E38F, -2.134387E38F, -1.9428212E38F, -7.5765184E37F, -1.3241114E38F, 3.3995391E38F, -1.6054675E38F}));
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_DISARMED);
            assert(pack.flags_GET() == 3087234459366664033L);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.controls_SET(new float[] {-3.864055E37F, 3.339838E37F, -2.5202287E38F, 6.1296733E37F, -2.569764E38F, 5.6757955E37F, 5.5741817E37F, 6.2261146E37F, 1.0211806E38F, 2.8826829E38F, -2.134387E38F, -1.9428212E38F, -7.5765184E37F, -1.3241114E38F, 3.3995391E38F, -1.6054675E38F}, 0) ;
        p93.time_usec_SET(519493261858270601L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED) ;
        p93.flags_SET(3087234459366664033L) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.ground_distance_GET() == -2.1392965E38F);
            assert(pack.flow_rate_y_TRY(ph) == -1.5146634E38F);
            assert(pack.flow_x_GET() == (short)10483);
            assert(pack.flow_comp_m_y_GET() == 7.5283117E37F);
            assert(pack.quality_GET() == (char)181);
            assert(pack.time_usec_GET() == 1195187759967451022L);
            assert(pack.sensor_id_GET() == (char)127);
            assert(pack.flow_comp_m_x_GET() == 3.361538E38F);
            assert(pack.flow_y_GET() == (short)23109);
            assert(pack.flow_rate_x_TRY(ph) == -2.941628E38F);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.sensor_id_SET((char)127) ;
        p100.flow_x_SET((short)10483) ;
        p100.flow_rate_y_SET(-1.5146634E38F, PH) ;
        p100.flow_rate_x_SET(-2.941628E38F, PH) ;
        p100.quality_SET((char)181) ;
        p100.ground_distance_SET(-2.1392965E38F) ;
        p100.flow_comp_m_y_SET(7.5283117E37F) ;
        p100.flow_comp_m_x_SET(3.361538E38F) ;
        p100.time_usec_SET(1195187759967451022L) ;
        p100.flow_y_SET((short)23109) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 2.928018E38F);
            assert(pack.usec_GET() == 485040936705717253L);
            assert(pack.y_GET() == 1.0904646E38F);
            assert(pack.z_GET() == 1.6520096E38F);
            assert(pack.roll_GET() == 1.3140551E38F);
            assert(pack.pitch_GET() == -1.4847995E37F);
            assert(pack.x_GET() == 4.9736915E37F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.pitch_SET(-1.4847995E37F) ;
        p101.usec_SET(485040936705717253L) ;
        p101.z_SET(1.6520096E38F) ;
        p101.x_SET(4.9736915E37F) ;
        p101.y_SET(1.0904646E38F) ;
        p101.roll_SET(1.3140551E38F) ;
        p101.yaw_SET(2.928018E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 2163285039371292069L);
            assert(pack.yaw_GET() == 3.3863978E38F);
            assert(pack.z_GET() == -2.9451653E38F);
            assert(pack.roll_GET() == -1.4151117E38F);
            assert(pack.x_GET() == 8.513286E37F);
            assert(pack.y_GET() == -2.5383215E38F);
            assert(pack.pitch_GET() == -3.2675838E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.z_SET(-2.9451653E38F) ;
        p102.yaw_SET(3.3863978E38F) ;
        p102.roll_SET(-1.4151117E38F) ;
        p102.y_SET(-2.5383215E38F) ;
        p102.usec_SET(2163285039371292069L) ;
        p102.pitch_SET(-3.2675838E38F) ;
        p102.x_SET(8.513286E37F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -2.3139651E38F);
            assert(pack.usec_GET() == 3277150568754112324L);
            assert(pack.y_GET() == -7.8314364E37F);
            assert(pack.z_GET() == 1.770787E38F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.x_SET(-2.3139651E38F) ;
        p103.z_SET(1.770787E38F) ;
        p103.usec_SET(3277150568754112324L) ;
        p103.y_SET(-7.8314364E37F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 1.9787598E38F);
            assert(pack.yaw_GET() == -1.7520166E38F);
            assert(pack.y_GET() == -2.1413578E38F);
            assert(pack.roll_GET() == -2.2467346E38F);
            assert(pack.z_GET() == -1.2578787E38F);
            assert(pack.x_GET() == 1.2391739E38F);
            assert(pack.usec_GET() == 5303550832461440746L);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.roll_SET(-2.2467346E38F) ;
        p104.usec_SET(5303550832461440746L) ;
        p104.y_SET(-2.1413578E38F) ;
        p104.yaw_SET(-1.7520166E38F) ;
        p104.z_SET(-1.2578787E38F) ;
        p104.pitch_SET(1.9787598E38F) ;
        p104.x_SET(1.2391739E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.fields_updated_GET() == (char)9501);
            assert(pack.diff_pressure_GET() == 1.8438116E38F);
            assert(pack.zgyro_GET() == -2.9485964E37F);
            assert(pack.pressure_alt_GET() == -1.9591258E38F);
            assert(pack.time_usec_GET() == 5534688675755600500L);
            assert(pack.xgyro_GET() == -1.6278047E38F);
            assert(pack.yacc_GET() == 1.6397359E38F);
            assert(pack.xacc_GET() == -2.5826904E38F);
            assert(pack.zmag_GET() == 1.5875917E37F);
            assert(pack.ygyro_GET() == -2.6222779E38F);
            assert(pack.abs_pressure_GET() == 2.5367092E38F);
            assert(pack.xmag_GET() == 2.684424E38F);
            assert(pack.temperature_GET() == -3.0317209E38F);
            assert(pack.ymag_GET() == 2.6235763E38F);
            assert(pack.zacc_GET() == 2.5964174E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.pressure_alt_SET(-1.9591258E38F) ;
        p105.zacc_SET(2.5964174E38F) ;
        p105.ygyro_SET(-2.6222779E38F) ;
        p105.yacc_SET(1.6397359E38F) ;
        p105.abs_pressure_SET(2.5367092E38F) ;
        p105.time_usec_SET(5534688675755600500L) ;
        p105.temperature_SET(-3.0317209E38F) ;
        p105.xmag_SET(2.684424E38F) ;
        p105.zgyro_SET(-2.9485964E37F) ;
        p105.xacc_SET(-2.5826904E38F) ;
        p105.fields_updated_SET((char)9501) ;
        p105.ymag_SET(2.6235763E38F) ;
        p105.diff_pressure_SET(1.8438116E38F) ;
        p105.zmag_SET(1.5875917E37F) ;
        p105.xgyro_SET(-1.6278047E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integration_time_us_GET() == 2243666438L);
            assert(pack.integrated_xgyro_GET() == -2.1915306E38F);
            assert(pack.distance_GET() == -2.7868614E37F);
            assert(pack.integrated_zgyro_GET() == 1.136666E38F);
            assert(pack.integrated_ygyro_GET() == 6.635992E37F);
            assert(pack.integrated_y_GET() == -2.406293E38F);
            assert(pack.time_usec_GET() == 8281120304064093235L);
            assert(pack.integrated_x_GET() == 9.728189E37F);
            assert(pack.quality_GET() == (char)109);
            assert(pack.sensor_id_GET() == (char)237);
            assert(pack.temperature_GET() == (short)22900);
            assert(pack.time_delta_distance_us_GET() == 1234374622L);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_ygyro_SET(6.635992E37F) ;
        p106.sensor_id_SET((char)237) ;
        p106.integration_time_us_SET(2243666438L) ;
        p106.quality_SET((char)109) ;
        p106.distance_SET(-2.7868614E37F) ;
        p106.integrated_y_SET(-2.406293E38F) ;
        p106.temperature_SET((short)22900) ;
        p106.time_delta_distance_us_SET(1234374622L) ;
        p106.integrated_zgyro_SET(1.136666E38F) ;
        p106.integrated_xgyro_SET(-2.1915306E38F) ;
        p106.time_usec_SET(8281120304064093235L) ;
        p106.integrated_x_SET(9.728189E37F) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == -5.5525094E37F);
            assert(pack.diff_pressure_GET() == 3.3283803E38F);
            assert(pack.zacc_GET() == -8.1804206E36F);
            assert(pack.ymag_GET() == -2.6048581E38F);
            assert(pack.pressure_alt_GET() == 6.8725703E37F);
            assert(pack.ygyro_GET() == -1.1293128E37F);
            assert(pack.zgyro_GET() == 2.702913E38F);
            assert(pack.yacc_GET() == -1.4145648E38F);
            assert(pack.abs_pressure_GET() == -3.0558314E38F);
            assert(pack.xgyro_GET() == 2.8787828E38F);
            assert(pack.xacc_GET() == 3.0656801E38F);
            assert(pack.zmag_GET() == -2.716765E38F);
            assert(pack.time_usec_GET() == 7991271828292921806L);
            assert(pack.xmag_GET() == 2.5269317E38F);
            assert(pack.fields_updated_GET() == 2264003786L);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.temperature_SET(-5.5525094E37F) ;
        p107.ygyro_SET(-1.1293128E37F) ;
        p107.time_usec_SET(7991271828292921806L) ;
        p107.fields_updated_SET(2264003786L) ;
        p107.zmag_SET(-2.716765E38F) ;
        p107.ymag_SET(-2.6048581E38F) ;
        p107.xmag_SET(2.5269317E38F) ;
        p107.abs_pressure_SET(-3.0558314E38F) ;
        p107.zgyro_SET(2.702913E38F) ;
        p107.zacc_SET(-8.1804206E36F) ;
        p107.yacc_SET(-1.4145648E38F) ;
        p107.pressure_alt_SET(6.8725703E37F) ;
        p107.xacc_SET(3.0656801E38F) ;
        p107.diff_pressure_SET(3.3283803E38F) ;
        p107.xgyro_SET(2.8787828E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == 2.5797714E38F);
            assert(pack.alt_GET() == 7.0386624E37F);
            assert(pack.q4_GET() == -1.631889E38F);
            assert(pack.lat_GET() == 2.841264E38F);
            assert(pack.q1_GET() == -1.0098653E38F);
            assert(pack.q2_GET() == -3.2203716E38F);
            assert(pack.pitch_GET() == 1.7379037E38F);
            assert(pack.vn_GET() == -2.5066048E38F);
            assert(pack.yaw_GET() == -1.5012383E38F);
            assert(pack.ygyro_GET() == -2.1498987E38F);
            assert(pack.std_dev_horz_GET() == 1.833493E38F);
            assert(pack.ve_GET() == 6.7392515E37F);
            assert(pack.std_dev_vert_GET() == 2.858281E37F);
            assert(pack.lon_GET() == -4.2680036E37F);
            assert(pack.vd_GET() == -2.668448E38F);
            assert(pack.xgyro_GET() == 1.3754288E38F);
            assert(pack.zacc_GET() == 1.3187409E38F);
            assert(pack.roll_GET() == -1.139104E38F);
            assert(pack.q3_GET() == -2.9395246E38F);
            assert(pack.xacc_GET() == 6.6481723E37F);
            assert(pack.yacc_GET() == 7.4433163E37F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.ve_SET(6.7392515E37F) ;
        p108.q2_SET(-3.2203716E38F) ;
        p108.zacc_SET(1.3187409E38F) ;
        p108.vn_SET(-2.5066048E38F) ;
        p108.lon_SET(-4.2680036E37F) ;
        p108.lat_SET(2.841264E38F) ;
        p108.q4_SET(-1.631889E38F) ;
        p108.alt_SET(7.0386624E37F) ;
        p108.ygyro_SET(-2.1498987E38F) ;
        p108.vd_SET(-2.668448E38F) ;
        p108.roll_SET(-1.139104E38F) ;
        p108.xacc_SET(6.6481723E37F) ;
        p108.xgyro_SET(1.3754288E38F) ;
        p108.yaw_SET(-1.5012383E38F) ;
        p108.std_dev_vert_SET(2.858281E37F) ;
        p108.zgyro_SET(2.5797714E38F) ;
        p108.std_dev_horz_SET(1.833493E38F) ;
        p108.q1_SET(-1.0098653E38F) ;
        p108.yacc_SET(7.4433163E37F) ;
        p108.pitch_SET(1.7379037E38F) ;
        p108.q3_SET(-2.9395246E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.remnoise_GET() == (char)214);
            assert(pack.remrssi_GET() == (char)6);
            assert(pack.txbuf_GET() == (char)17);
            assert(pack.rssi_GET() == (char)24);
            assert(pack.rxerrors_GET() == (char)13520);
            assert(pack.noise_GET() == (char)126);
            assert(pack.fixed__GET() == (char)50525);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.remnoise_SET((char)214) ;
        p109.txbuf_SET((char)17) ;
        p109.rxerrors_SET((char)13520) ;
        p109.remrssi_SET((char)6) ;
        p109.rssi_SET((char)24) ;
        p109.fixed__SET((char)50525) ;
        p109.noise_SET((char)126) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)70);
            assert(pack.target_network_GET() == (char)137);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)29, (char)7, (char)72, (char)50, (char)82, (char)51, (char)84, (char)113, (char)28, (char)104, (char)13, (char)27, (char)152, (char)134, (char)69, (char)73, (char)234, (char)251, (char)253, (char)162, (char)94, (char)41, (char)13, (char)7, (char)132, (char)14, (char)29, (char)165, (char)255, (char)42, (char)175, (char)28, (char)32, (char)44, (char)188, (char)142, (char)173, (char)248, (char)11, (char)149, (char)74, (char)65, (char)239, (char)189, (char)201, (char)188, (char)57, (char)123, (char)12, (char)64, (char)187, (char)254, (char)122, (char)27, (char)195, (char)132, (char)201, (char)2, (char)70, (char)19, (char)126, (char)231, (char)187, (char)200, (char)94, (char)198, (char)25, (char)108, (char)235, (char)30, (char)210, (char)105, (char)162, (char)196, (char)22, (char)201, (char)125, (char)86, (char)138, (char)245, (char)247, (char)175, (char)12, (char)97, (char)14, (char)254, (char)53, (char)42, (char)31, (char)45, (char)245, (char)175, (char)121, (char)25, (char)127, (char)31, (char)175, (char)174, (char)29, (char)28, (char)213, (char)131, (char)23, (char)69, (char)206, (char)190, (char)42, (char)196, (char)91, (char)84, (char)164, (char)92, (char)37, (char)22, (char)24, (char)71, (char)97, (char)118, (char)195, (char)189, (char)118, (char)108, (char)247, (char)193, (char)165, (char)150, (char)171, (char)199, (char)146, (char)35, (char)241, (char)160, (char)135, (char)245, (char)113, (char)163, (char)131, (char)13, (char)103, (char)91, (char)173, (char)117, (char)229, (char)254, (char)43, (char)62, (char)228, (char)113, (char)110, (char)201, (char)109, (char)236, (char)231, (char)151, (char)83, (char)144, (char)15, (char)250, (char)107, (char)253, (char)190, (char)42, (char)66, (char)227, (char)189, (char)234, (char)5, (char)190, (char)84, (char)133, (char)163, (char)230, (char)23, (char)215, (char)113, (char)16, (char)171, (char)229, (char)57, (char)155, (char)135, (char)160, (char)3, (char)144, (char)127, (char)217, (char)31, (char)25, (char)254, (char)112, (char)30, (char)56, (char)81, (char)2, (char)113, (char)26, (char)2, (char)209, (char)2, (char)197, (char)45, (char)69, (char)111, (char)163, (char)72, (char)131, (char)90, (char)207, (char)109, (char)26, (char)102, (char)143, (char)171, (char)216, (char)105, (char)77, (char)12, (char)65, (char)172, (char)205, (char)38, (char)53, (char)181, (char)24, (char)92, (char)99, (char)75, (char)144, (char)113, (char)96, (char)64, (char)64, (char)41, (char)163, (char)97, (char)43, (char)237, (char)200, (char)233, (char)149, (char)217, (char)220, (char)230, (char)203, (char)107, (char)251, (char)83, (char)63, (char)30, (char)174, (char)92}));
            assert(pack.target_system_GET() == (char)217);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_component_SET((char)70) ;
        p110.target_network_SET((char)137) ;
        p110.target_system_SET((char)217) ;
        p110.payload_SET(new char[] {(char)29, (char)7, (char)72, (char)50, (char)82, (char)51, (char)84, (char)113, (char)28, (char)104, (char)13, (char)27, (char)152, (char)134, (char)69, (char)73, (char)234, (char)251, (char)253, (char)162, (char)94, (char)41, (char)13, (char)7, (char)132, (char)14, (char)29, (char)165, (char)255, (char)42, (char)175, (char)28, (char)32, (char)44, (char)188, (char)142, (char)173, (char)248, (char)11, (char)149, (char)74, (char)65, (char)239, (char)189, (char)201, (char)188, (char)57, (char)123, (char)12, (char)64, (char)187, (char)254, (char)122, (char)27, (char)195, (char)132, (char)201, (char)2, (char)70, (char)19, (char)126, (char)231, (char)187, (char)200, (char)94, (char)198, (char)25, (char)108, (char)235, (char)30, (char)210, (char)105, (char)162, (char)196, (char)22, (char)201, (char)125, (char)86, (char)138, (char)245, (char)247, (char)175, (char)12, (char)97, (char)14, (char)254, (char)53, (char)42, (char)31, (char)45, (char)245, (char)175, (char)121, (char)25, (char)127, (char)31, (char)175, (char)174, (char)29, (char)28, (char)213, (char)131, (char)23, (char)69, (char)206, (char)190, (char)42, (char)196, (char)91, (char)84, (char)164, (char)92, (char)37, (char)22, (char)24, (char)71, (char)97, (char)118, (char)195, (char)189, (char)118, (char)108, (char)247, (char)193, (char)165, (char)150, (char)171, (char)199, (char)146, (char)35, (char)241, (char)160, (char)135, (char)245, (char)113, (char)163, (char)131, (char)13, (char)103, (char)91, (char)173, (char)117, (char)229, (char)254, (char)43, (char)62, (char)228, (char)113, (char)110, (char)201, (char)109, (char)236, (char)231, (char)151, (char)83, (char)144, (char)15, (char)250, (char)107, (char)253, (char)190, (char)42, (char)66, (char)227, (char)189, (char)234, (char)5, (char)190, (char)84, (char)133, (char)163, (char)230, (char)23, (char)215, (char)113, (char)16, (char)171, (char)229, (char)57, (char)155, (char)135, (char)160, (char)3, (char)144, (char)127, (char)217, (char)31, (char)25, (char)254, (char)112, (char)30, (char)56, (char)81, (char)2, (char)113, (char)26, (char)2, (char)209, (char)2, (char)197, (char)45, (char)69, (char)111, (char)163, (char)72, (char)131, (char)90, (char)207, (char)109, (char)26, (char)102, (char)143, (char)171, (char)216, (char)105, (char)77, (char)12, (char)65, (char)172, (char)205, (char)38, (char)53, (char)181, (char)24, (char)92, (char)99, (char)75, (char)144, (char)113, (char)96, (char)64, (char)64, (char)41, (char)163, (char)97, (char)43, (char)237, (char)200, (char)233, (char)149, (char)217, (char)220, (char)230, (char)203, (char)107, (char)251, (char)83, (char)63, (char)30, (char)174, (char)92}, 0) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == 3005566969598144232L);
            assert(pack.tc1_GET() == -8132159301778421933L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(3005566969598144232L) ;
        p111.tc1_SET(-8132159301778421933L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 1275849393L);
            assert(pack.time_usec_GET() == 2289396951264166085L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(2289396951264166085L) ;
        p112.seq_SET(1275849393L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.cog_GET() == (char)10536);
            assert(pack.satellites_visible_GET() == (char)176);
            assert(pack.eph_GET() == (char)18989);
            assert(pack.lat_GET() == -2040095289);
            assert(pack.vel_GET() == (char)58057);
            assert(pack.vn_GET() == (short) -18948);
            assert(pack.epv_GET() == (char)43890);
            assert(pack.lon_GET() == -700759030);
            assert(pack.alt_GET() == 975346705);
            assert(pack.vd_GET() == (short)14388);
            assert(pack.fix_type_GET() == (char)50);
            assert(pack.time_usec_GET() == 2997945455734380778L);
            assert(pack.ve_GET() == (short)900);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.cog_SET((char)10536) ;
        p113.alt_SET(975346705) ;
        p113.lat_SET(-2040095289) ;
        p113.vel_SET((char)58057) ;
        p113.vd_SET((short)14388) ;
        p113.satellites_visible_SET((char)176) ;
        p113.epv_SET((char)43890) ;
        p113.ve_SET((short)900) ;
        p113.lon_SET(-700759030) ;
        p113.time_usec_SET(2997945455734380778L) ;
        p113.vn_SET((short) -18948) ;
        p113.fix_type_SET((char)50) ;
        p113.eph_SET((char)18989) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.quality_GET() == (char)238);
            assert(pack.time_delta_distance_us_GET() == 3530830211L);
            assert(pack.integrated_ygyro_GET() == 2.8991816E38F);
            assert(pack.distance_GET() == 2.1505893E38F);
            assert(pack.sensor_id_GET() == (char)193);
            assert(pack.time_usec_GET() == 5535115614336208517L);
            assert(pack.integration_time_us_GET() == 894090097L);
            assert(pack.integrated_y_GET() == -5.2581625E37F);
            assert(pack.integrated_xgyro_GET() == -1.9231193E38F);
            assert(pack.temperature_GET() == (short)21499);
            assert(pack.integrated_x_GET() == -6.2875546E37F);
            assert(pack.integrated_zgyro_GET() == 1.4578431E38F);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_y_SET(-5.2581625E37F) ;
        p114.integrated_x_SET(-6.2875546E37F) ;
        p114.temperature_SET((short)21499) ;
        p114.integration_time_us_SET(894090097L) ;
        p114.integrated_ygyro_SET(2.8991816E38F) ;
        p114.sensor_id_SET((char)193) ;
        p114.time_usec_SET(5535115614336208517L) ;
        p114.quality_SET((char)238) ;
        p114.integrated_xgyro_SET(-1.9231193E38F) ;
        p114.distance_SET(2.1505893E38F) ;
        p114.integrated_zgyro_SET(1.4578431E38F) ;
        p114.time_delta_distance_us_SET(3530830211L) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short)9071);
            assert(pack.ind_airspeed_GET() == (char)739);
            assert(pack.pitchspeed_GET() == 3.341492E38F);
            assert(pack.yawspeed_GET() == -4.6118584E37F);
            assert(pack.alt_GET() == 1016868711);
            assert(pack.zacc_GET() == (short) -3637);
            assert(pack.vy_GET() == (short)984);
            assert(pack.vx_GET() == (short) -19619);
            assert(pack.lon_GET() == -1258934743);
            assert(pack.true_airspeed_GET() == (char)4685);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {1.0466079E38F, 3.2890103E38F, 2.9066269E38F, -2.5768558E38F}));
            assert(pack.vz_GET() == (short)19937);
            assert(pack.time_usec_GET() == 7293160053934907465L);
            assert(pack.yacc_GET() == (short) -3035);
            assert(pack.lat_GET() == -1995276997);
            assert(pack.rollspeed_GET() == -2.046064E38F);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.lat_SET(-1995276997) ;
        p115.time_usec_SET(7293160053934907465L) ;
        p115.xacc_SET((short)9071) ;
        p115.alt_SET(1016868711) ;
        p115.vy_SET((short)984) ;
        p115.ind_airspeed_SET((char)739) ;
        p115.lon_SET(-1258934743) ;
        p115.yacc_SET((short) -3035) ;
        p115.true_airspeed_SET((char)4685) ;
        p115.rollspeed_SET(-2.046064E38F) ;
        p115.attitude_quaternion_SET(new float[] {1.0466079E38F, 3.2890103E38F, 2.9066269E38F, -2.5768558E38F}, 0) ;
        p115.vz_SET((short)19937) ;
        p115.zacc_SET((short) -3637) ;
        p115.yawspeed_SET(-4.6118584E37F) ;
        p115.vx_SET((short) -19619) ;
        p115.pitchspeed_SET(3.341492E38F) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == (short) -925);
            assert(pack.yacc_GET() == (short)6262);
            assert(pack.xgyro_GET() == (short) -25394);
            assert(pack.xmag_GET() == (short)28418);
            assert(pack.zgyro_GET() == (short) -16009);
            assert(pack.time_boot_ms_GET() == 3934871599L);
            assert(pack.zacc_GET() == (short)23408);
            assert(pack.xacc_GET() == (short) -14794);
            assert(pack.ymag_GET() == (short) -31437);
            assert(pack.ygyro_GET() == (short)22572);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.xgyro_SET((short) -25394) ;
        p116.ymag_SET((short) -31437) ;
        p116.ygyro_SET((short)22572) ;
        p116.time_boot_ms_SET(3934871599L) ;
        p116.xmag_SET((short)28418) ;
        p116.yacc_SET((short)6262) ;
        p116.zgyro_SET((short) -16009) ;
        p116.xacc_SET((short) -14794) ;
        p116.zacc_SET((short)23408) ;
        p116.zmag_SET((short) -925) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_GET() == (char)40255);
            assert(pack.target_component_GET() == (char)36);
            assert(pack.target_system_GET() == (char)61);
            assert(pack.start_GET() == (char)64764);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.end_SET((char)40255) ;
        p117.target_system_SET((char)61) ;
        p117.start_SET((char)64764) ;
        p117.target_component_SET((char)36) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.time_utc_GET() == 3679151376L);
            assert(pack.last_log_num_GET() == (char)39692);
            assert(pack.id_GET() == (char)22806);
            assert(pack.num_logs_GET() == (char)48212);
            assert(pack.size_GET() == 2385486348L);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.last_log_num_SET((char)39692) ;
        p118.size_SET(2385486348L) ;
        p118.id_SET((char)22806) ;
        p118.time_utc_SET(3679151376L) ;
        p118.num_logs_SET((char)48212) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)21784);
            assert(pack.ofs_GET() == 1446887190L);
            assert(pack.target_system_GET() == (char)21);
            assert(pack.count_GET() == 653006828L);
            assert(pack.target_component_GET() == (char)166);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)21) ;
        p119.count_SET(653006828L) ;
        p119.id_SET((char)21784) ;
        p119.ofs_SET(1446887190L) ;
        p119.target_component_SET((char)166) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)168, (char)219, (char)153, (char)16, (char)215, (char)6, (char)90, (char)91, (char)222, (char)8, (char)95, (char)226, (char)149, (char)242, (char)85, (char)226, (char)82, (char)95, (char)128, (char)188, (char)89, (char)153, (char)0, (char)102, (char)183, (char)123, (char)126, (char)6, (char)199, (char)2, (char)92, (char)69, (char)16, (char)106, (char)248, (char)65, (char)91, (char)19, (char)243, (char)68, (char)202, (char)233, (char)33, (char)131, (char)13, (char)150, (char)158, (char)228, (char)195, (char)86, (char)20, (char)100, (char)191, (char)95, (char)10, (char)191, (char)118, (char)130, (char)164, (char)162, (char)227, (char)151, (char)59, (char)207, (char)78, (char)138, (char)6, (char)24, (char)148, (char)103, (char)191, (char)105, (char)23, (char)81, (char)188, (char)73, (char)122, (char)153, (char)61, (char)32, (char)129, (char)112, (char)31, (char)174, (char)111, (char)198, (char)236, (char)71, (char)32, (char)100}));
            assert(pack.id_GET() == (char)56043);
            assert(pack.count_GET() == (char)235);
            assert(pack.ofs_GET() == 3972474302L);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)56043) ;
        p120.ofs_SET(3972474302L) ;
        p120.data__SET(new char[] {(char)168, (char)219, (char)153, (char)16, (char)215, (char)6, (char)90, (char)91, (char)222, (char)8, (char)95, (char)226, (char)149, (char)242, (char)85, (char)226, (char)82, (char)95, (char)128, (char)188, (char)89, (char)153, (char)0, (char)102, (char)183, (char)123, (char)126, (char)6, (char)199, (char)2, (char)92, (char)69, (char)16, (char)106, (char)248, (char)65, (char)91, (char)19, (char)243, (char)68, (char)202, (char)233, (char)33, (char)131, (char)13, (char)150, (char)158, (char)228, (char)195, (char)86, (char)20, (char)100, (char)191, (char)95, (char)10, (char)191, (char)118, (char)130, (char)164, (char)162, (char)227, (char)151, (char)59, (char)207, (char)78, (char)138, (char)6, (char)24, (char)148, (char)103, (char)191, (char)105, (char)23, (char)81, (char)188, (char)73, (char)122, (char)153, (char)61, (char)32, (char)129, (char)112, (char)31, (char)174, (char)111, (char)198, (char)236, (char)71, (char)32, (char)100}, 0) ;
        p120.count_SET((char)235) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)81);
            assert(pack.target_system_GET() == (char)27);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)81) ;
        p121.target_system_SET((char)27) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)99);
            assert(pack.target_system_GET() == (char)67);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)67) ;
        p122.target_component_SET((char)99) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)85, (char)245, (char)135, (char)146, (char)80, (char)145, (char)170, (char)169, (char)172, (char)204, (char)204, (char)221, (char)87, (char)31, (char)187, (char)158, (char)138, (char)15, (char)60, (char)46, (char)125, (char)218, (char)242, (char)143, (char)234, (char)32, (char)18, (char)204, (char)165, (char)35, (char)181, (char)202, (char)143, (char)76, (char)103, (char)69, (char)121, (char)191, (char)99, (char)193, (char)112, (char)2, (char)238, (char)148, (char)72, (char)14, (char)223, (char)122, (char)15, (char)154, (char)4, (char)135, (char)195, (char)13, (char)208, (char)234, (char)124, (char)38, (char)127, (char)119, (char)159, (char)110, (char)145, (char)26, (char)59, (char)15, (char)238, (char)140, (char)220, (char)234, (char)152, (char)73, (char)232, (char)62, (char)17, (char)232, (char)215, (char)117, (char)243, (char)128, (char)125, (char)113, (char)0, (char)168, (char)167, (char)82, (char)168, (char)49, (char)212, (char)214, (char)92, (char)11, (char)125, (char)192, (char)196, (char)192, (char)76, (char)105, (char)15, (char)217, (char)47, (char)134, (char)209, (char)232, (char)58, (char)49, (char)28, (char)54, (char)253, (char)49}));
            assert(pack.target_system_GET() == (char)105);
            assert(pack.len_GET() == (char)133);
            assert(pack.target_component_GET() == (char)122);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.len_SET((char)133) ;
        p123.data__SET(new char[] {(char)85, (char)245, (char)135, (char)146, (char)80, (char)145, (char)170, (char)169, (char)172, (char)204, (char)204, (char)221, (char)87, (char)31, (char)187, (char)158, (char)138, (char)15, (char)60, (char)46, (char)125, (char)218, (char)242, (char)143, (char)234, (char)32, (char)18, (char)204, (char)165, (char)35, (char)181, (char)202, (char)143, (char)76, (char)103, (char)69, (char)121, (char)191, (char)99, (char)193, (char)112, (char)2, (char)238, (char)148, (char)72, (char)14, (char)223, (char)122, (char)15, (char)154, (char)4, (char)135, (char)195, (char)13, (char)208, (char)234, (char)124, (char)38, (char)127, (char)119, (char)159, (char)110, (char)145, (char)26, (char)59, (char)15, (char)238, (char)140, (char)220, (char)234, (char)152, (char)73, (char)232, (char)62, (char)17, (char)232, (char)215, (char)117, (char)243, (char)128, (char)125, (char)113, (char)0, (char)168, (char)167, (char)82, (char)168, (char)49, (char)212, (char)214, (char)92, (char)11, (char)125, (char)192, (char)196, (char)192, (char)76, (char)105, (char)15, (char)217, (char)47, (char)134, (char)209, (char)232, (char)58, (char)49, (char)28, (char)54, (char)253, (char)49}, 0) ;
        p123.target_component_SET((char)122) ;
        p123.target_system_SET((char)105) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 1230550472);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
            assert(pack.cog_GET() == (char)57838);
            assert(pack.lon_GET() == -2089432643);
            assert(pack.eph_GET() == (char)7165);
            assert(pack.time_usec_GET() == 4082665656511442285L);
            assert(pack.vel_GET() == (char)27861);
            assert(pack.satellites_visible_GET() == (char)248);
            assert(pack.epv_GET() == (char)7160);
            assert(pack.dgps_numch_GET() == (char)172);
            assert(pack.lat_GET() == -647798534);
            assert(pack.dgps_age_GET() == 3767311877L);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.dgps_numch_SET((char)172) ;
        p124.dgps_age_SET(3767311877L) ;
        p124.vel_SET((char)27861) ;
        p124.lon_SET(-2089432643) ;
        p124.eph_SET((char)7165) ;
        p124.alt_SET(1230550472) ;
        p124.lat_SET(-647798534) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED) ;
        p124.epv_SET((char)7160) ;
        p124.time_usec_SET(4082665656511442285L) ;
        p124.satellites_visible_SET((char)248) ;
        p124.cog_SET((char)57838) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED));
            assert(pack.Vservo_GET() == (char)8140);
            assert(pack.Vcc_GET() == (char)58994);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)58994) ;
        p125.Vservo_SET((char)8140) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED)) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.timeout_GET() == (char)52251);
            assert(pack.baudrate_GET() == 3802281781L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)132, (char)106, (char)133, (char)74, (char)7, (char)242, (char)140, (char)165, (char)184, (char)46, (char)186, (char)20, (char)31, (char)240, (char)146, (char)179, (char)255, (char)97, (char)85, (char)135, (char)65, (char)243, (char)174, (char)96, (char)173, (char)18, (char)79, (char)51, (char)88, (char)54, (char)6, (char)76, (char)238, (char)97, (char)162, (char)238, (char)236, (char)249, (char)88, (char)168, (char)30, (char)178, (char)67, (char)129, (char)73, (char)70, (char)62, (char)251, (char)108, (char)24, (char)38, (char)20, (char)135, (char)205, (char)98, (char)167, (char)235, (char)202, (char)75, (char)24, (char)80, (char)150, (char)202, (char)92, (char)132, (char)52, (char)209, (char)65, (char)159, (char)240}));
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY));
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
            assert(pack.count_GET() == (char)12);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.data__SET(new char[] {(char)132, (char)106, (char)133, (char)74, (char)7, (char)242, (char)140, (char)165, (char)184, (char)46, (char)186, (char)20, (char)31, (char)240, (char)146, (char)179, (char)255, (char)97, (char)85, (char)135, (char)65, (char)243, (char)174, (char)96, (char)173, (char)18, (char)79, (char)51, (char)88, (char)54, (char)6, (char)76, (char)238, (char)97, (char)162, (char)238, (char)236, (char)249, (char)88, (char)168, (char)30, (char)178, (char)67, (char)129, (char)73, (char)70, (char)62, (char)251, (char)108, (char)24, (char)38, (char)20, (char)135, (char)205, (char)98, (char)167, (char)235, (char)202, (char)75, (char)24, (char)80, (char)150, (char)202, (char)92, (char)132, (char)52, (char)209, (char)65, (char)159, (char)240}, 0) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL) ;
        p126.timeout_SET((char)52251) ;
        p126.baudrate_SET(3802281781L) ;
        p126.count_SET((char)12) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY)) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_b_mm_GET() == 1986467030);
            assert(pack.baseline_c_mm_GET() == 63631012);
            assert(pack.wn_GET() == (char)50006);
            assert(pack.iar_num_hypotheses_GET() == -599326179);
            assert(pack.nsats_GET() == (char)119);
            assert(pack.rtk_health_GET() == (char)248);
            assert(pack.baseline_a_mm_GET() == 581796587);
            assert(pack.baseline_coords_type_GET() == (char)17);
            assert(pack.time_last_baseline_ms_GET() == 2020434660L);
            assert(pack.tow_GET() == 1639108917L);
            assert(pack.rtk_rate_GET() == (char)255);
            assert(pack.accuracy_GET() == 1200607975L);
            assert(pack.rtk_receiver_id_GET() == (char)111);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.rtk_rate_SET((char)255) ;
        p127.rtk_receiver_id_SET((char)111) ;
        p127.time_last_baseline_ms_SET(2020434660L) ;
        p127.baseline_coords_type_SET((char)17) ;
        p127.rtk_health_SET((char)248) ;
        p127.baseline_c_mm_SET(63631012) ;
        p127.nsats_SET((char)119) ;
        p127.iar_num_hypotheses_SET(-599326179) ;
        p127.wn_SET((char)50006) ;
        p127.baseline_a_mm_SET(581796587) ;
        p127.accuracy_SET(1200607975L) ;
        p127.tow_SET(1639108917L) ;
        p127.baseline_b_mm_SET(1986467030) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.wn_GET() == (char)40894);
            assert(pack.baseline_c_mm_GET() == -1791191713);
            assert(pack.nsats_GET() == (char)236);
            assert(pack.tow_GET() == 1754917817L);
            assert(pack.baseline_b_mm_GET() == 1601438675);
            assert(pack.iar_num_hypotheses_GET() == 1111575785);
            assert(pack.rtk_rate_GET() == (char)99);
            assert(pack.rtk_health_GET() == (char)142);
            assert(pack.accuracy_GET() == 932099986L);
            assert(pack.baseline_coords_type_GET() == (char)80);
            assert(pack.rtk_receiver_id_GET() == (char)163);
            assert(pack.time_last_baseline_ms_GET() == 1584505502L);
            assert(pack.baseline_a_mm_GET() == -1969288501);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.iar_num_hypotheses_SET(1111575785) ;
        p128.rtk_receiver_id_SET((char)163) ;
        p128.tow_SET(1754917817L) ;
        p128.baseline_c_mm_SET(-1791191713) ;
        p128.wn_SET((char)40894) ;
        p128.baseline_b_mm_SET(1601438675) ;
        p128.baseline_coords_type_SET((char)80) ;
        p128.baseline_a_mm_SET(-1969288501) ;
        p128.nsats_SET((char)236) ;
        p128.accuracy_SET(932099986L) ;
        p128.time_last_baseline_ms_SET(1584505502L) ;
        p128.rtk_health_SET((char)142) ;
        p128.rtk_rate_SET((char)99) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short) -24530);
            assert(pack.zmag_GET() == (short) -8117);
            assert(pack.zgyro_GET() == (short)30458);
            assert(pack.zacc_GET() == (short) -12026);
            assert(pack.xgyro_GET() == (short)9719);
            assert(pack.xacc_GET() == (short) -26502);
            assert(pack.ymag_GET() == (short)31717);
            assert(pack.time_boot_ms_GET() == 2368227060L);
            assert(pack.xmag_GET() == (short)11197);
            assert(pack.yacc_GET() == (short)17547);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.yacc_SET((short)17547) ;
        p129.xacc_SET((short) -26502) ;
        p129.time_boot_ms_SET(2368227060L) ;
        p129.ymag_SET((short)31717) ;
        p129.zmag_SET((short) -8117) ;
        p129.xgyro_SET((short)9719) ;
        p129.zacc_SET((short) -12026) ;
        p129.zgyro_SET((short)30458) ;
        p129.xmag_SET((short)11197) ;
        p129.ygyro_SET((short) -24530) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.payload_GET() == (char)70);
            assert(pack.size_GET() == 1893025942L);
            assert(pack.height_GET() == (char)62039);
            assert(pack.width_GET() == (char)14538);
            assert(pack.type_GET() == (char)177);
            assert(pack.packets_GET() == (char)56585);
            assert(pack.jpg_quality_GET() == (char)237);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.size_SET(1893025942L) ;
        p130.type_SET((char)177) ;
        p130.height_SET((char)62039) ;
        p130.width_SET((char)14538) ;
        p130.packets_SET((char)56585) ;
        p130.jpg_quality_SET((char)237) ;
        p130.payload_SET((char)70) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)224, (char)221, (char)23, (char)176, (char)160, (char)229, (char)155, (char)70, (char)174, (char)25, (char)2, (char)49, (char)106, (char)48, (char)101, (char)13, (char)85, (char)88, (char)12, (char)181, (char)35, (char)119, (char)189, (char)240, (char)2, (char)99, (char)212, (char)10, (char)245, (char)61, (char)239, (char)152, (char)8, (char)203, (char)219, (char)8, (char)66, (char)161, (char)56, (char)111, (char)165, (char)220, (char)52, (char)64, (char)233, (char)103, (char)76, (char)64, (char)253, (char)148, (char)8, (char)204, (char)204, (char)0, (char)175, (char)36, (char)61, (char)168, (char)56, (char)229, (char)175, (char)130, (char)123, (char)194, (char)14, (char)182, (char)229, (char)212, (char)193, (char)24, (char)21, (char)25, (char)166, (char)171, (char)85, (char)154, (char)99, (char)199, (char)233, (char)162, (char)225, (char)85, (char)14, (char)129, (char)90, (char)188, (char)236, (char)62, (char)245, (char)50, (char)82, (char)26, (char)74, (char)7, (char)106, (char)114, (char)79, (char)22, (char)206, (char)165, (char)34, (char)21, (char)130, (char)69, (char)138, (char)160, (char)252, (char)5, (char)244, (char)45, (char)223, (char)133, (char)229, (char)209, (char)188, (char)80, (char)109, (char)56, (char)227, (char)174, (char)253, (char)183, (char)156, (char)43, (char)95, (char)160, (char)213, (char)134, (char)129, (char)228, (char)45, (char)135, (char)253, (char)199, (char)199, (char)122, (char)1, (char)11, (char)46, (char)242, (char)168, (char)123, (char)40, (char)55, (char)144, (char)6, (char)84, (char)251, (char)133, (char)84, (char)129, (char)202, (char)121, (char)116, (char)205, (char)108, (char)239, (char)196, (char)96, (char)168, (char)240, (char)183, (char)153, (char)133, (char)76, (char)97, (char)172, (char)123, (char)242, (char)188, (char)189, (char)110, (char)5, (char)62, (char)237, (char)154, (char)42, (char)9, (char)67, (char)122, (char)167, (char)5, (char)184, (char)196, (char)117, (char)126, (char)167, (char)48, (char)27, (char)140, (char)219, (char)123, (char)226, (char)57, (char)14, (char)173, (char)36, (char)125, (char)148, (char)228, (char)120, (char)146, (char)226, (char)255, (char)75, (char)139, (char)151, (char)98, (char)4, (char)166, (char)138, (char)32, (char)49, (char)235, (char)70, (char)46, (char)69, (char)112, (char)43, (char)174, (char)29, (char)29, (char)120, (char)222, (char)87, (char)47, (char)190, (char)83, (char)180, (char)99, (char)101, (char)176, (char)156, (char)138, (char)243, (char)116, (char)171, (char)18, (char)20, (char)62, (char)198, (char)66, (char)37, (char)56, (char)94, (char)248, (char)95, (char)44, (char)64, (char)79, (char)232, (char)193, (char)182}));
            assert(pack.seqnr_GET() == (char)64839);
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)64839) ;
        p131.data__SET(new char[] {(char)224, (char)221, (char)23, (char)176, (char)160, (char)229, (char)155, (char)70, (char)174, (char)25, (char)2, (char)49, (char)106, (char)48, (char)101, (char)13, (char)85, (char)88, (char)12, (char)181, (char)35, (char)119, (char)189, (char)240, (char)2, (char)99, (char)212, (char)10, (char)245, (char)61, (char)239, (char)152, (char)8, (char)203, (char)219, (char)8, (char)66, (char)161, (char)56, (char)111, (char)165, (char)220, (char)52, (char)64, (char)233, (char)103, (char)76, (char)64, (char)253, (char)148, (char)8, (char)204, (char)204, (char)0, (char)175, (char)36, (char)61, (char)168, (char)56, (char)229, (char)175, (char)130, (char)123, (char)194, (char)14, (char)182, (char)229, (char)212, (char)193, (char)24, (char)21, (char)25, (char)166, (char)171, (char)85, (char)154, (char)99, (char)199, (char)233, (char)162, (char)225, (char)85, (char)14, (char)129, (char)90, (char)188, (char)236, (char)62, (char)245, (char)50, (char)82, (char)26, (char)74, (char)7, (char)106, (char)114, (char)79, (char)22, (char)206, (char)165, (char)34, (char)21, (char)130, (char)69, (char)138, (char)160, (char)252, (char)5, (char)244, (char)45, (char)223, (char)133, (char)229, (char)209, (char)188, (char)80, (char)109, (char)56, (char)227, (char)174, (char)253, (char)183, (char)156, (char)43, (char)95, (char)160, (char)213, (char)134, (char)129, (char)228, (char)45, (char)135, (char)253, (char)199, (char)199, (char)122, (char)1, (char)11, (char)46, (char)242, (char)168, (char)123, (char)40, (char)55, (char)144, (char)6, (char)84, (char)251, (char)133, (char)84, (char)129, (char)202, (char)121, (char)116, (char)205, (char)108, (char)239, (char)196, (char)96, (char)168, (char)240, (char)183, (char)153, (char)133, (char)76, (char)97, (char)172, (char)123, (char)242, (char)188, (char)189, (char)110, (char)5, (char)62, (char)237, (char)154, (char)42, (char)9, (char)67, (char)122, (char)167, (char)5, (char)184, (char)196, (char)117, (char)126, (char)167, (char)48, (char)27, (char)140, (char)219, (char)123, (char)226, (char)57, (char)14, (char)173, (char)36, (char)125, (char)148, (char)228, (char)120, (char)146, (char)226, (char)255, (char)75, (char)139, (char)151, (char)98, (char)4, (char)166, (char)138, (char)32, (char)49, (char)235, (char)70, (char)46, (char)69, (char)112, (char)43, (char)174, (char)29, (char)29, (char)120, (char)222, (char)87, (char)47, (char)190, (char)83, (char)180, (char)99, (char)101, (char)176, (char)156, (char)138, (char)243, (char)116, (char)171, (char)18, (char)20, (char)62, (char)198, (char)66, (char)37, (char)56, (char)94, (char)248, (char)95, (char)44, (char)64, (char)79, (char)232, (char)193, (char)182}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2815849203L);
            assert(pack.id_GET() == (char)4);
            assert(pack.current_distance_GET() == (char)47778);
            assert(pack.min_distance_GET() == (char)58588);
            assert(pack.max_distance_GET() == (char)35889);
            assert(pack.covariance_GET() == (char)35);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.current_distance_SET((char)47778) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90) ;
        p132.min_distance_SET((char)58588) ;
        p132.time_boot_ms_SET(2815849203L) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p132.id_SET((char)4) ;
        p132.covariance_SET((char)35) ;
        p132.max_distance_SET((char)35889) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1441142205);
            assert(pack.lat_GET() == 1646072473);
            assert(pack.mask_GET() == 7553954194279647100L);
            assert(pack.grid_spacing_GET() == (char)16398);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.grid_spacing_SET((char)16398) ;
        p133.lat_SET(1646072473) ;
        p133.lon_SET(-1441142205) ;
        p133.mask_SET(7553954194279647100L) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -1190, (short) -29192, (short) -15447, (short) -24086, (short)14504, (short)26813, (short) -11268, (short) -6518, (short) -11961, (short) -6122, (short)6746, (short)9602, (short) -2151, (short)25641, (short)6577, (short)26620}));
            assert(pack.lat_GET() == 955023872);
            assert(pack.lon_GET() == 563117282);
            assert(pack.grid_spacing_GET() == (char)47609);
            assert(pack.gridbit_GET() == (char)212);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(955023872) ;
        p134.grid_spacing_SET((char)47609) ;
        p134.lon_SET(563117282) ;
        p134.gridbit_SET((char)212) ;
        p134.data__SET(new short[] {(short) -1190, (short) -29192, (short) -15447, (short) -24086, (short)14504, (short)26813, (short) -11268, (short) -6518, (short) -11961, (short) -6122, (short)6746, (short)9602, (short) -2151, (short)25641, (short)6577, (short)26620}, 0) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1396355150);
            assert(pack.lat_GET() == -411218800);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(-1396355150) ;
        p135.lat_SET(-411218800) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.terrain_height_GET() == 3.0093885E38F);
            assert(pack.current_height_GET() == -2.6681988E38F);
            assert(pack.pending_GET() == (char)25130);
            assert(pack.loaded_GET() == (char)43993);
            assert(pack.lon_GET() == -1364769504);
            assert(pack.lat_GET() == -5532404);
            assert(pack.spacing_GET() == (char)6431);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(-5532404) ;
        p136.pending_SET((char)25130) ;
        p136.lon_SET(-1364769504) ;
        p136.current_height_SET(-2.6681988E38F) ;
        p136.terrain_height_SET(3.0093885E38F) ;
        p136.spacing_SET((char)6431) ;
        p136.loaded_SET((char)43993) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3855005488L);
            assert(pack.press_diff_GET() == -2.9431062E38F);
            assert(pack.temperature_GET() == (short) -8807);
            assert(pack.press_abs_GET() == -2.2958396E38F);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_abs_SET(-2.2958396E38F) ;
        p137.temperature_SET((short) -8807) ;
        p137.time_boot_ms_SET(3855005488L) ;
        p137.press_diff_SET(-2.9431062E38F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -4.013178E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.8640792E38F, -3.0402683E38F, 2.6320473E38F, 8.2234954E37F}));
            assert(pack.x_GET() == 6.7020895E37F);
            assert(pack.y_GET() == -2.468678E38F);
            assert(pack.time_usec_GET() == 7002484458559550500L);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.x_SET(6.7020895E37F) ;
        p138.z_SET(-4.013178E37F) ;
        p138.time_usec_SET(7002484458559550500L) ;
        p138.q_SET(new float[] {-1.8640792E38F, -3.0402683E38F, 2.6320473E38F, 8.2234954E37F}, 0) ;
        p138.y_SET(-2.468678E38F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)130);
            assert(pack.target_system_GET() == (char)252);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.8610501E37F, -3.330673E38F, 3.2325908E38F, 2.2953812E38F, 3.1055886E38F, -1.859278E38F, 2.3169968E38F, 1.5203383E38F}));
            assert(pack.time_usec_GET() == 4644999128208766451L);
            assert(pack.group_mlx_GET() == (char)74);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.controls_SET(new float[] {2.8610501E37F, -3.330673E38F, 3.2325908E38F, 2.2953812E38F, 3.1055886E38F, -1.859278E38F, 2.3169968E38F, 1.5203383E38F}, 0) ;
        p139.group_mlx_SET((char)74) ;
        p139.time_usec_SET(4644999128208766451L) ;
        p139.target_component_SET((char)130) ;
        p139.target_system_SET((char)252) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5329252768201084395L);
            assert(pack.group_mlx_GET() == (char)208);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.1537868E38F, 1.1716077E38F, -1.667205E38F, -2.0424174E38F, 1.6237048E38F, -2.4428933E38F, -3.4394156E36F, -2.7033853E37F}));
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(5329252768201084395L) ;
        p140.controls_SET(new float[] {-2.1537868E38F, 1.1716077E38F, -1.667205E38F, -2.0424174E38F, 1.6237048E38F, -2.4428933E38F, -3.4394156E36F, -2.7033853E37F}, 0) ;
        p140.group_mlx_SET((char)208) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_relative_GET() == 3.3792628E38F);
            assert(pack.altitude_local_GET() == 2.4892246E38F);
            assert(pack.altitude_terrain_GET() == 9.488383E37F);
            assert(pack.altitude_amsl_GET() == 3.2398146E38F);
            assert(pack.bottom_clearance_GET() == 2.3550204E38F);
            assert(pack.altitude_monotonic_GET() == -1.2489205E38F);
            assert(pack.time_usec_GET() == 4388519152116436159L);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_relative_SET(3.3792628E38F) ;
        p141.time_usec_SET(4388519152116436159L) ;
        p141.altitude_local_SET(2.4892246E38F) ;
        p141.altitude_terrain_SET(9.488383E37F) ;
        p141.bottom_clearance_SET(2.3550204E38F) ;
        p141.altitude_monotonic_SET(-1.2489205E38F) ;
        p141.altitude_amsl_SET(3.2398146E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.transfer_type_GET() == (char)5);
            assert(pack.request_id_GET() == (char)64);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)152, (char)149, (char)250, (char)79, (char)138, (char)193, (char)152, (char)162, (char)93, (char)196, (char)31, (char)73, (char)119, (char)188, (char)214, (char)128, (char)214, (char)228, (char)25, (char)69, (char)133, (char)164, (char)40, (char)62, (char)30, (char)172, (char)254, (char)9, (char)90, (char)209, (char)115, (char)213, (char)27, (char)33, (char)46, (char)181, (char)194, (char)227, (char)195, (char)225, (char)238, (char)193, (char)106, (char)108, (char)142, (char)224, (char)246, (char)233, (char)83, (char)24, (char)5, (char)170, (char)197, (char)98, (char)19, (char)128, (char)180, (char)183, (char)112, (char)214, (char)211, (char)141, (char)231, (char)31, (char)40, (char)70, (char)52, (char)249, (char)209, (char)245, (char)195, (char)117, (char)61, (char)194, (char)140, (char)27, (char)60, (char)156, (char)22, (char)80, (char)174, (char)67, (char)181, (char)126, (char)73, (char)142, (char)55, (char)25, (char)244, (char)222, (char)205, (char)203, (char)193, (char)125, (char)95, (char)189, (char)14, (char)143, (char)253, (char)105, (char)135, (char)235, (char)192, (char)136, (char)8, (char)76, (char)21, (char)104, (char)220, (char)205, (char)225, (char)153, (char)37, (char)111, (char)121, (char)161, (char)27, (char)79, (char)237, (char)197}));
            assert(pack.uri_type_GET() == (char)215);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)63, (char)222, (char)123, (char)105, (char)184, (char)203, (char)150, (char)86, (char)151, (char)92, (char)199, (char)241, (char)104, (char)237, (char)185, (char)106, (char)113, (char)117, (char)9, (char)57, (char)2, (char)209, (char)225, (char)108, (char)4, (char)240, (char)116, (char)1, (char)158, (char)200, (char)123, (char)85, (char)18, (char)45, (char)125, (char)90, (char)49, (char)179, (char)159, (char)68, (char)55, (char)117, (char)195, (char)230, (char)85, (char)152, (char)131, (char)37, (char)19, (char)192, (char)156, (char)135, (char)86, (char)214, (char)206, (char)83, (char)115, (char)52, (char)245, (char)142, (char)233, (char)63, (char)64, (char)49, (char)222, (char)23, (char)16, (char)186, (char)47, (char)92, (char)156, (char)228, (char)42, (char)5, (char)217, (char)183, (char)44, (char)241, (char)47, (char)145, (char)237, (char)74, (char)98, (char)224, (char)188, (char)39, (char)198, (char)224, (char)222, (char)58, (char)121, (char)191, (char)107, (char)52, (char)32, (char)85, (char)225, (char)221, (char)35, (char)6, (char)187, (char)133, (char)135, (char)25, (char)190, (char)87, (char)220, (char)91, (char)152, (char)174, (char)168, (char)148, (char)237, (char)118, (char)250, (char)204, (char)28, (char)181, (char)92, (char)240}));
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_type_SET((char)215) ;
        p142.request_id_SET((char)64) ;
        p142.uri_SET(new char[] {(char)63, (char)222, (char)123, (char)105, (char)184, (char)203, (char)150, (char)86, (char)151, (char)92, (char)199, (char)241, (char)104, (char)237, (char)185, (char)106, (char)113, (char)117, (char)9, (char)57, (char)2, (char)209, (char)225, (char)108, (char)4, (char)240, (char)116, (char)1, (char)158, (char)200, (char)123, (char)85, (char)18, (char)45, (char)125, (char)90, (char)49, (char)179, (char)159, (char)68, (char)55, (char)117, (char)195, (char)230, (char)85, (char)152, (char)131, (char)37, (char)19, (char)192, (char)156, (char)135, (char)86, (char)214, (char)206, (char)83, (char)115, (char)52, (char)245, (char)142, (char)233, (char)63, (char)64, (char)49, (char)222, (char)23, (char)16, (char)186, (char)47, (char)92, (char)156, (char)228, (char)42, (char)5, (char)217, (char)183, (char)44, (char)241, (char)47, (char)145, (char)237, (char)74, (char)98, (char)224, (char)188, (char)39, (char)198, (char)224, (char)222, (char)58, (char)121, (char)191, (char)107, (char)52, (char)32, (char)85, (char)225, (char)221, (char)35, (char)6, (char)187, (char)133, (char)135, (char)25, (char)190, (char)87, (char)220, (char)91, (char)152, (char)174, (char)168, (char)148, (char)237, (char)118, (char)250, (char)204, (char)28, (char)181, (char)92, (char)240}, 0) ;
        p142.transfer_type_SET((char)5) ;
        p142.storage_SET(new char[] {(char)152, (char)149, (char)250, (char)79, (char)138, (char)193, (char)152, (char)162, (char)93, (char)196, (char)31, (char)73, (char)119, (char)188, (char)214, (char)128, (char)214, (char)228, (char)25, (char)69, (char)133, (char)164, (char)40, (char)62, (char)30, (char)172, (char)254, (char)9, (char)90, (char)209, (char)115, (char)213, (char)27, (char)33, (char)46, (char)181, (char)194, (char)227, (char)195, (char)225, (char)238, (char)193, (char)106, (char)108, (char)142, (char)224, (char)246, (char)233, (char)83, (char)24, (char)5, (char)170, (char)197, (char)98, (char)19, (char)128, (char)180, (char)183, (char)112, (char)214, (char)211, (char)141, (char)231, (char)31, (char)40, (char)70, (char)52, (char)249, (char)209, (char)245, (char)195, (char)117, (char)61, (char)194, (char)140, (char)27, (char)60, (char)156, (char)22, (char)80, (char)174, (char)67, (char)181, (char)126, (char)73, (char)142, (char)55, (char)25, (char)244, (char)222, (char)205, (char)203, (char)193, (char)125, (char)95, (char)189, (char)14, (char)143, (char)253, (char)105, (char)135, (char)235, (char)192, (char)136, (char)8, (char)76, (char)21, (char)104, (char)220, (char)205, (char)225, (char)153, (char)37, (char)111, (char)121, (char)161, (char)27, (char)79, (char)237, (char)197}, 0) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)26587);
            assert(pack.press_diff_GET() == 5.539823E37F);
            assert(pack.press_abs_GET() == 2.3411146E38F);
            assert(pack.time_boot_ms_GET() == 2185798955L);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.temperature_SET((short)26587) ;
        p143.press_abs_SET(2.3411146E38F) ;
        p143.time_boot_ms_SET(2185798955L) ;
        p143.press_diff_SET(5.539823E37F) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.vel_GET(),  new float[] {1.4033118E38F, -1.7612585E38F, 1.5038644E38F}));
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-3.06617E38F, -1.5772683E38F, 3.705571E36F}));
            assert(pack.custom_state_GET() == 5942531111271563952L);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-1.5703527E38F, -3.375774E38F, -7.6066916E37F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-1.9483145E38F, -2.4897706E38F, 1.1487731E38F, 2.7531564E38F}));
            assert(pack.timestamp_GET() == 4460962198102477647L);
            assert(pack.alt_GET() == -1.0894294E38F);
            assert(pack.est_capabilities_GET() == (char)46);
            assert(pack.lat_GET() == -772377435);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-2.4830004E38F, 1.388949E38F, 1.4652464E38F}));
            assert(pack.lon_GET() == 1294153632);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.lat_SET(-772377435) ;
        p144.custom_state_SET(5942531111271563952L) ;
        p144.acc_SET(new float[] {-3.06617E38F, -1.5772683E38F, 3.705571E36F}, 0) ;
        p144.timestamp_SET(4460962198102477647L) ;
        p144.alt_SET(-1.0894294E38F) ;
        p144.position_cov_SET(new float[] {-2.4830004E38F, 1.388949E38F, 1.4652464E38F}, 0) ;
        p144.vel_SET(new float[] {1.4033118E38F, -1.7612585E38F, 1.5038644E38F}, 0) ;
        p144.lon_SET(1294153632) ;
        p144.rates_SET(new float[] {-1.5703527E38F, -3.375774E38F, -7.6066916E37F}, 0) ;
        p144.est_capabilities_SET((char)46) ;
        p144.attitude_q_SET(new float[] {-1.9483145E38F, -2.4897706E38F, 1.1487731E38F, 2.7531564E38F}, 0) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.z_vel_GET() == -2.6413078E38F);
            assert(pack.y_vel_GET() == 1.9981585E38F);
            assert(pack.z_acc_GET() == 1.4862031E38F);
            assert(pack.roll_rate_GET() == -1.524503E38F);
            assert(pack.airspeed_GET() == -7.137843E36F);
            assert(pack.x_vel_GET() == -8.1368307E37F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {2.8514983E38F, 3.0254289E38F, -2.6904016E38F}));
            assert(pack.time_usec_GET() == 3803468503772503735L);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-3.2993118E38F, 2.0233245E38F, -5.369914E37F}));
            assert(pack.x_pos_GET() == -1.6145371E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.6998893E38F, 3.018938E38F, 2.0987785E38F, -2.923333E38F}));
            assert(pack.x_acc_GET() == -2.8691594E38F);
            assert(pack.y_acc_GET() == -6.2446796E37F);
            assert(pack.pitch_rate_GET() == 3.075612E38F);
            assert(pack.y_pos_GET() == 1.7903634E38F);
            assert(pack.z_pos_GET() == -2.4918575E37F);
            assert(pack.yaw_rate_GET() == -1.5730266E38F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.x_vel_SET(-8.1368307E37F) ;
        p146.y_acc_SET(-6.2446796E37F) ;
        p146.vel_variance_SET(new float[] {-3.2993118E38F, 2.0233245E38F, -5.369914E37F}, 0) ;
        p146.airspeed_SET(-7.137843E36F) ;
        p146.y_vel_SET(1.9981585E38F) ;
        p146.z_vel_SET(-2.6413078E38F) ;
        p146.pos_variance_SET(new float[] {2.8514983E38F, 3.0254289E38F, -2.6904016E38F}, 0) ;
        p146.q_SET(new float[] {1.6998893E38F, 3.018938E38F, 2.0987785E38F, -2.923333E38F}, 0) ;
        p146.pitch_rate_SET(3.075612E38F) ;
        p146.x_acc_SET(-2.8691594E38F) ;
        p146.z_acc_SET(1.4862031E38F) ;
        p146.roll_rate_SET(-1.524503E38F) ;
        p146.time_usec_SET(3803468503772503735L) ;
        p146.z_pos_SET(-2.4918575E37F) ;
        p146.yaw_rate_SET(-1.5730266E38F) ;
        p146.y_pos_SET(1.7903634E38F) ;
        p146.x_pos_SET(-1.6145371E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.current_battery_GET() == (short)25533);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
            assert(pack.id_GET() == (char)84);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)27589, (char)16937, (char)7612, (char)755, (char)42475, (char)6404, (char)57616, (char)29311, (char)28193, (char)10660}));
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
            assert(pack.battery_remaining_GET() == (byte)19);
            assert(pack.current_consumed_GET() == -437177529);
            assert(pack.temperature_GET() == (short) -1157);
            assert(pack.energy_consumed_GET() == -1254743195);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN) ;
        p147.temperature_SET((short) -1157) ;
        p147.current_consumed_SET(-437177529) ;
        p147.energy_consumed_SET(-1254743195) ;
        p147.current_battery_SET((short)25533) ;
        p147.battery_remaining_SET((byte)19) ;
        p147.voltages_SET(new char[] {(char)27589, (char)16937, (char)7612, (char)755, (char)42475, (char)6404, (char)57616, (char)29311, (char)28193, (char)10660}, 0) ;
        p147.id_SET((char)84) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.vendor_id_GET() == (char)10858);
            assert(pack.uid_GET() == 1604933336132649051L);
            assert(pack.board_version_GET() == 3855162487L);
            assert(pack.product_id_GET() == (char)35812);
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET));
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)194, (char)56, (char)213, (char)202, (char)30, (char)53, (char)64, (char)69}));
            assert(pack.os_sw_version_GET() == 4045973291L);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)119, (char)235, (char)232, (char)52, (char)191, (char)183, (char)12, (char)252}));
            assert(pack.middleware_sw_version_GET() == 773906833L);
            assert(pack.flight_sw_version_GET() == 11983377L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)245, (char)4, (char)188, (char)6, (char)227, (char)47, (char)245, (char)201, (char)126, (char)221, (char)43, (char)35, (char)132, (char)9, (char)130, (char)197, (char)88, (char)67}));
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)81, (char)204, (char)236, (char)209, (char)191, (char)14, (char)45, (char)114}));
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.os_sw_version_SET(4045973291L) ;
        p148.uid2_SET(new char[] {(char)245, (char)4, (char)188, (char)6, (char)227, (char)47, (char)245, (char)201, (char)126, (char)221, (char)43, (char)35, (char)132, (char)9, (char)130, (char)197, (char)88, (char)67}, 0, PH) ;
        p148.flight_custom_version_SET(new char[] {(char)194, (char)56, (char)213, (char)202, (char)30, (char)53, (char)64, (char)69}, 0) ;
        p148.middleware_sw_version_SET(773906833L) ;
        p148.product_id_SET((char)35812) ;
        p148.uid_SET(1604933336132649051L) ;
        p148.flight_sw_version_SET(11983377L) ;
        p148.middleware_custom_version_SET(new char[] {(char)119, (char)235, (char)232, (char)52, (char)191, (char)183, (char)12, (char)252}, 0) ;
        p148.vendor_id_SET((char)10858) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET)) ;
        p148.os_custom_version_SET(new char[] {(char)81, (char)204, (char)236, (char)209, (char)191, (char)14, (char)45, (char)114}, 0) ;
        p148.board_version_SET(3855162487L) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.position_valid_TRY(ph) == (char)162);
            assert(pack.y_TRY(ph) == 4.587095E36F);
            assert(pack.time_usec_GET() == 7973186336386351041L);
            assert(pack.angle_y_GET() == 3.3421069E38F);
            assert(pack.size_y_GET() == -2.920851E38F);
            assert(pack.x_TRY(ph) == -2.898018E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
            assert(pack.distance_GET() == 1.1115735E38F);
            assert(pack.z_TRY(ph) == 5.7498157E37F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {2.2536187E38F, 1.8648195E38F, 1.630434E38F, -4.5660227E37F}));
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.angle_x_GET() == 3.1495592E38F);
            assert(pack.target_num_GET() == (char)66);
            assert(pack.size_x_GET() == 2.5251017E37F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p149.position_valid_SET((char)162, PH) ;
        p149.target_num_SET((char)66) ;
        p149.q_SET(new float[] {2.2536187E38F, 1.8648195E38F, 1.630434E38F, -4.5660227E37F}, 0, PH) ;
        p149.y_SET(4.587095E36F, PH) ;
        p149.angle_y_SET(3.3421069E38F) ;
        p149.distance_SET(1.1115735E38F) ;
        p149.size_y_SET(-2.920851E38F) ;
        p149.size_x_SET(2.5251017E37F) ;
        p149.x_SET(-2.898018E38F, PH) ;
        p149.z_SET(5.7498157E37F, PH) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER) ;
        p149.time_usec_SET(7973186336386351041L) ;
        p149.angle_x_SET(3.1495592E38F) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_0.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {1772038843L, 735182289L, 1956531256L, 1726426993L}));
            assert(Arrays.equals(pack.ar_u8_GET(),  new char[] {(char)144, (char)78, (char)130, (char)196}));
            assert(Arrays.equals(pack.ar_u16_GET(),  new char[] {(char)61438, (char)18122, (char)13542, (char)59229}));
            assert(pack.v1_GET() == (char)36);
            assert(Arrays.equals(pack.ar_i8_GET(),  new byte[] {(byte)109, (byte)36, (byte)5, (byte)73}));
        });
        GroundControl.ARRAY_TEST_0 p150 = CommunicationChannel.new_ARRAY_TEST_0();
        PH.setPack(p150);
        p150.ar_i8_SET(new byte[] {(byte)109, (byte)36, (byte)5, (byte)73}, 0) ;
        p150.ar_u8_SET(new char[] {(char)144, (char)78, (char)130, (char)196}, 0) ;
        p150.ar_u32_SET(new long[] {1772038843L, 735182289L, 1956531256L, 1726426993L}, 0) ;
        p150.ar_u16_SET(new char[] {(char)61438, (char)18122, (char)13542, (char)59229}, 0) ;
        p150.v1_SET((char)36) ;
        CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_1.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {222721674L, 3283838973L, 2220603110L, 823359298L}));
        });
        GroundControl.ARRAY_TEST_1 p151 = CommunicationChannel.new_ARRAY_TEST_1();
        PH.setPack(p151);
        p151.ar_u32_SET(new long[] {222721674L, 3283838973L, 2220603110L, 823359298L}, 0) ;
        CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_3.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {408354475L, 3868694128L, 390869573L, 1354081735L}));
            assert(pack.v_GET() == (char)69);
        });
        GroundControl.ARRAY_TEST_3 p153 = CommunicationChannel.new_ARRAY_TEST_3();
        PH.setPack(p153);
        p153.ar_u32_SET(new long[] {408354475L, 3868694128L, 390869573L, 1354081735L}, 0) ;
        p153.v_SET((char)69) ;
        CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_4.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {1199008774L, 1998523171L, 252621123L, 2641376161L}));
            assert(pack.v_GET() == (char)33);
        });
        GroundControl.ARRAY_TEST_4 p154 = CommunicationChannel.new_ARRAY_TEST_4();
        PH.setPack(p154);
        p154.ar_u32_SET(new long[] {1199008774L, 1998523171L, 252621123L, 2641376161L}, 0) ;
        p154.v_SET((char)33) ;
        CommunicationChannel.instance.send(p154);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_5.add((src, ph, pack) ->
        {
            assert(pack.c1_LEN(ph) == 4);
            assert(pack.c1_TRY(ph).equals("kkym"));
            assert(pack.c2_LEN(ph) == 1);
            assert(pack.c2_TRY(ph).equals("h"));
        });
        GroundControl.ARRAY_TEST_5 p155 = CommunicationChannel.new_ARRAY_TEST_5();
        PH.setPack(p155);
        p155.c1_SET("kkym", PH) ;
        p155.c2_SET("h", PH) ;
        CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_6.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_i32_GET(),  new int[] {-145354226, 335838673}));
            assert(Arrays.equals(pack.ar_i8_GET(),  new byte[] {(byte)13, (byte)56}));
            assert(pack.v2_GET() == (char)19066);
            assert(Arrays.equals(pack.ar_u8_GET(),  new char[] {(char)96, (char)16}));
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {959708072L, 1876674256L}));
            assert(pack.v3_GET() == 578087504L);
            assert(Arrays.equals(pack.ar_i16_GET(),  new short[] {(short) -12828, (short) -16810}));
            assert(Arrays.equals(pack.ar_u16_GET(),  new char[] {(char)30338, (char)43262}));
            assert(Arrays.equals(pack.ar_d_GET(),  new double[] {-1.3661242119053974E308, -1.6590239752784334E308}));
            assert(Arrays.equals(pack.ar_f_GET(),  new float[] {3.571957E36F, 2.1633072E38F}));
            assert(pack.ar_c_LEN(ph) == 3);
            assert(pack.ar_c_TRY(ph).equals("wij"));
            assert(pack.v1_GET() == (char)239);
        });
        GroundControl.ARRAY_TEST_6 p156 = CommunicationChannel.new_ARRAY_TEST_6();
        PH.setPack(p156);
        p156.ar_d_SET(new double[] {-1.3661242119053974E308, -1.6590239752784334E308}, 0) ;
        p156.ar_i8_SET(new byte[] {(byte)13, (byte)56}, 0) ;
        p156.ar_i32_SET(new int[] {-145354226, 335838673}, 0) ;
        p156.v3_SET(578087504L) ;
        p156.v2_SET((char)19066) ;
        p156.ar_u16_SET(new char[] {(char)30338, (char)43262}, 0) ;
        p156.ar_u8_SET(new char[] {(char)96, (char)16}, 0) ;
        p156.ar_i16_SET(new short[] {(short) -12828, (short) -16810}, 0) ;
        p156.ar_u32_SET(new long[] {959708072L, 1876674256L}, 0) ;
        p156.ar_f_SET(new float[] {3.571957E36F, 2.1633072E38F}, 0) ;
        p156.ar_c_SET("wij", PH) ;
        p156.v1_SET((char)239) ;
        CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_7.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {2652999111L, 4211548313L}));
            assert(Arrays.equals(pack.ar_u16_GET(),  new char[] {(char)51856, (char)39366}));
            assert(Arrays.equals(pack.ar_i16_GET(),  new short[] {(short)12573, (short) -24525}));
            assert(Arrays.equals(pack.ar_i8_GET(),  new byte[] {(byte)38, (byte) - 107}));
            assert(Arrays.equals(pack.ar_u8_GET(),  new char[] {(char)3, (char)65}));
            assert(Arrays.equals(pack.ar_d_GET(),  new double[] {-1.3714483454143143E308, 1.476870643026448E308}));
            assert(Arrays.equals(pack.ar_f_GET(),  new float[] {-1.5844649E38F, -3.2216427E37F}));
            assert(Arrays.equals(pack.ar_i32_GET(),  new int[] {515471910, 1966983271}));
            assert(pack.ar_c_LEN(ph) == 15);
            assert(pack.ar_c_TRY(ph).equals("avbuexqeuyqebqm"));
        });
        GroundControl.ARRAY_TEST_7 p157 = CommunicationChannel.new_ARRAY_TEST_7();
        PH.setPack(p157);
        p157.ar_u32_SET(new long[] {2652999111L, 4211548313L}, 0) ;
        p157.ar_d_SET(new double[] {-1.3714483454143143E308, 1.476870643026448E308}, 0) ;
        p157.ar_u8_SET(new char[] {(char)3, (char)65}, 0) ;
        p157.ar_u16_SET(new char[] {(char)51856, (char)39366}, 0) ;
        p157.ar_i32_SET(new int[] {515471910, 1966983271}, 0) ;
        p157.ar_i8_SET(new byte[] {(byte)38, (byte) - 107}, 0) ;
        p157.ar_i16_SET(new short[] {(short)12573, (short) -24525}, 0) ;
        p157.ar_c_SET("avbuexqeuyqebqm", PH) ;
        p157.ar_f_SET(new float[] {-1.5844649E38F, -3.2216427E37F}, 0) ;
        CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ARRAY_TEST_8.add((src, ph, pack) ->
        {
            assert(pack.v3_GET() == 496888155L);
            assert(Arrays.equals(pack.ar_d_GET(),  new double[] {-1.4864969203403546E308, 6.017901373794539E307}));
            assert(Arrays.equals(pack.ar_u16_GET(),  new char[] {(char)952, (char)52680}));
        });
        GroundControl.ARRAY_TEST_8 p158 = CommunicationChannel.new_ARRAY_TEST_8();
        PH.setPack(p158);
        p158.v3_SET(496888155L) ;
        p158.ar_u16_SET(new char[] {(char)952, (char)52680}, 0) ;
        p158.ar_d_SET(new double[] {-1.4864969203403546E308, 6.017901373794539E307}, 0) ;
        CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pos_horiz_accuracy_GET() == 2.3553794E38F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE));
            assert(pack.pos_vert_accuracy_GET() == -1.8762192E38F);
            assert(pack.mag_ratio_GET() == -1.4232509E38F);
            assert(pack.pos_vert_ratio_GET() == -1.6780649E38F);
            assert(pack.vel_ratio_GET() == -1.3525253E38F);
            assert(pack.tas_ratio_GET() == -1.113889E38F);
            assert(pack.pos_horiz_ratio_GET() == 2.3095463E37F);
            assert(pack.hagl_ratio_GET() == -4.6873227E37F);
            assert(pack.time_usec_GET() == 9175955503855826331L);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.mag_ratio_SET(-1.4232509E38F) ;
        p230.pos_horiz_accuracy_SET(2.3553794E38F) ;
        p230.pos_vert_accuracy_SET(-1.8762192E38F) ;
        p230.time_usec_SET(9175955503855826331L) ;
        p230.vel_ratio_SET(-1.3525253E38F) ;
        p230.pos_vert_ratio_SET(-1.6780649E38F) ;
        p230.hagl_ratio_SET(-4.6873227E37F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE)) ;
        p230.tas_ratio_SET(-1.113889E38F) ;
        p230.pos_horiz_ratio_SET(2.3095463E37F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.var_horiz_GET() == 7.448266E37F);
            assert(pack.wind_y_GET() == 2.2927418E38F);
            assert(pack.vert_accuracy_GET() == -4.664672E37F);
            assert(pack.time_usec_GET() == 4447988195317268736L);
            assert(pack.wind_x_GET() == -3.0528434E38F);
            assert(pack.wind_alt_GET() == -3.2494678E38F);
            assert(pack.horiz_accuracy_GET() == 3.2285199E38F);
            assert(pack.var_vert_GET() == 2.5321822E38F);
            assert(pack.wind_z_GET() == -2.8349046E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.var_horiz_SET(7.448266E37F) ;
        p231.wind_z_SET(-2.8349046E38F) ;
        p231.vert_accuracy_SET(-4.664672E37F) ;
        p231.wind_y_SET(2.2927418E38F) ;
        p231.time_usec_SET(4447988195317268736L) ;
        p231.var_vert_SET(2.5321822E38F) ;
        p231.horiz_accuracy_SET(3.2285199E38F) ;
        p231.wind_alt_SET(-3.2494678E38F) ;
        p231.wind_x_SET(-3.0528434E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -136701677);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY));
            assert(pack.horiz_accuracy_GET() == 1.2622431E38F);
            assert(pack.ve_GET() == -2.961466E38F);
            assert(pack.alt_GET() == -1.9057068E38F);
            assert(pack.vert_accuracy_GET() == 3.0906255E38F);
            assert(pack.vn_GET() == 3.3147872E37F);
            assert(pack.time_week_ms_GET() == 2666742508L);
            assert(pack.hdop_GET() == 1.6429707E36F);
            assert(pack.gps_id_GET() == (char)37);
            assert(pack.vd_GET() == 3.3391195E38F);
            assert(pack.fix_type_GET() == (char)225);
            assert(pack.satellites_visible_GET() == (char)164);
            assert(pack.vdop_GET() == 3.2880334E36F);
            assert(pack.lat_GET() == 1806661968);
            assert(pack.time_usec_GET() == 4533440536520719095L);
            assert(pack.time_week_GET() == (char)18248);
            assert(pack.speed_accuracy_GET() == -3.1457062E38F);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.gps_id_SET((char)37) ;
        p232.ve_SET(-2.961466E38F) ;
        p232.vdop_SET(3.2880334E36F) ;
        p232.time_week_ms_SET(2666742508L) ;
        p232.hdop_SET(1.6429707E36F) ;
        p232.satellites_visible_SET((char)164) ;
        p232.vn_SET(3.3147872E37F) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY)) ;
        p232.horiz_accuracy_SET(1.2622431E38F) ;
        p232.vert_accuracy_SET(3.0906255E38F) ;
        p232.speed_accuracy_SET(-3.1457062E38F) ;
        p232.time_week_SET((char)18248) ;
        p232.alt_SET(-1.9057068E38F) ;
        p232.fix_type_SET((char)225) ;
        p232.lat_SET(1806661968) ;
        p232.time_usec_SET(4533440536520719095L) ;
        p232.vd_SET(3.3391195E38F) ;
        p232.lon_SET(-136701677) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)246, (char)85, (char)116, (char)248, (char)110, (char)24, (char)176, (char)137, (char)2, (char)255, (char)203, (char)118, (char)131, (char)210, (char)34, (char)87, (char)77, (char)254, (char)178, (char)196, (char)85, (char)57, (char)186, (char)124, (char)213, (char)251, (char)169, (char)22, (char)85, (char)95, (char)111, (char)96, (char)186, (char)246, (char)10, (char)237, (char)38, (char)158, (char)170, (char)3, (char)178, (char)177, (char)32, (char)210, (char)19, (char)6, (char)78, (char)224, (char)213, (char)75, (char)168, (char)206, (char)245, (char)212, (char)203, (char)65, (char)97, (char)228, (char)86, (char)139, (char)235, (char)117, (char)47, (char)116, (char)228, (char)176, (char)17, (char)227, (char)119, (char)15, (char)8, (char)158, (char)155, (char)101, (char)35, (char)215, (char)13, (char)8, (char)195, (char)147, (char)226, (char)74, (char)37, (char)170, (char)230, (char)183, (char)172, (char)209, (char)2, (char)90, (char)165, (char)245, (char)159, (char)52, (char)159, (char)193, (char)104, (char)19, (char)226, (char)234, (char)117, (char)85, (char)70, (char)82, (char)208, (char)85, (char)133, (char)250, (char)182, (char)94, (char)85, (char)137, (char)191, (char)255, (char)110, (char)103, (char)53, (char)137, (char)83, (char)230, (char)254, (char)157, (char)32, (char)71, (char)169, (char)100, (char)252, (char)224, (char)38, (char)6, (char)167, (char)185, (char)177, (char)123, (char)85, (char)202, (char)53, (char)112, (char)244, (char)194, (char)48, (char)97, (char)61, (char)174, (char)98, (char)38, (char)157, (char)49, (char)192, (char)92, (char)243, (char)58, (char)105, (char)40, (char)8, (char)133, (char)156, (char)177, (char)227, (char)246, (char)96, (char)82, (char)55, (char)53, (char)158, (char)123, (char)42, (char)41, (char)121, (char)65, (char)5, (char)67, (char)133, (char)34, (char)83, (char)8, (char)164, (char)237, (char)7, (char)30}));
            assert(pack.len_GET() == (char)194);
            assert(pack.flags_GET() == (char)230);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.len_SET((char)194) ;
        p233.flags_SET((char)230) ;
        p233.data__SET(new char[] {(char)246, (char)85, (char)116, (char)248, (char)110, (char)24, (char)176, (char)137, (char)2, (char)255, (char)203, (char)118, (char)131, (char)210, (char)34, (char)87, (char)77, (char)254, (char)178, (char)196, (char)85, (char)57, (char)186, (char)124, (char)213, (char)251, (char)169, (char)22, (char)85, (char)95, (char)111, (char)96, (char)186, (char)246, (char)10, (char)237, (char)38, (char)158, (char)170, (char)3, (char)178, (char)177, (char)32, (char)210, (char)19, (char)6, (char)78, (char)224, (char)213, (char)75, (char)168, (char)206, (char)245, (char)212, (char)203, (char)65, (char)97, (char)228, (char)86, (char)139, (char)235, (char)117, (char)47, (char)116, (char)228, (char)176, (char)17, (char)227, (char)119, (char)15, (char)8, (char)158, (char)155, (char)101, (char)35, (char)215, (char)13, (char)8, (char)195, (char)147, (char)226, (char)74, (char)37, (char)170, (char)230, (char)183, (char)172, (char)209, (char)2, (char)90, (char)165, (char)245, (char)159, (char)52, (char)159, (char)193, (char)104, (char)19, (char)226, (char)234, (char)117, (char)85, (char)70, (char)82, (char)208, (char)85, (char)133, (char)250, (char)182, (char)94, (char)85, (char)137, (char)191, (char)255, (char)110, (char)103, (char)53, (char)137, (char)83, (char)230, (char)254, (char)157, (char)32, (char)71, (char)169, (char)100, (char)252, (char)224, (char)38, (char)6, (char)167, (char)185, (char)177, (char)123, (char)85, (char)202, (char)53, (char)112, (char)244, (char)194, (char)48, (char)97, (char)61, (char)174, (char)98, (char)38, (char)157, (char)49, (char)192, (char)92, (char)243, (char)58, (char)105, (char)40, (char)8, (char)133, (char)156, (char)177, (char)227, (char)246, (char)96, (char)82, (char)55, (char)53, (char)158, (char)123, (char)42, (char)41, (char)121, (char)65, (char)5, (char)67, (char)133, (char)34, (char)83, (char)8, (char)164, (char)237, (char)7, (char)30}, 0) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.failsafe_GET() == (char)244);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            assert(pack.temperature_air_GET() == (byte)93);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED));
            assert(pack.heading_GET() == (char)10356);
            assert(pack.groundspeed_GET() == (char)236);
            assert(pack.throttle_GET() == (byte) - 85);
            assert(pack.heading_sp_GET() == (short)20005);
            assert(pack.pitch_GET() == (short) -14538);
            assert(pack.climb_rate_GET() == (byte) - 27);
            assert(pack.wp_num_GET() == (char)76);
            assert(pack.wp_distance_GET() == (char)31508);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
            assert(pack.gps_nsat_GET() == (char)109);
            assert(pack.altitude_amsl_GET() == (short) -26320);
            assert(pack.latitude_GET() == 1367780264);
            assert(pack.temperature_GET() == (byte) - 127);
            assert(pack.airspeed_GET() == (char)155);
            assert(pack.altitude_sp_GET() == (short)2041);
            assert(pack.roll_GET() == (short)30501);
            assert(pack.longitude_GET() == 1191660463);
            assert(pack.airspeed_sp_GET() == (char)9);
            assert(pack.battery_remaining_GET() == (char)152);
            assert(pack.custom_mode_GET() == 3266306020L);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED)) ;
        p234.airspeed_sp_SET((char)9) ;
        p234.throttle_SET((byte) - 85) ;
        p234.heading_sp_SET((short)20005) ;
        p234.groundspeed_SET((char)236) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF) ;
        p234.temperature_SET((byte) - 127) ;
        p234.wp_distance_SET((char)31508) ;
        p234.longitude_SET(1191660463) ;
        p234.heading_SET((char)10356) ;
        p234.custom_mode_SET(3266306020L) ;
        p234.climb_rate_SET((byte) - 27) ;
        p234.roll_SET((short)30501) ;
        p234.altitude_amsl_SET((short) -26320) ;
        p234.altitude_sp_SET((short)2041) ;
        p234.pitch_SET((short) -14538) ;
        p234.wp_num_SET((char)76) ;
        p234.battery_remaining_SET((char)152) ;
        p234.latitude_SET(1367780264) ;
        p234.airspeed_SET((char)155) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP) ;
        p234.failsafe_SET((char)244) ;
        p234.gps_nsat_SET((char)109) ;
        p234.temperature_air_SET((byte)93) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_x_GET() == -2.5585655E38F);
            assert(pack.vibration_y_GET() == -1.9840468E38F);
            assert(pack.time_usec_GET() == 2666872841687280715L);
            assert(pack.clipping_1_GET() == 3261084935L);
            assert(pack.clipping_2_GET() == 4011825289L);
            assert(pack.vibration_z_GET() == -3.262496E38F);
            assert(pack.clipping_0_GET() == 14503007L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.vibration_y_SET(-1.9840468E38F) ;
        p241.clipping_2_SET(4011825289L) ;
        p241.vibration_z_SET(-3.262496E38F) ;
        p241.clipping_1_SET(3261084935L) ;
        p241.time_usec_SET(2666872841687280715L) ;
        p241.vibration_x_SET(-2.5585655E38F) ;
        p241.clipping_0_SET(14503007L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_x_GET() == -2.923205E38F);
            assert(pack.z_GET() == -5.4772855E37F);
            assert(pack.altitude_GET() == 646120485);
            assert(pack.time_usec_TRY(ph) == 2093491404303972101L);
            assert(pack.longitude_GET() == -1546034903);
            assert(pack.y_GET() == -2.103416E38F);
            assert(pack.x_GET() == 5.7617884E37F);
            assert(pack.approach_y_GET() == 1.4012476E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.6669361E38F, 1.2033E38F, 3.2799662E38F, -1.8819423E38F}));
            assert(pack.latitude_GET() == -1075763058);
            assert(pack.approach_z_GET() == -2.6805271E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.longitude_SET(-1546034903) ;
        p242.altitude_SET(646120485) ;
        p242.q_SET(new float[] {-1.6669361E38F, 1.2033E38F, 3.2799662E38F, -1.8819423E38F}, 0) ;
        p242.approach_z_SET(-2.6805271E38F) ;
        p242.y_SET(-2.103416E38F) ;
        p242.latitude_SET(-1075763058) ;
        p242.time_usec_SET(2093491404303972101L, PH) ;
        p242.x_SET(5.7617884E37F) ;
        p242.approach_x_SET(-2.923205E38F) ;
        p242.approach_y_SET(1.4012476E38F) ;
        p242.z_SET(-5.4772855E37F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 3511787620472292685L);
            assert(pack.y_GET() == 3.1228296E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.2946466E38F, 7.732146E37F, -2.279161E38F, -3.6785836E37F}));
            assert(pack.approach_z_GET() == -1.79599E38F);
            assert(pack.approach_y_GET() == 2.0869825E38F);
            assert(pack.longitude_GET() == -2062064319);
            assert(pack.latitude_GET() == 1433186072);
            assert(pack.z_GET() == -2.052789E38F);
            assert(pack.altitude_GET() == -1008578887);
            assert(pack.x_GET() == 1.7623986E38F);
            assert(pack.approach_x_GET() == -8.294639E37F);
            assert(pack.target_system_GET() == (char)107);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.target_system_SET((char)107) ;
        p243.approach_x_SET(-8.294639E37F) ;
        p243.latitude_SET(1433186072) ;
        p243.x_SET(1.7623986E38F) ;
        p243.time_usec_SET(3511787620472292685L, PH) ;
        p243.longitude_SET(-2062064319) ;
        p243.approach_y_SET(2.0869825E38F) ;
        p243.approach_z_SET(-1.79599E38F) ;
        p243.q_SET(new float[] {-3.2946466E38F, 7.732146E37F, -2.279161E38F, -3.6785836E37F}, 0) ;
        p243.y_SET(3.1228296E37F) ;
        p243.z_SET(-2.052789E38F) ;
        p243.altitude_SET(-1008578887) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == 1547564609);
            assert(pack.message_id_GET() == (char)54188);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(1547564609) ;
        p244.message_id_SET((char)54188) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_GLIDER);
            assert(pack.ver_velocity_GET() == (short) -22525);
            assert(pack.heading_GET() == (char)44307);
            assert(pack.hor_velocity_GET() == (char)7819);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.lon_GET() == 600849452);
            assert(pack.tslc_GET() == (char)43);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING));
            assert(pack.squawk_GET() == (char)60304);
            assert(pack.callsign_LEN(ph) == 8);
            assert(pack.callsign_TRY(ph).equals("ywnjxjiz"));
            assert(pack.ICAO_address_GET() == 954944775L);
            assert(pack.altitude_GET() == -1106680273);
            assert(pack.lat_GET() == 4432815);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.tslc_SET((char)43) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_GLIDER) ;
        p246.squawk_SET((char)60304) ;
        p246.lon_SET(600849452) ;
        p246.hor_velocity_SET((char)7819) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.ICAO_address_SET(954944775L) ;
        p246.callsign_SET("ywnjxjiz", PH) ;
        p246.lat_SET(4432815) ;
        p246.altitude_SET(-1106680273) ;
        p246.heading_SET((char)44307) ;
        p246.ver_velocity_SET((short) -22525) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING)) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.time_to_minimum_delta_GET() == -2.9679E38F);
            assert(pack.id_GET() == 3070635561L);
            assert(pack.threat_level_GET() == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH));
            assert(pack.altitude_minimum_delta_GET() == 1.0793597E38F);
            assert(pack.horizontal_minimum_delta_GET() == 2.3281841E38F);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.id_SET(3070635561L) ;
        p247.threat_level_SET((MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH)) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.altitude_minimum_delta_SET(1.0793597E38F) ;
        p247.time_to_minimum_delta_SET(-2.9679E38F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE) ;
        p247.horizontal_minimum_delta_SET(2.3281841E38F) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)153);
            assert(pack.target_system_GET() == (char)224);
            assert(pack.message_type_GET() == (char)29641);
            assert(pack.target_component_GET() == (char)91);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)230, (char)7, (char)83, (char)243, (char)141, (char)154, (char)15, (char)135, (char)86, (char)187, (char)145, (char)145, (char)251, (char)166, (char)12, (char)125, (char)105, (char)254, (char)151, (char)236, (char)188, (char)90, (char)105, (char)190, (char)16, (char)231, (char)231, (char)226, (char)59, (char)156, (char)80, (char)165, (char)9, (char)160, (char)35, (char)81, (char)202, (char)108, (char)2, (char)200, (char)181, (char)146, (char)66, (char)86, (char)103, (char)57, (char)9, (char)42, (char)111, (char)69, (char)42, (char)152, (char)103, (char)127, (char)248, (char)211, (char)172, (char)189, (char)99, (char)82, (char)98, (char)90, (char)212, (char)254, (char)213, (char)157, (char)68, (char)190, (char)193, (char)174, (char)105, (char)36, (char)113, (char)85, (char)72, (char)177, (char)233, (char)145, (char)109, (char)118, (char)252, (char)67, (char)230, (char)229, (char)42, (char)147, (char)122, (char)129, (char)226, (char)164, (char)136, (char)107, (char)234, (char)32, (char)53, (char)117, (char)120, (char)153, (char)252, (char)187, (char)213, (char)129, (char)133, (char)81, (char)72, (char)230, (char)69, (char)85, (char)86, (char)13, (char)171, (char)175, (char)233, (char)40, (char)203, (char)64, (char)114, (char)198, (char)98, (char)59, (char)35, (char)156, (char)95, (char)116, (char)15, (char)95, (char)101, (char)248, (char)180, (char)56, (char)217, (char)128, (char)154, (char)207, (char)159, (char)177, (char)252, (char)52, (char)34, (char)187, (char)222, (char)49, (char)247, (char)192, (char)211, (char)238, (char)8, (char)238, (char)170, (char)72, (char)243, (char)90, (char)19, (char)78, (char)190, (char)32, (char)4, (char)89, (char)211, (char)76, (char)200, (char)43, (char)111, (char)49, (char)164, (char)210, (char)33, (char)96, (char)90, (char)58, (char)175, (char)47, (char)49, (char)122, (char)70, (char)206, (char)140, (char)46, (char)254, (char)250, (char)38, (char)174, (char)100, (char)162, (char)146, (char)106, (char)191, (char)176, (char)221, (char)123, (char)215, (char)251, (char)16, (char)244, (char)182, (char)196, (char)58, (char)2, (char)234, (char)233, (char)152, (char)231, (char)197, (char)183, (char)193, (char)87, (char)204, (char)88, (char)222, (char)23, (char)28, (char)154, (char)105, (char)57, (char)246, (char)69, (char)144, (char)136, (char)26, (char)35, (char)143, (char)253, (char)121, (char)231, (char)121, (char)71, (char)173, (char)61, (char)250, (char)220, (char)26, (char)37, (char)132, (char)106, (char)184, (char)140, (char)198, (char)211, (char)98, (char)243, (char)109, (char)162, (char)238, (char)9, (char)17, (char)152, (char)59, (char)169, (char)210}));
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.message_type_SET((char)29641) ;
        p248.target_system_SET((char)224) ;
        p248.target_network_SET((char)153) ;
        p248.payload_SET(new char[] {(char)230, (char)7, (char)83, (char)243, (char)141, (char)154, (char)15, (char)135, (char)86, (char)187, (char)145, (char)145, (char)251, (char)166, (char)12, (char)125, (char)105, (char)254, (char)151, (char)236, (char)188, (char)90, (char)105, (char)190, (char)16, (char)231, (char)231, (char)226, (char)59, (char)156, (char)80, (char)165, (char)9, (char)160, (char)35, (char)81, (char)202, (char)108, (char)2, (char)200, (char)181, (char)146, (char)66, (char)86, (char)103, (char)57, (char)9, (char)42, (char)111, (char)69, (char)42, (char)152, (char)103, (char)127, (char)248, (char)211, (char)172, (char)189, (char)99, (char)82, (char)98, (char)90, (char)212, (char)254, (char)213, (char)157, (char)68, (char)190, (char)193, (char)174, (char)105, (char)36, (char)113, (char)85, (char)72, (char)177, (char)233, (char)145, (char)109, (char)118, (char)252, (char)67, (char)230, (char)229, (char)42, (char)147, (char)122, (char)129, (char)226, (char)164, (char)136, (char)107, (char)234, (char)32, (char)53, (char)117, (char)120, (char)153, (char)252, (char)187, (char)213, (char)129, (char)133, (char)81, (char)72, (char)230, (char)69, (char)85, (char)86, (char)13, (char)171, (char)175, (char)233, (char)40, (char)203, (char)64, (char)114, (char)198, (char)98, (char)59, (char)35, (char)156, (char)95, (char)116, (char)15, (char)95, (char)101, (char)248, (char)180, (char)56, (char)217, (char)128, (char)154, (char)207, (char)159, (char)177, (char)252, (char)52, (char)34, (char)187, (char)222, (char)49, (char)247, (char)192, (char)211, (char)238, (char)8, (char)238, (char)170, (char)72, (char)243, (char)90, (char)19, (char)78, (char)190, (char)32, (char)4, (char)89, (char)211, (char)76, (char)200, (char)43, (char)111, (char)49, (char)164, (char)210, (char)33, (char)96, (char)90, (char)58, (char)175, (char)47, (char)49, (char)122, (char)70, (char)206, (char)140, (char)46, (char)254, (char)250, (char)38, (char)174, (char)100, (char)162, (char)146, (char)106, (char)191, (char)176, (char)221, (char)123, (char)215, (char)251, (char)16, (char)244, (char)182, (char)196, (char)58, (char)2, (char)234, (char)233, (char)152, (char)231, (char)197, (char)183, (char)193, (char)87, (char)204, (char)88, (char)222, (char)23, (char)28, (char)154, (char)105, (char)57, (char)246, (char)69, (char)144, (char)136, (char)26, (char)35, (char)143, (char)253, (char)121, (char)231, (char)121, (char)71, (char)173, (char)61, (char)250, (char)220, (char)26, (char)37, (char)132, (char)106, (char)184, (char)140, (char)198, (char)211, (char)98, (char)243, (char)109, (char)162, (char)238, (char)9, (char)17, (char)152, (char)59, (char)169, (char)210}, 0) ;
        p248.target_component_SET((char)91) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)104, (byte) - 74, (byte) - 123, (byte) - 12, (byte)37, (byte)104, (byte)66, (byte) - 22, (byte) - 119, (byte) - 47, (byte)111, (byte)68, (byte)27, (byte)104, (byte) - 100, (byte)5, (byte)32, (byte) - 97, (byte) - 82, (byte)65, (byte)96, (byte) - 12, (byte) - 85, (byte)10, (byte)26, (byte) - 35, (byte)85, (byte) - 70, (byte) - 43, (byte) - 33, (byte) - 121, (byte) - 91}));
            assert(pack.ver_GET() == (char)48);
            assert(pack.type_GET() == (char)146);
            assert(pack.address_GET() == (char)7618);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.type_SET((char)146) ;
        p249.ver_SET((char)48) ;
        p249.value_SET(new byte[] {(byte)104, (byte) - 74, (byte) - 123, (byte) - 12, (byte)37, (byte)104, (byte)66, (byte) - 22, (byte) - 119, (byte) - 47, (byte)111, (byte)68, (byte)27, (byte)104, (byte) - 100, (byte)5, (byte)32, (byte) - 97, (byte) - 82, (byte)65, (byte)96, (byte) - 12, (byte) - 85, (byte)10, (byte)26, (byte) - 35, (byte)85, (byte) - 70, (byte) - 43, (byte) - 33, (byte) - 121, (byte) - 91}, 0) ;
        p249.address_SET((char)7618) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -1.4954665E38F);
            assert(pack.name_LEN(ph) == 4);
            assert(pack.name_TRY(ph).equals("dpuk"));
            assert(pack.y_GET() == 1.9104821E38F);
            assert(pack.z_GET() == 1.039428E38F);
            assert(pack.time_usec_GET() == 4090970705331075039L);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.time_usec_SET(4090970705331075039L) ;
        p250.z_SET(1.039428E38F) ;
        p250.name_SET("dpuk", PH) ;
        p250.x_SET(-1.4954665E38F) ;
        p250.y_SET(1.9104821E38F) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 1.2110614E38F);
            assert(pack.name_LEN(ph) == 4);
            assert(pack.name_TRY(ph).equals("xfSs"));
            assert(pack.time_boot_ms_GET() == 1444620015L);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(1.2110614E38F) ;
        p251.time_boot_ms_SET(1444620015L) ;
        p251.name_SET("xfSs", PH) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -1715029772);
            assert(pack.time_boot_ms_GET() == 95346719L);
            assert(pack.name_LEN(ph) == 1);
            assert(pack.name_TRY(ph).equals("b"));
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(95346719L) ;
        p252.value_SET(-1715029772) ;
        p252.name_SET("b", PH) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_NOTICE);
            assert(pack.text_LEN(ph) == 24);
            assert(pack.text_TRY(ph).equals("peiyfgoumipxrxaoHwzezFea"));
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("peiyfgoumipxrxaoHwzezFea", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_NOTICE) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.ind_GET() == (char)170);
            assert(pack.value_GET() == -7.2044447E37F);
            assert(pack.time_boot_ms_GET() == 3074245010L);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.ind_SET((char)170) ;
        p254.time_boot_ms_SET(3074245010L) ;
        p254.value_SET(-7.2044447E37F) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.initial_timestamp_GET() == 8336463519466885522L);
            assert(pack.target_component_GET() == (char)50);
            assert(pack.target_system_GET() == (char)19);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)77, (char)129, (char)85, (char)22, (char)133, (char)157, (char)3, (char)1, (char)252, (char)69, (char)246, (char)71, (char)206, (char)42, (char)85, (char)160, (char)103, (char)211, (char)241, (char)117, (char)145, (char)44, (char)60, (char)35, (char)42, (char)119, (char)21, (char)129, (char)225, (char)74, (char)79, (char)144}));
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)19) ;
        p256.initial_timestamp_SET(8336463519466885522L) ;
        p256.secret_key_SET(new char[] {(char)77, (char)129, (char)85, (char)22, (char)133, (char)157, (char)3, (char)1, (char)252, (char)69, (char)246, (char)71, (char)206, (char)42, (char)85, (char)160, (char)103, (char)211, (char)241, (char)117, (char)145, (char)44, (char)60, (char)35, (char)42, (char)119, (char)21, (char)129, (char)225, (char)74, (char)79, (char)144}, 0) ;
        p256.target_component_SET((char)50) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.last_change_ms_GET() == 2896563364L);
            assert(pack.time_boot_ms_GET() == 2129624391L);
            assert(pack.state_GET() == (char)210);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.state_SET((char)210) ;
        p257.last_change_ms_SET(2896563364L) ;
        p257.time_boot_ms_SET(2129624391L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)23);
            assert(pack.target_system_GET() == (char)0);
            assert(pack.tune_LEN(ph) == 29);
            assert(pack.tune_TRY(ph).equals("mjfMfpcxpxoLsxxprxcyWkfehCoxe"));
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.tune_SET("mjfMfpcxpxoLsxxprxcyWkfehCoxe", PH) ;
        p258.target_system_SET((char)0) ;
        p258.target_component_SET((char)23) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.cam_definition_version_GET() == (char)47105);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)123, (char)188, (char)111, (char)14, (char)29, (char)59, (char)25, (char)249, (char)83, (char)121, (char)62, (char)99, (char)252, (char)119, (char)195, (char)38, (char)81, (char)120, (char)119, (char)205, (char)150, (char)53, (char)195, (char)244, (char)58, (char)40, (char)75, (char)24, (char)35, (char)254, (char)103, (char)164}));
            assert(pack.firmware_version_GET() == 482392188L);
            assert(pack.sensor_size_v_GET() == 6.0301064E37F);
            assert(pack.focal_length_GET() == 2.044233E38F);
            assert(pack.lens_id_GET() == (char)49);
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE));
            assert(pack.resolution_v_GET() == (char)23857);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)218, (char)58, (char)220, (char)204, (char)3, (char)71, (char)38, (char)7, (char)154, (char)63, (char)254, (char)62, (char)169, (char)186, (char)245, (char)88, (char)225, (char)51, (char)191, (char)139, (char)158, (char)218, (char)125, (char)44, (char)132, (char)229, (char)245, (char)154, (char)174, (char)55, (char)216, (char)20}));
            assert(pack.cam_definition_uri_LEN(ph) == 105);
            assert(pack.cam_definition_uri_TRY(ph).equals("iotvGfsfxnlDNecworseUuovhzeozsspboHlnUAectzedmyethshoginmbesnjrdTtnqfvubslzlcihccnreLcgqzrbpkJnvRzqhyiuft"));
            assert(pack.time_boot_ms_GET() == 2148622731L);
            assert(pack.resolution_h_GET() == (char)2033);
            assert(pack.sensor_size_h_GET() == -2.7648073E37F);
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.focal_length_SET(2.044233E38F) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE)) ;
        p259.model_name_SET(new char[] {(char)123, (char)188, (char)111, (char)14, (char)29, (char)59, (char)25, (char)249, (char)83, (char)121, (char)62, (char)99, (char)252, (char)119, (char)195, (char)38, (char)81, (char)120, (char)119, (char)205, (char)150, (char)53, (char)195, (char)244, (char)58, (char)40, (char)75, (char)24, (char)35, (char)254, (char)103, (char)164}, 0) ;
        p259.cam_definition_uri_SET("iotvGfsfxnlDNecworseUuovhzeozsspboHlnUAectzedmyethshoginmbesnjrdTtnqfvubslzlcihccnreLcgqzrbpkJnvRzqhyiuft", PH) ;
        p259.resolution_h_SET((char)2033) ;
        p259.cam_definition_version_SET((char)47105) ;
        p259.time_boot_ms_SET(2148622731L) ;
        p259.sensor_size_h_SET(-2.7648073E37F) ;
        p259.sensor_size_v_SET(6.0301064E37F) ;
        p259.resolution_v_SET((char)23857) ;
        p259.firmware_version_SET(482392188L) ;
        p259.vendor_name_SET(new char[] {(char)218, (char)58, (char)220, (char)204, (char)3, (char)71, (char)38, (char)7, (char)154, (char)63, (char)254, (char)62, (char)169, (char)186, (char)245, (char)88, (char)225, (char)51, (char)191, (char)139, (char)158, (char)218, (char)125, (char)44, (char)132, (char)229, (char)245, (char)154, (char)174, (char)55, (char)216, (char)20}, 0) ;
        p259.lens_id_SET((char)49) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == (CAMERA_MODE.CAMERA_MODE_VIDEO));
            assert(pack.time_boot_ms_GET() == 65298652L);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(65298652L) ;
        p260.mode_id_SET((CAMERA_MODE.CAMERA_MODE_VIDEO)) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 40062955L);
            assert(pack.storage_id_GET() == (char)149);
            assert(pack.storage_count_GET() == (char)16);
            assert(pack.write_speed_GET() == -2.9437958E38F);
            assert(pack.total_capacity_GET() == 2.944579E38F);
            assert(pack.read_speed_GET() == 3.393629E38F);
            assert(pack.used_capacity_GET() == -2.1652492E38F);
            assert(pack.status_GET() == (char)59);
            assert(pack.available_capacity_GET() == 2.8020555E38F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(40062955L) ;
        p261.write_speed_SET(-2.9437958E38F) ;
        p261.used_capacity_SET(-2.1652492E38F) ;
        p261.status_SET((char)59) ;
        p261.total_capacity_SET(2.944579E38F) ;
        p261.read_speed_SET(3.393629E38F) ;
        p261.storage_count_SET((char)16) ;
        p261.available_capacity_SET(2.8020555E38F) ;
        p261.storage_id_SET((char)149) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.video_status_GET() == (char)52);
            assert(pack.image_interval_GET() == -1.9798686E38F);
            assert(pack.image_status_GET() == (char)150);
            assert(pack.available_capacity_GET() == 2.8096015E37F);
            assert(pack.recording_time_ms_GET() == 3340368763L);
            assert(pack.time_boot_ms_GET() == 1716981392L);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.video_status_SET((char)52) ;
        p262.image_status_SET((char)150) ;
        p262.recording_time_ms_SET(3340368763L) ;
        p262.available_capacity_SET(2.8096015E37F) ;
        p262.image_interval_SET(-1.9798686E38F) ;
        p262.time_boot_ms_SET(1716981392L) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.image_index_GET() == -548482012);
            assert(pack.relative_alt_GET() == 1295941823);
            assert(pack.alt_GET() == 1020382569);
            assert(pack.time_boot_ms_GET() == 1409641749L);
            assert(pack.time_utc_GET() == 2688721648026843229L);
            assert(pack.camera_id_GET() == (char)16);
            assert(pack.capture_result_GET() == (byte)28);
            assert(pack.lon_GET() == 928208998);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-9.755043E37F, -1.6112387E38F, 4.765224E37F, 3.0132006E38F}));
            assert(pack.lat_GET() == -163858186);
            assert(pack.file_url_LEN(ph) == 165);
            assert(pack.file_url_TRY(ph).equals("wgqltxpnisbgQMpgsOekRKetzkmdjjifeaoprfolfgiQcmircflyrjczdhiohwkaVvxczxGkIbgaftovjlkaBSrKicgjsdtjvoiwbjgpmfzcnqobEiphhqrjhtdqKbupTvbropoqxqAzycdomzmRiluochtCsaHpwyrat"));
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.relative_alt_SET(1295941823) ;
        p263.time_utc_SET(2688721648026843229L) ;
        p263.lon_SET(928208998) ;
        p263.time_boot_ms_SET(1409641749L) ;
        p263.image_index_SET(-548482012) ;
        p263.alt_SET(1020382569) ;
        p263.q_SET(new float[] {-9.755043E37F, -1.6112387E38F, 4.765224E37F, 3.0132006E38F}, 0) ;
        p263.file_url_SET("wgqltxpnisbgQMpgsOekRKetzkmdjjifeaoprfolfgiQcmircflyrjczdhiohwkaVvxczxGkIbgaftovjlkaBSrKicgjsdtjvoiwbjgpmfzcnqobEiphhqrjhtdqKbupTvbropoqxqAzycdomzmRiluochtCsaHpwyrat", PH) ;
        p263.lat_SET(-163858186) ;
        p263.camera_id_SET((char)16) ;
        p263.capture_result_SET((byte)28) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.arming_time_utc_GET() == 1216368159626127088L);
            assert(pack.flight_uuid_GET() == 6850143341170454775L);
            assert(pack.takeoff_time_utc_GET() == 4122562366262166954L);
            assert(pack.time_boot_ms_GET() == 3363543399L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.arming_time_utc_SET(1216368159626127088L) ;
        p264.time_boot_ms_SET(3363543399L) ;
        p264.flight_uuid_SET(6850143341170454775L) ;
        p264.takeoff_time_utc_SET(4122562366262166954L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 1.962823E38F);
            assert(pack.roll_GET() == -9.191465E37F);
            assert(pack.yaw_GET() == -5.5126357E37F);
            assert(pack.time_boot_ms_GET() == 927041234L);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.pitch_SET(1.962823E38F) ;
        p265.yaw_SET(-5.5126357E37F) ;
        p265.time_boot_ms_SET(927041234L) ;
        p265.roll_SET(-9.191465E37F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.first_message_offset_GET() == (char)192);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)21, (char)143, (char)221, (char)156, (char)33, (char)105, (char)42, (char)61, (char)191, (char)211, (char)255, (char)12, (char)71, (char)198, (char)234, (char)217, (char)178, (char)135, (char)57, (char)201, (char)175, (char)61, (char)37, (char)132, (char)108, (char)124, (char)37, (char)165, (char)225, (char)133, (char)203, (char)65, (char)121, (char)76, (char)233, (char)110, (char)199, (char)220, (char)105, (char)73, (char)237, (char)92, (char)136, (char)17, (char)70, (char)160, (char)63, (char)208, (char)246, (char)5, (char)163, (char)157, (char)14, (char)74, (char)159, (char)14, (char)69, (char)183, (char)147, (char)17, (char)50, (char)198, (char)156, (char)128, (char)112, (char)83, (char)58, (char)149, (char)177, (char)106, (char)16, (char)144, (char)109, (char)247, (char)184, (char)86, (char)184, (char)214, (char)48, (char)139, (char)51, (char)119, (char)222, (char)217, (char)121, (char)42, (char)119, (char)60, (char)224, (char)167, (char)239, (char)181, (char)203, (char)255, (char)206, (char)219, (char)63, (char)134, (char)149, (char)140, (char)181, (char)203, (char)170, (char)235, (char)101, (char)128, (char)85, (char)237, (char)183, (char)40, (char)112, (char)134, (char)182, (char)57, (char)84, (char)67, (char)106, (char)231, (char)183, (char)53, (char)208, (char)168, (char)0, (char)43, (char)135, (char)163, (char)86, (char)32, (char)172, (char)215, (char)229, (char)66, (char)139, (char)251, (char)74, (char)214, (char)242, (char)160, (char)74, (char)212, (char)233, (char)64, (char)203, (char)213, (char)80, (char)32, (char)252, (char)148, (char)206, (char)29, (char)46, (char)185, (char)156, (char)74, (char)34, (char)151, (char)245, (char)244, (char)192, (char)243, (char)134, (char)0, (char)255, (char)63, (char)18, (char)129, (char)77, (char)128, (char)131, (char)236, (char)155, (char)187, (char)44, (char)33, (char)99, (char)235, (char)158, (char)223, (char)21, (char)33, (char)225, (char)211, (char)203, (char)88, (char)85, (char)43, (char)101, (char)195, (char)9, (char)22, (char)163, (char)15, (char)113, (char)45, (char)202, (char)178, (char)69, (char)96, (char)107, (char)193, (char)105, (char)216, (char)255, (char)162, (char)239, (char)80, (char)117, (char)207, (char)189, (char)162, (char)108, (char)229, (char)48, (char)182, (char)44, (char)26, (char)155, (char)48, (char)43, (char)141, (char)81, (char)76, (char)4, (char)185, (char)191, (char)114, (char)120, (char)165, (char)86, (char)46, (char)48, (char)71, (char)85, (char)116, (char)230, (char)65, (char)232, (char)162, (char)212, (char)223, (char)119, (char)5, (char)131, (char)212, (char)104, (char)138, (char)64, (char)35, (char)234}));
            assert(pack.length_GET() == (char)209);
            assert(pack.target_component_GET() == (char)3);
            assert(pack.target_system_GET() == (char)185);
            assert(pack.sequence_GET() == (char)44558);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.data__SET(new char[] {(char)21, (char)143, (char)221, (char)156, (char)33, (char)105, (char)42, (char)61, (char)191, (char)211, (char)255, (char)12, (char)71, (char)198, (char)234, (char)217, (char)178, (char)135, (char)57, (char)201, (char)175, (char)61, (char)37, (char)132, (char)108, (char)124, (char)37, (char)165, (char)225, (char)133, (char)203, (char)65, (char)121, (char)76, (char)233, (char)110, (char)199, (char)220, (char)105, (char)73, (char)237, (char)92, (char)136, (char)17, (char)70, (char)160, (char)63, (char)208, (char)246, (char)5, (char)163, (char)157, (char)14, (char)74, (char)159, (char)14, (char)69, (char)183, (char)147, (char)17, (char)50, (char)198, (char)156, (char)128, (char)112, (char)83, (char)58, (char)149, (char)177, (char)106, (char)16, (char)144, (char)109, (char)247, (char)184, (char)86, (char)184, (char)214, (char)48, (char)139, (char)51, (char)119, (char)222, (char)217, (char)121, (char)42, (char)119, (char)60, (char)224, (char)167, (char)239, (char)181, (char)203, (char)255, (char)206, (char)219, (char)63, (char)134, (char)149, (char)140, (char)181, (char)203, (char)170, (char)235, (char)101, (char)128, (char)85, (char)237, (char)183, (char)40, (char)112, (char)134, (char)182, (char)57, (char)84, (char)67, (char)106, (char)231, (char)183, (char)53, (char)208, (char)168, (char)0, (char)43, (char)135, (char)163, (char)86, (char)32, (char)172, (char)215, (char)229, (char)66, (char)139, (char)251, (char)74, (char)214, (char)242, (char)160, (char)74, (char)212, (char)233, (char)64, (char)203, (char)213, (char)80, (char)32, (char)252, (char)148, (char)206, (char)29, (char)46, (char)185, (char)156, (char)74, (char)34, (char)151, (char)245, (char)244, (char)192, (char)243, (char)134, (char)0, (char)255, (char)63, (char)18, (char)129, (char)77, (char)128, (char)131, (char)236, (char)155, (char)187, (char)44, (char)33, (char)99, (char)235, (char)158, (char)223, (char)21, (char)33, (char)225, (char)211, (char)203, (char)88, (char)85, (char)43, (char)101, (char)195, (char)9, (char)22, (char)163, (char)15, (char)113, (char)45, (char)202, (char)178, (char)69, (char)96, (char)107, (char)193, (char)105, (char)216, (char)255, (char)162, (char)239, (char)80, (char)117, (char)207, (char)189, (char)162, (char)108, (char)229, (char)48, (char)182, (char)44, (char)26, (char)155, (char)48, (char)43, (char)141, (char)81, (char)76, (char)4, (char)185, (char)191, (char)114, (char)120, (char)165, (char)86, (char)46, (char)48, (char)71, (char)85, (char)116, (char)230, (char)65, (char)232, (char)162, (char)212, (char)223, (char)119, (char)5, (char)131, (char)212, (char)104, (char)138, (char)64, (char)35, (char)234}, 0) ;
        p266.sequence_SET((char)44558) ;
        p266.target_system_SET((char)185) ;
        p266.first_message_offset_SET((char)192) ;
        p266.target_component_SET((char)3) ;
        p266.length_SET((char)209) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)222, (char)21, (char)27, (char)146, (char)90, (char)130, (char)133, (char)118, (char)214, (char)188, (char)154, (char)109, (char)16, (char)234, (char)32, (char)119, (char)208, (char)83, (char)46, (char)69, (char)116, (char)223, (char)230, (char)133, (char)59, (char)161, (char)197, (char)148, (char)95, (char)141, (char)130, (char)209, (char)189, (char)94, (char)155, (char)122, (char)8, (char)195, (char)249, (char)108, (char)246, (char)112, (char)160, (char)53, (char)97, (char)47, (char)124, (char)161, (char)158, (char)20, (char)36, (char)91, (char)72, (char)178, (char)168, (char)169, (char)114, (char)54, (char)215, (char)160, (char)42, (char)230, (char)29, (char)16, (char)156, (char)154, (char)100, (char)246, (char)157, (char)121, (char)90, (char)65, (char)84, (char)3, (char)171, (char)168, (char)139, (char)198, (char)92, (char)1, (char)141, (char)29, (char)16, (char)250, (char)35, (char)56, (char)44, (char)192, (char)147, (char)139, (char)46, (char)109, (char)92, (char)152, (char)147, (char)58, (char)26, (char)249, (char)251, (char)152, (char)142, (char)129, (char)103, (char)78, (char)211, (char)174, (char)167, (char)215, (char)15, (char)84, (char)104, (char)181, (char)106, (char)141, (char)245, (char)125, (char)130, (char)44, (char)176, (char)240, (char)68, (char)174, (char)251, (char)0, (char)38, (char)5, (char)137, (char)37, (char)68, (char)148, (char)89, (char)111, (char)229, (char)24, (char)161, (char)65, (char)108, (char)5, (char)229, (char)112, (char)234, (char)175, (char)16, (char)209, (char)153, (char)167, (char)110, (char)80, (char)182, (char)92, (char)46, (char)198, (char)1, (char)73, (char)216, (char)179, (char)192, (char)117, (char)91, (char)12, (char)3, (char)26, (char)121, (char)112, (char)4, (char)119, (char)63, (char)145, (char)52, (char)171, (char)0, (char)141, (char)58, (char)81, (char)168, (char)180, (char)175, (char)12, (char)247, (char)208, (char)16, (char)163, (char)51, (char)149, (char)82, (char)22, (char)132, (char)146, (char)95, (char)66, (char)37, (char)92, (char)220, (char)97, (char)193, (char)226, (char)181, (char)110, (char)86, (char)136, (char)134, (char)25, (char)65, (char)245, (char)28, (char)238, (char)174, (char)16, (char)19, (char)197, (char)52, (char)191, (char)5, (char)98, (char)38, (char)10, (char)9, (char)54, (char)152, (char)216, (char)250, (char)141, (char)236, (char)51, (char)64, (char)37, (char)217, (char)197, (char)66, (char)249, (char)158, (char)83, (char)50, (char)247, (char)122, (char)240, (char)250, (char)253, (char)44, (char)239, (char)199, (char)129, (char)235, (char)191, (char)74, (char)168, (char)69, (char)18, (char)167}));
            assert(pack.target_system_GET() == (char)253);
            assert(pack.length_GET() == (char)88);
            assert(pack.sequence_GET() == (char)30851);
            assert(pack.first_message_offset_GET() == (char)221);
            assert(pack.target_component_GET() == (char)231);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.first_message_offset_SET((char)221) ;
        p267.sequence_SET((char)30851) ;
        p267.data__SET(new char[] {(char)222, (char)21, (char)27, (char)146, (char)90, (char)130, (char)133, (char)118, (char)214, (char)188, (char)154, (char)109, (char)16, (char)234, (char)32, (char)119, (char)208, (char)83, (char)46, (char)69, (char)116, (char)223, (char)230, (char)133, (char)59, (char)161, (char)197, (char)148, (char)95, (char)141, (char)130, (char)209, (char)189, (char)94, (char)155, (char)122, (char)8, (char)195, (char)249, (char)108, (char)246, (char)112, (char)160, (char)53, (char)97, (char)47, (char)124, (char)161, (char)158, (char)20, (char)36, (char)91, (char)72, (char)178, (char)168, (char)169, (char)114, (char)54, (char)215, (char)160, (char)42, (char)230, (char)29, (char)16, (char)156, (char)154, (char)100, (char)246, (char)157, (char)121, (char)90, (char)65, (char)84, (char)3, (char)171, (char)168, (char)139, (char)198, (char)92, (char)1, (char)141, (char)29, (char)16, (char)250, (char)35, (char)56, (char)44, (char)192, (char)147, (char)139, (char)46, (char)109, (char)92, (char)152, (char)147, (char)58, (char)26, (char)249, (char)251, (char)152, (char)142, (char)129, (char)103, (char)78, (char)211, (char)174, (char)167, (char)215, (char)15, (char)84, (char)104, (char)181, (char)106, (char)141, (char)245, (char)125, (char)130, (char)44, (char)176, (char)240, (char)68, (char)174, (char)251, (char)0, (char)38, (char)5, (char)137, (char)37, (char)68, (char)148, (char)89, (char)111, (char)229, (char)24, (char)161, (char)65, (char)108, (char)5, (char)229, (char)112, (char)234, (char)175, (char)16, (char)209, (char)153, (char)167, (char)110, (char)80, (char)182, (char)92, (char)46, (char)198, (char)1, (char)73, (char)216, (char)179, (char)192, (char)117, (char)91, (char)12, (char)3, (char)26, (char)121, (char)112, (char)4, (char)119, (char)63, (char)145, (char)52, (char)171, (char)0, (char)141, (char)58, (char)81, (char)168, (char)180, (char)175, (char)12, (char)247, (char)208, (char)16, (char)163, (char)51, (char)149, (char)82, (char)22, (char)132, (char)146, (char)95, (char)66, (char)37, (char)92, (char)220, (char)97, (char)193, (char)226, (char)181, (char)110, (char)86, (char)136, (char)134, (char)25, (char)65, (char)245, (char)28, (char)238, (char)174, (char)16, (char)19, (char)197, (char)52, (char)191, (char)5, (char)98, (char)38, (char)10, (char)9, (char)54, (char)152, (char)216, (char)250, (char)141, (char)236, (char)51, (char)64, (char)37, (char)217, (char)197, (char)66, (char)249, (char)158, (char)83, (char)50, (char)247, (char)122, (char)240, (char)250, (char)253, (char)44, (char)239, (char)199, (char)129, (char)235, (char)191, (char)74, (char)168, (char)69, (char)18, (char)167}, 0) ;
        p267.length_SET((char)88) ;
        p267.target_component_SET((char)231) ;
        p267.target_system_SET((char)253) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)28707);
            assert(pack.target_system_GET() == (char)78);
            assert(pack.target_component_GET() == (char)214);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.sequence_SET((char)28707) ;
        p268.target_system_SET((char)78) ;
        p268.target_component_SET((char)214) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.framerate_GET() == 9.213043E37F);
            assert(pack.bitrate_GET() == 1135963229L);
            assert(pack.resolution_v_GET() == (char)51963);
            assert(pack.uri_LEN(ph) == 19);
            assert(pack.uri_TRY(ph).equals("mohdthbnqhuancenspr"));
            assert(pack.status_GET() == (char)210);
            assert(pack.camera_id_GET() == (char)233);
            assert(pack.rotation_GET() == (char)7318);
            assert(pack.resolution_h_GET() == (char)23567);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.resolution_v_SET((char)51963) ;
        p269.rotation_SET((char)7318) ;
        p269.status_SET((char)210) ;
        p269.bitrate_SET(1135963229L) ;
        p269.camera_id_SET((char)233) ;
        p269.resolution_h_SET((char)23567) ;
        p269.uri_SET("mohdthbnqhuancenspr", PH) ;
        p269.framerate_SET(9.213043E37F) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.camera_id_GET() == (char)83);
            assert(pack.framerate_GET() == -1.8344202E38F);
            assert(pack.uri_LEN(ph) == 34);
            assert(pack.uri_TRY(ph).equals("gmdpmctxeezpgwcfrritnmQqgguthmater"));
            assert(pack.resolution_v_GET() == (char)41830);
            assert(pack.target_system_GET() == (char)172);
            assert(pack.bitrate_GET() == 1626160554L);
            assert(pack.rotation_GET() == (char)10798);
            assert(pack.target_component_GET() == (char)71);
            assert(pack.resolution_h_GET() == (char)38275);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.framerate_SET(-1.8344202E38F) ;
        p270.bitrate_SET(1626160554L) ;
        p270.rotation_SET((char)10798) ;
        p270.target_system_SET((char)172) ;
        p270.camera_id_SET((char)83) ;
        p270.uri_SET("gmdpmctxeezpgwcfrritnmQqgguthmater", PH) ;
        p270.target_component_SET((char)71) ;
        p270.resolution_h_SET((char)38275) ;
        p270.resolution_v_SET((char)41830) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 31);
            assert(pack.ssid_TRY(ph).equals("xlLyqxwcwgdnidfeGlhqwzEgmhxawgn"));
            assert(pack.password_LEN(ph) == 62);
            assert(pack.password_TRY(ph).equals("lckxxjiaXirJatjvfcmVtmumzsgzriisYdduhkkrntwicqdkennSilkekaIeer"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("lckxxjiaXirJatjvfcmVtmumzsgzriisYdduhkkrntwicqdkennSilkekaIeer", PH) ;
        p299.ssid_SET("xlLyqxwcwgdnidfeGlhqwzEgmhxawgn", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.min_version_GET() == (char)39056);
            assert(pack.max_version_GET() == (char)47839);
            assert(pack.version_GET() == (char)48538);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)227, (char)153, (char)243, (char)102, (char)247, (char)253, (char)210, (char)25}));
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)126, (char)166, (char)178, (char)45, (char)225, (char)123, (char)193, (char)224}));
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.library_version_hash_SET(new char[] {(char)126, (char)166, (char)178, (char)45, (char)225, (char)123, (char)193, (char)224}, 0) ;
        p300.max_version_SET((char)47839) ;
        p300.version_SET((char)48538) ;
        p300.min_version_SET((char)39056) ;
        p300.spec_version_hash_SET(new char[] {(char)227, (char)153, (char)243, (char)102, (char)247, (char)253, (char)210, (char)25}, 0) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.uptime_sec_GET() == 3288860820L);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
            assert(pack.vendor_specific_status_code_GET() == (char)50449);
            assert(pack.time_usec_GET() == 525268879661695142L);
            assert(pack.sub_mode_GET() == (char)124);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.sub_mode_SET((char)124) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE) ;
        p310.vendor_specific_status_code_SET((char)50449) ;
        p310.time_usec_SET(525268879661695142L) ;
        p310.uptime_sec_SET(3288860820L) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.hw_version_major_GET() == (char)235);
            assert(pack.sw_version_major_GET() == (char)130);
            assert(pack.sw_version_minor_GET() == (char)232);
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("bsyZjksobl"));
            assert(pack.sw_vcs_commit_GET() == 2418199671L);
            assert(pack.uptime_sec_GET() == 4167883681L);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)48, (char)2, (char)50, (char)74, (char)53, (char)220, (char)147, (char)32, (char)196, (char)129, (char)176, (char)149, (char)89, (char)43, (char)137, (char)140}));
            assert(pack.time_usec_GET() == 4270800072323612784L);
            assert(pack.hw_version_minor_GET() == (char)139);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.uptime_sec_SET(4167883681L) ;
        p311.sw_version_major_SET((char)130) ;
        p311.hw_version_major_SET((char)235) ;
        p311.hw_version_minor_SET((char)139) ;
        p311.hw_unique_id_SET(new char[] {(char)48, (char)2, (char)50, (char)74, (char)53, (char)220, (char)147, (char)32, (char)196, (char)129, (char)176, (char)149, (char)89, (char)43, (char)137, (char)140}, 0) ;
        p311.name_SET("bsyZjksobl", PH) ;
        p311.time_usec_SET(4270800072323612784L) ;
        p311.sw_version_minor_SET((char)232) ;
        p311.sw_vcs_commit_SET(2418199671L) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)135);
            assert(pack.param_index_GET() == (short)31681);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("benCaydL"));
            assert(pack.target_component_GET() == (char)239);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)135) ;
        p320.param_index_SET((short)31681) ;
        p320.target_component_SET((char)239) ;
        p320.param_id_SET("benCaydL", PH) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)180);
            assert(pack.target_system_GET() == (char)29);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)180) ;
        p321.target_system_SET((char)29) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)19664);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
            assert(pack.param_index_GET() == (char)48746);
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("HRxdw"));
            assert(pack.param_value_LEN(ph) == 56);
            assert(pack.param_value_TRY(ph).equals("brFxhyfewCbwqxefqsfmcAKprdxziwoavqhsplcEqvljvVWsnajmXiam"));
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_value_SET("brFxhyfewCbwqxefqsfmcAKprdxziwoavqhsplcEqvljvVWsnajmXiam", PH) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32) ;
        p322.param_index_SET((char)48746) ;
        p322.param_id_SET("HRxdw", PH) ;
        p322.param_count_SET((char)19664) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("pnqdbbraip"));
            assert(pack.param_value_LEN(ph) == 107);
            assert(pack.param_value_TRY(ph).equals("WnstmpmrmiyndyyiatsrmjxudvtIomdmplbsfpdvfIeujfmpiuqgwqjlvhxludsbjkRrvqrgqrAitcsfcgyxtiaoszygyykirwtcuogbarf"));
            assert(pack.target_component_GET() == (char)4);
            assert(pack.target_system_GET() == (char)78);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)78) ;
        p323.target_component_SET((char)4) ;
        p323.param_value_SET("WnstmpmrmiyndyyiatsrmjxudvtIomdmplbsfpdvfIeujfmpiuqgwqjlvhxludsbjkRrvqrgqrAitcsfcgyxtiaoszygyykirwtcuogbarf", PH) ;
        p323.param_id_SET("pnqdbbraip", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_FAILED);
            assert(pack.param_value_LEN(ph) == 32);
            assert(pack.param_value_TRY(ph).equals("rbvqSffimxbckFbbwxdtvkysnuabTtbo"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
            assert(pack.param_id_LEN(ph) == 13);
            assert(pack.param_id_TRY(ph).equals("hijfysQdoufxa"));
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8) ;
        p324.param_id_SET("hijfysQdoufxa", PH) ;
        p324.param_value_SET("rbvqSffimxbckFbbwxdtvkysnuabTtbo", PH) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.min_distance_GET() == (char)36640);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
            assert(pack.time_usec_GET() == 4253526104251968417L);
            assert(pack.increment_GET() == (char)184);
            assert(pack.max_distance_GET() == (char)65054);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)53572, (char)11577, (char)13548, (char)23520, (char)43334, (char)51479, (char)11211, (char)25548, (char)28341, (char)44388, (char)62637, (char)22695, (char)45353, (char)18318, (char)37334, (char)41802, (char)21478, (char)55728, (char)19318, (char)26630, (char)36659, (char)54985, (char)40530, (char)37200, (char)34211, (char)27834, (char)54062, (char)17148, (char)40224, (char)54620, (char)19781, (char)57350, (char)42526, (char)32723, (char)44055, (char)15368, (char)54009, (char)60506, (char)57414, (char)35268, (char)20561, (char)5769, (char)1515, (char)8645, (char)27070, (char)35552, (char)51884, (char)12419, (char)26252, (char)59338, (char)59187, (char)27725, (char)27802, (char)38051, (char)47029, (char)20692, (char)49995, (char)2450, (char)53596, (char)17403, (char)6231, (char)47368, (char)58062, (char)63206, (char)6034, (char)21075, (char)38145, (char)57403, (char)46853, (char)50511, (char)49201, (char)1824}));
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED) ;
        p330.increment_SET((char)184) ;
        p330.min_distance_SET((char)36640) ;
        p330.max_distance_SET((char)65054) ;
        p330.distances_SET(new char[] {(char)53572, (char)11577, (char)13548, (char)23520, (char)43334, (char)51479, (char)11211, (char)25548, (char)28341, (char)44388, (char)62637, (char)22695, (char)45353, (char)18318, (char)37334, (char)41802, (char)21478, (char)55728, (char)19318, (char)26630, (char)36659, (char)54985, (char)40530, (char)37200, (char)34211, (char)27834, (char)54062, (char)17148, (char)40224, (char)54620, (char)19781, (char)57350, (char)42526, (char)32723, (char)44055, (char)15368, (char)54009, (char)60506, (char)57414, (char)35268, (char)20561, (char)5769, (char)1515, (char)8645, (char)27070, (char)35552, (char)51884, (char)12419, (char)26252, (char)59338, (char)59187, (char)27725, (char)27802, (char)38051, (char)47029, (char)20692, (char)49995, (char)2450, (char)53596, (char)17403, (char)6231, (char)47368, (char)58062, (char)63206, (char)6034, (char)21075, (char)38145, (char)57403, (char)46853, (char)50511, (char)49201, (char)1824}, 0) ;
        p330.time_usec_SET(4253526104251968417L) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
    }

}

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
            long id = id__y(src);
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
        *	 was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS*/
        public void progress_SET(char  src, Bounds.Inside ph)
        {
            if(ph.field_bit != 11)insert_field(ph, 11, 0);
            set_bytes((char)(src) & -1L, 1, data,  ph.BYTE);
        }/**
*WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to
*	 be denied*/
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
            assert(pack.custom_mode_GET() == 1908578915L);
            assert(pack.mavlink_version_GET() == (char)180);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_GIMBAL);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED));
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_STANDBY);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.system_status_SET(MAV_STATE.MAV_STATE_STANDBY) ;
        p0.custom_mode_SET(1908578915L) ;
        p0.mavlink_version_SET((char)180) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_GIMBAL) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED)) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.load_GET() == (char)44371);
            assert(pack.errors_count4_GET() == (char)42880);
            assert(pack.drop_rate_comm_GET() == (char)53097);
            assert(pack.errors_comm_GET() == (char)59600);
            assert(pack.voltage_battery_GET() == (char)20798);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING));
            assert(pack.errors_count2_GET() == (char)58851);
            assert(pack.battery_remaining_GET() == (byte)35);
            assert(pack.current_battery_GET() == (short) -19436);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
            assert(pack.errors_count3_GET() == (char)34221);
            assert(pack.errors_count1_GET() == (char)26191);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.errors_count1_SET((char)26191) ;
        p1.errors_count4_SET((char)42880) ;
        p1.load_SET((char)44371) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.battery_remaining_SET((byte)35) ;
        p1.current_battery_SET((short) -19436) ;
        p1.errors_comm_SET((char)59600) ;
        p1.voltage_battery_SET((char)20798) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING)) ;
        p1.errors_count2_SET((char)58851) ;
        p1.drop_rate_comm_SET((char)53097) ;
        p1.errors_count3_SET((char)34221) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 6974244611869967759L);
            assert(pack.time_boot_ms_GET() == 1546414074L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(6974244611869967759L) ;
        p2.time_boot_ms_SET(1546414074L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.afz_GET() == -5.341594E37F);
            assert(pack.vz_GET() == 2.1181985E38F);
            assert(pack.x_GET() == -3.1588389E38F);
            assert(pack.afy_GET() == 1.072155E38F);
            assert(pack.yaw_GET() == -2.3812575E38F);
            assert(pack.afx_GET() == 2.7109864E37F);
            assert(pack.vx_GET() == 1.0230856E38F);
            assert(pack.z_GET() == 1.228502E38F);
            assert(pack.y_GET() == -1.1779219E38F);
            assert(pack.yaw_rate_GET() == -2.2105816E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.type_mask_GET() == (char)24948);
            assert(pack.vy_GET() == 1.9795968E38F);
            assert(pack.time_boot_ms_GET() == 3195812624L);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.y_SET(-1.1779219E38F) ;
        p3.afz_SET(-5.341594E37F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p3.vx_SET(1.0230856E38F) ;
        p3.vz_SET(2.1181985E38F) ;
        p3.z_SET(1.228502E38F) ;
        p3.yaw_SET(-2.3812575E38F) ;
        p3.type_mask_SET((char)24948) ;
        p3.time_boot_ms_SET(3195812624L) ;
        p3.afx_SET(2.7109864E37F) ;
        p3.afy_SET(1.072155E38F) ;
        p3.yaw_rate_SET(-2.2105816E37F) ;
        p3.x_SET(-3.1588389E38F) ;
        p3.vy_SET(1.9795968E38F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 9123560753951550508L);
            assert(pack.target_system_GET() == (char)237);
            assert(pack.seq_GET() == 3321604855L);
            assert(pack.target_component_GET() == (char)71);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.seq_SET(3321604855L) ;
        p4.target_component_SET((char)71) ;
        p4.target_system_SET((char)237) ;
        p4.time_usec_SET(9123560753951550508L) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)25);
            assert(pack.passkey_LEN(ph) == 25);
            assert(pack.passkey_TRY(ph).equals("vcyJrdHgbNreydgzzauopilxo"));
            assert(pack.control_request_GET() == (char)113);
            assert(pack.version_GET() == (char)43);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.version_SET((char)43) ;
        p5.target_system_SET((char)25) ;
        p5.control_request_SET((char)113) ;
        p5.passkey_SET("vcyJrdHgbNreydgzzauopilxo", PH) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)51);
            assert(pack.gcs_system_id_GET() == (char)190);
            assert(pack.ack_GET() == (char)95);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.control_request_SET((char)51) ;
        p6.ack_SET((char)95) ;
        p6.gcs_system_id_SET((char)190) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 17);
            assert(pack.key_TRY(ph).equals("oyiwmrgrnkwidcmrn"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("oyiwmrgrnkwidcmrn", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_AUTO_ARMED);
            assert(pack.target_system_GET() == (char)231);
            assert(pack.custom_mode_GET() == 1702789546L);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)231) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED) ;
        p11.custom_mode_SET(1702789546L) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short)13693);
            assert(pack.target_system_GET() == (char)1);
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("kuiao"));
            assert(pack.target_component_GET() == (char)19);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_index_SET((short)13693) ;
        p20.target_system_SET((char)1) ;
        p20.target_component_SET((char)19) ;
        p20.param_id_SET("kuiao", PH) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)47);
            assert(pack.target_system_GET() == (char)189);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)189) ;
        p21.target_component_SET((char)47) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (char)45355);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
            assert(pack.param_value_GET() == -9.915982E37F);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("gx"));
            assert(pack.param_count_GET() == (char)61519);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64) ;
        p22.param_count_SET((char)61519) ;
        p22.param_value_SET(-9.915982E37F) ;
        p22.param_index_SET((char)45355) ;
        p22.param_id_SET("gx", PH) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)1);
            assert(pack.target_component_GET() == (char)27);
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("eaxnj"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
            assert(pack.param_value_GET() == 2.3587242E38F);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.target_component_SET((char)27) ;
        p23.param_value_SET(2.3587242E38F) ;
        p23.param_id_SET("eaxnj", PH) ;
        p23.target_system_SET((char)1) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.eph_GET() == (char)46083);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
            assert(pack.lat_GET() == 179623347);
            assert(pack.epv_GET() == (char)6638);
            assert(pack.hdg_acc_TRY(ph) == 2181716654L);
            assert(pack.vel_acc_TRY(ph) == 2136744064L);
            assert(pack.vel_GET() == (char)57423);
            assert(pack.time_usec_GET() == 3970103276513598766L);
            assert(pack.alt_ellipsoid_TRY(ph) == 1109760690);
            assert(pack.lon_GET() == -916449300);
            assert(pack.v_acc_TRY(ph) == 3038543275L);
            assert(pack.satellites_visible_GET() == (char)124);
            assert(pack.alt_GET() == 1194193356);
            assert(pack.cog_GET() == (char)62746);
            assert(pack.h_acc_TRY(ph) == 1860475696L);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX) ;
        p24.eph_SET((char)46083) ;
        p24.cog_SET((char)62746) ;
        p24.alt_SET(1194193356) ;
        p24.vel_acc_SET(2136744064L, PH) ;
        p24.vel_SET((char)57423) ;
        p24.satellites_visible_SET((char)124) ;
        p24.alt_ellipsoid_SET(1109760690, PH) ;
        p24.hdg_acc_SET(2181716654L, PH) ;
        p24.v_acc_SET(3038543275L, PH) ;
        p24.time_usec_SET(3970103276513598766L) ;
        p24.lon_SET(-916449300) ;
        p24.lat_SET(179623347) ;
        p24.h_acc_SET(1860475696L, PH) ;
        p24.epv_SET((char)6638) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)96);
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)61, (char)14, (char)14, (char)145, (char)191, (char)213, (char)42, (char)67, (char)133, (char)209, (char)148, (char)18, (char)167, (char)142, (char)114, (char)243, (char)226, (char)57, (char)103, (char)29}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)19, (char)201, (char)195, (char)180, (char)232, (char)76, (char)204, (char)125, (char)178, (char)139, (char)188, (char)140, (char)98, (char)140, (char)15, (char)78, (char)82, (char)69, (char)151, (char)68}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)44, (char)20, (char)23, (char)218, (char)248, (char)151, (char)225, (char)11, (char)50, (char)112, (char)201, (char)155, (char)197, (char)0, (char)89, (char)105, (char)182, (char)200, (char)52, (char)232}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)128, (char)221, (char)119, (char)66, (char)46, (char)57, (char)130, (char)80, (char)148, (char)5, (char)192, (char)188, (char)116, (char)166, (char)178, (char)26, (char)61, (char)146, (char)16, (char)68}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)53, (char)66, (char)115, (char)81, (char)150, (char)1, (char)154, (char)30, (char)198, (char)109, (char)22, (char)203, (char)190, (char)8, (char)74, (char)19, (char)57, (char)165, (char)91, (char)83}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_snr_SET(new char[] {(char)19, (char)201, (char)195, (char)180, (char)232, (char)76, (char)204, (char)125, (char)178, (char)139, (char)188, (char)140, (char)98, (char)140, (char)15, (char)78, (char)82, (char)69, (char)151, (char)68}, 0) ;
        p25.satellites_visible_SET((char)96) ;
        p25.satellite_elevation_SET(new char[] {(char)53, (char)66, (char)115, (char)81, (char)150, (char)1, (char)154, (char)30, (char)198, (char)109, (char)22, (char)203, (char)190, (char)8, (char)74, (char)19, (char)57, (char)165, (char)91, (char)83}, 0) ;
        p25.satellite_used_SET(new char[] {(char)128, (char)221, (char)119, (char)66, (char)46, (char)57, (char)130, (char)80, (char)148, (char)5, (char)192, (char)188, (char)116, (char)166, (char)178, (char)26, (char)61, (char)146, (char)16, (char)68}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)61, (char)14, (char)14, (char)145, (char)191, (char)213, (char)42, (char)67, (char)133, (char)209, (char)148, (char)18, (char)167, (char)142, (char)114, (char)243, (char)226, (char)57, (char)103, (char)29}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)44, (char)20, (char)23, (char)218, (char)248, (char)151, (char)225, (char)11, (char)50, (char)112, (char)201, (char)155, (char)197, (char)0, (char)89, (char)105, (char)182, (char)200, (char)52, (char)232}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == (short)7269);
            assert(pack.yacc_GET() == (short)11565);
            assert(pack.zgyro_GET() == (short)2516);
            assert(pack.ygyro_GET() == (short) -22164);
            assert(pack.time_boot_ms_GET() == 966324642L);
            assert(pack.zacc_GET() == (short)7241);
            assert(pack.ymag_GET() == (short) -16256);
            assert(pack.xgyro_GET() == (short) -24666);
            assert(pack.xmag_GET() == (short) -9509);
            assert(pack.xacc_GET() == (short) -15095);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.ygyro_SET((short) -22164) ;
        p26.xmag_SET((short) -9509) ;
        p26.xacc_SET((short) -15095) ;
        p26.zmag_SET((short)7269) ;
        p26.zgyro_SET((short)2516) ;
        p26.ymag_SET((short) -16256) ;
        p26.yacc_SET((short)11565) ;
        p26.zacc_SET((short)7241) ;
        p26.time_boot_ms_SET(966324642L) ;
        p26.xgyro_SET((short) -24666) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short) -7443);
            assert(pack.ygyro_GET() == (short) -9559);
            assert(pack.zmag_GET() == (short) -3180);
            assert(pack.zacc_GET() == (short)3864);
            assert(pack.zgyro_GET() == (short)24779);
            assert(pack.ymag_GET() == (short)24621);
            assert(pack.xmag_GET() == (short)15029);
            assert(pack.time_usec_GET() == 7897297317828522968L);
            assert(pack.xgyro_GET() == (short) -22282);
            assert(pack.yacc_GET() == (short)8120);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.yacc_SET((short)8120) ;
        p27.zgyro_SET((short)24779) ;
        p27.xmag_SET((short)15029) ;
        p27.xgyro_SET((short) -22282) ;
        p27.ymag_SET((short)24621) ;
        p27.xacc_SET((short) -7443) ;
        p27.zmag_SET((short) -3180) ;
        p27.zacc_SET((short)3864) ;
        p27.ygyro_SET((short) -9559) ;
        p27.time_usec_SET(7897297317828522968L) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff1_GET() == (short) -2973);
            assert(pack.time_usec_GET() == 7684578813617800757L);
            assert(pack.press_diff2_GET() == (short) -13744);
            assert(pack.press_abs_GET() == (short) -24202);
            assert(pack.temperature_GET() == (short)18726);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_abs_SET((short) -24202) ;
        p28.temperature_SET((short)18726) ;
        p28.press_diff2_SET((short) -13744) ;
        p28.time_usec_SET(7684578813617800757L) ;
        p28.press_diff1_SET((short) -2973) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -2.9361313E38F);
            assert(pack.time_boot_ms_GET() == 1963618702L);
            assert(pack.press_diff_GET() == -1.7322344E38F);
            assert(pack.temperature_GET() == (short)18044);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_diff_SET(-1.7322344E38F) ;
        p29.time_boot_ms_SET(1963618702L) ;
        p29.temperature_SET((short)18044) ;
        p29.press_abs_SET(-2.9361313E38F) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -2.2102032E36F);
            assert(pack.yawspeed_GET() == -6.827216E37F);
            assert(pack.pitchspeed_GET() == 2.0653602E38F);
            assert(pack.time_boot_ms_GET() == 2907225154L);
            assert(pack.pitch_GET() == -2.6271598E38F);
            assert(pack.rollspeed_GET() == -3.3059679E38F);
            assert(pack.yaw_GET() == -1.0871065E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.pitchspeed_SET(2.0653602E38F) ;
        p30.yaw_SET(-1.0871065E38F) ;
        p30.yawspeed_SET(-6.827216E37F) ;
        p30.roll_SET(-2.2102032E36F) ;
        p30.rollspeed_SET(-3.3059679E38F) ;
        p30.pitch_SET(-2.6271598E38F) ;
        p30.time_boot_ms_SET(2907225154L) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q3_GET() == 3.350757E38F);
            assert(pack.time_boot_ms_GET() == 4230650739L);
            assert(pack.q1_GET() == -3.606498E37F);
            assert(pack.q2_GET() == -1.1973006E37F);
            assert(pack.rollspeed_GET() == 6.9150023E36F);
            assert(pack.pitchspeed_GET() == -8.18662E36F);
            assert(pack.yawspeed_GET() == 1.5553337E38F);
            assert(pack.q4_GET() == -2.4821994E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.yawspeed_SET(1.5553337E38F) ;
        p31.time_boot_ms_SET(4230650739L) ;
        p31.q3_SET(3.350757E38F) ;
        p31.pitchspeed_SET(-8.18662E36F) ;
        p31.q1_SET(-3.606498E37F) ;
        p31.rollspeed_SET(6.9150023E36F) ;
        p31.q2_SET(-1.1973006E37F) ;
        p31.q4_SET(-2.4821994E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 9.359689E37F);
            assert(pack.vx_GET() == 2.1407903E38F);
            assert(pack.vy_GET() == 1.4902059E37F);
            assert(pack.z_GET() == -3.395965E38F);
            assert(pack.x_GET() == -1.6067586E38F);
            assert(pack.time_boot_ms_GET() == 2524772422L);
            assert(pack.vz_GET() == -1.1109526E38F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.x_SET(-1.6067586E38F) ;
        p32.y_SET(9.359689E37F) ;
        p32.vz_SET(-1.1109526E38F) ;
        p32.time_boot_ms_SET(2524772422L) ;
        p32.z_SET(-3.395965E38F) ;
        p32.vy_SET(1.4902059E37F) ;
        p32.vx_SET(2.1407903E38F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -1262877348);
            assert(pack.time_boot_ms_GET() == 2223967208L);
            assert(pack.vx_GET() == (short) -26680);
            assert(pack.lon_GET() == 1510081021);
            assert(pack.relative_alt_GET() == -957933983);
            assert(pack.lat_GET() == -1144556649);
            assert(pack.vz_GET() == (short) -16729);
            assert(pack.vy_GET() == (short) -3390);
            assert(pack.hdg_GET() == (char)46012);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.relative_alt_SET(-957933983) ;
        p33.vy_SET((short) -3390) ;
        p33.lat_SET(-1144556649) ;
        p33.lon_SET(1510081021) ;
        p33.time_boot_ms_SET(2223967208L) ;
        p33.hdg_SET((char)46012) ;
        p33.vz_SET((short) -16729) ;
        p33.alt_SET(-1262877348) ;
        p33.vx_SET((short) -26680) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan2_scaled_GET() == (short) -23745);
            assert(pack.chan8_scaled_GET() == (short) -22226);
            assert(pack.chan6_scaled_GET() == (short)16283);
            assert(pack.chan1_scaled_GET() == (short)11472);
            assert(pack.chan7_scaled_GET() == (short)12632);
            assert(pack.rssi_GET() == (char)8);
            assert(pack.time_boot_ms_GET() == 2577431346L);
            assert(pack.chan3_scaled_GET() == (short) -31549);
            assert(pack.port_GET() == (char)15);
            assert(pack.chan4_scaled_GET() == (short) -10337);
            assert(pack.chan5_scaled_GET() == (short) -9684);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan4_scaled_SET((short) -10337) ;
        p34.chan7_scaled_SET((short)12632) ;
        p34.chan1_scaled_SET((short)11472) ;
        p34.chan8_scaled_SET((short) -22226) ;
        p34.chan2_scaled_SET((short) -23745) ;
        p34.port_SET((char)15) ;
        p34.rssi_SET((char)8) ;
        p34.chan5_scaled_SET((short) -9684) ;
        p34.chan3_scaled_SET((short) -31549) ;
        p34.time_boot_ms_SET(2577431346L) ;
        p34.chan6_scaled_SET((short)16283) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)4908);
            assert(pack.port_GET() == (char)105);
            assert(pack.chan1_raw_GET() == (char)22854);
            assert(pack.chan6_raw_GET() == (char)18678);
            assert(pack.rssi_GET() == (char)165);
            assert(pack.chan5_raw_GET() == (char)2287);
            assert(pack.time_boot_ms_GET() == 1452944602L);
            assert(pack.chan7_raw_GET() == (char)64986);
            assert(pack.chan8_raw_GET() == (char)47902);
            assert(pack.chan2_raw_GET() == (char)64857);
            assert(pack.chan3_raw_GET() == (char)58177);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.time_boot_ms_SET(1452944602L) ;
        p35.chan4_raw_SET((char)4908) ;
        p35.chan1_raw_SET((char)22854) ;
        p35.chan7_raw_SET((char)64986) ;
        p35.port_SET((char)105) ;
        p35.chan5_raw_SET((char)2287) ;
        p35.rssi_SET((char)165) ;
        p35.chan3_raw_SET((char)58177) ;
        p35.chan2_raw_SET((char)64857) ;
        p35.chan6_raw_SET((char)18678) ;
        p35.chan8_raw_SET((char)47902) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo12_raw_TRY(ph) == (char)6780);
            assert(pack.servo14_raw_TRY(ph) == (char)24721);
            assert(pack.servo7_raw_GET() == (char)23863);
            assert(pack.servo11_raw_TRY(ph) == (char)5345);
            assert(pack.servo2_raw_GET() == (char)2589);
            assert(pack.time_usec_GET() == 2088970805L);
            assert(pack.servo9_raw_TRY(ph) == (char)38285);
            assert(pack.servo15_raw_TRY(ph) == (char)54583);
            assert(pack.servo16_raw_TRY(ph) == (char)5364);
            assert(pack.servo6_raw_GET() == (char)14421);
            assert(pack.port_GET() == (char)228);
            assert(pack.servo8_raw_GET() == (char)56325);
            assert(pack.servo3_raw_GET() == (char)1117);
            assert(pack.servo13_raw_TRY(ph) == (char)52871);
            assert(pack.servo4_raw_GET() == (char)3803);
            assert(pack.servo1_raw_GET() == (char)15482);
            assert(pack.servo5_raw_GET() == (char)37730);
            assert(pack.servo10_raw_TRY(ph) == (char)25271);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo5_raw_SET((char)37730) ;
        p36.servo1_raw_SET((char)15482) ;
        p36.servo3_raw_SET((char)1117) ;
        p36.servo10_raw_SET((char)25271, PH) ;
        p36.servo2_raw_SET((char)2589) ;
        p36.servo12_raw_SET((char)6780, PH) ;
        p36.servo14_raw_SET((char)24721, PH) ;
        p36.servo4_raw_SET((char)3803) ;
        p36.servo11_raw_SET((char)5345, PH) ;
        p36.port_SET((char)228) ;
        p36.servo8_raw_SET((char)56325) ;
        p36.servo9_raw_SET((char)38285, PH) ;
        p36.time_usec_SET(2088970805L) ;
        p36.servo6_raw_SET((char)14421) ;
        p36.servo7_raw_SET((char)23863) ;
        p36.servo15_raw_SET((char)54583, PH) ;
        p36.servo13_raw_SET((char)52871, PH) ;
        p36.servo16_raw_SET((char)5364, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)120);
            assert(pack.start_index_GET() == (short) -30260);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.end_index_GET() == (short) -13101);
            assert(pack.target_component_GET() == (char)42);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_component_SET((char)42) ;
        p37.end_index_SET((short) -13101) ;
        p37.target_system_SET((char)120) ;
        p37.start_index_SET((short) -30260) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)82);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.start_index_GET() == (short)363);
            assert(pack.end_index_GET() == (short) -12115);
            assert(pack.target_component_GET() == (char)131);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p38.start_index_SET((short)363) ;
        p38.target_system_SET((char)82) ;
        p38.target_component_SET((char)131) ;
        p38.end_index_SET((short) -12115) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.9470377E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.target_component_GET() == (char)106);
            assert(pack.x_GET() == 1.3903004E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.current_GET() == (char)144);
            assert(pack.param1_GET() == -6.761255E37F);
            assert(pack.param4_GET() == 2.889999E38F);
            assert(pack.autocontinue_GET() == (char)2);
            assert(pack.target_system_GET() == (char)92);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY);
            assert(pack.param2_GET() == 3.1905591E38F);
            assert(pack.y_GET() == -3.5290652E37F);
            assert(pack.seq_GET() == (char)5973);
            assert(pack.param3_GET() == -1.1942928E38F);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.param3_SET(-1.1942928E38F) ;
        p39.param1_SET(-6.761255E37F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY) ;
        p39.seq_SET((char)5973) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p39.autocontinue_SET((char)2) ;
        p39.z_SET(-1.9470377E38F) ;
        p39.x_SET(1.3903004E38F) ;
        p39.y_SET(-3.5290652E37F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p39.param4_SET(2.889999E38F) ;
        p39.target_system_SET((char)92) ;
        p39.target_component_SET((char)106) ;
        p39.current_SET((char)144) ;
        p39.param2_SET(3.1905591E38F) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)13);
            assert(pack.target_system_GET() == (char)42);
            assert(pack.seq_GET() == (char)28975);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_system_SET((char)42) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p40.target_component_SET((char)13) ;
        p40.seq_SET((char)28975) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)21);
            assert(pack.seq_GET() == (char)38550);
            assert(pack.target_component_GET() == (char)75);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.seq_SET((char)38550) ;
        p41.target_system_SET((char)21) ;
        p41.target_component_SET((char)75) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)31166);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)31166) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)158);
            assert(pack.target_component_GET() == (char)207);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p43.target_system_SET((char)158) ;
        p43.target_component_SET((char)207) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)172);
            assert(pack.target_component_GET() == (char)202);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.count_GET() == (char)43799);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.count_SET((char)43799) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p44.target_system_SET((char)172) ;
        p44.target_component_SET((char)202) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)59);
            assert(pack.target_component_GET() == (char)137);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_component_SET((char)137) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p45.target_system_SET((char)59) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)47270);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)47270) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM6_Y);
            assert(pack.target_component_GET() == (char)50);
            assert(pack.target_system_GET() == (char)33);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM6_Y) ;
        p47.target_component_SET((char)50) ;
        p47.target_system_SET((char)33) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 2730921533724586523L);
            assert(pack.altitude_GET() == -442952795);
            assert(pack.latitude_GET() == 975034884);
            assert(pack.target_system_GET() == (char)246);
            assert(pack.longitude_GET() == -2015756111);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.altitude_SET(-442952795) ;
        p48.latitude_SET(975034884) ;
        p48.time_usec_SET(2730921533724586523L, PH) ;
        p48.target_system_SET((char)246) ;
        p48.longitude_SET(-2015756111) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == -1093935503);
            assert(pack.latitude_GET() == 973969328);
            assert(pack.altitude_GET() == 1800859583);
            assert(pack.time_usec_TRY(ph) == 3327175821639554784L);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.longitude_SET(-1093935503) ;
        p49.altitude_SET(1800859583) ;
        p49.latitude_SET(973969328) ;
        p49.time_usec_SET(3327175821639554784L, PH) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short)27346);
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("jfpilummrhtyint"));
            assert(pack.scale_GET() == 2.7803707E38F);
            assert(pack.param_value_min_GET() == -2.2807484E38F);
            assert(pack.param_value0_GET() == -3.1578807E37F);
            assert(pack.param_value_max_GET() == 4.4796734E37F);
            assert(pack.target_system_GET() == (char)221);
            assert(pack.target_component_GET() == (char)79);
            assert(pack.parameter_rc_channel_index_GET() == (char)216);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.scale_SET(2.7803707E38F) ;
        p50.parameter_rc_channel_index_SET((char)216) ;
        p50.target_component_SET((char)79) ;
        p50.param_value_max_SET(4.4796734E37F) ;
        p50.param_index_SET((short)27346) ;
        p50.target_system_SET((char)221) ;
        p50.param_value_min_SET(-2.2807484E38F) ;
        p50.param_id_SET("jfpilummrhtyint", PH) ;
        p50.param_value0_SET(-3.1578807E37F) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)24);
            assert(pack.seq_GET() == (char)34698);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)209);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p51.target_system_SET((char)209) ;
        p51.seq_SET((char)34698) ;
        p51.target_component_SET((char)24) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)143);
            assert(pack.p2y_GET() == -2.0560968E38F);
            assert(pack.p2x_GET() == -7.453323E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.p2z_GET() == 1.9899354E38F);
            assert(pack.p1z_GET() == 2.8678506E38F);
            assert(pack.target_component_GET() == (char)65);
            assert(pack.p1y_GET() == -1.3961105E38F);
            assert(pack.p1x_GET() == 2.7024382E38F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.target_component_SET((char)65) ;
        p54.p1y_SET(-1.3961105E38F) ;
        p54.p1x_SET(2.7024382E38F) ;
        p54.p2y_SET(-2.0560968E38F) ;
        p54.target_system_SET((char)143) ;
        p54.p1z_SET(2.8678506E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p54.p2x_SET(-7.453323E37F) ;
        p54.p2z_SET(1.9899354E38F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1y_GET() == 1.1857726E36F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.p1z_GET() == -5.708816E37F);
            assert(pack.p1x_GET() == -1.9885577E37F);
            assert(pack.p2x_GET() == -3.5205124E37F);
            assert(pack.p2y_GET() == -1.0372469E38F);
            assert(pack.p2z_GET() == 2.1616053E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p55.p1x_SET(-1.9885577E37F) ;
        p55.p1y_SET(1.1857726E36F) ;
        p55.p1z_SET(-5.708816E37F) ;
        p55.p2y_SET(-1.0372469E38F) ;
        p55.p2x_SET(-3.5205124E37F) ;
        p55.p2z_SET(2.1616053E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.6473149E38F, 2.0578421E38F, 2.3463022E38F, -9.328383E37F, -2.9406125E38F, -3.1521637E38F, -2.3122831E38F, -5.6282657E37F, -3.9846954E37F}));
            assert(pack.rollspeed_GET() == 3.1033506E38F);
            assert(pack.yawspeed_GET() == -2.6523062E38F);
            assert(pack.pitchspeed_GET() == -2.3297578E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.4265512E38F, 3.2284162E38F, 3.133729E38F, -3.3741155E38F}));
            assert(pack.time_usec_GET() == 4119391029458134419L);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.q_SET(new float[] {1.4265512E38F, 3.2284162E38F, 3.133729E38F, -3.3741155E38F}, 0) ;
        p61.time_usec_SET(4119391029458134419L) ;
        p61.rollspeed_SET(3.1033506E38F) ;
        p61.covariance_SET(new float[] {2.6473149E38F, 2.0578421E38F, 2.3463022E38F, -9.328383E37F, -2.9406125E38F, -3.1521637E38F, -2.3122831E38F, -5.6282657E37F, -3.9846954E37F}, 0) ;
        p61.yawspeed_SET(-2.6523062E38F) ;
        p61.pitchspeed_SET(-2.3297578E38F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_bearing_GET() == (short) -32438);
            assert(pack.wp_dist_GET() == (char)4786);
            assert(pack.nav_pitch_GET() == 1.2226394E38F);
            assert(pack.alt_error_GET() == 1.4181816E38F);
            assert(pack.aspd_error_GET() == 2.2606814E37F);
            assert(pack.target_bearing_GET() == (short)644);
            assert(pack.nav_roll_GET() == -2.6643858E38F);
            assert(pack.xtrack_error_GET() == 3.3549107E38F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.alt_error_SET(1.4181816E38F) ;
        p62.target_bearing_SET((short)644) ;
        p62.aspd_error_SET(2.2606814E37F) ;
        p62.nav_bearing_SET((short) -32438) ;
        p62.nav_roll_SET(-2.6643858E38F) ;
        p62.xtrack_error_SET(3.3549107E38F) ;
        p62.wp_dist_SET((char)4786) ;
        p62.nav_pitch_SET(1.2226394E38F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.4671883E38F, 2.3634656E38F, -2.2325114E38F, 7.7588233E37F, 2.6609935E38F, 1.1949523E38F, 1.5161304E37F, 2.505762E38F, -7.2257256E37F, 1.1045499E38F, -1.9150039E38F, 1.898052E38F, 1.5390284E37F, -3.110039E38F, -8.833607E37F, 1.8457388E38F, -3.0387723E38F, 9.059788E37F, -3.1726051E38F, -1.5130709E38F, 9.060472E37F, 2.2480026E37F, 7.5844524E37F, -1.7086141E38F, 1.1371826E38F, 7.1354526E37F, -1.6275982E38F, 2.5339983E38F, 3.3826692E38F, 2.4776756E38F, -1.567606E37F, -3.1816077E38F, -2.0767234E38F, -7.734071E37F, -3.0518621E38F, 1.0030412E38F}));
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(pack.vz_GET() == -3.2351652E38F);
            assert(pack.relative_alt_GET() == 184283918);
            assert(pack.lon_GET() == -642122930);
            assert(pack.lat_GET() == 694468443);
            assert(pack.vx_GET() == 7.561034E37F);
            assert(pack.vy_GET() == -3.1888416E38F);
            assert(pack.alt_GET() == 968804130);
            assert(pack.time_usec_GET() == 3434585961943715793L);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.covariance_SET(new float[] {-1.4671883E38F, 2.3634656E38F, -2.2325114E38F, 7.7588233E37F, 2.6609935E38F, 1.1949523E38F, 1.5161304E37F, 2.505762E38F, -7.2257256E37F, 1.1045499E38F, -1.9150039E38F, 1.898052E38F, 1.5390284E37F, -3.110039E38F, -8.833607E37F, 1.8457388E38F, -3.0387723E38F, 9.059788E37F, -3.1726051E38F, -1.5130709E38F, 9.060472E37F, 2.2480026E37F, 7.5844524E37F, -1.7086141E38F, 1.1371826E38F, 7.1354526E37F, -1.6275982E38F, 2.5339983E38F, 3.3826692E38F, 2.4776756E38F, -1.567606E37F, -3.1816077E38F, -2.0767234E38F, -7.734071E37F, -3.0518621E38F, 1.0030412E38F}, 0) ;
        p63.vx_SET(7.561034E37F) ;
        p63.vy_SET(-3.1888416E38F) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p63.vz_SET(-3.2351652E38F) ;
        p63.time_usec_SET(3434585961943715793L) ;
        p63.lon_SET(-642122930) ;
        p63.lat_SET(694468443) ;
        p63.relative_alt_SET(184283918) ;
        p63.alt_SET(968804130) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -2.2776497E38F);
            assert(pack.x_GET() == -2.5082315E38F);
            assert(pack.vz_GET() == 1.4775896E38F);
            assert(pack.vy_GET() == 2.1986332E37F);
            assert(pack.az_GET() == -2.8596366E38F);
            assert(pack.ay_GET() == 3.334914E38F);
            assert(pack.vx_GET() == -2.9088782E38F);
            assert(pack.ax_GET() == -1.814741E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.2740328E37F, -3.0408835E38F, -1.0494343E38F, 2.7529586E38F, 1.0338521E38F, 4.855155E37F, -1.7540936E38F, 1.0532894E38F, 1.7666603E38F, -2.7094443E38F, -1.4120023E38F, 3.0157909E38F, 1.9695459E38F, -2.553556E37F, 2.8925576E38F, 9.837091E37F, 2.531083E38F, -1.9445663E38F, -3.2506395E38F, -7.151608E36F, -1.0553151E38F, -9.4954E37F, 5.062585E37F, 2.492751E38F, -1.3120714E38F, 1.6108596E37F, -5.4330095E37F, 2.6883955E38F, 5.3341013E37F, 2.4941451E38F, 1.6084276E38F, -8.034327E37F, 1.2948067E37F, -2.3232003E38F, 1.9005914E38F, 2.3729896E36F, -2.0840575E38F, -2.1175164E38F, 6.3673044E36F, 2.2965185E36F, -1.1637373E38F, -3.0445724E38F, -3.3741975E38F, 1.8052953E38F, -2.197366E38F}));
            assert(pack.z_GET() == 2.6623026E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.time_usec_GET() == 4142127435781075575L);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.y_SET(-2.2776497E38F) ;
        p64.vz_SET(1.4775896E38F) ;
        p64.time_usec_SET(4142127435781075575L) ;
        p64.ay_SET(3.334914E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p64.vy_SET(2.1986332E37F) ;
        p64.x_SET(-2.5082315E38F) ;
        p64.az_SET(-2.8596366E38F) ;
        p64.vx_SET(-2.9088782E38F) ;
        p64.z_SET(2.6623026E38F) ;
        p64.covariance_SET(new float[] {-1.2740328E37F, -3.0408835E38F, -1.0494343E38F, 2.7529586E38F, 1.0338521E38F, 4.855155E37F, -1.7540936E38F, 1.0532894E38F, 1.7666603E38F, -2.7094443E38F, -1.4120023E38F, 3.0157909E38F, 1.9695459E38F, -2.553556E37F, 2.8925576E38F, 9.837091E37F, 2.531083E38F, -1.9445663E38F, -3.2506395E38F, -7.151608E36F, -1.0553151E38F, -9.4954E37F, 5.062585E37F, 2.492751E38F, -1.3120714E38F, 1.6108596E37F, -5.4330095E37F, 2.6883955E38F, 5.3341013E37F, 2.4941451E38F, 1.6084276E38F, -8.034327E37F, 1.2948067E37F, -2.3232003E38F, 1.9005914E38F, 2.3729896E36F, -2.0840575E38F, -2.1175164E38F, 6.3673044E36F, 2.2965185E36F, -1.1637373E38F, -3.0445724E38F, -3.3741975E38F, 1.8052953E38F, -2.197366E38F}, 0) ;
        p64.ax_SET(-1.814741E38F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chancount_GET() == (char)199);
            assert(pack.chan8_raw_GET() == (char)30880);
            assert(pack.chan14_raw_GET() == (char)40847);
            assert(pack.chan4_raw_GET() == (char)15429);
            assert(pack.rssi_GET() == (char)227);
            assert(pack.chan5_raw_GET() == (char)64460);
            assert(pack.chan10_raw_GET() == (char)46174);
            assert(pack.chan1_raw_GET() == (char)32883);
            assert(pack.chan18_raw_GET() == (char)63093);
            assert(pack.chan11_raw_GET() == (char)22791);
            assert(pack.chan13_raw_GET() == (char)57678);
            assert(pack.chan2_raw_GET() == (char)6311);
            assert(pack.chan17_raw_GET() == (char)49156);
            assert(pack.chan7_raw_GET() == (char)4367);
            assert(pack.chan16_raw_GET() == (char)30959);
            assert(pack.chan6_raw_GET() == (char)19801);
            assert(pack.chan9_raw_GET() == (char)38380);
            assert(pack.time_boot_ms_GET() == 2002299379L);
            assert(pack.chan15_raw_GET() == (char)52616);
            assert(pack.chan12_raw_GET() == (char)8561);
            assert(pack.chan3_raw_GET() == (char)30021);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan14_raw_SET((char)40847) ;
        p65.chan13_raw_SET((char)57678) ;
        p65.chancount_SET((char)199) ;
        p65.chan8_raw_SET((char)30880) ;
        p65.chan15_raw_SET((char)52616) ;
        p65.chan3_raw_SET((char)30021) ;
        p65.chan16_raw_SET((char)30959) ;
        p65.chan2_raw_SET((char)6311) ;
        p65.time_boot_ms_SET(2002299379L) ;
        p65.chan11_raw_SET((char)22791) ;
        p65.chan18_raw_SET((char)63093) ;
        p65.chan12_raw_SET((char)8561) ;
        p65.chan6_raw_SET((char)19801) ;
        p65.chan9_raw_SET((char)38380) ;
        p65.chan5_raw_SET((char)64460) ;
        p65.chan17_raw_SET((char)49156) ;
        p65.chan4_raw_SET((char)15429) ;
        p65.chan7_raw_SET((char)4367) ;
        p65.chan1_raw_SET((char)32883) ;
        p65.chan10_raw_SET((char)46174) ;
        p65.rssi_SET((char)227) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.start_stop_GET() == (char)198);
            assert(pack.req_message_rate_GET() == (char)21493);
            assert(pack.target_system_GET() == (char)53);
            assert(pack.target_component_GET() == (char)45);
            assert(pack.req_stream_id_GET() == (char)8);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_component_SET((char)45) ;
        p66.req_stream_id_SET((char)8) ;
        p66.start_stop_SET((char)198) ;
        p66.req_message_rate_SET((char)21493) ;
        p66.target_system_SET((char)53) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.stream_id_GET() == (char)37);
            assert(pack.message_rate_GET() == (char)7158);
            assert(pack.on_off_GET() == (char)40);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)7158) ;
        p67.stream_id_SET((char)37) ;
        p67.on_off_SET((char)40) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.buttons_GET() == (char)15477);
            assert(pack.y_GET() == (short) -24878);
            assert(pack.x_GET() == (short) -32097);
            assert(pack.z_GET() == (short)11356);
            assert(pack.target_GET() == (char)102);
            assert(pack.r_GET() == (short) -5425);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.target_SET((char)102) ;
        p69.r_SET((short) -5425) ;
        p69.z_SET((short)11356) ;
        p69.y_SET((short) -24878) ;
        p69.x_SET((short) -32097) ;
        p69.buttons_SET((char)15477) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan1_raw_GET() == (char)61700);
            assert(pack.chan7_raw_GET() == (char)60238);
            assert(pack.chan3_raw_GET() == (char)28806);
            assert(pack.chan5_raw_GET() == (char)21833);
            assert(pack.target_system_GET() == (char)216);
            assert(pack.chan6_raw_GET() == (char)30146);
            assert(pack.chan8_raw_GET() == (char)12951);
            assert(pack.chan4_raw_GET() == (char)50949);
            assert(pack.target_component_GET() == (char)221);
            assert(pack.chan2_raw_GET() == (char)53996);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.target_component_SET((char)221) ;
        p70.chan2_raw_SET((char)53996) ;
        p70.chan8_raw_SET((char)12951) ;
        p70.chan6_raw_SET((char)30146) ;
        p70.chan5_raw_SET((char)21833) ;
        p70.chan7_raw_SET((char)60238) ;
        p70.chan3_raw_SET((char)28806) ;
        p70.chan4_raw_SET((char)50949) ;
        p70.target_system_SET((char)216) ;
        p70.chan1_raw_SET((char)61700) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 1.2283346E38F);
            assert(pack.param2_GET() == 1.9132758E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.autocontinue_GET() == (char)25);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.target_system_GET() == (char)67);
            assert(pack.x_GET() == -518723739);
            assert(pack.param3_GET() == -2.0997622E38F);
            assert(pack.param1_GET() == -9.263889E37F);
            assert(pack.current_GET() == (char)78);
            assert(pack.y_GET() == -1280326964);
            assert(pack.target_component_GET() == (char)10);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT);
            assert(pack.param4_GET() == -3.0921811E38F);
            assert(pack.seq_GET() == (char)48508);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.target_system_SET((char)67) ;
        p73.current_SET((char)78) ;
        p73.target_component_SET((char)10) ;
        p73.command_SET(MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) ;
        p73.seq_SET((char)48508) ;
        p73.param4_SET(-3.0921811E38F) ;
        p73.param3_SET(-2.0997622E38F) ;
        p73.param2_SET(1.9132758E38F) ;
        p73.y_SET(-1280326964) ;
        p73.x_SET(-518723739) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p73.autocontinue_SET((char)25) ;
        p73.z_SET(1.2283346E38F) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p73.param1_SET(-9.263889E37F) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.airspeed_GET() == 7.995626E37F);
            assert(pack.climb_GET() == -5.2558087E37F);
            assert(pack.alt_GET() == -1.8353938E38F);
            assert(pack.throttle_GET() == (char)61643);
            assert(pack.groundspeed_GET() == -6.4633574E37F);
            assert(pack.heading_GET() == (short) -13313);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.throttle_SET((char)61643) ;
        p74.alt_SET(-1.8353938E38F) ;
        p74.heading_SET((short) -13313) ;
        p74.climb_SET(-5.2558087E37F) ;
        p74.airspeed_SET(7.995626E37F) ;
        p74.groundspeed_SET(-6.4633574E37F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.autocontinue_GET() == (char)148);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.target_component_GET() == (char)10);
            assert(pack.param3_GET() == 1.1970831E38F);
            assert(pack.x_GET() == 239254466);
            assert(pack.param1_GET() == -1.4369899E38F);
            assert(pack.z_GET() == -2.3286802E38F);
            assert(pack.y_GET() == -1343115461);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT);
            assert(pack.current_GET() == (char)87);
            assert(pack.param2_GET() == -1.4989627E38F);
            assert(pack.param4_GET() == 2.9246519E38F);
            assert(pack.target_system_GET() == (char)228);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.target_component_SET((char)10) ;
        p75.target_system_SET((char)228) ;
        p75.y_SET(-1343115461) ;
        p75.current_SET((char)87) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p75.param2_SET(-1.4989627E38F) ;
        p75.autocontinue_SET((char)148) ;
        p75.command_SET(MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT) ;
        p75.param3_SET(1.1970831E38F) ;
        p75.param4_SET(2.9246519E38F) ;
        p75.x_SET(239254466) ;
        p75.z_SET(-2.3286802E38F) ;
        p75.param1_SET(-1.4369899E38F) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param5_GET() == 1.6603095E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_USER_2);
            assert(pack.target_system_GET() == (char)109);
            assert(pack.target_component_GET() == (char)153);
            assert(pack.param3_GET() == -2.2999944E38F);
            assert(pack.param7_GET() == -1.0736331E38F);
            assert(pack.param2_GET() == -2.7291578E38F);
            assert(pack.confirmation_GET() == (char)62);
            assert(pack.param6_GET() == 1.3777161E38F);
            assert(pack.param1_GET() == 2.4190236E38F);
            assert(pack.param4_GET() == -2.895242E38F);
        });
        COMMAND_LONG p76 = new COMMAND_LONG();
        PH.setPack(p76);
        p76.target_system_SET((char)109) ;
        p76.command_SET(MAV_CMD.MAV_CMD_USER_2) ;
        p76.param3_SET(-2.2999944E38F) ;
        p76.param1_SET(2.4190236E38F) ;
        p76.target_component_SET((char)153) ;
        p76.param4_SET(-2.895242E38F) ;
        p76.param7_SET(-1.0736331E38F) ;
        p76.param6_SET(1.3777161E38F) ;
        p76.param5_SET(1.6603095E38F) ;
        p76.param2_SET(-2.7291578E38F) ;
        p76.confirmation_SET((char)62) ;
        TestChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.progress_TRY(ph) == (char)228);
            assert(pack.target_component_TRY(ph) == (char)235);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_SERVO);
            assert(pack.result_param2_TRY(ph) == -929335652);
            assert(pack.target_system_TRY(ph) == (char)192);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_IN_PROGRESS);
        });
        COMMAND_ACK p77 = new COMMAND_ACK();
        PH.setPack(p77);
        p77.result_param2_SET(-929335652, PH) ;
        p77.progress_SET((char)228, PH) ;
        p77.target_system_SET((char)192, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_IN_PROGRESS) ;
        p77.command_SET(MAV_CMD.MAV_CMD_DO_SET_SERVO) ;
        p77.target_component_SET((char)235, PH) ;
        TestChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -1.3230813E38F);
            assert(pack.manual_override_switch_GET() == (char)74);
            assert(pack.thrust_GET() == -1.3840955E38F);
            assert(pack.yaw_GET() == 8.713124E37F);
            assert(pack.pitch_GET() == 2.2678174E38F);
            assert(pack.time_boot_ms_GET() == 2145508612L);
            assert(pack.mode_switch_GET() == (char)96);
        });
        MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.mode_switch_SET((char)96) ;
        p81.thrust_SET(-1.3840955E38F) ;
        p81.roll_SET(-1.3230813E38F) ;
        p81.yaw_SET(8.713124E37F) ;
        p81.pitch_SET(2.2678174E38F) ;
        p81.manual_override_switch_SET((char)74) ;
        p81.time_boot_ms_SET(2145508612L) ;
        TestChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)219);
            assert(pack.thrust_GET() == 6.0307636E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.4634157E38F, -1.5212263E38F, -3.3894125E38F, -3.3959412E38F}));
            assert(pack.type_mask_GET() == (char)102);
            assert(pack.body_roll_rate_GET() == -2.5709066E38F);
            assert(pack.time_boot_ms_GET() == 248779137L);
            assert(pack.body_yaw_rate_GET() == -5.9583985E37F);
            assert(pack.body_pitch_rate_GET() == 3.707621E37F);
            assert(pack.target_system_GET() == (char)59);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.q_SET(new float[] {1.4634157E38F, -1.5212263E38F, -3.3894125E38F, -3.3959412E38F}, 0) ;
        p82.type_mask_SET((char)102) ;
        p82.target_component_SET((char)219) ;
        p82.body_roll_rate_SET(-2.5709066E38F) ;
        p82.target_system_SET((char)59) ;
        p82.body_pitch_rate_SET(3.707621E37F) ;
        p82.body_yaw_rate_SET(-5.9583985E37F) ;
        p82.time_boot_ms_SET(248779137L) ;
        p82.thrust_SET(6.0307636E37F) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.thrust_GET() == -3.397979E38F);
            assert(pack.time_boot_ms_GET() == 461312547L);
            assert(pack.body_pitch_rate_GET() == -2.4449331E38F);
            assert(pack.type_mask_GET() == (char)254);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.5719227E38F, 3.1623865E38F, 1.0350357E38F, -2.6865532E38F}));
            assert(pack.body_yaw_rate_GET() == -3.2076156E38F);
            assert(pack.body_roll_rate_GET() == -2.984695E38F);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.thrust_SET(-3.397979E38F) ;
        p83.body_pitch_rate_SET(-2.4449331E38F) ;
        p83.q_SET(new float[] {2.5719227E38F, 3.1623865E38F, 1.0350357E38F, -2.6865532E38F}, 0) ;
        p83.body_roll_rate_SET(-2.984695E38F) ;
        p83.body_yaw_rate_SET(-3.2076156E38F) ;
        p83.time_boot_ms_SET(461312547L) ;
        p83.type_mask_SET((char)254) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)143);
            assert(pack.yaw_GET() == 8.2102423E37F);
            assert(pack.type_mask_GET() == (char)5863);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.z_GET() == -2.573921E38F);
            assert(pack.yaw_rate_GET() == -1.2751263E38F);
            assert(pack.vz_GET() == -3.3184661E37F);
            assert(pack.target_component_GET() == (char)172);
            assert(pack.afy_GET() == 4.718389E37F);
            assert(pack.vx_GET() == 1.8147755E38F);
            assert(pack.x_GET() == -9.9208554E36F);
            assert(pack.vy_GET() == 1.9995046E38F);
            assert(pack.time_boot_ms_GET() == 1674882058L);
            assert(pack.y_GET() == 9.160904E37F);
            assert(pack.afz_GET() == 1.5396129E38F);
            assert(pack.afx_GET() == 2.492642E38F);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.afx_SET(2.492642E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p84.target_component_SET((char)172) ;
        p84.afy_SET(4.718389E37F) ;
        p84.yaw_SET(8.2102423E37F) ;
        p84.vx_SET(1.8147755E38F) ;
        p84.target_system_SET((char)143) ;
        p84.vy_SET(1.9995046E38F) ;
        p84.y_SET(9.160904E37F) ;
        p84.afz_SET(1.5396129E38F) ;
        p84.time_boot_ms_SET(1674882058L) ;
        p84.yaw_rate_SET(-1.2751263E38F) ;
        p84.type_mask_SET((char)5863) ;
        p84.x_SET(-9.9208554E36F) ;
        p84.vz_SET(-3.3184661E37F) ;
        p84.z_SET(-2.573921E38F) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == -1.7562973E38F);
            assert(pack.afz_GET() == -1.3925562E38F);
            assert(pack.yaw_GET() == -2.992612E38F);
            assert(pack.alt_GET() == 2.430686E38F);
            assert(pack.afx_GET() == -3.3185237E38F);
            assert(pack.target_system_GET() == (char)222);
            assert(pack.lon_int_GET() == 346259357);
            assert(pack.lat_int_GET() == 313996645);
            assert(pack.vy_GET() == -2.2941608E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.yaw_rate_GET() == 3.1200804E38F);
            assert(pack.vz_GET() == 5.887305E37F);
            assert(pack.target_component_GET() == (char)202);
            assert(pack.time_boot_ms_GET() == 841182960L);
            assert(pack.afy_GET() == 3.0639427E37F);
            assert(pack.type_mask_GET() == (char)49792);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.vy_SET(-2.2941608E38F) ;
        p86.afx_SET(-3.3185237E38F) ;
        p86.type_mask_SET((char)49792) ;
        p86.afz_SET(-1.3925562E38F) ;
        p86.time_boot_ms_SET(841182960L) ;
        p86.alt_SET(2.430686E38F) ;
        p86.lon_int_SET(346259357) ;
        p86.yaw_rate_SET(3.1200804E38F) ;
        p86.vz_SET(5.887305E37F) ;
        p86.yaw_SET(-2.992612E38F) ;
        p86.target_component_SET((char)202) ;
        p86.lat_int_SET(313996645) ;
        p86.afy_SET(3.0639427E37F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p86.vx_SET(-1.7562973E38F) ;
        p86.target_system_SET((char)222) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 1.7367598E38F);
            assert(pack.afx_GET() == -1.8099138E38F);
            assert(pack.time_boot_ms_GET() == 3270967043L);
            assert(pack.vx_GET() == -3.3861884E38F);
            assert(pack.vy_GET() == -3.0906858E37F);
            assert(pack.alt_GET() == 2.2304588E38F);
            assert(pack.lon_int_GET() == 1456238166);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.afz_GET() == 2.204088E37F);
            assert(pack.lat_int_GET() == -1750459695);
            assert(pack.afy_GET() == -2.7435734E38F);
            assert(pack.vz_GET() == -1.7546306E38F);
            assert(pack.yaw_rate_GET() == 2.5800128E38F);
            assert(pack.type_mask_GET() == (char)25004);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.afz_SET(2.204088E37F) ;
        p87.vz_SET(-1.7546306E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p87.yaw_SET(1.7367598E38F) ;
        p87.alt_SET(2.2304588E38F) ;
        p87.lat_int_SET(-1750459695) ;
        p87.afx_SET(-1.8099138E38F) ;
        p87.lon_int_SET(1456238166) ;
        p87.type_mask_SET((char)25004) ;
        p87.vx_SET(-3.3861884E38F) ;
        p87.afy_SET(-2.7435734E38F) ;
        p87.vy_SET(-3.0906858E37F) ;
        p87.time_boot_ms_SET(3270967043L) ;
        p87.yaw_rate_SET(2.5800128E38F) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 2.7918435E37F);
            assert(pack.z_GET() == -2.900731E38F);
            assert(pack.y_GET() == 1.0409614E38F);
            assert(pack.yaw_GET() == 1.6374992E38F);
            assert(pack.time_boot_ms_GET() == 1198005171L);
            assert(pack.x_GET() == -5.0575446E37F);
            assert(pack.pitch_GET() == -2.887195E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.yaw_SET(1.6374992E38F) ;
        p89.pitch_SET(-2.887195E38F) ;
        p89.x_SET(-5.0575446E37F) ;
        p89.z_SET(-2.900731E38F) ;
        p89.y_SET(1.0409614E38F) ;
        p89.roll_SET(2.7918435E37F) ;
        p89.time_boot_ms_SET(1198005171L) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 1423605219);
            assert(pack.vz_GET() == (short) -16429);
            assert(pack.yaw_GET() == -7.183133E37F);
            assert(pack.lon_GET() == 783372862);
            assert(pack.time_usec_GET() == 2885855015131962423L);
            assert(pack.xacc_GET() == (short) -24378);
            assert(pack.roll_GET() == -3.1924924E38F);
            assert(pack.pitch_GET() == -2.9396704E38F);
            assert(pack.vy_GET() == (short)20643);
            assert(pack.rollspeed_GET() == 7.9601273E37F);
            assert(pack.yacc_GET() == (short) -28241);
            assert(pack.zacc_GET() == (short)31347);
            assert(pack.yawspeed_GET() == -1.5442923E38F);
            assert(pack.pitchspeed_GET() == -1.0955023E38F);
            assert(pack.lat_GET() == 870037932);
            assert(pack.vx_GET() == (short)13787);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.vz_SET((short) -16429) ;
        p90.alt_SET(1423605219) ;
        p90.pitchspeed_SET(-1.0955023E38F) ;
        p90.yacc_SET((short) -28241) ;
        p90.rollspeed_SET(7.9601273E37F) ;
        p90.lon_SET(783372862) ;
        p90.vx_SET((short)13787) ;
        p90.zacc_SET((short)31347) ;
        p90.lat_SET(870037932) ;
        p90.yaw_SET(-7.183133E37F) ;
        p90.xacc_SET((short) -24378) ;
        p90.yawspeed_SET(-1.5442923E38F) ;
        p90.vy_SET((short)20643) ;
        p90.pitch_SET(-2.9396704E38F) ;
        p90.roll_SET(-3.1924924E38F) ;
        p90.time_usec_SET(2885855015131962423L) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.throttle_GET() == 2.2555206E37F);
            assert(pack.aux2_GET() == -6.2785345E37F);
            assert(pack.roll_ailerons_GET() == 1.3592563E38F);
            assert(pack.yaw_rudder_GET() == -2.94954E38F);
            assert(pack.time_usec_GET() == 6687643420471791436L);
            assert(pack.aux3_GET() == 1.0955077E38F);
            assert(pack.aux4_GET() == 1.9212774E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
            assert(pack.pitch_elevator_GET() == -3.0483665E38F);
            assert(pack.nav_mode_GET() == (char)125);
            assert(pack.aux1_GET() == -5.922649E37F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.nav_mode_SET((char)125) ;
        p91.aux1_SET(-5.922649E37F) ;
        p91.pitch_elevator_SET(-3.0483665E38F) ;
        p91.time_usec_SET(6687643420471791436L) ;
        p91.aux4_SET(1.9212774E38F) ;
        p91.aux2_SET(-6.2785345E37F) ;
        p91.aux3_SET(1.0955077E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED) ;
        p91.yaw_rudder_SET(-2.94954E38F) ;
        p91.throttle_SET(2.2555206E37F) ;
        p91.roll_ailerons_SET(1.3592563E38F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan2_raw_GET() == (char)60724);
            assert(pack.chan7_raw_GET() == (char)56348);
            assert(pack.rssi_GET() == (char)102);
            assert(pack.chan5_raw_GET() == (char)16017);
            assert(pack.time_usec_GET() == 5724627307906063682L);
            assert(pack.chan4_raw_GET() == (char)62691);
            assert(pack.chan6_raw_GET() == (char)4128);
            assert(pack.chan10_raw_GET() == (char)18851);
            assert(pack.chan12_raw_GET() == (char)42363);
            assert(pack.chan11_raw_GET() == (char)33798);
            assert(pack.chan1_raw_GET() == (char)13043);
            assert(pack.chan9_raw_GET() == (char)15729);
            assert(pack.chan3_raw_GET() == (char)52393);
            assert(pack.chan8_raw_GET() == (char)6689);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan12_raw_SET((char)42363) ;
        p92.chan9_raw_SET((char)15729) ;
        p92.chan5_raw_SET((char)16017) ;
        p92.chan3_raw_SET((char)52393) ;
        p92.chan6_raw_SET((char)4128) ;
        p92.rssi_SET((char)102) ;
        p92.time_usec_SET(5724627307906063682L) ;
        p92.chan7_raw_SET((char)56348) ;
        p92.chan2_raw_SET((char)60724) ;
        p92.chan10_raw_SET((char)18851) ;
        p92.chan8_raw_SET((char)6689) ;
        p92.chan4_raw_SET((char)62691) ;
        p92.chan11_raw_SET((char)33798) ;
        p92.chan1_raw_SET((char)13043) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
            assert(pack.time_usec_GET() == 601947628886713589L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.9373044E37F, -2.6457902E38F, -6.650357E37F, -2.1480634E37F, 1.6036098E37F, 2.7993263E38F, -3.3983265E38F, 1.1810822E38F, 2.8531128E38F, 2.3723422E36F, 5.836989E37F, -4.5257844E37F, -9.869409E37F, 1.2177025E38F, 3.1183495E38F, 1.3326377E38F}));
            assert(pack.flags_GET() == 3831603541057844858L);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED) ;
        p93.flags_SET(3831603541057844858L) ;
        p93.time_usec_SET(601947628886713589L) ;
        p93.controls_SET(new float[] {1.9373044E37F, -2.6457902E38F, -6.650357E37F, -2.1480634E37F, 1.6036098E37F, 2.7993263E38F, -3.3983265E38F, 1.1810822E38F, 2.8531128E38F, 2.3723422E36F, 5.836989E37F, -4.5257844E37F, -9.869409E37F, 1.2177025E38F, 3.1183495E38F, 1.3326377E38F}, 0) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_rate_x_TRY(ph) == -6.0927096E37F);
            assert(pack.sensor_id_GET() == (char)134);
            assert(pack.quality_GET() == (char)71);
            assert(pack.flow_comp_m_x_GET() == -2.7620295E38F);
            assert(pack.flow_rate_y_TRY(ph) == 2.19234E38F);
            assert(pack.flow_x_GET() == (short)25281);
            assert(pack.flow_y_GET() == (short) -20056);
            assert(pack.time_usec_GET() == 8854566384743552052L);
            assert(pack.ground_distance_GET() == 1.7670293E38F);
            assert(pack.flow_comp_m_y_GET() == 1.1425425E38F);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_y_SET((short) -20056) ;
        p100.sensor_id_SET((char)134) ;
        p100.ground_distance_SET(1.7670293E38F) ;
        p100.flow_x_SET((short)25281) ;
        p100.flow_comp_m_y_SET(1.1425425E38F) ;
        p100.flow_rate_y_SET(2.19234E38F, PH) ;
        p100.quality_SET((char)71) ;
        p100.time_usec_SET(8854566384743552052L) ;
        p100.flow_comp_m_x_SET(-2.7620295E38F) ;
        p100.flow_rate_x_SET(-6.0927096E37F, PH) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 8.1478004E37F);
            assert(pack.z_GET() == -2.314765E38F);
            assert(pack.roll_GET() == -4.0532053E37F);
            assert(pack.pitch_GET() == 3.1466392E38F);
            assert(pack.usec_GET() == 8116261456861631224L);
            assert(pack.y_GET() == -3.118969E37F);
            assert(pack.yaw_GET() == -2.0085064E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.x_SET(8.1478004E37F) ;
        p101.y_SET(-3.118969E37F) ;
        p101.usec_SET(8116261456861631224L) ;
        p101.yaw_SET(-2.0085064E38F) ;
        p101.pitch_SET(3.1466392E38F) ;
        p101.z_SET(-2.314765E38F) ;
        p101.roll_SET(-4.0532053E37F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -8.70488E37F);
            assert(pack.roll_GET() == -2.971173E37F);
            assert(pack.y_GET() == 2.5433448E38F);
            assert(pack.usec_GET() == 2871868030168580564L);
            assert(pack.yaw_GET() == 7.1436406E37F);
            assert(pack.z_GET() == -4.953595E36F);
            assert(pack.pitch_GET() == 1.3805739E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.y_SET(2.5433448E38F) ;
        p102.usec_SET(2871868030168580564L) ;
        p102.roll_SET(-2.971173E37F) ;
        p102.yaw_SET(7.1436406E37F) ;
        p102.pitch_SET(1.3805739E38F) ;
        p102.z_SET(-4.953595E36F) ;
        p102.x_SET(-8.70488E37F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -2.4628715E38F);
            assert(pack.y_GET() == 9.1391783E36F);
            assert(pack.usec_GET() == 8850755573655623246L);
            assert(pack.z_GET() == -1.7038678E38F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(8850755573655623246L) ;
        p103.y_SET(9.1391783E36F) ;
        p103.z_SET(-1.7038678E38F) ;
        p103.x_SET(-2.4628715E38F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -1.1766951E38F);
            assert(pack.z_GET() == -7.903844E37F);
            assert(pack.pitch_GET() == 2.0983773E38F);
            assert(pack.x_GET() == 9.410792E37F);
            assert(pack.y_GET() == -5.0232E37F);
            assert(pack.yaw_GET() == 1.4486053E38F);
            assert(pack.usec_GET() == 5242463063741249936L);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(5242463063741249936L) ;
        p104.z_SET(-7.903844E37F) ;
        p104.pitch_SET(2.0983773E38F) ;
        p104.roll_SET(-1.1766951E38F) ;
        p104.y_SET(-5.0232E37F) ;
        p104.x_SET(9.410792E37F) ;
        p104.yaw_SET(1.4486053E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.pressure_alt_GET() == 1.7698347E38F);
            assert(pack.abs_pressure_GET() == -2.5354837E38F);
            assert(pack.diff_pressure_GET() == -3.794411E37F);
            assert(pack.time_usec_GET() == 1535255330826677246L);
            assert(pack.temperature_GET() == -2.9724995E38F);
            assert(pack.yacc_GET() == -1.2118488E38F);
            assert(pack.ymag_GET() == -1.3126578E38F);
            assert(pack.zgyro_GET() == 5.7065584E37F);
            assert(pack.zacc_GET() == -4.740545E37F);
            assert(pack.xacc_GET() == 2.0813882E38F);
            assert(pack.ygyro_GET() == 2.5896875E38F);
            assert(pack.zmag_GET() == -2.3907793E38F);
            assert(pack.xmag_GET() == -1.4850277E38F);
            assert(pack.xgyro_GET() == -2.2527433E38F);
            assert(pack.fields_updated_GET() == (char)48229);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.xacc_SET(2.0813882E38F) ;
        p105.ymag_SET(-1.3126578E38F) ;
        p105.zgyro_SET(5.7065584E37F) ;
        p105.xmag_SET(-1.4850277E38F) ;
        p105.xgyro_SET(-2.2527433E38F) ;
        p105.fields_updated_SET((char)48229) ;
        p105.time_usec_SET(1535255330826677246L) ;
        p105.ygyro_SET(2.5896875E38F) ;
        p105.yacc_SET(-1.2118488E38F) ;
        p105.pressure_alt_SET(1.7698347E38F) ;
        p105.zmag_SET(-2.3907793E38F) ;
        p105.diff_pressure_SET(-3.794411E37F) ;
        p105.zacc_SET(-4.740545E37F) ;
        p105.temperature_SET(-2.9724995E38F) ;
        p105.abs_pressure_SET(-2.5354837E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integration_time_us_GET() == 1015964171L);
            assert(pack.time_usec_GET() == 3463803512905756749L);
            assert(pack.quality_GET() == (char)100);
            assert(pack.integrated_x_GET() == 1.5473691E37F);
            assert(pack.integrated_zgyro_GET() == 2.9940585E38F);
            assert(pack.temperature_GET() == (short) -5141);
            assert(pack.distance_GET() == 1.8761708E38F);
            assert(pack.integrated_xgyro_GET() == -3.020299E38F);
            assert(pack.sensor_id_GET() == (char)20);
            assert(pack.time_delta_distance_us_GET() == 1898945808L);
            assert(pack.integrated_y_GET() == -3.0921594E38F);
            assert(pack.integrated_ygyro_GET() == 3.1376054E38F);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_x_SET(1.5473691E37F) ;
        p106.distance_SET(1.8761708E38F) ;
        p106.time_delta_distance_us_SET(1898945808L) ;
        p106.time_usec_SET(3463803512905756749L) ;
        p106.quality_SET((char)100) ;
        p106.integrated_y_SET(-3.0921594E38F) ;
        p106.integrated_zgyro_SET(2.9940585E38F) ;
        p106.integrated_ygyro_SET(3.1376054E38F) ;
        p106.integration_time_us_SET(1015964171L) ;
        p106.sensor_id_SET((char)20) ;
        p106.integrated_xgyro_SET(-3.020299E38F) ;
        p106.temperature_SET((short) -5141) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.pressure_alt_GET() == -2.3845873E38F);
            assert(pack.abs_pressure_GET() == -1.6640906E37F);
            assert(pack.xacc_GET() == -1.7919128E38F);
            assert(pack.ygyro_GET() == -9.926124E36F);
            assert(pack.zgyro_GET() == -1.3334678E38F);
            assert(pack.xmag_GET() == 2.3275994E38F);
            assert(pack.xgyro_GET() == 9.544443E37F);
            assert(pack.diff_pressure_GET() == -1.3294223E38F);
            assert(pack.ymag_GET() == -7.1704433E37F);
            assert(pack.fields_updated_GET() == 4225228666L);
            assert(pack.temperature_GET() == -2.7073244E38F);
            assert(pack.yacc_GET() == -3.6192677E37F);
            assert(pack.time_usec_GET() == 2837641774507746965L);
            assert(pack.zmag_GET() == -2.8413924E38F);
            assert(pack.zacc_GET() == -2.0098278E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.xacc_SET(-1.7919128E38F) ;
        p107.temperature_SET(-2.7073244E38F) ;
        p107.fields_updated_SET(4225228666L) ;
        p107.ygyro_SET(-9.926124E36F) ;
        p107.ymag_SET(-7.1704433E37F) ;
        p107.zgyro_SET(-1.3334678E38F) ;
        p107.diff_pressure_SET(-1.3294223E38F) ;
        p107.time_usec_SET(2837641774507746965L) ;
        p107.xmag_SET(2.3275994E38F) ;
        p107.zacc_SET(-2.0098278E38F) ;
        p107.pressure_alt_SET(-2.3845873E38F) ;
        p107.yacc_SET(-3.6192677E37F) ;
        p107.zmag_SET(-2.8413924E38F) ;
        p107.abs_pressure_SET(-1.6640906E37F) ;
        p107.xgyro_SET(9.544443E37F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.std_dev_horz_GET() == -4.1651617E37F);
            assert(pack.alt_GET() == -1.1877891E38F);
            assert(pack.vn_GET() == 2.7818828E38F);
            assert(pack.q4_GET() == 5.697303E37F);
            assert(pack.q3_GET() == 4.551711E37F);
            assert(pack.yaw_GET() == 9.017193E37F);
            assert(pack.xacc_GET() == -2.9360903E38F);
            assert(pack.pitch_GET() == -2.6811644E37F);
            assert(pack.zacc_GET() == 1.016223E38F);
            assert(pack.ygyro_GET() == 1.7764265E38F);
            assert(pack.lon_GET() == -1.7652815E38F);
            assert(pack.zgyro_GET() == 1.4337906E38F);
            assert(pack.xgyro_GET() == 9.448713E37F);
            assert(pack.vd_GET() == -3.1130576E37F);
            assert(pack.std_dev_vert_GET() == -1.141129E38F);
            assert(pack.q1_GET() == 3.1320809E38F);
            assert(pack.q2_GET() == -2.8964368E38F);
            assert(pack.roll_GET() == -3.1674252E38F);
            assert(pack.yacc_GET() == 1.6176047E38F);
            assert(pack.lat_GET() == 2.65444E38F);
            assert(pack.ve_GET() == 8.775176E37F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.xgyro_SET(9.448713E37F) ;
        p108.ve_SET(8.775176E37F) ;
        p108.q4_SET(5.697303E37F) ;
        p108.pitch_SET(-2.6811644E37F) ;
        p108.lat_SET(2.65444E38F) ;
        p108.q3_SET(4.551711E37F) ;
        p108.q2_SET(-2.8964368E38F) ;
        p108.std_dev_vert_SET(-1.141129E38F) ;
        p108.xacc_SET(-2.9360903E38F) ;
        p108.q1_SET(3.1320809E38F) ;
        p108.vd_SET(-3.1130576E37F) ;
        p108.yacc_SET(1.6176047E38F) ;
        p108.vn_SET(2.7818828E38F) ;
        p108.roll_SET(-3.1674252E38F) ;
        p108.std_dev_horz_SET(-4.1651617E37F) ;
        p108.alt_SET(-1.1877891E38F) ;
        p108.lon_SET(-1.7652815E38F) ;
        p108.ygyro_SET(1.7764265E38F) ;
        p108.zgyro_SET(1.4337906E38F) ;
        p108.yaw_SET(9.017193E37F) ;
        p108.zacc_SET(1.016223E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.noise_GET() == (char)99);
            assert(pack.fixed__GET() == (char)56165);
            assert(pack.remnoise_GET() == (char)182);
            assert(pack.rssi_GET() == (char)49);
            assert(pack.txbuf_GET() == (char)16);
            assert(pack.rxerrors_GET() == (char)62768);
            assert(pack.remrssi_GET() == (char)121);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)49) ;
        p109.remrssi_SET((char)121) ;
        p109.rxerrors_SET((char)62768) ;
        p109.fixed__SET((char)56165) ;
        p109.noise_SET((char)99) ;
        p109.txbuf_SET((char)16) ;
        p109.remnoise_SET((char)182) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)50);
            assert(pack.target_system_GET() == (char)230);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)233, (char)254, (char)235, (char)98, (char)10, (char)63, (char)218, (char)231, (char)99, (char)128, (char)93, (char)159, (char)214, (char)202, (char)22, (char)147, (char)106, (char)171, (char)69, (char)235, (char)148, (char)237, (char)229, (char)91, (char)183, (char)46, (char)199, (char)151, (char)212, (char)194, (char)125, (char)37, (char)171, (char)71, (char)69, (char)210, (char)125, (char)144, (char)60, (char)141, (char)199, (char)181, (char)30, (char)52, (char)138, (char)70, (char)23, (char)88, (char)112, (char)212, (char)111, (char)53, (char)36, (char)49, (char)244, (char)225, (char)251, (char)96, (char)197, (char)149, (char)41, (char)203, (char)202, (char)231, (char)107, (char)33, (char)127, (char)149, (char)17, (char)2, (char)244, (char)160, (char)92, (char)47, (char)151, (char)235, (char)241, (char)113, (char)214, (char)232, (char)239, (char)81, (char)56, (char)137, (char)56, (char)41, (char)114, (char)249, (char)249, (char)152, (char)166, (char)52, (char)33, (char)75, (char)83, (char)15, (char)97, (char)191, (char)156, (char)100, (char)120, (char)91, (char)129, (char)248, (char)216, (char)207, (char)237, (char)58, (char)126, (char)178, (char)122, (char)4, (char)138, (char)79, (char)242, (char)248, (char)82, (char)51, (char)113, (char)2, (char)72, (char)213, (char)214, (char)13, (char)172, (char)188, (char)161, (char)239, (char)87, (char)96, (char)49, (char)9, (char)21, (char)78, (char)247, (char)30, (char)239, (char)223, (char)81, (char)50, (char)228, (char)9, (char)56, (char)111, (char)220, (char)38, (char)202, (char)212, (char)29, (char)57, (char)82, (char)119, (char)25, (char)5, (char)8, (char)195, (char)251, (char)254, (char)210, (char)82, (char)69, (char)68, (char)161, (char)191, (char)42, (char)174, (char)200, (char)53, (char)208, (char)164, (char)181, (char)0, (char)183, (char)69, (char)197, (char)174, (char)189, (char)67, (char)174, (char)133, (char)201, (char)51, (char)114, (char)182, (char)39, (char)162, (char)29, (char)43, (char)31, (char)190, (char)120, (char)178, (char)174, (char)124, (char)238, (char)100, (char)51, (char)50, (char)183, (char)123, (char)116, (char)7, (char)144, (char)104, (char)121, (char)146, (char)146, (char)161, (char)83, (char)107, (char)42, (char)66, (char)23, (char)217, (char)15, (char)250, (char)59, (char)138, (char)175, (char)211, (char)13, (char)241, (char)246, (char)210, (char)236, (char)165, (char)66, (char)87, (char)154, (char)125, (char)62, (char)69, (char)57, (char)215, (char)139, (char)245, (char)30, (char)168, (char)26, (char)103, (char)229, (char)69, (char)97, (char)115, (char)82, (char)60, (char)236, (char)128, (char)147, (char)74, (char)251}));
            assert(pack.target_network_GET() == (char)232);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.payload_SET(new char[] {(char)233, (char)254, (char)235, (char)98, (char)10, (char)63, (char)218, (char)231, (char)99, (char)128, (char)93, (char)159, (char)214, (char)202, (char)22, (char)147, (char)106, (char)171, (char)69, (char)235, (char)148, (char)237, (char)229, (char)91, (char)183, (char)46, (char)199, (char)151, (char)212, (char)194, (char)125, (char)37, (char)171, (char)71, (char)69, (char)210, (char)125, (char)144, (char)60, (char)141, (char)199, (char)181, (char)30, (char)52, (char)138, (char)70, (char)23, (char)88, (char)112, (char)212, (char)111, (char)53, (char)36, (char)49, (char)244, (char)225, (char)251, (char)96, (char)197, (char)149, (char)41, (char)203, (char)202, (char)231, (char)107, (char)33, (char)127, (char)149, (char)17, (char)2, (char)244, (char)160, (char)92, (char)47, (char)151, (char)235, (char)241, (char)113, (char)214, (char)232, (char)239, (char)81, (char)56, (char)137, (char)56, (char)41, (char)114, (char)249, (char)249, (char)152, (char)166, (char)52, (char)33, (char)75, (char)83, (char)15, (char)97, (char)191, (char)156, (char)100, (char)120, (char)91, (char)129, (char)248, (char)216, (char)207, (char)237, (char)58, (char)126, (char)178, (char)122, (char)4, (char)138, (char)79, (char)242, (char)248, (char)82, (char)51, (char)113, (char)2, (char)72, (char)213, (char)214, (char)13, (char)172, (char)188, (char)161, (char)239, (char)87, (char)96, (char)49, (char)9, (char)21, (char)78, (char)247, (char)30, (char)239, (char)223, (char)81, (char)50, (char)228, (char)9, (char)56, (char)111, (char)220, (char)38, (char)202, (char)212, (char)29, (char)57, (char)82, (char)119, (char)25, (char)5, (char)8, (char)195, (char)251, (char)254, (char)210, (char)82, (char)69, (char)68, (char)161, (char)191, (char)42, (char)174, (char)200, (char)53, (char)208, (char)164, (char)181, (char)0, (char)183, (char)69, (char)197, (char)174, (char)189, (char)67, (char)174, (char)133, (char)201, (char)51, (char)114, (char)182, (char)39, (char)162, (char)29, (char)43, (char)31, (char)190, (char)120, (char)178, (char)174, (char)124, (char)238, (char)100, (char)51, (char)50, (char)183, (char)123, (char)116, (char)7, (char)144, (char)104, (char)121, (char)146, (char)146, (char)161, (char)83, (char)107, (char)42, (char)66, (char)23, (char)217, (char)15, (char)250, (char)59, (char)138, (char)175, (char)211, (char)13, (char)241, (char)246, (char)210, (char)236, (char)165, (char)66, (char)87, (char)154, (char)125, (char)62, (char)69, (char)57, (char)215, (char)139, (char)245, (char)30, (char)168, (char)26, (char)103, (char)229, (char)69, (char)97, (char)115, (char)82, (char)60, (char)236, (char)128, (char)147, (char)74, (char)251}, 0) ;
        p110.target_network_SET((char)232) ;
        p110.target_system_SET((char)230) ;
        p110.target_component_SET((char)50) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == 8483341175002499223L);
            assert(pack.tc1_GET() == -8119955359047593224L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(8483341175002499223L) ;
        p111.tc1_SET(-8119955359047593224L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 1300366236831659781L);
            assert(pack.seq_GET() == 321456649L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(321456649L) ;
        p112.time_usec_SET(1300366236831659781L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.eph_GET() == (char)14598);
            assert(pack.lat_GET() == 1039121166);
            assert(pack.satellites_visible_GET() == (char)135);
            assert(pack.lon_GET() == 1718014900);
            assert(pack.vd_GET() == (short)4787);
            assert(pack.vn_GET() == (short)2769);
            assert(pack.fix_type_GET() == (char)158);
            assert(pack.vel_GET() == (char)4506);
            assert(pack.cog_GET() == (char)47053);
            assert(pack.ve_GET() == (short) -32311);
            assert(pack.alt_GET() == -1793429825);
            assert(pack.time_usec_GET() == 1041986179474143466L);
            assert(pack.epv_GET() == (char)54030);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.eph_SET((char)14598) ;
        p113.satellites_visible_SET((char)135) ;
        p113.vd_SET((short)4787) ;
        p113.vn_SET((short)2769) ;
        p113.ve_SET((short) -32311) ;
        p113.time_usec_SET(1041986179474143466L) ;
        p113.alt_SET(-1793429825) ;
        p113.lat_SET(1039121166) ;
        p113.fix_type_SET((char)158) ;
        p113.lon_SET(1718014900) ;
        p113.vel_SET((char)4506) ;
        p113.cog_SET((char)47053) ;
        p113.epv_SET((char)54030) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.distance_GET() == 8.447265E37F);
            assert(pack.integration_time_us_GET() == 1541844104L);
            assert(pack.quality_GET() == (char)77);
            assert(pack.integrated_ygyro_GET() == 1.0177342E38F);
            assert(pack.integrated_xgyro_GET() == -1.9515402E38F);
            assert(pack.time_delta_distance_us_GET() == 1252119045L);
            assert(pack.integrated_zgyro_GET() == 1.599136E37F);
            assert(pack.time_usec_GET() == 2009255367638033014L);
            assert(pack.sensor_id_GET() == (char)225);
            assert(pack.integrated_x_GET() == 3.2578939E38F);
            assert(pack.integrated_y_GET() == 3.1604132E38F);
            assert(pack.temperature_GET() == (short) -31463);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.sensor_id_SET((char)225) ;
        p114.integrated_ygyro_SET(1.0177342E38F) ;
        p114.distance_SET(8.447265E37F) ;
        p114.integration_time_us_SET(1541844104L) ;
        p114.temperature_SET((short) -31463) ;
        p114.integrated_x_SET(3.2578939E38F) ;
        p114.quality_SET((char)77) ;
        p114.time_usec_SET(2009255367638033014L) ;
        p114.integrated_xgyro_SET(-1.9515402E38F) ;
        p114.integrated_y_SET(3.1604132E38F) ;
        p114.time_delta_distance_us_SET(1252119045L) ;
        p114.integrated_zgyro_SET(1.599136E37F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short) -8738);
            assert(pack.time_usec_GET() == 5863422833434944661L);
            assert(pack.yacc_GET() == (short) -21658);
            assert(pack.alt_GET() == -1408355535);
            assert(pack.vy_GET() == (short) -5630);
            assert(pack.lon_GET() == 383890005);
            assert(pack.pitchspeed_GET() == 9.984086E37F);
            assert(pack.lat_GET() == -71291055);
            assert(pack.true_airspeed_GET() == (char)63792);
            assert(pack.ind_airspeed_GET() == (char)25070);
            assert(pack.zacc_GET() == (short) -802);
            assert(pack.rollspeed_GET() == -4.5060564E36F);
            assert(pack.vx_GET() == (short)31094);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {2.0757614E38F, 2.0545343E38F, 7.8696277E37F, 1.7483715E38F}));
            assert(pack.yawspeed_GET() == 2.7097654E38F);
            assert(pack.vz_GET() == (short)24531);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.yacc_SET((short) -21658) ;
        p115.vx_SET((short)31094) ;
        p115.ind_airspeed_SET((char)25070) ;
        p115.xacc_SET((short) -8738) ;
        p115.zacc_SET((short) -802) ;
        p115.lat_SET(-71291055) ;
        p115.time_usec_SET(5863422833434944661L) ;
        p115.lon_SET(383890005) ;
        p115.alt_SET(-1408355535) ;
        p115.vz_SET((short)24531) ;
        p115.rollspeed_SET(-4.5060564E36F) ;
        p115.pitchspeed_SET(9.984086E37F) ;
        p115.true_airspeed_SET((char)63792) ;
        p115.attitude_quaternion_SET(new float[] {2.0757614E38F, 2.0545343E38F, 7.8696277E37F, 1.7483715E38F}, 0) ;
        p115.vy_SET((short) -5630) ;
        p115.yawspeed_SET(2.7097654E38F) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short)30966);
            assert(pack.xgyro_GET() == (short)1718);
            assert(pack.zmag_GET() == (short) -16992);
            assert(pack.zgyro_GET() == (short) -25653);
            assert(pack.xmag_GET() == (short)14941);
            assert(pack.zacc_GET() == (short) -19118);
            assert(pack.ymag_GET() == (short) -10215);
            assert(pack.xacc_GET() == (short) -18559);
            assert(pack.ygyro_GET() == (short) -28199);
            assert(pack.time_boot_ms_GET() == 964997539L);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.zacc_SET((short) -19118) ;
        p116.time_boot_ms_SET(964997539L) ;
        p116.xmag_SET((short)14941) ;
        p116.xacc_SET((short) -18559) ;
        p116.ymag_SET((short) -10215) ;
        p116.yacc_SET((short)30966) ;
        p116.xgyro_SET((short)1718) ;
        p116.zgyro_SET((short) -25653) ;
        p116.zmag_SET((short) -16992) ;
        p116.ygyro_SET((short) -28199) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_GET() == (char)23655);
            assert(pack.target_component_GET() == (char)41);
            assert(pack.start_GET() == (char)23659);
            assert(pack.target_system_GET() == (char)152);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_component_SET((char)41) ;
        p117.target_system_SET((char)152) ;
        p117.start_SET((char)23659) ;
        p117.end_SET((char)23655) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)9890);
            assert(pack.time_utc_GET() == 3352729130L);
            assert(pack.num_logs_GET() == (char)47577);
            assert(pack.last_log_num_GET() == (char)60934);
            assert(pack.size_GET() == 193249879L);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.last_log_num_SET((char)60934) ;
        p118.id_SET((char)9890) ;
        p118.num_logs_SET((char)47577) ;
        p118.size_SET(193249879L) ;
        p118.time_utc_SET(3352729130L) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)42120);
            assert(pack.count_GET() == 3404081639L);
            assert(pack.target_component_GET() == (char)132);
            assert(pack.target_system_GET() == (char)140);
            assert(pack.ofs_GET() == 318870425L);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.ofs_SET(318870425L) ;
        p119.id_SET((char)42120) ;
        p119.count_SET(3404081639L) ;
        p119.target_component_SET((char)132) ;
        p119.target_system_SET((char)140) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)1604);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)150, (char)66, (char)150, (char)81, (char)198, (char)24, (char)198, (char)65, (char)73, (char)159, (char)166, (char)31, (char)68, (char)49, (char)114, (char)205, (char)224, (char)217, (char)244, (char)164, (char)142, (char)48, (char)199, (char)85, (char)226, (char)40, (char)25, (char)6, (char)242, (char)154, (char)244, (char)12, (char)140, (char)111, (char)231, (char)2, (char)133, (char)144, (char)28, (char)87, (char)130, (char)46, (char)100, (char)63, (char)176, (char)89, (char)90, (char)91, (char)98, (char)28, (char)82, (char)140, (char)237, (char)78, (char)230, (char)123, (char)105, (char)14, (char)114, (char)87, (char)110, (char)214, (char)38, (char)51, (char)158, (char)45, (char)185, (char)233, (char)112, (char)33, (char)139, (char)116, (char)64, (char)138, (char)46, (char)154, (char)197, (char)163, (char)124, (char)234, (char)83, (char)164, (char)90, (char)230, (char)237, (char)187, (char)54, (char)203, (char)49, (char)21}));
            assert(pack.count_GET() == (char)212);
            assert(pack.ofs_GET() == 4250565599L);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.count_SET((char)212) ;
        p120.id_SET((char)1604) ;
        p120.ofs_SET(4250565599L) ;
        p120.data__SET(new char[] {(char)150, (char)66, (char)150, (char)81, (char)198, (char)24, (char)198, (char)65, (char)73, (char)159, (char)166, (char)31, (char)68, (char)49, (char)114, (char)205, (char)224, (char)217, (char)244, (char)164, (char)142, (char)48, (char)199, (char)85, (char)226, (char)40, (char)25, (char)6, (char)242, (char)154, (char)244, (char)12, (char)140, (char)111, (char)231, (char)2, (char)133, (char)144, (char)28, (char)87, (char)130, (char)46, (char)100, (char)63, (char)176, (char)89, (char)90, (char)91, (char)98, (char)28, (char)82, (char)140, (char)237, (char)78, (char)230, (char)123, (char)105, (char)14, (char)114, (char)87, (char)110, (char)214, (char)38, (char)51, (char)158, (char)45, (char)185, (char)233, (char)112, (char)33, (char)139, (char)116, (char)64, (char)138, (char)46, (char)154, (char)197, (char)163, (char)124, (char)234, (char)83, (char)164, (char)90, (char)230, (char)237, (char)187, (char)54, (char)203, (char)49, (char)21}, 0) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)244);
            assert(pack.target_component_GET() == (char)220);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)220) ;
        p121.target_system_SET((char)244) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)249);
            assert(pack.target_component_GET() == (char)196);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_component_SET((char)196) ;
        p122.target_system_SET((char)249) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)60);
            assert(pack.target_component_GET() == (char)173);
            assert(pack.target_system_GET() == (char)91);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)40, (char)151, (char)156, (char)104, (char)227, (char)134, (char)203, (char)90, (char)76, (char)255, (char)125, (char)227, (char)217, (char)90, (char)106, (char)47, (char)145, (char)86, (char)24, (char)59, (char)141, (char)170, (char)150, (char)8, (char)126, (char)235, (char)115, (char)83, (char)222, (char)95, (char)106, (char)149, (char)219, (char)65, (char)88, (char)239, (char)46, (char)56, (char)227, (char)21, (char)217, (char)185, (char)69, (char)172, (char)40, (char)148, (char)243, (char)39, (char)2, (char)156, (char)56, (char)0, (char)187, (char)88, (char)110, (char)252, (char)48, (char)34, (char)8, (char)33, (char)104, (char)191, (char)126, (char)56, (char)151, (char)23, (char)147, (char)181, (char)128, (char)75, (char)75, (char)99, (char)76, (char)14, (char)111, (char)27, (char)88, (char)110, (char)186, (char)83, (char)181, (char)8, (char)78, (char)85, (char)171, (char)186, (char)115, (char)167, (char)195, (char)162, (char)183, (char)133, (char)67, (char)218, (char)249, (char)161, (char)48, (char)183, (char)60, (char)241, (char)4, (char)207, (char)118, (char)158, (char)126, (char)82, (char)164, (char)198, (char)147, (char)191}));
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_component_SET((char)173) ;
        p123.data__SET(new char[] {(char)40, (char)151, (char)156, (char)104, (char)227, (char)134, (char)203, (char)90, (char)76, (char)255, (char)125, (char)227, (char)217, (char)90, (char)106, (char)47, (char)145, (char)86, (char)24, (char)59, (char)141, (char)170, (char)150, (char)8, (char)126, (char)235, (char)115, (char)83, (char)222, (char)95, (char)106, (char)149, (char)219, (char)65, (char)88, (char)239, (char)46, (char)56, (char)227, (char)21, (char)217, (char)185, (char)69, (char)172, (char)40, (char)148, (char)243, (char)39, (char)2, (char)156, (char)56, (char)0, (char)187, (char)88, (char)110, (char)252, (char)48, (char)34, (char)8, (char)33, (char)104, (char)191, (char)126, (char)56, (char)151, (char)23, (char)147, (char)181, (char)128, (char)75, (char)75, (char)99, (char)76, (char)14, (char)111, (char)27, (char)88, (char)110, (char)186, (char)83, (char)181, (char)8, (char)78, (char)85, (char)171, (char)186, (char)115, (char)167, (char)195, (char)162, (char)183, (char)133, (char)67, (char)218, (char)249, (char)161, (char)48, (char)183, (char)60, (char)241, (char)4, (char)207, (char)118, (char)158, (char)126, (char)82, (char)164, (char)198, (char)147, (char)191}, 0) ;
        p123.target_system_SET((char)91) ;
        p123.len_SET((char)60) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 933007140088785224L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.vel_GET() == (char)7808);
            assert(pack.alt_GET() == 1843269052);
            assert(pack.dgps_numch_GET() == (char)80);
            assert(pack.lon_GET() == -556831574);
            assert(pack.eph_GET() == (char)32945);
            assert(pack.satellites_visible_GET() == (char)198);
            assert(pack.lat_GET() == 146532352);
            assert(pack.cog_GET() == (char)25203);
            assert(pack.dgps_age_GET() == 3152547782L);
            assert(pack.epv_GET() == (char)26533);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.lat_SET(146532352) ;
        p124.vel_SET((char)7808) ;
        p124.alt_SET(1843269052) ;
        p124.lon_SET(-556831574) ;
        p124.satellites_visible_SET((char)198) ;
        p124.epv_SET((char)26533) ;
        p124.time_usec_SET(933007140088785224L) ;
        p124.dgps_numch_SET((char)80) ;
        p124.eph_SET((char)32945) ;
        p124.dgps_age_SET(3152547782L) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p124.cog_SET((char)25203) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vservo_GET() == (char)13618);
            assert(pack.Vcc_GET() == (char)67);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED));
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)13618) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED)) ;
        p125.Vcc_SET((char)67) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)139, (char)23, (char)205, (char)1, (char)207, (char)146, (char)40, (char)143, (char)171, (char)82, (char)121, (char)164, (char)239, (char)4, (char)107, (char)64, (char)201, (char)42, (char)51, (char)148, (char)2, (char)242, (char)209, (char)5, (char)215, (char)93, (char)180, (char)45, (char)204, (char)163, (char)163, (char)59, (char)115, (char)255, (char)180, (char)26, (char)6, (char)47, (char)99, (char)125, (char)96, (char)249, (char)23, (char)196, (char)33, (char)148, (char)161, (char)27, (char)178, (char)168, (char)146, (char)210, (char)148, (char)27, (char)206, (char)82, (char)140, (char)54, (char)15, (char)33, (char)95, (char)7, (char)22, (char)114, (char)146, (char)252, (char)64, (char)102, (char)205, (char)30}));
            assert(pack.count_GET() == (char)224);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI));
            assert(pack.timeout_GET() == (char)27127);
            assert(pack.baudrate_GET() == 3821000392L);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1) ;
        p126.count_SET((char)224) ;
        p126.data__SET(new char[] {(char)139, (char)23, (char)205, (char)1, (char)207, (char)146, (char)40, (char)143, (char)171, (char)82, (char)121, (char)164, (char)239, (char)4, (char)107, (char)64, (char)201, (char)42, (char)51, (char)148, (char)2, (char)242, (char)209, (char)5, (char)215, (char)93, (char)180, (char)45, (char)204, (char)163, (char)163, (char)59, (char)115, (char)255, (char)180, (char)26, (char)6, (char)47, (char)99, (char)125, (char)96, (char)249, (char)23, (char)196, (char)33, (char)148, (char)161, (char)27, (char)178, (char)168, (char)146, (char)210, (char)148, (char)27, (char)206, (char)82, (char)140, (char)54, (char)15, (char)33, (char)95, (char)7, (char)22, (char)114, (char)146, (char)252, (char)64, (char)102, (char)205, (char)30}, 0) ;
        p126.baudrate_SET(3821000392L) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI)) ;
        p126.timeout_SET((char)27127) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.rtk_receiver_id_GET() == (char)120);
            assert(pack.iar_num_hypotheses_GET() == -1910137507);
            assert(pack.baseline_a_mm_GET() == -11831489);
            assert(pack.tow_GET() == 3084534590L);
            assert(pack.accuracy_GET() == 2693402342L);
            assert(pack.baseline_c_mm_GET() == 231701366);
            assert(pack.wn_GET() == (char)41957);
            assert(pack.nsats_GET() == (char)203);
            assert(pack.rtk_health_GET() == (char)207);
            assert(pack.baseline_b_mm_GET() == -1643254955);
            assert(pack.baseline_coords_type_GET() == (char)11);
            assert(pack.rtk_rate_GET() == (char)163);
            assert(pack.time_last_baseline_ms_GET() == 2039795887L);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.accuracy_SET(2693402342L) ;
        p127.baseline_coords_type_SET((char)11) ;
        p127.tow_SET(3084534590L) ;
        p127.baseline_c_mm_SET(231701366) ;
        p127.rtk_health_SET((char)207) ;
        p127.baseline_b_mm_SET(-1643254955) ;
        p127.iar_num_hypotheses_SET(-1910137507) ;
        p127.wn_SET((char)41957) ;
        p127.rtk_receiver_id_SET((char)120) ;
        p127.baseline_a_mm_SET(-11831489) ;
        p127.nsats_SET((char)203) ;
        p127.rtk_rate_SET((char)163) ;
        p127.time_last_baseline_ms_SET(2039795887L) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_b_mm_GET() == 763402585);
            assert(pack.baseline_c_mm_GET() == 662648719);
            assert(pack.baseline_a_mm_GET() == -619705696);
            assert(pack.rtk_health_GET() == (char)83);
            assert(pack.tow_GET() == 2923426559L);
            assert(pack.wn_GET() == (char)61073);
            assert(pack.rtk_receiver_id_GET() == (char)151);
            assert(pack.baseline_coords_type_GET() == (char)35);
            assert(pack.nsats_GET() == (char)185);
            assert(pack.rtk_rate_GET() == (char)41);
            assert(pack.accuracy_GET() == 3788070817L);
            assert(pack.iar_num_hypotheses_GET() == -1452175841);
            assert(pack.time_last_baseline_ms_GET() == 1842956185L);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_coords_type_SET((char)35) ;
        p128.rtk_rate_SET((char)41) ;
        p128.rtk_health_SET((char)83) ;
        p128.tow_SET(2923426559L) ;
        p128.baseline_b_mm_SET(763402585) ;
        p128.baseline_c_mm_SET(662648719) ;
        p128.accuracy_SET(3788070817L) ;
        p128.iar_num_hypotheses_SET(-1452175841) ;
        p128.nsats_SET((char)185) ;
        p128.baseline_a_mm_SET(-619705696) ;
        p128.time_last_baseline_ms_SET(1842956185L) ;
        p128.rtk_receiver_id_SET((char)151) ;
        p128.wn_SET((char)61073) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == (short) -21707);
            assert(pack.xgyro_GET() == (short)17938);
            assert(pack.xmag_GET() == (short) -14140);
            assert(pack.zmag_GET() == (short)31609);
            assert(pack.time_boot_ms_GET() == 1780132063L);
            assert(pack.zacc_GET() == (short) -29364);
            assert(pack.yacc_GET() == (short) -14006);
            assert(pack.xacc_GET() == (short) -30364);
            assert(pack.zgyro_GET() == (short)15770);
            assert(pack.ygyro_GET() == (short)10134);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.zgyro_SET((short)15770) ;
        p129.ygyro_SET((short)10134) ;
        p129.yacc_SET((short) -14006) ;
        p129.xacc_SET((short) -30364) ;
        p129.zmag_SET((short)31609) ;
        p129.xgyro_SET((short)17938) ;
        p129.time_boot_ms_SET(1780132063L) ;
        p129.xmag_SET((short) -14140) ;
        p129.zacc_SET((short) -29364) ;
        p129.ymag_SET((short) -21707) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.height_GET() == (char)44564);
            assert(pack.packets_GET() == (char)12647);
            assert(pack.size_GET() == 1721482863L);
            assert(pack.payload_GET() == (char)139);
            assert(pack.jpg_quality_GET() == (char)166);
            assert(pack.width_GET() == (char)2350);
            assert(pack.type_GET() == (char)129);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.packets_SET((char)12647) ;
        p130.type_SET((char)129) ;
        p130.width_SET((char)2350) ;
        p130.jpg_quality_SET((char)166) ;
        p130.height_SET((char)44564) ;
        p130.size_SET(1721482863L) ;
        p130.payload_SET((char)139) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)42138);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)224, (char)56, (char)246, (char)224, (char)103, (char)228, (char)248, (char)182, (char)239, (char)69, (char)125, (char)79, (char)159, (char)160, (char)114, (char)177, (char)245, (char)16, (char)254, (char)226, (char)74, (char)55, (char)212, (char)220, (char)172, (char)245, (char)203, (char)147, (char)52, (char)85, (char)167, (char)242, (char)128, (char)62, (char)242, (char)191, (char)75, (char)83, (char)167, (char)48, (char)213, (char)75, (char)0, (char)189, (char)174, (char)160, (char)88, (char)121, (char)231, (char)32, (char)93, (char)83, (char)23, (char)124, (char)185, (char)48, (char)97, (char)234, (char)253, (char)50, (char)28, (char)36, (char)214, (char)219, (char)197, (char)4, (char)5, (char)51, (char)46, (char)202, (char)125, (char)96, (char)182, (char)241, (char)134, (char)152, (char)172, (char)240, (char)119, (char)78, (char)131, (char)221, (char)128, (char)187, (char)40, (char)186, (char)60, (char)67, (char)101, (char)214, (char)193, (char)174, (char)20, (char)105, (char)48, (char)77, (char)152, (char)122, (char)133, (char)111, (char)176, (char)220, (char)159, (char)206, (char)10, (char)123, (char)38, (char)136, (char)181, (char)213, (char)125, (char)232, (char)126, (char)13, (char)6, (char)137, (char)45, (char)138, (char)1, (char)184, (char)199, (char)88, (char)75, (char)247, (char)43, (char)152, (char)226, (char)51, (char)65, (char)142, (char)238, (char)136, (char)115, (char)73, (char)91, (char)179, (char)95, (char)122, (char)54, (char)57, (char)121, (char)169, (char)149, (char)184, (char)142, (char)184, (char)75, (char)193, (char)172, (char)246, (char)143, (char)143, (char)204, (char)45, (char)104, (char)3, (char)201, (char)159, (char)92, (char)82, (char)29, (char)28, (char)10, (char)56, (char)254, (char)237, (char)68, (char)169, (char)244, (char)139, (char)42, (char)78, (char)225, (char)42, (char)37, (char)231, (char)26, (char)16, (char)132, (char)36, (char)27, (char)233, (char)3, (char)188, (char)169, (char)244, (char)134, (char)32, (char)79, (char)131, (char)127, (char)237, (char)30, (char)75, (char)238, (char)47, (char)62, (char)149, (char)188, (char)140, (char)164, (char)153, (char)91, (char)213, (char)98, (char)47, (char)225, (char)230, (char)7, (char)4, (char)151, (char)144, (char)119, (char)192, (char)75, (char)19, (char)112, (char)11, (char)214, (char)18, (char)49, (char)194, (char)9, (char)181, (char)82, (char)210, (char)134, (char)159, (char)139, (char)30, (char)235, (char)26, (char)145, (char)116, (char)221, (char)175, (char)223, (char)32, (char)244, (char)160, (char)125, (char)168, (char)214, (char)141, (char)176, (char)119, (char)30, (char)193, (char)34, (char)13, (char)237, (char)66, (char)230}));
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)42138) ;
        p131.data__SET(new char[] {(char)224, (char)56, (char)246, (char)224, (char)103, (char)228, (char)248, (char)182, (char)239, (char)69, (char)125, (char)79, (char)159, (char)160, (char)114, (char)177, (char)245, (char)16, (char)254, (char)226, (char)74, (char)55, (char)212, (char)220, (char)172, (char)245, (char)203, (char)147, (char)52, (char)85, (char)167, (char)242, (char)128, (char)62, (char)242, (char)191, (char)75, (char)83, (char)167, (char)48, (char)213, (char)75, (char)0, (char)189, (char)174, (char)160, (char)88, (char)121, (char)231, (char)32, (char)93, (char)83, (char)23, (char)124, (char)185, (char)48, (char)97, (char)234, (char)253, (char)50, (char)28, (char)36, (char)214, (char)219, (char)197, (char)4, (char)5, (char)51, (char)46, (char)202, (char)125, (char)96, (char)182, (char)241, (char)134, (char)152, (char)172, (char)240, (char)119, (char)78, (char)131, (char)221, (char)128, (char)187, (char)40, (char)186, (char)60, (char)67, (char)101, (char)214, (char)193, (char)174, (char)20, (char)105, (char)48, (char)77, (char)152, (char)122, (char)133, (char)111, (char)176, (char)220, (char)159, (char)206, (char)10, (char)123, (char)38, (char)136, (char)181, (char)213, (char)125, (char)232, (char)126, (char)13, (char)6, (char)137, (char)45, (char)138, (char)1, (char)184, (char)199, (char)88, (char)75, (char)247, (char)43, (char)152, (char)226, (char)51, (char)65, (char)142, (char)238, (char)136, (char)115, (char)73, (char)91, (char)179, (char)95, (char)122, (char)54, (char)57, (char)121, (char)169, (char)149, (char)184, (char)142, (char)184, (char)75, (char)193, (char)172, (char)246, (char)143, (char)143, (char)204, (char)45, (char)104, (char)3, (char)201, (char)159, (char)92, (char)82, (char)29, (char)28, (char)10, (char)56, (char)254, (char)237, (char)68, (char)169, (char)244, (char)139, (char)42, (char)78, (char)225, (char)42, (char)37, (char)231, (char)26, (char)16, (char)132, (char)36, (char)27, (char)233, (char)3, (char)188, (char)169, (char)244, (char)134, (char)32, (char)79, (char)131, (char)127, (char)237, (char)30, (char)75, (char)238, (char)47, (char)62, (char)149, (char)188, (char)140, (char)164, (char)153, (char)91, (char)213, (char)98, (char)47, (char)225, (char)230, (char)7, (char)4, (char)151, (char)144, (char)119, (char)192, (char)75, (char)19, (char)112, (char)11, (char)214, (char)18, (char)49, (char)194, (char)9, (char)181, (char)82, (char)210, (char)134, (char)159, (char)139, (char)30, (char)235, (char)26, (char)145, (char)116, (char)221, (char)175, (char)223, (char)32, (char)244, (char)160, (char)125, (char)168, (char)214, (char)141, (char)176, (char)119, (char)30, (char)193, (char)34, (char)13, (char)237, (char)66, (char)230}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.time_boot_ms_GET() == 1822376340L);
            assert(pack.max_distance_GET() == (char)22335);
            assert(pack.current_distance_GET() == (char)40369);
            assert(pack.id_GET() == (char)111);
            assert(pack.covariance_GET() == (char)224);
            assert(pack.min_distance_GET() == (char)50521);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_270);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_270) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p132.current_distance_SET((char)40369) ;
        p132.max_distance_SET((char)22335) ;
        p132.id_SET((char)111) ;
        p132.covariance_SET((char)224) ;
        p132.min_distance_SET((char)50521) ;
        p132.time_boot_ms_SET(1822376340L) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1049442316);
            assert(pack.lat_GET() == 2142157400);
            assert(pack.mask_GET() == 421902622399668589L);
            assert(pack.grid_spacing_GET() == (char)20234);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(2142157400) ;
        p133.grid_spacing_SET((char)20234) ;
        p133.mask_SET(421902622399668589L) ;
        p133.lon_SET(-1049442316) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.grid_spacing_GET() == (char)45744);
            assert(pack.lon_GET() == -2026730569);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)5371, (short) -23380, (short)19415, (short)24218, (short)10257, (short) -9129, (short) -7318, (short) -27988, (short) -3134, (short) -9931, (short)14452, (short) -26229, (short) -23900, (short) -24957, (short)20399, (short)8249}));
            assert(pack.gridbit_GET() == (char)200);
            assert(pack.lat_GET() == 2120362381);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lon_SET(-2026730569) ;
        p134.grid_spacing_SET((char)45744) ;
        p134.gridbit_SET((char)200) ;
        p134.data__SET(new short[] {(short)5371, (short) -23380, (short)19415, (short)24218, (short)10257, (short) -9129, (short) -7318, (short) -27988, (short) -3134, (short) -9931, (short)14452, (short) -26229, (short) -23900, (short) -24957, (short)20399, (short)8249}, 0) ;
        p134.lat_SET(2120362381) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 2019421745);
            assert(pack.lon_GET() == -1126534905);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(-1126534905) ;
        p135.lat_SET(2019421745) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.loaded_GET() == (char)4397);
            assert(pack.lat_GET() == 840555911);
            assert(pack.current_height_GET() == 2.958492E38F);
            assert(pack.spacing_GET() == (char)49171);
            assert(pack.pending_GET() == (char)32364);
            assert(pack.lon_GET() == -1799255276);
            assert(pack.terrain_height_GET() == -4.3114198E36F);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.spacing_SET((char)49171) ;
        p136.terrain_height_SET(-4.3114198E36F) ;
        p136.lat_SET(840555911) ;
        p136.lon_SET(-1799255276) ;
        p136.loaded_SET((char)4397) ;
        p136.current_height_SET(2.958492E38F) ;
        p136.pending_SET((char)32364) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -20872);
            assert(pack.press_abs_GET() == -8.903245E37F);
            assert(pack.time_boot_ms_GET() == 1540246551L);
            assert(pack.press_diff_GET() == -1.6574289E38F);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(1540246551L) ;
        p137.temperature_SET((short) -20872) ;
        p137.press_abs_SET(-8.903245E37F) ;
        p137.press_diff_SET(-1.6574289E38F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -1.4707159E38F);
            assert(pack.y_GET() == 1.2806975E38F);
            assert(pack.z_GET() == 2.6277184E38F);
            assert(pack.time_usec_GET() == 5596118360228482019L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.6896573E38F, 3.3252883E38F, 1.4580497E38F, -1.5739307E38F}));
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.y_SET(1.2806975E38F) ;
        p138.time_usec_SET(5596118360228482019L) ;
        p138.x_SET(-1.4707159E38F) ;
        p138.z_SET(2.6277184E38F) ;
        p138.q_SET(new float[] {-1.6896573E38F, 3.3252883E38F, 1.4580497E38F, -1.5739307E38F}, 0) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 585448173618059861L);
            assert(pack.target_component_GET() == (char)142);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.5877845E38F, 2.3709573E38F, 2.562967E37F, -1.8287065E38F, 3.3551647E38F, -1.7429788E38F, 1.5542955E38F, -3.1148368E38F}));
            assert(pack.target_system_GET() == (char)191);
            assert(pack.group_mlx_GET() == (char)19);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.group_mlx_SET((char)19) ;
        p139.target_system_SET((char)191) ;
        p139.controls_SET(new float[] {-1.5877845E38F, 2.3709573E38F, 2.562967E37F, -1.8287065E38F, 3.3551647E38F, -1.7429788E38F, 1.5542955E38F, -3.1148368E38F}, 0) ;
        p139.target_component_SET((char)142) ;
        p139.time_usec_SET(585448173618059861L) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5033137818096784529L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.6944121E38F, -3.1998734E38F, -1.4699227E38F, -2.5511369E38F, -5.1129465E37F, 1.4260524E38F, -1.989608E37F, -2.920883E38F}));
            assert(pack.group_mlx_GET() == (char)14);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)14) ;
        p140.time_usec_SET(5033137818096784529L) ;
        p140.controls_SET(new float[] {-1.6944121E38F, -3.1998734E38F, -1.4699227E38F, -2.5511369E38F, -5.1129465E37F, 1.4260524E38F, -1.989608E37F, -2.920883E38F}, 0) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_local_GET() == -2.7508318E38F);
            assert(pack.bottom_clearance_GET() == 1.8464298E38F);
            assert(pack.altitude_amsl_GET() == 2.3383466E37F);
            assert(pack.altitude_relative_GET() == 2.6544476E38F);
            assert(pack.altitude_monotonic_GET() == 2.3653E38F);
            assert(pack.time_usec_GET() == 6211935720576252851L);
            assert(pack.altitude_terrain_GET() == -2.053942E38F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_monotonic_SET(2.3653E38F) ;
        p141.time_usec_SET(6211935720576252851L) ;
        p141.altitude_relative_SET(2.6544476E38F) ;
        p141.bottom_clearance_SET(1.8464298E38F) ;
        p141.altitude_local_SET(-2.7508318E38F) ;
        p141.altitude_amsl_SET(2.3383466E37F) ;
        p141.altitude_terrain_SET(-2.053942E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.uri_type_GET() == (char)217);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)35, (char)203, (char)153, (char)76, (char)156, (char)4, (char)16, (char)122, (char)148, (char)33, (char)164, (char)120, (char)207, (char)101, (char)212, (char)114, (char)213, (char)190, (char)240, (char)80, (char)200, (char)173, (char)246, (char)169, (char)204, (char)0, (char)66, (char)158, (char)192, (char)77, (char)85, (char)48, (char)42, (char)11, (char)31, (char)154, (char)69, (char)229, (char)48, (char)69, (char)143, (char)11, (char)110, (char)226, (char)176, (char)30, (char)184, (char)171, (char)218, (char)16, (char)130, (char)232, (char)155, (char)165, (char)221, (char)237, (char)179, (char)99, (char)156, (char)223, (char)109, (char)47, (char)29, (char)231, (char)36, (char)114, (char)54, (char)7, (char)23, (char)116, (char)29, (char)185, (char)79, (char)10, (char)237, (char)174, (char)164, (char)255, (char)127, (char)96, (char)5, (char)165, (char)201, (char)109, (char)10, (char)125, (char)59, (char)187, (char)213, (char)118, (char)62, (char)196, (char)17, (char)254, (char)152, (char)233, (char)157, (char)85, (char)0, (char)53, (char)164, (char)98, (char)71, (char)95, (char)14, (char)163, (char)187, (char)228, (char)98, (char)85, (char)179, (char)121, (char)115, (char)240, (char)205, (char)205, (char)36, (char)86, (char)108, (char)131}));
            assert(pack.request_id_GET() == (char)49);
            assert(pack.transfer_type_GET() == (char)122);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)63, (char)177, (char)60, (char)32, (char)59, (char)127, (char)184, (char)236, (char)195, (char)134, (char)220, (char)113, (char)189, (char)50, (char)182, (char)176, (char)163, (char)10, (char)219, (char)202, (char)230, (char)191, (char)63, (char)207, (char)98, (char)41, (char)99, (char)251, (char)67, (char)191, (char)232, (char)192, (char)104, (char)13, (char)129, (char)147, (char)52, (char)27, (char)14, (char)174, (char)201, (char)37, (char)194, (char)115, (char)88, (char)171, (char)95, (char)129, (char)134, (char)86, (char)69, (char)203, (char)221, (char)97, (char)39, (char)208, (char)100, (char)175, (char)144, (char)13, (char)116, (char)4, (char)237, (char)102, (char)106, (char)84, (char)178, (char)93, (char)64, (char)223, (char)227, (char)149, (char)119, (char)146, (char)19, (char)232, (char)185, (char)161, (char)58, (char)24, (char)252, (char)251, (char)57, (char)26, (char)157, (char)179, (char)249, (char)55, (char)100, (char)80, (char)54, (char)7, (char)120, (char)102, (char)230, (char)130, (char)51, (char)145, (char)16, (char)34, (char)155, (char)114, (char)235, (char)235, (char)253, (char)255, (char)81, (char)1, (char)130, (char)133, (char)188, (char)216, (char)111, (char)183, (char)184, (char)207, (char)182, (char)216, (char)171, (char)120}));
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.transfer_type_SET((char)122) ;
        p142.storage_SET(new char[] {(char)35, (char)203, (char)153, (char)76, (char)156, (char)4, (char)16, (char)122, (char)148, (char)33, (char)164, (char)120, (char)207, (char)101, (char)212, (char)114, (char)213, (char)190, (char)240, (char)80, (char)200, (char)173, (char)246, (char)169, (char)204, (char)0, (char)66, (char)158, (char)192, (char)77, (char)85, (char)48, (char)42, (char)11, (char)31, (char)154, (char)69, (char)229, (char)48, (char)69, (char)143, (char)11, (char)110, (char)226, (char)176, (char)30, (char)184, (char)171, (char)218, (char)16, (char)130, (char)232, (char)155, (char)165, (char)221, (char)237, (char)179, (char)99, (char)156, (char)223, (char)109, (char)47, (char)29, (char)231, (char)36, (char)114, (char)54, (char)7, (char)23, (char)116, (char)29, (char)185, (char)79, (char)10, (char)237, (char)174, (char)164, (char)255, (char)127, (char)96, (char)5, (char)165, (char)201, (char)109, (char)10, (char)125, (char)59, (char)187, (char)213, (char)118, (char)62, (char)196, (char)17, (char)254, (char)152, (char)233, (char)157, (char)85, (char)0, (char)53, (char)164, (char)98, (char)71, (char)95, (char)14, (char)163, (char)187, (char)228, (char)98, (char)85, (char)179, (char)121, (char)115, (char)240, (char)205, (char)205, (char)36, (char)86, (char)108, (char)131}, 0) ;
        p142.request_id_SET((char)49) ;
        p142.uri_SET(new char[] {(char)63, (char)177, (char)60, (char)32, (char)59, (char)127, (char)184, (char)236, (char)195, (char)134, (char)220, (char)113, (char)189, (char)50, (char)182, (char)176, (char)163, (char)10, (char)219, (char)202, (char)230, (char)191, (char)63, (char)207, (char)98, (char)41, (char)99, (char)251, (char)67, (char)191, (char)232, (char)192, (char)104, (char)13, (char)129, (char)147, (char)52, (char)27, (char)14, (char)174, (char)201, (char)37, (char)194, (char)115, (char)88, (char)171, (char)95, (char)129, (char)134, (char)86, (char)69, (char)203, (char)221, (char)97, (char)39, (char)208, (char)100, (char)175, (char)144, (char)13, (char)116, (char)4, (char)237, (char)102, (char)106, (char)84, (char)178, (char)93, (char)64, (char)223, (char)227, (char)149, (char)119, (char)146, (char)19, (char)232, (char)185, (char)161, (char)58, (char)24, (char)252, (char)251, (char)57, (char)26, (char)157, (char)179, (char)249, (char)55, (char)100, (char)80, (char)54, (char)7, (char)120, (char)102, (char)230, (char)130, (char)51, (char)145, (char)16, (char)34, (char)155, (char)114, (char)235, (char)235, (char)253, (char)255, (char)81, (char)1, (char)130, (char)133, (char)188, (char)216, (char)111, (char)183, (char)184, (char)207, (char)182, (char)216, (char)171, (char)120}, 0) ;
        p142.uri_type_SET((char)217) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)11382);
            assert(pack.press_abs_GET() == -2.9477391E38F);
            assert(pack.time_boot_ms_GET() == 2235957722L);
            assert(pack.press_diff_GET() == 7.8390974E35F);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(2235957722L) ;
        p143.temperature_SET((short)11382) ;
        p143.press_abs_SET(-2.9477391E38F) ;
        p143.press_diff_SET(7.8390974E35F) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.est_capabilities_GET() == (char)243);
            assert(pack.lon_GET() == 2019414227);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {2.0836304E38F, 1.326644E38F, -1.6156392E38F}));
            assert(pack.custom_state_GET() == 8922189062333800697L);
            assert(pack.lat_GET() == -87128249);
            assert(pack.timestamp_GET() == 3041199354871090700L);
            assert(pack.alt_GET() == -2.9244277E38F);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {5.112269E37F, 1.3515948E38F, 2.1773486E37F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-6.1734676E37F, -3.3671248E38F, 7.9632974E37F, 2.3900589E38F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {1.6874749E38F, 8.55539E37F, -3.2126438E38F}));
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-2.032432E37F, 2.9255135E38F, -1.0440637E38F}));
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.vel_SET(new float[] {5.112269E37F, 1.3515948E38F, 2.1773486E37F}, 0) ;
        p144.alt_SET(-2.9244277E38F) ;
        p144.timestamp_SET(3041199354871090700L) ;
        p144.rates_SET(new float[] {1.6874749E38F, 8.55539E37F, -3.2126438E38F}, 0) ;
        p144.est_capabilities_SET((char)243) ;
        p144.lon_SET(2019414227) ;
        p144.lat_SET(-87128249) ;
        p144.attitude_q_SET(new float[] {-6.1734676E37F, -3.3671248E38F, 7.9632974E37F, 2.3900589E38F}, 0) ;
        p144.acc_SET(new float[] {2.0836304E38F, 1.326644E38F, -1.6156392E38F}, 0) ;
        p144.position_cov_SET(new float[] {-2.032432E37F, 2.9255135E38F, -1.0440637E38F}, 0) ;
        p144.custom_state_SET(8922189062333800697L) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.z_pos_GET() == -2.3783005E38F);
            assert(pack.yaw_rate_GET() == 2.2809947E38F);
            assert(pack.x_vel_GET() == -9.008252E37F);
            assert(pack.x_pos_GET() == -2.5156467E38F);
            assert(pack.airspeed_GET() == -2.8311481E38F);
            assert(pack.pitch_rate_GET() == -3.3814499E38F);
            assert(pack.x_acc_GET() == -2.0484694E38F);
            assert(pack.y_pos_GET() == -3.0232377E37F);
            assert(pack.roll_rate_GET() == -2.5201892E38F);
            assert(pack.y_vel_GET() == -3.3815533E38F);
            assert(pack.time_usec_GET() == 4374607798558113851L);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-3.1231743E38F, -3.3792324E38F, -2.640868E38F}));
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {1.0410925E38F, -8.272435E37F, -1.2658583E38F}));
            assert(pack.y_acc_GET() == 2.0377593E38F);
            assert(pack.z_acc_GET() == 1.8216403E38F);
            assert(pack.z_vel_GET() == 3.5902083E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.2057743E38F, 2.439139E38F, 2.7275352E38F, -3.045775E38F}));
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.vel_variance_SET(new float[] {-3.1231743E38F, -3.3792324E38F, -2.640868E38F}, 0) ;
        p146.time_usec_SET(4374607798558113851L) ;
        p146.x_vel_SET(-9.008252E37F) ;
        p146.z_acc_SET(1.8216403E38F) ;
        p146.y_acc_SET(2.0377593E38F) ;
        p146.y_pos_SET(-3.0232377E37F) ;
        p146.x_pos_SET(-2.5156467E38F) ;
        p146.y_vel_SET(-3.3815533E38F) ;
        p146.roll_rate_SET(-2.5201892E38F) ;
        p146.x_acc_SET(-2.0484694E38F) ;
        p146.airspeed_SET(-2.8311481E38F) ;
        p146.yaw_rate_SET(2.2809947E38F) ;
        p146.z_pos_SET(-2.3783005E38F) ;
        p146.pitch_rate_SET(-3.3814499E38F) ;
        p146.pos_variance_SET(new float[] {1.0410925E38F, -8.272435E37F, -1.2658583E38F}, 0) ;
        p146.z_vel_SET(3.5902083E37F) ;
        p146.q_SET(new float[] {2.2057743E38F, 2.439139E38F, 2.7275352E38F, -3.045775E38F}, 0) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)199);
            assert(pack.temperature_GET() == (short) -3086);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
            assert(pack.current_consumed_GET() == 1021017330);
            assert(pack.battery_remaining_GET() == (byte)90);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)44851, (char)6898, (char)47241, (char)39248, (char)59398, (char)62346, (char)21404, (char)2950, (char)49475, (char)52023}));
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
            assert(pack.current_battery_GET() == (short)11289);
            assert(pack.energy_consumed_GET() == 1185780977);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.voltages_SET(new char[] {(char)44851, (char)6898, (char)47241, (char)39248, (char)59398, (char)62346, (char)21404, (char)2950, (char)49475, (char)52023}, 0) ;
        p147.current_consumed_SET(1021017330) ;
        p147.battery_remaining_SET((byte)90) ;
        p147.current_battery_SET((short)11289) ;
        p147.energy_consumed_SET(1185780977) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN) ;
        p147.temperature_SET((short) -3086) ;
        p147.id_SET((char)199) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.os_sw_version_GET() == 1242882894L);
            assert(pack.uid_GET() == 6217364873619614454L);
            assert(pack.board_version_GET() == 4085051824L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)196, (char)96, (char)234, (char)116, (char)96, (char)246, (char)191, (char)99}));
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT));
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)153, (char)182, (char)169, (char)48, (char)14, (char)149, (char)70, (char)94}));
            assert(pack.vendor_id_GET() == (char)32028);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)197, (char)183, (char)254, (char)171, (char)136, (char)245, (char)170, (char)209}));
            assert(pack.product_id_GET() == (char)5882);
            assert(pack.flight_sw_version_GET() == 3572083497L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)199, (char)182, (char)65, (char)172, (char)191, (char)77, (char)127, (char)49, (char)236, (char)101, (char)54, (char)168, (char)244, (char)101, (char)56, (char)192, (char)214, (char)182}));
            assert(pack.middleware_sw_version_GET() == 906694480L);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.board_version_SET(4085051824L) ;
        p148.uid2_SET(new char[] {(char)199, (char)182, (char)65, (char)172, (char)191, (char)77, (char)127, (char)49, (char)236, (char)101, (char)54, (char)168, (char)244, (char)101, (char)56, (char)192, (char)214, (char)182}, 0, PH) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT)) ;
        p148.product_id_SET((char)5882) ;
        p148.flight_sw_version_SET(3572083497L) ;
        p148.uid_SET(6217364873619614454L) ;
        p148.os_sw_version_SET(1242882894L) ;
        p148.vendor_id_SET((char)32028) ;
        p148.middleware_sw_version_SET(906694480L) ;
        p148.flight_custom_version_SET(new char[] {(char)197, (char)183, (char)254, (char)171, (char)136, (char)245, (char)170, (char)209}, 0) ;
        p148.middleware_custom_version_SET(new char[] {(char)153, (char)182, (char)169, (char)48, (char)14, (char)149, (char)70, (char)94}, 0) ;
        p148.os_custom_version_SET(new char[] {(char)196, (char)96, (char)234, (char)116, (char)96, (char)246, (char)191, (char)99}, 0) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_num_GET() == (char)199);
            assert(pack.angle_x_GET() == 2.9350103E38F);
            assert(pack.size_y_GET() == -1.3257644E38F);
            assert(pack.x_TRY(ph) == -1.1287639E38F);
            assert(pack.time_usec_GET() == 6789884698002290067L);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
            assert(pack.distance_GET() == -1.3822505E38F);
            assert(pack.angle_y_GET() == -1.6626533E37F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {9.405178E36F, 2.2190504E38F, -1.5871112E38F, -2.1969194E38F}));
            assert(pack.size_x_GET() == 2.6373757E38F);
            assert(pack.z_TRY(ph) == -4.9912556E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.y_TRY(ph) == -2.291021E38F);
            assert(pack.position_valid_TRY(ph) == (char)22);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.z_SET(-4.9912556E37F, PH) ;
        p149.position_valid_SET((char)22, PH) ;
        p149.size_y_SET(-1.3257644E38F) ;
        p149.x_SET(-1.1287639E38F, PH) ;
        p149.target_num_SET((char)199) ;
        p149.angle_y_SET(-1.6626533E37F) ;
        p149.size_x_SET(2.6373757E38F) ;
        p149.y_SET(-2.291021E38F, PH) ;
        p149.distance_SET(-1.3822505E38F) ;
        p149.q_SET(new float[] {9.405178E36F, 2.2190504E38F, -1.5871112E38F, -2.1969194E38F}, 0, PH) ;
        p149.time_usec_SET(6789884698002290067L) ;
        p149.angle_x_SET(2.9350103E38F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENS_POWER.add((src, ph, pack) ->
        {
            assert(pack.adc121_cs2_amp_GET() == 3.2497187E38F);
            assert(pack.adc121_cs1_amp_GET() == 1.0771535E37F);
            assert(pack.adc121_cspb_amp_GET() == 3.0338294E38F);
            assert(pack.adc121_vspb_volt_GET() == 1.9124531E38F);
        });
        GroundControl.SENS_POWER p201 = CommunicationChannel.new_SENS_POWER();
        PH.setPack(p201);
        p201.adc121_cspb_amp_SET(3.0338294E38F) ;
        p201.adc121_vspb_volt_SET(1.9124531E38F) ;
        p201.adc121_cs1_amp_SET(1.0771535E37F) ;
        p201.adc121_cs2_amp_SET(3.2497187E38F) ;
        CommunicationChannel.instance.send(p201);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENS_MPPT.add((src, ph, pack) ->
        {
            assert(pack.mppt1_pwm_GET() == (char)44626);
            assert(pack.mppt3_pwm_GET() == (char)18454);
            assert(pack.mppt3_amp_GET() == -6.061155E37F);
            assert(pack.mppt2_status_GET() == (char)229);
            assert(pack.mppt1_volt_GET() == -1.0258623E38F);
            assert(pack.mppt2_amp_GET() == -1.0407533E38F);
            assert(pack.mppt2_pwm_GET() == (char)6490);
            assert(pack.mppt1_amp_GET() == 1.2970489E38F);
            assert(pack.mppt_timestamp_GET() == 7198987226647600323L);
            assert(pack.mppt2_volt_GET() == 1.3484602E38F);
            assert(pack.mppt3_volt_GET() == 1.5706281E37F);
            assert(pack.mppt3_status_GET() == (char)97);
            assert(pack.mppt1_status_GET() == (char)103);
        });
        GroundControl.SENS_MPPT p202 = CommunicationChannel.new_SENS_MPPT();
        PH.setPack(p202);
        p202.mppt3_amp_SET(-6.061155E37F) ;
        p202.mppt3_pwm_SET((char)18454) ;
        p202.mppt2_amp_SET(-1.0407533E38F) ;
        p202.mppt2_volt_SET(1.3484602E38F) ;
        p202.mppt1_status_SET((char)103) ;
        p202.mppt3_volt_SET(1.5706281E37F) ;
        p202.mppt3_status_SET((char)97) ;
        p202.mppt_timestamp_SET(7198987226647600323L) ;
        p202.mppt1_volt_SET(-1.0258623E38F) ;
        p202.mppt1_pwm_SET((char)44626) ;
        p202.mppt2_status_SET((char)229) ;
        p202.mppt1_amp_SET(1.2970489E38F) ;
        p202.mppt2_pwm_SET((char)6490) ;
        CommunicationChannel.instance.send(p202);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ASLCTRL_DATA.add((src, ph, pack) ->
        {
            assert(pack.SpoilersEngaged_GET() == (char)48);
            assert(pack.uThrot2_GET() == -1.3821116E38F);
            assert(pack.AirspeedRef_GET() == -2.3977792E38F);
            assert(pack.qRef_GET() == -1.4929052E38F);
            assert(pack.RollAngleRef_GET() == -3.1560888E38F);
            assert(pack.h_GET() == 9.329861E37F);
            assert(pack.YawAngleRef_GET() == -1.5070954E38F);
            assert(pack.PitchAngle_GET() == -2.5114588E38F);
            assert(pack.YawAngle_GET() == -2.1532366E37F);
            assert(pack.uRud_GET() == -7.632468E37F);
            assert(pack.nZ_GET() == 2.5617217E36F);
            assert(pack.q_GET() == 1.2092113E38F);
            assert(pack.hRef_GET() == -2.7881874E38F);
            assert(pack.p_GET() == 3.1528318E38F);
            assert(pack.aslctrl_mode_GET() == (char)220);
            assert(pack.uElev_GET() == -3.019223E38F);
            assert(pack.RollAngle_GET() == 8.13793E37F);
            assert(pack.r_GET() == -1.778634E37F);
            assert(pack.hRef_t_GET() == 1.081174E38F);
            assert(pack.timestamp_GET() == 1151952471618727605L);
            assert(pack.uAil_GET() == 3.318116E38F);
            assert(pack.uThrot_GET() == 2.3935172E38F);
            assert(pack.pRef_GET() == -1.4624273E37F);
            assert(pack.rRef_GET() == -2.301036E37F);
            assert(pack.PitchAngleRef_GET() == 2.5208544E38F);
        });
        GroundControl.ASLCTRL_DATA p203 = CommunicationChannel.new_ASLCTRL_DATA();
        PH.setPack(p203);
        p203.PitchAngle_SET(-2.5114588E38F) ;
        p203.rRef_SET(-2.301036E37F) ;
        p203.pRef_SET(-1.4624273E37F) ;
        p203.hRef_t_SET(1.081174E38F) ;
        p203.RollAngle_SET(8.13793E37F) ;
        p203.q_SET(1.2092113E38F) ;
        p203.uElev_SET(-3.019223E38F) ;
        p203.hRef_SET(-2.7881874E38F) ;
        p203.p_SET(3.1528318E38F) ;
        p203.aslctrl_mode_SET((char)220) ;
        p203.nZ_SET(2.5617217E36F) ;
        p203.timestamp_SET(1151952471618727605L) ;
        p203.uThrot_SET(2.3935172E38F) ;
        p203.uRud_SET(-7.632468E37F) ;
        p203.AirspeedRef_SET(-2.3977792E38F) ;
        p203.YawAngle_SET(-2.1532366E37F) ;
        p203.h_SET(9.329861E37F) ;
        p203.RollAngleRef_SET(-3.1560888E38F) ;
        p203.r_SET(-1.778634E37F) ;
        p203.qRef_SET(-1.4929052E38F) ;
        p203.PitchAngleRef_SET(2.5208544E38F) ;
        p203.uThrot2_SET(-1.3821116E38F) ;
        p203.SpoilersEngaged_SET((char)48) ;
        p203.uAil_SET(3.318116E38F) ;
        p203.YawAngleRef_SET(-1.5070954E38F) ;
        CommunicationChannel.instance.send(p203);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ASLCTRL_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.f_8_GET() == -1.3193136E38F);
            assert(pack.i8_2_GET() == (char)187);
            assert(pack.i32_1_GET() == 3032259541L);
            assert(pack.f_4_GET() == -2.9517104E38F);
            assert(pack.f_5_GET() == 1.7199047E38F);
            assert(pack.f_3_GET() == 2.4900061E38F);
            assert(pack.i8_1_GET() == (char)55);
            assert(pack.f_1_GET() == 6.683862E37F);
            assert(pack.f_7_GET() == 3.2623793E38F);
            assert(pack.f_2_GET() == 2.49806E38F);
            assert(pack.f_6_GET() == 2.585794E38F);
        });
        GroundControl.ASLCTRL_DEBUG p204 = CommunicationChannel.new_ASLCTRL_DEBUG();
        PH.setPack(p204);
        p204.f_7_SET(3.2623793E38F) ;
        p204.f_2_SET(2.49806E38F) ;
        p204.f_1_SET(6.683862E37F) ;
        p204.f_8_SET(-1.3193136E38F) ;
        p204.i8_1_SET((char)55) ;
        p204.i32_1_SET(3032259541L) ;
        p204.f_3_SET(2.4900061E38F) ;
        p204.i8_2_SET((char)187) ;
        p204.f_6_SET(2.585794E38F) ;
        p204.f_4_SET(-2.9517104E38F) ;
        p204.f_5_SET(1.7199047E38F) ;
        CommunicationChannel.instance.send(p204);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ASLUAV_STATUS.add((src, ph, pack) ->
        {
            assert(pack.LED_status_GET() == (char)89);
            assert(pack.SATCOM_status_GET() == (char)20);
            assert(Arrays.equals(pack.Servo_status_GET(),  new char[] {(char)122, (char)162, (char)218, (char)33, (char)217, (char)171, (char)109, (char)64}));
            assert(pack.Motor_rpm_GET() == -6.3334937E37F);
        });
        GroundControl.ASLUAV_STATUS p205 = CommunicationChannel.new_ASLUAV_STATUS();
        PH.setPack(p205);
        p205.Servo_status_SET(new char[] {(char)122, (char)162, (char)218, (char)33, (char)217, (char)171, (char)109, (char)64}, 0) ;
        p205.Motor_rpm_SET(-6.3334937E37F) ;
        p205.SATCOM_status_SET((char)20) ;
        p205.LED_status_SET((char)89) ;
        CommunicationChannel.instance.send(p205);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_EKF_EXT.add((src, ph, pack) ->
        {
            assert(pack.WindZ_GET() == 2.988104E38F);
            assert(pack.Windspeed_GET() == -1.1006991E38F);
            assert(pack.beta_GET() == 2.7579653E38F);
            assert(pack.alpha_GET() == 2.2203864E37F);
            assert(pack.WindDir_GET() == 7.49095E37F);
            assert(pack.timestamp_GET() == 5998455690488132184L);
            assert(pack.Airspeed_GET() == -2.688787E38F);
        });
        GroundControl.EKF_EXT p206 = CommunicationChannel.new_EKF_EXT();
        PH.setPack(p206);
        p206.Windspeed_SET(-1.1006991E38F) ;
        p206.WindDir_SET(7.49095E37F) ;
        p206.timestamp_SET(5998455690488132184L) ;
        p206.Airspeed_SET(-2.688787E38F) ;
        p206.alpha_SET(2.2203864E37F) ;
        p206.WindZ_SET(2.988104E38F) ;
        p206.beta_SET(2.7579653E38F) ;
        CommunicationChannel.instance.send(p206);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ASL_OBCTRL.add((src, ph, pack) ->
        {
            assert(pack.timestamp_GET() == 739561269397997220L);
            assert(pack.uThrot_GET() == 1.2305049E38F);
            assert(pack.uAilR_GET() == 1.5744693E38F);
            assert(pack.uRud_GET() == -2.0966243E38F);
            assert(pack.uAilL_GET() == 2.759443E38F);
            assert(pack.uThrot2_GET() == 3.0679343E38F);
            assert(pack.obctrl_status_GET() == (char)106);
            assert(pack.uElev_GET() == 1.5564172E38F);
        });
        GroundControl.ASL_OBCTRL p207 = CommunicationChannel.new_ASL_OBCTRL();
        PH.setPack(p207);
        p207.uRud_SET(-2.0966243E38F) ;
        p207.uAilL_SET(2.759443E38F) ;
        p207.timestamp_SET(739561269397997220L) ;
        p207.uAilR_SET(1.5744693E38F) ;
        p207.uThrot_SET(1.2305049E38F) ;
        p207.obctrl_status_SET((char)106) ;
        p207.uThrot2_SET(3.0679343E38F) ;
        p207.uElev_SET(1.5564172E38F) ;
        CommunicationChannel.instance.send(p207);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENS_ATMOS.add((src, ph, pack) ->
        {
            assert(pack.Humidity_GET() == -2.674131E38F);
            assert(pack.TempAmbient_GET() == -1.676259E38F);
        });
        GroundControl.SENS_ATMOS p208 = CommunicationChannel.new_SENS_ATMOS();
        PH.setPack(p208);
        p208.Humidity_SET(-2.674131E38F) ;
        p208.TempAmbient_SET(-1.676259E38F) ;
        CommunicationChannel.instance.send(p208);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENS_BATMON.add((src, ph, pack) ->
        {
            assert(pack.batterystatus_GET() == (char)21855);
            assert(pack.SoC_GET() == (char)238);
            assert(pack.temperature_GET() == -9.662598E37F);
            assert(pack.hostfetcontrol_GET() == (char)32108);
            assert(pack.voltage_GET() == (char)61082);
            assert(pack.cellvoltage4_GET() == (char)28631);
            assert(pack.cellvoltage2_GET() == (char)60541);
            assert(pack.cellvoltage3_GET() == (char)11719);
            assert(pack.cellvoltage5_GET() == (char)6901);
            assert(pack.cellvoltage6_GET() == (char)18904);
            assert(pack.cellvoltage1_GET() == (char)57840);
            assert(pack.current_GET() == (short)24862);
            assert(pack.serialnumber_GET() == (char)30421);
        });
        GroundControl.SENS_BATMON p209 = CommunicationChannel.new_SENS_BATMON();
        PH.setPack(p209);
        p209.cellvoltage5_SET((char)6901) ;
        p209.serialnumber_SET((char)30421) ;
        p209.cellvoltage6_SET((char)18904) ;
        p209.cellvoltage2_SET((char)60541) ;
        p209.SoC_SET((char)238) ;
        p209.temperature_SET(-9.662598E37F) ;
        p209.batterystatus_SET((char)21855) ;
        p209.voltage_SET((char)61082) ;
        p209.cellvoltage3_SET((char)11719) ;
        p209.current_SET((short)24862) ;
        p209.cellvoltage4_SET((char)28631) ;
        p209.cellvoltage1_SET((char)57840) ;
        p209.hostfetcontrol_SET((char)32108) ;
        CommunicationChannel.instance.send(p209);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FW_SOARING_DATA.add((src, ph, pack) ->
        {
            assert(pack.VarLon_GET() == -6.230929E37F);
            assert(pack.VarR_GET() == 2.6226782E38F);
            assert(pack.ControlMode_GET() == (char)25);
            assert(pack.ThermalGSNorth_GET() == 3.1642916E38F);
            assert(pack.z1_exp_GET() == 2.0857158E38F);
            assert(pack.LoiterRadius_GET() == -5.9068025E37F);
            assert(pack.xW_GET() == -1.9065203E38F);
            assert(pack.xR_GET() == -2.0102894E38F);
            assert(pack.LoiterDirection_GET() == 1.2100127E38F);
            assert(pack.ThermalGSEast_GET() == 1.0612636E38F);
            assert(pack.z2_exp_GET() == 5.82729E37F);
            assert(pack.xLat_GET() == 1.7553227E38F);
            assert(pack.timestamp_GET() == 828600182262587530L);
            assert(pack.VarW_GET() == 2.9515816E38F);
            assert(pack.xLon_GET() == -2.2015298E38F);
            assert(pack.DebugVar2_GET() == -1.4801586E38F);
            assert(pack.timestampModeChanged_GET() == 4500561560838150288L);
            assert(pack.VarLat_GET() == 3.3994093E38F);
            assert(pack.z2_DeltaRoll_GET() == -3.2070447E38F);
            assert(pack.z1_LocalUpdraftSpeed_GET() == -7.115891E37F);
            assert(pack.TSE_dot_GET() == 2.894894E38F);
            assert(pack.vSinkExp_GET() == -2.2936973E38F);
            assert(pack.DistToSoarPoint_GET() == 7.709181E37F);
            assert(pack.valid_GET() == (char)241);
            assert(pack.DebugVar1_GET() == -2.377315E37F);
        });
        GroundControl.FW_SOARING_DATA p210 = CommunicationChannel.new_FW_SOARING_DATA();
        PH.setPack(p210);
        p210.z1_exp_SET(2.0857158E38F) ;
        p210.valid_SET((char)241) ;
        p210.z2_exp_SET(5.82729E37F) ;
        p210.ThermalGSNorth_SET(3.1642916E38F) ;
        p210.VarW_SET(2.9515816E38F) ;
        p210.TSE_dot_SET(2.894894E38F) ;
        p210.VarR_SET(2.6226782E38F) ;
        p210.xLat_SET(1.7553227E38F) ;
        p210.vSinkExp_SET(-2.2936973E38F) ;
        p210.VarLat_SET(3.3994093E38F) ;
        p210.timestamp_SET(828600182262587530L) ;
        p210.LoiterRadius_SET(-5.9068025E37F) ;
        p210.ControlMode_SET((char)25) ;
        p210.xR_SET(-2.0102894E38F) ;
        p210.DebugVar2_SET(-1.4801586E38F) ;
        p210.DistToSoarPoint_SET(7.709181E37F) ;
        p210.LoiterDirection_SET(1.2100127E38F) ;
        p210.xLon_SET(-2.2015298E38F) ;
        p210.DebugVar1_SET(-2.377315E37F) ;
        p210.z2_DeltaRoll_SET(-3.2070447E38F) ;
        p210.VarLon_SET(-6.230929E37F) ;
        p210.ThermalGSEast_SET(1.0612636E38F) ;
        p210.timestampModeChanged_SET(4500561560838150288L) ;
        p210.z1_LocalUpdraftSpeed_SET(-7.115891E37F) ;
        p210.xW_SET(-1.9065203E38F) ;
        CommunicationChannel.instance.send(p210);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENSORPOD_STATUS.add((src, ph, pack) ->
        {
            assert(pack.timestamp_GET() == 4279226302298126061L);
            assert(pack.visensor_rate_4_GET() == (char)116);
            assert(pack.recording_nodes_count_GET() == (char)19);
            assert(pack.visensor_rate_3_GET() == (char)206);
            assert(pack.visensor_rate_1_GET() == (char)31);
            assert(pack.free_space_GET() == (char)23305);
            assert(pack.cpu_temp_GET() == (char)118);
            assert(pack.visensor_rate_2_GET() == (char)124);
        });
        GroundControl.SENSORPOD_STATUS p211 = CommunicationChannel.new_SENSORPOD_STATUS();
        PH.setPack(p211);
        p211.visensor_rate_2_SET((char)124) ;
        p211.timestamp_SET(4279226302298126061L) ;
        p211.cpu_temp_SET((char)118) ;
        p211.visensor_rate_4_SET((char)116) ;
        p211.free_space_SET((char)23305) ;
        p211.visensor_rate_1_SET((char)31) ;
        p211.visensor_rate_3_SET((char)206) ;
        p211.recording_nodes_count_SET((char)19) ;
        CommunicationChannel.instance.send(p211);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SENS_POWER_BOARD.add((src, ph, pack) ->
        {
            assert(pack.pwr_brd_servo_volt_GET() == -6.6311805E35F);
            assert(pack.pwr_brd_status_GET() == (char)7);
            assert(pack.pwr_brd_mot_r_amp_GET() == -1.9373403E38F);
            assert(pack.pwr_brd_mot_l_amp_GET() == -9.708404E37F);
            assert(pack.pwr_brd_servo_2_amp_GET() == 1.3480455E38F);
            assert(pack.pwr_brd_led_status_GET() == (char)131);
            assert(pack.timestamp_GET() == 3938321839698489841L);
            assert(pack.pwr_brd_system_volt_GET() == 3.2238894E38F);
            assert(pack.pwr_brd_aux_amp_GET() == 2.0028048E38F);
            assert(pack.pwr_brd_servo_3_amp_GET() == -1.4565645E38F);
            assert(pack.pwr_brd_servo_1_amp_GET() == 5.6062157E37F);
            assert(pack.pwr_brd_servo_4_amp_GET() == -1.1641482E38F);
        });
        GroundControl.SENS_POWER_BOARD p212 = CommunicationChannel.new_SENS_POWER_BOARD();
        PH.setPack(p212);
        p212.pwr_brd_servo_2_amp_SET(1.3480455E38F) ;
        p212.pwr_brd_led_status_SET((char)131) ;
        p212.pwr_brd_servo_volt_SET(-6.6311805E35F) ;
        p212.pwr_brd_servo_3_amp_SET(-1.4565645E38F) ;
        p212.pwr_brd_mot_l_amp_SET(-9.708404E37F) ;
        p212.pwr_brd_status_SET((char)7) ;
        p212.pwr_brd_mot_r_amp_SET(-1.9373403E38F) ;
        p212.timestamp_SET(3938321839698489841L) ;
        p212.pwr_brd_system_volt_SET(3.2238894E38F) ;
        p212.pwr_brd_servo_1_amp_SET(5.6062157E37F) ;
        p212.pwr_brd_aux_amp_SET(2.0028048E38F) ;
        p212.pwr_brd_servo_4_amp_SET(-1.1641482E38F) ;
        CommunicationChannel.instance.send(p212);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 9166877949849177102L);
            assert(pack.pos_horiz_ratio_GET() == 3.2940706E38F);
            assert(pack.pos_vert_ratio_GET() == -1.2097129E38F);
            assert(pack.pos_vert_accuracy_GET() == -1.6568418E38F);
            assert(pack.mag_ratio_GET() == 2.5124731E38F);
            assert(pack.tas_ratio_GET() == -2.55062E38F);
            assert(pack.vel_ratio_GET() == -2.1563284E38F);
            assert(pack.hagl_ratio_GET() == 7.1270116E37F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE));
            assert(pack.pos_horiz_accuracy_GET() == -1.7429516E38F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.vel_ratio_SET(-2.1563284E38F) ;
        p230.tas_ratio_SET(-2.55062E38F) ;
        p230.pos_vert_accuracy_SET(-1.6568418E38F) ;
        p230.hagl_ratio_SET(7.1270116E37F) ;
        p230.pos_vert_ratio_SET(-1.2097129E38F) ;
        p230.mag_ratio_SET(2.5124731E38F) ;
        p230.time_usec_SET(9166877949849177102L) ;
        p230.pos_horiz_accuracy_SET(-1.7429516E38F) ;
        p230.pos_horiz_ratio_SET(3.2940706E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE)) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.var_horiz_GET() == -3.091521E38F);
            assert(pack.wind_x_GET() == 2.5988973E38F);
            assert(pack.wind_y_GET() == -2.5417475E37F);
            assert(pack.horiz_accuracy_GET() == 3.077799E38F);
            assert(pack.vert_accuracy_GET() == -6.036475E36F);
            assert(pack.wind_alt_GET() == 3.2371967E38F);
            assert(pack.time_usec_GET() == 2583792852719812480L);
            assert(pack.var_vert_GET() == -7.0357326E37F);
            assert(pack.wind_z_GET() == 1.7870387E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_y_SET(-2.5417475E37F) ;
        p231.time_usec_SET(2583792852719812480L) ;
        p231.var_vert_SET(-7.0357326E37F) ;
        p231.wind_z_SET(1.7870387E38F) ;
        p231.wind_alt_SET(3.2371967E38F) ;
        p231.var_horiz_SET(-3.091521E38F) ;
        p231.horiz_accuracy_SET(3.077799E38F) ;
        p231.wind_x_SET(2.5988973E38F) ;
        p231.vert_accuracy_SET(-6.036475E36F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.time_week_ms_GET() == 3086500856L);
            assert(pack.time_week_GET() == (char)27806);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY));
            assert(pack.speed_accuracy_GET() == -6.257029E37F);
            assert(pack.vdop_GET() == -5.517078E37F);
            assert(pack.alt_GET() == 3.3763888E38F);
            assert(pack.satellites_visible_GET() == (char)192);
            assert(pack.horiz_accuracy_GET() == 5.688596E37F);
            assert(pack.time_usec_GET() == 3367050751895503929L);
            assert(pack.hdop_GET() == 5.342879E37F);
            assert(pack.vn_GET() == -2.4133208E38F);
            assert(pack.lon_GET() == 1107161920);
            assert(pack.fix_type_GET() == (char)90);
            assert(pack.vd_GET() == -1.411241E38F);
            assert(pack.vert_accuracy_GET() == -2.8315692E38F);
            assert(pack.lat_GET() == 1437230054);
            assert(pack.gps_id_GET() == (char)183);
            assert(pack.ve_GET() == -3.1680254E38F);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.lat_SET(1437230054) ;
        p232.horiz_accuracy_SET(5.688596E37F) ;
        p232.vn_SET(-2.4133208E38F) ;
        p232.time_usec_SET(3367050751895503929L) ;
        p232.vd_SET(-1.411241E38F) ;
        p232.satellites_visible_SET((char)192) ;
        p232.alt_SET(3.3763888E38F) ;
        p232.vdop_SET(-5.517078E37F) ;
        p232.time_week_SET((char)27806) ;
        p232.vert_accuracy_SET(-2.8315692E38F) ;
        p232.hdop_SET(5.342879E37F) ;
        p232.ve_SET(-3.1680254E38F) ;
        p232.speed_accuracy_SET(-6.257029E37F) ;
        p232.time_week_ms_SET(3086500856L) ;
        p232.fix_type_SET((char)90) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY)) ;
        p232.lon_SET(1107161920) ;
        p232.gps_id_SET((char)183) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (char)1);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)57, (char)10, (char)108, (char)74, (char)249, (char)240, (char)172, (char)225, (char)122, (char)34, (char)89, (char)169, (char)20, (char)127, (char)5, (char)253, (char)75, (char)198, (char)133, (char)198, (char)142, (char)52, (char)88, (char)220, (char)26, (char)198, (char)101, (char)130, (char)155, (char)158, (char)246, (char)195, (char)17, (char)55, (char)72, (char)33, (char)44, (char)200, (char)244, (char)27, (char)152, (char)163, (char)216, (char)29, (char)18, (char)226, (char)238, (char)236, (char)9, (char)83, (char)240, (char)195, (char)23, (char)17, (char)200, (char)34, (char)128, (char)78, (char)26, (char)191, (char)234, (char)0, (char)84, (char)28, (char)128, (char)2, (char)247, (char)166, (char)185, (char)109, (char)9, (char)185, (char)254, (char)150, (char)193, (char)10, (char)45, (char)181, (char)119, (char)141, (char)86, (char)128, (char)181, (char)93, (char)67, (char)48, (char)198, (char)183, (char)62, (char)59, (char)249, (char)4, (char)47, (char)120, (char)35, (char)14, (char)70, (char)191, (char)178, (char)179, (char)34, (char)43, (char)48, (char)219, (char)84, (char)179, (char)58, (char)185, (char)131, (char)113, (char)56, (char)177, (char)155, (char)27, (char)24, (char)112, (char)208, (char)76, (char)110, (char)56, (char)122, (char)22, (char)209, (char)238, (char)28, (char)34, (char)161, (char)203, (char)20, (char)35, (char)5, (char)211, (char)82, (char)232, (char)92, (char)41, (char)223, (char)147, (char)87, (char)41, (char)52, (char)138, (char)16, (char)43, (char)125, (char)140, (char)140, (char)124, (char)201, (char)117, (char)25, (char)158, (char)127, (char)49, (char)220, (char)99, (char)150, (char)218, (char)161, (char)205, (char)119, (char)60, (char)150, (char)109, (char)5, (char)95, (char)135, (char)129, (char)187, (char)139, (char)237, (char)64, (char)0, (char)168, (char)9, (char)54, (char)144, (char)152, (char)48, (char)100}));
            assert(pack.len_GET() == (char)222);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.data__SET(new char[] {(char)57, (char)10, (char)108, (char)74, (char)249, (char)240, (char)172, (char)225, (char)122, (char)34, (char)89, (char)169, (char)20, (char)127, (char)5, (char)253, (char)75, (char)198, (char)133, (char)198, (char)142, (char)52, (char)88, (char)220, (char)26, (char)198, (char)101, (char)130, (char)155, (char)158, (char)246, (char)195, (char)17, (char)55, (char)72, (char)33, (char)44, (char)200, (char)244, (char)27, (char)152, (char)163, (char)216, (char)29, (char)18, (char)226, (char)238, (char)236, (char)9, (char)83, (char)240, (char)195, (char)23, (char)17, (char)200, (char)34, (char)128, (char)78, (char)26, (char)191, (char)234, (char)0, (char)84, (char)28, (char)128, (char)2, (char)247, (char)166, (char)185, (char)109, (char)9, (char)185, (char)254, (char)150, (char)193, (char)10, (char)45, (char)181, (char)119, (char)141, (char)86, (char)128, (char)181, (char)93, (char)67, (char)48, (char)198, (char)183, (char)62, (char)59, (char)249, (char)4, (char)47, (char)120, (char)35, (char)14, (char)70, (char)191, (char)178, (char)179, (char)34, (char)43, (char)48, (char)219, (char)84, (char)179, (char)58, (char)185, (char)131, (char)113, (char)56, (char)177, (char)155, (char)27, (char)24, (char)112, (char)208, (char)76, (char)110, (char)56, (char)122, (char)22, (char)209, (char)238, (char)28, (char)34, (char)161, (char)203, (char)20, (char)35, (char)5, (char)211, (char)82, (char)232, (char)92, (char)41, (char)223, (char)147, (char)87, (char)41, (char)52, (char)138, (char)16, (char)43, (char)125, (char)140, (char)140, (char)124, (char)201, (char)117, (char)25, (char)158, (char)127, (char)49, (char)220, (char)99, (char)150, (char)218, (char)161, (char)205, (char)119, (char)60, (char)150, (char)109, (char)5, (char)95, (char)135, (char)129, (char)187, (char)139, (char)237, (char)64, (char)0, (char)168, (char)9, (char)54, (char)144, (char)152, (char)48, (char)100}, 0) ;
        p233.len_SET((char)222) ;
        p233.flags_SET((char)1) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 1964766281);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
            assert(pack.groundspeed_GET() == (char)120);
            assert(pack.longitude_GET() == 1731547120);
            assert(pack.custom_mode_GET() == 2133687406L);
            assert(pack.battery_remaining_GET() == (char)147);
            assert(pack.airspeed_GET() == (char)198);
            assert(pack.wp_distance_GET() == (char)35714);
            assert(pack.wp_num_GET() == (char)44);
            assert(pack.airspeed_sp_GET() == (char)9);
            assert(pack.failsafe_GET() == (char)145);
            assert(pack.temperature_GET() == (byte)70);
            assert(pack.heading_GET() == (char)14635);
            assert(pack.heading_sp_GET() == (short) -11277);
            assert(pack.altitude_sp_GET() == (short)1102);
            assert(pack.roll_GET() == (short)25950);
            assert(pack.gps_nsat_GET() == (char)53);
            assert(pack.temperature_air_GET() == (byte) - 61);
            assert(pack.pitch_GET() == (short)6444);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            assert(pack.climb_rate_GET() == (byte) - 103);
            assert(pack.altitude_amsl_GET() == (short)15176);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED));
            assert(pack.throttle_GET() == (byte)57);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.pitch_SET((short)6444) ;
        p234.failsafe_SET((char)145) ;
        p234.throttle_SET((byte)57) ;
        p234.altitude_amsl_SET((short)15176) ;
        p234.roll_SET((short)25950) ;
        p234.gps_nsat_SET((char)53) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED)) ;
        p234.longitude_SET(1731547120) ;
        p234.wp_num_SET((char)44) ;
        p234.airspeed_SET((char)198) ;
        p234.wp_distance_SET((char)35714) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING) ;
        p234.climb_rate_SET((byte) - 103) ;
        p234.latitude_SET(1964766281) ;
        p234.custom_mode_SET(2133687406L) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS) ;
        p234.heading_sp_SET((short) -11277) ;
        p234.altitude_sp_SET((short)1102) ;
        p234.groundspeed_SET((char)120) ;
        p234.battery_remaining_SET((char)147) ;
        p234.heading_SET((char)14635) ;
        p234.temperature_SET((byte)70) ;
        p234.airspeed_sp_SET((char)9) ;
        p234.temperature_air_SET((byte) - 61) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.clipping_2_GET() == 1373751264L);
            assert(pack.clipping_0_GET() == 1325930036L);
            assert(pack.vibration_z_GET() == -1.6003767E38F);
            assert(pack.vibration_x_GET() == 1.6762801E38F);
            assert(pack.time_usec_GET() == 7344293563737905200L);
            assert(pack.clipping_1_GET() == 2589707361L);
            assert(pack.vibration_y_GET() == -1.3778508E38F);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(7344293563737905200L) ;
        p241.vibration_y_SET(-1.3778508E38F) ;
        p241.clipping_0_SET(1325930036L) ;
        p241.vibration_z_SET(-1.6003767E38F) ;
        p241.clipping_1_SET(2589707361L) ;
        p241.vibration_x_SET(1.6762801E38F) ;
        p241.clipping_2_SET(1373751264L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_z_GET() == 1.1708974E38F);
            assert(pack.altitude_GET() == -900912876);
            assert(pack.time_usec_TRY(ph) == 665947463453724051L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.7832317E38F, -2.3213897E38F, -2.7322789E38F, 1.1852942E38F}));
            assert(pack.z_GET() == -1.7707032E38F);
            assert(pack.approach_x_GET() == -2.9929058E38F);
            assert(pack.y_GET() == -2.708774E37F);
            assert(pack.approach_y_GET() == 5.0214023E37F);
            assert(pack.longitude_GET() == 1799783127);
            assert(pack.latitude_GET() == 135609814);
            assert(pack.x_GET() == 1.6685875E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(135609814) ;
        p242.time_usec_SET(665947463453724051L, PH) ;
        p242.q_SET(new float[] {1.7832317E38F, -2.3213897E38F, -2.7322789E38F, 1.1852942E38F}, 0) ;
        p242.x_SET(1.6685875E38F) ;
        p242.z_SET(-1.7707032E38F) ;
        p242.approach_y_SET(5.0214023E37F) ;
        p242.y_SET(-2.708774E37F) ;
        p242.approach_z_SET(1.1708974E38F) ;
        p242.altitude_SET(-900912876) ;
        p242.approach_x_SET(-2.9929058E38F) ;
        p242.longitude_SET(1799783127) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 8481536367370328681L);
            assert(pack.longitude_GET() == 2046203548);
            assert(pack.approach_x_GET() == -7.100262E37F);
            assert(pack.x_GET() == -6.7587626E37F);
            assert(pack.y_GET() == -1.8362732E38F);
            assert(pack.z_GET() == 2.3561227E38F);
            assert(pack.latitude_GET() == 1290133855);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.2045278E38F, -1.0796627E38F, -1.4600804E38F, -2.6960174E38F}));
            assert(pack.target_system_GET() == (char)185);
            assert(pack.approach_z_GET() == -2.8831989E38F);
            assert(pack.altitude_GET() == -479852727);
            assert(pack.approach_y_GET() == -3.2676923E38F);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.approach_y_SET(-3.2676923E38F) ;
        p243.x_SET(-6.7587626E37F) ;
        p243.altitude_SET(-479852727) ;
        p243.latitude_SET(1290133855) ;
        p243.approach_z_SET(-2.8831989E38F) ;
        p243.y_SET(-1.8362732E38F) ;
        p243.longitude_SET(2046203548) ;
        p243.approach_x_SET(-7.100262E37F) ;
        p243.time_usec_SET(8481536367370328681L, PH) ;
        p243.z_SET(2.3561227E38F) ;
        p243.target_system_SET((char)185) ;
        p243.q_SET(new float[] {-2.2045278E38F, -1.0796627E38F, -1.4600804E38F, -2.6960174E38F}, 0) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == 1315901430);
            assert(pack.message_id_GET() == (char)8564);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(1315901430) ;
        p244.message_id_SET((char)8564) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_MC) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.ver_velocity_GET() == (short) -812);
            assert(pack.lat_GET() == 682378355);
            assert(pack.altitude_GET() == -1829038189);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO);
            assert(pack.tslc_GET() == (char)51);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.callsign_LEN(ph) == 3);
            assert(pack.callsign_TRY(ph).equals("mAe"));
            assert(pack.lon_GET() == 1520758326);
            assert(pack.heading_GET() == (char)30258);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN));
            assert(pack.ICAO_address_GET() == 608058750L);
            assert(pack.squawk_GET() == (char)1016);
            assert(pack.hor_velocity_GET() == (char)3167);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.squawk_SET((char)1016) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN)) ;
        p246.lat_SET(682378355) ;
        p246.altitude_SET(-1829038189) ;
        p246.heading_SET((char)30258) ;
        p246.callsign_SET("mAe", PH) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.ICAO_address_SET(608058750L) ;
        p246.ver_velocity_SET((short) -812) ;
        p246.tslc_SET((char)51) ;
        p246.hor_velocity_SET((char)3167) ;
        p246.lon_SET(1520758326) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.altitude_minimum_delta_GET() == 1.4309824E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE);
            assert(pack.id_GET() == 580164833L);
            assert(pack.threat_level_GET() == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW));
            assert(pack.time_to_minimum_delta_GET() == 1.0179701E38F);
            assert(pack.horizontal_minimum_delta_GET() == 4.91181E37F);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE) ;
        p247.time_to_minimum_delta_SET(1.0179701E38F) ;
        p247.altitude_minimum_delta_SET(1.4309824E38F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.horizontal_minimum_delta_SET(4.91181E37F) ;
        p247.threat_level_SET((MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW)) ;
        p247.id_SET(580164833L) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.message_type_GET() == (char)36313);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)17, (char)34, (char)225, (char)0, (char)165, (char)62, (char)169, (char)23, (char)250, (char)68, (char)129, (char)13, (char)170, (char)92, (char)182, (char)170, (char)139, (char)2, (char)22, (char)60, (char)189, (char)190, (char)120, (char)26, (char)233, (char)5, (char)189, (char)139, (char)85, (char)188, (char)129, (char)89, (char)183, (char)236, (char)115, (char)2, (char)187, (char)194, (char)52, (char)3, (char)132, (char)219, (char)189, (char)12, (char)247, (char)166, (char)230, (char)19, (char)82, (char)4, (char)103, (char)220, (char)189, (char)23, (char)105, (char)144, (char)112, (char)113, (char)214, (char)6, (char)47, (char)128, (char)190, (char)90, (char)224, (char)222, (char)180, (char)97, (char)127, (char)80, (char)64, (char)156, (char)221, (char)63, (char)226, (char)129, (char)15, (char)33, (char)213, (char)6, (char)183, (char)36, (char)244, (char)222, (char)33, (char)12, (char)200, (char)77, (char)118, (char)30, (char)163, (char)173, (char)101, (char)48, (char)26, (char)145, (char)244, (char)186, (char)186, (char)178, (char)111, (char)68, (char)146, (char)254, (char)227, (char)96, (char)15, (char)6, (char)50, (char)249, (char)242, (char)134, (char)245, (char)106, (char)56, (char)165, (char)108, (char)122, (char)104, (char)90, (char)125, (char)151, (char)0, (char)15, (char)190, (char)138, (char)206, (char)165, (char)56, (char)0, (char)114, (char)123, (char)145, (char)52, (char)31, (char)190, (char)221, (char)212, (char)93, (char)127, (char)79, (char)48, (char)202, (char)147, (char)145, (char)241, (char)229, (char)33, (char)31, (char)213, (char)47, (char)237, (char)43, (char)175, (char)27, (char)85, (char)97, (char)74, (char)138, (char)36, (char)239, (char)68, (char)207, (char)63, (char)7, (char)33, (char)252, (char)43, (char)62, (char)122, (char)95, (char)238, (char)196, (char)255, (char)159, (char)194, (char)126, (char)165, (char)30, (char)230, (char)239, (char)156, (char)103, (char)167, (char)184, (char)45, (char)233, (char)151, (char)161, (char)99, (char)88, (char)66, (char)132, (char)227, (char)122, (char)44, (char)64, (char)211, (char)36, (char)41, (char)103, (char)189, (char)250, (char)172, (char)146, (char)185, (char)242, (char)44, (char)197, (char)111, (char)15, (char)211, (char)176, (char)242, (char)46, (char)172, (char)87, (char)103, (char)122, (char)109, (char)59, (char)76, (char)236, (char)131, (char)46, (char)68, (char)228, (char)160, (char)98, (char)123, (char)236, (char)133, (char)31, (char)78, (char)211, (char)204, (char)8, (char)126, (char)22, (char)8, (char)148, (char)202, (char)226, (char)49, (char)47, (char)211, (char)20, (char)213, (char)53}));
            assert(pack.target_network_GET() == (char)187);
            assert(pack.target_component_GET() == (char)155);
            assert(pack.target_system_GET() == (char)161);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.message_type_SET((char)36313) ;
        p248.target_network_SET((char)187) ;
        p248.payload_SET(new char[] {(char)17, (char)34, (char)225, (char)0, (char)165, (char)62, (char)169, (char)23, (char)250, (char)68, (char)129, (char)13, (char)170, (char)92, (char)182, (char)170, (char)139, (char)2, (char)22, (char)60, (char)189, (char)190, (char)120, (char)26, (char)233, (char)5, (char)189, (char)139, (char)85, (char)188, (char)129, (char)89, (char)183, (char)236, (char)115, (char)2, (char)187, (char)194, (char)52, (char)3, (char)132, (char)219, (char)189, (char)12, (char)247, (char)166, (char)230, (char)19, (char)82, (char)4, (char)103, (char)220, (char)189, (char)23, (char)105, (char)144, (char)112, (char)113, (char)214, (char)6, (char)47, (char)128, (char)190, (char)90, (char)224, (char)222, (char)180, (char)97, (char)127, (char)80, (char)64, (char)156, (char)221, (char)63, (char)226, (char)129, (char)15, (char)33, (char)213, (char)6, (char)183, (char)36, (char)244, (char)222, (char)33, (char)12, (char)200, (char)77, (char)118, (char)30, (char)163, (char)173, (char)101, (char)48, (char)26, (char)145, (char)244, (char)186, (char)186, (char)178, (char)111, (char)68, (char)146, (char)254, (char)227, (char)96, (char)15, (char)6, (char)50, (char)249, (char)242, (char)134, (char)245, (char)106, (char)56, (char)165, (char)108, (char)122, (char)104, (char)90, (char)125, (char)151, (char)0, (char)15, (char)190, (char)138, (char)206, (char)165, (char)56, (char)0, (char)114, (char)123, (char)145, (char)52, (char)31, (char)190, (char)221, (char)212, (char)93, (char)127, (char)79, (char)48, (char)202, (char)147, (char)145, (char)241, (char)229, (char)33, (char)31, (char)213, (char)47, (char)237, (char)43, (char)175, (char)27, (char)85, (char)97, (char)74, (char)138, (char)36, (char)239, (char)68, (char)207, (char)63, (char)7, (char)33, (char)252, (char)43, (char)62, (char)122, (char)95, (char)238, (char)196, (char)255, (char)159, (char)194, (char)126, (char)165, (char)30, (char)230, (char)239, (char)156, (char)103, (char)167, (char)184, (char)45, (char)233, (char)151, (char)161, (char)99, (char)88, (char)66, (char)132, (char)227, (char)122, (char)44, (char)64, (char)211, (char)36, (char)41, (char)103, (char)189, (char)250, (char)172, (char)146, (char)185, (char)242, (char)44, (char)197, (char)111, (char)15, (char)211, (char)176, (char)242, (char)46, (char)172, (char)87, (char)103, (char)122, (char)109, (char)59, (char)76, (char)236, (char)131, (char)46, (char)68, (char)228, (char)160, (char)98, (char)123, (char)236, (char)133, (char)31, (char)78, (char)211, (char)204, (char)8, (char)126, (char)22, (char)8, (char)148, (char)202, (char)226, (char)49, (char)47, (char)211, (char)20, (char)213, (char)53}, 0) ;
        p248.target_system_SET((char)161) ;
        p248.target_component_SET((char)155) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)206);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)89, (byte)29, (byte) - 11, (byte)17, (byte) - 79, (byte) - 12, (byte) - 102, (byte)60, (byte)7, (byte)33, (byte) - 69, (byte) - 65, (byte)56, (byte) - 38, (byte)22, (byte)43, (byte)56, (byte) - 61, (byte)75, (byte) - 34, (byte)6, (byte)89, (byte) - 66, (byte)106, (byte)55, (byte)27, (byte)50, (byte)77, (byte) - 27, (byte) - 34, (byte)48, (byte) - 21}));
            assert(pack.ver_GET() == (char)18);
            assert(pack.address_GET() == (char)17002);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.value_SET(new byte[] {(byte)89, (byte)29, (byte) - 11, (byte)17, (byte) - 79, (byte) - 12, (byte) - 102, (byte)60, (byte)7, (byte)33, (byte) - 69, (byte) - 65, (byte)56, (byte) - 38, (byte)22, (byte)43, (byte)56, (byte) - 61, (byte)75, (byte) - 34, (byte)6, (byte)89, (byte) - 66, (byte)106, (byte)55, (byte)27, (byte)50, (byte)77, (byte) - 27, (byte) - 34, (byte)48, (byte) - 21}, 0) ;
        p249.address_SET((char)17002) ;
        p249.type_SET((char)206) ;
        p249.ver_SET((char)18) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 2);
            assert(pack.name_TRY(ph).equals("av"));
            assert(pack.time_usec_GET() == 9209539116682593317L);
            assert(pack.y_GET() == 4.6928603E37F);
            assert(pack.x_GET() == 1.7333084E38F);
            assert(pack.z_GET() == 2.8430547E38F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.time_usec_SET(9209539116682593317L) ;
        p250.y_SET(4.6928603E37F) ;
        p250.z_SET(2.8430547E38F) ;
        p250.name_SET("av", PH) ;
        p250.x_SET(1.7333084E38F) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2041585442L);
            assert(pack.name_LEN(ph) == 5);
            assert(pack.name_TRY(ph).equals("Dkqzf"));
            assert(pack.value_GET() == -6.6988723E36F);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(-6.6988723E36F) ;
        p251.time_boot_ms_SET(2041585442L) ;
        p251.name_SET("Dkqzf", PH) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 1241726497);
            assert(pack.time_boot_ms_GET() == 1343150517L);
            assert(pack.name_LEN(ph) == 9);
            assert(pack.name_TRY(ph).equals("biRcbLydB"));
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.value_SET(1241726497) ;
        p252.name_SET("biRcbLydB", PH) ;
        p252.time_boot_ms_SET(1343150517L) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 11);
            assert(pack.text_TRY(ph).equals("dxKiiuDqysn"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_INFO);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("dxKiiuDqysn", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_INFO) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -3.7070137E37F);
            assert(pack.time_boot_ms_GET() == 2774187242L);
            assert(pack.ind_GET() == (char)164);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.value_SET(-3.7070137E37F) ;
        p254.time_boot_ms_SET(2774187242L) ;
        p254.ind_SET((char)164) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)54);
            assert(pack.initial_timestamp_GET() == 8586926919770000516L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)10, (char)234, (char)3, (char)39, (char)120, (char)196, (char)23, (char)221, (char)29, (char)63, (char)159, (char)130, (char)11, (char)128, (char)222, (char)88, (char)79, (char)97, (char)90, (char)56, (char)140, (char)26, (char)178, (char)223, (char)47, (char)88, (char)175, (char)174, (char)121, (char)158, (char)196, (char)30}));
            assert(pack.target_system_GET() == (char)68);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.initial_timestamp_SET(8586926919770000516L) ;
        p256.secret_key_SET(new char[] {(char)10, (char)234, (char)3, (char)39, (char)120, (char)196, (char)23, (char)221, (char)29, (char)63, (char)159, (char)130, (char)11, (char)128, (char)222, (char)88, (char)79, (char)97, (char)90, (char)56, (char)140, (char)26, (char)178, (char)223, (char)47, (char)88, (char)175, (char)174, (char)121, (char)158, (char)196, (char)30}, 0) ;
        p256.target_component_SET((char)54) ;
        p256.target_system_SET((char)68) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.state_GET() == (char)228);
            assert(pack.time_boot_ms_GET() == 3411181389L);
            assert(pack.last_change_ms_GET() == 3125858564L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(3411181389L) ;
        p257.state_SET((char)228) ;
        p257.last_change_ms_SET(3125858564L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)57);
            assert(pack.target_component_GET() == (char)156);
            assert(pack.tune_LEN(ph) == 2);
            assert(pack.tune_TRY(ph).equals("yj"));
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)57) ;
        p258.target_component_SET((char)156) ;
        p258.tune_SET("yj", PH) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.sensor_size_v_GET() == 2.0032559E38F);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)174, (char)102, (char)9, (char)54, (char)88, (char)162, (char)37, (char)161, (char)39, (char)153, (char)88, (char)118, (char)1, (char)69, (char)105, (char)237, (char)148, (char)145, (char)104, (char)201, (char)47, (char)67, (char)237, (char)76, (char)187, (char)30, (char)17, (char)92, (char)214, (char)224, (char)198, (char)77}));
            assert(pack.resolution_h_GET() == (char)58149);
            assert(pack.time_boot_ms_GET() == 2168026199L);
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO));
            assert(pack.focal_length_GET() == -3.2104596E38F);
            assert(pack.cam_definition_version_GET() == (char)28340);
            assert(pack.sensor_size_h_GET() == 1.4557516E38F);
            assert(pack.resolution_v_GET() == (char)25720);
            assert(pack.lens_id_GET() == (char)122);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)47, (char)56, (char)128, (char)187, (char)199, (char)223, (char)112, (char)192, (char)56, (char)60, (char)206, (char)96, (char)35, (char)18, (char)179, (char)218, (char)96, (char)195, (char)91, (char)193, (char)134, (char)174, (char)58, (char)78, (char)222, (char)25, (char)127, (char)240, (char)253, (char)48, (char)145, (char)229}));
            assert(pack.firmware_version_GET() == 3839346789L);
            assert(pack.cam_definition_uri_LEN(ph) == 74);
            assert(pack.cam_definition_uri_TRY(ph).equals("wJtuzhuejxddKeqtmihqhgyXsdmeapRvFmYiutguadzolAzicymjtobovncIrfhwfhionxeilw"));
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.resolution_h_SET((char)58149) ;
        p259.resolution_v_SET((char)25720) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO)) ;
        p259.focal_length_SET(-3.2104596E38F) ;
        p259.firmware_version_SET(3839346789L) ;
        p259.cam_definition_version_SET((char)28340) ;
        p259.lens_id_SET((char)122) ;
        p259.sensor_size_h_SET(1.4557516E38F) ;
        p259.cam_definition_uri_SET("wJtuzhuejxddKeqtmihqhgyXsdmeapRvFmYiutguadzolAzicymjtobovncIrfhwfhionxeilw", PH) ;
        p259.time_boot_ms_SET(2168026199L) ;
        p259.vendor_name_SET(new char[] {(char)47, (char)56, (char)128, (char)187, (char)199, (char)223, (char)112, (char)192, (char)56, (char)60, (char)206, (char)96, (char)35, (char)18, (char)179, (char)218, (char)96, (char)195, (char)91, (char)193, (char)134, (char)174, (char)58, (char)78, (char)222, (char)25, (char)127, (char)240, (char)253, (char)48, (char)145, (char)229}, 0) ;
        p259.sensor_size_v_SET(2.0032559E38F) ;
        p259.model_name_SET(new char[] {(char)174, (char)102, (char)9, (char)54, (char)88, (char)162, (char)37, (char)161, (char)39, (char)153, (char)88, (char)118, (char)1, (char)69, (char)105, (char)237, (char)148, (char)145, (char)104, (char)201, (char)47, (char)67, (char)237, (char)76, (char)187, (char)30, (char)17, (char)92, (char)214, (char)224, (char)198, (char)77}, 0) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == (CAMERA_MODE.CAMERA_MODE_VIDEO |
                                          CAMERA_MODE.CAMERA_MODE_IMAGE));
            assert(pack.time_boot_ms_GET() == 1310608546L);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(1310608546L) ;
        p260.mode_id_SET((CAMERA_MODE.CAMERA_MODE_VIDEO |
                          CAMERA_MODE.CAMERA_MODE_IMAGE)) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.available_capacity_GET() == 2.295215E38F);
            assert(pack.time_boot_ms_GET() == 1503456496L);
            assert(pack.total_capacity_GET() == -2.7694066E37F);
            assert(pack.used_capacity_GET() == 3.0129988E37F);
            assert(pack.storage_count_GET() == (char)154);
            assert(pack.status_GET() == (char)184);
            assert(pack.storage_id_GET() == (char)39);
            assert(pack.read_speed_GET() == 1.8732852E38F);
            assert(pack.write_speed_GET() == -2.3173036E38F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.read_speed_SET(1.8732852E38F) ;
        p261.storage_id_SET((char)39) ;
        p261.available_capacity_SET(2.295215E38F) ;
        p261.time_boot_ms_SET(1503456496L) ;
        p261.total_capacity_SET(-2.7694066E37F) ;
        p261.used_capacity_SET(3.0129988E37F) ;
        p261.status_SET((char)184) ;
        p261.storage_count_SET((char)154) ;
        p261.write_speed_SET(-2.3173036E38F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.recording_time_ms_GET() == 3087309060L);
            assert(pack.image_interval_GET() == 4.5329806E37F);
            assert(pack.video_status_GET() == (char)241);
            assert(pack.available_capacity_GET() == -9.510413E37F);
            assert(pack.time_boot_ms_GET() == 1939551856L);
            assert(pack.image_status_GET() == (char)15);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.video_status_SET((char)241) ;
        p262.time_boot_ms_SET(1939551856L) ;
        p262.available_capacity_SET(-9.510413E37F) ;
        p262.image_interval_SET(4.5329806E37F) ;
        p262.recording_time_ms_SET(3087309060L) ;
        p262.image_status_SET((char)15) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1418974459);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.7469726E38F, -4.274499E37F, -2.8401302E38F, 9.580641E37F}));
            assert(pack.file_url_LEN(ph) == 198);
            assert(pack.file_url_TRY(ph).equals("uuejeflcnwvfuzbSKvctvurjfwonvypvxcgkztujkqkeraqjjsbaiztgbiocxtmqNkjxNgrEizixwtAfkvschwsylNnibngdmiofsubgbxxyenUpxmgxmvvhkjakudwuzcnufjzqhaytNclfgvqqafufbmjcdgyCmzwggdvsgfgjdhnwjdjfwcmgujnexzaXoslank"));
            assert(pack.camera_id_GET() == (char)8);
            assert(pack.relative_alt_GET() == 31462622);
            assert(pack.image_index_GET() == -857070225);
            assert(pack.lon_GET() == 532234856);
            assert(pack.time_utc_GET() == 9130594043878769453L);
            assert(pack.alt_GET() == -201174354);
            assert(pack.capture_result_GET() == (byte)114);
            assert(pack.time_boot_ms_GET() == 2354270644L);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.lat_SET(-1418974459) ;
        p263.q_SET(new float[] {-1.7469726E38F, -4.274499E37F, -2.8401302E38F, 9.580641E37F}, 0) ;
        p263.image_index_SET(-857070225) ;
        p263.time_boot_ms_SET(2354270644L) ;
        p263.relative_alt_SET(31462622) ;
        p263.lon_SET(532234856) ;
        p263.capture_result_SET((byte)114) ;
        p263.file_url_SET("uuejeflcnwvfuzbSKvctvurjfwonvypvxcgkztujkqkeraqjjsbaiztgbiocxtmqNkjxNgrEizixwtAfkvschwsylNnibngdmiofsubgbxxyenUpxmgxmvvhkjakudwuzcnufjzqhaytNclfgvqqafufbmjcdgyCmzwggdvsgfgjdhnwjdjfwcmgujnexzaXoslank", PH) ;
        p263.time_utc_SET(9130594043878769453L) ;
        p263.camera_id_SET((char)8) ;
        p263.alt_SET(-201174354) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.flight_uuid_GET() == 6288676650567800599L);
            assert(pack.takeoff_time_utc_GET() == 9178550581477016640L);
            assert(pack.arming_time_utc_GET() == 3576828732447157154L);
            assert(pack.time_boot_ms_GET() == 72161234L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.takeoff_time_utc_SET(9178550581477016640L) ;
        p264.arming_time_utc_SET(3576828732447157154L) ;
        p264.flight_uuid_SET(6288676650567800599L) ;
        p264.time_boot_ms_SET(72161234L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 1.3018842E38F);
            assert(pack.pitch_GET() == 9.391074E37F);
            assert(pack.time_boot_ms_GET() == 395018949L);
            assert(pack.roll_GET() == 1.893713E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(1.893713E38F) ;
        p265.pitch_SET(9.391074E37F) ;
        p265.yaw_SET(1.3018842E38F) ;
        p265.time_boot_ms_SET(395018949L) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.length_GET() == (char)159);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)49, (char)97, (char)100, (char)22, (char)97, (char)146, (char)68, (char)0, (char)32, (char)205, (char)92, (char)206, (char)85, (char)155, (char)121, (char)98, (char)99, (char)235, (char)136, (char)155, (char)25, (char)6, (char)114, (char)217, (char)86, (char)219, (char)33, (char)47, (char)94, (char)222, (char)79, (char)66, (char)58, (char)2, (char)148, (char)127, (char)166, (char)152, (char)162, (char)50, (char)149, (char)185, (char)56, (char)82, (char)254, (char)39, (char)97, (char)49, (char)91, (char)166, (char)134, (char)252, (char)181, (char)181, (char)91, (char)146, (char)2, (char)39, (char)68, (char)109, (char)32, (char)13, (char)146, (char)241, (char)56, (char)49, (char)82, (char)163, (char)179, (char)80, (char)14, (char)120, (char)125, (char)134, (char)27, (char)167, (char)34, (char)102, (char)255, (char)4, (char)152, (char)135, (char)164, (char)23, (char)11, (char)167, (char)0, (char)45, (char)196, (char)200, (char)199, (char)58, (char)72, (char)72, (char)188, (char)19, (char)30, (char)121, (char)114, (char)247, (char)72, (char)2, (char)107, (char)183, (char)69, (char)224, (char)143, (char)77, (char)21, (char)40, (char)158, (char)91, (char)254, (char)99, (char)198, (char)115, (char)3, (char)241, (char)20, (char)251, (char)224, (char)134, (char)171, (char)87, (char)125, (char)180, (char)209, (char)245, (char)61, (char)54, (char)132, (char)114, (char)188, (char)17, (char)115, (char)151, (char)232, (char)119, (char)212, (char)215, (char)38, (char)157, (char)116, (char)247, (char)189, (char)245, (char)170, (char)196, (char)248, (char)79, (char)86, (char)102, (char)163, (char)170, (char)66, (char)216, (char)41, (char)2, (char)156, (char)29, (char)26, (char)254, (char)109, (char)22, (char)253, (char)250, (char)253, (char)88, (char)63, (char)112, (char)60, (char)185, (char)135, (char)29, (char)28, (char)69, (char)188, (char)41, (char)121, (char)116, (char)87, (char)56, (char)237, (char)187, (char)254, (char)190, (char)12, (char)22, (char)134, (char)95, (char)228, (char)130, (char)97, (char)65, (char)58, (char)58, (char)99, (char)215, (char)22, (char)153, (char)242, (char)128, (char)165, (char)72, (char)122, (char)23, (char)70, (char)71, (char)187, (char)150, (char)102, (char)126, (char)21, (char)116, (char)41, (char)116, (char)222, (char)32, (char)231, (char)215, (char)34, (char)116, (char)57, (char)195, (char)96, (char)250, (char)105, (char)218, (char)141, (char)126, (char)221, (char)142, (char)57, (char)120, (char)159, (char)72, (char)86, (char)56, (char)77, (char)139, (char)237, (char)80, (char)247, (char)218, (char)55, (char)125, (char)95, (char)251, (char)141}));
            assert(pack.first_message_offset_GET() == (char)144);
            assert(pack.sequence_GET() == (char)133);
            assert(pack.target_component_GET() == (char)88);
            assert(pack.target_system_GET() == (char)78);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.sequence_SET((char)133) ;
        p266.length_SET((char)159) ;
        p266.data__SET(new char[] {(char)49, (char)97, (char)100, (char)22, (char)97, (char)146, (char)68, (char)0, (char)32, (char)205, (char)92, (char)206, (char)85, (char)155, (char)121, (char)98, (char)99, (char)235, (char)136, (char)155, (char)25, (char)6, (char)114, (char)217, (char)86, (char)219, (char)33, (char)47, (char)94, (char)222, (char)79, (char)66, (char)58, (char)2, (char)148, (char)127, (char)166, (char)152, (char)162, (char)50, (char)149, (char)185, (char)56, (char)82, (char)254, (char)39, (char)97, (char)49, (char)91, (char)166, (char)134, (char)252, (char)181, (char)181, (char)91, (char)146, (char)2, (char)39, (char)68, (char)109, (char)32, (char)13, (char)146, (char)241, (char)56, (char)49, (char)82, (char)163, (char)179, (char)80, (char)14, (char)120, (char)125, (char)134, (char)27, (char)167, (char)34, (char)102, (char)255, (char)4, (char)152, (char)135, (char)164, (char)23, (char)11, (char)167, (char)0, (char)45, (char)196, (char)200, (char)199, (char)58, (char)72, (char)72, (char)188, (char)19, (char)30, (char)121, (char)114, (char)247, (char)72, (char)2, (char)107, (char)183, (char)69, (char)224, (char)143, (char)77, (char)21, (char)40, (char)158, (char)91, (char)254, (char)99, (char)198, (char)115, (char)3, (char)241, (char)20, (char)251, (char)224, (char)134, (char)171, (char)87, (char)125, (char)180, (char)209, (char)245, (char)61, (char)54, (char)132, (char)114, (char)188, (char)17, (char)115, (char)151, (char)232, (char)119, (char)212, (char)215, (char)38, (char)157, (char)116, (char)247, (char)189, (char)245, (char)170, (char)196, (char)248, (char)79, (char)86, (char)102, (char)163, (char)170, (char)66, (char)216, (char)41, (char)2, (char)156, (char)29, (char)26, (char)254, (char)109, (char)22, (char)253, (char)250, (char)253, (char)88, (char)63, (char)112, (char)60, (char)185, (char)135, (char)29, (char)28, (char)69, (char)188, (char)41, (char)121, (char)116, (char)87, (char)56, (char)237, (char)187, (char)254, (char)190, (char)12, (char)22, (char)134, (char)95, (char)228, (char)130, (char)97, (char)65, (char)58, (char)58, (char)99, (char)215, (char)22, (char)153, (char)242, (char)128, (char)165, (char)72, (char)122, (char)23, (char)70, (char)71, (char)187, (char)150, (char)102, (char)126, (char)21, (char)116, (char)41, (char)116, (char)222, (char)32, (char)231, (char)215, (char)34, (char)116, (char)57, (char)195, (char)96, (char)250, (char)105, (char)218, (char)141, (char)126, (char)221, (char)142, (char)57, (char)120, (char)159, (char)72, (char)86, (char)56, (char)77, (char)139, (char)237, (char)80, (char)247, (char)218, (char)55, (char)125, (char)95, (char)251, (char)141}, 0) ;
        p266.target_component_SET((char)88) ;
        p266.target_system_SET((char)78) ;
        p266.first_message_offset_SET((char)144) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)210);
            assert(pack.length_GET() == (char)206);
            assert(pack.sequence_GET() == (char)8263);
            assert(pack.first_message_offset_GET() == (char)45);
            assert(pack.target_system_GET() == (char)237);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)67, (char)3, (char)158, (char)182, (char)233, (char)10, (char)8, (char)201, (char)154, (char)12, (char)14, (char)134, (char)212, (char)98, (char)51, (char)94, (char)1, (char)220, (char)139, (char)8, (char)159, (char)245, (char)163, (char)4, (char)95, (char)254, (char)20, (char)181, (char)230, (char)120, (char)34, (char)125, (char)217, (char)213, (char)245, (char)251, (char)90, (char)122, (char)223, (char)26, (char)158, (char)128, (char)57, (char)131, (char)236, (char)183, (char)159, (char)53, (char)29, (char)221, (char)160, (char)40, (char)107, (char)61, (char)175, (char)228, (char)164, (char)230, (char)149, (char)119, (char)174, (char)225, (char)101, (char)71, (char)56, (char)137, (char)176, (char)139, (char)97, (char)159, (char)151, (char)190, (char)166, (char)94, (char)156, (char)28, (char)37, (char)82, (char)185, (char)148, (char)31, (char)27, (char)241, (char)124, (char)71, (char)164, (char)128, (char)226, (char)62, (char)207, (char)215, (char)239, (char)88, (char)10, (char)141, (char)204, (char)82, (char)140, (char)20, (char)1, (char)60, (char)66, (char)205, (char)233, (char)254, (char)111, (char)14, (char)243, (char)45, (char)21, (char)91, (char)3, (char)42, (char)123, (char)111, (char)150, (char)160, (char)86, (char)149, (char)191, (char)231, (char)225, (char)241, (char)149, (char)0, (char)99, (char)218, (char)205, (char)227, (char)72, (char)246, (char)88, (char)95, (char)86, (char)74, (char)36, (char)72, (char)59, (char)60, (char)9, (char)90, (char)54, (char)105, (char)206, (char)246, (char)41, (char)166, (char)64, (char)120, (char)29, (char)110, (char)19, (char)94, (char)221, (char)201, (char)216, (char)20, (char)189, (char)161, (char)244, (char)204, (char)153, (char)109, (char)63, (char)58, (char)127, (char)169, (char)70, (char)194, (char)184, (char)221, (char)176, (char)225, (char)172, (char)120, (char)96, (char)23, (char)99, (char)111, (char)207, (char)82, (char)184, (char)33, (char)225, (char)79, (char)86, (char)191, (char)44, (char)227, (char)255, (char)233, (char)245, (char)36, (char)63, (char)225, (char)8, (char)147, (char)35, (char)1, (char)23, (char)165, (char)162, (char)188, (char)193, (char)10, (char)72, (char)219, (char)62, (char)220, (char)86, (char)232, (char)178, (char)160, (char)42, (char)89, (char)221, (char)43, (char)82, (char)109, (char)89, (char)31, (char)184, (char)83, (char)232, (char)184, (char)207, (char)46, (char)207, (char)175, (char)202, (char)156, (char)3, (char)68, (char)114, (char)162, (char)14, (char)173, (char)103, (char)99, (char)77, (char)39, (char)183, (char)139, (char)143, (char)251, (char)32, (char)56, (char)14, (char)197}));
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)237) ;
        p267.target_component_SET((char)210) ;
        p267.first_message_offset_SET((char)45) ;
        p267.length_SET((char)206) ;
        p267.data__SET(new char[] {(char)67, (char)3, (char)158, (char)182, (char)233, (char)10, (char)8, (char)201, (char)154, (char)12, (char)14, (char)134, (char)212, (char)98, (char)51, (char)94, (char)1, (char)220, (char)139, (char)8, (char)159, (char)245, (char)163, (char)4, (char)95, (char)254, (char)20, (char)181, (char)230, (char)120, (char)34, (char)125, (char)217, (char)213, (char)245, (char)251, (char)90, (char)122, (char)223, (char)26, (char)158, (char)128, (char)57, (char)131, (char)236, (char)183, (char)159, (char)53, (char)29, (char)221, (char)160, (char)40, (char)107, (char)61, (char)175, (char)228, (char)164, (char)230, (char)149, (char)119, (char)174, (char)225, (char)101, (char)71, (char)56, (char)137, (char)176, (char)139, (char)97, (char)159, (char)151, (char)190, (char)166, (char)94, (char)156, (char)28, (char)37, (char)82, (char)185, (char)148, (char)31, (char)27, (char)241, (char)124, (char)71, (char)164, (char)128, (char)226, (char)62, (char)207, (char)215, (char)239, (char)88, (char)10, (char)141, (char)204, (char)82, (char)140, (char)20, (char)1, (char)60, (char)66, (char)205, (char)233, (char)254, (char)111, (char)14, (char)243, (char)45, (char)21, (char)91, (char)3, (char)42, (char)123, (char)111, (char)150, (char)160, (char)86, (char)149, (char)191, (char)231, (char)225, (char)241, (char)149, (char)0, (char)99, (char)218, (char)205, (char)227, (char)72, (char)246, (char)88, (char)95, (char)86, (char)74, (char)36, (char)72, (char)59, (char)60, (char)9, (char)90, (char)54, (char)105, (char)206, (char)246, (char)41, (char)166, (char)64, (char)120, (char)29, (char)110, (char)19, (char)94, (char)221, (char)201, (char)216, (char)20, (char)189, (char)161, (char)244, (char)204, (char)153, (char)109, (char)63, (char)58, (char)127, (char)169, (char)70, (char)194, (char)184, (char)221, (char)176, (char)225, (char)172, (char)120, (char)96, (char)23, (char)99, (char)111, (char)207, (char)82, (char)184, (char)33, (char)225, (char)79, (char)86, (char)191, (char)44, (char)227, (char)255, (char)233, (char)245, (char)36, (char)63, (char)225, (char)8, (char)147, (char)35, (char)1, (char)23, (char)165, (char)162, (char)188, (char)193, (char)10, (char)72, (char)219, (char)62, (char)220, (char)86, (char)232, (char)178, (char)160, (char)42, (char)89, (char)221, (char)43, (char)82, (char)109, (char)89, (char)31, (char)184, (char)83, (char)232, (char)184, (char)207, (char)46, (char)207, (char)175, (char)202, (char)156, (char)3, (char)68, (char)114, (char)162, (char)14, (char)173, (char)103, (char)99, (char)77, (char)39, (char)183, (char)139, (char)143, (char)251, (char)32, (char)56, (char)14, (char)197}, 0) ;
        p267.sequence_SET((char)8263) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)30584);
            assert(pack.target_system_GET() == (char)119);
            assert(pack.target_component_GET() == (char)97);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_component_SET((char)97) ;
        p268.sequence_SET((char)30584) ;
        p268.target_system_SET((char)119) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.camera_id_GET() == (char)22);
            assert(pack.rotation_GET() == (char)42381);
            assert(pack.framerate_GET() == 8.3548493E37F);
            assert(pack.resolution_v_GET() == (char)50090);
            assert(pack.bitrate_GET() == 43313202L);
            assert(pack.resolution_h_GET() == (char)25675);
            assert(pack.uri_LEN(ph) == 50);
            assert(pack.uri_TRY(ph).equals("srvupYfzxfsfkvrmuwrknGrzdwivifspRvpwIuQqxnyufqsfkt"));
            assert(pack.status_GET() == (char)175);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.uri_SET("srvupYfzxfsfkvrmuwrknGrzdwivifspRvpwIuQqxnyufqsfkt", PH) ;
        p269.framerate_SET(8.3548493E37F) ;
        p269.resolution_h_SET((char)25675) ;
        p269.bitrate_SET(43313202L) ;
        p269.rotation_SET((char)42381) ;
        p269.status_SET((char)175) ;
        p269.resolution_v_SET((char)50090) ;
        p269.camera_id_SET((char)22) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.framerate_GET() == -2.4886644E38F);
            assert(pack.resolution_h_GET() == (char)35260);
            assert(pack.camera_id_GET() == (char)51);
            assert(pack.target_component_GET() == (char)63);
            assert(pack.target_system_GET() == (char)120);
            assert(pack.rotation_GET() == (char)22179);
            assert(pack.uri_LEN(ph) == 98);
            assert(pack.uri_TRY(ph).equals("makhtrbyuwrvzHcrbfznrqqiwsqrpbvffybgicaunnkpfykqbxrnjihLCZjxwwqcdkdkoyssyvkdzkGfjlaqeHwkGjkldyvguj"));
            assert(pack.resolution_v_GET() == (char)29687);
            assert(pack.bitrate_GET() == 2101079738L);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.resolution_v_SET((char)29687) ;
        p270.camera_id_SET((char)51) ;
        p270.resolution_h_SET((char)35260) ;
        p270.target_system_SET((char)120) ;
        p270.framerate_SET(-2.4886644E38F) ;
        p270.rotation_SET((char)22179) ;
        p270.target_component_SET((char)63) ;
        p270.uri_SET("makhtrbyuwrvzHcrbfznrqqiwsqrpbvffybgicaunnkpfykqbxrnjihLCZjxwwqcdkdkoyssyvkdzkGfjlaqeHwkGjkldyvguj", PH) ;
        p270.bitrate_SET(2101079738L) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 11);
            assert(pack.ssid_TRY(ph).equals("XwyWltawinb"));
            assert(pack.password_LEN(ph) == 58);
            assert(pack.password_TRY(ph).equals("anlakmkloVijiwhogpBqzrbeAsctsvxvoPkavbaomcdFkupfjdqhaKddsl"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("anlakmkloVijiwhogpBqzrbeAsctsvxvoPkavbaomcdFkupfjdqhaKddsl", PH) ;
        p299.ssid_SET("XwyWltawinb", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.min_version_GET() == (char)14559);
            assert(pack.version_GET() == (char)60359);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)150, (char)73, (char)15, (char)101, (char)131, (char)13, (char)133, (char)182}));
            assert(pack.max_version_GET() == (char)51748);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)151, (char)226, (char)148, (char)213, (char)12, (char)131, (char)87, (char)200}));
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.min_version_SET((char)14559) ;
        p300.spec_version_hash_SET(new char[] {(char)150, (char)73, (char)15, (char)101, (char)131, (char)13, (char)133, (char)182}, 0) ;
        p300.max_version_SET((char)51748) ;
        p300.version_SET((char)60359) ;
        p300.library_version_hash_SET(new char[] {(char)151, (char)226, (char)148, (char)213, (char)12, (char)131, (char)87, (char)200}, 0) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.uptime_sec_GET() == 1426556888L);
            assert(pack.vendor_specific_status_code_GET() == (char)45738);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
            assert(pack.sub_mode_GET() == (char)108);
            assert(pack.time_usec_GET() == 4927542402846547437L);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.uptime_sec_SET(1426556888L) ;
        p310.time_usec_SET(4927542402846547437L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION) ;
        p310.sub_mode_SET((char)108) ;
        p310.vendor_specific_status_code_SET((char)45738) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.uptime_sec_GET() == 3963198130L);
            assert(pack.hw_version_major_GET() == (char)226);
            assert(pack.sw_vcs_commit_GET() == 414732194L);
            assert(pack.sw_version_minor_GET() == (char)153);
            assert(pack.hw_version_minor_GET() == (char)121);
            assert(pack.sw_version_major_GET() == (char)195);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)171, (char)22, (char)160, (char)75, (char)103, (char)161, (char)0, (char)70, (char)86, (char)238, (char)75, (char)93, (char)100, (char)131, (char)50, (char)104}));
            assert(pack.time_usec_GET() == 705560875462849494L);
            assert(pack.name_LEN(ph) == 48);
            assert(pack.name_TRY(ph).equals("upiuljhxjuozmpzjtiuqJutauoyaoyHXmvevdAxRJzutfpjt"));
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_unique_id_SET(new char[] {(char)171, (char)22, (char)160, (char)75, (char)103, (char)161, (char)0, (char)70, (char)86, (char)238, (char)75, (char)93, (char)100, (char)131, (char)50, (char)104}, 0) ;
        p311.hw_version_major_SET((char)226) ;
        p311.sw_vcs_commit_SET(414732194L) ;
        p311.time_usec_SET(705560875462849494L) ;
        p311.hw_version_minor_SET((char)121) ;
        p311.sw_version_major_SET((char)195) ;
        p311.name_SET("upiuljhxjuozmpzjtiuqJutauoyaoyHXmvevdAxRJzutfpjt", PH) ;
        p311.uptime_sec_SET(3963198130L) ;
        p311.sw_version_minor_SET((char)153) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)247);
            assert(pack.param_index_GET() == (short)21347);
            assert(pack.target_system_GET() == (char)80);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("o"));
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_component_SET((char)247) ;
        p320.param_id_SET("o", PH) ;
        p320.param_index_SET((short)21347) ;
        p320.target_system_SET((char)80) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)246);
            assert(pack.target_component_GET() == (char)69);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)69) ;
        p321.target_system_SET((char)246) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 67);
            assert(pack.param_value_TRY(ph).equals("uMnmgdoyFWounbxfbywLpfhcfnufkmibqdgllmnbpWsirzsooSwrFnskaTrdssbgTka"));
            assert(pack.param_index_GET() == (char)1617);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("mtnxyyvrWjmbzz"));
            assert(pack.param_count_GET() == (char)45486);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("mtnxyyvrWjmbzz", PH) ;
        p322.param_index_SET((char)1617) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16) ;
        p322.param_count_SET((char)45486) ;
        p322.param_value_SET("uMnmgdoyFWounbxfbywLpfhcfnufkmibqdgllmnbpWsirzsooSwrFnskaTrdssbgTka", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)54);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("zCmzh"));
            assert(pack.param_value_LEN(ph) == 67);
            assert(pack.param_value_TRY(ph).equals("isdvndywqkyhdronkdxofynlFldfoagmdqtavddcencikpnwskubBhhyusqncunpmqj"));
            assert(pack.target_system_GET() == (char)42);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_id_SET("zCmzh", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64) ;
        p323.param_value_SET("isdvndywqkyhdronkdxofynlFldfoagmdqtavddcencikpnwskubBhhyusqncunpmqj", PH) ;
        p323.target_system_SET((char)42) ;
        p323.target_component_SET((char)54) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 113);
            assert(pack.param_value_TRY(ph).equals("yuqRvgdocydUkhfhwjkunsiNlsqZsXaefgylyliFyplgcrqfxeosmlekimOkjlpgpvlysfrflGvacknqoacislpzzxnfdirxugwmwkEwcitjlYfgz"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("ruwgvfkjenstjw"));
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("ruwgvfkjenstjw", PH) ;
        p324.param_value_SET("yuqRvgdocydUkhfhwjkunsiNlsqZsXaefgylyliFyplgcrqfxeosmlekimOkjlpgpvlysfrflGvacknqoacislpzzxnfdirxugwmwkEwcitjlYfgz", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)65286, (char)9495, (char)34305, (char)24901, (char)42363, (char)35931, (char)53922, (char)43355, (char)47227, (char)19840, (char)11294, (char)34144, (char)55221, (char)46219, (char)53192, (char)39446, (char)11401, (char)30612, (char)19903, (char)9753, (char)46517, (char)2091, (char)62502, (char)54851, (char)26933, (char)32847, (char)54520, (char)31872, (char)61262, (char)11069, (char)62280, (char)858, (char)29772, (char)21358, (char)50477, (char)41992, (char)49918, (char)61990, (char)29207, (char)51388, (char)7649, (char)35286, (char)41052, (char)22903, (char)34961, (char)53898, (char)6827, (char)41750, (char)64516, (char)51808, (char)42724, (char)15593, (char)52262, (char)51698, (char)41093, (char)20238, (char)37037, (char)49006, (char)59048, (char)24547, (char)47605, (char)55311, (char)3197, (char)2732, (char)61002, (char)32613, (char)41020, (char)39113, (char)3302, (char)33082, (char)59766, (char)61739}));
            assert(pack.min_distance_GET() == (char)55771);
            assert(pack.time_usec_GET() == 1953861535748541514L);
            assert(pack.increment_GET() == (char)37);
            assert(pack.max_distance_GET() == (char)46245);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.min_distance_SET((char)55771) ;
        p330.increment_SET((char)37) ;
        p330.time_usec_SET(1953861535748541514L) ;
        p330.distances_SET(new char[] {(char)65286, (char)9495, (char)34305, (char)24901, (char)42363, (char)35931, (char)53922, (char)43355, (char)47227, (char)19840, (char)11294, (char)34144, (char)55221, (char)46219, (char)53192, (char)39446, (char)11401, (char)30612, (char)19903, (char)9753, (char)46517, (char)2091, (char)62502, (char)54851, (char)26933, (char)32847, (char)54520, (char)31872, (char)61262, (char)11069, (char)62280, (char)858, (char)29772, (char)21358, (char)50477, (char)41992, (char)49918, (char)61990, (char)29207, (char)51388, (char)7649, (char)35286, (char)41052, (char)22903, (char)34961, (char)53898, (char)6827, (char)41750, (char)64516, (char)51808, (char)42724, (char)15593, (char)52262, (char)51698, (char)41093, (char)20238, (char)37037, (char)49006, (char)59048, (char)24547, (char)47605, (char)55311, (char)3197, (char)2732, (char)61002, (char)32613, (char)41020, (char)39113, (char)3302, (char)33082, (char)59766, (char)61739}, 0) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND) ;
        p330.max_distance_SET((char)46245) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
    }

}
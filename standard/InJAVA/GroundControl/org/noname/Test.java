
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
            long id = id__g(src);
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
            long id = id__g(src);
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
        *	 this field to difference between instances*/
        public char group_mlx_GET()
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  10, 1)); }
        /**
        *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
        *	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
        *	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
        *	 mixer to repurpose them as generic outputs*/
        public float[] controls_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 11, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        /**
        *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
        *	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
        *	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
        *	 mixer to repurpose them as generic outputs*/
        public float[] controls_GET()
        {return controls_GET(new float[8], 0);}
    }
    public static class ACTUATOR_CONTROL_TARGET extends GroundControl.ACTUATOR_CONTROL_TARGET
    {
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        /**
        *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
        *	 this field to difference between instances*/
        public char group_mlx_GET()
        {  return (char)((char) get_bytes(data,  8, 1)); }
        /**
        *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
        *	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
        *	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
        *	 mixer to repurpose them as generic outputs*/
        public float[] controls_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 9, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        /**
        *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
        *	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
        *	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
        *	 mixer to repurpose them as generic outputs*/
        public float[] controls_GET()
        {return controls_GET(new float[8], 0);}
    }
    public static class ALTITUDE extends GroundControl.ALTITUDE
    {
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        /**
        *This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
        *	 local altitude change). The only guarantee on this field is that it will never be reset and is consistent
        *	 within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
        *	 time. This altitude will also drift and vary between flights*/
        public float altitude_monotonic_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        /**
        *This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
        *	 like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
        *	 are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
        *	 by default and not the WGS84 altitude*/
        public float altitude_amsl_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        /**
        *This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
        *	 to the coordinate origin (0, 0, 0). It is up-positive*/
        public float altitude_local_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float altitude_relative_GET()//This is the altitude above the home position. It resets on each change of the current home position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        /**
        *This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
        *	 than -1000 should be interpreted as unknown*/
        public float altitude_terrain_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        /**
        *This is not the altitude, but the clear space below the system according to the fused clearance estimate.
        *	 It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
        *	 target. A negative value indicates no measurement available*/
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
        *	 on the URI type enum*/
        public char[] uri_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 2, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
        *	 on the URI type enum*/
        public char[] uri_GET()
        {return uri_GET(new char[120], 0);} public char transfer_type_GET()//The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
        {  return (char)((char) get_bytes(data,  122, 1)); }
        /**
        *The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
        *	 has a storage associated (e.g. MAVLink FTP)*/
        public char[] storage_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 123, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
        *	 has a storage associated (e.g. MAVLink FTP)*/
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
            assert(pack.custom_mode_GET() == 4248769061L);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_POWEROFF);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_ADSB);
            assert(pack.mavlink_version_GET() == (char)58);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC) ;
        p0.mavlink_version_SET((char)58) ;
        p0.custom_mode_SET(4248769061L) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_ADSB) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_POWEROFF) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED)) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.load_GET() == (char)1441);
            assert(pack.drop_rate_comm_GET() == (char)53548);
            assert(pack.errors_comm_GET() == (char)27425);
            assert(pack.errors_count2_GET() == (char)63478);
            assert(pack.voltage_battery_GET() == (char)8565);
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH));
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION));
            assert(pack.errors_count4_GET() == (char)40208);
            assert(pack.errors_count1_GET() == (char)38692);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
            assert(pack.current_battery_GET() == (short) -31597);
            assert(pack.errors_count3_GET() == (char)57615);
            assert(pack.battery_remaining_GET() == (byte) - 15);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.drop_rate_comm_SET((char)53548) ;
        p1.errors_count3_SET((char)57615) ;
        p1.errors_count4_SET((char)40208) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW)) ;
        p1.errors_count1_SET((char)38692) ;
        p1.battery_remaining_SET((byte) - 15) ;
        p1.voltage_battery_SET((char)8565) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION)) ;
        p1.errors_comm_SET((char)27425) ;
        p1.errors_count2_SET((char)63478) ;
        p1.current_battery_SET((short) -31597) ;
        p1.load_SET((char)1441) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH)) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 4994097132887796648L);
            assert(pack.time_boot_ms_GET() == 3042813542L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(4994097132887796648L) ;
        p2.time_boot_ms_SET(3042813542L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -2.5675358E38F);
            assert(pack.x_GET() == -3.009271E38F);
            assert(pack.vz_GET() == 1.9397409E38F);
            assert(pack.type_mask_GET() == (char)2693);
            assert(pack.z_GET() == 7.32548E37F);
            assert(pack.vx_GET() == -6.279551E37F);
            assert(pack.yaw_rate_GET() == -1.064964E38F);
            assert(pack.time_boot_ms_GET() == 3541065319L);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.afz_GET() == 2.2801424E38F);
            assert(pack.afy_GET() == 3.320291E38F);
            assert(pack.afx_GET() == 4.647713E37F);
            assert(pack.vy_GET() == -1.8387389E37F);
            assert(pack.y_GET() == -2.238618E38F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.afz_SET(2.2801424E38F) ;
        p3.afx_SET(4.647713E37F) ;
        p3.y_SET(-2.238618E38F) ;
        p3.afy_SET(3.320291E38F) ;
        p3.vz_SET(1.9397409E38F) ;
        p3.type_mask_SET((char)2693) ;
        p3.yaw_SET(-2.5675358E38F) ;
        p3.z_SET(7.32548E37F) ;
        p3.time_boot_ms_SET(3541065319L) ;
        p3.x_SET(-3.009271E38F) ;
        p3.vy_SET(-1.8387389E37F) ;
        p3.yaw_rate_SET(-1.064964E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p3.vx_SET(-6.279551E37F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 2784270464L);
            assert(pack.time_usec_GET() == 83383704881866467L);
            assert(pack.target_system_GET() == (char)113);
            assert(pack.target_component_GET() == (char)87);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.seq_SET(2784270464L) ;
        p4.time_usec_SET(83383704881866467L) ;
        p4.target_system_SET((char)113) ;
        p4.target_component_SET((char)87) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.passkey_LEN(ph) == 2);
            assert(pack.passkey_TRY(ph).equals("cd"));
            assert(pack.version_GET() == (char)82);
            assert(pack.control_request_GET() == (char)26);
            assert(pack.target_system_GET() == (char)188);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)188) ;
        p5.version_SET((char)82) ;
        p5.passkey_SET("cd", PH) ;
        p5.control_request_SET((char)26) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)42);
            assert(pack.ack_GET() == (char)125);
            assert(pack.control_request_GET() == (char)36);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.ack_SET((char)125) ;
        p6.control_request_SET((char)36) ;
        p6.gcs_system_id_SET((char)42) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 18);
            assert(pack.key_TRY(ph).equals("VrQckmIungexpjdcdd"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("VrQckmIungexpjdcdd", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
            assert(pack.custom_mode_GET() == 1917325155L);
            assert(pack.target_system_GET() == (char)214);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.custom_mode_SET(1917325155L) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_STABILIZE_DISARMED) ;
        p11.target_system_SET((char)214) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short) -11719);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("gziq"));
            assert(pack.target_system_GET() == (char)103);
            assert(pack.target_component_GET() == (char)20);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_index_SET((short) -11719) ;
        p20.target_component_SET((char)20) ;
        p20.target_system_SET((char)103) ;
        p20.param_id_SET("gziq", PH) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)26);
            assert(pack.target_component_GET() == (char)187);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)26) ;
        p21.target_component_SET((char)187) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("ZddxvzhJzt"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
            assert(pack.param_count_GET() == (char)47619);
            assert(pack.param_index_GET() == (char)16354);
            assert(pack.param_value_GET() == 2.1469868E38F);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_value_SET(2.1469868E38F) ;
        p22.param_index_SET((char)16354) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64) ;
        p22.param_id_SET("ZddxvzhJzt", PH) ;
        p22.param_count_SET((char)47619) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_value_GET() == -6.5194566E37F);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("v"));
            assert(pack.target_component_GET() == (char)98);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
            assert(pack.target_system_GET() == (char)118);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.target_component_SET((char)98) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16) ;
        p23.param_id_SET("v", PH) ;
        p23.param_value_SET(-6.5194566E37F) ;
        p23.target_system_SET((char)118) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 798671703);
            assert(pack.hdg_acc_TRY(ph) == 844393076L);
            assert(pack.eph_GET() == (char)41058);
            assert(pack.cog_GET() == (char)21337);
            assert(pack.vel_acc_TRY(ph) == 2907021192L);
            assert(pack.lon_GET() == 1476243475);
            assert(pack.lat_GET() == -526277078);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
            assert(pack.vel_GET() == (char)6905);
            assert(pack.alt_ellipsoid_TRY(ph) == -581238269);
            assert(pack.time_usec_GET() == 1399136730047548632L);
            assert(pack.h_acc_TRY(ph) == 488062142L);
            assert(pack.satellites_visible_GET() == (char)7);
            assert(pack.epv_GET() == (char)56401);
            assert(pack.v_acc_TRY(ph) == 1313754687L);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.satellites_visible_SET((char)7) ;
        p24.vel_SET((char)6905) ;
        p24.h_acc_SET(488062142L, PH) ;
        p24.lat_SET(-526277078) ;
        p24.eph_SET((char)41058) ;
        p24.lon_SET(1476243475) ;
        p24.alt_SET(798671703) ;
        p24.cog_SET((char)21337) ;
        p24.time_usec_SET(1399136730047548632L) ;
        p24.hdg_acc_SET(844393076L, PH) ;
        p24.alt_ellipsoid_SET(-581238269, PH) ;
        p24.v_acc_SET(1313754687L, PH) ;
        p24.vel_acc_SET(2907021192L, PH) ;
        p24.epv_SET((char)56401) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)6, (char)51, (char)178, (char)154, (char)100, (char)53, (char)132, (char)152, (char)180, (char)157, (char)179, (char)250, (char)234, (char)197, (char)24, (char)174, (char)108, (char)41, (char)188, (char)47}));
            assert(pack.satellites_visible_GET() == (char)94);
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)211, (char)175, (char)193, (char)237, (char)33, (char)168, (char)144, (char)62, (char)250, (char)228, (char)65, (char)26, (char)138, (char)2, (char)30, (char)213, (char)178, (char)230, (char)236, (char)8}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)194, (char)57, (char)206, (char)109, (char)145, (char)134, (char)240, (char)30, (char)148, (char)74, (char)40, (char)41, (char)55, (char)135, (char)30, (char)74, (char)45, (char)70, (char)141, (char)26}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)164, (char)248, (char)192, (char)104, (char)189, (char)219, (char)175, (char)111, (char)120, (char)240, (char)32, (char)59, (char)6, (char)150, (char)130, (char)27, (char)43, (char)59, (char)45, (char)172}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)17, (char)87, (char)219, (char)50, (char)21, (char)191, (char)110, (char)31, (char)129, (char)146, (char)39, (char)8, (char)250, (char)65, (char)218, (char)56, (char)105, (char)220, (char)9, (char)140}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_prn_SET(new char[] {(char)194, (char)57, (char)206, (char)109, (char)145, (char)134, (char)240, (char)30, (char)148, (char)74, (char)40, (char)41, (char)55, (char)135, (char)30, (char)74, (char)45, (char)70, (char)141, (char)26}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)211, (char)175, (char)193, (char)237, (char)33, (char)168, (char)144, (char)62, (char)250, (char)228, (char)65, (char)26, (char)138, (char)2, (char)30, (char)213, (char)178, (char)230, (char)236, (char)8}, 0) ;
        p25.satellite_used_SET(new char[] {(char)6, (char)51, (char)178, (char)154, (char)100, (char)53, (char)132, (char)152, (char)180, (char)157, (char)179, (char)250, (char)234, (char)197, (char)24, (char)174, (char)108, (char)41, (char)188, (char)47}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)164, (char)248, (char)192, (char)104, (char)189, (char)219, (char)175, (char)111, (char)120, (char)240, (char)32, (char)59, (char)6, (char)150, (char)130, (char)27, (char)43, (char)59, (char)45, (char)172}, 0) ;
        p25.satellites_visible_SET((char)94) ;
        p25.satellite_elevation_SET(new char[] {(char)17, (char)87, (char)219, (char)50, (char)21, (char)191, (char)110, (char)31, (char)129, (char)146, (char)39, (char)8, (char)250, (char)65, (char)218, (char)56, (char)105, (char)220, (char)9, (char)140}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short) -32064);
            assert(pack.zacc_GET() == (short)24153);
            assert(pack.zmag_GET() == (short) -2174);
            assert(pack.ymag_GET() == (short) -4759);
            assert(pack.xacc_GET() == (short)19096);
            assert(pack.xmag_GET() == (short)12737);
            assert(pack.time_boot_ms_GET() == 2635290425L);
            assert(pack.zgyro_GET() == (short) -24776);
            assert(pack.ygyro_GET() == (short)14055);
            assert(pack.xgyro_GET() == (short) -11235);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.ymag_SET((short) -4759) ;
        p26.zacc_SET((short)24153) ;
        p26.ygyro_SET((short)14055) ;
        p26.time_boot_ms_SET(2635290425L) ;
        p26.xmag_SET((short)12737) ;
        p26.zmag_SET((short) -2174) ;
        p26.xacc_SET((short)19096) ;
        p26.yacc_SET((short) -32064) ;
        p26.xgyro_SET((short) -11235) ;
        p26.zgyro_SET((short) -24776) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short) -3600);
            assert(pack.yacc_GET() == (short) -1338);
            assert(pack.zmag_GET() == (short)22163);
            assert(pack.time_usec_GET() == 577047536923801085L);
            assert(pack.ymag_GET() == (short) -5807);
            assert(pack.zgyro_GET() == (short) -8118);
            assert(pack.xacc_GET() == (short)5634);
            assert(pack.xgyro_GET() == (short) -22711);
            assert(pack.xmag_GET() == (short) -8263);
            assert(pack.zacc_GET() == (short)5599);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.ygyro_SET((short) -3600) ;
        p27.yacc_SET((short) -1338) ;
        p27.zgyro_SET((short) -8118) ;
        p27.ymag_SET((short) -5807) ;
        p27.zacc_SET((short)5599) ;
        p27.time_usec_SET(577047536923801085L) ;
        p27.xacc_SET((short)5634) ;
        p27.xmag_SET((short) -8263) ;
        p27.xgyro_SET((short) -22711) ;
        p27.zmag_SET((short)22163) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == (short)27291);
            assert(pack.time_usec_GET() == 2765191927343084401L);
            assert(pack.press_diff1_GET() == (short)18725);
            assert(pack.press_diff2_GET() == (short)31989);
            assert(pack.temperature_GET() == (short)28433);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_abs_SET((short)27291) ;
        p28.temperature_SET((short)28433) ;
        p28.press_diff2_SET((short)31989) ;
        p28.time_usec_SET(2765191927343084401L) ;
        p28.press_diff1_SET((short)18725) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 1.2309697E38F);
            assert(pack.temperature_GET() == (short) -22017);
            assert(pack.press_diff_GET() == -1.1608504E38F);
            assert(pack.time_boot_ms_GET() == 1630271649L);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_diff_SET(-1.1608504E38F) ;
        p29.time_boot_ms_SET(1630271649L) ;
        p29.press_abs_SET(1.2309697E38F) ;
        p29.temperature_SET((short) -22017) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -4.6510577E37F);
            assert(pack.yaw_GET() == 6.2304555E37F);
            assert(pack.yawspeed_GET() == 3.1997992E38F);
            assert(pack.time_boot_ms_GET() == 3748813623L);
            assert(pack.pitchspeed_GET() == 3.3079555E38F);
            assert(pack.rollspeed_GET() == 6.386257E37F);
            assert(pack.pitch_GET() == -1.2633414E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.pitchspeed_SET(3.3079555E38F) ;
        p30.pitch_SET(-1.2633414E38F) ;
        p30.yawspeed_SET(3.1997992E38F) ;
        p30.roll_SET(-4.6510577E37F) ;
        p30.rollspeed_SET(6.386257E37F) ;
        p30.yaw_SET(6.2304555E37F) ;
        p30.time_boot_ms_SET(3748813623L) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q2_GET() == 3.2329053E38F);
            assert(pack.pitchspeed_GET() == -3.2822705E38F);
            assert(pack.q1_GET() == 2.3125018E38F);
            assert(pack.time_boot_ms_GET() == 3229044941L);
            assert(pack.q3_GET() == -7.437241E37F);
            assert(pack.yawspeed_GET() == 7.485073E37F);
            assert(pack.rollspeed_GET() == 2.7653662E38F);
            assert(pack.q4_GET() == -2.2094909E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q4_SET(-2.2094909E38F) ;
        p31.rollspeed_SET(2.7653662E38F) ;
        p31.time_boot_ms_SET(3229044941L) ;
        p31.pitchspeed_SET(-3.2822705E38F) ;
        p31.q2_SET(3.2329053E38F) ;
        p31.yawspeed_SET(7.485073E37F) ;
        p31.q1_SET(2.3125018E38F) ;
        p31.q3_SET(-7.437241E37F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == -2.9843787E38F);
            assert(pack.vx_GET() == 2.1115682E38F);
            assert(pack.z_GET() == -2.3226073E38F);
            assert(pack.y_GET() == -1.5374178E38F);
            assert(pack.vy_GET() == 2.2970425E38F);
            assert(pack.x_GET() == 1.1033124E38F);
            assert(pack.time_boot_ms_GET() == 1223997088L);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.vz_SET(-2.9843787E38F) ;
        p32.vx_SET(2.1115682E38F) ;
        p32.time_boot_ms_SET(1223997088L) ;
        p32.vy_SET(2.2970425E38F) ;
        p32.y_SET(-1.5374178E38F) ;
        p32.z_SET(-2.3226073E38F) ;
        p32.x_SET(1.1033124E38F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -432321827);
            assert(pack.vx_GET() == (short)8184);
            assert(pack.vy_GET() == (short) -23650);
            assert(pack.time_boot_ms_GET() == 1679619015L);
            assert(pack.hdg_GET() == (char)60613);
            assert(pack.alt_GET() == -782966674);
            assert(pack.lon_GET() == 1628454755);
            assert(pack.vz_GET() == (short)360);
            assert(pack.relative_alt_GET() == -1318284990);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.time_boot_ms_SET(1679619015L) ;
        p33.vy_SET((short) -23650) ;
        p33.vz_SET((short)360) ;
        p33.lon_SET(1628454755) ;
        p33.hdg_SET((char)60613) ;
        p33.relative_alt_SET(-1318284990) ;
        p33.vx_SET((short)8184) ;
        p33.lat_SET(-432321827) ;
        p33.alt_SET(-782966674) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan4_scaled_GET() == (short) -3530);
            assert(pack.chan2_scaled_GET() == (short)8792);
            assert(pack.rssi_GET() == (char)14);
            assert(pack.chan1_scaled_GET() == (short)10920);
            assert(pack.port_GET() == (char)172);
            assert(pack.chan8_scaled_GET() == (short) -12478);
            assert(pack.chan7_scaled_GET() == (short) -31888);
            assert(pack.chan5_scaled_GET() == (short) -6581);
            assert(pack.chan3_scaled_GET() == (short) -5160);
            assert(pack.chan6_scaled_GET() == (short)5939);
            assert(pack.time_boot_ms_GET() == 1410795942L);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan8_scaled_SET((short) -12478) ;
        p34.chan7_scaled_SET((short) -31888) ;
        p34.chan1_scaled_SET((short)10920) ;
        p34.rssi_SET((char)14) ;
        p34.time_boot_ms_SET(1410795942L) ;
        p34.port_SET((char)172) ;
        p34.chan5_scaled_SET((short) -6581) ;
        p34.chan3_scaled_SET((short) -5160) ;
        p34.chan6_scaled_SET((short)5939) ;
        p34.chan2_scaled_SET((short)8792) ;
        p34.chan4_scaled_SET((short) -3530) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.port_GET() == (char)63);
            assert(pack.chan1_raw_GET() == (char)7102);
            assert(pack.time_boot_ms_GET() == 3661166015L);
            assert(pack.rssi_GET() == (char)184);
            assert(pack.chan5_raw_GET() == (char)16476);
            assert(pack.chan8_raw_GET() == (char)52879);
            assert(pack.chan3_raw_GET() == (char)4269);
            assert(pack.chan7_raw_GET() == (char)60838);
            assert(pack.chan6_raw_GET() == (char)34976);
            assert(pack.chan2_raw_GET() == (char)16364);
            assert(pack.chan4_raw_GET() == (char)9008);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.port_SET((char)63) ;
        p35.chan8_raw_SET((char)52879) ;
        p35.chan4_raw_SET((char)9008) ;
        p35.chan5_raw_SET((char)16476) ;
        p35.chan7_raw_SET((char)60838) ;
        p35.time_boot_ms_SET(3661166015L) ;
        p35.chan1_raw_SET((char)7102) ;
        p35.rssi_SET((char)184) ;
        p35.chan6_raw_SET((char)34976) ;
        p35.chan3_raw_SET((char)4269) ;
        p35.chan2_raw_SET((char)16364) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo10_raw_TRY(ph) == (char)21023);
            assert(pack.servo2_raw_GET() == (char)45029);
            assert(pack.servo12_raw_TRY(ph) == (char)19899);
            assert(pack.servo13_raw_TRY(ph) == (char)10187);
            assert(pack.time_usec_GET() == 2196787803L);
            assert(pack.servo15_raw_TRY(ph) == (char)18929);
            assert(pack.servo8_raw_GET() == (char)2416);
            assert(pack.servo9_raw_TRY(ph) == (char)37669);
            assert(pack.servo6_raw_GET() == (char)28861);
            assert(pack.servo11_raw_TRY(ph) == (char)23600);
            assert(pack.servo5_raw_GET() == (char)6212);
            assert(pack.servo14_raw_TRY(ph) == (char)60303);
            assert(pack.servo3_raw_GET() == (char)35113);
            assert(pack.servo16_raw_TRY(ph) == (char)4752);
            assert(pack.port_GET() == (char)205);
            assert(pack.servo1_raw_GET() == (char)58350);
            assert(pack.servo4_raw_GET() == (char)40720);
            assert(pack.servo7_raw_GET() == (char)27118);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo14_raw_SET((char)60303, PH) ;
        p36.servo13_raw_SET((char)10187, PH) ;
        p36.servo10_raw_SET((char)21023, PH) ;
        p36.servo9_raw_SET((char)37669, PH) ;
        p36.servo4_raw_SET((char)40720) ;
        p36.port_SET((char)205) ;
        p36.servo7_raw_SET((char)27118) ;
        p36.servo3_raw_SET((char)35113) ;
        p36.servo16_raw_SET((char)4752, PH) ;
        p36.servo15_raw_SET((char)18929, PH) ;
        p36.servo1_raw_SET((char)58350) ;
        p36.servo2_raw_SET((char)45029) ;
        p36.servo12_raw_SET((char)19899, PH) ;
        p36.time_usec_SET(2196787803L) ;
        p36.servo11_raw_SET((char)23600, PH) ;
        p36.servo5_raw_SET((char)6212) ;
        p36.servo8_raw_SET((char)2416) ;
        p36.servo6_raw_SET((char)28861) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)117);
            assert(pack.target_system_GET() == (char)255);
            assert(pack.end_index_GET() == (short) -7145);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.start_index_GET() == (short) -20314);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.end_index_SET((short) -7145) ;
        p37.target_system_SET((char)255) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p37.target_component_SET((char)117) ;
        p37.start_index_SET((short) -20314) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short) -4690);
            assert(pack.target_system_GET() == (char)78);
            assert(pack.target_component_GET() == (char)202);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.start_index_GET() == (short) -32248);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_system_SET((char)78) ;
        p38.end_index_SET((short) -4690) ;
        p38.target_component_SET((char)202) ;
        p38.start_index_SET((short) -32248) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.current_GET() == (char)123);
            assert(pack.y_GET() == -2.6925282E38F);
            assert(pack.target_system_GET() == (char)46);
            assert(pack.param4_GET() == 2.7195503E38F);
            assert(pack.param2_GET() == 1.5145845E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.z_GET() == 7.907319E36F);
            assert(pack.seq_GET() == (char)53184);
            assert(pack.x_GET() == 3.1512847E38F);
            assert(pack.autocontinue_GET() == (char)163);
            assert(pack.param1_GET() == 3.1219456E38F);
            assert(pack.target_component_GET() == (char)22);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_PANORAMA_CREATE);
            assert(pack.param3_GET() == 1.991377E38F);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.command_SET(MAV_CMD.MAV_CMD_PANORAMA_CREATE) ;
        p39.target_system_SET((char)46) ;
        p39.param4_SET(2.7195503E38F) ;
        p39.target_component_SET((char)22) ;
        p39.y_SET(-2.6925282E38F) ;
        p39.z_SET(7.907319E36F) ;
        p39.autocontinue_SET((char)163) ;
        p39.seq_SET((char)53184) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p39.current_SET((char)123) ;
        p39.param1_SET(3.1219456E38F) ;
        p39.param3_SET(1.991377E38F) ;
        p39.param2_SET(1.5145845E38F) ;
        p39.x_SET(3.1512847E38F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)36);
            assert(pack.seq_GET() == (char)14268);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)213);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.seq_SET((char)14268) ;
        p40.target_component_SET((char)213) ;
        p40.target_system_SET((char)36) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)113);
            assert(pack.seq_GET() == (char)55069);
            assert(pack.target_component_GET() == (char)53);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.seq_SET((char)55069) ;
        p41.target_component_SET((char)53) ;
        p41.target_system_SET((char)113) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)4401);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)4401) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)121);
            assert(pack.target_component_GET() == (char)32);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)121) ;
        p43.target_component_SET((char)32) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)72);
            assert(pack.target_component_GET() == (char)34);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.count_GET() == (char)11147);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.count_SET((char)11147) ;
        p44.target_system_SET((char)72) ;
        p44.target_component_SET((char)34) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)34);
            assert(pack.target_component_GET() == (char)190);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)34) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p45.target_component_SET((char)190) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)42228);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)42228) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)173);
            assert(pack.target_system_GET() == (char)108);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p47.target_component_SET((char)173) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED) ;
        p47.target_system_SET((char)108) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == -2130393489);
            assert(pack.longitude_GET() == -1531226383);
            assert(pack.time_usec_TRY(ph) == 2064291240712036473L);
            assert(pack.latitude_GET() == -127797821);
            assert(pack.target_system_GET() == (char)226);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.target_system_SET((char)226) ;
        p48.latitude_SET(-127797821) ;
        p48.altitude_SET(-2130393489) ;
        p48.time_usec_SET(2064291240712036473L, PH) ;
        p48.longitude_SET(-1531226383) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 764941444);
            assert(pack.time_usec_TRY(ph) == 5373567722952032741L);
            assert(pack.latitude_GET() == 615055188);
            assert(pack.longitude_GET() == -731173469);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.time_usec_SET(5373567722952032741L, PH) ;
        p49.longitude_SET(-731173469) ;
        p49.latitude_SET(615055188) ;
        p49.altitude_SET(764941444) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)182);
            assert(pack.param_index_GET() == (short) -13583);
            assert(pack.param_value0_GET() == 2.7397899E38F);
            assert(pack.param_value_min_GET() == 2.9875555E38F);
            assert(pack.scale_GET() == 1.6616453E38F);
            assert(pack.param_value_max_GET() == 2.4863628E38F);
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("nvijfvjzlh"));
            assert(pack.target_component_GET() == (char)37);
            assert(pack.parameter_rc_channel_index_GET() == (char)227);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.scale_SET(1.6616453E38F) ;
        p50.target_system_SET((char)182) ;
        p50.param_index_SET((short) -13583) ;
        p50.target_component_SET((char)37) ;
        p50.param_value_max_SET(2.4863628E38F) ;
        p50.param_value_min_SET(2.9875555E38F) ;
        p50.parameter_rc_channel_index_SET((char)227) ;
        p50.param_value0_SET(2.7397899E38F) ;
        p50.param_id_SET("nvijfvjzlh", PH) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)152);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.seq_GET() == (char)49449);
            assert(pack.target_component_GET() == (char)157);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p51.target_component_SET((char)157) ;
        p51.seq_SET((char)49449) ;
        p51.target_system_SET((char)152) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2x_GET() == -1.719122E37F);
            assert(pack.target_system_GET() == (char)98);
            assert(pack.p1x_GET() == -2.9602477E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.p1y_GET() == 1.2189265E38F);
            assert(pack.p2y_GET() == -2.1834353E38F);
            assert(pack.target_component_GET() == (char)130);
            assert(pack.p2z_GET() == -1.973292E38F);
            assert(pack.p1z_GET() == -1.5127885E38F);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p1y_SET(1.2189265E38F) ;
        p54.target_system_SET((char)98) ;
        p54.p1z_SET(-1.5127885E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p54.p2x_SET(-1.719122E37F) ;
        p54.p2z_SET(-1.973292E38F) ;
        p54.target_component_SET((char)130) ;
        p54.p1x_SET(-2.9602477E38F) ;
        p54.p2y_SET(-2.1834353E38F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1x_GET() == -2.753254E38F);
            assert(pack.p1y_GET() == 1.2473897E38F);
            assert(pack.p1z_GET() == -2.2111999E38F);
            assert(pack.p2z_GET() == -5.055682E37F);
            assert(pack.p2x_GET() == -1.6478079E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.p2y_GET() == -3.3216409E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2z_SET(-5.055682E37F) ;
        p55.p1z_SET(-2.2111999E38F) ;
        p55.p1x_SET(-2.753254E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p55.p2y_SET(-3.3216409E38F) ;
        p55.p2x_SET(-1.6478079E37F) ;
        p55.p1y_SET(1.2473897E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == 2.2170507E37F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.4767013E37F, -5.168947E37F, -5.393441E37F, -4.6426354E37F, 1.4997771E38F, -1.5032449E37F, 2.1003871E38F, 2.642192E38F, 6.5414057E37F}));
            assert(pack.yawspeed_GET() == -2.693355E38F);
            assert(pack.time_usec_GET() == 2336727149066681989L);
            assert(pack.pitchspeed_GET() == -1.830167E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-8.820689E37F, -2.3672237E38F, 1.0777388E38F, -1.9561149E38F}));
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.q_SET(new float[] {-8.820689E37F, -2.3672237E38F, 1.0777388E38F, -1.9561149E38F}, 0) ;
        p61.yawspeed_SET(-2.693355E38F) ;
        p61.rollspeed_SET(2.2170507E37F) ;
        p61.pitchspeed_SET(-1.830167E38F) ;
        p61.time_usec_SET(2336727149066681989L) ;
        p61.covariance_SET(new float[] {-2.4767013E37F, -5.168947E37F, -5.393441E37F, -4.6426354E37F, 1.4997771E38F, -1.5032449E37F, 2.1003871E38F, 2.642192E38F, 6.5414057E37F}, 0) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.xtrack_error_GET() == 2.759531E38F);
            assert(pack.wp_dist_GET() == (char)35715);
            assert(pack.nav_bearing_GET() == (short)2685);
            assert(pack.nav_pitch_GET() == 2.5620178E38F);
            assert(pack.target_bearing_GET() == (short) -9040);
            assert(pack.alt_error_GET() == 3.3530758E38F);
            assert(pack.nav_roll_GET() == 2.1837036E38F);
            assert(pack.aspd_error_GET() == 3.401718E38F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.target_bearing_SET((short) -9040) ;
        p62.nav_roll_SET(2.1837036E38F) ;
        p62.nav_bearing_SET((short)2685) ;
        p62.wp_dist_SET((char)35715) ;
        p62.nav_pitch_SET(2.5620178E38F) ;
        p62.alt_error_SET(3.3530758E38F) ;
        p62.xtrack_error_SET(2.759531E38F) ;
        p62.aspd_error_SET(3.401718E38F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == -2.3709402E37F);
            assert(pack.alt_GET() == -1662823195);
            assert(pack.relative_alt_GET() == -1954810003);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.6293282E38F, -2.4332513E38F, 3.0110706E38F, 2.4245544E37F, 1.4319463E38F, -2.8474777E38F, -1.4252525E38F, 8.999876E37F, -6.40109E37F, -1.4302374E38F, -2.4850164E38F, 3.1395166E38F, 2.9837684E38F, -3.2226932E38F, -6.7579797E37F, -1.5200965E38F, -1.39166E38F, 1.7982952E38F, -3.5831232E37F, 1.778174E38F, -7.5902136E37F, -2.9941846E38F, 2.4087475E38F, -9.616998E37F, -1.302765E38F, 2.6840303E38F, 1.8954392E38F, -3.2810476E37F, 2.4679735E38F, 1.942447E38F, -1.6623856E38F, 3.1730491E38F, 9.165431E36F, -1.4287515E38F, -6.379098E37F, -3.2493016E38F}));
            assert(pack.vz_GET() == -2.073388E38F);
            assert(pack.lon_GET() == -860209686);
            assert(pack.lat_GET() == 682951256);
            assert(pack.time_usec_GET() == 30672938629434761L);
            assert(pack.vy_GET() == -5.4766654E37F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.relative_alt_SET(-1954810003) ;
        p63.time_usec_SET(30672938629434761L) ;
        p63.vy_SET(-5.4766654E37F) ;
        p63.vx_SET(-2.3709402E37F) ;
        p63.covariance_SET(new float[] {-2.6293282E38F, -2.4332513E38F, 3.0110706E38F, 2.4245544E37F, 1.4319463E38F, -2.8474777E38F, -1.4252525E38F, 8.999876E37F, -6.40109E37F, -1.4302374E38F, -2.4850164E38F, 3.1395166E38F, 2.9837684E38F, -3.2226932E38F, -6.7579797E37F, -1.5200965E38F, -1.39166E38F, 1.7982952E38F, -3.5831232E37F, 1.778174E38F, -7.5902136E37F, -2.9941846E38F, 2.4087475E38F, -9.616998E37F, -1.302765E38F, 2.6840303E38F, 1.8954392E38F, -3.2810476E37F, 2.4679735E38F, 1.942447E38F, -1.6623856E38F, 3.1730491E38F, 9.165431E36F, -1.4287515E38F, -6.379098E37F, -3.2493016E38F}, 0) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION) ;
        p63.lon_SET(-860209686) ;
        p63.alt_SET(-1662823195) ;
        p63.vz_SET(-2.073388E38F) ;
        p63.lat_SET(682951256) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -2.721095E38F);
            assert(pack.time_usec_GET() == 5025223024083540520L);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.vz_GET() == -1.5878722E38F);
            assert(pack.vx_GET() == -1.1583956E38F);
            assert(pack.y_GET() == 2.8265919E38F);
            assert(pack.ax_GET() == -1.3130372E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.0841405E38F, 4.456785E37F, -4.358602E37F, 2.4411833E38F, -2.3826495E38F, 1.8580515E38F, 1.9343095E38F, -2.9279845E38F, 4.3972964E37F, 3.3459974E38F, -3.2867766E38F, 1.7384163E38F, 2.4424548E38F, 3.1155426E38F, -5.694756E37F, -1.829706E38F, 2.5510156E38F, -5.804367E37F, 2.4422688E38F, -3.3519858E38F, 3.1161482E38F, 6.0171424E37F, 3.1108136E38F, 7.7173306E37F, 2.032308E38F, -2.3782669E38F, -5.918727E36F, -2.6215128E38F, 2.0145462E37F, -1.910842E38F, 1.5311538E38F, -2.93325E36F, -1.2339086E38F, 2.7436114E37F, -1.9353625E38F, -2.4629056E38F, -3.0365087E38F, -1.4764026E38F, -1.9895249E38F, -3.9643673E37F, -2.230148E38F, -8.645641E37F, 6.4447646E37F, 3.005457E38F, 1.0333274E38F}));
            assert(pack.az_GET() == -1.2539089E38F);
            assert(pack.x_GET() == -5.2965135E37F);
            assert(pack.ay_GET() == -2.2067154E38F);
            assert(pack.vy_GET() == 3.1728313E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.z_SET(-2.721095E38F) ;
        p64.ay_SET(-2.2067154E38F) ;
        p64.covariance_SET(new float[] {2.0841405E38F, 4.456785E37F, -4.358602E37F, 2.4411833E38F, -2.3826495E38F, 1.8580515E38F, 1.9343095E38F, -2.9279845E38F, 4.3972964E37F, 3.3459974E38F, -3.2867766E38F, 1.7384163E38F, 2.4424548E38F, 3.1155426E38F, -5.694756E37F, -1.829706E38F, 2.5510156E38F, -5.804367E37F, 2.4422688E38F, -3.3519858E38F, 3.1161482E38F, 6.0171424E37F, 3.1108136E38F, 7.7173306E37F, 2.032308E38F, -2.3782669E38F, -5.918727E36F, -2.6215128E38F, 2.0145462E37F, -1.910842E38F, 1.5311538E38F, -2.93325E36F, -1.2339086E38F, 2.7436114E37F, -1.9353625E38F, -2.4629056E38F, -3.0365087E38F, -1.4764026E38F, -1.9895249E38F, -3.9643673E37F, -2.230148E38F, -8.645641E37F, 6.4447646E37F, 3.005457E38F, 1.0333274E38F}, 0) ;
        p64.y_SET(2.8265919E38F) ;
        p64.vx_SET(-1.1583956E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p64.az_SET(-1.2539089E38F) ;
        p64.time_usec_SET(5025223024083540520L) ;
        p64.vz_SET(-1.5878722E38F) ;
        p64.vy_SET(3.1728313E38F) ;
        p64.x_SET(-5.2965135E37F) ;
        p64.ax_SET(-1.3130372E38F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan18_raw_GET() == (char)17813);
            assert(pack.chan15_raw_GET() == (char)62859);
            assert(pack.chancount_GET() == (char)69);
            assert(pack.chan16_raw_GET() == (char)34154);
            assert(pack.chan6_raw_GET() == (char)39390);
            assert(pack.chan5_raw_GET() == (char)22330);
            assert(pack.chan17_raw_GET() == (char)31785);
            assert(pack.chan13_raw_GET() == (char)33918);
            assert(pack.time_boot_ms_GET() == 3677202091L);
            assert(pack.chan8_raw_GET() == (char)49515);
            assert(pack.chan1_raw_GET() == (char)51086);
            assert(pack.chan9_raw_GET() == (char)18288);
            assert(pack.chan11_raw_GET() == (char)6667);
            assert(pack.chan14_raw_GET() == (char)49186);
            assert(pack.chan7_raw_GET() == (char)28303);
            assert(pack.chan12_raw_GET() == (char)15223);
            assert(pack.chan3_raw_GET() == (char)57575);
            assert(pack.rssi_GET() == (char)217);
            assert(pack.chan4_raw_GET() == (char)19863);
            assert(pack.chan2_raw_GET() == (char)54171);
            assert(pack.chan10_raw_GET() == (char)56513);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan6_raw_SET((char)39390) ;
        p65.chan4_raw_SET((char)19863) ;
        p65.chan11_raw_SET((char)6667) ;
        p65.rssi_SET((char)217) ;
        p65.chan10_raw_SET((char)56513) ;
        p65.chan9_raw_SET((char)18288) ;
        p65.time_boot_ms_SET(3677202091L) ;
        p65.chan15_raw_SET((char)62859) ;
        p65.chan8_raw_SET((char)49515) ;
        p65.chan17_raw_SET((char)31785) ;
        p65.chan7_raw_SET((char)28303) ;
        p65.chan14_raw_SET((char)49186) ;
        p65.chan3_raw_SET((char)57575) ;
        p65.chan12_raw_SET((char)15223) ;
        p65.chan16_raw_SET((char)34154) ;
        p65.chancount_SET((char)69) ;
        p65.chan13_raw_SET((char)33918) ;
        p65.chan5_raw_SET((char)22330) ;
        p65.chan2_raw_SET((char)54171) ;
        p65.chan1_raw_SET((char)51086) ;
        p65.chan18_raw_SET((char)17813) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.start_stop_GET() == (char)113);
            assert(pack.target_component_GET() == (char)216);
            assert(pack.target_system_GET() == (char)203);
            assert(pack.req_stream_id_GET() == (char)94);
            assert(pack.req_message_rate_GET() == (char)41009);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_stream_id_SET((char)94) ;
        p66.target_component_SET((char)216) ;
        p66.req_message_rate_SET((char)41009) ;
        p66.target_system_SET((char)203) ;
        p66.start_stop_SET((char)113) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.stream_id_GET() == (char)207);
            assert(pack.message_rate_GET() == (char)2189);
            assert(pack.on_off_GET() == (char)210);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.on_off_SET((char)210) ;
        p67.message_rate_SET((char)2189) ;
        p67.stream_id_SET((char)207) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.r_GET() == (short) -5722);
            assert(pack.y_GET() == (short) -20389);
            assert(pack.buttons_GET() == (char)65050);
            assert(pack.x_GET() == (short)30979);
            assert(pack.z_GET() == (short)6735);
            assert(pack.target_GET() == (char)121);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.r_SET((short) -5722) ;
        p69.z_SET((short)6735) ;
        p69.target_SET((char)121) ;
        p69.y_SET((short) -20389) ;
        p69.buttons_SET((char)65050) ;
        p69.x_SET((short)30979) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan6_raw_GET() == (char)33467);
            assert(pack.chan4_raw_GET() == (char)57168);
            assert(pack.chan1_raw_GET() == (char)38139);
            assert(pack.target_system_GET() == (char)137);
            assert(pack.chan2_raw_GET() == (char)50943);
            assert(pack.target_component_GET() == (char)240);
            assert(pack.chan3_raw_GET() == (char)58578);
            assert(pack.chan8_raw_GET() == (char)30044);
            assert(pack.chan5_raw_GET() == (char)9559);
            assert(pack.chan7_raw_GET() == (char)8578);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan4_raw_SET((char)57168) ;
        p70.chan3_raw_SET((char)58578) ;
        p70.chan5_raw_SET((char)9559) ;
        p70.chan8_raw_SET((char)30044) ;
        p70.target_component_SET((char)240) ;
        p70.chan2_raw_SET((char)50943) ;
        p70.target_system_SET((char)137) ;
        p70.chan6_raw_SET((char)33467) ;
        p70.chan7_raw_SET((char)8578) ;
        p70.chan1_raw_SET((char)38139) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param1_GET() == 3.2221455E37F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.param2_GET() == 1.9926069E37F);
            assert(pack.param4_GET() == 2.5372931E38F);
            assert(pack.x_GET() == -638164907);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.seq_GET() == (char)40876);
            assert(pack.autocontinue_GET() == (char)108);
            assert(pack.target_component_GET() == (char)224);
            assert(pack.z_GET() == 1.4609703E38F);
            assert(pack.param3_GET() == 9.24146E35F);
            assert(pack.target_system_GET() == (char)127);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_USER_3);
            assert(pack.y_GET() == 786801845);
            assert(pack.current_GET() == (char)163);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.target_component_SET((char)224) ;
        p73.command_SET(MAV_CMD.MAV_CMD_USER_3) ;
        p73.y_SET(786801845) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p73.param3_SET(9.24146E35F) ;
        p73.param1_SET(3.2221455E37F) ;
        p73.param2_SET(1.9926069E37F) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p73.param4_SET(2.5372931E38F) ;
        p73.z_SET(1.4609703E38F) ;
        p73.autocontinue_SET((char)108) ;
        p73.current_SET((char)163) ;
        p73.target_system_SET((char)127) ;
        p73.x_SET(-638164907) ;
        p73.seq_SET((char)40876) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.groundspeed_GET() == 2.9487165E37F);
            assert(pack.heading_GET() == (short)13942);
            assert(pack.alt_GET() == 1.0205998E38F);
            assert(pack.throttle_GET() == (char)54974);
            assert(pack.climb_GET() == 1.2455399E38F);
            assert(pack.airspeed_GET() == -1.6748835E38F);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.alt_SET(1.0205998E38F) ;
        p74.airspeed_SET(-1.6748835E38F) ;
        p74.throttle_SET((char)54974) ;
        p74.heading_SET((short)13942) ;
        p74.groundspeed_SET(2.9487165E37F) ;
        p74.climb_SET(1.2455399E38F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStreamAdvanced, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.param4_GET() == 2.7096415E38F);
            assert(pack.z_GET() == 1.757763E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION);
            assert(pack.current_GET() == (char)80);
            assert(pack.y_GET() == -116516427);
            assert(pack.autocontinue_GET() == (char)203);
            assert(pack.param3_GET() == -8.115783E37F);
            assert(pack.target_system_GET() == (char)228);
            assert(pack.param2_GET() == -3.5624563E36F);
            assert(pack.target_component_GET() == (char)47);
            assert(pack.x_GET() == -2047846598);
            assert(pack.param1_GET() == -2.1737752E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
        });
        GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.x_SET(-2047846598) ;
        p75.command_SET(MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION) ;
        p75.y_SET(-116516427) ;
        p75.param1_SET(-2.1737752E38F) ;
        p75.current_SET((char)80) ;
        p75.param4_SET(2.7096415E38F) ;
        p75.param2_SET(-3.5624563E36F) ;
        p75.autocontinue_SET((char)203) ;
        p75.target_component_SET((char)47) ;
        p75.param3_SET(-8.115783E37F) ;
        p75.target_system_SET((char)228) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p75.z_SET(1.757763E38F) ;
        CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param1_GET() == -9.253529E37F);
            assert(pack.param2_GET() == 2.8424404E38F);
            assert(pack.target_component_GET() == (char)154);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_CONDITION_DISTANCE);
            assert(pack.target_system_GET() == (char)95);
            assert(pack.confirmation_GET() == (char)133);
            assert(pack.param7_GET() == 3.037876E38F);
            assert(pack.param5_GET() == 7.1584513E37F);
            assert(pack.param4_GET() == -2.6377353E38F);
            assert(pack.param6_GET() == -8.3253283E37F);
            assert(pack.param3_GET() == 1.9602107E38F);
        });
        GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param7_SET(3.037876E38F) ;
        p76.param2_SET(2.8424404E38F) ;
        p76.target_system_SET((char)95) ;
        p76.param3_SET(1.9602107E38F) ;
        p76.param5_SET(7.1584513E37F) ;
        p76.param6_SET(-8.3253283E37F) ;
        p76.confirmation_SET((char)133) ;
        p76.target_component_SET((char)154) ;
        p76.param4_SET(-2.6377353E38F) ;
        p76.param1_SET(-9.253529E37F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_CONDITION_DISTANCE) ;
        CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_TRY(ph) == (char)50);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE);
            assert(pack.result_param2_TRY(ph) == -758522496);
            assert(pack.progress_TRY(ph) == (char)172);
            assert(pack.target_component_TRY(ph) == (char)182);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_IN_PROGRESS);
        });
        GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.result_SET(MAV_RESULT.MAV_RESULT_IN_PROGRESS) ;
        p77.result_param2_SET(-758522496, PH) ;
        p77.progress_SET((char)172, PH) ;
        p77.target_system_SET((char)50, PH) ;
        p77.target_component_SET((char)182, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE) ;
        CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -2.5640287E37F);
            assert(pack.manual_override_switch_GET() == (char)91);
            assert(pack.roll_GET() == 9.866696E37F);
            assert(pack.thrust_GET() == -3.0047414E38F);
            assert(pack.mode_switch_GET() == (char)176);
            assert(pack.time_boot_ms_GET() == 2006958314L);
            assert(pack.pitch_GET() == -1.8335207E37F);
        });
        GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.pitch_SET(-1.8335207E37F) ;
        p81.roll_SET(9.866696E37F) ;
        p81.yaw_SET(-2.5640287E37F) ;
        p81.thrust_SET(-3.0047414E38F) ;
        p81.manual_override_switch_SET((char)91) ;
        p81.mode_switch_SET((char)176) ;
        p81.time_boot_ms_SET(2006958314L) ;
        CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_pitch_rate_GET() == 2.2400502E37F);
            assert(pack.time_boot_ms_GET() == 3053884105L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.6131384E38F, -1.8256582E38F, -1.5361463E38F, -8.671543E37F}));
            assert(pack.target_component_GET() == (char)168);
            assert(pack.type_mask_GET() == (char)157);
            assert(pack.target_system_GET() == (char)178);
            assert(pack.thrust_GET() == 1.4137084E37F);
            assert(pack.body_yaw_rate_GET() == -3.128037E38F);
            assert(pack.body_roll_rate_GET() == -3.1394832E38F);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.body_yaw_rate_SET(-3.128037E38F) ;
        p82.time_boot_ms_SET(3053884105L) ;
        p82.q_SET(new float[] {-1.6131384E38F, -1.8256582E38F, -1.5361463E38F, -8.671543E37F}, 0) ;
        p82.target_component_SET((char)168) ;
        p82.target_system_SET((char)178) ;
        p82.body_roll_rate_SET(-3.1394832E38F) ;
        p82.body_pitch_rate_SET(2.2400502E37F) ;
        p82.type_mask_SET((char)157) ;
        p82.thrust_SET(1.4137084E37F) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_yaw_rate_GET() == -2.0759896E38F);
            assert(pack.time_boot_ms_GET() == 602703343L);
            assert(pack.type_mask_GET() == (char)188);
            assert(pack.thrust_GET() == -2.8120883E36F);
            assert(pack.body_pitch_rate_GET() == 1.5720812E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.5284086E38F, -1.0184635E38F, -1.7841849E37F, -1.3927241E38F}));
            assert(pack.body_roll_rate_GET() == -1.1935044E38F);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_pitch_rate_SET(1.5720812E38F) ;
        p83.time_boot_ms_SET(602703343L) ;
        p83.q_SET(new float[] {2.5284086E38F, -1.0184635E38F, -1.7841849E37F, -1.3927241E38F}, 0) ;
        p83.thrust_SET(-2.8120883E36F) ;
        p83.type_mask_SET((char)188) ;
        p83.body_roll_rate_SET(-1.1935044E38F) ;
        p83.body_yaw_rate_SET(-2.0759896E38F) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)111);
            assert(pack.afz_GET() == 8.139468E37F);
            assert(pack.vz_GET() == 1.7994057E38F);
            assert(pack.x_GET() == 7.4158975E37F);
            assert(pack.y_GET() == -1.6351218E38F);
            assert(pack.z_GET() == 1.767098E38F);
            assert(pack.afy_GET() == 7.691787E37F);
            assert(pack.yaw_rate_GET() == 1.7826362E38F);
            assert(pack.type_mask_GET() == (char)42920);
            assert(pack.vx_GET() == -9.553587E37F);
            assert(pack.vy_GET() == 3.3491333E38F);
            assert(pack.afx_GET() == 2.4964247E38F);
            assert(pack.time_boot_ms_GET() == 197714464L);
            assert(pack.yaw_GET() == -7.818981E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.target_component_GET() == (char)41);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.y_SET(-1.6351218E38F) ;
        p84.afx_SET(2.4964247E38F) ;
        p84.z_SET(1.767098E38F) ;
        p84.afz_SET(8.139468E37F) ;
        p84.target_system_SET((char)111) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p84.vx_SET(-9.553587E37F) ;
        p84.yaw_rate_SET(1.7826362E38F) ;
        p84.afy_SET(7.691787E37F) ;
        p84.yaw_SET(-7.818981E37F) ;
        p84.x_SET(7.4158975E37F) ;
        p84.vy_SET(3.3491333E38F) ;
        p84.time_boot_ms_SET(197714464L) ;
        p84.target_component_SET((char)41) ;
        p84.type_mask_SET((char)42920) ;
        p84.vz_SET(1.7994057E38F) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 505739437L);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.afx_GET() == -1.1780897E38F);
            assert(pack.alt_GET() == -1.4402428E38F);
            assert(pack.target_component_GET() == (char)204);
            assert(pack.type_mask_GET() == (char)37610);
            assert(pack.vz_GET() == 1.2674021E37F);
            assert(pack.vy_GET() == -3.0888931E38F);
            assert(pack.vx_GET() == -7.8119186E37F);
            assert(pack.afz_GET() == 2.2160078E38F);
            assert(pack.lat_int_GET() == 75939639);
            assert(pack.yaw_rate_GET() == 8.603981E37F);
            assert(pack.afy_GET() == -3.1414666E38F);
            assert(pack.target_system_GET() == (char)66);
            assert(pack.lon_int_GET() == 1469494289);
            assert(pack.yaw_GET() == -2.8569263E38F);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.lon_int_SET(1469494289) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p86.vx_SET(-7.8119186E37F) ;
        p86.time_boot_ms_SET(505739437L) ;
        p86.target_component_SET((char)204) ;
        p86.yaw_SET(-2.8569263E38F) ;
        p86.afy_SET(-3.1414666E38F) ;
        p86.afz_SET(2.2160078E38F) ;
        p86.afx_SET(-1.1780897E38F) ;
        p86.yaw_rate_SET(8.603981E37F) ;
        p86.alt_SET(-1.4402428E38F) ;
        p86.vy_SET(-3.0888931E38F) ;
        p86.lat_int_SET(75939639) ;
        p86.vz_SET(1.2674021E37F) ;
        p86.type_mask_SET((char)37610) ;
        p86.target_system_SET((char)66) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3921363531L);
            assert(pack.yaw_rate_GET() == 2.5324475E38F);
            assert(pack.lat_int_GET() == 2103955890);
            assert(pack.yaw_GET() == 9.434492E37F);
            assert(pack.afy_GET() == -2.491681E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.vy_GET() == 2.9231554E38F);
            assert(pack.afx_GET() == -8.2325885E37F);
            assert(pack.vz_GET() == 2.2925084E38F);
            assert(pack.lon_int_GET() == 1827478867);
            assert(pack.vx_GET() == 1.7955163E38F);
            assert(pack.afz_GET() == -5.677533E37F);
            assert(pack.type_mask_GET() == (char)17784);
            assert(pack.alt_GET() == -2.0365218E37F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.vz_SET(2.2925084E38F) ;
        p87.vy_SET(2.9231554E38F) ;
        p87.lat_int_SET(2103955890) ;
        p87.yaw_rate_SET(2.5324475E38F) ;
        p87.alt_SET(-2.0365218E37F) ;
        p87.afy_SET(-2.491681E38F) ;
        p87.vx_SET(1.7955163E38F) ;
        p87.afx_SET(-8.2325885E37F) ;
        p87.type_mask_SET((char)17784) ;
        p87.lon_int_SET(1827478867) ;
        p87.afz_SET(-5.677533E37F) ;
        p87.yaw_SET(9.434492E37F) ;
        p87.time_boot_ms_SET(3921363531L) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -1.3303734E38F);
            assert(pack.y_GET() == 3.119912E38F);
            assert(pack.z_GET() == -1.5329198E38F);
            assert(pack.yaw_GET() == -3.836872E37F);
            assert(pack.x_GET() == 2.1098856E38F);
            assert(pack.roll_GET() == -1.1142615E38F);
            assert(pack.time_boot_ms_GET() == 2683414485L);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.y_SET(3.119912E38F) ;
        p89.yaw_SET(-3.836872E37F) ;
        p89.pitch_SET(-1.3303734E38F) ;
        p89.x_SET(2.1098856E38F) ;
        p89.roll_SET(-1.1142615E38F) ;
        p89.z_SET(-1.5329198E38F) ;
        p89.time_boot_ms_SET(2683414485L) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -2.9072417E38F);
            assert(pack.vz_GET() == (short)31952);
            assert(pack.yacc_GET() == (short)8925);
            assert(pack.zacc_GET() == (short) -14003);
            assert(pack.roll_GET() == 1.7883869E38F);
            assert(pack.xacc_GET() == (short) -4993);
            assert(pack.time_usec_GET() == 6570290341736976159L);
            assert(pack.yaw_GET() == 1.4154459E38F);
            assert(pack.vx_GET() == (short)16888);
            assert(pack.lon_GET() == -246506181);
            assert(pack.rollspeed_GET() == -2.9495106E38F);
            assert(pack.vy_GET() == (short)3311);
            assert(pack.alt_GET() == 1158775255);
            assert(pack.yawspeed_GET() == -1.9668773E38F);
            assert(pack.lat_GET() == -32617872);
            assert(pack.pitchspeed_GET() == -7.060253E37F);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.yacc_SET((short)8925) ;
        p90.pitchspeed_SET(-7.060253E37F) ;
        p90.rollspeed_SET(-2.9495106E38F) ;
        p90.vz_SET((short)31952) ;
        p90.pitch_SET(-2.9072417E38F) ;
        p90.roll_SET(1.7883869E38F) ;
        p90.yaw_SET(1.4154459E38F) ;
        p90.vy_SET((short)3311) ;
        p90.vx_SET((short)16888) ;
        p90.lon_SET(-246506181) ;
        p90.xacc_SET((short) -4993) ;
        p90.zacc_SET((short) -14003) ;
        p90.yawspeed_SET(-1.9668773E38F) ;
        p90.time_usec_SET(6570290341736976159L) ;
        p90.lat_SET(-32617872) ;
        p90.alt_SET(1158775255) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux1_GET() == 3.3780175E38F);
            assert(pack.yaw_rudder_GET() == 5.619262E37F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_DISARMED);
            assert(pack.pitch_elevator_GET() == -2.4793607E38F);
            assert(pack.nav_mode_GET() == (char)147);
            assert(pack.aux3_GET() == 3.0178403E37F);
            assert(pack.throttle_GET() == 2.9440238E38F);
            assert(pack.roll_ailerons_GET() == -5.556127E37F);
            assert(pack.aux4_GET() == -4.868444E37F);
            assert(pack.time_usec_GET() == 4128813998635453653L);
            assert(pack.aux2_GET() == -9.171033E37F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.aux3_SET(3.0178403E37F) ;
        p91.yaw_rudder_SET(5.619262E37F) ;
        p91.aux4_SET(-4.868444E37F) ;
        p91.throttle_SET(2.9440238E38F) ;
        p91.time_usec_SET(4128813998635453653L) ;
        p91.roll_ailerons_SET(-5.556127E37F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED) ;
        p91.pitch_elevator_SET(-2.4793607E38F) ;
        p91.aux2_SET(-9.171033E37F) ;
        p91.aux1_SET(3.3780175E38F) ;
        p91.nav_mode_SET((char)147) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan12_raw_GET() == (char)48821);
            assert(pack.chan4_raw_GET() == (char)17425);
            assert(pack.chan11_raw_GET() == (char)49232);
            assert(pack.chan6_raw_GET() == (char)38856);
            assert(pack.chan8_raw_GET() == (char)43998);
            assert(pack.time_usec_GET() == 354503603843349499L);
            assert(pack.chan10_raw_GET() == (char)41588);
            assert(pack.chan7_raw_GET() == (char)40498);
            assert(pack.rssi_GET() == (char)97);
            assert(pack.chan5_raw_GET() == (char)60963);
            assert(pack.chan3_raw_GET() == (char)16473);
            assert(pack.chan1_raw_GET() == (char)63535);
            assert(pack.chan9_raw_GET() == (char)55208);
            assert(pack.chan2_raw_GET() == (char)61836);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.rssi_SET((char)97) ;
        p92.chan3_raw_SET((char)16473) ;
        p92.chan8_raw_SET((char)43998) ;
        p92.chan4_raw_SET((char)17425) ;
        p92.chan10_raw_SET((char)41588) ;
        p92.chan2_raw_SET((char)61836) ;
        p92.chan9_raw_SET((char)55208) ;
        p92.chan1_raw_SET((char)63535) ;
        p92.chan11_raw_SET((char)49232) ;
        p92.chan12_raw_SET((char)48821) ;
        p92.time_usec_SET(354503603843349499L) ;
        p92.chan6_raw_SET((char)38856) ;
        p92.chan7_raw_SET((char)40498) ;
        p92.chan5_raw_SET((char)60963) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4857225193226235037L);
            assert(pack.flags_GET() == 3696570912068844757L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_MANUAL_ARMED);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {3.3195926E38F, -2.1794763E38F, 1.0457506E38F, 2.0458477E38F, -2.1731683E38F, -1.1466622E37F, -1.9512465E38F, 2.8894505E38F, -3.7706637E37F, -4.577246E37F, -1.9607407E38F, 1.1901019E38F, 3.2751383E37F, -2.2121203E38F, -6.955362E37F, 1.9325686E38F}));
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.controls_SET(new float[] {3.3195926E38F, -2.1794763E38F, 1.0457506E38F, 2.0458477E38F, -2.1731683E38F, -1.1466622E37F, -1.9512465E38F, 2.8894505E38F, -3.7706637E37F, -4.577246E37F, -1.9607407E38F, 1.1901019E38F, 3.2751383E37F, -2.2121203E38F, -6.955362E37F, 1.9325686E38F}, 0) ;
        p93.time_usec_SET(4857225193226235037L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_MANUAL_ARMED) ;
        p93.flags_SET(3696570912068844757L) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.sensor_id_GET() == (char)1);
            assert(pack.flow_rate_x_TRY(ph) == 2.5738453E38F);
            assert(pack.flow_comp_m_y_GET() == -2.1996843E38F);
            assert(pack.flow_x_GET() == (short)19570);
            assert(pack.flow_y_GET() == (short) -28869);
            assert(pack.time_usec_GET() == 1086301419100737051L);
            assert(pack.ground_distance_GET() == -3.5913928E37F);
            assert(pack.flow_rate_y_TRY(ph) == 8.523754E37F);
            assert(pack.flow_comp_m_x_GET() == -1.6555828E38F);
            assert(pack.quality_GET() == (char)73);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.quality_SET((char)73) ;
        p100.flow_comp_m_x_SET(-1.6555828E38F) ;
        p100.sensor_id_SET((char)1) ;
        p100.ground_distance_SET(-3.5913928E37F) ;
        p100.flow_x_SET((short)19570) ;
        p100.flow_rate_y_SET(8.523754E37F, PH) ;
        p100.flow_rate_x_SET(2.5738453E38F, PH) ;
        p100.flow_y_SET((short) -28869) ;
        p100.time_usec_SET(1086301419100737051L) ;
        p100.flow_comp_m_y_SET(-2.1996843E38F) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 1.117352E37F);
            assert(pack.yaw_GET() == -3.1781467E38F);
            assert(pack.z_GET() == -1.827593E38F);
            assert(pack.x_GET() == -2.6433971E38F);
            assert(pack.usec_GET() == 7323346893562412641L);
            assert(pack.y_GET() == 1.9488773E38F);
            assert(pack.roll_GET() == -3.3691141E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.yaw_SET(-3.1781467E38F) ;
        p101.roll_SET(-3.3691141E38F) ;
        p101.y_SET(1.9488773E38F) ;
        p101.z_SET(-1.827593E38F) ;
        p101.x_SET(-2.6433971E38F) ;
        p101.pitch_SET(1.117352E37F) ;
        p101.usec_SET(7323346893562412641L) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -7.3813824E37F);
            assert(pack.usec_GET() == 7372032473741103387L);
            assert(pack.z_GET() == 2.1252558E38F);
            assert(pack.roll_GET() == -3.1185207E38F);
            assert(pack.y_GET() == -1.5797628E38F);
            assert(pack.pitch_GET() == -1.401167E38F);
            assert(pack.x_GET() == -1.6751322E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.pitch_SET(-1.401167E38F) ;
        p102.x_SET(-1.6751322E38F) ;
        p102.usec_SET(7372032473741103387L) ;
        p102.y_SET(-1.5797628E38F) ;
        p102.yaw_SET(-7.3813824E37F) ;
        p102.z_SET(2.1252558E38F) ;
        p102.roll_SET(-3.1185207E38F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 5138332286568523686L);
            assert(pack.x_GET() == 1.4656233E37F);
            assert(pack.z_GET() == -7.8107357E37F);
            assert(pack.y_GET() == -2.5972315E38F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.z_SET(-7.8107357E37F) ;
        p103.x_SET(1.4656233E37F) ;
        p103.usec_SET(5138332286568523686L) ;
        p103.y_SET(-2.5972315E38F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 4833573482294705131L);
            assert(pack.z_GET() == 1.1981383E38F);
            assert(pack.yaw_GET() == 1.7598872E38F);
            assert(pack.x_GET() == 1.9322399E38F);
            assert(pack.roll_GET() == -1.8842486E38F);
            assert(pack.pitch_GET() == 2.222974E38F);
            assert(pack.y_GET() == 5.351203E37F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(4833573482294705131L) ;
        p104.x_SET(1.9322399E38F) ;
        p104.roll_SET(-1.8842486E38F) ;
        p104.yaw_SET(1.7598872E38F) ;
        p104.z_SET(1.1981383E38F) ;
        p104.y_SET(5.351203E37F) ;
        p104.pitch_SET(2.222974E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == -3.346702E38F);
            assert(pack.xacc_GET() == -2.102574E38F);
            assert(pack.pressure_alt_GET() == 1.7249893E38F);
            assert(pack.temperature_GET() == 1.8776118E38F);
            assert(pack.ymag_GET() == -1.669496E38F);
            assert(pack.yacc_GET() == -2.4591906E38F);
            assert(pack.zmag_GET() == 9.795715E37F);
            assert(pack.diff_pressure_GET() == 1.8206118E38F);
            assert(pack.xgyro_GET() == 3.278677E38F);
            assert(pack.abs_pressure_GET() == -9.762057E37F);
            assert(pack.zgyro_GET() == -2.1105113E38F);
            assert(pack.time_usec_GET() == 624202846457287940L);
            assert(pack.fields_updated_GET() == (char)53413);
            assert(pack.zacc_GET() == 3.1539218E38F);
            assert(pack.ygyro_GET() == 3.3975539E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.ymag_SET(-1.669496E38F) ;
        p105.zacc_SET(3.1539218E38F) ;
        p105.diff_pressure_SET(1.8206118E38F) ;
        p105.abs_pressure_SET(-9.762057E37F) ;
        p105.fields_updated_SET((char)53413) ;
        p105.time_usec_SET(624202846457287940L) ;
        p105.zmag_SET(9.795715E37F) ;
        p105.xmag_SET(-3.346702E38F) ;
        p105.ygyro_SET(3.3975539E38F) ;
        p105.pressure_alt_SET(1.7249893E38F) ;
        p105.yacc_SET(-2.4591906E38F) ;
        p105.xgyro_SET(3.278677E38F) ;
        p105.zgyro_SET(-2.1105113E38F) ;
        p105.temperature_SET(1.8776118E38F) ;
        p105.xacc_SET(-2.102574E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.time_delta_distance_us_GET() == 4036565023L);
            assert(pack.time_usec_GET() == 6620662751758710283L);
            assert(pack.sensor_id_GET() == (char)203);
            assert(pack.integrated_y_GET() == -1.7273804E38F);
            assert(pack.quality_GET() == (char)204);
            assert(pack.integration_time_us_GET() == 3376315302L);
            assert(pack.integrated_ygyro_GET() == -5.320639E36F);
            assert(pack.integrated_zgyro_GET() == -2.4766854E38F);
            assert(pack.integrated_xgyro_GET() == 2.5309664E38F);
            assert(pack.temperature_GET() == (short) -2837);
            assert(pack.integrated_x_GET() == 2.8791678E38F);
            assert(pack.distance_GET() == 8.2825973E37F);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integration_time_us_SET(3376315302L) ;
        p106.quality_SET((char)204) ;
        p106.time_usec_SET(6620662751758710283L) ;
        p106.sensor_id_SET((char)203) ;
        p106.time_delta_distance_us_SET(4036565023L) ;
        p106.integrated_ygyro_SET(-5.320639E36F) ;
        p106.integrated_x_SET(2.8791678E38F) ;
        p106.distance_SET(8.2825973E37F) ;
        p106.integrated_zgyro_SET(-2.4766854E38F) ;
        p106.integrated_xgyro_SET(2.5309664E38F) ;
        p106.integrated_y_SET(-1.7273804E38F) ;
        p106.temperature_SET((short) -2837) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4819752139511265047L);
            assert(pack.xacc_GET() == -5.230802E37F);
            assert(pack.fields_updated_GET() == 4080943727L);
            assert(pack.ygyro_GET() == 1.7939968E38F);
            assert(pack.pressure_alt_GET() == -5.476415E36F);
            assert(pack.xgyro_GET() == 2.9510353E37F);
            assert(pack.temperature_GET() == -2.0571831E38F);
            assert(pack.yacc_GET() == -1.0902015E38F);
            assert(pack.diff_pressure_GET() == -1.1682687E38F);
            assert(pack.zgyro_GET() == -2.2828704E38F);
            assert(pack.zacc_GET() == 1.553582E38F);
            assert(pack.ymag_GET() == -1.345025E36F);
            assert(pack.xmag_GET() == 2.9835962E38F);
            assert(pack.zmag_GET() == 2.985448E38F);
            assert(pack.abs_pressure_GET() == -1.2904244E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.pressure_alt_SET(-5.476415E36F) ;
        p107.zacc_SET(1.553582E38F) ;
        p107.temperature_SET(-2.0571831E38F) ;
        p107.ymag_SET(-1.345025E36F) ;
        p107.xacc_SET(-5.230802E37F) ;
        p107.xmag_SET(2.9835962E38F) ;
        p107.diff_pressure_SET(-1.1682687E38F) ;
        p107.abs_pressure_SET(-1.2904244E38F) ;
        p107.yacc_SET(-1.0902015E38F) ;
        p107.xgyro_SET(2.9510353E37F) ;
        p107.ygyro_SET(1.7939968E38F) ;
        p107.fields_updated_SET(4080943727L) ;
        p107.time_usec_SET(4819752139511265047L) ;
        p107.zmag_SET(2.985448E38F) ;
        p107.zgyro_SET(-2.2828704E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -3.0295426E38F);
            assert(pack.q1_GET() == -1.8082547E38F);
            assert(pack.alt_GET() == 2.5638162E38F);
            assert(pack.q2_GET() == -2.5309423E38F);
            assert(pack.lon_GET() == 1.6513052E38F);
            assert(pack.q4_GET() == -1.0532965E38F);
            assert(pack.yacc_GET() == -3.6285373E37F);
            assert(pack.ve_GET() == -2.1061115E37F);
            assert(pack.roll_GET() == -4.0261382E37F);
            assert(pack.yaw_GET() == 1.6148689E38F);
            assert(pack.pitch_GET() == 8.398606E37F);
            assert(pack.zgyro_GET() == 1.7240535E38F);
            assert(pack.std_dev_horz_GET() == -3.2533618E38F);
            assert(pack.vn_GET() == -2.7377462E38F);
            assert(pack.zacc_GET() == 3.245604E38F);
            assert(pack.ygyro_GET() == -8.690393E37F);
            assert(pack.xacc_GET() == 2.4127007E38F);
            assert(pack.xgyro_GET() == -2.8658288E38F);
            assert(pack.vd_GET() == -1.5047916E38F);
            assert(pack.q3_GET() == 5.7106356E37F);
            assert(pack.std_dev_vert_GET() == -3.9597812E37F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.q3_SET(5.7106356E37F) ;
        p108.alt_SET(2.5638162E38F) ;
        p108.q2_SET(-2.5309423E38F) ;
        p108.q1_SET(-1.8082547E38F) ;
        p108.lat_SET(-3.0295426E38F) ;
        p108.roll_SET(-4.0261382E37F) ;
        p108.yacc_SET(-3.6285373E37F) ;
        p108.ygyro_SET(-8.690393E37F) ;
        p108.lon_SET(1.6513052E38F) ;
        p108.pitch_SET(8.398606E37F) ;
        p108.yaw_SET(1.6148689E38F) ;
        p108.vn_SET(-2.7377462E38F) ;
        p108.zacc_SET(3.245604E38F) ;
        p108.ve_SET(-2.1061115E37F) ;
        p108.q4_SET(-1.0532965E38F) ;
        p108.std_dev_horz_SET(-3.2533618E38F) ;
        p108.vd_SET(-1.5047916E38F) ;
        p108.xacc_SET(2.4127007E38F) ;
        p108.zgyro_SET(1.7240535E38F) ;
        p108.std_dev_vert_SET(-3.9597812E37F) ;
        p108.xgyro_SET(-2.8658288E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.fixed__GET() == (char)48406);
            assert(pack.remrssi_GET() == (char)107);
            assert(pack.noise_GET() == (char)167);
            assert(pack.txbuf_GET() == (char)239);
            assert(pack.remnoise_GET() == (char)241);
            assert(pack.rssi_GET() == (char)196);
            assert(pack.rxerrors_GET() == (char)7861);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.noise_SET((char)167) ;
        p109.fixed__SET((char)48406) ;
        p109.rssi_SET((char)196) ;
        p109.rxerrors_SET((char)7861) ;
        p109.remnoise_SET((char)241) ;
        p109.remrssi_SET((char)107) ;
        p109.txbuf_SET((char)239) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)244, (char)150, (char)151, (char)75, (char)216, (char)220, (char)47, (char)39, (char)42, (char)5, (char)173, (char)61, (char)124, (char)137, (char)114, (char)51, (char)113, (char)200, (char)85, (char)184, (char)113, (char)236, (char)221, (char)114, (char)251, (char)96, (char)173, (char)220, (char)11, (char)181, (char)217, (char)10, (char)99, (char)20, (char)91, (char)206, (char)142, (char)184, (char)27, (char)130, (char)188, (char)72, (char)30, (char)203, (char)110, (char)27, (char)236, (char)50, (char)251, (char)24, (char)41, (char)110, (char)47, (char)235, (char)78, (char)185, (char)206, (char)228, (char)36, (char)31, (char)177, (char)43, (char)206, (char)164, (char)43, (char)185, (char)233, (char)114, (char)196, (char)27, (char)154, (char)192, (char)85, (char)152, (char)119, (char)125, (char)248, (char)50, (char)13, (char)90, (char)157, (char)87, (char)152, (char)248, (char)230, (char)65, (char)107, (char)41, (char)218, (char)100, (char)225, (char)149, (char)201, (char)198, (char)83, (char)149, (char)206, (char)219, (char)71, (char)33, (char)227, (char)86, (char)26, (char)230, (char)166, (char)145, (char)89, (char)71, (char)40, (char)211, (char)186, (char)20, (char)232, (char)216, (char)36, (char)13, (char)136, (char)211, (char)217, (char)34, (char)31, (char)66, (char)192, (char)132, (char)34, (char)198, (char)135, (char)209, (char)89, (char)226, (char)183, (char)134, (char)149, (char)86, (char)178, (char)223, (char)188, (char)82, (char)250, (char)173, (char)221, (char)53, (char)56, (char)160, (char)159, (char)211, (char)252, (char)237, (char)221, (char)105, (char)164, (char)177, (char)128, (char)167, (char)47, (char)41, (char)151, (char)95, (char)18, (char)65, (char)215, (char)236, (char)77, (char)12, (char)48, (char)246, (char)80, (char)246, (char)100, (char)30, (char)91, (char)166, (char)222, (char)31, (char)155, (char)232, (char)82, (char)9, (char)195, (char)182, (char)46, (char)95, (char)157, (char)76, (char)41, (char)63, (char)55, (char)76, (char)162, (char)54, (char)71, (char)249, (char)133, (char)216, (char)181, (char)168, (char)45, (char)2, (char)154, (char)88, (char)242, (char)3, (char)156, (char)150, (char)21, (char)16, (char)158, (char)246, (char)141, (char)180, (char)164, (char)245, (char)120, (char)112, (char)125, (char)183, (char)97, (char)218, (char)35, (char)124, (char)235, (char)157, (char)57, (char)109, (char)160, (char)53, (char)221, (char)152, (char)154, (char)108, (char)242, (char)17, (char)119, (char)79, (char)242, (char)227, (char)40, (char)69, (char)19, (char)94, (char)124, (char)170, (char)165, (char)194, (char)248, (char)248, (char)45, (char)18, (char)255, (char)72, (char)187}));
            assert(pack.target_network_GET() == (char)67);
            assert(pack.target_component_GET() == (char)32);
            assert(pack.target_system_GET() == (char)60);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_system_SET((char)60) ;
        p110.payload_SET(new char[] {(char)244, (char)150, (char)151, (char)75, (char)216, (char)220, (char)47, (char)39, (char)42, (char)5, (char)173, (char)61, (char)124, (char)137, (char)114, (char)51, (char)113, (char)200, (char)85, (char)184, (char)113, (char)236, (char)221, (char)114, (char)251, (char)96, (char)173, (char)220, (char)11, (char)181, (char)217, (char)10, (char)99, (char)20, (char)91, (char)206, (char)142, (char)184, (char)27, (char)130, (char)188, (char)72, (char)30, (char)203, (char)110, (char)27, (char)236, (char)50, (char)251, (char)24, (char)41, (char)110, (char)47, (char)235, (char)78, (char)185, (char)206, (char)228, (char)36, (char)31, (char)177, (char)43, (char)206, (char)164, (char)43, (char)185, (char)233, (char)114, (char)196, (char)27, (char)154, (char)192, (char)85, (char)152, (char)119, (char)125, (char)248, (char)50, (char)13, (char)90, (char)157, (char)87, (char)152, (char)248, (char)230, (char)65, (char)107, (char)41, (char)218, (char)100, (char)225, (char)149, (char)201, (char)198, (char)83, (char)149, (char)206, (char)219, (char)71, (char)33, (char)227, (char)86, (char)26, (char)230, (char)166, (char)145, (char)89, (char)71, (char)40, (char)211, (char)186, (char)20, (char)232, (char)216, (char)36, (char)13, (char)136, (char)211, (char)217, (char)34, (char)31, (char)66, (char)192, (char)132, (char)34, (char)198, (char)135, (char)209, (char)89, (char)226, (char)183, (char)134, (char)149, (char)86, (char)178, (char)223, (char)188, (char)82, (char)250, (char)173, (char)221, (char)53, (char)56, (char)160, (char)159, (char)211, (char)252, (char)237, (char)221, (char)105, (char)164, (char)177, (char)128, (char)167, (char)47, (char)41, (char)151, (char)95, (char)18, (char)65, (char)215, (char)236, (char)77, (char)12, (char)48, (char)246, (char)80, (char)246, (char)100, (char)30, (char)91, (char)166, (char)222, (char)31, (char)155, (char)232, (char)82, (char)9, (char)195, (char)182, (char)46, (char)95, (char)157, (char)76, (char)41, (char)63, (char)55, (char)76, (char)162, (char)54, (char)71, (char)249, (char)133, (char)216, (char)181, (char)168, (char)45, (char)2, (char)154, (char)88, (char)242, (char)3, (char)156, (char)150, (char)21, (char)16, (char)158, (char)246, (char)141, (char)180, (char)164, (char)245, (char)120, (char)112, (char)125, (char)183, (char)97, (char)218, (char)35, (char)124, (char)235, (char)157, (char)57, (char)109, (char)160, (char)53, (char)221, (char)152, (char)154, (char)108, (char)242, (char)17, (char)119, (char)79, (char)242, (char)227, (char)40, (char)69, (char)19, (char)94, (char)124, (char)170, (char)165, (char)194, (char)248, (char)248, (char)45, (char)18, (char)255, (char)72, (char)187}, 0) ;
        p110.target_component_SET((char)32) ;
        p110.target_network_SET((char)67) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -1802212570094784322L);
            assert(pack.ts1_GET() == 109281611540888901L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(109281611540888901L) ;
        p111.tc1_SET(-1802212570094784322L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 9049645929299475834L);
            assert(pack.seq_GET() == 285906581L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(285906581L) ;
        p112.time_usec_SET(9049645929299475834L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.vel_GET() == (char)20608);
            assert(pack.eph_GET() == (char)47078);
            assert(pack.fix_type_GET() == (char)254);
            assert(pack.lat_GET() == 741487314);
            assert(pack.epv_GET() == (char)34061);
            assert(pack.ve_GET() == (short)19002);
            assert(pack.cog_GET() == (char)46219);
            assert(pack.vd_GET() == (short)24050);
            assert(pack.time_usec_GET() == 1418000461594394942L);
            assert(pack.vn_GET() == (short) -18923);
            assert(pack.satellites_visible_GET() == (char)185);
            assert(pack.lon_GET() == -723993004);
            assert(pack.alt_GET() == 1816861775);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.eph_SET((char)47078) ;
        p113.epv_SET((char)34061) ;
        p113.ve_SET((short)19002) ;
        p113.satellites_visible_SET((char)185) ;
        p113.fix_type_SET((char)254) ;
        p113.vd_SET((short)24050) ;
        p113.lon_SET(-723993004) ;
        p113.vn_SET((short) -18923) ;
        p113.time_usec_SET(1418000461594394942L) ;
        p113.alt_SET(1816861775) ;
        p113.cog_SET((char)46219) ;
        p113.vel_SET((char)20608) ;
        p113.lat_SET(741487314) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_ygyro_GET() == 1.9992016E38F);
            assert(pack.integration_time_us_GET() == 2613880671L);
            assert(pack.integrated_x_GET() == -3.2987638E37F);
            assert(pack.distance_GET() == 5.343076E37F);
            assert(pack.integrated_zgyro_GET() == -2.0500847E38F);
            assert(pack.integrated_y_GET() == -1.2468147E38F);
            assert(pack.temperature_GET() == (short) -6383);
            assert(pack.sensor_id_GET() == (char)111);
            assert(pack.time_delta_distance_us_GET() == 4040722887L);
            assert(pack.time_usec_GET() == 1652095285693967229L);
            assert(pack.integrated_xgyro_GET() == 1.4588821E38F);
            assert(pack.quality_GET() == (char)212);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.temperature_SET((short) -6383) ;
        p114.integration_time_us_SET(2613880671L) ;
        p114.integrated_ygyro_SET(1.9992016E38F) ;
        p114.time_usec_SET(1652095285693967229L) ;
        p114.integrated_xgyro_SET(1.4588821E38F) ;
        p114.integrated_zgyro_SET(-2.0500847E38F) ;
        p114.sensor_id_SET((char)111) ;
        p114.time_delta_distance_us_SET(4040722887L) ;
        p114.quality_SET((char)212) ;
        p114.integrated_y_SET(-1.2468147E38F) ;
        p114.distance_SET(5.343076E37F) ;
        p114.integrated_x_SET(-3.2987638E37F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {-7.889078E37F, -3.3177254E38F, 1.4419764E38F, -8.0166067E37F}));
            assert(pack.vy_GET() == (short) -9811);
            assert(pack.alt_GET() == 1123785386);
            assert(pack.zacc_GET() == (short)29957);
            assert(pack.vx_GET() == (short)20536);
            assert(pack.lat_GET() == 968931616);
            assert(pack.time_usec_GET() == 3249858859684456097L);
            assert(pack.yawspeed_GET() == 2.1625089E38F);
            assert(pack.pitchspeed_GET() == 1.8465826E38F);
            assert(pack.vz_GET() == (short) -26494);
            assert(pack.xacc_GET() == (short) -18635);
            assert(pack.rollspeed_GET() == 4.957123E37F);
            assert(pack.lon_GET() == -1014987031);
            assert(pack.ind_airspeed_GET() == (char)49838);
            assert(pack.true_airspeed_GET() == (char)59183);
            assert(pack.yacc_GET() == (short) -17553);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.yawspeed_SET(2.1625089E38F) ;
        p115.time_usec_SET(3249858859684456097L) ;
        p115.lat_SET(968931616) ;
        p115.vz_SET((short) -26494) ;
        p115.true_airspeed_SET((char)59183) ;
        p115.vx_SET((short)20536) ;
        p115.zacc_SET((short)29957) ;
        p115.ind_airspeed_SET((char)49838) ;
        p115.attitude_quaternion_SET(new float[] {-7.889078E37F, -3.3177254E38F, 1.4419764E38F, -8.0166067E37F}, 0) ;
        p115.pitchspeed_SET(1.8465826E38F) ;
        p115.alt_SET(1123785386) ;
        p115.vy_SET((short) -9811) ;
        p115.yacc_SET((short) -17553) ;
        p115.lon_SET(-1014987031) ;
        p115.rollspeed_SET(4.957123E37F) ;
        p115.xacc_SET((short) -18635) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == (short)9659);
            assert(pack.ymag_GET() == (short)7615);
            assert(pack.zgyro_GET() == (short) -9755);
            assert(pack.ygyro_GET() == (short) -14036);
            assert(pack.yacc_GET() == (short)14693);
            assert(pack.zacc_GET() == (short) -2326);
            assert(pack.xacc_GET() == (short) -27475);
            assert(pack.time_boot_ms_GET() == 2555842535L);
            assert(pack.xgyro_GET() == (short)27656);
            assert(pack.xmag_GET() == (short)7301);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.xmag_SET((short)7301) ;
        p116.ymag_SET((short)7615) ;
        p116.ygyro_SET((short) -14036) ;
        p116.xgyro_SET((short)27656) ;
        p116.zmag_SET((short)9659) ;
        p116.zacc_SET((short) -2326) ;
        p116.zgyro_SET((short) -9755) ;
        p116.yacc_SET((short)14693) ;
        p116.xacc_SET((short) -27475) ;
        p116.time_boot_ms_SET(2555842535L) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)68);
            assert(pack.start_GET() == (char)38312);
            assert(pack.end_GET() == (char)63831);
            assert(pack.target_system_GET() == (char)19);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_component_SET((char)68) ;
        p117.end_SET((char)63831) ;
        p117.start_SET((char)38312) ;
        p117.target_system_SET((char)19) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.num_logs_GET() == (char)14993);
            assert(pack.last_log_num_GET() == (char)12060);
            assert(pack.id_GET() == (char)25766);
            assert(pack.size_GET() == 938719633L);
            assert(pack.time_utc_GET() == 975145311L);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)25766) ;
        p118.num_logs_SET((char)14993) ;
        p118.last_log_num_SET((char)12060) ;
        p118.size_SET(938719633L) ;
        p118.time_utc_SET(975145311L) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)192);
            assert(pack.ofs_GET() == 129838030L);
            assert(pack.target_component_GET() == (char)105);
            assert(pack.id_GET() == (char)49517);
            assert(pack.count_GET() == 4028757564L);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_component_SET((char)105) ;
        p119.count_SET(4028757564L) ;
        p119.target_system_SET((char)192) ;
        p119.id_SET((char)49517) ;
        p119.ofs_SET(129838030L) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)53850);
            assert(pack.ofs_GET() == 3489566017L);
            assert(pack.count_GET() == (char)221);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)240, (char)32, (char)100, (char)95, (char)214, (char)18, (char)107, (char)59, (char)135, (char)74, (char)91, (char)136, (char)220, (char)39, (char)25, (char)219, (char)224, (char)143, (char)20, (char)253, (char)39, (char)56, (char)218, (char)147, (char)149, (char)158, (char)50, (char)88, (char)92, (char)161, (char)35, (char)28, (char)250, (char)104, (char)119, (char)255, (char)191, (char)201, (char)135, (char)255, (char)27, (char)174, (char)39, (char)30, (char)70, (char)148, (char)106, (char)228, (char)193, (char)132, (char)150, (char)60, (char)203, (char)159, (char)24, (char)84, (char)187, (char)154, (char)93, (char)66, (char)231, (char)143, (char)155, (char)168, (char)254, (char)244, (char)195, (char)235, (char)54, (char)80, (char)206, (char)119, (char)157, (char)62, (char)40, (char)175, (char)206, (char)201, (char)145, (char)81, (char)167, (char)11, (char)130, (char)63, (char)13, (char)203, (char)7, (char)35, (char)30, (char)69}));
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.ofs_SET(3489566017L) ;
        p120.count_SET((char)221) ;
        p120.id_SET((char)53850) ;
        p120.data__SET(new char[] {(char)240, (char)32, (char)100, (char)95, (char)214, (char)18, (char)107, (char)59, (char)135, (char)74, (char)91, (char)136, (char)220, (char)39, (char)25, (char)219, (char)224, (char)143, (char)20, (char)253, (char)39, (char)56, (char)218, (char)147, (char)149, (char)158, (char)50, (char)88, (char)92, (char)161, (char)35, (char)28, (char)250, (char)104, (char)119, (char)255, (char)191, (char)201, (char)135, (char)255, (char)27, (char)174, (char)39, (char)30, (char)70, (char)148, (char)106, (char)228, (char)193, (char)132, (char)150, (char)60, (char)203, (char)159, (char)24, (char)84, (char)187, (char)154, (char)93, (char)66, (char)231, (char)143, (char)155, (char)168, (char)254, (char)244, (char)195, (char)235, (char)54, (char)80, (char)206, (char)119, (char)157, (char)62, (char)40, (char)175, (char)206, (char)201, (char)145, (char)81, (char)167, (char)11, (char)130, (char)63, (char)13, (char)203, (char)7, (char)35, (char)30, (char)69}, 0) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)146);
            assert(pack.target_component_GET() == (char)164);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)146) ;
        p121.target_component_SET((char)164) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)10);
            assert(pack.target_system_GET() == (char)170);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_component_SET((char)10) ;
        p122.target_system_SET((char)170) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)135, (char)4, (char)36, (char)125, (char)80, (char)245, (char)64, (char)170, (char)125, (char)54, (char)255, (char)244, (char)127, (char)73, (char)153, (char)149, (char)160, (char)168, (char)214, (char)236, (char)110, (char)176, (char)75, (char)24, (char)94, (char)131, (char)46, (char)186, (char)56, (char)50, (char)223, (char)86, (char)174, (char)133, (char)168, (char)215, (char)144, (char)116, (char)88, (char)123, (char)65, (char)242, (char)183, (char)198, (char)204, (char)128, (char)192, (char)215, (char)164, (char)182, (char)25, (char)104, (char)92, (char)151, (char)55, (char)235, (char)36, (char)10, (char)137, (char)80, (char)93, (char)106, (char)80, (char)153, (char)238, (char)47, (char)28, (char)35, (char)78, (char)169, (char)213, (char)222, (char)148, (char)191, (char)128, (char)101, (char)136, (char)241, (char)92, (char)177, (char)54, (char)46, (char)67, (char)55, (char)56, (char)211, (char)52, (char)188, (char)65, (char)118, (char)202, (char)110, (char)42, (char)60, (char)190, (char)228, (char)177, (char)8, (char)194, (char)177, (char)88, (char)215, (char)25, (char)114, (char)53, (char)168, (char)99, (char)96, (char)245, (char)84}));
            assert(pack.target_system_GET() == (char)169);
            assert(pack.len_GET() == (char)150);
            assert(pack.target_component_GET() == (char)149);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.data__SET(new char[] {(char)135, (char)4, (char)36, (char)125, (char)80, (char)245, (char)64, (char)170, (char)125, (char)54, (char)255, (char)244, (char)127, (char)73, (char)153, (char)149, (char)160, (char)168, (char)214, (char)236, (char)110, (char)176, (char)75, (char)24, (char)94, (char)131, (char)46, (char)186, (char)56, (char)50, (char)223, (char)86, (char)174, (char)133, (char)168, (char)215, (char)144, (char)116, (char)88, (char)123, (char)65, (char)242, (char)183, (char)198, (char)204, (char)128, (char)192, (char)215, (char)164, (char)182, (char)25, (char)104, (char)92, (char)151, (char)55, (char)235, (char)36, (char)10, (char)137, (char)80, (char)93, (char)106, (char)80, (char)153, (char)238, (char)47, (char)28, (char)35, (char)78, (char)169, (char)213, (char)222, (char)148, (char)191, (char)128, (char)101, (char)136, (char)241, (char)92, (char)177, (char)54, (char)46, (char)67, (char)55, (char)56, (char)211, (char)52, (char)188, (char)65, (char)118, (char)202, (char)110, (char)42, (char)60, (char)190, (char)228, (char)177, (char)8, (char)194, (char)177, (char)88, (char)215, (char)25, (char)114, (char)53, (char)168, (char)99, (char)96, (char)245, (char)84}, 0) ;
        p123.target_system_SET((char)169) ;
        p123.len_SET((char)150) ;
        p123.target_component_SET((char)149) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.dgps_age_GET() == 486159940L);
            assert(pack.epv_GET() == (char)31300);
            assert(pack.cog_GET() == (char)13495);
            assert(pack.lon_GET() == 1727181954);
            assert(pack.vel_GET() == (char)26936);
            assert(pack.alt_GET() == -1374435169);
            assert(pack.lat_GET() == -366391121);
            assert(pack.eph_GET() == (char)28654);
            assert(pack.dgps_numch_GET() == (char)144);
            assert(pack.time_usec_GET() == 7482703746294611782L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
            assert(pack.satellites_visible_GET() == (char)104);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT) ;
        p124.vel_SET((char)26936) ;
        p124.dgps_age_SET(486159940L) ;
        p124.satellites_visible_SET((char)104) ;
        p124.time_usec_SET(7482703746294611782L) ;
        p124.epv_SET((char)31300) ;
        p124.cog_SET((char)13495) ;
        p124.alt_SET(-1374435169) ;
        p124.lon_SET(1727181954) ;
        p124.lat_SET(-366391121) ;
        p124.dgps_numch_SET((char)144) ;
        p124.eph_SET((char)28654) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vservo_GET() == (char)54937);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED));
            assert(pack.Vcc_GET() == (char)44181);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)44181) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED)) ;
        p125.Vservo_SET((char)54937) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.timeout_GET() == (char)51870);
            assert(pack.baudrate_GET() == 1795878157L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)141, (char)242, (char)178, (char)53, (char)58, (char)197, (char)56, (char)107, (char)14, (char)213, (char)32, (char)139, (char)230, (char)177, (char)137, (char)112, (char)247, (char)49, (char)48, (char)223, (char)248, (char)95, (char)160, (char)244, (char)140, (char)192, (char)187, (char)179, (char)105, (char)30, (char)18, (char)139, (char)126, (char)67, (char)16, (char)95, (char)14, (char)111, (char)52, (char)197, (char)166, (char)180, (char)182, (char)170, (char)221, (char)107, (char)246, (char)184, (char)73, (char)60, (char)55, (char)156, (char)106, (char)201, (char)194, (char)93, (char)247, (char)169, (char)241, (char)108, (char)20, (char)75, (char)114, (char)82, (char)44, (char)66, (char)147, (char)171, (char)92, (char)241}));
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
            assert(pack.count_GET() == (char)133);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.baudrate_SET(1795878157L) ;
        p126.timeout_SET((char)51870) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING)) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2) ;
        p126.data__SET(new char[] {(char)141, (char)242, (char)178, (char)53, (char)58, (char)197, (char)56, (char)107, (char)14, (char)213, (char)32, (char)139, (char)230, (char)177, (char)137, (char)112, (char)247, (char)49, (char)48, (char)223, (char)248, (char)95, (char)160, (char)244, (char)140, (char)192, (char)187, (char)179, (char)105, (char)30, (char)18, (char)139, (char)126, (char)67, (char)16, (char)95, (char)14, (char)111, (char)52, (char)197, (char)166, (char)180, (char)182, (char)170, (char)221, (char)107, (char)246, (char)184, (char)73, (char)60, (char)55, (char)156, (char)106, (char)201, (char)194, (char)93, (char)247, (char)169, (char)241, (char)108, (char)20, (char)75, (char)114, (char)82, (char)44, (char)66, (char)147, (char)171, (char)92, (char)241}, 0) ;
        p126.count_SET((char)133) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.nsats_GET() == (char)131);
            assert(pack.rtk_receiver_id_GET() == (char)214);
            assert(pack.accuracy_GET() == 3179208464L);
            assert(pack.baseline_a_mm_GET() == -1024018372);
            assert(pack.baseline_c_mm_GET() == -2129897067);
            assert(pack.tow_GET() == 566753303L);
            assert(pack.time_last_baseline_ms_GET() == 1656977974L);
            assert(pack.rtk_rate_GET() == (char)187);
            assert(pack.iar_num_hypotheses_GET() == 2099350252);
            assert(pack.baseline_b_mm_GET() == -881616193);
            assert(pack.baseline_coords_type_GET() == (char)204);
            assert(pack.wn_GET() == (char)10443);
            assert(pack.rtk_health_GET() == (char)181);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.wn_SET((char)10443) ;
        p127.rtk_health_SET((char)181) ;
        p127.baseline_b_mm_SET(-881616193) ;
        p127.baseline_a_mm_SET(-1024018372) ;
        p127.rtk_receiver_id_SET((char)214) ;
        p127.accuracy_SET(3179208464L) ;
        p127.rtk_rate_SET((char)187) ;
        p127.baseline_coords_type_SET((char)204) ;
        p127.time_last_baseline_ms_SET(1656977974L) ;
        p127.iar_num_hypotheses_SET(2099350252) ;
        p127.baseline_c_mm_SET(-2129897067) ;
        p127.tow_SET(566753303L) ;
        p127.nsats_SET((char)131) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_coords_type_GET() == (char)50);
            assert(pack.rtk_receiver_id_GET() == (char)207);
            assert(pack.rtk_health_GET() == (char)94);
            assert(pack.wn_GET() == (char)60107);
            assert(pack.baseline_c_mm_GET() == 1267642832);
            assert(pack.time_last_baseline_ms_GET() == 3661125899L);
            assert(pack.nsats_GET() == (char)29);
            assert(pack.iar_num_hypotheses_GET() == 1497727472);
            assert(pack.tow_GET() == 1591766153L);
            assert(pack.rtk_rate_GET() == (char)62);
            assert(pack.accuracy_GET() == 3920716497L);
            assert(pack.baseline_b_mm_GET() == -1018219525);
            assert(pack.baseline_a_mm_GET() == 300845539);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_c_mm_SET(1267642832) ;
        p128.wn_SET((char)60107) ;
        p128.rtk_rate_SET((char)62) ;
        p128.time_last_baseline_ms_SET(3661125899L) ;
        p128.rtk_health_SET((char)94) ;
        p128.baseline_a_mm_SET(300845539) ;
        p128.nsats_SET((char)29) ;
        p128.rtk_receiver_id_SET((char)207) ;
        p128.baseline_b_mm_SET(-1018219525) ;
        p128.baseline_coords_type_SET((char)50) ;
        p128.accuracy_SET(3920716497L) ;
        p128.iar_num_hypotheses_SET(1497727472) ;
        p128.tow_SET(1591766153L) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == (short) -6026);
            assert(pack.time_boot_ms_GET() == 1309507436L);
            assert(pack.zgyro_GET() == (short) -18469);
            assert(pack.ygyro_GET() == (short)17863);
            assert(pack.xmag_GET() == (short) -13951);
            assert(pack.ymag_GET() == (short) -26680);
            assert(pack.xacc_GET() == (short) -13295);
            assert(pack.xgyro_GET() == (short)25475);
            assert(pack.zacc_GET() == (short) -3654);
            assert(pack.yacc_GET() == (short)25761);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.xgyro_SET((short)25475) ;
        p129.zmag_SET((short) -6026) ;
        p129.xacc_SET((short) -13295) ;
        p129.time_boot_ms_SET(1309507436L) ;
        p129.zacc_SET((short) -3654) ;
        p129.zgyro_SET((short) -18469) ;
        p129.ygyro_SET((short)17863) ;
        p129.yacc_SET((short)25761) ;
        p129.xmag_SET((short) -13951) ;
        p129.ymag_SET((short) -26680) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.jpg_quality_GET() == (char)175);
            assert(pack.size_GET() == 3631806020L);
            assert(pack.height_GET() == (char)60985);
            assert(pack.packets_GET() == (char)26228);
            assert(pack.payload_GET() == (char)102);
            assert(pack.width_GET() == (char)13350);
            assert(pack.type_GET() == (char)186);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.payload_SET((char)102) ;
        p130.height_SET((char)60985) ;
        p130.jpg_quality_SET((char)175) ;
        p130.type_SET((char)186) ;
        p130.width_SET((char)13350) ;
        p130.size_SET(3631806020L) ;
        p130.packets_SET((char)26228) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)34722);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)10, (char)82, (char)199, (char)113, (char)41, (char)69, (char)173, (char)20, (char)234, (char)76, (char)154, (char)77, (char)114, (char)8, (char)179, (char)136, (char)191, (char)220, (char)10, (char)100, (char)78, (char)133, (char)213, (char)16, (char)99, (char)174, (char)253, (char)55, (char)252, (char)30, (char)204, (char)49, (char)135, (char)105, (char)93, (char)245, (char)110, (char)238, (char)157, (char)85, (char)78, (char)41, (char)20, (char)47, (char)45, (char)186, (char)30, (char)222, (char)217, (char)204, (char)190, (char)139, (char)164, (char)197, (char)183, (char)153, (char)146, (char)208, (char)182, (char)26, (char)55, (char)166, (char)207, (char)33, (char)44, (char)75, (char)253, (char)5, (char)236, (char)83, (char)250, (char)145, (char)208, (char)50, (char)129, (char)50, (char)36, (char)72, (char)143, (char)140, (char)159, (char)65, (char)155, (char)193, (char)228, (char)51, (char)97, (char)17, (char)87, (char)116, (char)88, (char)180, (char)230, (char)13, (char)219, (char)87, (char)154, (char)207, (char)193, (char)83, (char)58, (char)25, (char)223, (char)134, (char)40, (char)62, (char)220, (char)165, (char)62, (char)184, (char)107, (char)213, (char)53, (char)57, (char)224, (char)28, (char)50, (char)186, (char)2, (char)242, (char)65, (char)40, (char)112, (char)86, (char)4, (char)129, (char)21, (char)140, (char)76, (char)181, (char)130, (char)78, (char)40, (char)97, (char)55, (char)214, (char)224, (char)14, (char)215, (char)66, (char)107, (char)146, (char)93, (char)231, (char)1, (char)136, (char)133, (char)68, (char)199, (char)122, (char)30, (char)192, (char)27, (char)227, (char)158, (char)70, (char)240, (char)180, (char)37, (char)58, (char)30, (char)22, (char)44, (char)62, (char)238, (char)229, (char)214, (char)93, (char)108, (char)13, (char)138, (char)114, (char)198, (char)20, (char)18, (char)173, (char)245, (char)158, (char)114, (char)158, (char)117, (char)54, (char)242, (char)154, (char)27, (char)89, (char)51, (char)242, (char)242, (char)161, (char)128, (char)60, (char)221, (char)84, (char)251, (char)57, (char)0, (char)67, (char)203, (char)247, (char)172, (char)101, (char)69, (char)118, (char)241, (char)248, (char)194, (char)6, (char)186, (char)165, (char)182, (char)71, (char)39, (char)135, (char)175, (char)77, (char)184, (char)45, (char)101, (char)43, (char)104, (char)96, (char)187, (char)230, (char)48, (char)129, (char)208, (char)151, (char)156, (char)247, (char)95, (char)172, (char)40, (char)88, (char)209, (char)21, (char)73, (char)212, (char)105, (char)221, (char)82, (char)130, (char)131, (char)93, (char)254, (char)7, (char)44, (char)83, (char)31, (char)45, (char)37, (char)5, (char)39}));
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)34722) ;
        p131.data__SET(new char[] {(char)10, (char)82, (char)199, (char)113, (char)41, (char)69, (char)173, (char)20, (char)234, (char)76, (char)154, (char)77, (char)114, (char)8, (char)179, (char)136, (char)191, (char)220, (char)10, (char)100, (char)78, (char)133, (char)213, (char)16, (char)99, (char)174, (char)253, (char)55, (char)252, (char)30, (char)204, (char)49, (char)135, (char)105, (char)93, (char)245, (char)110, (char)238, (char)157, (char)85, (char)78, (char)41, (char)20, (char)47, (char)45, (char)186, (char)30, (char)222, (char)217, (char)204, (char)190, (char)139, (char)164, (char)197, (char)183, (char)153, (char)146, (char)208, (char)182, (char)26, (char)55, (char)166, (char)207, (char)33, (char)44, (char)75, (char)253, (char)5, (char)236, (char)83, (char)250, (char)145, (char)208, (char)50, (char)129, (char)50, (char)36, (char)72, (char)143, (char)140, (char)159, (char)65, (char)155, (char)193, (char)228, (char)51, (char)97, (char)17, (char)87, (char)116, (char)88, (char)180, (char)230, (char)13, (char)219, (char)87, (char)154, (char)207, (char)193, (char)83, (char)58, (char)25, (char)223, (char)134, (char)40, (char)62, (char)220, (char)165, (char)62, (char)184, (char)107, (char)213, (char)53, (char)57, (char)224, (char)28, (char)50, (char)186, (char)2, (char)242, (char)65, (char)40, (char)112, (char)86, (char)4, (char)129, (char)21, (char)140, (char)76, (char)181, (char)130, (char)78, (char)40, (char)97, (char)55, (char)214, (char)224, (char)14, (char)215, (char)66, (char)107, (char)146, (char)93, (char)231, (char)1, (char)136, (char)133, (char)68, (char)199, (char)122, (char)30, (char)192, (char)27, (char)227, (char)158, (char)70, (char)240, (char)180, (char)37, (char)58, (char)30, (char)22, (char)44, (char)62, (char)238, (char)229, (char)214, (char)93, (char)108, (char)13, (char)138, (char)114, (char)198, (char)20, (char)18, (char)173, (char)245, (char)158, (char)114, (char)158, (char)117, (char)54, (char)242, (char)154, (char)27, (char)89, (char)51, (char)242, (char)242, (char)161, (char)128, (char)60, (char)221, (char)84, (char)251, (char)57, (char)0, (char)67, (char)203, (char)247, (char)172, (char)101, (char)69, (char)118, (char)241, (char)248, (char)194, (char)6, (char)186, (char)165, (char)182, (char)71, (char)39, (char)135, (char)175, (char)77, (char)184, (char)45, (char)101, (char)43, (char)104, (char)96, (char)187, (char)230, (char)48, (char)129, (char)208, (char)151, (char)156, (char)247, (char)95, (char)172, (char)40, (char)88, (char)209, (char)21, (char)73, (char)212, (char)105, (char)221, (char)82, (char)130, (char)131, (char)93, (char)254, (char)7, (char)44, (char)83, (char)31, (char)45, (char)37, (char)5, (char)39}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            assert(pack.covariance_GET() == (char)24);
            assert(pack.time_boot_ms_GET() == 2228304569L);
            assert(pack.current_distance_GET() == (char)36846);
            assert(pack.max_distance_GET() == (char)21725);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_PITCH_90);
            assert(pack.id_GET() == (char)225);
            assert(pack.min_distance_GET() == (char)40246);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.current_distance_SET((char)36846) ;
        p132.min_distance_SET((char)40246) ;
        p132.covariance_SET((char)24) ;
        p132.id_SET((char)225) ;
        p132.max_distance_SET((char)21725) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_PITCH_90) ;
        p132.time_boot_ms_SET(2228304569L) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1568402644);
            assert(pack.mask_GET() == 3921166206337612190L);
            assert(pack.grid_spacing_GET() == (char)47902);
            assert(pack.lon_GET() == 1726550633);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lon_SET(1726550633) ;
        p133.lat_SET(1568402644) ;
        p133.grid_spacing_SET((char)47902) ;
        p133.mask_SET(3921166206337612190L) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 339858240);
            assert(pack.grid_spacing_GET() == (char)56993);
            assert(pack.lon_GET() == 223431640);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -2662, (short) -17967, (short)13702, (short)25764, (short) -7216, (short)23381, (short)17637, (short) -25363, (short) -2779, (short)2491, (short)1488, (short)27475, (short)12826, (short) -11533, (short)14738, (short) -863}));
            assert(pack.gridbit_GET() == (char)195);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.data__SET(new short[] {(short) -2662, (short) -17967, (short)13702, (short)25764, (short) -7216, (short)23381, (short)17637, (short) -25363, (short) -2779, (short)2491, (short)1488, (short)27475, (short)12826, (short) -11533, (short)14738, (short) -863}, 0) ;
        p134.lat_SET(339858240) ;
        p134.lon_SET(223431640) ;
        p134.grid_spacing_SET((char)56993) ;
        p134.gridbit_SET((char)195) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 432258470);
            assert(pack.lat_GET() == -1734271672);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-1734271672) ;
        p135.lon_SET(432258470) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.terrain_height_GET() == 1.7321654E38F);
            assert(pack.lon_GET() == -1809261293);
            assert(pack.pending_GET() == (char)16426);
            assert(pack.lat_GET() == -45825025);
            assert(pack.current_height_GET() == 2.1120014E38F);
            assert(pack.loaded_GET() == (char)39343);
            assert(pack.spacing_GET() == (char)9047);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lon_SET(-1809261293) ;
        p136.terrain_height_SET(1.7321654E38F) ;
        p136.pending_SET((char)16426) ;
        p136.spacing_SET((char)9047) ;
        p136.current_height_SET(2.1120014E38F) ;
        p136.loaded_SET((char)39343) ;
        p136.lat_SET(-45825025) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -12865);
            assert(pack.press_abs_GET() == -2.072601E38F);
            assert(pack.press_diff_GET() == -2.9709207E38F);
            assert(pack.time_boot_ms_GET() == 3563450619L);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_diff_SET(-2.9709207E38F) ;
        p137.time_boot_ms_SET(3563450619L) ;
        p137.press_abs_SET(-2.072601E38F) ;
        p137.temperature_SET((short) -12865) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -3.1331194E37F);
            assert(pack.x_GET() == -2.3292238E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.410452E38F, -5.820874E37F, -2.3293656E38F, 1.5571696E37F}));
            assert(pack.y_GET() == 1.7387556E38F);
            assert(pack.time_usec_GET() == 8809854554270836915L);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.x_SET(-2.3292238E38F) ;
        p138.q_SET(new float[] {2.410452E38F, -5.820874E37F, -2.3293656E38F, 1.5571696E37F}, 0) ;
        p138.z_SET(-3.1331194E37F) ;
        p138.y_SET(1.7387556E38F) ;
        p138.time_usec_SET(8809854554270836915L) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)163);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.060712E38F, -1.5023098E38F, 3.942848E35F, 2.2029214E38F, 1.0782133E38F, 1.1914231E38F, -2.3056985E37F, -2.5399167E38F}));
            assert(pack.target_component_GET() == (char)152);
            assert(pack.time_usec_GET() == 5895959291221951195L);
            assert(pack.target_system_GET() == (char)245);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.target_system_SET((char)245) ;
        p139.target_component_SET((char)152) ;
        p139.group_mlx_SET((char)163) ;
        p139.time_usec_SET(5895959291221951195L) ;
        p139.controls_SET(new float[] {2.060712E38F, -1.5023098E38F, 3.942848E35F, 2.2029214E38F, 1.0782133E38F, 1.1914231E38F, -2.3056985E37F, -2.5399167E38F}, 0) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {5.9282644E36F, 2.7091684E37F, 1.6903255E38F, 2.2035365E37F, 1.8919367E38F, -2.7618184E38F, 4.04918E37F, -2.390165E37F}));
            assert(pack.group_mlx_GET() == (char)122);
            assert(pack.time_usec_GET() == 6417421718142589211L);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.controls_SET(new float[] {5.9282644E36F, 2.7091684E37F, 1.6903255E38F, 2.2035365E37F, 1.8919367E38F, -2.7618184E38F, 4.04918E37F, -2.390165E37F}, 0) ;
        p140.group_mlx_SET((char)122) ;
        p140.time_usec_SET(6417421718142589211L) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.bottom_clearance_GET() == -3.066076E38F);
            assert(pack.altitude_amsl_GET() == 2.6988695E38F);
            assert(pack.altitude_terrain_GET() == -3.285833E38F);
            assert(pack.altitude_monotonic_GET() == -3.3952378E38F);
            assert(pack.altitude_relative_GET() == -2.5749324E38F);
            assert(pack.altitude_local_GET() == -1.4273926E38F);
            assert(pack.time_usec_GET() == 4302922840738366995L);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_terrain_SET(-3.285833E38F) ;
        p141.altitude_local_SET(-1.4273926E38F) ;
        p141.time_usec_SET(4302922840738366995L) ;
        p141.bottom_clearance_SET(-3.066076E38F) ;
        p141.altitude_relative_SET(-2.5749324E38F) ;
        p141.altitude_monotonic_SET(-3.3952378E38F) ;
        p141.altitude_amsl_SET(2.6988695E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.uri_type_GET() == (char)169);
            assert(pack.transfer_type_GET() == (char)237);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)96, (char)186, (char)67, (char)132, (char)182, (char)141, (char)240, (char)12, (char)88, (char)38, (char)211, (char)211, (char)233, (char)245, (char)112, (char)63, (char)157, (char)185, (char)237, (char)116, (char)7, (char)107, (char)72, (char)70, (char)101, (char)2, (char)117, (char)126, (char)0, (char)237, (char)136, (char)33, (char)106, (char)131, (char)177, (char)154, (char)102, (char)81, (char)194, (char)97, (char)166, (char)157, (char)143, (char)255, (char)188, (char)205, (char)229, (char)208, (char)115, (char)74, (char)102, (char)85, (char)68, (char)221, (char)25, (char)78, (char)92, (char)118, (char)81, (char)104, (char)54, (char)70, (char)64, (char)178, (char)96, (char)18, (char)240, (char)103, (char)144, (char)188, (char)42, (char)113, (char)220, (char)84, (char)88, (char)48, (char)14, (char)150, (char)62, (char)176, (char)66, (char)246, (char)123, (char)20, (char)216, (char)187, (char)16, (char)19, (char)79, (char)154, (char)255, (char)46, (char)147, (char)6, (char)217, (char)5, (char)184, (char)37, (char)198, (char)12, (char)33, (char)221, (char)174, (char)157, (char)251, (char)209, (char)253, (char)209, (char)214, (char)148, (char)148, (char)234, (char)207, (char)9, (char)17, (char)226, (char)237, (char)166, (char)18, (char)222}));
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)125, (char)71, (char)251, (char)80, (char)230, (char)53, (char)63, (char)130, (char)240, (char)173, (char)120, (char)155, (char)212, (char)135, (char)110, (char)134, (char)197, (char)206, (char)115, (char)16, (char)96, (char)167, (char)225, (char)8, (char)5, (char)190, (char)94, (char)65, (char)153, (char)97, (char)74, (char)185, (char)232, (char)171, (char)200, (char)55, (char)175, (char)155, (char)132, (char)57, (char)69, (char)9, (char)49, (char)101, (char)206, (char)203, (char)215, (char)220, (char)145, (char)249, (char)202, (char)80, (char)242, (char)191, (char)224, (char)136, (char)135, (char)49, (char)79, (char)20, (char)33, (char)165, (char)158, (char)143, (char)19, (char)232, (char)213, (char)189, (char)220, (char)173, (char)181, (char)233, (char)23, (char)113, (char)177, (char)21, (char)244, (char)150, (char)16, (char)66, (char)179, (char)115, (char)233, (char)212, (char)116, (char)138, (char)77, (char)56, (char)56, (char)248, (char)228, (char)72, (char)137, (char)98, (char)206, (char)163, (char)19, (char)67, (char)140, (char)120, (char)57, (char)164, (char)191, (char)140, (char)119, (char)108, (char)155, (char)182, (char)214, (char)151, (char)84, (char)18, (char)175, (char)45, (char)67, (char)165, (char)14, (char)229, (char)47, (char)88}));
            assert(pack.request_id_GET() == (char)210);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_SET(new char[] {(char)125, (char)71, (char)251, (char)80, (char)230, (char)53, (char)63, (char)130, (char)240, (char)173, (char)120, (char)155, (char)212, (char)135, (char)110, (char)134, (char)197, (char)206, (char)115, (char)16, (char)96, (char)167, (char)225, (char)8, (char)5, (char)190, (char)94, (char)65, (char)153, (char)97, (char)74, (char)185, (char)232, (char)171, (char)200, (char)55, (char)175, (char)155, (char)132, (char)57, (char)69, (char)9, (char)49, (char)101, (char)206, (char)203, (char)215, (char)220, (char)145, (char)249, (char)202, (char)80, (char)242, (char)191, (char)224, (char)136, (char)135, (char)49, (char)79, (char)20, (char)33, (char)165, (char)158, (char)143, (char)19, (char)232, (char)213, (char)189, (char)220, (char)173, (char)181, (char)233, (char)23, (char)113, (char)177, (char)21, (char)244, (char)150, (char)16, (char)66, (char)179, (char)115, (char)233, (char)212, (char)116, (char)138, (char)77, (char)56, (char)56, (char)248, (char)228, (char)72, (char)137, (char)98, (char)206, (char)163, (char)19, (char)67, (char)140, (char)120, (char)57, (char)164, (char)191, (char)140, (char)119, (char)108, (char)155, (char)182, (char)214, (char)151, (char)84, (char)18, (char)175, (char)45, (char)67, (char)165, (char)14, (char)229, (char)47, (char)88}, 0) ;
        p142.request_id_SET((char)210) ;
        p142.uri_type_SET((char)169) ;
        p142.storage_SET(new char[] {(char)96, (char)186, (char)67, (char)132, (char)182, (char)141, (char)240, (char)12, (char)88, (char)38, (char)211, (char)211, (char)233, (char)245, (char)112, (char)63, (char)157, (char)185, (char)237, (char)116, (char)7, (char)107, (char)72, (char)70, (char)101, (char)2, (char)117, (char)126, (char)0, (char)237, (char)136, (char)33, (char)106, (char)131, (char)177, (char)154, (char)102, (char)81, (char)194, (char)97, (char)166, (char)157, (char)143, (char)255, (char)188, (char)205, (char)229, (char)208, (char)115, (char)74, (char)102, (char)85, (char)68, (char)221, (char)25, (char)78, (char)92, (char)118, (char)81, (char)104, (char)54, (char)70, (char)64, (char)178, (char)96, (char)18, (char)240, (char)103, (char)144, (char)188, (char)42, (char)113, (char)220, (char)84, (char)88, (char)48, (char)14, (char)150, (char)62, (char)176, (char)66, (char)246, (char)123, (char)20, (char)216, (char)187, (char)16, (char)19, (char)79, (char)154, (char)255, (char)46, (char)147, (char)6, (char)217, (char)5, (char)184, (char)37, (char)198, (char)12, (char)33, (char)221, (char)174, (char)157, (char)251, (char)209, (char)253, (char)209, (char)214, (char)148, (char)148, (char)234, (char)207, (char)9, (char)17, (char)226, (char)237, (char)166, (char)18, (char)222}, 0) ;
        p142.transfer_type_SET((char)237) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3691497598L);
            assert(pack.press_diff_GET() == 2.176232E38F);
            assert(pack.press_abs_GET() == -1.8080171E37F);
            assert(pack.temperature_GET() == (short)26756);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.temperature_SET((short)26756) ;
        p143.press_abs_SET(-1.8080171E37F) ;
        p143.time_boot_ms_SET(3691497598L) ;
        p143.press_diff_SET(2.176232E38F) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 669905087);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-1.793753E38F, 1.7017003E38F, -5.9239016E37F}));
            assert(pack.custom_state_GET() == 4250514189533076580L);
            assert(pack.alt_GET() == -2.9785159E38F);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {-4.1788083E36F, 1.8227195E38F, 1.7026906E38F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-2.4607092E38F, 6.32512E37F, -1.785168E38F, -2.041986E38F}));
            assert(pack.lat_GET() == -677360125);
            assert(pack.est_capabilities_GET() == (char)85);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {6.9456736E37F, -7.245841E36F, 3.004598E38F}));
            assert(pack.timestamp_GET() == 5916823829855388184L);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {2.842349E38F, -2.7464855E38F, -3.0710811E38F}));
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.lat_SET(-677360125) ;
        p144.vel_SET(new float[] {-4.1788083E36F, 1.8227195E38F, 1.7026906E38F}, 0) ;
        p144.est_capabilities_SET((char)85) ;
        p144.lon_SET(669905087) ;
        p144.attitude_q_SET(new float[] {-2.4607092E38F, 6.32512E37F, -1.785168E38F, -2.041986E38F}, 0) ;
        p144.custom_state_SET(4250514189533076580L) ;
        p144.rates_SET(new float[] {6.9456736E37F, -7.245841E36F, 3.004598E38F}, 0) ;
        p144.timestamp_SET(5916823829855388184L) ;
        p144.alt_SET(-2.9785159E38F) ;
        p144.acc_SET(new float[] {-1.793753E38F, 1.7017003E38F, -5.9239016E37F}, 0) ;
        p144.position_cov_SET(new float[] {2.842349E38F, -2.7464855E38F, -3.0710811E38F}, 0) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.z_acc_GET() == 1.8829931E38F);
            assert(pack.pitch_rate_GET() == 6.3926175E37F);
            assert(pack.y_vel_GET() == 6.3770847E37F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {2.4869321E38F, 2.929595E38F, 5.1148703E35F}));
            assert(pack.z_vel_GET() == 2.9015322E38F);
            assert(pack.airspeed_GET() == -1.5465285E38F);
            assert(pack.y_pos_GET() == 1.9371182E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.3391339E38F, 1.7297119E38F, -1.4882252E38F, -1.1972902E38F}));
            assert(pack.time_usec_GET() == 1486281382377293178L);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {1.9983997E37F, 1.8958538E38F, 1.7196058E38F}));
            assert(pack.x_acc_GET() == 3.257292E38F);
            assert(pack.x_pos_GET() == -1.8960203E38F);
            assert(pack.roll_rate_GET() == -9.869816E37F);
            assert(pack.z_pos_GET() == -1.081167E38F);
            assert(pack.yaw_rate_GET() == -2.2480846E38F);
            assert(pack.y_acc_GET() == -2.2602194E38F);
            assert(pack.x_vel_GET() == -8.1415686E37F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.yaw_rate_SET(-2.2480846E38F) ;
        p146.vel_variance_SET(new float[] {1.9983997E37F, 1.8958538E38F, 1.7196058E38F}, 0) ;
        p146.x_vel_SET(-8.1415686E37F) ;
        p146.pos_variance_SET(new float[] {2.4869321E38F, 2.929595E38F, 5.1148703E35F}, 0) ;
        p146.roll_rate_SET(-9.869816E37F) ;
        p146.time_usec_SET(1486281382377293178L) ;
        p146.x_acc_SET(3.257292E38F) ;
        p146.z_acc_SET(1.8829931E38F) ;
        p146.pitch_rate_SET(6.3926175E37F) ;
        p146.airspeed_SET(-1.5465285E38F) ;
        p146.x_pos_SET(-1.8960203E38F) ;
        p146.y_vel_SET(6.3770847E37F) ;
        p146.z_vel_SET(2.9015322E38F) ;
        p146.y_pos_SET(1.9371182E38F) ;
        p146.q_SET(new float[] {3.3391339E38F, 1.7297119E38F, -1.4882252E38F, -1.1972902E38F}, 0) ;
        p146.z_pos_SET(-1.081167E38F) ;
        p146.y_acc_SET(-2.2602194E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.battery_remaining_GET() == (byte)71);
            assert(pack.id_GET() == (char)40);
            assert(pack.energy_consumed_GET() == 731835161);
            assert(pack.temperature_GET() == (short)5609);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
            assert(pack.current_consumed_GET() == -1959035362);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)17330, (char)28460, (char)7230, (char)24995, (char)39030, (char)46765, (char)34044, (char)45596, (char)58793, (char)40953}));
            assert(pack.current_battery_GET() == (short) -16836);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.energy_consumed_SET(731835161) ;
        p147.battery_remaining_SET((byte)71) ;
        p147.voltages_SET(new char[] {(char)17330, (char)28460, (char)7230, (char)24995, (char)39030, (char)46765, (char)34044, (char)45596, (char)58793, (char)40953}, 0) ;
        p147.current_consumed_SET(-1959035362) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD) ;
        p147.current_battery_SET((short) -16836) ;
        p147.id_SET((char)40) ;
        p147.temperature_SET((short)5609) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.middleware_sw_version_GET() == 607350339L);
            assert(pack.os_sw_version_GET() == 2344921855L);
            assert(pack.product_id_GET() == (char)35979);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)196, (char)52, (char)64, (char)223, (char)112, (char)111, (char)129, (char)100, (char)177, (char)13, (char)168, (char)152, (char)73, (char)111, (char)173, (char)171, (char)189, (char)232}));
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)108, (char)2, (char)212, (char)132, (char)153, (char)55, (char)191, (char)184}));
            assert(pack.uid_GET() == 5747363069011928084L);
            assert(pack.flight_sw_version_GET() == 3173769674L);
            assert(pack.board_version_GET() == 1382415064L);
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION));
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)142, (char)227, (char)38, (char)111, (char)150, (char)205, (char)199, (char)168}));
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)152, (char)22, (char)199, (char)218, (char)233, (char)196, (char)222, (char)160}));
            assert(pack.vendor_id_GET() == (char)53944);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.vendor_id_SET((char)53944) ;
        p148.os_sw_version_SET(2344921855L) ;
        p148.flight_custom_version_SET(new char[] {(char)108, (char)2, (char)212, (char)132, (char)153, (char)55, (char)191, (char)184}, 0) ;
        p148.flight_sw_version_SET(3173769674L) ;
        p148.middleware_sw_version_SET(607350339L) ;
        p148.middleware_custom_version_SET(new char[] {(char)152, (char)22, (char)199, (char)218, (char)233, (char)196, (char)222, (char)160}, 0) ;
        p148.board_version_SET(1382415064L) ;
        p148.uid_SET(5747363069011928084L) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION)) ;
        p148.os_custom_version_SET(new char[] {(char)142, (char)227, (char)38, (char)111, (char)150, (char)205, (char)199, (char)168}, 0) ;
        p148.uid2_SET(new char[] {(char)196, (char)52, (char)64, (char)223, (char)112, (char)111, (char)129, (char)100, (char)177, (char)13, (char)168, (char)152, (char)73, (char)111, (char)173, (char)171, (char)189, (char)232}, 0, PH) ;
        p148.product_id_SET((char)35979) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.angle_y_GET() == 3.1062044E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.angle_x_GET() == -2.4851203E38F);
            assert(pack.distance_GET() == 1.3805568E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {-1.4997223E38F, 1.8472E38F, -3.0491234E38F, -8.294907E37F}));
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
            assert(pack.y_TRY(ph) == 3.0457794E38F);
            assert(pack.z_TRY(ph) == -3.9175821E37F);
            assert(pack.target_num_GET() == (char)109);
            assert(pack.size_x_GET() == 2.2017635E38F);
            assert(pack.size_y_GET() == 2.749467E38F);
            assert(pack.position_valid_TRY(ph) == (char)114);
            assert(pack.time_usec_GET() == 4457190557283009449L);
            assert(pack.x_TRY(ph) == -3.0270701E38F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.position_valid_SET((char)114, PH) ;
        p149.q_SET(new float[] {-1.4997223E38F, 1.8472E38F, -3.0491234E38F, -8.294907E37F}, 0, PH) ;
        p149.size_y_SET(2.749467E38F) ;
        p149.y_SET(3.0457794E38F, PH) ;
        p149.distance_SET(1.3805568E38F) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL) ;
        p149.x_SET(-3.0270701E38F, PH) ;
        p149.angle_y_SET(3.1062044E38F) ;
        p149.size_x_SET(2.2017635E38F) ;
        p149.target_num_SET((char)109) ;
        p149.z_SET(-3.9175821E37F, PH) ;
        p149.angle_x_SET(-2.4851203E38F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p149.time_usec_SET(4457190557283009449L) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pos_horiz_accuracy_GET() == 2.0230385E38F);
            assert(pack.tas_ratio_GET() == -3.027689E38F);
            assert(pack.pos_vert_accuracy_GET() == -1.0829006E38F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE));
            assert(pack.vel_ratio_GET() == -3.1846655E38F);
            assert(pack.time_usec_GET() == 7710590428810980630L);
            assert(pack.hagl_ratio_GET() == 2.649073E38F);
            assert(pack.pos_vert_ratio_GET() == -2.5150068E38F);
            assert(pack.pos_horiz_ratio_GET() == 1.3318217E38F);
            assert(pack.mag_ratio_GET() == 1.5131114E37F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE)) ;
        p230.pos_vert_ratio_SET(-2.5150068E38F) ;
        p230.vel_ratio_SET(-3.1846655E38F) ;
        p230.mag_ratio_SET(1.5131114E37F) ;
        p230.tas_ratio_SET(-3.027689E38F) ;
        p230.hagl_ratio_SET(2.649073E38F) ;
        p230.pos_horiz_accuracy_SET(2.0230385E38F) ;
        p230.pos_vert_accuracy_SET(-1.0829006E38F) ;
        p230.pos_horiz_ratio_SET(1.3318217E38F) ;
        p230.time_usec_SET(7710590428810980630L) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_y_GET() == -2.8639018E38F);
            assert(pack.wind_alt_GET() == -2.3649827E38F);
            assert(pack.horiz_accuracy_GET() == -2.8600905E38F);
            assert(pack.wind_z_GET() == -3.5147267E36F);
            assert(pack.wind_x_GET() == -1.5417594E38F);
            assert(pack.var_vert_GET() == 2.221114E38F);
            assert(pack.var_horiz_GET() == -1.4809093E38F);
            assert(pack.time_usec_GET() == 6237860105690058252L);
            assert(pack.vert_accuracy_GET() == -3.857275E36F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.horiz_accuracy_SET(-2.8600905E38F) ;
        p231.time_usec_SET(6237860105690058252L) ;
        p231.var_horiz_SET(-1.4809093E38F) ;
        p231.wind_x_SET(-1.5417594E38F) ;
        p231.wind_alt_SET(-2.3649827E38F) ;
        p231.var_vert_SET(2.221114E38F) ;
        p231.wind_z_SET(-3.5147267E36F) ;
        p231.wind_y_SET(-2.8639018E38F) ;
        p231.vert_accuracy_SET(-3.857275E36F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.vdop_GET() == -2.7739757E38F);
            assert(pack.horiz_accuracy_GET() == 9.549769E37F);
            assert(pack.hdop_GET() == -2.7277364E38F);
            assert(pack.time_week_GET() == (char)42909);
            assert(pack.lat_GET() == 253658257);
            assert(pack.vn_GET() == -2.8318465E38F);
            assert(pack.vd_GET() == 1.743037E38F);
            assert(pack.time_usec_GET() == 4865612965182249602L);
            assert(pack.satellites_visible_GET() == (char)228);
            assert(pack.fix_type_GET() == (char)59);
            assert(pack.alt_GET() == -1.4077711E38F);
            assert(pack.speed_accuracy_GET() == -3.3026742E38F);
            assert(pack.gps_id_GET() == (char)82);
            assert(pack.vert_accuracy_GET() == 4.1297024E37F);
            assert(pack.lon_GET() == -1635934009);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY));
            assert(pack.ve_GET() == 1.6876857E38F);
            assert(pack.time_week_ms_GET() == 3368302253L);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.speed_accuracy_SET(-3.3026742E38F) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY)) ;
        p232.vert_accuracy_SET(4.1297024E37F) ;
        p232.ve_SET(1.6876857E38F) ;
        p232.vdop_SET(-2.7739757E38F) ;
        p232.time_week_SET((char)42909) ;
        p232.time_week_ms_SET(3368302253L) ;
        p232.horiz_accuracy_SET(9.549769E37F) ;
        p232.satellites_visible_SET((char)228) ;
        p232.gps_id_SET((char)82) ;
        p232.vd_SET(1.743037E38F) ;
        p232.lat_SET(253658257) ;
        p232.time_usec_SET(4865612965182249602L) ;
        p232.vn_SET(-2.8318465E38F) ;
        p232.hdop_SET(-2.7277364E38F) ;
        p232.fix_type_SET((char)59) ;
        p232.alt_SET(-1.4077711E38F) ;
        p232.lon_SET(-1635934009) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)51, (char)91, (char)200, (char)152, (char)208, (char)146, (char)140, (char)101, (char)195, (char)251, (char)26, (char)254, (char)48, (char)110, (char)105, (char)33, (char)4, (char)70, (char)41, (char)141, (char)97, (char)57, (char)136, (char)67, (char)44, (char)225, (char)27, (char)57, (char)35, (char)252, (char)81, (char)137, (char)93, (char)211, (char)93, (char)51, (char)197, (char)80, (char)141, (char)250, (char)3, (char)65, (char)175, (char)113, (char)25, (char)220, (char)215, (char)231, (char)138, (char)112, (char)231, (char)255, (char)157, (char)109, (char)169, (char)206, (char)126, (char)64, (char)43, (char)187, (char)69, (char)12, (char)57, (char)169, (char)74, (char)122, (char)105, (char)100, (char)49, (char)250, (char)53, (char)102, (char)112, (char)131, (char)72, (char)216, (char)44, (char)139, (char)109, (char)17, (char)182, (char)209, (char)192, (char)213, (char)61, (char)50, (char)35, (char)231, (char)31, (char)207, (char)214, (char)213, (char)38, (char)181, (char)43, (char)160, (char)154, (char)58, (char)155, (char)35, (char)13, (char)121, (char)191, (char)13, (char)177, (char)150, (char)105, (char)232, (char)191, (char)34, (char)19, (char)226, (char)229, (char)125, (char)100, (char)49, (char)137, (char)11, (char)33, (char)134, (char)9, (char)11, (char)143, (char)91, (char)113, (char)219, (char)251, (char)245, (char)50, (char)99, (char)212, (char)5, (char)84, (char)167, (char)132, (char)76, (char)171, (char)69, (char)146, (char)50, (char)98, (char)222, (char)14, (char)215, (char)43, (char)96, (char)164, (char)14, (char)242, (char)155, (char)13, (char)110, (char)151, (char)228, (char)117, (char)198, (char)243, (char)137, (char)93, (char)176, (char)210, (char)120, (char)210, (char)244, (char)73, (char)100, (char)104, (char)186, (char)5, (char)216, (char)91, (char)213, (char)25, (char)83, (char)43, (char)221, (char)178, (char)133, (char)218, (char)152}));
            assert(pack.len_GET() == (char)57);
            assert(pack.flags_GET() == (char)248);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.data__SET(new char[] {(char)51, (char)91, (char)200, (char)152, (char)208, (char)146, (char)140, (char)101, (char)195, (char)251, (char)26, (char)254, (char)48, (char)110, (char)105, (char)33, (char)4, (char)70, (char)41, (char)141, (char)97, (char)57, (char)136, (char)67, (char)44, (char)225, (char)27, (char)57, (char)35, (char)252, (char)81, (char)137, (char)93, (char)211, (char)93, (char)51, (char)197, (char)80, (char)141, (char)250, (char)3, (char)65, (char)175, (char)113, (char)25, (char)220, (char)215, (char)231, (char)138, (char)112, (char)231, (char)255, (char)157, (char)109, (char)169, (char)206, (char)126, (char)64, (char)43, (char)187, (char)69, (char)12, (char)57, (char)169, (char)74, (char)122, (char)105, (char)100, (char)49, (char)250, (char)53, (char)102, (char)112, (char)131, (char)72, (char)216, (char)44, (char)139, (char)109, (char)17, (char)182, (char)209, (char)192, (char)213, (char)61, (char)50, (char)35, (char)231, (char)31, (char)207, (char)214, (char)213, (char)38, (char)181, (char)43, (char)160, (char)154, (char)58, (char)155, (char)35, (char)13, (char)121, (char)191, (char)13, (char)177, (char)150, (char)105, (char)232, (char)191, (char)34, (char)19, (char)226, (char)229, (char)125, (char)100, (char)49, (char)137, (char)11, (char)33, (char)134, (char)9, (char)11, (char)143, (char)91, (char)113, (char)219, (char)251, (char)245, (char)50, (char)99, (char)212, (char)5, (char)84, (char)167, (char)132, (char)76, (char)171, (char)69, (char)146, (char)50, (char)98, (char)222, (char)14, (char)215, (char)43, (char)96, (char)164, (char)14, (char)242, (char)155, (char)13, (char)110, (char)151, (char)228, (char)117, (char)198, (char)243, (char)137, (char)93, (char)176, (char)210, (char)120, (char)210, (char)244, (char)73, (char)100, (char)104, (char)186, (char)5, (char)216, (char)91, (char)213, (char)25, (char)83, (char)43, (char)221, (char)178, (char)133, (char)218, (char)152}, 0) ;
        p233.len_SET((char)57) ;
        p233.flags_SET((char)248) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 1802009171);
            assert(pack.climb_rate_GET() == (byte)38);
            assert(pack.wp_distance_GET() == (char)44610);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.gps_nsat_GET() == (char)106);
            assert(pack.roll_GET() == (short) -830);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
            assert(pack.failsafe_GET() == (char)125);
            assert(pack.heading_GET() == (char)14235);
            assert(pack.airspeed_sp_GET() == (char)137);
            assert(pack.airspeed_GET() == (char)165);
            assert(pack.pitch_GET() == (short)2947);
            assert(pack.throttle_GET() == (byte)85);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
            assert(pack.heading_sp_GET() == (short) -16295);
            assert(pack.temperature_GET() == (byte)40);
            assert(pack.longitude_GET() == -1245754585);
            assert(pack.wp_num_GET() == (char)65);
            assert(pack.battery_remaining_GET() == (char)62);
            assert(pack.custom_mode_GET() == 484316309L);
            assert(pack.altitude_amsl_GET() == (short) -4461);
            assert(pack.groundspeed_GET() == (char)216);
            assert(pack.temperature_air_GET() == (byte) - 41);
            assert(pack.altitude_sp_GET() == (short) -26358);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.latitude_SET(1802009171) ;
        p234.throttle_SET((byte)85) ;
        p234.custom_mode_SET(484316309L) ;
        p234.altitude_amsl_SET((short) -4461) ;
        p234.temperature_SET((byte)40) ;
        p234.failsafe_SET((char)125) ;
        p234.heading_sp_SET((short) -16295) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS) ;
        p234.temperature_air_SET((byte) - 41) ;
        p234.climb_rate_SET((byte)38) ;
        p234.groundspeed_SET((char)216) ;
        p234.gps_nsat_SET((char)106) ;
        p234.wp_num_SET((char)65) ;
        p234.roll_SET((short) -830) ;
        p234.heading_SET((char)14235) ;
        p234.wp_distance_SET((char)44610) ;
        p234.airspeed_SET((char)165) ;
        p234.battery_remaining_SET((char)62) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) ;
        p234.altitude_sp_SET((short) -26358) ;
        p234.longitude_SET(-1245754585) ;
        p234.pitch_SET((short)2947) ;
        p234.airspeed_sp_SET((char)137) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_x_GET() == -2.1542673E38F);
            assert(pack.vibration_z_GET() == 1.8443815E38F);
            assert(pack.vibration_y_GET() == -3.168537E38F);
            assert(pack.time_usec_GET() == 2233979730062509571L);
            assert(pack.clipping_0_GET() == 309704082L);
            assert(pack.clipping_2_GET() == 2431553151L);
            assert(pack.clipping_1_GET() == 1649525414L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_2_SET(2431553151L) ;
        p241.clipping_0_SET(309704082L) ;
        p241.vibration_x_SET(-2.1542673E38F) ;
        p241.vibration_y_SET(-3.168537E38F) ;
        p241.clipping_1_SET(1649525414L) ;
        p241.vibration_z_SET(1.8443815E38F) ;
        p241.time_usec_SET(2233979730062509571L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -2.4870859E38F);
            assert(pack.z_GET() == -3.0706486E37F);
            assert(pack.altitude_GET() == -1049995078);
            assert(pack.time_usec_TRY(ph) == 3143352189732280623L);
            assert(pack.latitude_GET() == -724403568);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.9433471E38F, -3.6253829E37F, 2.7253924E38F, -1.949188E38F}));
            assert(pack.approach_y_GET() == -9.775793E37F);
            assert(pack.longitude_GET() == -938059615);
            assert(pack.y_GET() == -2.7659033E38F);
            assert(pack.approach_x_GET() == 1.0080014E38F);
            assert(pack.approach_z_GET() == 2.527375E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.z_SET(-3.0706486E37F) ;
        p242.y_SET(-2.7659033E38F) ;
        p242.approach_y_SET(-9.775793E37F) ;
        p242.latitude_SET(-724403568) ;
        p242.longitude_SET(-938059615) ;
        p242.time_usec_SET(3143352189732280623L, PH) ;
        p242.x_SET(-2.4870859E38F) ;
        p242.altitude_SET(-1049995078) ;
        p242.approach_z_SET(2.527375E38F) ;
        p242.q_SET(new float[] {1.9433471E38F, -3.6253829E37F, 2.7253924E38F, -1.949188E38F}, 0) ;
        p242.approach_x_SET(1.0080014E38F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 2.818471E38F);
            assert(pack.approach_z_GET() == 1.730677E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {5.752594E37F, -1.6645838E38F, -1.7332852E38F, -3.7136762E37F}));
            assert(pack.z_GET() == 5.790462E37F);
            assert(pack.x_GET() == 1.4124393E38F);
            assert(pack.longitude_GET() == 461798052);
            assert(pack.altitude_GET() == 1947624897);
            assert(pack.target_system_GET() == (char)81);
            assert(pack.latitude_GET() == -1578868618);
            assert(pack.approach_x_GET() == -3.2451166E38F);
            assert(pack.approach_y_GET() == 3.351607E38F);
            assert(pack.time_usec_TRY(ph) == 1231162237755441393L);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.q_SET(new float[] {5.752594E37F, -1.6645838E38F, -1.7332852E38F, -3.7136762E37F}, 0) ;
        p243.time_usec_SET(1231162237755441393L, PH) ;
        p243.target_system_SET((char)81) ;
        p243.z_SET(5.790462E37F) ;
        p243.approach_y_SET(3.351607E38F) ;
        p243.y_SET(2.818471E38F) ;
        p243.longitude_SET(461798052) ;
        p243.x_SET(1.4124393E38F) ;
        p243.approach_x_SET(-3.2451166E38F) ;
        p243.latitude_SET(-1578868618) ;
        p243.altitude_SET(1947624897) ;
        p243.approach_z_SET(1.730677E38F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)60845);
            assert(pack.interval_us_GET() == -719744382);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)60845) ;
        p244.interval_us_SET(-719744382) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSGINED3);
            assert(pack.squawk_GET() == (char)29816);
            assert(pack.altitude_GET() == -467767015);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.lat_GET() == 232677251);
            assert(pack.heading_GET() == (char)4937);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE));
            assert(pack.tslc_GET() == (char)77);
            assert(pack.ICAO_address_GET() == 271504908L);
            assert(pack.ver_velocity_GET() == (short) -19012);
            assert(pack.callsign_LEN(ph) == 2);
            assert(pack.callsign_TRY(ph).equals("vb"));
            assert(pack.hor_velocity_GET() == (char)21482);
            assert(pack.lon_GET() == 1295968659);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.lon_SET(1295968659) ;
        p246.lat_SET(232677251) ;
        p246.heading_SET((char)4937) ;
        p246.tslc_SET((char)77) ;
        p246.hor_velocity_SET((char)21482) ;
        p246.altitude_SET(-467767015) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE)) ;
        p246.ICAO_address_SET(271504908L) ;
        p246.ver_velocity_SET((short) -19012) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSGINED3) ;
        p246.squawk_SET((char)29816) ;
        p246.callsign_SET("vb", PH) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == 2282335924L);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.threat_level_GET() == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE));
            assert(pack.altitude_minimum_delta_GET() == -3.3719966E38F);
            assert(pack.horizontal_minimum_delta_GET() == -1.8256168E38F);
            assert(pack.time_to_minimum_delta_GET() == 1.2283168E38F);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.horizontal_minimum_delta_SET(-1.8256168E38F) ;
        p247.threat_level_SET((MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE)) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.altitude_minimum_delta_SET(-3.3719966E38F) ;
        p247.id_SET(2282335924L) ;
        p247.time_to_minimum_delta_SET(1.2283168E38F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)131, (char)32, (char)70, (char)33, (char)112, (char)243, (char)47, (char)185, (char)172, (char)121, (char)175, (char)81, (char)39, (char)89, (char)43, (char)1, (char)209, (char)161, (char)143, (char)202, (char)137, (char)51, (char)33, (char)128, (char)145, (char)39, (char)135, (char)189, (char)183, (char)24, (char)135, (char)44, (char)25, (char)8, (char)20, (char)119, (char)15, (char)239, (char)143, (char)111, (char)0, (char)15, (char)249, (char)173, (char)208, (char)201, (char)87, (char)218, (char)66, (char)153, (char)64, (char)103, (char)148, (char)198, (char)229, (char)106, (char)122, (char)32, (char)19, (char)22, (char)243, (char)152, (char)192, (char)225, (char)62, (char)132, (char)192, (char)39, (char)3, (char)166, (char)39, (char)173, (char)60, (char)158, (char)245, (char)87, (char)239, (char)206, (char)115, (char)31, (char)79, (char)109, (char)84, (char)114, (char)188, (char)15, (char)160, (char)125, (char)36, (char)26, (char)135, (char)148, (char)51, (char)18, (char)2, (char)11, (char)116, (char)48, (char)150, (char)129, (char)163, (char)52, (char)32, (char)57, (char)74, (char)237, (char)208, (char)193, (char)225, (char)108, (char)60, (char)135, (char)72, (char)37, (char)224, (char)142, (char)194, (char)248, (char)50, (char)148, (char)13, (char)216, (char)235, (char)158, (char)194, (char)205, (char)155, (char)43, (char)233, (char)76, (char)196, (char)8, (char)46, (char)41, (char)134, (char)208, (char)130, (char)242, (char)49, (char)99, (char)178, (char)170, (char)88, (char)233, (char)3, (char)39, (char)216, (char)61, (char)76, (char)219, (char)105, (char)20, (char)186, (char)37, (char)141, (char)104, (char)81, (char)189, (char)86, (char)242, (char)54, (char)21, (char)122, (char)7, (char)83, (char)76, (char)76, (char)50, (char)25, (char)193, (char)130, (char)15, (char)93, (char)201, (char)170, (char)203, (char)172, (char)39, (char)135, (char)195, (char)185, (char)55, (char)53, (char)207, (char)223, (char)26, (char)43, (char)131, (char)120, (char)38, (char)88, (char)97, (char)147, (char)49, (char)102, (char)224, (char)118, (char)171, (char)120, (char)236, (char)191, (char)182, (char)215, (char)228, (char)26, (char)225, (char)47, (char)105, (char)49, (char)13, (char)5, (char)85, (char)179, (char)108, (char)27, (char)49, (char)131, (char)83, (char)51, (char)77, (char)169, (char)181, (char)92, (char)251, (char)61, (char)1, (char)199, (char)6, (char)170, (char)66, (char)171, (char)53, (char)145, (char)27, (char)229, (char)146, (char)43, (char)143, (char)53, (char)6, (char)164, (char)199, (char)195, (char)239, (char)238, (char)232, (char)205, (char)241, (char)213}));
            assert(pack.target_system_GET() == (char)125);
            assert(pack.target_component_GET() == (char)66);
            assert(pack.message_type_GET() == (char)4821);
            assert(pack.target_network_GET() == (char)10);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_component_SET((char)66) ;
        p248.target_network_SET((char)10) ;
        p248.message_type_SET((char)4821) ;
        p248.payload_SET(new char[] {(char)131, (char)32, (char)70, (char)33, (char)112, (char)243, (char)47, (char)185, (char)172, (char)121, (char)175, (char)81, (char)39, (char)89, (char)43, (char)1, (char)209, (char)161, (char)143, (char)202, (char)137, (char)51, (char)33, (char)128, (char)145, (char)39, (char)135, (char)189, (char)183, (char)24, (char)135, (char)44, (char)25, (char)8, (char)20, (char)119, (char)15, (char)239, (char)143, (char)111, (char)0, (char)15, (char)249, (char)173, (char)208, (char)201, (char)87, (char)218, (char)66, (char)153, (char)64, (char)103, (char)148, (char)198, (char)229, (char)106, (char)122, (char)32, (char)19, (char)22, (char)243, (char)152, (char)192, (char)225, (char)62, (char)132, (char)192, (char)39, (char)3, (char)166, (char)39, (char)173, (char)60, (char)158, (char)245, (char)87, (char)239, (char)206, (char)115, (char)31, (char)79, (char)109, (char)84, (char)114, (char)188, (char)15, (char)160, (char)125, (char)36, (char)26, (char)135, (char)148, (char)51, (char)18, (char)2, (char)11, (char)116, (char)48, (char)150, (char)129, (char)163, (char)52, (char)32, (char)57, (char)74, (char)237, (char)208, (char)193, (char)225, (char)108, (char)60, (char)135, (char)72, (char)37, (char)224, (char)142, (char)194, (char)248, (char)50, (char)148, (char)13, (char)216, (char)235, (char)158, (char)194, (char)205, (char)155, (char)43, (char)233, (char)76, (char)196, (char)8, (char)46, (char)41, (char)134, (char)208, (char)130, (char)242, (char)49, (char)99, (char)178, (char)170, (char)88, (char)233, (char)3, (char)39, (char)216, (char)61, (char)76, (char)219, (char)105, (char)20, (char)186, (char)37, (char)141, (char)104, (char)81, (char)189, (char)86, (char)242, (char)54, (char)21, (char)122, (char)7, (char)83, (char)76, (char)76, (char)50, (char)25, (char)193, (char)130, (char)15, (char)93, (char)201, (char)170, (char)203, (char)172, (char)39, (char)135, (char)195, (char)185, (char)55, (char)53, (char)207, (char)223, (char)26, (char)43, (char)131, (char)120, (char)38, (char)88, (char)97, (char)147, (char)49, (char)102, (char)224, (char)118, (char)171, (char)120, (char)236, (char)191, (char)182, (char)215, (char)228, (char)26, (char)225, (char)47, (char)105, (char)49, (char)13, (char)5, (char)85, (char)179, (char)108, (char)27, (char)49, (char)131, (char)83, (char)51, (char)77, (char)169, (char)181, (char)92, (char)251, (char)61, (char)1, (char)199, (char)6, (char)170, (char)66, (char)171, (char)53, (char)145, (char)27, (char)229, (char)146, (char)43, (char)143, (char)53, (char)6, (char)164, (char)199, (char)195, (char)239, (char)238, (char)232, (char)205, (char)241, (char)213}, 0) ;
        p248.target_system_SET((char)125) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.ver_GET() == (char)79);
            assert(pack.type_GET() == (char)130);
            assert(pack.address_GET() == (char)34513);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 66, (byte)95, (byte)63, (byte)60, (byte)24, (byte)87, (byte)51, (byte) - 30, (byte) - 113, (byte)5, (byte) - 15, (byte)77, (byte) - 31, (byte)109, (byte)59, (byte)90, (byte)107, (byte) - 12, (byte)108, (byte) - 53, (byte) - 95, (byte) - 51, (byte)98, (byte)91, (byte) - 10, (byte) - 102, (byte)100, (byte)55, (byte)72, (byte)15, (byte)107, (byte) - 92}));
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.type_SET((char)130) ;
        p249.value_SET(new byte[] {(byte) - 66, (byte)95, (byte)63, (byte)60, (byte)24, (byte)87, (byte)51, (byte) - 30, (byte) - 113, (byte)5, (byte) - 15, (byte)77, (byte) - 31, (byte)109, (byte)59, (byte)90, (byte)107, (byte) - 12, (byte)108, (byte) - 53, (byte) - 95, (byte) - 51, (byte)98, (byte)91, (byte) - 10, (byte) - 102, (byte)100, (byte)55, (byte)72, (byte)15, (byte)107, (byte) - 92}, 0) ;
        p249.ver_SET((char)79) ;
        p249.address_SET((char)34513) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5067705049207693127L);
            assert(pack.y_GET() == 2.963916E38F);
            assert(pack.z_GET() == 2.5253571E38F);
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("xvocexdzvy"));
            assert(pack.x_GET() == -2.7821329E38F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.z_SET(2.5253571E38F) ;
        p250.x_SET(-2.7821329E38F) ;
        p250.time_usec_SET(5067705049207693127L) ;
        p250.name_SET("xvocexdzvy", PH) ;
        p250.y_SET(2.963916E38F) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 4);
            assert(pack.name_TRY(ph).equals("cnrh"));
            assert(pack.value_GET() == 2.7822986E38F);
            assert(pack.time_boot_ms_GET() == 3130705501L);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("cnrh", PH) ;
        p251.time_boot_ms_SET(3130705501L) ;
        p251.value_SET(2.7822986E38F) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 735819693);
            assert(pack.time_boot_ms_GET() == 1123745199L);
            assert(pack.name_LEN(ph) == 4);
            assert(pack.name_TRY(ph).equals("qqeX"));
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.value_SET(735819693) ;
        p252.name_SET("qqeX", PH) ;
        p252.time_boot_ms_SET(1123745199L) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 1);
            assert(pack.text_TRY(ph).equals("m"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("m", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_CRITICAL) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 1.2197082E38F);
            assert(pack.time_boot_ms_GET() == 2582694920L);
            assert(pack.ind_GET() == (char)198);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.value_SET(1.2197082E38F) ;
        p254.time_boot_ms_SET(2582694920L) ;
        p254.ind_SET((char)198) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.initial_timestamp_GET() == 460535452637906991L);
            assert(pack.target_system_GET() == (char)232);
            assert(pack.target_component_GET() == (char)90);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)126, (char)203, (char)69, (char)9, (char)133, (char)57, (char)213, (char)170, (char)51, (char)199, (char)144, (char)186, (char)132, (char)174, (char)241, (char)115, (char)231, (char)108, (char)48, (char)125, (char)209, (char)204, (char)251, (char)239, (char)45, (char)206, (char)208, (char)70, (char)132, (char)161, (char)221, (char)51}));
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)232) ;
        p256.target_component_SET((char)90) ;
        p256.secret_key_SET(new char[] {(char)126, (char)203, (char)69, (char)9, (char)133, (char)57, (char)213, (char)170, (char)51, (char)199, (char)144, (char)186, (char)132, (char)174, (char)241, (char)115, (char)231, (char)108, (char)48, (char)125, (char)209, (char)204, (char)251, (char)239, (char)45, (char)206, (char)208, (char)70, (char)132, (char)161, (char)221, (char)51}, 0) ;
        p256.initial_timestamp_SET(460535452637906991L) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.last_change_ms_GET() == 3784952386L);
            assert(pack.state_GET() == (char)118);
            assert(pack.time_boot_ms_GET() == 1281182642L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.last_change_ms_SET(3784952386L) ;
        p257.state_SET((char)118) ;
        p257.time_boot_ms_SET(1281182642L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)202);
            assert(pack.tune_LEN(ph) == 22);
            assert(pack.tune_TRY(ph).equals("ycflhbjnosgdnsglxjbral"));
            assert(pack.target_system_GET() == (char)54);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)54) ;
        p258.tune_SET("ycflhbjnosgdnsglxjbral", PH) ;
        p258.target_component_SET((char)202) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.sensor_size_h_GET() == 3.1233296E38F);
            assert(pack.firmware_version_GET() == 2396587185L);
            assert(pack.time_boot_ms_GET() == 22347270L);
            assert(pack.lens_id_GET() == (char)91);
            assert(pack.cam_definition_uri_LEN(ph) == 46);
            assert(pack.cam_definition_uri_TRY(ph).equals("yaQnPcuoordyboberwbcybvivXxboobcefwcGbybwblrcy"));
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)12, (char)236, (char)221, (char)76, (char)192, (char)177, (char)98, (char)245, (char)181, (char)228, (char)34, (char)150, (char)239, (char)187, (char)166, (char)55, (char)106, (char)118, (char)231, (char)125, (char)194, (char)29, (char)88, (char)162, (char)196, (char)134, (char)158, (char)214, (char)37, (char)20, (char)10, (char)27}));
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO));
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)3, (char)229, (char)102, (char)28, (char)187, (char)228, (char)130, (char)234, (char)146, (char)146, (char)157, (char)172, (char)206, (char)250, (char)205, (char)122, (char)254, (char)129, (char)143, (char)96, (char)225, (char)108, (char)125, (char)241, (char)214, (char)140, (char)241, (char)165, (char)238, (char)232, (char)59, (char)106}));
            assert(pack.focal_length_GET() == 9.16455E37F);
            assert(pack.resolution_h_GET() == (char)44038);
            assert(pack.cam_definition_version_GET() == (char)65329);
            assert(pack.resolution_v_GET() == (char)569);
            assert(pack.sensor_size_v_GET() == -3.3966923E38F);
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.resolution_v_SET((char)569) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO)) ;
        p259.time_boot_ms_SET(22347270L) ;
        p259.sensor_size_h_SET(3.1233296E38F) ;
        p259.vendor_name_SET(new char[] {(char)3, (char)229, (char)102, (char)28, (char)187, (char)228, (char)130, (char)234, (char)146, (char)146, (char)157, (char)172, (char)206, (char)250, (char)205, (char)122, (char)254, (char)129, (char)143, (char)96, (char)225, (char)108, (char)125, (char)241, (char)214, (char)140, (char)241, (char)165, (char)238, (char)232, (char)59, (char)106}, 0) ;
        p259.cam_definition_uri_SET("yaQnPcuoordyboberwbcybvivXxboobcefwcGbybwblrcy", PH) ;
        p259.firmware_version_SET(2396587185L) ;
        p259.model_name_SET(new char[] {(char)12, (char)236, (char)221, (char)76, (char)192, (char)177, (char)98, (char)245, (char)181, (char)228, (char)34, (char)150, (char)239, (char)187, (char)166, (char)55, (char)106, (char)118, (char)231, (char)125, (char)194, (char)29, (char)88, (char)162, (char)196, (char)134, (char)158, (char)214, (char)37, (char)20, (char)10, (char)27}, 0) ;
        p259.cam_definition_version_SET((char)65329) ;
        p259.lens_id_SET((char)91) ;
        p259.sensor_size_v_SET(-3.3966923E38F) ;
        p259.focal_length_SET(9.16455E37F) ;
        p259.resolution_h_SET((char)44038) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 419086154L);
            assert(pack.mode_id_GET() == (CAMERA_MODE.CAMERA_MODE_IMAGE |
                                          CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY));
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(419086154L) ;
        p260.mode_id_SET((CAMERA_MODE.CAMERA_MODE_IMAGE |
                          CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY)) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.storage_count_GET() == (char)62);
            assert(pack.read_speed_GET() == -2.7619796E38F);
            assert(pack.write_speed_GET() == 2.5517326E38F);
            assert(pack.time_boot_ms_GET() == 2414505147L);
            assert(pack.status_GET() == (char)39);
            assert(pack.storage_id_GET() == (char)149);
            assert(pack.total_capacity_GET() == 9.651497E37F);
            assert(pack.available_capacity_GET() == 5.084367E37F);
            assert(pack.used_capacity_GET() == 1.999735E38F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.status_SET((char)39) ;
        p261.available_capacity_SET(5.084367E37F) ;
        p261.time_boot_ms_SET(2414505147L) ;
        p261.storage_count_SET((char)62) ;
        p261.total_capacity_SET(9.651497E37F) ;
        p261.read_speed_SET(-2.7619796E38F) ;
        p261.storage_id_SET((char)149) ;
        p261.used_capacity_SET(1.999735E38F) ;
        p261.write_speed_SET(2.5517326E38F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2483778379L);
            assert(pack.recording_time_ms_GET() == 3759341485L);
            assert(pack.image_interval_GET() == 1.0304872E38F);
            assert(pack.image_status_GET() == (char)24);
            assert(pack.available_capacity_GET() == -7.6324396E37F);
            assert(pack.video_status_GET() == (char)182);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.available_capacity_SET(-7.6324396E37F) ;
        p262.recording_time_ms_SET(3759341485L) ;
        p262.image_interval_SET(1.0304872E38F) ;
        p262.time_boot_ms_SET(2483778379L) ;
        p262.image_status_SET((char)24) ;
        p262.video_status_SET((char)182) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 1810146716);
            assert(pack.lat_GET() == -703855593);
            assert(pack.file_url_LEN(ph) == 57);
            assert(pack.file_url_TRY(ph).equals("RhyhrzrixTxbkinqjxskpwaDomtjijidkNamrpjjnhiSBUdpdfQurdmwQ"));
            assert(pack.time_boot_ms_GET() == 2477991764L);
            assert(pack.time_utc_GET() == 2214291759468520252L);
            assert(pack.capture_result_GET() == (byte) - 49);
            assert(pack.image_index_GET() == 173918666);
            assert(pack.alt_GET() == -1916138552);
            assert(pack.camera_id_GET() == (char)95);
            assert(pack.relative_alt_GET() == -1741339399);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.0539856E38F, 1.3372343E38F, 1.3154955E38F, -2.9188667E38F}));
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.image_index_SET(173918666) ;
        p263.q_SET(new float[] {-2.0539856E38F, 1.3372343E38F, 1.3154955E38F, -2.9188667E38F}, 0) ;
        p263.lat_SET(-703855593) ;
        p263.time_utc_SET(2214291759468520252L) ;
        p263.lon_SET(1810146716) ;
        p263.time_boot_ms_SET(2477991764L) ;
        p263.file_url_SET("RhyhrzrixTxbkinqjxskpwaDomtjijidkNamrpjjnhiSBUdpdfQurdmwQ", PH) ;
        p263.alt_SET(-1916138552) ;
        p263.relative_alt_SET(-1741339399) ;
        p263.camera_id_SET((char)95) ;
        p263.capture_result_SET((byte) - 49) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.flight_uuid_GET() == 1260851440007104385L);
            assert(pack.time_boot_ms_GET() == 1303345282L);
            assert(pack.arming_time_utc_GET() == 6711687656614278762L);
            assert(pack.takeoff_time_utc_GET() == 1647804881471054547L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.arming_time_utc_SET(6711687656614278762L) ;
        p264.takeoff_time_utc_SET(1647804881471054547L) ;
        p264.time_boot_ms_SET(1303345282L) ;
        p264.flight_uuid_SET(1260851440007104385L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -2.8427574E37F);
            assert(pack.time_boot_ms_GET() == 1988833260L);
            assert(pack.roll_GET() == 2.4284015E38F);
            assert(pack.yaw_GET() == 2.0826684E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(1988833260L) ;
        p265.yaw_SET(2.0826684E38F) ;
        p265.roll_SET(2.4284015E38F) ;
        p265.pitch_SET(-2.8427574E37F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.first_message_offset_GET() == (char)173);
            assert(pack.target_component_GET() == (char)46);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)86, (char)71, (char)13, (char)78, (char)44, (char)60, (char)181, (char)237, (char)54, (char)93, (char)105, (char)190, (char)167, (char)233, (char)207, (char)74, (char)93, (char)170, (char)14, (char)7, (char)229, (char)11, (char)223, (char)118, (char)61, (char)212, (char)245, (char)178, (char)43, (char)144, (char)151, (char)133, (char)125, (char)169, (char)104, (char)66, (char)158, (char)82, (char)251, (char)49, (char)179, (char)144, (char)48, (char)56, (char)80, (char)117, (char)62, (char)75, (char)101, (char)244, (char)71, (char)38, (char)217, (char)244, (char)229, (char)122, (char)74, (char)204, (char)74, (char)48, (char)74, (char)88, (char)191, (char)74, (char)186, (char)45, (char)185, (char)27, (char)253, (char)121, (char)235, (char)100, (char)151, (char)238, (char)120, (char)93, (char)183, (char)255, (char)245, (char)64, (char)176, (char)168, (char)238, (char)245, (char)76, (char)140, (char)186, (char)201, (char)149, (char)38, (char)187, (char)106, (char)69, (char)184, (char)159, (char)220, (char)53, (char)220, (char)6, (char)166, (char)35, (char)76, (char)150, (char)19, (char)97, (char)164, (char)255, (char)188, (char)132, (char)140, (char)245, (char)7, (char)11, (char)99, (char)31, (char)101, (char)75, (char)75, (char)148, (char)5, (char)225, (char)196, (char)159, (char)172, (char)178, (char)195, (char)232, (char)100, (char)178, (char)133, (char)16, (char)212, (char)169, (char)123, (char)221, (char)235, (char)171, (char)155, (char)78, (char)110, (char)141, (char)142, (char)200, (char)47, (char)224, (char)119, (char)153, (char)77, (char)92, (char)202, (char)209, (char)199, (char)44, (char)28, (char)183, (char)243, (char)40, (char)55, (char)94, (char)110, (char)219, (char)58, (char)94, (char)26, (char)100, (char)186, (char)173, (char)86, (char)186, (char)95, (char)141, (char)143, (char)165, (char)187, (char)53, (char)218, (char)243, (char)244, (char)196, (char)88, (char)106, (char)159, (char)66, (char)252, (char)186, (char)200, (char)83, (char)8, (char)154, (char)225, (char)31, (char)177, (char)32, (char)193, (char)185, (char)235, (char)81, (char)174, (char)221, (char)147, (char)32, (char)56, (char)57, (char)203, (char)92, (char)196, (char)69, (char)62, (char)29, (char)99, (char)104, (char)248, (char)252, (char)204, (char)97, (char)230, (char)218, (char)83, (char)107, (char)205, (char)240, (char)29, (char)60, (char)101, (char)186, (char)90, (char)184, (char)96, (char)29, (char)120, (char)226, (char)58, (char)229, (char)104, (char)165, (char)151, (char)144, (char)199, (char)190, (char)232, (char)32, (char)69, (char)85, (char)6, (char)98, (char)204, (char)85, (char)247, (char)53}));
            assert(pack.length_GET() == (char)69);
            assert(pack.target_system_GET() == (char)16);
            assert(pack.sequence_GET() == (char)39642);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.first_message_offset_SET((char)173) ;
        p266.target_system_SET((char)16) ;
        p266.sequence_SET((char)39642) ;
        p266.target_component_SET((char)46) ;
        p266.data__SET(new char[] {(char)86, (char)71, (char)13, (char)78, (char)44, (char)60, (char)181, (char)237, (char)54, (char)93, (char)105, (char)190, (char)167, (char)233, (char)207, (char)74, (char)93, (char)170, (char)14, (char)7, (char)229, (char)11, (char)223, (char)118, (char)61, (char)212, (char)245, (char)178, (char)43, (char)144, (char)151, (char)133, (char)125, (char)169, (char)104, (char)66, (char)158, (char)82, (char)251, (char)49, (char)179, (char)144, (char)48, (char)56, (char)80, (char)117, (char)62, (char)75, (char)101, (char)244, (char)71, (char)38, (char)217, (char)244, (char)229, (char)122, (char)74, (char)204, (char)74, (char)48, (char)74, (char)88, (char)191, (char)74, (char)186, (char)45, (char)185, (char)27, (char)253, (char)121, (char)235, (char)100, (char)151, (char)238, (char)120, (char)93, (char)183, (char)255, (char)245, (char)64, (char)176, (char)168, (char)238, (char)245, (char)76, (char)140, (char)186, (char)201, (char)149, (char)38, (char)187, (char)106, (char)69, (char)184, (char)159, (char)220, (char)53, (char)220, (char)6, (char)166, (char)35, (char)76, (char)150, (char)19, (char)97, (char)164, (char)255, (char)188, (char)132, (char)140, (char)245, (char)7, (char)11, (char)99, (char)31, (char)101, (char)75, (char)75, (char)148, (char)5, (char)225, (char)196, (char)159, (char)172, (char)178, (char)195, (char)232, (char)100, (char)178, (char)133, (char)16, (char)212, (char)169, (char)123, (char)221, (char)235, (char)171, (char)155, (char)78, (char)110, (char)141, (char)142, (char)200, (char)47, (char)224, (char)119, (char)153, (char)77, (char)92, (char)202, (char)209, (char)199, (char)44, (char)28, (char)183, (char)243, (char)40, (char)55, (char)94, (char)110, (char)219, (char)58, (char)94, (char)26, (char)100, (char)186, (char)173, (char)86, (char)186, (char)95, (char)141, (char)143, (char)165, (char)187, (char)53, (char)218, (char)243, (char)244, (char)196, (char)88, (char)106, (char)159, (char)66, (char)252, (char)186, (char)200, (char)83, (char)8, (char)154, (char)225, (char)31, (char)177, (char)32, (char)193, (char)185, (char)235, (char)81, (char)174, (char)221, (char)147, (char)32, (char)56, (char)57, (char)203, (char)92, (char)196, (char)69, (char)62, (char)29, (char)99, (char)104, (char)248, (char)252, (char)204, (char)97, (char)230, (char)218, (char)83, (char)107, (char)205, (char)240, (char)29, (char)60, (char)101, (char)186, (char)90, (char)184, (char)96, (char)29, (char)120, (char)226, (char)58, (char)229, (char)104, (char)165, (char)151, (char)144, (char)199, (char)190, (char)232, (char)32, (char)69, (char)85, (char)6, (char)98, (char)204, (char)85, (char)247, (char)53}, 0) ;
        p266.length_SET((char)69) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.length_GET() == (char)217);
            assert(pack.sequence_GET() == (char)1047);
            assert(pack.target_system_GET() == (char)199);
            assert(pack.first_message_offset_GET() == (char)195);
            assert(pack.target_component_GET() == (char)146);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)141, (char)91, (char)25, (char)143, (char)253, (char)185, (char)91, (char)224, (char)45, (char)71, (char)33, (char)241, (char)156, (char)200, (char)153, (char)83, (char)70, (char)234, (char)162, (char)66, (char)42, (char)34, (char)193, (char)64, (char)154, (char)43, (char)69, (char)192, (char)25, (char)45, (char)81, (char)181, (char)239, (char)139, (char)177, (char)42, (char)49, (char)209, (char)221, (char)40, (char)78, (char)62, (char)86, (char)4, (char)98, (char)193, (char)62, (char)2, (char)4, (char)98, (char)3, (char)143, (char)233, (char)129, (char)249, (char)208, (char)138, (char)32, (char)147, (char)233, (char)84, (char)208, (char)137, (char)21, (char)181, (char)102, (char)130, (char)111, (char)140, (char)155, (char)9, (char)251, (char)71, (char)232, (char)95, (char)61, (char)74, (char)226, (char)21, (char)193, (char)153, (char)141, (char)60, (char)158, (char)92, (char)222, (char)96, (char)71, (char)183, (char)84, (char)62, (char)58, (char)247, (char)115, (char)232, (char)233, (char)16, (char)80, (char)164, (char)77, (char)250, (char)194, (char)225, (char)67, (char)60, (char)91, (char)142, (char)225, (char)223, (char)48, (char)37, (char)195, (char)119, (char)48, (char)230, (char)74, (char)241, (char)232, (char)205, (char)143, (char)255, (char)224, (char)230, (char)148, (char)241, (char)10, (char)103, (char)79, (char)247, (char)139, (char)247, (char)1, (char)221, (char)63, (char)100, (char)113, (char)126, (char)231, (char)87, (char)19, (char)127, (char)91, (char)213, (char)86, (char)20, (char)179, (char)132, (char)45, (char)138, (char)19, (char)148, (char)110, (char)201, (char)23, (char)242, (char)151, (char)115, (char)221, (char)188, (char)120, (char)135, (char)30, (char)73, (char)248, (char)3, (char)194, (char)144, (char)179, (char)46, (char)122, (char)11, (char)60, (char)153, (char)1, (char)60, (char)149, (char)135, (char)60, (char)166, (char)211, (char)242, (char)155, (char)37, (char)206, (char)94, (char)176, (char)39, (char)213, (char)238, (char)221, (char)75, (char)196, (char)211, (char)90, (char)154, (char)155, (char)65, (char)63, (char)100, (char)243, (char)135, (char)173, (char)75, (char)14, (char)56, (char)178, (char)127, (char)203, (char)112, (char)163, (char)107, (char)183, (char)98, (char)34, (char)213, (char)133, (char)153, (char)16, (char)47, (char)134, (char)243, (char)61, (char)139, (char)85, (char)174, (char)78, (char)158, (char)103, (char)11, (char)171, (char)35, (char)83, (char)179, (char)231, (char)20, (char)244, (char)175, (char)105, (char)147, (char)142, (char)134, (char)191, (char)128, (char)152, (char)238, (char)32, (char)31, (char)83, (char)125}));
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.first_message_offset_SET((char)195) ;
        p267.length_SET((char)217) ;
        p267.sequence_SET((char)1047) ;
        p267.target_component_SET((char)146) ;
        p267.data__SET(new char[] {(char)141, (char)91, (char)25, (char)143, (char)253, (char)185, (char)91, (char)224, (char)45, (char)71, (char)33, (char)241, (char)156, (char)200, (char)153, (char)83, (char)70, (char)234, (char)162, (char)66, (char)42, (char)34, (char)193, (char)64, (char)154, (char)43, (char)69, (char)192, (char)25, (char)45, (char)81, (char)181, (char)239, (char)139, (char)177, (char)42, (char)49, (char)209, (char)221, (char)40, (char)78, (char)62, (char)86, (char)4, (char)98, (char)193, (char)62, (char)2, (char)4, (char)98, (char)3, (char)143, (char)233, (char)129, (char)249, (char)208, (char)138, (char)32, (char)147, (char)233, (char)84, (char)208, (char)137, (char)21, (char)181, (char)102, (char)130, (char)111, (char)140, (char)155, (char)9, (char)251, (char)71, (char)232, (char)95, (char)61, (char)74, (char)226, (char)21, (char)193, (char)153, (char)141, (char)60, (char)158, (char)92, (char)222, (char)96, (char)71, (char)183, (char)84, (char)62, (char)58, (char)247, (char)115, (char)232, (char)233, (char)16, (char)80, (char)164, (char)77, (char)250, (char)194, (char)225, (char)67, (char)60, (char)91, (char)142, (char)225, (char)223, (char)48, (char)37, (char)195, (char)119, (char)48, (char)230, (char)74, (char)241, (char)232, (char)205, (char)143, (char)255, (char)224, (char)230, (char)148, (char)241, (char)10, (char)103, (char)79, (char)247, (char)139, (char)247, (char)1, (char)221, (char)63, (char)100, (char)113, (char)126, (char)231, (char)87, (char)19, (char)127, (char)91, (char)213, (char)86, (char)20, (char)179, (char)132, (char)45, (char)138, (char)19, (char)148, (char)110, (char)201, (char)23, (char)242, (char)151, (char)115, (char)221, (char)188, (char)120, (char)135, (char)30, (char)73, (char)248, (char)3, (char)194, (char)144, (char)179, (char)46, (char)122, (char)11, (char)60, (char)153, (char)1, (char)60, (char)149, (char)135, (char)60, (char)166, (char)211, (char)242, (char)155, (char)37, (char)206, (char)94, (char)176, (char)39, (char)213, (char)238, (char)221, (char)75, (char)196, (char)211, (char)90, (char)154, (char)155, (char)65, (char)63, (char)100, (char)243, (char)135, (char)173, (char)75, (char)14, (char)56, (char)178, (char)127, (char)203, (char)112, (char)163, (char)107, (char)183, (char)98, (char)34, (char)213, (char)133, (char)153, (char)16, (char)47, (char)134, (char)243, (char)61, (char)139, (char)85, (char)174, (char)78, (char)158, (char)103, (char)11, (char)171, (char)35, (char)83, (char)179, (char)231, (char)20, (char)244, (char)175, (char)105, (char)147, (char)142, (char)134, (char)191, (char)128, (char)152, (char)238, (char)32, (char)31, (char)83, (char)125}, 0) ;
        p267.target_system_SET((char)199) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)61);
            assert(pack.target_component_GET() == (char)15);
            assert(pack.sequence_GET() == (char)56611);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)61) ;
        p268.target_component_SET((char)15) ;
        p268.sequence_SET((char)56611) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.resolution_v_GET() == (char)23646);
            assert(pack.uri_LEN(ph) == 82);
            assert(pack.uri_TRY(ph).equals("kxtoysoydxCyewgoevpyvdealbgXegdykphgyfrargvvzzpQfvlzhbssoqcVhrvlekDYzKgnrzPnshlawa"));
            assert(pack.framerate_GET() == -2.7637789E38F);
            assert(pack.resolution_h_GET() == (char)30325);
            assert(pack.status_GET() == (char)55);
            assert(pack.camera_id_GET() == (char)162);
            assert(pack.bitrate_GET() == 3482742588L);
            assert(pack.rotation_GET() == (char)57315);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.rotation_SET((char)57315) ;
        p269.uri_SET("kxtoysoydxCyewgoevpyvdealbgXegdykphgyfrargvvzzpQfvlzhbssoqcVhrvlekDYzKgnrzPnshlawa", PH) ;
        p269.resolution_v_SET((char)23646) ;
        p269.resolution_h_SET((char)30325) ;
        p269.status_SET((char)55) ;
        p269.camera_id_SET((char)162) ;
        p269.framerate_SET(-2.7637789E38F) ;
        p269.bitrate_SET(3482742588L) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.uri_LEN(ph) == 117);
            assert(pack.uri_TRY(ph).equals("ixGHxjtzahylrsduqbqLybgyyKiejFwrrttrdgauetHumJqsrwovmGxyriyhfsnmoueipHugcqkisteLjhwusxlproghtodqdgMcCrpittjbjhnhuivcf"));
            assert(pack.rotation_GET() == (char)65271);
            assert(pack.resolution_v_GET() == (char)56224);
            assert(pack.resolution_h_GET() == (char)59092);
            assert(pack.framerate_GET() == 1.8599613E38F);
            assert(pack.camera_id_GET() == (char)78);
            assert(pack.target_system_GET() == (char)228);
            assert(pack.bitrate_GET() == 3520792971L);
            assert(pack.target_component_GET() == (char)157);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.uri_SET("ixGHxjtzahylrsduqbqLybgyyKiejFwrrttrdgauetHumJqsrwovmGxyriyhfsnmoueipHugcqkisteLjhwusxlproghtodqdgMcCrpittjbjhnhuivcf", PH) ;
        p270.bitrate_SET(3520792971L) ;
        p270.resolution_h_SET((char)59092) ;
        p270.framerate_SET(1.8599613E38F) ;
        p270.target_system_SET((char)228) ;
        p270.rotation_SET((char)65271) ;
        p270.resolution_v_SET((char)56224) ;
        p270.camera_id_SET((char)78) ;
        p270.target_component_SET((char)157) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 28);
            assert(pack.ssid_TRY(ph).equals("nuyqdoeulmSbwoqKXzqUvyjahgls"));
            assert(pack.password_LEN(ph) == 45);
            assert(pack.password_TRY(ph).equals("cecwrhhdajumnbttdypvkYjwaduppbirohxkubladppme"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("nuyqdoeulmSbwoqKXzqUvyjahgls", PH) ;
        p299.password_SET("cecwrhhdajumnbttdypvkYjwaduppbirohxkubladppme", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == (char)31293);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)144, (char)15, (char)60, (char)76, (char)159, (char)2, (char)254, (char)5}));
            assert(pack.max_version_GET() == (char)48145);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)167, (char)232, (char)28, (char)84, (char)116, (char)81, (char)252, (char)127}));
            assert(pack.min_version_GET() == (char)12534);
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)31293) ;
        p300.spec_version_hash_SET(new char[] {(char)167, (char)232, (char)28, (char)84, (char)116, (char)81, (char)252, (char)127}, 0) ;
        p300.max_version_SET((char)48145) ;
        p300.library_version_hash_SET(new char[] {(char)144, (char)15, (char)60, (char)76, (char)159, (char)2, (char)254, (char)5}, 0) ;
        p300.min_version_SET((char)12534) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.vendor_specific_status_code_GET() == (char)60214);
            assert(pack.sub_mode_GET() == (char)248);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
            assert(pack.uptime_sec_GET() == 2015264171L);
            assert(pack.time_usec_GET() == 7889853137886046718L);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR) ;
        p310.sub_mode_SET((char)248) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE) ;
        p310.uptime_sec_SET(2015264171L) ;
        p310.vendor_specific_status_code_SET((char)60214) ;
        p310.time_usec_SET(7889853137886046718L) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.hw_version_minor_GET() == (char)59);
            assert(pack.hw_version_major_GET() == (char)14);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)227, (char)179, (char)157, (char)126, (char)12, (char)165, (char)153, (char)23, (char)42, (char)122, (char)151, (char)88, (char)37, (char)18, (char)133, (char)163}));
            assert(pack.sw_vcs_commit_GET() == 2698939315L);
            assert(pack.sw_version_major_GET() == (char)124);
            assert(pack.name_LEN(ph) == 78);
            assert(pack.name_TRY(ph).equals("DjjynaWchfgefvqmjZbelqnudqaplkzoIsbOwfqawpmlgfvezcgNnxzBjgbdhzjuqxlZhwbudfsrop"));
            assert(pack.uptime_sec_GET() == 2939694182L);
            assert(pack.sw_version_minor_GET() == (char)23);
            assert(pack.time_usec_GET() == 8952230719989914000L);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_version_major_SET((char)14) ;
        p311.uptime_sec_SET(2939694182L) ;
        p311.sw_version_major_SET((char)124) ;
        p311.hw_version_minor_SET((char)59) ;
        p311.time_usec_SET(8952230719989914000L) ;
        p311.name_SET("DjjynaWchfgefvqmjZbelqnudqaplkzoIsbOwfqawpmlgfvezcgNnxzBjgbdhzjuqxlZhwbudfsrop", PH) ;
        p311.hw_unique_id_SET(new char[] {(char)227, (char)179, (char)157, (char)126, (char)12, (char)165, (char)153, (char)23, (char)42, (char)122, (char)151, (char)88, (char)37, (char)18, (char)133, (char)163}, 0) ;
        p311.sw_vcs_commit_SET(2698939315L) ;
        p311.sw_version_minor_SET((char)23) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 6);
            assert(pack.param_id_TRY(ph).equals("lkkKdr"));
            assert(pack.target_component_GET() == (char)173);
            assert(pack.target_system_GET() == (char)128);
            assert(pack.param_index_GET() == (short) -3);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_component_SET((char)173) ;
        p320.param_id_SET("lkkKdr", PH) ;
        p320.param_index_SET((short) -3) ;
        p320.target_system_SET((char)128) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)240);
            assert(pack.target_component_GET() == (char)157);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)240) ;
        p321.target_component_SET((char)157) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (char)43554);
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("jMy"));
            assert(pack.param_value_LEN(ph) == 65);
            assert(pack.param_value_TRY(ph).equals("mpxgusjfhaniuExkfeuzhbwxzgipEkvzmaPEkptqgbyutlztSTurkongfilxpqjlm"));
            assert(pack.param_count_GET() == (char)40553);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_value_SET("mpxgusjfhaniuExkfeuzhbwxzgipEkvzmaPEkptqgbyutlztSTurkongfilxpqjlm", PH) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8) ;
        p322.param_id_SET("jMy", PH) ;
        p322.param_count_SET((char)40553) ;
        p322.param_index_SET((char)43554) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
            assert(pack.target_system_GET() == (char)127);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("jhtjiufoomyt"));
            assert(pack.target_component_GET() == (char)189);
            assert(pack.param_value_LEN(ph) == 110);
            assert(pack.param_value_TRY(ph).equals("fguerfvoreshztXzsgsyfdipcrqsglckilnwNbXYdzXppzIsxvnnizclvwvcnadnxmGbfxhaYqjvqvltbeqlclqwuyvpzmmoymdevomxwthxur"));
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)127) ;
        p323.target_component_SET((char)189) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM) ;
        p323.param_id_SET("jhtjiufoomyt", PH) ;
        p323.param_value_SET("fguerfvoreshztXzsgsyfdipcrqsglckilnwNbXYdzXppzIsxvnnizclvwvcnadnxmGbfxhaYqjvqvltbeqlclqwuyvpzmmoymdevomxwthxur", PH) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("eytU"));
            assert(pack.param_value_LEN(ph) == 128);
            assert(pack.param_value_TRY(ph).equals("nqmSdwukFcnolrhwdxvMfbpxdhxwfkaqFsvcihcSUbnmzfhvGtdhnlkobrysxuonqsvxteZwydxntsjQswdytcSkTaqEwcgmzyqdgzwLatsafEjyqptCPkswumwkjalt"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_FAILED);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64) ;
        p324.param_id_SET("eytU", PH) ;
        p324.param_value_SET("nqmSdwukFcnolrhwdxvMfbpxdhxwfkaqFsvcihcSUbnmzfhvGtdhnlkobrysxuonqsvxteZwydxntsjQswdytcSkTaqEwcgmzyqdgzwLatsafEjyqptCPkswumwkjalt", PH) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.time_usec_GET() == 2656221098157226886L);
            assert(pack.min_distance_GET() == (char)48597);
            assert(pack.max_distance_GET() == (char)13443);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)25452, (char)33943, (char)99, (char)43675, (char)37233, (char)60038, (char)7578, (char)60777, (char)35354, (char)34140, (char)50728, (char)63003, (char)3363, (char)16073, (char)1124, (char)12686, (char)65185, (char)16651, (char)27411, (char)7770, (char)7536, (char)59854, (char)43563, (char)59869, (char)56692, (char)17572, (char)36782, (char)26864, (char)13744, (char)30059, (char)14425, (char)7267, (char)11706, (char)37714, (char)64975, (char)52192, (char)10825, (char)21601, (char)46062, (char)11792, (char)19187, (char)48734, (char)24615, (char)11493, (char)45470, (char)59952, (char)6318, (char)31865, (char)30212, (char)9962, (char)22218, (char)35439, (char)34770, (char)34172, (char)62560, (char)34494, (char)23480, (char)11425, (char)52704, (char)54317, (char)32696, (char)8536, (char)54524, (char)5036, (char)36722, (char)22540, (char)6946, (char)64655, (char)44613, (char)31876, (char)63364, (char)36761}));
            assert(pack.increment_GET() == (char)71);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.distances_SET(new char[] {(char)25452, (char)33943, (char)99, (char)43675, (char)37233, (char)60038, (char)7578, (char)60777, (char)35354, (char)34140, (char)50728, (char)63003, (char)3363, (char)16073, (char)1124, (char)12686, (char)65185, (char)16651, (char)27411, (char)7770, (char)7536, (char)59854, (char)43563, (char)59869, (char)56692, (char)17572, (char)36782, (char)26864, (char)13744, (char)30059, (char)14425, (char)7267, (char)11706, (char)37714, (char)64975, (char)52192, (char)10825, (char)21601, (char)46062, (char)11792, (char)19187, (char)48734, (char)24615, (char)11493, (char)45470, (char)59952, (char)6318, (char)31865, (char)30212, (char)9962, (char)22218, (char)35439, (char)34770, (char)34172, (char)62560, (char)34494, (char)23480, (char)11425, (char)52704, (char)54317, (char)32696, (char)8536, (char)54524, (char)5036, (char)36722, (char)22540, (char)6946, (char)64655, (char)44613, (char)31876, (char)63364, (char)36761}, 0) ;
        p330.min_distance_SET((char)48597) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p330.max_distance_SET((char)13443) ;
        p330.increment_SET((char)71) ;
        p330.time_usec_SET(2656221098157226886L) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStreamAdvanced, TestChannel.instance);
    }

}
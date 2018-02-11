
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
            long id = id__B(src);
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
            long id = id__o(src);
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
            long id = id__o(src);
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
            long id = id__o(src);
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
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_UDB);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_POWEROFF);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED));
            assert(pack.mavlink_version_GET() == (char)46);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_TRICOPTER);
            assert(pack.custom_mode_GET() == 3499390250L);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.custom_mode_SET(3499390250L) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_UDB) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED)) ;
        p0.mavlink_version_SET((char)46) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_POWEROFF) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_TRICOPTER) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.battery_remaining_GET() == (byte) - 124);
            assert(pack.errors_comm_GET() == (char)27589);
            assert(pack.voltage_battery_GET() == (char)13403);
            assert(pack.current_battery_GET() == (short) -22238);
            assert(pack.errors_count1_GET() == (char)7924);
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.load_GET() == (char)63690);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
            assert(pack.errors_count2_GET() == (char)60323);
            assert(pack.drop_rate_comm_GET() == (char)28770);
            assert(pack.errors_count4_GET() == (char)28502);
            assert(pack.errors_count3_GET() == (char)63940);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO));
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.errors_count4_SET((char)28502) ;
        p1.drop_rate_comm_SET((char)28770) ;
        p1.load_SET((char)63690) ;
        p1.errors_count1_SET((char)7924) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO)) ;
        p1.errors_count2_SET((char)60323) ;
        p1.battery_remaining_SET((byte) - 124) ;
        p1.current_battery_SET((short) -22238) ;
        p1.voltage_battery_SET((char)13403) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.errors_comm_SET((char)27589) ;
        p1.errors_count3_SET((char)63940) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3096169679L);
            assert(pack.time_unix_usec_GET() == 5865507860733226928L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(5865507860733226928L) ;
        p2.time_boot_ms_SET(3096169679L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.afy_GET() == -6.57621E37F);
            assert(pack.z_GET() == -1.4467833E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.yaw_rate_GET() == 1.195035E38F);
            assert(pack.time_boot_ms_GET() == 486532609L);
            assert(pack.x_GET() == -1.0904125E38F);
            assert(pack.afx_GET() == 1.9165325E38F);
            assert(pack.vx_GET() == 3.0359376E38F);
            assert(pack.vy_GET() == 5.3016657E37F);
            assert(pack.y_GET() == -7.578446E37F);
            assert(pack.yaw_GET() == 1.2702147E38F);
            assert(pack.afz_GET() == -1.9892202E38F);
            assert(pack.vz_GET() == 1.9147887E38F);
            assert(pack.type_mask_GET() == (char)49223);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.yaw_rate_SET(1.195035E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p3.vz_SET(1.9147887E38F) ;
        p3.afx_SET(1.9165325E38F) ;
        p3.vy_SET(5.3016657E37F) ;
        p3.y_SET(-7.578446E37F) ;
        p3.afz_SET(-1.9892202E38F) ;
        p3.time_boot_ms_SET(486532609L) ;
        p3.afy_SET(-6.57621E37F) ;
        p3.vx_SET(3.0359376E38F) ;
        p3.x_SET(-1.0904125E38F) ;
        p3.type_mask_SET((char)49223) ;
        p3.yaw_SET(1.2702147E38F) ;
        p3.z_SET(-1.4467833E38F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 3011233231L);
            assert(pack.target_component_GET() == (char)242);
            assert(pack.target_system_GET() == (char)234);
            assert(pack.time_usec_GET() == 6684718216810295159L);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.time_usec_SET(6684718216810295159L) ;
        p4.target_system_SET((char)234) ;
        p4.seq_SET(3011233231L) ;
        p4.target_component_SET((char)242) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == (char)45);
            assert(pack.control_request_GET() == (char)181);
            assert(pack.target_system_GET() == (char)198);
            assert(pack.passkey_LEN(ph) == 4);
            assert(pack.passkey_TRY(ph).equals("mecz"));
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)198) ;
        p5.control_request_SET((char)181) ;
        p5.passkey_SET("mecz", PH) ;
        p5.version_SET((char)45) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.ack_GET() == (char)112);
            assert(pack.control_request_GET() == (char)251);
            assert(pack.gcs_system_id_GET() == (char)45);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.ack_SET((char)112) ;
        p6.gcs_system_id_SET((char)45) ;
        p6.control_request_SET((char)251) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 17);
            assert(pack.key_TRY(ph).equals("swrvscAwUgekgaarv"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("swrvscAwUgekgaarv", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_GUIDED_ARMED);
            assert(pack.target_system_GET() == (char)47);
            assert(pack.custom_mode_GET() == 3322499830L);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.custom_mode_SET(3322499830L) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED) ;
        p11.target_system_SET((char)47) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)81);
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("hmaoa"));
            assert(pack.target_system_GET() == (char)69);
            assert(pack.param_index_GET() == (short) -23283);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_id_SET("hmaoa", PH) ;
        p20.target_system_SET((char)69) ;
        p20.param_index_SET((short) -23283) ;
        p20.target_component_SET((char)81) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)169);
            assert(pack.target_system_GET() == (char)98);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)169) ;
        p21.target_system_SET((char)98) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("Tcf"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8);
            assert(pack.param_value_GET() == 1.334357E38F);
            assert(pack.param_index_GET() == (char)60999);
            assert(pack.param_count_GET() == (char)574);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_value_SET(1.334357E38F) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8) ;
        p22.param_index_SET((char)60999) ;
        p22.param_id_SET("Tcf", PH) ;
        p22.param_count_SET((char)574) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("prhvqoWbssnyhh"));
            assert(pack.target_system_GET() == (char)203);
            assert(pack.target_component_GET() == (char)56);
            assert(pack.param_value_GET() == 1.1114417E38F);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.param_id_SET("prhvqoWbssnyhh", PH) ;
        p23.param_value_SET(1.1114417E38F) ;
        p23.target_system_SET((char)203) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32) ;
        p23.target_component_SET((char)56) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.vel_GET() == (char)1224);
            assert(pack.vel_acc_TRY(ph) == 3736415624L);
            assert(pack.lon_GET() == 151689524);
            assert(pack.alt_ellipsoid_TRY(ph) == 1558309038);
            assert(pack.time_usec_GET() == 5827786149069749932L);
            assert(pack.v_acc_TRY(ph) == 45176095L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
            assert(pack.hdg_acc_TRY(ph) == 83438394L);
            assert(pack.cog_GET() == (char)4456);
            assert(pack.lat_GET() == 644272550);
            assert(pack.satellites_visible_GET() == (char)10);
            assert(pack.eph_GET() == (char)44955);
            assert(pack.h_acc_TRY(ph) == 4005694485L);
            assert(pack.alt_GET() == -39321364);
            assert(pack.epv_GET() == (char)36756);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.v_acc_SET(45176095L, PH) ;
        p24.lat_SET(644272550) ;
        p24.cog_SET((char)4456) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX) ;
        p24.alt_ellipsoid_SET(1558309038, PH) ;
        p24.alt_SET(-39321364) ;
        p24.lon_SET(151689524) ;
        p24.eph_SET((char)44955) ;
        p24.epv_SET((char)36756) ;
        p24.vel_SET((char)1224) ;
        p24.satellites_visible_SET((char)10) ;
        p24.hdg_acc_SET(83438394L, PH) ;
        p24.vel_acc_SET(3736415624L, PH) ;
        p24.h_acc_SET(4005694485L, PH) ;
        p24.time_usec_SET(5827786149069749932L) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)231, (char)71, (char)165, (char)199, (char)162, (char)197, (char)197, (char)28, (char)74, (char)62, (char)253, (char)206, (char)33, (char)30, (char)154, (char)187, (char)231, (char)225, (char)242, (char)144}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)113, (char)34, (char)161, (char)254, (char)121, (char)85, (char)15, (char)24, (char)200, (char)251, (char)52, (char)150, (char)210, (char)34, (char)4, (char)162, (char)139, (char)238, (char)2, (char)200}));
            assert(pack.satellites_visible_GET() == (char)93);
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)26, (char)107, (char)35, (char)153, (char)184, (char)47, (char)98, (char)19, (char)149, (char)228, (char)17, (char)139, (char)222, (char)134, (char)55, (char)108, (char)132, (char)5, (char)36, (char)65}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)191, (char)152, (char)41, (char)37, (char)84, (char)106, (char)238, (char)85, (char)28, (char)209, (char)236, (char)44, (char)224, (char)178, (char)139, (char)162, (char)249, (char)89, (char)86, (char)152}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)101, (char)110, (char)220, (char)125, (char)193, (char)195, (char)96, (char)246, (char)225, (char)142, (char)34, (char)62, (char)169, (char)8, (char)250, (char)85, (char)237, (char)191, (char)168, (char)148}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_azimuth_SET(new char[] {(char)191, (char)152, (char)41, (char)37, (char)84, (char)106, (char)238, (char)85, (char)28, (char)209, (char)236, (char)44, (char)224, (char)178, (char)139, (char)162, (char)249, (char)89, (char)86, (char)152}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)231, (char)71, (char)165, (char)199, (char)162, (char)197, (char)197, (char)28, (char)74, (char)62, (char)253, (char)206, (char)33, (char)30, (char)154, (char)187, (char)231, (char)225, (char)242, (char)144}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)26, (char)107, (char)35, (char)153, (char)184, (char)47, (char)98, (char)19, (char)149, (char)228, (char)17, (char)139, (char)222, (char)134, (char)55, (char)108, (char)132, (char)5, (char)36, (char)65}, 0) ;
        p25.satellite_used_SET(new char[] {(char)101, (char)110, (char)220, (char)125, (char)193, (char)195, (char)96, (char)246, (char)225, (char)142, (char)34, (char)62, (char)169, (char)8, (char)250, (char)85, (char)237, (char)191, (char)168, (char)148}, 0) ;
        p25.satellites_visible_SET((char)93) ;
        p25.satellite_elevation_SET(new char[] {(char)113, (char)34, (char)161, (char)254, (char)121, (char)85, (char)15, (char)24, (char)200, (char)251, (char)52, (char)150, (char)210, (char)34, (char)4, (char)162, (char)139, (char)238, (char)2, (char)200}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == (short) -19866);
            assert(pack.time_boot_ms_GET() == 2964483342L);
            assert(pack.yacc_GET() == (short)26605);
            assert(pack.xgyro_GET() == (short)16281);
            assert(pack.ygyro_GET() == (short) -9237);
            assert(pack.zmag_GET() == (short) -22571);
            assert(pack.xmag_GET() == (short)31532);
            assert(pack.xacc_GET() == (short) -28005);
            assert(pack.zgyro_GET() == (short) -29811);
            assert(pack.zacc_GET() == (short)6397);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.yacc_SET((short)26605) ;
        p26.zacc_SET((short)6397) ;
        p26.xgyro_SET((short)16281) ;
        p26.ymag_SET((short) -19866) ;
        p26.xacc_SET((short) -28005) ;
        p26.xmag_SET((short)31532) ;
        p26.time_boot_ms_SET(2964483342L) ;
        p26.zgyro_SET((short) -29811) ;
        p26.zmag_SET((short) -22571) ;
        p26.ygyro_SET((short) -9237) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short)3986);
            assert(pack.yacc_GET() == (short) -2029);
            assert(pack.xgyro_GET() == (short)10489);
            assert(pack.time_usec_GET() == 1605791494817142278L);
            assert(pack.xmag_GET() == (short)8086);
            assert(pack.zmag_GET() == (short)25431);
            assert(pack.zacc_GET() == (short) -28242);
            assert(pack.xacc_GET() == (short)21385);
            assert(pack.ymag_GET() == (short)26539);
            assert(pack.zgyro_GET() == (short)29316);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.zgyro_SET((short)29316) ;
        p27.xmag_SET((short)8086) ;
        p27.zmag_SET((short)25431) ;
        p27.xacc_SET((short)21385) ;
        p27.xgyro_SET((short)10489) ;
        p27.yacc_SET((short) -2029) ;
        p27.zacc_SET((short) -28242) ;
        p27.time_usec_SET(1605791494817142278L) ;
        p27.ymag_SET((short)26539) ;
        p27.ygyro_SET((short)3986) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff2_GET() == (short) -15480);
            assert(pack.temperature_GET() == (short) -3310);
            assert(pack.press_diff1_GET() == (short)28481);
            assert(pack.time_usec_GET() == 2907649856014186475L);
            assert(pack.press_abs_GET() == (short)20834);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.time_usec_SET(2907649856014186475L) ;
        p28.temperature_SET((short) -3310) ;
        p28.press_abs_SET((short)20834) ;
        p28.press_diff1_SET((short)28481) ;
        p28.press_diff2_SET((short) -15480) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 4214895705L);
            assert(pack.press_abs_GET() == -2.4185642E38F);
            assert(pack.press_diff_GET() == -1.5175881E38F);
            assert(pack.temperature_GET() == (short) -11575);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.temperature_SET((short) -11575) ;
        p29.press_diff_SET(-1.5175881E38F) ;
        p29.press_abs_SET(-2.4185642E38F) ;
        p29.time_boot_ms_SET(4214895705L) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -2.949125E38F);
            assert(pack.yaw_GET() == 3.2137285E38F);
            assert(pack.roll_GET() == -1.0859401E38F);
            assert(pack.pitchspeed_GET() == 3.0829674E38F);
            assert(pack.time_boot_ms_GET() == 2739398455L);
            assert(pack.yawspeed_GET() == -2.1538227E38F);
            assert(pack.rollspeed_GET() == -2.5657828E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.roll_SET(-1.0859401E38F) ;
        p30.yaw_SET(3.2137285E38F) ;
        p30.pitch_SET(-2.949125E38F) ;
        p30.time_boot_ms_SET(2739398455L) ;
        p30.yawspeed_SET(-2.1538227E38F) ;
        p30.rollspeed_SET(-2.5657828E38F) ;
        p30.pitchspeed_SET(3.0829674E38F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == 2.6729318E38F);
            assert(pack.q4_GET() == 1.9432166E37F);
            assert(pack.q1_GET() == -2.4751841E38F);
            assert(pack.q2_GET() == -1.917535E38F);
            assert(pack.time_boot_ms_GET() == 472338167L);
            assert(pack.pitchspeed_GET() == -1.0013521E38F);
            assert(pack.q3_GET() == 3.396373E38F);
            assert(pack.yawspeed_GET() == -2.1103373E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.yawspeed_SET(-2.1103373E38F) ;
        p31.pitchspeed_SET(-1.0013521E38F) ;
        p31.q2_SET(-1.917535E38F) ;
        p31.q4_SET(1.9432166E37F) ;
        p31.time_boot_ms_SET(472338167L) ;
        p31.q1_SET(-2.4751841E38F) ;
        p31.rollspeed_SET(2.6729318E38F) ;
        p31.q3_SET(3.396373E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == -1.3301743E37F);
            assert(pack.time_boot_ms_GET() == 3273375645L);
            assert(pack.z_GET() == 1.8826084E38F);
            assert(pack.vz_GET() == 1.4389895E38F);
            assert(pack.y_GET() == -3.650769E37F);
            assert(pack.x_GET() == -2.4230523E38F);
            assert(pack.vy_GET() == -1.8174893E38F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.vz_SET(1.4389895E38F) ;
        p32.time_boot_ms_SET(3273375645L) ;
        p32.vx_SET(-1.3301743E37F) ;
        p32.vy_SET(-1.8174893E38F) ;
        p32.z_SET(1.8826084E38F) ;
        p32.x_SET(-2.4230523E38F) ;
        p32.y_SET(-3.650769E37F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1908812394);
            assert(pack.vx_GET() == (short) -12427);
            assert(pack.alt_GET() == -1554614181);
            assert(pack.time_boot_ms_GET() == 2711375506L);
            assert(pack.hdg_GET() == (char)50576);
            assert(pack.lon_GET() == -560803550);
            assert(pack.vz_GET() == (short) -18904);
            assert(pack.vy_GET() == (short) -15670);
            assert(pack.relative_alt_GET() == -1602387701);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.relative_alt_SET(-1602387701) ;
        p33.vz_SET((short) -18904) ;
        p33.time_boot_ms_SET(2711375506L) ;
        p33.vx_SET((short) -12427) ;
        p33.hdg_SET((char)50576) ;
        p33.alt_SET(-1554614181) ;
        p33.lon_SET(-560803550) ;
        p33.lat_SET(-1908812394) ;
        p33.vy_SET((short) -15670) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan6_scaled_GET() == (short) -28830);
            assert(pack.chan7_scaled_GET() == (short) -17752);
            assert(pack.time_boot_ms_GET() == 269946968L);
            assert(pack.rssi_GET() == (char)97);
            assert(pack.chan5_scaled_GET() == (short)5302);
            assert(pack.chan1_scaled_GET() == (short)30999);
            assert(pack.port_GET() == (char)191);
            assert(pack.chan3_scaled_GET() == (short)24577);
            assert(pack.chan2_scaled_GET() == (short) -22184);
            assert(pack.chan4_scaled_GET() == (short) -14147);
            assert(pack.chan8_scaled_GET() == (short)22178);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan4_scaled_SET((short) -14147) ;
        p34.chan1_scaled_SET((short)30999) ;
        p34.time_boot_ms_SET(269946968L) ;
        p34.chan3_scaled_SET((short)24577) ;
        p34.chan6_scaled_SET((short) -28830) ;
        p34.chan2_scaled_SET((short) -22184) ;
        p34.chan5_scaled_SET((short)5302) ;
        p34.chan8_scaled_SET((short)22178) ;
        p34.rssi_SET((char)97) ;
        p34.port_SET((char)191) ;
        p34.chan7_scaled_SET((short) -17752) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)122);
            assert(pack.chan1_raw_GET() == (char)31030);
            assert(pack.chan4_raw_GET() == (char)19742);
            assert(pack.chan8_raw_GET() == (char)28991);
            assert(pack.port_GET() == (char)179);
            assert(pack.chan3_raw_GET() == (char)52881);
            assert(pack.chan7_raw_GET() == (char)13314);
            assert(pack.chan2_raw_GET() == (char)13450);
            assert(pack.chan5_raw_GET() == (char)60578);
            assert(pack.time_boot_ms_GET() == 1089933017L);
            assert(pack.chan6_raw_GET() == (char)62577);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan1_raw_SET((char)31030) ;
        p35.chan3_raw_SET((char)52881) ;
        p35.chan2_raw_SET((char)13450) ;
        p35.rssi_SET((char)122) ;
        p35.time_boot_ms_SET(1089933017L) ;
        p35.port_SET((char)179) ;
        p35.chan8_raw_SET((char)28991) ;
        p35.chan5_raw_SET((char)60578) ;
        p35.chan6_raw_SET((char)62577) ;
        p35.chan7_raw_SET((char)13314) ;
        p35.chan4_raw_SET((char)19742) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo4_raw_GET() == (char)25065);
            assert(pack.servo10_raw_TRY(ph) == (char)20162);
            assert(pack.servo6_raw_GET() == (char)27404);
            assert(pack.servo11_raw_TRY(ph) == (char)3699);
            assert(pack.servo2_raw_GET() == (char)44222);
            assert(pack.servo3_raw_GET() == (char)61676);
            assert(pack.port_GET() == (char)138);
            assert(pack.servo8_raw_GET() == (char)56660);
            assert(pack.time_usec_GET() == 3674142415L);
            assert(pack.servo14_raw_TRY(ph) == (char)17094);
            assert(pack.servo15_raw_TRY(ph) == (char)2843);
            assert(pack.servo16_raw_TRY(ph) == (char)9360);
            assert(pack.servo13_raw_TRY(ph) == (char)4311);
            assert(pack.servo12_raw_TRY(ph) == (char)13899);
            assert(pack.servo7_raw_GET() == (char)58707);
            assert(pack.servo5_raw_GET() == (char)25265);
            assert(pack.servo9_raw_TRY(ph) == (char)38054);
            assert(pack.servo1_raw_GET() == (char)56616);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.port_SET((char)138) ;
        p36.servo3_raw_SET((char)61676) ;
        p36.servo1_raw_SET((char)56616) ;
        p36.servo2_raw_SET((char)44222) ;
        p36.servo9_raw_SET((char)38054, PH) ;
        p36.servo15_raw_SET((char)2843, PH) ;
        p36.servo13_raw_SET((char)4311, PH) ;
        p36.servo12_raw_SET((char)13899, PH) ;
        p36.servo8_raw_SET((char)56660) ;
        p36.time_usec_SET(3674142415L) ;
        p36.servo6_raw_SET((char)27404) ;
        p36.servo16_raw_SET((char)9360, PH) ;
        p36.servo4_raw_SET((char)25065) ;
        p36.servo11_raw_SET((char)3699, PH) ;
        p36.servo10_raw_SET((char)20162, PH) ;
        p36.servo5_raw_SET((char)25265) ;
        p36.servo14_raw_SET((char)17094, PH) ;
        p36.servo7_raw_SET((char)58707) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)221);
            assert(pack.end_index_GET() == (short)20766);
            assert(pack.target_component_GET() == (char)172);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.start_index_GET() == (short)12038);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_system_SET((char)221) ;
        p37.end_index_SET((short)20766) ;
        p37.target_component_SET((char)172) ;
        p37.start_index_SET((short)12038) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short)19138);
            assert(pack.target_system_GET() == (char)240);
            assert(pack.target_component_GET() == (char)9);
            assert(pack.end_index_GET() == (short) -5511);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.end_index_SET((short) -5511) ;
        p38.target_system_SET((char)240) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p38.start_index_SET((short)19138) ;
        p38.target_component_SET((char)9) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -1.6041488E38F);
            assert(pack.target_component_GET() == (char)139);
            assert(pack.autocontinue_GET() == (char)103);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.target_system_GET() == (char)124);
            assert(pack.current_GET() == (char)141);
            assert(pack.param4_GET() == 2.4270002E37F);
            assert(pack.z_GET() == -6.188172E36F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.seq_GET() == (char)42237);
            assert(pack.param2_GET() == 2.8924028E38F);
            assert(pack.param3_GET() == -4.4486124E37F);
            assert(pack.param1_GET() == -2.7312682E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_WAYPOINT_USER_1);
            assert(pack.y_GET() == 3.0994333E38F);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.seq_SET((char)42237) ;
        p39.param2_SET(2.8924028E38F) ;
        p39.autocontinue_SET((char)103) ;
        p39.target_system_SET((char)124) ;
        p39.y_SET(3.0994333E38F) ;
        p39.target_component_SET((char)139) ;
        p39.command_SET(MAV_CMD.MAV_CMD_WAYPOINT_USER_1) ;
        p39.x_SET(-1.6041488E38F) ;
        p39.param4_SET(2.4270002E37F) ;
        p39.current_SET((char)141) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p39.param1_SET(-2.7312682E38F) ;
        p39.param3_SET(-4.4486124E37F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p39.z_SET(-6.188172E36F) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)49);
            assert(pack.target_system_GET() == (char)170);
            assert(pack.seq_GET() == (char)12770);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p40.target_component_SET((char)49) ;
        p40.seq_SET((char)12770) ;
        p40.target_system_SET((char)170) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)223);
            assert(pack.seq_GET() == (char)5260);
            assert(pack.target_system_GET() == (char)254);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.seq_SET((char)5260) ;
        p41.target_component_SET((char)223) ;
        p41.target_system_SET((char)254) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)46689);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)46689) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)108);
            assert(pack.target_component_GET() == (char)6);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)108) ;
        p43.target_component_SET((char)6) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)108);
            assert(pack.count_GET() == (char)64808);
            assert(pack.target_system_GET() == (char)167);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_component_SET((char)108) ;
        p44.count_SET((char)64808) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p44.target_system_SET((char)167) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)81);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)129);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_component_SET((char)81) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p45.target_system_SET((char)129) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)4086);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)4086) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)85);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED);
            assert(pack.target_component_GET() == (char)80);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.target_component_SET((char)80) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED) ;
        p47.target_system_SET((char)85) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)214);
            assert(pack.latitude_GET() == 534834043);
            assert(pack.longitude_GET() == -826792877);
            assert(pack.time_usec_TRY(ph) == 2107541143577738091L);
            assert(pack.altitude_GET() == -511121678);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.altitude_SET(-511121678) ;
        p48.longitude_SET(-826792877) ;
        p48.target_system_SET((char)214) ;
        p48.time_usec_SET(2107541143577738091L, PH) ;
        p48.latitude_SET(534834043) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 926356203608399073L);
            assert(pack.latitude_GET() == -749329307);
            assert(pack.longitude_GET() == -1588028856);
            assert(pack.altitude_GET() == 842405504);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.altitude_SET(842405504) ;
        p49.time_usec_SET(926356203608399073L, PH) ;
        p49.longitude_SET(-1588028856) ;
        p49.latitude_SET(-749329307) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_value_min_GET() == -1.0409895E38F);
            assert(pack.param_index_GET() == (short) -2815);
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("hjgrlnrcelb"));
            assert(pack.target_component_GET() == (char)141);
            assert(pack.scale_GET() == -8.315332E37F);
            assert(pack.parameter_rc_channel_index_GET() == (char)16);
            assert(pack.param_value0_GET() == -3.7675025E37F);
            assert(pack.target_system_GET() == (char)96);
            assert(pack.param_value_max_GET() == -2.0931325E38F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.param_value_max_SET(-2.0931325E38F) ;
        p50.param_id_SET("hjgrlnrcelb", PH) ;
        p50.param_value_min_SET(-1.0409895E38F) ;
        p50.param_index_SET((short) -2815) ;
        p50.parameter_rc_channel_index_SET((char)16) ;
        p50.target_system_SET((char)96) ;
        p50.scale_SET(-8.315332E37F) ;
        p50.param_value0_SET(-3.7675025E37F) ;
        p50.target_component_SET((char)141) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)165);
            assert(pack.seq_GET() == (char)8838);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)106);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_component_SET((char)165) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p51.seq_SET((char)8838) ;
        p51.target_system_SET((char)106) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.p1z_GET() == 1.5941036E38F);
            assert(pack.p2z_GET() == -1.5693201E38F);
            assert(pack.p2x_GET() == 9.315126E37F);
            assert(pack.p1x_GET() == 1.0361094E38F);
            assert(pack.p1y_GET() == 1.3316113E38F);
            assert(pack.target_component_GET() == (char)62);
            assert(pack.p2y_GET() == 2.1788464E38F);
            assert(pack.target_system_GET() == (char)59);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p2y_SET(2.1788464E38F) ;
        p54.p1y_SET(1.3316113E38F) ;
        p54.p2x_SET(9.315126E37F) ;
        p54.target_system_SET((char)59) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p54.p2z_SET(-1.5693201E38F) ;
        p54.p1z_SET(1.5941036E38F) ;
        p54.target_component_SET((char)62) ;
        p54.p1x_SET(1.0361094E38F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1y_GET() == -2.1062885E38F);
            assert(pack.p1x_GET() == -2.2506086E38F);
            assert(pack.p2x_GET() == -1.4818649E38F);
            assert(pack.p2y_GET() == -3.7721578E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.p1z_GET() == -6.6738006E37F);
            assert(pack.p2z_GET() == 1.5703331E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p55.p2x_SET(-1.4818649E38F) ;
        p55.p2y_SET(-3.7721578E37F) ;
        p55.p1z_SET(-6.6738006E37F) ;
        p55.p2z_SET(1.5703331E38F) ;
        p55.p1y_SET(-2.1062885E38F) ;
        p55.p1x_SET(-2.2506086E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 2.926384E38F);
            assert(pack.pitchspeed_GET() == -2.9216671E38F);
            assert(pack.rollspeed_GET() == -2.5603165E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.5150397E38F, -1.0186882E38F, 5.307795E37F, -2.3316845E38F, 1.8272628E38F, -2.4570553E38F, -2.7924383E38F, 2.5337184E37F, 1.9869052E37F}));
            assert(pack.time_usec_GET() == 3449320184621944669L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-8.735964E37F, -5.019274E36F, -1.7111993E38F, -1.8864848E38F}));
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.q_SET(new float[] {-8.735964E37F, -5.019274E36F, -1.7111993E38F, -1.8864848E38F}, 0) ;
        p61.covariance_SET(new float[] {2.5150397E38F, -1.0186882E38F, 5.307795E37F, -2.3316845E38F, 1.8272628E38F, -2.4570553E38F, -2.7924383E38F, 2.5337184E37F, 1.9869052E37F}, 0) ;
        p61.time_usec_SET(3449320184621944669L) ;
        p61.yawspeed_SET(2.926384E38F) ;
        p61.rollspeed_SET(-2.5603165E38F) ;
        p61.pitchspeed_SET(-2.9216671E38F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.target_bearing_GET() == (short) -20422);
            assert(pack.nav_bearing_GET() == (short) -31549);
            assert(pack.alt_error_GET() == 2.1104186E38F);
            assert(pack.aspd_error_GET() == 3.3166364E38F);
            assert(pack.xtrack_error_GET() == 2.3654928E38F);
            assert(pack.nav_pitch_GET() == 9.02598E37F);
            assert(pack.wp_dist_GET() == (char)31086);
            assert(pack.nav_roll_GET() == -7.3646205E37F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_pitch_SET(9.02598E37F) ;
        p62.alt_error_SET(2.1104186E38F) ;
        p62.wp_dist_SET((char)31086) ;
        p62.target_bearing_SET((short) -20422) ;
        p62.xtrack_error_SET(2.3654928E38F) ;
        p62.nav_roll_SET(-7.3646205E37F) ;
        p62.nav_bearing_SET((short) -31549) ;
        p62.aspd_error_SET(3.3166364E38F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 1951567248);
            assert(pack.relative_alt_GET() == -217251630);
            assert(pack.vy_GET() == 1.7630748E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.6367984E38F, 7.533185E37F, -1.8234584E38F, -5.7089086E37F, -1.2030061E37F, 3.1918897E38F, -3.3906885E37F, -2.9856346E38F, -8.507665E37F, -2.7078177E38F, 1.9042623E38F, 2.0531693E38F, -9.675788E37F, 2.8762524E38F, 1.5990862E38F, -3.525205E37F, 1.6911202E38F, -2.6493754E38F, -5.0112104E37F, -1.5857029E38F, 2.48927E38F, -1.2617466E38F, 2.0411852E38F, 1.7255015E38F, 2.9203674E38F, -2.1968542E37F, 1.5364031E38F, -3.1461147E38F, 1.4402669E38F, -2.4492105E38F, 1.320282E38F, 1.7349334E38F, 2.4083743E38F, 2.0626687E38F, 2.3537876E38F, 1.3099973E38F}));
            assert(pack.lat_GET() == -812387164);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
            assert(pack.vx_GET() == 1.6823944E37F);
            assert(pack.vz_GET() == 2.5673679E38F);
            assert(pack.lon_GET() == 1198006681);
            assert(pack.time_usec_GET() == 672496232252084888L);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.vy_SET(1.7630748E38F) ;
        p63.vz_SET(2.5673679E38F) ;
        p63.alt_SET(1951567248) ;
        p63.relative_alt_SET(-217251630) ;
        p63.lon_SET(1198006681) ;
        p63.time_usec_SET(672496232252084888L) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS) ;
        p63.vx_SET(1.6823944E37F) ;
        p63.lat_SET(-812387164) ;
        p63.covariance_SET(new float[] {-2.6367984E38F, 7.533185E37F, -1.8234584E38F, -5.7089086E37F, -1.2030061E37F, 3.1918897E38F, -3.3906885E37F, -2.9856346E38F, -8.507665E37F, -2.7078177E38F, 1.9042623E38F, 2.0531693E38F, -9.675788E37F, 2.8762524E38F, 1.5990862E38F, -3.525205E37F, 1.6911202E38F, -2.6493754E38F, -5.0112104E37F, -1.5857029E38F, 2.48927E38F, -1.2617466E38F, 2.0411852E38F, 1.7255015E38F, 2.9203674E38F, -2.1968542E37F, 1.5364031E38F, -3.1461147E38F, 1.4402669E38F, -2.4492105E38F, 1.320282E38F, 1.7349334E38F, 2.4083743E38F, 2.0626687E38F, 2.3537876E38F, 1.3099973E38F}, 0) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.ay_GET() == -6.9156466E37F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(pack.time_usec_GET() == 2730418579860436113L);
            assert(pack.z_GET() == -1.3439598E38F);
            assert(pack.vz_GET() == -2.5741797E38F);
            assert(pack.az_GET() == -1.6173046E38F);
            assert(pack.vy_GET() == -3.6763092E37F);
            assert(pack.vx_GET() == 5.98005E37F);
            assert(pack.ax_GET() == 7.1969656E37F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {7.8732673E37F, -1.5092073E38F, -9.773057E37F, -1.4137158E37F, -5.0797706E37F, -1.9255913E38F, -1.124865E38F, 3.3576127E37F, 1.4497109E38F, 3.9699932E37F, -6.378134E37F, -3.3573479E38F, -3.014344E38F, 1.5951158E38F, -3.3418963E38F, 1.3619176E38F, -2.364201E38F, -9.821862E37F, 1.8470298E38F, -1.5534494E38F, 3.686974E37F, 3.7808496E37F, -6.1239577E37F, -1.3269702E38F, 2.852472E38F, 1.476602E38F, -3.2432727E38F, 3.3851254E38F, -1.0838695E37F, 2.7751123E38F, 1.9668159E38F, -2.6136288E38F, 6.3869617E37F, 2.3016105E38F, 1.6143963E38F, -9.422406E37F, 2.8497615E38F, -2.0920137E38F, -1.6876861E38F, 1.6321814E37F, -1.6805295E38F, 1.5952123E38F, 2.4939834E37F, 2.5643783E38F, -2.8950483E38F}));
            assert(pack.y_GET() == -2.0890125E38F);
            assert(pack.x_GET() == 8.680287E37F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.time_usec_SET(2730418579860436113L) ;
        p64.vx_SET(5.98005E37F) ;
        p64.vz_SET(-2.5741797E38F) ;
        p64.ax_SET(7.1969656E37F) ;
        p64.x_SET(8.680287E37F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p64.y_SET(-2.0890125E38F) ;
        p64.az_SET(-1.6173046E38F) ;
        p64.ay_SET(-6.9156466E37F) ;
        p64.vy_SET(-3.6763092E37F) ;
        p64.covariance_SET(new float[] {7.8732673E37F, -1.5092073E38F, -9.773057E37F, -1.4137158E37F, -5.0797706E37F, -1.9255913E38F, -1.124865E38F, 3.3576127E37F, 1.4497109E38F, 3.9699932E37F, -6.378134E37F, -3.3573479E38F, -3.014344E38F, 1.5951158E38F, -3.3418963E38F, 1.3619176E38F, -2.364201E38F, -9.821862E37F, 1.8470298E38F, -1.5534494E38F, 3.686974E37F, 3.7808496E37F, -6.1239577E37F, -1.3269702E38F, 2.852472E38F, 1.476602E38F, -3.2432727E38F, 3.3851254E38F, -1.0838695E37F, 2.7751123E38F, 1.9668159E38F, -2.6136288E38F, 6.3869617E37F, 2.3016105E38F, 1.6143963E38F, -9.422406E37F, 2.8497615E38F, -2.0920137E38F, -1.6876861E38F, 1.6321814E37F, -1.6805295E38F, 1.5952123E38F, 2.4939834E37F, 2.5643783E38F, -2.8950483E38F}, 0) ;
        p64.z_SET(-1.3439598E38F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan15_raw_GET() == (char)206);
            assert(pack.chan3_raw_GET() == (char)40478);
            assert(pack.chan4_raw_GET() == (char)19140);
            assert(pack.chan7_raw_GET() == (char)4181);
            assert(pack.chan8_raw_GET() == (char)28981);
            assert(pack.chan17_raw_GET() == (char)20703);
            assert(pack.chan11_raw_GET() == (char)61232);
            assert(pack.chan16_raw_GET() == (char)41304);
            assert(pack.chan5_raw_GET() == (char)36011);
            assert(pack.chan13_raw_GET() == (char)37287);
            assert(pack.time_boot_ms_GET() == 4270182094L);
            assert(pack.chan10_raw_GET() == (char)45533);
            assert(pack.chan1_raw_GET() == (char)56109);
            assert(pack.rssi_GET() == (char)150);
            assert(pack.chancount_GET() == (char)231);
            assert(pack.chan14_raw_GET() == (char)5851);
            assert(pack.chan12_raw_GET() == (char)15874);
            assert(pack.chan9_raw_GET() == (char)56725);
            assert(pack.chan18_raw_GET() == (char)31932);
            assert(pack.chan6_raw_GET() == (char)24678);
            assert(pack.chan2_raw_GET() == (char)10590);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan3_raw_SET((char)40478) ;
        p65.chan14_raw_SET((char)5851) ;
        p65.chan10_raw_SET((char)45533) ;
        p65.chan15_raw_SET((char)206) ;
        p65.chan12_raw_SET((char)15874) ;
        p65.chan4_raw_SET((char)19140) ;
        p65.chan13_raw_SET((char)37287) ;
        p65.chan1_raw_SET((char)56109) ;
        p65.chan17_raw_SET((char)20703) ;
        p65.chan2_raw_SET((char)10590) ;
        p65.rssi_SET((char)150) ;
        p65.chan6_raw_SET((char)24678) ;
        p65.chan11_raw_SET((char)61232) ;
        p65.chan18_raw_SET((char)31932) ;
        p65.chan9_raw_SET((char)56725) ;
        p65.chan16_raw_SET((char)41304) ;
        p65.time_boot_ms_SET(4270182094L) ;
        p65.chan5_raw_SET((char)36011) ;
        p65.chan7_raw_SET((char)4181) ;
        p65.chan8_raw_SET((char)28981) ;
        p65.chancount_SET((char)231) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.req_message_rate_GET() == (char)27948);
            assert(pack.start_stop_GET() == (char)152);
            assert(pack.target_system_GET() == (char)174);
            assert(pack.req_stream_id_GET() == (char)70);
            assert(pack.target_component_GET() == (char)189);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.start_stop_SET((char)152) ;
        p66.req_stream_id_SET((char)70) ;
        p66.target_component_SET((char)189) ;
        p66.req_message_rate_SET((char)27948) ;
        p66.target_system_SET((char)174) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.stream_id_GET() == (char)85);
            assert(pack.message_rate_GET() == (char)35560);
            assert(pack.on_off_GET() == (char)116);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.on_off_SET((char)116) ;
        p67.message_rate_SET((char)35560) ;
        p67.stream_id_SET((char)85) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_GET() == (char)77);
            assert(pack.x_GET() == (short) -25984);
            assert(pack.buttons_GET() == (char)27463);
            assert(pack.z_GET() == (short) -2053);
            assert(pack.r_GET() == (short) -29126);
            assert(pack.y_GET() == (short) -23100);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.r_SET((short) -29126) ;
        p69.target_SET((char)77) ;
        p69.buttons_SET((char)27463) ;
        p69.x_SET((short) -25984) ;
        p69.z_SET((short) -2053) ;
        p69.y_SET((short) -23100) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)37297);
            assert(pack.chan7_raw_GET() == (char)46866);
            assert(pack.chan5_raw_GET() == (char)27632);
            assert(pack.chan1_raw_GET() == (char)11822);
            assert(pack.target_system_GET() == (char)137);
            assert(pack.chan6_raw_GET() == (char)8128);
            assert(pack.chan3_raw_GET() == (char)47398);
            assert(pack.chan8_raw_GET() == (char)28719);
            assert(pack.chan2_raw_GET() == (char)7942);
            assert(pack.target_component_GET() == (char)136);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan4_raw_SET((char)37297) ;
        p70.chan6_raw_SET((char)8128) ;
        p70.chan2_raw_SET((char)7942) ;
        p70.chan5_raw_SET((char)27632) ;
        p70.chan3_raw_SET((char)47398) ;
        p70.chan7_raw_SET((char)46866) ;
        p70.chan1_raw_SET((char)11822) ;
        p70.target_system_SET((char)137) ;
        p70.target_component_SET((char)136) ;
        p70.chan8_raw_SET((char)28719) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)106);
            assert(pack.param2_GET() == 2.2687439E38F);
            assert(pack.autocontinue_GET() == (char)110);
            assert(pack.target_component_GET() == (char)125);
            assert(pack.seq_GET() == (char)11591);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.param4_GET() == -9.925474E37F);
            assert(pack.z_GET() == 1.4880721E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.param1_GET() == 1.898306E38F);
            assert(pack.current_GET() == (char)55);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_LAST);
            assert(pack.x_GET() == -1475820072);
            assert(pack.param3_GET() == -1.6249659E38F);
            assert(pack.y_GET() == -608958296);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.command_SET(MAV_CMD.MAV_CMD_DO_LAST) ;
        p73.param4_SET(-9.925474E37F) ;
        p73.seq_SET((char)11591) ;
        p73.param2_SET(2.2687439E38F) ;
        p73.current_SET((char)55) ;
        p73.y_SET(-608958296) ;
        p73.target_component_SET((char)125) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p73.target_system_SET((char)106) ;
        p73.z_SET(1.4880721E38F) ;
        p73.autocontinue_SET((char)110) ;
        p73.param1_SET(1.898306E38F) ;
        p73.param3_SET(-1.6249659E38F) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p73.x_SET(-1475820072) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.groundspeed_GET() == -1.9861251E38F);
            assert(pack.climb_GET() == -3.0148575E38F);
            assert(pack.airspeed_GET() == -2.518793E38F);
            assert(pack.heading_GET() == (short) -7830);
            assert(pack.alt_GET() == -8.012362E37F);
            assert(pack.throttle_GET() == (char)21);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.alt_SET(-8.012362E37F) ;
        p74.heading_SET((short) -7830) ;
        p74.climb_SET(-3.0148575E38F) ;
        p74.throttle_SET((char)21) ;
        p74.airspeed_SET(-2.518793E38F) ;
        p74.groundspeed_SET(-1.9861251E38F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -603788331);
            assert(pack.param1_GET() == -1.430898E38F);
            assert(pack.current_GET() == (char)27);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_ROI);
            assert(pack.param2_GET() == 2.4677068E38F);
            assert(pack.z_GET() == 2.5464476E38F);
            assert(pack.param3_GET() == 3.0779632E38F);
            assert(pack.target_component_GET() == (char)104);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.param4_GET() == -2.2182833E37F);
            assert(pack.autocontinue_GET() == (char)192);
            assert(pack.target_system_GET() == (char)149);
            assert(pack.y_GET() == 1213918907);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.command_SET(MAV_CMD.MAV_CMD_DO_SET_ROI) ;
        p75.x_SET(-603788331) ;
        p75.autocontinue_SET((char)192) ;
        p75.param2_SET(2.4677068E38F) ;
        p75.target_system_SET((char)149) ;
        p75.current_SET((char)27) ;
        p75.param4_SET(-2.2182833E37F) ;
        p75.target_component_SET((char)104) ;
        p75.param1_SET(-1.430898E38F) ;
        p75.param3_SET(3.0779632E38F) ;
        p75.z_SET(2.5464476E38F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p75.y_SET(1213918907) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param7_GET() == 3.6116048E37F);
            assert(pack.param4_GET() == 1.8133614E38F);
            assert(pack.param6_GET() == 9.615781E37F);
            assert(pack.param1_GET() == -1.2702385E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_TAKEOFF);
            assert(pack.confirmation_GET() == (char)161);
            assert(pack.param2_GET() == -8.063117E37F);
            assert(pack.target_component_GET() == (char)248);
            assert(pack.target_system_GET() == (char)0);
            assert(pack.param5_GET() == -7.0290506E37F);
            assert(pack.param3_GET() == -2.700495E38F);
        });
        GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.confirmation_SET((char)161) ;
        p76.param5_SET(-7.0290506E37F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_NAV_TAKEOFF) ;
        p76.target_system_SET((char)0) ;
        p76.param4_SET(1.8133614E38F) ;
        p76.param2_SET(-8.063117E37F) ;
        p76.param7_SET(3.6116048E37F) ;
        p76.param6_SET(9.615781E37F) ;
        p76.param3_SET(-2.700495E38F) ;
        p76.target_component_SET((char)248) ;
        p76.param1_SET(-1.2702385E38F) ;
        CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.result_param2_TRY(ph) == -333271849);
            assert(pack.target_component_TRY(ph) == (char)136);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_DENIED);
            assert(pack.target_system_TRY(ph) == (char)172);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL);
            assert(pack.progress_TRY(ph) == (char)208);
        });
        GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.command_SET(MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL) ;
        p77.result_param2_SET(-333271849, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_DENIED) ;
        p77.target_system_SET((char)172, PH) ;
        p77.progress_SET((char)208, PH) ;
        p77.target_component_SET((char)136, PH) ;
        CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -4.8794493E37F);
            assert(pack.manual_override_switch_GET() == (char)137);
            assert(pack.pitch_GET() == -2.8599792E38F);
            assert(pack.thrust_GET() == -3.095517E38F);
            assert(pack.time_boot_ms_GET() == 2832184512L);
            assert(pack.mode_switch_GET() == (char)170);
            assert(pack.roll_GET() == -2.1643806E38F);
        });
        GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.mode_switch_SET((char)170) ;
        p81.time_boot_ms_SET(2832184512L) ;
        p81.manual_override_switch_SET((char)137) ;
        p81.thrust_SET(-3.095517E38F) ;
        p81.yaw_SET(-4.8794493E37F) ;
        p81.pitch_SET(-2.8599792E38F) ;
        p81.roll_SET(-2.1643806E38F) ;
        CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)200);
            assert(pack.time_boot_ms_GET() == 2298914820L);
            assert(pack.body_pitch_rate_GET() == -1.1702259E38F);
            assert(pack.body_yaw_rate_GET() == 1.084455E38F);
            assert(pack.thrust_GET() == 2.5280395E38F);
            assert(pack.body_roll_rate_GET() == 2.6265215E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.4159924E38F, -2.7162361E38F, 1.673057E38F, -3.0592104E38F}));
            assert(pack.target_component_GET() == (char)127);
            assert(pack.target_system_GET() == (char)181);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.type_mask_SET((char)200) ;
        p82.body_roll_rate_SET(2.6265215E38F) ;
        p82.q_SET(new float[] {-2.4159924E38F, -2.7162361E38F, 1.673057E38F, -3.0592104E38F}, 0) ;
        p82.time_boot_ms_SET(2298914820L) ;
        p82.target_system_SET((char)181) ;
        p82.target_component_SET((char)127) ;
        p82.body_pitch_rate_SET(-1.1702259E38F) ;
        p82.body_yaw_rate_SET(1.084455E38F) ;
        p82.thrust_SET(2.5280395E38F) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.2281527E38F, -1.5528769E38F, -2.5826215E38F, 3.2677568E37F}));
            assert(pack.body_yaw_rate_GET() == 1.9307765E38F);
            assert(pack.time_boot_ms_GET() == 2354033198L);
            assert(pack.body_roll_rate_GET() == -2.9260994E38F);
            assert(pack.thrust_GET() == -3.7129694E37F);
            assert(pack.type_mask_GET() == (char)141);
            assert(pack.body_pitch_rate_GET() == 7.9508197E37F);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.type_mask_SET((char)141) ;
        p83.time_boot_ms_SET(2354033198L) ;
        p83.body_yaw_rate_SET(1.9307765E38F) ;
        p83.q_SET(new float[] {-2.2281527E38F, -1.5528769E38F, -2.5826215E38F, 3.2677568E37F}, 0) ;
        p83.body_roll_rate_SET(-2.9260994E38F) ;
        p83.thrust_SET(-3.7129694E37F) ;
        p83.body_pitch_rate_SET(7.9508197E37F) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 3.2932508E38F);
            assert(pack.x_GET() == 5.2567437E37F);
            assert(pack.afz_GET() == -2.0589858E38F);
            assert(pack.afx_GET() == 1.5352062E38F);
            assert(pack.z_GET() == 1.6609097E38F);
            assert(pack.target_component_GET() == (char)204);
            assert(pack.target_system_GET() == (char)207);
            assert(pack.type_mask_GET() == (char)20933);
            assert(pack.afy_GET() == 9.511507E37F);
            assert(pack.vz_GET() == -2.9160637E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.yaw_rate_GET() == -1.3883949E38F);
            assert(pack.y_GET() == -3.367255E38F);
            assert(pack.vx_GET() == -2.7126129E38F);
            assert(pack.time_boot_ms_GET() == 131699878L);
            assert(pack.vy_GET() == -8.906998E36F);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.yaw_SET(3.2932508E38F) ;
        p84.x_SET(5.2567437E37F) ;
        p84.z_SET(1.6609097E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p84.afy_SET(9.511507E37F) ;
        p84.afz_SET(-2.0589858E38F) ;
        p84.vy_SET(-8.906998E36F) ;
        p84.target_system_SET((char)207) ;
        p84.time_boot_ms_SET(131699878L) ;
        p84.yaw_rate_SET(-1.3883949E38F) ;
        p84.afx_SET(1.5352062E38F) ;
        p84.type_mask_SET((char)20933) ;
        p84.target_component_SET((char)204) ;
        p84.vx_SET(-2.7126129E38F) ;
        p84.y_SET(-3.367255E38F) ;
        p84.vz_SET(-2.9160637E38F) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3008590644L);
            assert(pack.lat_int_GET() == 93330543);
            assert(pack.target_system_GET() == (char)186);
            assert(pack.afx_GET() == 1.0514015E38F);
            assert(pack.vx_GET() == -2.5640016E38F);
            assert(pack.afy_GET() == 2.6607802E38F);
            assert(pack.vy_GET() == -3.0021009E37F);
            assert(pack.yaw_rate_GET() == -1.1081189E37F);
            assert(pack.type_mask_GET() == (char)36071);
            assert(pack.alt_GET() == -3.2179513E38F);
            assert(pack.target_component_GET() == (char)204);
            assert(pack.yaw_GET() == -1.3755258E38F);
            assert(pack.lon_int_GET() == -2027215610);
            assert(pack.afz_GET() == -2.8083606E38F);
            assert(pack.vz_GET() == -1.8714752E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.type_mask_SET((char)36071) ;
        p86.target_component_SET((char)204) ;
        p86.lat_int_SET(93330543) ;
        p86.afx_SET(1.0514015E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p86.afz_SET(-2.8083606E38F) ;
        p86.vx_SET(-2.5640016E38F) ;
        p86.time_boot_ms_SET(3008590644L) ;
        p86.yaw_SET(-1.3755258E38F) ;
        p86.afy_SET(2.6607802E38F) ;
        p86.yaw_rate_SET(-1.1081189E37F) ;
        p86.vy_SET(-3.0021009E37F) ;
        p86.vz_SET(-1.8714752E38F) ;
        p86.target_system_SET((char)186) ;
        p86.lon_int_SET(-2027215610) ;
        p86.alt_SET(-3.2179513E38F) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == -3.2335921E38F);
            assert(pack.lon_int_GET() == 1157948456);
            assert(pack.lat_int_GET() == -116289994);
            assert(pack.afx_GET() == 3.3339718E38F);
            assert(pack.vy_GET() == 7.5168633E37F);
            assert(pack.type_mask_GET() == (char)31791);
            assert(pack.yaw_GET() == 9.774546E37F);
            assert(pack.time_boot_ms_GET() == 3242209300L);
            assert(pack.vz_GET() == 9.069543E37F);
            assert(pack.afz_GET() == -1.1241034E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.afy_GET() == 1.1523767E38F);
            assert(pack.vx_GET() == 1.5278294E38F);
            assert(pack.alt_GET() == -2.9782094E38F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(3242209300L) ;
        p87.lat_int_SET(-116289994) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p87.yaw_rate_SET(-3.2335921E38F) ;
        p87.afy_SET(1.1523767E38F) ;
        p87.afz_SET(-1.1241034E38F) ;
        p87.lon_int_SET(1157948456) ;
        p87.afx_SET(3.3339718E38F) ;
        p87.yaw_SET(9.774546E37F) ;
        p87.vx_SET(1.5278294E38F) ;
        p87.type_mask_SET((char)31791) ;
        p87.vz_SET(9.069543E37F) ;
        p87.alt_SET(-2.9782094E38F) ;
        p87.vy_SET(7.5168633E37F) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -6.6542423E37F);
            assert(pack.x_GET() == -3.0416187E38F);
            assert(pack.z_GET() == -1.4184426E38F);
            assert(pack.pitch_GET() == 3.330357E36F);
            assert(pack.time_boot_ms_GET() == 3018583018L);
            assert(pack.roll_GET() == 2.9842684E38F);
            assert(pack.y_GET() == -1.8565465E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.pitch_SET(3.330357E36F) ;
        p89.roll_SET(2.9842684E38F) ;
        p89.z_SET(-1.4184426E38F) ;
        p89.time_boot_ms_SET(3018583018L) ;
        p89.y_SET(-1.8565465E38F) ;
        p89.yaw_SET(-6.6542423E37F) ;
        p89.x_SET(-3.0416187E38F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short)13086);
            assert(pack.rollspeed_GET() == -2.9308076E38F);
            assert(pack.lat_GET() == 726184928);
            assert(pack.zacc_GET() == (short)28960);
            assert(pack.vx_GET() == (short) -16209);
            assert(pack.alt_GET() == 1007265329);
            assert(pack.vy_GET() == (short)20050);
            assert(pack.pitch_GET() == 2.6524877E38F);
            assert(pack.yaw_GET() == -7.601093E37F);
            assert(pack.xacc_GET() == (short) -18778);
            assert(pack.time_usec_GET() == 369553237896127264L);
            assert(pack.yawspeed_GET() == -1.7471637E38F);
            assert(pack.roll_GET() == -2.1498517E38F);
            assert(pack.pitchspeed_GET() == 5.4923655E37F);
            assert(pack.lon_GET() == -456500923);
            assert(pack.vz_GET() == (short)24307);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.yawspeed_SET(-1.7471637E38F) ;
        p90.lon_SET(-456500923) ;
        p90.rollspeed_SET(-2.9308076E38F) ;
        p90.pitchspeed_SET(5.4923655E37F) ;
        p90.yaw_SET(-7.601093E37F) ;
        p90.pitch_SET(2.6524877E38F) ;
        p90.vz_SET((short)24307) ;
        p90.xacc_SET((short) -18778) ;
        p90.zacc_SET((short)28960) ;
        p90.roll_SET(-2.1498517E38F) ;
        p90.time_usec_SET(369553237896127264L) ;
        p90.yacc_SET((short)13086) ;
        p90.lat_SET(726184928) ;
        p90.alt_SET(1007265329) ;
        p90.vx_SET((short) -16209) ;
        p90.vy_SET((short)20050) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.throttle_GET() == 2.415857E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_PREFLIGHT);
            assert(pack.nav_mode_GET() == (char)152);
            assert(pack.time_usec_GET() == 1998100200890026983L);
            assert(pack.roll_ailerons_GET() == 3.3796664E38F);
            assert(pack.pitch_elevator_GET() == 1.181046E38F);
            assert(pack.aux2_GET() == 4.274556E37F);
            assert(pack.aux3_GET() == 2.060666E38F);
            assert(pack.aux1_GET() == -2.293305E38F);
            assert(pack.yaw_rudder_GET() == -2.4707423E38F);
            assert(pack.aux4_GET() == 2.8943262E38F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.throttle_SET(2.415857E38F) ;
        p91.aux2_SET(4.274556E37F) ;
        p91.pitch_elevator_SET(1.181046E38F) ;
        p91.time_usec_SET(1998100200890026983L) ;
        p91.aux1_SET(-2.293305E38F) ;
        p91.roll_ailerons_SET(3.3796664E38F) ;
        p91.aux4_SET(2.8943262E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_PREFLIGHT) ;
        p91.yaw_rudder_SET(-2.4707423E38F) ;
        p91.nav_mode_SET((char)152) ;
        p91.aux3_SET(2.060666E38F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan2_raw_GET() == (char)45849);
            assert(pack.chan7_raw_GET() == (char)13738);
            assert(pack.chan6_raw_GET() == (char)55502);
            assert(pack.chan1_raw_GET() == (char)45851);
            assert(pack.chan5_raw_GET() == (char)17144);
            assert(pack.chan10_raw_GET() == (char)6875);
            assert(pack.time_usec_GET() == 5843740097014236687L);
            assert(pack.chan9_raw_GET() == (char)59729);
            assert(pack.chan4_raw_GET() == (char)62456);
            assert(pack.chan8_raw_GET() == (char)22898);
            assert(pack.chan11_raw_GET() == (char)40767);
            assert(pack.chan3_raw_GET() == (char)64860);
            assert(pack.chan12_raw_GET() == (char)16439);
            assert(pack.rssi_GET() == (char)58);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.rssi_SET((char)58) ;
        p92.chan3_raw_SET((char)64860) ;
        p92.chan10_raw_SET((char)6875) ;
        p92.chan9_raw_SET((char)59729) ;
        p92.chan11_raw_SET((char)40767) ;
        p92.chan4_raw_SET((char)62456) ;
        p92.chan6_raw_SET((char)55502) ;
        p92.chan2_raw_SET((char)45849) ;
        p92.chan5_raw_SET((char)17144) ;
        p92.chan12_raw_SET((char)16439) ;
        p92.time_usec_SET(5843740097014236687L) ;
        p92.chan7_raw_SET((char)13738) ;
        p92.chan8_raw_SET((char)22898) ;
        p92.chan1_raw_SET((char)45851) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == 839877903242395306L);
            assert(pack.time_usec_GET() == 1667053576666549594L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.6878652E38F, -3.304193E38F, 2.9460873E38F, 8.165344E37F, -1.9991941E38F, -8.0673634E37F, 3.299434E37F, -2.1500431E38F, 1.1256598E38F, 3.9009313E37F, -1.1255932E38F, -2.0398084E38F, -2.700621E38F, 2.018239E38F, 2.7675723E38F, -1.3240478E38F}));
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_PREFLIGHT);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.controls_SET(new float[] {1.6878652E38F, -3.304193E38F, 2.9460873E38F, 8.165344E37F, -1.9991941E38F, -8.0673634E37F, 3.299434E37F, -2.1500431E38F, 1.1256598E38F, 3.9009313E37F, -1.1255932E38F, -2.0398084E38F, -2.700621E38F, 2.018239E38F, 2.7675723E38F, -1.3240478E38F}, 0) ;
        p93.time_usec_SET(1667053576666549594L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_PREFLIGHT) ;
        p93.flags_SET(839877903242395306L) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_comp_m_y_GET() == -3.0571098E38F);
            assert(pack.flow_y_GET() == (short) -2679);
            assert(pack.flow_x_GET() == (short)28636);
            assert(pack.quality_GET() == (char)76);
            assert(pack.time_usec_GET() == 7832637838684372271L);
            assert(pack.sensor_id_GET() == (char)35);
            assert(pack.flow_rate_x_TRY(ph) == 6.3344597E37F);
            assert(pack.ground_distance_GET() == 1.1346631E38F);
            assert(pack.flow_comp_m_x_GET() == 2.6785331E38F);
            assert(pack.flow_rate_y_TRY(ph) == 1.6936536E36F);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_y_SET((short) -2679) ;
        p100.ground_distance_SET(1.1346631E38F) ;
        p100.flow_rate_y_SET(1.6936536E36F, PH) ;
        p100.time_usec_SET(7832637838684372271L) ;
        p100.flow_comp_m_x_SET(2.6785331E38F) ;
        p100.flow_x_SET((short)28636) ;
        p100.quality_SET((char)76) ;
        p100.sensor_id_SET((char)35) ;
        p100.flow_rate_x_SET(6.3344597E37F, PH) ;
        p100.flow_comp_m_y_SET(-3.0571098E38F) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -2.8323797E38F);
            assert(pack.x_GET() == 1.2363417E38F);
            assert(pack.roll_GET() == 3.219382E38F);
            assert(pack.z_GET() == 1.1286453E38F);
            assert(pack.y_GET() == -1.9360246E38F);
            assert(pack.usec_GET() == 5966154938955506495L);
            assert(pack.pitch_GET() == 3.004491E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.roll_SET(3.219382E38F) ;
        p101.x_SET(1.2363417E38F) ;
        p101.yaw_SET(-2.8323797E38F) ;
        p101.usec_SET(5966154938955506495L) ;
        p101.z_SET(1.1286453E38F) ;
        p101.pitch_SET(3.004491E38F) ;
        p101.y_SET(-1.9360246E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 2.7900178E38F);
            assert(pack.y_GET() == 3.1154887E38F);
            assert(pack.usec_GET() == 9192247506153848575L);
            assert(pack.x_GET() == -2.4976303E38F);
            assert(pack.z_GET() == -7.0957975E37F);
            assert(pack.pitch_GET() == -1.909783E38F);
            assert(pack.yaw_GET() == 1.5357737E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.yaw_SET(1.5357737E38F) ;
        p102.x_SET(-2.4976303E38F) ;
        p102.z_SET(-7.0957975E37F) ;
        p102.pitch_SET(-1.909783E38F) ;
        p102.usec_SET(9192247506153848575L) ;
        p102.y_SET(3.1154887E38F) ;
        p102.roll_SET(2.7900178E38F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.3738691E38F);
            assert(pack.usec_GET() == 162055721828084645L);
            assert(pack.y_GET() == 3.8648154E37F);
            assert(pack.x_GET() == 2.8555607E38F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(162055721828084645L) ;
        p103.z_SET(-1.3738691E38F) ;
        p103.x_SET(2.8555607E38F) ;
        p103.y_SET(3.8648154E37F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 5.9993466E37F);
            assert(pack.yaw_GET() == -7.322639E37F);
            assert(pack.x_GET() == -3.0961295E38F);
            assert(pack.usec_GET() == 8458906576412012378L);
            assert(pack.y_GET() == 2.9077045E37F);
            assert(pack.z_GET() == -1.5247555E38F);
            assert(pack.pitch_GET() == -8.953922E37F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.z_SET(-1.5247555E38F) ;
        p104.x_SET(-3.0961295E38F) ;
        p104.roll_SET(5.9993466E37F) ;
        p104.y_SET(2.9077045E37F) ;
        p104.yaw_SET(-7.322639E37F) ;
        p104.usec_SET(8458906576412012378L) ;
        p104.pitch_SET(-8.953922E37F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.diff_pressure_GET() == 2.0629795E38F);
            assert(pack.temperature_GET() == 2.5745349E38F);
            assert(pack.zacc_GET() == 1.7281473E38F);
            assert(pack.pressure_alt_GET() == 3.6743048E37F);
            assert(pack.xgyro_GET() == 1.6391513E38F);
            assert(pack.ymag_GET() == -8.152848E37F);
            assert(pack.ygyro_GET() == -2.2968928E38F);
            assert(pack.fields_updated_GET() == (char)50456);
            assert(pack.abs_pressure_GET() == 2.6743823E38F);
            assert(pack.xacc_GET() == -1.9294151E38F);
            assert(pack.zgyro_GET() == -1.957903E38F);
            assert(pack.xmag_GET() == 3.0207163E38F);
            assert(pack.zmag_GET() == 1.7495858E38F);
            assert(pack.yacc_GET() == 6.919246E37F);
            assert(pack.time_usec_GET() == 1117029234714051586L);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(1117029234714051586L) ;
        p105.xgyro_SET(1.6391513E38F) ;
        p105.temperature_SET(2.5745349E38F) ;
        p105.ymag_SET(-8.152848E37F) ;
        p105.zacc_SET(1.7281473E38F) ;
        p105.zgyro_SET(-1.957903E38F) ;
        p105.xmag_SET(3.0207163E38F) ;
        p105.zmag_SET(1.7495858E38F) ;
        p105.abs_pressure_SET(2.6743823E38F) ;
        p105.fields_updated_SET((char)50456) ;
        p105.yacc_SET(6.919246E37F) ;
        p105.diff_pressure_SET(2.0629795E38F) ;
        p105.ygyro_SET(-2.2968928E38F) ;
        p105.pressure_alt_SET(3.6743048E37F) ;
        p105.xacc_SET(-1.9294151E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3020047837188688863L);
            assert(pack.quality_GET() == (char)103);
            assert(pack.distance_GET() == -1.3620528E38F);
            assert(pack.integrated_xgyro_GET() == 1.7730724E38F);
            assert(pack.time_delta_distance_us_GET() == 3023382215L);
            assert(pack.integrated_ygyro_GET() == 1.3226862E38F);
            assert(pack.integrated_zgyro_GET() == 4.746083E37F);
            assert(pack.sensor_id_GET() == (char)251);
            assert(pack.integration_time_us_GET() == 3872828945L);
            assert(pack.integrated_x_GET() == -7.8713116E37F);
            assert(pack.integrated_y_GET() == -1.2286245E38F);
            assert(pack.temperature_GET() == (short) -25247);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_zgyro_SET(4.746083E37F) ;
        p106.integration_time_us_SET(3872828945L) ;
        p106.quality_SET((char)103) ;
        p106.temperature_SET((short) -25247) ;
        p106.integrated_xgyro_SET(1.7730724E38F) ;
        p106.integrated_ygyro_SET(1.3226862E38F) ;
        p106.distance_SET(-1.3620528E38F) ;
        p106.integrated_x_SET(-7.8713116E37F) ;
        p106.time_delta_distance_us_SET(3023382215L) ;
        p106.sensor_id_SET((char)251) ;
        p106.time_usec_SET(3020047837188688863L) ;
        p106.integrated_y_SET(-1.2286245E38F) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.pressure_alt_GET() == 2.7659814E38F);
            assert(pack.temperature_GET() == 2.9313532E38F);
            assert(pack.xmag_GET() == -2.0726396E38F);
            assert(pack.zacc_GET() == 3.258863E38F);
            assert(pack.zmag_GET() == -2.5603358E38F);
            assert(pack.diff_pressure_GET() == -2.9972386E38F);
            assert(pack.abs_pressure_GET() == -3.1937418E38F);
            assert(pack.xgyro_GET() == -3.1539407E38F);
            assert(pack.fields_updated_GET() == 743192093L);
            assert(pack.yacc_GET() == 8.473716E37F);
            assert(pack.zgyro_GET() == 2.291729E38F);
            assert(pack.ymag_GET() == 1.6024586E38F);
            assert(pack.ygyro_GET() == 6.9711925E37F);
            assert(pack.xacc_GET() == -2.6583574E38F);
            assert(pack.time_usec_GET() == 4058170582585916591L);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.zgyro_SET(2.291729E38F) ;
        p107.time_usec_SET(4058170582585916591L) ;
        p107.abs_pressure_SET(-3.1937418E38F) ;
        p107.diff_pressure_SET(-2.9972386E38F) ;
        p107.zmag_SET(-2.5603358E38F) ;
        p107.zacc_SET(3.258863E38F) ;
        p107.fields_updated_SET(743192093L) ;
        p107.yacc_SET(8.473716E37F) ;
        p107.ymag_SET(1.6024586E38F) ;
        p107.xgyro_SET(-3.1539407E38F) ;
        p107.ygyro_SET(6.9711925E37F) ;
        p107.xacc_SET(-2.6583574E38F) ;
        p107.pressure_alt_SET(2.7659814E38F) ;
        p107.xmag_SET(-2.0726396E38F) ;
        p107.temperature_SET(2.9313532E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == -1.2117783E38F);
            assert(pack.q1_GET() == -1.0762095E38F);
            assert(pack.vd_GET() == 2.4505436E38F);
            assert(pack.vn_GET() == -2.0489613E37F);
            assert(pack.xgyro_GET() == 6.383984E36F);
            assert(pack.zacc_GET() == -3.0502042E38F);
            assert(pack.roll_GET() == 2.3337715E38F);
            assert(pack.std_dev_vert_GET() == -1.1386072E38F);
            assert(pack.ve_GET() == -2.6281995E38F);
            assert(pack.ygyro_GET() == -1.8393505E38F);
            assert(pack.lon_GET() == 9.900265E37F);
            assert(pack.q3_GET() == 4.9751133E37F);
            assert(pack.yacc_GET() == -3.2011695E38F);
            assert(pack.std_dev_horz_GET() == 8.548929E37F);
            assert(pack.q2_GET() == -2.7422355E38F);
            assert(pack.pitch_GET() == 2.7578195E38F);
            assert(pack.yaw_GET() == 1.0092684E38F);
            assert(pack.lat_GET() == 1.1814683E38F);
            assert(pack.zgyro_GET() == -1.3127339E38F);
            assert(pack.q4_GET() == -2.066717E38F);
            assert(pack.alt_GET() == 1.0864499E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.yacc_SET(-3.2011695E38F) ;
        p108.lon_SET(9.900265E37F) ;
        p108.q1_SET(-1.0762095E38F) ;
        p108.q4_SET(-2.066717E38F) ;
        p108.roll_SET(2.3337715E38F) ;
        p108.zgyro_SET(-1.3127339E38F) ;
        p108.std_dev_vert_SET(-1.1386072E38F) ;
        p108.yaw_SET(1.0092684E38F) ;
        p108.pitch_SET(2.7578195E38F) ;
        p108.xgyro_SET(6.383984E36F) ;
        p108.vd_SET(2.4505436E38F) ;
        p108.xacc_SET(-1.2117783E38F) ;
        p108.alt_SET(1.0864499E38F) ;
        p108.vn_SET(-2.0489613E37F) ;
        p108.q2_SET(-2.7422355E38F) ;
        p108.lat_SET(1.1814683E38F) ;
        p108.zacc_SET(-3.0502042E38F) ;
        p108.ve_SET(-2.6281995E38F) ;
        p108.q3_SET(4.9751133E37F) ;
        p108.std_dev_horz_SET(8.548929E37F) ;
        p108.ygyro_SET(-1.8393505E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)20);
            assert(pack.remrssi_GET() == (char)159);
            assert(pack.noise_GET() == (char)69);
            assert(pack.txbuf_GET() == (char)16);
            assert(pack.remnoise_GET() == (char)237);
            assert(pack.fixed__GET() == (char)5862);
            assert(pack.rxerrors_GET() == (char)58785);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.remnoise_SET((char)237) ;
        p109.remrssi_SET((char)159) ;
        p109.rxerrors_SET((char)58785) ;
        p109.txbuf_SET((char)16) ;
        p109.fixed__SET((char)5862) ;
        p109.noise_SET((char)69) ;
        p109.rssi_SET((char)20) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)199);
            assert(pack.target_system_GET() == (char)185);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)50, (char)106, (char)91, (char)54, (char)159, (char)48, (char)0, (char)83, (char)94, (char)183, (char)126, (char)251, (char)100, (char)6, (char)129, (char)56, (char)5, (char)79, (char)243, (char)179, (char)219, (char)1, (char)17, (char)177, (char)92, (char)141, (char)101, (char)7, (char)16, (char)123, (char)222, (char)33, (char)75, (char)54, (char)177, (char)230, (char)201, (char)236, (char)159, (char)73, (char)87, (char)252, (char)41, (char)62, (char)161, (char)11, (char)64, (char)188, (char)63, (char)37, (char)251, (char)222, (char)132, (char)160, (char)59, (char)111, (char)99, (char)250, (char)90, (char)211, (char)225, (char)114, (char)153, (char)19, (char)82, (char)76, (char)250, (char)218, (char)195, (char)84, (char)228, (char)250, (char)239, (char)215, (char)125, (char)141, (char)23, (char)45, (char)154, (char)157, (char)74, (char)26, (char)255, (char)115, (char)147, (char)86, (char)65, (char)196, (char)16, (char)115, (char)91, (char)67, (char)209, (char)18, (char)244, (char)13, (char)178, (char)179, (char)154, (char)168, (char)185, (char)64, (char)213, (char)137, (char)78, (char)146, (char)134, (char)158, (char)148, (char)24, (char)144, (char)29, (char)138, (char)49, (char)28, (char)24, (char)144, (char)143, (char)121, (char)24, (char)83, (char)186, (char)1, (char)141, (char)91, (char)159, (char)15, (char)36, (char)70, (char)54, (char)246, (char)87, (char)254, (char)31, (char)70, (char)130, (char)5, (char)113, (char)42, (char)164, (char)86, (char)48, (char)16, (char)250, (char)123, (char)145, (char)124, (char)248, (char)194, (char)126, (char)17, (char)182, (char)93, (char)136, (char)117, (char)178, (char)142, (char)208, (char)211, (char)101, (char)161, (char)181, (char)48, (char)8, (char)114, (char)156, (char)56, (char)186, (char)95, (char)203, (char)128, (char)147, (char)152, (char)68, (char)41, (char)88, (char)123, (char)43, (char)80, (char)244, (char)30, (char)136, (char)153, (char)58, (char)162, (char)228, (char)154, (char)126, (char)141, (char)69, (char)151, (char)30, (char)89, (char)41, (char)141, (char)175, (char)221, (char)237, (char)22, (char)113, (char)185, (char)142, (char)211, (char)233, (char)236, (char)95, (char)29, (char)186, (char)186, (char)172, (char)210, (char)174, (char)101, (char)113, (char)121, (char)191, (char)32, (char)143, (char)64, (char)86, (char)246, (char)137, (char)19, (char)137, (char)51, (char)146, (char)94, (char)221, (char)10, (char)23, (char)90, (char)66, (char)92, (char)54, (char)40, (char)43, (char)47, (char)231, (char)87, (char)45, (char)66, (char)86, (char)135, (char)154, (char)126, (char)144, (char)132, (char)21, (char)74, (char)108, (char)85}));
            assert(pack.target_component_GET() == (char)206);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_system_SET((char)185) ;
        p110.payload_SET(new char[] {(char)50, (char)106, (char)91, (char)54, (char)159, (char)48, (char)0, (char)83, (char)94, (char)183, (char)126, (char)251, (char)100, (char)6, (char)129, (char)56, (char)5, (char)79, (char)243, (char)179, (char)219, (char)1, (char)17, (char)177, (char)92, (char)141, (char)101, (char)7, (char)16, (char)123, (char)222, (char)33, (char)75, (char)54, (char)177, (char)230, (char)201, (char)236, (char)159, (char)73, (char)87, (char)252, (char)41, (char)62, (char)161, (char)11, (char)64, (char)188, (char)63, (char)37, (char)251, (char)222, (char)132, (char)160, (char)59, (char)111, (char)99, (char)250, (char)90, (char)211, (char)225, (char)114, (char)153, (char)19, (char)82, (char)76, (char)250, (char)218, (char)195, (char)84, (char)228, (char)250, (char)239, (char)215, (char)125, (char)141, (char)23, (char)45, (char)154, (char)157, (char)74, (char)26, (char)255, (char)115, (char)147, (char)86, (char)65, (char)196, (char)16, (char)115, (char)91, (char)67, (char)209, (char)18, (char)244, (char)13, (char)178, (char)179, (char)154, (char)168, (char)185, (char)64, (char)213, (char)137, (char)78, (char)146, (char)134, (char)158, (char)148, (char)24, (char)144, (char)29, (char)138, (char)49, (char)28, (char)24, (char)144, (char)143, (char)121, (char)24, (char)83, (char)186, (char)1, (char)141, (char)91, (char)159, (char)15, (char)36, (char)70, (char)54, (char)246, (char)87, (char)254, (char)31, (char)70, (char)130, (char)5, (char)113, (char)42, (char)164, (char)86, (char)48, (char)16, (char)250, (char)123, (char)145, (char)124, (char)248, (char)194, (char)126, (char)17, (char)182, (char)93, (char)136, (char)117, (char)178, (char)142, (char)208, (char)211, (char)101, (char)161, (char)181, (char)48, (char)8, (char)114, (char)156, (char)56, (char)186, (char)95, (char)203, (char)128, (char)147, (char)152, (char)68, (char)41, (char)88, (char)123, (char)43, (char)80, (char)244, (char)30, (char)136, (char)153, (char)58, (char)162, (char)228, (char)154, (char)126, (char)141, (char)69, (char)151, (char)30, (char)89, (char)41, (char)141, (char)175, (char)221, (char)237, (char)22, (char)113, (char)185, (char)142, (char)211, (char)233, (char)236, (char)95, (char)29, (char)186, (char)186, (char)172, (char)210, (char)174, (char)101, (char)113, (char)121, (char)191, (char)32, (char)143, (char)64, (char)86, (char)246, (char)137, (char)19, (char)137, (char)51, (char)146, (char)94, (char)221, (char)10, (char)23, (char)90, (char)66, (char)92, (char)54, (char)40, (char)43, (char)47, (char)231, (char)87, (char)45, (char)66, (char)86, (char)135, (char)154, (char)126, (char)144, (char)132, (char)21, (char)74, (char)108, (char)85}, 0) ;
        p110.target_network_SET((char)199) ;
        p110.target_component_SET((char)206) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == 2109633802772742322L);
            assert(pack.tc1_GET() == 2358500994516534368L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(2109633802772742322L) ;
        p111.tc1_SET(2358500994516534368L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 8417016417794540156L);
            assert(pack.seq_GET() == 2418841936L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(8417016417794540156L) ;
        p112.seq_SET(2418841936L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)223);
            assert(pack.cog_GET() == (char)19361);
            assert(pack.time_usec_GET() == 5790010518372654374L);
            assert(pack.vn_GET() == (short)21234);
            assert(pack.alt_GET() == 1976844797);
            assert(pack.lat_GET() == -1487267616);
            assert(pack.fix_type_GET() == (char)177);
            assert(pack.epv_GET() == (char)64355);
            assert(pack.vd_GET() == (short)25705);
            assert(pack.lon_GET() == 1643536249);
            assert(pack.eph_GET() == (char)10373);
            assert(pack.ve_GET() == (short)16396);
            assert(pack.vel_GET() == (char)35777);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.eph_SET((char)10373) ;
        p113.cog_SET((char)19361) ;
        p113.lat_SET(-1487267616) ;
        p113.vd_SET((short)25705) ;
        p113.epv_SET((char)64355) ;
        p113.ve_SET((short)16396) ;
        p113.vel_SET((char)35777) ;
        p113.satellites_visible_SET((char)223) ;
        p113.vn_SET((short)21234) ;
        p113.lon_SET(1643536249) ;
        p113.alt_SET(1976844797) ;
        p113.time_usec_SET(5790010518372654374L) ;
        p113.fix_type_SET((char)177) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.distance_GET() == 2.9190854E38F);
            assert(pack.temperature_GET() == (short) -30050);
            assert(pack.time_usec_GET() == 3434038620429356855L);
            assert(pack.sensor_id_GET() == (char)45);
            assert(pack.integrated_ygyro_GET() == 2.5156149E38F);
            assert(pack.integrated_x_GET() == 5.80053E37F);
            assert(pack.quality_GET() == (char)199);
            assert(pack.integrated_y_GET() == 2.5193063E38F);
            assert(pack.integrated_zgyro_GET() == 3.9817113E37F);
            assert(pack.time_delta_distance_us_GET() == 2769764444L);
            assert(pack.integrated_xgyro_GET() == -2.349024E38F);
            assert(pack.integration_time_us_GET() == 734693946L);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.quality_SET((char)199) ;
        p114.integrated_zgyro_SET(3.9817113E37F) ;
        p114.sensor_id_SET((char)45) ;
        p114.distance_SET(2.9190854E38F) ;
        p114.integration_time_us_SET(734693946L) ;
        p114.temperature_SET((short) -30050) ;
        p114.integrated_y_SET(2.5193063E38F) ;
        p114.integrated_x_SET(5.80053E37F) ;
        p114.time_usec_SET(3434038620429356855L) ;
        p114.time_delta_distance_us_SET(2769764444L) ;
        p114.integrated_xgyro_SET(-2.349024E38F) ;
        p114.integrated_ygyro_SET(2.5156149E38F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -769845058);
            assert(pack.time_usec_GET() == 5961664721094892402L);
            assert(pack.lon_GET() == -1178800387);
            assert(pack.yawspeed_GET() == 2.4431562E38F);
            assert(pack.pitchspeed_GET() == -8.288653E37F);
            assert(pack.true_airspeed_GET() == (char)57785);
            assert(pack.vz_GET() == (short)30947);
            assert(pack.yacc_GET() == (short) -24976);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {-3.2889088E37F, -2.2113402E38F, -2.5122178E38F, -2.9900933E38F}));
            assert(pack.alt_GET() == -309011692);
            assert(pack.rollspeed_GET() == 6.9223463E37F);
            assert(pack.ind_airspeed_GET() == (char)49374);
            assert(pack.vy_GET() == (short) -5002);
            assert(pack.xacc_GET() == (short)19755);
            assert(pack.zacc_GET() == (short)14048);
            assert(pack.vx_GET() == (short) -27002);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.vx_SET((short) -27002) ;
        p115.lon_SET(-1178800387) ;
        p115.vy_SET((short) -5002) ;
        p115.lat_SET(-769845058) ;
        p115.pitchspeed_SET(-8.288653E37F) ;
        p115.vz_SET((short)30947) ;
        p115.yacc_SET((short) -24976) ;
        p115.true_airspeed_SET((char)57785) ;
        p115.xacc_SET((short)19755) ;
        p115.zacc_SET((short)14048) ;
        p115.rollspeed_SET(6.9223463E37F) ;
        p115.alt_SET(-309011692) ;
        p115.ind_airspeed_SET((char)49374) ;
        p115.attitude_quaternion_SET(new float[] {-3.2889088E37F, -2.2113402E38F, -2.5122178E38F, -2.9900933E38F}, 0) ;
        p115.time_usec_SET(5961664721094892402L) ;
        p115.yawspeed_SET(2.4431562E38F) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == (short)15991);
            assert(pack.time_boot_ms_GET() == 1263286298L);
            assert(pack.ygyro_GET() == (short) -23554);
            assert(pack.zacc_GET() == (short)238);
            assert(pack.zgyro_GET() == (short)22984);
            assert(pack.xgyro_GET() == (short)8893);
            assert(pack.xmag_GET() == (short)30812);
            assert(pack.xacc_GET() == (short)24081);
            assert(pack.zmag_GET() == (short) -31599);
            assert(pack.yacc_GET() == (short) -29016);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.zmag_SET((short) -31599) ;
        p116.zacc_SET((short)238) ;
        p116.xgyro_SET((short)8893) ;
        p116.time_boot_ms_SET(1263286298L) ;
        p116.xacc_SET((short)24081) ;
        p116.ymag_SET((short)15991) ;
        p116.xmag_SET((short)30812) ;
        p116.ygyro_SET((short) -23554) ;
        p116.yacc_SET((short) -29016) ;
        p116.zgyro_SET((short)22984) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)232);
            assert(pack.end_GET() == (char)2214);
            assert(pack.target_system_GET() == (char)198);
            assert(pack.start_GET() == (char)61096);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.end_SET((char)2214) ;
        p117.target_component_SET((char)232) ;
        p117.target_system_SET((char)198) ;
        p117.start_SET((char)61096) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.time_utc_GET() == 1333772601L);
            assert(pack.size_GET() == 1499774650L);
            assert(pack.id_GET() == (char)27513);
            assert(pack.num_logs_GET() == (char)61478);
            assert(pack.last_log_num_GET() == (char)17473);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.time_utc_SET(1333772601L) ;
        p118.num_logs_SET((char)61478) ;
        p118.size_SET(1499774650L) ;
        p118.last_log_num_SET((char)17473) ;
        p118.id_SET((char)27513) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == 1679010252L);
            assert(pack.ofs_GET() == 2160184549L);
            assert(pack.id_GET() == (char)53406);
            assert(pack.target_component_GET() == (char)225);
            assert(pack.target_system_GET() == (char)41);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.count_SET(1679010252L) ;
        p119.target_system_SET((char)41) ;
        p119.id_SET((char)53406) ;
        p119.ofs_SET(2160184549L) ;
        p119.target_component_SET((char)225) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)62);
            assert(pack.ofs_GET() == 1293406439L);
            assert(pack.id_GET() == (char)48864);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)113, (char)180, (char)111, (char)140, (char)166, (char)250, (char)86, (char)196, (char)49, (char)162, (char)22, (char)137, (char)18, (char)98, (char)80, (char)1, (char)99, (char)228, (char)8, (char)174, (char)151, (char)45, (char)249, (char)23, (char)85, (char)185, (char)1, (char)93, (char)249, (char)61, (char)204, (char)39, (char)90, (char)124, (char)227, (char)214, (char)191, (char)175, (char)136, (char)11, (char)127, (char)206, (char)205, (char)224, (char)1, (char)135, (char)71, (char)29, (char)49, (char)32, (char)47, (char)106, (char)163, (char)0, (char)11, (char)111, (char)25, (char)101, (char)115, (char)59, (char)48, (char)217, (char)198, (char)47, (char)95, (char)122, (char)4, (char)67, (char)63, (char)106, (char)161, (char)168, (char)1, (char)215, (char)182, (char)190, (char)169, (char)250, (char)244, (char)171, (char)16, (char)149, (char)244, (char)230, (char)33, (char)246, (char)172, (char)230, (char)103, (char)164}));
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.count_SET((char)62) ;
        p120.data__SET(new char[] {(char)113, (char)180, (char)111, (char)140, (char)166, (char)250, (char)86, (char)196, (char)49, (char)162, (char)22, (char)137, (char)18, (char)98, (char)80, (char)1, (char)99, (char)228, (char)8, (char)174, (char)151, (char)45, (char)249, (char)23, (char)85, (char)185, (char)1, (char)93, (char)249, (char)61, (char)204, (char)39, (char)90, (char)124, (char)227, (char)214, (char)191, (char)175, (char)136, (char)11, (char)127, (char)206, (char)205, (char)224, (char)1, (char)135, (char)71, (char)29, (char)49, (char)32, (char)47, (char)106, (char)163, (char)0, (char)11, (char)111, (char)25, (char)101, (char)115, (char)59, (char)48, (char)217, (char)198, (char)47, (char)95, (char)122, (char)4, (char)67, (char)63, (char)106, (char)161, (char)168, (char)1, (char)215, (char)182, (char)190, (char)169, (char)250, (char)244, (char)171, (char)16, (char)149, (char)244, (char)230, (char)33, (char)246, (char)172, (char)230, (char)103, (char)164}, 0) ;
        p120.id_SET((char)48864) ;
        p120.ofs_SET(1293406439L) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)159);
            assert(pack.target_component_GET() == (char)210);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)210) ;
        p121.target_system_SET((char)159) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)189);
            assert(pack.target_component_GET() == (char)75);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)189) ;
        p122.target_component_SET((char)75) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)21);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)118, (char)151, (char)12, (char)93, (char)28, (char)94, (char)55, (char)236, (char)129, (char)125, (char)57, (char)5, (char)8, (char)49, (char)131, (char)53, (char)88, (char)52, (char)235, (char)253, (char)129, (char)231, (char)96, (char)5, (char)25, (char)12, (char)113, (char)29, (char)81, (char)146, (char)224, (char)97, (char)158, (char)111, (char)211, (char)78, (char)21, (char)150, (char)139, (char)206, (char)127, (char)12, (char)218, (char)153, (char)214, (char)230, (char)251, (char)205, (char)180, (char)88, (char)178, (char)239, (char)107, (char)160, (char)156, (char)253, (char)255, (char)242, (char)126, (char)105, (char)239, (char)90, (char)138, (char)229, (char)241, (char)246, (char)250, (char)250, (char)26, (char)151, (char)192, (char)97, (char)15, (char)100, (char)67, (char)191, (char)104, (char)241, (char)174, (char)39, (char)68, (char)90, (char)202, (char)52, (char)152, (char)157, (char)94, (char)51, (char)207, (char)226, (char)90, (char)35, (char)140, (char)202, (char)77, (char)201, (char)212, (char)101, (char)249, (char)186, (char)122, (char)37, (char)224, (char)177, (char)46, (char)247, (char)213, (char)242, (char)232, (char)8}));
            assert(pack.len_GET() == (char)243);
            assert(pack.target_component_GET() == (char)200);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.data__SET(new char[] {(char)118, (char)151, (char)12, (char)93, (char)28, (char)94, (char)55, (char)236, (char)129, (char)125, (char)57, (char)5, (char)8, (char)49, (char)131, (char)53, (char)88, (char)52, (char)235, (char)253, (char)129, (char)231, (char)96, (char)5, (char)25, (char)12, (char)113, (char)29, (char)81, (char)146, (char)224, (char)97, (char)158, (char)111, (char)211, (char)78, (char)21, (char)150, (char)139, (char)206, (char)127, (char)12, (char)218, (char)153, (char)214, (char)230, (char)251, (char)205, (char)180, (char)88, (char)178, (char)239, (char)107, (char)160, (char)156, (char)253, (char)255, (char)242, (char)126, (char)105, (char)239, (char)90, (char)138, (char)229, (char)241, (char)246, (char)250, (char)250, (char)26, (char)151, (char)192, (char)97, (char)15, (char)100, (char)67, (char)191, (char)104, (char)241, (char)174, (char)39, (char)68, (char)90, (char)202, (char)52, (char)152, (char)157, (char)94, (char)51, (char)207, (char)226, (char)90, (char)35, (char)140, (char)202, (char)77, (char)201, (char)212, (char)101, (char)249, (char)186, (char)122, (char)37, (char)224, (char)177, (char)46, (char)247, (char)213, (char)242, (char)232, (char)8}, 0) ;
        p123.len_SET((char)243) ;
        p123.target_component_SET((char)200) ;
        p123.target_system_SET((char)21) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.vel_GET() == (char)57009);
            assert(pack.dgps_age_GET() == 3545215506L);
            assert(pack.eph_GET() == (char)23427);
            assert(pack.time_usec_GET() == 7887700064036277879L);
            assert(pack.satellites_visible_GET() == (char)235);
            assert(pack.lon_GET() == -330347646);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
            assert(pack.lat_GET() == 1595361706);
            assert(pack.epv_GET() == (char)27943);
            assert(pack.dgps_numch_GET() == (char)94);
            assert(pack.alt_GET() == -1845502501);
            assert(pack.cog_GET() == (char)45428);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.vel_SET((char)57009) ;
        p124.lon_SET(-330347646) ;
        p124.dgps_age_SET(3545215506L) ;
        p124.eph_SET((char)23427) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED) ;
        p124.alt_SET(-1845502501) ;
        p124.time_usec_SET(7887700064036277879L) ;
        p124.cog_SET((char)45428) ;
        p124.epv_SET((char)27943) ;
        p124.lat_SET(1595361706) ;
        p124.dgps_numch_SET((char)94) ;
        p124.satellites_visible_SET((char)235) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID));
            assert(pack.Vservo_GET() == (char)52646);
            assert(pack.Vcc_GET() == (char)63339);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)63339) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID)) ;
        p125.Vservo_SET((char)52646) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
            assert(pack.timeout_GET() == (char)35123);
            assert(pack.baudrate_GET() == 373412142L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)91, (char)251, (char)159, (char)156, (char)45, (char)176, (char)188, (char)215, (char)255, (char)64, (char)245, (char)85, (char)41, (char)29, (char)179, (char)80, (char)133, (char)176, (char)156, (char)54, (char)204, (char)180, (char)107, (char)174, (char)79, (char)71, (char)76, (char)112, (char)102, (char)14, (char)179, (char)57, (char)222, (char)14, (char)108, (char)149, (char)195, (char)89, (char)230, (char)158, (char)148, (char)167, (char)121, (char)157, (char)9, (char)241, (char)220, (char)132, (char)202, (char)138, (char)196, (char)0, (char)18, (char)40, (char)7, (char)107, (char)60, (char)77, (char)37, (char)28, (char)173, (char)74, (char)32, (char)36, (char)171, (char)63, (char)249, (char)55, (char)89, (char)201}));
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI));
            assert(pack.count_GET() == (char)32);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.baudrate_SET(373412142L) ;
        p126.data__SET(new char[] {(char)91, (char)251, (char)159, (char)156, (char)45, (char)176, (char)188, (char)215, (char)255, (char)64, (char)245, (char)85, (char)41, (char)29, (char)179, (char)80, (char)133, (char)176, (char)156, (char)54, (char)204, (char)180, (char)107, (char)174, (char)79, (char)71, (char)76, (char)112, (char)102, (char)14, (char)179, (char)57, (char)222, (char)14, (char)108, (char)149, (char)195, (char)89, (char)230, (char)158, (char)148, (char)167, (char)121, (char)157, (char)9, (char)241, (char)220, (char)132, (char)202, (char)138, (char)196, (char)0, (char)18, (char)40, (char)7, (char)107, (char)60, (char)77, (char)37, (char)28, (char)173, (char)74, (char)32, (char)36, (char)171, (char)63, (char)249, (char)55, (char)89, (char)201}, 0) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI)) ;
        p126.timeout_SET((char)35123) ;
        p126.count_SET((char)32) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.rtk_health_GET() == (char)190);
            assert(pack.rtk_receiver_id_GET() == (char)176);
            assert(pack.tow_GET() == 1003747306L);
            assert(pack.baseline_c_mm_GET() == 1424267869);
            assert(pack.baseline_coords_type_GET() == (char)136);
            assert(pack.time_last_baseline_ms_GET() == 4277678424L);
            assert(pack.baseline_b_mm_GET() == 2041978762);
            assert(pack.baseline_a_mm_GET() == -1333327187);
            assert(pack.nsats_GET() == (char)39);
            assert(pack.accuracy_GET() == 83783924L);
            assert(pack.rtk_rate_GET() == (char)196);
            assert(pack.iar_num_hypotheses_GET() == 207310774);
            assert(pack.wn_GET() == (char)56907);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.wn_SET((char)56907) ;
        p127.baseline_c_mm_SET(1424267869) ;
        p127.rtk_receiver_id_SET((char)176) ;
        p127.baseline_a_mm_SET(-1333327187) ;
        p127.iar_num_hypotheses_SET(207310774) ;
        p127.tow_SET(1003747306L) ;
        p127.nsats_SET((char)39) ;
        p127.time_last_baseline_ms_SET(4277678424L) ;
        p127.rtk_health_SET((char)190) ;
        p127.baseline_coords_type_SET((char)136) ;
        p127.accuracy_SET(83783924L) ;
        p127.rtk_rate_SET((char)196) ;
        p127.baseline_b_mm_SET(2041978762) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_coords_type_GET() == (char)156);
            assert(pack.accuracy_GET() == 74788142L);
            assert(pack.baseline_c_mm_GET() == -467409943);
            assert(pack.rtk_health_GET() == (char)108);
            assert(pack.baseline_b_mm_GET() == 1293596308);
            assert(pack.time_last_baseline_ms_GET() == 1690619844L);
            assert(pack.baseline_a_mm_GET() == -2084187203);
            assert(pack.rtk_receiver_id_GET() == (char)12);
            assert(pack.rtk_rate_GET() == (char)15);
            assert(pack.nsats_GET() == (char)53);
            assert(pack.iar_num_hypotheses_GET() == 1591582434);
            assert(pack.wn_GET() == (char)6574);
            assert(pack.tow_GET() == 3511589190L);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.iar_num_hypotheses_SET(1591582434) ;
        p128.baseline_coords_type_SET((char)156) ;
        p128.baseline_c_mm_SET(-467409943) ;
        p128.rtk_health_SET((char)108) ;
        p128.tow_SET(3511589190L) ;
        p128.rtk_rate_SET((char)15) ;
        p128.wn_SET((char)6574) ;
        p128.baseline_a_mm_SET(-2084187203) ;
        p128.time_last_baseline_ms_SET(1690619844L) ;
        p128.rtk_receiver_id_SET((char)12) ;
        p128.accuracy_SET(74788142L) ;
        p128.nsats_SET((char)53) ;
        p128.baseline_b_mm_SET(1293596308) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short) -9836);
            assert(pack.zacc_GET() == (short)31117);
            assert(pack.zgyro_GET() == (short) -16081);
            assert(pack.yacc_GET() == (short)14002);
            assert(pack.time_boot_ms_GET() == 2377377912L);
            assert(pack.xmag_GET() == (short) -25717);
            assert(pack.xacc_GET() == (short)22755);
            assert(pack.xgyro_GET() == (short) -11549);
            assert(pack.zmag_GET() == (short)14064);
            assert(pack.ymag_GET() == (short)15612);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.yacc_SET((short)14002) ;
        p129.ymag_SET((short)15612) ;
        p129.zacc_SET((short)31117) ;
        p129.xmag_SET((short) -25717) ;
        p129.zmag_SET((short)14064) ;
        p129.zgyro_SET((short) -16081) ;
        p129.xgyro_SET((short) -11549) ;
        p129.ygyro_SET((short) -9836) ;
        p129.xacc_SET((short)22755) ;
        p129.time_boot_ms_SET(2377377912L) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.jpg_quality_GET() == (char)92);
            assert(pack.payload_GET() == (char)143);
            assert(pack.type_GET() == (char)199);
            assert(pack.width_GET() == (char)39402);
            assert(pack.size_GET() == 16137179L);
            assert(pack.packets_GET() == (char)10508);
            assert(pack.height_GET() == (char)38942);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.jpg_quality_SET((char)92) ;
        p130.payload_SET((char)143) ;
        p130.height_SET((char)38942) ;
        p130.size_SET(16137179L) ;
        p130.type_SET((char)199) ;
        p130.width_SET((char)39402) ;
        p130.packets_SET((char)10508) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)59047);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)151, (char)117, (char)169, (char)22, (char)98, (char)124, (char)155, (char)102, (char)148, (char)65, (char)96, (char)52, (char)205, (char)190, (char)252, (char)160, (char)49, (char)29, (char)176, (char)210, (char)222, (char)131, (char)91, (char)230, (char)184, (char)74, (char)168, (char)95, (char)230, (char)94, (char)71, (char)34, (char)88, (char)37, (char)25, (char)148, (char)21, (char)38, (char)125, (char)84, (char)49, (char)135, (char)103, (char)182, (char)193, (char)164, (char)193, (char)65, (char)226, (char)40, (char)113, (char)186, (char)235, (char)25, (char)79, (char)98, (char)79, (char)221, (char)221, (char)243, (char)134, (char)80, (char)239, (char)82, (char)45, (char)26, (char)214, (char)202, (char)8, (char)111, (char)28, (char)10, (char)179, (char)198, (char)225, (char)133, (char)153, (char)35, (char)8, (char)80, (char)51, (char)13, (char)212, (char)118, (char)202, (char)70, (char)89, (char)25, (char)81, (char)208, (char)203, (char)46, (char)102, (char)106, (char)23, (char)188, (char)114, (char)79, (char)43, (char)115, (char)198, (char)153, (char)174, (char)134, (char)220, (char)15, (char)186, (char)104, (char)222, (char)155, (char)21, (char)227, (char)110, (char)233, (char)177, (char)96, (char)72, (char)85, (char)213, (char)132, (char)248, (char)176, (char)151, (char)36, (char)130, (char)223, (char)27, (char)102, (char)79, (char)150, (char)96, (char)211, (char)84, (char)243, (char)203, (char)166, (char)120, (char)210, (char)140, (char)117, (char)160, (char)176, (char)154, (char)225, (char)150, (char)102, (char)127, (char)69, (char)250, (char)17, (char)4, (char)87, (char)194, (char)240, (char)113, (char)88, (char)115, (char)196, (char)18, (char)2, (char)218, (char)94, (char)74, (char)101, (char)93, (char)114, (char)72, (char)99, (char)234, (char)106, (char)116, (char)170, (char)227, (char)95, (char)172, (char)88, (char)5, (char)38, (char)238, (char)102, (char)220, (char)24, (char)101, (char)206, (char)35, (char)158, (char)222, (char)85, (char)198, (char)11, (char)10, (char)155, (char)220, (char)200, (char)69, (char)20, (char)58, (char)253, (char)183, (char)92, (char)28, (char)6, (char)129, (char)130, (char)118, (char)152, (char)114, (char)212, (char)161, (char)177, (char)183, (char)218, (char)107, (char)77, (char)204, (char)149, (char)101, (char)35, (char)58, (char)38, (char)89, (char)221, (char)1, (char)46, (char)34, (char)142, (char)134, (char)100, (char)207, (char)243, (char)236, (char)156, (char)70, (char)82, (char)212, (char)184, (char)56, (char)120, (char)2, (char)230, (char)94, (char)229, (char)92, (char)171, (char)100, (char)187, (char)35, (char)188, (char)237, (char)194, (char)105, (char)218, (char)202}));
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.data__SET(new char[] {(char)151, (char)117, (char)169, (char)22, (char)98, (char)124, (char)155, (char)102, (char)148, (char)65, (char)96, (char)52, (char)205, (char)190, (char)252, (char)160, (char)49, (char)29, (char)176, (char)210, (char)222, (char)131, (char)91, (char)230, (char)184, (char)74, (char)168, (char)95, (char)230, (char)94, (char)71, (char)34, (char)88, (char)37, (char)25, (char)148, (char)21, (char)38, (char)125, (char)84, (char)49, (char)135, (char)103, (char)182, (char)193, (char)164, (char)193, (char)65, (char)226, (char)40, (char)113, (char)186, (char)235, (char)25, (char)79, (char)98, (char)79, (char)221, (char)221, (char)243, (char)134, (char)80, (char)239, (char)82, (char)45, (char)26, (char)214, (char)202, (char)8, (char)111, (char)28, (char)10, (char)179, (char)198, (char)225, (char)133, (char)153, (char)35, (char)8, (char)80, (char)51, (char)13, (char)212, (char)118, (char)202, (char)70, (char)89, (char)25, (char)81, (char)208, (char)203, (char)46, (char)102, (char)106, (char)23, (char)188, (char)114, (char)79, (char)43, (char)115, (char)198, (char)153, (char)174, (char)134, (char)220, (char)15, (char)186, (char)104, (char)222, (char)155, (char)21, (char)227, (char)110, (char)233, (char)177, (char)96, (char)72, (char)85, (char)213, (char)132, (char)248, (char)176, (char)151, (char)36, (char)130, (char)223, (char)27, (char)102, (char)79, (char)150, (char)96, (char)211, (char)84, (char)243, (char)203, (char)166, (char)120, (char)210, (char)140, (char)117, (char)160, (char)176, (char)154, (char)225, (char)150, (char)102, (char)127, (char)69, (char)250, (char)17, (char)4, (char)87, (char)194, (char)240, (char)113, (char)88, (char)115, (char)196, (char)18, (char)2, (char)218, (char)94, (char)74, (char)101, (char)93, (char)114, (char)72, (char)99, (char)234, (char)106, (char)116, (char)170, (char)227, (char)95, (char)172, (char)88, (char)5, (char)38, (char)238, (char)102, (char)220, (char)24, (char)101, (char)206, (char)35, (char)158, (char)222, (char)85, (char)198, (char)11, (char)10, (char)155, (char)220, (char)200, (char)69, (char)20, (char)58, (char)253, (char)183, (char)92, (char)28, (char)6, (char)129, (char)130, (char)118, (char)152, (char)114, (char)212, (char)161, (char)177, (char)183, (char)218, (char)107, (char)77, (char)204, (char)149, (char)101, (char)35, (char)58, (char)38, (char)89, (char)221, (char)1, (char)46, (char)34, (char)142, (char)134, (char)100, (char)207, (char)243, (char)236, (char)156, (char)70, (char)82, (char)212, (char)184, (char)56, (char)120, (char)2, (char)230, (char)94, (char)229, (char)92, (char)171, (char)100, (char)187, (char)35, (char)188, (char)237, (char)194, (char)105, (char)218, (char)202}, 0) ;
        p131.seqnr_SET((char)59047) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(pack.min_distance_GET() == (char)43624);
            assert(pack.id_GET() == (char)196);
            assert(pack.covariance_GET() == (char)197);
            assert(pack.current_distance_GET() == (char)24878);
            assert(pack.time_boot_ms_GET() == 3479357730L);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_180);
            assert(pack.max_distance_GET() == (char)62607);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(3479357730L) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p132.max_distance_SET((char)62607) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_180) ;
        p132.id_SET((char)196) ;
        p132.min_distance_SET((char)43624) ;
        p132.current_distance_SET((char)24878) ;
        p132.covariance_SET((char)197) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mask_GET() == 5154497265799306330L);
            assert(pack.lat_GET() == -1281566101);
            assert(pack.grid_spacing_GET() == (char)60553);
            assert(pack.lon_GET() == 487084808);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.grid_spacing_SET((char)60553) ;
        p133.lat_SET(-1281566101) ;
        p133.lon_SET(487084808) ;
        p133.mask_SET(5154497265799306330L) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -826612555);
            assert(pack.lon_GET() == -646127031);
            assert(pack.grid_spacing_GET() == (char)6544);
            assert(pack.gridbit_GET() == (char)180);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)31328, (short) -4014, (short) -299, (short) -7065, (short) -26550, (short) -27663, (short)25932, (short)2458, (short) -19812, (short) -3403, (short) -14050, (short) -26864, (short)13253, (short)1822, (short) -13, (short) -23246}));
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lon_SET(-646127031) ;
        p134.lat_SET(-826612555) ;
        p134.gridbit_SET((char)180) ;
        p134.grid_spacing_SET((char)6544) ;
        p134.data__SET(new short[] {(short)31328, (short) -4014, (short) -299, (short) -7065, (short) -26550, (short) -27663, (short)25932, (short)2458, (short) -19812, (short) -3403, (short) -14050, (short) -26864, (short)13253, (short)1822, (short) -13, (short) -23246}, 0) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 374619660);
            assert(pack.lat_GET() == -2005128944);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-2005128944) ;
        p135.lon_SET(374619660) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.spacing_GET() == (char)17334);
            assert(pack.current_height_GET() == -5.554866E36F);
            assert(pack.terrain_height_GET() == 2.4211145E38F);
            assert(pack.pending_GET() == (char)5475);
            assert(pack.lon_GET() == -1137687693);
            assert(pack.lat_GET() == -1926293909);
            assert(pack.loaded_GET() == (char)12724);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.current_height_SET(-5.554866E36F) ;
        p136.terrain_height_SET(2.4211145E38F) ;
        p136.lon_SET(-1137687693) ;
        p136.lat_SET(-1926293909) ;
        p136.pending_SET((char)5475) ;
        p136.spacing_SET((char)17334) ;
        p136.loaded_SET((char)12724) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2865561313L);
            assert(pack.press_abs_GET() == -1.8075856E38F);
            assert(pack.press_diff_GET() == 3.2727477E38F);
            assert(pack.temperature_GET() == (short) -31328);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_diff_SET(3.2727477E38F) ;
        p137.time_boot_ms_SET(2865561313L) ;
        p137.temperature_SET((short) -31328) ;
        p137.press_abs_SET(-1.8075856E38F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 834063722049578554L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.2058951E38F, 1.0978428E36F, 3.2978291E38F, -1.3624199E38F}));
            assert(pack.x_GET() == -7.9247867E37F);
            assert(pack.y_GET() == 1.7681655E38F);
            assert(pack.z_GET() == -2.7310896E37F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(834063722049578554L) ;
        p138.z_SET(-2.7310896E37F) ;
        p138.x_SET(-7.9247867E37F) ;
        p138.y_SET(1.7681655E38F) ;
        p138.q_SET(new float[] {1.2058951E38F, 1.0978428E36F, 3.2978291E38F, -1.3624199E38F}, 0) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)201);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.5306243E38F, 6.284952E36F, -5.6637316E37F, 3.250161E38F, -1.47717E37F, 4.5302587E37F, 4.2149616E37F, 2.1475786E38F}));
            assert(pack.time_usec_GET() == 4247703477781908549L);
            assert(pack.target_system_GET() == (char)140);
            assert(pack.target_component_GET() == (char)50);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(4247703477781908549L) ;
        p139.group_mlx_SET((char)201) ;
        p139.controls_SET(new float[] {-1.5306243E38F, 6.284952E36F, -5.6637316E37F, 3.250161E38F, -1.47717E37F, 4.5302587E37F, 4.2149616E37F, 2.1475786E38F}, 0) ;
        p139.target_component_SET((char)50) ;
        p139.target_system_SET((char)140) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.0513327E38F, 1.0272511E38F, -1.1431714E38F, 2.1807367E38F, -2.6572346E38F, 9.846521E37F, 2.972011E38F, 1.8833597E38F}));
            assert(pack.group_mlx_GET() == (char)177);
            assert(pack.time_usec_GET() == 5011649016687064911L);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(5011649016687064911L) ;
        p140.group_mlx_SET((char)177) ;
        p140.controls_SET(new float[] {-2.0513327E38F, 1.0272511E38F, -1.1431714E38F, 2.1807367E38F, -2.6572346E38F, 9.846521E37F, 2.972011E38F, 1.8833597E38F}, 0) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_relative_GET() == -3.140477E38F);
            assert(pack.altitude_local_GET() == 1.2828416E38F);
            assert(pack.time_usec_GET() == 1867988826739502192L);
            assert(pack.altitude_terrain_GET() == -2.5567326E37F);
            assert(pack.altitude_amsl_GET() == -3.2363179E38F);
            assert(pack.bottom_clearance_GET() == -2.1902686E38F);
            assert(pack.altitude_monotonic_GET() == 1.0514308E38F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_monotonic_SET(1.0514308E38F) ;
        p141.altitude_amsl_SET(-3.2363179E38F) ;
        p141.altitude_relative_SET(-3.140477E38F) ;
        p141.altitude_terrain_SET(-2.5567326E37F) ;
        p141.bottom_clearance_SET(-2.1902686E38F) ;
        p141.time_usec_SET(1867988826739502192L) ;
        p141.altitude_local_SET(1.2828416E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)170, (char)125, (char)110, (char)29, (char)131, (char)12, (char)67, (char)64, (char)245, (char)132, (char)73, (char)162, (char)218, (char)105, (char)63, (char)243, (char)150, (char)197, (char)76, (char)251, (char)214, (char)169, (char)89, (char)117, (char)100, (char)73, (char)40, (char)108, (char)173, (char)251, (char)116, (char)182, (char)41, (char)83, (char)114, (char)47, (char)129, (char)136, (char)49, (char)25, (char)39, (char)216, (char)119, (char)7, (char)34, (char)7, (char)24, (char)75, (char)81, (char)69, (char)210, (char)84, (char)6, (char)116, (char)141, (char)123, (char)216, (char)219, (char)148, (char)221, (char)179, (char)178, (char)39, (char)86, (char)199, (char)84, (char)125, (char)4, (char)165, (char)97, (char)12, (char)151, (char)163, (char)88, (char)12, (char)141, (char)254, (char)221, (char)223, (char)31, (char)39, (char)155, (char)188, (char)75, (char)39, (char)175, (char)246, (char)213, (char)197, (char)105, (char)128, (char)46, (char)192, (char)244, (char)213, (char)187, (char)220, (char)140, (char)232, (char)133, (char)45, (char)183, (char)250, (char)96, (char)70, (char)188, (char)7, (char)63, (char)171, (char)117, (char)55, (char)39, (char)75, (char)185, (char)87, (char)147, (char)65, (char)220, (char)218, (char)153}));
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)2, (char)96, (char)43, (char)250, (char)208, (char)92, (char)11, (char)99, (char)27, (char)71, (char)213, (char)168, (char)97, (char)214, (char)104, (char)22, (char)4, (char)187, (char)81, (char)26, (char)132, (char)14, (char)163, (char)126, (char)238, (char)62, (char)242, (char)14, (char)95, (char)4, (char)119, (char)54, (char)71, (char)100, (char)68, (char)153, (char)243, (char)127, (char)78, (char)121, (char)122, (char)234, (char)67, (char)157, (char)203, (char)37, (char)228, (char)71, (char)38, (char)165, (char)11, (char)122, (char)175, (char)176, (char)66, (char)172, (char)78, (char)180, (char)61, (char)41, (char)90, (char)133, (char)243, (char)230, (char)53, (char)1, (char)249, (char)58, (char)224, (char)247, (char)136, (char)17, (char)162, (char)62, (char)249, (char)97, (char)188, (char)171, (char)45, (char)34, (char)144, (char)221, (char)231, (char)159, (char)133, (char)165, (char)80, (char)11, (char)0, (char)30, (char)171, (char)71, (char)153, (char)123, (char)197, (char)122, (char)95, (char)135, (char)225, (char)175, (char)227, (char)154, (char)98, (char)225, (char)229, (char)141, (char)25, (char)125, (char)187, (char)7, (char)14, (char)86, (char)188, (char)121, (char)247, (char)122, (char)164, (char)214, (char)70, (char)32}));
            assert(pack.uri_type_GET() == (char)154);
            assert(pack.transfer_type_GET() == (char)77);
            assert(pack.request_id_GET() == (char)62);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_SET(new char[] {(char)170, (char)125, (char)110, (char)29, (char)131, (char)12, (char)67, (char)64, (char)245, (char)132, (char)73, (char)162, (char)218, (char)105, (char)63, (char)243, (char)150, (char)197, (char)76, (char)251, (char)214, (char)169, (char)89, (char)117, (char)100, (char)73, (char)40, (char)108, (char)173, (char)251, (char)116, (char)182, (char)41, (char)83, (char)114, (char)47, (char)129, (char)136, (char)49, (char)25, (char)39, (char)216, (char)119, (char)7, (char)34, (char)7, (char)24, (char)75, (char)81, (char)69, (char)210, (char)84, (char)6, (char)116, (char)141, (char)123, (char)216, (char)219, (char)148, (char)221, (char)179, (char)178, (char)39, (char)86, (char)199, (char)84, (char)125, (char)4, (char)165, (char)97, (char)12, (char)151, (char)163, (char)88, (char)12, (char)141, (char)254, (char)221, (char)223, (char)31, (char)39, (char)155, (char)188, (char)75, (char)39, (char)175, (char)246, (char)213, (char)197, (char)105, (char)128, (char)46, (char)192, (char)244, (char)213, (char)187, (char)220, (char)140, (char)232, (char)133, (char)45, (char)183, (char)250, (char)96, (char)70, (char)188, (char)7, (char)63, (char)171, (char)117, (char)55, (char)39, (char)75, (char)185, (char)87, (char)147, (char)65, (char)220, (char)218, (char)153}, 0) ;
        p142.uri_type_SET((char)154) ;
        p142.storage_SET(new char[] {(char)2, (char)96, (char)43, (char)250, (char)208, (char)92, (char)11, (char)99, (char)27, (char)71, (char)213, (char)168, (char)97, (char)214, (char)104, (char)22, (char)4, (char)187, (char)81, (char)26, (char)132, (char)14, (char)163, (char)126, (char)238, (char)62, (char)242, (char)14, (char)95, (char)4, (char)119, (char)54, (char)71, (char)100, (char)68, (char)153, (char)243, (char)127, (char)78, (char)121, (char)122, (char)234, (char)67, (char)157, (char)203, (char)37, (char)228, (char)71, (char)38, (char)165, (char)11, (char)122, (char)175, (char)176, (char)66, (char)172, (char)78, (char)180, (char)61, (char)41, (char)90, (char)133, (char)243, (char)230, (char)53, (char)1, (char)249, (char)58, (char)224, (char)247, (char)136, (char)17, (char)162, (char)62, (char)249, (char)97, (char)188, (char)171, (char)45, (char)34, (char)144, (char)221, (char)231, (char)159, (char)133, (char)165, (char)80, (char)11, (char)0, (char)30, (char)171, (char)71, (char)153, (char)123, (char)197, (char)122, (char)95, (char)135, (char)225, (char)175, (char)227, (char)154, (char)98, (char)225, (char)229, (char)141, (char)25, (char)125, (char)187, (char)7, (char)14, (char)86, (char)188, (char)121, (char)247, (char)122, (char)164, (char)214, (char)70, (char)32}, 0) ;
        p142.request_id_SET((char)62) ;
        p142.transfer_type_SET((char)77) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)1401);
            assert(pack.time_boot_ms_GET() == 3136934910L);
            assert(pack.press_abs_GET() == 7.4884857E37F);
            assert(pack.press_diff_GET() == 3.1090894E38F);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.temperature_SET((short)1401) ;
        p143.time_boot_ms_SET(3136934910L) ;
        p143.press_diff_SET(3.1090894E38F) ;
        p143.press_abs_SET(7.4884857E37F) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.vel_GET(),  new float[] {1.9352541E37F, -1.9083679E38F, -2.0159701E38F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-3.3213202E38F, 3.172461E37F, 2.1552093E38F, 1.377977E38F}));
            assert(pack.lon_GET() == -1284619904);
            assert(pack.est_capabilities_GET() == (char)130);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {3.1571984E38F, -1.4944591E38F, 8.551869E37F}));
            assert(pack.alt_GET() == -1.5366558E37F);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {3.176875E38F, 2.1285266E37F, -1.6769018E38F}));
            assert(pack.lat_GET() == 925971871);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-2.3511278E38F, 1.1601177E38F, 2.4805095E38F}));
            assert(pack.timestamp_GET() == 8784048077513380943L);
            assert(pack.custom_state_GET() == 4783528514162373441L);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.est_capabilities_SET((char)130) ;
        p144.rates_SET(new float[] {3.176875E38F, 2.1285266E37F, -1.6769018E38F}, 0) ;
        p144.custom_state_SET(4783528514162373441L) ;
        p144.lat_SET(925971871) ;
        p144.lon_SET(-1284619904) ;
        p144.alt_SET(-1.5366558E37F) ;
        p144.timestamp_SET(8784048077513380943L) ;
        p144.acc_SET(new float[] {-2.3511278E38F, 1.1601177E38F, 2.4805095E38F}, 0) ;
        p144.position_cov_SET(new float[] {3.1571984E38F, -1.4944591E38F, 8.551869E37F}, 0) ;
        p144.attitude_q_SET(new float[] {-3.3213202E38F, 3.172461E37F, 2.1552093E38F, 1.377977E38F}, 0) ;
        p144.vel_SET(new float[] {1.9352541E37F, -1.9083679E38F, -2.0159701E38F}, 0) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.x_acc_GET() == -7.282214E37F);
            assert(pack.y_acc_GET() == 4.6349946E37F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {4.6825203E37F, 2.3479745E38F, -1.2668502E37F}));
            assert(pack.pitch_rate_GET() == 8.3004986E37F);
            assert(pack.x_vel_GET() == 3.1325255E38F);
            assert(pack.z_pos_GET() == 1.3751722E38F);
            assert(pack.z_vel_GET() == -1.8708649E38F);
            assert(pack.roll_rate_GET() == 2.2005836E38F);
            assert(pack.x_pos_GET() == 1.3659544E37F);
            assert(pack.yaw_rate_GET() == 6.068445E37F);
            assert(pack.airspeed_GET() == 3.2875802E38F);
            assert(pack.z_acc_GET() == 3.8781767E37F);
            assert(pack.time_usec_GET() == 2647620191409274159L);
            assert(pack.y_pos_GET() == -6.5314024E36F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {1.8773226E37F, -4.52495E37F, -2.8149758E38F}));
            assert(pack.y_vel_GET() == 2.7650512E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.003803E38F, -2.0199971E36F, -3.3528825E38F, 2.371997E38F}));
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.z_pos_SET(1.3751722E38F) ;
        p146.y_acc_SET(4.6349946E37F) ;
        p146.z_vel_SET(-1.8708649E38F) ;
        p146.pos_variance_SET(new float[] {4.6825203E37F, 2.3479745E38F, -1.2668502E37F}, 0) ;
        p146.pitch_rate_SET(8.3004986E37F) ;
        p146.time_usec_SET(2647620191409274159L) ;
        p146.yaw_rate_SET(6.068445E37F) ;
        p146.x_vel_SET(3.1325255E38F) ;
        p146.airspeed_SET(3.2875802E38F) ;
        p146.x_pos_SET(1.3659544E37F) ;
        p146.y_pos_SET(-6.5314024E36F) ;
        p146.x_acc_SET(-7.282214E37F) ;
        p146.z_acc_SET(3.8781767E37F) ;
        p146.y_vel_SET(2.7650512E37F) ;
        p146.vel_variance_SET(new float[] {1.8773226E37F, -4.52495E37F, -2.8149758E38F}, 0) ;
        p146.q_SET(new float[] {-3.003803E38F, -2.0199971E36F, -3.3528825E38F, 2.371997E38F}, 0) ;
        p146.roll_rate_SET(2.2005836E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)54211, (char)6597, (char)35771, (char)62511, (char)49660, (char)9465, (char)18878, (char)15947, (char)15091, (char)5578}));
            assert(pack.battery_remaining_GET() == (byte)84);
            assert(pack.temperature_GET() == (short)23690);
            assert(pack.current_consumed_GET() == -430950670);
            assert(pack.id_GET() == (char)189);
            assert(pack.current_battery_GET() == (short)16226);
            assert(pack.energy_consumed_GET() == -893597999);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.current_battery_SET((short)16226) ;
        p147.temperature_SET((short)23690) ;
        p147.battery_remaining_SET((byte)84) ;
        p147.current_consumed_SET(-430950670) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE) ;
        p147.id_SET((char)189) ;
        p147.energy_consumed_SET(-893597999) ;
        p147.voltages_SET(new char[] {(char)54211, (char)6597, (char)35771, (char)62511, (char)49660, (char)9465, (char)18878, (char)15947, (char)15091, (char)5578}, 0) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.uid_GET() == 2886233144305836376L);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)232, (char)146, (char)111, (char)245, (char)30, (char)218, (char)188, (char)243}));
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)178, (char)68, (char)58, (char)243, (char)74, (char)217, (char)170, (char)52}));
            assert(pack.flight_sw_version_GET() == 718116056L);
            assert(pack.product_id_GET() == (char)21362);
            assert(pack.vendor_id_GET() == (char)7816);
            assert(pack.middleware_sw_version_GET() == 4027933731L);
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT));
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)211, (char)108, (char)89, (char)224, (char)62, (char)43, (char)176, (char)73, (char)213, (char)51, (char)148, (char)185, (char)8, (char)87, (char)175, (char)235, (char)105, (char)80}));
            assert(pack.board_version_GET() == 388179011L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)138, (char)30, (char)76, (char)114, (char)130, (char)33, (char)46, (char)244}));
            assert(pack.os_sw_version_GET() == 2151309912L);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.vendor_id_SET((char)7816) ;
        p148.product_id_SET((char)21362) ;
        p148.board_version_SET(388179011L) ;
        p148.middleware_custom_version_SET(new char[] {(char)232, (char)146, (char)111, (char)245, (char)30, (char)218, (char)188, (char)243}, 0) ;
        p148.flight_custom_version_SET(new char[] {(char)178, (char)68, (char)58, (char)243, (char)74, (char)217, (char)170, (char)52}, 0) ;
        p148.os_custom_version_SET(new char[] {(char)138, (char)30, (char)76, (char)114, (char)130, (char)33, (char)46, (char)244}, 0) ;
        p148.uid_SET(2886233144305836376L) ;
        p148.flight_sw_version_SET(718116056L) ;
        p148.middleware_sw_version_SET(4027933731L) ;
        p148.os_sw_version_SET(2151309912L) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT)) ;
        p148.uid2_SET(new char[] {(char)211, (char)108, (char)89, (char)224, (char)62, (char)43, (char)176, (char)73, (char)213, (char)51, (char)148, (char)185, (char)8, (char)87, (char)175, (char)235, (char)105, (char)80}, 0, PH) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.x_TRY(ph) == 1.9297175E38F);
            assert(pack.size_x_GET() == 1.1627547E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {3.2122228E37F, -1.7691318E38F, 2.9798488E37F, 3.284843E38F}));
            assert(pack.target_num_GET() == (char)26);
            assert(pack.angle_y_GET() == -7.4646686E37F);
            assert(pack.z_TRY(ph) == 2.9796507E38F);
            assert(pack.distance_GET() == -4.0748783E37F);
            assert(pack.y_TRY(ph) == -1.3463706E38F);
            assert(pack.size_y_GET() == 9.207969E37F);
            assert(pack.time_usec_GET() == 2235970561659869735L);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
            assert(pack.position_valid_TRY(ph) == (char)2);
            assert(pack.angle_x_GET() == 1.8437487E38F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.position_valid_SET((char)2, PH) ;
        p149.size_y_SET(9.207969E37F) ;
        p149.distance_SET(-4.0748783E37F) ;
        p149.z_SET(2.9796507E38F, PH) ;
        p149.angle_x_SET(1.8437487E38F) ;
        p149.size_x_SET(1.1627547E38F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p149.y_SET(-1.3463706E38F, PH) ;
        p149.target_num_SET((char)26) ;
        p149.time_usec_SET(2235970561659869735L) ;
        p149.angle_y_SET(-7.4646686E37F) ;
        p149.x_SET(1.9297175E38F, PH) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON) ;
        p149.q_SET(new float[] {3.2122228E37F, -1.7691318E38F, 2.9798488E37F, 3.284843E38F}, 0, PH) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCRIPT_ITEM.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)178);
            assert(pack.seq_GET() == (char)61107);
            assert(pack.target_component_GET() == (char)84);
            assert(pack.name_LEN(ph) == 21);
            assert(pack.name_TRY(ph).equals("hDdeasbCcpevcgzybxvmt"));
        });
        GroundControl.SCRIPT_ITEM p180 = CommunicationChannel.new_SCRIPT_ITEM();
        PH.setPack(p180);
        p180.target_component_SET((char)84) ;
        p180.target_system_SET((char)178) ;
        p180.name_SET("hDdeasbCcpevcgzybxvmt", PH) ;
        p180.seq_SET((char)61107) ;
        CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCRIPT_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)7685);
            assert(pack.target_system_GET() == (char)25);
            assert(pack.target_component_GET() == (char)115);
        });
        GroundControl.SCRIPT_REQUEST p181 = CommunicationChannel.new_SCRIPT_REQUEST();
        PH.setPack(p181);
        p181.target_system_SET((char)25) ;
        p181.seq_SET((char)7685) ;
        p181.target_component_SET((char)115) ;
        CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCRIPT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)113);
            assert(pack.target_component_GET() == (char)178);
        });
        GroundControl.SCRIPT_REQUEST_LIST p182 = CommunicationChannel.new_SCRIPT_REQUEST_LIST();
        PH.setPack(p182);
        p182.target_component_SET((char)178) ;
        p182.target_system_SET((char)113) ;
        CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCRIPT_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)153);
            assert(pack.count_GET() == (char)48413);
            assert(pack.target_component_GET() == (char)118);
        });
        GroundControl.SCRIPT_COUNT p183 = CommunicationChannel.new_SCRIPT_COUNT();
        PH.setPack(p183);
        p183.count_SET((char)48413) ;
        p183.target_system_SET((char)153) ;
        p183.target_component_SET((char)118) ;
        CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCRIPT_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)43427);
        });
        GroundControl.SCRIPT_CURRENT p184 = CommunicationChannel.new_SCRIPT_CURRENT();
        PH.setPack(p184);
        p184.seq_SET((char)43427) ;
        CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS));
            assert(pack.hagl_ratio_GET() == -1.7267878E38F);
            assert(pack.pos_vert_accuracy_GET() == -5.940994E37F);
            assert(pack.time_usec_GET() == 950839809619548135L);
            assert(pack.pos_horiz_ratio_GET() == 3.13017E38F);
            assert(pack.mag_ratio_GET() == -2.144089E38F);
            assert(pack.pos_vert_ratio_GET() == 2.4257322E38F);
            assert(pack.tas_ratio_GET() == 2.753978E38F);
            assert(pack.vel_ratio_GET() == 1.7183951E38F);
            assert(pack.pos_horiz_accuracy_GET() == 5.725968E37F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.tas_ratio_SET(2.753978E38F) ;
        p230.mag_ratio_SET(-2.144089E38F) ;
        p230.pos_vert_accuracy_SET(-5.940994E37F) ;
        p230.time_usec_SET(950839809619548135L) ;
        p230.pos_vert_ratio_SET(2.4257322E38F) ;
        p230.pos_horiz_accuracy_SET(5.725968E37F) ;
        p230.pos_horiz_ratio_SET(3.13017E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS)) ;
        p230.hagl_ratio_SET(-1.7267878E38F) ;
        p230.vel_ratio_SET(1.7183951E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3931565422461976936L);
            assert(pack.var_vert_GET() == -5.715354E37F);
            assert(pack.wind_z_GET() == 1.8742733E38F);
            assert(pack.horiz_accuracy_GET() == 8.642025E37F);
            assert(pack.wind_x_GET() == -1.5777962E38F);
            assert(pack.wind_alt_GET() == 1.1524864E37F);
            assert(pack.vert_accuracy_GET() == -7.733188E37F);
            assert(pack.var_horiz_GET() == 4.03395E37F);
            assert(pack.wind_y_GET() == 2.627413E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_y_SET(2.627413E38F) ;
        p231.horiz_accuracy_SET(8.642025E37F) ;
        p231.vert_accuracy_SET(-7.733188E37F) ;
        p231.var_horiz_SET(4.03395E37F) ;
        p231.wind_x_SET(-1.5777962E38F) ;
        p231.var_vert_SET(-5.715354E37F) ;
        p231.wind_alt_SET(1.1524864E37F) ;
        p231.wind_z_SET(1.8742733E38F) ;
        p231.time_usec_SET(3931565422461976936L) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.vd_GET() == -1.1385373E38F);
            assert(pack.vdop_GET() == -3.0786732E38F);
            assert(pack.vn_GET() == 1.5182191E38F);
            assert(pack.satellites_visible_GET() == (char)248);
            assert(pack.lat_GET() == 433982635);
            assert(pack.vert_accuracy_GET() == 3.3970134E38F);
            assert(pack.fix_type_GET() == (char)93);
            assert(pack.gps_id_GET() == (char)89);
            assert(pack.time_week_GET() == (char)9184);
            assert(pack.time_week_ms_GET() == 2319313787L);
            assert(pack.lon_GET() == 1817959847);
            assert(pack.horiz_accuracy_GET() == 1.797066E38F);
            assert(pack.ve_GET() == -1.2324588E38F);
            assert(pack.alt_GET() == 3.3136857E38F);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP));
            assert(pack.hdop_GET() == 2.057009E38F);
            assert(pack.speed_accuracy_GET() == 3.2652124E38F);
            assert(pack.time_usec_GET() == 7693753091460278391L);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_week_SET((char)9184) ;
        p232.fix_type_SET((char)93) ;
        p232.vn_SET(1.5182191E38F) ;
        p232.speed_accuracy_SET(3.2652124E38F) ;
        p232.lat_SET(433982635) ;
        p232.vd_SET(-1.1385373E38F) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP)) ;
        p232.time_usec_SET(7693753091460278391L) ;
        p232.gps_id_SET((char)89) ;
        p232.lon_SET(1817959847) ;
        p232.vdop_SET(-3.0786732E38F) ;
        p232.ve_SET(-1.2324588E38F) ;
        p232.satellites_visible_SET((char)248) ;
        p232.vert_accuracy_SET(3.3970134E38F) ;
        p232.time_week_ms_SET(2319313787L) ;
        p232.alt_SET(3.3136857E38F) ;
        p232.hdop_SET(2.057009E38F) ;
        p232.horiz_accuracy_SET(1.797066E38F) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)220);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)195, (char)46, (char)54, (char)115, (char)123, (char)3, (char)186, (char)70, (char)3, (char)220, (char)118, (char)196, (char)203, (char)8, (char)224, (char)111, (char)143, (char)50, (char)98, (char)247, (char)21, (char)225, (char)140, (char)250, (char)16, (char)153, (char)86, (char)230, (char)165, (char)85, (char)129, (char)8, (char)82, (char)236, (char)5, (char)33, (char)37, (char)112, (char)195, (char)160, (char)242, (char)237, (char)187, (char)98, (char)54, (char)42, (char)22, (char)235, (char)115, (char)197, (char)225, (char)231, (char)10, (char)216, (char)229, (char)43, (char)232, (char)189, (char)116, (char)46, (char)33, (char)88, (char)215, (char)54, (char)197, (char)201, (char)159, (char)232, (char)231, (char)46, (char)214, (char)28, (char)17, (char)222, (char)75, (char)8, (char)71, (char)191, (char)211, (char)233, (char)135, (char)194, (char)41, (char)14, (char)45, (char)226, (char)74, (char)126, (char)37, (char)25, (char)71, (char)43, (char)246, (char)242, (char)178, (char)137, (char)120, (char)51, (char)102, (char)129, (char)3, (char)20, (char)158, (char)195, (char)55, (char)44, (char)146, (char)6, (char)118, (char)56, (char)47, (char)241, (char)85, (char)77, (char)142, (char)213, (char)149, (char)72, (char)228, (char)134, (char)151, (char)232, (char)199, (char)53, (char)84, (char)136, (char)24, (char)199, (char)105, (char)237, (char)42, (char)136, (char)57, (char)41, (char)11, (char)238, (char)27, (char)6, (char)130, (char)47, (char)234, (char)86, (char)156, (char)175, (char)9, (char)163, (char)244, (char)131, (char)136, (char)142, (char)212, (char)177, (char)171, (char)42, (char)3, (char)97, (char)29, (char)125, (char)81, (char)74, (char)14, (char)49, (char)220, (char)43, (char)84, (char)240, (char)133, (char)33, (char)193, (char)78, (char)107, (char)245, (char)50, (char)119, (char)110, (char)133, (char)250, (char)80, (char)38, (char)202}));
            assert(pack.flags_GET() == (char)54);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)54) ;
        p233.data__SET(new char[] {(char)195, (char)46, (char)54, (char)115, (char)123, (char)3, (char)186, (char)70, (char)3, (char)220, (char)118, (char)196, (char)203, (char)8, (char)224, (char)111, (char)143, (char)50, (char)98, (char)247, (char)21, (char)225, (char)140, (char)250, (char)16, (char)153, (char)86, (char)230, (char)165, (char)85, (char)129, (char)8, (char)82, (char)236, (char)5, (char)33, (char)37, (char)112, (char)195, (char)160, (char)242, (char)237, (char)187, (char)98, (char)54, (char)42, (char)22, (char)235, (char)115, (char)197, (char)225, (char)231, (char)10, (char)216, (char)229, (char)43, (char)232, (char)189, (char)116, (char)46, (char)33, (char)88, (char)215, (char)54, (char)197, (char)201, (char)159, (char)232, (char)231, (char)46, (char)214, (char)28, (char)17, (char)222, (char)75, (char)8, (char)71, (char)191, (char)211, (char)233, (char)135, (char)194, (char)41, (char)14, (char)45, (char)226, (char)74, (char)126, (char)37, (char)25, (char)71, (char)43, (char)246, (char)242, (char)178, (char)137, (char)120, (char)51, (char)102, (char)129, (char)3, (char)20, (char)158, (char)195, (char)55, (char)44, (char)146, (char)6, (char)118, (char)56, (char)47, (char)241, (char)85, (char)77, (char)142, (char)213, (char)149, (char)72, (char)228, (char)134, (char)151, (char)232, (char)199, (char)53, (char)84, (char)136, (char)24, (char)199, (char)105, (char)237, (char)42, (char)136, (char)57, (char)41, (char)11, (char)238, (char)27, (char)6, (char)130, (char)47, (char)234, (char)86, (char)156, (char)175, (char)9, (char)163, (char)244, (char)131, (char)136, (char)142, (char)212, (char)177, (char)171, (char)42, (char)3, (char)97, (char)29, (char)125, (char)81, (char)74, (char)14, (char)49, (char)220, (char)43, (char)84, (char)240, (char)133, (char)33, (char)193, (char)78, (char)107, (char)245, (char)50, (char)119, (char)110, (char)133, (char)250, (char)80, (char)38, (char)202}, 0) ;
        p233.len_SET((char)220) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.altitude_sp_GET() == (short)16631);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
            assert(pack.pitch_GET() == (short)16798);
            assert(pack.climb_rate_GET() == (byte)95);
            assert(pack.temperature_air_GET() == (byte)32);
            assert(pack.failsafe_GET() == (char)183);
            assert(pack.custom_mode_GET() == 3739971378L);
            assert(pack.battery_remaining_GET() == (char)161);
            assert(pack.groundspeed_GET() == (char)200);
            assert(pack.longitude_GET() == -1288124485);
            assert(pack.airspeed_GET() == (char)228);
            assert(pack.wp_distance_GET() == (char)11558);
            assert(pack.airspeed_sp_GET() == (char)17);
            assert(pack.latitude_GET() == 1405280698);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.gps_nsat_GET() == (char)159);
            assert(pack.altitude_amsl_GET() == (short)25205);
            assert(pack.wp_num_GET() == (char)200);
            assert(pack.throttle_GET() == (byte) - 65);
            assert(pack.roll_GET() == (short)23568);
            assert(pack.temperature_GET() == (byte)58);
            assert(pack.heading_sp_GET() == (short)25820);
            assert(pack.heading_GET() == (char)61193);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF) ;
        p234.wp_num_SET((char)200) ;
        p234.battery_remaining_SET((char)161) ;
        p234.airspeed_SET((char)228) ;
        p234.wp_distance_SET((char)11558) ;
        p234.heading_SET((char)61193) ;
        p234.custom_mode_SET(3739971378L) ;
        p234.pitch_SET((short)16798) ;
        p234.throttle_SET((byte) - 65) ;
        p234.gps_nsat_SET((char)159) ;
        p234.roll_SET((short)23568) ;
        p234.longitude_SET(-1288124485) ;
        p234.airspeed_sp_SET((char)17) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p234.groundspeed_SET((char)200) ;
        p234.temperature_SET((byte)58) ;
        p234.temperature_air_SET((byte)32) ;
        p234.climb_rate_SET((byte)95) ;
        p234.altitude_sp_SET((short)16631) ;
        p234.latitude_SET(1405280698) ;
        p234.altitude_amsl_SET((short)25205) ;
        p234.heading_sp_SET((short)25820) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED)) ;
        p234.failsafe_SET((char)183) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_x_GET() == 7.130329E37F);
            assert(pack.clipping_1_GET() == 3180419175L);
            assert(pack.vibration_y_GET() == -1.5468542E38F);
            assert(pack.clipping_2_GET() == 342129666L);
            assert(pack.vibration_z_GET() == -1.2961725E38F);
            assert(pack.time_usec_GET() == 7656428987321241695L);
            assert(pack.clipping_0_GET() == 629015844L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.vibration_y_SET(-1.5468542E38F) ;
        p241.clipping_1_SET(3180419175L) ;
        p241.vibration_z_SET(-1.2961725E38F) ;
        p241.clipping_0_SET(629015844L) ;
        p241.clipping_2_SET(342129666L) ;
        p241.vibration_x_SET(7.130329E37F) ;
        p241.time_usec_SET(7656428987321241695L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 3738406674816540780L);
            assert(pack.longitude_GET() == 1939339744);
            assert(pack.approach_z_GET() == 6.0891967E37F);
            assert(pack.altitude_GET() == 512734252);
            assert(pack.latitude_GET() == -935299941);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.0391321E38F, 1.5770361E38F, -4.388767E37F, -4.663169E37F}));
            assert(pack.approach_x_GET() == -3.2528632E38F);
            assert(pack.x_GET() == 2.2838738E38F);
            assert(pack.y_GET() == 2.7642972E37F);
            assert(pack.z_GET() == -5.143045E37F);
            assert(pack.approach_y_GET() == 1.5779748E37F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.approach_x_SET(-3.2528632E38F) ;
        p242.approach_y_SET(1.5779748E37F) ;
        p242.longitude_SET(1939339744) ;
        p242.z_SET(-5.143045E37F) ;
        p242.q_SET(new float[] {-1.0391321E38F, 1.5770361E38F, -4.388767E37F, -4.663169E37F}, 0) ;
        p242.latitude_SET(-935299941) ;
        p242.x_SET(2.2838738E38F) ;
        p242.y_SET(2.7642972E37F) ;
        p242.approach_z_SET(6.0891967E37F) ;
        p242.altitude_SET(512734252) ;
        p242.time_usec_SET(3738406674816540780L, PH) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 1589758965);
            assert(pack.approach_y_GET() == -8.0809024E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.025205E38F, 1.4362621E38F, 1.5311446E36F, 3.3916203E38F}));
            assert(pack.y_GET() == -1.4185446E38F);
            assert(pack.x_GET() == -2.8574587E38F);
            assert(pack.z_GET() == 1.0457374E38F);
            assert(pack.approach_z_GET() == -1.4989308E38F);
            assert(pack.target_system_GET() == (char)100);
            assert(pack.altitude_GET() == -1812069326);
            assert(pack.longitude_GET() == 226726057);
            assert(pack.time_usec_TRY(ph) == 2531576810327211860L);
            assert(pack.approach_x_GET() == 2.9993323E38F);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.time_usec_SET(2531576810327211860L, PH) ;
        p243.approach_x_SET(2.9993323E38F) ;
        p243.z_SET(1.0457374E38F) ;
        p243.altitude_SET(-1812069326) ;
        p243.y_SET(-1.4185446E38F) ;
        p243.approach_z_SET(-1.4989308E38F) ;
        p243.latitude_SET(1589758965) ;
        p243.longitude_SET(226726057) ;
        p243.q_SET(new float[] {-2.025205E38F, 1.4362621E38F, 1.5311446E36F, 3.3916203E38F}, 0) ;
        p243.approach_y_SET(-8.0809024E37F) ;
        p243.target_system_SET((char)100) ;
        p243.x_SET(-2.8574587E38F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)14749);
            assert(pack.interval_us_GET() == -1800669666);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-1800669666) ;
        p244.message_id_SET((char)14749) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_FW) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.tslc_GET() == (char)230);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.lon_GET() == -701121211);
            assert(pack.altitude_GET() == -1069343842);
            assert(pack.lat_GET() == -1685212404);
            assert(pack.heading_GET() == (char)28617);
            assert(pack.hor_velocity_GET() == (char)56768);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGHLY_MANUV);
            assert(pack.callsign_LEN(ph) == 6);
            assert(pack.callsign_TRY(ph).equals("rtfgau"));
            assert(pack.squawk_GET() == (char)50512);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING));
            assert(pack.ver_velocity_GET() == (short) -26147);
            assert(pack.ICAO_address_GET() == 2759743333L);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.hor_velocity_SET((char)56768) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGHLY_MANUV) ;
        p246.lon_SET(-701121211) ;
        p246.lat_SET(-1685212404) ;
        p246.ver_velocity_SET((short) -26147) ;
        p246.ICAO_address_SET(2759743333L) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING)) ;
        p246.altitude_SET(-1069343842) ;
        p246.callsign_SET("rtfgau", PH) ;
        p246.squawk_SET((char)50512) ;
        p246.heading_SET((char)28617) ;
        p246.tslc_SET((char)230) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.altitude_minimum_delta_GET() == -3.367332E38F);
            assert(pack.threat_level_GET() == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE |
                                               MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW));
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT);
            assert(pack.time_to_minimum_delta_GET() == 1.4014851E38F);
            assert(pack.id_GET() == 401215897L);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.horizontal_minimum_delta_GET() == -1.7044043E38F);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.time_to_minimum_delta_SET(1.4014851E38F) ;
        p247.threat_level_SET((MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE |
                               MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW)) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT) ;
        p247.altitude_minimum_delta_SET(-3.367332E38F) ;
        p247.horizontal_minimum_delta_SET(-1.7044043E38F) ;
        p247.id_SET(401215897L) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)200);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)42, (char)0, (char)204, (char)144, (char)21, (char)242, (char)4, (char)188, (char)85, (char)86, (char)145, (char)32, (char)42, (char)208, (char)162, (char)197, (char)51, (char)216, (char)175, (char)181, (char)28, (char)15, (char)215, (char)158, (char)14, (char)34, (char)207, (char)140, (char)181, (char)166, (char)219, (char)243, (char)46, (char)158, (char)83, (char)8, (char)167, (char)122, (char)90, (char)200, (char)148, (char)21, (char)50, (char)204, (char)50, (char)246, (char)61, (char)56, (char)1, (char)214, (char)172, (char)15, (char)193, (char)36, (char)203, (char)176, (char)15, (char)71, (char)225, (char)208, (char)23, (char)191, (char)157, (char)230, (char)196, (char)43, (char)242, (char)121, (char)68, (char)71, (char)0, (char)72, (char)218, (char)4, (char)99, (char)235, (char)143, (char)196, (char)98, (char)126, (char)35, (char)150, (char)10, (char)41, (char)149, (char)76, (char)147, (char)10, (char)147, (char)178, (char)35, (char)202, (char)240, (char)107, (char)93, (char)124, (char)149, (char)234, (char)45, (char)190, (char)65, (char)165, (char)5, (char)163, (char)239, (char)131, (char)25, (char)254, (char)130, (char)243, (char)187, (char)67, (char)72, (char)177, (char)98, (char)118, (char)218, (char)80, (char)116, (char)102, (char)216, (char)69, (char)156, (char)24, (char)218, (char)51, (char)234, (char)161, (char)214, (char)34, (char)231, (char)157, (char)82, (char)130, (char)30, (char)25, (char)129, (char)178, (char)253, (char)220, (char)129, (char)225, (char)151, (char)213, (char)8, (char)65, (char)162, (char)200, (char)195, (char)197, (char)168, (char)86, (char)132, (char)244, (char)88, (char)225, (char)104, (char)181, (char)35, (char)210, (char)97, (char)86, (char)90, (char)209, (char)106, (char)172, (char)85, (char)48, (char)71, (char)152, (char)248, (char)155, (char)12, (char)88, (char)12, (char)71, (char)13, (char)130, (char)143, (char)242, (char)97, (char)33, (char)204, (char)130, (char)56, (char)55, (char)9, (char)126, (char)72, (char)251, (char)205, (char)237, (char)226, (char)97, (char)239, (char)111, (char)80, (char)254, (char)184, (char)170, (char)197, (char)93, (char)77, (char)103, (char)81, (char)248, (char)56, (char)133, (char)173, (char)87, (char)239, (char)223, (char)149, (char)50, (char)44, (char)229, (char)111, (char)66, (char)215, (char)80, (char)202, (char)131, (char)117, (char)34, (char)40, (char)7, (char)113, (char)214, (char)107, (char)172, (char)77, (char)214, (char)122, (char)14, (char)38, (char)38, (char)252, (char)192, (char)174, (char)160, (char)141, (char)179, (char)193, (char)240, (char)75, (char)68, (char)104, (char)95, (char)51}));
            assert(pack.message_type_GET() == (char)18525);
            assert(pack.target_system_GET() == (char)178);
            assert(pack.target_component_GET() == (char)117);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.message_type_SET((char)18525) ;
        p248.target_network_SET((char)200) ;
        p248.payload_SET(new char[] {(char)42, (char)0, (char)204, (char)144, (char)21, (char)242, (char)4, (char)188, (char)85, (char)86, (char)145, (char)32, (char)42, (char)208, (char)162, (char)197, (char)51, (char)216, (char)175, (char)181, (char)28, (char)15, (char)215, (char)158, (char)14, (char)34, (char)207, (char)140, (char)181, (char)166, (char)219, (char)243, (char)46, (char)158, (char)83, (char)8, (char)167, (char)122, (char)90, (char)200, (char)148, (char)21, (char)50, (char)204, (char)50, (char)246, (char)61, (char)56, (char)1, (char)214, (char)172, (char)15, (char)193, (char)36, (char)203, (char)176, (char)15, (char)71, (char)225, (char)208, (char)23, (char)191, (char)157, (char)230, (char)196, (char)43, (char)242, (char)121, (char)68, (char)71, (char)0, (char)72, (char)218, (char)4, (char)99, (char)235, (char)143, (char)196, (char)98, (char)126, (char)35, (char)150, (char)10, (char)41, (char)149, (char)76, (char)147, (char)10, (char)147, (char)178, (char)35, (char)202, (char)240, (char)107, (char)93, (char)124, (char)149, (char)234, (char)45, (char)190, (char)65, (char)165, (char)5, (char)163, (char)239, (char)131, (char)25, (char)254, (char)130, (char)243, (char)187, (char)67, (char)72, (char)177, (char)98, (char)118, (char)218, (char)80, (char)116, (char)102, (char)216, (char)69, (char)156, (char)24, (char)218, (char)51, (char)234, (char)161, (char)214, (char)34, (char)231, (char)157, (char)82, (char)130, (char)30, (char)25, (char)129, (char)178, (char)253, (char)220, (char)129, (char)225, (char)151, (char)213, (char)8, (char)65, (char)162, (char)200, (char)195, (char)197, (char)168, (char)86, (char)132, (char)244, (char)88, (char)225, (char)104, (char)181, (char)35, (char)210, (char)97, (char)86, (char)90, (char)209, (char)106, (char)172, (char)85, (char)48, (char)71, (char)152, (char)248, (char)155, (char)12, (char)88, (char)12, (char)71, (char)13, (char)130, (char)143, (char)242, (char)97, (char)33, (char)204, (char)130, (char)56, (char)55, (char)9, (char)126, (char)72, (char)251, (char)205, (char)237, (char)226, (char)97, (char)239, (char)111, (char)80, (char)254, (char)184, (char)170, (char)197, (char)93, (char)77, (char)103, (char)81, (char)248, (char)56, (char)133, (char)173, (char)87, (char)239, (char)223, (char)149, (char)50, (char)44, (char)229, (char)111, (char)66, (char)215, (char)80, (char)202, (char)131, (char)117, (char)34, (char)40, (char)7, (char)113, (char)214, (char)107, (char)172, (char)77, (char)214, (char)122, (char)14, (char)38, (char)38, (char)252, (char)192, (char)174, (char)160, (char)141, (char)179, (char)193, (char)240, (char)75, (char)68, (char)104, (char)95, (char)51}, 0) ;
        p248.target_system_SET((char)178) ;
        p248.target_component_SET((char)117) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)6);
            assert(pack.address_GET() == (char)50321);
            assert(pack.ver_GET() == (char)8);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)127, (byte) - 40, (byte)39, (byte) - 50, (byte) - 13, (byte)37, (byte) - 65, (byte) - 62, (byte) - 58, (byte) - 16, (byte)95, (byte)113, (byte) - 22, (byte)124, (byte) - 111, (byte)108, (byte) - 81, (byte)83, (byte)44, (byte)103, (byte) - 8, (byte) - 124, (byte)121, (byte) - 28, (byte)94, (byte)25, (byte)73, (byte)17, (byte)17, (byte) - 34, (byte) - 34, (byte) - 119}));
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.ver_SET((char)8) ;
        p249.address_SET((char)50321) ;
        p249.type_SET((char)6) ;
        p249.value_SET(new byte[] {(byte)127, (byte) - 40, (byte)39, (byte) - 50, (byte) - 13, (byte)37, (byte) - 65, (byte) - 62, (byte) - 58, (byte) - 16, (byte)95, (byte)113, (byte) - 22, (byte)124, (byte) - 111, (byte)108, (byte) - 81, (byte)83, (byte)44, (byte)103, (byte) - 8, (byte) - 124, (byte)121, (byte) - 28, (byte)94, (byte)25, (byte)73, (byte)17, (byte)17, (byte) - 34, (byte) - 34, (byte) - 119}, 0) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -3.3987627E38F);
            assert(pack.x_GET() == -1.5440908E38F);
            assert(pack.time_usec_GET() == 1018973050468283541L);
            assert(pack.z_GET() == -2.7398133E37F);
            assert(pack.name_LEN(ph) == 6);
            assert(pack.name_TRY(ph).equals("iklzfc"));
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.x_SET(-1.5440908E38F) ;
        p250.y_SET(-3.3987627E38F) ;
        p250.time_usec_SET(1018973050468283541L) ;
        p250.z_SET(-2.7398133E37F) ;
        p250.name_SET("iklzfc", PH) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("xmbgafxrua"));
            assert(pack.time_boot_ms_GET() == 1641220487L);
            assert(pack.value_GET() == 4.0244727E37F);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("xmbgafxrua", PH) ;
        p251.time_boot_ms_SET(1641220487L) ;
        p251.value_SET(4.0244727E37F) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 767661552L);
            assert(pack.name_LEN(ph) == 3);
            assert(pack.name_TRY(ph).equals("flr"));
            assert(pack.value_GET() == 1842829962);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.name_SET("flr", PH) ;
        p252.time_boot_ms_SET(767661552L) ;
        p252.value_SET(1842829962) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 46);
            assert(pack.text_TRY(ph).equals("cnjpxpujHvVYoquprxozwgqwpGajweptzazpqsnhwQwljx"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_INFO);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("cnjpxpujHvVYoquprxozwgqwpGajweptzazpqsnhwQwljx", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_INFO) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 1.1480369E38F);
            assert(pack.time_boot_ms_GET() == 2497078925L);
            assert(pack.ind_GET() == (char)44);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(2497078925L) ;
        p254.ind_SET((char)44) ;
        p254.value_SET(1.1480369E38F) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)228);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)41, (char)233, (char)135, (char)180, (char)20, (char)107, (char)241, (char)234, (char)185, (char)184, (char)64, (char)122, (char)234, (char)34, (char)11, (char)129, (char)123, (char)232, (char)157, (char)179, (char)236, (char)201, (char)127, (char)207, (char)138, (char)125, (char)225, (char)80, (char)63, (char)208, (char)48, (char)15}));
            assert(pack.initial_timestamp_GET() == 2647961772401064450L);
            assert(pack.target_component_GET() == (char)0);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)228) ;
        p256.initial_timestamp_SET(2647961772401064450L) ;
        p256.secret_key_SET(new char[] {(char)41, (char)233, (char)135, (char)180, (char)20, (char)107, (char)241, (char)234, (char)185, (char)184, (char)64, (char)122, (char)234, (char)34, (char)11, (char)129, (char)123, (char)232, (char)157, (char)179, (char)236, (char)201, (char)127, (char)207, (char)138, (char)125, (char)225, (char)80, (char)63, (char)208, (char)48, (char)15}, 0) ;
        p256.target_component_SET((char)0) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3437780256L);
            assert(pack.state_GET() == (char)143);
            assert(pack.last_change_ms_GET() == 3545666241L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.last_change_ms_SET(3545666241L) ;
        p257.time_boot_ms_SET(3437780256L) ;
        p257.state_SET((char)143) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)209);
            assert(pack.target_system_GET() == (char)150);
            assert(pack.tune_LEN(ph) == 4);
            assert(pack.tune_TRY(ph).equals("Cdya"));
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)150) ;
        p258.target_component_SET((char)209) ;
        p258.tune_SET("Cdya", PH) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.resolution_h_GET() == (char)62141);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)156, (char)99, (char)163, (char)35, (char)80, (char)148, (char)247, (char)209, (char)207, (char)0, (char)248, (char)152, (char)207, (char)222, (char)37, (char)106, (char)193, (char)157, (char)232, (char)42, (char)238, (char)227, (char)189, (char)83, (char)184, (char)228, (char)136, (char)98, (char)212, (char)22, (char)215, (char)103}));
            assert(pack.sensor_size_v_GET() == 2.9750348E38F);
            assert(pack.lens_id_GET() == (char)169);
            assert(pack.resolution_v_GET() == (char)24732);
            assert(pack.time_boot_ms_GET() == 3308773198L);
            assert(pack.cam_definition_version_GET() == (char)47734);
            assert(pack.cam_definition_uri_LEN(ph) == 53);
            assert(pack.cam_definition_uri_TRY(ph).equals("xyywuhwgqscxqtojgupyhqLuIxzlaingrhsceWcwhqarixelbFrij"));
            assert(pack.sensor_size_h_GET() == 1.5321753E37F);
            assert(pack.focal_length_GET() == -1.6267257E38F);
            assert(pack.firmware_version_GET() == 4270134376L);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)194, (char)102, (char)206, (char)194, (char)207, (char)98, (char)55, (char)0, (char)207, (char)237, (char)161, (char)254, (char)147, (char)102, (char)58, (char)139, (char)119, (char)178, (char)240, (char)3, (char)40, (char)36, (char)61, (char)175, (char)233, (char)142, (char)206, (char)242, (char)88, (char)60, (char)159, (char)73}));
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES));
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.sensor_size_v_SET(2.9750348E38F) ;
        p259.lens_id_SET((char)169) ;
        p259.focal_length_SET(-1.6267257E38F) ;
        p259.vendor_name_SET(new char[] {(char)156, (char)99, (char)163, (char)35, (char)80, (char)148, (char)247, (char)209, (char)207, (char)0, (char)248, (char)152, (char)207, (char)222, (char)37, (char)106, (char)193, (char)157, (char)232, (char)42, (char)238, (char)227, (char)189, (char)83, (char)184, (char)228, (char)136, (char)98, (char)212, (char)22, (char)215, (char)103}, 0) ;
        p259.firmware_version_SET(4270134376L) ;
        p259.model_name_SET(new char[] {(char)194, (char)102, (char)206, (char)194, (char)207, (char)98, (char)55, (char)0, (char)207, (char)237, (char)161, (char)254, (char)147, (char)102, (char)58, (char)139, (char)119, (char)178, (char)240, (char)3, (char)40, (char)36, (char)61, (char)175, (char)233, (char)142, (char)206, (char)242, (char)88, (char)60, (char)159, (char)73}, 0) ;
        p259.cam_definition_version_SET((char)47734) ;
        p259.time_boot_ms_SET(3308773198L) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES)) ;
        p259.resolution_h_SET((char)62141) ;
        p259.cam_definition_uri_SET("xyywuhwgqscxqtojgupyhqLuIxzlaingrhsceWcwhqarixelbFrij", PH) ;
        p259.resolution_v_SET((char)24732) ;
        p259.sensor_size_h_SET(1.5321753E37F) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == (CAMERA_MODE.CAMERA_MODE_IMAGE));
            assert(pack.time_boot_ms_GET() == 2928632267L);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(2928632267L) ;
        p260.mode_id_SET((CAMERA_MODE.CAMERA_MODE_IMAGE)) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2267161140L);
            assert(pack.storage_id_GET() == (char)207);
            assert(pack.total_capacity_GET() == -3.0893793E38F);
            assert(pack.used_capacity_GET() == -1.403564E38F);
            assert(pack.status_GET() == (char)146);
            assert(pack.storage_count_GET() == (char)104);
            assert(pack.write_speed_GET() == -2.5276306E38F);
            assert(pack.read_speed_GET() == 3.1939838E38F);
            assert(pack.available_capacity_GET() == -1.2660309E38F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.write_speed_SET(-2.5276306E38F) ;
        p261.storage_count_SET((char)104) ;
        p261.read_speed_SET(3.1939838E38F) ;
        p261.time_boot_ms_SET(2267161140L) ;
        p261.available_capacity_SET(-1.2660309E38F) ;
        p261.storage_id_SET((char)207) ;
        p261.total_capacity_SET(-3.0893793E38F) ;
        p261.used_capacity_SET(-1.403564E38F) ;
        p261.status_SET((char)146) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.recording_time_ms_GET() == 3765020612L);
            assert(pack.image_interval_GET() == -2.56535E38F);
            assert(pack.time_boot_ms_GET() == 3418362709L);
            assert(pack.available_capacity_GET() == 1.8318606E37F);
            assert(pack.image_status_GET() == (char)40);
            assert(pack.video_status_GET() == (char)184);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.available_capacity_SET(1.8318606E37F) ;
        p262.video_status_SET((char)184) ;
        p262.image_status_SET((char)40) ;
        p262.time_boot_ms_SET(3418362709L) ;
        p262.image_interval_SET(-2.56535E38F) ;
        p262.recording_time_ms_SET(3765020612L) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 224186329);
            assert(pack.alt_GET() == 1328504517);
            assert(pack.time_utc_GET() == 75469379699597778L);
            assert(pack.relative_alt_GET() == -1757894592);
            assert(pack.image_index_GET() == 514715552);
            assert(pack.time_boot_ms_GET() == 2530216798L);
            assert(pack.camera_id_GET() == (char)43);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-7.4877647E37F, -3.131819E37F, -2.5506043E38F, -9.690099E37F}));
            assert(pack.lon_GET() == -998675436);
            assert(pack.capture_result_GET() == (byte) - 118);
            assert(pack.file_url_LEN(ph) == 136);
            assert(pack.file_url_TRY(ph).equals("dsiKYgynwuGzxhfcboxtgBjpXhxinvoMhuqpunrrmLelcfsWczplrTddVubxxyhsgrmLgdCrqjosaNvyeaggtIsobsTuihepxaldshadjJaghtpVfwnwiejoYmutjgUsvrcjkjhd"));
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.lat_SET(224186329) ;
        p263.relative_alt_SET(-1757894592) ;
        p263.camera_id_SET((char)43) ;
        p263.capture_result_SET((byte) - 118) ;
        p263.file_url_SET("dsiKYgynwuGzxhfcboxtgBjpXhxinvoMhuqpunrrmLelcfsWczplrTddVubxxyhsgrmLgdCrqjosaNvyeaggtIsobsTuihepxaldshadjJaghtpVfwnwiejoYmutjgUsvrcjkjhd", PH) ;
        p263.q_SET(new float[] {-7.4877647E37F, -3.131819E37F, -2.5506043E38F, -9.690099E37F}, 0) ;
        p263.lon_SET(-998675436) ;
        p263.time_utc_SET(75469379699597778L) ;
        p263.image_index_SET(514715552) ;
        p263.alt_SET(1328504517) ;
        p263.time_boot_ms_SET(2530216798L) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.takeoff_time_utc_GET() == 2492920871615414772L);
            assert(pack.time_boot_ms_GET() == 449531047L);
            assert(pack.flight_uuid_GET() == 2910015563800469724L);
            assert(pack.arming_time_utc_GET() == 2901805565172070219L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.arming_time_utc_SET(2901805565172070219L) ;
        p264.flight_uuid_SET(2910015563800469724L) ;
        p264.takeoff_time_utc_SET(2492920871615414772L) ;
        p264.time_boot_ms_SET(449531047L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 1.8941848E38F);
            assert(pack.time_boot_ms_GET() == 1336838921L);
            assert(pack.yaw_GET() == -1.5081856E38F);
            assert(pack.roll_GET() == 6.7296376E37F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(6.7296376E37F) ;
        p265.yaw_SET(-1.5081856E38F) ;
        p265.pitch_SET(1.8941848E38F) ;
        p265.time_boot_ms_SET(1336838921L) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.first_message_offset_GET() == (char)13);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)217, (char)149, (char)66, (char)225, (char)59, (char)198, (char)255, (char)135, (char)76, (char)91, (char)207, (char)244, (char)54, (char)198, (char)153, (char)86, (char)136, (char)47, (char)162, (char)235, (char)133, (char)84, (char)236, (char)99, (char)221, (char)202, (char)155, (char)245, (char)232, (char)86, (char)57, (char)75, (char)229, (char)123, (char)157, (char)77, (char)84, (char)250, (char)240, (char)254, (char)54, (char)201, (char)67, (char)87, (char)97, (char)103, (char)129, (char)74, (char)128, (char)227, (char)238, (char)121, (char)177, (char)175, (char)79, (char)36, (char)98, (char)220, (char)21, (char)87, (char)186, (char)124, (char)65, (char)102, (char)103, (char)146, (char)124, (char)156, (char)14, (char)126, (char)220, (char)63, (char)183, (char)20, (char)233, (char)94, (char)176, (char)74, (char)69, (char)106, (char)169, (char)23, (char)87, (char)189, (char)125, (char)26, (char)126, (char)31, (char)192, (char)74, (char)118, (char)211, (char)188, (char)190, (char)3, (char)54, (char)224, (char)33, (char)204, (char)195, (char)35, (char)42, (char)86, (char)202, (char)213, (char)184, (char)86, (char)251, (char)150, (char)3, (char)47, (char)175, (char)10, (char)72, (char)90, (char)137, (char)113, (char)97, (char)47, (char)92, (char)35, (char)126, (char)153, (char)190, (char)93, (char)40, (char)255, (char)230, (char)69, (char)230, (char)119, (char)23, (char)96, (char)147, (char)143, (char)110, (char)211, (char)135, (char)109, (char)105, (char)95, (char)53, (char)136, (char)115, (char)6, (char)230, (char)68, (char)14, (char)240, (char)233, (char)162, (char)129, (char)125, (char)135, (char)58, (char)163, (char)122, (char)46, (char)87, (char)144, (char)38, (char)232, (char)250, (char)62, (char)124, (char)254, (char)16, (char)170, (char)208, (char)149, (char)33, (char)113, (char)162, (char)251, (char)142, (char)162, (char)51, (char)123, (char)146, (char)139, (char)135, (char)182, (char)218, (char)58, (char)164, (char)110, (char)227, (char)144, (char)146, (char)43, (char)213, (char)237, (char)97, (char)142, (char)1, (char)146, (char)12, (char)114, (char)233, (char)198, (char)38, (char)245, (char)53, (char)99, (char)5, (char)156, (char)208, (char)155, (char)217, (char)13, (char)0, (char)4, (char)143, (char)5, (char)193, (char)122, (char)54, (char)115, (char)68, (char)7, (char)254, (char)159, (char)239, (char)21, (char)186, (char)37, (char)102, (char)102, (char)2, (char)214, (char)66, (char)231, (char)199, (char)231, (char)68, (char)87, (char)138, (char)32, (char)144, (char)241, (char)59, (char)148, (char)39, (char)109, (char)212, (char)198, (char)199, (char)38, (char)151}));
            assert(pack.target_component_GET() == (char)197);
            assert(pack.length_GET() == (char)174);
            assert(pack.sequence_GET() == (char)58382);
            assert(pack.target_system_GET() == (char)65);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_component_SET((char)197) ;
        p266.length_SET((char)174) ;
        p266.first_message_offset_SET((char)13) ;
        p266.data__SET(new char[] {(char)217, (char)149, (char)66, (char)225, (char)59, (char)198, (char)255, (char)135, (char)76, (char)91, (char)207, (char)244, (char)54, (char)198, (char)153, (char)86, (char)136, (char)47, (char)162, (char)235, (char)133, (char)84, (char)236, (char)99, (char)221, (char)202, (char)155, (char)245, (char)232, (char)86, (char)57, (char)75, (char)229, (char)123, (char)157, (char)77, (char)84, (char)250, (char)240, (char)254, (char)54, (char)201, (char)67, (char)87, (char)97, (char)103, (char)129, (char)74, (char)128, (char)227, (char)238, (char)121, (char)177, (char)175, (char)79, (char)36, (char)98, (char)220, (char)21, (char)87, (char)186, (char)124, (char)65, (char)102, (char)103, (char)146, (char)124, (char)156, (char)14, (char)126, (char)220, (char)63, (char)183, (char)20, (char)233, (char)94, (char)176, (char)74, (char)69, (char)106, (char)169, (char)23, (char)87, (char)189, (char)125, (char)26, (char)126, (char)31, (char)192, (char)74, (char)118, (char)211, (char)188, (char)190, (char)3, (char)54, (char)224, (char)33, (char)204, (char)195, (char)35, (char)42, (char)86, (char)202, (char)213, (char)184, (char)86, (char)251, (char)150, (char)3, (char)47, (char)175, (char)10, (char)72, (char)90, (char)137, (char)113, (char)97, (char)47, (char)92, (char)35, (char)126, (char)153, (char)190, (char)93, (char)40, (char)255, (char)230, (char)69, (char)230, (char)119, (char)23, (char)96, (char)147, (char)143, (char)110, (char)211, (char)135, (char)109, (char)105, (char)95, (char)53, (char)136, (char)115, (char)6, (char)230, (char)68, (char)14, (char)240, (char)233, (char)162, (char)129, (char)125, (char)135, (char)58, (char)163, (char)122, (char)46, (char)87, (char)144, (char)38, (char)232, (char)250, (char)62, (char)124, (char)254, (char)16, (char)170, (char)208, (char)149, (char)33, (char)113, (char)162, (char)251, (char)142, (char)162, (char)51, (char)123, (char)146, (char)139, (char)135, (char)182, (char)218, (char)58, (char)164, (char)110, (char)227, (char)144, (char)146, (char)43, (char)213, (char)237, (char)97, (char)142, (char)1, (char)146, (char)12, (char)114, (char)233, (char)198, (char)38, (char)245, (char)53, (char)99, (char)5, (char)156, (char)208, (char)155, (char)217, (char)13, (char)0, (char)4, (char)143, (char)5, (char)193, (char)122, (char)54, (char)115, (char)68, (char)7, (char)254, (char)159, (char)239, (char)21, (char)186, (char)37, (char)102, (char)102, (char)2, (char)214, (char)66, (char)231, (char)199, (char)231, (char)68, (char)87, (char)138, (char)32, (char)144, (char)241, (char)59, (char)148, (char)39, (char)109, (char)212, (char)198, (char)199, (char)38, (char)151}, 0) ;
        p266.sequence_SET((char)58382) ;
        p266.target_system_SET((char)65) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.length_GET() == (char)243);
            assert(pack.target_component_GET() == (char)167);
            assert(pack.sequence_GET() == (char)41611);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)52, (char)169, (char)178, (char)37, (char)218, (char)143, (char)213, (char)219, (char)99, (char)182, (char)141, (char)206, (char)70, (char)252, (char)157, (char)71, (char)29, (char)46, (char)162, (char)151, (char)54, (char)85, (char)103, (char)89, (char)144, (char)134, (char)88, (char)181, (char)179, (char)1, (char)74, (char)169, (char)149, (char)89, (char)197, (char)212, (char)82, (char)61, (char)219, (char)225, (char)38, (char)116, (char)107, (char)193, (char)90, (char)217, (char)12, (char)114, (char)138, (char)44, (char)14, (char)26, (char)93, (char)56, (char)229, (char)11, (char)93, (char)30, (char)183, (char)10, (char)68, (char)107, (char)66, (char)53, (char)87, (char)222, (char)252, (char)208, (char)205, (char)250, (char)153, (char)16, (char)133, (char)192, (char)255, (char)147, (char)222, (char)87, (char)206, (char)85, (char)74, (char)193, (char)186, (char)62, (char)40, (char)32, (char)225, (char)2, (char)199, (char)165, (char)26, (char)84, (char)165, (char)233, (char)82, (char)24, (char)252, (char)86, (char)49, (char)48, (char)16, (char)89, (char)112, (char)89, (char)186, (char)72, (char)16, (char)243, (char)133, (char)128, (char)1, (char)10, (char)14, (char)239, (char)38, (char)147, (char)254, (char)118, (char)11, (char)243, (char)201, (char)245, (char)1, (char)51, (char)72, (char)196, (char)178, (char)90, (char)49, (char)160, (char)219, (char)38, (char)33, (char)134, (char)39, (char)210, (char)151, (char)242, (char)123, (char)16, (char)90, (char)141, (char)36, (char)18, (char)179, (char)26, (char)33, (char)188, (char)48, (char)13, (char)244, (char)217, (char)169, (char)249, (char)48, (char)147, (char)253, (char)238, (char)82, (char)175, (char)220, (char)11, (char)76, (char)101, (char)47, (char)0, (char)41, (char)220, (char)7, (char)168, (char)223, (char)159, (char)152, (char)0, (char)25, (char)150, (char)21, (char)160, (char)167, (char)20, (char)236, (char)143, (char)85, (char)82, (char)166, (char)44, (char)143, (char)215, (char)235, (char)194, (char)8, (char)208, (char)157, (char)118, (char)111, (char)10, (char)179, (char)85, (char)114, (char)13, (char)50, (char)177, (char)81, (char)7, (char)103, (char)149, (char)254, (char)210, (char)168, (char)41, (char)114, (char)128, (char)170, (char)227, (char)92, (char)106, (char)223, (char)183, (char)3, (char)204, (char)110, (char)233, (char)97, (char)104, (char)49, (char)90, (char)244, (char)163, (char)205, (char)3, (char)236, (char)39, (char)184, (char)107, (char)59, (char)219, (char)45, (char)153, (char)182, (char)237, (char)211, (char)181, (char)122, (char)47, (char)67, (char)141, (char)106, (char)204, (char)227}));
            assert(pack.first_message_offset_GET() == (char)94);
            assert(pack.target_system_GET() == (char)64);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.sequence_SET((char)41611) ;
        p267.data__SET(new char[] {(char)52, (char)169, (char)178, (char)37, (char)218, (char)143, (char)213, (char)219, (char)99, (char)182, (char)141, (char)206, (char)70, (char)252, (char)157, (char)71, (char)29, (char)46, (char)162, (char)151, (char)54, (char)85, (char)103, (char)89, (char)144, (char)134, (char)88, (char)181, (char)179, (char)1, (char)74, (char)169, (char)149, (char)89, (char)197, (char)212, (char)82, (char)61, (char)219, (char)225, (char)38, (char)116, (char)107, (char)193, (char)90, (char)217, (char)12, (char)114, (char)138, (char)44, (char)14, (char)26, (char)93, (char)56, (char)229, (char)11, (char)93, (char)30, (char)183, (char)10, (char)68, (char)107, (char)66, (char)53, (char)87, (char)222, (char)252, (char)208, (char)205, (char)250, (char)153, (char)16, (char)133, (char)192, (char)255, (char)147, (char)222, (char)87, (char)206, (char)85, (char)74, (char)193, (char)186, (char)62, (char)40, (char)32, (char)225, (char)2, (char)199, (char)165, (char)26, (char)84, (char)165, (char)233, (char)82, (char)24, (char)252, (char)86, (char)49, (char)48, (char)16, (char)89, (char)112, (char)89, (char)186, (char)72, (char)16, (char)243, (char)133, (char)128, (char)1, (char)10, (char)14, (char)239, (char)38, (char)147, (char)254, (char)118, (char)11, (char)243, (char)201, (char)245, (char)1, (char)51, (char)72, (char)196, (char)178, (char)90, (char)49, (char)160, (char)219, (char)38, (char)33, (char)134, (char)39, (char)210, (char)151, (char)242, (char)123, (char)16, (char)90, (char)141, (char)36, (char)18, (char)179, (char)26, (char)33, (char)188, (char)48, (char)13, (char)244, (char)217, (char)169, (char)249, (char)48, (char)147, (char)253, (char)238, (char)82, (char)175, (char)220, (char)11, (char)76, (char)101, (char)47, (char)0, (char)41, (char)220, (char)7, (char)168, (char)223, (char)159, (char)152, (char)0, (char)25, (char)150, (char)21, (char)160, (char)167, (char)20, (char)236, (char)143, (char)85, (char)82, (char)166, (char)44, (char)143, (char)215, (char)235, (char)194, (char)8, (char)208, (char)157, (char)118, (char)111, (char)10, (char)179, (char)85, (char)114, (char)13, (char)50, (char)177, (char)81, (char)7, (char)103, (char)149, (char)254, (char)210, (char)168, (char)41, (char)114, (char)128, (char)170, (char)227, (char)92, (char)106, (char)223, (char)183, (char)3, (char)204, (char)110, (char)233, (char)97, (char)104, (char)49, (char)90, (char)244, (char)163, (char)205, (char)3, (char)236, (char)39, (char)184, (char)107, (char)59, (char)219, (char)45, (char)153, (char)182, (char)237, (char)211, (char)181, (char)122, (char)47, (char)67, (char)141, (char)106, (char)204, (char)227}, 0) ;
        p267.target_system_SET((char)64) ;
        p267.first_message_offset_SET((char)94) ;
        p267.length_SET((char)243) ;
        p267.target_component_SET((char)167) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)88);
            assert(pack.target_system_GET() == (char)106);
            assert(pack.sequence_GET() == (char)49194);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_component_SET((char)88) ;
        p268.sequence_SET((char)49194) ;
        p268.target_system_SET((char)106) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.uri_LEN(ph) == 132);
            assert(pack.uri_TRY(ph).equals("mmoqjomskqqbisjqlvotoQmpsavorgvxFdxmviTpdbipgywGqyvtuylfivkwbivhGrvfhgxVwramxgvoitlsqhkjwsmwqkqwxtqvvtpEchwfizqrcpmndqtchkkUpqegqLfY"));
            assert(pack.resolution_h_GET() == (char)16969);
            assert(pack.rotation_GET() == (char)2165);
            assert(pack.status_GET() == (char)23);
            assert(pack.bitrate_GET() == 3217480157L);
            assert(pack.resolution_v_GET() == (char)34319);
            assert(pack.camera_id_GET() == (char)254);
            assert(pack.framerate_GET() == -2.5560057E37F);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)254) ;
        p269.bitrate_SET(3217480157L) ;
        p269.resolution_v_SET((char)34319) ;
        p269.framerate_SET(-2.5560057E37F) ;
        p269.resolution_h_SET((char)16969) ;
        p269.status_SET((char)23) ;
        p269.uri_SET("mmoqjomskqqbisjqlvotoQmpsavorgvxFdxmviTpdbipgywGqyvtuylfivkwbivhGrvfhgxVwramxgvoitlsqhkjwsmwqkqwxtqvvtpEchwfizqrcpmndqtchkkUpqegqLfY", PH) ;
        p269.rotation_SET((char)2165) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.uri_LEN(ph) == 191);
            assert(pack.uri_TRY(ph).equals("prcryrobalaCvbxrUqYvNxmnhsapqsmrxbmgjkotvIunogcvkgjrywewzymtqdpuqcvhovcyrwkgsnaddifycbjjrvxlbtlitsbelgGicttsnrkTuxpgwtuLjtezqhlbgnvVhmnhntjKlrsmhdkmdZnzxrwcJZtstpsxcdsybBzguKmAgatmyjsijkykgru"));
            assert(pack.resolution_h_GET() == (char)21612);
            assert(pack.rotation_GET() == (char)42064);
            assert(pack.framerate_GET() == 2.5472885E38F);
            assert(pack.bitrate_GET() == 3709447071L);
            assert(pack.camera_id_GET() == (char)240);
            assert(pack.target_system_GET() == (char)88);
            assert(pack.target_component_GET() == (char)179);
            assert(pack.resolution_v_GET() == (char)54275);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.camera_id_SET((char)240) ;
        p270.target_component_SET((char)179) ;
        p270.resolution_v_SET((char)54275) ;
        p270.target_system_SET((char)88) ;
        p270.framerate_SET(2.5472885E38F) ;
        p270.uri_SET("prcryrobalaCvbxrUqYvNxmnhsapqsmrxbmgjkotvIunogcvkgjrywewzymtqdpuqcvhovcyrwkgsnaddifycbjjrvxlbtlitsbelgGicttsnrkTuxpgwtuLjtezqhlbgnvVhmnhntjKlrsmhdkmdZnzxrwcJZtstpsxcdsybBzguKmAgatmyjsijkykgru", PH) ;
        p270.bitrate_SET(3709447071L) ;
        p270.rotation_SET((char)42064) ;
        p270.resolution_h_SET((char)21612) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 12);
            assert(pack.password_TRY(ph).equals("desqijrkVqra"));
            assert(pack.ssid_LEN(ph) == 17);
            assert(pack.ssid_TRY(ph).equals("uepzidlcavdOybbnG"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("desqijrkVqra", PH) ;
        p299.ssid_SET("uepzidlcavdOybbnG", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == (char)24162);
            assert(pack.min_version_GET() == (char)56452);
            assert(pack.max_version_GET() == (char)54735);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)252, (char)10, (char)151, (char)25, (char)61, (char)116, (char)84, (char)49}));
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)186, (char)16, (char)43, (char)180, (char)117, (char)240, (char)130, (char)126}));
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.spec_version_hash_SET(new char[] {(char)252, (char)10, (char)151, (char)25, (char)61, (char)116, (char)84, (char)49}, 0) ;
        p300.min_version_SET((char)56452) ;
        p300.version_SET((char)24162) ;
        p300.max_version_SET((char)54735) ;
        p300.library_version_hash_SET(new char[] {(char)186, (char)16, (char)43, (char)180, (char)117, (char)240, (char)130, (char)126}, 0) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
            assert(pack.sub_mode_GET() == (char)191);
            assert(pack.vendor_specific_status_code_GET() == (char)39496);
            assert(pack.time_usec_GET() == 6509156782067534632L);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
            assert(pack.uptime_sec_GET() == 3298209534L);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL) ;
        p310.vendor_specific_status_code_SET((char)39496) ;
        p310.time_usec_SET(6509156782067534632L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE) ;
        p310.sub_mode_SET((char)191) ;
        p310.uptime_sec_SET(3298209534L) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.sw_vcs_commit_GET() == 1188041326L);
            assert(pack.sw_version_minor_GET() == (char)244);
            assert(pack.name_LEN(ph) == 47);
            assert(pack.name_TRY(ph).equals("edvewbEeyputfAnRjofngvxuxtnjezPlqyrtjsmggbkklsH"));
            assert(pack.time_usec_GET() == 3604807550796763896L);
            assert(pack.hw_version_minor_GET() == (char)143);
            assert(pack.sw_version_major_GET() == (char)128);
            assert(pack.hw_version_major_GET() == (char)179);
            assert(pack.uptime_sec_GET() == 3563428174L);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)79, (char)141, (char)73, (char)96, (char)184, (char)193, (char)47, (char)50, (char)157, (char)15, (char)21, (char)191, (char)74, (char)148, (char)32, (char)255}));
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_version_minor_SET((char)143) ;
        p311.hw_version_major_SET((char)179) ;
        p311.sw_version_major_SET((char)128) ;
        p311.time_usec_SET(3604807550796763896L) ;
        p311.uptime_sec_SET(3563428174L) ;
        p311.sw_version_minor_SET((char)244) ;
        p311.hw_unique_id_SET(new char[] {(char)79, (char)141, (char)73, (char)96, (char)184, (char)193, (char)47, (char)50, (char)157, (char)15, (char)21, (char)191, (char)74, (char)148, (char)32, (char)255}, 0) ;
        p311.name_SET("edvewbEeyputfAnRjofngvxuxtnjezPlqyrtjsmggbkklsH", PH) ;
        p311.sw_vcs_commit_SET(1188041326L) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)151);
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("ndielpl"));
            assert(pack.target_component_GET() == (char)164);
            assert(pack.param_index_GET() == (short) -5437);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)151) ;
        p320.target_component_SET((char)164) ;
        p320.param_index_SET((short) -5437) ;
        p320.param_id_SET("ndielpl", PH) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)58);
            assert(pack.target_component_GET() == (char)66);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)66) ;
        p321.target_system_SET((char)58) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 109);
            assert(pack.param_value_TRY(ph).equals("dqwixwvysrthNdqqymmixhzoiqwawPkjhyWwqhminnmhuouvsxcwooecbxzcuprycolmrsiUdcjvadpatTmbPAgpapjrbgwnvqyjqxxxstElj"));
            assert(pack.param_count_GET() == (char)2773);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("xm"));
            assert(pack.param_index_GET() == (char)39926);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("xm", PH) ;
        p322.param_index_SET((char)39926) ;
        p322.param_value_SET("dqwixwvysrthNdqqymmixhzoiqwawPkjhyWwqhminnmhuouvsxcwooecbxzcuprycolmrsiUdcjvadpatTmbPAgpapjrbgwnvqyjqxxxstElj", PH) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16) ;
        p322.param_count_SET((char)2773) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)239);
            assert(pack.target_system_GET() == (char)237);
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("gmmBeziPf"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
            assert(pack.param_value_LEN(ph) == 103);
            assert(pack.param_value_TRY(ph).equals("djtqogcdbsuynCFfnapnwJceDbudpscfusdgHsdwsziakmpBcyhrsbeCsaggolxisovofibvljEMvrgsifpkpyayVuutwlmlZfrfluk"));
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16) ;
        p323.target_system_SET((char)237) ;
        p323.target_component_SET((char)239) ;
        p323.param_value_SET("djtqogcdbsuynCFfnapnwJceDbudpscfusdgHsdwsziakmpBcyhrsbeCsaggolxisovofibvljEMvrgsifpkpyayVuutwlmlZfrfluk", PH) ;
        p323.param_id_SET("gmmBeziPf", PH) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 4);
            assert(pack.param_value_TRY(ph).equals("Fhdr"));
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("rz"));
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_IN_PROGRESS);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("rz", PH) ;
        p324.param_value_SET("Fhdr", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_IN_PROGRESS) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.increment_GET() == (char)214);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)1232, (char)107, (char)32467, (char)32096, (char)2040, (char)16147, (char)21006, (char)22909, (char)28381, (char)2109, (char)33080, (char)30977, (char)33897, (char)23517, (char)590, (char)12676, (char)7051, (char)26215, (char)10051, (char)18106, (char)7434, (char)38751, (char)43869, (char)3288, (char)22262, (char)60498, (char)633, (char)40118, (char)50046, (char)50352, (char)47498, (char)8580, (char)3015, (char)57271, (char)27055, (char)62551, (char)16521, (char)21304, (char)64545, (char)47534, (char)43899, (char)49308, (char)4725, (char)54256, (char)64263, (char)39827, (char)51444, (char)14847, (char)39300, (char)48184, (char)57390, (char)4888, (char)64464, (char)25440, (char)3559, (char)14499, (char)46268, (char)28720, (char)54091, (char)6899, (char)13767, (char)38978, (char)48036, (char)23897, (char)63651, (char)33136, (char)6822, (char)15085, (char)23580, (char)7169, (char)40191, (char)26491}));
            assert(pack.min_distance_GET() == (char)52967);
            assert(pack.max_distance_GET() == (char)57793);
            assert(pack.time_usec_GET() == 1944389489883777162L);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.distances_SET(new char[] {(char)1232, (char)107, (char)32467, (char)32096, (char)2040, (char)16147, (char)21006, (char)22909, (char)28381, (char)2109, (char)33080, (char)30977, (char)33897, (char)23517, (char)590, (char)12676, (char)7051, (char)26215, (char)10051, (char)18106, (char)7434, (char)38751, (char)43869, (char)3288, (char)22262, (char)60498, (char)633, (char)40118, (char)50046, (char)50352, (char)47498, (char)8580, (char)3015, (char)57271, (char)27055, (char)62551, (char)16521, (char)21304, (char)64545, (char)47534, (char)43899, (char)49308, (char)4725, (char)54256, (char)64263, (char)39827, (char)51444, (char)14847, (char)39300, (char)48184, (char)57390, (char)4888, (char)64464, (char)25440, (char)3559, (char)14499, (char)46268, (char)28720, (char)54091, (char)6899, (char)13767, (char)38978, (char)48036, (char)23897, (char)63651, (char)33136, (char)6822, (char)15085, (char)23580, (char)7169, (char)40191, (char)26491}, 0) ;
        p330.max_distance_SET((char)57793) ;
        p330.min_distance_SET((char)52967) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p330.time_usec_SET(1944389489883777162L) ;
        p330.increment_SET((char)214) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
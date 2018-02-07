
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
            long id = id__w(src);
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
                case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                    id = 3;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 4;
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
                case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                    id = 3;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 4;
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
            long id = id__D(src);
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
                case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                    id = 3;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 4;
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
                case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                    id = 3;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 4;
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
                case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                    id = 3;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 4;
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
                case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                    id = 3;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 4;
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
                case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                    id = 3;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 4;
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
                case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                    id = 3;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 4;
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
                case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                    id = 3;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 4;
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
            long id = id__D(src);
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
                case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                    id = 3;
                    break;
                case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                    id = 4;
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
    public static class AQ_TELEMETRY_F extends GroundControl.AQ_TELEMETRY_F
    {
        public char Index_GET()//Index of message
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public float value1_GET()//value1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  2, 4))); }
        public float value2_GET()//value2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public float value3_GET()//value3
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public float value4_GET()//value4
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public float value5_GET()//value5
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public float value6_GET()//value6
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public float value7_GET()//value7
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  26, 4))); }
        public float value8_GET()//value8
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  30, 4))); }
        public float value9_GET()//value9
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  34, 4))); }
        public float value10_GET()//value10
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  38, 4))); }
        public float value11_GET()//value11
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  42, 4))); }
        public float value12_GET()//value12
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  46, 4))); }
        public float value13_GET()//value13
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  50, 4))); }
        public float value14_GET()//value14
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  54, 4))); }
        public float value15_GET()//value15
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  58, 4))); }
        public float value16_GET()//value16
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  62, 4))); }
        public float value17_GET()//value17
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  66, 4))); }
        public float value18_GET()//value18
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  70, 4))); }
        public float value19_GET()//value19
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  74, 4))); }
        public float value20_GET()//value20
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  78, 4))); }
    }
    public static class AQ_ESC_TELEMETRY extends GroundControl.AQ_ESC_TELEMETRY
    {
        public char[] status_age_GET(char[]  dst_ch, int pos)  //Age of each ESC telemetry reading in ms compared to boot time. A value of 0xFFFF means timeout/no data
        {
            for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] status_age_GET()//Age of each ESC telemetry reading in ms compared to boot time. A value of 0xFFFF means timeout/no data
        {return status_age_GET(new char[4], 0);} public long time_boot_ms_GET()//Timestamp of the component clock since boot time in ms.
        {  return (get_bytes(data,  8, 4)); }
        public long[] data0_GET(long[]  dst_ch, int pos)  //Data bits 1-32 for each ESC.
        {
            for(int BYTE = 12, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (get_bytes(data,  BYTE, 4));
            return dst_ch;
        }
        public long[] data0_GET()//Data bits 1-32 for each ESC.
        {return data0_GET(new long[4], 0);} public long[] data1_GET(long[]  dst_ch, int pos)  //Data bits 33-64 for each ESC.
        {
            for(int BYTE = 28, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (get_bytes(data,  BYTE, 4));
            return dst_ch;
        }
        public long[] data1_GET()//Data bits 33-64 for each ESC.
        {return data1_GET(new long[4], 0);} public char seq_GET()//Sequence number of message (first set of 4 motors is #1, next 4 is #2, etc).
        {  return (char)((char) get_bytes(data,  44, 1)); }
        public char num_motors_GET()//Total number of active ESCs/motors on the system.
        {  return (char)((char) get_bytes(data,  45, 1)); }
        public char num_in_seq_GET()//Number of active ESCs in this sequence (1 through this many array members will be populated with data
        {  return (char)((char) get_bytes(data,  46, 1)); }
        public char[] escid_GET(char[]  dst_ch, int pos)  //ESC/Motor ID
        {
            for(int BYTE = 47, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] escid_GET()//ESC/Motor ID
        {return escid_GET(new char[4], 0);} public char[] data_version_GET(char[]  dst_ch, int pos)  //Version of data structure (determines contents).
        {
            for(int BYTE = 51, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data_version_GET()//Version of data structure (determines contents).
        {return data_version_GET(new char[4], 0);}
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
        {  return  0 + (int)get_bits(data, 132, 2); }
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

        static final Collection<OnReceive.Handler<ACTUATOR_CONTROL_TARGET, Channel>> on_ACTUATOR_CONTROL_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<ALTITUDE, Channel>> on_ALTITUDE = new OnReceive<>();
        static final Collection<OnReceive.Handler<RESOURCE_REQUEST, Channel>> on_RESOURCE_REQUEST = new OnReceive<>();
        static final Collection<OnReceive.Handler<SCALED_PRESSURE3, Channel>> on_SCALED_PRESSURE3 = new OnReceive<>();
        static final Collection<OnReceive.Handler<FOLLOW_TARGET, Channel>> on_FOLLOW_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<CONTROL_SYSTEM_STATE, Channel>> on_CONTROL_SYSTEM_STATE = new OnReceive<>();
        static final Collection<OnReceive.Handler<BATTERY_STATUS, Channel>> on_BATTERY_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<AUTOPILOT_VERSION, Channel>> on_AUTOPILOT_VERSION = new OnReceive<>();
        static final Collection<OnReceive.Handler<LANDING_TARGET, Channel>> on_LANDING_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<AQ_TELEMETRY_F, Channel>> on_AQ_TELEMETRY_F = new OnReceive<>();
        static final Collection<OnReceive.Handler<AQ_ESC_TELEMETRY, Channel>> on_AQ_ESC_TELEMETRY = new OnReceive<>();
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
                case 150:
                    if(pack == null) return new AQ_TELEMETRY_F();
                    ((OnReceive) on_AQ_TELEMETRY_F).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 152:
                    if(pack == null) return new AQ_ESC_TELEMETRY();
                    ((OnReceive) on_AQ_ESC_TELEMETRY).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
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
            assert(pack.custom_mode_GET() == 3171895137L);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED));
            assert(pack.mavlink_version_GET() == (char)214);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_STANDBY);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_GIMBAL);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_ASLUAV);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED)) ;
        p0.mavlink_version_SET((char)214) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_GIMBAL) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_ASLUAV) ;
        p0.custom_mode_SET(3171895137L) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_STANDBY) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_count4_GET() == (char)53527);
            assert(pack.current_battery_GET() == (short) -30202);
            assert(pack.errors_comm_GET() == (char)50597);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY));
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY));
            assert(pack.errors_count1_GET() == (char)45972);
            assert(pack.voltage_battery_GET() == (char)8333);
            assert(pack.errors_count3_GET() == (char)61677);
            assert(pack.battery_remaining_GET() == (byte) - 98);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.errors_count2_GET() == (char)44167);
            assert(pack.drop_rate_comm_GET() == (char)43547);
            assert(pack.load_GET() == (char)9536);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.battery_remaining_SET((byte) - 98) ;
        p1.errors_count1_SET((char)45972) ;
        p1.current_battery_SET((short) -30202) ;
        p1.errors_count3_SET((char)61677) ;
        p1.drop_rate_comm_SET((char)43547) ;
        p1.load_SET((char)9536) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.errors_count2_SET((char)44167) ;
        p1.errors_count4_SET((char)53527) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY)) ;
        p1.voltage_battery_SET((char)8333) ;
        p1.errors_comm_SET((char)50597) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY)) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2589285687L);
            assert(pack.time_unix_usec_GET() == 2474419278091286394L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(2474419278091286394L) ;
        p2.time_boot_ms_SET(2589285687L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.afy_GET() == 1.5007E37F);
            assert(pack.vy_GET() == -2.441492E38F);
            assert(pack.yaw_rate_GET() == 3.0093356E38F);
            assert(pack.y_GET() == -2.7611628E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.time_boot_ms_GET() == 1510984872L);
            assert(pack.afz_GET() == 1.7825287E38F);
            assert(pack.type_mask_GET() == (char)29496);
            assert(pack.vz_GET() == 1.562699E38F);
            assert(pack.z_GET() == 2.9050057E38F);
            assert(pack.afx_GET() == 3.2281986E38F);
            assert(pack.yaw_GET() == -2.627126E38F);
            assert(pack.x_GET() == 2.862273E38F);
            assert(pack.vx_GET() == 1.6858627E38F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.x_SET(2.862273E38F) ;
        p3.vx_SET(1.6858627E38F) ;
        p3.afy_SET(1.5007E37F) ;
        p3.afz_SET(1.7825287E38F) ;
        p3.afx_SET(3.2281986E38F) ;
        p3.vy_SET(-2.441492E38F) ;
        p3.z_SET(2.9050057E38F) ;
        p3.yaw_SET(-2.627126E38F) ;
        p3.type_mask_SET((char)29496) ;
        p3.vz_SET(1.562699E38F) ;
        p3.yaw_rate_SET(3.0093356E38F) ;
        p3.y_SET(-2.7611628E37F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p3.time_boot_ms_SET(1510984872L) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 1525242640L);
            assert(pack.target_system_GET() == (char)55);
            assert(pack.target_component_GET() == (char)103);
            assert(pack.time_usec_GET() == 3412778881775694400L);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.seq_SET(1525242640L) ;
        p4.time_usec_SET(3412778881775694400L) ;
        p4.target_system_SET((char)55) ;
        p4.target_component_SET((char)103) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)170);
            assert(pack.target_system_GET() == (char)60);
            assert(pack.passkey_LEN(ph) == 11);
            assert(pack.passkey_TRY(ph).equals("DhnYjpfjQxe"));
            assert(pack.version_GET() == (char)218);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.version_SET((char)218) ;
        p5.passkey_SET("DhnYjpfjQxe", PH) ;
        p5.control_request_SET((char)170) ;
        p5.target_system_SET((char)60) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.ack_GET() == (char)238);
            assert(pack.gcs_system_id_GET() == (char)128);
            assert(pack.control_request_GET() == (char)255);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.ack_SET((char)238) ;
        p6.gcs_system_id_SET((char)128) ;
        p6.control_request_SET((char)255) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 32);
            assert(pack.key_TRY(ph).equals("BuLplPiQwasyjumngCsaxcrvdpfnspcf"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("BuLplPiQwasyjumngCsaxcrvdpfnspcf", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)65);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_TEST_ARMED);
            assert(pack.custom_mode_GET() == 1400055140L);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.custom_mode_SET(1400055140L) ;
        p11.target_system_SET((char)65) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("cuep"));
            assert(pack.target_component_GET() == (char)93);
            assert(pack.param_index_GET() == (short)20989);
            assert(pack.target_system_GET() == (char)134);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_id_SET("cuep", PH) ;
        p20.target_component_SET((char)93) ;
        p20.target_system_SET((char)134) ;
        p20.param_index_SET((short)20989) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)80);
            assert(pack.target_component_GET() == (char)49);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)80) ;
        p21.target_component_SET((char)49) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_GET() == -2.6681522E38F);
            assert(pack.param_count_GET() == (char)36715);
            assert(pack.param_index_GET() == (char)3080);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("k"));
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_value_SET(-2.6681522E38F) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8) ;
        p22.param_count_SET((char)36715) ;
        p22.param_index_SET((char)3080) ;
        p22.param_id_SET("k", PH) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_value_GET() == -1.5306027E38F);
            assert(pack.target_component_GET() == (char)102);
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("oixxgrlBcph"));
            assert(pack.target_system_GET() == (char)84);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.param_id_SET("oixxgrlBcph", PH) ;
        p23.target_component_SET((char)102) ;
        p23.target_system_SET((char)84) ;
        p23.param_value_SET(-1.5306027E38F) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5005165301029312860L);
            assert(pack.h_acc_TRY(ph) == 23881368L);
            assert(pack.eph_GET() == (char)30993);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
            assert(pack.alt_GET() == 1988358280);
            assert(pack.cog_GET() == (char)62703);
            assert(pack.alt_ellipsoid_TRY(ph) == 435346779);
            assert(pack.vel_acc_TRY(ph) == 362066881L);
            assert(pack.lat_GET() == -1387963505);
            assert(pack.lon_GET() == -536988870);
            assert(pack.epv_GET() == (char)47913);
            assert(pack.v_acc_TRY(ph) == 2973056373L);
            assert(pack.vel_GET() == (char)58148);
            assert(pack.hdg_acc_TRY(ph) == 4044287151L);
            assert(pack.satellites_visible_GET() == (char)41);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.hdg_acc_SET(4044287151L, PH) ;
        p24.vel_acc_SET(362066881L, PH) ;
        p24.satellites_visible_SET((char)41) ;
        p24.alt_SET(1988358280) ;
        p24.alt_ellipsoid_SET(435346779, PH) ;
        p24.eph_SET((char)30993) ;
        p24.v_acc_SET(2973056373L, PH) ;
        p24.h_acc_SET(23881368L, PH) ;
        p24.lat_SET(-1387963505) ;
        p24.vel_SET((char)58148) ;
        p24.lon_SET(-536988870) ;
        p24.epv_SET((char)47913) ;
        p24.cog_SET((char)62703) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX) ;
        p24.time_usec_SET(5005165301029312860L) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)165, (char)121, (char)78, (char)98, (char)80, (char)232, (char)199, (char)183, (char)207, (char)41, (char)1, (char)196, (char)160, (char)16, (char)238, (char)101, (char)84, (char)197, (char)148, (char)19}));
            assert(pack.satellites_visible_GET() == (char)137);
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)198, (char)170, (char)146, (char)207, (char)246, (char)145, (char)8, (char)165, (char)246, (char)57, (char)93, (char)80, (char)161, (char)242, (char)244, (char)236, (char)197, (char)64, (char)21, (char)72}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)80, (char)134, (char)198, (char)53, (char)102, (char)110, (char)145, (char)33, (char)29, (char)247, (char)245, (char)233, (char)120, (char)128, (char)21, (char)84, (char)155, (char)156, (char)215, (char)178}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)95, (char)165, (char)176, (char)12, (char)73, (char)128, (char)127, (char)177, (char)66, (char)43, (char)249, (char)212, (char)18, (char)175, (char)124, (char)233, (char)84, (char)33, (char)129, (char)230}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)250, (char)43, (char)23, (char)97, (char)62, (char)36, (char)123, (char)196, (char)110, (char)241, (char)203, (char)196, (char)20, (char)30, (char)159, (char)32, (char)235, (char)1, (char)68, (char)159}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)137) ;
        p25.satellite_azimuth_SET(new char[] {(char)250, (char)43, (char)23, (char)97, (char)62, (char)36, (char)123, (char)196, (char)110, (char)241, (char)203, (char)196, (char)20, (char)30, (char)159, (char)32, (char)235, (char)1, (char)68, (char)159}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)80, (char)134, (char)198, (char)53, (char)102, (char)110, (char)145, (char)33, (char)29, (char)247, (char)245, (char)233, (char)120, (char)128, (char)21, (char)84, (char)155, (char)156, (char)215, (char)178}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)198, (char)170, (char)146, (char)207, (char)246, (char)145, (char)8, (char)165, (char)246, (char)57, (char)93, (char)80, (char)161, (char)242, (char)244, (char)236, (char)197, (char)64, (char)21, (char)72}, 0) ;
        p25.satellite_used_SET(new char[] {(char)95, (char)165, (char)176, (char)12, (char)73, (char)128, (char)127, (char)177, (char)66, (char)43, (char)249, (char)212, (char)18, (char)175, (char)124, (char)233, (char)84, (char)33, (char)129, (char)230}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)165, (char)121, (char)78, (char)98, (char)80, (char)232, (char)199, (char)183, (char)207, (char)41, (char)1, (char)196, (char)160, (char)16, (char)238, (char)101, (char)84, (char)197, (char)148, (char)19}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short) -15418);
            assert(pack.xgyro_GET() == (short) -9719);
            assert(pack.time_boot_ms_GET() == 228869605L);
            assert(pack.ymag_GET() == (short)21439);
            assert(pack.yacc_GET() == (short) -1137);
            assert(pack.zmag_GET() == (short)16969);
            assert(pack.xmag_GET() == (short) -26621);
            assert(pack.ygyro_GET() == (short)10821);
            assert(pack.xacc_GET() == (short) -23461);
            assert(pack.zgyro_GET() == (short)31711);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.xgyro_SET((short) -9719) ;
        p26.ygyro_SET((short)10821) ;
        p26.zgyro_SET((short)31711) ;
        p26.zacc_SET((short) -15418) ;
        p26.time_boot_ms_SET(228869605L) ;
        p26.zmag_SET((short)16969) ;
        p26.xmag_SET((short) -26621) ;
        p26.yacc_SET((short) -1137) ;
        p26.ymag_SET((short)21439) ;
        p26.xacc_SET((short) -23461) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short) -23209);
            assert(pack.xgyro_GET() == (short) -19771);
            assert(pack.zacc_GET() == (short) -18047);
            assert(pack.xacc_GET() == (short) -2989);
            assert(pack.zgyro_GET() == (short) -27744);
            assert(pack.xmag_GET() == (short)21229);
            assert(pack.zmag_GET() == (short) -2029);
            assert(pack.yacc_GET() == (short)4219);
            assert(pack.ymag_GET() == (short)25653);
            assert(pack.time_usec_GET() == 1845413237793922410L);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.ygyro_SET((short) -23209) ;
        p27.xacc_SET((short) -2989) ;
        p27.ymag_SET((short)25653) ;
        p27.zacc_SET((short) -18047) ;
        p27.xgyro_SET((short) -19771) ;
        p27.xmag_SET((short)21229) ;
        p27.zgyro_SET((short) -27744) ;
        p27.yacc_SET((short)4219) ;
        p27.time_usec_SET(1845413237793922410L) ;
        p27.zmag_SET((short) -2029) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -14504);
            assert(pack.press_diff2_GET() == (short) -547);
            assert(pack.press_diff1_GET() == (short)7703);
            assert(pack.time_usec_GET() == 7612275240471705875L);
            assert(pack.press_abs_GET() == (short) -6419);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_abs_SET((short) -6419) ;
        p28.press_diff2_SET((short) -547) ;
        p28.time_usec_SET(7612275240471705875L) ;
        p28.press_diff1_SET((short)7703) ;
        p28.temperature_SET((short) -14504) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3867595710L);
            assert(pack.temperature_GET() == (short) -28502);
            assert(pack.press_abs_GET() == 4.3940476E37F);
            assert(pack.press_diff_GET() == 3.3169295E38F);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.temperature_SET((short) -28502) ;
        p29.press_abs_SET(4.3940476E37F) ;
        p29.press_diff_SET(3.3169295E38F) ;
        p29.time_boot_ms_SET(3867595710L) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -2.4716043E38F);
            assert(pack.yaw_GET() == 1.451896E37F);
            assert(pack.rollspeed_GET() == 2.6550222E38F);
            assert(pack.pitch_GET() == 2.2171597E37F);
            assert(pack.time_boot_ms_GET() == 2139689561L);
            assert(pack.pitchspeed_GET() == -3.0401598E38F);
            assert(pack.yawspeed_GET() == -1.453056E38F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.pitch_SET(2.2171597E37F) ;
        p30.pitchspeed_SET(-3.0401598E38F) ;
        p30.roll_SET(-2.4716043E38F) ;
        p30.yawspeed_SET(-1.453056E38F) ;
        p30.rollspeed_SET(2.6550222E38F) ;
        p30.yaw_SET(1.451896E37F) ;
        p30.time_boot_ms_SET(2139689561L) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 3.125273E37F);
            assert(pack.q4_GET() == 3.3779094E38F);
            assert(pack.rollspeed_GET() == -1.6021845E38F);
            assert(pack.q3_GET() == 1.2841507E37F);
            assert(pack.q2_GET() == -2.5639511E38F);
            assert(pack.time_boot_ms_GET() == 2651876222L);
            assert(pack.q1_GET() == -2.0927062E38F);
            assert(pack.pitchspeed_GET() == 1.9239813E37F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q3_SET(1.2841507E37F) ;
        p31.rollspeed_SET(-1.6021845E38F) ;
        p31.pitchspeed_SET(1.9239813E37F) ;
        p31.time_boot_ms_SET(2651876222L) ;
        p31.q4_SET(3.3779094E38F) ;
        p31.q1_SET(-2.0927062E38F) ;
        p31.yawspeed_SET(3.125273E37F) ;
        p31.q2_SET(-2.5639511E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -2.3107741E38F);
            assert(pack.vx_GET() == 2.0279663E38F);
            assert(pack.x_GET() == 1.2738764E37F);
            assert(pack.time_boot_ms_GET() == 1659377656L);
            assert(pack.z_GET() == -2.8553412E38F);
            assert(pack.vz_GET() == 9.162561E37F);
            assert(pack.vy_GET() == 4.666214E37F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.vz_SET(9.162561E37F) ;
        p32.vy_SET(4.666214E37F) ;
        p32.y_SET(-2.3107741E38F) ;
        p32.z_SET(-2.8553412E38F) ;
        p32.x_SET(1.2738764E37F) ;
        p32.vx_SET(2.0279663E38F) ;
        p32.time_boot_ms_SET(1659377656L) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.relative_alt_GET() == 147600197);
            assert(pack.hdg_GET() == (char)25000);
            assert(pack.lon_GET() == -452208990);
            assert(pack.vz_GET() == (short) -3967);
            assert(pack.time_boot_ms_GET() == 3416376358L);
            assert(pack.vx_GET() == (short)14738);
            assert(pack.vy_GET() == (short) -5775);
            assert(pack.lat_GET() == -992392704);
            assert(pack.alt_GET() == -1471284976);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.vz_SET((short) -3967) ;
        p33.lat_SET(-992392704) ;
        p33.vy_SET((short) -5775) ;
        p33.time_boot_ms_SET(3416376358L) ;
        p33.lon_SET(-452208990) ;
        p33.alt_SET(-1471284976) ;
        p33.relative_alt_SET(147600197) ;
        p33.hdg_SET((char)25000) ;
        p33.vx_SET((short)14738) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan1_scaled_GET() == (short)23644);
            assert(pack.chan5_scaled_GET() == (short)17358);
            assert(pack.chan6_scaled_GET() == (short) -21129);
            assert(pack.chan2_scaled_GET() == (short)15844);
            assert(pack.port_GET() == (char)194);
            assert(pack.time_boot_ms_GET() == 3023589155L);
            assert(pack.chan7_scaled_GET() == (short)22396);
            assert(pack.chan8_scaled_GET() == (short)11694);
            assert(pack.chan3_scaled_GET() == (short) -21990);
            assert(pack.rssi_GET() == (char)123);
            assert(pack.chan4_scaled_GET() == (short)18290);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.rssi_SET((char)123) ;
        p34.chan4_scaled_SET((short)18290) ;
        p34.chan7_scaled_SET((short)22396) ;
        p34.chan3_scaled_SET((short) -21990) ;
        p34.chan1_scaled_SET((short)23644) ;
        p34.port_SET((char)194) ;
        p34.chan5_scaled_SET((short)17358) ;
        p34.chan8_scaled_SET((short)11694) ;
        p34.chan6_scaled_SET((short) -21129) ;
        p34.chan2_scaled_SET((short)15844) ;
        p34.time_boot_ms_SET(3023589155L) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)203);
            assert(pack.chan3_raw_GET() == (char)19325);
            assert(pack.chan4_raw_GET() == (char)26554);
            assert(pack.chan8_raw_GET() == (char)26782);
            assert(pack.chan5_raw_GET() == (char)35970);
            assert(pack.chan6_raw_GET() == (char)11738);
            assert(pack.chan1_raw_GET() == (char)55587);
            assert(pack.chan7_raw_GET() == (char)60651);
            assert(pack.chan2_raw_GET() == (char)51629);
            assert(pack.time_boot_ms_GET() == 2602135977L);
            assert(pack.port_GET() == (char)34);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.time_boot_ms_SET(2602135977L) ;
        p35.port_SET((char)34) ;
        p35.chan5_raw_SET((char)35970) ;
        p35.chan1_raw_SET((char)55587) ;
        p35.chan7_raw_SET((char)60651) ;
        p35.chan8_raw_SET((char)26782) ;
        p35.chan2_raw_SET((char)51629) ;
        p35.chan6_raw_SET((char)11738) ;
        p35.rssi_SET((char)203) ;
        p35.chan4_raw_SET((char)26554) ;
        p35.chan3_raw_SET((char)19325) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2557647966L);
            assert(pack.servo9_raw_TRY(ph) == (char)47796);
            assert(pack.servo14_raw_TRY(ph) == (char)17767);
            assert(pack.servo2_raw_GET() == (char)41204);
            assert(pack.servo8_raw_GET() == (char)37323);
            assert(pack.servo4_raw_GET() == (char)56822);
            assert(pack.servo11_raw_TRY(ph) == (char)33361);
            assert(pack.servo7_raw_GET() == (char)7775);
            assert(pack.servo6_raw_GET() == (char)24967);
            assert(pack.servo3_raw_GET() == (char)16266);
            assert(pack.servo10_raw_TRY(ph) == (char)6972);
            assert(pack.servo12_raw_TRY(ph) == (char)43168);
            assert(pack.servo15_raw_TRY(ph) == (char)35931);
            assert(pack.servo16_raw_TRY(ph) == (char)50886);
            assert(pack.servo1_raw_GET() == (char)2255);
            assert(pack.servo5_raw_GET() == (char)51466);
            assert(pack.servo13_raw_TRY(ph) == (char)51007);
            assert(pack.port_GET() == (char)253);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo8_raw_SET((char)37323) ;
        p36.port_SET((char)253) ;
        p36.servo14_raw_SET((char)17767, PH) ;
        p36.servo2_raw_SET((char)41204) ;
        p36.time_usec_SET(2557647966L) ;
        p36.servo13_raw_SET((char)51007, PH) ;
        p36.servo9_raw_SET((char)47796, PH) ;
        p36.servo16_raw_SET((char)50886, PH) ;
        p36.servo11_raw_SET((char)33361, PH) ;
        p36.servo4_raw_SET((char)56822) ;
        p36.servo1_raw_SET((char)2255) ;
        p36.servo6_raw_SET((char)24967) ;
        p36.servo7_raw_SET((char)7775) ;
        p36.servo5_raw_SET((char)51466) ;
        p36.servo3_raw_SET((char)16266) ;
        p36.servo12_raw_SET((char)43168, PH) ;
        p36.servo15_raw_SET((char)35931, PH) ;
        p36.servo10_raw_SET((char)6972, PH) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)243);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION);
            assert(pack.start_index_GET() == (short) -2171);
            assert(pack.end_index_GET() == (short) -14031);
            assert(pack.target_system_GET() == (char)122);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION) ;
        p37.target_system_SET((char)122) ;
        p37.target_component_SET((char)243) ;
        p37.start_index_SET((short) -2171) ;
        p37.end_index_SET((short) -14031) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)251);
            assert(pack.start_index_GET() == (short) -18733);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.end_index_GET() == (short) -24935);
            assert(pack.target_system_GET() == (char)89);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_component_SET((char)251) ;
        p38.start_index_SET((short) -18733) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p38.target_system_SET((char)89) ;
        p38.end_index_SET((short) -24935) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)87);
            assert(pack.param1_GET() == -2.1740764E38F);
            assert(pack.seq_GET() == (char)62525);
            assert(pack.target_system_GET() == (char)123);
            assert(pack.autocontinue_GET() == (char)56);
            assert(pack.current_GET() == (char)62);
            assert(pack.x_GET() == 9.295447E37F);
            assert(pack.y_GET() == -1.7998793E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.param2_GET() == -2.715565E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.param3_GET() == -9.717585E37F);
            assert(pack.param4_GET() == -9.904689E37F);
            assert(pack.z_GET() == 2.8493015E38F);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.x_SET(9.295447E37F) ;
        p39.param4_SET(-9.904689E37F) ;
        p39.target_system_SET((char)123) ;
        p39.autocontinue_SET((char)56) ;
        p39.target_component_SET((char)87) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p39.param3_SET(-9.717585E37F) ;
        p39.current_SET((char)62) ;
        p39.command_SET(MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p39.param2_SET(-2.715565E38F) ;
        p39.y_SET(-1.7998793E38F) ;
        p39.z_SET(2.8493015E38F) ;
        p39.param1_SET(-2.1740764E38F) ;
        p39.seq_SET((char)62525) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.seq_GET() == (char)48041);
            assert(pack.target_component_GET() == (char)190);
            assert(pack.target_system_GET() == (char)186);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.seq_SET((char)48041) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p40.target_component_SET((char)190) ;
        p40.target_system_SET((char)186) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)244);
            assert(pack.target_component_GET() == (char)73);
            assert(pack.seq_GET() == (char)7699);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_component_SET((char)73) ;
        p41.target_system_SET((char)244) ;
        p41.seq_SET((char)7699) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)33939);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)33939) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)51);
            assert(pack.target_system_GET() == (char)133);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)133) ;
        p43.target_component_SET((char)51) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)33843);
            assert(pack.target_system_GET() == (char)168);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)155);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)168) ;
        p44.count_SET((char)33843) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p44.target_component_SET((char)155) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)143);
            assert(pack.target_component_GET() == (char)117);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_component_SET((char)117) ;
        p45.target_system_SET((char)143) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)54043);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)54043) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)156);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE);
            assert(pack.target_component_GET() == (char)32);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p47.target_system_SET((char)156) ;
        p47.target_component_SET((char)32) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)62);
            assert(pack.time_usec_TRY(ph) == 3047916750487513683L);
            assert(pack.latitude_GET() == -626812310);
            assert(pack.longitude_GET() == 1375548955);
            assert(pack.altitude_GET() == 1338171803);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.longitude_SET(1375548955) ;
        p48.target_system_SET((char)62) ;
        p48.latitude_SET(-626812310) ;
        p48.altitude_SET(1338171803) ;
        p48.time_usec_SET(3047916750487513683L, PH) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == -1680788939);
            assert(pack.time_usec_TRY(ph) == 1694355868232272897L);
            assert(pack.altitude_GET() == -1548791326);
            assert(pack.latitude_GET() == 2099701229);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(2099701229) ;
        p49.altitude_SET(-1548791326) ;
        p49.longitude_SET(-1680788939) ;
        p49.time_usec_SET(1694355868232272897L, PH) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_value_max_GET() == 2.921592E37F);
            assert(pack.target_component_GET() == (char)201);
            assert(pack.param_value0_GET() == -9.267913E36F);
            assert(pack.parameter_rc_channel_index_GET() == (char)72);
            assert(pack.target_system_GET() == (char)133);
            assert(pack.scale_GET() == -3.3774765E38F);
            assert(pack.param_value_min_GET() == -2.8187189E38F);
            assert(pack.param_id_LEN(ph) == 13);
            assert(pack.param_id_TRY(ph).equals("ZbfWlwimltley"));
            assert(pack.param_index_GET() == (short)24753);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_component_SET((char)201) ;
        p50.param_value_min_SET(-2.8187189E38F) ;
        p50.param_index_SET((short)24753) ;
        p50.target_system_SET((char)133) ;
        p50.param_value_max_SET(2.921592E37F) ;
        p50.parameter_rc_channel_index_SET((char)72) ;
        p50.param_value0_SET(-9.267913E36F) ;
        p50.param_id_SET("ZbfWlwimltley", PH) ;
        p50.scale_SET(-3.3774765E38F) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)49028);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION);
            assert(pack.target_system_GET() == (char)111);
            assert(pack.target_component_GET() == (char)177);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_component_SET((char)177) ;
        p51.seq_SET((char)49028) ;
        p51.target_system_SET((char)111) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2z_GET() == -5.6842467E37F);
            assert(pack.p2x_GET() == -5.585042E36F);
            assert(pack.p2y_GET() == 2.8022325E38F);
            assert(pack.target_system_GET() == (char)94);
            assert(pack.p1x_GET() == 3.106511E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.p1y_GET() == -2.461803E38F);
            assert(pack.p1z_GET() == 5.801095E37F);
            assert(pack.target_component_GET() == (char)58);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p2z_SET(-5.6842467E37F) ;
        p54.p2x_SET(-5.585042E36F) ;
        p54.target_component_SET((char)58) ;
        p54.p1z_SET(5.801095E37F) ;
        p54.target_system_SET((char)94) ;
        p54.p2y_SET(2.8022325E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p54.p1x_SET(3.106511E37F) ;
        p54.p1y_SET(-2.461803E38F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2x_GET() == -3.3541226E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.p2z_GET() == 2.6588602E38F);
            assert(pack.p2y_GET() == -3.3859564E38F);
            assert(pack.p1x_GET() == -1.8482252E38F);
            assert(pack.p1y_GET() == 2.3471275E38F);
            assert(pack.p1z_GET() == 5.468523E37F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p1z_SET(5.468523E37F) ;
        p55.p2x_SET(-3.3541226E38F) ;
        p55.p1y_SET(2.3471275E38F) ;
        p55.p1x_SET(-1.8482252E38F) ;
        p55.p2y_SET(-3.3859564E38F) ;
        p55.p2z_SET(2.6588602E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == 1.3030526E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.7630907E37F, 2.5693182E38F, 1.1928854E38F, 6.5370346E36F}));
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.456177E38F, 1.8893517E38F, 3.133066E38F, -1.6278916E38F, 1.7927393E38F, 3.0065582E36F, -4.165711E36F, 2.0480149E38F, 2.7845962E38F}));
            assert(pack.yawspeed_GET() == -2.5692E38F);
            assert(pack.pitchspeed_GET() == 2.7755569E38F);
            assert(pack.time_usec_GET() == 9201865736937192435L);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.q_SET(new float[] {2.7630907E37F, 2.5693182E38F, 1.1928854E38F, 6.5370346E36F}, 0) ;
        p61.rollspeed_SET(1.3030526E38F) ;
        p61.time_usec_SET(9201865736937192435L) ;
        p61.covariance_SET(new float[] {-1.456177E38F, 1.8893517E38F, 3.133066E38F, -1.6278916E38F, 1.7927393E38F, 3.0065582E36F, -4.165711E36F, 2.0480149E38F, 2.7845962E38F}, 0) ;
        p61.pitchspeed_SET(2.7755569E38F) ;
        p61.yawspeed_SET(-2.5692E38F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.xtrack_error_GET() == -2.2251915E37F);
            assert(pack.target_bearing_GET() == (short) -18233);
            assert(pack.nav_bearing_GET() == (short)1008);
            assert(pack.nav_pitch_GET() == 7.2461555E37F);
            assert(pack.nav_roll_GET() == 8.1321033E37F);
            assert(pack.alt_error_GET() == 5.7572517E37F);
            assert(pack.aspd_error_GET() == -2.268449E37F);
            assert(pack.wp_dist_GET() == (char)52603);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.target_bearing_SET((short) -18233) ;
        p62.nav_roll_SET(8.1321033E37F) ;
        p62.nav_pitch_SET(7.2461555E37F) ;
        p62.alt_error_SET(5.7572517E37F) ;
        p62.wp_dist_SET((char)52603) ;
        p62.aspd_error_SET(-2.268449E37F) ;
        p62.xtrack_error_SET(-2.2251915E37F) ;
        p62.nav_bearing_SET((short)1008) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6546289605912188235L);
            assert(pack.lat_GET() == -1774113810);
            assert(pack.relative_alt_GET() == -1877841060);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.0203446E38F, 2.5971857E38F, -1.2322337E38F, -2.7878815E38F, 3.041765E37F, -3.1421825E38F, -3.6787253E37F, 3.335426E38F, 2.5990476E38F, -2.4906547E38F, 2.8813205E38F, -2.0898315E38F, -1.5023343E37F, -8.609421E35F, 6.7819865E36F, 2.7755155E38F, 5.846033E36F, -1.894465E38F, 2.8495877E38F, 1.2369857E38F, 2.1870539E38F, -3.3840612E38F, -3.1738411E38F, -3.067209E38F, 1.04246236E37F, 2.2979547E37F, -2.2424473E38F, -9.1137664E36F, 3.3599961E37F, 1.1791092E38F, -2.634554E38F, 8.442482E37F, -1.6581429E36F, -3.0276403E38F, -1.1857235E38F, 1.221733E38F}));
            assert(pack.lon_GET() == 801989510);
            assert(pack.vy_GET() == -2.1958396E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(pack.alt_GET() == 585992106);
            assert(pack.vx_GET() == -2.6498256E38F);
            assert(pack.vz_GET() == -2.9602846E38F);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.lat_SET(-1774113810) ;
        p63.vx_SET(-2.6498256E38F) ;
        p63.covariance_SET(new float[] {-2.0203446E38F, 2.5971857E38F, -1.2322337E38F, -2.7878815E38F, 3.041765E37F, -3.1421825E38F, -3.6787253E37F, 3.335426E38F, 2.5990476E38F, -2.4906547E38F, 2.8813205E38F, -2.0898315E38F, -1.5023343E37F, -8.609421E35F, 6.7819865E36F, 2.7755155E38F, 5.846033E36F, -1.894465E38F, 2.8495877E38F, 1.2369857E38F, 2.1870539E38F, -3.3840612E38F, -3.1738411E38F, -3.067209E38F, 1.04246236E37F, 2.2979547E37F, -2.2424473E38F, -9.1137664E36F, 3.3599961E37F, 1.1791092E38F, -2.634554E38F, 8.442482E37F, -1.6581429E36F, -3.0276403E38F, -1.1857235E38F, 1.221733E38F}, 0) ;
        p63.vy_SET(-2.1958396E38F) ;
        p63.time_usec_SET(6546289605912188235L) ;
        p63.alt_SET(585992106) ;
        p63.lon_SET(801989510) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p63.relative_alt_SET(-1877841060) ;
        p63.vz_SET(-2.9602846E38F) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.az_GET() == 2.6283147E38F);
            assert(pack.ay_GET() == -1.9453895E38F);
            assert(pack.vz_GET() == -2.9558998E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.9097185E38F, 2.2493446E38F, 1.0506433E38F, -3.2692141E38F, 1.383977E38F, 3.3849727E38F, -9.566952E37F, 1.3317726E38F, 7.4126376E37F, -2.5353241E38F, -2.1137918E38F, 8.78083E37F, 2.7841836E38F, -1.9591516E38F, 1.3029553E37F, 1.4456338E38F, 2.4578648E38F, -3.3941114E38F, 1.2159081E38F, 1.8763778E38F, 3.152908E38F, 1.3530148E38F, 1.7418807E38F, -8.534461E36F, -1.3305402E38F, 8.873365E37F, 1.8588697E38F, -2.3105553E38F, -9.0374957E36F, -3.3749597E37F, 2.6345816E38F, -3.3232544E38F, 2.8849654E37F, 5.730234E37F, 1.8087096E38F, -4.1825657E37F, 2.48433E38F, 2.0323648E38F, -2.652544E38F, -1.8224473E38F, -1.9539674E38F, 2.9062963E38F, -3.3323794E38F, 2.0852508E38F, -1.9102369E38F}));
            assert(pack.time_usec_GET() == 9019141821635854588L);
            assert(pack.vy_GET() == -1.6908239E38F);
            assert(pack.x_GET() == 2.5244797E38F);
            assert(pack.y_GET() == -1.1878019E38F);
            assert(pack.ax_GET() == 1.6048625E38F);
            assert(pack.z_GET() == 3.1392876E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
            assert(pack.vx_GET() == 1.4646536E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.y_SET(-1.1878019E38F) ;
        p64.covariance_SET(new float[] {2.9097185E38F, 2.2493446E38F, 1.0506433E38F, -3.2692141E38F, 1.383977E38F, 3.3849727E38F, -9.566952E37F, 1.3317726E38F, 7.4126376E37F, -2.5353241E38F, -2.1137918E38F, 8.78083E37F, 2.7841836E38F, -1.9591516E38F, 1.3029553E37F, 1.4456338E38F, 2.4578648E38F, -3.3941114E38F, 1.2159081E38F, 1.8763778E38F, 3.152908E38F, 1.3530148E38F, 1.7418807E38F, -8.534461E36F, -1.3305402E38F, 8.873365E37F, 1.8588697E38F, -2.3105553E38F, -9.0374957E36F, -3.3749597E37F, 2.6345816E38F, -3.3232544E38F, 2.8849654E37F, 5.730234E37F, 1.8087096E38F, -4.1825657E37F, 2.48433E38F, 2.0323648E38F, -2.652544E38F, -1.8224473E38F, -1.9539674E38F, 2.9062963E38F, -3.3323794E38F, 2.0852508E38F, -1.9102369E38F}, 0) ;
        p64.vx_SET(1.4646536E38F) ;
        p64.x_SET(2.5244797E38F) ;
        p64.az_SET(2.6283147E38F) ;
        p64.z_SET(3.1392876E38F) ;
        p64.ay_SET(-1.9453895E38F) ;
        p64.vy_SET(-1.6908239E38F) ;
        p64.time_usec_SET(9019141821635854588L) ;
        p64.ax_SET(1.6048625E38F) ;
        p64.vz_SET(-2.9558998E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan12_raw_GET() == (char)8027);
            assert(pack.chan13_raw_GET() == (char)8528);
            assert(pack.chancount_GET() == (char)23);
            assert(pack.chan10_raw_GET() == (char)58974);
            assert(pack.chan16_raw_GET() == (char)44100);
            assert(pack.chan15_raw_GET() == (char)3407);
            assert(pack.chan4_raw_GET() == (char)17655);
            assert(pack.chan3_raw_GET() == (char)50659);
            assert(pack.chan9_raw_GET() == (char)44189);
            assert(pack.chan1_raw_GET() == (char)5750);
            assert(pack.time_boot_ms_GET() == 1894466950L);
            assert(pack.chan14_raw_GET() == (char)51488);
            assert(pack.chan2_raw_GET() == (char)58719);
            assert(pack.chan8_raw_GET() == (char)33927);
            assert(pack.rssi_GET() == (char)17);
            assert(pack.chan5_raw_GET() == (char)37853);
            assert(pack.chan18_raw_GET() == (char)62500);
            assert(pack.chan17_raw_GET() == (char)18276);
            assert(pack.chan11_raw_GET() == (char)35714);
            assert(pack.chan7_raw_GET() == (char)8481);
            assert(pack.chan6_raw_GET() == (char)4516);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan15_raw_SET((char)3407) ;
        p65.chan18_raw_SET((char)62500) ;
        p65.chan14_raw_SET((char)51488) ;
        p65.chan17_raw_SET((char)18276) ;
        p65.chan8_raw_SET((char)33927) ;
        p65.chan16_raw_SET((char)44100) ;
        p65.chan3_raw_SET((char)50659) ;
        p65.chan1_raw_SET((char)5750) ;
        p65.chan10_raw_SET((char)58974) ;
        p65.chan5_raw_SET((char)37853) ;
        p65.chan2_raw_SET((char)58719) ;
        p65.chan7_raw_SET((char)8481) ;
        p65.rssi_SET((char)17) ;
        p65.chan12_raw_SET((char)8027) ;
        p65.chan13_raw_SET((char)8528) ;
        p65.chan4_raw_SET((char)17655) ;
        p65.chancount_SET((char)23) ;
        p65.time_boot_ms_SET(1894466950L) ;
        p65.chan9_raw_SET((char)44189) ;
        p65.chan6_raw_SET((char)4516) ;
        p65.chan11_raw_SET((char)35714) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.start_stop_GET() == (char)186);
            assert(pack.req_stream_id_GET() == (char)197);
            assert(pack.target_component_GET() == (char)205);
            assert(pack.target_system_GET() == (char)240);
            assert(pack.req_message_rate_GET() == (char)14759);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_message_rate_SET((char)14759) ;
        p66.req_stream_id_SET((char)197) ;
        p66.target_component_SET((char)205) ;
        p66.target_system_SET((char)240) ;
        p66.start_stop_SET((char)186) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.on_off_GET() == (char)25);
            assert(pack.message_rate_GET() == (char)50074);
            assert(pack.stream_id_GET() == (char)209);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)50074) ;
        p67.stream_id_SET((char)209) ;
        p67.on_off_SET((char)25) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_GET() == (char)150);
            assert(pack.y_GET() == (short) -21975);
            assert(pack.r_GET() == (short)3092);
            assert(pack.z_GET() == (short) -20778);
            assert(pack.buttons_GET() == (char)60047);
            assert(pack.x_GET() == (short) -26770);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.r_SET((short)3092) ;
        p69.y_SET((short) -21975) ;
        p69.buttons_SET((char)60047) ;
        p69.x_SET((short) -26770) ;
        p69.target_SET((char)150) ;
        p69.z_SET((short) -20778) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan7_raw_GET() == (char)51290);
            assert(pack.target_component_GET() == (char)139);
            assert(pack.chan5_raw_GET() == (char)39006);
            assert(pack.chan1_raw_GET() == (char)30116);
            assert(pack.chan8_raw_GET() == (char)39294);
            assert(pack.target_system_GET() == (char)134);
            assert(pack.chan4_raw_GET() == (char)14097);
            assert(pack.chan6_raw_GET() == (char)61182);
            assert(pack.chan2_raw_GET() == (char)13642);
            assert(pack.chan3_raw_GET() == (char)28245);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.target_component_SET((char)139) ;
        p70.chan8_raw_SET((char)39294) ;
        p70.chan2_raw_SET((char)13642) ;
        p70.chan4_raw_SET((char)14097) ;
        p70.chan6_raw_SET((char)61182) ;
        p70.target_system_SET((char)134) ;
        p70.chan1_raw_SET((char)30116) ;
        p70.chan7_raw_SET((char)51290) ;
        p70.chan3_raw_SET((char)28245) ;
        p70.chan5_raw_SET((char)39006) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.param1_GET() == -1.0894983E38F);
            assert(pack.z_GET() == -1.8017193E38F);
            assert(pack.param4_GET() == 3.024637E37F);
            assert(pack.seq_GET() == (char)58987);
            assert(pack.current_GET() == (char)7);
            assert(pack.y_GET() == -90054338);
            assert(pack.param2_GET() == 2.7608343E38F);
            assert(pack.param3_GET() == -7.2774264E37F);
            assert(pack.x_GET() == -1704439567);
            assert(pack.target_system_GET() == (char)221);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.autocontinue_GET() == (char)98);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN);
            assert(pack.target_component_GET() == (char)110);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.target_component_SET((char)110) ;
        p73.seq_SET((char)58987) ;
        p73.z_SET(-1.8017193E38F) ;
        p73.autocontinue_SET((char)98) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p73.command_SET(MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN) ;
        p73.param4_SET(3.024637E37F) ;
        p73.current_SET((char)7) ;
        p73.param2_SET(2.7608343E38F) ;
        p73.param3_SET(-7.2774264E37F) ;
        p73.target_system_SET((char)221) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p73.param1_SET(-1.0894983E38F) ;
        p73.x_SET(-1704439567) ;
        p73.y_SET(-90054338) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.throttle_GET() == (char)9292);
            assert(pack.airspeed_GET() == 2.7060653E38F);
            assert(pack.climb_GET() == 2.6876189E38F);
            assert(pack.groundspeed_GET() == 1.9249143E38F);
            assert(pack.heading_GET() == (short) -2349);
            assert(pack.alt_GET() == 2.9992118E38F);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.heading_SET((short) -2349) ;
        p74.airspeed_SET(2.7060653E38F) ;
        p74.climb_SET(2.6876189E38F) ;
        p74.groundspeed_SET(1.9249143E38F) ;
        p74.throttle_SET((char)9292) ;
        p74.alt_SET(2.9992118E38F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.param2_GET() == -1.2720681E38F);
            assert(pack.z_GET() == 2.8948394E38F);
            assert(pack.target_component_GET() == (char)50);
            assert(pack.param3_GET() == -6.2950915E37F);
            assert(pack.param4_GET() == -9.20242E36F);
            assert(pack.current_GET() == (char)55);
            assert(pack.autocontinue_GET() == (char)182);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL);
            assert(pack.x_GET() == -1842615893);
            assert(pack.target_system_GET() == (char)209);
            assert(pack.param1_GET() == 2.9949647E38F);
            assert(pack.y_GET() == 1628988563);
        });
        GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.param2_SET(-1.2720681E38F) ;
        p75.param1_SET(2.9949647E38F) ;
        p75.y_SET(1628988563) ;
        p75.param4_SET(-9.20242E36F) ;
        p75.autocontinue_SET((char)182) ;
        p75.current_SET((char)55) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p75.z_SET(2.8948394E38F) ;
        p75.target_system_SET((char)209) ;
        p75.param3_SET(-6.2950915E37F) ;
        p75.x_SET(-1842615893) ;
        p75.command_SET(MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL) ;
        p75.target_component_SET((char)50) ;
        CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)214);
            assert(pack.target_system_GET() == (char)190);
            assert(pack.confirmation_GET() == (char)246);
            assert(pack.param6_GET() == -2.4934199E38F);
            assert(pack.param7_GET() == 1.3683062E38F);
            assert(pack.param4_GET() == -8.911618E37F);
            assert(pack.param2_GET() == -1.9107113E38F);
            assert(pack.param1_GET() == -1.5012586E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_LAST);
            assert(pack.param3_GET() == -1.8515641E38F);
            assert(pack.param5_GET() == -2.3629372E38F);
        });
        GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param5_SET(-2.3629372E38F) ;
        p76.target_system_SET((char)190) ;
        p76.confirmation_SET((char)246) ;
        p76.param6_SET(-2.4934199E38F) ;
        p76.param2_SET(-1.9107113E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_DO_LAST) ;
        p76.param1_SET(-1.5012586E38F) ;
        p76.target_component_SET((char)214) ;
        p76.param4_SET(-8.911618E37F) ;
        p76.param3_SET(-1.8515641E38F) ;
        p76.param7_SET(1.3683062E38F) ;
        CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.progress_TRY(ph) == (char)61);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION);
            assert(pack.target_component_TRY(ph) == (char)43);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED);
            assert(pack.result_param2_TRY(ph) == 1773146466);
            assert(pack.target_system_TRY(ph) == (char)112);
        });
        GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.target_system_SET((char)112, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED) ;
        p77.target_component_SET((char)43, PH) ;
        p77.progress_SET((char)61, PH) ;
        p77.result_param2_SET(1773146466, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION) ;
        CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.manual_override_switch_GET() == (char)186);
            assert(pack.pitch_GET() == -2.9681708E38F);
            assert(pack.roll_GET() == 1.4332041E38F);
            assert(pack.mode_switch_GET() == (char)53);
            assert(pack.thrust_GET() == 3.3335205E38F);
            assert(pack.time_boot_ms_GET() == 2582627943L);
            assert(pack.yaw_GET() == -1.6726669E38F);
        });
        GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.pitch_SET(-2.9681708E38F) ;
        p81.mode_switch_SET((char)53) ;
        p81.thrust_SET(3.3335205E38F) ;
        p81.yaw_SET(-1.6726669E38F) ;
        p81.manual_override_switch_SET((char)186) ;
        p81.time_boot_ms_SET(2582627943L) ;
        p81.roll_SET(1.4332041E38F) ;
        CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.thrust_GET() == -8.268137E37F);
            assert(pack.type_mask_GET() == (char)15);
            assert(pack.body_pitch_rate_GET() == -3.2624651E38F);
            assert(pack.target_system_GET() == (char)146);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.3925875E38F, -1.2258108E38F, -6.801281E37F, 3.0600812E38F}));
            assert(pack.body_roll_rate_GET() == -2.6521259E38F);
            assert(pack.target_component_GET() == (char)90);
            assert(pack.body_yaw_rate_GET() == 2.752288E38F);
            assert(pack.time_boot_ms_GET() == 3014843138L);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.q_SET(new float[] {3.3925875E38F, -1.2258108E38F, -6.801281E37F, 3.0600812E38F}, 0) ;
        p82.target_component_SET((char)90) ;
        p82.target_system_SET((char)146) ;
        p82.body_pitch_rate_SET(-3.2624651E38F) ;
        p82.thrust_SET(-8.268137E37F) ;
        p82.body_yaw_rate_SET(2.752288E38F) ;
        p82.time_boot_ms_SET(3014843138L) ;
        p82.body_roll_rate_SET(-2.6521259E38F) ;
        p82.type_mask_SET((char)15) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.0245288E37F, -1.8588918E38F, -1.6773962E38F, 1.2580055E38F}));
            assert(pack.body_yaw_rate_GET() == -4.6623687E37F);
            assert(pack.time_boot_ms_GET() == 1445960514L);
            assert(pack.body_roll_rate_GET() == -1.0946972E38F);
            assert(pack.body_pitch_rate_GET() == -1.0046882E38F);
            assert(pack.type_mask_GET() == (char)137);
            assert(pack.thrust_GET() == 2.8782737E38F);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.q_SET(new float[] {2.0245288E37F, -1.8588918E38F, -1.6773962E38F, 1.2580055E38F}, 0) ;
        p83.body_pitch_rate_SET(-1.0046882E38F) ;
        p83.body_roll_rate_SET(-1.0946972E38F) ;
        p83.body_yaw_rate_SET(-4.6623687E37F) ;
        p83.thrust_SET(2.8782737E38F) ;
        p83.type_mask_SET((char)137) ;
        p83.time_boot_ms_SET(1445960514L) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1412144045L);
            assert(pack.vy_GET() == -1.6977248E37F);
            assert(pack.vz_GET() == 2.4973084E38F);
            assert(pack.type_mask_GET() == (char)52748);
            assert(pack.yaw_GET() == -2.5900195E38F);
            assert(pack.x_GET() == 1.1550539E38F);
            assert(pack.yaw_rate_GET() == 1.1996795E38F);
            assert(pack.afz_GET() == 3.193762E38F);
            assert(pack.vx_GET() == 2.046237E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.target_component_GET() == (char)53);
            assert(pack.afx_GET() == 2.2083475E38F);
            assert(pack.z_GET() == 2.522956E38F);
            assert(pack.afy_GET() == -2.6826493E38F);
            assert(pack.y_GET() == 1.6027234E38F);
            assert(pack.target_system_GET() == (char)62);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.yaw_SET(-2.5900195E38F) ;
        p84.vz_SET(2.4973084E38F) ;
        p84.vx_SET(2.046237E38F) ;
        p84.x_SET(1.1550539E38F) ;
        p84.afy_SET(-2.6826493E38F) ;
        p84.z_SET(2.522956E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p84.afx_SET(2.2083475E38F) ;
        p84.y_SET(1.6027234E38F) ;
        p84.vy_SET(-1.6977248E37F) ;
        p84.target_component_SET((char)53) ;
        p84.yaw_rate_SET(1.1996795E38F) ;
        p84.type_mask_SET((char)52748) ;
        p84.time_boot_ms_SET(1412144045L) ;
        p84.target_system_SET((char)62) ;
        p84.afz_SET(3.193762E38F) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)188);
            assert(pack.yaw_rate_GET() == 2.8883654E38F);
            assert(pack.time_boot_ms_GET() == 2141129882L);
            assert(pack.afz_GET() == 5.0769924E37F);
            assert(pack.lat_int_GET() == 770019574);
            assert(pack.target_system_GET() == (char)200);
            assert(pack.afx_GET() == 2.6253987E38F);
            assert(pack.lon_int_GET() == 592660752);
            assert(pack.yaw_GET() == -1.5780085E38F);
            assert(pack.type_mask_GET() == (char)24841);
            assert(pack.alt_GET() == 4.1351138E37F);
            assert(pack.afy_GET() == -2.0228769E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.vy_GET() == 2.4895707E38F);
            assert(pack.vx_GET() == -1.6114604E38F);
            assert(pack.vz_GET() == 2.566325E38F);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.vy_SET(2.4895707E38F) ;
        p86.target_component_SET((char)188) ;
        p86.lon_int_SET(592660752) ;
        p86.target_system_SET((char)200) ;
        p86.lat_int_SET(770019574) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p86.time_boot_ms_SET(2141129882L) ;
        p86.alt_SET(4.1351138E37F) ;
        p86.afy_SET(-2.0228769E38F) ;
        p86.vx_SET(-1.6114604E38F) ;
        p86.afx_SET(2.6253987E38F) ;
        p86.afz_SET(5.0769924E37F) ;
        p86.yaw_SET(-1.5780085E38F) ;
        p86.yaw_rate_SET(2.8883654E38F) ;
        p86.vz_SET(2.566325E38F) ;
        p86.type_mask_SET((char)24841) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == -1.2614424E38F);
            assert(pack.afx_GET() == -1.015305E38F);
            assert(pack.afz_GET() == 2.2706833E38F);
            assert(pack.time_boot_ms_GET() == 3972622051L);
            assert(pack.vy_GET() == -9.337372E37F);
            assert(pack.lon_int_GET() == -1876456370);
            assert(pack.afy_GET() == -1.830448E38F);
            assert(pack.yaw_rate_GET() == -8.377179E37F);
            assert(pack.vx_GET() == 6.0369477E37F);
            assert(pack.lat_int_GET() == 1913250464);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.yaw_GET() == -6.015935E36F);
            assert(pack.alt_GET() == -1.1431227E38F);
            assert(pack.type_mask_GET() == (char)54261);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(3972622051L) ;
        p87.afx_SET(-1.015305E38F) ;
        p87.vx_SET(6.0369477E37F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p87.type_mask_SET((char)54261) ;
        p87.yaw_SET(-6.015935E36F) ;
        p87.yaw_rate_SET(-8.377179E37F) ;
        p87.alt_SET(-1.1431227E38F) ;
        p87.vz_SET(-1.2614424E38F) ;
        p87.lon_int_SET(-1876456370) ;
        p87.lat_int_SET(1913250464) ;
        p87.afz_SET(2.2706833E38F) ;
        p87.afy_SET(-1.830448E38F) ;
        p87.vy_SET(-9.337372E37F) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 1.344923E37F);
            assert(pack.time_boot_ms_GET() == 1775644807L);
            assert(pack.yaw_GET() == 1.796526E38F);
            assert(pack.roll_GET() == -1.3144618E38F);
            assert(pack.z_GET() == -2.7238092E38F);
            assert(pack.x_GET() == 1.5599848E38F);
            assert(pack.y_GET() == 1.3900864E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.y_SET(1.3900864E38F) ;
        p89.time_boot_ms_SET(1775644807L) ;
        p89.roll_SET(-1.3144618E38F) ;
        p89.x_SET(1.5599848E38F) ;
        p89.pitch_SET(1.344923E37F) ;
        p89.z_SET(-2.7238092E38F) ;
        p89.yaw_SET(1.796526E38F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 1641275479);
            assert(pack.roll_GET() == -1.5521148E38F);
            assert(pack.yawspeed_GET() == -2.131791E38F);
            assert(pack.vy_GET() == (short) -19052);
            assert(pack.xacc_GET() == (short)17202);
            assert(pack.vz_GET() == (short)26928);
            assert(pack.yacc_GET() == (short) -11365);
            assert(pack.pitch_GET() == 3.1426352E38F);
            assert(pack.pitchspeed_GET() == 2.6369546E38F);
            assert(pack.alt_GET() == -317884342);
            assert(pack.zacc_GET() == (short) -25823);
            assert(pack.lat_GET() == -1098415426);
            assert(pack.time_usec_GET() == 3155976735736302353L);
            assert(pack.yaw_GET() == -1.0329278E38F);
            assert(pack.rollspeed_GET() == -2.6181E38F);
            assert(pack.vx_GET() == (short)30070);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.roll_SET(-1.5521148E38F) ;
        p90.yacc_SET((short) -11365) ;
        p90.rollspeed_SET(-2.6181E38F) ;
        p90.vz_SET((short)26928) ;
        p90.alt_SET(-317884342) ;
        p90.zacc_SET((short) -25823) ;
        p90.lat_SET(-1098415426) ;
        p90.yaw_SET(-1.0329278E38F) ;
        p90.lon_SET(1641275479) ;
        p90.pitchspeed_SET(2.6369546E38F) ;
        p90.xacc_SET((short)17202) ;
        p90.vy_SET((short) -19052) ;
        p90.vx_SET((short)30070) ;
        p90.yawspeed_SET(-2.131791E38F) ;
        p90.pitch_SET(3.1426352E38F) ;
        p90.time_usec_SET(3155976735736302353L) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux1_GET() == -2.4061503E38F);
            assert(pack.throttle_GET() == 2.5693E38F);
            assert(pack.roll_ailerons_GET() == -2.560314E38F);
            assert(pack.nav_mode_GET() == (char)236);
            assert(pack.yaw_rudder_GET() == 7.5920044E36F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_ARMED);
            assert(pack.time_usec_GET() == 6047309859485224400L);
            assert(pack.pitch_elevator_GET() == -1.7025515E38F);
            assert(pack.aux4_GET() == -8.1012827E37F);
            assert(pack.aux3_GET() == 1.0742194E38F);
            assert(pack.aux2_GET() == -1.3285584E38F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.nav_mode_SET((char)236) ;
        p91.aux1_SET(-2.4061503E38F) ;
        p91.throttle_SET(2.5693E38F) ;
        p91.aux3_SET(1.0742194E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_STABILIZE_ARMED) ;
        p91.aux2_SET(-1.3285584E38F) ;
        p91.aux4_SET(-8.1012827E37F) ;
        p91.yaw_rudder_SET(7.5920044E36F) ;
        p91.time_usec_SET(6047309859485224400L) ;
        p91.roll_ailerons_SET(-2.560314E38F) ;
        p91.pitch_elevator_SET(-1.7025515E38F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan11_raw_GET() == (char)56423);
            assert(pack.chan10_raw_GET() == (char)24398);
            assert(pack.chan4_raw_GET() == (char)36234);
            assert(pack.rssi_GET() == (char)7);
            assert(pack.chan1_raw_GET() == (char)18496);
            assert(pack.chan2_raw_GET() == (char)63716);
            assert(pack.chan6_raw_GET() == (char)26985);
            assert(pack.chan12_raw_GET() == (char)29871);
            assert(pack.chan3_raw_GET() == (char)53904);
            assert(pack.chan8_raw_GET() == (char)32044);
            assert(pack.chan5_raw_GET() == (char)58776);
            assert(pack.time_usec_GET() == 5835958439114775618L);
            assert(pack.chan9_raw_GET() == (char)51238);
            assert(pack.chan7_raw_GET() == (char)64932);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan2_raw_SET((char)63716) ;
        p92.chan6_raw_SET((char)26985) ;
        p92.chan4_raw_SET((char)36234) ;
        p92.chan1_raw_SET((char)18496) ;
        p92.chan5_raw_SET((char)58776) ;
        p92.chan9_raw_SET((char)51238) ;
        p92.chan10_raw_SET((char)24398) ;
        p92.rssi_SET((char)7) ;
        p92.chan3_raw_SET((char)53904) ;
        p92.time_usec_SET(5835958439114775618L) ;
        p92.chan7_raw_SET((char)64932) ;
        p92.chan12_raw_SET((char)29871) ;
        p92.chan8_raw_SET((char)32044) ;
        p92.chan11_raw_SET((char)56423) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_PREFLIGHT);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.0407923E37F, -1.4415169E38F, 1.3014316E38F, 1.0637286E38F, 1.4212943E38F, -3.2589457E38F, -1.9291768E38F, 3.388163E38F, -1.0397897E38F, 2.6272994E38F, 1.4219584E38F, -1.5453903E38F, 2.531507E38F, -2.364271E38F, 1.1803311E38F, -2.4108217E37F}));
            assert(pack.time_usec_GET() == 8245302853571870156L);
            assert(pack.flags_GET() == 3326999015561878597L);
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.mode_SET(MAV_MODE.MAV_MODE_PREFLIGHT) ;
        p93.time_usec_SET(8245302853571870156L) ;
        p93.flags_SET(3326999015561878597L) ;
        p93.controls_SET(new float[] {2.0407923E37F, -1.4415169E38F, 1.3014316E38F, 1.0637286E38F, 1.4212943E38F, -3.2589457E38F, -1.9291768E38F, 3.388163E38F, -1.0397897E38F, 2.6272994E38F, 1.4219584E38F, -1.5453903E38F, 2.531507E38F, -2.364271E38F, 1.1803311E38F, -2.4108217E37F}, 0) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.ground_distance_GET() == -5.3191654E37F);
            assert(pack.flow_comp_m_x_GET() == -9.2944E37F);
            assert(pack.flow_rate_x_TRY(ph) == 1.2006643E38F);
            assert(pack.flow_comp_m_y_GET() == 8.2892276E37F);
            assert(pack.flow_rate_y_TRY(ph) == 2.1402526E37F);
            assert(pack.time_usec_GET() == 3364081136355612522L);
            assert(pack.flow_y_GET() == (short) -1365);
            assert(pack.quality_GET() == (char)248);
            assert(pack.flow_x_GET() == (short)2264);
            assert(pack.sensor_id_GET() == (char)20);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_rate_y_SET(2.1402526E37F, PH) ;
        p100.quality_SET((char)248) ;
        p100.flow_comp_m_y_SET(8.2892276E37F) ;
        p100.flow_y_SET((short) -1365) ;
        p100.ground_distance_SET(-5.3191654E37F) ;
        p100.flow_rate_x_SET(1.2006643E38F, PH) ;
        p100.time_usec_SET(3364081136355612522L) ;
        p100.flow_comp_m_x_SET(-9.2944E37F) ;
        p100.flow_x_SET((short)2264) ;
        p100.sensor_id_SET((char)20) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 1.937387E38F);
            assert(pack.roll_GET() == -1.8619441E37F);
            assert(pack.yaw_GET() == -2.2728312E38F);
            assert(pack.y_GET() == 7.1747624E37F);
            assert(pack.pitch_GET() == 9.154722E37F);
            assert(pack.usec_GET() == 592812691829447332L);
            assert(pack.z_GET() == 2.654048E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.yaw_SET(-2.2728312E38F) ;
        p101.usec_SET(592812691829447332L) ;
        p101.roll_SET(-1.8619441E37F) ;
        p101.x_SET(1.937387E38F) ;
        p101.y_SET(7.1747624E37F) ;
        p101.pitch_SET(9.154722E37F) ;
        p101.z_SET(2.654048E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 2.8574212E38F);
            assert(pack.usec_GET() == 3756641190428961445L);
            assert(pack.pitch_GET() == 2.3795319E38F);
            assert(pack.y_GET() == 2.8528943E38F);
            assert(pack.x_GET() == 2.0692074E38F);
            assert(pack.roll_GET() == -3.261869E38F);
            assert(pack.z_GET() == -2.6505592E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.z_SET(-2.6505592E38F) ;
        p102.roll_SET(-3.261869E38F) ;
        p102.y_SET(2.8528943E38F) ;
        p102.pitch_SET(2.3795319E38F) ;
        p102.yaw_SET(2.8574212E38F) ;
        p102.usec_SET(3756641190428961445L) ;
        p102.x_SET(2.0692074E38F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 8248145175525905507L);
            assert(pack.z_GET() == 2.786651E38F);
            assert(pack.y_GET() == 6.6107315E37F);
            assert(pack.x_GET() == -1.8120084E38F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(8248145175525905507L) ;
        p103.x_SET(-1.8120084E38F) ;
        p103.y_SET(6.6107315E37F) ;
        p103.z_SET(2.786651E38F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 1.5937387E38F);
            assert(pack.pitch_GET() == 7.631229E37F);
            assert(pack.yaw_GET() == -3.393822E38F);
            assert(pack.usec_GET() == 2103850038368905579L);
            assert(pack.roll_GET() == -3.3353317E38F);
            assert(pack.z_GET() == 1.5413482E38F);
            assert(pack.y_GET() == -1.7715162E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.pitch_SET(7.631229E37F) ;
        p104.x_SET(1.5937387E38F) ;
        p104.z_SET(1.5413482E38F) ;
        p104.usec_SET(2103850038368905579L) ;
        p104.y_SET(-1.7715162E38F) ;
        p104.yaw_SET(-3.393822E38F) ;
        p104.roll_SET(-3.3353317E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.xgyro_GET() == -3.2371519E38F);
            assert(pack.ymag_GET() == -2.5330458E38F);
            assert(pack.abs_pressure_GET() == 1.1012872E38F);
            assert(pack.temperature_GET() == 7.8994306E37F);
            assert(pack.xmag_GET() == 1.8926488E38F);
            assert(pack.pressure_alt_GET() == -2.08749E38F);
            assert(pack.diff_pressure_GET() == -6.4228337E37F);
            assert(pack.zacc_GET() == 3.347537E38F);
            assert(pack.yacc_GET() == -2.1876605E38F);
            assert(pack.xacc_GET() == 2.7227687E38F);
            assert(pack.zmag_GET() == -2.8330515E37F);
            assert(pack.fields_updated_GET() == (char)25117);
            assert(pack.time_usec_GET() == 434723975917240585L);
            assert(pack.ygyro_GET() == 3.067717E38F);
            assert(pack.zgyro_GET() == 4.0039305E37F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.zacc_SET(3.347537E38F) ;
        p105.pressure_alt_SET(-2.08749E38F) ;
        p105.xmag_SET(1.8926488E38F) ;
        p105.fields_updated_SET((char)25117) ;
        p105.diff_pressure_SET(-6.4228337E37F) ;
        p105.abs_pressure_SET(1.1012872E38F) ;
        p105.xgyro_SET(-3.2371519E38F) ;
        p105.yacc_SET(-2.1876605E38F) ;
        p105.zgyro_SET(4.0039305E37F) ;
        p105.time_usec_SET(434723975917240585L) ;
        p105.temperature_SET(7.8994306E37F) ;
        p105.xacc_SET(2.7227687E38F) ;
        p105.zmag_SET(-2.8330515E37F) ;
        p105.ygyro_SET(3.067717E38F) ;
        p105.ymag_SET(-2.5330458E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3839928077309836096L);
            assert(pack.distance_GET() == 1.5118587E38F);
            assert(pack.integration_time_us_GET() == 2027579889L);
            assert(pack.temperature_GET() == (short)14406);
            assert(pack.integrated_x_GET() == 1.1497257E38F);
            assert(pack.sensor_id_GET() == (char)116);
            assert(pack.time_delta_distance_us_GET() == 3808884788L);
            assert(pack.integrated_ygyro_GET() == -5.597206E37F);
            assert(pack.integrated_zgyro_GET() == -2.4667124E38F);
            assert(pack.quality_GET() == (char)253);
            assert(pack.integrated_xgyro_GET() == 7.8648973E37F);
            assert(pack.integrated_y_GET() == -2.6951053E38F);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.distance_SET(1.5118587E38F) ;
        p106.quality_SET((char)253) ;
        p106.temperature_SET((short)14406) ;
        p106.time_usec_SET(3839928077309836096L) ;
        p106.integrated_y_SET(-2.6951053E38F) ;
        p106.integrated_xgyro_SET(7.8648973E37F) ;
        p106.time_delta_distance_us_SET(3808884788L) ;
        p106.integration_time_us_SET(2027579889L) ;
        p106.integrated_zgyro_SET(-2.4667124E38F) ;
        p106.integrated_ygyro_SET(-5.597206E37F) ;
        p106.sensor_id_SET((char)116) ;
        p106.integrated_x_SET(1.1497257E38F) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.pressure_alt_GET() == 3.1972391E38F);
            assert(pack.ymag_GET() == 1.0189508E38F);
            assert(pack.yacc_GET() == -1.2968334E38F);
            assert(pack.zmag_GET() == -1.0327253E38F);
            assert(pack.fields_updated_GET() == 876370228L);
            assert(pack.xgyro_GET() == 1.981671E38F);
            assert(pack.temperature_GET() == -1.4596805E38F);
            assert(pack.xacc_GET() == 3.9704394E37F);
            assert(pack.diff_pressure_GET() == -4.1371917E37F);
            assert(pack.abs_pressure_GET() == -3.2659395E38F);
            assert(pack.zacc_GET() == 2.6627901E38F);
            assert(pack.time_usec_GET() == 7159846942658205654L);
            assert(pack.ygyro_GET() == 2.7678114E38F);
            assert(pack.zgyro_GET() == 1.6164299E38F);
            assert(pack.xmag_GET() == 1.5092378E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.ymag_SET(1.0189508E38F) ;
        p107.ygyro_SET(2.7678114E38F) ;
        p107.yacc_SET(-1.2968334E38F) ;
        p107.diff_pressure_SET(-4.1371917E37F) ;
        p107.zmag_SET(-1.0327253E38F) ;
        p107.zacc_SET(2.6627901E38F) ;
        p107.fields_updated_SET(876370228L) ;
        p107.xacc_SET(3.9704394E37F) ;
        p107.zgyro_SET(1.6164299E38F) ;
        p107.abs_pressure_SET(-3.2659395E38F) ;
        p107.temperature_SET(-1.4596805E38F) ;
        p107.xmag_SET(1.5092378E38F) ;
        p107.pressure_alt_SET(3.1972391E38F) ;
        p107.time_usec_SET(7159846942658205654L) ;
        p107.xgyro_SET(1.981671E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.ve_GET() == -1.8823135E38F);
            assert(pack.q1_GET() == -1.1598493E38F);
            assert(pack.vn_GET() == -2.4920388E38F);
            assert(pack.q3_GET() == 9.106644E37F);
            assert(pack.zacc_GET() == 2.3320183E38F);
            assert(pack.ygyro_GET() == -5.24161E37F);
            assert(pack.zgyro_GET() == 3.3808248E38F);
            assert(pack.vd_GET() == -1.6426892E38F);
            assert(pack.std_dev_horz_GET() == 7.862604E37F);
            assert(pack.yacc_GET() == 1.8747849E38F);
            assert(pack.pitch_GET() == 2.1233123E38F);
            assert(pack.alt_GET() == 1.8309972E38F);
            assert(pack.roll_GET() == 3.3703532E38F);
            assert(pack.xacc_GET() == -6.614679E37F);
            assert(pack.xgyro_GET() == -1.6604593E38F);
            assert(pack.q2_GET() == 7.569468E37F);
            assert(pack.yaw_GET() == 1.3485913E38F);
            assert(pack.q4_GET() == -2.0899573E38F);
            assert(pack.std_dev_vert_GET() == 7.879971E37F);
            assert(pack.lon_GET() == -1.285684E37F);
            assert(pack.lat_GET() == -1.567441E37F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.xacc_SET(-6.614679E37F) ;
        p108.lat_SET(-1.567441E37F) ;
        p108.alt_SET(1.8309972E38F) ;
        p108.pitch_SET(2.1233123E38F) ;
        p108.lon_SET(-1.285684E37F) ;
        p108.zgyro_SET(3.3808248E38F) ;
        p108.vn_SET(-2.4920388E38F) ;
        p108.roll_SET(3.3703532E38F) ;
        p108.std_dev_horz_SET(7.862604E37F) ;
        p108.xgyro_SET(-1.6604593E38F) ;
        p108.q2_SET(7.569468E37F) ;
        p108.vd_SET(-1.6426892E38F) ;
        p108.ygyro_SET(-5.24161E37F) ;
        p108.yacc_SET(1.8747849E38F) ;
        p108.zacc_SET(2.3320183E38F) ;
        p108.q3_SET(9.106644E37F) ;
        p108.ve_SET(-1.8823135E38F) ;
        p108.q1_SET(-1.1598493E38F) ;
        p108.std_dev_vert_SET(7.879971E37F) ;
        p108.q4_SET(-2.0899573E38F) ;
        p108.yaw_SET(1.3485913E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.noise_GET() == (char)164);
            assert(pack.rxerrors_GET() == (char)25974);
            assert(pack.txbuf_GET() == (char)248);
            assert(pack.fixed__GET() == (char)63149);
            assert(pack.remrssi_GET() == (char)31);
            assert(pack.remnoise_GET() == (char)154);
            assert(pack.rssi_GET() == (char)166);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.remnoise_SET((char)154) ;
        p109.noise_SET((char)164) ;
        p109.rssi_SET((char)166) ;
        p109.remrssi_SET((char)31) ;
        p109.fixed__SET((char)63149) ;
        p109.txbuf_SET((char)248) ;
        p109.rxerrors_SET((char)25974) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)185);
            assert(pack.target_component_GET() == (char)66);
            assert(pack.target_system_GET() == (char)164);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)251, (char)182, (char)152, (char)226, (char)105, (char)196, (char)194, (char)157, (char)21, (char)43, (char)142, (char)196, (char)255, (char)40, (char)52, (char)9, (char)205, (char)76, (char)31, (char)251, (char)153, (char)3, (char)126, (char)116, (char)32, (char)3, (char)34, (char)254, (char)241, (char)250, (char)216, (char)81, (char)36, (char)42, (char)133, (char)14, (char)19, (char)198, (char)104, (char)203, (char)228, (char)44, (char)125, (char)215, (char)190, (char)115, (char)163, (char)186, (char)122, (char)199, (char)43, (char)127, (char)88, (char)62, (char)202, (char)43, (char)221, (char)160, (char)216, (char)17, (char)54, (char)31, (char)138, (char)137, (char)247, (char)215, (char)7, (char)25, (char)153, (char)57, (char)237, (char)20, (char)73, (char)6, (char)51, (char)233, (char)76, (char)8, (char)62, (char)138, (char)235, (char)22, (char)223, (char)185, (char)39, (char)176, (char)110, (char)73, (char)57, (char)188, (char)215, (char)160, (char)64, (char)227, (char)30, (char)237, (char)112, (char)182, (char)225, (char)70, (char)40, (char)78, (char)106, (char)76, (char)188, (char)190, (char)119, (char)154, (char)112, (char)185, (char)152, (char)200, (char)99, (char)175, (char)244, (char)24, (char)216, (char)143, (char)228, (char)15, (char)193, (char)198, (char)208, (char)227, (char)55, (char)117, (char)23, (char)244, (char)72, (char)49, (char)208, (char)67, (char)117, (char)121, (char)105, (char)169, (char)113, (char)3, (char)229, (char)182, (char)18, (char)70, (char)83, (char)121, (char)69, (char)226, (char)201, (char)72, (char)215, (char)117, (char)85, (char)182, (char)2, (char)50, (char)54, (char)24, (char)156, (char)35, (char)96, (char)102, (char)58, (char)104, (char)64, (char)207, (char)76, (char)3, (char)130, (char)165, (char)212, (char)127, (char)133, (char)207, (char)161, (char)19, (char)141, (char)216, (char)52, (char)143, (char)196, (char)234, (char)140, (char)200, (char)172, (char)101, (char)75, (char)90, (char)205, (char)169, (char)189, (char)154, (char)79, (char)90, (char)205, (char)93, (char)186, (char)124, (char)35, (char)118, (char)105, (char)166, (char)96, (char)161, (char)31, (char)24, (char)238, (char)245, (char)103, (char)71, (char)209, (char)151, (char)226, (char)141, (char)181, (char)103, (char)169, (char)172, (char)201, (char)203, (char)234, (char)92, (char)57, (char)7, (char)23, (char)186, (char)28, (char)179, (char)212, (char)131, (char)107, (char)130, (char)239, (char)216, (char)16, (char)33, (char)133, (char)9, (char)108, (char)250, (char)37, (char)114, (char)178, (char)24, (char)107, (char)251, (char)45, (char)91, (char)164, (char)228, (char)5, (char)173, (char)231}));
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_component_SET((char)66) ;
        p110.target_network_SET((char)185) ;
        p110.target_system_SET((char)164) ;
        p110.payload_SET(new char[] {(char)251, (char)182, (char)152, (char)226, (char)105, (char)196, (char)194, (char)157, (char)21, (char)43, (char)142, (char)196, (char)255, (char)40, (char)52, (char)9, (char)205, (char)76, (char)31, (char)251, (char)153, (char)3, (char)126, (char)116, (char)32, (char)3, (char)34, (char)254, (char)241, (char)250, (char)216, (char)81, (char)36, (char)42, (char)133, (char)14, (char)19, (char)198, (char)104, (char)203, (char)228, (char)44, (char)125, (char)215, (char)190, (char)115, (char)163, (char)186, (char)122, (char)199, (char)43, (char)127, (char)88, (char)62, (char)202, (char)43, (char)221, (char)160, (char)216, (char)17, (char)54, (char)31, (char)138, (char)137, (char)247, (char)215, (char)7, (char)25, (char)153, (char)57, (char)237, (char)20, (char)73, (char)6, (char)51, (char)233, (char)76, (char)8, (char)62, (char)138, (char)235, (char)22, (char)223, (char)185, (char)39, (char)176, (char)110, (char)73, (char)57, (char)188, (char)215, (char)160, (char)64, (char)227, (char)30, (char)237, (char)112, (char)182, (char)225, (char)70, (char)40, (char)78, (char)106, (char)76, (char)188, (char)190, (char)119, (char)154, (char)112, (char)185, (char)152, (char)200, (char)99, (char)175, (char)244, (char)24, (char)216, (char)143, (char)228, (char)15, (char)193, (char)198, (char)208, (char)227, (char)55, (char)117, (char)23, (char)244, (char)72, (char)49, (char)208, (char)67, (char)117, (char)121, (char)105, (char)169, (char)113, (char)3, (char)229, (char)182, (char)18, (char)70, (char)83, (char)121, (char)69, (char)226, (char)201, (char)72, (char)215, (char)117, (char)85, (char)182, (char)2, (char)50, (char)54, (char)24, (char)156, (char)35, (char)96, (char)102, (char)58, (char)104, (char)64, (char)207, (char)76, (char)3, (char)130, (char)165, (char)212, (char)127, (char)133, (char)207, (char)161, (char)19, (char)141, (char)216, (char)52, (char)143, (char)196, (char)234, (char)140, (char)200, (char)172, (char)101, (char)75, (char)90, (char)205, (char)169, (char)189, (char)154, (char)79, (char)90, (char)205, (char)93, (char)186, (char)124, (char)35, (char)118, (char)105, (char)166, (char)96, (char)161, (char)31, (char)24, (char)238, (char)245, (char)103, (char)71, (char)209, (char)151, (char)226, (char)141, (char)181, (char)103, (char)169, (char)172, (char)201, (char)203, (char)234, (char)92, (char)57, (char)7, (char)23, (char)186, (char)28, (char)179, (char)212, (char)131, (char)107, (char)130, (char)239, (char)216, (char)16, (char)33, (char)133, (char)9, (char)108, (char)250, (char)37, (char)114, (char)178, (char)24, (char)107, (char)251, (char)45, (char)91, (char)164, (char)228, (char)5, (char)173, (char)231}, 0) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == 5045784751057861124L);
            assert(pack.tc1_GET() == -3790965592969377081L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-3790965592969377081L) ;
        p111.ts1_SET(5045784751057861124L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 9082920537003510417L);
            assert(pack.seq_GET() == 2162472813L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(2162472813L) ;
        p112.time_usec_SET(9082920537003510417L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.fix_type_GET() == (char)58);
            assert(pack.cog_GET() == (char)9353);
            assert(pack.vel_GET() == (char)32272);
            assert(pack.eph_GET() == (char)37886);
            assert(pack.vd_GET() == (short) -13687);
            assert(pack.lon_GET() == 1236131094);
            assert(pack.satellites_visible_GET() == (char)223);
            assert(pack.time_usec_GET() == 4893975870855757857L);
            assert(pack.lat_GET() == -243184814);
            assert(pack.vn_GET() == (short)8404);
            assert(pack.epv_GET() == (char)60225);
            assert(pack.ve_GET() == (short)9045);
            assert(pack.alt_GET() == -757332571);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.vd_SET((short) -13687) ;
        p113.lat_SET(-243184814) ;
        p113.fix_type_SET((char)58) ;
        p113.alt_SET(-757332571) ;
        p113.eph_SET((char)37886) ;
        p113.epv_SET((char)60225) ;
        p113.time_usec_SET(4893975870855757857L) ;
        p113.satellites_visible_SET((char)223) ;
        p113.cog_SET((char)9353) ;
        p113.lon_SET(1236131094) ;
        p113.vn_SET((short)8404) ;
        p113.ve_SET((short)9045) ;
        p113.vel_SET((char)32272) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.sensor_id_GET() == (char)235);
            assert(pack.integration_time_us_GET() == 622990165L);
            assert(pack.distance_GET() == -3.3586861E38F);
            assert(pack.integrated_zgyro_GET() == 2.7245951E38F);
            assert(pack.temperature_GET() == (short)5156);
            assert(pack.quality_GET() == (char)21);
            assert(pack.time_delta_distance_us_GET() == 4020199283L);
            assert(pack.time_usec_GET() == 3879679082391336826L);
            assert(pack.integrated_xgyro_GET() == 7.122503E37F);
            assert(pack.integrated_y_GET() == -1.7542627E38F);
            assert(pack.integrated_ygyro_GET() == -7.3305537E37F);
            assert(pack.integrated_x_GET() == -4.5228916E37F);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_xgyro_SET(7.122503E37F) ;
        p114.integrated_y_SET(-1.7542627E38F) ;
        p114.time_usec_SET(3879679082391336826L) ;
        p114.integrated_x_SET(-4.5228916E37F) ;
        p114.integration_time_us_SET(622990165L) ;
        p114.integrated_ygyro_SET(-7.3305537E37F) ;
        p114.time_delta_distance_us_SET(4020199283L) ;
        p114.temperature_SET((short)5156) ;
        p114.distance_SET(-3.3586861E38F) ;
        p114.quality_SET((char)21) ;
        p114.sensor_id_SET((char)235) ;
        p114.integrated_zgyro_SET(2.7245951E38F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short)26543);
            assert(pack.lat_GET() == 805410429);
            assert(pack.alt_GET() == -479140820);
            assert(pack.time_usec_GET() == 1674937752668090216L);
            assert(pack.true_airspeed_GET() == (char)443);
            assert(pack.pitchspeed_GET() == -2.9502178E38F);
            assert(pack.lon_GET() == 749511161);
            assert(pack.vx_GET() == (short) -31267);
            assert(pack.vz_GET() == (short) -4803);
            assert(pack.vy_GET() == (short)26487);
            assert(pack.yacc_GET() == (short) -2360);
            assert(pack.rollspeed_GET() == -1.9668798E38F);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {3.6439489E37F, 1.5982012E38F, 1.1599794E38F, -8.599127E37F}));
            assert(pack.xacc_GET() == (short) -15957);
            assert(pack.ind_airspeed_GET() == (char)62519);
            assert(pack.yawspeed_GET() == 9.764909E37F);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.yacc_SET((short) -2360) ;
        p115.rollspeed_SET(-1.9668798E38F) ;
        p115.lat_SET(805410429) ;
        p115.ind_airspeed_SET((char)62519) ;
        p115.lon_SET(749511161) ;
        p115.vx_SET((short) -31267) ;
        p115.pitchspeed_SET(-2.9502178E38F) ;
        p115.time_usec_SET(1674937752668090216L) ;
        p115.xacc_SET((short) -15957) ;
        p115.attitude_quaternion_SET(new float[] {3.6439489E37F, 1.5982012E38F, 1.1599794E38F, -8.599127E37F}, 0) ;
        p115.yawspeed_SET(9.764909E37F) ;
        p115.zacc_SET((short)26543) ;
        p115.true_airspeed_SET((char)443) ;
        p115.vz_SET((short) -4803) ;
        p115.vy_SET((short)26487) ;
        p115.alt_SET(-479140820) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == (short) -30524);
            assert(pack.ymag_GET() == (short)2995);
            assert(pack.xacc_GET() == (short) -4858);
            assert(pack.zgyro_GET() == (short)9049);
            assert(pack.ygyro_GET() == (short) -18759);
            assert(pack.zacc_GET() == (short)32735);
            assert(pack.time_boot_ms_GET() == 91414611L);
            assert(pack.xmag_GET() == (short) -8590);
            assert(pack.xgyro_GET() == (short)6267);
            assert(pack.yacc_GET() == (short)27720);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.zacc_SET((short)32735) ;
        p116.zgyro_SET((short)9049) ;
        p116.xgyro_SET((short)6267) ;
        p116.time_boot_ms_SET(91414611L) ;
        p116.xmag_SET((short) -8590) ;
        p116.xacc_SET((short) -4858) ;
        p116.yacc_SET((short)27720) ;
        p116.ymag_SET((short)2995) ;
        p116.zmag_SET((short) -30524) ;
        p116.ygyro_SET((short) -18759) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)84);
            assert(pack.end_GET() == (char)5763);
            assert(pack.target_component_GET() == (char)52);
            assert(pack.start_GET() == (char)55447);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.start_SET((char)55447) ;
        p117.end_SET((char)5763) ;
        p117.target_component_SET((char)52) ;
        p117.target_system_SET((char)84) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.num_logs_GET() == (char)12980);
            assert(pack.id_GET() == (char)32637);
            assert(pack.last_log_num_GET() == (char)57809);
            assert(pack.size_GET() == 137747985L);
            assert(pack.time_utc_GET() == 1597668027L);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.last_log_num_SET((char)57809) ;
        p118.size_SET(137747985L) ;
        p118.num_logs_SET((char)12980) ;
        p118.time_utc_SET(1597668027L) ;
        p118.id_SET((char)32637) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.ofs_GET() == 380486424L);
            assert(pack.id_GET() == (char)19410);
            assert(pack.count_GET() == 2685514259L);
            assert(pack.target_system_GET() == (char)150);
            assert(pack.target_component_GET() == (char)63);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.ofs_SET(380486424L) ;
        p119.target_component_SET((char)63) ;
        p119.id_SET((char)19410) ;
        p119.count_SET(2685514259L) ;
        p119.target_system_SET((char)150) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)13764);
            assert(pack.count_GET() == (char)186);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)252, (char)71, (char)180, (char)136, (char)70, (char)54, (char)152, (char)96, (char)182, (char)210, (char)18, (char)212, (char)246, (char)134, (char)106, (char)234, (char)198, (char)74, (char)148, (char)223, (char)88, (char)177, (char)157, (char)82, (char)36, (char)157, (char)242, (char)46, (char)254, (char)151, (char)52, (char)242, (char)27, (char)130, (char)33, (char)95, (char)253, (char)112, (char)16, (char)209, (char)153, (char)150, (char)120, (char)103, (char)185, (char)52, (char)116, (char)124, (char)183, (char)86, (char)195, (char)50, (char)19, (char)23, (char)186, (char)12, (char)80, (char)55, (char)89, (char)60, (char)173, (char)203, (char)110, (char)211, (char)214, (char)8, (char)39, (char)229, (char)214, (char)23, (char)78, (char)156, (char)25, (char)175, (char)79, (char)114, (char)34, (char)71, (char)64, (char)189, (char)49, (char)111, (char)131, (char)97, (char)236, (char)165, (char)136, (char)59, (char)198, (char)101}));
            assert(pack.ofs_GET() == 2190258412L);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.count_SET((char)186) ;
        p120.data__SET(new char[] {(char)252, (char)71, (char)180, (char)136, (char)70, (char)54, (char)152, (char)96, (char)182, (char)210, (char)18, (char)212, (char)246, (char)134, (char)106, (char)234, (char)198, (char)74, (char)148, (char)223, (char)88, (char)177, (char)157, (char)82, (char)36, (char)157, (char)242, (char)46, (char)254, (char)151, (char)52, (char)242, (char)27, (char)130, (char)33, (char)95, (char)253, (char)112, (char)16, (char)209, (char)153, (char)150, (char)120, (char)103, (char)185, (char)52, (char)116, (char)124, (char)183, (char)86, (char)195, (char)50, (char)19, (char)23, (char)186, (char)12, (char)80, (char)55, (char)89, (char)60, (char)173, (char)203, (char)110, (char)211, (char)214, (char)8, (char)39, (char)229, (char)214, (char)23, (char)78, (char)156, (char)25, (char)175, (char)79, (char)114, (char)34, (char)71, (char)64, (char)189, (char)49, (char)111, (char)131, (char)97, (char)236, (char)165, (char)136, (char)59, (char)198, (char)101}, 0) ;
        p120.ofs_SET(2190258412L) ;
        p120.id_SET((char)13764) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)217);
            assert(pack.target_component_GET() == (char)53);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)53) ;
        p121.target_system_SET((char)217) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)51);
            assert(pack.target_system_GET() == (char)17);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)17) ;
        p122.target_component_SET((char)51) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)175, (char)150, (char)184, (char)200, (char)95, (char)25, (char)200, (char)51, (char)151, (char)206, (char)236, (char)60, (char)125, (char)225, (char)172, (char)64, (char)220, (char)184, (char)63, (char)80, (char)101, (char)76, (char)112, (char)190, (char)21, (char)177, (char)125, (char)30, (char)197, (char)205, (char)52, (char)122, (char)17, (char)250, (char)175, (char)70, (char)177, (char)86, (char)248, (char)94, (char)80, (char)88, (char)33, (char)190, (char)240, (char)175, (char)28, (char)237, (char)101, (char)138, (char)57, (char)47, (char)225, (char)158, (char)205, (char)190, (char)190, (char)240, (char)98, (char)229, (char)116, (char)217, (char)79, (char)6, (char)228, (char)232, (char)178, (char)26, (char)127, (char)193, (char)73, (char)249, (char)89, (char)114, (char)143, (char)168, (char)97, (char)108, (char)161, (char)16, (char)128, (char)13, (char)223, (char)16, (char)198, (char)75, (char)137, (char)239, (char)115, (char)26, (char)221, (char)193, (char)249, (char)147, (char)42, (char)86, (char)191, (char)209, (char)187, (char)64, (char)4, (char)219, (char)85, (char)194, (char)202, (char)166, (char)103, (char)208, (char)31, (char)70}));
            assert(pack.target_component_GET() == (char)235);
            assert(pack.len_GET() == (char)167);
            assert(pack.target_system_GET() == (char)134);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_component_SET((char)235) ;
        p123.len_SET((char)167) ;
        p123.data__SET(new char[] {(char)175, (char)150, (char)184, (char)200, (char)95, (char)25, (char)200, (char)51, (char)151, (char)206, (char)236, (char)60, (char)125, (char)225, (char)172, (char)64, (char)220, (char)184, (char)63, (char)80, (char)101, (char)76, (char)112, (char)190, (char)21, (char)177, (char)125, (char)30, (char)197, (char)205, (char)52, (char)122, (char)17, (char)250, (char)175, (char)70, (char)177, (char)86, (char)248, (char)94, (char)80, (char)88, (char)33, (char)190, (char)240, (char)175, (char)28, (char)237, (char)101, (char)138, (char)57, (char)47, (char)225, (char)158, (char)205, (char)190, (char)190, (char)240, (char)98, (char)229, (char)116, (char)217, (char)79, (char)6, (char)228, (char)232, (char)178, (char)26, (char)127, (char)193, (char)73, (char)249, (char)89, (char)114, (char)143, (char)168, (char)97, (char)108, (char)161, (char)16, (char)128, (char)13, (char)223, (char)16, (char)198, (char)75, (char)137, (char)239, (char)115, (char)26, (char)221, (char)193, (char)249, (char)147, (char)42, (char)86, (char)191, (char)209, (char)187, (char)64, (char)4, (char)219, (char)85, (char)194, (char)202, (char)166, (char)103, (char)208, (char)31, (char)70}, 0) ;
        p123.target_system_SET((char)134) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -577546680);
            assert(pack.vel_GET() == (char)29910);
            assert(pack.cog_GET() == (char)52500);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
            assert(pack.eph_GET() == (char)17788);
            assert(pack.lon_GET() == -1669559516);
            assert(pack.alt_GET() == -1569573321);
            assert(pack.dgps_numch_GET() == (char)59);
            assert(pack.epv_GET() == (char)34433);
            assert(pack.time_usec_GET() == 7212258987794404880L);
            assert(pack.dgps_age_GET() == 4185467896L);
            assert(pack.satellites_visible_GET() == (char)134);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS) ;
        p124.dgps_numch_SET((char)59) ;
        p124.dgps_age_SET(4185467896L) ;
        p124.lat_SET(-577546680) ;
        p124.satellites_visible_SET((char)134) ;
        p124.alt_SET(-1569573321) ;
        p124.time_usec_SET(7212258987794404880L) ;
        p124.epv_SET((char)34433) ;
        p124.eph_SET((char)17788) ;
        p124.cog_SET((char)52500) ;
        p124.vel_SET((char)29910) ;
        p124.lon_SET(-1669559516) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID));
            assert(pack.Vservo_GET() == (char)12403);
            assert(pack.Vcc_GET() == (char)41059);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID)) ;
        p125.Vservo_SET((char)12403) ;
        p125.Vcc_SET((char)41059) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.timeout_GET() == (char)61736);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
            assert(pack.count_GET() == (char)221);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY));
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)229, (char)176, (char)181, (char)123, (char)93, (char)118, (char)59, (char)52, (char)231, (char)32, (char)10, (char)60, (char)31, (char)136, (char)149, (char)151, (char)246, (char)64, (char)96, (char)70, (char)87, (char)144, (char)101, (char)124, (char)80, (char)179, (char)13, (char)109, (char)87, (char)56, (char)2, (char)190, (char)131, (char)113, (char)246, (char)57, (char)128, (char)11, (char)91, (char)236, (char)163, (char)235, (char)150, (char)229, (char)61, (char)90, (char)113, (char)56, (char)255, (char)195, (char)33, (char)214, (char)123, (char)115, (char)192, (char)127, (char)71, (char)7, (char)199, (char)57, (char)241, (char)167, (char)137, (char)249, (char)23, (char)64, (char)215, (char)230, (char)183, (char)24}));
            assert(pack.baudrate_GET() == 2216532327L);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.baudrate_SET(2216532327L) ;
        p126.timeout_SET((char)61736) ;
        p126.data__SET(new char[] {(char)229, (char)176, (char)181, (char)123, (char)93, (char)118, (char)59, (char)52, (char)231, (char)32, (char)10, (char)60, (char)31, (char)136, (char)149, (char)151, (char)246, (char)64, (char)96, (char)70, (char)87, (char)144, (char)101, (char)124, (char)80, (char)179, (char)13, (char)109, (char)87, (char)56, (char)2, (char)190, (char)131, (char)113, (char)246, (char)57, (char)128, (char)11, (char)91, (char)236, (char)163, (char)235, (char)150, (char)229, (char)61, (char)90, (char)113, (char)56, (char)255, (char)195, (char)33, (char)214, (char)123, (char)115, (char)192, (char)127, (char)71, (char)7, (char)199, (char)57, (char)241, (char)167, (char)137, (char)249, (char)23, (char)64, (char)215, (char)230, (char)183, (char)24}, 0) ;
        p126.count_SET((char)221) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY)) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.wn_GET() == (char)43336);
            assert(pack.nsats_GET() == (char)133);
            assert(pack.iar_num_hypotheses_GET() == -1866911935);
            assert(pack.baseline_coords_type_GET() == (char)58);
            assert(pack.accuracy_GET() == 1811935339L);
            assert(pack.rtk_rate_GET() == (char)37);
            assert(pack.baseline_b_mm_GET() == -1677030811);
            assert(pack.baseline_c_mm_GET() == -2032468042);
            assert(pack.tow_GET() == 2188471195L);
            assert(pack.time_last_baseline_ms_GET() == 474373856L);
            assert(pack.baseline_a_mm_GET() == 1704374313);
            assert(pack.rtk_health_GET() == (char)75);
            assert(pack.rtk_receiver_id_GET() == (char)117);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.nsats_SET((char)133) ;
        p127.rtk_health_SET((char)75) ;
        p127.time_last_baseline_ms_SET(474373856L) ;
        p127.rtk_rate_SET((char)37) ;
        p127.rtk_receiver_id_SET((char)117) ;
        p127.wn_SET((char)43336) ;
        p127.baseline_c_mm_SET(-2032468042) ;
        p127.baseline_a_mm_SET(1704374313) ;
        p127.accuracy_SET(1811935339L) ;
        p127.baseline_b_mm_SET(-1677030811) ;
        p127.iar_num_hypotheses_SET(-1866911935) ;
        p127.baseline_coords_type_SET((char)58) ;
        p127.tow_SET(2188471195L) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_c_mm_GET() == -1081095510);
            assert(pack.rtk_receiver_id_GET() == (char)148);
            assert(pack.tow_GET() == 2098177156L);
            assert(pack.baseline_coords_type_GET() == (char)114);
            assert(pack.iar_num_hypotheses_GET() == -1169468722);
            assert(pack.time_last_baseline_ms_GET() == 3739130576L);
            assert(pack.accuracy_GET() == 2589450619L);
            assert(pack.rtk_health_GET() == (char)155);
            assert(pack.wn_GET() == (char)61231);
            assert(pack.baseline_a_mm_GET() == -487666185);
            assert(pack.baseline_b_mm_GET() == 871782962);
            assert(pack.nsats_GET() == (char)209);
            assert(pack.rtk_rate_GET() == (char)6);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(3739130576L) ;
        p128.baseline_c_mm_SET(-1081095510) ;
        p128.iar_num_hypotheses_SET(-1169468722) ;
        p128.baseline_b_mm_SET(871782962) ;
        p128.rtk_receiver_id_SET((char)148) ;
        p128.tow_SET(2098177156L) ;
        p128.baseline_coords_type_SET((char)114) ;
        p128.baseline_a_mm_SET(-487666185) ;
        p128.wn_SET((char)61231) ;
        p128.rtk_health_SET((char)155) ;
        p128.accuracy_SET(2589450619L) ;
        p128.rtk_rate_SET((char)6) ;
        p128.nsats_SET((char)209) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short)14844);
            assert(pack.zmag_GET() == (short)1800);
            assert(pack.ymag_GET() == (short) -21231);
            assert(pack.ygyro_GET() == (short) -13902);
            assert(pack.xgyro_GET() == (short) -22315);
            assert(pack.time_boot_ms_GET() == 4131631103L);
            assert(pack.zgyro_GET() == (short) -9386);
            assert(pack.zacc_GET() == (short)10782);
            assert(pack.xmag_GET() == (short) -12398);
            assert(pack.xacc_GET() == (short) -22116);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.xgyro_SET((short) -22315) ;
        p129.xacc_SET((short) -22116) ;
        p129.time_boot_ms_SET(4131631103L) ;
        p129.zacc_SET((short)10782) ;
        p129.ygyro_SET((short) -13902) ;
        p129.ymag_SET((short) -21231) ;
        p129.yacc_SET((short)14844) ;
        p129.zmag_SET((short)1800) ;
        p129.xmag_SET((short) -12398) ;
        p129.zgyro_SET((short) -9386) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)94);
            assert(pack.packets_GET() == (char)33659);
            assert(pack.size_GET() == 1150730497L);
            assert(pack.payload_GET() == (char)123);
            assert(pack.height_GET() == (char)13403);
            assert(pack.width_GET() == (char)45510);
            assert(pack.jpg_quality_GET() == (char)217);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.width_SET((char)45510) ;
        p130.size_SET(1150730497L) ;
        p130.type_SET((char)94) ;
        p130.payload_SET((char)123) ;
        p130.jpg_quality_SET((char)217) ;
        p130.height_SET((char)13403) ;
        p130.packets_SET((char)33659) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)181, (char)64, (char)244, (char)224, (char)174, (char)182, (char)114, (char)192, (char)110, (char)151, (char)232, (char)125, (char)49, (char)138, (char)193, (char)55, (char)240, (char)141, (char)213, (char)84, (char)105, (char)63, (char)165, (char)3, (char)45, (char)142, (char)79, (char)233, (char)14, (char)133, (char)205, (char)214, (char)202, (char)206, (char)99, (char)23, (char)179, (char)219, (char)85, (char)136, (char)111, (char)159, (char)19, (char)225, (char)222, (char)28, (char)114, (char)45, (char)96, (char)119, (char)204, (char)32, (char)255, (char)3, (char)4, (char)36, (char)176, (char)210, (char)144, (char)165, (char)253, (char)121, (char)27, (char)50, (char)121, (char)157, (char)120, (char)194, (char)229, (char)227, (char)200, (char)135, (char)114, (char)42, (char)223, (char)127, (char)43, (char)178, (char)232, (char)208, (char)113, (char)123, (char)31, (char)10, (char)64, (char)89, (char)24, (char)214, (char)180, (char)242, (char)238, (char)28, (char)190, (char)176, (char)204, (char)194, (char)13, (char)0, (char)192, (char)129, (char)8, (char)78, (char)41, (char)23, (char)32, (char)54, (char)68, (char)128, (char)144, (char)245, (char)170, (char)178, (char)215, (char)202, (char)121, (char)142, (char)204, (char)140, (char)73, (char)112, (char)198, (char)139, (char)226, (char)127, (char)149, (char)43, (char)88, (char)139, (char)12, (char)220, (char)19, (char)125, (char)9, (char)57, (char)29, (char)192, (char)194, (char)59, (char)122, (char)252, (char)173, (char)33, (char)128, (char)236, (char)249, (char)55, (char)26, (char)18, (char)91, (char)202, (char)133, (char)55, (char)248, (char)222, (char)29, (char)71, (char)37, (char)130, (char)66, (char)144, (char)20, (char)130, (char)158, (char)44, (char)71, (char)194, (char)109, (char)192, (char)113, (char)38, (char)236, (char)14, (char)145, (char)173, (char)1, (char)129, (char)183, (char)99, (char)230, (char)54, (char)59, (char)5, (char)208, (char)204, (char)65, (char)252, (char)166, (char)200, (char)14, (char)69, (char)233, (char)202, (char)244, (char)196, (char)162, (char)101, (char)248, (char)189, (char)164, (char)208, (char)134, (char)19, (char)248, (char)82, (char)217, (char)188, (char)24, (char)55, (char)94, (char)64, (char)10, (char)52, (char)168, (char)252, (char)242, (char)51, (char)85, (char)193, (char)194, (char)9, (char)131, (char)164, (char)45, (char)23, (char)29, (char)70, (char)157, (char)37, (char)89, (char)33, (char)134, (char)121, (char)240, (char)48, (char)36, (char)107, (char)201, (char)134, (char)27, (char)224, (char)3, (char)27, (char)98, (char)116, (char)94, (char)147, (char)72, (char)241, (char)98, (char)115, (char)204, (char)80, (char)84}));
            assert(pack.seqnr_GET() == (char)54547);
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)54547) ;
        p131.data__SET(new char[] {(char)181, (char)64, (char)244, (char)224, (char)174, (char)182, (char)114, (char)192, (char)110, (char)151, (char)232, (char)125, (char)49, (char)138, (char)193, (char)55, (char)240, (char)141, (char)213, (char)84, (char)105, (char)63, (char)165, (char)3, (char)45, (char)142, (char)79, (char)233, (char)14, (char)133, (char)205, (char)214, (char)202, (char)206, (char)99, (char)23, (char)179, (char)219, (char)85, (char)136, (char)111, (char)159, (char)19, (char)225, (char)222, (char)28, (char)114, (char)45, (char)96, (char)119, (char)204, (char)32, (char)255, (char)3, (char)4, (char)36, (char)176, (char)210, (char)144, (char)165, (char)253, (char)121, (char)27, (char)50, (char)121, (char)157, (char)120, (char)194, (char)229, (char)227, (char)200, (char)135, (char)114, (char)42, (char)223, (char)127, (char)43, (char)178, (char)232, (char)208, (char)113, (char)123, (char)31, (char)10, (char)64, (char)89, (char)24, (char)214, (char)180, (char)242, (char)238, (char)28, (char)190, (char)176, (char)204, (char)194, (char)13, (char)0, (char)192, (char)129, (char)8, (char)78, (char)41, (char)23, (char)32, (char)54, (char)68, (char)128, (char)144, (char)245, (char)170, (char)178, (char)215, (char)202, (char)121, (char)142, (char)204, (char)140, (char)73, (char)112, (char)198, (char)139, (char)226, (char)127, (char)149, (char)43, (char)88, (char)139, (char)12, (char)220, (char)19, (char)125, (char)9, (char)57, (char)29, (char)192, (char)194, (char)59, (char)122, (char)252, (char)173, (char)33, (char)128, (char)236, (char)249, (char)55, (char)26, (char)18, (char)91, (char)202, (char)133, (char)55, (char)248, (char)222, (char)29, (char)71, (char)37, (char)130, (char)66, (char)144, (char)20, (char)130, (char)158, (char)44, (char)71, (char)194, (char)109, (char)192, (char)113, (char)38, (char)236, (char)14, (char)145, (char)173, (char)1, (char)129, (char)183, (char)99, (char)230, (char)54, (char)59, (char)5, (char)208, (char)204, (char)65, (char)252, (char)166, (char)200, (char)14, (char)69, (char)233, (char)202, (char)244, (char)196, (char)162, (char)101, (char)248, (char)189, (char)164, (char)208, (char)134, (char)19, (char)248, (char)82, (char)217, (char)188, (char)24, (char)55, (char)94, (char)64, (char)10, (char)52, (char)168, (char)252, (char)242, (char)51, (char)85, (char)193, (char)194, (char)9, (char)131, (char)164, (char)45, (char)23, (char)29, (char)70, (char)157, (char)37, (char)89, (char)33, (char)134, (char)121, (char)240, (char)48, (char)36, (char)107, (char)201, (char)134, (char)27, (char)224, (char)3, (char)27, (char)98, (char)116, (char)94, (char)147, (char)72, (char)241, (char)98, (char)115, (char)204, (char)80, (char)84}, 0) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.max_distance_GET() == (char)8233);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.min_distance_GET() == (char)26215);
            assert(pack.covariance_GET() == (char)56);
            assert(pack.time_boot_ms_GET() == 3179410199L);
            assert(pack.id_GET() == (char)177);
            assert(pack.current_distance_GET() == (char)54727);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_PITCH_90);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.current_distance_SET((char)54727) ;
        p132.min_distance_SET((char)26215) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p132.time_boot_ms_SET(3179410199L) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_PITCH_90) ;
        p132.id_SET((char)177) ;
        p132.covariance_SET((char)56) ;
        p132.max_distance_SET((char)8233) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 506594960);
            assert(pack.grid_spacing_GET() == (char)61264);
            assert(pack.mask_GET() == 2168811723810215647L);
            assert(pack.lat_GET() == 240843297);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lon_SET(506594960) ;
        p133.grid_spacing_SET((char)61264) ;
        p133.lat_SET(240843297) ;
        p133.mask_SET(2168811723810215647L) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -193136100);
            assert(pack.lat_GET() == -812778498);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -3735, (short)27196, (short) -22551, (short)13205, (short)4308, (short) -427, (short) -8264, (short)14255, (short)28196, (short)25701, (short)12763, (short) -12193, (short) -23806, (short)23632, (short) -15355, (short) -25236}));
            assert(pack.gridbit_GET() == (char)137);
            assert(pack.grid_spacing_GET() == (char)35687);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(-812778498) ;
        p134.data__SET(new short[] {(short) -3735, (short)27196, (short) -22551, (short)13205, (short)4308, (short) -427, (short) -8264, (short)14255, (short)28196, (short)25701, (short)12763, (short) -12193, (short) -23806, (short)23632, (short) -15355, (short) -25236}, 0) ;
        p134.gridbit_SET((char)137) ;
        p134.lon_SET(-193136100) ;
        p134.grid_spacing_SET((char)35687) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1669286665);
            assert(pack.lat_GET() == 768944796);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(-1669286665) ;
        p135.lat_SET(768944796) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.loaded_GET() == (char)51147);
            assert(pack.current_height_GET() == 1.5833216E38F);
            assert(pack.lon_GET() == -30742549);
            assert(pack.lat_GET() == 1003537728);
            assert(pack.spacing_GET() == (char)7011);
            assert(pack.terrain_height_GET() == -1.3878715E38F);
            assert(pack.pending_GET() == (char)1801);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.terrain_height_SET(-1.3878715E38F) ;
        p136.loaded_SET((char)51147) ;
        p136.lat_SET(1003537728) ;
        p136.lon_SET(-30742549) ;
        p136.pending_SET((char)1801) ;
        p136.current_height_SET(1.5833216E38F) ;
        p136.spacing_SET((char)7011) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 3.2343392E36F);
            assert(pack.temperature_GET() == (short) -19665);
            assert(pack.press_diff_GET() == 2.1963278E38F);
            assert(pack.time_boot_ms_GET() == 2276494249L);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(2276494249L) ;
        p137.temperature_SET((short) -19665) ;
        p137.press_diff_SET(2.1963278E38F) ;
        p137.press_abs_SET(3.2343392E36F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -2.5802776E38F);
            assert(pack.time_usec_GET() == 9074189784355562076L);
            assert(pack.z_GET() == 2.4142063E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.4316047E38F, 1.929559E38F, 1.1424938E38F, -1.1217533E38F}));
            assert(pack.y_GET() == -1.1262748E38F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.x_SET(-2.5802776E38F) ;
        p138.time_usec_SET(9074189784355562076L) ;
        p138.z_SET(2.4142063E38F) ;
        p138.q_SET(new float[] {1.4316047E38F, 1.929559E38F, 1.1424938E38F, -1.1217533E38F}, 0) ;
        p138.y_SET(-1.1262748E38F) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)60);
            assert(pack.time_usec_GET() == 8799445485400615059L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.748622E38F, 2.1118246E38F, -2.6412658E38F, -1.2089037E38F, -3.1731773E38F, -2.1484455E38F, 9.497901E37F, 1.9876516E38F}));
            assert(pack.target_component_GET() == (char)111);
            assert(pack.target_system_GET() == (char)2);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(8799445485400615059L) ;
        p139.group_mlx_SET((char)60) ;
        p139.controls_SET(new float[] {-2.748622E38F, 2.1118246E38F, -2.6412658E38F, -1.2089037E38F, -3.1731773E38F, -2.1484455E38F, 9.497901E37F, 1.9876516E38F}, 0) ;
        p139.target_system_SET((char)2) ;
        p139.target_component_SET((char)111) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.0279787E38F, -1.6234053E38F, -2.3967068E38F, -8.827919E37F, -3.1296853E38F, 1.8139774E38F, 2.1287312E37F, 4.408689E37F}));
            assert(pack.time_usec_GET() == 727671232342045112L);
            assert(pack.group_mlx_GET() == (char)151);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(727671232342045112L) ;
        p140.controls_SET(new float[] {2.0279787E38F, -1.6234053E38F, -2.3967068E38F, -8.827919E37F, -3.1296853E38F, 1.8139774E38F, 2.1287312E37F, 4.408689E37F}, 0) ;
        p140.group_mlx_SET((char)151) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.bottom_clearance_GET() == 1.4695216E38F);
            assert(pack.time_usec_GET() == 6911593734468618968L);
            assert(pack.altitude_terrain_GET() == -2.3559459E38F);
            assert(pack.altitude_monotonic_GET() == -1.0886131E38F);
            assert(pack.altitude_relative_GET() == -1.3861702E38F);
            assert(pack.altitude_local_GET() == 3.250372E38F);
            assert(pack.altitude_amsl_GET() == -2.4354446E38F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_terrain_SET(-2.3559459E38F) ;
        p141.altitude_relative_SET(-1.3861702E38F) ;
        p141.altitude_monotonic_SET(-1.0886131E38F) ;
        p141.altitude_local_SET(3.250372E38F) ;
        p141.time_usec_SET(6911593734468618968L) ;
        p141.bottom_clearance_SET(1.4695216E38F) ;
        p141.altitude_amsl_SET(-2.4354446E38F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)1, (char)137, (char)29, (char)130, (char)107, (char)63, (char)88, (char)233, (char)111, (char)115, (char)73, (char)93, (char)119, (char)130, (char)152, (char)71, (char)32, (char)98, (char)150, (char)43, (char)125, (char)244, (char)156, (char)94, (char)196, (char)118, (char)4, (char)56, (char)100, (char)105, (char)118, (char)206, (char)83, (char)227, (char)20, (char)237, (char)95, (char)92, (char)13, (char)135, (char)36, (char)150, (char)255, (char)35, (char)246, (char)95, (char)170, (char)150, (char)83, (char)80, (char)96, (char)61, (char)218, (char)187, (char)71, (char)105, (char)152, (char)78, (char)233, (char)28, (char)89, (char)249, (char)1, (char)17, (char)170, (char)59, (char)131, (char)101, (char)63, (char)156, (char)91, (char)176, (char)152, (char)182, (char)116, (char)143, (char)248, (char)79, (char)39, (char)255, (char)69, (char)5, (char)112, (char)46, (char)17, (char)105, (char)242, (char)178, (char)20, (char)201, (char)64, (char)63, (char)55, (char)179, (char)145, (char)239, (char)45, (char)80, (char)4, (char)29, (char)138, (char)251, (char)177, (char)66, (char)183, (char)240, (char)201, (char)187, (char)38, (char)121, (char)89, (char)169, (char)198, (char)18, (char)182, (char)122, (char)187, (char)68, (char)132, (char)99}));
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)249, (char)81, (char)96, (char)1, (char)34, (char)150, (char)27, (char)120, (char)241, (char)165, (char)130, (char)0, (char)170, (char)135, (char)67, (char)149, (char)19, (char)239, (char)118, (char)209, (char)40, (char)68, (char)23, (char)47, (char)138, (char)104, (char)154, (char)63, (char)151, (char)190, (char)84, (char)80, (char)193, (char)76, (char)143, (char)85, (char)41, (char)181, (char)217, (char)40, (char)41, (char)77, (char)196, (char)236, (char)11, (char)177, (char)46, (char)253, (char)107, (char)45, (char)171, (char)12, (char)255, (char)106, (char)21, (char)11, (char)167, (char)24, (char)251, (char)238, (char)118, (char)200, (char)243, (char)34, (char)29, (char)158, (char)196, (char)11, (char)111, (char)29, (char)91, (char)251, (char)108, (char)88, (char)231, (char)223, (char)137, (char)113, (char)5, (char)186, (char)204, (char)70, (char)22, (char)63, (char)221, (char)192, (char)113, (char)118, (char)154, (char)75, (char)143, (char)154, (char)238, (char)237, (char)171, (char)48, (char)61, (char)222, (char)33, (char)119, (char)115, (char)235, (char)88, (char)47, (char)29, (char)98, (char)60, (char)131, (char)10, (char)233, (char)68, (char)4, (char)205, (char)120, (char)192, (char)229, (char)128, (char)120, (char)185, (char)49}));
            assert(pack.request_id_GET() == (char)215);
            assert(pack.uri_type_GET() == (char)194);
            assert(pack.transfer_type_GET() == (char)70);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_SET(new char[] {(char)249, (char)81, (char)96, (char)1, (char)34, (char)150, (char)27, (char)120, (char)241, (char)165, (char)130, (char)0, (char)170, (char)135, (char)67, (char)149, (char)19, (char)239, (char)118, (char)209, (char)40, (char)68, (char)23, (char)47, (char)138, (char)104, (char)154, (char)63, (char)151, (char)190, (char)84, (char)80, (char)193, (char)76, (char)143, (char)85, (char)41, (char)181, (char)217, (char)40, (char)41, (char)77, (char)196, (char)236, (char)11, (char)177, (char)46, (char)253, (char)107, (char)45, (char)171, (char)12, (char)255, (char)106, (char)21, (char)11, (char)167, (char)24, (char)251, (char)238, (char)118, (char)200, (char)243, (char)34, (char)29, (char)158, (char)196, (char)11, (char)111, (char)29, (char)91, (char)251, (char)108, (char)88, (char)231, (char)223, (char)137, (char)113, (char)5, (char)186, (char)204, (char)70, (char)22, (char)63, (char)221, (char)192, (char)113, (char)118, (char)154, (char)75, (char)143, (char)154, (char)238, (char)237, (char)171, (char)48, (char)61, (char)222, (char)33, (char)119, (char)115, (char)235, (char)88, (char)47, (char)29, (char)98, (char)60, (char)131, (char)10, (char)233, (char)68, (char)4, (char)205, (char)120, (char)192, (char)229, (char)128, (char)120, (char)185, (char)49}, 0) ;
        p142.request_id_SET((char)215) ;
        p142.uri_type_SET((char)194) ;
        p142.transfer_type_SET((char)70) ;
        p142.storage_SET(new char[] {(char)1, (char)137, (char)29, (char)130, (char)107, (char)63, (char)88, (char)233, (char)111, (char)115, (char)73, (char)93, (char)119, (char)130, (char)152, (char)71, (char)32, (char)98, (char)150, (char)43, (char)125, (char)244, (char)156, (char)94, (char)196, (char)118, (char)4, (char)56, (char)100, (char)105, (char)118, (char)206, (char)83, (char)227, (char)20, (char)237, (char)95, (char)92, (char)13, (char)135, (char)36, (char)150, (char)255, (char)35, (char)246, (char)95, (char)170, (char)150, (char)83, (char)80, (char)96, (char)61, (char)218, (char)187, (char)71, (char)105, (char)152, (char)78, (char)233, (char)28, (char)89, (char)249, (char)1, (char)17, (char)170, (char)59, (char)131, (char)101, (char)63, (char)156, (char)91, (char)176, (char)152, (char)182, (char)116, (char)143, (char)248, (char)79, (char)39, (char)255, (char)69, (char)5, (char)112, (char)46, (char)17, (char)105, (char)242, (char)178, (char)20, (char)201, (char)64, (char)63, (char)55, (char)179, (char)145, (char)239, (char)45, (char)80, (char)4, (char)29, (char)138, (char)251, (char)177, (char)66, (char)183, (char)240, (char)201, (char)187, (char)38, (char)121, (char)89, (char)169, (char)198, (char)18, (char)182, (char)122, (char)187, (char)68, (char)132, (char)99}, 0) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1391322892L);
            assert(pack.temperature_GET() == (short)32767);
            assert(pack.press_abs_GET() == 1.3139254E38F);
            assert(pack.press_diff_GET() == 2.675197E38F);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.temperature_SET((short)32767) ;
        p143.time_boot_ms_SET(1391322892L) ;
        p143.press_diff_SET(2.675197E38F) ;
        p143.press_abs_SET(1.3139254E38F) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-3.3375575E38F, -1.6886407E38F, -1.9145798E38F}));
            assert(Arrays.equals(pack.vel_GET(),  new float[] {-3.3900236E38F, 1.998325E38F, -1.2501959E38F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-1.6953704E38F, -1.781754E38F, -7.971593E37F}));
            assert(pack.lon_GET() == 162116588);
            assert(pack.alt_GET() == 2.2090327E38F);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-2.0504082E38F, 3.2752335E38F, -7.06E37F, 2.4907702E38F}));
            assert(pack.lat_GET() == 1852402594);
            assert(pack.est_capabilities_GET() == (char)39);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {5.2097073E37F, 2.1104204E38F, -5.9895036E37F}));
            assert(pack.custom_state_GET() == 3140275191457838080L);
            assert(pack.timestamp_GET() == 5511737982595919123L);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.lon_SET(162116588) ;
        p144.alt_SET(2.2090327E38F) ;
        p144.lat_SET(1852402594) ;
        p144.rates_SET(new float[] {-1.6953704E38F, -1.781754E38F, -7.971593E37F}, 0) ;
        p144.custom_state_SET(3140275191457838080L) ;
        p144.position_cov_SET(new float[] {5.2097073E37F, 2.1104204E38F, -5.9895036E37F}, 0) ;
        p144.attitude_q_SET(new float[] {-2.0504082E38F, 3.2752335E38F, -7.06E37F, 2.4907702E38F}, 0) ;
        p144.est_capabilities_SET((char)39) ;
        p144.acc_SET(new float[] {-3.3375575E38F, -1.6886407E38F, -1.9145798E38F}, 0) ;
        p144.vel_SET(new float[] {-3.3900236E38F, 1.998325E38F, -1.2501959E38F}, 0) ;
        p144.timestamp_SET(5511737982595919123L) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.z_acc_GET() == 1.2517131E38F);
            assert(pack.x_acc_GET() == 9.288282E36F);
            assert(pack.z_pos_GET() == 6.7999694E37F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {1.9441706E38F, -2.8942775E38F, 4.605116E37F}));
            assert(pack.y_pos_GET() == -1.964113E38F);
            assert(pack.y_vel_GET() == 1.294989E38F);
            assert(pack.airspeed_GET() == 1.1770665E38F);
            assert(pack.roll_rate_GET() == 1.6269108E37F);
            assert(pack.yaw_rate_GET() == 1.7648694E38F);
            assert(pack.x_vel_GET() == -2.2782888E38F);
            assert(pack.x_pos_GET() == 2.8852533E38F);
            assert(pack.z_vel_GET() == 3.1013183E38F);
            assert(pack.pitch_rate_GET() == 3.0763797E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {1.766389E38F, 6.5384693E37F, 2.2084346E38F}));
            assert(pack.time_usec_GET() == 8976088926841038957L);
            assert(pack.y_acc_GET() == 1.1877911E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {9.138266E37F, -7.9926313E37F, 3.1691825E38F, -1.8891199E38F}));
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.y_pos_SET(-1.964113E38F) ;
        p146.x_pos_SET(2.8852533E38F) ;
        p146.z_pos_SET(6.7999694E37F) ;
        p146.roll_rate_SET(1.6269108E37F) ;
        p146.x_acc_SET(9.288282E36F) ;
        p146.pitch_rate_SET(3.0763797E38F) ;
        p146.pos_variance_SET(new float[] {1.766389E38F, 6.5384693E37F, 2.2084346E38F}, 0) ;
        p146.vel_variance_SET(new float[] {1.9441706E38F, -2.8942775E38F, 4.605116E37F}, 0) ;
        p146.yaw_rate_SET(1.7648694E38F) ;
        p146.z_vel_SET(3.1013183E38F) ;
        p146.x_vel_SET(-2.2782888E38F) ;
        p146.q_SET(new float[] {9.138266E37F, -7.9926313E37F, 3.1691825E38F, -1.8891199E38F}, 0) ;
        p146.y_acc_SET(1.1877911E38F) ;
        p146.y_vel_SET(1.294989E38F) ;
        p146.airspeed_SET(1.1770665E38F) ;
        p146.time_usec_SET(8976088926841038957L) ;
        p146.z_acc_SET(1.2517131E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)169);
            assert(pack.current_battery_GET() == (short) -16814);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
            assert(pack.battery_remaining_GET() == (byte)52);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)10536, (char)55277, (char)36781, (char)19580, (char)9596, (char)48512, (char)54500, (char)17696, (char)56618, (char)58119}));
            assert(pack.current_consumed_GET() == -1595207842);
            assert(pack.temperature_GET() == (short) -29143);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
            assert(pack.energy_consumed_GET() == -472575647);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION) ;
        p147.voltages_SET(new char[] {(char)10536, (char)55277, (char)36781, (char)19580, (char)9596, (char)48512, (char)54500, (char)17696, (char)56618, (char)58119}, 0) ;
        p147.battery_remaining_SET((byte)52) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN) ;
        p147.current_consumed_SET(-1595207842) ;
        p147.energy_consumed_SET(-472575647) ;
        p147.temperature_SET((short) -29143) ;
        p147.id_SET((char)169) ;
        p147.current_battery_SET((short) -16814) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.board_version_GET() == 1172105672L);
            assert(pack.vendor_id_GET() == (char)45778);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)70, (char)210, (char)164, (char)162, (char)3, (char)109, (char)146, (char)241}));
            assert(pack.middleware_sw_version_GET() == 1139600194L);
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP));
            assert(pack.os_sw_version_GET() == 594650265L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)3, (char)76, (char)176, (char)89, (char)93, (char)92, (char)102, (char)125, (char)69, (char)2, (char)178, (char)38, (char)182, (char)252, (char)241, (char)63, (char)210, (char)36}));
            assert(pack.uid_GET() == 1018544402699940345L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)103, (char)224, (char)79, (char)81, (char)201, (char)223, (char)157, (char)100}));
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)61, (char)65, (char)182, (char)162, (char)36, (char)67, (char)94, (char)239}));
            assert(pack.product_id_GET() == (char)10124);
            assert(pack.flight_sw_version_GET() == 4104870869L);
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.os_sw_version_SET(594650265L) ;
        p148.middleware_sw_version_SET(1139600194L) ;
        p148.os_custom_version_SET(new char[] {(char)103, (char)224, (char)79, (char)81, (char)201, (char)223, (char)157, (char)100}, 0) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP)) ;
        p148.flight_sw_version_SET(4104870869L) ;
        p148.uid_SET(1018544402699940345L) ;
        p148.board_version_SET(1172105672L) ;
        p148.product_id_SET((char)10124) ;
        p148.middleware_custom_version_SET(new char[] {(char)61, (char)65, (char)182, (char)162, (char)36, (char)67, (char)94, (char)239}, 0) ;
        p148.flight_custom_version_SET(new char[] {(char)70, (char)210, (char)164, (char)162, (char)3, (char)109, (char)146, (char)241}, 0) ;
        p148.uid2_SET(new char[] {(char)3, (char)76, (char)176, (char)89, (char)93, (char)92, (char)102, (char)125, (char)69, (char)2, (char)178, (char)38, (char)182, (char)252, (char)241, (char)63, (char)210, (char)36}, 0, PH) ;
        p148.vendor_id_SET((char)45778) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.position_valid_TRY(ph) == (char)14);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
            assert(pack.angle_x_GET() == -2.9334047E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {-9.505066E37F, 6.409763E37F, 8.1808546E37F, -2.9639387E38F}));
            assert(pack.x_TRY(ph) == -8.2654926E37F);
            assert(pack.size_y_GET() == -2.3788636E38F);
            assert(pack.size_x_GET() == -7.2488815E37F);
            assert(pack.angle_y_GET() == 2.2489247E37F);
            assert(pack.target_num_GET() == (char)223);
            assert(pack.z_TRY(ph) == -1.013716E38F);
            assert(pack.y_TRY(ph) == -1.4974678E38F);
            assert(pack.distance_GET() == 2.1524632E38F);
            assert(pack.time_usec_GET() == 5419516404311726863L);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.y_SET(-1.4974678E38F, PH) ;
        p149.angle_x_SET(-2.9334047E38F) ;
        p149.position_valid_SET((char)14, PH) ;
        p149.z_SET(-1.013716E38F, PH) ;
        p149.angle_y_SET(2.2489247E37F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p149.x_SET(-8.2654926E37F, PH) ;
        p149.target_num_SET((char)223) ;
        p149.q_SET(new float[] {-9.505066E37F, 6.409763E37F, 8.1808546E37F, -2.9639387E38F}, 0, PH) ;
        p149.time_usec_SET(5419516404311726863L) ;
        p149.size_x_SET(-7.2488815E37F) ;
        p149.size_y_SET(-2.3788636E38F) ;
        p149.distance_SET(2.1524632E38F) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AQ_TELEMETRY_F.add((src, ph, pack) ->
        {
            assert(pack.value18_GET() == 2.085413E38F);
            assert(pack.value14_GET() == -9.546923E37F);
            assert(pack.value12_GET() == 2.0466838E38F);
            assert(pack.value5_GET() == -2.518782E38F);
            assert(pack.value16_GET() == 3.1019869E38F);
            assert(pack.value2_GET() == 2.3217082E38F);
            assert(pack.value8_GET() == 6.31646E37F);
            assert(pack.value1_GET() == -1.457916E38F);
            assert(pack.value11_GET() == -2.3985604E38F);
            assert(pack.Index_GET() == (char)41897);
            assert(pack.value3_GET() == -6.396962E37F);
            assert(pack.value6_GET() == -1.4512927E38F);
            assert(pack.value9_GET() == 1.4926241E38F);
            assert(pack.value17_GET() == 3.305773E38F);
            assert(pack.value15_GET() == -3.5234846E36F);
            assert(pack.value7_GET() == -1.2608077E38F);
            assert(pack.value20_GET() == -1.04427516E37F);
            assert(pack.value4_GET() == -1.2822967E38F);
            assert(pack.value13_GET() == 5.567903E37F);
            assert(pack.value19_GET() == -2.133597E38F);
            assert(pack.value10_GET() == -1.0589369E38F);
        });
        GroundControl.AQ_TELEMETRY_F p150 = CommunicationChannel.new_AQ_TELEMETRY_F();
        PH.setPack(p150);
        p150.value14_SET(-9.546923E37F) ;
        p150.value3_SET(-6.396962E37F) ;
        p150.value12_SET(2.0466838E38F) ;
        p150.value6_SET(-1.4512927E38F) ;
        p150.value18_SET(2.085413E38F) ;
        p150.value20_SET(-1.04427516E37F) ;
        p150.value4_SET(-1.2822967E38F) ;
        p150.value7_SET(-1.2608077E38F) ;
        p150.value13_SET(5.567903E37F) ;
        p150.value5_SET(-2.518782E38F) ;
        p150.value17_SET(3.305773E38F) ;
        p150.value11_SET(-2.3985604E38F) ;
        p150.value15_SET(-3.5234846E36F) ;
        p150.value19_SET(-2.133597E38F) ;
        p150.value16_SET(3.1019869E38F) ;
        p150.value9_SET(1.4926241E38F) ;
        p150.Index_SET((char)41897) ;
        p150.value10_SET(-1.0589369E38F) ;
        p150.value1_SET(-1.457916E38F) ;
        p150.value2_SET(2.3217082E38F) ;
        p150.value8_SET(6.31646E37F) ;
        CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AQ_ESC_TELEMETRY.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2941774287L);
            assert(pack.num_motors_GET() == (char)226);
            assert(Arrays.equals(pack.data1_GET(),  new long[] {1945307979L, 3916601309L, 605215168L, 3890290475L}));
            assert(Arrays.equals(pack.data0_GET(),  new long[] {593231891L, 1824076514L, 1912825218L, 1453718429L}));
            assert(Arrays.equals(pack.status_age_GET(),  new char[] {(char)41913, (char)62858, (char)50124, (char)20245}));
            assert(pack.num_in_seq_GET() == (char)186);
            assert(Arrays.equals(pack.data_version_GET(),  new char[] {(char)240, (char)217, (char)171, (char)95}));
            assert(Arrays.equals(pack.escid_GET(),  new char[] {(char)132, (char)142, (char)116, (char)110}));
            assert(pack.seq_GET() == (char)179);
        });
        GroundControl.AQ_ESC_TELEMETRY p152 = CommunicationChannel.new_AQ_ESC_TELEMETRY();
        PH.setPack(p152);
        p152.seq_SET((char)179) ;
        p152.data0_SET(new long[] {593231891L, 1824076514L, 1912825218L, 1453718429L}, 0) ;
        p152.escid_SET(new char[] {(char)132, (char)142, (char)116, (char)110}, 0) ;
        p152.status_age_SET(new char[] {(char)41913, (char)62858, (char)50124, (char)20245}, 0) ;
        p152.num_in_seq_SET((char)186) ;
        p152.data1_SET(new long[] {1945307979L, 3916601309L, 605215168L, 3890290475L}, 0) ;
        p152.num_motors_SET((char)226) ;
        p152.time_boot_ms_SET(2941774287L) ;
        p152.data_version_SET(new char[] {(char)240, (char)217, (char)171, (char)95}, 0) ;
        CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.mag_ratio_GET() == 1.3644758E37F);
            assert(pack.time_usec_GET() == 7300766869342542949L);
            assert(pack.pos_horiz_ratio_GET() == -2.3681423E38F);
            assert(pack.pos_horiz_accuracy_GET() == 5.5040664E37F);
            assert(pack.pos_vert_accuracy_GET() == 1.9726554E38F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE));
            assert(pack.vel_ratio_GET() == 1.8122653E38F);
            assert(pack.tas_ratio_GET() == -8.717742E37F);
            assert(pack.pos_vert_ratio_GET() == -4.7705926E37F);
            assert(pack.hagl_ratio_GET() == -1.1288155E38F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.hagl_ratio_SET(-1.1288155E38F) ;
        p230.tas_ratio_SET(-8.717742E37F) ;
        p230.pos_vert_accuracy_SET(1.9726554E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE)) ;
        p230.pos_horiz_ratio_SET(-2.3681423E38F) ;
        p230.pos_vert_ratio_SET(-4.7705926E37F) ;
        p230.time_usec_SET(7300766869342542949L) ;
        p230.mag_ratio_SET(1.3644758E37F) ;
        p230.vel_ratio_SET(1.8122653E38F) ;
        p230.pos_horiz_accuracy_SET(5.5040664E37F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_x_GET() == -8.8545477E36F);
            assert(pack.vert_accuracy_GET() == -1.9016128E38F);
            assert(pack.var_vert_GET() == 3.1929346E38F);
            assert(pack.wind_y_GET() == -1.8588939E37F);
            assert(pack.horiz_accuracy_GET() == -8.4830456E36F);
            assert(pack.var_horiz_GET() == 6.102973E36F);
            assert(pack.time_usec_GET() == 6339922954382566340L);
            assert(pack.wind_alt_GET() == -1.5040766E38F);
            assert(pack.wind_z_GET() == -2.0374684E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_z_SET(-2.0374684E38F) ;
        p231.time_usec_SET(6339922954382566340L) ;
        p231.wind_y_SET(-1.8588939E37F) ;
        p231.wind_x_SET(-8.8545477E36F) ;
        p231.var_vert_SET(3.1929346E38F) ;
        p231.horiz_accuracy_SET(-8.4830456E36F) ;
        p231.var_horiz_SET(6.102973E36F) ;
        p231.vert_accuracy_SET(-1.9016128E38F) ;
        p231.wind_alt_SET(-1.5040766E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.horiz_accuracy_GET() == 1.1110866E38F);
            assert(pack.vdop_GET() == -1.358927E38F);
            assert(pack.lat_GET() == 191220749);
            assert(pack.speed_accuracy_GET() == -3.9442059E37F);
            assert(pack.satellites_visible_GET() == (char)171);
            assert(pack.time_usec_GET() == 3090827653192614296L);
            assert(pack.vn_GET() == -7.411166E37F);
            assert(pack.vd_GET() == -1.4472581E38F);
            assert(pack.time_week_GET() == (char)25209);
            assert(pack.vert_accuracy_GET() == -3.1308248E38F);
            assert(pack.ve_GET() == -2.6003635E38F);
            assert(pack.time_week_ms_GET() == 3140587696L);
            assert(pack.alt_GET() == -1.9255594E38F);
            assert(pack.fix_type_GET() == (char)251);
            assert(pack.lon_GET() == 1184081172);
            assert(pack.gps_id_GET() == (char)104);
            assert(pack.hdop_GET() == -6.312504E37F);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP));
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.vn_SET(-7.411166E37F) ;
        p232.satellites_visible_SET((char)171) ;
        p232.hdop_SET(-6.312504E37F) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP)) ;
        p232.ve_SET(-2.6003635E38F) ;
        p232.vert_accuracy_SET(-3.1308248E38F) ;
        p232.lat_SET(191220749) ;
        p232.speed_accuracy_SET(-3.9442059E37F) ;
        p232.vd_SET(-1.4472581E38F) ;
        p232.gps_id_SET((char)104) ;
        p232.horiz_accuracy_SET(1.1110866E38F) ;
        p232.time_week_SET((char)25209) ;
        p232.time_usec_SET(3090827653192614296L) ;
        p232.time_week_ms_SET(3140587696L) ;
        p232.vdop_SET(-1.358927E38F) ;
        p232.alt_SET(-1.9255594E38F) ;
        p232.lon_SET(1184081172) ;
        p232.fix_type_SET((char)251) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (char)147);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)64, (char)38, (char)225, (char)0, (char)19, (char)63, (char)88, (char)27, (char)93, (char)102, (char)155, (char)238, (char)53, (char)8, (char)10, (char)177, (char)210, (char)154, (char)29, (char)24, (char)68, (char)229, (char)34, (char)128, (char)21, (char)113, (char)8, (char)152, (char)101, (char)155, (char)82, (char)180, (char)75, (char)54, (char)47, (char)123, (char)80, (char)201, (char)107, (char)99, (char)229, (char)219, (char)206, (char)188, (char)1, (char)140, (char)93, (char)56, (char)152, (char)110, (char)18, (char)166, (char)202, (char)170, (char)3, (char)12, (char)89, (char)247, (char)221, (char)84, (char)200, (char)191, (char)208, (char)106, (char)186, (char)218, (char)108, (char)255, (char)132, (char)113, (char)48, (char)73, (char)75, (char)72, (char)181, (char)237, (char)78, (char)144, (char)97, (char)63, (char)63, (char)56, (char)164, (char)241, (char)206, (char)216, (char)105, (char)163, (char)29, (char)79, (char)207, (char)249, (char)141, (char)65, (char)227, (char)104, (char)175, (char)234, (char)179, (char)128, (char)25, (char)201, (char)107, (char)124, (char)196, (char)155, (char)41, (char)156, (char)43, (char)25, (char)35, (char)1, (char)254, (char)213, (char)138, (char)104, (char)80, (char)247, (char)65, (char)159, (char)5, (char)35, (char)232, (char)222, (char)25, (char)22, (char)216, (char)202, (char)246, (char)78, (char)32, (char)65, (char)119, (char)206, (char)131, (char)109, (char)224, (char)161, (char)49, (char)107, (char)225, (char)29, (char)112, (char)26, (char)155, (char)134, (char)232, (char)170, (char)164, (char)189, (char)177, (char)177, (char)94, (char)219, (char)168, (char)48, (char)231, (char)101, (char)155, (char)237, (char)83, (char)1, (char)193, (char)211, (char)62, (char)8, (char)195, (char)166, (char)198, (char)31, (char)178, (char)217, (char)180, (char)50, (char)23, (char)116, (char)13, (char)199, (char)92, (char)45}));
            assert(pack.len_GET() == (char)111);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.data__SET(new char[] {(char)64, (char)38, (char)225, (char)0, (char)19, (char)63, (char)88, (char)27, (char)93, (char)102, (char)155, (char)238, (char)53, (char)8, (char)10, (char)177, (char)210, (char)154, (char)29, (char)24, (char)68, (char)229, (char)34, (char)128, (char)21, (char)113, (char)8, (char)152, (char)101, (char)155, (char)82, (char)180, (char)75, (char)54, (char)47, (char)123, (char)80, (char)201, (char)107, (char)99, (char)229, (char)219, (char)206, (char)188, (char)1, (char)140, (char)93, (char)56, (char)152, (char)110, (char)18, (char)166, (char)202, (char)170, (char)3, (char)12, (char)89, (char)247, (char)221, (char)84, (char)200, (char)191, (char)208, (char)106, (char)186, (char)218, (char)108, (char)255, (char)132, (char)113, (char)48, (char)73, (char)75, (char)72, (char)181, (char)237, (char)78, (char)144, (char)97, (char)63, (char)63, (char)56, (char)164, (char)241, (char)206, (char)216, (char)105, (char)163, (char)29, (char)79, (char)207, (char)249, (char)141, (char)65, (char)227, (char)104, (char)175, (char)234, (char)179, (char)128, (char)25, (char)201, (char)107, (char)124, (char)196, (char)155, (char)41, (char)156, (char)43, (char)25, (char)35, (char)1, (char)254, (char)213, (char)138, (char)104, (char)80, (char)247, (char)65, (char)159, (char)5, (char)35, (char)232, (char)222, (char)25, (char)22, (char)216, (char)202, (char)246, (char)78, (char)32, (char)65, (char)119, (char)206, (char)131, (char)109, (char)224, (char)161, (char)49, (char)107, (char)225, (char)29, (char)112, (char)26, (char)155, (char)134, (char)232, (char)170, (char)164, (char)189, (char)177, (char)177, (char)94, (char)219, (char)168, (char)48, (char)231, (char)101, (char)155, (char)237, (char)83, (char)1, (char)193, (char)211, (char)62, (char)8, (char)195, (char)166, (char)198, (char)31, (char)178, (char)217, (char)180, (char)50, (char)23, (char)116, (char)13, (char)199, (char)92, (char)45}, 0) ;
        p233.flags_SET((char)147) ;
        p233.len_SET((char)111) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.throttle_GET() == (byte) - 77);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED));
            assert(pack.groundspeed_GET() == (char)249);
            assert(pack.longitude_GET() == 1363885302);
            assert(pack.custom_mode_GET() == 1750018222L);
            assert(pack.failsafe_GET() == (char)217);
            assert(pack.heading_GET() == (char)55108);
            assert(pack.pitch_GET() == (short)29925);
            assert(pack.wp_distance_GET() == (char)49462);
            assert(pack.climb_rate_GET() == (byte) - 52);
            assert(pack.airspeed_GET() == (char)139);
            assert(pack.roll_GET() == (short) -6301);
            assert(pack.latitude_GET() == -2011598051);
            assert(pack.battery_remaining_GET() == (char)173);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.gps_nsat_GET() == (char)106);
            assert(pack.temperature_GET() == (byte) - 95);
            assert(pack.temperature_air_GET() == (byte) - 43);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.airspeed_sp_GET() == (char)14);
            assert(pack.altitude_sp_GET() == (short) -17145);
            assert(pack.heading_sp_GET() == (short) -26045);
            assert(pack.altitude_amsl_GET() == (short)12249);
            assert(pack.wp_num_GET() == (char)96);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.heading_SET((char)55108) ;
        p234.custom_mode_SET(1750018222L) ;
        p234.roll_SET((short) -6301) ;
        p234.airspeed_SET((char)139) ;
        p234.longitude_SET(1363885302) ;
        p234.heading_sp_SET((short) -26045) ;
        p234.throttle_SET((byte) - 77) ;
        p234.pitch_SET((short)29925) ;
        p234.gps_nsat_SET((char)106) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED)) ;
        p234.temperature_air_SET((byte) - 43) ;
        p234.failsafe_SET((char)217) ;
        p234.wp_distance_SET((char)49462) ;
        p234.groundspeed_SET((char)249) ;
        p234.wp_num_SET((char)96) ;
        p234.battery_remaining_SET((char)173) ;
        p234.temperature_SET((byte) - 95) ;
        p234.altitude_amsl_SET((short)12249) ;
        p234.climb_rate_SET((byte) - 52) ;
        p234.airspeed_sp_SET((char)14) ;
        p234.latitude_SET(-2011598051) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        p234.altitude_sp_SET((short) -17145) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_x_GET() == -1.910655E37F);
            assert(pack.time_usec_GET() == 7113545224345259627L);
            assert(pack.vibration_y_GET() == 2.8771014E37F);
            assert(pack.vibration_z_GET() == -4.8386588E36F);
            assert(pack.clipping_1_GET() == 853470049L);
            assert(pack.clipping_0_GET() == 3995244077L);
            assert(pack.clipping_2_GET() == 3588773911L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_0_SET(3995244077L) ;
        p241.vibration_x_SET(-1.910655E37F) ;
        p241.clipping_2_SET(3588773911L) ;
        p241.clipping_1_SET(853470049L) ;
        p241.vibration_y_SET(2.8771014E37F) ;
        p241.vibration_z_SET(-4.8386588E36F) ;
        p241.time_usec_SET(7113545224345259627L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_z_GET() == 3.052269E38F);
            assert(pack.approach_y_GET() == 2.7972416E38F);
            assert(pack.x_GET() == -3.4673474E37F);
            assert(pack.latitude_GET() == 1247441873);
            assert(pack.time_usec_TRY(ph) == 2533972518089739200L);
            assert(pack.longitude_GET() == 1159793241);
            assert(pack.y_GET() == 4.891524E37F);
            assert(pack.z_GET() == -2.5032382E38F);
            assert(pack.altitude_GET() == 896641774);
            assert(pack.approach_x_GET() == 7.3319907E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.8442276E38F, 3.286963E38F, -2.2108575E38F, -1.1294175E38F}));
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.altitude_SET(896641774) ;
        p242.y_SET(4.891524E37F) ;
        p242.time_usec_SET(2533972518089739200L, PH) ;
        p242.approach_y_SET(2.7972416E38F) ;
        p242.latitude_SET(1247441873) ;
        p242.x_SET(-3.4673474E37F) ;
        p242.approach_x_SET(7.3319907E37F) ;
        p242.q_SET(new float[] {1.8442276E38F, 3.286963E38F, -2.2108575E38F, -1.1294175E38F}, 0) ;
        p242.approach_z_SET(3.052269E38F) ;
        p242.longitude_SET(1159793241) ;
        p242.z_SET(-2.5032382E38F) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_z_GET() == -9.029197E36F);
            assert(pack.z_GET() == 3.0552744E38F);
            assert(pack.y_GET() == 2.047914E38F);
            assert(pack.altitude_GET() == -830083830);
            assert(pack.longitude_GET() == 1144903401);
            assert(pack.approach_y_GET() == 2.3021395E38F);
            assert(pack.time_usec_TRY(ph) == 3012746813048410724L);
            assert(pack.approach_x_GET() == -1.3908624E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.494631E38F, -5.532925E37F, 1.9357122E38F, -2.5542393E38F}));
            assert(pack.target_system_GET() == (char)90);
            assert(pack.x_GET() == -3.222237E38F);
            assert(pack.latitude_GET() == -1564851110);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.z_SET(3.0552744E38F) ;
        p243.latitude_SET(-1564851110) ;
        p243.time_usec_SET(3012746813048410724L, PH) ;
        p243.approach_z_SET(-9.029197E36F) ;
        p243.y_SET(2.047914E38F) ;
        p243.x_SET(-3.222237E38F) ;
        p243.longitude_SET(1144903401) ;
        p243.approach_y_SET(2.3021395E38F) ;
        p243.q_SET(new float[] {-2.494631E38F, -5.532925E37F, 1.9357122E38F, -2.5542393E38F}, 0) ;
        p243.target_system_SET((char)90) ;
        p243.approach_x_SET(-1.3908624E37F) ;
        p243.altitude_SET(-830083830) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == -1210761509);
            assert(pack.message_id_GET() == (char)37921);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-1210761509) ;
        p244.message_id_SET((char)37921) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND) ;
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_MC) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.ver_velocity_GET() == (short)31987);
            assert(pack.heading_GET() == (char)61666);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.hor_velocity_GET() == (char)13143);
            assert(pack.altitude_GET() == -1983447131);
            assert(pack.callsign_LEN(ph) == 7);
            assert(pack.callsign_TRY(ph).equals("tryjiye"));
            assert(pack.lat_GET() == -1060897395);
            assert(pack.squawk_GET() == (char)60030);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SERVICE_SURFACE);
            assert(pack.lon_GET() == 846528287);
            assert(pack.ICAO_address_GET() == 3154962439L);
            assert(pack.tslc_GET() == (char)209);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK));
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ICAO_address_SET(3154962439L) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK)) ;
        p246.tslc_SET((char)209) ;
        p246.callsign_SET("tryjiye", PH) ;
        p246.lat_SET(-1060897395) ;
        p246.ver_velocity_SET((short)31987) ;
        p246.hor_velocity_SET((char)13143) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SERVICE_SURFACE) ;
        p246.heading_SET((char)61666) ;
        p246.squawk_SET((char)60030) ;
        p246.lon_SET(846528287) ;
        p246.altitude_SET(-1983447131) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == 1612962527L);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            assert(pack.time_to_minimum_delta_GET() == 7.5821874E37F);
            assert(pack.altitude_minimum_delta_GET() == 5.315309E37F);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
            assert(pack.horizontal_minimum_delta_GET() == 3.8774293E37F);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.id_SET(1612962527L) ;
        p247.altitude_minimum_delta_SET(5.315309E37F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.time_to_minimum_delta_SET(7.5821874E37F) ;
        p247.horizontal_minimum_delta_SET(3.8774293E37F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.message_type_GET() == (char)18607);
            assert(pack.target_component_GET() == (char)37);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)51, (char)235, (char)7, (char)160, (char)48, (char)11, (char)199, (char)182, (char)149, (char)218, (char)154, (char)109, (char)86, (char)58, (char)15, (char)119, (char)200, (char)195, (char)220, (char)233, (char)254, (char)147, (char)6, (char)111, (char)131, (char)139, (char)252, (char)240, (char)215, (char)149, (char)41, (char)20, (char)152, (char)214, (char)56, (char)188, (char)51, (char)97, (char)53, (char)86, (char)95, (char)236, (char)71, (char)188, (char)215, (char)133, (char)53, (char)220, (char)224, (char)211, (char)155, (char)193, (char)91, (char)197, (char)232, (char)193, (char)41, (char)108, (char)175, (char)96, (char)134, (char)95, (char)17, (char)222, (char)103, (char)38, (char)153, (char)157, (char)105, (char)37, (char)198, (char)105, (char)43, (char)113, (char)192, (char)69, (char)245, (char)238, (char)211, (char)125, (char)92, (char)199, (char)235, (char)88, (char)112, (char)6, (char)120, (char)114, (char)245, (char)96, (char)159, (char)117, (char)166, (char)233, (char)186, (char)178, (char)197, (char)216, (char)15, (char)228, (char)106, (char)119, (char)10, (char)12, (char)106, (char)191, (char)128, (char)16, (char)169, (char)44, (char)104, (char)185, (char)249, (char)12, (char)252, (char)105, (char)203, (char)211, (char)147, (char)90, (char)194, (char)66, (char)79, (char)101, (char)135, (char)251, (char)34, (char)68, (char)130, (char)22, (char)41, (char)172, (char)234, (char)190, (char)196, (char)5, (char)231, (char)117, (char)249, (char)82, (char)159, (char)58, (char)27, (char)75, (char)177, (char)202, (char)234, (char)221, (char)146, (char)224, (char)249, (char)39, (char)8, (char)37, (char)54, (char)180, (char)169, (char)140, (char)206, (char)46, (char)35, (char)185, (char)235, (char)168, (char)76, (char)33, (char)169, (char)61, (char)196, (char)179, (char)39, (char)241, (char)107, (char)39, (char)232, (char)221, (char)16, (char)229, (char)110, (char)88, (char)199, (char)23, (char)241, (char)180, (char)138, (char)139, (char)33, (char)172, (char)2, (char)198, (char)236, (char)24, (char)38, (char)197, (char)180, (char)224, (char)79, (char)99, (char)155, (char)115, (char)128, (char)167, (char)89, (char)126, (char)137, (char)94, (char)116, (char)116, (char)187, (char)126, (char)98, (char)246, (char)218, (char)106, (char)180, (char)162, (char)209, (char)159, (char)30, (char)175, (char)76, (char)16, (char)185, (char)242, (char)71, (char)120, (char)131, (char)242, (char)48, (char)215, (char)118, (char)160, (char)167, (char)167, (char)37, (char)180, (char)97, (char)55, (char)10, (char)155, (char)34, (char)141, (char)11, (char)243, (char)115, (char)72, (char)209, (char)74, (char)140}));
            assert(pack.target_network_GET() == (char)29);
            assert(pack.target_system_GET() == (char)229);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)29) ;
        p248.target_system_SET((char)229) ;
        p248.payload_SET(new char[] {(char)51, (char)235, (char)7, (char)160, (char)48, (char)11, (char)199, (char)182, (char)149, (char)218, (char)154, (char)109, (char)86, (char)58, (char)15, (char)119, (char)200, (char)195, (char)220, (char)233, (char)254, (char)147, (char)6, (char)111, (char)131, (char)139, (char)252, (char)240, (char)215, (char)149, (char)41, (char)20, (char)152, (char)214, (char)56, (char)188, (char)51, (char)97, (char)53, (char)86, (char)95, (char)236, (char)71, (char)188, (char)215, (char)133, (char)53, (char)220, (char)224, (char)211, (char)155, (char)193, (char)91, (char)197, (char)232, (char)193, (char)41, (char)108, (char)175, (char)96, (char)134, (char)95, (char)17, (char)222, (char)103, (char)38, (char)153, (char)157, (char)105, (char)37, (char)198, (char)105, (char)43, (char)113, (char)192, (char)69, (char)245, (char)238, (char)211, (char)125, (char)92, (char)199, (char)235, (char)88, (char)112, (char)6, (char)120, (char)114, (char)245, (char)96, (char)159, (char)117, (char)166, (char)233, (char)186, (char)178, (char)197, (char)216, (char)15, (char)228, (char)106, (char)119, (char)10, (char)12, (char)106, (char)191, (char)128, (char)16, (char)169, (char)44, (char)104, (char)185, (char)249, (char)12, (char)252, (char)105, (char)203, (char)211, (char)147, (char)90, (char)194, (char)66, (char)79, (char)101, (char)135, (char)251, (char)34, (char)68, (char)130, (char)22, (char)41, (char)172, (char)234, (char)190, (char)196, (char)5, (char)231, (char)117, (char)249, (char)82, (char)159, (char)58, (char)27, (char)75, (char)177, (char)202, (char)234, (char)221, (char)146, (char)224, (char)249, (char)39, (char)8, (char)37, (char)54, (char)180, (char)169, (char)140, (char)206, (char)46, (char)35, (char)185, (char)235, (char)168, (char)76, (char)33, (char)169, (char)61, (char)196, (char)179, (char)39, (char)241, (char)107, (char)39, (char)232, (char)221, (char)16, (char)229, (char)110, (char)88, (char)199, (char)23, (char)241, (char)180, (char)138, (char)139, (char)33, (char)172, (char)2, (char)198, (char)236, (char)24, (char)38, (char)197, (char)180, (char)224, (char)79, (char)99, (char)155, (char)115, (char)128, (char)167, (char)89, (char)126, (char)137, (char)94, (char)116, (char)116, (char)187, (char)126, (char)98, (char)246, (char)218, (char)106, (char)180, (char)162, (char)209, (char)159, (char)30, (char)175, (char)76, (char)16, (char)185, (char)242, (char)71, (char)120, (char)131, (char)242, (char)48, (char)215, (char)118, (char)160, (char)167, (char)167, (char)37, (char)180, (char)97, (char)55, (char)10, (char)155, (char)34, (char)141, (char)11, (char)243, (char)115, (char)72, (char)209, (char)74, (char)140}, 0) ;
        p248.target_component_SET((char)37) ;
        p248.message_type_SET((char)18607) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.address_GET() == (char)22248);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)83, (byte) - 112, (byte)53, (byte) - 80, (byte)18, (byte) - 78, (byte)63, (byte) - 43, (byte)24, (byte)74, (byte) - 117, (byte)51, (byte) - 107, (byte)15, (byte)99, (byte)95, (byte)16, (byte) - 30, (byte) - 12, (byte) - 70, (byte) - 25, (byte) - 105, (byte) - 94, (byte)127, (byte) - 85, (byte)123, (byte)85, (byte) - 59, (byte)70, (byte)2, (byte) - 103, (byte) - 106}));
            assert(pack.type_GET() == (char)85);
            assert(pack.ver_GET() == (char)99);
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.type_SET((char)85) ;
        p249.ver_SET((char)99) ;
        p249.address_SET((char)22248) ;
        p249.value_SET(new byte[] {(byte)83, (byte) - 112, (byte)53, (byte) - 80, (byte)18, (byte) - 78, (byte)63, (byte) - 43, (byte)24, (byte)74, (byte) - 117, (byte)51, (byte) - 107, (byte)15, (byte)99, (byte)95, (byte)16, (byte) - 30, (byte) - 12, (byte) - 70, (byte) - 25, (byte) - 105, (byte) - 94, (byte)127, (byte) - 85, (byte)123, (byte)85, (byte) - 59, (byte)70, (byte)2, (byte) - 103, (byte) - 106}, 0) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.6433646E38F);
            assert(pack.y_GET() == -1.6558137E38F);
            assert(pack.name_LEN(ph) == 6);
            assert(pack.name_TRY(ph).equals("hwrrgn"));
            assert(pack.x_GET() == 3.1922113E38F);
            assert(pack.time_usec_GET() == 6081534392659876209L);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.time_usec_SET(6081534392659876209L) ;
        p250.y_SET(-1.6558137E38F) ;
        p250.x_SET(3.1922113E38F) ;
        p250.z_SET(-1.6433646E38F) ;
        p250.name_SET("hwrrgn", PH) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -2.254672E37F);
            assert(pack.time_boot_ms_GET() == 2017222062L);
            assert(pack.name_LEN(ph) == 3);
            assert(pack.name_TRY(ph).equals("goj"));
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("goj", PH) ;
        p251.time_boot_ms_SET(2017222062L) ;
        p251.value_SET(-2.254672E37F) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -768972085);
            assert(pack.time_boot_ms_GET() == 3061412137L);
            assert(pack.name_LEN(ph) == 7);
            assert(pack.name_TRY(ph).equals("rdekemn"));
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.value_SET(-768972085) ;
        p252.name_SET("rdekemn", PH) ;
        p252.time_boot_ms_SET(3061412137L) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
            assert(pack.text_LEN(ph) == 25);
            assert(pack.text_TRY(ph).equals("jtqZjozaucwlEszQrmspmiLwv"));
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("jtqZjozaucwlEszQrmspmiLwv", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_EMERGENCY) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.ind_GET() == (char)209);
            assert(pack.value_GET() == 3.3993876E38F);
            assert(pack.time_boot_ms_GET() == 4275361884L);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(4275361884L) ;
        p254.ind_SET((char)209) ;
        p254.value_SET(3.3993876E38F) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)147, (char)149, (char)18, (char)128, (char)174, (char)157, (char)87, (char)76, (char)104, (char)151, (char)158, (char)248, (char)192, (char)165, (char)240, (char)30, (char)249, (char)11, (char)132, (char)150, (char)145, (char)177, (char)157, (char)136, (char)13, (char)112, (char)187, (char)232, (char)38, (char)41, (char)5, (char)220}));
            assert(pack.initial_timestamp_GET() == 6971774611849498355L);
            assert(pack.target_system_GET() == (char)65);
            assert(pack.target_component_GET() == (char)44);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)65) ;
        p256.secret_key_SET(new char[] {(char)147, (char)149, (char)18, (char)128, (char)174, (char)157, (char)87, (char)76, (char)104, (char)151, (char)158, (char)248, (char)192, (char)165, (char)240, (char)30, (char)249, (char)11, (char)132, (char)150, (char)145, (char)177, (char)157, (char)136, (char)13, (char)112, (char)187, (char)232, (char)38, (char)41, (char)5, (char)220}, 0) ;
        p256.initial_timestamp_SET(6971774611849498355L) ;
        p256.target_component_SET((char)44) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.last_change_ms_GET() == 1215203195L);
            assert(pack.time_boot_ms_GET() == 1456439783L);
            assert(pack.state_GET() == (char)127);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(1456439783L) ;
        p257.last_change_ms_SET(1215203195L) ;
        p257.state_SET((char)127) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)178);
            assert(pack.target_component_GET() == (char)181);
            assert(pack.tune_LEN(ph) == 29);
            assert(pack.tune_TRY(ph).equals("jiqaqlnnarjlmRdpvlszrwfvIgTef"));
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)178) ;
        p258.tune_SET("jiqaqlnnarjlmRdpvlszrwfvIgTef", PH) ;
        p258.target_component_SET((char)181) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE));
            assert(pack.resolution_v_GET() == (char)13987);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)208, (char)68, (char)169, (char)155, (char)247, (char)59, (char)213, (char)170, (char)185, (char)250, (char)75, (char)24, (char)187, (char)160, (char)228, (char)175, (char)79, (char)219, (char)66, (char)60, (char)103, (char)75, (char)171, (char)199, (char)45, (char)244, (char)172, (char)43, (char)134, (char)123, (char)119, (char)244}));
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)83, (char)6, (char)59, (char)104, (char)182, (char)152, (char)188, (char)38, (char)63, (char)220, (char)221, (char)179, (char)178, (char)131, (char)190, (char)78, (char)237, (char)65, (char)143, (char)101, (char)254, (char)250, (char)96, (char)47, (char)4, (char)61, (char)35, (char)38, (char)199, (char)1, (char)253, (char)195}));
            assert(pack.focal_length_GET() == 2.2415894E38F);
            assert(pack.time_boot_ms_GET() == 1902804829L);
            assert(pack.resolution_h_GET() == (char)17062);
            assert(pack.sensor_size_v_GET() == -1.5502856E38F);
            assert(pack.firmware_version_GET() == 1958086404L);
            assert(pack.cam_definition_version_GET() == (char)24976);
            assert(pack.cam_definition_uri_LEN(ph) == 18);
            assert(pack.cam_definition_uri_TRY(ph).equals("bzvnuTqcaPeuodmfrk"));
            assert(pack.lens_id_GET() == (char)38);
            assert(pack.sensor_size_h_GET() == -2.0426327E38F);
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.vendor_name_SET(new char[] {(char)208, (char)68, (char)169, (char)155, (char)247, (char)59, (char)213, (char)170, (char)185, (char)250, (char)75, (char)24, (char)187, (char)160, (char)228, (char)175, (char)79, (char)219, (char)66, (char)60, (char)103, (char)75, (char)171, (char)199, (char)45, (char)244, (char)172, (char)43, (char)134, (char)123, (char)119, (char)244}, 0) ;
        p259.lens_id_SET((char)38) ;
        p259.cam_definition_uri_SET("bzvnuTqcaPeuodmfrk", PH) ;
        p259.time_boot_ms_SET(1902804829L) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE)) ;
        p259.sensor_size_h_SET(-2.0426327E38F) ;
        p259.cam_definition_version_SET((char)24976) ;
        p259.focal_length_SET(2.2415894E38F) ;
        p259.resolution_h_SET((char)17062) ;
        p259.firmware_version_SET(1958086404L) ;
        p259.resolution_v_SET((char)13987) ;
        p259.model_name_SET(new char[] {(char)83, (char)6, (char)59, (char)104, (char)182, (char)152, (char)188, (char)38, (char)63, (char)220, (char)221, (char)179, (char)178, (char)131, (char)190, (char)78, (char)237, (char)65, (char)143, (char)101, (char)254, (char)250, (char)96, (char)47, (char)4, (char)61, (char)35, (char)38, (char)199, (char)1, (char)253, (char)195}, 0) ;
        p259.sensor_size_v_SET(-1.5502856E38F) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2065857373L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY) ;
        p260.time_boot_ms_SET(2065857373L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3809831129L);
            assert(pack.used_capacity_GET() == -1.8902346E38F);
            assert(pack.available_capacity_GET() == 3.2749181E38F);
            assert(pack.status_GET() == (char)171);
            assert(pack.storage_count_GET() == (char)137);
            assert(pack.write_speed_GET() == -1.5670151E38F);
            assert(pack.read_speed_GET() == 3.1008072E38F);
            assert(pack.total_capacity_GET() == -2.9793671E38F);
            assert(pack.storage_id_GET() == (char)65);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.write_speed_SET(-1.5670151E38F) ;
        p261.used_capacity_SET(-1.8902346E38F) ;
        p261.available_capacity_SET(3.2749181E38F) ;
        p261.time_boot_ms_SET(3809831129L) ;
        p261.storage_id_SET((char)65) ;
        p261.read_speed_SET(3.1008072E38F) ;
        p261.status_SET((char)171) ;
        p261.storage_count_SET((char)137) ;
        p261.total_capacity_SET(-2.9793671E38F) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.video_status_GET() == (char)154);
            assert(pack.available_capacity_GET() == 7.4311E37F);
            assert(pack.image_status_GET() == (char)187);
            assert(pack.time_boot_ms_GET() == 2507376316L);
            assert(pack.image_interval_GET() == -8.538312E35F);
            assert(pack.recording_time_ms_GET() == 2013876037L);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.image_status_SET((char)187) ;
        p262.available_capacity_SET(7.4311E37F) ;
        p262.image_interval_SET(-8.538312E35F) ;
        p262.time_boot_ms_SET(2507376316L) ;
        p262.video_status_SET((char)154) ;
        p262.recording_time_ms_SET(2013876037L) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.camera_id_GET() == (char)44);
            assert(pack.capture_result_GET() == (byte)117);
            assert(Arrays.equals(pack.q_GET(),  new float[] {6.4805503E37F, -1.788069E38F, -1.272559E38F, -9.360649E37F}));
            assert(pack.lon_GET() == 1658507956);
            assert(pack.lat_GET() == -513585821);
            assert(pack.time_utc_GET() == 2308641465334969383L);
            assert(pack.relative_alt_GET() == 1993841403);
            assert(pack.alt_GET() == 1658399241);
            assert(pack.image_index_GET() == 1585506874);
            assert(pack.file_url_LEN(ph) == 12);
            assert(pack.file_url_TRY(ph).equals("tnqtxqogcmBr"));
            assert(pack.time_boot_ms_GET() == 2370664955L);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.q_SET(new float[] {6.4805503E37F, -1.788069E38F, -1.272559E38F, -9.360649E37F}, 0) ;
        p263.relative_alt_SET(1993841403) ;
        p263.camera_id_SET((char)44) ;
        p263.time_boot_ms_SET(2370664955L) ;
        p263.image_index_SET(1585506874) ;
        p263.capture_result_SET((byte)117) ;
        p263.lat_SET(-513585821) ;
        p263.alt_SET(1658399241) ;
        p263.lon_SET(1658507956) ;
        p263.file_url_SET("tnqtxqogcmBr", PH) ;
        p263.time_utc_SET(2308641465334969383L) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.takeoff_time_utc_GET() == 3525903596740146658L);
            assert(pack.time_boot_ms_GET() == 2695533582L);
            assert(pack.flight_uuid_GET() == 6477160334434764756L);
            assert(pack.arming_time_utc_GET() == 4576112880856379382L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.arming_time_utc_SET(4576112880856379382L) ;
        p264.flight_uuid_SET(6477160334434764756L) ;
        p264.takeoff_time_utc_SET(3525903596740146658L) ;
        p264.time_boot_ms_SET(2695533582L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 2.1366705E38F);
            assert(pack.pitch_GET() == -2.431204E38F);
            assert(pack.time_boot_ms_GET() == 230655208L);
            assert(pack.roll_GET() == 9.954745E37F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(9.954745E37F) ;
        p265.pitch_SET(-2.431204E38F) ;
        p265.time_boot_ms_SET(230655208L) ;
        p265.yaw_SET(2.1366705E38F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)193);
            assert(pack.sequence_GET() == (char)16623);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)36, (char)252, (char)30, (char)28, (char)166, (char)239, (char)222, (char)196, (char)230, (char)221, (char)59, (char)184, (char)14, (char)246, (char)238, (char)249, (char)197, (char)151, (char)187, (char)179, (char)40, (char)14, (char)231, (char)102, (char)110, (char)182, (char)99, (char)9, (char)83, (char)80, (char)229, (char)102, (char)181, (char)61, (char)1, (char)68, (char)222, (char)62, (char)201, (char)85, (char)114, (char)48, (char)33, (char)73, (char)170, (char)155, (char)225, (char)182, (char)37, (char)187, (char)191, (char)10, (char)99, (char)161, (char)99, (char)141, (char)85, (char)66, (char)42, (char)49, (char)72, (char)191, (char)16, (char)208, (char)132, (char)58, (char)178, (char)35, (char)184, (char)94, (char)171, (char)175, (char)188, (char)234, (char)12, (char)41, (char)97, (char)65, (char)128, (char)163, (char)201, (char)13, (char)212, (char)97, (char)116, (char)167, (char)203, (char)153, (char)130, (char)190, (char)211, (char)40, (char)245, (char)149, (char)44, (char)3, (char)142, (char)213, (char)78, (char)153, (char)59, (char)173, (char)134, (char)29, (char)27, (char)93, (char)210, (char)20, (char)250, (char)90, (char)65, (char)88, (char)167, (char)112, (char)19, (char)22, (char)49, (char)5, (char)86, (char)144, (char)108, (char)42, (char)247, (char)85, (char)125, (char)176, (char)240, (char)125, (char)245, (char)31, (char)117, (char)75, (char)19, (char)9, (char)105, (char)202, (char)105, (char)228, (char)152, (char)185, (char)110, (char)222, (char)65, (char)45, (char)6, (char)78, (char)196, (char)119, (char)91, (char)122, (char)206, (char)192, (char)186, (char)110, (char)182, (char)12, (char)174, (char)181, (char)131, (char)170, (char)149, (char)220, (char)134, (char)233, (char)26, (char)252, (char)84, (char)100, (char)82, (char)43, (char)96, (char)57, (char)209, (char)155, (char)225, (char)183, (char)84, (char)124, (char)171, (char)83, (char)141, (char)253, (char)157, (char)110, (char)184, (char)141, (char)28, (char)72, (char)167, (char)156, (char)168, (char)14, (char)0, (char)53, (char)1, (char)134, (char)152, (char)156, (char)36, (char)224, (char)139, (char)107, (char)3, (char)162, (char)41, (char)136, (char)234, (char)34, (char)125, (char)17, (char)183, (char)19, (char)9, (char)83, (char)224, (char)217, (char)82, (char)34, (char)57, (char)167, (char)161, (char)143, (char)244, (char)48, (char)170, (char)98, (char)164, (char)208, (char)36, (char)205, (char)97, (char)4, (char)223, (char)95, (char)6, (char)72, (char)150, (char)188, (char)196, (char)224, (char)125, (char)50, (char)210, (char)83, (char)37, (char)229, (char)157, (char)163, (char)35}));
            assert(pack.target_component_GET() == (char)207);
            assert(pack.first_message_offset_GET() == (char)223);
            assert(pack.length_GET() == (char)153);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)193) ;
        p266.data__SET(new char[] {(char)36, (char)252, (char)30, (char)28, (char)166, (char)239, (char)222, (char)196, (char)230, (char)221, (char)59, (char)184, (char)14, (char)246, (char)238, (char)249, (char)197, (char)151, (char)187, (char)179, (char)40, (char)14, (char)231, (char)102, (char)110, (char)182, (char)99, (char)9, (char)83, (char)80, (char)229, (char)102, (char)181, (char)61, (char)1, (char)68, (char)222, (char)62, (char)201, (char)85, (char)114, (char)48, (char)33, (char)73, (char)170, (char)155, (char)225, (char)182, (char)37, (char)187, (char)191, (char)10, (char)99, (char)161, (char)99, (char)141, (char)85, (char)66, (char)42, (char)49, (char)72, (char)191, (char)16, (char)208, (char)132, (char)58, (char)178, (char)35, (char)184, (char)94, (char)171, (char)175, (char)188, (char)234, (char)12, (char)41, (char)97, (char)65, (char)128, (char)163, (char)201, (char)13, (char)212, (char)97, (char)116, (char)167, (char)203, (char)153, (char)130, (char)190, (char)211, (char)40, (char)245, (char)149, (char)44, (char)3, (char)142, (char)213, (char)78, (char)153, (char)59, (char)173, (char)134, (char)29, (char)27, (char)93, (char)210, (char)20, (char)250, (char)90, (char)65, (char)88, (char)167, (char)112, (char)19, (char)22, (char)49, (char)5, (char)86, (char)144, (char)108, (char)42, (char)247, (char)85, (char)125, (char)176, (char)240, (char)125, (char)245, (char)31, (char)117, (char)75, (char)19, (char)9, (char)105, (char)202, (char)105, (char)228, (char)152, (char)185, (char)110, (char)222, (char)65, (char)45, (char)6, (char)78, (char)196, (char)119, (char)91, (char)122, (char)206, (char)192, (char)186, (char)110, (char)182, (char)12, (char)174, (char)181, (char)131, (char)170, (char)149, (char)220, (char)134, (char)233, (char)26, (char)252, (char)84, (char)100, (char)82, (char)43, (char)96, (char)57, (char)209, (char)155, (char)225, (char)183, (char)84, (char)124, (char)171, (char)83, (char)141, (char)253, (char)157, (char)110, (char)184, (char)141, (char)28, (char)72, (char)167, (char)156, (char)168, (char)14, (char)0, (char)53, (char)1, (char)134, (char)152, (char)156, (char)36, (char)224, (char)139, (char)107, (char)3, (char)162, (char)41, (char)136, (char)234, (char)34, (char)125, (char)17, (char)183, (char)19, (char)9, (char)83, (char)224, (char)217, (char)82, (char)34, (char)57, (char)167, (char)161, (char)143, (char)244, (char)48, (char)170, (char)98, (char)164, (char)208, (char)36, (char)205, (char)97, (char)4, (char)223, (char)95, (char)6, (char)72, (char)150, (char)188, (char)196, (char)224, (char)125, (char)50, (char)210, (char)83, (char)37, (char)229, (char)157, (char)163, (char)35}, 0) ;
        p266.first_message_offset_SET((char)223) ;
        p266.target_component_SET((char)207) ;
        p266.sequence_SET((char)16623) ;
        p266.length_SET((char)153) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)30523);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)144, (char)7, (char)142, (char)51, (char)69, (char)245, (char)58, (char)24, (char)141, (char)1, (char)26, (char)101, (char)140, (char)60, (char)178, (char)162, (char)225, (char)225, (char)23, (char)192, (char)170, (char)49, (char)2, (char)73, (char)78, (char)8, (char)123, (char)118, (char)96, (char)217, (char)43, (char)83, (char)212, (char)205, (char)99, (char)181, (char)118, (char)41, (char)160, (char)99, (char)243, (char)85, (char)72, (char)55, (char)23, (char)165, (char)191, (char)107, (char)11, (char)100, (char)217, (char)21, (char)45, (char)88, (char)155, (char)94, (char)228, (char)87, (char)180, (char)65, (char)123, (char)10, (char)231, (char)142, (char)126, (char)36, (char)196, (char)205, (char)106, (char)50, (char)57, (char)153, (char)0, (char)112, (char)176, (char)78, (char)105, (char)208, (char)170, (char)9, (char)184, (char)196, (char)36, (char)233, (char)165, (char)211, (char)120, (char)71, (char)242, (char)132, (char)227, (char)163, (char)28, (char)217, (char)104, (char)17, (char)116, (char)86, (char)163, (char)192, (char)172, (char)136, (char)107, (char)195, (char)198, (char)79, (char)63, (char)144, (char)198, (char)41, (char)9, (char)249, (char)17, (char)63, (char)142, (char)154, (char)210, (char)3, (char)180, (char)190, (char)18, (char)83, (char)139, (char)145, (char)5, (char)235, (char)58, (char)150, (char)60, (char)7, (char)225, (char)169, (char)82, (char)116, (char)9, (char)247, (char)31, (char)152, (char)26, (char)142, (char)161, (char)154, (char)188, (char)9, (char)210, (char)167, (char)169, (char)59, (char)159, (char)2, (char)152, (char)52, (char)246, (char)135, (char)218, (char)77, (char)6, (char)129, (char)7, (char)87, (char)92, (char)101, (char)252, (char)39, (char)203, (char)19, (char)182, (char)155, (char)35, (char)151, (char)236, (char)50, (char)234, (char)115, (char)98, (char)200, (char)228, (char)27, (char)67, (char)220, (char)81, (char)231, (char)110, (char)49, (char)116, (char)12, (char)195, (char)13, (char)60, (char)215, (char)76, (char)155, (char)43, (char)59, (char)154, (char)109, (char)98, (char)20, (char)4, (char)241, (char)215, (char)128, (char)20, (char)67, (char)55, (char)82, (char)239, (char)83, (char)168, (char)151, (char)192, (char)141, (char)149, (char)76, (char)26, (char)60, (char)208, (char)161, (char)115, (char)79, (char)229, (char)201, (char)187, (char)114, (char)115, (char)201, (char)41, (char)253, (char)232, (char)116, (char)213, (char)81, (char)26, (char)232, (char)80, (char)180, (char)22, (char)144, (char)145, (char)5, (char)32, (char)229, (char)66, (char)191, (char)113, (char)163, (char)74, (char)13, (char)146}));
            assert(pack.target_component_GET() == (char)22);
            assert(pack.first_message_offset_GET() == (char)133);
            assert(pack.length_GET() == (char)122);
            assert(pack.target_system_GET() == (char)223);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)223) ;
        p267.length_SET((char)122) ;
        p267.target_component_SET((char)22) ;
        p267.first_message_offset_SET((char)133) ;
        p267.sequence_SET((char)30523) ;
        p267.data__SET(new char[] {(char)144, (char)7, (char)142, (char)51, (char)69, (char)245, (char)58, (char)24, (char)141, (char)1, (char)26, (char)101, (char)140, (char)60, (char)178, (char)162, (char)225, (char)225, (char)23, (char)192, (char)170, (char)49, (char)2, (char)73, (char)78, (char)8, (char)123, (char)118, (char)96, (char)217, (char)43, (char)83, (char)212, (char)205, (char)99, (char)181, (char)118, (char)41, (char)160, (char)99, (char)243, (char)85, (char)72, (char)55, (char)23, (char)165, (char)191, (char)107, (char)11, (char)100, (char)217, (char)21, (char)45, (char)88, (char)155, (char)94, (char)228, (char)87, (char)180, (char)65, (char)123, (char)10, (char)231, (char)142, (char)126, (char)36, (char)196, (char)205, (char)106, (char)50, (char)57, (char)153, (char)0, (char)112, (char)176, (char)78, (char)105, (char)208, (char)170, (char)9, (char)184, (char)196, (char)36, (char)233, (char)165, (char)211, (char)120, (char)71, (char)242, (char)132, (char)227, (char)163, (char)28, (char)217, (char)104, (char)17, (char)116, (char)86, (char)163, (char)192, (char)172, (char)136, (char)107, (char)195, (char)198, (char)79, (char)63, (char)144, (char)198, (char)41, (char)9, (char)249, (char)17, (char)63, (char)142, (char)154, (char)210, (char)3, (char)180, (char)190, (char)18, (char)83, (char)139, (char)145, (char)5, (char)235, (char)58, (char)150, (char)60, (char)7, (char)225, (char)169, (char)82, (char)116, (char)9, (char)247, (char)31, (char)152, (char)26, (char)142, (char)161, (char)154, (char)188, (char)9, (char)210, (char)167, (char)169, (char)59, (char)159, (char)2, (char)152, (char)52, (char)246, (char)135, (char)218, (char)77, (char)6, (char)129, (char)7, (char)87, (char)92, (char)101, (char)252, (char)39, (char)203, (char)19, (char)182, (char)155, (char)35, (char)151, (char)236, (char)50, (char)234, (char)115, (char)98, (char)200, (char)228, (char)27, (char)67, (char)220, (char)81, (char)231, (char)110, (char)49, (char)116, (char)12, (char)195, (char)13, (char)60, (char)215, (char)76, (char)155, (char)43, (char)59, (char)154, (char)109, (char)98, (char)20, (char)4, (char)241, (char)215, (char)128, (char)20, (char)67, (char)55, (char)82, (char)239, (char)83, (char)168, (char)151, (char)192, (char)141, (char)149, (char)76, (char)26, (char)60, (char)208, (char)161, (char)115, (char)79, (char)229, (char)201, (char)187, (char)114, (char)115, (char)201, (char)41, (char)253, (char)232, (char)116, (char)213, (char)81, (char)26, (char)232, (char)80, (char)180, (char)22, (char)144, (char)145, (char)5, (char)32, (char)229, (char)66, (char)191, (char)113, (char)163, (char)74, (char)13, (char)146}, 0) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)139);
            assert(pack.target_component_GET() == (char)126);
            assert(pack.sequence_GET() == (char)34451);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_component_SET((char)126) ;
        p268.sequence_SET((char)34451) ;
        p268.target_system_SET((char)139) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.resolution_v_GET() == (char)27061);
            assert(pack.camera_id_GET() == (char)76);
            assert(pack.resolution_h_GET() == (char)22702);
            assert(pack.framerate_GET() == -6.204151E37F);
            assert(pack.status_GET() == (char)88);
            assert(pack.bitrate_GET() == 177698055L);
            assert(pack.uri_LEN(ph) == 133);
            assert(pack.uri_TRY(ph).equals("nwwgyemcveFwqdttgomosingoQvnjomdpqPamrfYfuoZbXmkmImVpiznvshsbedgnhautyMbosOhahizxzpJmzxifqxBrdzrCgJbbhbrbcvxjsdjsonQrxaepbwkahfhlofes"));
            assert(pack.rotation_GET() == (char)35340);
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.framerate_SET(-6.204151E37F) ;
        p269.status_SET((char)88) ;
        p269.rotation_SET((char)35340) ;
        p269.resolution_v_SET((char)27061) ;
        p269.uri_SET("nwwgyemcveFwqdttgomosingoQvnjomdpqPamrfYfuoZbXmkmImVpiznvshsbedgnhautyMbosOhahizxzpJmzxifqxBrdzrCgJbbhbrbcvxjsdjsonQrxaepbwkahfhlofes", PH) ;
        p269.camera_id_SET((char)76) ;
        p269.resolution_h_SET((char)22702) ;
        p269.bitrate_SET(177698055L) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.framerate_GET() == 3.2279185E38F);
            assert(pack.rotation_GET() == (char)13760);
            assert(pack.target_component_GET() == (char)168);
            assert(pack.camera_id_GET() == (char)179);
            assert(pack.bitrate_GET() == 1712615659L);
            assert(pack.resolution_h_GET() == (char)50878);
            assert(pack.uri_LEN(ph) == 153);
            assert(pack.uri_TRY(ph).equals("ojrdyoqqarovuoojuKqjhpomdlcosaiKfzsuIyuikcjwpJafuiltgqbbtanuHcosKagedyzQafHyhturllslburqOxopszvjcfrahwdwdclgjdMjlasxasChrqgcvsuxtXhQnsbxpkldzpxzSqwhUzwad"));
            assert(pack.target_system_GET() == (char)32);
            assert(pack.resolution_v_GET() == (char)26037);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.resolution_h_SET((char)50878) ;
        p270.resolution_v_SET((char)26037) ;
        p270.target_system_SET((char)32) ;
        p270.uri_SET("ojrdyoqqarovuoojuKqjhpomdlcosaiKfzsuIyuikcjwpJafuiltgqbbtanuHcosKagedyzQafHyhturllslburqOxopszvjcfrahwdwdclgjdMjlasxasChrqgcvsuxtXhQnsbxpkldzpxzSqwhUzwad", PH) ;
        p270.bitrate_SET(1712615659L) ;
        p270.framerate_SET(3.2279185E38F) ;
        p270.camera_id_SET((char)179) ;
        p270.target_component_SET((char)168) ;
        p270.rotation_SET((char)13760) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 7);
            assert(pack.password_TRY(ph).equals("odztmeE"));
            assert(pack.ssid_LEN(ph) == 30);
            assert(pack.ssid_TRY(ph).equals("hdwrobnmdaungySdwvXgszdeVnxrtl"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("odztmeE", PH) ;
        p299.ssid_SET("hdwrobnmdaungySdwvXgszdeVnxrtl", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)41, (char)214, (char)91, (char)123, (char)174, (char)97, (char)98, (char)64}));
            assert(pack.version_GET() == (char)6425);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)23, (char)221, (char)24, (char)174, (char)140, (char)40, (char)226, (char)187}));
            assert(pack.max_version_GET() == (char)47087);
            assert(pack.min_version_GET() == (char)18918);
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.spec_version_hash_SET(new char[] {(char)23, (char)221, (char)24, (char)174, (char)140, (char)40, (char)226, (char)187}, 0) ;
        p300.min_version_SET((char)18918) ;
        p300.version_SET((char)6425) ;
        p300.library_version_hash_SET(new char[] {(char)41, (char)214, (char)91, (char)123, (char)174, (char)97, (char)98, (char)64}, 0) ;
        p300.max_version_SET((char)47087) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
            assert(pack.uptime_sec_GET() == 1485619866L);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
            assert(pack.sub_mode_GET() == (char)235);
            assert(pack.time_usec_GET() == 2036954354421995549L);
            assert(pack.vendor_specific_status_code_GET() == (char)21661);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.sub_mode_SET((char)235) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK) ;
        p310.uptime_sec_SET(1485619866L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE) ;
        p310.time_usec_SET(2036954354421995549L) ;
        p310.vendor_specific_status_code_SET((char)21661) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.sw_vcs_commit_GET() == 2166896494L);
            assert(pack.name_LEN(ph) == 68);
            assert(pack.name_TRY(ph).equals("dqjqfzpdfsYdmaZixXcrrTmwnOdqixjclScjtyjslFOmDfcztjlygimcwadxvubyktny"));
            assert(pack.uptime_sec_GET() == 3152564761L);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)227, (char)80, (char)227, (char)201, (char)83, (char)86, (char)171, (char)98, (char)152, (char)194, (char)78, (char)97, (char)107, (char)189, (char)241, (char)169}));
            assert(pack.time_usec_GET() == 5299955468279248354L);
            assert(pack.hw_version_minor_GET() == (char)92);
            assert(pack.hw_version_major_GET() == (char)233);
            assert(pack.sw_version_minor_GET() == (char)45);
            assert(pack.sw_version_major_GET() == (char)193);
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_unique_id_SET(new char[] {(char)227, (char)80, (char)227, (char)201, (char)83, (char)86, (char)171, (char)98, (char)152, (char)194, (char)78, (char)97, (char)107, (char)189, (char)241, (char)169}, 0) ;
        p311.hw_version_major_SET((char)233) ;
        p311.sw_version_minor_SET((char)45) ;
        p311.sw_version_major_SET((char)193) ;
        p311.time_usec_SET(5299955468279248354L) ;
        p311.name_SET("dqjqfzpdfsYdmaZixXcrrTmwnOdqixjclScjtyjslFOmDfcztjlygimcwadxvubyktny", PH) ;
        p311.sw_vcs_commit_SET(2166896494L) ;
        p311.uptime_sec_SET(3152564761L) ;
        p311.hw_version_minor_SET((char)92) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)53);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("Sr"));
            assert(pack.target_component_GET() == (char)58);
            assert(pack.param_index_GET() == (short) -7190);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_component_SET((char)58) ;
        p320.param_index_SET((short) -7190) ;
        p320.target_system_SET((char)53) ;
        p320.param_id_SET("Sr", PH) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)227);
            assert(pack.target_system_GET() == (char)91);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)91) ;
        p321.target_component_SET((char)227) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("vngmhvIDwnfmliy"));
            assert(pack.param_count_GET() == (char)15982);
            assert(pack.param_value_LEN(ph) == 93);
            assert(pack.param_value_TRY(ph).equals("bxdiIjucvpeywTxHltblfMnitgefhzzndbwkbpvvadgjajlbzphypubSRxwnynMbUjfosmkspxXnltphqzmmekTbtDqaR"));
            assert(pack.param_index_GET() == (char)48169);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64) ;
        p322.param_count_SET((char)15982) ;
        p322.param_index_SET((char)48169) ;
        p322.param_id_SET("vngmhvIDwnfmliy", PH) ;
        p322.param_value_SET("bxdiIjucvpeywTxHltblfMnitgefhzzndbwkbpvvadgjajlbzphypubSRxwnynMbUjfosmkspxXnltphqzmmekTbtDqaR", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 71);
            assert(pack.param_value_TRY(ph).equals("iaamwdrbmosmevNrVNqqtbddyHmzaaQbeiaqavObrlkrmnnzbacMffdyqhelyFwqqyqJZec"));
            assert(pack.target_component_GET() == (char)183);
            assert(pack.param_id_LEN(ph) == 13);
            assert(pack.param_id_TRY(ph).equals("aeawhdxiidhgh"));
            assert(pack.target_system_GET() == (char)242);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16) ;
        p323.target_system_SET((char)242) ;
        p323.param_id_SET("aeawhdxiidhgh", PH) ;
        p323.target_component_SET((char)183) ;
        p323.param_value_SET("iaamwdrbmosmevNrVNqqtbddyHmzaaQbeiaqavObrlkrmnnzbacMffdyqhelyFwqqyqJZec", PH) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("buobuqkvvXla"));
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_FAILED);
            assert(pack.param_value_LEN(ph) == 54);
            assert(pack.param_value_TRY(ph).equals("yOiGtsttklxrzkyfpjnihjzpasnidNrZrtvfnDzpwvfcdBtTezxYxr"));
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM) ;
        p324.param_value_SET("yOiGtsttklxrzkyfpjnihjzpasnidNrZrtvfnDzpwvfcdBtTezxYxr", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED) ;
        p324.param_id_SET("buobuqkvvXla", PH) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 1732852819402344775L);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)42268, (char)2313, (char)18834, (char)9129, (char)29463, (char)34926, (char)30345, (char)23591, (char)38535, (char)42569, (char)3969, (char)1174, (char)3241, (char)3686, (char)3731, (char)47738, (char)38350, (char)20322, (char)36400, (char)37174, (char)32200, (char)42800, (char)9003, (char)21616, (char)7302, (char)5601, (char)33856, (char)47417, (char)30851, (char)19486, (char)24310, (char)63429, (char)64457, (char)14021, (char)38227, (char)34886, (char)1716, (char)46949, (char)34831, (char)51360, (char)62121, (char)17263, (char)29599, (char)50006, (char)44293, (char)24754, (char)49287, (char)24275, (char)34056, (char)38212, (char)26056, (char)30549, (char)44650, (char)12137, (char)43100, (char)7495, (char)42364, (char)23266, (char)40066, (char)50240, (char)14660, (char)30552, (char)51764, (char)31333, (char)59781, (char)44700, (char)501, (char)1305, (char)6911, (char)4837, (char)61247, (char)56037}));
            assert(pack.increment_GET() == (char)165);
            assert(pack.max_distance_GET() == (char)64456);
            assert(pack.min_distance_GET() == (char)55894);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.min_distance_SET((char)55894) ;
        p330.max_distance_SET((char)64456) ;
        p330.increment_SET((char)165) ;
        p330.distances_SET(new char[] {(char)42268, (char)2313, (char)18834, (char)9129, (char)29463, (char)34926, (char)30345, (char)23591, (char)38535, (char)42569, (char)3969, (char)1174, (char)3241, (char)3686, (char)3731, (char)47738, (char)38350, (char)20322, (char)36400, (char)37174, (char)32200, (char)42800, (char)9003, (char)21616, (char)7302, (char)5601, (char)33856, (char)47417, (char)30851, (char)19486, (char)24310, (char)63429, (char)64457, (char)14021, (char)38227, (char)34886, (char)1716, (char)46949, (char)34831, (char)51360, (char)62121, (char)17263, (char)29599, (char)50006, (char)44293, (char)24754, (char)49287, (char)24275, (char)34056, (char)38212, (char)26056, (char)30549, (char)44650, (char)12137, (char)43100, (char)7495, (char)42364, (char)23266, (char)40066, (char)50240, (char)14660, (char)30552, (char)51764, (char)31333, (char)59781, (char)44700, (char)501, (char)1305, (char)6911, (char)4837, (char)61247, (char)56037}, 0) ;
        p330.time_usec_SET(1732852819402344775L) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
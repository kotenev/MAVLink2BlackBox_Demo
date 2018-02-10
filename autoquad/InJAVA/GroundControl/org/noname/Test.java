
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
            long id = id__I(src);
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
            long id = id__a(src);
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
            long id = id__a(src);
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
            assert(pack.mavlink_version_GET() == (char)90);
            assert(pack.custom_mode_GET() == 979395143L);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_SURFACE_BOAT);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_RESERVED);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_FLIGHT_TERMINATION);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.type_SET(MAV_TYPE.MAV_TYPE_SURFACE_BOAT) ;
        p0.custom_mode_SET(979395143L) ;
        p0.mavlink_version_SET((char)90) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_FLIGHT_TERMINATION) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED)) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_RESERVED) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.voltage_battery_GET() == (char)6180);
            assert(pack.drop_rate_comm_GET() == (char)57967);
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.battery_remaining_GET() == (byte) - 79);
            assert(pack.load_GET() == (char)19302);
            assert(pack.errors_comm_GET() == (char)8542);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY));
            assert(pack.errors_count3_GET() == (char)55413);
            assert(pack.current_battery_GET() == (short) -25105);
            assert(pack.errors_count1_GET() == (char)6641);
            assert(pack.errors_count2_GET() == (char)53034);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION));
            assert(pack.errors_count4_GET() == (char)14977);
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.errors_count2_SET((char)53034) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.battery_remaining_SET((byte) - 79) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY)) ;
        p1.current_battery_SET((short) -25105) ;
        p1.errors_count1_SET((char)6641) ;
        p1.errors_comm_SET((char)8542) ;
        p1.voltage_battery_SET((char)6180) ;
        p1.errors_count4_SET((char)14977) ;
        p1.errors_count3_SET((char)55413) ;
        p1.drop_rate_comm_SET((char)57967) ;
        p1.load_SET((char)19302) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION)) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 3142600620425599716L);
            assert(pack.time_boot_ms_GET() == 611565983L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(611565983L) ;
        p2.time_unix_usec_SET(3142600620425599716L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -1.212052E38F);
            assert(pack.vz_GET() == -1.7540025E37F);
            assert(pack.yaw_rate_GET() == 2.4064651E38F);
            assert(pack.type_mask_GET() == (char)4619);
            assert(pack.afx_GET() == 1.6086261E38F);
            assert(pack.time_boot_ms_GET() == 3833771644L);
            assert(pack.vx_GET() == 3.3449543E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.afz_GET() == 2.9422093E38F);
            assert(pack.afy_GET() == -3.3807497E38F);
            assert(pack.yaw_GET() == -8.683238E37F);
            assert(pack.vy_GET() == 2.8046388E38F);
            assert(pack.z_GET() == -1.209722E38F);
            assert(pack.x_GET() == 3.2533173E37F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.time_boot_ms_SET(3833771644L) ;
        p3.yaw_SET(-8.683238E37F) ;
        p3.y_SET(-1.212052E38F) ;
        p3.afx_SET(1.6086261E38F) ;
        p3.yaw_rate_SET(2.4064651E38F) ;
        p3.afy_SET(-3.3807497E38F) ;
        p3.z_SET(-1.209722E38F) ;
        p3.vz_SET(-1.7540025E37F) ;
        p3.vy_SET(2.8046388E38F) ;
        p3.vx_SET(3.3449543E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p3.type_mask_SET((char)4619) ;
        p3.afz_SET(2.9422093E38F) ;
        p3.x_SET(3.2533173E37F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 1029383531L);
            assert(pack.target_system_GET() == (char)86);
            assert(pack.time_usec_GET() == 5128266278369491463L);
            assert(pack.target_component_GET() == (char)157);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.target_system_SET((char)86) ;
        p4.target_component_SET((char)157) ;
        p4.seq_SET(1029383531L) ;
        p4.time_usec_SET(5128266278369491463L) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)150);
            assert(pack.control_request_GET() == (char)157);
            assert(pack.passkey_LEN(ph) == 19);
            assert(pack.passkey_TRY(ph).equals("plEykaOnurxolirdYdN"));
            assert(pack.version_GET() == (char)206);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.passkey_SET("plEykaOnurxolirdYdN", PH) ;
        p5.target_system_SET((char)150) ;
        p5.control_request_SET((char)157) ;
        p5.version_SET((char)206) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)158);
            assert(pack.ack_GET() == (char)121);
            assert(pack.control_request_GET() == (char)158);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)158) ;
        p6.ack_SET((char)121) ;
        p6.control_request_SET((char)158) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 27);
            assert(pack.key_TRY(ph).equals("gtkaxvNtoJyieVqzkMxvbrewArQ"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("gtkaxvNtoJyieVqzkMxvbrewArQ", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.custom_mode_GET() == 2406645494L);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_ARMED);
            assert(pack.target_system_GET() == (char)198);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)198) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_STABILIZE_ARMED) ;
        p11.custom_mode_SET(2406645494L) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("J"));
            assert(pack.target_system_GET() == (char)222);
            assert(pack.target_component_GET() == (char)177);
            assert(pack.param_index_GET() == (short)19868);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)222) ;
        p20.param_id_SET("J", PH) ;
        p20.param_index_SET((short)19868) ;
        p20.target_component_SET((char)177) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)228);
            assert(pack.target_component_GET() == (char)33);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)33) ;
        p21.target_system_SET((char)228) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)42313);
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("sgjEmvkiSNYfrdLm"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
            assert(pack.param_index_GET() == (char)44526);
            assert(pack.param_value_GET() == -7.122984E37F);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_count_SET((char)42313) ;
        p22.param_id_SET("sgjEmvkiSNYfrdLm", PH) ;
        p22.param_index_SET((char)44526) ;
        p22.param_value_SET(-7.122984E37F) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)54);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("k"));
            assert(pack.target_component_GET() == (char)163);
            assert(pack.param_value_GET() == -3.565973E37F);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64) ;
        p23.param_id_SET("k", PH) ;
        p23.param_value_SET(-3.565973E37F) ;
        p23.target_component_SET((char)163) ;
        p23.target_system_SET((char)54) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)44);
            assert(pack.alt_ellipsoid_TRY(ph) == -1810597632);
            assert(pack.vel_GET() == (char)36374);
            assert(pack.epv_GET() == (char)49011);
            assert(pack.alt_GET() == -1840841703);
            assert(pack.lon_GET() == 633797526);
            assert(pack.lat_GET() == -1563061512);
            assert(pack.vel_acc_TRY(ph) == 639410824L);
            assert(pack.cog_GET() == (char)3796);
            assert(pack.v_acc_TRY(ph) == 2177129182L);
            assert(pack.h_acc_TRY(ph) == 3836276666L);
            assert(pack.eph_GET() == (char)61745);
            assert(pack.time_usec_GET() == 2600449929711950815L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
            assert(pack.hdg_acc_TRY(ph) == 3053111542L);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.eph_SET((char)61745) ;
        p24.time_usec_SET(2600449929711950815L) ;
        p24.vel_SET((char)36374) ;
        p24.vel_acc_SET(639410824L, PH) ;
        p24.h_acc_SET(3836276666L, PH) ;
        p24.cog_SET((char)3796) ;
        p24.satellites_visible_SET((char)44) ;
        p24.alt_ellipsoid_SET(-1810597632, PH) ;
        p24.lat_SET(-1563061512) ;
        p24.epv_SET((char)49011) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX) ;
        p24.v_acc_SET(2177129182L, PH) ;
        p24.alt_SET(-1840841703) ;
        p24.hdg_acc_SET(3053111542L, PH) ;
        p24.lon_SET(633797526) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)116, (char)205, (char)34, (char)142, (char)87, (char)71, (char)188, (char)1, (char)16, (char)208, (char)153, (char)10, (char)182, (char)254, (char)51, (char)30, (char)55, (char)166, (char)216, (char)144}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)202, (char)152, (char)238, (char)135, (char)244, (char)38, (char)149, (char)54, (char)207, (char)166, (char)78, (char)128, (char)107, (char)131, (char)109, (char)39, (char)124, (char)168, (char)249, (char)203}));
            assert(pack.satellites_visible_GET() == (char)201);
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)32, (char)246, (char)150, (char)178, (char)168, (char)155, (char)6, (char)45, (char)151, (char)40, (char)177, (char)191, (char)96, (char)59, (char)177, (char)218, (char)179, (char)88, (char)233, (char)25}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)89, (char)250, (char)2, (char)192, (char)127, (char)0, (char)96, (char)93, (char)237, (char)149, (char)23, (char)209, (char)40, (char)174, (char)240, (char)64, (char)192, (char)162, (char)219, (char)48}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)177, (char)171, (char)67, (char)138, (char)160, (char)119, (char)89, (char)237, (char)4, (char)87, (char)66, (char)92, (char)252, (char)81, (char)228, (char)38, (char)249, (char)228, (char)244, (char)49}));
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_snr_SET(new char[] {(char)32, (char)246, (char)150, (char)178, (char)168, (char)155, (char)6, (char)45, (char)151, (char)40, (char)177, (char)191, (char)96, (char)59, (char)177, (char)218, (char)179, (char)88, (char)233, (char)25}, 0) ;
        p25.satellite_used_SET(new char[] {(char)116, (char)205, (char)34, (char)142, (char)87, (char)71, (char)188, (char)1, (char)16, (char)208, (char)153, (char)10, (char)182, (char)254, (char)51, (char)30, (char)55, (char)166, (char)216, (char)144}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)89, (char)250, (char)2, (char)192, (char)127, (char)0, (char)96, (char)93, (char)237, (char)149, (char)23, (char)209, (char)40, (char)174, (char)240, (char)64, (char)192, (char)162, (char)219, (char)48}, 0) ;
        p25.satellites_visible_SET((char)201) ;
        p25.satellite_prn_SET(new char[] {(char)177, (char)171, (char)67, (char)138, (char)160, (char)119, (char)89, (char)237, (char)4, (char)87, (char)66, (char)92, (char)252, (char)81, (char)228, (char)38, (char)249, (char)228, (char)244, (char)49}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)202, (char)152, (char)238, (char)135, (char)244, (char)38, (char)149, (char)54, (char)207, (char)166, (char)78, (char)128, (char)107, (char)131, (char)109, (char)39, (char)124, (char)168, (char)249, (char)203}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short) -30908);
            assert(pack.ymag_GET() == (short)10881);
            assert(pack.yacc_GET() == (short) -20854);
            assert(pack.xgyro_GET() == (short) -15717);
            assert(pack.time_boot_ms_GET() == 372021641L);
            assert(pack.zgyro_GET() == (short) -21995);
            assert(pack.xmag_GET() == (short)2771);
            assert(pack.zmag_GET() == (short) -12074);
            assert(pack.zacc_GET() == (short) -29952);
            assert(pack.ygyro_GET() == (short) -21027);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.xgyro_SET((short) -15717) ;
        p26.xacc_SET((short) -30908) ;
        p26.ygyro_SET((short) -21027) ;
        p26.xmag_SET((short)2771) ;
        p26.zmag_SET((short) -12074) ;
        p26.ymag_SET((short)10881) ;
        p26.zgyro_SET((short) -21995) ;
        p26.zacc_SET((short) -29952) ;
        p26.yacc_SET((short) -20854) ;
        p26.time_boot_ms_SET(372021641L) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short)6443);
            assert(pack.time_usec_GET() == 5848763499615459696L);
            assert(pack.yacc_GET() == (short)1955);
            assert(pack.xmag_GET() == (short) -17763);
            assert(pack.xgyro_GET() == (short)11945);
            assert(pack.zmag_GET() == (short)19316);
            assert(pack.zacc_GET() == (short) -11587);
            assert(pack.ymag_GET() == (short)1763);
            assert(pack.ygyro_GET() == (short) -5971);
            assert(pack.zgyro_GET() == (short)22265);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.zgyro_SET((short)22265) ;
        p27.yacc_SET((short)1955) ;
        p27.zacc_SET((short) -11587) ;
        p27.xgyro_SET((short)11945) ;
        p27.ygyro_SET((short) -5971) ;
        p27.time_usec_SET(5848763499615459696L) ;
        p27.zmag_SET((short)19316) ;
        p27.xmag_SET((short) -17763) ;
        p27.xacc_SET((short)6443) ;
        p27.ymag_SET((short)1763) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)32136);
            assert(pack.press_diff2_GET() == (short)5206);
            assert(pack.time_usec_GET() == 1843411240702412984L);
            assert(pack.press_diff1_GET() == (short)21842);
            assert(pack.press_abs_GET() == (short)12054);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_diff1_SET((short)21842) ;
        p28.temperature_SET((short)32136) ;
        p28.press_abs_SET((short)12054) ;
        p28.press_diff2_SET((short)5206) ;
        p28.time_usec_SET(1843411240702412984L) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1186511813L);
            assert(pack.press_diff_GET() == -8.162789E37F);
            assert(pack.press_abs_GET() == -3.5127047E37F);
            assert(pack.temperature_GET() == (short) -1303);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_abs_SET(-3.5127047E37F) ;
        p29.press_diff_SET(-8.162789E37F) ;
        p29.time_boot_ms_SET(1186511813L) ;
        p29.temperature_SET((short) -1303) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == -8.1456053E37F);
            assert(pack.pitchspeed_GET() == -2.9548138E38F);
            assert(pack.pitch_GET() == -1.9283444E38F);
            assert(pack.roll_GET() == 1.0541589E38F);
            assert(pack.rollspeed_GET() == -2.5826016E38F);
            assert(pack.time_boot_ms_GET() == 754278972L);
            assert(pack.yaw_GET() == 7.3370836E37F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.roll_SET(1.0541589E38F) ;
        p30.rollspeed_SET(-2.5826016E38F) ;
        p30.yaw_SET(7.3370836E37F) ;
        p30.pitch_SET(-1.9283444E38F) ;
        p30.yawspeed_SET(-8.1456053E37F) ;
        p30.pitchspeed_SET(-2.9548138E38F) ;
        p30.time_boot_ms_SET(754278972L) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q1_GET() == 2.9166549E37F);
            assert(pack.q3_GET() == -8.802339E37F);
            assert(pack.rollspeed_GET() == -4.328534E37F);
            assert(pack.q4_GET() == 1.4524296E37F);
            assert(pack.q2_GET() == 6.1790655E37F);
            assert(pack.yawspeed_GET() == 3.365448E38F);
            assert(pack.pitchspeed_GET() == -2.2926244E38F);
            assert(pack.time_boot_ms_GET() == 899543867L);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q1_SET(2.9166549E37F) ;
        p31.time_boot_ms_SET(899543867L) ;
        p31.yawspeed_SET(3.365448E38F) ;
        p31.q3_SET(-8.802339E37F) ;
        p31.q2_SET(6.1790655E37F) ;
        p31.pitchspeed_SET(-2.2926244E38F) ;
        p31.rollspeed_SET(-4.328534E37F) ;
        p31.q4_SET(1.4524296E37F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -2.1689355E37F);
            assert(pack.x_GET() == 2.5185386E38F);
            assert(pack.time_boot_ms_GET() == 2595650995L);
            assert(pack.vx_GET() == 1.895469E38F);
            assert(pack.vz_GET() == 9.758136E37F);
            assert(pack.vy_GET() == 1.759317E38F);
            assert(pack.y_GET() == 2.533068E37F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.time_boot_ms_SET(2595650995L) ;
        p32.vy_SET(1.759317E38F) ;
        p32.vx_SET(1.895469E38F) ;
        p32.z_SET(-2.1689355E37F) ;
        p32.y_SET(2.533068E37F) ;
        p32.x_SET(2.5185386E38F) ;
        p32.vz_SET(9.758136E37F) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1403576882L);
            assert(pack.relative_alt_GET() == -170860694);
            assert(pack.vx_GET() == (short)22427);
            assert(pack.hdg_GET() == (char)21415);
            assert(pack.lat_GET() == 60089425);
            assert(pack.lon_GET() == 1520375582);
            assert(pack.vz_GET() == (short) -28225);
            assert(pack.vy_GET() == (short)3917);
            assert(pack.alt_GET() == -1501976665);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.lon_SET(1520375582) ;
        p33.vx_SET((short)22427) ;
        p33.vy_SET((short)3917) ;
        p33.vz_SET((short) -28225) ;
        p33.hdg_SET((char)21415) ;
        p33.relative_alt_SET(-170860694) ;
        p33.lat_SET(60089425) ;
        p33.time_boot_ms_SET(1403576882L) ;
        p33.alt_SET(-1501976665) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan1_scaled_GET() == (short)5706);
            assert(pack.chan2_scaled_GET() == (short)11453);
            assert(pack.rssi_GET() == (char)209);
            assert(pack.chan4_scaled_GET() == (short) -20133);
            assert(pack.time_boot_ms_GET() == 1812354909L);
            assert(pack.chan5_scaled_GET() == (short) -7292);
            assert(pack.chan6_scaled_GET() == (short) -12534);
            assert(pack.port_GET() == (char)83);
            assert(pack.chan3_scaled_GET() == (short) -16848);
            assert(pack.chan7_scaled_GET() == (short)11398);
            assert(pack.chan8_scaled_GET() == (short)15786);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.rssi_SET((char)209) ;
        p34.port_SET((char)83) ;
        p34.time_boot_ms_SET(1812354909L) ;
        p34.chan2_scaled_SET((short)11453) ;
        p34.chan7_scaled_SET((short)11398) ;
        p34.chan8_scaled_SET((short)15786) ;
        p34.chan4_scaled_SET((short) -20133) ;
        p34.chan5_scaled_SET((short) -7292) ;
        p34.chan3_scaled_SET((short) -16848) ;
        p34.chan6_scaled_SET((short) -12534) ;
        p34.chan1_scaled_SET((short)5706) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan5_raw_GET() == (char)52970);
            assert(pack.chan8_raw_GET() == (char)31466);
            assert(pack.time_boot_ms_GET() == 3336021032L);
            assert(pack.chan4_raw_GET() == (char)43907);
            assert(pack.port_GET() == (char)143);
            assert(pack.chan2_raw_GET() == (char)36814);
            assert(pack.chan1_raw_GET() == (char)35527);
            assert(pack.chan3_raw_GET() == (char)45440);
            assert(pack.rssi_GET() == (char)207);
            assert(pack.chan7_raw_GET() == (char)16366);
            assert(pack.chan6_raw_GET() == (char)6227);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan7_raw_SET((char)16366) ;
        p35.chan8_raw_SET((char)31466) ;
        p35.chan4_raw_SET((char)43907) ;
        p35.chan5_raw_SET((char)52970) ;
        p35.chan3_raw_SET((char)45440) ;
        p35.time_boot_ms_SET(3336021032L) ;
        p35.rssi_SET((char)207) ;
        p35.chan2_raw_SET((char)36814) ;
        p35.port_SET((char)143) ;
        p35.chan1_raw_SET((char)35527) ;
        p35.chan6_raw_SET((char)6227) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo7_raw_GET() == (char)12582);
            assert(pack.servo6_raw_GET() == (char)61088);
            assert(pack.servo8_raw_GET() == (char)14768);
            assert(pack.servo16_raw_TRY(ph) == (char)17675);
            assert(pack.servo15_raw_TRY(ph) == (char)48229);
            assert(pack.servo12_raw_TRY(ph) == (char)277);
            assert(pack.servo3_raw_GET() == (char)16883);
            assert(pack.servo13_raw_TRY(ph) == (char)12747);
            assert(pack.port_GET() == (char)135);
            assert(pack.servo9_raw_TRY(ph) == (char)54189);
            assert(pack.servo14_raw_TRY(ph) == (char)61256);
            assert(pack.time_usec_GET() == 2405930709L);
            assert(pack.servo2_raw_GET() == (char)65336);
            assert(pack.servo11_raw_TRY(ph) == (char)14632);
            assert(pack.servo5_raw_GET() == (char)4239);
            assert(pack.servo10_raw_TRY(ph) == (char)24356);
            assert(pack.servo1_raw_GET() == (char)59226);
            assert(pack.servo4_raw_GET() == (char)28439);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo4_raw_SET((char)28439) ;
        p36.servo11_raw_SET((char)14632, PH) ;
        p36.servo7_raw_SET((char)12582) ;
        p36.servo13_raw_SET((char)12747, PH) ;
        p36.servo10_raw_SET((char)24356, PH) ;
        p36.servo8_raw_SET((char)14768) ;
        p36.servo3_raw_SET((char)16883) ;
        p36.servo2_raw_SET((char)65336) ;
        p36.servo12_raw_SET((char)277, PH) ;
        p36.servo1_raw_SET((char)59226) ;
        p36.servo9_raw_SET((char)54189, PH) ;
        p36.servo6_raw_SET((char)61088) ;
        p36.servo14_raw_SET((char)61256, PH) ;
        p36.port_SET((char)135) ;
        p36.servo15_raw_SET((char)48229, PH) ;
        p36.servo16_raw_SET((char)17675, PH) ;
        p36.servo5_raw_SET((char)4239) ;
        p36.time_usec_SET(2405930709L) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short)4451);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)246);
            assert(pack.target_component_GET() == (char)113);
            assert(pack.end_index_GET() == (short)27274);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_system_SET((char)246) ;
        p37.target_component_SET((char)113) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p37.end_index_SET((short)27274) ;
        p37.start_index_SET((short)4451) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)24);
            assert(pack.target_component_GET() == (char)236);
            assert(pack.start_index_GET() == (short) -27146);
            assert(pack.end_index_GET() == (short) -19650);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.start_index_SET((short) -27146) ;
        p38.target_system_SET((char)24) ;
        p38.end_index_SET((short) -19650) ;
        p38.target_component_SET((char)236) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)53516);
            assert(pack.param2_GET() == 1.9007492E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.param1_GET() == 2.268556E38F);
            assert(pack.target_component_GET() == (char)218);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.current_GET() == (char)16);
            assert(pack.param3_GET() == -1.3923655E38F);
            assert(pack.param4_GET() == 3.3051345E38F);
            assert(pack.target_system_GET() == (char)242);
            assert(pack.autocontinue_GET() == (char)145);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH);
            assert(pack.y_GET() == -2.332195E38F);
            assert(pack.x_GET() == 1.9513996E38F);
            assert(pack.z_GET() == -2.1268003E38F);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p39.target_system_SET((char)242) ;
        p39.z_SET(-2.1268003E38F) ;
        p39.param3_SET(-1.3923655E38F) ;
        p39.x_SET(1.9513996E38F) ;
        p39.current_SET((char)16) ;
        p39.autocontinue_SET((char)145) ;
        p39.target_component_SET((char)218) ;
        p39.param1_SET(2.268556E38F) ;
        p39.y_SET(-2.332195E38F) ;
        p39.seq_SET((char)53516) ;
        p39.param2_SET(1.9007492E38F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p39.param4_SET(3.3051345E38F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)194);
            assert(pack.seq_GET() == (char)7697);
            assert(pack.target_component_GET() == (char)218);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION) ;
        p40.target_system_SET((char)194) ;
        p40.seq_SET((char)7697) ;
        p40.target_component_SET((char)218) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)240);
            assert(pack.target_system_GET() == (char)232);
            assert(pack.seq_GET() == (char)29639);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.seq_SET((char)29639) ;
        p41.target_system_SET((char)232) ;
        p41.target_component_SET((char)240) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)14520);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)14520) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)140);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)191);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p43.target_system_SET((char)191) ;
        p43.target_component_SET((char)140) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)46);
            assert(pack.count_GET() == (char)4998);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_component_GET() == (char)9);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p44.count_SET((char)4998) ;
        p44.target_system_SET((char)46) ;
        p44.target_component_SET((char)9) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)80);
            assert(pack.target_component_GET() == (char)140);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)80) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p45.target_component_SET((char)140) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)55079);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)55079) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM4);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION);
            assert(pack.target_component_GET() == (char)83);
            assert(pack.target_system_GET() == (char)32);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION) ;
        p47.target_component_SET((char)83) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM4) ;
        p47.target_system_SET((char)32) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)44);
            assert(pack.altitude_GET() == -875669337);
            assert(pack.time_usec_TRY(ph) == 3742962289157442381L);
            assert(pack.longitude_GET() == -1323204888);
            assert(pack.latitude_GET() == 1657919680);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.altitude_SET(-875669337) ;
        p48.longitude_SET(-1323204888) ;
        p48.latitude_SET(1657919680) ;
        p48.time_usec_SET(3742962289157442381L, PH) ;
        p48.target_system_SET((char)44) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 174879845);
            assert(pack.altitude_GET() == 1739352908);
            assert(pack.time_usec_TRY(ph) == 7654909677385456114L);
            assert(pack.longitude_GET() == -1222048673);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(174879845) ;
        p49.time_usec_SET(7654909677385456114L, PH) ;
        p49.altitude_SET(1739352908) ;
        p49.longitude_SET(-1222048673) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short)17342);
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("owa"));
            assert(pack.parameter_rc_channel_index_GET() == (char)234);
            assert(pack.target_system_GET() == (char)230);
            assert(pack.param_value_min_GET() == 5.439433E37F);
            assert(pack.param_value_max_GET() == -3.3709136E38F);
            assert(pack.target_component_GET() == (char)243);
            assert(pack.scale_GET() == 5.861053E37F);
            assert(pack.param_value0_GET() == 2.947243E38F);
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.param_value_max_SET(-3.3709136E38F) ;
        p50.target_component_SET((char)243) ;
        p50.param_value0_SET(2.947243E38F) ;
        p50.param_value_min_SET(5.439433E37F) ;
        p50.scale_SET(5.861053E37F) ;
        p50.target_system_SET((char)230) ;
        p50.parameter_rc_channel_index_SET((char)234) ;
        p50.param_id_SET("owa", PH) ;
        p50.param_index_SET((short)17342) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)41735);
            assert(pack.target_system_GET() == (char)91);
            assert(pack.target_component_GET() == (char)201);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.seq_SET((char)41735) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p51.target_component_SET((char)201) ;
        p51.target_system_SET((char)91) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2y_GET() == 2.7602926E38F);
            assert(pack.p1y_GET() == 1.1440007E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.p2x_GET() == 4.857587E36F);
            assert(pack.p1z_GET() == -1.3307142E38F);
            assert(pack.p2z_GET() == 2.710444E38F);
            assert(pack.target_system_GET() == (char)175);
            assert(pack.p1x_GET() == 6.52941E37F);
            assert(pack.target_component_GET() == (char)170);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p1x_SET(6.52941E37F) ;
        p54.p2z_SET(2.710444E38F) ;
        p54.p1z_SET(-1.3307142E38F) ;
        p54.p1y_SET(1.1440007E38F) ;
        p54.target_component_SET((char)170) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p54.p2x_SET(4.857587E36F) ;
        p54.target_system_SET((char)175) ;
        p54.p2y_SET(2.7602926E38F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1x_GET() == -2.6255533E38F);
            assert(pack.p2y_GET() == 2.7389023E38F);
            assert(pack.p1z_GET() == -7.071191E36F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.p2z_GET() == 1.171031E38F);
            assert(pack.p1y_GET() == -2.9089417E38F);
            assert(pack.p2x_GET() == -1.1725757E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p1z_SET(-7.071191E36F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p55.p1x_SET(-2.6255533E38F) ;
        p55.p2y_SET(2.7389023E38F) ;
        p55.p2x_SET(-1.1725757E38F) ;
        p55.p2z_SET(1.171031E38F) ;
        p55.p1y_SET(-2.9089417E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.2842453E37F, -1.0813711E38F, 1.4004349E38F, -1.7340602E38F, -1.7651167E38F, 1.6576786E38F, 3.1112602E38F, 2.289521E37F, -3.0819166E38F}));
            assert(pack.pitchspeed_GET() == 3.1901926E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.914857E38F, 2.6754046E38F, 1.8274755E38F, 2.2225132E37F}));
            assert(pack.time_usec_GET() == 5811706753490706588L);
            assert(pack.rollspeed_GET() == 7.865074E37F);
            assert(pack.yawspeed_GET() == 1.3522997E38F);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.rollspeed_SET(7.865074E37F) ;
        p61.time_usec_SET(5811706753490706588L) ;
        p61.pitchspeed_SET(3.1901926E38F) ;
        p61.covariance_SET(new float[] {2.2842453E37F, -1.0813711E38F, 1.4004349E38F, -1.7340602E38F, -1.7651167E38F, 1.6576786E38F, 3.1112602E38F, 2.289521E37F, -3.0819166E38F}, 0) ;
        p61.q_SET(new float[] {-2.914857E38F, 2.6754046E38F, 1.8274755E38F, 2.2225132E37F}, 0) ;
        p61.yawspeed_SET(1.3522997E38F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_pitch_GET() == 1.2533096E38F);
            assert(pack.target_bearing_GET() == (short)11970);
            assert(pack.nav_roll_GET() == 6.51133E37F);
            assert(pack.xtrack_error_GET() == 6.754616E37F);
            assert(pack.alt_error_GET() == -1.4769817E38F);
            assert(pack.nav_bearing_GET() == (short) -10684);
            assert(pack.wp_dist_GET() == (char)56521);
            assert(pack.aspd_error_GET() == -3.3413187E38F);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.wp_dist_SET((char)56521) ;
        p62.xtrack_error_SET(6.754616E37F) ;
        p62.nav_bearing_SET((short) -10684) ;
        p62.aspd_error_SET(-3.3413187E38F) ;
        p62.nav_pitch_SET(1.2533096E38F) ;
        p62.target_bearing_SET((short)11970) ;
        p62.alt_error_SET(-1.4769817E38F) ;
        p62.nav_roll_SET(6.51133E37F) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.6211398E38F, 1.5700433E38F, 2.1213115E38F, 3.2997268E38F, 3.3810284E38F, 1.2001938E38F, -3.0574221E38F, -1.0991589E38F, -2.1700858E38F, -1.3464546E38F, 7.6872264E37F, -1.3756299E38F, -3.3557547E38F, 1.6857709E38F, -7.1872564E37F, 3.2484155E38F, 2.6475968E38F, 2.5489563E38F, -2.7857711E38F, -3.714629E36F, 1.3365825E38F, 8.251603E37F, -3.185235E38F, -3.0902746E38F, 2.7513249E38F, 2.1075123E38F, -1.4563202E38F, -2.1540769E38F, -2.4601369E37F, 3.1592997E38F, -1.6492659E38F, 1.0690399E38F, 3.4391024E37F, 1.1971618E38F, -3.3437347E38F, 2.8645482E38F}));
            assert(pack.time_usec_GET() == 957524292277559321L);
            assert(pack.vz_GET() == -3.1659762E38F);
            assert(pack.lon_GET() == 1305181863);
            assert(pack.relative_alt_GET() == -897361728);
            assert(pack.vx_GET() == 2.9232506E38F);
            assert(pack.vy_GET() == 1.2747547E38F);
            assert(pack.alt_GET() == 1628640602);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.lat_GET() == 163544781);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.lat_SET(163544781) ;
        p63.vx_SET(2.9232506E38F) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p63.time_usec_SET(957524292277559321L) ;
        p63.covariance_SET(new float[] {2.6211398E38F, 1.5700433E38F, 2.1213115E38F, 3.2997268E38F, 3.3810284E38F, 1.2001938E38F, -3.0574221E38F, -1.0991589E38F, -2.1700858E38F, -1.3464546E38F, 7.6872264E37F, -1.3756299E38F, -3.3557547E38F, 1.6857709E38F, -7.1872564E37F, 3.2484155E38F, 2.6475968E38F, 2.5489563E38F, -2.7857711E38F, -3.714629E36F, 1.3365825E38F, 8.251603E37F, -3.185235E38F, -3.0902746E38F, 2.7513249E38F, 2.1075123E38F, -1.4563202E38F, -2.1540769E38F, -2.4601369E37F, 3.1592997E38F, -1.6492659E38F, 1.0690399E38F, 3.4391024E37F, 1.1971618E38F, -3.3437347E38F, 2.8645482E38F}, 0) ;
        p63.vy_SET(1.2747547E38F) ;
        p63.relative_alt_SET(-897361728) ;
        p63.alt_SET(1628640602) ;
        p63.lon_SET(1305181863) ;
        p63.vz_SET(-3.1659762E38F) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 2.5649807E38F);
            assert(pack.time_usec_GET() == 5240889776826852948L);
            assert(pack.vz_GET() == -1.5280691E38F);
            assert(pack.x_GET() == -2.1191155E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {6.523335E37F, -1.3352566E38F, 1.6569163E38F, -1.9785775E38F, -1.9011392E38F, 1.8448983E38F, -2.9863398E38F, -3.8004882E37F, -3.3474933E38F, 1.4017678E38F, 3.1838231E38F, 3.3133466E38F, 3.2708884E38F, 2.489085E38F, 1.9247771E38F, 5.761843E36F, 9.053083E36F, -3.3857118E38F, -3.004993E38F, 8.444314E36F, 9.379955E37F, 1.5836949E38F, -1.1698255E37F, -1.9596064E37F, 1.0104562E38F, -2.0617104E38F, -1.991428E38F, 1.9519856E38F, -1.9731811E38F, 1.3802972E38F, 3.632361E37F, -4.1266555E37F, 9.898162E37F, -1.6551065E38F, -3.2314874E38F, -1.7364905E36F, 1.3269308E38F, 1.5211448E38F, -3.2821914E38F, 1.2879955E38F, 3.2228165E38F, -3.322568E38F, 2.5694624E37F, -7.2854917E37F, -9.364923E37F}));
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.vx_GET() == 3.2330133E38F);
            assert(pack.ay_GET() == 3.2234586E38F);
            assert(pack.z_GET() == -1.0089557E38F);
            assert(pack.y_GET() == -3.350631E38F);
            assert(pack.az_GET() == -2.1621183E38F);
            assert(pack.ax_GET() == -3.2158677E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.x_SET(-2.1191155E38F) ;
        p64.vz_SET(-1.5280691E38F) ;
        p64.ay_SET(3.2234586E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p64.az_SET(-2.1621183E38F) ;
        p64.vx_SET(3.2330133E38F) ;
        p64.covariance_SET(new float[] {6.523335E37F, -1.3352566E38F, 1.6569163E38F, -1.9785775E38F, -1.9011392E38F, 1.8448983E38F, -2.9863398E38F, -3.8004882E37F, -3.3474933E38F, 1.4017678E38F, 3.1838231E38F, 3.3133466E38F, 3.2708884E38F, 2.489085E38F, 1.9247771E38F, 5.761843E36F, 9.053083E36F, -3.3857118E38F, -3.004993E38F, 8.444314E36F, 9.379955E37F, 1.5836949E38F, -1.1698255E37F, -1.9596064E37F, 1.0104562E38F, -2.0617104E38F, -1.991428E38F, 1.9519856E38F, -1.9731811E38F, 1.3802972E38F, 3.632361E37F, -4.1266555E37F, 9.898162E37F, -1.6551065E38F, -3.2314874E38F, -1.7364905E36F, 1.3269308E38F, 1.5211448E38F, -3.2821914E38F, 1.2879955E38F, 3.2228165E38F, -3.322568E38F, 2.5694624E37F, -7.2854917E37F, -9.364923E37F}, 0) ;
        p64.y_SET(-3.350631E38F) ;
        p64.z_SET(-1.0089557E38F) ;
        p64.time_usec_SET(5240889776826852948L) ;
        p64.vy_SET(2.5649807E38F) ;
        p64.ax_SET(-3.2158677E38F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan18_raw_GET() == (char)46318);
            assert(pack.chan11_raw_GET() == (char)17088);
            assert(pack.chan13_raw_GET() == (char)7199);
            assert(pack.chan10_raw_GET() == (char)6515);
            assert(pack.chan2_raw_GET() == (char)63822);
            assert(pack.chan7_raw_GET() == (char)14366);
            assert(pack.chancount_GET() == (char)158);
            assert(pack.chan16_raw_GET() == (char)63109);
            assert(pack.chan9_raw_GET() == (char)20025);
            assert(pack.rssi_GET() == (char)50);
            assert(pack.chan3_raw_GET() == (char)37855);
            assert(pack.chan5_raw_GET() == (char)39290);
            assert(pack.chan1_raw_GET() == (char)57636);
            assert(pack.chan15_raw_GET() == (char)24418);
            assert(pack.chan8_raw_GET() == (char)15708);
            assert(pack.chan17_raw_GET() == (char)17893);
            assert(pack.time_boot_ms_GET() == 1216919681L);
            assert(pack.chan12_raw_GET() == (char)30088);
            assert(pack.chan6_raw_GET() == (char)9456);
            assert(pack.chan14_raw_GET() == (char)9791);
            assert(pack.chan4_raw_GET() == (char)55908);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan9_raw_SET((char)20025) ;
        p65.rssi_SET((char)50) ;
        p65.chan2_raw_SET((char)63822) ;
        p65.chan10_raw_SET((char)6515) ;
        p65.chan14_raw_SET((char)9791) ;
        p65.chan5_raw_SET((char)39290) ;
        p65.chan3_raw_SET((char)37855) ;
        p65.time_boot_ms_SET(1216919681L) ;
        p65.chan15_raw_SET((char)24418) ;
        p65.chan6_raw_SET((char)9456) ;
        p65.chan18_raw_SET((char)46318) ;
        p65.chan4_raw_SET((char)55908) ;
        p65.chan17_raw_SET((char)17893) ;
        p65.chan7_raw_SET((char)14366) ;
        p65.chan12_raw_SET((char)30088) ;
        p65.chancount_SET((char)158) ;
        p65.chan11_raw_SET((char)17088) ;
        p65.chan16_raw_SET((char)63109) ;
        p65.chan1_raw_SET((char)57636) ;
        p65.chan8_raw_SET((char)15708) ;
        p65.chan13_raw_SET((char)7199) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.req_message_rate_GET() == (char)39430);
            assert(pack.start_stop_GET() == (char)212);
            assert(pack.req_stream_id_GET() == (char)46);
            assert(pack.target_system_GET() == (char)139);
            assert(pack.target_component_GET() == (char)246);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_message_rate_SET((char)39430) ;
        p66.target_component_SET((char)246) ;
        p66.start_stop_SET((char)212) ;
        p66.target_system_SET((char)139) ;
        p66.req_stream_id_SET((char)46) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.stream_id_GET() == (char)255);
            assert(pack.message_rate_GET() == (char)38958);
            assert(pack.on_off_GET() == (char)182);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)38958) ;
        p67.stream_id_SET((char)255) ;
        p67.on_off_SET((char)182) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_GET() == (char)163);
            assert(pack.y_GET() == (short)19424);
            assert(pack.x_GET() == (short)13407);
            assert(pack.r_GET() == (short)18410);
            assert(pack.buttons_GET() == (char)30525);
            assert(pack.z_GET() == (short)30649);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.target_SET((char)163) ;
        p69.y_SET((short)19424) ;
        p69.z_SET((short)30649) ;
        p69.r_SET((short)18410) ;
        p69.buttons_SET((char)30525) ;
        p69.x_SET((short)13407) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan1_raw_GET() == (char)52590);
            assert(pack.chan4_raw_GET() == (char)60822);
            assert(pack.chan8_raw_GET() == (char)29855);
            assert(pack.target_component_GET() == (char)97);
            assert(pack.chan2_raw_GET() == (char)33315);
            assert(pack.chan5_raw_GET() == (char)29099);
            assert(pack.chan6_raw_GET() == (char)18791);
            assert(pack.chan7_raw_GET() == (char)54310);
            assert(pack.target_system_GET() == (char)168);
            assert(pack.chan3_raw_GET() == (char)59513);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan7_raw_SET((char)54310) ;
        p70.chan5_raw_SET((char)29099) ;
        p70.chan3_raw_SET((char)59513) ;
        p70.chan6_raw_SET((char)18791) ;
        p70.target_component_SET((char)97) ;
        p70.chan2_raw_SET((char)33315) ;
        p70.chan8_raw_SET((char)29855) ;
        p70.chan4_raw_SET((char)60822) ;
        p70.target_system_SET((char)168) ;
        p70.chan1_raw_SET((char)52590) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.seq_GET() == (char)5405);
            assert(pack.target_component_GET() == (char)235);
            assert(pack.param2_GET() == 1.3425601E38F);
            assert(pack.x_GET() == -1491408035);
            assert(pack.current_GET() == (char)98);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.y_GET() == 252726303);
            assert(pack.autocontinue_GET() == (char)244);
            assert(pack.target_system_GET() == (char)235);
            assert(pack.param4_GET() == 1.9170258E38F);
            assert(pack.param3_GET() == -3.3280617E38F);
            assert(pack.param1_GET() == -3.349858E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_MODE);
            assert(pack.z_GET() == 1.3905599E38F);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.autocontinue_SET((char)244) ;
        p73.param4_SET(1.9170258E38F) ;
        p73.param3_SET(-3.3280617E38F) ;
        p73.command_SET(MAV_CMD.MAV_CMD_DO_SET_MODE) ;
        p73.x_SET(-1491408035) ;
        p73.seq_SET((char)5405) ;
        p73.z_SET(1.3905599E38F) ;
        p73.param1_SET(-3.349858E37F) ;
        p73.param2_SET(1.3425601E38F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p73.target_system_SET((char)235) ;
        p73.y_SET(252726303) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p73.current_SET((char)98) ;
        p73.target_component_SET((char)235) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.throttle_GET() == (char)17613);
            assert(pack.groundspeed_GET() == -2.914935E37F);
            assert(pack.airspeed_GET() == 1.2655685E37F);
            assert(pack.climb_GET() == 3.339306E38F);
            assert(pack.alt_GET() == -2.9267775E38F);
            assert(pack.heading_GET() == (short) -16051);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.climb_SET(3.339306E38F) ;
        p74.heading_SET((short) -16051) ;
        p74.groundspeed_SET(-2.914935E37F) ;
        p74.alt_SET(-2.9267775E38F) ;
        p74.throttle_SET((char)17613) ;
        p74.airspeed_SET(1.2655685E37F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 390868307);
            assert(pack.z_GET() == 1.6776673E38F);
            assert(pack.y_GET() == 1263447959);
            assert(pack.autocontinue_GET() == (char)139);
            assert(pack.param1_GET() == -3.3874675E38F);
            assert(pack.param2_GET() == -2.5943516E38F);
            assert(pack.current_GET() == (char)79);
            assert(pack.target_system_GET() == (char)14);
            assert(pack.target_component_GET() == (char)14);
            assert(pack.param4_GET() == -3.3309684E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.param3_GET() == 8.3363523E37F);
        });
        GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p75.param1_SET(-3.3874675E38F) ;
        p75.command_SET(MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE) ;
        p75.param4_SET(-3.3309684E38F) ;
        p75.target_system_SET((char)14) ;
        p75.z_SET(1.6776673E38F) ;
        p75.x_SET(390868307) ;
        p75.autocontinue_SET((char)139) ;
        p75.param3_SET(8.3363523E37F) ;
        p75.y_SET(1263447959) ;
        p75.param2_SET(-2.5943516E38F) ;
        p75.current_SET((char)79) ;
        p75.target_component_SET((char)14) ;
        CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)98);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_SPATIAL_USER_2);
            assert(pack.param2_GET() == -3.3507305E38F);
            assert(pack.target_system_GET() == (char)29);
            assert(pack.param1_GET() == 7.147272E37F);
            assert(pack.param7_GET() == -1.1999771E38F);
            assert(pack.param6_GET() == 3.385698E38F);
            assert(pack.confirmation_GET() == (char)226);
            assert(pack.param4_GET() == 4.315485E37F);
            assert(pack.param3_GET() == 1.6731766E38F);
            assert(pack.param5_GET() == -1.8123605E38F);
        });
        GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param1_SET(7.147272E37F) ;
        p76.confirmation_SET((char)226) ;
        p76.command_SET(MAV_CMD.MAV_CMD_SPATIAL_USER_2) ;
        p76.param6_SET(3.385698E38F) ;
        p76.param3_SET(1.6731766E38F) ;
        p76.param5_SET(-1.8123605E38F) ;
        p76.target_component_SET((char)98) ;
        p76.param2_SET(-3.3507305E38F) ;
        p76.target_system_SET((char)29) ;
        p76.param4_SET(4.315485E37F) ;
        p76.param7_SET(-1.1999771E38F) ;
        CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.result_param2_TRY(ph) == 1058567443);
            assert(pack.progress_TRY(ph) == (char)28);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_DENIED);
            assert(pack.target_system_TRY(ph) == (char)89);
            assert(pack.target_component_TRY(ph) == (char)72);
        });
        GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.target_component_SET((char)72, PH) ;
        p77.target_system_SET((char)89, PH) ;
        p77.progress_SET((char)28, PH) ;
        p77.result_param2_SET(1058567443, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_DENIED) ;
        CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -4.4616053E37F);
            assert(pack.time_boot_ms_GET() == 1362087431L);
            assert(pack.yaw_GET() == 2.3166828E38F);
            assert(pack.thrust_GET() == 1.8398951E37F);
            assert(pack.manual_override_switch_GET() == (char)184);
            assert(pack.mode_switch_GET() == (char)228);
            assert(pack.pitch_GET() == -2.173046E38F);
        });
        GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.pitch_SET(-2.173046E38F) ;
        p81.time_boot_ms_SET(1362087431L) ;
        p81.yaw_SET(2.3166828E38F) ;
        p81.mode_switch_SET((char)228) ;
        p81.thrust_SET(1.8398951E37F) ;
        p81.manual_override_switch_SET((char)184) ;
        p81.roll_SET(-4.4616053E37F) ;
        CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.thrust_GET() == -2.8996534E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.0769064E38F, -1.5866152E37F, -1.6493345E38F, 2.2760064E38F}));
            assert(pack.body_pitch_rate_GET() == -1.312284E38F);
            assert(pack.time_boot_ms_GET() == 887175579L);
            assert(pack.target_system_GET() == (char)102);
            assert(pack.body_yaw_rate_GET() == -2.51549E38F);
            assert(pack.type_mask_GET() == (char)87);
            assert(pack.body_roll_rate_GET() == 2.5144785E38F);
            assert(pack.target_component_GET() == (char)224);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.type_mask_SET((char)87) ;
        p82.body_roll_rate_SET(2.5144785E38F) ;
        p82.body_pitch_rate_SET(-1.312284E38F) ;
        p82.target_component_SET((char)224) ;
        p82.time_boot_ms_SET(887175579L) ;
        p82.target_system_SET((char)102) ;
        p82.body_yaw_rate_SET(-2.51549E38F) ;
        p82.q_SET(new float[] {-2.0769064E38F, -1.5866152E37F, -1.6493345E38F, 2.2760064E38F}, 0) ;
        p82.thrust_SET(-2.8996534E38F) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.thrust_GET() == -2.2221905E38F);
            assert(pack.body_yaw_rate_GET() == 2.2967725E37F);
            assert(pack.type_mask_GET() == (char)202);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.0590193E38F, 2.7573376E38F, -8.758764E37F, 1.7333903E38F}));
            assert(pack.body_roll_rate_GET() == 2.731568E38F);
            assert(pack.time_boot_ms_GET() == 3160179539L);
            assert(pack.body_pitch_rate_GET() == -3.0209598E38F);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.type_mask_SET((char)202) ;
        p83.thrust_SET(-2.2221905E38F) ;
        p83.time_boot_ms_SET(3160179539L) ;
        p83.body_pitch_rate_SET(-3.0209598E38F) ;
        p83.body_roll_rate_SET(2.731568E38F) ;
        p83.q_SET(new float[] {-1.0590193E38F, 2.7573376E38F, -8.758764E37F, 1.7333903E38F}, 0) ;
        p83.body_yaw_rate_SET(2.2967725E37F) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)52676);
            assert(pack.y_GET() == 1.4167484E38F);
            assert(pack.target_component_GET() == (char)118);
            assert(pack.afx_GET() == -3.373375E35F);
            assert(pack.yaw_rate_GET() == 7.117819E37F);
            assert(pack.vz_GET() == 2.0663374E38F);
            assert(pack.afy_GET() == 1.0781944E38F);
            assert(pack.yaw_GET() == -1.1279185E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.afz_GET() == -9.706624E37F);
            assert(pack.vy_GET() == -1.725011E38F);
            assert(pack.vx_GET() == 2.69788E38F);
            assert(pack.x_GET() == 3.0257936E38F);
            assert(pack.target_system_GET() == (char)51);
            assert(pack.time_boot_ms_GET() == 845234483L);
            assert(pack.z_GET() == -6.3292887E37F);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.y_SET(1.4167484E38F) ;
        p84.afz_SET(-9.706624E37F) ;
        p84.vx_SET(2.69788E38F) ;
        p84.x_SET(3.0257936E38F) ;
        p84.vy_SET(-1.725011E38F) ;
        p84.target_component_SET((char)118) ;
        p84.yaw_SET(-1.1279185E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p84.z_SET(-6.3292887E37F) ;
        p84.target_system_SET((char)51) ;
        p84.afx_SET(-3.373375E35F) ;
        p84.time_boot_ms_SET(845234483L) ;
        p84.yaw_rate_SET(7.117819E37F) ;
        p84.vz_SET(2.0663374E38F) ;
        p84.afy_SET(1.0781944E38F) ;
        p84.type_mask_SET((char)52676) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)38107);
            assert(pack.target_system_GET() == (char)12);
            assert(pack.vz_GET() == -5.3369257E37F);
            assert(pack.vy_GET() == 1.9637021E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.vx_GET() == 1.893054E38F);
            assert(pack.afz_GET() == 1.8574059E38F);
            assert(pack.target_component_GET() == (char)142);
            assert(pack.time_boot_ms_GET() == 3108445752L);
            assert(pack.alt_GET() == -1.4414535E37F);
            assert(pack.lat_int_GET() == -1201758474);
            assert(pack.yaw_rate_GET() == -3.9645413E37F);
            assert(pack.afx_GET() == -1.9989597E38F);
            assert(pack.lon_int_GET() == 1127833455);
            assert(pack.yaw_GET() == -1.5153982E38F);
            assert(pack.afy_GET() == 2.8289416E38F);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.vy_SET(1.9637021E38F) ;
        p86.lat_int_SET(-1201758474) ;
        p86.target_component_SET((char)142) ;
        p86.afx_SET(-1.9989597E38F) ;
        p86.afz_SET(1.8574059E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p86.vx_SET(1.893054E38F) ;
        p86.vz_SET(-5.3369257E37F) ;
        p86.target_system_SET((char)12) ;
        p86.type_mask_SET((char)38107) ;
        p86.lon_int_SET(1127833455) ;
        p86.time_boot_ms_SET(3108445752L) ;
        p86.afy_SET(2.8289416E38F) ;
        p86.yaw_SET(-1.5153982E38F) ;
        p86.alt_SET(-1.4414535E37F) ;
        p86.yaw_rate_SET(-3.9645413E37F) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)48123);
            assert(pack.yaw_rate_GET() == 3.1611472E38F);
            assert(pack.lat_int_GET() == -1352686522);
            assert(pack.alt_GET() == 1.0757558E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.lon_int_GET() == -1386840506);
            assert(pack.vx_GET() == 2.4810565E38F);
            assert(pack.afx_GET() == 7.111384E37F);
            assert(pack.time_boot_ms_GET() == 3981778045L);
            assert(pack.vz_GET() == -3.2195344E38F);
            assert(pack.afz_GET() == 1.0095456E38F);
            assert(pack.vy_GET() == 7.152187E37F);
            assert(pack.yaw_GET() == 1.0089553E38F);
            assert(pack.afy_GET() == 1.8890322E38F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.lat_int_SET(-1352686522) ;
        p87.vz_SET(-3.2195344E38F) ;
        p87.lon_int_SET(-1386840506) ;
        p87.afy_SET(1.8890322E38F) ;
        p87.vy_SET(7.152187E37F) ;
        p87.yaw_rate_SET(3.1611472E38F) ;
        p87.afx_SET(7.111384E37F) ;
        p87.alt_SET(1.0757558E37F) ;
        p87.time_boot_ms_SET(3981778045L) ;
        p87.vx_SET(2.4810565E38F) ;
        p87.afz_SET(1.0095456E38F) ;
        p87.yaw_SET(1.0089553E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p87.type_mask_SET((char)48123) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 2.1506737E38F);
            assert(pack.pitch_GET() == 2.017819E38F);
            assert(pack.yaw_GET() == -1.9878108E38F);
            assert(pack.roll_GET() == 8.3862896E37F);
            assert(pack.z_GET() == 1.6339399E38F);
            assert(pack.time_boot_ms_GET() == 1350481056L);
            assert(pack.x_GET() == -7.6573286E37F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.roll_SET(8.3862896E37F) ;
        p89.z_SET(1.6339399E38F) ;
        p89.time_boot_ms_SET(1350481056L) ;
        p89.pitch_SET(2.017819E38F) ;
        p89.y_SET(2.1506737E38F) ;
        p89.x_SET(-7.6573286E37F) ;
        p89.yaw_SET(-1.9878108E38F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.pitchspeed_GET() == 2.3881448E38F);
            assert(pack.vy_GET() == (short) -24802);
            assert(pack.lat_GET() == 804473607);
            assert(pack.rollspeed_GET() == -1.0301443E38F);
            assert(pack.xacc_GET() == (short)29823);
            assert(pack.roll_GET() == 3.2065786E38F);
            assert(pack.vx_GET() == (short)9527);
            assert(pack.time_usec_GET() == 5166718628646390187L);
            assert(pack.yaw_GET() == 2.196475E38F);
            assert(pack.vz_GET() == (short)1386);
            assert(pack.yacc_GET() == (short)3530);
            assert(pack.pitch_GET() == 1.5721131E38F);
            assert(pack.zacc_GET() == (short)31823);
            assert(pack.yawspeed_GET() == -2.26005E37F);
            assert(pack.lon_GET() == -998405440);
            assert(pack.alt_GET() == -1121006232);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.yaw_SET(2.196475E38F) ;
        p90.vz_SET((short)1386) ;
        p90.yacc_SET((short)3530) ;
        p90.pitch_SET(1.5721131E38F) ;
        p90.vy_SET((short) -24802) ;
        p90.lon_SET(-998405440) ;
        p90.yawspeed_SET(-2.26005E37F) ;
        p90.roll_SET(3.2065786E38F) ;
        p90.time_usec_SET(5166718628646390187L) ;
        p90.xacc_SET((short)29823) ;
        p90.zacc_SET((short)31823) ;
        p90.alt_SET(-1121006232) ;
        p90.pitchspeed_SET(2.3881448E38F) ;
        p90.lat_SET(804473607) ;
        p90.rollspeed_SET(-1.0301443E38F) ;
        p90.vx_SET((short)9527) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux4_GET() == -2.3497428E37F);
            assert(pack.aux1_GET() == 2.3147442E38F);
            assert(pack.nav_mode_GET() == (char)137);
            assert(pack.throttle_GET() == -5.8762167E37F);
            assert(pack.time_usec_GET() == 5387059881582921979L);
            assert(pack.yaw_rudder_GET() == -1.0333128E38F);
            assert(pack.aux3_GET() == 1.0774311E37F);
            assert(pack.roll_ailerons_GET() == 1.5746087E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
            assert(pack.pitch_elevator_GET() == 3.3565638E38F);
            assert(pack.aux2_GET() == 1.9516718E38F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.throttle_SET(-5.8762167E37F) ;
        p91.aux3_SET(1.0774311E37F) ;
        p91.pitch_elevator_SET(3.3565638E38F) ;
        p91.aux4_SET(-2.3497428E37F) ;
        p91.yaw_rudder_SET(-1.0333128E38F) ;
        p91.roll_ailerons_SET(1.5746087E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_STABILIZE_DISARMED) ;
        p91.aux1_SET(2.3147442E38F) ;
        p91.time_usec_SET(5387059881582921979L) ;
        p91.nav_mode_SET((char)137) ;
        p91.aux2_SET(1.9516718E38F) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan8_raw_GET() == (char)11719);
            assert(pack.chan12_raw_GET() == (char)47020);
            assert(pack.chan9_raw_GET() == (char)52632);
            assert(pack.chan3_raw_GET() == (char)7531);
            assert(pack.chan7_raw_GET() == (char)19585);
            assert(pack.chan5_raw_GET() == (char)1825);
            assert(pack.chan6_raw_GET() == (char)16110);
            assert(pack.chan11_raw_GET() == (char)29122);
            assert(pack.chan10_raw_GET() == (char)60897);
            assert(pack.chan2_raw_GET() == (char)46689);
            assert(pack.rssi_GET() == (char)243);
            assert(pack.chan1_raw_GET() == (char)64827);
            assert(pack.time_usec_GET() == 7930316737929663333L);
            assert(pack.chan4_raw_GET() == (char)49659);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(7930316737929663333L) ;
        p92.chan7_raw_SET((char)19585) ;
        p92.rssi_SET((char)243) ;
        p92.chan1_raw_SET((char)64827) ;
        p92.chan4_raw_SET((char)49659) ;
        p92.chan9_raw_SET((char)52632) ;
        p92.chan8_raw_SET((char)11719) ;
        p92.chan12_raw_SET((char)47020) ;
        p92.chan5_raw_SET((char)1825) ;
        p92.chan2_raw_SET((char)46689) ;
        p92.chan11_raw_SET((char)29122) ;
        p92.chan3_raw_SET((char)7531) ;
        p92.chan10_raw_SET((char)60897) ;
        p92.chan6_raw_SET((char)16110) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6335090410353046593L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
            assert(pack.flags_GET() == 6114641339831364597L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {3.1100662E38F, -2.586012E38F, 2.6397637E38F, -7.2931767E37F, 4.798222E37F, 4.104375E37F, -2.4266382E38F, -3.2075312E38F, -2.330258E38F, -7.1615804E37F, 2.9304545E38F, -3.938557E37F, 3.3446639E38F, -2.4762482E38F, 3.1347918E38F, -1.5066287E38F}));
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.flags_SET(6114641339831364597L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_STABILIZE_DISARMED) ;
        p93.controls_SET(new float[] {3.1100662E38F, -2.586012E38F, 2.6397637E38F, -7.2931767E37F, 4.798222E37F, 4.104375E37F, -2.4266382E38F, -3.2075312E38F, -2.330258E38F, -7.1615804E37F, 2.9304545E38F, -3.938557E37F, 3.3446639E38F, -2.4762482E38F, 3.1347918E38F, -1.5066287E38F}, 0) ;
        p93.time_usec_SET(6335090410353046593L) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6670881792742937902L);
            assert(pack.ground_distance_GET() == 3.0508403E38F);
            assert(pack.quality_GET() == (char)248);
            assert(pack.flow_x_GET() == (short) -25434);
            assert(pack.flow_rate_y_TRY(ph) == 1.0332298E38F);
            assert(pack.sensor_id_GET() == (char)17);
            assert(pack.flow_comp_m_y_GET() == 1.4524101E38F);
            assert(pack.flow_y_GET() == (short)23501);
            assert(pack.flow_rate_x_TRY(ph) == -2.757152E38F);
            assert(pack.flow_comp_m_x_GET() == 5.0373915E37F);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_rate_y_SET(1.0332298E38F, PH) ;
        p100.sensor_id_SET((char)17) ;
        p100.ground_distance_SET(3.0508403E38F) ;
        p100.flow_comp_m_y_SET(1.4524101E38F) ;
        p100.quality_SET((char)248) ;
        p100.time_usec_SET(6670881792742937902L) ;
        p100.flow_y_SET((short)23501) ;
        p100.flow_rate_x_SET(-2.757152E38F, PH) ;
        p100.flow_x_SET((short) -25434) ;
        p100.flow_comp_m_x_SET(5.0373915E37F) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -2.3760088E38F);
            assert(pack.pitch_GET() == 1.0183082E38F);
            assert(pack.usec_GET() == 2985616735202780415L);
            assert(pack.roll_GET() == 2.0788829E38F);
            assert(pack.z_GET() == 3.2763797E38F);
            assert(pack.yaw_GET() == -2.8156658E38F);
            assert(pack.y_GET() == -1.0713672E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.roll_SET(2.0788829E38F) ;
        p101.x_SET(-2.3760088E38F) ;
        p101.usec_SET(2985616735202780415L) ;
        p101.pitch_SET(1.0183082E38F) ;
        p101.yaw_SET(-2.8156658E38F) ;
        p101.y_SET(-1.0713672E38F) ;
        p101.z_SET(3.2763797E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 2.2855604E38F);
            assert(pack.roll_GET() == -1.1599982E37F);
            assert(pack.pitch_GET() == -9.650929E37F);
            assert(pack.yaw_GET() == 5.17617E37F);
            assert(pack.z_GET() == -1.0508404E38F);
            assert(pack.usec_GET() == 3450658990129298226L);
            assert(pack.x_GET() == -2.7258972E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.y_SET(2.2855604E38F) ;
        p102.roll_SET(-1.1599982E37F) ;
        p102.yaw_SET(5.17617E37F) ;
        p102.x_SET(-2.7258972E38F) ;
        p102.pitch_SET(-9.650929E37F) ;
        p102.z_SET(-1.0508404E38F) ;
        p102.usec_SET(3450658990129298226L) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -3.0980525E38F);
            assert(pack.x_GET() == -8.59735E37F);
            assert(pack.z_GET() == -7.263179E37F);
            assert(pack.usec_GET() == 4505786428281321836L);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(4505786428281321836L) ;
        p103.z_SET(-7.263179E37F) ;
        p103.x_SET(-8.59735E37F) ;
        p103.y_SET(-3.0980525E38F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 2.8046293E37F);
            assert(pack.usec_GET() == 2483180418578132701L);
            assert(pack.z_GET() == 1.2419794E38F);
            assert(pack.y_GET() == 1.3637184E38F);
            assert(pack.roll_GET() == -2.9580797E38F);
            assert(pack.x_GET() == 1.641633E37F);
            assert(pack.pitch_GET() == -1.9500732E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(2483180418578132701L) ;
        p104.x_SET(1.641633E37F) ;
        p104.roll_SET(-2.9580797E38F) ;
        p104.yaw_SET(2.8046293E37F) ;
        p104.z_SET(1.2419794E38F) ;
        p104.y_SET(1.3637184E38F) ;
        p104.pitch_SET(-1.9500732E38F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == -1.5274704E38F);
            assert(pack.xmag_GET() == 2.3943616E38F);
            assert(pack.ygyro_GET() == 1.064445E37F);
            assert(pack.zmag_GET() == -8.699472E37F);
            assert(pack.abs_pressure_GET() == -6.4128333E36F);
            assert(pack.fields_updated_GET() == (char)31348);
            assert(pack.yacc_GET() == 6.02272E37F);
            assert(pack.zgyro_GET() == -2.4732163E38F);
            assert(pack.xacc_GET() == 1.7980537E38F);
            assert(pack.pressure_alt_GET() == -2.592647E38F);
            assert(pack.diff_pressure_GET() == -3.3211914E38F);
            assert(pack.xgyro_GET() == -2.3727375E38F);
            assert(pack.zacc_GET() == -2.23203E38F);
            assert(pack.ymag_GET() == -2.6452408E38F);
            assert(pack.time_usec_GET() == 714932793246784271L);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.xmag_SET(2.3943616E38F) ;
        p105.yacc_SET(6.02272E37F) ;
        p105.abs_pressure_SET(-6.4128333E36F) ;
        p105.zmag_SET(-8.699472E37F) ;
        p105.time_usec_SET(714932793246784271L) ;
        p105.fields_updated_SET((char)31348) ;
        p105.zacc_SET(-2.23203E38F) ;
        p105.zgyro_SET(-2.4732163E38F) ;
        p105.temperature_SET(-1.5274704E38F) ;
        p105.xgyro_SET(-2.3727375E38F) ;
        p105.diff_pressure_SET(-3.3211914E38F) ;
        p105.xacc_SET(1.7980537E38F) ;
        p105.ygyro_SET(1.064445E37F) ;
        p105.ymag_SET(-2.6452408E38F) ;
        p105.pressure_alt_SET(-2.592647E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.time_delta_distance_us_GET() == 3634813301L);
            assert(pack.integrated_xgyro_GET() == 9.076696E37F);
            assert(pack.sensor_id_GET() == (char)94);
            assert(pack.integrated_x_GET() == -2.114186E38F);
            assert(pack.distance_GET() == 2.5448921E38F);
            assert(pack.temperature_GET() == (short)6747);
            assert(pack.time_usec_GET() == 7311186143256867578L);
            assert(pack.integrated_ygyro_GET() == -3.2198952E38F);
            assert(pack.integration_time_us_GET() == 520352652L);
            assert(pack.integrated_zgyro_GET() == -1.4855615E38F);
            assert(pack.integrated_y_GET() == 1.6225583E38F);
            assert(pack.quality_GET() == (char)241);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_x_SET(-2.114186E38F) ;
        p106.time_usec_SET(7311186143256867578L) ;
        p106.distance_SET(2.5448921E38F) ;
        p106.integrated_zgyro_SET(-1.4855615E38F) ;
        p106.integrated_y_SET(1.6225583E38F) ;
        p106.integration_time_us_SET(520352652L) ;
        p106.integrated_xgyro_SET(9.076696E37F) ;
        p106.temperature_SET((short)6747) ;
        p106.time_delta_distance_us_SET(3634813301L) ;
        p106.sensor_id_SET((char)94) ;
        p106.integrated_ygyro_SET(-3.2198952E38F) ;
        p106.quality_SET((char)241) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.diff_pressure_GET() == -9.459358E37F);
            assert(pack.zacc_GET() == -2.1397425E38F);
            assert(pack.xacc_GET() == 8.979202E37F);
            assert(pack.yacc_GET() == 4.2530583E37F);
            assert(pack.pressure_alt_GET() == 2.980322E38F);
            assert(pack.zgyro_GET() == -3.3033628E38F);
            assert(pack.xmag_GET() == -2.3620693E38F);
            assert(pack.zmag_GET() == 7.088833E37F);
            assert(pack.fields_updated_GET() == 3580328299L);
            assert(pack.abs_pressure_GET() == -2.0768934E38F);
            assert(pack.time_usec_GET() == 3748324109351031606L);
            assert(pack.ymag_GET() == -4.1541653E37F);
            assert(pack.xgyro_GET() == 2.708564E38F);
            assert(pack.temperature_GET() == -2.9806528E38F);
            assert(pack.ygyro_GET() == 3.066035E36F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.temperature_SET(-2.9806528E38F) ;
        p107.xmag_SET(-2.3620693E38F) ;
        p107.abs_pressure_SET(-2.0768934E38F) ;
        p107.xgyro_SET(2.708564E38F) ;
        p107.diff_pressure_SET(-9.459358E37F) ;
        p107.ygyro_SET(3.066035E36F) ;
        p107.time_usec_SET(3748324109351031606L) ;
        p107.zmag_SET(7.088833E37F) ;
        p107.pressure_alt_SET(2.980322E38F) ;
        p107.zacc_SET(-2.1397425E38F) ;
        p107.ymag_SET(-4.1541653E37F) ;
        p107.yacc_SET(4.2530583E37F) ;
        p107.fields_updated_SET(3580328299L) ;
        p107.zgyro_SET(-3.3033628E38F) ;
        p107.xacc_SET(8.979202E37F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == 1.016762E38F);
            assert(pack.ve_GET() == -3.0013102E38F);
            assert(pack.ygyro_GET() == -1.0650738E38F);
            assert(pack.std_dev_horz_GET() == -1.770569E37F);
            assert(pack.yacc_GET() == 1.1778354E38F);
            assert(pack.xgyro_GET() == 4.6449507E37F);
            assert(pack.pitch_GET() == -6.067122E37F);
            assert(pack.q2_GET() == -8.813807E37F);
            assert(pack.zacc_GET() == 5.0252296E37F);
            assert(pack.vd_GET() == 9.989892E37F);
            assert(pack.q3_GET() == -9.229295E37F);
            assert(pack.alt_GET() == -3.115173E38F);
            assert(pack.q4_GET() == 4.4803214E37F);
            assert(pack.yaw_GET() == -3.305226E38F);
            assert(pack.roll_GET() == -1.5651422E38F);
            assert(pack.lon_GET() == -3.0304263E38F);
            assert(pack.vn_GET() == 3.0331808E38F);
            assert(pack.lat_GET() == -8.295166E37F);
            assert(pack.std_dev_vert_GET() == -2.5458495E38F);
            assert(pack.q1_GET() == -3.3384086E38F);
            assert(pack.zgyro_GET() == 1.6593694E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.lon_SET(-3.0304263E38F) ;
        p108.q4_SET(4.4803214E37F) ;
        p108.yaw_SET(-3.305226E38F) ;
        p108.zgyro_SET(1.6593694E38F) ;
        p108.xacc_SET(1.016762E38F) ;
        p108.std_dev_vert_SET(-2.5458495E38F) ;
        p108.lat_SET(-8.295166E37F) ;
        p108.vd_SET(9.989892E37F) ;
        p108.q3_SET(-9.229295E37F) ;
        p108.zacc_SET(5.0252296E37F) ;
        p108.vn_SET(3.0331808E38F) ;
        p108.yacc_SET(1.1778354E38F) ;
        p108.pitch_SET(-6.067122E37F) ;
        p108.std_dev_horz_SET(-1.770569E37F) ;
        p108.ve_SET(-3.0013102E38F) ;
        p108.q2_SET(-8.813807E37F) ;
        p108.roll_SET(-1.5651422E38F) ;
        p108.alt_SET(-3.115173E38F) ;
        p108.xgyro_SET(4.6449507E37F) ;
        p108.ygyro_SET(-1.0650738E38F) ;
        p108.q1_SET(-3.3384086E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.noise_GET() == (char)243);
            assert(pack.remnoise_GET() == (char)194);
            assert(pack.rxerrors_GET() == (char)30105);
            assert(pack.fixed__GET() == (char)55534);
            assert(pack.remrssi_GET() == (char)214);
            assert(pack.txbuf_GET() == (char)223);
            assert(pack.rssi_GET() == (char)197);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.fixed__SET((char)55534) ;
        p109.remnoise_SET((char)194) ;
        p109.txbuf_SET((char)223) ;
        p109.noise_SET((char)243) ;
        p109.rssi_SET((char)197) ;
        p109.rxerrors_SET((char)30105) ;
        p109.remrssi_SET((char)214) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)219);
            assert(pack.target_component_GET() == (char)183);
            assert(pack.target_system_GET() == (char)20);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)35, (char)225, (char)127, (char)241, (char)34, (char)47, (char)84, (char)184, (char)206, (char)170, (char)59, (char)238, (char)129, (char)72, (char)9, (char)4, (char)111, (char)104, (char)25, (char)117, (char)213, (char)64, (char)226, (char)56, (char)141, (char)143, (char)18, (char)6, (char)170, (char)125, (char)114, (char)19, (char)24, (char)16, (char)135, (char)185, (char)237, (char)149, (char)58, (char)191, (char)239, (char)217, (char)58, (char)45, (char)83, (char)193, (char)207, (char)207, (char)119, (char)252, (char)56, (char)252, (char)66, (char)245, (char)1, (char)249, (char)31, (char)132, (char)137, (char)99, (char)40, (char)198, (char)196, (char)114, (char)218, (char)252, (char)6, (char)249, (char)243, (char)205, (char)222, (char)86, (char)241, (char)0, (char)104, (char)83, (char)114, (char)39, (char)0, (char)162, (char)180, (char)39, (char)165, (char)137, (char)15, (char)19, (char)94, (char)142, (char)146, (char)153, (char)224, (char)6, (char)79, (char)166, (char)226, (char)168, (char)8, (char)224, (char)42, (char)96, (char)220, (char)203, (char)84, (char)176, (char)249, (char)129, (char)244, (char)116, (char)48, (char)246, (char)233, (char)240, (char)89, (char)236, (char)42, (char)206, (char)164, (char)199, (char)45, (char)136, (char)24, (char)225, (char)32, (char)37, (char)182, (char)179, (char)18, (char)202, (char)159, (char)165, (char)131, (char)107, (char)86, (char)192, (char)13, (char)33, (char)212, (char)140, (char)44, (char)162, (char)15, (char)189, (char)56, (char)236, (char)189, (char)109, (char)177, (char)65, (char)223, (char)80, (char)174, (char)162, (char)15, (char)74, (char)53, (char)174, (char)110, (char)129, (char)195, (char)38, (char)24, (char)36, (char)94, (char)40, (char)39, (char)77, (char)240, (char)24, (char)220, (char)67, (char)138, (char)166, (char)152, (char)30, (char)149, (char)5, (char)139, (char)29, (char)126, (char)138, (char)159, (char)176, (char)107, (char)4, (char)247, (char)126, (char)37, (char)174, (char)208, (char)244, (char)143, (char)40, (char)205, (char)201, (char)81, (char)122, (char)181, (char)62, (char)129, (char)76, (char)8, (char)168, (char)58, (char)95, (char)146, (char)216, (char)194, (char)10, (char)255, (char)223, (char)238, (char)36, (char)146, (char)12, (char)174, (char)65, (char)254, (char)132, (char)36, (char)23, (char)129, (char)81, (char)49, (char)135, (char)95, (char)44, (char)118, (char)97, (char)246, (char)79, (char)71, (char)41, (char)71, (char)117, (char)191, (char)151, (char)101, (char)201, (char)175, (char)127, (char)76, (char)190, (char)242, (char)62, (char)62, (char)198, (char)125, (char)157, (char)137, (char)52, (char)6}));
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_component_SET((char)183) ;
        p110.target_network_SET((char)219) ;
        p110.target_system_SET((char)20) ;
        p110.payload_SET(new char[] {(char)35, (char)225, (char)127, (char)241, (char)34, (char)47, (char)84, (char)184, (char)206, (char)170, (char)59, (char)238, (char)129, (char)72, (char)9, (char)4, (char)111, (char)104, (char)25, (char)117, (char)213, (char)64, (char)226, (char)56, (char)141, (char)143, (char)18, (char)6, (char)170, (char)125, (char)114, (char)19, (char)24, (char)16, (char)135, (char)185, (char)237, (char)149, (char)58, (char)191, (char)239, (char)217, (char)58, (char)45, (char)83, (char)193, (char)207, (char)207, (char)119, (char)252, (char)56, (char)252, (char)66, (char)245, (char)1, (char)249, (char)31, (char)132, (char)137, (char)99, (char)40, (char)198, (char)196, (char)114, (char)218, (char)252, (char)6, (char)249, (char)243, (char)205, (char)222, (char)86, (char)241, (char)0, (char)104, (char)83, (char)114, (char)39, (char)0, (char)162, (char)180, (char)39, (char)165, (char)137, (char)15, (char)19, (char)94, (char)142, (char)146, (char)153, (char)224, (char)6, (char)79, (char)166, (char)226, (char)168, (char)8, (char)224, (char)42, (char)96, (char)220, (char)203, (char)84, (char)176, (char)249, (char)129, (char)244, (char)116, (char)48, (char)246, (char)233, (char)240, (char)89, (char)236, (char)42, (char)206, (char)164, (char)199, (char)45, (char)136, (char)24, (char)225, (char)32, (char)37, (char)182, (char)179, (char)18, (char)202, (char)159, (char)165, (char)131, (char)107, (char)86, (char)192, (char)13, (char)33, (char)212, (char)140, (char)44, (char)162, (char)15, (char)189, (char)56, (char)236, (char)189, (char)109, (char)177, (char)65, (char)223, (char)80, (char)174, (char)162, (char)15, (char)74, (char)53, (char)174, (char)110, (char)129, (char)195, (char)38, (char)24, (char)36, (char)94, (char)40, (char)39, (char)77, (char)240, (char)24, (char)220, (char)67, (char)138, (char)166, (char)152, (char)30, (char)149, (char)5, (char)139, (char)29, (char)126, (char)138, (char)159, (char)176, (char)107, (char)4, (char)247, (char)126, (char)37, (char)174, (char)208, (char)244, (char)143, (char)40, (char)205, (char)201, (char)81, (char)122, (char)181, (char)62, (char)129, (char)76, (char)8, (char)168, (char)58, (char)95, (char)146, (char)216, (char)194, (char)10, (char)255, (char)223, (char)238, (char)36, (char)146, (char)12, (char)174, (char)65, (char)254, (char)132, (char)36, (char)23, (char)129, (char)81, (char)49, (char)135, (char)95, (char)44, (char)118, (char)97, (char)246, (char)79, (char)71, (char)41, (char)71, (char)117, (char)191, (char)151, (char)101, (char)201, (char)175, (char)127, (char)76, (char)190, (char)242, (char)62, (char)62, (char)198, (char)125, (char)157, (char)137, (char)52, (char)6}, 0) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == 4690180401728560248L);
            assert(pack.tc1_GET() == 1489054476548835179L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(1489054476548835179L) ;
        p111.ts1_SET(4690180401728560248L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2121473070113818866L);
            assert(pack.seq_GET() == 2686036313L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(2686036313L) ;
        p112.time_usec_SET(2121473070113818866L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.vel_GET() == (char)58018);
            assert(pack.satellites_visible_GET() == (char)0);
            assert(pack.epv_GET() == (char)46730);
            assert(pack.alt_GET() == 675718484);
            assert(pack.cog_GET() == (char)40063);
            assert(pack.lon_GET() == 994092115);
            assert(pack.ve_GET() == (short)14396);
            assert(pack.eph_GET() == (char)65281);
            assert(pack.vn_GET() == (short) -8003);
            assert(pack.fix_type_GET() == (char)114);
            assert(pack.time_usec_GET() == 2012992551268204939L);
            assert(pack.vd_GET() == (short) -17925);
            assert(pack.lat_GET() == 1131089183);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(2012992551268204939L) ;
        p113.fix_type_SET((char)114) ;
        p113.ve_SET((short)14396) ;
        p113.vel_SET((char)58018) ;
        p113.lat_SET(1131089183) ;
        p113.eph_SET((char)65281) ;
        p113.lon_SET(994092115) ;
        p113.epv_SET((char)46730) ;
        p113.satellites_visible_SET((char)0) ;
        p113.vn_SET((short) -8003) ;
        p113.cog_SET((char)40063) ;
        p113.alt_SET(675718484) ;
        p113.vd_SET((short) -17925) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_zgyro_GET() == 2.385486E38F);
            assert(pack.integrated_x_GET() == -5.7999736E37F);
            assert(pack.quality_GET() == (char)213);
            assert(pack.time_delta_distance_us_GET() == 3321054981L);
            assert(pack.time_usec_GET() == 8938912448663850845L);
            assert(pack.sensor_id_GET() == (char)245);
            assert(pack.integrated_xgyro_GET() == 9.338393E37F);
            assert(pack.temperature_GET() == (short) -11048);
            assert(pack.integration_time_us_GET() == 3870979623L);
            assert(pack.integrated_ygyro_GET() == 2.0539657E38F);
            assert(pack.distance_GET() == -2.3707214E38F);
            assert(pack.integrated_y_GET() == -2.2837912E38F);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_delta_distance_us_SET(3321054981L) ;
        p114.distance_SET(-2.3707214E38F) ;
        p114.integration_time_us_SET(3870979623L) ;
        p114.sensor_id_SET((char)245) ;
        p114.quality_SET((char)213) ;
        p114.integrated_x_SET(-5.7999736E37F) ;
        p114.time_usec_SET(8938912448663850845L) ;
        p114.integrated_xgyro_SET(9.338393E37F) ;
        p114.integrated_y_SET(-2.2837912E38F) ;
        p114.temperature_SET((short) -11048) ;
        p114.integrated_ygyro_SET(2.0539657E38F) ;
        p114.integrated_zgyro_SET(2.385486E38F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {2.4994723E38F, -1.937645E38F, 2.1817847E38F, 3.9693745E37F}));
            assert(pack.yawspeed_GET() == -1.2767833E37F);
            assert(pack.true_airspeed_GET() == (char)32673);
            assert(pack.xacc_GET() == (short) -16533);
            assert(pack.lon_GET() == -2099091391);
            assert(pack.lat_GET() == -710720322);
            assert(pack.zacc_GET() == (short)11765);
            assert(pack.ind_airspeed_GET() == (char)19256);
            assert(pack.yacc_GET() == (short) -15278);
            assert(pack.vx_GET() == (short)681);
            assert(pack.vy_GET() == (short)5536);
            assert(pack.time_usec_GET() == 7055398531856911820L);
            assert(pack.vz_GET() == (short)14557);
            assert(pack.alt_GET() == -1552754684);
            assert(pack.rollspeed_GET() == 9.932523E37F);
            assert(pack.pitchspeed_GET() == -9.473615E37F);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.pitchspeed_SET(-9.473615E37F) ;
        p115.lat_SET(-710720322) ;
        p115.alt_SET(-1552754684) ;
        p115.lon_SET(-2099091391) ;
        p115.true_airspeed_SET((char)32673) ;
        p115.vz_SET((short)14557) ;
        p115.ind_airspeed_SET((char)19256) ;
        p115.xacc_SET((short) -16533) ;
        p115.vx_SET((short)681) ;
        p115.yawspeed_SET(-1.2767833E37F) ;
        p115.vy_SET((short)5536) ;
        p115.rollspeed_SET(9.932523E37F) ;
        p115.yacc_SET((short) -15278) ;
        p115.time_usec_SET(7055398531856911820L) ;
        p115.zacc_SET((short)11765) ;
        p115.attitude_quaternion_SET(new float[] {2.4994723E38F, -1.937645E38F, 2.1817847E38F, 3.9693745E37F}, 0) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short) -435);
            assert(pack.xmag_GET() == (short) -27325);
            assert(pack.time_boot_ms_GET() == 900788884L);
            assert(pack.xgyro_GET() == (short) -27291);
            assert(pack.yacc_GET() == (short)14918);
            assert(pack.xacc_GET() == (short)24325);
            assert(pack.ygyro_GET() == (short)19941);
            assert(pack.zmag_GET() == (short) -4490);
            assert(pack.ymag_GET() == (short)18785);
            assert(pack.zacc_GET() == (short) -18334);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.ymag_SET((short)18785) ;
        p116.xmag_SET((short) -27325) ;
        p116.zacc_SET((short) -18334) ;
        p116.ygyro_SET((short)19941) ;
        p116.xgyro_SET((short) -27291) ;
        p116.xacc_SET((short)24325) ;
        p116.time_boot_ms_SET(900788884L) ;
        p116.yacc_SET((short)14918) ;
        p116.zgyro_SET((short) -435) ;
        p116.zmag_SET((short) -4490) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)220);
            assert(pack.start_GET() == (char)43988);
            assert(pack.target_system_GET() == (char)135);
            assert(pack.end_GET() == (char)7002);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.end_SET((char)7002) ;
        p117.target_system_SET((char)135) ;
        p117.target_component_SET((char)220) ;
        p117.start_SET((char)43988) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.num_logs_GET() == (char)36428);
            assert(pack.last_log_num_GET() == (char)27787);
            assert(pack.size_GET() == 1290602923L);
            assert(pack.time_utc_GET() == 1822239633L);
            assert(pack.id_GET() == (char)2678);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)2678) ;
        p118.num_logs_SET((char)36428) ;
        p118.time_utc_SET(1822239633L) ;
        p118.last_log_num_SET((char)27787) ;
        p118.size_SET(1290602923L) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)46);
            assert(pack.target_system_GET() == (char)174);
            assert(pack.count_GET() == 2669048603L);
            assert(pack.id_GET() == (char)48426);
            assert(pack.ofs_GET() == 1309972802L);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.ofs_SET(1309972802L) ;
        p119.count_SET(2669048603L) ;
        p119.target_component_SET((char)46) ;
        p119.target_system_SET((char)174) ;
        p119.id_SET((char)48426) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)37947);
            assert(pack.count_GET() == (char)74);
            assert(pack.ofs_GET() == 624883949L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)56, (char)168, (char)155, (char)228, (char)87, (char)156, (char)250, (char)115, (char)6, (char)156, (char)216, (char)55, (char)152, (char)240, (char)143, (char)73, (char)69, (char)53, (char)191, (char)86, (char)32, (char)98, (char)127, (char)183, (char)169, (char)185, (char)15, (char)204, (char)29, (char)254, (char)44, (char)24, (char)241, (char)96, (char)238, (char)92, (char)59, (char)40, (char)74, (char)32, (char)219, (char)202, (char)39, (char)8, (char)26, (char)12, (char)57, (char)197, (char)130, (char)11, (char)116, (char)132, (char)81, (char)100, (char)189, (char)96, (char)111, (char)118, (char)236, (char)148, (char)156, (char)50, (char)210, (char)205, (char)36, (char)82, (char)13, (char)193, (char)217, (char)44, (char)188, (char)121, (char)30, (char)202, (char)32, (char)91, (char)136, (char)157, (char)183, (char)246, (char)73, (char)30, (char)83, (char)237, (char)161, (char)27, (char)122, (char)33, (char)28, (char)52}));
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)37947) ;
        p120.count_SET((char)74) ;
        p120.ofs_SET(624883949L) ;
        p120.data__SET(new char[] {(char)56, (char)168, (char)155, (char)228, (char)87, (char)156, (char)250, (char)115, (char)6, (char)156, (char)216, (char)55, (char)152, (char)240, (char)143, (char)73, (char)69, (char)53, (char)191, (char)86, (char)32, (char)98, (char)127, (char)183, (char)169, (char)185, (char)15, (char)204, (char)29, (char)254, (char)44, (char)24, (char)241, (char)96, (char)238, (char)92, (char)59, (char)40, (char)74, (char)32, (char)219, (char)202, (char)39, (char)8, (char)26, (char)12, (char)57, (char)197, (char)130, (char)11, (char)116, (char)132, (char)81, (char)100, (char)189, (char)96, (char)111, (char)118, (char)236, (char)148, (char)156, (char)50, (char)210, (char)205, (char)36, (char)82, (char)13, (char)193, (char)217, (char)44, (char)188, (char)121, (char)30, (char)202, (char)32, (char)91, (char)136, (char)157, (char)183, (char)246, (char)73, (char)30, (char)83, (char)237, (char)161, (char)27, (char)122, (char)33, (char)28, (char)52}, 0) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)172);
            assert(pack.target_component_GET() == (char)107);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)172) ;
        p121.target_component_SET((char)107) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)120);
            assert(pack.target_component_GET() == (char)181);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)120) ;
        p122.target_component_SET((char)181) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)201, (char)220, (char)239, (char)143, (char)112, (char)45, (char)90, (char)146, (char)134, (char)194, (char)236, (char)250, (char)65, (char)100, (char)137, (char)234, (char)59, (char)7, (char)191, (char)214, (char)247, (char)10, (char)143, (char)127, (char)98, (char)44, (char)153, (char)86, (char)56, (char)27, (char)28, (char)62, (char)241, (char)86, (char)229, (char)29, (char)145, (char)241, (char)47, (char)111, (char)113, (char)37, (char)220, (char)70, (char)100, (char)56, (char)189, (char)235, (char)236, (char)71, (char)21, (char)125, (char)28, (char)6, (char)62, (char)222, (char)60, (char)246, (char)99, (char)76, (char)12, (char)33, (char)176, (char)165, (char)48, (char)144, (char)187, (char)144, (char)4, (char)237, (char)241, (char)66, (char)46, (char)225, (char)44, (char)60, (char)99, (char)59, (char)4, (char)44, (char)141, (char)23, (char)30, (char)78, (char)41, (char)27, (char)71, (char)106, (char)255, (char)255, (char)213, (char)95, (char)119, (char)31, (char)25, (char)128, (char)156, (char)210, (char)216, (char)36, (char)215, (char)45, (char)129, (char)181, (char)60, (char)51, (char)190, (char)21, (char)137, (char)23}));
            assert(pack.target_system_GET() == (char)186);
            assert(pack.len_GET() == (char)24);
            assert(pack.target_component_GET() == (char)250);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.data__SET(new char[] {(char)201, (char)220, (char)239, (char)143, (char)112, (char)45, (char)90, (char)146, (char)134, (char)194, (char)236, (char)250, (char)65, (char)100, (char)137, (char)234, (char)59, (char)7, (char)191, (char)214, (char)247, (char)10, (char)143, (char)127, (char)98, (char)44, (char)153, (char)86, (char)56, (char)27, (char)28, (char)62, (char)241, (char)86, (char)229, (char)29, (char)145, (char)241, (char)47, (char)111, (char)113, (char)37, (char)220, (char)70, (char)100, (char)56, (char)189, (char)235, (char)236, (char)71, (char)21, (char)125, (char)28, (char)6, (char)62, (char)222, (char)60, (char)246, (char)99, (char)76, (char)12, (char)33, (char)176, (char)165, (char)48, (char)144, (char)187, (char)144, (char)4, (char)237, (char)241, (char)66, (char)46, (char)225, (char)44, (char)60, (char)99, (char)59, (char)4, (char)44, (char)141, (char)23, (char)30, (char)78, (char)41, (char)27, (char)71, (char)106, (char)255, (char)255, (char)213, (char)95, (char)119, (char)31, (char)25, (char)128, (char)156, (char)210, (char)216, (char)36, (char)215, (char)45, (char)129, (char)181, (char)60, (char)51, (char)190, (char)21, (char)137, (char)23}, 0) ;
        p123.target_system_SET((char)186) ;
        p123.len_SET((char)24) ;
        p123.target_component_SET((char)250) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.dgps_numch_GET() == (char)25);
            assert(pack.vel_GET() == (char)36903);
            assert(pack.time_usec_GET() == 3617846130327475326L);
            assert(pack.satellites_visible_GET() == (char)248);
            assert(pack.cog_GET() == (char)48198);
            assert(pack.lat_GET() == -744770591);
            assert(pack.lon_GET() == -1759883358);
            assert(pack.alt_GET() == 903225816);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
            assert(pack.dgps_age_GET() == 2444476335L);
            assert(pack.epv_GET() == (char)15579);
            assert(pack.eph_GET() == (char)65270);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.epv_SET((char)15579) ;
        p124.vel_SET((char)36903) ;
        p124.satellites_visible_SET((char)248) ;
        p124.lon_SET(-1759883358) ;
        p124.alt_SET(903225816) ;
        p124.dgps_age_SET(2444476335L) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP) ;
        p124.time_usec_SET(3617846130327475326L) ;
        p124.dgps_numch_SET((char)25) ;
        p124.lat_SET(-744770591) ;
        p124.cog_SET((char)48198) ;
        p124.eph_SET((char)65270) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT));
            assert(pack.Vservo_GET() == (char)57520);
            assert(pack.Vcc_GET() == (char)56989);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT)) ;
        p125.Vservo_SET((char)57520) ;
        p125.Vcc_SET((char)56989) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI));
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)148, (char)193, (char)45, (char)221, (char)155, (char)184, (char)26, (char)255, (char)175, (char)221, (char)106, (char)161, (char)30, (char)113, (char)194, (char)154, (char)76, (char)20, (char)91, (char)111, (char)172, (char)108, (char)66, (char)142, (char)207, (char)95, (char)164, (char)20, (char)215, (char)86, (char)211, (char)129, (char)8, (char)85, (char)118, (char)151, (char)118, (char)129, (char)105, (char)7, (char)5, (char)236, (char)193, (char)229, (char)131, (char)123, (char)105, (char)19, (char)87, (char)231, (char)158, (char)207, (char)61, (char)34, (char)220, (char)255, (char)147, (char)211, (char)142, (char)194, (char)204, (char)9, (char)3, (char)30, (char)93, (char)0, (char)18, (char)125, (char)87, (char)37}));
            assert(pack.count_GET() == (char)131);
            assert(pack.baudrate_GET() == 4037819109L);
            assert(pack.timeout_GET() == (char)55020);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.timeout_SET((char)55020) ;
        p126.data__SET(new char[] {(char)148, (char)193, (char)45, (char)221, (char)155, (char)184, (char)26, (char)255, (char)175, (char)221, (char)106, (char)161, (char)30, (char)113, (char)194, (char)154, (char)76, (char)20, (char)91, (char)111, (char)172, (char)108, (char)66, (char)142, (char)207, (char)95, (char)164, (char)20, (char)215, (char)86, (char)211, (char)129, (char)8, (char)85, (char)118, (char)151, (char)118, (char)129, (char)105, (char)7, (char)5, (char)236, (char)193, (char)229, (char)131, (char)123, (char)105, (char)19, (char)87, (char)231, (char)158, (char)207, (char)61, (char)34, (char)220, (char)255, (char)147, (char)211, (char)142, (char)194, (char)204, (char)9, (char)3, (char)30, (char)93, (char)0, (char)18, (char)125, (char)87, (char)37}, 0) ;
        p126.count_SET((char)131) ;
        p126.baudrate_SET(4037819109L) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI)) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.wn_GET() == (char)31585);
            assert(pack.tow_GET() == 4009204511L);
            assert(pack.nsats_GET() == (char)172);
            assert(pack.baseline_coords_type_GET() == (char)4);
            assert(pack.baseline_a_mm_GET() == -561345865);
            assert(pack.time_last_baseline_ms_GET() == 2364546463L);
            assert(pack.rtk_receiver_id_GET() == (char)40);
            assert(pack.iar_num_hypotheses_GET() == 633783376);
            assert(pack.baseline_b_mm_GET() == 440134461);
            assert(pack.accuracy_GET() == 275306618L);
            assert(pack.baseline_c_mm_GET() == 811377523);
            assert(pack.rtk_rate_GET() == (char)14);
            assert(pack.rtk_health_GET() == (char)43);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(2364546463L) ;
        p127.tow_SET(4009204511L) ;
        p127.baseline_b_mm_SET(440134461) ;
        p127.baseline_a_mm_SET(-561345865) ;
        p127.iar_num_hypotheses_SET(633783376) ;
        p127.nsats_SET((char)172) ;
        p127.rtk_health_SET((char)43) ;
        p127.wn_SET((char)31585) ;
        p127.rtk_receiver_id_SET((char)40) ;
        p127.rtk_rate_SET((char)14) ;
        p127.accuracy_SET(275306618L) ;
        p127.baseline_c_mm_SET(811377523) ;
        p127.baseline_coords_type_SET((char)4) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_c_mm_GET() == -1099429460);
            assert(pack.iar_num_hypotheses_GET() == 1759338171);
            assert(pack.baseline_b_mm_GET() == 694415555);
            assert(pack.baseline_coords_type_GET() == (char)55);
            assert(pack.tow_GET() == 2308223405L);
            assert(pack.rtk_rate_GET() == (char)121);
            assert(pack.baseline_a_mm_GET() == -949043551);
            assert(pack.time_last_baseline_ms_GET() == 2807454355L);
            assert(pack.nsats_GET() == (char)1);
            assert(pack.rtk_health_GET() == (char)192);
            assert(pack.wn_GET() == (char)19814);
            assert(pack.accuracy_GET() == 2814995099L);
            assert(pack.rtk_receiver_id_GET() == (char)242);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_c_mm_SET(-1099429460) ;
        p128.baseline_coords_type_SET((char)55) ;
        p128.tow_SET(2308223405L) ;
        p128.rtk_rate_SET((char)121) ;
        p128.baseline_b_mm_SET(694415555) ;
        p128.nsats_SET((char)1) ;
        p128.wn_SET((char)19814) ;
        p128.accuracy_SET(2814995099L) ;
        p128.rtk_health_SET((char)192) ;
        p128.rtk_receiver_id_SET((char)242) ;
        p128.baseline_a_mm_SET(-949043551) ;
        p128.time_last_baseline_ms_SET(2807454355L) ;
        p128.iar_num_hypotheses_SET(1759338171) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 272821326L);
            assert(pack.zmag_GET() == (short) -17460);
            assert(pack.zgyro_GET() == (short)15242);
            assert(pack.ygyro_GET() == (short) -6586);
            assert(pack.ymag_GET() == (short)19312);
            assert(pack.xmag_GET() == (short) -29832);
            assert(pack.xgyro_GET() == (short)22011);
            assert(pack.xacc_GET() == (short) -29143);
            assert(pack.zacc_GET() == (short) -32039);
            assert(pack.yacc_GET() == (short) -21110);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.xacc_SET((short) -29143) ;
        p129.time_boot_ms_SET(272821326L) ;
        p129.yacc_SET((short) -21110) ;
        p129.xmag_SET((short) -29832) ;
        p129.zacc_SET((short) -32039) ;
        p129.zgyro_SET((short)15242) ;
        p129.ymag_SET((short)19312) ;
        p129.ygyro_SET((short) -6586) ;
        p129.zmag_SET((short) -17460) ;
        p129.xgyro_SET((short)22011) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.payload_GET() == (char)255);
            assert(pack.jpg_quality_GET() == (char)149);
            assert(pack.size_GET() == 1697595697L);
            assert(pack.height_GET() == (char)15542);
            assert(pack.packets_GET() == (char)20894);
            assert(pack.type_GET() == (char)254);
            assert(pack.width_GET() == (char)63172);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)254) ;
        p130.width_SET((char)63172) ;
        p130.height_SET((char)15542) ;
        p130.jpg_quality_SET((char)149) ;
        p130.packets_SET((char)20894) ;
        p130.payload_SET((char)255) ;
        p130.size_SET(1697595697L) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)161, (char)243, (char)68, (char)85, (char)96, (char)197, (char)114, (char)126, (char)40, (char)189, (char)46, (char)45, (char)160, (char)230, (char)233, (char)117, (char)187, (char)195, (char)97, (char)14, (char)164, (char)124, (char)13, (char)10, (char)160, (char)63, (char)198, (char)141, (char)80, (char)18, (char)250, (char)162, (char)122, (char)73, (char)186, (char)249, (char)40, (char)202, (char)164, (char)212, (char)170, (char)137, (char)124, (char)241, (char)4, (char)230, (char)52, (char)121, (char)174, (char)12, (char)248, (char)168, (char)37, (char)89, (char)48, (char)20, (char)219, (char)254, (char)184, (char)7, (char)2, (char)114, (char)71, (char)153, (char)167, (char)195, (char)197, (char)88, (char)1, (char)190, (char)103, (char)63, (char)126, (char)199, (char)194, (char)97, (char)205, (char)56, (char)150, (char)196, (char)213, (char)12, (char)237, (char)250, (char)23, (char)12, (char)203, (char)229, (char)200, (char)252, (char)176, (char)111, (char)31, (char)163, (char)31, (char)176, (char)127, (char)55, (char)16, (char)89, (char)62, (char)1, (char)17, (char)32, (char)20, (char)15, (char)68, (char)13, (char)131, (char)151, (char)241, (char)193, (char)206, (char)194, (char)153, (char)158, (char)0, (char)142, (char)77, (char)59, (char)123, (char)46, (char)101, (char)25, (char)7, (char)139, (char)127, (char)151, (char)128, (char)219, (char)9, (char)238, (char)41, (char)38, (char)160, (char)229, (char)203, (char)78, (char)113, (char)75, (char)224, (char)253, (char)68, (char)173, (char)249, (char)2, (char)166, (char)29, (char)31, (char)49, (char)174, (char)80, (char)162, (char)50, (char)199, (char)107, (char)16, (char)75, (char)72, (char)89, (char)133, (char)131, (char)41, (char)18, (char)109, (char)104, (char)234, (char)186, (char)23, (char)7, (char)212, (char)60, (char)17, (char)221, (char)111, (char)244, (char)168, (char)211, (char)76, (char)88, (char)173, (char)191, (char)255, (char)9, (char)124, (char)210, (char)23, (char)202, (char)223, (char)198, (char)49, (char)62, (char)253, (char)85, (char)13, (char)198, (char)13, (char)242, (char)89, (char)73, (char)33, (char)55, (char)70, (char)238, (char)66, (char)81, (char)82, (char)149, (char)1, (char)186, (char)226, (char)163, (char)16, (char)45, (char)185, (char)191, (char)110, (char)85, (char)237, (char)151, (char)198, (char)91, (char)79, (char)247, (char)19, (char)17, (char)61, (char)149, (char)13, (char)173, (char)224, (char)196, (char)175, (char)211, (char)35, (char)83, (char)89, (char)95, (char)197, (char)158, (char)118, (char)42, (char)128, (char)85, (char)234, (char)226, (char)192, (char)196, (char)146, (char)214, (char)37, (char)35, (char)174}));
            assert(pack.seqnr_GET() == (char)3766);
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.data__SET(new char[] {(char)161, (char)243, (char)68, (char)85, (char)96, (char)197, (char)114, (char)126, (char)40, (char)189, (char)46, (char)45, (char)160, (char)230, (char)233, (char)117, (char)187, (char)195, (char)97, (char)14, (char)164, (char)124, (char)13, (char)10, (char)160, (char)63, (char)198, (char)141, (char)80, (char)18, (char)250, (char)162, (char)122, (char)73, (char)186, (char)249, (char)40, (char)202, (char)164, (char)212, (char)170, (char)137, (char)124, (char)241, (char)4, (char)230, (char)52, (char)121, (char)174, (char)12, (char)248, (char)168, (char)37, (char)89, (char)48, (char)20, (char)219, (char)254, (char)184, (char)7, (char)2, (char)114, (char)71, (char)153, (char)167, (char)195, (char)197, (char)88, (char)1, (char)190, (char)103, (char)63, (char)126, (char)199, (char)194, (char)97, (char)205, (char)56, (char)150, (char)196, (char)213, (char)12, (char)237, (char)250, (char)23, (char)12, (char)203, (char)229, (char)200, (char)252, (char)176, (char)111, (char)31, (char)163, (char)31, (char)176, (char)127, (char)55, (char)16, (char)89, (char)62, (char)1, (char)17, (char)32, (char)20, (char)15, (char)68, (char)13, (char)131, (char)151, (char)241, (char)193, (char)206, (char)194, (char)153, (char)158, (char)0, (char)142, (char)77, (char)59, (char)123, (char)46, (char)101, (char)25, (char)7, (char)139, (char)127, (char)151, (char)128, (char)219, (char)9, (char)238, (char)41, (char)38, (char)160, (char)229, (char)203, (char)78, (char)113, (char)75, (char)224, (char)253, (char)68, (char)173, (char)249, (char)2, (char)166, (char)29, (char)31, (char)49, (char)174, (char)80, (char)162, (char)50, (char)199, (char)107, (char)16, (char)75, (char)72, (char)89, (char)133, (char)131, (char)41, (char)18, (char)109, (char)104, (char)234, (char)186, (char)23, (char)7, (char)212, (char)60, (char)17, (char)221, (char)111, (char)244, (char)168, (char)211, (char)76, (char)88, (char)173, (char)191, (char)255, (char)9, (char)124, (char)210, (char)23, (char)202, (char)223, (char)198, (char)49, (char)62, (char)253, (char)85, (char)13, (char)198, (char)13, (char)242, (char)89, (char)73, (char)33, (char)55, (char)70, (char)238, (char)66, (char)81, (char)82, (char)149, (char)1, (char)186, (char)226, (char)163, (char)16, (char)45, (char)185, (char)191, (char)110, (char)85, (char)237, (char)151, (char)198, (char)91, (char)79, (char)247, (char)19, (char)17, (char)61, (char)149, (char)13, (char)173, (char)224, (char)196, (char)175, (char)211, (char)35, (char)83, (char)89, (char)95, (char)197, (char)158, (char)118, (char)42, (char)128, (char)85, (char)234, (char)226, (char)192, (char)196, (char)146, (char)214, (char)37, (char)35, (char)174}, 0) ;
        p131.seqnr_SET((char)3766) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.min_distance_GET() == (char)16841);
            assert(pack.current_distance_GET() == (char)32223);
            assert(pack.max_distance_GET() == (char)29312);
            assert(pack.id_GET() == (char)229);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_45);
            assert(pack.covariance_GET() == (char)235);
            assert(pack.time_boot_ms_GET() == 4067426706L);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p132.time_boot_ms_SET(4067426706L) ;
        p132.covariance_SET((char)235) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_45) ;
        p132.current_distance_SET((char)32223) ;
        p132.min_distance_SET((char)16841) ;
        p132.id_SET((char)229) ;
        p132.max_distance_SET((char)29312) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 1781451557);
            assert(pack.grid_spacing_GET() == (char)21177);
            assert(pack.lat_GET() == 232028952);
            assert(pack.mask_GET() == 4841331668978225330L);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lon_SET(1781451557) ;
        p133.grid_spacing_SET((char)21177) ;
        p133.lat_SET(232028952) ;
        p133.mask_SET(4841331668978225330L) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -14724, (short) -19615, (short)21824, (short)12037, (short)12262, (short) -1415, (short)7761, (short)11164, (short) -3710, (short)28696, (short) -8412, (short)3964, (short)30294, (short) -1456, (short)12134, (short) -16706}));
            assert(pack.lat_GET() == 2122943375);
            assert(pack.grid_spacing_GET() == (char)60284);
            assert(pack.lon_GET() == 896112392);
            assert(pack.gridbit_GET() == (char)57);
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.gridbit_SET((char)57) ;
        p134.grid_spacing_SET((char)60284) ;
        p134.lon_SET(896112392) ;
        p134.data__SET(new short[] {(short) -14724, (short) -19615, (short)21824, (short)12037, (short)12262, (short) -1415, (short)7761, (short)11164, (short) -3710, (short)28696, (short) -8412, (short)3964, (short)30294, (short) -1456, (short)12134, (short) -16706}, 0) ;
        p134.lat_SET(2122943375) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 105310979);
            assert(pack.lon_GET() == 671739536);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(671739536) ;
        p135.lat_SET(105310979) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.loaded_GET() == (char)55558);
            assert(pack.current_height_GET() == 2.9793673E38F);
            assert(pack.lat_GET() == 100729284);
            assert(pack.pending_GET() == (char)25977);
            assert(pack.lon_GET() == -1546444321);
            assert(pack.spacing_GET() == (char)62538);
            assert(pack.terrain_height_GET() == 2.1462329E38F);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lon_SET(-1546444321) ;
        p136.terrain_height_SET(2.1462329E38F) ;
        p136.current_height_SET(2.9793673E38F) ;
        p136.spacing_SET((char)62538) ;
        p136.lat_SET(100729284) ;
        p136.loaded_SET((char)55558) ;
        p136.pending_SET((char)25977) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -2.648089E38F);
            assert(pack.temperature_GET() == (short)1209);
            assert(pack.press_diff_GET() == 7.220203E37F);
            assert(pack.time_boot_ms_GET() == 1085855150L);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_abs_SET(-2.648089E38F) ;
        p137.temperature_SET((short)1209) ;
        p137.time_boot_ms_SET(1085855150L) ;
        p137.press_diff_SET(7.220203E37F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 1414028246076953855L);
            assert(pack.x_GET() == 3.171567E38F);
            assert(pack.z_GET() == 1.6586056E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.979522E37F, -7.050558E37F, -7.7503083E37F, 1.6359651E38F}));
            assert(pack.y_GET() == -1.7269944E38F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.z_SET(1.6586056E38F) ;
        p138.x_SET(3.171567E38F) ;
        p138.time_usec_SET(1414028246076953855L) ;
        p138.y_SET(-1.7269944E38F) ;
        p138.q_SET(new float[] {-2.979522E37F, -7.050558E37F, -7.7503083E37F, 1.6359651E38F}, 0) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6279707013010061225L);
            assert(pack.target_component_GET() == (char)91);
            assert(pack.group_mlx_GET() == (char)128);
            assert(pack.target_system_GET() == (char)124);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.525157E37F, -1.4303895E38F, -3.0066907E35F, 3.292148E38F, -1.945795E38F, 1.1790605E38F, 3.020181E38F, -5.4771215E36F}));
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.target_component_SET((char)91) ;
        p139.group_mlx_SET((char)128) ;
        p139.time_usec_SET(6279707013010061225L) ;
        p139.target_system_SET((char)124) ;
        p139.controls_SET(new float[] {-2.525157E37F, -1.4303895E38F, -3.0066907E35F, 3.292148E38F, -1.945795E38F, 1.1790605E38F, 3.020181E38F, -5.4771215E36F}, 0) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)91);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-6.578843E37F, 2.7369213E38F, 2.5465772E38F, -2.881017E38F, 2.942841E38F, -9.433784E37F, 2.1681476E38F, 8.582257E37F}));
            assert(pack.time_usec_GET() == 4159844181104539941L);
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(4159844181104539941L) ;
        p140.group_mlx_SET((char)91) ;
        p140.controls_SET(new float[] {-6.578843E37F, 2.7369213E38F, 2.5465772E38F, -2.881017E38F, 2.942841E38F, -9.433784E37F, 2.1681476E38F, 8.582257E37F}, 0) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.bottom_clearance_GET() == -1.902378E38F);
            assert(pack.time_usec_GET() == 3242167612900503900L);
            assert(pack.altitude_monotonic_GET() == -1.2680997E38F);
            assert(pack.altitude_local_GET() == -3.3445152E38F);
            assert(pack.altitude_relative_GET() == -1.2378462E38F);
            assert(pack.altitude_amsl_GET() == -3.063888E38F);
            assert(pack.altitude_terrain_GET() == 1.6523988E38F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_local_SET(-3.3445152E38F) ;
        p141.altitude_relative_SET(-1.2378462E38F) ;
        p141.altitude_amsl_SET(-3.063888E38F) ;
        p141.bottom_clearance_SET(-1.902378E38F) ;
        p141.altitude_monotonic_SET(-1.2680997E38F) ;
        p141.altitude_terrain_SET(1.6523988E38F) ;
        p141.time_usec_SET(3242167612900503900L) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)130, (char)75, (char)2, (char)253, (char)247, (char)178, (char)214, (char)46, (char)87, (char)37, (char)204, (char)159, (char)223, (char)59, (char)126, (char)114, (char)190, (char)94, (char)84, (char)171, (char)223, (char)214, (char)133, (char)186, (char)88, (char)129, (char)245, (char)160, (char)203, (char)20, (char)83, (char)43, (char)73, (char)129, (char)178, (char)26, (char)101, (char)32, (char)61, (char)16, (char)72, (char)189, (char)86, (char)56, (char)156, (char)104, (char)223, (char)104, (char)58, (char)10, (char)102, (char)115, (char)23, (char)223, (char)144, (char)152, (char)244, (char)222, (char)54, (char)93, (char)27, (char)236, (char)163, (char)10, (char)232, (char)77, (char)142, (char)118, (char)135, (char)147, (char)175, (char)126, (char)109, (char)47, (char)86, (char)51, (char)103, (char)40, (char)192, (char)216, (char)249, (char)69, (char)179, (char)132, (char)72, (char)176, (char)89, (char)165, (char)35, (char)95, (char)28, (char)174, (char)81, (char)127, (char)236, (char)238, (char)235, (char)152, (char)162, (char)124, (char)55, (char)244, (char)104, (char)155, (char)68, (char)14, (char)180, (char)19, (char)8, (char)89, (char)194, (char)56, (char)40, (char)74, (char)207, (char)200, (char)29, (char)17, (char)5, (char)140}));
            assert(pack.request_id_GET() == (char)149);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)183, (char)222, (char)173, (char)194, (char)11, (char)112, (char)195, (char)33, (char)78, (char)246, (char)200, (char)119, (char)234, (char)93, (char)27, (char)244, (char)185, (char)26, (char)8, (char)230, (char)193, (char)143, (char)18, (char)241, (char)40, (char)100, (char)150, (char)56, (char)227, (char)51, (char)169, (char)31, (char)198, (char)230, (char)42, (char)54, (char)110, (char)34, (char)143, (char)193, (char)101, (char)59, (char)77, (char)225, (char)153, (char)36, (char)19, (char)131, (char)8, (char)112, (char)163, (char)14, (char)225, (char)217, (char)243, (char)93, (char)99, (char)26, (char)246, (char)90, (char)203, (char)208, (char)220, (char)171, (char)9, (char)129, (char)246, (char)36, (char)182, (char)189, (char)13, (char)177, (char)162, (char)122, (char)207, (char)70, (char)247, (char)220, (char)237, (char)49, (char)26, (char)69, (char)102, (char)209, (char)5, (char)134, (char)99, (char)112, (char)38, (char)32, (char)177, (char)175, (char)143, (char)97, (char)172, (char)3, (char)59, (char)246, (char)159, (char)202, (char)7, (char)92, (char)141, (char)204, (char)146, (char)38, (char)157, (char)134, (char)229, (char)236, (char)31, (char)241, (char)67, (char)219, (char)191, (char)20, (char)15, (char)163, (char)113, (char)249}));
            assert(pack.transfer_type_GET() == (char)80);
            assert(pack.uri_type_GET() == (char)210);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.transfer_type_SET((char)80) ;
        p142.uri_type_SET((char)210) ;
        p142.request_id_SET((char)149) ;
        p142.uri_SET(new char[] {(char)183, (char)222, (char)173, (char)194, (char)11, (char)112, (char)195, (char)33, (char)78, (char)246, (char)200, (char)119, (char)234, (char)93, (char)27, (char)244, (char)185, (char)26, (char)8, (char)230, (char)193, (char)143, (char)18, (char)241, (char)40, (char)100, (char)150, (char)56, (char)227, (char)51, (char)169, (char)31, (char)198, (char)230, (char)42, (char)54, (char)110, (char)34, (char)143, (char)193, (char)101, (char)59, (char)77, (char)225, (char)153, (char)36, (char)19, (char)131, (char)8, (char)112, (char)163, (char)14, (char)225, (char)217, (char)243, (char)93, (char)99, (char)26, (char)246, (char)90, (char)203, (char)208, (char)220, (char)171, (char)9, (char)129, (char)246, (char)36, (char)182, (char)189, (char)13, (char)177, (char)162, (char)122, (char)207, (char)70, (char)247, (char)220, (char)237, (char)49, (char)26, (char)69, (char)102, (char)209, (char)5, (char)134, (char)99, (char)112, (char)38, (char)32, (char)177, (char)175, (char)143, (char)97, (char)172, (char)3, (char)59, (char)246, (char)159, (char)202, (char)7, (char)92, (char)141, (char)204, (char)146, (char)38, (char)157, (char)134, (char)229, (char)236, (char)31, (char)241, (char)67, (char)219, (char)191, (char)20, (char)15, (char)163, (char)113, (char)249}, 0) ;
        p142.storage_SET(new char[] {(char)130, (char)75, (char)2, (char)253, (char)247, (char)178, (char)214, (char)46, (char)87, (char)37, (char)204, (char)159, (char)223, (char)59, (char)126, (char)114, (char)190, (char)94, (char)84, (char)171, (char)223, (char)214, (char)133, (char)186, (char)88, (char)129, (char)245, (char)160, (char)203, (char)20, (char)83, (char)43, (char)73, (char)129, (char)178, (char)26, (char)101, (char)32, (char)61, (char)16, (char)72, (char)189, (char)86, (char)56, (char)156, (char)104, (char)223, (char)104, (char)58, (char)10, (char)102, (char)115, (char)23, (char)223, (char)144, (char)152, (char)244, (char)222, (char)54, (char)93, (char)27, (char)236, (char)163, (char)10, (char)232, (char)77, (char)142, (char)118, (char)135, (char)147, (char)175, (char)126, (char)109, (char)47, (char)86, (char)51, (char)103, (char)40, (char)192, (char)216, (char)249, (char)69, (char)179, (char)132, (char)72, (char)176, (char)89, (char)165, (char)35, (char)95, (char)28, (char)174, (char)81, (char)127, (char)236, (char)238, (char)235, (char)152, (char)162, (char)124, (char)55, (char)244, (char)104, (char)155, (char)68, (char)14, (char)180, (char)19, (char)8, (char)89, (char)194, (char)56, (char)40, (char)74, (char)207, (char)200, (char)29, (char)17, (char)5, (char)140}, 0) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -1.2007622E38F);
            assert(pack.temperature_GET() == (short) -16233);
            assert(pack.time_boot_ms_GET() == 4189497299L);
            assert(pack.press_diff_GET() == 6.814453E37F);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.temperature_SET((short) -16233) ;
        p143.press_abs_SET(-1.2007622E38F) ;
        p143.press_diff_SET(6.814453E37F) ;
        p143.time_boot_ms_SET(4189497299L) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.custom_state_GET() == 714547988653746009L);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {2.427946E38F, -3.3472975E38F, -2.4606116E38F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {3.215456E38F, 3.105352E38F, -1.3354526E38F, -5.025331E36F}));
            assert(pack.est_capabilities_GET() == (char)121);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {9.383843E37F, -8.699392E37F, 1.0245096E38F}));
            assert(pack.timestamp_GET() == 3793067147096081446L);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-3.9564159E37F, -1.4203428E38F, 2.6195166E38F}));
            assert(Arrays.equals(pack.acc_GET(),  new float[] {3.9901058E36F, -1.7432928E38F, 7.288535E37F}));
            assert(pack.alt_GET() == -2.1705413E38F);
            assert(pack.lat_GET() == -173061658);
            assert(pack.lon_GET() == -1527782585);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.vel_SET(new float[] {9.383843E37F, -8.699392E37F, 1.0245096E38F}, 0) ;
        p144.lon_SET(-1527782585) ;
        p144.attitude_q_SET(new float[] {3.215456E38F, 3.105352E38F, -1.3354526E38F, -5.025331E36F}, 0) ;
        p144.lat_SET(-173061658) ;
        p144.est_capabilities_SET((char)121) ;
        p144.rates_SET(new float[] {2.427946E38F, -3.3472975E38F, -2.4606116E38F}, 0) ;
        p144.position_cov_SET(new float[] {-3.9564159E37F, -1.4203428E38F, 2.6195166E38F}, 0) ;
        p144.timestamp_SET(3793067147096081446L) ;
        p144.alt_SET(-2.1705413E38F) ;
        p144.acc_SET(new float[] {3.9901058E36F, -1.7432928E38F, 7.288535E37F}, 0) ;
        p144.custom_state_SET(714547988653746009L) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.9991466E37F, -2.463303E38F, -1.3504934E38F, -2.0954698E38F}));
            assert(pack.z_acc_GET() == 1.4390973E38F);
            assert(pack.airspeed_GET() == 1.6957673E37F);
            assert(pack.z_vel_GET() == 8.070898E37F);
            assert(pack.y_vel_GET() == -2.3767942E38F);
            assert(pack.roll_rate_GET() == 1.6335524E38F);
            assert(pack.x_pos_GET() == -1.9393983E38F);
            assert(pack.yaw_rate_GET() == 1.0026321E38F);
            assert(pack.z_pos_GET() == -3.878147E37F);
            assert(pack.x_vel_GET() == 1.7046876E38F);
            assert(pack.time_usec_GET() == 4544373027619765424L);
            assert(pack.y_acc_GET() == -2.3616726E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {-2.4755464E38F, -8.682522E37F, -1.0487463E38F}));
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {9.434611E37F, -1.0451792E38F, 9.091011E37F}));
            assert(pack.y_pos_GET() == 3.0603008E38F);
            assert(pack.x_acc_GET() == 1.8019142E37F);
            assert(pack.pitch_rate_GET() == 1.1290037E38F);
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.x_pos_SET(-1.9393983E38F) ;
        p146.pitch_rate_SET(1.1290037E38F) ;
        p146.z_pos_SET(-3.878147E37F) ;
        p146.z_vel_SET(8.070898E37F) ;
        p146.roll_rate_SET(1.6335524E38F) ;
        p146.z_acc_SET(1.4390973E38F) ;
        p146.time_usec_SET(4544373027619765424L) ;
        p146.pos_variance_SET(new float[] {-2.4755464E38F, -8.682522E37F, -1.0487463E38F}, 0) ;
        p146.airspeed_SET(1.6957673E37F) ;
        p146.x_acc_SET(1.8019142E37F) ;
        p146.vel_variance_SET(new float[] {9.434611E37F, -1.0451792E38F, 9.091011E37F}, 0) ;
        p146.q_SET(new float[] {3.9991466E37F, -2.463303E38F, -1.3504934E38F, -2.0954698E38F}, 0) ;
        p146.y_pos_SET(3.0603008E38F) ;
        p146.y_vel_SET(-2.3767942E38F) ;
        p146.yaw_rate_SET(1.0026321E38F) ;
        p146.y_acc_SET(-2.3616726E38F) ;
        p146.x_vel_SET(1.7046876E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.current_consumed_GET() == -1198922734);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
            assert(pack.temperature_GET() == (short) -15662);
            assert(pack.energy_consumed_GET() == 1138681107);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
            assert(pack.battery_remaining_GET() == (byte) - 108);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)55407, (char)52061, (char)17687, (char)46868, (char)22618, (char)44258, (char)13167, (char)36282, (char)49676, (char)15911}));
            assert(pack.current_battery_GET() == (short)7807);
            assert(pack.id_GET() == (char)99);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.current_consumed_SET(-1198922734) ;
        p147.battery_remaining_SET((byte) - 108) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD) ;
        p147.current_battery_SET((short)7807) ;
        p147.energy_consumed_SET(1138681107) ;
        p147.voltages_SET(new char[] {(char)55407, (char)52061, (char)17687, (char)46868, (char)22618, (char)44258, (char)13167, (char)36282, (char)49676, (char)15911}, 0) ;
        p147.id_SET((char)99) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION) ;
        p147.temperature_SET((short) -15662) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.middleware_sw_version_GET() == 3649780622L);
            assert(pack.board_version_GET() == 2297477769L);
            assert(pack.os_sw_version_GET() == 3590891549L);
            assert(pack.vendor_id_GET() == (char)47457);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)16, (char)107, (char)234, (char)169, (char)127, (char)173, (char)82, (char)205}));
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)214, (char)184, (char)44, (char)206, (char)220, (char)172, (char)215, (char)160}));
            assert(pack.flight_sw_version_GET() == 1917798443L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)36, (char)139, (char)251, (char)17, (char)253, (char)206, (char)127, (char)99, (char)68, (char)239, (char)170, (char)91, (char)44, (char)177, (char)156, (char)56, (char)218, (char)244}));
            assert(pack.product_id_GET() == (char)31188);
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
            assert(pack.uid_GET() == 7361367353714997343L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)23, (char)60, (char)58, (char)91, (char)32, (char)49, (char)3, (char)156}));
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.vendor_id_SET((char)47457) ;
        p148.product_id_SET((char)31188) ;
        p148.os_custom_version_SET(new char[] {(char)23, (char)60, (char)58, (char)91, (char)32, (char)49, (char)3, (char)156}, 0) ;
        p148.middleware_sw_version_SET(3649780622L) ;
        p148.flight_sw_version_SET(1917798443L) ;
        p148.flight_custom_version_SET(new char[] {(char)16, (char)107, (char)234, (char)169, (char)127, (char)173, (char)82, (char)205}, 0) ;
        p148.uid2_SET(new char[] {(char)36, (char)139, (char)251, (char)17, (char)253, (char)206, (char)127, (char)99, (char)68, (char)239, (char)170, (char)91, (char)44, (char)177, (char)156, (char)56, (char)218, (char)244}, 0, PH) ;
        p148.middleware_custom_version_SET(new char[] {(char)214, (char)184, (char)44, (char)206, (char)220, (char)172, (char)215, (char)160}, 0) ;
        p148.os_sw_version_SET(3590891549L) ;
        p148.board_version_SET(2297477769L) ;
        p148.uid_SET(7361367353714997343L) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE)) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.size_x_GET() == 1.491505E37F);
            assert(pack.angle_x_GET() == -1.8205268E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
            assert(pack.size_y_GET() == -1.3108754E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {2.0239236E38F, -1.754513E38F, 5.147967E37F, -1.3944636E38F}));
            assert(pack.x_TRY(ph) == -2.9747403E38F);
            assert(pack.position_valid_TRY(ph) == (char)68);
            assert(pack.distance_GET() == -1.2636812E38F);
            assert(pack.target_num_GET() == (char)199);
            assert(pack.z_TRY(ph) == -1.8432611E38F);
            assert(pack.time_usec_GET() == 6305009699560519871L);
            assert(pack.angle_y_GET() == 2.2639816E38F);
            assert(pack.y_TRY(ph) == 3.2071428E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p149.time_usec_SET(6305009699560519871L) ;
        p149.x_SET(-2.9747403E38F, PH) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON) ;
        p149.size_x_SET(1.491505E37F) ;
        p149.position_valid_SET((char)68, PH) ;
        p149.angle_x_SET(-1.8205268E38F) ;
        p149.target_num_SET((char)199) ;
        p149.y_SET(3.2071428E38F, PH) ;
        p149.z_SET(-1.8432611E38F, PH) ;
        p149.angle_y_SET(2.2639816E38F) ;
        p149.q_SET(new float[] {2.0239236E38F, -1.754513E38F, 5.147967E37F, -1.3944636E38F}, 0, PH) ;
        p149.distance_SET(-1.2636812E38F) ;
        p149.size_y_SET(-1.3108754E38F) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AQ_TELEMETRY_F.add((src, ph, pack) ->
        {
            assert(pack.value1_GET() == -1.5485194E38F);
            assert(pack.value12_GET() == 1.830033E38F);
            assert(pack.value8_GET() == -2.1048543E38F);
            assert(pack.value6_GET() == -2.097909E38F);
            assert(pack.value5_GET() == -3.692315E37F);
            assert(pack.value3_GET() == -1.1046077E38F);
            assert(pack.value17_GET() == -1.3526038E38F);
            assert(pack.value4_GET() == -2.4962467E37F);
            assert(pack.value18_GET() == 1.2437903E38F);
            assert(pack.value13_GET() == 1.6999387E38F);
            assert(pack.value15_GET() == -1.3286615E37F);
            assert(pack.value14_GET() == 1.8965002E38F);
            assert(pack.value20_GET() == 6.3451216E37F);
            assert(pack.value10_GET() == 2.7518135E38F);
            assert(pack.Index_GET() == (char)45638);
            assert(pack.value9_GET() == 2.6268075E38F);
            assert(pack.value2_GET() == 2.7444321E38F);
            assert(pack.value11_GET() == -2.4743958E38F);
            assert(pack.value16_GET() == -3.22878E38F);
            assert(pack.value7_GET() == 2.9143529E38F);
            assert(pack.value19_GET() == 1.2732335E38F);
        });
        GroundControl.AQ_TELEMETRY_F p150 = CommunicationChannel.new_AQ_TELEMETRY_F();
        PH.setPack(p150);
        p150.Index_SET((char)45638) ;
        p150.value15_SET(-1.3286615E37F) ;
        p150.value14_SET(1.8965002E38F) ;
        p150.value2_SET(2.7444321E38F) ;
        p150.value19_SET(1.2732335E38F) ;
        p150.value20_SET(6.3451216E37F) ;
        p150.value5_SET(-3.692315E37F) ;
        p150.value16_SET(-3.22878E38F) ;
        p150.value11_SET(-2.4743958E38F) ;
        p150.value8_SET(-2.1048543E38F) ;
        p150.value12_SET(1.830033E38F) ;
        p150.value17_SET(-1.3526038E38F) ;
        p150.value4_SET(-2.4962467E37F) ;
        p150.value13_SET(1.6999387E38F) ;
        p150.value6_SET(-2.097909E38F) ;
        p150.value10_SET(2.7518135E38F) ;
        p150.value7_SET(2.9143529E38F) ;
        p150.value9_SET(2.6268075E38F) ;
        p150.value18_SET(1.2437903E38F) ;
        p150.value1_SET(-1.5485194E38F) ;
        p150.value3_SET(-1.1046077E38F) ;
        CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AQ_ESC_TELEMETRY.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1284658435L);
            assert(Arrays.equals(pack.data0_GET(),  new long[] {1231973784L, 2994359695L, 4263266756L, 1947826120L}));
            assert(pack.num_motors_GET() == (char)198);
            assert(pack.num_in_seq_GET() == (char)89);
            assert(Arrays.equals(pack.escid_GET(),  new char[] {(char)113, (char)227, (char)113, (char)187}));
            assert(Arrays.equals(pack.data1_GET(),  new long[] {2941719355L, 4219327104L, 503878411L, 3458838973L}));
            assert(Arrays.equals(pack.data_version_GET(),  new char[] {(char)165, (char)126, (char)52, (char)223}));
            assert(pack.seq_GET() == (char)135);
            assert(Arrays.equals(pack.status_age_GET(),  new char[] {(char)21490, (char)15960, (char)56777, (char)25769}));
        });
        GroundControl.AQ_ESC_TELEMETRY p152 = CommunicationChannel.new_AQ_ESC_TELEMETRY();
        PH.setPack(p152);
        p152.num_motors_SET((char)198) ;
        p152.data1_SET(new long[] {2941719355L, 4219327104L, 503878411L, 3458838973L}, 0) ;
        p152.time_boot_ms_SET(1284658435L) ;
        p152.seq_SET((char)135) ;
        p152.data0_SET(new long[] {1231973784L, 2994359695L, 4263266756L, 1947826120L}, 0) ;
        p152.data_version_SET(new char[] {(char)165, (char)126, (char)52, (char)223}, 0) ;
        p152.status_age_SET(new char[] {(char)21490, (char)15960, (char)56777, (char)25769}, 0) ;
        p152.num_in_seq_SET((char)89) ;
        p152.escid_SET(new char[] {(char)113, (char)227, (char)113, (char)187}, 0) ;
        CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.vel_ratio_GET() == -1.8633156E38F);
            assert(pack.mag_ratio_GET() == -2.7804667E38F);
            assert(pack.pos_vert_ratio_GET() == -3.2416657E38F);
            assert(pack.hagl_ratio_GET() == -3.3259223E38F);
            assert(pack.pos_horiz_ratio_GET() == -1.5020179E38F);
            assert(pack.pos_vert_accuracy_GET() == 2.4919258E38F);
            assert(pack.time_usec_GET() == 5237142067801541827L);
            assert(pack.tas_ratio_GET() == -1.6677335E38F);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS));
            assert(pack.pos_horiz_accuracy_GET() == -2.9966853E38F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.hagl_ratio_SET(-3.3259223E38F) ;
        p230.pos_horiz_ratio_SET(-1.5020179E38F) ;
        p230.pos_vert_accuracy_SET(2.4919258E38F) ;
        p230.pos_vert_ratio_SET(-3.2416657E38F) ;
        p230.vel_ratio_SET(-1.8633156E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS)) ;
        p230.tas_ratio_SET(-1.6677335E38F) ;
        p230.mag_ratio_SET(-2.7804667E38F) ;
        p230.time_usec_SET(5237142067801541827L) ;
        p230.pos_horiz_accuracy_SET(-2.9966853E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_z_GET() == 2.0393296E38F);
            assert(pack.time_usec_GET() == 2880682202662810406L);
            assert(pack.wind_y_GET() == 1.5261302E38F);
            assert(pack.var_vert_GET() == -2.007332E38F);
            assert(pack.var_horiz_GET() == -3.1223822E38F);
            assert(pack.horiz_accuracy_GET() == 2.879077E38F);
            assert(pack.wind_x_GET() == -2.9410601E38F);
            assert(pack.vert_accuracy_GET() == 7.50254E36F);
            assert(pack.wind_alt_GET() == 1.817326E38F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.vert_accuracy_SET(7.50254E36F) ;
        p231.var_vert_SET(-2.007332E38F) ;
        p231.time_usec_SET(2880682202662810406L) ;
        p231.wind_y_SET(1.5261302E38F) ;
        p231.horiz_accuracy_SET(2.879077E38F) ;
        p231.wind_z_SET(2.0393296E38F) ;
        p231.var_horiz_SET(-3.1223822E38F) ;
        p231.wind_alt_SET(1.817326E38F) ;
        p231.wind_x_SET(-2.9410601E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY));
            assert(pack.time_week_GET() == (char)60110);
            assert(pack.time_usec_GET() == 2159580844416179622L);
            assert(pack.vd_GET() == -1.3296171E38F);
            assert(pack.alt_GET() == -2.2756983E38F);
            assert(pack.gps_id_GET() == (char)188);
            assert(pack.ve_GET() == -1.2952135E38F);
            assert(pack.speed_accuracy_GET() == -1.5429397E37F);
            assert(pack.fix_type_GET() == (char)31);
            assert(pack.vn_GET() == -2.1805692E38F);
            assert(pack.time_week_ms_GET() == 3286327721L);
            assert(pack.vdop_GET() == -1.3511052E38F);
            assert(pack.vert_accuracy_GET() == 2.0051068E38F);
            assert(pack.lon_GET() == 577735474);
            assert(pack.satellites_visible_GET() == (char)52);
            assert(pack.hdop_GET() == 6.492707E37F);
            assert(pack.lat_GET() == 1827757537);
            assert(pack.horiz_accuracy_GET() == 2.3927733E38F);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.vd_SET(-1.3296171E38F) ;
        p232.vdop_SET(-1.3511052E38F) ;
        p232.lon_SET(577735474) ;
        p232.lat_SET(1827757537) ;
        p232.alt_SET(-2.2756983E38F) ;
        p232.satellites_visible_SET((char)52) ;
        p232.time_week_SET((char)60110) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY)) ;
        p232.time_week_ms_SET(3286327721L) ;
        p232.time_usec_SET(2159580844416179622L) ;
        p232.speed_accuracy_SET(-1.5429397E37F) ;
        p232.vert_accuracy_SET(2.0051068E38F) ;
        p232.horiz_accuracy_SET(2.3927733E38F) ;
        p232.fix_type_SET((char)31) ;
        p232.ve_SET(-1.2952135E38F) ;
        p232.gps_id_SET((char)188) ;
        p232.vn_SET(-2.1805692E38F) ;
        p232.hdop_SET(6.492707E37F) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)114);
            assert(pack.flags_GET() == (char)221);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)120, (char)79, (char)183, (char)10, (char)153, (char)204, (char)164, (char)248, (char)59, (char)242, (char)178, (char)53, (char)211, (char)7, (char)165, (char)15, (char)42, (char)73, (char)235, (char)11, (char)57, (char)209, (char)161, (char)10, (char)168, (char)104, (char)42, (char)218, (char)5, (char)219, (char)1, (char)132, (char)39, (char)249, (char)189, (char)30, (char)189, (char)68, (char)47, (char)9, (char)21, (char)211, (char)54, (char)221, (char)16, (char)169, (char)61, (char)95, (char)44, (char)72, (char)212, (char)78, (char)161, (char)246, (char)248, (char)67, (char)73, (char)176, (char)188, (char)37, (char)126, (char)55, (char)239, (char)115, (char)238, (char)144, (char)87, (char)128, (char)202, (char)46, (char)182, (char)11, (char)149, (char)13, (char)228, (char)150, (char)231, (char)231, (char)105, (char)220, (char)168, (char)205, (char)156, (char)146, (char)168, (char)116, (char)215, (char)91, (char)65, (char)236, (char)15, (char)73, (char)178, (char)28, (char)98, (char)146, (char)2, (char)238, (char)19, (char)202, (char)241, (char)97, (char)2, (char)124, (char)37, (char)72, (char)13, (char)88, (char)44, (char)128, (char)6, (char)104, (char)181, (char)70, (char)72, (char)203, (char)243, (char)22, (char)74, (char)104, (char)139, (char)211, (char)6, (char)187, (char)133, (char)178, (char)4, (char)181, (char)194, (char)101, (char)66, (char)186, (char)196, (char)3, (char)120, (char)228, (char)191, (char)142, (char)10, (char)227, (char)188, (char)136, (char)9, (char)109, (char)82, (char)235, (char)39, (char)210, (char)239, (char)94, (char)202, (char)175, (char)245, (char)123, (char)6, (char)56, (char)157, (char)52, (char)171, (char)22, (char)138, (char)204, (char)250, (char)173, (char)219, (char)227, (char)23, (char)31, (char)151, (char)35, (char)4, (char)242, (char)196, (char)141, (char)61, (char)233, (char)179, (char)173, (char)113, (char)223}));
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.data__SET(new char[] {(char)120, (char)79, (char)183, (char)10, (char)153, (char)204, (char)164, (char)248, (char)59, (char)242, (char)178, (char)53, (char)211, (char)7, (char)165, (char)15, (char)42, (char)73, (char)235, (char)11, (char)57, (char)209, (char)161, (char)10, (char)168, (char)104, (char)42, (char)218, (char)5, (char)219, (char)1, (char)132, (char)39, (char)249, (char)189, (char)30, (char)189, (char)68, (char)47, (char)9, (char)21, (char)211, (char)54, (char)221, (char)16, (char)169, (char)61, (char)95, (char)44, (char)72, (char)212, (char)78, (char)161, (char)246, (char)248, (char)67, (char)73, (char)176, (char)188, (char)37, (char)126, (char)55, (char)239, (char)115, (char)238, (char)144, (char)87, (char)128, (char)202, (char)46, (char)182, (char)11, (char)149, (char)13, (char)228, (char)150, (char)231, (char)231, (char)105, (char)220, (char)168, (char)205, (char)156, (char)146, (char)168, (char)116, (char)215, (char)91, (char)65, (char)236, (char)15, (char)73, (char)178, (char)28, (char)98, (char)146, (char)2, (char)238, (char)19, (char)202, (char)241, (char)97, (char)2, (char)124, (char)37, (char)72, (char)13, (char)88, (char)44, (char)128, (char)6, (char)104, (char)181, (char)70, (char)72, (char)203, (char)243, (char)22, (char)74, (char)104, (char)139, (char)211, (char)6, (char)187, (char)133, (char)178, (char)4, (char)181, (char)194, (char)101, (char)66, (char)186, (char)196, (char)3, (char)120, (char)228, (char)191, (char)142, (char)10, (char)227, (char)188, (char)136, (char)9, (char)109, (char)82, (char)235, (char)39, (char)210, (char)239, (char)94, (char)202, (char)175, (char)245, (char)123, (char)6, (char)56, (char)157, (char)52, (char)171, (char)22, (char)138, (char)204, (char)250, (char)173, (char)219, (char)227, (char)23, (char)31, (char)151, (char)35, (char)4, (char)242, (char)196, (char)141, (char)61, (char)233, (char)179, (char)173, (char)113, (char)223}, 0) ;
        p233.len_SET((char)114) ;
        p233.flags_SET((char)221) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (byte)90);
            assert(pack.heading_sp_GET() == (short)27812);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            assert(pack.longitude_GET() == 1663582298);
            assert(pack.wp_distance_GET() == (char)7419);
            assert(pack.latitude_GET() == 1669464296);
            assert(pack.throttle_GET() == (byte) - 30);
            assert(pack.altitude_amsl_GET() == (short)24626);
            assert(pack.climb_rate_GET() == (byte) - 111);
            assert(pack.custom_mode_GET() == 1991262703L);
            assert(pack.battery_remaining_GET() == (char)5);
            assert(pack.airspeed_sp_GET() == (char)238);
            assert(pack.failsafe_GET() == (char)194);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
            assert(pack.heading_GET() == (char)14593);
            assert(pack.airspeed_GET() == (char)186);
            assert(pack.wp_num_GET() == (char)247);
            assert(pack.roll_GET() == (short) -210);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
            assert(pack.gps_nsat_GET() == (char)110);
            assert(pack.groundspeed_GET() == (char)247);
            assert(pack.pitch_GET() == (short)26689);
            assert(pack.temperature_air_GET() == (byte) - 95);
            assert(pack.altitude_sp_GET() == (short)14977);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.altitude_sp_SET((short)14977) ;
        p234.failsafe_SET((char)194) ;
        p234.altitude_amsl_SET((short)24626) ;
        p234.temperature_SET((byte)90) ;
        p234.pitch_SET((short)26689) ;
        p234.throttle_SET((byte) - 30) ;
        p234.temperature_air_SET((byte) - 95) ;
        p234.latitude_SET(1669464296) ;
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED)) ;
        p234.custom_mode_SET(1991262703L) ;
        p234.longitude_SET(1663582298) ;
        p234.airspeed_SET((char)186) ;
        p234.wp_num_SET((char)247) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING) ;
        p234.heading_sp_SET((short)27812) ;
        p234.wp_distance_SET((char)7419) ;
        p234.airspeed_sp_SET((char)238) ;
        p234.battery_remaining_SET((char)5) ;
        p234.heading_SET((char)14593) ;
        p234.climb_rate_SET((byte) - 111) ;
        p234.roll_SET((short) -210) ;
        p234.gps_nsat_SET((char)110) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP) ;
        p234.groundspeed_SET((char)247) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_x_GET() == -8.2619843E37F);
            assert(pack.vibration_z_GET() == 3.3140603E38F);
            assert(pack.clipping_2_GET() == 4164381611L);
            assert(pack.clipping_1_GET() == 1776163053L);
            assert(pack.clipping_0_GET() == 1197200549L);
            assert(pack.time_usec_GET() == 8115020225044750549L);
            assert(pack.vibration_y_GET() == 2.5122827E38F);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_0_SET(1197200549L) ;
        p241.vibration_y_SET(2.5122827E38F) ;
        p241.clipping_1_SET(1776163053L) ;
        p241.vibration_x_SET(-8.2619843E37F) ;
        p241.clipping_2_SET(4164381611L) ;
        p241.vibration_z_SET(3.3140603E38F) ;
        p241.time_usec_SET(8115020225044750549L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == -669291139);
            assert(pack.approach_z_GET() == 3.284356E38F);
            assert(pack.z_GET() == -1.0922334E38F);
            assert(pack.altitude_GET() == 223320697);
            assert(pack.approach_y_GET() == 9.334122E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.884998E38F, 4.2654494E36F, -1.1219136E38F, -3.31714E38F}));
            assert(pack.time_usec_TRY(ph) == 1255307748603007111L);
            assert(pack.longitude_GET() == -46340469);
            assert(pack.x_GET() == 3.1163947E38F);
            assert(pack.approach_x_GET() == 1.2591376E38F);
            assert(pack.y_GET() == -1.091969E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.approach_y_SET(9.334122E37F) ;
        p242.approach_x_SET(1.2591376E38F) ;
        p242.time_usec_SET(1255307748603007111L, PH) ;
        p242.z_SET(-1.0922334E38F) ;
        p242.approach_z_SET(3.284356E38F) ;
        p242.x_SET(3.1163947E38F) ;
        p242.longitude_SET(-46340469) ;
        p242.latitude_SET(-669291139) ;
        p242.y_SET(-1.091969E38F) ;
        p242.altitude_SET(223320697) ;
        p242.q_SET(new float[] {-2.884998E38F, 4.2654494E36F, -1.1219136E38F, -3.31714E38F}, 0) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_x_GET() == 7.275792E37F);
            assert(pack.time_usec_TRY(ph) == 3792782131720170928L);
            assert(pack.altitude_GET() == 389798128);
            assert(pack.z_GET() == -1.2296581E38F);
            assert(pack.longitude_GET() == 2133239103);
            assert(pack.y_GET() == -1.3405136E38F);
            assert(pack.latitude_GET() == 1376804836);
            assert(pack.target_system_GET() == (char)11);
            assert(pack.approach_z_GET() == -3.1187042E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.6366708E38F, -3.3309454E37F, 3.2585516E36F, -7.0217575E37F}));
            assert(pack.x_GET() == -1.2667004E38F);
            assert(pack.approach_y_GET() == -9.81526E37F);
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.z_SET(-1.2296581E38F) ;
        p243.approach_z_SET(-3.1187042E38F) ;
        p243.approach_y_SET(-9.81526E37F) ;
        p243.target_system_SET((char)11) ;
        p243.longitude_SET(2133239103) ;
        p243.altitude_SET(389798128) ;
        p243.latitude_SET(1376804836) ;
        p243.time_usec_SET(3792782131720170928L, PH) ;
        p243.approach_x_SET(7.275792E37F) ;
        p243.q_SET(new float[] {-1.6366708E38F, -3.3309454E37F, 3.2585516E36F, -7.0217575E37F}, 0) ;
        p243.x_SET(-1.2667004E38F) ;
        p243.y_SET(-1.3405136E38F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == -971396300);
            assert(pack.message_id_GET() == (char)39418);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-971396300) ;
        p244.message_id_SET((char)39418) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_MC) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.callsign_LEN(ph) == 6);
            assert(pack.callsign_TRY(ph).equals("dNgwjw"));
            assert(pack.hor_velocity_GET() == (char)35499);
            assert(pack.lat_GET() == 1269565359);
            assert(pack.tslc_GET() == (char)238);
            assert(pack.squawk_GET() == (char)10289);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED);
            assert(pack.ICAO_address_GET() == 607199198L);
            assert(pack.heading_GET() == (char)7419);
            assert(pack.ver_velocity_GET() == (short)751);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY));
            assert(pack.altitude_GET() == -1534529137);
            assert(pack.lon_GET() == -1281463981);
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.heading_SET((char)7419) ;
        p246.tslc_SET((char)238) ;
        p246.altitude_SET(-1534529137) ;
        p246.ICAO_address_SET(607199198L) ;
        p246.hor_velocity_SET((char)35499) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED) ;
        p246.lon_SET(-1281463981) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY)) ;
        p246.squawk_SET((char)10289) ;
        p246.callsign_SET("dNgwjw", PH) ;
        p246.lat_SET(1269565359) ;
        p246.ver_velocity_SET((short)751) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == 529528181L);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
            assert(pack.altitude_minimum_delta_GET() == -2.3588224E37F);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
            assert(pack.time_to_minimum_delta_GET() == -2.0514258E38F);
            assert(pack.horizontal_minimum_delta_GET() == 3.2906245E37F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.id_SET(529528181L) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR) ;
        p247.horizontal_minimum_delta_SET(3.2906245E37F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.time_to_minimum_delta_SET(-2.0514258E38F) ;
        p247.altitude_minimum_delta_SET(-2.3588224E37F) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)135);
            assert(pack.message_type_GET() == (char)17312);
            assert(pack.target_component_GET() == (char)101);
            assert(pack.target_network_GET() == (char)144);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)2, (char)82, (char)136, (char)140, (char)251, (char)135, (char)107, (char)231, (char)207, (char)66, (char)102, (char)68, (char)163, (char)27, (char)111, (char)197, (char)3, (char)249, (char)81, (char)208, (char)206, (char)112, (char)88, (char)16, (char)41, (char)219, (char)170, (char)78, (char)3, (char)161, (char)141, (char)187, (char)189, (char)136, (char)222, (char)201, (char)187, (char)150, (char)152, (char)63, (char)21, (char)122, (char)83, (char)112, (char)71, (char)116, (char)211, (char)105, (char)115, (char)147, (char)249, (char)33, (char)129, (char)204, (char)118, (char)217, (char)9, (char)245, (char)80, (char)217, (char)149, (char)69, (char)82, (char)147, (char)51, (char)100, (char)77, (char)114, (char)240, (char)93, (char)157, (char)163, (char)161, (char)83, (char)80, (char)47, (char)21, (char)21, (char)21, (char)239, (char)39, (char)243, (char)21, (char)204, (char)197, (char)250, (char)65, (char)126, (char)82, (char)141, (char)60, (char)161, (char)141, (char)96, (char)197, (char)196, (char)84, (char)197, (char)120, (char)133, (char)228, (char)156, (char)101, (char)19, (char)36, (char)248, (char)199, (char)98, (char)210, (char)152, (char)78, (char)104, (char)237, (char)51, (char)2, (char)30, (char)131, (char)1, (char)117, (char)227, (char)86, (char)201, (char)60, (char)59, (char)83, (char)193, (char)56, (char)106, (char)247, (char)152, (char)163, (char)187, (char)93, (char)45, (char)151, (char)244, (char)137, (char)130, (char)200, (char)125, (char)144, (char)102, (char)244, (char)249, (char)106, (char)14, (char)210, (char)116, (char)64, (char)196, (char)43, (char)98, (char)77, (char)80, (char)34, (char)74, (char)247, (char)164, (char)2, (char)123, (char)105, (char)76, (char)209, (char)11, (char)4, (char)249, (char)182, (char)250, (char)27, (char)19, (char)195, (char)243, (char)137, (char)90, (char)202, (char)224, (char)20, (char)233, (char)36, (char)245, (char)52, (char)25, (char)216, (char)199, (char)253, (char)238, (char)168, (char)214, (char)190, (char)190, (char)43, (char)217, (char)155, (char)1, (char)241, (char)93, (char)238, (char)149, (char)139, (char)200, (char)207, (char)32, (char)146, (char)223, (char)198, (char)163, (char)61, (char)87, (char)232, (char)149, (char)57, (char)156, (char)134, (char)218, (char)107, (char)18, (char)38, (char)48, (char)162, (char)202, (char)81, (char)36, (char)43, (char)119, (char)134, (char)142, (char)253, (char)88, (char)195, (char)216, (char)186, (char)173, (char)53, (char)196, (char)104, (char)160, (char)243, (char)118, (char)123, (char)126, (char)156, (char)212, (char)178, (char)31, (char)108, (char)9, (char)82, (char)159, (char)184}));
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.payload_SET(new char[] {(char)2, (char)82, (char)136, (char)140, (char)251, (char)135, (char)107, (char)231, (char)207, (char)66, (char)102, (char)68, (char)163, (char)27, (char)111, (char)197, (char)3, (char)249, (char)81, (char)208, (char)206, (char)112, (char)88, (char)16, (char)41, (char)219, (char)170, (char)78, (char)3, (char)161, (char)141, (char)187, (char)189, (char)136, (char)222, (char)201, (char)187, (char)150, (char)152, (char)63, (char)21, (char)122, (char)83, (char)112, (char)71, (char)116, (char)211, (char)105, (char)115, (char)147, (char)249, (char)33, (char)129, (char)204, (char)118, (char)217, (char)9, (char)245, (char)80, (char)217, (char)149, (char)69, (char)82, (char)147, (char)51, (char)100, (char)77, (char)114, (char)240, (char)93, (char)157, (char)163, (char)161, (char)83, (char)80, (char)47, (char)21, (char)21, (char)21, (char)239, (char)39, (char)243, (char)21, (char)204, (char)197, (char)250, (char)65, (char)126, (char)82, (char)141, (char)60, (char)161, (char)141, (char)96, (char)197, (char)196, (char)84, (char)197, (char)120, (char)133, (char)228, (char)156, (char)101, (char)19, (char)36, (char)248, (char)199, (char)98, (char)210, (char)152, (char)78, (char)104, (char)237, (char)51, (char)2, (char)30, (char)131, (char)1, (char)117, (char)227, (char)86, (char)201, (char)60, (char)59, (char)83, (char)193, (char)56, (char)106, (char)247, (char)152, (char)163, (char)187, (char)93, (char)45, (char)151, (char)244, (char)137, (char)130, (char)200, (char)125, (char)144, (char)102, (char)244, (char)249, (char)106, (char)14, (char)210, (char)116, (char)64, (char)196, (char)43, (char)98, (char)77, (char)80, (char)34, (char)74, (char)247, (char)164, (char)2, (char)123, (char)105, (char)76, (char)209, (char)11, (char)4, (char)249, (char)182, (char)250, (char)27, (char)19, (char)195, (char)243, (char)137, (char)90, (char)202, (char)224, (char)20, (char)233, (char)36, (char)245, (char)52, (char)25, (char)216, (char)199, (char)253, (char)238, (char)168, (char)214, (char)190, (char)190, (char)43, (char)217, (char)155, (char)1, (char)241, (char)93, (char)238, (char)149, (char)139, (char)200, (char)207, (char)32, (char)146, (char)223, (char)198, (char)163, (char)61, (char)87, (char)232, (char)149, (char)57, (char)156, (char)134, (char)218, (char)107, (char)18, (char)38, (char)48, (char)162, (char)202, (char)81, (char)36, (char)43, (char)119, (char)134, (char)142, (char)253, (char)88, (char)195, (char)216, (char)186, (char)173, (char)53, (char)196, (char)104, (char)160, (char)243, (char)118, (char)123, (char)126, (char)156, (char)212, (char)178, (char)31, (char)108, (char)9, (char)82, (char)159, (char)184}, 0) ;
        p248.target_network_SET((char)144) ;
        p248.target_component_SET((char)101) ;
        p248.message_type_SET((char)17312) ;
        p248.target_system_SET((char)135) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.ver_GET() == (char)228);
            assert(pack.type_GET() == (char)79);
            assert(pack.address_GET() == (char)64292);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 118, (byte)74, (byte)120, (byte) - 38, (byte)17, (byte)23, (byte)80, (byte)8, (byte) - 33, (byte)3, (byte) - 121, (byte)126, (byte)20, (byte) - 85, (byte) - 72, (byte)110, (byte) - 64, (byte) - 54, (byte)19, (byte)15, (byte)24, (byte) - 21, (byte)9, (byte) - 76, (byte) - 90, (byte)55, (byte)27, (byte) - 84, (byte) - 99, (byte)81, (byte)17, (byte)8}));
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)64292) ;
        p249.type_SET((char)79) ;
        p249.ver_SET((char)228) ;
        p249.value_SET(new byte[] {(byte) - 118, (byte)74, (byte)120, (byte) - 38, (byte)17, (byte)23, (byte)80, (byte)8, (byte) - 33, (byte)3, (byte) - 121, (byte)126, (byte)20, (byte) - 85, (byte) - 72, (byte)110, (byte) - 64, (byte) - 54, (byte)19, (byte)15, (byte)24, (byte) - 21, (byte)9, (byte) - 76, (byte) - 90, (byte)55, (byte)27, (byte) - 84, (byte) - 99, (byte)81, (byte)17, (byte)8}, 0) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 3.3685176E38F);
            assert(pack.name_LEN(ph) == 8);
            assert(pack.name_TRY(ph).equals("JXpjkOko"));
            assert(pack.time_usec_GET() == 7574756635302884775L);
            assert(pack.y_GET() == -1.7324496E38F);
            assert(pack.x_GET() == 2.4990853E38F);
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.y_SET(-1.7324496E38F) ;
        p250.z_SET(3.3685176E38F) ;
        p250.name_SET("JXpjkOko", PH) ;
        p250.x_SET(2.4990853E38F) ;
        p250.time_usec_SET(7574756635302884775L) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -9.564705E37F);
            assert(pack.time_boot_ms_GET() == 516937271L);
            assert(pack.name_LEN(ph) == 8);
            assert(pack.name_TRY(ph).equals("Obermcvn"));
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(-9.564705E37F) ;
        p251.time_boot_ms_SET(516937271L) ;
        p251.name_SET("Obermcvn", PH) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 755681272L);
            assert(pack.value_GET() == 222143509);
            assert(pack.name_LEN(ph) == 2);
            assert(pack.name_TRY(ph).equals("us"));
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.value_SET(222143509) ;
        p252.name_SET("us", PH) ;
        p252.time_boot_ms_SET(755681272L) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 34);
            assert(pack.text_TRY(ph).equals("puRcchnqynfjyfbbuvjvuioeiyutvRqirO"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_NOTICE);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("puRcchnqynfjyfbbuvjvuioeiyutvRqirO", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_NOTICE) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.ind_GET() == (char)54);
            assert(pack.value_GET() == 1.2821733E38F);
            assert(pack.time_boot_ms_GET() == 4026571969L);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.ind_SET((char)54) ;
        p254.value_SET(1.2821733E38F) ;
        p254.time_boot_ms_SET(4026571969L) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.initial_timestamp_GET() == 2322843597332192970L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)241, (char)122, (char)125, (char)75, (char)228, (char)193, (char)177, (char)148, (char)161, (char)59, (char)33, (char)0, (char)199, (char)65, (char)40, (char)101, (char)186, (char)147, (char)147, (char)73, (char)221, (char)199, (char)255, (char)137, (char)42, (char)77, (char)7, (char)94, (char)31, (char)137, (char)126, (char)196}));
            assert(pack.target_system_GET() == (char)138);
            assert(pack.target_component_GET() == (char)176);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.initial_timestamp_SET(2322843597332192970L) ;
        p256.secret_key_SET(new char[] {(char)241, (char)122, (char)125, (char)75, (char)228, (char)193, (char)177, (char)148, (char)161, (char)59, (char)33, (char)0, (char)199, (char)65, (char)40, (char)101, (char)186, (char)147, (char)147, (char)73, (char)221, (char)199, (char)255, (char)137, (char)42, (char)77, (char)7, (char)94, (char)31, (char)137, (char)126, (char)196}, 0) ;
        p256.target_component_SET((char)176) ;
        p256.target_system_SET((char)138) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1807327178L);
            assert(pack.state_GET() == (char)212);
            assert(pack.last_change_ms_GET() == 2184329310L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.state_SET((char)212) ;
        p257.last_change_ms_SET(2184329310L) ;
        p257.time_boot_ms_SET(1807327178L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.tune_LEN(ph) == 9);
            assert(pack.tune_TRY(ph).equals("cZuqjtdve"));
            assert(pack.target_component_GET() == (char)99);
            assert(pack.target_system_GET() == (char)229);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)229) ;
        p258.tune_SET("cZuqjtdve", PH) ;
        p258.target_component_SET((char)99) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)162, (char)236, (char)104, (char)94, (char)40, (char)147, (char)207, (char)181, (char)35, (char)149, (char)10, (char)46, (char)163, (char)245, (char)129, (char)105, (char)15, (char)18, (char)127, (char)153, (char)31, (char)43, (char)140, (char)235, (char)144, (char)214, (char)228, (char)154, (char)15, (char)212, (char)21, (char)120}));
            assert(pack.firmware_version_GET() == 3951596096L);
            assert(pack.resolution_h_GET() == (char)15429);
            assert(pack.sensor_size_v_GET() == -3.1062673E38F);
            assert(pack.focal_length_GET() == -8.646108E37F);
            assert(pack.lens_id_GET() == (char)106);
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES));
            assert(pack.time_boot_ms_GET() == 972100382L);
            assert(pack.cam_definition_uri_LEN(ph) == 87);
            assert(pack.cam_definition_uri_TRY(ph).equals("pjjhwyvpriibnikunftBvxgslunhhtxwjkbjijarQcqjpbspbzhwubeHCumkatOriNrexeqczrwuvoIesVnptef"));
            assert(pack.cam_definition_version_GET() == (char)59491);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)126, (char)77, (char)98, (char)227, (char)146, (char)100, (char)34, (char)151, (char)171, (char)176, (char)36, (char)108, (char)25, (char)193, (char)165, (char)14, (char)242, (char)94, (char)229, (char)97, (char)118, (char)17, (char)35, (char)149, (char)189, (char)254, (char)28, (char)217, (char)193, (char)176, (char)95, (char)102}));
            assert(pack.resolution_v_GET() == (char)14640);
            assert(pack.sensor_size_h_GET() == 2.570651E38F);
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.focal_length_SET(-8.646108E37F) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES)) ;
        p259.resolution_v_SET((char)14640) ;
        p259.cam_definition_uri_SET("pjjhwyvpriibnikunftBvxgslunhhtxwjkbjijarQcqjpbspbzhwubeHCumkatOriNrexeqczrwuvoIesVnptef", PH) ;
        p259.firmware_version_SET(3951596096L) ;
        p259.resolution_h_SET((char)15429) ;
        p259.time_boot_ms_SET(972100382L) ;
        p259.vendor_name_SET(new char[] {(char)126, (char)77, (char)98, (char)227, (char)146, (char)100, (char)34, (char)151, (char)171, (char)176, (char)36, (char)108, (char)25, (char)193, (char)165, (char)14, (char)242, (char)94, (char)229, (char)97, (char)118, (char)17, (char)35, (char)149, (char)189, (char)254, (char)28, (char)217, (char)193, (char)176, (char)95, (char)102}, 0) ;
        p259.cam_definition_version_SET((char)59491) ;
        p259.model_name_SET(new char[] {(char)162, (char)236, (char)104, (char)94, (char)40, (char)147, (char)207, (char)181, (char)35, (char)149, (char)10, (char)46, (char)163, (char)245, (char)129, (char)105, (char)15, (char)18, (char)127, (char)153, (char)31, (char)43, (char)140, (char)235, (char)144, (char)214, (char)228, (char)154, (char)15, (char)212, (char)21, (char)120}, 0) ;
        p259.sensor_size_h_SET(2.570651E38F) ;
        p259.sensor_size_v_SET(-3.1062673E38F) ;
        p259.lens_id_SET((char)106) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
            assert(pack.time_boot_ms_GET() == 1518023421L);
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY) ;
        p260.time_boot_ms_SET(1518023421L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.read_speed_GET() == 2.8493883E38F);
            assert(pack.time_boot_ms_GET() == 1710378443L);
            assert(pack.used_capacity_GET() == -1.2922566E38F);
            assert(pack.available_capacity_GET() == 2.9744529E38F);
            assert(pack.write_speed_GET() == 2.2965219E38F);
            assert(pack.storage_id_GET() == (char)62);
            assert(pack.total_capacity_GET() == 7.1139365E37F);
            assert(pack.storage_count_GET() == (char)213);
            assert(pack.status_GET() == (char)226);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.storage_count_SET((char)213) ;
        p261.total_capacity_SET(7.1139365E37F) ;
        p261.status_SET((char)226) ;
        p261.write_speed_SET(2.2965219E38F) ;
        p261.read_speed_SET(2.8493883E38F) ;
        p261.used_capacity_SET(-1.2922566E38F) ;
        p261.time_boot_ms_SET(1710378443L) ;
        p261.available_capacity_SET(2.9744529E38F) ;
        p261.storage_id_SET((char)62) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3560823440L);
            assert(pack.available_capacity_GET() == 9.814424E37F);
            assert(pack.recording_time_ms_GET() == 830470473L);
            assert(pack.image_interval_GET() == -1.6414966E38F);
            assert(pack.image_status_GET() == (char)197);
            assert(pack.video_status_GET() == (char)44);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.image_interval_SET(-1.6414966E38F) ;
        p262.time_boot_ms_SET(3560823440L) ;
        p262.video_status_SET((char)44) ;
        p262.recording_time_ms_SET(830470473L) ;
        p262.available_capacity_SET(9.814424E37F) ;
        p262.image_status_SET((char)197) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.image_index_GET() == -1081613405);
            assert(pack.alt_GET() == -68153297);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.2672364E38F, -3.2731778E36F, 9.898202E37F, 1.8698995E37F}));
            assert(pack.lat_GET() == -62849345);
            assert(pack.time_utc_GET() == 47579750276850630L);
            assert(pack.capture_result_GET() == (byte)108);
            assert(pack.camera_id_GET() == (char)205);
            assert(pack.lon_GET() == -2020759417);
            assert(pack.relative_alt_GET() == 1162907088);
            assert(pack.time_boot_ms_GET() == 1758300389L);
            assert(pack.file_url_LEN(ph) == 115);
            assert(pack.file_url_TRY(ph).equals("tMtoldzkmloegRqxRbilqqJKsRGbxwtsdlKjxYiqqiqmmjschovygoueoKswfXtdagkeXptbZdGrymjwiosiofwyejcnBawyolpldhkvfvajmppknht"));
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.relative_alt_SET(1162907088) ;
        p263.alt_SET(-68153297) ;
        p263.file_url_SET("tMtoldzkmloegRqxRbilqqJKsRGbxwtsdlKjxYiqqiqmmjschovygoueoKswfXtdagkeXptbZdGrymjwiosiofwyejcnBawyolpldhkvfvajmppknht", PH) ;
        p263.time_boot_ms_SET(1758300389L) ;
        p263.capture_result_SET((byte)108) ;
        p263.lat_SET(-62849345) ;
        p263.lon_SET(-2020759417) ;
        p263.time_utc_SET(47579750276850630L) ;
        p263.camera_id_SET((char)205) ;
        p263.image_index_SET(-1081613405) ;
        p263.q_SET(new float[] {3.2672364E38F, -3.2731778E36F, 9.898202E37F, 1.8698995E37F}, 0) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.flight_uuid_GET() == 3237154953519047770L);
            assert(pack.time_boot_ms_GET() == 401754147L);
            assert(pack.arming_time_utc_GET() == 222352036438010019L);
            assert(pack.takeoff_time_utc_GET() == 7222589623588993877L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.takeoff_time_utc_SET(7222589623588993877L) ;
        p264.arming_time_utc_SET(222352036438010019L) ;
        p264.time_boot_ms_SET(401754147L) ;
        p264.flight_uuid_SET(3237154953519047770L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 9.112615E37F);
            assert(pack.time_boot_ms_GET() == 2439398236L);
            assert(pack.pitch_GET() == 1.3012788E38F);
            assert(pack.roll_GET() == 2.981506E38F);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(2.981506E38F) ;
        p265.pitch_SET(1.3012788E38F) ;
        p265.time_boot_ms_SET(2439398236L) ;
        p265.yaw_SET(9.112615E37F) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.length_GET() == (char)206);
            assert(pack.target_system_GET() == (char)63);
            assert(pack.target_component_GET() == (char)197);
            assert(pack.sequence_GET() == (char)16803);
            assert(pack.first_message_offset_GET() == (char)18);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)218, (char)123, (char)104, (char)147, (char)96, (char)230, (char)129, (char)13, (char)96, (char)177, (char)141, (char)161, (char)3, (char)239, (char)207, (char)142, (char)116, (char)214, (char)78, (char)139, (char)127, (char)16, (char)136, (char)255, (char)140, (char)198, (char)149, (char)253, (char)94, (char)141, (char)34, (char)152, (char)151, (char)186, (char)121, (char)131, (char)101, (char)176, (char)143, (char)65, (char)155, (char)23, (char)144, (char)119, (char)126, (char)98, (char)132, (char)109, (char)103, (char)223, (char)16, (char)113, (char)28, (char)192, (char)46, (char)255, (char)43, (char)135, (char)87, (char)197, (char)93, (char)84, (char)134, (char)132, (char)61, (char)194, (char)0, (char)214, (char)91, (char)135, (char)105, (char)99, (char)169, (char)172, (char)155, (char)88, (char)250, (char)245, (char)92, (char)36, (char)208, (char)219, (char)233, (char)115, (char)101, (char)57, (char)146, (char)136, (char)5, (char)13, (char)197, (char)237, (char)51, (char)143, (char)192, (char)190, (char)30, (char)238, (char)34, (char)231, (char)242, (char)119, (char)244, (char)57, (char)7, (char)41, (char)207, (char)28, (char)233, (char)32, (char)47, (char)67, (char)64, (char)134, (char)56, (char)136, (char)127, (char)15, (char)2, (char)119, (char)199, (char)7, (char)171, (char)187, (char)233, (char)110, (char)243, (char)106, (char)167, (char)167, (char)5, (char)214, (char)95, (char)64, (char)172, (char)124, (char)196, (char)131, (char)24, (char)70, (char)22, (char)205, (char)71, (char)178, (char)239, (char)119, (char)55, (char)38, (char)59, (char)249, (char)252, (char)47, (char)163, (char)101, (char)47, (char)61, (char)64, (char)198, (char)124, (char)100, (char)135, (char)167, (char)162, (char)192, (char)11, (char)162, (char)143, (char)215, (char)196, (char)76, (char)23, (char)189, (char)170, (char)16, (char)223, (char)71, (char)174, (char)254, (char)145, (char)171, (char)129, (char)157, (char)156, (char)65, (char)102, (char)120, (char)46, (char)254, (char)159, (char)23, (char)100, (char)31, (char)68, (char)193, (char)100, (char)189, (char)37, (char)35, (char)136, (char)245, (char)18, (char)118, (char)20, (char)8, (char)156, (char)140, (char)30, (char)163, (char)52, (char)224, (char)5, (char)50, (char)41, (char)194, (char)74, (char)60, (char)223, (char)80, (char)90, (char)98, (char)71, (char)111, (char)8, (char)47, (char)173, (char)177, (char)49, (char)57, (char)211, (char)36, (char)120, (char)90, (char)250, (char)43, (char)228, (char)182, (char)67, (char)147, (char)106, (char)177, (char)147, (char)60, (char)240, (char)208, (char)119, (char)63, (char)217, (char)42, (char)161}));
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_component_SET((char)197) ;
        p266.target_system_SET((char)63) ;
        p266.data__SET(new char[] {(char)218, (char)123, (char)104, (char)147, (char)96, (char)230, (char)129, (char)13, (char)96, (char)177, (char)141, (char)161, (char)3, (char)239, (char)207, (char)142, (char)116, (char)214, (char)78, (char)139, (char)127, (char)16, (char)136, (char)255, (char)140, (char)198, (char)149, (char)253, (char)94, (char)141, (char)34, (char)152, (char)151, (char)186, (char)121, (char)131, (char)101, (char)176, (char)143, (char)65, (char)155, (char)23, (char)144, (char)119, (char)126, (char)98, (char)132, (char)109, (char)103, (char)223, (char)16, (char)113, (char)28, (char)192, (char)46, (char)255, (char)43, (char)135, (char)87, (char)197, (char)93, (char)84, (char)134, (char)132, (char)61, (char)194, (char)0, (char)214, (char)91, (char)135, (char)105, (char)99, (char)169, (char)172, (char)155, (char)88, (char)250, (char)245, (char)92, (char)36, (char)208, (char)219, (char)233, (char)115, (char)101, (char)57, (char)146, (char)136, (char)5, (char)13, (char)197, (char)237, (char)51, (char)143, (char)192, (char)190, (char)30, (char)238, (char)34, (char)231, (char)242, (char)119, (char)244, (char)57, (char)7, (char)41, (char)207, (char)28, (char)233, (char)32, (char)47, (char)67, (char)64, (char)134, (char)56, (char)136, (char)127, (char)15, (char)2, (char)119, (char)199, (char)7, (char)171, (char)187, (char)233, (char)110, (char)243, (char)106, (char)167, (char)167, (char)5, (char)214, (char)95, (char)64, (char)172, (char)124, (char)196, (char)131, (char)24, (char)70, (char)22, (char)205, (char)71, (char)178, (char)239, (char)119, (char)55, (char)38, (char)59, (char)249, (char)252, (char)47, (char)163, (char)101, (char)47, (char)61, (char)64, (char)198, (char)124, (char)100, (char)135, (char)167, (char)162, (char)192, (char)11, (char)162, (char)143, (char)215, (char)196, (char)76, (char)23, (char)189, (char)170, (char)16, (char)223, (char)71, (char)174, (char)254, (char)145, (char)171, (char)129, (char)157, (char)156, (char)65, (char)102, (char)120, (char)46, (char)254, (char)159, (char)23, (char)100, (char)31, (char)68, (char)193, (char)100, (char)189, (char)37, (char)35, (char)136, (char)245, (char)18, (char)118, (char)20, (char)8, (char)156, (char)140, (char)30, (char)163, (char)52, (char)224, (char)5, (char)50, (char)41, (char)194, (char)74, (char)60, (char)223, (char)80, (char)90, (char)98, (char)71, (char)111, (char)8, (char)47, (char)173, (char)177, (char)49, (char)57, (char)211, (char)36, (char)120, (char)90, (char)250, (char)43, (char)228, (char)182, (char)67, (char)147, (char)106, (char)177, (char)147, (char)60, (char)240, (char)208, (char)119, (char)63, (char)217, (char)42, (char)161}, 0) ;
        p266.sequence_SET((char)16803) ;
        p266.length_SET((char)206) ;
        p266.first_message_offset_SET((char)18) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)44427);
            assert(pack.target_system_GET() == (char)161);
            assert(pack.length_GET() == (char)1);
            assert(pack.first_message_offset_GET() == (char)170);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)81, (char)61, (char)228, (char)74, (char)146, (char)39, (char)216, (char)181, (char)165, (char)177, (char)238, (char)125, (char)12, (char)193, (char)184, (char)118, (char)71, (char)78, (char)239, (char)100, (char)144, (char)21, (char)223, (char)247, (char)212, (char)173, (char)184, (char)136, (char)54, (char)236, (char)20, (char)222, (char)36, (char)36, (char)98, (char)148, (char)157, (char)13, (char)216, (char)93, (char)12, (char)188, (char)22, (char)137, (char)7, (char)122, (char)201, (char)36, (char)102, (char)250, (char)192, (char)5, (char)202, (char)219, (char)218, (char)218, (char)202, (char)143, (char)74, (char)210, (char)18, (char)73, (char)122, (char)95, (char)171, (char)50, (char)40, (char)37, (char)19, (char)252, (char)212, (char)19, (char)118, (char)178, (char)236, (char)117, (char)229, (char)39, (char)109, (char)157, (char)41, (char)19, (char)243, (char)196, (char)27, (char)149, (char)143, (char)49, (char)150, (char)27, (char)125, (char)244, (char)196, (char)210, (char)110, (char)232, (char)86, (char)187, (char)8, (char)144, (char)171, (char)207, (char)244, (char)163, (char)188, (char)106, (char)193, (char)141, (char)218, (char)204, (char)68, (char)16, (char)145, (char)127, (char)153, (char)13, (char)161, (char)108, (char)219, (char)24, (char)201, (char)89, (char)33, (char)42, (char)132, (char)209, (char)247, (char)161, (char)210, (char)112, (char)29, (char)168, (char)168, (char)133, (char)37, (char)52, (char)200, (char)73, (char)248, (char)72, (char)230, (char)229, (char)92, (char)11, (char)74, (char)220, (char)131, (char)29, (char)112, (char)223, (char)225, (char)108, (char)115, (char)54, (char)139, (char)74, (char)128, (char)120, (char)254, (char)203, (char)19, (char)111, (char)159, (char)150, (char)153, (char)119, (char)145, (char)48, (char)176, (char)141, (char)231, (char)231, (char)122, (char)178, (char)97, (char)99, (char)231, (char)86, (char)92, (char)216, (char)110, (char)1, (char)161, (char)50, (char)212, (char)190, (char)173, (char)150, (char)64, (char)178, (char)132, (char)229, (char)73, (char)66, (char)177, (char)24, (char)232, (char)215, (char)228, (char)151, (char)185, (char)52, (char)69, (char)170, (char)30, (char)226, (char)242, (char)74, (char)213, (char)111, (char)78, (char)4, (char)135, (char)209, (char)161, (char)90, (char)243, (char)82, (char)249, (char)99, (char)69, (char)34, (char)108, (char)84, (char)16, (char)151, (char)39, (char)201, (char)163, (char)95, (char)79, (char)158, (char)2, (char)54, (char)108, (char)130, (char)223, (char)104, (char)243, (char)77, (char)3, (char)223, (char)205, (char)38, (char)216, (char)112, (char)244, (char)165, (char)45}));
            assert(pack.target_component_GET() == (char)196);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_component_SET((char)196) ;
        p267.length_SET((char)1) ;
        p267.first_message_offset_SET((char)170) ;
        p267.sequence_SET((char)44427) ;
        p267.data__SET(new char[] {(char)81, (char)61, (char)228, (char)74, (char)146, (char)39, (char)216, (char)181, (char)165, (char)177, (char)238, (char)125, (char)12, (char)193, (char)184, (char)118, (char)71, (char)78, (char)239, (char)100, (char)144, (char)21, (char)223, (char)247, (char)212, (char)173, (char)184, (char)136, (char)54, (char)236, (char)20, (char)222, (char)36, (char)36, (char)98, (char)148, (char)157, (char)13, (char)216, (char)93, (char)12, (char)188, (char)22, (char)137, (char)7, (char)122, (char)201, (char)36, (char)102, (char)250, (char)192, (char)5, (char)202, (char)219, (char)218, (char)218, (char)202, (char)143, (char)74, (char)210, (char)18, (char)73, (char)122, (char)95, (char)171, (char)50, (char)40, (char)37, (char)19, (char)252, (char)212, (char)19, (char)118, (char)178, (char)236, (char)117, (char)229, (char)39, (char)109, (char)157, (char)41, (char)19, (char)243, (char)196, (char)27, (char)149, (char)143, (char)49, (char)150, (char)27, (char)125, (char)244, (char)196, (char)210, (char)110, (char)232, (char)86, (char)187, (char)8, (char)144, (char)171, (char)207, (char)244, (char)163, (char)188, (char)106, (char)193, (char)141, (char)218, (char)204, (char)68, (char)16, (char)145, (char)127, (char)153, (char)13, (char)161, (char)108, (char)219, (char)24, (char)201, (char)89, (char)33, (char)42, (char)132, (char)209, (char)247, (char)161, (char)210, (char)112, (char)29, (char)168, (char)168, (char)133, (char)37, (char)52, (char)200, (char)73, (char)248, (char)72, (char)230, (char)229, (char)92, (char)11, (char)74, (char)220, (char)131, (char)29, (char)112, (char)223, (char)225, (char)108, (char)115, (char)54, (char)139, (char)74, (char)128, (char)120, (char)254, (char)203, (char)19, (char)111, (char)159, (char)150, (char)153, (char)119, (char)145, (char)48, (char)176, (char)141, (char)231, (char)231, (char)122, (char)178, (char)97, (char)99, (char)231, (char)86, (char)92, (char)216, (char)110, (char)1, (char)161, (char)50, (char)212, (char)190, (char)173, (char)150, (char)64, (char)178, (char)132, (char)229, (char)73, (char)66, (char)177, (char)24, (char)232, (char)215, (char)228, (char)151, (char)185, (char)52, (char)69, (char)170, (char)30, (char)226, (char)242, (char)74, (char)213, (char)111, (char)78, (char)4, (char)135, (char)209, (char)161, (char)90, (char)243, (char)82, (char)249, (char)99, (char)69, (char)34, (char)108, (char)84, (char)16, (char)151, (char)39, (char)201, (char)163, (char)95, (char)79, (char)158, (char)2, (char)54, (char)108, (char)130, (char)223, (char)104, (char)243, (char)77, (char)3, (char)223, (char)205, (char)38, (char)216, (char)112, (char)244, (char)165, (char)45}, 0) ;
        p267.target_system_SET((char)161) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)53584);
            assert(pack.target_component_GET() == (char)33);
            assert(pack.target_system_GET() == (char)224);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.sequence_SET((char)53584) ;
        p268.target_system_SET((char)224) ;
        p268.target_component_SET((char)33) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.bitrate_GET() == 3108223431L);
            assert(pack.framerate_GET() == -1.6415268E37F);
            assert(pack.resolution_v_GET() == (char)59604);
            assert(pack.status_GET() == (char)229);
            assert(pack.camera_id_GET() == (char)100);
            assert(pack.rotation_GET() == (char)41608);
            assert(pack.resolution_h_GET() == (char)39869);
            assert(pack.uri_LEN(ph) == 93);
            assert(pack.uri_TRY(ph).equals("arfaBpdjWuairvkgjLsrWnibvqmjCvqtthpynuzoqeotdyatyuvyrmbmuskjitwlxymhsHwVbxepvqmcnxvdroqhPvvbl"));
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.framerate_SET(-1.6415268E37F) ;
        p269.status_SET((char)229) ;
        p269.bitrate_SET(3108223431L) ;
        p269.resolution_v_SET((char)59604) ;
        p269.camera_id_SET((char)100) ;
        p269.rotation_SET((char)41608) ;
        p269.uri_SET("arfaBpdjWuairvkgjLsrWnibvqmjCvqtthpynuzoqeotdyatyuvyrmbmuskjitwlxymhsHwVbxepvqmcnxvdroqhPvvbl", PH) ;
        p269.resolution_h_SET((char)39869) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)105);
            assert(pack.uri_LEN(ph) == 53);
            assert(pack.uri_TRY(ph).equals("honojvhxnoEuFmlyjntuxmldqnpmdraluzeqTizocsmuvhhddWkea"));
            assert(pack.bitrate_GET() == 571585280L);
            assert(pack.camera_id_GET() == (char)64);
            assert(pack.resolution_v_GET() == (char)39691);
            assert(pack.resolution_h_GET() == (char)62851);
            assert(pack.framerate_GET() == 1.0792432E38F);
            assert(pack.rotation_GET() == (char)17799);
            assert(pack.target_component_GET() == (char)249);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.rotation_SET((char)17799) ;
        p270.resolution_v_SET((char)39691) ;
        p270.target_system_SET((char)105) ;
        p270.bitrate_SET(571585280L) ;
        p270.uri_SET("honojvhxnoEuFmlyjntuxmldqnpmdraluzeqTizocsmuvhhddWkea", PH) ;
        p270.resolution_h_SET((char)62851) ;
        p270.camera_id_SET((char)64) ;
        p270.framerate_SET(1.0792432E38F) ;
        p270.target_component_SET((char)249) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 2);
            assert(pack.ssid_TRY(ph).equals("et"));
            assert(pack.password_LEN(ph) == 32);
            assert(pack.password_TRY(ph).equals("tuqbuIofTfxxhreaxrszSgbyhdvnwxIq"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("tuqbuIofTfxxhreaxrszSgbyhdvnwxIq", PH) ;
        p299.ssid_SET("et", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.max_version_GET() == (char)57052);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)228, (char)26, (char)189, (char)255, (char)68, (char)145, (char)203, (char)212}));
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)222, (char)61, (char)189, (char)165, (char)225, (char)109, (char)112, (char)76}));
            assert(pack.min_version_GET() == (char)21649);
            assert(pack.version_GET() == (char)18442);
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.spec_version_hash_SET(new char[] {(char)222, (char)61, (char)189, (char)165, (char)225, (char)109, (char)112, (char)76}, 0) ;
        p300.max_version_SET((char)57052) ;
        p300.library_version_hash_SET(new char[] {(char)228, (char)26, (char)189, (char)255, (char)68, (char)145, (char)203, (char)212}, 0) ;
        p300.version_SET((char)18442) ;
        p300.min_version_SET((char)21649) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
            assert(pack.vendor_specific_status_code_GET() == (char)33347);
            assert(pack.time_usec_GET() == 5799037016306676164L);
            assert(pack.uptime_sec_GET() == 2624090096L);
            assert(pack.sub_mode_GET() == (char)211);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(5799037016306676164L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION) ;
        p310.vendor_specific_status_code_SET((char)33347) ;
        p310.uptime_sec_SET(2624090096L) ;
        p310.sub_mode_SET((char)211) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.sw_version_major_GET() == (char)60);
            assert(pack.hw_version_minor_GET() == (char)117);
            assert(pack.sw_version_minor_GET() == (char)220);
            assert(pack.uptime_sec_GET() == 2783095096L);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)142, (char)157, (char)219, (char)223, (char)192, (char)254, (char)222, (char)97, (char)155, (char)14, (char)179, (char)90, (char)109, (char)111, (char)40, (char)122}));
            assert(pack.hw_version_major_GET() == (char)145);
            assert(pack.sw_vcs_commit_GET() == 646529324L);
            assert(pack.time_usec_GET() == 2420520992261356454L);
            assert(pack.name_LEN(ph) == 42);
            assert(pack.name_TRY(ph).equals("giasltFxxsgxtrgBdecpmpvvpysPmpnpajdBvPwJmq"));
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_version_major_SET((char)145) ;
        p311.name_SET("giasltFxxsgxtrgBdecpmpvvpysPmpnpajdBvPwJmq", PH) ;
        p311.sw_vcs_commit_SET(646529324L) ;
        p311.hw_version_minor_SET((char)117) ;
        p311.sw_version_minor_SET((char)220) ;
        p311.time_usec_SET(2420520992261356454L) ;
        p311.hw_unique_id_SET(new char[] {(char)142, (char)157, (char)219, (char)223, (char)192, (char)254, (char)222, (char)97, (char)155, (char)14, (char)179, (char)90, (char)109, (char)111, (char)40, (char)122}, 0) ;
        p311.uptime_sec_SET(2783095096L) ;
        p311.sw_version_major_SET((char)60) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)137);
            assert(pack.target_system_GET() == (char)242);
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("diRtstfucHzzGas"));
            assert(pack.param_index_GET() == (short)28926);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)242) ;
        p320.param_id_SET("diRtstfucHzzGas", PH) ;
        p320.param_index_SET((short)28926) ;
        p320.target_component_SET((char)137) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)179);
            assert(pack.target_component_GET() == (char)175);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)179) ;
        p321.target_component_SET((char)175) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 13);
            assert(pack.param_value_TRY(ph).equals("qzpcYepphgmfc"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
            assert(pack.param_index_GET() == (char)52283);
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("aqSfchcTdgcnsxg"));
            assert(pack.param_count_GET() == (char)432);
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_count_SET((char)432) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8) ;
        p322.param_id_SET("aqSfchcTdgcnsxg", PH) ;
        p322.param_index_SET((char)52283) ;
        p322.param_value_SET("qzpcYepphgmfc", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
            assert(pack.target_component_GET() == (char)106);
            assert(pack.param_value_LEN(ph) == 115);
            assert(pack.param_value_TRY(ph).equals("whioivocwkincUxYlbkhjesdooqDQvnvmlssxqnuErbtsftxjkyccdJlOtamztnjbmtdkbhnNxjdejoHlgEzilSdBttkynFzcrtcMpcnxmzbfsqkcdl"));
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("mvfvdowKjfo"));
            assert(pack.target_system_GET() == (char)194);
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32) ;
        p323.target_system_SET((char)194) ;
        p323.target_component_SET((char)106) ;
        p323.param_id_SET("mvfvdowKjfo", PH) ;
        p323.param_value_SET("whioivocwkincUxYlbkhjesdooqDQvnvmlssxqnuErbtsftxjkyccdJlOtamztnjbmtdkbhnNxjdejoHlgEzilSdBttkynFzcrtcMpcnxmzbfsqkcdl", PH) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_ACCEPTED);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("mn"));
            assert(pack.param_value_LEN(ph) == 61);
            assert(pack.param_value_TRY(ph).equals("mzSsrusuZqyTligtlzyfdxmevrjsgxpaArkmgpdsqzbzvnRkmjvwVmgjhCAuk"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_value_SET("mzSsrusuZqyTligtlzyfdxmevrjsgxpaArkmgpdsqzbzvnRkmjvwVmgjhCAuk", PH) ;
        p324.param_id_SET("mn", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_ACCEPTED) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3981870949512027740L);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            assert(pack.min_distance_GET() == (char)10904);
            assert(pack.increment_GET() == (char)94);
            assert(pack.max_distance_GET() == (char)57011);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)7877, (char)51270, (char)34392, (char)44269, (char)2256, (char)4426, (char)20775, (char)50566, (char)40124, (char)47038, (char)64974, (char)38754, (char)2500, (char)42086, (char)39506, (char)10950, (char)5349, (char)46169, (char)27191, (char)25246, (char)40445, (char)27321, (char)12217, (char)22191, (char)14025, (char)24661, (char)65108, (char)14882, (char)16133, (char)13121, (char)17152, (char)34214, (char)60210, (char)29527, (char)29394, (char)18747, (char)37668, (char)44196, (char)59777, (char)43692, (char)32641, (char)33768, (char)12992, (char)43447, (char)44320, (char)3831, (char)57575, (char)15277, (char)47750, (char)3992, (char)15978, (char)42531, (char)55753, (char)49489, (char)8918, (char)26513, (char)44689, (char)44615, (char)3547, (char)24059, (char)47694, (char)13997, (char)14338, (char)46951, (char)5958, (char)35011, (char)2288, (char)52330, (char)28160, (char)40156, (char)13818, (char)28081}));
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND) ;
        p330.max_distance_SET((char)57011) ;
        p330.distances_SET(new char[] {(char)7877, (char)51270, (char)34392, (char)44269, (char)2256, (char)4426, (char)20775, (char)50566, (char)40124, (char)47038, (char)64974, (char)38754, (char)2500, (char)42086, (char)39506, (char)10950, (char)5349, (char)46169, (char)27191, (char)25246, (char)40445, (char)27321, (char)12217, (char)22191, (char)14025, (char)24661, (char)65108, (char)14882, (char)16133, (char)13121, (char)17152, (char)34214, (char)60210, (char)29527, (char)29394, (char)18747, (char)37668, (char)44196, (char)59777, (char)43692, (char)32641, (char)33768, (char)12992, (char)43447, (char)44320, (char)3831, (char)57575, (char)15277, (char)47750, (char)3992, (char)15978, (char)42531, (char)55753, (char)49489, (char)8918, (char)26513, (char)44689, (char)44615, (char)3547, (char)24059, (char)47694, (char)13997, (char)14338, (char)46951, (char)5958, (char)35011, (char)2288, (char)52330, (char)28160, (char)40156, (char)13818, (char)28081}, 0) ;
        p330.time_usec_SET(3981870949512027740L) ;
        p330.min_distance_SET((char)10904) ;
        p330.increment_SET((char)94) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
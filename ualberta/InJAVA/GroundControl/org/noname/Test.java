
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
            long id = id__c(src);
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
            long id = id__g(src);
            set_bits(id, 7, data, 260);
        }
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
    public static class NAV_FILTER_BIAS extends GroundControl.NAV_FILTER_BIAS
    {
        public long usec_GET()//Timestamp (microseconds)
        {  return (get_bytes(data,  0, 8)); }
        public float accel_0_GET()//b_f[0]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float accel_1_GET()//b_f[1]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float accel_2_GET()//b_f[2]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float gyro_0_GET()//b_f[0]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float gyro_1_GET()//b_f[1]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float gyro_2_GET()//b_f[2]
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
    }
    public static class RADIO_CALIBRATION extends GroundControl.RADIO_CALIBRATION
    {
        public char[] aileron_GET(char[]  dst_ch, int pos)  //Aileron setpoints: left, center, right
        {
            for(int BYTE = 0, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] aileron_GET()//Aileron setpoints: left, center, right
        {return aileron_GET(new char[3], 0);} public char[] elevator_GET(char[]  dst_ch, int pos)  //Elevator setpoints: nose down, center, nose up
        {
            for(int BYTE = 6, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] elevator_GET()//Elevator setpoints: nose down, center, nose up
        {return elevator_GET(new char[3], 0);} public char[] rudder_GET(char[]  dst_ch, int pos)  //Rudder setpoints: nose left, center, nose right
        {
            for(int BYTE = 12, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] rudder_GET()//Rudder setpoints: nose left, center, nose right
        {return rudder_GET(new char[3], 0);} public char[] gyro_GET(char[]  dst_ch, int pos)  //Tail gyro mode/gain setpoints: heading hold, rate mode
        {
            for(int BYTE = 18, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] gyro_GET()//Tail gyro mode/gain setpoints: heading hold, rate mode
        {return gyro_GET(new char[2], 0);} public char[] pitch_GET(char[]  dst_ch, int pos)  //Pitch curve setpoints (every 25%)
        {
            for(int BYTE = 22, dst_max = pos + 5; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] pitch_GET()//Pitch curve setpoints (every 25%)
        {return pitch_GET(new char[5], 0);} public char[] throttle_GET(char[]  dst_ch, int pos)  //Throttle curve setpoints (every 25%)
        {
            for(int BYTE = 32, dst_max = pos + 5; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] throttle_GET()//Throttle curve setpoints (every 25%)
        {return throttle_GET(new char[5], 0);}
    }
    public static class UALBERTA_SYS_STATUS extends GroundControl.UALBERTA_SYS_STATUS
    {
        public char mode_GET()//System mode, see UALBERTA_AUTOPILOT_MODE ENUM
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char nav_mode_GET()//Navigation mode, see UALBERTA_NAV_MODE ENUM
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char pilot_GET()//Pilot mode, see UALBERTA_PILOT_MODE
        {  return (char)((char) get_bytes(data,  2, 1)); }
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

        static final Collection<OnReceive.Handler<ALTITUDE, Channel>> on_ALTITUDE = new OnReceive<>();
        static final Collection<OnReceive.Handler<RESOURCE_REQUEST, Channel>> on_RESOURCE_REQUEST = new OnReceive<>();
        static final Collection<OnReceive.Handler<SCALED_PRESSURE3, Channel>> on_SCALED_PRESSURE3 = new OnReceive<>();
        static final Collection<OnReceive.Handler<FOLLOW_TARGET, Channel>> on_FOLLOW_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<CONTROL_SYSTEM_STATE, Channel>> on_CONTROL_SYSTEM_STATE = new OnReceive<>();
        static final Collection<OnReceive.Handler<BATTERY_STATUS, Channel>> on_BATTERY_STATUS = new OnReceive<>();
        static final Collection<OnReceive.Handler<AUTOPILOT_VERSION, Channel>> on_AUTOPILOT_VERSION = new OnReceive<>();
        static final Collection<OnReceive.Handler<LANDING_TARGET, Channel>> on_LANDING_TARGET = new OnReceive<>();
        static final Collection<OnReceive.Handler<NAV_FILTER_BIAS, Channel>> on_NAV_FILTER_BIAS = new OnReceive<>();
        static final Collection<OnReceive.Handler<RADIO_CALIBRATION, Channel>> on_RADIO_CALIBRATION = new OnReceive<>();
        static final Collection<OnReceive.Handler<UALBERTA_SYS_STATUS, Channel>> on_UALBERTA_SYS_STATUS = new OnReceive<>();
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
                case 220:
                    if(pack == null) return new NAV_FILTER_BIAS();
                    ((OnReceive) on_NAV_FILTER_BIAS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 221:
                    if(pack == null) return new RADIO_CALIBRATION();
                    ((OnReceive) on_RADIO_CALIBRATION).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
                    break;
                case 222:
                    if(pack == null) return new UALBERTA_SYS_STATUS();
                    ((OnReceive) on_UALBERTA_SYS_STATUS).handle(this, ph);//no any host channels can receive this pack. Handle it with test channel handler
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
            assert(pack.custom_mode_GET() == 1278911512L);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_SMARTAP);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_FLIGHT_TERMINATION);
            assert(pack.mavlink_version_GET() == (char)47);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_GCS);
        });
        HEARTBEAT p0 = new HEARTBEAT();
        PH.setPack(p0);
        p0.system_status_SET(MAV_STATE.MAV_STATE_FLIGHT_TERMINATION) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_SMARTAP) ;
        p0.mavlink_version_SET((char)47) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_GCS) ;
        p0.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                          MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED)) ;
        p0.custom_mode_SET(1278911512L) ;
        TestChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_comm_GET() == (char)40856);
            assert(pack.drop_rate_comm_GET() == (char)27702);
            assert(pack.onboard_control_sensors_present_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
            assert(pack.errors_count1_GET() == (char)64068);
            assert(pack.errors_count4_GET() == (char)6279);
            assert(pack.voltage_battery_GET() == (char)54907);
            assert(pack.load_GET() == (char)9409);
            assert(pack.onboard_control_sensors_health_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            assert(pack.current_battery_GET() == (short)13170);
            assert(pack.errors_count2_GET() == (char)63499);
            assert(pack.errors_count3_GET() == (char)10607);
            assert(pack.battery_remaining_GET() == (byte)116);
            assert(pack.onboard_control_sensors_enabled_GET() == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                    MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
        });
        SYS_STATUS p1 = new SYS_STATUS();
        PH.setPack(p1);
        p1.errors_count2_SET((char)63499) ;
        p1.onboard_control_sensors_present_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL)) ;
        p1.errors_count1_SET((char)64068) ;
        p1.current_battery_SET((short)13170) ;
        p1.errors_comm_SET((char)40856) ;
        p1.onboard_control_sensors_health_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                               MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)) ;
        p1.errors_count4_SET((char)6279) ;
        p1.load_SET((char)9409) ;
        p1.voltage_battery_SET((char)54907) ;
        p1.errors_count3_SET((char)10607) ;
        p1.drop_rate_comm_SET((char)27702) ;
        p1.onboard_control_sensors_enabled_SET((MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW)) ;
        p1.battery_remaining_SET((byte)116) ;
        TestChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1689611714L);
            assert(pack.time_unix_usec_GET() == 1249632845263910755L);
        });
        SYSTEM_TIME p2 = new SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(1689611714L) ;
        p2.time_unix_usec_SET(1249632845263910755L) ;
        TestChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == -3.1610561E38F);
            assert(pack.afy_GET() == -2.3889113E38F);
            assert(pack.afz_GET() == -2.3809107E36F);
            assert(pack.vz_GET() == -1.5488595E38F);
            assert(pack.type_mask_GET() == (char)10312);
            assert(pack.vy_GET() == -2.8782648E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.y_GET() == -5.9579735E37F);
            assert(pack.x_GET() == 4.5385573E37F);
            assert(pack.vx_GET() == -6.275132E37F);
            assert(pack.z_GET() == 8.1112687E37F);
            assert(pack.yaw_GET() == 9.215384E37F);
            assert(pack.time_boot_ms_GET() == 1019314798L);
            assert(pack.afx_GET() == -9.932855E37F);
        });
        GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p3.afz_SET(-2.3809107E36F) ;
        p3.vz_SET(-1.5488595E38F) ;
        p3.afx_SET(-9.932855E37F) ;
        p3.z_SET(8.1112687E37F) ;
        p3.yaw_rate_SET(-3.1610561E38F) ;
        p3.y_SET(-5.9579735E37F) ;
        p3.afy_SET(-2.3889113E38F) ;
        p3.vx_SET(-6.275132E37F) ;
        p3.x_SET(4.5385573E37F) ;
        p3.time_boot_ms_SET(1019314798L) ;
        p3.type_mask_SET((char)10312) ;
        p3.yaw_SET(9.215384E37F) ;
        p3.vy_SET(-2.8782648E38F) ;
        CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)167);
            assert(pack.target_system_GET() == (char)82);
            assert(pack.time_usec_GET() == 398330723600237531L);
            assert(pack.seq_GET() == 1037544000L);
        });
        PING p4 = new PING();
        PH.setPack(p4);
        p4.seq_SET(1037544000L) ;
        p4.target_component_SET((char)167) ;
        p4.time_usec_SET(398330723600237531L) ;
        p4.target_system_SET((char)82) ;
        TestChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.passkey_LEN(ph) == 21);
            assert(pack.passkey_TRY(ph).equals("doclXralwdwowxmruzrkx"));
            assert(pack.target_system_GET() == (char)146);
            assert(pack.version_GET() == (char)65);
            assert(pack.control_request_GET() == (char)239);
        });
        CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)146) ;
        p5.passkey_SET("doclXralwdwowxmruzrkx", PH) ;
        p5.control_request_SET((char)239) ;
        p5.version_SET((char)65) ;
        TestChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)246);
            assert(pack.ack_GET() == (char)249);
            assert(pack.gcs_system_id_GET() == (char)100);
        });
        CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.ack_SET((char)249) ;
        p6.gcs_system_id_SET((char)100) ;
        p6.control_request_SET((char)246) ;
        TestChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 26);
            assert(pack.key_TRY(ph).equals("mZiyggdLmwqzRrclyijalmorzq"));
        });
        AUTH_KEY p7 = new AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("mZiyggdLmwqzRrclyijalmorzq", PH) ;
        TestChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)238);
            assert(pack.custom_mode_GET() == 1544960956L);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_ARMED);
        });
        SET_MODE p11 = new SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)238) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_STABILIZE_ARMED) ;
        p11.custom_mode_SET(1544960956L) ;
        TestChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short) -19755);
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("fnm"));
            assert(pack.target_component_GET() == (char)18);
            assert(pack.target_system_GET() == (char)212);
        });
        PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)212) ;
        p20.target_component_SET((char)18) ;
        p20.param_id_SET("fnm", PH) ;
        p20.param_index_SET((short) -19755) ;
        TestChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)134);
            assert(pack.target_system_GET() == (char)50);
        });
        PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)50) ;
        p21.target_component_SET((char)134) ;
        TestChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_GET() == 3.034728E38F);
            assert(pack.param_index_GET() == (char)88);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("seiwhRfybstz"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
            assert(pack.param_count_GET() == (char)22143);
        });
        PARAM_VALUE p22 = new PARAM_VALUE();
        PH.setPack(p22);
        p22.param_value_SET(3.034728E38F) ;
        p22.param_index_SET((char)88) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8) ;
        p22.param_count_SET((char)22143) ;
        p22.param_id_SET("seiwhRfybstz", PH) ;
        TestChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)236);
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("wlmrkuzbvlz"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
            assert(pack.target_system_GET() == (char)199);
            assert(pack.param_value_GET() == -2.8662677E38F);
        });
        PARAM_SET p23 = new PARAM_SET();
        PH.setPack(p23);
        p23.param_value_SET(-2.8662677E38F) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64) ;
        p23.param_id_SET("wlmrkuzbvlz", PH) ;
        p23.target_component_SET((char)236) ;
        p23.target_system_SET((char)199) ;
        TestChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.vel_acc_TRY(ph) == 2980445215L);
            assert(pack.alt_ellipsoid_TRY(ph) == -1869299355);
            assert(pack.lat_GET() == -892721741);
            assert(pack.h_acc_TRY(ph) == 1159130287L);
            assert(pack.cog_GET() == (char)59945);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
            assert(pack.time_usec_GET() == 1602272326532159816L);
            assert(pack.alt_GET() == -1372411600);
            assert(pack.lon_GET() == 269840070);
            assert(pack.epv_GET() == (char)48245);
            assert(pack.eph_GET() == (char)58589);
            assert(pack.vel_GET() == (char)18738);
            assert(pack.hdg_acc_TRY(ph) == 2434921224L);
            assert(pack.satellites_visible_GET() == (char)45);
            assert(pack.v_acc_TRY(ph) == 2859074490L);
        });
        GPS_RAW_INT p24 = new GPS_RAW_INT();
        PH.setPack(p24);
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX) ;
        p24.lon_SET(269840070) ;
        p24.time_usec_SET(1602272326532159816L) ;
        p24.h_acc_SET(1159130287L, PH) ;
        p24.vel_acc_SET(2980445215L, PH) ;
        p24.v_acc_SET(2859074490L, PH) ;
        p24.alt_ellipsoid_SET(-1869299355, PH) ;
        p24.epv_SET((char)48245) ;
        p24.hdg_acc_SET(2434921224L, PH) ;
        p24.vel_SET((char)18738) ;
        p24.eph_SET((char)58589) ;
        p24.alt_SET(-1372411600) ;
        p24.cog_SET((char)59945) ;
        p24.lat_SET(-892721741) ;
        p24.satellites_visible_SET((char)45) ;
        TestChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)96, (char)141, (char)43, (char)49, (char)192, (char)21, (char)223, (char)219, (char)76, (char)163, (char)180, (char)173, (char)145, (char)154, (char)184, (char)161, (char)71, (char)154, (char)33, (char)239}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)248, (char)215, (char)255, (char)254, (char)17, (char)25, (char)102, (char)229, (char)231, (char)35, (char)174, (char)210, (char)81, (char)13, (char)204, (char)216, (char)85, (char)150, (char)146, (char)172}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)72, (char)137, (char)96, (char)253, (char)222, (char)74, (char)83, (char)2, (char)175, (char)196, (char)87, (char)181, (char)53, (char)240, (char)164, (char)193, (char)14, (char)210, (char)30, (char)67}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)15, (char)24, (char)132, (char)211, (char)34, (char)179, (char)228, (char)79, (char)152, (char)226, (char)104, (char)45, (char)103, (char)96, (char)94, (char)102, (char)3, (char)108, (char)81, (char)174}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)204, (char)140, (char)52, (char)208, (char)45, (char)95, (char)199, (char)249, (char)205, (char)86, (char)100, (char)30, (char)194, (char)187, (char)140, (char)50, (char)166, (char)5, (char)252, (char)213}));
            assert(pack.satellites_visible_GET() == (char)67);
        });
        GPS_STATUS p25 = new GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)67) ;
        p25.satellite_azimuth_SET(new char[] {(char)204, (char)140, (char)52, (char)208, (char)45, (char)95, (char)199, (char)249, (char)205, (char)86, (char)100, (char)30, (char)194, (char)187, (char)140, (char)50, (char)166, (char)5, (char)252, (char)213}, 0) ;
        p25.satellite_used_SET(new char[] {(char)248, (char)215, (char)255, (char)254, (char)17, (char)25, (char)102, (char)229, (char)231, (char)35, (char)174, (char)210, (char)81, (char)13, (char)204, (char)216, (char)85, (char)150, (char)146, (char)172}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)96, (char)141, (char)43, (char)49, (char)192, (char)21, (char)223, (char)219, (char)76, (char)163, (char)180, (char)173, (char)145, (char)154, (char)184, (char)161, (char)71, (char)154, (char)33, (char)239}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)15, (char)24, (char)132, (char)211, (char)34, (char)179, (char)228, (char)79, (char)152, (char)226, (char)104, (char)45, (char)103, (char)96, (char)94, (char)102, (char)3, (char)108, (char)81, (char)174}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)72, (char)137, (char)96, (char)253, (char)222, (char)74, (char)83, (char)2, (char)175, (char)196, (char)87, (char)181, (char)53, (char)240, (char)164, (char)193, (char)14, (char)210, (char)30, (char)67}, 0) ;
        TestChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short) -30415);
            assert(pack.zgyro_GET() == (short)5804);
            assert(pack.xacc_GET() == (short)18546);
            assert(pack.ymag_GET() == (short) -21104);
            assert(pack.ygyro_GET() == (short) -11869);
            assert(pack.yacc_GET() == (short)19041);
            assert(pack.time_boot_ms_GET() == 597992954L);
            assert(pack.zmag_GET() == (short)10490);
            assert(pack.xgyro_GET() == (short) -13203);
            assert(pack.xmag_GET() == (short)14598);
        });
        SCALED_IMU p26 = new SCALED_IMU();
        PH.setPack(p26);
        p26.xgyro_SET((short) -13203) ;
        p26.ygyro_SET((short) -11869) ;
        p26.zgyro_SET((short)5804) ;
        p26.yacc_SET((short)19041) ;
        p26.time_boot_ms_SET(597992954L) ;
        p26.xacc_SET((short)18546) ;
        p26.zacc_SET((short) -30415) ;
        p26.ymag_SET((short) -21104) ;
        p26.xmag_SET((short)14598) ;
        p26.zmag_SET((short)10490) ;
        TestChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.xgyro_GET() == (short) -14037);
            assert(pack.yacc_GET() == (short) -15644);
            assert(pack.ygyro_GET() == (short) -6080);
            assert(pack.zmag_GET() == (short)944);
            assert(pack.zgyro_GET() == (short)18092);
            assert(pack.zacc_GET() == (short) -1386);
            assert(pack.ymag_GET() == (short)11902);
            assert(pack.time_usec_GET() == 4982005059726066006L);
            assert(pack.xacc_GET() == (short)16741);
            assert(pack.xmag_GET() == (short) -129);
        });
        RAW_IMU p27 = new RAW_IMU();
        PH.setPack(p27);
        p27.ygyro_SET((short) -6080) ;
        p27.ymag_SET((short)11902) ;
        p27.zacc_SET((short) -1386) ;
        p27.xacc_SET((short)16741) ;
        p27.xmag_SET((short) -129) ;
        p27.yacc_SET((short) -15644) ;
        p27.time_usec_SET(4982005059726066006L) ;
        p27.zmag_SET((short)944) ;
        p27.xgyro_SET((short) -14037) ;
        p27.zgyro_SET((short)18092) ;
        TestChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == (short) -5800);
            assert(pack.temperature_GET() == (short) -24086);
            assert(pack.press_diff2_GET() == (short)30572);
            assert(pack.time_usec_GET() == 3963256094591329337L);
            assert(pack.press_diff1_GET() == (short) -1001);
        });
        RAW_PRESSURE p28 = new RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_diff1_SET((short) -1001) ;
        p28.press_diff2_SET((short)30572) ;
        p28.time_usec_SET(3963256094591329337L) ;
        p28.press_abs_SET((short) -5800) ;
        p28.temperature_SET((short) -24086) ;
        TestChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == -6.091502E36F);
            assert(pack.temperature_GET() == (short)11287);
            assert(pack.time_boot_ms_GET() == 1109227595L);
            assert(pack.press_abs_GET() == 7.7964274E37F);
        });
        SCALED_PRESSURE p29 = new SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_abs_SET(7.7964274E37F) ;
        p29.temperature_SET((short)11287) ;
        p29.time_boot_ms_SET(1109227595L) ;
        p29.press_diff_SET(-6.091502E36F) ;
        TestChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.pitchspeed_GET() == 2.898748E38F);
            assert(pack.time_boot_ms_GET() == 415436834L);
            assert(pack.roll_GET() == -3.3680633E38F);
            assert(pack.yaw_GET() == 2.7140695E38F);
            assert(pack.pitch_GET() == 3.269662E38F);
            assert(pack.rollspeed_GET() == 8.380176E37F);
            assert(pack.yawspeed_GET() == -3.965107E37F);
        });
        ATTITUDE p30 = new ATTITUDE();
        PH.setPack(p30);
        p30.pitch_SET(3.269662E38F) ;
        p30.yawspeed_SET(-3.965107E37F) ;
        p30.roll_SET(-3.3680633E38F) ;
        p30.pitchspeed_SET(2.898748E38F) ;
        p30.rollspeed_SET(8.380176E37F) ;
        p30.time_boot_ms_SET(415436834L) ;
        p30.yaw_SET(2.7140695E38F) ;
        TestChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 1.732828E38F);
            assert(pack.q4_GET() == 3.0659984E38F);
            assert(pack.pitchspeed_GET() == 1.2685497E38F);
            assert(pack.q2_GET() == 5.766391E37F);
            assert(pack.q1_GET() == 1.4087599E38F);
            assert(pack.q3_GET() == 3.5448718E36F);
            assert(pack.time_boot_ms_GET() == 2614574093L);
            assert(pack.rollspeed_GET() == -1.1410472E38F);
        });
        ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q2_SET(5.766391E37F) ;
        p31.rollspeed_SET(-1.1410472E38F) ;
        p31.q1_SET(1.4087599E38F) ;
        p31.q3_SET(3.5448718E36F) ;
        p31.pitchspeed_SET(1.2685497E38F) ;
        p31.yawspeed_SET(1.732828E38F) ;
        p31.time_boot_ms_SET(2614574093L) ;
        p31.q4_SET(3.0659984E38F) ;
        TestChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == 2.1036057E38F);
            assert(pack.x_GET() == -8.2379983E37F);
            assert(pack.z_GET() == 1.4626005E38F);
            assert(pack.y_GET() == 1.1692411E38F);
            assert(pack.time_boot_ms_GET() == 83275373L);
            assert(pack.vy_GET() == -2.568199E38F);
            assert(pack.vx_GET() == 1.4074698E38F);
        });
        LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.y_SET(1.1692411E38F) ;
        p32.z_SET(1.4626005E38F) ;
        p32.x_SET(-8.2379983E37F) ;
        p32.vz_SET(2.1036057E38F) ;
        p32.vy_SET(-2.568199E38F) ;
        p32.vx_SET(1.4074698E38F) ;
        p32.time_boot_ms_SET(83275373L) ;
        TestChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == (short) -30586);
            assert(pack.alt_GET() == -669801800);
            assert(pack.relative_alt_GET() == 1508785017);
            assert(pack.vz_GET() == (short)15320);
            assert(pack.lon_GET() == -160225325);
            assert(pack.time_boot_ms_GET() == 4099929677L);
            assert(pack.hdg_GET() == (char)21924);
            assert(pack.lat_GET() == -629492976);
            assert(pack.vy_GET() == (short) -22427);
        });
        GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.relative_alt_SET(1508785017) ;
        p33.lat_SET(-629492976) ;
        p33.hdg_SET((char)21924) ;
        p33.time_boot_ms_SET(4099929677L) ;
        p33.vx_SET((short) -30586) ;
        p33.alt_SET(-669801800) ;
        p33.vy_SET((short) -22427) ;
        p33.vz_SET((short)15320) ;
        p33.lon_SET(-160225325) ;
        TestChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan3_scaled_GET() == (short)22213);
            assert(pack.chan8_scaled_GET() == (short)10840);
            assert(pack.chan5_scaled_GET() == (short)11189);
            assert(pack.port_GET() == (char)38);
            assert(pack.chan4_scaled_GET() == (short) -12308);
            assert(pack.rssi_GET() == (char)233);
            assert(pack.chan6_scaled_GET() == (short)9291);
            assert(pack.time_boot_ms_GET() == 995200914L);
            assert(pack.chan2_scaled_GET() == (short)32417);
            assert(pack.chan7_scaled_GET() == (short) -17332);
            assert(pack.chan1_scaled_GET() == (short) -32755);
        });
        RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan8_scaled_SET((short)10840) ;
        p34.chan3_scaled_SET((short)22213) ;
        p34.chan6_scaled_SET((short)9291) ;
        p34.chan1_scaled_SET((short) -32755) ;
        p34.port_SET((char)38) ;
        p34.chan4_scaled_SET((short) -12308) ;
        p34.chan7_scaled_SET((short) -17332) ;
        p34.chan5_scaled_SET((short)11189) ;
        p34.chan2_scaled_SET((short)32417) ;
        p34.rssi_SET((char)233) ;
        p34.time_boot_ms_SET(995200914L) ;
        TestChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan2_raw_GET() == (char)16817);
            assert(pack.chan4_raw_GET() == (char)26634);
            assert(pack.chan8_raw_GET() == (char)65038);
            assert(pack.chan1_raw_GET() == (char)8377);
            assert(pack.rssi_GET() == (char)124);
            assert(pack.chan3_raw_GET() == (char)55367);
            assert(pack.chan6_raw_GET() == (char)51677);
            assert(pack.chan7_raw_GET() == (char)29063);
            assert(pack.time_boot_ms_GET() == 2625549536L);
            assert(pack.port_GET() == (char)62);
            assert(pack.chan5_raw_GET() == (char)2968);
        });
        RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.port_SET((char)62) ;
        p35.chan3_raw_SET((char)55367) ;
        p35.rssi_SET((char)124) ;
        p35.chan5_raw_SET((char)2968) ;
        p35.chan2_raw_SET((char)16817) ;
        p35.chan1_raw_SET((char)8377) ;
        p35.chan6_raw_SET((char)51677) ;
        p35.time_boot_ms_SET(2625549536L) ;
        p35.chan4_raw_SET((char)26634) ;
        p35.chan8_raw_SET((char)65038) ;
        p35.chan7_raw_SET((char)29063) ;
        TestChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo11_raw_TRY(ph) == (char)8412);
            assert(pack.servo9_raw_TRY(ph) == (char)64901);
            assert(pack.servo7_raw_GET() == (char)30778);
            assert(pack.port_GET() == (char)17);
            assert(pack.servo13_raw_TRY(ph) == (char)53586);
            assert(pack.servo14_raw_TRY(ph) == (char)18290);
            assert(pack.servo12_raw_TRY(ph) == (char)57340);
            assert(pack.servo8_raw_GET() == (char)17072);
            assert(pack.servo5_raw_GET() == (char)30312);
            assert(pack.servo4_raw_GET() == (char)48250);
            assert(pack.servo6_raw_GET() == (char)64676);
            assert(pack.time_usec_GET() == 3991568637L);
            assert(pack.servo2_raw_GET() == (char)61556);
            assert(pack.servo15_raw_TRY(ph) == (char)50013);
            assert(pack.servo16_raw_TRY(ph) == (char)36443);
            assert(pack.servo1_raw_GET() == (char)18578);
            assert(pack.servo3_raw_GET() == (char)23051);
            assert(pack.servo10_raw_TRY(ph) == (char)60341);
        });
        SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo6_raw_SET((char)64676) ;
        p36.servo13_raw_SET((char)53586, PH) ;
        p36.servo16_raw_SET((char)36443, PH) ;
        p36.servo14_raw_SET((char)18290, PH) ;
        p36.servo12_raw_SET((char)57340, PH) ;
        p36.servo15_raw_SET((char)50013, PH) ;
        p36.port_SET((char)17) ;
        p36.time_usec_SET(3991568637L) ;
        p36.servo11_raw_SET((char)8412, PH) ;
        p36.servo5_raw_SET((char)30312) ;
        p36.servo9_raw_SET((char)64901, PH) ;
        p36.servo10_raw_SET((char)60341, PH) ;
        p36.servo3_raw_SET((char)23051) ;
        p36.servo2_raw_SET((char)61556) ;
        p36.servo7_raw_SET((char)30778) ;
        p36.servo4_raw_SET((char)48250) ;
        p36.servo1_raw_SET((char)18578) ;
        p36.servo8_raw_SET((char)17072) ;
        TestChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)39);
            assert(pack.target_system_GET() == (char)57);
            assert(pack.end_index_GET() == (short) -16816);
            assert(pack.start_index_GET() == (short) -27538);
        });
        MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_system_SET((char)57) ;
        p37.target_component_SET((char)39) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p37.start_index_SET((short) -27538) ;
        p37.end_index_SET((short) -16816) ;
        TestChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)59);
            assert(pack.end_index_GET() == (short) -9740);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.start_index_GET() == (short) -25892);
            assert(pack.target_component_GET() == (char)82);
        });
        MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_system_SET((char)59) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p38.target_component_SET((char)82) ;
        p38.start_index_SET((short) -25892) ;
        p38.end_index_SET((short) -9740) ;
        TestChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.current_GET() == (char)132);
            assert(pack.param2_GET() == 2.844053E38F);
            assert(pack.seq_GET() == (char)44679);
            assert(pack.x_GET() == 6.7922646E37F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.y_GET() == 2.2153249E38F);
            assert(pack.z_GET() == -2.0339107E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.param3_GET() == -2.5362571E38F);
            assert(pack.autocontinue_GET() == (char)13);
            assert(pack.target_component_GET() == (char)152);
            assert(pack.param1_GET() == 1.7857303E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT);
            assert(pack.param4_GET() == -3.2618686E38F);
            assert(pack.target_system_GET() == (char)96);
        });
        MISSION_ITEM p39 = new MISSION_ITEM();
        PH.setPack(p39);
        p39.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p39.target_system_SET((char)96) ;
        p39.z_SET(-2.0339107E38F) ;
        p39.param3_SET(-2.5362571E38F) ;
        p39.target_component_SET((char)152) ;
        p39.current_SET((char)132) ;
        p39.param1_SET(1.7857303E38F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p39.command_SET(MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) ;
        p39.param4_SET(-3.2618686E38F) ;
        p39.seq_SET((char)44679) ;
        p39.y_SET(2.2153249E38F) ;
        p39.param2_SET(2.844053E38F) ;
        p39.autocontinue_SET((char)13) ;
        p39.x_SET(6.7922646E37F) ;
        TestChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)16845);
            assert(pack.target_component_GET() == (char)17);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)117);
        });
        MISSION_REQUEST p40 = new MISSION_REQUEST();
        PH.setPack(p40);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p40.target_system_SET((char)117) ;
        p40.seq_SET((char)16845) ;
        p40.target_component_SET((char)17) ;
        TestChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)198);
            assert(pack.seq_GET() == (char)50590);
            assert(pack.target_system_GET() == (char)45);
        });
        MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)45) ;
        p41.seq_SET((char)50590) ;
        p41.target_component_SET((char)198) ;
        TestChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)24669);
        });
        MISSION_CURRENT p42 = new MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)24669) ;
        TestChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)235);
            assert(pack.target_component_GET() == (char)245);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p43.target_system_SET((char)235) ;
        p43.target_component_SET((char)245) ;
        TestChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)1046);
            assert(pack.target_component_GET() == (char)160);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)125);
        });
        MISSION_COUNT p44 = new MISSION_COUNT();
        PH.setPack(p44);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p44.target_component_SET((char)160) ;
        p44.count_SET((char)1046) ;
        p44.target_system_SET((char)125) ;
        TestChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)171);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)144);
        });
        MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_component_SET((char)144) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p45.target_system_SET((char)171) ;
        TestChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)63437);
        });
        MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)63437) ;
        TestChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)174);
            assert(pack.target_component_GET() == (char)33);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        MISSION_ACK p47 = new MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED) ;
        p47.target_component_SET((char)33) ;
        p47.target_system_SET((char)174) ;
        TestChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 6523391310138941119L);
            assert(pack.target_system_GET() == (char)193);
            assert(pack.latitude_GET() == 1870874095);
            assert(pack.altitude_GET() == 1060349323);
            assert(pack.longitude_GET() == -609729070);
        });
        SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.longitude_SET(-609729070) ;
        p48.latitude_SET(1870874095) ;
        p48.altitude_SET(1060349323) ;
        p48.time_usec_SET(6523391310138941119L, PH) ;
        p48.target_system_SET((char)193) ;
        TestChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 1067573658271238813L);
            assert(pack.latitude_GET() == -1076552003);
            assert(pack.longitude_GET() == 998282131);
            assert(pack.altitude_GET() == 1311629950);
        });
        GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.altitude_SET(1311629950) ;
        p49.time_usec_SET(1067573658271238813L, PH) ;
        p49.latitude_SET(-1076552003) ;
        p49.longitude_SET(998282131) ;
        TestChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)7);
            assert(pack.parameter_rc_channel_index_GET() == (char)155);
            assert(pack.target_system_GET() == (char)140);
            assert(pack.param_value_min_GET() == -2.583348E38F);
            assert(pack.param_value0_GET() == 3.1503965E38F);
            assert(pack.param_index_GET() == (short) -16062);
            assert(pack.param_value_max_GET() == -8.3730097E37F);
            assert(pack.scale_GET() == -3.2164687E38F);
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("mwcJscoDguwamPz"));
        });
        PARAM_MAP_RC p50 = new PARAM_MAP_RC();
        PH.setPack(p50);
        p50.scale_SET(-3.2164687E38F) ;
        p50.param_index_SET((short) -16062) ;
        p50.target_system_SET((char)140) ;
        p50.param_value_max_SET(-8.3730097E37F) ;
        p50.param_value0_SET(3.1503965E38F) ;
        p50.parameter_rc_channel_index_SET((char)155) ;
        p50.param_value_min_SET(-2.583348E38F) ;
        p50.param_id_SET("mwcJscoDguwamPz", PH) ;
        p50.target_component_SET((char)7) ;
        TestChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)242);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.seq_GET() == (char)7763);
            assert(pack.target_system_GET() == (char)9);
        });
        MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_system_SET((char)9) ;
        p51.target_component_SET((char)242) ;
        p51.seq_SET((char)7763) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        TestChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)124);
            assert(pack.target_system_GET() == (char)247);
            assert(pack.p2z_GET() == -2.2726062E37F);
            assert(pack.p1z_GET() == -1.4890054E38F);
            assert(pack.p1x_GET() == 4.285737E37F);
            assert(pack.p2y_GET() == -2.246916E38F);
            assert(pack.p2x_GET() == 8.171123E37F);
            assert(pack.p1y_GET() == -3.2111829E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
        });
        SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p2x_SET(8.171123E37F) ;
        p54.p1x_SET(4.285737E37F) ;
        p54.p2y_SET(-2.246916E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p54.p1y_SET(-3.2111829E38F) ;
        p54.target_component_SET((char)124) ;
        p54.p1z_SET(-1.4890054E38F) ;
        p54.target_system_SET((char)247) ;
        p54.p2z_SET(-2.2726062E37F) ;
        TestChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2z_GET() == 2.5556077E38F);
            assert(pack.p1z_GET() == 1.4856472E38F);
            assert(pack.p1x_GET() == -1.5139703E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.p2x_GET() == 1.1629493E38F);
            assert(pack.p2y_GET() == 3.1224224E38F);
            assert(pack.p1y_GET() == -1.8921064E38F);
        });
        SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p1y_SET(-1.8921064E38F) ;
        p55.p2z_SET(2.5556077E38F) ;
        p55.p2y_SET(3.1224224E38F) ;
        p55.p1x_SET(-1.5139703E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p55.p2x_SET(1.1629493E38F) ;
        p55.p1z_SET(1.4856472E38F) ;
        TestChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 1.7501616E38F);
            assert(pack.rollspeed_GET() == 1.9785807E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-7.5721826E37F, -1.6522517E38F, 2.1382552E38F, 3.33846E38F}));
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-4.566021E37F, 2.9356038E38F, 2.6041324E38F, 3.3230345E38F, 2.0210186E38F, -2.1648394E37F, -6.8727585E36F, -4.162619E37F, -1.1068494E38F}));
            assert(pack.pitchspeed_GET() == -1.9662165E38F);
            assert(pack.time_usec_GET() == 350990413844194264L);
        });
        ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.covariance_SET(new float[] {-4.566021E37F, 2.9356038E38F, 2.6041324E38F, 3.3230345E38F, 2.0210186E38F, -2.1648394E37F, -6.8727585E36F, -4.162619E37F, -1.1068494E38F}, 0) ;
        p61.pitchspeed_SET(-1.9662165E38F) ;
        p61.q_SET(new float[] {-7.5721826E37F, -1.6522517E38F, 2.1382552E38F, 3.33846E38F}, 0) ;
        p61.time_usec_SET(350990413844194264L) ;
        p61.rollspeed_SET(1.9785807E38F) ;
        p61.yawspeed_SET(1.7501616E38F) ;
        TestChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_bearing_GET() == (short) -5007);
            assert(pack.nav_roll_GET() == -8.973901E37F);
            assert(pack.nav_pitch_GET() == 2.0434276E38F);
            assert(pack.wp_dist_GET() == (char)8290);
            assert(pack.alt_error_GET() == -7.6349E37F);
            assert(pack.aspd_error_GET() == 1.9922277E38F);
            assert(pack.xtrack_error_GET() == -1.976487E38F);
            assert(pack.target_bearing_GET() == (short) -17943);
        });
        NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_pitch_SET(2.0434276E38F) ;
        p62.xtrack_error_SET(-1.976487E38F) ;
        p62.alt_error_SET(-7.6349E37F) ;
        p62.nav_bearing_SET((short) -5007) ;
        p62.wp_dist_SET((char)8290) ;
        p62.nav_roll_SET(-8.973901E37F) ;
        p62.aspd_error_SET(1.9922277E38F) ;
        p62.target_bearing_SET((short) -17943) ;
        TestChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == -1.2945912E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
            assert(pack.vz_GET() == -1.5435191E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.727409E38F, 2.693443E38F, -2.2178158E38F, -2.5161783E38F, 1.7909276E38F, -1.0025965E37F, 1.972707E38F, 1.4426762E38F, -2.8314578E38F, -1.5798126E38F, 1.9205286E38F, 2.6154944E38F, 2.5569472E38F, -3.7478346E37F, 2.1275065E38F, 2.7629719E38F, -5.4550174E37F, -1.5787891E38F, 6.9150903E37F, 1.3610138E38F, -3.3722206E38F, 4.3318085E37F, -2.5313283E38F, -2.5605528E38F, 1.568015E38F, 3.3946787E38F, 7.9082484E37F, 8.4287513E37F, 1.4091193E38F, 2.8459628E38F, 1.6769983E38F, -8.05056E37F, -3.0685452E38F, 3.3752641E38F, -4.0208083E36F, -1.1733403E38F}));
            assert(pack.time_usec_GET() == 966074472526232378L);
            assert(pack.vy_GET() == -1.3208822E38F);
            assert(pack.relative_alt_GET() == 1491199591);
            assert(pack.lat_GET() == 1145415008);
            assert(pack.alt_GET() == 860409948);
            assert(pack.lon_GET() == -1390785162);
        });
        GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.vx_SET(-1.2945912E38F) ;
        p63.lon_SET(-1390785162) ;
        p63.vz_SET(-1.5435191E38F) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE) ;
        p63.relative_alt_SET(1491199591) ;
        p63.lat_SET(1145415008) ;
        p63.alt_SET(860409948) ;
        p63.time_usec_SET(966074472526232378L) ;
        p63.vy_SET(-1.3208822E38F) ;
        p63.covariance_SET(new float[] {-1.727409E38F, 2.693443E38F, -2.2178158E38F, -2.5161783E38F, 1.7909276E38F, -1.0025965E37F, 1.972707E38F, 1.4426762E38F, -2.8314578E38F, -1.5798126E38F, 1.9205286E38F, 2.6154944E38F, 2.5569472E38F, -3.7478346E37F, 2.1275065E38F, 2.7629719E38F, -5.4550174E37F, -1.5787891E38F, 6.9150903E37F, 1.3610138E38F, -3.3722206E38F, 4.3318085E37F, -2.5313283E38F, -2.5605528E38F, 1.568015E38F, 3.3946787E38F, 7.9082484E37F, 8.4287513E37F, 1.4091193E38F, 2.8459628E38F, 1.6769983E38F, -8.05056E37F, -3.0685452E38F, 3.3752641E38F, -4.0208083E36F, -1.1733403E38F}, 0) ;
        TestChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == -2.7455844E38F);
            assert(pack.ay_GET() == -7.3904025E37F);
            assert(pack.z_GET() == 2.1139633E38F);
            assert(pack.time_usec_GET() == 4451298139285016736L);
            assert(pack.az_GET() == -2.5728403E36F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
            assert(pack.y_GET() == 1.2551236E38F);
            assert(pack.ax_GET() == -6.2661166E37F);
            assert(pack.x_GET() == -7.6148263E37F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.549376E38F, -1.2662383E38F, 2.4615295E37F, 3.273793E38F, -2.7277991E38F, 1.6971092E38F, 2.7630818E37F, 8.812279E37F, 5.1041196E37F, 3.2920766E38F, -2.2841646E38F, 7.7206686E37F, -3.9490947E37F, 5.6902067E37F, 2.3817153E38F, -8.677272E37F, 3.0767444E38F, -2.2693434E38F, 2.3939093E38F, -2.3527593E38F, -3.0100053E38F, 2.680794E38F, 3.1813499E38F, 1.2670784E38F, -5.5877085E37F, 3.1886865E38F, 2.909961E38F, -2.946196E38F, 2.7349227E38F, -2.8092318E38F, -1.5136416E38F, -3.3147761E38F, 2.8972402E38F, 2.0911375E38F, -1.9261263E38F, -2.3794725E38F, -2.0831556E38F, 2.8052319E38F, -1.1744347E38F, 2.1354177E38F, -2.4713252E38F, -3.0197275E38F, -1.6592161E38F, -1.5808022E38F, -1.6805539E38F}));
            assert(pack.vz_GET() == -3.2075566E38F);
            assert(pack.vy_GET() == 2.8642012E38F);
        });
        LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.y_SET(1.2551236E38F) ;
        p64.time_usec_SET(4451298139285016736L) ;
        p64.z_SET(2.1139633E38F) ;
        p64.vz_SET(-3.2075566E38F) ;
        p64.vy_SET(2.8642012E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION) ;
        p64.covariance_SET(new float[] {-1.549376E38F, -1.2662383E38F, 2.4615295E37F, 3.273793E38F, -2.7277991E38F, 1.6971092E38F, 2.7630818E37F, 8.812279E37F, 5.1041196E37F, 3.2920766E38F, -2.2841646E38F, 7.7206686E37F, -3.9490947E37F, 5.6902067E37F, 2.3817153E38F, -8.677272E37F, 3.0767444E38F, -2.2693434E38F, 2.3939093E38F, -2.3527593E38F, -3.0100053E38F, 2.680794E38F, 3.1813499E38F, 1.2670784E38F, -5.5877085E37F, 3.1886865E38F, 2.909961E38F, -2.946196E38F, 2.7349227E38F, -2.8092318E38F, -1.5136416E38F, -3.3147761E38F, 2.8972402E38F, 2.0911375E38F, -1.9261263E38F, -2.3794725E38F, -2.0831556E38F, 2.8052319E38F, -1.1744347E38F, 2.1354177E38F, -2.4713252E38F, -3.0197275E38F, -1.6592161E38F, -1.5808022E38F, -1.6805539E38F}, 0) ;
        p64.ax_SET(-6.2661166E37F) ;
        p64.vx_SET(-2.7455844E38F) ;
        p64.az_SET(-2.5728403E36F) ;
        p64.ay_SET(-7.3904025E37F) ;
        p64.x_SET(-7.6148263E37F) ;
        TestChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan15_raw_GET() == (char)59893);
            assert(pack.chan4_raw_GET() == (char)11375);
            assert(pack.chan9_raw_GET() == (char)32301);
            assert(pack.chan7_raw_GET() == (char)62377);
            assert(pack.chan18_raw_GET() == (char)61437);
            assert(pack.chan1_raw_GET() == (char)38189);
            assert(pack.chan3_raw_GET() == (char)36546);
            assert(pack.chan13_raw_GET() == (char)25220);
            assert(pack.chan10_raw_GET() == (char)52085);
            assert(pack.chan17_raw_GET() == (char)49484);
            assert(pack.chancount_GET() == (char)225);
            assert(pack.chan6_raw_GET() == (char)30966);
            assert(pack.chan8_raw_GET() == (char)28643);
            assert(pack.chan16_raw_GET() == (char)731);
            assert(pack.rssi_GET() == (char)193);
            assert(pack.chan12_raw_GET() == (char)19366);
            assert(pack.time_boot_ms_GET() == 2980414448L);
            assert(pack.chan14_raw_GET() == (char)26661);
            assert(pack.chan11_raw_GET() == (char)46938);
            assert(pack.chan2_raw_GET() == (char)59659);
            assert(pack.chan5_raw_GET() == (char)4569);
        });
        RC_CHANNELS p65 = new RC_CHANNELS();
        PH.setPack(p65);
        p65.chan2_raw_SET((char)59659) ;
        p65.chan5_raw_SET((char)4569) ;
        p65.chan4_raw_SET((char)11375) ;
        p65.chancount_SET((char)225) ;
        p65.chan16_raw_SET((char)731) ;
        p65.chan11_raw_SET((char)46938) ;
        p65.chan14_raw_SET((char)26661) ;
        p65.chan6_raw_SET((char)30966) ;
        p65.chan13_raw_SET((char)25220) ;
        p65.chan8_raw_SET((char)28643) ;
        p65.chan9_raw_SET((char)32301) ;
        p65.chan17_raw_SET((char)49484) ;
        p65.chan12_raw_SET((char)19366) ;
        p65.rssi_SET((char)193) ;
        p65.chan1_raw_SET((char)38189) ;
        p65.chan10_raw_SET((char)52085) ;
        p65.time_boot_ms_SET(2980414448L) ;
        p65.chan7_raw_SET((char)62377) ;
        p65.chan18_raw_SET((char)61437) ;
        p65.chan15_raw_SET((char)59893) ;
        p65.chan3_raw_SET((char)36546) ;
        TestChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.start_stop_GET() == (char)219);
            assert(pack.req_message_rate_GET() == (char)54868);
            assert(pack.target_system_GET() == (char)122);
            assert(pack.req_stream_id_GET() == (char)175);
            assert(pack.target_component_GET() == (char)199);
        });
        REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.start_stop_SET((char)219) ;
        p66.target_component_SET((char)199) ;
        p66.req_stream_id_SET((char)175) ;
        p66.target_system_SET((char)122) ;
        p66.req_message_rate_SET((char)54868) ;
        TestChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.message_rate_GET() == (char)62057);
            assert(pack.stream_id_GET() == (char)143);
            assert(pack.on_off_GET() == (char)223);
        });
        DATA_STREAM p67 = new DATA_STREAM();
        PH.setPack(p67);
        p67.on_off_SET((char)223) ;
        p67.stream_id_SET((char)143) ;
        p67.message_rate_SET((char)62057) ;
        TestChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == (short) -14797);
            assert(pack.target_GET() == (char)129);
            assert(pack.x_GET() == (short) -19198);
            assert(pack.z_GET() == (short)6044);
            assert(pack.buttons_GET() == (char)60974);
            assert(pack.r_GET() == (short)26799);
        });
        MANUAL_CONTROL p69 = new MANUAL_CONTROL();
        PH.setPack(p69);
        p69.z_SET((short)6044) ;
        p69.y_SET((short) -14797) ;
        p69.r_SET((short)26799) ;
        p69.target_SET((char)129) ;
        p69.buttons_SET((char)60974) ;
        p69.x_SET((short) -19198) ;
        TestChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)62811);
            assert(pack.chan8_raw_GET() == (char)11865);
            assert(pack.chan1_raw_GET() == (char)38160);
            assert(pack.chan2_raw_GET() == (char)37320);
            assert(pack.chan7_raw_GET() == (char)8734);
            assert(pack.target_system_GET() == (char)216);
            assert(pack.chan6_raw_GET() == (char)13636);
            assert(pack.chan3_raw_GET() == (char)34723);
            assert(pack.target_component_GET() == (char)5);
            assert(pack.chan5_raw_GET() == (char)51263);
        });
        RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.target_system_SET((char)216) ;
        p70.chan4_raw_SET((char)62811) ;
        p70.target_component_SET((char)5) ;
        p70.chan7_raw_SET((char)8734) ;
        p70.chan2_raw_SET((char)37320) ;
        p70.chan6_raw_SET((char)13636) ;
        p70.chan8_raw_SET((char)11865) ;
        p70.chan1_raw_SET((char)38160) ;
        p70.chan5_raw_SET((char)51263) ;
        p70.chan3_raw_SET((char)34723) ;
        TestChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.current_GET() == (char)107);
            assert(pack.param3_GET() == 2.1715498E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.autocontinue_GET() == (char)110);
            assert(pack.y_GET() == -1589662339);
            assert(pack.param1_GET() == 1.7367278E38F);
            assert(pack.z_GET() == -2.8937494E38F);
            assert(pack.seq_GET() == (char)18554);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF);
            assert(pack.target_system_GET() == (char)76);
            assert(pack.target_component_GET() == (char)58);
            assert(pack.param4_GET() == 1.5910272E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.x_GET() == 1872201707);
            assert(pack.param2_GET() == 1.8938777E37F);
        });
        MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.z_SET(-2.8937494E38F) ;
        p73.param3_SET(2.1715498E38F) ;
        p73.autocontinue_SET((char)110) ;
        p73.x_SET(1872201707) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p73.seq_SET((char)18554) ;
        p73.param4_SET(1.5910272E38F) ;
        p73.target_component_SET((char)58) ;
        p73.param2_SET(1.8938777E37F) ;
        p73.target_system_SET((char)76) ;
        p73.param1_SET(1.7367278E38F) ;
        p73.y_SET(-1589662339) ;
        p73.command_SET(MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF) ;
        p73.current_SET((char)107) ;
        TestChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.climb_GET() == 1.0243803E38F);
            assert(pack.alt_GET() == 7.056822E37F);
            assert(pack.heading_GET() == (short) -5819);
            assert(pack.airspeed_GET() == -2.9075536E38F);
            assert(pack.throttle_GET() == (char)37528);
            assert(pack.groundspeed_GET() == 5.950606E37F);
        });
        VFR_HUD p74 = new VFR_HUD();
        PH.setPack(p74);
        p74.throttle_SET((char)37528) ;
        p74.heading_SET((short) -5819) ;
        p74.alt_SET(7.056822E37F) ;
        p74.climb_SET(1.0243803E38F) ;
        p74.airspeed_SET(-2.9075536E38F) ;
        p74.groundspeed_SET(5.950606E37F) ;
        TestChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.param2_GET() == 8.433176E37F);
            assert(pack.param3_GET() == 2.4586173E38F);
            assert(pack.param4_GET() == 9.1200115E36F);
            assert(pack.target_component_GET() == (char)37);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.param1_GET() == 1.6240885E38F);
            assert(pack.y_GET() == 215583455);
            assert(pack.autocontinue_GET() == (char)187);
            assert(pack.x_GET() == 1856766504);
            assert(pack.z_GET() == -3.0862116E38F);
            assert(pack.target_system_GET() == (char)225);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED);
            assert(pack.current_GET() == (char)216);
        });
        COMMAND_INT p75 = new COMMAND_INT();
        PH.setPack(p75);
        p75.x_SET(1856766504) ;
        p75.y_SET(215583455) ;
        p75.param2_SET(8.433176E37F) ;
        p75.target_system_SET((char)225) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p75.param1_SET(1.6240885E38F) ;
        p75.target_component_SET((char)37) ;
        p75.current_SET((char)216) ;
        p75.command_SET(MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED) ;
        p75.param3_SET(2.4586173E38F) ;
        p75.param4_SET(9.1200115E36F) ;
        p75.z_SET(-3.0862116E38F) ;
        p75.autocontinue_SET((char)187) ;
        TestChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(TestChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param5_GET() == 1.8058622E38F);
            assert(pack.param7_GET() == 2.8064393E38F);
            assert(pack.param4_GET() == 2.111022E38F);
            assert(pack.target_system_GET() == (char)17);
            assert(pack.param2_GET() == 5.802978E37F);
            assert(pack.target_component_GET() == (char)233);
            assert(pack.param1_GET() == 1.5760344E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_USER_1);
            assert(pack.param3_GET() == 8.2654313E37F);
            assert(pack.param6_GET() == -4.067941E37F);
            assert(pack.confirmation_GET() == (char)246);
        });
        GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param3_SET(8.2654313E37F) ;
        p76.param6_SET(-4.067941E37F) ;
        p76.param4_SET(2.111022E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_USER_1) ;
        p76.target_system_SET((char)17) ;
        p76.param5_SET(1.8058622E38F) ;
        p76.param7_SET(2.8064393E38F) ;
        p76.target_component_SET((char)233) ;
        p76.confirmation_SET((char)246) ;
        p76.param1_SET(1.5760344E38F) ;
        p76.param2_SET(5.802978E37F) ;
        CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_TRY(ph) == (char)115);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_CHANGE_SPEED);
            assert(pack.result_param2_TRY(ph) == 1079059562);
            assert(pack.target_system_TRY(ph) == (char)138);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_FAILED);
            assert(pack.progress_TRY(ph) == (char)10);
        });
        GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.target_system_SET((char)138, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_FAILED) ;
        p77.command_SET(MAV_CMD.MAV_CMD_DO_CHANGE_SPEED) ;
        p77.result_param2_SET(1079059562, PH) ;
        p77.target_component_SET((char)115, PH) ;
        p77.progress_SET((char)10, PH) ;
        CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.mode_switch_GET() == (char)140);
            assert(pack.pitch_GET() == 8.117874E37F);
            assert(pack.roll_GET() == 2.2182861E38F);
            assert(pack.manual_override_switch_GET() == (char)45);
            assert(pack.yaw_GET() == 1.8563623E38F);
            assert(pack.thrust_GET() == -1.3632685E38F);
            assert(pack.time_boot_ms_GET() == 275895071L);
        });
        GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.yaw_SET(1.8563623E38F) ;
        p81.pitch_SET(8.117874E37F) ;
        p81.manual_override_switch_SET((char)45) ;
        p81.thrust_SET(-1.3632685E38F) ;
        p81.mode_switch_SET((char)140) ;
        p81.roll_SET(2.2182861E38F) ;
        p81.time_boot_ms_SET(275895071L) ;
        CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3323432016L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.8325617E38F, 2.7297318E38F, -1.5189725E38F, -5.499172E36F}));
            assert(pack.thrust_GET() == -1.8871306E38F);
            assert(pack.body_pitch_rate_GET() == -1.389937E38F);
            assert(pack.target_system_GET() == (char)17);
            assert(pack.target_component_GET() == (char)172);
            assert(pack.body_roll_rate_GET() == -2.8715792E37F);
            assert(pack.body_yaw_rate_GET() == -1.7060334E38F);
            assert(pack.type_mask_GET() == (char)113);
        });
        GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.time_boot_ms_SET(3323432016L) ;
        p82.body_pitch_rate_SET(-1.389937E38F) ;
        p82.body_roll_rate_SET(-2.8715792E37F) ;
        p82.thrust_SET(-1.8871306E38F) ;
        p82.target_system_SET((char)17) ;
        p82.type_mask_SET((char)113) ;
        p82.target_component_SET((char)172) ;
        p82.body_yaw_rate_SET(-1.7060334E38F) ;
        p82.q_SET(new float[] {-1.8325617E38F, 2.7297318E38F, -1.5189725E38F, -5.499172E36F}, 0) ;
        CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)147);
            assert(pack.body_yaw_rate_GET() == 1.5970943E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.9567666E38F, 3.0494428E38F, 2.0626868E37F, -1.1521994E38F}));
            assert(pack.body_pitch_rate_GET() == 3.1853891E38F);
            assert(pack.time_boot_ms_GET() == 4166929035L);
            assert(pack.thrust_GET() == -1.4923346E38F);
            assert(pack.body_roll_rate_GET() == -8.0322814E37F);
        });
        GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_pitch_rate_SET(3.1853891E38F) ;
        p83.body_roll_rate_SET(-8.0322814E37F) ;
        p83.body_yaw_rate_SET(1.5970943E38F) ;
        p83.time_boot_ms_SET(4166929035L) ;
        p83.thrust_SET(-1.4923346E38F) ;
        p83.q_SET(new float[] {1.9567666E38F, 3.0494428E38F, 2.0626868E37F, -1.1521994E38F}, 0) ;
        p83.type_mask_SET((char)147) ;
        CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == -4.4216175E37F);
            assert(pack.type_mask_GET() == (char)41535);
            assert(pack.afz_GET() == -6.998284E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.target_system_GET() == (char)199);
            assert(pack.vz_GET() == 2.4027295E38F);
            assert(pack.y_GET() == 1.4318826E38F);
            assert(pack.afx_GET() == 5.3899946E36F);
            assert(pack.time_boot_ms_GET() == 2014417593L);
            assert(pack.x_GET() == 1.0632432E38F);
            assert(pack.yaw_GET() == -7.000613E37F);
            assert(pack.vx_GET() == -2.6341719E37F);
            assert(pack.afy_GET() == -9.46221E37F);
            assert(pack.z_GET() == -1.6167232E38F);
            assert(pack.target_component_GET() == (char)220);
            assert(pack.yaw_rate_GET() == 1.804126E38F);
        });
        GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.z_SET(-1.6167232E38F) ;
        p84.yaw_SET(-7.000613E37F) ;
        p84.y_SET(1.4318826E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p84.yaw_rate_SET(1.804126E38F) ;
        p84.target_system_SET((char)199) ;
        p84.afx_SET(5.3899946E36F) ;
        p84.afz_SET(-6.998284E37F) ;
        p84.type_mask_SET((char)41535) ;
        p84.time_boot_ms_SET(2014417593L) ;
        p84.x_SET(1.0632432E38F) ;
        p84.vy_SET(-4.4216175E37F) ;
        p84.target_component_SET((char)220) ;
        p84.afy_SET(-9.46221E37F) ;
        p84.vx_SET(-2.6341719E37F) ;
        p84.vz_SET(2.4027295E38F) ;
        CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == 1.2265999E38F);
            assert(pack.vz_GET() == 1.0173545E37F);
            assert(pack.type_mask_GET() == (char)38870);
            assert(pack.vy_GET() == -9.41926E36F);
            assert(pack.afy_GET() == 4.894253E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.yaw_GET() == -3.3955532E38F);
            assert(pack.afz_GET() == -1.0103202E37F);
            assert(pack.lat_int_GET() == -1978387883);
            assert(pack.target_component_GET() == (char)64);
            assert(pack.time_boot_ms_GET() == 445315657L);
            assert(pack.target_system_GET() == (char)48);
            assert(pack.alt_GET() == -1.799669E38F);
            assert(pack.vx_GET() == 2.182088E38F);
            assert(pack.lon_int_GET() == 1195051767);
            assert(pack.afx_GET() == 9.050431E37F);
        });
        GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.yaw_SET(-3.3955532E38F) ;
        p86.vz_SET(1.0173545E37F) ;
        p86.afx_SET(9.050431E37F) ;
        p86.type_mask_SET((char)38870) ;
        p86.alt_SET(-1.799669E38F) ;
        p86.afy_SET(4.894253E37F) ;
        p86.vx_SET(2.182088E38F) ;
        p86.yaw_rate_SET(1.2265999E38F) ;
        p86.lat_int_SET(-1978387883) ;
        p86.afz_SET(-1.0103202E37F) ;
        p86.target_component_SET((char)64) ;
        p86.vy_SET(-9.41926E36F) ;
        p86.lon_int_SET(1195051767) ;
        p86.target_system_SET((char)48) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p86.time_boot_ms_SET(445315657L) ;
        CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -1.8586545E38F);
            assert(pack.type_mask_GET() == (char)19835);
            assert(pack.alt_GET() == 3.3719985E38F);
            assert(pack.time_boot_ms_GET() == 2572885492L);
            assert(pack.lon_int_GET() == 435877750);
            assert(pack.afx_GET() == -1.2035417E38F);
            assert(pack.afz_GET() == 3.2885073E38F);
            assert(pack.afy_GET() == 2.8327693E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.vx_GET() == 8.350567E37F);
            assert(pack.yaw_rate_GET() == -1.809084E37F);
            assert(pack.lat_int_GET() == -1746461648);
            assert(pack.vy_GET() == 2.6305395E38F);
            assert(pack.vz_GET() == -5.887192E37F);
        });
        GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p87.alt_SET(3.3719985E38F) ;
        p87.yaw_SET(-1.8586545E38F) ;
        p87.afx_SET(-1.2035417E38F) ;
        p87.time_boot_ms_SET(2572885492L) ;
        p87.type_mask_SET((char)19835) ;
        p87.afy_SET(2.8327693E38F) ;
        p87.afz_SET(3.2885073E38F) ;
        p87.lon_int_SET(435877750) ;
        p87.vz_SET(-5.887192E37F) ;
        p87.vx_SET(8.350567E37F) ;
        p87.vy_SET(2.6305395E38F) ;
        p87.yaw_rate_SET(-1.809084E37F) ;
        p87.lat_int_SET(-1746461648) ;
        CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 7.971383E37F);
            assert(pack.y_GET() == -3.3155047E38F);
            assert(pack.roll_GET() == 7.589932E37F);
            assert(pack.yaw_GET() == -3.0016283E36F);
            assert(pack.pitch_GET() == -1.5611476E38F);
            assert(pack.time_boot_ms_GET() == 2803424334L);
            assert(pack.z_GET() == 1.6702743E38F);
        });
        GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.z_SET(1.6702743E38F) ;
        p89.time_boot_ms_SET(2803424334L) ;
        p89.yaw_SET(-3.0016283E36F) ;
        p89.y_SET(-3.3155047E38F) ;
        p89.pitch_SET(-1.5611476E38F) ;
        p89.roll_SET(7.589932E37F) ;
        p89.x_SET(7.971383E37F) ;
        CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 3.1033202E38F);
            assert(pack.zacc_GET() == (short)23511);
            assert(pack.pitchspeed_GET() == -2.9293312E38F);
            assert(pack.rollspeed_GET() == -1.5255688E38F);
            assert(pack.lon_GET() == 61536289);
            assert(pack.yawspeed_GET() == -8.687689E36F);
            assert(pack.vz_GET() == (short) -32651);
            assert(pack.vx_GET() == (short) -32589);
            assert(pack.vy_GET() == (short) -31021);
            assert(pack.lat_GET() == 597908813);
            assert(pack.roll_GET() == 7.693417E37F);
            assert(pack.pitch_GET() == 2.5254663E38F);
            assert(pack.alt_GET() == 295547103);
            assert(pack.time_usec_GET() == 3120878908825930754L);
            assert(pack.xacc_GET() == (short) -19224);
            assert(pack.yacc_GET() == (short)19051);
        });
        GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.rollspeed_SET(-1.5255688E38F) ;
        p90.vy_SET((short) -31021) ;
        p90.yawspeed_SET(-8.687689E36F) ;
        p90.vz_SET((short) -32651) ;
        p90.time_usec_SET(3120878908825930754L) ;
        p90.vx_SET((short) -32589) ;
        p90.pitch_SET(2.5254663E38F) ;
        p90.roll_SET(7.693417E37F) ;
        p90.alt_SET(295547103) ;
        p90.pitchspeed_SET(-2.9293312E38F) ;
        p90.yacc_SET((short)19051) ;
        p90.lat_SET(597908813) ;
        p90.lon_SET(61536289) ;
        p90.xacc_SET((short) -19224) ;
        p90.zacc_SET((short)23511) ;
        p90.yaw_SET(3.1033202E38F) ;
        CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3761760854001586525L);
            assert(pack.aux4_GET() == 9.334421E36F);
            assert(pack.nav_mode_GET() == (char)104);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_MANUAL_DISARMED);
            assert(pack.roll_ailerons_GET() == -1.3449822E38F);
            assert(pack.throttle_GET() == 2.6496745E37F);
            assert(pack.aux3_GET() == -1.1459031E38F);
            assert(pack.yaw_rudder_GET() == -1.5666313E38F);
            assert(pack.aux1_GET() == -3.7499227E37F);
            assert(pack.aux2_GET() == 6.4123477E37F);
            assert(pack.pitch_elevator_GET() == -1.9189687E35F);
        });
        GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.aux3_SET(-1.1459031E38F) ;
        p91.nav_mode_SET((char)104) ;
        p91.aux4_SET(9.334421E36F) ;
        p91.roll_ailerons_SET(-1.3449822E38F) ;
        p91.yaw_rudder_SET(-1.5666313E38F) ;
        p91.aux2_SET(6.4123477E37F) ;
        p91.throttle_SET(2.6496745E37F) ;
        p91.time_usec_SET(3761760854001586525L) ;
        p91.aux1_SET(-3.7499227E37F) ;
        p91.pitch_elevator_SET(-1.9189687E35F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_MANUAL_DISARMED) ;
        CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)51125);
            assert(pack.chan3_raw_GET() == (char)58978);
            assert(pack.chan10_raw_GET() == (char)37960);
            assert(pack.chan6_raw_GET() == (char)17916);
            assert(pack.chan11_raw_GET() == (char)5126);
            assert(pack.chan1_raw_GET() == (char)3482);
            assert(pack.chan5_raw_GET() == (char)855);
            assert(pack.rssi_GET() == (char)230);
            assert(pack.chan2_raw_GET() == (char)36693);
            assert(pack.chan9_raw_GET() == (char)59227);
            assert(pack.chan12_raw_GET() == (char)62947);
            assert(pack.time_usec_GET() == 8321985371127661065L);
            assert(pack.chan8_raw_GET() == (char)13034);
            assert(pack.chan7_raw_GET() == (char)25654);
        });
        GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan12_raw_SET((char)62947) ;
        p92.chan7_raw_SET((char)25654) ;
        p92.rssi_SET((char)230) ;
        p92.chan6_raw_SET((char)17916) ;
        p92.chan5_raw_SET((char)855) ;
        p92.chan8_raw_SET((char)13034) ;
        p92.chan3_raw_SET((char)58978) ;
        p92.chan1_raw_SET((char)3482) ;
        p92.chan9_raw_SET((char)59227) ;
        p92.chan4_raw_SET((char)51125) ;
        p92.chan10_raw_SET((char)37960) ;
        p92.time_usec_SET(8321985371127661065L) ;
        p92.chan11_raw_SET((char)5126) ;
        p92.chan2_raw_SET((char)36693) ;
        CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == 2396798418854118838L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_MANUAL_DISARMED);
            assert(pack.time_usec_GET() == 111121396554335921L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.06939E38F, -1.069991E38F, -7.3393207E37F, 4.36436E37F, -1.2082261E38F, 2.3028597E38F, -5.720822E37F, -2.3090511E38F, 1.9657097E38F, 1.3421605E38F, -2.5953532E38F, 3.278652E38F, -1.4263453E38F, -2.4970735E38F, 9.747138E37F, 2.6048119E38F}));
        });
        GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.flags_SET(2396798418854118838L) ;
        p93.time_usec_SET(111121396554335921L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_MANUAL_DISARMED) ;
        p93.controls_SET(new float[] {2.06939E38F, -1.069991E38F, -7.3393207E37F, 4.36436E37F, -1.2082261E38F, 2.3028597E38F, -5.720822E37F, -2.3090511E38F, 1.9657097E38F, 1.3421605E38F, -2.5953532E38F, 3.278652E38F, -1.4263453E38F, -2.4970735E38F, 9.747138E37F, 2.6048119E38F}, 0) ;
        CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_rate_y_TRY(ph) == -3.3134517E38F);
            assert(pack.time_usec_GET() == 6716098455947164057L);
            assert(pack.sensor_id_GET() == (char)214);
            assert(pack.quality_GET() == (char)155);
            assert(pack.flow_comp_m_y_GET() == -2.0193542E38F);
            assert(pack.flow_rate_x_TRY(ph) == -9.741065E37F);
            assert(pack.flow_comp_m_x_GET() == -3.2584316E38F);
            assert(pack.ground_distance_GET() == -1.3658576E38F);
            assert(pack.flow_y_GET() == (short)19660);
            assert(pack.flow_x_GET() == (short) -29990);
        });
        GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(6716098455947164057L) ;
        p100.flow_comp_m_y_SET(-2.0193542E38F) ;
        p100.flow_x_SET((short) -29990) ;
        p100.ground_distance_SET(-1.3658576E38F) ;
        p100.flow_y_SET((short)19660) ;
        p100.flow_comp_m_x_SET(-3.2584316E38F) ;
        p100.sensor_id_SET((char)214) ;
        p100.quality_SET((char)155) ;
        p100.flow_rate_y_SET(-3.3134517E38F, PH) ;
        p100.flow_rate_x_SET(-9.741065E37F, PH) ;
        CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 3197381030365976066L);
            assert(pack.yaw_GET() == -1.2191577E38F);
            assert(pack.pitch_GET() == -3.0255368E38F);
            assert(pack.y_GET() == 7.6575533E37F);
            assert(pack.roll_GET() == 2.1423153E38F);
            assert(pack.x_GET() == -7.5289537E37F);
            assert(pack.z_GET() == -2.7479312E38F);
        });
        GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.x_SET(-7.5289537E37F) ;
        p101.z_SET(-2.7479312E38F) ;
        p101.usec_SET(3197381030365976066L) ;
        p101.pitch_SET(-3.0255368E38F) ;
        p101.y_SET(7.6575533E37F) ;
        p101.roll_SET(2.1423153E38F) ;
        p101.yaw_SET(-1.2191577E38F) ;
        CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 6.449002E37F);
            assert(pack.x_GET() == 2.6499652E38F);
            assert(pack.usec_GET() == 9192463734768619325L);
            assert(pack.yaw_GET() == -6.464749E37F);
            assert(pack.pitch_GET() == -1.6563621E38F);
            assert(pack.y_GET() == 2.193618E38F);
            assert(pack.z_GET() == 2.6631493E38F);
        });
        GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.x_SET(2.6499652E38F) ;
        p102.y_SET(2.193618E38F) ;
        p102.pitch_SET(-1.6563621E38F) ;
        p102.usec_SET(9192463734768619325L) ;
        p102.yaw_SET(-6.464749E37F) ;
        p102.z_SET(2.6631493E38F) ;
        p102.roll_SET(6.449002E37F) ;
        CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 2.8229837E38F);
            assert(pack.z_GET() == 5.5073136E37F);
            assert(pack.usec_GET() == 1833462786201890297L);
            assert(pack.y_GET() == -2.9393618E37F);
        });
        GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.y_SET(-2.9393618E37F) ;
        p103.x_SET(2.8229837E38F) ;
        p103.usec_SET(1833462786201890297L) ;
        p103.z_SET(5.5073136E37F) ;
        CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.8711263E38F);
            assert(pack.y_GET() == 3.2036334E38F);
            assert(pack.pitch_GET() == -3.321892E37F);
            assert(pack.roll_GET() == 3.4531748E37F);
            assert(pack.yaw_GET() == 7.1257226E37F);
            assert(pack.usec_GET() == 203753041253797681L);
            assert(pack.x_GET() == -2.3639177E38F);
        });
        GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(203753041253797681L) ;
        p104.roll_SET(3.4531748E37F) ;
        p104.x_SET(-2.3639177E38F) ;
        p104.y_SET(3.2036334E38F) ;
        p104.z_SET(-1.8711263E38F) ;
        p104.pitch_SET(-3.321892E37F) ;
        p104.yaw_SET(7.1257226E37F) ;
        CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.diff_pressure_GET() == -2.6646322E38F);
            assert(pack.fields_updated_GET() == (char)7092);
            assert(pack.pressure_alt_GET() == 1.4860419E38F);
            assert(pack.abs_pressure_GET() == 6.4662893E37F);
            assert(pack.zmag_GET() == 2.9269929E38F);
            assert(pack.xgyro_GET() == 1.3618458E38F);
            assert(pack.yacc_GET() == -1.5308956E38F);
            assert(pack.ymag_GET() == -2.683902E38F);
            assert(pack.xmag_GET() == 2.1690815E37F);
            assert(pack.time_usec_GET() == 8027049397941788844L);
            assert(pack.temperature_GET() == -2.7688533E38F);
            assert(pack.zgyro_GET() == 9.131148E37F);
            assert(pack.ygyro_GET() == -2.7321091E38F);
            assert(pack.xacc_GET() == -7.358623E37F);
            assert(pack.zacc_GET() == -1.5648236E38F);
        });
        GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.xgyro_SET(1.3618458E38F) ;
        p105.zacc_SET(-1.5648236E38F) ;
        p105.abs_pressure_SET(6.4662893E37F) ;
        p105.fields_updated_SET((char)7092) ;
        p105.xacc_SET(-7.358623E37F) ;
        p105.pressure_alt_SET(1.4860419E38F) ;
        p105.zmag_SET(2.9269929E38F) ;
        p105.zgyro_SET(9.131148E37F) ;
        p105.time_usec_SET(8027049397941788844L) ;
        p105.diff_pressure_SET(-2.6646322E38F) ;
        p105.ymag_SET(-2.683902E38F) ;
        p105.xmag_SET(2.1690815E37F) ;
        p105.ygyro_SET(-2.7321091E38F) ;
        p105.yacc_SET(-1.5308956E38F) ;
        p105.temperature_SET(-2.7688533E38F) ;
        CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integration_time_us_GET() == 2896712551L);
            assert(pack.integrated_ygyro_GET() == -2.6077984E38F);
            assert(pack.quality_GET() == (char)185);
            assert(pack.integrated_xgyro_GET() == -9.974249E37F);
            assert(pack.time_usec_GET() == 3245760095867939684L);
            assert(pack.integrated_x_GET() == 2.92986E38F);
            assert(pack.integrated_y_GET() == 2.5106917E38F);
            assert(pack.temperature_GET() == (short)15834);
            assert(pack.integrated_zgyro_GET() == -3.0888579E38F);
            assert(pack.distance_GET() == 1.7290014E38F);
            assert(pack.sensor_id_GET() == (char)150);
            assert(pack.time_delta_distance_us_GET() == 3832031977L);
        });
        GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_delta_distance_us_SET(3832031977L) ;
        p106.sensor_id_SET((char)150) ;
        p106.time_usec_SET(3245760095867939684L) ;
        p106.integration_time_us_SET(2896712551L) ;
        p106.integrated_y_SET(2.5106917E38F) ;
        p106.integrated_x_SET(2.92986E38F) ;
        p106.integrated_xgyro_SET(-9.974249E37F) ;
        p106.distance_SET(1.7290014E38F) ;
        p106.quality_SET((char)185) ;
        p106.integrated_ygyro_SET(-2.6077984E38F) ;
        p106.integrated_zgyro_SET(-3.0888579E38F) ;
        p106.temperature_SET((short)15834) ;
        CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.abs_pressure_GET() == 2.0091246E38F);
            assert(pack.zacc_GET() == -1.3453721E38F);
            assert(pack.zgyro_GET() == -2.3226345E38F);
            assert(pack.xacc_GET() == 4.2026718E36F);
            assert(pack.ymag_GET() == -2.9251036E38F);
            assert(pack.temperature_GET() == 1.2455262E38F);
            assert(pack.xmag_GET() == 2.1636585E38F);
            assert(pack.pressure_alt_GET() == 3.02564E38F);
            assert(pack.ygyro_GET() == 1.0654706E38F);
            assert(pack.diff_pressure_GET() == -1.3358242E38F);
            assert(pack.xgyro_GET() == -5.7625515E37F);
            assert(pack.zmag_GET() == -2.1389127E38F);
            assert(pack.fields_updated_GET() == 1422889757L);
            assert(pack.time_usec_GET() == 977391029078024472L);
            assert(pack.yacc_GET() == 1.5829008E38F);
        });
        GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.fields_updated_SET(1422889757L) ;
        p107.zmag_SET(-2.1389127E38F) ;
        p107.zacc_SET(-1.3453721E38F) ;
        p107.zgyro_SET(-2.3226345E38F) ;
        p107.xgyro_SET(-5.7625515E37F) ;
        p107.diff_pressure_SET(-1.3358242E38F) ;
        p107.pressure_alt_SET(3.02564E38F) ;
        p107.time_usec_SET(977391029078024472L) ;
        p107.xacc_SET(4.2026718E36F) ;
        p107.ygyro_SET(1.0654706E38F) ;
        p107.temperature_SET(1.2455262E38F) ;
        p107.ymag_SET(-2.9251036E38F) ;
        p107.xmag_SET(2.1636585E38F) ;
        p107.abs_pressure_SET(2.0091246E38F) ;
        p107.yacc_SET(1.5829008E38F) ;
        CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == 4.697628E37F);
            assert(pack.q2_GET() == 2.3784151E38F);
            assert(pack.std_dev_horz_GET() == 4.794143E37F);
            assert(pack.ygyro_GET() == 5.9565436E37F);
            assert(pack.q4_GET() == -2.4961075E38F);
            assert(pack.yaw_GET() == -1.7032064E38F);
            assert(pack.yacc_GET() == 3.1784663E37F);
            assert(pack.zacc_GET() == 1.7319825E38F);
            assert(pack.xacc_GET() == -1.6138113E38F);
            assert(pack.roll_GET() == 1.6210194E38F);
            assert(pack.ve_GET() == 2.425266E38F);
            assert(pack.std_dev_vert_GET() == 3.0850894E38F);
            assert(pack.vd_GET() == 1.757512E38F);
            assert(pack.q3_GET() == 2.5881975E38F);
            assert(pack.xgyro_GET() == -1.758804E38F);
            assert(pack.vn_GET() == -1.3638736E38F);
            assert(pack.alt_GET() == 4.664825E37F);
            assert(pack.pitch_GET() == 4.7119044E37F);
            assert(pack.q1_GET() == -1.5514179E38F);
            assert(pack.lat_GET() == -2.187191E38F);
            assert(pack.lon_GET() == -2.1537915E38F);
        });
        GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.lon_SET(-2.1537915E38F) ;
        p108.std_dev_vert_SET(3.0850894E38F) ;
        p108.xacc_SET(-1.6138113E38F) ;
        p108.q1_SET(-1.5514179E38F) ;
        p108.vd_SET(1.757512E38F) ;
        p108.zacc_SET(1.7319825E38F) ;
        p108.lat_SET(-2.187191E38F) ;
        p108.ve_SET(2.425266E38F) ;
        p108.alt_SET(4.664825E37F) ;
        p108.q3_SET(2.5881975E38F) ;
        p108.zgyro_SET(4.697628E37F) ;
        p108.ygyro_SET(5.9565436E37F) ;
        p108.roll_SET(1.6210194E38F) ;
        p108.q2_SET(2.3784151E38F) ;
        p108.std_dev_horz_SET(4.794143E37F) ;
        p108.pitch_SET(4.7119044E37F) ;
        p108.xgyro_SET(-1.758804E38F) ;
        p108.yacc_SET(3.1784663E37F) ;
        p108.q4_SET(-2.4961075E38F) ;
        p108.vn_SET(-1.3638736E38F) ;
        p108.yaw_SET(-1.7032064E38F) ;
        CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.txbuf_GET() == (char)78);
            assert(pack.remnoise_GET() == (char)206);
            assert(pack.rssi_GET() == (char)14);
            assert(pack.noise_GET() == (char)105);
            assert(pack.fixed__GET() == (char)12675);
            assert(pack.rxerrors_GET() == (char)17393);
            assert(pack.remrssi_GET() == (char)134);
        });
        GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)14) ;
        p109.fixed__SET((char)12675) ;
        p109.rxerrors_SET((char)17393) ;
        p109.remnoise_SET((char)206) ;
        p109.remrssi_SET((char)134) ;
        p109.noise_SET((char)105) ;
        p109.txbuf_SET((char)78) ;
        CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)187);
            assert(pack.target_component_GET() == (char)147);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)205, (char)34, (char)171, (char)113, (char)89, (char)16, (char)172, (char)104, (char)214, (char)168, (char)1, (char)13, (char)29, (char)228, (char)20, (char)194, (char)83, (char)195, (char)59, (char)127, (char)51, (char)53, (char)226, (char)13, (char)114, (char)32, (char)16, (char)22, (char)35, (char)81, (char)233, (char)87, (char)9, (char)75, (char)38, (char)27, (char)122, (char)7, (char)246, (char)80, (char)34, (char)237, (char)249, (char)192, (char)72, (char)45, (char)115, (char)230, (char)63, (char)48, (char)52, (char)112, (char)117, (char)158, (char)164, (char)245, (char)237, (char)57, (char)126, (char)85, (char)215, (char)44, (char)116, (char)72, (char)46, (char)188, (char)44, (char)30, (char)72, (char)3, (char)50, (char)71, (char)241, (char)6, (char)194, (char)252, (char)50, (char)202, (char)130, (char)250, (char)182, (char)113, (char)21, (char)211, (char)7, (char)237, (char)207, (char)66, (char)94, (char)1, (char)219, (char)103, (char)91, (char)21, (char)130, (char)168, (char)110, (char)139, (char)190, (char)255, (char)168, (char)24, (char)104, (char)234, (char)85, (char)57, (char)188, (char)61, (char)170, (char)194, (char)143, (char)75, (char)149, (char)48, (char)243, (char)223, (char)112, (char)36, (char)127, (char)157, (char)56, (char)226, (char)153, (char)177, (char)75, (char)18, (char)119, (char)48, (char)205, (char)76, (char)242, (char)171, (char)205, (char)24, (char)29, (char)103, (char)144, (char)226, (char)198, (char)147, (char)219, (char)173, (char)23, (char)214, (char)227, (char)125, (char)119, (char)155, (char)212, (char)246, (char)129, (char)76, (char)210, (char)4, (char)5, (char)154, (char)54, (char)37, (char)255, (char)172, (char)80, (char)251, (char)87, (char)249, (char)90, (char)2, (char)78, (char)45, (char)72, (char)205, (char)208, (char)104, (char)196, (char)125, (char)244, (char)214, (char)162, (char)7, (char)214, (char)214, (char)26, (char)145, (char)228, (char)58, (char)239, (char)47, (char)201, (char)156, (char)207, (char)139, (char)159, (char)59, (char)112, (char)117, (char)237, (char)4, (char)197, (char)116, (char)44, (char)219, (char)63, (char)68, (char)192, (char)179, (char)52, (char)12, (char)187, (char)3, (char)10, (char)155, (char)248, (char)149, (char)229, (char)49, (char)130, (char)204, (char)194, (char)149, (char)187, (char)54, (char)156, (char)181, (char)13, (char)250, (char)123, (char)164, (char)193, (char)32, (char)136, (char)117, (char)154, (char)128, (char)52, (char)27, (char)118, (char)74, (char)218, (char)182, (char)148, (char)169, (char)197, (char)177, (char)119, (char)191, (char)65, (char)28, (char)42, (char)95, (char)22, (char)39, (char)213}));
            assert(pack.target_system_GET() == (char)226);
        });
        GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_component_SET((char)147) ;
        p110.target_network_SET((char)187) ;
        p110.payload_SET(new char[] {(char)205, (char)34, (char)171, (char)113, (char)89, (char)16, (char)172, (char)104, (char)214, (char)168, (char)1, (char)13, (char)29, (char)228, (char)20, (char)194, (char)83, (char)195, (char)59, (char)127, (char)51, (char)53, (char)226, (char)13, (char)114, (char)32, (char)16, (char)22, (char)35, (char)81, (char)233, (char)87, (char)9, (char)75, (char)38, (char)27, (char)122, (char)7, (char)246, (char)80, (char)34, (char)237, (char)249, (char)192, (char)72, (char)45, (char)115, (char)230, (char)63, (char)48, (char)52, (char)112, (char)117, (char)158, (char)164, (char)245, (char)237, (char)57, (char)126, (char)85, (char)215, (char)44, (char)116, (char)72, (char)46, (char)188, (char)44, (char)30, (char)72, (char)3, (char)50, (char)71, (char)241, (char)6, (char)194, (char)252, (char)50, (char)202, (char)130, (char)250, (char)182, (char)113, (char)21, (char)211, (char)7, (char)237, (char)207, (char)66, (char)94, (char)1, (char)219, (char)103, (char)91, (char)21, (char)130, (char)168, (char)110, (char)139, (char)190, (char)255, (char)168, (char)24, (char)104, (char)234, (char)85, (char)57, (char)188, (char)61, (char)170, (char)194, (char)143, (char)75, (char)149, (char)48, (char)243, (char)223, (char)112, (char)36, (char)127, (char)157, (char)56, (char)226, (char)153, (char)177, (char)75, (char)18, (char)119, (char)48, (char)205, (char)76, (char)242, (char)171, (char)205, (char)24, (char)29, (char)103, (char)144, (char)226, (char)198, (char)147, (char)219, (char)173, (char)23, (char)214, (char)227, (char)125, (char)119, (char)155, (char)212, (char)246, (char)129, (char)76, (char)210, (char)4, (char)5, (char)154, (char)54, (char)37, (char)255, (char)172, (char)80, (char)251, (char)87, (char)249, (char)90, (char)2, (char)78, (char)45, (char)72, (char)205, (char)208, (char)104, (char)196, (char)125, (char)244, (char)214, (char)162, (char)7, (char)214, (char)214, (char)26, (char)145, (char)228, (char)58, (char)239, (char)47, (char)201, (char)156, (char)207, (char)139, (char)159, (char)59, (char)112, (char)117, (char)237, (char)4, (char)197, (char)116, (char)44, (char)219, (char)63, (char)68, (char)192, (char)179, (char)52, (char)12, (char)187, (char)3, (char)10, (char)155, (char)248, (char)149, (char)229, (char)49, (char)130, (char)204, (char)194, (char)149, (char)187, (char)54, (char)156, (char)181, (char)13, (char)250, (char)123, (char)164, (char)193, (char)32, (char)136, (char)117, (char)154, (char)128, (char)52, (char)27, (char)118, (char)74, (char)218, (char)182, (char)148, (char)169, (char)197, (char)177, (char)119, (char)191, (char)65, (char)28, (char)42, (char)95, (char)22, (char)39, (char)213}, 0) ;
        p110.target_system_SET((char)226) ;
        CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == 8268536991358059528L);
            assert(pack.tc1_GET() == -7419328954789676192L);
        });
        GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(8268536991358059528L) ;
        p111.tc1_SET(-7419328954789676192L) ;
        CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 2521239176L);
            assert(pack.time_usec_GET() == 8001614081465963530L);
        });
        GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(8001614081465963530L) ;
        p112.seq_SET(2521239176L) ;
        CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1196016004);
            assert(pack.vd_GET() == (short)4810);
            assert(pack.fix_type_GET() == (char)236);
            assert(pack.cog_GET() == (char)30486);
            assert(pack.epv_GET() == (char)60472);
            assert(pack.satellites_visible_GET() == (char)117);
            assert(pack.vn_GET() == (short)3350);
            assert(pack.time_usec_GET() == 1072967172994117846L);
            assert(pack.alt_GET() == -1340343587);
            assert(pack.lon_GET() == -812164562);
            assert(pack.ve_GET() == (short) -347);
            assert(pack.eph_GET() == (char)62374);
            assert(pack.vel_GET() == (char)32775);
        });
        GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.epv_SET((char)60472) ;
        p113.lat_SET(-1196016004) ;
        p113.vd_SET((short)4810) ;
        p113.lon_SET(-812164562) ;
        p113.time_usec_SET(1072967172994117846L) ;
        p113.fix_type_SET((char)236) ;
        p113.satellites_visible_SET((char)117) ;
        p113.ve_SET((short) -347) ;
        p113.eph_SET((char)62374) ;
        p113.vel_SET((char)32775) ;
        p113.alt_SET(-1340343587) ;
        p113.vn_SET((short)3350) ;
        p113.cog_SET((char)30486) ;
        CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_zgyro_GET() == 7.5248155E37F);
            assert(pack.distance_GET() == -2.4889226E38F);
            assert(pack.temperature_GET() == (short) -1958);
            assert(pack.time_delta_distance_us_GET() == 2690589417L);
            assert(pack.integrated_xgyro_GET() == 2.0437988E37F);
            assert(pack.sensor_id_GET() == (char)226);
            assert(pack.integrated_y_GET() == 1.983971E38F);
            assert(pack.time_usec_GET() == 1923408869010363697L);
            assert(pack.integrated_x_GET() == -1.8463905E38F);
            assert(pack.integrated_ygyro_GET() == -2.5384942E37F);
            assert(pack.integration_time_us_GET() == 2701166895L);
            assert(pack.quality_GET() == (char)3);
        });
        GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_y_SET(1.983971E38F) ;
        p114.integration_time_us_SET(2701166895L) ;
        p114.time_usec_SET(1923408869010363697L) ;
        p114.time_delta_distance_us_SET(2690589417L) ;
        p114.integrated_ygyro_SET(-2.5384942E37F) ;
        p114.integrated_zgyro_SET(7.5248155E37F) ;
        p114.quality_SET((char)3) ;
        p114.sensor_id_SET((char)226) ;
        p114.distance_SET(-2.4889226E38F) ;
        p114.integrated_xgyro_SET(2.0437988E37F) ;
        p114.temperature_SET((short) -1958) ;
        p114.integrated_x_SET(-1.8463905E38F) ;
        CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.true_airspeed_GET() == (char)64666);
            assert(pack.vy_GET() == (short) -14299);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {-8.141777E37F, -1.5549419E38F, 5.406319E37F, 7.3821856E37F}));
            assert(pack.lon_GET() == -887392712);
            assert(pack.vz_GET() == (short)22254);
            assert(pack.lat_GET() == 898971389);
            assert(pack.rollspeed_GET() == 1.0731574E38F);
            assert(pack.zacc_GET() == (short)15030);
            assert(pack.time_usec_GET() == 7960011242353683333L);
            assert(pack.xacc_GET() == (short) -10020);
            assert(pack.pitchspeed_GET() == 3.3916683E38F);
            assert(pack.yacc_GET() == (short) -32750);
            assert(pack.vx_GET() == (short)2181);
            assert(pack.ind_airspeed_GET() == (char)43948);
            assert(pack.alt_GET() == -1383986600);
            assert(pack.yawspeed_GET() == -2.6910245E38F);
        });
        GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.attitude_quaternion_SET(new float[] {-8.141777E37F, -1.5549419E38F, 5.406319E37F, 7.3821856E37F}, 0) ;
        p115.vx_SET((short)2181) ;
        p115.vz_SET((short)22254) ;
        p115.yacc_SET((short) -32750) ;
        p115.ind_airspeed_SET((char)43948) ;
        p115.pitchspeed_SET(3.3916683E38F) ;
        p115.zacc_SET((short)15030) ;
        p115.true_airspeed_SET((char)64666) ;
        p115.xacc_SET((short) -10020) ;
        p115.rollspeed_SET(1.0731574E38F) ;
        p115.lat_SET(898971389) ;
        p115.vy_SET((short) -14299) ;
        p115.alt_SET(-1383986600) ;
        p115.yawspeed_SET(-2.6910245E38F) ;
        p115.time_usec_SET(7960011242353683333L) ;
        p115.lon_SET(-887392712) ;
        CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == (short) -14647);
            assert(pack.ygyro_GET() == (short) -12240);
            assert(pack.yacc_GET() == (short)1309);
            assert(pack.zgyro_GET() == (short) -11094);
            assert(pack.zacc_GET() == (short)3684);
            assert(pack.zmag_GET() == (short)11713);
            assert(pack.xacc_GET() == (short)9842);
            assert(pack.time_boot_ms_GET() == 2424614397L);
            assert(pack.xgyro_GET() == (short) -24382);
            assert(pack.xmag_GET() == (short) -22694);
        });
        GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.yacc_SET((short)1309) ;
        p116.xmag_SET((short) -22694) ;
        p116.xacc_SET((short)9842) ;
        p116.zacc_SET((short)3684) ;
        p116.xgyro_SET((short) -24382) ;
        p116.ygyro_SET((short) -12240) ;
        p116.ymag_SET((short) -14647) ;
        p116.time_boot_ms_SET(2424614397L) ;
        p116.zmag_SET((short)11713) ;
        p116.zgyro_SET((short) -11094) ;
        CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_GET() == (char)7818);
            assert(pack.target_system_GET() == (char)251);
            assert(pack.target_component_GET() == (char)175);
            assert(pack.start_GET() == (char)16049);
        });
        GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_component_SET((char)175) ;
        p117.start_SET((char)16049) ;
        p117.end_SET((char)7818) ;
        p117.target_system_SET((char)251) ;
        CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)40999);
            assert(pack.last_log_num_GET() == (char)22460);
            assert(pack.num_logs_GET() == (char)56158);
            assert(pack.time_utc_GET() == 952407713L);
            assert(pack.size_GET() == 2181146618L);
        });
        GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.size_SET(2181146618L) ;
        p118.num_logs_SET((char)56158) ;
        p118.last_log_num_SET((char)22460) ;
        p118.time_utc_SET(952407713L) ;
        p118.id_SET((char)40999) ;
        CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.ofs_GET() == 648630065L);
            assert(pack.count_GET() == 429627082L);
            assert(pack.target_system_GET() == (char)187);
            assert(pack.target_component_GET() == (char)241);
            assert(pack.id_GET() == (char)5939);
        });
        GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.count_SET(429627082L) ;
        p119.target_component_SET((char)241) ;
        p119.target_system_SET((char)187) ;
        p119.ofs_SET(648630065L) ;
        p119.id_SET((char)5939) ;
        CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)128, (char)238, (char)142, (char)100, (char)196, (char)35, (char)177, (char)6, (char)27, (char)77, (char)38, (char)200, (char)138, (char)173, (char)93, (char)102, (char)121, (char)185, (char)250, (char)32, (char)232, (char)126, (char)124, (char)227, (char)216, (char)116, (char)180, (char)98, (char)103, (char)66, (char)69, (char)43, (char)67, (char)6, (char)29, (char)77, (char)47, (char)195, (char)251, (char)183, (char)70, (char)11, (char)255, (char)117, (char)231, (char)201, (char)142, (char)48, (char)233, (char)61, (char)154, (char)219, (char)99, (char)238, (char)137, (char)149, (char)7, (char)151, (char)83, (char)15, (char)19, (char)238, (char)154, (char)76, (char)109, (char)190, (char)193, (char)129, (char)222, (char)46, (char)142, (char)30, (char)32, (char)209, (char)150, (char)177, (char)110, (char)132, (char)16, (char)59, (char)126, (char)113, (char)158, (char)186, (char)112, (char)149, (char)252, (char)160, (char)2, (char)56}));
            assert(pack.ofs_GET() == 3670151563L);
            assert(pack.count_GET() == (char)178);
            assert(pack.id_GET() == (char)32313);
        });
        GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.data__SET(new char[] {(char)128, (char)238, (char)142, (char)100, (char)196, (char)35, (char)177, (char)6, (char)27, (char)77, (char)38, (char)200, (char)138, (char)173, (char)93, (char)102, (char)121, (char)185, (char)250, (char)32, (char)232, (char)126, (char)124, (char)227, (char)216, (char)116, (char)180, (char)98, (char)103, (char)66, (char)69, (char)43, (char)67, (char)6, (char)29, (char)77, (char)47, (char)195, (char)251, (char)183, (char)70, (char)11, (char)255, (char)117, (char)231, (char)201, (char)142, (char)48, (char)233, (char)61, (char)154, (char)219, (char)99, (char)238, (char)137, (char)149, (char)7, (char)151, (char)83, (char)15, (char)19, (char)238, (char)154, (char)76, (char)109, (char)190, (char)193, (char)129, (char)222, (char)46, (char)142, (char)30, (char)32, (char)209, (char)150, (char)177, (char)110, (char)132, (char)16, (char)59, (char)126, (char)113, (char)158, (char)186, (char)112, (char)149, (char)252, (char)160, (char)2, (char)56}, 0) ;
        p120.id_SET((char)32313) ;
        p120.ofs_SET(3670151563L) ;
        p120.count_SET((char)178) ;
        CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)61);
            assert(pack.target_system_GET() == (char)84);
        });
        GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)84) ;
        p121.target_component_SET((char)61) ;
        CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)133);
            assert(pack.target_component_GET() == (char)192);
        });
        GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_component_SET((char)192) ;
        p122.target_system_SET((char)133) ;
        CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)82);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)19, (char)191, (char)178, (char)110, (char)237, (char)69, (char)168, (char)230, (char)10, (char)127, (char)16, (char)141, (char)74, (char)206, (char)159, (char)57, (char)69, (char)0, (char)50, (char)210, (char)64, (char)229, (char)209, (char)223, (char)14, (char)245, (char)214, (char)253, (char)219, (char)214, (char)179, (char)74, (char)217, (char)152, (char)175, (char)24, (char)104, (char)35, (char)89, (char)254, (char)71, (char)16, (char)179, (char)194, (char)223, (char)47, (char)9, (char)252, (char)68, (char)156, (char)245, (char)82, (char)51, (char)111, (char)180, (char)108, (char)114, (char)201, (char)252, (char)230, (char)173, (char)209, (char)44, (char)211, (char)91, (char)249, (char)153, (char)137, (char)97, (char)34, (char)37, (char)121, (char)51, (char)132, (char)192, (char)2, (char)26, (char)162, (char)73, (char)158, (char)81, (char)21, (char)50, (char)220, (char)201, (char)184, (char)40, (char)37, (char)173, (char)55, (char)233, (char)237, (char)240, (char)34, (char)235, (char)203, (char)49, (char)3, (char)186, (char)34, (char)135, (char)238, (char)156, (char)1, (char)239, (char)175, (char)222, (char)196, (char)26, (char)132}));
            assert(pack.target_component_GET() == (char)48);
            assert(pack.len_GET() == (char)56);
        });
        GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)82) ;
        p123.target_component_SET((char)48) ;
        p123.data__SET(new char[] {(char)19, (char)191, (char)178, (char)110, (char)237, (char)69, (char)168, (char)230, (char)10, (char)127, (char)16, (char)141, (char)74, (char)206, (char)159, (char)57, (char)69, (char)0, (char)50, (char)210, (char)64, (char)229, (char)209, (char)223, (char)14, (char)245, (char)214, (char)253, (char)219, (char)214, (char)179, (char)74, (char)217, (char)152, (char)175, (char)24, (char)104, (char)35, (char)89, (char)254, (char)71, (char)16, (char)179, (char)194, (char)223, (char)47, (char)9, (char)252, (char)68, (char)156, (char)245, (char)82, (char)51, (char)111, (char)180, (char)108, (char)114, (char)201, (char)252, (char)230, (char)173, (char)209, (char)44, (char)211, (char)91, (char)249, (char)153, (char)137, (char)97, (char)34, (char)37, (char)121, (char)51, (char)132, (char)192, (char)2, (char)26, (char)162, (char)73, (char)158, (char)81, (char)21, (char)50, (char)220, (char)201, (char)184, (char)40, (char)37, (char)173, (char)55, (char)233, (char)237, (char)240, (char)34, (char)235, (char)203, (char)49, (char)3, (char)186, (char)34, (char)135, (char)238, (char)156, (char)1, (char)239, (char)175, (char)222, (char)196, (char)26, (char)132}, 0) ;
        p123.len_SET((char)56) ;
        CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.dgps_age_GET() == 2711919229L);
            assert(pack.dgps_numch_GET() == (char)184);
            assert(pack.epv_GET() == (char)33023);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
            assert(pack.lat_GET() == -1002154423);
            assert(pack.satellites_visible_GET() == (char)253);
            assert(pack.eph_GET() == (char)20838);
            assert(pack.alt_GET() == 1266536776);
            assert(pack.lon_GET() == -1285414495);
            assert(pack.vel_GET() == (char)24606);
            assert(pack.time_usec_GET() == 643012817154779119L);
            assert(pack.cog_GET() == (char)33773);
        });
        GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.alt_SET(1266536776) ;
        p124.vel_SET((char)24606) ;
        p124.lon_SET(-1285414495) ;
        p124.cog_SET((char)33773) ;
        p124.dgps_numch_SET((char)184) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX) ;
        p124.eph_SET((char)20838) ;
        p124.lat_SET(-1002154423) ;
        p124.satellites_visible_SET((char)253) ;
        p124.dgps_age_SET(2711919229L) ;
        p124.epv_SET((char)33023) ;
        p124.time_usec_SET(643012817154779119L) ;
        CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vservo_GET() == (char)33777);
            assert(pack.flags_GET() == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED));
            assert(pack.Vcc_GET() == (char)6738);
        });
        GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)33777) ;
        p125.Vcc_SET((char)6738) ;
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED)) ;
        CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)125, (char)114, (char)109, (char)213, (char)106, (char)176, (char)158, (char)141, (char)186, (char)44, (char)0, (char)57, (char)97, (char)223, (char)221, (char)246, (char)216, (char)224, (char)84, (char)38, (char)168, (char)194, (char)251, (char)47, (char)186, (char)244, (char)212, (char)66, (char)5, (char)184, (char)222, (char)209, (char)252, (char)215, (char)188, (char)96, (char)28, (char)234, (char)151, (char)56, (char)31, (char)148, (char)135, (char)194, (char)107, (char)124, (char)73, (char)225, (char)23, (char)141, (char)115, (char)90, (char)190, (char)169, (char)179, (char)26, (char)84, (char)170, (char)7, (char)136, (char)197, (char)20, (char)56, (char)25, (char)109, (char)18, (char)143, (char)113, (char)26, (char)123}));
            assert(pack.baudrate_GET() == 3054242728L);
            assert(pack.timeout_GET() == (char)21403);
            assert(pack.flags_GET() == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
            assert(pack.count_GET() == (char)178);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
        });
        GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.baudrate_SET(3054242728L) ;
        p126.count_SET((char)178) ;
        p126.timeout_SET((char)21403) ;
        p126.data__SET(new char[] {(char)125, (char)114, (char)109, (char)213, (char)106, (char)176, (char)158, (char)141, (char)186, (char)44, (char)0, (char)57, (char)97, (char)223, (char)221, (char)246, (char)216, (char)224, (char)84, (char)38, (char)168, (char)194, (char)251, (char)47, (char)186, (char)244, (char)212, (char)66, (char)5, (char)184, (char)222, (char)209, (char)252, (char)215, (char)188, (char)96, (char)28, (char)234, (char)151, (char)56, (char)31, (char)148, (char)135, (char)194, (char)107, (char)124, (char)73, (char)225, (char)23, (char)141, (char)115, (char)90, (char)190, (char)169, (char)179, (char)26, (char)84, (char)170, (char)7, (char)136, (char)197, (char)20, (char)56, (char)25, (char)109, (char)18, (char)143, (char)113, (char)26, (char)123}, 0) ;
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING)) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2) ;
        CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.iar_num_hypotheses_GET() == 100723284);
            assert(pack.wn_GET() == (char)38615);
            assert(pack.tow_GET() == 1787502749L);
            assert(pack.baseline_c_mm_GET() == 730448440);
            assert(pack.rtk_receiver_id_GET() == (char)77);
            assert(pack.time_last_baseline_ms_GET() == 3024000214L);
            assert(pack.baseline_b_mm_GET() == -1830517364);
            assert(pack.baseline_a_mm_GET() == -378788956);
            assert(pack.accuracy_GET() == 3766859856L);
            assert(pack.nsats_GET() == (char)213);
            assert(pack.rtk_health_GET() == (char)220);
            assert(pack.baseline_coords_type_GET() == (char)241);
            assert(pack.rtk_rate_GET() == (char)235);
        });
        GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.rtk_health_SET((char)220) ;
        p127.rtk_receiver_id_SET((char)77) ;
        p127.tow_SET(1787502749L) ;
        p127.accuracy_SET(3766859856L) ;
        p127.nsats_SET((char)213) ;
        p127.baseline_c_mm_SET(730448440) ;
        p127.baseline_coords_type_SET((char)241) ;
        p127.wn_SET((char)38615) ;
        p127.rtk_rate_SET((char)235) ;
        p127.iar_num_hypotheses_SET(100723284) ;
        p127.baseline_a_mm_SET(-378788956) ;
        p127.baseline_b_mm_SET(-1830517364) ;
        p127.time_last_baseline_ms_SET(3024000214L) ;
        CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.accuracy_GET() == 2892903877L);
            assert(pack.baseline_coords_type_GET() == (char)165);
            assert(pack.baseline_a_mm_GET() == -825591567);
            assert(pack.wn_GET() == (char)874);
            assert(pack.rtk_health_GET() == (char)77);
            assert(pack.rtk_receiver_id_GET() == (char)198);
            assert(pack.nsats_GET() == (char)132);
            assert(pack.baseline_b_mm_GET() == 1087856886);
            assert(pack.iar_num_hypotheses_GET() == -2110837081);
            assert(pack.time_last_baseline_ms_GET() == 261518164L);
            assert(pack.tow_GET() == 1688235592L);
            assert(pack.rtk_rate_GET() == (char)99);
            assert(pack.baseline_c_mm_GET() == -235343542);
        });
        GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_coords_type_SET((char)165) ;
        p128.iar_num_hypotheses_SET(-2110837081) ;
        p128.rtk_health_SET((char)77) ;
        p128.baseline_b_mm_SET(1087856886) ;
        p128.accuracy_SET(2892903877L) ;
        p128.baseline_c_mm_SET(-235343542) ;
        p128.time_last_baseline_ms_SET(261518164L) ;
        p128.wn_SET((char)874) ;
        p128.tow_SET(1688235592L) ;
        p128.rtk_rate_SET((char)99) ;
        p128.nsats_SET((char)132) ;
        p128.baseline_a_mm_SET(-825591567) ;
        p128.rtk_receiver_id_SET((char)198) ;
        CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == (short) -22601);
            assert(pack.xacc_GET() == (short)20209);
            assert(pack.zacc_GET() == (short)12036);
            assert(pack.yacc_GET() == (short)29370);
            assert(pack.ymag_GET() == (short) -22989);
            assert(pack.time_boot_ms_GET() == 86596875L);
            assert(pack.xgyro_GET() == (short)18229);
            assert(pack.zgyro_GET() == (short)4818);
            assert(pack.ygyro_GET() == (short)15760);
            assert(pack.xmag_GET() == (short)17783);
        });
        GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.xgyro_SET((short)18229) ;
        p129.yacc_SET((short)29370) ;
        p129.time_boot_ms_SET(86596875L) ;
        p129.ygyro_SET((short)15760) ;
        p129.xmag_SET((short)17783) ;
        p129.zgyro_SET((short)4818) ;
        p129.zacc_SET((short)12036) ;
        p129.zmag_SET((short) -22601) ;
        p129.xacc_SET((short)20209) ;
        p129.ymag_SET((short) -22989) ;
        CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.jpg_quality_GET() == (char)109);
            assert(pack.type_GET() == (char)151);
            assert(pack.width_GET() == (char)41252);
            assert(pack.height_GET() == (char)33298);
            assert(pack.packets_GET() == (char)35536);
            assert(pack.size_GET() == 1486755600L);
            assert(pack.payload_GET() == (char)127);
        });
        GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.packets_SET((char)35536) ;
        p130.payload_SET((char)127) ;
        p130.jpg_quality_SET((char)109) ;
        p130.height_SET((char)33298) ;
        p130.type_SET((char)151) ;
        p130.size_SET(1486755600L) ;
        p130.width_SET((char)41252) ;
        CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)62274);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)92, (char)94, (char)166, (char)217, (char)69, (char)129, (char)85, (char)107, (char)37, (char)200, (char)74, (char)55, (char)204, (char)85, (char)250, (char)191, (char)75, (char)240, (char)169, (char)66, (char)36, (char)220, (char)37, (char)108, (char)182, (char)147, (char)60, (char)170, (char)5, (char)230, (char)177, (char)171, (char)75, (char)127, (char)94, (char)80, (char)38, (char)100, (char)9, (char)244, (char)61, (char)94, (char)222, (char)24, (char)184, (char)143, (char)48, (char)96, (char)163, (char)117, (char)44, (char)164, (char)175, (char)171, (char)153, (char)235, (char)11, (char)61, (char)178, (char)26, (char)134, (char)201, (char)211, (char)222, (char)123, (char)156, (char)166, (char)216, (char)217, (char)88, (char)14, (char)251, (char)229, (char)167, (char)252, (char)141, (char)25, (char)119, (char)88, (char)82, (char)65, (char)195, (char)242, (char)188, (char)144, (char)177, (char)48, (char)64, (char)47, (char)72, (char)61, (char)237, (char)49, (char)253, (char)20, (char)225, (char)55, (char)7, (char)200, (char)28, (char)87, (char)137, (char)121, (char)144, (char)76, (char)47, (char)109, (char)194, (char)34, (char)243, (char)158, (char)212, (char)155, (char)231, (char)223, (char)27, (char)187, (char)120, (char)56, (char)32, (char)35, (char)157, (char)155, (char)51, (char)175, (char)159, (char)55, (char)185, (char)206, (char)183, (char)204, (char)253, (char)126, (char)216, (char)134, (char)37, (char)125, (char)100, (char)14, (char)106, (char)238, (char)189, (char)77, (char)50, (char)96, (char)239, (char)247, (char)182, (char)155, (char)88, (char)230, (char)159, (char)205, (char)40, (char)150, (char)111, (char)210, (char)28, (char)150, (char)95, (char)171, (char)95, (char)103, (char)232, (char)57, (char)21, (char)155, (char)132, (char)218, (char)26, (char)67, (char)62, (char)221, (char)198, (char)141, (char)128, (char)189, (char)239, (char)21, (char)150, (char)91, (char)109, (char)238, (char)38, (char)172, (char)32, (char)248, (char)210, (char)186, (char)47, (char)199, (char)63, (char)202, (char)56, (char)145, (char)151, (char)134, (char)243, (char)127, (char)81, (char)186, (char)235, (char)75, (char)149, (char)184, (char)13, (char)204, (char)37, (char)214, (char)138, (char)109, (char)36, (char)6, (char)24, (char)187, (char)237, (char)170, (char)118, (char)239, (char)12, (char)41, (char)253, (char)1, (char)156, (char)134, (char)61, (char)236, (char)200, (char)138, (char)93, (char)153, (char)226, (char)173, (char)208, (char)32, (char)107, (char)75, (char)183, (char)239, (char)30, (char)31, (char)230, (char)128, (char)101, (char)163, (char)109, (char)119, (char)168, (char)12, (char)252, (char)61, (char)56, (char)7}));
        });
        GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.data__SET(new char[] {(char)92, (char)94, (char)166, (char)217, (char)69, (char)129, (char)85, (char)107, (char)37, (char)200, (char)74, (char)55, (char)204, (char)85, (char)250, (char)191, (char)75, (char)240, (char)169, (char)66, (char)36, (char)220, (char)37, (char)108, (char)182, (char)147, (char)60, (char)170, (char)5, (char)230, (char)177, (char)171, (char)75, (char)127, (char)94, (char)80, (char)38, (char)100, (char)9, (char)244, (char)61, (char)94, (char)222, (char)24, (char)184, (char)143, (char)48, (char)96, (char)163, (char)117, (char)44, (char)164, (char)175, (char)171, (char)153, (char)235, (char)11, (char)61, (char)178, (char)26, (char)134, (char)201, (char)211, (char)222, (char)123, (char)156, (char)166, (char)216, (char)217, (char)88, (char)14, (char)251, (char)229, (char)167, (char)252, (char)141, (char)25, (char)119, (char)88, (char)82, (char)65, (char)195, (char)242, (char)188, (char)144, (char)177, (char)48, (char)64, (char)47, (char)72, (char)61, (char)237, (char)49, (char)253, (char)20, (char)225, (char)55, (char)7, (char)200, (char)28, (char)87, (char)137, (char)121, (char)144, (char)76, (char)47, (char)109, (char)194, (char)34, (char)243, (char)158, (char)212, (char)155, (char)231, (char)223, (char)27, (char)187, (char)120, (char)56, (char)32, (char)35, (char)157, (char)155, (char)51, (char)175, (char)159, (char)55, (char)185, (char)206, (char)183, (char)204, (char)253, (char)126, (char)216, (char)134, (char)37, (char)125, (char)100, (char)14, (char)106, (char)238, (char)189, (char)77, (char)50, (char)96, (char)239, (char)247, (char)182, (char)155, (char)88, (char)230, (char)159, (char)205, (char)40, (char)150, (char)111, (char)210, (char)28, (char)150, (char)95, (char)171, (char)95, (char)103, (char)232, (char)57, (char)21, (char)155, (char)132, (char)218, (char)26, (char)67, (char)62, (char)221, (char)198, (char)141, (char)128, (char)189, (char)239, (char)21, (char)150, (char)91, (char)109, (char)238, (char)38, (char)172, (char)32, (char)248, (char)210, (char)186, (char)47, (char)199, (char)63, (char)202, (char)56, (char)145, (char)151, (char)134, (char)243, (char)127, (char)81, (char)186, (char)235, (char)75, (char)149, (char)184, (char)13, (char)204, (char)37, (char)214, (char)138, (char)109, (char)36, (char)6, (char)24, (char)187, (char)237, (char)170, (char)118, (char)239, (char)12, (char)41, (char)253, (char)1, (char)156, (char)134, (char)61, (char)236, (char)200, (char)138, (char)93, (char)153, (char)226, (char)173, (char)208, (char)32, (char)107, (char)75, (char)183, (char)239, (char)30, (char)31, (char)230, (char)128, (char)101, (char)163, (char)109, (char)119, (char)168, (char)12, (char)252, (char)61, (char)56, (char)7}, 0) ;
        p131.seqnr_SET((char)62274) ;
        CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.current_distance_GET() == (char)24088);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_45);
            assert(pack.covariance_GET() == (char)45);
            assert(pack.time_boot_ms_GET() == 1855430960L);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.id_GET() == (char)202);
            assert(pack.max_distance_GET() == (char)385);
            assert(pack.min_distance_GET() == (char)28026);
        });
        GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.current_distance_SET((char)24088) ;
        p132.id_SET((char)202) ;
        p132.time_boot_ms_SET(1855430960L) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p132.covariance_SET((char)45) ;
        p132.max_distance_SET((char)385) ;
        p132.min_distance_SET((char)28026) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_45) ;
        CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mask_GET() == 1794996228894364286L);
            assert(pack.lon_GET() == -1616111400);
            assert(pack.lat_GET() == -161724196);
            assert(pack.grid_spacing_GET() == (char)36314);
        });
        GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.grid_spacing_SET((char)36314) ;
        p133.lon_SET(-1616111400) ;
        p133.mask_SET(1794996228894364286L) ;
        p133.lat_SET(-161724196) ;
        CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.grid_spacing_GET() == (char)36473);
            assert(pack.lat_GET() == 1904317861);
            assert(pack.lon_GET() == 530447974);
            assert(pack.gridbit_GET() == (char)118);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)18170, (short) -4996, (short) -8268, (short) -18474, (short) -18652, (short) -11443, (short) -12470, (short)16033, (short) -13018, (short)5360, (short)5509, (short) -12312, (short) -30198, (short) -27899, (short) -8994, (short) -24154}));
        });
        GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.data__SET(new short[] {(short)18170, (short) -4996, (short) -8268, (short) -18474, (short) -18652, (short) -11443, (short) -12470, (short)16033, (short) -13018, (short)5360, (short)5509, (short) -12312, (short) -30198, (short) -27899, (short) -8994, (short) -24154}, 0) ;
        p134.grid_spacing_SET((char)36473) ;
        p134.lat_SET(1904317861) ;
        p134.gridbit_SET((char)118) ;
        p134.lon_SET(530447974) ;
        CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 995719765);
            assert(pack.lat_GET() == 611750263);
        });
        GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(995719765) ;
        p135.lat_SET(611750263) ;
        CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 531070682);
            assert(pack.terrain_height_GET() == 1.4642862E38F);
            assert(pack.pending_GET() == (char)24559);
            assert(pack.spacing_GET() == (char)59385);
            assert(pack.lon_GET() == 1756955559);
            assert(pack.current_height_GET() == -3.3226394E38F);
            assert(pack.loaded_GET() == (char)51152);
        });
        GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.loaded_SET((char)51152) ;
        p136.pending_SET((char)24559) ;
        p136.terrain_height_SET(1.4642862E38F) ;
        p136.lat_SET(531070682) ;
        p136.lon_SET(1756955559) ;
        p136.current_height_SET(-3.3226394E38F) ;
        p136.spacing_SET((char)59385) ;
        CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -31739);
            assert(pack.press_abs_GET() == -4.541957E37F);
            assert(pack.press_diff_GET() == 1.461187E38F);
            assert(pack.time_boot_ms_GET() == 447877138L);
        });
        GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.temperature_SET((short) -31739) ;
        p137.time_boot_ms_SET(447877138L) ;
        p137.press_diff_SET(1.461187E38F) ;
        p137.press_abs_SET(-4.541957E37F) ;
        CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3528893694803414890L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.713866E38F, -2.7762197E38F, 2.3209674E38F, 5.516718E37F}));
            assert(pack.y_GET() == -3.1597923E38F);
            assert(pack.z_GET() == -1.664202E38F);
            assert(pack.x_GET() == 2.5643837E38F);
        });
        GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.y_SET(-3.1597923E38F) ;
        p138.z_SET(-1.664202E38F) ;
        p138.q_SET(new float[] {1.713866E38F, -2.7762197E38F, 2.3209674E38F, 5.516718E37F}, 0) ;
        p138.x_SET(2.5643837E38F) ;
        p138.time_usec_SET(3528893694803414890L) ;
        CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.4403279E38F, -1.5216206E38F, -2.128937E37F, 2.229878E38F, 4.6918127E37F, 6.9368685E37F, 7.6681483E37F, -1.4236105E38F}));
            assert(pack.group_mlx_GET() == (char)105);
            assert(pack.target_component_GET() == (char)39);
            assert(pack.target_system_GET() == (char)129);
            assert(pack.time_usec_GET() == 7985660750420466091L);
        });
        GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.controls_SET(new float[] {1.4403279E38F, -1.5216206E38F, -2.128937E37F, 2.229878E38F, 4.6918127E37F, 6.9368685E37F, 7.6681483E37F, -1.4236105E38F}, 0) ;
        p139.target_component_SET((char)39) ;
        p139.group_mlx_SET((char)105) ;
        p139.time_usec_SET(7985660750420466091L) ;
        p139.target_system_SET((char)129) ;
        CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7383663364323711422L);
            assert(pack.group_mlx_GET() == (char)38);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.4331525E38F, 7.0897786E37F, 1.1513864E38F, -7.3713517E37F, -1.575075E38F, -2.7138566E38F, -1.5782952E38F, 1.3300317E38F}));
        });
        GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)38) ;
        p140.time_usec_SET(7383663364323711422L) ;
        p140.controls_SET(new float[] {-2.4331525E38F, 7.0897786E37F, 1.1513864E38F, -7.3713517E37F, -1.575075E38F, -2.7138566E38F, -1.5782952E38F, 1.3300317E38F}, 0) ;
        CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, CommunicationChannel.instance.outputStream, CommunicationChannel.instance);
        TestChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_monotonic_GET() == 9.380023E37F);
            assert(pack.altitude_amsl_GET() == -2.7938995E38F);
            assert(pack.altitude_relative_GET() == 2.6586103E38F);
            assert(pack.time_usec_GET() == 4445103667872883897L);
            assert(pack.bottom_clearance_GET() == 3.792353E37F);
            assert(pack.altitude_terrain_GET() == 1.1274142E38F);
            assert(pack.altitude_local_GET() == -1.5184914E38F);
        });
        GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_local_SET(-1.5184914E38F) ;
        p141.altitude_relative_SET(2.6586103E38F) ;
        p141.bottom_clearance_SET(3.792353E37F) ;
        p141.altitude_terrain_SET(1.1274142E38F) ;
        p141.time_usec_SET(4445103667872883897L) ;
        p141.altitude_amsl_SET(-2.7938995E38F) ;
        p141.altitude_monotonic_SET(9.380023E37F) ;
        CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.request_id_GET() == (char)237);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)106, (char)125, (char)205, (char)71, (char)100, (char)192, (char)116, (char)80, (char)228, (char)36, (char)142, (char)155, (char)105, (char)82, (char)83, (char)232, (char)139, (char)153, (char)23, (char)26, (char)251, (char)8, (char)148, (char)59, (char)18, (char)138, (char)156, (char)195, (char)114, (char)64, (char)62, (char)169, (char)189, (char)52, (char)160, (char)221, (char)66, (char)206, (char)143, (char)3, (char)241, (char)184, (char)88, (char)103, (char)5, (char)241, (char)93, (char)217, (char)202, (char)66, (char)145, (char)255, (char)44, (char)174, (char)179, (char)212, (char)18, (char)58, (char)42, (char)235, (char)121, (char)88, (char)139, (char)116, (char)31, (char)24, (char)13, (char)183, (char)65, (char)4, (char)38, (char)202, (char)118, (char)22, (char)147, (char)199, (char)187, (char)183, (char)37, (char)58, (char)253, (char)216, (char)33, (char)64, (char)81, (char)156, (char)24, (char)225, (char)89, (char)103, (char)224, (char)245, (char)177, (char)16, (char)58, (char)89, (char)205, (char)144, (char)146, (char)141, (char)115, (char)217, (char)126, (char)24, (char)209, (char)152, (char)150, (char)151, (char)164, (char)59, (char)143, (char)95, (char)132, (char)208, (char)200, (char)129, (char)229, (char)134, (char)128, (char)247}));
            assert(pack.transfer_type_GET() == (char)127);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)57, (char)73, (char)60, (char)97, (char)25, (char)240, (char)180, (char)221, (char)12, (char)154, (char)2, (char)212, (char)226, (char)94, (char)211, (char)154, (char)144, (char)139, (char)163, (char)217, (char)221, (char)9, (char)10, (char)85, (char)128, (char)197, (char)191, (char)21, (char)191, (char)111, (char)177, (char)117, (char)251, (char)46, (char)167, (char)84, (char)52, (char)58, (char)1, (char)119, (char)37, (char)212, (char)138, (char)117, (char)191, (char)11, (char)120, (char)200, (char)30, (char)0, (char)83, (char)44, (char)10, (char)57, (char)211, (char)105, (char)142, (char)15, (char)199, (char)54, (char)21, (char)222, (char)41, (char)248, (char)57, (char)251, (char)5, (char)159, (char)89, (char)210, (char)112, (char)6, (char)37, (char)136, (char)181, (char)5, (char)136, (char)56, (char)174, (char)236, (char)252, (char)71, (char)59, (char)32, (char)37, (char)2, (char)142, (char)122, (char)211, (char)14, (char)57, (char)172, (char)166, (char)104, (char)109, (char)201, (char)180, (char)31, (char)94, (char)196, (char)149, (char)202, (char)10, (char)10, (char)122, (char)162, (char)99, (char)195, (char)172, (char)118, (char)126, (char)47, (char)64, (char)29, (char)180, (char)221, (char)63, (char)11, (char)171, (char)125}));
            assert(pack.uri_type_GET() == (char)130);
        });
        GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.transfer_type_SET((char)127) ;
        p142.request_id_SET((char)237) ;
        p142.uri_type_SET((char)130) ;
        p142.storage_SET(new char[] {(char)57, (char)73, (char)60, (char)97, (char)25, (char)240, (char)180, (char)221, (char)12, (char)154, (char)2, (char)212, (char)226, (char)94, (char)211, (char)154, (char)144, (char)139, (char)163, (char)217, (char)221, (char)9, (char)10, (char)85, (char)128, (char)197, (char)191, (char)21, (char)191, (char)111, (char)177, (char)117, (char)251, (char)46, (char)167, (char)84, (char)52, (char)58, (char)1, (char)119, (char)37, (char)212, (char)138, (char)117, (char)191, (char)11, (char)120, (char)200, (char)30, (char)0, (char)83, (char)44, (char)10, (char)57, (char)211, (char)105, (char)142, (char)15, (char)199, (char)54, (char)21, (char)222, (char)41, (char)248, (char)57, (char)251, (char)5, (char)159, (char)89, (char)210, (char)112, (char)6, (char)37, (char)136, (char)181, (char)5, (char)136, (char)56, (char)174, (char)236, (char)252, (char)71, (char)59, (char)32, (char)37, (char)2, (char)142, (char)122, (char)211, (char)14, (char)57, (char)172, (char)166, (char)104, (char)109, (char)201, (char)180, (char)31, (char)94, (char)196, (char)149, (char)202, (char)10, (char)10, (char)122, (char)162, (char)99, (char)195, (char)172, (char)118, (char)126, (char)47, (char)64, (char)29, (char)180, (char)221, (char)63, (char)11, (char)171, (char)125}, 0) ;
        p142.uri_SET(new char[] {(char)106, (char)125, (char)205, (char)71, (char)100, (char)192, (char)116, (char)80, (char)228, (char)36, (char)142, (char)155, (char)105, (char)82, (char)83, (char)232, (char)139, (char)153, (char)23, (char)26, (char)251, (char)8, (char)148, (char)59, (char)18, (char)138, (char)156, (char)195, (char)114, (char)64, (char)62, (char)169, (char)189, (char)52, (char)160, (char)221, (char)66, (char)206, (char)143, (char)3, (char)241, (char)184, (char)88, (char)103, (char)5, (char)241, (char)93, (char)217, (char)202, (char)66, (char)145, (char)255, (char)44, (char)174, (char)179, (char)212, (char)18, (char)58, (char)42, (char)235, (char)121, (char)88, (char)139, (char)116, (char)31, (char)24, (char)13, (char)183, (char)65, (char)4, (char)38, (char)202, (char)118, (char)22, (char)147, (char)199, (char)187, (char)183, (char)37, (char)58, (char)253, (char)216, (char)33, (char)64, (char)81, (char)156, (char)24, (char)225, (char)89, (char)103, (char)224, (char)245, (char)177, (char)16, (char)58, (char)89, (char)205, (char)144, (char)146, (char)141, (char)115, (char)217, (char)126, (char)24, (char)209, (char)152, (char)150, (char)151, (char)164, (char)59, (char)143, (char)95, (char)132, (char)208, (char)200, (char)129, (char)229, (char)134, (char)128, (char)247}, 0) ;
        CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)22522);
            assert(pack.press_abs_GET() == 2.1084415E38F);
            assert(pack.press_diff_GET() == -2.3623868E37F);
            assert(pack.time_boot_ms_GET() == 1951539859L);
        });
        GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(1951539859L) ;
        p143.temperature_SET((short)22522) ;
        p143.press_abs_SET(2.1084415E38F) ;
        p143.press_diff_SET(-2.3623868E37F) ;
        CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.custom_state_GET() == 5572949099902585961L);
            assert(pack.timestamp_GET() == 1668198821683702980L);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-2.9876756E38F, 2.4737335E38F, 1.787838E37F}));
            assert(pack.lat_GET() == -1055259201);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {1.973903E38F, 2.0847488E37F, 9.1273955E36F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-8.710221E37F, -1.5938285E38F, 1.4232354E38F, 1.3426132E38F}));
            assert(pack.lon_GET() == -2097732668);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {-8.889671E37F, 1.3892249E38F, -9.226981E37F}));
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {2.3597995E38F, 1.4782395E38F, -2.3599387E37F}));
            assert(pack.est_capabilities_GET() == (char)94);
            assert(pack.alt_GET() == 1.6674855E38F);
        });
        GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.attitude_q_SET(new float[] {-8.710221E37F, -1.5938285E38F, 1.4232354E38F, 1.3426132E38F}, 0) ;
        p144.custom_state_SET(5572949099902585961L) ;
        p144.acc_SET(new float[] {1.973903E38F, 2.0847488E37F, 9.1273955E36F}, 0) ;
        p144.vel_SET(new float[] {-8.889671E37F, 1.3892249E38F, -9.226981E37F}, 0) ;
        p144.alt_SET(1.6674855E38F) ;
        p144.lon_SET(-2097732668) ;
        p144.est_capabilities_SET((char)94) ;
        p144.timestamp_SET(1668198821683702980L) ;
        p144.rates_SET(new float[] {-2.9876756E38F, 2.4737335E38F, 1.787838E37F}, 0) ;
        p144.lat_SET(-1055259201) ;
        p144.position_cov_SET(new float[] {2.3597995E38F, 1.4782395E38F, -2.3599387E37F}, 0) ;
        CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == -2.0116996E38F);
            assert(pack.y_pos_GET() == -2.00593E38F);
            assert(pack.z_acc_GET() == -2.0629912E38F);
            assert(pack.y_vel_GET() == 1.90099E38F);
            assert(pack.time_usec_GET() == 974419660911991355L);
            assert(pack.x_pos_GET() == -2.9609984E38F);
            assert(pack.z_vel_GET() == -2.3934846E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.9402102E37F, -3.3610658E37F, 3.363884E38F, 4.2374742E36F}));
            assert(pack.roll_rate_GET() == 2.1539136E38F);
            assert(pack.airspeed_GET() == 3.009603E38F);
            assert(pack.x_vel_GET() == -2.672484E38F);
            assert(pack.y_acc_GET() == 1.760865E38F);
            assert(pack.x_acc_GET() == -3.3821697E38F);
            assert(pack.pitch_rate_GET() == -3.2177163E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {2.8995226E37F, 3.2548947E38F, -2.3756876E38F}));
            assert(pack.z_pos_GET() == -3.8295793E37F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {1.7503316E38F, -2.1673432E38F, 3.6038031E37F}));
        });
        GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.y_vel_SET(1.90099E38F) ;
        p146.x_vel_SET(-2.672484E38F) ;
        p146.x_pos_SET(-2.9609984E38F) ;
        p146.roll_rate_SET(2.1539136E38F) ;
        p146.y_acc_SET(1.760865E38F) ;
        p146.z_vel_SET(-2.3934846E38F) ;
        p146.vel_variance_SET(new float[] {2.8995226E37F, 3.2548947E38F, -2.3756876E38F}, 0) ;
        p146.x_acc_SET(-3.3821697E38F) ;
        p146.pitch_rate_SET(-3.2177163E38F) ;
        p146.q_SET(new float[] {3.9402102E37F, -3.3610658E37F, 3.363884E38F, 4.2374742E36F}, 0) ;
        p146.z_pos_SET(-3.8295793E37F) ;
        p146.airspeed_SET(3.009603E38F) ;
        p146.pos_variance_SET(new float[] {1.7503316E38F, -2.1673432E38F, 3.6038031E37F}, 0) ;
        p146.y_pos_SET(-2.00593E38F) ;
        p146.time_usec_SET(974419660911991355L) ;
        p146.z_acc_SET(-2.0629912E38F) ;
        p146.yaw_rate_SET(-2.0116996E38F) ;
        CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
            assert(pack.current_consumed_GET() == 1779974215);
            assert(pack.current_battery_GET() == (short)17715);
            assert(pack.temperature_GET() == (short) -13417);
            assert(pack.energy_consumed_GET() == -775818366);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)61497, (char)33304, (char)22211, (char)53255, (char)11453, (char)54878, (char)55115, (char)64502, (char)55645, (char)18177}));
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
            assert(pack.battery_remaining_GET() == (byte) - 14);
            assert(pack.id_GET() == (char)72);
        });
        GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION) ;
        p147.energy_consumed_SET(-775818366) ;
        p147.current_consumed_SET(1779974215) ;
        p147.id_SET((char)72) ;
        p147.voltages_SET(new char[] {(char)61497, (char)33304, (char)22211, (char)53255, (char)11453, (char)54878, (char)55115, (char)64502, (char)55645, (char)18177}, 0) ;
        p147.temperature_SET((short) -13417) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION) ;
        p147.battery_remaining_SET((byte) - 14) ;
        p147.current_battery_SET((short)17715) ;
        CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)204, (char)203, (char)235, (char)45, (char)39, (char)107, (char)131, (char)169}));
            assert(pack.capabilities_GET() == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
            assert(pack.vendor_id_GET() == (char)23612);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)242, (char)215, (char)169, (char)244, (char)230, (char)240, (char)249, (char)109}));
            assert(pack.board_version_GET() == 2862949747L);
            assert(pack.flight_sw_version_GET() == 2249381649L);
            assert(pack.middleware_sw_version_GET() == 287251916L);
            assert(pack.uid_GET() == 8430518226477430750L);
            assert(pack.product_id_GET() == (char)56240);
            assert(pack.os_sw_version_GET() == 1239080840L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)44, (char)98, (char)135, (char)53, (char)63, (char)223, (char)253, (char)200, (char)5, (char)168, (char)131, (char)188, (char)186, (char)146, (char)36, (char)225, (char)4, (char)196}));
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)214, (char)232, (char)245, (char)111, (char)38, (char)117, (char)82, (char)68}));
        });
        GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.flight_sw_version_SET(2249381649L) ;
        p148.os_sw_version_SET(1239080840L) ;
        p148.os_custom_version_SET(new char[] {(char)204, (char)203, (char)235, (char)45, (char)39, (char)107, (char)131, (char)169}, 0) ;
        p148.middleware_custom_version_SET(new char[] {(char)242, (char)215, (char)169, (char)244, (char)230, (char)240, (char)249, (char)109}, 0) ;
        p148.uid2_SET(new char[] {(char)44, (char)98, (char)135, (char)53, (char)63, (char)223, (char)253, (char)200, (char)5, (char)168, (char)131, (char)188, (char)186, (char)146, (char)36, (char)225, (char)4, (char)196}, 0, PH) ;
        p148.flight_custom_version_SET(new char[] {(char)214, (char)232, (char)245, (char)111, (char)38, (char)117, (char)82, (char)68}, 0) ;
        p148.board_version_SET(2862949747L) ;
        p148.product_id_SET((char)56240) ;
        p148.vendor_id_SET((char)23612) ;
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE)) ;
        p148.middleware_sw_version_SET(287251916L) ;
        p148.uid_SET(8430518226477430750L) ;
        CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.size_y_GET() == -3.544155E37F);
            assert(pack.size_x_GET() == -1.6788583E38F);
            assert(pack.distance_GET() == -1.4670968E38F);
            assert(pack.angle_x_GET() == -1.5203644E38F);
            assert(pack.position_valid_TRY(ph) == (char)62);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {-1.2274825E38F, -7.9577375E37F, 3.0675128E38F, 3.2461729E38F}));
            assert(pack.y_TRY(ph) == -3.2617636E38F);
            assert(pack.angle_y_GET() == -5.8321577E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.x_TRY(ph) == -2.9204662E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
            assert(pack.target_num_GET() == (char)218);
            assert(pack.time_usec_GET() == 2522465229399542159L);
            assert(pack.z_TRY(ph) == -2.961579E38F);
        });
        GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.target_num_SET((char)218) ;
        p149.size_x_SET(-1.6788583E38F) ;
        p149.position_valid_SET((char)62, PH) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON) ;
        p149.q_SET(new float[] {-1.2274825E38F, -7.9577375E37F, 3.0675128E38F, 3.2461729E38F}, 0, PH) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p149.angle_y_SET(-5.8321577E37F) ;
        p149.size_y_SET(-3.544155E37F) ;
        p149.y_SET(-3.2617636E38F, PH) ;
        p149.angle_x_SET(-1.5203644E38F) ;
        p149.z_SET(-2.961579E38F, PH) ;
        p149.x_SET(-2.9204662E38F, PH) ;
        p149.time_usec_SET(2522465229399542159L) ;
        p149.distance_SET(-1.4670968E38F) ;
        CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAV_FILTER_BIAS.add((src, ph, pack) ->
        {
            assert(pack.accel_0_GET() == -2.118853E37F);
            assert(pack.gyro_0_GET() == -3.0032957E38F);
            assert(pack.usec_GET() == 3222969582693498347L);
            assert(pack.gyro_2_GET() == -3.0544337E38F);
            assert(pack.accel_2_GET() == -3.2794823E38F);
            assert(pack.gyro_1_GET() == 1.0336674E38F);
            assert(pack.accel_1_GET() == -2.8329701E38F);
        });
        GroundControl.NAV_FILTER_BIAS p220 = CommunicationChannel.new_NAV_FILTER_BIAS();
        PH.setPack(p220);
        p220.gyro_0_SET(-3.0032957E38F) ;
        p220.accel_1_SET(-2.8329701E38F) ;
        p220.accel_0_SET(-2.118853E37F) ;
        p220.gyro_1_SET(1.0336674E38F) ;
        p220.gyro_2_SET(-3.0544337E38F) ;
        p220.usec_SET(3222969582693498347L) ;
        p220.accel_2_SET(-3.2794823E38F) ;
        CommunicationChannel.instance.send(p220);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_RADIO_CALIBRATION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.gyro_GET(),  new char[] {(char)65391, (char)50270}));
            assert(Arrays.equals(pack.rudder_GET(),  new char[] {(char)14412, (char)52418, (char)35859}));
            assert(Arrays.equals(pack.throttle_GET(),  new char[] {(char)60181, (char)12339, (char)24386, (char)14283, (char)13193}));
            assert(Arrays.equals(pack.pitch_GET(),  new char[] {(char)49876, (char)47942, (char)55831, (char)12137, (char)8214}));
            assert(Arrays.equals(pack.elevator_GET(),  new char[] {(char)15130, (char)60317, (char)32109}));
            assert(Arrays.equals(pack.aileron_GET(),  new char[] {(char)11400, (char)31681, (char)40808}));
        });
        GroundControl.RADIO_CALIBRATION p221 = CommunicationChannel.new_RADIO_CALIBRATION();
        PH.setPack(p221);
        p221.elevator_SET(new char[] {(char)15130, (char)60317, (char)32109}, 0) ;
        p221.aileron_SET(new char[] {(char)11400, (char)31681, (char)40808}, 0) ;
        p221.rudder_SET(new char[] {(char)14412, (char)52418, (char)35859}, 0) ;
        p221.gyro_SET(new char[] {(char)65391, (char)50270}, 0) ;
        p221.throttle_SET(new char[] {(char)60181, (char)12339, (char)24386, (char)14283, (char)13193}, 0) ;
        p221.pitch_SET(new char[] {(char)49876, (char)47942, (char)55831, (char)12137, (char)8214}, 0) ;
        CommunicationChannel.instance.send(p221);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UALBERTA_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == (char)74);
            assert(pack.pilot_GET() == (char)145);
            assert(pack.nav_mode_GET() == (char)198);
        });
        GroundControl.UALBERTA_SYS_STATUS p222 = CommunicationChannel.new_UALBERTA_SYS_STATUS();
        PH.setPack(p222);
        p222.pilot_SET((char)145) ;
        p222.nav_mode_SET((char)198) ;
        p222.mode_SET((char)74) ;
        CommunicationChannel.instance.send(p222);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pos_vert_accuracy_GET() == 5.7394245E37F);
            assert(pack.vel_ratio_GET() == 3.3539453E38F);
            assert(pack.mag_ratio_GET() == -1.025781E38F);
            assert(pack.pos_horiz_accuracy_GET() == -2.9008127E38F);
            assert(pack.pos_vert_ratio_GET() == 2.504028E38F);
            assert(pack.tas_ratio_GET() == 3.18871E38F);
            assert(pack.time_usec_GET() == 3318596233148754711L);
            assert(pack.flags_GET() == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS));
            assert(pack.pos_horiz_ratio_GET() == 1.8123682E38F);
            assert(pack.hagl_ratio_GET() == 3.1593494E38F);
        });
        GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.pos_horiz_accuracy_SET(-2.9008127E38F) ;
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS)) ;
        p230.tas_ratio_SET(3.18871E38F) ;
        p230.pos_vert_accuracy_SET(5.7394245E37F) ;
        p230.vel_ratio_SET(3.3539453E38F) ;
        p230.mag_ratio_SET(-1.025781E38F) ;
        p230.hagl_ratio_SET(3.1593494E38F) ;
        p230.pos_vert_ratio_SET(2.504028E38F) ;
        p230.time_usec_SET(3318596233148754711L) ;
        p230.pos_horiz_ratio_SET(1.8123682E38F) ;
        CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_alt_GET() == -1.5671069E38F);
            assert(pack.wind_x_GET() == 2.2499861E38F);
            assert(pack.wind_y_GET() == 1.743058E38F);
            assert(pack.var_vert_GET() == 3.1233282E38F);
            assert(pack.vert_accuracy_GET() == -2.1382414E38F);
            assert(pack.horiz_accuracy_GET() == -3.1377342E38F);
            assert(pack.wind_z_GET() == 1.0224092E38F);
            assert(pack.time_usec_GET() == 1663161652384231253L);
            assert(pack.var_horiz_GET() == 4.2780946E37F);
        });
        GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_z_SET(1.0224092E38F) ;
        p231.wind_y_SET(1.743058E38F) ;
        p231.wind_x_SET(2.2499861E38F) ;
        p231.time_usec_SET(1663161652384231253L) ;
        p231.var_vert_SET(3.1233282E38F) ;
        p231.horiz_accuracy_SET(-3.1377342E38F) ;
        p231.var_horiz_SET(4.2780946E37F) ;
        p231.wind_alt_SET(-1.5671069E38F) ;
        p231.vert_accuracy_SET(-2.1382414E38F) ;
        CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.hdop_GET() == 2.3809878E38F);
            assert(pack.lon_GET() == -1071567424);
            assert(pack.vn_GET() == -1.6194586E38F);
            assert(pack.vert_accuracy_GET() == -2.2635893E38F);
            assert(pack.time_week_ms_GET() == 3280623868L);
            assert(pack.fix_type_GET() == (char)39);
            assert(pack.vd_GET() == 1.2815158E38F);
            assert(pack.time_week_GET() == (char)53372);
            assert(pack.alt_GET() == 1.9565106E38F);
            assert(pack.vdop_GET() == 1.2420441E38F);
            assert(pack.horiz_accuracy_GET() == -3.3124661E38F);
            assert(pack.lat_GET() == 1706283027);
            assert(pack.speed_accuracy_GET() == -1.9554179E37F);
            assert(pack.time_usec_GET() == 7708336421049130450L);
            assert(pack.satellites_visible_GET() == (char)212);
            assert(pack.ve_GET() == 1.9314513E38F);
            assert(pack.ignore_flags_GET() == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT));
            assert(pack.gps_id_GET() == (char)170);
        });
        GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.vert_accuracy_SET(-2.2635893E38F) ;
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT)) ;
        p232.horiz_accuracy_SET(-3.3124661E38F) ;
        p232.lon_SET(-1071567424) ;
        p232.lat_SET(1706283027) ;
        p232.alt_SET(1.9565106E38F) ;
        p232.time_week_SET((char)53372) ;
        p232.gps_id_SET((char)170) ;
        p232.hdop_SET(2.3809878E38F) ;
        p232.time_week_ms_SET(3280623868L) ;
        p232.time_usec_SET(7708336421049130450L) ;
        p232.vn_SET(-1.6194586E38F) ;
        p232.vd_SET(1.2815158E38F) ;
        p232.satellites_visible_SET((char)212) ;
        p232.fix_type_SET((char)39) ;
        p232.vdop_SET(1.2420441E38F) ;
        p232.ve_SET(1.9314513E38F) ;
        p232.speed_accuracy_SET(-1.9554179E37F) ;
        CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)86);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)190, (char)201, (char)138, (char)167, (char)106, (char)39, (char)24, (char)4, (char)0, (char)175, (char)126, (char)213, (char)133, (char)37, (char)44, (char)195, (char)47, (char)132, (char)196, (char)154, (char)98, (char)249, (char)23, (char)215, (char)188, (char)123, (char)245, (char)135, (char)35, (char)234, (char)129, (char)8, (char)194, (char)19, (char)141, (char)237, (char)156, (char)94, (char)41, (char)248, (char)204, (char)138, (char)9, (char)219, (char)198, (char)81, (char)57, (char)61, (char)86, (char)18, (char)46, (char)246, (char)59, (char)58, (char)163, (char)135, (char)4, (char)190, (char)224, (char)146, (char)174, (char)143, (char)48, (char)255, (char)159, (char)23, (char)110, (char)139, (char)228, (char)159, (char)201, (char)215, (char)196, (char)85, (char)44, (char)106, (char)41, (char)109, (char)104, (char)223, (char)90, (char)58, (char)112, (char)161, (char)235, (char)221, (char)160, (char)30, (char)66, (char)102, (char)192, (char)157, (char)251, (char)108, (char)238, (char)68, (char)48, (char)81, (char)90, (char)97, (char)99, (char)206, (char)88, (char)141, (char)35, (char)17, (char)134, (char)246, (char)228, (char)224, (char)215, (char)100, (char)83, (char)188, (char)131, (char)156, (char)80, (char)12, (char)201, (char)54, (char)124, (char)223, (char)126, (char)197, (char)127, (char)211, (char)129, (char)179, (char)247, (char)249, (char)61, (char)50, (char)186, (char)24, (char)131, (char)213, (char)210, (char)76, (char)181, (char)42, (char)36, (char)15, (char)118, (char)62, (char)206, (char)160, (char)214, (char)27, (char)210, (char)181, (char)43, (char)12, (char)227, (char)208, (char)219, (char)147, (char)164, (char)11, (char)195, (char)57, (char)185, (char)7, (char)155, (char)244, (char)43, (char)170, (char)245, (char)115, (char)189, (char)230, (char)105, (char)0, (char)250, (char)79, (char)213, (char)103, (char)144, (char)160, (char)90, (char)144}));
            assert(pack.flags_GET() == (char)107);
        });
        GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)107) ;
        p233.data__SET(new char[] {(char)190, (char)201, (char)138, (char)167, (char)106, (char)39, (char)24, (char)4, (char)0, (char)175, (char)126, (char)213, (char)133, (char)37, (char)44, (char)195, (char)47, (char)132, (char)196, (char)154, (char)98, (char)249, (char)23, (char)215, (char)188, (char)123, (char)245, (char)135, (char)35, (char)234, (char)129, (char)8, (char)194, (char)19, (char)141, (char)237, (char)156, (char)94, (char)41, (char)248, (char)204, (char)138, (char)9, (char)219, (char)198, (char)81, (char)57, (char)61, (char)86, (char)18, (char)46, (char)246, (char)59, (char)58, (char)163, (char)135, (char)4, (char)190, (char)224, (char)146, (char)174, (char)143, (char)48, (char)255, (char)159, (char)23, (char)110, (char)139, (char)228, (char)159, (char)201, (char)215, (char)196, (char)85, (char)44, (char)106, (char)41, (char)109, (char)104, (char)223, (char)90, (char)58, (char)112, (char)161, (char)235, (char)221, (char)160, (char)30, (char)66, (char)102, (char)192, (char)157, (char)251, (char)108, (char)238, (char)68, (char)48, (char)81, (char)90, (char)97, (char)99, (char)206, (char)88, (char)141, (char)35, (char)17, (char)134, (char)246, (char)228, (char)224, (char)215, (char)100, (char)83, (char)188, (char)131, (char)156, (char)80, (char)12, (char)201, (char)54, (char)124, (char)223, (char)126, (char)197, (char)127, (char)211, (char)129, (char)179, (char)247, (char)249, (char)61, (char)50, (char)186, (char)24, (char)131, (char)213, (char)210, (char)76, (char)181, (char)42, (char)36, (char)15, (char)118, (char)62, (char)206, (char)160, (char)214, (char)27, (char)210, (char)181, (char)43, (char)12, (char)227, (char)208, (char)219, (char)147, (char)164, (char)11, (char)195, (char)57, (char)185, (char)7, (char)155, (char)244, (char)43, (char)170, (char)245, (char)115, (char)189, (char)230, (char)105, (char)0, (char)250, (char)79, (char)213, (char)103, (char)144, (char)160, (char)90, (char)144}, 0) ;
        p233.len_SET((char)86) ;
        CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == (short) -12928);
            assert(pack.airspeed_GET() == (char)49);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.heading_sp_GET() == (short)26771);
            assert(pack.altitude_sp_GET() == (short) -24399);
            assert(pack.gps_nsat_GET() == (char)139);
            assert(pack.failsafe_GET() == (char)247);
            assert(pack.airspeed_sp_GET() == (char)26);
            assert(pack.roll_GET() == (short)656);
            assert(pack.altitude_amsl_GET() == (short)21166);
            assert(pack.temperature_air_GET() == (byte)43);
            assert(pack.throttle_GET() == (byte) - 71);
            assert(pack.base_mode_GET() == (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED));
            assert(pack.wp_distance_GET() == (char)21084);
            assert(pack.longitude_GET() == -2036646644);
            assert(pack.heading_GET() == (char)16128);
            assert(pack.latitude_GET() == 1181835735);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.custom_mode_GET() == 197185662L);
            assert(pack.temperature_GET() == (byte) - 60);
            assert(pack.wp_num_GET() == (char)208);
            assert(pack.climb_rate_GET() == (byte) - 16);
            assert(pack.battery_remaining_GET() == (char)62);
            assert(pack.groundspeed_GET() == (char)82);
        });
        GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED)) ;
        p234.wp_num_SET((char)208) ;
        p234.climb_rate_SET((byte) - 16) ;
        p234.altitude_amsl_SET((short)21166) ;
        p234.longitude_SET(-2036646644) ;
        p234.failsafe_SET((char)247) ;
        p234.altitude_sp_SET((short) -24399) ;
        p234.latitude_SET(1181835735) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        p234.temperature_SET((byte) - 60) ;
        p234.heading_sp_SET((short)26771) ;
        p234.airspeed_sp_SET((char)26) ;
        p234.custom_mode_SET(197185662L) ;
        p234.wp_distance_SET((char)21084) ;
        p234.roll_SET((short)656) ;
        p234.pitch_SET((short) -12928) ;
        p234.airspeed_SET((char)49) ;
        p234.gps_nsat_SET((char)139) ;
        p234.throttle_SET((byte) - 71) ;
        p234.groundspeed_SET((char)82) ;
        p234.temperature_air_SET((byte)43) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p234.heading_SET((char)16128) ;
        p234.battery_remaining_SET((char)62) ;
        CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.clipping_0_GET() == 632183258L);
            assert(pack.clipping_2_GET() == 3300247373L);
            assert(pack.clipping_1_GET() == 1889169039L);
            assert(pack.vibration_x_GET() == 2.7930229E38F);
            assert(pack.vibration_y_GET() == 4.2966835E37F);
            assert(pack.vibration_z_GET() == -4.224432E37F);
            assert(pack.time_usec_GET() == 6665092616267459741L);
        });
        GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(6665092616267459741L) ;
        p241.clipping_2_SET(3300247373L) ;
        p241.clipping_1_SET(1889169039L) ;
        p241.vibration_z_SET(-4.224432E37F) ;
        p241.vibration_x_SET(2.7930229E38F) ;
        p241.vibration_y_SET(4.2966835E37F) ;
        p241.clipping_0_SET(632183258L) ;
        CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.4478249E38F, 2.3739372E38F, -1.2305972E38F, -2.784829E38F}));
            assert(pack.time_usec_TRY(ph) == 2935681336349419432L);
            assert(pack.approach_x_GET() == 1.4193138E38F);
            assert(pack.altitude_GET() == 1239199209);
            assert(pack.z_GET() == -8.698789E37F);
            assert(pack.longitude_GET() == -1585319302);
            assert(pack.approach_z_GET() == 1.8115735E38F);
            assert(pack.y_GET() == 1.2870826E38F);
            assert(pack.x_GET() == 1.4840816E38F);
            assert(pack.latitude_GET() == 4542716);
            assert(pack.approach_y_GET() == -1.3835932E38F);
        });
        GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.z_SET(-8.698789E37F) ;
        p242.altitude_SET(1239199209) ;
        p242.approach_z_SET(1.8115735E38F) ;
        p242.latitude_SET(4542716) ;
        p242.approach_y_SET(-1.3835932E38F) ;
        p242.approach_x_SET(1.4193138E38F) ;
        p242.x_SET(1.4840816E38F) ;
        p242.longitude_SET(-1585319302) ;
        p242.q_SET(new float[] {1.4478249E38F, 2.3739372E38F, -1.2305972E38F, -2.784829E38F}, 0) ;
        p242.y_SET(1.2870826E38F) ;
        p242.time_usec_SET(2935681336349419432L, PH) ;
        CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_z_GET() == -1.9128758E38F);
            assert(pack.approach_x_GET() == 3.02638E38F);
            assert(pack.x_GET() == -7.511294E37F);
            assert(pack.latitude_GET() == -1247599118);
            assert(pack.longitude_GET() == -1268931789);
            assert(pack.altitude_GET() == 1563088783);
            assert(pack.target_system_GET() == (char)93);
            assert(pack.y_GET() == -2.0685082E38F);
            assert(pack.approach_y_GET() == -2.2506493E38F);
            assert(pack.z_GET() == -1.8583809E36F);
            assert(pack.time_usec_TRY(ph) == 2447202568143446767L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.9397492E38F, 1.3436839E38F, -2.0956177E38F, 2.0467608E38F}));
        });
        GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.approach_z_SET(-1.9128758E38F) ;
        p243.target_system_SET((char)93) ;
        p243.z_SET(-1.8583809E36F) ;
        p243.time_usec_SET(2447202568143446767L, PH) ;
        p243.approach_x_SET(3.02638E38F) ;
        p243.q_SET(new float[] {1.9397492E38F, 1.3436839E38F, -2.0956177E38F, 2.0467608E38F}, 0) ;
        p243.longitude_SET(-1268931789) ;
        p243.altitude_SET(1563088783) ;
        p243.x_SET(-7.511294E37F) ;
        p243.latitude_SET(-1247599118) ;
        p243.approach_y_SET(-2.2506493E38F) ;
        p243.y_SET(-2.0685082E38F) ;
        CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)20738);
            assert(pack.interval_us_GET() == -574792764);
        });
        GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-574792764) ;
        p244.message_id_SET((char)20738) ;
        CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
        });
        GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_MC) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND) ;
        CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.callsign_LEN(ph) == 6);
            assert(pack.callsign_TRY(ph).equals("svAjvl"));
            assert(pack.ICAO_address_GET() == 2384935873L);
            assert(pack.heading_GET() == (char)24246);
            assert(pack.tslc_GET() == (char)189);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_EMERGENCY_SURFACE);
            assert(pack.squawk_GET() == (char)318);
            assert(pack.hor_velocity_GET() == (char)12549);
            assert(pack.lat_GET() == 1918770450);
            assert(pack.lon_GET() == 1919403632);
            assert(pack.altitude_GET() == -1012289672);
            assert(pack.ver_velocity_GET() == (short)25795);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.flags_GET() == (ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                                        ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN));
        });
        GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.hor_velocity_SET((char)12549) ;
        p246.callsign_SET("svAjvl", PH) ;
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN)) ;
        p246.altitude_SET(-1012289672) ;
        p246.ver_velocity_SET((short)25795) ;
        p246.lat_SET(1918770450) ;
        p246.ICAO_address_SET(2384935873L) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_EMERGENCY_SURFACE) ;
        p246.lon_SET(1919403632) ;
        p246.tslc_SET((char)189) ;
        p246.heading_SET((char)24246) ;
        p246.squawk_SET((char)318) ;
        CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.time_to_minimum_delta_GET() == -3.2548248E38F);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
            assert(pack.id_GET() == 160059338L);
            assert(pack.altitude_minimum_delta_GET() == 1.5269827E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            assert(pack.threat_level_GET() == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW));
            assert(pack.horizontal_minimum_delta_GET() == -9.494118E37F);
        });
        GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.id_SET(160059338L) ;
        p247.time_to_minimum_delta_SET(-3.2548248E38F) ;
        p247.horizontal_minimum_delta_SET(-9.494118E37F) ;
        p247.altitude_minimum_delta_SET(1.5269827E38F) ;
        p247.threat_level_SET((MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW)) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY) ;
        CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)255);
            assert(pack.target_component_GET() == (char)184);
            assert(pack.target_system_GET() == (char)39);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)234, (char)139, (char)148, (char)108, (char)87, (char)0, (char)69, (char)133, (char)57, (char)246, (char)70, (char)210, (char)115, (char)30, (char)172, (char)186, (char)33, (char)6, (char)77, (char)84, (char)46, (char)226, (char)159, (char)19, (char)67, (char)239, (char)253, (char)50, (char)43, (char)249, (char)216, (char)158, (char)72, (char)21, (char)13, (char)165, (char)178, (char)182, (char)171, (char)43, (char)112, (char)241, (char)72, (char)6, (char)184, (char)182, (char)5, (char)139, (char)91, (char)191, (char)32, (char)191, (char)208, (char)210, (char)105, (char)74, (char)62, (char)158, (char)7, (char)75, (char)72, (char)110, (char)242, (char)16, (char)189, (char)32, (char)178, (char)115, (char)213, (char)183, (char)76, (char)94, (char)73, (char)142, (char)60, (char)84, (char)207, (char)187, (char)76, (char)63, (char)248, (char)221, (char)109, (char)231, (char)35, (char)55, (char)127, (char)54, (char)58, (char)61, (char)84, (char)214, (char)193, (char)120, (char)250, (char)11, (char)207, (char)1, (char)218, (char)209, (char)72, (char)40, (char)109, (char)14, (char)82, (char)54, (char)189, (char)121, (char)155, (char)250, (char)124, (char)95, (char)243, (char)84, (char)4, (char)229, (char)133, (char)243, (char)154, (char)207, (char)39, (char)173, (char)4, (char)210, (char)220, (char)164, (char)3, (char)221, (char)106, (char)60, (char)160, (char)8, (char)29, (char)247, (char)184, (char)16, (char)133, (char)16, (char)114, (char)203, (char)246, (char)159, (char)70, (char)138, (char)183, (char)179, (char)189, (char)106, (char)78, (char)219, (char)51, (char)90, (char)60, (char)86, (char)11, (char)227, (char)171, (char)209, (char)221, (char)140, (char)48, (char)87, (char)125, (char)156, (char)45, (char)125, (char)139, (char)154, (char)85, (char)118, (char)107, (char)88, (char)200, (char)162, (char)229, (char)48, (char)9, (char)221, (char)229, (char)103, (char)113, (char)217, (char)246, (char)145, (char)87, (char)218, (char)85, (char)246, (char)93, (char)192, (char)125, (char)13, (char)89, (char)127, (char)181, (char)91, (char)159, (char)244, (char)66, (char)118, (char)249, (char)31, (char)108, (char)190, (char)63, (char)164, (char)245, (char)167, (char)95, (char)132, (char)67, (char)71, (char)203, (char)87, (char)186, (char)251, (char)42, (char)78, (char)94, (char)189, (char)125, (char)253, (char)46, (char)218, (char)236, (char)151, (char)167, (char)14, (char)51, (char)118, (char)123, (char)212, (char)232, (char)53, (char)84, (char)153, (char)26, (char)57, (char)3, (char)84, (char)197, (char)77, (char)75, (char)255, (char)20, (char)192, (char)79, (char)255, (char)124}));
            assert(pack.message_type_GET() == (char)36662);
        });
        GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)255) ;
        p248.target_component_SET((char)184) ;
        p248.message_type_SET((char)36662) ;
        p248.target_system_SET((char)39) ;
        p248.payload_SET(new char[] {(char)234, (char)139, (char)148, (char)108, (char)87, (char)0, (char)69, (char)133, (char)57, (char)246, (char)70, (char)210, (char)115, (char)30, (char)172, (char)186, (char)33, (char)6, (char)77, (char)84, (char)46, (char)226, (char)159, (char)19, (char)67, (char)239, (char)253, (char)50, (char)43, (char)249, (char)216, (char)158, (char)72, (char)21, (char)13, (char)165, (char)178, (char)182, (char)171, (char)43, (char)112, (char)241, (char)72, (char)6, (char)184, (char)182, (char)5, (char)139, (char)91, (char)191, (char)32, (char)191, (char)208, (char)210, (char)105, (char)74, (char)62, (char)158, (char)7, (char)75, (char)72, (char)110, (char)242, (char)16, (char)189, (char)32, (char)178, (char)115, (char)213, (char)183, (char)76, (char)94, (char)73, (char)142, (char)60, (char)84, (char)207, (char)187, (char)76, (char)63, (char)248, (char)221, (char)109, (char)231, (char)35, (char)55, (char)127, (char)54, (char)58, (char)61, (char)84, (char)214, (char)193, (char)120, (char)250, (char)11, (char)207, (char)1, (char)218, (char)209, (char)72, (char)40, (char)109, (char)14, (char)82, (char)54, (char)189, (char)121, (char)155, (char)250, (char)124, (char)95, (char)243, (char)84, (char)4, (char)229, (char)133, (char)243, (char)154, (char)207, (char)39, (char)173, (char)4, (char)210, (char)220, (char)164, (char)3, (char)221, (char)106, (char)60, (char)160, (char)8, (char)29, (char)247, (char)184, (char)16, (char)133, (char)16, (char)114, (char)203, (char)246, (char)159, (char)70, (char)138, (char)183, (char)179, (char)189, (char)106, (char)78, (char)219, (char)51, (char)90, (char)60, (char)86, (char)11, (char)227, (char)171, (char)209, (char)221, (char)140, (char)48, (char)87, (char)125, (char)156, (char)45, (char)125, (char)139, (char)154, (char)85, (char)118, (char)107, (char)88, (char)200, (char)162, (char)229, (char)48, (char)9, (char)221, (char)229, (char)103, (char)113, (char)217, (char)246, (char)145, (char)87, (char)218, (char)85, (char)246, (char)93, (char)192, (char)125, (char)13, (char)89, (char)127, (char)181, (char)91, (char)159, (char)244, (char)66, (char)118, (char)249, (char)31, (char)108, (char)190, (char)63, (char)164, (char)245, (char)167, (char)95, (char)132, (char)67, (char)71, (char)203, (char)87, (char)186, (char)251, (char)42, (char)78, (char)94, (char)189, (char)125, (char)253, (char)46, (char)218, (char)236, (char)151, (char)167, (char)14, (char)51, (char)118, (char)123, (char)212, (char)232, (char)53, (char)84, (char)153, (char)26, (char)57, (char)3, (char)84, (char)197, (char)77, (char)75, (char)255, (char)20, (char)192, (char)79, (char)255, (char)124}, 0) ;
        CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.address_GET() == (char)46443);
            assert(pack.ver_GET() == (char)42);
            assert(pack.type_GET() == (char)31);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)95, (byte)57, (byte) - 34, (byte)18, (byte)27, (byte) - 112, (byte)86, (byte)72, (byte) - 89, (byte)10, (byte)31, (byte) - 33, (byte)105, (byte)42, (byte) - 109, (byte) - 95, (byte)32, (byte)36, (byte)0, (byte) - 82, (byte) - 81, (byte)11, (byte)77, (byte) - 30, (byte)37, (byte) - 3, (byte) - 80, (byte) - 89, (byte)113, (byte) - 61, (byte)53, (byte) - 11}));
        });
        GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.ver_SET((char)42) ;
        p249.value_SET(new byte[] {(byte)95, (byte)57, (byte) - 34, (byte)18, (byte)27, (byte) - 112, (byte)86, (byte)72, (byte) - 89, (byte)10, (byte)31, (byte) - 33, (byte)105, (byte)42, (byte) - 109, (byte) - 95, (byte)32, (byte)36, (byte)0, (byte) - 82, (byte) - 81, (byte)11, (byte)77, (byte) - 30, (byte)37, (byte) - 3, (byte) - 80, (byte) - 89, (byte)113, (byte) - 61, (byte)53, (byte) - 11}, 0) ;
        p249.type_SET((char)31) ;
        p249.address_SET((char)46443) ;
        CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7157161027345178548L);
            assert(pack.z_GET() == -9.509624E37F);
            assert(pack.x_GET() == 3.2753883E38F);
            assert(pack.y_GET() == 2.5380144E38F);
            assert(pack.name_LEN(ph) == 7);
            assert(pack.name_TRY(ph).equals("tftuuik"));
        });
        GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("tftuuik", PH) ;
        p250.x_SET(3.2753883E38F) ;
        p250.y_SET(2.5380144E38F) ;
        p250.time_usec_SET(7157161027345178548L) ;
        p250.z_SET(-9.509624E37F) ;
        CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 3.07649E37F);
            assert(pack.name_LEN(ph) == 6);
            assert(pack.name_TRY(ph).equals("yejBdq"));
            assert(pack.time_boot_ms_GET() == 4165488466L);
        });
        GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(3.07649E37F) ;
        p251.name_SET("yejBdq", PH) ;
        p251.time_boot_ms_SET(4165488466L) ;
        CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 645239189L);
            assert(pack.name_LEN(ph) == 9);
            assert(pack.name_TRY(ph).equals("Ncejojvyw"));
            assert(pack.value_GET() == 626351775);
        });
        GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(645239189L) ;
        p252.name_SET("Ncejojvyw", PH) ;
        p252.value_SET(626351775) ;
        CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 31);
            assert(pack.text_TRY(ph).equals("wsgnmzOshrtqiduoSijfoqQadircweg"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_WARNING);
        });
        GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_WARNING) ;
        p253.text_SET("wsgnmzOshrtqiduoSijfoqQadircweg", PH) ;
        CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.ind_GET() == (char)134);
            assert(pack.time_boot_ms_GET() == 3553516903L);
            assert(pack.value_GET() == -2.4960123E38F);
        });
        GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(3553516903L) ;
        p254.ind_SET((char)134) ;
        p254.value_SET(-2.4960123E38F) ;
        CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)39);
            assert(pack.initial_timestamp_GET() == 875192182847099221L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)118, (char)87, (char)139, (char)140, (char)114, (char)137, (char)156, (char)145, (char)82, (char)90, (char)96, (char)40, (char)128, (char)122, (char)226, (char)87, (char)179, (char)244, (char)250, (char)218, (char)186, (char)171, (char)23, (char)19, (char)7, (char)237, (char)96, (char)178, (char)11, (char)54, (char)1, (char)70}));
            assert(pack.target_component_GET() == (char)227);
        });
        GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_component_SET((char)227) ;
        p256.target_system_SET((char)39) ;
        p256.secret_key_SET(new char[] {(char)118, (char)87, (char)139, (char)140, (char)114, (char)137, (char)156, (char)145, (char)82, (char)90, (char)96, (char)40, (char)128, (char)122, (char)226, (char)87, (char)179, (char)244, (char)250, (char)218, (char)186, (char)171, (char)23, (char)19, (char)7, (char)237, (char)96, (char)178, (char)11, (char)54, (char)1, (char)70}, 0) ;
        p256.initial_timestamp_SET(875192182847099221L) ;
        CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.state_GET() == (char)43);
            assert(pack.time_boot_ms_GET() == 3855266184L);
            assert(pack.last_change_ms_GET() == 90705380L);
        });
        GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.last_change_ms_SET(90705380L) ;
        p257.state_SET((char)43) ;
        p257.time_boot_ms_SET(3855266184L) ;
        CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.tune_LEN(ph) == 23);
            assert(pack.tune_TRY(ph).equals("RynokrwuCwsytgfplVoihiy"));
            assert(pack.target_system_GET() == (char)30);
            assert(pack.target_component_GET() == (char)132);
        });
        GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_component_SET((char)132) ;
        p258.tune_SET("RynokrwuCwsytgfplVoihiy", PH) ;
        p258.target_system_SET((char)30) ;
        CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.sensor_size_h_GET() == -8.378266E37F);
            assert(pack.resolution_v_GET() == (char)15486);
            assert(pack.firmware_version_GET() == 3857155506L);
            assert(pack.lens_id_GET() == (char)10);
            assert(pack.sensor_size_v_GET() == -1.2437458E37F);
            assert(pack.resolution_h_GET() == (char)2913);
            assert(pack.cam_definition_uri_LEN(ph) == 8);
            assert(pack.cam_definition_uri_TRY(ph).equals("nihEjfoh"));
            assert(pack.flags_GET() == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO));
            assert(pack.cam_definition_version_GET() == (char)45885);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)64, (char)163, (char)38, (char)109, (char)94, (char)143, (char)30, (char)240, (char)26, (char)0, (char)79, (char)224, (char)119, (char)160, (char)177, (char)144, (char)191, (char)2, (char)221, (char)5, (char)170, (char)159, (char)228, (char)133, (char)77, (char)70, (char)56, (char)221, (char)139, (char)247, (char)31, (char)7}));
            assert(pack.focal_length_GET() == -7.9501493E37F);
            assert(pack.time_boot_ms_GET() == 3905301670L);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)179, (char)141, (char)165, (char)60, (char)73, (char)85, (char)54, (char)196, (char)183, (char)93, (char)33, (char)148, (char)92, (char)57, (char)98, (char)93, (char)236, (char)210, (char)161, (char)123, (char)191, (char)23, (char)29, (char)190, (char)105, (char)196, (char)79, (char)198, (char)135, (char)255, (char)58, (char)110}));
        });
        GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.resolution_h_SET((char)2913) ;
        p259.cam_definition_version_SET((char)45885) ;
        p259.time_boot_ms_SET(3905301670L) ;
        p259.sensor_size_h_SET(-8.378266E37F) ;
        p259.cam_definition_uri_SET("nihEjfoh", PH) ;
        p259.lens_id_SET((char)10) ;
        p259.firmware_version_SET(3857155506L) ;
        p259.sensor_size_v_SET(-1.2437458E37F) ;
        p259.resolution_v_SET((char)15486) ;
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO)) ;
        p259.focal_length_SET(-7.9501493E37F) ;
        p259.model_name_SET(new char[] {(char)179, (char)141, (char)165, (char)60, (char)73, (char)85, (char)54, (char)196, (char)183, (char)93, (char)33, (char)148, (char)92, (char)57, (char)98, (char)93, (char)236, (char)210, (char)161, (char)123, (char)191, (char)23, (char)29, (char)190, (char)105, (char)196, (char)79, (char)198, (char)135, (char)255, (char)58, (char)110}, 0) ;
        p259.vendor_name_SET(new char[] {(char)64, (char)163, (char)38, (char)109, (char)94, (char)143, (char)30, (char)240, (char)26, (char)0, (char)79, (char)224, (char)119, (char)160, (char)177, (char)144, (char)191, (char)2, (char)221, (char)5, (char)170, (char)159, (char)228, (char)133, (char)77, (char)70, (char)56, (char)221, (char)139, (char)247, (char)31, (char)7}, 0) ;
        CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 125015753L);
            assert(pack.mode_id_GET() == (CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY |
                                          CAMERA_MODE.CAMERA_MODE_IMAGE));
        });
        GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET((CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY |
                          CAMERA_MODE.CAMERA_MODE_IMAGE)) ;
        p260.time_boot_ms_SET(125015753L) ;
        CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.read_speed_GET() == -2.4003763E38F);
            assert(pack.storage_count_GET() == (char)215);
            assert(pack.write_speed_GET() == 5.036817E37F);
            assert(pack.used_capacity_GET() == 5.947775E37F);
            assert(pack.time_boot_ms_GET() == 2070925502L);
            assert(pack.status_GET() == (char)56);
            assert(pack.storage_id_GET() == (char)29);
            assert(pack.available_capacity_GET() == 1.5852313E38F);
            assert(pack.total_capacity_GET() == 2.3945808E38F);
        });
        GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.status_SET((char)56) ;
        p261.used_capacity_SET(5.947775E37F) ;
        p261.total_capacity_SET(2.3945808E38F) ;
        p261.read_speed_SET(-2.4003763E38F) ;
        p261.time_boot_ms_SET(2070925502L) ;
        p261.write_speed_SET(5.036817E37F) ;
        p261.available_capacity_SET(1.5852313E38F) ;
        p261.storage_count_SET((char)215) ;
        p261.storage_id_SET((char)29) ;
        CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.available_capacity_GET() == 1.0305169E37F);
            assert(pack.image_status_GET() == (char)178);
            assert(pack.recording_time_ms_GET() == 3912416651L);
            assert(pack.time_boot_ms_GET() == 3536745970L);
            assert(pack.image_interval_GET() == -3.1672729E38F);
            assert(pack.video_status_GET() == (char)48);
        });
        GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.available_capacity_SET(1.0305169E37F) ;
        p262.video_status_SET((char)48) ;
        p262.recording_time_ms_SET(3912416651L) ;
        p262.time_boot_ms_SET(3536745970L) ;
        p262.image_status_SET((char)178) ;
        p262.image_interval_SET(-3.1672729E38F) ;
        CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.file_url_LEN(ph) == 197);
            assert(pack.file_url_TRY(ph).equals("HmhSclckdpfovuZspKovmslkipvktyatgphpbhcxtikqvbktdcsengrymbazvmeunlnwkazbcZdRyiLCSzlwzagfcnptbzgszygfvlukuhnqowhfwcpkkbijrfxyenraljxqkxkyznucbgafutlNeyruwaydCrrrzmqvfquvqproMtrhvzgxqfzgfmycythsciyqo"));
            assert(pack.lat_GET() == -1084998943);
            assert(pack.camera_id_GET() == (char)16);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.3763861E38F, 7.438708E36F, -3.2082937E38F, 1.3320007E38F}));
            assert(pack.capture_result_GET() == (byte) - 48);
            assert(pack.relative_alt_GET() == 2059571754);
            assert(pack.alt_GET() == 1536293340);
            assert(pack.time_boot_ms_GET() == 3436935073L);
            assert(pack.image_index_GET() == -1426450715);
            assert(pack.lon_GET() == 32046545);
            assert(pack.time_utc_GET() == 2036977004136450623L);
        });
        GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.file_url_SET("HmhSclckdpfovuZspKovmslkipvktyatgphpbhcxtikqvbktdcsengrymbazvmeunlnwkazbcZdRyiLCSzlwzagfcnptbzgszygfvlukuhnqowhfwcpkkbijrfxyenraljxqkxkyznucbgafutlNeyruwaydCrrrzmqvfquvqproMtrhvzgxqfzgfmycythsciyqo", PH) ;
        p263.camera_id_SET((char)16) ;
        p263.lat_SET(-1084998943) ;
        p263.lon_SET(32046545) ;
        p263.alt_SET(1536293340) ;
        p263.time_utc_SET(2036977004136450623L) ;
        p263.q_SET(new float[] {1.3763861E38F, 7.438708E36F, -3.2082937E38F, 1.3320007E38F}, 0) ;
        p263.capture_result_SET((byte) - 48) ;
        p263.time_boot_ms_SET(3436935073L) ;
        p263.relative_alt_SET(2059571754) ;
        p263.image_index_SET(-1426450715) ;
        CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 527248706L);
            assert(pack.flight_uuid_GET() == 6934824543701808347L);
            assert(pack.takeoff_time_utc_GET() == 7554804843939123052L);
            assert(pack.arming_time_utc_GET() == 7726498629972540875L);
        });
        GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.takeoff_time_utc_SET(7554804843939123052L) ;
        p264.time_boot_ms_SET(527248706L) ;
        p264.arming_time_utc_SET(7726498629972540875L) ;
        p264.flight_uuid_SET(6934824543701808347L) ;
        CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 1.2125929E38F);
            assert(pack.yaw_GET() == 1.3703906E38F);
            assert(pack.roll_GET() == 3.0025818E38F);
            assert(pack.time_boot_ms_GET() == 1453041586L);
        });
        GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.yaw_SET(1.3703906E38F) ;
        p265.roll_SET(3.0025818E38F) ;
        p265.pitch_SET(1.2125929E38F) ;
        p265.time_boot_ms_SET(1453041586L) ;
        CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)61338);
            assert(pack.target_component_GET() == (char)157);
            assert(pack.target_system_GET() == (char)172);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)124, (char)61, (char)78, (char)235, (char)124, (char)122, (char)116, (char)127, (char)40, (char)87, (char)147, (char)101, (char)14, (char)191, (char)204, (char)40, (char)208, (char)216, (char)136, (char)107, (char)119, (char)125, (char)67, (char)136, (char)254, (char)138, (char)199, (char)151, (char)63, (char)58, (char)118, (char)116, (char)14, (char)231, (char)188, (char)124, (char)71, (char)85, (char)239, (char)66, (char)130, (char)140, (char)29, (char)192, (char)175, (char)115, (char)89, (char)151, (char)209, (char)67, (char)115, (char)94, (char)116, (char)107, (char)159, (char)157, (char)105, (char)84, (char)117, (char)254, (char)94, (char)176, (char)136, (char)20, (char)144, (char)108, (char)78, (char)37, (char)253, (char)172, (char)208, (char)70, (char)48, (char)20, (char)218, (char)201, (char)173, (char)95, (char)61, (char)44, (char)241, (char)20, (char)180, (char)212, (char)39, (char)33, (char)58, (char)26, (char)115, (char)151, (char)251, (char)254, (char)145, (char)98, (char)22, (char)57, (char)86, (char)132, (char)47, (char)216, (char)12, (char)172, (char)23, (char)118, (char)8, (char)215, (char)171, (char)0, (char)117, (char)22, (char)103, (char)244, (char)225, (char)133, (char)253, (char)168, (char)199, (char)190, (char)78, (char)6, (char)78, (char)79, (char)238, (char)208, (char)89, (char)166, (char)252, (char)107, (char)126, (char)151, (char)97, (char)144, (char)219, (char)159, (char)150, (char)204, (char)105, (char)125, (char)92, (char)191, (char)206, (char)81, (char)153, (char)227, (char)118, (char)50, (char)14, (char)176, (char)118, (char)76, (char)237, (char)81, (char)66, (char)161, (char)141, (char)142, (char)113, (char)45, (char)26, (char)176, (char)109, (char)52, (char)157, (char)252, (char)244, (char)27, (char)163, (char)70, (char)189, (char)111, (char)208, (char)97, (char)99, (char)184, (char)20, (char)208, (char)170, (char)127, (char)152, (char)212, (char)144, (char)170, (char)110, (char)26, (char)174, (char)243, (char)239, (char)59, (char)6, (char)129, (char)242, (char)192, (char)153, (char)49, (char)144, (char)131, (char)247, (char)222, (char)175, (char)191, (char)69, (char)236, (char)2, (char)149, (char)68, (char)250, (char)239, (char)186, (char)16, (char)182, (char)13, (char)28, (char)35, (char)22, (char)169, (char)110, (char)88, (char)94, (char)147, (char)104, (char)94, (char)41, (char)203, (char)246, (char)139, (char)24, (char)107, (char)177, (char)228, (char)212, (char)204, (char)198, (char)174, (char)94, (char)97, (char)59, (char)121, (char)250, (char)86, (char)137, (char)219, (char)164, (char)30, (char)253, (char)143, (char)80, (char)49, (char)150, (char)234}));
            assert(pack.first_message_offset_GET() == (char)53);
            assert(pack.length_GET() == (char)140);
        });
        GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.data__SET(new char[] {(char)124, (char)61, (char)78, (char)235, (char)124, (char)122, (char)116, (char)127, (char)40, (char)87, (char)147, (char)101, (char)14, (char)191, (char)204, (char)40, (char)208, (char)216, (char)136, (char)107, (char)119, (char)125, (char)67, (char)136, (char)254, (char)138, (char)199, (char)151, (char)63, (char)58, (char)118, (char)116, (char)14, (char)231, (char)188, (char)124, (char)71, (char)85, (char)239, (char)66, (char)130, (char)140, (char)29, (char)192, (char)175, (char)115, (char)89, (char)151, (char)209, (char)67, (char)115, (char)94, (char)116, (char)107, (char)159, (char)157, (char)105, (char)84, (char)117, (char)254, (char)94, (char)176, (char)136, (char)20, (char)144, (char)108, (char)78, (char)37, (char)253, (char)172, (char)208, (char)70, (char)48, (char)20, (char)218, (char)201, (char)173, (char)95, (char)61, (char)44, (char)241, (char)20, (char)180, (char)212, (char)39, (char)33, (char)58, (char)26, (char)115, (char)151, (char)251, (char)254, (char)145, (char)98, (char)22, (char)57, (char)86, (char)132, (char)47, (char)216, (char)12, (char)172, (char)23, (char)118, (char)8, (char)215, (char)171, (char)0, (char)117, (char)22, (char)103, (char)244, (char)225, (char)133, (char)253, (char)168, (char)199, (char)190, (char)78, (char)6, (char)78, (char)79, (char)238, (char)208, (char)89, (char)166, (char)252, (char)107, (char)126, (char)151, (char)97, (char)144, (char)219, (char)159, (char)150, (char)204, (char)105, (char)125, (char)92, (char)191, (char)206, (char)81, (char)153, (char)227, (char)118, (char)50, (char)14, (char)176, (char)118, (char)76, (char)237, (char)81, (char)66, (char)161, (char)141, (char)142, (char)113, (char)45, (char)26, (char)176, (char)109, (char)52, (char)157, (char)252, (char)244, (char)27, (char)163, (char)70, (char)189, (char)111, (char)208, (char)97, (char)99, (char)184, (char)20, (char)208, (char)170, (char)127, (char)152, (char)212, (char)144, (char)170, (char)110, (char)26, (char)174, (char)243, (char)239, (char)59, (char)6, (char)129, (char)242, (char)192, (char)153, (char)49, (char)144, (char)131, (char)247, (char)222, (char)175, (char)191, (char)69, (char)236, (char)2, (char)149, (char)68, (char)250, (char)239, (char)186, (char)16, (char)182, (char)13, (char)28, (char)35, (char)22, (char)169, (char)110, (char)88, (char)94, (char)147, (char)104, (char)94, (char)41, (char)203, (char)246, (char)139, (char)24, (char)107, (char)177, (char)228, (char)212, (char)204, (char)198, (char)174, (char)94, (char)97, (char)59, (char)121, (char)250, (char)86, (char)137, (char)219, (char)164, (char)30, (char)253, (char)143, (char)80, (char)49, (char)150, (char)234}, 0) ;
        p266.sequence_SET((char)61338) ;
        p266.first_message_offset_SET((char)53) ;
        p266.target_component_SET((char)157) ;
        p266.target_system_SET((char)172) ;
        p266.length_SET((char)140) ;
        CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.first_message_offset_GET() == (char)99);
            assert(pack.target_system_GET() == (char)235);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)159, (char)169, (char)157, (char)61, (char)221, (char)120, (char)57, (char)94, (char)41, (char)162, (char)70, (char)246, (char)160, (char)210, (char)70, (char)49, (char)101, (char)93, (char)8, (char)104, (char)88, (char)170, (char)14, (char)191, (char)243, (char)86, (char)176, (char)135, (char)75, (char)127, (char)106, (char)104, (char)1, (char)179, (char)207, (char)184, (char)194, (char)37, (char)241, (char)202, (char)68, (char)90, (char)135, (char)192, (char)53, (char)178, (char)121, (char)79, (char)206, (char)249, (char)9, (char)237, (char)123, (char)158, (char)169, (char)154, (char)175, (char)91, (char)74, (char)198, (char)178, (char)51, (char)95, (char)132, (char)164, (char)120, (char)51, (char)107, (char)193, (char)248, (char)217, (char)64, (char)218, (char)67, (char)229, (char)112, (char)26, (char)101, (char)201, (char)41, (char)248, (char)242, (char)200, (char)159, (char)21, (char)103, (char)179, (char)216, (char)222, (char)73, (char)66, (char)190, (char)25, (char)248, (char)211, (char)13, (char)248, (char)62, (char)171, (char)186, (char)93, (char)100, (char)139, (char)168, (char)125, (char)180, (char)139, (char)60, (char)56, (char)88, (char)78, (char)115, (char)196, (char)89, (char)55, (char)118, (char)165, (char)213, (char)210, (char)60, (char)79, (char)148, (char)14, (char)105, (char)195, (char)249, (char)237, (char)28, (char)16, (char)171, (char)134, (char)28, (char)2, (char)214, (char)230, (char)96, (char)135, (char)187, (char)182, (char)141, (char)195, (char)102, (char)37, (char)208, (char)57, (char)172, (char)1, (char)140, (char)146, (char)116, (char)170, (char)175, (char)166, (char)69, (char)215, (char)212, (char)40, (char)130, (char)108, (char)78, (char)210, (char)97, (char)58, (char)164, (char)182, (char)234, (char)222, (char)243, (char)99, (char)112, (char)119, (char)4, (char)136, (char)209, (char)14, (char)173, (char)42, (char)234, (char)102, (char)67, (char)37, (char)11, (char)202, (char)216, (char)162, (char)9, (char)193, (char)243, (char)17, (char)9, (char)118, (char)200, (char)80, (char)221, (char)119, (char)111, (char)38, (char)53, (char)146, (char)116, (char)189, (char)76, (char)137, (char)176, (char)22, (char)233, (char)10, (char)166, (char)223, (char)237, (char)181, (char)191, (char)192, (char)130, (char)86, (char)180, (char)104, (char)78, (char)192, (char)8, (char)138, (char)131, (char)9, (char)129, (char)186, (char)194, (char)92, (char)212, (char)56, (char)167, (char)113, (char)155, (char)58, (char)162, (char)101, (char)245, (char)50, (char)17, (char)26, (char)176, (char)114, (char)37, (char)246, (char)244, (char)143, (char)64, (char)219, (char)21, (char)173}));
            assert(pack.length_GET() == (char)0);
            assert(pack.target_component_GET() == (char)130);
            assert(pack.sequence_GET() == (char)12438);
        });
        GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.first_message_offset_SET((char)99) ;
        p267.target_system_SET((char)235) ;
        p267.target_component_SET((char)130) ;
        p267.length_SET((char)0) ;
        p267.sequence_SET((char)12438) ;
        p267.data__SET(new char[] {(char)159, (char)169, (char)157, (char)61, (char)221, (char)120, (char)57, (char)94, (char)41, (char)162, (char)70, (char)246, (char)160, (char)210, (char)70, (char)49, (char)101, (char)93, (char)8, (char)104, (char)88, (char)170, (char)14, (char)191, (char)243, (char)86, (char)176, (char)135, (char)75, (char)127, (char)106, (char)104, (char)1, (char)179, (char)207, (char)184, (char)194, (char)37, (char)241, (char)202, (char)68, (char)90, (char)135, (char)192, (char)53, (char)178, (char)121, (char)79, (char)206, (char)249, (char)9, (char)237, (char)123, (char)158, (char)169, (char)154, (char)175, (char)91, (char)74, (char)198, (char)178, (char)51, (char)95, (char)132, (char)164, (char)120, (char)51, (char)107, (char)193, (char)248, (char)217, (char)64, (char)218, (char)67, (char)229, (char)112, (char)26, (char)101, (char)201, (char)41, (char)248, (char)242, (char)200, (char)159, (char)21, (char)103, (char)179, (char)216, (char)222, (char)73, (char)66, (char)190, (char)25, (char)248, (char)211, (char)13, (char)248, (char)62, (char)171, (char)186, (char)93, (char)100, (char)139, (char)168, (char)125, (char)180, (char)139, (char)60, (char)56, (char)88, (char)78, (char)115, (char)196, (char)89, (char)55, (char)118, (char)165, (char)213, (char)210, (char)60, (char)79, (char)148, (char)14, (char)105, (char)195, (char)249, (char)237, (char)28, (char)16, (char)171, (char)134, (char)28, (char)2, (char)214, (char)230, (char)96, (char)135, (char)187, (char)182, (char)141, (char)195, (char)102, (char)37, (char)208, (char)57, (char)172, (char)1, (char)140, (char)146, (char)116, (char)170, (char)175, (char)166, (char)69, (char)215, (char)212, (char)40, (char)130, (char)108, (char)78, (char)210, (char)97, (char)58, (char)164, (char)182, (char)234, (char)222, (char)243, (char)99, (char)112, (char)119, (char)4, (char)136, (char)209, (char)14, (char)173, (char)42, (char)234, (char)102, (char)67, (char)37, (char)11, (char)202, (char)216, (char)162, (char)9, (char)193, (char)243, (char)17, (char)9, (char)118, (char)200, (char)80, (char)221, (char)119, (char)111, (char)38, (char)53, (char)146, (char)116, (char)189, (char)76, (char)137, (char)176, (char)22, (char)233, (char)10, (char)166, (char)223, (char)237, (char)181, (char)191, (char)192, (char)130, (char)86, (char)180, (char)104, (char)78, (char)192, (char)8, (char)138, (char)131, (char)9, (char)129, (char)186, (char)194, (char)92, (char)212, (char)56, (char)167, (char)113, (char)155, (char)58, (char)162, (char)101, (char)245, (char)50, (char)17, (char)26, (char)176, (char)114, (char)37, (char)246, (char)244, (char)143, (char)64, (char)219, (char)21, (char)173}, 0) ;
        CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)108);
            assert(pack.target_system_GET() == (char)124);
            assert(pack.sequence_GET() == (char)55774);
        });
        GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_component_SET((char)108) ;
        p268.sequence_SET((char)55774) ;
        p268.target_system_SET((char)124) ;
        CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.bitrate_GET() == 3897664036L);
            assert(pack.framerate_GET() == -6.440873E37F);
            assert(pack.resolution_h_GET() == (char)35966);
            assert(pack.rotation_GET() == (char)54132);
            assert(pack.status_GET() == (char)172);
            assert(pack.camera_id_GET() == (char)89);
            assert(pack.resolution_v_GET() == (char)16965);
            assert(pack.uri_LEN(ph) == 200);
            assert(pack.uri_TRY(ph).equals("jyaqnzrjyzdbkjccGqremtJbjpxfnlvGxunbtdfxjonvmpiavrFLLifcmtqlzLixrsvKnndbqyyhafvfutfytzlxnccXrrhvlglvnojtiYirCmlusubuvfjgviniulEkfFVcrkvfmrssiANvhbkcygIobikadqmykmqbbteAhdwoiOdfresuiuunksuovXSnqJxvegoB"));
        });
        GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.resolution_h_SET((char)35966) ;
        p269.status_SET((char)172) ;
        p269.uri_SET("jyaqnzrjyzdbkjccGqremtJbjpxfnlvGxunbtdfxjonvmpiavrFLLifcmtqlzLixrsvKnndbqyyhafvfutfytzlxnccXrrhvlglvnojtiYirCmlusubuvfjgviniulEkfFVcrkvfmrssiANvhbkcygIobikadqmykmqbbteAhdwoiOdfresuiuunksuovXSnqJxvegoB", PH) ;
        p269.framerate_SET(-6.440873E37F) ;
        p269.camera_id_SET((char)89) ;
        p269.rotation_SET((char)54132) ;
        p269.resolution_v_SET((char)16965) ;
        p269.bitrate_SET(3897664036L) ;
        CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)190);
            assert(pack.framerate_GET() == 1.1022433E38F);
            assert(pack.uri_LEN(ph) == 88);
            assert(pack.uri_TRY(ph).equals("xnxpiWafzzpheSxhjcegjjtuvjMahtsihwmimexrtvbskdknnuteikoucMaitwynofnxyyrGeztpvqzujdisWssz"));
            assert(pack.rotation_GET() == (char)60405);
            assert(pack.resolution_h_GET() == (char)49962);
            assert(pack.camera_id_GET() == (char)52);
            assert(pack.resolution_v_GET() == (char)40226);
            assert(pack.bitrate_GET() == 3391989062L);
            assert(pack.target_component_GET() == (char)214);
        });
        GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.framerate_SET(1.1022433E38F) ;
        p270.camera_id_SET((char)52) ;
        p270.uri_SET("xnxpiWafzzpheSxhjcegjjtuvjMahtsihwmimexrtvbskdknnuteikoucMaitwynofnxyyrGeztpvqzujdisWssz", PH) ;
        p270.target_component_SET((char)214) ;
        p270.resolution_v_SET((char)40226) ;
        p270.rotation_SET((char)60405) ;
        p270.target_system_SET((char)190) ;
        p270.resolution_h_SET((char)49962) ;
        p270.bitrate_SET(3391989062L) ;
        CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 5);
            assert(pack.ssid_TRY(ph).equals("AeoSt"));
            assert(pack.password_LEN(ph) == 2);
            assert(pack.password_TRY(ph).equals("ij"));
        });
        GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("ij", PH) ;
        p299.ssid_SET("AeoSt", PH) ;
        CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)85, (char)190, (char)138, (char)154, (char)177, (char)161, (char)208, (char)163}));
            assert(pack.version_GET() == (char)1967);
            assert(pack.max_version_GET() == (char)38984);
            assert(pack.min_version_GET() == (char)16022);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)246, (char)231, (char)240, (char)190, (char)186, (char)118, (char)159, (char)157}));
        });
        GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.library_version_hash_SET(new char[] {(char)246, (char)231, (char)240, (char)190, (char)186, (char)118, (char)159, (char)157}, 0) ;
        p300.spec_version_hash_SET(new char[] {(char)85, (char)190, (char)138, (char)154, (char)177, (char)161, (char)208, (char)163}, 0) ;
        p300.min_version_SET((char)16022) ;
        p300.max_version_SET((char)38984) ;
        p300.version_SET((char)1967) ;
        CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
            assert(pack.time_usec_GET() == 2362150394622108604L);
            assert(pack.uptime_sec_GET() == 128898786L);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
            assert(pack.vendor_specific_status_code_GET() == (char)26016);
            assert(pack.sub_mode_GET() == (char)212);
        });
        GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.sub_mode_SET((char)212) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE) ;
        p310.vendor_specific_status_code_SET((char)26016) ;
        p310.time_usec_SET(2362150394622108604L) ;
        p310.uptime_sec_SET(128898786L) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING) ;
        CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.hw_version_minor_GET() == (char)11);
            assert(pack.sw_version_minor_GET() == (char)32);
            assert(pack.hw_version_major_GET() == (char)107);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)3, (char)84, (char)81, (char)108, (char)120, (char)252, (char)101, (char)128, (char)102, (char)172, (char)79, (char)33, (char)43, (char)150, (char)220, (char)42}));
            assert(pack.uptime_sec_GET() == 3336892337L);
            assert(pack.sw_version_major_GET() == (char)154);
            assert(pack.time_usec_GET() == 4655999554519103434L);
            assert(pack.sw_vcs_commit_GET() == 3763490832L);
            assert(pack.name_LEN(ph) == 56);
            assert(pack.name_TRY(ph).equals("ukwykbkfzacteaqvpiyksPrgwelFoxwWodPodbtfFNrxlcxctgdvoepi"));
        });
        GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.name_SET("ukwykbkfzacteaqvpiyksPrgwelFoxwWodPodbtfFNrxlcxctgdvoepi", PH) ;
        p311.hw_unique_id_SET(new char[] {(char)3, (char)84, (char)81, (char)108, (char)120, (char)252, (char)101, (char)128, (char)102, (char)172, (char)79, (char)33, (char)43, (char)150, (char)220, (char)42}, 0) ;
        p311.sw_vcs_commit_SET(3763490832L) ;
        p311.sw_version_minor_SET((char)32) ;
        p311.sw_version_major_SET((char)154) ;
        p311.time_usec_SET(4655999554519103434L) ;
        p311.hw_version_minor_SET((char)11) ;
        p311.uptime_sec_SET(3336892337L) ;
        p311.hw_version_major_SET((char)107) ;
        CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short) -10523);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("llbmjtkm"));
            assert(pack.target_system_GET() == (char)197);
            assert(pack.target_component_GET() == (char)92);
        });
        GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_component_SET((char)92) ;
        p320.target_system_SET((char)197) ;
        p320.param_index_SET((short) -10523) ;
        p320.param_id_SET("llbmjtkm", PH) ;
        CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)131);
            assert(pack.target_system_GET() == (char)52);
        });
        GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)131) ;
        p321.target_system_SET((char)52) ;
        CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)42670);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
            assert(pack.param_index_GET() == (char)33437);
            assert(pack.param_value_LEN(ph) == 63);
            assert(pack.param_value_TRY(ph).equals("yxbzChbrlaixmvAcbgfgEcZLwhbukdauuidmWprkcupoqLrntljojsbpkrznuwf"));
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("vdie"));
        });
        GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_index_SET((char)33437) ;
        p322.param_count_SET((char)42670) ;
        p322.param_value_SET("yxbzChbrlaixmvAcbgfgEcZLwhbukdauuidmWprkcupoqLrntljojsbpkrznuwf", PH) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM) ;
        p322.param_id_SET("vdie", PH) ;
        CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)213);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("jKasllwSuwitzr"));
            assert(pack.target_component_GET() == (char)234);
            assert(pack.param_value_LEN(ph) == 51);
            assert(pack.param_value_TRY(ph).equals("xpuzpaxyslschedwMuiJdwoutkqyXoejhtkWamydjzcyvxcszsv"));
        });
        GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_component_SET((char)234) ;
        p323.param_value_SET("xpuzpaxyslschedwMuiJdwoutkqyXoejhtkWamydjzcyvxcszsv", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8) ;
        p323.target_system_SET((char)213) ;
        p323.param_id_SET("jKasllwSuwitzr", PH) ;
        CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("yiqdQantwprvbu"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
            assert(pack.param_value_LEN(ph) == 88);
            assert(pack.param_value_TRY(ph).equals("kavehjldkdQsowvpwumpuanvvgiygjqxzezepvtmhmxpuvhzvnztirplussiKcugdeMBghBrxoiexauzgsasicnk"));
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
        });
        GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED) ;
        p324.param_id_SET("yiqdQantwprvbu", PH) ;
        p324.param_value_SET("kavehjldkdQsowvpwumpuanvvgiygjqxzezepvtmhmxpuvhzvnztirplussiKcugdeMBghBrxoiexauzgsasicnk", PH) ;
        CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
        TestChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.max_distance_GET() == (char)14999);
            assert(pack.increment_GET() == (char)177);
            assert(pack.time_usec_GET() == 6178151485988930884L);
            assert(pack.min_distance_GET() == (char)41527);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)62848, (char)4859, (char)36005, (char)46931, (char)48347, (char)2627, (char)63979, (char)5960, (char)37076, (char)45944, (char)45006, (char)46060, (char)7889, (char)44691, (char)49742, (char)26870, (char)12488, (char)50417, (char)50325, (char)5868, (char)58391, (char)2901, (char)5683, (char)29043, (char)15060, (char)62733, (char)21867, (char)33537, (char)13468, (char)52984, (char)40094, (char)35901, (char)31510, (char)56291, (char)5153, (char)63150, (char)17718, (char)12424, (char)40987, (char)43524, (char)62106, (char)31053, (char)22770, (char)61723, (char)39427, (char)55255, (char)42416, (char)42347, (char)25901, (char)30343, (char)19462, (char)45478, (char)5807, (char)25777, (char)8495, (char)65446, (char)1215, (char)31369, (char)49870, (char)51386, (char)61181, (char)32362, (char)31394, (char)32346, (char)638, (char)22207, (char)31351, (char)5955, (char)32147, (char)36062, (char)6877, (char)51467}));
        });
        GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.min_distance_SET((char)41527) ;
        p330.max_distance_SET((char)14999) ;
        p330.distances_SET(new char[] {(char)62848, (char)4859, (char)36005, (char)46931, (char)48347, (char)2627, (char)63979, (char)5960, (char)37076, (char)45944, (char)45006, (char)46060, (char)7889, (char)44691, (char)49742, (char)26870, (char)12488, (char)50417, (char)50325, (char)5868, (char)58391, (char)2901, (char)5683, (char)29043, (char)15060, (char)62733, (char)21867, (char)33537, (char)13468, (char)52984, (char)40094, (char)35901, (char)31510, (char)56291, (char)5153, (char)63150, (char)17718, (char)12424, (char)40987, (char)43524, (char)62106, (char)31053, (char)22770, (char)61723, (char)39427, (char)55255, (char)42416, (char)42347, (char)25901, (char)30343, (char)19462, (char)45478, (char)5807, (char)25777, (char)8495, (char)65446, (char)1215, (char)31369, (char)49870, (char)51386, (char)61181, (char)32362, (char)31394, (char)32346, (char)638, (char)22207, (char)31351, (char)5955, (char)32147, (char)36062, (char)6877, (char)51467}, 0) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p330.time_usec_SET(6178151485988930884L) ;
        p330.increment_SET((char)177) ;
        CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(CommunicationChannel.instance.inputStream, TestChannel.instance.outputStream, TestChannel.instance);
    }

}
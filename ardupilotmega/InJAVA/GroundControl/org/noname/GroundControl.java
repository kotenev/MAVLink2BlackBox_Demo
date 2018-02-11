package org.noname;
import java.util.*;
import org.unirail.BlackBox.Host;
import org.unirail.BlackBox.Host.Pack.Meta.Field;
import org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds;
import static org.unirail.BlackBox.BitUtils.*;
import java.util.concurrent.ConcurrentLinkedQueue;


public class GroundControl extends Host
{

    /**
    *The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot
    *	 hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying
    *	 out the user interface based on the autopilot)*/
    public static class HEARTBEAT extends Pack
    {

        HEARTBEAT() { super(meta, 0); }
        HEARTBEAT(int bytes) { super(meta, bytes); }
        public long custom_mode_GET()//A bitfield for use for autopilot-specific flags.
        {  return (get_bytes(data,  0, 4)); }
        public char mavlink_version_GET()//MAVLink version, not writable by user, gets added by protocol because of magic data type: char_mavlink_versio
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public @MAV_TYPE int type_GET()//Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        {  return  0 + (int)get_bits(data, 40, 5); }
        public @MAV_AUTOPILOT int autopilot_GET()//Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        {  return  0 + (int)get_bits(data, 45, 5); }
        public @MAV_MODE_FLAG int base_mode_GET()//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
        {  return  1 + (int)get_bits(data, 50, 8); }
        public @MAV_STATE int system_status_GET()//System status flag, see MAV_STATE ENUM
        {  return  0 + (int)get_bits(data, 58, 4); }
        static final Meta meta = new Meta(0, 0, 1, 0, 8, 62);
    }/**
*The general system state. If the system is following the MAVLink standard, the system state is mainly
*	 defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and
*	 locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position
*	 setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined
*	 the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents
*	 the internal navigation state machine. The system status shows whether the system is currently active
*	 or not and if an emergency occured. During the CRITICAL and EMERGENCY states the MAV is still considered
*	 to be active, but should start emergency procedures autonomously. After a failure occured it should first
*	 move from active to critical to allow manual intervention and then move to emergency after a certain
*	 timeout*/
    public static class SYS_STATUS extends Pack
    {

        SYS_STATUS() { super(meta, 0); }
        SYS_STATUS(int bytes) { super(meta, bytes); }
        public char load_GET()//Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char voltage_battery_GET()//Battery voltage, in millivolts (1 = 1 millivolt)
        {  return (char)((char) get_bytes(data,  2, 2)); }
        /**
        *Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links
        *	 (packets that were corrupted on reception on the MAV*/
        public char drop_rate_comm_GET()
        {  return (char)((char) get_bytes(data,  4, 2)); }
        /**
        *Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
        *	 on reception on the MAV*/
        public char errors_comm_GET()
        {  return (char)((char) get_bytes(data,  6, 2)); }
        public char errors_count1_GET()//Autopilot-specific errors
        {  return (char)((char) get_bytes(data,  8, 2)); }
        public char errors_count2_GET()//Autopilot-specific errors
        {  return (char)((char) get_bytes(data,  10, 2)); }
        public char errors_count3_GET()//Autopilot-specific errors
        {  return (char)((char) get_bytes(data,  12, 2)); }
        public char errors_count4_GET()//Autopilot-specific errors
        {  return (char)((char) get_bytes(data,  14, 2)); }
        public short current_battery_GET()//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
        {  return (short)((short) get_bytes(data,  16, 2)); }
        public byte battery_remaining_GET()//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
        {  return (byte)((byte) get_bytes(data,  18, 1)); }
        /**
        *Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1:
        *	 present. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
        public @MAV_SYS_STATUS_SENSOR int onboard_control_sensors_present_GET()
        {  return  1 + (int)get_bits(data, 152, 26); }
        /**
        *Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of
        *	 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
        public @MAV_SYS_STATUS_SENSOR int onboard_control_sensors_enabled_GET()
        {  return  1 + (int)get_bits(data, 178, 26); }
        /**
        *Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not
        *	 enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
        public @MAV_SYS_STATUS_SENSOR int onboard_control_sensors_health_GET()
        {  return  1 + (int)get_bits(data, 204, 26); }
        static final Meta meta = new Meta(1, 8, 0, 0, 29, 230);
    }/**
*The system time is the time of the master clock, typically the computer clock of the main onboard computer*/
    public static class SYSTEM_TIME extends Pack
    {

        SYSTEM_TIME() { super(meta, 0); }
        SYSTEM_TIME(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp of the component clock since boot time in milliseconds.
        {  return (get_bytes(data,  0, 4)); }
        public long time_unix_usec_GET()//Timestamp of the master clock in microseconds since UNIX epoch.
        {  return (get_bytes(data,  4, 8)); }
        static final Meta meta = new Meta(2, 0, 1, 1, 12, 96);
    }/**
*Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
*	 This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled
*	 this way*/
    public static class POSITION_TARGET_LOCAL_NED extends Pack
    {

        POSITION_TARGET_LOCAL_NED() { super(meta, 0); }
        POSITION_TARGET_LOCAL_NED(int bytes) { super(meta, bytes); }
        /**
        *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
        *	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
        *	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
        *	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
        *	 bit 11: yaw, bit 12: yaw rat*/
        public char type_mask_GET()
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public long time_boot_ms_GET()//Timestamp in milliseconds since system boot
        {  return (get_bytes(data,  2, 4)); }
        public float x_GET()//X Position in NED frame in meters
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public float y_GET()//Y Position in NED frame in meters
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public float z_GET()//Z Position in NED frame in meters (note, altitude is negative in NED)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public float vx_GET()//X velocity in NED frame in meter / s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public float vy_GET()//Y velocity in NED frame in meter / s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public float vz_GET()//Z velocity in NED frame in meter / s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  26, 4))); }
        public float afx_GET()//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  30, 4))); }
        public float afy_GET()//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  34, 4))); }
        public float afz_GET()//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  38, 4))); }
        public float yaw_GET()//yaw setpoint in rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  42, 4))); }
        public float yaw_rate_GET()//yaw rate setpoint in rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  46, 4))); }
        /**
        *Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
        *	 =*/
        public @MAV_FRAME int coordinate_frame_GET()
        {  return  0 + (int)get_bits(data, 400, 4); }
        static final Meta meta = new Meta(3, 1, 1, 0, 51, 404);
    }/**
*A ping message either requesting or responding to a ping. This allows to measure the system latencies,
*	 including serial port, radio modem and UDP connections*/
    public static class PING extends Pack
    {

        PING() { super(meta, 0); }
        PING(int bytes) { super(meta, bytes); }
        public long seq_GET()//PING sequence
        {  return (get_bytes(data,  0, 4)); }
        public long time_usec_GET()//Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
        {  return (get_bytes(data,  4, 8)); }
        /**
        *0: request ping from all receiving systems, if greater than 0: message is a ping response and number is
        *	 the system id of the requesting syste*/
        public char target_system_GET()
        {  return (char)((char) get_bytes(data,  12, 1)); }
        /**
        *0: request ping from all receiving components, if greater than 0: message is a ping response and number
        *	 is the system id of the requesting syste*/
        public char target_component_GET()
        {  return (char)((char) get_bytes(data,  13, 1)); }
        static final Meta meta = new Meta(4, 0, 1, 1, 14, 112);
    }/**
*Request to control this MAV*/
    public static class CHANGE_OPERATOR_CONTROL extends Pack
    {

        CHANGE_OPERATOR_CONTROL() { super(meta, 0); }
        CHANGE_OPERATOR_CONTROL(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System the GCS requests control for
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char control_request_GET()//0: request control of this MAV, 1: Release control of this MAV
        {  return (char)((char) get_bytes(data,  1, 1)); }
        /**
        *0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use
        *	 the safest mode possible initially and then gradually move down the encryption level if it gets a NACK
        *	 message indicating an encryption mismatch*/
        public char version_GET()
        {  return (char)((char) get_bytes(data,  2, 1)); }
        /**
        *Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
        *	 characters may involve A-Z, a-z, 0-9, and "!?,.-*/
        public String passkey_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  24 && !try_visit_field(ph, 24)  ||  !try_visit_item(ph, 0)) return null;
            return new String(passkey_GET(ph, new char[ph.items], 0));
        }
        /**
        *Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
        *	 characters may involve A-Z, a-z, 0-9, and "!?,.-*/
        public char[] passkey_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int passkey_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  24 && !try_visit_field(ph, 24)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } static final Meta meta = new Meta(5, 0, 0, 0, 4, 24, 0, _J);
    }/**
*Accept / deny control of this MAV*/
    public static class CHANGE_OPERATOR_CONTROL_ACK extends Pack
    {

        CHANGE_OPERATOR_CONTROL_ACK() { super(meta, 0); }
        CHANGE_OPERATOR_CONTROL_ACK(int bytes) { super(meta, bytes); }
        public char gcs_system_id_GET()//ID of the GCS this message
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char control_request_GET()//0: request control of this MAV, 1: Release control of this MAV
        {  return (char)((char) get_bytes(data,  1, 1)); }
        /**
        *0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under
        *	 contro*/
        public char ack_GET()
        {  return (char)((char) get_bytes(data,  2, 1)); }
        static final Meta meta = new Meta(6, 0, 0, 0, 3, 24);
    }/**
*Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple,
*	 so transmitting the key requires an encrypted channel for true safety*/
    public static class AUTH_KEY extends Pack
    {

        AUTH_KEY() { super(meta, 0); }
        AUTH_KEY(int bytes) { super(meta, bytes); }
        public String key_TRY(Bounds.Inside ph)//key
        {
            if(ph.field_bit !=  0 && !try_visit_field(ph, 0)  ||  !try_visit_item(ph, 0)) return null;
            return new String(key_GET(ph, new char[ph.items], 0));
        }
        public char[] key_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //key
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int key_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  0 && !try_visit_field(ph, 0)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } static final Meta meta = new Meta(7, 0, 0, 0, 1, 0, 0, _W);
    }/**
*THIS INTERFACE IS DEPRECATED. USE COMMAND_LONG with MAV_CMD_DO_SET_MODE INSTEAD. Set the system mode,
*	 as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall
*	 aircraft, not only for one component*/
    public static class SET_MODE extends Pack
    {

        SET_MODE() { super(meta, 0); }
        SET_MODE(int bytes) { super(meta, bytes); }
        public long custom_mode_GET()//The new autopilot-specific mode. This field can be ignored by an autopilot.
        {  return (get_bytes(data,  0, 4)); }
        public char target_system_GET()//The system setting the mode
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public @MAV_MODE int base_mode_GET()//The new base mode
        {  return  en__P((int)get_bits(data, 40, 4)); }
        static final Meta meta = new Meta(11, 0, 1, 0, 6, 44);
    }/**
*value[float]. This allows to send a parameter to any other component (such as the GCS) without the need
*	 of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for
*	 different autopilots. See also http:qgroundcontrol.org/parameter_interface for a full documentation
*	 of QGroundControl and IMU code*/
    public static class PARAM_REQUEST_READ extends Pack
    {

        PARAM_REQUEST_READ() { super(meta, 0); }
        PARAM_REQUEST_READ(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public short param_index_GET()//Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored
        {  return (short)((short) get_bytes(data,  2, 2)); }
        /**
        *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
        *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	 storage if the ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        /**
        *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
        *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	 storage if the ID is stored as strin*/
        public char[] param_id_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_id_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } static final Meta meta = new Meta(20, 0, 0, 0, 5, 32, 0, _h);
    }/**
*Request all parameters of this component. After this request, all parameters are emitted.*/
    public static class PARAM_REQUEST_LIST extends Pack
    {

        PARAM_REQUEST_LIST() { super(meta, 0); }
        PARAM_REQUEST_LIST(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        static final Meta meta = new Meta(21, 0, 0, 0, 2, 16);
    }/**
*Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows
*	 the recipient to keep track of received parameters and allows him to re-request missing parameters after
*	 a loss or timeout*/
    public static class PARAM_VALUE extends Pack
    {

        PARAM_VALUE() { super(meta, 0); }
        PARAM_VALUE(int bytes) { super(meta, bytes); }
        public char param_count_GET()//Total number of onboard parameters
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char param_index_GET()//Index of this onboard parameter
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public float param_value_GET()//Onboard parameter value
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public @MAV_PARAM_TYPE int param_type_GET()//Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
        {  return  1 + (int)get_bits(data, 64, 4); }
        /**
        *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
        *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	 storage if the ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  68 && !try_visit_field(ph, 68)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        /**
        *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
        *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	 storage if the ID is stored as strin*/
        public char[] param_id_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_id_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  68 && !try_visit_field(ph, 68)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } static final Meta meta = new Meta(22, 2, 0, 0, 10, 68, 0, _w);
    }/**
*Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION
*	 MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component
*	 should acknowledge the new parameter value by sending a param_value message to all communication partners.
*	 This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending
*	 GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message*/
    public static class PARAM_SET extends Pack
    {

        PARAM_SET() { super(meta, 0); }
        PARAM_SET(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public float param_value_GET()//Onboard parameter value
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  2, 4))); }
        public @MAV_PARAM_TYPE int param_type_GET()//Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
        {  return  1 + (int)get_bits(data, 48, 4); }
        /**
        *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
        *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	 storage if the ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  52 && !try_visit_field(ph, 52)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        /**
        *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
        *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	 storage if the ID is stored as strin*/
        public char[] param_id_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_id_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  52 && !try_visit_field(ph, 52)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } static final Meta meta = new Meta(23, 0, 0, 0, 8, 52, 0, _o);
    }/**
*The global position, as returned by the Global Positioning System (GPS). This is
*	 NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
    public static class GPS_RAW_INT extends Pack
    {

        GPS_RAW_INT() { super(meta, 0); }
        GPS_RAW_INT(int bytes) { super(meta, bytes); }
        public char eph_GET()//GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char epv_GET()//GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char vel_GET()//GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
        {  return (char)((char) get_bytes(data,  4, 2)); }
        /**
        *Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
        *	 unknown, set to: UINT16_MA*/
        public char cog_GET()
        {  return (char)((char) get_bytes(data,  6, 2)); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  return (get_bytes(data,  8, 8)); }
        public int lat_GET()//Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  16, 4)); }
        public int lon_GET()//Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  20, 4)); }
        /**
        *Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide
        *	 the AMSL altitude in addition to the WGS84 altitude*/
        public int alt_GET()
        {  return (int)((int) get_bytes(data,  24, 4)); }
        public char satellites_visible_GET()//Number of satellites visible. If unknown, set to 255
        {  return (char)((char) get_bytes(data,  28, 1)); }
        public @GPS_FIX_TYPE int fix_type_GET()//See the GPS_FIX_TYPE enum.
        {  return  0 + (int)get_bits(data, 232, 4); }
        public int  alt_ellipsoid_TRY(Bounds.Inside ph)//Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up).
        {
            if(ph.field_bit !=  236 && !try_visit_field(ph, 236)) return 0;
            return (int)((int) get_bytes(data,  ph.BYTE, 4));
        }
        public long  h_acc_TRY(Bounds.Inside ph)//Position uncertainty in meters * 1000 (positive for up).
        {
            if(ph.field_bit !=  237 && !try_visit_field(ph, 237)) return 0;
            return (get_bytes(data,  ph.BYTE, 4));
        }
        public long  v_acc_TRY(Bounds.Inside ph)//Altitude uncertainty in meters * 1000 (positive for up).
        {
            if(ph.field_bit !=  238 && !try_visit_field(ph, 238)) return 0;
            return (get_bytes(data,  ph.BYTE, 4));
        }
        public long  vel_acc_TRY(Bounds.Inside ph)//Speed uncertainty in meters * 1000 (positive for up).
        {
            if(ph.field_bit !=  239 && !try_visit_field(ph, 239)) return 0;
            return (get_bytes(data,  ph.BYTE, 4));
        }
        public long  hdg_acc_TRY(Bounds.Inside ph)//Heading / track uncertainty in degrees * 1e5.
        {
            if(ph.field_bit !=  240 && !try_visit_field(ph, 240)) return 0;
            return (get_bytes(data,  ph.BYTE, 4));
        }
        static final Meta meta = new Meta(24, 4, 0, 1, 31, 236, 0, _Hd, _Cd, _rd, _Sd, _id);
    }/**
*The positioning status, as reported by GPS. This message is intended to display status information about
*	 each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate.
*	 This message can contain information for up to 20 satellites*/
    public static class GPS_STATUS extends Pack
    {

        GPS_STATUS() { super(meta, 0); }
        GPS_STATUS(int bytes) { super(meta, bytes); }
        public char satellites_visible_GET()//Number of satellites visible
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char[] satellite_prn_GET(char[]  dst_ch, int pos)  //Global satellite ID
        {
            for(int BYTE = 1, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] satellite_prn_GET()//Global satellite ID
        {return satellite_prn_GET(new char[20], 0);} public char[] satellite_used_GET(char[]  dst_ch, int pos)  //0: Satellite not used, 1: used for localization
        {
            for(int BYTE = 21, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] satellite_used_GET()//0: Satellite not used, 1: used for localization
        {return satellite_used_GET(new char[20], 0);} public char[] satellite_elevation_GET(char[]  dst_ch, int pos)  //Elevation (0: right on top of receiver, 90: on the horizon) of satellite
        {
            for(int BYTE = 41, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] satellite_elevation_GET()//Elevation (0: right on top of receiver, 90: on the horizon) of satellite
        {return satellite_elevation_GET(new char[20], 0);} public char[] satellite_azimuth_GET(char[]  dst_ch, int pos)  //Direction of satellite, 0: 0 deg, 255: 360 deg.
        {
            for(int BYTE = 61, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] satellite_azimuth_GET()//Direction of satellite, 0: 0 deg, 255: 360 deg.
        {return satellite_azimuth_GET(new char[20], 0);} public char[] satellite_snr_GET(char[]  dst_ch, int pos)  //Signal to noise ratio of satellite
        {
            for(int BYTE = 81, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] satellite_snr_GET()//Signal to noise ratio of satellite
        {return satellite_snr_GET(new char[20], 0);} static final Meta meta = new Meta(25, 0, 0, 0, 101, 808);
    }/**
*The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to
*	 the described unit*/
    public static class SCALED_IMU extends Pack
    {

        SCALED_IMU() { super(meta, 0); }
        SCALED_IMU(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public short xacc_GET()//X acceleration (mg)
        {  return (short)((short) get_bytes(data,  4, 2)); }
        public short yacc_GET()//Y acceleration (mg)
        {  return (short)((short) get_bytes(data,  6, 2)); }
        public short zacc_GET()//Z acceleration (mg)
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public short xgyro_GET()//Angular speed around X axis (millirad /sec)
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public short ygyro_GET()//Angular speed around Y axis (millirad /sec)
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public short zgyro_GET()//Angular speed around Z axis (millirad /sec)
        {  return (short)((short) get_bytes(data,  14, 2)); }
        public short xmag_GET()//X Magnetic field (milli tesla)
        {  return (short)((short) get_bytes(data,  16, 2)); }
        public short ymag_GET()//Y Magnetic field (milli tesla)
        {  return (short)((short) get_bytes(data,  18, 2)); }
        public short zmag_GET()//Z Magnetic field (milli tesla)
        {  return (short)((short) get_bytes(data,  20, 2)); }
        static final Meta meta = new Meta(26, 0, 1, 0, 22, 176);
    }/**
*The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw
*	 values without any scaling to allow data capture and system debugging*/
    public static class RAW_IMU extends Pack
    {

        RAW_IMU() { super(meta, 0); }
        RAW_IMU(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  return (get_bytes(data,  0, 8)); }
        public short xacc_GET()//X acceleration (raw)
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public short yacc_GET()//Y acceleration (raw)
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public short zacc_GET()//Z acceleration (raw)
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public short xgyro_GET()//Angular speed around X axis (raw)
        {  return (short)((short) get_bytes(data,  14, 2)); }
        public short ygyro_GET()//Angular speed around Y axis (raw)
        {  return (short)((short) get_bytes(data,  16, 2)); }
        public short zgyro_GET()//Angular speed around Z axis (raw)
        {  return (short)((short) get_bytes(data,  18, 2)); }
        public short xmag_GET()//X Magnetic field (raw)
        {  return (short)((short) get_bytes(data,  20, 2)); }
        public short ymag_GET()//Y Magnetic field (raw)
        {  return (short)((short) get_bytes(data,  22, 2)); }
        public short zmag_GET()//Z Magnetic field (raw)
        {  return (short)((short) get_bytes(data,  24, 2)); }
        static final Meta meta = new Meta(27, 0, 0, 1, 26, 208);
    }/**
*The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure
*	 sensor. The sensor values should be the raw, UNSCALED ADC values*/
    public static class RAW_PRESSURE extends Pack
    {

        RAW_PRESSURE() { super(meta, 0); }
        RAW_PRESSURE(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  return (get_bytes(data,  0, 8)); }
        public short press_abs_GET()//Absolute pressure (raw)
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public short press_diff1_GET()//Differential pressure 1 (raw, 0 if nonexistant)
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public short press_diff2_GET()//Differential pressure 2 (raw, 0 if nonexistant)
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public short temperature_GET()//Raw Temperature measurement (raw)
        {  return (short)((short) get_bytes(data,  14, 2)); }
        static final Meta meta = new Meta(28, 0, 0, 1, 16, 128);
    }/**
*The pressure readings for the typical setup of one absolute and differential pressure sensor. The units
*	 are as specified in each field*/
    public static class SCALED_PRESSURE extends Pack
    {

        SCALED_PRESSURE() { super(meta, 0); }
        SCALED_PRESSURE(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public float press_abs_GET()//Absolute pressure (hectopascal)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float press_diff_GET()//Differential pressure 1 (hectopascal)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public short temperature_GET()//Temperature measurement (0.01 degrees celsius)
        {  return (short)((short) get_bytes(data,  12, 2)); }
        static final Meta meta = new Meta(29, 0, 1, 0, 14, 112);
    }/**
*The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).*/
    public static class ATTITUDE extends Pack
    {

        ATTITUDE() { super(meta, 0); }
        ATTITUDE(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public float roll_GET()//Roll angle (rad, -pi..+pi)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float pitch_GET()//Pitch angle (rad, -pi..+pi)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float yaw_GET()//Yaw angle (rad, -pi..+pi)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float rollspeed_GET()//Roll angular speed (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float pitchspeed_GET()//Pitch angular speed (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float yawspeed_GET()//Yaw angular speed (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        static final Meta meta = new Meta(30, 0, 1, 0, 28, 224);
    }/**
*The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
*	 Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)*/
    public static class ATTITUDE_QUATERNION extends Pack
    {

        ATTITUDE_QUATERNION() { super(meta, 0); }
        ATTITUDE_QUATERNION(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public float q1_GET()//Quaternion component 1, w (1 in null-rotation)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float q2_GET()//Quaternion component 2, x (0 in null-rotation)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float q3_GET()//Quaternion component 3, y (0 in null-rotation)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float q4_GET()//Quaternion component 4, z (0 in null-rotation)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float rollspeed_GET()//Roll angular speed (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float pitchspeed_GET()//Pitch angular speed (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float yawspeed_GET()//Yaw angular speed (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        static final Meta meta = new Meta(31, 0, 1, 0, 32, 256);
    }/**
*The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
*	 Z-axis down (aeronautical frame, NED / north-east-down convention*/
    public static class LOCAL_POSITION_NED extends Pack
    {

        LOCAL_POSITION_NED() { super(meta, 0); }
        LOCAL_POSITION_NED(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public float x_GET()//X Position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float y_GET()//Y Position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float z_GET()//Z Position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float vx_GET()//X Speed
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float vy_GET()//Y Speed
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float vz_GET()//Z Speed
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        static final Meta meta = new Meta(32, 0, 1, 0, 28, 224);
    }/**
*nt.*/
    public static class GLOBAL_POSITION_INT extends Pack
    {

        GLOBAL_POSITION_INT() { super(meta, 0); }
        GLOBAL_POSITION_INT(int bytes) { super(meta, bytes); }
        public char hdg_GET()//Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  2, 4)); }
        public int lat_GET()//Latitude, expressed as degrees * 1E7
        {  return (int)((int) get_bytes(data,  6, 4)); }
        public int lon_GET()//Longitude, expressed as degrees * 1E7
        {  return (int)((int) get_bytes(data,  10, 4)); }
        /**
        *Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules
        *	 provide the AMSL as well*/
        public int alt_GET()
        {  return (int)((int) get_bytes(data,  14, 4)); }
        public int relative_alt_GET()//Altitude above ground in meters, expressed as * 1000 (millimeters)
        {  return (int)((int) get_bytes(data,  18, 4)); }
        public short vx_GET()//Ground X Speed (Latitude, positive north), expressed as m/s * 100
        {  return (short)((short) get_bytes(data,  22, 2)); }
        public short vy_GET()//Ground Y Speed (Longitude, positive east), expressed as m/s * 100
        {  return (short)((short) get_bytes(data,  24, 2)); }
        public short vz_GET()//Ground Z Speed (Altitude, positive down), expressed as m/s * 100
        {  return (short)((short) get_bytes(data,  26, 2)); }
        static final Meta meta = new Meta(33, 1, 1, 0, 28, 224);
    }/**
*The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are
*	 inactive should be set to UINT16_MAX*/
    public static class RC_CHANNELS_SCALED extends Pack
    {

        RC_CHANNELS_SCALED() { super(meta, 0); }
        RC_CHANNELS_SCALED(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        /**
        *Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
        *	 8 servos*/
        public char port_GET()
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public short chan1_scaled_GET()//RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        {  return (short)((short) get_bytes(data,  5, 2)); }
        public short chan2_scaled_GET()//RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        {  return (short)((short) get_bytes(data,  7, 2)); }
        public short chan3_scaled_GET()//RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        {  return (short)((short) get_bytes(data,  9, 2)); }
        public short chan4_scaled_GET()//RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        {  return (short)((short) get_bytes(data,  11, 2)); }
        public short chan5_scaled_GET()//RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        {  return (short)((short) get_bytes(data,  13, 2)); }
        public short chan6_scaled_GET()//RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        {  return (short)((short) get_bytes(data,  15, 2)); }
        public short chan7_scaled_GET()//RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        {  return (short)((short) get_bytes(data,  17, 2)); }
        public short chan8_scaled_GET()//RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        {  return (short)((short) get_bytes(data,  19, 2)); }
        public char rssi_GET()//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
        {  return (char)((char) get_bytes(data,  21, 1)); }
        static final Meta meta = new Meta(34, 0, 1, 0, 22, 176);
    }/**
*The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
*	 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification*/
    public static class RC_CHANNELS_RAW extends Pack
    {

        RC_CHANNELS_RAW() { super(meta, 0); }
        RC_CHANNELS_RAW(int bytes) { super(meta, bytes); }
        public char chan1_raw_GET()//RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char chan2_raw_GET()//RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char chan3_raw_GET()//RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public char chan4_raw_GET()//RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  6, 2)); }
        public char chan5_raw_GET()//RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  8, 2)); }
        public char chan6_raw_GET()//RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  10, 2)); }
        public char chan7_raw_GET()//RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  12, 2)); }
        public char chan8_raw_GET()//RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  14, 2)); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  16, 4)); }
        /**
        *Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
        *	 8 servos*/
        public char port_GET()
        {  return (char)((char) get_bytes(data,  20, 1)); }
        public char rssi_GET()//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
        {  return (char)((char) get_bytes(data,  21, 1)); }
        static final Meta meta = new Meta(35, 8, 1, 0, 22, 176);
    }/**
*The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The
*	 standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%*/
    public static class SERVO_OUTPUT_RAW extends Pack
    {

        SERVO_OUTPUT_RAW() { super(meta, 0); }
        SERVO_OUTPUT_RAW(int bytes) { super(meta, bytes); }
        public char servo1_raw_GET()//Servo output 1 value, in microseconds
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char servo2_raw_GET()//Servo output 2 value, in microseconds
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char servo3_raw_GET()//Servo output 3 value, in microseconds
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public char servo4_raw_GET()//Servo output 4 value, in microseconds
        {  return (char)((char) get_bytes(data,  6, 2)); }
        public char servo5_raw_GET()//Servo output 5 value, in microseconds
        {  return (char)((char) get_bytes(data,  8, 2)); }
        public char servo6_raw_GET()//Servo output 6 value, in microseconds
        {  return (char)((char) get_bytes(data,  10, 2)); }
        public char servo7_raw_GET()//Servo output 7 value, in microseconds
        {  return (char)((char) get_bytes(data,  12, 2)); }
        public char servo8_raw_GET()//Servo output 8 value, in microseconds
        {  return (char)((char) get_bytes(data,  14, 2)); }
        public long time_usec_GET()//Timestamp (microseconds since system boot)
        {  return (get_bytes(data,  16, 4)); }
        /**
        *Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode
        *	 more than 8 servos*/
        public char port_GET()
        {  return (char)((char) get_bytes(data,  20, 1)); }
        public char  servo9_raw_TRY(Bounds.Inside ph)//Servo output 9 value, in microseconds
        {
            if(ph.field_bit !=  168 && !try_visit_field(ph, 168)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 2));
        }
        public char  servo10_raw_TRY(Bounds.Inside ph)//Servo output 10 value, in microseconds
        {
            if(ph.field_bit !=  169 && !try_visit_field(ph, 169)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 2));
        }
        public char  servo11_raw_TRY(Bounds.Inside ph)//Servo output 11 value, in microseconds
        {
            if(ph.field_bit !=  170 && !try_visit_field(ph, 170)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 2));
        }
        public char  servo12_raw_TRY(Bounds.Inside ph)//Servo output 12 value, in microseconds
        {
            if(ph.field_bit !=  171 && !try_visit_field(ph, 171)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 2));
        }
        public char  servo13_raw_TRY(Bounds.Inside ph)//Servo output 13 value, in microseconds
        {
            if(ph.field_bit !=  172 && !try_visit_field(ph, 172)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 2));
        }
        public char  servo14_raw_TRY(Bounds.Inside ph)//Servo output 14 value, in microseconds
        {
            if(ph.field_bit !=  173 && !try_visit_field(ph, 173)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 2));
        }
        public char  servo15_raw_TRY(Bounds.Inside ph)//Servo output 15 value, in microseconds
        {
            if(ph.field_bit !=  174 && !try_visit_field(ph, 174)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 2));
        }
        public char  servo16_raw_TRY(Bounds.Inside ph)//Servo output 16 value, in microseconds
        {
            if(ph.field_bit !=  175 && !try_visit_field(ph, 175)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 2));
        }
        static final Meta meta = new Meta(36, 8, 1, 0, 22, 168, 0, _OB, _HB, _CB, _rB, _SB, _iB, _lB, _aB);
    }/**
*Request a partial list of mission items from the system/component. http:qgroundcontrol.org/mavlink/waypoint_protocol.
*	 If start and end index are the same, just send one waypoint*/
    public static class MISSION_REQUEST_PARTIAL_LIST extends Pack
    {

        MISSION_REQUEST_PARTIAL_LIST() { super(meta, 0); }
        MISSION_REQUEST_PARTIAL_LIST(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public short start_index_GET()//Start index, 0 by default
        {  return (short)((short) get_bytes(data,  2, 2)); }
        public short end_index_GET()//End index, -1 by default (-1: send list to end). Else a valid index of the list
        {  return (short)((short) get_bytes(data,  4, 2)); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYPE
        {  return  en__W((int)get_bits(data, 48, 3)); }
        static final Meta meta = new Meta(37, 0, 0, 0, 7, 51);
    }/**
*This message is sent to the MAV to write a partial list. If start index == end index, only one item will
*	 be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should
*	 be REJECTED*/
    public static class MISSION_WRITE_PARTIAL_LIST extends Pack
    {

        MISSION_WRITE_PARTIAL_LIST() { super(meta, 0); }
        MISSION_WRITE_PARTIAL_LIST(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public short start_index_GET()//Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
        {  return (short)((short) get_bytes(data,  2, 2)); }
        public short end_index_GET()//End index, equal or greater than start index.
        {  return (short)((short) get_bytes(data,  4, 2)); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYPE
        {  return  en__W((int)get_bits(data, 48, 3)); }
        static final Meta meta = new Meta(38, 0, 0, 0, 7, 51);
    }/**
*Message encoding a mission item. This message is emitted to announce
*	 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http:qgroundcontrol.org/mavlink/waypoint_protocol.*/
    public static class MISSION_ITEM extends Pack
    {

        MISSION_ITEM() { super(meta, 0); }
        MISSION_ITEM(int bytes) { super(meta, bytes); }
        public char seq_GET()//Sequence
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char current_GET()//false:0, true:1
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char autocontinue_GET()//autocontinue to next wp
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public float param1_GET()//PARAM1, see MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public float param2_GET()//PARAM2, see MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public float param3_GET()//PARAM3, see MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public float param4_GET()//PARAM4, see MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public float x_GET()//PARAM5 / local: x position, global: latitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public float y_GET()//PARAM6 / y position: global: longitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  26, 4))); }
        public float z_GET()//PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  30, 4))); }
        public @MAV_FRAME int frame_GET()//The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
        {  return  0 + (int)get_bits(data, 272, 4); }
        public @MAV_CMD int command_GET()//The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
        {  return  en__T((int)get_bits(data, 276, 8)); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYPE
        {  return  en__W((int)get_bits(data, 284, 3)); }
        static final Meta meta = new Meta(39, 1, 0, 0, 36, 287);
    }/**
*Request the information of the mission item with the sequence number seq. The response of the system to
*	 this message should be a MISSION_ITEM message. http:qgroundcontrol.org/mavlink/waypoint_protoco*/
    public static class MISSION_REQUEST extends Pack
    {

        MISSION_REQUEST() { super(meta, 0); }
        MISSION_REQUEST(int bytes) { super(meta, bytes); }
        public char seq_GET()//Sequence
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYPE
        {  return  en__W((int)get_bits(data, 32, 3)); }
        static final Meta meta = new Meta(40, 1, 0, 0, 5, 35);
    }/**
*Set the mission item with sequence number seq as current item. This means that the MAV will continue to
*	 this mission item on the shortest path (not following the mission items in-between)*/
    public static class MISSION_SET_CURRENT extends Pack
    {

        MISSION_SET_CURRENT() { super(meta, 0); }
        MISSION_SET_CURRENT(int bytes) { super(meta, bytes); }
        public char seq_GET()//Sequence
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  3, 1)); }
        static final Meta meta = new Meta(41, 1, 0, 0, 4, 32);
    }/**
*Message that announces the sequence number of the current active mission item. The MAV will fly towards
*	 this mission item*/
    public static class MISSION_CURRENT extends Pack
    {

        MISSION_CURRENT() { super(meta, 0); }
        MISSION_CURRENT(int bytes) { super(meta, bytes); }
        public char seq_GET()//Sequence
        {  return (char)((char) get_bytes(data,  0, 2)); }
        static final Meta meta = new Meta(42, 1, 0, 0, 2, 16);
    }/**
*Request the overall list of mission items from the system/component.*/
    public static class MISSION_REQUEST_LIST extends Pack
    {

        MISSION_REQUEST_LIST() { super(meta, 0); }
        MISSION_REQUEST_LIST(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYPE
        {  return  en__W((int)get_bits(data, 16, 3)); }
        static final Meta meta = new Meta(43, 0, 0, 0, 3, 19);
    }/**
*This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction.
*	 The GCS can then request the individual mission item based on the knowledge of the total number of waypoints*/
    public static class MISSION_COUNT extends Pack
    {

        MISSION_COUNT() { super(meta, 0); }
        MISSION_COUNT(int bytes) { super(meta, bytes); }
        public char count_GET()//Number of mission items in the sequence
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYPE
        {  return  en__W((int)get_bits(data, 32, 3)); }
        static final Meta meta = new Meta(44, 1, 0, 0, 5, 35);
    }/**
*Delete all mission items at once.*/
    public static class MISSION_CLEAR_ALL extends Pack
    {

        MISSION_CLEAR_ALL() { super(meta, 0); }
        MISSION_CLEAR_ALL(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYPE
        {  return  en__W((int)get_bits(data, 16, 3)); }
        static final Meta meta = new Meta(45, 0, 0, 0, 3, 19);
    }/**
*A certain mission item has been reached. The system will either hold this position (or circle on the orbit)
*	 or (if the autocontinue on the WP was set) continue to the next waypoint*/
    public static class MISSION_ITEM_REACHED extends Pack
    {

        MISSION_ITEM_REACHED() { super(meta, 0); }
        MISSION_ITEM_REACHED(int bytes) { super(meta, bytes); }
        public char seq_GET()//Sequence
        {  return (char)((char) get_bytes(data,  0, 2)); }
        static final Meta meta = new Meta(46, 1, 0, 0, 2, 16);
    }/**
*Ack message during waypoint handling. The type field states if this message is a positive ack (type=0)
*	 or if an error happened (type=non-zero)*/
    public static class MISSION_ACK extends Pack
    {

        MISSION_ACK() { super(meta, 0); }
        MISSION_ACK(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public @MAV_MISSION_RESULT int type_GET()//See MAV_MISSION_RESULT enum
        {  return  0 + (int)get_bits(data, 16, 4); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYPE
        {  return  en__W((int)get_bits(data, 20, 3)); }
        static final Meta meta = new Meta(47, 0, 0, 0, 3, 23);
    }/**
*As local waypoints exist, the global waypoint reference allows to transform between the local coordinate
*	 frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings
*	 are connected and the MAV should move from in- to outdoor*/
    public static class SET_GPS_GLOBAL_ORIGIN extends Pack
    {

        SET_GPS_GLOBAL_ORIGIN() { super(meta, 0); }
        SET_GPS_GLOBAL_ORIGIN(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public int latitude_GET()//Latitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  1, 4)); }
        public int longitude_GET()//Longitude (WGS84, in degrees * 1E7
        {  return (int)((int) get_bytes(data,  5, 4)); }
        public int altitude_GET()//Altitude (AMSL), in meters * 1000 (positive for up)
        {  return (int)((int) get_bytes(data,  9, 4)); }
        public long  time_usec_TRY(Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {
            if(ph.field_bit !=  104 && !try_visit_field(ph, 104)) return 0;
            return (get_bytes(data,  ph.BYTE, 8));
        }
        static final Meta meta = new Meta(48, 0, 0, 0, 14, 104, 0, _cM);
    }/**
*Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) positio*/
    public static class GPS_GLOBAL_ORIGIN extends Pack
    {

        GPS_GLOBAL_ORIGIN() { super(meta, 0); }
        GPS_GLOBAL_ORIGIN(int bytes) { super(meta, bytes); }
        public int latitude_GET()//Latitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  0, 4)); }
        public int longitude_GET()//Longitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  4, 4)); }
        public int altitude_GET()//Altitude (AMSL), in meters * 1000 (positive for up)
        {  return (int)((int) get_bytes(data,  8, 4)); }
        public long  time_usec_TRY(Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {
            if(ph.field_bit !=  96 && !try_visit_field(ph, 96)) return 0;
            return (get_bytes(data,  ph.BYTE, 8));
        }
        static final Meta meta = new Meta(49, 0, 0, 0, 13, 96, 0, _IM);
    }/**
*Bind a RC channel to a parameter. The parameter should change accoding to the RC channel value.*/
    public static class PARAM_MAP_RC extends Pack
    {

        PARAM_MAP_RC() { super(meta, 0); }
        PARAM_MAP_RC(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        /**
        *Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored),
        *	 send -2 to disable any existing map for this rc_channel_index*/
        public short param_index_GET()
        {  return (short)((short) get_bytes(data,  2, 2)); }
        /**
        *Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob
        *	 on the RC*/
        public char parameter_rc_channel_index_GET()
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public float param_value0_GET()//Initial parameter value
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  5, 4))); }
        public float scale_GET()//Scale, maps the RC range [-1, 1] to a parameter value
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  9, 4))); }
        /**
        *Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends
        *	 on implementation*/
        public float param_value_min_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  13, 4))); }
        /**
        *Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends
        *	 on implementation*/
        public float param_value_max_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  17, 4))); }
        /**
        *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
        *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	 storage if the ID is stored as strin*/
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  168 && !try_visit_field(ph, 168)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        /**
        *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
        *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
        *	 storage if the ID is stored as strin*/
        public char[] param_id_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_id_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  168 && !try_visit_field(ph, 168)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } static final Meta meta = new Meta(50, 0, 0, 0, 22, 168, 0, _GM);
    }/**
*Request the information of the mission item with the sequence number seq. The response of the system to
*	 this message should be a MISSION_ITEM_INT message. http:qgroundcontrol.org/mavlink/waypoint_protoco*/
    public static class MISSION_REQUEST_INT extends Pack
    {

        MISSION_REQUEST_INT() { super(meta, 0); }
        MISSION_REQUEST_INT(int bytes) { super(meta, bytes); }
        public char seq_GET()//Sequence
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYPE
        {  return  en__W((int)get_bits(data, 32, 3)); }
        static final Meta meta = new Meta(51, 1, 0, 0, 5, 35);
    }/**
*Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell
*	 the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national
*	 or competition regulations*/
    public static class SAFETY_SET_ALLOWED_AREA extends Pack
    {

        SAFETY_SET_ALLOWED_AREA() { super(meta, 0); }
        SAFETY_SET_ALLOWED_AREA(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public float p1x_GET()//x position 1 / Latitude 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  2, 4))); }
        public float p1y_GET()//y position 1 / Longitude 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public float p1z_GET()//z position 1 / Altitude 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public float p2x_GET()//x position 2 / Latitude 2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public float p2y_GET()//y position 2 / Longitude 2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public float p2z_GET()//z position 2 / Altitude 2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        /**
        *Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
        *	 with Z axis up or local, right handed, Z axis down*/
        public @MAV_FRAME int frame_GET()
        {  return  0 + (int)get_bits(data, 208, 4); }
        static final Meta meta = new Meta(54, 0, 0, 0, 27, 212);
    }/**
*Read out the safety zone the MAV currently assumes.*/
    public static class SAFETY_ALLOWED_AREA extends Pack
    {

        SAFETY_ALLOWED_AREA() { super(meta, 0); }
        SAFETY_ALLOWED_AREA(int bytes) { super(meta, bytes); }
        public float p1x_GET()//x position 1 / Latitude 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public float p1y_GET()//y position 1 / Longitude 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float p1z_GET()//z position 1 / Altitude 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float p2x_GET()//x position 2 / Latitude 2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float p2y_GET()//y position 2 / Longitude 2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float p2z_GET()//z position 2 / Altitude 2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        /**
        *Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
        *	 with Z axis up or local, right handed, Z axis down*/
        public @MAV_FRAME int frame_GET()
        {  return  0 + (int)get_bits(data, 192, 4); }
        static final Meta meta = new Meta(55, 0, 0, 0, 25, 196);
    }/**
*The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
*	 Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)*/
    public static class ATTITUDE_QUATERNION_COV extends Pack
    {

        ATTITUDE_QUATERNION_COV() { super(meta, 0); }
        ATTITUDE_QUATERNION_COV(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (microseconds since system boot or since UNIX epoch)
        {  return (get_bytes(data,  0, 8)); }
        public float[] q_GET(float[]  dst_ch, int pos)  //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
        {
            for(int BYTE = 8, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] q_GET()//Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
        {return q_GET(new float[4], 0);} public float rollspeed_GET()//Roll angular speed (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float pitchspeed_GET()//Pitch angular speed (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public float yawspeed_GET()//Yaw angular speed (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public float[] covariance_GET(float[]  dst_ch, int pos)  //Attitude covariance
        {
            for(int BYTE = 36, dst_max = pos + 9; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] covariance_GET()//Attitude covariance
        {return covariance_GET(new float[9], 0);} static final Meta meta = new Meta(61, 0, 0, 1, 72, 576);
    }/**
*The state of the fixed wing navigation and position controller.*/
    public static class NAV_CONTROLLER_OUTPUT extends Pack
    {

        NAV_CONTROLLER_OUTPUT() { super(meta, 0); }
        NAV_CONTROLLER_OUTPUT(int bytes) { super(meta, bytes); }
        public char wp_dist_GET()//Distance to active waypoint in meters
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public float nav_roll_GET()//Current desired roll in degrees
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  2, 4))); }
        public float nav_pitch_GET()//Current desired pitch in degrees
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public short nav_bearing_GET()//Current desired heading in degrees
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public short target_bearing_GET()//Bearing to current waypoint/target in degrees
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public float alt_error_GET()//Current altitude error in meters
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public float aspd_error_GET()//Current airspeed error in meters/second
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public float xtrack_error_GET()//Current crosstrack error on x-y plane in meters
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        static final Meta meta = new Meta(62, 1, 0, 0, 26, 208);
    }/**
*The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed,
*	 Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE:
*	 This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized
*	 for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset*/
    public static class GLOBAL_POSITION_INT_COV extends Pack
    {

        GLOBAL_POSITION_INT_COV() { super(meta, 0); }
        GLOBAL_POSITION_INT_COV(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (microseconds since system boot or since UNIX epoch)
        {  return (get_bytes(data,  0, 8)); }
        public int lat_GET()//Latitude, expressed as degrees * 1E7
        {  return (int)((int) get_bytes(data,  8, 4)); }
        public int lon_GET()//Longitude, expressed as degrees * 1E7
        {  return (int)((int) get_bytes(data,  12, 4)); }
        public int alt_GET()//Altitude in meters, expressed as * 1000 (millimeters), above MSL
        {  return (int)((int) get_bytes(data,  16, 4)); }
        public int relative_alt_GET()//Altitude above ground in meters, expressed as * 1000 (millimeters)
        {  return (int)((int) get_bytes(data,  20, 4)); }
        public float vx_GET()//Ground X Speed (Latitude), expressed as m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float vy_GET()//Ground Y Speed (Longitude), expressed as m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public float vz_GET()//Ground Z Speed (Altitude), expressed as m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public float[] covariance_GET(float[]  dst_ch, int pos)  //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
        {
            for(int BYTE = 36, dst_max = pos + 36; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] covariance_GET()//Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
        {return covariance_GET(new float[36], 0);} public @MAV_ESTIMATOR_TYPE int estimator_type_GET()//Class id of the estimator this estimate originated from.
        {  return  1 + (int)get_bits(data, 1440, 3); }
        static final Meta meta = new Meta(63, 0, 0, 1, 181, 1443);
    }/**
*The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
*	 Z-axis down (aeronautical frame, NED / north-east-down convention*/
    public static class LOCAL_POSITION_NED_COV extends Pack
    {

        LOCAL_POSITION_NED_COV() { super(meta, 0); }
        LOCAL_POSITION_NED_COV(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (microseconds since system boot or since UNIX epoch)
        {  return (get_bytes(data,  0, 8)); }
        public float x_GET()//X Position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float y_GET()//Y Position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float z_GET()//Z Position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float vx_GET()//X Speed (m/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float vy_GET()//Y Speed (m/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float vz_GET()//Z Speed (m/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public float ax_GET()//X Acceleration (m/s^2)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public float ay_GET()//Y Acceleration (m/s^2)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public float az_GET()//Z Acceleration (m/s^2)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        /**
        *Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
        *	 the second row, etc.*/
        public float[] covariance_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 44, dst_max = pos + 45; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        /**
        *Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
        *	 the second row, etc.*/
        public float[] covariance_GET()
        {return covariance_GET(new float[45], 0);} public @MAV_ESTIMATOR_TYPE int estimator_type_GET()//Class id of the estimator this estimate originated from.
        {  return  1 + (int)get_bits(data, 1792, 3); }
        static final Meta meta = new Meta(64, 0, 0, 1, 225, 1795);
    }/**
*The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
*	 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification*/
    public static class RC_CHANNELS extends Pack
    {

        RC_CHANNELS() { super(meta, 0); }
        RC_CHANNELS(int bytes) { super(meta, bytes); }
        public char chan1_raw_GET()//RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char chan2_raw_GET()//RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char chan3_raw_GET()//RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public char chan4_raw_GET()//RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  6, 2)); }
        public char chan5_raw_GET()//RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  8, 2)); }
        public char chan6_raw_GET()//RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  10, 2)); }
        public char chan7_raw_GET()//RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  12, 2)); }
        public char chan8_raw_GET()//RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  14, 2)); }
        public char chan9_raw_GET()//RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  16, 2)); }
        public char chan10_raw_GET()//RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  18, 2)); }
        public char chan11_raw_GET()//RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  20, 2)); }
        public char chan12_raw_GET()//RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  22, 2)); }
        public char chan13_raw_GET()//RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  24, 2)); }
        public char chan14_raw_GET()//RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  26, 2)); }
        public char chan15_raw_GET()//RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  28, 2)); }
        public char chan16_raw_GET()//RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  30, 2)); }
        public char chan17_raw_GET()//RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  32, 2)); }
        public char chan18_raw_GET()//RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        {  return (char)((char) get_bytes(data,  34, 2)); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  36, 4)); }
        /**
        *Total number of RC channels being received. This can be larger than 18, indicating that more channels
        *	 are available but not given in this message. This value should be 0 when no RC channels are available*/
        public char chancount_GET()
        {  return (char)((char) get_bytes(data,  40, 1)); }
        public char rssi_GET()//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
        {  return (char)((char) get_bytes(data,  41, 1)); }
        static final Meta meta = new Meta(65, 18, 1, 0, 42, 336);
    }/**
*THIS INTERFACE IS DEPRECATED. USE SET_MESSAGE_INTERVAL INSTEAD.*/
    public static class REQUEST_DATA_STREAM extends Pack
    {

        REQUEST_DATA_STREAM() { super(meta, 0); }
        REQUEST_DATA_STREAM(int bytes) { super(meta, bytes); }
        public char req_message_rate_GET()//The requested message rate
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_system_GET()//The target requested to send the message stream.
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_component_GET()//The target requested to send the message stream.
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char req_stream_id_GET()//The ID of the requested data stream
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char start_stop_GET()//1 to start sending, 0 to stop sending.
        {  return (char)((char) get_bytes(data,  5, 1)); }
        static final Meta meta = new Meta(66, 1, 0, 0, 6, 48);
    }/**
*THIS INTERFACE IS DEPRECATED. USE MESSAGE_INTERVAL INSTEAD.*/
    public static class DATA_STREAM extends Pack
    {

        DATA_STREAM() { super(meta, 0); }
        DATA_STREAM(int bytes) { super(meta, bytes); }
        public char message_rate_GET()//The message rate
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char stream_id_GET()//The ID of the requested data stream
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char on_off_GET()//1 stream is enabled, 0 stream is stopped.
        {  return (char)((char) get_bytes(data,  3, 1)); }
        static final Meta meta = new Meta(67, 1, 0, 0, 4, 32);
    }/**
*This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature,
*	 along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as
*	 boolean values of their*/
    public static class MANUAL_CONTROL extends Pack
    {

        MANUAL_CONTROL() { super(meta, 0); }
        MANUAL_CONTROL(int bytes) { super(meta, bytes); }
        /**
        *A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest
        *	 bit corresponds to Button 1*/
        public char buttons_GET()
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_GET()//The system to be controlled.
        {  return (char)((char) get_bytes(data,  2, 1)); }
        /**
        *X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
        *	 Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle*/
        public short x_GET()
        {  return (short)((short) get_bytes(data,  3, 2)); }
        /**
        *Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
        *	 Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle*/
        public short y_GET()
        {  return (short)((short) get_bytes(data,  5, 2)); }
        /**
        *Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
        *	 Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on
        *	 a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
        *	 thrust*/
        public short z_GET()
        {  return (short)((short) get_bytes(data,  7, 2)); }
        /**
        *R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
        *	 Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise
        *	 being -1000, and the yaw of a vehicle*/
        public short r_GET()
        {  return (short)((short) get_bytes(data,  9, 2)); }
        static final Meta meta = new Meta(69, 1, 0, 0, 11, 88);
    }/**
*The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value
*	 of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released
*	 back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds:
*	 100%. Individual receivers/transmitters might violate this specification*/
    public static class RC_CHANNELS_OVERRIDE extends Pack
    {

        RC_CHANNELS_OVERRIDE() { super(meta, 0); }
        RC_CHANNELS_OVERRIDE(int bytes) { super(meta, bytes); }
        public char chan1_raw_GET()//RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char chan2_raw_GET()//RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char chan3_raw_GET()//RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public char chan4_raw_GET()//RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        {  return (char)((char) get_bytes(data,  6, 2)); }
        public char chan5_raw_GET()//RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        {  return (char)((char) get_bytes(data,  8, 2)); }
        public char chan6_raw_GET()//RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        {  return (char)((char) get_bytes(data,  10, 2)); }
        public char chan7_raw_GET()//RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        {  return (char)((char) get_bytes(data,  12, 2)); }
        public char chan8_raw_GET()//RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        {  return (char)((char) get_bytes(data,  14, 2)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  16, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  17, 1)); }
        static final Meta meta = new Meta(70, 8, 0, 0, 18, 144);
    }/**
*Message encoding a mission item. This message is emitted to announce
*	 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See alsohttp:qgroundcontrol.org/mavlink/waypoint_protocol.*/
    public static class MISSION_ITEM_INT extends Pack
    {

        MISSION_ITEM_INT() { super(meta, 0); }
        MISSION_ITEM_INT(int bytes) { super(meta, bytes); }
        /**
        *Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the
        *	 sequence (0,1,2,3,4)*/
        public char seq_GET()
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public char current_GET()//false:0, true:1
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char autocontinue_GET()//autocontinue to next wp
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public float param1_GET()//PARAM1, see MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public float param2_GET()//PARAM2, see MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public float param3_GET()//PARAM3, see MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public float param4_GET()//PARAM4, see MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public int x_GET()//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
        {  return (int)((int) get_bytes(data,  22, 4)); }
        public int y_GET()//PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
        {  return (int)((int) get_bytes(data,  26, 4)); }
        public float z_GET()//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  30, 4))); }
        public @MAV_FRAME int frame_GET()//The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
        {  return  0 + (int)get_bits(data, 272, 4); }
        public @MAV_CMD int command_GET()//The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
        {  return  en__T((int)get_bits(data, 276, 8)); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYPE
        {  return  en__W((int)get_bits(data, 284, 3)); }
        static final Meta meta = new Meta(73, 1, 0, 0, 36, 287);
    }/**
*Metrics typically displayed on a HUD for fixed wing aircraft*/
    public static class VFR_HUD extends Pack
    {

        VFR_HUD() { super(meta, 0); }
        VFR_HUD(int bytes) { super(meta, bytes); }
        public char throttle_GET()//Current throttle setting in integer percent, 0 to 100
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public float airspeed_GET()//Current airspeed in m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  2, 4))); }
        public float groundspeed_GET()//Current ground speed in m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public short heading_GET()//Current heading in degrees, in compass units (0..360, 0=north)
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public float alt_GET()//Current altitude (MSL), in meters
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float climb_GET()//Current climb rate in meters/second
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        static final Meta meta = new Meta(74, 1, 0, 0, 20, 160);
    }/**
*Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value*/
    public static class COMMAND_INT extends Pack
    {

        COMMAND_INT() { super(meta, 0); }
        COMMAND_INT(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char current_GET()//false:0, true:1
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public char autocontinue_GET()//autocontinue to next wp
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public float param1_GET()//PARAM1, see MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float param2_GET()//PARAM2, see MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float param3_GET()//PARAM3, see MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float param4_GET()//PARAM4, see MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public int x_GET()//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
        {  return (int)((int) get_bytes(data,  20, 4)); }
        public int y_GET()//PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
        {  return (int)((int) get_bytes(data,  24, 4)); }
        public float z_GET()//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public @MAV_FRAME int frame_GET()//The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
        {  return  0 + (int)get_bits(data, 256, 4); }
        public @MAV_CMD int command_GET()//The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
        {  return  en__T((int)get_bits(data, 260, 8)); }
        static final Meta meta = new Meta(75, 0, 0, 0, 34, 268);
    }/**
*Send a command with up to seven parameters to the MAV*/
    public static class COMMAND_LONG extends Pack
    {

        COMMAND_LONG() { super(meta, 0); }
        COMMAND_LONG(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System which should execute the command
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public char target_component_GET()//Component which should execute the command, 0 for all components
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public char confirmation_GET()//0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public float param1_GET()//Parameter 1, as defined by MAV_CMD enum.
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  3, 4))); }
        public float param2_GET()//Parameter 2, as defined by MAV_CMD enum.
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  7, 4))); }
        public float param3_GET()//Parameter 3, as defined by MAV_CMD enum.
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  11, 4))); }
        public float param4_GET()//Parameter 4, as defined by MAV_CMD enum.
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  15, 4))); }
        public float param5_GET()//Parameter 5, as defined by MAV_CMD enum.
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  19, 4))); }
        public float param6_GET()//Parameter 6, as defined by MAV_CMD enum.
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  23, 4))); }
        public float param7_GET()//Parameter 7, as defined by MAV_CMD enum.
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  27, 4))); }
        public @MAV_CMD int command_GET()//Command ID, as defined by MAV_CMD enum.
        {  return  en__T((int)get_bits(data, 248, 8)); }
        static final Meta meta = new Meta(76, 0, 0, 0, 32, 256);
    }/**
*Report status of a command. Includes feedback whether the command was executed.*/
    public static class COMMAND_ACK extends Pack
    {

        COMMAND_ACK() { super(meta, 0); }
        COMMAND_ACK(int bytes) { super(meta, bytes); }
        public @MAV_CMD int command_GET()//Command ID, as defined by MAV_CMD enum.
        {  return  en__T((int)get_bits(data, 0, 8)); }
        public @MAV_RESULT int result_GET()//See MAV_RESULT enum
        {  return  0 + (int)get_bits(data, 8, 3); }
        /**
        *WIP: Also used as result_param1, it can be set with a enum containing the errors reasons of why the command
        *	 was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS*/
        public char  progress_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  11 && !try_visit_field(ph, 11)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 1));
        }
        /**
        *WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to
        *	 be denied*/
        public int  result_param2_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  12 && !try_visit_field(ph, 12)) return 0;
            return (int)((int) get_bytes(data,  ph.BYTE, 4));
        }
        public char  target_system_TRY(Bounds.Inside ph)//WIP: System which requested the command to be executed
        {
            if(ph.field_bit !=  13 && !try_visit_field(ph, 13)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 1));
        }
        public char  target_component_TRY(Bounds.Inside ph)//WIP: Component which requested the command to be executed
        {
            if(ph.field_bit !=  14 && !try_visit_field(ph, 14)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 1));
        }
        static final Meta meta = new Meta(77, 0, 0, 0, 3, 11, 0, _kP, _GP, _fP, _tP);
    }/**
*Setpoint in roll, pitch, yaw and thrust from the operator*/
    public static class MANUAL_SETPOINT extends Pack
    {

        MANUAL_SETPOINT() { super(meta, 0); }
        MANUAL_SETPOINT(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp in milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public float roll_GET()//Desired roll rate in radians per second
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float pitch_GET()//Desired pitch rate in radians per second
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float yaw_GET()//Desired yaw rate in radians per second
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float thrust_GET()//Collective thrust, normalized to 0 .. 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public char mode_switch_GET()//Flight mode switch position, 0.. 255
        {  return (char)((char) get_bytes(data,  20, 1)); }
        public char manual_override_switch_GET()//Override mode switch position, 0.. 255
        {  return (char)((char) get_bytes(data,  21, 1)); }
        static final Meta meta = new Meta(81, 0, 1, 0, 22, 176);
    }/**
*Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller
*	 or other system)*/
    public static class SET_ATTITUDE_TARGET extends Pack
    {

        SET_ATTITUDE_TARGET() { super(meta, 0); }
        SET_ATTITUDE_TARGET(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp in milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  5, 1)); }
        /**
        *Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
        *	 bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitud*/
        public char type_mask_GET()
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public float[] q_GET(float[]  dst_ch, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        {
            for(int BYTE = 7, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] q_GET()//Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        {return q_GET(new float[4], 0);} public float body_roll_rate_GET()//Body roll rate in radians per second
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  23, 4))); }
        public float body_pitch_rate_GET()//Body roll rate in radians per second
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  27, 4))); }
        public float body_yaw_rate_GET()//Body roll rate in radians per second
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  31, 4))); }
        public float thrust_GET()//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  35, 4))); }
        static final Meta meta = new Meta(82, 0, 1, 0, 39, 312);
    }/**
*Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match
*	 the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way*/
    public static class ATTITUDE_TARGET extends Pack
    {

        ATTITUDE_TARGET() { super(meta, 0); }
        ATTITUDE_TARGET(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp in milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        /**
        *Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
        *	 bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitud*/
        public char type_mask_GET()
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public float[] q_GET(float[]  dst_ch, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        {
            for(int BYTE = 5, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] q_GET()//Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        {return q_GET(new float[4], 0);} public float body_roll_rate_GET()//Body roll rate in radians per second
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  21, 4))); }
        public float body_pitch_rate_GET()//Body pitch rate in radians per second
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  25, 4))); }
        public float body_yaw_rate_GET()//Body yaw rate in radians per second
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  29, 4))); }
        public float thrust_GET()//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  33, 4))); }
        static final Meta meta = new Meta(83, 0, 1, 0, 37, 296);
    }/**
*Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller
*	 to command the vehicle (manual controller or other system)*/
    public static class SET_POSITION_TARGET_LOCAL_NED extends Pack
    {

        SET_POSITION_TARGET_LOCAL_NED() { super(meta, 0); }
        SET_POSITION_TARGET_LOCAL_NED(int bytes) { super(meta, bytes); }
        /**
        *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
        *	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
        *	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
        *	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
        *	 bit 11: yaw, bit 12: yaw rat*/
        public char type_mask_GET()
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public long time_boot_ms_GET()//Timestamp in milliseconds since system boot
        {  return (get_bytes(data,  2, 4)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  7, 1)); }
        public float x_GET()//X Position in NED frame in meters
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float y_GET()//Y Position in NED frame in meters
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float z_GET()//Z Position in NED frame in meters (note, altitude is negative in NED)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float vx_GET()//X velocity in NED frame in meter / s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float vy_GET()//Y velocity in NED frame in meter / s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float vz_GET()//Z velocity in NED frame in meter / s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public float afx_GET()//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public float afy_GET()//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public float afz_GET()//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public float yaw_GET()//yaw setpoint in rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public float yaw_rate_GET()//yaw rate setpoint in rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  48, 4))); }
        /**
        *Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
        *	 =*/
        public @MAV_FRAME int coordinate_frame_GET()
        {  return  0 + (int)get_bits(data, 416, 4); }
        static final Meta meta = new Meta(84, 1, 1, 0, 53, 420);
    }/**
*Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84).
*	 Used by an external controller to command the vehicle (manual controller or other system)*/
    public static class SET_POSITION_TARGET_GLOBAL_INT extends Pack
    {

        SET_POSITION_TARGET_GLOBAL_INT() { super(meta, 0); }
        SET_POSITION_TARGET_GLOBAL_INT(int bytes) { super(meta, bytes); }
        /**
        *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
        *	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
        *	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
        *	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
        *	 bit 11: yaw, bit 12: yaw rat*/
        public char type_mask_GET()
        {  return (char)((char) get_bytes(data,  0, 2)); }
        /**
        *Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
        *	 the system to compensate for the transport delay of the setpoint. This allows the system to compensate
        *	 processing latency*/
        public long time_boot_ms_GET()
        {  return (get_bytes(data,  2, 4)); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  7, 1)); }
        public int lat_int_GET()//X Position in WGS84 frame in 1e7 * meters
        {  return (int)((int) get_bytes(data,  8, 4)); }
        public int lon_int_GET()//Y Position in WGS84 frame in 1e7 * meters
        {  return (int)((int) get_bytes(data,  12, 4)); }
        public float alt_GET()//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float vx_GET()//X velocity in NED frame in meter / s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float vy_GET()//Y velocity in NED frame in meter / s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float vz_GET()//Z velocity in NED frame in meter / s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public float afx_GET()//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public float afy_GET()//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public float afz_GET()//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public float yaw_GET()//yaw setpoint in rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public float yaw_rate_GET()//yaw rate setpoint in rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  48, 4))); }
        /**
        *Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
        *	 = 1*/
        public @MAV_FRAME int coordinate_frame_GET()
        {  return  0 + (int)get_bits(data, 416, 4); }
        static final Meta meta = new Meta(86, 1, 1, 0, 53, 420);
    }/**
*Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
*	 This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled
*	 this way*/
    public static class POSITION_TARGET_GLOBAL_INT extends Pack
    {

        POSITION_TARGET_GLOBAL_INT() { super(meta, 0); }
        POSITION_TARGET_GLOBAL_INT(int bytes) { super(meta, bytes); }
        /**
        *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
        *	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
        *	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
        *	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
        *	 bit 11: yaw, bit 12: yaw rat*/
        public char type_mask_GET()
        {  return (char)((char) get_bytes(data,  0, 2)); }
        /**
        *Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
        *	 the system to compensate for the transport delay of the setpoint. This allows the system to compensate
        *	 processing latency*/
        public long time_boot_ms_GET()
        {  return (get_bytes(data,  2, 4)); }
        public int lat_int_GET()//X Position in WGS84 frame in 1e7 * meters
        {  return (int)((int) get_bytes(data,  6, 4)); }
        public int lon_int_GET()//Y Position in WGS84 frame in 1e7 * meters
        {  return (int)((int) get_bytes(data,  10, 4)); }
        public float alt_GET()//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public float vx_GET()//X velocity in NED frame in meter / s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public float vy_GET()//Y velocity in NED frame in meter / s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public float vz_GET()//Z velocity in NED frame in meter / s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  26, 4))); }
        public float afx_GET()//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  30, 4))); }
        public float afy_GET()//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  34, 4))); }
        public float afz_GET()//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  38, 4))); }
        public float yaw_GET()//yaw setpoint in rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  42, 4))); }
        public float yaw_rate_GET()//yaw rate setpoint in rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  46, 4))); }
        /**
        *Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
        *	 = 1*/
        public @MAV_FRAME int coordinate_frame_GET()
        {  return  0 + (int)get_bits(data, 400, 4); }
        static final Meta meta = new Meta(87, 1, 1, 0, 51, 404);
    }/**
*The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate
*	 frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down
*	 convention*/
    public static class LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET extends Pack
    {

        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET() { super(meta, 0); }
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public float x_GET()//X Position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public float y_GET()//Y Position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float z_GET()//Z Position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float roll_GET()//Roll
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float pitch_GET()//Pitch
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float yaw_GET()//Yaw
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        static final Meta meta = new Meta(89, 0, 1, 0, 28, 224);
    }/**
*DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please
*	 use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput
*	 applications such as hardware in the loop simulations*/
    public static class HIL_STATE extends Pack
    {

        HIL_STATE() { super(meta, 0); }
        HIL_STATE(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  return (get_bytes(data,  0, 8)); }
        public float roll_GET()//Roll angle (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float pitch_GET()//Pitch angle (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float yaw_GET()//Yaw angle (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float rollspeed_GET()//Body frame roll / phi angular speed (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float pitchspeed_GET()//Body frame pitch / theta angular speed (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float yawspeed_GET()//Body frame yaw / psi angular speed (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public int lat_GET()//Latitude, expressed as * 1E7
        {  return (int)((int) get_bytes(data,  32, 4)); }
        public int lon_GET()//Longitude, expressed as * 1E7
        {  return (int)((int) get_bytes(data,  36, 4)); }
        public int alt_GET()//Altitude in meters, expressed as * 1000 (millimeters)
        {  return (int)((int) get_bytes(data,  40, 4)); }
        public short vx_GET()//Ground X Speed (Latitude), expressed as m/s * 100
        {  return (short)((short) get_bytes(data,  44, 2)); }
        public short vy_GET()//Ground Y Speed (Longitude), expressed as m/s * 100
        {  return (short)((short) get_bytes(data,  46, 2)); }
        public short vz_GET()//Ground Z Speed (Altitude), expressed as m/s * 100
        {  return (short)((short) get_bytes(data,  48, 2)); }
        public short xacc_GET()//X acceleration (mg)
        {  return (short)((short) get_bytes(data,  50, 2)); }
        public short yacc_GET()//Y acceleration (mg)
        {  return (short)((short) get_bytes(data,  52, 2)); }
        public short zacc_GET()//Z acceleration (mg)
        {  return (short)((short) get_bytes(data,  54, 2)); }
        static final Meta meta = new Meta(90, 0, 0, 1, 56, 448);
    }/**
*Sent from autopilot to simulation. Hardware in the loop control outputs*/
    public static class HIL_CONTROLS extends Pack
    {

        HIL_CONTROLS() { super(meta, 0); }
        HIL_CONTROLS(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  return (get_bytes(data,  0, 8)); }
        public float roll_ailerons_GET()//Control output -1 .. 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float pitch_elevator_GET()//Control output -1 .. 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float yaw_rudder_GET()//Control output -1 .. 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float throttle_GET()//Throttle 0 .. 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float aux1_GET()//Aux 1, -1 .. 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float aux2_GET()//Aux 2, -1 .. 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public float aux3_GET()//Aux 3, -1 .. 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public float aux4_GET()//Aux 4, -1 .. 1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public char nav_mode_GET()//Navigation mode (MAV_NAV_MODE)
        {  return (char)((char) get_bytes(data,  40, 1)); }
        public @MAV_MODE int mode_GET()//System mode (MAV_MODE)
        {  return  en__P((int)get_bits(data, 328, 4)); }
        static final Meta meta = new Meta(91, 0, 0, 1, 42, 332);
    }/**
*Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation
*	 is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might
*	 violate this specification*/
    public static class HIL_RC_INPUTS_RAW extends Pack
    {

        HIL_RC_INPUTS_RAW() { super(meta, 0); }
        HIL_RC_INPUTS_RAW(int bytes) { super(meta, bytes); }
        public char chan1_raw_GET()//RC channel 1 value, in microseconds
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public char chan2_raw_GET()//RC channel 2 value, in microseconds
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public char chan3_raw_GET()//RC channel 3 value, in microseconds
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public char chan4_raw_GET()//RC channel 4 value, in microseconds
        {  return (char)((char) get_bytes(data,  6, 2)); }
        public char chan5_raw_GET()//RC channel 5 value, in microseconds
        {  return (char)((char) get_bytes(data,  8, 2)); }
        public char chan6_raw_GET()//RC channel 6 value, in microseconds
        {  return (char)((char) get_bytes(data,  10, 2)); }
        public char chan7_raw_GET()//RC channel 7 value, in microseconds
        {  return (char)((char) get_bytes(data,  12, 2)); }
        public char chan8_raw_GET()//RC channel 8 value, in microseconds
        {  return (char)((char) get_bytes(data,  14, 2)); }
        public char chan9_raw_GET()//RC channel 9 value, in microseconds
        {  return (char)((char) get_bytes(data,  16, 2)); }
        public char chan10_raw_GET()//RC channel 10 value, in microseconds
        {  return (char)((char) get_bytes(data,  18, 2)); }
        public char chan11_raw_GET()//RC channel 11 value, in microseconds
        {  return (char)((char) get_bytes(data,  20, 2)); }
        public char chan12_raw_GET()//RC channel 12 value, in microseconds
        {  return (char)((char) get_bytes(data,  22, 2)); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  return (get_bytes(data,  24, 8)); }
        public char rssi_GET()//Receive signal strength indicator, 0: 0%, 255: 100%
        {  return (char)((char) get_bytes(data,  32, 1)); }
        static final Meta meta = new Meta(92, 12, 0, 1, 33, 264);
    }/**
*Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS*/
    public static class HIL_ACTUATOR_CONTROLS extends Pack
    {

        HIL_ACTUATOR_CONTROLS() { super(meta, 0); }
        HIL_ACTUATOR_CONTROLS(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  return (get_bytes(data,  0, 8)); }
        public long flags_GET()//Flags as bitfield, reserved for future use.
        {  return (get_bytes(data,  8, 8)); }
        public float[] controls_GET(float[]  dst_ch, int pos)  //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
        {
            for(int BYTE = 16, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] controls_GET()//Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
        {return controls_GET(new float[16], 0);} public @MAV_MODE int mode_GET()//System mode (MAV_MODE), includes arming state.
        {  return  en__P((int)get_bits(data, 640, 4)); }
        static final Meta meta = new Meta(93, 0, 0, 2, 81, 644);
    }/**
*Optical flow from a flow sensor (e.g. optical mouse sensor)*/
    public static class OPTICAL_FLOW extends Pack
    {

        OPTICAL_FLOW() { super(meta, 0); }
        OPTICAL_FLOW(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (UNIX)
        {  return (get_bytes(data,  0, 8)); }
        public char sensor_id_GET()//Sensor ID
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public short flow_x_GET()//Flow in pixels * 10 in x-sensor direction (dezi-pixels)
        {  return (short)((short) get_bytes(data,  9, 2)); }
        public short flow_y_GET()//Flow in pixels * 10 in y-sensor direction (dezi-pixels)
        {  return (short)((short) get_bytes(data,  11, 2)); }
        public float flow_comp_m_x_GET()//Flow in meters in x-sensor direction, angular-speed compensated
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  13, 4))); }
        public float flow_comp_m_y_GET()//Flow in meters in y-sensor direction, angular-speed compensated
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  17, 4))); }
        public char quality_GET()//Optical flow quality / confidence. 0: bad, 255: maximum quality
        {  return (char)((char) get_bytes(data,  21, 1)); }
        public float ground_distance_GET()//Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public float  flow_rate_x_TRY(Bounds.Inside ph)//Flow rate in radians/second about X axis
        {
            if(ph.field_bit !=  208 && !try_visit_field(ph, 208)) return 0;
            return (float)(Float.intBitsToFloat((int) get_bytes(data,  ph.BYTE, 4)));
        }
        public float  flow_rate_y_TRY(Bounds.Inside ph)//Flow rate in radians/second about Y axis
        {
            if(ph.field_bit !=  209 && !try_visit_field(ph, 209)) return 0;
            return (float)(Float.intBitsToFloat((int) get_bytes(data,  ph.BYTE, 4)));
        }
        static final Meta meta = new Meta(100, 0, 0, 1, 27, 208, 0, _iH, _lH);
    } public static class GLOBAL_VISION_POSITION_ESTIMATE extends Pack
    {

        GLOBAL_VISION_POSITION_ESTIMATE() { super(meta, 0); }
        GLOBAL_VISION_POSITION_ESTIMATE(int bytes) { super(meta, bytes); }
        public long usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot)
        {  return (get_bytes(data,  0, 8)); }
        public float x_GET()//Global X position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float y_GET()//Global Y position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float z_GET()//Global Z position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float roll_GET()//Roll angle in rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float pitch_GET()//Pitch angle in rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float yaw_GET()//Yaw angle in rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        static final Meta meta = new Meta(101, 0, 0, 1, 32, 256);
    } public static class VISION_POSITION_ESTIMATE extends Pack
    {

        VISION_POSITION_ESTIMATE() { super(meta, 0); }
        VISION_POSITION_ESTIMATE(int bytes) { super(meta, bytes); }
        public long usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot)
        {  return (get_bytes(data,  0, 8)); }
        public float x_GET()//Global X position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public float y_GET()//Global Y position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public float z_GET()//Global Z position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public float roll_GET()//Roll angle in rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public float pitch_GET()//Pitch angle in rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public float yaw_GET()//Yaw angle in rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        static final Meta meta = new Meta(102, 0, 0, 1, 32, 256);
    } public static class VISION_SPEED_ESTIMATE extends Pack  implements CommunicationChannel.Sendable
    {

        VISION_SPEED_ESTIMATE() { super(meta, 0); }
        VISION_SPEED_ESTIMATE(int bytes) { super(meta, bytes); }
        public long usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot)
        {  return (get_bytes(data,  0, 8)); }
        public void usec_SET(long  src) //Timestamp (microseconds, synced to UNIX time or since system boot)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float x_GET()//Global X speed
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void x_SET(float  src) //Global X speed
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float y_GET()//Global Y speed
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void y_SET(float  src) //Global Y speed
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float z_GET()//Global Z speed
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void z_SET(float  src) //Global Z speed
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        static final Meta meta = new Meta(103, 0, 0, 1, 20, 160);
    } public static class VICON_POSITION_ESTIMATE extends Pack  implements CommunicationChannel.Sendable
    {

        VICON_POSITION_ESTIMATE() { super(meta, 0); }
        VICON_POSITION_ESTIMATE(int bytes) { super(meta, bytes); }
        public long usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot)
        {  return (get_bytes(data,  0, 8)); }
        public void usec_SET(long  src) //Timestamp (microseconds, synced to UNIX time or since system boot)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float x_GET()//Global X position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void x_SET(float  src) //Global X position
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float y_GET()//Global Y position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void y_SET(float  src) //Global Y position
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float z_GET()//Global Z position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void z_SET(float  src) //Global Z position
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float roll_GET()//Roll angle in rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void roll_SET(float  src) //Roll angle in rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float pitch_GET()//Pitch angle in rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void pitch_SET(float  src) //Pitch angle in rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float yaw_GET()//Yaw angle in rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void yaw_SET(float  src) //Yaw angle in rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        static final Meta meta = new Meta(104, 0, 0, 1, 32, 256);
    }/**
*The IMU readings in SI units in NED body frame*/
    public static class HIGHRES_IMU extends Pack  implements CommunicationChannel.Sendable
    {

        HIGHRES_IMU() { super(meta, 0); }
        HIGHRES_IMU(int bytes) { super(meta, bytes); }
        public char fields_updated_GET()//Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void fields_updated_SET(char  src) //Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long time_usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot)
        {  return (get_bytes(data,  2, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds, synced to UNIX time or since system boot)
        {  set_bytes((src) & -1L, 8, data,  2); }
        public float xacc_GET()//X acceleration (m/s^2)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public void xacc_SET(float  src) //X acceleration (m/s^2)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }
        public float yacc_GET()//Y acceleration (m/s^2)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public void yacc_SET(float  src) //Y acceleration (m/s^2)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public float zacc_GET()//Z acceleration (m/s^2)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public void zacc_SET(float  src) //Z acceleration (m/s^2)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public float xgyro_GET()//Angular speed around X axis (rad / sec)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public void xgyro_SET(float  src) //Angular speed around X axis (rad / sec)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
        public float ygyro_GET()//Angular speed around Y axis (rad / sec)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  26, 4))); }
        public void ygyro_SET(float  src) //Angular speed around Y axis (rad / sec)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 26); }
        public float zgyro_GET()//Angular speed around Z axis (rad / sec)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  30, 4))); }
        public void zgyro_SET(float  src) //Angular speed around Z axis (rad / sec)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 30); }
        public float xmag_GET()//X Magnetic field (Gauss)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  34, 4))); }
        public void xmag_SET(float  src) //X Magnetic field (Gauss)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 34); }
        public float ymag_GET()//Y Magnetic field (Gauss)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  38, 4))); }
        public void ymag_SET(float  src) //Y Magnetic field (Gauss)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 38); }
        public float zmag_GET()//Z Magnetic field (Gauss)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  42, 4))); }
        public void zmag_SET(float  src) //Z Magnetic field (Gauss)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 42); }
        public float abs_pressure_GET()//Absolute pressure in millibar
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  46, 4))); }
        public void abs_pressure_SET(float  src) //Absolute pressure in millibar
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 46); }
        public float diff_pressure_GET()//Differential pressure in millibar
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  50, 4))); }
        public void diff_pressure_SET(float  src) //Differential pressure in millibar
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 50); }
        public float pressure_alt_GET()//Altitude calculated from pressure
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  54, 4))); }
        public void pressure_alt_SET(float  src) //Altitude calculated from pressure
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 54); }
        public float temperature_GET()//Temperature in degrees celsius
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  58, 4))); }
        public void temperature_SET(float  src) //Temperature in degrees celsius
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 58); }
        static final Meta meta = new Meta(105, 1, 0, 1, 62, 496);
    }/**
*Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)*/
    public static class OPTICAL_FLOW_RAD extends Pack  implements CommunicationChannel.Sendable
    {

        OPTICAL_FLOW_RAD() { super(meta, 0); }
        OPTICAL_FLOW_RAD(int bytes) { super(meta, bytes); }
        /**
        *Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain
        *	 average flow. The integration time also indicates the*/
        public long integration_time_us_GET()
        {  return (get_bytes(data,  0, 4)); }
        /**
        *Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain
        *	 average flow. The integration time also indicates the*/
        public void integration_time_us_SET(long  src)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long time_delta_distance_us_GET()//Time in microseconds since the distance was sampled.
        {  return (get_bytes(data,  4, 4)); }
        public void time_delta_distance_us_SET(long  src) //Time in microseconds since the distance was sampled.
        {  set_bytes((src) & -1L, 4, data,  4); }
        public long time_usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot)
        {  return (get_bytes(data,  8, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds, synced to UNIX time or since system boot)
        {  set_bytes((src) & -1L, 8, data,  8); }
        public char sensor_id_GET()//Sensor ID
        {  return (char)((char) get_bytes(data,  16, 1)); }
        public void sensor_id_SET(char  src) //Sensor ID
        {  set_bytes((char)(src) & -1L, 1, data,  16); }
        /**
        *Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
        *	 motion along the positive Y axis induces a negative flow.*/
        public float integrated_x_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  17, 4))); }
        /**
        *Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
        *	 motion along the positive Y axis induces a negative flow.*/
        public void integrated_x_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 17); }
        /**
        *Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
        *	 motion along the positive X axis induces a positive flow.*/
        public float integrated_y_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  21, 4))); }
        /**
        *Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
        *	 motion along the positive X axis induces a positive flow.*/
        public void integrated_y_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 21); }
        public float integrated_xgyro_GET()//RH rotation around X axis (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  25, 4))); }
        public void integrated_xgyro_SET(float  src) //RH rotation around X axis (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 25); }
        public float integrated_ygyro_GET()//RH rotation around Y axis (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  29, 4))); }
        public void integrated_ygyro_SET(float  src) //RH rotation around Y axis (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 29); }
        public float integrated_zgyro_GET()//RH rotation around Z axis (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  33, 4))); }
        public void integrated_zgyro_SET(float  src) //RH rotation around Z axis (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 33); }
        public short temperature_GET()//Temperature * 100 in centi-degrees Celsius
        {  return (short)((short) get_bytes(data,  37, 2)); }
        public void temperature_SET(short  src) //Temperature * 100 in centi-degrees Celsius
        {  set_bytes((short)(src) & -1L, 2, data,  37); }
        public char quality_GET()//Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
        {  return (char)((char) get_bytes(data,  39, 1)); }
        public void quality_SET(char  src) //Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
        {  set_bytes((char)(src) & -1L, 1, data,  39); }
        /**
        *Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative
        *	 value: Unknown distance*/
        public float distance_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        /**
        *Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative
        *	 value: Unknown distance*/
        public void distance_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        static final Meta meta = new Meta(106, 0, 2, 1, 44, 352);
    }/**
*The IMU readings in SI units in NED body frame*/
    public static class HIL_SENSOR extends Pack  implements CommunicationChannel.Sendable
    {

        HIL_SENSOR() { super(meta, 0); }
        HIL_SENSOR(int bytes) { super(meta, bytes); }
        /**
        *Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full
        *	 reset of attitude/position/velocities/etc was performed in sim*/
        public long fields_updated_GET()
        {  return (get_bytes(data,  0, 4)); }
        /**
        *Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full
        *	 reset of attitude/position/velocities/etc was performed in sim*/
        public void fields_updated_SET(long  src)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long time_usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot)
        {  return (get_bytes(data,  4, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds, synced to UNIX time or since system boot)
        {  set_bytes((src) & -1L, 8, data,  4); }
        public float xacc_GET()//X acceleration (m/s^2)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void xacc_SET(float  src) //X acceleration (m/s^2)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float yacc_GET()//Y acceleration (m/s^2)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void yacc_SET(float  src) //Y acceleration (m/s^2)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float zacc_GET()//Z acceleration (m/s^2)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void zacc_SET(float  src) //Z acceleration (m/s^2)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float xgyro_GET()//Angular speed around X axis in body frame (rad / sec)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void xgyro_SET(float  src) //Angular speed around X axis in body frame (rad / sec)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float ygyro_GET()//Angular speed around Y axis in body frame (rad / sec)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void ygyro_SET(float  src) //Angular speed around Y axis in body frame (rad / sec)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float zgyro_GET()//Angular speed around Z axis in body frame (rad / sec)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void zgyro_SET(float  src) //Angular speed around Z axis in body frame (rad / sec)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float xmag_GET()//X Magnetic field (Gauss)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void xmag_SET(float  src) //X Magnetic field (Gauss)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public float ymag_GET()//Y Magnetic field (Gauss)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public void ymag_SET(float  src) //Y Magnetic field (Gauss)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public float zmag_GET()//Z Magnetic field (Gauss)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public void zmag_SET(float  src) //Z Magnetic field (Gauss)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }
        public float abs_pressure_GET()//Absolute pressure in millibar
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  48, 4))); }
        public void abs_pressure_SET(float  src) //Absolute pressure in millibar
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 48); }
        public float diff_pressure_GET()//Differential pressure (airspeed) in millibar
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  52, 4))); }
        public void diff_pressure_SET(float  src) //Differential pressure (airspeed) in millibar
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 52); }
        public float pressure_alt_GET()//Altitude calculated from pressure
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  56, 4))); }
        public void pressure_alt_SET(float  src) //Altitude calculated from pressure
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 56); }
        public float temperature_GET()//Temperature in degrees celsius
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  60, 4))); }
        public void temperature_SET(float  src) //Temperature in degrees celsius
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 60); }
        static final Meta meta = new Meta(107, 0, 1, 1, 64, 512);
    }/**
*Status of simulation environment, if used*/
    public static class SIM_STATE extends Pack  implements CommunicationChannel.Sendable
    {

        SIM_STATE() { super(meta, 0); }
        SIM_STATE(int bytes) { super(meta, bytes); }
        public float q1_GET()//True attitude quaternion component 1, w (1 in null-rotation)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public void q1_SET(float  src) //True attitude quaternion component 1, w (1 in null-rotation)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 0); }
        public float q2_GET()//True attitude quaternion component 2, x (0 in null-rotation)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void q2_SET(float  src) //True attitude quaternion component 2, x (0 in null-rotation)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float q3_GET()//True attitude quaternion component 3, y (0 in null-rotation)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void q3_SET(float  src) //True attitude quaternion component 3, y (0 in null-rotation)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float q4_GET()//True attitude quaternion component 4, z (0 in null-rotation)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void q4_SET(float  src) //True attitude quaternion component 4, z (0 in null-rotation)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float roll_GET()//Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void roll_SET(float  src) //Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float pitch_GET()//Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void pitch_SET(float  src) //Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float yaw_GET()//Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void yaw_SET(float  src) //Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float xacc_GET()//X acceleration m/s/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void xacc_SET(float  src) //X acceleration m/s/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float yacc_GET()//Y acceleration m/s/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void yacc_SET(float  src) //Y acceleration m/s/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float zacc_GET()//Z acceleration m/s/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void zacc_SET(float  src) //Z acceleration m/s/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public float xgyro_GET()//Angular speed around X axis rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public void xgyro_SET(float  src) //Angular speed around X axis rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public float ygyro_GET()//Angular speed around Y axis rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public void ygyro_SET(float  src) //Angular speed around Y axis rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }
        public float zgyro_GET()//Angular speed around Z axis rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  48, 4))); }
        public void zgyro_SET(float  src) //Angular speed around Z axis rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 48); }
        public float lat_GET()//Latitude in degrees
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  52, 4))); }
        public void lat_SET(float  src) //Latitude in degrees
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 52); }
        public float lon_GET()//Longitude in degrees
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  56, 4))); }
        public void lon_SET(float  src) //Longitude in degrees
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 56); }
        public float alt_GET()//Altitude in meters
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  60, 4))); }
        public void alt_SET(float  src) //Altitude in meters
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 60); }
        public float std_dev_horz_GET()//Horizontal position standard deviation
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  64, 4))); }
        public void std_dev_horz_SET(float  src) //Horizontal position standard deviation
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 64); }
        public float std_dev_vert_GET()//Vertical position standard deviation
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  68, 4))); }
        public void std_dev_vert_SET(float  src) //Vertical position standard deviation
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 68); }
        public float vn_GET()//True velocity in m/s in NORTH direction in earth-fixed NED frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  72, 4))); }
        public void vn_SET(float  src) //True velocity in m/s in NORTH direction in earth-fixed NED frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 72); }
        public float ve_GET()//True velocity in m/s in EAST direction in earth-fixed NED frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  76, 4))); }
        public void ve_SET(float  src) //True velocity in m/s in EAST direction in earth-fixed NED frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 76); }
        public float vd_GET()//True velocity in m/s in DOWN direction in earth-fixed NED frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  80, 4))); }
        public void vd_SET(float  src) //True velocity in m/s in DOWN direction in earth-fixed NED frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 80); }
        static final Meta meta = new Meta(108, 0, 0, 0, 84, 672);
    }/**
*Status generated by radio and injected into MAVLink stream.*/
    public static class RADIO_STATUS extends Pack  implements CommunicationChannel.Sendable
    {

        RADIO_STATUS() { super(meta, 0); }
        RADIO_STATUS(int bytes) { super(meta, bytes); }
        public char rxerrors_GET()//Receive errors
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void rxerrors_SET(char  src) //Receive errors
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char fixed__GET()//Count of error corrected packets
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void fixed__SET(char  src) //Count of error corrected packets
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char rssi_GET()//Local signal strength
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void rssi_SET(char  src) //Local signal strength
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public char remrssi_GET()//Remote signal strength
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public void remrssi_SET(char  src) //Remote signal strength
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public char txbuf_GET()//Remaining free buffer space in percent.
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public void txbuf_SET(char  src) //Remaining free buffer space in percent.
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public char noise_GET()//Background noise level
        {  return (char)((char) get_bytes(data,  7, 1)); }
        public void noise_SET(char  src) //Background noise level
        {  set_bytes((char)(src) & -1L, 1, data,  7); }
        public char remnoise_GET()//Remote background noise level
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public void remnoise_SET(char  src) //Remote background noise level
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        static final Meta meta = new Meta(109, 2, 0, 0, 9, 72);
    }/**
*File transfer message*/
    public static class FILE_TRANSFER_PROTOCOL extends Pack  implements CommunicationChannel.Sendable
    {

        FILE_TRANSFER_PROTOCOL() { super(meta, 0); }
        FILE_TRANSFER_PROTOCOL(int bytes) { super(meta, bytes); }
        public char target_network_GET()//Network ID (0 for broadcast)
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_network_SET(char  src) //Network ID (0 for broadcast)
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_system_GET()//System ID (0 for broadcast)
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_system_SET(char  src) //System ID (0 for broadcast)
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public char target_component_GET()//Component ID (0 for broadcast)
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void target_component_SET(char  src) //Component ID (0 for broadcast)
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        /**
        *Variable length payload. The length is defined by the remaining message length when subtracting the header
        *	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
        *	 message_type.  The particular encoding used can be extension specific and might not always be documented
        *	 as part of the mavlink specification*/
        public char[] payload_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 3, dst_max = pos + 251; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        /**
        *Variable length payload. The length is defined by the remaining message length when subtracting the header
        *	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
        *	 message_type.  The particular encoding used can be extension specific and might not always be documented
        *	 as part of the mavlink specification*/
        public char[] payload_GET()
        {return payload_GET(new char[251], 0);}/**
*Variable length payload. The length is defined by the remaining message length when subtracting the header
*	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
*	 message_type.  The particular encoding used can be extension specific and might not always be documented
*	 as part of the mavlink specification*/
        public void payload_SET(char[]  src, int pos)
        {
            for(int BYTE =  3, src_max = pos + 251; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(110, 0, 0, 0, 254, 2032);
    }/**
*Time synchronization message.*/
    public static class TIMESYNC extends Pack  implements CommunicationChannel.Sendable
    {

        TIMESYNC() { super(meta, 0); }
        TIMESYNC(int bytes) { super(meta, bytes); }
        public long tc1_GET()//Time sync timestamp 1
        {  return (get_bytes(data,  0, 8)); }
        public void tc1_SET(long  src) //Time sync timestamp 1
        {  set_bytes((src) & -1L, 8, data,  0); }
        public long ts1_GET()//Time sync timestamp 2
        {  return (get_bytes(data,  8, 8)); }
        public void ts1_SET(long  src) //Time sync timestamp 2
        {  set_bytes((src) & -1L, 8, data,  8); }
        static final Meta meta = new Meta(111, 0, 0, 0, 16, 128);
    }/**
*Camera-IMU triggering and synchronisation message.*/
    public static class CAMERA_TRIGGER extends Pack  implements CommunicationChannel.Sendable
    {

        CAMERA_TRIGGER() { super(meta, 0); }
        CAMERA_TRIGGER(int bytes) { super(meta, bytes); }
        public long seq_GET()//Image frame sequence
        {  return (get_bytes(data,  0, 4)); }
        public void seq_SET(long  src) //Image frame sequence
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long time_usec_GET()//Timestamp for the image frame in microseconds
        {  return (get_bytes(data,  4, 8)); }
        public void time_usec_SET(long  src) //Timestamp for the image frame in microseconds
        {  set_bytes((src) & -1L, 8, data,  4); }
        static final Meta meta = new Meta(112, 0, 1, 1, 12, 96);
    }/**
*The global position, as returned by the Global Positioning System (GPS). This is
*	 NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
    public static class HIL_GPS extends Pack  implements CommunicationChannel.Sendable
    {

        HIL_GPS() { super(meta, 0); }
        HIL_GPS(int bytes) { super(meta, bytes); }
        public char eph_GET()//GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void eph_SET(char  src) //GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char epv_GET()//GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void epv_SET(char  src) //GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char vel_GET()//GPS ground speed in cm/s. If unknown, set to: 65535
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void vel_SET(char  src) //GPS ground speed in cm/s. If unknown, set to: 65535
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        /**
        *Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
        *	 unknown, set to: 6553*/
        public char cog_GET()
        {  return (char)((char) get_bytes(data,  6, 2)); }
        /**
        *Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
        *	 unknown, set to: 6553*/
        public void cog_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  return (get_bytes(data,  8, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  set_bytes((src) & -1L, 8, data,  8); }
        /**
        *0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is
        *	 at least two, so always correctly fill in the fix*/
        public char fix_type_GET()
        {  return (char)((char) get_bytes(data,  16, 1)); }
        /**
        *0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is
        *	 at least two, so always correctly fill in the fix*/
        public void fix_type_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  16); }
        public int lat_GET()//Latitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  17, 4)); }
        public void lat_SET(int  src) //Latitude (WGS84), in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  17); }
        public int lon_GET()//Longitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  21, 4)); }
        public void lon_SET(int  src) //Longitude (WGS84), in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  21); }
        public int alt_GET()//Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
        {  return (int)((int) get_bytes(data,  25, 4)); }
        public void alt_SET(int  src) //Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
        {  set_bytes((int)(src) & -1L, 4, data,  25); }
        public short vn_GET()//GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
        {  return (short)((short) get_bytes(data,  29, 2)); }
        public void vn_SET(short  src) //GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
        {  set_bytes((short)(src) & -1L, 2, data,  29); }
        public short ve_GET()//GPS velocity in cm/s in EAST direction in earth-fixed NED frame
        {  return (short)((short) get_bytes(data,  31, 2)); }
        public void ve_SET(short  src) //GPS velocity in cm/s in EAST direction in earth-fixed NED frame
        {  set_bytes((short)(src) & -1L, 2, data,  31); }
        public short vd_GET()//GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
        {  return (short)((short) get_bytes(data,  33, 2)); }
        public void vd_SET(short  src) //GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
        {  set_bytes((short)(src) & -1L, 2, data,  33); }
        public char satellites_visible_GET()//Number of satellites visible. If unknown, set to 255
        {  return (char)((char) get_bytes(data,  35, 1)); }
        public void satellites_visible_SET(char  src) //Number of satellites visible. If unknown, set to 255
        {  set_bytes((char)(src) & -1L, 1, data,  35); }
        static final Meta meta = new Meta(113, 4, 0, 1, 36, 288);
    }/**
*Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)*/
    public static class HIL_OPTICAL_FLOW extends Pack  implements CommunicationChannel.Sendable
    {

        HIL_OPTICAL_FLOW() { super(meta, 0); }
        HIL_OPTICAL_FLOW(int bytes) { super(meta, bytes); }
        /**
        *Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain
        *	 average flow. The integration time also indicates the*/
        public long integration_time_us_GET()
        {  return (get_bytes(data,  0, 4)); }
        /**
        *Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain
        *	 average flow. The integration time also indicates the*/
        public void integration_time_us_SET(long  src)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long time_delta_distance_us_GET()//Time in microseconds since the distance was sampled.
        {  return (get_bytes(data,  4, 4)); }
        public void time_delta_distance_us_SET(long  src) //Time in microseconds since the distance was sampled.
        {  set_bytes((src) & -1L, 4, data,  4); }
        public long time_usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot)
        {  return (get_bytes(data,  8, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds, synced to UNIX time or since system boot)
        {  set_bytes((src) & -1L, 8, data,  8); }
        public char sensor_id_GET()//Sensor ID
        {  return (char)((char) get_bytes(data,  16, 1)); }
        public void sensor_id_SET(char  src) //Sensor ID
        {  set_bytes((char)(src) & -1L, 1, data,  16); }
        /**
        *Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
        *	 motion along the positive Y axis induces a negative flow.*/
        public float integrated_x_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  17, 4))); }
        /**
        *Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
        *	 motion along the positive Y axis induces a negative flow.*/
        public void integrated_x_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 17); }
        /**
        *Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
        *	 motion along the positive X axis induces a positive flow.*/
        public float integrated_y_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  21, 4))); }
        /**
        *Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
        *	 motion along the positive X axis induces a positive flow.*/
        public void integrated_y_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 21); }
        public float integrated_xgyro_GET()//RH rotation around X axis (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  25, 4))); }
        public void integrated_xgyro_SET(float  src) //RH rotation around X axis (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 25); }
        public float integrated_ygyro_GET()//RH rotation around Y axis (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  29, 4))); }
        public void integrated_ygyro_SET(float  src) //RH rotation around Y axis (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 29); }
        public float integrated_zgyro_GET()//RH rotation around Z axis (rad)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  33, 4))); }
        public void integrated_zgyro_SET(float  src) //RH rotation around Z axis (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 33); }
        public short temperature_GET()//Temperature * 100 in centi-degrees Celsius
        {  return (short)((short) get_bytes(data,  37, 2)); }
        public void temperature_SET(short  src) //Temperature * 100 in centi-degrees Celsius
        {  set_bytes((short)(src) & -1L, 2, data,  37); }
        public char quality_GET()//Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
        {  return (char)((char) get_bytes(data,  39, 1)); }
        public void quality_SET(char  src) //Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
        {  set_bytes((char)(src) & -1L, 1, data,  39); }
        /**
        *Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative
        *	 value: Unknown distance*/
        public float distance_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        /**
        *Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative
        *	 value: Unknown distance*/
        public void distance_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        static final Meta meta = new Meta(114, 0, 2, 1, 44, 352);
    }/**
*Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful
*	 for high throughput applications such as hardware in the loop simulations*/
    public static class HIL_STATE_QUATERNION extends Pack  implements CommunicationChannel.Sendable
    {

        HIL_STATE_QUATERNION() { super(meta, 0); }
        HIL_STATE_QUATERNION(int bytes) { super(meta, bytes); }
        public char ind_airspeed_GET()//Indicated airspeed, expressed as cm/s
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void ind_airspeed_SET(char  src) //Indicated airspeed, expressed as cm/s
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char true_airspeed_GET()//True airspeed, expressed as cm/s
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void true_airspeed_SET(char  src) //True airspeed, expressed as cm/s
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  return (get_bytes(data,  4, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  set_bytes((src) & -1L, 8, data,  4); }
        public float[] attitude_quaternion_GET(float[]  dst_ch, int pos)  //Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation
        {
            for(int BYTE = 12, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] attitude_quaternion_GET()//Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation
        {return attitude_quaternion_GET(new float[4], 0);} public void attitude_quaternion_SET(float[]  src, int pos)  //Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation
        {
            for(int BYTE =  12, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float rollspeed_GET()//Body frame roll / phi angular speed (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void rollspeed_SET(float  src) //Body frame roll / phi angular speed (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float pitchspeed_GET()//Body frame pitch / theta angular speed (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void pitchspeed_SET(float  src) //Body frame pitch / theta angular speed (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float yawspeed_GET()//Body frame yaw / psi angular speed (rad/s)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void yawspeed_SET(float  src) //Body frame yaw / psi angular speed (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public int lat_GET()//Latitude, expressed as * 1E7
        {  return (int)((int) get_bytes(data,  40, 4)); }
        public void lat_SET(int  src) //Latitude, expressed as * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  40); }
        public int lon_GET()//Longitude, expressed as * 1E7
        {  return (int)((int) get_bytes(data,  44, 4)); }
        public void lon_SET(int  src) //Longitude, expressed as * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  44); }
        public int alt_GET()//Altitude in meters, expressed as * 1000 (millimeters)
        {  return (int)((int) get_bytes(data,  48, 4)); }
        public void alt_SET(int  src) //Altitude in meters, expressed as * 1000 (millimeters)
        {  set_bytes((int)(src) & -1L, 4, data,  48); }
        public short vx_GET()//Ground X Speed (Latitude), expressed as cm/s
        {  return (short)((short) get_bytes(data,  52, 2)); }
        public void vx_SET(short  src) //Ground X Speed (Latitude), expressed as cm/s
        {  set_bytes((short)(src) & -1L, 2, data,  52); }
        public short vy_GET()//Ground Y Speed (Longitude), expressed as cm/s
        {  return (short)((short) get_bytes(data,  54, 2)); }
        public void vy_SET(short  src) //Ground Y Speed (Longitude), expressed as cm/s
        {  set_bytes((short)(src) & -1L, 2, data,  54); }
        public short vz_GET()//Ground Z Speed (Altitude), expressed as cm/s
        {  return (short)((short) get_bytes(data,  56, 2)); }
        public void vz_SET(short  src) //Ground Z Speed (Altitude), expressed as cm/s
        {  set_bytes((short)(src) & -1L, 2, data,  56); }
        public short xacc_GET()//X acceleration (mg)
        {  return (short)((short) get_bytes(data,  58, 2)); }
        public void xacc_SET(short  src) //X acceleration (mg)
        {  set_bytes((short)(src) & -1L, 2, data,  58); }
        public short yacc_GET()//Y acceleration (mg)
        {  return (short)((short) get_bytes(data,  60, 2)); }
        public void yacc_SET(short  src) //Y acceleration (mg)
        {  set_bytes((short)(src) & -1L, 2, data,  60); }
        public short zacc_GET()//Z acceleration (mg)
        {  return (short)((short) get_bytes(data,  62, 2)); }
        public void zacc_SET(short  src) //Z acceleration (mg)
        {  set_bytes((short)(src) & -1L, 2, data,  62); }
        static final Meta meta = new Meta(115, 2, 0, 1, 64, 512);
    }/**
*The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to
*	 the described unit*/
    public static class SCALED_IMU2 extends Pack  implements CommunicationChannel.Sendable
    {

        SCALED_IMU2() { super(meta, 0); }
        SCALED_IMU2(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public short xacc_GET()//X acceleration (mg)
        {  return (short)((short) get_bytes(data,  4, 2)); }
        public void xacc_SET(short  src) //X acceleration (mg)
        {  set_bytes((short)(src) & -1L, 2, data,  4); }
        public short yacc_GET()//Y acceleration (mg)
        {  return (short)((short) get_bytes(data,  6, 2)); }
        public void yacc_SET(short  src) //Y acceleration (mg)
        {  set_bytes((short)(src) & -1L, 2, data,  6); }
        public short zacc_GET()//Z acceleration (mg)
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public void zacc_SET(short  src) //Z acceleration (mg)
        {  set_bytes((short)(src) & -1L, 2, data,  8); }
        public short xgyro_GET()//Angular speed around X axis (millirad /sec)
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public void xgyro_SET(short  src) //Angular speed around X axis (millirad /sec)
        {  set_bytes((short)(src) & -1L, 2, data,  10); }
        public short ygyro_GET()//Angular speed around Y axis (millirad /sec)
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public void ygyro_SET(short  src) //Angular speed around Y axis (millirad /sec)
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        public short zgyro_GET()//Angular speed around Z axis (millirad /sec)
        {  return (short)((short) get_bytes(data,  14, 2)); }
        public void zgyro_SET(short  src) //Angular speed around Z axis (millirad /sec)
        {  set_bytes((short)(src) & -1L, 2, data,  14); }
        public short xmag_GET()//X Magnetic field (milli tesla)
        {  return (short)((short) get_bytes(data,  16, 2)); }
        public void xmag_SET(short  src) //X Magnetic field (milli tesla)
        {  set_bytes((short)(src) & -1L, 2, data,  16); }
        public short ymag_GET()//Y Magnetic field (milli tesla)
        {  return (short)((short) get_bytes(data,  18, 2)); }
        public void ymag_SET(short  src) //Y Magnetic field (milli tesla)
        {  set_bytes((short)(src) & -1L, 2, data,  18); }
        public short zmag_GET()//Z Magnetic field (milli tesla)
        {  return (short)((short) get_bytes(data,  20, 2)); }
        public void zmag_SET(short  src) //Z Magnetic field (milli tesla)
        {  set_bytes((short)(src) & -1L, 2, data,  20); }
        static final Meta meta = new Meta(116, 0, 1, 0, 22, 176);
    }/**
*Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END
*	 is called*/
    public static class LOG_REQUEST_LIST extends Pack  implements CommunicationChannel.Sendable
    {

        LOG_REQUEST_LIST() { super(meta, 0); }
        LOG_REQUEST_LIST(int bytes) { super(meta, bytes); }
        public char start_GET()//First log id (0 for first available)
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void start_SET(char  src) //First log id (0 for first available)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char end_GET()//Last log id (0xffff for last available)
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void end_SET(char  src) //Last log id (0xffff for last available)
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        static final Meta meta = new Meta(117, 2, 0, 0, 6, 48);
    }/**
*Reply to LOG_REQUEST_LIST*/
    public static class LOG_ENTRY extends Pack  implements CommunicationChannel.Sendable
    {

        LOG_ENTRY() { super(meta, 0); }
        LOG_ENTRY(int bytes) { super(meta, bytes); }
        public char id_GET()//Log id
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void id_SET(char  src) //Log id
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char num_logs_GET()//Total number of logs
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void num_logs_SET(char  src) //Total number of logs
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char last_log_num_GET()//High log number
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void last_log_num_SET(char  src) //High log number
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public long time_utc_GET()//UTC timestamp of log in seconds since 1970, or 0 if not available
        {  return (get_bytes(data,  6, 4)); }
        public void time_utc_SET(long  src) //UTC timestamp of log in seconds since 1970, or 0 if not available
        {  set_bytes((src) & -1L, 4, data,  6); }
        public long size_GET()//Size of the log (may be approximate) in bytes
        {  return (get_bytes(data,  10, 4)); }
        public void size_SET(long  src) //Size of the log (may be approximate) in bytes
        {  set_bytes((src) & -1L, 4, data,  10); }
        static final Meta meta = new Meta(118, 3, 2, 0, 14, 112);
    }/**
*Request a chunk of a log*/
    public static class LOG_REQUEST_DATA extends Pack  implements CommunicationChannel.Sendable
    {

        LOG_REQUEST_DATA() { super(meta, 0); }
        LOG_REQUEST_DATA(int bytes) { super(meta, bytes); }
        public char id_GET()//Log id (from LOG_ENTRY reply)
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void id_SET(char  src) //Log id (from LOG_ENTRY reply)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long ofs_GET()//Offset into the log
        {  return (get_bytes(data,  2, 4)); }
        public void ofs_SET(long  src) //Offset into the log
        {  set_bytes((src) & -1L, 4, data,  2); }
        public long count_GET()//Number of bytes
        {  return (get_bytes(data,  6, 4)); }
        public void count_SET(long  src) //Number of bytes
        {  set_bytes((src) & -1L, 4, data,  6); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  11, 1)); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  11); }
        static final Meta meta = new Meta(119, 1, 2, 0, 12, 96);
    }/**
*Reply to LOG_REQUEST_DATA*/
    public static class LOG_DATA extends Pack  implements CommunicationChannel.Sendable
    {

        LOG_DATA() { super(meta, 0); }
        LOG_DATA(int bytes) { super(meta, bytes); }
        public char id_GET()//Log id (from LOG_ENTRY reply)
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void id_SET(char  src) //Log id (from LOG_ENTRY reply)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long ofs_GET()//Offset into the log
        {  return (get_bytes(data,  2, 4)); }
        public void ofs_SET(long  src) //Offset into the log
        {  set_bytes((src) & -1L, 4, data,  2); }
        public char count_GET()//Number of bytes (zero for end of log)
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public void count_SET(char  src) //Number of bytes (zero for end of log)
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public char[] data__GET(char[]  dst_ch, int pos)  //log data
        {
            for(int BYTE = 7, dst_max = pos + 90; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//log data
        {return data__GET(new char[90], 0);} public void data__SET(char[]  src, int pos)  //log data
        {
            for(int BYTE =  7, src_max = pos + 90; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(120, 1, 1, 0, 97, 776);
    }/**
*Erase all logs*/
    public static class LOG_ERASE extends Pack  implements CommunicationChannel.Sendable
    {

        LOG_ERASE() { super(meta, 0); }
        LOG_ERASE(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        static final Meta meta = new Meta(121, 0, 0, 0, 2, 16);
    }/**
*Stop log transfer and resume normal logging*/
    public static class LOG_REQUEST_END extends Pack  implements CommunicationChannel.Sendable
    {

        LOG_REQUEST_END() { super(meta, 0); }
        LOG_REQUEST_END(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        static final Meta meta = new Meta(122, 0, 0, 0, 2, 16);
    }/**
*data for injecting into the onboard GPS (used for DGPS)*/
    public static class GPS_INJECT_DATA extends Pack  implements CommunicationChannel.Sendable
    {

        GPS_INJECT_DATA() { super(meta, 0); }
        GPS_INJECT_DATA(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public char len_GET()//data length
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void len_SET(char  src) //data length
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char[] data__GET(char[]  dst_ch, int pos)  //raw data (110 is enough for 12 satellites of RTCMv2)
        {
            for(int BYTE = 3, dst_max = pos + 110; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//raw data (110 is enough for 12 satellites of RTCMv2)
        {return data__GET(new char[110], 0);} public void data__SET(char[]  src, int pos)  //raw data (110 is enough for 12 satellites of RTCMv2)
        {
            for(int BYTE =  3, src_max = pos + 110; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(123, 0, 0, 0, 113, 904);
    }/**
*Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
    public static class GPS2_RAW extends Pack  implements CommunicationChannel.Sendable
    {

        GPS2_RAW() { super(meta, 0); }
        GPS2_RAW(int bytes) { super(meta, bytes); }
        public char eph_GET()//GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void eph_SET(char  src) //GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char epv_GET()//GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void epv_SET(char  src) //GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char vel_GET()//GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void vel_SET(char  src) //GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        /**
        *Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
        *	 unknown, set to: UINT16_MA*/
        public char cog_GET()
        {  return (char)((char) get_bytes(data,  6, 2)); }
        /**
        *Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
        *	 unknown, set to: UINT16_MA*/
        public void cog_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public long dgps_age_GET()//Age of DGPS info
        {  return (get_bytes(data,  8, 4)); }
        public void dgps_age_SET(long  src) //Age of DGPS info
        {  set_bytes((src) & -1L, 4, data,  8); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  return (get_bytes(data,  12, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  set_bytes((src) & -1L, 8, data,  12); }
        public int lat_GET()//Latitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  20, 4)); }
        public void lat_SET(int  src) //Latitude (WGS84), in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  20); }
        public int lon_GET()//Longitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  24, 4)); }
        public void lon_SET(int  src) //Longitude (WGS84), in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  24); }
        public int alt_GET()//Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
        {  return (int)((int) get_bytes(data,  28, 4)); }
        public void alt_SET(int  src) //Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
        {  set_bytes((int)(src) & -1L, 4, data,  28); }
        public char satellites_visible_GET()//Number of satellites visible. If unknown, set to 255
        {  return (char)((char) get_bytes(data,  32, 1)); }
        public void satellites_visible_SET(char  src) //Number of satellites visible. If unknown, set to 255
        {  set_bytes((char)(src) & -1L, 1, data,  32); }
        public char dgps_numch_GET()//Number of DGPS satellites
        {  return (char)((char) get_bytes(data,  33, 1)); }
        public void dgps_numch_SET(char  src) //Number of DGPS satellites
        {  set_bytes((char)(src) & -1L, 1, data,  33); }
        public @GPS_FIX_TYPE int fix_type_GET()//See the GPS_FIX_TYPE enum.
        {  return  0 + (int)get_bits(data, 272, 4); }
        public void fix_type_SET(@GPS_FIX_TYPE int  src) //See the GPS_FIX_TYPE enum.
        {  set_bits(- 0 +   src, 4, data, 272); }
        static final Meta meta = new Meta(124, 4, 1, 1, 35, 276);
    }/**
*Power supply status*/
    public static class POWER_STATUS extends Pack  implements CommunicationChannel.Sendable
    {

        POWER_STATUS() { super(meta, 0); }
        POWER_STATUS(int bytes) { super(meta, bytes); }
        public char Vcc_GET()//5V rail voltage in millivolts
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void Vcc_SET(char  src) //5V rail voltage in millivolts
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char Vservo_GET()//servo rail voltage in millivolts
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void Vservo_SET(char  src) //servo rail voltage in millivolts
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public @MAV_POWER_STATUS int flags_GET()//power supply status flags (see MAV_POWER_STATUS enum)
        {  return  1 + (int)get_bits(data, 32, 6); }
        public void flags_SET(@MAV_POWER_STATUS int  src) //power supply status flags (see MAV_POWER_STATUS enum)
        {  set_bits(- 1 +   src, 6, data, 32); }
        static final Meta meta = new Meta(125, 2, 0, 0, 5, 38);
    }/**
*Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or
*	 telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages
*	 or change the devices settings. A message with zero bytes can be used to change just the baudrate*/
    public static class SERIAL_CONTROL extends Pack  implements CommunicationChannel.Sendable
    {

        SERIAL_CONTROL() { super(meta, 0); }
        SERIAL_CONTROL(int bytes) { super(meta, bytes); }
        public char timeout_GET()//Timeout for reply data in milliseconds
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void timeout_SET(char  src) //Timeout for reply data in milliseconds
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long baudrate_GET()//Baudrate of transfer. Zero means no change.
        {  return (get_bytes(data,  2, 4)); }
        public void baudrate_SET(long  src) //Baudrate of transfer. Zero means no change.
        {  set_bytes((src) & -1L, 4, data,  2); }
        public char count_GET()//how many bytes in this transfer
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public void count_SET(char  src) //how many bytes in this transfer
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public char[] data__GET(char[]  dst_ch, int pos)  //serial data
        {
            for(int BYTE = 7, dst_max = pos + 70; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//serial data
        {return data__GET(new char[70], 0);} public void data__SET(char[]  src, int pos)  //serial data
        {
            for(int BYTE =  7, src_max = pos + 70; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public @SERIAL_CONTROL_DEV int device_GET()//See SERIAL_CONTROL_DEV enum
        {
            switch((int)get_bits(data, 616, 3))
            {
                case 0:
                    return SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1;
                case 1:
                    return SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2;
                case 2:
                    return SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
                case 3:
                    return SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2;
                case 4:
                    return SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL;
            }
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void device_SET(@SERIAL_CONTROL_DEV int  src) //See SERIAL_CONTROL_DEV enum
        {
            long id = 0;
            switch(src)
            {
                case SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1:
                    id = 0;
                    break;
                case SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2:
                    id = 1;
                    break;
                case SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1:
                    id = 2;
                    break;
                case SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2:
                    id = 3;
                    break;
                case SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL:
                    id = 4;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 616);
        }
        public @SERIAL_CONTROL_FLAG int flags_GET()//See SERIAL_CONTROL_FLAG enum
        {  return  1 + (int)get_bits(data, 619, 5); }
        public void flags_SET(@SERIAL_CONTROL_FLAG int  src) //See SERIAL_CONTROL_FLAG enum
        {  set_bits(- 1 +   src, 5, data, 619); }
        static final Meta meta = new Meta(126, 1, 1, 0, 78, 624);
    }/**
*RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting*/
    public static class GPS_RTK extends Pack  implements CommunicationChannel.Sendable
    {

        GPS_RTK() { super(meta, 0); }
        GPS_RTK(int bytes) { super(meta, bytes); }
        public char wn_GET()//GPS Week Number of last baseline
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void wn_SET(char  src) //GPS Week Number of last baseline
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long time_last_baseline_ms_GET()//Time since boot of last baseline message received in ms.
        {  return (get_bytes(data,  2, 4)); }
        public void time_last_baseline_ms_SET(long  src) //Time since boot of last baseline message received in ms.
        {  set_bytes((src) & -1L, 4, data,  2); }
        public long tow_GET()//GPS Time of Week of last baseline
        {  return (get_bytes(data,  6, 4)); }
        public void tow_SET(long  src) //GPS Time of Week of last baseline
        {  set_bytes((src) & -1L, 4, data,  6); }
        public long accuracy_GET()//Current estimate of baseline accuracy.
        {  return (get_bytes(data,  10, 4)); }
        public void accuracy_SET(long  src) //Current estimate of baseline accuracy.
        {  set_bytes((src) & -1L, 4, data,  10); }
        public char rtk_receiver_id_GET()//Identification of connected RTK receiver.
        {  return (char)((char) get_bytes(data,  14, 1)); }
        public void rtk_receiver_id_SET(char  src) //Identification of connected RTK receiver.
        {  set_bytes((char)(src) & -1L, 1, data,  14); }
        public char rtk_health_GET()//GPS-specific health report for RTK data.
        {  return (char)((char) get_bytes(data,  15, 1)); }
        public void rtk_health_SET(char  src) //GPS-specific health report for RTK data.
        {  set_bytes((char)(src) & -1L, 1, data,  15); }
        public char rtk_rate_GET()//Rate of baseline messages being received by GPS, in HZ
        {  return (char)((char) get_bytes(data,  16, 1)); }
        public void rtk_rate_SET(char  src) //Rate of baseline messages being received by GPS, in HZ
        {  set_bytes((char)(src) & -1L, 1, data,  16); }
        public char nsats_GET()//Current number of sats used for RTK calculation.
        {  return (char)((char) get_bytes(data,  17, 1)); }
        public void nsats_SET(char  src) //Current number of sats used for RTK calculation.
        {  set_bytes((char)(src) & -1L, 1, data,  17); }
        public char baseline_coords_type_GET()//Coordinate system of baseline. 0 == ECEF, 1 == NED
        {  return (char)((char) get_bytes(data,  18, 1)); }
        public void baseline_coords_type_SET(char  src) //Coordinate system of baseline. 0 == ECEF, 1 == NED
        {  set_bytes((char)(src) & -1L, 1, data,  18); }
        public int baseline_a_mm_GET()//Current baseline in ECEF x or NED north component in mm.
        {  return (int)((int) get_bytes(data,  19, 4)); }
        public void baseline_a_mm_SET(int  src) //Current baseline in ECEF x or NED north component in mm.
        {  set_bytes((int)(src) & -1L, 4, data,  19); }
        public int baseline_b_mm_GET()//Current baseline in ECEF y or NED east component in mm.
        {  return (int)((int) get_bytes(data,  23, 4)); }
        public void baseline_b_mm_SET(int  src) //Current baseline in ECEF y or NED east component in mm.
        {  set_bytes((int)(src) & -1L, 4, data,  23); }
        public int baseline_c_mm_GET()//Current baseline in ECEF z or NED down component in mm.
        {  return (int)((int) get_bytes(data,  27, 4)); }
        public void baseline_c_mm_SET(int  src) //Current baseline in ECEF z or NED down component in mm.
        {  set_bytes((int)(src) & -1L, 4, data,  27); }
        public int iar_num_hypotheses_GET()//Current number of integer ambiguity hypotheses.
        {  return (int)((int) get_bytes(data,  31, 4)); }
        public void iar_num_hypotheses_SET(int  src) //Current number of integer ambiguity hypotheses.
        {  set_bytes((int)(src) & -1L, 4, data,  31); }
        static final Meta meta = new Meta(127, 1, 3, 0, 35, 280);
    }/**
*RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting*/
    public static class GPS2_RTK extends Pack  implements CommunicationChannel.Sendable
    {

        GPS2_RTK() { super(meta, 0); }
        GPS2_RTK(int bytes) { super(meta, bytes); }
        public char wn_GET()//GPS Week Number of last baseline
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void wn_SET(char  src) //GPS Week Number of last baseline
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long time_last_baseline_ms_GET()//Time since boot of last baseline message received in ms.
        {  return (get_bytes(data,  2, 4)); }
        public void time_last_baseline_ms_SET(long  src) //Time since boot of last baseline message received in ms.
        {  set_bytes((src) & -1L, 4, data,  2); }
        public long tow_GET()//GPS Time of Week of last baseline
        {  return (get_bytes(data,  6, 4)); }
        public void tow_SET(long  src) //GPS Time of Week of last baseline
        {  set_bytes((src) & -1L, 4, data,  6); }
        public long accuracy_GET()//Current estimate of baseline accuracy.
        {  return (get_bytes(data,  10, 4)); }
        public void accuracy_SET(long  src) //Current estimate of baseline accuracy.
        {  set_bytes((src) & -1L, 4, data,  10); }
        public char rtk_receiver_id_GET()//Identification of connected RTK receiver.
        {  return (char)((char) get_bytes(data,  14, 1)); }
        public void rtk_receiver_id_SET(char  src) //Identification of connected RTK receiver.
        {  set_bytes((char)(src) & -1L, 1, data,  14); }
        public char rtk_health_GET()//GPS-specific health report for RTK data.
        {  return (char)((char) get_bytes(data,  15, 1)); }
        public void rtk_health_SET(char  src) //GPS-specific health report for RTK data.
        {  set_bytes((char)(src) & -1L, 1, data,  15); }
        public char rtk_rate_GET()//Rate of baseline messages being received by GPS, in HZ
        {  return (char)((char) get_bytes(data,  16, 1)); }
        public void rtk_rate_SET(char  src) //Rate of baseline messages being received by GPS, in HZ
        {  set_bytes((char)(src) & -1L, 1, data,  16); }
        public char nsats_GET()//Current number of sats used for RTK calculation.
        {  return (char)((char) get_bytes(data,  17, 1)); }
        public void nsats_SET(char  src) //Current number of sats used for RTK calculation.
        {  set_bytes((char)(src) & -1L, 1, data,  17); }
        public char baseline_coords_type_GET()//Coordinate system of baseline. 0 == ECEF, 1 == NED
        {  return (char)((char) get_bytes(data,  18, 1)); }
        public void baseline_coords_type_SET(char  src) //Coordinate system of baseline. 0 == ECEF, 1 == NED
        {  set_bytes((char)(src) & -1L, 1, data,  18); }
        public int baseline_a_mm_GET()//Current baseline in ECEF x or NED north component in mm.
        {  return (int)((int) get_bytes(data,  19, 4)); }
        public void baseline_a_mm_SET(int  src) //Current baseline in ECEF x or NED north component in mm.
        {  set_bytes((int)(src) & -1L, 4, data,  19); }
        public int baseline_b_mm_GET()//Current baseline in ECEF y or NED east component in mm.
        {  return (int)((int) get_bytes(data,  23, 4)); }
        public void baseline_b_mm_SET(int  src) //Current baseline in ECEF y or NED east component in mm.
        {  set_bytes((int)(src) & -1L, 4, data,  23); }
        public int baseline_c_mm_GET()//Current baseline in ECEF z or NED down component in mm.
        {  return (int)((int) get_bytes(data,  27, 4)); }
        public void baseline_c_mm_SET(int  src) //Current baseline in ECEF z or NED down component in mm.
        {  set_bytes((int)(src) & -1L, 4, data,  27); }
        public int iar_num_hypotheses_GET()//Current number of integer ambiguity hypotheses.
        {  return (int)((int) get_bytes(data,  31, 4)); }
        public void iar_num_hypotheses_SET(int  src) //Current number of integer ambiguity hypotheses.
        {  set_bytes((int)(src) & -1L, 4, data,  31); }
        static final Meta meta = new Meta(128, 1, 3, 0, 35, 280);
    }/**
*The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described
*	 unit*/
    public static class SCALED_IMU3 extends Pack  implements CommunicationChannel.Sendable
    {

        SCALED_IMU3() { super(meta, 0); }
        SCALED_IMU3(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public short xacc_GET()//X acceleration (mg)
        {  return (short)((short) get_bytes(data,  4, 2)); }
        public void xacc_SET(short  src) //X acceleration (mg)
        {  set_bytes((short)(src) & -1L, 2, data,  4); }
        public short yacc_GET()//Y acceleration (mg)
        {  return (short)((short) get_bytes(data,  6, 2)); }
        public void yacc_SET(short  src) //Y acceleration (mg)
        {  set_bytes((short)(src) & -1L, 2, data,  6); }
        public short zacc_GET()//Z acceleration (mg)
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public void zacc_SET(short  src) //Z acceleration (mg)
        {  set_bytes((short)(src) & -1L, 2, data,  8); }
        public short xgyro_GET()//Angular speed around X axis (millirad /sec)
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public void xgyro_SET(short  src) //Angular speed around X axis (millirad /sec)
        {  set_bytes((short)(src) & -1L, 2, data,  10); }
        public short ygyro_GET()//Angular speed around Y axis (millirad /sec)
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public void ygyro_SET(short  src) //Angular speed around Y axis (millirad /sec)
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        public short zgyro_GET()//Angular speed around Z axis (millirad /sec)
        {  return (short)((short) get_bytes(data,  14, 2)); }
        public void zgyro_SET(short  src) //Angular speed around Z axis (millirad /sec)
        {  set_bytes((short)(src) & -1L, 2, data,  14); }
        public short xmag_GET()//X Magnetic field (milli tesla)
        {  return (short)((short) get_bytes(data,  16, 2)); }
        public void xmag_SET(short  src) //X Magnetic field (milli tesla)
        {  set_bytes((short)(src) & -1L, 2, data,  16); }
        public short ymag_GET()//Y Magnetic field (milli tesla)
        {  return (short)((short) get_bytes(data,  18, 2)); }
        public void ymag_SET(short  src) //Y Magnetic field (milli tesla)
        {  set_bytes((short)(src) & -1L, 2, data,  18); }
        public short zmag_GET()//Z Magnetic field (milli tesla)
        {  return (short)((short) get_bytes(data,  20, 2)); }
        public void zmag_SET(short  src) //Z Magnetic field (milli tesla)
        {  set_bytes((short)(src) & -1L, 2, data,  20); }
        static final Meta meta = new Meta(129, 0, 1, 0, 22, 176);
    } public static class DATA_TRANSMISSION_HANDSHAKE extends Pack  implements CommunicationChannel.Sendable
    {

        DATA_TRANSMISSION_HANDSHAKE() { super(meta, 0); }
        DATA_TRANSMISSION_HANDSHAKE(int bytes) { super(meta, bytes); }
        public char width_GET()//Width of a matrix or image
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void width_SET(char  src) //Width of a matrix or image
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char height_GET()//Height of a matrix or image
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void height_SET(char  src) //Height of a matrix or image
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char packets_GET()//number of packets beeing sent (set on ACK only)
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void packets_SET(char  src) //number of packets beeing sent (set on ACK only)
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public long size_GET()//total data size in bytes (set on ACK only)
        {  return (get_bytes(data,  6, 4)); }
        public void size_SET(long  src) //total data size in bytes (set on ACK only)
        {  set_bytes((src) & -1L, 4, data,  6); }
        public char type_GET()//type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public void type_SET(char  src) //type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
        /**
        *payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on
        *	 ACK only*/
        public char payload_GET()
        {  return (char)((char) get_bytes(data,  11, 1)); }
        /**
        *payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on
        *	 ACK only*/
        public void payload_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  11); }
        public char jpg_quality_GET()//JPEG quality out of [1,100]
        {  return (char)((char) get_bytes(data,  12, 1)); }
        public void jpg_quality_SET(char  src) //JPEG quality out of [1,100]
        {  set_bytes((char)(src) & -1L, 1, data,  12); }
        static final Meta meta = new Meta(130, 3, 1, 0, 13, 104);
    } public static class ENCAPSULATED_DATA extends Pack  implements CommunicationChannel.Sendable
    {

        ENCAPSULATED_DATA() { super(meta, 0); }
        ENCAPSULATED_DATA(int bytes) { super(meta, bytes); }
        public char seqnr_GET()//sequence number (starting with 0 on every transmission)
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void seqnr_SET(char  src) //sequence number (starting with 0 on every transmission)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char[] data__GET(char[]  dst_ch, int pos)  //image data bytes
        {
            for(int BYTE = 2, dst_max = pos + 253; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//image data bytes
        {return data__GET(new char[253], 0);} public void data__SET(char[]  src, int pos)  //image data bytes
        {
            for(int BYTE =  2, src_max = pos + 253; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(131, 1, 0, 0, 255, 2040);
    } public static class DISTANCE_SENSOR extends Pack  implements CommunicationChannel.Sendable
    {

        DISTANCE_SENSOR() { super(meta, 0); }
        DISTANCE_SENSOR(int bytes) { super(meta, bytes); }
        public char min_distance_GET()//Minimum distance the sensor can measure in centimeters
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void min_distance_SET(char  src) //Minimum distance the sensor can measure in centimeters
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char max_distance_GET()//Maximum distance the sensor can measure in centimeters
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void max_distance_SET(char  src) //Maximum distance the sensor can measure in centimeters
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char current_distance_GET()//Current distance reading
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void current_distance_SET(char  src) //Current distance reading
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public long time_boot_ms_GET()//Time since system boot
        {  return (get_bytes(data,  6, 4)); }
        public void time_boot_ms_SET(long  src) //Time since system boot
        {  set_bytes((src) & -1L, 4, data,  6); }
        public char id_GET()//Onboard ID of the sensor
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public void id_SET(char  src) //Onboard ID of the sensor
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
        public char covariance_GET()//Measurement covariance in centimeters, 0 for unknown / invalid readings
        {  return (char)((char) get_bytes(data,  11, 1)); }
        public void covariance_SET(char  src) //Measurement covariance in centimeters, 0 for unknown / invalid readings
        {  set_bytes((char)(src) & -1L, 1, data,  11); }
        public @MAV_DISTANCE_SENSOR int type_GET()//Type from MAV_DISTANCE_SENSOR enum.
        {  return  0 + (int)get_bits(data, 96, 3); }
        public void type_SET(@MAV_DISTANCE_SENSOR int  src) //Type from MAV_DISTANCE_SENSOR enum.
        {  set_bits(- 0 +   src, 3, data, 96); }
        /**
        *Direction the sensor faces from MAV_SENSOR_ORIENTATION enum. downward-facing: ROTATION_PITCH_270, upward-facing:
        *	 ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90,
        *	 right-facing: ROTATION_YAW_27*/
        public @MAV_SENSOR_ORIENTATION int orientation_GET()
        {  return  0 + (int)get_bits(data, 99, 6); }
        /**
        *Direction the sensor faces from MAV_SENSOR_ORIENTATION enum. downward-facing: ROTATION_PITCH_270, upward-facing:
        *	 ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90,
        *	 right-facing: ROTATION_YAW_27*/
        public void orientation_SET(@MAV_SENSOR_ORIENTATION int  src)
        {  set_bits(- 0 +   src, 6, data, 99); }
        static final Meta meta = new Meta(132, 3, 1, 0, 14, 105);
    }/**
*Request for terrain data and terrain status*/
    public static class TERRAIN_REQUEST extends Pack  implements CommunicationChannel.Sendable
    {

        TERRAIN_REQUEST() { super(meta, 0); }
        TERRAIN_REQUEST(int bytes) { super(meta, bytes); }
        public char grid_spacing_GET()//Grid spacing in meters
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void grid_spacing_SET(char  src) //Grid spacing in meters
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long mask_GET()//Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
        {  return (get_bytes(data,  2, 8)); }
        public void mask_SET(long  src) //Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
        {  set_bytes((src) & -1L, 8, data,  2); }
        public int lat_GET()//Latitude of SW corner of first grid (degrees *10^7)
        {  return (int)((int) get_bytes(data,  10, 4)); }
        public void lat_SET(int  src) //Latitude of SW corner of first grid (degrees *10^7)
        {  set_bytes((int)(src) & -1L, 4, data,  10); }
        public int lon_GET()//Longitude of SW corner of first grid (in degrees *10^7)
        {  return (int)((int) get_bytes(data,  14, 4)); }
        public void lon_SET(int  src) //Longitude of SW corner of first grid (in degrees *10^7)
        {  set_bytes((int)(src) & -1L, 4, data,  14); }
        static final Meta meta = new Meta(133, 1, 0, 1, 18, 144);
    }/**
*Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUES*/
    public static class TERRAIN_DATA extends Pack  implements CommunicationChannel.Sendable
    {

        TERRAIN_DATA() { super(meta, 0); }
        TERRAIN_DATA(int bytes) { super(meta, bytes); }
        public char grid_spacing_GET()//Grid spacing in meters
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void grid_spacing_SET(char  src) //Grid spacing in meters
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public int lat_GET()//Latitude of SW corner of first grid (degrees *10^7)
        {  return (int)((int) get_bytes(data,  2, 4)); }
        public void lat_SET(int  src) //Latitude of SW corner of first grid (degrees *10^7)
        {  set_bytes((int)(src) & -1L, 4, data,  2); }
        public int lon_GET()//Longitude of SW corner of first grid (in degrees *10^7)
        {  return (int)((int) get_bytes(data,  6, 4)); }
        public void lon_SET(int  src) //Longitude of SW corner of first grid (in degrees *10^7)
        {  set_bytes((int)(src) & -1L, 4, data,  6); }
        public char gridbit_GET()//bit within the terrain request mask
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public void gridbit_SET(char  src) //bit within the terrain request mask
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
        public short[] data__GET(short[]  dst_ch, int pos)  //Terrain data in meters AMSL
        {
            for(int BYTE = 11, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (short)((short) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public short[] data__GET()//Terrain data in meters AMSL
        {return data__GET(new short[16], 0);} public void data__SET(short[]  src, int pos)  //Terrain data in meters AMSL
        {
            for(int BYTE =  11, src_max = pos + 16; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        }
        static final Meta meta = new Meta(134, 1, 0, 0, 43, 344);
    }/**
*Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle
*	 has all terrain data needed for a mission*/
    public static class TERRAIN_CHECK extends Pack  implements CommunicationChannel.Sendable
    {

        TERRAIN_CHECK() { super(meta, 0); }
        TERRAIN_CHECK(int bytes) { super(meta, bytes); }
        public int lat_GET()//Latitude (degrees *10^7)
        {  return (int)((int) get_bytes(data,  0, 4)); }
        public void lat_SET(int  src) //Latitude (degrees *10^7)
        {  set_bytes((int)(src) & -1L, 4, data,  0); }
        public int lon_GET()//Longitude (degrees *10^7)
        {  return (int)((int) get_bytes(data,  4, 4)); }
        public void lon_SET(int  src) //Longitude (degrees *10^7)
        {  set_bytes((int)(src) & -1L, 4, data,  4); }
        static final Meta meta = new Meta(135, 0, 0, 0, 8, 64);
    }/**
*Response from a TERRAIN_CHECK request*/
    public static class TERRAIN_REPORT extends Pack  implements CommunicationChannel.Sendable
    {

        TERRAIN_REPORT() { super(meta, 0); }
        TERRAIN_REPORT(int bytes) { super(meta, bytes); }
        public char spacing_GET()//grid spacing (zero if terrain at this location unavailable)
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void spacing_SET(char  src) //grid spacing (zero if terrain at this location unavailable)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char pending_GET()//Number of 4x4 terrain blocks waiting to be received or read from disk
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void pending_SET(char  src) //Number of 4x4 terrain blocks waiting to be received or read from disk
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char loaded_GET()//Number of 4x4 terrain blocks in memory
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void loaded_SET(char  src) //Number of 4x4 terrain blocks in memory
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public int lat_GET()//Latitude (degrees *10^7)
        {  return (int)((int) get_bytes(data,  6, 4)); }
        public void lat_SET(int  src) //Latitude (degrees *10^7)
        {  set_bytes((int)(src) & -1L, 4, data,  6); }
        public int lon_GET()//Longitude (degrees *10^7)
        {  return (int)((int) get_bytes(data,  10, 4)); }
        public void lon_SET(int  src) //Longitude (degrees *10^7)
        {  set_bytes((int)(src) & -1L, 4, data,  10); }
        public float terrain_height_GET()//Terrain height in meters AMSL
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public void terrain_height_SET(float  src) //Terrain height in meters AMSL
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public float current_height_GET()//Current vehicle height above lat/lon terrain height (meters)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public void current_height_SET(float  src) //Current vehicle height above lat/lon terrain height (meters)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        static final Meta meta = new Meta(136, 3, 0, 0, 22, 176);
    }/**
*Barometer readings for 2nd barometer*/
    public static class SCALED_PRESSURE2 extends Pack  implements CommunicationChannel.Sendable
    {

        SCALED_PRESSURE2() { super(meta, 0); }
        SCALED_PRESSURE2(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public float press_abs_GET()//Absolute pressure (hectopascal)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void press_abs_SET(float  src) //Absolute pressure (hectopascal)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float press_diff_GET()//Differential pressure 1 (hectopascal)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void press_diff_SET(float  src) //Differential pressure 1 (hectopascal)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public short temperature_GET()//Temperature measurement (0.01 degrees celsius)
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public void temperature_SET(short  src) //Temperature measurement (0.01 degrees celsius)
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        static final Meta meta = new Meta(137, 0, 1, 0, 14, 112);
    }/**
*Motion capture attitude and position*/
    public static class ATT_POS_MOCAP extends Pack  implements CommunicationChannel.Sendable
    {

        ATT_POS_MOCAP() { super(meta, 0); }
        ATT_POS_MOCAP(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float[] q_GET(float[]  dst_ch, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        {
            for(int BYTE = 8, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] q_GET()//Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        {return q_GET(new float[4], 0);} public void q_SET(float[]  src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        {
            for(int BYTE =  8, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float x_GET()//X position in meters (NED)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void x_SET(float  src) //X position in meters (NED)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float y_GET()//Y position in meters (NED)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void y_SET(float  src) //Y position in meters (NED)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float z_GET()//Z position in meters (NED)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void z_SET(float  src) //Z position in meters (NED)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        static final Meta meta = new Meta(138, 0, 0, 1, 36, 288);
    }/**
*Set the vehicle attitude and body angular rates.*/
    public static class SET_ACTUATOR_CONTROL_TARGET extends Pack  implements CommunicationChannel.Sendable
    {

        SET_ACTUATOR_CONTROL_TARGET() { super(meta, 0); }
        SET_ACTUATOR_CONTROL_TARGET(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch)
        {  set_bytes((src) & -1L, 8, data,  0); }
        /**
        *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
        *	 this field to difference between instances*/
        public char group_mlx_GET()
        {  return (char)((char) get_bytes(data,  8, 1)); }
        /**
        *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
        *	 this field to difference between instances*/
        public void group_mlx_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  9); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
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
        {return controls_GET(new float[8], 0);}/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*	 mixer to repurpose them as generic outputs*/
        public void controls_SET(float[]  src, int pos)
        {
            for(int BYTE =  11, src_max = pos + 8; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        static final Meta meta = new Meta(139, 0, 0, 1, 43, 344);
    }/**
*Set the vehicle attitude and body angular rates.*/
    public static class ACTUATOR_CONTROL_TARGET extends Pack  implements CommunicationChannel.Sendable
    {

        ACTUATOR_CONTROL_TARGET() { super(meta, 0); }
        ACTUATOR_CONTROL_TARGET(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch)
        {  set_bytes((src) & -1L, 8, data,  0); }
        /**
        *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
        *	 this field to difference between instances*/
        public char group_mlx_GET()
        {  return (char)((char) get_bytes(data,  8, 1)); }
        /**
        *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
        *	 this field to difference between instances*/
        public void group_mlx_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
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
        {return controls_GET(new float[8], 0);}/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*	 mixer to repurpose them as generic outputs*/
        public void controls_SET(float[]  src, int pos)
        {
            for(int BYTE =  9, src_max = pos + 8; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        static final Meta meta = new Meta(140, 0, 0, 1, 41, 328);
    }/**
*The current system altitude.*/
    public static class ALTITUDE extends Pack  implements CommunicationChannel.Sendable
    {

        ALTITUDE() { super(meta, 0); }
        ALTITUDE(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch)
        {  set_bytes((src) & -1L, 8, data,  0); }
        /**
        *This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
        *	 local altitude change). The only guarantee on this field is that it will never be reset and is consistent
        *	 within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
        *	 time. This altitude will also drift and vary between flights*/
        public float altitude_monotonic_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        /**
        *This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
        *	 local altitude change). The only guarantee on this field is that it will never be reset and is consistent
        *	 within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
        *	 time. This altitude will also drift and vary between flights*/
        public void altitude_monotonic_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        /**
        *This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
        *	 like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
        *	 are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
        *	 by default and not the WGS84 altitude*/
        public float altitude_amsl_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        /**
        *This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
        *	 like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
        *	 are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
        *	 by default and not the WGS84 altitude*/
        public void altitude_amsl_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        /**
        *This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
        *	 to the coordinate origin (0, 0, 0). It is up-positive*/
        public float altitude_local_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        /**
        *This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
        *	 to the coordinate origin (0, 0, 0). It is up-positive*/
        public void altitude_local_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float altitude_relative_GET()//This is the altitude above the home position. It resets on each change of the current home position
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void altitude_relative_SET(float  src) //This is the altitude above the home position. It resets on each change of the current home position
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        /**
        *This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
        *	 than -1000 should be interpreted as unknown*/
        public float altitude_terrain_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        /**
        *This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
        *	 than -1000 should be interpreted as unknown*/
        public void altitude_terrain_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        /**
        *This is not the altitude, but the clear space below the system according to the fused clearance estimate.
        *	 It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
        *	 target. A negative value indicates no measurement available*/
        public float bottom_clearance_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        /**
        *This is not the altitude, but the clear space below the system according to the fused clearance estimate.
        *	 It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
        *	 target. A negative value indicates no measurement available*/
        public void bottom_clearance_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        static final Meta meta = new Meta(141, 0, 0, 1, 32, 256);
    }/**
*The autopilot is requesting a resource (file, binary, other type of data)*/
    public static class RESOURCE_REQUEST extends Pack  implements CommunicationChannel.Sendable
    {

        RESOURCE_REQUEST() { super(meta, 0); }
        RESOURCE_REQUEST(int bytes) { super(meta, bytes); }
        public char request_id_GET()//Request ID. This ID should be re-used when sending back URI contents
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void request_id_SET(char  src) //Request ID. This ID should be re-used when sending back URI contents
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char uri_type_GET()//The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void uri_type_SET(char  src) //The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
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
        {return uri_GET(new char[120], 0);}/**
*The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
*	 on the URI type enum*/
        public void uri_SET(char[]  src, int pos)
        {
            for(int BYTE =  2, src_max = pos + 120; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public char transfer_type_GET()//The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
        {  return (char)((char) get_bytes(data,  122, 1)); }
        public void transfer_type_SET(char  src) //The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
        {  set_bytes((char)(src) & -1L, 1, data,  122); }
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
        {return storage_GET(new char[120], 0);}/**
*The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
*	 has a storage associated (e.g. MAVLink FTP)*/
        public void storage_SET(char[]  src, int pos)
        {
            for(int BYTE =  123, src_max = pos + 120; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(142, 0, 0, 0, 243, 1944);
    }/**
*Barometer readings for 3rd barometer*/
    public static class SCALED_PRESSURE3 extends Pack  implements CommunicationChannel.Sendable
    {

        SCALED_PRESSURE3() { super(meta, 0); }
        SCALED_PRESSURE3(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public float press_abs_GET()//Absolute pressure (hectopascal)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void press_abs_SET(float  src) //Absolute pressure (hectopascal)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float press_diff_GET()//Differential pressure 1 (hectopascal)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void press_diff_SET(float  src) //Differential pressure 1 (hectopascal)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public short temperature_GET()//Temperature measurement (0.01 degrees celsius)
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public void temperature_SET(short  src) //Temperature measurement (0.01 degrees celsius)
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        static final Meta meta = new Meta(143, 0, 1, 0, 14, 112);
    }/**
*current motion information from a designated system*/
    public static class FOLLOW_TARGET extends Pack  implements CommunicationChannel.Sendable
    {

        FOLLOW_TARGET() { super(meta, 0); }
        FOLLOW_TARGET(int bytes) { super(meta, bytes); }
        public long timestamp_GET()//Timestamp in milliseconds since system boot
        {  return (get_bytes(data,  0, 8)); }
        public void timestamp_SET(long  src) //Timestamp in milliseconds since system boot
        {  set_bytes((src) & -1L, 8, data,  0); }
        public long custom_state_GET()//button states or switches of a tracker device
        {  return (get_bytes(data,  8, 8)); }
        public void custom_state_SET(long  src) //button states or switches of a tracker device
        {  set_bytes((src) & -1L, 8, data,  8); }
        public char est_capabilities_GET()//bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
        {  return (char)((char) get_bytes(data,  16, 1)); }
        public void est_capabilities_SET(char  src) //bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
        {  set_bytes((char)(src) & -1L, 1, data,  16); }
        public int lat_GET()//Latitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  17, 4)); }
        public void lat_SET(int  src) //Latitude (WGS84), in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  17); }
        public int lon_GET()//Longitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  21, 4)); }
        public void lon_SET(int  src) //Longitude (WGS84), in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  21); }
        public float alt_GET()//AMSL, in meters
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  25, 4))); }
        public void alt_SET(float  src) //AMSL, in meters
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 25); }
        public float[] vel_GET(float[]  dst_ch, int pos)  //target velocity (0,0,0) for unknown
        {
            for(int BYTE = 29, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] vel_GET()//target velocity (0,0,0) for unknown
        {return vel_GET(new float[3], 0);} public void vel_SET(float[]  src, int pos)  //target velocity (0,0,0) for unknown
        {
            for(int BYTE =  29, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float[] acc_GET(float[]  dst_ch, int pos)  //linear target acceleration (0,0,0) for unknown
        {
            for(int BYTE = 41, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] acc_GET()//linear target acceleration (0,0,0) for unknown
        {return acc_GET(new float[3], 0);} public void acc_SET(float[]  src, int pos)  //linear target acceleration (0,0,0) for unknown
        {
            for(int BYTE =  41, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float[] attitude_q_GET(float[]  dst_ch, int pos)  //(1 0 0 0 for unknown)
        {
            for(int BYTE = 53, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] attitude_q_GET()//(1 0 0 0 for unknown)
        {return attitude_q_GET(new float[4], 0);} public void attitude_q_SET(float[]  src, int pos)  //(1 0 0 0 for unknown)
        {
            for(int BYTE =  53, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float[] rates_GET(float[]  dst_ch, int pos)  //(0 0 0 for unknown)
        {
            for(int BYTE = 69, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] rates_GET()//(0 0 0 for unknown)
        {return rates_GET(new float[3], 0);} public void rates_SET(float[]  src, int pos)  //(0 0 0 for unknown)
        {
            for(int BYTE =  69, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float[] position_cov_GET(float[]  dst_ch, int pos)  //eph epv
        {
            for(int BYTE = 81, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] position_cov_GET()//eph epv
        {return position_cov_GET(new float[3], 0);} public void position_cov_SET(float[]  src, int pos)  //eph epv
        {
            for(int BYTE =  81, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        static final Meta meta = new Meta(144, 0, 0, 2, 93, 744);
    }/**
*The smoothed, monotonic system state used to feed the control loops of the system.*/
    public static class CONTROL_SYSTEM_STATE extends Pack  implements CommunicationChannel.Sendable
    {

        CONTROL_SYSTEM_STATE() { super(meta, 0); }
        CONTROL_SYSTEM_STATE(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float x_acc_GET()//X acceleration in body frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void x_acc_SET(float  src) //X acceleration in body frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float y_acc_GET()//Y acceleration in body frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void y_acc_SET(float  src) //Y acceleration in body frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float z_acc_GET()//Z acceleration in body frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void z_acc_SET(float  src) //Z acceleration in body frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float x_vel_GET()//X velocity in body frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void x_vel_SET(float  src) //X velocity in body frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float y_vel_GET()//Y velocity in body frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void y_vel_SET(float  src) //Y velocity in body frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float z_vel_GET()//Z velocity in body frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void z_vel_SET(float  src) //Z velocity in body frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float x_pos_GET()//X position in local frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void x_pos_SET(float  src) //X position in local frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float y_pos_GET()//Y position in local frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void y_pos_SET(float  src) //Y position in local frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public float z_pos_GET()//Z position in local frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public void z_pos_SET(float  src) //Z position in local frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public float airspeed_GET()//Airspeed, set to -1 if unknown
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public void airspeed_SET(float  src) //Airspeed, set to -1 if unknown
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }
        public float[] vel_variance_GET(float[]  dst_ch, int pos)  //Variance of body velocity estimate
        {
            for(int BYTE = 48, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] vel_variance_GET()//Variance of body velocity estimate
        {return vel_variance_GET(new float[3], 0);} public void vel_variance_SET(float[]  src, int pos)  //Variance of body velocity estimate
        {
            for(int BYTE =  48, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float[] pos_variance_GET(float[]  dst_ch, int pos)  //Variance in local position
        {
            for(int BYTE = 60, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] pos_variance_GET()//Variance in local position
        {return pos_variance_GET(new float[3], 0);} public void pos_variance_SET(float[]  src, int pos)  //Variance in local position
        {
            for(int BYTE =  60, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float[] q_GET(float[]  dst_ch, int pos)  //The attitude, represented as Quaternion
        {
            for(int BYTE = 72, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] q_GET()//The attitude, represented as Quaternion
        {return q_GET(new float[4], 0);} public void q_SET(float[]  src, int pos)  //The attitude, represented as Quaternion
        {
            for(int BYTE =  72, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float roll_rate_GET()//Angular rate in roll axis
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  88, 4))); }
        public void roll_rate_SET(float  src) //Angular rate in roll axis
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 88); }
        public float pitch_rate_GET()//Angular rate in pitch axis
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  92, 4))); }
        public void pitch_rate_SET(float  src) //Angular rate in pitch axis
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 92); }
        public float yaw_rate_GET()//Angular rate in yaw axis
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  96, 4))); }
        public void yaw_rate_SET(float  src) //Angular rate in yaw axis
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 96); }
        static final Meta meta = new Meta(146, 0, 0, 1, 100, 800);
    }/**
*Battery information*/
    public static class BATTERY_STATUS extends Pack  implements CommunicationChannel.Sendable
    {

        BATTERY_STATUS() { super(meta, 0); }
        BATTERY_STATUS(int bytes) { super(meta, bytes); }
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
        {return voltages_GET(new char[10], 0);}/**
*Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
*	 should have the UINT16_MAX value*/
        public void voltages_SET(char[]  src, int pos)
        {
            for(int BYTE =  0, src_max = pos + 10; pos < src_max; pos++, BYTE += 2)
                set_bytes((char)(src[pos]) & -1L, 2, data,  BYTE);
        }
        public char id_GET()//Battery ID
        {  return (char)((char) get_bytes(data,  20, 1)); }
        public void id_SET(char  src) //Battery ID
        {  set_bytes((char)(src) & -1L, 1, data,  20); }
        public short temperature_GET()//Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
        {  return (short)((short) get_bytes(data,  21, 2)); }
        public void temperature_SET(short  src) //Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
        {  set_bytes((short)(src) & -1L, 2, data,  21); }
        public short current_battery_GET()//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
        {  return (short)((short) get_bytes(data,  23, 2)); }
        public void current_battery_SET(short  src) //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
        {  set_bytes((short)(src) & -1L, 2, data,  23); }
        public int current_consumed_GET()//Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimat
        {  return (int)((int) get_bytes(data,  25, 4)); }
        public void current_consumed_SET(int  src) //Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimat
        {  set_bytes((int)(src) & -1L, 4, data,  25); }
        /**
        *Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide
        *	 energy consumption estimat*/
        public int energy_consumed_GET()
        {  return (int)((int) get_bytes(data,  29, 4)); }
        /**
        *Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide
        *	 energy consumption estimat*/
        public void energy_consumed_SET(int  src)
        {  set_bytes((int)(src) & -1L, 4, data,  29); }
        public byte battery_remaining_GET()//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
        {  return (byte)((byte) get_bytes(data,  33, 1)); }
        public void battery_remaining_SET(byte  src) //Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
        {  set_bytes((byte)(src) & -1L, 1, data,  33); }
        public @MAV_BATTERY_FUNCTION int battery_function_GET()//Function of the battery
        {  return  0 + (int)get_bits(data, 272, 3); }
        public void battery_function_SET(@MAV_BATTERY_FUNCTION int  src) //Function of the battery
        {  set_bits(- 0 +   src, 3, data, 272); }
        public @MAV_BATTERY_TYPE int type_GET()//Type (chemistry) of the battery
        {  return  0 + (int)get_bits(data, 275, 3); }
        public void type_SET(@MAV_BATTERY_TYPE int  src) //Type (chemistry) of the battery
        {  set_bits(- 0 +   src, 3, data, 275); }
        static final Meta meta = new Meta(147, 10, 0, 0, 35, 278);
    }/**
*Version and capability of autopilot software*/
    public static class AUTOPILOT_VERSION extends Pack  implements CommunicationChannel.Sendable
    {

        AUTOPILOT_VERSION() { super(meta, 0); }
        AUTOPILOT_VERSION(int bytes) { super(meta, bytes); }
        public char vendor_id_GET()//ID of the board vendor
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void vendor_id_SET(char  src) //ID of the board vendor
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char product_id_GET()//ID of the product
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void product_id_SET(char  src) //ID of the product
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public long flight_sw_version_GET()//Firmware version number
        {  return (get_bytes(data,  4, 4)); }
        public void flight_sw_version_SET(long  src) //Firmware version number
        {  set_bytes((src) & -1L, 4, data,  4); }
        public long middleware_sw_version_GET()//Middleware version number
        {  return (get_bytes(data,  8, 4)); }
        public void middleware_sw_version_SET(long  src) //Middleware version number
        {  set_bytes((src) & -1L, 4, data,  8); }
        public long os_sw_version_GET()//Operating system version number
        {  return (get_bytes(data,  12, 4)); }
        public void os_sw_version_SET(long  src) //Operating system version number
        {  set_bytes((src) & -1L, 4, data,  12); }
        public long board_version_GET()//HW / board version (last 8 bytes should be silicon ID, if any)
        {  return (get_bytes(data,  16, 4)); }
        public void board_version_SET(long  src) //HW / board version (last 8 bytes should be silicon ID, if any)
        {  set_bytes((src) & -1L, 4, data,  16); }
        public long uid_GET()//UID if provided by hardware (see uid2)
        {  return (get_bytes(data,  20, 8)); }
        public void uid_SET(long  src) //UID if provided by hardware (see uid2)
        {  set_bytes((src) & -1L, 8, data,  20); }
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
        public void flight_custom_version_SET(char[]  src, int pos)
        {
            for(int BYTE =  28, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        /**
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
        public void middleware_custom_version_SET(char[]  src, int pos)
        {
            for(int BYTE =  36, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        /**
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
        {return os_custom_version_GET(new char[8], 0);}/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	 should allow to identify the commit using the main version number even for very large code bases*/
        public void os_custom_version_SET(char[]  src, int pos)
        {
            for(int BYTE =  44, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public @MAV_PROTOCOL_CAPABILITY int capabilities_GET()//bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
        {  return  1 + (int)get_bits(data, 416, 17); }
        public void capabilities_SET(@MAV_PROTOCOL_CAPABILITY int  src) //bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
        {  set_bits(- 1 +   src, 17, data, 416); }
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
        }/**
*UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
*	 use uid*/
        public void uid2_SET(char[]  src, int pos, Bounds.Inside ph)
        {
            if(ph.field_bit != 433)insert_field(ph, 433, 0);
            for(int BYTE =  ph.BYTE, src_max = pos + 18; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        } static final Meta meta = new Meta(148, 2, 4, 1, 56, 433, 0, _mc);
    }/**
*The location of a landing area captured from a downward facing camera*/
    public static class LANDING_TARGET extends Pack  implements CommunicationChannel.Sendable
    {

        LANDING_TARGET() { super(meta, 0); }
        LANDING_TARGET(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public char target_num_GET()//The ID of the target if multiple targets are present
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public void target_num_SET(char  src) //The ID of the target if multiple targets are present
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        public float angle_x_GET()//X-axis angular offset (in radians) of the target from the center of the image
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  9, 4))); }
        public void angle_x_SET(float  src) //X-axis angular offset (in radians) of the target from the center of the image
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 9); }
        public float angle_y_GET()//Y-axis angular offset (in radians) of the target from the center of the image
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  13, 4))); }
        public void angle_y_SET(float  src) //Y-axis angular offset (in radians) of the target from the center of the image
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 13); }
        public float distance_GET()//Distance to the target from the vehicle in meters
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  17, 4))); }
        public void distance_SET(float  src) //Distance to the target from the vehicle in meters
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 17); }
        public float size_x_GET()//Size in radians of target along x-axis
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  21, 4))); }
        public void size_x_SET(float  src) //Size in radians of target along x-axis
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 21); }
        public float size_y_GET()//Size in radians of target along y-axis
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  25, 4))); }
        public void size_y_SET(float  src) //Size in radians of target along y-axis
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 25); }
        public @MAV_FRAME int frame_GET()//MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
        {  return  0 + (int)get_bits(data, 232, 4); }
        public void frame_SET(@MAV_FRAME int  src) //MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
        {  set_bits(- 0 +   src, 4, data, 232); }
        public @LANDING_TARGET_TYPE int type_GET()//LANDING_TARGET_TYPE enum specifying the type of landing target
        {  return  0 + (int)get_bits(data, 236, 2); }
        public void type_SET(@LANDING_TARGET_TYPE int  src) //LANDING_TARGET_TYPE enum specifying the type of landing target
        {  set_bits(- 0 +   src, 2, data, 236); }
        public float  x_TRY(Bounds.Inside ph)//X Position of the landing target on MAV_FRAME
        {
            if(ph.field_bit !=  238 && !try_visit_field(ph, 238)) return 0;
            return (float)(Float.intBitsToFloat((int) get_bytes(data,  ph.BYTE, 4)));
        }
        public void x_SET(float  src, Bounds.Inside ph)//X Position of the landing target on MAV_FRAME
        {
            if(ph.field_bit != 238)insert_field(ph, 238, 0);
            set_bytes(Float.floatToIntBits(src) & -1L, 4, data, ph.BYTE);
        } public float  y_TRY(Bounds.Inside ph) //Y Position of the landing target on MAV_FRAME
        {
            if(ph.field_bit !=  239 && !try_visit_field(ph, 239)) return 0;
            return (float)(Float.intBitsToFloat((int) get_bytes(data,  ph.BYTE, 4)));
        }
        public void y_SET(float  src, Bounds.Inside ph)//Y Position of the landing target on MAV_FRAME
        {
            if(ph.field_bit != 239)insert_field(ph, 239, 0);
            set_bytes(Float.floatToIntBits(src) & -1L, 4, data, ph.BYTE);
        } public float  z_TRY(Bounds.Inside ph) //Z Position of the landing target on MAV_FRAME
        {
            if(ph.field_bit !=  240 && !try_visit_field(ph, 240)) return 0;
            return (float)(Float.intBitsToFloat((int) get_bytes(data,  ph.BYTE, 4)));
        }
        public void z_SET(float  src, Bounds.Inside ph)//Z Position of the landing target on MAV_FRAME
        {
            if(ph.field_bit != 240)insert_field(ph, 240, 0);
            set_bytes(Float.floatToIntBits(src) & -1L, 4, data, ph.BYTE);
        } public float[]  q_TRY(Bounds.Inside ph) //Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
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
        } public void q_SET(float[]  src, int pos, Bounds.Inside ph)//Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        {
            if(ph.field_bit != 241)insert_field(ph, 241, 0);
            for(int BYTE =  ph.BYTE, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }/**
*Boolean indicating known position (1) or default unkown position (0), for validation of positioning of
*	 the landing targe*/
        public char  position_valid_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  242 && !try_visit_field(ph, 242)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 1));
        }
        /**
        *Boolean indicating known position (1) or default unkown position (0), for validation of positioning of
        *	 the landing targe*/
        public void position_valid_SET(char  src, Bounds.Inside ph)
        {
            if(ph.field_bit != 242)insert_field(ph, 242, 0);
            set_bytes((char)(src) & -1L, 1, data,  ph.BYTE);
        } static final Meta meta = new Meta(149, 0, 0, 1, 31, 238, 0, _ic, _lc, _ac, _cc, _Uc);
    }/**
*Offsets and calibrations values for hardware sensors. This makes it easier to debug the calibration process*/
    public static class SENSOR_OFFSETS extends Pack  implements CommunicationChannel.Sendable
    {

        SENSOR_OFFSETS() { super(meta, 0); }
        public void mag_ofs_x_SET(short  src) //magnetometer X offset
        {  set_bytes((short)(src) & -1L, 2, data,  0); }
        public void mag_ofs_y_SET(short  src) //magnetometer Y offset
        {  set_bytes((short)(src) & -1L, 2, data,  2); }
        public void mag_ofs_z_SET(short  src) //magnetometer Z offset
        {  set_bytes((short)(src) & -1L, 2, data,  4); }
        public void mag_declination_SET(float  src) //magnetic declination (radians)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }
        public void raw_press_SET(int  src) //raw pressure from barometer
        {  set_bytes((int)(src) & -1L, 4, data,  10); }
        public void raw_temp_SET(int  src) //raw temperature from barometer
        {  set_bytes((int)(src) & -1L, 4, data,  14); }
        public void gyro_cal_x_SET(float  src) //gyro X calibration
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public void gyro_cal_y_SET(float  src) //gyro Y calibration
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
        public void gyro_cal_z_SET(float  src) //gyro Z calibration
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 26); }
        public void accel_cal_x_SET(float  src) //accel X calibration
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 30); }
        public void accel_cal_y_SET(float  src) //accel Y calibration
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 34); }
        public void accel_cal_z_SET(float  src) //accel Z calibration
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 38); }
        static final Meta meta = new Meta(150, 0, 0, 0, 42, 336);
    }/**
*Deprecated. Use MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS instead. Set the magnetometer offsets*/
    public static class SET_MAG_OFFSETS extends Pack  implements CommunicationChannel.Sendable
    {

        SET_MAG_OFFSETS() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void mag_ofs_x_SET(short  src) //magnetometer X offset
        {  set_bytes((short)(src) & -1L, 2, data,  2); }
        public void mag_ofs_y_SET(short  src) //magnetometer Y offset
        {  set_bytes((short)(src) & -1L, 2, data,  4); }
        public void mag_ofs_z_SET(short  src) //magnetometer Z offset
        {  set_bytes((short)(src) & -1L, 2, data,  6); }
        static final Meta meta = new Meta(151, 0, 0, 0, 8, 64);
    }/**
*state of APM memory*/
    public static class MEMINFO extends Pack  implements CommunicationChannel.Sendable
    {

        MEMINFO() { super(meta, 0); }
        public void brkval_SET(char  src) //heap top
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void freemem_SET(char  src) //free memory
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public void freemem32_SET(long  src, Bounds.Inside ph)//free memory (32 bit)
        {
            if(ph.field_bit != 32)insert_field(ph, 32, 0);
            set_bytes((src) & -1L, 4, data,  ph.BYTE);
        } static final Meta meta = new Meta(152, 2, 0, 0, 5, 32, 0, _Nk);
    }/**
*raw ADC output*/
    public static class AP_ADC extends Pack  implements CommunicationChannel.Sendable
    {

        AP_ADC() { super(meta, 0); }
        public void adc1_SET(char  src) //ADC output 1
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void adc2_SET(char  src) //ADC output 2
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public void adc3_SET(char  src) //ADC output 3
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public void adc4_SET(char  src) //ADC output 4
        {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public void adc5_SET(char  src) //ADC output 5
        {  set_bytes((char)(src) & -1L, 2, data,  8); }
        public void adc6_SET(char  src) //ADC output 6
        {  set_bytes((char)(src) & -1L, 2, data,  10); }
        static final Meta meta = new Meta(153, 6, 0, 0, 12, 96);
    }/**
*Configure on-board Camera Control System.*/
    public static class DIGICAM_CONFIGURE extends Pack  implements CommunicationChannel.Sendable
    {

        DIGICAM_CONFIGURE() { super(meta, 0); }
        public void shutter_speed_SET(char  src) //Divisor number e.g. 1000 means 1/1000 (0 means ignore)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public void mode_SET(char  src) //Mode enumeration from 1 to N P, TV, AV, M, Etc (0 means ignore)
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void aperture_SET(char  src) //F stop number x 10 e.g. 28 means 2.8 (0 means ignore)
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public void iso_SET(char  src) //ISO enumeration from 1 to N e.g. 80, 100, 200, Etc (0 means ignore)
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public void exposure_type_SET(char  src) //Exposure type enumeration from 1 to N (0 means ignore)
        {  set_bytes((char)(src) & -1L, 1, data,  7); }
        /**
        *Command Identity (incremental loop: 0 to 255)A command sent multiple times will be executed or pooled
        *	 just onc*/
        public void command_id_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        public void engine_cut_off_SET(char  src) //Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
        {  set_bytes((char)(src) & -1L, 1, data,  9); }
        public void extra_param_SET(char  src) //Extra parameters enumeration (0 means ignore)
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
        public void extra_value_SET(float  src) //Correspondent value to given extra_param
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 11); }
        static final Meta meta = new Meta(154, 1, 0, 0, 15, 120);
    }/**
*Control on-board Camera Control System to take shots.*/
    public static class DIGICAM_CONTROL extends Pack  implements CommunicationChannel.Sendable
    {

        DIGICAM_CONTROL() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void session_SET(char  src) //0: stop, 1: start or keep it up Session control e.g. show/hide lens
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void zoom_pos_SET(char  src) //1 to N Zoom's absolute position (0 means ignore)
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public void zoom_step_SET(byte  src) //-100 to 100 Zooming step value to offset zoom from the current position
        {  set_bytes((byte)(src) & -1L, 1, data,  4); }
        public void focus_lock_SET(char  src) //0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public void shot_SET(char  src) //0: ignore, 1: shot or start filming
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        /**
        *Command Identity (incremental loop: 0 to 255)A command sent multiple times will be executed or pooled
        *	 just onc*/
        public void command_id_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  7); }
        public void extra_param_SET(char  src) //Extra parameters enumeration (0 means ignore)
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        public void extra_value_SET(float  src) //Correspondent value to given extra_param
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 9); }
        static final Meta meta = new Meta(155, 0, 0, 0, 13, 104);
    }/**
*Message to configure a camera mount, directional antenna, etc.*/
    public static class MOUNT_CONFIGURE extends Pack  implements CommunicationChannel.Sendable
    {

        MOUNT_CONFIGURE() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void stab_roll_SET(char  src) //(1 = yes, 0 = no)
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void stab_pitch_SET(char  src) //(1 = yes, 0 = no)
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public void stab_yaw_SET(char  src) //(1 = yes, 0 = no)
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void mount_mode_SET(@MAV_MOUNT_MODE int  src) //mount operating mode (see MAV_MOUNT_MODE enum)
        {  set_bits(- 0 +   src, 3, data, 40); }
        static final Meta meta = new Meta(156, 0, 0, 0, 6, 43);
    }/**
*Message to control a camera mount, directional antenna, etc.*/
    public static class MOUNT_CONTROL extends Pack  implements CommunicationChannel.Sendable
    {

        MOUNT_CONTROL() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void input_a_SET(int  src) //pitch(deg*100) or lat, depending on mount mode
        {  set_bytes((int)(src) & -1L, 4, data,  2); }
        public void input_b_SET(int  src) //roll(deg*100) or lon depending on mount mode
        {  set_bytes((int)(src) & -1L, 4, data,  6); }
        public void input_c_SET(int  src) //yaw(deg*100) or alt (in cm) depending on mount mode
        {  set_bytes((int)(src) & -1L, 4, data,  10); }
        public void save_position_SET(char  src) //if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
        {  set_bytes((char)(src) & -1L, 1, data,  14); }
        static final Meta meta = new Meta(157, 0, 0, 0, 15, 120);
    }/**
*Message with some status from APM to GCS about camera or antenna mount*/
    public static class MOUNT_STATUS extends Pack  implements CommunicationChannel.Sendable
    {

        MOUNT_STATUS() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void pointing_a_SET(int  src) //pitch(deg*100)
        {  set_bytes((int)(src) & -1L, 4, data,  2); }
        public void pointing_b_SET(int  src) //roll(deg*100)
        {  set_bytes((int)(src) & -1L, 4, data,  6); }
        public void pointing_c_SET(int  src) //yaw(deg*100)
        {  set_bytes((int)(src) & -1L, 4, data,  10); }
        static final Meta meta = new Meta(158, 0, 0, 0, 14, 112);
    }/**
*GCS*/
    public static class FENCE_POINT extends Pack  implements CommunicationChannel.Sendable
    {

        FENCE_POINT() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void idx_SET(char  src) //point index (first point is 1, 0 is for return point)
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void count_SET(char  src) //total number of points (for sanity checking)
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public void lat_SET(float  src) //Latitude of point
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void lng_SET(float  src) //Longitude of point
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        static final Meta meta = new Meta(160, 0, 0, 0, 12, 96);
    }/**
*Request a current fence point from MAV*/
    public static class FENCE_FETCH_POINT extends Pack  implements CommunicationChannel.Sendable
    {

        FENCE_FETCH_POINT() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void idx_SET(char  src) //point index (first point is 1, 0 is for return point)
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        static final Meta meta = new Meta(161, 0, 0, 0, 3, 24);
    }/**
*Status of geo-fencing. Sent in extended status stream when fencing enabled*/
    public static class FENCE_STATUS extends Pack  implements CommunicationChannel.Sendable
    {

        FENCE_STATUS() { super(meta, 0); }
        public void breach_count_SET(char  src) //number of fence breaches
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void breach_time_SET(long  src) //time of last breach in milliseconds since boot
        {  set_bytes((src) & -1L, 4, data,  2); }
        public void breach_status_SET(char  src) //0 if currently inside fence, 1 if outside
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public void breach_type_SET(@FENCE_BREACH int  src) //last breach type (see FENCE_BREACH_* enum)
        {  set_bits(- 0 +   src, 2, data, 56); }
        static final Meta meta = new Meta(162, 1, 1, 0, 8, 58);
    }/**
*Status of DCM attitude estimator*/
    public static class AHRS extends Pack  implements CommunicationChannel.Sendable
    {

        AHRS() { super(meta, 0); }
        public void omegaIx_SET(float  src) //X gyro drift estimate rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 0); }
        public void omegaIy_SET(float  src) //Y gyro drift estimate rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void omegaIz_SET(float  src) //Z gyro drift estimate rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void accel_weight_SET(float  src) //average accel_weight
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void renorm_val_SET(float  src) //average renormalisation value
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void error_rp_SET(float  src) //average error_roll_pitch value
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void error_yaw_SET(float  src) //average error_yaw value
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        static final Meta meta = new Meta(163, 0, 0, 0, 28, 224);
    }/**
*Status of simulation environment, if used*/
    public static class SIMSTATE extends Pack  implements CommunicationChannel.Sendable
    {

        SIMSTATE() { super(meta, 0); }
        public void roll_SET(float  src) //Roll angle (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 0); }
        public void pitch_SET(float  src) //Pitch angle (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void yaw_SET(float  src) //Yaw angle (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void xacc_SET(float  src) //X acceleration m/s/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void yacc_SET(float  src) //Y acceleration m/s/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void zacc_SET(float  src) //Z acceleration m/s/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void xgyro_SET(float  src) //Angular speed around X axis rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public void ygyro_SET(float  src) //Angular speed around Y axis rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public void zgyro_SET(float  src) //Angular speed around Z axis rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public void lat_SET(int  src) //Latitude in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  36); }
        public void lng_SET(int  src) //Longitude in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  40); }
        static final Meta meta = new Meta(164, 0, 0, 0, 44, 352);
    }/**
*Status of key hardware*/
    public static class HWSTATUS extends Pack  implements CommunicationChannel.Sendable
    {

        HWSTATUS() { super(meta, 0); }
        public void Vcc_SET(char  src) //board voltage (mV)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void I2Cerr_SET(char  src) //I2C error count
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        static final Meta meta = new Meta(165, 1, 0, 0, 3, 24);
    }/**
*Status generated by radio*/
    public static class RADIO extends Pack  implements CommunicationChannel.Sendable
    {

        RADIO() { super(meta, 0); }
        public void rxerrors_SET(char  src) //receive errors
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void fixed__SET(char  src) //count of error corrected packets
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public void rssi_SET(char  src) //local signal strength
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void remrssi_SET(char  src) //remote signal strength
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public void txbuf_SET(char  src) //how full the tx buffer is as a percentage
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public void noise_SET(char  src) //background noise level
        {  set_bytes((char)(src) & -1L, 1, data,  7); }
        public void remnoise_SET(char  src) //remote background noise level
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        static final Meta meta = new Meta(166, 2, 0, 0, 9, 72);
    }/**
*Status of AP_Limits. Sent in extended status stream when AP_Limits is enabled*/
    public static class LIMITS_STATUS extends Pack  implements CommunicationChannel.Sendable
    {

        LIMITS_STATUS() { super(meta, 0); }
        public void breach_count_SET(char  src) //number of fence breaches
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void last_trigger_SET(long  src) //time of last breach in milliseconds since boot
        {  set_bytes((src) & -1L, 4, data,  2); }
        public void last_action_SET(long  src) //time of last recovery action in milliseconds since boot
        {  set_bytes((src) & -1L, 4, data,  6); }
        public void last_recovery_SET(long  src) //time of last successful recovery in milliseconds since boot
        {  set_bytes((src) & -1L, 4, data,  10); }
        public void last_clear_SET(long  src) //time of last all-clear in milliseconds since boot
        {  set_bytes((src) & -1L, 4, data,  14); }
        public void limits_state_SET(@LIMITS_STATE int  src) //state of AP_Limits, (see enum LimitState, LIMITS_STATE)
        {  set_bits(- 0 +   src, 3, data, 144); }
        public void mods_enabled_SET(@LIMIT_MODULE int  src) //AP_Limit_Module bitfield of enabled modules, (see enum moduleid or LIMIT_MODULE)
        {  set_bits(- 1 +   src, 3, data, 147); }
        public void mods_required_SET(@LIMIT_MODULE int  src) //AP_Limit_Module bitfield of required modules, (see enum moduleid or LIMIT_MODULE)
        {  set_bits(- 1 +   src, 3, data, 150); }
        public void mods_triggered_SET(@LIMIT_MODULE int  src) //AP_Limit_Module bitfield of triggered modules, (see enum moduleid or LIMIT_MODULE)
        {  set_bits(- 1 +   src, 3, data, 153); }
        static final Meta meta = new Meta(167, 1, 4, 0, 20, 156);
    }/**
*Wind estimation*/
    public static class WIND extends Pack  implements CommunicationChannel.Sendable
    {

        WIND() { super(meta, 0); }
        public void direction_SET(float  src) //wind direction that wind is coming from (degrees)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 0); }
        public void speed_SET(float  src) //wind speed in ground plane (m/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void speed_z_SET(float  src) //vertical wind speed (m/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        static final Meta meta = new Meta(168, 0, 0, 0, 12, 96);
    }/**
*Data packet, size 16*/
    public static class DATA16 extends Pack  implements CommunicationChannel.Sendable
    {

        DATA16() { super(meta, 0); }
        public void type_SET(char  src) //data type
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void len_SET(char  src) //data length
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void data__SET(char[]  src, int pos)  //raw data
        {
            for(int BYTE =  2, src_max = pos + 16; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(169, 0, 0, 0, 18, 144);
    }/**
*Data packet, size 32*/
    public static class DATA32 extends Pack  implements CommunicationChannel.Sendable
    {

        DATA32() { super(meta, 0); }
        public void type_SET(char  src) //data type
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void len_SET(char  src) //data length
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void data__SET(char[]  src, int pos)  //raw data
        {
            for(int BYTE =  2, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(170, 0, 0, 0, 34, 272);
    }/**
*Data packet, size 64*/
    public static class DATA64 extends Pack  implements CommunicationChannel.Sendable
    {

        DATA64() { super(meta, 0); }
        public void type_SET(char  src) //data type
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void len_SET(char  src) //data length
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void data__SET(char[]  src, int pos)  //raw data
        {
            for(int BYTE =  2, src_max = pos + 64; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(171, 0, 0, 0, 66, 528);
    }/**
*Data packet, size 96*/
    public static class DATA96 extends Pack  implements CommunicationChannel.Sendable
    {

        DATA96() { super(meta, 0); }
        public void type_SET(char  src) //data type
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void len_SET(char  src) //data length
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void data__SET(char[]  src, int pos)  //raw data
        {
            for(int BYTE =  2, src_max = pos + 96; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(172, 0, 0, 0, 98, 784);
    }/**
*Rangefinder reporting*/
    public static class RANGEFINDER extends Pack  implements CommunicationChannel.Sendable
    {

        RANGEFINDER() { super(meta, 0); }
        public void distance_SET(float  src) //distance in meters
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 0); }
        public void voltage_SET(float  src) //raw voltage if available, zero otherwise
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        static final Meta meta = new Meta(173, 0, 0, 0, 8, 64);
    }/**
*Airspeed auto-calibration*/
    public static class AIRSPEED_AUTOCAL extends Pack  implements CommunicationChannel.Sendable
    {

        AIRSPEED_AUTOCAL() { super(meta, 0); }
        public void vx_SET(float  src) //GPS velocity north m/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 0); }
        public void vy_SET(float  src) //GPS velocity east m/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void vz_SET(float  src) //GPS velocity down m/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void diff_pressure_SET(float  src) //Differential pressure pascals
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void EAS2TAS_SET(float  src) //Estimated to true airspeed ratio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void ratio_SET(float  src) //Airspeed ratio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void state_x_SET(float  src) //EKF state x
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public void state_y_SET(float  src) //EKF state y
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public void state_z_SET(float  src) //EKF state z
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public void Pax_SET(float  src) //EKF Pax
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public void Pby_SET(float  src) //EKF Pby
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public void Pcz_SET(float  src) //EKF Pcz
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }
        static final Meta meta = new Meta(174, 0, 0, 0, 48, 384);
    }/**
*GCS*/
    public static class RALLY_POINT extends Pack  implements CommunicationChannel.Sendable
    {

        RALLY_POINT() { super(meta, 0); }
        public void land_dir_SET(char  src) //Heading to aim for when landing. In centi-degrees.
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public void idx_SET(char  src) //point index (first point is 0)
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void count_SET(char  src) //total number of points (for sanity checking)
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public void lat_SET(int  src) //Latitude of point in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  6); }
        public void lng_SET(int  src) //Longitude of point in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  10); }
        public void alt_SET(short  src) //Transit / loiter altitude in meters relative to home
        {  set_bytes((short)(src) & -1L, 2, data,  14); }
        public void break_alt_SET(short  src) //Break altitude in meters relative to home
        {  set_bytes((short)(src) & -1L, 2, data,  16); }
        public void flags_SET(@RALLY_FLAGS int  src) //See RALLY_FLAGS enum for definition of the bitmask.
        {  set_bits(- 1 +   src, 1, data, 144); }
        static final Meta meta = new Meta(175, 1, 0, 0, 19, 145);
    }/**
*Request a current rally point from MAV. MAV should respond with a RALLY_POINT message. MAV should not
*	 respond if the request is invalid*/
    public static class RALLY_FETCH_POINT extends Pack  implements CommunicationChannel.Sendable
    {

        RALLY_FETCH_POINT() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void idx_SET(char  src) //point index (first point is 0)
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        static final Meta meta = new Meta(176, 0, 0, 0, 3, 24);
    }/**
*Status of compassmot calibration*/
    public static class COMPASSMOT_STATUS extends Pack  implements CommunicationChannel.Sendable
    {

        COMPASSMOT_STATUS() { super(meta, 0); }
        public void throttle_SET(char  src) //throttle (percent*10)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void interference_SET(char  src) //interference (percent)
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public void current_SET(float  src) //current (Ampere)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void CompensationX_SET(float  src) //Motor Compensation X
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void CompensationY_SET(float  src) //Motor Compensation Y
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void CompensationZ_SET(float  src) //Motor Compensation Z
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        static final Meta meta = new Meta(177, 2, 0, 0, 20, 160);
    }/**
*Status of secondary AHRS filter if available*/
    public static class AHRS2 extends Pack  implements CommunicationChannel.Sendable
    {

        AHRS2() { super(meta, 0); }
        public void roll_SET(float  src) //Roll angle (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 0); }
        public void pitch_SET(float  src) //Pitch angle (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void yaw_SET(float  src) //Yaw angle (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void altitude_SET(float  src) //Altitude (MSL)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void lat_SET(int  src) //Latitude in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  16); }
        public void lng_SET(int  src) //Longitude in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  20); }
        static final Meta meta = new Meta(178, 0, 0, 0, 24, 192);
    }/**
*Camera Event*/
    public static class CAMERA_STATUS extends Pack  implements CommunicationChannel.Sendable
    {

        CAMERA_STATUS() { super(meta, 0); }
        public void img_idx_SET(char  src) //Image index
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void time_usec_SET(long  src) //Image timestamp (microseconds since UNIX epoch, according to camera clock)
        {  set_bytes((src) & -1L, 8, data,  2); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
        public void cam_idx_SET(char  src) //Camera ID
        {  set_bytes((char)(src) & -1L, 1, data,  11); }
        public void p1_SET(float  src) //Parameter 1 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void p2_SET(float  src) //Parameter 2 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void p3_SET(float  src) //Parameter 3 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void p4_SET(float  src) //Parameter 4 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public void event_id_SET(@CAMERA_STATUS_TYPES int  src) //See CAMERA_STATUS_TYPES enum for definition of the bitmask
        {  set_bits(- 0 +   src, 3, data, 224); }
        static final Meta meta = new Meta(179, 1, 0, 1, 29, 227);
    }/**
*Camera Capture Feedback*/
    public static class CAMERA_FEEDBACK extends Pack  implements CommunicationChannel.Sendable
    {

        CAMERA_FEEDBACK() { super(meta, 0); }
        public void img_idx_SET(char  src) //Image index
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        /**
        *Image timestamp (microseconds since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if
        *	 no CCB*/
        public void time_usec_SET(long  src)
        {  set_bytes((src) & -1L, 8, data,  2); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
        public void cam_idx_SET(char  src) //Camera ID
        {  set_bytes((char)(src) & -1L, 1, data,  11); }
        public void lat_SET(int  src) //Latitude in (deg * 1E7)
        {  set_bytes((int)(src) & -1L, 4, data,  12); }
        public void lng_SET(int  src) //Longitude in (deg * 1E7)
        {  set_bytes((int)(src) & -1L, 4, data,  16); }
        public void alt_msl_SET(float  src) //Altitude Absolute (meters AMSL)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void alt_rel_SET(float  src) //Altitude Relative (meters above HOME location)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public void roll_SET(float  src) //Camera Roll angle (earth frame, degrees, +-180)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public void pitch_SET(float  src) //Camera Pitch angle (earth frame, degrees, +-180)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public void yaw_SET(float  src) //Camera Yaw (earth frame, degrees, 0-360, true)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public void foc_len_SET(float  src) //Focal Length (mm)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public void flags_SET(@CAMERA_FEEDBACK_FLAGS int  src) //See CAMERA_FEEDBACK_FLAGS enum for definition of the bitmask
        {  set_bits(- 0 +   src, 3, data, 352); }
        static final Meta meta = new Meta(180, 1, 0, 1, 45, 355);
    }/**
*2nd Battery status*/
    public static class BATTERY2 extends Pack  implements CommunicationChannel.Sendable
    {

        BATTERY2() { super(meta, 0); }
        public void voltage_SET(char  src) //voltage in millivolts
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void current_battery_SET(short  src) //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
        {  set_bytes((short)(src) & -1L, 2, data,  2); }
        static final Meta meta = new Meta(181, 1, 0, 0, 4, 32);
    }/**
*Status of third AHRS filter if available. This is for ANU research group (Ali and Sean)*/
    public static class AHRS3 extends Pack  implements CommunicationChannel.Sendable
    {

        AHRS3() { super(meta, 0); }
        public void roll_SET(float  src) //Roll angle (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 0); }
        public void pitch_SET(float  src) //Pitch angle (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void yaw_SET(float  src) //Yaw angle (rad)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void altitude_SET(float  src) //Altitude (MSL)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void lat_SET(int  src) //Latitude in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  16); }
        public void lng_SET(int  src) //Longitude in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  20); }
        public void v1_SET(float  src) //test variable1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public void v2_SET(float  src) //test variable2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public void v3_SET(float  src) //test variable3
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public void v4_SET(float  src) //test variable4
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        static final Meta meta = new Meta(182, 0, 0, 0, 40, 320);
    }/**
*Request the autopilot version from the system/component.*/
    public static class AUTOPILOT_VERSION_REQUEST extends Pack  implements CommunicationChannel.Sendable
    {

        AUTOPILOT_VERSION_REQUEST() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        static final Meta meta = new Meta(183, 0, 0, 0, 2, 16);
    }/**
*Send a block of log data to remote location*/
    public static class REMOTE_LOG_DATA_BLOCK extends Pack  implements CommunicationChannel.Sendable
    {

        REMOTE_LOG_DATA_BLOCK() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void data__SET(char[]  src, int pos)  //log data block
        {
            for(int BYTE =  2, src_max = pos + 200; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public void seqno_SET(@MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS int  src) //log data block sequence number
        {  set_bits(- 2147483645 +   src, 1, data, 1616); }
        static final Meta meta = new Meta(184, 0, 0, 0, 203, 1617);
    }/**
*Send Status of each log block that autopilot board might have sent*/
    public static class REMOTE_LOG_BLOCK_STATUS extends Pack  implements CommunicationChannel.Sendable
    {

        REMOTE_LOG_BLOCK_STATUS() { super(meta, 0); }
        public void seqno_SET(long  src) //log data block sequence number
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public void status_SET(@MAV_REMOTE_LOG_DATA_BLOCK_STATUSES int  src) //log data block status
        {  set_bits(- 0 +   src, 1, data, 48); }
        static final Meta meta = new Meta(185, 0, 1, 0, 7, 49);
    }/**
*Control vehicle LEDs*/
    public static class LED_CONTROL extends Pack  implements CommunicationChannel.Sendable
    {

        LED_CONTROL() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void instance_SET(char  src) //Instance (LED instance to control or 255 for all LEDs)
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void pattern_SET(char  src) //Pattern (see LED_PATTERN_ENUM)
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public void custom_len_SET(char  src) //Custom Byte Length
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void custom_bytes_SET(char[]  src, int pos)  //Custom Bytes
        {
            for(int BYTE =  5, src_max = pos + 24; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(186, 0, 0, 0, 29, 232);
    }/**
*Reports progress of compass calibration.*/
    public static class MAG_CAL_PROGRESS extends Pack  implements CommunicationChannel.Sendable
    {

        MAG_CAL_PROGRESS() { super(meta, 0); }
        public void compass_id_SET(char  src) //Compass being calibrated
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void cal_mask_SET(char  src) //Bitmask of compasses being calibrated
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void attempt_SET(char  src) //Attempt number
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void completion_pct_SET(char  src) //Completion percentage
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public void completion_mask_SET(char[]  src, int pos)  //Bitmask of sphere sections (see http:en.wikipedia.org/wiki/Geodesic_grid)
        {
            for(int BYTE =  4, src_max = pos + 10; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public void direction_x_SET(float  src) //Body frame direction vector for display
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public void direction_y_SET(float  src) //Body frame direction vector for display
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public void direction_z_SET(float  src) //Body frame direction vector for display
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
        public void cal_status_SET(@MAG_CAL_STATUS int  src) //Status (see MAG_CAL_STATUS enum)
        {  set_bits(- 0 +   src, 3, data, 208); }
        static final Meta meta = new Meta(191, 0, 0, 0, 27, 211);
    }/**
*Reports results of completed compass calibration. Sent until MAG_CAL_ACK received.*/
    public static class MAG_CAL_REPORT extends Pack  implements CommunicationChannel.Sendable
    {

        MAG_CAL_REPORT() { super(meta, 0); }
        public void compass_id_SET(char  src) //Compass being calibrated
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void cal_mask_SET(char  src) //Bitmask of compasses being calibrated
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void autosaved_SET(char  src) //0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void fitness_SET(float  src) //RMS milligauss residuals
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 3); }
        public void ofs_x_SET(float  src) //X offset
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 7); }
        public void ofs_y_SET(float  src) //Y offset
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 11); }
        public void ofs_z_SET(float  src) //Z offset
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 15); }
        public void diag_x_SET(float  src) //X diagonal (matrix 11)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 19); }
        public void diag_y_SET(float  src) //Y diagonal (matrix 22)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 23); }
        public void diag_z_SET(float  src) //Z diagonal (matrix 33)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 27); }
        public void offdiag_x_SET(float  src) //X off-diagonal (matrix 12 and 21)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 31); }
        public void offdiag_y_SET(float  src) //Y off-diagonal (matrix 13 and 31)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 35); }
        public void offdiag_z_SET(float  src) //Z off-diagonal (matrix 32 and 23)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 39); }
        public void cal_status_SET(@MAG_CAL_STATUS int  src) //Status (see MAG_CAL_STATUS enum)
        {  set_bits(- 0 +   src, 3, data, 344); }
        static final Meta meta = new Meta(192, 0, 0, 0, 44, 347);
    }/**
*EKF Status message including flags and variances*/
    public static class EKF_STATUS_REPORT extends Pack  implements CommunicationChannel.Sendable
    {

        EKF_STATUS_REPORT() { super(meta, 0); }
        public void velocity_variance_SET(float  src) //Velocity variance
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 0); }
        public void pos_horiz_variance_SET(float  src) //Horizontal Position variance
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void pos_vert_variance_SET(float  src) //Vertical Position variance
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void compass_variance_SET(float  src) //Compass variance
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void terrain_alt_variance_SET(float  src) //Terrain Altitude variance
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void flags_SET(@EKF_STATUS_FLAGS int  src) //Flags
        {  set_bits(- 1 +   src, 10, data, 160); }
        static final Meta meta = new Meta(193, 0, 0, 0, 22, 170);
    }/**
*PID tuning information*/
    public static class PID_TUNING extends Pack  implements CommunicationChannel.Sendable
    {

        PID_TUNING() { super(meta, 0); }
        public void desired_SET(float  src) //desired rate (degrees/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 0); }
        public void achieved_SET(float  src) //achieved rate (degrees/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void FF_SET(float  src) //FF component
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void P_SET(float  src) //P component
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void I_SET(float  src) //I component
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void D_SET(float  src) //D component
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void axis_SET(@PID_TUNING_AXIS int  src) //axis
        {  set_bits(- 1 +   src, 3, data, 192); }
        static final Meta meta = new Meta(194, 0, 0, 0, 25, 195);
    }/**
*3 axis gimbal mesuraments*/
    public static class GIMBAL_REPORT extends Pack  implements CommunicationChannel.Sendable
    {

        GIMBAL_REPORT() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void delta_time_SET(float  src) //Time since last update (seconds)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 2); }
        public void delta_angle_x_SET(float  src) //Delta angle X (radians)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }
        public void delta_angle_y_SET(float  src) //Delta angle Y (radians)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }
        public void delta_angle_z_SET(float  src) //Delta angle X (radians)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public void delta_velocity_x_SET(float  src) //Delta velocity X (m/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public void delta_velocity_y_SET(float  src) //Delta velocity Y (m/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
        public void delta_velocity_z_SET(float  src) //Delta velocity Z (m/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 26); }
        public void joint_roll_SET(float  src) //Joint ROLL (radians)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 30); }
        public void joint_el_SET(float  src) //Joint EL (radians)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 34); }
        public void joint_az_SET(float  src) //Joint AZ (radians)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 38); }
        static final Meta meta = new Meta(200, 0, 0, 0, 42, 336);
    }/**
*Control message for rate gimbal*/
    public static class GIMBAL_CONTROL extends Pack  implements CommunicationChannel.Sendable
    {

        GIMBAL_CONTROL() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void demanded_rate_x_SET(float  src) //Demanded angular rate X (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 2); }
        public void demanded_rate_y_SET(float  src) //Demanded angular rate Y (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }
        public void demanded_rate_z_SET(float  src) //Demanded angular rate Z (rad/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }
        static final Meta meta = new Meta(201, 0, 0, 0, 14, 112);
    }/**
*100 Hz gimbal torque command telemetry*/
    public static class GIMBAL_TORQUE_CMD_REPORT extends Pack  implements CommunicationChannel.Sendable
    {

        GIMBAL_TORQUE_CMD_REPORT() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void rl_torque_cmd_SET(short  src) //Roll Torque Command
        {  set_bytes((short)(src) & -1L, 2, data,  2); }
        public void el_torque_cmd_SET(short  src) //Elevation Torque Command
        {  set_bytes((short)(src) & -1L, 2, data,  4); }
        public void az_torque_cmd_SET(short  src) //Azimuth Torque Command
        {  set_bytes((short)(src) & -1L, 2, data,  6); }
        static final Meta meta = new Meta(214, 0, 0, 0, 8, 64);
    }/**
*Heartbeat from a HeroBus attached GoPro*/
    public static class GOPRO_HEARTBEAT extends Pack  implements CommunicationChannel.Sendable
    {

        GOPRO_HEARTBEAT() { super(meta, 0); }
        public void status_SET(@GOPRO_HEARTBEAT_STATUS int  src) //Status
        {  set_bits(- 0 +   src, 2, data, 0); }
        public void capture_mode_SET(@GOPRO_CAPTURE_MODE int  src) //Current capture mode
        {
            long id = 0;
            switch(src)
            {
                case GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_VIDEO:
                    id = 0;
                    break;
                case GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PHOTO:
                    id = 1;
                    break;
                case GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_BURST:
                    id = 2;
                    break;
                case GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_TIME_LAPSE:
                    id = 3;
                    break;
                case GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_MULTI_SHOT:
                    id = 4;
                    break;
                case GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PLAYBACK:
                    id = 5;
                    break;
                case GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_SETUP:
                    id = 6;
                    break;
                case GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_UNKNOWN:
                    id = 7;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 4, data, 2);
        }
        public void flags_SET(@GOPRO_HEARTBEAT_FLAGS int  src) //additional status bits
        {  set_bits(- 1 +   src, 1, data, 6); }
        static final Meta meta = new Meta(215, 0, 0, 0, 1, 7);
    }/**
*Request a GOPRO_COMMAND response from the GoPro*/
    public static class GOPRO_GET_REQUEST extends Pack  implements CommunicationChannel.Sendable
    {

        GOPRO_GET_REQUEST() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void cmd_id_SET(@GOPRO_COMMAND int  src) //Command ID
        {  set_bits(- 0 +   src, 5, data, 16); }
        static final Meta meta = new Meta(216, 0, 0, 0, 3, 21);
    }/**
*Response from a GOPRO_COMMAND get request*/
    public static class GOPRO_GET_RESPONSE extends Pack  implements CommunicationChannel.Sendable
    {

        GOPRO_GET_RESPONSE() { super(meta, 0); }
        public void value_SET(char[]  src, int pos)  //Value
        {
            for(int BYTE =  0, src_max = pos + 4; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public void cmd_id_SET(@GOPRO_COMMAND int  src) //Command ID
        {  set_bits(- 0 +   src, 5, data, 32); }
        public void status_SET(@GOPRO_REQUEST_STATUS int  src) //Status
        {  set_bits(- 0 +   src, 1, data, 37); }
        static final Meta meta = new Meta(217, 0, 0, 0, 5, 38);
    }/**
*Request to set a GOPRO_COMMAND with a desired*/
    public static class GOPRO_SET_REQUEST extends Pack  implements CommunicationChannel.Sendable
    {

        GOPRO_SET_REQUEST() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void value_SET(char[]  src, int pos)  //Value
        {
            for(int BYTE =  2, src_max = pos + 4; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public void cmd_id_SET(@GOPRO_COMMAND int  src) //Command ID
        {  set_bits(- 0 +   src, 5, data, 48); }
        static final Meta meta = new Meta(218, 0, 0, 0, 7, 53);
    }/**
*Response from a GOPRO_COMMAND set request*/
    public static class GOPRO_SET_RESPONSE extends Pack  implements CommunicationChannel.Sendable
    {

        GOPRO_SET_RESPONSE() { super(meta, 0); }
        public void cmd_id_SET(@GOPRO_COMMAND int  src) //Command ID
        {  set_bits(- 0 +   src, 5, data, 0); }
        public void status_SET(@GOPRO_REQUEST_STATUS int  src) //Status
        {  set_bits(- 0 +   src, 1, data, 5); }
        static final Meta meta = new Meta(219, 0, 0, 0, 1, 6);
    }/**
*RPM sensor output*/
    public static class RPM extends Pack  implements CommunicationChannel.Sendable
    {

        RPM() { super(meta, 0); }
        public void rpm1_SET(float  src) //RPM Sensor1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 0); }
        public void rpm2_SET(float  src) //RPM Sensor2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        static final Meta meta = new Meta(226, 0, 0, 0, 8, 64);
    }/**
*Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message
*	 is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS
*	 enum definition for further information. The innovaton test ratios show the magnitude of the sensor innovation
*	 divided by the innovation check threshold. Under normal operation the innovaton test ratios should be
*	 below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation
*	 and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation
*	 test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should
*	 be optional and controllable by the user*/
    public static class ESTIMATOR_STATUS extends Pack  implements CommunicationChannel.Sendable
    {

        ESTIMATOR_STATUS() { super(meta, 0); }
        ESTIMATOR_STATUS(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float vel_ratio_GET()//Velocity innovation test ratio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void vel_ratio_SET(float  src) //Velocity innovation test ratio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float pos_horiz_ratio_GET()//Horizontal position innovation test ratio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void pos_horiz_ratio_SET(float  src) //Horizontal position innovation test ratio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float pos_vert_ratio_GET()//Vertical position innovation test ratio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void pos_vert_ratio_SET(float  src) //Vertical position innovation test ratio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float mag_ratio_GET()//Magnetometer innovation test ratio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void mag_ratio_SET(float  src) //Magnetometer innovation test ratio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float hagl_ratio_GET()//Height above terrain innovation test ratio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void hagl_ratio_SET(float  src) //Height above terrain innovation test ratio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float tas_ratio_GET()//True airspeed innovation test ratio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void tas_ratio_SET(float  src) //True airspeed innovation test ratio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float pos_horiz_accuracy_GET()//Horizontal position 1-STD accuracy relative to the EKF local origin (m)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void pos_horiz_accuracy_SET(float  src) //Horizontal position 1-STD accuracy relative to the EKF local origin (m)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float pos_vert_accuracy_GET()//Vertical position 1-STD accuracy relative to the EKF local origin (m)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void pos_vert_accuracy_SET(float  src) //Vertical position 1-STD accuracy relative to the EKF local origin (m)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public @ESTIMATOR_STATUS_FLAGS int flags_GET()//Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS.
        {  return  1 + (int)get_bits(data, 320, 11); }
        public void flags_SET(@ESTIMATOR_STATUS_FLAGS int  src) //Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS.
        {  set_bits(- 1 +   src, 11, data, 320); }
        static final Meta meta = new Meta(230, 0, 0, 1, 42, 331);
    } public static class WIND_COV extends Pack  implements CommunicationChannel.Sendable
    {

        WIND_COV() { super(meta, 0); }
        WIND_COV(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float wind_x_GET()//Wind in X (NED) direction in m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void wind_x_SET(float  src) //Wind in X (NED) direction in m/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float wind_y_GET()//Wind in Y (NED) direction in m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void wind_y_SET(float  src) //Wind in Y (NED) direction in m/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float wind_z_GET()//Wind in Z (NED) direction in m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void wind_z_SET(float  src) //Wind in Z (NED) direction in m/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float var_horiz_GET()//Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void var_horiz_SET(float  src) //Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float var_vert_GET()//Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void var_vert_SET(float  src) //Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float wind_alt_GET()//AMSL altitude (m) this measurement was taken at
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void wind_alt_SET(float  src) //AMSL altitude (m) this measurement was taken at
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float horiz_accuracy_GET()//Horizontal speed 1-STD accuracy
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void horiz_accuracy_SET(float  src) //Horizontal speed 1-STD accuracy
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float vert_accuracy_GET()//Vertical speed 1-STD accuracy
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void vert_accuracy_SET(float  src) //Vertical speed 1-STD accuracy
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        static final Meta meta = new Meta(231, 0, 0, 1, 40, 320);
    }/**
*GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position
*	 estimate of the sytem*/
    public static class GPS_INPUT extends Pack  implements CommunicationChannel.Sendable
    {

        GPS_INPUT() { super(meta, 0); }
        GPS_INPUT(int bytes) { super(meta, bytes); }
        public char time_week_GET()//GPS week number
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void time_week_SET(char  src) //GPS week number
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long time_week_ms_GET()//GPS time (milliseconds from start of GPS week)
        {  return (get_bytes(data,  2, 4)); }
        public void time_week_ms_SET(long  src) //GPS time (milliseconds from start of GPS week)
        {  set_bytes((src) & -1L, 4, data,  2); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  6, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch)
        {  set_bytes((src) & -1L, 8, data,  6); }
        public char gps_id_GET()//ID of the GPS for multiple GPS inputs
        {  return (char)((char) get_bytes(data,  14, 1)); }
        public void gps_id_SET(char  src) //ID of the GPS for multiple GPS inputs
        {  set_bytes((char)(src) & -1L, 1, data,  14); }
        public char fix_type_GET()//0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        {  return (char)((char) get_bytes(data,  15, 1)); }
        public void fix_type_SET(char  src) //0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        {  set_bytes((char)(src) & -1L, 1, data,  15); }
        public int lat_GET()//Latitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  16, 4)); }
        public void lat_SET(int  src) //Latitude (WGS84), in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  16); }
        public int lon_GET()//Longitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  20, 4)); }
        public void lon_SET(int  src) //Longitude (WGS84), in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  20); }
        public float alt_GET()//Altitude (AMSL, not WGS84), in m (positive for up)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void alt_SET(float  src) //Altitude (AMSL, not WGS84), in m (positive for up)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float hdop_GET()//GPS HDOP horizontal dilution of position in m
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void hdop_SET(float  src) //GPS HDOP horizontal dilution of position in m
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float vdop_GET()//GPS VDOP vertical dilution of position in m
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void vdop_SET(float  src) //GPS VDOP vertical dilution of position in m
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float vn_GET()//GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void vn_SET(float  src) //GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public float ve_GET()//GPS velocity in m/s in EAST direction in earth-fixed NED frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public void ve_SET(float  src) //GPS velocity in m/s in EAST direction in earth-fixed NED frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public float vd_GET()//GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public void vd_SET(float  src) //GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }
        public float speed_accuracy_GET()//GPS speed accuracy in m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  48, 4))); }
        public void speed_accuracy_SET(float  src) //GPS speed accuracy in m/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 48); }
        public float horiz_accuracy_GET()//GPS horizontal accuracy in m
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  52, 4))); }
        public void horiz_accuracy_SET(float  src) //GPS horizontal accuracy in m
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 52); }
        public float vert_accuracy_GET()//GPS vertical accuracy in m
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  56, 4))); }
        public void vert_accuracy_SET(float  src) //GPS vertical accuracy in m
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 56); }
        public char satellites_visible_GET()//Number of satellites visible.
        {  return (char)((char) get_bytes(data,  60, 1)); }
        public void satellites_visible_SET(char  src) //Number of satellites visible.
        {  set_bytes((char)(src) & -1L, 1, data,  60); }
        public @GPS_INPUT_IGNORE_FLAGS int ignore_flags_GET()//Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided
        {  return  1 + (int)get_bits(data, 488, 8); }
        public void ignore_flags_SET(@GPS_INPUT_IGNORE_FLAGS int  src) //Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided
        {  set_bits(- 1 +   src, 8, data, 488); }
        static final Meta meta = new Meta(232, 1, 1, 1, 62, 496);
    }/**
*RTCM message for injecting into the onboard GPS (used for DGPS)*/
    public static class GPS_RTCM_DATA extends Pack  implements CommunicationChannel.Sendable
    {

        GPS_RTCM_DATA() { super(meta, 0); }
        GPS_RTCM_DATA(int bytes) { super(meta, bytes); }
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
        /**
        *LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for
        *	 the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
        *	 on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
        *	 while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
        *	 fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
        *	 with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
        *	 corrupt RTCM data, and to recover from a unreliable transport delivery order*/
        public void flags_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char len_GET()//data length
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void len_SET(char  src) //data length
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public char[] data__GET(char[]  dst_ch, int pos)  //RTCM message (may be fragmented)
        {
            for(int BYTE = 2, dst_max = pos + 180; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//RTCM message (may be fragmented)
        {return data__GET(new char[180], 0);} public void data__SET(char[]  src, int pos)  //RTCM message (may be fragmented)
        {
            for(int BYTE =  2, src_max = pos + 180; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(233, 0, 0, 0, 182, 1456);
    }/**
*Message appropriate for high latency connections like Iridium*/
    public static class HIGH_LATENCY extends Pack  implements CommunicationChannel.Sendable
    {

        HIGH_LATENCY() { super(meta, 0); }
        HIGH_LATENCY(int bytes) { super(meta, bytes); }
        public char heading_GET()//heading (centidegrees)
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void heading_SET(char  src) //heading (centidegrees)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char wp_distance_GET()//distance to target (meters)
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void wp_distance_SET(char  src) //distance to target (meters)
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public long custom_mode_GET()//A bitfield for use for autopilot-specific flags.
        {  return (get_bytes(data,  4, 4)); }
        public void custom_mode_SET(long  src) //A bitfield for use for autopilot-specific flags.
        {  set_bytes((src) & -1L, 4, data,  4); }
        public short roll_GET()//roll (centidegrees)
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public void roll_SET(short  src) //roll (centidegrees)
        {  set_bytes((short)(src) & -1L, 2, data,  8); }
        public short pitch_GET()//pitch (centidegrees)
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public void pitch_SET(short  src) //pitch (centidegrees)
        {  set_bytes((short)(src) & -1L, 2, data,  10); }
        public byte throttle_GET()//throttle (percentage)
        {  return (byte)((byte) get_bytes(data,  12, 1)); }
        public void throttle_SET(byte  src) //throttle (percentage)
        {  set_bytes((byte)(src) & -1L, 1, data,  12); }
        public short heading_sp_GET()//heading setpoint (centidegrees)
        {  return (short)((short) get_bytes(data,  13, 2)); }
        public void heading_sp_SET(short  src) //heading setpoint (centidegrees)
        {  set_bytes((short)(src) & -1L, 2, data,  13); }
        public int latitude_GET()//Latitude, expressed as degrees * 1E7
        {  return (int)((int) get_bytes(data,  15, 4)); }
        public void latitude_SET(int  src) //Latitude, expressed as degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  15); }
        public int longitude_GET()//Longitude, expressed as degrees * 1E7
        {  return (int)((int) get_bytes(data,  19, 4)); }
        public void longitude_SET(int  src) //Longitude, expressed as degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  19); }
        public short altitude_amsl_GET()//Altitude above mean sea level (meters)
        {  return (short)((short) get_bytes(data,  23, 2)); }
        public void altitude_amsl_SET(short  src) //Altitude above mean sea level (meters)
        {  set_bytes((short)(src) & -1L, 2, data,  23); }
        public short altitude_sp_GET()//Altitude setpoint relative to the home position (meters)
        {  return (short)((short) get_bytes(data,  25, 2)); }
        public void altitude_sp_SET(short  src) //Altitude setpoint relative to the home position (meters)
        {  set_bytes((short)(src) & -1L, 2, data,  25); }
        public char airspeed_GET()//airspeed (m/s)
        {  return (char)((char) get_bytes(data,  27, 1)); }
        public void airspeed_SET(char  src) //airspeed (m/s)
        {  set_bytes((char)(src) & -1L, 1, data,  27); }
        public char airspeed_sp_GET()//airspeed setpoint (m/s)
        {  return (char)((char) get_bytes(data,  28, 1)); }
        public void airspeed_sp_SET(char  src) //airspeed setpoint (m/s)
        {  set_bytes((char)(src) & -1L, 1, data,  28); }
        public char groundspeed_GET()//groundspeed (m/s)
        {  return (char)((char) get_bytes(data,  29, 1)); }
        public void groundspeed_SET(char  src) //groundspeed (m/s)
        {  set_bytes((char)(src) & -1L, 1, data,  29); }
        public byte climb_rate_GET()//climb rate (m/s)
        {  return (byte)((byte) get_bytes(data,  30, 1)); }
        public void climb_rate_SET(byte  src) //climb rate (m/s)
        {  set_bytes((byte)(src) & -1L, 1, data,  30); }
        public char gps_nsat_GET()//Number of satellites visible. If unknown, set to 255
        {  return (char)((char) get_bytes(data,  31, 1)); }
        public void gps_nsat_SET(char  src) //Number of satellites visible. If unknown, set to 255
        {  set_bytes((char)(src) & -1L, 1, data,  31); }
        public char battery_remaining_GET()//Remaining battery (percentage)
        {  return (char)((char) get_bytes(data,  32, 1)); }
        public void battery_remaining_SET(char  src) //Remaining battery (percentage)
        {  set_bytes((char)(src) & -1L, 1, data,  32); }
        public byte temperature_GET()//Autopilot temperature (degrees C)
        {  return (byte)((byte) get_bytes(data,  33, 1)); }
        public void temperature_SET(byte  src) //Autopilot temperature (degrees C)
        {  set_bytes((byte)(src) & -1L, 1, data,  33); }
        public byte temperature_air_GET()//Air temperature (degrees C) from airspeed sensor
        {  return (byte)((byte) get_bytes(data,  34, 1)); }
        public void temperature_air_SET(byte  src) //Air temperature (degrees C) from airspeed sensor
        {  set_bytes((byte)(src) & -1L, 1, data,  34); }
        /**
        *failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS,
        *	 bit3:GCS, bit4:fence*/
        public char failsafe_GET()
        {  return (char)((char) get_bytes(data,  35, 1)); }
        /**
        *failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS,
        *	 bit3:GCS, bit4:fence*/
        public void failsafe_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  35); }
        public char wp_num_GET()//current waypoint number
        {  return (char)((char) get_bytes(data,  36, 1)); }
        public void wp_num_SET(char  src) //current waypoint number
        {  set_bytes((char)(src) & -1L, 1, data,  36); }
        public @MAV_MODE_FLAG int base_mode_GET()//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
        {  return  1 + (int)get_bits(data, 296, 8); }
        public void base_mode_SET(@MAV_MODE_FLAG int  src) //System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
        {  set_bits(- 1 +   src, 8, data, 296); }
        public @MAV_LANDED_STATE int landed_state_GET()//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
        {  return  0 + (int)get_bits(data, 304, 3); }
        public void landed_state_SET(@MAV_LANDED_STATE int  src) //The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
        {  set_bits(- 0 +   src, 3, data, 304); }
        public @GPS_FIX_TYPE int gps_fix_type_GET()//See the GPS_FIX_TYPE enum.
        {  return  0 + (int)get_bits(data, 307, 4); }
        public void gps_fix_type_SET(@GPS_FIX_TYPE int  src) //See the GPS_FIX_TYPE enum.
        {  set_bits(- 0 +   src, 4, data, 307); }
        static final Meta meta = new Meta(234, 2, 1, 0, 39, 311);
    }/**
*Vibration levels and accelerometer clipping*/
    public static class VIBRATION extends Pack  implements CommunicationChannel.Sendable
    {

        VIBRATION() { super(meta, 0); }
        VIBRATION(int bytes) { super(meta, bytes); }
        public long clipping_0_GET()//first accelerometer clipping count
        {  return (get_bytes(data,  0, 4)); }
        public void clipping_0_SET(long  src) //first accelerometer clipping count
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long clipping_1_GET()//second accelerometer clipping count
        {  return (get_bytes(data,  4, 4)); }
        public void clipping_1_SET(long  src) //second accelerometer clipping count
        {  set_bytes((src) & -1L, 4, data,  4); }
        public long clipping_2_GET()//third accelerometer clipping count
        {  return (get_bytes(data,  8, 4)); }
        public void clipping_2_SET(long  src) //third accelerometer clipping count
        {  set_bytes((src) & -1L, 4, data,  8); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch)
        {  return (get_bytes(data,  12, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch)
        {  set_bytes((src) & -1L, 8, data,  12); }
        public float vibration_x_GET()//Vibration levels on X-axis
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void vibration_x_SET(float  src) //Vibration levels on X-axis
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float vibration_y_GET()//Vibration levels on Y-axis
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void vibration_y_SET(float  src) //Vibration levels on Y-axis
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float vibration_z_GET()//Vibration levels on Z-axis
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void vibration_z_SET(float  src) //Vibration levels on Z-axis
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        static final Meta meta = new Meta(241, 0, 3, 1, 32, 256);
    }/**
*This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system
*	 will return to and land on. The position is set automatically by the system during the takeoff in case
*	 it was not explicitely set by the operator before or after. The position the system will return to and
*	 land on. The global and local positions encode the position in the respective coordinate frames, while
*	 the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading
*	 and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes
*	 the point to which the system should fly in normal flight mode and then perform a landing sequence along
*	 the vector*/
    public static class HOME_POSITION extends Pack  implements CommunicationChannel.Sendable
    {

        HOME_POSITION() { super(meta, 0); }
        HOME_POSITION(int bytes) { super(meta, bytes); }
        public int latitude_GET()//Latitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  0, 4)); }
        public void latitude_SET(int  src) //Latitude (WGS84), in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  0); }
        public int longitude_GET()//Longitude (WGS84, in degrees * 1E7
        {  return (int)((int) get_bytes(data,  4, 4)); }
        public void longitude_SET(int  src) //Longitude (WGS84, in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  4); }
        public int altitude_GET()//Altitude (AMSL), in meters * 1000 (positive for up)
        {  return (int)((int) get_bytes(data,  8, 4)); }
        public void altitude_SET(int  src) //Altitude (AMSL), in meters * 1000 (positive for up)
        {  set_bytes((int)(src) & -1L, 4, data,  8); }
        public float x_GET()//Local X position of this position in the local coordinate frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void x_SET(float  src) //Local X position of this position in the local coordinate frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float y_GET()//Local Y position of this position in the local coordinate frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void y_SET(float  src) //Local Y position of this position in the local coordinate frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float z_GET()//Local Z position of this position in the local coordinate frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void z_SET(float  src) //Local Z position of this position in the local coordinate frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
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
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*	 and slope of the groun*/
        public void q_SET(float[]  src, int pos)
        {
            for(int BYTE =  24, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        /**
        *Local X position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
        public float approach_x_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        /**
        *Local X position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
        public void approach_x_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        /**
        *Local Y position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
        public float approach_y_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        /**
        *Local Y position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
        public void approach_y_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }
        /**
        *Local Z position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
        public float approach_z_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  48, 4))); }
        /**
        *Local Z position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
        public void approach_z_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 48); }
        public long  time_usec_TRY(Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {
            if(ph.field_bit !=  416 && !try_visit_field(ph, 416)) return 0;
            return (get_bytes(data,  ph.BYTE, 8));
        }
        public void time_usec_SET(long  src, Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {
            if(ph.field_bit != 416)insert_field(ph, 416, 0);
            set_bytes((src) & -1L, 8, data,  ph.BYTE);
        } static final Meta meta = new Meta(242, 0, 0, 0, 53, 416, 0, _ob);
    }/**
*The position the system will return to and land on. The position is set automatically by the system during
*	 the takeoff in case it was not explicitely set by the operator before or after. The global and local
*	 positions encode the position in the respective coordinate frames, while the q parameter encodes the
*	 orientation of the surface. Under normal conditions it describes the heading and terrain slope, which
*	 can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which
*	 the system should fly in normal flight mode and then perform a landing sequence along the vector*/
    public static class SET_HOME_POSITION extends Pack  implements CommunicationChannel.Sendable
    {

        SET_HOME_POSITION() { super(meta, 0); }
        SET_HOME_POSITION(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID.
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System ID.
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public int latitude_GET()//Latitude (WGS84), in degrees * 1E7
        {  return (int)((int) get_bytes(data,  1, 4)); }
        public void latitude_SET(int  src) //Latitude (WGS84), in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  1); }
        public int longitude_GET()//Longitude (WGS84, in degrees * 1E7
        {  return (int)((int) get_bytes(data,  5, 4)); }
        public void longitude_SET(int  src) //Longitude (WGS84, in degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  5); }
        public int altitude_GET()//Altitude (AMSL), in meters * 1000 (positive for up)
        {  return (int)((int) get_bytes(data,  9, 4)); }
        public void altitude_SET(int  src) //Altitude (AMSL), in meters * 1000 (positive for up)
        {  set_bytes((int)(src) & -1L, 4, data,  9); }
        public float x_GET()//Local X position of this position in the local coordinate frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  13, 4))); }
        public void x_SET(float  src) //Local X position of this position in the local coordinate frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 13); }
        public float y_GET()//Local Y position of this position in the local coordinate frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  17, 4))); }
        public void y_SET(float  src) //Local Y position of this position in the local coordinate frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 17); }
        public float z_GET()//Local Z position of this position in the local coordinate frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  21, 4))); }
        public void z_SET(float  src) //Local Z position of this position in the local coordinate frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 21); }
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
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*	 and slope of the groun*/
        public void q_SET(float[]  src, int pos)
        {
            for(int BYTE =  25, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        /**
        *Local X position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
        public float approach_x_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  41, 4))); }
        /**
        *Local X position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
        public void approach_x_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 41); }
        /**
        *Local Y position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
        public float approach_y_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  45, 4))); }
        /**
        *Local Y position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
        public void approach_y_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 45); }
        /**
        *Local Z position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
        public float approach_z_GET()
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  49, 4))); }
        /**
        *Local Z position of the end of the approach vector. Multicopters should set this position based on their
        *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
        *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
        *	 from the threshold / touchdown zone*/
        public void approach_z_SET(float  src)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 49); }
        public long  time_usec_TRY(Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {
            if(ph.field_bit !=  424 && !try_visit_field(ph, 424)) return 0;
            return (get_bytes(data,  ph.BYTE, 8));
        }
        public void time_usec_SET(long  src, Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {
            if(ph.field_bit != 424)insert_field(ph, 424, 0);
            set_bytes((src) & -1L, 8, data,  ph.BYTE);
        } static final Meta meta = new Meta(243, 0, 0, 0, 54, 424, 0, _OU);
    }/**
*This interface replaces DATA_STREAM*/
    public static class MESSAGE_INTERVAL extends Pack  implements CommunicationChannel.Sendable
    {

        MESSAGE_INTERVAL() { super(meta, 0); }
        MESSAGE_INTERVAL(int bytes) { super(meta, bytes); }
        public char message_id_GET()//The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void message_id_SET(char  src) //The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public int interval_us_GET()//0 indicates the interval at which it is sent.
        {  return (int)((int) get_bytes(data,  2, 4)); }
        public void interval_us_SET(int  src) //0 indicates the interval at which it is sent.
        {  set_bytes((int)(src) & -1L, 4, data,  2); }
        static final Meta meta = new Meta(244, 1, 0, 0, 6, 48);
    }/**
*Provides state for additional features*/
    public static class EXTENDED_SYS_STATE extends Pack  implements CommunicationChannel.Sendable
    {

        EXTENDED_SYS_STATE() { super(meta, 0); }
        EXTENDED_SYS_STATE(int bytes) { super(meta, bytes); }
        public @MAV_VTOL_STATE int vtol_state_GET()//The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration
        {  return  0 + (int)get_bits(data, 0, 3); }
        public void vtol_state_SET(@MAV_VTOL_STATE int  src) //The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration
        {  set_bits(- 0 +   src, 3, data, 0); }
        public @MAV_LANDED_STATE int landed_state_GET()//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
        {  return  0 + (int)get_bits(data, 3, 3); }
        public void landed_state_SET(@MAV_LANDED_STATE int  src) //The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
        {  set_bits(- 0 +   src, 3, data, 3); }
        static final Meta meta = new Meta(245, 0, 0, 0, 1, 6);
    }/**
*The location and information of an ADSB vehicle*/
    public static class ADSB_VEHICLE extends Pack  implements CommunicationChannel.Sendable
    {

        ADSB_VEHICLE() { super(meta, 0); }
        ADSB_VEHICLE(int bytes) { super(meta, bytes); }
        public char heading_GET()//Course over ground in centidegrees
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void heading_SET(char  src) //Course over ground in centidegrees
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char hor_velocity_GET()//The horizontal velocity in centimeters/second
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void hor_velocity_SET(char  src) //The horizontal velocity in centimeters/second
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char squawk_GET()//Squawk code
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void squawk_SET(char  src) //Squawk code
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public long ICAO_address_GET()//ICAO address
        {  return (get_bytes(data,  6, 4)); }
        public void ICAO_address_SET(long  src) //ICAO address
        {  set_bytes((src) & -1L, 4, data,  6); }
        public int lat_GET()//Latitude, expressed as degrees * 1E7
        {  return (int)((int) get_bytes(data,  10, 4)); }
        public void lat_SET(int  src) //Latitude, expressed as degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  10); }
        public int lon_GET()//Longitude, expressed as degrees * 1E7
        {  return (int)((int) get_bytes(data,  14, 4)); }
        public void lon_SET(int  src) //Longitude, expressed as degrees * 1E7
        {  set_bytes((int)(src) & -1L, 4, data,  14); }
        public int altitude_GET()//Altitude(ASL) in millimeters
        {  return (int)((int) get_bytes(data,  18, 4)); }
        public void altitude_SET(int  src) //Altitude(ASL) in millimeters
        {  set_bytes((int)(src) & -1L, 4, data,  18); }
        public short ver_velocity_GET()//The vertical velocity in centimeters/second, positive is up
        {  return (short)((short) get_bytes(data,  22, 2)); }
        public void ver_velocity_SET(short  src) //The vertical velocity in centimeters/second, positive is up
        {  set_bytes((short)(src) & -1L, 2, data,  22); }
        public char tslc_GET()//Time since last communication in seconds
        {  return (char)((char) get_bytes(data,  24, 1)); }
        public void tslc_SET(char  src) //Time since last communication in seconds
        {  set_bytes((char)(src) & -1L, 1, data,  24); }
        public @ADSB_ALTITUDE_TYPE int altitude_type_GET()//Type from ADSB_ALTITUDE_TYPE enum
        {  return  0 + (int)get_bits(data, 200, 1); }
        public void altitude_type_SET(@ADSB_ALTITUDE_TYPE int  src) //Type from ADSB_ALTITUDE_TYPE enum
        {  set_bits(- 0 +   src, 1, data, 200); }
        public @ADSB_EMITTER_TYPE int emitter_type_GET()//Type from ADSB_EMITTER_TYPE enum
        {  return  0 + (int)get_bits(data, 201, 5); }
        public void emitter_type_SET(@ADSB_EMITTER_TYPE int  src) //Type from ADSB_EMITTER_TYPE enum
        {  set_bits(- 0 +   src, 5, data, 201); }
        public @ADSB_FLAGS int flags_GET()//Flags to indicate various statuses including valid data fields
        {  return  1 + (int)get_bits(data, 206, 7); }
        public void flags_SET(@ADSB_FLAGS int  src) //Flags to indicate various statuses including valid data fields
        {  set_bits(- 1 +   src, 7, data, 206); }
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
        } public void callsign_SET(String src, Bounds.Inside ph) //The callsign, 8+null
        {callsign_SET(src.toCharArray(), 0, src.length(), ph);} public void callsign_SET(char[]  src, int pos, int items, Bounds.Inside ph) //The callsign, 8+null
        {
            if(ph.field_bit != 213 && insert_field(ph, 213, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(246, 3, 1, 0, 28, 213, 0, _xU);
    }/**
*Information about a potential collision*/
    public static class COLLISION extends Pack  implements CommunicationChannel.Sendable
    {

        COLLISION() { super(meta, 0); }
        COLLISION(int bytes) { super(meta, bytes); }
        public long id_GET()//Unique identifier, domain based on src field
        {  return (get_bytes(data,  0, 4)); }
        public void id_SET(long  src) //Unique identifier, domain based on src field
        {  set_bytes((src) & -1L, 4, data,  0); }
        public float time_to_minimum_delta_GET()//Estimated time until collision occurs (seconds)
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void time_to_minimum_delta_SET(float  src) //Estimated time until collision occurs (seconds)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float altitude_minimum_delta_GET()//Closest vertical distance in meters between vehicle and object
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void altitude_minimum_delta_SET(float  src) //Closest vertical distance in meters between vehicle and object
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float horizontal_minimum_delta_GET()//Closest horizontal distance in meteres between vehicle and object
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void horizontal_minimum_delta_SET(float  src) //Closest horizontal distance in meteres between vehicle and object
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public @MAV_COLLISION_SRC int src__GET()//Collision data source
        {  return  0 + (int)get_bits(data, 128, 1); }
        public void src__SET(@MAV_COLLISION_SRC int  src) //Collision data source
        {  set_bits(- 0 +   src, 1, data, 128); }
        public @MAV_COLLISION_ACTION int action_GET()//Action that is being taken to avoid this collision
        {  return  0 + (int)get_bits(data, 129, 3); }
        public void action_SET(@MAV_COLLISION_ACTION int  src) //Action that is being taken to avoid this collision
        {  set_bits(- 0 +   src, 3, data, 129); }
        public @MAV_COLLISION_THREAT_LEVEL int threat_level_GET()//How concerned the aircraft is about this collision
        {  return  0 + (int)get_bits(data, 132, 3); }
        public void threat_level_SET(@MAV_COLLISION_THREAT_LEVEL int  src) //How concerned the aircraft is about this collision
        {  set_bits(- 0 +   src, 3, data, 132); }
        static final Meta meta = new Meta(247, 0, 1, 0, 17, 135);
    }/**
*Message implementing parts of the V2 payload specs in V1 frames for transitional support.*/
    public static class V2_EXTENSION extends Pack  implements CommunicationChannel.Sendable
    {

        V2_EXTENSION() { super(meta, 0); }
        V2_EXTENSION(int bytes) { super(meta, bytes); }
        /**
        *A code that identifies the software component that understands this message (analogous to usb device classes
        *	 or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
        *	 and the corresponding entry should be added to https:github.com/mavlink/mavlink/extension-message-ids.xml.
        *	 Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
        *	 Message_types greater than 32767 are considered local experiments and should not be checked in to any
        *	 widely distributed codebase*/
        public char message_type_GET()
        {  return (char)((char) get_bytes(data,  0, 2)); }
        /**
        *A code that identifies the software component that understands this message (analogous to usb device classes
        *	 or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
        *	 and the corresponding entry should be added to https:github.com/mavlink/mavlink/extension-message-ids.xml.
        *	 Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
        *	 Message_types greater than 32767 are considered local experiments and should not be checked in to any
        *	 widely distributed codebase*/
        public void message_type_SET(char  src)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char target_network_GET()//Network ID (0 for broadcast)
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void target_network_SET(char  src) //Network ID (0 for broadcast)
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char target_system_GET()//System ID (0 for broadcast)
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public void target_system_SET(char  src) //System ID (0 for broadcast)
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public char target_component_GET()//Component ID (0 for broadcast)
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void target_component_SET(char  src) //Component ID (0 for broadcast)
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
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
        {return payload_GET(new char[249], 0);}/**
*Variable length payload. The length is defined by the remaining message length when subtracting the header
*	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
*	 message_type.  The particular encoding used can be extension specific and might not always be documented
*	 as part of the mavlink specification*/
        public void payload_SET(char[]  src, int pos)
        {
            for(int BYTE =  5, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(248, 1, 0, 0, 254, 2032);
    }/**
*Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient
*	 way for testing new messages and getting experimental debug output*/
    public static class MEMORY_VECT extends Pack  implements CommunicationChannel.Sendable
    {

        MEMORY_VECT() { super(meta, 0); }
        MEMORY_VECT(int bytes) { super(meta, bytes); }
        public char address_GET()//Starting address of the debug variables
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void address_SET(char  src) //Starting address of the debug variables
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char ver_GET()//Version code of the type variable. 0=unknown, type ignored and assumed short. 1=as below
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void ver_SET(char  src) //Version code of the type variable. 0=unknown, type ignored and assumed short. 1=as below
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char type_GET()//Type code of the memory variables. for ver = 1: 0=16 x short, 1=16 x char, 2=16 x Q15, 3=16 x 1Q1
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public void type_SET(char  src) //Type code of the memory variables. for ver = 1: 0=16 x short, 1=16 x char, 2=16 x Q15, 3=16 x 1Q1
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public byte[] value_GET(byte[]  dst_ch, int pos)  //Memory contents at specified address
        {
            for(int BYTE = 4, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (byte)((byte) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public byte[] value_GET()//Memory contents at specified address
        {return value_GET(new byte[32], 0);} public void value_SET(byte[]  src, int pos)  //Memory contents at specified address
        {
            for(int BYTE =  4, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                set_bytes((byte)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(249, 1, 0, 0, 36, 288);
    } public static class DEBUG_VECT extends Pack  implements CommunicationChannel.Sendable
    {

        DEBUG_VECT() { super(meta, 0); }
        DEBUG_VECT(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float x_GET()//x
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void x_SET(float  src) //x
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float y_GET()//y
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void y_SET(float  src) //y
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float z_GET()//z
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void z_SET(float  src) //z
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
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
        } public void name_SET(String src, Bounds.Inside ph) //Name
        {name_SET(src.toCharArray(), 0, src.length(), ph);} public void name_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Name
        {
            if(ph.field_bit != 160 && insert_field(ph, 160, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(250, 0, 0, 1, 21, 160, 0, _FU);
    }/**
*Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite
*	 efficient way for testing new messages and getting experimental debug output*/
    public static class NAMED_VALUE_FLOAT extends Pack  implements CommunicationChannel.Sendable
    {

        NAMED_VALUE_FLOAT() { super(meta, 0); }
        NAMED_VALUE_FLOAT(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public float value_GET()//Floating point value
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void value_SET(float  src) //Floating point value
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
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
        } public void name_SET(String src, Bounds.Inside ph) //Name of the debug variable
        {name_SET(src.toCharArray(), 0, src.length(), ph);} public void name_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Name of the debug variable
        {
            if(ph.field_bit != 64 && insert_field(ph, 64, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(251, 0, 1, 0, 9, 64, 0, _oU);
    }/**
*Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite
*	 efficient way for testing new messages and getting experimental debug output*/
    public static class NAMED_VALUE_INT extends Pack  implements CommunicationChannel.Sendable
    {

        NAMED_VALUE_INT() { super(meta, 0); }
        NAMED_VALUE_INT(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public int value_GET()//Signed integer value
        {  return (int)((int) get_bytes(data,  4, 4)); }
        public void value_SET(int  src) //Signed integer value
        {  set_bytes((int)(src) & -1L, 4, data,  4); }
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
        } public void name_SET(String src, Bounds.Inside ph) //Name of the debug variable
        {name_SET(src.toCharArray(), 0, src.length(), ph);} public void name_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Name of the debug variable
        {
            if(ph.field_bit != 64 && insert_field(ph, 64, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(252, 0, 1, 0, 9, 64, 0, _TD);
    }/**
*Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING:
*	 They consume quite some bandwidth, so use only for important status and error messages. If implemented
*	 wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz)*/
    public static class STATUSTEXT extends Pack  implements CommunicationChannel.Sendable
    {

        STATUSTEXT() { super(meta, 0); }
        STATUSTEXT(int bytes) { super(meta, bytes); }
        public @MAV_SEVERITY int severity_GET()//Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
        {  return  0 + (int)get_bits(data, 0, 3); }
        public void severity_SET(@MAV_SEVERITY int  src) //Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
        {  set_bits(- 0 +   src, 3, data, 0); }
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
        } public void text_SET(String src, Bounds.Inside ph) //Status text message, without null termination character
        {text_SET(src.toCharArray(), 0, src.length(), ph);} public void text_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Status text message, without null termination character
        {
            if(ph.field_bit != 3 && insert_field(ph, 3, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(253, 0, 0, 0, 2, 3, 0, _BD);
    }/**
*Send a debug value. The index is used to discriminate between values. These values show up in the plot
*	 of QGroundControl as DEBUG N*/
    public static class DEBUG extends Pack  implements CommunicationChannel.Sendable
    {

        DEBUG() { super(meta, 0); }
        DEBUG(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public char ind_GET()//index of debug variable
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void ind_SET(char  src) //index of debug variable
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public float value_GET()//DEBUG value
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  5, 4))); }
        public void value_SET(float  src) //DEBUG value
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 5); }
        static final Meta meta = new Meta(254, 0, 1, 0, 9, 72);
    }/**
*Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will disable
*	 signin*/
    public static class SETUP_SIGNING extends Pack  implements CommunicationChannel.Sendable
    {

        SETUP_SIGNING() { super(meta, 0); }
        SETUP_SIGNING(int bytes) { super(meta, bytes); }
        public long initial_timestamp_GET()//initial timestamp
        {  return (get_bytes(data,  0, 8)); }
        public void initial_timestamp_SET(long  src) //initial timestamp
        {  set_bytes((src) & -1L, 8, data,  0); }
        public char target_system_GET()//system id of the target
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public void target_system_SET(char  src) //system id of the target
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        public char target_component_GET()//component ID of the target
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public void target_component_SET(char  src) //component ID of the target
        {  set_bytes((char)(src) & -1L, 1, data,  9); }
        public char[] secret_key_GET(char[]  dst_ch, int pos)  //signing key
        {
            for(int BYTE = 10, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] secret_key_GET()//signing key
        {return secret_key_GET(new char[32], 0);} public void secret_key_SET(char[]  src, int pos)  //signing key
        {
            for(int BYTE =  10, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(256, 0, 0, 1, 42, 336);
    }/**
*Report button state change*/
    public static class BUTTON_CHANGE extends Pack  implements CommunicationChannel.Sendable
    {

        BUTTON_CHANGE() { super(meta, 0); }
        BUTTON_CHANGE(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long last_change_ms_GET()//Time of last change of button state
        {  return (get_bytes(data,  4, 4)); }
        public void last_change_ms_SET(long  src) //Time of last change of button state
        {  set_bytes((src) & -1L, 4, data,  4); }
        public char state_GET()//Bitmap state of buttons
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public void state_SET(char  src) //Bitmap state of buttons
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        static final Meta meta = new Meta(257, 0, 2, 0, 9, 72);
    }/**
*Control vehicle tone generation (buzzer)*/
    public static class PLAY_TUNE extends Pack  implements CommunicationChannel.Sendable
    {

        PLAY_TUNE() { super(meta, 0); }
        PLAY_TUNE(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component ID
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
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
        } public void tune_SET(String src, Bounds.Inside ph) //tune in board specific format
        {tune_SET(src.toCharArray(), 0, src.length(), ph);} public void tune_SET(char[]  src, int pos, int items, Bounds.Inside ph) //tune in board specific format
        {
            if(ph.field_bit != 16 && insert_field(ph, 16, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(258, 0, 0, 0, 3, 16, 0, _aD);
    }/**
*WIP: Information about a camera*/
    public static class CAMERA_INFORMATION extends Pack  implements CommunicationChannel.Sendable
    {

        CAMERA_INFORMATION() { super(meta, 0); }
        CAMERA_INFORMATION(int bytes) { super(meta, bytes); }
        public char resolution_h_GET()//Image resolution in pixels horizontal
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void resolution_h_SET(char  src) //Image resolution in pixels horizontal
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char resolution_v_GET()//Image resolution in pixels vertical
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void resolution_v_SET(char  src) //Image resolution in pixels vertical
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char cam_definition_version_GET()//Camera definition version (iteration)
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void cam_definition_version_SET(char  src) //Camera definition version (iteration)
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  6, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  6); }
        public long firmware_version_GET()//0xff = Major)
        {  return (get_bytes(data,  10, 4)); }
        public void firmware_version_SET(long  src) //0xff = Major)
        {  set_bytes((src) & -1L, 4, data,  10); }
        public char[] vendor_name_GET(char[]  dst_ch, int pos)  //Name of the camera vendor
        {
            for(int BYTE = 14, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] vendor_name_GET()//Name of the camera vendor
        {return vendor_name_GET(new char[32], 0);} public void vendor_name_SET(char[]  src, int pos)  //Name of the camera vendor
        {
            for(int BYTE =  14, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public char[] model_name_GET(char[]  dst_ch, int pos)  //Name of the camera model
        {
            for(int BYTE = 46, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] model_name_GET()//Name of the camera model
        {return model_name_GET(new char[32], 0);} public void model_name_SET(char[]  src, int pos)  //Name of the camera model
        {
            for(int BYTE =  46, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public float focal_length_GET()//Focal length in mm
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  78, 4))); }
        public void focal_length_SET(float  src) //Focal length in mm
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 78); }
        public float sensor_size_h_GET()//Image sensor size horizontal in mm
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  82, 4))); }
        public void sensor_size_h_SET(float  src) //Image sensor size horizontal in mm
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 82); }
        public float sensor_size_v_GET()//Image sensor size vertical in mm
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  86, 4))); }
        public void sensor_size_v_SET(float  src) //Image sensor size vertical in mm
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 86); }
        public char lens_id_GET()//Reserved for a lens ID
        {  return (char)((char) get_bytes(data,  90, 1)); }
        public void lens_id_SET(char  src) //Reserved for a lens ID
        {  set_bytes((char)(src) & -1L, 1, data,  90); }
        public @CAMERA_CAP_FLAGS int flags_GET()//CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities.
        {  return  1 + (int)get_bits(data, 728, 6); }
        public void flags_SET(@CAMERA_CAP_FLAGS int  src) //CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities.
        {  set_bits(- 1 +   src, 6, data, 728); }
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
        } public void cam_definition_uri_SET(String src, Bounds.Inside ph) //Camera definition URI (if any, otherwise only basic functions will be available).
        {cam_definition_uri_SET(src.toCharArray(), 0, src.length(), ph);} public void cam_definition_uri_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Camera definition URI (if any, otherwise only basic functions will be available).
        {
            if(ph.field_bit != 734 && insert_field(ph, 734, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(259, 3, 2, 0, 93, 734, 0, _QD);
    }/**
*WIP: Settings of a camera, can be requested using MAV_CMD_REQUEST_CAMERA_SETTINGS.*/
    public static class CAMERA_SETTINGS extends Pack  implements CommunicationChannel.Sendable
    {

        CAMERA_SETTINGS() { super(meta, 0); }
        CAMERA_SETTINGS(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public @CAMERA_MODE int mode_id_GET()//Camera mode (CAMERA_MODE)
        {  return  0 + (int)get_bits(data, 32, 3); }
        public void mode_id_SET(@CAMERA_MODE int  src) //Camera mode (CAMERA_MODE)
        {  set_bits(- 0 +   src, 3, data, 32); }
        static final Meta meta = new Meta(260, 0, 1, 0, 5, 35);
    }/**
*WIP: Information about a storage medium.*/
    public static class STORAGE_INFORMATION extends Pack  implements CommunicationChannel.Sendable
    {

        STORAGE_INFORMATION() { super(meta, 0); }
        STORAGE_INFORMATION(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public char storage_id_GET()//Storage ID (1 for first, 2 for second, etc.)
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void storage_id_SET(char  src) //Storage ID (1 for first, 2 for second, etc.)
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public char storage_count_GET()//Number of storage devices
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public void storage_count_SET(char  src) //Number of storage devices
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public char status_GET()//Status of storage (0 not available, 1 unformatted, 2 formatted)
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public void status_SET(char  src) //Status of storage (0 not available, 1 unformatted, 2 formatted)
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public float total_capacity_GET()//Total capacity in MiB
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  7, 4))); }
        public void total_capacity_SET(float  src) //Total capacity in MiB
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 7); }
        public float used_capacity_GET()//Used capacity in MiB
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  11, 4))); }
        public void used_capacity_SET(float  src) //Used capacity in MiB
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 11); }
        public float available_capacity_GET()//Available capacity in MiB
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  15, 4))); }
        public void available_capacity_SET(float  src) //Available capacity in MiB
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 15); }
        public float read_speed_GET()//Read speed in MiB/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  19, 4))); }
        public void read_speed_SET(float  src) //Read speed in MiB/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 19); }
        public float write_speed_GET()//Write speed in MiB/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  23, 4))); }
        public void write_speed_SET(float  src) //Write speed in MiB/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 23); }
        static final Meta meta = new Meta(261, 0, 1, 0, 27, 216);
    }/**
*WIP: Information about the status of a capture*/
    public static class CAMERA_CAPTURE_STATUS extends Pack  implements CommunicationChannel.Sendable
    {

        CAMERA_CAPTURE_STATUS() { super(meta, 0); }
        CAMERA_CAPTURE_STATUS(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long recording_time_ms_GET()//Time in milliseconds since recording started
        {  return (get_bytes(data,  4, 4)); }
        public void recording_time_ms_SET(long  src) //Time in milliseconds since recording started
        {  set_bytes((src) & -1L, 4, data,  4); }
        /**
        *Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval
        *	 set and capture in progress*/
        public char image_status_GET()
        {  return (char)((char) get_bytes(data,  8, 1)); }
        /**
        *Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval
        *	 set and capture in progress*/
        public void image_status_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        public char video_status_GET()//Current status of video capturing (0: idle, 1: capture in progress)
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public void video_status_SET(char  src) //Current status of video capturing (0: idle, 1: capture in progress)
        {  set_bytes((char)(src) & -1L, 1, data,  9); }
        public float image_interval_GET()//Image capture interval in seconds
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public void image_interval_SET(float  src) //Image capture interval in seconds
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }
        public float available_capacity_GET()//Available storage capacity in MiB
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public void available_capacity_SET(float  src) //Available storage capacity in MiB
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        static final Meta meta = new Meta(262, 0, 2, 0, 18, 144);
    }/**
*Information about a captured image*/
    public static class CAMERA_IMAGE_CAPTURED extends Pack  implements CommunicationChannel.Sendable
    {

        CAMERA_IMAGE_CAPTURED() { super(meta, 0); }
        CAMERA_IMAGE_CAPTURED(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long time_utc_GET()//Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
        {  return (get_bytes(data,  4, 8)); }
        public void time_utc_SET(long  src) //Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
        {  set_bytes((src) & -1L, 8, data,  4); }
        public char camera_id_GET()//Camera ID (1 for first, 2 for second, etc.)
        {  return (char)((char) get_bytes(data,  12, 1)); }
        public void camera_id_SET(char  src) //Camera ID (1 for first, 2 for second, etc.)
        {  set_bytes((char)(src) & -1L, 1, data,  12); }
        public int lat_GET()//Latitude, expressed as degrees * 1E7 where image was taken
        {  return (int)((int) get_bytes(data,  13, 4)); }
        public void lat_SET(int  src) //Latitude, expressed as degrees * 1E7 where image was taken
        {  set_bytes((int)(src) & -1L, 4, data,  13); }
        public int lon_GET()//Longitude, expressed as degrees * 1E7 where capture was taken
        {  return (int)((int) get_bytes(data,  17, 4)); }
        public void lon_SET(int  src) //Longitude, expressed as degrees * 1E7 where capture was taken
        {  set_bytes((int)(src) & -1L, 4, data,  17); }
        public int alt_GET()//Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
        {  return (int)((int) get_bytes(data,  21, 4)); }
        public void alt_SET(int  src) //Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
        {  set_bytes((int)(src) & -1L, 4, data,  21); }
        public int relative_alt_GET()//Altitude above ground in meters, expressed as * 1E3 where image was taken
        {  return (int)((int) get_bytes(data,  25, 4)); }
        public void relative_alt_SET(int  src) //Altitude above ground in meters, expressed as * 1E3 where image was taken
        {  set_bytes((int)(src) & -1L, 4, data,  25); }
        public float[] q_GET(float[]  dst_ch, int pos)  //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
        {
            for(int BYTE = 29, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] q_GET()//Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
        {return q_GET(new float[4], 0);} public void q_SET(float[]  src, int pos)  //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
        {
            for(int BYTE =  29, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public int image_index_GET()//Zero based index of this image (image count since armed -1)
        {  return (int)((int) get_bytes(data,  45, 4)); }
        public void image_index_SET(int  src) //Zero based index of this image (image count since armed -1)
        {  set_bytes((int)(src) & -1L, 4, data,  45); }
        public byte capture_result_GET()//Boolean indicating success (1) or failure (0) while capturing this image.
        {  return (byte)((byte) get_bytes(data,  49, 1)); }
        public void capture_result_SET(byte  src) //Boolean indicating success (1) or failure (0) while capturing this image.
        {  set_bytes((byte)(src) & -1L, 1, data,  49); }
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
        } public void file_url_SET(String src, Bounds.Inside ph) //URL of image taken. Either local storage or http:foo.jpg if camera provides an HTTP interface.
        {file_url_SET(src.toCharArray(), 0, src.length(), ph);} public void file_url_SET(char[]  src, int pos, int items, Bounds.Inside ph) //URL of image taken. Either local storage or http:foo.jpg if camera provides an HTTP interface.
        {
            if(ph.field_bit != 402 && insert_field(ph, 402, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(263, 0, 1, 1, 51, 402, 2, _mI);
    }/**
*WIP: Information about flight since last arming*/
    public static class FLIGHT_INFORMATION extends Pack  implements CommunicationChannel.Sendable
    {

        FLIGHT_INFORMATION() { super(meta, 0); }
        FLIGHT_INFORMATION(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot)
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long arming_time_utc_GET()//Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
        {  return (get_bytes(data,  4, 8)); }
        public void arming_time_utc_SET(long  src) //Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
        {  set_bytes((src) & -1L, 8, data,  4); }
        public long takeoff_time_utc_GET()//Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
        {  return (get_bytes(data,  12, 8)); }
        public void takeoff_time_utc_SET(long  src) //Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
        {  set_bytes((src) & -1L, 8, data,  12); }
        public long flight_uuid_GET()//Universally unique identifier (UUID) of flight, should correspond to name of logfiles
        {  return (get_bytes(data,  20, 8)); }
        public void flight_uuid_SET(long  src) //Universally unique identifier (UUID) of flight, should correspond to name of logfiles
        {  set_bytes((src) & -1L, 8, data,  20); }
        static final Meta meta = new Meta(264, 0, 1, 3, 28, 224);
    } public static class MOUNT_ORIENTATION extends Pack  implements CommunicationChannel.Sendable
    {

        MOUNT_ORIENTATION() { super(meta, 0); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot)
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void roll_SET(float  src) //Roll in degrees
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void pitch_SET(float  src) //Pitch in degrees
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void yaw_SET(float  src) //Yaw in degrees
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        static final Meta meta = new Meta(265, 0, 1, 0, 16, 128);
    }/**
*A message containing logged data (see also MAV_CMD_LOGGING_START)*/
    public static class LOGGING_DATA extends Pack  implements CommunicationChannel.Sendable
    {

        LOGGING_DATA() { super(meta, 0); }
        public void sequence_SET(char  src) //sequence number (can wrap)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void target_system_SET(char  src) //system ID of the target
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void target_component_SET(char  src) //component ID of the target
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public void length_SET(char  src) //data length
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        /**
        *offset into data where first message starts. This can be used for recovery, when a previous message got
        *	 lost (set to 255 if no start exists)*/
        public void first_message_offset_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public void data__SET(char[]  src, int pos)  //logged data
        {
            for(int BYTE =  6, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(266, 1, 0, 0, 255, 2040);
    }/**
*A message containing logged data which requires a LOGGING_ACK to be sent back*/
    public static class LOGGING_DATA_ACKED extends Pack  implements CommunicationChannel.Sendable
    {

        LOGGING_DATA_ACKED() { super(meta, 0); }
        public void sequence_SET(char  src) //sequence number (can wrap)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void target_system_SET(char  src) //system ID of the target
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void target_component_SET(char  src) //component ID of the target
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public void length_SET(char  src) //data length
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        /**
        *offset into data where first message starts. This can be used for recovery, when a previous message got
        *	 lost (set to 255 if no start exists)*/
        public void first_message_offset_SET(char  src)
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public void data__SET(char[]  src, int pos)  //logged data
        {
            for(int BYTE =  6, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(267, 1, 0, 0, 255, 2040);
    }/**
*An ack for a LOGGING_DATA_ACKED message*/
    public static class LOGGING_ACK extends Pack  implements CommunicationChannel.Sendable
    {

        LOGGING_ACK() { super(meta, 0); }
        public void sequence_SET(char  src) //sequence number (must match the one in LOGGING_DATA_ACKED)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void target_system_SET(char  src) //system ID of the target
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public void target_component_SET(char  src) //component ID of the target
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        static final Meta meta = new Meta(268, 1, 0, 0, 4, 32);
    }/**
*WIP: Information about video stream*/
    public static class VIDEO_STREAM_INFORMATION extends Pack  implements CommunicationChannel.Sendable
    {

        VIDEO_STREAM_INFORMATION() { super(meta, 0); }
        public void resolution_h_SET(char  src) //Resolution horizontal in pixels
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void resolution_v_SET(char  src) //Resolution vertical in pixels
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public void rotation_SET(char  src) //Video image rotation clockwise
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public void bitrate_SET(long  src) //Bit rate in bits per second
        {  set_bytes((src) & -1L, 4, data,  6); }
        public void camera_id_SET(char  src) //Camera ID (1 for first, 2 for second, etc.)
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
        public void status_SET(char  src) //Current status of video streaming (0: not running, 1: in progress)
        {  set_bytes((char)(src) & -1L, 1, data,  11); }
        public void framerate_SET(float  src) //Frames per second
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void uri_SET(String src, Bounds.Inside ph)//Video stream URI
        {uri_SET(src.toCharArray(), 0, src.length(), ph);} public void uri_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Video stream URI
        {
            if(ph.field_bit != 130 && insert_field(ph, 130, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(269, 3, 1, 0, 17, 130, 2, _YI);
    }/**
*WIP: Message that sets video stream settings*/
    public static class SET_VIDEO_STREAM_SETTINGS extends Pack  implements CommunicationChannel.Sendable
    {

        SET_VIDEO_STREAM_SETTINGS() { super(meta, 0); }
        public void resolution_h_SET(char  src) //Resolution horizontal in pixels (set to -1 for highest resolution possible)
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void resolution_v_SET(char  src) //Resolution vertical in pixels (set to -1 for highest resolution possible)
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public void rotation_SET(char  src) //Video image rotation clockwise (0-359 degrees)
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public void bitrate_SET(long  src) //Bit rate in bits per second (set to -1 for auto)
        {  set_bytes((src) & -1L, 4, data,  6); }
        public void target_system_SET(char  src) //system ID of the target
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
        public void target_component_SET(char  src) //component ID of the target
        {  set_bytes((char)(src) & -1L, 1, data,  11); }
        public void camera_id_SET(char  src) //Camera ID (1 for first, 2 for second, etc.)
        {  set_bytes((char)(src) & -1L, 1, data,  12); }
        public void framerate_SET(float  src) //Frames per second (set to -1 for highest framerate possible)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 13); }
        public void uri_SET(String src, Bounds.Inside ph)//Video stream URI
        {uri_SET(src.toCharArray(), 0, src.length(), ph);} public void uri_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Video stream URI
        {
            if(ph.field_bit != 138 && insert_field(ph, 138, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(270, 3, 1, 0, 18, 138, 2, _zI);
    }/**
*Configure AP SSID and Password.*/
    public static class WIFI_CONFIG_AP extends Pack  implements CommunicationChannel.Sendable
    {

        WIFI_CONFIG_AP() { super(meta, 0); }
        public void ssid_SET(String src, Bounds.Inside ph)//Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
        {ssid_SET(src.toCharArray(), 0, src.length(), ph);} public void ssid_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
        {
            if(ph.field_bit != 2 && insert_field(ph, 2, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } public void password_SET(String src, Bounds.Inside ph) //Password. Leave it blank for an open AP.
        {password_SET(src.toCharArray(), 0, src.length(), ph);} public void password_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Password. Leave it blank for an open AP.
        {
            if(ph.field_bit != 3 && insert_field(ph, 3, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(299, 0, 0, 0, 1, 2, 2, _VI, _RI);
    }/**
*WIP: Version and capability of protocol version. This message is the response to REQUEST_PROTOCOL_VERSION
*	 and is used as part of the handshaking to establish which MAVLink version should be used on the network.
*	 Every node should respond to REQUEST_PROTOCOL_VERSION to enable the handshaking. Library implementers
*	 should consider adding this into the default decoding state machine to allow the protocol core to respond
*	 directly*/
    public static class PROTOCOL_VERSION extends Pack  implements CommunicationChannel.Sendable
    {

        PROTOCOL_VERSION() { super(meta, 0); }
        public void version_SET(char  src) //Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void min_version_SET(char  src) //Minimum MAVLink version supported
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public void max_version_SET(char  src) //Maximum MAVLink version supported (set to the same value as version by default)
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public void spec_version_hash_SET(char[]  src, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash.
        {
            for(int BYTE =  6, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public void library_version_hash_SET(char[]  src, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash.
        {
            for(int BYTE =  14, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(300, 3, 0, 0, 22, 176);
    }/**
*General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message "uavcan.protocol.NodeStatus"
*	 for the background information. The UAVCAN specification is available at http:uavcan.org*/
    public static class UAVCAN_NODE_STATUS extends Pack  implements CommunicationChannel.Sendable
    {

        UAVCAN_NODE_STATUS() { super(meta, 0); }
        public void vendor_specific_status_code_SET(char  src) //Vendor-specific status information.
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void uptime_sec_SET(long  src) //The number of seconds since the start-up of the node.
        {  set_bytes((src) & -1L, 4, data,  2); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  set_bytes((src) & -1L, 8, data,  6); }
        public void sub_mode_SET(char  src) //Not used currently.
        {  set_bytes((char)(src) & -1L, 1, data,  14); }
        public void health_SET(@UAVCAN_NODE_HEALTH int  src) //Generalized node health status.
        {  set_bits(- 0 +   src, 2, data, 120); }
        public void mode_SET(@UAVCAN_NODE_MODE int  src) //Generalized operating mode.
        {
            long id = 0;
            switch(src)
            {
                case UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL:
                    id = 0;
                    break;
                case UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION:
                    id = 1;
                    break;
                case UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE:
                    id = 2;
                    break;
                case UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE:
                    id = 3;
                    break;
                case UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE:
                    id = 4;
                    break;
                default:
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 122);
        }
        static final Meta meta = new Meta(310, 1, 1, 1, 16, 125);
    }/**
*General information describing a particular UAVCAN node. Please refer to the definition of the UAVCAN
*	 service "uavcan.protocol.GetNodeInfo" for the background information. This message should be emitted
*	 by the system whenever a new node appears online, or an existing node reboots. Additionally, it can be
*	 emitted upon request from the other end of the MAVLink channel (see MAV_CMD_UAVCAN_GET_NODE_INFO). It
*	 is also not prohibited to emit this message unconditionally at a low frequency. The UAVCAN specification
*	 is available at http:uavcan.org*/
    public static class UAVCAN_NODE_INFO extends Pack  implements CommunicationChannel.Sendable
    {

        UAVCAN_NODE_INFO() { super(meta, 0); }
        public void uptime_sec_SET(long  src) //The number of seconds since the start-up of the node.
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void sw_vcs_commit_SET(long  src) //Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
        {  set_bytes((src) & -1L, 4, data,  4); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        {  set_bytes((src) & -1L, 8, data,  8); }
        public void hw_version_major_SET(char  src) //Hardware major version number.
        {  set_bytes((char)(src) & -1L, 1, data,  16); }
        public void hw_version_minor_SET(char  src) //Hardware minor version number.
        {  set_bytes((char)(src) & -1L, 1, data,  17); }
        public void hw_unique_id_SET(char[]  src, int pos)  //Hardware unique 128-bit ID.
        {
            for(int BYTE =  18, src_max = pos + 16; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public void sw_version_major_SET(char  src) //Software major version number.
        {  set_bytes((char)(src) & -1L, 1, data,  34); }
        public void sw_version_minor_SET(char  src) //Software minor version number.
        {  set_bytes((char)(src) & -1L, 1, data,  35); }
        public void name_SET(String src, Bounds.Inside ph)//Node name string. For example, "sapog.px4.io".
        {name_SET(src.toCharArray(), 0, src.length(), ph);} public void name_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Node name string. For example, "sapog.px4.io".
        {
            if(ph.field_bit != 288 && insert_field(ph, 288, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(311, 0, 2, 1, 37, 288, 0, _Ox);
    }/**
*Request to read the value of a parameter with the either the param_id string id or param_index.*/
    public static class PARAM_EXT_REQUEST_READ extends Pack  implements CommunicationChannel.Sendable
    {

        PARAM_EXT_REQUEST_READ() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void param_index_SET(short  src) //Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored
        {  set_bytes((short)(src) & -1L, 2, data,  2); }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
        public void param_id_SET(String src, Bounds.Inside ph)
        {param_id_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	 ID is stored as strin*/
        public void param_id_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 32 && insert_field(ph, 32, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(320, 0, 0, 0, 5, 32, 0, _bx);
    }/**
*Request all parameters of this component. After this request, all parameters are emitted.*/
    public static class PARAM_EXT_REQUEST_LIST extends Pack  implements CommunicationChannel.Sendable
    {

        PARAM_EXT_REQUEST_LIST() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        static final Meta meta = new Meta(321, 0, 0, 0, 2, 16);
    }/**
*Emit the value of a parameter. The inclusion of param_count and param_index in the message allows the
*	 recipient to keep track of received parameters and allows them to re-request missing parameters after
*	 a loss or timeout*/
    public static class PARAM_EXT_VALUE extends Pack  implements CommunicationChannel.Sendable
    {

        PARAM_EXT_VALUE() { super(meta, 0); }
        public void param_count_SET(char  src) //Total number of parameters
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void param_index_SET(char  src) //Index of this parameter
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public void param_type_SET(@MAV_PARAM_EXT_TYPE int  src) //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
        {  set_bits(- 1 +   src, 4, data, 32); }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
        public void param_id_SET(String src, Bounds.Inside ph)
        {param_id_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	 ID is stored as strin*/
        public void param_id_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 38 && insert_field(ph, 38, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } public void param_value_SET(String src, Bounds.Inside ph) //Parameter value
        {param_value_SET(src.toCharArray(), 0, src.length(), ph);} public void param_value_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Parameter value
        {
            if(ph.field_bit != 39 && insert_field(ph, 39, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(322, 2, 0, 0, 5, 38, 2, _xx, _kx);
    }/**
*Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET), when
*	 setting a parameter value and the new value is the same as the current value, you will immediately get
*	 a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you will accordingly receive
*	 a PARAM_ACK_IN_PROGRESS in response*/
    public static class PARAM_EXT_SET extends Pack  implements CommunicationChannel.Sendable
    {

        PARAM_EXT_SET() { super(meta, 0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public void param_type_SET(@MAV_PARAM_EXT_TYPE int  src) //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
        {  set_bits(- 1 +   src, 4, data, 16); }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
        public void param_id_SET(String src, Bounds.Inside ph)
        {param_id_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	 ID is stored as strin*/
        public void param_id_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 22 && insert_field(ph, 22, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } public void param_value_SET(String src, Bounds.Inside ph) //Parameter value
        {param_value_SET(src.toCharArray(), 0, src.length(), ph);} public void param_value_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Parameter value
        {
            if(ph.field_bit != 23 && insert_field(ph, 23, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(323, 0, 0, 0, 3, 22, 2, _Qx, _qx);
    }/**
*Response from a PARAM_EXT_SET message.*/
    public static class PARAM_EXT_ACK extends Pack  implements CommunicationChannel.Sendable
    {

        PARAM_EXT_ACK() { super(meta, 0); }
        public void param_type_SET(@MAV_PARAM_EXT_TYPE int  src) //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
        {  set_bits(- 1 +   src, 4, data, 0); }
        public void param_result_SET(@PARAM_ACK int  src) //Result code: see the PARAM_ACK enum for possible codes.
        {  set_bits(- 0 +   src, 2, data, 4); }
        /**
        *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
        *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
        *	 ID is stored as strin*/
        public void param_id_SET(String src, Bounds.Inside ph)
        {param_id_SET(src.toCharArray(), 0, src.length(), ph);}/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	 ID is stored as strin*/
        public void param_id_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 8 && insert_field(ph, 8, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } public void param_value_SET(String src, Bounds.Inside ph) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
        {param_value_SET(src.toCharArray(), 0, src.length(), ph);} public void param_value_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
        {
            if(ph.field_bit != 9 && insert_field(ph, 9, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(324, 0, 0, 0, 1, 8, 2, _Wx, _jx);
    }/**
*Obstacle distances in front of the sensor, starting from the left in increment degrees to the right*/
    public static class OBSTACLE_DISTANCE extends Pack  implements CommunicationChannel.Sendable
    {

        OBSTACLE_DISTANCE() { super(meta, 0); }
        /**
        *Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
        *	 is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
        *	 for unknown/not used. In a array element, each unit corresponds to 1cm*/
        public void distances_SET(char[]  src, int pos)
        {
            for(int BYTE =  0, src_max = pos + 72; pos < src_max; pos++, BYTE += 2)
                set_bytes((char)(src[pos]) & -1L, 2, data,  BYTE);
        }
        public void min_distance_SET(char  src) //Minimum distance the sensor can measure in centimeters
        {  set_bytes((char)(src) & -1L, 2, data,  144); }
        public void max_distance_SET(char  src) //Maximum distance the sensor can measure in centimeters
        {  set_bytes((char)(src) & -1L, 2, data,  146); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since system boot or since UNIX epoch)
        {  set_bytes((src) & -1L, 8, data,  148); }
        public void increment_SET(char  src) //Angular width in degrees of each array element.
        {  set_bytes((char)(src) & -1L, 1, data,  156); }
        public void sensor_type_SET(@MAV_DISTANCE_SENSOR int  src) //Class id of the distance sensor type.
        {  set_bits(- 0 +   src, 3, data, 1256); }
        static final Meta meta = new Meta(330, 74, 0, 1, 158, 1259);
    }/**
*Static data to configure the ADS-B transponder (send within 10 sec of a POR and every 10 sec thereafter*/
    public static class UAVIONIX_ADSB_OUT_CFG extends Pack  implements CommunicationChannel.Sendable
    {

        UAVIONIX_ADSB_OUT_CFG() { super(meta, 0); }
        public void stallSpeed_SET(char  src) //Aircraft stall speed in cm/s
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void ICAO_SET(long  src) //Vehicle address (24 bit)
        {  set_bytes((src) & -1L, 4, data,  2); }
        public void emitterType_SET(@ADSB_EMITTER_TYPE int  src) //Transmitting vehicle type. See ADSB_EMITTER_TYPE enum
        {  set_bits(- 0 +   src, 5, data, 48); }
        public void aircraftSize_SET(@UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE int  src) //Aircraft length and width encoding (table 2-35 of DO-282B)
        {  set_bits(- 0 +   src, 4, data, 53); }
        public void gpsOffsetLat_SET(@UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT int  src) //GPS antenna lateral offset (table 2-36 of DO-282B)
        {  set_bits(- 0 +   src, 3, data, 57); }
        /**
        *GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add
        *	 one] (table 2-37 DO-282B*/
        public void gpsOffsetLon_SET(@UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON int  src)
        {  set_bits(- 0 +   src, 1, data, 60); }
        public void rfSelect_SET(@UAVIONIX_ADSB_OUT_RF_SELECT int  src) //ADS-B transponder reciever and transmit enable flags
        {  set_bits(- 0 +   src, 3, data, 61); }
        public void callsign_SET(String src, Bounds.Inside ph)//Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
        {callsign_SET(src.toCharArray(), 0, src.length(), ph);} public void callsign_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
        {
            if(ph.field_bit != 64 && insert_field(ph, 64, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(10001, 1, 1, 0, 9, 64, 0, _Fx);
    }/**
*Dynamic data used to generate ADS-B out transponder data (send at 5Hz)*/
    public static class UAVIONIX_ADSB_OUT_DYNAMIC extends Pack  implements CommunicationChannel.Sendable
    {

        UAVIONIX_ADSB_OUT_DYNAMIC() { super(meta, 0); }
        public void accuracyVert_SET(char  src) //Vertical accuracy in cm. If unknown set to UINT16_MAX
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public void accuracyVel_SET(char  src) //Velocity accuracy in mm/s (m * 1E-3). If unknown set to UINT16_MAX
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public void squawk_SET(char  src) //Mode A code (typically 1200 [0x04B0] for VFR)
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public void utcTime_SET(long  src) //UTC time in seconds since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX
        {  set_bytes((src) & -1L, 4, data,  6); }
        public void accuracyHor_SET(long  src) //Horizontal accuracy in mm (m * 1E-3). If unknown set to UINT32_MAX
        {  set_bytes((src) & -1L, 4, data,  10); }
        public void gpsLat_SET(int  src) //Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
        {  set_bytes((int)(src) & -1L, 4, data,  14); }
        public void gpsLon_SET(int  src) //Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
        {  set_bytes((int)(src) & -1L, 4, data,  18); }
        public void gpsAlt_SET(int  src) //Altitude in mm (m * 1E-3) UP +ve. WGS84 altitude. If unknown set to INT32_MAX
        {  set_bytes((int)(src) & -1L, 4, data,  22); }
        public void numSats_SET(char  src) //Number of satellites visible. If unknown set to UINT8_MAX
        {  set_bytes((char)(src) & -1L, 1, data,  26); }
        /**
        *Barometric pressure altitude relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude
        *	 (m * 1E-3). (up +ve). If unknown set to INT32_MA*/
        public void baroAltMSL_SET(int  src)
        {  set_bytes((int)(src) & -1L, 4, data,  27); }
        public void velVert_SET(short  src) //GPS vertical speed in cm/s. If unknown set to INT16_MAX
        {  set_bytes((short)(src) & -1L, 2, data,  31); }
        public void velNS_SET(short  src) //North-South velocity over ground in cm/s North +ve. If unknown set to INT16_MAX
        {  set_bytes((short)(src) & -1L, 2, data,  33); }
        public void VelEW_SET(short  src) //East-West velocity over ground in cm/s East +ve. If unknown set to INT16_MAX
        {  set_bytes((short)(src) & -1L, 2, data,  35); }
        public void gpsFix_SET(@UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX int  src) //0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK
        {  set_bits(- 0 +   src, 3, data, 296); }
        public void emergencyStatus_SET(@UAVIONIX_ADSB_EMERGENCY_STATUS int  src) //Emergency status
        {  set_bits(- 0 +   src, 3, data, 299); }
        public void state_SET(@UAVIONIX_ADSB_OUT_DYNAMIC_STATE int  src) //ADS-B transponder dynamic input state flags
        {  set_bits(- 1 +   src, 5, data, 302); }
        static final Meta meta = new Meta(10002, 3, 2, 0, 39, 307);
    }/**
*Transceiver heartbeat with health report (updated every 10s)*/
    public static class UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT extends Pack  implements CommunicationChannel.Sendable
    {

        UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT() { super(meta, 0); }
        public void rfHealth_SET(@UAVIONIX_ADSB_RF_HEALTH int  src) //ADS-B transponder messages
        {  set_bits(- 0 +   src, 6, data, 0); }
        static final Meta meta = new Meta(10003, 0, 0, 0, 1, 6);
    }/**
*Read registers for a device*/
    public static class DEVICE_OP_READ extends Pack  implements CommunicationChannel.Sendable
    {

        DEVICE_OP_READ() { super(meta, 0); }
        public void request_id_SET(long  src) //request ID - copied to reply
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public void bus_SET(char  src) //Bus number
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public void address_SET(char  src) //Bus address
        {  set_bytes((char)(src) & -1L, 1, data,  7); }
        public void regstart_SET(char  src) //First register to read
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        public void count_SET(char  src) //count of registers to read
        {  set_bytes((char)(src) & -1L, 1, data,  9); }
        public void bustype_SET(@DEVICE_OP_BUSTYPE int  src) //The bus type
        {  set_bits(- 0 +   src, 1, data, 80); }
        public void busname_SET(String src, Bounds.Inside ph)//Name of device on bus (for SPI)
        {busname_SET(src.toCharArray(), 0, src.length(), ph);} public void busname_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Name of device on bus (for SPI)
        {
            if(ph.field_bit != 81 && insert_field(ph, 81, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(11000, 0, 1, 0, 12, 81, 0, _dQ);
    }/**
*Read registers reply*/
    public static class DEVICE_OP_READ_REPLY extends Pack  implements CommunicationChannel.Sendable
    {

        DEVICE_OP_READ_REPLY() { super(meta, 0); }
        public void request_id_SET(long  src) //request ID - copied from request
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void result_SET(char  src) //0 for success, anything else is failure code
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void regstart_SET(char  src) //starting register
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public void count_SET(char  src) //count of bytes read
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public void data__SET(char[]  src, int pos)  //reply data
        {
            for(int BYTE =  7, src_max = pos + 128; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(11001, 0, 1, 0, 135, 1080);
    }/**
*Write registers for a device*/
    public static class DEVICE_OP_WRITE extends Pack  implements CommunicationChannel.Sendable
    {

        DEVICE_OP_WRITE() { super(meta, 0); }
        public void request_id_SET(long  src) //request ID - copied to reply
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public void target_component_SET(char  src) //Component ID
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public void bus_SET(char  src) //Bus number
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public void address_SET(char  src) //Bus address
        {  set_bytes((char)(src) & -1L, 1, data,  7); }
        public void regstart_SET(char  src) //First register to write
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        public void count_SET(char  src) //count of registers to write
        {  set_bytes((char)(src) & -1L, 1, data,  9); }
        public void data__SET(char[]  src, int pos)  //write data
        {
            for(int BYTE =  10, src_max = pos + 128; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public void bustype_SET(@DEVICE_OP_BUSTYPE int  src) //The bus type
        {  set_bits(- 0 +   src, 1, data, 1104); }
        public void busname_SET(String src, Bounds.Inside ph)//Name of device on bus (for SPI)
        {busname_SET(src.toCharArray(), 0, src.length(), ph);} public void busname_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Name of device on bus (for SPI)
        {
            if(ph.field_bit != 1105 && insert_field(ph, 1105, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(11002, 0, 1, 0, 140, 1105, 0, _lQ);
    }/**
*Write registers reply*/
    public static class DEVICE_OP_WRITE_REPLY extends Pack  implements CommunicationChannel.Sendable
    {

        DEVICE_OP_WRITE_REPLY() { super(meta, 0); }
        public void request_id_SET(long  src) //request ID - copied from request
        {  set_bytes((src) & -1L, 4, data,  0); }
        public void result_SET(char  src) //0 for success, anything else is failure code
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        static final Meta meta = new Meta(11003, 0, 1, 0, 5, 40);
    }/**
*Adaptive Controller tuning information*/
    public static class ADAP_TUNING extends Pack  implements CommunicationChannel.Sendable
    {

        ADAP_TUNING() { super(meta, 0); }
        public void desired_SET(float  src) //desired rate (degrees/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 0); }
        public void achieved_SET(float  src) //achieved rate (degrees/s)
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public void error_SET(float  src) //error between model and vehicle
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public void theta_SET(float  src) //theta estimated state predictor
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public void omega_SET(float  src) //omega estimated state predictor
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public void sigma_SET(float  src) //sigma estimated state predictor
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public void theta_dot_SET(float  src) //theta derivative
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public void omega_dot_SET(float  src) //omega derivative
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public void sigma_dot_SET(float  src) //sigma derivative
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public void f_SET(float  src) //projection operator value
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public void f_dot_SET(float  src) //projection operator derivative
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public void u_SET(float  src) //u adaptive controlled output command
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }
        public void axis_SET(@PID_TUNING_AXIS int  src) //axis
        {  set_bits(- 1 +   src, 3, data, 384); }
        static final Meta meta = new Meta(11010, 0, 0, 0, 49, 387);
    }/**
*camera vision based attitude and position deltas*/
    public static class VISION_POSITION_DELTA extends Pack  implements CommunicationChannel.Sendable
    {

        VISION_POSITION_DELTA() { super(meta, 0); }
        public void time_usec_SET(long  src) //Timestamp (microseconds, synced to UNIX time or since system boot)
        {  set_bytes((src) & -1L, 8, data,  0); }
        public void time_delta_usec_SET(long  src) //Time in microseconds since the last reported camera frame
        {  set_bytes((src) & -1L, 8, data,  8); }
        public void angle_delta_SET(float[]  src, int pos)  //Defines a rotation vector in body frame that rotates the vehicle from the previous to the current orientatio
        {
            for(int BYTE =  16, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        /**
        *Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right,
        *	 2=down*/
        public void position_delta_SET(float[]  src, int pos)
        {
            for(int BYTE =  28, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public void confidence_SET(float  src) //normalised confidence value from 0 to 100
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        static final Meta meta = new Meta(11011, 0, 0, 2, 44, 352);
    }

    public static class CommunicationChannel  extends Channel
    {
        static {pack_id_bytes = 2; }

        public static  CommunicationChannel instance = new CommunicationChannel();

        public final java.io.InputStream inputStream = new  InputStream();
        //interface-to-mark of sendable through this channel packs_Schs_Rchs
        public interface Sendable {}
        public static VISION_SPEED_ESTIMATE new_VISION_SPEED_ESTIMATE() {return new  VISION_SPEED_ESTIMATE();}
        public static VICON_POSITION_ESTIMATE new_VICON_POSITION_ESTIMATE() {return new  VICON_POSITION_ESTIMATE();}
        /**
        *The IMU readings in SI units in NED body frame*/
        public static HIGHRES_IMU new_HIGHRES_IMU() {return new  HIGHRES_IMU();}
        /**
        *Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)*/
        public static OPTICAL_FLOW_RAD new_OPTICAL_FLOW_RAD() {return new  OPTICAL_FLOW_RAD();}
        /**
        *The IMU readings in SI units in NED body frame*/
        public static HIL_SENSOR new_HIL_SENSOR() {return new  HIL_SENSOR();}
        /**
        *Status of simulation environment, if used*/
        public static SIM_STATE new_SIM_STATE() {return new  SIM_STATE();}
        /**
        *Status generated by radio and injected into MAVLink stream.*/
        public static RADIO_STATUS new_RADIO_STATUS() {return new  RADIO_STATUS();}
        /**
        *File transfer message*/
        public static FILE_TRANSFER_PROTOCOL new_FILE_TRANSFER_PROTOCOL() {return new  FILE_TRANSFER_PROTOCOL();}
        /**
        *Time synchronization message.*/
        public static TIMESYNC new_TIMESYNC() {return new  TIMESYNC();}
        /**
        *Camera-IMU triggering and synchronisation message.*/
        public static CAMERA_TRIGGER new_CAMERA_TRIGGER() {return new  CAMERA_TRIGGER();}
        /**
        *The global position, as returned by the Global Positioning System (GPS). This is
        *	 NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
        public static HIL_GPS new_HIL_GPS() {return new  HIL_GPS();}
        /**
        *Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)*/
        public static HIL_OPTICAL_FLOW new_HIL_OPTICAL_FLOW() {return new  HIL_OPTICAL_FLOW();}
        /**
        *Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful
        *	 for high throughput applications such as hardware in the loop simulations*/
        public static HIL_STATE_QUATERNION new_HIL_STATE_QUATERNION() {return new  HIL_STATE_QUATERNION();}
        /**
        *The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to
        *	 the described unit*/
        public static SCALED_IMU2 new_SCALED_IMU2() {return new  SCALED_IMU2();}
        /**
        *Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END
        *	 is called*/
        public static LOG_REQUEST_LIST new_LOG_REQUEST_LIST() {return new  LOG_REQUEST_LIST();}
        /**
        *Reply to LOG_REQUEST_LIST*/
        public static LOG_ENTRY new_LOG_ENTRY() {return new  LOG_ENTRY();}
        /**
        *Request a chunk of a log*/
        public static LOG_REQUEST_DATA new_LOG_REQUEST_DATA() {return new  LOG_REQUEST_DATA();}
        /**
        *Reply to LOG_REQUEST_DATA*/
        public static LOG_DATA new_LOG_DATA() {return new  LOG_DATA();}
        /**
        *Erase all logs*/
        public static LOG_ERASE new_LOG_ERASE() {return new  LOG_ERASE();}
        /**
        *Stop log transfer and resume normal logging*/
        public static LOG_REQUEST_END new_LOG_REQUEST_END() {return new  LOG_REQUEST_END();}
        /**
        *data for injecting into the onboard GPS (used for DGPS)*/
        public static GPS_INJECT_DATA new_GPS_INJECT_DATA() {return new  GPS_INJECT_DATA();}
        /**
        *Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
        public static GPS2_RAW new_GPS2_RAW() {return new  GPS2_RAW();}
        /**
        *Power supply status*/
        public static POWER_STATUS new_POWER_STATUS() {return new  POWER_STATUS();}
        /**
        *Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or
        *	 telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages
        *	 or change the devices settings. A message with zero bytes can be used to change just the baudrate*/
        public static SERIAL_CONTROL new_SERIAL_CONTROL() {return new  SERIAL_CONTROL();}
        /**
        *RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting*/
        public static GPS_RTK new_GPS_RTK() {return new  GPS_RTK();}
        /**
        *RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting*/
        public static GPS2_RTK new_GPS2_RTK() {return new  GPS2_RTK();}
        /**
        *The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described
        *	 unit*/
        public static SCALED_IMU3 new_SCALED_IMU3() {return new  SCALED_IMU3();}
        public static DATA_TRANSMISSION_HANDSHAKE new_DATA_TRANSMISSION_HANDSHAKE() {return new  DATA_TRANSMISSION_HANDSHAKE();}
        public static ENCAPSULATED_DATA new_ENCAPSULATED_DATA() {return new  ENCAPSULATED_DATA();}
        public static DISTANCE_SENSOR new_DISTANCE_SENSOR() {return new  DISTANCE_SENSOR();}
        /**
        *Request for terrain data and terrain status*/
        public static TERRAIN_REQUEST new_TERRAIN_REQUEST() {return new  TERRAIN_REQUEST();}
        /**
        *Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUES*/
        public static TERRAIN_DATA new_TERRAIN_DATA() {return new  TERRAIN_DATA();}
        /**
        *Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle
        *	 has all terrain data needed for a mission*/
        public static TERRAIN_CHECK new_TERRAIN_CHECK() {return new  TERRAIN_CHECK();}
        /**
        *Response from a TERRAIN_CHECK request*/
        public static TERRAIN_REPORT new_TERRAIN_REPORT() {return new  TERRAIN_REPORT();}
        /**
        *Barometer readings for 2nd barometer*/
        public static SCALED_PRESSURE2 new_SCALED_PRESSURE2() {return new  SCALED_PRESSURE2();}
        /**
        *Motion capture attitude and position*/
        public static ATT_POS_MOCAP new_ATT_POS_MOCAP() {return new  ATT_POS_MOCAP();}
        /**
        *Set the vehicle attitude and body angular rates.*/
        public static SET_ACTUATOR_CONTROL_TARGET new_SET_ACTUATOR_CONTROL_TARGET() {return new  SET_ACTUATOR_CONTROL_TARGET();}
        /**
        *Set the vehicle attitude and body angular rates.*/
        public static ACTUATOR_CONTROL_TARGET new_ACTUATOR_CONTROL_TARGET() {return new  ACTUATOR_CONTROL_TARGET();}
        /**
        *The current system altitude.*/
        public static ALTITUDE new_ALTITUDE() {return new  ALTITUDE();}
        /**
        *The autopilot is requesting a resource (file, binary, other type of data)*/
        public static RESOURCE_REQUEST new_RESOURCE_REQUEST() {return new  RESOURCE_REQUEST();}
        /**
        *Barometer readings for 3rd barometer*/
        public static SCALED_PRESSURE3 new_SCALED_PRESSURE3() {return new  SCALED_PRESSURE3();}
        /**
        *current motion information from a designated system*/
        public static FOLLOW_TARGET new_FOLLOW_TARGET() {return new  FOLLOW_TARGET();}
        /**
        *The smoothed, monotonic system state used to feed the control loops of the system.*/
        public static CONTROL_SYSTEM_STATE new_CONTROL_SYSTEM_STATE() {return new  CONTROL_SYSTEM_STATE();}
        /**
        *Battery information*/
        public static BATTERY_STATUS new_BATTERY_STATUS() {return new  BATTERY_STATUS();}
        /**
        *Version and capability of autopilot software*/
        public static AUTOPILOT_VERSION new_AUTOPILOT_VERSION() {return new  AUTOPILOT_VERSION();}
        /**
        *The location of a landing area captured from a downward facing camera*/
        public static LANDING_TARGET new_LANDING_TARGET() {return new  LANDING_TARGET();}
        /**
        *Offsets and calibrations values for hardware sensors. This makes it easier to debug the calibration process*/
        public static SENSOR_OFFSETS new_SENSOR_OFFSETS() {return new  SENSOR_OFFSETS();}
        /**
        *Deprecated. Use MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS instead. Set the magnetometer offsets*/
        public static SET_MAG_OFFSETS new_SET_MAG_OFFSETS() {return new  SET_MAG_OFFSETS();}
        /**
        *state of APM memory*/
        public static MEMINFO new_MEMINFO() {return new  MEMINFO();}
        /**
        *raw ADC output*/
        public static AP_ADC new_AP_ADC() {return new  AP_ADC();}
        /**
        *Configure on-board Camera Control System.*/
        public static DIGICAM_CONFIGURE new_DIGICAM_CONFIGURE() {return new  DIGICAM_CONFIGURE();}
        /**
        *Control on-board Camera Control System to take shots.*/
        public static DIGICAM_CONTROL new_DIGICAM_CONTROL() {return new  DIGICAM_CONTROL();}
        /**
        *Message to configure a camera mount, directional antenna, etc.*/
        public static MOUNT_CONFIGURE new_MOUNT_CONFIGURE() {return new  MOUNT_CONFIGURE();}
        /**
        *Message to control a camera mount, directional antenna, etc.*/
        public static MOUNT_CONTROL new_MOUNT_CONTROL() {return new  MOUNT_CONTROL();}
        /**
        *Message with some status from APM to GCS about camera or antenna mount*/
        public static MOUNT_STATUS new_MOUNT_STATUS() {return new  MOUNT_STATUS();}
        /**
        *GCS*/
        public static FENCE_POINT new_FENCE_POINT() {return new  FENCE_POINT();}
        /**
        *Request a current fence point from MAV*/
        public static FENCE_FETCH_POINT new_FENCE_FETCH_POINT() {return new  FENCE_FETCH_POINT();}
        /**
        *Status of geo-fencing. Sent in extended status stream when fencing enabled*/
        public static FENCE_STATUS new_FENCE_STATUS() {return new  FENCE_STATUS();}
        /**
        *Status of DCM attitude estimator*/
        public static AHRS new_AHRS() {return new  AHRS();}
        /**
        *Status of simulation environment, if used*/
        public static SIMSTATE new_SIMSTATE() {return new  SIMSTATE();}
        /**
        *Status of key hardware*/
        public static HWSTATUS new_HWSTATUS() {return new  HWSTATUS();}
        /**
        *Status generated by radio*/
        public static RADIO new_RADIO() {return new  RADIO();}
        /**
        *Status of AP_Limits. Sent in extended status stream when AP_Limits is enabled*/
        public static LIMITS_STATUS new_LIMITS_STATUS() {return new  LIMITS_STATUS();}
        /**
        *Wind estimation*/
        public static WIND new_WIND() {return new  WIND();}
        /**
        *Data packet, size 16*/
        public static DATA16 new_DATA16() {return new  DATA16();}
        /**
        *Data packet, size 32*/
        public static DATA32 new_DATA32() {return new  DATA32();}
        /**
        *Data packet, size 64*/
        public static DATA64 new_DATA64() {return new  DATA64();}
        /**
        *Data packet, size 96*/
        public static DATA96 new_DATA96() {return new  DATA96();}
        /**
        *Rangefinder reporting*/
        public static RANGEFINDER new_RANGEFINDER() {return new  RANGEFINDER();}
        /**
        *Airspeed auto-calibration*/
        public static AIRSPEED_AUTOCAL new_AIRSPEED_AUTOCAL() {return new  AIRSPEED_AUTOCAL();}
        /**
        *GCS*/
        public static RALLY_POINT new_RALLY_POINT() {return new  RALLY_POINT();}
        /**
        *Request a current rally point from MAV. MAV should respond with a RALLY_POINT message. MAV should not
        *	 respond if the request is invalid*/
        public static RALLY_FETCH_POINT new_RALLY_FETCH_POINT() {return new  RALLY_FETCH_POINT();}
        /**
        *Status of compassmot calibration*/
        public static COMPASSMOT_STATUS new_COMPASSMOT_STATUS() {return new  COMPASSMOT_STATUS();}
        /**
        *Status of secondary AHRS filter if available*/
        public static AHRS2 new_AHRS2() {return new  AHRS2();}
        /**
        *Camera Event*/
        public static CAMERA_STATUS new_CAMERA_STATUS() {return new  CAMERA_STATUS();}
        /**
        *Camera Capture Feedback*/
        public static CAMERA_FEEDBACK new_CAMERA_FEEDBACK() {return new  CAMERA_FEEDBACK();}
        /**
        *2nd Battery status*/
        public static BATTERY2 new_BATTERY2() {return new  BATTERY2();}
        /**
        *Status of third AHRS filter if available. This is for ANU research group (Ali and Sean)*/
        public static AHRS3 new_AHRS3() {return new  AHRS3();}
        /**
        *Request the autopilot version from the system/component.*/
        public static AUTOPILOT_VERSION_REQUEST new_AUTOPILOT_VERSION_REQUEST() {return new  AUTOPILOT_VERSION_REQUEST();}
        /**
        *Send a block of log data to remote location*/
        public static REMOTE_LOG_DATA_BLOCK new_REMOTE_LOG_DATA_BLOCK() {return new  REMOTE_LOG_DATA_BLOCK();}
        /**
        *Send Status of each log block that autopilot board might have sent*/
        public static REMOTE_LOG_BLOCK_STATUS new_REMOTE_LOG_BLOCK_STATUS() {return new  REMOTE_LOG_BLOCK_STATUS();}
        /**
        *Control vehicle LEDs*/
        public static LED_CONTROL new_LED_CONTROL() {return new  LED_CONTROL();}
        /**
        *Reports progress of compass calibration.*/
        public static MAG_CAL_PROGRESS new_MAG_CAL_PROGRESS() {return new  MAG_CAL_PROGRESS();}
        /**
        *Reports results of completed compass calibration. Sent until MAG_CAL_ACK received.*/
        public static MAG_CAL_REPORT new_MAG_CAL_REPORT() {return new  MAG_CAL_REPORT();}
        /**
        *EKF Status message including flags and variances*/
        public static EKF_STATUS_REPORT new_EKF_STATUS_REPORT() {return new  EKF_STATUS_REPORT();}
        /**
        *PID tuning information*/
        public static PID_TUNING new_PID_TUNING() {return new  PID_TUNING();}
        /**
        *3 axis gimbal mesuraments*/
        public static GIMBAL_REPORT new_GIMBAL_REPORT() {return new  GIMBAL_REPORT();}
        /**
        *Control message for rate gimbal*/
        public static GIMBAL_CONTROL new_GIMBAL_CONTROL() {return new  GIMBAL_CONTROL();}
        /**
        *100 Hz gimbal torque command telemetry*/
        public static GIMBAL_TORQUE_CMD_REPORT new_GIMBAL_TORQUE_CMD_REPORT() {return new  GIMBAL_TORQUE_CMD_REPORT();}
        /**
        *Heartbeat from a HeroBus attached GoPro*/
        public static GOPRO_HEARTBEAT new_GOPRO_HEARTBEAT() {return new  GOPRO_HEARTBEAT();}
        /**
        *Request a GOPRO_COMMAND response from the GoPro*/
        public static GOPRO_GET_REQUEST new_GOPRO_GET_REQUEST() {return new  GOPRO_GET_REQUEST();}
        /**
        *Response from a GOPRO_COMMAND get request*/
        public static GOPRO_GET_RESPONSE new_GOPRO_GET_RESPONSE() {return new  GOPRO_GET_RESPONSE();}
        /**
        *Request to set a GOPRO_COMMAND with a desired*/
        public static GOPRO_SET_REQUEST new_GOPRO_SET_REQUEST() {return new  GOPRO_SET_REQUEST();}
        /**
        *Response from a GOPRO_COMMAND set request*/
        public static GOPRO_SET_RESPONSE new_GOPRO_SET_RESPONSE() {return new  GOPRO_SET_RESPONSE();}
        /**
        *RPM sensor output*/
        public static RPM new_RPM() {return new  RPM();}
        /**
        *Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message
        *	 is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS
        *	 enum definition for further information. The innovaton test ratios show the magnitude of the sensor innovation
        *	 divided by the innovation check threshold. Under normal operation the innovaton test ratios should be
        *	 below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation
        *	 and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation
        *	 test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should
        *	 be optional and controllable by the user*/
        public static ESTIMATOR_STATUS new_ESTIMATOR_STATUS() {return new  ESTIMATOR_STATUS();}
        public static WIND_COV new_WIND_COV() {return new  WIND_COV();}
        /**
        *GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position
        *	 estimate of the sytem*/
        public static GPS_INPUT new_GPS_INPUT() {return new  GPS_INPUT();}
        /**
        *RTCM message for injecting into the onboard GPS (used for DGPS)*/
        public static GPS_RTCM_DATA new_GPS_RTCM_DATA() {return new  GPS_RTCM_DATA();}
        /**
        *Message appropriate for high latency connections like Iridium*/
        public static HIGH_LATENCY new_HIGH_LATENCY() {return new  HIGH_LATENCY();}
        /**
        *Vibration levels and accelerometer clipping*/
        public static VIBRATION new_VIBRATION() {return new  VIBRATION();}
        /**
        *This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system
        *	 will return to and land on. The position is set automatically by the system during the takeoff in case
        *	 it was not explicitely set by the operator before or after. The position the system will return to and
        *	 land on. The global and local positions encode the position in the respective coordinate frames, while
        *	 the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading
        *	 and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes
        *	 the point to which the system should fly in normal flight mode and then perform a landing sequence along
        *	 the vector*/
        public static HOME_POSITION new_HOME_POSITION() {return new  HOME_POSITION();}
        /**
        *The position the system will return to and land on. The position is set automatically by the system during
        *	 the takeoff in case it was not explicitely set by the operator before or after. The global and local
        *	 positions encode the position in the respective coordinate frames, while the q parameter encodes the
        *	 orientation of the surface. Under normal conditions it describes the heading and terrain slope, which
        *	 can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which
        *	 the system should fly in normal flight mode and then perform a landing sequence along the vector*/
        public static SET_HOME_POSITION new_SET_HOME_POSITION() {return new  SET_HOME_POSITION();}
        /**
        *This interface replaces DATA_STREAM*/
        public static MESSAGE_INTERVAL new_MESSAGE_INTERVAL() {return new  MESSAGE_INTERVAL();}
        /**
        *Provides state for additional features*/
        public static EXTENDED_SYS_STATE new_EXTENDED_SYS_STATE() {return new  EXTENDED_SYS_STATE();}
        /**
        *The location and information of an ADSB vehicle*/
        public static ADSB_VEHICLE new_ADSB_VEHICLE() {return new  ADSB_VEHICLE();}
        /**
        *Information about a potential collision*/
        public static COLLISION new_COLLISION() {return new  COLLISION();}
        /**
        *Message implementing parts of the V2 payload specs in V1 frames for transitional support.*/
        public static V2_EXTENSION new_V2_EXTENSION() {return new  V2_EXTENSION();}
        /**
        *Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient
        *	 way for testing new messages and getting experimental debug output*/
        public static MEMORY_VECT new_MEMORY_VECT() {return new  MEMORY_VECT();}
        public static DEBUG_VECT new_DEBUG_VECT() {return new  DEBUG_VECT();}
        /**
        *Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite
        *	 efficient way for testing new messages and getting experimental debug output*/
        public static NAMED_VALUE_FLOAT new_NAMED_VALUE_FLOAT() {return new  NAMED_VALUE_FLOAT();}
        /**
        *Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite
        *	 efficient way for testing new messages and getting experimental debug output*/
        public static NAMED_VALUE_INT new_NAMED_VALUE_INT() {return new  NAMED_VALUE_INT();}
        /**
        *Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING:
        *	 They consume quite some bandwidth, so use only for important status and error messages. If implemented
        *	 wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz)*/
        public static STATUSTEXT new_STATUSTEXT() {return new  STATUSTEXT();}
        /**
        *Send a debug value. The index is used to discriminate between values. These values show up in the plot
        *	 of QGroundControl as DEBUG N*/
        public static DEBUG new_DEBUG() {return new  DEBUG();}
        /**
        *Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will disable
        *	 signin*/
        public static SETUP_SIGNING new_SETUP_SIGNING() {return new  SETUP_SIGNING();}
        /**
        *Report button state change*/
        public static BUTTON_CHANGE new_BUTTON_CHANGE() {return new  BUTTON_CHANGE();}
        /**
        *Control vehicle tone generation (buzzer)*/
        public static PLAY_TUNE new_PLAY_TUNE() {return new  PLAY_TUNE();}
        /**
        *WIP: Information about a camera*/
        public static CAMERA_INFORMATION new_CAMERA_INFORMATION() {return new  CAMERA_INFORMATION();}
        /**
        *WIP: Settings of a camera, can be requested using MAV_CMD_REQUEST_CAMERA_SETTINGS.*/
        public static CAMERA_SETTINGS new_CAMERA_SETTINGS() {return new  CAMERA_SETTINGS();}
        /**
        *WIP: Information about a storage medium.*/
        public static STORAGE_INFORMATION new_STORAGE_INFORMATION() {return new  STORAGE_INFORMATION();}
        /**
        *WIP: Information about the status of a capture*/
        public static CAMERA_CAPTURE_STATUS new_CAMERA_CAPTURE_STATUS() {return new  CAMERA_CAPTURE_STATUS();}
        /**
        *Information about a captured image*/
        public static CAMERA_IMAGE_CAPTURED new_CAMERA_IMAGE_CAPTURED() {return new  CAMERA_IMAGE_CAPTURED();}
        /**
        *WIP: Information about flight since last arming*/
        public static FLIGHT_INFORMATION new_FLIGHT_INFORMATION() {return new  FLIGHT_INFORMATION();}
        public static MOUNT_ORIENTATION new_MOUNT_ORIENTATION() {return new  MOUNT_ORIENTATION();}
        /**
        *A message containing logged data (see also MAV_CMD_LOGGING_START)*/
        public static LOGGING_DATA new_LOGGING_DATA() {return new  LOGGING_DATA();}
        /**
        *A message containing logged data which requires a LOGGING_ACK to be sent back*/
        public static LOGGING_DATA_ACKED new_LOGGING_DATA_ACKED() {return new  LOGGING_DATA_ACKED();}
        /**
        *An ack for a LOGGING_DATA_ACKED message*/
        public static LOGGING_ACK new_LOGGING_ACK() {return new  LOGGING_ACK();}
        /**
        *WIP: Information about video stream*/
        public static VIDEO_STREAM_INFORMATION new_VIDEO_STREAM_INFORMATION() {return new  VIDEO_STREAM_INFORMATION();}
        /**
        *WIP: Message that sets video stream settings*/
        public static SET_VIDEO_STREAM_SETTINGS new_SET_VIDEO_STREAM_SETTINGS() {return new  SET_VIDEO_STREAM_SETTINGS();}
        /**
        *Configure AP SSID and Password.*/
        public static WIFI_CONFIG_AP new_WIFI_CONFIG_AP() {return new  WIFI_CONFIG_AP();}
        /**
        *WIP: Version and capability of protocol version. This message is the response to REQUEST_PROTOCOL_VERSION
        *	 and is used as part of the handshaking to establish which MAVLink version should be used on the network.
        *	 Every node should respond to REQUEST_PROTOCOL_VERSION to enable the handshaking. Library implementers
        *	 should consider adding this into the default decoding state machine to allow the protocol core to respond
        *	 directly*/
        public static PROTOCOL_VERSION new_PROTOCOL_VERSION() {return new  PROTOCOL_VERSION();}
        /**
        *General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message "uavcan.protocol.NodeStatus"
        *	 for the background information. The UAVCAN specification is available at http:uavcan.org*/
        public static UAVCAN_NODE_STATUS new_UAVCAN_NODE_STATUS() {return new  UAVCAN_NODE_STATUS();}
        /**
        *General information describing a particular UAVCAN node. Please refer to the definition of the UAVCAN
        *	 service "uavcan.protocol.GetNodeInfo" for the background information. This message should be emitted
        *	 by the system whenever a new node appears online, or an existing node reboots. Additionally, it can be
        *	 emitted upon request from the other end of the MAVLink channel (see MAV_CMD_UAVCAN_GET_NODE_INFO). It
        *	 is also not prohibited to emit this message unconditionally at a low frequency. The UAVCAN specification
        *	 is available at http:uavcan.org*/
        public static UAVCAN_NODE_INFO new_UAVCAN_NODE_INFO() {return new  UAVCAN_NODE_INFO();}
        /**
        *Request to read the value of a parameter with the either the param_id string id or param_index.*/
        public static PARAM_EXT_REQUEST_READ new_PARAM_EXT_REQUEST_READ() {return new  PARAM_EXT_REQUEST_READ();}
        /**
        *Request all parameters of this component. After this request, all parameters are emitted.*/
        public static PARAM_EXT_REQUEST_LIST new_PARAM_EXT_REQUEST_LIST() {return new  PARAM_EXT_REQUEST_LIST();}
        /**
        *Emit the value of a parameter. The inclusion of param_count and param_index in the message allows the
        *	 recipient to keep track of received parameters and allows them to re-request missing parameters after
        *	 a loss or timeout*/
        public static PARAM_EXT_VALUE new_PARAM_EXT_VALUE() {return new  PARAM_EXT_VALUE();}
        /**
        *Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET), when
        *	 setting a parameter value and the new value is the same as the current value, you will immediately get
        *	 a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you will accordingly receive
        *	 a PARAM_ACK_IN_PROGRESS in response*/
        public static PARAM_EXT_SET new_PARAM_EXT_SET() {return new  PARAM_EXT_SET();}
        /**
        *Response from a PARAM_EXT_SET message.*/
        public static PARAM_EXT_ACK new_PARAM_EXT_ACK() {return new  PARAM_EXT_ACK();}
        /**
        *Obstacle distances in front of the sensor, starting from the left in increment degrees to the right*/
        public static OBSTACLE_DISTANCE new_OBSTACLE_DISTANCE() {return new  OBSTACLE_DISTANCE();}
        /**
        *Static data to configure the ADS-B transponder (send within 10 sec of a POR and every 10 sec thereafter*/
        public static UAVIONIX_ADSB_OUT_CFG new_UAVIONIX_ADSB_OUT_CFG() {return new  UAVIONIX_ADSB_OUT_CFG();}
        /**
        *Dynamic data used to generate ADS-B out transponder data (send at 5Hz)*/
        public static UAVIONIX_ADSB_OUT_DYNAMIC new_UAVIONIX_ADSB_OUT_DYNAMIC() {return new  UAVIONIX_ADSB_OUT_DYNAMIC();}
        /**
        *Transceiver heartbeat with health report (updated every 10s)*/
        public static UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT() {return new  UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();}
        /**
        *Read registers for a device*/
        public static DEVICE_OP_READ new_DEVICE_OP_READ() {return new  DEVICE_OP_READ();}
        /**
        *Read registers reply*/
        public static DEVICE_OP_READ_REPLY new_DEVICE_OP_READ_REPLY() {return new  DEVICE_OP_READ_REPLY();}
        /**
        *Write registers for a device*/
        public static DEVICE_OP_WRITE new_DEVICE_OP_WRITE() {return new  DEVICE_OP_WRITE();}
        /**
        *Write registers reply*/
        public static DEVICE_OP_WRITE_REPLY new_DEVICE_OP_WRITE_REPLY() {return new  DEVICE_OP_WRITE_REPLY();}
        /**
        *Adaptive Controller tuning information*/
        public static ADAP_TUNING new_ADAP_TUNING() {return new  ADAP_TUNING();}
        /**
        *camera vision based attitude and position deltas*/
        public static VISION_POSITION_DELTA new_VISION_POSITION_DELTA() {return new  VISION_POSITION_DELTA();}

        public void send(Sendable pack) { sendout_packs.add((Pack) pack);}
        protected final Queue<Pack> sendout_packs = new ConcurrentLinkedQueue<Pack>()
        {
            @Override public boolean add(Pack pack)
            {
                synchronized(this)
                {
                    boolean ret = super.add(pack);
                    this.notify();
                    return ret;
                }
            }
        };

        public boolean waitingSendoutPack()
        {
            try
            {
                synchronized(sendout_packs)
                {
                    while(sendout_packs.size() == 0) sendout_packs.wait();
                }
                return true;
            }
            catch(InterruptedException e) {}
            return false;
        }

        public final java.io.OutputStream outputStream = new  OutputStream();
        protected final Queue<Pack> received_packs = new ConcurrentLinkedQueue<Pack>()
        {
            @Override public boolean add(Pack pack)
            {
                synchronized(this)
                {
                    boolean ret = super.add(pack);
                    this.notify();
                    return ret;
                }
            }
        };

        public boolean waitingReceivedPack()
        {
            try
            {
                synchronized(received_packs)
                {
                    while(received_packs.size() == 0) received_packs.wait();
                    return true;
                }
            }
            catch(InterruptedException e) {}
            return false;
        }


        private final Bounds.Inside ph    = new Bounds.Inside();


        @Override protected Pack process(Pack pack, int id)
        {
            for(boolean LOOP = false; ;)
            {
                switch(id)
                {
                    default:
                        assert(false);
                        return null;
                    case 0:
                        if(pack == null) return new HEARTBEAT();
                        final HEARTBEAT pi0 = (HEARTBEAT) pack;
                        ph.setPack(pi0);
                        on_HEARTBEAT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi0));
                        if(LOOP) break;
                        return null;
                    case 1:
                        if(pack == null) return new SYS_STATUS();
                        final SYS_STATUS pi1 = (SYS_STATUS) pack;
                        ph.setPack(pi1);
                        on_SYS_STATUS.forEach(h -> h.handle(CommunicationChannel.this, ph, pi1));
                        if(LOOP) break;
                        return null;
                    case 2:
                        if(pack == null) return new SYSTEM_TIME();
                        final SYSTEM_TIME pi2 = (SYSTEM_TIME) pack;
                        ph.setPack(pi2);
                        on_SYSTEM_TIME.forEach(h -> h.handle(CommunicationChannel.this, ph, pi2));
                        if(LOOP) break;
                        return null;
                    case 3:
                        if(pack == null) return new POSITION_TARGET_LOCAL_NED();
                        final POSITION_TARGET_LOCAL_NED pi3 = (POSITION_TARGET_LOCAL_NED) pack;
                        ph.setPack(pi3);
                        on_POSITION_TARGET_LOCAL_NED.forEach(h -> h.handle(CommunicationChannel.this, ph, pi3));
                        if(LOOP) break;
                        return null;
                    case 4:
                        if(pack == null) return new PING();
                        final PING pi4 = (PING) pack;
                        ph.setPack(pi4);
                        on_PING.forEach(h -> h.handle(CommunicationChannel.this, ph, pi4));
                        if(LOOP) break;
                        return null;
                    case 5:
                        if(pack == null) return new CHANGE_OPERATOR_CONTROL(-1);
                        final CHANGE_OPERATOR_CONTROL pi5 = (CHANGE_OPERATOR_CONTROL) pack;
                        ph.setPack(pi5);
                        on_CHANGE_OPERATOR_CONTROL.forEach(h -> h.handle(CommunicationChannel.this, ph, pi5));
                        if(LOOP) break;
                        return null;
                    case 6:
                        if(pack == null) return new CHANGE_OPERATOR_CONTROL_ACK();
                        final CHANGE_OPERATOR_CONTROL_ACK pi6 = (CHANGE_OPERATOR_CONTROL_ACK) pack;
                        ph.setPack(pi6);
                        on_CHANGE_OPERATOR_CONTROL_ACK.forEach(h -> h.handle(CommunicationChannel.this, ph, pi6));
                        if(LOOP) break;
                        return null;
                    case 7:
                        if(pack == null) return new AUTH_KEY(-1);
                        final AUTH_KEY pi7 = (AUTH_KEY) pack;
                        ph.setPack(pi7);
                        on_AUTH_KEY.forEach(h -> h.handle(CommunicationChannel.this, ph, pi7));
                        if(LOOP) break;
                        return null;
                    case 11:
                        if(pack == null) return new SET_MODE();
                        final SET_MODE pi11 = (SET_MODE) pack;
                        ph.setPack(pi11);
                        on_SET_MODE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi11));
                        if(LOOP) break;
                        return null;
                    case 20:
                        if(pack == null) return new PARAM_REQUEST_READ(-1);
                        final PARAM_REQUEST_READ pi20 = (PARAM_REQUEST_READ) pack;
                        ph.setPack(pi20);
                        on_PARAM_REQUEST_READ.forEach(h -> h.handle(CommunicationChannel.this, ph, pi20));
                        if(LOOP) break;
                        return null;
                    case 21:
                        if(pack == null) return new PARAM_REQUEST_LIST();
                        final PARAM_REQUEST_LIST pi21 = (PARAM_REQUEST_LIST) pack;
                        ph.setPack(pi21);
                        on_PARAM_REQUEST_LIST.forEach(h -> h.handle(CommunicationChannel.this, ph, pi21));
                        if(LOOP) break;
                        return null;
                    case 22:
                        if(pack == null) return new PARAM_VALUE(-1);
                        final PARAM_VALUE pi22 = (PARAM_VALUE) pack;
                        ph.setPack(pi22);
                        on_PARAM_VALUE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi22));
                        if(LOOP) break;
                        return null;
                    case 23:
                        if(pack == null) return new PARAM_SET(-1);
                        final PARAM_SET pi23 = (PARAM_SET) pack;
                        ph.setPack(pi23);
                        on_PARAM_SET.forEach(h -> h.handle(CommunicationChannel.this, ph, pi23));
                        if(LOOP) break;
                        return null;
                    case 24:
                        if(pack == null) return new GPS_RAW_INT(-1);
                        final GPS_RAW_INT pi24 = (GPS_RAW_INT) pack;
                        ph.setPack(pi24);
                        on_GPS_RAW_INT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi24));
                        if(LOOP) break;
                        return null;
                    case 25:
                        if(pack == null) return new GPS_STATUS();
                        final GPS_STATUS pi25 = (GPS_STATUS) pack;
                        ph.setPack(pi25);
                        on_GPS_STATUS.forEach(h -> h.handle(CommunicationChannel.this, ph, pi25));
                        if(LOOP) break;
                        return null;
                    case 26:
                        if(pack == null) return new SCALED_IMU();
                        final SCALED_IMU pi26 = (SCALED_IMU) pack;
                        ph.setPack(pi26);
                        on_SCALED_IMU.forEach(h -> h.handle(CommunicationChannel.this, ph, pi26));
                        if(LOOP) break;
                        return null;
                    case 27:
                        if(pack == null) return new RAW_IMU();
                        final RAW_IMU pi27 = (RAW_IMU) pack;
                        ph.setPack(pi27);
                        on_RAW_IMU.forEach(h -> h.handle(CommunicationChannel.this, ph, pi27));
                        if(LOOP) break;
                        return null;
                    case 28:
                        if(pack == null) return new RAW_PRESSURE();
                        final RAW_PRESSURE pi28 = (RAW_PRESSURE) pack;
                        ph.setPack(pi28);
                        on_RAW_PRESSURE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi28));
                        if(LOOP) break;
                        return null;
                    case 29:
                        if(pack == null) return new SCALED_PRESSURE();
                        final SCALED_PRESSURE pi29 = (SCALED_PRESSURE) pack;
                        ph.setPack(pi29);
                        on_SCALED_PRESSURE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi29));
                        if(LOOP) break;
                        return null;
                    case 30:
                        if(pack == null) return new ATTITUDE();
                        final ATTITUDE pi30 = (ATTITUDE) pack;
                        ph.setPack(pi30);
                        on_ATTITUDE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi30));
                        if(LOOP) break;
                        return null;
                    case 31:
                        if(pack == null) return new ATTITUDE_QUATERNION();
                        final ATTITUDE_QUATERNION pi31 = (ATTITUDE_QUATERNION) pack;
                        ph.setPack(pi31);
                        on_ATTITUDE_QUATERNION.forEach(h -> h.handle(CommunicationChannel.this, ph, pi31));
                        if(LOOP) break;
                        return null;
                    case 32:
                        if(pack == null) return new LOCAL_POSITION_NED();
                        final LOCAL_POSITION_NED pi32 = (LOCAL_POSITION_NED) pack;
                        ph.setPack(pi32);
                        on_LOCAL_POSITION_NED.forEach(h -> h.handle(CommunicationChannel.this, ph, pi32));
                        if(LOOP) break;
                        return null;
                    case 33:
                        if(pack == null) return new GLOBAL_POSITION_INT();
                        final GLOBAL_POSITION_INT pi33 = (GLOBAL_POSITION_INT) pack;
                        ph.setPack(pi33);
                        on_GLOBAL_POSITION_INT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi33));
                        if(LOOP) break;
                        return null;
                    case 34:
                        if(pack == null) return new RC_CHANNELS_SCALED();
                        final RC_CHANNELS_SCALED pi34 = (RC_CHANNELS_SCALED) pack;
                        ph.setPack(pi34);
                        on_RC_CHANNELS_SCALED.forEach(h -> h.handle(CommunicationChannel.this, ph, pi34));
                        if(LOOP) break;
                        return null;
                    case 35:
                        if(pack == null) return new RC_CHANNELS_RAW();
                        final RC_CHANNELS_RAW pi35 = (RC_CHANNELS_RAW) pack;
                        ph.setPack(pi35);
                        on_RC_CHANNELS_RAW.forEach(h -> h.handle(CommunicationChannel.this, ph, pi35));
                        if(LOOP) break;
                        return null;
                    case 36:
                        if(pack == null) return new SERVO_OUTPUT_RAW(-1);
                        final SERVO_OUTPUT_RAW pi36 = (SERVO_OUTPUT_RAW) pack;
                        ph.setPack(pi36);
                        on_SERVO_OUTPUT_RAW.forEach(h -> h.handle(CommunicationChannel.this, ph, pi36));
                        if(LOOP) break;
                        return null;
                    case 37:
                        if(pack == null) return new MISSION_REQUEST_PARTIAL_LIST();
                        final MISSION_REQUEST_PARTIAL_LIST pi37 = (MISSION_REQUEST_PARTIAL_LIST) pack;
                        ph.setPack(pi37);
                        on_MISSION_REQUEST_PARTIAL_LIST.forEach(h -> h.handle(CommunicationChannel.this, ph, pi37));
                        if(LOOP) break;
                        return null;
                    case 38:
                        if(pack == null) return new MISSION_WRITE_PARTIAL_LIST();
                        final MISSION_WRITE_PARTIAL_LIST pi38 = (MISSION_WRITE_PARTIAL_LIST) pack;
                        ph.setPack(pi38);
                        on_MISSION_WRITE_PARTIAL_LIST.forEach(h -> h.handle(CommunicationChannel.this, ph, pi38));
                        if(LOOP) break;
                        return null;
                    case 39:
                        if(pack == null) return new MISSION_ITEM();
                        final MISSION_ITEM pi39 = (MISSION_ITEM) pack;
                        ph.setPack(pi39);
                        on_MISSION_ITEM.forEach(h -> h.handle(CommunicationChannel.this, ph, pi39));
                        if(LOOP) break;
                        return null;
                    case 40:
                        if(pack == null) return new MISSION_REQUEST();
                        final MISSION_REQUEST pi40 = (MISSION_REQUEST) pack;
                        ph.setPack(pi40);
                        on_MISSION_REQUEST.forEach(h -> h.handle(CommunicationChannel.this, ph, pi40));
                        if(LOOP) break;
                        return null;
                    case 41:
                        if(pack == null) return new MISSION_SET_CURRENT();
                        final MISSION_SET_CURRENT pi41 = (MISSION_SET_CURRENT) pack;
                        ph.setPack(pi41);
                        on_MISSION_SET_CURRENT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi41));
                        if(LOOP) break;
                        return null;
                    case 42:
                        if(pack == null) return new MISSION_CURRENT();
                        final MISSION_CURRENT pi42 = (MISSION_CURRENT) pack;
                        ph.setPack(pi42);
                        on_MISSION_CURRENT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi42));
                        if(LOOP) break;
                        return null;
                    case 43:
                        if(pack == null) return new MISSION_REQUEST_LIST();
                        final MISSION_REQUEST_LIST pi43 = (MISSION_REQUEST_LIST) pack;
                        ph.setPack(pi43);
                        on_MISSION_REQUEST_LIST.forEach(h -> h.handle(CommunicationChannel.this, ph, pi43));
                        if(LOOP) break;
                        return null;
                    case 44:
                        if(pack == null) return new MISSION_COUNT();
                        final MISSION_COUNT pi44 = (MISSION_COUNT) pack;
                        ph.setPack(pi44);
                        on_MISSION_COUNT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi44));
                        if(LOOP) break;
                        return null;
                    case 45:
                        if(pack == null) return new MISSION_CLEAR_ALL();
                        final MISSION_CLEAR_ALL pi45 = (MISSION_CLEAR_ALL) pack;
                        ph.setPack(pi45);
                        on_MISSION_CLEAR_ALL.forEach(h -> h.handle(CommunicationChannel.this, ph, pi45));
                        if(LOOP) break;
                        return null;
                    case 46:
                        if(pack == null) return new MISSION_ITEM_REACHED();
                        final MISSION_ITEM_REACHED pi46 = (MISSION_ITEM_REACHED) pack;
                        ph.setPack(pi46);
                        on_MISSION_ITEM_REACHED.forEach(h -> h.handle(CommunicationChannel.this, ph, pi46));
                        if(LOOP) break;
                        return null;
                    case 47:
                        if(pack == null) return new MISSION_ACK();
                        final MISSION_ACK pi47 = (MISSION_ACK) pack;
                        ph.setPack(pi47);
                        on_MISSION_ACK.forEach(h -> h.handle(CommunicationChannel.this, ph, pi47));
                        if(LOOP) break;
                        return null;
                    case 48:
                        if(pack == null) return new SET_GPS_GLOBAL_ORIGIN(-1);
                        final SET_GPS_GLOBAL_ORIGIN pi48 = (SET_GPS_GLOBAL_ORIGIN) pack;
                        ph.setPack(pi48);
                        on_SET_GPS_GLOBAL_ORIGIN.forEach(h -> h.handle(CommunicationChannel.this, ph, pi48));
                        if(LOOP) break;
                        return null;
                    case 49:
                        if(pack == null) return new GPS_GLOBAL_ORIGIN(-1);
                        final GPS_GLOBAL_ORIGIN pi49 = (GPS_GLOBAL_ORIGIN) pack;
                        ph.setPack(pi49);
                        on_GPS_GLOBAL_ORIGIN.forEach(h -> h.handle(CommunicationChannel.this, ph, pi49));
                        if(LOOP) break;
                        return null;
                    case 50:
                        if(pack == null) return new PARAM_MAP_RC(-1);
                        final PARAM_MAP_RC pi50 = (PARAM_MAP_RC) pack;
                        ph.setPack(pi50);
                        on_PARAM_MAP_RC.forEach(h -> h.handle(CommunicationChannel.this, ph, pi50));
                        if(LOOP) break;
                        return null;
                    case 51:
                        if(pack == null) return new MISSION_REQUEST_INT();
                        final MISSION_REQUEST_INT pi51 = (MISSION_REQUEST_INT) pack;
                        ph.setPack(pi51);
                        on_MISSION_REQUEST_INT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi51));
                        if(LOOP) break;
                        return null;
                    case 54:
                        if(pack == null) return new SAFETY_SET_ALLOWED_AREA();
                        final SAFETY_SET_ALLOWED_AREA pi54 = (SAFETY_SET_ALLOWED_AREA) pack;
                        ph.setPack(pi54);
                        on_SAFETY_SET_ALLOWED_AREA.forEach(h -> h.handle(CommunicationChannel.this, ph, pi54));
                        if(LOOP) break;
                        return null;
                    case 55:
                        if(pack == null) return new SAFETY_ALLOWED_AREA();
                        final SAFETY_ALLOWED_AREA pi55 = (SAFETY_ALLOWED_AREA) pack;
                        ph.setPack(pi55);
                        on_SAFETY_ALLOWED_AREA.forEach(h -> h.handle(CommunicationChannel.this, ph, pi55));
                        if(LOOP) break;
                        return null;
                    case 61:
                        if(pack == null) return new ATTITUDE_QUATERNION_COV();
                        final ATTITUDE_QUATERNION_COV pi61 = (ATTITUDE_QUATERNION_COV) pack;
                        ph.setPack(pi61);
                        on_ATTITUDE_QUATERNION_COV.forEach(h -> h.handle(CommunicationChannel.this, ph, pi61));
                        if(LOOP) break;
                        return null;
                    case 62:
                        if(pack == null) return new NAV_CONTROLLER_OUTPUT();
                        final NAV_CONTROLLER_OUTPUT pi62 = (NAV_CONTROLLER_OUTPUT) pack;
                        ph.setPack(pi62);
                        on_NAV_CONTROLLER_OUTPUT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi62));
                        if(LOOP) break;
                        return null;
                    case 63:
                        if(pack == null) return new GLOBAL_POSITION_INT_COV();
                        final GLOBAL_POSITION_INT_COV pi63 = (GLOBAL_POSITION_INT_COV) pack;
                        ph.setPack(pi63);
                        on_GLOBAL_POSITION_INT_COV.forEach(h -> h.handle(CommunicationChannel.this, ph, pi63));
                        if(LOOP) break;
                        return null;
                    case 64:
                        if(pack == null) return new LOCAL_POSITION_NED_COV();
                        final LOCAL_POSITION_NED_COV pi64 = (LOCAL_POSITION_NED_COV) pack;
                        ph.setPack(pi64);
                        on_LOCAL_POSITION_NED_COV.forEach(h -> h.handle(CommunicationChannel.this, ph, pi64));
                        if(LOOP) break;
                        return null;
                    case 65:
                        if(pack == null) return new RC_CHANNELS();
                        final RC_CHANNELS pi65 = (RC_CHANNELS) pack;
                        ph.setPack(pi65);
                        on_RC_CHANNELS.forEach(h -> h.handle(CommunicationChannel.this, ph, pi65));
                        if(LOOP) break;
                        return null;
                    case 66:
                        if(pack == null) return new REQUEST_DATA_STREAM();
                        final REQUEST_DATA_STREAM pi66 = (REQUEST_DATA_STREAM) pack;
                        ph.setPack(pi66);
                        on_REQUEST_DATA_STREAM.forEach(h -> h.handle(CommunicationChannel.this, ph, pi66));
                        if(LOOP) break;
                        return null;
                    case 67:
                        if(pack == null) return new DATA_STREAM();
                        final DATA_STREAM pi67 = (DATA_STREAM) pack;
                        ph.setPack(pi67);
                        on_DATA_STREAM.forEach(h -> h.handle(CommunicationChannel.this, ph, pi67));
                        if(LOOP) break;
                        return null;
                    case 69:
                        if(pack == null) return new MANUAL_CONTROL();
                        final MANUAL_CONTROL pi69 = (MANUAL_CONTROL) pack;
                        ph.setPack(pi69);
                        on_MANUAL_CONTROL.forEach(h -> h.handle(CommunicationChannel.this, ph, pi69));
                        if(LOOP) break;
                        return null;
                    case 70:
                        if(pack == null) return new RC_CHANNELS_OVERRIDE();
                        final RC_CHANNELS_OVERRIDE pi70 = (RC_CHANNELS_OVERRIDE) pack;
                        ph.setPack(pi70);
                        on_RC_CHANNELS_OVERRIDE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi70));
                        if(LOOP) break;
                        return null;
                    case 73:
                        if(pack == null) return new MISSION_ITEM_INT();
                        final MISSION_ITEM_INT pi73 = (MISSION_ITEM_INT) pack;
                        ph.setPack(pi73);
                        on_MISSION_ITEM_INT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi73));
                        if(LOOP) break;
                        return null;
                    case 74:
                        if(pack == null) return new VFR_HUD();
                        final VFR_HUD pi74 = (VFR_HUD) pack;
                        ph.setPack(pi74);
                        on_VFR_HUD.forEach(h -> h.handle(CommunicationChannel.this, ph, pi74));
                        if(LOOP) break;
                        return null;
                    case 75:
                        if(pack == null) return new COMMAND_INT();
                        final COMMAND_INT pi75 = (COMMAND_INT) pack;
                        ph.setPack(pi75);
                        on_COMMAND_INT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi75));
                        if(LOOP) break;
                        return null;
                    case 76:
                        if(pack == null) return new COMMAND_LONG();
                        final COMMAND_LONG pi76 = (COMMAND_LONG) pack;
                        ph.setPack(pi76);
                        on_COMMAND_LONG.forEach(h -> h.handle(CommunicationChannel.this, ph, pi76));
                        if(LOOP) break;
                        return null;
                    case 77:
                        if(pack == null) return new COMMAND_ACK(-1);
                        final COMMAND_ACK pi77 = (COMMAND_ACK) pack;
                        ph.setPack(pi77);
                        on_COMMAND_ACK.forEach(h -> h.handle(CommunicationChannel.this, ph, pi77));
                        if(LOOP) break;
                        return null;
                    case 81:
                        if(pack == null) return new MANUAL_SETPOINT();
                        final MANUAL_SETPOINT pi81 = (MANUAL_SETPOINT) pack;
                        ph.setPack(pi81);
                        on_MANUAL_SETPOINT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi81));
                        if(LOOP) break;
                        return null;
                    case 82:
                        if(pack == null) return new SET_ATTITUDE_TARGET();
                        final SET_ATTITUDE_TARGET pi82 = (SET_ATTITUDE_TARGET) pack;
                        ph.setPack(pi82);
                        on_SET_ATTITUDE_TARGET.forEach(h -> h.handle(CommunicationChannel.this, ph, pi82));
                        if(LOOP) break;
                        return null;
                    case 83:
                        if(pack == null) return new ATTITUDE_TARGET();
                        final ATTITUDE_TARGET pi83 = (ATTITUDE_TARGET) pack;
                        ph.setPack(pi83);
                        on_ATTITUDE_TARGET.forEach(h -> h.handle(CommunicationChannel.this, ph, pi83));
                        if(LOOP) break;
                        return null;
                    case 84:
                        if(pack == null) return new SET_POSITION_TARGET_LOCAL_NED();
                        final SET_POSITION_TARGET_LOCAL_NED pi84 = (SET_POSITION_TARGET_LOCAL_NED) pack;
                        ph.setPack(pi84);
                        on_SET_POSITION_TARGET_LOCAL_NED.forEach(h -> h.handle(CommunicationChannel.this, ph, pi84));
                        if(LOOP) break;
                        return null;
                    case 86:
                        if(pack == null) return new SET_POSITION_TARGET_GLOBAL_INT();
                        final SET_POSITION_TARGET_GLOBAL_INT pi86 = (SET_POSITION_TARGET_GLOBAL_INT) pack;
                        ph.setPack(pi86);
                        on_SET_POSITION_TARGET_GLOBAL_INT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi86));
                        if(LOOP) break;
                        return null;
                    case 87:
                        if(pack == null) return new POSITION_TARGET_GLOBAL_INT();
                        final POSITION_TARGET_GLOBAL_INT pi87 = (POSITION_TARGET_GLOBAL_INT) pack;
                        ph.setPack(pi87);
                        on_POSITION_TARGET_GLOBAL_INT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi87));
                        if(LOOP) break;
                        return null;
                    case 89:
                        if(pack == null) return new LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
                        final LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET pi89 = (LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET) pack;
                        ph.setPack(pi89);
                        on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.forEach(h -> h.handle(CommunicationChannel.this, ph, pi89));
                        if(LOOP) break;
                        return null;
                    case 90:
                        if(pack == null) return new HIL_STATE();
                        final HIL_STATE pi90 = (HIL_STATE) pack;
                        ph.setPack(pi90);
                        on_HIL_STATE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi90));
                        if(LOOP) break;
                        return null;
                    case 91:
                        if(pack == null) return new HIL_CONTROLS();
                        final HIL_CONTROLS pi91 = (HIL_CONTROLS) pack;
                        ph.setPack(pi91);
                        on_HIL_CONTROLS.forEach(h -> h.handle(CommunicationChannel.this, ph, pi91));
                        if(LOOP) break;
                        return null;
                    case 92:
                        if(pack == null) return new HIL_RC_INPUTS_RAW();
                        final HIL_RC_INPUTS_RAW pi92 = (HIL_RC_INPUTS_RAW) pack;
                        ph.setPack(pi92);
                        on_HIL_RC_INPUTS_RAW.forEach(h -> h.handle(CommunicationChannel.this, ph, pi92));
                        if(LOOP) break;
                        return null;
                    case 93:
                        if(pack == null) return new HIL_ACTUATOR_CONTROLS();
                        final HIL_ACTUATOR_CONTROLS pi93 = (HIL_ACTUATOR_CONTROLS) pack;
                        ph.setPack(pi93);
                        on_HIL_ACTUATOR_CONTROLS.forEach(h -> h.handle(CommunicationChannel.this, ph, pi93));
                        if(LOOP) break;
                        return null;
                    case 100:
                        if(pack == null) return new OPTICAL_FLOW(-1);
                        final OPTICAL_FLOW pi100 = (OPTICAL_FLOW) pack;
                        ph.setPack(pi100);
                        on_OPTICAL_FLOW.forEach(h -> h.handle(CommunicationChannel.this, ph, pi100));
                        if(LOOP) break;
                        return null;
                    case 101:
                        if(pack == null) return new GLOBAL_VISION_POSITION_ESTIMATE();
                        final GLOBAL_VISION_POSITION_ESTIMATE pi101 = (GLOBAL_VISION_POSITION_ESTIMATE) pack;
                        ph.setPack(pi101);
                        on_GLOBAL_VISION_POSITION_ESTIMATE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi101));
                        if(LOOP) break;
                        return null;
                    case 102:
                        if(pack == null) return new VISION_POSITION_ESTIMATE();
                        final VISION_POSITION_ESTIMATE pi102 = (VISION_POSITION_ESTIMATE) pack;
                        ph.setPack(pi102);
                        on_VISION_POSITION_ESTIMATE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi102));
                        if(LOOP) break;
                        return null;
                    case 103:
                        if(pack == null) return new VISION_SPEED_ESTIMATE();
                        final VISION_SPEED_ESTIMATE pi103 = (VISION_SPEED_ESTIMATE) pack;
                        ph.setPack(pi103);
                        on_VISION_SPEED_ESTIMATE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi103));
                        if(LOOP) break;
                        return null;
                    case 104:
                        if(pack == null) return new VICON_POSITION_ESTIMATE();
                        final VICON_POSITION_ESTIMATE pi104 = (VICON_POSITION_ESTIMATE) pack;
                        ph.setPack(pi104);
                        on_VICON_POSITION_ESTIMATE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi104));
                        if(LOOP) break;
                        return null;
                    case 105:
                        if(pack == null) return new HIGHRES_IMU();
                        final HIGHRES_IMU pi105 = (HIGHRES_IMU) pack;
                        ph.setPack(pi105);
                        on_HIGHRES_IMU.forEach(h -> h.handle(CommunicationChannel.this, ph, pi105));
                        if(LOOP) break;
                        return null;
                    case 106:
                        if(pack == null) return new OPTICAL_FLOW_RAD();
                        final OPTICAL_FLOW_RAD pi106 = (OPTICAL_FLOW_RAD) pack;
                        ph.setPack(pi106);
                        on_OPTICAL_FLOW_RAD.forEach(h -> h.handle(CommunicationChannel.this, ph, pi106));
                        if(LOOP) break;
                        return null;
                    case 107:
                        if(pack == null) return new HIL_SENSOR();
                        final HIL_SENSOR pi107 = (HIL_SENSOR) pack;
                        ph.setPack(pi107);
                        on_HIL_SENSOR.forEach(h -> h.handle(CommunicationChannel.this, ph, pi107));
                        if(LOOP) break;
                        return null;
                    case 108:
                        if(pack == null) return new SIM_STATE();
                        final SIM_STATE pi108 = (SIM_STATE) pack;
                        ph.setPack(pi108);
                        on_SIM_STATE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi108));
                        if(LOOP) break;
                        return null;
                    case 109:
                        if(pack == null) return new RADIO_STATUS();
                        final RADIO_STATUS pi109 = (RADIO_STATUS) pack;
                        ph.setPack(pi109);
                        on_RADIO_STATUS.forEach(h -> h.handle(CommunicationChannel.this, ph, pi109));
                        if(LOOP) break;
                        return null;
                    case 110:
                        if(pack == null) return new FILE_TRANSFER_PROTOCOL();
                        final FILE_TRANSFER_PROTOCOL pi110 = (FILE_TRANSFER_PROTOCOL) pack;
                        ph.setPack(pi110);
                        on_FILE_TRANSFER_PROTOCOL.forEach(h -> h.handle(CommunicationChannel.this, ph, pi110));
                        if(LOOP) break;
                        return null;
                    case 111:
                        if(pack == null) return new TIMESYNC();
                        final TIMESYNC pi111 = (TIMESYNC) pack;
                        ph.setPack(pi111);
                        on_TIMESYNC.forEach(h -> h.handle(CommunicationChannel.this, ph, pi111));
                        if(LOOP) break;
                        return null;
                    case 112:
                        if(pack == null) return new CAMERA_TRIGGER();
                        final CAMERA_TRIGGER pi112 = (CAMERA_TRIGGER) pack;
                        ph.setPack(pi112);
                        on_CAMERA_TRIGGER.forEach(h -> h.handle(CommunicationChannel.this, ph, pi112));
                        if(LOOP) break;
                        return null;
                    case 113:
                        if(pack == null) return new HIL_GPS();
                        final HIL_GPS pi113 = (HIL_GPS) pack;
                        ph.setPack(pi113);
                        on_HIL_GPS.forEach(h -> h.handle(CommunicationChannel.this, ph, pi113));
                        if(LOOP) break;
                        return null;
                    case 114:
                        if(pack == null) return new HIL_OPTICAL_FLOW();
                        final HIL_OPTICAL_FLOW pi114 = (HIL_OPTICAL_FLOW) pack;
                        ph.setPack(pi114);
                        on_HIL_OPTICAL_FLOW.forEach(h -> h.handle(CommunicationChannel.this, ph, pi114));
                        if(LOOP) break;
                        return null;
                    case 115:
                        if(pack == null) return new HIL_STATE_QUATERNION();
                        final HIL_STATE_QUATERNION pi115 = (HIL_STATE_QUATERNION) pack;
                        ph.setPack(pi115);
                        on_HIL_STATE_QUATERNION.forEach(h -> h.handle(CommunicationChannel.this, ph, pi115));
                        if(LOOP) break;
                        return null;
                    case 116:
                        if(pack == null) return new SCALED_IMU2();
                        final SCALED_IMU2 pi116 = (SCALED_IMU2) pack;
                        ph.setPack(pi116);
                        on_SCALED_IMU2.forEach(h -> h.handle(CommunicationChannel.this, ph, pi116));
                        if(LOOP) break;
                        return null;
                    case 117:
                        if(pack == null) return new LOG_REQUEST_LIST();
                        final LOG_REQUEST_LIST pi117 = (LOG_REQUEST_LIST) pack;
                        ph.setPack(pi117);
                        on_LOG_REQUEST_LIST.forEach(h -> h.handle(CommunicationChannel.this, ph, pi117));
                        if(LOOP) break;
                        return null;
                    case 118:
                        if(pack == null) return new LOG_ENTRY();
                        final LOG_ENTRY pi118 = (LOG_ENTRY) pack;
                        ph.setPack(pi118);
                        on_LOG_ENTRY.forEach(h -> h.handle(CommunicationChannel.this, ph, pi118));
                        if(LOOP) break;
                        return null;
                    case 119:
                        if(pack == null) return new LOG_REQUEST_DATA();
                        final LOG_REQUEST_DATA pi119 = (LOG_REQUEST_DATA) pack;
                        ph.setPack(pi119);
                        on_LOG_REQUEST_DATA.forEach(h -> h.handle(CommunicationChannel.this, ph, pi119));
                        if(LOOP) break;
                        return null;
                    case 120:
                        if(pack == null) return new LOG_DATA();
                        final LOG_DATA pi120 = (LOG_DATA) pack;
                        ph.setPack(pi120);
                        on_LOG_DATA.forEach(h -> h.handle(CommunicationChannel.this, ph, pi120));
                        if(LOOP) break;
                        return null;
                    case 121:
                        if(pack == null) return new LOG_ERASE();
                        final LOG_ERASE pi121 = (LOG_ERASE) pack;
                        ph.setPack(pi121);
                        on_LOG_ERASE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi121));
                        if(LOOP) break;
                        return null;
                    case 122:
                        if(pack == null) return new LOG_REQUEST_END();
                        final LOG_REQUEST_END pi122 = (LOG_REQUEST_END) pack;
                        ph.setPack(pi122);
                        on_LOG_REQUEST_END.forEach(h -> h.handle(CommunicationChannel.this, ph, pi122));
                        if(LOOP) break;
                        return null;
                    case 123:
                        if(pack == null) return new GPS_INJECT_DATA();
                        final GPS_INJECT_DATA pi123 = (GPS_INJECT_DATA) pack;
                        ph.setPack(pi123);
                        on_GPS_INJECT_DATA.forEach(h -> h.handle(CommunicationChannel.this, ph, pi123));
                        if(LOOP) break;
                        return null;
                    case 124:
                        if(pack == null) return new GPS2_RAW();
                        final GPS2_RAW pi124 = (GPS2_RAW) pack;
                        ph.setPack(pi124);
                        on_GPS2_RAW.forEach(h -> h.handle(CommunicationChannel.this, ph, pi124));
                        if(LOOP) break;
                        return null;
                    case 125:
                        if(pack == null) return new POWER_STATUS();
                        final POWER_STATUS pi125 = (POWER_STATUS) pack;
                        ph.setPack(pi125);
                        on_POWER_STATUS.forEach(h -> h.handle(CommunicationChannel.this, ph, pi125));
                        if(LOOP) break;
                        return null;
                    case 126:
                        if(pack == null) return new SERIAL_CONTROL();
                        final SERIAL_CONTROL pi126 = (SERIAL_CONTROL) pack;
                        ph.setPack(pi126);
                        on_SERIAL_CONTROL.forEach(h -> h.handle(CommunicationChannel.this, ph, pi126));
                        if(LOOP) break;
                        return null;
                    case 127:
                        if(pack == null) return new GPS_RTK();
                        final GPS_RTK pi127 = (GPS_RTK) pack;
                        ph.setPack(pi127);
                        on_GPS_RTK.forEach(h -> h.handle(CommunicationChannel.this, ph, pi127));
                        if(LOOP) break;
                        return null;
                    case 128:
                        if(pack == null) return new GPS2_RTK();
                        final GPS2_RTK pi128 = (GPS2_RTK) pack;
                        ph.setPack(pi128);
                        on_GPS2_RTK.forEach(h -> h.handle(CommunicationChannel.this, ph, pi128));
                        if(LOOP) break;
                        return null;
                    case 129:
                        if(pack == null) return new SCALED_IMU3();
                        final SCALED_IMU3 pi129 = (SCALED_IMU3) pack;
                        ph.setPack(pi129);
                        on_SCALED_IMU3.forEach(h -> h.handle(CommunicationChannel.this, ph, pi129));
                        if(LOOP) break;
                        return null;
                    case 130:
                        if(pack == null) return new DATA_TRANSMISSION_HANDSHAKE();
                        final DATA_TRANSMISSION_HANDSHAKE pi130 = (DATA_TRANSMISSION_HANDSHAKE) pack;
                        ph.setPack(pi130);
                        on_DATA_TRANSMISSION_HANDSHAKE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi130));
                        if(LOOP) break;
                        return null;
                    case 131:
                        if(pack == null) return new ENCAPSULATED_DATA();
                        final ENCAPSULATED_DATA pi131 = (ENCAPSULATED_DATA) pack;
                        ph.setPack(pi131);
                        on_ENCAPSULATED_DATA.forEach(h -> h.handle(CommunicationChannel.this, ph, pi131));
                        if(LOOP) break;
                        return null;
                    case 132:
                        if(pack == null) return new DISTANCE_SENSOR();
                        final DISTANCE_SENSOR pi132 = (DISTANCE_SENSOR) pack;
                        ph.setPack(pi132);
                        on_DISTANCE_SENSOR.forEach(h -> h.handle(CommunicationChannel.this, ph, pi132));
                        if(LOOP) break;
                        return null;
                    case 133:
                        if(pack == null) return new TERRAIN_REQUEST();
                        final TERRAIN_REQUEST pi133 = (TERRAIN_REQUEST) pack;
                        ph.setPack(pi133);
                        on_TERRAIN_REQUEST.forEach(h -> h.handle(CommunicationChannel.this, ph, pi133));
                        if(LOOP) break;
                        return null;
                    case 134:
                        if(pack == null) return new TERRAIN_DATA();
                        final TERRAIN_DATA pi134 = (TERRAIN_DATA) pack;
                        ph.setPack(pi134);
                        on_TERRAIN_DATA.forEach(h -> h.handle(CommunicationChannel.this, ph, pi134));
                        if(LOOP) break;
                        return null;
                    case 135:
                        if(pack == null) return new TERRAIN_CHECK();
                        final TERRAIN_CHECK pi135 = (TERRAIN_CHECK) pack;
                        ph.setPack(pi135);
                        on_TERRAIN_CHECK.forEach(h -> h.handle(CommunicationChannel.this, ph, pi135));
                        if(LOOP) break;
                        return null;
                    case 136:
                        if(pack == null) return new TERRAIN_REPORT();
                        final TERRAIN_REPORT pi136 = (TERRAIN_REPORT) pack;
                        ph.setPack(pi136);
                        on_TERRAIN_REPORT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi136));
                        if(LOOP) break;
                        return null;
                    case 137:
                        if(pack == null) return new SCALED_PRESSURE2();
                        final SCALED_PRESSURE2 pi137 = (SCALED_PRESSURE2) pack;
                        ph.setPack(pi137);
                        on_SCALED_PRESSURE2.forEach(h -> h.handle(CommunicationChannel.this, ph, pi137));
                        if(LOOP) break;
                        return null;
                    case 138:
                        if(pack == null) return new ATT_POS_MOCAP();
                        final ATT_POS_MOCAP pi138 = (ATT_POS_MOCAP) pack;
                        ph.setPack(pi138);
                        on_ATT_POS_MOCAP.forEach(h -> h.handle(CommunicationChannel.this, ph, pi138));
                        if(LOOP) break;
                        return null;
                    case 139:
                        if(pack == null) return new SET_ACTUATOR_CONTROL_TARGET();
                        final SET_ACTUATOR_CONTROL_TARGET pi139 = (SET_ACTUATOR_CONTROL_TARGET) pack;
                        ph.setPack(pi139);
                        on_SET_ACTUATOR_CONTROL_TARGET.forEach(h -> h.handle(CommunicationChannel.this, ph, pi139));
                        if(LOOP) break;
                        return null;
                    case 140:
                        if(pack == null) return new ACTUATOR_CONTROL_TARGET();
                        final ACTUATOR_CONTROL_TARGET pi140 = (ACTUATOR_CONTROL_TARGET) pack;
                        ph.setPack(pi140);
                        on_ACTUATOR_CONTROL_TARGET.forEach(h -> h.handle(CommunicationChannel.this, ph, pi140));
                        if(LOOP) break;
                        return null;
                    case 141:
                        if(pack == null) return new ALTITUDE();
                        final ALTITUDE pi141 = (ALTITUDE) pack;
                        ph.setPack(pi141);
                        on_ALTITUDE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi141));
                        if(LOOP) break;
                        return null;
                    case 142:
                        if(pack == null) return new RESOURCE_REQUEST();
                        final RESOURCE_REQUEST pi142 = (RESOURCE_REQUEST) pack;
                        ph.setPack(pi142);
                        on_RESOURCE_REQUEST.forEach(h -> h.handle(CommunicationChannel.this, ph, pi142));
                        if(LOOP) break;
                        return null;
                    case 143:
                        if(pack == null) return new SCALED_PRESSURE3();
                        final SCALED_PRESSURE3 pi143 = (SCALED_PRESSURE3) pack;
                        ph.setPack(pi143);
                        on_SCALED_PRESSURE3.forEach(h -> h.handle(CommunicationChannel.this, ph, pi143));
                        if(LOOP) break;
                        return null;
                    case 144:
                        if(pack == null) return new FOLLOW_TARGET();
                        final FOLLOW_TARGET pi144 = (FOLLOW_TARGET) pack;
                        ph.setPack(pi144);
                        on_FOLLOW_TARGET.forEach(h -> h.handle(CommunicationChannel.this, ph, pi144));
                        if(LOOP) break;
                        return null;
                    case 146:
                        if(pack == null) return new CONTROL_SYSTEM_STATE();
                        final CONTROL_SYSTEM_STATE pi146 = (CONTROL_SYSTEM_STATE) pack;
                        ph.setPack(pi146);
                        on_CONTROL_SYSTEM_STATE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi146));
                        if(LOOP) break;
                        return null;
                    case 147:
                        if(pack == null) return new BATTERY_STATUS();
                        final BATTERY_STATUS pi147 = (BATTERY_STATUS) pack;
                        ph.setPack(pi147);
                        on_BATTERY_STATUS.forEach(h -> h.handle(CommunicationChannel.this, ph, pi147));
                        if(LOOP) break;
                        return null;
                    case 148:
                        if(pack == null) return new AUTOPILOT_VERSION(-1);
                        final AUTOPILOT_VERSION pi148 = (AUTOPILOT_VERSION) pack;
                        ph.setPack(pi148);
                        on_AUTOPILOT_VERSION.forEach(h -> h.handle(CommunicationChannel.this, ph, pi148));
                        if(LOOP) break;
                        return null;
                    case 149:
                        if(pack == null) return new LANDING_TARGET(-1);
                        final LANDING_TARGET pi149 = (LANDING_TARGET) pack;
                        ph.setPack(pi149);
                        on_LANDING_TARGET.forEach(h -> h.handle(CommunicationChannel.this, ph, pi149));
                        if(LOOP) break;
                        return null;
                    case 230:
                        if(pack == null) return new ESTIMATOR_STATUS();
                        final ESTIMATOR_STATUS pi230 = (ESTIMATOR_STATUS) pack;
                        ph.setPack(pi230);
                        on_ESTIMATOR_STATUS.forEach(h -> h.handle(CommunicationChannel.this, ph, pi230));
                        if(LOOP) break;
                        return null;
                    case 231:
                        if(pack == null) return new WIND_COV();
                        final WIND_COV pi231 = (WIND_COV) pack;
                        ph.setPack(pi231);
                        on_WIND_COV.forEach(h -> h.handle(CommunicationChannel.this, ph, pi231));
                        if(LOOP) break;
                        return null;
                    case 232:
                        if(pack == null) return new GPS_INPUT();
                        final GPS_INPUT pi232 = (GPS_INPUT) pack;
                        ph.setPack(pi232);
                        on_GPS_INPUT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi232));
                        if(LOOP) break;
                        return null;
                    case 233:
                        if(pack == null) return new GPS_RTCM_DATA();
                        final GPS_RTCM_DATA pi233 = (GPS_RTCM_DATA) pack;
                        ph.setPack(pi233);
                        on_GPS_RTCM_DATA.forEach(h -> h.handle(CommunicationChannel.this, ph, pi233));
                        if(LOOP) break;
                        return null;
                    case 234:
                        if(pack == null) return new HIGH_LATENCY();
                        final HIGH_LATENCY pi234 = (HIGH_LATENCY) pack;
                        ph.setPack(pi234);
                        on_HIGH_LATENCY.forEach(h -> h.handle(CommunicationChannel.this, ph, pi234));
                        if(LOOP) break;
                        return null;
                    case 241:
                        if(pack == null) return new VIBRATION();
                        final VIBRATION pi241 = (VIBRATION) pack;
                        ph.setPack(pi241);
                        on_VIBRATION.forEach(h -> h.handle(CommunicationChannel.this, ph, pi241));
                        if(LOOP) break;
                        return null;
                    case 242:
                        if(pack == null) return new HOME_POSITION(-1);
                        final HOME_POSITION pi242 = (HOME_POSITION) pack;
                        ph.setPack(pi242);
                        on_HOME_POSITION.forEach(h -> h.handle(CommunicationChannel.this, ph, pi242));
                        if(LOOP) break;
                        return null;
                    case 243:
                        if(pack == null) return new SET_HOME_POSITION(-1);
                        final SET_HOME_POSITION pi243 = (SET_HOME_POSITION) pack;
                        ph.setPack(pi243);
                        on_SET_HOME_POSITION.forEach(h -> h.handle(CommunicationChannel.this, ph, pi243));
                        if(LOOP) break;
                        return null;
                    case 244:
                        if(pack == null) return new MESSAGE_INTERVAL();
                        final MESSAGE_INTERVAL pi244 = (MESSAGE_INTERVAL) pack;
                        ph.setPack(pi244);
                        on_MESSAGE_INTERVAL.forEach(h -> h.handle(CommunicationChannel.this, ph, pi244));
                        if(LOOP) break;
                        return null;
                    case 245:
                        if(pack == null) return new EXTENDED_SYS_STATE();
                        final EXTENDED_SYS_STATE pi245 = (EXTENDED_SYS_STATE) pack;
                        ph.setPack(pi245);
                        on_EXTENDED_SYS_STATE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi245));
                        if(LOOP) break;
                        return null;
                    case 246:
                        if(pack == null) return new ADSB_VEHICLE(-1);
                        final ADSB_VEHICLE pi246 = (ADSB_VEHICLE) pack;
                        ph.setPack(pi246);
                        on_ADSB_VEHICLE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi246));
                        if(LOOP) break;
                        return null;
                    case 247:
                        if(pack == null) return new COLLISION();
                        final COLLISION pi247 = (COLLISION) pack;
                        ph.setPack(pi247);
                        on_COLLISION.forEach(h -> h.handle(CommunicationChannel.this, ph, pi247));
                        if(LOOP) break;
                        return null;
                    case 248:
                        if(pack == null) return new V2_EXTENSION();
                        final V2_EXTENSION pi248 = (V2_EXTENSION) pack;
                        ph.setPack(pi248);
                        on_V2_EXTENSION.forEach(h -> h.handle(CommunicationChannel.this, ph, pi248));
                        if(LOOP) break;
                        return null;
                    case 249:
                        if(pack == null) return new MEMORY_VECT();
                        final MEMORY_VECT pi249 = (MEMORY_VECT) pack;
                        ph.setPack(pi249);
                        on_MEMORY_VECT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi249));
                        if(LOOP) break;
                        return null;
                    case 250:
                        if(pack == null) return new DEBUG_VECT(-1);
                        final DEBUG_VECT pi250 = (DEBUG_VECT) pack;
                        ph.setPack(pi250);
                        on_DEBUG_VECT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi250));
                        if(LOOP) break;
                        return null;
                    case 251:
                        if(pack == null) return new NAMED_VALUE_FLOAT(-1);
                        final NAMED_VALUE_FLOAT pi251 = (NAMED_VALUE_FLOAT) pack;
                        ph.setPack(pi251);
                        on_NAMED_VALUE_FLOAT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi251));
                        if(LOOP) break;
                        return null;
                    case 252:
                        if(pack == null) return new NAMED_VALUE_INT(-1);
                        final NAMED_VALUE_INT pi252 = (NAMED_VALUE_INT) pack;
                        ph.setPack(pi252);
                        on_NAMED_VALUE_INT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi252));
                        if(LOOP) break;
                        return null;
                    case 253:
                        if(pack == null) return new STATUSTEXT(-1);
                        final STATUSTEXT pi253 = (STATUSTEXT) pack;
                        ph.setPack(pi253);
                        on_STATUSTEXT.forEach(h -> h.handle(CommunicationChannel.this, ph, pi253));
                        if(LOOP) break;
                        return null;
                    case 254:
                        if(pack == null) return new DEBUG();
                        final DEBUG pi254 = (DEBUG) pack;
                        ph.setPack(pi254);
                        on_DEBUG.forEach(h -> h.handle(CommunicationChannel.this, ph, pi254));
                        if(LOOP) break;
                        return null;
                    case 256:
                        if(pack == null) return new SETUP_SIGNING();
                        final SETUP_SIGNING pi256 = (SETUP_SIGNING) pack;
                        ph.setPack(pi256);
                        on_SETUP_SIGNING.forEach(h -> h.handle(CommunicationChannel.this, ph, pi256));
                        if(LOOP) break;
                        return null;
                    case 257:
                        if(pack == null) return new BUTTON_CHANGE();
                        final BUTTON_CHANGE pi257 = (BUTTON_CHANGE) pack;
                        ph.setPack(pi257);
                        on_BUTTON_CHANGE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi257));
                        if(LOOP) break;
                        return null;
                    case 258:
                        if(pack == null) return new PLAY_TUNE(-1);
                        final PLAY_TUNE pi258 = (PLAY_TUNE) pack;
                        ph.setPack(pi258);
                        on_PLAY_TUNE.forEach(h -> h.handle(CommunicationChannel.this, ph, pi258));
                        if(LOOP) break;
                        return null;
                    case 259:
                        if(pack == null) return new CAMERA_INFORMATION(-1);
                        final CAMERA_INFORMATION pi259 = (CAMERA_INFORMATION) pack;
                        ph.setPack(pi259);
                        on_CAMERA_INFORMATION.forEach(h -> h.handle(CommunicationChannel.this, ph, pi259));
                        if(LOOP) break;
                        return null;
                    case 260:
                        if(pack == null) return new CAMERA_SETTINGS();
                        final CAMERA_SETTINGS pi260 = (CAMERA_SETTINGS) pack;
                        ph.setPack(pi260);
                        on_CAMERA_SETTINGS.forEach(h -> h.handle(CommunicationChannel.this, ph, pi260));
                        if(LOOP) break;
                        return null;
                    case 261:
                        if(pack == null) return new STORAGE_INFORMATION();
                        final STORAGE_INFORMATION pi261 = (STORAGE_INFORMATION) pack;
                        ph.setPack(pi261);
                        on_STORAGE_INFORMATION.forEach(h -> h.handle(CommunicationChannel.this, ph, pi261));
                        if(LOOP) break;
                        return null;
                    case 262:
                        if(pack == null) return new CAMERA_CAPTURE_STATUS();
                        final CAMERA_CAPTURE_STATUS pi262 = (CAMERA_CAPTURE_STATUS) pack;
                        ph.setPack(pi262);
                        on_CAMERA_CAPTURE_STATUS.forEach(h -> h.handle(CommunicationChannel.this, ph, pi262));
                        if(LOOP) break;
                        return null;
                    case 263:
                        if(pack == null) return new CAMERA_IMAGE_CAPTURED(-1);
                        final CAMERA_IMAGE_CAPTURED pi263 = (CAMERA_IMAGE_CAPTURED) pack;
                        ph.setPack(pi263);
                        on_CAMERA_IMAGE_CAPTURED.forEach(h -> h.handle(CommunicationChannel.this, ph, pi263));
                        if(LOOP) break;
                        return null;
                    case 264:
                        if(pack == null) return new FLIGHT_INFORMATION();
                        final FLIGHT_INFORMATION pi264 = (FLIGHT_INFORMATION) pack;
                        ph.setPack(pi264);
                        on_FLIGHT_INFORMATION.forEach(h -> h.handle(CommunicationChannel.this, ph, pi264));
                        if(LOOP) break;
                        return null;
                    case Channel.PROCESS_CHANNEL_REQEST:
                        if(pack == null) return sendout_packs.poll();
                        received_packs.add(pack);
                        return null;
                    case Channel.PROCESS_HOST_REQEST:
                        if(pack == null) return received_packs.poll();
                        sendout_packs.add(pack);
                        return null;
                    case Channel.PROCESS_RECEIVED:
                        LOOP = true;
                }
                if(received_packs.isEmpty()) return null;
                pack = received_packs.remove();
                id = pack.meta.id;
            }
        }
        /**
        *The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot
        *	 hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying
        *	 out the user interface based on the autopilot)*/
        public final Collection<OnReceive.Handler<HEARTBEAT, CommunicationChannel>> on_HEARTBEAT = new OnReceive<>();
        /**
        *The general system state. If the system is following the MAVLink standard, the system state is mainly
        *	 defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and
        *	 locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position
        *	 setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined
        *	 the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents
        *	 the internal navigation state machine. The system status shows whether the system is currently active
        *	 or not and if an emergency occured. During the CRITICAL and EMERGENCY states the MAV is still considered
        *	 to be active, but should start emergency procedures autonomously. After a failure occured it should first
        *	 move from active to critical to allow manual intervention and then move to emergency after a certain
        *	 timeout*/
        public final Collection<OnReceive.Handler<SYS_STATUS, CommunicationChannel>> on_SYS_STATUS = new OnReceive<>();
        /**
        *The system time is the time of the master clock, typically the computer clock of the main onboard computer*/
        public final Collection<OnReceive.Handler<SYSTEM_TIME, CommunicationChannel>> on_SYSTEM_TIME = new OnReceive<>();
        /**
        *Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
        *	 This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled
        *	 this way*/
        public final Collection<OnReceive.Handler<POSITION_TARGET_LOCAL_NED, CommunicationChannel>> on_POSITION_TARGET_LOCAL_NED = new OnReceive<>();
        /**
        *A ping message either requesting or responding to a ping. This allows to measure the system latencies,
        *	 including serial port, radio modem and UDP connections*/
        public final Collection<OnReceive.Handler<PING, CommunicationChannel>> on_PING = new OnReceive<>();
        /**
        *Request to control this MAV*/
        public final Collection<OnReceive.Handler<CHANGE_OPERATOR_CONTROL, CommunicationChannel>> on_CHANGE_OPERATOR_CONTROL = new OnReceive<>();
        /**
        *Accept / deny control of this MAV*/
        public final Collection<OnReceive.Handler<CHANGE_OPERATOR_CONTROL_ACK, CommunicationChannel>> on_CHANGE_OPERATOR_CONTROL_ACK = new OnReceive<>();
        /**
        *Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple,
        *	 so transmitting the key requires an encrypted channel for true safety*/
        public final Collection<OnReceive.Handler<AUTH_KEY, CommunicationChannel>> on_AUTH_KEY = new OnReceive<>();
        /**
        *THIS INTERFACE IS DEPRECATED. USE COMMAND_LONG with MAV_CMD_DO_SET_MODE INSTEAD. Set the system mode,
        *	 as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall
        *	 aircraft, not only for one component*/
        public final Collection<OnReceive.Handler<SET_MODE, CommunicationChannel>> on_SET_MODE = new OnReceive<>();
        /**
        *value[float]. This allows to send a parameter to any other component (such as the GCS) without the need
        *	 of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for
        *	 different autopilots. See also http:qgroundcontrol.org/parameter_interface for a full documentation
        *	 of QGroundControl and IMU code*/
        public final Collection<OnReceive.Handler<PARAM_REQUEST_READ, CommunicationChannel>> on_PARAM_REQUEST_READ = new OnReceive<>();
        /**
        *Request all parameters of this component. After this request, all parameters are emitted.*/
        public final Collection<OnReceive.Handler<PARAM_REQUEST_LIST, CommunicationChannel>> on_PARAM_REQUEST_LIST = new OnReceive<>();
        /**
        *Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows
        *	 the recipient to keep track of received parameters and allows him to re-request missing parameters after
        *	 a loss or timeout*/
        public final Collection<OnReceive.Handler<PARAM_VALUE, CommunicationChannel>> on_PARAM_VALUE = new OnReceive<>();
        /**
        *Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION
        *	 MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component
        *	 should acknowledge the new parameter value by sending a param_value message to all communication partners.
        *	 This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending
        *	 GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message*/
        public final Collection<OnReceive.Handler<PARAM_SET, CommunicationChannel>> on_PARAM_SET = new OnReceive<>();
        /**
        *The global position, as returned by the Global Positioning System (GPS). This is
        *	 NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
        public final Collection<OnReceive.Handler<GPS_RAW_INT, CommunicationChannel>> on_GPS_RAW_INT = new OnReceive<>();
        /**
        *The positioning status, as reported by GPS. This message is intended to display status information about
        *	 each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate.
        *	 This message can contain information for up to 20 satellites*/
        public final Collection<OnReceive.Handler<GPS_STATUS, CommunicationChannel>> on_GPS_STATUS = new OnReceive<>();
        /**
        *The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to
        *	 the described unit*/
        public final Collection<OnReceive.Handler<SCALED_IMU, CommunicationChannel>> on_SCALED_IMU = new OnReceive<>();
        /**
        *The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw
        *	 values without any scaling to allow data capture and system debugging*/
        public final Collection<OnReceive.Handler<RAW_IMU, CommunicationChannel>> on_RAW_IMU = new OnReceive<>();
        /**
        *The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure
        *	 sensor. The sensor values should be the raw, UNSCALED ADC values*/
        public final Collection<OnReceive.Handler<RAW_PRESSURE, CommunicationChannel>> on_RAW_PRESSURE = new OnReceive<>();
        /**
        *The pressure readings for the typical setup of one absolute and differential pressure sensor. The units
        *	 are as specified in each field*/
        public final Collection<OnReceive.Handler<SCALED_PRESSURE, CommunicationChannel>> on_SCALED_PRESSURE = new OnReceive<>();
        /**
        *The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).*/
        public final Collection<OnReceive.Handler<ATTITUDE, CommunicationChannel>> on_ATTITUDE = new OnReceive<>();
        /**
        *The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
        *	 Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)*/
        public final Collection<OnReceive.Handler<ATTITUDE_QUATERNION, CommunicationChannel>> on_ATTITUDE_QUATERNION = new OnReceive<>();
        /**
        *The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
        *	 Z-axis down (aeronautical frame, NED / north-east-down convention*/
        public final Collection<OnReceive.Handler<LOCAL_POSITION_NED, CommunicationChannel>> on_LOCAL_POSITION_NED = new OnReceive<>();
        /**
        *nt.*/
        public final Collection<OnReceive.Handler<GLOBAL_POSITION_INT, CommunicationChannel>> on_GLOBAL_POSITION_INT = new OnReceive<>();
        /**
        *The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are
        *	 inactive should be set to UINT16_MAX*/
        public final Collection<OnReceive.Handler<RC_CHANNELS_SCALED, CommunicationChannel>> on_RC_CHANNELS_SCALED = new OnReceive<>();
        /**
        *The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
        *	 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification*/
        public final Collection<OnReceive.Handler<RC_CHANNELS_RAW, CommunicationChannel>> on_RC_CHANNELS_RAW = new OnReceive<>();
        /**
        *The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The
        *	 standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%*/
        public final Collection<OnReceive.Handler<SERVO_OUTPUT_RAW, CommunicationChannel>> on_SERVO_OUTPUT_RAW = new OnReceive<>();
        /**
        *Request a partial list of mission items from the system/component. http:qgroundcontrol.org/mavlink/waypoint_protocol.
        *	 If start and end index are the same, just send one waypoint*/
        public final Collection<OnReceive.Handler<MISSION_REQUEST_PARTIAL_LIST, CommunicationChannel>> on_MISSION_REQUEST_PARTIAL_LIST = new OnReceive<>();
        /**
        *This message is sent to the MAV to write a partial list. If start index == end index, only one item will
        *	 be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should
        *	 be REJECTED*/
        public final Collection<OnReceive.Handler<MISSION_WRITE_PARTIAL_LIST, CommunicationChannel>> on_MISSION_WRITE_PARTIAL_LIST = new OnReceive<>();
        /**
        *Message encoding a mission item. This message is emitted to announce
        *	 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http:qgroundcontrol.org/mavlink/waypoint_protocol.*/
        public final Collection<OnReceive.Handler<MISSION_ITEM, CommunicationChannel>> on_MISSION_ITEM = new OnReceive<>();
        /**
        *Request the information of the mission item with the sequence number seq. The response of the system to
        *	 this message should be a MISSION_ITEM message. http:qgroundcontrol.org/mavlink/waypoint_protoco*/
        public final Collection<OnReceive.Handler<MISSION_REQUEST, CommunicationChannel>> on_MISSION_REQUEST = new OnReceive<>();
        /**
        *Set the mission item with sequence number seq as current item. This means that the MAV will continue to
        *	 this mission item on the shortest path (not following the mission items in-between)*/
        public final Collection<OnReceive.Handler<MISSION_SET_CURRENT, CommunicationChannel>> on_MISSION_SET_CURRENT = new OnReceive<>();
        /**
        *Message that announces the sequence number of the current active mission item. The MAV will fly towards
        *	 this mission item*/
        public final Collection<OnReceive.Handler<MISSION_CURRENT, CommunicationChannel>> on_MISSION_CURRENT = new OnReceive<>();
        /**
        *Request the overall list of mission items from the system/component.*/
        public final Collection<OnReceive.Handler<MISSION_REQUEST_LIST, CommunicationChannel>> on_MISSION_REQUEST_LIST = new OnReceive<>();
        /**
        *This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction.
        *	 The GCS can then request the individual mission item based on the knowledge of the total number of waypoints*/
        public final Collection<OnReceive.Handler<MISSION_COUNT, CommunicationChannel>> on_MISSION_COUNT = new OnReceive<>();
        /**
        *Delete all mission items at once.*/
        public final Collection<OnReceive.Handler<MISSION_CLEAR_ALL, CommunicationChannel>> on_MISSION_CLEAR_ALL = new OnReceive<>();
        /**
        *A certain mission item has been reached. The system will either hold this position (or circle on the orbit)
        *	 or (if the autocontinue on the WP was set) continue to the next waypoint*/
        public final Collection<OnReceive.Handler<MISSION_ITEM_REACHED, CommunicationChannel>> on_MISSION_ITEM_REACHED = new OnReceive<>();
        /**
        *Ack message during waypoint handling. The type field states if this message is a positive ack (type=0)
        *	 or if an error happened (type=non-zero)*/
        public final Collection<OnReceive.Handler<MISSION_ACK, CommunicationChannel>> on_MISSION_ACK = new OnReceive<>();
        /**
        *As local waypoints exist, the global waypoint reference allows to transform between the local coordinate
        *	 frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings
        *	 are connected and the MAV should move from in- to outdoor*/
        public final Collection<OnReceive.Handler<SET_GPS_GLOBAL_ORIGIN, CommunicationChannel>> on_SET_GPS_GLOBAL_ORIGIN = new OnReceive<>();
        /**
        *Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) positio*/
        public final Collection<OnReceive.Handler<GPS_GLOBAL_ORIGIN, CommunicationChannel>> on_GPS_GLOBAL_ORIGIN = new OnReceive<>();
        /**
        *Bind a RC channel to a parameter. The parameter should change accoding to the RC channel value.*/
        public final Collection<OnReceive.Handler<PARAM_MAP_RC, CommunicationChannel>> on_PARAM_MAP_RC = new OnReceive<>();
        /**
        *Request the information of the mission item with the sequence number seq. The response of the system to
        *	 this message should be a MISSION_ITEM_INT message. http:qgroundcontrol.org/mavlink/waypoint_protoco*/
        public final Collection<OnReceive.Handler<MISSION_REQUEST_INT, CommunicationChannel>> on_MISSION_REQUEST_INT = new OnReceive<>();
        /**
        *Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell
        *	 the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national
        *	 or competition regulations*/
        public final Collection<OnReceive.Handler<SAFETY_SET_ALLOWED_AREA, CommunicationChannel>> on_SAFETY_SET_ALLOWED_AREA = new OnReceive<>();
        /**
        *Read out the safety zone the MAV currently assumes.*/
        public final Collection<OnReceive.Handler<SAFETY_ALLOWED_AREA, CommunicationChannel>> on_SAFETY_ALLOWED_AREA = new OnReceive<>();
        /**
        *The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
        *	 Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)*/
        public final Collection<OnReceive.Handler<ATTITUDE_QUATERNION_COV, CommunicationChannel>> on_ATTITUDE_QUATERNION_COV = new OnReceive<>();
        /**
        *The state of the fixed wing navigation and position controller.*/
        public final Collection<OnReceive.Handler<NAV_CONTROLLER_OUTPUT, CommunicationChannel>> on_NAV_CONTROLLER_OUTPUT = new OnReceive<>();
        /**
        *The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed,
        *	 Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE:
        *	 This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized
        *	 for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset*/
        public final Collection<OnReceive.Handler<GLOBAL_POSITION_INT_COV, CommunicationChannel>> on_GLOBAL_POSITION_INT_COV = new OnReceive<>();
        /**
        *The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
        *	 Z-axis down (aeronautical frame, NED / north-east-down convention*/
        public final Collection<OnReceive.Handler<LOCAL_POSITION_NED_COV, CommunicationChannel>> on_LOCAL_POSITION_NED_COV = new OnReceive<>();
        /**
        *The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
        *	 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification*/
        public final Collection<OnReceive.Handler<RC_CHANNELS, CommunicationChannel>> on_RC_CHANNELS = new OnReceive<>();
        /**
        *THIS INTERFACE IS DEPRECATED. USE SET_MESSAGE_INTERVAL INSTEAD.*/
        public final Collection<OnReceive.Handler<REQUEST_DATA_STREAM, CommunicationChannel>> on_REQUEST_DATA_STREAM = new OnReceive<>();
        /**
        *THIS INTERFACE IS DEPRECATED. USE MESSAGE_INTERVAL INSTEAD.*/
        public final Collection<OnReceive.Handler<DATA_STREAM, CommunicationChannel>> on_DATA_STREAM = new OnReceive<>();
        /**
        *This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature,
        *	 along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as
        *	 boolean values of their*/
        public final Collection<OnReceive.Handler<MANUAL_CONTROL, CommunicationChannel>> on_MANUAL_CONTROL = new OnReceive<>();
        /**
        *The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value
        *	 of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released
        *	 back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds:
        *	 100%. Individual receivers/transmitters might violate this specification*/
        public final Collection<OnReceive.Handler<RC_CHANNELS_OVERRIDE, CommunicationChannel>> on_RC_CHANNELS_OVERRIDE = new OnReceive<>();
        /**
        *Message encoding a mission item. This message is emitted to announce
        *	 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See alsohttp:qgroundcontrol.org/mavlink/waypoint_protocol.*/
        public final Collection<OnReceive.Handler<MISSION_ITEM_INT, CommunicationChannel>> on_MISSION_ITEM_INT = new OnReceive<>();
        /**
        *Metrics typically displayed on a HUD for fixed wing aircraft*/
        public final Collection<OnReceive.Handler<VFR_HUD, CommunicationChannel>> on_VFR_HUD = new OnReceive<>();
        /**
        *Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value*/
        public final Collection<OnReceive.Handler<COMMAND_INT, CommunicationChannel>> on_COMMAND_INT = new OnReceive<>();
        /**
        *Send a command with up to seven parameters to the MAV*/
        public final Collection<OnReceive.Handler<COMMAND_LONG, CommunicationChannel>> on_COMMAND_LONG = new OnReceive<>();
        /**
        *Report status of a command. Includes feedback whether the command was executed.*/
        public final Collection<OnReceive.Handler<COMMAND_ACK, CommunicationChannel>> on_COMMAND_ACK = new OnReceive<>();
        /**
        *Setpoint in roll, pitch, yaw and thrust from the operator*/
        public final Collection<OnReceive.Handler<MANUAL_SETPOINT, CommunicationChannel>> on_MANUAL_SETPOINT = new OnReceive<>();
        /**
        *Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller
        *	 or other system)*/
        public final Collection<OnReceive.Handler<SET_ATTITUDE_TARGET, CommunicationChannel>> on_SET_ATTITUDE_TARGET = new OnReceive<>();
        /**
        *Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match
        *	 the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way*/
        public final Collection<OnReceive.Handler<ATTITUDE_TARGET, CommunicationChannel>> on_ATTITUDE_TARGET = new OnReceive<>();
        /**
        *Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller
        *	 to command the vehicle (manual controller or other system)*/
        public final Collection<OnReceive.Handler<SET_POSITION_TARGET_LOCAL_NED, CommunicationChannel>> on_SET_POSITION_TARGET_LOCAL_NED = new OnReceive<>();
        /**
        *Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84).
        *	 Used by an external controller to command the vehicle (manual controller or other system)*/
        public final Collection<OnReceive.Handler<SET_POSITION_TARGET_GLOBAL_INT, CommunicationChannel>> on_SET_POSITION_TARGET_GLOBAL_INT = new OnReceive<>();
        /**
        *Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
        *	 This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled
        *	 this way*/
        public final Collection<OnReceive.Handler<POSITION_TARGET_GLOBAL_INT, CommunicationChannel>> on_POSITION_TARGET_GLOBAL_INT = new OnReceive<>();
        /**
        *The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate
        *	 frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down
        *	 convention*/
        public final Collection<OnReceive.Handler<LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET, CommunicationChannel>> on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET = new OnReceive<>();
        /**
        *DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please
        *	 use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput
        *	 applications such as hardware in the loop simulations*/
        public final Collection<OnReceive.Handler<HIL_STATE, CommunicationChannel>> on_HIL_STATE = new OnReceive<>();
        /**
        *Sent from autopilot to simulation. Hardware in the loop control outputs*/
        public final Collection<OnReceive.Handler<HIL_CONTROLS, CommunicationChannel>> on_HIL_CONTROLS = new OnReceive<>();
        /**
        *Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation
        *	 is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might
        *	 violate this specification*/
        public final Collection<OnReceive.Handler<HIL_RC_INPUTS_RAW, CommunicationChannel>> on_HIL_RC_INPUTS_RAW = new OnReceive<>();
        /**
        *Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS*/
        public final Collection<OnReceive.Handler<HIL_ACTUATOR_CONTROLS, CommunicationChannel>> on_HIL_ACTUATOR_CONTROLS = new OnReceive<>();
        /**
        *Optical flow from a flow sensor (e.g. optical mouse sensor)*/
        public final Collection<OnReceive.Handler<OPTICAL_FLOW, CommunicationChannel>> on_OPTICAL_FLOW = new OnReceive<>();
        public final Collection<OnReceive.Handler<GLOBAL_VISION_POSITION_ESTIMATE, CommunicationChannel>> on_GLOBAL_VISION_POSITION_ESTIMATE = new OnReceive<>();
        public final Collection<OnReceive.Handler<VISION_POSITION_ESTIMATE, CommunicationChannel>> on_VISION_POSITION_ESTIMATE = new OnReceive<>();
        public final Collection<OnReceive.Handler<VISION_SPEED_ESTIMATE, CommunicationChannel>> on_VISION_SPEED_ESTIMATE = new OnReceive<>();
        public final Collection<OnReceive.Handler<VICON_POSITION_ESTIMATE, CommunicationChannel>> on_VICON_POSITION_ESTIMATE = new OnReceive<>();
        /**
        *The IMU readings in SI units in NED body frame*/
        public final Collection<OnReceive.Handler<HIGHRES_IMU, CommunicationChannel>> on_HIGHRES_IMU = new OnReceive<>();
        /**
        *Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)*/
        public final Collection<OnReceive.Handler<OPTICAL_FLOW_RAD, CommunicationChannel>> on_OPTICAL_FLOW_RAD = new OnReceive<>();
        /**
        *The IMU readings in SI units in NED body frame*/
        public final Collection<OnReceive.Handler<HIL_SENSOR, CommunicationChannel>> on_HIL_SENSOR = new OnReceive<>();
        /**
        *Status of simulation environment, if used*/
        public final Collection<OnReceive.Handler<SIM_STATE, CommunicationChannel>> on_SIM_STATE = new OnReceive<>();
        /**
        *Status generated by radio and injected into MAVLink stream.*/
        public final Collection<OnReceive.Handler<RADIO_STATUS, CommunicationChannel>> on_RADIO_STATUS = new OnReceive<>();
        /**
        *File transfer message*/
        public final Collection<OnReceive.Handler<FILE_TRANSFER_PROTOCOL, CommunicationChannel>> on_FILE_TRANSFER_PROTOCOL = new OnReceive<>();
        /**
        *Time synchronization message.*/
        public final Collection<OnReceive.Handler<TIMESYNC, CommunicationChannel>> on_TIMESYNC = new OnReceive<>();
        /**
        *Camera-IMU triggering and synchronisation message.*/
        public final Collection<OnReceive.Handler<CAMERA_TRIGGER, CommunicationChannel>> on_CAMERA_TRIGGER = new OnReceive<>();
        /**
        *The global position, as returned by the Global Positioning System (GPS). This is
        *	 NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
        public final Collection<OnReceive.Handler<HIL_GPS, CommunicationChannel>> on_HIL_GPS = new OnReceive<>();
        /**
        *Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)*/
        public final Collection<OnReceive.Handler<HIL_OPTICAL_FLOW, CommunicationChannel>> on_HIL_OPTICAL_FLOW = new OnReceive<>();
        /**
        *Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful
        *	 for high throughput applications such as hardware in the loop simulations*/
        public final Collection<OnReceive.Handler<HIL_STATE_QUATERNION, CommunicationChannel>> on_HIL_STATE_QUATERNION = new OnReceive<>();
        /**
        *The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to
        *	 the described unit*/
        public final Collection<OnReceive.Handler<SCALED_IMU2, CommunicationChannel>> on_SCALED_IMU2 = new OnReceive<>();
        /**
        *Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END
        *	 is called*/
        public final Collection<OnReceive.Handler<LOG_REQUEST_LIST, CommunicationChannel>> on_LOG_REQUEST_LIST = new OnReceive<>();
        /**
        *Reply to LOG_REQUEST_LIST*/
        public final Collection<OnReceive.Handler<LOG_ENTRY, CommunicationChannel>> on_LOG_ENTRY = new OnReceive<>();
        /**
        *Request a chunk of a log*/
        public final Collection<OnReceive.Handler<LOG_REQUEST_DATA, CommunicationChannel>> on_LOG_REQUEST_DATA = new OnReceive<>();
        /**
        *Reply to LOG_REQUEST_DATA*/
        public final Collection<OnReceive.Handler<LOG_DATA, CommunicationChannel>> on_LOG_DATA = new OnReceive<>();
        /**
        *Erase all logs*/
        public final Collection<OnReceive.Handler<LOG_ERASE, CommunicationChannel>> on_LOG_ERASE = new OnReceive<>();
        /**
        *Stop log transfer and resume normal logging*/
        public final Collection<OnReceive.Handler<LOG_REQUEST_END, CommunicationChannel>> on_LOG_REQUEST_END = new OnReceive<>();
        /**
        *data for injecting into the onboard GPS (used for DGPS)*/
        public final Collection<OnReceive.Handler<GPS_INJECT_DATA, CommunicationChannel>> on_GPS_INJECT_DATA = new OnReceive<>();
        /**
        *Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
        public final Collection<OnReceive.Handler<GPS2_RAW, CommunicationChannel>> on_GPS2_RAW = new OnReceive<>();
        /**
        *Power supply status*/
        public final Collection<OnReceive.Handler<POWER_STATUS, CommunicationChannel>> on_POWER_STATUS = new OnReceive<>();
        /**
        *Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or
        *	 telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages
        *	 or change the devices settings. A message with zero bytes can be used to change just the baudrate*/
        public final Collection<OnReceive.Handler<SERIAL_CONTROL, CommunicationChannel>> on_SERIAL_CONTROL = new OnReceive<>();
        /**
        *RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting*/
        public final Collection<OnReceive.Handler<GPS_RTK, CommunicationChannel>> on_GPS_RTK = new OnReceive<>();
        /**
        *RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting*/
        public final Collection<OnReceive.Handler<GPS2_RTK, CommunicationChannel>> on_GPS2_RTK = new OnReceive<>();
        /**
        *The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described
        *	 unit*/
        public final Collection<OnReceive.Handler<SCALED_IMU3, CommunicationChannel>> on_SCALED_IMU3 = new OnReceive<>();
        public final Collection<OnReceive.Handler<DATA_TRANSMISSION_HANDSHAKE, CommunicationChannel>> on_DATA_TRANSMISSION_HANDSHAKE = new OnReceive<>();
        public final Collection<OnReceive.Handler<ENCAPSULATED_DATA, CommunicationChannel>> on_ENCAPSULATED_DATA = new OnReceive<>();
        public final Collection<OnReceive.Handler<DISTANCE_SENSOR, CommunicationChannel>> on_DISTANCE_SENSOR = new OnReceive<>();
        /**
        *Request for terrain data and terrain status*/
        public final Collection<OnReceive.Handler<TERRAIN_REQUEST, CommunicationChannel>> on_TERRAIN_REQUEST = new OnReceive<>();
        /**
        *Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUES*/
        public final Collection<OnReceive.Handler<TERRAIN_DATA, CommunicationChannel>> on_TERRAIN_DATA = new OnReceive<>();
        /**
        *Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle
        *	 has all terrain data needed for a mission*/
        public final Collection<OnReceive.Handler<TERRAIN_CHECK, CommunicationChannel>> on_TERRAIN_CHECK = new OnReceive<>();
        /**
        *Response from a TERRAIN_CHECK request*/
        public final Collection<OnReceive.Handler<TERRAIN_REPORT, CommunicationChannel>> on_TERRAIN_REPORT = new OnReceive<>();
        /**
        *Barometer readings for 2nd barometer*/
        public final Collection<OnReceive.Handler<SCALED_PRESSURE2, CommunicationChannel>> on_SCALED_PRESSURE2 = new OnReceive<>();
        /**
        *Motion capture attitude and position*/
        public final Collection<OnReceive.Handler<ATT_POS_MOCAP, CommunicationChannel>> on_ATT_POS_MOCAP = new OnReceive<>();
        /**
        *Set the vehicle attitude and body angular rates.*/
        public final Collection<OnReceive.Handler<SET_ACTUATOR_CONTROL_TARGET, CommunicationChannel>> on_SET_ACTUATOR_CONTROL_TARGET = new OnReceive<>();
        /**
        *Set the vehicle attitude and body angular rates.*/
        public final Collection<OnReceive.Handler<ACTUATOR_CONTROL_TARGET, CommunicationChannel>> on_ACTUATOR_CONTROL_TARGET = new OnReceive<>();
        /**
        *The current system altitude.*/
        public final Collection<OnReceive.Handler<ALTITUDE, CommunicationChannel>> on_ALTITUDE = new OnReceive<>();
        /**
        *The autopilot is requesting a resource (file, binary, other type of data)*/
        public final Collection<OnReceive.Handler<RESOURCE_REQUEST, CommunicationChannel>> on_RESOURCE_REQUEST = new OnReceive<>();
        /**
        *Barometer readings for 3rd barometer*/
        public final Collection<OnReceive.Handler<SCALED_PRESSURE3, CommunicationChannel>> on_SCALED_PRESSURE3 = new OnReceive<>();
        /**
        *current motion information from a designated system*/
        public final Collection<OnReceive.Handler<FOLLOW_TARGET, CommunicationChannel>> on_FOLLOW_TARGET = new OnReceive<>();
        /**
        *The smoothed, monotonic system state used to feed the control loops of the system.*/
        public final Collection<OnReceive.Handler<CONTROL_SYSTEM_STATE, CommunicationChannel>> on_CONTROL_SYSTEM_STATE = new OnReceive<>();
        /**
        *Battery information*/
        public final Collection<OnReceive.Handler<BATTERY_STATUS, CommunicationChannel>> on_BATTERY_STATUS = new OnReceive<>();
        /**
        *Version and capability of autopilot software*/
        public final Collection<OnReceive.Handler<AUTOPILOT_VERSION, CommunicationChannel>> on_AUTOPILOT_VERSION = new OnReceive<>();
        /**
        *The location of a landing area captured from a downward facing camera*/
        public final Collection<OnReceive.Handler<LANDING_TARGET, CommunicationChannel>> on_LANDING_TARGET = new OnReceive<>();
        /**
        *Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message
        *	 is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS
        *	 enum definition for further information. The innovaton test ratios show the magnitude of the sensor innovation
        *	 divided by the innovation check threshold. Under normal operation the innovaton test ratios should be
        *	 below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation
        *	 and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation
        *	 test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should
        *	 be optional and controllable by the user*/
        public final Collection<OnReceive.Handler<ESTIMATOR_STATUS, CommunicationChannel>> on_ESTIMATOR_STATUS = new OnReceive<>();
        public final Collection<OnReceive.Handler<WIND_COV, CommunicationChannel>> on_WIND_COV = new OnReceive<>();
        /**
        *GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position
        *	 estimate of the sytem*/
        public final Collection<OnReceive.Handler<GPS_INPUT, CommunicationChannel>> on_GPS_INPUT = new OnReceive<>();
        /**
        *RTCM message for injecting into the onboard GPS (used for DGPS)*/
        public final Collection<OnReceive.Handler<GPS_RTCM_DATA, CommunicationChannel>> on_GPS_RTCM_DATA = new OnReceive<>();
        /**
        *Message appropriate for high latency connections like Iridium*/
        public final Collection<OnReceive.Handler<HIGH_LATENCY, CommunicationChannel>> on_HIGH_LATENCY = new OnReceive<>();
        /**
        *Vibration levels and accelerometer clipping*/
        public final Collection<OnReceive.Handler<VIBRATION, CommunicationChannel>> on_VIBRATION = new OnReceive<>();
        /**
        *This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system
        *	 will return to and land on. The position is set automatically by the system during the takeoff in case
        *	 it was not explicitely set by the operator before or after. The position the system will return to and
        *	 land on. The global and local positions encode the position in the respective coordinate frames, while
        *	 the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading
        *	 and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes
        *	 the point to which the system should fly in normal flight mode and then perform a landing sequence along
        *	 the vector*/
        public final Collection<OnReceive.Handler<HOME_POSITION, CommunicationChannel>> on_HOME_POSITION = new OnReceive<>();
        /**
        *The position the system will return to and land on. The position is set automatically by the system during
        *	 the takeoff in case it was not explicitely set by the operator before or after. The global and local
        *	 positions encode the position in the respective coordinate frames, while the q parameter encodes the
        *	 orientation of the surface. Under normal conditions it describes the heading and terrain slope, which
        *	 can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which
        *	 the system should fly in normal flight mode and then perform a landing sequence along the vector*/
        public final Collection<OnReceive.Handler<SET_HOME_POSITION, CommunicationChannel>> on_SET_HOME_POSITION = new OnReceive<>();
        /**
        *This interface replaces DATA_STREAM*/
        public final Collection<OnReceive.Handler<MESSAGE_INTERVAL, CommunicationChannel>> on_MESSAGE_INTERVAL = new OnReceive<>();
        /**
        *Provides state for additional features*/
        public final Collection<OnReceive.Handler<EXTENDED_SYS_STATE, CommunicationChannel>> on_EXTENDED_SYS_STATE = new OnReceive<>();
        /**
        *The location and information of an ADSB vehicle*/
        public final Collection<OnReceive.Handler<ADSB_VEHICLE, CommunicationChannel>> on_ADSB_VEHICLE = new OnReceive<>();
        /**
        *Information about a potential collision*/
        public final Collection<OnReceive.Handler<COLLISION, CommunicationChannel>> on_COLLISION = new OnReceive<>();
        /**
        *Message implementing parts of the V2 payload specs in V1 frames for transitional support.*/
        public final Collection<OnReceive.Handler<V2_EXTENSION, CommunicationChannel>> on_V2_EXTENSION = new OnReceive<>();
        /**
        *Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient
        *	 way for testing new messages and getting experimental debug output*/
        public final Collection<OnReceive.Handler<MEMORY_VECT, CommunicationChannel>> on_MEMORY_VECT = new OnReceive<>();
        public final Collection<OnReceive.Handler<DEBUG_VECT, CommunicationChannel>> on_DEBUG_VECT = new OnReceive<>();
        /**
        *Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite
        *	 efficient way for testing new messages and getting experimental debug output*/
        public final Collection<OnReceive.Handler<NAMED_VALUE_FLOAT, CommunicationChannel>> on_NAMED_VALUE_FLOAT = new OnReceive<>();
        /**
        *Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite
        *	 efficient way for testing new messages and getting experimental debug output*/
        public final Collection<OnReceive.Handler<NAMED_VALUE_INT, CommunicationChannel>> on_NAMED_VALUE_INT = new OnReceive<>();
        /**
        *Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING:
        *	 They consume quite some bandwidth, so use only for important status and error messages. If implemented
        *	 wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz)*/
        public final Collection<OnReceive.Handler<STATUSTEXT, CommunicationChannel>> on_STATUSTEXT = new OnReceive<>();
        /**
        *Send a debug value. The index is used to discriminate between values. These values show up in the plot
        *	 of QGroundControl as DEBUG N*/
        public final Collection<OnReceive.Handler<DEBUG, CommunicationChannel>> on_DEBUG = new OnReceive<>();
        /**
        *Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will disable
        *	 signin*/
        public final Collection<OnReceive.Handler<SETUP_SIGNING, CommunicationChannel>> on_SETUP_SIGNING = new OnReceive<>();
        /**
        *Report button state change*/
        public final Collection<OnReceive.Handler<BUTTON_CHANGE, CommunicationChannel>> on_BUTTON_CHANGE = new OnReceive<>();
        /**
        *Control vehicle tone generation (buzzer)*/
        public final Collection<OnReceive.Handler<PLAY_TUNE, CommunicationChannel>> on_PLAY_TUNE = new OnReceive<>();
        /**
        *WIP: Information about a camera*/
        public final Collection<OnReceive.Handler<CAMERA_INFORMATION, CommunicationChannel>> on_CAMERA_INFORMATION = new OnReceive<>();
        /**
        *WIP: Settings of a camera, can be requested using MAV_CMD_REQUEST_CAMERA_SETTINGS.*/
        public final Collection<OnReceive.Handler<CAMERA_SETTINGS, CommunicationChannel>> on_CAMERA_SETTINGS = new OnReceive<>();
        /**
        *WIP: Information about a storage medium.*/
        public final Collection<OnReceive.Handler<STORAGE_INFORMATION, CommunicationChannel>> on_STORAGE_INFORMATION = new OnReceive<>();
        /**
        *WIP: Information about the status of a capture*/
        public final Collection<OnReceive.Handler<CAMERA_CAPTURE_STATUS, CommunicationChannel>> on_CAMERA_CAPTURE_STATUS = new OnReceive<>();
        /**
        *Information about a captured image*/
        public final Collection<OnReceive.Handler<CAMERA_IMAGE_CAPTURED, CommunicationChannel>> on_CAMERA_IMAGE_CAPTURED = new OnReceive<>();
        /**
        *WIP: Information about flight since last arming*/
        public final Collection<OnReceive.Handler<FLIGHT_INFORMATION, CommunicationChannel>> on_FLIGHT_INFORMATION = new OnReceive<>();
    }


    public @interface MAV_TYPE
    {

        int
        MAV_TYPE_GENERIC = 0,//Generic micro air vehicle.
        MAV_TYPE_FIXED_WING = 1,//Fixed wing aircraft.
        MAV_TYPE_QUADROTOR = 2,//Quadrotor
        MAV_TYPE_COAXIAL = 3,//Coaxial helicopter
        MAV_TYPE_HELICOPTER = 4,//Normal helicopter with tail rotor.
        MAV_TYPE_ANTENNA_TRACKER = 5,//Ground installation
        MAV_TYPE_GCS = 6,//Operator control unit / ground control station
        MAV_TYPE_AIRSHIP = 7,//Airship, controlled
        MAV_TYPE_FREE_BALLOON = 8,//Free balloon, uncontrolled
        MAV_TYPE_ROCKET = 9,//Rocket
        MAV_TYPE_GROUND_ROVER = 10,//Ground rover
        MAV_TYPE_SURFACE_BOAT = 11,//Surface vessel, boat, ship
        MAV_TYPE_SUBMARINE = 12,//Submarine
        MAV_TYPE_HEXAROTOR = 13,//Hexarotor
        MAV_TYPE_OCTOROTOR = 14,//Octorotor
        MAV_TYPE_TRICOPTER = 15,//Tricopter
        MAV_TYPE_FLAPPING_WING = 16,//Flapping wing
        MAV_TYPE_KITE = 17,//Kite
        MAV_TYPE_ONBOARD_CONTROLLER = 18,//Onboard companion controller
        MAV_TYPE_VTOL_DUOROTOR = 19,//Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
        MAV_TYPE_VTOL_QUADROTOR = 20,//Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
        MAV_TYPE_VTOL_TILTROTOR = 21,//Tiltrotor VTOL
        MAV_TYPE_VTOL_RESERVED2 = 22,//VTOL reserved 2
        MAV_TYPE_VTOL_RESERVED3 = 23,//VTOL reserved 3
        MAV_TYPE_VTOL_RESERVED4 = 24,//VTOL reserved 4
        MAV_TYPE_VTOL_RESERVED5 = 25,//VTOL reserved 5
        MAV_TYPE_GIMBAL = 26,//Onboard gimbal
        MAV_TYPE_ADSB = 27,//Onboard ADSB peripheral
        MAV_TYPE_PARAFOIL = 28;//Steerable, nonrigid airfoil
    }

    /**
    *Micro air vehicle / autopilot classes. This identifies the individual model.*/
    public @interface MAV_AUTOPILOT
    {

        int
        MAV_AUTOPILOT_GENERIC = 0,//Generic autopilot, full support for everything
        MAV_AUTOPILOT_RESERVED = 1,//Reserved for future use.
        MAV_AUTOPILOT_SLUGS = 2,//SLUGS autopilot, http:slugsuav.soe.ucsc.edu
        MAV_AUTOPILOT_ARDUPILOTMEGA = 3,//ArduPilotMega / ArduCopter, http:diydrones.com
        MAV_AUTOPILOT_OPENPILOT = 4,//OpenPilot, http:openpilot.org
        MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5,//Generic autopilot only supporting simple waypoints
        MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6,//Generic autopilot supporting waypoints and other simple navigation commands
        MAV_AUTOPILOT_GENERIC_MISSION_FULL = 7,//Generic autopilot supporting the full mission command set
        MAV_AUTOPILOT_INVALID = 8,//No valid autopilot, e.g. a GCS or other MAVLink component
        MAV_AUTOPILOT_PPZ = 9,//PPZ UAV - http:nongnu.org/paparazzi
        MAV_AUTOPILOT_UDB = 10,//UAV Dev Board
        MAV_AUTOPILOT_FP = 11,//FlexiPilot
        MAV_AUTOPILOT_PX4 = 12,//PX4 Autopilot - http:pixhawk.ethz.ch/px4/
        MAV_AUTOPILOT_SMACCMPILOT = 13,//SMACCMPilot - http:smaccmpilot.org
        MAV_AUTOPILOT_AUTOQUAD = 14,//AutoQuad -- http:autoquad.org
        MAV_AUTOPILOT_ARMAZILA = 15,//Armazila -- http:armazila.com
        MAV_AUTOPILOT_AEROB = 16,//Aerob -- http:aerob.ru
        MAV_AUTOPILOT_ASLUAV = 17,//ASLUAV autopilot -- http:www.asl.ethz.ch
        MAV_AUTOPILOT_SMARTAP = 18;//SmartAP Autopilot - http:sky-drones.com
    }

    /**
    *These flags encode the MAV mode.*/
    public @interface MAV_MODE_FLAG
    {

        int
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1,//0b00000001 Reserved for future use.
        /**
        ** 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should
        *		 * not be used for stable implementations*/
        MAV_MODE_FLAG_TEST_ENABLED = 2,
        /**
        ** 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not,
        *		 * depends on the actual implementation*/
        MAV_MODE_FLAG_AUTO_ENABLED = 4,
        MAV_MODE_FLAG_GUIDED_ENABLED = 8,//0b00001000 guided mode enabled, system flies waypoints / mission items.
        /**
        ** 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further
        *		 * control inputs to move around*/
        MAV_MODE_FLAG_STABILIZE_ENABLED = 16,
        /**
        ** 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software
        *		 * is full operational*/
        MAV_MODE_FLAG_HIL_ENABLED = 32,
        MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,//0b01000000 remote control input is enabled.
        /**
        *0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional
        *				 note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM
        *				 shall be used instead. The flag can still be used to report the armed state*/
        MAV_MODE_FLAG_SAFETY_ARMED = 128;
    }

    public @interface MAV_STATE
    {

        int
        MAV_STATE_UNINIT = 0,//Uninitialized system, state is unknown.
        MAV_STATE_BOOT = 1,//System is booting up.
        MAV_STATE_CALIBRATING = 2,//System is calibrating and not flight-ready.
        MAV_STATE_STANDBY = 3,//System is grounded and on standby. It can be launched any time.
        MAV_STATE_ACTIVE = 4,//System is active and might be already airborne. Motors are engaged.
        MAV_STATE_CRITICAL = 5,//System is in a non-normal flight mode. It can however still navigate.
        /**
        *System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in
        *	 mayday and going down*/
        MAV_STATE_EMERGENCY = 6,
        MAV_STATE_POWEROFF = 7,//System just initialized its power-down sequence, will shut down now.
        MAV_STATE_FLIGHT_TERMINATION = 8;//System is terminating itself.
    }

    /**
    *These encode the sensors whose status is sent as part of the SYS_STATUS message.*/
    public @interface MAV_SYS_STATUS_SENSOR
    {

        int
        MAV_SYS_STATUS_SENSOR_3D_GYRO = 1,//0x01 3D gyro
        MAV_SYS_STATUS_SENSOR_3D_ACCEL = 2,//0x02 3D accelerometer
        MAV_SYS_STATUS_SENSOR_3D_MAG = 4,//0x04 3D magnetometer
        MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 8,//0x08 absolute pressure
        MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE = 16,//0x10 differential pressure
        MAV_SYS_STATUS_SENSOR_GPS = 32,//0x20 GPS
        MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW = 64,//0x40 optical flow
        MAV_SYS_STATUS_SENSOR_VISION_POSITION = 128,//0x80 computer vision position
        MAV_SYS_STATUS_SENSOR_LASER_POSITION = 256,//0x100 laser based position
        MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH = 512,//0x200 external ground truth (Vicon or Leica)
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL = 1024,//0x400 3D angular rate control
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048,//0x800 attitude stabilization
        MAV_SYS_STATUS_SENSOR_YAW_POSITION = 4096,//0x1000 yaw position
        MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL = 8192,//0x2000 z/altitude control
        MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL = 16384,//0x4000 x/y position control
        MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS = 32768,//0x8000 motor outputs / control
        MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 65536,//0x10000 rc receiver
        MAV_SYS_STATUS_SENSOR_3D_GYRO2 = 131072,//0x20000 2nd 3D gyro
        MAV_SYS_STATUS_SENSOR_3D_ACCEL2 = 262144,//0x40000 2nd 3D accelerometer
        MAV_SYS_STATUS_SENSOR_3D_MAG2 = 524288,//0x80000 2nd 3D magnetometer
        MAV_SYS_STATUS_GEOFENCE = 1048576,//0x100000 geofence
        MAV_SYS_STATUS_AHRS = 2097152,//0x200000 AHRS subsystem health
        MAV_SYS_STATUS_TERRAIN = 4194304,//0x400000 Terrain subsystem health
        MAV_SYS_STATUS_REVERSE_MOTOR = 8388608,//0x800000 Motors are reversed
        MAV_SYS_STATUS_LOGGING = 16777216,//0x1000000 Logging
        MAV_SYS_STATUS_SENSOR_BATTERY = 33554432;//0x2000000 Battery
    }

    public @interface MAV_FRAME
    {

        int
        /**
        *Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude,
        *				 third value / z: positive altitude over mean sea level (MSL*/
        MAV_FRAME_GLOBAL = 0,
        MAV_FRAME_LOCAL_NED = 1,//Local coordinate frame, Z-up (x: north, y: east, z: down).
        MAV_FRAME_MISSION = 2,//NOT a coordinate frame, indicates a mission command.
        /**
        ** Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home
        *		 * position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude
        *		 * with 0 being at the altitude of the home location*/
        MAV_FRAME_GLOBAL_RELATIVE_ALT = 3,
        MAV_FRAME_LOCAL_ENU = 4,//Local coordinate frame, Z-down (x: east, y: north, z: up)
        /**
        ** Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second
        *		 * value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL*/
        MAV_FRAME_GLOBAL_INT = 5,
        /**
        ** Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home
        *		 * position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third
        *		 * value / z: positive altitude with 0 being at the altitude of the home location*/
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
        /**
        ** Offset to the current local frame. Anything expressed in this frame should be added to the current local
        *		 * frame position*/
        MAV_FRAME_LOCAL_OFFSET_NED = 7,
        /**
        ** Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to
        *		 * command 2 m/s^2 acceleration to the right*/
        MAV_FRAME_BODY_NED = 8,
        /**
        ** Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an
        *		 * obstacle - e.g. useful to command 2 m/s^2 acceleration to the east*/
        MAV_FRAME_BODY_OFFSET_NED = 9,
        /**
        ** Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude
        *		 * over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value
        *		 * / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level
        *		 * in terrain model*/
        MAV_FRAME_GLOBAL_TERRAIN_ALT = 10,
        /**
        ** Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude
        *		 * over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second
        *		 * value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground
        *		 * level in terrain model*/
        MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11;
    }

    /**
    *These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
    *	 simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.*/
    public @interface MAV_MODE
    {

        int
        MAV_MODE_PREFLIGHT = 0,//System is not ready to fly, booting, calibrating, etc. No flag is set.
        MAV_MODE_MANUAL_DISARMED = 64,//System is allowed to be active, under manual (RC) control, no stabilization
        MAV_MODE_TEST_DISARMED = 66,//UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
        MAV_MODE_STABILIZE_DISARMED = 80,//System is allowed to be active, under assisted RC control.
        MAV_MODE_GUIDED_DISARMED = 88,//System is allowed to be active, under autonomous control, manual setpoint
        /**
        ** System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard
        *		 * and not pre-programmed by waypoints*/
        MAV_MODE_AUTO_DISARMED = 92,
        MAV_MODE_MANUAL_ARMED = 192,//System is allowed to be active, under manual (RC) control, no stabilization
        MAV_MODE_TEST_ARMED = 194,//UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
        MAV_MODE_STABILIZE_ARMED = 208,//System is allowed to be active, under assisted RC control.
        MAV_MODE_GUIDED_ARMED = 216,//System is allowed to be active, under autonomous control, manual setpoint
        /**
        ** System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard
        *		 * and not pre-programmed by waypoints*/
        MAV_MODE_AUTO_ARMED = 220;
    }

    protected static int en__P(int id)
    {
        switch(id)
        {
            case 0:
                return MAV_MODE.MAV_MODE_PREFLIGHT;
            case 1:
                return MAV_MODE.MAV_MODE_MANUAL_DISARMED;
            case 2:
                return MAV_MODE.MAV_MODE_TEST_DISARMED;
            case 3:
                return MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            case 4:
                return MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            case 5:
                return MAV_MODE.MAV_MODE_AUTO_DISARMED;
            case 6:
                return MAV_MODE.MAV_MODE_MANUAL_ARMED;
            case 7:
                return MAV_MODE.MAV_MODE_TEST_ARMED;
            case 8:
                return MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            case 9:
                return MAV_MODE.MAV_MODE_GUIDED_ARMED;
            case 10:
                return MAV_MODE.MAV_MODE_AUTO_ARMED;
        }
        assert(false);//("Unknown enum ID " + id);
        return  Integer.MIN_VALUE;
    }
    /**
    *Specifies the datatype of a MAVLink parameter.*/
    public @interface MAV_PARAM_TYPE
    {

        int
        MAV_PARAM_TYPE_UINT8 = 1,//8-bit unsigned integer
        MAV_PARAM_TYPE_INT8 = 2,//8-bit signed integer
        MAV_PARAM_TYPE_UINT16 = 3,//16-bit unsigned integer
        MAV_PARAM_TYPE_INT16 = 4,//16-bit signed integer
        MAV_PARAM_TYPE_UINT32 = 5,//32-bit unsigned integer
        MAV_PARAM_TYPE_INT32 = 6,//32-bit signed integer
        MAV_PARAM_TYPE_UINT64 = 7,//64-bit unsigned integer
        MAV_PARAM_TYPE_INT64 = 8,//64-bit signed integer
        MAV_PARAM_TYPE_REAL32 = 9,//32-bit floating-point
        MAV_PARAM_TYPE_REAL64 = 10;//64-bit floating-point
    }

    /**
    *Type of GPS fix*/
    public @interface GPS_FIX_TYPE
    {

        int
        GPS_FIX_TYPE_NO_GPS = 0,//No GPS connected
        GPS_FIX_TYPE_NO_FIX = 1,//No position information, GPS is connected
        GPS_FIX_TYPE_2D_FIX = 2,//2D position
        GPS_FIX_TYPE_3D_FIX = 3,//3D position
        GPS_FIX_TYPE_DGPS = 4,//DGPS/SBAS aided 3D position
        GPS_FIX_TYPE_RTK_FLOAT = 5,//RTK float, 3D position
        GPS_FIX_TYPE_RTK_FIXED = 6,//RTK Fixed, 3D position
        GPS_FIX_TYPE_STATIC = 7,//Static fixed, typically used for base stations
        GPS_FIX_TYPE_PPP = 8;//PPP, 3D position.
    }

    /**
    *Type of mission items being requested/sent in mission protocol.*/
    public @interface MAV_MISSION_TYPE
    {

        int
        MAV_MISSION_TYPE_MISSION = 0,//Items are mission commands for main mission.
        MAV_MISSION_TYPE_FENCE = 1,//Specifies GeoFence area(s). Items are MAV_CMD_FENCE_ GeoFence items.
        /**
        ** Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_RALLY_POINT
        *		 * rally point items*/
        MAV_MISSION_TYPE_RALLY = 2,
        MAV_MISSION_TYPE_ALL = 255;//Only used in MISSION_CLEAR_ALL to clear all mission types.
    }

    protected static int en__W(int id)
    {
        switch(id)
        {
            case 0:
                return MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            case 1:
                return MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            case 2:
                return MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            case 3:
                return MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
        }
        assert(false);//("Unknown enum ID " + id);
        return  Integer.MIN_VALUE;
    }
    /**
    *Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script.
    *	 If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows:
    *	 Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what
    *	 ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data*/
    public @interface MAV_CMD
    {

        int
        /**
        *Navigate to waypoint.
        *				 1	Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)
        *				 2	Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
        *				 3	0 to pass through the WP, if 	>	0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
        *				 4	Desired yaw angle at waypoint (rotary wing). NaN for unchanged.
        *				 5	Latitude
        *				 6	Longitude
        *				 7	Altitude*/
        MAV_CMD_NAV_WAYPOINT = 16,
        /**
        ** Loiter around this waypoint an unlimited amount of time
        *		 * 1	Empty
        *		 * 2	Empty
        *		 * 3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
        *		 * 4	Desired yaw angle.
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Altitude*/
        MAV_CMD_NAV_LOITER_UNLIM = 17,
        /**
        ** Loiter around this waypoint for X turns
        *		 * 1	Turns
        *		 * 2	Empty
        *		 * 3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
        *		 * 4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Altitude*/
        MAV_CMD_NAV_LOITER_TURNS = 18,
        /**
        ** Loiter around this waypoint for X seconds
        *		 * 1	Seconds (decimal)
        *		 * 2	Empty
        *		 * 3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
        *		 * 4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Altitude*/
        MAV_CMD_NAV_LOITER_TIME = 19,
        /**
        ** Return to launch location
        *		 * 1	Empty
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_NAV_RETURN_TO_LAUNCH = 20,
        /**
        ** Land at location
        *		 * 1	Abort Alt
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Desired yaw angle. NaN for unchanged.
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Altitude (ground level)*/
        MAV_CMD_NAV_LAND = 21,
        /**
        ** Takeoff from ground / hand
        *		 * 1	Minimum pitch (if airspeed sensor present), desired pitch without sensor
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Altitude*/
        MAV_CMD_NAV_TAKEOFF = 22,
        /**
        ** Land at local position (local frame only)
        *		 * 1	Landing target number (if available)
        *		 * 2	Maximum accepted offset from desired landing position [m] - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land
        *		 * 3	Landing descend rate [ms^-1]
        *		 * 4	Desired yaw angle [rad]
        *		 * 5	Y-axis position [m]
        *		 * 6	X-axis position [m]
        *		 * 7	Z-axis / ground level position [m]*/
        MAV_CMD_NAV_LAND_LOCAL = 23,
        /**
        ** Takeoff from local position (local frame only)
        *		 * 1	Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]
        *		 * 2	Empty
        *		 * 3	Takeoff ascend rate [ms^-1]
        *		 * 4	Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these
        *		 * 5	Y-axis position [m]
        *		 * 6	X-axis position [m]
        *		 * 7	Z-axis position [m]*/
        MAV_CMD_NAV_TAKEOFF_LOCAL = 24,
        /**
        ** Vehicle following, i.e. this waypoint represents the position of a moving vehicle
        *		 * 1	Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation
        *		 * 2	Ground speed of vehicle to be followed
        *		 * 3	Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise
        *		 * 4	Desired yaw angle.
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Altitude*/
        MAV_CMD_NAV_FOLLOW = 25,
        /**
        ** Continue on the current course and climb/descend to specified altitude.  When the altitude is reached
        *		 * continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached
        *		 * 1	Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Desired altitude in meters*/
        MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30,
        /**
        ** Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.
        *		 * Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.
        *		 * Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter
        *		 * until heading toward the next waypoint.
        *		 * 1	Heading Required (0 = False)
        *		 * 2	Radius in meters. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.
        *		 * 3	Empty
        *		 * 4	Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Altitude*/
        MAV_CMD_NAV_LOITER_TO_ALT = 31,
        /**
        ** Being following a target
        *		 * 1	System ID (the system ID of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode
        *		 * 2	RESERVED
        *		 * 3	RESERVED
        *		 * 4	altitude flag: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home
        *		 * 5	altitude
        *		 * 6	RESERVED
        *		 * 7	TTL in seconds in which the MAV should go to the default position hold mode after a message rx timeout*/
        MAV_CMD_DO_FOLLOW = 32,
        /**
        ** Reposition the MAV after a follow target command has been sent
        *		 * 1	Camera q1 (where 0 is on the ray from the camera to the tracking device)
        *		 * 2	Camera q2
        *		 * 3	Camera q3
        *		 * 4	Camera q4
        *		 * 5	altitude offset from target (m)
        *		 * 6	X offset from target (m)
        *		 * 7	Y offset from target (m)*/
        MAV_CMD_DO_FOLLOW_REPOSITION = 33,
        /**
        ** Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the
        *		 * vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras
        *		 * 1	Region of intereset mode. (see MAV_ROI enum)
        *		 * 2	Waypoint index/ target ID. (see MAV_ROI enum)
        *		 * 3	ROI index (allows a vehicle to manage multiple ROI's)
        *		 * 4	Empty
        *		 * 5	x the location of the fixed ROI (see MAV_FRAME)
        *		 * 6	y
        *		 * 7	z*/
        MAV_CMD_NAV_ROI = 80,
        /**
        ** Control autonomous path planning on the MAV.
        *		 * 1	0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning
        *		 * 2	0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid
        *		 * 3	Empty
        *		 * 4	Yaw angle at goal, in compass degrees, [0..360]
        *		 * 5	Latitude/X of goal
        *		 * 6	Longitude/Y of goal
        *		 * 7	Altitude/Z of goal*/
        MAV_CMD_NAV_PATHPLANNING = 81,
        /**
        ** Navigate to waypoint using a spline path.
        *		 * 1	Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Latitude/X of goal
        *		 * 6	Longitude/Y of goal
        *		 * 7	Altitude/Z of goal*/
        MAV_CMD_NAV_SPLINE_WAYPOINT = 82,
        /**
        ** Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon
        *		 * launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical
        *		 * speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control
        *		 * surfaces to prevent them seizing up
        *		 * 1	altitude (m)
        *		 * 2	descent speed (m/s)
        *		 * 3	Wiggle Time (s)
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_NAV_ALTITUDE_WAIT = 83,
        /**
        ** Takeoff from ground using VTOL mode
        *		 * 1	Empty
        *		 * 2	Front transition heading, see VTOL_TRANSITION_HEADING enum.
        *		 * 3	Empty
        *		 * 4	Yaw angle in degrees. NaN for unchanged.
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Altitude*/
        MAV_CMD_NAV_VTOL_TAKEOFF = 84,
        /**
        ** Land using VTOL mode
        *		 * 1	Empty
        *		 * 2	Empty
        *		 * 3	Approach altitude (with the same reference as the Altitude field). NaN if unspecified.
        *		 * 4	Yaw angle in degrees. NaN for unchanged.
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Altitude (ground level)*/
        MAV_CMD_NAV_VTOL_LAND = 85,
        /**
        ** hand control over to an external controller
        *		 * 1	On / Off (	>	0.5f on)
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_NAV_GUIDED_ENABLE = 92,
        /**
        ** Delay the next navigation command a number of seconds or until a specified time
        *		 * 1	Delay in seconds (decimal, -1 to enable time-of-day fields)
        *		 * 2	hour (24h format, UTC, -1 to ignore)
        *		 * 3	minute (24h format, UTC, -1 to ignore)
        *		 * 4	second (24h format, UTC)
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_NAV_DELAY = 93,
        /**
        ** Descend and place payload.  Vehicle descends until it detects a hanging payload has reached the ground,
        *		 * the gripper is opened to release the payloa
        *		 * 1	Maximum distance to descend (meters)
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Latitude (deg * 1E7)
        *		 * 6	Longitude (deg * 1E7)
        *		 * 7	Altitude (meters)*/
        MAV_CMD_NAV_PAYLOAD_PLACE = 94,
        /**
        ** NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeratio
        *		 * 1	Empty
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_NAV_LAST = 95,
        /**
        ** Delay mission state machine.
        *		 * 1	Delay in seconds (decimal)
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_CONDITION_DELAY = 112,
        /**
        ** Ascend/descend at rate.  Delay mission state machine until desired altitude reached.
        *		 * 1	Descent / Ascend rate (m/s)
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Finish Altitude*/
        MAV_CMD_CONDITION_CHANGE_ALT = 113,
        /**
        ** Delay mission state machine until within desired distance of next NAV point.
        *		 * 1	Distance (meters)
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_CONDITION_DISTANCE = 114,
        /**
        ** Reach a certain target angle.
        *		 * 1	target angle: [0-360], 0 is north
        *		 * 2	speed during yaw change:[deg per second]
        *		 * 3	direction: negative: counter clockwise, positive: clockwise [-1,1]
        *		 * 4	relative offset or absolute angle: [ 1,0]
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_CONDITION_YAW = 115,
        /**
        ** NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeratio
        *		 * 1	Empty
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_CONDITION_LAST = 159,
        /**
        ** Set system mode.
        *		 * 1	Mode, as defined by ENUM MAV_MODE
        *		 * 2	Custom mode - this is system specific, please refer to the individual autopilot specifications for details.
        *		 * 3	Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_SET_MODE = 176,
        /**
        ** Jump to the desired command in the mission list.  Repeat this action only the specified number of time
        *		 * 1	Sequence number
        *		 * 2	Repeat count
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_JUMP = 177,
        /**
        ** Change speed and/or throttle set points.
        *		 * 1	Speed type (0=Airspeed, 1=Ground Speed)
        *		 * 2	Speed  (m/s, -1 indicates no change)
        *		 * 3	Throttle  ( Percent, -1 indicates no change)
        *		 * 4	absolute or relative [0,1]
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_CHANGE_SPEED = 178,
        /**
        ** Changes the home location either to the current location or a specified location.
        *		 * 1	Use current (1=use current location, 0=use specified location)
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Altitude*/
        MAV_CMD_DO_SET_HOME = 179,
        /**
        ** Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value
        *		 * of the parameter
        *		 * 1	Parameter number
        *		 * 2	Parameter value
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_SET_PARAMETER = 180,
        /**
        ** Set a relay to a condition.
        *		 * 1	Relay number
        *		 * 2	Setting (1=on, 0=off, others possible depending on system hardware)
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_SET_RELAY = 181,
        /**
        ** Cycle a relay on and off for a desired number of cyles with a desired period.
        *		 * 1	Relay number
        *		 * 2	Cycle count
        *		 * 3	Cycle time (seconds, decimal)
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_REPEAT_RELAY = 182,
        /**
        ** Set a servo to a desired PWM value.
        *		 * 1	Servo number
        *		 * 2	PWM (microseconds, 1000 to 2000 typical)
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_SET_SERVO = 183,
        /**
        ** Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period
        *		 * 1	Servo number
        *		 * 2	PWM (microseconds, 1000 to 2000 typical)
        *		 * 3	Cycle count
        *		 * 4	Cycle time (seconds)
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_REPEAT_SERVO = 184,
        /**
        ** Terminate flight immediately
        *		 * 1	Flight termination activated if 	>	0.5
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_FLIGHTTERMINATION = 185,
        /**
        ** Change altitude set point.
        *		 * 1	Altitude in meters
        *		 * 2	Mav frame of new altitude (see MAV_FRAME)
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_CHANGE_ALTITUDE = 186,
        /**
        ** Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where
        *		 * a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG
        *		 * to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will
        *		 * be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it
        *		 * will be used to help find the closest landing sequence
        *		 * 1	Empty
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Empty*/
        MAV_CMD_DO_LAND_START = 189,
        /**
        ** Mission command to perform a landing from a rally point.
        *		 * 1	Break altitude (meters)
        *		 * 2	Landing speed (m/s)
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_RALLY_LAND = 190,
        /**
        ** Mission command to safely abort an autonmous landing.
        *		 * 1	Altitude (meters)
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_GO_AROUND = 191,
        /**
        ** Reposition the vehicle to a specific WGS84 global position.
        *		 * 1	Ground speed, less than 0 (-1) for default
        *		 * 2	Bitmask of option flags, see the MAV_DO_REPOSITION_FLAGS enum.
        *		 * 3	Reserved
        *		 * 4	Yaw heading, NaN for unchanged. For planes indicates loiter direction (0: clockwise, 1: counter clockwise)
        *		 * 5	Latitude (deg * 1E7)
        *		 * 6	Longitude (deg * 1E7)
        *		 * 7	Altitude (meters)*/
        MAV_CMD_DO_REPOSITION = 192,
        /**
        ** If in a GPS controlled position mode, hold the current position or continue.
        *		 * 1	0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.
        *		 * 2	Reserved
        *		 * 3	Reserved
        *		 * 4	Reserved
        *		 * 5	Reserved
        *		 * 6	Reserved
        *		 * 7	Reserved*/
        MAV_CMD_DO_PAUSE_CONTINUE = 193,
        /**
        ** Set moving direction to forward or reverse.
        *		 * 1	Direction (0=Forward, 1=Reverse)
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_SET_REVERSE = 194,
        /**
        ** Control onboard camera system.
        *		 * 1	Camera ID (-1 for all)
        *		 * 2	Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw
        *		 * 3	Transmission mode: 0: video stream, 	>	0: single images every n seconds (decimal)
        *		 * 4	Recording: 0: disabled, 1: enabled compressed, 2: enabled raw
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_CONTROL_VIDEO = 200,
        /**
        ** Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the
        *		 * vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras
        *		 * 1	Region of intereset mode. (see MAV_ROI enum)
        *		 * 2	Waypoint index/ target ID. (see MAV_ROI enum)
        *		 * 3	ROI index (allows a vehicle to manage multiple ROI's)
        *		 * 4	Empty
        *		 * 5	MAV_ROI_WPNEXT: pitch offset from next waypoint, MAV_ROI_LOCATION: latitude
        *		 * 6	MAV_ROI_WPNEXT: roll offset from next waypoint, MAV_ROI_LOCATION: longitude
        *		 * 7	MAV_ROI_WPNEXT: yaw offset from next waypoint, MAV_ROI_LOCATION: altitude*/
        MAV_CMD_DO_SET_ROI = 201,
        /**
        ** Mission command to configure an on-board camera controller system.
        *		 * 1	Modes: P, TV, AV, M, Etc
        *		 * 2	Shutter speed: Divisor number for one second
        *		 * 3	Aperture: F stop number
        *		 * 4	ISO number e.g. 80, 100, 200, Etc
        *		 * 5	Exposure type enumerator
        *		 * 6	Command Identity
        *		 * 7	Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)*/
        MAV_CMD_DO_DIGICAM_CONFIGURE = 202,
        /**
        ** Mission command to control an on-board camera controller system.
        *		 * 1	Session control e.g. show/hide lens
        *		 * 2	Zoom's absolute position
        *		 * 3	Zooming step value to offset zoom from the current position
        *		 * 4	Focus Locking, Unlocking or Re-locking
        *		 * 5	Shooting Command
        *		 * 6	Command Identity
        *		 * 7	Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.*/
        MAV_CMD_DO_DIGICAM_CONTROL = 203,
        /**
        ** Mission command to configure a camera or antenna mount
        *		 * 1	Mount operation mode (see MAV_MOUNT_MODE enum)
        *		 * 2	stabilize roll? (1 = yes, 0 = no)
        *		 * 3	stabilize pitch? (1 = yes, 0 = no)
        *		 * 4	stabilize yaw? (1 = yes, 0 = no)
        *		 * 5	roll input (0 = angle, 1 = angular rate)
        *		 * 6	pitch input (0 = angle, 1 = angular rate)
        *		 * 7	yaw input (0 = angle, 1 = angular rate)*/
        MAV_CMD_DO_MOUNT_CONFIGURE = 204,
        /**
        ** Mission command to control a camera or antenna mount
        *		 * 1	pitch depending on mount mode (degrees or degrees/second depending on pitch input).
        *		 * 2	roll depending on mount mode (degrees or degrees/second depending on roll input).
        *		 * 3	yaw depending on mount mode (degrees or degrees/second depending on yaw input).
        *		 * 4	alt in meters depending on mount mode.
        *		 * 5	latitude in degrees * 1E7, set if appropriate mount mode.
        *		 * 6	longitude in degrees * 1E7, set if appropriate mount mode.
        *		 * 7	MAV_MOUNT_MODE enum value*/
        MAV_CMD_DO_MOUNT_CONTROL = 205,
        /**
        ** Mission command to set camera trigger distance for this flight. The camera is trigerred each time this
        *		 * distance is exceeded. This command can also be used to set the shutter integration time for the camera
        *		 * 1	Camera trigger distance (meters). 0 to stop triggering.
        *		 * 2	Camera shutter integration time (milliseconds). -1 or 0 to ignore
        *		 * 3	Trigger camera once immediately. (0 = no trigger, 1 = trigger)
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206,
        /**
        ** Mission command to enable the geofence
        *		 * 1	enable? (0=disable, 1=enable, 2=disable_floor_only)
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_FENCE_ENABLE = 207,
        /**
        ** Mission command to trigger a parachute
        *		 * 1	action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_PARACHUTE = 208,
        /**
        ** Mission command to perform motor test
        *		 * 1	motor sequence number (a number from 1 to max number of motors on the vehicle)
        *		 * 2	throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
        *		 * 3	throttle
        *		 * 4	timeout (in seconds)
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_MOTOR_TEST = 209,
        /**
        ** Change to/from inverted flight
        *		 * 1	inverted (0=normal, 1=inverted)
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_INVERTED_FLIGHT = 210,
        /**
        ** Mission command to operate EPM gripper
        *		 * 1	gripper number (a number from 1 to max number of grippers on the vehicle)
        *		 * 2	gripper action (0=release, 1=grab. See GRIPPER_ACTIONS enum)
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_GRIPPER = 211,
        /**
        ** Enable/disable autotune
        *		 * 1	enable (1: enable, 0:disable)
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_AUTOTUNE_ENABLE = 212,
        /**
        ** Sets a desired vehicle turn angle and speed change
        *		 * 1	yaw angle to adjust steering by in centidegress
        *		 * 2	speed - normalized to 0 .. 1
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_NAV_SET_YAW_SPEED = 213,
        /**
        ** Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is
        *		 * triggered each time this interval expires. This command can also be used to set the shutter integration
        *		 * time for the camera
        *		 * 1	Camera trigger cycle time (milliseconds). -1 or 0 to ignore.
        *		 * 2	Camera shutter integration time (milliseconds). Should be less than trigger cycle time. -1 or 0 to ignore.
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214,
        /**
        ** Mission command to control a camera or antenna mount, using a quaternion as reference.
        *		 * 1	q1 - quaternion param #1, w (1 in null-rotation)
        *		 * 2	q2 - quaternion param #2, x (0 in null-rotation)
        *		 * 3	q3 - quaternion param #3, y (0 in null-rotation)
        *		 * 4	q4 - quaternion param #4, z (0 in null-rotation)
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_MOUNT_CONTROL_QUAT = 220,
        /**
        ** set id of master controller
        *		 * 1	System ID
        *		 * 2	Component ID
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_GUIDED_MASTER = 221,
        /**
        ** set limits for external control
        *		 * 1	timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout
        *		 * 2	absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit
        *		 * 3	absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit
        *		 * 4	horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_GUIDED_LIMITS = 222,
        /**
        ** Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine
        *		 * state. It is intended for vehicles with internal combustion engine
        *		 * 1	0: Stop engine, 1:Start Engine
        *		 * 2	0: Warm start, 1:Cold start. Controls use of choke where applicable
        *		 * 3	Height delay (meters). This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_ENGINE_CONTROL = 223,
        /**
        ** NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
        *		 * 1	Empty
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_LAST = 240,
        /**
        ** Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature
        *		 * Calibration, only one sensor should be set in a single message and all others should be zero
        *		 * 1	1: gyro calibration, 3: gyro temperature calibration
        *		 * 2	1: magnetometer calibration
        *		 * 3	1: ground pressure calibration
        *		 * 4	1: radio RC calibration, 2: RC trim calibration
        *		 * 5	1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration
        *		 * 6	1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration
        *		 * 7	1: ESC calibration, 3: barometer temperature calibration*/
        MAV_CMD_PREFLIGHT_CALIBRATION = 241,
        /**
        ** Set sensor offsets. This command will be only accepted if in pre-flight mode.
        *		 * 1	Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer
        *		 * 2	X axis offset (or generic dimension 1), in the sensor's raw units
        *		 * 3	Y axis offset (or generic dimension 2), in the sensor's raw units
        *		 * 4	Z axis offset (or generic dimension 3), in the sensor's raw units
        *		 * 5	Generic dimension 4, in the sensor's raw units
        *		 * 6	Generic dimension 5, in the sensor's raw units
        *		 * 7	Generic dimension 6, in the sensor's raw units*/
        MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242,
        /**
        ** Trigger UAVCAN config. This command will be only accepted if in pre-flight mode.
        *		 * 1	1: Trigger actuator ID assignment and direction mapping.
        *		 * 2	Reserved
        *		 * 3	Reserved
        *		 * 4	Reserved
        *		 * 5	Reserved
        *		 * 6	Reserved
        *		 * 7	Reserved*/
        MAV_CMD_PREFLIGHT_UAVCAN = 243,
        /**
        ** Request storage of different parameter values and logs. This command will be only accepted if in pre-flight
        *		 * mode
        *		 * 1	Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults
        *		 * 2	Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults
        *		 * 3	Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, 	>	1: start logging with rate of param 3 in Hz (e.g. set to 1000 for 1000 Hz logging)
        *		 * 4	Reserved
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_PREFLIGHT_STORAGE = 245,
        /**
        ** Request the reboot or shutdown of system components.
        *		 * 1	0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.
        *		 * 2	0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.
        *		 * 3	WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded
        *		 * 4	WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded
        *		 * 5	Reserved, send 0
        *		 * 6	Reserved, send 0
        *		 * 7	WIP: ID (e.g. camera ID -1 for all IDs)*/
        MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246,
        /**
        ** Hold / continue the current action
        *		 * 1	MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan
        *		 * 2	MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position
        *		 * 3	MAV_FRAME coordinate frame of hold point
        *		 * 4	Desired yaw angle in degrees
        *		 * 5	Latitude / X position
        *		 * 6	Longitude / Y position
        *		 * 7	Altitude / Z position*/
        MAV_CMD_OVERRIDE_GOTO = 252,
        /**
        ** start running a mission
        *		 * 1	first_item: the first mission item to run
        *		 * 2	last_item:  the last mission item to run (after this item is run, the mission ends)*/
        MAV_CMD_MISSION_START = 300,
        /**
        ** Arms / Disarms a component
        *		 * 1	1 to arm, 0 to disarm*/
        MAV_CMD_COMPONENT_ARM_DISARM = 400,
        /**
        ** Request the home position from the vehicle.
        *		 * 1	Reserved
        *		 * 2	Reserved
        *		 * 3	Reserved
        *		 * 4	Reserved
        *		 * 5	Reserved
        *		 * 6	Reserved
        *		 * 7	Reserved*/
        MAV_CMD_GET_HOME_POSITION = 410,
        /**
        ** Starts receiver pairing
        *		 * 1	0:Spektrum
        *		 * 2	0:Spektrum DSM2, 1:Spektrum DSMX*/
        MAV_CMD_START_RX_PAIR = 500,
        /**
        ** Request the interval between messages for a particular MAVLink message ID
        *		 * 1	The MAVLink message ID*/
        MAV_CMD_GET_MESSAGE_INTERVAL = 510,
        /**
        ** Request the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREA
        *		 * 1	The MAVLink message ID
        *		 * 2	The interval between two messages, in microseconds. Set to -1 to disable and 0 to request default rate.*/
        MAV_CMD_SET_MESSAGE_INTERVAL = 511,
        /**
        ** Request MAVLink protocol version compatibility
        *		 * 1	1: Request supported protocol versions by all nodes on the network
        *		 * 2	Reserved (all remaining params)*/
        MAV_CMD_REQUEST_PROTOCOL_VERSION = 519,
        /**
        ** Request autopilot capabilities
        *		 * 1	1: Request autopilot version
        *		 * 2	Reserved (all remaining params)*/
        MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520,
        /**
        ** WIP: Request camera information (CAMERA_INFORMATION).
        *		 * 1	0: No action 1: Request camera capabilities
        *		 * 2	Reserved (all remaining params)*/
        MAV_CMD_REQUEST_CAMERA_INFORMATION = 521,
        /**
        ** WIP: Request camera settings (CAMERA_SETTINGS).
        *		 * 1	0: No Action 1: Request camera settings
        *		 * 2	Reserved (all remaining params)*/
        MAV_CMD_REQUEST_CAMERA_SETTINGS = 522,
        /**
        ** WIP: Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a
        *		 * specific component's storage
        *		 * 1	Storage ID (0 for all, 1 for first, 2 for second, etc.)
        *		 * 2	0: No Action 1: Request storage information
        *		 * 3	Reserved (all remaining params)*/
        MAV_CMD_REQUEST_STORAGE_INFORMATION = 525,
        /**
        ** WIP: Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the
        *		 * command's target_component to target a specific component's storage
        *		 * 1	Storage ID (1 for first, 2 for second, etc.)
        *		 * 2	0: No action 1: Format storage
        *		 * 3	Reserved (all remaining params)*/
        MAV_CMD_STORAGE_FORMAT = 526,
        /**
        ** WIP: Request camera capture status (CAMERA_CAPTURE_STATUS)
        *		 * 1	0: No Action 1: Request camera capture status
        *		 * 2	Reserved (all remaining params)*/
        MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS = 527,
        /**
        ** WIP: Request flight information (FLIGHT_INFORMATION)
        *		 * 1	1: Request flight information
        *		 * 2	Reserved (all remaining params)*/
        MAV_CMD_REQUEST_FLIGHT_INFORMATION = 528,
        /**
        ** WIP: Reset all camera settings to Factory Default
        *		 * 1	0: No Action 1: Reset all settings
        *		 * 2	Reserved (all remaining params)*/
        MAV_CMD_RESET_CAMERA_SETTINGS = 529,
        /**
        ** Set camera running mode. Use NAN for reserved values.
        *		 * 1	Reserved (Set to 0)
        *		 * 2	Camera mode (see CAMERA_MODE enum)
        *		 * 3	Reserved (all remaining params)*/
        MAV_CMD_SET_CAMERA_MODE = 530,
        /**
        ** Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NAN for reserved values
        *		 * 1	Reserved (Set to 0)
        *		 * 2	Duration between two consecutive pictures (in seconds)
        *		 * 3	Number of images to capture total - 0 for unlimited capture
        *		 * 4	Reserved (all remaining params)*/
        MAV_CMD_IMAGE_START_CAPTURE = 2000,
        /**
        ** Stop image capture sequence Use NAN for reserved values.
        *		 * 1	Reserved (Set to 0)
        *		 * 2	Reserved (all remaining params)*/
        MAV_CMD_IMAGE_STOP_CAPTURE = 2001,
        /**
        ** WIP: Re-request a CAMERA_IMAGE_CAPTURE packet. Use NAN for reserved values.
        *		 * 1	Sequence number for missing CAMERA_IMAGE_CAPTURE packet
        *		 * 2	Reserved (all remaining params)*/
        MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE = 2002,
        /**
        ** Enable or disable on-board camera triggering system.
        *		 * 1	Trigger enable/disable (0 for disable, 1 for start), -1 to ignore
        *		 * 2	1 to reset the trigger sequence, -1 or 0 to ignore
        *		 * 3	1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore*/
        MAV_CMD_DO_TRIGGER_CONTROL = 2003,
        /**
        ** Starts video capture (recording). Use NAN for reserved values.
        *		 * 1	Reserved (Set to 0)
        *		 * 2	Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency in Hz)
        *		 * 3	Reserved (all remaining params)*/
        MAV_CMD_VIDEO_START_CAPTURE = 2500,
        /**
        ** Stop the current video capture (recording). Use NAN for reserved values.
        *		 * 1	Reserved (Set to 0)
        *		 * 2	Reserved (all remaining params)*/
        MAV_CMD_VIDEO_STOP_CAPTURE = 2501,
        /**
        ** WIP: Start video streaming
        *		 * 1	Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
        *		 * 2	Reserved*/
        MAV_CMD_VIDEO_START_STREAMING = 2502,
        /**
        ** WIP: Stop the current video streaming
        *		 * 1	Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
        *		 * 2	Reserved*/
        MAV_CMD_VIDEO_STOP_STREAMING = 2503,
        /**
        ** WIP: Request video stream information (VIDEO_STREAM_INFORMATION)
        *		 * 1	Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)
        *		 * 2	0: No Action 1: Request video stream information
        *		 * 3	Reserved (all remaining params)*/
        MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION = 2504,
        /**
        ** Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
        *		 * 1	Format: 0: ULog
        *		 * 2	Reserved (set to 0)
        *		 * 3	Reserved (set to 0)
        *		 * 4	Reserved (set to 0)
        *		 * 5	Reserved (set to 0)
        *		 * 6	Reserved (set to 0)
        *		 * 7	Reserved (set to 0)*/
        MAV_CMD_LOGGING_START = 2510,
        /**
        ** Request to stop streaming log data over MAVLink
        *		 * 1	Reserved (set to 0)
        *		 * 2	Reserved (set to 0)
        *		 * 3	Reserved (set to 0)
        *		 * 4	Reserved (set to 0)
        *		 * 5	Reserved (set to 0)
        *		 * 6	Reserved (set to 0)
        *		 * 7	Reserved (set to 0)*/
        MAV_CMD_LOGGING_STOP = 2511,
        /**
        ** 1	Landing gear ID (default: 0, -1 for all)
        *		 * 2	Landing gear position (Down: 0, Up: 1, NAN for no change)
        *		 * 3	Reserved, set to NAN
        *		 * 4	Reserved, set to NAN
        *		 * 5	Reserved, set to NAN
        *		 * 6	Reserved, set to NAN
        *		 * 7	Reserved, set to NAN*/
        MAV_CMD_AIRFRAME_CONFIGURATION = 2520,
        /**
        ** Create a panorama at the current position
        *		 * 1	Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)
        *		 * 2	Viewing angle vertical of panorama (in degrees)
        *		 * 3	Speed of the horizontal rotation (in degrees per second)
        *		 * 4	Speed of the vertical rotation (in degrees per second)*/
        MAV_CMD_PANORAMA_CREATE = 2800,
        /**
        ** Request VTOL transition
        *		 * 1	The target VTOL state, as defined by ENUM MAV_VTOL_STATE. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.*/
        MAV_CMD_DO_VTOL_TRANSITION = 3000,
        /**
        ** Request authorization to arm the vehicle to a external entity, the arm authorizer is resposible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
        *		 * <p>
        *		 * 1	Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle*/
        MAV_CMD_ARM_AUTHORIZATION_REQUEST = 3001,
        /**
        ** This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocites along all three axes.*/
        MAV_CMD_SET_GUIDED_SUBMODE_STANDARD = 4000,
        /**
        ** This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
        *		 * <p>
        *		 * 1	Radius of desired circle in CIRCLE_MODE
        *		 * 2	User defined
        *		 * 3	User defined
        *		 * 4	User defined
        *		 * 5	Unscaled target latitude of center of circle in CIRCLE_MODE
        *		 * 6	Unscaled target longitude of center of circle in CIRCLE_MODE*/
        MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE = 4001,
        /**
        ** WIP: Delay mission state machine until gate has been reached.
        *		 * 1	Geometry: 0: orthogonal to path between previous and next waypoint.
        *		 * 2	Altitude: 0: ignore altitude
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Altitude*/
        MAV_CMD_CONDITION_GATE = 4501,
        /**
        ** Fence return point. There can only be one fence return point.
        *		 * <p>
        *		 * 1	Reserved
        *		 * 2	Reserved
        *		 * 3	Reserved
        *		 * 4	Reserved
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Altitude*/
        MAV_CMD_NAV_FENCE_RETURN_POINT = 5000,
        /**
        ** Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
        *		 * <p>
        *		 * 1	Polygon vertex count
        *		 * 2	Reserved
        *		 * 3	Reserved
        *		 * 4	Reserved
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Reserved*/
        MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001,
        /**
        ** Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
        *		 * <p>
        *		 * 1	Polygon vertex count
        *		 * 2	Reserved
        *		 * 3	Reserved
        *		 * 4	Reserved
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Reserved*/
        MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002,
        /**
        ** Circular fence area. The vehicle must stay inside this area.
        *		 * <p>
        *		 * 1	radius in meters
        *		 * 2	Reserved
        *		 * 3	Reserved
        *		 * 4	Reserved
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Reserved*/
        MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION = 5003,
        /**
        ** Circular fence area. The vehicle must stay outside this area.
        *		 * <p>
        *		 * 1	radius in meters
        *		 * 2	Reserved
        *		 * 3	Reserved
        *		 * 4	Reserved
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Reserved*/
        MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004,
        /**
        ** Rally point. You can have multiple rally points defined.
        *		 * <p>
        *		 * 1	Reserved
        *		 * 2	Reserved
        *		 * 3	Reserved
        *		 * 4	Reserved
        *		 * 5	Latitude
        *		 * 6	Longitude
        *		 * 7	Altitude*/
        MAV_CMD_NAV_RALLY_POINT = 5100,
        /**
        ** Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN
        *		 * node that is online. Note that some of the response messages can be lost, which the receiver can detect
        *		 * easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO
        *		 * received earlier; if not, this command should be sent again in order to request re-transmission of the
        *		 * node information messages
        *		 * 1	Reserved (set to 0)
        *		 * 2	Reserved (set to 0)
        *		 * 3	Reserved (set to 0)
        *		 * 4	Reserved (set to 0)
        *		 * 5	Reserved (set to 0)
        *		 * 6	Reserved (set to 0)
        *		 * 7	Reserved (set to 0)*/
        MAV_CMD_UAVCAN_GET_NODE_INFO = 5200,
        /**
        ** Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release
        *		 * position and velocity
        *		 * 1	Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.
        *		 * 2	Desired approach vector in degrees compass heading (0..360). A negative value indicates the system can define the approach vector at will.
        *		 * 3	Desired ground speed at release time. This can be overriden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.
        *		 * 4	Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.
        *		 * 5	Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT
        *		 * 6	Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT
        *		 * 7	Altitude, in meters AMSL*/
        MAV_CMD_PAYLOAD_PREPARE_DEPLOY = 30001,
        /**
        ** Control the payload deployment.
        *		 * 1	Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deploment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.
        *		 * 2	Reserved
        *		 * 3	Reserved
        *		 * 4	Reserved
        *		 * 5	Reserved
        *		 * 6	Reserved
        *		 * 7	Reserved*/
        MAV_CMD_PAYLOAD_CONTROL_DEPLOY = 30002,
        /**
        ** User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
        *		 * 1	User defined
        *		 * 2	User defined
        *		 * 3	User defined
        *		 * 4	User defined
        *		 * 5	Latitude unscaled
        *		 * 6	Longitude unscaled
        *		 * 7	Altitude, in meters AMSL*/
        MAV_CMD_WAYPOINT_USER_1 = 31000,
        /**
        ** User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
        *		 * 1	User defined
        *		 * 2	User defined
        *		 * 3	User defined
        *		 * 4	User defined
        *		 * 5	Latitude unscaled
        *		 * 6	Longitude unscaled
        *		 * 7	Altitude, in meters AMSL*/
        MAV_CMD_WAYPOINT_USER_2 = 31001,
        /**
        ** User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
        *		 * 1	User defined
        *		 * 2	User defined
        *		 * 3	User defined
        *		 * 4	User defined
        *		 * 5	Latitude unscaled
        *		 * 6	Longitude unscaled
        *		 * 7	Altitude, in meters AMSL*/
        MAV_CMD_WAYPOINT_USER_3 = 31002,
        /**
        ** User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
        *		 * 1	User defined
        *		 * 2	User defined
        *		 * 3	User defined
        *		 * 4	User defined
        *		 * 5	Latitude unscaled
        *		 * 6	Longitude unscaled
        *		 * 7	Altitude, in meters AMSL*/
        MAV_CMD_WAYPOINT_USER_4 = 31003,
        /**
        ** User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
        *		 * 1	User defined
        *		 * 2	User defined
        *		 * 3	User defined
        *		 * 4	User defined
        *		 * 5	Latitude unscaled
        *		 * 6	Longitude unscaled
        *		 * 7	Altitude, in meters AMSL*/
        MAV_CMD_WAYPOINT_USER_5 = 31004,
        /**
        ** User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
        *		 * ROI item
        *		 * 1	User defined
        *		 * 2	User defined
        *		 * 3	User defined
        *		 * 4	User defined
        *		 * 5	Latitude unscaled
        *		 * 6	Longitude unscaled
        *		 * 7	Altitude, in meters AMSL*/
        MAV_CMD_SPATIAL_USER_1 = 31005,
        /**
        ** User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
        *		 * ROI item
        *		 * 1	User defined
        *		 * 2	User defined
        *		 * 3	User defined
        *		 * 4	User defined
        *		 * 5	Latitude unscaled
        *		 * 6	Longitude unscaled
        *		 * 7	Altitude, in meters AMSL*/
        MAV_CMD_SPATIAL_USER_2 = 31006,
        /**
        ** User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
        *		 * ROI item
        *		 * 1	User defined
        *		 * 2	User defined
        *		 * 3	User defined
        *		 * 4	User defined
        *		 * 5	Latitude unscaled
        *		 * 6	Longitude unscaled
        *		 * 7	Altitude, in meters AMSL*/
        MAV_CMD_SPATIAL_USER_3 = 31007,
        /**
        ** User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
        *		 * ROI item
        *		 * 1	User defined
        *		 * 2	User defined
        *		 * 3	User defined
        *		 * 4	User defined
        *		 * 5	Latitude unscaled
        *		 * 6	Longitude unscaled
        *		 * 7	Altitude, in meters AMSL*/
        MAV_CMD_SPATIAL_USER_4 = 31008,
        /**
        ** User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example:
        *		 * ROI item
        *		 * 1	User defined
        *		 * 2	User defined
        *		 * 3	User defined
        *		 * 4	User defined
        *		 * 5	Latitude unscaled
        *		 * 6	Longitude unscaled
        *		 * 7	Altitude, in meters AMSL*/
        MAV_CMD_SPATIAL_USER_5 = 31009,
        /**
        ** User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
        *		 * item
        *		 * 1	User defined
        *		 * 2	User defined
        *		 * 3	User defined
        *		 * 4	User defined
        *		 * 5	User defined
        *		 * 6	User defined
        *		 * 7	User defined*/
        MAV_CMD_USER_1 = 31010,
        /**
        ** User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
        *		 * item
        *		 * 1	User defined
        *		 * 2	User defined
        *		 * 3	User defined
        *		 * 4	User defined
        *		 * 5	User defined
        *		 * 6	User defined
        *		 * 7	User defined*/
        MAV_CMD_USER_2 = 31011,
        /**
        ** User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
        *		 * item
        *		 * 1	User defined
        *		 * 2	User defined
        *		 * 3	User defined
        *		 * 4	User defined
        *		 * 5	User defined
        *		 * 6	User defined
        *		 * 7	User defined*/
        MAV_CMD_USER_3 = 31012,
        /**
        ** User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
        *		 * item
        *		 * 1	User defined
        *		 * 2	User defined
        *		 * 3	User defined
        *		 * 4	User defined
        *		 * 5	User defined
        *		 * 6	User defined
        *		 * 7	User defined*/
        MAV_CMD_USER_4 = 31013,
        /**
        ** User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER
        *		 * item
        *		 * 1	User defined
        *		 * 2	User defined
        *		 * 3	User defined
        *		 * 4	User defined
        *		 * 5	User defined
        *		 * 6	User defined
        *		 * 7	User defined*/
        MAV_CMD_USER_5 = 31014,
        /**
        ** A system wide power-off event has been initiated.
        *		 * 1	Empty
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_POWER_OFF_INITIATED = 42000,
        /**
        ** FLY button has been clicked.
        *		 * 1	Empty
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_SOLO_BTN_FLY_CLICK = 42001,
        /**
        ** FLY button has been held for 1.5 seconds.
        *		 * 1	Takeoff altitude
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_SOLO_BTN_FLY_HOLD = 42002,
        /**
        ** PAUSE button has been clicked.
        *		 * 1	1 if Solo is in a shot mode, 0 otherwise
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_SOLO_BTN_PAUSE_CLICK = 42003,
        /**
        ** Initiate a magnetometer calibration
        *		 * 1	char bitmask of magnetometers (0 means all)
        *		 * 2	Automatically retry on failure (0=no retry, 1=retry).
        *		 * 3	Save without user input (0=require input, 1=autosave).
        *		 * 4	Delay (seconds)
        *		 * 5	Autoreboot (0=user reboot, 1=autoreboot)
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_START_MAG_CAL = 42424,
        /**
        ** Initiate a magnetometer calibration
        *		 * 1	char bitmask of magnetometers (0 means all)
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_ACCEPT_MAG_CAL = 42425,
        /**
        ** Cancel a running magnetometer calibration
        *		 * 1	char bitmask of magnetometers (0 means all)
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_CANCEL_MAG_CAL = 42426,
        /**
        ** Command autopilot to get into factory test/diagnostic mode
        *		 * 1	0 means get out of test mode, 1 means get into test mode
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_SET_FACTORY_TEST_MODE = 42427,
        /**
        ** Reply with the version banner
        *		 * 1	Empty
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_DO_SEND_BANNER = 42428,
        /**
        ** Used when doing accelerometer calibration. When sent to the GCS tells it what position to put the vehicle
        *		 * in. When sent to the vehicle says what position the vehicle is in
        *		 * 1	Position, one of the ACCELCAL_VEHICLE_POS enum values
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_ACCELCAL_VEHICLE_POS = 42429,
        /**
        ** Causes the gimbal to reset and boot as if it was just powered on
        *		 * 1	Empty
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_GIMBAL_RESET = 42501,
        /**
        ** Reports progress and success or failure of gimbal axis calibration procedure
        *		 * 1	Gimbal axis we're reporting calibration progress for
        *		 * 2	Current calibration progress for this axis, 0x64=100%
        *		 * 3	Status of the calibration
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS = 42502,
        /**
        ** Starts commutation calibration on the gimbal
        *		 * 1	Empty
        *		 * 2	Empty
        *		 * 3	Empty
        *		 * 4	Empty
        *		 * 5	Empty
        *		 * 6	Empty
        *		 * 7	Empty*/
        MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION = 42503,
        /**
        ** Erases gimbal application and parameters
        *		 * 1	Magic number
        *		 * 2	Magic number
        *		 * 3	Magic number
        *		 * 4	Magic number
        *		 * 5	Magic number
        *		 * 6	Magic number
        *		 * 7	Magic number*/
        MAV_CMD_GIMBAL_FULL_RESET = 42505;
    }

    protected static int en__T(int id)
    {
        switch(id)
        {
            case 0:
                return MAV_CMD.MAV_CMD_NAV_WAYPOINT;
            case 1:
                return MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM;
            case 2:
                return MAV_CMD.MAV_CMD_NAV_LOITER_TURNS;
            case 3:
                return MAV_CMD.MAV_CMD_NAV_LOITER_TIME;
            case 4:
                return MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH;
            case 5:
                return MAV_CMD.MAV_CMD_NAV_LAND;
            case 6:
                return MAV_CMD.MAV_CMD_NAV_TAKEOFF;
            case 7:
                return MAV_CMD.MAV_CMD_NAV_LAND_LOCAL;
            case 8:
                return MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL;
            case 9:
                return MAV_CMD.MAV_CMD_NAV_FOLLOW;
            case 10:
                return MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
            case 11:
                return MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT;
            case 12:
                return MAV_CMD.MAV_CMD_DO_FOLLOW;
            case 13:
                return MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION;
            case 14:
                return MAV_CMD.MAV_CMD_NAV_ROI;
            case 15:
                return MAV_CMD.MAV_CMD_NAV_PATHPLANNING;
            case 16:
                return MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT;
            case 17:
                return MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT;
            case 18:
                return MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF;
            case 19:
                return MAV_CMD.MAV_CMD_NAV_VTOL_LAND;
            case 20:
                return MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE;
            case 21:
                return MAV_CMD.MAV_CMD_NAV_DELAY;
            case 22:
                return MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE;
            case 23:
                return MAV_CMD.MAV_CMD_NAV_LAST;
            case 24:
                return MAV_CMD.MAV_CMD_CONDITION_DELAY;
            case 25:
                return MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT;
            case 26:
                return MAV_CMD.MAV_CMD_CONDITION_DISTANCE;
            case 27:
                return MAV_CMD.MAV_CMD_CONDITION_YAW;
            case 28:
                return MAV_CMD.MAV_CMD_CONDITION_LAST;
            case 29:
                return MAV_CMD.MAV_CMD_DO_SET_MODE;
            case 30:
                return MAV_CMD.MAV_CMD_DO_JUMP;
            case 31:
                return MAV_CMD.MAV_CMD_DO_CHANGE_SPEED;
            case 32:
                return MAV_CMD.MAV_CMD_DO_SET_HOME;
            case 33:
                return MAV_CMD.MAV_CMD_DO_SET_PARAMETER;
            case 34:
                return MAV_CMD.MAV_CMD_DO_SET_RELAY;
            case 35:
                return MAV_CMD.MAV_CMD_DO_REPEAT_RELAY;
            case 36:
                return MAV_CMD.MAV_CMD_DO_SET_SERVO;
            case 37:
                return MAV_CMD.MAV_CMD_DO_REPEAT_SERVO;
            case 38:
                return MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION;
            case 39:
                return MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE;
            case 40:
                return MAV_CMD.MAV_CMD_DO_LAND_START;
            case 41:
                return MAV_CMD.MAV_CMD_DO_RALLY_LAND;
            case 42:
                return MAV_CMD.MAV_CMD_DO_GO_AROUND;
            case 43:
                return MAV_CMD.MAV_CMD_DO_REPOSITION;
            case 44:
                return MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE;
            case 45:
                return MAV_CMD.MAV_CMD_DO_SET_REVERSE;
            case 46:
                return MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO;
            case 47:
                return MAV_CMD.MAV_CMD_DO_SET_ROI;
            case 48:
                return MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE;
            case 49:
                return MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL;
            case 50:
                return MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE;
            case 51:
                return MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL;
            case 52:
                return MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST;
            case 53:
                return MAV_CMD.MAV_CMD_DO_FENCE_ENABLE;
            case 54:
                return MAV_CMD.MAV_CMD_DO_PARACHUTE;
            case 55:
                return MAV_CMD.MAV_CMD_DO_MOTOR_TEST;
            case 56:
                return MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT;
            case 57:
                return MAV_CMD.MAV_CMD_DO_GRIPPER;
            case 58:
                return MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE;
            case 59:
                return MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED;
            case 60:
                return MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
            case 61:
                return MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT;
            case 62:
                return MAV_CMD.MAV_CMD_DO_GUIDED_MASTER;
            case 63:
                return MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS;
            case 64:
                return MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL;
            case 65:
                return MAV_CMD.MAV_CMD_DO_LAST;
            case 66:
                return MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION;
            case 67:
                return MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
            case 68:
                return MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN;
            case 69:
                return MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE;
            case 70:
                return MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
            case 71:
                return MAV_CMD.MAV_CMD_OVERRIDE_GOTO;
            case 72:
                return MAV_CMD.MAV_CMD_MISSION_START;
            case 73:
                return MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM;
            case 74:
                return MAV_CMD.MAV_CMD_GET_HOME_POSITION;
            case 75:
                return MAV_CMD.MAV_CMD_START_RX_PAIR;
            case 76:
                return MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL;
            case 77:
                return MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL;
            case 78:
                return MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION;
            case 79:
                return MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
            case 80:
                return MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION;
            case 81:
                return MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS;
            case 82:
                return MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION;
            case 83:
                return MAV_CMD.MAV_CMD_STORAGE_FORMAT;
            case 84:
                return MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
            case 85:
                return MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION;
            case 86:
                return MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS;
            case 87:
                return MAV_CMD.MAV_CMD_SET_CAMERA_MODE;
            case 88:
                return MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE;
            case 89:
                return MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE;
            case 90:
                return MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
            case 91:
                return MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL;
            case 92:
                return MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE;
            case 93:
                return MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE;
            case 94:
                return MAV_CMD.MAV_CMD_VIDEO_START_STREAMING;
            case 95:
                return MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING;
            case 96:
                return MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
            case 97:
                return MAV_CMD.MAV_CMD_LOGGING_START;
            case 98:
                return MAV_CMD.MAV_CMD_LOGGING_STOP;
            case 99:
                return MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION;
            case 100:
                return MAV_CMD.MAV_CMD_PANORAMA_CREATE;
            case 101:
                return MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION;
            case 102:
                return MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST;
            case 103:
                return MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
            case 104:
                return MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
            case 105:
                return MAV_CMD.MAV_CMD_CONDITION_GATE;
            case 106:
                return MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT;
            case 107:
                return MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
            case 108:
                return MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
            case 109:
                return MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
            case 110:
                return MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
            case 111:
                return MAV_CMD.MAV_CMD_NAV_RALLY_POINT;
            case 112:
                return MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO;
            case 113:
                return MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
            case 114:
                return MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
            case 115:
                return MAV_CMD.MAV_CMD_WAYPOINT_USER_1;
            case 116:
                return MAV_CMD.MAV_CMD_WAYPOINT_USER_2;
            case 117:
                return MAV_CMD.MAV_CMD_WAYPOINT_USER_3;
            case 118:
                return MAV_CMD.MAV_CMD_WAYPOINT_USER_4;
            case 119:
                return MAV_CMD.MAV_CMD_WAYPOINT_USER_5;
            case 120:
                return MAV_CMD.MAV_CMD_SPATIAL_USER_1;
            case 121:
                return MAV_CMD.MAV_CMD_SPATIAL_USER_2;
            case 122:
                return MAV_CMD.MAV_CMD_SPATIAL_USER_3;
            case 123:
                return MAV_CMD.MAV_CMD_SPATIAL_USER_4;
            case 124:
                return MAV_CMD.MAV_CMD_SPATIAL_USER_5;
            case 125:
                return MAV_CMD.MAV_CMD_USER_1;
            case 126:
                return MAV_CMD.MAV_CMD_USER_2;
            case 127:
                return MAV_CMD.MAV_CMD_USER_3;
            case 128:
                return MAV_CMD.MAV_CMD_USER_4;
            case 129:
                return MAV_CMD.MAV_CMD_USER_5;
            case 130:
                return MAV_CMD.MAV_CMD_POWER_OFF_INITIATED;
            case 131:
                return MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK;
            case 132:
                return MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD;
            case 133:
                return MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK;
            case 134:
                return MAV_CMD.MAV_CMD_DO_START_MAG_CAL;
            case 135:
                return MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL;
            case 136:
                return MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL;
            case 137:
                return MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE;
            case 138:
                return MAV_CMD.MAV_CMD_DO_SEND_BANNER;
            case 139:
                return MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS;
            case 140:
                return MAV_CMD.MAV_CMD_GIMBAL_RESET;
            case 141:
                return MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS;
            case 142:
                return MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION;
            case 143:
                return MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET;
        }
        assert(false);//("Unknown enum ID " + id);
        return  Integer.MIN_VALUE;
    }
    /**
    *result in a mavlink mission ack*/
    public @interface MAV_MISSION_RESULT
    {

        int
        MAV_MISSION_ACCEPTED = 0,//mission accepted OK
        MAV_MISSION_ERROR = 1,//generic error / not accepting mission commands at all right now
        MAV_MISSION_UNSUPPORTED_FRAME = 2,//coordinate frame is not supported
        MAV_MISSION_UNSUPPORTED = 3,//command is not supported
        MAV_MISSION_NO_SPACE = 4,//mission item exceeds storage space
        MAV_MISSION_INVALID = 5,//one of the parameters has an invalid value
        MAV_MISSION_INVALID_PARAM1 = 6,//param1 has an invalid value
        MAV_MISSION_INVALID_PARAM2 = 7,//param2 has an invalid value
        MAV_MISSION_INVALID_PARAM3 = 8,//param3 has an invalid value
        MAV_MISSION_INVALID_PARAM4 = 9,//param4 has an invalid value
        MAV_MISSION_INVALID_PARAM5_X = 10,//x/param5 has an invalid value
        MAV_MISSION_INVALID_PARAM6_Y = 11,//y/param6 has an invalid value
        MAV_MISSION_INVALID_PARAM7 = 12,//param7 has an invalid value
        MAV_MISSION_INVALID_SEQUENCE = 13,//received waypoint out of sequence
        MAV_MISSION_DENIED = 14;//not accepting any mission commands from this communication partner
    }

    /**
    *Enumeration of estimator types*/
    public @interface MAV_ESTIMATOR_TYPE
    {

        int
        MAV_ESTIMATOR_TYPE_NAIVE = 1,//This is a naive estimator without any real covariance feedback.
        MAV_ESTIMATOR_TYPE_VISION = 2,//Computer vision based estimate. Might be up to scale.
        MAV_ESTIMATOR_TYPE_VIO = 3,//Visual-inertial estimate.
        MAV_ESTIMATOR_TYPE_GPS = 4,//Plain GPS estimate.
        MAV_ESTIMATOR_TYPE_GPS_INS = 5;//Estimator integrating GPS and inertial sensing.
    }

    /**
    *result from a mavlink command*/
    public @interface MAV_RESULT
    {

        int
        MAV_RESULT_ACCEPTED = 0,//Command ACCEPTED and EXECUTED
        MAV_RESULT_TEMPORARILY_REJECTED = 1,//Command TEMPORARY REJECTED/DENIED
        MAV_RESULT_DENIED = 2,//Command PERMANENTLY DENIED
        MAV_RESULT_UNSUPPORTED = 3,//Command UNKNOWN/UNSUPPORTED
        MAV_RESULT_FAILED = 4,//Command executed, but failed
        MAV_RESULT_IN_PROGRESS = 5;//WIP: Command being executed
    }

    /**
    *Power supply status flags (bitmask)*/
    public @interface MAV_POWER_STATUS
    {

        int
        MAV_POWER_STATUS_BRICK_VALID = 1,//main brick power supply valid
        MAV_POWER_STATUS_SERVO_VALID = 2,//main servo power supply valid for FMU
        MAV_POWER_STATUS_USB_CONNECTED = 4,//USB power is connected
        MAV_POWER_STATUS_PERIPH_OVERCURRENT = 8,//peripheral supply is in over-current state
        MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16,//hi-power peripheral supply is in over-current state
        MAV_POWER_STATUS_CHANGED = 32;//Power status has changed since boot
    }

    /**
    *SERIAL_CONTROL device types*/
    public @interface SERIAL_CONTROL_DEV
    {

        int
        SERIAL_CONTROL_DEV_TELEM1 = 0,//First telemetry port
        SERIAL_CONTROL_DEV_TELEM2 = 1,//Second telemetry port
        SERIAL_CONTROL_DEV_GPS1 = 2,//First GPS port
        SERIAL_CONTROL_DEV_GPS2 = 3,//Second GPS port
        SERIAL_CONTROL_DEV_SHELL = 10;//system shell
    }

    /**
    *SERIAL_CONTROL flags (bitmask)*/
    public @interface SERIAL_CONTROL_FLAG
    {

        int
        SERIAL_CONTROL_FLAG_REPLY = 1,//Set if this is a reply
        SERIAL_CONTROL_FLAG_RESPOND = 2,//Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
        /**
        ** Set if access to the serial port should be removed from whatever driver is currently using it, giving
        *		 * exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without
        *		 * this flag se*/
        SERIAL_CONTROL_FLAG_EXCLUSIVE = 4,
        SERIAL_CONTROL_FLAG_BLOCKING = 8,//Block on writes to the serial port
        SERIAL_CONTROL_FLAG_MULTI = 16;//Send multiple replies until port is drained
    }

    /**
    *Enumeration of distance sensor types*/
    public @interface MAV_DISTANCE_SENSOR
    {

        int
        MAV_DISTANCE_SENSOR_LASER = 0,//Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
        MAV_DISTANCE_SENSOR_ULTRASOUND = 1,//Ultrasound rangefinder, e.g. MaxBotix units
        MAV_DISTANCE_SENSOR_INFRARED = 2,//Infrared rangefinder, e.g. Sharp units
        MAV_DISTANCE_SENSOR_RADAR = 3,//Radar type, e.g. uLanding units
        MAV_DISTANCE_SENSOR_UNKNOWN = 4;//Broken or unknown type, e.g. analog units
    }

    /**
    *Enumeration of sensor orientation, according to its rotations*/
    public @interface MAV_SENSOR_ORIENTATION
    {

        int
        MAV_SENSOR_ROTATION_NONE = 0,//Roll: 0, Pitch: 0, Yaw: 0
        MAV_SENSOR_ROTATION_YAW_45 = 1,//Roll: 0, Pitch: 0, Yaw: 45
        MAV_SENSOR_ROTATION_YAW_90 = 2,//Roll: 0, Pitch: 0, Yaw: 90
        MAV_SENSOR_ROTATION_YAW_135 = 3,//Roll: 0, Pitch: 0, Yaw: 135
        MAV_SENSOR_ROTATION_YAW_180 = 4,//Roll: 0, Pitch: 0, Yaw: 180
        MAV_SENSOR_ROTATION_YAW_225 = 5,//Roll: 0, Pitch: 0, Yaw: 225
        MAV_SENSOR_ROTATION_YAW_270 = 6,//Roll: 0, Pitch: 0, Yaw: 270
        MAV_SENSOR_ROTATION_YAW_315 = 7,//Roll: 0, Pitch: 0, Yaw: 315
        MAV_SENSOR_ROTATION_ROLL_180 = 8,//Roll: 180, Pitch: 0, Yaw: 0
        MAV_SENSOR_ROTATION_ROLL_180_YAW_45 = 9,//Roll: 180, Pitch: 0, Yaw: 45
        MAV_SENSOR_ROTATION_ROLL_180_YAW_90 = 10,//Roll: 180, Pitch: 0, Yaw: 90
        MAV_SENSOR_ROTATION_ROLL_180_YAW_135 = 11,//Roll: 180, Pitch: 0, Yaw: 135
        MAV_SENSOR_ROTATION_PITCH_180 = 12,//Roll: 0, Pitch: 180, Yaw: 0
        MAV_SENSOR_ROTATION_ROLL_180_YAW_225 = 13,//Roll: 180, Pitch: 0, Yaw: 225
        MAV_SENSOR_ROTATION_ROLL_180_YAW_270 = 14,//Roll: 180, Pitch: 0, Yaw: 270
        MAV_SENSOR_ROTATION_ROLL_180_YAW_315 = 15,//Roll: 180, Pitch: 0, Yaw: 315
        MAV_SENSOR_ROTATION_ROLL_90 = 16,//Roll: 90, Pitch: 0, Yaw: 0
        MAV_SENSOR_ROTATION_ROLL_90_YAW_45 = 17,//Roll: 90, Pitch: 0, Yaw: 45
        MAV_SENSOR_ROTATION_ROLL_90_YAW_90 = 18,//Roll: 90, Pitch: 0, Yaw: 90
        MAV_SENSOR_ROTATION_ROLL_90_YAW_135 = 19,//Roll: 90, Pitch: 0, Yaw: 135
        MAV_SENSOR_ROTATION_ROLL_270 = 20,//Roll: 270, Pitch: 0, Yaw: 0
        MAV_SENSOR_ROTATION_ROLL_270_YAW_45 = 21,//Roll: 270, Pitch: 0, Yaw: 45
        MAV_SENSOR_ROTATION_ROLL_270_YAW_90 = 22,//Roll: 270, Pitch: 0, Yaw: 90
        MAV_SENSOR_ROTATION_ROLL_270_YAW_135 = 23,//Roll: 270, Pitch: 0, Yaw: 135
        MAV_SENSOR_ROTATION_PITCH_90 = 24,//Roll: 0, Pitch: 90, Yaw: 0
        MAV_SENSOR_ROTATION_PITCH_270 = 25,//Roll: 0, Pitch: 270, Yaw: 0
        MAV_SENSOR_ROTATION_PITCH_180_YAW_90 = 26,//Roll: 0, Pitch: 180, Yaw: 90
        MAV_SENSOR_ROTATION_PITCH_180_YAW_270 = 27,//Roll: 0, Pitch: 180, Yaw: 270
        MAV_SENSOR_ROTATION_ROLL_90_PITCH_90 = 28,//Roll: 90, Pitch: 90, Yaw: 0
        MAV_SENSOR_ROTATION_ROLL_180_PITCH_90 = 29,//Roll: 180, Pitch: 90, Yaw: 0
        MAV_SENSOR_ROTATION_ROLL_270_PITCH_90 = 30,//Roll: 270, Pitch: 90, Yaw: 0
        MAV_SENSOR_ROTATION_ROLL_90_PITCH_180 = 31,//Roll: 90, Pitch: 180, Yaw: 0
        MAV_SENSOR_ROTATION_ROLL_270_PITCH_180 = 32,//Roll: 270, Pitch: 180, Yaw: 0
        MAV_SENSOR_ROTATION_ROLL_90_PITCH_270 = 33,//Roll: 90, Pitch: 270, Yaw: 0
        MAV_SENSOR_ROTATION_ROLL_180_PITCH_270 = 34,//Roll: 180, Pitch: 270, Yaw: 0
        MAV_SENSOR_ROTATION_ROLL_270_PITCH_270 = 35,//Roll: 270, Pitch: 270, Yaw: 0
        MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 = 36,//Roll: 90, Pitch: 180, Yaw: 90
        MAV_SENSOR_ROTATION_ROLL_90_YAW_270 = 37,//Roll: 90, Pitch: 0, Yaw: 270
        MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315 = 38;//Roll: 315, Pitch: 315, Yaw: 315
    }

    /**
    *Enumeration of battery functions*/
    public @interface MAV_BATTERY_FUNCTION
    {

        int
        MAV_BATTERY_FUNCTION_UNKNOWN = 0,//Battery function is unknown
        MAV_BATTERY_FUNCTION_ALL = 1,//Battery supports all flight systems
        MAV_BATTERY_FUNCTION_PROPULSION = 2,//Battery for the propulsion system
        MAV_BATTERY_FUNCTION_AVIONICS = 3,//Avionics battery
        MAV_BATTERY_TYPE_PAYLOAD = 4;//Payload battery
    }

    /**
    *Enumeration of battery types*/
    public @interface MAV_BATTERY_TYPE
    {

        int
        MAV_BATTERY_TYPE_UNKNOWN = 0,//Not specified.
        MAV_BATTERY_TYPE_LIPO = 1,//Lithium polymer battery
        MAV_BATTERY_TYPE_LIFE = 2,//Lithium-iron-phosphate battery
        MAV_BATTERY_TYPE_LION = 3,//Lithium-ION battery
        MAV_BATTERY_TYPE_NIMH = 4;//Nickel metal hydride battery
    }

    /**
    *Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability*/
    public @interface MAV_PROTOCOL_CAPABILITY
    {

        int
        MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT = 1,//Autopilot supports MISSION float message type.
        MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT = 2,//Autopilot supports the new param float message type.
        MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4,//Autopilot supports MISSION_INT scaled integer message type.
        MAV_PROTOCOL_CAPABILITY_COMMAND_INT = 8,//Autopilot supports COMMAND_INT scaled integer message type.
        MAV_PROTOCOL_CAPABILITY_PARAM_UNION = 16,//Autopilot supports the new param union message type.
        MAV_PROTOCOL_CAPABILITY_FTP = 32,//Autopilot supports the new FILE_TRANSFER_PROTOCOL message type.
        MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET = 64,//Autopilot supports commanding attitude offboard.
        MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED = 128,//Autopilot supports commanding position and velocity targets in local NED frame.
        MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 256,//Autopilot supports commanding position and velocity targets in global scaled integers.
        MAV_PROTOCOL_CAPABILITY_TERRAIN = 512,//Autopilot supports terrain protocol / data handling.
        MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET = 1024,//Autopilot supports direct actuator control.
        MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION = 2048,//Autopilot supports the flight termination command.
        MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION = 4096,//Autopilot supports onboard compass calibration.
        MAV_PROTOCOL_CAPABILITY_MAVLINK2 = 8192,//Autopilot supports mavlink version 2.
        MAV_PROTOCOL_CAPABILITY_MISSION_FENCE = 16384,//Autopilot supports mission fence protocol.
        MAV_PROTOCOL_CAPABILITY_MISSION_RALLY = 32768,//Autopilot supports mission rally point protocol.
        MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION = 65536;//Autopilot supports the flight information protocol.
    }

    /**
    *Type of landing target*/
    public @interface LANDING_TARGET_TYPE
    {

        int
        LANDING_TARGET_TYPE_LIGHT_BEACON = 0,//Landing target signaled by light beacon (ex: IR-LOCK)
        LANDING_TARGET_TYPE_RADIO_BEACON = 1,//Landing target signaled by radio beacon (ex: ILS, NDB)
        LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2,//Landing target represented by a fiducial marker (ex: ARTag)
        LANDING_TARGET_TYPE_VISION_OTHER = 3;//Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
    }

    /**
    *Enumeration of possible mount operation modes*/
    public @interface MAV_MOUNT_MODE
    {

        int
        MAV_MOUNT_MODE_RETRACT = 0,//Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
        MAV_MOUNT_MODE_NEUTRAL = 1,//Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
        MAV_MOUNT_MODE_MAVLINK_TARGETING = 2,//Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
        MAV_MOUNT_MODE_RC_TARGETING = 3,//Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
        MAV_MOUNT_MODE_GPS_POINT = 4;//Load neutral position and start to point to Lat,Lon,Alt
    }

    public @interface FENCE_BREACH
    {

        int
        FENCE_BREACH_NONE = 0,//No last fence breach
        FENCE_BREACH_MINALT = 1,//Breached minimum altitude
        FENCE_BREACH_MAXALT = 2,//Breached maximum altitude
        FENCE_BREACH_BOUNDARY = 3;//Breached fence boundary
    }

    public @interface LIMITS_STATE
    {

        int
        LIMITS_INIT = 0,//pre-initialization
        LIMITS_DISABLED = 1,//disabled
        LIMITS_ENABLED = 2,//checking limits
        LIMITS_TRIGGERED = 3,//a limit has been breached
        LIMITS_RECOVERING = 4,//taking action eg. RTL
        LIMITS_RECOVERED = 5;//we're no longer in breach of a limit
    }

    public @interface LIMIT_MODULE
    {

        int
        LIMIT_GPSLOCK = 1,//pre-initialization
        LIMIT_GEOFENCE = 2,//disabled
        LIMIT_ALTITUDE = 4;//checking limits
    }

    /**
    *Flags in RALLY_POINT message*/
    public @interface RALLY_FLAGS
    {

        int
        FAVORABLE_WIND = 1,//Flag set when requiring favorable winds for landing.
        /**
        ** Flag set when plane is to immediately descend to break altitude and land without GCS intervention. Flag
        *		 * not set when plane is to loiter at Rally point until commanded to land*/
        LAND_IMMEDIATELY = 2;
    }

    public @interface CAMERA_STATUS_TYPES
    {

        int
        CAMERA_STATUS_TYPE_HEARTBEAT = 0,//Camera heartbeat, announce camera component ID at 1hz
        CAMERA_STATUS_TYPE_TRIGGER = 1,//Camera image triggered
        CAMERA_STATUS_TYPE_DISCONNECT = 2,//Camera connection lost
        CAMERA_STATUS_TYPE_ERROR = 3,//Camera unknown error
        CAMERA_STATUS_TYPE_LOWBATT = 4,//Camera battery low. Parameter p1 shows reported voltage
        CAMERA_STATUS_TYPE_LOWSTORE = 5,//Camera storage low. Parameter p1 shows reported shots remaining
        CAMERA_STATUS_TYPE_LOWSTOREV = 6;//Camera storage low. Parameter p1 shows reported video minutes remaining
    }

    public @interface CAMERA_FEEDBACK_FLAGS
    {

        int
        CAMERA_FEEDBACK_PHOTO = 0,//Shooting photos, not video
        CAMERA_FEEDBACK_VIDEO = 1,//Shooting video, not stills
        CAMERA_FEEDBACK_BADEXPOSURE = 2,//Unable to achieve requested exposure (e.g. shutter speed too low)
        CAMERA_FEEDBACK_CLOSEDLOOP = 3,//Closed loop feedback from camera, we know for sure it has successfully taken a picture
        /**
        ** Open loop camera, an image trigger has been requested but we can't know for sure it has successfully taken
        *		 * a pictur*/
        CAMERA_FEEDBACK_OPENLOOP = 4;
    }

    /**
    *Special ACK block numbers control activation of dataflash log streaming*/
    public @interface MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS
    {

        int
        MAV_REMOTE_LOG_DATA_BLOCK_STOP = 2147483645,//UAV to stop sending DataFlash blocks
        MAV_REMOTE_LOG_DATA_BLOCK_START = 2147483646;//UAV to start sending DataFlash blocks
    }

    /**
    *Possible remote log data block statuses*/
    public @interface MAV_REMOTE_LOG_DATA_BLOCK_STATUSES
    {

        int
        MAV_REMOTE_LOG_DATA_BLOCK_NACK = 0,//This block has NOT been received
        MAV_REMOTE_LOG_DATA_BLOCK_ACK = 1;//This block has been received
    }

    public @interface MAG_CAL_STATUS
    {

        int
        MAG_CAL_NOT_STARTED = 0,
        MAG_CAL_WAITING_TO_START = 1,
        MAG_CAL_RUNNING_STEP_ONE = 2,
        MAG_CAL_RUNNING_STEP_TWO = 3,
        MAG_CAL_SUCCESS = 4,
        MAG_CAL_FAILED = 5;
    }

    /**
    *Flags in EKF_STATUS message*/
    public @interface EKF_STATUS_FLAGS
    {

        int
        EKF_ATTITUDE = 1,//set if EKF's attitude estimate is good
        EKF_VELOCITY_HORIZ = 2,//set if EKF's horizontal velocity estimate is good
        EKF_VELOCITY_VERT = 4,//set if EKF's vertical velocity estimate is good
        EKF_POS_HORIZ_REL = 8,//set if EKF's horizontal position (relative) estimate is good
        EKF_POS_HORIZ_ABS = 16,//set if EKF's horizontal position (absolute) estimate is good
        EKF_POS_VERT_ABS = 32,//set if EKF's vertical position (absolute) estimate is good
        EKF_POS_VERT_AGL = 64,//set if EKF's vertical position (above ground) estimate is good
        EKF_CONST_POS_MODE = 128,//EKF is in constant position mode and does not know it's absolute or relative position
        EKF_PRED_POS_HORIZ_REL = 256,//set if EKF's predicted horizontal position (relative) estimate is good
        EKF_PRED_POS_HORIZ_ABS = 512;//set if EKF's predicted horizontal position (absolute) estimate is good
    }

    public @interface PID_TUNING_AXIS
    {

        int
        PID_TUNING_ROLL = 1,
        PID_TUNING_PITCH = 2,
        PID_TUNING_YAW = 3,
        PID_TUNING_ACCZ = 4,
        PID_TUNING_STEER = 5,
        PID_TUNING_LANDING = 6;
    }

    public @interface GOPRO_HEARTBEAT_STATUS
    {

        int
        GOPRO_HEARTBEAT_STATUS_DISCONNECTED = 0,//No GoPro connected
        GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE = 1,//The detected GoPro is not HeroBus compatible
        GOPRO_HEARTBEAT_STATUS_CONNECTED = 2,//A HeroBus compatible GoPro is connected
        GOPRO_HEARTBEAT_STATUS_ERROR = 3;//An unrecoverable error was encountered with the connected GoPro, it may require a power cycle
    }

    public @interface GOPRO_CAPTURE_MODE
    {

        int
        GOPRO_CAPTURE_MODE_VIDEO = 0,//Video mode
        GOPRO_CAPTURE_MODE_PHOTO = 1,//Photo mode
        GOPRO_CAPTURE_MODE_BURST = 2,//Burst mode, hero 3+ only
        GOPRO_CAPTURE_MODE_TIME_LAPSE = 3,//Time lapse mode, hero 3+ only
        GOPRO_CAPTURE_MODE_MULTI_SHOT = 4,//Multi shot mode, hero 4 only
        GOPRO_CAPTURE_MODE_PLAYBACK = 5,//Playback mode, hero 4 only, silver only except when LCD or HDMI is connected to black
        GOPRO_CAPTURE_MODE_SETUP = 6,//Playback mode, hero 4 only
        GOPRO_CAPTURE_MODE_UNKNOWN = 255;//Mode not yet known
    }

    public @interface GOPRO_HEARTBEAT_FLAGS
    {

        int
        GOPRO_FLAG_RECORDING = 1;//GoPro is currently recording
    }

    public @interface GOPRO_COMMAND
    {

        int
        GOPRO_COMMAND_POWER = 0,//(Get/Set)
        GOPRO_COMMAND_CAPTURE_MODE = 1,//(Get/Set)
        GOPRO_COMMAND_SHUTTER = 2,//(___/Set)
        GOPRO_COMMAND_BATTERY = 3,//(Get/___)
        GOPRO_COMMAND_MODEL = 4,//(Get/___)
        GOPRO_COMMAND_VIDEO_SETTINGS = 5,//(Get/Set)
        GOPRO_COMMAND_LOW_LIGHT = 6,//(Get/Set)
        GOPRO_COMMAND_PHOTO_RESOLUTION = 7,//(Get/Set)
        GOPRO_COMMAND_PHOTO_BURST_RATE = 8,//(Get/Set)
        GOPRO_COMMAND_PROTUNE = 9,//(Get/Set)
        GOPRO_COMMAND_PROTUNE_WHITE_BALANCE = 10,//(Get/Set) Hero 3+ Only
        GOPRO_COMMAND_PROTUNE_COLOUR = 11,//(Get/Set) Hero 3+ Only
        GOPRO_COMMAND_PROTUNE_GAIN = 12,//(Get/Set) Hero 3+ Only
        GOPRO_COMMAND_PROTUNE_SHARPNESS = 13,//(Get/Set) Hero 3+ Only
        GOPRO_COMMAND_PROTUNE_EXPOSURE = 14,//(Get/Set) Hero 3+ Only
        GOPRO_COMMAND_TIME = 15,//(Get/Set)
        GOPRO_COMMAND_CHARGING = 16;//(Get/Set)
    }

    public @interface GOPRO_REQUEST_STATUS
    {

        int
        GOPRO_REQUEST_SUCCESS = 0,//The write message with ID indicated succeeded
        GOPRO_REQUEST_FAILED = 1;//The write message with ID indicated failed
    }

    /**
    *Flags in EKF_STATUS message*/
    public @interface ESTIMATOR_STATUS_FLAGS
    {

        int
        ESTIMATOR_ATTITUDE = 1,//True if the attitude estimate is good
        ESTIMATOR_VELOCITY_HORIZ = 2,//True if the horizontal velocity estimate is good
        ESTIMATOR_VELOCITY_VERT = 4,//True if the  vertical velocity estimate is good
        ESTIMATOR_POS_HORIZ_REL = 8,//True if the horizontal position (relative) estimate is good
        ESTIMATOR_POS_HORIZ_ABS = 16,//True if the horizontal position (absolute) estimate is good
        ESTIMATOR_POS_VERT_ABS = 32,//True if the vertical position (absolute) estimate is good
        ESTIMATOR_POS_VERT_AGL = 64,//True if the vertical position (above ground) estimate is good
        /**
        ** True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical
        *		 * flow*/
        ESTIMATOR_CONST_POS_MODE = 128,
        ESTIMATOR_PRED_POS_HORIZ_REL = 256,//True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimat
        ESTIMATOR_PRED_POS_HORIZ_ABS = 512,//True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimat
        ESTIMATOR_GPS_GLITCH = 1024;//True if the EKF has detected a GPS glitch
    }

    public @interface GPS_INPUT_IGNORE_FLAGS
    {

        int
        GPS_INPUT_IGNORE_FLAG_ALT = 1,//ignore altitude field
        GPS_INPUT_IGNORE_FLAG_HDOP = 2,//ignore hdop field
        GPS_INPUT_IGNORE_FLAG_VDOP = 4,//ignore vdop field
        GPS_INPUT_IGNORE_FLAG_VEL_HORIZ = 8,//ignore horizontal velocity field (vn and ve)
        GPS_INPUT_IGNORE_FLAG_VEL_VERT = 16,//ignore vertical velocity field (vd)
        GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY = 32,//ignore speed accuracy field
        GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY = 64,//ignore horizontal accuracy field
        GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY = 128;//ignore vertical accuracy field
    }

    /**
    *Enumeration of landed detector states*/
    public @interface MAV_LANDED_STATE
    {

        int
        MAV_LANDED_STATE_UNDEFINED = 0,//MAV landed state is unknown
        MAV_LANDED_STATE_ON_GROUND = 1,//MAV is landed (on ground)
        MAV_LANDED_STATE_IN_AIR = 2,//MAV is in air
        MAV_LANDED_STATE_TAKEOFF = 3,//MAV currently taking off
        MAV_LANDED_STATE_LANDING = 4;//MAV currently landing
    }

    /**
    *Enumeration of VTOL states*/
    public @interface MAV_VTOL_STATE
    {

        int
        MAV_VTOL_STATE_UNDEFINED = 0,//MAV is not configured as VTOL
        MAV_VTOL_STATE_TRANSITION_TO_FW = 1,//VTOL is in transition from multicopter to fixed-wing
        MAV_VTOL_STATE_TRANSITION_TO_MC = 2,//VTOL is in transition from fixed-wing to multicopter
        MAV_VTOL_STATE_MC = 3,//VTOL is in multicopter state
        MAV_VTOL_STATE_FW = 4;//VTOL is in fixed-wing state
    }

    /**
    *Enumeration of the ADSB altimeter types*/
    public @interface ADSB_ALTITUDE_TYPE
    {

        int
        ADSB_ALTITUDE_TYPE_PRESSURE_QNH = 0,//Altitude reported from a Baro source using QNH reference
        ADSB_ALTITUDE_TYPE_GEOMETRIC = 1;//Altitude reported from a GNSS source
    }

    /**
    *ADSB classification for the type of vehicle emitting the transponder signal*/
    public @interface ADSB_EMITTER_TYPE
    {

        int
        ADSB_EMITTER_TYPE_NO_INFO = 0,
        ADSB_EMITTER_TYPE_LIGHT = 1,
        ADSB_EMITTER_TYPE_SMALL = 2,
        ADSB_EMITTER_TYPE_LARGE = 3,
        ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE = 4,
        ADSB_EMITTER_TYPE_HEAVY = 5,
        ADSB_EMITTER_TYPE_HIGHLY_MANUV = 6,
        ADSB_EMITTER_TYPE_ROTOCRAFT = 7,
        ADSB_EMITTER_TYPE_UNASSIGNED = 8,
        ADSB_EMITTER_TYPE_GLIDER = 9,
        ADSB_EMITTER_TYPE_LIGHTER_AIR = 10,
        ADSB_EMITTER_TYPE_PARACHUTE = 11,
        ADSB_EMITTER_TYPE_ULTRA_LIGHT = 12,
        ADSB_EMITTER_TYPE_UNASSIGNED2 = 13,
        ADSB_EMITTER_TYPE_UAV = 14,
        ADSB_EMITTER_TYPE_SPACE = 15,
        ADSB_EMITTER_TYPE_UNASSGINED3 = 16,
        ADSB_EMITTER_TYPE_EMERGENCY_SURFACE = 17,
        ADSB_EMITTER_TYPE_SERVICE_SURFACE = 18,
        ADSB_EMITTER_TYPE_POINT_OBSTACLE = 19;
    }

    /**
    *These flags indicate status such as data validity of each data source. Set = data valid*/
    public @interface ADSB_FLAGS
    {

        int
        ADSB_FLAGS_VALID_COORDS = 1,
        ADSB_FLAGS_VALID_ALTITUDE = 2,
        ADSB_FLAGS_VALID_HEADING = 4,
        ADSB_FLAGS_VALID_VELOCITY = 8,
        ADSB_FLAGS_VALID_CALLSIGN = 16,
        ADSB_FLAGS_VALID_SQUAWK = 32,
        ADSB_FLAGS_SIMULATED = 64;
    }

    /**
    *Source of information about this collision.*/
    public @interface MAV_COLLISION_SRC
    {

        int
        MAV_COLLISION_SRC_ADSB = 0,//ID field references ADSB_VEHICLE packets
        MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT = 1;//ID field references MAVLink SRC ID
    }

    /**
    *Possible actions an aircraft can take to avoid a collision.*/
    public @interface MAV_COLLISION_ACTION
    {

        int
        MAV_COLLISION_ACTION_NONE = 0,//Ignore any potential collisions
        MAV_COLLISION_ACTION_REPORT = 1,//Report potential collision
        MAV_COLLISION_ACTION_ASCEND_OR_DESCEND = 2,//Ascend or Descend to avoid threat
        MAV_COLLISION_ACTION_MOVE_HORIZONTALLY = 3,//Move horizontally to avoid threat
        MAV_COLLISION_ACTION_MOVE_PERPENDICULAR = 4,//Aircraft to move perpendicular to the collision's velocity vector
        MAV_COLLISION_ACTION_RTL = 5,//Aircraft to fly directly back to its launch point
        MAV_COLLISION_ACTION_HOVER = 6;//Aircraft to stop in place
    }

    /**
    *Aircraft-rated danger from this threat.*/
    public @interface MAV_COLLISION_THREAT_LEVEL
    {

        int
        MAV_COLLISION_THREAT_LEVEL_NONE = 0,//Not a threat
        MAV_COLLISION_THREAT_LEVEL_LOW = 1,//Craft is mildly concerned about this threat
        MAV_COLLISION_THREAT_LEVEL_HIGH = 2;//Craft is panicing, and may take actions to avoid threat
    }

    /**
    *Indicates the severity level, generally used for status messages to indicate their relative urgency. Based
    *	 on RFC-5424 using expanded definitions at: http:www.kiwisyslog.com/kb/info:-syslog-message-levels/*/
    public @interface MAV_SEVERITY
    {

        int
        MAV_SEVERITY_EMERGENCY = 0,//System is unusable. This is a "panic" condition.
        MAV_SEVERITY_ALERT = 1,//Action should be taken immediately. Indicates error in non-critical systems.
        MAV_SEVERITY_CRITICAL = 2,//Action must be taken immediately. Indicates failure in a primary system.
        MAV_SEVERITY_ERROR = 3,//Indicates an error in secondary/redundant systems.
        /**
        ** Indicates about a possible future error if this is not resolved within a given timeframe. Example would
        *		 * be a low battery warning*/
        MAV_SEVERITY_WARNING = 4,
        /**
        ** An unusual event has occured, though not an error condition. This should be investigated for the root
        *		 * cause*/
        MAV_SEVERITY_NOTICE = 5,
        MAV_SEVERITY_INFO = 6,//Normal operational messages. Useful for logging. No action is required for these messages.
        MAV_SEVERITY_DEBUG = 7;//Useful non-operational messages that can assist in debugging. These should not occur during normal operation
    }

    /**
    *Camera capability flags (Bitmap).*/
    public @interface CAMERA_CAP_FLAGS
    {

        int
        CAMERA_CAP_FLAGS_CAPTURE_VIDEO = 1,//Camera is able to record video.
        CAMERA_CAP_FLAGS_CAPTURE_IMAGE = 2,//Camera is able to capture images.
        CAMERA_CAP_FLAGS_HAS_MODES = 4,//Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)
        CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = 8,//Camera can capture images while in video mode
        CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = 16,//Camera can capture videos while in Photo/Image mode
        CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE = 32;//Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)
    }

    /**
    *Camera Modes.*/
    public @interface CAMERA_MODE
    {

        int
        CAMERA_MODE_IMAGE = 0,//Camera is in image/photo capture mode.
        CAMERA_MODE_VIDEO = 1,//Camera is in video capture mode.
        CAMERA_MODE_IMAGE_SURVEY = 2;//Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys
    }

    /**
    *Generalized UAVCAN node health*/
    public @interface UAVCAN_NODE_HEALTH
    {

        int
        UAVCAN_NODE_HEALTH_OK = 0,//The node is functioning properly.
        UAVCAN_NODE_HEALTH_WARNING = 1,//A critical parameter went out of range or the node has encountered a minor failure.
        UAVCAN_NODE_HEALTH_ERROR = 2,//The node has encountered a major failure.
        UAVCAN_NODE_HEALTH_CRITICAL = 3;//The node has suffered a fatal malfunction.
    }

    /**
    *Generalized UAVCAN node mode*/
    public @interface UAVCAN_NODE_MODE
    {

        int
        UAVCAN_NODE_MODE_OPERATIONAL = 0,//The node is performing its primary functions.
        UAVCAN_NODE_MODE_INITIALIZATION = 1,//The node is initializing; this mode is entered immediately after startup.
        UAVCAN_NODE_MODE_MAINTENANCE = 2,//The node is under maintenance.
        UAVCAN_NODE_MODE_SOFTWARE_UPDATE = 3,//The node is in the process of updating its software.
        UAVCAN_NODE_MODE_OFFLINE = 7;//The node is no longer available online.
    }

    /**
    *Specifies the datatype of a MAVLink extended parameter.*/
    public @interface MAV_PARAM_EXT_TYPE
    {

        int
        MAV_PARAM_EXT_TYPE_UINT8 = 1,//8-bit unsigned integer
        MAV_PARAM_EXT_TYPE_INT8 = 2,//8-bit signed integer
        MAV_PARAM_EXT_TYPE_UINT16 = 3,//16-bit unsigned integer
        MAV_PARAM_EXT_TYPE_INT16 = 4,//16-bit signed integer
        MAV_PARAM_EXT_TYPE_UINT32 = 5,//32-bit unsigned integer
        MAV_PARAM_EXT_TYPE_INT32 = 6,//32-bit signed integer
        MAV_PARAM_EXT_TYPE_UINT64 = 7,//64-bit unsigned integer
        MAV_PARAM_EXT_TYPE_INT64 = 8,//64-bit signed integer
        MAV_PARAM_EXT_TYPE_REAL32 = 9,//32-bit floating-point
        MAV_PARAM_EXT_TYPE_REAL64 = 10,//64-bit floating-point
        MAV_PARAM_EXT_TYPE_CUSTOM = 11;//Custom Type
    }

    /**
    *Result from a PARAM_EXT_SET message.*/
    public @interface PARAM_ACK
    {

        int
        PARAM_ACK_ACCEPTED = 0,//Parameter value ACCEPTED and SET
        PARAM_ACK_VALUE_UNSUPPORTED = 1,//Parameter value UNKNOWN/UNSUPPORTED
        PARAM_ACK_FAILED = 2,//Parameter failed to set
        /**
        ** Parameter value received but not yet validated or set. A subsequent PARAM_EXT_ACK will follow once operation
        *		 * is completed with the actual result. These are for parameters that may take longer to set. Instead of
        *		 * waiting for an ACK and potentially timing out, you will immediately receive this response to let you
        *		 * know it was received*/
        PARAM_ACK_IN_PROGRESS = 3;
    }

    /**
    *Definitions for aircraft size*/
    public @interface UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE
    {

        int
        UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA = 0,
        UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L15M_W23M = 1,
        UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25M_W28P5M = 2,
        UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25_34M = 3,
        UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_33M = 4,
        UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_38M = 5,
        UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_39P5M = 6,
        UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_45M = 7,
        UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_45M = 8,
        UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_52M = 9,
        UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_59P5M = 10,
        UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_67M = 11,
        UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W72P5M = 12,
        UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W80M = 13,
        UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W80M = 14,
        UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W90M = 15;
    }

    /**
    *GPS lataral offset encoding*/
    public @interface UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT
    {

        int
        UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_NO_DATA = 0,
        UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_2M = 1,
        UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_4M = 2,
        UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_6M = 3,
        UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M = 4,
        UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_2M = 5,
        UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_4M = 6,
        UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M = 7;
    }

    /**
    *GPS longitudinal offset encoding*/
    public @interface UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON
    {

        int
        UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA = 0,
        UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR = 1;
    }

    /**
    *Transceiver RF control flags for ADS-B transponder dynamic reports*/
    public @interface UAVIONIX_ADSB_OUT_RF_SELECT
    {

        int
        UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY = 0,
        UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED = 1,
        UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED = 2;
    }

    /**
    *Status for ADS-B transponder dynamic input*/
    public @interface UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX
    {

        int
        UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0 = 0,
        UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_1 = 1,
        UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_2D = 2,
        UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_3D = 3,
        UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS = 4,
        UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK = 5;
    }

    /**
    *Emergency status encoding*/
    public @interface UAVIONIX_ADSB_EMERGENCY_STATUS
    {

        int
        UAVIONIX_ADSB_OUT_NO_EMERGENCY = 0,
        UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY = 1,
        UAVIONIX_ADSB_OUT_LIFEGUARD_EMERGENCY = 2,
        UAVIONIX_ADSB_OUT_MINIMUM_FUEL_EMERGENCY = 3,
        UAVIONIX_ADSB_OUT_NO_COMM_EMERGENCY = 4,
        UAVIONIX_ADSB_OUT_UNLAWFUL_INTERFERANCE_EMERGENCY = 5,
        UAVIONIX_ADSB_OUT_DOWNED_AIRCRAFT_EMERGENCY = 6,
        UAVIONIX_ADSB_OUT_RESERVED = 7;
    }

    /**
    *State flags for ADS-B transponder dynamic report*/
    public @interface UAVIONIX_ADSB_OUT_DYNAMIC_STATE
    {

        int
        UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE = 1,
        UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED = 2,
        UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED = 4,
        UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND = 8,
        UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT = 16;
    }

    /**
    *Status flags for ADS-B transponder dynamic output*/
    public @interface UAVIONIX_ADSB_RF_HEALTH
    {

        int
        UAVIONIX_ADSB_RF_HEALTH_INITIALIZING = 0,
        UAVIONIX_ADSB_RF_HEALTH_OK = 1,
        UAVIONIX_ADSB_RF_HEALTH_FAIL_TX = 2,
        UAVIONIX_ADSB_RF_HEALTH_FAIL_RX = 16;
    }

    /**
    *Bus types for device operations*/
    public @interface DEVICE_OP_BUSTYPE
    {

        int
        DEVICE_OP_BUSTYPE_I2C = 0,//I2C Device operation
        DEVICE_OP_BUSTYPE_SPI = 1;//SPI Device operation
    }


    private static final Field _J = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _W = new Field(5, true, -6, 2, 1, 0, 0, 0, 1);
    private static final Field _h = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _w = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _o = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _Hd = new Field(0, false, 1, 4, 1, 0, 0, 0);
    private static final Field _Cd = new Field(0, true, 1, 4, 1, 0, 0, 0);
    private static final Field _rd = new Field(0, true, 1, 4, 1, 0, 0, 0);
    private static final Field _Sd = new Field(0, true, 1, 4, 1, 0, 0, 0);
    private static final Field _id = new Field(0, true, 1, 4, 1, 0, 0, 0);
    private static final Field _OB = new Field(0, true, 1, 2, 1, 0, 0, 0);
    private static final Field _HB = new Field(0, true, 1, 2, 1, 0, 0, 0);
    private static final Field _CB = new Field(0, true, 1, 2, 1, 0, 0, 0);
    private static final Field _rB = new Field(0, true, 1, 2, 1, 0, 0, 0);
    private static final Field _SB = new Field(0, true, 1, 2, 1, 0, 0, 0);
    private static final Field _iB = new Field(0, true, 1, 2, 1, 0, 0, 0);
    private static final Field _lB = new Field(0, true, 1, 2, 1, 0, 0, 0);
    private static final Field _aB = new Field(0, true, 1, 2, 1, 0, 0, 0);
    private static final Field _cM = new Field(0, true, 1, 8, 1, 0, 0, 0);
    private static final Field _IM = new Field(0, true, 1, 8, 1, 0, 0, 0);
    private static final Field _GM = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _kP = new Field(0, false, 1, 1, 1, 0, 0, 0);
    private static final Field _GP = new Field(0, false, 1, 4, 1, 0, 0, 0);
    private static final Field _fP = new Field(0, false, 1, 1, 1, 0, 0, 0);
    private static final Field _tP = new Field(0, false, 1, 1, 1, 0, 0, 0);
    private static final Field _iH = new Field(0, false, 1, 4, 1, 0, 0, 0);
    private static final Field _lH = new Field(0, false, 1, 4, 1, 0, 0, 0);
    private static final Field _mc = new Field(0, false, 18, 1, 1, 0, 0, 0);
    private static final Field _ic = new Field(0, false, 1, 4, 1, 0, 0, 0);
    private static final Field _lc = new Field(0, false, 1, 4, 1, 0, 0, 0);
    private static final Field _ac = new Field(0, false, 1, 4, 1, 0, 0, 0);
    private static final Field _cc = new Field(0, false, 4, 4, 1, 0, 0, 0);
    private static final Field _Uc = new Field(0, false, 1, 1, 1, 0, 0, 0);
    private static final Field _ob = new Field(0, true, 1, 8, 1, 0, 0, 0);
    private static final Field _OU = new Field(0, true, 1, 8, 1, 0, 0, 0);
    private static final Field _xU = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
    private static final Field _FU = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
    private static final Field _oU = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
    private static final Field _TD = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
    private static final Field _BD = new Field(5, true, -6, 2, 1, 0, 0, 0, 1);
    private static final Field _aD = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _QD = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
    private static final Field _mI = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
    private static final Field _YI = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
    private static final Field _zI = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
    private static final Field _VI = new Field(5, true, -6, 2, 1, 0, 0, 0, 1);
    private static final Field _RI = new Field(5, true, -7, 2, 1, 0, 0, 0, 1);
    private static final Field _Ox = new Field(5, true, -7, 2, 1, 0, 0, 0, 1);
    private static final Field _bx = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _xx = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _kx = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
    private static final Field _Qx = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _qx = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
    private static final Field _Wx = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _jx = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
    private static final Field _Fx = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
    private static final Field _Nk = new Field(0, true, 1, 4, 1, 0, 0, 0);
    private static final Field _dQ = new Field(5, true, -6, 2, 1, 0, 0, 0, 1);
    private static final Field _lQ = new Field(5, true, -6, 2, 1, 0, 0, 0, 1);

}
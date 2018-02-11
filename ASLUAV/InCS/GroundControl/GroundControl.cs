
using System;
using System.Collections.Generic;
using System.Threading;
using System.Diagnostics;
using org.unirail.BlackBox;
using Inside = org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds.Inside;
using Field = org.unirail.BlackBox.Host.Pack.Meta.Field;
namespace org.noname
{
    public class GroundControl : Host
    {
        /**
        *The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot
        *	 hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying
        *	 out the user interface based on the autopilot)*/
        public class HEARTBEAT : Pack
        {
            internal HEARTBEAT() : base(meta0, 0) { }
            internal HEARTBEAT(int bytes) : base(meta0, bytes) { }
            public uint custom_mode //A bitfield for use for autopilot-specific flags.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte mavlink_version //MAVLink version, not writable by user, gets added by protocol because of magic data type: byte_mavlink_versio
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public MAV_TYPE type //Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
            {
                get {  return (MAV_TYPE)(0 +  BitUtils.get_bits(data, 40, 5));}
            }

            public MAV_AUTOPILOT autopilot //Autopilot type / class. defined in MAV_AUTOPILOT ENUM
            {
                get {  return (MAV_AUTOPILOT)(0 +  BitUtils.get_bits(data, 45, 5));}
            }

            public MAV_MODE_FLAG base_mode //System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
            {
                get {  return (MAV_MODE_FLAG)(1 +  BitUtils.get_bits(data, 50, 8));}
            }

            public MAV_STATE system_status //System status flag, see MAV_STATE ENUM
            {
                get {  return (MAV_STATE)(0 +  BitUtils.get_bits(data, 58, 4));}
            }
            static readonly Meta meta0 = new Meta(0, 0, 1, 0, 8, 62);
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
        public class SYS_STATUS : Pack
        {
            internal SYS_STATUS() : base(meta1, 0) { }
            internal SYS_STATUS(int bytes) : base(meta1, bytes) { }
            public ushort load //Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort voltage_battery //Battery voltage, in millivolts (1 = 1 millivolt)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            /**
            *Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links
            *	 (packets that were corrupted on reception on the MAV*/
            public ushort drop_rate_comm
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            /**
            *Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
            *	 on reception on the MAV*/
            public ushort errors_comm
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
            }

            public ushort errors_count1 //Autopilot-specific errors
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  8, 2));}
            }

            public ushort errors_count2 //Autopilot-specific errors
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  10, 2));}
            }

            public ushort errors_count3 //Autopilot-specific errors
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  12, 2));}
            }

            public ushort errors_count4 //Autopilot-specific errors
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  14, 2));}
            }

            public short current_battery //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  16, 2));}
            }

            public sbyte battery_remaining //Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  18, 1));}
            }

            /**
            *Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1:
            *	 present. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
            public MAV_SYS_STATUS_SENSOR onboard_control_sensors_present
            {
                get {  return (MAV_SYS_STATUS_SENSOR)(1 +  BitUtils.get_bits(data, 152, 26));}
            }

            /**
            *Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of
            *	 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
            public MAV_SYS_STATUS_SENSOR onboard_control_sensors_enabled
            {
                get {  return (MAV_SYS_STATUS_SENSOR)(1 +  BitUtils.get_bits(data, 178, 26));}
            }

            /**
            *Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not
            *	 enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
            public MAV_SYS_STATUS_SENSOR onboard_control_sensors_health
            {
                get {  return (MAV_SYS_STATUS_SENSOR)(1 +  BitUtils.get_bits(data, 204, 26));}
            }
            static readonly Meta meta1 = new Meta(1, 8, 0, 0, 29, 230);
        }/**
*The system time is the time of the master clock, typically the computer clock of the main onboard computer*/
        public class SYSTEM_TIME : Pack
        {
            internal SYSTEM_TIME() : base(meta2, 0) { }
            internal SYSTEM_TIME(int bytes) : base(meta2, bytes) { }
            public uint time_boot_ms //Timestamp of the component clock since boot time in milliseconds.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public ulong time_unix_usec //Timestamp of the master clock in microseconds since UNIX epoch.
            {
                get {  return (BitUtils.get_bytes(data,  4, 8));}
            }
            static readonly Meta meta2 = new Meta(2, 0, 1, 1, 12, 96);
        }/**
*Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
*	 This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled
*	 this way*/
        public class POSITION_TARGET_LOCAL_NED : Pack, CommunicationChannel.Sendable
        {
            internal POSITION_TARGET_LOCAL_NED() : base(meta3, 0) { }
            internal POSITION_TARGET_LOCAL_NED(int bytes) : base(meta3, bytes) { }
            /**
            *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
            *	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
            *	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
            *	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
            *	 bit 11: yaw, bit 12: yaw rat*/
            public ushort type_mask
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_boot_ms //Timestamp in milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public float x //X Position in NED frame in meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public float y //Y Position in NED frame in meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float z //Z Position in NED frame in meters (note, altitude is negative in NED)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float vx //X velocity in NED frame in meter / s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float vy //Y velocity in NED frame in meter / s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            public float vz //Z velocity in NED frame in meter / s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 26);}
            }

            public float afx //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public float afy //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  34, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 34);}
            }

            public float afz //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  38, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 38);}
            }

            public float yaw //yaw setpoint in rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  42, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 42);}
            }

            public float yaw_rate //yaw rate setpoint in rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  46, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 46);}
            }

            /**
            *Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
            *	 =*/
            public MAV_FRAME coordinate_frame
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 400, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 400);}
            }
            static readonly Meta meta3 = new Meta(3, 1, 1, 0, 51, 404);
        }/**
*A ping message either requesting or responding to a ping. This allows to measure the system latencies,
*	 including serial port, radio modem and UDP connections*/
        public class PING : Pack
        {
            internal PING() : base(meta4, 0) { }
            internal PING(int bytes) : base(meta4, bytes) { }
            public uint seq //PING sequence
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public ulong time_usec //Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
            {
                get {  return (BitUtils.get_bytes(data,  4, 8));}
            }

            /**
            *0: request ping from all receiving systems, if greater than 0: message is a ping response and number is
            *	 the system id of the requesting syste*/
            public byte target_system
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  12, 1));}
            }

            /**
            *0: request ping from all receiving components, if greater than 0: message is a ping response and number
            *	 is the system id of the requesting syste*/
            public byte target_component
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  13, 1));}
            }
            static readonly Meta meta4 = new Meta(4, 0, 1, 1, 14, 112);
        }/**
*Request to control this MAV*/
        public class CHANGE_OPERATOR_CONTROL : Pack
        {
            internal CHANGE_OPERATOR_CONTROL() : base(meta5, 0) { }
            internal CHANGE_OPERATOR_CONTROL(int bytes) : base(meta5, bytes) { }
            public byte target_system //System the GCS requests control for
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte control_request //0: request control of this MAV, 1: Release control of this MAV
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            /**
            *0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use
            *	 the safest mode possible initially and then gradually move down the encryption level if it gets a NACK
            *	 message indicating an encryption mismatch*/
            public byte version
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }
            /**
            *Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
            *	 characters may involve A-Z, a-z, 0-9, and "!?,.-*/
            public string passkey_TRY(Inside ph)
            {
                if(ph.field_bit !=  24 && !try_visit_field(ph, 24)  ||  !try_visit_item(ph, 0)) return null;
                return new string(passkey_GET(ph, new char[ph.items], 0));
            }
            /**
            *Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
            *	 characters may involve A-Z, a-z, 0-9, and "!?,.-*/
            public char[]passkey_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int passkey_LEN(Inside ph)
            {
                return (ph.field_bit !=  24 && !try_visit_field(ph, 24)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } static readonly Meta meta5 = new Meta(5, 0, 0, 0, 4, 24, 0, _Z);
        }/**
*Accept / deny control of this MAV*/
        public class CHANGE_OPERATOR_CONTROL_ACK : Pack
        {
            internal CHANGE_OPERATOR_CONTROL_ACK() : base(meta6, 0) { }
            internal CHANGE_OPERATOR_CONTROL_ACK(int bytes) : base(meta6, bytes) { }
            public byte gcs_system_id //ID of the GCS this message
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte control_request //0: request control of this MAV, 1: Release control of this MAV
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            /**
            *0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under
            *	 contro*/
            public byte ack
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }
            static readonly Meta meta6 = new Meta(6, 0, 0, 0, 3, 24);
        }/**
*Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple,
*	 so transmitting the key requires an encrypted channel for true safety*/
        public class AUTH_KEY : Pack
        {
            internal AUTH_KEY() : base(meta7, 0) { }
            internal AUTH_KEY(int bytes) : base(meta7, bytes) { }  public string key_TRY(Inside ph)//key
            {
                if(ph.field_bit !=  0 && !try_visit_field(ph, 0)  ||  !try_visit_item(ph, 0)) return null;
                return new string(key_GET(ph, new char[ph.items], 0));
            }
            public char[]key_GET(Inside ph, char[] dst_ch, int pos) //key
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int key_LEN(Inside ph)
            {
                return (ph.field_bit !=  0 && !try_visit_field(ph, 0)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } static readonly Meta meta7 = new Meta(7, 0, 0, 0, 1, 0, 0, _X);
        }/**
*THIS INTERFACE IS DEPRECATED. USE COMMAND_LONG with MAV_CMD_DO_SET_MODE INSTEAD. Set the system mode,
*	 as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall
*	 aircraft, not only for one component*/
        public class SET_MODE : Pack
        {
            internal SET_MODE() : base(meta11, 0) { }
            internal SET_MODE(int bytes) : base(meta11, bytes) { }
            public uint custom_mode //The new autopilot-specific mode. This field can be ignored by an autopilot.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte target_system //The system setting the mode
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public MAV_MODE base_mode //The new base mode
            {
                get {  return  en__j(BitUtils.get_bits(data, 40, 4));}
            }
            static readonly Meta meta11 = new Meta(11, 0, 1, 0, 6, 44);
        }/**
*value[float]. This allows to send a parameter to any other component (such as the GCS) without the need
*	 of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for
*	 different autopilots. See also http:qgroundcontrol.org/parameter_interface for a full documentation
*	 of QGroundControl and IMU code*/
        public class PARAM_REQUEST_READ : Pack
        {
            internal PARAM_REQUEST_READ() : base(meta20, 0) { }
            internal PARAM_REQUEST_READ(int bytes) : base(meta20, bytes) { }
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public short param_index //Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	 storage if the ID is stored as strin*/
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	 storage if the ID is stored as strin*/
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } static readonly Meta meta20 = new Meta(20, 0, 0, 0, 5, 32, 0, _P);
        }/**
*Request all parameters of this component. After this request, all parameters are emitted.*/
        public class PARAM_REQUEST_LIST : Pack
        {
            internal PARAM_REQUEST_LIST() : base(meta21, 0) { }
            internal PARAM_REQUEST_LIST(int bytes) : base(meta21, bytes) { }
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }
            static readonly Meta meta21 = new Meta(21, 0, 0, 0, 2, 16);
        }/**
*Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows
*	 the recipient to keep track of received parameters and allows him to re-request missing parameters after
*	 a loss or timeout*/
        public class PARAM_VALUE : Pack
        {
            internal PARAM_VALUE() : base(meta22, 0) { }
            internal PARAM_VALUE(int bytes) : base(meta22, bytes) { }
            public ushort param_count //Total number of onboard parameters
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort param_index //Index of this onboard parameter
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public float param_value //Onboard parameter value
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public MAV_PARAM_TYPE param_type //Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
            {
                get {  return (MAV_PARAM_TYPE)(1 +  BitUtils.get_bits(data, 64, 4));}
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	 storage if the ID is stored as strin*/
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  68 && !try_visit_field(ph, 68)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	 storage if the ID is stored as strin*/
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  68 && !try_visit_field(ph, 68)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } static readonly Meta meta22 = new Meta(22, 2, 0, 0, 10, 68, 0, _q);
        }/**
*Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION
*	 MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component
*	 should acknowledge the new parameter value by sending a param_value message to all communication partners.
*	 This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending
*	 GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message*/
        public class PARAM_SET : Pack
        {
            internal PARAM_SET() : base(meta23, 0) { }
            internal PARAM_SET(int bytes) : base(meta23, bytes) { }
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public float param_value //Onboard parameter value
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  2, 4)));}
            }

            public MAV_PARAM_TYPE param_type //Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
            {
                get {  return (MAV_PARAM_TYPE)(1 +  BitUtils.get_bits(data, 48, 4));}
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	 storage if the ID is stored as strin*/
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  52 && !try_visit_field(ph, 52)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	 storage if the ID is stored as strin*/
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  52 && !try_visit_field(ph, 52)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } static readonly Meta meta23 = new Meta(23, 0, 0, 0, 8, 52, 0, _U);
        }/**
*The global position, as returned by the Global Positioning System (GPS). This is
*	 NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
        public class GPS_RAW_INT : Pack
        {
            internal GPS_RAW_INT() : base(meta24, 0) { }
            internal GPS_RAW_INT(int bytes) : base(meta24, bytes) { }
            public ushort eph //GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort epv //GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort vel //GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            /**
            *Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
            *	 unknown, set to: UINT16_MA*/
            public ushort cog
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
            }

            public int lat //Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  16, 4));}
            }

            public int lon //Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
            }

            /**
            *Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide
            *	 the AMSL altitude in addition to the WGS84 altitude*/
            public int alt
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  24, 4));}
            }

            public byte satellites_visible //Number of satellites visible. If unknown, set to 255
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  28, 1));}
            }

            public GPS_FIX_TYPE fix_type //See the GPS_FIX_TYPE enum.
            {
                get {  return (GPS_FIX_TYPE)(0 +  BitUtils.get_bits(data, 232, 4));}
            }
            public int alt_ellipsoid_TRY(Inside ph)//Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up).
            {
                if(ph.field_bit !=  236 && !try_visit_field(ph, 236)) return 0;
                return (int)((int) BitUtils.get_bytes(data,  ph.BYTE, 4));
            }
            public uint h_acc_TRY(Inside ph)//Position uncertainty in meters * 1000 (positive for up).
            {
                if(ph.field_bit !=  237 && !try_visit_field(ph, 237)) return 0;
                return (uint)((uint) BitUtils.get_bytes(data,  ph.BYTE, 4));
            }
            public uint v_acc_TRY(Inside ph)//Altitude uncertainty in meters * 1000 (positive for up).
            {
                if(ph.field_bit !=  238 && !try_visit_field(ph, 238)) return 0;
                return (uint)((uint) BitUtils.get_bytes(data,  ph.BYTE, 4));
            }
            public uint vel_acc_TRY(Inside ph)//Speed uncertainty in meters * 1000 (positive for up).
            {
                if(ph.field_bit !=  239 && !try_visit_field(ph, 239)) return 0;
                return (uint)((uint) BitUtils.get_bytes(data,  ph.BYTE, 4));
            }
            public uint hdg_acc_TRY(Inside ph)//Heading / track uncertainty in degrees * 1e5.
            {
                if(ph.field_bit !=  240 && !try_visit_field(ph, 240)) return 0;
                return (uint)((uint) BitUtils.get_bytes(data,  ph.BYTE, 4));
            }
            static readonly Meta meta24 = new Meta(24, 4, 0, 1, 31, 236, 0, _fs, _ts, _Os, _Is, _us);
        }/**
*The positioning status, as reported by GPS. This message is intended to display status information about
*	 each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate.
*	 This message can contain information for up to 20 satellites*/
        public class GPS_STATUS : Pack
        {
            internal GPS_STATUS() : base(meta25, 0) { }
            internal GPS_STATUS(int bytes) : base(meta25, bytes) { }
            public byte satellites_visible //Number of satellites visible
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte[] satellite_prn //Global satellite ID
            {
                get {return satellite_prn_GET(new byte[20], 0);}
            }
            public byte[]satellite_prn_GET(byte[] dst_ch, int pos)  //Global satellite ID
            {
                for(int BYTE = 1, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public byte[] satellite_used //0: Satellite not used, 1: used for localization
            {
                get {return satellite_used_GET(new byte[20], 0);}
            }
            public byte[]satellite_used_GET(byte[] dst_ch, int pos)  //0: Satellite not used, 1: used for localization
            {
                for(int BYTE = 21, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public byte[] satellite_elevation //Elevation (0: right on top of receiver, 90: on the horizon) of satellite
            {
                get {return satellite_elevation_GET(new byte[20], 0);}
            }
            public byte[]satellite_elevation_GET(byte[] dst_ch, int pos)  //Elevation (0: right on top of receiver, 90: on the horizon) of satellite
            {
                for(int BYTE = 41, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public byte[] satellite_azimuth //Direction of satellite, 0: 0 deg, 255: 360 deg.
            {
                get {return satellite_azimuth_GET(new byte[20], 0);}
            }
            public byte[]satellite_azimuth_GET(byte[] dst_ch, int pos)  //Direction of satellite, 0: 0 deg, 255: 360 deg.
            {
                for(int BYTE = 61, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public byte[] satellite_snr //Signal to noise ratio of satellite
            {
                get {return satellite_snr_GET(new byte[20], 0);}
            }
            public byte[]satellite_snr_GET(byte[] dst_ch, int pos)  //Signal to noise ratio of satellite
            {
                for(int BYTE = 81, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            static readonly Meta meta25 = new Meta(25, 0, 0, 0, 101, 808);
        }/**
*The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to
*	 the described unit*/
        public class SCALED_IMU : Pack
        {
            internal SCALED_IMU() : base(meta26, 0) { }
            internal SCALED_IMU(int bytes) : base(meta26, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public short xacc //X acceleration (mg)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }

            public short yacc //Y acceleration (mg)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  6, 2));}
            }

            public short zacc //Z acceleration (mg)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
            }

            public short xgyro //Angular speed around X axis (millirad /sec)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
            }

            public short ygyro //Angular speed around Y axis (millirad /sec)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
            }

            public short zgyro //Angular speed around Z axis (millirad /sec)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
            }

            public short xmag //X Magnetic field (milli tesla)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  16, 2));}
            }

            public short ymag //Y Magnetic field (milli tesla)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  18, 2));}
            }

            public short zmag //Z Magnetic field (milli tesla)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  20, 2));}
            }
            static readonly Meta meta26 = new Meta(26, 0, 1, 0, 22, 176);
        }/**
*The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw
*	 values without any scaling to allow data capture and system debugging*/
        public class RAW_IMU : Pack
        {
            internal RAW_IMU() : base(meta27, 0) { }
            internal RAW_IMU(int bytes) : base(meta27, bytes) { }
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public short xacc //X acceleration (raw)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
            }

            public short yacc //Y acceleration (raw)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
            }

            public short zacc //Z acceleration (raw)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
            }

            public short xgyro //Angular speed around X axis (raw)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
            }

            public short ygyro //Angular speed around Y axis (raw)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  16, 2));}
            }

            public short zgyro //Angular speed around Z axis (raw)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  18, 2));}
            }

            public short xmag //X Magnetic field (raw)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  20, 2));}
            }

            public short ymag //Y Magnetic field (raw)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  22, 2));}
            }

            public short zmag //Z Magnetic field (raw)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  24, 2));}
            }
            static readonly Meta meta27 = new Meta(27, 0, 0, 1, 26, 208);
        }/**
*The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure
*	 sensor. The sensor values should be the raw, UNSCALED ADC values*/
        public class RAW_PRESSURE : Pack
        {
            internal RAW_PRESSURE() : base(meta28, 0) { }
            internal RAW_PRESSURE(int bytes) : base(meta28, bytes) { }
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public short press_abs //Absolute pressure (raw)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
            }

            public short press_diff1 //Differential pressure 1 (raw, 0 if nonexistant)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
            }

            public short press_diff2 //Differential pressure 2 (raw, 0 if nonexistant)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
            }

            public short temperature //Raw Temperature measurement (raw)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
            }
            static readonly Meta meta28 = new Meta(28, 0, 0, 1, 16, 128);
        }/**
*The pressure readings for the typical setup of one absolute and differential pressure sensor. The units
*	 are as specified in each field*/
        public class SCALED_PRESSURE : Pack
        {
            internal SCALED_PRESSURE() : base(meta29, 0) { }
            internal SCALED_PRESSURE(int bytes) : base(meta29, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public float press_abs //Absolute pressure (hectopascal)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float press_diff //Differential pressure 1 (hectopascal)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public short temperature //Temperature measurement (0.01 degrees celsius)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
            }
            static readonly Meta meta29 = new Meta(29, 0, 1, 0, 14, 112);
        }/**
*The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).*/
        public class ATTITUDE : Pack
        {
            internal ATTITUDE() : base(meta30, 0) { }
            internal ATTITUDE(int bytes) : base(meta30, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public float roll //Roll angle (rad, -pi..+pi)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float pitch //Pitch angle (rad, -pi..+pi)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float yaw //Yaw angle (rad, -pi..+pi)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float rollspeed //Roll angular speed (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float pitchspeed //Pitch angular speed (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float yawspeed //Yaw angular speed (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }
            static readonly Meta meta30 = new Meta(30, 0, 1, 0, 28, 224);
        }/**
*The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
*	 Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)*/
        public class ATTITUDE_QUATERNION : Pack
        {
            internal ATTITUDE_QUATERNION() : base(meta31, 0) { }
            internal ATTITUDE_QUATERNION(int bytes) : base(meta31, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public float q1 //Quaternion component 1, w (1 in null-rotation)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float q2 //Quaternion component 2, x (0 in null-rotation)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float q3 //Quaternion component 3, y (0 in null-rotation)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float q4 //Quaternion component 4, z (0 in null-rotation)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float rollspeed //Roll angular speed (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float pitchspeed //Pitch angular speed (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float yawspeed //Yaw angular speed (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }
            static readonly Meta meta31 = new Meta(31, 0, 1, 0, 32, 256);
        }/**
*The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
*	 Z-axis down (aeronautical frame, NED / north-east-down convention*/
        public class LOCAL_POSITION_NED : Pack
        {
            internal LOCAL_POSITION_NED() : base(meta32, 0) { }
            internal LOCAL_POSITION_NED(int bytes) : base(meta32, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public float x //X Position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float y //Y Position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float z //Z Position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float vx //X Speed
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float vy //Y Speed
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float vz //Z Speed
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }
            static readonly Meta meta32 = new Meta(32, 0, 1, 0, 28, 224);
        }/**
*nt.*/
        public class GLOBAL_POSITION_INT : Pack
        {
            internal GLOBAL_POSITION_INT() : base(meta33, 0) { }
            internal GLOBAL_POSITION_INT(int bytes) : base(meta33, bytes) { }
            public ushort hdg //Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
            }

            public int lat //Latitude, expressed as degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  6, 4));}
            }

            public int lon //Longitude, expressed as degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
            }

            /**
            *Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules
            *	 provide the AMSL as well*/
            public int alt
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  14, 4));}
            }

            public int relative_alt //Altitude above ground in meters, expressed as * 1000 (millimeters)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  18, 4));}
            }

            public short vx //Ground X Speed (Latitude, positive north), expressed as m/s * 100
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  22, 2));}
            }

            public short vy //Ground Y Speed (Longitude, positive east), expressed as m/s * 100
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  24, 2));}
            }

            public short vz //Ground Z Speed (Altitude, positive down), expressed as m/s * 100
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  26, 2));}
            }
            static readonly Meta meta33 = new Meta(33, 1, 1, 0, 28, 224);
        }/**
*The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are
*	 inactive should be set to UINT16_MAX*/
        public class RC_CHANNELS_SCALED : Pack
        {
            internal RC_CHANNELS_SCALED() : base(meta34, 0) { }
            internal RC_CHANNELS_SCALED(int bytes) : base(meta34, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            /**
            *Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
            *	 8 servos*/
            public byte port
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public short chan1_scaled //RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  5, 2));}
            }

            public short chan2_scaled //RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  7, 2));}
            }

            public short chan3_scaled //RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  9, 2));}
            }

            public short chan4_scaled //RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  11, 2));}
            }

            public short chan5_scaled //RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  13, 2));}
            }

            public short chan6_scaled //RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  15, 2));}
            }

            public short chan7_scaled //RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  17, 2));}
            }

            public short chan8_scaled //RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  19, 2));}
            }

            public byte rssi //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  21, 1));}
            }
            static readonly Meta meta34 = new Meta(34, 0, 1, 0, 22, 176);
        }/**
*The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
*	 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification*/
        public class RC_CHANNELS_RAW : Pack
        {
            internal RC_CHANNELS_RAW() : base(meta35, 0) { }
            internal RC_CHANNELS_RAW(int bytes) : base(meta35, bytes) { }
            public ushort chan1_raw //RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort chan2_raw //RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort chan3_raw //RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public ushort chan4_raw //RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
            }

            public ushort chan5_raw //RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  8, 2));}
            }

            public ushort chan6_raw //RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  10, 2));}
            }

            public ushort chan7_raw //RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  12, 2));}
            }

            public ushort chan8_raw //RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  14, 2));}
            }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  16, 4));}
            }

            /**
            *Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
            *	 8 servos*/
            public byte port
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  20, 1));}
            }

            public byte rssi //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  21, 1));}
            }
            static readonly Meta meta35 = new Meta(35, 8, 1, 0, 22, 176);
        }/**
*The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The
*	 standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%*/
        public class SERVO_OUTPUT_RAW : Pack
        {
            internal SERVO_OUTPUT_RAW() : base(meta36, 0) { }
            internal SERVO_OUTPUT_RAW(int bytes) : base(meta36, bytes) { }
            public ushort servo1_raw //Servo output 1 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort servo2_raw //Servo output 2 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort servo3_raw //Servo output 3 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public ushort servo4_raw //Servo output 4 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
            }

            public ushort servo5_raw //Servo output 5 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  8, 2));}
            }

            public ushort servo6_raw //Servo output 6 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  10, 2));}
            }

            public ushort servo7_raw //Servo output 7 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  12, 2));}
            }

            public ushort servo8_raw //Servo output 8 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  14, 2));}
            }

            public uint time_usec //Timestamp (microseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  16, 4));}
            }

            /**
            *Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode
            *	 more than 8 servos*/
            public byte port
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  20, 1));}
            }
            public ushort servo9_raw_TRY(Inside ph)//Servo output 9 value, in microseconds
            {
                if(ph.field_bit !=  168 && !try_visit_field(ph, 168)) return 0;
                return (ushort)((ushort) BitUtils.get_bytes(data,  ph.BYTE, 2));
            }
            public ushort servo10_raw_TRY(Inside ph)//Servo output 10 value, in microseconds
            {
                if(ph.field_bit !=  169 && !try_visit_field(ph, 169)) return 0;
                return (ushort)((ushort) BitUtils.get_bytes(data,  ph.BYTE, 2));
            }
            public ushort servo11_raw_TRY(Inside ph)//Servo output 11 value, in microseconds
            {
                if(ph.field_bit !=  170 && !try_visit_field(ph, 170)) return 0;
                return (ushort)((ushort) BitUtils.get_bytes(data,  ph.BYTE, 2));
            }
            public ushort servo12_raw_TRY(Inside ph)//Servo output 12 value, in microseconds
            {
                if(ph.field_bit !=  171 && !try_visit_field(ph, 171)) return 0;
                return (ushort)((ushort) BitUtils.get_bytes(data,  ph.BYTE, 2));
            }
            public ushort servo13_raw_TRY(Inside ph)//Servo output 13 value, in microseconds
            {
                if(ph.field_bit !=  172 && !try_visit_field(ph, 172)) return 0;
                return (ushort)((ushort) BitUtils.get_bytes(data,  ph.BYTE, 2));
            }
            public ushort servo14_raw_TRY(Inside ph)//Servo output 14 value, in microseconds
            {
                if(ph.field_bit !=  173 && !try_visit_field(ph, 173)) return 0;
                return (ushort)((ushort) BitUtils.get_bytes(data,  ph.BYTE, 2));
            }
            public ushort servo15_raw_TRY(Inside ph)//Servo output 15 value, in microseconds
            {
                if(ph.field_bit !=  174 && !try_visit_field(ph, 174)) return 0;
                return (ushort)((ushort) BitUtils.get_bytes(data,  ph.BYTE, 2));
            }
            public ushort servo16_raw_TRY(Inside ph)//Servo output 16 value, in microseconds
            {
                if(ph.field_bit !=  175 && !try_visit_field(ph, 175)) return 0;
                return (ushort)((ushort) BitUtils.get_bytes(data,  ph.BYTE, 2));
            }
            static readonly Meta meta36 = new Meta(36, 8, 1, 0, 22, 168, 0, _VK, _fK, _tK, _OK, _IK, _uK, _TK, _aK);
        }/**
*Request a partial list of mission items from the system/component. http:qgroundcontrol.org/mavlink/waypoint_protocol.
*	 If start and end index are the same, just send one waypoint*/
        public class MISSION_REQUEST_PARTIAL_LIST : Pack
        {
            internal MISSION_REQUEST_PARTIAL_LIST() : base(meta37, 0) { }
            internal MISSION_REQUEST_PARTIAL_LIST(int bytes) : base(meta37, bytes) { }
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public short start_index //Start index, 0 by default
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }

            public short end_index //End index, -1 by default (-1: send list to end). Else a valid index of the list
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                get {  return  en__X(BitUtils.get_bits(data, 48, 3));}
            }
            static readonly Meta meta37 = new Meta(37, 0, 0, 0, 7, 51);
        }/**
*This message is sent to the MAV to write a partial list. If start index == end index, only one item will
*	 be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should
*	 be REJECTED*/
        public class MISSION_WRITE_PARTIAL_LIST : Pack
        {
            internal MISSION_WRITE_PARTIAL_LIST() : base(meta38, 0) { }
            internal MISSION_WRITE_PARTIAL_LIST(int bytes) : base(meta38, bytes) { }
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public short start_index //Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }

            public short end_index //End index, equal or greater than start index.
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                get {  return  en__X(BitUtils.get_bits(data, 48, 3));}
            }
            static readonly Meta meta38 = new Meta(38, 0, 0, 0, 7, 51);
        }/**
*Message encoding a mission item. This message is emitted to announce
*	 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http:qgroundcontrol.org/mavlink/waypoint_protocol.*/
        public class MISSION_ITEM : Pack
        {
            internal MISSION_ITEM() : base(meta39, 0) { }
            internal MISSION_ITEM(int bytes) : base(meta39, bytes) { }
            public ushort seq //Sequence
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte current //false:0, true:1
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte autocontinue //autocontinue to next wp
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public float param1 //PARAM1, see MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }

            public float param2 //PARAM2, see MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }

            public float param3 //PARAM3, see MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float param4 //PARAM4, see MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public float x //PARAM5 / local: x position, global: latitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
            }

            public float y //PARAM6 / y position: global: longitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
            }

            public float z //PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
            }

            public MAV_FRAME frame //The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 272, 4));}
            }

            public MAV_CMD command //The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
            {
                get {  return  en__m(BitUtils.get_bits(data, 276, 8));}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                get {  return  en__X(BitUtils.get_bits(data, 284, 3));}
            }
            static readonly Meta meta39 = new Meta(39, 1, 0, 0, 36, 287);
        }/**
*Request the information of the mission item with the sequence number seq. The response of the system to
*	 this message should be a MISSION_ITEM message. http:qgroundcontrol.org/mavlink/waypoint_protoco*/
        public class MISSION_REQUEST : Pack
        {
            internal MISSION_REQUEST() : base(meta40, 0) { }
            internal MISSION_REQUEST(int bytes) : base(meta40, bytes) { }
            public ushort seq //Sequence
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                get {  return  en__X(BitUtils.get_bits(data, 32, 3));}
            }
            static readonly Meta meta40 = new Meta(40, 1, 0, 0, 5, 35);
        }/**
*Set the mission item with sequence number seq as current item. This means that the MAV will continue to
*	 this mission item on the shortest path (not following the mission items in-between)*/
        public class MISSION_SET_CURRENT : Pack
        {
            internal MISSION_SET_CURRENT() : base(meta41, 0) { }
            internal MISSION_SET_CURRENT(int bytes) : base(meta41, bytes) { }
            public ushort seq //Sequence
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }
            static readonly Meta meta41 = new Meta(41, 1, 0, 0, 4, 32);
        }/**
*Message that announces the sequence number of the current active mission item. The MAV will fly towards
*	 this mission item*/
        public class MISSION_CURRENT : Pack
        {
            internal MISSION_CURRENT() : base(meta42, 0) { }
            internal MISSION_CURRENT(int bytes) : base(meta42, bytes) { }
            public ushort seq //Sequence
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }
            static readonly Meta meta42 = new Meta(42, 1, 0, 0, 2, 16);
        }/**
*Request the overall list of mission items from the system/component.*/
        public class MISSION_REQUEST_LIST : Pack
        {
            internal MISSION_REQUEST_LIST() : base(meta43, 0) { }
            internal MISSION_REQUEST_LIST(int bytes) : base(meta43, bytes) { }
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                get {  return  en__X(BitUtils.get_bits(data, 16, 3));}
            }
            static readonly Meta meta43 = new Meta(43, 0, 0, 0, 3, 19);
        }/**
*This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction.
*	 The GCS can then request the individual mission item based on the knowledge of the total number of waypoints*/
        public class MISSION_COUNT : Pack
        {
            internal MISSION_COUNT() : base(meta44, 0) { }
            internal MISSION_COUNT(int bytes) : base(meta44, bytes) { }
            public ushort count //Number of mission items in the sequence
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                get {  return  en__X(BitUtils.get_bits(data, 32, 3));}
            }
            static readonly Meta meta44 = new Meta(44, 1, 0, 0, 5, 35);
        }/**
*Delete all mission items at once.*/
        public class MISSION_CLEAR_ALL : Pack
        {
            internal MISSION_CLEAR_ALL() : base(meta45, 0) { }
            internal MISSION_CLEAR_ALL(int bytes) : base(meta45, bytes) { }
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                get {  return  en__X(BitUtils.get_bits(data, 16, 3));}
            }
            static readonly Meta meta45 = new Meta(45, 0, 0, 0, 3, 19);
        }/**
*A certain mission item has been reached. The system will either hold this position (or circle on the orbit)
*	 or (if the autocontinue on the WP was set) continue to the next waypoint*/
        public class MISSION_ITEM_REACHED : Pack
        {
            internal MISSION_ITEM_REACHED() : base(meta46, 0) { }
            internal MISSION_ITEM_REACHED(int bytes) : base(meta46, bytes) { }
            public ushort seq //Sequence
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }
            static readonly Meta meta46 = new Meta(46, 1, 0, 0, 2, 16);
        }/**
*Ack message during waypoint handling. The type field states if this message is a positive ack (type=0)
*	 or if an error happened (type=non-zero)*/
        public class MISSION_ACK : Pack
        {
            internal MISSION_ACK() : base(meta47, 0) { }
            internal MISSION_ACK(int bytes) : base(meta47, bytes) { }
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public MAV_MISSION_RESULT type //See MAV_MISSION_RESULT enum
            {
                get {  return (MAV_MISSION_RESULT)(0 +  BitUtils.get_bits(data, 16, 4));}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                get {  return  en__X(BitUtils.get_bits(data, 20, 3));}
            }
            static readonly Meta meta47 = new Meta(47, 0, 0, 0, 3, 23);
        }/**
*As local waypoints exist, the global waypoint reference allows to transform between the local coordinate
*	 frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings
*	 are connected and the MAV should move from in- to outdoor*/
        public class SET_GPS_GLOBAL_ORIGIN : Pack
        {
            internal SET_GPS_GLOBAL_ORIGIN() : base(meta48, 0) { }
            internal SET_GPS_GLOBAL_ORIGIN(int bytes) : base(meta48, bytes) { }
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public int latitude //Latitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  1, 4));}
            }

            public int longitude //Longitude (WGS84, in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  5, 4));}
            }

            public int altitude //Altitude (AMSL), in meters * 1000 (positive for up)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  9, 4));}
            }
            public ulong time_usec_TRY(Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                if(ph.field_bit !=  104 && !try_visit_field(ph, 104)) return 0;
                return (BitUtils.get_bytes(data,  ph.BYTE, 8));
            }
            static readonly Meta meta48 = new Meta(48, 0, 0, 0, 14, 104, 0, _gJ);
        }/**
*Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) positio*/
        public class GPS_GLOBAL_ORIGIN : Pack
        {
            internal GPS_GLOBAL_ORIGIN() : base(meta49, 0) { }
            internal GPS_GLOBAL_ORIGIN(int bytes) : base(meta49, bytes) { }
            public int latitude //Latitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  0, 4));}
            }

            public int longitude //Longitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  4, 4));}
            }

            public int altitude //Altitude (AMSL), in meters * 1000 (positive for up)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  8, 4));}
            }
            public ulong time_usec_TRY(Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                if(ph.field_bit !=  96 && !try_visit_field(ph, 96)) return 0;
                return (BitUtils.get_bytes(data,  ph.BYTE, 8));
            }
            static readonly Meta meta49 = new Meta(49, 0, 0, 0, 13, 96, 0, _xJ);
        }/**
*Bind a RC channel to a parameter. The parameter should change accoding to the RC channel value.*/
        public class PARAM_MAP_RC : Pack
        {
            internal PARAM_MAP_RC() : base(meta50, 0) { }
            internal PARAM_MAP_RC(int bytes) : base(meta50, bytes) { }
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            /**
            *Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored),
            *	 send -2 to disable any existing map for this rc_channel_index*/
            public short param_index
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }

            /**
            *Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob
            *	 on the RC*/
            public byte parameter_rc_channel_index
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public float param_value0 //Initial parameter value
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  5, 4)));}
            }

            public float scale //Scale, maps the RC range [-1, 1] to a parameter value
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  9, 4)));}
            }

            /**
            *Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends
            *	 on implementation*/
            public float param_value_min
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  13, 4)));}
            }

            /**
            *Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends
            *	 on implementation*/
            public float param_value_max
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	 storage if the ID is stored as strin*/
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  168 && !try_visit_field(ph, 168)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	 storage if the ID is stored as strin*/
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  168 && !try_visit_field(ph, 168)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } static readonly Meta meta50 = new Meta(50, 0, 0, 0, 22, 168, 0, _yJ);
        }/**
*Request the information of the mission item with the sequence number seq. The response of the system to
*	 this message should be a MISSION_ITEM_INT message. http:qgroundcontrol.org/mavlink/waypoint_protoco*/
        public class MISSION_REQUEST_INT : Pack
        {
            internal MISSION_REQUEST_INT() : base(meta51, 0) { }
            internal MISSION_REQUEST_INT(int bytes) : base(meta51, bytes) { }
            public ushort seq //Sequence
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                get {  return  en__X(BitUtils.get_bits(data, 32, 3));}
            }
            static readonly Meta meta51 = new Meta(51, 1, 0, 0, 5, 35);
        }/**
*Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell
*	 the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national
*	 or competition regulations*/
        public class SAFETY_SET_ALLOWED_AREA : Pack
        {
            internal SAFETY_SET_ALLOWED_AREA() : base(meta54, 0) { }
            internal SAFETY_SET_ALLOWED_AREA(int bytes) : base(meta54, bytes) { }
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public float p1x //x position 1 / Latitude 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  2, 4)));}
            }

            public float p1y //y position 1 / Longitude 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }

            public float p1z //z position 1 / Altitude 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }

            public float p2x //x position 2 / Latitude 2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float p2y //y position 2 / Longitude 2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public float p2z //z position 2 / Altitude 2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
            }

            /**
            *Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
            *	 with Z axis up or local, right handed, Z axis down*/
            public MAV_FRAME frame
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 208, 4));}
            }
            static readonly Meta meta54 = new Meta(54, 0, 0, 0, 27, 212);
        }/**
*Read out the safety zone the MAV currently assumes.*/
        public class SAFETY_ALLOWED_AREA : Pack
        {
            internal SAFETY_ALLOWED_AREA() : base(meta55, 0) { }
            internal SAFETY_ALLOWED_AREA(int bytes) : base(meta55, bytes) { }
            public float p1x //x position 1 / Latitude 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float p1y //y position 1 / Longitude 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float p1z //z position 1 / Altitude 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float p2x //x position 2 / Latitude 2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float p2y //y position 2 / Longitude 2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float p2z //z position 2 / Altitude 2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            /**
            *Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
            *	 with Z axis up or local, right handed, Z axis down*/
            public MAV_FRAME frame
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 192, 4));}
            }
            static readonly Meta meta55 = new Meta(55, 0, 0, 0, 25, 196);
        }/**
*The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
*	 Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)*/
        public class ATTITUDE_QUATERNION_COV : Pack
        {
            internal ATTITUDE_QUATERNION_COV() : base(meta61, 0) { }
            internal ATTITUDE_QUATERNION_COV(int bytes) : base(meta61, bytes) { }
            public ulong time_usec //Timestamp (microseconds since system boot or since UNIX epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float[] q //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
            {
                get {return q_GET(new float[4], 0);}
            }
            public float[]q_GET(float[] dst_ch, int pos)  //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
            {
                for(int BYTE = 8, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public float rollspeed //Roll angular speed (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float pitchspeed //Pitch angular speed (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float yawspeed //Yaw angular speed (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float[] covariance //Attitude covariance
            {
                get {return covariance_GET(new float[9], 0);}
            }
            public float[]covariance_GET(float[] dst_ch, int pos)  //Attitude covariance
            {
                for(int BYTE = 36, dst_max = pos + 9; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            static readonly Meta meta61 = new Meta(61, 0, 0, 1, 72, 576);
        }/**
*The state of the fixed wing navigation and position controller.*/
        public class NAV_CONTROLLER_OUTPUT : Pack
        {
            internal NAV_CONTROLLER_OUTPUT() : base(meta62, 0) { }
            internal NAV_CONTROLLER_OUTPUT(int bytes) : base(meta62, bytes) { }
            public ushort wp_dist //Distance to active waypoint in meters
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public float nav_roll //Current desired roll in degrees
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  2, 4)));}
            }

            public float nav_pitch //Current desired pitch in degrees
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }

            public short nav_bearing //Current desired heading in degrees
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
            }

            public short target_bearing //Bearing to current waypoint/target in degrees
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
            }

            public float alt_error //Current altitude error in meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float aspd_error //Current airspeed error in meters/second
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public float xtrack_error //Current crosstrack error on x-y plane in meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
            }
            static readonly Meta meta62 = new Meta(62, 1, 0, 0, 26, 208);
        }/**
*The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed,
*	 Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE:
*	 This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized
*	 for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset*/
        public class GLOBAL_POSITION_INT_COV : Pack
        {
            internal GLOBAL_POSITION_INT_COV() : base(meta63, 0) { }
            internal GLOBAL_POSITION_INT_COV(int bytes) : base(meta63, bytes) { }
            public ulong time_usec //Timestamp (microseconds since system boot or since UNIX epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public int lat //Latitude, expressed as degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  8, 4));}
            }

            public int lon //Longitude, expressed as degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  12, 4));}
            }

            public int alt //Altitude in meters, expressed as * 1000 (millimeters), above MSL
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  16, 4));}
            }

            public int relative_alt //Altitude above ground in meters, expressed as * 1000 (millimeters)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
            }

            public float vx //Ground X Speed (Latitude), expressed as m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float vy //Ground Y Speed (Longitude), expressed as m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float vz //Ground Z Speed (Altitude), expressed as m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float[] covariance //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
            {
                get {return covariance_GET(new float[36], 0);}
            }
            public float[]covariance_GET(float[] dst_ch, int pos)  //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
            {
                for(int BYTE = 36, dst_max = pos + 36; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public MAV_ESTIMATOR_TYPE estimator_type //Class id of the estimator this estimate originated from.
            {
                get {  return (MAV_ESTIMATOR_TYPE)(1 +  BitUtils.get_bits(data, 1440, 3));}
            }
            static readonly Meta meta63 = new Meta(63, 0, 0, 1, 181, 1443);
        }/**
*The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
*	 Z-axis down (aeronautical frame, NED / north-east-down convention*/
        public class LOCAL_POSITION_NED_COV : Pack
        {
            internal LOCAL_POSITION_NED_COV() : base(meta64, 0) { }
            internal LOCAL_POSITION_NED_COV(int bytes) : base(meta64, bytes) { }
            public ulong time_usec //Timestamp (microseconds since system boot or since UNIX epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float x //X Position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float y //Y Position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float z //Z Position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float vx //X Speed (m/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float vy //Y Speed (m/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float vz //Z Speed (m/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float ax //X Acceleration (m/s^2)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float ay //Y Acceleration (m/s^2)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public float az //Z Acceleration (m/s^2)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
            }

            /**
            *Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
            *	 the second row, etc.*/
            public float[] covariance
            {
                get {return covariance_GET(new float[45], 0);}
            }
            /**
            *Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
            *	 the second row, etc.*/
            public float[]covariance_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 44, dst_max = pos + 45; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public MAV_ESTIMATOR_TYPE estimator_type //Class id of the estimator this estimate originated from.
            {
                get {  return (MAV_ESTIMATOR_TYPE)(1 +  BitUtils.get_bits(data, 1792, 3));}
            }
            static readonly Meta meta64 = new Meta(64, 0, 0, 1, 225, 1795);
        }/**
*The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
*	 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification*/
        public class RC_CHANNELS : Pack
        {
            internal RC_CHANNELS() : base(meta65, 0) { }
            internal RC_CHANNELS(int bytes) : base(meta65, bytes) { }
            public ushort chan1_raw //RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort chan2_raw //RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort chan3_raw //RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public ushort chan4_raw //RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
            }

            public ushort chan5_raw //RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  8, 2));}
            }

            public ushort chan6_raw //RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  10, 2));}
            }

            public ushort chan7_raw //RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  12, 2));}
            }

            public ushort chan8_raw //RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  14, 2));}
            }

            public ushort chan9_raw //RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  16, 2));}
            }

            public ushort chan10_raw //RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  18, 2));}
            }

            public ushort chan11_raw //RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  20, 2));}
            }

            public ushort chan12_raw //RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  22, 2));}
            }

            public ushort chan13_raw //RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  24, 2));}
            }

            public ushort chan14_raw //RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  26, 2));}
            }

            public ushort chan15_raw //RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  28, 2));}
            }

            public ushort chan16_raw //RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  30, 2));}
            }

            public ushort chan17_raw //RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  32, 2));}
            }

            public ushort chan18_raw //RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  34, 2));}
            }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  36, 4));}
            }

            /**
            *Total number of RC channels being received. This can be larger than 18, indicating that more channels
            *	 are available but not given in this message. This value should be 0 when no RC channels are available*/
            public byte chancount
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  40, 1));}
            }

            public byte rssi //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  41, 1));}
            }
            static readonly Meta meta65 = new Meta(65, 18, 1, 0, 42, 336);
        }/**
*THIS INTERFACE IS DEPRECATED. USE SET_MESSAGE_INTERVAL INSTEAD.*/
        public class REQUEST_DATA_STREAM : Pack
        {
            internal REQUEST_DATA_STREAM() : base(meta66, 0) { }
            internal REQUEST_DATA_STREAM(int bytes) : base(meta66, bytes) { }
            public ushort req_message_rate //The requested message rate
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target_system //The target requested to send the message stream.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte target_component //The target requested to send the message stream.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte req_stream_id //The ID of the requested data stream
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte start_stop //1 to start sending, 0 to stop sending.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }
            static readonly Meta meta66 = new Meta(66, 1, 0, 0, 6, 48);
        }/**
*THIS INTERFACE IS DEPRECATED. USE MESSAGE_INTERVAL INSTEAD.*/
        public class DATA_STREAM : Pack
        {
            internal DATA_STREAM() : base(meta67, 0) { }
            internal DATA_STREAM(int bytes) : base(meta67, bytes) { }
            public ushort message_rate //The message rate
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte stream_id //The ID of the requested data stream
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte on_off //1 stream is enabled, 0 stream is stopped.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }
            static readonly Meta meta67 = new Meta(67, 1, 0, 0, 4, 32);
        }/**
*This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature,
*	 along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as
*	 boolean values of their*/
        public class MANUAL_CONTROL : Pack
        {
            internal MANUAL_CONTROL() : base(meta69, 0) { }
            internal MANUAL_CONTROL(int bytes) : base(meta69, bytes) { }
            /**
            *A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest
            *	 bit corresponds to Button 1*/
            public ushort buttons
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target //The system to be controlled.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            /**
            *X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            *	 Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle*/
            public short x
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  3, 2));}
            }

            /**
            *Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            *	 Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle*/
            public short y
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  5, 2));}
            }

            /**
            *Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            *	 Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on
            *	 a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
            *	 thrust*/
            public short z
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  7, 2));}
            }

            /**
            *R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            *	 Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise
            *	 being -1000, and the yaw of a vehicle*/
            public short r
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  9, 2));}
            }
            static readonly Meta meta69 = new Meta(69, 1, 0, 0, 11, 88);
        }/**
*The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value
*	 of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released
*	 back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds:
*	 100%. Individual receivers/transmitters might violate this specification*/
        public class RC_CHANNELS_OVERRIDE : Pack
        {
            internal RC_CHANNELS_OVERRIDE() : base(meta70, 0) { }
            internal RC_CHANNELS_OVERRIDE(int bytes) : base(meta70, bytes) { }
            public ushort chan1_raw //RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort chan2_raw //RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort chan3_raw //RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public ushort chan4_raw //RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
            }

            public ushort chan5_raw //RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  8, 2));}
            }

            public ushort chan6_raw //RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  10, 2));}
            }

            public ushort chan7_raw //RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  12, 2));}
            }

            public ushort chan8_raw //RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  14, 2));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  17, 1));}
            }
            static readonly Meta meta70 = new Meta(70, 8, 0, 0, 18, 144);
        }/**
*Message encoding a mission item. This message is emitted to announce
*	 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See alsohttp:qgroundcontrol.org/mavlink/waypoint_protocol.*/
        public class MISSION_ITEM_INT : Pack
        {
            internal MISSION_ITEM_INT() : base(meta73, 0) { }
            internal MISSION_ITEM_INT(int bytes) : base(meta73, bytes) { }
            /**
            *Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the
            *	 sequence (0,1,2,3,4)*/
            public ushort seq
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte current //false:0, true:1
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte autocontinue //autocontinue to next wp
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public float param1 //PARAM1, see MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }

            public float param2 //PARAM2, see MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }

            public float param3 //PARAM3, see MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float param4 //PARAM4, see MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public int x //PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  22, 4));}
            }

            public int y //PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  26, 4));}
            }

            public float z //PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
            }

            public MAV_FRAME frame //The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 272, 4));}
            }

            public MAV_CMD command //The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
            {
                get {  return  en__m(BitUtils.get_bits(data, 276, 8));}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                get {  return  en__X(BitUtils.get_bits(data, 284, 3));}
            }
            static readonly Meta meta73 = new Meta(73, 1, 0, 0, 36, 287);
        }/**
*Metrics typically displayed on a HUD for fixed wing aircraft*/
        public class VFR_HUD : Pack
        {
            internal VFR_HUD() : base(meta74, 0) { }
            internal VFR_HUD(int bytes) : base(meta74, bytes) { }
            public ushort throttle //Current throttle setting in integer percent, 0 to 100
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public float airspeed //Current airspeed in m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  2, 4)));}
            }

            public float groundspeed //Current ground speed in m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }

            public short heading //Current heading in degrees, in compass units (0..360, 0=north)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
            }

            public float alt //Current altitude (MSL), in meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float climb //Current climb rate in meters/second
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }
            static readonly Meta meta74 = new Meta(74, 1, 0, 0, 20, 160);
        }/**
*Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value*/
        public class COMMAND_INT : Pack
        {
            internal COMMAND_INT() : base(meta75, 0) { }
            internal COMMAND_INT(int bytes) : base(meta75, bytes) { }
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte current //false:0, true:1
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte autocontinue //autocontinue to next wp
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public float param1 //PARAM1, see MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float param2 //PARAM2, see MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float param3 //PARAM3, see MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float param4 //PARAM4, see MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public int x //PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
            }

            public int y //PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  24, 4));}
            }

            public float z //PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public MAV_FRAME frame //The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 256, 4));}
            }

            public MAV_CMD command //The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
            {
                get {  return  en__m(BitUtils.get_bits(data, 260, 8));}
            }
            static readonly Meta meta75 = new Meta(75, 0, 0, 0, 34, 268);
        }/**
*Send a command with up to seven parameters to the MAV*/
        public class COMMAND_LONG : Pack
        {
            internal COMMAND_LONG() : base(meta76, 0) { }
            internal COMMAND_LONG(int bytes) : base(meta76, bytes) { }
            public byte target_system //System which should execute the command
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component which should execute the command, 0 for all components
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte confirmation //0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public float param1 //Parameter 1, as defined by MAV_CMD enum.
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  3, 4)));}
            }

            public float param2 //Parameter 2, as defined by MAV_CMD enum.
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  7, 4)));}
            }

            public float param3 //Parameter 3, as defined by MAV_CMD enum.
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  11, 4)));}
            }

            public float param4 //Parameter 4, as defined by MAV_CMD enum.
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  15, 4)));}
            }

            public float param5 //Parameter 5, as defined by MAV_CMD enum.
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  19, 4)));}
            }

            public float param6 //Parameter 6, as defined by MAV_CMD enum.
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  23, 4)));}
            }

            public float param7 //Parameter 7, as defined by MAV_CMD enum.
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  27, 4)));}
            }

            public MAV_CMD command //Command ID, as defined by MAV_CMD enum.
            {
                get {  return  en__m(BitUtils.get_bits(data, 248, 8));}
            }
            static readonly Meta meta76 = new Meta(76, 0, 0, 0, 32, 256);
        }/**
*Report status of a command. Includes feedback whether the command was executed.*/
        public class COMMAND_ACK : Pack
        {
            internal COMMAND_ACK() : base(meta77, 0) { }
            internal COMMAND_ACK(int bytes) : base(meta77, bytes) { }
            public MAV_CMD command //Command ID, as defined by MAV_CMD enum.
            {
                get {  return  en__m(BitUtils.get_bits(data, 0, 8));}
            }

            public MAV_RESULT result //See MAV_RESULT enum
            {
                get {  return (MAV_RESULT)(0 +  BitUtils.get_bits(data, 8, 3));}
            }
            /**
            *WIP: Also used as result_param1, it can be set with a enum containing the errors reasons of why the command
            *	 was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS*/
            public byte progress_TRY(Inside ph)
            {
                if(ph.field_bit !=  11 && !try_visit_field(ph, 11)) return 0;
                return (byte)((byte) BitUtils.get_bytes(data,  ph.BYTE, 1));
            }
            /**
            *WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to
            *	 be denied*/
            public int result_param2_TRY(Inside ph)
            {
                if(ph.field_bit !=  12 && !try_visit_field(ph, 12)) return 0;
                return (int)((int) BitUtils.get_bytes(data,  ph.BYTE, 4));
            }
            public byte target_system_TRY(Inside ph)//WIP: System which requested the command to be executed
            {
                if(ph.field_bit !=  13 && !try_visit_field(ph, 13)) return 0;
                return (byte)((byte) BitUtils.get_bytes(data,  ph.BYTE, 1));
            }
            public byte target_component_TRY(Inside ph)//WIP: Component which requested the command to be executed
            {
                if(ph.field_bit !=  14 && !try_visit_field(ph, 14)) return 0;
                return (byte)((byte) BitUtils.get_bytes(data,  ph.BYTE, 1));
            }
            static readonly Meta meta77 = new Meta(77, 0, 0, 0, 3, 11, 0, _Rj, _yj, _bj, _Ej);
        }/**
*Setpoint in roll, pitch, yaw and thrust from the operator*/
        public class MANUAL_SETPOINT : Pack
        {
            internal MANUAL_SETPOINT() : base(meta81, 0) { }
            internal MANUAL_SETPOINT(int bytes) : base(meta81, bytes) { }
            public uint time_boot_ms //Timestamp in milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public float roll //Desired roll rate in radians per second
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float pitch //Desired pitch rate in radians per second
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float yaw //Desired yaw rate in radians per second
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float thrust //Collective thrust, normalized to 0 .. 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public byte mode_switch //Flight mode switch position, 0.. 255
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  20, 1));}
            }

            public byte manual_override_switch //Override mode switch position, 0.. 255
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  21, 1));}
            }
            static readonly Meta meta81 = new Meta(81, 0, 1, 0, 22, 176);
        } public class SET_ATTITUDE_TARGET : Pack, CommunicationChannel.Sendable
        {
            internal SET_ATTITUDE_TARGET() : base(meta82, 0) { }
            internal SET_ATTITUDE_TARGET(int bytes) : base(meta82, bytes) { }
            public uint time_boot_ms //Timestamp in milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            /**
            *Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
            *	 bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitud*/
            public byte type_mask
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public float[] q //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                get {return q_GET(new float[4], 0);}
                set {q_SET(value, 0)  ;}
            }
            public float[]q_GET(float[] dst_ch, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                for(int BYTE = 7, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public void q_SET(float[] src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                for(int BYTE =  7, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            public float body_roll_rate //Body roll rate in radians per second
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  23, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 23);}
            }

            public float body_pitch_rate //Body roll rate in radians per second
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  27, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 27);}
            }

            public float body_yaw_rate //Body roll rate in radians per second
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  31, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 31);}
            }

            public float thrust //Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  35, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 35);}
            }
            static readonly Meta meta82 = new Meta(82, 0, 1, 0, 39, 312);
        }/**
*Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match
*	 the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way*/
        public class ATTITUDE_TARGET : Pack, CommunicationChannel.Sendable
        {
            internal ATTITUDE_TARGET() : base(meta83, 0) { }
            internal ATTITUDE_TARGET(int bytes) : base(meta83, bytes) { }
            public uint time_boot_ms //Timestamp in milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            /**
            *Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
            *	 bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitud*/
            public byte type_mask
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public float[] q //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                get {return q_GET(new float[4], 0);}
                set {q_SET(value, 0)  ;}
            }
            public float[]q_GET(float[] dst_ch, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                for(int BYTE = 5, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public void q_SET(float[] src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                for(int BYTE =  5, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            public float body_roll_rate //Body roll rate in radians per second
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  21, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 21);}
            }

            public float body_pitch_rate //Body pitch rate in radians per second
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  25, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 25);}
            }

            public float body_yaw_rate //Body yaw rate in radians per second
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  29, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 29);}
            }

            public float thrust //Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  33, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 33);}
            }
            static readonly Meta meta83 = new Meta(83, 0, 1, 0, 37, 296);
        }/**
*Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller
*	 to command the vehicle (manual controller or other system)*/
        public class SET_POSITION_TARGET_LOCAL_NED : Pack, CommunicationChannel.Sendable
        {
            internal SET_POSITION_TARGET_LOCAL_NED() : base(meta84, 0) { }
            internal SET_POSITION_TARGET_LOCAL_NED(int bytes) : base(meta84, bytes) { }
            /**
            *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
            *	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
            *	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
            *	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
            *	 bit 11: yaw, bit 12: yaw rat*/
            public ushort type_mask
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_boot_ms //Timestamp in milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  7);}
            }

            public float x //X Position in NED frame in meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Y Position in NED frame in meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Z Position in NED frame in meters (note, altitude is negative in NED)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float vx //X velocity in NED frame in meter / s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float vy //Y velocity in NED frame in meter / s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float vz //Z velocity in NED frame in meter / s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float afx //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float afy //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float afz //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float yaw //yaw setpoint in rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float yaw_rate //yaw rate setpoint in rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  48, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }

            /**
            *Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
            *	 =*/
            public MAV_FRAME coordinate_frame
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 416, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 416);}
            }
            static readonly Meta meta84 = new Meta(84, 1, 1, 0, 53, 420);
        }/**
*Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84).
*	 Used by an external controller to command the vehicle (manual controller or other system)*/
        public class SET_POSITION_TARGET_GLOBAL_INT : Pack, CommunicationChannel.Sendable
        {
            internal SET_POSITION_TARGET_GLOBAL_INT() : base(meta86, 0) { }
            internal SET_POSITION_TARGET_GLOBAL_INT(int bytes) : base(meta86, bytes) { }
            /**
            *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
            *	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
            *	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
            *	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
            *	 bit 11: yaw, bit 12: yaw rat*/
            public ushort type_mask
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            /**
            *Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
            *	 the system to compensate for the transport delay of the setpoint. This allows the system to compensate
            *	 processing latency*/
            public uint time_boot_ms
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  7);}
            }

            public int lat_int //X Position in WGS84 frame in 1e7 * meters
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  8, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  8);}
            }

            public int lon_int //Y Position in WGS84 frame in 1e7 * meters
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  12, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  12);}
            }

            public float alt //Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float vx //X velocity in NED frame in meter / s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float vy //Y velocity in NED frame in meter / s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float vz //Z velocity in NED frame in meter / s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float afx //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float afy //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float afz //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float yaw //yaw setpoint in rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float yaw_rate //yaw rate setpoint in rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  48, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }

            /**
            *Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
            *	 = 1*/
            public MAV_FRAME coordinate_frame
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 416, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 416);}
            }
            static readonly Meta meta86 = new Meta(86, 1, 1, 0, 53, 420);
        }/**
*Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
*	 This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled
*	 this way*/
        public class POSITION_TARGET_GLOBAL_INT : Pack, CommunicationChannel.Sendable
        {
            internal POSITION_TARGET_GLOBAL_INT() : base(meta87, 0) { }
            internal POSITION_TARGET_GLOBAL_INT(int bytes) : base(meta87, bytes) { }
            /**
            *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
            *	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
            *	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
            *	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
            *	 bit 11: yaw, bit 12: yaw rat*/
            public ushort type_mask
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            /**
            *Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
            *	 the system to compensate for the transport delay of the setpoint. This allows the system to compensate
            *	 processing latency*/
            public uint time_boot_ms
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public int lat_int //X Position in WGS84 frame in 1e7 * meters
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  6);}
            }

            public int lon_int //Y Position in WGS84 frame in 1e7 * meters
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  10);}
            }

            public float alt //Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float vx //X velocity in NED frame in meter / s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float vy //Y velocity in NED frame in meter / s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            public float vz //Z velocity in NED frame in meter / s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 26);}
            }

            public float afx //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public float afy //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  34, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 34);}
            }

            public float afz //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  38, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 38);}
            }

            public float yaw //yaw setpoint in rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  42, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 42);}
            }

            public float yaw_rate //yaw rate setpoint in rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  46, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 46);}
            }

            /**
            *Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
            *	 = 1*/
            public MAV_FRAME coordinate_frame
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 400, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 400);}
            }
            static readonly Meta meta87 = new Meta(87, 1, 1, 0, 51, 404);
        }/**
*The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate
*	 frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down
*	 convention*/
        public class LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET : Pack, CommunicationChannel.Sendable
        {
            internal LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET() : base(meta89, 0) { }
            internal LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(int bytes) : base(meta89, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float x //X Position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float y //Y Position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float z //Z Position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float roll //Roll
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float pitch //Pitch
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float yaw //Yaw
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }
            static readonly Meta meta89 = new Meta(89, 0, 1, 0, 28, 224);
        }/**
*DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please
*	 use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput
*	 applications such as hardware in the loop simulations*/
        public class HIL_STATE : Pack, CommunicationChannel.Sendable
        {
            internal HIL_STATE() : base(meta90, 0) { }
            internal HIL_STATE(int bytes) : base(meta90, bytes) { }
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float roll //Roll angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float pitch //Pitch angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float yaw //Yaw angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float rollspeed //Body frame roll / phi angular speed (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float pitchspeed //Body frame pitch / theta angular speed (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float yawspeed //Body frame yaw / psi angular speed (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public int lat //Latitude, expressed as * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  32, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  32);}
            }

            public int lon //Longitude, expressed as * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  36, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  36);}
            }

            public int alt //Altitude in meters, expressed as * 1000 (millimeters)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  40, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  40);}
            }

            public short vx //Ground X Speed (Latitude), expressed as m/s * 100
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  44, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  44);}
            }

            public short vy //Ground Y Speed (Longitude), expressed as m/s * 100
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  46, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  46);}
            }

            public short vz //Ground Z Speed (Altitude), expressed as m/s * 100
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  48, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  48);}
            }

            public short xacc //X acceleration (mg)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  50, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  50);}
            }

            public short yacc //Y acceleration (mg)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  52, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  52);}
            }

            public short zacc //Z acceleration (mg)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  54, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  54);}
            }
            static readonly Meta meta90 = new Meta(90, 0, 0, 1, 56, 448);
        }/**
*Sent from autopilot to simulation. Hardware in the loop control outputs*/
        public class HIL_CONTROLS : Pack, CommunicationChannel.Sendable
        {
            internal HIL_CONTROLS() : base(meta91, 0) { }
            internal HIL_CONTROLS(int bytes) : base(meta91, bytes) { }
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float roll_ailerons //Control output -1 .. 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float pitch_elevator //Control output -1 .. 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float yaw_rudder //Control output -1 .. 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float throttle //Throttle 0 .. 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float aux1 //Aux 1, -1 .. 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float aux2 //Aux 2, -1 .. 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float aux3 //Aux 3, -1 .. 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float aux4 //Aux 4, -1 .. 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public byte nav_mode //Navigation mode (MAV_NAV_MODE)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  40, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  40);}
            }

            public MAV_MODE mode //System mode (MAV_MODE)
            {
                get {  return  en__j(BitUtils.get_bits(data, 328, 4));}
                set
                {
                    ulong id = id__j(value);
                    BitUtils.set_bits(id, 4, data, 328);
                }
            }
            static readonly Meta meta91 = new Meta(91, 0, 0, 1, 42, 332);
        }/**
*Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation
*	 is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might
*	 violate this specification*/
        public class HIL_RC_INPUTS_RAW : Pack, CommunicationChannel.Sendable
        {
            internal HIL_RC_INPUTS_RAW() : base(meta92, 0) { }
            internal HIL_RC_INPUTS_RAW(int bytes) : base(meta92, bytes) { }
            public ushort chan1_raw //RC channel 1 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort chan2_raw //RC channel 2 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort chan3_raw //RC channel 3 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort chan4_raw //RC channel 4 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort chan5_raw //RC channel 5 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  8, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort chan6_raw //RC channel 6 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort chan7_raw //RC channel 7 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort chan8_raw //RC channel 8 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  14, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public ushort chan9_raw //RC channel 9 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  16, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  16);}
            }

            public ushort chan10_raw //RC channel 10 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  18, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  18);}
            }

            public ushort chan11_raw //RC channel 11 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  20, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  20);}
            }

            public ushort chan12_raw //RC channel 12 value, in microseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  22, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  22);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  24, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  24);}
            }

            public byte rssi //Receive signal strength indicator, 0: 0%, 255: 100%
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  32, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  32);}
            }
            static readonly Meta meta92 = new Meta(92, 12, 0, 1, 33, 264);
        }/**
*Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS*/
        public class HIL_ACTUATOR_CONTROLS : Pack, CommunicationChannel.Sendable
        {
            internal HIL_ACTUATOR_CONTROLS() : base(meta93, 0) { }
            internal HIL_ACTUATOR_CONTROLS(int bytes) : base(meta93, bytes) { }
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public ulong flags //Flags as bitfield, reserved for future use.
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public float[] controls //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
            {
                get {return controls_GET(new float[16], 0);}
                set {controls_SET(value, 0)  ;}
            }
            public float[]controls_GET(float[] dst_ch, int pos)  //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
            {
                for(int BYTE = 16, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public void controls_SET(float[] src, int pos)  //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
            {
                for(int BYTE =  16, src_max = pos + 16; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            public MAV_MODE mode //System mode (MAV_MODE), includes arming state.
            {
                get {  return  en__j(BitUtils.get_bits(data, 640, 4));}
                set
                {
                    ulong id = id__j(value);
                    BitUtils.set_bits(id, 4, data, 640);
                }
            }
            static readonly Meta meta93 = new Meta(93, 0, 0, 2, 81, 644);
        }/**
*Optical flow from a flow sensor (e.g. optical mouse sensor)*/
        public class OPTICAL_FLOW : Pack, CommunicationChannel.Sendable
        {
            internal OPTICAL_FLOW() : base(meta100, 0) { }
            internal OPTICAL_FLOW(int bytes) : base(meta100, bytes) { }
            public ulong time_usec //Timestamp (UNIX)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public byte sensor_id //Sensor ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }

            public short flow_x //Flow in pixels * 10 in x-sensor direction (dezi-pixels)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  9, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  9);}
            }

            public short flow_y //Flow in pixels * 10 in y-sensor direction (dezi-pixels)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  11, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  11);}
            }

            public float flow_comp_m_x //Flow in meters in x-sensor direction, angular-speed compensated
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  13, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 13);}
            }

            public float flow_comp_m_y //Flow in meters in y-sensor direction, angular-speed compensated
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }

            public byte quality //Optical flow quality / confidence. 0: bad, 255: maximum quality
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  21, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  21);}
            }

            public float ground_distance //Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }
            public float flow_rate_x_TRY(Inside ph)//Flow rate in radians/second about X axis
            {
                if(ph.field_bit !=  208 && !try_visit_field(ph, 208)) return 0;
                return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  ph.BYTE, 4)));
            }
            public void flow_rate_x_SET(float src, Inside ph)//Flow rate in radians/second about X axis
            {
                if(ph.field_bit != 208)insert_field(ph, 208, 0);
                BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src), 4, data, ph.BYTE);
            } public float flow_rate_y_TRY(Inside ph) //Flow rate in radians/second about Y axis
            {
                if(ph.field_bit !=  209 && !try_visit_field(ph, 209)) return 0;
                return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  ph.BYTE, 4)));
            }
            public void flow_rate_y_SET(float src, Inside ph)//Flow rate in radians/second about Y axis
            {
                if(ph.field_bit != 209)insert_field(ph, 209, 0);
                BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src), 4, data, ph.BYTE);
            } static readonly Meta meta100 = new Meta(100, 0, 0, 1, 27, 208, 0, _uf, _Tf);
        } public class GLOBAL_VISION_POSITION_ESTIMATE : Pack, CommunicationChannel.Sendable
        {
            internal GLOBAL_VISION_POSITION_ESTIMATE() : base(meta101, 0) { }
            internal GLOBAL_VISION_POSITION_ESTIMATE(int bytes) : base(meta101, bytes) { }
            public ulong usec //Timestamp (microseconds, synced to UNIX time or since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x //Global X position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Global Y position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Global Z position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float roll //Roll angle in rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float pitch //Pitch angle in rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float yaw //Yaw angle in rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
            static readonly Meta meta101 = new Meta(101, 0, 0, 1, 32, 256);
        } public class VISION_POSITION_ESTIMATE : Pack, CommunicationChannel.Sendable
        {
            internal VISION_POSITION_ESTIMATE() : base(meta102, 0) { }
            internal VISION_POSITION_ESTIMATE(int bytes) : base(meta102, bytes) { }
            public ulong usec //Timestamp (microseconds, synced to UNIX time or since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x //Global X position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Global Y position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Global Z position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float roll //Roll angle in rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float pitch //Pitch angle in rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float yaw //Yaw angle in rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
            static readonly Meta meta102 = new Meta(102, 0, 0, 1, 32, 256);
        } public class VISION_SPEED_ESTIMATE : Pack, CommunicationChannel.Sendable
        {
            internal VISION_SPEED_ESTIMATE() : base(meta103, 0) { }
            internal VISION_SPEED_ESTIMATE(int bytes) : base(meta103, bytes) { }
            public ulong usec //Timestamp (microseconds, synced to UNIX time or since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x //Global X speed
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Global Y speed
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Global Z speed
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }
            static readonly Meta meta103 = new Meta(103, 0, 0, 1, 20, 160);
        } public class VICON_POSITION_ESTIMATE : Pack, CommunicationChannel.Sendable
        {
            internal VICON_POSITION_ESTIMATE() : base(meta104, 0) { }
            internal VICON_POSITION_ESTIMATE(int bytes) : base(meta104, bytes) { }
            public ulong usec //Timestamp (microseconds, synced to UNIX time or since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x //Global X position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Global Y position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Global Z position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float roll //Roll angle in rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float pitch //Pitch angle in rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float yaw //Yaw angle in rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
            static readonly Meta meta104 = new Meta(104, 0, 0, 1, 32, 256);
        }/**
*The IMU readings in SI units in NED body frame*/
        public class HIGHRES_IMU : Pack, CommunicationChannel.Sendable
        {
            internal HIGHRES_IMU() : base(meta105, 0) { }
            internal HIGHRES_IMU(int bytes) : base(meta105, bytes) { }
            public ushort fields_updated //Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ulong time_usec //Timestamp (microseconds, synced to UNIX time or since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  2, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  2);}
            }

            public float xacc //X acceleration (m/s^2)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float yacc //Y acceleration (m/s^2)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float zacc //Z acceleration (m/s^2)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float xgyro //Angular speed around X axis (rad / sec)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            public float ygyro //Angular speed around Y axis (rad / sec)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 26);}
            }

            public float zgyro //Angular speed around Z axis (rad / sec)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public float xmag //X Magnetic field (Gauss)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  34, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 34);}
            }

            public float ymag //Y Magnetic field (Gauss)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  38, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 38);}
            }

            public float zmag //Z Magnetic field (Gauss)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  42, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 42);}
            }

            public float abs_pressure //Absolute pressure in millibar
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  46, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 46);}
            }

            public float diff_pressure //Differential pressure in millibar
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  50, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 50);}
            }

            public float pressure_alt //Altitude calculated from pressure
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  54, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 54);}
            }

            public float temperature //Temperature in degrees celsius
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  58, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 58);}
            }
            static readonly Meta meta105 = new Meta(105, 1, 0, 1, 62, 496);
        }/**
*Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)*/
        public class OPTICAL_FLOW_RAD : Pack, CommunicationChannel.Sendable
        {
            internal OPTICAL_FLOW_RAD() : base(meta106, 0) { }
            internal OPTICAL_FLOW_RAD(int bytes) : base(meta106, bytes) { }
            /**
            *Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain
            *	 average flow. The integration time also indicates the*/
            public uint integration_time_us
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public uint time_delta_distance_us //Time in microseconds since the distance was sampled.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  4);}
            }

            public ulong time_usec //Timestamp (microseconds, synced to UNIX time or since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public byte sensor_id //Sensor ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            /**
            *Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
            *	 motion along the positive Y axis induces a negative flow.*/
            public float integrated_x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }

            /**
            *Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
            *	 motion along the positive X axis induces a positive flow.*/
            public float integrated_y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  21, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 21);}
            }

            public float integrated_xgyro //RH rotation around X axis (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  25, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 25);}
            }

            public float integrated_ygyro //RH rotation around Y axis (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  29, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 29);}
            }

            public float integrated_zgyro //RH rotation around Z axis (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  33, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 33);}
            }

            public short temperature //Temperature * 100 in centi-degrees Celsius
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  37, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  37);}
            }

            public byte quality //Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  39, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  39);}
            }

            /**
            *Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative
            *	 value: Unknown distance*/
            public float distance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }
            static readonly Meta meta106 = new Meta(106, 0, 2, 1, 44, 352);
        }/**
*The IMU readings in SI units in NED body frame*/
        public class HIL_SENSOR : Pack, CommunicationChannel.Sendable
        {
            internal HIL_SENSOR() : base(meta107, 0) { }
            internal HIL_SENSOR(int bytes) : base(meta107, bytes) { }
            /**
            *Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full
            *	 reset of attitude/position/velocities/etc was performed in sim*/
            public uint fields_updated
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public ulong time_usec //Timestamp (microseconds, synced to UNIX time or since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  4, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  4);}
            }

            public float xacc //X acceleration (m/s^2)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float yacc //Y acceleration (m/s^2)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float zacc //Z acceleration (m/s^2)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float xgyro //Angular speed around X axis in body frame (rad / sec)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float ygyro //Angular speed around Y axis in body frame (rad / sec)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float zgyro //Angular speed around Z axis in body frame (rad / sec)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float xmag //X Magnetic field (Gauss)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float ymag //Y Magnetic field (Gauss)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float zmag //Z Magnetic field (Gauss)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float abs_pressure //Absolute pressure in millibar
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  48, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }

            public float diff_pressure //Differential pressure (airspeed) in millibar
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  52, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 52);}
            }

            public float pressure_alt //Altitude calculated from pressure
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  56, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 56);}
            }

            public float temperature //Temperature in degrees celsius
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  60, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 60);}
            }
            static readonly Meta meta107 = new Meta(107, 0, 1, 1, 64, 512);
        }/**
*Status of simulation environment, if used*/
        public class SIM_STATE : Pack, CommunicationChannel.Sendable
        {
            internal SIM_STATE() : base(meta108, 0) { }
            internal SIM_STATE(int bytes) : base(meta108, bytes) { }
            public float q1 //True attitude quaternion component 1, w (1 in null-rotation)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 0);}
            }

            public float q2 //True attitude quaternion component 2, x (0 in null-rotation)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float q3 //True attitude quaternion component 3, y (0 in null-rotation)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float q4 //True attitude quaternion component 4, z (0 in null-rotation)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float roll //Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float pitch //Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float yaw //Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float xacc //X acceleration m/s/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float yacc //Y acceleration m/s/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float zacc //Z acceleration m/s/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float xgyro //Angular speed around X axis rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float ygyro //Angular speed around Y axis rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float zgyro //Angular speed around Z axis rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  48, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }

            public float lat //Latitude in degrees
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  52, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 52);}
            }

            public float lon //Longitude in degrees
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  56, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 56);}
            }

            public float alt //Altitude in meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  60, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 60);}
            }

            public float std_dev_horz //Horizontal position standard deviation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  64, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 64);}
            }

            public float std_dev_vert //Vertical position standard deviation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  68, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 68);}
            }

            public float vn //True velocity in m/s in NORTH direction in earth-fixed NED frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  72, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 72);}
            }

            public float ve //True velocity in m/s in EAST direction in earth-fixed NED frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  76, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 76);}
            }

            public float vd //True velocity in m/s in DOWN direction in earth-fixed NED frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  80, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 80);}
            }
            static readonly Meta meta108 = new Meta(108, 0, 0, 0, 84, 672);
        }/**
*Status generated by radio and injected into MAVLink stream.*/
        public class RADIO_STATUS : Pack, CommunicationChannel.Sendable
        {
            internal RADIO_STATUS() : base(meta109, 0) { }
            internal RADIO_STATUS(int bytes) : base(meta109, bytes) { }
            public ushort rxerrors //Receive errors
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort fixed_ //Count of error corrected packets
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public byte rssi //Local signal strength
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte remrssi //Remote signal strength
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public byte txbuf //Remaining free buffer space in percent.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public byte noise //Background noise level
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  7);}
            }

            public byte remnoise //Remote background noise level
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }
            static readonly Meta meta109 = new Meta(109, 2, 0, 0, 9, 72);
        }/**
*File transfer message*/
        public class FILE_TRANSFER_PROTOCOL : Pack, CommunicationChannel.Sendable
        {
            internal FILE_TRANSFER_PROTOCOL() : base(meta110, 0) { }
            internal FILE_TRANSFER_PROTOCOL(int bytes) : base(meta110, bytes) { }
            public byte target_network //Network ID (0 for broadcast)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_system //System ID (0 for broadcast)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public byte target_component //Component ID (0 for broadcast)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            /**
            *Variable length payload. The length is defined by the remaining message length when subtracting the header
            *	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
            *	 message_type.  The particular encoding used can be extension specific and might not always be documented
            *	 as part of the mavlink specification*/
            public byte[] payload
            {
                get {return payload_GET(new byte[251], 0);}
                set {payload_SET(value, 0)  ;}
            }
            /**
            *Variable length payload. The length is defined by the remaining message length when subtracting the header
            *	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
            *	 message_type.  The particular encoding used can be extension specific and might not always be documented
            *	 as part of the mavlink specification*/
            public byte[]payload_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 3, dst_max = pos + 251; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            /**
            *Variable length payload. The length is defined by the remaining message length when subtracting the header
            *	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
            *	 message_type.  The particular encoding used can be extension specific and might not always be documented
            *	 as part of the mavlink specification*/
            public void payload_SET(byte[] src, int pos)
            {
                for(int BYTE =  3, src_max = pos + 251; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            static readonly Meta meta110 = new Meta(110, 0, 0, 0, 254, 2032);
        }/**
*Time synchronization message.*/
        public class TIMESYNC : Pack, CommunicationChannel.Sendable
        {
            internal TIMESYNC() : base(meta111, 0) { }
            internal TIMESYNC(int bytes) : base(meta111, bytes) { }
            public long tc1 //Time sync timestamp 1
            {
                get {  return (long)((long) BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public long ts1 //Time sync timestamp 2
            {
                get {  return (long)((long) BitUtils.get_bytes(data,  8, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }
            static readonly Meta meta111 = new Meta(111, 0, 0, 0, 16, 128);
        }/**
*Camera-IMU triggering and synchronisation message.*/
        public class CAMERA_TRIGGER : Pack, CommunicationChannel.Sendable
        {
            internal CAMERA_TRIGGER() : base(meta112, 0) { }
            internal CAMERA_TRIGGER(int bytes) : base(meta112, bytes) { }
            public uint seq //Image frame sequence
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public ulong time_usec //Timestamp for the image frame in microseconds
            {
                get {  return (BitUtils.get_bytes(data,  4, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  4);}
            }
            static readonly Meta meta112 = new Meta(112, 0, 1, 1, 12, 96);
        }/**
*The global position, as returned by the Global Positioning System (GPS). This is
*	 NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
        public class HIL_GPS : Pack, CommunicationChannel.Sendable
        {
            internal HIL_GPS() : base(meta113, 0) { }
            internal HIL_GPS(int bytes) : base(meta113, bytes) { }
            public ushort eph //GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort epv //GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort vel //GPS ground speed in cm/s. If unknown, set to: 65535
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            /**
            *Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
            *	 unknown, set to: 6553*/
            public ushort cog
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            /**
            *0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is
            *	 at least two, so always correctly fill in the fix*/
            public byte fix_type
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            public int lat //Latitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  17, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  17);}
            }

            public int lon //Longitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  21, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  21);}
            }

            public int alt //Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  25, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  25);}
            }

            public short vn //GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  29, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  29);}
            }

            public short ve //GPS velocity in cm/s in EAST direction in earth-fixed NED frame
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  31, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  31);}
            }

            public short vd //GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  33, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  33);}
            }

            public byte satellites_visible //Number of satellites visible. If unknown, set to 255
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  35, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  35);}
            }
            static readonly Meta meta113 = new Meta(113, 4, 0, 1, 36, 288);
        }/**
*Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)*/
        public class HIL_OPTICAL_FLOW : Pack, CommunicationChannel.Sendable
        {
            internal HIL_OPTICAL_FLOW() : base(meta114, 0) { }
            internal HIL_OPTICAL_FLOW(int bytes) : base(meta114, bytes) { }
            /**
            *Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain
            *	 average flow. The integration time also indicates the*/
            public uint integration_time_us
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public uint time_delta_distance_us //Time in microseconds since the distance was sampled.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  4);}
            }

            public ulong time_usec //Timestamp (microseconds, synced to UNIX time or since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public byte sensor_id //Sensor ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            /**
            *Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
            *	 motion along the positive Y axis induces a negative flow.*/
            public float integrated_x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }

            /**
            *Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
            *	 motion along the positive X axis induces a positive flow.*/
            public float integrated_y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  21, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 21);}
            }

            public float integrated_xgyro //RH rotation around X axis (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  25, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 25);}
            }

            public float integrated_ygyro //RH rotation around Y axis (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  29, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 29);}
            }

            public float integrated_zgyro //RH rotation around Z axis (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  33, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 33);}
            }

            public short temperature //Temperature * 100 in centi-degrees Celsius
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  37, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  37);}
            }

            public byte quality //Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  39, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  39);}
            }

            /**
            *Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative
            *	 value: Unknown distance*/
            public float distance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }
            static readonly Meta meta114 = new Meta(114, 0, 2, 1, 44, 352);
        }/**
*Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful
*	 for high throughput applications such as hardware in the loop simulations*/
        public class HIL_STATE_QUATERNION : Pack, CommunicationChannel.Sendable
        {
            internal HIL_STATE_QUATERNION() : base(meta115, 0) { }
            internal HIL_STATE_QUATERNION(int bytes) : base(meta115, bytes) { }
            public ushort ind_airspeed //Indicated airspeed, expressed as cm/s
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort true_airspeed //True airspeed, expressed as cm/s
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  4, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  4);}
            }

            public float[] attitude_quaternion //Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation
            {
                get {return attitude_quaternion_GET(new float[4], 0);}
                set {attitude_quaternion_SET(value, 0)  ;}
            }
            public float[]attitude_quaternion_GET(float[] dst_ch, int pos)  //Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation
            {
                for(int BYTE = 12, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public void attitude_quaternion_SET(float[] src, int pos)  //Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation
            {
                for(int BYTE =  12, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            public float rollspeed //Body frame roll / phi angular speed (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float pitchspeed //Body frame pitch / theta angular speed (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float yawspeed //Body frame yaw / psi angular speed (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public int lat //Latitude, expressed as * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  40, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  40);}
            }

            public int lon //Longitude, expressed as * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  44, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  44);}
            }

            public int alt //Altitude in meters, expressed as * 1000 (millimeters)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  48, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  48);}
            }

            public short vx //Ground X Speed (Latitude), expressed as cm/s
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  52, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  52);}
            }

            public short vy //Ground Y Speed (Longitude), expressed as cm/s
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  54, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  54);}
            }

            public short vz //Ground Z Speed (Altitude), expressed as cm/s
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  56, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  56);}
            }

            public short xacc //X acceleration (mg)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  58, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  58);}
            }

            public short yacc //Y acceleration (mg)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  60, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  60);}
            }

            public short zacc //Z acceleration (mg)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  62, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  62);}
            }
            static readonly Meta meta115 = new Meta(115, 2, 0, 1, 64, 512);
        }/**
*The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to
*	 the described unit*/
        public class SCALED_IMU2 : Pack, CommunicationChannel.Sendable
        {
            internal SCALED_IMU2() : base(meta116, 0) { }
            internal SCALED_IMU2(int bytes) : base(meta116, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public short xacc //X acceleration (mg)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  4);}
            }

            public short yacc //Y acceleration (mg)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  6);}
            }

            public short zacc //Z acceleration (mg)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  8);}
            }

            public short xgyro //Angular speed around X axis (millirad /sec)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public short ygyro //Angular speed around Y axis (millirad /sec)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }

            public short zgyro //Angular speed around Z axis (millirad /sec)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  14);}
            }

            public short xmag //X Magnetic field (milli tesla)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  16, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  16);}
            }

            public short ymag //Y Magnetic field (milli tesla)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  18, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  18);}
            }

            public short zmag //Z Magnetic field (milli tesla)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  20, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  20);}
            }
            static readonly Meta meta116 = new Meta(116, 0, 1, 0, 22, 176);
        }/**
*Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END
*	 is called*/
        public class LOG_REQUEST_LIST : Pack, CommunicationChannel.Sendable
        {
            internal LOG_REQUEST_LIST() : base(meta117, 0) { }
            internal LOG_REQUEST_LIST(int bytes) : base(meta117, bytes) { }
            public ushort start //First log id (0 for first available)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort end //Last log id (0xffff for last available)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }
            static readonly Meta meta117 = new Meta(117, 2, 0, 0, 6, 48);
        }/**
*Reply to LOG_REQUEST_LIST*/
        public class LOG_ENTRY : Pack, CommunicationChannel.Sendable
        {
            internal LOG_ENTRY() : base(meta118, 0) { }
            internal LOG_ENTRY(int bytes) : base(meta118, bytes) { }
            public ushort id //Log id
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort num_logs //Total number of logs
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort last_log_num //High log number
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public uint time_utc //UTC timestamp of log in seconds since 1970, or 0 if not available
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public uint size //Size of the log (may be approximate) in bytes
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  10);}
            }
            static readonly Meta meta118 = new Meta(118, 3, 2, 0, 14, 112);
        }/**
*Request a chunk of a log*/
        public class LOG_REQUEST_DATA : Pack, CommunicationChannel.Sendable
        {
            internal LOG_REQUEST_DATA() : base(meta119, 0) { }
            internal LOG_REQUEST_DATA(int bytes) : base(meta119, bytes) { }
            public ushort id //Log id (from LOG_ENTRY reply)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint ofs //Offset into the log
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public uint count //Number of bytes
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  10);}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  11);}
            }
            static readonly Meta meta119 = new Meta(119, 1, 2, 0, 12, 96);
        }/**
*Reply to LOG_REQUEST_DATA*/
        public class LOG_DATA : Pack, CommunicationChannel.Sendable
        {
            internal LOG_DATA() : base(meta120, 0) { }
            internal LOG_DATA(int bytes) : base(meta120, bytes) { }
            public ushort id //Log id (from LOG_ENTRY reply)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint ofs //Offset into the log
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public byte count //Number of bytes (zero for end of log)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public byte[] data_ //log data
            {
                get {return data__GET(new byte[90], 0);}
                set {data__SET(value, 0)  ;}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //log data
            {
                for(int BYTE = 7, dst_max = pos + 90; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public void data__SET(byte[] src, int pos)  //log data
            {
                for(int BYTE =  7, src_max = pos + 90; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            static readonly Meta meta120 = new Meta(120, 1, 1, 0, 97, 776);
        }/**
*Erase all logs*/
        public class LOG_ERASE : Pack, CommunicationChannel.Sendable
        {
            internal LOG_ERASE() : base(meta121, 0) { }
            internal LOG_ERASE(int bytes) : base(meta121, bytes) { }
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }
            static readonly Meta meta121 = new Meta(121, 0, 0, 0, 2, 16);
        }/**
*Stop log transfer and resume normal logging*/
        public class LOG_REQUEST_END : Pack, CommunicationChannel.Sendable
        {
            internal LOG_REQUEST_END() : base(meta122, 0) { }
            internal LOG_REQUEST_END(int bytes) : base(meta122, bytes) { }
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }
            static readonly Meta meta122 = new Meta(122, 0, 0, 0, 2, 16);
        }/**
*data for injecting into the onboard GPS (used for DGPS)*/
        public class GPS_INJECT_DATA : Pack, CommunicationChannel.Sendable
        {
            internal GPS_INJECT_DATA() : base(meta123, 0) { }
            internal GPS_INJECT_DATA(int bytes) : base(meta123, bytes) { }
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public byte len //data length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte[] data_ //raw data (110 is enough for 12 satellites of RTCMv2)
            {
                get {return data__GET(new byte[110], 0);}
                set {data__SET(value, 0)  ;}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //raw data (110 is enough for 12 satellites of RTCMv2)
            {
                for(int BYTE = 3, dst_max = pos + 110; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public void data__SET(byte[] src, int pos)  //raw data (110 is enough for 12 satellites of RTCMv2)
            {
                for(int BYTE =  3, src_max = pos + 110; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            static readonly Meta meta123 = new Meta(123, 0, 0, 0, 113, 904);
        }/**
*Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
        public class GPS2_RAW : Pack, CommunicationChannel.Sendable
        {
            internal GPS2_RAW() : base(meta124, 0) { }
            internal GPS2_RAW(int bytes) : base(meta124, bytes) { }
            public ushort eph //GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort epv //GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort vel //GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            /**
            *Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
            *	 unknown, set to: UINT16_MA*/
            public ushort cog
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public uint dgps_age //Age of DGPS info
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  8, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  8);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  12, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  12);}
            }

            public int lat //Latitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  20);}
            }

            public int lon //Longitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  24, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  24);}
            }

            public int alt //Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  28, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  28);}
            }

            public byte satellites_visible //Number of satellites visible. If unknown, set to 255
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  32, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  32);}
            }

            public byte dgps_numch //Number of DGPS satellites
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  33, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  33);}
            }

            public GPS_FIX_TYPE fix_type //See the GPS_FIX_TYPE enum.
            {
                get {  return (GPS_FIX_TYPE)(0 +  BitUtils.get_bits(data, 272, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 272);}
            }
            static readonly Meta meta124 = new Meta(124, 4, 1, 1, 35, 276);
        }/**
*Power supply status*/
        public class POWER_STATUS : Pack, CommunicationChannel.Sendable
        {
            internal POWER_STATUS() : base(meta125, 0) { }
            internal POWER_STATUS(int bytes) : base(meta125, bytes) { }
            public ushort Vcc //5V rail voltage in millivolts
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort Vservo //servo rail voltage in millivolts
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public MAV_POWER_STATUS flags //power supply status flags (see MAV_POWER_STATUS enum)
            {
                get {  return (MAV_POWER_STATUS)(1 +  BitUtils.get_bits(data, 32, 6));}
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 6, data, 32);}
            }
            static readonly Meta meta125 = new Meta(125, 2, 0, 0, 5, 38);
        }/**
*Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or
*	 telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages
*	 or change the devices settings. A message with zero bytes can be used to change just the baudrate*/
        public class SERIAL_CONTROL : Pack, CommunicationChannel.Sendable
        {
            internal SERIAL_CONTROL() : base(meta126, 0) { }
            internal SERIAL_CONTROL(int bytes) : base(meta126, bytes) { }
            public ushort timeout //Timeout for reply data in milliseconds
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint baudrate //Baudrate of transfer. Zero means no change.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public byte count //how many bytes in this transfer
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public byte[] data_ //serial data
            {
                get {return data__GET(new byte[70], 0);}
                set {data__SET(value, 0)  ;}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //serial data
            {
                for(int BYTE = 7, dst_max = pos + 70; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public void data__SET(byte[] src, int pos)  //serial data
            {
                for(int BYTE =  7, src_max = pos + 70; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            public SERIAL_CONTROL_DEV device //See SERIAL_CONTROL_DEV enum
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 616, 3))
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
                    throw  new ArgumentException("Unknown enum ID ");
                }
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 616);
                }
            }

            public SERIAL_CONTROL_FLAG flags //See SERIAL_CONTROL_FLAG enum
            {
                get {  return (SERIAL_CONTROL_FLAG)(1 +  BitUtils.get_bits(data, 619, 5));}
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 5, data, 619);}
            }
            static readonly Meta meta126 = new Meta(126, 1, 1, 0, 78, 624);
        }/**
*RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting*/
        public class GPS_RTK : Pack, CommunicationChannel.Sendable
        {
            internal GPS_RTK() : base(meta127, 0) { }
            internal GPS_RTK(int bytes) : base(meta127, bytes) { }
            public ushort wn //GPS Week Number of last baseline
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_last_baseline_ms //Time since boot of last baseline message received in ms.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public uint tow //GPS Time of Week of last baseline
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public uint accuracy //Current estimate of baseline accuracy.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  10);}
            }

            public byte rtk_receiver_id //Identification of connected RTK receiver.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  14, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  14);}
            }

            public byte rtk_health //GPS-specific health report for RTK data.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  15, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  15);}
            }

            public byte rtk_rate //Rate of baseline messages being received by GPS, in HZ
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            public byte nsats //Current number of sats used for RTK calculation.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  17, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  17);}
            }

            public byte baseline_coords_type //Coordinate system of baseline. 0 == ECEF, 1 == NED
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  18, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  18);}
            }

            public int baseline_a_mm //Current baseline in ECEF x or NED north component in mm.
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  19, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  19);}
            }

            public int baseline_b_mm //Current baseline in ECEF y or NED east component in mm.
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  23, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  23);}
            }

            public int baseline_c_mm //Current baseline in ECEF z or NED down component in mm.
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  27, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  27);}
            }

            public int iar_num_hypotheses //Current number of integer ambiguity hypotheses.
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  31, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  31);}
            }
            static readonly Meta meta127 = new Meta(127, 1, 3, 0, 35, 280);
        }/**
*RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting*/
        public class GPS2_RTK : Pack, CommunicationChannel.Sendable
        {
            internal GPS2_RTK() : base(meta128, 0) { }
            internal GPS2_RTK(int bytes) : base(meta128, bytes) { }
            public ushort wn //GPS Week Number of last baseline
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_last_baseline_ms //Time since boot of last baseline message received in ms.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public uint tow //GPS Time of Week of last baseline
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public uint accuracy //Current estimate of baseline accuracy.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  10);}
            }

            public byte rtk_receiver_id //Identification of connected RTK receiver.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  14, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  14);}
            }

            public byte rtk_health //GPS-specific health report for RTK data.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  15, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  15);}
            }

            public byte rtk_rate //Rate of baseline messages being received by GPS, in HZ
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            public byte nsats //Current number of sats used for RTK calculation.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  17, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  17);}
            }

            public byte baseline_coords_type //Coordinate system of baseline. 0 == ECEF, 1 == NED
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  18, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  18);}
            }

            public int baseline_a_mm //Current baseline in ECEF x or NED north component in mm.
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  19, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  19);}
            }

            public int baseline_b_mm //Current baseline in ECEF y or NED east component in mm.
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  23, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  23);}
            }

            public int baseline_c_mm //Current baseline in ECEF z or NED down component in mm.
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  27, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  27);}
            }

            public int iar_num_hypotheses //Current number of integer ambiguity hypotheses.
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  31, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  31);}
            }
            static readonly Meta meta128 = new Meta(128, 1, 3, 0, 35, 280);
        }/**
*The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described
*	 unit*/
        public class SCALED_IMU3 : Pack, CommunicationChannel.Sendable
        {
            internal SCALED_IMU3() : base(meta129, 0) { }
            internal SCALED_IMU3(int bytes) : base(meta129, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public short xacc //X acceleration (mg)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  4);}
            }

            public short yacc //Y acceleration (mg)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  6);}
            }

            public short zacc //Z acceleration (mg)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  8);}
            }

            public short xgyro //Angular speed around X axis (millirad /sec)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public short ygyro //Angular speed around Y axis (millirad /sec)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }

            public short zgyro //Angular speed around Z axis (millirad /sec)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  14);}
            }

            public short xmag //X Magnetic field (milli tesla)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  16, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  16);}
            }

            public short ymag //Y Magnetic field (milli tesla)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  18, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  18);}
            }

            public short zmag //Z Magnetic field (milli tesla)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  20, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  20);}
            }
            static readonly Meta meta129 = new Meta(129, 0, 1, 0, 22, 176);
        } public class DATA_TRANSMISSION_HANDSHAKE : Pack, CommunicationChannel.Sendable
        {
            internal DATA_TRANSMISSION_HANDSHAKE() : base(meta130, 0) { }
            internal DATA_TRANSMISSION_HANDSHAKE(int bytes) : base(meta130, bytes) { }
            public ushort width //Width of a matrix or image
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort height //Height of a matrix or image
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort packets //number of packets beeing sent (set on ACK only)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public uint size //total data size in bytes (set on ACK only)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public byte type //type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  10);}
            }

            /**
            *payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on
            *	 ACK only*/
            public byte payload
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  11);}
            }

            public byte jpg_quality //JPEG quality out of [1,100]
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  12, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  12);}
            }
            static readonly Meta meta130 = new Meta(130, 3, 1, 0, 13, 104);
        } public class ENCAPSULATED_DATA : Pack, CommunicationChannel.Sendable
        {
            internal ENCAPSULATED_DATA() : base(meta131, 0) { }
            internal ENCAPSULATED_DATA(int bytes) : base(meta131, bytes) { }
            public ushort seqnr //sequence number (starting with 0 on every transmission)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte[] data_ //image data bytes
            {
                get {return data__GET(new byte[253], 0);}
                set {data__SET(value, 0)  ;}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //image data bytes
            {
                for(int BYTE = 2, dst_max = pos + 253; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public void data__SET(byte[] src, int pos)  //image data bytes
            {
                for(int BYTE =  2, src_max = pos + 253; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            static readonly Meta meta131 = new Meta(131, 1, 0, 0, 255, 2040);
        } public class DISTANCE_SENSOR : Pack, CommunicationChannel.Sendable
        {
            internal DISTANCE_SENSOR() : base(meta132, 0) { }
            internal DISTANCE_SENSOR(int bytes) : base(meta132, bytes) { }
            public ushort min_distance //Minimum distance the sensor can measure in centimeters
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort max_distance //Maximum distance the sensor can measure in centimeters
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort current_distance //Current distance reading
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public uint time_boot_ms //Time since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public byte id //Onboard ID of the sensor
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  10);}
            }

            public byte covariance //Measurement covariance in centimeters, 0 for unknown / invalid readings
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  11);}
            }

            public MAV_DISTANCE_SENSOR type //Type from MAV_DISTANCE_SENSOR enum.
            {
                get {  return (MAV_DISTANCE_SENSOR)(0 +  BitUtils.get_bits(data, 96, 3));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 96);}
            }

            /**
            *Direction the sensor faces from MAV_SENSOR_ORIENTATION enum. downward-facing: ROTATION_PITCH_270, upward-facing:
            *	 ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90,
            *	 right-facing: ROTATION_YAW_27*/
            public MAV_SENSOR_ORIENTATION orientation
            {
                get {  return (MAV_SENSOR_ORIENTATION)(0 +  BitUtils.get_bits(data, 99, 6));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 6, data, 99);}
            }
            static readonly Meta meta132 = new Meta(132, 3, 1, 0, 14, 105);
        }/**
*Request for terrain data and terrain status*/
        public class TERRAIN_REQUEST : Pack, CommunicationChannel.Sendable
        {
            internal TERRAIN_REQUEST() : base(meta133, 0) { }
            internal TERRAIN_REQUEST(int bytes) : base(meta133, bytes) { }
            public ushort grid_spacing //Grid spacing in meters
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ulong mask //Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
            {
                get {  return (BitUtils.get_bytes(data,  2, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  2);}
            }

            public int lat //Latitude of SW corner of first grid (degrees *10^7)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  10);}
            }

            public int lon //Longitude of SW corner of first grid (in degrees *10^7)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  14, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  14);}
            }
            static readonly Meta meta133 = new Meta(133, 1, 0, 1, 18, 144);
        }/**
*Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUES*/
        public class TERRAIN_DATA : Pack, CommunicationChannel.Sendable
        {
            internal TERRAIN_DATA() : base(meta134, 0) { }
            internal TERRAIN_DATA(int bytes) : base(meta134, bytes) { }
            public ushort grid_spacing //Grid spacing in meters
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public int lat //Latitude of SW corner of first grid (degrees *10^7)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  2);}
            }

            public int lon //Longitude of SW corner of first grid (in degrees *10^7)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  6);}
            }

            public byte gridbit //bit within the terrain request mask
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  10);}
            }

            public short[] data_ //Terrain data in meters AMSL
            {
                get {return data__GET(new short[16], 0);}
                set {data__SET(value, 0)  ;}
            }
            public short[]data__GET(short[] dst_ch, int pos)  //Terrain data in meters AMSL
            {
                for(int BYTE = 11, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (short)((short) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public void data__SET(short[] src, int pos)  //Terrain data in meters AMSL
            {
                for(int BYTE =  11, src_max = pos + 16; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            }
            static readonly Meta meta134 = new Meta(134, 1, 0, 0, 43, 344);
        }/**
*Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle
*	 has all terrain data needed for a mission*/
        public class TERRAIN_CHECK : Pack, CommunicationChannel.Sendable
        {
            internal TERRAIN_CHECK() : base(meta135, 0) { }
            internal TERRAIN_CHECK(int bytes) : base(meta135, bytes) { }
            public int lat //Latitude (degrees *10^7)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  0);}
            }

            public int lon //Longitude (degrees *10^7)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  4, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  4);}
            }
            static readonly Meta meta135 = new Meta(135, 0, 0, 0, 8, 64);
        }/**
*Response from a TERRAIN_CHECK request*/
        public class TERRAIN_REPORT : Pack, CommunicationChannel.Sendable
        {
            internal TERRAIN_REPORT() : base(meta136, 0) { }
            internal TERRAIN_REPORT(int bytes) : base(meta136, bytes) { }
            public ushort spacing //grid spacing (zero if terrain at this location unavailable)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort pending //Number of 4x4 terrain blocks waiting to be received or read from disk
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort loaded //Number of 4x4 terrain blocks in memory
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public int lat //Latitude (degrees *10^7)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  6);}
            }

            public int lon //Longitude (degrees *10^7)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  10);}
            }

            public float terrain_height //Terrain height in meters AMSL
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float current_height //Current vehicle height above lat/lon terrain height (meters)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }
            static readonly Meta meta136 = new Meta(136, 3, 0, 0, 22, 176);
        }/**
*Barometer readings for 2nd barometer*/
        public class SCALED_PRESSURE2 : Pack, CommunicationChannel.Sendable
        {
            internal SCALED_PRESSURE2() : base(meta137, 0) { }
            internal SCALED_PRESSURE2(int bytes) : base(meta137, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float press_abs //Absolute pressure (hectopascal)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float press_diff //Differential pressure 1 (hectopascal)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public short temperature //Temperature measurement (0.01 degrees celsius)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }
            static readonly Meta meta137 = new Meta(137, 0, 1, 0, 14, 112);
        }/**
*Motion capture attitude and position*/
        public class ATT_POS_MOCAP : Pack, CommunicationChannel.Sendable
        {
            internal ATT_POS_MOCAP() : base(meta138, 0) { }
            internal ATT_POS_MOCAP(int bytes) : base(meta138, bytes) { }
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float[] q //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                get {return q_GET(new float[4], 0);}
                set {q_SET(value, 0)  ;}
            }
            public float[]q_GET(float[] dst_ch, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                for(int BYTE = 8, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public void q_SET(float[] src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                for(int BYTE =  8, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            public float x //X position in meters (NED)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float y //Y position in meters (NED)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float z //Z position in meters (NED)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }
            static readonly Meta meta138 = new Meta(138, 0, 0, 1, 36, 288);
        }/**
*Set the vehicle attitude and body angular rates.*/
        public class SET_ACTUATOR_CONTROL_TARGET : Pack, CommunicationChannel.Sendable
        {
            internal SET_ACTUATOR_CONTROL_TARGET() : base(meta139, 0) { }
            internal SET_ACTUATOR_CONTROL_TARGET(int bytes) : base(meta139, bytes) { }
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            /**
            *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
            *	 this field to difference between instances*/
            public byte group_mlx
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  9);}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  10);}
            }

            /**
            *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
            *	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
            *	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
            *	 mixer to repurpose them as generic outputs*/
            public float[] controls
            {
                get {return controls_GET(new float[8], 0);}
                set {controls_SET(value, 0)  ;}
            }
            /**
            *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
            *	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
            *	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
            *	 mixer to repurpose them as generic outputs*/
            public float[]controls_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 11, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            /**
            *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
            *	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
            *	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
            *	 mixer to repurpose them as generic outputs*/
            public void controls_SET(float[] src, int pos)
            {
                for(int BYTE =  11, src_max = pos + 8; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
            static readonly Meta meta139 = new Meta(139, 0, 0, 1, 43, 344);
        }/**
*Set the vehicle attitude and body angular rates.*/
        public class ACTUATOR_CONTROL_TARGET : Pack, CommunicationChannel.Sendable
        {
            internal ACTUATOR_CONTROL_TARGET() : base(meta140, 0) { }
            internal ACTUATOR_CONTROL_TARGET(int bytes) : base(meta140, bytes) { }
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            /**
            *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
            *	 this field to difference between instances*/
            public byte group_mlx
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }

            /**
            *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
            *	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
            *	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
            *	 mixer to repurpose them as generic outputs*/
            public float[] controls
            {
                get {return controls_GET(new float[8], 0);}
                set {controls_SET(value, 0)  ;}
            }
            /**
            *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
            *	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
            *	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
            *	 mixer to repurpose them as generic outputs*/
            public float[]controls_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 9, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            /**
            *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
            *	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
            *	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
            *	 mixer to repurpose them as generic outputs*/
            public void controls_SET(float[] src, int pos)
            {
                for(int BYTE =  9, src_max = pos + 8; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
            static readonly Meta meta140 = new Meta(140, 0, 0, 1, 41, 328);
        }/**
*The current system altitude.*/
        public class ALTITUDE : Pack, CommunicationChannel.Sendable
        {
            internal ALTITUDE() : base(meta141, 0) { }
            internal ALTITUDE(int bytes) : base(meta141, bytes) { }
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            /**
            *This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
            *	 local altitude change). The only guarantee on this field is that it will never be reset and is consistent
            *	 within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
            *	 time. This altitude will also drift and vary between flights*/
            public float altitude_monotonic
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            /**
            *This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
            *	 like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
            *	 are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
            *	 by default and not the WGS84 altitude*/
            public float altitude_amsl
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            /**
            *This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
            *	 to the coordinate origin (0, 0, 0). It is up-positive*/
            public float altitude_local
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float altitude_relative //This is the altitude above the home position. It resets on each change of the current home position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            /**
            *This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
            *	 than -1000 should be interpreted as unknown*/
            public float altitude_terrain
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            /**
            *This is not the altitude, but the clear space below the system according to the fused clearance estimate.
            *	 It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
            *	 target. A negative value indicates no measurement available*/
            public float bottom_clearance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
            static readonly Meta meta141 = new Meta(141, 0, 0, 1, 32, 256);
        }/**
*The autopilot is requesting a resource (file, binary, other type of data)*/
        public class RESOURCE_REQUEST : Pack, CommunicationChannel.Sendable
        {
            internal RESOURCE_REQUEST() : base(meta142, 0) { }
            internal RESOURCE_REQUEST(int bytes) : base(meta142, bytes) { }
            public byte request_id //Request ID. This ID should be re-used when sending back URI contents
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte uri_type //The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            /**
            *The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
            *	 on the URI type enum*/
            public byte[] uri
            {
                get {return uri_GET(new byte[120], 0);}
                set {uri_SET(value, 0)  ;}
            }
            /**
            *The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
            *	 on the URI type enum*/
            public byte[]uri_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 2, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            /**
            *The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
            *	 on the URI type enum*/
            public void uri_SET(byte[] src, int pos)
            {
                for(int BYTE =  2, src_max = pos + 120; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            public byte transfer_type //The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  122, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  122);}
            }

            /**
            *The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
            *	 has a storage associated (e.g. MAVLink FTP)*/
            public byte[] storage
            {
                get {return storage_GET(new byte[120], 0);}
                set {storage_SET(value, 0)  ;}
            }
            /**
            *The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
            *	 has a storage associated (e.g. MAVLink FTP)*/
            public byte[]storage_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 123, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            /**
            *The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
            *	 has a storage associated (e.g. MAVLink FTP)*/
            public void storage_SET(byte[] src, int pos)
            {
                for(int BYTE =  123, src_max = pos + 120; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            static readonly Meta meta142 = new Meta(142, 0, 0, 0, 243, 1944);
        }/**
*Barometer readings for 3rd barometer*/
        public class SCALED_PRESSURE3 : Pack, CommunicationChannel.Sendable
        {
            internal SCALED_PRESSURE3() : base(meta143, 0) { }
            internal SCALED_PRESSURE3(int bytes) : base(meta143, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float press_abs //Absolute pressure (hectopascal)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float press_diff //Differential pressure 1 (hectopascal)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public short temperature //Temperature measurement (0.01 degrees celsius)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }
            static readonly Meta meta143 = new Meta(143, 0, 1, 0, 14, 112);
        }/**
*current motion information from a designated system*/
        public class FOLLOW_TARGET : Pack, CommunicationChannel.Sendable
        {
            internal FOLLOW_TARGET() : base(meta144, 0) { }
            internal FOLLOW_TARGET(int bytes) : base(meta144, bytes) { }
            public ulong timestamp //Timestamp in milliseconds since system boot
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public ulong custom_state //button states or switches of a tracker device
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public byte est_capabilities //bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            public int lat //Latitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  17, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  17);}
            }

            public int lon //Longitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  21, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  21);}
            }

            public float alt //AMSL, in meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  25, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 25);}
            }

            public float[] vel //target velocity (0,0,0) for unknown
            {
                get {return vel_GET(new float[3], 0);}
                set {vel_SET(value, 0)  ;}
            }
            public float[]vel_GET(float[] dst_ch, int pos)  //target velocity (0,0,0) for unknown
            {
                for(int BYTE = 29, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public void vel_SET(float[] src, int pos)  //target velocity (0,0,0) for unknown
            {
                for(int BYTE =  29, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            public float[] acc //linear target acceleration (0,0,0) for unknown
            {
                get {return acc_GET(new float[3], 0);}
                set {acc_SET(value, 0)  ;}
            }
            public float[]acc_GET(float[] dst_ch, int pos)  //linear target acceleration (0,0,0) for unknown
            {
                for(int BYTE = 41, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public void acc_SET(float[] src, int pos)  //linear target acceleration (0,0,0) for unknown
            {
                for(int BYTE =  41, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            public float[] attitude_q //(1 0 0 0 for unknown)
            {
                get {return attitude_q_GET(new float[4], 0);}
                set {attitude_q_SET(value, 0)  ;}
            }
            public float[]attitude_q_GET(float[] dst_ch, int pos)  //(1 0 0 0 for unknown)
            {
                for(int BYTE = 53, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public void attitude_q_SET(float[] src, int pos)  //(1 0 0 0 for unknown)
            {
                for(int BYTE =  53, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            public float[] rates //(0 0 0 for unknown)
            {
                get {return rates_GET(new float[3], 0);}
                set {rates_SET(value, 0)  ;}
            }
            public float[]rates_GET(float[] dst_ch, int pos)  //(0 0 0 for unknown)
            {
                for(int BYTE = 69, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public void rates_SET(float[] src, int pos)  //(0 0 0 for unknown)
            {
                for(int BYTE =  69, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            public float[] position_cov //eph epv
            {
                get {return position_cov_GET(new float[3], 0);}
                set {position_cov_SET(value, 0)  ;}
            }
            public float[]position_cov_GET(float[] dst_ch, int pos)  //eph epv
            {
                for(int BYTE = 81, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public void position_cov_SET(float[] src, int pos)  //eph epv
            {
                for(int BYTE =  81, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
            static readonly Meta meta144 = new Meta(144, 0, 0, 2, 93, 744);
        }/**
*The smoothed, monotonic system state used to feed the control loops of the system.*/
        public class CONTROL_SYSTEM_STATE : Pack, CommunicationChannel.Sendable
        {
            internal CONTROL_SYSTEM_STATE() : base(meta146, 0) { }
            internal CONTROL_SYSTEM_STATE(int bytes) : base(meta146, bytes) { }
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x_acc //X acceleration in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y_acc //Y acceleration in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z_acc //Z acceleration in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float x_vel //X velocity in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float y_vel //Y velocity in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float z_vel //Z velocity in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float x_pos //X position in local frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float y_pos //Y position in local frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float z_pos //Z position in local frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float airspeed //Airspeed, set to -1 if unknown
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float[] vel_variance //Variance of body velocity estimate
            {
                get {return vel_variance_GET(new float[3], 0);}
                set {vel_variance_SET(value, 0)  ;}
            }
            public float[]vel_variance_GET(float[] dst_ch, int pos)  //Variance of body velocity estimate
            {
                for(int BYTE = 48, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public void vel_variance_SET(float[] src, int pos)  //Variance of body velocity estimate
            {
                for(int BYTE =  48, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            public float[] pos_variance //Variance in local position
            {
                get {return pos_variance_GET(new float[3], 0);}
                set {pos_variance_SET(value, 0)  ;}
            }
            public float[]pos_variance_GET(float[] dst_ch, int pos)  //Variance in local position
            {
                for(int BYTE = 60, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public void pos_variance_SET(float[] src, int pos)  //Variance in local position
            {
                for(int BYTE =  60, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            public float[] q //The attitude, represented as Quaternion
            {
                get {return q_GET(new float[4], 0);}
                set {q_SET(value, 0)  ;}
            }
            public float[]q_GET(float[] dst_ch, int pos)  //The attitude, represented as Quaternion
            {
                for(int BYTE = 72, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public void q_SET(float[] src, int pos)  //The attitude, represented as Quaternion
            {
                for(int BYTE =  72, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            public float roll_rate //Angular rate in roll axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  88, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 88);}
            }

            public float pitch_rate //Angular rate in pitch axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  92, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 92);}
            }

            public float yaw_rate //Angular rate in yaw axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  96, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 96);}
            }
            static readonly Meta meta146 = new Meta(146, 0, 0, 1, 100, 800);
        }/**
*Battery information*/
        public class BATTERY_STATUS : Pack, CommunicationChannel.Sendable
        {
            internal BATTERY_STATUS() : base(meta147, 0) { }
            internal BATTERY_STATUS(int bytes) : base(meta147, bytes) { }
            /**
            *Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
            *	 should have the UINT16_MAX value*/
            public ushort[] voltages
            {
                get {return voltages_GET(new ushort[10], 0);}
                set {voltages_SET(value, 0)  ;}
            }
            /**
            *Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
            *	 should have the UINT16_MAX value*/
            public ushort[]voltages_GET(ushort[] dst_ch, int pos)
            {
                for(int BYTE = 0, dst_max = pos + 10; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            /**
            *Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
            *	 should have the UINT16_MAX value*/
            public void voltages_SET(ushort[] src, int pos)
            {
                for(int BYTE =  0, src_max = pos + 10; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ulong)(src[pos]), 2, data,  BYTE);
            }

            public byte id //Battery ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  20, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  20);}
            }

            public short temperature //Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  21, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  21);}
            }

            public short current_battery //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  23, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  23);}
            }

            public int current_consumed //Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimat
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  25, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  25);}
            }

            /**
            *Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide
            *	 energy consumption estimat*/
            public int energy_consumed
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  29, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  29);}
            }

            public sbyte battery_remaining //Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  33, 1));}
                set {  BitUtils.set_bytes((byte)(value), 1, data,  33);}
            }

            public MAV_BATTERY_FUNCTION battery_function //Function of the battery
            {
                get {  return (MAV_BATTERY_FUNCTION)(0 +  BitUtils.get_bits(data, 272, 3));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 272);}
            }

            public MAV_BATTERY_TYPE type //Type (chemistry) of the battery
            {
                get {  return (MAV_BATTERY_TYPE)(0 +  BitUtils.get_bits(data, 275, 3));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 275);}
            }
            static readonly Meta meta147 = new Meta(147, 10, 0, 0, 35, 278);
        } public class AUTOPILOT_VERSION : Pack, CommunicationChannel.Sendable
        {
            internal AUTOPILOT_VERSION() : base(meta148, 0) { }

            public ushort vendor_id //ID of the board vendor
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort product_id //ID of the product
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public uint flight_sw_version //Firmware version number
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  4);}
            }

            public uint middleware_sw_version //Middleware version number
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  8);}
            }

            public uint os_sw_version //Operating system version number
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  12);}
            }

            public uint board_version //HW / board version (last 8 bytes should be silicon ID, if any)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  16);}
            }

            public ulong uid //UID if provided by hardware (see uid2)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  20);}
            }

            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	 should allow to identify the commit using the main version number even for very large code bases*/
            public byte[] flight_custom_version
            {
                set {flight_custom_version_SET(value, 0)  ;}
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	 should allow to identify the commit using the main version number even for very large code bases*/
            public void flight_custom_version_SET(byte[] src, int pos)
            {
                for(int BYTE =  28, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	 should allow to identify the commit using the main version number even for very large code bases*/
            public byte[] middleware_custom_version
            {
                set {middleware_custom_version_SET(value, 0)  ;}
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	 should allow to identify the commit using the main version number even for very large code bases*/
            public void middleware_custom_version_SET(byte[] src, int pos)
            {
                for(int BYTE =  36, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	 should allow to identify the commit using the main version number even for very large code bases*/
            public byte[] os_custom_version
            {
                set {os_custom_version_SET(value, 0)  ;}
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	 should allow to identify the commit using the main version number even for very large code bases*/
            public void os_custom_version_SET(byte[] src, int pos)
            {
                for(int BYTE =  44, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            public MAV_PROTOCOL_CAPABILITY capabilities //bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 17, data, 416);}
            }
            /**
            *UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
            *	 use uid*/
            public void uid2_SET(byte[] src, int pos, Inside ph)
            {
                if(ph.field_bit != 433)insert_field(ph, 433, 0);
                for(int BYTE =  ph.BYTE, src_max = pos + 18; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            } static readonly Meta meta148 = new Meta(148, 2, 4, 1, 56, 433, 0, _eg);
        }/**
*The location of a landing area captured from a downward facing camera*/
        public class LANDING_TARGET : Pack, CommunicationChannel.Sendable
        {
            internal LANDING_TARGET() : base(meta149, 0) { }

            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public byte target_num //The ID of the target if multiple targets are present
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }

            public float angle_x //X-axis angular offset (in radians) of the target from the center of the image
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 9);}
            }

            public float angle_y //Y-axis angular offset (in radians) of the target from the center of the image
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 13);}
            }

            public float distance //Distance to the target from the vehicle in meters
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }

            public float size_x //Size in radians of target along x-axis
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 21);}
            }

            public float size_y //Size in radians of target along y-axis
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 25);}
            }

            public MAV_FRAME frame //MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 232);}
            }

            public LANDING_TARGET_TYPE type //LANDING_TARGET_TYPE enum specifying the type of landing target
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 2, data, 236);}
            }
            public void x_SET(float src, Inside ph)//X Position of the landing target on MAV_FRAME
            {
                if(ph.field_bit != 238)insert_field(ph, 238, 0);
                BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src), 4, data, ph.BYTE);
            } public void y_SET(float src, Inside ph) //Y Position of the landing target on MAV_FRAME
            {
                if(ph.field_bit != 239)insert_field(ph, 239, 0);
                BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src), 4, data, ph.BYTE);
            } public void z_SET(float src, Inside ph) //Z Position of the landing target on MAV_FRAME
            {
                if(ph.field_bit != 240)insert_field(ph, 240, 0);
                BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src), 4, data, ph.BYTE);
            } public void q_SET(float[] src, int pos, Inside ph)//Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                if(ph.field_bit != 241)insert_field(ph, 241, 0);
                for(int BYTE =  ph.BYTE, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }/**
*Boolean indicating known position (1) or default unkown position (0), for validation of positioning of
*	 the landing targe*/
            public void position_valid_SET(byte src, Inside ph)
            {
                if(ph.field_bit != 242)insert_field(ph, 242, 0);
                BitUtils.set_bytes((ulong)(src), 1, data,  ph.BYTE);
            } static readonly Meta meta149 = new Meta(149, 0, 0, 1, 31, 238, 0, _ug, _Tg, _ag, _gg, _Bg);
        }/**
*Voltage and current sensor data*/
        public class SENS_POWER : Pack, CommunicationChannel.Sendable
        {
            internal SENS_POWER() : base(meta201, 0) { }

            public float adc121_vspb_volt //Power board voltage sensor reading in volts
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 0);}
            }

            public float adc121_cspb_amp //Power board current sensor reading in amps
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float adc121_cs1_amp //Board current sensor 1 reading in amps
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float adc121_cs2_amp //Board current sensor 2 reading in amps
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }
            static readonly Meta meta201 = new Meta(201, 0, 0, 0, 16, 128);
        }/**
*Maximum Power Point Tracker (MPPT) sensor data for solar module power performance tracking*/
        public class SENS_MPPT : Pack, CommunicationChannel.Sendable
        {
            internal SENS_MPPT() : base(meta202, 0) { }

            public ushort mppt1_pwm //MPPT1 pwm
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort mppt2_pwm //MPPT2 pwm
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort mppt3_pwm //MPPT3 pwm
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ulong mppt_timestamp //MPPT last timestamp
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  6);}
            }

            public float mppt1_volt //MPPT1 voltage
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float mppt1_amp //MPPT1 current
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public byte mppt1_status //MPPT1 status
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  22);}
            }

            public float mppt2_volt //MPPT2 voltage
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 23);}
            }

            public float mppt2_amp //MPPT2 current
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 27);}
            }

            public byte mppt2_status //MPPT2 status
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  31);}
            }

            public float mppt3_volt //MPPT3 voltage
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float mppt3_amp //MPPT3 current
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public byte mppt3_status //MPPT3 status
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  40);}
            }
            static readonly Meta meta202 = new Meta(202, 3, 0, 1, 41, 328);
        }/**
*ASL-fixed-wing controller data*/
        public class ASLCTRL_DATA : Pack, CommunicationChannel.Sendable
        {
            internal ASLCTRL_DATA() : base(meta203, 0) { }

            public ulong timestamp //Timestamp
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public byte aslctrl_mode //ASLCTRL control-mode (manual, stabilized, auto, etc...)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }

            public float h //See sourcecode for a description of these values...
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 9);}
            }

            public float hRef //null
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 13);}
            }

            public float hRef_t //null
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }

            public float PitchAngle //Pitch angle [deg]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 21);}
            }

            public float PitchAngleRef //Pitch angle reference[deg]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 25);}
            }

            public float q //null
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 29);}
            }

            public float qRef //null
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 33);}
            }

            public float uElev //null
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 37);}
            }

            public float uThrot //null
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 41);}
            }

            public float uThrot2 //null
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 45);}
            }

            public float nZ //null
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 49);}
            }

            public float AirspeedRef //Airspeed reference [m/s]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 53);}
            }

            public byte SpoilersEngaged //null
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  57);}
            }

            public float YawAngle //Yaw angle [deg]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 58);}
            }

            public float YawAngleRef //Yaw angle reference[deg]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 62);}
            }

            public float RollAngle //Roll angle [deg]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 66);}
            }

            public float RollAngleRef //Roll angle reference[deg]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 70);}
            }

            public float p //null
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 74);}
            }

            public float pRef //null
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 78);}
            }

            public float r //null
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 82);}
            }

            public float rRef //null
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 86);}
            }

            public float uAil //null
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 90);}
            }

            public float uRud //null
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 94);}
            }
            static readonly Meta meta203 = new Meta(203, 0, 0, 1, 98, 784);
        }/**
*ASL-fixed-wing controller debug data*/
        public class ASLCTRL_DEBUG : Pack, CommunicationChannel.Sendable
        {
            internal ASLCTRL_DEBUG() : base(meta204, 0) { }

            public uint i32_1 //Debug data
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte i8_1 //Debug data
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte i8_2 //Debug data
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public float f_1 //Debug data
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public float f_2 //Debug data
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float f_3 //Debug data
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float f_4 //Debug data
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float f_5 //Debug data
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            public float f_6 //Debug data
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 26);}
            }

            public float f_7 //Debug data
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public float f_8 //Debug data
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 34);}
            }
            static readonly Meta meta204 = new Meta(204, 0, 1, 0, 38, 304);
        }/**
*Extended state information for ASLUAVs*/
        public class ASLUAV_STATUS : Pack, CommunicationChannel.Sendable
        {
            internal ASLUAV_STATUS() : base(meta205, 0) { }

            public byte LED_status //Status of the position-indicator LEDs
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte SATCOM_status //Status of the IRIDIUM satellite communication system
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public byte[] Servo_status //Status vector for up to 8 servos
            {
                set {Servo_status_SET(value, 0)  ;}
            }
            public void Servo_status_SET(byte[] src, int pos)  //Status vector for up to 8 servos
            {
                for(int BYTE =  2, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            public float Motor_rpm //Motor RPM
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }
            static readonly Meta meta205 = new Meta(205, 0, 0, 0, 14, 112);
        }/**
*Extended EKF state estimates for ASLUAVs*/
        public class EKF_EXT : Pack, CommunicationChannel.Sendable
        {
            internal EKF_EXT() : base(meta206, 0) { }

            public ulong timestamp //Time since system start [us]
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float Windspeed //Magnitude of wind velocity (in lateral inertial plane) [m/s]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float WindDir //Wind heading angle from North [rad]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float WindZ //Z (Down) component of inertial wind velocity [m/s]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float Airspeed //Magnitude of air velocity [m/s]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float beta //Sideslip angle [rad]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float alpha //Angle of attack [rad]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
            static readonly Meta meta206 = new Meta(206, 0, 0, 1, 32, 256);
        }/**
*Off-board controls/commands for ASLUAVs*/
        public class ASL_OBCTRL : Pack, CommunicationChannel.Sendable
        {
            internal ASL_OBCTRL() : base(meta207, 0) { }

            public ulong timestamp //Time since system start [us]
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float uElev //Elevator command [~]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float uThrot //Throttle command [~]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float uThrot2 //Throttle 2 command [~]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float uAilL //Left aileron command [~]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float uAilR //Right aileron command [~]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float uRud //Rudder command [~]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public byte obctrl_status //Off-board computer status
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  32);}
            }
            static readonly Meta meta207 = new Meta(207, 0, 0, 1, 33, 264);
        }/**
*Atmospheric sensors (temperature, humidity, ...)*/
        public class SENS_ATMOS : Pack, CommunicationChannel.Sendable
        {
            internal SENS_ATMOS() : base(meta208, 0) { }

            public float TempAmbient //Ambient temperature [degrees Celsius]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 0);}
            }

            public float Humidity //Relative humidity [%]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }
            static readonly Meta meta208 = new Meta(208, 0, 0, 0, 8, 64);
        }/**
*Battery pack monitoring data for Li-Ion batteries*/
        public class SENS_BATMON : Pack, CommunicationChannel.Sendable
        {
            internal SENS_BATMON() : base(meta209, 0) { }

            public ushort voltage //Battery pack voltage in [mV]
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort batterystatus //Battery monitor status report bits in Hex
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort serialnumber //Battery monitor serial number in Hex
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort hostfetcontrol //Battery monitor sensor host FET control in Hex
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort cellvoltage1 //Battery pack cell 1 voltage in [mV]
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort cellvoltage2 //Battery pack cell 2 voltage in [mV]
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort cellvoltage3 //Battery pack cell 3 voltage in [mV]
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort cellvoltage4 //Battery pack cell 4 voltage in [mV]
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public ushort cellvoltage5 //Battery pack cell 5 voltage in [mV]
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  16);}
            }

            public ushort cellvoltage6 //Battery pack cell 6 voltage in [mV]
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  18);}
            }

            public float temperature //Battery pack temperature in [deg C]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public short current //Battery pack current in [mA]
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  24);}
            }

            public byte SoC //Battery pack state-of-charge
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  26);}
            }
            static readonly Meta meta209 = new Meta(209, 10, 0, 0, 27, 216);
        }/**
*Fixed-wing soaring (i.e. thermal seeking) data*/
        public class FW_SOARING_DATA : Pack, CommunicationChannel.Sendable
        {
            internal FW_SOARING_DATA() : base(meta210, 0) { }

            public ulong timestamp //Timestamp [ms]
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public ulong timestampModeChanged //Timestamp since last mode change[ms]
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public float xW //Thermal core updraft strength [m/s]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float xR //Thermal radius [m]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float xLat //Thermal center latitude [deg]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float xLon //Thermal center longitude [deg]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float VarW //Variance W
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float VarR //Variance R
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float VarLat //Variance Lat
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float VarLon //Variance Lon
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float LoiterRadius //Suggested loiter radius [m]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }

            public float LoiterDirection //Suggested loiter direction
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 52);}
            }

            public float DistToSoarPoint //Distance to soar point [m]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 56);}
            }

            public float vSinkExp //Expected sink rate at current airspeed, roll and throttle [m/s]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 60);}
            }

            public float z1_LocalUpdraftSpeed //Measurement / updraft speed at current/local airplane position [m/s]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 64);}
            }

            public float z2_DeltaRoll //Measurement / roll angle tracking error [deg]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 68);}
            }

            public float z1_exp //Expected measurement 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 72);}
            }

            public float z2_exp //Expected measurement 2
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 76);}
            }

            public float ThermalGSNorth //Thermal drift (from estimator prediction step only) [m/s]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 80);}
            }

            public float ThermalGSEast //Thermal drift (from estimator prediction step only) [m/s]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 84);}
            }

            public float TSE_dot //Total specific energy change (filtered) [m/s]
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 88);}
            }

            public float DebugVar1 //Debug variable 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 92);}
            }

            public float DebugVar2 //Debug variable 2
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 96);}
            }

            public byte ControlMode //Control Mode [-]
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  100);}
            }

            public byte valid //Data valid [-]
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  101);}
            }
            static readonly Meta meta210 = new Meta(210, 0, 0, 2, 102, 816);
        }/**
*Monitoring of sensorpod status*/
        public class SENSORPOD_STATUS : Pack, CommunicationChannel.Sendable
        {
            internal SENSORPOD_STATUS() : base(meta211, 0) { }

            public ushort free_space //Free space available in recordings directory in [Gb] * 1e2
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ulong timestamp //Timestamp in linuxtime [ms] (since 1.1.1970)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  2);}
            }

            public byte visensor_rate_1 //Rate of ROS topic 1
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  10);}
            }

            public byte visensor_rate_2 //Rate of ROS topic 2
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  11);}
            }

            public byte visensor_rate_3 //Rate of ROS topic 3
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  12);}
            }

            public byte visensor_rate_4 //Rate of ROS topic 4
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  13);}
            }

            public byte recording_nodes_count //Number of recording nodes
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  14);}
            }

            public byte cpu_temp //Temperature of sensorpod CPU in [deg C]
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  15);}
            }
            static readonly Meta meta211 = new Meta(211, 1, 0, 1, 16, 128);
        }/**
*Monitoring of power board status*/
        public class SENS_POWER_BOARD : Pack, CommunicationChannel.Sendable
        {
            internal SENS_POWER_BOARD() : base(meta212, 0) { }

            public ulong timestamp //Timestamp
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public byte pwr_brd_status //Power board status register
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }

            public byte pwr_brd_led_status //Power board leds status
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  9);}
            }

            public float pwr_brd_system_volt //Power board system voltage
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float pwr_brd_servo_volt //Power board servo voltage
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float pwr_brd_mot_l_amp //Power board left motor current sensor
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float pwr_brd_mot_r_amp //Power board right motor current sensor
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            public float pwr_brd_servo_1_amp //Power board servo1 current sensor
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 26);}
            }

            public float pwr_brd_servo_2_amp //Power board servo1 current sensor
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public float pwr_brd_servo_3_amp //Power board servo1 current sensor
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 34);}
            }

            public float pwr_brd_servo_4_amp //Power board servo1 current sensor
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 38);}
            }

            public float pwr_brd_aux_amp //Power board aux current sensor
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 42);}
            }
            static readonly Meta meta212 = new Meta(212, 0, 0, 1, 46, 368);
        }/**
*Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message
*	 is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS
*	 enum definition for further information. The innovaton test ratios show the magnitude of the sensor innovation
*	 divided by the innovation check threshold. Under normal operation the innovaton test ratios should be
*	 below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation
*	 and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation
*	 test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should
*	 be optional and controllable by the user*/
        public class ESTIMATOR_STATUS : Pack, CommunicationChannel.Sendable
        {
            internal ESTIMATOR_STATUS() : base(meta230, 0) { }

            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float vel_ratio //Velocity innovation test ratio
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float pos_horiz_ratio //Horizontal position innovation test ratio
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float pos_vert_ratio //Vertical position innovation test ratio
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float mag_ratio //Magnetometer innovation test ratio
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float hagl_ratio //Height above terrain innovation test ratio
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float tas_ratio //True airspeed innovation test ratio
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float pos_horiz_accuracy //Horizontal position 1-STD accuracy relative to the EKF local origin (m)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float pos_vert_accuracy //Vertical position 1-STD accuracy relative to the EKF local origin (m)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public ESTIMATOR_STATUS_FLAGS flags //Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS.
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 11, data, 320);}
            }
            static readonly Meta meta230 = new Meta(230, 0, 0, 1, 42, 331);
        } public class WIND_COV : Pack, CommunicationChannel.Sendable
        {
            internal WIND_COV() : base(meta231, 0) { }

            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float wind_x //Wind in X (NED) direction in m/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float wind_y //Wind in Y (NED) direction in m/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float wind_z //Wind in Z (NED) direction in m/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float var_horiz //Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float var_vert //Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float wind_alt //AMSL altitude (m) this measurement was taken at
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float horiz_accuracy //Horizontal speed 1-STD accuracy
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float vert_accuracy //Vertical speed 1-STD accuracy
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }
            static readonly Meta meta231 = new Meta(231, 0, 0, 1, 40, 320);
        }/**
*GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position
*	 estimate of the sytem*/
        public class GPS_INPUT : Pack, CommunicationChannel.Sendable
        {
            internal GPS_INPUT() : base(meta232, 0) { }

            public ushort time_week //GPS week number
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_week_ms //GPS time (milliseconds from start of GPS week)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  6);}
            }

            public byte gps_id //ID of the GPS for multiple GPS inputs
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  14);}
            }

            public byte fix_type //0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  15);}
            }

            public int lat //Latitude (WGS84), in degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  16);}
            }

            public int lon //Longitude (WGS84), in degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  20);}
            }

            public float alt //Altitude (AMSL, not WGS84), in m (positive for up)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float hdop //GPS HDOP horizontal dilution of position in m
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float vdop //GPS VDOP vertical dilution of position in m
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float vn //GPS velocity in m/s in NORTH direction in earth-fixed NED frame
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float ve //GPS velocity in m/s in EAST direction in earth-fixed NED frame
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float vd //GPS velocity in m/s in DOWN direction in earth-fixed NED frame
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float speed_accuracy //GPS speed accuracy in m/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }

            public float horiz_accuracy //GPS horizontal accuracy in m
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 52);}
            }

            public float vert_accuracy //GPS vertical accuracy in m
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 56);}
            }

            public byte satellites_visible //Number of satellites visible.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  60);}
            }

            public GPS_INPUT_IGNORE_FLAGS ignore_flags //Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 8, data, 488);}
            }
            static readonly Meta meta232 = new Meta(232, 1, 1, 1, 62, 496);
        }/**
*RTCM message for injecting into the onboard GPS (used for DGPS)*/
        public class GPS_RTCM_DATA : Pack, CommunicationChannel.Sendable
        {
            internal GPS_RTCM_DATA() : base(meta233, 0) { }

            /**
            *LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for
            *	 the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
            *	 on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
            *	 while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
            *	 fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
            *	 with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
            *	 corrupt RTCM data, and to recover from a unreliable transport delivery order*/
            public byte flags
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte len //data length
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public byte[] data_ //RTCM message (may be fragmented)
            {
                set {data__SET(value, 0)  ;}
            }
            public void data__SET(byte[] src, int pos)  //RTCM message (may be fragmented)
            {
                for(int BYTE =  2, src_max = pos + 180; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            static readonly Meta meta233 = new Meta(233, 0, 0, 0, 182, 1456);
        }/**
*Message appropriate for high latency connections like Iridium*/
        public class HIGH_LATENCY : Pack, CommunicationChannel.Sendable
        {
            internal HIGH_LATENCY() : base(meta234, 0) { }

            public ushort heading //heading (centidegrees)
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort wp_distance //distance to target (meters)
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public uint custom_mode //A bitfield for use for autopilot-specific flags.
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  4);}
            }

            public short roll //roll (centidegrees)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  8);}
            }

            public short pitch //pitch (centidegrees)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public sbyte throttle //throttle (percentage)
            {
                set {  BitUtils.set_bytes((byte)(value), 1, data,  12);}
            }

            public short heading_sp //heading setpoint (centidegrees)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  13);}
            }

            public int latitude //Latitude, expressed as degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  15);}
            }

            public int longitude //Longitude, expressed as degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  19);}
            }

            public short altitude_amsl //Altitude above mean sea level (meters)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  23);}
            }

            public short altitude_sp //Altitude setpoint relative to the home position (meters)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  25);}
            }

            public byte airspeed //airspeed (m/s)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  27);}
            }

            public byte airspeed_sp //airspeed setpoint (m/s)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  28);}
            }

            public byte groundspeed //groundspeed (m/s)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  29);}
            }

            public sbyte climb_rate //climb rate (m/s)
            {
                set {  BitUtils.set_bytes((byte)(value), 1, data,  30);}
            }

            public byte gps_nsat //Number of satellites visible. If unknown, set to 255
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  31);}
            }

            public byte battery_remaining //Remaining battery (percentage)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  32);}
            }

            public sbyte temperature //Autopilot temperature (degrees C)
            {
                set {  BitUtils.set_bytes((byte)(value), 1, data,  33);}
            }

            public sbyte temperature_air //Air temperature (degrees C) from airspeed sensor
            {
                set {  BitUtils.set_bytes((byte)(value), 1, data,  34);}
            }

            /**
            *failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS,
            *	 bit3:GCS, bit4:fence*/
            public byte failsafe
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  35);}
            }

            public byte wp_num //current waypoint number
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  36);}
            }

            public MAV_MODE_FLAG base_mode //System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 8, data, 296);}
            }

            public MAV_LANDED_STATE landed_state //The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 304);}
            }

            public GPS_FIX_TYPE gps_fix_type //See the GPS_FIX_TYPE enum.
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 307);}
            }
            static readonly Meta meta234 = new Meta(234, 2, 1, 0, 39, 311);
        }/**
*Vibration levels and accelerometer clipping*/
        public class VIBRATION : Pack, CommunicationChannel.Sendable
        {
            internal VIBRATION() : base(meta241, 0) { }

            public uint clipping_0 //first accelerometer clipping count
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public uint clipping_1 //second accelerometer clipping count
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  4);}
            }

            public uint clipping_2 //third accelerometer clipping count
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  8);}
            }

            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  12);}
            }

            public float vibration_x //Vibration levels on X-axis
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float vibration_y //Vibration levels on Y-axis
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float vibration_z //Vibration levels on Z-axis
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
            static readonly Meta meta241 = new Meta(241, 0, 3, 1, 32, 256);
        }/**
*This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system
*	 will return to and land on. The position is set automatically by the system during the takeoff in case
*	 it was not explicitely set by the operator before or after. The position the system will return to and
*	 land on. The global and local positions encode the position in the respective coordinate frames, while
*	 the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading
*	 and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes
*	 the point to which the system should fly in normal flight mode and then perform a landing sequence along
*	 the vector*/
        public class HOME_POSITION : Pack, CommunicationChannel.Sendable
        {
            internal HOME_POSITION() : base(meta242, 0) { }

            public int latitude //Latitude (WGS84), in degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  0);}
            }

            public int longitude //Longitude (WGS84, in degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  4);}
            }

            public int altitude //Altitude (AMSL), in meters * 1000 (positive for up)
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  8);}
            }

            public float x //Local X position of this position in the local coordinate frame
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float y //Local Y position of this position in the local coordinate frame
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float z //Local Z position of this position in the local coordinate frame
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            /**
            *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
            *	 and slope of the groun*/
            public float[] q
            {
                set {q_SET(value, 0)  ;}
            }
            /**
            *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
            *	 and slope of the groun*/
            public void q_SET(float[] src, int pos)
            {
                for(int BYTE =  24, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            /**
            *Local X position of the end of the approach vector. Multicopters should set this position based on their
            *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	 from the threshold / touchdown zone*/
            public float approach_x
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            /**
            *Local Y position of the end of the approach vector. Multicopters should set this position based on their
            *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	 from the threshold / touchdown zone*/
            public float approach_y
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            /**
            *Local Z position of the end of the approach vector. Multicopters should set this position based on their
            *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	 from the threshold / touchdown zone*/
            public float approach_z
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }
            public void time_usec_SET(ulong src, Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                if(ph.field_bit != 416)insert_field(ph, 416, 0);
                BitUtils.set_bytes((ulong)(src), 8, data,  ph.BYTE);
            } static readonly Meta meta242 = new Meta(242, 0, 0, 0, 53, 416, 0, _Uc);
        }/**
*The position the system will return to and land on. The position is set automatically by the system during
*	 the takeoff in case it was not explicitely set by the operator before or after. The global and local
*	 positions encode the position in the respective coordinate frames, while the q parameter encodes the
*	 orientation of the surface. Under normal conditions it describes the heading and terrain slope, which
*	 can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which
*	 the system should fly in normal flight mode and then perform a landing sequence along the vector*/
        public class SET_HOME_POSITION : Pack, CommunicationChannel.Sendable
        {
            internal SET_HOME_POSITION() : base(meta243, 0) { }

            public byte target_system //System ID.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public int latitude //Latitude (WGS84), in degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  1);}
            }

            public int longitude //Longitude (WGS84, in degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  5);}
            }

            public int altitude //Altitude (AMSL), in meters * 1000 (positive for up)
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  9);}
            }

            public float x //Local X position of this position in the local coordinate frame
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 13);}
            }

            public float y //Local Y position of this position in the local coordinate frame
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }

            public float z //Local Z position of this position in the local coordinate frame
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 21);}
            }

            /**
            *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
            *	 and slope of the groun*/
            public float[] q
            {
                set {q_SET(value, 0)  ;}
            }
            /**
            *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
            *	 and slope of the groun*/
            public void q_SET(float[] src, int pos)
            {
                for(int BYTE =  25, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            /**
            *Local X position of the end of the approach vector. Multicopters should set this position based on their
            *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	 from the threshold / touchdown zone*/
            public float approach_x
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 41);}
            }

            /**
            *Local Y position of the end of the approach vector. Multicopters should set this position based on their
            *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	 from the threshold / touchdown zone*/
            public float approach_y
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 45);}
            }

            /**
            *Local Z position of the end of the approach vector. Multicopters should set this position based on their
            *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	 from the threshold / touchdown zone*/
            public float approach_z
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 49);}
            }
            public void time_usec_SET(ulong src, Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                if(ph.field_bit != 424)insert_field(ph, 424, 0);
                BitUtils.set_bytes((ulong)(src), 8, data,  ph.BYTE);
            } static readonly Meta meta243 = new Meta(243, 0, 0, 0, 54, 424, 0, _VB);
        }/**
*This interface replaces DATA_STREAM*/
        public class MESSAGE_INTERVAL : Pack, CommunicationChannel.Sendable
        {
            internal MESSAGE_INTERVAL() : base(meta244, 0) { }

            public ushort message_id //The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public int interval_us //0 indicates the interval at which it is sent.
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  2);}
            }
            static readonly Meta meta244 = new Meta(244, 1, 0, 0, 6, 48);
        }/**
*Provides state for additional features*/
        public class EXTENDED_SYS_STATE : Pack, CommunicationChannel.Sendable
        {
            internal EXTENDED_SYS_STATE() : base(meta245, 0) { }

            public MAV_VTOL_STATE vtol_state //The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 0);}
            }

            public MAV_LANDED_STATE landed_state //The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 3);}
            }
            static readonly Meta meta245 = new Meta(245, 0, 0, 0, 1, 6);
        }/**
*The location and information of an ADSB vehicle*/
        public class ADSB_VEHICLE : Pack, CommunicationChannel.Sendable
        {
            internal ADSB_VEHICLE() : base(meta246, 0) { }

            public ushort heading //Course over ground in centidegrees
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort hor_velocity //The horizontal velocity in centimeters/second
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort squawk //Squawk code
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public uint ICAO_address //ICAO address
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public int lat //Latitude, expressed as degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  10);}
            }

            public int lon //Longitude, expressed as degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  14);}
            }

            public int altitude //Altitude(ASL) in millimeters
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  18);}
            }

            public short ver_velocity //The vertical velocity in centimeters/second, positive is up
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  22);}
            }

            public byte tslc //Time since last communication in seconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  24);}
            }

            public ADSB_ALTITUDE_TYPE altitude_type //Type from ADSB_ALTITUDE_TYPE enum
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 1, data, 200);}
            }

            public ADSB_EMITTER_TYPE emitter_type //Type from ADSB_EMITTER_TYPE enum
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 5, data, 201);}
            }

            public ADSB_FLAGS flags //Flags to indicate various statuses including valid data fields
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 7, data, 206);}
            }
            public void callsign_SET(string src, Inside ph)//The callsign, 8+null
            {callsign_SET(src.ToCharArray(), 0, src.Length, ph);} public void callsign_SET(char[] src, int pos, int items, Inside ph) //The callsign, 8+null
            {
                if(ph.field_bit != 213 && insert_field(ph, 213, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta246 = new Meta(246, 3, 1, 0, 28, 213, 0, _hB);
        }/**
*Information about a potential collision*/
        public class COLLISION : Pack, CommunicationChannel.Sendable
        {
            internal COLLISION() : base(meta247, 0) { }

            public uint id //Unique identifier, domain based on src field
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float time_to_minimum_delta //Estimated time until collision occurs (seconds)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float altitude_minimum_delta //Closest vertical distance in meters between vehicle and object
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float horizontal_minimum_delta //Closest horizontal distance in meteres between vehicle and object
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public MAV_COLLISION_SRC src_ //Collision data source
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 1, data, 128);}
            }

            public MAV_COLLISION_ACTION action //Action that is being taken to avoid this collision
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 129);}
            }

            public MAV_COLLISION_THREAT_LEVEL threat_level //How concerned the aircraft is about this collision
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 132);}
            }
            static readonly Meta meta247 = new Meta(247, 0, 1, 0, 17, 135);
        }/**
*Message implementing parts of the V2 payload specs in V1 frames for transitional support.*/
        public class V2_EXTENSION : Pack, CommunicationChannel.Sendable
        {
            internal V2_EXTENSION() : base(meta248, 0) { }

            /**
            *A code that identifies the software component that understands this message (analogous to usb device classes
            *	 or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
            *	 and the corresponding entry should be added to https:github.com/mavlink/mavlink/extension-message-ids.xml.
            *	 Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
            *	 Message_types greater than 32767 are considered local experiments and should not be checked in to any
            *	 widely distributed codebase*/
            public ushort message_type
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_network //Network ID (0 for broadcast)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_system //System ID (0 for broadcast)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public byte target_component //Component ID (0 for broadcast)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            /**
            *Variable length payload. The length is defined by the remaining message length when subtracting the header
            *	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
            *	 message_type.  The particular encoding used can be extension specific and might not always be documented
            *	 as part of the mavlink specification*/
            public byte[] payload
            {
                set {payload_SET(value, 0)  ;}
            }
            /**
            *Variable length payload. The length is defined by the remaining message length when subtracting the header
            *	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
            *	 message_type.  The particular encoding used can be extension specific and might not always be documented
            *	 as part of the mavlink specification*/
            public void payload_SET(byte[] src, int pos)
            {
                for(int BYTE =  5, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            static readonly Meta meta248 = new Meta(248, 1, 0, 0, 254, 2032);
        }/**
*Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient
*	 way for testing new messages and getting experimental debug output*/
        public class MEMORY_VECT : Pack, CommunicationChannel.Sendable
        {
            internal MEMORY_VECT() : base(meta249, 0) { }

            public ushort address //Starting address of the debug variables
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte ver //Version code of the type variable. 0=unknown, type ignored and assumed short. 1=as below
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte type //Type code of the memory variables. for ver = 1: 0=16 x short, 1=16 x ushort, 2=16 x Q15, 3=16 x 1Q1
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public sbyte[] value //Memory contents at specified address
            {
                set {value_SET(value, 0)  ;}
            }
            public void value_SET(sbyte[] src, int pos)  //Memory contents at specified address
            {
                for(int BYTE =  4, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((byte)(src[pos]), 1, data,  BYTE);
            }
            static readonly Meta meta249 = new Meta(249, 1, 0, 0, 36, 288);
        } public class DEBUG_VECT : Pack, CommunicationChannel.Sendable
        {
            internal DEBUG_VECT() : base(meta250, 0) { }

            public ulong time_usec //Timestamp
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x //x
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //y
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //z
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }
            public void name_SET(string src, Inside ph)//Name
            {name_SET(src.ToCharArray(), 0, src.Length, ph);} public void name_SET(char[] src, int pos, int items, Inside ph) //Name
            {
                if(ph.field_bit != 160 && insert_field(ph, 160, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta250 = new Meta(250, 0, 0, 1, 21, 160, 0, _NB);
        }/**
*Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite
*	 efficient way for testing new messages and getting experimental debug output*/
        public class NAMED_VALUE_FLOAT : Pack, CommunicationChannel.Sendable
        {
            internal NAMED_VALUE_FLOAT() : base(meta251, 0) { }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float value //Floating point value
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }
            public void name_SET(string src, Inside ph)//Name of the debug variable
            {name_SET(src.ToCharArray(), 0, src.Length, ph);} public void name_SET(char[] src, int pos, int items, Inside ph) //Name of the debug variable
            {
                if(ph.field_bit != 64 && insert_field(ph, 64, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta251 = new Meta(251, 0, 1, 0, 9, 64, 0, _UB);
        }/**
*Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite
*	 efficient way for testing new messages and getting experimental debug output*/
        public class NAMED_VALUE_INT : Pack, CommunicationChannel.Sendable
        {
            internal NAMED_VALUE_INT() : base(meta252, 0) { }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public int value //Signed integer value
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  4);}
            }
            public void name_SET(string src, Inside ph)//Name of the debug variable
            {name_SET(src.ToCharArray(), 0, src.Length, ph);} public void name_SET(char[] src, int pos, int items, Inside ph) //Name of the debug variable
            {
                if(ph.field_bit != 64 && insert_field(ph, 64, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta252 = new Meta(252, 0, 1, 0, 9, 64, 0, _mL);
        }/**
*Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING:
*	 They consume quite some bandwidth, so use only for important status and error messages. If implemented
*	 wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz)*/
        public class STATUSTEXT : Pack, CommunicationChannel.Sendable
        {
            internal STATUSTEXT() : base(meta253, 0) { }

            public MAV_SEVERITY severity //Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 0);}
            }
            public void text_SET(string src, Inside ph)//Status text message, without null termination character
            {text_SET(src.ToCharArray(), 0, src.Length, ph);} public void text_SET(char[] src, int pos, int items, Inside ph) //Status text message, without null termination character
            {
                if(ph.field_bit != 3 && insert_field(ph, 3, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta253 = new Meta(253, 0, 0, 0, 2, 3, 0, _KL);
        }/**
*Send a debug value. The index is used to discriminate between values. These values show up in the plot
*	 of QGroundControl as DEBUG N*/
        public class DEBUG : Pack, CommunicationChannel.Sendable
        {
            internal DEBUG() : base(meta254, 0) { }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte ind //index of debug variable
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public float value //DEBUG value
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 5);}
            }
            static readonly Meta meta254 = new Meta(254, 0, 1, 0, 9, 72);
        }/**
*Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will disable
*	 signin*/
        public class SETUP_SIGNING : Pack, CommunicationChannel.Sendable
        {
            internal SETUP_SIGNING() : base(meta256, 0) { }

            public ulong initial_timestamp //initial timestamp
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public byte target_system //system id of the target
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }

            public byte target_component //component ID of the target
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  9);}
            }

            public byte[] secret_key //signing key
            {
                set {secret_key_SET(value, 0)  ;}
            }
            public void secret_key_SET(byte[] src, int pos)  //signing key
            {
                for(int BYTE =  10, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            static readonly Meta meta256 = new Meta(256, 0, 0, 1, 42, 336);
        }/**
*Report button state change*/
        public class BUTTON_CHANGE : Pack, CommunicationChannel.Sendable
        {
            internal BUTTON_CHANGE() : base(meta257, 0) { }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public uint last_change_ms //Time of last change of button state
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  4);}
            }

            public byte state //Bitmap state of buttons
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }
            static readonly Meta meta257 = new Meta(257, 0, 2, 0, 9, 72);
        }/**
*Control vehicle tone generation (buzzer)*/
        public class PLAY_TUNE : Pack, CommunicationChannel.Sendable
        {
            internal PLAY_TUNE() : base(meta258, 0) { }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }
            public void tune_SET(string src, Inside ph)//tune in board specific format
            {tune_SET(src.ToCharArray(), 0, src.Length, ph);} public void tune_SET(char[] src, int pos, int items, Inside ph) //tune in board specific format
            {
                if(ph.field_bit != 16 && insert_field(ph, 16, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta258 = new Meta(258, 0, 0, 0, 3, 16, 0, _aL);
        }/**
*WIP: Information about a camera*/
        public class CAMERA_INFORMATION : Pack, CommunicationChannel.Sendable
        {
            internal CAMERA_INFORMATION() : base(meta259, 0) { }

            public ushort resolution_h //Image resolution in pixels horizontal
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort resolution_v //Image resolution in pixels vertical
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort cam_definition_version //Camera definition version (iteration)
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public uint firmware_version //0xff = Major)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  10);}
            }

            public byte[] vendor_name //Name of the camera vendor
            {
                set {vendor_name_SET(value, 0)  ;}
            }
            public void vendor_name_SET(byte[] src, int pos)  //Name of the camera vendor
            {
                for(int BYTE =  14, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            public byte[] model_name //Name of the camera model
            {
                set {model_name_SET(value, 0)  ;}
            }
            public void model_name_SET(byte[] src, int pos)  //Name of the camera model
            {
                for(int BYTE =  46, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            public float focal_length //Focal length in mm
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 78);}
            }

            public float sensor_size_h //Image sensor size horizontal in mm
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 82);}
            }

            public float sensor_size_v //Image sensor size vertical in mm
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 86);}
            }

            public byte lens_id //Reserved for a lens ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  90);}
            }

            public CAMERA_CAP_FLAGS flags //CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities.
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 6, data, 728);}
            }
            public void cam_definition_uri_SET(string src, Inside ph)//Camera definition URI (if any, otherwise only basic functions will be available).
            {cam_definition_uri_SET(src.ToCharArray(), 0, src.Length, ph);} public void cam_definition_uri_SET(char[] src, int pos, int items, Inside ph) //Camera definition URI (if any, otherwise only basic functions will be available).
            {
                if(ph.field_bit != 734 && insert_field(ph, 734, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta259 = new Meta(259, 3, 2, 0, 93, 734, 0, _iL);
        }/**
*WIP: Settings of a camera, can be requested using MAV_CMD_REQUEST_CAMERA_SETTINGS.*/
        public class CAMERA_SETTINGS : Pack, CommunicationChannel.Sendable
        {
            internal CAMERA_SETTINGS() : base(meta260, 0) { }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public CAMERA_MODE mode_id //Camera mode (CAMERA_MODE)
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 32);}
            }
            static readonly Meta meta260 = new Meta(260, 0, 1, 0, 5, 35);
        }/**
*WIP: Information about a storage medium.*/
        public class STORAGE_INFORMATION : Pack, CommunicationChannel.Sendable
        {
            internal STORAGE_INFORMATION() : base(meta261, 0) { }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte storage_id //Storage ID (1 for first, 2 for second, etc.)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte storage_count //Number of storage devices
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public byte status //Status of storage (0 not available, 1 unformatted, 2 formatted)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public float total_capacity //Total capacity in MiB
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 7);}
            }

            public float used_capacity //Used capacity in MiB
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 11);}
            }

            public float available_capacity //Available capacity in MiB
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 15);}
            }

            public float read_speed //Read speed in MiB/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 19);}
            }

            public float write_speed //Write speed in MiB/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 23);}
            }
            static readonly Meta meta261 = new Meta(261, 0, 1, 0, 27, 216);
        }/**
*WIP: Information about the status of a capture*/
        public class CAMERA_CAPTURE_STATUS : Pack, CommunicationChannel.Sendable
        {
            internal CAMERA_CAPTURE_STATUS() : base(meta262, 0) { }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public uint recording_time_ms //Time in milliseconds since recording started
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  4);}
            }

            /**
            *Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval
            *	 set and capture in progress*/
            public byte image_status
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }

            public byte video_status //Current status of video capturing (0: idle, 1: capture in progress)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  9);}
            }

            public float image_interval //Image capture interval in seconds
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float available_capacity //Available storage capacity in MiB
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }
            static readonly Meta meta262 = new Meta(262, 0, 2, 0, 18, 144);
        }/**
*Information about a captured image*/
        public class CAMERA_IMAGE_CAPTURED : Pack, CommunicationChannel.Sendable
        {
            internal CAMERA_IMAGE_CAPTURED() : base(meta263, 0) { }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public ulong time_utc //Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  4);}
            }

            public byte camera_id //Camera ID (1 for first, 2 for second, etc.)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  12);}
            }

            public int lat //Latitude, expressed as degrees * 1E7 where image was taken
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  13);}
            }

            public int lon //Longitude, expressed as degrees * 1E7 where capture was taken
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  17);}
            }

            public int alt //Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  21);}
            }

            public int relative_alt //Altitude above ground in meters, expressed as * 1E3 where image was taken
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  25);}
            }

            public float[] q //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
            {
                set {q_SET(value, 0)  ;}
            }
            public void q_SET(float[] src, int pos)  //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
            {
                for(int BYTE =  29, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            public int image_index //Zero based index of this image (image count since armed -1)
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  45);}
            }

            public sbyte capture_result //Boolean indicating success (1) or failure (0) while capturing this image.
            {
                set {  BitUtils.set_bytes((byte)(value), 1, data,  49);}
            }
            public void file_url_SET(string src, Inside ph)//URL of image taken. Either local storage or http:foo.jpg if camera provides an HTTP interface.
            {file_url_SET(src.ToCharArray(), 0, src.Length, ph);} public void file_url_SET(char[] src, int pos, int items, Inside ph) //URL of image taken. Either local storage or http:foo.jpg if camera provides an HTTP interface.
            {
                if(ph.field_bit != 402 && insert_field(ph, 402, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta263 = new Meta(263, 0, 1, 1, 51, 402, 2, _ex);
        }/**
*WIP: Information about flight since last arming*/
        public class FLIGHT_INFORMATION : Pack, CommunicationChannel.Sendable
        {
            internal FLIGHT_INFORMATION() : base(meta264, 0) { }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public ulong arming_time_utc //Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  4);}
            }

            public ulong takeoff_time_utc //Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  12);}
            }

            public ulong flight_uuid //Universally unique identifier (UUID) of flight, should correspond to name of logfiles
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  20);}
            }
            static readonly Meta meta264 = new Meta(264, 0, 1, 3, 28, 224);
        }/**
*WIP: Orientation of a mount*/
        public class MOUNT_ORIENTATION : Pack, CommunicationChannel.Sendable
        {
            internal MOUNT_ORIENTATION() : base(meta265, 0) { }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float roll //Roll in degrees
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float pitch //Pitch in degrees
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float yaw //Yaw in degrees
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }
            static readonly Meta meta265 = new Meta(265, 0, 1, 0, 16, 128);
        }/**
*A message containing logged data (see also MAV_CMD_LOGGING_START)*/
        public class LOGGING_DATA : Pack, CommunicationChannel.Sendable
        {
            internal LOGGING_DATA() : base(meta266, 0) { }

            public ushort sequence //sequence number (can wrap)
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //system ID of the target
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //component ID of the target
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public byte length //data length
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            /**
            *offset into data where first message starts. This can be used for recovery, when a previous message got
            *	 lost (set to 255 if no start exists)*/
            public byte first_message_offset
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public byte[] data_ //logged data
            {
                set {data__SET(value, 0)  ;}
            }
            public void data__SET(byte[] src, int pos)  //logged data
            {
                for(int BYTE =  6, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            static readonly Meta meta266 = new Meta(266, 1, 0, 0, 255, 2040);
        }/**
*A message containing logged data which requires a LOGGING_ACK to be sent back*/
        public class LOGGING_DATA_ACKED : Pack, CommunicationChannel.Sendable
        {
            internal LOGGING_DATA_ACKED() : base(meta267, 0) { }

            public ushort sequence //sequence number (can wrap)
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //system ID of the target
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //component ID of the target
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public byte length //data length
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            /**
            *offset into data where first message starts. This can be used for recovery, when a previous message got
            *	 lost (set to 255 if no start exists)*/
            public byte first_message_offset
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public byte[] data_ //logged data
            {
                set {data__SET(value, 0)  ;}
            }
            public void data__SET(byte[] src, int pos)  //logged data
            {
                for(int BYTE =  6, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            static readonly Meta meta267 = new Meta(267, 1, 0, 0, 255, 2040);
        }/**
*An ack for a LOGGING_DATA_ACKED message*/
        public class LOGGING_ACK : Pack, CommunicationChannel.Sendable
        {
            internal LOGGING_ACK() : base(meta268, 0) { }

            public ushort sequence //sequence number (must match the one in LOGGING_DATA_ACKED)
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //system ID of the target
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //component ID of the target
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }
            static readonly Meta meta268 = new Meta(268, 1, 0, 0, 4, 32);
        }/**
*WIP: Information about video stream*/
        public class VIDEO_STREAM_INFORMATION : Pack, CommunicationChannel.Sendable
        {
            internal VIDEO_STREAM_INFORMATION() : base(meta269, 0) { }

            public ushort resolution_h //Resolution horizontal in pixels
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort resolution_v //Resolution vertical in pixels
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort rotation //Video image rotation clockwise
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public uint bitrate //Bit rate in bits per second
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public byte camera_id //Camera ID (1 for first, 2 for second, etc.)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  10);}
            }

            public byte status //Current status of video streaming (0: not running, 1: in progress)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  11);}
            }

            public float framerate //Frames per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }
            public void uri_SET(string src, Inside ph)//Video stream URI
            {uri_SET(src.ToCharArray(), 0, src.Length, ph);} public void uri_SET(char[] src, int pos, int items, Inside ph) //Video stream URI
            {
                if(ph.field_bit != 130 && insert_field(ph, 130, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta269 = new Meta(269, 3, 1, 0, 17, 130, 2, _Ax);
        }/**
*WIP: Message that sets video stream settings*/
        public class SET_VIDEO_STREAM_SETTINGS : Pack, CommunicationChannel.Sendable
        {
            internal SET_VIDEO_STREAM_SETTINGS() : base(meta270, 0) { }

            public ushort resolution_h //Resolution horizontal in pixels (set to -1 for highest resolution possible)
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort resolution_v //Resolution vertical in pixels (set to -1 for highest resolution possible)
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort rotation //Video image rotation clockwise (0-359 degrees)
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public uint bitrate //Bit rate in bits per second (set to -1 for auto)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public byte target_system //system ID of the target
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  10);}
            }

            public byte target_component //component ID of the target
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  11);}
            }

            public byte camera_id //Camera ID (1 for first, 2 for second, etc.)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  12);}
            }

            public float framerate //Frames per second (set to -1 for highest framerate possible)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 13);}
            }
            public void uri_SET(string src, Inside ph)//Video stream URI
            {uri_SET(src.ToCharArray(), 0, src.Length, ph);} public void uri_SET(char[] src, int pos, int items, Inside ph) //Video stream URI
            {
                if(ph.field_bit != 138 && insert_field(ph, 138, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta270 = new Meta(270, 3, 1, 0, 18, 138, 2, _ox);
        }/**
*Configure AP SSID and Password.*/
        public class WIFI_CONFIG_AP : Pack, CommunicationChannel.Sendable
        {
            internal WIFI_CONFIG_AP() : base(meta299, 0) { }
            public void ssid_SET(string src, Inside ph)//Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
            {ssid_SET(src.ToCharArray(), 0, src.Length, ph);} public void ssid_SET(char[] src, int pos, int items, Inside ph) //Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
            {
                if(ph.field_bit != 2 && insert_field(ph, 2, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } public void password_SET(string src, Inside ph) //Password. Leave it blank for an open AP.
            {password_SET(src.ToCharArray(), 0, src.Length, ph);} public void password_SET(char[] src, int pos, int items, Inside ph) //Password. Leave it blank for an open AP.
            {
                if(ph.field_bit != 3 && insert_field(ph, 3, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta299 = new Meta(299, 0, 0, 0, 1, 2, 2, _lx, _Cx);
        }/**
*WIP: Version and capability of protocol version. This message is the response to REQUEST_PROTOCOL_VERSION
*	 and is used as part of the handshaking to establish which MAVLink version should be used on the network.
*	 Every node should respond to REQUEST_PROTOCOL_VERSION to enable the handshaking. Library implementers
*	 should consider adding this into the default decoding state machine to allow the protocol core to respond
*	 directly*/
        public class PROTOCOL_VERSION : Pack, CommunicationChannel.Sendable
        {
            internal PROTOCOL_VERSION() : base(meta300, 0) { }

            public ushort version //Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort min_version //Minimum MAVLink version supported
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort max_version //Maximum MAVLink version supported (set to the same value as version by default)
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public byte[] spec_version_hash //The first 8 bytes (not characters printed in hex!) of the git hash.
            {
                set {spec_version_hash_SET(value, 0)  ;}
            }
            public void spec_version_hash_SET(byte[] src, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash.
            {
                for(int BYTE =  6, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            public byte[] library_version_hash //The first 8 bytes (not characters printed in hex!) of the git hash.
            {
                set {library_version_hash_SET(value, 0)  ;}
            }
            public void library_version_hash_SET(byte[] src, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash.
            {
                for(int BYTE =  14, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            static readonly Meta meta300 = new Meta(300, 3, 0, 0, 22, 176);
        }/**
*General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message "uavcan.protocol.NodeStatus"
*	 for the background information. The UAVCAN specification is available at http:uavcan.org*/
        public class UAVCAN_NODE_STATUS : Pack, CommunicationChannel.Sendable
        {
            internal UAVCAN_NODE_STATUS() : base(meta310, 0) { }

            public ushort vendor_specific_status_code //Vendor-specific status information.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint uptime_sec //The number of seconds since the start-up of the node.
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  6);}
            }

            public byte sub_mode //Not used currently.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  14);}
            }

            public UAVCAN_NODE_HEALTH health //Generalized node health status.
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 2, data, 120);}
            }

            public UAVCAN_NODE_MODE mode //Generalized operating mode.
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 122);
                }
            }
            static readonly Meta meta310 = new Meta(310, 1, 1, 1, 16, 125);
        }/**
*General information describing a particular UAVCAN node. Please refer to the definition of the UAVCAN
*	 service "uavcan.protocol.GetNodeInfo" for the background information. This message should be emitted
*	 by the system whenever a new node appears online, or an existing node reboots. Additionally, it can be
*	 emitted upon request from the other end of the MAVLink channel (see MAV_CMD_UAVCAN_GET_NODE_INFO). It
*	 is also not prohibited to emit this message unconditionally at a low frequency. The UAVCAN specification
*	 is available at http:uavcan.org*/
        public class UAVCAN_NODE_INFO : Pack, CommunicationChannel.Sendable
        {
            internal UAVCAN_NODE_INFO() : base(meta311, 0) { }

            public uint uptime_sec //The number of seconds since the start-up of the node.
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public uint sw_vcs_commit //Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  4);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public byte hw_version_major //Hardware major version number.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            public byte hw_version_minor //Hardware minor version number.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  17);}
            }

            public byte[] hw_unique_id //Hardware unique 128-bit ID.
            {
                set {hw_unique_id_SET(value, 0)  ;}
            }
            public void hw_unique_id_SET(byte[] src, int pos)  //Hardware unique 128-bit ID.
            {
                for(int BYTE =  18, src_max = pos + 16; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            public byte sw_version_major //Software major version number.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  34);}
            }

            public byte sw_version_minor //Software minor version number.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  35);}
            }
            public void name_SET(string src, Inside ph)//Node name string. For example, "sapog.px4.io".
            {name_SET(src.ToCharArray(), 0, src.Length, ph);} public void name_SET(char[] src, int pos, int items, Inside ph) //Node name string. For example, "sapog.px4.io".
            {
                if(ph.field_bit != 288 && insert_field(ph, 288, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta311 = new Meta(311, 0, 2, 1, 37, 288, 0, _Vh);
        }/**
*Request to read the value of a parameter with the either the param_id string id or param_index.*/
        public class PARAM_EXT_REQUEST_READ : Pack, CommunicationChannel.Sendable
        {
            internal PARAM_EXT_REQUEST_READ() : base(meta320, 0) { }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public short param_index //Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  2);}
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	 ID is stored as strin*/
            public void param_id_SET(string src, Inside ph)
            {param_id_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	 ID is stored as strin*/
            public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 32 && insert_field(ph, 32, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta320 = new Meta(320, 0, 0, 0, 5, 32, 0, _ch);
        }/**
*Request all parameters of this component. After this request, all parameters are emitted.*/
        public class PARAM_EXT_REQUEST_LIST : Pack, CommunicationChannel.Sendable
        {
            internal PARAM_EXT_REQUEST_LIST() : base(meta321, 0) { }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }
            static readonly Meta meta321 = new Meta(321, 0, 0, 0, 2, 16);
        }/**
*Emit the value of a parameter. The inclusion of param_count and param_index in the message allows the
*	 recipient to keep track of received parameters and allows them to re-request missing parameters after
*	 a loss or timeout*/
        public class PARAM_EXT_VALUE : Pack, CommunicationChannel.Sendable
        {
            internal PARAM_EXT_VALUE() : base(meta322, 0) { }

            public ushort param_count //Total number of parameters
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort param_index //Index of this parameter
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public MAV_PARAM_EXT_TYPE param_type //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 4, data, 32);}
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	 ID is stored as strin*/
            public void param_id_SET(string src, Inside ph)
            {param_id_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	 ID is stored as strin*/
            public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 38 && insert_field(ph, 38, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } public void param_value_SET(string src, Inside ph) //Parameter value
            {param_value_SET(src.ToCharArray(), 0, src.Length, ph);} public void param_value_SET(char[] src, int pos, int items, Inside ph) //Parameter value
            {
                if(ph.field_bit != 39 && insert_field(ph, 39, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta322 = new Meta(322, 2, 0, 0, 5, 38, 2, _hh, _Rh);
        }/**
*Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET), when
*	 setting a parameter value and the new value is the same as the current value, you will immediately get
*	 a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you will accordingly receive
*	 a PARAM_ACK_IN_PROGRESS in response*/
        public class PARAM_EXT_SET : Pack, CommunicationChannel.Sendable
        {
            internal PARAM_EXT_SET() : base(meta323, 0) { }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public MAV_PARAM_EXT_TYPE param_type //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 4, data, 16);}
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	 ID is stored as strin*/
            public void param_id_SET(string src, Inside ph)
            {param_id_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	 ID is stored as strin*/
            public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 22 && insert_field(ph, 22, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } public void param_value_SET(string src, Inside ph) //Parameter value
            {param_value_SET(src.ToCharArray(), 0, src.Length, ph);} public void param_value_SET(char[] src, int pos, int items, Inside ph) //Parameter value
            {
                if(ph.field_bit != 23 && insert_field(ph, 23, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta323 = new Meta(323, 0, 0, 0, 3, 22, 2, _ih, _Dh);
        }/**
*Response from a PARAM_EXT_SET message.*/
        public class PARAM_EXT_ACK : Pack, CommunicationChannel.Sendable
        {
            internal PARAM_EXT_ACK() : base(meta324, 0) { }

            public MAV_PARAM_EXT_TYPE param_type //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 4, data, 0);}
            }

            public PARAM_ACK param_result //Result code: see the PARAM_ACK enum for possible codes.
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 2, data, 4);}
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	 ID is stored as strin*/
            public void param_id_SET(string src, Inside ph)
            {param_id_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	 ID is stored as strin*/
            public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 8 && insert_field(ph, 8, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } public void param_value_SET(string src, Inside ph) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
            {param_value_SET(src.ToCharArray(), 0, src.Length, ph);} public void param_value_SET(char[] src, int pos, int items, Inside ph) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
            {
                if(ph.field_bit != 9 && insert_field(ph, 9, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta324 = new Meta(324, 0, 0, 0, 1, 8, 2, _Xh, _Yh);
        }/**
*Obstacle distances in front of the sensor, starting from the left in increment degrees to the right*/
        public class OBSTACLE_DISTANCE : Pack, CommunicationChannel.Sendable
        {
            internal OBSTACLE_DISTANCE() : base(meta330, 0) { }

            /**
            *Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
            *	 is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
            *	 for unknown/not used. In a array element, each unit corresponds to 1cm*/
            public ushort[] distances
            {
                set {distances_SET(value, 0)  ;}
            }
            /**
            *Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
            *	 is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
            *	 for unknown/not used. In a array element, each unit corresponds to 1cm*/
            public void distances_SET(ushort[] src, int pos)
            {
                for(int BYTE =  0, src_max = pos + 72; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ulong)(src[pos]), 2, data,  BYTE);
            }

            public ushort min_distance //Minimum distance the sensor can measure in centimeters
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  144);}
            }

            public ushort max_distance //Maximum distance the sensor can measure in centimeters
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  146);}
            }

            public ulong time_usec //Timestamp (microseconds since system boot or since UNIX epoch)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  148);}
            }

            public byte increment //Angular width in degrees of each array element.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  156);}
            }

            public MAV_DISTANCE_SENSOR sensor_type //Class id of the distance sensor type.
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 1256);}
            }
            static readonly Meta meta330 = new Meta(330, 74, 0, 1, 158, 1259);
        }

        public class CommunicationChannel : Channel.Advanced
        {
            public override bool CanRead { get { return true ; } }
            public override bool CanWrite { get { return true; } }
            static CommunicationChannel() {pack_id_bytes = 2;}

            public static  CommunicationChannel instance = new CommunicationChannel();

            //interface-to-mark of sendable through this channel packs_Schs_Rchs
            public interface Sendable {}
            /**
            *Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
            *	 This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled
            *	 this way*/
            public static POSITION_TARGET_LOCAL_NED new_POSITION_TARGET_LOCAL_NED() {return new  POSITION_TARGET_LOCAL_NED();}
            public static SET_ATTITUDE_TARGET new_SET_ATTITUDE_TARGET() {return new  SET_ATTITUDE_TARGET();}
            /**
            *Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match
            *	 the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way*/
            public static ATTITUDE_TARGET new_ATTITUDE_TARGET() {return new  ATTITUDE_TARGET();}
            /**
            *Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller
            *	 to command the vehicle (manual controller or other system)*/
            public static SET_POSITION_TARGET_LOCAL_NED new_SET_POSITION_TARGET_LOCAL_NED() {return new  SET_POSITION_TARGET_LOCAL_NED();}
            /**
            *Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84).
            *	 Used by an external controller to command the vehicle (manual controller or other system)*/
            public static SET_POSITION_TARGET_GLOBAL_INT new_SET_POSITION_TARGET_GLOBAL_INT() {return new  SET_POSITION_TARGET_GLOBAL_INT();}
            /**
            *Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
            *	 This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled
            *	 this way*/
            public static POSITION_TARGET_GLOBAL_INT new_POSITION_TARGET_GLOBAL_INT() {return new  POSITION_TARGET_GLOBAL_INT();}
            /**
            *The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate
            *	 frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down
            *	 convention*/
            public static LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET() {return new  LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();}
            /**
            *DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please
            *	 use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput
            *	 applications such as hardware in the loop simulations*/
            public static HIL_STATE new_HIL_STATE() {return new  HIL_STATE();}
            /**
            *Sent from autopilot to simulation. Hardware in the loop control outputs*/
            public static HIL_CONTROLS new_HIL_CONTROLS() {return new  HIL_CONTROLS();}
            /**
            *Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation
            *	 is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might
            *	 violate this specification*/
            public static HIL_RC_INPUTS_RAW new_HIL_RC_INPUTS_RAW() {return new  HIL_RC_INPUTS_RAW();}
            /**
            *Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS*/
            public static HIL_ACTUATOR_CONTROLS new_HIL_ACTUATOR_CONTROLS() {return new  HIL_ACTUATOR_CONTROLS();}
            /**
            *Optical flow from a flow sensor (e.g. optical mouse sensor)*/
            public static OPTICAL_FLOW new_OPTICAL_FLOW() {return new  OPTICAL_FLOW();}
            public static GLOBAL_VISION_POSITION_ESTIMATE new_GLOBAL_VISION_POSITION_ESTIMATE() {return new  GLOBAL_VISION_POSITION_ESTIMATE();}
            public static VISION_POSITION_ESTIMATE new_VISION_POSITION_ESTIMATE() {return new  VISION_POSITION_ESTIMATE();}
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
            public static AUTOPILOT_VERSION new_AUTOPILOT_VERSION() {return new  AUTOPILOT_VERSION();}
            /**
            *The location of a landing area captured from a downward facing camera*/
            public static LANDING_TARGET new_LANDING_TARGET() {return new  LANDING_TARGET();}
            /**
            *Voltage and current sensor data*/
            public static SENS_POWER new_SENS_POWER() {return new  SENS_POWER();}
            /**
            *Maximum Power Point Tracker (MPPT) sensor data for solar module power performance tracking*/
            public static SENS_MPPT new_SENS_MPPT() {return new  SENS_MPPT();}
            /**
            *ASL-fixed-wing controller data*/
            public static ASLCTRL_DATA new_ASLCTRL_DATA() {return new  ASLCTRL_DATA();}
            /**
            *ASL-fixed-wing controller debug data*/
            public static ASLCTRL_DEBUG new_ASLCTRL_DEBUG() {return new  ASLCTRL_DEBUG();}
            /**
            *Extended state information for ASLUAVs*/
            public static ASLUAV_STATUS new_ASLUAV_STATUS() {return new  ASLUAV_STATUS();}
            /**
            *Extended EKF state estimates for ASLUAVs*/
            public static EKF_EXT new_EKF_EXT() {return new  EKF_EXT();}
            /**
            *Off-board controls/commands for ASLUAVs*/
            public static ASL_OBCTRL new_ASL_OBCTRL() {return new  ASL_OBCTRL();}
            /**
            *Atmospheric sensors (temperature, humidity, ...)*/
            public static SENS_ATMOS new_SENS_ATMOS() {return new  SENS_ATMOS();}
            /**
            *Battery pack monitoring data for Li-Ion batteries*/
            public static SENS_BATMON new_SENS_BATMON() {return new  SENS_BATMON();}
            /**
            *Fixed-wing soaring (i.e. thermal seeking) data*/
            public static FW_SOARING_DATA new_FW_SOARING_DATA() {return new  FW_SOARING_DATA();}
            /**
            *Monitoring of sensorpod status*/
            public static SENSORPOD_STATUS new_SENSORPOD_STATUS() {return new  SENSORPOD_STATUS();}
            /**
            *Monitoring of power board status*/
            public static SENS_POWER_BOARD new_SENS_POWER_BOARD() {return new  SENS_POWER_BOARD();}
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
            /**
            *WIP: Orientation of a mount*/
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

            public void send(Sendable pack) {lock(sendout_packs) {sendout_packs.Enqueue((Pack) pack); Monitor.PulseAll(sendout_packs);}}
            protected readonly Queue<Pack> sendout_packs = new Queue<Pack>();
            protected readonly Queue<Pack> received_packs = new Queue<Pack>();
            private readonly Inside ph    = new Inside();
            protected internal override Pack process(Pack pack, int id)
            {
                for(bool LOOP = false; ;)
                {
                    switch(id)
                    {
                        default:
                            Debug.Assert(false);
                            return null;
                        case 0:
                            if(pack == null) return new HEARTBEAT();
                            if(OnHEARTBEATReceive == null) return null;
                            ph.setPack(pack);
                            OnHEARTBEATReceive(this, ph, (HEARTBEAT) pack);
                            if(LOOP) break;
                            return null;
                        case 1:
                            if(pack == null) return new SYS_STATUS();
                            if(OnSYS_STATUSReceive == null) return null;
                            ph.setPack(pack);
                            OnSYS_STATUSReceive(this, ph, (SYS_STATUS) pack);
                            if(LOOP) break;
                            return null;
                        case 2:
                            if(pack == null) return new SYSTEM_TIME();
                            if(OnSYSTEM_TIMEReceive == null) return null;
                            ph.setPack(pack);
                            OnSYSTEM_TIMEReceive(this, ph, (SYSTEM_TIME) pack);
                            if(LOOP) break;
                            return null;
                        case 3:
                            if(pack == null) return new POSITION_TARGET_LOCAL_NED();
                            if(OnPOSITION_TARGET_LOCAL_NEDReceive == null) return null;
                            ph.setPack(pack);
                            OnPOSITION_TARGET_LOCAL_NEDReceive(this, ph, (POSITION_TARGET_LOCAL_NED) pack);
                            if(LOOP) break;
                            return null;
                        case 4:
                            if(pack == null) return new PING();
                            if(OnPINGReceive == null) return null;
                            ph.setPack(pack);
                            OnPINGReceive(this, ph, (PING) pack);
                            if(LOOP) break;
                            return null;
                        case 5:
                            if(pack == null) return new CHANGE_OPERATOR_CONTROL(-1);
                            if(OnCHANGE_OPERATOR_CONTROLReceive == null) return null;
                            ph.setPack(pack);
                            OnCHANGE_OPERATOR_CONTROLReceive(this, ph, (CHANGE_OPERATOR_CONTROL) pack);
                            if(LOOP) break;
                            return null;
                        case 6:
                            if(pack == null) return new CHANGE_OPERATOR_CONTROL_ACK();
                            if(OnCHANGE_OPERATOR_CONTROL_ACKReceive == null) return null;
                            ph.setPack(pack);
                            OnCHANGE_OPERATOR_CONTROL_ACKReceive(this, ph, (CHANGE_OPERATOR_CONTROL_ACK) pack);
                            if(LOOP) break;
                            return null;
                        case 7:
                            if(pack == null) return new AUTH_KEY(-1);
                            if(OnAUTH_KEYReceive == null) return null;
                            ph.setPack(pack);
                            OnAUTH_KEYReceive(this, ph, (AUTH_KEY) pack);
                            if(LOOP) break;
                            return null;
                        case 11:
                            if(pack == null) return new SET_MODE();
                            if(OnSET_MODEReceive == null) return null;
                            ph.setPack(pack);
                            OnSET_MODEReceive(this, ph, (SET_MODE) pack);
                            if(LOOP) break;
                            return null;
                        case 20:
                            if(pack == null) return new PARAM_REQUEST_READ(-1);
                            if(OnPARAM_REQUEST_READReceive == null) return null;
                            ph.setPack(pack);
                            OnPARAM_REQUEST_READReceive(this, ph, (PARAM_REQUEST_READ) pack);
                            if(LOOP) break;
                            return null;
                        case 21:
                            if(pack == null) return new PARAM_REQUEST_LIST();
                            if(OnPARAM_REQUEST_LISTReceive == null) return null;
                            ph.setPack(pack);
                            OnPARAM_REQUEST_LISTReceive(this, ph, (PARAM_REQUEST_LIST) pack);
                            if(LOOP) break;
                            return null;
                        case 22:
                            if(pack == null) return new PARAM_VALUE(-1);
                            if(OnPARAM_VALUEReceive == null) return null;
                            ph.setPack(pack);
                            OnPARAM_VALUEReceive(this, ph, (PARAM_VALUE) pack);
                            if(LOOP) break;
                            return null;
                        case 23:
                            if(pack == null) return new PARAM_SET(-1);
                            if(OnPARAM_SETReceive == null) return null;
                            ph.setPack(pack);
                            OnPARAM_SETReceive(this, ph, (PARAM_SET) pack);
                            if(LOOP) break;
                            return null;
                        case 24:
                            if(pack == null) return new GPS_RAW_INT(-1);
                            if(OnGPS_RAW_INTReceive == null) return null;
                            ph.setPack(pack);
                            OnGPS_RAW_INTReceive(this, ph, (GPS_RAW_INT) pack);
                            if(LOOP) break;
                            return null;
                        case 25:
                            if(pack == null) return new GPS_STATUS();
                            if(OnGPS_STATUSReceive == null) return null;
                            ph.setPack(pack);
                            OnGPS_STATUSReceive(this, ph, (GPS_STATUS) pack);
                            if(LOOP) break;
                            return null;
                        case 26:
                            if(pack == null) return new SCALED_IMU();
                            if(OnSCALED_IMUReceive == null) return null;
                            ph.setPack(pack);
                            OnSCALED_IMUReceive(this, ph, (SCALED_IMU) pack);
                            if(LOOP) break;
                            return null;
                        case 27:
                            if(pack == null) return new RAW_IMU();
                            if(OnRAW_IMUReceive == null) return null;
                            ph.setPack(pack);
                            OnRAW_IMUReceive(this, ph, (RAW_IMU) pack);
                            if(LOOP) break;
                            return null;
                        case 28:
                            if(pack == null) return new RAW_PRESSURE();
                            if(OnRAW_PRESSUREReceive == null) return null;
                            ph.setPack(pack);
                            OnRAW_PRESSUREReceive(this, ph, (RAW_PRESSURE) pack);
                            if(LOOP) break;
                            return null;
                        case 29:
                            if(pack == null) return new SCALED_PRESSURE();
                            if(OnSCALED_PRESSUREReceive == null) return null;
                            ph.setPack(pack);
                            OnSCALED_PRESSUREReceive(this, ph, (SCALED_PRESSURE) pack);
                            if(LOOP) break;
                            return null;
                        case 30:
                            if(pack == null) return new ATTITUDE();
                            if(OnATTITUDEReceive == null) return null;
                            ph.setPack(pack);
                            OnATTITUDEReceive(this, ph, (ATTITUDE) pack);
                            if(LOOP) break;
                            return null;
                        case 31:
                            if(pack == null) return new ATTITUDE_QUATERNION();
                            if(OnATTITUDE_QUATERNIONReceive == null) return null;
                            ph.setPack(pack);
                            OnATTITUDE_QUATERNIONReceive(this, ph, (ATTITUDE_QUATERNION) pack);
                            if(LOOP) break;
                            return null;
                        case 32:
                            if(pack == null) return new LOCAL_POSITION_NED();
                            if(OnLOCAL_POSITION_NEDReceive == null) return null;
                            ph.setPack(pack);
                            OnLOCAL_POSITION_NEDReceive(this, ph, (LOCAL_POSITION_NED) pack);
                            if(LOOP) break;
                            return null;
                        case 33:
                            if(pack == null) return new GLOBAL_POSITION_INT();
                            if(OnGLOBAL_POSITION_INTReceive == null) return null;
                            ph.setPack(pack);
                            OnGLOBAL_POSITION_INTReceive(this, ph, (GLOBAL_POSITION_INT) pack);
                            if(LOOP) break;
                            return null;
                        case 34:
                            if(pack == null) return new RC_CHANNELS_SCALED();
                            if(OnRC_CHANNELS_SCALEDReceive == null) return null;
                            ph.setPack(pack);
                            OnRC_CHANNELS_SCALEDReceive(this, ph, (RC_CHANNELS_SCALED) pack);
                            if(LOOP) break;
                            return null;
                        case 35:
                            if(pack == null) return new RC_CHANNELS_RAW();
                            if(OnRC_CHANNELS_RAWReceive == null) return null;
                            ph.setPack(pack);
                            OnRC_CHANNELS_RAWReceive(this, ph, (RC_CHANNELS_RAW) pack);
                            if(LOOP) break;
                            return null;
                        case 36:
                            if(pack == null) return new SERVO_OUTPUT_RAW(-1);
                            if(OnSERVO_OUTPUT_RAWReceive == null) return null;
                            ph.setPack(pack);
                            OnSERVO_OUTPUT_RAWReceive(this, ph, (SERVO_OUTPUT_RAW) pack);
                            if(LOOP) break;
                            return null;
                        case 37:
                            if(pack == null) return new MISSION_REQUEST_PARTIAL_LIST();
                            if(OnMISSION_REQUEST_PARTIAL_LISTReceive == null) return null;
                            ph.setPack(pack);
                            OnMISSION_REQUEST_PARTIAL_LISTReceive(this, ph, (MISSION_REQUEST_PARTIAL_LIST) pack);
                            if(LOOP) break;
                            return null;
                        case 38:
                            if(pack == null) return new MISSION_WRITE_PARTIAL_LIST();
                            if(OnMISSION_WRITE_PARTIAL_LISTReceive == null) return null;
                            ph.setPack(pack);
                            OnMISSION_WRITE_PARTIAL_LISTReceive(this, ph, (MISSION_WRITE_PARTIAL_LIST) pack);
                            if(LOOP) break;
                            return null;
                        case 39:
                            if(pack == null) return new MISSION_ITEM();
                            if(OnMISSION_ITEMReceive == null) return null;
                            ph.setPack(pack);
                            OnMISSION_ITEMReceive(this, ph, (MISSION_ITEM) pack);
                            if(LOOP) break;
                            return null;
                        case 40:
                            if(pack == null) return new MISSION_REQUEST();
                            if(OnMISSION_REQUESTReceive == null) return null;
                            ph.setPack(pack);
                            OnMISSION_REQUESTReceive(this, ph, (MISSION_REQUEST) pack);
                            if(LOOP) break;
                            return null;
                        case 41:
                            if(pack == null) return new MISSION_SET_CURRENT();
                            if(OnMISSION_SET_CURRENTReceive == null) return null;
                            ph.setPack(pack);
                            OnMISSION_SET_CURRENTReceive(this, ph, (MISSION_SET_CURRENT) pack);
                            if(LOOP) break;
                            return null;
                        case 42:
                            if(pack == null) return new MISSION_CURRENT();
                            if(OnMISSION_CURRENTReceive == null) return null;
                            ph.setPack(pack);
                            OnMISSION_CURRENTReceive(this, ph, (MISSION_CURRENT) pack);
                            if(LOOP) break;
                            return null;
                        case 43:
                            if(pack == null) return new MISSION_REQUEST_LIST();
                            if(OnMISSION_REQUEST_LISTReceive == null) return null;
                            ph.setPack(pack);
                            OnMISSION_REQUEST_LISTReceive(this, ph, (MISSION_REQUEST_LIST) pack);
                            if(LOOP) break;
                            return null;
                        case 44:
                            if(pack == null) return new MISSION_COUNT();
                            if(OnMISSION_COUNTReceive == null) return null;
                            ph.setPack(pack);
                            OnMISSION_COUNTReceive(this, ph, (MISSION_COUNT) pack);
                            if(LOOP) break;
                            return null;
                        case 45:
                            if(pack == null) return new MISSION_CLEAR_ALL();
                            if(OnMISSION_CLEAR_ALLReceive == null) return null;
                            ph.setPack(pack);
                            OnMISSION_CLEAR_ALLReceive(this, ph, (MISSION_CLEAR_ALL) pack);
                            if(LOOP) break;
                            return null;
                        case 46:
                            if(pack == null) return new MISSION_ITEM_REACHED();
                            if(OnMISSION_ITEM_REACHEDReceive == null) return null;
                            ph.setPack(pack);
                            OnMISSION_ITEM_REACHEDReceive(this, ph, (MISSION_ITEM_REACHED) pack);
                            if(LOOP) break;
                            return null;
                        case 47:
                            if(pack == null) return new MISSION_ACK();
                            if(OnMISSION_ACKReceive == null) return null;
                            ph.setPack(pack);
                            OnMISSION_ACKReceive(this, ph, (MISSION_ACK) pack);
                            if(LOOP) break;
                            return null;
                        case 48:
                            if(pack == null) return new SET_GPS_GLOBAL_ORIGIN(-1);
                            if(OnSET_GPS_GLOBAL_ORIGINReceive == null) return null;
                            ph.setPack(pack);
                            OnSET_GPS_GLOBAL_ORIGINReceive(this, ph, (SET_GPS_GLOBAL_ORIGIN) pack);
                            if(LOOP) break;
                            return null;
                        case 49:
                            if(pack == null) return new GPS_GLOBAL_ORIGIN(-1);
                            if(OnGPS_GLOBAL_ORIGINReceive == null) return null;
                            ph.setPack(pack);
                            OnGPS_GLOBAL_ORIGINReceive(this, ph, (GPS_GLOBAL_ORIGIN) pack);
                            if(LOOP) break;
                            return null;
                        case 50:
                            if(pack == null) return new PARAM_MAP_RC(-1);
                            if(OnPARAM_MAP_RCReceive == null) return null;
                            ph.setPack(pack);
                            OnPARAM_MAP_RCReceive(this, ph, (PARAM_MAP_RC) pack);
                            if(LOOP) break;
                            return null;
                        case 51:
                            if(pack == null) return new MISSION_REQUEST_INT();
                            if(OnMISSION_REQUEST_INTReceive == null) return null;
                            ph.setPack(pack);
                            OnMISSION_REQUEST_INTReceive(this, ph, (MISSION_REQUEST_INT) pack);
                            if(LOOP) break;
                            return null;
                        case 54:
                            if(pack == null) return new SAFETY_SET_ALLOWED_AREA();
                            if(OnSAFETY_SET_ALLOWED_AREAReceive == null) return null;
                            ph.setPack(pack);
                            OnSAFETY_SET_ALLOWED_AREAReceive(this, ph, (SAFETY_SET_ALLOWED_AREA) pack);
                            if(LOOP) break;
                            return null;
                        case 55:
                            if(pack == null) return new SAFETY_ALLOWED_AREA();
                            if(OnSAFETY_ALLOWED_AREAReceive == null) return null;
                            ph.setPack(pack);
                            OnSAFETY_ALLOWED_AREAReceive(this, ph, (SAFETY_ALLOWED_AREA) pack);
                            if(LOOP) break;
                            return null;
                        case 61:
                            if(pack == null) return new ATTITUDE_QUATERNION_COV();
                            if(OnATTITUDE_QUATERNION_COVReceive == null) return null;
                            ph.setPack(pack);
                            OnATTITUDE_QUATERNION_COVReceive(this, ph, (ATTITUDE_QUATERNION_COV) pack);
                            if(LOOP) break;
                            return null;
                        case 62:
                            if(pack == null) return new NAV_CONTROLLER_OUTPUT();
                            if(OnNAV_CONTROLLER_OUTPUTReceive == null) return null;
                            ph.setPack(pack);
                            OnNAV_CONTROLLER_OUTPUTReceive(this, ph, (NAV_CONTROLLER_OUTPUT) pack);
                            if(LOOP) break;
                            return null;
                        case 63:
                            if(pack == null) return new GLOBAL_POSITION_INT_COV();
                            if(OnGLOBAL_POSITION_INT_COVReceive == null) return null;
                            ph.setPack(pack);
                            OnGLOBAL_POSITION_INT_COVReceive(this, ph, (GLOBAL_POSITION_INT_COV) pack);
                            if(LOOP) break;
                            return null;
                        case 64:
                            if(pack == null) return new LOCAL_POSITION_NED_COV();
                            if(OnLOCAL_POSITION_NED_COVReceive == null) return null;
                            ph.setPack(pack);
                            OnLOCAL_POSITION_NED_COVReceive(this, ph, (LOCAL_POSITION_NED_COV) pack);
                            if(LOOP) break;
                            return null;
                        case 65:
                            if(pack == null) return new RC_CHANNELS();
                            if(OnRC_CHANNELSReceive == null) return null;
                            ph.setPack(pack);
                            OnRC_CHANNELSReceive(this, ph, (RC_CHANNELS) pack);
                            if(LOOP) break;
                            return null;
                        case 66:
                            if(pack == null) return new REQUEST_DATA_STREAM();
                            if(OnREQUEST_DATA_STREAMReceive == null) return null;
                            ph.setPack(pack);
                            OnREQUEST_DATA_STREAMReceive(this, ph, (REQUEST_DATA_STREAM) pack);
                            if(LOOP) break;
                            return null;
                        case 67:
                            if(pack == null) return new DATA_STREAM();
                            if(OnDATA_STREAMReceive == null) return null;
                            ph.setPack(pack);
                            OnDATA_STREAMReceive(this, ph, (DATA_STREAM) pack);
                            if(LOOP) break;
                            return null;
                        case 69:
                            if(pack == null) return new MANUAL_CONTROL();
                            if(OnMANUAL_CONTROLReceive == null) return null;
                            ph.setPack(pack);
                            OnMANUAL_CONTROLReceive(this, ph, (MANUAL_CONTROL) pack);
                            if(LOOP) break;
                            return null;
                        case 70:
                            if(pack == null) return new RC_CHANNELS_OVERRIDE();
                            if(OnRC_CHANNELS_OVERRIDEReceive == null) return null;
                            ph.setPack(pack);
                            OnRC_CHANNELS_OVERRIDEReceive(this, ph, (RC_CHANNELS_OVERRIDE) pack);
                            if(LOOP) break;
                            return null;
                        case 73:
                            if(pack == null) return new MISSION_ITEM_INT();
                            if(OnMISSION_ITEM_INTReceive == null) return null;
                            ph.setPack(pack);
                            OnMISSION_ITEM_INTReceive(this, ph, (MISSION_ITEM_INT) pack);
                            if(LOOP) break;
                            return null;
                        case 74:
                            if(pack == null) return new VFR_HUD();
                            if(OnVFR_HUDReceive == null) return null;
                            ph.setPack(pack);
                            OnVFR_HUDReceive(this, ph, (VFR_HUD) pack);
                            if(LOOP) break;
                            return null;
                        case 75:
                            if(pack == null) return new COMMAND_INT();
                            if(OnCOMMAND_INTReceive == null) return null;
                            ph.setPack(pack);
                            OnCOMMAND_INTReceive(this, ph, (COMMAND_INT) pack);
                            if(LOOP) break;
                            return null;
                        case 76:
                            if(pack == null) return new COMMAND_LONG();
                            if(OnCOMMAND_LONGReceive == null) return null;
                            ph.setPack(pack);
                            OnCOMMAND_LONGReceive(this, ph, (COMMAND_LONG) pack);
                            if(LOOP) break;
                            return null;
                        case 77:
                            if(pack == null) return new COMMAND_ACK(-1);
                            if(OnCOMMAND_ACKReceive == null) return null;
                            ph.setPack(pack);
                            OnCOMMAND_ACKReceive(this, ph, (COMMAND_ACK) pack);
                            if(LOOP) break;
                            return null;
                        case 81:
                            if(pack == null) return new MANUAL_SETPOINT();
                            if(OnMANUAL_SETPOINTReceive == null) return null;
                            ph.setPack(pack);
                            OnMANUAL_SETPOINTReceive(this, ph, (MANUAL_SETPOINT) pack);
                            if(LOOP) break;
                            return null;
                        case 82:
                            if(pack == null) return new SET_ATTITUDE_TARGET();
                            if(OnSET_ATTITUDE_TARGETReceive == null) return null;
                            ph.setPack(pack);
                            OnSET_ATTITUDE_TARGETReceive(this, ph, (SET_ATTITUDE_TARGET) pack);
                            if(LOOP) break;
                            return null;
                        case 83:
                            if(pack == null) return new ATTITUDE_TARGET();
                            if(OnATTITUDE_TARGETReceive == null) return null;
                            ph.setPack(pack);
                            OnATTITUDE_TARGETReceive(this, ph, (ATTITUDE_TARGET) pack);
                            if(LOOP) break;
                            return null;
                        case 84:
                            if(pack == null) return new SET_POSITION_TARGET_LOCAL_NED();
                            if(OnSET_POSITION_TARGET_LOCAL_NEDReceive == null) return null;
                            ph.setPack(pack);
                            OnSET_POSITION_TARGET_LOCAL_NEDReceive(this, ph, (SET_POSITION_TARGET_LOCAL_NED) pack);
                            if(LOOP) break;
                            return null;
                        case 86:
                            if(pack == null) return new SET_POSITION_TARGET_GLOBAL_INT();
                            if(OnSET_POSITION_TARGET_GLOBAL_INTReceive == null) return null;
                            ph.setPack(pack);
                            OnSET_POSITION_TARGET_GLOBAL_INTReceive(this, ph, (SET_POSITION_TARGET_GLOBAL_INT) pack);
                            if(LOOP) break;
                            return null;
                        case 87:
                            if(pack == null) return new POSITION_TARGET_GLOBAL_INT();
                            if(OnPOSITION_TARGET_GLOBAL_INTReceive == null) return null;
                            ph.setPack(pack);
                            OnPOSITION_TARGET_GLOBAL_INTReceive(this, ph, (POSITION_TARGET_GLOBAL_INT) pack);
                            if(LOOP) break;
                            return null;
                        case 89:
                            if(pack == null) return new LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
                            if(OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive == null) return null;
                            ph.setPack(pack);
                            OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive(this, ph, (LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET) pack);
                            if(LOOP) break;
                            return null;
                        case 90:
                            if(pack == null) return new HIL_STATE();
                            if(OnHIL_STATEReceive == null) return null;
                            ph.setPack(pack);
                            OnHIL_STATEReceive(this, ph, (HIL_STATE) pack);
                            if(LOOP) break;
                            return null;
                        case 91:
                            if(pack == null) return new HIL_CONTROLS();
                            if(OnHIL_CONTROLSReceive == null) return null;
                            ph.setPack(pack);
                            OnHIL_CONTROLSReceive(this, ph, (HIL_CONTROLS) pack);
                            if(LOOP) break;
                            return null;
                        case 92:
                            if(pack == null) return new HIL_RC_INPUTS_RAW();
                            if(OnHIL_RC_INPUTS_RAWReceive == null) return null;
                            ph.setPack(pack);
                            OnHIL_RC_INPUTS_RAWReceive(this, ph, (HIL_RC_INPUTS_RAW) pack);
                            if(LOOP) break;
                            return null;
                        case 93:
                            if(pack == null) return new HIL_ACTUATOR_CONTROLS();
                            if(OnHIL_ACTUATOR_CONTROLSReceive == null) return null;
                            ph.setPack(pack);
                            OnHIL_ACTUATOR_CONTROLSReceive(this, ph, (HIL_ACTUATOR_CONTROLS) pack);
                            if(LOOP) break;
                            return null;
                        case 100:
                            if(pack == null) return new OPTICAL_FLOW(-1);
                            if(OnOPTICAL_FLOWReceive == null) return null;
                            ph.setPack(pack);
                            OnOPTICAL_FLOWReceive(this, ph, (OPTICAL_FLOW) pack);
                            if(LOOP) break;
                            return null;
                        case 101:
                            if(pack == null) return new GLOBAL_VISION_POSITION_ESTIMATE();
                            if(OnGLOBAL_VISION_POSITION_ESTIMATEReceive == null) return null;
                            ph.setPack(pack);
                            OnGLOBAL_VISION_POSITION_ESTIMATEReceive(this, ph, (GLOBAL_VISION_POSITION_ESTIMATE) pack);
                            if(LOOP) break;
                            return null;
                        case 102:
                            if(pack == null) return new VISION_POSITION_ESTIMATE();
                            if(OnVISION_POSITION_ESTIMATEReceive == null) return null;
                            ph.setPack(pack);
                            OnVISION_POSITION_ESTIMATEReceive(this, ph, (VISION_POSITION_ESTIMATE) pack);
                            if(LOOP) break;
                            return null;
                        case 103:
                            if(pack == null) return new VISION_SPEED_ESTIMATE();
                            if(OnVISION_SPEED_ESTIMATEReceive == null) return null;
                            ph.setPack(pack);
                            OnVISION_SPEED_ESTIMATEReceive(this, ph, (VISION_SPEED_ESTIMATE) pack);
                            if(LOOP) break;
                            return null;
                        case 104:
                            if(pack == null) return new VICON_POSITION_ESTIMATE();
                            if(OnVICON_POSITION_ESTIMATEReceive == null) return null;
                            ph.setPack(pack);
                            OnVICON_POSITION_ESTIMATEReceive(this, ph, (VICON_POSITION_ESTIMATE) pack);
                            if(LOOP) break;
                            return null;
                        case 105:
                            if(pack == null) return new HIGHRES_IMU();
                            if(OnHIGHRES_IMUReceive == null) return null;
                            ph.setPack(pack);
                            OnHIGHRES_IMUReceive(this, ph, (HIGHRES_IMU) pack);
                            if(LOOP) break;
                            return null;
                        case 106:
                            if(pack == null) return new OPTICAL_FLOW_RAD();
                            if(OnOPTICAL_FLOW_RADReceive == null) return null;
                            ph.setPack(pack);
                            OnOPTICAL_FLOW_RADReceive(this, ph, (OPTICAL_FLOW_RAD) pack);
                            if(LOOP) break;
                            return null;
                        case 107:
                            if(pack == null) return new HIL_SENSOR();
                            if(OnHIL_SENSORReceive == null) return null;
                            ph.setPack(pack);
                            OnHIL_SENSORReceive(this, ph, (HIL_SENSOR) pack);
                            if(LOOP) break;
                            return null;
                        case 108:
                            if(pack == null) return new SIM_STATE();
                            if(OnSIM_STATEReceive == null) return null;
                            ph.setPack(pack);
                            OnSIM_STATEReceive(this, ph, (SIM_STATE) pack);
                            if(LOOP) break;
                            return null;
                        case 109:
                            if(pack == null) return new RADIO_STATUS();
                            if(OnRADIO_STATUSReceive == null) return null;
                            ph.setPack(pack);
                            OnRADIO_STATUSReceive(this, ph, (RADIO_STATUS) pack);
                            if(LOOP) break;
                            return null;
                        case 110:
                            if(pack == null) return new FILE_TRANSFER_PROTOCOL();
                            if(OnFILE_TRANSFER_PROTOCOLReceive == null) return null;
                            ph.setPack(pack);
                            OnFILE_TRANSFER_PROTOCOLReceive(this, ph, (FILE_TRANSFER_PROTOCOL) pack);
                            if(LOOP) break;
                            return null;
                        case 111:
                            if(pack == null) return new TIMESYNC();
                            if(OnTIMESYNCReceive == null) return null;
                            ph.setPack(pack);
                            OnTIMESYNCReceive(this, ph, (TIMESYNC) pack);
                            if(LOOP) break;
                            return null;
                        case 112:
                            if(pack == null) return new CAMERA_TRIGGER();
                            if(OnCAMERA_TRIGGERReceive == null) return null;
                            ph.setPack(pack);
                            OnCAMERA_TRIGGERReceive(this, ph, (CAMERA_TRIGGER) pack);
                            if(LOOP) break;
                            return null;
                        case 113:
                            if(pack == null) return new HIL_GPS();
                            if(OnHIL_GPSReceive == null) return null;
                            ph.setPack(pack);
                            OnHIL_GPSReceive(this, ph, (HIL_GPS) pack);
                            if(LOOP) break;
                            return null;
                        case 114:
                            if(pack == null) return new HIL_OPTICAL_FLOW();
                            if(OnHIL_OPTICAL_FLOWReceive == null) return null;
                            ph.setPack(pack);
                            OnHIL_OPTICAL_FLOWReceive(this, ph, (HIL_OPTICAL_FLOW) pack);
                            if(LOOP) break;
                            return null;
                        case 115:
                            if(pack == null) return new HIL_STATE_QUATERNION();
                            if(OnHIL_STATE_QUATERNIONReceive == null) return null;
                            ph.setPack(pack);
                            OnHIL_STATE_QUATERNIONReceive(this, ph, (HIL_STATE_QUATERNION) pack);
                            if(LOOP) break;
                            return null;
                        case 116:
                            if(pack == null) return new SCALED_IMU2();
                            if(OnSCALED_IMU2Receive == null) return null;
                            ph.setPack(pack);
                            OnSCALED_IMU2Receive(this, ph, (SCALED_IMU2) pack);
                            if(LOOP) break;
                            return null;
                        case 117:
                            if(pack == null) return new LOG_REQUEST_LIST();
                            if(OnLOG_REQUEST_LISTReceive == null) return null;
                            ph.setPack(pack);
                            OnLOG_REQUEST_LISTReceive(this, ph, (LOG_REQUEST_LIST) pack);
                            if(LOOP) break;
                            return null;
                        case 118:
                            if(pack == null) return new LOG_ENTRY();
                            if(OnLOG_ENTRYReceive == null) return null;
                            ph.setPack(pack);
                            OnLOG_ENTRYReceive(this, ph, (LOG_ENTRY) pack);
                            if(LOOP) break;
                            return null;
                        case 119:
                            if(pack == null) return new LOG_REQUEST_DATA();
                            if(OnLOG_REQUEST_DATAReceive == null) return null;
                            ph.setPack(pack);
                            OnLOG_REQUEST_DATAReceive(this, ph, (LOG_REQUEST_DATA) pack);
                            if(LOOP) break;
                            return null;
                        case 120:
                            if(pack == null) return new LOG_DATA();
                            if(OnLOG_DATAReceive == null) return null;
                            ph.setPack(pack);
                            OnLOG_DATAReceive(this, ph, (LOG_DATA) pack);
                            if(LOOP) break;
                            return null;
                        case 121:
                            if(pack == null) return new LOG_ERASE();
                            if(OnLOG_ERASEReceive == null) return null;
                            ph.setPack(pack);
                            OnLOG_ERASEReceive(this, ph, (LOG_ERASE) pack);
                            if(LOOP) break;
                            return null;
                        case 122:
                            if(pack == null) return new LOG_REQUEST_END();
                            if(OnLOG_REQUEST_ENDReceive == null) return null;
                            ph.setPack(pack);
                            OnLOG_REQUEST_ENDReceive(this, ph, (LOG_REQUEST_END) pack);
                            if(LOOP) break;
                            return null;
                        case 123:
                            if(pack == null) return new GPS_INJECT_DATA();
                            if(OnGPS_INJECT_DATAReceive == null) return null;
                            ph.setPack(pack);
                            OnGPS_INJECT_DATAReceive(this, ph, (GPS_INJECT_DATA) pack);
                            if(LOOP) break;
                            return null;
                        case 124:
                            if(pack == null) return new GPS2_RAW();
                            if(OnGPS2_RAWReceive == null) return null;
                            ph.setPack(pack);
                            OnGPS2_RAWReceive(this, ph, (GPS2_RAW) pack);
                            if(LOOP) break;
                            return null;
                        case 125:
                            if(pack == null) return new POWER_STATUS();
                            if(OnPOWER_STATUSReceive == null) return null;
                            ph.setPack(pack);
                            OnPOWER_STATUSReceive(this, ph, (POWER_STATUS) pack);
                            if(LOOP) break;
                            return null;
                        case 126:
                            if(pack == null) return new SERIAL_CONTROL();
                            if(OnSERIAL_CONTROLReceive == null) return null;
                            ph.setPack(pack);
                            OnSERIAL_CONTROLReceive(this, ph, (SERIAL_CONTROL) pack);
                            if(LOOP) break;
                            return null;
                        case 127:
                            if(pack == null) return new GPS_RTK();
                            if(OnGPS_RTKReceive == null) return null;
                            ph.setPack(pack);
                            OnGPS_RTKReceive(this, ph, (GPS_RTK) pack);
                            if(LOOP) break;
                            return null;
                        case 128:
                            if(pack == null) return new GPS2_RTK();
                            if(OnGPS2_RTKReceive == null) return null;
                            ph.setPack(pack);
                            OnGPS2_RTKReceive(this, ph, (GPS2_RTK) pack);
                            if(LOOP) break;
                            return null;
                        case 129:
                            if(pack == null) return new SCALED_IMU3();
                            if(OnSCALED_IMU3Receive == null) return null;
                            ph.setPack(pack);
                            OnSCALED_IMU3Receive(this, ph, (SCALED_IMU3) pack);
                            if(LOOP) break;
                            return null;
                        case 130:
                            if(pack == null) return new DATA_TRANSMISSION_HANDSHAKE();
                            if(OnDATA_TRANSMISSION_HANDSHAKEReceive == null) return null;
                            ph.setPack(pack);
                            OnDATA_TRANSMISSION_HANDSHAKEReceive(this, ph, (DATA_TRANSMISSION_HANDSHAKE) pack);
                            if(LOOP) break;
                            return null;
                        case 131:
                            if(pack == null) return new ENCAPSULATED_DATA();
                            if(OnENCAPSULATED_DATAReceive == null) return null;
                            ph.setPack(pack);
                            OnENCAPSULATED_DATAReceive(this, ph, (ENCAPSULATED_DATA) pack);
                            if(LOOP) break;
                            return null;
                        case 132:
                            if(pack == null) return new DISTANCE_SENSOR();
                            if(OnDISTANCE_SENSORReceive == null) return null;
                            ph.setPack(pack);
                            OnDISTANCE_SENSORReceive(this, ph, (DISTANCE_SENSOR) pack);
                            if(LOOP) break;
                            return null;
                        case 133:
                            if(pack == null) return new TERRAIN_REQUEST();
                            if(OnTERRAIN_REQUESTReceive == null) return null;
                            ph.setPack(pack);
                            OnTERRAIN_REQUESTReceive(this, ph, (TERRAIN_REQUEST) pack);
                            if(LOOP) break;
                            return null;
                        case 134:
                            if(pack == null) return new TERRAIN_DATA();
                            if(OnTERRAIN_DATAReceive == null) return null;
                            ph.setPack(pack);
                            OnTERRAIN_DATAReceive(this, ph, (TERRAIN_DATA) pack);
                            if(LOOP) break;
                            return null;
                        case 135:
                            if(pack == null) return new TERRAIN_CHECK();
                            if(OnTERRAIN_CHECKReceive == null) return null;
                            ph.setPack(pack);
                            OnTERRAIN_CHECKReceive(this, ph, (TERRAIN_CHECK) pack);
                            if(LOOP) break;
                            return null;
                        case 136:
                            if(pack == null) return new TERRAIN_REPORT();
                            if(OnTERRAIN_REPORTReceive == null) return null;
                            ph.setPack(pack);
                            OnTERRAIN_REPORTReceive(this, ph, (TERRAIN_REPORT) pack);
                            if(LOOP) break;
                            return null;
                        case 137:
                            if(pack == null) return new SCALED_PRESSURE2();
                            if(OnSCALED_PRESSURE2Receive == null) return null;
                            ph.setPack(pack);
                            OnSCALED_PRESSURE2Receive(this, ph, (SCALED_PRESSURE2) pack);
                            if(LOOP) break;
                            return null;
                        case 138:
                            if(pack == null) return new ATT_POS_MOCAP();
                            if(OnATT_POS_MOCAPReceive == null) return null;
                            ph.setPack(pack);
                            OnATT_POS_MOCAPReceive(this, ph, (ATT_POS_MOCAP) pack);
                            if(LOOP) break;
                            return null;
                        case 139:
                            if(pack == null) return new SET_ACTUATOR_CONTROL_TARGET();
                            if(OnSET_ACTUATOR_CONTROL_TARGETReceive == null) return null;
                            ph.setPack(pack);
                            OnSET_ACTUATOR_CONTROL_TARGETReceive(this, ph, (SET_ACTUATOR_CONTROL_TARGET) pack);
                            if(LOOP) break;
                            return null;
                        case 140:
                            if(pack == null) return new ACTUATOR_CONTROL_TARGET();
                            if(OnACTUATOR_CONTROL_TARGETReceive == null) return null;
                            ph.setPack(pack);
                            OnACTUATOR_CONTROL_TARGETReceive(this, ph, (ACTUATOR_CONTROL_TARGET) pack);
                            if(LOOP) break;
                            return null;
                        case 141:
                            if(pack == null) return new ALTITUDE();
                            if(OnALTITUDEReceive == null) return null;
                            ph.setPack(pack);
                            OnALTITUDEReceive(this, ph, (ALTITUDE) pack);
                            if(LOOP) break;
                            return null;
                        case 142:
                            if(pack == null) return new RESOURCE_REQUEST();
                            if(OnRESOURCE_REQUESTReceive == null) return null;
                            ph.setPack(pack);
                            OnRESOURCE_REQUESTReceive(this, ph, (RESOURCE_REQUEST) pack);
                            if(LOOP) break;
                            return null;
                        case 143:
                            if(pack == null) return new SCALED_PRESSURE3();
                            if(OnSCALED_PRESSURE3Receive == null) return null;
                            ph.setPack(pack);
                            OnSCALED_PRESSURE3Receive(this, ph, (SCALED_PRESSURE3) pack);
                            if(LOOP) break;
                            return null;
                        case 144:
                            if(pack == null) return new FOLLOW_TARGET();
                            if(OnFOLLOW_TARGETReceive == null) return null;
                            ph.setPack(pack);
                            OnFOLLOW_TARGETReceive(this, ph, (FOLLOW_TARGET) pack);
                            if(LOOP) break;
                            return null;
                        case 146:
                            if(pack == null) return new CONTROL_SYSTEM_STATE();
                            if(OnCONTROL_SYSTEM_STATEReceive == null) return null;
                            ph.setPack(pack);
                            OnCONTROL_SYSTEM_STATEReceive(this, ph, (CONTROL_SYSTEM_STATE) pack);
                            if(LOOP) break;
                            return null;
                        case 147:
                            if(pack == null) return new BATTERY_STATUS();
                            if(OnBATTERY_STATUSReceive == null) return null;
                            ph.setPack(pack);
                            OnBATTERY_STATUSReceive(this, ph, (BATTERY_STATUS) pack);
                            if(LOOP) break;
                            return null;
                        case Channel.PROCESS_CHANNEL_REQEST:
                            if(pack == null) return sendout_packs.Count == 0 ? null : sendout_packs.Dequeue();
                            lock(received_packs) {received_packs.Enqueue(pack); Monitor.PulseAll(received_packs); }
                            return null;
                        case Channel.PROCESS_HOST_REQEST:
                            if(pack == null) return received_packs.Count == 0 ? null : received_packs.Dequeue();
                            lock(sendout_packs) { sendout_packs.Enqueue(pack); Monitor.PulseAll(sendout_packs); }
                            return null;
                        case Channel.PROCESS_RECEIVED:
                            LOOP = true;
                            break;
                    }
                    if(received_packs.Count == 0) return null;
                    pack = received_packs.Dequeue();
                    id = pack.meta.id;
                }
            }
            /**
            *The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot
            *	 hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying
            *	 out the user interface based on the autopilot)*/
            public event HEARTBEATReceiveHandler OnHEARTBEATReceive;
            public delegate void HEARTBEATReceiveHandler(Channel src, Inside ph, HEARTBEAT pack);

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
            public event SYS_STATUSReceiveHandler OnSYS_STATUSReceive;
            public delegate void SYS_STATUSReceiveHandler(Channel src, Inside ph, SYS_STATUS pack);

            /**
            *The system time is the time of the master clock, typically the computer clock of the main onboard computer*/
            public event SYSTEM_TIMEReceiveHandler OnSYSTEM_TIMEReceive;
            public delegate void SYSTEM_TIMEReceiveHandler(Channel src, Inside ph, SYSTEM_TIME pack);

            /**
            *Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
            *	 This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled
            *	 this way*/
            public event POSITION_TARGET_LOCAL_NEDReceiveHandler OnPOSITION_TARGET_LOCAL_NEDReceive;
            public delegate void POSITION_TARGET_LOCAL_NEDReceiveHandler(Channel src, Inside ph, POSITION_TARGET_LOCAL_NED pack);

            /**
            *A ping message either requesting or responding to a ping. This allows to measure the system latencies,
            *	 including serial port, radio modem and UDP connections*/
            public event PINGReceiveHandler OnPINGReceive;
            public delegate void PINGReceiveHandler(Channel src, Inside ph, PING pack);

            /**
            *Request to control this MAV*/
            public event CHANGE_OPERATOR_CONTROLReceiveHandler OnCHANGE_OPERATOR_CONTROLReceive;
            public delegate void CHANGE_OPERATOR_CONTROLReceiveHandler(Channel src, Inside ph, CHANGE_OPERATOR_CONTROL pack);

            /**
            *Accept / deny control of this MAV*/
            public event CHANGE_OPERATOR_CONTROL_ACKReceiveHandler OnCHANGE_OPERATOR_CONTROL_ACKReceive;
            public delegate void CHANGE_OPERATOR_CONTROL_ACKReceiveHandler(Channel src, Inside ph, CHANGE_OPERATOR_CONTROL_ACK pack);

            /**
            *Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple,
            *	 so transmitting the key requires an encrypted channel for true safety*/
            public event AUTH_KEYReceiveHandler OnAUTH_KEYReceive;
            public delegate void AUTH_KEYReceiveHandler(Channel src, Inside ph, AUTH_KEY pack);

            /**
            *THIS INTERFACE IS DEPRECATED. USE COMMAND_LONG with MAV_CMD_DO_SET_MODE INSTEAD. Set the system mode,
            *	 as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall
            *	 aircraft, not only for one component*/
            public event SET_MODEReceiveHandler OnSET_MODEReceive;
            public delegate void SET_MODEReceiveHandler(Channel src, Inside ph, SET_MODE pack);

            /**
            *value[float]. This allows to send a parameter to any other component (such as the GCS) without the need
            *	 of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for
            *	 different autopilots. See also http:qgroundcontrol.org/parameter_interface for a full documentation
            *	 of QGroundControl and IMU code*/
            public event PARAM_REQUEST_READReceiveHandler OnPARAM_REQUEST_READReceive;
            public delegate void PARAM_REQUEST_READReceiveHandler(Channel src, Inside ph, PARAM_REQUEST_READ pack);

            /**
            *Request all parameters of this component. After this request, all parameters are emitted.*/
            public event PARAM_REQUEST_LISTReceiveHandler OnPARAM_REQUEST_LISTReceive;
            public delegate void PARAM_REQUEST_LISTReceiveHandler(Channel src, Inside ph, PARAM_REQUEST_LIST pack);

            /**
            *Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows
            *	 the recipient to keep track of received parameters and allows him to re-request missing parameters after
            *	 a loss or timeout*/
            public event PARAM_VALUEReceiveHandler OnPARAM_VALUEReceive;
            public delegate void PARAM_VALUEReceiveHandler(Channel src, Inside ph, PARAM_VALUE pack);

            /**
            *Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION
            *	 MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component
            *	 should acknowledge the new parameter value by sending a param_value message to all communication partners.
            *	 This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending
            *	 GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message*/
            public event PARAM_SETReceiveHandler OnPARAM_SETReceive;
            public delegate void PARAM_SETReceiveHandler(Channel src, Inside ph, PARAM_SET pack);

            /**
            *The global position, as returned by the Global Positioning System (GPS). This is
            *	 NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
            public event GPS_RAW_INTReceiveHandler OnGPS_RAW_INTReceive;
            public delegate void GPS_RAW_INTReceiveHandler(Channel src, Inside ph, GPS_RAW_INT pack);

            /**
            *The positioning status, as reported by GPS. This message is intended to display status information about
            *	 each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate.
            *	 This message can contain information for up to 20 satellites*/
            public event GPS_STATUSReceiveHandler OnGPS_STATUSReceive;
            public delegate void GPS_STATUSReceiveHandler(Channel src, Inside ph, GPS_STATUS pack);

            /**
            *The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to
            *	 the described unit*/
            public event SCALED_IMUReceiveHandler OnSCALED_IMUReceive;
            public delegate void SCALED_IMUReceiveHandler(Channel src, Inside ph, SCALED_IMU pack);

            /**
            *The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw
            *	 values without any scaling to allow data capture and system debugging*/
            public event RAW_IMUReceiveHandler OnRAW_IMUReceive;
            public delegate void RAW_IMUReceiveHandler(Channel src, Inside ph, RAW_IMU pack);

            /**
            *The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure
            *	 sensor. The sensor values should be the raw, UNSCALED ADC values*/
            public event RAW_PRESSUREReceiveHandler OnRAW_PRESSUREReceive;
            public delegate void RAW_PRESSUREReceiveHandler(Channel src, Inside ph, RAW_PRESSURE pack);

            /**
            *The pressure readings for the typical setup of one absolute and differential pressure sensor. The units
            *	 are as specified in each field*/
            public event SCALED_PRESSUREReceiveHandler OnSCALED_PRESSUREReceive;
            public delegate void SCALED_PRESSUREReceiveHandler(Channel src, Inside ph, SCALED_PRESSURE pack);

            /**
            *The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).*/
            public event ATTITUDEReceiveHandler OnATTITUDEReceive;
            public delegate void ATTITUDEReceiveHandler(Channel src, Inside ph, ATTITUDE pack);

            /**
            *The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
            *	 Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)*/
            public event ATTITUDE_QUATERNIONReceiveHandler OnATTITUDE_QUATERNIONReceive;
            public delegate void ATTITUDE_QUATERNIONReceiveHandler(Channel src, Inside ph, ATTITUDE_QUATERNION pack);

            /**
            *The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
            *	 Z-axis down (aeronautical frame, NED / north-east-down convention*/
            public event LOCAL_POSITION_NEDReceiveHandler OnLOCAL_POSITION_NEDReceive;
            public delegate void LOCAL_POSITION_NEDReceiveHandler(Channel src, Inside ph, LOCAL_POSITION_NED pack);

            /**
            *nt.*/
            public event GLOBAL_POSITION_INTReceiveHandler OnGLOBAL_POSITION_INTReceive;
            public delegate void GLOBAL_POSITION_INTReceiveHandler(Channel src, Inside ph, GLOBAL_POSITION_INT pack);

            /**
            *The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are
            *	 inactive should be set to UINT16_MAX*/
            public event RC_CHANNELS_SCALEDReceiveHandler OnRC_CHANNELS_SCALEDReceive;
            public delegate void RC_CHANNELS_SCALEDReceiveHandler(Channel src, Inside ph, RC_CHANNELS_SCALED pack);

            /**
            *The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
            *	 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification*/
            public event RC_CHANNELS_RAWReceiveHandler OnRC_CHANNELS_RAWReceive;
            public delegate void RC_CHANNELS_RAWReceiveHandler(Channel src, Inside ph, RC_CHANNELS_RAW pack);

            /**
            *The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The
            *	 standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%*/
            public event SERVO_OUTPUT_RAWReceiveHandler OnSERVO_OUTPUT_RAWReceive;
            public delegate void SERVO_OUTPUT_RAWReceiveHandler(Channel src, Inside ph, SERVO_OUTPUT_RAW pack);

            /**
            *Request a partial list of mission items from the system/component. http:qgroundcontrol.org/mavlink/waypoint_protocol.
            *	 If start and end index are the same, just send one waypoint*/
            public event MISSION_REQUEST_PARTIAL_LISTReceiveHandler OnMISSION_REQUEST_PARTIAL_LISTReceive;
            public delegate void MISSION_REQUEST_PARTIAL_LISTReceiveHandler(Channel src, Inside ph, MISSION_REQUEST_PARTIAL_LIST pack);

            /**
            *This message is sent to the MAV to write a partial list. If start index == end index, only one item will
            *	 be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should
            *	 be REJECTED*/
            public event MISSION_WRITE_PARTIAL_LISTReceiveHandler OnMISSION_WRITE_PARTIAL_LISTReceive;
            public delegate void MISSION_WRITE_PARTIAL_LISTReceiveHandler(Channel src, Inside ph, MISSION_WRITE_PARTIAL_LIST pack);

            /**
            *Message encoding a mission item. This message is emitted to announce
            *	 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http:qgroundcontrol.org/mavlink/waypoint_protocol.*/
            public event MISSION_ITEMReceiveHandler OnMISSION_ITEMReceive;
            public delegate void MISSION_ITEMReceiveHandler(Channel src, Inside ph, MISSION_ITEM pack);

            /**
            *Request the information of the mission item with the sequence number seq. The response of the system to
            *	 this message should be a MISSION_ITEM message. http:qgroundcontrol.org/mavlink/waypoint_protoco*/
            public event MISSION_REQUESTReceiveHandler OnMISSION_REQUESTReceive;
            public delegate void MISSION_REQUESTReceiveHandler(Channel src, Inside ph, MISSION_REQUEST pack);

            /**
            *Set the mission item with sequence number seq as current item. This means that the MAV will continue to
            *	 this mission item on the shortest path (not following the mission items in-between)*/
            public event MISSION_SET_CURRENTReceiveHandler OnMISSION_SET_CURRENTReceive;
            public delegate void MISSION_SET_CURRENTReceiveHandler(Channel src, Inside ph, MISSION_SET_CURRENT pack);

            /**
            *Message that announces the sequence number of the current active mission item. The MAV will fly towards
            *	 this mission item*/
            public event MISSION_CURRENTReceiveHandler OnMISSION_CURRENTReceive;
            public delegate void MISSION_CURRENTReceiveHandler(Channel src, Inside ph, MISSION_CURRENT pack);

            /**
            *Request the overall list of mission items from the system/component.*/
            public event MISSION_REQUEST_LISTReceiveHandler OnMISSION_REQUEST_LISTReceive;
            public delegate void MISSION_REQUEST_LISTReceiveHandler(Channel src, Inside ph, MISSION_REQUEST_LIST pack);

            /**
            *This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction.
            *	 The GCS can then request the individual mission item based on the knowledge of the total number of waypoints*/
            public event MISSION_COUNTReceiveHandler OnMISSION_COUNTReceive;
            public delegate void MISSION_COUNTReceiveHandler(Channel src, Inside ph, MISSION_COUNT pack);

            /**
            *Delete all mission items at once.*/
            public event MISSION_CLEAR_ALLReceiveHandler OnMISSION_CLEAR_ALLReceive;
            public delegate void MISSION_CLEAR_ALLReceiveHandler(Channel src, Inside ph, MISSION_CLEAR_ALL pack);

            /**
            *A certain mission item has been reached. The system will either hold this position (or circle on the orbit)
            *	 or (if the autocontinue on the WP was set) continue to the next waypoint*/
            public event MISSION_ITEM_REACHEDReceiveHandler OnMISSION_ITEM_REACHEDReceive;
            public delegate void MISSION_ITEM_REACHEDReceiveHandler(Channel src, Inside ph, MISSION_ITEM_REACHED pack);

            /**
            *Ack message during waypoint handling. The type field states if this message is a positive ack (type=0)
            *	 or if an error happened (type=non-zero)*/
            public event MISSION_ACKReceiveHandler OnMISSION_ACKReceive;
            public delegate void MISSION_ACKReceiveHandler(Channel src, Inside ph, MISSION_ACK pack);

            /**
            *As local waypoints exist, the global waypoint reference allows to transform between the local coordinate
            *	 frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings
            *	 are connected and the MAV should move from in- to outdoor*/
            public event SET_GPS_GLOBAL_ORIGINReceiveHandler OnSET_GPS_GLOBAL_ORIGINReceive;
            public delegate void SET_GPS_GLOBAL_ORIGINReceiveHandler(Channel src, Inside ph, SET_GPS_GLOBAL_ORIGIN pack);

            /**
            *Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) positio*/
            public event GPS_GLOBAL_ORIGINReceiveHandler OnGPS_GLOBAL_ORIGINReceive;
            public delegate void GPS_GLOBAL_ORIGINReceiveHandler(Channel src, Inside ph, GPS_GLOBAL_ORIGIN pack);

            /**
            *Bind a RC channel to a parameter. The parameter should change accoding to the RC channel value.*/
            public event PARAM_MAP_RCReceiveHandler OnPARAM_MAP_RCReceive;
            public delegate void PARAM_MAP_RCReceiveHandler(Channel src, Inside ph, PARAM_MAP_RC pack);

            /**
            *Request the information of the mission item with the sequence number seq. The response of the system to
            *	 this message should be a MISSION_ITEM_INT message. http:qgroundcontrol.org/mavlink/waypoint_protoco*/
            public event MISSION_REQUEST_INTReceiveHandler OnMISSION_REQUEST_INTReceive;
            public delegate void MISSION_REQUEST_INTReceiveHandler(Channel src, Inside ph, MISSION_REQUEST_INT pack);

            /**
            *Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell
            *	 the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national
            *	 or competition regulations*/
            public event SAFETY_SET_ALLOWED_AREAReceiveHandler OnSAFETY_SET_ALLOWED_AREAReceive;
            public delegate void SAFETY_SET_ALLOWED_AREAReceiveHandler(Channel src, Inside ph, SAFETY_SET_ALLOWED_AREA pack);

            /**
            *Read out the safety zone the MAV currently assumes.*/
            public event SAFETY_ALLOWED_AREAReceiveHandler OnSAFETY_ALLOWED_AREAReceive;
            public delegate void SAFETY_ALLOWED_AREAReceiveHandler(Channel src, Inside ph, SAFETY_ALLOWED_AREA pack);

            /**
            *The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
            *	 Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)*/
            public event ATTITUDE_QUATERNION_COVReceiveHandler OnATTITUDE_QUATERNION_COVReceive;
            public delegate void ATTITUDE_QUATERNION_COVReceiveHandler(Channel src, Inside ph, ATTITUDE_QUATERNION_COV pack);

            /**
            *The state of the fixed wing navigation and position controller.*/
            public event NAV_CONTROLLER_OUTPUTReceiveHandler OnNAV_CONTROLLER_OUTPUTReceive;
            public delegate void NAV_CONTROLLER_OUTPUTReceiveHandler(Channel src, Inside ph, NAV_CONTROLLER_OUTPUT pack);

            /**
            *The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed,
            *	 Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE:
            *	 This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized
            *	 for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset*/
            public event GLOBAL_POSITION_INT_COVReceiveHandler OnGLOBAL_POSITION_INT_COVReceive;
            public delegate void GLOBAL_POSITION_INT_COVReceiveHandler(Channel src, Inside ph, GLOBAL_POSITION_INT_COV pack);

            /**
            *The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed,
            *	 Z-axis down (aeronautical frame, NED / north-east-down convention*/
            public event LOCAL_POSITION_NED_COVReceiveHandler OnLOCAL_POSITION_NED_COVReceive;
            public delegate void LOCAL_POSITION_NED_COVReceiveHandler(Channel src, Inside ph, LOCAL_POSITION_NED_COV pack);

            /**
            *The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds:
            *	 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification*/
            public event RC_CHANNELSReceiveHandler OnRC_CHANNELSReceive;
            public delegate void RC_CHANNELSReceiveHandler(Channel src, Inside ph, RC_CHANNELS pack);

            /**
            *THIS INTERFACE IS DEPRECATED. USE SET_MESSAGE_INTERVAL INSTEAD.*/
            public event REQUEST_DATA_STREAMReceiveHandler OnREQUEST_DATA_STREAMReceive;
            public delegate void REQUEST_DATA_STREAMReceiveHandler(Channel src, Inside ph, REQUEST_DATA_STREAM pack);

            /**
            *THIS INTERFACE IS DEPRECATED. USE MESSAGE_INTERVAL INSTEAD.*/
            public event DATA_STREAMReceiveHandler OnDATA_STREAMReceive;
            public delegate void DATA_STREAMReceiveHandler(Channel src, Inside ph, DATA_STREAM pack);

            /**
            *This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature,
            *	 along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as
            *	 boolean values of their*/
            public event MANUAL_CONTROLReceiveHandler OnMANUAL_CONTROLReceive;
            public delegate void MANUAL_CONTROLReceiveHandler(Channel src, Inside ph, MANUAL_CONTROL pack);

            /**
            *The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value
            *	 of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released
            *	 back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds:
            *	 100%. Individual receivers/transmitters might violate this specification*/
            public event RC_CHANNELS_OVERRIDEReceiveHandler OnRC_CHANNELS_OVERRIDEReceive;
            public delegate void RC_CHANNELS_OVERRIDEReceiveHandler(Channel src, Inside ph, RC_CHANNELS_OVERRIDE pack);

            /**
            *Message encoding a mission item. This message is emitted to announce
            *	 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See alsohttp:qgroundcontrol.org/mavlink/waypoint_protocol.*/
            public event MISSION_ITEM_INTReceiveHandler OnMISSION_ITEM_INTReceive;
            public delegate void MISSION_ITEM_INTReceiveHandler(Channel src, Inside ph, MISSION_ITEM_INT pack);

            /**
            *Metrics typically displayed on a HUD for fixed wing aircraft*/
            public event VFR_HUDReceiveHandler OnVFR_HUDReceive;
            public delegate void VFR_HUDReceiveHandler(Channel src, Inside ph, VFR_HUD pack);

            /**
            *Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value*/
            public event COMMAND_INTReceiveHandler OnCOMMAND_INTReceive;
            public delegate void COMMAND_INTReceiveHandler(Channel src, Inside ph, COMMAND_INT pack);

            /**
            *Send a command with up to seven parameters to the MAV*/
            public event COMMAND_LONGReceiveHandler OnCOMMAND_LONGReceive;
            public delegate void COMMAND_LONGReceiveHandler(Channel src, Inside ph, COMMAND_LONG pack);

            /**
            *Report status of a command. Includes feedback whether the command was executed.*/
            public event COMMAND_ACKReceiveHandler OnCOMMAND_ACKReceive;
            public delegate void COMMAND_ACKReceiveHandler(Channel src, Inside ph, COMMAND_ACK pack);

            /**
            *Setpoint in roll, pitch, yaw and thrust from the operator*/
            public event MANUAL_SETPOINTReceiveHandler OnMANUAL_SETPOINTReceive;
            public delegate void MANUAL_SETPOINTReceiveHandler(Channel src, Inside ph, MANUAL_SETPOINT pack);

            public event SET_ATTITUDE_TARGETReceiveHandler OnSET_ATTITUDE_TARGETReceive;
            public delegate void SET_ATTITUDE_TARGETReceiveHandler(Channel src, Inside ph, SET_ATTITUDE_TARGET pack);

            /**
            *Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match
            *	 the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way*/
            public event ATTITUDE_TARGETReceiveHandler OnATTITUDE_TARGETReceive;
            public delegate void ATTITUDE_TARGETReceiveHandler(Channel src, Inside ph, ATTITUDE_TARGET pack);

            /**
            *Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller
            *	 to command the vehicle (manual controller or other system)*/
            public event SET_POSITION_TARGET_LOCAL_NEDReceiveHandler OnSET_POSITION_TARGET_LOCAL_NEDReceive;
            public delegate void SET_POSITION_TARGET_LOCAL_NEDReceiveHandler(Channel src, Inside ph, SET_POSITION_TARGET_LOCAL_NED pack);

            /**
            *Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84).
            *	 Used by an external controller to command the vehicle (manual controller or other system)*/
            public event SET_POSITION_TARGET_GLOBAL_INTReceiveHandler OnSET_POSITION_TARGET_GLOBAL_INTReceive;
            public delegate void SET_POSITION_TARGET_GLOBAL_INTReceiveHandler(Channel src, Inside ph, SET_POSITION_TARGET_GLOBAL_INT pack);

            /**
            *Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
            *	 This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled
            *	 this way*/
            public event POSITION_TARGET_GLOBAL_INTReceiveHandler OnPOSITION_TARGET_GLOBAL_INTReceive;
            public delegate void POSITION_TARGET_GLOBAL_INTReceiveHandler(Channel src, Inside ph, POSITION_TARGET_GLOBAL_INT pack);

            /**
            *The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate
            *	 frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down
            *	 convention*/
            public event LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceiveHandler OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive;
            public delegate void LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceiveHandler(Channel src, Inside ph, LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET pack);

            /**
            *DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please
            *	 use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput
            *	 applications such as hardware in the loop simulations*/
            public event HIL_STATEReceiveHandler OnHIL_STATEReceive;
            public delegate void HIL_STATEReceiveHandler(Channel src, Inside ph, HIL_STATE pack);

            /**
            *Sent from autopilot to simulation. Hardware in the loop control outputs*/
            public event HIL_CONTROLSReceiveHandler OnHIL_CONTROLSReceive;
            public delegate void HIL_CONTROLSReceiveHandler(Channel src, Inside ph, HIL_CONTROLS pack);

            /**
            *Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation
            *	 is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might
            *	 violate this specification*/
            public event HIL_RC_INPUTS_RAWReceiveHandler OnHIL_RC_INPUTS_RAWReceive;
            public delegate void HIL_RC_INPUTS_RAWReceiveHandler(Channel src, Inside ph, HIL_RC_INPUTS_RAW pack);

            /**
            *Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS*/
            public event HIL_ACTUATOR_CONTROLSReceiveHandler OnHIL_ACTUATOR_CONTROLSReceive;
            public delegate void HIL_ACTUATOR_CONTROLSReceiveHandler(Channel src, Inside ph, HIL_ACTUATOR_CONTROLS pack);

            /**
            *Optical flow from a flow sensor (e.g. optical mouse sensor)*/
            public event OPTICAL_FLOWReceiveHandler OnOPTICAL_FLOWReceive;
            public delegate void OPTICAL_FLOWReceiveHandler(Channel src, Inside ph, OPTICAL_FLOW pack);

            public event GLOBAL_VISION_POSITION_ESTIMATEReceiveHandler OnGLOBAL_VISION_POSITION_ESTIMATEReceive;
            public delegate void GLOBAL_VISION_POSITION_ESTIMATEReceiveHandler(Channel src, Inside ph, GLOBAL_VISION_POSITION_ESTIMATE pack);

            public event VISION_POSITION_ESTIMATEReceiveHandler OnVISION_POSITION_ESTIMATEReceive;
            public delegate void VISION_POSITION_ESTIMATEReceiveHandler(Channel src, Inside ph, VISION_POSITION_ESTIMATE pack);

            public event VISION_SPEED_ESTIMATEReceiveHandler OnVISION_SPEED_ESTIMATEReceive;
            public delegate void VISION_SPEED_ESTIMATEReceiveHandler(Channel src, Inside ph, VISION_SPEED_ESTIMATE pack);

            public event VICON_POSITION_ESTIMATEReceiveHandler OnVICON_POSITION_ESTIMATEReceive;
            public delegate void VICON_POSITION_ESTIMATEReceiveHandler(Channel src, Inside ph, VICON_POSITION_ESTIMATE pack);

            /**
            *The IMU readings in SI units in NED body frame*/
            public event HIGHRES_IMUReceiveHandler OnHIGHRES_IMUReceive;
            public delegate void HIGHRES_IMUReceiveHandler(Channel src, Inside ph, HIGHRES_IMU pack);

            /**
            *Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)*/
            public event OPTICAL_FLOW_RADReceiveHandler OnOPTICAL_FLOW_RADReceive;
            public delegate void OPTICAL_FLOW_RADReceiveHandler(Channel src, Inside ph, OPTICAL_FLOW_RAD pack);

            /**
            *The IMU readings in SI units in NED body frame*/
            public event HIL_SENSORReceiveHandler OnHIL_SENSORReceive;
            public delegate void HIL_SENSORReceiveHandler(Channel src, Inside ph, HIL_SENSOR pack);

            /**
            *Status of simulation environment, if used*/
            public event SIM_STATEReceiveHandler OnSIM_STATEReceive;
            public delegate void SIM_STATEReceiveHandler(Channel src, Inside ph, SIM_STATE pack);

            /**
            *Status generated by radio and injected into MAVLink stream.*/
            public event RADIO_STATUSReceiveHandler OnRADIO_STATUSReceive;
            public delegate void RADIO_STATUSReceiveHandler(Channel src, Inside ph, RADIO_STATUS pack);

            /**
            *File transfer message*/
            public event FILE_TRANSFER_PROTOCOLReceiveHandler OnFILE_TRANSFER_PROTOCOLReceive;
            public delegate void FILE_TRANSFER_PROTOCOLReceiveHandler(Channel src, Inside ph, FILE_TRANSFER_PROTOCOL pack);

            /**
            *Time synchronization message.*/
            public event TIMESYNCReceiveHandler OnTIMESYNCReceive;
            public delegate void TIMESYNCReceiveHandler(Channel src, Inside ph, TIMESYNC pack);

            /**
            *Camera-IMU triggering and synchronisation message.*/
            public event CAMERA_TRIGGERReceiveHandler OnCAMERA_TRIGGERReceive;
            public delegate void CAMERA_TRIGGERReceiveHandler(Channel src, Inside ph, CAMERA_TRIGGER pack);

            /**
            *The global position, as returned by the Global Positioning System (GPS). This is
            *	 NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
            public event HIL_GPSReceiveHandler OnHIL_GPSReceive;
            public delegate void HIL_GPSReceiveHandler(Channel src, Inside ph, HIL_GPS pack);

            /**
            *Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)*/
            public event HIL_OPTICAL_FLOWReceiveHandler OnHIL_OPTICAL_FLOWReceive;
            public delegate void HIL_OPTICAL_FLOWReceiveHandler(Channel src, Inside ph, HIL_OPTICAL_FLOW pack);

            /**
            *Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful
            *	 for high throughput applications such as hardware in the loop simulations*/
            public event HIL_STATE_QUATERNIONReceiveHandler OnHIL_STATE_QUATERNIONReceive;
            public delegate void HIL_STATE_QUATERNIONReceiveHandler(Channel src, Inside ph, HIL_STATE_QUATERNION pack);

            /**
            *The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to
            *	 the described unit*/
            public event SCALED_IMU2ReceiveHandler OnSCALED_IMU2Receive;
            public delegate void SCALED_IMU2ReceiveHandler(Channel src, Inside ph, SCALED_IMU2 pack);

            /**
            *Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END
            *	 is called*/
            public event LOG_REQUEST_LISTReceiveHandler OnLOG_REQUEST_LISTReceive;
            public delegate void LOG_REQUEST_LISTReceiveHandler(Channel src, Inside ph, LOG_REQUEST_LIST pack);

            /**
            *Reply to LOG_REQUEST_LIST*/
            public event LOG_ENTRYReceiveHandler OnLOG_ENTRYReceive;
            public delegate void LOG_ENTRYReceiveHandler(Channel src, Inside ph, LOG_ENTRY pack);

            /**
            *Request a chunk of a log*/
            public event LOG_REQUEST_DATAReceiveHandler OnLOG_REQUEST_DATAReceive;
            public delegate void LOG_REQUEST_DATAReceiveHandler(Channel src, Inside ph, LOG_REQUEST_DATA pack);

            /**
            *Reply to LOG_REQUEST_DATA*/
            public event LOG_DATAReceiveHandler OnLOG_DATAReceive;
            public delegate void LOG_DATAReceiveHandler(Channel src, Inside ph, LOG_DATA pack);

            /**
            *Erase all logs*/
            public event LOG_ERASEReceiveHandler OnLOG_ERASEReceive;
            public delegate void LOG_ERASEReceiveHandler(Channel src, Inside ph, LOG_ERASE pack);

            /**
            *Stop log transfer and resume normal logging*/
            public event LOG_REQUEST_ENDReceiveHandler OnLOG_REQUEST_ENDReceive;
            public delegate void LOG_REQUEST_ENDReceiveHandler(Channel src, Inside ph, LOG_REQUEST_END pack);

            /**
            *data for injecting into the onboard GPS (used for DGPS)*/
            public event GPS_INJECT_DATAReceiveHandler OnGPS_INJECT_DATAReceive;
            public delegate void GPS_INJECT_DATAReceiveHandler(Channel src, Inside ph, GPS_INJECT_DATA pack);

            /**
            *Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame).*/
            public event GPS2_RAWReceiveHandler OnGPS2_RAWReceive;
            public delegate void GPS2_RAWReceiveHandler(Channel src, Inside ph, GPS2_RAW pack);

            /**
            *Power supply status*/
            public event POWER_STATUSReceiveHandler OnPOWER_STATUSReceive;
            public delegate void POWER_STATUSReceiveHandler(Channel src, Inside ph, POWER_STATUS pack);

            /**
            *Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or
            *	 telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages
            *	 or change the devices settings. A message with zero bytes can be used to change just the baudrate*/
            public event SERIAL_CONTROLReceiveHandler OnSERIAL_CONTROLReceive;
            public delegate void SERIAL_CONTROLReceiveHandler(Channel src, Inside ph, SERIAL_CONTROL pack);

            /**
            *RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting*/
            public event GPS_RTKReceiveHandler OnGPS_RTKReceive;
            public delegate void GPS_RTKReceiveHandler(Channel src, Inside ph, GPS_RTK pack);

            /**
            *RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting*/
            public event GPS2_RTKReceiveHandler OnGPS2_RTKReceive;
            public delegate void GPS2_RTKReceiveHandler(Channel src, Inside ph, GPS2_RTK pack);

            /**
            *The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described
            *	 unit*/
            public event SCALED_IMU3ReceiveHandler OnSCALED_IMU3Receive;
            public delegate void SCALED_IMU3ReceiveHandler(Channel src, Inside ph, SCALED_IMU3 pack);

            public event DATA_TRANSMISSION_HANDSHAKEReceiveHandler OnDATA_TRANSMISSION_HANDSHAKEReceive;
            public delegate void DATA_TRANSMISSION_HANDSHAKEReceiveHandler(Channel src, Inside ph, DATA_TRANSMISSION_HANDSHAKE pack);

            public event ENCAPSULATED_DATAReceiveHandler OnENCAPSULATED_DATAReceive;
            public delegate void ENCAPSULATED_DATAReceiveHandler(Channel src, Inside ph, ENCAPSULATED_DATA pack);

            public event DISTANCE_SENSORReceiveHandler OnDISTANCE_SENSORReceive;
            public delegate void DISTANCE_SENSORReceiveHandler(Channel src, Inside ph, DISTANCE_SENSOR pack);

            /**
            *Request for terrain data and terrain status*/
            public event TERRAIN_REQUESTReceiveHandler OnTERRAIN_REQUESTReceive;
            public delegate void TERRAIN_REQUESTReceiveHandler(Channel src, Inside ph, TERRAIN_REQUEST pack);

            /**
            *Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUES*/
            public event TERRAIN_DATAReceiveHandler OnTERRAIN_DATAReceive;
            public delegate void TERRAIN_DATAReceiveHandler(Channel src, Inside ph, TERRAIN_DATA pack);

            /**
            *Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle
            *	 has all terrain data needed for a mission*/
            public event TERRAIN_CHECKReceiveHandler OnTERRAIN_CHECKReceive;
            public delegate void TERRAIN_CHECKReceiveHandler(Channel src, Inside ph, TERRAIN_CHECK pack);

            /**
            *Response from a TERRAIN_CHECK request*/
            public event TERRAIN_REPORTReceiveHandler OnTERRAIN_REPORTReceive;
            public delegate void TERRAIN_REPORTReceiveHandler(Channel src, Inside ph, TERRAIN_REPORT pack);

            /**
            *Barometer readings for 2nd barometer*/
            public event SCALED_PRESSURE2ReceiveHandler OnSCALED_PRESSURE2Receive;
            public delegate void SCALED_PRESSURE2ReceiveHandler(Channel src, Inside ph, SCALED_PRESSURE2 pack);

            /**
            *Motion capture attitude and position*/
            public event ATT_POS_MOCAPReceiveHandler OnATT_POS_MOCAPReceive;
            public delegate void ATT_POS_MOCAPReceiveHandler(Channel src, Inside ph, ATT_POS_MOCAP pack);

            /**
            *Set the vehicle attitude and body angular rates.*/
            public event SET_ACTUATOR_CONTROL_TARGETReceiveHandler OnSET_ACTUATOR_CONTROL_TARGETReceive;
            public delegate void SET_ACTUATOR_CONTROL_TARGETReceiveHandler(Channel src, Inside ph, SET_ACTUATOR_CONTROL_TARGET pack);

            /**
            *Set the vehicle attitude and body angular rates.*/
            public event ACTUATOR_CONTROL_TARGETReceiveHandler OnACTUATOR_CONTROL_TARGETReceive;
            public delegate void ACTUATOR_CONTROL_TARGETReceiveHandler(Channel src, Inside ph, ACTUATOR_CONTROL_TARGET pack);

            /**
            *The current system altitude.*/
            public event ALTITUDEReceiveHandler OnALTITUDEReceive;
            public delegate void ALTITUDEReceiveHandler(Channel src, Inside ph, ALTITUDE pack);

            /**
            *The autopilot is requesting a resource (file, binary, other type of data)*/
            public event RESOURCE_REQUESTReceiveHandler OnRESOURCE_REQUESTReceive;
            public delegate void RESOURCE_REQUESTReceiveHandler(Channel src, Inside ph, RESOURCE_REQUEST pack);

            /**
            *Barometer readings for 3rd barometer*/
            public event SCALED_PRESSURE3ReceiveHandler OnSCALED_PRESSURE3Receive;
            public delegate void SCALED_PRESSURE3ReceiveHandler(Channel src, Inside ph, SCALED_PRESSURE3 pack);

            /**
            *current motion information from a designated system*/
            public event FOLLOW_TARGETReceiveHandler OnFOLLOW_TARGETReceive;
            public delegate void FOLLOW_TARGETReceiveHandler(Channel src, Inside ph, FOLLOW_TARGET pack);

            /**
            *The smoothed, monotonic system state used to feed the control loops of the system.*/
            public event CONTROL_SYSTEM_STATEReceiveHandler OnCONTROL_SYSTEM_STATEReceive;
            public delegate void CONTROL_SYSTEM_STATEReceiveHandler(Channel src, Inside ph, CONTROL_SYSTEM_STATE pack);

            /**
            *Battery information*/
            public event BATTERY_STATUSReceiveHandler OnBATTERY_STATUSReceive;
            public delegate void BATTERY_STATUSReceiveHandler(Channel src, Inside ph, BATTERY_STATUS pack);
        }


        public enum MAV_TYPE
        {
            MAV_TYPE_GENERIC = 0, //Generic micro air vehicle.
            MAV_TYPE_FIXED_WING = 1, //Fixed wing aircraft.
            MAV_TYPE_QUADROTOR = 2, //Quadrotor
            MAV_TYPE_COAXIAL = 3, //Coaxial helicopter
            MAV_TYPE_HELICOPTER = 4, //Normal helicopter with tail rotor.
            MAV_TYPE_ANTENNA_TRACKER = 5, //Ground installation
            MAV_TYPE_GCS = 6, //Operator control unit / ground control station
            MAV_TYPE_AIRSHIP = 7, //Airship, controlled
            MAV_TYPE_FREE_BALLOON = 8, //Free balloon, uncontrolled
            MAV_TYPE_ROCKET = 9, //Rocket
            MAV_TYPE_GROUND_ROVER = 10, //Ground rover
            MAV_TYPE_SURFACE_BOAT = 11, //Surface vessel, boat, ship
            MAV_TYPE_SUBMARINE = 12, //Submarine
            MAV_TYPE_HEXAROTOR = 13, //Hexarotor
            MAV_TYPE_OCTOROTOR = 14, //Octorotor
            MAV_TYPE_TRICOPTER = 15, //Tricopter
            MAV_TYPE_FLAPPING_WING = 16, //Flapping wing
            MAV_TYPE_KITE = 17, //Kite
            MAV_TYPE_ONBOARD_CONTROLLER = 18, //Onboard companion controller
            MAV_TYPE_VTOL_DUOROTOR = 19, //Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
            MAV_TYPE_VTOL_QUADROTOR = 20, //Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
            MAV_TYPE_VTOL_TILTROTOR = 21, //Tiltrotor VTOL
            MAV_TYPE_VTOL_RESERVED2 = 22, //VTOL reserved 2
            MAV_TYPE_VTOL_RESERVED3 = 23, //VTOL reserved 3
            MAV_TYPE_VTOL_RESERVED4 = 24, //VTOL reserved 4
            MAV_TYPE_VTOL_RESERVED5 = 25, //VTOL reserved 5
            MAV_TYPE_GIMBAL = 26, //Onboard gimbal
            MAV_TYPE_ADSB = 27, //Onboard ADSB peripheral
            MAV_TYPE_PARAFOIL = 28 //Steerable, nonrigid airfoil
        }

        /**
        *Micro air vehicle / autopilot classes. This identifies the individual model.*/

        public enum MAV_AUTOPILOT
        {
            MAV_AUTOPILOT_GENERIC = 0, //Generic autopilot, full support for everything
            MAV_AUTOPILOT_RESERVED = 1, //Reserved for future use.
            MAV_AUTOPILOT_SLUGS = 2, //SLUGS autopilot, http:slugsuav.soe.ucsc.edu
            MAV_AUTOPILOT_ARDUPILOTMEGA = 3, //ArduPilotMega / ArduCopter, http:diydrones.com
            MAV_AUTOPILOT_OPENPILOT = 4, //OpenPilot, http:openpilot.org
            MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5, //Generic autopilot only supporting simple waypoints
            MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6, //Generic autopilot supporting waypoints and other simple navigation commands
            MAV_AUTOPILOT_GENERIC_MISSION_FULL = 7, //Generic autopilot supporting the full mission command set
            MAV_AUTOPILOT_INVALID = 8, //No valid autopilot, e.g. a GCS or other MAVLink component
            MAV_AUTOPILOT_PPZ = 9, //PPZ UAV - http:nongnu.org/paparazzi
            MAV_AUTOPILOT_UDB = 10, //UAV Dev Board
            MAV_AUTOPILOT_FP = 11, //FlexiPilot
            MAV_AUTOPILOT_PX4 = 12, //PX4 Autopilot - http:pixhawk.ethz.ch/px4/
            MAV_AUTOPILOT_SMACCMPILOT = 13, //SMACCMPilot - http:smaccmpilot.org
            MAV_AUTOPILOT_AUTOQUAD = 14, //AutoQuad -- http:autoquad.org
            MAV_AUTOPILOT_ARMAZILA = 15, //Armazila -- http:armazila.com
            MAV_AUTOPILOT_AEROB = 16, //Aerob -- http:aerob.ru
            MAV_AUTOPILOT_ASLUAV = 17, //ASLUAV autopilot -- http:www.asl.ethz.ch
            MAV_AUTOPILOT_SMARTAP = 18 //SmartAP Autopilot - http:sky-drones.com
        }

        /**
        *These flags encode the MAV mode.*/
        [FlagsAttribute]
        public enum MAV_MODE_FLAG
        {
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1, //0b00000001 Reserved for future use.
            /**
            ** 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should
            *		 * not be used for stable implementations*/
            MAV_MODE_FLAG_TEST_ENABLED = 2,
            /**
            ** 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not,
            *		 * depends on the actual implementation*/
            MAV_MODE_FLAG_AUTO_ENABLED = 4,
            MAV_MODE_FLAG_GUIDED_ENABLED = 8, //0b00001000 guided mode enabled, system flies waypoints / mission items.
            /**
            ** 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further
            *		 * control inputs to move around*/
            MAV_MODE_FLAG_STABILIZE_ENABLED = 16,
            /**
            ** 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software
            *		 * is full operational*/
            MAV_MODE_FLAG_HIL_ENABLED = 32,
            MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64, //0b01000000 remote control input is enabled.
            /**
            *0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional
            *				 note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM
            *				 shall be used instead. The flag can still be used to report the armed state*/
            MAV_MODE_FLAG_SAFETY_ARMED = 128
        }


        public enum MAV_STATE
        {
            MAV_STATE_UNINIT = 0, //Uninitialized system, state is unknown.
            MAV_STATE_BOOT = 1, //System is booting up.
            MAV_STATE_CALIBRATING = 2, //System is calibrating and not flight-ready.
            MAV_STATE_STANDBY = 3, //System is grounded and on standby. It can be launched any time.
            MAV_STATE_ACTIVE = 4, //System is active and might be already airborne. Motors are engaged.
            MAV_STATE_CRITICAL = 5, //System is in a non-normal flight mode. It can however still navigate.
            /**
            *System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in
            *	 mayday and going down*/
            MAV_STATE_EMERGENCY = 6,
            MAV_STATE_POWEROFF = 7, //System just initialized its power-down sequence, will shut down now.
            MAV_STATE_FLIGHT_TERMINATION = 8 //System is terminating itself.
        }

        /**
        *These encode the sensors whose status is sent as part of the SYS_STATUS message.*/
        [FlagsAttribute]
        public enum MAV_SYS_STATUS_SENSOR
        {
            MAV_SYS_STATUS_SENSOR_3D_GYRO = 1, //0x01 3D gyro
            MAV_SYS_STATUS_SENSOR_3D_ACCEL = 2, //0x02 3D accelerometer
            MAV_SYS_STATUS_SENSOR_3D_MAG = 4, //0x04 3D magnetometer
            MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 8, //0x08 absolute pressure
            MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE = 16, //0x10 differential pressure
            MAV_SYS_STATUS_SENSOR_GPS = 32, //0x20 GPS
            MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW = 64, //0x40 optical flow
            MAV_SYS_STATUS_SENSOR_VISION_POSITION = 128, //0x80 computer vision position
            MAV_SYS_STATUS_SENSOR_LASER_POSITION = 256, //0x100 laser based position
            MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH = 512, //0x200 external ground truth (Vicon or Leica)
            MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL = 1024, //0x400 3D angular rate control
            MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048, //0x800 attitude stabilization
            MAV_SYS_STATUS_SENSOR_YAW_POSITION = 4096, //0x1000 yaw position
            MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL = 8192, //0x2000 z/altitude control
            MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL = 16384, //0x4000 x/y position control
            MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS = 32768, //0x8000 motor outputs / control
            MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 65536, //0x10000 rc receiver
            MAV_SYS_STATUS_SENSOR_3D_GYRO2 = 131072, //0x20000 2nd 3D gyro
            MAV_SYS_STATUS_SENSOR_3D_ACCEL2 = 262144, //0x40000 2nd 3D accelerometer
            MAV_SYS_STATUS_SENSOR_3D_MAG2 = 524288, //0x80000 2nd 3D magnetometer
            MAV_SYS_STATUS_GEOFENCE = 1048576, //0x100000 geofence
            MAV_SYS_STATUS_AHRS = 2097152, //0x200000 AHRS subsystem health
            MAV_SYS_STATUS_TERRAIN = 4194304, //0x400000 Terrain subsystem health
            MAV_SYS_STATUS_REVERSE_MOTOR = 8388608, //0x800000 Motors are reversed
            MAV_SYS_STATUS_LOGGING = 16777216, //0x1000000 Logging
            MAV_SYS_STATUS_SENSOR_BATTERY = 33554432 //0x2000000 Battery
        }


        public enum MAV_FRAME
        {
            /**
            *Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude,
            *				 third value / z: positive altitude over mean sea level (MSL*/
            MAV_FRAME_GLOBAL = 0,
            MAV_FRAME_LOCAL_NED = 1, //Local coordinate frame, Z-up (x: north, y: east, z: down).
            MAV_FRAME_MISSION = 2, //NOT a coordinate frame, indicates a mission command.
            /**
            ** Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home
            *		 * position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude
            *		 * with 0 being at the altitude of the home location*/
            MAV_FRAME_GLOBAL_RELATIVE_ALT = 3,
            MAV_FRAME_LOCAL_ENU = 4, //Local coordinate frame, Z-down (x: east, y: north, z: up)
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
            MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
        }

        /**
        *These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
        *	 simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.*/

        public enum MAV_MODE
        {
            MAV_MODE_PREFLIGHT = 0, //System is not ready to fly, booting, calibrating, etc. No flag is set.
            MAV_MODE_MANUAL_DISARMED = 64, //System is allowed to be active, under manual (RC) control, no stabilization
            MAV_MODE_TEST_DISARMED = 66, //UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
            MAV_MODE_STABILIZE_DISARMED = 80, //System is allowed to be active, under assisted RC control.
            MAV_MODE_GUIDED_DISARMED = 88, //System is allowed to be active, under autonomous control, manual setpoint
            /**
            ** System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard
            *		 * and not pre-programmed by waypoints*/
            MAV_MODE_AUTO_DISARMED = 92,
            MAV_MODE_MANUAL_ARMED = 192, //System is allowed to be active, under manual (RC) control, no stabilization
            MAV_MODE_TEST_ARMED = 194, //UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
            MAV_MODE_STABILIZE_ARMED = 208, //System is allowed to be active, under assisted RC control.
            MAV_MODE_GUIDED_ARMED = 216, //System is allowed to be active, under autonomous control, manual setpoint
            /**
            ** System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard
            *		 * and not pre-programmed by waypoints*/
            MAV_MODE_AUTO_ARMED = 220
        }

        internal static MAV_MODE en__j(ulong id)
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
            throw  new ArgumentException("Unknown enum ID ");
        }
        internal static uint id__j(MAV_MODE en)
        {
            switch(en)
            {
                case MAV_MODE.MAV_MODE_PREFLIGHT:
                    return 0;
                case MAV_MODE.MAV_MODE_MANUAL_DISARMED:
                    return 1;
                case MAV_MODE.MAV_MODE_TEST_DISARMED:
                    return 2;
                case MAV_MODE.MAV_MODE_STABILIZE_DISARMED:
                    return 3;
                case MAV_MODE.MAV_MODE_GUIDED_DISARMED:
                    return 4;
                case MAV_MODE.MAV_MODE_AUTO_DISARMED:
                    return 5;
                case MAV_MODE.MAV_MODE_MANUAL_ARMED:
                    return 6;
                case MAV_MODE.MAV_MODE_TEST_ARMED:
                    return 7;
                case MAV_MODE.MAV_MODE_STABILIZE_ARMED:
                    return 8;
                case MAV_MODE.MAV_MODE_GUIDED_ARMED:
                    return 9;
                case MAV_MODE.MAV_MODE_AUTO_ARMED:
                    return 10;
                default:
                    throw  new ArgumentException("Unknown enum " + en);
            }
        }

        /**
        *Specifies the datatype of a MAVLink parameter.*/

        public enum MAV_PARAM_TYPE
        {
            MAV_PARAM_TYPE_UINT8 = 1, //8-bit unsigned integer
            MAV_PARAM_TYPE_INT8 = 2, //8-bit signed integer
            MAV_PARAM_TYPE_UINT16 = 3, //16-bit unsigned integer
            MAV_PARAM_TYPE_INT16 = 4, //16-bit signed integer
            MAV_PARAM_TYPE_UINT32 = 5, //32-bit unsigned integer
            MAV_PARAM_TYPE_INT32 = 6, //32-bit signed integer
            MAV_PARAM_TYPE_UINT64 = 7, //64-bit unsigned integer
            MAV_PARAM_TYPE_INT64 = 8, //64-bit signed integer
            MAV_PARAM_TYPE_REAL32 = 9, //32-bit floating-point
            MAV_PARAM_TYPE_REAL64 = 10 //64-bit floating-point
        }

        /**
        *Type of GPS fix*/

        public enum GPS_FIX_TYPE
        {
            GPS_FIX_TYPE_NO_GPS = 0, //No GPS connected
            GPS_FIX_TYPE_NO_FIX = 1, //No position information, GPS is connected
            GPS_FIX_TYPE_2D_FIX = 2, //2D position
            GPS_FIX_TYPE_3D_FIX = 3, //3D position
            GPS_FIX_TYPE_DGPS = 4, //DGPS/SBAS aided 3D position
            GPS_FIX_TYPE_RTK_FLOAT = 5, //RTK float, 3D position
            GPS_FIX_TYPE_RTK_FIXED = 6, //RTK Fixed, 3D position
            GPS_FIX_TYPE_STATIC = 7, //Static fixed, typically used for base stations
            GPS_FIX_TYPE_PPP = 8 //PPP, 3D position.
        }

        /**
        *Type of mission items being requested/sent in mission protocol.*/

        public enum MAV_MISSION_TYPE
        {
            MAV_MISSION_TYPE_MISSION = 0, //Items are mission commands for main mission.
            MAV_MISSION_TYPE_FENCE = 1, //Specifies GeoFence area(s). Items are MAV_CMD_FENCE_ GeoFence items.
            /**
            ** Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_RALLY_POINT
            *		 * rally point items*/
            MAV_MISSION_TYPE_RALLY = 2,
            MAV_MISSION_TYPE_ALL = 255 //Only used in MISSION_CLEAR_ALL to clear all mission types.
        }

        internal static MAV_MISSION_TYPE en__X(ulong id)
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
            throw  new ArgumentException("Unknown enum ID ");
        }
        /**
        *Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script.
        *	 If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows:
        *	 Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what
        *	 ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data*/

        public enum MAV_CMD
        {
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
            ** Mission command to reset Maximum Power Point Tracker (MPPT)
            *		 * 1	MPPT number
            *		 * 2	Empty
            *		 * 3	Empty
            *		 * 4	Empty
            *		 * 5	Empty
            *		 * 6	Empty
            *		 * 7	Empty*/
            MAV_CMD_RESET_MPPT = 40001,
            /**
            ** Mission command to perform a power cycle on payload
            *		 * 1	Complete power cycle
            *		 * 2	VISensor power cycle
            *		 * 3	Empty
            *		 * 4	Empty
            *		 * 5	Empty
            *		 * 6	Empty
            *		 * 7	Empty*/
            MAV_CMD_PAYLOAD_CONTROL = 40002
        }

        internal static MAV_CMD en__m(ulong id)
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
                    return MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF;
                case 18:
                    return MAV_CMD.MAV_CMD_NAV_VTOL_LAND;
                case 19:
                    return MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE;
                case 20:
                    return MAV_CMD.MAV_CMD_NAV_DELAY;
                case 21:
                    return MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE;
                case 22:
                    return MAV_CMD.MAV_CMD_NAV_LAST;
                case 23:
                    return MAV_CMD.MAV_CMD_CONDITION_DELAY;
                case 24:
                    return MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT;
                case 25:
                    return MAV_CMD.MAV_CMD_CONDITION_DISTANCE;
                case 26:
                    return MAV_CMD.MAV_CMD_CONDITION_YAW;
                case 27:
                    return MAV_CMD.MAV_CMD_CONDITION_LAST;
                case 28:
                    return MAV_CMD.MAV_CMD_DO_SET_MODE;
                case 29:
                    return MAV_CMD.MAV_CMD_DO_JUMP;
                case 30:
                    return MAV_CMD.MAV_CMD_DO_CHANGE_SPEED;
                case 31:
                    return MAV_CMD.MAV_CMD_DO_SET_HOME;
                case 32:
                    return MAV_CMD.MAV_CMD_DO_SET_PARAMETER;
                case 33:
                    return MAV_CMD.MAV_CMD_DO_SET_RELAY;
                case 34:
                    return MAV_CMD.MAV_CMD_DO_REPEAT_RELAY;
                case 35:
                    return MAV_CMD.MAV_CMD_DO_SET_SERVO;
                case 36:
                    return MAV_CMD.MAV_CMD_DO_REPEAT_SERVO;
                case 37:
                    return MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION;
                case 38:
                    return MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE;
                case 39:
                    return MAV_CMD.MAV_CMD_DO_LAND_START;
                case 40:
                    return MAV_CMD.MAV_CMD_DO_RALLY_LAND;
                case 41:
                    return MAV_CMD.MAV_CMD_DO_GO_AROUND;
                case 42:
                    return MAV_CMD.MAV_CMD_DO_REPOSITION;
                case 43:
                    return MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE;
                case 44:
                    return MAV_CMD.MAV_CMD_DO_SET_REVERSE;
                case 45:
                    return MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO;
                case 46:
                    return MAV_CMD.MAV_CMD_DO_SET_ROI;
                case 47:
                    return MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE;
                case 48:
                    return MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL;
                case 49:
                    return MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE;
                case 50:
                    return MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL;
                case 51:
                    return MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST;
                case 52:
                    return MAV_CMD.MAV_CMD_DO_FENCE_ENABLE;
                case 53:
                    return MAV_CMD.MAV_CMD_DO_PARACHUTE;
                case 54:
                    return MAV_CMD.MAV_CMD_DO_MOTOR_TEST;
                case 55:
                    return MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT;
                case 56:
                    return MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED;
                case 57:
                    return MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
                case 58:
                    return MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT;
                case 59:
                    return MAV_CMD.MAV_CMD_DO_GUIDED_MASTER;
                case 60:
                    return MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS;
                case 61:
                    return MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL;
                case 62:
                    return MAV_CMD.MAV_CMD_DO_LAST;
                case 63:
                    return MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION;
                case 64:
                    return MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
                case 65:
                    return MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN;
                case 66:
                    return MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE;
                case 67:
                    return MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
                case 68:
                    return MAV_CMD.MAV_CMD_OVERRIDE_GOTO;
                case 69:
                    return MAV_CMD.MAV_CMD_MISSION_START;
                case 70:
                    return MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM;
                case 71:
                    return MAV_CMD.MAV_CMD_GET_HOME_POSITION;
                case 72:
                    return MAV_CMD.MAV_CMD_START_RX_PAIR;
                case 73:
                    return MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL;
                case 74:
                    return MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL;
                case 75:
                    return MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION;
                case 76:
                    return MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
                case 77:
                    return MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION;
                case 78:
                    return MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS;
                case 79:
                    return MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION;
                case 80:
                    return MAV_CMD.MAV_CMD_STORAGE_FORMAT;
                case 81:
                    return MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
                case 82:
                    return MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION;
                case 83:
                    return MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS;
                case 84:
                    return MAV_CMD.MAV_CMD_SET_CAMERA_MODE;
                case 85:
                    return MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE;
                case 86:
                    return MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE;
                case 87:
                    return MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
                case 88:
                    return MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL;
                case 89:
                    return MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE;
                case 90:
                    return MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE;
                case 91:
                    return MAV_CMD.MAV_CMD_VIDEO_START_STREAMING;
                case 92:
                    return MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING;
                case 93:
                    return MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
                case 94:
                    return MAV_CMD.MAV_CMD_LOGGING_START;
                case 95:
                    return MAV_CMD.MAV_CMD_LOGGING_STOP;
                case 96:
                    return MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION;
                case 97:
                    return MAV_CMD.MAV_CMD_PANORAMA_CREATE;
                case 98:
                    return MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION;
                case 99:
                    return MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST;
                case 100:
                    return MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
                case 101:
                    return MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
                case 102:
                    return MAV_CMD.MAV_CMD_CONDITION_GATE;
                case 103:
                    return MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT;
                case 104:
                    return MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
                case 105:
                    return MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
                case 106:
                    return MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
                case 107:
                    return MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
                case 108:
                    return MAV_CMD.MAV_CMD_NAV_RALLY_POINT;
                case 109:
                    return MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO;
                case 110:
                    return MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
                case 111:
                    return MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
                case 112:
                    return MAV_CMD.MAV_CMD_WAYPOINT_USER_1;
                case 113:
                    return MAV_CMD.MAV_CMD_WAYPOINT_USER_2;
                case 114:
                    return MAV_CMD.MAV_CMD_WAYPOINT_USER_3;
                case 115:
                    return MAV_CMD.MAV_CMD_WAYPOINT_USER_4;
                case 116:
                    return MAV_CMD.MAV_CMD_WAYPOINT_USER_5;
                case 117:
                    return MAV_CMD.MAV_CMD_SPATIAL_USER_1;
                case 118:
                    return MAV_CMD.MAV_CMD_SPATIAL_USER_2;
                case 119:
                    return MAV_CMD.MAV_CMD_SPATIAL_USER_3;
                case 120:
                    return MAV_CMD.MAV_CMD_SPATIAL_USER_4;
                case 121:
                    return MAV_CMD.MAV_CMD_SPATIAL_USER_5;
                case 122:
                    return MAV_CMD.MAV_CMD_USER_1;
                case 123:
                    return MAV_CMD.MAV_CMD_USER_2;
                case 124:
                    return MAV_CMD.MAV_CMD_USER_3;
                case 125:
                    return MAV_CMD.MAV_CMD_USER_4;
                case 126:
                    return MAV_CMD.MAV_CMD_USER_5;
                case 127:
                    return MAV_CMD.MAV_CMD_RESET_MPPT;
                case 128:
                    return MAV_CMD.MAV_CMD_PAYLOAD_CONTROL;
            }
            throw  new ArgumentException("Unknown enum ID ");
        }
        /**
        *result in a mavlink mission ack*/

        public enum MAV_MISSION_RESULT
        {
            MAV_MISSION_ACCEPTED = 0, //mission accepted OK
            MAV_MISSION_ERROR = 1, //generic error / not accepting mission commands at all right now
            MAV_MISSION_UNSUPPORTED_FRAME = 2, //coordinate frame is not supported
            MAV_MISSION_UNSUPPORTED = 3, //command is not supported
            MAV_MISSION_NO_SPACE = 4, //mission item exceeds storage space
            MAV_MISSION_INVALID = 5, //one of the parameters has an invalid value
            MAV_MISSION_INVALID_PARAM1 = 6, //param1 has an invalid value
            MAV_MISSION_INVALID_PARAM2 = 7, //param2 has an invalid value
            MAV_MISSION_INVALID_PARAM3 = 8, //param3 has an invalid value
            MAV_MISSION_INVALID_PARAM4 = 9, //param4 has an invalid value
            MAV_MISSION_INVALID_PARAM5_X = 10, //x/param5 has an invalid value
            MAV_MISSION_INVALID_PARAM6_Y = 11, //y/param6 has an invalid value
            MAV_MISSION_INVALID_PARAM7 = 12, //param7 has an invalid value
            MAV_MISSION_INVALID_SEQUENCE = 13, //received waypoint out of sequence
            MAV_MISSION_DENIED = 14 //not accepting any mission commands from this communication partner
        }

        /**
        *Enumeration of estimator types*/

        public enum MAV_ESTIMATOR_TYPE
        {
            MAV_ESTIMATOR_TYPE_NAIVE = 1, //This is a naive estimator without any real covariance feedback.
            MAV_ESTIMATOR_TYPE_VISION = 2, //Computer vision based estimate. Might be up to scale.
            MAV_ESTIMATOR_TYPE_VIO = 3, //Visual-inertial estimate.
            MAV_ESTIMATOR_TYPE_GPS = 4, //Plain GPS estimate.
            MAV_ESTIMATOR_TYPE_GPS_INS = 5 //Estimator integrating GPS and inertial sensing.
        }

        /**
        *result from a mavlink command*/

        public enum MAV_RESULT
        {
            MAV_RESULT_ACCEPTED = 0, //Command ACCEPTED and EXECUTED
            MAV_RESULT_TEMPORARILY_REJECTED = 1, //Command TEMPORARY REJECTED/DENIED
            MAV_RESULT_DENIED = 2, //Command PERMANENTLY DENIED
            MAV_RESULT_UNSUPPORTED = 3, //Command UNKNOWN/UNSUPPORTED
            MAV_RESULT_FAILED = 4, //Command executed, but failed
            MAV_RESULT_IN_PROGRESS = 5 //WIP: Command being executed
        }

        /**
        *Power supply status flags (bitmask)*/
        [FlagsAttribute]
        public enum MAV_POWER_STATUS
        {
            MAV_POWER_STATUS_BRICK_VALID = 1, //main brick power supply valid
            MAV_POWER_STATUS_SERVO_VALID = 2, //main servo power supply valid for FMU
            MAV_POWER_STATUS_USB_CONNECTED = 4, //USB power is connected
            MAV_POWER_STATUS_PERIPH_OVERCURRENT = 8, //peripheral supply is in over-current state
            MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16, //hi-power peripheral supply is in over-current state
            MAV_POWER_STATUS_CHANGED = 32 //Power status has changed since boot
        }

        /**
        *SERIAL_CONTROL device types*/

        public enum SERIAL_CONTROL_DEV
        {
            SERIAL_CONTROL_DEV_TELEM1 = 0, //First telemetry port
            SERIAL_CONTROL_DEV_TELEM2 = 1, //Second telemetry port
            SERIAL_CONTROL_DEV_GPS1 = 2, //First GPS port
            SERIAL_CONTROL_DEV_GPS2 = 3, //Second GPS port
            SERIAL_CONTROL_DEV_SHELL = 10 //system shell
        }

        /**
        *SERIAL_CONTROL flags (bitmask)*/
        [FlagsAttribute]
        public enum SERIAL_CONTROL_FLAG
        {
            SERIAL_CONTROL_FLAG_REPLY = 1, //Set if this is a reply
            SERIAL_CONTROL_FLAG_RESPOND = 2, //Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
            /**
            ** Set if access to the serial port should be removed from whatever driver is currently using it, giving
            *		 * exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without
            *		 * this flag se*/
            SERIAL_CONTROL_FLAG_EXCLUSIVE = 4,
            SERIAL_CONTROL_FLAG_BLOCKING = 8, //Block on writes to the serial port
            SERIAL_CONTROL_FLAG_MULTI = 16 //Send multiple replies until port is drained
        }

        /**
        *Enumeration of distance sensor types*/

        public enum MAV_DISTANCE_SENSOR
        {
            MAV_DISTANCE_SENSOR_LASER = 0, //Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
            MAV_DISTANCE_SENSOR_ULTRASOUND = 1, //Ultrasound rangefinder, e.g. MaxBotix units
            MAV_DISTANCE_SENSOR_INFRARED = 2, //Infrared rangefinder, e.g. Sharp units
            MAV_DISTANCE_SENSOR_RADAR = 3, //Radar type, e.g. uLanding units
            MAV_DISTANCE_SENSOR_UNKNOWN = 4 //Broken or unknown type, e.g. analog units
        }

        /**
        *Enumeration of sensor orientation, according to its rotations*/

        public enum MAV_SENSOR_ORIENTATION
        {
            MAV_SENSOR_ROTATION_NONE = 0, //Roll: 0, Pitch: 0, Yaw: 0
            MAV_SENSOR_ROTATION_YAW_45 = 1, //Roll: 0, Pitch: 0, Yaw: 45
            MAV_SENSOR_ROTATION_YAW_90 = 2, //Roll: 0, Pitch: 0, Yaw: 90
            MAV_SENSOR_ROTATION_YAW_135 = 3, //Roll: 0, Pitch: 0, Yaw: 135
            MAV_SENSOR_ROTATION_YAW_180 = 4, //Roll: 0, Pitch: 0, Yaw: 180
            MAV_SENSOR_ROTATION_YAW_225 = 5, //Roll: 0, Pitch: 0, Yaw: 225
            MAV_SENSOR_ROTATION_YAW_270 = 6, //Roll: 0, Pitch: 0, Yaw: 270
            MAV_SENSOR_ROTATION_YAW_315 = 7, //Roll: 0, Pitch: 0, Yaw: 315
            MAV_SENSOR_ROTATION_ROLL_180 = 8, //Roll: 180, Pitch: 0, Yaw: 0
            MAV_SENSOR_ROTATION_ROLL_180_YAW_45 = 9, //Roll: 180, Pitch: 0, Yaw: 45
            MAV_SENSOR_ROTATION_ROLL_180_YAW_90 = 10, //Roll: 180, Pitch: 0, Yaw: 90
            MAV_SENSOR_ROTATION_ROLL_180_YAW_135 = 11, //Roll: 180, Pitch: 0, Yaw: 135
            MAV_SENSOR_ROTATION_PITCH_180 = 12, //Roll: 0, Pitch: 180, Yaw: 0
            MAV_SENSOR_ROTATION_ROLL_180_YAW_225 = 13, //Roll: 180, Pitch: 0, Yaw: 225
            MAV_SENSOR_ROTATION_ROLL_180_YAW_270 = 14, //Roll: 180, Pitch: 0, Yaw: 270
            MAV_SENSOR_ROTATION_ROLL_180_YAW_315 = 15, //Roll: 180, Pitch: 0, Yaw: 315
            MAV_SENSOR_ROTATION_ROLL_90 = 16, //Roll: 90, Pitch: 0, Yaw: 0
            MAV_SENSOR_ROTATION_ROLL_90_YAW_45 = 17, //Roll: 90, Pitch: 0, Yaw: 45
            MAV_SENSOR_ROTATION_ROLL_90_YAW_90 = 18, //Roll: 90, Pitch: 0, Yaw: 90
            MAV_SENSOR_ROTATION_ROLL_90_YAW_135 = 19, //Roll: 90, Pitch: 0, Yaw: 135
            MAV_SENSOR_ROTATION_ROLL_270 = 20, //Roll: 270, Pitch: 0, Yaw: 0
            MAV_SENSOR_ROTATION_ROLL_270_YAW_45 = 21, //Roll: 270, Pitch: 0, Yaw: 45
            MAV_SENSOR_ROTATION_ROLL_270_YAW_90 = 22, //Roll: 270, Pitch: 0, Yaw: 90
            MAV_SENSOR_ROTATION_ROLL_270_YAW_135 = 23, //Roll: 270, Pitch: 0, Yaw: 135
            MAV_SENSOR_ROTATION_PITCH_90 = 24, //Roll: 0, Pitch: 90, Yaw: 0
            MAV_SENSOR_ROTATION_PITCH_270 = 25, //Roll: 0, Pitch: 270, Yaw: 0
            MAV_SENSOR_ROTATION_PITCH_180_YAW_90 = 26, //Roll: 0, Pitch: 180, Yaw: 90
            MAV_SENSOR_ROTATION_PITCH_180_YAW_270 = 27, //Roll: 0, Pitch: 180, Yaw: 270
            MAV_SENSOR_ROTATION_ROLL_90_PITCH_90 = 28, //Roll: 90, Pitch: 90, Yaw: 0
            MAV_SENSOR_ROTATION_ROLL_180_PITCH_90 = 29, //Roll: 180, Pitch: 90, Yaw: 0
            MAV_SENSOR_ROTATION_ROLL_270_PITCH_90 = 30, //Roll: 270, Pitch: 90, Yaw: 0
            MAV_SENSOR_ROTATION_ROLL_90_PITCH_180 = 31, //Roll: 90, Pitch: 180, Yaw: 0
            MAV_SENSOR_ROTATION_ROLL_270_PITCH_180 = 32, //Roll: 270, Pitch: 180, Yaw: 0
            MAV_SENSOR_ROTATION_ROLL_90_PITCH_270 = 33, //Roll: 90, Pitch: 270, Yaw: 0
            MAV_SENSOR_ROTATION_ROLL_180_PITCH_270 = 34, //Roll: 180, Pitch: 270, Yaw: 0
            MAV_SENSOR_ROTATION_ROLL_270_PITCH_270 = 35, //Roll: 270, Pitch: 270, Yaw: 0
            MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 = 36, //Roll: 90, Pitch: 180, Yaw: 90
            MAV_SENSOR_ROTATION_ROLL_90_YAW_270 = 37, //Roll: 90, Pitch: 0, Yaw: 270
            MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315 = 38 //Roll: 315, Pitch: 315, Yaw: 315
        }

        /**
        *Enumeration of battery functions*/

        public enum MAV_BATTERY_FUNCTION
        {
            MAV_BATTERY_FUNCTION_UNKNOWN = 0, //Battery function is unknown
            MAV_BATTERY_FUNCTION_ALL = 1, //Battery supports all flight systems
            MAV_BATTERY_FUNCTION_PROPULSION = 2, //Battery for the propulsion system
            MAV_BATTERY_FUNCTION_AVIONICS = 3, //Avionics battery
            MAV_BATTERY_TYPE_PAYLOAD = 4 //Payload battery
        }

        /**
        *Enumeration of battery types*/

        public enum MAV_BATTERY_TYPE
        {
            MAV_BATTERY_TYPE_UNKNOWN = 0, //Not specified.
            MAV_BATTERY_TYPE_LIPO = 1, //Lithium polymer battery
            MAV_BATTERY_TYPE_LIFE = 2, //Lithium-iron-phosphate battery
            MAV_BATTERY_TYPE_LION = 3, //Lithium-ION battery
            MAV_BATTERY_TYPE_NIMH = 4 //Nickel metal hydride battery
        }

        /**
        *Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability*/
        [FlagsAttribute]
        public enum MAV_PROTOCOL_CAPABILITY
        {
            MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT = 1, //Autopilot supports MISSION float message type.
            MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT = 2, //Autopilot supports the new param float message type.
            MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4, //Autopilot supports MISSION_INT scaled integer message type.
            MAV_PROTOCOL_CAPABILITY_COMMAND_INT = 8, //Autopilot supports COMMAND_INT scaled integer message type.
            MAV_PROTOCOL_CAPABILITY_PARAM_UNION = 16, //Autopilot supports the new param union message type.
            MAV_PROTOCOL_CAPABILITY_FTP = 32, //Autopilot supports the new FILE_TRANSFER_PROTOCOL message type.
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET = 64, //Autopilot supports commanding attitude offboard.
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED = 128, //Autopilot supports commanding position and velocity targets in local NED frame.
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 256, //Autopilot supports commanding position and velocity targets in global scaled integers.
            MAV_PROTOCOL_CAPABILITY_TERRAIN = 512, //Autopilot supports terrain protocol / data handling.
            MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET = 1024, //Autopilot supports direct actuator control.
            MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION = 2048, //Autopilot supports the flight termination command.
            MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION = 4096, //Autopilot supports onboard compass calibration.
            MAV_PROTOCOL_CAPABILITY_MAVLINK2 = 8192, //Autopilot supports mavlink version 2.
            MAV_PROTOCOL_CAPABILITY_MISSION_FENCE = 16384, //Autopilot supports mission fence protocol.
            MAV_PROTOCOL_CAPABILITY_MISSION_RALLY = 32768, //Autopilot supports mission rally point protocol.
            MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION = 65536 //Autopilot supports the flight information protocol.
        }

        /**
        *Type of landing target*/

        public enum LANDING_TARGET_TYPE
        {
            LANDING_TARGET_TYPE_LIGHT_BEACON = 0, //Landing target signaled by light beacon (ex: IR-LOCK)
            LANDING_TARGET_TYPE_RADIO_BEACON = 1, //Landing target signaled by radio beacon (ex: ILS, NDB)
            LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2, //Landing target represented by a fiducial marker (ex: ARTag)
            LANDING_TARGET_TYPE_VISION_OTHER = 3 //Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
        }

        /**
        *Flags in EKF_STATUS message*/
        [FlagsAttribute]
        public enum ESTIMATOR_STATUS_FLAGS
        {
            ESTIMATOR_ATTITUDE = 1, //True if the attitude estimate is good
            ESTIMATOR_VELOCITY_HORIZ = 2, //True if the horizontal velocity estimate is good
            ESTIMATOR_VELOCITY_VERT = 4, //True if the  vertical velocity estimate is good
            ESTIMATOR_POS_HORIZ_REL = 8, //True if the horizontal position (relative) estimate is good
            ESTIMATOR_POS_HORIZ_ABS = 16, //True if the horizontal position (absolute) estimate is good
            ESTIMATOR_POS_VERT_ABS = 32, //True if the vertical position (absolute) estimate is good
            ESTIMATOR_POS_VERT_AGL = 64, //True if the vertical position (above ground) estimate is good
            /**
            ** True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical
            *		 * flow*/
            ESTIMATOR_CONST_POS_MODE = 128,
            ESTIMATOR_PRED_POS_HORIZ_REL = 256, //True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimat
            ESTIMATOR_PRED_POS_HORIZ_ABS = 512, //True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimat
            ESTIMATOR_GPS_GLITCH = 1024 //True if the EKF has detected a GPS glitch
        }

        [FlagsAttribute]
        public enum GPS_INPUT_IGNORE_FLAGS
        {
            GPS_INPUT_IGNORE_FLAG_ALT = 1, //ignore altitude field
            GPS_INPUT_IGNORE_FLAG_HDOP = 2, //ignore hdop field
            GPS_INPUT_IGNORE_FLAG_VDOP = 4, //ignore vdop field
            GPS_INPUT_IGNORE_FLAG_VEL_HORIZ = 8, //ignore horizontal velocity field (vn and ve)
            GPS_INPUT_IGNORE_FLAG_VEL_VERT = 16, //ignore vertical velocity field (vd)
            GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY = 32, //ignore speed accuracy field
            GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY = 64, //ignore horizontal accuracy field
            GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY = 128 //ignore vertical accuracy field
        }

        /**
        *Enumeration of landed detector states*/

        public enum MAV_LANDED_STATE
        {
            MAV_LANDED_STATE_UNDEFINED = 0, //MAV landed state is unknown
            MAV_LANDED_STATE_ON_GROUND = 1, //MAV is landed (on ground)
            MAV_LANDED_STATE_IN_AIR = 2, //MAV is in air
            MAV_LANDED_STATE_TAKEOFF = 3, //MAV currently taking off
            MAV_LANDED_STATE_LANDING = 4 //MAV currently landing
        }

        /**
        *Enumeration of VTOL states*/

        public enum MAV_VTOL_STATE
        {
            MAV_VTOL_STATE_UNDEFINED = 0, //MAV is not configured as VTOL
            MAV_VTOL_STATE_TRANSITION_TO_FW = 1, //VTOL is in transition from multicopter to fixed-wing
            MAV_VTOL_STATE_TRANSITION_TO_MC = 2, //VTOL is in transition from fixed-wing to multicopter
            MAV_VTOL_STATE_MC = 3, //VTOL is in multicopter state
            MAV_VTOL_STATE_FW = 4 //VTOL is in fixed-wing state
        }

        /**
        *Enumeration of the ADSB altimeter types*/

        public enum ADSB_ALTITUDE_TYPE
        {
            ADSB_ALTITUDE_TYPE_PRESSURE_QNH = 0, //Altitude reported from a Baro source using QNH reference
            ADSB_ALTITUDE_TYPE_GEOMETRIC = 1 //Altitude reported from a GNSS source
        }

        /**
        *ADSB classification for the type of vehicle emitting the transponder signal*/

        public enum ADSB_EMITTER_TYPE
        {
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
            ADSB_EMITTER_TYPE_POINT_OBSTACLE = 19
        }

        /**
        *These flags indicate status such as data validity of each data source. Set = data valid*/
        [FlagsAttribute]
        public enum ADSB_FLAGS
        {
            ADSB_FLAGS_VALID_COORDS = 1,
            ADSB_FLAGS_VALID_ALTITUDE = 2,
            ADSB_FLAGS_VALID_HEADING = 4,
            ADSB_FLAGS_VALID_VELOCITY = 8,
            ADSB_FLAGS_VALID_CALLSIGN = 16,
            ADSB_FLAGS_VALID_SQUAWK = 32,
            ADSB_FLAGS_SIMULATED = 64
        }

        /**
        *Source of information about this collision.*/

        public enum MAV_COLLISION_SRC
        {
            MAV_COLLISION_SRC_ADSB = 0, //ID field references ADSB_VEHICLE packets
            MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT = 1 //ID field references MAVLink SRC ID
        }

        /**
        *Possible actions an aircraft can take to avoid a collision.*/

        public enum MAV_COLLISION_ACTION
        {
            MAV_COLLISION_ACTION_NONE = 0, //Ignore any potential collisions
            MAV_COLLISION_ACTION_REPORT = 1, //Report potential collision
            MAV_COLLISION_ACTION_ASCEND_OR_DESCEND = 2, //Ascend or Descend to avoid threat
            MAV_COLLISION_ACTION_MOVE_HORIZONTALLY = 3, //Move horizontally to avoid threat
            MAV_COLLISION_ACTION_MOVE_PERPENDICULAR = 4, //Aircraft to move perpendicular to the collision's velocity vector
            MAV_COLLISION_ACTION_RTL = 5, //Aircraft to fly directly back to its launch point
            MAV_COLLISION_ACTION_HOVER = 6 //Aircraft to stop in place
        }

        /**
        *Aircraft-rated danger from this threat.*/
        [FlagsAttribute]
        public enum MAV_COLLISION_THREAT_LEVEL
        {
            MAV_COLLISION_THREAT_LEVEL_NONE = 0, //Not a threat
            MAV_COLLISION_THREAT_LEVEL_LOW = 1, //Craft is mildly concerned about this threat
            MAV_COLLISION_THREAT_LEVEL_HIGH = 2 //Craft is panicing, and may take actions to avoid threat
        }

        /**
        *Indicates the severity level, generally used for status messages to indicate their relative urgency. Based
        *	 on RFC-5424 using expanded definitions at: http:www.kiwisyslog.com/kb/info:-syslog-message-levels/*/

        public enum MAV_SEVERITY
        {
            MAV_SEVERITY_EMERGENCY = 0, //System is unusable. This is a "panic" condition.
            MAV_SEVERITY_ALERT = 1, //Action should be taken immediately. Indicates error in non-critical systems.
            MAV_SEVERITY_CRITICAL = 2, //Action must be taken immediately. Indicates failure in a primary system.
            MAV_SEVERITY_ERROR = 3, //Indicates an error in secondary/redundant systems.
            /**
            ** Indicates about a possible future error if this is not resolved within a given timeframe. Example would
            *		 * be a low battery warning*/
            MAV_SEVERITY_WARNING = 4,
            /**
            ** An unusual event has occured, though not an error condition. This should be investigated for the root
            *		 * cause*/
            MAV_SEVERITY_NOTICE = 5,
            MAV_SEVERITY_INFO = 6, //Normal operational messages. Useful for logging. No action is required for these messages.
            MAV_SEVERITY_DEBUG = 7 //Useful non-operational messages that can assist in debugging. These should not occur during normal operation
        }

        /**
        *Camera capability flags (Bitmap).*/
        [FlagsAttribute]
        public enum CAMERA_CAP_FLAGS
        {
            CAMERA_CAP_FLAGS_CAPTURE_VIDEO = 1, //Camera is able to record video.
            CAMERA_CAP_FLAGS_CAPTURE_IMAGE = 2, //Camera is able to capture images.
            CAMERA_CAP_FLAGS_HAS_MODES = 4, //Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)
            CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = 8, //Camera can capture images while in video mode
            CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = 16, //Camera can capture videos while in Photo/Image mode
            CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE = 32 //Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)
        }

        /**
        *Camera Modes.*/
        [FlagsAttribute]
        public enum CAMERA_MODE
        {
            CAMERA_MODE_IMAGE = 0, //Camera is in image/photo capture mode.
            CAMERA_MODE_VIDEO = 1, //Camera is in video capture mode.
            CAMERA_MODE_IMAGE_SURVEY = 2 //Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys
        }

        /**
        *Generalized UAVCAN node health*/

        public enum UAVCAN_NODE_HEALTH
        {
            UAVCAN_NODE_HEALTH_OK = 0, //The node is functioning properly.
            UAVCAN_NODE_HEALTH_WARNING = 1, //A critical parameter went out of range or the node has encountered a minor failure.
            UAVCAN_NODE_HEALTH_ERROR = 2, //The node has encountered a major failure.
            UAVCAN_NODE_HEALTH_CRITICAL = 3 //The node has suffered a fatal malfunction.
        }

        /**
        *Generalized UAVCAN node mode*/

        public enum UAVCAN_NODE_MODE
        {
            UAVCAN_NODE_MODE_OPERATIONAL = 0, //The node is performing its primary functions.
            UAVCAN_NODE_MODE_INITIALIZATION = 1, //The node is initializing; this mode is entered immediately after startup.
            UAVCAN_NODE_MODE_MAINTENANCE = 2, //The node is under maintenance.
            UAVCAN_NODE_MODE_SOFTWARE_UPDATE = 3, //The node is in the process of updating its software.
            UAVCAN_NODE_MODE_OFFLINE = 7 //The node is no longer available online.
        }

        /**
        *Specifies the datatype of a MAVLink extended parameter.*/

        public enum MAV_PARAM_EXT_TYPE
        {
            MAV_PARAM_EXT_TYPE_UINT8 = 1, //8-bit unsigned integer
            MAV_PARAM_EXT_TYPE_INT8 = 2, //8-bit signed integer
            MAV_PARAM_EXT_TYPE_UINT16 = 3, //16-bit unsigned integer
            MAV_PARAM_EXT_TYPE_INT16 = 4, //16-bit signed integer
            MAV_PARAM_EXT_TYPE_UINT32 = 5, //32-bit unsigned integer
            MAV_PARAM_EXT_TYPE_INT32 = 6, //32-bit signed integer
            MAV_PARAM_EXT_TYPE_UINT64 = 7, //64-bit unsigned integer
            MAV_PARAM_EXT_TYPE_INT64 = 8, //64-bit signed integer
            MAV_PARAM_EXT_TYPE_REAL32 = 9, //32-bit floating-point
            MAV_PARAM_EXT_TYPE_REAL64 = 10, //64-bit floating-point
            MAV_PARAM_EXT_TYPE_CUSTOM = 11 //Custom Type
        }

        /**
        *Result from a PARAM_EXT_SET message.*/

        public enum PARAM_ACK
        {
            PARAM_ACK_ACCEPTED = 0, //Parameter value ACCEPTED and SET
            PARAM_ACK_VALUE_UNSUPPORTED = 1, //Parameter value UNKNOWN/UNSUPPORTED
            PARAM_ACK_FAILED = 2, //Parameter failed to set
            /**
            ** Parameter value received but not yet validated or set. A subsequent PARAM_EXT_ACK will follow once operation
            *		 * is completed with the actual result. These are for parameters that may take longer to set. Instead of
            *		 * waiting for an ACK and potentially timing out, you will immediately receive this response to let you
            *		 * know it was received*/
            PARAM_ACK_IN_PROGRESS = 3
        }

        static readonly Field _Z = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _X = new Field(5, true, -6, 2, 1, 0, 0, 0, 1);
        static readonly Field _P = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _q = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _U = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _fs = new Field(0, false, 1, 4, 1, 0, 0, 0);
        static readonly Field _ts = new Field(0, true, 1, 4, 1, 0, 0, 0);
        static readonly Field _Os = new Field(0, true, 1, 4, 1, 0, 0, 0);
        static readonly Field _Is = new Field(0, true, 1, 4, 1, 0, 0, 0);
        static readonly Field _us = new Field(0, true, 1, 4, 1, 0, 0, 0);
        static readonly Field _VK = new Field(0, true, 1, 2, 1, 0, 0, 0);
        static readonly Field _fK = new Field(0, true, 1, 2, 1, 0, 0, 0);
        static readonly Field _tK = new Field(0, true, 1, 2, 1, 0, 0, 0);
        static readonly Field _OK = new Field(0, true, 1, 2, 1, 0, 0, 0);
        static readonly Field _IK = new Field(0, true, 1, 2, 1, 0, 0, 0);
        static readonly Field _uK = new Field(0, true, 1, 2, 1, 0, 0, 0);
        static readonly Field _TK = new Field(0, true, 1, 2, 1, 0, 0, 0);
        static readonly Field _aK = new Field(0, true, 1, 2, 1, 0, 0, 0);
        static readonly Field _gJ = new Field(0, true, 1, 8, 1, 0, 0, 0);
        static readonly Field _xJ = new Field(0, true, 1, 8, 1, 0, 0, 0);
        static readonly Field _yJ = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _Rj = new Field(0, false, 1, 1, 1, 0, 0, 0);
        static readonly Field _yj = new Field(0, false, 1, 4, 1, 0, 0, 0);
        static readonly Field _bj = new Field(0, false, 1, 1, 1, 0, 0, 0);
        static readonly Field _Ej = new Field(0, false, 1, 1, 1, 0, 0, 0);
        static readonly Field _uf = new Field(0, false, 1, 4, 1, 0, 0, 0);
        static readonly Field _Tf = new Field(0, false, 1, 4, 1, 0, 0, 0);
        static readonly Field _eg = new Field(0, false, 18, 1, 1, 0, 0, 0);
        static readonly Field _ug = new Field(0, false, 1, 4, 1, 0, 0, 0);
        static readonly Field _Tg = new Field(0, false, 1, 4, 1, 0, 0, 0);
        static readonly Field _ag = new Field(0, false, 1, 4, 1, 0, 0, 0);
        static readonly Field _gg = new Field(0, false, 4, 4, 1, 0, 0, 0);
        static readonly Field _Bg = new Field(0, false, 1, 1, 1, 0, 0, 0);
        static readonly Field _Uc = new Field(0, true, 1, 8, 1, 0, 0, 0);
        static readonly Field _VB = new Field(0, true, 1, 8, 1, 0, 0, 0);
        static readonly Field _hB = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
        static readonly Field _NB = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
        static readonly Field _UB = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
        static readonly Field _mL = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
        static readonly Field _KL = new Field(5, true, -6, 2, 1, 0, 0, 0, 1);
        static readonly Field _aL = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _iL = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
        static readonly Field _ex = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
        static readonly Field _Ax = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
        static readonly Field _ox = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
        static readonly Field _lx = new Field(5, true, -6, 2, 1, 0, 0, 0, 1);
        static readonly Field _Cx = new Field(5, true, -7, 2, 1, 0, 0, 0, 1);
        static readonly Field _Vh = new Field(5, true, -7, 2, 1, 0, 0, 0, 1);
        static readonly Field _ch = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _hh = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _Rh = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
        static readonly Field _ih = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _Dh = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
        static readonly Field _Xh = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _Yh = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);

    }
}
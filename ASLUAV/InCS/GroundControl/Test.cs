
using System;
using System.Diagnostics;
using System.Linq;
using org.unirail.BlackBox;
using Inside = org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds.Inside;
namespace org.noname
{
    public class Test : GroundControl
    {


        public class HEARTBEAT : GroundControl.HEARTBEAT
        {
            public uint custom_mode //A bitfield for use for autopilot-specific flags.
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte mavlink_version //MAVLink version, not writable by user, gets added by protocol because of magic data type: byte_mavlink_versio
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public MAV_TYPE type //Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 5, data, 40);}
            }

            public MAV_AUTOPILOT autopilot //Autopilot type / class. defined in MAV_AUTOPILOT ENUM
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 5, data, 45);}
            }

            public MAV_MODE_FLAG base_mode //System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
            {
                set
                {
                    ulong id = 0;
                    switch(value)
                    {
                        case MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
                            id = 0;
                            break;
                        case MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED:
                            id = 1;
                            break;
                        case MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED:
                            id = 2;
                            break;
                        case MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED:
                            id = 3;
                            break;
                        case MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED:
                            id = 4;
                            break;
                        case MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED:
                            id = 5;
                            break;
                        case MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:
                            id = 6;
                            break;
                        case MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED:
                            id = 7;
                            break;
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 4, data, 50);
                }
            }

            public MAV_STATE system_status //System status flag, see MAV_STATE ENUM
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 54);}
            }
        }
        public class SYS_STATUS : GroundControl.SYS_STATUS
        {
            public ushort load //Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort voltage_battery //Battery voltage, in millivolts (1 = 1 millivolt)
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            /**
            *Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links
            *	(packets that were corrupted on reception on the MAV*/
            public ushort drop_rate_comm
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            /**
            *Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
            *	on reception on the MAV*/
            public ushort errors_comm
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort errors_count1 //Autopilot-specific errors
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort errors_count2 //Autopilot-specific errors
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort errors_count3 //Autopilot-specific errors
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort errors_count4 //Autopilot-specific errors
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public short current_battery //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  16);}
            }

            public sbyte battery_remaining //Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
            {
                set {  BitUtils.set_bytes((byte)(value), 1, data,  18);}
            }

            /**
            *Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1:
            *	present. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
            public MAV_SYS_STATUS_SENSOR onboard_control_sensors_present
            {
                set
                {
                    ulong id = 0;
                    switch(value)
                    {
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO:
                            id = 0;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL:
                            id = 1;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG:
                            id = 2;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:
                            id = 3;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE:
                            id = 4;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS:
                            id = 5;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW:
                            id = 6;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION:
                            id = 7;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION:
                            id = 8;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH:
                            id = 9;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL:
                            id = 10;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION:
                            id = 11;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION:
                            id = 12;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL:
                            id = 13;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL:
                            id = 14;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS:
                            id = 15;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER:
                            id = 16;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2:
                            id = 17;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2:
                            id = 18;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2:
                            id = 19;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE:
                            id = 20;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS:
                            id = 21;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN:
                            id = 22;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR:
                            id = 23;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING:
                            id = 24;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY:
                            id = 25;
                            break;
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 5, data, 152);
                }
            }

            /**
            *Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of
            *	1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
            public MAV_SYS_STATUS_SENSOR onboard_control_sensors_enabled
            {
                set
                {
                    ulong id = 0;
                    switch(value)
                    {
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO:
                            id = 0;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL:
                            id = 1;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG:
                            id = 2;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:
                            id = 3;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE:
                            id = 4;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS:
                            id = 5;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW:
                            id = 6;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION:
                            id = 7;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION:
                            id = 8;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH:
                            id = 9;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL:
                            id = 10;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION:
                            id = 11;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION:
                            id = 12;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL:
                            id = 13;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL:
                            id = 14;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS:
                            id = 15;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER:
                            id = 16;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2:
                            id = 17;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2:
                            id = 18;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2:
                            id = 19;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE:
                            id = 20;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS:
                            id = 21;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN:
                            id = 22;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR:
                            id = 23;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING:
                            id = 24;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY:
                            id = 25;
                            break;
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 5, data, 157);
                }
            }

            /**
            *Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not
            *	enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
            public MAV_SYS_STATUS_SENSOR onboard_control_sensors_health
            {
                set
                {
                    ulong id = 0;
                    switch(value)
                    {
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO:
                            id = 0;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL:
                            id = 1;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG:
                            id = 2;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:
                            id = 3;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE:
                            id = 4;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS:
                            id = 5;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW:
                            id = 6;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION:
                            id = 7;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION:
                            id = 8;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH:
                            id = 9;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL:
                            id = 10;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION:
                            id = 11;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION:
                            id = 12;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL:
                            id = 13;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL:
                            id = 14;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS:
                            id = 15;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER:
                            id = 16;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2:
                            id = 17;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2:
                            id = 18;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2:
                            id = 19;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE:
                            id = 20;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS:
                            id = 21;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN:
                            id = 22;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR:
                            id = 23;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING:
                            id = 24;
                            break;
                        case MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY:
                            id = 25;
                            break;
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 5, data, 162);
                }
            }
        }
        public class SYSTEM_TIME : GroundControl.SYSTEM_TIME
        {
            public uint time_boot_ms //Timestamp of the component clock since boot time in milliseconds.
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public ulong time_unix_usec //Timestamp of the master clock in microseconds since UNIX epoch.
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  4);}
            }
        }
        public class PING : GroundControl.PING
        {
            public uint seq //PING sequence
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public ulong time_usec //Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  4);}
            }

            /**
            *0: request ping from all receiving systems, if greater than 0: message is a ping response and number is
            *	the system id of the requesting syste*/
            public byte target_system
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  12);}
            }

            /**
            *0: request ping from all receiving components, if greater than 0: message is a ping response and number
            *	is the system id of the requesting syste*/
            public byte target_component
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  13);}
            }
        }
        public class CHANGE_OPERATOR_CONTROL : GroundControl.CHANGE_OPERATOR_CONTROL
        {
            public byte target_system //System the GCS requests control for
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte control_request //0: request control of this MAV, 1: Release control of this MAV
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            /**
            *0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use
            *	the safest mode possible initially and then gradually move down the encryption level if it gets a NACK
            *	message indicating an encryption mismatch*/
            public byte version
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }
            /**
            *Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
            *	characters may involve A-Z, a-z, 0-9, and "!?,.-*/
            public void passkey_SET(string src, Inside ph)
            {passkey_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
*	characters may involve A-Z, a-z, 0-9, and "!?,.-*/
            public void passkey_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 24 && insert_field(ph, 24, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            }
        }
        public class CHANGE_OPERATOR_CONTROL_ACK : GroundControl.CHANGE_OPERATOR_CONTROL_ACK
        {
            public byte gcs_system_id //ID of the GCS this message
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte control_request //0: request control of this MAV, 1: Release control of this MAV
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            /**
            *0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under
            *	contro*/
            public byte ack
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }
        }
        public class AUTH_KEY : GroundControl.AUTH_KEY
        {
            public void key_SET(string src, Inside ph)//key
            {key_SET(src.ToCharArray(), 0, src.Length, ph);} public void key_SET(char[] src, int pos, int items, Inside ph) //key
            {
                if(ph.field_bit != 0 && insert_field(ph, 0, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            }
        }
        public class SET_MODE : GroundControl.SET_MODE
        {
            public uint custom_mode //The new autopilot-specific mode. This field can be ignored by an autopilot.
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte target_system //The system setting the mode
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public MAV_MODE base_mode //The new base mode
            {
                set
                {
                    ulong id = 0;
                    switch(value)
                    {
                        case MAV_MODE.MAV_MODE_PREFLIGHT:
                            id = 0;
                            break;
                        case MAV_MODE.MAV_MODE_MANUAL_DISARMED:
                            id = 1;
                            break;
                        case MAV_MODE.MAV_MODE_TEST_DISARMED:
                            id = 2;
                            break;
                        case MAV_MODE.MAV_MODE_STABILIZE_DISARMED:
                            id = 3;
                            break;
                        case MAV_MODE.MAV_MODE_GUIDED_DISARMED:
                            id = 4;
                            break;
                        case MAV_MODE.MAV_MODE_AUTO_DISARMED:
                            id = 5;
                            break;
                        case MAV_MODE.MAV_MODE_MANUAL_ARMED:
                            id = 6;
                            break;
                        case MAV_MODE.MAV_MODE_TEST_ARMED:
                            id = 7;
                            break;
                        case MAV_MODE.MAV_MODE_STABILIZE_ARMED:
                            id = 8;
                            break;
                        case MAV_MODE.MAV_MODE_GUIDED_ARMED:
                            id = 9;
                            break;
                        case MAV_MODE.MAV_MODE_AUTO_ARMED:
                            id = 10;
                            break;
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 4, data, 40);
                }
            }
        }
        public class PARAM_REQUEST_READ : GroundControl.PARAM_REQUEST_READ
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public short param_index //Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  2);}
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	storage if the ID is stored as strin*/
            public void param_id_SET(string src, Inside ph)
            {param_id_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
            public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 32 && insert_field(ph, 32, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            }
        }
        public class PARAM_REQUEST_LIST : GroundControl.PARAM_REQUEST_LIST
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }
        }
        public class PARAM_VALUE : GroundControl.PARAM_VALUE
        {
            public ushort param_count //Total number of onboard parameters
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort param_index //Index of this onboard parameter
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public float param_value //Onboard parameter value
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public MAV_PARAM_TYPE param_type //Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 4, data, 64);}
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	storage if the ID is stored as strin*/
            public void param_id_SET(string src, Inside ph)
            {param_id_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
            public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 68 && insert_field(ph, 68, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            }
        }
        public class PARAM_SET : GroundControl.PARAM_SET
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public float param_value //Onboard parameter value
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 2);}
            }

            public MAV_PARAM_TYPE param_type //Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 4, data, 48);}
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	storage if the ID is stored as strin*/
            public void param_id_SET(string src, Inside ph)
            {param_id_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
            public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 52 && insert_field(ph, 52, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            }
        }
        public class GPS_RAW_INT : GroundControl.GPS_RAW_INT
        {
            public ushort eph //GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort epv //GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort vel //GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            /**
            *Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
            *	unknown, set to: UINT16_MA*/
            public ushort cog
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public int lat //Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  16);}
            }

            public int lon //Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  20);}
            }

            /**
            *Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide
            *	the AMSL altitude in addition to the WGS84 altitude*/
            public int alt
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  24);}
            }

            public byte satellites_visible //Number of satellites visible. If unknown, set to 255
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  28);}
            }

            public GPS_FIX_TYPE fix_type //See the GPS_FIX_TYPE enum.
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 232);}
            }
            public void alt_ellipsoid_SET(int src, Inside ph)//Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up).
            {
                if(ph.field_bit != 236)insert_field(ph, 236, 0);
                BitUtils.set_bytes((uint)(src), 4, data,  ph.BYTE);
            } public void h_acc_SET(uint src, Inside ph) //Position uncertainty in meters * 1000 (positive for up).
            {
                if(ph.field_bit != 237)insert_field(ph, 237, 0);
                BitUtils.set_bytes((ulong)(src), 4, data,  ph.BYTE);
            } public void v_acc_SET(uint src, Inside ph) //Altitude uncertainty in meters * 1000 (positive for up).
            {
                if(ph.field_bit != 238)insert_field(ph, 238, 0);
                BitUtils.set_bytes((ulong)(src), 4, data,  ph.BYTE);
            } public void vel_acc_SET(uint src, Inside ph) //Speed uncertainty in meters * 1000 (positive for up).
            {
                if(ph.field_bit != 239)insert_field(ph, 239, 0);
                BitUtils.set_bytes((ulong)(src), 4, data,  ph.BYTE);
            } public void hdg_acc_SET(uint src, Inside ph) //Heading / track uncertainty in degrees * 1e5.
            {
                if(ph.field_bit != 240)insert_field(ph, 240, 0);
                BitUtils.set_bytes((ulong)(src), 4, data,  ph.BYTE);
            }
        }
        public class GPS_STATUS : GroundControl.GPS_STATUS
        {
            public byte satellites_visible //Number of satellites visible
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte[] satellite_prn //Global satellite ID
            {
                set {satellite_prn_SET(value, 0)  ;}
            }
            public void satellite_prn_SET(byte[] src, int pos)  //Global satellite ID
            {
                for(int BYTE =  1, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            public byte[] satellite_used //0: Satellite not used, 1: used for localization
            {
                set {satellite_used_SET(value, 0)  ;}
            }
            public void satellite_used_SET(byte[] src, int pos)  //0: Satellite not used, 1: used for localization
            {
                for(int BYTE =  21, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            public byte[] satellite_elevation //Elevation (0: right on top of receiver, 90: on the horizon) of satellite
            {
                set {satellite_elevation_SET(value, 0)  ;}
            }
            public void satellite_elevation_SET(byte[] src, int pos)  //Elevation (0: right on top of receiver, 90: on the horizon) of satellite
            {
                for(int BYTE =  41, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            public byte[] satellite_azimuth //Direction of satellite, 0: 0 deg, 255: 360 deg.
            {
                set {satellite_azimuth_SET(value, 0)  ;}
            }
            public void satellite_azimuth_SET(byte[] src, int pos)  //Direction of satellite, 0: 0 deg, 255: 360 deg.
            {
                for(int BYTE =  61, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
            public byte[] satellite_snr //Signal to noise ratio of satellite
            {
                set {satellite_snr_SET(value, 0)  ;}
            }
            public void satellite_snr_SET(byte[] src, int pos)  //Signal to noise ratio of satellite
            {
                for(int BYTE =  81, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }
        }
        public class SCALED_IMU : GroundControl.SCALED_IMU
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public short xacc //X acceleration (mg)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  4);}
            }

            public short yacc //Y acceleration (mg)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  6);}
            }

            public short zacc //Z acceleration (mg)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  8);}
            }

            public short xgyro //Angular speed around X axis (millirad /sec)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public short ygyro //Angular speed around Y axis (millirad /sec)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }

            public short zgyro //Angular speed around Z axis (millirad /sec)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  14);}
            }

            public short xmag //X Magnetic field (milli tesla)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  16);}
            }

            public short ymag //Y Magnetic field (milli tesla)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  18);}
            }

            public short zmag //Z Magnetic field (milli tesla)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  20);}
            }
        }
        public class RAW_IMU : GroundControl.RAW_IMU
        {
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public short xacc //X acceleration (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  8);}
            }

            public short yacc //Y acceleration (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public short zacc //Z acceleration (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }

            public short xgyro //Angular speed around X axis (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  14);}
            }

            public short ygyro //Angular speed around Y axis (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  16);}
            }

            public short zgyro //Angular speed around Z axis (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  18);}
            }

            public short xmag //X Magnetic field (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  20);}
            }

            public short ymag //Y Magnetic field (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  22);}
            }

            public short zmag //Z Magnetic field (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  24);}
            }
        }
        public class RAW_PRESSURE : GroundControl.RAW_PRESSURE
        {
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public short press_abs //Absolute pressure (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  8);}
            }

            public short press_diff1 //Differential pressure 1 (raw, 0 if nonexistant)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public short press_diff2 //Differential pressure 2 (raw, 0 if nonexistant)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }

            public short temperature //Raw Temperature measurement (raw)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  14);}
            }
        }
        public class SCALED_PRESSURE : GroundControl.SCALED_PRESSURE
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float press_abs //Absolute pressure (hectopascal)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float press_diff //Differential pressure 1 (hectopascal)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public short temperature //Temperature measurement (0.01 degrees celsius)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }
        }
        public class ATTITUDE : GroundControl.ATTITUDE
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float roll //Roll angle (rad, -pi..+pi)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float pitch //Pitch angle (rad, -pi..+pi)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float yaw //Yaw angle (rad, -pi..+pi)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float rollspeed //Roll angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float pitchspeed //Pitch angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float yawspeed //Yaw angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }
        }
        public class ATTITUDE_QUATERNION : GroundControl.ATTITUDE_QUATERNION
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float q1 //Quaternion component 1, w (1 in null-rotation)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float q2 //Quaternion component 2, x (0 in null-rotation)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float q3 //Quaternion component 3, y (0 in null-rotation)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float q4 //Quaternion component 4, z (0 in null-rotation)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float rollspeed //Roll angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float pitchspeed //Pitch angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float yawspeed //Yaw angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
        }
        public class LOCAL_POSITION_NED : GroundControl.LOCAL_POSITION_NED
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float x //X Position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float y //Y Position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float z //Z Position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float vx //X Speed
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float vy //Y Speed
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float vz //Z Speed
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }
        }
        public class GLOBAL_POSITION_INT : GroundControl.GLOBAL_POSITION_INT
        {
            public ushort hdg //Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public int lat //Latitude, expressed as degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  6);}
            }

            public int lon //Longitude, expressed as degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  10);}
            }

            /**
            *Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules
            *	provide the AMSL as well*/
            public int alt
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  14);}
            }

            public int relative_alt //Altitude above ground in meters, expressed as * 1000 (millimeters)
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  18);}
            }

            public short vx //Ground X Speed (Latitude, positive north), expressed as m/s * 100
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  22);}
            }

            public short vy //Ground Y Speed (Longitude, positive east), expressed as m/s * 100
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  24);}
            }

            public short vz //Ground Z Speed (Altitude, positive down), expressed as m/s * 100
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  26);}
            }
        }
        public class RC_CHANNELS_SCALED : GroundControl.RC_CHANNELS_SCALED
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            /**
            *Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
            *	8 servos*/
            public byte port
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public short chan1_scaled //RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  5);}
            }

            public short chan2_scaled //RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  7);}
            }

            public short chan3_scaled //RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  9);}
            }

            public short chan4_scaled //RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  11);}
            }

            public short chan5_scaled //RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  13);}
            }

            public short chan6_scaled //RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  15);}
            }

            public short chan7_scaled //RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  17);}
            }

            public short chan8_scaled //RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  19);}
            }

            public byte rssi //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  21);}
            }
        }
        public class RC_CHANNELS_RAW : GroundControl.RC_CHANNELS_RAW
        {
            public ushort chan1_raw //RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort chan2_raw //RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort chan3_raw //RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort chan4_raw //RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort chan5_raw //RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort chan6_raw //RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort chan7_raw //RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort chan8_raw //RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  16);}
            }

            /**
            *Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
            *	8 servos*/
            public byte port
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  20);}
            }

            public byte rssi //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  21);}
            }
        }
        public class SERVO_OUTPUT_RAW : GroundControl.SERVO_OUTPUT_RAW
        {
            public ushort servo1_raw //Servo output 1 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort servo2_raw //Servo output 2 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort servo3_raw //Servo output 3 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort servo4_raw //Servo output 4 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort servo5_raw //Servo output 5 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort servo6_raw //Servo output 6 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort servo7_raw //Servo output 7 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort servo8_raw //Servo output 8 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public uint time_usec //Timestamp (microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  16);}
            }

            /**
            *Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode
            *	more than 8 servos*/
            public byte port
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  20);}
            }
            public void servo9_raw_SET(ushort src, Inside ph)//Servo output 9 value, in microseconds
            {
                if(ph.field_bit != 168)insert_field(ph, 168, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public void servo10_raw_SET(ushort src, Inside ph) //Servo output 10 value, in microseconds
            {
                if(ph.field_bit != 169)insert_field(ph, 169, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public void servo11_raw_SET(ushort src, Inside ph) //Servo output 11 value, in microseconds
            {
                if(ph.field_bit != 170)insert_field(ph, 170, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public void servo12_raw_SET(ushort src, Inside ph) //Servo output 12 value, in microseconds
            {
                if(ph.field_bit != 171)insert_field(ph, 171, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public void servo13_raw_SET(ushort src, Inside ph) //Servo output 13 value, in microseconds
            {
                if(ph.field_bit != 172)insert_field(ph, 172, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public void servo14_raw_SET(ushort src, Inside ph) //Servo output 14 value, in microseconds
            {
                if(ph.field_bit != 173)insert_field(ph, 173, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public void servo15_raw_SET(ushort src, Inside ph) //Servo output 15 value, in microseconds
            {
                if(ph.field_bit != 174)insert_field(ph, 174, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public void servo16_raw_SET(ushort src, Inside ph) //Servo output 16 value, in microseconds
            {
                if(ph.field_bit != 175)insert_field(ph, 175, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            }
        }
        public class MISSION_REQUEST_PARTIAL_LIST : GroundControl.MISSION_REQUEST_PARTIAL_LIST
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public short start_index //Start index, 0 by default
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  2);}
            }

            public short end_index //End index, -1 by default (-1: send list to end). Else a valid index of the list
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  4);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 48);
                }
            }
        }
        public class MISSION_WRITE_PARTIAL_LIST : GroundControl.MISSION_WRITE_PARTIAL_LIST
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public short start_index //Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  2);}
            }

            public short end_index //End index, equal or greater than start index.
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  4);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 48);
                }
            }
        }
        public class MISSION_ITEM : GroundControl.MISSION_ITEM
        {
            public ushort seq //Sequence
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public byte current //false:0, true:1
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte autocontinue //autocontinue to next wp
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public float param1 //PARAM1, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public float param2 //PARAM2, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float param3 //PARAM3, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float param4 //PARAM4, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float x //PARAM5 / local: x position, global: latitude
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            public float y //PARAM6 / y position: global: longitude
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 26);}
            }

            public float z //PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public MAV_FRAME frame //The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 272);}
            }

            public MAV_CMD command //The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 8, data, 276);
                }
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 284);
                }
            }
        }
        public class MISSION_REQUEST : GroundControl.MISSION_REQUEST
        {
            public ushort seq //Sequence
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 32);
                }
            }
        }
        public class MISSION_SET_CURRENT : GroundControl.MISSION_SET_CURRENT
        {
            public ushort seq //Sequence
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }
        }
        public class MISSION_CURRENT : GroundControl.MISSION_CURRENT
        {
            public ushort seq //Sequence
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }
        }
        public class MISSION_REQUEST_LIST : GroundControl.MISSION_REQUEST_LIST
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 16);
                }
            }
        }
        public class MISSION_COUNT : GroundControl.MISSION_COUNT
        {
            public ushort count //Number of mission items in the sequence
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 32);
                }
            }
        }
        public class MISSION_CLEAR_ALL : GroundControl.MISSION_CLEAR_ALL
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 16);
                }
            }
        }
        public class MISSION_ITEM_REACHED : GroundControl.MISSION_ITEM_REACHED
        {
            public ushort seq //Sequence
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }
        }
        public class MISSION_ACK : GroundControl.MISSION_ACK
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public MAV_MISSION_RESULT type //See MAV_MISSION_RESULT enum
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 16);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 20);
                }
            }
        }
        public class SET_GPS_GLOBAL_ORIGIN : GroundControl.SET_GPS_GLOBAL_ORIGIN
        {
            public byte target_system //System ID
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
            public void time_usec_SET(ulong src, Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                if(ph.field_bit != 104)insert_field(ph, 104, 0);
                BitUtils.set_bytes((ulong)(src), 8, data,  ph.BYTE);
            }
        }
        public class GPS_GLOBAL_ORIGIN : GroundControl.GPS_GLOBAL_ORIGIN
        {
            public int latitude //Latitude (WGS84), in degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  0);}
            }

            public int longitude //Longitude (WGS84), in degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  4);}
            }

            public int altitude //Altitude (AMSL), in meters * 1000 (positive for up)
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  8);}
            }
            public void time_usec_SET(ulong src, Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                if(ph.field_bit != 96)insert_field(ph, 96, 0);
                BitUtils.set_bytes((ulong)(src), 8, data,  ph.BYTE);
            }
        }
        public class PARAM_MAP_RC : GroundControl.PARAM_MAP_RC
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            /**
            *Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored),
            *	send -2 to disable any existing map for this rc_channel_index*/
            public short param_index
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  2);}
            }

            /**
            *Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob
            *	on the RC*/
            public byte parameter_rc_channel_index
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public float param_value0 //Initial parameter value
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 5);}
            }

            public float scale //Scale, maps the RC range [-1, 1] to a parameter value
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 9);}
            }

            /**
            *Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends
            *	on implementation*/
            public float param_value_min
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 13);}
            }

            /**
            *Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends
            *	on implementation*/
            public float param_value_max
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }
            /**
            *Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
            *	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
            *	storage if the ID is stored as strin*/
            public void param_id_SET(string src, Inside ph)
            {param_id_SET(src.ToCharArray(), 0, src.Length, ph);}/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
            public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 168 && insert_field(ph, 168, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            }
        }
        public class MISSION_REQUEST_INT : GroundControl.MISSION_REQUEST_INT
        {
            public ushort seq //Sequence
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 32);
                }
            }
        }
        public class SAFETY_SET_ALLOWED_AREA : GroundControl.SAFETY_SET_ALLOWED_AREA
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public float p1x //x position 1 / Latitude 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 2);}
            }

            public float p1y //y position 1 / Longitude 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public float p1z //z position 1 / Altitude 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float p2x //x position 2 / Latitude 2
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float p2y //y position 2 / Longitude 2
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float p2z //z position 2 / Altitude 2
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            /**
            *Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
            *	with Z axis up or local, right handed, Z axis down*/
            public MAV_FRAME frame
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 208);}
            }
        }
        public class SAFETY_ALLOWED_AREA : GroundControl.SAFETY_ALLOWED_AREA
        {
            public float p1x //x position 1 / Latitude 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 0);}
            }

            public float p1y //y position 1 / Longitude 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float p1z //z position 1 / Altitude 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float p2x //x position 2 / Latitude 2
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float p2y //y position 2 / Longitude 2
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float p2z //z position 2 / Altitude 2
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            /**
            *Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
            *	with Z axis up or local, right handed, Z axis down*/
            public MAV_FRAME frame
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 192);}
            }
        }
        public class ATTITUDE_QUATERNION_COV : GroundControl.ATTITUDE_QUATERNION_COV
        {
            public ulong time_usec //Timestamp (microseconds since system boot or since UNIX epoch)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float[] q //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
            {
                set {q_SET(value, 0)  ;}
            }
            public void q_SET(float[] src, int pos)  //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
            {
                for(int BYTE =  8, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
            public float rollspeed //Roll angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float pitchspeed //Pitch angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float yawspeed //Yaw angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float[] covariance //Attitude covariance
            {
                set {covariance_SET(value, 0)  ;}
            }
            public void covariance_SET(float[] src, int pos)  //Attitude covariance
            {
                for(int BYTE =  36, src_max = pos + 9; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
        }
        public class NAV_CONTROLLER_OUTPUT : GroundControl.NAV_CONTROLLER_OUTPUT
        {
            public ushort wp_dist //Distance to active waypoint in meters
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public float nav_roll //Current desired roll in degrees
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 2);}
            }

            public float nav_pitch //Current desired pitch in degrees
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public short nav_bearing //Current desired heading in degrees
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public short target_bearing //Bearing to current waypoint/target in degrees
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }

            public float alt_error //Current altitude error in meters
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float aspd_error //Current airspeed error in meters/second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float xtrack_error //Current crosstrack error on x-y plane in meters
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }
        }
        public class GLOBAL_POSITION_INT_COV : GroundControl.GLOBAL_POSITION_INT_COV
        {
            public ulong time_usec //Timestamp (microseconds since system boot or since UNIX epoch)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public int lat //Latitude, expressed as degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  8);}
            }

            public int lon //Longitude, expressed as degrees * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  12);}
            }

            public int alt //Altitude in meters, expressed as * 1000 (millimeters), above MSL
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  16);}
            }

            public int relative_alt //Altitude above ground in meters, expressed as * 1000 (millimeters)
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  20);}
            }

            public float vx //Ground X Speed (Latitude), expressed as m/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float vy //Ground Y Speed (Longitude), expressed as m/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float vz //Ground Z Speed (Altitude), expressed as m/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float[] covariance //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
            {
                set {covariance_SET(value, 0)  ;}
            }
            public void covariance_SET(float[] src, int pos)  //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
            {
                for(int BYTE =  36, src_max = pos + 36; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
            public MAV_ESTIMATOR_TYPE estimator_type //Class id of the estimator this estimate originated from.
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 3, data, 1440);}
            }
        }
        public class LOCAL_POSITION_NED_COV : GroundControl.LOCAL_POSITION_NED_COV
        {
            public ulong time_usec //Timestamp (microseconds since system boot or since UNIX epoch)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x //X Position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Y Position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Z Position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float vx //X Speed (m/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float vy //Y Speed (m/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float vz //Z Speed (m/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float ax //X Acceleration (m/s^2)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float ay //Y Acceleration (m/s^2)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float az //Z Acceleration (m/s^2)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            /**
            *Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
            *	the second row, etc.*/
            public float[] covariance
            {
                set {covariance_SET(value, 0)  ;}
            }
            /**
            *Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
            *	the second row, etc.*/
            public void covariance_SET(float[] src, int pos)
            {
                for(int BYTE =  44, src_max = pos + 45; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
            public MAV_ESTIMATOR_TYPE estimator_type //Class id of the estimator this estimate originated from.
            {
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 3, data, 1792);}
            }
        }
        public class RC_CHANNELS : GroundControl.RC_CHANNELS
        {
            public ushort chan1_raw //RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort chan2_raw //RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort chan3_raw //RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort chan4_raw //RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort chan5_raw //RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort chan6_raw //RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort chan7_raw //RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort chan8_raw //RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public ushort chan9_raw //RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  16);}
            }

            public ushort chan10_raw //RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  18);}
            }

            public ushort chan11_raw //RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  20);}
            }

            public ushort chan12_raw //RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  22);}
            }

            public ushort chan13_raw //RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  24);}
            }

            public ushort chan14_raw //RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  26);}
            }

            public ushort chan15_raw //RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  28);}
            }

            public ushort chan16_raw //RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  30);}
            }

            public ushort chan17_raw //RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  32);}
            }

            public ushort chan18_raw //RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  34);}
            }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  36);}
            }

            /**
            *Total number of RC channels being received. This can be larger than 18, indicating that more channels
            *	are available but not given in this message. This value should be 0 when no RC channels are available*/
            public byte chancount
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  40);}
            }

            public byte rssi //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  41);}
            }
        }
        public class REQUEST_DATA_STREAM : GroundControl.REQUEST_DATA_STREAM
        {
            public ushort req_message_rate //The requested message rate
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //The target requested to send the message stream.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //The target requested to send the message stream.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public byte req_stream_id //The ID of the requested data stream
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte start_stop //1 to start sending, 0 to stop sending.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }
        }
        public class DATA_STREAM : GroundControl.DATA_STREAM
        {
            public ushort message_rate //The message rate
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte stream_id //The ID of the requested data stream
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte on_off //1 stream is enabled, 0 stream is stopped.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }
        }
        public class MANUAL_CONTROL : GroundControl.MANUAL_CONTROL
        {
            /**
            *A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest
            *	bit corresponds to Button 1*/
            public ushort buttons
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target //The system to be controlled.
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            /**
            *X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            *	Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle*/
            public short x
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  3);}
            }

            /**
            *Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            *	Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle*/
            public short y
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  5);}
            }

            /**
            *Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            *	Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on
            *	a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
            *	thrust*/
            public short z
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  7);}
            }

            /**
            *R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            *	Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise
            *	being -1000, and the yaw of a vehicle*/
            public short r
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  9);}
            }
        }
        public class RC_CHANNELS_OVERRIDE : GroundControl.RC_CHANNELS_OVERRIDE
        {
            public ushort chan1_raw //RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort chan2_raw //RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort chan3_raw //RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort chan4_raw //RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort chan5_raw //RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort chan6_raw //RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort chan7_raw //RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort chan8_raw //RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  17);}
            }
        }
        public class MISSION_ITEM_INT : GroundControl.MISSION_ITEM_INT
        {
            /**
            *Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the
            *	sequence (0,1,2,3,4)*/
            public ushort seq
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public byte current //false:0, true:1
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte autocontinue //autocontinue to next wp
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public float param1 //PARAM1, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public float param2 //PARAM2, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float param3 //PARAM3, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float param4 //PARAM4, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public int x //PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  22);}
            }

            public int y //PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  26);}
            }

            public float z //PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public MAV_FRAME frame //The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 272);}
            }

            public MAV_CMD command //The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 8, data, 276);
                }
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 284);
                }
            }
        }
        public class VFR_HUD : GroundControl.VFR_HUD
        {
            public ushort throttle //Current throttle setting in integer percent, 0 to 100
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public float airspeed //Current airspeed in m/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 2);}
            }

            public float groundspeed //Current ground speed in m/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public short heading //Current heading in degrees, in compass units (0..360, 0=north)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public float alt //Current altitude (MSL), in meters
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float climb //Current climb rate in meters/second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }
        }
        public class COMMAND_INT : GroundControl.COMMAND_INT
        {
            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public byte current //false:0, true:1
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte autocontinue //autocontinue to next wp
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public float param1 //PARAM1, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float param2 //PARAM2, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float param3 //PARAM3, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float param4 //PARAM4, see MAV_CMD enum
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public int x //PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  20);}
            }

            public int y //PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  24);}
            }

            public float z //PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public MAV_FRAME frame //The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 256);}
            }

            public MAV_CMD command //The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 8, data, 260);
                }
            }
        }
        public class COMMAND_LONG : GroundControl.COMMAND_LONG
        {
            public byte target_system //System which should execute the command
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component which should execute the command, 0 for all components
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public byte confirmation //0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public float param1 //Parameter 1, as defined by MAV_CMD enum.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 3);}
            }

            public float param2 //Parameter 2, as defined by MAV_CMD enum.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 7);}
            }

            public float param3 //Parameter 3, as defined by MAV_CMD enum.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 11);}
            }

            public float param4 //Parameter 4, as defined by MAV_CMD enum.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 15);}
            }

            public float param5 //Parameter 5, as defined by MAV_CMD enum.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 19);}
            }

            public float param6 //Parameter 6, as defined by MAV_CMD enum.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 23);}
            }

            public float param7 //Parameter 7, as defined by MAV_CMD enum.
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 27);}
            }

            public MAV_CMD command //Command ID, as defined by MAV_CMD enum.
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 8, data, 248);
                }
            }
        }
        public class COMMAND_ACK : GroundControl.COMMAND_ACK
        {
            public MAV_CMD command //Command ID, as defined by MAV_CMD enum.
            {
                set
                {
                    ulong id = 0;
                    switch(value)
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
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 8, data, 0);
                }
            }

            public MAV_RESULT result //See MAV_RESULT enum
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 8);}
            }
            /**
            *WIP: Also used as result_param1, it can be set with a enum containing the errors reasons of why the command
            *	was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS*/
            public void progress_SET(byte src, Inside ph)
            {
                if(ph.field_bit != 11)insert_field(ph, 11, 0);
                BitUtils.set_bytes((ulong)(src), 1, data,  ph.BYTE);
            }/**
*WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to
*	be denied*/
            public void result_param2_SET(int src, Inside ph)
            {
                if(ph.field_bit != 12)insert_field(ph, 12, 0);
                BitUtils.set_bytes((uint)(src), 4, data,  ph.BYTE);
            } public void target_system_SET(byte src, Inside ph) //WIP: System which requested the command to be executed
            {
                if(ph.field_bit != 13)insert_field(ph, 13, 0);
                BitUtils.set_bytes((ulong)(src), 1, data,  ph.BYTE);
            } public void target_component_SET(byte src, Inside ph) //WIP: Component which requested the command to be executed
            {
                if(ph.field_bit != 14)insert_field(ph, 14, 0);
                BitUtils.set_bytes((ulong)(src), 1, data,  ph.BYTE);
            }
        }
        new class BATTERY_STATUS : GroundControl.BATTERY_STATUS
        {
            /**
            *Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
            *	should have the UINT16_MAX value*/
            public ushort[] voltages
            {
                get {return voltages_GET(new ushort[10], 0);}
            }
            /**
            *Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
            *	should have the UINT16_MAX value*/
            public ushort[]voltages_GET(ushort[] dst_ch, int pos)
            {
                for(int BYTE = 0, dst_max = pos + 10; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public byte id //Battery ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  20, 1));}
            }

            public short temperature //Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  21, 2));}
            }

            public short current_battery //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  23, 2));}
            }

            public int current_consumed //Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimat
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  25, 4));}
            }

            /**
            *Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide
            *	energy consumption estimat*/
            public int energy_consumed
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  29, 4));}
            }

            public sbyte battery_remaining //Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  33, 1));}
            }

            public MAV_BATTERY_FUNCTION battery_function //Function of the battery
            {
                get {  return (MAV_BATTERY_FUNCTION)(0 +  BitUtils.get_bits(data, 272, 3));}
            }

            public MAV_BATTERY_TYPE type //Type (chemistry) of the battery
            {
                get {  return (MAV_BATTERY_TYPE)(0 +  BitUtils.get_bits(data, 275, 3));}
            }
        }
        new class AUTOPILOT_VERSION : GroundControl.AUTOPILOT_VERSION
        {
            public ushort vendor_id //ID of the board vendor
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort product_id //ID of the product
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public uint flight_sw_version //Firmware version number
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
            }

            public uint middleware_sw_version //Middleware version number
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  8, 4));}
            }

            public uint os_sw_version //Operating system version number
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  12, 4));}
            }

            public uint board_version //HW / board version (last 8 bytes should be silicon ID, if any)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  16, 4));}
            }

            public ulong uid //UID if provided by hardware (see uid2)
            {
                get {  return (BitUtils.get_bytes(data,  20, 8));}
            }

            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	should allow to identify the commit using the main version number even for very large code bases*/
            public byte[] flight_custom_version
            {
                get {return flight_custom_version_GET(new byte[8], 0);}
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	should allow to identify the commit using the main version number even for very large code bases*/
            public byte[]flight_custom_version_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 28, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	should allow to identify the commit using the main version number even for very large code bases*/
            public byte[] middleware_custom_version
            {
                get {return middleware_custom_version_GET(new byte[8], 0);}
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	should allow to identify the commit using the main version number even for very large code bases*/
            public byte[]middleware_custom_version_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 36, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	should allow to identify the commit using the main version number even for very large code bases*/
            public byte[] os_custom_version
            {
                get {return os_custom_version_GET(new byte[8], 0);}
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	should allow to identify the commit using the main version number even for very large code bases*/
            public byte[]os_custom_version_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 44, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public MAV_PROTOCOL_CAPABILITY capabilities //bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 416, 5))
                    {
                        case 0:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
                        case 1:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
                        case 2:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT;
                        case 3:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT;
                        case 4:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION;
                        case 5:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP;
                        case 6:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
                        case 7:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED;
                        case 8:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT;
                        case 9:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN;
                        case 10:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET;
                        case 11:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION;
                        case 12:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION;
                        case 13:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2;
                        case 14:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE;
                        case 15:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;
                        case 16:
                            return MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }
            /**
            *UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
            *	use uid*/
            public byte[] uid2_TRY(Inside ph)
            {
                if(ph.field_bit !=  421 && !try_visit_field(ph, 421)) return null;
                return uid2_GET(ph, new byte[ph.items], 0);
            }
            /**
            *UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
            *	use uid*/
            public byte[]uid2_GET(Inside ph, byte[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + 18; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public int uid2_LEN()
            {
                return 18;
            }
        }
        new class LANDING_TARGET : GroundControl.LANDING_TARGET
        {
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public byte target_num //The ID of the target if multiple targets are present
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public float angle_x //X-axis angular offset (in radians) of the target from the center of the image
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  9, 4)));}
            }

            public float angle_y //Y-axis angular offset (in radians) of the target from the center of the image
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  13, 4)));}
            }

            public float distance //Distance to the target from the vehicle in meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
            }

            public float size_x //Size in radians of target along x-axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  21, 4)));}
            }

            public float size_y //Size in radians of target along y-axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  25, 4)));}
            }

            public MAV_FRAME frame //MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 232, 4));}
            }

            public LANDING_TARGET_TYPE type //LANDING_TARGET_TYPE enum specifying the type of landing target
            {
                get {  return (LANDING_TARGET_TYPE)(0 +  BitUtils.get_bits(data, 236, 3));}
            }
            public float x_TRY(Inside ph)//X Position of the landing target on MAV_FRAME
            {
                if(ph.field_bit !=  239 && !try_visit_field(ph, 239)) return 0;
                return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  ph.BYTE, 4)));
            }
            public float y_TRY(Inside ph)//Y Position of the landing target on MAV_FRAME
            {
                if(ph.field_bit !=  240 && !try_visit_field(ph, 240)) return 0;
                return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  ph.BYTE, 4)));
            }
            public float z_TRY(Inside ph)//Z Position of the landing target on MAV_FRAME
            {
                if(ph.field_bit !=  241 && !try_visit_field(ph, 241)) return 0;
                return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  ph.BYTE, 4)));
            }
            public float[] q_TRY(Inside ph)//Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                if(ph.field_bit !=  242 && !try_visit_field(ph, 242)) return null;
                return q_GET(ph, new float[ph.items], 0);
            }
            public float[]q_GET(Inside ph, float[] dst_ch, int pos) //Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public int q_LEN()
            {
                return 4;
            }/**
*Boolean indicating known position (1) or default unkown position (0), for validation of positioning of
*	the landing targe*/
            public byte position_valid_TRY(Inside ph)
            {
                if(ph.field_bit !=  243 && !try_visit_field(ph, 243)) return 0;
                return (byte)((byte) BitUtils.get_bytes(data,  ph.BYTE, 1));
            }
        }
        new class SENS_POWER : GroundControl.SENS_POWER
        {
            public float adc121_vspb_volt //Power board voltage sensor reading in volts
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float adc121_cspb_amp //Power board current sensor reading in amps
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float adc121_cs1_amp //Board current sensor 1 reading in amps
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float adc121_cs2_amp //Board current sensor 2 reading in amps
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }
        }
        new class SENS_MPPT : GroundControl.SENS_MPPT
        {
            public ushort mppt1_pwm //MPPT1 pwm
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort mppt2_pwm //MPPT2 pwm
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort mppt3_pwm //MPPT3 pwm
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public ulong mppt_timestamp //MPPT last timestamp
            {
                get {  return (BitUtils.get_bytes(data,  6, 8));}
            }

            public float mppt1_volt //MPPT1 voltage
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float mppt1_amp //MPPT1 current
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public byte mppt1_status //MPPT1 status
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  22, 1));}
            }

            public float mppt2_volt //MPPT2 voltage
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  23, 4)));}
            }

            public float mppt2_amp //MPPT2 current
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  27, 4)));}
            }

            public byte mppt2_status //MPPT2 status
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  31, 1));}
            }

            public float mppt3_volt //MPPT3 voltage
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float mppt3_amp //MPPT3 current
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public byte mppt3_status //MPPT3 status
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  40, 1));}
            }
        }
        new class ASLCTRL_DATA : GroundControl.ASLCTRL_DATA
        {
            public ulong timestamp //Timestamp
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public byte aslctrl_mode //ASLCTRL control-mode (manual, stabilized, auto, etc...)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public float h //See sourcecode for a description of these values...
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  9, 4)));}
            }

            public float hRef //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  13, 4)));}
            }

            public float hRef_t //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
            }

            public float PitchAngle //Pitch angle [deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  21, 4)));}
            }

            public float PitchAngleRef //Pitch angle reference[deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  25, 4)));}
            }

            public float q //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  29, 4)));}
            }

            public float qRef //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  33, 4)));}
            }

            public float uElev //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  37, 4)));}
            }

            public float uThrot //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  41, 4)));}
            }

            public float uThrot2 //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  45, 4)));}
            }

            public float nZ //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  49, 4)));}
            }

            public float AirspeedRef //Airspeed reference [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  53, 4)));}
            }

            public byte SpoilersEngaged //null
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  57, 1));}
            }

            public float YawAngle //Yaw angle [deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  58, 4)));}
            }

            public float YawAngleRef //Yaw angle reference[deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  62, 4)));}
            }

            public float RollAngle //Roll angle [deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  66, 4)));}
            }

            public float RollAngleRef //Roll angle reference[deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  70, 4)));}
            }

            public float p //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  74, 4)));}
            }

            public float pRef //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  78, 4)));}
            }

            public float r //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  82, 4)));}
            }

            public float rRef //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  86, 4)));}
            }

            public float uAil //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  90, 4)));}
            }

            public float uRud //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  94, 4)));}
            }
        }
        new class ASLCTRL_DEBUG : GroundControl.ASLCTRL_DEBUG
        {
            public uint i32_1 //Debug data
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte i8_1 //Debug data
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte i8_2 //Debug data
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public float f_1 //Debug data
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }

            public float f_2 //Debug data
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }

            public float f_3 //Debug data
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float f_4 //Debug data
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public float f_5 //Debug data
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
            }

            public float f_6 //Debug data
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
            }

            public float f_7 //Debug data
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
            }

            public float f_8 //Debug data
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  34, 4)));}
            }
        }
        new class ASLUAV_STATUS : GroundControl.ASLUAV_STATUS
        {
            public byte LED_status //Status of the position-indicator LEDs
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte SATCOM_status //Status of the IRIDIUM satellite communication system
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte[] Servo_status //Status vector for up to 8 servos
            {
                get {return Servo_status_GET(new byte[8], 0);}
            }
            public byte[]Servo_status_GET(byte[] dst_ch, int pos)  //Status vector for up to 8 servos
            {
                for(int BYTE = 2, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public float Motor_rpm //Motor RPM
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }
        }
        new class EKF_EXT : GroundControl.EKF_EXT
        {
            public ulong timestamp //Time since system start [us]
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float Windspeed //Magnitude of wind velocity (in lateral inertial plane) [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float WindDir //Wind heading angle from North [rad]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float WindZ //Z (Down) component of inertial wind velocity [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float Airspeed //Magnitude of air velocity [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float beta //Sideslip angle [rad]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float alpha //Angle of attack [rad]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }
        }
        new class ASL_OBCTRL : GroundControl.ASL_OBCTRL
        {
            public ulong timestamp //Time since system start [us]
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float uElev //Elevator command [~]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float uThrot //Throttle command [~]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float uThrot2 //Throttle 2 command [~]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float uAilL //Left aileron command [~]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float uAilR //Right aileron command [~]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float uRud //Rudder command [~]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public byte obctrl_status //Off-board computer status
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  32, 1));}
            }
        }
        new class SENS_ATMOS : GroundControl.SENS_ATMOS
        {
            public float TempAmbient //Ambient temperature [degrees Celsius]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float Humidity //Relative humidity [%]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }
        }
        new class SENS_BATMON : GroundControl.SENS_BATMON
        {
            public ushort voltage //Battery pack voltage in [mV]
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort batterystatus //Battery monitor status report bits in Hex
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort serialnumber //Battery monitor serial number in Hex
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public ushort hostfetcontrol //Battery monitor sensor host FET control in Hex
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
            }

            public ushort cellvoltage1 //Battery pack cell 1 voltage in [mV]
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  8, 2));}
            }

            public ushort cellvoltage2 //Battery pack cell 2 voltage in [mV]
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  10, 2));}
            }

            public ushort cellvoltage3 //Battery pack cell 3 voltage in [mV]
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  12, 2));}
            }

            public ushort cellvoltage4 //Battery pack cell 4 voltage in [mV]
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  14, 2));}
            }

            public ushort cellvoltage5 //Battery pack cell 5 voltage in [mV]
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  16, 2));}
            }

            public ushort cellvoltage6 //Battery pack cell 6 voltage in [mV]
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  18, 2));}
            }

            public float temperature //Battery pack temperature in [deg C]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public short current //Battery pack current in [mA]
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  24, 2));}
            }

            public byte SoC //Battery pack state-of-charge
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  26, 1));}
            }
        }
        new class FW_SOARING_DATA : GroundControl.FW_SOARING_DATA
        {
            public ulong timestamp //Timestamp [ms]
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public ulong timestampModeChanged //Timestamp since last mode change[ms]
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
            }

            public float xW //Thermal core updraft strength [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float xR //Thermal radius [m]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float xLat //Thermal center latitude [deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float xLon //Thermal center longitude [deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float VarW //Variance W
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float VarR //Variance R
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public float VarLat //Variance Lat
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
            }

            public float VarLon //Variance Lon
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
            }

            public float LoiterRadius //Suggested loiter radius [m]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  48, 4)));}
            }

            public float LoiterDirection //Suggested loiter direction
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  52, 4)));}
            }

            public float DistToSoarPoint //Distance to soar point [m]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  56, 4)));}
            }

            public float vSinkExp //Expected sink rate at current airspeed, roll and throttle [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  60, 4)));}
            }

            public float z1_LocalUpdraftSpeed //Measurement / updraft speed at current/local airplane position [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  64, 4)));}
            }

            public float z2_DeltaRoll //Measurement / roll angle tracking error [deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  68, 4)));}
            }

            public float z1_exp //Expected measurement 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  72, 4)));}
            }

            public float z2_exp //Expected measurement 2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  76, 4)));}
            }

            public float ThermalGSNorth //Thermal drift (from estimator prediction step only) [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  80, 4)));}
            }

            public float ThermalGSEast //Thermal drift (from estimator prediction step only) [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  84, 4)));}
            }

            public float TSE_dot //Total specific energy change (filtered) [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  88, 4)));}
            }

            public float DebugVar1 //Debug variable 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  92, 4)));}
            }

            public float DebugVar2 //Debug variable 2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  96, 4)));}
            }

            public byte ControlMode //Control Mode [-]
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  100, 1));}
            }

            public byte valid //Data valid [-]
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  101, 1));}
            }
        }
        new class SENSORPOD_STATUS : GroundControl.SENSORPOD_STATUS
        {
            public ushort free_space //Free space available in recordings directory in [Gb] * 1e2
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ulong timestamp //Timestamp in linuxtime [ms] (since 1.1.1970)
            {
                get {  return (BitUtils.get_bytes(data,  2, 8));}
            }

            public byte visensor_rate_1 //Rate of ROS topic 1
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
            }

            public byte visensor_rate_2 //Rate of ROS topic 2
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
            }

            public byte visensor_rate_3 //Rate of ROS topic 3
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  12, 1));}
            }

            public byte visensor_rate_4 //Rate of ROS topic 4
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  13, 1));}
            }

            public byte recording_nodes_count //Number of recording nodes
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  14, 1));}
            }

            public byte cpu_temp //Temperature of sensorpod CPU in [deg C]
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  15, 1));}
            }
        }
        new class SENS_POWER_BOARD : GroundControl.SENS_POWER_BOARD
        {
            public ulong timestamp //Timestamp
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public byte pwr_brd_status //Power board status register
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte pwr_brd_led_status //Power board leds status
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public float pwr_brd_system_volt //Power board system voltage
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }

            public float pwr_brd_servo_volt //Power board servo voltage
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float pwr_brd_mot_l_amp //Power board left motor current sensor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public float pwr_brd_mot_r_amp //Power board right motor current sensor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
            }

            public float pwr_brd_servo_1_amp //Power board servo1 current sensor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
            }

            public float pwr_brd_servo_2_amp //Power board servo1 current sensor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
            }

            public float pwr_brd_servo_3_amp //Power board servo1 current sensor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  34, 4)));}
            }

            public float pwr_brd_servo_4_amp //Power board servo1 current sensor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  38, 4)));}
            }

            public float pwr_brd_aux_amp //Power board aux current sensor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  42, 4)));}
            }
        }
        new class ESTIMATOR_STATUS : GroundControl.ESTIMATOR_STATUS
        {
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float vel_ratio //Velocity innovation test ratio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float pos_horiz_ratio //Horizontal position innovation test ratio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float pos_vert_ratio //Vertical position innovation test ratio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float mag_ratio //Magnetometer innovation test ratio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float hagl_ratio //Height above terrain innovation test ratio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float tas_ratio //True airspeed innovation test ratio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float pos_horiz_accuracy //Horizontal position 1-STD accuracy relative to the EKF local origin (m)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float pos_vert_accuracy //Vertical position 1-STD accuracy relative to the EKF local origin (m)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public ESTIMATOR_STATUS_FLAGS flags //Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS.
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 320, 4))
                    {
                        case 0:
                            return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE;
                        case 1:
                            return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ;
                        case 2:
                            return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT;
                        case 3:
                            return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL;
                        case 4:
                            return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS;
                        case 5:
                            return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS;
                        case 6:
                            return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL;
                        case 7:
                            return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE;
                        case 8:
                            return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL;
                        case 9:
                            return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS;
                        case 10:
                            return ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }
        }
        new class WIND_COV : GroundControl.WIND_COV
        {
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float wind_x //Wind in X (NED) direction in m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float wind_y //Wind in Y (NED) direction in m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float wind_z //Wind in Z (NED) direction in m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float var_horiz //Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float var_vert //Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float wind_alt //AMSL altitude (m) this measurement was taken at
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float horiz_accuracy //Horizontal speed 1-STD accuracy
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float vert_accuracy //Vertical speed 1-STD accuracy
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }
        }
        new class GPS_INPUT : GroundControl.GPS_INPUT
        {
            public ushort time_week //GPS week number
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public uint time_week_ms //GPS time (milliseconds from start of GPS week)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
            }

            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  6, 8));}
            }

            public byte gps_id //ID of the GPS for multiple GPS inputs
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  14, 1));}
            }

            public byte fix_type //0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  15, 1));}
            }

            public int lat //Latitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  16, 4));}
            }

            public int lon //Longitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
            }

            public float alt //Altitude (AMSL, not WGS84), in m (positive for up)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float hdop //GPS HDOP horizontal dilution of position in m
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float vdop //GPS VDOP vertical dilution of position in m
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float vn //GPS velocity in m/s in NORTH direction in earth-fixed NED frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public float ve //GPS velocity in m/s in EAST direction in earth-fixed NED frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
            }

            public float vd //GPS velocity in m/s in DOWN direction in earth-fixed NED frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
            }

            public float speed_accuracy //GPS speed accuracy in m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  48, 4)));}
            }

            public float horiz_accuracy //GPS horizontal accuracy in m
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  52, 4)));}
            }

            public float vert_accuracy //GPS vertical accuracy in m
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  56, 4)));}
            }

            public byte satellites_visible //Number of satellites visible.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  60, 1));}
            }

            public GPS_INPUT_IGNORE_FLAGS ignore_flags //Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 488, 4))
                    {
                        case 0:
                            return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT;
                        case 1:
                            return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP;
                        case 2:
                            return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP;
                        case 3:
                            return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ;
                        case 4:
                            return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT;
                        case 5:
                            return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY;
                        case 6:
                            return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY;
                        case 7:
                            return GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }
        }
        new class GPS_RTCM_DATA : GroundControl.GPS_RTCM_DATA
        {
            /**
            *LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for
            *	the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
            *	on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
            *	while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
            *	fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
            *	with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
            *	corrupt RTCM data, and to recover from a unreliable transport delivery order*/
            public byte flags
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte len //data length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte[] data_ //RTCM message (may be fragmented)
            {
                get {return data__GET(new byte[180], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //RTCM message (may be fragmented)
            {
                for(int BYTE = 2, dst_max = pos + 180; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class HIGH_LATENCY : GroundControl.HIGH_LATENCY
        {
            public ushort heading //heading (centidegrees)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort wp_distance //distance to target (meters)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public uint custom_mode //A bitfield for use for autopilot-specific flags.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
            }

            public short roll //roll (centidegrees)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
            }

            public short pitch //pitch (centidegrees)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
            }

            public sbyte throttle //throttle (percentage)
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  12, 1));}
            }

            public short heading_sp //heading setpoint (centidegrees)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  13, 2));}
            }

            public int latitude //Latitude, expressed as degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  15, 4));}
            }

            public int longitude //Longitude, expressed as degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  19, 4));}
            }

            public short altitude_amsl //Altitude above mean sea level (meters)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  23, 2));}
            }

            public short altitude_sp //Altitude setpoint relative to the home position (meters)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  25, 2));}
            }

            public byte airspeed //airspeed (m/s)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  27, 1));}
            }

            public byte airspeed_sp //airspeed setpoint (m/s)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  28, 1));}
            }

            public byte groundspeed //groundspeed (m/s)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  29, 1));}
            }

            public sbyte climb_rate //climb rate (m/s)
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  30, 1));}
            }

            public byte gps_nsat //Number of satellites visible. If unknown, set to 255
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  31, 1));}
            }

            public byte battery_remaining //Remaining battery (percentage)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  32, 1));}
            }

            public sbyte temperature //Autopilot temperature (degrees C)
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  33, 1));}
            }

            public sbyte temperature_air //Air temperature (degrees C) from airspeed sensor
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  34, 1));}
            }

            /**
            *failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS,
            *	bit3:GCS, bit4:fence*/
            public byte failsafe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  35, 1));}
            }

            public byte wp_num //current waypoint number
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  36, 1));}
            }

            public MAV_MODE_FLAG base_mode //System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 296, 4))
                    {
                        case 0:
                            return MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
                        case 1:
                            return MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED;
                        case 2:
                            return MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED;
                        case 3:
                            return MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
                        case 4:
                            return MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED;
                        case 5:
                            return MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED;
                        case 6:
                            return MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
                        case 7:
                            return MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }

            public MAV_LANDED_STATE landed_state //The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
            {
                get {  return (MAV_LANDED_STATE)(0 +  BitUtils.get_bits(data, 300, 3));}
            }

            public GPS_FIX_TYPE gps_fix_type //See the GPS_FIX_TYPE enum.
            {
                get {  return (GPS_FIX_TYPE)(0 +  BitUtils.get_bits(data, 303, 4));}
            }
        }
        new class VIBRATION : GroundControl.VIBRATION
        {
            public uint clipping_0 //first accelerometer clipping count
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public uint clipping_1 //second accelerometer clipping count
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
            }

            public uint clipping_2 //third accelerometer clipping count
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  8, 4));}
            }

            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  12, 8));}
            }

            public float vibration_x //Vibration levels on X-axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float vibration_y //Vibration levels on Y-axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float vibration_z //Vibration levels on Z-axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }
        }
        new class HOME_POSITION : GroundControl.HOME_POSITION
        {
            public int latitude //Latitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  0, 4));}
            }

            public int longitude //Longitude (WGS84, in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  4, 4));}
            }

            public int altitude //Altitude (AMSL), in meters * 1000 (positive for up)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  8, 4));}
            }

            public float x //Local X position of this position in the local coordinate frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float y //Local Y position of this position in the local coordinate frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float z //Local Z position of this position in the local coordinate frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            /**
            *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
            *	and slope of the groun*/
            public float[] q
            {
                get {return q_GET(new float[4], 0);}
            }
            /**
            *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
            *	and slope of the groun*/
            public float[]q_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 24, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            /**
            *Local X position of the end of the approach vector. Multicopters should set this position based on their
            *	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	from the threshold / touchdown zone*/
            public float approach_x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
            }

            /**
            *Local Y position of the end of the approach vector. Multicopters should set this position based on their
            *	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	from the threshold / touchdown zone*/
            public float approach_y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
            }

            /**
            *Local Z position of the end of the approach vector. Multicopters should set this position based on their
            *	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	from the threshold / touchdown zone*/
            public float approach_z
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  48, 4)));}
            }
            public ulong time_usec_TRY(Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                if(ph.field_bit !=  416 && !try_visit_field(ph, 416)) return 0;
                return (BitUtils.get_bytes(data,  ph.BYTE, 8));
            }
        }
        new class SET_HOME_POSITION : GroundControl.SET_HOME_POSITION
        {
            public byte target_system //System ID.
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

            public float x //Local X position of this position in the local coordinate frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  13, 4)));}
            }

            public float y //Local Y position of this position in the local coordinate frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
            }

            public float z //Local Z position of this position in the local coordinate frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  21, 4)));}
            }

            /**
            *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
            *	and slope of the groun*/
            public float[] q
            {
                get {return q_GET(new float[4], 0);}
            }
            /**
            *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
            *	and slope of the groun*/
            public float[]q_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 25, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            /**
            *Local X position of the end of the approach vector. Multicopters should set this position based on their
            *	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	from the threshold / touchdown zone*/
            public float approach_x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  41, 4)));}
            }

            /**
            *Local Y position of the end of the approach vector. Multicopters should set this position based on their
            *	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	from the threshold / touchdown zone*/
            public float approach_y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  45, 4)));}
            }

            /**
            *Local Z position of the end of the approach vector. Multicopters should set this position based on their
            *	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	from the threshold / touchdown zone*/
            public float approach_z
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  49, 4)));}
            }
            public ulong time_usec_TRY(Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                if(ph.field_bit !=  424 && !try_visit_field(ph, 424)) return 0;
                return (BitUtils.get_bytes(data,  ph.BYTE, 8));
            }
        }
        new class MESSAGE_INTERVAL : GroundControl.MESSAGE_INTERVAL
        {
            public ushort message_id //The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public int interval_us //0 indicates the interval at which it is sent.
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  2, 4));}
            }
        }
        new class EXTENDED_SYS_STATE : GroundControl.EXTENDED_SYS_STATE
        {
            public MAV_VTOL_STATE vtol_state //The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration
            {
                get {  return (MAV_VTOL_STATE)(0 +  BitUtils.get_bits(data, 0, 3));}
            }

            public MAV_LANDED_STATE landed_state //The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
            {
                get {  return (MAV_LANDED_STATE)(0 +  BitUtils.get_bits(data, 3, 3));}
            }
        }
        new class ADSB_VEHICLE : GroundControl.ADSB_VEHICLE
        {
            public ushort heading //Course over ground in centidegrees
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort hor_velocity //The horizontal velocity in centimeters/second
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort squawk //Squawk code
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public uint ICAO_address //ICAO address
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
            }

            public int lat //Latitude, expressed as degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
            }

            public int lon //Longitude, expressed as degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  14, 4));}
            }

            public int altitude //Altitude(ASL) in millimeters
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  18, 4));}
            }

            public short ver_velocity //The vertical velocity in centimeters/second, positive is up
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  22, 2));}
            }

            public byte tslc //Time since last communication in seconds
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  24, 1));}
            }

            public ADSB_ALTITUDE_TYPE altitude_type //Type from ADSB_ALTITUDE_TYPE enum
            {
                get {  return (ADSB_ALTITUDE_TYPE)(0 +  BitUtils.get_bits(data, 200, 2));}
            }

            public ADSB_EMITTER_TYPE emitter_type //Type from ADSB_EMITTER_TYPE enum
            {
                get {  return (ADSB_EMITTER_TYPE)(0 +  BitUtils.get_bits(data, 202, 5));}
            }

            public ADSB_FLAGS flags //Flags to indicate various statuses including valid data fields
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 207, 3))
                    {
                        case 0:
                            return ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS;
                        case 1:
                            return ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE;
                        case 2:
                            return ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING;
                        case 3:
                            return ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY;
                        case 4:
                            return ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN;
                        case 5:
                            return ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK;
                        case 6:
                            return ADSB_FLAGS.ADSB_FLAGS_SIMULATED;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }
            public string callsign_TRY(Inside ph)//The callsign, 8+null
            {
                if(ph.field_bit !=  210 && !try_visit_field(ph, 210)  ||  !try_visit_item(ph, 0)) return null;
                return new string(callsign_GET(ph, new char[ph.items], 0));
            }
            public char[]callsign_GET(Inside ph, char[] dst_ch, int pos) //The callsign, 8+null
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int callsign_LEN(Inside ph)
            {
                return (ph.field_bit !=  210 && !try_visit_field(ph, 210)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class COLLISION : GroundControl.COLLISION
        {
            public uint id //Unique identifier, domain based on src field
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public float time_to_minimum_delta //Estimated time until collision occurs (seconds)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float altitude_minimum_delta //Closest vertical distance in meters between vehicle and object
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float horizontal_minimum_delta //Closest horizontal distance in meteres between vehicle and object
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public MAV_COLLISION_SRC src_ //Collision data source
            {
                get {  return (MAV_COLLISION_SRC)(0 +  BitUtils.get_bits(data, 128, 2));}
            }

            public MAV_COLLISION_ACTION action //Action that is being taken to avoid this collision
            {
                get {  return (MAV_COLLISION_ACTION)(0 +  BitUtils.get_bits(data, 130, 3));}
            }

            public MAV_COLLISION_THREAT_LEVEL threat_level //How concerned the aircraft is about this collision
            {
                get {  return (MAV_COLLISION_THREAT_LEVEL)(0 +  BitUtils.get_bits(data, 133, 2));}
            }
        }
        new class V2_EXTENSION : GroundControl.V2_EXTENSION
        {
            /**
            *A code that identifies the software component that understands this message (analogous to usb device classes
            *	or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
            *	and the corresponding entry should be added to https:github.com/mavlink/mavlink/extension-message-ids.xml.
            *	 Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
            *	Message_types greater than 32767 are considered local experiments and should not be checked in to any
            *	widely distributed codebase*/
            public ushort message_type
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target_network //Network ID (0 for broadcast)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte target_system //System ID (0 for broadcast)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte target_component //Component ID (0 for broadcast)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            /**
            *Variable length payload. The length is defined by the remaining message length when subtracting the header
            *	and other fields.  The entire content of this block is opaque unless you understand any the encoding
            *	message_type.  The particular encoding used can be extension specific and might not always be documented
            *	as part of the mavlink specification*/
            public byte[] payload
            {
                get {return payload_GET(new byte[249], 0);}
            }
            /**
            *Variable length payload. The length is defined by the remaining message length when subtracting the header
            *	and other fields.  The entire content of this block is opaque unless you understand any the encoding
            *	message_type.  The particular encoding used can be extension specific and might not always be documented
            *	as part of the mavlink specification*/
            public byte[]payload_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 5, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class MEMORY_VECT : GroundControl.MEMORY_VECT
        {
            public ushort address //Starting address of the debug variables
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte ver //Version code of the type variable. 0=unknown, type ignored and assumed short. 1=as below
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte type //Type code of the memory variables. for ver = 1: 0=16 x short, 1=16 x ushort, 2=16 x Q15, 3=16 x 1Q1
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public sbyte[] value //Memory contents at specified address
            {
                get {return value_GET(new sbyte[32], 0);}
            }
            public sbyte[]value_GET(sbyte[] dst_ch, int pos)  //Memory contents at specified address
            {
                for(int BYTE = 4, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (sbyte)((sbyte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class DEBUG_VECT : GroundControl.DEBUG_VECT
        {
            public ulong time_usec //Timestamp
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float x //x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float y //y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float z //z
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }
            public string name_TRY(Inside ph)//Name
            {
                if(ph.field_bit !=  160 && !try_visit_field(ph, 160)  ||  !try_visit_item(ph, 0)) return null;
                return new string(name_GET(ph, new char[ph.items], 0));
            }
            public char[]name_GET(Inside ph, char[] dst_ch, int pos) //Name
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int name_LEN(Inside ph)
            {
                return (ph.field_bit !=  160 && !try_visit_field(ph, 160)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class NAMED_VALUE_FLOAT : GroundControl.NAMED_VALUE_FLOAT
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public float value //Floating point value
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }
            public string name_TRY(Inside ph)//Name of the debug variable
            {
                if(ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) return null;
                return new string(name_GET(ph, new char[ph.items], 0));
            }
            public char[]name_GET(Inside ph, char[] dst_ch, int pos) //Name of the debug variable
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int name_LEN(Inside ph)
            {
                return (ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class NAMED_VALUE_INT : GroundControl.NAMED_VALUE_INT
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public int value //Signed integer value
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  4, 4));}
            }
            public string name_TRY(Inside ph)//Name of the debug variable
            {
                if(ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) return null;
                return new string(name_GET(ph, new char[ph.items], 0));
            }
            public char[]name_GET(Inside ph, char[] dst_ch, int pos) //Name of the debug variable
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int name_LEN(Inside ph)
            {
                return (ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class STATUSTEXT : GroundControl.STATUSTEXT
        {
            public MAV_SEVERITY severity //Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
            {
                get {  return (MAV_SEVERITY)(0 +  BitUtils.get_bits(data, 0, 4));}
            }
            public string text_TRY(Inside ph)//Status text message, without null termination character
            {
                if(ph.field_bit !=  4 && !try_visit_field(ph, 4)  ||  !try_visit_item(ph, 0)) return null;
                return new string(text_GET(ph, new char[ph.items], 0));
            }
            public char[]text_GET(Inside ph, char[] dst_ch, int pos) //Status text message, without null termination character
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int text_LEN(Inside ph)
            {
                return (ph.field_bit !=  4 && !try_visit_field(ph, 4)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class DEBUG : GroundControl.DEBUG
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte ind //index of debug variable
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public float value //DEBUG value
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  5, 4)));}
            }
        }
        new class SETUP_SIGNING : GroundControl.SETUP_SIGNING
        {
            public ulong initial_timestamp //initial timestamp
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public byte target_system //system id of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte target_component //component ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public byte[] secret_key //signing key
            {
                get {return secret_key_GET(new byte[32], 0);}
            }
            public byte[]secret_key_GET(byte[] dst_ch, int pos)  //signing key
            {
                for(int BYTE = 10, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class BUTTON_CHANGE : GroundControl.BUTTON_CHANGE
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public uint last_change_ms //Time of last change of button state
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
            }

            public byte state //Bitmap state of buttons
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }
        }
        new class PLAY_TUNE : GroundControl.PLAY_TUNE
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }
            public string tune_TRY(Inside ph)//tune in board specific format
            {
                if(ph.field_bit !=  16 && !try_visit_field(ph, 16)  ||  !try_visit_item(ph, 0)) return null;
                return new string(tune_GET(ph, new char[ph.items], 0));
            }
            public char[]tune_GET(Inside ph, char[] dst_ch, int pos) //tune in board specific format
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int tune_LEN(Inside ph)
            {
                return (ph.field_bit !=  16 && !try_visit_field(ph, 16)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class CAMERA_INFORMATION : GroundControl.CAMERA_INFORMATION
        {
            public ushort resolution_h //Image resolution in pixels horizontal
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort resolution_v //Image resolution in pixels vertical
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort cam_definition_version //Camera definition version (iteration)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
            }

            public uint firmware_version //0xff = Major)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  10, 4));}
            }

            public byte[] vendor_name //Name of the camera vendor
            {
                get {return vendor_name_GET(new byte[32], 0);}
            }
            public byte[]vendor_name_GET(byte[] dst_ch, int pos)  //Name of the camera vendor
            {
                for(int BYTE = 14, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public byte[] model_name //Name of the camera model
            {
                get {return model_name_GET(new byte[32], 0);}
            }
            public byte[]model_name_GET(byte[] dst_ch, int pos)  //Name of the camera model
            {
                for(int BYTE = 46, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public float focal_length //Focal length in mm
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  78, 4)));}
            }

            public float sensor_size_h //Image sensor size horizontal in mm
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  82, 4)));}
            }

            public float sensor_size_v //Image sensor size vertical in mm
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  86, 4)));}
            }

            public byte lens_id //Reserved for a lens ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  90, 1));}
            }

            public CAMERA_CAP_FLAGS flags //CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities.
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 728, 3))
                    {
                        case 0:
                            return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO;
                        case 1:
                            return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE;
                        case 2:
                            return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES;
                        case 3:
                            return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE;
                        case 4:
                            return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE;
                        case 5:
                            return CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }
            public string cam_definition_uri_TRY(Inside ph)//Camera definition URI (if any, otherwise only basic functions will be available).
            {
                if(ph.field_bit !=  731 && !try_visit_field(ph, 731)  ||  !try_visit_item(ph, 0)) return null;
                return new string(cam_definition_uri_GET(ph, new char[ph.items], 0));
            }
            public char[]cam_definition_uri_GET(Inside ph, char[] dst_ch, int pos) //Camera definition URI (if any, otherwise only basic functions will be available).
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int cam_definition_uri_LEN(Inside ph)
            {
                return (ph.field_bit !=  731 && !try_visit_field(ph, 731)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class CAMERA_SETTINGS : GroundControl.CAMERA_SETTINGS
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public CAMERA_MODE mode_id //Camera mode (CAMERA_MODE)
            {
                get {  return (CAMERA_MODE)(0 +  BitUtils.get_bits(data, 32, 2));}
            }
        }
        new class STORAGE_INFORMATION : GroundControl.STORAGE_INFORMATION
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte storage_id //Storage ID (1 for first, 2 for second, etc.)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte storage_count //Number of storage devices
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte status //Status of storage (0 not available, 1 unformatted, 2 formatted)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public float total_capacity //Total capacity in MiB
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  7, 4)));}
            }

            public float used_capacity //Used capacity in MiB
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  11, 4)));}
            }

            public float available_capacity //Available capacity in MiB
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  15, 4)));}
            }

            public float read_speed //Read speed in MiB/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  19, 4)));}
            }

            public float write_speed //Write speed in MiB/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  23, 4)));}
            }
        }
        new class CAMERA_CAPTURE_STATUS : GroundControl.CAMERA_CAPTURE_STATUS
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public uint recording_time_ms //Time in milliseconds since recording started
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
            }

            /**
            *Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval
            *	set and capture in progress*/
            public byte image_status
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte video_status //Current status of video capturing (0: idle, 1: capture in progress)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public float image_interval //Image capture interval in seconds
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }

            public float available_capacity //Available storage capacity in MiB
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }
        }
        new class CAMERA_IMAGE_CAPTURED : GroundControl.CAMERA_IMAGE_CAPTURED
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public ulong time_utc //Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
            {
                get {  return (BitUtils.get_bytes(data,  4, 8));}
            }

            public byte camera_id //Camera ID (1 for first, 2 for second, etc.)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  12, 1));}
            }

            public int lat //Latitude, expressed as degrees * 1E7 where image was taken
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  13, 4));}
            }

            public int lon //Longitude, expressed as degrees * 1E7 where capture was taken
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  17, 4));}
            }

            public int alt //Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  21, 4));}
            }

            public int relative_alt //Altitude above ground in meters, expressed as * 1E3 where image was taken
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  25, 4));}
            }

            public float[] q //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
            {
                get {return q_GET(new float[4], 0);}
            }
            public float[]q_GET(float[] dst_ch, int pos)  //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
            {
                for(int BYTE = 29, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public int image_index //Zero based index of this image (image count since armed -1)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  45, 4));}
            }

            public sbyte capture_result //Boolean indicating success (1) or failure (0) while capturing this image.
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  49, 1));}
            }
            public string file_url_TRY(Inside ph)//URL of image taken. Either local storage or http:foo.jpg if camera provides an HTTP interface.
            {
                if(ph.field_bit !=  402 && !try_visit_field(ph, 402)  ||  !try_visit_item(ph, 0)) return null;
                return new string(file_url_GET(ph, new char[ph.items], 0));
            }
            public char[]file_url_GET(Inside ph, char[] dst_ch, int pos) //URL of image taken. Either local storage or http:foo.jpg if camera provides an HTTP interface.
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int file_url_LEN(Inside ph)
            {
                return (ph.field_bit !=  402 && !try_visit_field(ph, 402)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class FLIGHT_INFORMATION : GroundControl.FLIGHT_INFORMATION
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public ulong arming_time_utc //Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
            {
                get {  return (BitUtils.get_bytes(data,  4, 8));}
            }

            public ulong takeoff_time_utc //Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
            {
                get {  return (BitUtils.get_bytes(data,  12, 8));}
            }

            public ulong flight_uuid //Universally unique identifier (UUID) of flight, should correspond to name of logfiles
            {
                get {  return (BitUtils.get_bytes(data,  20, 8));}
            }
        }
        new class MOUNT_ORIENTATION : GroundControl.MOUNT_ORIENTATION
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public float roll //Roll in degrees
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float pitch //Pitch in degrees
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float yaw //Yaw in degrees
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }
        }
        new class LOGGING_DATA : GroundControl.LOGGING_DATA
        {
            public ushort sequence //sequence number (can wrap)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target_system //system ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte target_component //component ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte length //data length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            /**
            *offset into data where first message starts. This can be used for recovery, when a previous message got
            *	lost (set to 255 if no start exists)*/
            public byte first_message_offset
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte[] data_ //logged data
            {
                get {return data__GET(new byte[249], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //logged data
            {
                for(int BYTE = 6, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class LOGGING_DATA_ACKED : GroundControl.LOGGING_DATA_ACKED
        {
            public ushort sequence //sequence number (can wrap)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target_system //system ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte target_component //component ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte length //data length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            /**
            *offset into data where first message starts. This can be used for recovery, when a previous message got
            *	lost (set to 255 if no start exists)*/
            public byte first_message_offset
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte[] data_ //logged data
            {
                get {return data__GET(new byte[249], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //logged data
            {
                for(int BYTE = 6, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class LOGGING_ACK : GroundControl.LOGGING_ACK
        {
            public ushort sequence //sequence number (must match the one in LOGGING_DATA_ACKED)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target_system //system ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte target_component //component ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }
        }
        new class VIDEO_STREAM_INFORMATION : GroundControl.VIDEO_STREAM_INFORMATION
        {
            public ushort resolution_h //Resolution horizontal in pixels
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort resolution_v //Resolution vertical in pixels
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort rotation //Video image rotation clockwise
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public uint bitrate //Bit rate in bits per second
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
            }

            public byte camera_id //Camera ID (1 for first, 2 for second, etc.)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
            }

            public byte status //Current status of video streaming (0: not running, 1: in progress)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
            }

            public float framerate //Frames per second
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }
            public string uri_TRY(Inside ph)//Video stream URI
            {
                if(ph.field_bit !=  130 && !try_visit_field(ph, 130)  ||  !try_visit_item(ph, 0)) return null;
                return new string(uri_GET(ph, new char[ph.items], 0));
            }
            public char[]uri_GET(Inside ph, char[] dst_ch, int pos) //Video stream URI
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int uri_LEN(Inside ph)
            {
                return (ph.field_bit !=  130 && !try_visit_field(ph, 130)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class SET_VIDEO_STREAM_SETTINGS : GroundControl.SET_VIDEO_STREAM_SETTINGS
        {
            public ushort resolution_h //Resolution horizontal in pixels (set to -1 for highest resolution possible)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort resolution_v //Resolution vertical in pixels (set to -1 for highest resolution possible)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort rotation //Video image rotation clockwise (0-359 degrees)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public uint bitrate //Bit rate in bits per second (set to -1 for auto)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
            }

            public byte target_system //system ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
            }

            public byte target_component //component ID of the target
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
            }

            public byte camera_id //Camera ID (1 for first, 2 for second, etc.)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  12, 1));}
            }

            public float framerate //Frames per second (set to -1 for highest framerate possible)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  13, 4)));}
            }
            public string uri_TRY(Inside ph)//Video stream URI
            {
                if(ph.field_bit !=  138 && !try_visit_field(ph, 138)  ||  !try_visit_item(ph, 0)) return null;
                return new string(uri_GET(ph, new char[ph.items], 0));
            }
            public char[]uri_GET(Inside ph, char[] dst_ch, int pos) //Video stream URI
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int uri_LEN(Inside ph)
            {
                return (ph.field_bit !=  138 && !try_visit_field(ph, 138)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class WIFI_CONFIG_AP : GroundControl.WIFI_CONFIG_AP
        {
            public string ssid_TRY(Inside ph)//Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
            {
                if(ph.field_bit !=  2 && !try_visit_field(ph, 2)  ||  !try_visit_item(ph, 0)) return null;
                return new string(ssid_GET(ph, new char[ph.items], 0));
            }
            public char[]ssid_GET(Inside ph, char[] dst_ch, int pos) //Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int ssid_LEN(Inside ph)
            {
                return (ph.field_bit !=  2 && !try_visit_field(ph, 2)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public string password_TRY(Inside ph)//Password. Leave it blank for an open AP.
            {
                if(ph.field_bit !=  3 && !try_visit_field(ph, 3)  ||  !try_visit_item(ph, 0)) return null;
                return new string(password_GET(ph, new char[ph.items], 0));
            }
            public char[]password_GET(Inside ph, char[] dst_ch, int pos) //Password. Leave it blank for an open AP.
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int password_LEN(Inside ph)
            {
                return (ph.field_bit !=  3 && !try_visit_field(ph, 3)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class PROTOCOL_VERSION : GroundControl.PROTOCOL_VERSION
        {
            public ushort version //Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort min_version //Minimum MAVLink version supported
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort max_version //Maximum MAVLink version supported (set to the same value as version by default)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public byte[] spec_version_hash //The first 8 bytes (not characters printed in hex!) of the git hash.
            {
                get {return spec_version_hash_GET(new byte[8], 0);}
            }
            public byte[]spec_version_hash_GET(byte[] dst_ch, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash.
            {
                for(int BYTE = 6, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public byte[] library_version_hash //The first 8 bytes (not characters printed in hex!) of the git hash.
            {
                get {return library_version_hash_GET(new byte[8], 0);}
            }
            public byte[]library_version_hash_GET(byte[] dst_ch, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash.
            {
                for(int BYTE = 14, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class UAVCAN_NODE_STATUS : GroundControl.UAVCAN_NODE_STATUS
        {
            public ushort vendor_specific_status_code //Vendor-specific status information.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public uint uptime_sec //The number of seconds since the start-up of the node.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  6, 8));}
            }

            public byte sub_mode //Not used currently.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  14, 1));}
            }

            public UAVCAN_NODE_HEALTH health //Generalized node health status.
            {
                get {  return (UAVCAN_NODE_HEALTH)(0 +  BitUtils.get_bits(data, 120, 3));}
            }

            public UAVCAN_NODE_MODE mode //Generalized operating mode.
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 123, 3))
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
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }
        }
        new class UAVCAN_NODE_INFO : GroundControl.UAVCAN_NODE_INFO
        {
            public uint uptime_sec //The number of seconds since the start-up of the node.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public uint sw_vcs_commit //Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
            }

            public byte hw_version_major //Hardware major version number.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
            }

            public byte hw_version_minor //Hardware minor version number.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  17, 1));}
            }

            public byte[] hw_unique_id //Hardware unique 128-bit ID.
            {
                get {return hw_unique_id_GET(new byte[16], 0);}
            }
            public byte[]hw_unique_id_GET(byte[] dst_ch, int pos)  //Hardware unique 128-bit ID.
            {
                for(int BYTE = 18, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public byte sw_version_major //Software major version number.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  34, 1));}
            }

            public byte sw_version_minor //Software minor version number.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  35, 1));}
            }
            public string name_TRY(Inside ph)//Node name string. For example, "sapog.px4.io".
            {
                if(ph.field_bit !=  288 && !try_visit_field(ph, 288)  ||  !try_visit_item(ph, 0)) return null;
                return new string(name_GET(ph, new char[ph.items], 0));
            }
            public char[]name_GET(Inside ph, char[] dst_ch, int pos) //Node name string. For example, "sapog.px4.io".
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int name_LEN(Inside ph)
            {
                return (ph.field_bit !=  288 && !try_visit_field(ph, 288)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class PARAM_EXT_REQUEST_READ : GroundControl.PARAM_EXT_REQUEST_READ
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public short param_index //Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	ID is stored as strin*/
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	ID is stored as strin*/
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class PARAM_EXT_REQUEST_LIST : GroundControl.PARAM_EXT_REQUEST_LIST
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }
        }
        new class PARAM_EXT_VALUE : GroundControl.PARAM_EXT_VALUE
        {
            public ushort param_count //Total number of parameters
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort param_index //Index of this parameter
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public MAV_PARAM_EXT_TYPE param_type //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
            {
                get {  return (MAV_PARAM_EXT_TYPE)(1 +  BitUtils.get_bits(data, 32, 4));}
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	ID is stored as strin*/
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  38 && !try_visit_field(ph, 38)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	ID is stored as strin*/
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  38 && !try_visit_field(ph, 38)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public string param_value_TRY(Inside ph)//Parameter value
            {
                if(ph.field_bit !=  39 && !try_visit_field(ph, 39)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_value_GET(ph, new char[ph.items], 0));
            }
            public char[]param_value_GET(Inside ph, char[] dst_ch, int pos) //Parameter value
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_value_LEN(Inside ph)
            {
                return (ph.field_bit !=  39 && !try_visit_field(ph, 39)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class PARAM_EXT_SET : GroundControl.PARAM_EXT_SET
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public MAV_PARAM_EXT_TYPE param_type //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
            {
                get {  return (MAV_PARAM_EXT_TYPE)(1 +  BitUtils.get_bits(data, 16, 4));}
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	ID is stored as strin*/
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  22 && !try_visit_field(ph, 22)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	ID is stored as strin*/
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  22 && !try_visit_field(ph, 22)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public string param_value_TRY(Inside ph)//Parameter value
            {
                if(ph.field_bit !=  23 && !try_visit_field(ph, 23)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_value_GET(ph, new char[ph.items], 0));
            }
            public char[]param_value_GET(Inside ph, char[] dst_ch, int pos) //Parameter value
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_value_LEN(Inside ph)
            {
                return (ph.field_bit !=  23 && !try_visit_field(ph, 23)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class PARAM_EXT_ACK : GroundControl.PARAM_EXT_ACK
        {
            public MAV_PARAM_EXT_TYPE param_type //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
            {
                get {  return (MAV_PARAM_EXT_TYPE)(1 +  BitUtils.get_bits(data, 0, 4));}
            }

            public PARAM_ACK param_result //Result code: see the PARAM_ACK enum for possible codes.
            {
                get {  return (PARAM_ACK)(0 +  BitUtils.get_bits(data, 4, 3));}
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	ID is stored as strin*/
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  9 && !try_visit_field(ph, 9)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            /**
            *Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
            *	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
            *	ID is stored as strin*/
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  9 && !try_visit_field(ph, 9)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public string param_value_TRY(Inside ph)//Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
            {
                if(ph.field_bit !=  10 && !try_visit_field(ph, 10)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_value_GET(ph, new char[ph.items], 0));
            }
            public char[]param_value_GET(Inside ph, char[] dst_ch, int pos) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_value_LEN(Inside ph)
            {
                return (ph.field_bit !=  10 && !try_visit_field(ph, 10)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class OBSTACLE_DISTANCE : GroundControl.OBSTACLE_DISTANCE
        {
            /**
            *Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
            *	is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
            *	for unknown/not used. In a array element, each unit corresponds to 1cm*/
            public ushort[] distances
            {
                get {return distances_GET(new ushort[72], 0);}
            }
            /**
            *Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
            *	is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
            *	for unknown/not used. In a array element, each unit corresponds to 1cm*/
            public ushort[]distances_GET(ushort[] dst_ch, int pos)
            {
                for(int BYTE = 0, dst_max = pos + 72; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort min_distance //Minimum distance the sensor can measure in centimeters
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  144, 2));}
            }

            public ushort max_distance //Maximum distance the sensor can measure in centimeters
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  146, 2));}
            }

            public ulong time_usec //Timestamp (microseconds since system boot or since UNIX epoch)
            {
                get {  return (BitUtils.get_bytes(data,  148, 8));}
            }

            public byte increment //Angular width in degrees of each array element.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  156, 1));}
            }

            public MAV_DISTANCE_SENSOR sensor_type //Class id of the distance sensor type.
            {
                get {  return (MAV_DISTANCE_SENSOR)(0 +  BitUtils.get_bits(data, 1256, 3));}
            }
        }



        class TestChannelAdvanced : Channel.Advanced
        {
            public override bool CanRead { get { return true; } }
            public override bool CanWrite { get { return true; } }

            public override void failure(string reason)
            {
                base.failure(reason);
                Debug.Assert(false);
            }

            public void OnBATTERY_STATUSReceive_direct(Channel src, Inside ph, BATTERY_STATUS pack) {OnBATTERY_STATUSReceive(this, ph,  pack);}
            public event BATTERY_STATUSReceiveHandler OnBATTERY_STATUSReceive;
            public delegate void BATTERY_STATUSReceiveHandler(Channel src, Inside ph, BATTERY_STATUS pack);
            public void OnAUTOPILOT_VERSIONReceive_direct(Channel src, Inside ph, AUTOPILOT_VERSION pack) {OnAUTOPILOT_VERSIONReceive(this, ph,  pack);}
            public event AUTOPILOT_VERSIONReceiveHandler OnAUTOPILOT_VERSIONReceive;
            public delegate void AUTOPILOT_VERSIONReceiveHandler(Channel src, Inside ph, AUTOPILOT_VERSION pack);
            public void OnLANDING_TARGETReceive_direct(Channel src, Inside ph, LANDING_TARGET pack) {OnLANDING_TARGETReceive(this, ph,  pack);}
            public event LANDING_TARGETReceiveHandler OnLANDING_TARGETReceive;
            public delegate void LANDING_TARGETReceiveHandler(Channel src, Inside ph, LANDING_TARGET pack);
            public void OnSENS_POWERReceive_direct(Channel src, Inside ph, SENS_POWER pack) {OnSENS_POWERReceive(this, ph,  pack);}
            public event SENS_POWERReceiveHandler OnSENS_POWERReceive;
            public delegate void SENS_POWERReceiveHandler(Channel src, Inside ph, SENS_POWER pack);
            public void OnSENS_MPPTReceive_direct(Channel src, Inside ph, SENS_MPPT pack) {OnSENS_MPPTReceive(this, ph,  pack);}
            public event SENS_MPPTReceiveHandler OnSENS_MPPTReceive;
            public delegate void SENS_MPPTReceiveHandler(Channel src, Inside ph, SENS_MPPT pack);
            public void OnASLCTRL_DATAReceive_direct(Channel src, Inside ph, ASLCTRL_DATA pack) {OnASLCTRL_DATAReceive(this, ph,  pack);}
            public event ASLCTRL_DATAReceiveHandler OnASLCTRL_DATAReceive;
            public delegate void ASLCTRL_DATAReceiveHandler(Channel src, Inside ph, ASLCTRL_DATA pack);
            public void OnASLCTRL_DEBUGReceive_direct(Channel src, Inside ph, ASLCTRL_DEBUG pack) {OnASLCTRL_DEBUGReceive(this, ph,  pack);}
            public event ASLCTRL_DEBUGReceiveHandler OnASLCTRL_DEBUGReceive;
            public delegate void ASLCTRL_DEBUGReceiveHandler(Channel src, Inside ph, ASLCTRL_DEBUG pack);
            public void OnASLUAV_STATUSReceive_direct(Channel src, Inside ph, ASLUAV_STATUS pack) {OnASLUAV_STATUSReceive(this, ph,  pack);}
            public event ASLUAV_STATUSReceiveHandler OnASLUAV_STATUSReceive;
            public delegate void ASLUAV_STATUSReceiveHandler(Channel src, Inside ph, ASLUAV_STATUS pack);
            public void OnEKF_EXTReceive_direct(Channel src, Inside ph, EKF_EXT pack) {OnEKF_EXTReceive(this, ph,  pack);}
            public event EKF_EXTReceiveHandler OnEKF_EXTReceive;
            public delegate void EKF_EXTReceiveHandler(Channel src, Inside ph, EKF_EXT pack);
            public void OnASL_OBCTRLReceive_direct(Channel src, Inside ph, ASL_OBCTRL pack) {OnASL_OBCTRLReceive(this, ph,  pack);}
            public event ASL_OBCTRLReceiveHandler OnASL_OBCTRLReceive;
            public delegate void ASL_OBCTRLReceiveHandler(Channel src, Inside ph, ASL_OBCTRL pack);
            public void OnSENS_ATMOSReceive_direct(Channel src, Inside ph, SENS_ATMOS pack) {OnSENS_ATMOSReceive(this, ph,  pack);}
            public event SENS_ATMOSReceiveHandler OnSENS_ATMOSReceive;
            public delegate void SENS_ATMOSReceiveHandler(Channel src, Inside ph, SENS_ATMOS pack);
            public void OnSENS_BATMONReceive_direct(Channel src, Inside ph, SENS_BATMON pack) {OnSENS_BATMONReceive(this, ph,  pack);}
            public event SENS_BATMONReceiveHandler OnSENS_BATMONReceive;
            public delegate void SENS_BATMONReceiveHandler(Channel src, Inside ph, SENS_BATMON pack);
            public void OnFW_SOARING_DATAReceive_direct(Channel src, Inside ph, FW_SOARING_DATA pack) {OnFW_SOARING_DATAReceive(this, ph,  pack);}
            public event FW_SOARING_DATAReceiveHandler OnFW_SOARING_DATAReceive;
            public delegate void FW_SOARING_DATAReceiveHandler(Channel src, Inside ph, FW_SOARING_DATA pack);
            public void OnSENSORPOD_STATUSReceive_direct(Channel src, Inside ph, SENSORPOD_STATUS pack) {OnSENSORPOD_STATUSReceive(this, ph,  pack);}
            public event SENSORPOD_STATUSReceiveHandler OnSENSORPOD_STATUSReceive;
            public delegate void SENSORPOD_STATUSReceiveHandler(Channel src, Inside ph, SENSORPOD_STATUS pack);
            public void OnSENS_POWER_BOARDReceive_direct(Channel src, Inside ph, SENS_POWER_BOARD pack) {OnSENS_POWER_BOARDReceive(this, ph,  pack);}
            public event SENS_POWER_BOARDReceiveHandler OnSENS_POWER_BOARDReceive;
            public delegate void SENS_POWER_BOARDReceiveHandler(Channel src, Inside ph, SENS_POWER_BOARD pack);
            public void OnESTIMATOR_STATUSReceive_direct(Channel src, Inside ph, ESTIMATOR_STATUS pack) {OnESTIMATOR_STATUSReceive(this, ph,  pack);}
            public event ESTIMATOR_STATUSReceiveHandler OnESTIMATOR_STATUSReceive;
            public delegate void ESTIMATOR_STATUSReceiveHandler(Channel src, Inside ph, ESTIMATOR_STATUS pack);
            public void OnWIND_COVReceive_direct(Channel src, Inside ph, WIND_COV pack) {OnWIND_COVReceive(this, ph,  pack);}
            public event WIND_COVReceiveHandler OnWIND_COVReceive;
            public delegate void WIND_COVReceiveHandler(Channel src, Inside ph, WIND_COV pack);
            public void OnGPS_INPUTReceive_direct(Channel src, Inside ph, GPS_INPUT pack) {OnGPS_INPUTReceive(this, ph,  pack);}
            public event GPS_INPUTReceiveHandler OnGPS_INPUTReceive;
            public delegate void GPS_INPUTReceiveHandler(Channel src, Inside ph, GPS_INPUT pack);
            public void OnGPS_RTCM_DATAReceive_direct(Channel src, Inside ph, GPS_RTCM_DATA pack) {OnGPS_RTCM_DATAReceive(this, ph,  pack);}
            public event GPS_RTCM_DATAReceiveHandler OnGPS_RTCM_DATAReceive;
            public delegate void GPS_RTCM_DATAReceiveHandler(Channel src, Inside ph, GPS_RTCM_DATA pack);
            public void OnHIGH_LATENCYReceive_direct(Channel src, Inside ph, HIGH_LATENCY pack) {OnHIGH_LATENCYReceive(this, ph,  pack);}
            public event HIGH_LATENCYReceiveHandler OnHIGH_LATENCYReceive;
            public delegate void HIGH_LATENCYReceiveHandler(Channel src, Inside ph, HIGH_LATENCY pack);
            public void OnVIBRATIONReceive_direct(Channel src, Inside ph, VIBRATION pack) {OnVIBRATIONReceive(this, ph,  pack);}
            public event VIBRATIONReceiveHandler OnVIBRATIONReceive;
            public delegate void VIBRATIONReceiveHandler(Channel src, Inside ph, VIBRATION pack);
            public void OnHOME_POSITIONReceive_direct(Channel src, Inside ph, HOME_POSITION pack) {OnHOME_POSITIONReceive(this, ph,  pack);}
            public event HOME_POSITIONReceiveHandler OnHOME_POSITIONReceive;
            public delegate void HOME_POSITIONReceiveHandler(Channel src, Inside ph, HOME_POSITION pack);
            public void OnSET_HOME_POSITIONReceive_direct(Channel src, Inside ph, SET_HOME_POSITION pack) {OnSET_HOME_POSITIONReceive(this, ph,  pack);}
            public event SET_HOME_POSITIONReceiveHandler OnSET_HOME_POSITIONReceive;
            public delegate void SET_HOME_POSITIONReceiveHandler(Channel src, Inside ph, SET_HOME_POSITION pack);
            public void OnMESSAGE_INTERVALReceive_direct(Channel src, Inside ph, MESSAGE_INTERVAL pack) {OnMESSAGE_INTERVALReceive(this, ph,  pack);}
            public event MESSAGE_INTERVALReceiveHandler OnMESSAGE_INTERVALReceive;
            public delegate void MESSAGE_INTERVALReceiveHandler(Channel src, Inside ph, MESSAGE_INTERVAL pack);
            public void OnEXTENDED_SYS_STATEReceive_direct(Channel src, Inside ph, EXTENDED_SYS_STATE pack) {OnEXTENDED_SYS_STATEReceive(this, ph,  pack);}
            public event EXTENDED_SYS_STATEReceiveHandler OnEXTENDED_SYS_STATEReceive;
            public delegate void EXTENDED_SYS_STATEReceiveHandler(Channel src, Inside ph, EXTENDED_SYS_STATE pack);
            public void OnADSB_VEHICLEReceive_direct(Channel src, Inside ph, ADSB_VEHICLE pack) {OnADSB_VEHICLEReceive(this, ph,  pack);}
            public event ADSB_VEHICLEReceiveHandler OnADSB_VEHICLEReceive;
            public delegate void ADSB_VEHICLEReceiveHandler(Channel src, Inside ph, ADSB_VEHICLE pack);
            public void OnCOLLISIONReceive_direct(Channel src, Inside ph, COLLISION pack) {OnCOLLISIONReceive(this, ph,  pack);}
            public event COLLISIONReceiveHandler OnCOLLISIONReceive;
            public delegate void COLLISIONReceiveHandler(Channel src, Inside ph, COLLISION pack);
            public void OnV2_EXTENSIONReceive_direct(Channel src, Inside ph, V2_EXTENSION pack) {OnV2_EXTENSIONReceive(this, ph,  pack);}
            public event V2_EXTENSIONReceiveHandler OnV2_EXTENSIONReceive;
            public delegate void V2_EXTENSIONReceiveHandler(Channel src, Inside ph, V2_EXTENSION pack);
            public void OnMEMORY_VECTReceive_direct(Channel src, Inside ph, MEMORY_VECT pack) {OnMEMORY_VECTReceive(this, ph,  pack);}
            public event MEMORY_VECTReceiveHandler OnMEMORY_VECTReceive;
            public delegate void MEMORY_VECTReceiveHandler(Channel src, Inside ph, MEMORY_VECT pack);
            public void OnDEBUG_VECTReceive_direct(Channel src, Inside ph, DEBUG_VECT pack) {OnDEBUG_VECTReceive(this, ph,  pack);}
            public event DEBUG_VECTReceiveHandler OnDEBUG_VECTReceive;
            public delegate void DEBUG_VECTReceiveHandler(Channel src, Inside ph, DEBUG_VECT pack);
            public void OnNAMED_VALUE_FLOATReceive_direct(Channel src, Inside ph, NAMED_VALUE_FLOAT pack) {OnNAMED_VALUE_FLOATReceive(this, ph,  pack);}
            public event NAMED_VALUE_FLOATReceiveHandler OnNAMED_VALUE_FLOATReceive;
            public delegate void NAMED_VALUE_FLOATReceiveHandler(Channel src, Inside ph, NAMED_VALUE_FLOAT pack);
            public void OnNAMED_VALUE_INTReceive_direct(Channel src, Inside ph, NAMED_VALUE_INT pack) {OnNAMED_VALUE_INTReceive(this, ph,  pack);}
            public event NAMED_VALUE_INTReceiveHandler OnNAMED_VALUE_INTReceive;
            public delegate void NAMED_VALUE_INTReceiveHandler(Channel src, Inside ph, NAMED_VALUE_INT pack);
            public void OnSTATUSTEXTReceive_direct(Channel src, Inside ph, STATUSTEXT pack) {OnSTATUSTEXTReceive(this, ph,  pack);}
            public event STATUSTEXTReceiveHandler OnSTATUSTEXTReceive;
            public delegate void STATUSTEXTReceiveHandler(Channel src, Inside ph, STATUSTEXT pack);
            public void OnDEBUGReceive_direct(Channel src, Inside ph, DEBUG pack) {OnDEBUGReceive(this, ph,  pack);}
            public event DEBUGReceiveHandler OnDEBUGReceive;
            public delegate void DEBUGReceiveHandler(Channel src, Inside ph, DEBUG pack);
            public void OnSETUP_SIGNINGReceive_direct(Channel src, Inside ph, SETUP_SIGNING pack) {OnSETUP_SIGNINGReceive(this, ph,  pack);}
            public event SETUP_SIGNINGReceiveHandler OnSETUP_SIGNINGReceive;
            public delegate void SETUP_SIGNINGReceiveHandler(Channel src, Inside ph, SETUP_SIGNING pack);
            public void OnBUTTON_CHANGEReceive_direct(Channel src, Inside ph, BUTTON_CHANGE pack) {OnBUTTON_CHANGEReceive(this, ph,  pack);}
            public event BUTTON_CHANGEReceiveHandler OnBUTTON_CHANGEReceive;
            public delegate void BUTTON_CHANGEReceiveHandler(Channel src, Inside ph, BUTTON_CHANGE pack);
            public void OnPLAY_TUNEReceive_direct(Channel src, Inside ph, PLAY_TUNE pack) {OnPLAY_TUNEReceive(this, ph,  pack);}
            public event PLAY_TUNEReceiveHandler OnPLAY_TUNEReceive;
            public delegate void PLAY_TUNEReceiveHandler(Channel src, Inside ph, PLAY_TUNE pack);
            public void OnCAMERA_INFORMATIONReceive_direct(Channel src, Inside ph, CAMERA_INFORMATION pack) {OnCAMERA_INFORMATIONReceive(this, ph,  pack);}
            public event CAMERA_INFORMATIONReceiveHandler OnCAMERA_INFORMATIONReceive;
            public delegate void CAMERA_INFORMATIONReceiveHandler(Channel src, Inside ph, CAMERA_INFORMATION pack);
            public void OnCAMERA_SETTINGSReceive_direct(Channel src, Inside ph, CAMERA_SETTINGS pack) {OnCAMERA_SETTINGSReceive(this, ph,  pack);}
            public event CAMERA_SETTINGSReceiveHandler OnCAMERA_SETTINGSReceive;
            public delegate void CAMERA_SETTINGSReceiveHandler(Channel src, Inside ph, CAMERA_SETTINGS pack);
            public void OnSTORAGE_INFORMATIONReceive_direct(Channel src, Inside ph, STORAGE_INFORMATION pack) {OnSTORAGE_INFORMATIONReceive(this, ph,  pack);}
            public event STORAGE_INFORMATIONReceiveHandler OnSTORAGE_INFORMATIONReceive;
            public delegate void STORAGE_INFORMATIONReceiveHandler(Channel src, Inside ph, STORAGE_INFORMATION pack);
            public void OnCAMERA_CAPTURE_STATUSReceive_direct(Channel src, Inside ph, CAMERA_CAPTURE_STATUS pack) {OnCAMERA_CAPTURE_STATUSReceive(this, ph,  pack);}
            public event CAMERA_CAPTURE_STATUSReceiveHandler OnCAMERA_CAPTURE_STATUSReceive;
            public delegate void CAMERA_CAPTURE_STATUSReceiveHandler(Channel src, Inside ph, CAMERA_CAPTURE_STATUS pack);
            public void OnCAMERA_IMAGE_CAPTUREDReceive_direct(Channel src, Inside ph, CAMERA_IMAGE_CAPTURED pack) {OnCAMERA_IMAGE_CAPTUREDReceive(this, ph,  pack);}
            public event CAMERA_IMAGE_CAPTUREDReceiveHandler OnCAMERA_IMAGE_CAPTUREDReceive;
            public delegate void CAMERA_IMAGE_CAPTUREDReceiveHandler(Channel src, Inside ph, CAMERA_IMAGE_CAPTURED pack);
            public void OnFLIGHT_INFORMATIONReceive_direct(Channel src, Inside ph, FLIGHT_INFORMATION pack) {OnFLIGHT_INFORMATIONReceive(this, ph,  pack);}
            public event FLIGHT_INFORMATIONReceiveHandler OnFLIGHT_INFORMATIONReceive;
            public delegate void FLIGHT_INFORMATIONReceiveHandler(Channel src, Inside ph, FLIGHT_INFORMATION pack);
            public void OnMOUNT_ORIENTATIONReceive_direct(Channel src, Inside ph, MOUNT_ORIENTATION pack) {OnMOUNT_ORIENTATIONReceive(this, ph,  pack);}
            public event MOUNT_ORIENTATIONReceiveHandler OnMOUNT_ORIENTATIONReceive;
            public delegate void MOUNT_ORIENTATIONReceiveHandler(Channel src, Inside ph, MOUNT_ORIENTATION pack);
            public void OnLOGGING_DATAReceive_direct(Channel src, Inside ph, LOGGING_DATA pack) {OnLOGGING_DATAReceive(this, ph,  pack);}
            public event LOGGING_DATAReceiveHandler OnLOGGING_DATAReceive;
            public delegate void LOGGING_DATAReceiveHandler(Channel src, Inside ph, LOGGING_DATA pack);
            public void OnLOGGING_DATA_ACKEDReceive_direct(Channel src, Inside ph, LOGGING_DATA_ACKED pack) {OnLOGGING_DATA_ACKEDReceive(this, ph,  pack);}
            public event LOGGING_DATA_ACKEDReceiveHandler OnLOGGING_DATA_ACKEDReceive;
            public delegate void LOGGING_DATA_ACKEDReceiveHandler(Channel src, Inside ph, LOGGING_DATA_ACKED pack);
            public void OnLOGGING_ACKReceive_direct(Channel src, Inside ph, LOGGING_ACK pack) {OnLOGGING_ACKReceive(this, ph,  pack);}
            public event LOGGING_ACKReceiveHandler OnLOGGING_ACKReceive;
            public delegate void LOGGING_ACKReceiveHandler(Channel src, Inside ph, LOGGING_ACK pack);
            public void OnVIDEO_STREAM_INFORMATIONReceive_direct(Channel src, Inside ph, VIDEO_STREAM_INFORMATION pack) {OnVIDEO_STREAM_INFORMATIONReceive(this, ph,  pack);}
            public event VIDEO_STREAM_INFORMATIONReceiveHandler OnVIDEO_STREAM_INFORMATIONReceive;
            public delegate void VIDEO_STREAM_INFORMATIONReceiveHandler(Channel src, Inside ph, VIDEO_STREAM_INFORMATION pack);
            public void OnSET_VIDEO_STREAM_SETTINGSReceive_direct(Channel src, Inside ph, SET_VIDEO_STREAM_SETTINGS pack) {OnSET_VIDEO_STREAM_SETTINGSReceive(this, ph,  pack);}
            public event SET_VIDEO_STREAM_SETTINGSReceiveHandler OnSET_VIDEO_STREAM_SETTINGSReceive;
            public delegate void SET_VIDEO_STREAM_SETTINGSReceiveHandler(Channel src, Inside ph, SET_VIDEO_STREAM_SETTINGS pack);
            public void OnWIFI_CONFIG_APReceive_direct(Channel src, Inside ph, WIFI_CONFIG_AP pack) {OnWIFI_CONFIG_APReceive(this, ph,  pack);}
            public event WIFI_CONFIG_APReceiveHandler OnWIFI_CONFIG_APReceive;
            public delegate void WIFI_CONFIG_APReceiveHandler(Channel src, Inside ph, WIFI_CONFIG_AP pack);
            public void OnPROTOCOL_VERSIONReceive_direct(Channel src, Inside ph, PROTOCOL_VERSION pack) {OnPROTOCOL_VERSIONReceive(this, ph,  pack);}
            public event PROTOCOL_VERSIONReceiveHandler OnPROTOCOL_VERSIONReceive;
            public delegate void PROTOCOL_VERSIONReceiveHandler(Channel src, Inside ph, PROTOCOL_VERSION pack);
            public void OnUAVCAN_NODE_STATUSReceive_direct(Channel src, Inside ph, UAVCAN_NODE_STATUS pack) {OnUAVCAN_NODE_STATUSReceive(this, ph,  pack);}
            public event UAVCAN_NODE_STATUSReceiveHandler OnUAVCAN_NODE_STATUSReceive;
            public delegate void UAVCAN_NODE_STATUSReceiveHandler(Channel src, Inside ph, UAVCAN_NODE_STATUS pack);
            public void OnUAVCAN_NODE_INFOReceive_direct(Channel src, Inside ph, UAVCAN_NODE_INFO pack) {OnUAVCAN_NODE_INFOReceive(this, ph,  pack);}
            public event UAVCAN_NODE_INFOReceiveHandler OnUAVCAN_NODE_INFOReceive;
            public delegate void UAVCAN_NODE_INFOReceiveHandler(Channel src, Inside ph, UAVCAN_NODE_INFO pack);
            public void OnPARAM_EXT_REQUEST_READReceive_direct(Channel src, Inside ph, PARAM_EXT_REQUEST_READ pack) {OnPARAM_EXT_REQUEST_READReceive(this, ph,  pack);}
            public event PARAM_EXT_REQUEST_READReceiveHandler OnPARAM_EXT_REQUEST_READReceive;
            public delegate void PARAM_EXT_REQUEST_READReceiveHandler(Channel src, Inside ph, PARAM_EXT_REQUEST_READ pack);
            public void OnPARAM_EXT_REQUEST_LISTReceive_direct(Channel src, Inside ph, PARAM_EXT_REQUEST_LIST pack) {OnPARAM_EXT_REQUEST_LISTReceive(this, ph,  pack);}
            public event PARAM_EXT_REQUEST_LISTReceiveHandler OnPARAM_EXT_REQUEST_LISTReceive;
            public delegate void PARAM_EXT_REQUEST_LISTReceiveHandler(Channel src, Inside ph, PARAM_EXT_REQUEST_LIST pack);
            public void OnPARAM_EXT_VALUEReceive_direct(Channel src, Inside ph, PARAM_EXT_VALUE pack) {OnPARAM_EXT_VALUEReceive(this, ph,  pack);}
            public event PARAM_EXT_VALUEReceiveHandler OnPARAM_EXT_VALUEReceive;
            public delegate void PARAM_EXT_VALUEReceiveHandler(Channel src, Inside ph, PARAM_EXT_VALUE pack);
            public void OnPARAM_EXT_SETReceive_direct(Channel src, Inside ph, PARAM_EXT_SET pack) {OnPARAM_EXT_SETReceive(this, ph,  pack);}
            public event PARAM_EXT_SETReceiveHandler OnPARAM_EXT_SETReceive;
            public delegate void PARAM_EXT_SETReceiveHandler(Channel src, Inside ph, PARAM_EXT_SET pack);
            public void OnPARAM_EXT_ACKReceive_direct(Channel src, Inside ph, PARAM_EXT_ACK pack) {OnPARAM_EXT_ACKReceive(this, ph,  pack);}
            public event PARAM_EXT_ACKReceiveHandler OnPARAM_EXT_ACKReceive;
            public delegate void PARAM_EXT_ACKReceiveHandler(Channel src, Inside ph, PARAM_EXT_ACK pack);
            public void OnOBSTACLE_DISTANCEReceive_direct(Channel src, Inside ph, OBSTACLE_DISTANCE pack) {OnOBSTACLE_DISTANCEReceive(this, ph,  pack);}
            public event OBSTACLE_DISTANCEReceiveHandler OnOBSTACLE_DISTANCEReceive;
            public delegate void OBSTACLE_DISTANCEReceiveHandler(Channel src, Inside ph, OBSTACLE_DISTANCE pack);

            static Pack testing_pack;
            readonly  byte[]                        buf = new byte[4024];

            internal void send(Pack pack)
            {
                testing_pack = pack;
            }

            readonly Inside ph = new Inside();

            protected internal override Pack process(Pack pack, int id)
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
                        break;
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
                    case 147:
                        if(pack == null) return new BATTERY_STATUS();
                        OnBATTERY_STATUSReceive(this, ph, (BATTERY_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 148:
                        if(pack == null) return new AUTOPILOT_VERSION();
                        OnAUTOPILOT_VERSIONReceive(this, ph, (AUTOPILOT_VERSION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 149:
                        if(pack == null) return new LANDING_TARGET();
                        OnLANDING_TARGETReceive(this, ph, (LANDING_TARGET) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 201:
                        if(pack == null) return new SENS_POWER();
                        OnSENS_POWERReceive(this, ph, (SENS_POWER) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 202:
                        if(pack == null) return new SENS_MPPT();
                        OnSENS_MPPTReceive(this, ph, (SENS_MPPT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 203:
                        if(pack == null) return new ASLCTRL_DATA();
                        OnASLCTRL_DATAReceive(this, ph, (ASLCTRL_DATA) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 204:
                        if(pack == null) return new ASLCTRL_DEBUG();
                        OnASLCTRL_DEBUGReceive(this, ph, (ASLCTRL_DEBUG) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 205:
                        if(pack == null) return new ASLUAV_STATUS();
                        OnASLUAV_STATUSReceive(this, ph, (ASLUAV_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 206:
                        if(pack == null) return new EKF_EXT();
                        OnEKF_EXTReceive(this, ph, (EKF_EXT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 207:
                        if(pack == null) return new ASL_OBCTRL();
                        OnASL_OBCTRLReceive(this, ph, (ASL_OBCTRL) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 208:
                        if(pack == null) return new SENS_ATMOS();
                        OnSENS_ATMOSReceive(this, ph, (SENS_ATMOS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 209:
                        if(pack == null) return new SENS_BATMON();
                        OnSENS_BATMONReceive(this, ph, (SENS_BATMON) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 210:
                        if(pack == null) return new FW_SOARING_DATA();
                        OnFW_SOARING_DATAReceive(this, ph, (FW_SOARING_DATA) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 211:
                        if(pack == null) return new SENSORPOD_STATUS();
                        OnSENSORPOD_STATUSReceive(this, ph, (SENSORPOD_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 212:
                        if(pack == null) return new SENS_POWER_BOARD();
                        OnSENS_POWER_BOARDReceive(this, ph, (SENS_POWER_BOARD) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 230:
                        if(pack == null) return new ESTIMATOR_STATUS();
                        OnESTIMATOR_STATUSReceive(this, ph, (ESTIMATOR_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 231:
                        if(pack == null) return new WIND_COV();
                        OnWIND_COVReceive(this, ph, (WIND_COV) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 232:
                        if(pack == null) return new GPS_INPUT();
                        OnGPS_INPUTReceive(this, ph, (GPS_INPUT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 233:
                        if(pack == null) return new GPS_RTCM_DATA();
                        OnGPS_RTCM_DATAReceive(this, ph, (GPS_RTCM_DATA) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 234:
                        if(pack == null) return new HIGH_LATENCY();
                        OnHIGH_LATENCYReceive(this, ph, (HIGH_LATENCY) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 241:
                        if(pack == null) return new VIBRATION();
                        OnVIBRATIONReceive(this, ph, (VIBRATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 242:
                        if(pack == null) return new HOME_POSITION();
                        OnHOME_POSITIONReceive(this, ph, (HOME_POSITION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 243:
                        if(pack == null) return new SET_HOME_POSITION();
                        OnSET_HOME_POSITIONReceive(this, ph, (SET_HOME_POSITION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 244:
                        if(pack == null) return new MESSAGE_INTERVAL();
                        OnMESSAGE_INTERVALReceive(this, ph, (MESSAGE_INTERVAL) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 245:
                        if(pack == null) return new EXTENDED_SYS_STATE();
                        OnEXTENDED_SYS_STATEReceive(this, ph, (EXTENDED_SYS_STATE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 246:
                        if(pack == null) return new ADSB_VEHICLE();
                        OnADSB_VEHICLEReceive(this, ph, (ADSB_VEHICLE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 247:
                        if(pack == null) return new COLLISION();
                        OnCOLLISIONReceive(this, ph, (COLLISION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 248:
                        if(pack == null) return new V2_EXTENSION();
                        OnV2_EXTENSIONReceive(this, ph, (V2_EXTENSION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 249:
                        if(pack == null) return new MEMORY_VECT();
                        OnMEMORY_VECTReceive(this, ph, (MEMORY_VECT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 250:
                        if(pack == null) return new DEBUG_VECT();
                        OnDEBUG_VECTReceive(this, ph, (DEBUG_VECT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 251:
                        if(pack == null) return new NAMED_VALUE_FLOAT();
                        OnNAMED_VALUE_FLOATReceive(this, ph, (NAMED_VALUE_FLOAT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 252:
                        if(pack == null) return new NAMED_VALUE_INT();
                        OnNAMED_VALUE_INTReceive(this, ph, (NAMED_VALUE_INT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 253:
                        if(pack == null) return new STATUSTEXT();
                        OnSTATUSTEXTReceive(this, ph, (STATUSTEXT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 254:
                        if(pack == null) return new DEBUG();
                        OnDEBUGReceive(this, ph, (DEBUG) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 256:
                        if(pack == null) return new SETUP_SIGNING();
                        OnSETUP_SIGNINGReceive(this, ph, (SETUP_SIGNING) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 257:
                        if(pack == null) return new BUTTON_CHANGE();
                        OnBUTTON_CHANGEReceive(this, ph, (BUTTON_CHANGE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 258:
                        if(pack == null) return new PLAY_TUNE();
                        OnPLAY_TUNEReceive(this, ph, (PLAY_TUNE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 259:
                        if(pack == null) return new CAMERA_INFORMATION();
                        OnCAMERA_INFORMATIONReceive(this, ph, (CAMERA_INFORMATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 260:
                        if(pack == null) return new CAMERA_SETTINGS();
                        OnCAMERA_SETTINGSReceive(this, ph, (CAMERA_SETTINGS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 261:
                        if(pack == null) return new STORAGE_INFORMATION();
                        OnSTORAGE_INFORMATIONReceive(this, ph, (STORAGE_INFORMATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 262:
                        if(pack == null) return new CAMERA_CAPTURE_STATUS();
                        OnCAMERA_CAPTURE_STATUSReceive(this, ph, (CAMERA_CAPTURE_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 263:
                        if(pack == null) return new CAMERA_IMAGE_CAPTURED();
                        OnCAMERA_IMAGE_CAPTUREDReceive(this, ph, (CAMERA_IMAGE_CAPTURED) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 264:
                        if(pack == null) return new FLIGHT_INFORMATION();
                        OnFLIGHT_INFORMATIONReceive(this, ph, (FLIGHT_INFORMATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 265:
                        if(pack == null) return new MOUNT_ORIENTATION();
                        OnMOUNT_ORIENTATIONReceive(this, ph, (MOUNT_ORIENTATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 266:
                        if(pack == null) return new LOGGING_DATA();
                        OnLOGGING_DATAReceive(this, ph, (LOGGING_DATA) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 267:
                        if(pack == null) return new LOGGING_DATA_ACKED();
                        OnLOGGING_DATA_ACKEDReceive(this, ph, (LOGGING_DATA_ACKED) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 268:
                        if(pack == null) return new LOGGING_ACK();
                        OnLOGGING_ACKReceive(this, ph, (LOGGING_ACK) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 269:
                        if(pack == null) return new VIDEO_STREAM_INFORMATION();
                        OnVIDEO_STREAM_INFORMATIONReceive(this, ph, (VIDEO_STREAM_INFORMATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 270:
                        if(pack == null) return new SET_VIDEO_STREAM_SETTINGS();
                        OnSET_VIDEO_STREAM_SETTINGSReceive(this, ph, (SET_VIDEO_STREAM_SETTINGS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 299:
                        if(pack == null) return new WIFI_CONFIG_AP();
                        OnWIFI_CONFIG_APReceive(this, ph, (WIFI_CONFIG_AP) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 300:
                        if(pack == null) return new PROTOCOL_VERSION();
                        OnPROTOCOL_VERSIONReceive(this, ph, (PROTOCOL_VERSION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 310:
                        if(pack == null) return new UAVCAN_NODE_STATUS();
                        OnUAVCAN_NODE_STATUSReceive(this, ph, (UAVCAN_NODE_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 311:
                        if(pack == null) return new UAVCAN_NODE_INFO();
                        OnUAVCAN_NODE_INFOReceive(this, ph, (UAVCAN_NODE_INFO) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 320:
                        if(pack == null) return new PARAM_EXT_REQUEST_READ();
                        OnPARAM_EXT_REQUEST_READReceive(this, ph, (PARAM_EXT_REQUEST_READ) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 321:
                        if(pack == null) return new PARAM_EXT_REQUEST_LIST();
                        OnPARAM_EXT_REQUEST_LISTReceive(this, ph, (PARAM_EXT_REQUEST_LIST) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 322:
                        if(pack == null) return new PARAM_EXT_VALUE();
                        OnPARAM_EXT_VALUEReceive(this, ph, (PARAM_EXT_VALUE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 323:
                        if(pack == null) return new PARAM_EXT_SET();
                        OnPARAM_EXT_SETReceive(this, ph, (PARAM_EXT_SET) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 324:
                        if(pack == null) return new PARAM_EXT_ACK();
                        OnPARAM_EXT_ACKReceive(this, ph, (PARAM_EXT_ACK) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 330:
                        if(pack == null) return new OBSTACLE_DISTANCE();
                        OnOBSTACLE_DISTANCEReceive(this, ph, (OBSTACLE_DISTANCE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                }
                return null;
            }
            static readonly byte[] buff = new byte[1024];
            internal static void transmission(Channel src, Channel dst)
            {
                if(src is Channel.Advanced && !(dst is Channel.Advanced))
                {
                    for(int bytes; 0 < (bytes = src.Read(buff, 0, buff.Length));) ADV_TEST_CH.Write(buff, 0, bytes);
                    for(int bytes; 0 < (bytes = SMP_TEST_CH.Read(buff, 0, buff.Length));) dst.Write(buff, 0, bytes);
                }
                else if(!(src is Channel.Advanced) && dst is Channel.Advanced)
                {
                    for(int bytes; 0 < (bytes = src.Read(buff, 0, buff.Length));) SMP_TEST_CH.Write(buff, 0, bytes);
                    for(int bytes; 0 < (bytes = ADV_TEST_CH.Read(buff, 0, buff.Length));) dst.Write(buff, 0, bytes);
                }
                else
                    for(int bytes; 0 < (bytes = src.Read(buff, 0, buff.Length));) dst.Write(buff, 0, bytes);
                dst.process(null, Channel.PROCESS_RECEIVED);
            }
        }
        static readonly TestChannelAdvanced ADV_TEST_CH = new TestChannelAdvanced();

        public class TestChannelSimple : Channel
        {
            public override bool CanRead { get { return true; } }
            public override bool CanWrite { get { return true; } }

            public override void failure(String reason) { ADV_TEST_CH.failure(reason);}
            public void send(Pack pack) {ADV_TEST_CH.send(pack); }
            protected internal override Pack process(Pack pack, int id) { return ADV_TEST_CH.process(pack, id); }
        }
        static readonly TestChannelSimple SMP_TEST_CH = new TestChannelSimple();  //test channel with SimpleProtocol



        public static void Main(string[] args)
        {
            Inside PH = new Inside();
            CommunicationChannel.instance.OnHEARTBEATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_QUADROTOR);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_FLIGHT_TERMINATION);
                Debug.Assert(pack.custom_mode == (uint)4289603996U);
                Debug.Assert(pack.mavlink_version == (byte)(byte)214);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY;
            p0.mavlink_version = (byte)(byte)214;
            p0.custom_mode = (uint)4289603996U;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_FLIGHT_TERMINATION;
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_QUADROTOR;
            ADV_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.errors_comm == (ushort)(ushort)46484);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)1181);
                Debug.Assert(pack.current_battery == (short)(short)32527);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL);
                Debug.Assert(pack.load == (ushort)(ushort)13132);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)11157);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)3967);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)53118);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)83);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)30407);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)11323);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.errors_comm = (ushort)(ushort)46484;
            p1.errors_count1 = (ushort)(ushort)3967;
            p1.voltage_battery = (ushort)(ushort)11323;
            p1.current_battery = (short)(short)32527;
            p1.drop_rate_comm = (ushort)(ushort)1181;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION;
            p1.errors_count2 = (ushort)(ushort)30407;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
            p1.errors_count3 = (ushort)(ushort)53118;
            p1.battery_remaining = (sbyte)(sbyte)83;
            p1.load = (ushort)(ushort)13132;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL;
            p1.errors_count4 = (ushort)(ushort)11157;
            ADV_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3584082387U);
                Debug.Assert(pack.time_unix_usec == (ulong)9052582953734367565L);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_boot_ms = (uint)3584082387U;
            p2.time_unix_usec = (ulong)9052582953734367565L;
            ADV_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (ushort)(ushort)8355);
                Debug.Assert(pack.vx == (float) -2.2960308E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.x == (float)1.292857E38F);
                Debug.Assert(pack.yaw == (float)2.5747288E38F);
                Debug.Assert(pack.yaw_rate == (float) -3.2116088E38F);
                Debug.Assert(pack.vz == (float)2.4140203E38F);
                Debug.Assert(pack.afx == (float) -2.1696702E38F);
                Debug.Assert(pack.vy == (float)4.2186692E37F);
                Debug.Assert(pack.y == (float)8.390624E37F);
                Debug.Assert(pack.z == (float)1.4302489E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3897239209U);
                Debug.Assert(pack.afy == (float) -5.2632564E37F);
                Debug.Assert(pack.afz == (float)1.7471525E38F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)3897239209U;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p3.x = (float)1.292857E38F;
            p3.vy = (float)4.2186692E37F;
            p3.vx = (float) -2.2960308E38F;
            p3.yaw_rate = (float) -3.2116088E38F;
            p3.z = (float)1.4302489E38F;
            p3.afx = (float) -2.1696702E38F;
            p3.type_mask = (ushort)(ushort)8355;
            p3.yaw = (float)2.5747288E38F;
            p3.afz = (float)1.7471525E38F;
            p3.vz = (float)2.4140203E38F;
            p3.afy = (float) -5.2632564E37F;
            p3.y = (float)8.390624E37F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)158);
                Debug.Assert(pack.seq == (uint)1715025301U);
                Debug.Assert(pack.target_system == (byte)(byte)226);
                Debug.Assert(pack.time_usec == (ulong)1307634041501701899L);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.target_system = (byte)(byte)226;
            p4.time_usec = (ulong)1307634041501701899L;
            p4.seq = (uint)1715025301U;
            p4.target_component = (byte)(byte)158;
            ADV_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)76);
                Debug.Assert(pack.version == (byte)(byte)228);
                Debug.Assert(pack.passkey_LEN(ph) == 19);
                Debug.Assert(pack.passkey_TRY(ph).Equals("CvavghupicnaLntvilo"));
                Debug.Assert(pack.control_request == (byte)(byte)199);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.passkey_SET("CvavghupicnaLntvilo", PH) ;
            p5.version = (byte)(byte)228;
            p5.control_request = (byte)(byte)199;
            p5.target_system = (byte)(byte)76;
            ADV_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)154);
                Debug.Assert(pack.ack == (byte)(byte)39);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)25);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)39;
            p6.gcs_system_id = (byte)(byte)25;
            p6.control_request = (byte)(byte)154;
            ADV_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 5);
                Debug.Assert(pack.key_TRY(ph).Equals("vuqcy"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("vuqcy", PH) ;
            ADV_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_mode == (uint)2923110739U);
                Debug.Assert(pack.target_system == (byte)(byte)232);
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)232;
            p11.custom_mode = (uint)2923110739U;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            ADV_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)60);
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("xyhcrkq"));
                Debug.Assert(pack.target_system == (byte)(byte)21);
                Debug.Assert(pack.param_index == (short)(short) -517);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)21;
            p20.param_id_SET("xyhcrkq", PH) ;
            p20.target_component = (byte)(byte)60;
            p20.param_index = (short)(short) -517;
            ADV_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)62);
                Debug.Assert(pack.target_system == (byte)(byte)162);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)162;
            p21.target_component = (byte)(byte)62;
            ADV_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value == (float) -1.743799E38F);
                Debug.Assert(pack.param_index == (ushort)(ushort)13186);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ci"));
                Debug.Assert(pack.param_count == (ushort)(ushort)29453);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_value = (float) -1.743799E38F;
            p22.param_index = (ushort)(ushort)13186;
            p22.param_id_SET("ci", PH) ;
            p22.param_count = (ushort)(ushort)29453;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32;
            ADV_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ffbcedkvwNglxgsi"));
                Debug.Assert(pack.param_value == (float)2.399098E38F);
                Debug.Assert(pack.target_component == (byte)(byte)238);
                Debug.Assert(pack.target_system == (byte)(byte)227);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_id_SET("ffbcedkvwNglxgsi", PH) ;
            p23.target_system = (byte)(byte)227;
            p23.param_value = (float)2.399098E38F;
            p23.target_component = (byte)(byte)238;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32;
            ADV_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)2659415275U);
                Debug.Assert(pack.lat == (int) -98398187);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -2135714018);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
                Debug.Assert(pack.time_usec == (ulong)7650905494164879041L);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)3049932597U);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)766076639U);
                Debug.Assert(pack.eph == (ushort)(ushort)16673);
                Debug.Assert(pack.epv == (ushort)(ushort)1470);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)624375395U);
                Debug.Assert(pack.vel == (ushort)(ushort)41196);
                Debug.Assert(pack.satellites_visible == (byte)(byte)249);
                Debug.Assert(pack.alt == (int) -658727378);
                Debug.Assert(pack.lon == (int)2017583530);
                Debug.Assert(pack.cog == (ushort)(ushort)57305);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.satellites_visible = (byte)(byte)249;
            p24.alt_ellipsoid_SET((int) -2135714018, PH) ;
            p24.hdg_acc_SET((uint)766076639U, PH) ;
            p24.lon = (int)2017583530;
            p24.epv = (ushort)(ushort)1470;
            p24.eph = (ushort)(ushort)16673;
            p24.lat = (int) -98398187;
            p24.time_usec = (ulong)7650905494164879041L;
            p24.vel_acc_SET((uint)3049932597U, PH) ;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED;
            p24.alt = (int) -658727378;
            p24.h_acc_SET((uint)2659415275U, PH) ;
            p24.vel = (ushort)(ushort)41196;
            p24.cog = (ushort)(ushort)57305;
            p24.v_acc_SET((uint)624375395U, PH) ;
            ADV_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)86);
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)56, (byte)241, (byte)125, (byte)53, (byte)69, (byte)136, (byte)243, (byte)134, (byte)89, (byte)77, (byte)248, (byte)170, (byte)4, (byte)122, (byte)99, (byte)23, (byte)207, (byte)2, (byte)76, (byte)114}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)68, (byte)204, (byte)239, (byte)87, (byte)181, (byte)141, (byte)179, (byte)68, (byte)216, (byte)220, (byte)204, (byte)92, (byte)169, (byte)226, (byte)124, (byte)102, (byte)174, (byte)226, (byte)119, (byte)144}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)179, (byte)63, (byte)152, (byte)107, (byte)35, (byte)5, (byte)180, (byte)70, (byte)68, (byte)30, (byte)2, (byte)130, (byte)2, (byte)246, (byte)107, (byte)119, (byte)13, (byte)111, (byte)87, (byte)124}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)142, (byte)88, (byte)147, (byte)114, (byte)203, (byte)184, (byte)94, (byte)233, (byte)240, (byte)84, (byte)89, (byte)80, (byte)153, (byte)134, (byte)155, (byte)115, (byte)215, (byte)205, (byte)186, (byte)52}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)15, (byte)92, (byte)254, (byte)127, (byte)138, (byte)189, (byte)123, (byte)63, (byte)242, (byte)80, (byte)157, (byte)239, (byte)33, (byte)152, (byte)217, (byte)137, (byte)191, (byte)63, (byte)211, (byte)74}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_used_SET(new byte[] {(byte)15, (byte)92, (byte)254, (byte)127, (byte)138, (byte)189, (byte)123, (byte)63, (byte)242, (byte)80, (byte)157, (byte)239, (byte)33, (byte)152, (byte)217, (byte)137, (byte)191, (byte)63, (byte)211, (byte)74}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)142, (byte)88, (byte)147, (byte)114, (byte)203, (byte)184, (byte)94, (byte)233, (byte)240, (byte)84, (byte)89, (byte)80, (byte)153, (byte)134, (byte)155, (byte)115, (byte)215, (byte)205, (byte)186, (byte)52}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)68, (byte)204, (byte)239, (byte)87, (byte)181, (byte)141, (byte)179, (byte)68, (byte)216, (byte)220, (byte)204, (byte)92, (byte)169, (byte)226, (byte)124, (byte)102, (byte)174, (byte)226, (byte)119, (byte)144}, 0) ;
            p25.satellites_visible = (byte)(byte)86;
            p25.satellite_elevation_SET(new byte[] {(byte)56, (byte)241, (byte)125, (byte)53, (byte)69, (byte)136, (byte)243, (byte)134, (byte)89, (byte)77, (byte)248, (byte)170, (byte)4, (byte)122, (byte)99, (byte)23, (byte)207, (byte)2, (byte)76, (byte)114}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)179, (byte)63, (byte)152, (byte)107, (byte)35, (byte)5, (byte)180, (byte)70, (byte)68, (byte)30, (byte)2, (byte)130, (byte)2, (byte)246, (byte)107, (byte)119, (byte)13, (byte)111, (byte)87, (byte)124}, 0) ;
            ADV_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short)899);
                Debug.Assert(pack.yacc == (short)(short) -4033);
                Debug.Assert(pack.xmag == (short)(short)1767);
                Debug.Assert(pack.ymag == (short)(short)22119);
                Debug.Assert(pack.zmag == (short)(short)27042);
                Debug.Assert(pack.xgyro == (short)(short) -9679);
                Debug.Assert(pack.xacc == (short)(short) -30423);
                Debug.Assert(pack.time_boot_ms == (uint)4055748087U);
                Debug.Assert(pack.ygyro == (short)(short)6054);
                Debug.Assert(pack.zgyro == (short)(short)32006);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.yacc = (short)(short) -4033;
            p26.ygyro = (short)(short)6054;
            p26.xgyro = (short)(short) -9679;
            p26.zacc = (short)(short)899;
            p26.xacc = (short)(short) -30423;
            p26.xmag = (short)(short)1767;
            p26.time_boot_ms = (uint)4055748087U;
            p26.zgyro = (short)(short)32006;
            p26.ymag = (short)(short)22119;
            p26.zmag = (short)(short)27042;
            ADV_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short)6947);
                Debug.Assert(pack.xacc == (short)(short) -23656);
                Debug.Assert(pack.ymag == (short)(short) -16388);
                Debug.Assert(pack.zacc == (short)(short)10443);
                Debug.Assert(pack.zmag == (short)(short)9879);
                Debug.Assert(pack.xgyro == (short)(short)15109);
                Debug.Assert(pack.zgyro == (short)(short) -22531);
                Debug.Assert(pack.time_usec == (ulong)9132620170841163453L);
                Debug.Assert(pack.ygyro == (short)(short)6506);
                Debug.Assert(pack.xmag == (short)(short)20558);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.zgyro = (short)(short) -22531;
            p27.yacc = (short)(short)6947;
            p27.xacc = (short)(short) -23656;
            p27.ymag = (short)(short) -16388;
            p27.xgyro = (short)(short)15109;
            p27.time_usec = (ulong)9132620170841163453L;
            p27.ygyro = (short)(short)6506;
            p27.xmag = (short)(short)20558;
            p27.zmag = (short)(short)9879;
            p27.zacc = (short)(short)10443;
            ADV_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)8680244228575147483L);
                Debug.Assert(pack.press_diff1 == (short)(short) -24410);
                Debug.Assert(pack.press_abs == (short)(short) -32184);
                Debug.Assert(pack.temperature == (short)(short)27891);
                Debug.Assert(pack.press_diff2 == (short)(short)8451);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.temperature = (short)(short)27891;
            p28.time_usec = (ulong)8680244228575147483L;
            p28.press_abs = (short)(short) -32184;
            p28.press_diff2 = (short)(short)8451;
            p28.press_diff1 = (short)(short) -24410;
            ADV_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float)1.8056154E38F);
                Debug.Assert(pack.press_abs == (float)1.9473083E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3305338647U);
                Debug.Assert(pack.temperature == (short)(short)18498);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.time_boot_ms = (uint)3305338647U;
            p29.temperature = (short)(short)18498;
            p29.press_abs = (float)1.9473083E38F;
            p29.press_diff = (float)1.8056154E38F;
            ADV_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)5.4567825E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3041210405U);
                Debug.Assert(pack.roll == (float)3.621373E36F);
                Debug.Assert(pack.pitchspeed == (float)6.6044764E37F);
                Debug.Assert(pack.yawspeed == (float) -3.051003E38F);
                Debug.Assert(pack.yaw == (float) -3.2160093E38F);
                Debug.Assert(pack.pitch == (float) -2.0558964E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.pitch = (float) -2.0558964E38F;
            p30.roll = (float)3.621373E36F;
            p30.time_boot_ms = (uint)3041210405U;
            p30.yaw = (float) -3.2160093E38F;
            p30.rollspeed = (float)5.4567825E37F;
            p30.pitchspeed = (float)6.6044764E37F;
            p30.yawspeed = (float) -3.051003E38F;
            ADV_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)1.8798508E38F);
                Debug.Assert(pack.pitchspeed == (float)2.5403363E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2699033150U);
                Debug.Assert(pack.q4 == (float)2.662843E38F);
                Debug.Assert(pack.q1 == (float) -2.914082E38F);
                Debug.Assert(pack.q2 == (float) -1.5380959E38F);
                Debug.Assert(pack.yawspeed == (float) -7.316674E37F);
                Debug.Assert(pack.q3 == (float)1.964632E38F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.pitchspeed = (float)2.5403363E38F;
            p31.q2 = (float) -1.5380959E38F;
            p31.q3 = (float)1.964632E38F;
            p31.q4 = (float)2.662843E38F;
            p31.rollspeed = (float)1.8798508E38F;
            p31.q1 = (float) -2.914082E38F;
            p31.time_boot_ms = (uint)2699033150U;
            p31.yawspeed = (float) -7.316674E37F;
            ADV_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float) -5.723088E37F);
                Debug.Assert(pack.vy == (float)2.28361E38F);
                Debug.Assert(pack.z == (float)2.2329205E38F);
                Debug.Assert(pack.y == (float)3.023429E38F);
                Debug.Assert(pack.vx == (float) -1.7054626E38F);
                Debug.Assert(pack.time_boot_ms == (uint)899685878U);
                Debug.Assert(pack.x == (float)2.1000865E38F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.x = (float)2.1000865E38F;
            p32.y = (float)3.023429E38F;
            p32.vx = (float) -1.7054626E38F;
            p32.time_boot_ms = (uint)899685878U;
            p32.z = (float)2.2329205E38F;
            p32.vy = (float)2.28361E38F;
            p32.vz = (float) -5.723088E37F;
            ADV_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (short)(short) -24595);
                Debug.Assert(pack.vx == (short)(short) -11351);
                Debug.Assert(pack.hdg == (ushort)(ushort)44553);
                Debug.Assert(pack.lat == (int)106445882);
                Debug.Assert(pack.time_boot_ms == (uint)2221364805U);
                Debug.Assert(pack.relative_alt == (int)360134967);
                Debug.Assert(pack.vz == (short)(short) -3801);
                Debug.Assert(pack.lon == (int) -1441663513);
                Debug.Assert(pack.alt == (int) -1559751068);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.vy = (short)(short) -24595;
            p33.lon = (int) -1441663513;
            p33.vx = (short)(short) -11351;
            p33.relative_alt = (int)360134967;
            p33.time_boot_ms = (uint)2221364805U;
            p33.vz = (short)(short) -3801;
            p33.hdg = (ushort)(ushort)44553;
            p33.lat = (int)106445882;
            p33.alt = (int) -1559751068;
            ADV_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_scaled == (short)(short)8256);
                Debug.Assert(pack.chan1_scaled == (short)(short) -13210);
                Debug.Assert(pack.chan8_scaled == (short)(short) -31161);
                Debug.Assert(pack.rssi == (byte)(byte)144);
                Debug.Assert(pack.chan5_scaled == (short)(short) -19557);
                Debug.Assert(pack.time_boot_ms == (uint)772331250U);
                Debug.Assert(pack.chan6_scaled == (short)(short) -17246);
                Debug.Assert(pack.chan4_scaled == (short)(short) -5251);
                Debug.Assert(pack.chan2_scaled == (short)(short) -3243);
                Debug.Assert(pack.port == (byte)(byte)106);
                Debug.Assert(pack.chan7_scaled == (short)(short)4270);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan8_scaled = (short)(short) -31161;
            p34.rssi = (byte)(byte)144;
            p34.chan7_scaled = (short)(short)4270;
            p34.time_boot_ms = (uint)772331250U;
            p34.chan6_scaled = (short)(short) -17246;
            p34.chan5_scaled = (short)(short) -19557;
            p34.chan2_scaled = (short)(short) -3243;
            p34.chan1_scaled = (short)(short) -13210;
            p34.chan3_scaled = (short)(short)8256;
            p34.port = (byte)(byte)106;
            p34.chan4_scaled = (short)(short) -5251;
            ADV_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.port == (byte)(byte)104);
                Debug.Assert(pack.time_boot_ms == (uint)3516468386U);
                Debug.Assert(pack.rssi == (byte)(byte)237);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)14584);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)18431);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)9162);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)63038);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)20311);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)43878);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)45373);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)17143);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.time_boot_ms = (uint)3516468386U;
            p35.chan2_raw = (ushort)(ushort)18431;
            p35.port = (byte)(byte)104;
            p35.chan5_raw = (ushort)(ushort)9162;
            p35.chan7_raw = (ushort)(ushort)20311;
            p35.chan1_raw = (ushort)(ushort)63038;
            p35.chan6_raw = (ushort)(ushort)14584;
            p35.rssi = (byte)(byte)237;
            p35.chan3_raw = (ushort)(ushort)45373;
            p35.chan8_raw = (ushort)(ushort)17143;
            p35.chan4_raw = (ushort)(ushort)43878;
            ADV_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)4256);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)57497);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)63025);
                Debug.Assert(pack.port == (byte)(byte)118);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)42924);
                Debug.Assert(pack.time_usec == (uint)931895531U);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)35226);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)46023);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)51104);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)48422);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)39594);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)51296);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)20294);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)20416);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)44487);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)6233);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)12824);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)60661);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo6_raw = (ushort)(ushort)4256;
            p36.servo4_raw = (ushort)(ushort)42924;
            p36.servo7_raw = (ushort)(ushort)48422;
            p36.servo8_raw = (ushort)(ushort)44487;
            p36.servo14_raw_SET((ushort)(ushort)63025, PH) ;
            p36.port = (byte)(byte)118;
            p36.servo12_raw_SET((ushort)(ushort)51104, PH) ;
            p36.servo10_raw_SET((ushort)(ushort)20416, PH) ;
            p36.servo5_raw = (ushort)(ushort)57497;
            p36.time_usec = (uint)931895531U;
            p36.servo15_raw_SET((ushort)(ushort)35226, PH) ;
            p36.servo3_raw = (ushort)(ushort)46023;
            p36.servo13_raw_SET((ushort)(ushort)6233, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)60661, PH) ;
            p36.servo2_raw = (ushort)(ushort)39594;
            p36.servo1_raw = (ushort)(ushort)20294;
            p36.servo11_raw_SET((ushort)(ushort)51296, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)12824, PH) ;
            ADV_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short) -1994);
                Debug.Assert(pack.target_system == (byte)(byte)247);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)137);
                Debug.Assert(pack.start_index == (short)(short) -9797);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.end_index = (short)(short) -1994;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p37.target_system = (byte)(byte)247;
            p37.start_index = (short)(short) -9797;
            p37.target_component = (byte)(byte)137;
            ADV_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short) -26913);
                Debug.Assert(pack.target_component == (byte)(byte)248);
                Debug.Assert(pack.target_system == (byte)(byte)178);
                Debug.Assert(pack.start_index == (short)(short) -14861);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p38.target_component = (byte)(byte)248;
            p38.target_system = (byte)(byte)178;
            p38.end_index = (short)(short) -26913;
            p38.start_index = (short)(short) -14861;
            ADV_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param2 == (float)1.5301581E37F);
                Debug.Assert(pack.x == (float)2.819657E38F);
                Debug.Assert(pack.current == (byte)(byte)177);
                Debug.Assert(pack.y == (float) -3.0088807E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE);
                Debug.Assert(pack.seq == (ushort)(ushort)43698);
                Debug.Assert(pack.target_component == (byte)(byte)143);
                Debug.Assert(pack.z == (float) -1.1376048E38F);
                Debug.Assert(pack.param3 == (float)3.315178E38F);
                Debug.Assert(pack.target_system == (byte)(byte)155);
                Debug.Assert(pack.param4 == (float) -2.535078E37F);
                Debug.Assert(pack.param1 == (float)7.820878E37F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.autocontinue == (byte)(byte)73);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.param3 = (float)3.315178E38F;
            p39.z = (float) -1.1376048E38F;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p39.x = (float)2.819657E38F;
            p39.y = (float) -3.0088807E38F;
            p39.param4 = (float) -2.535078E37F;
            p39.current = (byte)(byte)177;
            p39.param1 = (float)7.820878E37F;
            p39.target_component = (byte)(byte)143;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE;
            p39.autocontinue = (byte)(byte)73;
            p39.param2 = (float)1.5301581E37F;
            p39.seq = (ushort)(ushort)43698;
            p39.target_system = (byte)(byte)155;
            ADV_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)133);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)237);
                Debug.Assert(pack.seq == (ushort)(ushort)27085);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.seq = (ushort)(ushort)27085;
            p40.target_system = (byte)(byte)237;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p40.target_component = (byte)(byte)133;
            ADV_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)168);
                Debug.Assert(pack.target_system == (byte)(byte)45);
                Debug.Assert(pack.seq == (ushort)(ushort)31919);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_component = (byte)(byte)168;
            p41.seq = (ushort)(ushort)31919;
            p41.target_system = (byte)(byte)45;
            ADV_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)61213);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)61213;
            ADV_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)11);
                Debug.Assert(pack.target_component == (byte)(byte)132);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_component = (byte)(byte)132;
            p43.target_system = (byte)(byte)11;
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            ADV_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)76);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)240);
                Debug.Assert(pack.count == (ushort)(ushort)39269);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)240;
            p44.target_component = (byte)(byte)76;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p44.count = (ushort)(ushort)39269;
            ADV_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)134);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)7);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p45.target_component = (byte)(byte)134;
            p45.target_system = (byte)(byte)7;
            ADV_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)20783);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)20783;
            ADV_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)253);
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_DENIED);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)111);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)253;
            p47.target_component = (byte)(byte)111;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_DENIED;
            ADV_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)394276322);
                Debug.Assert(pack.latitude == (int)155900545);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3744039930644878540L);
                Debug.Assert(pack.target_system == (byte)(byte)173);
                Debug.Assert(pack.longitude == (int) -183101904);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.latitude = (int)155900545;
            p48.time_usec_SET((ulong)3744039930644878540L, PH) ;
            p48.target_system = (byte)(byte)173;
            p48.altitude = (int)394276322;
            p48.longitude = (int) -183101904;
            ADV_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int) -619980635);
                Debug.Assert(pack.longitude == (int)1028589981);
                Debug.Assert(pack.latitude == (int)1640412739);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6134264396933988589L);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.latitude = (int)1640412739;
            p49.altitude = (int) -619980635;
            p49.longitude = (int)1028589981;
            p49.time_usec_SET((ulong)6134264396933988589L, PH) ;
            ADV_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_max == (float)5.601883E37F);
                Debug.Assert(pack.target_component == (byte)(byte)253);
                Debug.Assert(pack.scale == (float)9.294262E37F);
                Debug.Assert(pack.param_index == (short)(short) -31187);
                Debug.Assert(pack.target_system == (byte)(byte)39);
                Debug.Assert(pack.param_value0 == (float)1.1587686E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("wdqfgxhuhzAmKpfk"));
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)147);
                Debug.Assert(pack.param_value_min == (float) -1.1734306E38F);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_component = (byte)(byte)253;
            p50.param_value_min = (float) -1.1734306E38F;
            p50.param_value_max = (float)5.601883E37F;
            p50.parameter_rc_channel_index = (byte)(byte)147;
            p50.param_value0 = (float)1.1587686E38F;
            p50.target_system = (byte)(byte)39;
            p50.param_index = (short)(short) -31187;
            p50.param_id_SET("wdqfgxhuhzAmKpfk", PH) ;
            p50.scale = (float)9.294262E37F;
            ADV_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.seq == (ushort)(ushort)63120);
                Debug.Assert(pack.target_system == (byte)(byte)58);
                Debug.Assert(pack.target_component == (byte)(byte)53);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_component = (byte)(byte)53;
            p51.target_system = (byte)(byte)58;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p51.seq = (ushort)(ushort)63120;
            ADV_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2y == (float)1.2494915E38F);
                Debug.Assert(pack.target_system == (byte)(byte)64);
                Debug.Assert(pack.p2x == (float)3.2805804E37F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.p1x == (float)7.458017E37F);
                Debug.Assert(pack.target_component == (byte)(byte)202);
                Debug.Assert(pack.p2z == (float) -2.7129238E38F);
                Debug.Assert(pack.p1y == (float) -2.5207289E38F);
                Debug.Assert(pack.p1z == (float) -8.318879E37F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1z = (float) -8.318879E37F;
            p54.target_system = (byte)(byte)64;
            p54.p2y = (float)1.2494915E38F;
            p54.p1x = (float)7.458017E37F;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p54.p1y = (float) -2.5207289E38F;
            p54.p2z = (float) -2.7129238E38F;
            p54.target_component = (byte)(byte)202;
            p54.p2x = (float)3.2805804E37F;
            ADV_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2x == (float) -1.7101355E38F);
                Debug.Assert(pack.p1z == (float) -5.929742E37F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.p2z == (float) -1.4249627E38F);
                Debug.Assert(pack.p1y == (float)1.263299E38F);
                Debug.Assert(pack.p2y == (float) -7.3351187E37F);
                Debug.Assert(pack.p1x == (float)1.3978555E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2y = (float) -7.3351187E37F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p55.p1x = (float)1.3978555E38F;
            p55.p1z = (float) -5.929742E37F;
            p55.p2x = (float) -1.7101355E38F;
            p55.p1y = (float)1.263299E38F;
            p55.p2z = (float) -1.4249627E38F;
            ADV_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.776924E37F, 1.051299E38F, 1.826193E38F, 2.936562E38F, -1.2837924E38F, -4.5447216E37F, -1.6679006E37F, 1.2037832E38F, 5.0494255E37F}));
                Debug.Assert(pack.pitchspeed == (float) -1.877539E38F);
                Debug.Assert(pack.time_usec == (ulong)3709824782568880546L);
                Debug.Assert(pack.yawspeed == (float) -1.6840289E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {7.232965E37F, 1.7533443E38F, 1.1500609E38F, -1.8590463E38F}));
                Debug.Assert(pack.rollspeed == (float) -1.2454446E38F);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.covariance_SET(new float[] {2.776924E37F, 1.051299E38F, 1.826193E38F, 2.936562E38F, -1.2837924E38F, -4.5447216E37F, -1.6679006E37F, 1.2037832E38F, 5.0494255E37F}, 0) ;
            p61.rollspeed = (float) -1.2454446E38F;
            p61.time_usec = (ulong)3709824782568880546L;
            p61.pitchspeed = (float) -1.877539E38F;
            p61.yawspeed = (float) -1.6840289E38F;
            p61.q_SET(new float[] {7.232965E37F, 1.7533443E38F, 1.1500609E38F, -1.8590463E38F}, 0) ;
            ADV_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_roll == (float)1.7486725E38F);
                Debug.Assert(pack.xtrack_error == (float)7.7428296E37F);
                Debug.Assert(pack.nav_pitch == (float) -2.9515064E38F);
                Debug.Assert(pack.target_bearing == (short)(short)18927);
                Debug.Assert(pack.alt_error == (float) -1.1666133E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)3499);
                Debug.Assert(pack.nav_bearing == (short)(short) -899);
                Debug.Assert(pack.aspd_error == (float)3.3405283E38F);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.xtrack_error = (float)7.7428296E37F;
            p62.aspd_error = (float)3.3405283E38F;
            p62.nav_bearing = (short)(short) -899;
            p62.nav_pitch = (float) -2.9515064E38F;
            p62.wp_dist = (ushort)(ushort)3499;
            p62.nav_roll = (float)1.7486725E38F;
            p62.alt_error = (float) -1.1666133E38F;
            p62.target_bearing = (short)(short)18927;
            ADV_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.relative_alt == (int) -455000683);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
                Debug.Assert(pack.lon == (int) -118250939);
                Debug.Assert(pack.vy == (float) -2.2193351E38F);
                Debug.Assert(pack.vx == (float) -1.188937E38F);
                Debug.Assert(pack.alt == (int) -1456250370);
                Debug.Assert(pack.vz == (float) -1.6228084E38F);
                Debug.Assert(pack.time_usec == (ulong)4036932495591004040L);
                Debug.Assert(pack.lat == (int)1012539904);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.3601029E38F, 2.9082134E38F, -4.8030287E37F, 3.1414684E38F, 2.6627372E38F, -7.2361913E37F, 1.6866796E36F, 1.913458E38F, -3.0294353E38F, 1.9185113E38F, -2.5572516E38F, -3.1377227E37F, -4.9228775E37F, -8.522234E37F, 8.0751027E37F, 1.814753E38F, -3.2635614E38F, -1.2137514E38F, -9.670046E37F, 3.664673E37F, -5.823144E37F, -2.214933E38F, -8.936762E37F, 5.6200296E37F, -6.1510215E37F, -1.299082E38F, 2.3416118E37F, -1.3309602E37F, 8.661056E37F, -3.1768774E38F, 9.595564E37F, -1.7543915E38F, 2.0311965E38F, -3.0077408E38F, -3.348572E38F, 2.5697906E38F}));
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.relative_alt = (int) -455000683;
            p63.time_usec = (ulong)4036932495591004040L;
            p63.covariance_SET(new float[] {1.3601029E38F, 2.9082134E38F, -4.8030287E37F, 3.1414684E38F, 2.6627372E38F, -7.2361913E37F, 1.6866796E36F, 1.913458E38F, -3.0294353E38F, 1.9185113E38F, -2.5572516E38F, -3.1377227E37F, -4.9228775E37F, -8.522234E37F, 8.0751027E37F, 1.814753E38F, -3.2635614E38F, -1.2137514E38F, -9.670046E37F, 3.664673E37F, -5.823144E37F, -2.214933E38F, -8.936762E37F, 5.6200296E37F, -6.1510215E37F, -1.299082E38F, 2.3416118E37F, -1.3309602E37F, 8.661056E37F, -3.1768774E38F, 9.595564E37F, -1.7543915E38F, 2.0311965E38F, -3.0077408E38F, -3.348572E38F, 2.5697906E38F}, 0) ;
            p63.alt = (int) -1456250370;
            p63.vx = (float) -1.188937E38F;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            p63.vy = (float) -2.2193351E38F;
            p63.vz = (float) -1.6228084E38F;
            p63.lat = (int)1012539904;
            p63.lon = (int) -118250939;
            ADV_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.az == (float) -1.2292459E38F);
                Debug.Assert(pack.time_usec == (ulong)5260756898811763170L);
                Debug.Assert(pack.ay == (float) -1.731908E38F);
                Debug.Assert(pack.z == (float)2.86939E38F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
                Debug.Assert(pack.ax == (float)3.0316093E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {5.7163365E37F, -3.3207091E38F, 3.3441957E38F, -1.2018443E38F, 1.939478E38F, 2.7199376E37F, -3.0613156E38F, 1.8178556E38F, -2.9138243E38F, 1.7462041E38F, 1.2265053E38F, 1.1372407E37F, -3.3217985E38F, -1.6939575E38F, 3.0980697E38F, 2.6198302E38F, 7.664299E36F, -3.0392592E38F, 2.3729855E38F, 1.586536E38F, 2.398129E38F, -1.5748492E38F, -7.013637E37F, 1.7712352E38F, 3.8306692E37F, -9.766255E37F, 1.1460631E38F, 2.0956124E38F, -3.1595782E38F, -2.0828609E38F, -3.6213183E37F, -3.3064753E38F, -1.6419801E38F, 8.274854E37F, 1.0447018E38F, 2.038441E38F, -1.9462578E38F, 1.2712162E38F, 6.847134E37F, -3.5953885E37F, -4.3569714E37F, 2.295196E38F, 4.317671E37F, 2.2009864E38F, 5.685496E37F}));
                Debug.Assert(pack.vx == (float) -2.965433E38F);
                Debug.Assert(pack.vz == (float)2.1894312E38F);
                Debug.Assert(pack.y == (float) -1.7139766E38F);
                Debug.Assert(pack.x == (float) -6.679608E37F);
                Debug.Assert(pack.vy == (float) -2.503645E38F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.time_usec = (ulong)5260756898811763170L;
            p64.vz = (float)2.1894312E38F;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            p64.x = (float) -6.679608E37F;
            p64.vx = (float) -2.965433E38F;
            p64.ay = (float) -1.731908E38F;
            p64.az = (float) -1.2292459E38F;
            p64.vy = (float) -2.503645E38F;
            p64.covariance_SET(new float[] {5.7163365E37F, -3.3207091E38F, 3.3441957E38F, -1.2018443E38F, 1.939478E38F, 2.7199376E37F, -3.0613156E38F, 1.8178556E38F, -2.9138243E38F, 1.7462041E38F, 1.2265053E38F, 1.1372407E37F, -3.3217985E38F, -1.6939575E38F, 3.0980697E38F, 2.6198302E38F, 7.664299E36F, -3.0392592E38F, 2.3729855E38F, 1.586536E38F, 2.398129E38F, -1.5748492E38F, -7.013637E37F, 1.7712352E38F, 3.8306692E37F, -9.766255E37F, 1.1460631E38F, 2.0956124E38F, -3.1595782E38F, -2.0828609E38F, -3.6213183E37F, -3.3064753E38F, -1.6419801E38F, 8.274854E37F, 1.0447018E38F, 2.038441E38F, -1.9462578E38F, 1.2712162E38F, 6.847134E37F, -3.5953885E37F, -4.3569714E37F, 2.295196E38F, 4.317671E37F, 2.2009864E38F, 5.685496E37F}, 0) ;
            p64.y = (float) -1.7139766E38F;
            p64.ax = (float)3.0316093E38F;
            p64.z = (float)2.86939E38F;
            ADV_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chancount == (byte)(byte)117);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)34348);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)35922);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)54531);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)41736);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)31695);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)43122);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)20078);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)25103);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)42366);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)57304);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)53099);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)65265);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)50438);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)16513);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)49864);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)17128);
                Debug.Assert(pack.time_boot_ms == (uint)1459267674U);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)50638);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)28530);
                Debug.Assert(pack.rssi == (byte)(byte)108);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan7_raw = (ushort)(ushort)53099;
            p65.chan2_raw = (ushort)(ushort)57304;
            p65.rssi = (byte)(byte)108;
            p65.chan4_raw = (ushort)(ushort)25103;
            p65.chan16_raw = (ushort)(ushort)16513;
            p65.chan5_raw = (ushort)(ushort)28530;
            p65.chan6_raw = (ushort)(ushort)35922;
            p65.time_boot_ms = (uint)1459267674U;
            p65.chancount = (byte)(byte)117;
            p65.chan18_raw = (ushort)(ushort)49864;
            p65.chan9_raw = (ushort)(ushort)50638;
            p65.chan15_raw = (ushort)(ushort)34348;
            p65.chan12_raw = (ushort)(ushort)41736;
            p65.chan14_raw = (ushort)(ushort)65265;
            p65.chan13_raw = (ushort)(ushort)50438;
            p65.chan11_raw = (ushort)(ushort)42366;
            p65.chan3_raw = (ushort)(ushort)43122;
            p65.chan8_raw = (ushort)(ushort)17128;
            p65.chan10_raw = (ushort)(ushort)20078;
            p65.chan1_raw = (ushort)(ushort)54531;
            p65.chan17_raw = (ushort)(ushort)31695;
            ADV_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_stop == (byte)(byte)232);
                Debug.Assert(pack.req_stream_id == (byte)(byte)22);
                Debug.Assert(pack.target_component == (byte)(byte)120);
                Debug.Assert(pack.target_system == (byte)(byte)7);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)35197);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_system = (byte)(byte)7;
            p66.start_stop = (byte)(byte)232;
            p66.target_component = (byte)(byte)120;
            p66.req_message_rate = (ushort)(ushort)35197;
            p66.req_stream_id = (byte)(byte)22;
            ADV_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)49292);
                Debug.Assert(pack.on_off == (byte)(byte)162);
                Debug.Assert(pack.stream_id == (byte)(byte)39);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.message_rate = (ushort)(ushort)49292;
            p67.on_off = (byte)(byte)162;
            p67.stream_id = (byte)(byte)39;
            ADV_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.buttons == (ushort)(ushort)20335);
                Debug.Assert(pack.y == (short)(short) -7640);
                Debug.Assert(pack.z == (short)(short)18883);
                Debug.Assert(pack.target == (byte)(byte)187);
                Debug.Assert(pack.x == (short)(short)1874);
                Debug.Assert(pack.r == (short)(short) -21531);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.z = (short)(short)18883;
            p69.r = (short)(short) -21531;
            p69.target = (byte)(byte)187;
            p69.buttons = (ushort)(ushort)20335;
            p69.y = (short)(short) -7640;
            p69.x = (short)(short)1874;
            ADV_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)21950);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)53178);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)39424);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)4008);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)61101);
                Debug.Assert(pack.target_component == (byte)(byte)93);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)36440);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)64572);
                Debug.Assert(pack.target_system == (byte)(byte)17);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)9962);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan6_raw = (ushort)(ushort)9962;
            p70.target_system = (byte)(byte)17;
            p70.chan4_raw = (ushort)(ushort)36440;
            p70.chan7_raw = (ushort)(ushort)39424;
            p70.target_component = (byte)(byte)93;
            p70.chan1_raw = (ushort)(ushort)64572;
            p70.chan8_raw = (ushort)(ushort)4008;
            p70.chan2_raw = (ushort)(ushort)61101;
            p70.chan5_raw = (ushort)(ushort)53178;
            p70.chan3_raw = (ushort)(ushort)21950;
            ADV_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.autocontinue == (byte)(byte)148);
                Debug.Assert(pack.seq == (ushort)(ushort)60711);
                Debug.Assert(pack.y == (int)283581515);
                Debug.Assert(pack.param3 == (float) -3.282035E38F);
                Debug.Assert(pack.z == (float) -2.685436E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE);
                Debug.Assert(pack.target_component == (byte)(byte)160);
                Debug.Assert(pack.current == (byte)(byte)210);
                Debug.Assert(pack.param2 == (float) -2.8755546E38F);
                Debug.Assert(pack.param4 == (float) -2.3448585E38F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.param1 == (float) -6.635364E37F);
                Debug.Assert(pack.target_system == (byte)(byte)60);
                Debug.Assert(pack.x == (int)1489197095);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.param1 = (float) -6.635364E37F;
            p73.autocontinue = (byte)(byte)148;
            p73.current = (byte)(byte)210;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p73.seq = (ushort)(ushort)60711;
            p73.target_system = (byte)(byte)60;
            p73.x = (int)1489197095;
            p73.target_component = (byte)(byte)160;
            p73.z = (float) -2.685436E38F;
            p73.param2 = (float) -2.8755546E38F;
            p73.param3 = (float) -3.282035E38F;
            p73.param4 = (float) -2.3448585E38F;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE;
            p73.y = (int)283581515;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            ADV_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.groundspeed == (float)1.7042341E38F);
                Debug.Assert(pack.heading == (short)(short)6473);
                Debug.Assert(pack.airspeed == (float) -4.8950166E37F);
                Debug.Assert(pack.alt == (float) -4.3018347E37F);
                Debug.Assert(pack.climb == (float)4.3139245E37F);
                Debug.Assert(pack.throttle == (ushort)(ushort)15994);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.heading = (short)(short)6473;
            p74.airspeed = (float) -4.8950166E37F;
            p74.alt = (float) -4.3018347E37F;
            p74.climb = (float)4.3139245E37F;
            p74.groundspeed = (float)1.7042341E38F;
            p74.throttle = (ushort)(ushort)15994;
            ADV_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)88);
                Debug.Assert(pack.autocontinue == (byte)(byte)34);
                Debug.Assert(pack.x == (int)1298140009);
                Debug.Assert(pack.param1 == (float) -7.5322065E37F);
                Debug.Assert(pack.y == (int)1975553709);
                Debug.Assert(pack.current == (byte)(byte)184);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.z == (float)1.4232891E38F);
                Debug.Assert(pack.param4 == (float)2.3087236E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION);
                Debug.Assert(pack.target_system == (byte)(byte)37);
                Debug.Assert(pack.param3 == (float) -6.185867E37F);
                Debug.Assert(pack.param2 == (float) -1.0037379E38F);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.x = (int)1298140009;
            p75.autocontinue = (byte)(byte)34;
            p75.y = (int)1975553709;
            p75.current = (byte)(byte)184;
            p75.target_system = (byte)(byte)37;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION;
            p75.param4 = (float)2.3087236E38F;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p75.param2 = (float) -1.0037379E38F;
            p75.param3 = (float) -6.185867E37F;
            p75.param1 = (float) -7.5322065E37F;
            p75.z = (float)1.4232891E38F;
            p75.target_component = (byte)(byte)88;
            ADV_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)195);
                Debug.Assert(pack.param7 == (float)2.2502802E38F);
                Debug.Assert(pack.target_component == (byte)(byte)251);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_REPEAT_RELAY);
                Debug.Assert(pack.param5 == (float)2.887788E36F);
                Debug.Assert(pack.confirmation == (byte)(byte)243);
                Debug.Assert(pack.param1 == (float)2.2934947E38F);
                Debug.Assert(pack.param6 == (float) -9.68724E37F);
                Debug.Assert(pack.param4 == (float) -6.0847863E37F);
                Debug.Assert(pack.param2 == (float)7.570132E37F);
                Debug.Assert(pack.param3 == (float)1.3985212E38F);
            };
            COMMAND_LONG p76 = new COMMAND_LONG();
            PH.setPack(p76);
            p76.param5 = (float)2.887788E36F;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_REPEAT_RELAY;
            p76.param6 = (float) -9.68724E37F;
            p76.target_component = (byte)(byte)251;
            p76.param3 = (float)1.3985212E38F;
            p76.param1 = (float)2.2934947E38F;
            p76.target_system = (byte)(byte)195;
            p76.param2 = (float)7.570132E37F;
            p76.confirmation = (byte)(byte)243;
            p76.param7 = (float)2.2502802E38F;
            p76.param4 = (float) -6.0847863E37F;
            ADV_TEST_CH.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -146738513);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_UNSUPPORTED);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)154);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)166);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)83);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT);
            };
            COMMAND_ACK p77 = new COMMAND_ACK();
            PH.setPack(p77);
            p77.target_system_SET((byte)(byte)83, PH) ;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_UNSUPPORTED;
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT;
            p77.target_component_SET((byte)(byte)166, PH) ;
            p77.progress_SET((byte)(byte)154, PH) ;
            p77.result_param2_SET((int) -146738513, PH) ;
            ADV_TEST_CH.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.manual_override_switch == (byte)(byte)141);
                Debug.Assert(pack.time_boot_ms == (uint)279625170U);
                Debug.Assert(pack.yaw == (float)3.3431737E38F);
                Debug.Assert(pack.roll == (float)1.2167507E38F);
                Debug.Assert(pack.pitch == (float) -2.8195022E38F);
                Debug.Assert(pack.thrust == (float) -4.733411E37F);
                Debug.Assert(pack.mode_switch == (byte)(byte)155);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.mode_switch = (byte)(byte)155;
            p81.pitch = (float) -2.8195022E38F;
            p81.thrust = (float) -4.733411E37F;
            p81.yaw = (float)3.3431737E38F;
            p81.manual_override_switch = (byte)(byte)141;
            p81.roll = (float)1.2167507E38F;
            p81.time_boot_ms = (uint)279625170U;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3118593193U);
                Debug.Assert(pack.body_pitch_rate == (float) -1.705571E38F);
                Debug.Assert(pack.thrust == (float) -8.574925E36F);
                Debug.Assert(pack.type_mask == (byte)(byte)189);
                Debug.Assert(pack.target_system == (byte)(byte)74);
                Debug.Assert(pack.body_roll_rate == (float)5.5850475E37F);
                Debug.Assert(pack.body_yaw_rate == (float)2.6976483E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.2997077E38F, 2.0332128E38F, -1.6894456E38F, -1.0842115E38F}));
                Debug.Assert(pack.target_component == (byte)(byte)29);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_roll_rate = (float)5.5850475E37F;
            p82.target_component = (byte)(byte)29;
            p82.target_system = (byte)(byte)74;
            p82.thrust = (float) -8.574925E36F;
            p82.body_yaw_rate = (float)2.6976483E38F;
            p82.q_SET(new float[] {1.2997077E38F, 2.0332128E38F, -1.6894456E38F, -1.0842115E38F}, 0) ;
            p82.time_boot_ms = (uint)3118593193U;
            p82.type_mask = (byte)(byte)189;
            p82.body_pitch_rate = (float) -1.705571E38F;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_yaw_rate == (float) -2.8385216E38F);
                Debug.Assert(pack.body_roll_rate == (float)1.7764786E37F);
                Debug.Assert(pack.thrust == (float) -2.8261406E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {8.787377E37F, 3.093174E37F, 1.1423355E38F, -3.1855922E38F}));
                Debug.Assert(pack.time_boot_ms == (uint)1240751904U);
                Debug.Assert(pack.body_pitch_rate == (float)6.5875436E37F);
                Debug.Assert(pack.type_mask == (byte)(byte)185);
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_yaw_rate = (float) -2.8385216E38F;
            p83.body_roll_rate = (float)1.7764786E37F;
            p83.body_pitch_rate = (float)6.5875436E37F;
            p83.time_boot_ms = (uint)1240751904U;
            p83.q_SET(new float[] {8.787377E37F, 3.093174E37F, 1.1423355E38F, -3.1855922E38F}, 0) ;
            p83.thrust = (float) -2.8261406E38F;
            p83.type_mask = (byte)(byte)185;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afy == (float) -2.8113497E38F);
                Debug.Assert(pack.y == (float) -1.2124472E38F);
                Debug.Assert(pack.target_system == (byte)(byte)67);
                Debug.Assert(pack.vz == (float) -2.3390527E38F);
                Debug.Assert(pack.vx == (float)2.5127735E38F);
                Debug.Assert(pack.afx == (float) -8.934968E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)45421);
                Debug.Assert(pack.vy == (float)1.9594108E38F);
                Debug.Assert(pack.time_boot_ms == (uint)943186344U);
                Debug.Assert(pack.target_component == (byte)(byte)38);
                Debug.Assert(pack.z == (float)2.6572549E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.yaw == (float)2.6533868E38F);
                Debug.Assert(pack.afz == (float) -6.052346E37F);
                Debug.Assert(pack.x == (float) -1.9769774E38F);
                Debug.Assert(pack.yaw_rate == (float)2.3960424E38F);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.vx = (float)2.5127735E38F;
            p84.target_system = (byte)(byte)67;
            p84.vz = (float) -2.3390527E38F;
            p84.afz = (float) -6.052346E37F;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED;
            p84.vy = (float)1.9594108E38F;
            p84.y = (float) -1.2124472E38F;
            p84.x = (float) -1.9769774E38F;
            p84.afx = (float) -8.934968E37F;
            p84.yaw = (float)2.6533868E38F;
            p84.type_mask = (ushort)(ushort)45421;
            p84.afy = (float) -2.8113497E38F;
            p84.z = (float)2.6572549E38F;
            p84.yaw_rate = (float)2.3960424E38F;
            p84.target_component = (byte)(byte)38;
            p84.time_boot_ms = (uint)943186344U;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float)1.3827795E38F);
                Debug.Assert(pack.time_boot_ms == (uint)827243191U);
                Debug.Assert(pack.afx == (float)1.4993525E38F);
                Debug.Assert(pack.yaw == (float) -2.159321E38F);
                Debug.Assert(pack.target_component == (byte)(byte)191);
                Debug.Assert(pack.vx == (float)3.4024967E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.target_system == (byte)(byte)177);
                Debug.Assert(pack.yaw_rate == (float) -3.3341172E38F);
                Debug.Assert(pack.lat_int == (int) -1465877189);
                Debug.Assert(pack.lon_int == (int)1211191193);
                Debug.Assert(pack.afy == (float) -1.4911474E38F);
                Debug.Assert(pack.alt == (float) -2.030527E38F);
                Debug.Assert(pack.afz == (float)1.5512458E38F);
                Debug.Assert(pack.vy == (float) -2.7188382E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)4331);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.afz = (float)1.5512458E38F;
            p86.time_boot_ms = (uint)827243191U;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p86.vx = (float)3.4024967E38F;
            p86.vz = (float)1.3827795E38F;
            p86.yaw_rate = (float) -3.3341172E38F;
            p86.lon_int = (int)1211191193;
            p86.alt = (float) -2.030527E38F;
            p86.afx = (float)1.4993525E38F;
            p86.target_system = (byte)(byte)177;
            p86.yaw = (float) -2.159321E38F;
            p86.vy = (float) -2.7188382E37F;
            p86.afy = (float) -1.4911474E38F;
            p86.type_mask = (ushort)(ushort)4331;
            p86.lat_int = (int) -1465877189;
            p86.target_component = (byte)(byte)191;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afy == (float)4.512525E37F);
                Debug.Assert(pack.afx == (float)1.7408514E38F);
                Debug.Assert(pack.afz == (float)9.620832E37F);
                Debug.Assert(pack.yaw_rate == (float)7.168357E37F);
                Debug.Assert(pack.vz == (float) -2.1011391E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)2536);
                Debug.Assert(pack.lat_int == (int) -646395822);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.time_boot_ms == (uint)3675008872U);
                Debug.Assert(pack.yaw == (float)3.1387753E38F);
                Debug.Assert(pack.vx == (float)2.736611E38F);
                Debug.Assert(pack.vy == (float) -2.6632331E38F);
                Debug.Assert(pack.lon_int == (int)1784893930);
                Debug.Assert(pack.alt == (float)9.884328E37F);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.afz = (float)9.620832E37F;
            p87.afy = (float)4.512525E37F;
            p87.time_boot_ms = (uint)3675008872U;
            p87.vx = (float)2.736611E38F;
            p87.afx = (float)1.7408514E38F;
            p87.vy = (float) -2.6632331E38F;
            p87.yaw = (float)3.1387753E38F;
            p87.vz = (float) -2.1011391E37F;
            p87.lon_int = (int)1784893930;
            p87.lat_int = (int) -646395822;
            p87.type_mask = (ushort)(ushort)2536;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p87.yaw_rate = (float)7.168357E37F;
            p87.alt = (float)9.884328E37F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)8.197232E37F);
                Debug.Assert(pack.z == (float) -1.6455694E38F);
                Debug.Assert(pack.yaw == (float) -1.911434E38F);
                Debug.Assert(pack.pitch == (float)9.431412E37F);
                Debug.Assert(pack.roll == (float)2.799167E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3578945178U);
                Debug.Assert(pack.y == (float) -4.640355E37F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.z = (float) -1.6455694E38F;
            p89.roll = (float)2.799167E38F;
            p89.pitch = (float)9.431412E37F;
            p89.yaw = (float) -1.911434E38F;
            p89.y = (float) -4.640355E37F;
            p89.time_boot_ms = (uint)3578945178U;
            p89.x = (float)8.197232E37F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int)1815840777);
                Debug.Assert(pack.yacc == (short)(short) -12159);
                Debug.Assert(pack.pitchspeed == (float) -1.0043866E38F);
                Debug.Assert(pack.vy == (short)(short) -14603);
                Debug.Assert(pack.yaw == (float) -3.2328409E38F);
                Debug.Assert(pack.xacc == (short)(short)12243);
                Debug.Assert(pack.vz == (short)(short) -18815);
                Debug.Assert(pack.pitch == (float) -2.025043E38F);
                Debug.Assert(pack.lat == (int) -1503378868);
                Debug.Assert(pack.time_usec == (ulong)7467299182701099766L);
                Debug.Assert(pack.vx == (short)(short)5797);
                Debug.Assert(pack.zacc == (short)(short) -22923);
                Debug.Assert(pack.yawspeed == (float)1.4388034E38F);
                Debug.Assert(pack.lon == (int) -1953463255);
                Debug.Assert(pack.roll == (float) -1.5582543E38F);
                Debug.Assert(pack.rollspeed == (float)3.1156742E38F);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.vz = (short)(short) -18815;
            p90.rollspeed = (float)3.1156742E38F;
            p90.yawspeed = (float)1.4388034E38F;
            p90.lon = (int) -1953463255;
            p90.vx = (short)(short)5797;
            p90.yaw = (float) -3.2328409E38F;
            p90.zacc = (short)(short) -22923;
            p90.roll = (float) -1.5582543E38F;
            p90.alt = (int)1815840777;
            p90.pitch = (float) -2.025043E38F;
            p90.time_usec = (ulong)7467299182701099766L;
            p90.yacc = (short)(short) -12159;
            p90.pitchspeed = (float) -1.0043866E38F;
            p90.xacc = (short)(short)12243;
            p90.vy = (short)(short) -14603;
            p90.lat = (int) -1503378868;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch_elevator == (float) -1.0796722E37F);
                Debug.Assert(pack.nav_mode == (byte)(byte)125);
                Debug.Assert(pack.aux2 == (float) -6.9860093E37F);
                Debug.Assert(pack.time_usec == (ulong)2475945478235670058L);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED);
                Debug.Assert(pack.yaw_rudder == (float)5.7751307E37F);
                Debug.Assert(pack.aux1 == (float)1.6221567E38F);
                Debug.Assert(pack.aux4 == (float)1.840458E38F);
                Debug.Assert(pack.aux3 == (float)6.752959E37F);
                Debug.Assert(pack.throttle == (float)2.833549E38F);
                Debug.Assert(pack.roll_ailerons == (float)1.0263328E38F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.throttle = (float)2.833549E38F;
            p91.nav_mode = (byte)(byte)125;
            p91.time_usec = (ulong)2475945478235670058L;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            p91.aux3 = (float)6.752959E37F;
            p91.aux2 = (float) -6.9860093E37F;
            p91.roll_ailerons = (float)1.0263328E38F;
            p91.pitch_elevator = (float) -1.0796722E37F;
            p91.aux4 = (float)1.840458E38F;
            p91.yaw_rudder = (float)5.7751307E37F;
            p91.aux1 = (float)1.6221567E38F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)42835);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)16055);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)43685);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)37066);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)10488);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)13472);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)29075);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)40035);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)54930);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)20739);
                Debug.Assert(pack.time_usec == (ulong)7774815735820478766L);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)14105);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)42140);
                Debug.Assert(pack.rssi == (byte)(byte)86);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan1_raw = (ushort)(ushort)43685;
            p92.chan10_raw = (ushort)(ushort)13472;
            p92.chan8_raw = (ushort)(ushort)40035;
            p92.chan9_raw = (ushort)(ushort)10488;
            p92.chan2_raw = (ushort)(ushort)42835;
            p92.chan11_raw = (ushort)(ushort)54930;
            p92.rssi = (byte)(byte)86;
            p92.chan4_raw = (ushort)(ushort)42140;
            p92.chan5_raw = (ushort)(ushort)20739;
            p92.chan7_raw = (ushort)(ushort)29075;
            p92.chan3_raw = (ushort)(ushort)14105;
            p92.chan6_raw = (ushort)(ushort)16055;
            p92.chan12_raw = (ushort)(ushort)37066;
            p92.time_usec = (ulong)7774815735820478766L;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ulong)796803513312704157L);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_ARMED);
                Debug.Assert(pack.time_usec == (ulong)1575390130862372600L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-3.1035421E38F, -1.7903437E38F, 1.1442458E38F, 1.1934116E38F, 1.596707E38F, -1.3908926E38F, 2.8840599E38F, 2.6527958E38F, -2.8490013E38F, 2.696894E38F, -2.436168E38F, -2.1871256E38F, 1.2106401E38F, 1.9025354E37F, 1.3694219E37F, -1.1913686E37F}));
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.controls_SET(new float[] {-3.1035421E38F, -1.7903437E38F, 1.1442458E38F, 1.1934116E38F, 1.596707E38F, -1.3908926E38F, 2.8840599E38F, 2.6527958E38F, -2.8490013E38F, 2.696894E38F, -2.436168E38F, -2.1871256E38F, 1.2106401E38F, 1.9025354E37F, 1.3694219E37F, -1.1913686E37F}, 0) ;
            p93.time_usec = (ulong)1575390130862372600L;
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_ARMED;
            p93.flags = (ulong)796803513312704157L;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_x == (short)(short) -15035);
                Debug.Assert(pack.ground_distance == (float) -1.7672698E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)82);
                Debug.Assert(pack.flow_comp_m_x == (float)1.534821E37F);
                Debug.Assert(pack.time_usec == (ulong)4416817716013971361L);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)3.3040084E38F);
                Debug.Assert(pack.quality == (byte)(byte)138);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -3.3407729E38F);
                Debug.Assert(pack.flow_comp_m_y == (float)1.6127659E38F);
                Debug.Assert(pack.flow_y == (short)(short) -13460);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_comp_m_y = (float)1.6127659E38F;
            p100.flow_x = (short)(short) -15035;
            p100.sensor_id = (byte)(byte)82;
            p100.flow_rate_x_SET((float)3.3040084E38F, PH) ;
            p100.flow_rate_y_SET((float) -3.3407729E38F, PH) ;
            p100.flow_comp_m_x = (float)1.534821E37F;
            p100.quality = (byte)(byte)138;
            p100.ground_distance = (float) -1.7672698E38F;
            p100.time_usec = (ulong)4416817716013971361L;
            p100.flow_y = (short)(short) -13460;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -3.3192495E37F);
                Debug.Assert(pack.z == (float) -1.6739487E38F);
                Debug.Assert(pack.y == (float)1.1047141E37F);
                Debug.Assert(pack.usec == (ulong)3077205609435687425L);
                Debug.Assert(pack.yaw == (float)3.2917199E38F);
                Debug.Assert(pack.roll == (float)2.9237712E37F);
                Debug.Assert(pack.x == (float) -5.060143E37F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.yaw = (float)3.2917199E38F;
            p101.z = (float) -1.6739487E38F;
            p101.x = (float) -5.060143E37F;
            p101.pitch = (float) -3.3192495E37F;
            p101.roll = (float)2.9237712E37F;
            p101.usec = (ulong)3077205609435687425L;
            p101.y = (float)1.1047141E37F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)8.616803E37F);
                Debug.Assert(pack.yaw == (float) -1.1056772E38F);
                Debug.Assert(pack.x == (float) -2.5747294E38F);
                Debug.Assert(pack.roll == (float) -2.5427637E38F);
                Debug.Assert(pack.usec == (ulong)143236914580976981L);
                Debug.Assert(pack.pitch == (float)2.0302503E38F);
                Debug.Assert(pack.y == (float) -6.2228517E37F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.yaw = (float) -1.1056772E38F;
            p102.z = (float)8.616803E37F;
            p102.x = (float) -2.5747294E38F;
            p102.y = (float) -6.2228517E37F;
            p102.roll = (float) -2.5427637E38F;
            p102.usec = (ulong)143236914580976981L;
            p102.pitch = (float)2.0302503E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)9.135102E37F);
                Debug.Assert(pack.usec == (ulong)8194803847954004334L);
                Debug.Assert(pack.z == (float) -1.7405344E38F);
                Debug.Assert(pack.y == (float) -2.6681398E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.y = (float) -2.6681398E38F;
            p103.usec = (ulong)8194803847954004334L;
            p103.z = (float) -1.7405344E38F;
            p103.x = (float)9.135102E37F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -3.0688718E38F);
                Debug.Assert(pack.x == (float)7.6831395E37F);
                Debug.Assert(pack.roll == (float)9.440321E37F);
                Debug.Assert(pack.z == (float)4.0069797E37F);
                Debug.Assert(pack.yaw == (float) -1.4785068E38F);
                Debug.Assert(pack.usec == (ulong)4794539232531694940L);
                Debug.Assert(pack.pitch == (float)2.0869533E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.pitch = (float)2.0869533E38F;
            p104.usec = (ulong)4794539232531694940L;
            p104.z = (float)4.0069797E37F;
            p104.x = (float)7.6831395E37F;
            p104.yaw = (float) -1.4785068E38F;
            p104.y = (float) -3.0688718E38F;
            p104.roll = (float)9.440321E37F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (float) -5.010812E37F);
                Debug.Assert(pack.xgyro == (float)2.7325123E38F);
                Debug.Assert(pack.zacc == (float)3.2615455E38F);
                Debug.Assert(pack.xacc == (float) -1.515901E38F);
                Debug.Assert(pack.ygyro == (float)7.8287064E37F);
                Debug.Assert(pack.ymag == (float)2.183328E38F);
                Debug.Assert(pack.xmag == (float) -2.4441127E38F);
                Debug.Assert(pack.time_usec == (ulong)326209707442110441L);
                Debug.Assert(pack.pressure_alt == (float)3.0815866E38F);
                Debug.Assert(pack.yacc == (float) -1.6732072E38F);
                Debug.Assert(pack.zgyro == (float) -2.2705902E38F);
                Debug.Assert(pack.diff_pressure == (float)1.7625438E38F);
                Debug.Assert(pack.zmag == (float)3.2043101E37F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)51493);
                Debug.Assert(pack.abs_pressure == (float) -3.089841E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.fields_updated = (ushort)(ushort)51493;
            p105.yacc = (float) -1.6732072E38F;
            p105.zgyro = (float) -2.2705902E38F;
            p105.pressure_alt = (float)3.0815866E38F;
            p105.ygyro = (float)7.8287064E37F;
            p105.abs_pressure = (float) -3.089841E38F;
            p105.time_usec = (ulong)326209707442110441L;
            p105.xacc = (float) -1.515901E38F;
            p105.temperature = (float) -5.010812E37F;
            p105.xgyro = (float)2.7325123E38F;
            p105.ymag = (float)2.183328E38F;
            p105.diff_pressure = (float)1.7625438E38F;
            p105.zacc = (float)3.2615455E38F;
            p105.xmag = (float) -2.4441127E38F;
            p105.zmag = (float)3.2043101E37F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_id == (byte)(byte)51);
                Debug.Assert(pack.quality == (byte)(byte)177);
                Debug.Assert(pack.distance == (float)1.6447398E38F);
                Debug.Assert(pack.time_usec == (ulong)1853698846247079254L);
                Debug.Assert(pack.time_delta_distance_us == (uint)1210116284U);
                Debug.Assert(pack.integrated_x == (float)2.3759368E38F);
                Debug.Assert(pack.integrated_y == (float)1.4167793E37F);
                Debug.Assert(pack.temperature == (short)(short) -15217);
                Debug.Assert(pack.integrated_xgyro == (float)3.3063916E38F);
                Debug.Assert(pack.integration_time_us == (uint)3940741311U);
                Debug.Assert(pack.integrated_zgyro == (float)1.1719028E38F);
                Debug.Assert(pack.integrated_ygyro == (float) -1.9780818E38F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)1853698846247079254L;
            p106.integrated_ygyro = (float) -1.9780818E38F;
            p106.integrated_zgyro = (float)1.1719028E38F;
            p106.integrated_xgyro = (float)3.3063916E38F;
            p106.time_delta_distance_us = (uint)1210116284U;
            p106.temperature = (short)(short) -15217;
            p106.integrated_x = (float)2.3759368E38F;
            p106.quality = (byte)(byte)177;
            p106.integrated_y = (float)1.4167793E37F;
            p106.sensor_id = (byte)(byte)51;
            p106.integration_time_us = (uint)3940741311U;
            p106.distance = (float)1.6447398E38F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.abs_pressure == (float)2.7969506E38F);
                Debug.Assert(pack.zgyro == (float) -1.2601479E38F);
                Debug.Assert(pack.zacc == (float) -2.2829022E38F);
                Debug.Assert(pack.xgyro == (float) -4.7148013E37F);
                Debug.Assert(pack.pressure_alt == (float)5.4758865E37F);
                Debug.Assert(pack.xacc == (float) -1.6207269E38F);
                Debug.Assert(pack.xmag == (float) -2.4812904E38F);
                Debug.Assert(pack.zmag == (float)2.6665162E37F);
                Debug.Assert(pack.fields_updated == (uint)532352214U);
                Debug.Assert(pack.ygyro == (float)8.692806E36F);
                Debug.Assert(pack.ymag == (float) -1.633495E38F);
                Debug.Assert(pack.diff_pressure == (float) -3.0065245E38F);
                Debug.Assert(pack.temperature == (float) -3.1007063E37F);
                Debug.Assert(pack.yacc == (float)1.8945008E38F);
                Debug.Assert(pack.time_usec == (ulong)2255354554410916869L);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.fields_updated = (uint)532352214U;
            p107.temperature = (float) -3.1007063E37F;
            p107.zgyro = (float) -1.2601479E38F;
            p107.time_usec = (ulong)2255354554410916869L;
            p107.yacc = (float)1.8945008E38F;
            p107.zacc = (float) -2.2829022E38F;
            p107.ygyro = (float)8.692806E36F;
            p107.zmag = (float)2.6665162E37F;
            p107.xacc = (float) -1.6207269E38F;
            p107.xgyro = (float) -4.7148013E37F;
            p107.xmag = (float) -2.4812904E38F;
            p107.diff_pressure = (float) -3.0065245E38F;
            p107.ymag = (float) -1.633495E38F;
            p107.abs_pressure = (float)2.7969506E38F;
            p107.pressure_alt = (float)5.4758865E37F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (float) -3.1725041E38F);
                Debug.Assert(pack.ygyro == (float) -5.2798717E37F);
                Debug.Assert(pack.zgyro == (float) -1.9092183E37F);
                Debug.Assert(pack.vd == (float)2.928928E38F);
                Debug.Assert(pack.std_dev_vert == (float) -1.7389158E38F);
                Debug.Assert(pack.lat == (float) -2.9272555E37F);
                Debug.Assert(pack.roll == (float)7.058127E37F);
                Debug.Assert(pack.q3 == (float) -1.8330907E38F);
                Debug.Assert(pack.ve == (float) -3.2618163E38F);
                Debug.Assert(pack.yacc == (float)2.7095454E38F);
                Debug.Assert(pack.q4 == (float) -2.9441804E38F);
                Debug.Assert(pack.xgyro == (float)1.1621231E38F);
                Debug.Assert(pack.zacc == (float) -2.5117584E38F);
                Debug.Assert(pack.pitch == (float) -6.124069E37F);
                Debug.Assert(pack.std_dev_horz == (float)1.3224183E38F);
                Debug.Assert(pack.vn == (float)2.5963782E38F);
                Debug.Assert(pack.yaw == (float) -3.2390976E38F);
                Debug.Assert(pack.alt == (float)1.4323323E38F);
                Debug.Assert(pack.lon == (float) -2.3803904E38F);
                Debug.Assert(pack.q1 == (float) -8.612609E37F);
                Debug.Assert(pack.q2 == (float)1.280107E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.roll = (float)7.058127E37F;
            p108.lat = (float) -2.9272555E37F;
            p108.std_dev_vert = (float) -1.7389158E38F;
            p108.std_dev_horz = (float)1.3224183E38F;
            p108.xgyro = (float)1.1621231E38F;
            p108.pitch = (float) -6.124069E37F;
            p108.ygyro = (float) -5.2798717E37F;
            p108.yacc = (float)2.7095454E38F;
            p108.yaw = (float) -3.2390976E38F;
            p108.zgyro = (float) -1.9092183E37F;
            p108.q3 = (float) -1.8330907E38F;
            p108.zacc = (float) -2.5117584E38F;
            p108.lon = (float) -2.3803904E38F;
            p108.vd = (float)2.928928E38F;
            p108.alt = (float)1.4323323E38F;
            p108.ve = (float) -3.2618163E38F;
            p108.q2 = (float)1.280107E38F;
            p108.q1 = (float) -8.612609E37F;
            p108.q4 = (float) -2.9441804E38F;
            p108.xacc = (float) -3.1725041E38F;
            p108.vn = (float)2.5963782E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.remrssi == (byte)(byte)147);
                Debug.Assert(pack.noise == (byte)(byte)155);
                Debug.Assert(pack.remnoise == (byte)(byte)76);
                Debug.Assert(pack.txbuf == (byte)(byte)133);
                Debug.Assert(pack.rssi == (byte)(byte)250);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)50468);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)49736);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rxerrors = (ushort)(ushort)49736;
            p109.fixed_ = (ushort)(ushort)50468;
            p109.remnoise = (byte)(byte)76;
            p109.rssi = (byte)(byte)250;
            p109.noise = (byte)(byte)155;
            p109.txbuf = (byte)(byte)133;
            p109.remrssi = (byte)(byte)147;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)149, (byte)137, (byte)173, (byte)238, (byte)243, (byte)88, (byte)209, (byte)236, (byte)188, (byte)4, (byte)97, (byte)166, (byte)121, (byte)140, (byte)243, (byte)108, (byte)5, (byte)240, (byte)229, (byte)209, (byte)160, (byte)2, (byte)146, (byte)31, (byte)23, (byte)201, (byte)254, (byte)114, (byte)146, (byte)152, (byte)245, (byte)117, (byte)109, (byte)134, (byte)255, (byte)152, (byte)145, (byte)57, (byte)183, (byte)81, (byte)18, (byte)102, (byte)154, (byte)81, (byte)9, (byte)210, (byte)25, (byte)158, (byte)168, (byte)105, (byte)14, (byte)242, (byte)217, (byte)224, (byte)77, (byte)240, (byte)239, (byte)204, (byte)228, (byte)199, (byte)250, (byte)102, (byte)150, (byte)179, (byte)112, (byte)255, (byte)46, (byte)165, (byte)5, (byte)119, (byte)163, (byte)198, (byte)75, (byte)242, (byte)154, (byte)134, (byte)235, (byte)10, (byte)152, (byte)4, (byte)164, (byte)16, (byte)20, (byte)133, (byte)119, (byte)175, (byte)44, (byte)158, (byte)32, (byte)4, (byte)127, (byte)22, (byte)177, (byte)74, (byte)249, (byte)176, (byte)46, (byte)89, (byte)226, (byte)112, (byte)14, (byte)99, (byte)149, (byte)26, (byte)59, (byte)117, (byte)176, (byte)252, (byte)137, (byte)75, (byte)10, (byte)20, (byte)115, (byte)103, (byte)188, (byte)201, (byte)206, (byte)199, (byte)95, (byte)49, (byte)74, (byte)180, (byte)184, (byte)12, (byte)183, (byte)222, (byte)75, (byte)68, (byte)188, (byte)246, (byte)1, (byte)140, (byte)215, (byte)196, (byte)65, (byte)66, (byte)128, (byte)223, (byte)126, (byte)32, (byte)27, (byte)126, (byte)94, (byte)255, (byte)208, (byte)60, (byte)213, (byte)21, (byte)179, (byte)13, (byte)254, (byte)165, (byte)144, (byte)46, (byte)227, (byte)85, (byte)239, (byte)169, (byte)208, (byte)144, (byte)161, (byte)152, (byte)70, (byte)50, (byte)216, (byte)242, (byte)103, (byte)8, (byte)74, (byte)187, (byte)169, (byte)128, (byte)111, (byte)27, (byte)71, (byte)195, (byte)238, (byte)61, (byte)10, (byte)16, (byte)175, (byte)134, (byte)60, (byte)240, (byte)124, (byte)102, (byte)231, (byte)181, (byte)245, (byte)66, (byte)114, (byte)252, (byte)253, (byte)59, (byte)231, (byte)249, (byte)63, (byte)161, (byte)25, (byte)178, (byte)16, (byte)197, (byte)221, (byte)219, (byte)156, (byte)143, (byte)218, (byte)164, (byte)248, (byte)123, (byte)208, (byte)88, (byte)80, (byte)47, (byte)214, (byte)218, (byte)152, (byte)205, (byte)79, (byte)217, (byte)251, (byte)121, (byte)8, (byte)89, (byte)131, (byte)37, (byte)105, (byte)95, (byte)60, (byte)168, (byte)202, (byte)207, (byte)8, (byte)140, (byte)40, (byte)35, (byte)142, (byte)230, (byte)242, (byte)140, (byte)174, (byte)124, (byte)24, (byte)171, (byte)155, (byte)68, (byte)176, (byte)144, (byte)249, (byte)234, (byte)57}));
                Debug.Assert(pack.target_component == (byte)(byte)199);
                Debug.Assert(pack.target_network == (byte)(byte)181);
                Debug.Assert(pack.target_system == (byte)(byte)171);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.payload_SET(new byte[] {(byte)149, (byte)137, (byte)173, (byte)238, (byte)243, (byte)88, (byte)209, (byte)236, (byte)188, (byte)4, (byte)97, (byte)166, (byte)121, (byte)140, (byte)243, (byte)108, (byte)5, (byte)240, (byte)229, (byte)209, (byte)160, (byte)2, (byte)146, (byte)31, (byte)23, (byte)201, (byte)254, (byte)114, (byte)146, (byte)152, (byte)245, (byte)117, (byte)109, (byte)134, (byte)255, (byte)152, (byte)145, (byte)57, (byte)183, (byte)81, (byte)18, (byte)102, (byte)154, (byte)81, (byte)9, (byte)210, (byte)25, (byte)158, (byte)168, (byte)105, (byte)14, (byte)242, (byte)217, (byte)224, (byte)77, (byte)240, (byte)239, (byte)204, (byte)228, (byte)199, (byte)250, (byte)102, (byte)150, (byte)179, (byte)112, (byte)255, (byte)46, (byte)165, (byte)5, (byte)119, (byte)163, (byte)198, (byte)75, (byte)242, (byte)154, (byte)134, (byte)235, (byte)10, (byte)152, (byte)4, (byte)164, (byte)16, (byte)20, (byte)133, (byte)119, (byte)175, (byte)44, (byte)158, (byte)32, (byte)4, (byte)127, (byte)22, (byte)177, (byte)74, (byte)249, (byte)176, (byte)46, (byte)89, (byte)226, (byte)112, (byte)14, (byte)99, (byte)149, (byte)26, (byte)59, (byte)117, (byte)176, (byte)252, (byte)137, (byte)75, (byte)10, (byte)20, (byte)115, (byte)103, (byte)188, (byte)201, (byte)206, (byte)199, (byte)95, (byte)49, (byte)74, (byte)180, (byte)184, (byte)12, (byte)183, (byte)222, (byte)75, (byte)68, (byte)188, (byte)246, (byte)1, (byte)140, (byte)215, (byte)196, (byte)65, (byte)66, (byte)128, (byte)223, (byte)126, (byte)32, (byte)27, (byte)126, (byte)94, (byte)255, (byte)208, (byte)60, (byte)213, (byte)21, (byte)179, (byte)13, (byte)254, (byte)165, (byte)144, (byte)46, (byte)227, (byte)85, (byte)239, (byte)169, (byte)208, (byte)144, (byte)161, (byte)152, (byte)70, (byte)50, (byte)216, (byte)242, (byte)103, (byte)8, (byte)74, (byte)187, (byte)169, (byte)128, (byte)111, (byte)27, (byte)71, (byte)195, (byte)238, (byte)61, (byte)10, (byte)16, (byte)175, (byte)134, (byte)60, (byte)240, (byte)124, (byte)102, (byte)231, (byte)181, (byte)245, (byte)66, (byte)114, (byte)252, (byte)253, (byte)59, (byte)231, (byte)249, (byte)63, (byte)161, (byte)25, (byte)178, (byte)16, (byte)197, (byte)221, (byte)219, (byte)156, (byte)143, (byte)218, (byte)164, (byte)248, (byte)123, (byte)208, (byte)88, (byte)80, (byte)47, (byte)214, (byte)218, (byte)152, (byte)205, (byte)79, (byte)217, (byte)251, (byte)121, (byte)8, (byte)89, (byte)131, (byte)37, (byte)105, (byte)95, (byte)60, (byte)168, (byte)202, (byte)207, (byte)8, (byte)140, (byte)40, (byte)35, (byte)142, (byte)230, (byte)242, (byte)140, (byte)174, (byte)124, (byte)24, (byte)171, (byte)155, (byte)68, (byte)176, (byte)144, (byte)249, (byte)234, (byte)57}, 0) ;
            p110.target_component = (byte)(byte)199;
            p110.target_system = (byte)(byte)171;
            p110.target_network = (byte)(byte)181;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long)6383357752597933893L);
                Debug.Assert(pack.tc1 == (long) -2726197148169912954L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -2726197148169912954L;
            p111.ts1 = (long)6383357752597933893L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1833219521645274208L);
                Debug.Assert(pack.seq == (uint)3836366083U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)3836366083U;
            p112.time_usec = (ulong)1833219521645274208L;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -631783665);
                Debug.Assert(pack.vel == (ushort)(ushort)61862);
                Debug.Assert(pack.vd == (short)(short) -32325);
                Debug.Assert(pack.fix_type == (byte)(byte)140);
                Debug.Assert(pack.satellites_visible == (byte)(byte)144);
                Debug.Assert(pack.epv == (ushort)(ushort)49947);
                Debug.Assert(pack.time_usec == (ulong)5359726100189774837L);
                Debug.Assert(pack.cog == (ushort)(ushort)11955);
                Debug.Assert(pack.eph == (ushort)(ushort)17302);
                Debug.Assert(pack.alt == (int) -1641819005);
                Debug.Assert(pack.lon == (int)563348415);
                Debug.Assert(pack.ve == (short)(short)17030);
                Debug.Assert(pack.vn == (short)(short) -29313);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.cog = (ushort)(ushort)11955;
            p113.lat = (int) -631783665;
            p113.eph = (ushort)(ushort)17302;
            p113.time_usec = (ulong)5359726100189774837L;
            p113.epv = (ushort)(ushort)49947;
            p113.alt = (int) -1641819005;
            p113.vel = (ushort)(ushort)61862;
            p113.lon = (int)563348415;
            p113.ve = (short)(short)17030;
            p113.vn = (short)(short) -29313;
            p113.vd = (short)(short) -32325;
            p113.satellites_visible = (byte)(byte)144;
            p113.fix_type = (byte)(byte)140;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_x == (float) -1.8812359E38F);
                Debug.Assert(pack.time_usec == (ulong)1211915363650001832L);
                Debug.Assert(pack.integration_time_us == (uint)1187523986U);
                Debug.Assert(pack.integrated_xgyro == (float) -1.356516E38F);
                Debug.Assert(pack.integrated_zgyro == (float) -2.4762914E38F);
                Debug.Assert(pack.distance == (float) -9.007198E36F);
                Debug.Assert(pack.temperature == (short)(short)3923);
                Debug.Assert(pack.integrated_ygyro == (float) -1.4732831E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)202);
                Debug.Assert(pack.integrated_y == (float)2.6678871E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)3966319105U);
                Debug.Assert(pack.quality == (byte)(byte)251);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.quality = (byte)(byte)251;
            p114.integrated_zgyro = (float) -2.4762914E38F;
            p114.integrated_xgyro = (float) -1.356516E38F;
            p114.integrated_y = (float)2.6678871E38F;
            p114.time_usec = (ulong)1211915363650001832L;
            p114.distance = (float) -9.007198E36F;
            p114.integration_time_us = (uint)1187523986U;
            p114.temperature = (short)(short)3923;
            p114.sensor_id = (byte)(byte)202;
            p114.integrated_ygyro = (float) -1.4732831E38F;
            p114.integrated_x = (float) -1.8812359E38F;
            p114.time_delta_distance_us = (uint)3966319105U;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (short)(short) -13600);
                Debug.Assert(pack.xacc == (short)(short) -2992);
                Debug.Assert(pack.rollspeed == (float) -2.1242918E38F);
                Debug.Assert(pack.zacc == (short)(short) -26688);
                Debug.Assert(pack.lat == (int) -1835221230);
                Debug.Assert(pack.time_usec == (ulong)1811544395065416648L);
                Debug.Assert(pack.vx == (short)(short) -14074);
                Debug.Assert(pack.alt == (int) -1400517245);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)25723);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-5.12282E37F, 1.2526223E38F, 9.117564E37F, 4.6123533E37F}));
                Debug.Assert(pack.vz == (short)(short)21767);
                Debug.Assert(pack.yawspeed == (float)2.9335935E37F);
                Debug.Assert(pack.pitchspeed == (float)4.068466E36F);
                Debug.Assert(pack.lon == (int) -471965694);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)33785);
                Debug.Assert(pack.yacc == (short)(short) -8930);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.alt = (int) -1400517245;
            p115.xacc = (short)(short) -2992;
            p115.time_usec = (ulong)1811544395065416648L;
            p115.vx = (short)(short) -14074;
            p115.rollspeed = (float) -2.1242918E38F;
            p115.ind_airspeed = (ushort)(ushort)25723;
            p115.yawspeed = (float)2.9335935E37F;
            p115.attitude_quaternion_SET(new float[] {-5.12282E37F, 1.2526223E38F, 9.117564E37F, 4.6123533E37F}, 0) ;
            p115.zacc = (short)(short) -26688;
            p115.vy = (short)(short) -13600;
            p115.true_airspeed = (ushort)(ushort)33785;
            p115.yacc = (short)(short) -8930;
            p115.lat = (int) -1835221230;
            p115.pitchspeed = (float)4.068466E36F;
            p115.vz = (short)(short)21767;
            p115.lon = (int) -471965694;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (short)(short)26141);
                Debug.Assert(pack.yacc == (short)(short) -24701);
                Debug.Assert(pack.xgyro == (short)(short) -6515);
                Debug.Assert(pack.xmag == (short)(short)14523);
                Debug.Assert(pack.ygyro == (short)(short)19686);
                Debug.Assert(pack.ymag == (short)(short)26867);
                Debug.Assert(pack.time_boot_ms == (uint)2851042621U);
                Debug.Assert(pack.zgyro == (short)(short)13986);
                Debug.Assert(pack.zacc == (short)(short)21957);
                Debug.Assert(pack.xacc == (short)(short)6322);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.xgyro = (short)(short) -6515;
            p116.yacc = (short)(short) -24701;
            p116.zgyro = (short)(short)13986;
            p116.time_boot_ms = (uint)2851042621U;
            p116.zmag = (short)(short)26141;
            p116.zacc = (short)(short)21957;
            p116.ymag = (short)(short)26867;
            p116.xacc = (short)(short)6322;
            p116.xmag = (short)(short)14523;
            p116.ygyro = (short)(short)19686;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start == (ushort)(ushort)47916);
                Debug.Assert(pack.target_component == (byte)(byte)11);
                Debug.Assert(pack.end == (ushort)(ushort)33259);
                Debug.Assert(pack.target_system == (byte)(byte)131);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)33259;
            p117.start = (ushort)(ushort)47916;
            p117.target_component = (byte)(byte)11;
            p117.target_system = (byte)(byte)131;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)6330);
                Debug.Assert(pack.size == (uint)2619358898U);
                Debug.Assert(pack.time_utc == (uint)3775100900U);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)17579);
                Debug.Assert(pack.num_logs == (ushort)(ushort)5167);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.size = (uint)2619358898U;
            p118.last_log_num = (ushort)(ushort)17579;
            p118.time_utc = (uint)3775100900U;
            p118.id = (ushort)(ushort)6330;
            p118.num_logs = (ushort)(ushort)5167;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)107);
                Debug.Assert(pack.ofs == (uint)3077454139U);
                Debug.Assert(pack.target_component == (byte)(byte)123);
                Debug.Assert(pack.count == (uint)1674612393U);
                Debug.Assert(pack.id == (ushort)(ushort)60238);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.id = (ushort)(ushort)60238;
            p119.count = (uint)1674612393U;
            p119.target_component = (byte)(byte)123;
            p119.ofs = (uint)3077454139U;
            p119.target_system = (byte)(byte)107;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)115, (byte)202, (byte)105, (byte)101, (byte)177, (byte)155, (byte)193, (byte)28, (byte)183, (byte)154, (byte)0, (byte)18, (byte)47, (byte)125, (byte)186, (byte)55, (byte)60, (byte)45, (byte)28, (byte)164, (byte)145, (byte)27, (byte)133, (byte)202, (byte)253, (byte)198, (byte)220, (byte)107, (byte)222, (byte)31, (byte)85, (byte)123, (byte)181, (byte)189, (byte)52, (byte)115, (byte)102, (byte)223, (byte)156, (byte)219, (byte)23, (byte)130, (byte)23, (byte)220, (byte)11, (byte)11, (byte)241, (byte)98, (byte)60, (byte)57, (byte)120, (byte)226, (byte)187, (byte)84, (byte)32, (byte)116, (byte)45, (byte)81, (byte)108, (byte)151, (byte)56, (byte)200, (byte)175, (byte)105, (byte)93, (byte)107, (byte)131, (byte)240, (byte)94, (byte)227, (byte)162, (byte)106, (byte)121, (byte)70, (byte)60, (byte)177, (byte)93, (byte)56, (byte)202, (byte)9, (byte)123, (byte)83, (byte)172, (byte)168, (byte)115, (byte)38, (byte)102, (byte)16, (byte)101, (byte)9}));
                Debug.Assert(pack.id == (ushort)(ushort)33563);
                Debug.Assert(pack.count == (byte)(byte)105);
                Debug.Assert(pack.ofs == (uint)1861062190U);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)33563;
            p120.count = (byte)(byte)105;
            p120.ofs = (uint)1861062190U;
            p120.data__SET(new byte[] {(byte)115, (byte)202, (byte)105, (byte)101, (byte)177, (byte)155, (byte)193, (byte)28, (byte)183, (byte)154, (byte)0, (byte)18, (byte)47, (byte)125, (byte)186, (byte)55, (byte)60, (byte)45, (byte)28, (byte)164, (byte)145, (byte)27, (byte)133, (byte)202, (byte)253, (byte)198, (byte)220, (byte)107, (byte)222, (byte)31, (byte)85, (byte)123, (byte)181, (byte)189, (byte)52, (byte)115, (byte)102, (byte)223, (byte)156, (byte)219, (byte)23, (byte)130, (byte)23, (byte)220, (byte)11, (byte)11, (byte)241, (byte)98, (byte)60, (byte)57, (byte)120, (byte)226, (byte)187, (byte)84, (byte)32, (byte)116, (byte)45, (byte)81, (byte)108, (byte)151, (byte)56, (byte)200, (byte)175, (byte)105, (byte)93, (byte)107, (byte)131, (byte)240, (byte)94, (byte)227, (byte)162, (byte)106, (byte)121, (byte)70, (byte)60, (byte)177, (byte)93, (byte)56, (byte)202, (byte)9, (byte)123, (byte)83, (byte)172, (byte)168, (byte)115, (byte)38, (byte)102, (byte)16, (byte)101, (byte)9}, 0) ;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)213);
                Debug.Assert(pack.target_system == (byte)(byte)83);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)213;
            p121.target_system = (byte)(byte)83;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)150);
                Debug.Assert(pack.target_component == (byte)(byte)31);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)150;
            p122.target_component = (byte)(byte)31;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)219);
                Debug.Assert(pack.target_system == (byte)(byte)74);
                Debug.Assert(pack.target_component == (byte)(byte)200);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)222, (byte)168, (byte)142, (byte)170, (byte)149, (byte)104, (byte)129, (byte)40, (byte)154, (byte)55, (byte)62, (byte)192, (byte)42, (byte)219, (byte)238, (byte)213, (byte)103, (byte)161, (byte)230, (byte)49, (byte)217, (byte)82, (byte)132, (byte)62, (byte)173, (byte)43, (byte)66, (byte)12, (byte)126, (byte)165, (byte)184, (byte)29, (byte)248, (byte)4, (byte)218, (byte)155, (byte)240, (byte)130, (byte)169, (byte)22, (byte)232, (byte)132, (byte)157, (byte)201, (byte)117, (byte)234, (byte)42, (byte)198, (byte)121, (byte)106, (byte)156, (byte)244, (byte)123, (byte)164, (byte)195, (byte)64, (byte)103, (byte)48, (byte)254, (byte)244, (byte)60, (byte)193, (byte)23, (byte)152, (byte)219, (byte)174, (byte)231, (byte)150, (byte)247, (byte)151, (byte)175, (byte)43, (byte)7, (byte)47, (byte)66, (byte)222, (byte)68, (byte)129, (byte)167, (byte)189, (byte)159, (byte)193, (byte)43, (byte)222, (byte)47, (byte)78, (byte)31, (byte)250, (byte)52, (byte)69, (byte)49, (byte)18, (byte)170, (byte)50, (byte)32, (byte)169, (byte)162, (byte)251, (byte)212, (byte)173, (byte)48, (byte)142, (byte)155, (byte)37, (byte)118, (byte)208, (byte)84, (byte)32, (byte)9, (byte)116}));
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)74;
            p123.data__SET(new byte[] {(byte)222, (byte)168, (byte)142, (byte)170, (byte)149, (byte)104, (byte)129, (byte)40, (byte)154, (byte)55, (byte)62, (byte)192, (byte)42, (byte)219, (byte)238, (byte)213, (byte)103, (byte)161, (byte)230, (byte)49, (byte)217, (byte)82, (byte)132, (byte)62, (byte)173, (byte)43, (byte)66, (byte)12, (byte)126, (byte)165, (byte)184, (byte)29, (byte)248, (byte)4, (byte)218, (byte)155, (byte)240, (byte)130, (byte)169, (byte)22, (byte)232, (byte)132, (byte)157, (byte)201, (byte)117, (byte)234, (byte)42, (byte)198, (byte)121, (byte)106, (byte)156, (byte)244, (byte)123, (byte)164, (byte)195, (byte)64, (byte)103, (byte)48, (byte)254, (byte)244, (byte)60, (byte)193, (byte)23, (byte)152, (byte)219, (byte)174, (byte)231, (byte)150, (byte)247, (byte)151, (byte)175, (byte)43, (byte)7, (byte)47, (byte)66, (byte)222, (byte)68, (byte)129, (byte)167, (byte)189, (byte)159, (byte)193, (byte)43, (byte)222, (byte)47, (byte)78, (byte)31, (byte)250, (byte)52, (byte)69, (byte)49, (byte)18, (byte)170, (byte)50, (byte)32, (byte)169, (byte)162, (byte)251, (byte)212, (byte)173, (byte)48, (byte)142, (byte)155, (byte)37, (byte)118, (byte)208, (byte)84, (byte)32, (byte)9, (byte)116}, 0) ;
            p123.target_component = (byte)(byte)200;
            p123.len = (byte)(byte)219;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cog == (ushort)(ushort)34573);
                Debug.Assert(pack.vel == (ushort)(ushort)51035);
                Debug.Assert(pack.dgps_age == (uint)1899100674U);
                Debug.Assert(pack.dgps_numch == (byte)(byte)180);
                Debug.Assert(pack.alt == (int) -248468631);
                Debug.Assert(pack.satellites_visible == (byte)(byte)115);
                Debug.Assert(pack.epv == (ushort)(ushort)1087);
                Debug.Assert(pack.lon == (int) -1159469823);
                Debug.Assert(pack.time_usec == (ulong)2238584267014229562L);
                Debug.Assert(pack.lat == (int)969638493);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
                Debug.Assert(pack.eph == (ushort)(ushort)983);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.eph = (ushort)(ushort)983;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS;
            p124.cog = (ushort)(ushort)34573;
            p124.epv = (ushort)(ushort)1087;
            p124.vel = (ushort)(ushort)51035;
            p124.dgps_numch = (byte)(byte)180;
            p124.lat = (int)969638493;
            p124.time_usec = (ulong)2238584267014229562L;
            p124.alt = (int) -248468631;
            p124.dgps_age = (uint)1899100674U;
            p124.lon = (int) -1159469823;
            p124.satellites_visible = (byte)(byte)115;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)64357);
                Debug.Assert(pack.Vservo == (ushort)(ushort)28940);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)64357;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED;
            p125.Vservo = (ushort)(ushort)28940;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baudrate == (uint)4156786088U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)115, (byte)198, (byte)11, (byte)108, (byte)51, (byte)156, (byte)117, (byte)136, (byte)9, (byte)8, (byte)133, (byte)172, (byte)253, (byte)171, (byte)135, (byte)75, (byte)133, (byte)33, (byte)213, (byte)198, (byte)75, (byte)104, (byte)193, (byte)208, (byte)44, (byte)246, (byte)240, (byte)175, (byte)159, (byte)64, (byte)160, (byte)206, (byte)183, (byte)203, (byte)31, (byte)230, (byte)137, (byte)100, (byte)47, (byte)116, (byte)149, (byte)84, (byte)114, (byte)143, (byte)225, (byte)182, (byte)59, (byte)164, (byte)183, (byte)219, (byte)144, (byte)102, (byte)244, (byte)23, (byte)145, (byte)58, (byte)9, (byte)133, (byte)71, (byte)86, (byte)180, (byte)240, (byte)232, (byte)28, (byte)245, (byte)191, (byte)252, (byte)99, (byte)85, (byte)106}));
                Debug.Assert(pack.count == (byte)(byte)135);
                Debug.Assert(pack.timeout == (ushort)(ushort)1466);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND);
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.data__SET(new byte[] {(byte)115, (byte)198, (byte)11, (byte)108, (byte)51, (byte)156, (byte)117, (byte)136, (byte)9, (byte)8, (byte)133, (byte)172, (byte)253, (byte)171, (byte)135, (byte)75, (byte)133, (byte)33, (byte)213, (byte)198, (byte)75, (byte)104, (byte)193, (byte)208, (byte)44, (byte)246, (byte)240, (byte)175, (byte)159, (byte)64, (byte)160, (byte)206, (byte)183, (byte)203, (byte)31, (byte)230, (byte)137, (byte)100, (byte)47, (byte)116, (byte)149, (byte)84, (byte)114, (byte)143, (byte)225, (byte)182, (byte)59, (byte)164, (byte)183, (byte)219, (byte)144, (byte)102, (byte)244, (byte)23, (byte)145, (byte)58, (byte)9, (byte)133, (byte)71, (byte)86, (byte)180, (byte)240, (byte)232, (byte)28, (byte)245, (byte)191, (byte)252, (byte)99, (byte)85, (byte)106}, 0) ;
            p126.timeout = (ushort)(ushort)1466;
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            p126.baudrate = (uint)4156786088U;
            p126.count = (byte)(byte)135;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_c_mm == (int) -683709552);
                Debug.Assert(pack.rtk_rate == (byte)(byte)253);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)6);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2805200826U);
                Debug.Assert(pack.baseline_a_mm == (int) -1421246338);
                Debug.Assert(pack.rtk_health == (byte)(byte)217);
                Debug.Assert(pack.accuracy == (uint)191516829U);
                Debug.Assert(pack.baseline_b_mm == (int) -1520511011);
                Debug.Assert(pack.tow == (uint)2631860602U);
                Debug.Assert(pack.iar_num_hypotheses == (int)1500887141);
                Debug.Assert(pack.wn == (ushort)(ushort)11278);
                Debug.Assert(pack.nsats == (byte)(byte)239);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)150);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.accuracy = (uint)191516829U;
            p127.rtk_rate = (byte)(byte)253;
            p127.rtk_health = (byte)(byte)217;
            p127.baseline_a_mm = (int) -1421246338;
            p127.tow = (uint)2631860602U;
            p127.wn = (ushort)(ushort)11278;
            p127.baseline_b_mm = (int) -1520511011;
            p127.baseline_c_mm = (int) -683709552;
            p127.nsats = (byte)(byte)239;
            p127.time_last_baseline_ms = (uint)2805200826U;
            p127.rtk_receiver_id = (byte)(byte)6;
            p127.iar_num_hypotheses = (int)1500887141;
            p127.baseline_coords_type = (byte)(byte)150;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rtk_rate == (byte)(byte)49);
                Debug.Assert(pack.tow == (uint)471216715U);
                Debug.Assert(pack.accuracy == (uint)340718176U);
                Debug.Assert(pack.time_last_baseline_ms == (uint)528886968U);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)250);
                Debug.Assert(pack.iar_num_hypotheses == (int) -865070596);
                Debug.Assert(pack.baseline_a_mm == (int)285950109);
                Debug.Assert(pack.nsats == (byte)(byte)194);
                Debug.Assert(pack.wn == (ushort)(ushort)3197);
                Debug.Assert(pack.rtk_health == (byte)(byte)6);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)193);
                Debug.Assert(pack.baseline_b_mm == (int) -1737130244);
                Debug.Assert(pack.baseline_c_mm == (int)1828438958);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.baseline_a_mm = (int)285950109;
            p128.time_last_baseline_ms = (uint)528886968U;
            p128.rtk_rate = (byte)(byte)49;
            p128.accuracy = (uint)340718176U;
            p128.baseline_c_mm = (int)1828438958;
            p128.tow = (uint)471216715U;
            p128.rtk_health = (byte)(byte)6;
            p128.rtk_receiver_id = (byte)(byte)193;
            p128.baseline_b_mm = (int) -1737130244;
            p128.baseline_coords_type = (byte)(byte)250;
            p128.wn = (ushort)(ushort)3197;
            p128.nsats = (byte)(byte)194;
            p128.iar_num_hypotheses = (int) -865070596;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short)31260);
                Debug.Assert(pack.ymag == (short)(short) -3337);
                Debug.Assert(pack.yacc == (short)(short)5359);
                Debug.Assert(pack.xmag == (short)(short)25900);
                Debug.Assert(pack.zacc == (short)(short)30518);
                Debug.Assert(pack.zgyro == (short)(short) -14761);
                Debug.Assert(pack.zmag == (short)(short) -1724);
                Debug.Assert(pack.xacc == (short)(short)5967);
                Debug.Assert(pack.xgyro == (short)(short) -28147);
                Debug.Assert(pack.time_boot_ms == (uint)4062824866U);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)4062824866U;
            p129.ygyro = (short)(short)31260;
            p129.xacc = (short)(short)5967;
            p129.xmag = (short)(short)25900;
            p129.zmag = (short)(short) -1724;
            p129.zgyro = (short)(short) -14761;
            p129.zacc = (short)(short)30518;
            p129.ymag = (short)(short) -3337;
            p129.yacc = (short)(short)5359;
            p129.xgyro = (short)(short) -28147;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.height == (ushort)(ushort)39752);
                Debug.Assert(pack.packets == (ushort)(ushort)50197);
                Debug.Assert(pack.width == (ushort)(ushort)9418);
                Debug.Assert(pack.type == (byte)(byte)191);
                Debug.Assert(pack.jpg_quality == (byte)(byte)143);
                Debug.Assert(pack.payload == (byte)(byte)123);
                Debug.Assert(pack.size == (uint)1548766394U);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.payload = (byte)(byte)123;
            p130.size = (uint)1548766394U;
            p130.height = (ushort)(ushort)39752;
            p130.packets = (ushort)(ushort)50197;
            p130.type = (byte)(byte)191;
            p130.width = (ushort)(ushort)9418;
            p130.jpg_quality = (byte)(byte)143;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)94, (byte)216, (byte)117, (byte)238, (byte)238, (byte)58, (byte)157, (byte)41, (byte)16, (byte)165, (byte)166, (byte)139, (byte)255, (byte)35, (byte)183, (byte)138, (byte)27, (byte)215, (byte)88, (byte)21, (byte)161, (byte)81, (byte)67, (byte)219, (byte)81, (byte)222, (byte)33, (byte)105, (byte)225, (byte)63, (byte)130, (byte)170, (byte)43, (byte)159, (byte)170, (byte)72, (byte)223, (byte)194, (byte)226, (byte)8, (byte)128, (byte)242, (byte)197, (byte)0, (byte)65, (byte)117, (byte)204, (byte)95, (byte)124, (byte)41, (byte)180, (byte)175, (byte)125, (byte)251, (byte)242, (byte)147, (byte)98, (byte)189, (byte)213, (byte)201, (byte)224, (byte)234, (byte)233, (byte)234, (byte)186, (byte)135, (byte)40, (byte)185, (byte)191, (byte)125, (byte)156, (byte)61, (byte)22, (byte)136, (byte)0, (byte)92, (byte)248, (byte)23, (byte)27, (byte)81, (byte)38, (byte)204, (byte)0, (byte)192, (byte)38, (byte)95, (byte)178, (byte)5, (byte)201, (byte)128, (byte)251, (byte)116, (byte)36, (byte)167, (byte)102, (byte)247, (byte)239, (byte)137, (byte)176, (byte)141, (byte)179, (byte)217, (byte)9, (byte)105, (byte)218, (byte)193, (byte)35, (byte)148, (byte)108, (byte)23, (byte)65, (byte)135, (byte)247, (byte)152, (byte)219, (byte)103, (byte)234, (byte)142, (byte)2, (byte)240, (byte)255, (byte)159, (byte)197, (byte)7, (byte)43, (byte)139, (byte)119, (byte)207, (byte)232, (byte)234, (byte)27, (byte)18, (byte)245, (byte)104, (byte)218, (byte)128, (byte)3, (byte)240, (byte)13, (byte)208, (byte)79, (byte)249, (byte)93, (byte)241, (byte)37, (byte)184, (byte)84, (byte)65, (byte)20, (byte)101, (byte)49, (byte)229, (byte)157, (byte)21, (byte)181, (byte)183, (byte)240, (byte)2, (byte)125, (byte)28, (byte)22, (byte)67, (byte)232, (byte)195, (byte)134, (byte)97, (byte)185, (byte)51, (byte)223, (byte)54, (byte)210, (byte)149, (byte)223, (byte)35, (byte)239, (byte)37, (byte)24, (byte)238, (byte)98, (byte)68, (byte)40, (byte)187, (byte)210, (byte)179, (byte)42, (byte)82, (byte)69, (byte)1, (byte)133, (byte)100, (byte)192, (byte)30, (byte)143, (byte)231, (byte)83, (byte)155, (byte)105, (byte)114, (byte)14, (byte)215, (byte)57, (byte)97, (byte)199, (byte)172, (byte)161, (byte)102, (byte)21, (byte)158, (byte)213, (byte)83, (byte)222, (byte)166, (byte)255, (byte)43, (byte)140, (byte)68, (byte)157, (byte)165, (byte)250, (byte)59, (byte)51, (byte)162, (byte)42, (byte)3, (byte)190, (byte)27, (byte)146, (byte)67, (byte)106, (byte)58, (byte)21, (byte)29, (byte)243, (byte)240, (byte)209, (byte)5, (byte)244, (byte)104, (byte)14, (byte)144, (byte)118, (byte)21, (byte)187, (byte)117, (byte)127, (byte)62, (byte)241, (byte)219, (byte)57, (byte)253, (byte)54, (byte)206, (byte)49}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)14625);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)94, (byte)216, (byte)117, (byte)238, (byte)238, (byte)58, (byte)157, (byte)41, (byte)16, (byte)165, (byte)166, (byte)139, (byte)255, (byte)35, (byte)183, (byte)138, (byte)27, (byte)215, (byte)88, (byte)21, (byte)161, (byte)81, (byte)67, (byte)219, (byte)81, (byte)222, (byte)33, (byte)105, (byte)225, (byte)63, (byte)130, (byte)170, (byte)43, (byte)159, (byte)170, (byte)72, (byte)223, (byte)194, (byte)226, (byte)8, (byte)128, (byte)242, (byte)197, (byte)0, (byte)65, (byte)117, (byte)204, (byte)95, (byte)124, (byte)41, (byte)180, (byte)175, (byte)125, (byte)251, (byte)242, (byte)147, (byte)98, (byte)189, (byte)213, (byte)201, (byte)224, (byte)234, (byte)233, (byte)234, (byte)186, (byte)135, (byte)40, (byte)185, (byte)191, (byte)125, (byte)156, (byte)61, (byte)22, (byte)136, (byte)0, (byte)92, (byte)248, (byte)23, (byte)27, (byte)81, (byte)38, (byte)204, (byte)0, (byte)192, (byte)38, (byte)95, (byte)178, (byte)5, (byte)201, (byte)128, (byte)251, (byte)116, (byte)36, (byte)167, (byte)102, (byte)247, (byte)239, (byte)137, (byte)176, (byte)141, (byte)179, (byte)217, (byte)9, (byte)105, (byte)218, (byte)193, (byte)35, (byte)148, (byte)108, (byte)23, (byte)65, (byte)135, (byte)247, (byte)152, (byte)219, (byte)103, (byte)234, (byte)142, (byte)2, (byte)240, (byte)255, (byte)159, (byte)197, (byte)7, (byte)43, (byte)139, (byte)119, (byte)207, (byte)232, (byte)234, (byte)27, (byte)18, (byte)245, (byte)104, (byte)218, (byte)128, (byte)3, (byte)240, (byte)13, (byte)208, (byte)79, (byte)249, (byte)93, (byte)241, (byte)37, (byte)184, (byte)84, (byte)65, (byte)20, (byte)101, (byte)49, (byte)229, (byte)157, (byte)21, (byte)181, (byte)183, (byte)240, (byte)2, (byte)125, (byte)28, (byte)22, (byte)67, (byte)232, (byte)195, (byte)134, (byte)97, (byte)185, (byte)51, (byte)223, (byte)54, (byte)210, (byte)149, (byte)223, (byte)35, (byte)239, (byte)37, (byte)24, (byte)238, (byte)98, (byte)68, (byte)40, (byte)187, (byte)210, (byte)179, (byte)42, (byte)82, (byte)69, (byte)1, (byte)133, (byte)100, (byte)192, (byte)30, (byte)143, (byte)231, (byte)83, (byte)155, (byte)105, (byte)114, (byte)14, (byte)215, (byte)57, (byte)97, (byte)199, (byte)172, (byte)161, (byte)102, (byte)21, (byte)158, (byte)213, (byte)83, (byte)222, (byte)166, (byte)255, (byte)43, (byte)140, (byte)68, (byte)157, (byte)165, (byte)250, (byte)59, (byte)51, (byte)162, (byte)42, (byte)3, (byte)190, (byte)27, (byte)146, (byte)67, (byte)106, (byte)58, (byte)21, (byte)29, (byte)243, (byte)240, (byte)209, (byte)5, (byte)244, (byte)104, (byte)14, (byte)144, (byte)118, (byte)21, (byte)187, (byte)117, (byte)127, (byte)62, (byte)241, (byte)219, (byte)57, (byte)253, (byte)54, (byte)206, (byte)49}, 0) ;
            p131.seqnr = (ushort)(ushort)14625;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_distance == (ushort)(ushort)18464);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315);
                Debug.Assert(pack.id == (byte)(byte)9);
                Debug.Assert(pack.current_distance == (ushort)(ushort)51923);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
                Debug.Assert(pack.covariance == (byte)(byte)115);
                Debug.Assert(pack.time_boot_ms == (uint)3761312272U);
                Debug.Assert(pack.max_distance == (ushort)(ushort)28621);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)3761312272U;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p132.current_distance = (ushort)(ushort)51923;
            p132.id = (byte)(byte)9;
            p132.min_distance = (ushort)(ushort)18464;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315;
            p132.max_distance = (ushort)(ushort)28621;
            p132.covariance = (byte)(byte)115;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)49126);
                Debug.Assert(pack.lat == (int) -449137704);
                Debug.Assert(pack.mask == (ulong)8741058190666625336L);
                Debug.Assert(pack.lon == (int)373983818);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -449137704;
            p133.grid_spacing = (ushort)(ushort)49126;
            p133.mask = (ulong)8741058190666625336L;
            p133.lon = (int)373983818;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)623218714);
                Debug.Assert(pack.gridbit == (byte)(byte)181);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)2463);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -15745, (short)30942, (short)14233, (short)11679, (short)3981, (short)27885, (short) -11297, (short) -25064, (short)30377, (short)29148, (short) -22128, (short) -4976, (short) -22306, (short)7283, (short)16881, (short)3706}));
                Debug.Assert(pack.lat == (int)1529189697);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.gridbit = (byte)(byte)181;
            p134.data__SET(new short[] {(short) -15745, (short)30942, (short)14233, (short)11679, (short)3981, (short)27885, (short) -11297, (short) -25064, (short)30377, (short)29148, (short) -22128, (short) -4976, (short) -22306, (short)7283, (short)16881, (short)3706}, 0) ;
            p134.grid_spacing = (ushort)(ushort)2463;
            p134.lat = (int)1529189697;
            p134.lon = (int)623218714;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -524080230);
                Debug.Assert(pack.lon == (int) -1317537373);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -524080230;
            p135.lon = (int) -1317537373;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pending == (ushort)(ushort)12207);
                Debug.Assert(pack.spacing == (ushort)(ushort)19317);
                Debug.Assert(pack.loaded == (ushort)(ushort)12661);
                Debug.Assert(pack.current_height == (float) -8.894643E37F);
                Debug.Assert(pack.lon == (int) -25509871);
                Debug.Assert(pack.terrain_height == (float)1.6693762E38F);
                Debug.Assert(pack.lat == (int)1439745016);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.current_height = (float) -8.894643E37F;
            p136.spacing = (ushort)(ushort)19317;
            p136.pending = (ushort)(ushort)12207;
            p136.lat = (int)1439745016;
            p136.loaded = (ushort)(ushort)12661;
            p136.lon = (int) -25509871;
            p136.terrain_height = (float)1.6693762E38F;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float)2.6105165E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3504154421U);
                Debug.Assert(pack.temperature == (short)(short)18740);
                Debug.Assert(pack.press_abs == (float)2.5969295E38F);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.temperature = (short)(short)18740;
            p137.press_abs = (float)2.5969295E38F;
            p137.time_boot_ms = (uint)3504154421U;
            p137.press_diff = (float)2.6105165E38F;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -3.9796438E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.6275065E38F, -2.8924045E38F, 1.6030936E38F, -1.9766602E38F}));
                Debug.Assert(pack.z == (float) -1.9989309E38F);
                Debug.Assert(pack.time_usec == (ulong)4751226143481439090L);
                Debug.Assert(pack.y == (float)1.9353287E38F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.z = (float) -1.9989309E38F;
            p138.x = (float) -3.9796438E37F;
            p138.time_usec = (ulong)4751226143481439090L;
            p138.q_SET(new float[] {2.6275065E38F, -2.8924045E38F, 1.6030936E38F, -1.9766602E38F}, 0) ;
            p138.y = (float)1.9353287E38F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)8922505449839229247L);
                Debug.Assert(pack.group_mlx == (byte)(byte)51);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.4661594E38F, 6.399655E37F, 1.0512167E38F, 3.7166953E37F, -1.1255602E38F, -1.8935159E38F, -5.13143E37F, -2.5786102E37F}));
                Debug.Assert(pack.target_system == (byte)(byte)245);
                Debug.Assert(pack.target_component == (byte)(byte)229);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_component = (byte)(byte)229;
            p139.group_mlx = (byte)(byte)51;
            p139.target_system = (byte)(byte)245;
            p139.controls_SET(new float[] {1.4661594E38F, 6.399655E37F, 1.0512167E38F, 3.7166953E37F, -1.1255602E38F, -1.8935159E38F, -5.13143E37F, -2.5786102E37F}, 0) ;
            p139.time_usec = (ulong)8922505449839229247L;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)155);
                Debug.Assert(pack.time_usec == (ulong)4149853373996031283L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.5463583E38F, 2.954291E38F, -2.8427612E38F, -1.5388387E38F, 1.8834647E38F, 2.7100765E38F, 8.0382845E37F, 2.2420218E38F}));
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)155;
            p140.time_usec = (ulong)4149853373996031283L;
            p140.controls_SET(new float[] {1.5463583E38F, 2.954291E38F, -2.8427612E38F, -1.5388387E38F, 1.8834647E38F, 2.7100765E38F, 8.0382845E37F, 2.2420218E38F}, 0) ;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_relative == (float) -1.4658106E38F);
                Debug.Assert(pack.bottom_clearance == (float) -2.4194935E38F);
                Debug.Assert(pack.altitude_local == (float) -2.9267674E37F);
                Debug.Assert(pack.altitude_amsl == (float) -2.6870088E38F);
                Debug.Assert(pack.altitude_monotonic == (float) -2.56096E38F);
                Debug.Assert(pack.time_usec == (ulong)6516186921206101048L);
                Debug.Assert(pack.altitude_terrain == (float)2.8215357E38F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_local = (float) -2.9267674E37F;
            p141.time_usec = (ulong)6516186921206101048L;
            p141.altitude_monotonic = (float) -2.56096E38F;
            p141.altitude_amsl = (float) -2.6870088E38F;
            p141.bottom_clearance = (float) -2.4194935E38F;
            p141.altitude_relative = (float) -1.4658106E38F;
            p141.altitude_terrain = (float)2.8215357E38F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.request_id == (byte)(byte)68);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)235, (byte)233, (byte)193, (byte)11, (byte)31, (byte)65, (byte)162, (byte)187, (byte)115, (byte)5, (byte)37, (byte)97, (byte)232, (byte)68, (byte)174, (byte)247, (byte)11, (byte)128, (byte)198, (byte)107, (byte)155, (byte)212, (byte)112, (byte)112, (byte)177, (byte)79, (byte)114, (byte)242, (byte)55, (byte)15, (byte)50, (byte)229, (byte)203, (byte)194, (byte)161, (byte)152, (byte)161, (byte)167, (byte)42, (byte)159, (byte)184, (byte)153, (byte)186, (byte)66, (byte)122, (byte)60, (byte)223, (byte)98, (byte)215, (byte)228, (byte)176, (byte)135, (byte)137, (byte)241, (byte)166, (byte)36, (byte)15, (byte)155, (byte)213, (byte)13, (byte)224, (byte)119, (byte)140, (byte)216, (byte)90, (byte)35, (byte)149, (byte)126, (byte)68, (byte)176, (byte)12, (byte)40, (byte)123, (byte)89, (byte)166, (byte)75, (byte)78, (byte)249, (byte)115, (byte)77, (byte)146, (byte)226, (byte)240, (byte)70, (byte)60, (byte)181, (byte)171, (byte)104, (byte)94, (byte)146, (byte)228, (byte)64, (byte)241, (byte)63, (byte)166, (byte)189, (byte)111, (byte)13, (byte)229, (byte)38, (byte)91, (byte)191, (byte)162, (byte)89, (byte)171, (byte)243, (byte)147, (byte)35, (byte)119, (byte)241, (byte)19, (byte)130, (byte)33, (byte)6, (byte)55, (byte)60, (byte)109, (byte)213, (byte)136, (byte)214}));
                Debug.Assert(pack.transfer_type == (byte)(byte)119);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)217, (byte)96, (byte)63, (byte)58, (byte)133, (byte)103, (byte)4, (byte)182, (byte)106, (byte)60, (byte)128, (byte)251, (byte)157, (byte)42, (byte)151, (byte)147, (byte)149, (byte)137, (byte)9, (byte)158, (byte)234, (byte)34, (byte)116, (byte)36, (byte)159, (byte)30, (byte)191, (byte)78, (byte)13, (byte)141, (byte)45, (byte)136, (byte)128, (byte)138, (byte)128, (byte)250, (byte)190, (byte)137, (byte)159, (byte)204, (byte)7, (byte)53, (byte)135, (byte)101, (byte)122, (byte)84, (byte)3, (byte)57, (byte)104, (byte)82, (byte)64, (byte)101, (byte)198, (byte)60, (byte)22, (byte)138, (byte)68, (byte)246, (byte)244, (byte)190, (byte)1, (byte)62, (byte)105, (byte)26, (byte)180, (byte)103, (byte)3, (byte)148, (byte)114, (byte)248, (byte)182, (byte)50, (byte)173, (byte)102, (byte)184, (byte)237, (byte)55, (byte)241, (byte)191, (byte)119, (byte)188, (byte)222, (byte)22, (byte)225, (byte)13, (byte)43, (byte)172, (byte)60, (byte)203, (byte)59, (byte)176, (byte)218, (byte)51, (byte)225, (byte)248, (byte)191, (byte)34, (byte)103, (byte)156, (byte)251, (byte)182, (byte)204, (byte)230, (byte)177, (byte)45, (byte)224, (byte)45, (byte)118, (byte)118, (byte)54, (byte)196, (byte)59, (byte)241, (byte)169, (byte)15, (byte)104, (byte)228, (byte)69, (byte)61, (byte)7}));
                Debug.Assert(pack.uri_type == (byte)(byte)148);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.transfer_type = (byte)(byte)119;
            p142.request_id = (byte)(byte)68;
            p142.uri_SET(new byte[] {(byte)217, (byte)96, (byte)63, (byte)58, (byte)133, (byte)103, (byte)4, (byte)182, (byte)106, (byte)60, (byte)128, (byte)251, (byte)157, (byte)42, (byte)151, (byte)147, (byte)149, (byte)137, (byte)9, (byte)158, (byte)234, (byte)34, (byte)116, (byte)36, (byte)159, (byte)30, (byte)191, (byte)78, (byte)13, (byte)141, (byte)45, (byte)136, (byte)128, (byte)138, (byte)128, (byte)250, (byte)190, (byte)137, (byte)159, (byte)204, (byte)7, (byte)53, (byte)135, (byte)101, (byte)122, (byte)84, (byte)3, (byte)57, (byte)104, (byte)82, (byte)64, (byte)101, (byte)198, (byte)60, (byte)22, (byte)138, (byte)68, (byte)246, (byte)244, (byte)190, (byte)1, (byte)62, (byte)105, (byte)26, (byte)180, (byte)103, (byte)3, (byte)148, (byte)114, (byte)248, (byte)182, (byte)50, (byte)173, (byte)102, (byte)184, (byte)237, (byte)55, (byte)241, (byte)191, (byte)119, (byte)188, (byte)222, (byte)22, (byte)225, (byte)13, (byte)43, (byte)172, (byte)60, (byte)203, (byte)59, (byte)176, (byte)218, (byte)51, (byte)225, (byte)248, (byte)191, (byte)34, (byte)103, (byte)156, (byte)251, (byte)182, (byte)204, (byte)230, (byte)177, (byte)45, (byte)224, (byte)45, (byte)118, (byte)118, (byte)54, (byte)196, (byte)59, (byte)241, (byte)169, (byte)15, (byte)104, (byte)228, (byte)69, (byte)61, (byte)7}, 0) ;
            p142.storage_SET(new byte[] {(byte)235, (byte)233, (byte)193, (byte)11, (byte)31, (byte)65, (byte)162, (byte)187, (byte)115, (byte)5, (byte)37, (byte)97, (byte)232, (byte)68, (byte)174, (byte)247, (byte)11, (byte)128, (byte)198, (byte)107, (byte)155, (byte)212, (byte)112, (byte)112, (byte)177, (byte)79, (byte)114, (byte)242, (byte)55, (byte)15, (byte)50, (byte)229, (byte)203, (byte)194, (byte)161, (byte)152, (byte)161, (byte)167, (byte)42, (byte)159, (byte)184, (byte)153, (byte)186, (byte)66, (byte)122, (byte)60, (byte)223, (byte)98, (byte)215, (byte)228, (byte)176, (byte)135, (byte)137, (byte)241, (byte)166, (byte)36, (byte)15, (byte)155, (byte)213, (byte)13, (byte)224, (byte)119, (byte)140, (byte)216, (byte)90, (byte)35, (byte)149, (byte)126, (byte)68, (byte)176, (byte)12, (byte)40, (byte)123, (byte)89, (byte)166, (byte)75, (byte)78, (byte)249, (byte)115, (byte)77, (byte)146, (byte)226, (byte)240, (byte)70, (byte)60, (byte)181, (byte)171, (byte)104, (byte)94, (byte)146, (byte)228, (byte)64, (byte)241, (byte)63, (byte)166, (byte)189, (byte)111, (byte)13, (byte)229, (byte)38, (byte)91, (byte)191, (byte)162, (byte)89, (byte)171, (byte)243, (byte)147, (byte)35, (byte)119, (byte)241, (byte)19, (byte)130, (byte)33, (byte)6, (byte)55, (byte)60, (byte)109, (byte)213, (byte)136, (byte)214}, 0) ;
            p142.uri_type = (byte)(byte)148;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float)2.3128979E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2441189545U);
                Debug.Assert(pack.temperature == (short)(short)31869);
                Debug.Assert(pack.press_diff == (float) -2.4368004E37F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.temperature = (short)(short)31869;
            p143.press_abs = (float)2.3128979E38F;
            p143.press_diff = (float) -2.4368004E37F;
            p143.time_boot_ms = (uint)2441189545U;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -77963732);
                Debug.Assert(pack.custom_state == (ulong)8708467960447281635L);
                Debug.Assert(pack.timestamp == (ulong)2707687290277872895L);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {2.7511125E38F, -1.0658229E38F, 1.9461142E38F}));
                Debug.Assert(pack.alt == (float) -6.598934E37F);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-8.709483E37F, 1.4870339E37F, 1.2778239E38F}));
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-1.592727E38F, -3.5765045E37F, -1.9124519E37F, -7.4365924E36F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)136);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {5.271854E37F, -2.6145484E38F, -3.2015816E38F}));
                Debug.Assert(pack.lon == (int) -1242304328);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {-2.21392E38F, -1.8267464E38F, 4.310798E37F}));
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.alt = (float) -6.598934E37F;
            p144.rates_SET(new float[] {-8.709483E37F, 1.4870339E37F, 1.2778239E38F}, 0) ;
            p144.position_cov_SET(new float[] {-2.21392E38F, -1.8267464E38F, 4.310798E37F}, 0) ;
            p144.lat = (int) -77963732;
            p144.est_capabilities = (byte)(byte)136;
            p144.custom_state = (ulong)8708467960447281635L;
            p144.timestamp = (ulong)2707687290277872895L;
            p144.vel_SET(new float[] {2.7511125E38F, -1.0658229E38F, 1.9461142E38F}, 0) ;
            p144.attitude_q_SET(new float[] {-1.592727E38F, -3.5765045E37F, -1.9124519E37F, -7.4365924E36F}, 0) ;
            p144.acc_SET(new float[] {5.271854E37F, -2.6145484E38F, -3.2015816E38F}, 0) ;
            p144.lon = (int) -1242304328;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z_pos == (float) -2.3875495E38F);
                Debug.Assert(pack.y_pos == (float) -1.8593867E38F);
                Debug.Assert(pack.time_usec == (ulong)6918560086113770388L);
                Debug.Assert(pack.roll_rate == (float)1.6537953E38F);
                Debug.Assert(pack.pitch_rate == (float)2.7352052E38F);
                Debug.Assert(pack.y_vel == (float)2.397723E38F);
                Debug.Assert(pack.x_pos == (float)1.0373592E38F);
                Debug.Assert(pack.y_acc == (float) -3.3494172E38F);
                Debug.Assert(pack.z_vel == (float)4.8549264E37F);
                Debug.Assert(pack.yaw_rate == (float) -2.2430477E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-2.8455584E38F, -2.5081617E38F, -1.8630052E38F}));
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {2.6671466E38F, 1.4016018E38F, 1.2230472E38F}));
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.805066E38F, 1.7789472E38F, -3.2037301E38F, -3.1527649E38F}));
                Debug.Assert(pack.z_acc == (float) -2.346868E37F);
                Debug.Assert(pack.airspeed == (float) -3.3803216E38F);
                Debug.Assert(pack.x_vel == (float) -2.4266181E38F);
                Debug.Assert(pack.x_acc == (float)6.779963E37F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)6918560086113770388L;
            p146.q_SET(new float[] {-2.805066E38F, 1.7789472E38F, -3.2037301E38F, -3.1527649E38F}, 0) ;
            p146.pitch_rate = (float)2.7352052E38F;
            p146.y_vel = (float)2.397723E38F;
            p146.vel_variance_SET(new float[] {2.6671466E38F, 1.4016018E38F, 1.2230472E38F}, 0) ;
            p146.z_acc = (float) -2.346868E37F;
            p146.y_acc = (float) -3.3494172E38F;
            p146.z_pos = (float) -2.3875495E38F;
            p146.z_vel = (float)4.8549264E37F;
            p146.x_acc = (float)6.779963E37F;
            p146.pos_variance_SET(new float[] {-2.8455584E38F, -2.5081617E38F, -1.8630052E38F}, 0) ;
            p146.roll_rate = (float)1.6537953E38F;
            p146.x_vel = (float) -2.4266181E38F;
            p146.yaw_rate = (float) -2.2430477E38F;
            p146.airspeed = (float) -3.3803216E38F;
            p146.y_pos = (float) -1.8593867E38F;
            p146.x_pos = (float)1.0373592E38F;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
                Debug.Assert(pack.current_battery == (short)(short) -16111);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)8645, (ushort)47553, (ushort)28705, (ushort)62658, (ushort)1075, (ushort)59609, (ushort)52133, (ushort)1204, (ushort)39594, (ushort)23629}));
                Debug.Assert(pack.current_consumed == (int)342473097);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
                Debug.Assert(pack.energy_consumed == (int)851542187);
                Debug.Assert(pack.id == (byte)(byte)221);
                Debug.Assert(pack.temperature == (short)(short)20650);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 1);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.current_consumed = (int)342473097;
            p147.current_battery = (short)(short) -16111;
            p147.temperature = (short)(short)20650;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO;
            p147.voltages_SET(new ushort[] {(ushort)8645, (ushort)47553, (ushort)28705, (ushort)62658, (ushort)1075, (ushort)59609, (ushort)52133, (ushort)1204, (ushort)39594, (ushort)23629}, 0) ;
            p147.id = (byte)(byte)221;
            p147.energy_consumed = (int)851542187;
            p147.battery_remaining = (sbyte)(sbyte) - 1;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)77, (byte)155, (byte)202, (byte)108, (byte)254, (byte)172, (byte)93, (byte)27}));
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
                Debug.Assert(pack.uid == (ulong)5634211550221713385L);
                Debug.Assert(pack.middleware_sw_version == (uint)2240360932U);
                Debug.Assert(pack.board_version == (uint)342467738U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)224, (byte)173, (byte)229, (byte)187, (byte)236, (byte)113, (byte)250, (byte)35}));
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)115, (byte)42, (byte)64, (byte)65, (byte)99, (byte)25, (byte)200, (byte)189, (byte)193, (byte)173, (byte)74, (byte)53, (byte)59, (byte)227, (byte)154, (byte)158, (byte)76, (byte)15}));
                Debug.Assert(pack.os_sw_version == (uint)2734289335U);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)49811);
                Debug.Assert(pack.flight_sw_version == (uint)1151631347U);
                Debug.Assert(pack.product_id == (ushort)(ushort)41215);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)219, (byte)244, (byte)49, (byte)244, (byte)56, (byte)247, (byte)179, (byte)31}));
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT;
            p148.product_id = (ushort)(ushort)41215;
            p148.flight_sw_version = (uint)1151631347U;
            p148.middleware_sw_version = (uint)2240360932U;
            p148.uid = (ulong)5634211550221713385L;
            p148.board_version = (uint)342467738U;
            p148.uid2_SET(new byte[] {(byte)115, (byte)42, (byte)64, (byte)65, (byte)99, (byte)25, (byte)200, (byte)189, (byte)193, (byte)173, (byte)74, (byte)53, (byte)59, (byte)227, (byte)154, (byte)158, (byte)76, (byte)15}, 0, PH) ;
            p148.os_custom_version_SET(new byte[] {(byte)77, (byte)155, (byte)202, (byte)108, (byte)254, (byte)172, (byte)93, (byte)27}, 0) ;
            p148.vendor_id = (ushort)(ushort)49811;
            p148.flight_custom_version_SET(new byte[] {(byte)219, (byte)244, (byte)49, (byte)244, (byte)56, (byte)247, (byte)179, (byte)31}, 0) ;
            p148.os_sw_version = (uint)2734289335U;
            p148.middleware_custom_version_SET(new byte[] {(byte)224, (byte)173, (byte)229, (byte)187, (byte)236, (byte)113, (byte)250, (byte)35}, 0) ;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.distance == (float)1.1710305E38F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {3.3841527E37F, -2.7570228E38F, 3.2700067E38F, -7.4940983E37F}));
                Debug.Assert(pack.x_TRY(ph) == (float)3.2217437E38F);
                Debug.Assert(pack.angle_x == (float)1.6260043E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.y_TRY(ph) == (float) -8.377167E37F);
                Debug.Assert(pack.angle_y == (float)3.331789E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)159);
                Debug.Assert(pack.target_num == (byte)(byte)96);
                Debug.Assert(pack.time_usec == (ulong)2976452818026955930L);
                Debug.Assert(pack.z_TRY(ph) == (float) -1.5441619E38F);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
                Debug.Assert(pack.size_x == (float) -2.0455855E38F);
                Debug.Assert(pack.size_y == (float)2.9187377E38F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.angle_y = (float)3.331789E38F;
            p149.q_SET(new float[] {3.3841527E37F, -2.7570228E38F, 3.2700067E38F, -7.4940983E37F}, 0, PH) ;
            p149.x_SET((float)3.2217437E38F, PH) ;
            p149.time_usec = (ulong)2976452818026955930L;
            p149.z_SET((float) -1.5441619E38F, PH) ;
            p149.y_SET((float) -8.377167E37F, PH) ;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p149.size_x = (float) -2.0455855E38F;
            p149.position_valid_SET((byte)(byte)159, PH) ;
            p149.distance = (float)1.1710305E38F;
            p149.target_num = (byte)(byte)96;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.size_y = (float)2.9187377E38F;
            p149.angle_x = (float)1.6260043E38F;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_POWERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.adc121_cs1_amp == (float) -2.7140349E38F);
                Debug.Assert(pack.adc121_cs2_amp == (float)2.743349E38F);
                Debug.Assert(pack.adc121_vspb_volt == (float) -3.3925048E38F);
                Debug.Assert(pack.adc121_cspb_amp == (float)3.1691204E38F);
            };
            GroundControl.SENS_POWER p201 = CommunicationChannel.new_SENS_POWER();
            PH.setPack(p201);
            p201.adc121_cs1_amp = (float) -2.7140349E38F;
            p201.adc121_cs2_amp = (float)2.743349E38F;
            p201.adc121_cspb_amp = (float)3.1691204E38F;
            p201.adc121_vspb_volt = (float) -3.3925048E38F;
            CommunicationChannel.instance.send(p201);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_MPPTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mppt_timestamp == (ulong)4585782394314722311L);
                Debug.Assert(pack.mppt3_volt == (float)1.3073081E38F);
                Debug.Assert(pack.mppt1_amp == (float) -3.3022716E37F);
                Debug.Assert(pack.mppt2_status == (byte)(byte)49);
                Debug.Assert(pack.mppt1_pwm == (ushort)(ushort)40641);
                Debug.Assert(pack.mppt1_volt == (float) -9.652879E36F);
                Debug.Assert(pack.mppt3_amp == (float)3.1413944E38F);
                Debug.Assert(pack.mppt2_amp == (float) -2.1909231E38F);
                Debug.Assert(pack.mppt3_pwm == (ushort)(ushort)17169);
                Debug.Assert(pack.mppt2_volt == (float) -2.1221866E38F);
                Debug.Assert(pack.mppt3_status == (byte)(byte)252);
                Debug.Assert(pack.mppt1_status == (byte)(byte)44);
                Debug.Assert(pack.mppt2_pwm == (ushort)(ushort)64546);
            };
            GroundControl.SENS_MPPT p202 = CommunicationChannel.new_SENS_MPPT();
            PH.setPack(p202);
            p202.mppt3_pwm = (ushort)(ushort)17169;
            p202.mppt3_volt = (float)1.3073081E38F;
            p202.mppt3_status = (byte)(byte)252;
            p202.mppt1_volt = (float) -9.652879E36F;
            p202.mppt2_volt = (float) -2.1221866E38F;
            p202.mppt2_amp = (float) -2.1909231E38F;
            p202.mppt2_status = (byte)(byte)49;
            p202.mppt2_pwm = (ushort)(ushort)64546;
            p202.mppt1_amp = (float) -3.3022716E37F;
            p202.mppt3_amp = (float)3.1413944E38F;
            p202.mppt1_status = (byte)(byte)44;
            p202.mppt_timestamp = (ulong)4585782394314722311L;
            p202.mppt1_pwm = (ushort)(ushort)40641;
            CommunicationChannel.instance.send(p202);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnASLCTRL_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aslctrl_mode == (byte)(byte)26);
                Debug.Assert(pack.uRud == (float)1.7300502E38F);
                Debug.Assert(pack.hRef_t == (float)1.0526431E38F);
                Debug.Assert(pack.qRef == (float) -1.8590378E38F);
                Debug.Assert(pack.PitchAngleRef == (float) -3.150795E37F);
                Debug.Assert(pack.SpoilersEngaged == (byte)(byte)92);
                Debug.Assert(pack.timestamp == (ulong)6353182153093295971L);
                Debug.Assert(pack.uElev == (float)1.8352786E38F);
                Debug.Assert(pack.uThrot2 == (float)1.6672608E38F);
                Debug.Assert(pack.YawAngleRef == (float)1.7552828E37F);
                Debug.Assert(pack.rRef == (float) -2.1078979E38F);
                Debug.Assert(pack.hRef == (float)3.2450636E38F);
                Debug.Assert(pack.uAil == (float)2.6154017E38F);
                Debug.Assert(pack.pRef == (float) -1.5227303E38F);
                Debug.Assert(pack.nZ == (float)1.3998711E38F);
                Debug.Assert(pack.AirspeedRef == (float) -2.1475833E38F);
                Debug.Assert(pack.h == (float)2.2531816E38F);
                Debug.Assert(pack.PitchAngle == (float)5.1601117E37F);
                Debug.Assert(pack.RollAngle == (float) -1.9136642E38F);
                Debug.Assert(pack.RollAngleRef == (float)1.8929226E38F);
                Debug.Assert(pack.uThrot == (float) -1.1127043E38F);
                Debug.Assert(pack.r == (float)3.617709E37F);
                Debug.Assert(pack.q == (float) -1.0328589E38F);
                Debug.Assert(pack.YawAngle == (float) -9.924554E37F);
                Debug.Assert(pack.p == (float) -9.210284E37F);
            };
            GroundControl.ASLCTRL_DATA p203 = CommunicationChannel.new_ASLCTRL_DATA();
            PH.setPack(p203);
            p203.aslctrl_mode = (byte)(byte)26;
            p203.h = (float)2.2531816E38F;
            p203.r = (float)3.617709E37F;
            p203.qRef = (float) -1.8590378E38F;
            p203.uThrot2 = (float)1.6672608E38F;
            p203.RollAngle = (float) -1.9136642E38F;
            p203.PitchAngle = (float)5.1601117E37F;
            p203.uThrot = (float) -1.1127043E38F;
            p203.YawAngleRef = (float)1.7552828E37F;
            p203.q = (float) -1.0328589E38F;
            p203.uElev = (float)1.8352786E38F;
            p203.hRef = (float)3.2450636E38F;
            p203.uRud = (float)1.7300502E38F;
            p203.timestamp = (ulong)6353182153093295971L;
            p203.YawAngle = (float) -9.924554E37F;
            p203.uAil = (float)2.6154017E38F;
            p203.SpoilersEngaged = (byte)(byte)92;
            p203.RollAngleRef = (float)1.8929226E38F;
            p203.pRef = (float) -1.5227303E38F;
            p203.AirspeedRef = (float) -2.1475833E38F;
            p203.nZ = (float)1.3998711E38F;
            p203.PitchAngleRef = (float) -3.150795E37F;
            p203.rRef = (float) -2.1078979E38F;
            p203.p = (float) -9.210284E37F;
            p203.hRef_t = (float)1.0526431E38F;
            CommunicationChannel.instance.send(p203);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnASLCTRL_DEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.f_2 == (float) -3.2375695E38F);
                Debug.Assert(pack.f_5 == (float) -1.5717817E38F);
                Debug.Assert(pack.f_1 == (float)1.9316618E38F);
                Debug.Assert(pack.f_6 == (float)2.5853077E38F);
                Debug.Assert(pack.f_8 == (float) -8.405366E37F);
                Debug.Assert(pack.i32_1 == (uint)867027963U);
                Debug.Assert(pack.i8_2 == (byte)(byte)41);
                Debug.Assert(pack.f_7 == (float)1.6187633E38F);
                Debug.Assert(pack.i8_1 == (byte)(byte)108);
                Debug.Assert(pack.f_3 == (float)2.8417875E38F);
                Debug.Assert(pack.f_4 == (float)1.9548274E38F);
            };
            GroundControl.ASLCTRL_DEBUG p204 = CommunicationChannel.new_ASLCTRL_DEBUG();
            PH.setPack(p204);
            p204.f_1 = (float)1.9316618E38F;
            p204.f_4 = (float)1.9548274E38F;
            p204.f_2 = (float) -3.2375695E38F;
            p204.f_3 = (float)2.8417875E38F;
            p204.f_5 = (float) -1.5717817E38F;
            p204.i8_1 = (byte)(byte)108;
            p204.i32_1 = (uint)867027963U;
            p204.f_8 = (float) -8.405366E37F;
            p204.f_6 = (float)2.5853077E38F;
            p204.i8_2 = (byte)(byte)41;
            p204.f_7 = (float)1.6187633E38F;
            CommunicationChannel.instance.send(p204);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnASLUAV_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Servo_status.SequenceEqual(new byte[] {(byte)245, (byte)157, (byte)187, (byte)6, (byte)10, (byte)50, (byte)229, (byte)180}));
                Debug.Assert(pack.LED_status == (byte)(byte)201);
                Debug.Assert(pack.SATCOM_status == (byte)(byte)31);
                Debug.Assert(pack.Motor_rpm == (float) -3.2696834E38F);
            };
            GroundControl.ASLUAV_STATUS p205 = CommunicationChannel.new_ASLUAV_STATUS();
            PH.setPack(p205);
            p205.SATCOM_status = (byte)(byte)31;
            p205.Servo_status_SET(new byte[] {(byte)245, (byte)157, (byte)187, (byte)6, (byte)10, (byte)50, (byte)229, (byte)180}, 0) ;
            p205.Motor_rpm = (float) -3.2696834E38F;
            p205.LED_status = (byte)(byte)201;
            CommunicationChannel.instance.send(p205);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEKF_EXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.timestamp == (ulong)1385312539579594954L);
                Debug.Assert(pack.Windspeed == (float)1.0817127E38F);
                Debug.Assert(pack.beta == (float)3.1359092E38F);
                Debug.Assert(pack.alpha == (float)1.6832487E38F);
                Debug.Assert(pack.WindZ == (float)1.3831105E38F);
                Debug.Assert(pack.Airspeed == (float)1.02138E38F);
                Debug.Assert(pack.WindDir == (float) -1.4303731E38F);
            };
            GroundControl.EKF_EXT p206 = CommunicationChannel.new_EKF_EXT();
            PH.setPack(p206);
            p206.Airspeed = (float)1.02138E38F;
            p206.beta = (float)3.1359092E38F;
            p206.alpha = (float)1.6832487E38F;
            p206.WindZ = (float)1.3831105E38F;
            p206.Windspeed = (float)1.0817127E38F;
            p206.WindDir = (float) -1.4303731E38F;
            p206.timestamp = (ulong)1385312539579594954L;
            CommunicationChannel.instance.send(p206);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnASL_OBCTRLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.timestamp == (ulong)5539124520782995634L);
                Debug.Assert(pack.uAilL == (float)2.0534417E38F);
                Debug.Assert(pack.uAilR == (float) -2.7822172E38F);
                Debug.Assert(pack.uElev == (float) -9.961675E37F);
                Debug.Assert(pack.obctrl_status == (byte)(byte)229);
                Debug.Assert(pack.uThrot2 == (float) -8.507747E37F);
                Debug.Assert(pack.uRud == (float)2.0981727E38F);
                Debug.Assert(pack.uThrot == (float) -1.361552E37F);
            };
            GroundControl.ASL_OBCTRL p207 = CommunicationChannel.new_ASL_OBCTRL();
            PH.setPack(p207);
            p207.obctrl_status = (byte)(byte)229;
            p207.uAilL = (float)2.0534417E38F;
            p207.uElev = (float) -9.961675E37F;
            p207.timestamp = (ulong)5539124520782995634L;
            p207.uAilR = (float) -2.7822172E38F;
            p207.uThrot = (float) -1.361552E37F;
            p207.uRud = (float)2.0981727E38F;
            p207.uThrot2 = (float) -8.507747E37F;
            CommunicationChannel.instance.send(p207);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_ATMOSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Humidity == (float)2.232206E37F);
                Debug.Assert(pack.TempAmbient == (float) -2.8214428E38F);
            };
            GroundControl.SENS_ATMOS p208 = CommunicationChannel.new_SENS_ATMOS();
            PH.setPack(p208);
            p208.TempAmbient = (float) -2.8214428E38F;
            p208.Humidity = (float)2.232206E37F;
            CommunicationChannel.instance.send(p208);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_BATMONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cellvoltage3 == (ushort)(ushort)14172);
                Debug.Assert(pack.cellvoltage1 == (ushort)(ushort)30649);
                Debug.Assert(pack.temperature == (float) -2.2424198E38F);
                Debug.Assert(pack.serialnumber == (ushort)(ushort)28297);
                Debug.Assert(pack.cellvoltage2 == (ushort)(ushort)54994);
                Debug.Assert(pack.cellvoltage6 == (ushort)(ushort)32724);
                Debug.Assert(pack.SoC == (byte)(byte)97);
                Debug.Assert(pack.batterystatus == (ushort)(ushort)24773);
                Debug.Assert(pack.cellvoltage5 == (ushort)(ushort)18642);
                Debug.Assert(pack.voltage == (ushort)(ushort)4125);
                Debug.Assert(pack.hostfetcontrol == (ushort)(ushort)41022);
                Debug.Assert(pack.current == (short)(short)23477);
                Debug.Assert(pack.cellvoltage4 == (ushort)(ushort)58111);
            };
            GroundControl.SENS_BATMON p209 = CommunicationChannel.new_SENS_BATMON();
            PH.setPack(p209);
            p209.cellvoltage6 = (ushort)(ushort)32724;
            p209.cellvoltage4 = (ushort)(ushort)58111;
            p209.serialnumber = (ushort)(ushort)28297;
            p209.current = (short)(short)23477;
            p209.hostfetcontrol = (ushort)(ushort)41022;
            p209.cellvoltage5 = (ushort)(ushort)18642;
            p209.batterystatus = (ushort)(ushort)24773;
            p209.voltage = (ushort)(ushort)4125;
            p209.cellvoltage2 = (ushort)(ushort)54994;
            p209.temperature = (float) -2.2424198E38F;
            p209.cellvoltage1 = (ushort)(ushort)30649;
            p209.cellvoltage3 = (ushort)(ushort)14172;
            p209.SoC = (byte)(byte)97;
            CommunicationChannel.instance.send(p209);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFW_SOARING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.LoiterRadius == (float)1.6316294E38F);
                Debug.Assert(pack.ThermalGSEast == (float)2.8432062E38F);
                Debug.Assert(pack.TSE_dot == (float)2.7048944E38F);
                Debug.Assert(pack.xR == (float)2.4899226E38F);
                Debug.Assert(pack.z1_exp == (float)4.2704613E37F);
                Debug.Assert(pack.timestampModeChanged == (ulong)8232569728858840099L);
                Debug.Assert(pack.xLon == (float) -3.7114553E37F);
                Debug.Assert(pack.xLat == (float) -1.0751467E38F);
                Debug.Assert(pack.DebugVar1 == (float)7.1680373E37F);
                Debug.Assert(pack.xW == (float)1.0080926E38F);
                Debug.Assert(pack.DistToSoarPoint == (float)1.0573408E38F);
                Debug.Assert(pack.VarLon == (float)5.0501496E37F);
                Debug.Assert(pack.LoiterDirection == (float) -1.3434214E38F);
                Debug.Assert(pack.VarLat == (float) -1.849431E38F);
                Debug.Assert(pack.timestamp == (ulong)9110819813368808262L);
                Debug.Assert(pack.valid == (byte)(byte)83);
                Debug.Assert(pack.VarW == (float) -1.5100891E38F);
                Debug.Assert(pack.ControlMode == (byte)(byte)105);
                Debug.Assert(pack.z2_exp == (float) -9.554521E37F);
                Debug.Assert(pack.DebugVar2 == (float) -3.004417E38F);
                Debug.Assert(pack.vSinkExp == (float) -2.9018333E38F);
                Debug.Assert(pack.ThermalGSNorth == (float)1.4819689E38F);
                Debug.Assert(pack.VarR == (float) -1.5664208E38F);
                Debug.Assert(pack.z2_DeltaRoll == (float)8.508448E37F);
                Debug.Assert(pack.z1_LocalUpdraftSpeed == (float) -2.0470748E38F);
            };
            GroundControl.FW_SOARING_DATA p210 = CommunicationChannel.new_FW_SOARING_DATA();
            PH.setPack(p210);
            p210.timestamp = (ulong)9110819813368808262L;
            p210.DebugVar2 = (float) -3.004417E38F;
            p210.z2_exp = (float) -9.554521E37F;
            p210.timestampModeChanged = (ulong)8232569728858840099L;
            p210.vSinkExp = (float) -2.9018333E38F;
            p210.TSE_dot = (float)2.7048944E38F;
            p210.VarLat = (float) -1.849431E38F;
            p210.VarLon = (float)5.0501496E37F;
            p210.xR = (float)2.4899226E38F;
            p210.VarW = (float) -1.5100891E38F;
            p210.valid = (byte)(byte)83;
            p210.ThermalGSNorth = (float)1.4819689E38F;
            p210.ControlMode = (byte)(byte)105;
            p210.z2_DeltaRoll = (float)8.508448E37F;
            p210.DistToSoarPoint = (float)1.0573408E38F;
            p210.xLat = (float) -1.0751467E38F;
            p210.LoiterDirection = (float) -1.3434214E38F;
            p210.DebugVar1 = (float)7.1680373E37F;
            p210.xLon = (float) -3.7114553E37F;
            p210.ThermalGSEast = (float)2.8432062E38F;
            p210.z1_LocalUpdraftSpeed = (float) -2.0470748E38F;
            p210.z1_exp = (float)4.2704613E37F;
            p210.xW = (float)1.0080926E38F;
            p210.VarR = (float) -1.5664208E38F;
            p210.LoiterRadius = (float)1.6316294E38F;
            CommunicationChannel.instance.send(p210);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENSORPOD_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.visensor_rate_3 == (byte)(byte)218);
                Debug.Assert(pack.visensor_rate_4 == (byte)(byte)196);
                Debug.Assert(pack.visensor_rate_1 == (byte)(byte)209);
                Debug.Assert(pack.cpu_temp == (byte)(byte)39);
                Debug.Assert(pack.visensor_rate_2 == (byte)(byte)173);
                Debug.Assert(pack.free_space == (ushort)(ushort)11082);
                Debug.Assert(pack.recording_nodes_count == (byte)(byte)236);
                Debug.Assert(pack.timestamp == (ulong)5508515720069463657L);
            };
            GroundControl.SENSORPOD_STATUS p211 = CommunicationChannel.new_SENSORPOD_STATUS();
            PH.setPack(p211);
            p211.timestamp = (ulong)5508515720069463657L;
            p211.visensor_rate_1 = (byte)(byte)209;
            p211.recording_nodes_count = (byte)(byte)236;
            p211.visensor_rate_2 = (byte)(byte)173;
            p211.visensor_rate_3 = (byte)(byte)218;
            p211.free_space = (ushort)(ushort)11082;
            p211.cpu_temp = (byte)(byte)39;
            p211.visensor_rate_4 = (byte)(byte)196;
            CommunicationChannel.instance.send(p211);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_POWER_BOARDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pwr_brd_mot_l_amp == (float)9.969584E37F);
                Debug.Assert(pack.pwr_brd_status == (byte)(byte)202);
                Debug.Assert(pack.pwr_brd_system_volt == (float)2.6848941E38F);
                Debug.Assert(pack.pwr_brd_servo_2_amp == (float)7.9847524E36F);
                Debug.Assert(pack.pwr_brd_servo_4_amp == (float) -1.1028987E38F);
                Debug.Assert(pack.timestamp == (ulong)7380278403609962083L);
                Debug.Assert(pack.pwr_brd_led_status == (byte)(byte)115);
                Debug.Assert(pack.pwr_brd_mot_r_amp == (float) -3.0038417E38F);
                Debug.Assert(pack.pwr_brd_servo_3_amp == (float)1.8380157E38F);
                Debug.Assert(pack.pwr_brd_servo_1_amp == (float) -1.804217E38F);
                Debug.Assert(pack.pwr_brd_aux_amp == (float) -2.3251785E38F);
                Debug.Assert(pack.pwr_brd_servo_volt == (float)4.401708E36F);
            };
            GroundControl.SENS_POWER_BOARD p212 = CommunicationChannel.new_SENS_POWER_BOARD();
            PH.setPack(p212);
            p212.pwr_brd_mot_r_amp = (float) -3.0038417E38F;
            p212.pwr_brd_mot_l_amp = (float)9.969584E37F;
            p212.pwr_brd_servo_3_amp = (float)1.8380157E38F;
            p212.pwr_brd_led_status = (byte)(byte)115;
            p212.pwr_brd_aux_amp = (float) -2.3251785E38F;
            p212.timestamp = (ulong)7380278403609962083L;
            p212.pwr_brd_servo_volt = (float)4.401708E36F;
            p212.pwr_brd_servo_2_amp = (float)7.9847524E36F;
            p212.pwr_brd_servo_1_amp = (float) -1.804217E38F;
            p212.pwr_brd_servo_4_amp = (float) -1.1028987E38F;
            p212.pwr_brd_system_volt = (float)2.6848941E38F;
            p212.pwr_brd_status = (byte)(byte)202;
            CommunicationChannel.instance.send(p212);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_horiz_accuracy == (float)1.7734174E38F);
                Debug.Assert(pack.time_usec == (ulong)1947831998362137009L);
                Debug.Assert(pack.mag_ratio == (float) -2.9908161E38F);
                Debug.Assert(pack.vel_ratio == (float) -3.3832467E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float)3.629998E37F);
                Debug.Assert(pack.hagl_ratio == (float)1.0910602E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE);
                Debug.Assert(pack.pos_vert_ratio == (float)1.4088708E38F);
                Debug.Assert(pack.tas_ratio == (float) -7.649307E37F);
                Debug.Assert(pack.pos_horiz_ratio == (float)2.0549748E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.tas_ratio = (float) -7.649307E37F;
            p230.pos_vert_accuracy = (float)3.629998E37F;
            p230.pos_horiz_accuracy = (float)1.7734174E38F;
            p230.pos_horiz_ratio = (float)2.0549748E38F;
            p230.time_usec = (ulong)1947831998362137009L;
            p230.vel_ratio = (float) -3.3832467E38F;
            p230.mag_ratio = (float) -2.9908161E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE;
            p230.hagl_ratio = (float)1.0910602E38F;
            p230.pos_vert_ratio = (float)1.4088708E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vert_accuracy == (float)2.0265587E38F);
                Debug.Assert(pack.wind_z == (float)3.2698997E38F);
                Debug.Assert(pack.var_horiz == (float) -2.4239577E35F);
                Debug.Assert(pack.time_usec == (ulong)4790416327850574204L);
                Debug.Assert(pack.horiz_accuracy == (float)2.9594934E38F);
                Debug.Assert(pack.wind_alt == (float) -1.7842313E37F);
                Debug.Assert(pack.wind_x == (float)3.0277895E38F);
                Debug.Assert(pack.wind_y == (float) -4.6631754E37F);
                Debug.Assert(pack.var_vert == (float) -2.671961E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.var_vert = (float) -2.671961E38F;
            p231.wind_x = (float)3.0277895E38F;
            p231.var_horiz = (float) -2.4239577E35F;
            p231.vert_accuracy = (float)2.0265587E38F;
            p231.wind_alt = (float) -1.7842313E37F;
            p231.wind_z = (float)3.2698997E38F;
            p231.horiz_accuracy = (float)2.9594934E38F;
            p231.time_usec = (ulong)4790416327850574204L;
            p231.wind_y = (float) -4.6631754E37F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == (byte)(byte)143);
                Debug.Assert(pack.time_week_ms == (uint)2766750555U);
                Debug.Assert(pack.time_usec == (ulong)7750202804299010953L);
                Debug.Assert(pack.vn == (float)6.082705E37F);
                Debug.Assert(pack.vert_accuracy == (float) -4.396435E37F);
                Debug.Assert(pack.lon == (int)508055237);
                Debug.Assert(pack.hdop == (float)5.7954445E37F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)160);
                Debug.Assert(pack.lat == (int)1730525561);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY);
                Debug.Assert(pack.horiz_accuracy == (float)2.8709615E38F);
                Debug.Assert(pack.vdop == (float) -1.7251427E38F);
                Debug.Assert(pack.speed_accuracy == (float)2.1825009E38F);
                Debug.Assert(pack.ve == (float) -3.5307213E37F);
                Debug.Assert(pack.vd == (float) -2.7405428E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)22);
                Debug.Assert(pack.alt == (float)2.9231467E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)42411);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_week = (ushort)(ushort)42411;
            p232.gps_id = (byte)(byte)22;
            p232.ve = (float) -3.5307213E37F;
            p232.vert_accuracy = (float) -4.396435E37F;
            p232.fix_type = (byte)(byte)143;
            p232.lon = (int)508055237;
            p232.time_usec = (ulong)7750202804299010953L;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;
            p232.lat = (int)1730525561;
            p232.vdop = (float) -1.7251427E38F;
            p232.vn = (float)6.082705E37F;
            p232.vd = (float) -2.7405428E38F;
            p232.alt = (float)2.9231467E38F;
            p232.time_week_ms = (uint)2766750555U;
            p232.speed_accuracy = (float)2.1825009E38F;
            p232.hdop = (float)5.7954445E37F;
            p232.horiz_accuracy = (float)2.8709615E38F;
            p232.satellites_visible = (byte)(byte)160;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)235, (byte)2, (byte)126, (byte)81, (byte)168, (byte)63, (byte)43, (byte)67, (byte)253, (byte)237, (byte)71, (byte)219, (byte)202, (byte)69, (byte)12, (byte)119, (byte)63, (byte)50, (byte)6, (byte)13, (byte)6, (byte)252, (byte)47, (byte)90, (byte)212, (byte)238, (byte)19, (byte)233, (byte)45, (byte)246, (byte)234, (byte)68, (byte)239, (byte)80, (byte)160, (byte)12, (byte)44, (byte)28, (byte)158, (byte)111, (byte)67, (byte)165, (byte)82, (byte)214, (byte)217, (byte)165, (byte)62, (byte)45, (byte)34, (byte)52, (byte)139, (byte)90, (byte)196, (byte)122, (byte)27, (byte)254, (byte)245, (byte)117, (byte)6, (byte)148, (byte)207, (byte)241, (byte)71, (byte)100, (byte)84, (byte)201, (byte)173, (byte)159, (byte)69, (byte)48, (byte)243, (byte)137, (byte)137, (byte)60, (byte)169, (byte)164, (byte)151, (byte)30, (byte)243, (byte)141, (byte)151, (byte)246, (byte)35, (byte)11, (byte)205, (byte)204, (byte)117, (byte)217, (byte)104, (byte)199, (byte)189, (byte)166, (byte)242, (byte)26, (byte)19, (byte)81, (byte)139, (byte)117, (byte)29, (byte)21, (byte)170, (byte)214, (byte)247, (byte)68, (byte)101, (byte)205, (byte)149, (byte)85, (byte)169, (byte)5, (byte)143, (byte)89, (byte)66, (byte)93, (byte)113, (byte)136, (byte)82, (byte)109, (byte)5, (byte)159, (byte)102, (byte)50, (byte)48, (byte)214, (byte)116, (byte)184, (byte)82, (byte)25, (byte)185, (byte)179, (byte)239, (byte)75, (byte)84, (byte)195, (byte)239, (byte)250, (byte)201, (byte)61, (byte)43, (byte)198, (byte)11, (byte)97, (byte)145, (byte)102, (byte)87, (byte)85, (byte)218, (byte)210, (byte)251, (byte)66, (byte)62, (byte)31, (byte)174, (byte)102, (byte)166, (byte)6, (byte)41, (byte)45, (byte)137, (byte)112, (byte)246, (byte)70, (byte)66, (byte)183, (byte)5, (byte)107, (byte)12, (byte)47, (byte)33, (byte)132, (byte)76, (byte)213, (byte)220, (byte)108, (byte)151, (byte)161, (byte)138, (byte)99, (byte)43, (byte)218}));
                Debug.Assert(pack.flags == (byte)(byte)240);
                Debug.Assert(pack.len == (byte)(byte)137);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)240;
            p233.data__SET(new byte[] {(byte)235, (byte)2, (byte)126, (byte)81, (byte)168, (byte)63, (byte)43, (byte)67, (byte)253, (byte)237, (byte)71, (byte)219, (byte)202, (byte)69, (byte)12, (byte)119, (byte)63, (byte)50, (byte)6, (byte)13, (byte)6, (byte)252, (byte)47, (byte)90, (byte)212, (byte)238, (byte)19, (byte)233, (byte)45, (byte)246, (byte)234, (byte)68, (byte)239, (byte)80, (byte)160, (byte)12, (byte)44, (byte)28, (byte)158, (byte)111, (byte)67, (byte)165, (byte)82, (byte)214, (byte)217, (byte)165, (byte)62, (byte)45, (byte)34, (byte)52, (byte)139, (byte)90, (byte)196, (byte)122, (byte)27, (byte)254, (byte)245, (byte)117, (byte)6, (byte)148, (byte)207, (byte)241, (byte)71, (byte)100, (byte)84, (byte)201, (byte)173, (byte)159, (byte)69, (byte)48, (byte)243, (byte)137, (byte)137, (byte)60, (byte)169, (byte)164, (byte)151, (byte)30, (byte)243, (byte)141, (byte)151, (byte)246, (byte)35, (byte)11, (byte)205, (byte)204, (byte)117, (byte)217, (byte)104, (byte)199, (byte)189, (byte)166, (byte)242, (byte)26, (byte)19, (byte)81, (byte)139, (byte)117, (byte)29, (byte)21, (byte)170, (byte)214, (byte)247, (byte)68, (byte)101, (byte)205, (byte)149, (byte)85, (byte)169, (byte)5, (byte)143, (byte)89, (byte)66, (byte)93, (byte)113, (byte)136, (byte)82, (byte)109, (byte)5, (byte)159, (byte)102, (byte)50, (byte)48, (byte)214, (byte)116, (byte)184, (byte)82, (byte)25, (byte)185, (byte)179, (byte)239, (byte)75, (byte)84, (byte)195, (byte)239, (byte)250, (byte)201, (byte)61, (byte)43, (byte)198, (byte)11, (byte)97, (byte)145, (byte)102, (byte)87, (byte)85, (byte)218, (byte)210, (byte)251, (byte)66, (byte)62, (byte)31, (byte)174, (byte)102, (byte)166, (byte)6, (byte)41, (byte)45, (byte)137, (byte)112, (byte)246, (byte)70, (byte)66, (byte)183, (byte)5, (byte)107, (byte)12, (byte)47, (byte)33, (byte)132, (byte)76, (byte)213, (byte)220, (byte)108, (byte)151, (byte)161, (byte)138, (byte)99, (byte)43, (byte)218}, 0) ;
            p233.len = (byte)(byte)137;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_amsl == (short)(short)5355);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)71);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)3705);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)87);
                Debug.Assert(pack.failsafe == (byte)(byte)211);
                Debug.Assert(pack.gps_nsat == (byte)(byte)23);
                Debug.Assert(pack.roll == (short)(short)19185);
                Debug.Assert(pack.airspeed == (byte)(byte)124);
                Debug.Assert(pack.battery_remaining == (byte)(byte)55);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)163);
                Debug.Assert(pack.custom_mode == (uint)4172268502U);
                Debug.Assert(pack.pitch == (short)(short)30880);
                Debug.Assert(pack.longitude == (int) -1926162044);
                Debug.Assert(pack.altitude_sp == (short)(short)18249);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
                Debug.Assert(pack.heading == (ushort)(ushort)22162);
                Debug.Assert(pack.latitude == (int) -1158493563);
                Debug.Assert(pack.groundspeed == (byte)(byte)59);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte) - 16);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)47);
                Debug.Assert(pack.heading_sp == (short)(short)32227);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
                Debug.Assert(pack.wp_num == (byte)(byte)224);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED;
            p234.latitude = (int) -1158493563;
            p234.climb_rate = (sbyte)(sbyte)87;
            p234.heading_sp = (short)(short)32227;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p234.temperature = (sbyte)(sbyte)47;
            p234.pitch = (short)(short)30880;
            p234.custom_mode = (uint)4172268502U;
            p234.altitude_sp = (short)(short)18249;
            p234.heading = (ushort)(ushort)22162;
            p234.airspeed = (byte)(byte)124;
            p234.roll = (short)(short)19185;
            p234.battery_remaining = (byte)(byte)55;
            p234.longitude = (int) -1926162044;
            p234.failsafe = (byte)(byte)211;
            p234.temperature_air = (sbyte)(sbyte) - 16;
            p234.altitude_amsl = (short)(short)5355;
            p234.gps_nsat = (byte)(byte)23;
            p234.groundspeed = (byte)(byte)59;
            p234.wp_distance = (ushort)(ushort)3705;
            p234.airspeed_sp = (byte)(byte)163;
            p234.wp_num = (byte)(byte)224;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p234.throttle = (sbyte)(sbyte)71;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_2 == (uint)971833648U);
                Debug.Assert(pack.time_usec == (ulong)6880297500538283420L);
                Debug.Assert(pack.vibration_x == (float) -2.0213904E38F);
                Debug.Assert(pack.clipping_0 == (uint)1346632881U);
                Debug.Assert(pack.vibration_z == (float)3.3433108E38F);
                Debug.Assert(pack.clipping_1 == (uint)274251832U);
                Debug.Assert(pack.vibration_y == (float)2.7859052E38F);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_2 = (uint)971833648U;
            p241.clipping_0 = (uint)1346632881U;
            p241.vibration_z = (float)3.3433108E38F;
            p241.clipping_1 = (uint)274251832U;
            p241.time_usec = (ulong)6880297500538283420L;
            p241.vibration_y = (float)2.7859052E38F;
            p241.vibration_x = (float) -2.0213904E38F;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)8.0561244E37F);
                Debug.Assert(pack.approach_y == (float) -1.0975143E38F);
                Debug.Assert(pack.x == (float)1.1467431E38F);
                Debug.Assert(pack.altitude == (int) -1574797142);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6379043155090025530L);
                Debug.Assert(pack.approach_x == (float)7.5644884E37F);
                Debug.Assert(pack.latitude == (int) -296760019);
                Debug.Assert(pack.y == (float) -1.6502109E38F);
                Debug.Assert(pack.longitude == (int) -1992132956);
                Debug.Assert(pack.approach_z == (float)4.5276007E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.5534135E37F, 7.1306725E37F, 2.9331534E38F, 2.7969238E38F}));
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.y = (float) -1.6502109E38F;
            p242.x = (float)1.1467431E38F;
            p242.approach_x = (float)7.5644884E37F;
            p242.approach_z = (float)4.5276007E37F;
            p242.time_usec_SET((ulong)6379043155090025530L, PH) ;
            p242.altitude = (int) -1574797142;
            p242.z = (float)8.0561244E37F;
            p242.q_SET(new float[] {-3.5534135E37F, 7.1306725E37F, 2.9331534E38F, 2.7969238E38F}, 0) ;
            p242.latitude = (int) -296760019;
            p242.approach_y = (float) -1.0975143E38F;
            p242.longitude = (int) -1992132956;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)4.0108597E37F);
                Debug.Assert(pack.approach_y == (float) -3.0709594E38F);
                Debug.Assert(pack.altitude == (int) -653045400);
                Debug.Assert(pack.approach_z == (float) -2.4611755E38F);
                Debug.Assert(pack.approach_x == (float)3.451826E37F);
                Debug.Assert(pack.x == (float) -1.3628498E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)714214941038457413L);
                Debug.Assert(pack.y == (float)2.5885168E38F);
                Debug.Assert(pack.latitude == (int)1596679858);
                Debug.Assert(pack.target_system == (byte)(byte)16);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.8475137E38F, 2.976031E37F, -1.1141286E38F, -2.9356352E38F}));
                Debug.Assert(pack.longitude == (int) -1844840395);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)16;
            p243.approach_x = (float)3.451826E37F;
            p243.q_SET(new float[] {1.8475137E38F, 2.976031E37F, -1.1141286E38F, -2.9356352E38F}, 0) ;
            p243.approach_y = (float) -3.0709594E38F;
            p243.latitude = (int)1596679858;
            p243.y = (float)2.5885168E38F;
            p243.altitude = (int) -653045400;
            p243.longitude = (int) -1844840395;
            p243.approach_z = (float) -2.4611755E38F;
            p243.z = (float)4.0108597E37F;
            p243.x = (float) -1.3628498E38F;
            p243.time_usec_SET((ulong)714214941038457413L, PH) ;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)13077);
                Debug.Assert(pack.interval_us == (int)12395845);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int)12395845;
            p244.message_id = (ushort)(ushort)13077;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ver_velocity == (short)(short) -26212);
                Debug.Assert(pack.lat == (int)544235823);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN);
                Debug.Assert(pack.tslc == (byte)(byte)20);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)19756);
                Debug.Assert(pack.squawk == (ushort)(ushort)22840);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.lon == (int)1152012707);
                Debug.Assert(pack.heading == (ushort)(ushort)14325);
                Debug.Assert(pack.ICAO_address == (uint)2505242292U);
                Debug.Assert(pack.altitude == (int)1842525258);
                Debug.Assert(pack.callsign_LEN(ph) == 6);
                Debug.Assert(pack.callsign_TRY(ph).Equals("poupar"));
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_POINT_OBSTACLE);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.squawk = (ushort)(ushort)22840;
            p246.callsign_SET("poupar", PH) ;
            p246.ICAO_address = (uint)2505242292U;
            p246.altitude = (int)1842525258;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_POINT_OBSTACLE;
            p246.heading = (ushort)(ushort)14325;
            p246.tslc = (byte)(byte)20;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN;
            p246.hor_velocity = (ushort)(ushort)19756;
            p246.lon = (int)1152012707;
            p246.lat = (int)544235823;
            p246.ver_velocity = (short)(short) -26212;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
                Debug.Assert(pack.horizontal_minimum_delta == (float) -1.1177714E38F);
                Debug.Assert(pack.time_to_minimum_delta == (float) -3.2141614E38F);
                Debug.Assert(pack.altitude_minimum_delta == (float) -7.840814E36F);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.id == (uint)54730630U);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.altitude_minimum_delta = (float) -7.840814E36F;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL;
            p247.id = (uint)54730630U;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            p247.time_to_minimum_delta = (float) -3.2141614E38F;
            p247.horizontal_minimum_delta = (float) -1.1177714E38F;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)102);
                Debug.Assert(pack.target_system == (byte)(byte)230);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)156, (byte)171, (byte)238, (byte)188, (byte)0, (byte)4, (byte)45, (byte)67, (byte)73, (byte)183, (byte)130, (byte)202, (byte)168, (byte)158, (byte)60, (byte)52, (byte)58, (byte)108, (byte)181, (byte)159, (byte)107, (byte)50, (byte)212, (byte)74, (byte)15, (byte)104, (byte)177, (byte)140, (byte)78, (byte)103, (byte)119, (byte)37, (byte)80, (byte)191, (byte)135, (byte)77, (byte)239, (byte)131, (byte)240, (byte)202, (byte)121, (byte)134, (byte)202, (byte)42, (byte)15, (byte)24, (byte)207, (byte)244, (byte)190, (byte)96, (byte)29, (byte)204, (byte)48, (byte)159, (byte)9, (byte)254, (byte)120, (byte)63, (byte)185, (byte)80, (byte)149, (byte)130, (byte)221, (byte)82, (byte)17, (byte)28, (byte)107, (byte)142, (byte)48, (byte)156, (byte)105, (byte)62, (byte)145, (byte)35, (byte)97, (byte)138, (byte)9, (byte)97, (byte)4, (byte)32, (byte)16, (byte)99, (byte)69, (byte)25, (byte)222, (byte)182, (byte)194, (byte)93, (byte)111, (byte)151, (byte)88, (byte)4, (byte)0, (byte)182, (byte)137, (byte)248, (byte)159, (byte)228, (byte)164, (byte)122, (byte)25, (byte)5, (byte)34, (byte)234, (byte)23, (byte)210, (byte)28, (byte)192, (byte)111, (byte)241, (byte)232, (byte)49, (byte)233, (byte)162, (byte)186, (byte)193, (byte)25, (byte)252, (byte)11, (byte)4, (byte)179, (byte)15, (byte)77, (byte)99, (byte)250, (byte)59, (byte)37, (byte)49, (byte)56, (byte)56, (byte)55, (byte)10, (byte)212, (byte)193, (byte)86, (byte)186, (byte)176, (byte)240, (byte)202, (byte)126, (byte)146, (byte)101, (byte)52, (byte)167, (byte)167, (byte)7, (byte)204, (byte)226, (byte)60, (byte)245, (byte)209, (byte)40, (byte)105, (byte)87, (byte)205, (byte)80, (byte)137, (byte)183, (byte)138, (byte)134, (byte)37, (byte)17, (byte)211, (byte)164, (byte)92, (byte)244, (byte)8, (byte)11, (byte)57, (byte)32, (byte)13, (byte)59, (byte)85, (byte)14, (byte)36, (byte)101, (byte)96, (byte)139, (byte)46, (byte)54, (byte)66, (byte)236, (byte)34, (byte)142, (byte)120, (byte)25, (byte)9, (byte)6, (byte)18, (byte)33, (byte)201, (byte)89, (byte)59, (byte)243, (byte)104, (byte)222, (byte)143, (byte)129, (byte)155, (byte)164, (byte)207, (byte)242, (byte)248, (byte)2, (byte)111, (byte)127, (byte)203, (byte)227, (byte)197, (byte)12, (byte)185, (byte)48, (byte)204, (byte)169, (byte)80, (byte)180, (byte)122, (byte)140, (byte)62, (byte)10, (byte)129, (byte)152, (byte)100, (byte)54, (byte)143, (byte)136, (byte)140, (byte)128, (byte)58, (byte)239, (byte)139, (byte)31, (byte)15, (byte)233, (byte)65, (byte)152, (byte)55, (byte)234, (byte)227, (byte)46, (byte)118, (byte)63, (byte)57, (byte)50, (byte)61, (byte)0, (byte)204, (byte)124, (byte)190}));
                Debug.Assert(pack.message_type == (ushort)(ushort)62977);
                Debug.Assert(pack.target_component == (byte)(byte)253);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.payload_SET(new byte[] {(byte)156, (byte)171, (byte)238, (byte)188, (byte)0, (byte)4, (byte)45, (byte)67, (byte)73, (byte)183, (byte)130, (byte)202, (byte)168, (byte)158, (byte)60, (byte)52, (byte)58, (byte)108, (byte)181, (byte)159, (byte)107, (byte)50, (byte)212, (byte)74, (byte)15, (byte)104, (byte)177, (byte)140, (byte)78, (byte)103, (byte)119, (byte)37, (byte)80, (byte)191, (byte)135, (byte)77, (byte)239, (byte)131, (byte)240, (byte)202, (byte)121, (byte)134, (byte)202, (byte)42, (byte)15, (byte)24, (byte)207, (byte)244, (byte)190, (byte)96, (byte)29, (byte)204, (byte)48, (byte)159, (byte)9, (byte)254, (byte)120, (byte)63, (byte)185, (byte)80, (byte)149, (byte)130, (byte)221, (byte)82, (byte)17, (byte)28, (byte)107, (byte)142, (byte)48, (byte)156, (byte)105, (byte)62, (byte)145, (byte)35, (byte)97, (byte)138, (byte)9, (byte)97, (byte)4, (byte)32, (byte)16, (byte)99, (byte)69, (byte)25, (byte)222, (byte)182, (byte)194, (byte)93, (byte)111, (byte)151, (byte)88, (byte)4, (byte)0, (byte)182, (byte)137, (byte)248, (byte)159, (byte)228, (byte)164, (byte)122, (byte)25, (byte)5, (byte)34, (byte)234, (byte)23, (byte)210, (byte)28, (byte)192, (byte)111, (byte)241, (byte)232, (byte)49, (byte)233, (byte)162, (byte)186, (byte)193, (byte)25, (byte)252, (byte)11, (byte)4, (byte)179, (byte)15, (byte)77, (byte)99, (byte)250, (byte)59, (byte)37, (byte)49, (byte)56, (byte)56, (byte)55, (byte)10, (byte)212, (byte)193, (byte)86, (byte)186, (byte)176, (byte)240, (byte)202, (byte)126, (byte)146, (byte)101, (byte)52, (byte)167, (byte)167, (byte)7, (byte)204, (byte)226, (byte)60, (byte)245, (byte)209, (byte)40, (byte)105, (byte)87, (byte)205, (byte)80, (byte)137, (byte)183, (byte)138, (byte)134, (byte)37, (byte)17, (byte)211, (byte)164, (byte)92, (byte)244, (byte)8, (byte)11, (byte)57, (byte)32, (byte)13, (byte)59, (byte)85, (byte)14, (byte)36, (byte)101, (byte)96, (byte)139, (byte)46, (byte)54, (byte)66, (byte)236, (byte)34, (byte)142, (byte)120, (byte)25, (byte)9, (byte)6, (byte)18, (byte)33, (byte)201, (byte)89, (byte)59, (byte)243, (byte)104, (byte)222, (byte)143, (byte)129, (byte)155, (byte)164, (byte)207, (byte)242, (byte)248, (byte)2, (byte)111, (byte)127, (byte)203, (byte)227, (byte)197, (byte)12, (byte)185, (byte)48, (byte)204, (byte)169, (byte)80, (byte)180, (byte)122, (byte)140, (byte)62, (byte)10, (byte)129, (byte)152, (byte)100, (byte)54, (byte)143, (byte)136, (byte)140, (byte)128, (byte)58, (byte)239, (byte)139, (byte)31, (byte)15, (byte)233, (byte)65, (byte)152, (byte)55, (byte)234, (byte)227, (byte)46, (byte)118, (byte)63, (byte)57, (byte)50, (byte)61, (byte)0, (byte)204, (byte)124, (byte)190}, 0) ;
            p248.target_network = (byte)(byte)102;
            p248.target_system = (byte)(byte)230;
            p248.message_type = (ushort)(ushort)62977;
            p248.target_component = (byte)(byte)253;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)230);
                Debug.Assert(pack.address == (ushort)(ushort)54673);
                Debug.Assert(pack.ver == (byte)(byte)87);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 8, (sbyte) - 101, (sbyte)52, (sbyte)60, (sbyte) - 23, (sbyte)7, (sbyte)54, (sbyte) - 8, (sbyte) - 45, (sbyte) - 121, (sbyte) - 22, (sbyte) - 117, (sbyte) - 41, (sbyte) - 55, (sbyte) - 35, (sbyte)58, (sbyte)79, (sbyte) - 10, (sbyte) - 105, (sbyte)110, (sbyte) - 92, (sbyte) - 20, (sbyte)111, (sbyte)47, (sbyte) - 120, (sbyte) - 72, (sbyte) - 8, (sbyte)19, (sbyte)99, (sbyte)96, (sbyte) - 2, (sbyte) - 15}));
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.type = (byte)(byte)230;
            p249.ver = (byte)(byte)87;
            p249.address = (ushort)(ushort)54673;
            p249.value_SET(new sbyte[] {(sbyte) - 8, (sbyte) - 101, (sbyte)52, (sbyte)60, (sbyte) - 23, (sbyte)7, (sbyte)54, (sbyte) - 8, (sbyte) - 45, (sbyte) - 121, (sbyte) - 22, (sbyte) - 117, (sbyte) - 41, (sbyte) - 55, (sbyte) - 35, (sbyte)58, (sbyte)79, (sbyte) - 10, (sbyte) - 105, (sbyte)110, (sbyte) - 92, (sbyte) - 20, (sbyte)111, (sbyte)47, (sbyte) - 120, (sbyte) - 72, (sbyte) - 8, (sbyte)19, (sbyte)99, (sbyte)96, (sbyte) - 2, (sbyte) - 15}, 0) ;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -9.999103E36F);
                Debug.Assert(pack.x == (float)3.0963127E38F);
                Debug.Assert(pack.name_LEN(ph) == 3);
                Debug.Assert(pack.name_TRY(ph).Equals("uus"));
                Debug.Assert(pack.time_usec == (ulong)2216677175332924185L);
                Debug.Assert(pack.y == (float) -2.6293329E38F);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("uus", PH) ;
            p250.y = (float) -2.6293329E38F;
            p250.time_usec = (ulong)2216677175332924185L;
            p250.x = (float)3.0963127E38F;
            p250.z = (float) -9.999103E36F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float)1.5838654E36F);
                Debug.Assert(pack.time_boot_ms == (uint)3780398912U);
                Debug.Assert(pack.name_LEN(ph) == 2);
                Debug.Assert(pack.name_TRY(ph).Equals("dl"));
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)3780398912U;
            p251.name_SET("dl", PH) ;
            p251.value = (float)1.5838654E36F;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (int)2012059850);
                Debug.Assert(pack.time_boot_ms == (uint)475354479U);
                Debug.Assert(pack.name_LEN(ph) == 4);
                Debug.Assert(pack.name_TRY(ph).Equals("ibfn"));
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)475354479U;
            p252.name_SET("ibfn", PH) ;
            p252.value = (int)2012059850;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 36);
                Debug.Assert(pack.text_TRY(ph).Equals("bviujTibPcbchtsfewcqrjytnqylEmcPhAdo"));
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_ERROR);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_ERROR;
            p253.text_SET("bviujTibPcbchtsfewcqrjytnqylEmcPhAdo", PH) ;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3692922984U);
                Debug.Assert(pack.ind == (byte)(byte)178);
                Debug.Assert(pack.value == (float)2.1117708E38F);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.value = (float)2.1117708E38F;
            p254.time_boot_ms = (uint)3692922984U;
            p254.ind = (byte)(byte)178;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.initial_timestamp == (ulong)4939868778454320543L);
                Debug.Assert(pack.target_component == (byte)(byte)247);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)55, (byte)109, (byte)171, (byte)137, (byte)121, (byte)56, (byte)109, (byte)60, (byte)70, (byte)21, (byte)117, (byte)105, (byte)90, (byte)189, (byte)102, (byte)80, (byte)3, (byte)122, (byte)62, (byte)111, (byte)207, (byte)246, (byte)113, (byte)93, (byte)42, (byte)100, (byte)119, (byte)135, (byte)234, (byte)80, (byte)20, (byte)136}));
                Debug.Assert(pack.target_system == (byte)(byte)206);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.secret_key_SET(new byte[] {(byte)55, (byte)109, (byte)171, (byte)137, (byte)121, (byte)56, (byte)109, (byte)60, (byte)70, (byte)21, (byte)117, (byte)105, (byte)90, (byte)189, (byte)102, (byte)80, (byte)3, (byte)122, (byte)62, (byte)111, (byte)207, (byte)246, (byte)113, (byte)93, (byte)42, (byte)100, (byte)119, (byte)135, (byte)234, (byte)80, (byte)20, (byte)136}, 0) ;
            p256.target_system = (byte)(byte)206;
            p256.target_component = (byte)(byte)247;
            p256.initial_timestamp = (ulong)4939868778454320543L;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_change_ms == (uint)3481628272U);
                Debug.Assert(pack.time_boot_ms == (uint)171386154U);
                Debug.Assert(pack.state == (byte)(byte)249);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)171386154U;
            p257.last_change_ms = (uint)3481628272U;
            p257.state = (byte)(byte)249;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)16);
                Debug.Assert(pack.tune_LEN(ph) == 11);
                Debug.Assert(pack.tune_TRY(ph).Equals("uaueivpugrj"));
                Debug.Assert(pack.target_component == (byte)(byte)148);
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_component = (byte)(byte)148;
            p258.tune_SET("uaueivpugrj", PH) ;
            p258.target_system = (byte)(byte)16;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)64996);
                Debug.Assert(pack.focal_length == (float)1.2864107E38F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)1312);
                Debug.Assert(pack.firmware_version == (uint)4031347914U);
                Debug.Assert(pack.time_boot_ms == (uint)2107298597U);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)52410);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES);
                Debug.Assert(pack.sensor_size_v == (float)3.1014112E38F);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)60, (byte)214, (byte)189, (byte)252, (byte)67, (byte)123, (byte)132, (byte)113, (byte)255, (byte)247, (byte)25, (byte)199, (byte)226, (byte)27, (byte)189, (byte)45, (byte)153, (byte)199, (byte)94, (byte)9, (byte)208, (byte)137, (byte)0, (byte)61, (byte)193, (byte)53, (byte)77, (byte)67, (byte)71, (byte)223, (byte)90, (byte)28}));
                Debug.Assert(pack.lens_id == (byte)(byte)143);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 65);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("mZrxkjNxatpqxodznuxfxnppumvkvgowcpmnjrauzfroJjtwddueeafbzqxxSqsdp"));
                Debug.Assert(pack.sensor_size_h == (float)1.1238168E37F);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)115, (byte)43, (byte)46, (byte)131, (byte)11, (byte)101, (byte)15, (byte)79, (byte)45, (byte)17, (byte)219, (byte)50, (byte)227, (byte)152, (byte)8, (byte)82, (byte)165, (byte)63, (byte)118, (byte)188, (byte)112, (byte)156, (byte)210, (byte)122, (byte)57, (byte)30, (byte)118, (byte)54, (byte)73, (byte)253, (byte)155, (byte)166}));
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES;
            p259.sensor_size_h = (float)1.1238168E37F;
            p259.resolution_v = (ushort)(ushort)64996;
            p259.time_boot_ms = (uint)2107298597U;
            p259.vendor_name_SET(new byte[] {(byte)115, (byte)43, (byte)46, (byte)131, (byte)11, (byte)101, (byte)15, (byte)79, (byte)45, (byte)17, (byte)219, (byte)50, (byte)227, (byte)152, (byte)8, (byte)82, (byte)165, (byte)63, (byte)118, (byte)188, (byte)112, (byte)156, (byte)210, (byte)122, (byte)57, (byte)30, (byte)118, (byte)54, (byte)73, (byte)253, (byte)155, (byte)166}, 0) ;
            p259.firmware_version = (uint)4031347914U;
            p259.lens_id = (byte)(byte)143;
            p259.cam_definition_version = (ushort)(ushort)52410;
            p259.cam_definition_uri_SET("mZrxkjNxatpqxodznuxfxnppumvkvgowcpmnjrauzfroJjtwddueeafbzqxxSqsdp", PH) ;
            p259.focal_length = (float)1.2864107E38F;
            p259.sensor_size_v = (float)3.1014112E38F;
            p259.resolution_h = (ushort)(ushort)1312;
            p259.model_name_SET(new byte[] {(byte)60, (byte)214, (byte)189, (byte)252, (byte)67, (byte)123, (byte)132, (byte)113, (byte)255, (byte)247, (byte)25, (byte)199, (byte)226, (byte)27, (byte)189, (byte)45, (byte)153, (byte)199, (byte)94, (byte)9, (byte)208, (byte)137, (byte)0, (byte)61, (byte)193, (byte)53, (byte)77, (byte)67, (byte)71, (byte)223, (byte)90, (byte)28}, 0) ;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)680042229U);
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY;
            p260.time_boot_ms = (uint)680042229U;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.write_speed == (float) -9.454954E37F);
                Debug.Assert(pack.available_capacity == (float) -1.8284821E38F);
                Debug.Assert(pack.read_speed == (float) -7.4662836E37F);
                Debug.Assert(pack.storage_id == (byte)(byte)114);
                Debug.Assert(pack.total_capacity == (float)1.9557918E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4201512253U);
                Debug.Assert(pack.storage_count == (byte)(byte)124);
                Debug.Assert(pack.used_capacity == (float)2.1424243E37F);
                Debug.Assert(pack.status == (byte)(byte)32);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.read_speed = (float) -7.4662836E37F;
            p261.used_capacity = (float)2.1424243E37F;
            p261.write_speed = (float) -9.454954E37F;
            p261.total_capacity = (float)1.9557918E38F;
            p261.storage_count = (byte)(byte)124;
            p261.status = (byte)(byte)32;
            p261.time_boot_ms = (uint)4201512253U;
            p261.available_capacity = (float) -1.8284821E38F;
            p261.storage_id = (byte)(byte)114;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.image_interval == (float)2.4726567E38F);
                Debug.Assert(pack.video_status == (byte)(byte)117);
                Debug.Assert(pack.available_capacity == (float) -5.3741307E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2757910U);
                Debug.Assert(pack.image_status == (byte)(byte)107);
                Debug.Assert(pack.recording_time_ms == (uint)323557706U);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)2757910U;
            p262.recording_time_ms = (uint)323557706U;
            p262.video_status = (byte)(byte)117;
            p262.image_interval = (float)2.4726567E38F;
            p262.available_capacity = (float) -5.3741307E37F;
            p262.image_status = (byte)(byte)107;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)12);
                Debug.Assert(pack.time_utc == (ulong)1990306558103924895L);
                Debug.Assert(pack.image_index == (int)1767681364);
                Debug.Assert(pack.relative_alt == (int)1082996897);
                Debug.Assert(pack.file_url_LEN(ph) == 54);
                Debug.Assert(pack.file_url_TRY(ph).Equals("YhqwepbyOmitnotpwnleldjuncqabtdviVmehdiWhWuozCvkmbwmyk"));
                Debug.Assert(pack.lon == (int)220154086);
                Debug.Assert(pack.q.SequenceEqual(new float[] {4.887521E37F, 2.931349E37F, 2.4708895E38F, 2.6372497E38F}));
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 27);
                Debug.Assert(pack.lat == (int) -1774227795);
                Debug.Assert(pack.alt == (int)451589163);
                Debug.Assert(pack.time_boot_ms == (uint)1571494170U);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)1571494170U;
            p263.camera_id = (byte)(byte)12;
            p263.lon = (int)220154086;
            p263.capture_result = (sbyte)(sbyte) - 27;
            p263.alt = (int)451589163;
            p263.time_utc = (ulong)1990306558103924895L;
            p263.file_url_SET("YhqwepbyOmitnotpwnleldjuncqabtdviVmehdiWhWuozCvkmbwmyk", PH) ;
            p263.relative_alt = (int)1082996897;
            p263.lat = (int) -1774227795;
            p263.q_SET(new float[] {4.887521E37F, 2.931349E37F, 2.4708895E38F, 2.6372497E38F}, 0) ;
            p263.image_index = (int)1767681364;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1693081888U);
                Debug.Assert(pack.flight_uuid == (ulong)2350045259085179717L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)5295387010717054600L);
                Debug.Assert(pack.arming_time_utc == (ulong)5219580073205841037L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.arming_time_utc = (ulong)5219580073205841037L;
            p264.flight_uuid = (ulong)2350045259085179717L;
            p264.takeoff_time_utc = (ulong)5295387010717054600L;
            p264.time_boot_ms = (uint)1693081888U;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -5.362393E36F);
                Debug.Assert(pack.time_boot_ms == (uint)3241484921U);
                Debug.Assert(pack.pitch == (float)1.7937278E38F);
                Debug.Assert(pack.roll == (float) -1.0219376E38F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)3241484921U;
            p265.pitch = (float)1.7937278E38F;
            p265.yaw = (float) -5.362393E36F;
            p265.roll = (float) -1.0219376E38F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)161, (byte)225, (byte)56, (byte)224, (byte)33, (byte)68, (byte)153, (byte)191, (byte)68, (byte)148, (byte)102, (byte)193, (byte)212, (byte)130, (byte)196, (byte)11, (byte)37, (byte)17, (byte)243, (byte)171, (byte)65, (byte)24, (byte)179, (byte)201, (byte)224, (byte)250, (byte)144, (byte)150, (byte)253, (byte)221, (byte)78, (byte)56, (byte)147, (byte)109, (byte)226, (byte)214, (byte)12, (byte)93, (byte)103, (byte)195, (byte)227, (byte)116, (byte)133, (byte)16, (byte)197, (byte)112, (byte)69, (byte)73, (byte)228, (byte)203, (byte)103, (byte)248, (byte)56, (byte)1, (byte)1, (byte)71, (byte)191, (byte)10, (byte)28, (byte)121, (byte)188, (byte)236, (byte)214, (byte)70, (byte)52, (byte)149, (byte)100, (byte)68, (byte)80, (byte)220, (byte)195, (byte)5, (byte)101, (byte)203, (byte)13, (byte)70, (byte)144, (byte)44, (byte)140, (byte)141, (byte)180, (byte)53, (byte)214, (byte)181, (byte)117, (byte)145, (byte)86, (byte)69, (byte)97, (byte)186, (byte)9, (byte)142, (byte)182, (byte)61, (byte)109, (byte)142, (byte)134, (byte)51, (byte)73, (byte)130, (byte)135, (byte)237, (byte)109, (byte)9, (byte)64, (byte)220, (byte)55, (byte)105, (byte)16, (byte)235, (byte)234, (byte)46, (byte)155, (byte)188, (byte)52, (byte)14, (byte)171, (byte)50, (byte)125, (byte)209, (byte)132, (byte)97, (byte)169, (byte)141, (byte)34, (byte)19, (byte)134, (byte)194, (byte)88, (byte)154, (byte)230, (byte)42, (byte)30, (byte)110, (byte)76, (byte)13, (byte)189, (byte)50, (byte)83, (byte)92, (byte)204, (byte)50, (byte)115, (byte)130, (byte)84, (byte)224, (byte)175, (byte)237, (byte)68, (byte)94, (byte)177, (byte)244, (byte)211, (byte)124, (byte)176, (byte)140, (byte)187, (byte)243, (byte)87, (byte)217, (byte)170, (byte)223, (byte)31, (byte)238, (byte)45, (byte)203, (byte)135, (byte)29, (byte)89, (byte)45, (byte)225, (byte)167, (byte)121, (byte)156, (byte)158, (byte)46, (byte)200, (byte)129, (byte)131, (byte)118, (byte)200, (byte)101, (byte)144, (byte)98, (byte)47, (byte)155, (byte)170, (byte)50, (byte)202, (byte)87, (byte)85, (byte)34, (byte)35, (byte)36, (byte)253, (byte)248, (byte)61, (byte)111, (byte)154, (byte)157, (byte)86, (byte)20, (byte)247, (byte)81, (byte)211, (byte)190, (byte)72, (byte)21, (byte)138, (byte)147, (byte)12, (byte)215, (byte)131, (byte)121, (byte)50, (byte)170, (byte)135, (byte)155, (byte)44, (byte)218, (byte)52, (byte)134, (byte)124, (byte)152, (byte)13, (byte)255, (byte)30, (byte)179, (byte)252, (byte)102, (byte)59, (byte)210, (byte)40, (byte)229, (byte)241, (byte)200, (byte)216, (byte)114, (byte)41, (byte)96, (byte)22, (byte)183, (byte)53, (byte)44, (byte)11, (byte)81, (byte)170, (byte)52, (byte)93}));
                Debug.Assert(pack.first_message_offset == (byte)(byte)164);
                Debug.Assert(pack.target_system == (byte)(byte)15);
                Debug.Assert(pack.length == (byte)(byte)210);
                Debug.Assert(pack.sequence == (ushort)(ushort)32865);
                Debug.Assert(pack.target_component == (byte)(byte)226);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_component = (byte)(byte)226;
            p266.first_message_offset = (byte)(byte)164;
            p266.target_system = (byte)(byte)15;
            p266.length = (byte)(byte)210;
            p266.sequence = (ushort)(ushort)32865;
            p266.data__SET(new byte[] {(byte)161, (byte)225, (byte)56, (byte)224, (byte)33, (byte)68, (byte)153, (byte)191, (byte)68, (byte)148, (byte)102, (byte)193, (byte)212, (byte)130, (byte)196, (byte)11, (byte)37, (byte)17, (byte)243, (byte)171, (byte)65, (byte)24, (byte)179, (byte)201, (byte)224, (byte)250, (byte)144, (byte)150, (byte)253, (byte)221, (byte)78, (byte)56, (byte)147, (byte)109, (byte)226, (byte)214, (byte)12, (byte)93, (byte)103, (byte)195, (byte)227, (byte)116, (byte)133, (byte)16, (byte)197, (byte)112, (byte)69, (byte)73, (byte)228, (byte)203, (byte)103, (byte)248, (byte)56, (byte)1, (byte)1, (byte)71, (byte)191, (byte)10, (byte)28, (byte)121, (byte)188, (byte)236, (byte)214, (byte)70, (byte)52, (byte)149, (byte)100, (byte)68, (byte)80, (byte)220, (byte)195, (byte)5, (byte)101, (byte)203, (byte)13, (byte)70, (byte)144, (byte)44, (byte)140, (byte)141, (byte)180, (byte)53, (byte)214, (byte)181, (byte)117, (byte)145, (byte)86, (byte)69, (byte)97, (byte)186, (byte)9, (byte)142, (byte)182, (byte)61, (byte)109, (byte)142, (byte)134, (byte)51, (byte)73, (byte)130, (byte)135, (byte)237, (byte)109, (byte)9, (byte)64, (byte)220, (byte)55, (byte)105, (byte)16, (byte)235, (byte)234, (byte)46, (byte)155, (byte)188, (byte)52, (byte)14, (byte)171, (byte)50, (byte)125, (byte)209, (byte)132, (byte)97, (byte)169, (byte)141, (byte)34, (byte)19, (byte)134, (byte)194, (byte)88, (byte)154, (byte)230, (byte)42, (byte)30, (byte)110, (byte)76, (byte)13, (byte)189, (byte)50, (byte)83, (byte)92, (byte)204, (byte)50, (byte)115, (byte)130, (byte)84, (byte)224, (byte)175, (byte)237, (byte)68, (byte)94, (byte)177, (byte)244, (byte)211, (byte)124, (byte)176, (byte)140, (byte)187, (byte)243, (byte)87, (byte)217, (byte)170, (byte)223, (byte)31, (byte)238, (byte)45, (byte)203, (byte)135, (byte)29, (byte)89, (byte)45, (byte)225, (byte)167, (byte)121, (byte)156, (byte)158, (byte)46, (byte)200, (byte)129, (byte)131, (byte)118, (byte)200, (byte)101, (byte)144, (byte)98, (byte)47, (byte)155, (byte)170, (byte)50, (byte)202, (byte)87, (byte)85, (byte)34, (byte)35, (byte)36, (byte)253, (byte)248, (byte)61, (byte)111, (byte)154, (byte)157, (byte)86, (byte)20, (byte)247, (byte)81, (byte)211, (byte)190, (byte)72, (byte)21, (byte)138, (byte)147, (byte)12, (byte)215, (byte)131, (byte)121, (byte)50, (byte)170, (byte)135, (byte)155, (byte)44, (byte)218, (byte)52, (byte)134, (byte)124, (byte)152, (byte)13, (byte)255, (byte)30, (byte)179, (byte)252, (byte)102, (byte)59, (byte)210, (byte)40, (byte)229, (byte)241, (byte)200, (byte)216, (byte)114, (byte)41, (byte)96, (byte)22, (byte)183, (byte)53, (byte)44, (byte)11, (byte)81, (byte)170, (byte)52, (byte)93}, 0) ;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.first_message_offset == (byte)(byte)150);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)21, (byte)3, (byte)22, (byte)4, (byte)218, (byte)148, (byte)29, (byte)206, (byte)159, (byte)138, (byte)112, (byte)123, (byte)100, (byte)233, (byte)211, (byte)123, (byte)134, (byte)0, (byte)179, (byte)116, (byte)211, (byte)24, (byte)189, (byte)129, (byte)4, (byte)205, (byte)104, (byte)69, (byte)141, (byte)163, (byte)189, (byte)192, (byte)41, (byte)110, (byte)221, (byte)208, (byte)89, (byte)190, (byte)19, (byte)207, (byte)64, (byte)239, (byte)205, (byte)126, (byte)182, (byte)139, (byte)222, (byte)33, (byte)179, (byte)192, (byte)172, (byte)47, (byte)178, (byte)112, (byte)107, (byte)61, (byte)191, (byte)144, (byte)110, (byte)57, (byte)226, (byte)94, (byte)150, (byte)233, (byte)41, (byte)179, (byte)171, (byte)189, (byte)144, (byte)169, (byte)202, (byte)221, (byte)62, (byte)219, (byte)98, (byte)32, (byte)42, (byte)10, (byte)125, (byte)36, (byte)108, (byte)227, (byte)215, (byte)124, (byte)16, (byte)163, (byte)154, (byte)77, (byte)97, (byte)78, (byte)1, (byte)7, (byte)211, (byte)37, (byte)203, (byte)220, (byte)179, (byte)121, (byte)0, (byte)184, (byte)219, (byte)90, (byte)51, (byte)98, (byte)18, (byte)186, (byte)126, (byte)89, (byte)18, (byte)178, (byte)46, (byte)246, (byte)213, (byte)115, (byte)186, (byte)253, (byte)97, (byte)86, (byte)49, (byte)175, (byte)140, (byte)89, (byte)19, (byte)176, (byte)104, (byte)56, (byte)150, (byte)208, (byte)161, (byte)3, (byte)144, (byte)87, (byte)170, (byte)187, (byte)202, (byte)231, (byte)76, (byte)228, (byte)108, (byte)18, (byte)110, (byte)226, (byte)184, (byte)71, (byte)95, (byte)159, (byte)111, (byte)81, (byte)246, (byte)186, (byte)31, (byte)30, (byte)21, (byte)118, (byte)55, (byte)176, (byte)8, (byte)11, (byte)164, (byte)142, (byte)76, (byte)15, (byte)48, (byte)253, (byte)71, (byte)3, (byte)230, (byte)186, (byte)81, (byte)206, (byte)176, (byte)214, (byte)71, (byte)248, (byte)87, (byte)127, (byte)234, (byte)119, (byte)173, (byte)115, (byte)189, (byte)172, (byte)168, (byte)129, (byte)0, (byte)125, (byte)68, (byte)195, (byte)118, (byte)179, (byte)246, (byte)106, (byte)216, (byte)82, (byte)108, (byte)212, (byte)4, (byte)214, (byte)24, (byte)124, (byte)184, (byte)28, (byte)93, (byte)143, (byte)237, (byte)126, (byte)141, (byte)192, (byte)55, (byte)2, (byte)3, (byte)125, (byte)176, (byte)248, (byte)60, (byte)49, (byte)8, (byte)186, (byte)216, (byte)2, (byte)231, (byte)152, (byte)193, (byte)255, (byte)145, (byte)199, (byte)35, (byte)101, (byte)203, (byte)171, (byte)101, (byte)202, (byte)127, (byte)67, (byte)7, (byte)58, (byte)156, (byte)142, (byte)151, (byte)18, (byte)165, (byte)87, (byte)190, (byte)113, (byte)236, (byte)136, (byte)151, (byte)196, (byte)178}));
                Debug.Assert(pack.target_component == (byte)(byte)117);
                Debug.Assert(pack.sequence == (ushort)(ushort)37306);
                Debug.Assert(pack.length == (byte)(byte)39);
                Debug.Assert(pack.target_system == (byte)(byte)70);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.data__SET(new byte[] {(byte)21, (byte)3, (byte)22, (byte)4, (byte)218, (byte)148, (byte)29, (byte)206, (byte)159, (byte)138, (byte)112, (byte)123, (byte)100, (byte)233, (byte)211, (byte)123, (byte)134, (byte)0, (byte)179, (byte)116, (byte)211, (byte)24, (byte)189, (byte)129, (byte)4, (byte)205, (byte)104, (byte)69, (byte)141, (byte)163, (byte)189, (byte)192, (byte)41, (byte)110, (byte)221, (byte)208, (byte)89, (byte)190, (byte)19, (byte)207, (byte)64, (byte)239, (byte)205, (byte)126, (byte)182, (byte)139, (byte)222, (byte)33, (byte)179, (byte)192, (byte)172, (byte)47, (byte)178, (byte)112, (byte)107, (byte)61, (byte)191, (byte)144, (byte)110, (byte)57, (byte)226, (byte)94, (byte)150, (byte)233, (byte)41, (byte)179, (byte)171, (byte)189, (byte)144, (byte)169, (byte)202, (byte)221, (byte)62, (byte)219, (byte)98, (byte)32, (byte)42, (byte)10, (byte)125, (byte)36, (byte)108, (byte)227, (byte)215, (byte)124, (byte)16, (byte)163, (byte)154, (byte)77, (byte)97, (byte)78, (byte)1, (byte)7, (byte)211, (byte)37, (byte)203, (byte)220, (byte)179, (byte)121, (byte)0, (byte)184, (byte)219, (byte)90, (byte)51, (byte)98, (byte)18, (byte)186, (byte)126, (byte)89, (byte)18, (byte)178, (byte)46, (byte)246, (byte)213, (byte)115, (byte)186, (byte)253, (byte)97, (byte)86, (byte)49, (byte)175, (byte)140, (byte)89, (byte)19, (byte)176, (byte)104, (byte)56, (byte)150, (byte)208, (byte)161, (byte)3, (byte)144, (byte)87, (byte)170, (byte)187, (byte)202, (byte)231, (byte)76, (byte)228, (byte)108, (byte)18, (byte)110, (byte)226, (byte)184, (byte)71, (byte)95, (byte)159, (byte)111, (byte)81, (byte)246, (byte)186, (byte)31, (byte)30, (byte)21, (byte)118, (byte)55, (byte)176, (byte)8, (byte)11, (byte)164, (byte)142, (byte)76, (byte)15, (byte)48, (byte)253, (byte)71, (byte)3, (byte)230, (byte)186, (byte)81, (byte)206, (byte)176, (byte)214, (byte)71, (byte)248, (byte)87, (byte)127, (byte)234, (byte)119, (byte)173, (byte)115, (byte)189, (byte)172, (byte)168, (byte)129, (byte)0, (byte)125, (byte)68, (byte)195, (byte)118, (byte)179, (byte)246, (byte)106, (byte)216, (byte)82, (byte)108, (byte)212, (byte)4, (byte)214, (byte)24, (byte)124, (byte)184, (byte)28, (byte)93, (byte)143, (byte)237, (byte)126, (byte)141, (byte)192, (byte)55, (byte)2, (byte)3, (byte)125, (byte)176, (byte)248, (byte)60, (byte)49, (byte)8, (byte)186, (byte)216, (byte)2, (byte)231, (byte)152, (byte)193, (byte)255, (byte)145, (byte)199, (byte)35, (byte)101, (byte)203, (byte)171, (byte)101, (byte)202, (byte)127, (byte)67, (byte)7, (byte)58, (byte)156, (byte)142, (byte)151, (byte)18, (byte)165, (byte)87, (byte)190, (byte)113, (byte)236, (byte)136, (byte)151, (byte)196, (byte)178}, 0) ;
            p267.first_message_offset = (byte)(byte)150;
            p267.target_component = (byte)(byte)117;
            p267.sequence = (ushort)(ushort)37306;
            p267.length = (byte)(byte)39;
            p267.target_system = (byte)(byte)70;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)59583);
                Debug.Assert(pack.target_component == (byte)(byte)246);
                Debug.Assert(pack.target_system == (byte)(byte)141);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.sequence = (ushort)(ushort)59583;
            p268.target_system = (byte)(byte)141;
            p268.target_component = (byte)(byte)246;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)217);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)13252);
                Debug.Assert(pack.rotation == (ushort)(ushort)57828);
                Debug.Assert(pack.uri_LEN(ph) == 173);
                Debug.Assert(pack.uri_TRY(ph).Equals("tvijtouacIinGvwrtyrosvdmybtfscdmInlpzyvrpCnlxigucPttqyyVbknvbjqopUmrzdFIioInytyJjrixkFajwQGWthwhnxqrpzwvAxRgnYiVjiutxsbyEjqluocxsLcxfeusobyzbbzsxkhzokziyawshwpnjnlnpvvvpqmlc"));
                Debug.Assert(pack.framerate == (float)3.3147978E38F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)24216);
                Debug.Assert(pack.status == (byte)(byte)2);
                Debug.Assert(pack.bitrate == (uint)537002653U);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.bitrate = (uint)537002653U;
            p269.rotation = (ushort)(ushort)57828;
            p269.resolution_v = (ushort)(ushort)13252;
            p269.framerate = (float)3.3147978E38F;
            p269.status = (byte)(byte)2;
            p269.resolution_h = (ushort)(ushort)24216;
            p269.camera_id = (byte)(byte)217;
            p269.uri_SET("tvijtouacIinGvwrtyrosvdmybtfscdmInlpzyvrpCnlxigucPttqyyVbknvbjqopUmrzdFIioInytyJjrixkFajwQGWthwhnxqrpzwvAxRgnYiVjiutxsbyEjqluocxsLcxfeusobyzbbzsxkhzokziyawshwpnjnlnpvvvpqmlc", PH) ;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)71);
                Debug.Assert(pack.uri_LEN(ph) == 44);
                Debug.Assert(pack.uri_TRY(ph).Equals("lgpNipgtlpZTzBmEuybyidEdJYhfkcAaJhdagJRfprXz"));
                Debug.Assert(pack.target_component == (byte)(byte)208);
                Debug.Assert(pack.target_system == (byte)(byte)152);
                Debug.Assert(pack.rotation == (ushort)(ushort)26679);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)30429);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)10156);
                Debug.Assert(pack.bitrate == (uint)3528381563U);
                Debug.Assert(pack.framerate == (float) -1.268125E37F);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)152;
            p270.resolution_h = (ushort)(ushort)10156;
            p270.target_component = (byte)(byte)208;
            p270.uri_SET("lgpNipgtlpZTzBmEuybyidEdJYhfkcAaJhdagJRfprXz", PH) ;
            p270.camera_id = (byte)(byte)71;
            p270.resolution_v = (ushort)(ushort)30429;
            p270.rotation = (ushort)(ushort)26679;
            p270.framerate = (float) -1.268125E37F;
            p270.bitrate = (uint)3528381563U;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 31);
                Debug.Assert(pack.password_TRY(ph).Equals("rlfqbphwrusceayjarndHzepueseyiB"));
                Debug.Assert(pack.ssid_LEN(ph) == 11);
                Debug.Assert(pack.ssid_TRY(ph).Equals("ujjzyEtqavb"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("ujjzyEtqavb", PH) ;
            p299.password_SET("rlfqbphwrusceayjarndHzepueseyiB", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)13, (byte)76, (byte)220, (byte)31, (byte)224, (byte)204, (byte)234, (byte)244}));
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)81, (byte)51, (byte)132, (byte)86, (byte)22, (byte)204, (byte)38, (byte)251}));
                Debug.Assert(pack.min_version == (ushort)(ushort)25464);
                Debug.Assert(pack.max_version == (ushort)(ushort)64268);
                Debug.Assert(pack.version == (ushort)(ushort)65379);
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.max_version = (ushort)(ushort)64268;
            p300.spec_version_hash_SET(new byte[] {(byte)13, (byte)76, (byte)220, (byte)31, (byte)224, (byte)204, (byte)234, (byte)244}, 0) ;
            p300.library_version_hash_SET(new byte[] {(byte)81, (byte)51, (byte)132, (byte)86, (byte)22, (byte)204, (byte)38, (byte)251}, 0) ;
            p300.min_version = (ushort)(ushort)25464;
            p300.version = (ushort)(ushort)65379;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)261305374837272391L);
                Debug.Assert(pack.uptime_sec == (uint)47310885U);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
                Debug.Assert(pack.sub_mode == (byte)(byte)141);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)61227);
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.uptime_sec = (uint)47310885U;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
            p310.sub_mode = (byte)(byte)141;
            p310.time_usec = (ulong)261305374837272391L;
            p310.vendor_specific_status_code = (ushort)(ushort)61227;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4430944820571762762L);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)95, (byte)230, (byte)25, (byte)68, (byte)76, (byte)254, (byte)27, (byte)37, (byte)252, (byte)26, (byte)135, (byte)32, (byte)70, (byte)7, (byte)239, (byte)178}));
                Debug.Assert(pack.hw_version_minor == (byte)(byte)210);
                Debug.Assert(pack.sw_vcs_commit == (uint)1201263611U);
                Debug.Assert(pack.sw_version_major == (byte)(byte)142);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)177);
                Debug.Assert(pack.uptime_sec == (uint)2348722363U);
                Debug.Assert(pack.hw_version_major == (byte)(byte)112);
                Debug.Assert(pack.name_LEN(ph) == 21);
                Debug.Assert(pack.name_TRY(ph).Equals("klrmomnivuirhqRbfyjyt"));
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_version_minor = (byte)(byte)210;
            p311.hw_unique_id_SET(new byte[] {(byte)95, (byte)230, (byte)25, (byte)68, (byte)76, (byte)254, (byte)27, (byte)37, (byte)252, (byte)26, (byte)135, (byte)32, (byte)70, (byte)7, (byte)239, (byte)178}, 0) ;
            p311.hw_version_major = (byte)(byte)112;
            p311.sw_version_major = (byte)(byte)142;
            p311.sw_version_minor = (byte)(byte)177;
            p311.uptime_sec = (uint)2348722363U;
            p311.name_SET("klrmomnivuirhqRbfyjyt", PH) ;
            p311.sw_vcs_commit = (uint)1201263611U;
            p311.time_usec = (ulong)4430944820571762762L;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)69);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("xjrx"));
                Debug.Assert(pack.param_index == (short)(short)27467);
                Debug.Assert(pack.target_system == (byte)(byte)135);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)135;
            p320.param_index = (short)(short)27467;
            p320.param_id_SET("xjrx", PH) ;
            p320.target_component = (byte)(byte)69;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)121);
                Debug.Assert(pack.target_system == (byte)(byte)196);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)196;
            p321.target_component = (byte)(byte)121;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 54);
                Debug.Assert(pack.param_value_TRY(ph).Equals("byxnynzhxWmhEpelhiqkeierwzvldTlcfpldgfcmrpaxEmKmnpenok"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("kZRFaigxraa"));
                Debug.Assert(pack.param_index == (ushort)(ushort)64475);
                Debug.Assert(pack.param_count == (ushort)(ushort)18403);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("kZRFaigxraa", PH) ;
            p322.param_index = (ushort)(ushort)64475;
            p322.param_value_SET("byxnynzhxWmhEpelhiqkeierwzvldTlcfpldgfcmrpaxEmKmnpenok", PH) ;
            p322.param_count = (ushort)(ushort)18403;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)182);
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("taetximejcyAexi"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
                Debug.Assert(pack.target_component == (byte)(byte)118);
                Debug.Assert(pack.param_value_LEN(ph) == 94);
                Debug.Assert(pack.param_value_TRY(ph).Equals("kelEgRlifstxkbfslulbanwqlhywtxleszwrvicldyscHtdoeiokbqjxatwwmozvtqnrmriJwlMnnpdemYmhecqcfnhaVj"));
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)182;
            p323.target_component = (byte)(byte)118;
            p323.param_id_SET("taetximejcyAexi", PH) ;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32;
            p323.param_value_SET("kelEgRlifstxkbfslulbanwqlhywtxleszwrvicldyscHtdoeiokbqjxatwwmozvtqnrmriJwlMnnpdemYmhecqcfnhaVj", PH) ;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 89);
                Debug.Assert(pack.param_value_TRY(ph).Equals("rmcjjumklmbmutwnecjesjejbvupsmqxpaxfxpulFudssVhnzzujOonrgovehrsznicolqwnvqZtltfecxghlOkmm"));
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_FAILED);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
                Debug.Assert(pack.param_id_LEN(ph) == 13);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ipijCjzawgojQ"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_value_SET("rmcjjumklmbmutwnecjesjejbvupsmqxpaxfxpulFudssVhnzzujOonrgovehrsznicolqwnvqZtltfecxghlOkmm", PH) ;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_FAILED;
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            p324.param_id_SET("ipijCjzawgojQ", PH) ;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_distance == (ushort)(ushort)49738);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)43325, (ushort)51646, (ushort)62680, (ushort)63931, (ushort)28718, (ushort)35211, (ushort)20074, (ushort)61267, (ushort)8055, (ushort)17406, (ushort)39422, (ushort)63653, (ushort)51495, (ushort)42605, (ushort)24195, (ushort)45359, (ushort)46803, (ushort)13177, (ushort)6459, (ushort)7015, (ushort)6321, (ushort)23898, (ushort)11448, (ushort)54140, (ushort)36293, (ushort)8681, (ushort)16550, (ushort)9983, (ushort)59184, (ushort)8974, (ushort)30515, (ushort)30608, (ushort)38489, (ushort)32572, (ushort)8073, (ushort)54526, (ushort)64805, (ushort)33547, (ushort)4948, (ushort)47360, (ushort)4263, (ushort)59865, (ushort)65464, (ushort)39661, (ushort)17030, (ushort)6749, (ushort)12174, (ushort)6025, (ushort)4747, (ushort)2087, (ushort)14495, (ushort)21110, (ushort)30659, (ushort)4603, (ushort)43022, (ushort)11661, (ushort)61155, (ushort)8795, (ushort)28052, (ushort)18574, (ushort)34498, (ushort)37149, (ushort)38366, (ushort)3784, (ushort)43348, (ushort)2538, (ushort)729, (ushort)34019, (ushort)26588, (ushort)60343, (ushort)643, (ushort)53543}));
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
                Debug.Assert(pack.max_distance == (ushort)(ushort)34378);
                Debug.Assert(pack.time_usec == (ulong)1685356408512444283L);
                Debug.Assert(pack.increment == (byte)(byte)14);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.max_distance = (ushort)(ushort)34378;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p330.min_distance = (ushort)(ushort)49738;
            p330.time_usec = (ulong)1685356408512444283L;
            p330.increment = (byte)(byte)14;
            p330.distances_SET(new ushort[] {(ushort)43325, (ushort)51646, (ushort)62680, (ushort)63931, (ushort)28718, (ushort)35211, (ushort)20074, (ushort)61267, (ushort)8055, (ushort)17406, (ushort)39422, (ushort)63653, (ushort)51495, (ushort)42605, (ushort)24195, (ushort)45359, (ushort)46803, (ushort)13177, (ushort)6459, (ushort)7015, (ushort)6321, (ushort)23898, (ushort)11448, (ushort)54140, (ushort)36293, (ushort)8681, (ushort)16550, (ushort)9983, (ushort)59184, (ushort)8974, (ushort)30515, (ushort)30608, (ushort)38489, (ushort)32572, (ushort)8073, (ushort)54526, (ushort)64805, (ushort)33547, (ushort)4948, (ushort)47360, (ushort)4263, (ushort)59865, (ushort)65464, (ushort)39661, (ushort)17030, (ushort)6749, (ushort)12174, (ushort)6025, (ushort)4747, (ushort)2087, (ushort)14495, (ushort)21110, (ushort)30659, (ushort)4603, (ushort)43022, (ushort)11661, (ushort)61155, (ushort)8795, (ushort)28052, (ushort)18574, (ushort)34498, (ushort)37149, (ushort)38366, (ushort)3784, (ushort)43348, (ushort)2538, (ushort)729, (ushort)34019, (ushort)26588, (ushort)60343, (ushort)643, (ushort)53543}, 0) ;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
        }
    }
}
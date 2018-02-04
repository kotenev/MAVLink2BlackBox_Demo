
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
                        case MAV_CMD.MAV_CMD_AQ_NAV_LEG_ORBIT:
                            id = 0;
                            break;
                        case MAV_CMD.MAV_CMD_AQ_TELEMETRY:
                            id = 1;
                            break;
                        case MAV_CMD.MAV_CMD_AQ_REQUEST_VERSION:
                            id = 2;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                            id = 3;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                            id = 4;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                            id = 5;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                            id = 6;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                            id = 7;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAND:
                            id = 8;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                            id = 9;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                            id = 10;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                            id = 11;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                            id = 12;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                            id = 13;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                            id = 14;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FOLLOW:
                            id = 15;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                            id = 16;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_ROI:
                            id = 17;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                            id = 18;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
                            id = 19;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                            id = 20;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                            id = 21;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                            id = 22;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_DELAY:
                            id = 23;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                            id = 24;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAST:
                            id = 25;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                            id = 26;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                            id = 27;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                            id = 28;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_YAW:
                            id = 29;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_LAST:
                            id = 30;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_MODE:
                            id = 31;
                            break;
                        case MAV_CMD.MAV_CMD_DO_JUMP:
                            id = 32;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                            id = 33;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_HOME:
                            id = 34;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                            id = 35;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                            id = 36;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                            id = 37;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                            id = 38;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                            id = 39;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                            id = 40;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                            id = 41;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAND_START:
                            id = 42;
                            break;
                        case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                            id = 43;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                            id = 44;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPOSITION:
                            id = 45;
                            break;
                        case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                            id = 46;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                            id = 47;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                            id = 48;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_ROI:
                            id = 49;
                            break;
                        case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                            id = 50;
                            break;
                        case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                            id = 51;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                            id = 52;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                            id = 53;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                            id = 54;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                            id = 55;
                            break;
                        case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                            id = 56;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                            id = 58;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                            id = 59;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                            id = 60;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                            id = 61;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                            id = 62;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                            id = 63;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                            id = 64;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAST:
                            id = 65;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                            id = 66;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                            id = 67;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                            id = 68;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                            id = 69;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                            id = 70;
                            break;
                        case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                            id = 71;
                            break;
                        case MAV_CMD.MAV_CMD_MISSION_START:
                            id = 72;
                            break;
                        case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                            id = 73;
                            break;
                        case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                            id = 74;
                            break;
                        case MAV_CMD.MAV_CMD_START_RX_PAIR:
                            id = 75;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                            id = 76;
                            break;
                        case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                            id = 77;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                            id = 78;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                            id = 79;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                            id = 80;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                            id = 81;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                            id = 82;
                            break;
                        case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                            id = 83;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                            id = 84;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                            id = 85;
                            break;
                        case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                            id = 86;
                            break;
                        case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                            id = 87;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                            id = 88;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                            id = 89;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                            id = 90;
                            break;
                        case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                            id = 91;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                            id = 92;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                            id = 93;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                            id = 94;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                            id = 95;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                            id = 96;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_START:
                            id = 97;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_STOP:
                            id = 98;
                            break;
                        case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                            id = 99;
                            break;
                        case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                            id = 100;
                            break;
                        case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                            id = 101;
                            break;
                        case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                            id = 102;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                            id = 103;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                            id = 104;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_GATE:
                            id = 105;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                            id = 106;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                            id = 107;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                            id = 108;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                            id = 109;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 128;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 129;
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
                        case MAV_CMD.MAV_CMD_AQ_NAV_LEG_ORBIT:
                            id = 0;
                            break;
                        case MAV_CMD.MAV_CMD_AQ_TELEMETRY:
                            id = 1;
                            break;
                        case MAV_CMD.MAV_CMD_AQ_REQUEST_VERSION:
                            id = 2;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                            id = 3;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                            id = 4;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                            id = 5;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                            id = 6;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                            id = 7;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAND:
                            id = 8;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                            id = 9;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                            id = 10;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                            id = 11;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                            id = 12;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                            id = 13;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                            id = 14;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FOLLOW:
                            id = 15;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                            id = 16;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_ROI:
                            id = 17;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                            id = 18;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
                            id = 19;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                            id = 20;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                            id = 21;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                            id = 22;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_DELAY:
                            id = 23;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                            id = 24;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAST:
                            id = 25;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                            id = 26;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                            id = 27;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                            id = 28;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_YAW:
                            id = 29;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_LAST:
                            id = 30;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_MODE:
                            id = 31;
                            break;
                        case MAV_CMD.MAV_CMD_DO_JUMP:
                            id = 32;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                            id = 33;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_HOME:
                            id = 34;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                            id = 35;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                            id = 36;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                            id = 37;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                            id = 38;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                            id = 39;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                            id = 40;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                            id = 41;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAND_START:
                            id = 42;
                            break;
                        case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                            id = 43;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                            id = 44;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPOSITION:
                            id = 45;
                            break;
                        case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                            id = 46;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                            id = 47;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                            id = 48;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_ROI:
                            id = 49;
                            break;
                        case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                            id = 50;
                            break;
                        case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                            id = 51;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                            id = 52;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                            id = 53;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                            id = 54;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                            id = 55;
                            break;
                        case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                            id = 56;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                            id = 58;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                            id = 59;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                            id = 60;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                            id = 61;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                            id = 62;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                            id = 63;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                            id = 64;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAST:
                            id = 65;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                            id = 66;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                            id = 67;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                            id = 68;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                            id = 69;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                            id = 70;
                            break;
                        case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                            id = 71;
                            break;
                        case MAV_CMD.MAV_CMD_MISSION_START:
                            id = 72;
                            break;
                        case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                            id = 73;
                            break;
                        case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                            id = 74;
                            break;
                        case MAV_CMD.MAV_CMD_START_RX_PAIR:
                            id = 75;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                            id = 76;
                            break;
                        case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                            id = 77;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                            id = 78;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                            id = 79;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                            id = 80;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                            id = 81;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                            id = 82;
                            break;
                        case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                            id = 83;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                            id = 84;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                            id = 85;
                            break;
                        case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                            id = 86;
                            break;
                        case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                            id = 87;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                            id = 88;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                            id = 89;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                            id = 90;
                            break;
                        case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                            id = 91;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                            id = 92;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                            id = 93;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                            id = 94;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                            id = 95;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                            id = 96;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_START:
                            id = 97;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_STOP:
                            id = 98;
                            break;
                        case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                            id = 99;
                            break;
                        case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                            id = 100;
                            break;
                        case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                            id = 101;
                            break;
                        case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                            id = 102;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                            id = 103;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                            id = 104;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_GATE:
                            id = 105;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                            id = 106;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                            id = 107;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                            id = 108;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                            id = 109;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 128;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 129;
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
        new class SET_ACTUATOR_CONTROL_TARGET : GroundControl.SET_ACTUATOR_CONTROL_TARGET
        {
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            /**
            *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
            *	this field to difference between instances*/
            public byte group_mlx
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
            }

            /**
            *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
            *	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
            *	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
            *	mixer to repurpose them as generic outputs*/
            public float[] controls
            {
                get {return controls_GET(new float[8], 0);}
            }
            /**
            *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
            *	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
            *	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
            *	mixer to repurpose them as generic outputs*/
            public float[]controls_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 11, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
        }
        new class ACTUATOR_CONTROL_TARGET : GroundControl.ACTUATOR_CONTROL_TARGET
        {
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            /**
            *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
            *	this field to difference between instances*/
            public byte group_mlx
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            /**
            *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
            *	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
            *	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
            *	mixer to repurpose them as generic outputs*/
            public float[] controls
            {
                get {return controls_GET(new float[8], 0);}
            }
            /**
            *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
            *	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
            *	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
            *	mixer to repurpose them as generic outputs*/
            public float[]controls_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 9, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
        }
        new class ALTITUDE : GroundControl.ALTITUDE
        {
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            /**
            *This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
            *	local altitude change). The only guarantee on this field is that it will never be reset and is consistent
            *	within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
            *	time. This altitude will also drift and vary between flights*/
            public float altitude_monotonic
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            /**
            *This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
            *	like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
            *	are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
            *	by default and not the WGS84 altitude*/
            public float altitude_amsl
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            /**
            *This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
            *	to the coordinate origin (0, 0, 0). It is up-positive*/
            public float altitude_local
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float altitude_relative //This is the altitude above the home position. It resets on each change of the current home position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            /**
            *This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
            *	than -1000 should be interpreted as unknown*/
            public float altitude_terrain
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            /**
            *This is not the altitude, but the clear space below the system according to the fused clearance estimate.
            *	It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
            *	target. A negative value indicates no measurement available*/
            public float bottom_clearance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }
        }
        new class RESOURCE_REQUEST : GroundControl.RESOURCE_REQUEST
        {
            public byte request_id //Request ID. This ID should be re-used when sending back URI contents
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte uri_type //The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            /**
            *The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
            *	on the URI type enum*/
            public byte[] uri
            {
                get {return uri_GET(new byte[120], 0);}
            }
            /**
            *The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
            *	on the URI type enum*/
            public byte[]uri_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 2, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public byte transfer_type //The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  122, 1));}
            }

            /**
            *The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
            *	has a storage associated (e.g. MAVLink FTP)*/
            public byte[] storage
            {
                get {return storage_GET(new byte[120], 0);}
            }
            /**
            *The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
            *	has a storage associated (e.g. MAVLink FTP)*/
            public byte[]storage_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 123, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class SCALED_PRESSURE3 : GroundControl.SCALED_PRESSURE3
        {
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
        }
        new class FOLLOW_TARGET : GroundControl.FOLLOW_TARGET
        {
            public ulong timestamp //Timestamp in milliseconds since system boot
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public ulong custom_state //button states or switches of a tracker device
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
            }

            public byte est_capabilities //bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
            }

            public int lat //Latitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  17, 4));}
            }

            public int lon //Longitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  21, 4));}
            }

            public float alt //AMSL, in meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  25, 4)));}
            }

            public float[] vel //target velocity (0,0,0) for unknown
            {
                get {return vel_GET(new float[3], 0);}
            }
            public float[]vel_GET(float[] dst_ch, int pos)  //target velocity (0,0,0) for unknown
            {
                for(int BYTE = 29, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float[] acc //linear target acceleration (0,0,0) for unknown
            {
                get {return acc_GET(new float[3], 0);}
            }
            public float[]acc_GET(float[] dst_ch, int pos)  //linear target acceleration (0,0,0) for unknown
            {
                for(int BYTE = 41, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float[] attitude_q //(1 0 0 0 for unknown)
            {
                get {return attitude_q_GET(new float[4], 0);}
            }
            public float[]attitude_q_GET(float[] dst_ch, int pos)  //(1 0 0 0 for unknown)
            {
                for(int BYTE = 53, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float[] rates //(0 0 0 for unknown)
            {
                get {return rates_GET(new float[3], 0);}
            }
            public float[]rates_GET(float[] dst_ch, int pos)  //(0 0 0 for unknown)
            {
                for(int BYTE = 69, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float[] position_cov //eph epv
            {
                get {return position_cov_GET(new float[3], 0);}
            }
            public float[]position_cov_GET(float[] dst_ch, int pos)  //eph epv
            {
                for(int BYTE = 81, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
        }
        new class CONTROL_SYSTEM_STATE : GroundControl.CONTROL_SYSTEM_STATE
        {
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float x_acc //X acceleration in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float y_acc //Y acceleration in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float z_acc //Z acceleration in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float x_vel //X velocity in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float y_vel //Y velocity in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float z_vel //Z velocity in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float x_pos //X position in local frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float y_pos //Y position in local frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public float z_pos //Z position in local frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
            }

            public float airspeed //Airspeed, set to -1 if unknown
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
            }

            public float[] vel_variance //Variance of body velocity estimate
            {
                get {return vel_variance_GET(new float[3], 0);}
            }
            public float[]vel_variance_GET(float[] dst_ch, int pos)  //Variance of body velocity estimate
            {
                for(int BYTE = 48, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float[] pos_variance //Variance in local position
            {
                get {return pos_variance_GET(new float[3], 0);}
            }
            public float[]pos_variance_GET(float[] dst_ch, int pos)  //Variance in local position
            {
                for(int BYTE = 60, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float[] q //The attitude, represented as Quaternion
            {
                get {return q_GET(new float[4], 0);}
            }
            public float[]q_GET(float[] dst_ch, int pos)  //The attitude, represented as Quaternion
            {
                for(int BYTE = 72, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float roll_rate //Angular rate in roll axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  88, 4)));}
            }

            public float pitch_rate //Angular rate in pitch axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  92, 4)));}
            }

            public float yaw_rate //Angular rate in yaw axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  96, 4)));}
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
        new class AQ_TELEMETRY_F : GroundControl.AQ_TELEMETRY_F
        {
            public ushort Index //Index of message
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public float value1 //value1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  2, 4)));}
            }

            public float value2 //value2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }

            public float value3 //value3
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }

            public float value4 //value4
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float value5 //value5
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public float value6 //value6
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
            }

            public float value7 //value7
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
            }

            public float value8 //value8
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
            }

            public float value9 //value9
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  34, 4)));}
            }

            public float value10 //value10
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  38, 4)));}
            }

            public float value11 //value11
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  42, 4)));}
            }

            public float value12 //value12
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  46, 4)));}
            }

            public float value13 //value13
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  50, 4)));}
            }

            public float value14 //value14
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  54, 4)));}
            }

            public float value15 //value15
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  58, 4)));}
            }

            public float value16 //value16
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  62, 4)));}
            }

            public float value17 //value17
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  66, 4)));}
            }

            public float value18 //value18
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  70, 4)));}
            }

            public float value19 //value19
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  74, 4)));}
            }

            public float value20 //value20
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  78, 4)));}
            }
        }
        new class AQ_ESC_TELEMETRY : GroundControl.AQ_ESC_TELEMETRY
        {
            public ushort[] status_age //Age of each ESC telemetry reading in ms compared to boot time. A value of 0xFFFF means timeout/no data
            {
                get {return status_age_GET(new ushort[4], 0);}
            }
            public ushort[]status_age_GET(ushort[] dst_ch, int pos)  //Age of each ESC telemetry reading in ms compared to boot time. A value of 0xFFFF means timeout/no data
            {
                for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public uint time_boot_ms //Timestamp of the component clock since boot time in ms.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  8, 4));}
            }

            public uint[] data0 //Data bits 1-32 for each ESC.
            {
                get {return data0_GET(new uint[4], 0);}
            }
            public uint[]data0_GET(uint[] dst_ch, int pos)  //Data bits 1-32 for each ESC.
            {
                for(int BYTE = 12, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public uint[] data1 //Data bits 33-64 for each ESC.
            {
                get {return data1_GET(new uint[4], 0);}
            }
            public uint[]data1_GET(uint[] dst_ch, int pos)  //Data bits 33-64 for each ESC.
            {
                for(int BYTE = 28, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public byte seq //Sequence number of message (first set of 4 motors is #1, next 4 is #2, etc).
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  44, 1));}
            }

            public byte num_motors //Total number of active ESCs/motors on the system.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  45, 1));}
            }

            public byte num_in_seq //Number of active ESCs in this sequence (1 through this many array members will be populated with data
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  46, 1));}
            }

            public byte[] escid //ESC/Motor ID
            {
                get {return escid_GET(new byte[4], 0);}
            }
            public byte[]escid_GET(byte[] dst_ch, int pos)  //ESC/Motor ID
            {
                for(int BYTE = 47, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public byte[] data_version //Version of data structure (determines contents).
            {
                get {return data_version_GET(new byte[4], 0);}
            }
            public byte[]data_version_GET(byte[] dst_ch, int pos)  //Version of data structure (determines contents).
            {
                for(int BYTE = 51, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
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

            public void OnSET_ACTUATOR_CONTROL_TARGETReceive_direct(Channel src, Inside ph, SET_ACTUATOR_CONTROL_TARGET pack) {OnSET_ACTUATOR_CONTROL_TARGETReceive(this, ph,  pack);}
            public event SET_ACTUATOR_CONTROL_TARGETReceiveHandler OnSET_ACTUATOR_CONTROL_TARGETReceive;
            public delegate void SET_ACTUATOR_CONTROL_TARGETReceiveHandler(Channel src, Inside ph, SET_ACTUATOR_CONTROL_TARGET pack);
            public void OnACTUATOR_CONTROL_TARGETReceive_direct(Channel src, Inside ph, ACTUATOR_CONTROL_TARGET pack) {OnACTUATOR_CONTROL_TARGETReceive(this, ph,  pack);}
            public event ACTUATOR_CONTROL_TARGETReceiveHandler OnACTUATOR_CONTROL_TARGETReceive;
            public delegate void ACTUATOR_CONTROL_TARGETReceiveHandler(Channel src, Inside ph, ACTUATOR_CONTROL_TARGET pack);
            public void OnALTITUDEReceive_direct(Channel src, Inside ph, ALTITUDE pack) {OnALTITUDEReceive(this, ph,  pack);}
            public event ALTITUDEReceiveHandler OnALTITUDEReceive;
            public delegate void ALTITUDEReceiveHandler(Channel src, Inside ph, ALTITUDE pack);
            public void OnRESOURCE_REQUESTReceive_direct(Channel src, Inside ph, RESOURCE_REQUEST pack) {OnRESOURCE_REQUESTReceive(this, ph,  pack);}
            public event RESOURCE_REQUESTReceiveHandler OnRESOURCE_REQUESTReceive;
            public delegate void RESOURCE_REQUESTReceiveHandler(Channel src, Inside ph, RESOURCE_REQUEST pack);
            public void OnSCALED_PRESSURE3Receive_direct(Channel src, Inside ph, SCALED_PRESSURE3 pack) {OnSCALED_PRESSURE3Receive(this, ph,  pack);}
            public event SCALED_PRESSURE3ReceiveHandler OnSCALED_PRESSURE3Receive;
            public delegate void SCALED_PRESSURE3ReceiveHandler(Channel src, Inside ph, SCALED_PRESSURE3 pack);
            public void OnFOLLOW_TARGETReceive_direct(Channel src, Inside ph, FOLLOW_TARGET pack) {OnFOLLOW_TARGETReceive(this, ph,  pack);}
            public event FOLLOW_TARGETReceiveHandler OnFOLLOW_TARGETReceive;
            public delegate void FOLLOW_TARGETReceiveHandler(Channel src, Inside ph, FOLLOW_TARGET pack);
            public void OnCONTROL_SYSTEM_STATEReceive_direct(Channel src, Inside ph, CONTROL_SYSTEM_STATE pack) {OnCONTROL_SYSTEM_STATEReceive(this, ph,  pack);}
            public event CONTROL_SYSTEM_STATEReceiveHandler OnCONTROL_SYSTEM_STATEReceive;
            public delegate void CONTROL_SYSTEM_STATEReceiveHandler(Channel src, Inside ph, CONTROL_SYSTEM_STATE pack);
            public void OnBATTERY_STATUSReceive_direct(Channel src, Inside ph, BATTERY_STATUS pack) {OnBATTERY_STATUSReceive(this, ph,  pack);}
            public event BATTERY_STATUSReceiveHandler OnBATTERY_STATUSReceive;
            public delegate void BATTERY_STATUSReceiveHandler(Channel src, Inside ph, BATTERY_STATUS pack);
            public void OnAUTOPILOT_VERSIONReceive_direct(Channel src, Inside ph, AUTOPILOT_VERSION pack) {OnAUTOPILOT_VERSIONReceive(this, ph,  pack);}
            public event AUTOPILOT_VERSIONReceiveHandler OnAUTOPILOT_VERSIONReceive;
            public delegate void AUTOPILOT_VERSIONReceiveHandler(Channel src, Inside ph, AUTOPILOT_VERSION pack);
            public void OnLANDING_TARGETReceive_direct(Channel src, Inside ph, LANDING_TARGET pack) {OnLANDING_TARGETReceive(this, ph,  pack);}
            public event LANDING_TARGETReceiveHandler OnLANDING_TARGETReceive;
            public delegate void LANDING_TARGETReceiveHandler(Channel src, Inside ph, LANDING_TARGET pack);
            public void OnAQ_TELEMETRY_FReceive_direct(Channel src, Inside ph, AQ_TELEMETRY_F pack) {OnAQ_TELEMETRY_FReceive(this, ph,  pack);}
            public event AQ_TELEMETRY_FReceiveHandler OnAQ_TELEMETRY_FReceive;
            public delegate void AQ_TELEMETRY_FReceiveHandler(Channel src, Inside ph, AQ_TELEMETRY_F pack);
            public void OnAQ_ESC_TELEMETRYReceive_direct(Channel src, Inside ph, AQ_ESC_TELEMETRY pack) {OnAQ_ESC_TELEMETRYReceive(this, ph,  pack);}
            public event AQ_ESC_TELEMETRYReceiveHandler OnAQ_ESC_TELEMETRYReceive;
            public delegate void AQ_ESC_TELEMETRYReceiveHandler(Channel src, Inside ph, AQ_ESC_TELEMETRY pack);
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
                    case 139:
                        if(pack == null) return new SET_ACTUATOR_CONTROL_TARGET();
                        OnSET_ACTUATOR_CONTROL_TARGETReceive(this, ph, (SET_ACTUATOR_CONTROL_TARGET) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 140:
                        if(pack == null) return new ACTUATOR_CONTROL_TARGET();
                        OnACTUATOR_CONTROL_TARGETReceive(this, ph, (ACTUATOR_CONTROL_TARGET) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 141:
                        if(pack == null) return new ALTITUDE();
                        OnALTITUDEReceive(this, ph, (ALTITUDE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 142:
                        if(pack == null) return new RESOURCE_REQUEST();
                        OnRESOURCE_REQUESTReceive(this, ph, (RESOURCE_REQUEST) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 143:
                        if(pack == null) return new SCALED_PRESSURE3();
                        OnSCALED_PRESSURE3Receive(this, ph, (SCALED_PRESSURE3) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 144:
                        if(pack == null) return new FOLLOW_TARGET();
                        OnFOLLOW_TARGETReceive(this, ph, (FOLLOW_TARGET) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 146:
                        if(pack == null) return new CONTROL_SYSTEM_STATE();
                        OnCONTROL_SYSTEM_STATEReceive(this, ph, (CONTROL_SYSTEM_STATE) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                    case 150:
                        if(pack == null) return new AQ_TELEMETRY_F();
                        OnAQ_TELEMETRY_FReceive(this, ph, (AQ_TELEMETRY_F) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 152:
                        if(pack == null) return new AQ_ESC_TELEMETRY();
                        OnAQ_ESC_TELEMETRYReceive(this, ph, (AQ_ESC_TELEMETRY) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_GENERIC);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_POWEROFF);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
                Debug.Assert(pack.custom_mode == (uint)182790531U);
                Debug.Assert(pack.mavlink_version == (byte)(byte)101);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_SMARTAP);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_SMARTAP;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED;
            p0.mavlink_version = (byte)(byte)101;
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_GENERIC;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_POWEROFF;
            p0.custom_mode = (uint)182790531U;
            SMP_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)63259);
                Debug.Assert(pack.load == (ushort)(ushort)57579);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)37180);
                Debug.Assert(pack.current_battery == (short)(short) -19594);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)32656);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)32);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)17700);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)1709);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)4391);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)31987);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.errors_count3 = (ushort)(ushort)17700;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO;
            p1.errors_count2 = (ushort)(ushort)1709;
            p1.voltage_battery = (ushort)(ushort)63259;
            p1.errors_comm = (ushort)(ushort)4391;
            p1.load = (ushort)(ushort)57579;
            p1.current_battery = (short)(short) -19594;
            p1.drop_rate_comm = (ushort)(ushort)31987;
            p1.errors_count1 = (ushort)(ushort)32656;
            p1.battery_remaining = (sbyte)(sbyte)32;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
            p1.errors_count4 = (ushort)(ushort)37180;
            SMP_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)902725893728552865L);
                Debug.Assert(pack.time_boot_ms == (uint)3488811810U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)902725893728552865L;
            p2.time_boot_ms = (uint)3488811810U;
            SMP_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.vz == (float) -3.1710085E38F);
                Debug.Assert(pack.afz == (float) -3.1991242E38F);
                Debug.Assert(pack.time_boot_ms == (uint)970859739U);
                Debug.Assert(pack.z == (float)4.8714177E36F);
                Debug.Assert(pack.yaw_rate == (float) -1.8832017E38F);
                Debug.Assert(pack.afx == (float) -2.5327261E38F);
                Debug.Assert(pack.vx == (float) -4.574629E37F);
                Debug.Assert(pack.y == (float)1.0397395E38F);
                Debug.Assert(pack.vy == (float) -5.947033E36F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)10362);
                Debug.Assert(pack.yaw == (float)2.4530718E38F);
                Debug.Assert(pack.x == (float) -1.7584835E38F);
                Debug.Assert(pack.afy == (float)2.441177E38F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.afz = (float) -3.1991242E38F;
            p3.yaw_rate = (float) -1.8832017E38F;
            p3.y = (float)1.0397395E38F;
            p3.afy = (float)2.441177E38F;
            p3.vy = (float) -5.947033E36F;
            p3.vx = (float) -4.574629E37F;
            p3.afx = (float) -2.5327261E38F;
            p3.x = (float) -1.7584835E38F;
            p3.z = (float)4.8714177E36F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED;
            p3.vz = (float) -3.1710085E38F;
            p3.type_mask = (ushort)(ushort)10362;
            p3.yaw = (float)2.4530718E38F;
            p3.time_boot_ms = (uint)970859739U;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)186);
                Debug.Assert(pack.seq == (uint)4032016164U);
                Debug.Assert(pack.target_system == (byte)(byte)160);
                Debug.Assert(pack.time_usec == (ulong)6020820109671474266L);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)6020820109671474266L;
            p4.seq = (uint)4032016164U;
            p4.target_system = (byte)(byte)160;
            p4.target_component = (byte)(byte)186;
            SMP_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)203);
                Debug.Assert(pack.control_request == (byte)(byte)134);
                Debug.Assert(pack.version == (byte)(byte)0);
                Debug.Assert(pack.passkey_LEN(ph) == 4);
                Debug.Assert(pack.passkey_TRY(ph).Equals("bsym"));
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.passkey_SET("bsym", PH) ;
            p5.control_request = (byte)(byte)134;
            p5.version = (byte)(byte)0;
            p5.target_system = (byte)(byte)203;
            SMP_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gcs_system_id == (byte)(byte)193);
                Debug.Assert(pack.ack == (byte)(byte)44);
                Debug.Assert(pack.control_request == (byte)(byte)13);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)193;
            p6.control_request = (byte)(byte)13;
            p6.ack = (byte)(byte)44;
            SMP_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 8);
                Debug.Assert(pack.key_TRY(ph).Equals("qitxonBY"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("qitxonBY", PH) ;
            SMP_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_mode == (uint)180291619U);
                Debug.Assert(pack.target_system == (byte)(byte)133);
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.custom_mode = (uint)180291619U;
            p11.target_system = (byte)(byte)133;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            SMP_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)163);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("zrqfg"));
                Debug.Assert(pack.param_index == (short)(short)2326);
                Debug.Assert(pack.target_component == (byte)(byte)22);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_index = (short)(short)2326;
            p20.param_id_SET("zrqfg", PH) ;
            p20.target_system = (byte)(byte)163;
            p20.target_component = (byte)(byte)22;
            SMP_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)54);
                Debug.Assert(pack.target_component == (byte)(byte)122);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)122;
            p21.target_system = (byte)(byte)54;
            SMP_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value == (float) -2.4989797E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("lalq"));
                Debug.Assert(pack.param_count == (ushort)(ushort)26923);
                Debug.Assert(pack.param_index == (ushort)(ushort)14910);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_index = (ushort)(ushort)14910;
            p22.param_id_SET("lalq", PH) ;
            p22.param_count = (ushort)(ushort)26923;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32;
            p22.param_value = (float) -2.4989797E38F;
            SMP_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)9);
                Debug.Assert(pack.param_value == (float) -1.2180222E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 6);
                Debug.Assert(pack.param_id_TRY(ph).Equals("faXevr"));
                Debug.Assert(pack.target_system == (byte)(byte)180);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_value = (float) -1.2180222E38F;
            p23.target_system = (byte)(byte)180;
            p23.target_component = (byte)(byte)9;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16;
            p23.param_id_SET("faXevr", PH) ;
            SMP_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.epv == (ushort)(ushort)16612);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)258551408U);
                Debug.Assert(pack.cog == (ushort)(ushort)19815);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)2597942788U);
                Debug.Assert(pack.alt == (int)1537137722);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.lat == (int)213279769);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1375998112U);
                Debug.Assert(pack.eph == (ushort)(ushort)29205);
                Debug.Assert(pack.vel == (ushort)(ushort)29458);
                Debug.Assert(pack.satellites_visible == (byte)(byte)67);
                Debug.Assert(pack.lon == (int) -1973771577);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)2703524360U);
                Debug.Assert(pack.time_usec == (ulong)1381762888705476629L);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -1974713445);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.lon = (int) -1973771577;
            p24.v_acc_SET((uint)1375998112U, PH) ;
            p24.h_acc_SET((uint)2597942788U, PH) ;
            p24.vel = (ushort)(ushort)29458;
            p24.vel_acc_SET((uint)2703524360U, PH) ;
            p24.epv = (ushort)(ushort)16612;
            p24.alt = (int)1537137722;
            p24.hdg_acc_SET((uint)258551408U, PH) ;
            p24.cog = (ushort)(ushort)19815;
            p24.satellites_visible = (byte)(byte)67;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p24.time_usec = (ulong)1381762888705476629L;
            p24.lat = (int)213279769;
            p24.alt_ellipsoid_SET((int) -1974713445, PH) ;
            p24.eph = (ushort)(ushort)29205;
            SMP_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)63);
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)91, (byte)62, (byte)50, (byte)196, (byte)201, (byte)23, (byte)154, (byte)161, (byte)87, (byte)250, (byte)60, (byte)246, (byte)241, (byte)246, (byte)243, (byte)240, (byte)14, (byte)85, (byte)103, (byte)26}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)55, (byte)235, (byte)195, (byte)205, (byte)241, (byte)100, (byte)61, (byte)81, (byte)213, (byte)212, (byte)63, (byte)194, (byte)221, (byte)187, (byte)17, (byte)217, (byte)244, (byte)13, (byte)50, (byte)225}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)196, (byte)165, (byte)144, (byte)10, (byte)147, (byte)214, (byte)212, (byte)250, (byte)254, (byte)10, (byte)88, (byte)99, (byte)227, (byte)115, (byte)31, (byte)41, (byte)95, (byte)159, (byte)214, (byte)95}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)15, (byte)190, (byte)3, (byte)8, (byte)49, (byte)175, (byte)5, (byte)185, (byte)63, (byte)117, (byte)240, (byte)92, (byte)95, (byte)158, (byte)166, (byte)127, (byte)166, (byte)41, (byte)241, (byte)174}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)243, (byte)249, (byte)213, (byte)202, (byte)127, (byte)67, (byte)62, (byte)133, (byte)141, (byte)189, (byte)165, (byte)95, (byte)131, (byte)34, (byte)156, (byte)92, (byte)220, (byte)220, (byte)37, (byte)161}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_azimuth_SET(new byte[] {(byte)196, (byte)165, (byte)144, (byte)10, (byte)147, (byte)214, (byte)212, (byte)250, (byte)254, (byte)10, (byte)88, (byte)99, (byte)227, (byte)115, (byte)31, (byte)41, (byte)95, (byte)159, (byte)214, (byte)95}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)55, (byte)235, (byte)195, (byte)205, (byte)241, (byte)100, (byte)61, (byte)81, (byte)213, (byte)212, (byte)63, (byte)194, (byte)221, (byte)187, (byte)17, (byte)217, (byte)244, (byte)13, (byte)50, (byte)225}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)91, (byte)62, (byte)50, (byte)196, (byte)201, (byte)23, (byte)154, (byte)161, (byte)87, (byte)250, (byte)60, (byte)246, (byte)241, (byte)246, (byte)243, (byte)240, (byte)14, (byte)85, (byte)103, (byte)26}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)243, (byte)249, (byte)213, (byte)202, (byte)127, (byte)67, (byte)62, (byte)133, (byte)141, (byte)189, (byte)165, (byte)95, (byte)131, (byte)34, (byte)156, (byte)92, (byte)220, (byte)220, (byte)37, (byte)161}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)15, (byte)190, (byte)3, (byte)8, (byte)49, (byte)175, (byte)5, (byte)185, (byte)63, (byte)117, (byte)240, (byte)92, (byte)95, (byte)158, (byte)166, (byte)127, (byte)166, (byte)41, (byte)241, (byte)174}, 0) ;
            p25.satellites_visible = (byte)(byte)63;
            SMP_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ymag == (short)(short) -29563);
                Debug.Assert(pack.time_boot_ms == (uint)3374929426U);
                Debug.Assert(pack.yacc == (short)(short) -8479);
                Debug.Assert(pack.zgyro == (short)(short)2784);
                Debug.Assert(pack.xgyro == (short)(short) -4089);
                Debug.Assert(pack.xmag == (short)(short) -17758);
                Debug.Assert(pack.ygyro == (short)(short)5843);
                Debug.Assert(pack.zacc == (short)(short) -27929);
                Debug.Assert(pack.xacc == (short)(short) -13486);
                Debug.Assert(pack.zmag == (short)(short)20964);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.ygyro = (short)(short)5843;
            p26.ymag = (short)(short) -29563;
            p26.time_boot_ms = (uint)3374929426U;
            p26.zmag = (short)(short)20964;
            p26.xacc = (short)(short) -13486;
            p26.xgyro = (short)(short) -4089;
            p26.xmag = (short)(short) -17758;
            p26.zgyro = (short)(short)2784;
            p26.zacc = (short)(short) -27929;
            p26.yacc = (short)(short) -8479;
            SMP_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short)3787);
                Debug.Assert(pack.zgyro == (short)(short)4061);
                Debug.Assert(pack.zmag == (short)(short) -17118);
                Debug.Assert(pack.zacc == (short)(short)32589);
                Debug.Assert(pack.ygyro == (short)(short)19377);
                Debug.Assert(pack.ymag == (short)(short) -12052);
                Debug.Assert(pack.yacc == (short)(short) -27218);
                Debug.Assert(pack.xgyro == (short)(short) -30619);
                Debug.Assert(pack.time_usec == (ulong)8518941436268005783L);
                Debug.Assert(pack.xmag == (short)(short)28970);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.xgyro = (short)(short) -30619;
            p27.zgyro = (short)(short)4061;
            p27.ygyro = (short)(short)19377;
            p27.xacc = (short)(short)3787;
            p27.xmag = (short)(short)28970;
            p27.time_usec = (ulong)8518941436268005783L;
            p27.zacc = (short)(short)32589;
            p27.yacc = (short)(short) -27218;
            p27.ymag = (short)(short) -12052;
            p27.zmag = (short)(short) -17118;
            SMP_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff2 == (short)(short) -22489);
                Debug.Assert(pack.temperature == (short)(short) -6015);
                Debug.Assert(pack.time_usec == (ulong)7544567248344767759L);
                Debug.Assert(pack.press_abs == (short)(short)27670);
                Debug.Assert(pack.press_diff1 == (short)(short)702);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.time_usec = (ulong)7544567248344767759L;
            p28.press_abs = (short)(short)27670;
            p28.press_diff1 = (short)(short)702;
            p28.temperature = (short)(short) -6015;
            p28.press_diff2 = (short)(short) -22489;
            SMP_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float)1.036809E38F);
                Debug.Assert(pack.temperature == (short)(short) -30565);
                Debug.Assert(pack.press_diff == (float) -1.3551042E38F);
                Debug.Assert(pack.time_boot_ms == (uint)213633837U);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.temperature = (short)(short) -30565;
            p29.press_abs = (float)1.036809E38F;
            p29.press_diff = (float) -1.3551042E38F;
            p29.time_boot_ms = (uint)213633837U;
            SMP_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float) -1.0297601E38F);
                Debug.Assert(pack.roll == (float) -2.676664E37F);
                Debug.Assert(pack.pitch == (float)1.2261133E38F);
                Debug.Assert(pack.rollspeed == (float) -1.4273388E38F);
                Debug.Assert(pack.pitchspeed == (float)2.6996104E38F);
                Debug.Assert(pack.time_boot_ms == (uint)256050510U);
                Debug.Assert(pack.yaw == (float) -3.2737203E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.roll = (float) -2.676664E37F;
            p30.pitchspeed = (float)2.6996104E38F;
            p30.yawspeed = (float) -1.0297601E38F;
            p30.rollspeed = (float) -1.4273388E38F;
            p30.yaw = (float) -3.2737203E38F;
            p30.pitch = (float)1.2261133E38F;
            p30.time_boot_ms = (uint)256050510U;
            SMP_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q4 == (float) -1.1262122E38F);
                Debug.Assert(pack.q3 == (float)7.988329E37F);
                Debug.Assert(pack.yawspeed == (float)8.859117E37F);
                Debug.Assert(pack.rollspeed == (float) -2.9621384E38F);
                Debug.Assert(pack.pitchspeed == (float) -1.8992852E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2367025836U);
                Debug.Assert(pack.q2 == (float) -5.044596E37F);
                Debug.Assert(pack.q1 == (float) -3.0991562E38F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.pitchspeed = (float) -1.8992852E38F;
            p31.q3 = (float)7.988329E37F;
            p31.q2 = (float) -5.044596E37F;
            p31.q1 = (float) -3.0991562E38F;
            p31.time_boot_ms = (uint)2367025836U;
            p31.rollspeed = (float) -2.9621384E38F;
            p31.q4 = (float) -1.1262122E38F;
            p31.yawspeed = (float)8.859117E37F;
            SMP_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -1.3295495E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1202972202U);
                Debug.Assert(pack.vx == (float)2.3876645E37F);
                Debug.Assert(pack.vy == (float)2.328318E38F);
                Debug.Assert(pack.y == (float) -2.9831526E38F);
                Debug.Assert(pack.vz == (float) -2.9053071E38F);
                Debug.Assert(pack.x == (float) -4.1765226E37F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vx = (float)2.3876645E37F;
            p32.time_boot_ms = (uint)1202972202U;
            p32.vy = (float)2.328318E38F;
            p32.x = (float) -4.1765226E37F;
            p32.z = (float) -1.3295495E38F;
            p32.vz = (float) -2.9053071E38F;
            p32.y = (float) -2.9831526E38F;
            SMP_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1927627928);
                Debug.Assert(pack.time_boot_ms == (uint)466822770U);
                Debug.Assert(pack.alt == (int) -1550233521);
                Debug.Assert(pack.lat == (int)2097541749);
                Debug.Assert(pack.vz == (short)(short)13783);
                Debug.Assert(pack.vx == (short)(short) -4345);
                Debug.Assert(pack.vy == (short)(short) -6606);
                Debug.Assert(pack.relative_alt == (int)1551150473);
                Debug.Assert(pack.hdg == (ushort)(ushort)31269);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.lon = (int) -1927627928;
            p33.relative_alt = (int)1551150473;
            p33.hdg = (ushort)(ushort)31269;
            p33.lat = (int)2097541749;
            p33.vy = (short)(short) -6606;
            p33.time_boot_ms = (uint)466822770U;
            p33.vx = (short)(short) -4345;
            p33.vz = (short)(short)13783;
            p33.alt = (int) -1550233521;
            SMP_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rssi == (byte)(byte)229);
                Debug.Assert(pack.chan7_scaled == (short)(short)3101);
                Debug.Assert(pack.chan4_scaled == (short)(short) -8899);
                Debug.Assert(pack.chan5_scaled == (short)(short)24269);
                Debug.Assert(pack.chan1_scaled == (short)(short)11237);
                Debug.Assert(pack.chan3_scaled == (short)(short) -27371);
                Debug.Assert(pack.port == (byte)(byte)157);
                Debug.Assert(pack.chan6_scaled == (short)(short) -31271);
                Debug.Assert(pack.time_boot_ms == (uint)2402173516U);
                Debug.Assert(pack.chan8_scaled == (short)(short) -19930);
                Debug.Assert(pack.chan2_scaled == (short)(short)5339);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan8_scaled = (short)(short) -19930;
            p34.chan6_scaled = (short)(short) -31271;
            p34.chan7_scaled = (short)(short)3101;
            p34.chan2_scaled = (short)(short)5339;
            p34.chan1_scaled = (short)(short)11237;
            p34.port = (byte)(byte)157;
            p34.rssi = (byte)(byte)229;
            p34.chan5_scaled = (short)(short)24269;
            p34.time_boot_ms = (uint)2402173516U;
            p34.chan3_scaled = (short)(short) -27371;
            p34.chan4_scaled = (short)(short) -8899;
            SMP_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)9409);
                Debug.Assert(pack.rssi == (byte)(byte)68);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)35468);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)19238);
                Debug.Assert(pack.time_boot_ms == (uint)2637326928U);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)16366);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)43962);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)50252);
                Debug.Assert(pack.port == (byte)(byte)167);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)51510);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)20485);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan5_raw = (ushort)(ushort)50252;
            p35.chan2_raw = (ushort)(ushort)19238;
            p35.time_boot_ms = (uint)2637326928U;
            p35.chan1_raw = (ushort)(ushort)16366;
            p35.chan8_raw = (ushort)(ushort)51510;
            p35.chan3_raw = (ushort)(ushort)20485;
            p35.chan6_raw = (ushort)(ushort)35468;
            p35.rssi = (byte)(byte)68;
            p35.port = (byte)(byte)167;
            p35.chan7_raw = (ushort)(ushort)9409;
            p35.chan4_raw = (ushort)(ushort)43962;
            SMP_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)36805);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)24515);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)42476);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)47256);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)26075);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)6122);
                Debug.Assert(pack.time_usec == (uint)3803906090U);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)60935);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)14488);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)21943);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)50116);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)56418);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)36005);
                Debug.Assert(pack.port == (byte)(byte)72);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)2658);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)44256);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)19493);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)31519);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo5_raw = (ushort)(ushort)6122;
            p36.time_usec = (uint)3803906090U;
            p36.servo7_raw = (ushort)(ushort)60935;
            p36.servo15_raw_SET((ushort)(ushort)19493, PH) ;
            p36.servo1_raw = (ushort)(ushort)47256;
            p36.servo8_raw = (ushort)(ushort)2658;
            p36.servo16_raw_SET((ushort)(ushort)24515, PH) ;
            p36.servo3_raw = (ushort)(ushort)36805;
            p36.servo11_raw_SET((ushort)(ushort)44256, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)50116, PH) ;
            p36.servo10_raw_SET((ushort)(ushort)42476, PH) ;
            p36.servo6_raw = (ushort)(ushort)56418;
            p36.servo14_raw_SET((ushort)(ushort)26075, PH) ;
            p36.servo2_raw = (ushort)(ushort)14488;
            p36.servo13_raw_SET((ushort)(ushort)21943, PH) ;
            p36.port = (byte)(byte)72;
            p36.servo9_raw_SET((ushort)(ushort)36005, PH) ;
            p36.servo4_raw = (ushort)(ushort)31519;
            SMP_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short)10905);
                Debug.Assert(pack.target_system == (byte)(byte)247);
                Debug.Assert(pack.target_component == (byte)(byte)49);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.start_index == (short)(short) -1923);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p37.start_index = (short)(short) -1923;
            p37.end_index = (short)(short)10905;
            p37.target_system = (byte)(byte)247;
            p37.target_component = (byte)(byte)49;
            SMP_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.end_index == (short)(short) -205);
                Debug.Assert(pack.target_component == (byte)(byte)89);
                Debug.Assert(pack.start_index == (short)(short)16436);
                Debug.Assert(pack.target_system == (byte)(byte)32);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p38.target_component = (byte)(byte)89;
            p38.end_index = (short)(short) -205;
            p38.start_index = (short)(short)16436;
            p38.target_system = (byte)(byte)32;
            SMP_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)250);
                Debug.Assert(pack.param3 == (float) -3.0256122E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION);
                Debug.Assert(pack.param2 == (float) -4.5755144E37F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.autocontinue == (byte)(byte)255);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.y == (float)1.4452576E37F);
                Debug.Assert(pack.x == (float) -2.77652E38F);
                Debug.Assert(pack.param4 == (float) -1.956562E38F);
                Debug.Assert(pack.z == (float) -3.2722145E38F);
                Debug.Assert(pack.param1 == (float) -2.6480371E38F);
                Debug.Assert(pack.target_system == (byte)(byte)193);
                Debug.Assert(pack.seq == (ushort)(ushort)51116);
                Debug.Assert(pack.current == (byte)(byte)200);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.z = (float) -3.2722145E38F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p39.autocontinue = (byte)(byte)255;
            p39.current = (byte)(byte)200;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p39.param2 = (float) -4.5755144E37F;
            p39.x = (float) -2.77652E38F;
            p39.target_component = (byte)(byte)250;
            p39.param3 = (float) -3.0256122E38F;
            p39.target_system = (byte)(byte)193;
            p39.param4 = (float) -1.956562E38F;
            p39.param1 = (float) -2.6480371E38F;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
            p39.seq = (ushort)(ushort)51116;
            p39.y = (float)1.4452576E37F;
            SMP_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)129);
                Debug.Assert(pack.seq == (ushort)(ushort)50859);
                Debug.Assert(pack.target_system == (byte)(byte)100);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p40.target_component = (byte)(byte)129;
            p40.target_system = (byte)(byte)100;
            p40.seq = (ushort)(ushort)50859;
            SMP_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)233);
                Debug.Assert(pack.target_component == (byte)(byte)102);
                Debug.Assert(pack.seq == (ushort)(ushort)27134);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)233;
            p41.seq = (ushort)(ushort)27134;
            p41.target_component = (byte)(byte)102;
            SMP_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)18457);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)18457;
            SMP_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)178);
                Debug.Assert(pack.target_system == (byte)(byte)60);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_component = (byte)(byte)178;
            p43.target_system = (byte)(byte)60;
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            SMP_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.count == (ushort)(ushort)48878);
                Debug.Assert(pack.target_system == (byte)(byte)185);
                Debug.Assert(pack.target_component == (byte)(byte)229);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.count = (ushort)(ushort)48878;
            p44.target_component = (byte)(byte)229;
            p44.target_system = (byte)(byte)185;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            SMP_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)239);
                Debug.Assert(pack.target_system == (byte)(byte)144);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_component = (byte)(byte)239;
            p45.target_system = (byte)(byte)144;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            SMP_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)35915);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)35915;
            SMP_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)39);
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED_FRAME);
                Debug.Assert(pack.target_system == (byte)(byte)252);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED_FRAME;
            p47.target_system = (byte)(byte)252;
            p47.target_component = (byte)(byte)39;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            SMP_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int) -1888819419);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)5815634781670035908L);
                Debug.Assert(pack.target_system == (byte)(byte)43);
                Debug.Assert(pack.altitude == (int) -792814006);
                Debug.Assert(pack.latitude == (int)1149058960);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.longitude = (int) -1888819419;
            p48.target_system = (byte)(byte)43;
            p48.latitude = (int)1149058960;
            p48.time_usec_SET((ulong)5815634781670035908L, PH) ;
            p48.altitude = (int) -792814006;
            SMP_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6387831781523248338L);
                Debug.Assert(pack.longitude == (int)786648807);
                Debug.Assert(pack.latitude == (int) -810377955);
                Debug.Assert(pack.altitude == (int)853722839);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.time_usec_SET((ulong)6387831781523248338L, PH) ;
            p49.longitude = (int)786648807;
            p49.latitude = (int) -810377955;
            p49.altitude = (int)853722839;
            SMP_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)247);
                Debug.Assert(pack.scale == (float) -3.2728787E38F);
                Debug.Assert(pack.param_value_max == (float)2.603103E38F);
                Debug.Assert(pack.target_system == (byte)(byte)75);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("hodmycZkrvym"));
                Debug.Assert(pack.target_component == (byte)(byte)202);
                Debug.Assert(pack.param_value0 == (float)2.9138655E38F);
                Debug.Assert(pack.param_value_min == (float) -1.4177828E38F);
                Debug.Assert(pack.param_index == (short)(short)3246);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_value_max = (float)2.603103E38F;
            p50.param_index = (short)(short)3246;
            p50.target_system = (byte)(byte)75;
            p50.param_value0 = (float)2.9138655E38F;
            p50.parameter_rc_channel_index = (byte)(byte)247;
            p50.param_id_SET("hodmycZkrvym", PH) ;
            p50.target_component = (byte)(byte)202;
            p50.scale = (float) -3.2728787E38F;
            p50.param_value_min = (float) -1.4177828E38F;
            SMP_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)51);
                Debug.Assert(pack.seq == (ushort)(ushort)3394);
                Debug.Assert(pack.target_component == (byte)(byte)79);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.seq = (ushort)(ushort)3394;
            p51.target_component = (byte)(byte)79;
            p51.target_system = (byte)(byte)51;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            SMP_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1x == (float)3.1143003E38F);
                Debug.Assert(pack.p2y == (float)1.7867904E37F);
                Debug.Assert(pack.target_component == (byte)(byte)143);
                Debug.Assert(pack.p1y == (float)7.302531E37F);
                Debug.Assert(pack.target_system == (byte)(byte)113);
                Debug.Assert(pack.p2z == (float) -2.8965585E38F);
                Debug.Assert(pack.p2x == (float)1.0528079E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.p1z == (float)1.6925891E38F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p2z = (float) -2.8965585E38F;
            p54.p1y = (float)7.302531E37F;
            p54.p1x = (float)3.1143003E38F;
            p54.p1z = (float)1.6925891E38F;
            p54.p2y = (float)1.7867904E37F;
            p54.target_component = (byte)(byte)143;
            p54.p2x = (float)1.0528079E38F;
            p54.target_system = (byte)(byte)113;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            SMP_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2z == (float) -2.3225973E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.p1x == (float) -1.9900743E38F);
                Debug.Assert(pack.p2y == (float)2.3856874E38F);
                Debug.Assert(pack.p1y == (float) -8.754769E37F);
                Debug.Assert(pack.p2x == (float)1.0395152E38F);
                Debug.Assert(pack.p1z == (float) -2.6677322E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2x = (float)1.0395152E38F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED;
            p55.p1z = (float) -2.6677322E38F;
            p55.p2z = (float) -2.3225973E38F;
            p55.p1x = (float) -1.9900743E38F;
            p55.p1y = (float) -8.754769E37F;
            p55.p2y = (float)2.3856874E38F;
            SMP_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.7136783E38F, 2.9036296E38F, 3.25591E38F, 1.2756744E38F}));
                Debug.Assert(pack.pitchspeed == (float)2.983857E38F);
                Debug.Assert(pack.time_usec == (ulong)4573452597238456340L);
                Debug.Assert(pack.yawspeed == (float) -8.325871E36F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.510721E38F, 2.4655555E38F, -1.4354421E38F, -3.9109947E37F, -9.353894E37F, -3.3557194E38F, 1.9394908E38F, -3.204121E38F, -1.1717904E38F}));
                Debug.Assert(pack.rollspeed == (float)2.714428E37F);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.q_SET(new float[] {2.7136783E38F, 2.9036296E38F, 3.25591E38F, 1.2756744E38F}, 0) ;
            p61.rollspeed = (float)2.714428E37F;
            p61.yawspeed = (float) -8.325871E36F;
            p61.time_usec = (ulong)4573452597238456340L;
            p61.pitchspeed = (float)2.983857E38F;
            p61.covariance_SET(new float[] {2.510721E38F, 2.4655555E38F, -1.4354421E38F, -3.9109947E37F, -9.353894E37F, -3.3557194E38F, 1.9394908E38F, -3.204121E38F, -1.1717904E38F}, 0) ;
            SMP_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_roll == (float) -1.5362614E38F);
                Debug.Assert(pack.aspd_error == (float) -1.9600673E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)27228);
                Debug.Assert(pack.nav_bearing == (short)(short)22987);
                Debug.Assert(pack.alt_error == (float)2.2899777E38F);
                Debug.Assert(pack.nav_pitch == (float) -3.3570023E38F);
                Debug.Assert(pack.target_bearing == (short)(short)16045);
                Debug.Assert(pack.xtrack_error == (float) -1.3508172E38F);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.target_bearing = (short)(short)16045;
            p62.xtrack_error = (float) -1.3508172E38F;
            p62.wp_dist = (ushort)(ushort)27228;
            p62.nav_pitch = (float) -3.3570023E38F;
            p62.nav_bearing = (short)(short)22987;
            p62.aspd_error = (float) -1.9600673E38F;
            p62.nav_roll = (float) -1.5362614E38F;
            p62.alt_error = (float)2.2899777E38F;
            SMP_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.relative_alt == (int)344185902);
                Debug.Assert(pack.vx == (float)6.842858E37F);
                Debug.Assert(pack.alt == (int) -514986676);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-3.6006834E37F, -8.0258636E37F, 3.3043643E38F, 2.1198075E38F, -2.0892488E37F, -2.2669685E37F, 2.2941436E38F, 1.1507633E38F, -2.8405298E38F, 1.4504501E38F, 1.6112041E38F, 1.9003482E38F, 8.703628E37F, -2.0656577E38F, -9.856982E37F, 2.399454E38F, 3.559819E37F, -2.8488437E38F, -1.1455792E38F, -1.802158E38F, -2.9218198E38F, 5.792073E37F, -1.9122722E38F, -8.801748E37F, 1.1566055E37F, -1.2839999E38F, 3.0263409E37F, 1.7970803E38F, -1.9375828E38F, -2.8420053E38F, 7.336803E37F, -1.5737703E38F, 1.4737836E38F, 2.4364007E38F, 2.2709366E38F, 2.5207269E38F}));
                Debug.Assert(pack.lon == (int)1550817645);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
                Debug.Assert(pack.lat == (int) -443508380);
                Debug.Assert(pack.time_usec == (ulong)6639213942059903671L);
                Debug.Assert(pack.vz == (float) -1.97088E38F);
                Debug.Assert(pack.vy == (float)1.9966864E38F);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.lon = (int)1550817645;
            p63.time_usec = (ulong)6639213942059903671L;
            p63.covariance_SET(new float[] {-3.6006834E37F, -8.0258636E37F, 3.3043643E38F, 2.1198075E38F, -2.0892488E37F, -2.2669685E37F, 2.2941436E38F, 1.1507633E38F, -2.8405298E38F, 1.4504501E38F, 1.6112041E38F, 1.9003482E38F, 8.703628E37F, -2.0656577E38F, -9.856982E37F, 2.399454E38F, 3.559819E37F, -2.8488437E38F, -1.1455792E38F, -1.802158E38F, -2.9218198E38F, 5.792073E37F, -1.9122722E38F, -8.801748E37F, 1.1566055E37F, -1.2839999E38F, 3.0263409E37F, 1.7970803E38F, -1.9375828E38F, -2.8420053E38F, 7.336803E37F, -1.5737703E38F, 1.4737836E38F, 2.4364007E38F, 2.2709366E38F, 2.5207269E38F}, 0) ;
            p63.relative_alt = (int)344185902;
            p63.lat = (int) -443508380;
            p63.alt = (int) -514986676;
            p63.vy = (float)1.9966864E38F;
            p63.vx = (float)6.842858E37F;
            p63.vz = (float) -1.97088E38F;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            SMP_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
                Debug.Assert(pack.vx == (float)3.1465299E38F);
                Debug.Assert(pack.ay == (float) -1.0145716E38F);
                Debug.Assert(pack.vy == (float)7.827898E37F);
                Debug.Assert(pack.time_usec == (ulong)724732504305667506L);
                Debug.Assert(pack.y == (float)1.7591936E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.3857718E38F, -2.314418E38F, -2.8233964E38F, 3.3887779E38F, 3.05873E37F, 6.344201E37F, 1.1349162E38F, 3.7042236E37F, -2.2355554E38F, 6.822354E37F, 1.4781856E38F, 2.1221371E37F, -3.2244865E38F, 2.6485541E38F, 3.0405293E38F, 2.8313893E38F, -9.429859E37F, -1.4247902E37F, -1.2365828E38F, 3.237555E38F, 7.7830096E37F, -3.6244248E37F, 3.1720011E38F, -3.0339978E38F, -2.1828718E38F, -2.775041E38F, -3.231507E38F, -7.2271347E36F, -8.514523E37F, -3.9511554E37F, 2.3650778E38F, 2.6200873E37F, 4.850076E37F, 9.751254E37F, 3.0849653E38F, -1.0859311E38F, -2.7820227E38F, -8.951727E37F, -2.752017E38F, 1.4393544E38F, -2.717566E38F, 1.1444219E38F, 1.2354427E38F, 1.6930499E38F, -1.4601475E38F}));
                Debug.Assert(pack.ax == (float)2.5900087E38F);
                Debug.Assert(pack.x == (float) -1.3870859E38F);
                Debug.Assert(pack.az == (float) -1.7444878E38F);
                Debug.Assert(pack.vz == (float)2.165913E38F);
                Debug.Assert(pack.z == (float) -8.2284174E37F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.y = (float)1.7591936E38F;
            p64.vx = (float)3.1465299E38F;
            p64.az = (float) -1.7444878E38F;
            p64.vz = (float)2.165913E38F;
            p64.ax = (float)2.5900087E38F;
            p64.z = (float) -8.2284174E37F;
            p64.time_usec = (ulong)724732504305667506L;
            p64.x = (float) -1.3870859E38F;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            p64.vy = (float)7.827898E37F;
            p64.ay = (float) -1.0145716E38F;
            p64.covariance_SET(new float[] {-2.3857718E38F, -2.314418E38F, -2.8233964E38F, 3.3887779E38F, 3.05873E37F, 6.344201E37F, 1.1349162E38F, 3.7042236E37F, -2.2355554E38F, 6.822354E37F, 1.4781856E38F, 2.1221371E37F, -3.2244865E38F, 2.6485541E38F, 3.0405293E38F, 2.8313893E38F, -9.429859E37F, -1.4247902E37F, -1.2365828E38F, 3.237555E38F, 7.7830096E37F, -3.6244248E37F, 3.1720011E38F, -3.0339978E38F, -2.1828718E38F, -2.775041E38F, -3.231507E38F, -7.2271347E36F, -8.514523E37F, -3.9511554E37F, 2.3650778E38F, 2.6200873E37F, 4.850076E37F, 9.751254E37F, 3.0849653E38F, -1.0859311E38F, -2.7820227E38F, -8.951727E37F, -2.752017E38F, 1.4393544E38F, -2.717566E38F, 1.1444219E38F, 1.2354427E38F, 1.6930499E38F, -1.4601475E38F}, 0) ;
            SMP_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)6728);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)9252);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)33945);
                Debug.Assert(pack.rssi == (byte)(byte)176);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)61425);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)46961);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)25172);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)41981);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)61718);
                Debug.Assert(pack.time_boot_ms == (uint)2747921719U);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)24886);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)51918);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)29110);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)53595);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)30034);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)40401);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)53988);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)809);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)58085);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)32104);
                Debug.Assert(pack.chancount == (byte)(byte)242);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.time_boot_ms = (uint)2747921719U;
            p65.chan2_raw = (ushort)(ushort)40401;
            p65.rssi = (byte)(byte)176;
            p65.chan16_raw = (ushort)(ushort)30034;
            p65.chan8_raw = (ushort)(ushort)53595;
            p65.chan12_raw = (ushort)(ushort)61718;
            p65.chan15_raw = (ushort)(ushort)809;
            p65.chan4_raw = (ushort)(ushort)41981;
            p65.chan6_raw = (ushort)(ushort)9252;
            p65.chan10_raw = (ushort)(ushort)33945;
            p65.chan1_raw = (ushort)(ushort)25172;
            p65.chan9_raw = (ushort)(ushort)51918;
            p65.chan7_raw = (ushort)(ushort)53988;
            p65.chan13_raw = (ushort)(ushort)6728;
            p65.chan11_raw = (ushort)(ushort)46961;
            p65.chancount = (byte)(byte)242;
            p65.chan18_raw = (ushort)(ushort)24886;
            p65.chan5_raw = (ushort)(ushort)58085;
            p65.chan3_raw = (ushort)(ushort)32104;
            p65.chan17_raw = (ushort)(ushort)29110;
            p65.chan14_raw = (ushort)(ushort)61425;
            SMP_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_stop == (byte)(byte)101);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)46368);
                Debug.Assert(pack.target_component == (byte)(byte)235);
                Debug.Assert(pack.req_stream_id == (byte)(byte)187);
                Debug.Assert(pack.target_system == (byte)(byte)129);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.req_stream_id = (byte)(byte)187;
            p66.target_component = (byte)(byte)235;
            p66.start_stop = (byte)(byte)101;
            p66.target_system = (byte)(byte)129;
            p66.req_message_rate = (ushort)(ushort)46368;
            SMP_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.stream_id == (byte)(byte)92);
                Debug.Assert(pack.on_off == (byte)(byte)232);
                Debug.Assert(pack.message_rate == (ushort)(ushort)22819);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)92;
            p67.on_off = (byte)(byte)232;
            p67.message_rate = (ushort)(ushort)22819;
            SMP_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.buttons == (ushort)(ushort)24430);
                Debug.Assert(pack.target == (byte)(byte)30);
                Debug.Assert(pack.r == (short)(short) -6602);
                Debug.Assert(pack.z == (short)(short)19571);
                Debug.Assert(pack.y == (short)(short)25352);
                Debug.Assert(pack.x == (short)(short)2355);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.r = (short)(short) -6602;
            p69.x = (short)(short)2355;
            p69.z = (short)(short)19571;
            p69.target = (byte)(byte)30;
            p69.buttons = (ushort)(ushort)24430;
            p69.y = (short)(short)25352;
            SMP_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)26156);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)50523);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)19040);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)23136);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)32812);
                Debug.Assert(pack.target_system == (byte)(byte)69);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)22040);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)30527);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)9719);
                Debug.Assert(pack.target_component == (byte)(byte)178);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan1_raw = (ushort)(ushort)50523;
            p70.target_component = (byte)(byte)178;
            p70.chan4_raw = (ushort)(ushort)26156;
            p70.chan5_raw = (ushort)(ushort)9719;
            p70.chan7_raw = (ushort)(ushort)19040;
            p70.chan2_raw = (ushort)(ushort)22040;
            p70.chan3_raw = (ushort)(ushort)32812;
            p70.chan8_raw = (ushort)(ushort)30527;
            p70.target_system = (byte)(byte)69;
            p70.chan6_raw = (ushort)(ushort)23136;
            SMP_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (int) -226234722);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_USER_1);
                Debug.Assert(pack.target_component == (byte)(byte)57);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.current == (byte)(byte)191);
                Debug.Assert(pack.seq == (ushort)(ushort)35198);
                Debug.Assert(pack.param3 == (float)2.5135708E38F);
                Debug.Assert(pack.y == (int)1586521986);
                Debug.Assert(pack.param4 == (float)1.0412066E38F);
                Debug.Assert(pack.param2 == (float)3.6544285E36F);
                Debug.Assert(pack.param1 == (float)3.1779498E38F);
                Debug.Assert(pack.target_system == (byte)(byte)60);
                Debug.Assert(pack.z == (float)1.7955192E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)66);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.param3 = (float)2.5135708E38F;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_USER_1;
            p73.autocontinue = (byte)(byte)66;
            p73.param1 = (float)3.1779498E38F;
            p73.current = (byte)(byte)191;
            p73.y = (int)1586521986;
            p73.target_component = (byte)(byte)57;
            p73.param2 = (float)3.6544285E36F;
            p73.param4 = (float)1.0412066E38F;
            p73.z = (float)1.7955192E38F;
            p73.x = (int) -226234722;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p73.seq = (ushort)(ushort)35198;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p73.target_system = (byte)(byte)60;
            SMP_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.groundspeed == (float) -5.8393777E37F);
                Debug.Assert(pack.heading == (short)(short) -19703);
                Debug.Assert(pack.alt == (float) -6.0522913E37F);
                Debug.Assert(pack.airspeed == (float)1.7378208E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)11499);
                Debug.Assert(pack.climb == (float)2.7534294E38F);
            };
            GroundControl.VFR_HUD p74 = CommunicationChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.climb = (float)2.7534294E38F;
            p74.throttle = (ushort)(ushort)11499;
            p74.alt = (float) -6.0522913E37F;
            p74.groundspeed = (float) -5.8393777E37F;
            p74.airspeed = (float)1.7378208E38F;
            p74.heading = (short)(short) -19703;
            CommunicationChannel.instance.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param4 == (float) -3.1370596E38F);
                Debug.Assert(pack.x == (int)1970516179);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.current == (byte)(byte)253);
                Debug.Assert(pack.target_component == (byte)(byte)5);
                Debug.Assert(pack.param1 == (float)8.827172E37F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION);
                Debug.Assert(pack.target_system == (byte)(byte)181);
                Debug.Assert(pack.y == (int)476621106);
                Debug.Assert(pack.param3 == (float) -7.0144614E37F);
                Debug.Assert(pack.autocontinue == (byte)(byte)127);
                Debug.Assert(pack.param2 == (float)4.334591E36F);
                Debug.Assert(pack.z == (float) -3.0266154E38F);
            };
            GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.current = (byte)(byte)253;
            p75.target_component = (byte)(byte)5;
            p75.z = (float) -3.0266154E38F;
            p75.x = (int)1970516179;
            p75.param2 = (float)4.334591E36F;
            p75.param4 = (float) -3.1370596E38F;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION;
            p75.param1 = (float)8.827172E37F;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p75.autocontinue = (byte)(byte)127;
            p75.y = (int)476621106;
            p75.param3 = (float) -7.0144614E37F;
            p75.target_system = (byte)(byte)181;
            CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param4 == (float)2.7802623E37F);
                Debug.Assert(pack.param1 == (float) -2.8240925E38F);
                Debug.Assert(pack.target_component == (byte)(byte)35);
                Debug.Assert(pack.param6 == (float) -3.0611026E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_PANORAMA_CREATE);
                Debug.Assert(pack.param7 == (float) -2.8413672E38F);
                Debug.Assert(pack.param3 == (float)2.1266707E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)191);
                Debug.Assert(pack.param5 == (float)2.5684272E38F);
                Debug.Assert(pack.param2 == (float)9.314238E37F);
                Debug.Assert(pack.target_system == (byte)(byte)85);
            };
            GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.param3 = (float)2.1266707E38F;
            p76.param4 = (float)2.7802623E37F;
            p76.param6 = (float) -3.0611026E38F;
            p76.param5 = (float)2.5684272E38F;
            p76.confirmation = (byte)(byte)191;
            p76.target_system = (byte)(byte)85;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_PANORAMA_CREATE;
            p76.param1 = (float) -2.8240925E38F;
            p76.param2 = (float)9.314238E37F;
            p76.target_component = (byte)(byte)35;
            p76.param7 = (float) -2.8413672E38F;
            CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_IN_PROGRESS);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)93);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_CHANGE_SPEED);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)36);
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -2045231928);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)209);
            };
            GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.target_component_SET((byte)(byte)36, PH) ;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_IN_PROGRESS;
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_CHANGE_SPEED;
            p77.result_param2_SET((int) -2045231928, PH) ;
            p77.target_system_SET((byte)(byte)209, PH) ;
            p77.progress_SET((byte)(byte)93, PH) ;
            CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2005163667U);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)22);
                Debug.Assert(pack.pitch == (float)4.950369E37F);
                Debug.Assert(pack.mode_switch == (byte)(byte)206);
                Debug.Assert(pack.yaw == (float)2.5670154E38F);
                Debug.Assert(pack.thrust == (float) -7.9987977E37F);
                Debug.Assert(pack.roll == (float) -1.867339E38F);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.manual_override_switch = (byte)(byte)22;
            p81.thrust = (float) -7.9987977E37F;
            p81.pitch = (float)4.950369E37F;
            p81.yaw = (float)2.5670154E38F;
            p81.mode_switch = (byte)(byte)206;
            p81.time_boot_ms = (uint)2005163667U;
            p81.roll = (float) -1.867339E38F;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.7638273E38F, -2.1158145E38F, 2.0498967E38F, 1.2790614E38F}));
                Debug.Assert(pack.body_pitch_rate == (float) -2.8523828E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3129698937U);
                Debug.Assert(pack.body_roll_rate == (float)1.0350489E38F);
                Debug.Assert(pack.body_yaw_rate == (float)3.2018455E38F);
                Debug.Assert(pack.target_system == (byte)(byte)170);
                Debug.Assert(pack.type_mask == (byte)(byte)188);
                Debug.Assert(pack.target_component == (byte)(byte)78);
                Debug.Assert(pack.thrust == (float)2.7985042E38F);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.q_SET(new float[] {-1.7638273E38F, -2.1158145E38F, 2.0498967E38F, 1.2790614E38F}, 0) ;
            p82.body_yaw_rate = (float)3.2018455E38F;
            p82.body_pitch_rate = (float) -2.8523828E38F;
            p82.target_component = (byte)(byte)78;
            p82.type_mask = (byte)(byte)188;
            p82.body_roll_rate = (float)1.0350489E38F;
            p82.target_system = (byte)(byte)170;
            p82.time_boot_ms = (uint)3129698937U;
            p82.thrust = (float)2.7985042E38F;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_pitch_rate == (float)2.1912545E38F);
                Debug.Assert(pack.thrust == (float) -1.582558E37F);
                Debug.Assert(pack.body_yaw_rate == (float) -2.1365932E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)111);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.6928197E38F, -1.2304358E38F, -1.5583536E38F, 1.6732921E38F}));
                Debug.Assert(pack.time_boot_ms == (uint)2675418172U);
                Debug.Assert(pack.body_roll_rate == (float)2.8397323E38F);
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_roll_rate = (float)2.8397323E38F;
            p83.thrust = (float) -1.582558E37F;
            p83.time_boot_ms = (uint)2675418172U;
            p83.q_SET(new float[] {2.6928197E38F, -1.2304358E38F, -1.5583536E38F, 1.6732921E38F}, 0) ;
            p83.body_pitch_rate = (float)2.1912545E38F;
            p83.type_mask = (byte)(byte)111;
            p83.body_yaw_rate = (float) -2.1365932E38F;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)82);
                Debug.Assert(pack.yaw_rate == (float) -4.5529158E36F);
                Debug.Assert(pack.z == (float)3.2724747E38F);
                Debug.Assert(pack.afy == (float)1.9236588E38F);
                Debug.Assert(pack.afx == (float)3.5222382E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)59600);
                Debug.Assert(pack.time_boot_ms == (uint)1171254071U);
                Debug.Assert(pack.vx == (float)1.5296005E38F);
                Debug.Assert(pack.afz == (float)4.58775E37F);
                Debug.Assert(pack.vy == (float)3.3223331E38F);
                Debug.Assert(pack.y == (float) -2.540991E38F);
                Debug.Assert(pack.x == (float) -7.195292E37F);
                Debug.Assert(pack.yaw == (float) -2.660962E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.target_component == (byte)(byte)172);
                Debug.Assert(pack.vz == (float)7.5299555E36F);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.y = (float) -2.540991E38F;
            p84.vz = (float)7.5299555E36F;
            p84.type_mask = (ushort)(ushort)59600;
            p84.afz = (float)4.58775E37F;
            p84.target_component = (byte)(byte)172;
            p84.afy = (float)1.9236588E38F;
            p84.vx = (float)1.5296005E38F;
            p84.yaw_rate = (float) -4.5529158E36F;
            p84.time_boot_ms = (uint)1171254071U;
            p84.vy = (float)3.3223331E38F;
            p84.target_system = (byte)(byte)82;
            p84.x = (float) -7.195292E37F;
            p84.yaw = (float) -2.660962E38F;
            p84.z = (float)3.2724747E38F;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p84.afx = (float)3.5222382E37F;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat_int == (int)1199058596);
                Debug.Assert(pack.time_boot_ms == (uint)4089318573U);
                Debug.Assert(pack.target_system == (byte)(byte)29);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.afx == (float) -2.1452134E37F);
                Debug.Assert(pack.afz == (float)2.3492924E38F);
                Debug.Assert(pack.vx == (float)2.2956945E38F);
                Debug.Assert(pack.yaw == (float)1.1898718E38F);
                Debug.Assert(pack.vy == (float)6.1528256E37F);
                Debug.Assert(pack.alt == (float)2.2128517E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)48745);
                Debug.Assert(pack.lon_int == (int)891598079);
                Debug.Assert(pack.yaw_rate == (float) -2.559295E38F);
                Debug.Assert(pack.vz == (float)2.9051598E37F);
                Debug.Assert(pack.target_component == (byte)(byte)227);
                Debug.Assert(pack.afy == (float)6.3097527E37F);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.afx = (float) -2.1452134E37F;
            p86.target_system = (byte)(byte)29;
            p86.vz = (float)2.9051598E37F;
            p86.target_component = (byte)(byte)227;
            p86.afy = (float)6.3097527E37F;
            p86.alt = (float)2.2128517E38F;
            p86.type_mask = (ushort)(ushort)48745;
            p86.yaw = (float)1.1898718E38F;
            p86.afz = (float)2.3492924E38F;
            p86.time_boot_ms = (uint)4089318573U;
            p86.vx = (float)2.2956945E38F;
            p86.lat_int = (int)1199058596;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p86.yaw_rate = (float) -2.559295E38F;
            p86.lon_int = (int)891598079;
            p86.vy = (float)6.1528256E37F;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -2.8779417E38F);
                Debug.Assert(pack.yaw_rate == (float) -1.0102292E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.time_boot_ms == (uint)1556576078U);
                Debug.Assert(pack.lon_int == (int) -656954453);
                Debug.Assert(pack.afx == (float)2.8144997E38F);
                Debug.Assert(pack.afy == (float)1.6031961E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)6200);
                Debug.Assert(pack.alt == (float) -1.4974662E38F);
                Debug.Assert(pack.vz == (float) -2.0055636E38F);
                Debug.Assert(pack.afz == (float) -2.4592525E38F);
                Debug.Assert(pack.vx == (float) -2.6288505E37F);
                Debug.Assert(pack.lat_int == (int)857723513);
                Debug.Assert(pack.yaw == (float)3.0575578E38F);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.vx = (float) -2.6288505E37F;
            p87.lat_int = (int)857723513;
            p87.type_mask = (ushort)(ushort)6200;
            p87.afz = (float) -2.4592525E38F;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p87.alt = (float) -1.4974662E38F;
            p87.yaw_rate = (float) -1.0102292E38F;
            p87.afx = (float)2.8144997E38F;
            p87.lon_int = (int) -656954453;
            p87.afy = (float)1.6031961E38F;
            p87.vy = (float) -2.8779417E38F;
            p87.vz = (float) -2.0055636E38F;
            p87.time_boot_ms = (uint)1556576078U;
            p87.yaw = (float)3.0575578E38F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -3.3628756E38F);
                Debug.Assert(pack.time_boot_ms == (uint)249469117U);
                Debug.Assert(pack.yaw == (float) -1.1754833E38F);
                Debug.Assert(pack.roll == (float) -2.3907318E38F);
                Debug.Assert(pack.x == (float) -6.76248E37F);
                Debug.Assert(pack.z == (float)1.8287209E38F);
                Debug.Assert(pack.y == (float) -1.4470937E38F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.y = (float) -1.4470937E38F;
            p89.x = (float) -6.76248E37F;
            p89.z = (float)1.8287209E38F;
            p89.pitch = (float) -3.3628756E38F;
            p89.roll = (float) -2.3907318E38F;
            p89.yaw = (float) -1.1754833E38F;
            p89.time_boot_ms = (uint)249469117U;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (short)(short) -17168);
                Debug.Assert(pack.roll == (float) -2.0841683E38F);
                Debug.Assert(pack.zacc == (short)(short) -22454);
                Debug.Assert(pack.pitchspeed == (float)1.9680312E38F);
                Debug.Assert(pack.pitch == (float) -7.7886E37F);
                Debug.Assert(pack.yaw == (float) -2.9983224E38F);
                Debug.Assert(pack.time_usec == (ulong)7519995025153816883L);
                Debug.Assert(pack.lon == (int)1849163215);
                Debug.Assert(pack.vz == (short)(short)30795);
                Debug.Assert(pack.vx == (short)(short)3512);
                Debug.Assert(pack.xacc == (short)(short)82);
                Debug.Assert(pack.yacc == (short)(short) -4362);
                Debug.Assert(pack.yawspeed == (float)1.2986109E38F);
                Debug.Assert(pack.alt == (int) -46762579);
                Debug.Assert(pack.lat == (int)1040760825);
                Debug.Assert(pack.rollspeed == (float)2.25984E38F);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.vz = (short)(short)30795;
            p90.vx = (short)(short)3512;
            p90.xacc = (short)(short)82;
            p90.time_usec = (ulong)7519995025153816883L;
            p90.lat = (int)1040760825;
            p90.vy = (short)(short) -17168;
            p90.zacc = (short)(short) -22454;
            p90.lon = (int)1849163215;
            p90.alt = (int) -46762579;
            p90.yacc = (short)(short) -4362;
            p90.yawspeed = (float)1.2986109E38F;
            p90.yaw = (float) -2.9983224E38F;
            p90.roll = (float) -2.0841683E38F;
            p90.pitchspeed = (float)1.9680312E38F;
            p90.rollspeed = (float)2.25984E38F;
            p90.pitch = (float) -7.7886E37F;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_mode == (byte)(byte)114);
                Debug.Assert(pack.pitch_elevator == (float) -2.2576344E38F);
                Debug.Assert(pack.throttle == (float) -2.2310141E38F);
                Debug.Assert(pack.aux4 == (float)6.6098456E37F);
                Debug.Assert(pack.yaw_rudder == (float) -5.5249476E37F);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
                Debug.Assert(pack.aux2 == (float)2.2995827E38F);
                Debug.Assert(pack.roll_ailerons == (float)1.3159048E38F);
                Debug.Assert(pack.aux3 == (float)2.7166882E38F);
                Debug.Assert(pack.aux1 == (float) -9.626144E37F);
                Debug.Assert(pack.time_usec == (ulong)5949390759776371684L);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            p91.time_usec = (ulong)5949390759776371684L;
            p91.aux2 = (float)2.2995827E38F;
            p91.aux1 = (float) -9.626144E37F;
            p91.nav_mode = (byte)(byte)114;
            p91.throttle = (float) -2.2310141E38F;
            p91.aux3 = (float)2.7166882E38F;
            p91.aux4 = (float)6.6098456E37F;
            p91.pitch_elevator = (float) -2.2576344E38F;
            p91.yaw_rudder = (float) -5.5249476E37F;
            p91.roll_ailerons = (float)1.3159048E38F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)28304);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)31096);
                Debug.Assert(pack.time_usec == (ulong)3557156844398715407L);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)6507);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)26303);
                Debug.Assert(pack.rssi == (byte)(byte)249);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)27190);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)57684);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)25353);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)32541);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)52556);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)31463);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)55876);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)63457);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan1_raw = (ushort)(ushort)31096;
            p92.chan3_raw = (ushort)(ushort)28304;
            p92.chan10_raw = (ushort)(ushort)63457;
            p92.chan6_raw = (ushort)(ushort)52556;
            p92.chan7_raw = (ushort)(ushort)31463;
            p92.chan8_raw = (ushort)(ushort)55876;
            p92.time_usec = (ulong)3557156844398715407L;
            p92.chan11_raw = (ushort)(ushort)26303;
            p92.chan2_raw = (ushort)(ushort)57684;
            p92.chan5_raw = (ushort)(ushort)25353;
            p92.chan12_raw = (ushort)(ushort)27190;
            p92.rssi = (byte)(byte)249;
            p92.chan4_raw = (ushort)(ushort)32541;
            p92.chan9_raw = (ushort)(ushort)6507;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)8819323167337579549L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {5.5636997E37F, -1.0366636E38F, -1.6491035E38F, 2.6917402E37F, -2.7986001E38F, -3.2438729E38F, 1.9262196E38F, 1.8710314E38F, 3.240153E38F, -1.9968468E37F, 1.4905646E38F, 9.682525E37F, 8.91844E37F, -2.3637942E38F, 8.701892E37F, -2.7537981E38F}));
                Debug.Assert(pack.flags == (ulong)8013970390048223809L);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_ARMED);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_ARMED;
            p93.controls_SET(new float[] {5.5636997E37F, -1.0366636E38F, -1.6491035E38F, 2.6917402E37F, -2.7986001E38F, -3.2438729E38F, 1.9262196E38F, 1.8710314E38F, 3.240153E38F, -1.9968468E37F, 1.4905646E38F, 9.682525E37F, 8.91844E37F, -2.3637942E38F, 8.701892E37F, -2.7537981E38F}, 0) ;
            p93.flags = (ulong)8013970390048223809L;
            p93.time_usec = (ulong)8819323167337579549L;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6070591305459097964L);
                Debug.Assert(pack.flow_comp_m_x == (float) -9.845857E37F);
                Debug.Assert(pack.flow_comp_m_y == (float)1.8947959E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)93);
                Debug.Assert(pack.ground_distance == (float) -2.4079755E38F);
                Debug.Assert(pack.flow_x == (short)(short)12667);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -7.362821E37F);
                Debug.Assert(pack.quality == (byte)(byte)216);
                Debug.Assert(pack.flow_y == (short)(short) -21048);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -2.3234861E38F);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_rate_x_SET((float) -2.3234861E38F, PH) ;
            p100.flow_x = (short)(short)12667;
            p100.time_usec = (ulong)6070591305459097964L;
            p100.ground_distance = (float) -2.4079755E38F;
            p100.quality = (byte)(byte)216;
            p100.flow_rate_y_SET((float) -7.362821E37F, PH) ;
            p100.flow_comp_m_y = (float)1.8947959E38F;
            p100.sensor_id = (byte)(byte)93;
            p100.flow_y = (short)(short) -21048;
            p100.flow_comp_m_x = (float) -9.845857E37F;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -2.5543016E38F);
                Debug.Assert(pack.yaw == (float)1.4861343E38F);
                Debug.Assert(pack.usec == (ulong)5613517244656533855L);
                Debug.Assert(pack.y == (float)1.4089418E38F);
                Debug.Assert(pack.z == (float) -4.2619194E37F);
                Debug.Assert(pack.pitch == (float) -6.1967264E37F);
                Debug.Assert(pack.roll == (float)3.316284E37F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.y = (float)1.4089418E38F;
            p101.pitch = (float) -6.1967264E37F;
            p101.yaw = (float)1.4861343E38F;
            p101.usec = (ulong)5613517244656533855L;
            p101.roll = (float)3.316284E37F;
            p101.x = (float) -2.5543016E38F;
            p101.z = (float) -4.2619194E37F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -1.2902181E38F);
                Debug.Assert(pack.pitch == (float)5.633763E36F);
                Debug.Assert(pack.usec == (ulong)7158565631131267372L);
                Debug.Assert(pack.yaw == (float) -2.3092174E38F);
                Debug.Assert(pack.y == (float) -9.332465E36F);
                Debug.Assert(pack.x == (float) -7.9222443E37F);
                Debug.Assert(pack.z == (float)1.4685819E38F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.roll = (float) -1.2902181E38F;
            p102.y = (float) -9.332465E36F;
            p102.yaw = (float) -2.3092174E38F;
            p102.pitch = (float)5.633763E36F;
            p102.x = (float) -7.9222443E37F;
            p102.usec = (ulong)7158565631131267372L;
            p102.z = (float)1.4685819E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)7.207601E37F);
                Debug.Assert(pack.usec == (ulong)7619267773209731207L);
                Debug.Assert(pack.x == (float) -3.1143845E38F);
                Debug.Assert(pack.y == (float) -1.5861094E37F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)7619267773209731207L;
            p103.y = (float) -1.5861094E37F;
            p103.z = (float)7.207601E37F;
            p103.x = (float) -3.1143845E38F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)2.7218588E38F);
                Debug.Assert(pack.y == (float) -2.6678005E38F);
                Debug.Assert(pack.roll == (float)3.0277155E38F);
                Debug.Assert(pack.usec == (ulong)7733133622654335365L);
                Debug.Assert(pack.yaw == (float) -9.843307E37F);
                Debug.Assert(pack.z == (float) -3.589044E37F);
                Debug.Assert(pack.pitch == (float)1.1664958E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.roll = (float)3.0277155E38F;
            p104.z = (float) -3.589044E37F;
            p104.usec = (ulong)7733133622654335365L;
            p104.x = (float)2.7218588E38F;
            p104.pitch = (float)1.1664958E38F;
            p104.y = (float) -2.6678005E38F;
            p104.yaw = (float) -9.843307E37F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (float) -1.7795523E38F);
                Debug.Assert(pack.xmag == (float) -6.981097E37F);
                Debug.Assert(pack.pressure_alt == (float)1.7906523E37F);
                Debug.Assert(pack.xacc == (float)1.5756798E37F);
                Debug.Assert(pack.yacc == (float)4.7083246E37F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)18240);
                Debug.Assert(pack.zgyro == (float) -3.1991437E38F);
                Debug.Assert(pack.time_usec == (ulong)76494343289426308L);
                Debug.Assert(pack.xgyro == (float) -1.4437008E37F);
                Debug.Assert(pack.temperature == (float)2.9534346E38F);
                Debug.Assert(pack.ygyro == (float)3.5511305E36F);
                Debug.Assert(pack.diff_pressure == (float) -3.8975842E37F);
                Debug.Assert(pack.ymag == (float)8.699803E37F);
                Debug.Assert(pack.abs_pressure == (float)8.4799533E37F);
                Debug.Assert(pack.zmag == (float) -3.2284422E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.zmag = (float) -3.2284422E38F;
            p105.pressure_alt = (float)1.7906523E37F;
            p105.xgyro = (float) -1.4437008E37F;
            p105.fields_updated = (ushort)(ushort)18240;
            p105.xacc = (float)1.5756798E37F;
            p105.ymag = (float)8.699803E37F;
            p105.diff_pressure = (float) -3.8975842E37F;
            p105.xmag = (float) -6.981097E37F;
            p105.zacc = (float) -1.7795523E38F;
            p105.yacc = (float)4.7083246E37F;
            p105.ygyro = (float)3.5511305E36F;
            p105.zgyro = (float) -3.1991437E38F;
            p105.time_usec = (ulong)76494343289426308L;
            p105.temperature = (float)2.9534346E38F;
            p105.abs_pressure = (float)8.4799533E37F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_ygyro == (float)3.0505318E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)57);
                Debug.Assert(pack.quality == (byte)(byte)243);
                Debug.Assert(pack.time_usec == (ulong)145587671314274531L);
                Debug.Assert(pack.integrated_xgyro == (float)6.743307E37F);
                Debug.Assert(pack.distance == (float) -9.487269E37F);
                Debug.Assert(pack.integrated_zgyro == (float)9.13121E37F);
                Debug.Assert(pack.time_delta_distance_us == (uint)66125962U);
                Debug.Assert(pack.integrated_x == (float) -1.3527424E38F);
                Debug.Assert(pack.integrated_y == (float) -2.5811407E38F);
                Debug.Assert(pack.temperature == (short)(short)16399);
                Debug.Assert(pack.integration_time_us == (uint)1369661304U);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.quality = (byte)(byte)243;
            p106.integrated_y = (float) -2.5811407E38F;
            p106.integrated_ygyro = (float)3.0505318E38F;
            p106.distance = (float) -9.487269E37F;
            p106.integrated_x = (float) -1.3527424E38F;
            p106.time_usec = (ulong)145587671314274531L;
            p106.integration_time_us = (uint)1369661304U;
            p106.time_delta_distance_us = (uint)66125962U;
            p106.temperature = (short)(short)16399;
            p106.integrated_zgyro = (float)9.13121E37F;
            p106.integrated_xgyro = (float)6.743307E37F;
            p106.sensor_id = (byte)(byte)57;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pressure_alt == (float) -1.7245253E38F);
                Debug.Assert(pack.zacc == (float)1.5562699E38F);
                Debug.Assert(pack.yacc == (float) -3.8803147E37F);
                Debug.Assert(pack.zmag == (float) -9.321956E37F);
                Debug.Assert(pack.xmag == (float) -7.694602E37F);
                Debug.Assert(pack.temperature == (float)2.3021482E38F);
                Debug.Assert(pack.zgyro == (float)2.1700383E38F);
                Debug.Assert(pack.xgyro == (float)1.3873543E38F);
                Debug.Assert(pack.time_usec == (ulong)8515465074904075322L);
                Debug.Assert(pack.fields_updated == (uint)3438626528U);
                Debug.Assert(pack.ymag == (float) -8.178067E37F);
                Debug.Assert(pack.xacc == (float) -1.4071856E38F);
                Debug.Assert(pack.diff_pressure == (float) -1.5464319E38F);
                Debug.Assert(pack.ygyro == (float) -5.3471926E37F);
                Debug.Assert(pack.abs_pressure == (float)6.09091E37F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.zgyro = (float)2.1700383E38F;
            p107.zmag = (float) -9.321956E37F;
            p107.abs_pressure = (float)6.09091E37F;
            p107.pressure_alt = (float) -1.7245253E38F;
            p107.diff_pressure = (float) -1.5464319E38F;
            p107.ygyro = (float) -5.3471926E37F;
            p107.xmag = (float) -7.694602E37F;
            p107.xgyro = (float)1.3873543E38F;
            p107.fields_updated = (uint)3438626528U;
            p107.ymag = (float) -8.178067E37F;
            p107.temperature = (float)2.3021482E38F;
            p107.time_usec = (ulong)8515465074904075322L;
            p107.zacc = (float)1.5562699E38F;
            p107.yacc = (float) -3.8803147E37F;
            p107.xacc = (float) -1.4071856E38F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q3 == (float) -2.1030467E38F);
                Debug.Assert(pack.xacc == (float) -1.5330276E38F);
                Debug.Assert(pack.q2 == (float)1.3762999E38F);
                Debug.Assert(pack.std_dev_vert == (float)2.6917335E38F);
                Debug.Assert(pack.pitch == (float)1.051977E38F);
                Debug.Assert(pack.yaw == (float) -3.0425186E38F);
                Debug.Assert(pack.std_dev_horz == (float) -3.319159E38F);
                Debug.Assert(pack.zacc == (float) -2.4614692E38F);
                Debug.Assert(pack.vd == (float)3.4809189E37F);
                Debug.Assert(pack.vn == (float) -1.3915755E37F);
                Debug.Assert(pack.xgyro == (float)1.5062751E38F);
                Debug.Assert(pack.lon == (float) -2.3723598E38F);
                Debug.Assert(pack.zgyro == (float) -3.20253E38F);
                Debug.Assert(pack.roll == (float) -2.6644756E38F);
                Debug.Assert(pack.q4 == (float)2.1481625E37F);
                Debug.Assert(pack.alt == (float) -3.0467224E38F);
                Debug.Assert(pack.yacc == (float) -2.7468832E38F);
                Debug.Assert(pack.ygyro == (float) -2.8530515E38F);
                Debug.Assert(pack.q1 == (float) -1.3283825E38F);
                Debug.Assert(pack.ve == (float)2.1716171E38F);
                Debug.Assert(pack.lat == (float) -3.0852894E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.alt = (float) -3.0467224E38F;
            p108.vn = (float) -1.3915755E37F;
            p108.q3 = (float) -2.1030467E38F;
            p108.q2 = (float)1.3762999E38F;
            p108.std_dev_horz = (float) -3.319159E38F;
            p108.ygyro = (float) -2.8530515E38F;
            p108.yaw = (float) -3.0425186E38F;
            p108.xgyro = (float)1.5062751E38F;
            p108.zgyro = (float) -3.20253E38F;
            p108.xacc = (float) -1.5330276E38F;
            p108.yacc = (float) -2.7468832E38F;
            p108.lat = (float) -3.0852894E38F;
            p108.zacc = (float) -2.4614692E38F;
            p108.lon = (float) -2.3723598E38F;
            p108.vd = (float)3.4809189E37F;
            p108.ve = (float)2.1716171E38F;
            p108.std_dev_vert = (float)2.6917335E38F;
            p108.roll = (float) -2.6644756E38F;
            p108.q1 = (float) -1.3283825E38F;
            p108.pitch = (float)1.051977E38F;
            p108.q4 = (float)2.1481625E37F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.remrssi == (byte)(byte)11);
                Debug.Assert(pack.rssi == (byte)(byte)151);
                Debug.Assert(pack.remnoise == (byte)(byte)181);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)58684);
                Debug.Assert(pack.txbuf == (byte)(byte)222);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)25281);
                Debug.Assert(pack.noise == (byte)(byte)41);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.fixed_ = (ushort)(ushort)58684;
            p109.remnoise = (byte)(byte)181;
            p109.rxerrors = (ushort)(ushort)25281;
            p109.txbuf = (byte)(byte)222;
            p109.noise = (byte)(byte)41;
            p109.remrssi = (byte)(byte)11;
            p109.rssi = (byte)(byte)151;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)172);
                Debug.Assert(pack.target_network == (byte)(byte)231);
                Debug.Assert(pack.target_system == (byte)(byte)241);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)62, (byte)53, (byte)227, (byte)149, (byte)93, (byte)154, (byte)213, (byte)92, (byte)170, (byte)72, (byte)15, (byte)89, (byte)97, (byte)78, (byte)185, (byte)162, (byte)10, (byte)59, (byte)241, (byte)168, (byte)134, (byte)169, (byte)100, (byte)9, (byte)88, (byte)150, (byte)138, (byte)48, (byte)185, (byte)116, (byte)167, (byte)184, (byte)247, (byte)77, (byte)189, (byte)176, (byte)16, (byte)89, (byte)212, (byte)34, (byte)163, (byte)252, (byte)134, (byte)55, (byte)27, (byte)22, (byte)13, (byte)84, (byte)67, (byte)193, (byte)118, (byte)204, (byte)27, (byte)225, (byte)193, (byte)70, (byte)8, (byte)43, (byte)243, (byte)109, (byte)97, (byte)154, (byte)39, (byte)41, (byte)57, (byte)148, (byte)241, (byte)139, (byte)14, (byte)158, (byte)250, (byte)209, (byte)212, (byte)50, (byte)233, (byte)156, (byte)159, (byte)209, (byte)96, (byte)64, (byte)195, (byte)207, (byte)11, (byte)255, (byte)133, (byte)74, (byte)65, (byte)130, (byte)52, (byte)135, (byte)255, (byte)133, (byte)90, (byte)67, (byte)16, (byte)133, (byte)78, (byte)31, (byte)196, (byte)3, (byte)145, (byte)145, (byte)88, (byte)194, (byte)10, (byte)45, (byte)202, (byte)163, (byte)169, (byte)28, (byte)72, (byte)109, (byte)121, (byte)214, (byte)119, (byte)147, (byte)249, (byte)101, (byte)112, (byte)191, (byte)201, (byte)29, (byte)67, (byte)161, (byte)249, (byte)136, (byte)6, (byte)143, (byte)151, (byte)78, (byte)65, (byte)177, (byte)5, (byte)201, (byte)232, (byte)33, (byte)228, (byte)20, (byte)213, (byte)238, (byte)20, (byte)121, (byte)82, (byte)128, (byte)80, (byte)24, (byte)208, (byte)239, (byte)230, (byte)148, (byte)64, (byte)71, (byte)121, (byte)227, (byte)41, (byte)215, (byte)140, (byte)130, (byte)69, (byte)233, (byte)182, (byte)123, (byte)61, (byte)210, (byte)47, (byte)108, (byte)201, (byte)253, (byte)120, (byte)204, (byte)69, (byte)87, (byte)232, (byte)187, (byte)223, (byte)187, (byte)138, (byte)22, (byte)96, (byte)166, (byte)184, (byte)131, (byte)118, (byte)67, (byte)203, (byte)108, (byte)240, (byte)212, (byte)143, (byte)25, (byte)62, (byte)239, (byte)223, (byte)129, (byte)82, (byte)25, (byte)74, (byte)11, (byte)20, (byte)5, (byte)245, (byte)230, (byte)250, (byte)54, (byte)175, (byte)110, (byte)220, (byte)180, (byte)100, (byte)90, (byte)87, (byte)74, (byte)141, (byte)232, (byte)84, (byte)9, (byte)144, (byte)160, (byte)33, (byte)202, (byte)235, (byte)61, (byte)109, (byte)85, (byte)138, (byte)173, (byte)133, (byte)101, (byte)67, (byte)107, (byte)220, (byte)26, (byte)135, (byte)250, (byte)182, (byte)101, (byte)212, (byte)116, (byte)184, (byte)77, (byte)178, (byte)139, (byte)73, (byte)187, (byte)223, (byte)104, (byte)29, (byte)196, (byte)203, (byte)28, (byte)233}));
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_component = (byte)(byte)172;
            p110.payload_SET(new byte[] {(byte)62, (byte)53, (byte)227, (byte)149, (byte)93, (byte)154, (byte)213, (byte)92, (byte)170, (byte)72, (byte)15, (byte)89, (byte)97, (byte)78, (byte)185, (byte)162, (byte)10, (byte)59, (byte)241, (byte)168, (byte)134, (byte)169, (byte)100, (byte)9, (byte)88, (byte)150, (byte)138, (byte)48, (byte)185, (byte)116, (byte)167, (byte)184, (byte)247, (byte)77, (byte)189, (byte)176, (byte)16, (byte)89, (byte)212, (byte)34, (byte)163, (byte)252, (byte)134, (byte)55, (byte)27, (byte)22, (byte)13, (byte)84, (byte)67, (byte)193, (byte)118, (byte)204, (byte)27, (byte)225, (byte)193, (byte)70, (byte)8, (byte)43, (byte)243, (byte)109, (byte)97, (byte)154, (byte)39, (byte)41, (byte)57, (byte)148, (byte)241, (byte)139, (byte)14, (byte)158, (byte)250, (byte)209, (byte)212, (byte)50, (byte)233, (byte)156, (byte)159, (byte)209, (byte)96, (byte)64, (byte)195, (byte)207, (byte)11, (byte)255, (byte)133, (byte)74, (byte)65, (byte)130, (byte)52, (byte)135, (byte)255, (byte)133, (byte)90, (byte)67, (byte)16, (byte)133, (byte)78, (byte)31, (byte)196, (byte)3, (byte)145, (byte)145, (byte)88, (byte)194, (byte)10, (byte)45, (byte)202, (byte)163, (byte)169, (byte)28, (byte)72, (byte)109, (byte)121, (byte)214, (byte)119, (byte)147, (byte)249, (byte)101, (byte)112, (byte)191, (byte)201, (byte)29, (byte)67, (byte)161, (byte)249, (byte)136, (byte)6, (byte)143, (byte)151, (byte)78, (byte)65, (byte)177, (byte)5, (byte)201, (byte)232, (byte)33, (byte)228, (byte)20, (byte)213, (byte)238, (byte)20, (byte)121, (byte)82, (byte)128, (byte)80, (byte)24, (byte)208, (byte)239, (byte)230, (byte)148, (byte)64, (byte)71, (byte)121, (byte)227, (byte)41, (byte)215, (byte)140, (byte)130, (byte)69, (byte)233, (byte)182, (byte)123, (byte)61, (byte)210, (byte)47, (byte)108, (byte)201, (byte)253, (byte)120, (byte)204, (byte)69, (byte)87, (byte)232, (byte)187, (byte)223, (byte)187, (byte)138, (byte)22, (byte)96, (byte)166, (byte)184, (byte)131, (byte)118, (byte)67, (byte)203, (byte)108, (byte)240, (byte)212, (byte)143, (byte)25, (byte)62, (byte)239, (byte)223, (byte)129, (byte)82, (byte)25, (byte)74, (byte)11, (byte)20, (byte)5, (byte)245, (byte)230, (byte)250, (byte)54, (byte)175, (byte)110, (byte)220, (byte)180, (byte)100, (byte)90, (byte)87, (byte)74, (byte)141, (byte)232, (byte)84, (byte)9, (byte)144, (byte)160, (byte)33, (byte)202, (byte)235, (byte)61, (byte)109, (byte)85, (byte)138, (byte)173, (byte)133, (byte)101, (byte)67, (byte)107, (byte)220, (byte)26, (byte)135, (byte)250, (byte)182, (byte)101, (byte)212, (byte)116, (byte)184, (byte)77, (byte)178, (byte)139, (byte)73, (byte)187, (byte)223, (byte)104, (byte)29, (byte)196, (byte)203, (byte)28, (byte)233}, 0) ;
            p110.target_system = (byte)(byte)241;
            p110.target_network = (byte)(byte)231;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long) -5058357558372993663L);
                Debug.Assert(pack.tc1 == (long)2560314922788681974L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long)2560314922788681974L;
            p111.ts1 = (long) -5058357558372993663L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7576328086119621826L);
                Debug.Assert(pack.seq == (uint)2709448716U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)7576328086119621826L;
            p112.seq = (uint)2709448716U;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vd == (short)(short) -32271);
                Debug.Assert(pack.fix_type == (byte)(byte)213);
                Debug.Assert(pack.alt == (int) -1479388848);
                Debug.Assert(pack.ve == (short)(short) -11513);
                Debug.Assert(pack.time_usec == (ulong)6381167799336262234L);
                Debug.Assert(pack.eph == (ushort)(ushort)43100);
                Debug.Assert(pack.vel == (ushort)(ushort)2759);
                Debug.Assert(pack.lon == (int)634570284);
                Debug.Assert(pack.vn == (short)(short)11477);
                Debug.Assert(pack.cog == (ushort)(ushort)27227);
                Debug.Assert(pack.lat == (int)1795198468);
                Debug.Assert(pack.satellites_visible == (byte)(byte)204);
                Debug.Assert(pack.epv == (ushort)(ushort)23178);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.lat = (int)1795198468;
            p113.satellites_visible = (byte)(byte)204;
            p113.eph = (ushort)(ushort)43100;
            p113.cog = (ushort)(ushort)27227;
            p113.epv = (ushort)(ushort)23178;
            p113.vd = (short)(short) -32271;
            p113.fix_type = (byte)(byte)213;
            p113.time_usec = (ulong)6381167799336262234L;
            p113.alt = (int) -1479388848;
            p113.vel = (ushort)(ushort)2759;
            p113.vn = (short)(short)11477;
            p113.lon = (int)634570284;
            p113.ve = (short)(short) -11513;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)4526);
                Debug.Assert(pack.integrated_zgyro == (float) -4.30889E37F);
                Debug.Assert(pack.time_delta_distance_us == (uint)1328802670U);
                Debug.Assert(pack.integrated_x == (float)1.3069312E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)45);
                Debug.Assert(pack.integrated_xgyro == (float) -7.3210407E37F);
                Debug.Assert(pack.integrated_ygyro == (float)6.7227634E37F);
                Debug.Assert(pack.distance == (float)2.6522208E38F);
                Debug.Assert(pack.integrated_y == (float) -7.517664E37F);
                Debug.Assert(pack.time_usec == (ulong)7831901238923805072L);
                Debug.Assert(pack.quality == (byte)(byte)152);
                Debug.Assert(pack.integration_time_us == (uint)164294431U);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integrated_ygyro = (float)6.7227634E37F;
            p114.sensor_id = (byte)(byte)45;
            p114.integration_time_us = (uint)164294431U;
            p114.distance = (float)2.6522208E38F;
            p114.time_usec = (ulong)7831901238923805072L;
            p114.integrated_zgyro = (float) -4.30889E37F;
            p114.integrated_xgyro = (float) -7.3210407E37F;
            p114.temperature = (short)(short)4526;
            p114.time_delta_distance_us = (uint)1328802670U;
            p114.integrated_x = (float)1.3069312E38F;
            p114.integrated_y = (float) -7.517664E37F;
            p114.quality = (byte)(byte)152;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short) -29797);
                Debug.Assert(pack.zacc == (short)(short) -25266);
                Debug.Assert(pack.vy == (short)(short)7176);
                Debug.Assert(pack.vz == (short)(short)22152);
                Debug.Assert(pack.lon == (int) -1234387107);
                Debug.Assert(pack.lat == (int)1040642632);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)29698);
                Debug.Assert(pack.vx == (short)(short) -23616);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)58267);
                Debug.Assert(pack.yawspeed == (float)1.1282766E38F);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {1.6749297E38F, 7.7437226E37F, 1.0156894E38F, -3.0366124E38F}));
                Debug.Assert(pack.rollspeed == (float) -2.9744095E38F);
                Debug.Assert(pack.xacc == (short)(short)31995);
                Debug.Assert(pack.alt == (int)1455841873);
                Debug.Assert(pack.pitchspeed == (float) -3.3746445E38F);
                Debug.Assert(pack.time_usec == (ulong)4577866321281487886L);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.rollspeed = (float) -2.9744095E38F;
            p115.true_airspeed = (ushort)(ushort)58267;
            p115.zacc = (short)(short) -25266;
            p115.alt = (int)1455841873;
            p115.lat = (int)1040642632;
            p115.vx = (short)(short) -23616;
            p115.yawspeed = (float)1.1282766E38F;
            p115.yacc = (short)(short) -29797;
            p115.vy = (short)(short)7176;
            p115.time_usec = (ulong)4577866321281487886L;
            p115.vz = (short)(short)22152;
            p115.lon = (int) -1234387107;
            p115.pitchspeed = (float) -3.3746445E38F;
            p115.attitude_quaternion_SET(new float[] {1.6749297E38F, 7.7437226E37F, 1.0156894E38F, -3.0366124E38F}, 0) ;
            p115.xacc = (short)(short)31995;
            p115.ind_airspeed = (ushort)(ushort)29698;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short) -24784);
                Debug.Assert(pack.ymag == (short)(short)14577);
                Debug.Assert(pack.zgyro == (short)(short) -14628);
                Debug.Assert(pack.xacc == (short)(short) -29572);
                Debug.Assert(pack.zacc == (short)(short) -3105);
                Debug.Assert(pack.zmag == (short)(short)32693);
                Debug.Assert(pack.xgyro == (short)(short) -6253);
                Debug.Assert(pack.xmag == (short)(short) -25522);
                Debug.Assert(pack.time_boot_ms == (uint)3898156574U);
                Debug.Assert(pack.yacc == (short)(short) -15303);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.ymag = (short)(short)14577;
            p116.ygyro = (short)(short) -24784;
            p116.zmag = (short)(short)32693;
            p116.xacc = (short)(short) -29572;
            p116.xmag = (short)(short) -25522;
            p116.zacc = (short)(short) -3105;
            p116.time_boot_ms = (uint)3898156574U;
            p116.yacc = (short)(short) -15303;
            p116.zgyro = (short)(short) -14628;
            p116.xgyro = (short)(short) -6253;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end == (ushort)(ushort)62584);
                Debug.Assert(pack.start == (ushort)(ushort)9639);
                Debug.Assert(pack.target_system == (byte)(byte)253);
                Debug.Assert(pack.target_component == (byte)(byte)156);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)253;
            p117.target_component = (byte)(byte)156;
            p117.end = (ushort)(ushort)62584;
            p117.start = (ushort)(ushort)9639;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)47895);
                Debug.Assert(pack.size == (uint)2967627839U);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)49002);
                Debug.Assert(pack.time_utc == (uint)1207564233U);
                Debug.Assert(pack.num_logs == (ushort)(ushort)58759);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.num_logs = (ushort)(ushort)58759;
            p118.id = (ushort)(ushort)47895;
            p118.time_utc = (uint)1207564233U;
            p118.last_log_num = (ushort)(ushort)49002;
            p118.size = (uint)2967627839U;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (uint)4054546209U);
                Debug.Assert(pack.target_system == (byte)(byte)116);
                Debug.Assert(pack.ofs == (uint)3656521606U);
                Debug.Assert(pack.id == (ushort)(ushort)26365);
                Debug.Assert(pack.target_component == (byte)(byte)68);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.id = (ushort)(ushort)26365;
            p119.target_component = (byte)(byte)68;
            p119.count = (uint)4054546209U;
            p119.ofs = (uint)3656521606U;
            p119.target_system = (byte)(byte)116;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)50301);
                Debug.Assert(pack.ofs == (uint)3575502283U);
                Debug.Assert(pack.count == (byte)(byte)130);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)168, (byte)239, (byte)85, (byte)165, (byte)121, (byte)119, (byte)229, (byte)84, (byte)200, (byte)102, (byte)133, (byte)97, (byte)241, (byte)240, (byte)185, (byte)15, (byte)241, (byte)202, (byte)55, (byte)66, (byte)234, (byte)53, (byte)121, (byte)155, (byte)187, (byte)226, (byte)63, (byte)162, (byte)180, (byte)195, (byte)183, (byte)137, (byte)180, (byte)150, (byte)3, (byte)233, (byte)102, (byte)18, (byte)84, (byte)134, (byte)178, (byte)241, (byte)28, (byte)65, (byte)16, (byte)41, (byte)165, (byte)78, (byte)244, (byte)234, (byte)8, (byte)60, (byte)210, (byte)135, (byte)227, (byte)182, (byte)31, (byte)205, (byte)157, (byte)241, (byte)94, (byte)58, (byte)79, (byte)48, (byte)103, (byte)39, (byte)84, (byte)69, (byte)87, (byte)155, (byte)235, (byte)11, (byte)76, (byte)215, (byte)157, (byte)56, (byte)118, (byte)44, (byte)238, (byte)42, (byte)237, (byte)159, (byte)190, (byte)251, (byte)178, (byte)149, (byte)33, (byte)117, (byte)254, (byte)62}));
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)50301;
            p120.ofs = (uint)3575502283U;
            p120.data__SET(new byte[] {(byte)168, (byte)239, (byte)85, (byte)165, (byte)121, (byte)119, (byte)229, (byte)84, (byte)200, (byte)102, (byte)133, (byte)97, (byte)241, (byte)240, (byte)185, (byte)15, (byte)241, (byte)202, (byte)55, (byte)66, (byte)234, (byte)53, (byte)121, (byte)155, (byte)187, (byte)226, (byte)63, (byte)162, (byte)180, (byte)195, (byte)183, (byte)137, (byte)180, (byte)150, (byte)3, (byte)233, (byte)102, (byte)18, (byte)84, (byte)134, (byte)178, (byte)241, (byte)28, (byte)65, (byte)16, (byte)41, (byte)165, (byte)78, (byte)244, (byte)234, (byte)8, (byte)60, (byte)210, (byte)135, (byte)227, (byte)182, (byte)31, (byte)205, (byte)157, (byte)241, (byte)94, (byte)58, (byte)79, (byte)48, (byte)103, (byte)39, (byte)84, (byte)69, (byte)87, (byte)155, (byte)235, (byte)11, (byte)76, (byte)215, (byte)157, (byte)56, (byte)118, (byte)44, (byte)238, (byte)42, (byte)237, (byte)159, (byte)190, (byte)251, (byte)178, (byte)149, (byte)33, (byte)117, (byte)254, (byte)62}, 0) ;
            p120.count = (byte)(byte)130;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)69);
                Debug.Assert(pack.target_component == (byte)(byte)91);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)69;
            p121.target_component = (byte)(byte)91;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)185);
                Debug.Assert(pack.target_component == (byte)(byte)75);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)75;
            p122.target_system = (byte)(byte)185;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)151);
                Debug.Assert(pack.len == (byte)(byte)21);
                Debug.Assert(pack.target_component == (byte)(byte)203);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)134, (byte)233, (byte)225, (byte)89, (byte)201, (byte)79, (byte)117, (byte)136, (byte)119, (byte)60, (byte)92, (byte)244, (byte)223, (byte)171, (byte)246, (byte)231, (byte)132, (byte)72, (byte)90, (byte)145, (byte)46, (byte)161, (byte)199, (byte)243, (byte)253, (byte)196, (byte)103, (byte)209, (byte)167, (byte)70, (byte)160, (byte)155, (byte)56, (byte)77, (byte)203, (byte)1, (byte)123, (byte)71, (byte)25, (byte)244, (byte)111, (byte)150, (byte)72, (byte)179, (byte)128, (byte)2, (byte)216, (byte)93, (byte)145, (byte)255, (byte)100, (byte)74, (byte)6, (byte)36, (byte)121, (byte)127, (byte)88, (byte)244, (byte)209, (byte)207, (byte)211, (byte)159, (byte)138, (byte)98, (byte)51, (byte)4, (byte)187, (byte)137, (byte)171, (byte)46, (byte)111, (byte)53, (byte)184, (byte)6, (byte)199, (byte)223, (byte)210, (byte)168, (byte)202, (byte)185, (byte)182, (byte)234, (byte)71, (byte)208, (byte)29, (byte)89, (byte)91, (byte)179, (byte)176, (byte)245, (byte)235, (byte)180, (byte)239, (byte)250, (byte)123, (byte)189, (byte)122, (byte)88, (byte)5, (byte)146, (byte)33, (byte)36, (byte)66, (byte)193, (byte)143, (byte)132, (byte)47, (byte)65, (byte)25, (byte)231}));
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.data__SET(new byte[] {(byte)134, (byte)233, (byte)225, (byte)89, (byte)201, (byte)79, (byte)117, (byte)136, (byte)119, (byte)60, (byte)92, (byte)244, (byte)223, (byte)171, (byte)246, (byte)231, (byte)132, (byte)72, (byte)90, (byte)145, (byte)46, (byte)161, (byte)199, (byte)243, (byte)253, (byte)196, (byte)103, (byte)209, (byte)167, (byte)70, (byte)160, (byte)155, (byte)56, (byte)77, (byte)203, (byte)1, (byte)123, (byte)71, (byte)25, (byte)244, (byte)111, (byte)150, (byte)72, (byte)179, (byte)128, (byte)2, (byte)216, (byte)93, (byte)145, (byte)255, (byte)100, (byte)74, (byte)6, (byte)36, (byte)121, (byte)127, (byte)88, (byte)244, (byte)209, (byte)207, (byte)211, (byte)159, (byte)138, (byte)98, (byte)51, (byte)4, (byte)187, (byte)137, (byte)171, (byte)46, (byte)111, (byte)53, (byte)184, (byte)6, (byte)199, (byte)223, (byte)210, (byte)168, (byte)202, (byte)185, (byte)182, (byte)234, (byte)71, (byte)208, (byte)29, (byte)89, (byte)91, (byte)179, (byte)176, (byte)245, (byte)235, (byte)180, (byte)239, (byte)250, (byte)123, (byte)189, (byte)122, (byte)88, (byte)5, (byte)146, (byte)33, (byte)36, (byte)66, (byte)193, (byte)143, (byte)132, (byte)47, (byte)65, (byte)25, (byte)231}, 0) ;
            p123.target_system = (byte)(byte)151;
            p123.len = (byte)(byte)21;
            p123.target_component = (byte)(byte)203;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -671604079);
                Debug.Assert(pack.alt == (int)116387020);
                Debug.Assert(pack.epv == (ushort)(ushort)4260);
                Debug.Assert(pack.dgps_age == (uint)1401897687U);
                Debug.Assert(pack.vel == (ushort)(ushort)37501);
                Debug.Assert(pack.dgps_numch == (byte)(byte)5);
                Debug.Assert(pack.lat == (int) -1197693011);
                Debug.Assert(pack.eph == (ushort)(ushort)9977);
                Debug.Assert(pack.cog == (ushort)(ushort)25033);
                Debug.Assert(pack.satellites_visible == (byte)(byte)235);
                Debug.Assert(pack.time_usec == (ulong)8136139317489380779L);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.satellites_visible = (byte)(byte)235;
            p124.vel = (ushort)(ushort)37501;
            p124.lon = (int) -671604079;
            p124.eph = (ushort)(ushort)9977;
            p124.dgps_numch = (byte)(byte)5;
            p124.dgps_age = (uint)1401897687U;
            p124.lat = (int) -1197693011;
            p124.time_usec = (ulong)8136139317489380779L;
            p124.cog = (ushort)(ushort)25033;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p124.alt = (int)116387020;
            p124.epv = (ushort)(ushort)4260;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vservo == (ushort)(ushort)39741);
                Debug.Assert(pack.Vcc == (ushort)(ushort)24542);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID;
            p125.Vcc = (ushort)(ushort)24542;
            p125.Vservo = (ushort)(ushort)39741;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)135, (byte)238, (byte)101, (byte)21, (byte)114, (byte)197, (byte)103, (byte)229, (byte)52, (byte)20, (byte)250, (byte)124, (byte)167, (byte)230, (byte)108, (byte)91, (byte)250, (byte)181, (byte)116, (byte)255, (byte)99, (byte)218, (byte)242, (byte)27, (byte)29, (byte)73, (byte)14, (byte)99, (byte)179, (byte)238, (byte)17, (byte)5, (byte)36, (byte)239, (byte)35, (byte)158, (byte)202, (byte)17, (byte)239, (byte)37, (byte)131, (byte)235, (byte)33, (byte)43, (byte)148, (byte)45, (byte)162, (byte)60, (byte)33, (byte)249, (byte)165, (byte)218, (byte)111, (byte)7, (byte)236, (byte)75, (byte)17, (byte)0, (byte)133, (byte)73, (byte)186, (byte)82, (byte)113, (byte)198, (byte)25, (byte)228, (byte)82, (byte)228, (byte)245, (byte)40}));
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
                Debug.Assert(pack.timeout == (ushort)(ushort)9306);
                Debug.Assert(pack.count == (byte)(byte)137);
                Debug.Assert(pack.baudrate == (uint)1924329502U);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.timeout = (ushort)(ushort)9306;
            p126.data__SET(new byte[] {(byte)135, (byte)238, (byte)101, (byte)21, (byte)114, (byte)197, (byte)103, (byte)229, (byte)52, (byte)20, (byte)250, (byte)124, (byte)167, (byte)230, (byte)108, (byte)91, (byte)250, (byte)181, (byte)116, (byte)255, (byte)99, (byte)218, (byte)242, (byte)27, (byte)29, (byte)73, (byte)14, (byte)99, (byte)179, (byte)238, (byte)17, (byte)5, (byte)36, (byte)239, (byte)35, (byte)158, (byte)202, (byte)17, (byte)239, (byte)37, (byte)131, (byte)235, (byte)33, (byte)43, (byte)148, (byte)45, (byte)162, (byte)60, (byte)33, (byte)249, (byte)165, (byte)218, (byte)111, (byte)7, (byte)236, (byte)75, (byte)17, (byte)0, (byte)133, (byte)73, (byte)186, (byte)82, (byte)113, (byte)198, (byte)25, (byte)228, (byte)82, (byte)228, (byte)245, (byte)40}, 0) ;
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND;
            p126.baudrate = (uint)1924329502U;
            p126.count = (byte)(byte)137;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_b_mm == (int) -1406889017);
                Debug.Assert(pack.accuracy == (uint)4212954640U);
                Debug.Assert(pack.iar_num_hypotheses == (int) -874595505);
                Debug.Assert(pack.rtk_rate == (byte)(byte)231);
                Debug.Assert(pack.rtk_health == (byte)(byte)175);
                Debug.Assert(pack.time_last_baseline_ms == (uint)3147465283U);
                Debug.Assert(pack.wn == (ushort)(ushort)13184);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)42);
                Debug.Assert(pack.tow == (uint)2736549087U);
                Debug.Assert(pack.baseline_c_mm == (int) -1696019710);
                Debug.Assert(pack.nsats == (byte)(byte)255);
                Debug.Assert(pack.baseline_a_mm == (int) -1453650219);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)225);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.iar_num_hypotheses = (int) -874595505;
            p127.baseline_coords_type = (byte)(byte)42;
            p127.wn = (ushort)(ushort)13184;
            p127.nsats = (byte)(byte)255;
            p127.accuracy = (uint)4212954640U;
            p127.rtk_health = (byte)(byte)175;
            p127.tow = (uint)2736549087U;
            p127.rtk_receiver_id = (byte)(byte)225;
            p127.baseline_c_mm = (int) -1696019710;
            p127.rtk_rate = (byte)(byte)231;
            p127.baseline_a_mm = (int) -1453650219;
            p127.time_last_baseline_ms = (uint)3147465283U;
            p127.baseline_b_mm = (int) -1406889017;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_a_mm == (int)502478137);
                Debug.Assert(pack.baseline_b_mm == (int) -2116032000);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)24);
                Debug.Assert(pack.time_last_baseline_ms == (uint)1044436894U);
                Debug.Assert(pack.tow == (uint)1751574906U);
                Debug.Assert(pack.accuracy == (uint)2047453564U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)250);
                Debug.Assert(pack.nsats == (byte)(byte)9);
                Debug.Assert(pack.iar_num_hypotheses == (int) -645408196);
                Debug.Assert(pack.rtk_health == (byte)(byte)144);
                Debug.Assert(pack.wn == (ushort)(ushort)7743);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)222);
                Debug.Assert(pack.baseline_c_mm == (int)522718168);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.baseline_coords_type = (byte)(byte)24;
            p128.time_last_baseline_ms = (uint)1044436894U;
            p128.rtk_rate = (byte)(byte)250;
            p128.baseline_b_mm = (int) -2116032000;
            p128.baseline_c_mm = (int)522718168;
            p128.rtk_health = (byte)(byte)144;
            p128.rtk_receiver_id = (byte)(byte)222;
            p128.tow = (uint)1751574906U;
            p128.nsats = (byte)(byte)9;
            p128.iar_num_hypotheses = (int) -645408196;
            p128.baseline_a_mm = (int)502478137;
            p128.accuracy = (uint)2047453564U;
            p128.wn = (ushort)(ushort)7743;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short) -32374);
                Debug.Assert(pack.zmag == (short)(short)14611);
                Debug.Assert(pack.ymag == (short)(short)8713);
                Debug.Assert(pack.time_boot_ms == (uint)2669934890U);
                Debug.Assert(pack.yacc == (short)(short)8295);
                Debug.Assert(pack.xgyro == (short)(short) -4705);
                Debug.Assert(pack.xmag == (short)(short)12564);
                Debug.Assert(pack.ygyro == (short)(short)10730);
                Debug.Assert(pack.zgyro == (short)(short) -9250);
                Debug.Assert(pack.xacc == (short)(short) -20899);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.yacc = (short)(short)8295;
            p129.xacc = (short)(short) -20899;
            p129.zgyro = (short)(short) -9250;
            p129.xmag = (short)(short)12564;
            p129.zacc = (short)(short) -32374;
            p129.ymag = (short)(short)8713;
            p129.ygyro = (short)(short)10730;
            p129.zmag = (short)(short)14611;
            p129.xgyro = (short)(short) -4705;
            p129.time_boot_ms = (uint)2669934890U;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.jpg_quality == (byte)(byte)186);
                Debug.Assert(pack.payload == (byte)(byte)230);
                Debug.Assert(pack.packets == (ushort)(ushort)35307);
                Debug.Assert(pack.size == (uint)920954412U);
                Debug.Assert(pack.width == (ushort)(ushort)15314);
                Debug.Assert(pack.height == (ushort)(ushort)47508);
                Debug.Assert(pack.type == (byte)(byte)120);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.size = (uint)920954412U;
            p130.type = (byte)(byte)120;
            p130.height = (ushort)(ushort)47508;
            p130.payload = (byte)(byte)230;
            p130.packets = (ushort)(ushort)35307;
            p130.width = (ushort)(ushort)15314;
            p130.jpg_quality = (byte)(byte)186;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)179, (byte)211, (byte)62, (byte)51, (byte)193, (byte)173, (byte)118, (byte)86, (byte)130, (byte)204, (byte)240, (byte)223, (byte)122, (byte)247, (byte)76, (byte)31, (byte)62, (byte)185, (byte)47, (byte)228, (byte)136, (byte)132, (byte)214, (byte)73, (byte)42, (byte)44, (byte)71, (byte)155, (byte)108, (byte)165, (byte)16, (byte)10, (byte)97, (byte)179, (byte)158, (byte)15, (byte)16, (byte)100, (byte)52, (byte)111, (byte)33, (byte)24, (byte)174, (byte)15, (byte)234, (byte)149, (byte)194, (byte)78, (byte)144, (byte)177, (byte)217, (byte)173, (byte)168, (byte)165, (byte)196, (byte)197, (byte)36, (byte)234, (byte)40, (byte)225, (byte)104, (byte)142, (byte)16, (byte)224, (byte)167, (byte)218, (byte)198, (byte)85, (byte)170, (byte)217, (byte)39, (byte)100, (byte)217, (byte)106, (byte)194, (byte)134, (byte)222, (byte)103, (byte)253, (byte)66, (byte)66, (byte)79, (byte)55, (byte)124, (byte)211, (byte)169, (byte)137, (byte)108, (byte)84, (byte)206, (byte)233, (byte)96, (byte)7, (byte)92, (byte)77, (byte)83, (byte)20, (byte)190, (byte)105, (byte)87, (byte)107, (byte)30, (byte)84, (byte)177, (byte)142, (byte)208, (byte)103, (byte)69, (byte)191, (byte)85, (byte)95, (byte)195, (byte)21, (byte)245, (byte)183, (byte)125, (byte)177, (byte)175, (byte)90, (byte)155, (byte)51, (byte)96, (byte)2, (byte)37, (byte)133, (byte)111, (byte)187, (byte)104, (byte)22, (byte)153, (byte)55, (byte)38, (byte)64, (byte)64, (byte)133, (byte)27, (byte)39, (byte)189, (byte)34, (byte)95, (byte)80, (byte)19, (byte)39, (byte)213, (byte)23, (byte)126, (byte)223, (byte)61, (byte)220, (byte)152, (byte)77, (byte)53, (byte)207, (byte)100, (byte)29, (byte)210, (byte)222, (byte)235, (byte)82, (byte)217, (byte)240, (byte)105, (byte)165, (byte)37, (byte)92, (byte)101, (byte)137, (byte)242, (byte)138, (byte)70, (byte)43, (byte)111, (byte)209, (byte)160, (byte)198, (byte)161, (byte)39, (byte)234, (byte)2, (byte)62, (byte)222, (byte)77, (byte)3, (byte)196, (byte)215, (byte)119, (byte)36, (byte)15, (byte)181, (byte)4, (byte)121, (byte)25, (byte)117, (byte)197, (byte)244, (byte)113, (byte)135, (byte)233, (byte)119, (byte)23, (byte)247, (byte)188, (byte)107, (byte)171, (byte)244, (byte)76, (byte)89, (byte)147, (byte)208, (byte)31, (byte)24, (byte)18, (byte)175, (byte)192, (byte)232, (byte)238, (byte)96, (byte)99, (byte)46, (byte)194, (byte)75, (byte)194, (byte)74, (byte)10, (byte)66, (byte)185, (byte)154, (byte)113, (byte)86, (byte)226, (byte)156, (byte)209, (byte)139, (byte)180, (byte)188, (byte)215, (byte)131, (byte)128, (byte)86, (byte)222, (byte)184, (byte)198, (byte)14, (byte)37, (byte)81, (byte)52, (byte)220, (byte)96, (byte)150, (byte)254, (byte)71, (byte)234, (byte)236}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)50221);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)179, (byte)211, (byte)62, (byte)51, (byte)193, (byte)173, (byte)118, (byte)86, (byte)130, (byte)204, (byte)240, (byte)223, (byte)122, (byte)247, (byte)76, (byte)31, (byte)62, (byte)185, (byte)47, (byte)228, (byte)136, (byte)132, (byte)214, (byte)73, (byte)42, (byte)44, (byte)71, (byte)155, (byte)108, (byte)165, (byte)16, (byte)10, (byte)97, (byte)179, (byte)158, (byte)15, (byte)16, (byte)100, (byte)52, (byte)111, (byte)33, (byte)24, (byte)174, (byte)15, (byte)234, (byte)149, (byte)194, (byte)78, (byte)144, (byte)177, (byte)217, (byte)173, (byte)168, (byte)165, (byte)196, (byte)197, (byte)36, (byte)234, (byte)40, (byte)225, (byte)104, (byte)142, (byte)16, (byte)224, (byte)167, (byte)218, (byte)198, (byte)85, (byte)170, (byte)217, (byte)39, (byte)100, (byte)217, (byte)106, (byte)194, (byte)134, (byte)222, (byte)103, (byte)253, (byte)66, (byte)66, (byte)79, (byte)55, (byte)124, (byte)211, (byte)169, (byte)137, (byte)108, (byte)84, (byte)206, (byte)233, (byte)96, (byte)7, (byte)92, (byte)77, (byte)83, (byte)20, (byte)190, (byte)105, (byte)87, (byte)107, (byte)30, (byte)84, (byte)177, (byte)142, (byte)208, (byte)103, (byte)69, (byte)191, (byte)85, (byte)95, (byte)195, (byte)21, (byte)245, (byte)183, (byte)125, (byte)177, (byte)175, (byte)90, (byte)155, (byte)51, (byte)96, (byte)2, (byte)37, (byte)133, (byte)111, (byte)187, (byte)104, (byte)22, (byte)153, (byte)55, (byte)38, (byte)64, (byte)64, (byte)133, (byte)27, (byte)39, (byte)189, (byte)34, (byte)95, (byte)80, (byte)19, (byte)39, (byte)213, (byte)23, (byte)126, (byte)223, (byte)61, (byte)220, (byte)152, (byte)77, (byte)53, (byte)207, (byte)100, (byte)29, (byte)210, (byte)222, (byte)235, (byte)82, (byte)217, (byte)240, (byte)105, (byte)165, (byte)37, (byte)92, (byte)101, (byte)137, (byte)242, (byte)138, (byte)70, (byte)43, (byte)111, (byte)209, (byte)160, (byte)198, (byte)161, (byte)39, (byte)234, (byte)2, (byte)62, (byte)222, (byte)77, (byte)3, (byte)196, (byte)215, (byte)119, (byte)36, (byte)15, (byte)181, (byte)4, (byte)121, (byte)25, (byte)117, (byte)197, (byte)244, (byte)113, (byte)135, (byte)233, (byte)119, (byte)23, (byte)247, (byte)188, (byte)107, (byte)171, (byte)244, (byte)76, (byte)89, (byte)147, (byte)208, (byte)31, (byte)24, (byte)18, (byte)175, (byte)192, (byte)232, (byte)238, (byte)96, (byte)99, (byte)46, (byte)194, (byte)75, (byte)194, (byte)74, (byte)10, (byte)66, (byte)185, (byte)154, (byte)113, (byte)86, (byte)226, (byte)156, (byte)209, (byte)139, (byte)180, (byte)188, (byte)215, (byte)131, (byte)128, (byte)86, (byte)222, (byte)184, (byte)198, (byte)14, (byte)37, (byte)81, (byte)52, (byte)220, (byte)96, (byte)150, (byte)254, (byte)71, (byte)234, (byte)236}, 0) ;
            p131.seqnr = (ushort)(ushort)50221;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)992723044U);
                Debug.Assert(pack.min_distance == (ushort)(ushort)54317);
                Debug.Assert(pack.max_distance == (ushort)(ushort)590);
                Debug.Assert(pack.covariance == (byte)(byte)78);
                Debug.Assert(pack.current_distance == (ushort)(ushort)41369);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180);
                Debug.Assert(pack.id == (byte)(byte)186);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.id = (byte)(byte)186;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p132.max_distance = (ushort)(ushort)590;
            p132.min_distance = (ushort)(ushort)54317;
            p132.covariance = (byte)(byte)78;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180;
            p132.time_boot_ms = (uint)992723044U;
            p132.current_distance = (ushort)(ushort)41369;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)6981);
                Debug.Assert(pack.lat == (int)217657272);
                Debug.Assert(pack.mask == (ulong)5200853318732264866L);
                Debug.Assert(pack.lon == (int)1324638909);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int)217657272;
            p133.mask = (ulong)5200853318732264866L;
            p133.lon = (int)1324638909;
            p133.grid_spacing = (ushort)(ushort)6981;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gridbit == (byte)(byte)228);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)61825);
                Debug.Assert(pack.lat == (int) -944963935);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -6890, (short)7555, (short) -12018, (short) -27612, (short) -32635, (short) -28605, (short) -28183, (short) -15567, (short)13770, (short) -20633, (short) -16429, (short) -19615, (short) -8305, (short)25532, (short)4303, (short) -28926}));
                Debug.Assert(pack.lon == (int)1016229152);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.data__SET(new short[] {(short) -6890, (short)7555, (short) -12018, (short) -27612, (short) -32635, (short) -28605, (short) -28183, (short) -15567, (short)13770, (short) -20633, (short) -16429, (short) -19615, (short) -8305, (short)25532, (short)4303, (short) -28926}, 0) ;
            p134.grid_spacing = (ushort)(ushort)61825;
            p134.gridbit = (byte)(byte)228;
            p134.lat = (int) -944963935;
            p134.lon = (int)1016229152;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)613224589);
                Debug.Assert(pack.lon == (int) -1394740005);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)613224589;
            p135.lon = (int) -1394740005;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.loaded == (ushort)(ushort)20791);
                Debug.Assert(pack.lat == (int)1609743725);
                Debug.Assert(pack.terrain_height == (float)9.967716E37F);
                Debug.Assert(pack.current_height == (float) -2.6308894E38F);
                Debug.Assert(pack.pending == (ushort)(ushort)38043);
                Debug.Assert(pack.spacing == (ushort)(ushort)38817);
                Debug.Assert(pack.lon == (int)19570742);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.current_height = (float) -2.6308894E38F;
            p136.lon = (int)19570742;
            p136.terrain_height = (float)9.967716E37F;
            p136.spacing = (ushort)(ushort)38817;
            p136.pending = (ushort)(ushort)38043;
            p136.lat = (int)1609743725;
            p136.loaded = (ushort)(ushort)20791;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -2.288906E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3524599153U);
                Debug.Assert(pack.press_diff == (float)1.5024571E37F);
                Debug.Assert(pack.temperature == (short)(short)17865);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_abs = (float) -2.288906E38F;
            p137.press_diff = (float)1.5024571E37F;
            p137.temperature = (short)(short)17865;
            p137.time_boot_ms = (uint)3524599153U;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.2764108E37F, 2.9178737E38F, 5.0176993E37F, -9.402543E37F}));
                Debug.Assert(pack.y == (float) -1.2218509E38F);
                Debug.Assert(pack.z == (float)1.8591649E37F);
                Debug.Assert(pack.x == (float) -3.0214827E38F);
                Debug.Assert(pack.time_usec == (ulong)4551302129553499366L);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.q_SET(new float[] {1.2764108E37F, 2.9178737E38F, 5.0176993E37F, -9.402543E37F}, 0) ;
            p138.y = (float) -1.2218509E38F;
            p138.x = (float) -3.0214827E38F;
            p138.time_usec = (ulong)4551302129553499366L;
            p138.z = (float)1.8591649E37F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)14);
                Debug.Assert(pack.time_usec == (ulong)293840491560164347L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-2.5146687E38F, 8.0017944E37F, -2.2922794E38F, 1.1984574E38F, 1.0891177E38F, 1.8927078E37F, -5.0789745E37F, 2.5447729E38F}));
                Debug.Assert(pack.target_system == (byte)(byte)186);
                Debug.Assert(pack.group_mlx == (byte)(byte)125);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_component = (byte)(byte)14;
            p139.target_system = (byte)(byte)186;
            p139.controls_SET(new float[] {-2.5146687E38F, 8.0017944E37F, -2.2922794E38F, 1.1984574E38F, 1.0891177E38F, 1.8927078E37F, -5.0789745E37F, 2.5447729E38F}, 0) ;
            p139.time_usec = (ulong)293840491560164347L;
            p139.group_mlx = (byte)(byte)125;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)248);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.9985096E38F, 1.1075343E38F, -2.6972534E38F, -2.1746453E38F, -1.4566939E38F, 2.9512291E38F, -1.3562921E38F, 4.0829568E37F}));
                Debug.Assert(pack.time_usec == (ulong)5837550930240871261L);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)5837550930240871261L;
            p140.group_mlx = (byte)(byte)248;
            p140.controls_SET(new float[] {1.9985096E38F, 1.1075343E38F, -2.6972534E38F, -2.1746453E38F, -1.4566939E38F, 2.9512291E38F, -1.3562921E38F, 4.0829568E37F}, 0) ;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_monotonic == (float)3.1578262E38F);
                Debug.Assert(pack.altitude_terrain == (float) -3.2522578E37F);
                Debug.Assert(pack.time_usec == (ulong)4729532039498901738L);
                Debug.Assert(pack.altitude_amsl == (float) -2.4999743E38F);
                Debug.Assert(pack.altitude_local == (float)9.256323E37F);
                Debug.Assert(pack.altitude_relative == (float)1.923141E38F);
                Debug.Assert(pack.bottom_clearance == (float)2.7252312E38F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_amsl = (float) -2.4999743E38F;
            p141.altitude_relative = (float)1.923141E38F;
            p141.altitude_local = (float)9.256323E37F;
            p141.altitude_monotonic = (float)3.1578262E38F;
            p141.bottom_clearance = (float)2.7252312E38F;
            p141.time_usec = (ulong)4729532039498901738L;
            p141.altitude_terrain = (float) -3.2522578E37F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.transfer_type == (byte)(byte)204);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)217, (byte)193, (byte)37, (byte)138, (byte)6, (byte)246, (byte)176, (byte)45, (byte)52, (byte)151, (byte)215, (byte)236, (byte)78, (byte)67, (byte)180, (byte)181, (byte)72, (byte)173, (byte)94, (byte)112, (byte)211, (byte)190, (byte)16, (byte)112, (byte)97, (byte)208, (byte)210, (byte)159, (byte)10, (byte)167, (byte)223, (byte)249, (byte)138, (byte)252, (byte)110, (byte)192, (byte)17, (byte)214, (byte)236, (byte)180, (byte)150, (byte)155, (byte)162, (byte)71, (byte)108, (byte)72, (byte)1, (byte)248, (byte)228, (byte)129, (byte)41, (byte)102, (byte)213, (byte)21, (byte)65, (byte)37, (byte)188, (byte)196, (byte)57, (byte)222, (byte)63, (byte)170, (byte)180, (byte)248, (byte)151, (byte)39, (byte)245, (byte)140, (byte)39, (byte)16, (byte)92, (byte)114, (byte)254, (byte)234, (byte)17, (byte)219, (byte)198, (byte)214, (byte)78, (byte)225, (byte)143, (byte)164, (byte)211, (byte)132, (byte)182, (byte)179, (byte)101, (byte)49, (byte)155, (byte)166, (byte)211, (byte)174, (byte)95, (byte)121, (byte)99, (byte)101, (byte)148, (byte)191, (byte)45, (byte)238, (byte)164, (byte)69, (byte)198, (byte)90, (byte)23, (byte)168, (byte)62, (byte)92, (byte)234, (byte)224, (byte)88, (byte)34, (byte)158, (byte)12, (byte)156, (byte)179, (byte)112, (byte)43, (byte)150, (byte)23}));
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)67, (byte)13, (byte)209, (byte)103, (byte)99, (byte)232, (byte)163, (byte)185, (byte)44, (byte)38, (byte)249, (byte)164, (byte)113, (byte)130, (byte)116, (byte)2, (byte)190, (byte)189, (byte)237, (byte)83, (byte)186, (byte)115, (byte)18, (byte)160, (byte)236, (byte)68, (byte)212, (byte)94, (byte)164, (byte)253, (byte)106, (byte)60, (byte)168, (byte)105, (byte)214, (byte)57, (byte)48, (byte)176, (byte)84, (byte)222, (byte)52, (byte)195, (byte)26, (byte)106, (byte)56, (byte)235, (byte)54, (byte)10, (byte)252, (byte)206, (byte)7, (byte)189, (byte)161, (byte)82, (byte)242, (byte)108, (byte)111, (byte)227, (byte)186, (byte)13, (byte)142, (byte)180, (byte)14, (byte)194, (byte)114, (byte)39, (byte)87, (byte)24, (byte)200, (byte)62, (byte)29, (byte)24, (byte)60, (byte)123, (byte)84, (byte)80, (byte)6, (byte)139, (byte)100, (byte)75, (byte)144, (byte)31, (byte)7, (byte)196, (byte)131, (byte)145, (byte)156, (byte)39, (byte)137, (byte)201, (byte)37, (byte)2, (byte)118, (byte)226, (byte)64, (byte)55, (byte)8, (byte)157, (byte)18, (byte)206, (byte)54, (byte)172, (byte)108, (byte)228, (byte)178, (byte)69, (byte)213, (byte)224, (byte)18, (byte)28, (byte)217, (byte)223, (byte)64, (byte)203, (byte)81, (byte)80, (byte)87, (byte)78, (byte)15, (byte)2}));
                Debug.Assert(pack.uri_type == (byte)(byte)227);
                Debug.Assert(pack.request_id == (byte)(byte)44);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.storage_SET(new byte[] {(byte)67, (byte)13, (byte)209, (byte)103, (byte)99, (byte)232, (byte)163, (byte)185, (byte)44, (byte)38, (byte)249, (byte)164, (byte)113, (byte)130, (byte)116, (byte)2, (byte)190, (byte)189, (byte)237, (byte)83, (byte)186, (byte)115, (byte)18, (byte)160, (byte)236, (byte)68, (byte)212, (byte)94, (byte)164, (byte)253, (byte)106, (byte)60, (byte)168, (byte)105, (byte)214, (byte)57, (byte)48, (byte)176, (byte)84, (byte)222, (byte)52, (byte)195, (byte)26, (byte)106, (byte)56, (byte)235, (byte)54, (byte)10, (byte)252, (byte)206, (byte)7, (byte)189, (byte)161, (byte)82, (byte)242, (byte)108, (byte)111, (byte)227, (byte)186, (byte)13, (byte)142, (byte)180, (byte)14, (byte)194, (byte)114, (byte)39, (byte)87, (byte)24, (byte)200, (byte)62, (byte)29, (byte)24, (byte)60, (byte)123, (byte)84, (byte)80, (byte)6, (byte)139, (byte)100, (byte)75, (byte)144, (byte)31, (byte)7, (byte)196, (byte)131, (byte)145, (byte)156, (byte)39, (byte)137, (byte)201, (byte)37, (byte)2, (byte)118, (byte)226, (byte)64, (byte)55, (byte)8, (byte)157, (byte)18, (byte)206, (byte)54, (byte)172, (byte)108, (byte)228, (byte)178, (byte)69, (byte)213, (byte)224, (byte)18, (byte)28, (byte)217, (byte)223, (byte)64, (byte)203, (byte)81, (byte)80, (byte)87, (byte)78, (byte)15, (byte)2}, 0) ;
            p142.transfer_type = (byte)(byte)204;
            p142.request_id = (byte)(byte)44;
            p142.uri_type = (byte)(byte)227;
            p142.uri_SET(new byte[] {(byte)217, (byte)193, (byte)37, (byte)138, (byte)6, (byte)246, (byte)176, (byte)45, (byte)52, (byte)151, (byte)215, (byte)236, (byte)78, (byte)67, (byte)180, (byte)181, (byte)72, (byte)173, (byte)94, (byte)112, (byte)211, (byte)190, (byte)16, (byte)112, (byte)97, (byte)208, (byte)210, (byte)159, (byte)10, (byte)167, (byte)223, (byte)249, (byte)138, (byte)252, (byte)110, (byte)192, (byte)17, (byte)214, (byte)236, (byte)180, (byte)150, (byte)155, (byte)162, (byte)71, (byte)108, (byte)72, (byte)1, (byte)248, (byte)228, (byte)129, (byte)41, (byte)102, (byte)213, (byte)21, (byte)65, (byte)37, (byte)188, (byte)196, (byte)57, (byte)222, (byte)63, (byte)170, (byte)180, (byte)248, (byte)151, (byte)39, (byte)245, (byte)140, (byte)39, (byte)16, (byte)92, (byte)114, (byte)254, (byte)234, (byte)17, (byte)219, (byte)198, (byte)214, (byte)78, (byte)225, (byte)143, (byte)164, (byte)211, (byte)132, (byte)182, (byte)179, (byte)101, (byte)49, (byte)155, (byte)166, (byte)211, (byte)174, (byte)95, (byte)121, (byte)99, (byte)101, (byte)148, (byte)191, (byte)45, (byte)238, (byte)164, (byte)69, (byte)198, (byte)90, (byte)23, (byte)168, (byte)62, (byte)92, (byte)234, (byte)224, (byte)88, (byte)34, (byte)158, (byte)12, (byte)156, (byte)179, (byte)112, (byte)43, (byte)150, (byte)23}, 0) ;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3143914231U);
                Debug.Assert(pack.press_abs == (float)1.2812456E38F);
                Debug.Assert(pack.press_diff == (float) -8.032971E37F);
                Debug.Assert(pack.temperature == (short)(short)20652);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_abs = (float)1.2812456E38F;
            p143.time_boot_ms = (uint)3143914231U;
            p143.temperature = (short)(short)20652;
            p143.press_diff = (float) -8.032971E37F;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.timestamp == (ulong)8601275426946927530L);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {1.8390852E38F, 9.684475E37F, -2.4598942E38F}));
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {2.0717013E38F, -2.2293642E38F, 2.752425E38F}));
                Debug.Assert(pack.lon == (int) -1424100808);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {1.0259383E38F, 1.0103276E38F, 1.3767291E38F}));
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-2.8638945E38F, -2.6656577E38F, 2.5898477E38F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)27);
                Debug.Assert(pack.alt == (float)1.5510823E38F);
                Debug.Assert(pack.lat == (int) -993143912);
                Debug.Assert(pack.custom_state == (ulong)6767874753375062892L);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-3.6683413E37F, -2.1825E38F, 2.8366007E38F, 1.2754101E38F}));
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.rates_SET(new float[] {1.8390852E38F, 9.684475E37F, -2.4598942E38F}, 0) ;
            p144.acc_SET(new float[] {-2.8638945E38F, -2.6656577E38F, 2.5898477E38F}, 0) ;
            p144.timestamp = (ulong)8601275426946927530L;
            p144.attitude_q_SET(new float[] {-3.6683413E37F, -2.1825E38F, 2.8366007E38F, 1.2754101E38F}, 0) ;
            p144.lon = (int) -1424100808;
            p144.position_cov_SET(new float[] {2.0717013E38F, -2.2293642E38F, 2.752425E38F}, 0) ;
            p144.alt = (float)1.5510823E38F;
            p144.est_capabilities = (byte)(byte)27;
            p144.lat = (int) -993143912;
            p144.vel_SET(new float[] {1.0259383E38F, 1.0103276E38F, 1.3767291E38F}, 0) ;
            p144.custom_state = (ulong)6767874753375062892L;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll_rate == (float) -7.109426E37F);
                Debug.Assert(pack.yaw_rate == (float)9.916738E37F);
                Debug.Assert(pack.y_acc == (float) -8.873801E37F);
                Debug.Assert(pack.time_usec == (ulong)6547517110070203456L);
                Debug.Assert(pack.pitch_rate == (float)2.8733027E38F);
                Debug.Assert(pack.z_acc == (float) -3.2237722E38F);
                Debug.Assert(pack.z_vel == (float)2.6274391E38F);
                Debug.Assert(pack.y_pos == (float) -3.3000515E38F);
                Debug.Assert(pack.z_pos == (float) -7.3510597E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.2547298E38F, 2.9912023E38F, 2.0381829E37F, -2.083741E38F}));
                Debug.Assert(pack.airspeed == (float)2.14724E38F);
                Debug.Assert(pack.x_acc == (float) -3.0801392E37F);
                Debug.Assert(pack.x_vel == (float) -2.8231666E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-1.0271616E38F, -6.2724173E37F, 2.6265647E37F}));
                Debug.Assert(pack.x_pos == (float) -2.8731997E38F);
                Debug.Assert(pack.y_vel == (float)1.7226493E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {4.955822E37F, -3.2349774E37F, -6.3799694E37F}));
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.y_vel = (float)1.7226493E38F;
            p146.x_vel = (float) -2.8231666E38F;
            p146.x_acc = (float) -3.0801392E37F;
            p146.z_vel = (float)2.6274391E38F;
            p146.z_acc = (float) -3.2237722E38F;
            p146.y_pos = (float) -3.3000515E38F;
            p146.q_SET(new float[] {2.2547298E38F, 2.9912023E38F, 2.0381829E37F, -2.083741E38F}, 0) ;
            p146.pitch_rate = (float)2.8733027E38F;
            p146.vel_variance_SET(new float[] {4.955822E37F, -3.2349774E37F, -6.3799694E37F}, 0) ;
            p146.yaw_rate = (float)9.916738E37F;
            p146.airspeed = (float)2.14724E38F;
            p146.time_usec = (ulong)6547517110070203456L;
            p146.x_pos = (float) -2.8731997E38F;
            p146.z_pos = (float) -7.3510597E37F;
            p146.roll_rate = (float) -7.109426E37F;
            p146.pos_variance_SET(new float[] {-1.0271616E38F, -6.2724173E37F, 2.6265647E37F}, 0) ;
            p146.y_acc = (float) -8.873801E37F;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_battery == (short)(short) -13794);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)2);
                Debug.Assert(pack.temperature == (short)(short) -4223);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
                Debug.Assert(pack.id == (byte)(byte)154);
                Debug.Assert(pack.current_consumed == (int) -1168390066);
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)55303, (ushort)28026, (ushort)1460, (ushort)28650, (ushort)52501, (ushort)6611, (ushort)28651, (ushort)54636, (ushort)34181, (ushort)58122}));
                Debug.Assert(pack.energy_consumed == (int)1213785191);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.voltages_SET(new ushort[] {(ushort)55303, (ushort)28026, (ushort)1460, (ushort)28650, (ushort)52501, (ushort)6611, (ushort)28651, (ushort)54636, (ushort)34181, (ushort)58122}, 0) ;
            p147.energy_consumed = (int)1213785191;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN;
            p147.id = (byte)(byte)154;
            p147.current_battery = (short)(short) -13794;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            p147.current_consumed = (int) -1168390066;
            p147.temperature = (short)(short) -4223;
            p147.battery_remaining = (sbyte)(sbyte)2;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uid == (ulong)6786150071282925417L);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)49932);
                Debug.Assert(pack.middleware_sw_version == (uint)2781702434U);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)209, (byte)111, (byte)52, (byte)176, (byte)220, (byte)231, (byte)81, (byte)62}));
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)17, (byte)108, (byte)183, (byte)0, (byte)53, (byte)248, (byte)66, (byte)195, (byte)236, (byte)56, (byte)174, (byte)101, (byte)71, (byte)21, (byte)2, (byte)80, (byte)41, (byte)249}));
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)204, (byte)208, (byte)181, (byte)93, (byte)48, (byte)112, (byte)37, (byte)244}));
                Debug.Assert(pack.os_sw_version == (uint)4237648408U);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET);
                Debug.Assert(pack.board_version == (uint)577665371U);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)70, (byte)244, (byte)146, (byte)50, (byte)4, (byte)52, (byte)84, (byte)192}));
                Debug.Assert(pack.flight_sw_version == (uint)3182543952U);
                Debug.Assert(pack.product_id == (ushort)(ushort)20966);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.uid = (ulong)6786150071282925417L;
            p148.middleware_custom_version_SET(new byte[] {(byte)204, (byte)208, (byte)181, (byte)93, (byte)48, (byte)112, (byte)37, (byte)244}, 0) ;
            p148.vendor_id = (ushort)(ushort)49932;
            p148.os_sw_version = (uint)4237648408U;
            p148.board_version = (uint)577665371U;
            p148.product_id = (ushort)(ushort)20966;
            p148.os_custom_version_SET(new byte[] {(byte)209, (byte)111, (byte)52, (byte)176, (byte)220, (byte)231, (byte)81, (byte)62}, 0) ;
            p148.flight_custom_version_SET(new byte[] {(byte)70, (byte)244, (byte)146, (byte)50, (byte)4, (byte)52, (byte)84, (byte)192}, 0) ;
            p148.flight_sw_version = (uint)3182543952U;
            p148.middleware_sw_version = (uint)2781702434U;
            p148.uid2_SET(new byte[] {(byte)17, (byte)108, (byte)183, (byte)0, (byte)53, (byte)248, (byte)66, (byte)195, (byte)236, (byte)56, (byte)174, (byte)101, (byte)71, (byte)21, (byte)2, (byte)80, (byte)41, (byte)249}, 0, PH) ;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.distance == (float)1.2048267E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)223);
                Debug.Assert(pack.time_usec == (ulong)697515348864221269L);
                Debug.Assert(pack.angle_x == (float)3.306954E38F);
                Debug.Assert(pack.z_TRY(ph) == (float)1.1123727E38F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {2.8092377E37F, 2.515976E38F, -2.4552743E38F, 2.672668E37F}));
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
                Debug.Assert(pack.size_y == (float)2.868132E38F);
                Debug.Assert(pack.size_x == (float)1.3925104E38F);
                Debug.Assert(pack.target_num == (byte)(byte)128);
                Debug.Assert(pack.y_TRY(ph) == (float)4.3387624E37F);
                Debug.Assert(pack.x_TRY(ph) == (float)3.400323E38F);
                Debug.Assert(pack.angle_y == (float)2.59737E38F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.y_SET((float)4.3387624E37F, PH) ;
            p149.distance = (float)1.2048267E38F;
            p149.angle_y = (float)2.59737E38F;
            p149.angle_x = (float)3.306954E38F;
            p149.position_valid_SET((byte)(byte)223, PH) ;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p149.time_usec = (ulong)697515348864221269L;
            p149.target_num = (byte)(byte)128;
            p149.size_y = (float)2.868132E38F;
            p149.z_SET((float)1.1123727E38F, PH) ;
            p149.x_SET((float)3.400323E38F, PH) ;
            p149.q_SET(new float[] {2.8092377E37F, 2.515976E38F, -2.4552743E38F, 2.672668E37F}, 0, PH) ;
            p149.size_x = (float)1.3925104E38F;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAQ_TELEMETRY_FReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value19 == (float)1.7717963E38F);
                Debug.Assert(pack.value10 == (float) -1.6647911E38F);
                Debug.Assert(pack.value3 == (float) -2.6080903E38F);
                Debug.Assert(pack.value15 == (float) -2.2481867E38F);
                Debug.Assert(pack.value4 == (float) -2.7012507E38F);
                Debug.Assert(pack.value6 == (float) -1.4382736E38F);
                Debug.Assert(pack.value7 == (float)8.2748616E37F);
                Debug.Assert(pack.value20 == (float) -1.0343737E37F);
                Debug.Assert(pack.value17 == (float) -3.3884954E38F);
                Debug.Assert(pack.value14 == (float)2.1431078E38F);
                Debug.Assert(pack.value11 == (float) -2.0998117E38F);
                Debug.Assert(pack.value13 == (float) -1.4629255E38F);
                Debug.Assert(pack.value16 == (float)1.180282E38F);
                Debug.Assert(pack.value12 == (float)2.3612365E38F);
                Debug.Assert(pack.value2 == (float) -1.9768847E38F);
                Debug.Assert(pack.value1 == (float) -2.2499837E37F);
                Debug.Assert(pack.value5 == (float)7.0878E37F);
                Debug.Assert(pack.value18 == (float)2.0059542E38F);
                Debug.Assert(pack.value9 == (float)4.6717E37F);
                Debug.Assert(pack.Index == (ushort)(ushort)41779);
                Debug.Assert(pack.value8 == (float)3.1898326E38F);
            };
            GroundControl.AQ_TELEMETRY_F p150 = CommunicationChannel.new_AQ_TELEMETRY_F();
            PH.setPack(p150);
            p150.value14 = (float)2.1431078E38F;
            p150.value7 = (float)8.2748616E37F;
            p150.value2 = (float) -1.9768847E38F;
            p150.value6 = (float) -1.4382736E38F;
            p150.value12 = (float)2.3612365E38F;
            p150.value15 = (float) -2.2481867E38F;
            p150.value9 = (float)4.6717E37F;
            p150.value8 = (float)3.1898326E38F;
            p150.value5 = (float)7.0878E37F;
            p150.value13 = (float) -1.4629255E38F;
            p150.value1 = (float) -2.2499837E37F;
            p150.value20 = (float) -1.0343737E37F;
            p150.value3 = (float) -2.6080903E38F;
            p150.value17 = (float) -3.3884954E38F;
            p150.Index = (ushort)(ushort)41779;
            p150.value10 = (float) -1.6647911E38F;
            p150.value11 = (float) -2.0998117E38F;
            p150.value18 = (float)2.0059542E38F;
            p150.value4 = (float) -2.7012507E38F;
            p150.value19 = (float)1.7717963E38F;
            p150.value16 = (float)1.180282E38F;
            CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAQ_ESC_TELEMETRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)672469230U);
                Debug.Assert(pack.seq == (byte)(byte)19);
                Debug.Assert(pack.data_version.SequenceEqual(new byte[] {(byte)7, (byte)187, (byte)200, (byte)90}));
                Debug.Assert(pack.escid.SequenceEqual(new byte[] {(byte)90, (byte)185, (byte)31, (byte)237}));
                Debug.Assert(pack.num_motors == (byte)(byte)11);
                Debug.Assert(pack.num_in_seq == (byte)(byte)219);
                Debug.Assert(pack.data1.SequenceEqual(new uint[] {1986183138U, 3007420378U, 3695418018U, 1260779022U}));
                Debug.Assert(pack.data0.SequenceEqual(new uint[] {144584847U, 3280394744U, 518210891U, 3444541649U}));
                Debug.Assert(pack.status_age.SequenceEqual(new ushort[] {(ushort)46153, (ushort)7615, (ushort)27934, (ushort)37541}));
            };
            GroundControl.AQ_ESC_TELEMETRY p152 = CommunicationChannel.new_AQ_ESC_TELEMETRY();
            PH.setPack(p152);
            p152.data_version_SET(new byte[] {(byte)7, (byte)187, (byte)200, (byte)90}, 0) ;
            p152.seq = (byte)(byte)19;
            p152.status_age_SET(new ushort[] {(ushort)46153, (ushort)7615, (ushort)27934, (ushort)37541}, 0) ;
            p152.escid_SET(new byte[] {(byte)90, (byte)185, (byte)31, (byte)237}, 0) ;
            p152.data0_SET(new uint[] {144584847U, 3280394744U, 518210891U, 3444541649U}, 0) ;
            p152.num_in_seq = (byte)(byte)219;
            p152.num_motors = (byte)(byte)11;
            p152.data1_SET(new uint[] {1986183138U, 3007420378U, 3695418018U, 1260779022U}, 0) ;
            p152.time_boot_ms = (uint)672469230U;
            CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tas_ratio == (float)2.5023392E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE);
                Debug.Assert(pack.pos_vert_ratio == (float) -2.448447E38F);
                Debug.Assert(pack.hagl_ratio == (float) -8.813489E37F);
                Debug.Assert(pack.vel_ratio == (float)1.8433162E37F);
                Debug.Assert(pack.time_usec == (ulong)8708323716031637080L);
                Debug.Assert(pack.pos_horiz_ratio == (float) -1.4928274E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)5.866704E37F);
                Debug.Assert(pack.pos_vert_accuracy == (float)1.4285684E38F);
                Debug.Assert(pack.mag_ratio == (float)1.2409309E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE;
            p230.mag_ratio = (float)1.2409309E38F;
            p230.pos_horiz_accuracy = (float)5.866704E37F;
            p230.pos_vert_ratio = (float) -2.448447E38F;
            p230.pos_vert_accuracy = (float)1.4285684E38F;
            p230.hagl_ratio = (float) -8.813489E37F;
            p230.time_usec = (ulong)8708323716031637080L;
            p230.pos_horiz_ratio = (float) -1.4928274E38F;
            p230.vel_ratio = (float)1.8433162E37F;
            p230.tas_ratio = (float)2.5023392E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.var_horiz == (float) -3.3295372E38F);
                Debug.Assert(pack.wind_x == (float)1.7851747E38F);
                Debug.Assert(pack.horiz_accuracy == (float)1.6794788E38F);
                Debug.Assert(pack.wind_y == (float)3.2963276E38F);
                Debug.Assert(pack.var_vert == (float)3.1241776E38F);
                Debug.Assert(pack.wind_alt == (float)2.1053973E38F);
                Debug.Assert(pack.wind_z == (float)3.3140356E38F);
                Debug.Assert(pack.time_usec == (ulong)532559737480265485L);
                Debug.Assert(pack.vert_accuracy == (float)2.2093613E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.var_horiz = (float) -3.3295372E38F;
            p231.wind_x = (float)1.7851747E38F;
            p231.wind_alt = (float)2.1053973E38F;
            p231.vert_accuracy = (float)2.2093613E38F;
            p231.wind_y = (float)3.2963276E38F;
            p231.horiz_accuracy = (float)1.6794788E38F;
            p231.var_vert = (float)3.1241776E38F;
            p231.time_usec = (ulong)532559737480265485L;
            p231.wind_z = (float)3.3140356E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vn == (float)8.08469E36F);
                Debug.Assert(pack.gps_id == (byte)(byte)211);
                Debug.Assert(pack.vdop == (float)1.3476839E38F);
                Debug.Assert(pack.time_usec == (ulong)7849534640555166804L);
                Debug.Assert(pack.fix_type == (byte)(byte)141);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP);
                Debug.Assert(pack.speed_accuracy == (float) -2.6871574E38F);
                Debug.Assert(pack.vert_accuracy == (float) -2.1848922E38F);
                Debug.Assert(pack.time_week_ms == (uint)924049343U);
                Debug.Assert(pack.horiz_accuracy == (float)2.156765E38F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)72);
                Debug.Assert(pack.lat == (int) -960496878);
                Debug.Assert(pack.vd == (float) -2.9516224E38F);
                Debug.Assert(pack.hdop == (float) -1.1517534E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)28036);
                Debug.Assert(pack.ve == (float) -2.5720698E38F);
                Debug.Assert(pack.lon == (int)1521941817);
                Debug.Assert(pack.alt == (float)3.5060794E37F);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.satellites_visible = (byte)(byte)72;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP;
            p232.speed_accuracy = (float) -2.6871574E38F;
            p232.alt = (float)3.5060794E37F;
            p232.time_usec = (ulong)7849534640555166804L;
            p232.time_week = (ushort)(ushort)28036;
            p232.lon = (int)1521941817;
            p232.horiz_accuracy = (float)2.156765E38F;
            p232.vd = (float) -2.9516224E38F;
            p232.ve = (float) -2.5720698E38F;
            p232.gps_id = (byte)(byte)211;
            p232.fix_type = (byte)(byte)141;
            p232.lat = (int) -960496878;
            p232.vn = (float)8.08469E36F;
            p232.vert_accuracy = (float) -2.1848922E38F;
            p232.time_week_ms = (uint)924049343U;
            p232.vdop = (float)1.3476839E38F;
            p232.hdop = (float) -1.1517534E38F;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)156);
                Debug.Assert(pack.flags == (byte)(byte)153);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)255, (byte)190, (byte)44, (byte)82, (byte)126, (byte)167, (byte)241, (byte)13, (byte)201, (byte)144, (byte)129, (byte)223, (byte)93, (byte)56, (byte)62, (byte)137, (byte)219, (byte)195, (byte)235, (byte)25, (byte)164, (byte)130, (byte)92, (byte)169, (byte)167, (byte)42, (byte)224, (byte)96, (byte)118, (byte)98, (byte)45, (byte)218, (byte)170, (byte)15, (byte)101, (byte)118, (byte)14, (byte)129, (byte)145, (byte)228, (byte)2, (byte)13, (byte)146, (byte)135, (byte)202, (byte)99, (byte)116, (byte)143, (byte)34, (byte)61, (byte)215, (byte)214, (byte)97, (byte)5, (byte)180, (byte)241, (byte)242, (byte)245, (byte)185, (byte)249, (byte)167, (byte)12, (byte)122, (byte)221, (byte)18, (byte)128, (byte)57, (byte)226, (byte)137, (byte)206, (byte)244, (byte)50, (byte)19, (byte)61, (byte)73, (byte)88, (byte)29, (byte)89, (byte)229, (byte)112, (byte)27, (byte)179, (byte)24, (byte)13, (byte)102, (byte)142, (byte)178, (byte)40, (byte)105, (byte)133, (byte)137, (byte)244, (byte)175, (byte)45, (byte)132, (byte)44, (byte)42, (byte)167, (byte)214, (byte)144, (byte)167, (byte)6, (byte)147, (byte)67, (byte)59, (byte)59, (byte)67, (byte)197, (byte)20, (byte)120, (byte)37, (byte)13, (byte)207, (byte)171, (byte)232, (byte)15, (byte)67, (byte)143, (byte)133, (byte)214, (byte)155, (byte)194, (byte)249, (byte)180, (byte)204, (byte)85, (byte)92, (byte)62, (byte)111, (byte)36, (byte)226, (byte)27, (byte)130, (byte)143, (byte)250, (byte)13, (byte)104, (byte)248, (byte)103, (byte)249, (byte)216, (byte)85, (byte)19, (byte)66, (byte)17, (byte)162, (byte)180, (byte)154, (byte)101, (byte)61, (byte)181, (byte)228, (byte)56, (byte)134, (byte)232, (byte)197, (byte)70, (byte)113, (byte)72, (byte)118, (byte)207, (byte)39, (byte)59, (byte)59, (byte)118, (byte)237, (byte)244, (byte)151, (byte)243, (byte)50, (byte)111, (byte)40, (byte)214, (byte)67, (byte)254, (byte)150, (byte)94, (byte)21, (byte)30, (byte)214}));
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.len = (byte)(byte)156;
            p233.flags = (byte)(byte)153;
            p233.data__SET(new byte[] {(byte)255, (byte)190, (byte)44, (byte)82, (byte)126, (byte)167, (byte)241, (byte)13, (byte)201, (byte)144, (byte)129, (byte)223, (byte)93, (byte)56, (byte)62, (byte)137, (byte)219, (byte)195, (byte)235, (byte)25, (byte)164, (byte)130, (byte)92, (byte)169, (byte)167, (byte)42, (byte)224, (byte)96, (byte)118, (byte)98, (byte)45, (byte)218, (byte)170, (byte)15, (byte)101, (byte)118, (byte)14, (byte)129, (byte)145, (byte)228, (byte)2, (byte)13, (byte)146, (byte)135, (byte)202, (byte)99, (byte)116, (byte)143, (byte)34, (byte)61, (byte)215, (byte)214, (byte)97, (byte)5, (byte)180, (byte)241, (byte)242, (byte)245, (byte)185, (byte)249, (byte)167, (byte)12, (byte)122, (byte)221, (byte)18, (byte)128, (byte)57, (byte)226, (byte)137, (byte)206, (byte)244, (byte)50, (byte)19, (byte)61, (byte)73, (byte)88, (byte)29, (byte)89, (byte)229, (byte)112, (byte)27, (byte)179, (byte)24, (byte)13, (byte)102, (byte)142, (byte)178, (byte)40, (byte)105, (byte)133, (byte)137, (byte)244, (byte)175, (byte)45, (byte)132, (byte)44, (byte)42, (byte)167, (byte)214, (byte)144, (byte)167, (byte)6, (byte)147, (byte)67, (byte)59, (byte)59, (byte)67, (byte)197, (byte)20, (byte)120, (byte)37, (byte)13, (byte)207, (byte)171, (byte)232, (byte)15, (byte)67, (byte)143, (byte)133, (byte)214, (byte)155, (byte)194, (byte)249, (byte)180, (byte)204, (byte)85, (byte)92, (byte)62, (byte)111, (byte)36, (byte)226, (byte)27, (byte)130, (byte)143, (byte)250, (byte)13, (byte)104, (byte)248, (byte)103, (byte)249, (byte)216, (byte)85, (byte)19, (byte)66, (byte)17, (byte)162, (byte)180, (byte)154, (byte)101, (byte)61, (byte)181, (byte)228, (byte)56, (byte)134, (byte)232, (byte)197, (byte)70, (byte)113, (byte)72, (byte)118, (byte)207, (byte)39, (byte)59, (byte)59, (byte)118, (byte)237, (byte)244, (byte)151, (byte)243, (byte)50, (byte)111, (byte)40, (byte)214, (byte)67, (byte)254, (byte)150, (byte)94, (byte)21, (byte)30, (byte)214}, 0) ;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
                Debug.Assert(pack.heading_sp == (short)(short) -29691);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
                Debug.Assert(pack.battery_remaining == (byte)(byte)201);
                Debug.Assert(pack.latitude == (int)246358124);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte) - 120);
                Debug.Assert(pack.altitude_sp == (short)(short)28088);
                Debug.Assert(pack.roll == (short)(short) -13043);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)153);
                Debug.Assert(pack.failsafe == (byte)(byte)130);
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 9);
                Debug.Assert(pack.altitude_amsl == (short)(short) -21031);
                Debug.Assert(pack.wp_num == (byte)(byte)73);
                Debug.Assert(pack.airspeed == (byte)(byte)126);
                Debug.Assert(pack.groundspeed == (byte)(byte)61);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
                Debug.Assert(pack.gps_nsat == (byte)(byte)38);
                Debug.Assert(pack.pitch == (short)(short) -14953);
                Debug.Assert(pack.longitude == (int) -2127175671);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 11);
                Debug.Assert(pack.custom_mode == (uint)1350453478U);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)1678);
                Debug.Assert(pack.heading == (ushort)(ushort)44602);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 57);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.heading_sp = (short)(short) -29691;
            p234.pitch = (short)(short) -14953;
            p234.roll = (short)(short) -13043;
            p234.airspeed_sp = (byte)(byte)153;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p234.longitude = (int) -2127175671;
            p234.airspeed = (byte)(byte)126;
            p234.custom_mode = (uint)1350453478U;
            p234.throttle = (sbyte)(sbyte) - 9;
            p234.gps_nsat = (byte)(byte)38;
            p234.temperature = (sbyte)(sbyte) - 11;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED;
            p234.climb_rate = (sbyte)(sbyte) - 57;
            p234.altitude_sp = (short)(short)28088;
            p234.groundspeed = (byte)(byte)61;
            p234.altitude_amsl = (short)(short) -21031;
            p234.heading = (ushort)(ushort)44602;
            p234.failsafe = (byte)(byte)130;
            p234.wp_num = (byte)(byte)73;
            p234.temperature_air = (sbyte)(sbyte) - 120;
            p234.latitude = (int)246358124;
            p234.wp_distance = (ushort)(ushort)1678;
            p234.battery_remaining = (byte)(byte)201;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_2 == (uint)2600777908U);
                Debug.Assert(pack.vibration_z == (float) -2.306172E38F);
                Debug.Assert(pack.vibration_x == (float) -2.564883E38F);
                Debug.Assert(pack.vibration_y == (float)6.3544226E37F);
                Debug.Assert(pack.clipping_1 == (uint)638073895U);
                Debug.Assert(pack.clipping_0 == (uint)2188461583U);
                Debug.Assert(pack.time_usec == (ulong)279948623362211730L);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.vibration_y = (float)6.3544226E37F;
            p241.vibration_z = (float) -2.306172E38F;
            p241.vibration_x = (float) -2.564883E38F;
            p241.clipping_0 = (uint)2188461583U;
            p241.clipping_2 = (uint)2600777908U;
            p241.clipping_1 = (uint)638073895U;
            p241.time_usec = (ulong)279948623362211730L;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)1.8476172E38F);
                Debug.Assert(pack.altitude == (int) -537652089);
                Debug.Assert(pack.y == (float) -2.5457302E37F);
                Debug.Assert(pack.approach_z == (float) -2.4960868E38F);
                Debug.Assert(pack.latitude == (int) -1334851408);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.8130646E37F, 1.1593844E38F, -3.0059413E38F, -2.4843277E38F}));
                Debug.Assert(pack.approach_y == (float) -2.7845576E37F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)1150102822270593801L);
                Debug.Assert(pack.approach_x == (float)4.1987653E37F);
                Debug.Assert(pack.x == (float) -1.1157247E38F);
                Debug.Assert(pack.longitude == (int)1007861933);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.z = (float)1.8476172E38F;
            p242.altitude = (int) -537652089;
            p242.time_usec_SET((ulong)1150102822270593801L, PH) ;
            p242.approach_y = (float) -2.7845576E37F;
            p242.y = (float) -2.5457302E37F;
            p242.x = (float) -1.1157247E38F;
            p242.longitude = (int)1007861933;
            p242.approach_z = (float) -2.4960868E38F;
            p242.q_SET(new float[] {3.8130646E37F, 1.1593844E38F, -3.0059413E38F, -2.4843277E38F}, 0) ;
            p242.latitude = (int) -1334851408;
            p242.approach_x = (float)4.1987653E37F;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int) -1143715022);
                Debug.Assert(pack.altitude == (int) -572232375);
                Debug.Assert(pack.target_system == (byte)(byte)118);
                Debug.Assert(pack.approach_y == (float) -1.8041974E38F);
                Debug.Assert(pack.approach_x == (float) -2.0985438E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.7097038E38F, 2.6740416E38F, -1.5966582E38F, -5.638811E37F}));
                Debug.Assert(pack.y == (float)7.1679805E37F);
                Debug.Assert(pack.latitude == (int) -1208278447);
                Debug.Assert(pack.x == (float)1.0702085E38F);
                Debug.Assert(pack.z == (float)1.4448444E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)4292232315428705797L);
                Debug.Assert(pack.approach_z == (float) -9.88638E37F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.time_usec_SET((ulong)4292232315428705797L, PH) ;
            p243.approach_z = (float) -9.88638E37F;
            p243.q_SET(new float[] {-2.7097038E38F, 2.6740416E38F, -1.5966582E38F, -5.638811E37F}, 0) ;
            p243.approach_y = (float) -1.8041974E38F;
            p243.approach_x = (float) -2.0985438E38F;
            p243.altitude = (int) -572232375;
            p243.latitude = (int) -1208278447;
            p243.target_system = (byte)(byte)118;
            p243.z = (float)1.4448444E38F;
            p243.y = (float)7.1679805E37F;
            p243.longitude = (int) -1143715022;
            p243.x = (float)1.0702085E38F;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int) -409104527);
                Debug.Assert(pack.message_id == (ushort)(ushort)36926);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int) -409104527;
            p244.message_id = (ushort)(ushort)36926;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_FW;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -591170759);
                Debug.Assert(pack.tslc == (byte)(byte)68);
                Debug.Assert(pack.ICAO_address == (uint)904695499U);
                Debug.Assert(pack.squawk == (ushort)(ushort)20232);
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED2);
                Debug.Assert(pack.lon == (int)2033675480);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)26167);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.ver_velocity == (short)(short)4040);
                Debug.Assert(pack.altitude == (int)1792313141);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS);
                Debug.Assert(pack.heading == (ushort)(ushort)58778);
                Debug.Assert(pack.callsign_LEN(ph) == 3);
                Debug.Assert(pack.callsign_TRY(ph).Equals("Myp"));
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED2;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.tslc = (byte)(byte)68;
            p246.squawk = (ushort)(ushort)20232;
            p246.callsign_SET("Myp", PH) ;
            p246.lat = (int) -591170759;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS;
            p246.altitude = (int)1792313141;
            p246.heading = (ushort)(ushort)58778;
            p246.ICAO_address = (uint)904695499U;
            p246.lon = (int)2033675480;
            p246.ver_velocity = (short)(short)4040;
            p246.hor_velocity = (ushort)(ushort)26167;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
                Debug.Assert(pack.horizontal_minimum_delta == (float) -2.6356732E38F);
                Debug.Assert(pack.time_to_minimum_delta == (float)4.8970286E37F);
                Debug.Assert(pack.id == (uint)2839056796U);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
                Debug.Assert(pack.altitude_minimum_delta == (float)3.441367E37F);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER;
            p247.altitude_minimum_delta = (float)3.441367E37F;
            p247.horizontal_minimum_delta = (float) -2.6356732E38F;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW;
            p247.id = (uint)2839056796U;
            p247.time_to_minimum_delta = (float)4.8970286E37F;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)172);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)81, (byte)249, (byte)160, (byte)87, (byte)129, (byte)90, (byte)148, (byte)249, (byte)251, (byte)147, (byte)128, (byte)114, (byte)226, (byte)97, (byte)84, (byte)141, (byte)118, (byte)16, (byte)114, (byte)78, (byte)243, (byte)106, (byte)188, (byte)14, (byte)73, (byte)46, (byte)29, (byte)72, (byte)186, (byte)103, (byte)61, (byte)146, (byte)201, (byte)138, (byte)140, (byte)93, (byte)26, (byte)181, (byte)112, (byte)33, (byte)37, (byte)100, (byte)118, (byte)12, (byte)181, (byte)24, (byte)107, (byte)237, (byte)92, (byte)211, (byte)247, (byte)57, (byte)242, (byte)66, (byte)50, (byte)121, (byte)94, (byte)204, (byte)213, (byte)150, (byte)210, (byte)36, (byte)5, (byte)123, (byte)74, (byte)146, (byte)175, (byte)39, (byte)226, (byte)253, (byte)56, (byte)174, (byte)87, (byte)28, (byte)7, (byte)255, (byte)16, (byte)69, (byte)207, (byte)42, (byte)160, (byte)180, (byte)106, (byte)12, (byte)145, (byte)172, (byte)136, (byte)43, (byte)115, (byte)172, (byte)119, (byte)99, (byte)0, (byte)22, (byte)43, (byte)24, (byte)129, (byte)154, (byte)110, (byte)81, (byte)253, (byte)55, (byte)232, (byte)197, (byte)194, (byte)121, (byte)160, (byte)149, (byte)12, (byte)106, (byte)104, (byte)216, (byte)180, (byte)248, (byte)171, (byte)31, (byte)212, (byte)142, (byte)184, (byte)202, (byte)49, (byte)203, (byte)33, (byte)7, (byte)87, (byte)122, (byte)63, (byte)96, (byte)76, (byte)222, (byte)86, (byte)48, (byte)172, (byte)169, (byte)123, (byte)1, (byte)100, (byte)207, (byte)143, (byte)13, (byte)164, (byte)119, (byte)195, (byte)124, (byte)230, (byte)137, (byte)103, (byte)252, (byte)69, (byte)140, (byte)254, (byte)3, (byte)165, (byte)136, (byte)7, (byte)163, (byte)160, (byte)83, (byte)152, (byte)79, (byte)33, (byte)28, (byte)113, (byte)153, (byte)135, (byte)137, (byte)53, (byte)25, (byte)79, (byte)110, (byte)130, (byte)88, (byte)203, (byte)10, (byte)163, (byte)106, (byte)250, (byte)0, (byte)232, (byte)27, (byte)109, (byte)249, (byte)160, (byte)48, (byte)152, (byte)27, (byte)246, (byte)101, (byte)16, (byte)227, (byte)20, (byte)120, (byte)82, (byte)100, (byte)140, (byte)155, (byte)77, (byte)114, (byte)117, (byte)139, (byte)16, (byte)45, (byte)10, (byte)41, (byte)125, (byte)197, (byte)254, (byte)99, (byte)172, (byte)175, (byte)238, (byte)227, (byte)100, (byte)162, (byte)7, (byte)10, (byte)166, (byte)18, (byte)174, (byte)66, (byte)52, (byte)91, (byte)174, (byte)96, (byte)76, (byte)180, (byte)192, (byte)23, (byte)199, (byte)127, (byte)2, (byte)57, (byte)250, (byte)14, (byte)157, (byte)117, (byte)125, (byte)209, (byte)103, (byte)123, (byte)100, (byte)105, (byte)244, (byte)77, (byte)80, (byte)153, (byte)212, (byte)67, (byte)168}));
                Debug.Assert(pack.target_network == (byte)(byte)114);
                Debug.Assert(pack.message_type == (ushort)(ushort)17482);
                Debug.Assert(pack.target_component == (byte)(byte)105);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.payload_SET(new byte[] {(byte)81, (byte)249, (byte)160, (byte)87, (byte)129, (byte)90, (byte)148, (byte)249, (byte)251, (byte)147, (byte)128, (byte)114, (byte)226, (byte)97, (byte)84, (byte)141, (byte)118, (byte)16, (byte)114, (byte)78, (byte)243, (byte)106, (byte)188, (byte)14, (byte)73, (byte)46, (byte)29, (byte)72, (byte)186, (byte)103, (byte)61, (byte)146, (byte)201, (byte)138, (byte)140, (byte)93, (byte)26, (byte)181, (byte)112, (byte)33, (byte)37, (byte)100, (byte)118, (byte)12, (byte)181, (byte)24, (byte)107, (byte)237, (byte)92, (byte)211, (byte)247, (byte)57, (byte)242, (byte)66, (byte)50, (byte)121, (byte)94, (byte)204, (byte)213, (byte)150, (byte)210, (byte)36, (byte)5, (byte)123, (byte)74, (byte)146, (byte)175, (byte)39, (byte)226, (byte)253, (byte)56, (byte)174, (byte)87, (byte)28, (byte)7, (byte)255, (byte)16, (byte)69, (byte)207, (byte)42, (byte)160, (byte)180, (byte)106, (byte)12, (byte)145, (byte)172, (byte)136, (byte)43, (byte)115, (byte)172, (byte)119, (byte)99, (byte)0, (byte)22, (byte)43, (byte)24, (byte)129, (byte)154, (byte)110, (byte)81, (byte)253, (byte)55, (byte)232, (byte)197, (byte)194, (byte)121, (byte)160, (byte)149, (byte)12, (byte)106, (byte)104, (byte)216, (byte)180, (byte)248, (byte)171, (byte)31, (byte)212, (byte)142, (byte)184, (byte)202, (byte)49, (byte)203, (byte)33, (byte)7, (byte)87, (byte)122, (byte)63, (byte)96, (byte)76, (byte)222, (byte)86, (byte)48, (byte)172, (byte)169, (byte)123, (byte)1, (byte)100, (byte)207, (byte)143, (byte)13, (byte)164, (byte)119, (byte)195, (byte)124, (byte)230, (byte)137, (byte)103, (byte)252, (byte)69, (byte)140, (byte)254, (byte)3, (byte)165, (byte)136, (byte)7, (byte)163, (byte)160, (byte)83, (byte)152, (byte)79, (byte)33, (byte)28, (byte)113, (byte)153, (byte)135, (byte)137, (byte)53, (byte)25, (byte)79, (byte)110, (byte)130, (byte)88, (byte)203, (byte)10, (byte)163, (byte)106, (byte)250, (byte)0, (byte)232, (byte)27, (byte)109, (byte)249, (byte)160, (byte)48, (byte)152, (byte)27, (byte)246, (byte)101, (byte)16, (byte)227, (byte)20, (byte)120, (byte)82, (byte)100, (byte)140, (byte)155, (byte)77, (byte)114, (byte)117, (byte)139, (byte)16, (byte)45, (byte)10, (byte)41, (byte)125, (byte)197, (byte)254, (byte)99, (byte)172, (byte)175, (byte)238, (byte)227, (byte)100, (byte)162, (byte)7, (byte)10, (byte)166, (byte)18, (byte)174, (byte)66, (byte)52, (byte)91, (byte)174, (byte)96, (byte)76, (byte)180, (byte)192, (byte)23, (byte)199, (byte)127, (byte)2, (byte)57, (byte)250, (byte)14, (byte)157, (byte)117, (byte)125, (byte)209, (byte)103, (byte)123, (byte)100, (byte)105, (byte)244, (byte)77, (byte)80, (byte)153, (byte)212, (byte)67, (byte)168}, 0) ;
            p248.target_network = (byte)(byte)114;
            p248.message_type = (ushort)(ushort)17482;
            p248.target_system = (byte)(byte)172;
            p248.target_component = (byte)(byte)105;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ver == (byte)(byte)220);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 62, (sbyte) - 20, (sbyte) - 74, (sbyte) - 110, (sbyte)55, (sbyte) - 55, (sbyte)22, (sbyte)122, (sbyte) - 90, (sbyte)54, (sbyte) - 106, (sbyte) - 88, (sbyte)53, (sbyte)22, (sbyte)121, (sbyte)109, (sbyte)126, (sbyte)2, (sbyte) - 92, (sbyte)41, (sbyte) - 77, (sbyte) - 73, (sbyte) - 2, (sbyte)77, (sbyte)75, (sbyte) - 27, (sbyte)30, (sbyte) - 111, (sbyte)1, (sbyte)101, (sbyte) - 49, (sbyte) - 109}));
                Debug.Assert(pack.type == (byte)(byte)184);
                Debug.Assert(pack.address == (ushort)(ushort)25518);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.type = (byte)(byte)184;
            p249.address = (ushort)(ushort)25518;
            p249.ver = (byte)(byte)220;
            p249.value_SET(new sbyte[] {(sbyte) - 62, (sbyte) - 20, (sbyte) - 74, (sbyte) - 110, (sbyte)55, (sbyte) - 55, (sbyte)22, (sbyte)122, (sbyte) - 90, (sbyte)54, (sbyte) - 106, (sbyte) - 88, (sbyte)53, (sbyte)22, (sbyte)121, (sbyte)109, (sbyte)126, (sbyte)2, (sbyte) - 92, (sbyte)41, (sbyte) - 77, (sbyte) - 73, (sbyte) - 2, (sbyte)77, (sbyte)75, (sbyte) - 27, (sbyte)30, (sbyte) - 111, (sbyte)1, (sbyte)101, (sbyte) - 49, (sbyte) - 109}, 0) ;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -2.9031825E37F);
                Debug.Assert(pack.z == (float)2.831124E38F);
                Debug.Assert(pack.y == (float) -1.1987258E38F);
                Debug.Assert(pack.name_LEN(ph) == 7);
                Debug.Assert(pack.name_TRY(ph).Equals("gdyFsur"));
                Debug.Assert(pack.time_usec == (ulong)4461916565062148638L);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.x = (float) -2.9031825E37F;
            p250.y = (float) -1.1987258E38F;
            p250.z = (float)2.831124E38F;
            p250.name_SET("gdyFsur", PH) ;
            p250.time_usec = (ulong)4461916565062148638L;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2723149112U);
                Debug.Assert(pack.name_LEN(ph) == 7);
                Debug.Assert(pack.name_TRY(ph).Equals("yvxlqlt"));
                Debug.Assert(pack.value == (float)2.5328468E38F);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.name_SET("yvxlqlt", PH) ;
            p251.value = (float)2.5328468E38F;
            p251.time_boot_ms = (uint)2723149112U;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("mztQdcuUcw"));
                Debug.Assert(pack.value == (int)1942603323);
                Debug.Assert(pack.time_boot_ms == (uint)744458105U);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.name_SET("mztQdcuUcw", PH) ;
            p252.time_boot_ms = (uint)744458105U;
            p252.value = (int)1942603323;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_ERROR);
                Debug.Assert(pack.text_LEN(ph) == 39);
                Debug.Assert(pack.text_TRY(ph).Equals("lyfrylwriilbxXguctpmlawlwvkmnwfoopiswVy"));
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("lyfrylwriilbxXguctpmlawlwvkmnwfoopiswVy", PH) ;
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_ERROR;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ind == (byte)(byte)141);
                Debug.Assert(pack.time_boot_ms == (uint)648290620U);
                Debug.Assert(pack.value == (float)5.138467E37F);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.value = (float)5.138467E37F;
            p254.time_boot_ms = (uint)648290620U;
            p254.ind = (byte)(byte)141;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.initial_timestamp == (ulong)151206917047988585L);
                Debug.Assert(pack.target_system == (byte)(byte)127);
                Debug.Assert(pack.target_component == (byte)(byte)224);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)114, (byte)247, (byte)223, (byte)154, (byte)197, (byte)132, (byte)166, (byte)194, (byte)30, (byte)82, (byte)195, (byte)214, (byte)58, (byte)196, (byte)176, (byte)14, (byte)81, (byte)165, (byte)7, (byte)193, (byte)145, (byte)156, (byte)138, (byte)7, (byte)55, (byte)2, (byte)102, (byte)52, (byte)52, (byte)197, (byte)0, (byte)111}));
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.initial_timestamp = (ulong)151206917047988585L;
            p256.target_system = (byte)(byte)127;
            p256.secret_key_SET(new byte[] {(byte)114, (byte)247, (byte)223, (byte)154, (byte)197, (byte)132, (byte)166, (byte)194, (byte)30, (byte)82, (byte)195, (byte)214, (byte)58, (byte)196, (byte)176, (byte)14, (byte)81, (byte)165, (byte)7, (byte)193, (byte)145, (byte)156, (byte)138, (byte)7, (byte)55, (byte)2, (byte)102, (byte)52, (byte)52, (byte)197, (byte)0, (byte)111}, 0) ;
            p256.target_component = (byte)(byte)224;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1998030745U);
                Debug.Assert(pack.state == (byte)(byte)80);
                Debug.Assert(pack.last_change_ms == (uint)121938452U);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.last_change_ms = (uint)121938452U;
            p257.time_boot_ms = (uint)1998030745U;
            p257.state = (byte)(byte)80;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)155);
                Debug.Assert(pack.target_component == (byte)(byte)36);
                Debug.Assert(pack.tune_LEN(ph) == 18);
                Debug.Assert(pack.tune_TRY(ph).Equals("unishdkrselojqwoFf"));
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.tune_SET("unishdkrselojqwoFf", PH) ;
            p258.target_system = (byte)(byte)155;
            p258.target_component = (byte)(byte)36;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_size_v == (float) -4.495618E37F);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)15253);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)82, (byte)130, (byte)218, (byte)32, (byte)138, (byte)36, (byte)84, (byte)123, (byte)184, (byte)137, (byte)181, (byte)90, (byte)112, (byte)194, (byte)110, (byte)5, (byte)48, (byte)156, (byte)47, (byte)128, (byte)120, (byte)86, (byte)95, (byte)71, (byte)64, (byte)131, (byte)184, (byte)144, (byte)151, (byte)58, (byte)106, (byte)56}));
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 138);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("yyzumxqsaMyynOkEqvmnfraciqxngrdbbzrkiupjthWabzhaynudrwAgxxfgllvvhdmxlayrnssfkrjmnMisjmsetxiicjenwervfazmrsNqmqnkpxwvGwukrrckmywRjnhbuSbnac"));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)25023);
                Debug.Assert(pack.time_boot_ms == (uint)1054857549U);
                Debug.Assert(pack.focal_length == (float)2.5644983E38F);
                Debug.Assert(pack.sensor_size_h == (float)9.566142E37F);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
                Debug.Assert(pack.lens_id == (byte)(byte)141);
                Debug.Assert(pack.firmware_version == (uint)675826339U);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)49710);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)72, (byte)225, (byte)117, (byte)130, (byte)78, (byte)14, (byte)115, (byte)176, (byte)28, (byte)71, (byte)139, (byte)41, (byte)95, (byte)165, (byte)107, (byte)25, (byte)254, (byte)32, (byte)138, (byte)10, (byte)82, (byte)250, (byte)30, (byte)59, (byte)57, (byte)121, (byte)175, (byte)75, (byte)61, (byte)222, (byte)79, (byte)195}));
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)1054857549U;
            p259.firmware_version = (uint)675826339U;
            p259.focal_length = (float)2.5644983E38F;
            p259.cam_definition_uri_SET("yyzumxqsaMyynOkEqvmnfraciqxngrdbbzrkiupjthWabzhaynudrwAgxxfgllvvhdmxlayrnssfkrjmnMisjmsetxiicjenwervfazmrsNqmqnkpxwvGwukrrckmywRjnhbuSbnac", PH) ;
            p259.cam_definition_version = (ushort)(ushort)15253;
            p259.vendor_name_SET(new byte[] {(byte)82, (byte)130, (byte)218, (byte)32, (byte)138, (byte)36, (byte)84, (byte)123, (byte)184, (byte)137, (byte)181, (byte)90, (byte)112, (byte)194, (byte)110, (byte)5, (byte)48, (byte)156, (byte)47, (byte)128, (byte)120, (byte)86, (byte)95, (byte)71, (byte)64, (byte)131, (byte)184, (byte)144, (byte)151, (byte)58, (byte)106, (byte)56}, 0) ;
            p259.sensor_size_h = (float)9.566142E37F;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE;
            p259.model_name_SET(new byte[] {(byte)72, (byte)225, (byte)117, (byte)130, (byte)78, (byte)14, (byte)115, (byte)176, (byte)28, (byte)71, (byte)139, (byte)41, (byte)95, (byte)165, (byte)107, (byte)25, (byte)254, (byte)32, (byte)138, (byte)10, (byte)82, (byte)250, (byte)30, (byte)59, (byte)57, (byte)121, (byte)175, (byte)75, (byte)61, (byte)222, (byte)79, (byte)195}, 0) ;
            p259.resolution_v = (ushort)(ushort)25023;
            p259.lens_id = (byte)(byte)141;
            p259.sensor_size_v = (float) -4.495618E37F;
            p259.resolution_h = (ushort)(ushort)49710;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
                Debug.Assert(pack.time_boot_ms == (uint)586492300U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)586492300U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.status == (byte)(byte)50);
                Debug.Assert(pack.time_boot_ms == (uint)4071306104U);
                Debug.Assert(pack.used_capacity == (float) -2.5881434E38F);
                Debug.Assert(pack.write_speed == (float) -1.7589727E38F);
                Debug.Assert(pack.read_speed == (float)1.8806408E38F);
                Debug.Assert(pack.storage_count == (byte)(byte)118);
                Debug.Assert(pack.available_capacity == (float) -3.179487E38F);
                Debug.Assert(pack.total_capacity == (float)3.0066303E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)212);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)4071306104U;
            p261.total_capacity = (float)3.0066303E38F;
            p261.storage_count = (byte)(byte)118;
            p261.status = (byte)(byte)50;
            p261.write_speed = (float) -1.7589727E38F;
            p261.storage_id = (byte)(byte)212;
            p261.available_capacity = (float) -3.179487E38F;
            p261.used_capacity = (float) -2.5881434E38F;
            p261.read_speed = (float)1.8806408E38F;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.recording_time_ms == (uint)757118280U);
                Debug.Assert(pack.image_status == (byte)(byte)248);
                Debug.Assert(pack.image_interval == (float) -6.047425E37F);
                Debug.Assert(pack.available_capacity == (float)3.1221818E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3773607584U);
                Debug.Assert(pack.video_status == (byte)(byte)204);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.video_status = (byte)(byte)204;
            p262.recording_time_ms = (uint)757118280U;
            p262.image_interval = (float) -6.047425E37F;
            p262.available_capacity = (float)3.1221818E38F;
            p262.image_status = (byte)(byte)248;
            p262.time_boot_ms = (uint)3773607584U;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.image_index == (int) -233281619);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 26);
                Debug.Assert(pack.lat == (int)230228496);
                Debug.Assert(pack.lon == (int)1643037277);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.2774866E38F, 2.5737717E38F, -1.5996132E38F, 4.267809E37F}));
                Debug.Assert(pack.time_boot_ms == (uint)303925815U);
                Debug.Assert(pack.file_url_LEN(ph) == 163);
                Debug.Assert(pack.file_url_TRY(ph).Equals("raxIufyjqcdybfpobawjwwetelgkmxjnkbpyskbccrdXygzxvpbnzlyFAxtKibknnlczosiwsakevtnyvpowWstplbzpcldpXlWpbuoctyWkarxejpjqxgvxkxrbyzkCvbsudgzuvaXhpMxaugseiyyetfpeoJxzfta"));
                Debug.Assert(pack.relative_alt == (int)232690313);
                Debug.Assert(pack.camera_id == (byte)(byte)89);
                Debug.Assert(pack.alt == (int) -743475230);
                Debug.Assert(pack.time_utc == (ulong)6568157159623249943L);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_utc = (ulong)6568157159623249943L;
            p263.capture_result = (sbyte)(sbyte) - 26;
            p263.lat = (int)230228496;
            p263.alt = (int) -743475230;
            p263.file_url_SET("raxIufyjqcdybfpobawjwwetelgkmxjnkbpyskbccrdXygzxvpbnzlyFAxtKibknnlczosiwsakevtnyvpowWstplbzpcldpXlWpbuoctyWkarxejpjqxgvxkxrbyzkCvbsudgzuvaXhpMxaugseiyyetfpeoJxzfta", PH) ;
            p263.lon = (int)1643037277;
            p263.image_index = (int) -233281619;
            p263.camera_id = (byte)(byte)89;
            p263.time_boot_ms = (uint)303925815U;
            p263.relative_alt = (int)232690313;
            p263.q_SET(new float[] {1.2774866E38F, 2.5737717E38F, -1.5996132E38F, 4.267809E37F}, 0) ;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.takeoff_time_utc == (ulong)3560088216937415683L);
                Debug.Assert(pack.flight_uuid == (ulong)3584009688456972908L);
                Debug.Assert(pack.arming_time_utc == (ulong)8485658341646927909L);
                Debug.Assert(pack.time_boot_ms == (uint)1492739914U);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.takeoff_time_utc = (ulong)3560088216937415683L;
            p264.time_boot_ms = (uint)1492739914U;
            p264.arming_time_utc = (ulong)8485658341646927909L;
            p264.flight_uuid = (ulong)3584009688456972908L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)2.3945654E38F);
                Debug.Assert(pack.time_boot_ms == (uint)937943549U);
                Debug.Assert(pack.roll == (float)1.6611657E38F);
                Debug.Assert(pack.pitch == (float)3.3179511E38F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.yaw = (float)2.3945654E38F;
            p265.pitch = (float)3.3179511E38F;
            p265.roll = (float)1.6611657E38F;
            p265.time_boot_ms = (uint)937943549U;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.first_message_offset == (byte)(byte)254);
                Debug.Assert(pack.length == (byte)(byte)28);
                Debug.Assert(pack.target_system == (byte)(byte)195);
                Debug.Assert(pack.sequence == (ushort)(ushort)55311);
                Debug.Assert(pack.target_component == (byte)(byte)20);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)52, (byte)2, (byte)105, (byte)239, (byte)93, (byte)241, (byte)253, (byte)94, (byte)79, (byte)166, (byte)44, (byte)194, (byte)169, (byte)226, (byte)86, (byte)113, (byte)56, (byte)139, (byte)15, (byte)78, (byte)248, (byte)73, (byte)98, (byte)43, (byte)125, (byte)51, (byte)82, (byte)160, (byte)110, (byte)198, (byte)52, (byte)255, (byte)64, (byte)16, (byte)213, (byte)227, (byte)73, (byte)164, (byte)115, (byte)101, (byte)229, (byte)254, (byte)41, (byte)20, (byte)255, (byte)16, (byte)236, (byte)16, (byte)199, (byte)149, (byte)202, (byte)232, (byte)187, (byte)178, (byte)177, (byte)136, (byte)229, (byte)44, (byte)0, (byte)194, (byte)156, (byte)129, (byte)61, (byte)176, (byte)142, (byte)14, (byte)178, (byte)54, (byte)202, (byte)150, (byte)95, (byte)132, (byte)16, (byte)157, (byte)98, (byte)128, (byte)151, (byte)25, (byte)189, (byte)233, (byte)64, (byte)191, (byte)239, (byte)1, (byte)242, (byte)154, (byte)11, (byte)61, (byte)97, (byte)200, (byte)56, (byte)138, (byte)2, (byte)172, (byte)27, (byte)67, (byte)79, (byte)233, (byte)115, (byte)44, (byte)131, (byte)96, (byte)4, (byte)187, (byte)189, (byte)211, (byte)27, (byte)166, (byte)114, (byte)65, (byte)25, (byte)65, (byte)89, (byte)79, (byte)197, (byte)64, (byte)192, (byte)180, (byte)81, (byte)117, (byte)0, (byte)100, (byte)249, (byte)35, (byte)128, (byte)202, (byte)95, (byte)8, (byte)34, (byte)222, (byte)72, (byte)193, (byte)152, (byte)244, (byte)239, (byte)52, (byte)232, (byte)240, (byte)211, (byte)250, (byte)8, (byte)238, (byte)179, (byte)162, (byte)43, (byte)165, (byte)64, (byte)62, (byte)193, (byte)79, (byte)141, (byte)150, (byte)98, (byte)40, (byte)150, (byte)32, (byte)134, (byte)89, (byte)234, (byte)161, (byte)26, (byte)122, (byte)251, (byte)119, (byte)65, (byte)192, (byte)113, (byte)141, (byte)47, (byte)23, (byte)195, (byte)88, (byte)46, (byte)159, (byte)84, (byte)167, (byte)165, (byte)11, (byte)123, (byte)102, (byte)235, (byte)74, (byte)201, (byte)250, (byte)191, (byte)220, (byte)239, (byte)35, (byte)222, (byte)42, (byte)168, (byte)228, (byte)125, (byte)255, (byte)29, (byte)195, (byte)233, (byte)187, (byte)210, (byte)48, (byte)72, (byte)127, (byte)189, (byte)35, (byte)197, (byte)158, (byte)183, (byte)20, (byte)146, (byte)9, (byte)59, (byte)17, (byte)188, (byte)86, (byte)56, (byte)200, (byte)182, (byte)88, (byte)220, (byte)85, (byte)6, (byte)17, (byte)78, (byte)202, (byte)176, (byte)115, (byte)115, (byte)130, (byte)79, (byte)227, (byte)41, (byte)140, (byte)165, (byte)63, (byte)166, (byte)239, (byte)90, (byte)75, (byte)226, (byte)69, (byte)77, (byte)47, (byte)16, (byte)229, (byte)108, (byte)125, (byte)203, (byte)182, (byte)134}));
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.first_message_offset = (byte)(byte)254;
            p266.data__SET(new byte[] {(byte)52, (byte)2, (byte)105, (byte)239, (byte)93, (byte)241, (byte)253, (byte)94, (byte)79, (byte)166, (byte)44, (byte)194, (byte)169, (byte)226, (byte)86, (byte)113, (byte)56, (byte)139, (byte)15, (byte)78, (byte)248, (byte)73, (byte)98, (byte)43, (byte)125, (byte)51, (byte)82, (byte)160, (byte)110, (byte)198, (byte)52, (byte)255, (byte)64, (byte)16, (byte)213, (byte)227, (byte)73, (byte)164, (byte)115, (byte)101, (byte)229, (byte)254, (byte)41, (byte)20, (byte)255, (byte)16, (byte)236, (byte)16, (byte)199, (byte)149, (byte)202, (byte)232, (byte)187, (byte)178, (byte)177, (byte)136, (byte)229, (byte)44, (byte)0, (byte)194, (byte)156, (byte)129, (byte)61, (byte)176, (byte)142, (byte)14, (byte)178, (byte)54, (byte)202, (byte)150, (byte)95, (byte)132, (byte)16, (byte)157, (byte)98, (byte)128, (byte)151, (byte)25, (byte)189, (byte)233, (byte)64, (byte)191, (byte)239, (byte)1, (byte)242, (byte)154, (byte)11, (byte)61, (byte)97, (byte)200, (byte)56, (byte)138, (byte)2, (byte)172, (byte)27, (byte)67, (byte)79, (byte)233, (byte)115, (byte)44, (byte)131, (byte)96, (byte)4, (byte)187, (byte)189, (byte)211, (byte)27, (byte)166, (byte)114, (byte)65, (byte)25, (byte)65, (byte)89, (byte)79, (byte)197, (byte)64, (byte)192, (byte)180, (byte)81, (byte)117, (byte)0, (byte)100, (byte)249, (byte)35, (byte)128, (byte)202, (byte)95, (byte)8, (byte)34, (byte)222, (byte)72, (byte)193, (byte)152, (byte)244, (byte)239, (byte)52, (byte)232, (byte)240, (byte)211, (byte)250, (byte)8, (byte)238, (byte)179, (byte)162, (byte)43, (byte)165, (byte)64, (byte)62, (byte)193, (byte)79, (byte)141, (byte)150, (byte)98, (byte)40, (byte)150, (byte)32, (byte)134, (byte)89, (byte)234, (byte)161, (byte)26, (byte)122, (byte)251, (byte)119, (byte)65, (byte)192, (byte)113, (byte)141, (byte)47, (byte)23, (byte)195, (byte)88, (byte)46, (byte)159, (byte)84, (byte)167, (byte)165, (byte)11, (byte)123, (byte)102, (byte)235, (byte)74, (byte)201, (byte)250, (byte)191, (byte)220, (byte)239, (byte)35, (byte)222, (byte)42, (byte)168, (byte)228, (byte)125, (byte)255, (byte)29, (byte)195, (byte)233, (byte)187, (byte)210, (byte)48, (byte)72, (byte)127, (byte)189, (byte)35, (byte)197, (byte)158, (byte)183, (byte)20, (byte)146, (byte)9, (byte)59, (byte)17, (byte)188, (byte)86, (byte)56, (byte)200, (byte)182, (byte)88, (byte)220, (byte)85, (byte)6, (byte)17, (byte)78, (byte)202, (byte)176, (byte)115, (byte)115, (byte)130, (byte)79, (byte)227, (byte)41, (byte)140, (byte)165, (byte)63, (byte)166, (byte)239, (byte)90, (byte)75, (byte)226, (byte)69, (byte)77, (byte)47, (byte)16, (byte)229, (byte)108, (byte)125, (byte)203, (byte)182, (byte)134}, 0) ;
            p266.length = (byte)(byte)28;
            p266.target_component = (byte)(byte)20;
            p266.sequence = (ushort)(ushort)55311;
            p266.target_system = (byte)(byte)195;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.first_message_offset == (byte)(byte)127);
                Debug.Assert(pack.length == (byte)(byte)156);
                Debug.Assert(pack.target_component == (byte)(byte)124);
                Debug.Assert(pack.sequence == (ushort)(ushort)17031);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)185, (byte)255, (byte)110, (byte)216, (byte)208, (byte)30, (byte)170, (byte)236, (byte)194, (byte)109, (byte)142, (byte)172, (byte)20, (byte)19, (byte)15, (byte)115, (byte)38, (byte)110, (byte)66, (byte)19, (byte)147, (byte)73, (byte)51, (byte)206, (byte)111, (byte)23, (byte)176, (byte)186, (byte)245, (byte)83, (byte)69, (byte)90, (byte)190, (byte)130, (byte)36, (byte)159, (byte)56, (byte)3, (byte)154, (byte)127, (byte)3, (byte)75, (byte)178, (byte)233, (byte)135, (byte)11, (byte)228, (byte)99, (byte)141, (byte)248, (byte)65, (byte)250, (byte)230, (byte)46, (byte)18, (byte)234, (byte)238, (byte)232, (byte)53, (byte)18, (byte)2, (byte)21, (byte)190, (byte)13, (byte)105, (byte)254, (byte)99, (byte)196, (byte)50, (byte)171, (byte)66, (byte)155, (byte)253, (byte)9, (byte)74, (byte)209, (byte)215, (byte)19, (byte)169, (byte)107, (byte)240, (byte)240, (byte)151, (byte)204, (byte)114, (byte)7, (byte)231, (byte)169, (byte)5, (byte)146, (byte)86, (byte)204, (byte)40, (byte)194, (byte)27, (byte)120, (byte)2, (byte)225, (byte)149, (byte)34, (byte)184, (byte)74, (byte)250, (byte)58, (byte)242, (byte)211, (byte)61, (byte)132, (byte)170, (byte)241, (byte)185, (byte)128, (byte)76, (byte)143, (byte)131, (byte)26, (byte)97, (byte)46, (byte)11, (byte)252, (byte)208, (byte)121, (byte)45, (byte)182, (byte)120, (byte)155, (byte)249, (byte)253, (byte)200, (byte)248, (byte)95, (byte)50, (byte)223, (byte)237, (byte)238, (byte)184, (byte)117, (byte)62, (byte)13, (byte)106, (byte)98, (byte)142, (byte)225, (byte)164, (byte)187, (byte)168, (byte)171, (byte)66, (byte)11, (byte)109, (byte)233, (byte)226, (byte)166, (byte)64, (byte)79, (byte)36, (byte)117, (byte)235, (byte)251, (byte)35, (byte)154, (byte)234, (byte)33, (byte)49, (byte)45, (byte)77, (byte)222, (byte)105, (byte)213, (byte)24, (byte)40, (byte)161, (byte)93, (byte)145, (byte)84, (byte)246, (byte)33, (byte)42, (byte)196, (byte)45, (byte)53, (byte)235, (byte)221, (byte)241, (byte)166, (byte)128, (byte)173, (byte)65, (byte)193, (byte)121, (byte)250, (byte)33, (byte)183, (byte)39, (byte)130, (byte)55, (byte)105, (byte)236, (byte)132, (byte)174, (byte)246, (byte)21, (byte)225, (byte)104, (byte)200, (byte)30, (byte)97, (byte)115, (byte)188, (byte)49, (byte)111, (byte)234, (byte)133, (byte)86, (byte)31, (byte)24, (byte)47, (byte)38, (byte)151, (byte)160, (byte)238, (byte)213, (byte)92, (byte)217, (byte)126, (byte)105, (byte)3, (byte)125, (byte)217, (byte)11, (byte)158, (byte)123, (byte)139, (byte)121, (byte)153, (byte)8, (byte)158, (byte)142, (byte)149, (byte)25, (byte)97, (byte)134, (byte)104, (byte)54, (byte)178, (byte)4, (byte)103, (byte)84, (byte)217}));
                Debug.Assert(pack.target_system == (byte)(byte)47);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.length = (byte)(byte)156;
            p267.target_system = (byte)(byte)47;
            p267.data__SET(new byte[] {(byte)185, (byte)255, (byte)110, (byte)216, (byte)208, (byte)30, (byte)170, (byte)236, (byte)194, (byte)109, (byte)142, (byte)172, (byte)20, (byte)19, (byte)15, (byte)115, (byte)38, (byte)110, (byte)66, (byte)19, (byte)147, (byte)73, (byte)51, (byte)206, (byte)111, (byte)23, (byte)176, (byte)186, (byte)245, (byte)83, (byte)69, (byte)90, (byte)190, (byte)130, (byte)36, (byte)159, (byte)56, (byte)3, (byte)154, (byte)127, (byte)3, (byte)75, (byte)178, (byte)233, (byte)135, (byte)11, (byte)228, (byte)99, (byte)141, (byte)248, (byte)65, (byte)250, (byte)230, (byte)46, (byte)18, (byte)234, (byte)238, (byte)232, (byte)53, (byte)18, (byte)2, (byte)21, (byte)190, (byte)13, (byte)105, (byte)254, (byte)99, (byte)196, (byte)50, (byte)171, (byte)66, (byte)155, (byte)253, (byte)9, (byte)74, (byte)209, (byte)215, (byte)19, (byte)169, (byte)107, (byte)240, (byte)240, (byte)151, (byte)204, (byte)114, (byte)7, (byte)231, (byte)169, (byte)5, (byte)146, (byte)86, (byte)204, (byte)40, (byte)194, (byte)27, (byte)120, (byte)2, (byte)225, (byte)149, (byte)34, (byte)184, (byte)74, (byte)250, (byte)58, (byte)242, (byte)211, (byte)61, (byte)132, (byte)170, (byte)241, (byte)185, (byte)128, (byte)76, (byte)143, (byte)131, (byte)26, (byte)97, (byte)46, (byte)11, (byte)252, (byte)208, (byte)121, (byte)45, (byte)182, (byte)120, (byte)155, (byte)249, (byte)253, (byte)200, (byte)248, (byte)95, (byte)50, (byte)223, (byte)237, (byte)238, (byte)184, (byte)117, (byte)62, (byte)13, (byte)106, (byte)98, (byte)142, (byte)225, (byte)164, (byte)187, (byte)168, (byte)171, (byte)66, (byte)11, (byte)109, (byte)233, (byte)226, (byte)166, (byte)64, (byte)79, (byte)36, (byte)117, (byte)235, (byte)251, (byte)35, (byte)154, (byte)234, (byte)33, (byte)49, (byte)45, (byte)77, (byte)222, (byte)105, (byte)213, (byte)24, (byte)40, (byte)161, (byte)93, (byte)145, (byte)84, (byte)246, (byte)33, (byte)42, (byte)196, (byte)45, (byte)53, (byte)235, (byte)221, (byte)241, (byte)166, (byte)128, (byte)173, (byte)65, (byte)193, (byte)121, (byte)250, (byte)33, (byte)183, (byte)39, (byte)130, (byte)55, (byte)105, (byte)236, (byte)132, (byte)174, (byte)246, (byte)21, (byte)225, (byte)104, (byte)200, (byte)30, (byte)97, (byte)115, (byte)188, (byte)49, (byte)111, (byte)234, (byte)133, (byte)86, (byte)31, (byte)24, (byte)47, (byte)38, (byte)151, (byte)160, (byte)238, (byte)213, (byte)92, (byte)217, (byte)126, (byte)105, (byte)3, (byte)125, (byte)217, (byte)11, (byte)158, (byte)123, (byte)139, (byte)121, (byte)153, (byte)8, (byte)158, (byte)142, (byte)149, (byte)25, (byte)97, (byte)134, (byte)104, (byte)54, (byte)178, (byte)4, (byte)103, (byte)84, (byte)217}, 0) ;
            p267.first_message_offset = (byte)(byte)127;
            p267.target_component = (byte)(byte)124;
            p267.sequence = (ushort)(ushort)17031;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)49359);
                Debug.Assert(pack.target_system == (byte)(byte)135);
                Debug.Assert(pack.target_component == (byte)(byte)4);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)135;
            p268.sequence = (ushort)(ushort)49359;
            p268.target_component = (byte)(byte)4;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.framerate == (float) -1.1855863E38F);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)59590);
                Debug.Assert(pack.bitrate == (uint)2990116503U);
                Debug.Assert(pack.rotation == (ushort)(ushort)28959);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)61375);
                Debug.Assert(pack.camera_id == (byte)(byte)46);
                Debug.Assert(pack.status == (byte)(byte)13);
                Debug.Assert(pack.uri_LEN(ph) == 24);
                Debug.Assert(pack.uri_TRY(ph).Equals("lprcrqgxijWvBevvuiibkqxf"));
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.framerate = (float) -1.1855863E38F;
            p269.status = (byte)(byte)13;
            p269.camera_id = (byte)(byte)46;
            p269.resolution_v = (ushort)(ushort)59590;
            p269.rotation = (ushort)(ushort)28959;
            p269.resolution_h = (ushort)(ushort)61375;
            p269.bitrate = (uint)2990116503U;
            p269.uri_SET("lprcrqgxijWvBevvuiibkqxf", PH) ;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_LEN(ph) == 219);
                Debug.Assert(pack.uri_TRY(ph).Equals("kjzeybmlcqjsegukYinqugAtnhWckecvyzdqbkbjwTzavyXnccdhvgGwtiIkaqgkwzjvhlbrjszfosOhmwEpqqyqxrrEKlroovkzubwhvjmqmuajzcmbpzwvoybhhuqzmwooytbrsqnWgajmbxozykvaxrzRbjfohlScwsVlqgxejwnbufzwqayfsQhcadaknjqbfngzinqqndqpzttebgpYxjo"));
                Debug.Assert(pack.target_component == (byte)(byte)131);
                Debug.Assert(pack.rotation == (ushort)(ushort)16681);
                Debug.Assert(pack.framerate == (float)1.9566656E37F);
                Debug.Assert(pack.target_system == (byte)(byte)167);
                Debug.Assert(pack.camera_id == (byte)(byte)155);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)23592);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)17781);
                Debug.Assert(pack.bitrate == (uint)3884688091U);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)167;
            p270.camera_id = (byte)(byte)155;
            p270.bitrate = (uint)3884688091U;
            p270.framerate = (float)1.9566656E37F;
            p270.target_component = (byte)(byte)131;
            p270.rotation = (ushort)(ushort)16681;
            p270.resolution_h = (ushort)(ushort)23592;
            p270.uri_SET("kjzeybmlcqjsegukYinqugAtnhWckecvyzdqbkbjwTzavyXnccdhvgGwtiIkaqgkwzjvhlbrjszfosOhmwEpqqyqxrrEKlroovkzubwhvjmqmuajzcmbpzwvoybhhuqzmwooytbrsqnWgajmbxozykvaxrzRbjfohlScwsVlqgxejwnbufzwqayfsQhcadaknjqbfngzinqqndqpzttebgpYxjo", PH) ;
            p270.resolution_v = (ushort)(ushort)17781;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 14);
                Debug.Assert(pack.password_TRY(ph).Equals("NygklYjbwgWfav"));
                Debug.Assert(pack.ssid_LEN(ph) == 11);
                Debug.Assert(pack.ssid_TRY(ph).Equals("duktuwlhotj"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("NygklYjbwgWfav", PH) ;
            p299.ssid_SET("duktuwlhotj", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)161, (byte)144, (byte)151, (byte)59, (byte)55, (byte)66, (byte)199, (byte)13}));
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)94, (byte)38, (byte)4, (byte)185, (byte)46, (byte)40, (byte)109, (byte)12}));
                Debug.Assert(pack.max_version == (ushort)(ushort)39280);
                Debug.Assert(pack.version == (ushort)(ushort)58428);
                Debug.Assert(pack.min_version == (ushort)(ushort)33989);
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.spec_version_hash_SET(new byte[] {(byte)94, (byte)38, (byte)4, (byte)185, (byte)46, (byte)40, (byte)109, (byte)12}, 0) ;
            p300.min_version = (ushort)(ushort)33989;
            p300.library_version_hash_SET(new byte[] {(byte)161, (byte)144, (byte)151, (byte)59, (byte)55, (byte)66, (byte)199, (byte)13}, 0) ;
            p300.max_version = (ushort)(ushort)39280;
            p300.version = (ushort)(ushort)58428;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
                Debug.Assert(pack.time_usec == (ulong)3209196349939166963L);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
                Debug.Assert(pack.sub_mode == (byte)(byte)79);
                Debug.Assert(pack.uptime_sec == (uint)3810057786U);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)44820);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.vendor_specific_status_code = (ushort)(ushort)44820;
            p310.sub_mode = (byte)(byte)79;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            p310.uptime_sec = (uint)3810057786U;
            p310.time_usec = (ulong)3209196349939166963L;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_version_major == (byte)(byte)112);
                Debug.Assert(pack.uptime_sec == (uint)306833042U);
                Debug.Assert(pack.sw_vcs_commit == (uint)3547649511U);
                Debug.Assert(pack.time_usec == (ulong)3480696860596703450L);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)32, (byte)221, (byte)51, (byte)179, (byte)97, (byte)168, (byte)109, (byte)209, (byte)47, (byte)230, (byte)77, (byte)96, (byte)130, (byte)70, (byte)123, (byte)102}));
                Debug.Assert(pack.name_LEN(ph) == 24);
                Debug.Assert(pack.name_TRY(ph).Equals("duauTyonozzxmuvhswbjqgZy"));
                Debug.Assert(pack.hw_version_minor == (byte)(byte)58);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)90);
                Debug.Assert(pack.hw_version_major == (byte)(byte)170);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.sw_version_minor = (byte)(byte)90;
            p311.name_SET("duauTyonozzxmuvhswbjqgZy", PH) ;
            p311.uptime_sec = (uint)306833042U;
            p311.sw_version_major = (byte)(byte)112;
            p311.hw_unique_id_SET(new byte[] {(byte)32, (byte)221, (byte)51, (byte)179, (byte)97, (byte)168, (byte)109, (byte)209, (byte)47, (byte)230, (byte)77, (byte)96, (byte)130, (byte)70, (byte)123, (byte)102}, 0) ;
            p311.hw_version_major = (byte)(byte)170;
            p311.sw_vcs_commit = (uint)3547649511U;
            p311.hw_version_minor = (byte)(byte)58;
            p311.time_usec = (ulong)3480696860596703450L;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)92);
                Debug.Assert(pack.param_index == (short)(short) -28856);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("v"));
                Debug.Assert(pack.target_system == (byte)(byte)19);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_id_SET("v", PH) ;
            p320.target_component = (byte)(byte)92;
            p320.target_system = (byte)(byte)19;
            p320.param_index = (short)(short) -28856;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)205);
                Debug.Assert(pack.target_component == (byte)(byte)19);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)19;
            p321.target_system = (byte)(byte)205;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_count == (ushort)(ushort)61805);
                Debug.Assert(pack.param_index == (ushort)(ushort)42658);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ckxft"));
                Debug.Assert(pack.param_value_LEN(ph) == 116);
                Debug.Assert(pack.param_value_TRY(ph).Equals("shbtsYreyifocfzpffamJyiepIsevivhkfxvntogylrqjzmlonqyeyaaInjcfcsntftxyengbqyexccyzxkwzahmpvjloofwhnqLiomShhqIyBpivecl"));
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_index = (ushort)(ushort)42658;
            p322.param_count = (ushort)(ushort)61805;
            p322.param_id_SET("ckxft", PH) ;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p322.param_value_SET("shbtsYreyifocfzpffamJyiepIsevivhkfxvntogylrqjzmlonqyeyaaInjcfcsntftxyengbqyexccyzxkwzahmpvjloofwhnqLiomShhqIyBpivecl", PH) ;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
                Debug.Assert(pack.target_system == (byte)(byte)197);
                Debug.Assert(pack.param_value_LEN(ph) == 67);
                Debug.Assert(pack.param_value_TRY(ph).Equals("oicnvoriovujseXaqhhVdpQkqvirgiBibawxcuBxmDbmsbeutnuCyoegwjYaegjczta"));
                Debug.Assert(pack.target_component == (byte)(byte)99);
                Debug.Assert(pack.param_id_LEN(ph) == 6);
                Debug.Assert(pack.param_id_TRY(ph).Equals("rapdXm"));
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_id_SET("rapdXm", PH) ;
            p323.target_system = (byte)(byte)197;
            p323.target_component = (byte)(byte)99;
            p323.param_value_SET("oicnvoriovujseXaqhhVdpQkqvirgiBibawxcuBxmDbmsbeutnuCyoegwjYaegjczta", PH) ;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 26);
                Debug.Assert(pack.param_value_TRY(ph).Equals("ycnFljillcorBuLOjlpXmladWh"));
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("h"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            p324.param_id_SET("h", PH) ;
            p324.param_value_SET("ycnFljillcorBuLOjlpXmladWh", PH) ;
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.increment == (byte)(byte)227);
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
                Debug.Assert(pack.time_usec == (ulong)5478830033433216451L);
                Debug.Assert(pack.max_distance == (ushort)(ushort)20779);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)63380, (ushort)16668, (ushort)50874, (ushort)2930, (ushort)39884, (ushort)57968, (ushort)12961, (ushort)16781, (ushort)39656, (ushort)13669, (ushort)53440, (ushort)26515, (ushort)56184, (ushort)51677, (ushort)11305, (ushort)24570, (ushort)62292, (ushort)47606, (ushort)51258, (ushort)20035, (ushort)24709, (ushort)61964, (ushort)49083, (ushort)59791, (ushort)56776, (ushort)60510, (ushort)4773, (ushort)31, (ushort)48800, (ushort)35203, (ushort)23778, (ushort)48833, (ushort)10855, (ushort)5288, (ushort)64247, (ushort)6623, (ushort)28271, (ushort)27853, (ushort)54971, (ushort)28538, (ushort)23034, (ushort)3656, (ushort)10936, (ushort)30053, (ushort)14821, (ushort)34462, (ushort)30218, (ushort)17926, (ushort)28673, (ushort)51450, (ushort)56876, (ushort)56964, (ushort)39651, (ushort)28250, (ushort)406, (ushort)22539, (ushort)994, (ushort)24144, (ushort)55775, (ushort)53456, (ushort)55898, (ushort)19011, (ushort)27628, (ushort)38327, (ushort)12343, (ushort)65101, (ushort)33637, (ushort)1286, (ushort)22048, (ushort)4578, (ushort)41794, (ushort)60191}));
                Debug.Assert(pack.min_distance == (ushort)(ushort)40756);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.min_distance = (ushort)(ushort)40756;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p330.max_distance = (ushort)(ushort)20779;
            p330.increment = (byte)(byte)227;
            p330.time_usec = (ulong)5478830033433216451L;
            p330.distances_SET(new ushort[] {(ushort)63380, (ushort)16668, (ushort)50874, (ushort)2930, (ushort)39884, (ushort)57968, (ushort)12961, (ushort)16781, (ushort)39656, (ushort)13669, (ushort)53440, (ushort)26515, (ushort)56184, (ushort)51677, (ushort)11305, (ushort)24570, (ushort)62292, (ushort)47606, (ushort)51258, (ushort)20035, (ushort)24709, (ushort)61964, (ushort)49083, (ushort)59791, (ushort)56776, (ushort)60510, (ushort)4773, (ushort)31, (ushort)48800, (ushort)35203, (ushort)23778, (ushort)48833, (ushort)10855, (ushort)5288, (ushort)64247, (ushort)6623, (ushort)28271, (ushort)27853, (ushort)54971, (ushort)28538, (ushort)23034, (ushort)3656, (ushort)10936, (ushort)30053, (ushort)14821, (ushort)34462, (ushort)30218, (ushort)17926, (ushort)28673, (ushort)51450, (ushort)56876, (ushort)56964, (ushort)39651, (ushort)28250, (ushort)406, (ushort)22539, (ushort)994, (ushort)24144, (ushort)55775, (ushort)53456, (ushort)55898, (ushort)19011, (ushort)27628, (ushort)38327, (ushort)12343, (ushort)65101, (ushort)33637, (ushort)1286, (ushort)22048, (ushort)4578, (ushort)41794, (ushort)60191}, 0) ;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
        }
    }
}
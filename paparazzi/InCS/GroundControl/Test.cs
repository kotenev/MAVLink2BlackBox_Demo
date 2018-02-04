
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
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 7, data, 276);
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
                    BitUtils.set_bits(id, 3, data, 283);
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
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 7, data, 276);
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
                    BitUtils.set_bits(id, 3, data, 283);
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
        new class SCRIPT_ITEM : GroundControl.SCRIPT_ITEM
        {
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
            public string name_TRY(Inside ph)//The name of the mission script, NULL terminated.
            {
                if(ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) return null;
                return new string(name_GET(ph, new char[ph.items], 0));
            }
            public char[]name_GET(Inside ph, char[] dst_ch, int pos) //The name of the mission script, NULL terminated.
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int name_LEN(Inside ph)
            {
                return (ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class SCRIPT_REQUEST : GroundControl.SCRIPT_REQUEST
        {
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
        }
        new class SCRIPT_REQUEST_LIST : GroundControl.SCRIPT_REQUEST_LIST
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
        new class SCRIPT_COUNT : GroundControl.SCRIPT_COUNT
        {
            public ushort count //Number of script items in the sequence
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
        }
        new class SCRIPT_CURRENT : GroundControl.SCRIPT_CURRENT
        {
            public ushort seq //Active Sequence
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
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
            public void OnSCRIPT_ITEMReceive_direct(Channel src, Inside ph, SCRIPT_ITEM pack) {OnSCRIPT_ITEMReceive(this, ph,  pack);}
            public event SCRIPT_ITEMReceiveHandler OnSCRIPT_ITEMReceive;
            public delegate void SCRIPT_ITEMReceiveHandler(Channel src, Inside ph, SCRIPT_ITEM pack);
            public void OnSCRIPT_REQUESTReceive_direct(Channel src, Inside ph, SCRIPT_REQUEST pack) {OnSCRIPT_REQUESTReceive(this, ph,  pack);}
            public event SCRIPT_REQUESTReceiveHandler OnSCRIPT_REQUESTReceive;
            public delegate void SCRIPT_REQUESTReceiveHandler(Channel src, Inside ph, SCRIPT_REQUEST pack);
            public void OnSCRIPT_REQUEST_LISTReceive_direct(Channel src, Inside ph, SCRIPT_REQUEST_LIST pack) {OnSCRIPT_REQUEST_LISTReceive(this, ph,  pack);}
            public event SCRIPT_REQUEST_LISTReceiveHandler OnSCRIPT_REQUEST_LISTReceive;
            public delegate void SCRIPT_REQUEST_LISTReceiveHandler(Channel src, Inside ph, SCRIPT_REQUEST_LIST pack);
            public void OnSCRIPT_COUNTReceive_direct(Channel src, Inside ph, SCRIPT_COUNT pack) {OnSCRIPT_COUNTReceive(this, ph,  pack);}
            public event SCRIPT_COUNTReceiveHandler OnSCRIPT_COUNTReceive;
            public delegate void SCRIPT_COUNTReceiveHandler(Channel src, Inside ph, SCRIPT_COUNT pack);
            public void OnSCRIPT_CURRENTReceive_direct(Channel src, Inside ph, SCRIPT_CURRENT pack) {OnSCRIPT_CURRENTReceive(this, ph,  pack);}
            public event SCRIPT_CURRENTReceiveHandler OnSCRIPT_CURRENTReceive;
            public delegate void SCRIPT_CURRENTReceiveHandler(Channel src, Inside ph, SCRIPT_CURRENT pack);
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
                    case 180:
                        if(pack == null) return new SCRIPT_ITEM();
                        OnSCRIPT_ITEMReceive(this, ph, (SCRIPT_ITEM) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 181:
                        if(pack == null) return new SCRIPT_REQUEST();
                        OnSCRIPT_REQUESTReceive(this, ph, (SCRIPT_REQUEST) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 182:
                        if(pack == null) return new SCRIPT_REQUEST_LIST();
                        OnSCRIPT_REQUEST_LISTReceive(this, ph, (SCRIPT_REQUEST_LIST) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 183:
                        if(pack == null) return new SCRIPT_COUNT();
                        OnSCRIPT_COUNTReceive(this, ph, (SCRIPT_COUNT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 184:
                        if(pack == null) return new SCRIPT_CURRENT();
                        OnSCRIPT_CURRENTReceive(this, ph, (SCRIPT_CURRENT) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                Debug.Assert(pack.mavlink_version == (byte)(byte)162);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_CRITICAL);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY);
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_ADSB);
                Debug.Assert(pack.custom_mode == (uint)1715919364U);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.custom_mode = (uint)1715919364U;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_CRITICAL;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED;
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_ADSB;
            p0.mavlink_version = (byte)(byte)162;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY;
            SMP_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)59240);
                Debug.Assert(pack.current_battery == (short)(short)21409);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)24813);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)40033);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)62);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)45887);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)58848);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)32813);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)17210);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
                Debug.Assert(pack.load == (ushort)(ushort)34105);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.errors_comm = (ushort)(ushort)40033;
            p1.voltage_battery = (ushort)(ushort)45887;
            p1.battery_remaining = (sbyte)(sbyte)62;
            p1.load = (ushort)(ushort)34105;
            p1.errors_count1 = (ushort)(ushort)17210;
            p1.errors_count2 = (ushort)(ushort)32813;
            p1.current_battery = (short)(short)21409;
            p1.errors_count4 = (ushort)(ushort)24813;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION;
            p1.errors_count3 = (ushort)(ushort)58848;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
            p1.drop_rate_comm = (ushort)(ushort)59240;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2;
            SMP_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)468254144U);
                Debug.Assert(pack.time_unix_usec == (ulong)7176839524060159049L);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)7176839524060159049L;
            p2.time_boot_ms = (uint)468254144U;
            SMP_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -7.9860725E37F);
                Debug.Assert(pack.z == (float) -1.5604214E38F);
                Debug.Assert(pack.yaw_rate == (float)1.1448546E38F);
                Debug.Assert(pack.yaw == (float)1.3948733E38F);
                Debug.Assert(pack.vz == (float) -9.727882E37F);
                Debug.Assert(pack.y == (float)1.5279001E38F);
                Debug.Assert(pack.afy == (float)8.965196E37F);
                Debug.Assert(pack.afx == (float) -1.4077847E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)11366);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.afz == (float)1.6374609E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2050380197U);
                Debug.Assert(pack.vy == (float)2.9377955E38F);
                Debug.Assert(pack.vx == (float) -1.3014637E38F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.afz = (float)1.6374609E38F;
            p3.yaw_rate = (float)1.1448546E38F;
            p3.x = (float) -7.9860725E37F;
            p3.type_mask = (ushort)(ushort)11366;
            p3.vz = (float) -9.727882E37F;
            p3.afy = (float)8.965196E37F;
            p3.y = (float)1.5279001E38F;
            p3.vx = (float) -1.3014637E38F;
            p3.z = (float) -1.5604214E38F;
            p3.yaw = (float)1.3948733E38F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p3.vy = (float)2.9377955E38F;
            p3.time_boot_ms = (uint)2050380197U;
            p3.afx = (float) -1.4077847E37F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)40);
                Debug.Assert(pack.seq == (uint)1717734882U);
                Debug.Assert(pack.time_usec == (ulong)8237667044918170324L);
                Debug.Assert(pack.target_system == (byte)(byte)170);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.target_system = (byte)(byte)170;
            p4.seq = (uint)1717734882U;
            p4.target_component = (byte)(byte)40;
            p4.time_usec = (ulong)8237667044918170324L;
            SMP_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)51);
                Debug.Assert(pack.version == (byte)(byte)152);
                Debug.Assert(pack.passkey_LEN(ph) == 22);
                Debug.Assert(pack.passkey_TRY(ph).Equals("rrejBlBadgtnggcjdrcvbo"));
                Debug.Assert(pack.control_request == (byte)(byte)183);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.target_system = (byte)(byte)51;
            p5.control_request = (byte)(byte)183;
            p5.passkey_SET("rrejBlBadgtnggcjdrcvbo", PH) ;
            p5.version = (byte)(byte)152;
            SMP_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gcs_system_id == (byte)(byte)68);
                Debug.Assert(pack.ack == (byte)(byte)88);
                Debug.Assert(pack.control_request == (byte)(byte)40);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)88;
            p6.gcs_system_id = (byte)(byte)68;
            p6.control_request = (byte)(byte)40;
            SMP_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 12);
                Debug.Assert(pack.key_TRY(ph).Equals("myywmnmfuaCy"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("myywmnmfuaCy", PH) ;
            SMP_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_DISARMED);
                Debug.Assert(pack.custom_mode == (uint)3274216899U);
                Debug.Assert(pack.target_system == (byte)(byte)173);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)173;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_DISARMED;
            p11.custom_mode = (uint)3274216899U;
            SMP_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)212);
                Debug.Assert(pack.param_id_LEN(ph) == 6);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ibyuPy"));
                Debug.Assert(pack.param_index == (short)(short) -24438);
                Debug.Assert(pack.target_component == (byte)(byte)182);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_index = (short)(short) -24438;
            p20.target_component = (byte)(byte)182;
            p20.param_id_SET("ibyuPy", PH) ;
            p20.target_system = (byte)(byte)212;
            SMP_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)34);
                Debug.Assert(pack.target_component == (byte)(byte)150);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)150;
            p21.target_system = (byte)(byte)34;
            SMP_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_count == (ushort)(ushort)49928);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("oThapryjilV"));
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
                Debug.Assert(pack.param_index == (ushort)(ushort)2856);
                Debug.Assert(pack.param_value == (float)1.8932604E37F);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_count = (ushort)(ushort)49928;
            p22.param_value = (float)1.8932604E37F;
            p22.param_id_SET("oThapryjilV", PH) ;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64;
            p22.param_index = (ushort)(ushort)2856;
            SMP_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("dnkym"));
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32);
                Debug.Assert(pack.target_system == (byte)(byte)132);
                Debug.Assert(pack.param_value == (float)2.1287125E38F);
                Debug.Assert(pack.target_component == (byte)(byte)166);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32;
            p23.target_system = (byte)(byte)132;
            p23.param_id_SET("dnkym", PH) ;
            p23.target_component = (byte)(byte)166;
            p23.param_value = (float)2.1287125E38F;
            SMP_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)432806073U);
                Debug.Assert(pack.vel == (ushort)(ushort)55715);
                Debug.Assert(pack.satellites_visible == (byte)(byte)171);
                Debug.Assert(pack.epv == (ushort)(ushort)48934);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)4053709429U);
                Debug.Assert(pack.lat == (int) -924110633);
                Debug.Assert(pack.lon == (int)134305832);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)2021697755U);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -1368791633);
                Debug.Assert(pack.time_usec == (ulong)7566156872380407871L);
                Debug.Assert(pack.cog == (ushort)(ushort)44387);
                Debug.Assert(pack.alt == (int)852505893);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.eph == (ushort)(ushort)17939);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1930845299U);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.alt = (int)852505893;
            p24.lat = (int) -924110633;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p24.time_usec = (ulong)7566156872380407871L;
            p24.vel = (ushort)(ushort)55715;
            p24.vel_acc_SET((uint)2021697755U, PH) ;
            p24.cog = (ushort)(ushort)44387;
            p24.lon = (int)134305832;
            p24.h_acc_SET((uint)4053709429U, PH) ;
            p24.v_acc_SET((uint)1930845299U, PH) ;
            p24.hdg_acc_SET((uint)432806073U, PH) ;
            p24.epv = (ushort)(ushort)48934;
            p24.eph = (ushort)(ushort)17939;
            p24.satellites_visible = (byte)(byte)171;
            p24.alt_ellipsoid_SET((int) -1368791633, PH) ;
            SMP_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)169, (byte)44, (byte)102, (byte)111, (byte)157, (byte)123, (byte)159, (byte)78, (byte)113, (byte)239, (byte)106, (byte)60, (byte)50, (byte)17, (byte)218, (byte)166, (byte)165, (byte)34, (byte)178, (byte)212}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)219, (byte)71, (byte)155, (byte)3, (byte)133, (byte)84, (byte)154, (byte)156, (byte)196, (byte)104, (byte)172, (byte)86, (byte)242, (byte)81, (byte)110, (byte)82, (byte)206, (byte)8, (byte)76, (byte)132}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)57, (byte)186, (byte)153, (byte)217, (byte)165, (byte)188, (byte)184, (byte)170, (byte)192, (byte)69, (byte)154, (byte)89, (byte)206, (byte)24, (byte)187, (byte)146, (byte)73, (byte)48, (byte)120, (byte)205}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)139, (byte)241, (byte)115, (byte)153, (byte)14, (byte)79, (byte)237, (byte)42, (byte)114, (byte)55, (byte)55, (byte)23, (byte)220, (byte)58, (byte)76, (byte)120, (byte)173, (byte)50, (byte)212, (byte)123}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)146, (byte)119, (byte)143, (byte)42, (byte)164, (byte)202, (byte)153, (byte)63, (byte)198, (byte)108, (byte)50, (byte)161, (byte)223, (byte)151, (byte)14, (byte)204, (byte)25, (byte)73, (byte)157, (byte)34}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)55);
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_snr_SET(new byte[] {(byte)219, (byte)71, (byte)155, (byte)3, (byte)133, (byte)84, (byte)154, (byte)156, (byte)196, (byte)104, (byte)172, (byte)86, (byte)242, (byte)81, (byte)110, (byte)82, (byte)206, (byte)8, (byte)76, (byte)132}, 0) ;
            p25.satellites_visible = (byte)(byte)55;
            p25.satellite_azimuth_SET(new byte[] {(byte)57, (byte)186, (byte)153, (byte)217, (byte)165, (byte)188, (byte)184, (byte)170, (byte)192, (byte)69, (byte)154, (byte)89, (byte)206, (byte)24, (byte)187, (byte)146, (byte)73, (byte)48, (byte)120, (byte)205}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)146, (byte)119, (byte)143, (byte)42, (byte)164, (byte)202, (byte)153, (byte)63, (byte)198, (byte)108, (byte)50, (byte)161, (byte)223, (byte)151, (byte)14, (byte)204, (byte)25, (byte)73, (byte)157, (byte)34}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)139, (byte)241, (byte)115, (byte)153, (byte)14, (byte)79, (byte)237, (byte)42, (byte)114, (byte)55, (byte)55, (byte)23, (byte)220, (byte)58, (byte)76, (byte)120, (byte)173, (byte)50, (byte)212, (byte)123}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)169, (byte)44, (byte)102, (byte)111, (byte)157, (byte)123, (byte)159, (byte)78, (byte)113, (byte)239, (byte)106, (byte)60, (byte)50, (byte)17, (byte)218, (byte)166, (byte)165, (byte)34, (byte)178, (byte)212}, 0) ;
            SMP_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short) -10603);
                Debug.Assert(pack.ymag == (short)(short) -5260);
                Debug.Assert(pack.ygyro == (short)(short) -22240);
                Debug.Assert(pack.xmag == (short)(short)32715);
                Debug.Assert(pack.xgyro == (short)(short) -27561);
                Debug.Assert(pack.zmag == (short)(short)15854);
                Debug.Assert(pack.xacc == (short)(short) -4224);
                Debug.Assert(pack.zgyro == (short)(short)19709);
                Debug.Assert(pack.time_boot_ms == (uint)1424136598U);
                Debug.Assert(pack.yacc == (short)(short)13219);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.xmag = (short)(short)32715;
            p26.ymag = (short)(short) -5260;
            p26.yacc = (short)(short)13219;
            p26.time_boot_ms = (uint)1424136598U;
            p26.xgyro = (short)(short) -27561;
            p26.zacc = (short)(short) -10603;
            p26.ygyro = (short)(short) -22240;
            p26.zmag = (short)(short)15854;
            p26.zgyro = (short)(short)19709;
            p26.xacc = (short)(short) -4224;
            SMP_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (short)(short)1354);
                Debug.Assert(pack.xacc == (short)(short)11402);
                Debug.Assert(pack.time_usec == (ulong)7390461501402940684L);
                Debug.Assert(pack.ygyro == (short)(short)26168);
                Debug.Assert(pack.xmag == (short)(short) -23409);
                Debug.Assert(pack.zacc == (short)(short) -17896);
                Debug.Assert(pack.zgyro == (short)(short)30977);
                Debug.Assert(pack.zmag == (short)(short) -14440);
                Debug.Assert(pack.ymag == (short)(short) -7449);
                Debug.Assert(pack.yacc == (short)(short) -27736);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.xmag = (short)(short) -23409;
            p27.zmag = (short)(short) -14440;
            p27.ygyro = (short)(short)26168;
            p27.zgyro = (short)(short)30977;
            p27.yacc = (short)(short) -27736;
            p27.zacc = (short)(short) -17896;
            p27.xacc = (short)(short)11402;
            p27.ymag = (short)(short) -7449;
            p27.xgyro = (short)(short)1354;
            p27.time_usec = (ulong)7390461501402940684L;
            SMP_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff2 == (short)(short)5848);
                Debug.Assert(pack.temperature == (short)(short)5734);
                Debug.Assert(pack.press_diff1 == (short)(short) -20232);
                Debug.Assert(pack.time_usec == (ulong)9158750506751677741L);
                Debug.Assert(pack.press_abs == (short)(short)26133);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff2 = (short)(short)5848;
            p28.temperature = (short)(short)5734;
            p28.press_diff1 = (short)(short) -20232;
            p28.time_usec = (ulong)9158750506751677741L;
            p28.press_abs = (short)(short)26133;
            SMP_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)24165);
                Debug.Assert(pack.time_boot_ms == (uint)3798710454U);
                Debug.Assert(pack.press_diff == (float)1.05634763E37F);
                Debug.Assert(pack.press_abs == (float)1.1224174E38F);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.time_boot_ms = (uint)3798710454U;
            p29.press_diff = (float)1.05634763E37F;
            p29.press_abs = (float)1.1224174E38F;
            p29.temperature = (short)(short)24165;
            SMP_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)1.5703631E38F);
                Debug.Assert(pack.rollspeed == (float) -2.8563448E38F);
                Debug.Assert(pack.pitch == (float)1.7242947E38F);
                Debug.Assert(pack.pitchspeed == (float) -1.1835585E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2536116267U);
                Debug.Assert(pack.yawspeed == (float)2.1830418E38F);
                Debug.Assert(pack.roll == (float) -4.1771732E37F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.rollspeed = (float) -2.8563448E38F;
            p30.pitchspeed = (float) -1.1835585E38F;
            p30.time_boot_ms = (uint)2536116267U;
            p30.roll = (float) -4.1771732E37F;
            p30.yaw = (float)1.5703631E38F;
            p30.yawspeed = (float)2.1830418E38F;
            p30.pitch = (float)1.7242947E38F;
            SMP_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float) -1.2461772E38F);
                Debug.Assert(pack.q1 == (float)1.5538647E38F);
                Debug.Assert(pack.pitchspeed == (float) -2.6162572E38F);
                Debug.Assert(pack.q3 == (float)1.031864E38F);
                Debug.Assert(pack.q2 == (float) -3.031008E38F);
                Debug.Assert(pack.q4 == (float)1.8493205E38F);
                Debug.Assert(pack.rollspeed == (float) -2.6971522E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3358511470U);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.q3 = (float)1.031864E38F;
            p31.q4 = (float)1.8493205E38F;
            p31.q1 = (float)1.5538647E38F;
            p31.q2 = (float) -3.031008E38F;
            p31.rollspeed = (float) -2.6971522E38F;
            p31.time_boot_ms = (uint)3358511470U;
            p31.pitchspeed = (float) -2.6162572E38F;
            p31.yawspeed = (float) -1.2461772E38F;
            SMP_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3130901159U);
                Debug.Assert(pack.y == (float) -1.8150629E38F);
                Debug.Assert(pack.z == (float)3.230126E38F);
                Debug.Assert(pack.vx == (float) -6.6153315E37F);
                Debug.Assert(pack.x == (float) -5.8190786E37F);
                Debug.Assert(pack.vy == (float) -2.262988E38F);
                Debug.Assert(pack.vz == (float) -2.75632E38F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vz = (float) -2.75632E38F;
            p32.z = (float)3.230126E38F;
            p32.vx = (float) -6.6153315E37F;
            p32.vy = (float) -2.262988E38F;
            p32.time_boot_ms = (uint)3130901159U;
            p32.x = (float) -5.8190786E37F;
            p32.y = (float) -1.8150629E38F;
            SMP_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdg == (ushort)(ushort)60754);
                Debug.Assert(pack.vx == (short)(short) -32649);
                Debug.Assert(pack.vy == (short)(short)165);
                Debug.Assert(pack.time_boot_ms == (uint)3569289780U);
                Debug.Assert(pack.relative_alt == (int) -661360118);
                Debug.Assert(pack.lon == (int)457432887);
                Debug.Assert(pack.lat == (int) -382059884);
                Debug.Assert(pack.vz == (short)(short) -28490);
                Debug.Assert(pack.alt == (int)1770623355);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.alt = (int)1770623355;
            p33.vz = (short)(short) -28490;
            p33.vy = (short)(short)165;
            p33.lat = (int) -382059884;
            p33.time_boot_ms = (uint)3569289780U;
            p33.relative_alt = (int) -661360118;
            p33.hdg = (ushort)(ushort)60754;
            p33.lon = (int)457432887;
            p33.vx = (short)(short) -32649;
            SMP_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rssi == (byte)(byte)208);
                Debug.Assert(pack.chan2_scaled == (short)(short) -30439);
                Debug.Assert(pack.chan5_scaled == (short)(short)19691);
                Debug.Assert(pack.chan7_scaled == (short)(short)32280);
                Debug.Assert(pack.chan4_scaled == (short)(short)20598);
                Debug.Assert(pack.chan8_scaled == (short)(short) -7693);
                Debug.Assert(pack.time_boot_ms == (uint)3601678268U);
                Debug.Assert(pack.chan3_scaled == (short)(short) -4229);
                Debug.Assert(pack.port == (byte)(byte)152);
                Debug.Assert(pack.chan6_scaled == (short)(short)9798);
                Debug.Assert(pack.chan1_scaled == (short)(short)8494);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan5_scaled = (short)(short)19691;
            p34.chan7_scaled = (short)(short)32280;
            p34.chan3_scaled = (short)(short) -4229;
            p34.chan2_scaled = (short)(short) -30439;
            p34.chan8_scaled = (short)(short) -7693;
            p34.chan6_scaled = (short)(short)9798;
            p34.chan4_scaled = (short)(short)20598;
            p34.port = (byte)(byte)152;
            p34.chan1_scaled = (short)(short)8494;
            p34.rssi = (byte)(byte)208;
            p34.time_boot_ms = (uint)3601678268U;
            SMP_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)42308);
                Debug.Assert(pack.time_boot_ms == (uint)2523913308U);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)54872);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)19702);
                Debug.Assert(pack.port == (byte)(byte)179);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)12093);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)52163);
                Debug.Assert(pack.rssi == (byte)(byte)6);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)29309);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)23760);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)47695);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan3_raw = (ushort)(ushort)23760;
            p35.chan4_raw = (ushort)(ushort)52163;
            p35.chan1_raw = (ushort)(ushort)42308;
            p35.chan2_raw = (ushort)(ushort)29309;
            p35.chan8_raw = (ushort)(ushort)54872;
            p35.chan6_raw = (ushort)(ushort)47695;
            p35.rssi = (byte)(byte)6;
            p35.chan5_raw = (ushort)(ushort)12093;
            p35.chan7_raw = (ushort)(ushort)19702;
            p35.time_boot_ms = (uint)2523913308U;
            p35.port = (byte)(byte)179;
            SMP_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)60184);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)25634);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)11657);
                Debug.Assert(pack.port == (byte)(byte)51);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)44507);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)14317);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)60424);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)53757);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)27064);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)63348);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)24701);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)39547);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)8650);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)60471);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)12921);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)43122);
                Debug.Assert(pack.time_usec == (uint)2468649573U);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)56893);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo3_raw = (ushort)(ushort)12921;
            p36.servo5_raw = (ushort)(ushort)8650;
            p36.servo9_raw_SET((ushort)(ushort)11657, PH) ;
            p36.servo10_raw_SET((ushort)(ushort)53757, PH) ;
            p36.servo4_raw = (ushort)(ushort)14317;
            p36.servo7_raw = (ushort)(ushort)43122;
            p36.port = (byte)(byte)51;
            p36.servo8_raw = (ushort)(ushort)63348;
            p36.servo1_raw = (ushort)(ushort)56893;
            p36.servo15_raw_SET((ushort)(ushort)60184, PH) ;
            p36.servo2_raw = (ushort)(ushort)39547;
            p36.servo13_raw_SET((ushort)(ushort)60471, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)25634, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)44507, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)60424, PH) ;
            p36.time_usec = (uint)2468649573U;
            p36.servo14_raw_SET((ushort)(ushort)24701, PH) ;
            p36.servo6_raw = (ushort)(ushort)27064;
            SMP_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)94);
                Debug.Assert(pack.start_index == (short)(short)28872);
                Debug.Assert(pack.end_index == (short)(short)11265);
                Debug.Assert(pack.target_system == (byte)(byte)227);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.end_index = (short)(short)11265;
            p37.target_system = (byte)(byte)227;
            p37.start_index = (short)(short)28872;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p37.target_component = (byte)(byte)94;
            SMP_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)39);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.start_index == (short)(short)22394);
                Debug.Assert(pack.end_index == (short)(short) -16113);
                Debug.Assert(pack.target_component == (byte)(byte)119);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.start_index = (short)(short)22394;
            p38.target_component = (byte)(byte)119;
            p38.target_system = (byte)(byte)39;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p38.end_index = (short)(short) -16113;
            SMP_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -1.0044402E38F);
                Debug.Assert(pack.target_system == (byte)(byte)14);
                Debug.Assert(pack.z == (float) -8.8609525E36F);
                Debug.Assert(pack.param2 == (float)2.7731974E38F);
                Debug.Assert(pack.param1 == (float)1.5923007E38F);
                Debug.Assert(pack.param4 == (float) -4.0589447E37F);
                Debug.Assert(pack.autocontinue == (byte)(byte)163);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.seq == (ushort)(ushort)36656);
                Debug.Assert(pack.current == (byte)(byte)90);
                Debug.Assert(pack.param3 == (float) -4.9766244E37F);
                Debug.Assert(pack.target_component == (byte)(byte)233);
                Debug.Assert(pack.y == (float) -2.6672054E38F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_WAYPOINT_USER_1);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.param4 = (float) -4.0589447E37F;
            p39.param2 = (float)2.7731974E38F;
            p39.current = (byte)(byte)90;
            p39.param3 = (float) -4.9766244E37F;
            p39.y = (float) -2.6672054E38F;
            p39.param1 = (float)1.5923007E38F;
            p39.x = (float) -1.0044402E38F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_WAYPOINT_USER_1;
            p39.z = (float) -8.8609525E36F;
            p39.seq = (ushort)(ushort)36656;
            p39.target_component = (byte)(byte)233;
            p39.autocontinue = (byte)(byte)163;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED;
            p39.target_system = (byte)(byte)14;
            SMP_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)82);
                Debug.Assert(pack.seq == (ushort)(ushort)13527);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)64);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p40.target_system = (byte)(byte)82;
            p40.seq = (ushort)(ushort)13527;
            p40.target_component = (byte)(byte)64;
            SMP_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)49101);
                Debug.Assert(pack.target_component == (byte)(byte)159);
                Debug.Assert(pack.target_system == (byte)(byte)182);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_component = (byte)(byte)159;
            p41.target_system = (byte)(byte)182;
            p41.seq = (ushort)(ushort)49101;
            SMP_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)34970);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)34970;
            SMP_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)3);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)18);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p43.target_component = (byte)(byte)18;
            p43.target_system = (byte)(byte)3;
            SMP_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.count == (ushort)(ushort)51833);
                Debug.Assert(pack.target_component == (byte)(byte)29);
                Debug.Assert(pack.target_system == (byte)(byte)235);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p44.target_component = (byte)(byte)29;
            p44.target_system = (byte)(byte)235;
            p44.count = (ushort)(ushort)51833;
            SMP_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)187);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)52);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_component = (byte)(byte)187;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p45.target_system = (byte)(byte)52;
            SMP_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)38856);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)38856;
            SMP_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)149);
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)9);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)149;
            p47.target_component = (byte)(byte)9;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE;
            SMP_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)112);
                Debug.Assert(pack.longitude == (int)1634502976);
                Debug.Assert(pack.altitude == (int)1603589813);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7118485001758988402L);
                Debug.Assert(pack.latitude == (int) -520189795);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.longitude = (int)1634502976;
            p48.time_usec_SET((ulong)7118485001758988402L, PH) ;
            p48.altitude = (int)1603589813;
            p48.target_system = (byte)(byte)112;
            p48.latitude = (int) -520189795;
            SMP_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int)749231894);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7089363121864847289L);
                Debug.Assert(pack.altitude == (int) -86717269);
                Debug.Assert(pack.latitude == (int)535744281);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.longitude = (int)749231894;
            p49.time_usec_SET((ulong)7089363121864847289L, PH) ;
            p49.altitude = (int) -86717269;
            p49.latitude = (int)535744281;
            SMP_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value0 == (float) -1.517178E38F);
                Debug.Assert(pack.param_value_max == (float)8.671901E37F);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("neqezgqrogos"));
                Debug.Assert(pack.param_value_min == (float)1.7243433E38F);
                Debug.Assert(pack.target_system == (byte)(byte)213);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)92);
                Debug.Assert(pack.scale == (float) -1.9844512E37F);
                Debug.Assert(pack.target_component == (byte)(byte)53);
                Debug.Assert(pack.param_index == (short)(short) -21053);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_value_max = (float)8.671901E37F;
            p50.target_component = (byte)(byte)53;
            p50.param_id_SET("neqezgqrogos", PH) ;
            p50.param_value_min = (float)1.7243433E38F;
            p50.target_system = (byte)(byte)213;
            p50.scale = (float) -1.9844512E37F;
            p50.parameter_rc_channel_index = (byte)(byte)92;
            p50.param_value0 = (float) -1.517178E38F;
            p50.param_index = (short)(short) -21053;
            SMP_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)36824);
                Debug.Assert(pack.target_system == (byte)(byte)230);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)203);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p51.target_system = (byte)(byte)230;
            p51.target_component = (byte)(byte)203;
            p51.seq = (ushort)(ushort)36824;
            SMP_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.target_system == (byte)(byte)126);
                Debug.Assert(pack.target_component == (byte)(byte)169);
                Debug.Assert(pack.p1z == (float)3.270512E38F);
                Debug.Assert(pack.p2x == (float)7.8131016E37F);
                Debug.Assert(pack.p1y == (float) -3.0572666E38F);
                Debug.Assert(pack.p2z == (float) -2.4776835E38F);
                Debug.Assert(pack.p2y == (float) -1.6505822E38F);
                Debug.Assert(pack.p1x == (float) -1.942954E38F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p2z = (float) -2.4776835E38F;
            p54.p2y = (float) -1.6505822E38F;
            p54.target_system = (byte)(byte)126;
            p54.p2x = (float)7.8131016E37F;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p54.p1x = (float) -1.942954E38F;
            p54.p1z = (float)3.270512E38F;
            p54.target_component = (byte)(byte)169;
            p54.p1y = (float) -3.0572666E38F;
            SMP_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1y == (float) -1.3962375E38F);
                Debug.Assert(pack.p2z == (float)2.697057E37F);
                Debug.Assert(pack.p2y == (float) -3.0258266E38F);
                Debug.Assert(pack.p2x == (float) -1.4690762E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.p1x == (float)2.2507353E38F);
                Debug.Assert(pack.p1z == (float) -1.7497497E36F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p1x = (float)2.2507353E38F;
            p55.p1z = (float) -1.7497497E36F;
            p55.p2x = (float) -1.4690762E38F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p55.p2y = (float) -3.0258266E38F;
            p55.p2z = (float)2.697057E37F;
            p55.p1y = (float) -1.3962375E38F;
            SMP_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float) -1.5401364E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.7000648E37F, 1.6817553E38F, 1.7967759E38F, -3.2609442E38F, -3.2966004E38F, -2.996502E38F, 1.683175E38F, 2.8172024E38F, -3.0361118E38F}));
                Debug.Assert(pack.q.SequenceEqual(new float[] {-5.305288E36F, 2.2996902E38F, 7.729961E37F, -2.037458E38F}));
                Debug.Assert(pack.yawspeed == (float) -3.3106525E38F);
                Debug.Assert(pack.time_usec == (ulong)4692287599862233529L);
                Debug.Assert(pack.pitchspeed == (float)8.022187E37F);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.pitchspeed = (float)8.022187E37F;
            p61.time_usec = (ulong)4692287599862233529L;
            p61.yawspeed = (float) -3.3106525E38F;
            p61.rollspeed = (float) -1.5401364E38F;
            p61.q_SET(new float[] {-5.305288E36F, 2.2996902E38F, 7.729961E37F, -2.037458E38F}, 0) ;
            p61.covariance_SET(new float[] {-2.7000648E37F, 1.6817553E38F, 1.7967759E38F, -3.2609442E38F, -3.2966004E38F, -2.996502E38F, 1.683175E38F, 2.8172024E38F, -3.0361118E38F}, 0) ;
            SMP_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wp_dist == (ushort)(ushort)48845);
                Debug.Assert(pack.nav_roll == (float) -2.8203914E38F);
                Debug.Assert(pack.alt_error == (float) -2.5739723E38F);
                Debug.Assert(pack.nav_pitch == (float) -3.2353743E38F);
                Debug.Assert(pack.xtrack_error == (float)2.1930122E38F);
                Debug.Assert(pack.target_bearing == (short)(short)845);
                Debug.Assert(pack.aspd_error == (float) -2.9006902E38F);
                Debug.Assert(pack.nav_bearing == (short)(short)9155);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.target_bearing = (short)(short)845;
            p62.alt_error = (float) -2.5739723E38F;
            p62.nav_pitch = (float) -3.2353743E38F;
            p62.aspd_error = (float) -2.9006902E38F;
            p62.nav_bearing = (short)(short)9155;
            p62.nav_roll = (float) -2.8203914E38F;
            p62.wp_dist = (ushort)(ushort)48845;
            p62.xtrack_error = (float)2.1930122E38F;
            SMP_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float)1.3513048E38F);
                Debug.Assert(pack.lon == (int) -860746575);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.3615022E38F, 8.532821E37F, 1.7452817E38F, 3.1297504E38F, 8.71199E37F, -1.9060526E38F, -1.6110907E38F, 8.3880795E37F, 5.1891594E36F, -1.3652202E38F, -1.7067545E36F, -6.9352287E37F, -1.8859033E38F, 3.3565828E38F, 3.0744421E38F, 9.129561E37F, 8.2735103E37F, 8.768192E37F, 2.0608672E38F, -7.484914E36F, -1.4711201E38F, -4.6682237E37F, -2.3432767E38F, 2.713301E38F, 5.821625E37F, 8.690762E37F, 2.9197737E38F, 2.1100026E38F, 2.0063749E37F, -9.127824E37F, -2.2163579E38F, -2.4432554E38F, 2.3655577E38F, -2.5858452E38F, -2.0819433E38F, 1.6170584E38F}));
                Debug.Assert(pack.time_usec == (ulong)9153036803755924548L);
                Debug.Assert(pack.relative_alt == (int)385290421);
                Debug.Assert(pack.lat == (int)1188646796);
                Debug.Assert(pack.vy == (float)2.065746E38F);
                Debug.Assert(pack.vx == (float)2.1857085E38F);
                Debug.Assert(pack.alt == (int) -480653039);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.lon = (int) -860746575;
            p63.time_usec = (ulong)9153036803755924548L;
            p63.lat = (int)1188646796;
            p63.covariance_SET(new float[] {2.3615022E38F, 8.532821E37F, 1.7452817E38F, 3.1297504E38F, 8.71199E37F, -1.9060526E38F, -1.6110907E38F, 8.3880795E37F, 5.1891594E36F, -1.3652202E38F, -1.7067545E36F, -6.9352287E37F, -1.8859033E38F, 3.3565828E38F, 3.0744421E38F, 9.129561E37F, 8.2735103E37F, 8.768192E37F, 2.0608672E38F, -7.484914E36F, -1.4711201E38F, -4.6682237E37F, -2.3432767E38F, 2.713301E38F, 5.821625E37F, 8.690762E37F, 2.9197737E38F, 2.1100026E38F, 2.0063749E37F, -9.127824E37F, -2.2163579E38F, -2.4432554E38F, 2.3655577E38F, -2.5858452E38F, -2.0819433E38F, 1.6170584E38F}, 0) ;
            p63.vx = (float)2.1857085E38F;
            p63.vz = (float)1.3513048E38F;
            p63.alt = (int) -480653039;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            p63.relative_alt = (int)385290421;
            p63.vy = (float)2.065746E38F;
            SMP_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -1.5747221E38F);
                Debug.Assert(pack.az == (float) -1.1216612E38F);
                Debug.Assert(pack.z == (float) -2.1506867E38F);
                Debug.Assert(pack.vz == (float) -9.037155E37F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.x == (float)8.719358E36F);
                Debug.Assert(pack.time_usec == (ulong)8267632700706364536L);
                Debug.Assert(pack.ay == (float)3.3165263E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.6551729E38F, 3.3275072E38F, -8.3947195E36F, 2.8620969E38F, -2.4982248E38F, 2.7398862E38F, 7.9627777E37F, 2.1695586E38F, -1.1293874E38F, -2.5006651E38F, 2.0686046E38F, 9.532509E37F, 1.7465848E38F, -1.1284679E38F, 1.2908724E38F, 2.930728E38F, -3.038929E36F, -2.690706E37F, -2.3506228E37F, -1.1862605E38F, 3.2718948E38F, 1.3746474E38F, -1.9122386E38F, 1.1822536E38F, 3.0338692E38F, 1.8864586E38F, -1.6086107E38F, 1.0707524E38F, 7.5540917E37F, -2.3677117E38F, 5.921314E37F, -9.60375E37F, 1.9328262E38F, -1.6654979E38F, 3.4027326E38F, -3.206147E38F, -2.3206827E38F, -1.9742756E38F, -2.8367108E38F, 2.6350357E38F, 2.5224588E38F, -2.2406442E38F, 7.954706E37F, 1.2244403E38F, 3.252246E38F}));
                Debug.Assert(pack.vx == (float)2.1469456E38F);
                Debug.Assert(pack.ax == (float)9.88568E37F);
                Debug.Assert(pack.vy == (float) -3.299797E38F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.y = (float) -1.5747221E38F;
            p64.az = (float) -1.1216612E38F;
            p64.x = (float)8.719358E36F;
            p64.ay = (float)3.3165263E38F;
            p64.vx = (float)2.1469456E38F;
            p64.ax = (float)9.88568E37F;
            p64.vy = (float) -3.299797E38F;
            p64.vz = (float) -9.037155E37F;
            p64.time_usec = (ulong)8267632700706364536L;
            p64.covariance_SET(new float[] {-2.6551729E38F, 3.3275072E38F, -8.3947195E36F, 2.8620969E38F, -2.4982248E38F, 2.7398862E38F, 7.9627777E37F, 2.1695586E38F, -1.1293874E38F, -2.5006651E38F, 2.0686046E38F, 9.532509E37F, 1.7465848E38F, -1.1284679E38F, 1.2908724E38F, 2.930728E38F, -3.038929E36F, -2.690706E37F, -2.3506228E37F, -1.1862605E38F, 3.2718948E38F, 1.3746474E38F, -1.9122386E38F, 1.1822536E38F, 3.0338692E38F, 1.8864586E38F, -1.6086107E38F, 1.0707524E38F, 7.5540917E37F, -2.3677117E38F, 5.921314E37F, -9.60375E37F, 1.9328262E38F, -1.6654979E38F, 3.4027326E38F, -3.206147E38F, -2.3206827E38F, -1.9742756E38F, -2.8367108E38F, 2.6350357E38F, 2.5224588E38F, -2.2406442E38F, 7.954706E37F, 1.2244403E38F, 3.252246E38F}, 0) ;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p64.z = (float) -2.1506867E38F;
            SMP_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2095690017U);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)27433);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)25773);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)13595);
                Debug.Assert(pack.rssi == (byte)(byte)239);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)15167);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)36618);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)61391);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)47162);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)30137);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)55051);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)26855);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)39090);
                Debug.Assert(pack.chancount == (byte)(byte)209);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)9878);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)42386);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)58637);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)52558);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)50500);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)27770);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)52562);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan5_raw = (ushort)(ushort)9878;
            p65.chan12_raw = (ushort)(ushort)52558;
            p65.chan6_raw = (ushort)(ushort)42386;
            p65.chan4_raw = (ushort)(ushort)55051;
            p65.chan17_raw = (ushort)(ushort)30137;
            p65.chan2_raw = (ushort)(ushort)58637;
            p65.chan10_raw = (ushort)(ushort)47162;
            p65.chan18_raw = (ushort)(ushort)52562;
            p65.rssi = (byte)(byte)239;
            p65.chan14_raw = (ushort)(ushort)15167;
            p65.chan8_raw = (ushort)(ushort)26855;
            p65.chan9_raw = (ushort)(ushort)36618;
            p65.chan7_raw = (ushort)(ushort)61391;
            p65.chan11_raw = (ushort)(ushort)50500;
            p65.time_boot_ms = (uint)2095690017U;
            p65.chan13_raw = (ushort)(ushort)27433;
            p65.chan16_raw = (ushort)(ushort)39090;
            p65.chan1_raw = (ushort)(ushort)13595;
            p65.chan3_raw = (ushort)(ushort)25773;
            p65.chancount = (byte)(byte)209;
            p65.chan15_raw = (ushort)(ushort)27770;
            SMP_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)7152);
                Debug.Assert(pack.target_system == (byte)(byte)104);
                Debug.Assert(pack.req_stream_id == (byte)(byte)86);
                Debug.Assert(pack.target_component == (byte)(byte)37);
                Debug.Assert(pack.start_stop == (byte)(byte)245);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.req_stream_id = (byte)(byte)86;
            p66.req_message_rate = (ushort)(ushort)7152;
            p66.start_stop = (byte)(byte)245;
            p66.target_system = (byte)(byte)104;
            p66.target_component = (byte)(byte)37;
            SMP_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)23447);
                Debug.Assert(pack.stream_id == (byte)(byte)61);
                Debug.Assert(pack.on_off == (byte)(byte)154);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.on_off = (byte)(byte)154;
            p67.stream_id = (byte)(byte)61;
            p67.message_rate = (ushort)(ushort)23447;
            SMP_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.buttons == (ushort)(ushort)38238);
                Debug.Assert(pack.r == (short)(short) -11234);
                Debug.Assert(pack.y == (short)(short)16069);
                Debug.Assert(pack.target == (byte)(byte)191);
                Debug.Assert(pack.x == (short)(short) -12270);
                Debug.Assert(pack.z == (short)(short)12218);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.x = (short)(short) -12270;
            p69.target = (byte)(byte)191;
            p69.z = (short)(short)12218;
            p69.buttons = (ushort)(ushort)38238;
            p69.r = (short)(short) -11234;
            p69.y = (short)(short)16069;
            SMP_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)20937);
                Debug.Assert(pack.target_system == (byte)(byte)65);
                Debug.Assert(pack.target_component == (byte)(byte)117);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)280);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)34999);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)35455);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)41080);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)51607);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)47122);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)44749);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan7_raw = (ushort)(ushort)47122;
            p70.chan1_raw = (ushort)(ushort)35455;
            p70.chan4_raw = (ushort)(ushort)280;
            p70.chan2_raw = (ushort)(ushort)34999;
            p70.target_component = (byte)(byte)117;
            p70.chan8_raw = (ushort)(ushort)41080;
            p70.chan3_raw = (ushort)(ushort)51607;
            p70.chan5_raw = (ushort)(ushort)20937;
            p70.chan6_raw = (ushort)(ushort)44749;
            p70.target_system = (byte)(byte)65;
            SMP_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param4 == (float) -2.6743542E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_OVERRIDE_GOTO);
                Debug.Assert(pack.autocontinue == (byte)(byte)236);
                Debug.Assert(pack.param1 == (float) -2.3034706E38F);
                Debug.Assert(pack.param2 == (float)1.5027846E38F);
                Debug.Assert(pack.z == (float)9.891832E37F);
                Debug.Assert(pack.param3 == (float)1.6789967E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.x == (int) -139457150);
                Debug.Assert(pack.target_system == (byte)(byte)45);
                Debug.Assert(pack.seq == (ushort)(ushort)27676);
                Debug.Assert(pack.y == (int)296126476);
                Debug.Assert(pack.current == (byte)(byte)160);
                Debug.Assert(pack.target_component == (byte)(byte)204);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.target_system = (byte)(byte)45;
            p73.param2 = (float)1.5027846E38F;
            p73.y = (int)296126476;
            p73.seq = (ushort)(ushort)27676;
            p73.param1 = (float) -2.3034706E38F;
            p73.x = (int) -139457150;
            p73.current = (byte)(byte)160;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p73.target_component = (byte)(byte)204;
            p73.z = (float)9.891832E37F;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_OVERRIDE_GOTO;
            p73.param4 = (float) -2.6743542E38F;
            p73.param3 = (float)1.6789967E38F;
            p73.autocontinue = (byte)(byte)236;
            SMP_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.groundspeed == (float) -1.5688201E38F);
                Debug.Assert(pack.heading == (short)(short)22445);
                Debug.Assert(pack.throttle == (ushort)(ushort)19183);
                Debug.Assert(pack.climb == (float)3.3235948E37F);
                Debug.Assert(pack.airspeed == (float) -2.6387226E38F);
                Debug.Assert(pack.alt == (float) -1.0976571E38F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.throttle = (ushort)(ushort)19183;
            p74.climb = (float)3.3235948E37F;
            p74.airspeed = (float) -2.6387226E38F;
            p74.heading = (short)(short)22445;
            p74.alt = (float) -1.0976571E38F;
            p74.groundspeed = (float) -1.5688201E38F;
            SMP_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)144);
                Debug.Assert(pack.z == (float)2.59007E37F);
                Debug.Assert(pack.param4 == (float) -2.0897443E38F);
                Debug.Assert(pack.target_component == (byte)(byte)127);
                Debug.Assert(pack.param3 == (float) -1.9714855E38F);
                Debug.Assert(pack.x == (int)162101898);
                Debug.Assert(pack.autocontinue == (byte)(byte)86);
                Debug.Assert(pack.param2 == (float) -2.4025508E37F);
                Debug.Assert(pack.y == (int) -710063533);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_USER_3);
                Debug.Assert(pack.current == (byte)(byte)38);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.param1 == (float) -2.7867284E38F);
            };
            GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.param4 = (float) -2.0897443E38F;
            p75.param3 = (float) -1.9714855E38F;
            p75.autocontinue = (byte)(byte)86;
            p75.target_component = (byte)(byte)127;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p75.z = (float)2.59007E37F;
            p75.y = (int) -710063533;
            p75.current = (byte)(byte)38;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_USER_3;
            p75.x = (int)162101898;
            p75.param2 = (float) -2.4025508E37F;
            p75.param1 = (float) -2.7867284E38F;
            p75.target_system = (byte)(byte)144;
            CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION);
                Debug.Assert(pack.param4 == (float) -1.6924268E38F);
                Debug.Assert(pack.param2 == (float) -9.264809E37F);
                Debug.Assert(pack.param3 == (float) -1.1059666E38F);
                Debug.Assert(pack.param7 == (float) -2.7516387E38F);
                Debug.Assert(pack.target_component == (byte)(byte)129);
                Debug.Assert(pack.target_system == (byte)(byte)35);
                Debug.Assert(pack.param1 == (float)2.9371466E38F);
                Debug.Assert(pack.param6 == (float) -6.6981796E36F);
                Debug.Assert(pack.confirmation == (byte)(byte)204);
                Debug.Assert(pack.param5 == (float)6.9886947E37F);
            };
            GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.param3 = (float) -1.1059666E38F;
            p76.param7 = (float) -2.7516387E38F;
            p76.confirmation = (byte)(byte)204;
            p76.param4 = (float) -1.6924268E38F;
            p76.target_component = (byte)(byte)129;
            p76.param1 = (float)2.9371466E38F;
            p76.param6 = (float) -6.6981796E36F;
            p76.param5 = (float)6.9886947E37F;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION;
            p76.param2 = (float) -9.264809E37F;
            p76.target_system = (byte)(byte)35;
            CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_SERVO);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)6);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)240);
                Debug.Assert(pack.result_param2_TRY(ph) == (int)638381135);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)191);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_UNSUPPORTED);
            };
            GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.target_component_SET((byte)(byte)6, PH) ;
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_SERVO;
            p77.result_param2_SET((int)638381135, PH) ;
            p77.progress_SET((byte)(byte)240, PH) ;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_UNSUPPORTED;
            p77.target_system_SET((byte)(byte)191, PH) ;
            CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -1.3498397E38F);
                Debug.Assert(pack.roll == (float)5.1685953E37F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)126);
                Debug.Assert(pack.yaw == (float)2.5313565E38F);
                Debug.Assert(pack.thrust == (float)2.1175377E38F);
                Debug.Assert(pack.time_boot_ms == (uint)523586115U);
                Debug.Assert(pack.mode_switch == (byte)(byte)47);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.mode_switch = (byte)(byte)47;
            p81.time_boot_ms = (uint)523586115U;
            p81.yaw = (float)2.5313565E38F;
            p81.thrust = (float)2.1175377E38F;
            p81.roll = (float)5.1685953E37F;
            p81.manual_override_switch = (byte)(byte)126;
            p81.pitch = (float) -1.3498397E38F;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_yaw_rate == (float)1.6729735E38F);
                Debug.Assert(pack.target_system == (byte)(byte)246);
                Debug.Assert(pack.body_roll_rate == (float) -9.805023E37F);
                Debug.Assert(pack.type_mask == (byte)(byte)246);
                Debug.Assert(pack.target_component == (byte)(byte)29);
                Debug.Assert(pack.time_boot_ms == (uint)976792614U);
                Debug.Assert(pack.body_pitch_rate == (float) -1.7193295E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.319854E38F, -5.445797E37F, 3.0632903E38F, 3.2690415E38F}));
                Debug.Assert(pack.thrust == (float) -2.6788516E38F);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_roll_rate = (float) -9.805023E37F;
            p82.type_mask = (byte)(byte)246;
            p82.thrust = (float) -2.6788516E38F;
            p82.q_SET(new float[] {-2.319854E38F, -5.445797E37F, 3.0632903E38F, 3.2690415E38F}, 0) ;
            p82.time_boot_ms = (uint)976792614U;
            p82.body_yaw_rate = (float)1.6729735E38F;
            p82.target_component = (byte)(byte)29;
            p82.target_system = (byte)(byte)246;
            p82.body_pitch_rate = (float) -1.7193295E38F;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.thrust == (float)1.4706347E38F);
                Debug.Assert(pack.body_roll_rate == (float)2.205331E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.5227969E38F, 1.7834769E38F, 6.8110775E37F, -5.630745E37F}));
                Debug.Assert(pack.body_pitch_rate == (float) -2.5229186E38F);
                Debug.Assert(pack.body_yaw_rate == (float) -2.2653178E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)147);
                Debug.Assert(pack.time_boot_ms == (uint)3587762338U);
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.q_SET(new float[] {-2.5227969E38F, 1.7834769E38F, 6.8110775E37F, -5.630745E37F}, 0) ;
            p83.time_boot_ms = (uint)3587762338U;
            p83.body_roll_rate = (float)2.205331E38F;
            p83.type_mask = (byte)(byte)147;
            p83.body_pitch_rate = (float) -2.5229186E38F;
            p83.thrust = (float)1.4706347E38F;
            p83.body_yaw_rate = (float) -2.2653178E38F;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)2.5471386E38F);
                Debug.Assert(pack.yaw == (float)3.400164E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.vy == (float)1.1402646E38F);
                Debug.Assert(pack.y == (float) -2.333313E38F);
                Debug.Assert(pack.yaw_rate == (float) -1.1182485E38F);
                Debug.Assert(pack.z == (float)2.5101683E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3794264591U);
                Debug.Assert(pack.afx == (float)2.8123638E38F);
                Debug.Assert(pack.vx == (float)1.527584E38F);
                Debug.Assert(pack.target_component == (byte)(byte)217);
                Debug.Assert(pack.afy == (float) -1.8616518E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)9092);
                Debug.Assert(pack.vz == (float) -1.5327076E38F);
                Debug.Assert(pack.afz == (float)3.0233682E38F);
                Debug.Assert(pack.target_system == (byte)(byte)189);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.afz = (float)3.0233682E38F;
            p84.y = (float) -2.333313E38F;
            p84.x = (float)2.5471386E38F;
            p84.vx = (float)1.527584E38F;
            p84.yaw = (float)3.400164E38F;
            p84.vz = (float) -1.5327076E38F;
            p84.afx = (float)2.8123638E38F;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p84.target_component = (byte)(byte)217;
            p84.yaw_rate = (float) -1.1182485E38F;
            p84.vy = (float)1.1402646E38F;
            p84.afy = (float) -1.8616518E38F;
            p84.target_system = (byte)(byte)189;
            p84.type_mask = (ushort)(ushort)9092;
            p84.z = (float)2.5101683E38F;
            p84.time_boot_ms = (uint)3794264591U;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float)1.2792359E38F);
                Debug.Assert(pack.yaw_rate == (float)2.4696797E38F);
                Debug.Assert(pack.yaw == (float)5.4575908E35F);
                Debug.Assert(pack.target_system == (byte)(byte)248);
                Debug.Assert(pack.afx == (float)9.071123E37F);
                Debug.Assert(pack.time_boot_ms == (uint)4176364202U);
                Debug.Assert(pack.lat_int == (int)580664459);
                Debug.Assert(pack.type_mask == (ushort)(ushort)7579);
                Debug.Assert(pack.target_component == (byte)(byte)201);
                Debug.Assert(pack.afy == (float) -2.6342828E38F);
                Debug.Assert(pack.vx == (float)2.0652884E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.afz == (float)1.1934582E38F);
                Debug.Assert(pack.lon_int == (int)1664117557);
                Debug.Assert(pack.alt == (float) -2.3574285E38F);
                Debug.Assert(pack.vz == (float) -1.302275E35F);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.yaw_rate = (float)2.4696797E38F;
            p86.lat_int = (int)580664459;
            p86.yaw = (float)5.4575908E35F;
            p86.type_mask = (ushort)(ushort)7579;
            p86.vy = (float)1.2792359E38F;
            p86.afx = (float)9.071123E37F;
            p86.vz = (float) -1.302275E35F;
            p86.afz = (float)1.1934582E38F;
            p86.target_component = (byte)(byte)201;
            p86.lon_int = (int)1664117557;
            p86.afy = (float) -2.6342828E38F;
            p86.time_boot_ms = (uint)4176364202U;
            p86.target_system = (byte)(byte)248;
            p86.vx = (float)2.0652884E38F;
            p86.alt = (float) -2.3574285E38F;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)8.730518E37F);
                Debug.Assert(pack.vy == (float)1.4395419E38F);
                Debug.Assert(pack.alt == (float)2.01874E38F);
                Debug.Assert(pack.afz == (float)3.071904E38F);
                Debug.Assert(pack.afy == (float) -2.9550842E38F);
                Debug.Assert(pack.vx == (float) -3.8623648E37F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.afx == (float) -3.1928997E38F);
                Debug.Assert(pack.vz == (float)3.1127956E36F);
                Debug.Assert(pack.lat_int == (int)1710326111);
                Debug.Assert(pack.time_boot_ms == (uint)2446929031U);
                Debug.Assert(pack.type_mask == (ushort)(ushort)34545);
                Debug.Assert(pack.lon_int == (int) -1644368145);
                Debug.Assert(pack.yaw_rate == (float)8.485803E37F);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.vz = (float)3.1127956E36F;
            p87.lat_int = (int)1710326111;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p87.yaw_rate = (float)8.485803E37F;
            p87.time_boot_ms = (uint)2446929031U;
            p87.lon_int = (int) -1644368145;
            p87.afy = (float) -2.9550842E38F;
            p87.afz = (float)3.071904E38F;
            p87.vy = (float)1.4395419E38F;
            p87.yaw = (float)8.730518E37F;
            p87.vx = (float) -3.8623648E37F;
            p87.type_mask = (ushort)(ushort)34545;
            p87.afx = (float) -3.1928997E38F;
            p87.alt = (float)2.01874E38F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -3.2816772E38F);
                Debug.Assert(pack.z == (float) -1.8563845E38F);
                Debug.Assert(pack.yaw == (float)1.9799761E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3230487084U);
                Debug.Assert(pack.roll == (float) -3.1742095E38F);
                Debug.Assert(pack.x == (float) -2.761666E38F);
                Debug.Assert(pack.pitch == (float)5.4791094E37F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.pitch = (float)5.4791094E37F;
            p89.x = (float) -2.761666E38F;
            p89.z = (float) -1.8563845E38F;
            p89.time_boot_ms = (uint)3230487084U;
            p89.yaw = (float)1.9799761E38F;
            p89.roll = (float) -3.1742095E38F;
            p89.y = (float) -3.2816772E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)453290663);
                Debug.Assert(pack.xacc == (short)(short) -2995);
                Debug.Assert(pack.vz == (short)(short)11864);
                Debug.Assert(pack.vy == (short)(short) -18766);
                Debug.Assert(pack.pitchspeed == (float)2.6946126E38F);
                Debug.Assert(pack.lat == (int) -584460467);
                Debug.Assert(pack.yawspeed == (float)3.7503012E37F);
                Debug.Assert(pack.roll == (float)4.585296E37F);
                Debug.Assert(pack.rollspeed == (float)9.161845E37F);
                Debug.Assert(pack.yaw == (float) -2.3898676E38F);
                Debug.Assert(pack.zacc == (short)(short)16350);
                Debug.Assert(pack.pitch == (float) -3.315018E38F);
                Debug.Assert(pack.vx == (short)(short) -26460);
                Debug.Assert(pack.alt == (int) -177964630);
                Debug.Assert(pack.time_usec == (ulong)5236217077585900670L);
                Debug.Assert(pack.yacc == (short)(short) -26675);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.rollspeed = (float)9.161845E37F;
            p90.lon = (int)453290663;
            p90.vy = (short)(short) -18766;
            p90.roll = (float)4.585296E37F;
            p90.xacc = (short)(short) -2995;
            p90.time_usec = (ulong)5236217077585900670L;
            p90.zacc = (short)(short)16350;
            p90.pitchspeed = (float)2.6946126E38F;
            p90.pitch = (float) -3.315018E38F;
            p90.yaw = (float) -2.3898676E38F;
            p90.yacc = (short)(short) -26675;
            p90.vz = (short)(short)11864;
            p90.vx = (short)(short) -26460;
            p90.lat = (int) -584460467;
            p90.alt = (int) -177964630;
            p90.yawspeed = (float)3.7503012E37F;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aux3 == (float) -8.5477895E36F);
                Debug.Assert(pack.yaw_rudder == (float)2.8745412E37F);
                Debug.Assert(pack.aux2 == (float)1.9284477E37F);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
                Debug.Assert(pack.aux1 == (float)1.5076261E38F);
                Debug.Assert(pack.pitch_elevator == (float) -1.4716645E38F);
                Debug.Assert(pack.roll_ailerons == (float) -2.0655728E38F);
                Debug.Assert(pack.time_usec == (ulong)8537799169281618921L);
                Debug.Assert(pack.aux4 == (float)2.6899285E37F);
                Debug.Assert(pack.throttle == (float) -3.1375969E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)134);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.throttle = (float) -3.1375969E38F;
            p91.aux2 = (float)1.9284477E37F;
            p91.pitch_elevator = (float) -1.4716645E38F;
            p91.time_usec = (ulong)8537799169281618921L;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            p91.nav_mode = (byte)(byte)134;
            p91.aux4 = (float)2.6899285E37F;
            p91.roll_ailerons = (float) -2.0655728E38F;
            p91.aux1 = (float)1.5076261E38F;
            p91.aux3 = (float) -8.5477895E36F;
            p91.yaw_rudder = (float)2.8745412E37F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)11127);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)9851);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)53899);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)55336);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)14021);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)41472);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)21412);
                Debug.Assert(pack.time_usec == (ulong)2239075000801730708L);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)40139);
                Debug.Assert(pack.rssi == (byte)(byte)254);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)24712);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)11897);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)57168);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)38408);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.rssi = (byte)(byte)254;
            p92.chan3_raw = (ushort)(ushort)11127;
            p92.chan9_raw = (ushort)(ushort)40139;
            p92.chan10_raw = (ushort)(ushort)21412;
            p92.time_usec = (ulong)2239075000801730708L;
            p92.chan6_raw = (ushort)(ushort)24712;
            p92.chan12_raw = (ushort)(ushort)41472;
            p92.chan7_raw = (ushort)(ushort)9851;
            p92.chan11_raw = (ushort)(ushort)38408;
            p92.chan1_raw = (ushort)(ushort)14021;
            p92.chan5_raw = (ushort)(ushort)57168;
            p92.chan4_raw = (ushort)(ushort)11897;
            p92.chan8_raw = (ushort)(ushort)53899;
            p92.chan2_raw = (ushort)(ushort)55336;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_TEST_DISARMED);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {6.140985E37F, 1.1698603E38F, -3.1189578E38F, 1.480477E38F, 2.1712273E38F, 2.8497075E37F, 2.0142708E38F, -2.4377884E38F, -1.3697958E38F, 1.2385243E37F, -1.3606425E38F, -1.6510413E38F, 2.467362E38F, -2.1341293E38F, -3.1003476E38F, -1.2785537E37F}));
                Debug.Assert(pack.time_usec == (ulong)4004593543789240172L);
                Debug.Assert(pack.flags == (ulong)6005423893857222103L);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.flags = (ulong)6005423893857222103L;
            p93.controls_SET(new float[] {6.140985E37F, 1.1698603E38F, -3.1189578E38F, 1.480477E38F, 2.1712273E38F, 2.8497075E37F, 2.0142708E38F, -2.4377884E38F, -1.3697958E38F, 1.2385243E37F, -1.3606425E38F, -1.6510413E38F, 2.467362E38F, -2.1341293E38F, -3.1003476E38F, -1.2785537E37F}, 0) ;
            p93.time_usec = (ulong)4004593543789240172L;
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_TEST_DISARMED;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)6.5145705E36F);
                Debug.Assert(pack.flow_comp_m_y == (float) -1.4216518E38F);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)2.8297322E38F);
                Debug.Assert(pack.flow_comp_m_x == (float)1.02826214E37F);
                Debug.Assert(pack.flow_x == (short)(short)27397);
                Debug.Assert(pack.sensor_id == (byte)(byte)69);
                Debug.Assert(pack.quality == (byte)(byte)242);
                Debug.Assert(pack.ground_distance == (float) -1.5103413E38F);
                Debug.Assert(pack.flow_y == (short)(short)17861);
                Debug.Assert(pack.time_usec == (ulong)5624786440764379780L);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_rate_x_SET((float)2.8297322E38F, PH) ;
            p100.flow_comp_m_x = (float)1.02826214E37F;
            p100.flow_x = (short)(short)27397;
            p100.flow_y = (short)(short)17861;
            p100.flow_rate_y_SET((float)6.5145705E36F, PH) ;
            p100.quality = (byte)(byte)242;
            p100.ground_distance = (float) -1.5103413E38F;
            p100.time_usec = (ulong)5624786440764379780L;
            p100.sensor_id = (byte)(byte)69;
            p100.flow_comp_m_y = (float) -1.4216518E38F;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)9.856505E37F);
                Debug.Assert(pack.yaw == (float)1.388041E37F);
                Debug.Assert(pack.x == (float) -2.2175217E38F);
                Debug.Assert(pack.y == (float)1.3809695E38F);
                Debug.Assert(pack.z == (float)2.1821126E38F);
                Debug.Assert(pack.pitch == (float) -3.143707E38F);
                Debug.Assert(pack.usec == (ulong)8028899244456832530L);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.roll = (float)9.856505E37F;
            p101.y = (float)1.3809695E38F;
            p101.pitch = (float) -3.143707E38F;
            p101.x = (float) -2.2175217E38F;
            p101.z = (float)2.1821126E38F;
            p101.usec = (ulong)8028899244456832530L;
            p101.yaw = (float)1.388041E37F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)4.979714E37F);
                Debug.Assert(pack.yaw == (float) -8.85524E37F);
                Debug.Assert(pack.x == (float)2.7813566E38F);
                Debug.Assert(pack.z == (float)2.9567782E38F);
                Debug.Assert(pack.pitch == (float)3.0024677E37F);
                Debug.Assert(pack.y == (float)1.0359099E38F);
                Debug.Assert(pack.usec == (ulong)8808369366813714621L);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.x = (float)2.7813566E38F;
            p102.y = (float)1.0359099E38F;
            p102.pitch = (float)3.0024677E37F;
            p102.usec = (ulong)8808369366813714621L;
            p102.z = (float)2.9567782E38F;
            p102.roll = (float)4.979714E37F;
            p102.yaw = (float) -8.85524E37F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)7887518395809254240L);
                Debug.Assert(pack.z == (float) -2.4441363E37F);
                Debug.Assert(pack.x == (float) -2.9396059E38F);
                Debug.Assert(pack.y == (float)2.7429146E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.x = (float) -2.9396059E38F;
            p103.z = (float) -2.4441363E37F;
            p103.usec = (ulong)7887518395809254240L;
            p103.y = (float)2.7429146E38F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)1.8963687E37F);
                Debug.Assert(pack.yaw == (float)1.8084415E38F);
                Debug.Assert(pack.z == (float) -2.1677067E38F);
                Debug.Assert(pack.y == (float)3.0312485E38F);
                Debug.Assert(pack.pitch == (float) -2.4808257E38F);
                Debug.Assert(pack.usec == (ulong)199250410449994838L);
                Debug.Assert(pack.x == (float) -3.0416315E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.pitch = (float) -2.4808257E38F;
            p104.z = (float) -2.1677067E38F;
            p104.y = (float)3.0312485E38F;
            p104.x = (float) -3.0416315E38F;
            p104.roll = (float)1.8963687E37F;
            p104.yaw = (float)1.8084415E38F;
            p104.usec = (ulong)199250410449994838L;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (float) -2.5061846E38F);
                Debug.Assert(pack.zacc == (float)3.3505259E38F);
                Debug.Assert(pack.xgyro == (float) -2.8351158E38F);
                Debug.Assert(pack.xmag == (float)4.1434724E37F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)46803);
                Debug.Assert(pack.yacc == (float)1.6645157E38F);
                Debug.Assert(pack.abs_pressure == (float) -1.3622121E38F);
                Debug.Assert(pack.ymag == (float)7.404629E37F);
                Debug.Assert(pack.diff_pressure == (float)2.1900792E38F);
                Debug.Assert(pack.zmag == (float)1.2719702E38F);
                Debug.Assert(pack.pressure_alt == (float) -2.64852E38F);
                Debug.Assert(pack.ygyro == (float) -1.1110889E38F);
                Debug.Assert(pack.xacc == (float) -2.4194444E38F);
                Debug.Assert(pack.zgyro == (float) -1.87822E38F);
                Debug.Assert(pack.time_usec == (ulong)3162916474109178499L);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.zacc = (float)3.3505259E38F;
            p105.ygyro = (float) -1.1110889E38F;
            p105.xgyro = (float) -2.8351158E38F;
            p105.fields_updated = (ushort)(ushort)46803;
            p105.xmag = (float)4.1434724E37F;
            p105.zgyro = (float) -1.87822E38F;
            p105.yacc = (float)1.6645157E38F;
            p105.zmag = (float)1.2719702E38F;
            p105.ymag = (float)7.404629E37F;
            p105.pressure_alt = (float) -2.64852E38F;
            p105.diff_pressure = (float)2.1900792E38F;
            p105.abs_pressure = (float) -1.3622121E38F;
            p105.xacc = (float) -2.4194444E38F;
            p105.temperature = (float) -2.5061846E38F;
            p105.time_usec = (ulong)3162916474109178499L;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integration_time_us == (uint)2017045630U);
                Debug.Assert(pack.sensor_id == (byte)(byte)82);
                Debug.Assert(pack.integrated_x == (float) -1.9314385E38F);
                Debug.Assert(pack.integrated_xgyro == (float) -2.3268921E38F);
                Debug.Assert(pack.temperature == (short)(short)3081);
                Debug.Assert(pack.time_delta_distance_us == (uint)1013838504U);
                Debug.Assert(pack.time_usec == (ulong)2190522671971028459L);
                Debug.Assert(pack.distance == (float)7.738664E37F);
                Debug.Assert(pack.integrated_y == (float)1.6718242E38F);
                Debug.Assert(pack.quality == (byte)(byte)204);
                Debug.Assert(pack.integrated_zgyro == (float) -1.377847E38F);
                Debug.Assert(pack.integrated_ygyro == (float) -8.4207717E37F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_delta_distance_us = (uint)1013838504U;
            p106.integrated_y = (float)1.6718242E38F;
            p106.quality = (byte)(byte)204;
            p106.integrated_ygyro = (float) -8.4207717E37F;
            p106.integrated_xgyro = (float) -2.3268921E38F;
            p106.integrated_zgyro = (float) -1.377847E38F;
            p106.time_usec = (ulong)2190522671971028459L;
            p106.temperature = (short)(short)3081;
            p106.distance = (float)7.738664E37F;
            p106.integration_time_us = (uint)2017045630U;
            p106.integrated_x = (float) -1.9314385E38F;
            p106.sensor_id = (byte)(byte)82;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fields_updated == (uint)103733894U);
                Debug.Assert(pack.xacc == (float)2.4325775E38F);
                Debug.Assert(pack.xmag == (float)2.7573804E38F);
                Debug.Assert(pack.pressure_alt == (float) -1.1222042E38F);
                Debug.Assert(pack.abs_pressure == (float)2.6298349E38F);
                Debug.Assert(pack.xgyro == (float) -1.954569E38F);
                Debug.Assert(pack.zacc == (float) -2.2662936E38F);
                Debug.Assert(pack.zgyro == (float) -6.703591E37F);
                Debug.Assert(pack.temperature == (float)2.7400811E38F);
                Debug.Assert(pack.diff_pressure == (float) -1.6606018E38F);
                Debug.Assert(pack.yacc == (float) -3.5495984E37F);
                Debug.Assert(pack.ymag == (float)2.7219904E38F);
                Debug.Assert(pack.time_usec == (ulong)4425539205048305080L);
                Debug.Assert(pack.ygyro == (float) -2.4613187E38F);
                Debug.Assert(pack.zmag == (float)1.4696408E38F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.zmag = (float)1.4696408E38F;
            p107.zacc = (float) -2.2662936E38F;
            p107.temperature = (float)2.7400811E38F;
            p107.ygyro = (float) -2.4613187E38F;
            p107.diff_pressure = (float) -1.6606018E38F;
            p107.yacc = (float) -3.5495984E37F;
            p107.fields_updated = (uint)103733894U;
            p107.ymag = (float)2.7219904E38F;
            p107.zgyro = (float) -6.703591E37F;
            p107.xgyro = (float) -1.954569E38F;
            p107.xmag = (float)2.7573804E38F;
            p107.time_usec = (ulong)4425539205048305080L;
            p107.xacc = (float)2.4325775E38F;
            p107.abs_pressure = (float)2.6298349E38F;
            p107.pressure_alt = (float) -1.1222042E38F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ve == (float)1.402139E38F);
                Debug.Assert(pack.xacc == (float)7.5804527E37F);
                Debug.Assert(pack.q4 == (float) -1.7696822E38F);
                Debug.Assert(pack.lon == (float) -3.372702E37F);
                Debug.Assert(pack.yacc == (float)8.868072E36F);
                Debug.Assert(pack.yaw == (float) -3.3078626E38F);
                Debug.Assert(pack.zacc == (float) -1.4716548E38F);
                Debug.Assert(pack.vn == (float) -3.0276772E38F);
                Debug.Assert(pack.vd == (float)2.5081057E38F);
                Debug.Assert(pack.q2 == (float) -1.5858063E38F);
                Debug.Assert(pack.zgyro == (float) -1.9630596E38F);
                Debug.Assert(pack.std_dev_vert == (float)1.660421E38F);
                Debug.Assert(pack.q1 == (float)3.4185383E36F);
                Debug.Assert(pack.lat == (float)4.982183E36F);
                Debug.Assert(pack.alt == (float) -1.5357773E38F);
                Debug.Assert(pack.q3 == (float) -3.2105472E38F);
                Debug.Assert(pack.std_dev_horz == (float) -1.936219E38F);
                Debug.Assert(pack.pitch == (float) -3.382691E38F);
                Debug.Assert(pack.ygyro == (float)1.2555027E38F);
                Debug.Assert(pack.xgyro == (float)1.1849652E37F);
                Debug.Assert(pack.roll == (float) -2.5554408E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.lon = (float) -3.372702E37F;
            p108.zgyro = (float) -1.9630596E38F;
            p108.std_dev_vert = (float)1.660421E38F;
            p108.alt = (float) -1.5357773E38F;
            p108.zacc = (float) -1.4716548E38F;
            p108.vn = (float) -3.0276772E38F;
            p108.q2 = (float) -1.5858063E38F;
            p108.ygyro = (float)1.2555027E38F;
            p108.q1 = (float)3.4185383E36F;
            p108.lat = (float)4.982183E36F;
            p108.yacc = (float)8.868072E36F;
            p108.xacc = (float)7.5804527E37F;
            p108.vd = (float)2.5081057E38F;
            p108.xgyro = (float)1.1849652E37F;
            p108.q4 = (float) -1.7696822E38F;
            p108.pitch = (float) -3.382691E38F;
            p108.ve = (float)1.402139E38F;
            p108.roll = (float) -2.5554408E38F;
            p108.std_dev_horz = (float) -1.936219E38F;
            p108.q3 = (float) -3.2105472E38F;
            p108.yaw = (float) -3.3078626E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.noise == (byte)(byte)163);
                Debug.Assert(pack.remrssi == (byte)(byte)114);
                Debug.Assert(pack.rssi == (byte)(byte)153);
                Debug.Assert(pack.remnoise == (byte)(byte)111);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)12424);
                Debug.Assert(pack.txbuf == (byte)(byte)234);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)45811);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.remrssi = (byte)(byte)114;
            p109.txbuf = (byte)(byte)234;
            p109.rssi = (byte)(byte)153;
            p109.rxerrors = (ushort)(ushort)45811;
            p109.remnoise = (byte)(byte)111;
            p109.fixed_ = (ushort)(ushort)12424;
            p109.noise = (byte)(byte)163;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)34, (byte)63, (byte)227, (byte)216, (byte)197, (byte)175, (byte)156, (byte)56, (byte)190, (byte)70, (byte)159, (byte)169, (byte)34, (byte)122, (byte)133, (byte)108, (byte)190, (byte)199, (byte)201, (byte)85, (byte)231, (byte)33, (byte)232, (byte)179, (byte)186, (byte)42, (byte)162, (byte)70, (byte)226, (byte)183, (byte)75, (byte)168, (byte)249, (byte)76, (byte)82, (byte)83, (byte)137, (byte)99, (byte)172, (byte)9, (byte)20, (byte)239, (byte)179, (byte)224, (byte)95, (byte)2, (byte)44, (byte)7, (byte)155, (byte)250, (byte)243, (byte)148, (byte)238, (byte)205, (byte)230, (byte)116, (byte)122, (byte)2, (byte)4, (byte)0, (byte)57, (byte)38, (byte)119, (byte)28, (byte)22, (byte)29, (byte)46, (byte)237, (byte)254, (byte)42, (byte)64, (byte)245, (byte)96, (byte)216, (byte)92, (byte)201, (byte)239, (byte)117, (byte)36, (byte)1, (byte)202, (byte)28, (byte)185, (byte)95, (byte)222, (byte)95, (byte)175, (byte)107, (byte)88, (byte)140, (byte)168, (byte)215, (byte)201, (byte)226, (byte)216, (byte)60, (byte)249, (byte)139, (byte)229, (byte)116, (byte)155, (byte)195, (byte)157, (byte)49, (byte)218, (byte)165, (byte)104, (byte)47, (byte)39, (byte)246, (byte)198, (byte)182, (byte)153, (byte)34, (byte)58, (byte)161, (byte)248, (byte)221, (byte)46, (byte)219, (byte)15, (byte)143, (byte)198, (byte)152, (byte)38, (byte)22, (byte)93, (byte)10, (byte)65, (byte)114, (byte)252, (byte)43, (byte)32, (byte)21, (byte)22, (byte)133, (byte)85, (byte)120, (byte)213, (byte)137, (byte)92, (byte)165, (byte)247, (byte)20, (byte)238, (byte)216, (byte)183, (byte)108, (byte)197, (byte)65, (byte)175, (byte)163, (byte)14, (byte)1, (byte)108, (byte)2, (byte)205, (byte)92, (byte)22, (byte)122, (byte)192, (byte)219, (byte)53, (byte)188, (byte)237, (byte)61, (byte)22, (byte)149, (byte)93, (byte)85, (byte)12, (byte)210, (byte)115, (byte)89, (byte)37, (byte)12, (byte)184, (byte)159, (byte)131, (byte)25, (byte)175, (byte)94, (byte)201, (byte)123, (byte)119, (byte)153, (byte)120, (byte)42, (byte)226, (byte)180, (byte)232, (byte)56, (byte)242, (byte)47, (byte)101, (byte)132, (byte)13, (byte)230, (byte)80, (byte)134, (byte)68, (byte)105, (byte)77, (byte)3, (byte)127, (byte)163, (byte)25, (byte)24, (byte)53, (byte)173, (byte)247, (byte)192, (byte)34, (byte)191, (byte)42, (byte)45, (byte)102, (byte)195, (byte)86, (byte)145, (byte)183, (byte)189, (byte)224, (byte)242, (byte)225, (byte)108, (byte)169, (byte)62, (byte)14, (byte)95, (byte)223, (byte)218, (byte)213, (byte)133, (byte)99, (byte)86, (byte)76, (byte)132, (byte)236, (byte)195, (byte)75, (byte)25, (byte)65, (byte)219, (byte)161, (byte)252, (byte)26, (byte)109, (byte)205, (byte)155, (byte)122}));
                Debug.Assert(pack.target_component == (byte)(byte)167);
                Debug.Assert(pack.target_network == (byte)(byte)47);
                Debug.Assert(pack.target_system == (byte)(byte)214);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_system = (byte)(byte)214;
            p110.target_component = (byte)(byte)167;
            p110.payload_SET(new byte[] {(byte)34, (byte)63, (byte)227, (byte)216, (byte)197, (byte)175, (byte)156, (byte)56, (byte)190, (byte)70, (byte)159, (byte)169, (byte)34, (byte)122, (byte)133, (byte)108, (byte)190, (byte)199, (byte)201, (byte)85, (byte)231, (byte)33, (byte)232, (byte)179, (byte)186, (byte)42, (byte)162, (byte)70, (byte)226, (byte)183, (byte)75, (byte)168, (byte)249, (byte)76, (byte)82, (byte)83, (byte)137, (byte)99, (byte)172, (byte)9, (byte)20, (byte)239, (byte)179, (byte)224, (byte)95, (byte)2, (byte)44, (byte)7, (byte)155, (byte)250, (byte)243, (byte)148, (byte)238, (byte)205, (byte)230, (byte)116, (byte)122, (byte)2, (byte)4, (byte)0, (byte)57, (byte)38, (byte)119, (byte)28, (byte)22, (byte)29, (byte)46, (byte)237, (byte)254, (byte)42, (byte)64, (byte)245, (byte)96, (byte)216, (byte)92, (byte)201, (byte)239, (byte)117, (byte)36, (byte)1, (byte)202, (byte)28, (byte)185, (byte)95, (byte)222, (byte)95, (byte)175, (byte)107, (byte)88, (byte)140, (byte)168, (byte)215, (byte)201, (byte)226, (byte)216, (byte)60, (byte)249, (byte)139, (byte)229, (byte)116, (byte)155, (byte)195, (byte)157, (byte)49, (byte)218, (byte)165, (byte)104, (byte)47, (byte)39, (byte)246, (byte)198, (byte)182, (byte)153, (byte)34, (byte)58, (byte)161, (byte)248, (byte)221, (byte)46, (byte)219, (byte)15, (byte)143, (byte)198, (byte)152, (byte)38, (byte)22, (byte)93, (byte)10, (byte)65, (byte)114, (byte)252, (byte)43, (byte)32, (byte)21, (byte)22, (byte)133, (byte)85, (byte)120, (byte)213, (byte)137, (byte)92, (byte)165, (byte)247, (byte)20, (byte)238, (byte)216, (byte)183, (byte)108, (byte)197, (byte)65, (byte)175, (byte)163, (byte)14, (byte)1, (byte)108, (byte)2, (byte)205, (byte)92, (byte)22, (byte)122, (byte)192, (byte)219, (byte)53, (byte)188, (byte)237, (byte)61, (byte)22, (byte)149, (byte)93, (byte)85, (byte)12, (byte)210, (byte)115, (byte)89, (byte)37, (byte)12, (byte)184, (byte)159, (byte)131, (byte)25, (byte)175, (byte)94, (byte)201, (byte)123, (byte)119, (byte)153, (byte)120, (byte)42, (byte)226, (byte)180, (byte)232, (byte)56, (byte)242, (byte)47, (byte)101, (byte)132, (byte)13, (byte)230, (byte)80, (byte)134, (byte)68, (byte)105, (byte)77, (byte)3, (byte)127, (byte)163, (byte)25, (byte)24, (byte)53, (byte)173, (byte)247, (byte)192, (byte)34, (byte)191, (byte)42, (byte)45, (byte)102, (byte)195, (byte)86, (byte)145, (byte)183, (byte)189, (byte)224, (byte)242, (byte)225, (byte)108, (byte)169, (byte)62, (byte)14, (byte)95, (byte)223, (byte)218, (byte)213, (byte)133, (byte)99, (byte)86, (byte)76, (byte)132, (byte)236, (byte)195, (byte)75, (byte)25, (byte)65, (byte)219, (byte)161, (byte)252, (byte)26, (byte)109, (byte)205, (byte)155, (byte)122}, 0) ;
            p110.target_network = (byte)(byte)47;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long) -7988213300453226224L);
                Debug.Assert(pack.tc1 == (long) -2913814605830090161L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long) -7988213300453226224L;
            p111.tc1 = (long) -2913814605830090161L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5145502327632659252L);
                Debug.Assert(pack.seq == (uint)3250760079U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)3250760079U;
            p112.time_usec = (ulong)5145502327632659252L;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == (byte)(byte)162);
                Debug.Assert(pack.vel == (ushort)(ushort)46109);
                Debug.Assert(pack.epv == (ushort)(ushort)62126);
                Debug.Assert(pack.cog == (ushort)(ushort)61810);
                Debug.Assert(pack.lat == (int) -202281376);
                Debug.Assert(pack.satellites_visible == (byte)(byte)127);
                Debug.Assert(pack.alt == (int) -1854347972);
                Debug.Assert(pack.vd == (short)(short)27597);
                Debug.Assert(pack.vn == (short)(short) -23187);
                Debug.Assert(pack.lon == (int)1208974885);
                Debug.Assert(pack.eph == (ushort)(ushort)59766);
                Debug.Assert(pack.ve == (short)(short) -17904);
                Debug.Assert(pack.time_usec == (ulong)9076252247854266581L);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.lon = (int)1208974885;
            p113.vel = (ushort)(ushort)46109;
            p113.vn = (short)(short) -23187;
            p113.eph = (ushort)(ushort)59766;
            p113.epv = (ushort)(ushort)62126;
            p113.fix_type = (byte)(byte)162;
            p113.ve = (short)(short) -17904;
            p113.satellites_visible = (byte)(byte)127;
            p113.alt = (int) -1854347972;
            p113.cog = (ushort)(ushort)61810;
            p113.vd = (short)(short)27597;
            p113.time_usec = (ulong)9076252247854266581L;
            p113.lat = (int) -202281376;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_y == (float) -1.917292E38F);
                Debug.Assert(pack.integrated_ygyro == (float) -1.4635651E38F);
                Debug.Assert(pack.temperature == (short)(short)31119);
                Debug.Assert(pack.sensor_id == (byte)(byte)205);
                Debug.Assert(pack.quality == (byte)(byte)255);
                Debug.Assert(pack.integrated_xgyro == (float) -3.3125686E38F);
                Debug.Assert(pack.time_usec == (ulong)4613961965653545049L);
                Debug.Assert(pack.integrated_x == (float) -4.6652544E37F);
                Debug.Assert(pack.integrated_zgyro == (float) -2.9071946E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)2764579593U);
                Debug.Assert(pack.integration_time_us == (uint)3811598902U);
                Debug.Assert(pack.distance == (float)1.0346029E38F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integrated_x = (float) -4.6652544E37F;
            p114.integrated_xgyro = (float) -3.3125686E38F;
            p114.sensor_id = (byte)(byte)205;
            p114.integrated_zgyro = (float) -2.9071946E38F;
            p114.time_delta_distance_us = (uint)2764579593U;
            p114.quality = (byte)(byte)255;
            p114.time_usec = (ulong)4613961965653545049L;
            p114.temperature = (short)(short)31119;
            p114.integrated_ygyro = (float) -1.4635651E38F;
            p114.distance = (float)1.0346029E38F;
            p114.integrated_y = (float) -1.917292E38F;
            p114.integration_time_us = (uint)3811598902U;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (short)(short) -16829);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)9883);
                Debug.Assert(pack.alt == (int)1227445317);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)56758);
                Debug.Assert(pack.rollspeed == (float) -2.6772375E38F);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {3.1270066E38F, 1.6602353E38F, 1.8625077E38F, 2.0846338E38F}));
                Debug.Assert(pack.yawspeed == (float)2.8522353E38F);
                Debug.Assert(pack.lat == (int) -1493837628);
                Debug.Assert(pack.zacc == (short)(short) -28067);
                Debug.Assert(pack.pitchspeed == (float) -7.0654514E37F);
                Debug.Assert(pack.vz == (short)(short)14140);
                Debug.Assert(pack.time_usec == (ulong)7965994415009993948L);
                Debug.Assert(pack.lon == (int) -1468423744);
                Debug.Assert(pack.vx == (short)(short) -22948);
                Debug.Assert(pack.xacc == (short)(short)22038);
                Debug.Assert(pack.yacc == (short)(short)8099);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.zacc = (short)(short) -28067;
            p115.attitude_quaternion_SET(new float[] {3.1270066E38F, 1.6602353E38F, 1.8625077E38F, 2.0846338E38F}, 0) ;
            p115.vx = (short)(short) -22948;
            p115.pitchspeed = (float) -7.0654514E37F;
            p115.lat = (int) -1493837628;
            p115.vy = (short)(short) -16829;
            p115.true_airspeed = (ushort)(ushort)56758;
            p115.rollspeed = (float) -2.6772375E38F;
            p115.lon = (int) -1468423744;
            p115.vz = (short)(short)14140;
            p115.ind_airspeed = (ushort)(ushort)9883;
            p115.yawspeed = (float)2.8522353E38F;
            p115.alt = (int)1227445317;
            p115.time_usec = (ulong)7965994415009993948L;
            p115.yacc = (short)(short)8099;
            p115.xacc = (short)(short)22038;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (short)(short)13496);
                Debug.Assert(pack.yacc == (short)(short)3827);
                Debug.Assert(pack.zgyro == (short)(short) -15293);
                Debug.Assert(pack.xmag == (short)(short) -15192);
                Debug.Assert(pack.ygyro == (short)(short) -151);
                Debug.Assert(pack.time_boot_ms == (uint)3147287820U);
                Debug.Assert(pack.ymag == (short)(short) -32094);
                Debug.Assert(pack.xacc == (short)(short) -24278);
                Debug.Assert(pack.zacc == (short)(short)2815);
                Debug.Assert(pack.xgyro == (short)(short)32231);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.zmag = (short)(short)13496;
            p116.zacc = (short)(short)2815;
            p116.ygyro = (short)(short) -151;
            p116.xacc = (short)(short) -24278;
            p116.yacc = (short)(short)3827;
            p116.time_boot_ms = (uint)3147287820U;
            p116.zgyro = (short)(short) -15293;
            p116.ymag = (short)(short) -32094;
            p116.xmag = (short)(short) -15192;
            p116.xgyro = (short)(short)32231;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start == (ushort)(ushort)49343);
                Debug.Assert(pack.target_system == (byte)(byte)25);
                Debug.Assert(pack.target_component == (byte)(byte)124);
                Debug.Assert(pack.end == (ushort)(ushort)11778);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.start = (ushort)(ushort)49343;
            p117.target_component = (byte)(byte)124;
            p117.target_system = (byte)(byte)25;
            p117.end = (ushort)(ushort)11778;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)26714);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)1113);
                Debug.Assert(pack.num_logs == (ushort)(ushort)6324);
                Debug.Assert(pack.time_utc == (uint)3558752988U);
                Debug.Assert(pack.size == (uint)1799486204U);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.size = (uint)1799486204U;
            p118.last_log_num = (ushort)(ushort)1113;
            p118.id = (ushort)(ushort)26714;
            p118.time_utc = (uint)3558752988U;
            p118.num_logs = (ushort)(ushort)6324;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)203);
                Debug.Assert(pack.id == (ushort)(ushort)10476);
                Debug.Assert(pack.count == (uint)303725264U);
                Debug.Assert(pack.ofs == (uint)959259378U);
                Debug.Assert(pack.target_component == (byte)(byte)83);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.count = (uint)303725264U;
            p119.id = (ushort)(ushort)10476;
            p119.target_system = (byte)(byte)203;
            p119.target_component = (byte)(byte)83;
            p119.ofs = (uint)959259378U;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)1956833192U);
                Debug.Assert(pack.id == (ushort)(ushort)32403);
                Debug.Assert(pack.count == (byte)(byte)240);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)246, (byte)99, (byte)254, (byte)205, (byte)151, (byte)138, (byte)113, (byte)50, (byte)77, (byte)190, (byte)158, (byte)197, (byte)60, (byte)78, (byte)214, (byte)131, (byte)8, (byte)140, (byte)241, (byte)211, (byte)86, (byte)3, (byte)49, (byte)210, (byte)57, (byte)4, (byte)242, (byte)187, (byte)195, (byte)209, (byte)33, (byte)149, (byte)126, (byte)99, (byte)202, (byte)20, (byte)102, (byte)28, (byte)103, (byte)7, (byte)48, (byte)138, (byte)115, (byte)206, (byte)62, (byte)142, (byte)92, (byte)206, (byte)171, (byte)156, (byte)137, (byte)66, (byte)67, (byte)110, (byte)221, (byte)182, (byte)207, (byte)50, (byte)33, (byte)158, (byte)155, (byte)215, (byte)232, (byte)31, (byte)155, (byte)172, (byte)85, (byte)27, (byte)28, (byte)91, (byte)42, (byte)35, (byte)137, (byte)180, (byte)11, (byte)34, (byte)86, (byte)62, (byte)87, (byte)117, (byte)59, (byte)103, (byte)167, (byte)126, (byte)111, (byte)51, (byte)187, (byte)104, (byte)209, (byte)231}));
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)32403;
            p120.ofs = (uint)1956833192U;
            p120.data__SET(new byte[] {(byte)246, (byte)99, (byte)254, (byte)205, (byte)151, (byte)138, (byte)113, (byte)50, (byte)77, (byte)190, (byte)158, (byte)197, (byte)60, (byte)78, (byte)214, (byte)131, (byte)8, (byte)140, (byte)241, (byte)211, (byte)86, (byte)3, (byte)49, (byte)210, (byte)57, (byte)4, (byte)242, (byte)187, (byte)195, (byte)209, (byte)33, (byte)149, (byte)126, (byte)99, (byte)202, (byte)20, (byte)102, (byte)28, (byte)103, (byte)7, (byte)48, (byte)138, (byte)115, (byte)206, (byte)62, (byte)142, (byte)92, (byte)206, (byte)171, (byte)156, (byte)137, (byte)66, (byte)67, (byte)110, (byte)221, (byte)182, (byte)207, (byte)50, (byte)33, (byte)158, (byte)155, (byte)215, (byte)232, (byte)31, (byte)155, (byte)172, (byte)85, (byte)27, (byte)28, (byte)91, (byte)42, (byte)35, (byte)137, (byte)180, (byte)11, (byte)34, (byte)86, (byte)62, (byte)87, (byte)117, (byte)59, (byte)103, (byte)167, (byte)126, (byte)111, (byte)51, (byte)187, (byte)104, (byte)209, (byte)231}, 0) ;
            p120.count = (byte)(byte)240;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)234);
                Debug.Assert(pack.target_system == (byte)(byte)69);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)69;
            p121.target_component = (byte)(byte)234;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)8);
                Debug.Assert(pack.target_system == (byte)(byte)143);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)143;
            p122.target_component = (byte)(byte)8;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)129);
                Debug.Assert(pack.target_system == (byte)(byte)42);
                Debug.Assert(pack.target_component == (byte)(byte)217);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)232, (byte)163, (byte)215, (byte)47, (byte)107, (byte)253, (byte)242, (byte)120, (byte)201, (byte)246, (byte)244, (byte)9, (byte)171, (byte)100, (byte)66, (byte)235, (byte)219, (byte)84, (byte)13, (byte)203, (byte)21, (byte)24, (byte)241, (byte)46, (byte)208, (byte)179, (byte)141, (byte)26, (byte)118, (byte)13, (byte)246, (byte)194, (byte)154, (byte)249, (byte)235, (byte)26, (byte)188, (byte)56, (byte)211, (byte)98, (byte)179, (byte)167, (byte)84, (byte)216, (byte)144, (byte)241, (byte)81, (byte)108, (byte)33, (byte)111, (byte)33, (byte)133, (byte)57, (byte)94, (byte)90, (byte)34, (byte)87, (byte)196, (byte)36, (byte)139, (byte)216, (byte)171, (byte)238, (byte)211, (byte)163, (byte)56, (byte)108, (byte)91, (byte)177, (byte)18, (byte)83, (byte)89, (byte)209, (byte)104, (byte)140, (byte)76, (byte)151, (byte)158, (byte)179, (byte)55, (byte)186, (byte)118, (byte)90, (byte)172, (byte)133, (byte)249, (byte)60, (byte)145, (byte)42, (byte)90, (byte)198, (byte)102, (byte)72, (byte)189, (byte)230, (byte)166, (byte)143, (byte)234, (byte)26, (byte)169, (byte)177, (byte)33, (byte)61, (byte)216, (byte)220, (byte)143, (byte)227, (byte)191, (byte)84, (byte)54}));
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.data__SET(new byte[] {(byte)232, (byte)163, (byte)215, (byte)47, (byte)107, (byte)253, (byte)242, (byte)120, (byte)201, (byte)246, (byte)244, (byte)9, (byte)171, (byte)100, (byte)66, (byte)235, (byte)219, (byte)84, (byte)13, (byte)203, (byte)21, (byte)24, (byte)241, (byte)46, (byte)208, (byte)179, (byte)141, (byte)26, (byte)118, (byte)13, (byte)246, (byte)194, (byte)154, (byte)249, (byte)235, (byte)26, (byte)188, (byte)56, (byte)211, (byte)98, (byte)179, (byte)167, (byte)84, (byte)216, (byte)144, (byte)241, (byte)81, (byte)108, (byte)33, (byte)111, (byte)33, (byte)133, (byte)57, (byte)94, (byte)90, (byte)34, (byte)87, (byte)196, (byte)36, (byte)139, (byte)216, (byte)171, (byte)238, (byte)211, (byte)163, (byte)56, (byte)108, (byte)91, (byte)177, (byte)18, (byte)83, (byte)89, (byte)209, (byte)104, (byte)140, (byte)76, (byte)151, (byte)158, (byte)179, (byte)55, (byte)186, (byte)118, (byte)90, (byte)172, (byte)133, (byte)249, (byte)60, (byte)145, (byte)42, (byte)90, (byte)198, (byte)102, (byte)72, (byte)189, (byte)230, (byte)166, (byte)143, (byte)234, (byte)26, (byte)169, (byte)177, (byte)33, (byte)61, (byte)216, (byte)220, (byte)143, (byte)227, (byte)191, (byte)84, (byte)54}, 0) ;
            p123.len = (byte)(byte)129;
            p123.target_system = (byte)(byte)42;
            p123.target_component = (byte)(byte)217;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5120032120367576575L);
                Debug.Assert(pack.vel == (ushort)(ushort)45905);
                Debug.Assert(pack.alt == (int) -774918528);
                Debug.Assert(pack.dgps_age == (uint)3662492354U);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
                Debug.Assert(pack.satellites_visible == (byte)(byte)239);
                Debug.Assert(pack.eph == (ushort)(ushort)26765);
                Debug.Assert(pack.dgps_numch == (byte)(byte)2);
                Debug.Assert(pack.epv == (ushort)(ushort)15465);
                Debug.Assert(pack.lon == (int) -742726760);
                Debug.Assert(pack.cog == (ushort)(ushort)24935);
                Debug.Assert(pack.lat == (int)780079543);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED;
            p124.lon = (int) -742726760;
            p124.eph = (ushort)(ushort)26765;
            p124.lat = (int)780079543;
            p124.dgps_age = (uint)3662492354U;
            p124.satellites_visible = (byte)(byte)239;
            p124.alt = (int) -774918528;
            p124.cog = (ushort)(ushort)24935;
            p124.dgps_numch = (byte)(byte)2;
            p124.vel = (ushort)(ushort)45905;
            p124.epv = (ushort)(ushort)15465;
            p124.time_usec = (ulong)5120032120367576575L;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)59471);
                Debug.Assert(pack.Vservo == (ushort)(ushort)6489);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)59471;
            p125.Vservo = (ushort)(ushort)6489;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)190, (byte)57, (byte)41, (byte)80, (byte)138, (byte)210, (byte)52, (byte)24, (byte)26, (byte)57, (byte)238, (byte)161, (byte)224, (byte)143, (byte)195, (byte)174, (byte)13, (byte)24, (byte)20, (byte)111, (byte)173, (byte)75, (byte)38, (byte)39, (byte)109, (byte)78, (byte)127, (byte)253, (byte)5, (byte)77, (byte)101, (byte)210, (byte)77, (byte)211, (byte)152, (byte)187, (byte)74, (byte)219, (byte)145, (byte)11, (byte)255, (byte)127, (byte)220, (byte)150, (byte)47, (byte)103, (byte)215, (byte)144, (byte)185, (byte)85, (byte)185, (byte)78, (byte)122, (byte)50, (byte)153, (byte)172, (byte)110, (byte)93, (byte)249, (byte)116, (byte)187, (byte)105, (byte)18, (byte)227, (byte)85, (byte)143, (byte)52, (byte)91, (byte)118, (byte)17}));
                Debug.Assert(pack.count == (byte)(byte)124);
                Debug.Assert(pack.baudrate == (uint)296592654U);
                Debug.Assert(pack.timeout == (ushort)(ushort)51512);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.baudrate = (uint)296592654U;
            p126.data__SET(new byte[] {(byte)190, (byte)57, (byte)41, (byte)80, (byte)138, (byte)210, (byte)52, (byte)24, (byte)26, (byte)57, (byte)238, (byte)161, (byte)224, (byte)143, (byte)195, (byte)174, (byte)13, (byte)24, (byte)20, (byte)111, (byte)173, (byte)75, (byte)38, (byte)39, (byte)109, (byte)78, (byte)127, (byte)253, (byte)5, (byte)77, (byte)101, (byte)210, (byte)77, (byte)211, (byte)152, (byte)187, (byte)74, (byte)219, (byte)145, (byte)11, (byte)255, (byte)127, (byte)220, (byte)150, (byte)47, (byte)103, (byte)215, (byte)144, (byte)185, (byte)85, (byte)185, (byte)78, (byte)122, (byte)50, (byte)153, (byte)172, (byte)110, (byte)93, (byte)249, (byte)116, (byte)187, (byte)105, (byte)18, (byte)227, (byte)85, (byte)143, (byte)52, (byte)91, (byte)118, (byte)17}, 0) ;
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2;
            p126.count = (byte)(byte)124;
            p126.timeout = (ushort)(ushort)51512;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nsats == (byte)(byte)128);
                Debug.Assert(pack.rtk_rate == (byte)(byte)8);
                Debug.Assert(pack.baseline_b_mm == (int) -259723312);
                Debug.Assert(pack.rtk_health == (byte)(byte)93);
                Debug.Assert(pack.wn == (ushort)(ushort)10588);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)168);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2085862641U);
                Debug.Assert(pack.tow == (uint)3422807667U);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)138);
                Debug.Assert(pack.accuracy == (uint)3250151015U);
                Debug.Assert(pack.baseline_c_mm == (int)752656817);
                Debug.Assert(pack.baseline_a_mm == (int)723371977);
                Debug.Assert(pack.iar_num_hypotheses == (int) -766937168);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.nsats = (byte)(byte)128;
            p127.time_last_baseline_ms = (uint)2085862641U;
            p127.baseline_b_mm = (int) -259723312;
            p127.accuracy = (uint)3250151015U;
            p127.rtk_receiver_id = (byte)(byte)168;
            p127.baseline_coords_type = (byte)(byte)138;
            p127.wn = (ushort)(ushort)10588;
            p127.rtk_health = (byte)(byte)93;
            p127.baseline_c_mm = (int)752656817;
            p127.tow = (uint)3422807667U;
            p127.iar_num_hypotheses = (int) -766937168;
            p127.rtk_rate = (byte)(byte)8;
            p127.baseline_a_mm = (int)723371977;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tow == (uint)3909105703U);
                Debug.Assert(pack.rtk_health == (byte)(byte)121);
                Debug.Assert(pack.iar_num_hypotheses == (int) -228394812);
                Debug.Assert(pack.accuracy == (uint)177893981U);
                Debug.Assert(pack.time_last_baseline_ms == (uint)283571766U);
                Debug.Assert(pack.baseline_a_mm == (int) -1697679476);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)22);
                Debug.Assert(pack.rtk_rate == (byte)(byte)0);
                Debug.Assert(pack.nsats == (byte)(byte)114);
                Debug.Assert(pack.wn == (ushort)(ushort)37440);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)71);
                Debug.Assert(pack.baseline_b_mm == (int)1163609835);
                Debug.Assert(pack.baseline_c_mm == (int) -384064642);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)283571766U;
            p128.baseline_c_mm = (int) -384064642;
            p128.baseline_a_mm = (int) -1697679476;
            p128.rtk_rate = (byte)(byte)0;
            p128.rtk_receiver_id = (byte)(byte)22;
            p128.wn = (ushort)(ushort)37440;
            p128.baseline_b_mm = (int)1163609835;
            p128.iar_num_hypotheses = (int) -228394812;
            p128.baseline_coords_type = (byte)(byte)71;
            p128.nsats = (byte)(byte)114;
            p128.accuracy = (uint)177893981U;
            p128.tow = (uint)3909105703U;
            p128.rtk_health = (byte)(byte)121;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (short)(short)32546);
                Debug.Assert(pack.time_boot_ms == (uint)2420939833U);
                Debug.Assert(pack.xgyro == (short)(short) -8148);
                Debug.Assert(pack.zacc == (short)(short)4512);
                Debug.Assert(pack.yacc == (short)(short)26575);
                Debug.Assert(pack.xacc == (short)(short)20845);
                Debug.Assert(pack.ymag == (short)(short) -2365);
                Debug.Assert(pack.zmag == (short)(short) -15983);
                Debug.Assert(pack.ygyro == (short)(short)8158);
                Debug.Assert(pack.xmag == (short)(short)28608);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.zacc = (short)(short)4512;
            p129.zgyro = (short)(short)32546;
            p129.xacc = (short)(short)20845;
            p129.xgyro = (short)(short) -8148;
            p129.ymag = (short)(short) -2365;
            p129.time_boot_ms = (uint)2420939833U;
            p129.yacc = (short)(short)26575;
            p129.xmag = (short)(short)28608;
            p129.zmag = (short)(short) -15983;
            p129.ygyro = (short)(short)8158;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.size == (uint)2999875039U);
                Debug.Assert(pack.jpg_quality == (byte)(byte)64);
                Debug.Assert(pack.type == (byte)(byte)205);
                Debug.Assert(pack.width == (ushort)(ushort)50378);
                Debug.Assert(pack.height == (ushort)(ushort)35640);
                Debug.Assert(pack.payload == (byte)(byte)254);
                Debug.Assert(pack.packets == (ushort)(ushort)31452);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.width = (ushort)(ushort)50378;
            p130.height = (ushort)(ushort)35640;
            p130.jpg_quality = (byte)(byte)64;
            p130.packets = (ushort)(ushort)31452;
            p130.payload = (byte)(byte)254;
            p130.size = (uint)2999875039U;
            p130.type = (byte)(byte)205;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seqnr == (ushort)(ushort)40847);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)217, (byte)154, (byte)224, (byte)40, (byte)82, (byte)252, (byte)158, (byte)44, (byte)67, (byte)85, (byte)237, (byte)175, (byte)211, (byte)106, (byte)179, (byte)135, (byte)85, (byte)30, (byte)194, (byte)1, (byte)1, (byte)161, (byte)90, (byte)169, (byte)28, (byte)65, (byte)187, (byte)208, (byte)97, (byte)235, (byte)114, (byte)163, (byte)41, (byte)79, (byte)84, (byte)20, (byte)221, (byte)135, (byte)21, (byte)2, (byte)134, (byte)159, (byte)144, (byte)45, (byte)119, (byte)31, (byte)162, (byte)141, (byte)238, (byte)248, (byte)58, (byte)223, (byte)242, (byte)3, (byte)52, (byte)91, (byte)221, (byte)51, (byte)241, (byte)230, (byte)38, (byte)19, (byte)154, (byte)70, (byte)69, (byte)255, (byte)209, (byte)34, (byte)71, (byte)58, (byte)110, (byte)149, (byte)121, (byte)126, (byte)6, (byte)226, (byte)164, (byte)195, (byte)218, (byte)53, (byte)188, (byte)223, (byte)90, (byte)34, (byte)248, (byte)75, (byte)151, (byte)196, (byte)219, (byte)34, (byte)244, (byte)211, (byte)17, (byte)100, (byte)213, (byte)233, (byte)218, (byte)215, (byte)16, (byte)239, (byte)102, (byte)195, (byte)13, (byte)83, (byte)79, (byte)243, (byte)13, (byte)13, (byte)34, (byte)158, (byte)248, (byte)83, (byte)116, (byte)111, (byte)185, (byte)231, (byte)120, (byte)198, (byte)9, (byte)254, (byte)173, (byte)28, (byte)200, (byte)221, (byte)196, (byte)189, (byte)48, (byte)187, (byte)115, (byte)98, (byte)7, (byte)6, (byte)255, (byte)17, (byte)159, (byte)219, (byte)49, (byte)213, (byte)94, (byte)90, (byte)244, (byte)152, (byte)120, (byte)225, (byte)101, (byte)96, (byte)114, (byte)55, (byte)185, (byte)21, (byte)173, (byte)162, (byte)45, (byte)138, (byte)248, (byte)48, (byte)152, (byte)59, (byte)149, (byte)213, (byte)165, (byte)5, (byte)185, (byte)214, (byte)22, (byte)180, (byte)207, (byte)96, (byte)113, (byte)98, (byte)51, (byte)128, (byte)232, (byte)8, (byte)233, (byte)128, (byte)192, (byte)67, (byte)228, (byte)250, (byte)201, (byte)198, (byte)5, (byte)177, (byte)219, (byte)255, (byte)87, (byte)32, (byte)26, (byte)226, (byte)87, (byte)26, (byte)97, (byte)207, (byte)142, (byte)164, (byte)209, (byte)236, (byte)104, (byte)164, (byte)10, (byte)23, (byte)11, (byte)25, (byte)49, (byte)64, (byte)24, (byte)175, (byte)223, (byte)232, (byte)200, (byte)217, (byte)128, (byte)216, (byte)38, (byte)33, (byte)149, (byte)122, (byte)123, (byte)10, (byte)209, (byte)218, (byte)137, (byte)146, (byte)9, (byte)31, (byte)19, (byte)223, (byte)180, (byte)60, (byte)194, (byte)105, (byte)107, (byte)59, (byte)174, (byte)14, (byte)242, (byte)16, (byte)62, (byte)226, (byte)93, (byte)117, (byte)52, (byte)190, (byte)38, (byte)100, (byte)174, (byte)222, (byte)207, (byte)125, (byte)139, (byte)62, (byte)191}));
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)217, (byte)154, (byte)224, (byte)40, (byte)82, (byte)252, (byte)158, (byte)44, (byte)67, (byte)85, (byte)237, (byte)175, (byte)211, (byte)106, (byte)179, (byte)135, (byte)85, (byte)30, (byte)194, (byte)1, (byte)1, (byte)161, (byte)90, (byte)169, (byte)28, (byte)65, (byte)187, (byte)208, (byte)97, (byte)235, (byte)114, (byte)163, (byte)41, (byte)79, (byte)84, (byte)20, (byte)221, (byte)135, (byte)21, (byte)2, (byte)134, (byte)159, (byte)144, (byte)45, (byte)119, (byte)31, (byte)162, (byte)141, (byte)238, (byte)248, (byte)58, (byte)223, (byte)242, (byte)3, (byte)52, (byte)91, (byte)221, (byte)51, (byte)241, (byte)230, (byte)38, (byte)19, (byte)154, (byte)70, (byte)69, (byte)255, (byte)209, (byte)34, (byte)71, (byte)58, (byte)110, (byte)149, (byte)121, (byte)126, (byte)6, (byte)226, (byte)164, (byte)195, (byte)218, (byte)53, (byte)188, (byte)223, (byte)90, (byte)34, (byte)248, (byte)75, (byte)151, (byte)196, (byte)219, (byte)34, (byte)244, (byte)211, (byte)17, (byte)100, (byte)213, (byte)233, (byte)218, (byte)215, (byte)16, (byte)239, (byte)102, (byte)195, (byte)13, (byte)83, (byte)79, (byte)243, (byte)13, (byte)13, (byte)34, (byte)158, (byte)248, (byte)83, (byte)116, (byte)111, (byte)185, (byte)231, (byte)120, (byte)198, (byte)9, (byte)254, (byte)173, (byte)28, (byte)200, (byte)221, (byte)196, (byte)189, (byte)48, (byte)187, (byte)115, (byte)98, (byte)7, (byte)6, (byte)255, (byte)17, (byte)159, (byte)219, (byte)49, (byte)213, (byte)94, (byte)90, (byte)244, (byte)152, (byte)120, (byte)225, (byte)101, (byte)96, (byte)114, (byte)55, (byte)185, (byte)21, (byte)173, (byte)162, (byte)45, (byte)138, (byte)248, (byte)48, (byte)152, (byte)59, (byte)149, (byte)213, (byte)165, (byte)5, (byte)185, (byte)214, (byte)22, (byte)180, (byte)207, (byte)96, (byte)113, (byte)98, (byte)51, (byte)128, (byte)232, (byte)8, (byte)233, (byte)128, (byte)192, (byte)67, (byte)228, (byte)250, (byte)201, (byte)198, (byte)5, (byte)177, (byte)219, (byte)255, (byte)87, (byte)32, (byte)26, (byte)226, (byte)87, (byte)26, (byte)97, (byte)207, (byte)142, (byte)164, (byte)209, (byte)236, (byte)104, (byte)164, (byte)10, (byte)23, (byte)11, (byte)25, (byte)49, (byte)64, (byte)24, (byte)175, (byte)223, (byte)232, (byte)200, (byte)217, (byte)128, (byte)216, (byte)38, (byte)33, (byte)149, (byte)122, (byte)123, (byte)10, (byte)209, (byte)218, (byte)137, (byte)146, (byte)9, (byte)31, (byte)19, (byte)223, (byte)180, (byte)60, (byte)194, (byte)105, (byte)107, (byte)59, (byte)174, (byte)14, (byte)242, (byte)16, (byte)62, (byte)226, (byte)93, (byte)117, (byte)52, (byte)190, (byte)38, (byte)100, (byte)174, (byte)222, (byte)207, (byte)125, (byte)139, (byte)62, (byte)191}, 0) ;
            p131.seqnr = (ushort)(ushort)40847;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance == (byte)(byte)156);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135);
                Debug.Assert(pack.min_distance == (ushort)(ushort)20033);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
                Debug.Assert(pack.id == (byte)(byte)78);
                Debug.Assert(pack.current_distance == (ushort)(ushort)51182);
                Debug.Assert(pack.max_distance == (ushort)(ushort)11324);
                Debug.Assert(pack.time_boot_ms == (uint)3036396374U);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135;
            p132.current_distance = (ushort)(ushort)51182;
            p132.max_distance = (ushort)(ushort)11324;
            p132.time_boot_ms = (uint)3036396374U;
            p132.covariance = (byte)(byte)156;
            p132.id = (byte)(byte)78;
            p132.min_distance = (ushort)(ushort)20033;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -2032780441);
                Debug.Assert(pack.lon == (int) -7306670);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)3503);
                Debug.Assert(pack.mask == (ulong)5658327520433556778L);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -2032780441;
            p133.grid_spacing = (ushort)(ushort)3503;
            p133.mask = (ulong)5658327520433556778L;
            p133.lon = (int) -7306670;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)43892);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -14987, (short) -7214, (short)7509, (short) -17437, (short)6161, (short) -13378, (short) -20139, (short)26913, (short) -22608, (short)1639, (short)10628, (short)10202, (short)24685, (short) -9111, (short) -887, (short) -29387}));
                Debug.Assert(pack.gridbit == (byte)(byte)151);
                Debug.Assert(pack.lat == (int)960319783);
                Debug.Assert(pack.lon == (int) -186121511);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)960319783;
            p134.gridbit = (byte)(byte)151;
            p134.grid_spacing = (ushort)(ushort)43892;
            p134.lon = (int) -186121511;
            p134.data__SET(new short[] {(short) -14987, (short) -7214, (short)7509, (short) -17437, (short)6161, (short) -13378, (short) -20139, (short)26913, (short) -22608, (short)1639, (short)10628, (short)10202, (short)24685, (short) -9111, (short) -887, (short) -29387}, 0) ;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1774405086);
                Debug.Assert(pack.lat == (int)870499523);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)870499523;
            p135.lon = (int) -1774405086;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_height == (float) -8.560507E37F);
                Debug.Assert(pack.pending == (ushort)(ushort)32157);
                Debug.Assert(pack.spacing == (ushort)(ushort)2216);
                Debug.Assert(pack.lat == (int) -82603507);
                Debug.Assert(pack.terrain_height == (float) -1.1959009E38F);
                Debug.Assert(pack.loaded == (ushort)(ushort)14904);
                Debug.Assert(pack.lon == (int) -1177759569);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lon = (int) -1177759569;
            p136.spacing = (ushort)(ushort)2216;
            p136.current_height = (float) -8.560507E37F;
            p136.terrain_height = (float) -1.1959009E38F;
            p136.pending = (ushort)(ushort)32157;
            p136.lat = (int) -82603507;
            p136.loaded = (ushort)(ushort)14904;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -1.8064145E38F);
                Debug.Assert(pack.temperature == (short)(short)13752);
                Debug.Assert(pack.press_diff == (float) -1.5814613E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4197808065U);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)4197808065U;
            p137.press_diff = (float) -1.5814613E38F;
            p137.press_abs = (float) -1.8064145E38F;
            p137.temperature = (short)(short)13752;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)8136568534012615051L);
                Debug.Assert(pack.x == (float)4.730851E37F);
                Debug.Assert(pack.z == (float) -2.8876598E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.3813835E38F, -2.146112E38F, 2.6971193E38F, -1.4440101E38F}));
                Debug.Assert(pack.y == (float)9.955562E37F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)8136568534012615051L;
            p138.z = (float) -2.8876598E38F;
            p138.q_SET(new float[] {-3.3813835E38F, -2.146112E38F, 2.6971193E38F, -1.4440101E38F}, 0) ;
            p138.y = (float)9.955562E37F;
            p138.x = (float)4.730851E37F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)85);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.4468325E38F, -1.2882452E38F, 7.9786446E37F, -9.380308E37F, -1.4506089E38F, 3.284576E37F, 1.740461E38F, 2.4010674E37F}));
                Debug.Assert(pack.group_mlx == (byte)(byte)167);
                Debug.Assert(pack.time_usec == (ulong)766019946597602962L);
                Debug.Assert(pack.target_system == (byte)(byte)199);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_system = (byte)(byte)199;
            p139.controls_SET(new float[] {1.4468325E38F, -1.2882452E38F, 7.9786446E37F, -9.380308E37F, -1.4506089E38F, 3.284576E37F, 1.740461E38F, 2.4010674E37F}, 0) ;
            p139.time_usec = (ulong)766019946597602962L;
            p139.target_component = (byte)(byte)85;
            p139.group_mlx = (byte)(byte)167;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)32);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.4748997E37F, -1.1656948E38F, 6.2205516E37F, -2.1521306E38F, -3.311736E38F, -2.3951169E38F, 1.822129E38F, -1.4932885E38F}));
                Debug.Assert(pack.time_usec == (ulong)19713796982678669L);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)32;
            p140.time_usec = (ulong)19713796982678669L;
            p140.controls_SET(new float[] {-1.4748997E37F, -1.1656948E38F, 6.2205516E37F, -2.1521306E38F, -3.311736E38F, -2.3951169E38F, 1.822129E38F, -1.4932885E38F}, 0) ;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_monotonic == (float)1.1469813E38F);
                Debug.Assert(pack.altitude_terrain == (float) -1.9761914E38F);
                Debug.Assert(pack.altitude_amsl == (float)2.6456067E38F);
                Debug.Assert(pack.altitude_relative == (float) -6.9586726E37F);
                Debug.Assert(pack.bottom_clearance == (float) -2.7514959E38F);
                Debug.Assert(pack.time_usec == (ulong)6740421201282807389L);
                Debug.Assert(pack.altitude_local == (float)5.3521593E37F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_amsl = (float)2.6456067E38F;
            p141.altitude_terrain = (float) -1.9761914E38F;
            p141.altitude_monotonic = (float)1.1469813E38F;
            p141.altitude_relative = (float) -6.9586726E37F;
            p141.time_usec = (ulong)6740421201282807389L;
            p141.bottom_clearance = (float) -2.7514959E38F;
            p141.altitude_local = (float)5.3521593E37F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)95, (byte)253, (byte)43, (byte)248, (byte)96, (byte)170, (byte)100, (byte)117, (byte)219, (byte)98, (byte)88, (byte)103, (byte)29, (byte)203, (byte)148, (byte)134, (byte)170, (byte)27, (byte)11, (byte)219, (byte)158, (byte)100, (byte)246, (byte)21, (byte)136, (byte)105, (byte)46, (byte)140, (byte)202, (byte)88, (byte)122, (byte)205, (byte)205, (byte)53, (byte)187, (byte)118, (byte)121, (byte)3, (byte)235, (byte)65, (byte)142, (byte)40, (byte)187, (byte)214, (byte)100, (byte)99, (byte)195, (byte)92, (byte)235, (byte)231, (byte)6, (byte)35, (byte)248, (byte)88, (byte)244, (byte)27, (byte)120, (byte)2, (byte)148, (byte)186, (byte)92, (byte)205, (byte)107, (byte)233, (byte)60, (byte)58, (byte)141, (byte)174, (byte)201, (byte)5, (byte)111, (byte)153, (byte)223, (byte)17, (byte)61, (byte)177, (byte)233, (byte)229, (byte)207, (byte)127, (byte)248, (byte)74, (byte)2, (byte)71, (byte)254, (byte)45, (byte)199, (byte)90, (byte)64, (byte)209, (byte)25, (byte)124, (byte)153, (byte)134, (byte)61, (byte)185, (byte)160, (byte)25, (byte)180, (byte)62, (byte)204, (byte)43, (byte)171, (byte)59, (byte)156, (byte)141, (byte)10, (byte)182, (byte)108, (byte)39, (byte)201, (byte)69, (byte)46, (byte)41, (byte)38, (byte)138, (byte)162, (byte)90, (byte)81, (byte)56}));
                Debug.Assert(pack.transfer_type == (byte)(byte)77);
                Debug.Assert(pack.request_id == (byte)(byte)206);
                Debug.Assert(pack.uri_type == (byte)(byte)185);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)54, (byte)125, (byte)48, (byte)50, (byte)111, (byte)184, (byte)38, (byte)67, (byte)114, (byte)128, (byte)13, (byte)243, (byte)56, (byte)162, (byte)42, (byte)52, (byte)250, (byte)236, (byte)73, (byte)125, (byte)57, (byte)211, (byte)79, (byte)15, (byte)156, (byte)30, (byte)184, (byte)69, (byte)25, (byte)226, (byte)28, (byte)125, (byte)108, (byte)39, (byte)66, (byte)15, (byte)39, (byte)102, (byte)26, (byte)249, (byte)45, (byte)113, (byte)214, (byte)217, (byte)133, (byte)112, (byte)237, (byte)21, (byte)176, (byte)96, (byte)240, (byte)184, (byte)125, (byte)166, (byte)212, (byte)139, (byte)85, (byte)68, (byte)53, (byte)140, (byte)170, (byte)158, (byte)88, (byte)36, (byte)67, (byte)232, (byte)138, (byte)189, (byte)157, (byte)82, (byte)160, (byte)37, (byte)47, (byte)40, (byte)73, (byte)135, (byte)95, (byte)98, (byte)144, (byte)131, (byte)209, (byte)14, (byte)170, (byte)173, (byte)145, (byte)221, (byte)54, (byte)24, (byte)60, (byte)30, (byte)158, (byte)175, (byte)211, (byte)82, (byte)63, (byte)109, (byte)248, (byte)92, (byte)122, (byte)71, (byte)223, (byte)66, (byte)32, (byte)95, (byte)150, (byte)148, (byte)41, (byte)133, (byte)190, (byte)231, (byte)250, (byte)204, (byte)123, (byte)174, (byte)165, (byte)40, (byte)63, (byte)142, (byte)243, (byte)162}));
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.uri_type = (byte)(byte)185;
            p142.uri_SET(new byte[] {(byte)54, (byte)125, (byte)48, (byte)50, (byte)111, (byte)184, (byte)38, (byte)67, (byte)114, (byte)128, (byte)13, (byte)243, (byte)56, (byte)162, (byte)42, (byte)52, (byte)250, (byte)236, (byte)73, (byte)125, (byte)57, (byte)211, (byte)79, (byte)15, (byte)156, (byte)30, (byte)184, (byte)69, (byte)25, (byte)226, (byte)28, (byte)125, (byte)108, (byte)39, (byte)66, (byte)15, (byte)39, (byte)102, (byte)26, (byte)249, (byte)45, (byte)113, (byte)214, (byte)217, (byte)133, (byte)112, (byte)237, (byte)21, (byte)176, (byte)96, (byte)240, (byte)184, (byte)125, (byte)166, (byte)212, (byte)139, (byte)85, (byte)68, (byte)53, (byte)140, (byte)170, (byte)158, (byte)88, (byte)36, (byte)67, (byte)232, (byte)138, (byte)189, (byte)157, (byte)82, (byte)160, (byte)37, (byte)47, (byte)40, (byte)73, (byte)135, (byte)95, (byte)98, (byte)144, (byte)131, (byte)209, (byte)14, (byte)170, (byte)173, (byte)145, (byte)221, (byte)54, (byte)24, (byte)60, (byte)30, (byte)158, (byte)175, (byte)211, (byte)82, (byte)63, (byte)109, (byte)248, (byte)92, (byte)122, (byte)71, (byte)223, (byte)66, (byte)32, (byte)95, (byte)150, (byte)148, (byte)41, (byte)133, (byte)190, (byte)231, (byte)250, (byte)204, (byte)123, (byte)174, (byte)165, (byte)40, (byte)63, (byte)142, (byte)243, (byte)162}, 0) ;
            p142.request_id = (byte)(byte)206;
            p142.storage_SET(new byte[] {(byte)95, (byte)253, (byte)43, (byte)248, (byte)96, (byte)170, (byte)100, (byte)117, (byte)219, (byte)98, (byte)88, (byte)103, (byte)29, (byte)203, (byte)148, (byte)134, (byte)170, (byte)27, (byte)11, (byte)219, (byte)158, (byte)100, (byte)246, (byte)21, (byte)136, (byte)105, (byte)46, (byte)140, (byte)202, (byte)88, (byte)122, (byte)205, (byte)205, (byte)53, (byte)187, (byte)118, (byte)121, (byte)3, (byte)235, (byte)65, (byte)142, (byte)40, (byte)187, (byte)214, (byte)100, (byte)99, (byte)195, (byte)92, (byte)235, (byte)231, (byte)6, (byte)35, (byte)248, (byte)88, (byte)244, (byte)27, (byte)120, (byte)2, (byte)148, (byte)186, (byte)92, (byte)205, (byte)107, (byte)233, (byte)60, (byte)58, (byte)141, (byte)174, (byte)201, (byte)5, (byte)111, (byte)153, (byte)223, (byte)17, (byte)61, (byte)177, (byte)233, (byte)229, (byte)207, (byte)127, (byte)248, (byte)74, (byte)2, (byte)71, (byte)254, (byte)45, (byte)199, (byte)90, (byte)64, (byte)209, (byte)25, (byte)124, (byte)153, (byte)134, (byte)61, (byte)185, (byte)160, (byte)25, (byte)180, (byte)62, (byte)204, (byte)43, (byte)171, (byte)59, (byte)156, (byte)141, (byte)10, (byte)182, (byte)108, (byte)39, (byte)201, (byte)69, (byte)46, (byte)41, (byte)38, (byte)138, (byte)162, (byte)90, (byte)81, (byte)56}, 0) ;
            p142.transfer_type = (byte)(byte)77;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float)3.1312138E38F);
                Debug.Assert(pack.press_diff == (float)1.0926406E38F);
                Debug.Assert(pack.temperature == (short)(short) -9027);
                Debug.Assert(pack.time_boot_ms == (uint)3176962277U);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.temperature = (short)(short) -9027;
            p143.press_abs = (float)3.1312138E38F;
            p143.time_boot_ms = (uint)3176962277U;
            p143.press_diff = (float)1.0926406E38F;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -114511613);
                Debug.Assert(pack.lat == (int)580234104);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-2.1166104E38F, -2.4167292E38F, -2.8656242E38F}));
                Debug.Assert(pack.timestamp == (ulong)665351761304112210L);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-2.2935487E38F, -1.0199401E38F, -1.5438348E38F, 1.5017309E38F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)144);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {4.2436462E37F, -2.8517196E38F, 1.6852707E37F}));
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {3.2100998E38F, -1.4327228E38F, 3.3175607E38F}));
                Debug.Assert(pack.alt == (float) -1.5249054E38F);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {2.0478218E38F, -3.1779917E38F, 2.8562803E38F}));
                Debug.Assert(pack.custom_state == (ulong)6762379102154327162L);
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.acc_SET(new float[] {4.2436462E37F, -2.8517196E38F, 1.6852707E37F}, 0) ;
            p144.alt = (float) -1.5249054E38F;
            p144.lat = (int)580234104;
            p144.vel_SET(new float[] {-2.1166104E38F, -2.4167292E38F, -2.8656242E38F}, 0) ;
            p144.lon = (int) -114511613;
            p144.timestamp = (ulong)665351761304112210L;
            p144.attitude_q_SET(new float[] {-2.2935487E38F, -1.0199401E38F, -1.5438348E38F, 1.5017309E38F}, 0) ;
            p144.rates_SET(new float[] {2.0478218E38F, -3.1779917E38F, 2.8562803E38F}, 0) ;
            p144.position_cov_SET(new float[] {3.2100998E38F, -1.4327228E38F, 3.3175607E38F}, 0) ;
            p144.est_capabilities = (byte)(byte)144;
            p144.custom_state = (ulong)6762379102154327162L;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.1592448E38F, 2.4588753E38F, 3.2864795E38F, 1.1878676E38F}));
                Debug.Assert(pack.x_pos == (float)1.3327445E37F);
                Debug.Assert(pack.time_usec == (ulong)6937736545915364226L);
                Debug.Assert(pack.roll_rate == (float) -1.9978297E38F);
                Debug.Assert(pack.pitch_rate == (float) -3.2683345E38F);
                Debug.Assert(pack.z_vel == (float)3.9023587E37F);
                Debug.Assert(pack.y_acc == (float) -3.0785106E38F);
                Debug.Assert(pack.z_acc == (float) -6.2549106E37F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-1.1523408E38F, -2.4536047E38F, -1.3531757E36F}));
                Debug.Assert(pack.x_vel == (float) -2.5923193E38F);
                Debug.Assert(pack.airspeed == (float)1.3434852E38F);
                Debug.Assert(pack.x_acc == (float)1.8536824E38F);
                Debug.Assert(pack.y_vel == (float) -2.1328495E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-2.648408E38F, -2.634371E38F, 1.5036293E38F}));
                Debug.Assert(pack.yaw_rate == (float) -7.0133495E37F);
                Debug.Assert(pack.z_pos == (float) -2.4920984E37F);
                Debug.Assert(pack.y_pos == (float)1.9122337E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.y_acc = (float) -3.0785106E38F;
            p146.yaw_rate = (float) -7.0133495E37F;
            p146.vel_variance_SET(new float[] {-1.1523408E38F, -2.4536047E38F, -1.3531757E36F}, 0) ;
            p146.q_SET(new float[] {2.1592448E38F, 2.4588753E38F, 3.2864795E38F, 1.1878676E38F}, 0) ;
            p146.y_vel = (float) -2.1328495E38F;
            p146.y_pos = (float)1.9122337E38F;
            p146.pos_variance_SET(new float[] {-2.648408E38F, -2.634371E38F, 1.5036293E38F}, 0) ;
            p146.x_pos = (float)1.3327445E37F;
            p146.z_acc = (float) -6.2549106E37F;
            p146.pitch_rate = (float) -3.2683345E38F;
            p146.roll_rate = (float) -1.9978297E38F;
            p146.x_vel = (float) -2.5923193E38F;
            p146.airspeed = (float)1.3434852E38F;
            p146.z_vel = (float)3.9023587E37F;
            p146.z_pos = (float) -2.4920984E37F;
            p146.x_acc = (float)1.8536824E38F;
            p146.time_usec = (ulong)6937736545915364226L;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)25007, (ushort)42505, (ushort)9270, (ushort)61829, (ushort)39460, (ushort)52977, (ushort)7601, (ushort)2026, (ushort)37937, (ushort)41277}));
                Debug.Assert(pack.temperature == (short)(short)7758);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
                Debug.Assert(pack.energy_consumed == (int)1720972425);
                Debug.Assert(pack.current_consumed == (int)116051959);
                Debug.Assert(pack.current_battery == (short)(short) -26241);
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
                Debug.Assert(pack.id == (byte)(byte)204);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)45);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.battery_remaining = (sbyte)(sbyte)45;
            p147.voltages_SET(new ushort[] {(ushort)25007, (ushort)42505, (ushort)9270, (ushort)61829, (ushort)39460, (ushort)52977, (ushort)7601, (ushort)2026, (ushort)37937, (ushort)41277}, 0) ;
            p147.current_battery = (short)(short) -26241;
            p147.energy_consumed = (int)1720972425;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            p147.current_consumed = (int)116051959;
            p147.temperature = (short)(short)7758;
            p147.id = (byte)(byte)204;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_sw_version == (uint)1568320350U);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)36415);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)115, (byte)167, (byte)78, (byte)28, (byte)71, (byte)238, (byte)8, (byte)1}));
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)56, (byte)128, (byte)166, (byte)122, (byte)168, (byte)238, (byte)227, (byte)142, (byte)72, (byte)140, (byte)9, (byte)176, (byte)45, (byte)59, (byte)171, (byte)28, (byte)242, (byte)81}));
                Debug.Assert(pack.os_sw_version == (uint)2064706752U);
                Debug.Assert(pack.uid == (ulong)7877908855820181920L);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)172, (byte)4, (byte)242, (byte)212, (byte)41, (byte)176, (byte)254, (byte)190}));
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)73, (byte)140, (byte)148, (byte)124, (byte)72, (byte)176, (byte)185, (byte)252}));
                Debug.Assert(pack.board_version == (uint)3120309973U);
                Debug.Assert(pack.middleware_sw_version == (uint)3706865668U);
                Debug.Assert(pack.product_id == (ushort)(ushort)28636);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.flight_custom_version_SET(new byte[] {(byte)73, (byte)140, (byte)148, (byte)124, (byte)72, (byte)176, (byte)185, (byte)252}, 0) ;
            p148.middleware_custom_version_SET(new byte[] {(byte)172, (byte)4, (byte)242, (byte)212, (byte)41, (byte)176, (byte)254, (byte)190}, 0) ;
            p148.board_version = (uint)3120309973U;
            p148.middleware_sw_version = (uint)3706865668U;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET;
            p148.flight_sw_version = (uint)1568320350U;
            p148.os_custom_version_SET(new byte[] {(byte)115, (byte)167, (byte)78, (byte)28, (byte)71, (byte)238, (byte)8, (byte)1}, 0) ;
            p148.os_sw_version = (uint)2064706752U;
            p148.uid = (ulong)7877908855820181920L;
            p148.vendor_id = (ushort)(ushort)36415;
            p148.product_id = (ushort)(ushort)28636;
            p148.uid2_SET(new byte[] {(byte)56, (byte)128, (byte)166, (byte)122, (byte)168, (byte)238, (byte)227, (byte)142, (byte)72, (byte)140, (byte)9, (byte)176, (byte)45, (byte)59, (byte)171, (byte)28, (byte)242, (byte)81}, 0, PH) ;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.angle_y == (float)2.805888E37F);
                Debug.Assert(pack.time_usec == (ulong)833524127816765928L);
                Debug.Assert(pack.angle_x == (float) -7.4634825E37F);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
                Debug.Assert(pack.z_TRY(ph) == (float) -2.9508062E38F);
                Debug.Assert(pack.y_TRY(ph) == (float) -7.473913E37F);
                Debug.Assert(pack.size_y == (float) -1.0684524E38F);
                Debug.Assert(pack.target_num == (byte)(byte)204);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {1.4324465E38F, -1.1080769E38F, -3.3952993E38F, 5.497145E37F}));
                Debug.Assert(pack.distance == (float) -1.0071087E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)53);
                Debug.Assert(pack.size_x == (float) -1.8645132E38F);
                Debug.Assert(pack.x_TRY(ph) == (float)1.7485333E38F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.x_SET((float)1.7485333E38F, PH) ;
            p149.target_num = (byte)(byte)204;
            p149.q_SET(new float[] {1.4324465E38F, -1.1080769E38F, -3.3952993E38F, 5.497145E37F}, 0, PH) ;
            p149.size_y = (float) -1.0684524E38F;
            p149.position_valid_SET((byte)(byte)53, PH) ;
            p149.angle_x = (float) -7.4634825E37F;
            p149.y_SET((float) -7.473913E37F, PH) ;
            p149.z_SET((float) -2.9508062E38F, PH) ;
            p149.distance = (float) -1.0071087E38F;
            p149.size_x = (float) -1.8645132E38F;
            p149.time_usec = (ulong)833524127816765928L;
            p149.angle_y = (float)2.805888E37F;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)9195);
                Debug.Assert(pack.target_component == (byte)(byte)163);
                Debug.Assert(pack.target_system == (byte)(byte)109);
                Debug.Assert(pack.name_LEN(ph) == 12);
                Debug.Assert(pack.name_TRY(ph).Equals("aoslvobuwNho"));
            };
            GroundControl.SCRIPT_ITEM p180 = CommunicationChannel.new_SCRIPT_ITEM();
            PH.setPack(p180);
            p180.target_system = (byte)(byte)109;
            p180.target_component = (byte)(byte)163;
            p180.seq = (ushort)(ushort)9195;
            p180.name_SET("aoslvobuwNho", PH) ;
            CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)13213);
                Debug.Assert(pack.target_component == (byte)(byte)67);
                Debug.Assert(pack.target_system == (byte)(byte)21);
            };
            GroundControl.SCRIPT_REQUEST p181 = CommunicationChannel.new_SCRIPT_REQUEST();
            PH.setPack(p181);
            p181.target_component = (byte)(byte)67;
            p181.seq = (ushort)(ushort)13213;
            p181.target_system = (byte)(byte)21;
            CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)193);
                Debug.Assert(pack.target_system == (byte)(byte)80);
            };
            GroundControl.SCRIPT_REQUEST_LIST p182 = CommunicationChannel.new_SCRIPT_REQUEST_LIST();
            PH.setPack(p182);
            p182.target_system = (byte)(byte)80;
            p182.target_component = (byte)(byte)193;
            CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)229);
                Debug.Assert(pack.count == (ushort)(ushort)31654);
                Debug.Assert(pack.target_component == (byte)(byte)119);
            };
            GroundControl.SCRIPT_COUNT p183 = CommunicationChannel.new_SCRIPT_COUNT();
            PH.setPack(p183);
            p183.count = (ushort)(ushort)31654;
            p183.target_component = (byte)(byte)119;
            p183.target_system = (byte)(byte)229;
            CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)9267);
            };
            GroundControl.SCRIPT_CURRENT p184 = CommunicationChannel.new_SCRIPT_CURRENT();
            PH.setPack(p184);
            p184.seq = (ushort)(ushort)9267;
            CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_vert_ratio == (float) -1.0759041E37F);
                Debug.Assert(pack.tas_ratio == (float)3.3275565E38F);
                Debug.Assert(pack.hagl_ratio == (float)1.4379925E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float) -3.33237E38F);
                Debug.Assert(pack.time_usec == (ulong)9000851900518355660L);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS);
                Debug.Assert(pack.pos_horiz_accuracy == (float) -3.3518964E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float)1.0320682E38F);
                Debug.Assert(pack.vel_ratio == (float) -2.72348E37F);
                Debug.Assert(pack.mag_ratio == (float) -2.4322651E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)9000851900518355660L;
            p230.hagl_ratio = (float)1.4379925E38F;
            p230.pos_horiz_accuracy = (float) -3.3518964E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS;
            p230.tas_ratio = (float)3.3275565E38F;
            p230.pos_vert_accuracy = (float) -3.33237E38F;
            p230.vel_ratio = (float) -2.72348E37F;
            p230.pos_vert_ratio = (float) -1.0759041E37F;
            p230.pos_horiz_ratio = (float)1.0320682E38F;
            p230.mag_ratio = (float) -2.4322651E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.var_horiz == (float)1.6207262E38F);
                Debug.Assert(pack.wind_z == (float) -4.8291865E37F);
                Debug.Assert(pack.var_vert == (float)2.4272095E38F);
                Debug.Assert(pack.wind_alt == (float)2.8426049E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -1.987672E38F);
                Debug.Assert(pack.wind_y == (float) -2.9283309E38F);
                Debug.Assert(pack.wind_x == (float)2.7863358E38F);
                Debug.Assert(pack.vert_accuracy == (float)5.608171E37F);
                Debug.Assert(pack.time_usec == (ulong)1541833236358028749L);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.wind_x = (float)2.7863358E38F;
            p231.vert_accuracy = (float)5.608171E37F;
            p231.wind_alt = (float)2.8426049E38F;
            p231.time_usec = (ulong)1541833236358028749L;
            p231.var_horiz = (float)1.6207262E38F;
            p231.wind_z = (float) -4.8291865E37F;
            p231.var_vert = (float)2.4272095E38F;
            p231.wind_y = (float) -2.9283309E38F;
            p231.horiz_accuracy = (float) -1.987672E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vert_accuracy == (float) -2.9731153E38F);
                Debug.Assert(pack.time_week_ms == (uint)2548525845U);
                Debug.Assert(pack.gps_id == (byte)(byte)10);
                Debug.Assert(pack.speed_accuracy == (float)1.6946423E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)61958);
                Debug.Assert(pack.hdop == (float)2.459981E38F);
                Debug.Assert(pack.lat == (int) -860584394);
                Debug.Assert(pack.horiz_accuracy == (float)6.5300146E37F);
                Debug.Assert(pack.fix_type == (byte)(byte)150);
                Debug.Assert(pack.ve == (float) -1.1090949E38F);
                Debug.Assert(pack.alt == (float)1.4249719E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
                Debug.Assert(pack.lon == (int)561694316);
                Debug.Assert(pack.vdop == (float) -1.4077693E38F);
                Debug.Assert(pack.time_usec == (ulong)1284518180071719588L);
                Debug.Assert(pack.vn == (float) -9.808507E37F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)40);
                Debug.Assert(pack.vd == (float) -4.47275E37F);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.vert_accuracy = (float) -2.9731153E38F;
            p232.gps_id = (byte)(byte)10;
            p232.satellites_visible = (byte)(byte)40;
            p232.hdop = (float)2.459981E38F;
            p232.speed_accuracy = (float)1.6946423E38F;
            p232.time_week_ms = (uint)2548525845U;
            p232.alt = (float)1.4249719E38F;
            p232.ve = (float) -1.1090949E38F;
            p232.vdop = (float) -1.4077693E38F;
            p232.horiz_accuracy = (float)6.5300146E37F;
            p232.lat = (int) -860584394;
            p232.time_week = (ushort)(ushort)61958;
            p232.time_usec = (ulong)1284518180071719588L;
            p232.vd = (float) -4.47275E37F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT;
            p232.vn = (float) -9.808507E37F;
            p232.lon = (int)561694316;
            p232.fix_type = (byte)(byte)150;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)36);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)147, (byte)227, (byte)49, (byte)130, (byte)2, (byte)17, (byte)192, (byte)122, (byte)164, (byte)65, (byte)135, (byte)116, (byte)33, (byte)44, (byte)119, (byte)39, (byte)57, (byte)164, (byte)124, (byte)92, (byte)47, (byte)32, (byte)1, (byte)75, (byte)187, (byte)115, (byte)37, (byte)9, (byte)162, (byte)129, (byte)34, (byte)144, (byte)6, (byte)222, (byte)80, (byte)6, (byte)100, (byte)73, (byte)16, (byte)242, (byte)7, (byte)41, (byte)57, (byte)172, (byte)94, (byte)90, (byte)19, (byte)221, (byte)233, (byte)44, (byte)80, (byte)147, (byte)174, (byte)223, (byte)196, (byte)26, (byte)189, (byte)234, (byte)59, (byte)1, (byte)58, (byte)89, (byte)201, (byte)60, (byte)222, (byte)167, (byte)39, (byte)38, (byte)109, (byte)100, (byte)191, (byte)89, (byte)172, (byte)64, (byte)89, (byte)69, (byte)65, (byte)101, (byte)20, (byte)95, (byte)182, (byte)167, (byte)109, (byte)179, (byte)58, (byte)252, (byte)234, (byte)105, (byte)209, (byte)177, (byte)11, (byte)161, (byte)52, (byte)198, (byte)247, (byte)34, (byte)72, (byte)86, (byte)59, (byte)170, (byte)77, (byte)11, (byte)191, (byte)179, (byte)83, (byte)150, (byte)123, (byte)88, (byte)156, (byte)209, (byte)253, (byte)216, (byte)18, (byte)184, (byte)84, (byte)40, (byte)162, (byte)27, (byte)241, (byte)253, (byte)149, (byte)23, (byte)132, (byte)180, (byte)184, (byte)44, (byte)106, (byte)253, (byte)73, (byte)106, (byte)134, (byte)39, (byte)168, (byte)242, (byte)40, (byte)235, (byte)6, (byte)45, (byte)231, (byte)113, (byte)38, (byte)182, (byte)143, (byte)242, (byte)151, (byte)222, (byte)184, (byte)130, (byte)20, (byte)199, (byte)177, (byte)185, (byte)161, (byte)44, (byte)192, (byte)91, (byte)164, (byte)27, (byte)47, (byte)97, (byte)78, (byte)123, (byte)127, (byte)124, (byte)43, (byte)113, (byte)150, (byte)72, (byte)64, (byte)3, (byte)191, (byte)52, (byte)185, (byte)111, (byte)149, (byte)68, (byte)193, (byte)117, (byte)72, (byte)236}));
                Debug.Assert(pack.flags == (byte)(byte)81);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.data__SET(new byte[] {(byte)147, (byte)227, (byte)49, (byte)130, (byte)2, (byte)17, (byte)192, (byte)122, (byte)164, (byte)65, (byte)135, (byte)116, (byte)33, (byte)44, (byte)119, (byte)39, (byte)57, (byte)164, (byte)124, (byte)92, (byte)47, (byte)32, (byte)1, (byte)75, (byte)187, (byte)115, (byte)37, (byte)9, (byte)162, (byte)129, (byte)34, (byte)144, (byte)6, (byte)222, (byte)80, (byte)6, (byte)100, (byte)73, (byte)16, (byte)242, (byte)7, (byte)41, (byte)57, (byte)172, (byte)94, (byte)90, (byte)19, (byte)221, (byte)233, (byte)44, (byte)80, (byte)147, (byte)174, (byte)223, (byte)196, (byte)26, (byte)189, (byte)234, (byte)59, (byte)1, (byte)58, (byte)89, (byte)201, (byte)60, (byte)222, (byte)167, (byte)39, (byte)38, (byte)109, (byte)100, (byte)191, (byte)89, (byte)172, (byte)64, (byte)89, (byte)69, (byte)65, (byte)101, (byte)20, (byte)95, (byte)182, (byte)167, (byte)109, (byte)179, (byte)58, (byte)252, (byte)234, (byte)105, (byte)209, (byte)177, (byte)11, (byte)161, (byte)52, (byte)198, (byte)247, (byte)34, (byte)72, (byte)86, (byte)59, (byte)170, (byte)77, (byte)11, (byte)191, (byte)179, (byte)83, (byte)150, (byte)123, (byte)88, (byte)156, (byte)209, (byte)253, (byte)216, (byte)18, (byte)184, (byte)84, (byte)40, (byte)162, (byte)27, (byte)241, (byte)253, (byte)149, (byte)23, (byte)132, (byte)180, (byte)184, (byte)44, (byte)106, (byte)253, (byte)73, (byte)106, (byte)134, (byte)39, (byte)168, (byte)242, (byte)40, (byte)235, (byte)6, (byte)45, (byte)231, (byte)113, (byte)38, (byte)182, (byte)143, (byte)242, (byte)151, (byte)222, (byte)184, (byte)130, (byte)20, (byte)199, (byte)177, (byte)185, (byte)161, (byte)44, (byte)192, (byte)91, (byte)164, (byte)27, (byte)47, (byte)97, (byte)78, (byte)123, (byte)127, (byte)124, (byte)43, (byte)113, (byte)150, (byte)72, (byte)64, (byte)3, (byte)191, (byte)52, (byte)185, (byte)111, (byte)149, (byte)68, (byte)193, (byte)117, (byte)72, (byte)236}, 0) ;
            p233.len = (byte)(byte)36;
            p233.flags = (byte)(byte)81;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 104);
                Debug.Assert(pack.heading_sp == (short)(short) -13975);
                Debug.Assert(pack.groundspeed == (byte)(byte)66);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.heading == (ushort)(ushort)37041);
                Debug.Assert(pack.altitude_sp == (short)(short)4724);
                Debug.Assert(pack.pitch == (short)(short)23393);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
                Debug.Assert(pack.airspeed == (byte)(byte)189);
                Debug.Assert(pack.roll == (short)(short) -17755);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)18);
                Debug.Assert(pack.longitude == (int) -1719492027);
                Debug.Assert(pack.custom_mode == (uint)1483982473U);
                Debug.Assert(pack.latitude == (int)887684394);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)17);
                Debug.Assert(pack.gps_nsat == (byte)(byte)42);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)48);
                Debug.Assert(pack.altitude_amsl == (short)(short)9591);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)32400);
                Debug.Assert(pack.failsafe == (byte)(byte)17);
                Debug.Assert(pack.battery_remaining == (byte)(byte)239);
                Debug.Assert(pack.wp_num == (byte)(byte)81);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)26);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.gps_nsat = (byte)(byte)42;
            p234.battery_remaining = (byte)(byte)239;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p234.temperature = (sbyte)(sbyte) - 104;
            p234.roll = (short)(short) -17755;
            p234.airspeed_sp = (byte)(byte)17;
            p234.custom_mode = (uint)1483982473U;
            p234.heading_sp = (short)(short) -13975;
            p234.altitude_sp = (short)(short)4724;
            p234.pitch = (short)(short)23393;
            p234.failsafe = (byte)(byte)17;
            p234.wp_distance = (ushort)(ushort)32400;
            p234.altitude_amsl = (short)(short)9591;
            p234.climb_rate = (sbyte)(sbyte)18;
            p234.wp_num = (byte)(byte)81;
            p234.groundspeed = (byte)(byte)66;
            p234.heading = (ushort)(ushort)37041;
            p234.airspeed = (byte)(byte)189;
            p234.throttle = (sbyte)(sbyte)26;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED;
            p234.longitude = (int) -1719492027;
            p234.latitude = (int)887684394;
            p234.temperature_air = (sbyte)(sbyte)48;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_0 == (uint)2672191681U);
                Debug.Assert(pack.clipping_1 == (uint)2003767506U);
                Debug.Assert(pack.clipping_2 == (uint)1441159405U);
                Debug.Assert(pack.vibration_z == (float) -2.6520311E38F);
                Debug.Assert(pack.vibration_x == (float)6.344793E36F);
                Debug.Assert(pack.vibration_y == (float) -1.826898E38F);
                Debug.Assert(pack.time_usec == (ulong)7705768592850928450L);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_1 = (uint)2003767506U;
            p241.clipping_2 = (uint)1441159405U;
            p241.vibration_y = (float) -1.826898E38F;
            p241.clipping_0 = (uint)2672191681U;
            p241.time_usec = (ulong)7705768592850928450L;
            p241.vibration_x = (float)6.344793E36F;
            p241.vibration_z = (float) -2.6520311E38F;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)2.5828627E37F);
                Debug.Assert(pack.approach_z == (float) -2.9524773E38F);
                Debug.Assert(pack.x == (float)2.2470835E38F);
                Debug.Assert(pack.altitude == (int) -2146862595);
                Debug.Assert(pack.approach_y == (float) -3.2504431E38F);
                Debug.Assert(pack.approach_x == (float) -2.3764528E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {8.497489E35F, 3.6337499E37F, 2.8934161E38F, -2.3052228E38F}));
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)5077739140841799841L);
                Debug.Assert(pack.latitude == (int) -1929645224);
                Debug.Assert(pack.longitude == (int) -106027561);
                Debug.Assert(pack.z == (float)1.0519798E38F);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.z = (float)1.0519798E38F;
            p242.altitude = (int) -2146862595;
            p242.y = (float)2.5828627E37F;
            p242.x = (float)2.2470835E38F;
            p242.time_usec_SET((ulong)5077739140841799841L, PH) ;
            p242.longitude = (int) -106027561;
            p242.approach_y = (float) -3.2504431E38F;
            p242.approach_z = (float) -2.9524773E38F;
            p242.q_SET(new float[] {8.497489E35F, 3.6337499E37F, 2.8934161E38F, -2.3052228E38F}, 0) ;
            p242.approach_x = (float) -2.3764528E38F;
            p242.latitude = (int) -1929645224;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {8.965328E37F, -3.2389489E38F, 1.6645543E38F, -2.5066607E37F}));
                Debug.Assert(pack.altitude == (int) -1524189905);
                Debug.Assert(pack.approach_z == (float) -1.0584347E38F);
                Debug.Assert(pack.y == (float) -1.9609985E38F);
                Debug.Assert(pack.x == (float) -1.1049531E38F);
                Debug.Assert(pack.longitude == (int)225834641);
                Debug.Assert(pack.approach_x == (float) -3.011563E38F);
                Debug.Assert(pack.latitude == (int)863200762);
                Debug.Assert(pack.z == (float)2.6623074E38F);
                Debug.Assert(pack.target_system == (byte)(byte)114);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)8847765276306763376L);
                Debug.Assert(pack.approach_y == (float) -2.5589825E38F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.approach_z = (float) -1.0584347E38F;
            p243.x = (float) -1.1049531E38F;
            p243.latitude = (int)863200762;
            p243.approach_y = (float) -2.5589825E38F;
            p243.target_system = (byte)(byte)114;
            p243.q_SET(new float[] {8.965328E37F, -3.2389489E38F, 1.6645543E38F, -2.5066607E37F}, 0) ;
            p243.z = (float)2.6623074E38F;
            p243.longitude = (int)225834641;
            p243.approach_x = (float) -3.011563E38F;
            p243.altitude = (int) -1524189905;
            p243.time_usec_SET((ulong)8847765276306763376L, PH) ;
            p243.y = (float) -1.9609985E38F;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)52905);
                Debug.Assert(pack.interval_us == (int)845941601);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int)845941601;
            p244.message_id = (ushort)(ushort)52905;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.callsign_LEN(ph) == 2);
                Debug.Assert(pack.callsign_TRY(ph).Equals("cn"));
                Debug.Assert(pack.lat == (int) -1646235576);
                Debug.Assert(pack.tslc == (byte)(byte)81);
                Debug.Assert(pack.lon == (int)891003412);
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED2);
                Debug.Assert(pack.ver_velocity == (short)(short) -7340);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN);
                Debug.Assert(pack.ICAO_address == (uint)2058993657U);
                Debug.Assert(pack.squawk == (ushort)(ushort)45620);
                Debug.Assert(pack.altitude == (int) -2022128580);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.heading == (ushort)(ushort)37575);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)25115);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.tslc = (byte)(byte)81;
            p246.ICAO_address = (uint)2058993657U;
            p246.lat = (int) -1646235576;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.squawk = (ushort)(ushort)45620;
            p246.hor_velocity = (ushort)(ushort)25115;
            p246.heading = (ushort)(ushort)37575;
            p246.callsign_SET("cn", PH) ;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED2;
            p246.lon = (int)891003412;
            p246.altitude = (int) -2022128580;
            p246.ver_velocity = (short)(short) -7340;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horizontal_minimum_delta == (float)8.46518E37F);
                Debug.Assert(pack.time_to_minimum_delta == (float) -2.067513E38F);
                Debug.Assert(pack.id == (uint)2887956332U);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
                Debug.Assert(pack.altitude_minimum_delta == (float) -2.7829867E38F);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY;
            p247.time_to_minimum_delta = (float) -2.067513E38F;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.horizontal_minimum_delta = (float)8.46518E37F;
            p247.id = (uint)2887956332U;
            p247.altitude_minimum_delta = (float) -2.7829867E38F;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)143);
                Debug.Assert(pack.message_type == (ushort)(ushort)49583);
                Debug.Assert(pack.target_network == (byte)(byte)142);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)108, (byte)206, (byte)193, (byte)199, (byte)160, (byte)168, (byte)212, (byte)174, (byte)35, (byte)56, (byte)247, (byte)12, (byte)86, (byte)123, (byte)244, (byte)122, (byte)33, (byte)2, (byte)120, (byte)17, (byte)149, (byte)55, (byte)48, (byte)158, (byte)137, (byte)165, (byte)38, (byte)159, (byte)4, (byte)236, (byte)204, (byte)60, (byte)164, (byte)14, (byte)220, (byte)149, (byte)181, (byte)67, (byte)188, (byte)197, (byte)13, (byte)252, (byte)73, (byte)181, (byte)63, (byte)75, (byte)134, (byte)235, (byte)62, (byte)241, (byte)71, (byte)109, (byte)136, (byte)123, (byte)35, (byte)98, (byte)9, (byte)124, (byte)223, (byte)35, (byte)82, (byte)229, (byte)39, (byte)38, (byte)2, (byte)117, (byte)179, (byte)19, (byte)53, (byte)73, (byte)59, (byte)143, (byte)218, (byte)68, (byte)118, (byte)8, (byte)233, (byte)200, (byte)89, (byte)144, (byte)241, (byte)115, (byte)221, (byte)169, (byte)2, (byte)141, (byte)145, (byte)189, (byte)191, (byte)97, (byte)125, (byte)220, (byte)171, (byte)144, (byte)12, (byte)20, (byte)100, (byte)24, (byte)220, (byte)249, (byte)66, (byte)71, (byte)34, (byte)198, (byte)145, (byte)140, (byte)33, (byte)203, (byte)208, (byte)109, (byte)116, (byte)210, (byte)249, (byte)24, (byte)62, (byte)147, (byte)247, (byte)4, (byte)219, (byte)110, (byte)29, (byte)79, (byte)23, (byte)164, (byte)119, (byte)5, (byte)28, (byte)74, (byte)9, (byte)214, (byte)202, (byte)195, (byte)177, (byte)231, (byte)102, (byte)53, (byte)127, (byte)156, (byte)236, (byte)200, (byte)150, (byte)65, (byte)121, (byte)152, (byte)35, (byte)211, (byte)250, (byte)86, (byte)219, (byte)199, (byte)115, (byte)238, (byte)119, (byte)251, (byte)40, (byte)186, (byte)128, (byte)106, (byte)75, (byte)73, (byte)7, (byte)172, (byte)130, (byte)172, (byte)69, (byte)175, (byte)98, (byte)37, (byte)164, (byte)185, (byte)217, (byte)99, (byte)66, (byte)140, (byte)135, (byte)101, (byte)26, (byte)5, (byte)218, (byte)172, (byte)9, (byte)216, (byte)141, (byte)204, (byte)119, (byte)166, (byte)190, (byte)37, (byte)245, (byte)184, (byte)174, (byte)5, (byte)158, (byte)22, (byte)193, (byte)100, (byte)89, (byte)16, (byte)191, (byte)178, (byte)163, (byte)179, (byte)172, (byte)47, (byte)68, (byte)75, (byte)251, (byte)44, (byte)15, (byte)236, (byte)86, (byte)173, (byte)153, (byte)218, (byte)64, (byte)183, (byte)215, (byte)181, (byte)235, (byte)166, (byte)168, (byte)170, (byte)229, (byte)239, (byte)112, (byte)228, (byte)12, (byte)34, (byte)78, (byte)58, (byte)101, (byte)248, (byte)14, (byte)255, (byte)52, (byte)114, (byte)109, (byte)9, (byte)51, (byte)211, (byte)8, (byte)173, (byte)82, (byte)15, (byte)191, (byte)192, (byte)174, (byte)62, (byte)75}));
                Debug.Assert(pack.target_system == (byte)(byte)11);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)142;
            p248.message_type = (ushort)(ushort)49583;
            p248.target_system = (byte)(byte)11;
            p248.target_component = (byte)(byte)143;
            p248.payload_SET(new byte[] {(byte)108, (byte)206, (byte)193, (byte)199, (byte)160, (byte)168, (byte)212, (byte)174, (byte)35, (byte)56, (byte)247, (byte)12, (byte)86, (byte)123, (byte)244, (byte)122, (byte)33, (byte)2, (byte)120, (byte)17, (byte)149, (byte)55, (byte)48, (byte)158, (byte)137, (byte)165, (byte)38, (byte)159, (byte)4, (byte)236, (byte)204, (byte)60, (byte)164, (byte)14, (byte)220, (byte)149, (byte)181, (byte)67, (byte)188, (byte)197, (byte)13, (byte)252, (byte)73, (byte)181, (byte)63, (byte)75, (byte)134, (byte)235, (byte)62, (byte)241, (byte)71, (byte)109, (byte)136, (byte)123, (byte)35, (byte)98, (byte)9, (byte)124, (byte)223, (byte)35, (byte)82, (byte)229, (byte)39, (byte)38, (byte)2, (byte)117, (byte)179, (byte)19, (byte)53, (byte)73, (byte)59, (byte)143, (byte)218, (byte)68, (byte)118, (byte)8, (byte)233, (byte)200, (byte)89, (byte)144, (byte)241, (byte)115, (byte)221, (byte)169, (byte)2, (byte)141, (byte)145, (byte)189, (byte)191, (byte)97, (byte)125, (byte)220, (byte)171, (byte)144, (byte)12, (byte)20, (byte)100, (byte)24, (byte)220, (byte)249, (byte)66, (byte)71, (byte)34, (byte)198, (byte)145, (byte)140, (byte)33, (byte)203, (byte)208, (byte)109, (byte)116, (byte)210, (byte)249, (byte)24, (byte)62, (byte)147, (byte)247, (byte)4, (byte)219, (byte)110, (byte)29, (byte)79, (byte)23, (byte)164, (byte)119, (byte)5, (byte)28, (byte)74, (byte)9, (byte)214, (byte)202, (byte)195, (byte)177, (byte)231, (byte)102, (byte)53, (byte)127, (byte)156, (byte)236, (byte)200, (byte)150, (byte)65, (byte)121, (byte)152, (byte)35, (byte)211, (byte)250, (byte)86, (byte)219, (byte)199, (byte)115, (byte)238, (byte)119, (byte)251, (byte)40, (byte)186, (byte)128, (byte)106, (byte)75, (byte)73, (byte)7, (byte)172, (byte)130, (byte)172, (byte)69, (byte)175, (byte)98, (byte)37, (byte)164, (byte)185, (byte)217, (byte)99, (byte)66, (byte)140, (byte)135, (byte)101, (byte)26, (byte)5, (byte)218, (byte)172, (byte)9, (byte)216, (byte)141, (byte)204, (byte)119, (byte)166, (byte)190, (byte)37, (byte)245, (byte)184, (byte)174, (byte)5, (byte)158, (byte)22, (byte)193, (byte)100, (byte)89, (byte)16, (byte)191, (byte)178, (byte)163, (byte)179, (byte)172, (byte)47, (byte)68, (byte)75, (byte)251, (byte)44, (byte)15, (byte)236, (byte)86, (byte)173, (byte)153, (byte)218, (byte)64, (byte)183, (byte)215, (byte)181, (byte)235, (byte)166, (byte)168, (byte)170, (byte)229, (byte)239, (byte)112, (byte)228, (byte)12, (byte)34, (byte)78, (byte)58, (byte)101, (byte)248, (byte)14, (byte)255, (byte)52, (byte)114, (byte)109, (byte)9, (byte)51, (byte)211, (byte)8, (byte)173, (byte)82, (byte)15, (byte)191, (byte)192, (byte)174, (byte)62, (byte)75}, 0) ;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)253);
                Debug.Assert(pack.ver == (byte)(byte)56);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 48, (sbyte)106, (sbyte) - 4, (sbyte)86, (sbyte)72, (sbyte) - 106, (sbyte)15, (sbyte)30, (sbyte) - 97, (sbyte) - 50, (sbyte) - 82, (sbyte)76, (sbyte)5, (sbyte)69, (sbyte)29, (sbyte)91, (sbyte)126, (sbyte)68, (sbyte)101, (sbyte)14, (sbyte) - 123, (sbyte)123, (sbyte)38, (sbyte)19, (sbyte) - 95, (sbyte) - 4, (sbyte) - 22, (sbyte) - 84, (sbyte) - 40, (sbyte) - 6, (sbyte) - 51, (sbyte)88}));
                Debug.Assert(pack.address == (ushort)(ushort)56898);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.ver = (byte)(byte)56;
            p249.type = (byte)(byte)253;
            p249.address = (ushort)(ushort)56898;
            p249.value_SET(new sbyte[] {(sbyte) - 48, (sbyte)106, (sbyte) - 4, (sbyte)86, (sbyte)72, (sbyte) - 106, (sbyte)15, (sbyte)30, (sbyte) - 97, (sbyte) - 50, (sbyte) - 82, (sbyte)76, (sbyte)5, (sbyte)69, (sbyte)29, (sbyte)91, (sbyte)126, (sbyte)68, (sbyte)101, (sbyte)14, (sbyte) - 123, (sbyte)123, (sbyte)38, (sbyte)19, (sbyte) - 95, (sbyte) - 4, (sbyte) - 22, (sbyte) - 84, (sbyte) - 40, (sbyte) - 6, (sbyte) - 51, (sbyte)88}, 0) ;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)4.1291525E37F);
                Debug.Assert(pack.x == (float) -2.3760975E38F);
                Debug.Assert(pack.time_usec == (ulong)1899244321119218375L);
                Debug.Assert(pack.name_LEN(ph) == 2);
                Debug.Assert(pack.name_TRY(ph).Equals("iq"));
                Debug.Assert(pack.z == (float)2.4487403E38F);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.y = (float)4.1291525E37F;
            p250.time_usec = (ulong)1899244321119218375L;
            p250.name_SET("iq", PH) ;
            p250.x = (float) -2.3760975E38F;
            p250.z = (float)2.4487403E38F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float)1.7095629E37F);
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("naUaqedycg"));
                Debug.Assert(pack.time_boot_ms == (uint)1630978595U);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.value = (float)1.7095629E37F;
            p251.time_boot_ms = (uint)1630978595U;
            p251.name_SET("naUaqedycg", PH) ;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (int)1974158474);
                Debug.Assert(pack.name_LEN(ph) == 5);
                Debug.Assert(pack.name_TRY(ph).Equals("fybtx"));
                Debug.Assert(pack.time_boot_ms == (uint)2806958428U);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.value = (int)1974158474;
            p252.time_boot_ms = (uint)2806958428U;
            p252.name_SET("fybtx", PH) ;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 46);
                Debug.Assert(pack.text_TRY(ph).Equals("jmbdcreedvtddbvbqhzgNggwzAwvmvFwubyqjanizeppcl"));
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("jmbdcreedvtddbvbqhzgNggwzAwvmvFwubyqjanizeppcl", PH) ;
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_CRITICAL;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1777235091U);
                Debug.Assert(pack.ind == (byte)(byte)150);
                Debug.Assert(pack.value == (float)2.1137796E38F);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.value = (float)2.1137796E38F;
            p254.ind = (byte)(byte)150;
            p254.time_boot_ms = (uint)1777235091U;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)96, (byte)168, (byte)80, (byte)255, (byte)19, (byte)196, (byte)121, (byte)230, (byte)32, (byte)36, (byte)14, (byte)149, (byte)255, (byte)3, (byte)197, (byte)75, (byte)166, (byte)38, (byte)165, (byte)157, (byte)30, (byte)113, (byte)178, (byte)154, (byte)147, (byte)234, (byte)194, (byte)133, (byte)75, (byte)115, (byte)163, (byte)32}));
                Debug.Assert(pack.initial_timestamp == (ulong)2729317810975113006L);
                Debug.Assert(pack.target_system == (byte)(byte)216);
                Debug.Assert(pack.target_component == (byte)(byte)97);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.initial_timestamp = (ulong)2729317810975113006L;
            p256.secret_key_SET(new byte[] {(byte)96, (byte)168, (byte)80, (byte)255, (byte)19, (byte)196, (byte)121, (byte)230, (byte)32, (byte)36, (byte)14, (byte)149, (byte)255, (byte)3, (byte)197, (byte)75, (byte)166, (byte)38, (byte)165, (byte)157, (byte)30, (byte)113, (byte)178, (byte)154, (byte)147, (byte)234, (byte)194, (byte)133, (byte)75, (byte)115, (byte)163, (byte)32}, 0) ;
            p256.target_system = (byte)(byte)216;
            p256.target_component = (byte)(byte)97;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_change_ms == (uint)907119037U);
                Debug.Assert(pack.time_boot_ms == (uint)3764834672U);
                Debug.Assert(pack.state == (byte)(byte)80);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)3764834672U;
            p257.last_change_ms = (uint)907119037U;
            p257.state = (byte)(byte)80;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)225);
                Debug.Assert(pack.target_component == (byte)(byte)219);
                Debug.Assert(pack.tune_LEN(ph) == 3);
                Debug.Assert(pack.tune_TRY(ph).Equals("hzg"));
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_component = (byte)(byte)219;
            p258.tune_SET("hzg", PH) ;
            p258.target_system = (byte)(byte)225;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2473798275U);
                Debug.Assert(pack.sensor_size_h == (float) -2.469875E38F);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)11573);
                Debug.Assert(pack.firmware_version == (uint)2394192924U);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)26804);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)7, (byte)124, (byte)53, (byte)40, (byte)101, (byte)12, (byte)148, (byte)157, (byte)180, (byte)213, (byte)124, (byte)138, (byte)67, (byte)221, (byte)0, (byte)58, (byte)148, (byte)158, (byte)242, (byte)141, (byte)234, (byte)40, (byte)165, (byte)210, (byte)66, (byte)73, (byte)54, (byte)98, (byte)247, (byte)221, (byte)254, (byte)223}));
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)68, (byte)6, (byte)107, (byte)254, (byte)143, (byte)253, (byte)193, (byte)54, (byte)110, (byte)216, (byte)178, (byte)157, (byte)106, (byte)92, (byte)138, (byte)22, (byte)143, (byte)211, (byte)48, (byte)68, (byte)103, (byte)18, (byte)19, (byte)218, (byte)130, (byte)126, (byte)78, (byte)137, (byte)85, (byte)152, (byte)173, (byte)105}));
                Debug.Assert(pack.lens_id == (byte)(byte)186);
                Debug.Assert(pack.sensor_size_v == (float) -2.6685672E38F);
                Debug.Assert(pack.focal_length == (float)2.8640551E38F);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)50823);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 126);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("oxgcagzzajnmStDeyexyfemaAkkmmdxkTtqejzxrhzoihAcmfweZyzldxzKAaFjlhchrdlWZhgheuerqkfomJthbnqZqvxywmydbykcqmxppdhkqayyacbyucajIzC"));
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.resolution_h = (ushort)(ushort)26804;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE;
            p259.cam_definition_uri_SET("oxgcagzzajnmStDeyexyfemaAkkmmdxkTtqejzxrhzoihAcmfweZyzldxzKAaFjlhchrdlWZhgheuerqkfomJthbnqZqvxywmydbykcqmxppdhkqayyacbyucajIzC", PH) ;
            p259.focal_length = (float)2.8640551E38F;
            p259.vendor_name_SET(new byte[] {(byte)7, (byte)124, (byte)53, (byte)40, (byte)101, (byte)12, (byte)148, (byte)157, (byte)180, (byte)213, (byte)124, (byte)138, (byte)67, (byte)221, (byte)0, (byte)58, (byte)148, (byte)158, (byte)242, (byte)141, (byte)234, (byte)40, (byte)165, (byte)210, (byte)66, (byte)73, (byte)54, (byte)98, (byte)247, (byte)221, (byte)254, (byte)223}, 0) ;
            p259.lens_id = (byte)(byte)186;
            p259.resolution_v = (ushort)(ushort)11573;
            p259.sensor_size_h = (float) -2.469875E38F;
            p259.sensor_size_v = (float) -2.6685672E38F;
            p259.firmware_version = (uint)2394192924U;
            p259.time_boot_ms = (uint)2473798275U;
            p259.model_name_SET(new byte[] {(byte)68, (byte)6, (byte)107, (byte)254, (byte)143, (byte)253, (byte)193, (byte)54, (byte)110, (byte)216, (byte)178, (byte)157, (byte)106, (byte)92, (byte)138, (byte)22, (byte)143, (byte)211, (byte)48, (byte)68, (byte)103, (byte)18, (byte)19, (byte)218, (byte)130, (byte)126, (byte)78, (byte)137, (byte)85, (byte)152, (byte)173, (byte)105}, 0) ;
            p259.cam_definition_version = (ushort)(ushort)50823;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE);
                Debug.Assert(pack.time_boot_ms == (uint)1493526739U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE;
            p260.time_boot_ms = (uint)1493526739U;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage_count == (byte)(byte)56);
                Debug.Assert(pack.time_boot_ms == (uint)4000657787U);
                Debug.Assert(pack.status == (byte)(byte)107);
                Debug.Assert(pack.used_capacity == (float) -3.2879263E38F);
                Debug.Assert(pack.read_speed == (float) -3.0893527E38F);
                Debug.Assert(pack.available_capacity == (float)2.7615024E38F);
                Debug.Assert(pack.total_capacity == (float) -2.2723722E38F);
                Debug.Assert(pack.write_speed == (float) -8.2209687E37F);
                Debug.Assert(pack.storage_id == (byte)(byte)77);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.total_capacity = (float) -2.2723722E38F;
            p261.available_capacity = (float)2.7615024E38F;
            p261.time_boot_ms = (uint)4000657787U;
            p261.storage_count = (byte)(byte)56;
            p261.write_speed = (float) -8.2209687E37F;
            p261.read_speed = (float) -3.0893527E38F;
            p261.used_capacity = (float) -3.2879263E38F;
            p261.storage_id = (byte)(byte)77;
            p261.status = (byte)(byte)107;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1712823190U);
                Debug.Assert(pack.image_status == (byte)(byte)222);
                Debug.Assert(pack.recording_time_ms == (uint)3435587471U);
                Debug.Assert(pack.video_status == (byte)(byte)226);
                Debug.Assert(pack.image_interval == (float) -2.8751165E38F);
                Debug.Assert(pack.available_capacity == (float) -1.9918115E38F);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.video_status = (byte)(byte)226;
            p262.time_boot_ms = (uint)1712823190U;
            p262.available_capacity = (float) -1.9918115E38F;
            p262.image_status = (byte)(byte)222;
            p262.recording_time_ms = (uint)3435587471U;
            p262.image_interval = (float) -2.8751165E38F;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)50);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.1916807E38F, 1.0279594E38F, -8.394576E37F, 3.2638228E38F}));
                Debug.Assert(pack.relative_alt == (int)906783109);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)86);
                Debug.Assert(pack.alt == (int) -1658996473);
                Debug.Assert(pack.image_index == (int) -349528178);
                Debug.Assert(pack.time_boot_ms == (uint)2143763889U);
                Debug.Assert(pack.time_utc == (ulong)7917663290690078570L);
                Debug.Assert(pack.lat == (int) -1250655339);
                Debug.Assert(pack.file_url_LEN(ph) == 18);
                Debug.Assert(pack.file_url_TRY(ph).Equals("eybOgyedxiqzofhQac"));
                Debug.Assert(pack.lon == (int)1033486451);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.alt = (int) -1658996473;
            p263.capture_result = (sbyte)(sbyte)86;
            p263.time_utc = (ulong)7917663290690078570L;
            p263.camera_id = (byte)(byte)50;
            p263.q_SET(new float[] {3.1916807E38F, 1.0279594E38F, -8.394576E37F, 3.2638228E38F}, 0) ;
            p263.image_index = (int) -349528178;
            p263.lat = (int) -1250655339;
            p263.lon = (int)1033486451;
            p263.relative_alt = (int)906783109;
            p263.file_url_SET("eybOgyedxiqzofhQac", PH) ;
            p263.time_boot_ms = (uint)2143763889U;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.arming_time_utc == (ulong)2236175703036893194L);
                Debug.Assert(pack.time_boot_ms == (uint)1044648177U);
                Debug.Assert(pack.flight_uuid == (ulong)7509686230920342538L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)5361370565498576719L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)1044648177U;
            p264.takeoff_time_utc = (ulong)5361370565498576719L;
            p264.arming_time_utc = (ulong)2236175703036893194L;
            p264.flight_uuid = (ulong)7509686230920342538L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -6.1142236E37F);
                Debug.Assert(pack.roll == (float) -1.5262155E38F);
                Debug.Assert(pack.pitch == (float)2.2190483E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1652907899U);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.roll = (float) -1.5262155E38F;
            p265.time_boot_ms = (uint)1652907899U;
            p265.yaw = (float) -6.1142236E37F;
            p265.pitch = (float)2.2190483E38F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)61143);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)112, (byte)124, (byte)54, (byte)251, (byte)219, (byte)196, (byte)190, (byte)74, (byte)58, (byte)138, (byte)101, (byte)14, (byte)179, (byte)6, (byte)205, (byte)223, (byte)57, (byte)169, (byte)41, (byte)164, (byte)200, (byte)8, (byte)107, (byte)2, (byte)59, (byte)72, (byte)201, (byte)154, (byte)118, (byte)36, (byte)177, (byte)14, (byte)202, (byte)43, (byte)82, (byte)48, (byte)158, (byte)39, (byte)66, (byte)213, (byte)29, (byte)35, (byte)12, (byte)114, (byte)97, (byte)163, (byte)231, (byte)219, (byte)149, (byte)215, (byte)122, (byte)37, (byte)143, (byte)119, (byte)190, (byte)238, (byte)174, (byte)177, (byte)102, (byte)187, (byte)18, (byte)23, (byte)194, (byte)4, (byte)188, (byte)109, (byte)129, (byte)23, (byte)108, (byte)134, (byte)171, (byte)193, (byte)162, (byte)245, (byte)144, (byte)241, (byte)3, (byte)99, (byte)205, (byte)78, (byte)39, (byte)51, (byte)42, (byte)149, (byte)134, (byte)78, (byte)222, (byte)158, (byte)181, (byte)222, (byte)18, (byte)145, (byte)217, (byte)141, (byte)159, (byte)37, (byte)80, (byte)91, (byte)75, (byte)120, (byte)136, (byte)208, (byte)124, (byte)28, (byte)220, (byte)249, (byte)103, (byte)238, (byte)231, (byte)102, (byte)190, (byte)10, (byte)94, (byte)83, (byte)73, (byte)60, (byte)167, (byte)221, (byte)200, (byte)224, (byte)185, (byte)42, (byte)80, (byte)226, (byte)13, (byte)141, (byte)197, (byte)193, (byte)162, (byte)33, (byte)119, (byte)117, (byte)7, (byte)20, (byte)0, (byte)177, (byte)65, (byte)74, (byte)150, (byte)35, (byte)231, (byte)93, (byte)246, (byte)159, (byte)72, (byte)24, (byte)146, (byte)204, (byte)223, (byte)179, (byte)242, (byte)171, (byte)228, (byte)229, (byte)90, (byte)166, (byte)158, (byte)9, (byte)37, (byte)231, (byte)43, (byte)136, (byte)99, (byte)219, (byte)191, (byte)245, (byte)80, (byte)91, (byte)106, (byte)202, (byte)67, (byte)89, (byte)165, (byte)239, (byte)47, (byte)48, (byte)238, (byte)213, (byte)145, (byte)186, (byte)160, (byte)246, (byte)193, (byte)99, (byte)24, (byte)59, (byte)106, (byte)120, (byte)209, (byte)65, (byte)77, (byte)28, (byte)122, (byte)0, (byte)185, (byte)229, (byte)165, (byte)14, (byte)27, (byte)184, (byte)33, (byte)195, (byte)103, (byte)167, (byte)61, (byte)169, (byte)118, (byte)108, (byte)245, (byte)146, (byte)229, (byte)49, (byte)84, (byte)49, (byte)234, (byte)124, (byte)10, (byte)69, (byte)174, (byte)39, (byte)246, (byte)198, (byte)143, (byte)232, (byte)172, (byte)34, (byte)150, (byte)87, (byte)188, (byte)218, (byte)237, (byte)28, (byte)1, (byte)8, (byte)221, (byte)78, (byte)34, (byte)37, (byte)172, (byte)92, (byte)97, (byte)144, (byte)208, (byte)191, (byte)106, (byte)164, (byte)49, (byte)2, (byte)0}));
                Debug.Assert(pack.target_component == (byte)(byte)64);
                Debug.Assert(pack.target_system == (byte)(byte)165);
                Debug.Assert(pack.first_message_offset == (byte)(byte)74);
                Debug.Assert(pack.length == (byte)(byte)161);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.first_message_offset = (byte)(byte)74;
            p266.data__SET(new byte[] {(byte)112, (byte)124, (byte)54, (byte)251, (byte)219, (byte)196, (byte)190, (byte)74, (byte)58, (byte)138, (byte)101, (byte)14, (byte)179, (byte)6, (byte)205, (byte)223, (byte)57, (byte)169, (byte)41, (byte)164, (byte)200, (byte)8, (byte)107, (byte)2, (byte)59, (byte)72, (byte)201, (byte)154, (byte)118, (byte)36, (byte)177, (byte)14, (byte)202, (byte)43, (byte)82, (byte)48, (byte)158, (byte)39, (byte)66, (byte)213, (byte)29, (byte)35, (byte)12, (byte)114, (byte)97, (byte)163, (byte)231, (byte)219, (byte)149, (byte)215, (byte)122, (byte)37, (byte)143, (byte)119, (byte)190, (byte)238, (byte)174, (byte)177, (byte)102, (byte)187, (byte)18, (byte)23, (byte)194, (byte)4, (byte)188, (byte)109, (byte)129, (byte)23, (byte)108, (byte)134, (byte)171, (byte)193, (byte)162, (byte)245, (byte)144, (byte)241, (byte)3, (byte)99, (byte)205, (byte)78, (byte)39, (byte)51, (byte)42, (byte)149, (byte)134, (byte)78, (byte)222, (byte)158, (byte)181, (byte)222, (byte)18, (byte)145, (byte)217, (byte)141, (byte)159, (byte)37, (byte)80, (byte)91, (byte)75, (byte)120, (byte)136, (byte)208, (byte)124, (byte)28, (byte)220, (byte)249, (byte)103, (byte)238, (byte)231, (byte)102, (byte)190, (byte)10, (byte)94, (byte)83, (byte)73, (byte)60, (byte)167, (byte)221, (byte)200, (byte)224, (byte)185, (byte)42, (byte)80, (byte)226, (byte)13, (byte)141, (byte)197, (byte)193, (byte)162, (byte)33, (byte)119, (byte)117, (byte)7, (byte)20, (byte)0, (byte)177, (byte)65, (byte)74, (byte)150, (byte)35, (byte)231, (byte)93, (byte)246, (byte)159, (byte)72, (byte)24, (byte)146, (byte)204, (byte)223, (byte)179, (byte)242, (byte)171, (byte)228, (byte)229, (byte)90, (byte)166, (byte)158, (byte)9, (byte)37, (byte)231, (byte)43, (byte)136, (byte)99, (byte)219, (byte)191, (byte)245, (byte)80, (byte)91, (byte)106, (byte)202, (byte)67, (byte)89, (byte)165, (byte)239, (byte)47, (byte)48, (byte)238, (byte)213, (byte)145, (byte)186, (byte)160, (byte)246, (byte)193, (byte)99, (byte)24, (byte)59, (byte)106, (byte)120, (byte)209, (byte)65, (byte)77, (byte)28, (byte)122, (byte)0, (byte)185, (byte)229, (byte)165, (byte)14, (byte)27, (byte)184, (byte)33, (byte)195, (byte)103, (byte)167, (byte)61, (byte)169, (byte)118, (byte)108, (byte)245, (byte)146, (byte)229, (byte)49, (byte)84, (byte)49, (byte)234, (byte)124, (byte)10, (byte)69, (byte)174, (byte)39, (byte)246, (byte)198, (byte)143, (byte)232, (byte)172, (byte)34, (byte)150, (byte)87, (byte)188, (byte)218, (byte)237, (byte)28, (byte)1, (byte)8, (byte)221, (byte)78, (byte)34, (byte)37, (byte)172, (byte)92, (byte)97, (byte)144, (byte)208, (byte)191, (byte)106, (byte)164, (byte)49, (byte)2, (byte)0}, 0) ;
            p266.sequence = (ushort)(ushort)61143;
            p266.target_component = (byte)(byte)64;
            p266.target_system = (byte)(byte)165;
            p266.length = (byte)(byte)161;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.length == (byte)(byte)169);
                Debug.Assert(pack.target_component == (byte)(byte)245);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)156, (byte)239, (byte)235, (byte)205, (byte)135, (byte)91, (byte)98, (byte)177, (byte)212, (byte)131, (byte)47, (byte)28, (byte)64, (byte)136, (byte)190, (byte)113, (byte)163, (byte)193, (byte)205, (byte)206, (byte)100, (byte)182, (byte)13, (byte)165, (byte)168, (byte)158, (byte)239, (byte)186, (byte)12, (byte)81, (byte)236, (byte)76, (byte)39, (byte)234, (byte)199, (byte)123, (byte)157, (byte)251, (byte)128, (byte)21, (byte)182, (byte)240, (byte)21, (byte)251, (byte)103, (byte)131, (byte)3, (byte)229, (byte)127, (byte)189, (byte)19, (byte)78, (byte)176, (byte)121, (byte)112, (byte)198, (byte)17, (byte)6, (byte)155, (byte)201, (byte)11, (byte)86, (byte)1, (byte)211, (byte)105, (byte)72, (byte)165, (byte)20, (byte)67, (byte)1, (byte)85, (byte)40, (byte)106, (byte)195, (byte)57, (byte)108, (byte)175, (byte)150, (byte)183, (byte)134, (byte)112, (byte)46, (byte)26, (byte)85, (byte)68, (byte)73, (byte)28, (byte)113, (byte)180, (byte)237, (byte)194, (byte)196, (byte)222, (byte)61, (byte)119, (byte)69, (byte)12, (byte)87, (byte)30, (byte)176, (byte)242, (byte)230, (byte)25, (byte)231, (byte)148, (byte)128, (byte)200, (byte)5, (byte)46, (byte)151, (byte)79, (byte)92, (byte)241, (byte)185, (byte)69, (byte)192, (byte)220, (byte)166, (byte)181, (byte)181, (byte)15, (byte)52, (byte)134, (byte)190, (byte)152, (byte)219, (byte)234, (byte)247, (byte)121, (byte)21, (byte)161, (byte)158, (byte)106, (byte)19, (byte)60, (byte)224, (byte)104, (byte)172, (byte)77, (byte)45, (byte)194, (byte)189, (byte)242, (byte)51, (byte)142, (byte)78, (byte)255, (byte)34, (byte)186, (byte)125, (byte)156, (byte)105, (byte)165, (byte)213, (byte)214, (byte)177, (byte)129, (byte)17, (byte)144, (byte)162, (byte)126, (byte)104, (byte)66, (byte)50, (byte)19, (byte)222, (byte)40, (byte)87, (byte)139, (byte)157, (byte)227, (byte)3, (byte)11, (byte)77, (byte)54, (byte)42, (byte)172, (byte)101, (byte)126, (byte)28, (byte)130, (byte)197, (byte)18, (byte)252, (byte)34, (byte)245, (byte)11, (byte)11, (byte)211, (byte)95, (byte)35, (byte)106, (byte)36, (byte)53, (byte)113, (byte)194, (byte)123, (byte)139, (byte)220, (byte)129, (byte)228, (byte)202, (byte)132, (byte)22, (byte)168, (byte)62, (byte)198, (byte)219, (byte)148, (byte)210, (byte)198, (byte)94, (byte)40, (byte)200, (byte)200, (byte)92, (byte)114, (byte)249, (byte)204, (byte)149, (byte)10, (byte)127, (byte)70, (byte)226, (byte)1, (byte)8, (byte)38, (byte)139, (byte)17, (byte)243, (byte)121, (byte)79, (byte)162, (byte)56, (byte)200, (byte)36, (byte)48, (byte)206, (byte)168, (byte)43, (byte)163, (byte)214, (byte)170, (byte)69, (byte)244, (byte)92, (byte)190, (byte)85, (byte)194}));
                Debug.Assert(pack.sequence == (ushort)(ushort)7978);
                Debug.Assert(pack.first_message_offset == (byte)(byte)4);
                Debug.Assert(pack.target_system == (byte)(byte)245);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.first_message_offset = (byte)(byte)4;
            p267.data__SET(new byte[] {(byte)156, (byte)239, (byte)235, (byte)205, (byte)135, (byte)91, (byte)98, (byte)177, (byte)212, (byte)131, (byte)47, (byte)28, (byte)64, (byte)136, (byte)190, (byte)113, (byte)163, (byte)193, (byte)205, (byte)206, (byte)100, (byte)182, (byte)13, (byte)165, (byte)168, (byte)158, (byte)239, (byte)186, (byte)12, (byte)81, (byte)236, (byte)76, (byte)39, (byte)234, (byte)199, (byte)123, (byte)157, (byte)251, (byte)128, (byte)21, (byte)182, (byte)240, (byte)21, (byte)251, (byte)103, (byte)131, (byte)3, (byte)229, (byte)127, (byte)189, (byte)19, (byte)78, (byte)176, (byte)121, (byte)112, (byte)198, (byte)17, (byte)6, (byte)155, (byte)201, (byte)11, (byte)86, (byte)1, (byte)211, (byte)105, (byte)72, (byte)165, (byte)20, (byte)67, (byte)1, (byte)85, (byte)40, (byte)106, (byte)195, (byte)57, (byte)108, (byte)175, (byte)150, (byte)183, (byte)134, (byte)112, (byte)46, (byte)26, (byte)85, (byte)68, (byte)73, (byte)28, (byte)113, (byte)180, (byte)237, (byte)194, (byte)196, (byte)222, (byte)61, (byte)119, (byte)69, (byte)12, (byte)87, (byte)30, (byte)176, (byte)242, (byte)230, (byte)25, (byte)231, (byte)148, (byte)128, (byte)200, (byte)5, (byte)46, (byte)151, (byte)79, (byte)92, (byte)241, (byte)185, (byte)69, (byte)192, (byte)220, (byte)166, (byte)181, (byte)181, (byte)15, (byte)52, (byte)134, (byte)190, (byte)152, (byte)219, (byte)234, (byte)247, (byte)121, (byte)21, (byte)161, (byte)158, (byte)106, (byte)19, (byte)60, (byte)224, (byte)104, (byte)172, (byte)77, (byte)45, (byte)194, (byte)189, (byte)242, (byte)51, (byte)142, (byte)78, (byte)255, (byte)34, (byte)186, (byte)125, (byte)156, (byte)105, (byte)165, (byte)213, (byte)214, (byte)177, (byte)129, (byte)17, (byte)144, (byte)162, (byte)126, (byte)104, (byte)66, (byte)50, (byte)19, (byte)222, (byte)40, (byte)87, (byte)139, (byte)157, (byte)227, (byte)3, (byte)11, (byte)77, (byte)54, (byte)42, (byte)172, (byte)101, (byte)126, (byte)28, (byte)130, (byte)197, (byte)18, (byte)252, (byte)34, (byte)245, (byte)11, (byte)11, (byte)211, (byte)95, (byte)35, (byte)106, (byte)36, (byte)53, (byte)113, (byte)194, (byte)123, (byte)139, (byte)220, (byte)129, (byte)228, (byte)202, (byte)132, (byte)22, (byte)168, (byte)62, (byte)198, (byte)219, (byte)148, (byte)210, (byte)198, (byte)94, (byte)40, (byte)200, (byte)200, (byte)92, (byte)114, (byte)249, (byte)204, (byte)149, (byte)10, (byte)127, (byte)70, (byte)226, (byte)1, (byte)8, (byte)38, (byte)139, (byte)17, (byte)243, (byte)121, (byte)79, (byte)162, (byte)56, (byte)200, (byte)36, (byte)48, (byte)206, (byte)168, (byte)43, (byte)163, (byte)214, (byte)170, (byte)69, (byte)244, (byte)92, (byte)190, (byte)85, (byte)194}, 0) ;
            p267.target_system = (byte)(byte)245;
            p267.sequence = (ushort)(ushort)7978;
            p267.target_component = (byte)(byte)245;
            p267.length = (byte)(byte)169;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)192);
                Debug.Assert(pack.sequence == (ushort)(ushort)52414);
                Debug.Assert(pack.target_component == (byte)(byte)14);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.sequence = (ushort)(ushort)52414;
            p268.target_component = (byte)(byte)14;
            p268.target_system = (byte)(byte)192;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.framerate == (float)3.176939E38F);
                Debug.Assert(pack.rotation == (ushort)(ushort)24377);
                Debug.Assert(pack.camera_id == (byte)(byte)42);
                Debug.Assert(pack.status == (byte)(byte)37);
                Debug.Assert(pack.uri_LEN(ph) == 155);
                Debug.Assert(pack.uri_TRY(ph).Equals("iwteryebijaoypumbwacvbkZqxnppkeagilfuvjmsbavlnSiuasxjdoisbfyaQjfUrxpcpUfuwYgIcefzuwqdeAxxmfvHwrzhrrgaafrjpvinmnnwieUqeyqkwgzaisdisceakdAccgknbjoghFtvXhskgw"));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)34220);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)35570);
                Debug.Assert(pack.bitrate == (uint)1529495250U);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.resolution_h = (ushort)(ushort)35570;
            p269.rotation = (ushort)(ushort)24377;
            p269.camera_id = (byte)(byte)42;
            p269.uri_SET("iwteryebijaoypumbwacvbkZqxnppkeagilfuvjmsbavlnSiuasxjdoisbfyaQjfUrxpcpUfuwYgIcefzuwqdeAxxmfvHwrzhrrgaafrjpvinmnnwieUqeyqkwgzaisdisceakdAccgknbjoghFtvXhskgw", PH) ;
            p269.status = (byte)(byte)37;
            p269.framerate = (float)3.176939E38F;
            p269.bitrate = (uint)1529495250U;
            p269.resolution_v = (ushort)(ushort)34220;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)150);
                Debug.Assert(pack.uri_LEN(ph) == 214);
                Debug.Assert(pack.uri_TRY(ph).Equals("tohptzrbgdYnonsopvlPomoimTfeccmxYwllIpuzCskqxdsydvgdsapjlUvWvifhzlnzvkBlegrizvufpglhuNwnardyLpYvijxfmbyuzkYlyCsctfqDxksffEaflqjaJwjfkwsomkfghkcphhxdanawetYAfilljxndkbzghodvlAZyrRfctvslkfgjcsZlFjhbrhlqnfnoaenufdsbnf"));
                Debug.Assert(pack.rotation == (ushort)(ushort)41718);
                Debug.Assert(pack.framerate == (float) -1.25313E38F);
                Debug.Assert(pack.bitrate == (uint)2389376920U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)51655);
                Debug.Assert(pack.target_system == (byte)(byte)119);
                Debug.Assert(pack.camera_id == (byte)(byte)69);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)51009);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.uri_SET("tohptzrbgdYnonsopvlPomoimTfeccmxYwllIpuzCskqxdsydvgdsapjlUvWvifhzlnzvkBlegrizvufpglhuNwnardyLpYvijxfmbyuzkYlyCsctfqDxksffEaflqjaJwjfkwsomkfghkcphhxdanawetYAfilljxndkbzghodvlAZyrRfctvslkfgjcsZlFjhbrhlqnfnoaenufdsbnf", PH) ;
            p270.camera_id = (byte)(byte)69;
            p270.resolution_h = (ushort)(ushort)51009;
            p270.target_system = (byte)(byte)119;
            p270.resolution_v = (ushort)(ushort)51655;
            p270.framerate = (float) -1.25313E38F;
            p270.rotation = (ushort)(ushort)41718;
            p270.target_component = (byte)(byte)150;
            p270.bitrate = (uint)2389376920U;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ssid_LEN(ph) == 31);
                Debug.Assert(pack.ssid_TRY(ph).Equals("RawyrmabqdbuEkthawxfjfxjhmyhaeL"));
                Debug.Assert(pack.password_LEN(ph) == 3);
                Debug.Assert(pack.password_TRY(ph).Equals("gyd"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("gyd", PH) ;
            p299.ssid_SET("RawyrmabqdbuEkthawxfjfxjhmyhaeL", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)139, (byte)138, (byte)83, (byte)21, (byte)191, (byte)133, (byte)3, (byte)1}));
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)226, (byte)132, (byte)58, (byte)96, (byte)241, (byte)12, (byte)118, (byte)48}));
                Debug.Assert(pack.max_version == (ushort)(ushort)3551);
                Debug.Assert(pack.version == (ushort)(ushort)41870);
                Debug.Assert(pack.min_version == (ushort)(ushort)42055);
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.max_version = (ushort)(ushort)3551;
            p300.min_version = (ushort)(ushort)42055;
            p300.library_version_hash_SET(new byte[] {(byte)139, (byte)138, (byte)83, (byte)21, (byte)191, (byte)133, (byte)3, (byte)1}, 0) ;
            p300.version = (ushort)(ushort)41870;
            p300.spec_version_hash_SET(new byte[] {(byte)226, (byte)132, (byte)58, (byte)96, (byte)241, (byte)12, (byte)118, (byte)48}, 0) ;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sub_mode == (byte)(byte)212);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)64713);
                Debug.Assert(pack.time_usec == (ulong)2361627062250385367L);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
                Debug.Assert(pack.uptime_sec == (uint)3262938863U);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            p310.time_usec = (ulong)2361627062250385367L;
            p310.vendor_specific_status_code = (ushort)(ushort)64713;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE;
            p310.sub_mode = (byte)(byte)212;
            p310.uptime_sec = (uint)3262938863U;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 43);
                Debug.Assert(pack.name_TRY(ph).Equals("qjjrmygsyyqoqujEtcoksxrxmhvqszpnepvbTrwedsf"));
                Debug.Assert(pack.uptime_sec == (uint)4100812606U);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)106);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)21, (byte)213, (byte)211, (byte)253, (byte)208, (byte)193, (byte)100, (byte)28, (byte)98, (byte)129, (byte)34, (byte)100, (byte)164, (byte)243, (byte)86, (byte)85}));
                Debug.Assert(pack.time_usec == (ulong)5865109378541232922L);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)182);
                Debug.Assert(pack.sw_vcs_commit == (uint)2154911662U);
                Debug.Assert(pack.hw_version_major == (byte)(byte)162);
                Debug.Assert(pack.sw_version_major == (byte)(byte)236);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_version_minor = (byte)(byte)182;
            p311.time_usec = (ulong)5865109378541232922L;
            p311.hw_version_major = (byte)(byte)162;
            p311.sw_version_minor = (byte)(byte)106;
            p311.sw_version_major = (byte)(byte)236;
            p311.name_SET("qjjrmygsyyqoqujEtcoksxrxmhvqszpnepvbTrwedsf", PH) ;
            p311.sw_vcs_commit = (uint)2154911662U;
            p311.hw_unique_id_SET(new byte[] {(byte)21, (byte)213, (byte)211, (byte)253, (byte)208, (byte)193, (byte)100, (byte)28, (byte)98, (byte)129, (byte)34, (byte)100, (byte)164, (byte)243, (byte)86, (byte)85}, 0) ;
            p311.uptime_sec = (uint)4100812606U;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("pwbwSdqXnhrebncu"));
                Debug.Assert(pack.param_index == (short)(short)7639);
                Debug.Assert(pack.target_system == (byte)(byte)235);
                Debug.Assert(pack.target_component == (byte)(byte)214);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_component = (byte)(byte)214;
            p320.param_index = (short)(short)7639;
            p320.param_id_SET("pwbwSdqXnhrebncu", PH) ;
            p320.target_system = (byte)(byte)235;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)64);
                Debug.Assert(pack.target_system == (byte)(byte)102);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)64;
            p321.target_system = (byte)(byte)102;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("hbpsc"));
                Debug.Assert(pack.param_count == (ushort)(ushort)20248);
                Debug.Assert(pack.param_index == (ushort)(ushort)29835);
                Debug.Assert(pack.param_value_LEN(ph) == 8);
                Debug.Assert(pack.param_value_TRY(ph).Equals("tzncherM"));
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_count = (ushort)(ushort)20248;
            p322.param_value_SET("tzncherM", PH) ;
            p322.param_id_SET("hbpsc", PH) ;
            p322.param_index = (ushort)(ushort)29835;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 31);
                Debug.Assert(pack.param_value_TRY(ph).Equals("iwmnqkvsxhrwlodWadxyhiEifiuithk"));
                Debug.Assert(pack.target_component == (byte)(byte)80);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("okWbvgNzgr"));
                Debug.Assert(pack.target_system == (byte)(byte)246);
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_value_SET("iwmnqkvsxhrwlodWadxyhiEifiuithk", PH) ;
            p323.target_component = (byte)(byte)80;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p323.target_system = (byte)(byte)246;
            p323.param_id_SET("okWbvgNzgr", PH) ;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 86);
                Debug.Assert(pack.param_value_TRY(ph).Equals("lvrddydvInbfBziwlCoqftrDihyaipuiWijjdMnjdczqtiqwqbupezsxxxVxdnxwrpfxvRemrndhndohnZlwHe"));
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("fzikx"));
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_ACCEPTED);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("fzikx", PH) ;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_ACCEPTED;
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16;
            p324.param_value_SET("lvrddydvInbfBziwlCoqftrDihyaipuiWijjdMnjdczqtiqwqbupezsxxxVxdnxwrpfxvRemrndhndohnZlwHe", PH) ;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_distance == (ushort)(ushort)47934);
                Debug.Assert(pack.time_usec == (ulong)712780979131697777L);
                Debug.Assert(pack.max_distance == (ushort)(ushort)52621);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)6936, (ushort)26856, (ushort)26815, (ushort)11974, (ushort)35040, (ushort)6917, (ushort)44042, (ushort)18338, (ushort)61087, (ushort)7531, (ushort)19734, (ushort)48022, (ushort)33165, (ushort)51040, (ushort)7426, (ushort)6658, (ushort)28808, (ushort)8764, (ushort)52778, (ushort)62108, (ushort)38638, (ushort)49196, (ushort)55771, (ushort)23984, (ushort)59986, (ushort)7336, (ushort)38947, (ushort)18360, (ushort)57845, (ushort)6218, (ushort)43489, (ushort)63350, (ushort)21264, (ushort)58487, (ushort)59313, (ushort)33153, (ushort)22207, (ushort)4086, (ushort)9497, (ushort)39091, (ushort)8394, (ushort)60602, (ushort)20598, (ushort)45226, (ushort)3354, (ushort)43364, (ushort)14026, (ushort)53653, (ushort)47986, (ushort)36865, (ushort)10839, (ushort)56344, (ushort)3342, (ushort)62095, (ushort)42298, (ushort)47116, (ushort)22327, (ushort)43018, (ushort)57173, (ushort)44927, (ushort)40990, (ushort)29379, (ushort)62046, (ushort)47404, (ushort)46178, (ushort)48700, (ushort)5753, (ushort)3624, (ushort)20166, (ushort)14479, (ushort)1737, (ushort)21441}));
                Debug.Assert(pack.increment == (byte)(byte)19);
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.min_distance = (ushort)(ushort)47934;
            p330.distances_SET(new ushort[] {(ushort)6936, (ushort)26856, (ushort)26815, (ushort)11974, (ushort)35040, (ushort)6917, (ushort)44042, (ushort)18338, (ushort)61087, (ushort)7531, (ushort)19734, (ushort)48022, (ushort)33165, (ushort)51040, (ushort)7426, (ushort)6658, (ushort)28808, (ushort)8764, (ushort)52778, (ushort)62108, (ushort)38638, (ushort)49196, (ushort)55771, (ushort)23984, (ushort)59986, (ushort)7336, (ushort)38947, (ushort)18360, (ushort)57845, (ushort)6218, (ushort)43489, (ushort)63350, (ushort)21264, (ushort)58487, (ushort)59313, (ushort)33153, (ushort)22207, (ushort)4086, (ushort)9497, (ushort)39091, (ushort)8394, (ushort)60602, (ushort)20598, (ushort)45226, (ushort)3354, (ushort)43364, (ushort)14026, (ushort)53653, (ushort)47986, (ushort)36865, (ushort)10839, (ushort)56344, (ushort)3342, (ushort)62095, (ushort)42298, (ushort)47116, (ushort)22327, (ushort)43018, (ushort)57173, (ushort)44927, (ushort)40990, (ushort)29379, (ushort)62046, (ushort)47404, (ushort)46178, (ushort)48700, (ushort)5753, (ushort)3624, (ushort)20166, (ushort)14479, (ushort)1737, (ushort)21441}, 0) ;
            p330.max_distance = (ushort)(ushort)52621;
            p330.time_usec = (ulong)712780979131697777L;
            p330.increment = (byte)(byte)19;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
        }
    }
}
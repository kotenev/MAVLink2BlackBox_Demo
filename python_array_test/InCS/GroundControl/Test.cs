
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
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 7, data, 260);
                }
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
        new class ARRAY_TEST_0 : GroundControl.ARRAY_TEST_0
        {
            public ushort[] ar_u16 //Value array
            {
                get {return ar_u16_GET(new ushort[4], 0);}
            }
            public ushort[]ar_u16_GET(ushort[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public uint[] ar_u32 //Value array
            {
                get {return ar_u32_GET(new uint[4], 0);}
            }
            public uint[]ar_u32_GET(uint[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 8, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public byte v1 //Stub field
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  24, 1));}
            }

            public sbyte[] ar_i8 //Value array
            {
                get {return ar_i8_GET(new sbyte[4], 0);}
            }
            public sbyte[]ar_i8_GET(sbyte[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 25, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (sbyte)((sbyte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public byte[] ar_u8 //Value array
            {
                get {return ar_u8_GET(new byte[4], 0);}
            }
            public byte[]ar_u8_GET(byte[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 29, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class ARRAY_TEST_1 : GroundControl.ARRAY_TEST_1
        {
            public uint[] ar_u32 //Value array
            {
                get {return ar_u32_GET(new uint[4], 0);}
            }
            public uint[]ar_u32_GET(uint[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
        }
        new class ARRAY_TEST_3 : GroundControl.ARRAY_TEST_3
        {
            public uint[] ar_u32 //Value array
            {
                get {return ar_u32_GET(new uint[4], 0);}
            }
            public uint[]ar_u32_GET(uint[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public byte v //Stub field
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
            }
        }
        new class ARRAY_TEST_4 : GroundControl.ARRAY_TEST_4
        {
            public uint[] ar_u32 //Value array
            {
                get {return ar_u32_GET(new uint[4], 0);}
            }
            public uint[]ar_u32_GET(uint[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public byte v //Stub field
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
            }
        }
        new class ARRAY_TEST_5 : GroundControl.ARRAY_TEST_5
        {
            public string c1_TRY(Inside ph)//Value array
            {
                if(ph.field_bit !=  0 && !try_visit_field(ph, 0)  ||  !try_visit_item(ph, 0)) return null;
                return new string(c1_GET(ph, new char[ph.items], 0));
            }
            public char[]c1_GET(Inside ph, char[] dst_ch, int pos) //Value array
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int c1_LEN(Inside ph)
            {
                return (ph.field_bit !=  0 && !try_visit_field(ph, 0)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public string c2_TRY(Inside ph)//Value array
            {
                if(ph.field_bit !=  1 && !try_visit_field(ph, 1)  ||  !try_visit_item(ph, 0)) return null;
                return new string(c2_GET(ph, new char[ph.items], 0));
            }
            public char[]c2_GET(Inside ph, char[] dst_ch, int pos) //Value array
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int c2_LEN(Inside ph)
            {
                return (ph.field_bit !=  1 && !try_visit_field(ph, 1)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class ARRAY_TEST_6 : GroundControl.ARRAY_TEST_6
        {
            public ushort v2 //Stub field
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort[] ar_u16 //Value array
            {
                get {return ar_u16_GET(new ushort[2], 0);}
            }
            public ushort[]ar_u16_GET(ushort[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 2, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public uint v3 //Stub field
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
            }

            public uint[] ar_u32 //Value array
            {
                get {return ar_u32_GET(new uint[2], 0);}
            }
            public uint[]ar_u32_GET(uint[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 10, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public byte v1 //Stub field
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  18, 1));}
            }

            public int[] ar_i32 //Value array
            {
                get {return ar_i32_GET(new int[2], 0);}
            }
            public int[]ar_i32_GET(int[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 19, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (int)((int) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public short[] ar_i16 //Value array
            {
                get {return ar_i16_GET(new short[2], 0);}
            }
            public short[]ar_i16_GET(short[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 27, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (short)((short) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public byte[] ar_u8 //Value array
            {
                get {return ar_u8_GET(new byte[2], 0);}
            }
            public byte[]ar_u8_GET(byte[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 31, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public sbyte[] ar_i8 //Value array
            {
                get {return ar_i8_GET(new sbyte[2], 0);}
            }
            public sbyte[]ar_i8_GET(sbyte[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 33, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (sbyte)((sbyte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public double[] ar_d //Value array
            {
                get {return ar_d_GET(new double[2], 0);}
            }
            public double[]ar_d_GET(double[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 35, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 8)
                    dst_ch[pos] = (double)(BitConverter.Int64BitsToDouble(BitUtils.get_bytes(data, BYTE, 8)));
                return dst_ch;
            }
            public float[] ar_f //Value array
            {
                get {return ar_f_GET(new float[2], 0);}
            }
            public float[]ar_f_GET(float[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 51, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            } public string ar_c_TRY(Inside ph)//Value array
            {
                if(ph.field_bit !=  472 && !try_visit_field(ph, 472)  ||  !try_visit_item(ph, 0)) return null;
                return new string(ar_c_GET(ph, new char[ph.items], 0));
            }
            public char[]ar_c_GET(Inside ph, char[] dst_ch, int pos) //Value array
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int ar_c_LEN(Inside ph)
            {
                return (ph.field_bit !=  472 && !try_visit_field(ph, 472)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class ARRAY_TEST_7 : GroundControl.ARRAY_TEST_7
        {
            public ushort[] ar_u16 //Value array
            {
                get {return ar_u16_GET(new ushort[2], 0);}
            }
            public ushort[]ar_u16_GET(ushort[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 0, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public uint[] ar_u32 //Value array
            {
                get {return ar_u32_GET(new uint[2], 0);}
            }
            public uint[]ar_u32_GET(uint[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 4, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public double[] ar_d //Value array
            {
                get {return ar_d_GET(new double[2], 0);}
            }
            public double[]ar_d_GET(double[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 12, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 8)
                    dst_ch[pos] = (double)(BitConverter.Int64BitsToDouble(BitUtils.get_bytes(data, BYTE, 8)));
                return dst_ch;
            }
            public float[] ar_f //Value array
            {
                get {return ar_f_GET(new float[2], 0);}
            }
            public float[]ar_f_GET(float[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 28, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public int[] ar_i32 //Value array
            {
                get {return ar_i32_GET(new int[2], 0);}
            }
            public int[]ar_i32_GET(int[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 36, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (int)((int) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public short[] ar_i16 //Value array
            {
                get {return ar_i16_GET(new short[2], 0);}
            }
            public short[]ar_i16_GET(short[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 44, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (short)((short) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public byte[] ar_u8 //Value array
            {
                get {return ar_u8_GET(new byte[2], 0);}
            }
            public byte[]ar_u8_GET(byte[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 48, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public sbyte[] ar_i8 //Value array
            {
                get {return ar_i8_GET(new sbyte[2], 0);}
            }
            public sbyte[]ar_i8_GET(sbyte[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 50, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (sbyte)((sbyte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            } public string ar_c_TRY(Inside ph)//Value array
            {
                if(ph.field_bit !=  416 && !try_visit_field(ph, 416)  ||  !try_visit_item(ph, 0)) return null;
                return new string(ar_c_GET(ph, new char[ph.items], 0));
            }
            public char[]ar_c_GET(Inside ph, char[] dst_ch, int pos) //Value array
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int ar_c_LEN(Inside ph)
            {
                return (ph.field_bit !=  416 && !try_visit_field(ph, 416)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class ARRAY_TEST_8 : GroundControl.ARRAY_TEST_8
        {
            public ushort[] ar_u16 //Value array
            {
                get {return ar_u16_GET(new ushort[2], 0);}
            }
            public ushort[]ar_u16_GET(ushort[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 0, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public uint v3 //Stub field
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
            }

            public double[] ar_d //Value array
            {
                get {return ar_d_GET(new double[2], 0);}
            }
            public double[]ar_d_GET(double[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 8, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 8)
                    dst_ch[pos] = (double)(BitConverter.Int64BitsToDouble(BitUtils.get_bytes(data, BYTE, 8)));
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
            public void OnARRAY_TEST_0Receive_direct(Channel src, Inside ph, ARRAY_TEST_0 pack) {OnARRAY_TEST_0Receive(this, ph,  pack);}
            public event ARRAY_TEST_0ReceiveHandler OnARRAY_TEST_0Receive;
            public delegate void ARRAY_TEST_0ReceiveHandler(Channel src, Inside ph, ARRAY_TEST_0 pack);
            public void OnARRAY_TEST_1Receive_direct(Channel src, Inside ph, ARRAY_TEST_1 pack) {OnARRAY_TEST_1Receive(this, ph,  pack);}
            public event ARRAY_TEST_1ReceiveHandler OnARRAY_TEST_1Receive;
            public delegate void ARRAY_TEST_1ReceiveHandler(Channel src, Inside ph, ARRAY_TEST_1 pack);
            public void OnARRAY_TEST_3Receive_direct(Channel src, Inside ph, ARRAY_TEST_3 pack) {OnARRAY_TEST_3Receive(this, ph,  pack);}
            public event ARRAY_TEST_3ReceiveHandler OnARRAY_TEST_3Receive;
            public delegate void ARRAY_TEST_3ReceiveHandler(Channel src, Inside ph, ARRAY_TEST_3 pack);
            public void OnARRAY_TEST_4Receive_direct(Channel src, Inside ph, ARRAY_TEST_4 pack) {OnARRAY_TEST_4Receive(this, ph,  pack);}
            public event ARRAY_TEST_4ReceiveHandler OnARRAY_TEST_4Receive;
            public delegate void ARRAY_TEST_4ReceiveHandler(Channel src, Inside ph, ARRAY_TEST_4 pack);
            public void OnARRAY_TEST_5Receive_direct(Channel src, Inside ph, ARRAY_TEST_5 pack) {OnARRAY_TEST_5Receive(this, ph,  pack);}
            public event ARRAY_TEST_5ReceiveHandler OnARRAY_TEST_5Receive;
            public delegate void ARRAY_TEST_5ReceiveHandler(Channel src, Inside ph, ARRAY_TEST_5 pack);
            public void OnARRAY_TEST_6Receive_direct(Channel src, Inside ph, ARRAY_TEST_6 pack) {OnARRAY_TEST_6Receive(this, ph,  pack);}
            public event ARRAY_TEST_6ReceiveHandler OnARRAY_TEST_6Receive;
            public delegate void ARRAY_TEST_6ReceiveHandler(Channel src, Inside ph, ARRAY_TEST_6 pack);
            public void OnARRAY_TEST_7Receive_direct(Channel src, Inside ph, ARRAY_TEST_7 pack) {OnARRAY_TEST_7Receive(this, ph,  pack);}
            public event ARRAY_TEST_7ReceiveHandler OnARRAY_TEST_7Receive;
            public delegate void ARRAY_TEST_7ReceiveHandler(Channel src, Inside ph, ARRAY_TEST_7 pack);
            public void OnARRAY_TEST_8Receive_direct(Channel src, Inside ph, ARRAY_TEST_8 pack) {OnARRAY_TEST_8Receive(this, ph,  pack);}
            public event ARRAY_TEST_8ReceiveHandler OnARRAY_TEST_8Receive;
            public delegate void ARRAY_TEST_8ReceiveHandler(Channel src, Inside ph, ARRAY_TEST_8 pack);
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
                        if(pack == null) return new ARRAY_TEST_0();
                        OnARRAY_TEST_0Receive(this, ph, (ARRAY_TEST_0) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 151:
                        if(pack == null) return new ARRAY_TEST_1();
                        OnARRAY_TEST_1Receive(this, ph, (ARRAY_TEST_1) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 153:
                        if(pack == null) return new ARRAY_TEST_3();
                        OnARRAY_TEST_3Receive(this, ph, (ARRAY_TEST_3) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 154:
                        if(pack == null) return new ARRAY_TEST_4();
                        OnARRAY_TEST_4Receive(this, ph, (ARRAY_TEST_4) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 155:
                        if(pack == null) return new ARRAY_TEST_5();
                        OnARRAY_TEST_5Receive(this, ph, (ARRAY_TEST_5) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 156:
                        if(pack == null) return new ARRAY_TEST_6();
                        OnARRAY_TEST_6Receive(this, ph, (ARRAY_TEST_6) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 157:
                        if(pack == null) return new ARRAY_TEST_7();
                        OnARRAY_TEST_7Receive(this, ph, (ARRAY_TEST_7) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 158:
                        if(pack == null) return new ARRAY_TEST_8();
                        OnARRAY_TEST_8Receive(this, ph, (ARRAY_TEST_8) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_ASLUAV);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED);
                Debug.Assert(pack.mavlink_version == (byte)(byte)139);
                Debug.Assert(pack.custom_mode == (uint)2415888766U);
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_HELICOPTER);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_ACTIVE);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_ASLUAV;
            p0.custom_mode = (uint)2415888766U;
            p0.mavlink_version = (byte)(byte)139;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_ACTIVE;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED;
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_HELICOPTER;
            ADV_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_battery == (short)(short)28638);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)74);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)55276);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)5129);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)61248);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)62936);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)34821);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)57863);
                Debug.Assert(pack.load == (ushort)(ushort)36550);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)58509);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.drop_rate_comm = (ushort)(ushort)62936;
            p1.current_battery = (short)(short)28638;
            p1.voltage_battery = (ushort)(ushort)58509;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION;
            p1.errors_count1 = (ushort)(ushort)34821;
            p1.load = (ushort)(ushort)36550;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
            p1.errors_count2 = (ushort)(ushort)57863;
            p1.errors_count3 = (ushort)(ushort)61248;
            p1.battery_remaining = (sbyte)(sbyte)74;
            p1.errors_count4 = (ushort)(ushort)5129;
            p1.errors_comm = (ushort)(ushort)55276;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
            ADV_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)9119551690016094453L);
                Debug.Assert(pack.time_boot_ms == (uint)1059530973U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_boot_ms = (uint)1059530973U;
            p2.time_unix_usec = (ulong)9119551690016094453L;
            ADV_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -1.5337804E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3638609482U);
                Debug.Assert(pack.type_mask == (ushort)(ushort)38075);
                Debug.Assert(pack.vx == (float) -8.099109E37F);
                Debug.Assert(pack.afx == (float)9.754258E37F);
                Debug.Assert(pack.yaw_rate == (float)1.2967302E38F);
                Debug.Assert(pack.afy == (float)1.4280827E38F);
                Debug.Assert(pack.y == (float) -2.8192886E38F);
                Debug.Assert(pack.vz == (float)1.277725E38F);
                Debug.Assert(pack.yaw == (float)8.839132E37F);
                Debug.Assert(pack.afz == (float)3.3014863E38F);
                Debug.Assert(pack.z == (float)2.549846E38F);
                Debug.Assert(pack.x == (float) -2.145109E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)3638609482U;
            p3.vy = (float) -1.5337804E38F;
            p3.z = (float)2.549846E38F;
            p3.afx = (float)9.754258E37F;
            p3.vz = (float)1.277725E38F;
            p3.y = (float) -2.8192886E38F;
            p3.x = (float) -2.145109E38F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p3.afy = (float)1.4280827E38F;
            p3.vx = (float) -8.099109E37F;
            p3.type_mask = (ushort)(ushort)38075;
            p3.afz = (float)3.3014863E38F;
            p3.yaw_rate = (float)1.2967302E38F;
            p3.yaw = (float)8.839132E37F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)101);
                Debug.Assert(pack.time_usec == (ulong)1440784585465073129L);
                Debug.Assert(pack.target_system == (byte)(byte)85);
                Debug.Assert(pack.seq == (uint)1314368284U);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.target_component = (byte)(byte)101;
            p4.target_system = (byte)(byte)85;
            p4.time_usec = (ulong)1440784585465073129L;
            p4.seq = (uint)1314368284U;
            ADV_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (byte)(byte)188);
                Debug.Assert(pack.control_request == (byte)(byte)23);
                Debug.Assert(pack.target_system == (byte)(byte)56);
                Debug.Assert(pack.passkey_LEN(ph) == 17);
                Debug.Assert(pack.passkey_TRY(ph).Equals("pbrpYsjwglevukmcQ"));
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.version = (byte)(byte)188;
            p5.passkey_SET("pbrpYsjwglevukmcQ", PH) ;
            p5.target_system = (byte)(byte)56;
            p5.control_request = (byte)(byte)23;
            ADV_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ack == (byte)(byte)60);
                Debug.Assert(pack.control_request == (byte)(byte)116);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)96);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)96;
            p6.ack = (byte)(byte)60;
            p6.control_request = (byte)(byte)116;
            ADV_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 17);
                Debug.Assert(pack.key_TRY(ph).Equals("ezbzzdiOOmqhillru"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("ezbzzdiOOmqhillru", PH) ;
            ADV_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_mode == (uint)3065446488U);
                Debug.Assert(pack.target_system == (byte)(byte)142);
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            p11.custom_mode = (uint)3065446488U;
            p11.target_system = (byte)(byte)142;
            ADV_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)219);
                Debug.Assert(pack.param_index == (short)(short) -15253);
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ax"));
                Debug.Assert(pack.target_component == (byte)(byte)52);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_index = (short)(short) -15253;
            p20.target_system = (byte)(byte)219;
            p20.target_component = (byte)(byte)52;
            p20.param_id_SET("ax", PH) ;
            ADV_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)46);
                Debug.Assert(pack.target_system == (byte)(byte)187);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)187;
            p21.target_component = (byte)(byte)46;
            ADV_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("wqcdiGp"));
                Debug.Assert(pack.param_index == (ushort)(ushort)2191);
                Debug.Assert(pack.param_value == (float) -2.8277095E38F);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
                Debug.Assert(pack.param_count == (ushort)(ushort)35715);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64;
            p22.param_index = (ushort)(ushort)2191;
            p22.param_count = (ushort)(ushort)35715;
            p22.param_value = (float) -2.8277095E38F;
            p22.param_id_SET("wqcdiGp", PH) ;
            ADV_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)232);
                Debug.Assert(pack.param_value == (float)1.6116548E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("jcde"));
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
                Debug.Assert(pack.target_system == (byte)(byte)224);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.target_system = (byte)(byte)224;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16;
            p23.param_id_SET("jcde", PH) ;
            p23.target_component = (byte)(byte)232;
            p23.param_value = (float)1.6116548E38F;
            ADV_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)335129602);
                Debug.Assert(pack.epv == (ushort)(ushort)55818);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)1596124181U);
                Debug.Assert(pack.cog == (ushort)(ushort)52536);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)1211733640U);
                Debug.Assert(pack.lon == (int) -861652651);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1291292358U);
                Debug.Assert(pack.eph == (ushort)(ushort)20437);
                Debug.Assert(pack.alt == (int) -945642347);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -945960540);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
                Debug.Assert(pack.vel == (ushort)(ushort)30063);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)3201355194U);
                Debug.Assert(pack.time_usec == (ulong)5610753091469283923L);
                Debug.Assert(pack.satellites_visible == (byte)(byte)99);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p24.eph = (ushort)(ushort)20437;
            p24.alt = (int) -945642347;
            p24.lon = (int) -861652651;
            p24.vel = (ushort)(ushort)30063;
            p24.h_acc_SET((uint)3201355194U, PH) ;
            p24.alt_ellipsoid_SET((int) -945960540, PH) ;
            p24.cog = (ushort)(ushort)52536;
            p24.lat = (int)335129602;
            p24.satellites_visible = (byte)(byte)99;
            p24.hdg_acc_SET((uint)1211733640U, PH) ;
            p24.vel_acc_SET((uint)1596124181U, PH) ;
            p24.epv = (ushort)(ushort)55818;
            p24.time_usec = (ulong)5610753091469283923L;
            p24.v_acc_SET((uint)1291292358U, PH) ;
            ADV_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)166, (byte)162, (byte)194, (byte)12, (byte)144, (byte)234, (byte)154, (byte)230, (byte)123, (byte)28, (byte)72, (byte)35, (byte)156, (byte)186, (byte)194, (byte)220, (byte)57, (byte)228, (byte)239, (byte)88}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)121, (byte)126, (byte)94, (byte)11, (byte)237, (byte)41, (byte)210, (byte)17, (byte)78, (byte)159, (byte)5, (byte)102, (byte)56, (byte)109, (byte)254, (byte)139, (byte)181, (byte)56, (byte)144, (byte)179}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)139, (byte)86, (byte)144, (byte)243, (byte)197, (byte)93, (byte)193, (byte)89, (byte)151, (byte)48, (byte)207, (byte)255, (byte)217, (byte)133, (byte)85, (byte)71, (byte)83, (byte)103, (byte)164, (byte)53}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)187, (byte)253, (byte)182, (byte)185, (byte)136, (byte)75, (byte)82, (byte)209, (byte)55, (byte)251, (byte)173, (byte)114, (byte)183, (byte)115, (byte)20, (byte)46, (byte)235, (byte)64, (byte)53, (byte)65}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)96);
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)184, (byte)213, (byte)74, (byte)91, (byte)156, (byte)183, (byte)90, (byte)85, (byte)0, (byte)19, (byte)86, (byte)237, (byte)101, (byte)186, (byte)114, (byte)116, (byte)0, (byte)3, (byte)204, (byte)30}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_snr_SET(new byte[] {(byte)166, (byte)162, (byte)194, (byte)12, (byte)144, (byte)234, (byte)154, (byte)230, (byte)123, (byte)28, (byte)72, (byte)35, (byte)156, (byte)186, (byte)194, (byte)220, (byte)57, (byte)228, (byte)239, (byte)88}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)121, (byte)126, (byte)94, (byte)11, (byte)237, (byte)41, (byte)210, (byte)17, (byte)78, (byte)159, (byte)5, (byte)102, (byte)56, (byte)109, (byte)254, (byte)139, (byte)181, (byte)56, (byte)144, (byte)179}, 0) ;
            p25.satellites_visible = (byte)(byte)96;
            p25.satellite_prn_SET(new byte[] {(byte)187, (byte)253, (byte)182, (byte)185, (byte)136, (byte)75, (byte)82, (byte)209, (byte)55, (byte)251, (byte)173, (byte)114, (byte)183, (byte)115, (byte)20, (byte)46, (byte)235, (byte)64, (byte)53, (byte)65}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)139, (byte)86, (byte)144, (byte)243, (byte)197, (byte)93, (byte)193, (byte)89, (byte)151, (byte)48, (byte)207, (byte)255, (byte)217, (byte)133, (byte)85, (byte)71, (byte)83, (byte)103, (byte)164, (byte)53}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)184, (byte)213, (byte)74, (byte)91, (byte)156, (byte)183, (byte)90, (byte)85, (byte)0, (byte)19, (byte)86, (byte)237, (byte)101, (byte)186, (byte)114, (byte)116, (byte)0, (byte)3, (byte)204, (byte)30}, 0) ;
            ADV_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (short)(short) -384);
                Debug.Assert(pack.zacc == (short)(short)1074);
                Debug.Assert(pack.xacc == (short)(short)11471);
                Debug.Assert(pack.ygyro == (short)(short) -13558);
                Debug.Assert(pack.ymag == (short)(short) -30251);
                Debug.Assert(pack.time_boot_ms == (uint)2127653812U);
                Debug.Assert(pack.yacc == (short)(short)26986);
                Debug.Assert(pack.zgyro == (short)(short)1725);
                Debug.Assert(pack.xmag == (short)(short)5868);
                Debug.Assert(pack.zmag == (short)(short) -10736);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.ygyro = (short)(short) -13558;
            p26.xmag = (short)(short)5868;
            p26.time_boot_ms = (uint)2127653812U;
            p26.zgyro = (short)(short)1725;
            p26.zacc = (short)(short)1074;
            p26.yacc = (short)(short)26986;
            p26.xgyro = (short)(short) -384;
            p26.ymag = (short)(short) -30251;
            p26.xacc = (short)(short)11471;
            p26.zmag = (short)(short) -10736;
            ADV_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short)16500);
                Debug.Assert(pack.ymag == (short)(short) -32665);
                Debug.Assert(pack.time_usec == (ulong)1587659104426257272L);
                Debug.Assert(pack.zacc == (short)(short)11591);
                Debug.Assert(pack.xacc == (short)(short)493);
                Debug.Assert(pack.zmag == (short)(short)24023);
                Debug.Assert(pack.zgyro == (short)(short)12118);
                Debug.Assert(pack.ygyro == (short)(short) -16669);
                Debug.Assert(pack.yacc == (short)(short)3649);
                Debug.Assert(pack.xgyro == (short)(short)14202);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.zgyro = (short)(short)12118;
            p27.xmag = (short)(short)16500;
            p27.ygyro = (short)(short) -16669;
            p27.xacc = (short)(short)493;
            p27.zacc = (short)(short)11591;
            p27.ymag = (short)(short) -32665;
            p27.zmag = (short)(short)24023;
            p27.time_usec = (ulong)1587659104426257272L;
            p27.yacc = (short)(short)3649;
            p27.xgyro = (short)(short)14202;
            ADV_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff2 == (short)(short)8113);
                Debug.Assert(pack.press_diff1 == (short)(short)13068);
                Debug.Assert(pack.temperature == (short)(short) -29997);
                Debug.Assert(pack.press_abs == (short)(short)16557);
                Debug.Assert(pack.time_usec == (ulong)4762090643146795997L);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff1 = (short)(short)13068;
            p28.press_abs = (short)(short)16557;
            p28.time_usec = (ulong)4762090643146795997L;
            p28.temperature = (short)(short) -29997;
            p28.press_diff2 = (short)(short)8113;
            ADV_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -2.804384E38F);
                Debug.Assert(pack.temperature == (short)(short)9851);
                Debug.Assert(pack.time_boot_ms == (uint)177596805U);
                Debug.Assert(pack.press_diff == (float) -2.7943295E38F);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.temperature = (short)(short)9851;
            p29.time_boot_ms = (uint)177596805U;
            p29.press_diff = (float) -2.7943295E38F;
            p29.press_abs = (float) -2.804384E38F;
            ADV_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float) -3.1236493E38F);
                Debug.Assert(pack.yaw == (float)3.5873153E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2655956088U);
                Debug.Assert(pack.pitch == (float) -2.2361624E38F);
                Debug.Assert(pack.pitchspeed == (float)3.2296594E38F);
                Debug.Assert(pack.rollspeed == (float) -2.5280137E38F);
                Debug.Assert(pack.roll == (float)2.4195075E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.yawspeed = (float) -3.1236493E38F;
            p30.roll = (float)2.4195075E38F;
            p30.pitch = (float) -2.2361624E38F;
            p30.yaw = (float)3.5873153E37F;
            p30.rollspeed = (float) -2.5280137E38F;
            p30.pitchspeed = (float)3.2296594E38F;
            p30.time_boot_ms = (uint)2655956088U;
            ADV_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q4 == (float) -8.516277E37F);
                Debug.Assert(pack.q3 == (float)3.3051278E38F);
                Debug.Assert(pack.yawspeed == (float) -1.327521E38F);
                Debug.Assert(pack.pitchspeed == (float)2.4310093E37F);
                Debug.Assert(pack.q1 == (float) -2.6046638E38F);
                Debug.Assert(pack.time_boot_ms == (uint)783286380U);
                Debug.Assert(pack.rollspeed == (float)1.8204286E38F);
                Debug.Assert(pack.q2 == (float)8.249243E37F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.q2 = (float)8.249243E37F;
            p31.pitchspeed = (float)2.4310093E37F;
            p31.rollspeed = (float)1.8204286E38F;
            p31.yawspeed = (float) -1.327521E38F;
            p31.q4 = (float) -8.516277E37F;
            p31.time_boot_ms = (uint)783286380U;
            p31.q3 = (float)3.3051278E38F;
            p31.q1 = (float) -2.6046638E38F;
            ADV_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -1.549184E38F);
                Debug.Assert(pack.vx == (float) -8.940696E37F);
                Debug.Assert(pack.x == (float)1.2233521E38F);
                Debug.Assert(pack.y == (float) -2.8604394E38F);
                Debug.Assert(pack.z == (float)1.3370891E38F);
                Debug.Assert(pack.vz == (float)7.789883E37F);
                Debug.Assert(pack.time_boot_ms == (uint)398689287U);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.y = (float) -2.8604394E38F;
            p32.time_boot_ms = (uint)398689287U;
            p32.vz = (float)7.789883E37F;
            p32.vx = (float) -8.940696E37F;
            p32.z = (float)1.3370891E38F;
            p32.vy = (float) -1.549184E38F;
            p32.x = (float)1.2233521E38F;
            ADV_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (short)(short) -27332);
                Debug.Assert(pack.relative_alt == (int)1990670877);
                Debug.Assert(pack.time_boot_ms == (uint)616826587U);
                Debug.Assert(pack.vx == (short)(short)23008);
                Debug.Assert(pack.lat == (int)696918037);
                Debug.Assert(pack.lon == (int) -702255857);
                Debug.Assert(pack.hdg == (ushort)(ushort)26732);
                Debug.Assert(pack.vz == (short)(short) -18787);
                Debug.Assert(pack.alt == (int) -178515311);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.lon = (int) -702255857;
            p33.hdg = (ushort)(ushort)26732;
            p33.time_boot_ms = (uint)616826587U;
            p33.vy = (short)(short) -27332;
            p33.lat = (int)696918037;
            p33.vx = (short)(short)23008;
            p33.alt = (int) -178515311;
            p33.relative_alt = (int)1990670877;
            p33.vz = (short)(short) -18787;
            ADV_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan8_scaled == (short)(short)9387);
                Debug.Assert(pack.chan6_scaled == (short)(short)2708);
                Debug.Assert(pack.chan5_scaled == (short)(short)23414);
                Debug.Assert(pack.chan3_scaled == (short)(short)31472);
                Debug.Assert(pack.chan7_scaled == (short)(short) -14444);
                Debug.Assert(pack.port == (byte)(byte)51);
                Debug.Assert(pack.time_boot_ms == (uint)1656008384U);
                Debug.Assert(pack.chan4_scaled == (short)(short)17464);
                Debug.Assert(pack.rssi == (byte)(byte)92);
                Debug.Assert(pack.chan2_scaled == (short)(short) -24211);
                Debug.Assert(pack.chan1_scaled == (short)(short)31205);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan8_scaled = (short)(short)9387;
            p34.chan3_scaled = (short)(short)31472;
            p34.rssi = (byte)(byte)92;
            p34.chan6_scaled = (short)(short)2708;
            p34.time_boot_ms = (uint)1656008384U;
            p34.chan4_scaled = (short)(short)17464;
            p34.chan7_scaled = (short)(short) -14444;
            p34.chan5_scaled = (short)(short)23414;
            p34.chan1_scaled = (short)(short)31205;
            p34.port = (byte)(byte)51;
            p34.chan2_scaled = (short)(short) -24211;
            ADV_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)12728);
                Debug.Assert(pack.rssi == (byte)(byte)90);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)12465);
                Debug.Assert(pack.time_boot_ms == (uint)1120916620U);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)27927);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)7519);
                Debug.Assert(pack.port == (byte)(byte)18);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)17184);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)40581);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)33050);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)5631);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan6_raw = (ushort)(ushort)40581;
            p35.chan3_raw = (ushort)(ushort)5631;
            p35.chan2_raw = (ushort)(ushort)33050;
            p35.chan5_raw = (ushort)(ushort)12465;
            p35.chan7_raw = (ushort)(ushort)17184;
            p35.chan8_raw = (ushort)(ushort)7519;
            p35.rssi = (byte)(byte)90;
            p35.chan1_raw = (ushort)(ushort)12728;
            p35.chan4_raw = (ushort)(ushort)27927;
            p35.port = (byte)(byte)18;
            p35.time_boot_ms = (uint)1120916620U;
            ADV_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)25510);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)9406);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)48703);
                Debug.Assert(pack.port == (byte)(byte)249);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)22289);
                Debug.Assert(pack.time_usec == (uint)1347440952U);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)7994);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)29762);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)47839);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)9064);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)55253);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)10994);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)51134);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)35501);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)24997);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)3860);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)23455);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)15123);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo10_raw_SET((ushort)(ushort)47839, PH) ;
            p36.servo6_raw = (ushort)(ushort)22289;
            p36.servo2_raw = (ushort)(ushort)9406;
            p36.servo4_raw = (ushort)(ushort)7994;
            p36.servo13_raw_SET((ushort)(ushort)48703, PH) ;
            p36.servo3_raw = (ushort)(ushort)25510;
            p36.servo9_raw_SET((ushort)(ushort)29762, PH) ;
            p36.servo5_raw = (ushort)(ushort)3860;
            p36.servo15_raw_SET((ushort)(ushort)35501, PH) ;
            p36.servo8_raw = (ushort)(ushort)24997;
            p36.servo14_raw_SET((ushort)(ushort)15123, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)9064, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)51134, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)55253, PH) ;
            p36.time_usec = (uint)1347440952U;
            p36.servo1_raw = (ushort)(ushort)10994;
            p36.port = (byte)(byte)249;
            p36.servo7_raw = (ushort)(ushort)23455;
            ADV_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)54);
                Debug.Assert(pack.start_index == (short)(short)22690);
                Debug.Assert(pack.end_index == (short)(short) -14992);
                Debug.Assert(pack.target_component == (byte)(byte)49);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_component = (byte)(byte)49;
            p37.start_index = (short)(short)22690;
            p37.target_system = (byte)(byte)54;
            p37.end_index = (short)(short) -14992;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            ADV_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)237);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.start_index == (short)(short) -30314);
                Debug.Assert(pack.end_index == (short)(short) -26749);
                Debug.Assert(pack.target_system == (byte)(byte)177);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.start_index = (short)(short) -30314;
            p38.target_system = (byte)(byte)177;
            p38.target_component = (byte)(byte)237;
            p38.end_index = (short)(short) -26749;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            ADV_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -2.5175805E37F);
                Debug.Assert(pack.param3 == (float) -1.5007803E38F);
                Debug.Assert(pack.param4 == (float) -1.3735258E37F);
                Debug.Assert(pack.autocontinue == (byte)(byte)124);
                Debug.Assert(pack.target_system == (byte)(byte)110);
                Debug.Assert(pack.z == (float) -2.2387322E38F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.seq == (ushort)(ushort)53350);
                Debug.Assert(pack.target_component == (byte)(byte)88);
                Debug.Assert(pack.param1 == (float) -1.3497025E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.param2 == (float) -2.3242076E38F);
                Debug.Assert(pack.y == (float) -1.6862461E38F);
                Debug.Assert(pack.current == (byte)(byte)101);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_ROI);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.param2 = (float) -2.3242076E38F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p39.param3 = (float) -1.5007803E38F;
            p39.param1 = (float) -1.3497025E38F;
            p39.target_system = (byte)(byte)110;
            p39.seq = (ushort)(ushort)53350;
            p39.z = (float) -2.2387322E38F;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_ROI;
            p39.param4 = (float) -1.3735258E37F;
            p39.x = (float) -2.5175805E37F;
            p39.target_component = (byte)(byte)88;
            p39.autocontinue = (byte)(byte)124;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p39.current = (byte)(byte)101;
            p39.y = (float) -1.6862461E38F;
            ADV_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)100);
                Debug.Assert(pack.seq == (ushort)(ushort)39369);
                Debug.Assert(pack.target_system == (byte)(byte)59);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.seq = (ushort)(ushort)39369;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p40.target_system = (byte)(byte)59;
            p40.target_component = (byte)(byte)100;
            ADV_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)64745);
                Debug.Assert(pack.target_component == (byte)(byte)216);
                Debug.Assert(pack.target_system == (byte)(byte)118);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)118;
            p41.target_component = (byte)(byte)216;
            p41.seq = (ushort)(ushort)64745;
            ADV_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)19782);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)19782;
            ADV_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)63);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)39);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_system = (byte)(byte)39;
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p43.target_component = (byte)(byte)63;
            ADV_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)61);
                Debug.Assert(pack.count == (ushort)(ushort)40807);
                Debug.Assert(pack.target_system == (byte)(byte)201);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)201;
            p44.count = (ushort)(ushort)40807;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p44.target_component = (byte)(byte)61;
            ADV_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)118);
                Debug.Assert(pack.target_system == (byte)(byte)54);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)54;
            p45.target_component = (byte)(byte)118;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            ADV_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)43577);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)43577;
            ADV_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM6_Y);
                Debug.Assert(pack.target_component == (byte)(byte)46);
                Debug.Assert(pack.target_system == (byte)(byte)108);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)108;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM6_Y;
            p47.target_component = (byte)(byte)46;
            ADV_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int) -663711750);
                Debug.Assert(pack.longitude == (int)519724240);
                Debug.Assert(pack.altitude == (int) -2076244044);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)9022880035136697812L);
                Debug.Assert(pack.target_system == (byte)(byte)139);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.target_system = (byte)(byte)139;
            p48.longitude = (int)519724240;
            p48.latitude = (int) -663711750;
            p48.altitude = (int) -2076244044;
            p48.time_usec_SET((ulong)9022880035136697812L, PH) ;
            ADV_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7866318773612235977L);
                Debug.Assert(pack.altitude == (int) -920214036);
                Debug.Assert(pack.latitude == (int) -1264760770);
                Debug.Assert(pack.longitude == (int)206635374);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.longitude = (int)206635374;
            p49.time_usec_SET((ulong)7866318773612235977L, PH) ;
            p49.altitude = (int) -920214036;
            p49.latitude = (int) -1264760770;
            ADV_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value0 == (float) -5.7786314E37F);
                Debug.Assert(pack.param_index == (short)(short) -28013);
                Debug.Assert(pack.target_component == (byte)(byte)24);
                Debug.Assert(pack.param_value_min == (float) -3.2465185E38F);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)72);
                Debug.Assert(pack.scale == (float)2.5004394E38F);
                Debug.Assert(pack.param_value_max == (float) -4.2511525E36F);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("zrh"));
                Debug.Assert(pack.target_system == (byte)(byte)104);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_value_max = (float) -4.2511525E36F;
            p50.param_value_min = (float) -3.2465185E38F;
            p50.scale = (float)2.5004394E38F;
            p50.target_system = (byte)(byte)104;
            p50.param_index = (short)(short) -28013;
            p50.param_id_SET("zrh", PH) ;
            p50.target_component = (byte)(byte)24;
            p50.param_value0 = (float) -5.7786314E37F;
            p50.parameter_rc_channel_index = (byte)(byte)72;
            ADV_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)24);
                Debug.Assert(pack.seq == (ushort)(ushort)10755);
                Debug.Assert(pack.target_system == (byte)(byte)57);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_system = (byte)(byte)57;
            p51.target_component = (byte)(byte)24;
            p51.seq = (ushort)(ushort)10755;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            ADV_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2z == (float)1.3887804E38F);
                Debug.Assert(pack.p1x == (float)1.978457E38F);
                Debug.Assert(pack.target_system == (byte)(byte)146);
                Debug.Assert(pack.p2x == (float) -1.375892E38F);
                Debug.Assert(pack.p1y == (float)2.2320285E38F);
                Debug.Assert(pack.p2y == (float) -2.251718E37F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.p1z == (float)1.6031591E38F);
                Debug.Assert(pack.target_component == (byte)(byte)243);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p2y = (float) -2.251718E37F;
            p54.p1z = (float)1.6031591E38F;
            p54.p2z = (float)1.3887804E38F;
            p54.target_system = (byte)(byte)146;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p54.p1x = (float)1.978457E38F;
            p54.p1y = (float)2.2320285E38F;
            p54.p2x = (float) -1.375892E38F;
            p54.target_component = (byte)(byte)243;
            ADV_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1y == (float)2.1236547E38F);
                Debug.Assert(pack.p2z == (float) -1.2050751E38F);
                Debug.Assert(pack.p2y == (float) -1.783384E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.p1z == (float)2.9870282E38F);
                Debug.Assert(pack.p1x == (float)2.1123517E37F);
                Debug.Assert(pack.p2x == (float)1.6494207E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p55.p1z = (float)2.9870282E38F;
            p55.p2z = (float) -1.2050751E38F;
            p55.p1x = (float)2.1123517E37F;
            p55.p2x = (float)1.6494207E38F;
            p55.p2y = (float) -1.783384E38F;
            p55.p1y = (float)2.1236547E38F;
            ADV_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float)2.5820777E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {4.1139531E37F, -1.4120025E38F, 2.0576255E38F, 2.553566E38F}));
                Debug.Assert(pack.time_usec == (ulong)2781485177138278584L);
                Debug.Assert(pack.pitchspeed == (float) -2.4509343E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.4334114E38F, -9.51163E37F, -2.3205166E38F, -1.1916085E38F, -5.13186E37F, 3.249499E38F, -2.8319323E38F, 7.115573E36F, 1.99048E38F}));
                Debug.Assert(pack.rollspeed == (float) -5.485797E37F);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.pitchspeed = (float) -2.4509343E38F;
            p61.covariance_SET(new float[] {1.4334114E38F, -9.51163E37F, -2.3205166E38F, -1.1916085E38F, -5.13186E37F, 3.249499E38F, -2.8319323E38F, 7.115573E36F, 1.99048E38F}, 0) ;
            p61.time_usec = (ulong)2781485177138278584L;
            p61.yawspeed = (float)2.5820777E38F;
            p61.rollspeed = (float) -5.485797E37F;
            p61.q_SET(new float[] {4.1139531E37F, -1.4120025E38F, 2.0576255E38F, 2.553566E38F}, 0) ;
            ADV_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_pitch == (float)2.393438E38F);
                Debug.Assert(pack.target_bearing == (short)(short)31330);
                Debug.Assert(pack.nav_roll == (float) -2.1185514E38F);
                Debug.Assert(pack.nav_bearing == (short)(short) -14888);
                Debug.Assert(pack.xtrack_error == (float)7.1557893E37F);
                Debug.Assert(pack.aspd_error == (float) -7.3528197E37F);
                Debug.Assert(pack.alt_error == (float)1.6432102E37F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)52428);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.aspd_error = (float) -7.3528197E37F;
            p62.nav_pitch = (float)2.393438E38F;
            p62.xtrack_error = (float)7.1557893E37F;
            p62.wp_dist = (ushort)(ushort)52428;
            p62.alt_error = (float)1.6432102E37F;
            p62.nav_roll = (float) -2.1185514E38F;
            p62.nav_bearing = (short)(short) -14888;
            p62.target_bearing = (short)(short)31330;
            ADV_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float) -1.332053E38F);
                Debug.Assert(pack.time_usec == (ulong)2275140382151121034L);
                Debug.Assert(pack.vy == (float) -3.1159199E38F);
                Debug.Assert(pack.lat == (int) -39655239);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.2293654E38F, -4.7100946E36F, 2.922402E38F, -2.0696841E36F, 3.2295841E38F, -1.5019327E38F, -1.1933034E38F, 2.1315216E38F, -4.3240267E37F, 1.3479582E38F, -3.0350721E38F, 2.06593E38F, 5.5547673E37F, -5.524519E37F, -8.3786173E37F, 2.961021E38F, -1.9863458E38F, 2.2153458E38F, 8.961026E37F, -1.3999134E38F, 2.1960217E38F, 3.3368018E36F, 6.768033E37F, 2.2109033E38F, 2.7565255E38F, -1.9526095E38F, -1.8658042E38F, -1.0118497E38F, 1.0157345E38F, 2.387256E38F, 8.142707E37F, 2.9614306E38F, -1.5418251E38F, -2.2743652E38F, -1.5232171E38F, 2.3785421E38F}));
                Debug.Assert(pack.alt == (int) -1744017289);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.lon == (int)351809312);
                Debug.Assert(pack.vx == (float)1.0524328E38F);
                Debug.Assert(pack.relative_alt == (int)587456766);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.lon = (int)351809312;
            p63.vx = (float)1.0524328E38F;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p63.vy = (float) -3.1159199E38F;
            p63.vz = (float) -1.332053E38F;
            p63.time_usec = (ulong)2275140382151121034L;
            p63.relative_alt = (int)587456766;
            p63.covariance_SET(new float[] {2.2293654E38F, -4.7100946E36F, 2.922402E38F, -2.0696841E36F, 3.2295841E38F, -1.5019327E38F, -1.1933034E38F, 2.1315216E38F, -4.3240267E37F, 1.3479582E38F, -3.0350721E38F, 2.06593E38F, 5.5547673E37F, -5.524519E37F, -8.3786173E37F, 2.961021E38F, -1.9863458E38F, 2.2153458E38F, 8.961026E37F, -1.3999134E38F, 2.1960217E38F, 3.3368018E36F, 6.768033E37F, 2.2109033E38F, 2.7565255E38F, -1.9526095E38F, -1.8658042E38F, -1.0118497E38F, 1.0157345E38F, 2.387256E38F, 8.142707E37F, 2.9614306E38F, -1.5418251E38F, -2.2743652E38F, -1.5232171E38F, 2.3785421E38F}, 0) ;
            p63.lat = (int) -39655239;
            p63.alt = (int) -1744017289;
            ADV_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3160909612793610621L);
                Debug.Assert(pack.vx == (float)2.2834667E38F);
                Debug.Assert(pack.y == (float) -2.5757458E37F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-7.625214E37F, -3.307481E38F, 3.1839314E38F, 1.8825792E38F, -3.0815357E38F, 9.740346E37F, 2.6165868E38F, 2.2609088E38F, -1.9309097E38F, -2.7900371E38F, -2.284138E38F, 3.0177145E38F, 2.244993E38F, 1.6593391E38F, -2.0067146E38F, -1.1185901E37F, -3.2687075E38F, 3.1919694E38F, -6.2438024E37F, -2.39167E38F, -1.0835027E38F, 1.450094E38F, 3.1827614E38F, 3.5966703E37F, 1.9906485E38F, -4.58978E36F, 3.402283E38F, -2.199354E38F, -1.2159325E38F, 2.0201108E38F, -1.4715786E37F, 2.6953371E38F, -2.9930328E38F, 3.4119772E37F, 2.8611253E38F, 8.862149E37F, 2.6367982E38F, -1.2148264E38F, -1.0414629E38F, 2.000477E38F, -3.372255E38F, 2.9631992E38F, -2.5647659E38F, -2.6775105E38F, 4.2775257E37F}));
                Debug.Assert(pack.az == (float) -6.8474946E37F);
                Debug.Assert(pack.vy == (float) -5.46595E37F);
                Debug.Assert(pack.ay == (float)1.6188477E38F);
                Debug.Assert(pack.z == (float)1.8783231E38F);
                Debug.Assert(pack.x == (float) -4.9411545E37F);
                Debug.Assert(pack.ax == (float) -2.8118392E37F);
                Debug.Assert(pack.vz == (float) -1.4878678E38F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.ay = (float)1.6188477E38F;
            p64.vx = (float)2.2834667E38F;
            p64.time_usec = (ulong)3160909612793610621L;
            p64.y = (float) -2.5757458E37F;
            p64.x = (float) -4.9411545E37F;
            p64.z = (float)1.8783231E38F;
            p64.ax = (float) -2.8118392E37F;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            p64.covariance_SET(new float[] {-7.625214E37F, -3.307481E38F, 3.1839314E38F, 1.8825792E38F, -3.0815357E38F, 9.740346E37F, 2.6165868E38F, 2.2609088E38F, -1.9309097E38F, -2.7900371E38F, -2.284138E38F, 3.0177145E38F, 2.244993E38F, 1.6593391E38F, -2.0067146E38F, -1.1185901E37F, -3.2687075E38F, 3.1919694E38F, -6.2438024E37F, -2.39167E38F, -1.0835027E38F, 1.450094E38F, 3.1827614E38F, 3.5966703E37F, 1.9906485E38F, -4.58978E36F, 3.402283E38F, -2.199354E38F, -1.2159325E38F, 2.0201108E38F, -1.4715786E37F, 2.6953371E38F, -2.9930328E38F, 3.4119772E37F, 2.8611253E38F, 8.862149E37F, 2.6367982E38F, -1.2148264E38F, -1.0414629E38F, 2.000477E38F, -3.372255E38F, 2.9631992E38F, -2.5647659E38F, -2.6775105E38F, 4.2775257E37F}, 0) ;
            p64.vy = (float) -5.46595E37F;
            p64.vz = (float) -1.4878678E38F;
            p64.az = (float) -6.8474946E37F;
            ADV_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)2061);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)39702);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)19092);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)54111);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)18022);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)56996);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)25553);
                Debug.Assert(pack.time_boot_ms == (uint)2932469987U);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)40098);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)10035);
                Debug.Assert(pack.rssi == (byte)(byte)200);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)21289);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)25221);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)24454);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)44550);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)32087);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)34775);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)60969);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)53732);
                Debug.Assert(pack.chancount == (byte)(byte)94);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)17619);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan3_raw = (ushort)(ushort)34775;
            p65.chan4_raw = (ushort)(ushort)32087;
            p65.chan2_raw = (ushort)(ushort)40098;
            p65.chan9_raw = (ushort)(ushort)56996;
            p65.time_boot_ms = (uint)2932469987U;
            p65.chancount = (byte)(byte)94;
            p65.chan10_raw = (ushort)(ushort)54111;
            p65.chan13_raw = (ushort)(ushort)53732;
            p65.chan8_raw = (ushort)(ushort)10035;
            p65.chan5_raw = (ushort)(ushort)24454;
            p65.rssi = (byte)(byte)200;
            p65.chan12_raw = (ushort)(ushort)60969;
            p65.chan16_raw = (ushort)(ushort)18022;
            p65.chan11_raw = (ushort)(ushort)21289;
            p65.chan18_raw = (ushort)(ushort)25553;
            p65.chan1_raw = (ushort)(ushort)25221;
            p65.chan15_raw = (ushort)(ushort)44550;
            p65.chan17_raw = (ushort)(ushort)19092;
            p65.chan7_raw = (ushort)(ushort)39702;
            p65.chan6_raw = (ushort)(ushort)17619;
            p65.chan14_raw = (ushort)(ushort)2061;
            ADV_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)178);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)15519);
                Debug.Assert(pack.start_stop == (byte)(byte)65);
                Debug.Assert(pack.target_component == (byte)(byte)228);
                Debug.Assert(pack.req_stream_id == (byte)(byte)200);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.req_stream_id = (byte)(byte)200;
            p66.target_system = (byte)(byte)178;
            p66.start_stop = (byte)(byte)65;
            p66.req_message_rate = (ushort)(ushort)15519;
            p66.target_component = (byte)(byte)228;
            ADV_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)22289);
                Debug.Assert(pack.stream_id == (byte)(byte)77);
                Debug.Assert(pack.on_off == (byte)(byte)31);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.on_off = (byte)(byte)31;
            p67.message_rate = (ushort)(ushort)22289;
            p67.stream_id = (byte)(byte)77;
            ADV_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.buttons == (ushort)(ushort)53393);
                Debug.Assert(pack.y == (short)(short)18058);
                Debug.Assert(pack.target == (byte)(byte)152);
                Debug.Assert(pack.r == (short)(short)31144);
                Debug.Assert(pack.x == (short)(short)27991);
                Debug.Assert(pack.z == (short)(short)20683);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.target = (byte)(byte)152;
            p69.x = (short)(short)27991;
            p69.z = (short)(short)20683;
            p69.buttons = (ushort)(ushort)53393;
            p69.r = (short)(short)31144;
            p69.y = (short)(short)18058;
            ADV_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)61804);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)57678);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)6783);
                Debug.Assert(pack.target_system == (byte)(byte)99);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)39358);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)60773);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)39532);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)34491);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)55555);
                Debug.Assert(pack.target_component == (byte)(byte)131);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan8_raw = (ushort)(ushort)39532;
            p70.chan3_raw = (ushort)(ushort)60773;
            p70.chan1_raw = (ushort)(ushort)55555;
            p70.chan7_raw = (ushort)(ushort)39358;
            p70.target_component = (byte)(byte)131;
            p70.chan4_raw = (ushort)(ushort)34491;
            p70.target_system = (byte)(byte)99;
            p70.chan6_raw = (ushort)(ushort)6783;
            p70.chan5_raw = (ushort)(ushort)61804;
            p70.chan2_raw = (ushort)(ushort)57678;
            ADV_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param4 == (float)1.5886065E38F);
                Debug.Assert(pack.current == (byte)(byte)186);
                Debug.Assert(pack.y == (int)243516320);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.param1 == (float) -2.7088745E37F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS);
                Debug.Assert(pack.seq == (ushort)(ushort)26847);
                Debug.Assert(pack.param3 == (float) -1.0674211E38F);
                Debug.Assert(pack.x == (int) -761014731);
                Debug.Assert(pack.target_component == (byte)(byte)159);
                Debug.Assert(pack.z == (float)2.2750195E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)158);
                Debug.Assert(pack.target_system == (byte)(byte)222);
                Debug.Assert(pack.param2 == (float)3.980827E37F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.param4 = (float)1.5886065E38F;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p73.current = (byte)(byte)186;
            p73.param3 = (float) -1.0674211E38F;
            p73.param2 = (float)3.980827E37F;
            p73.y = (int)243516320;
            p73.autocontinue = (byte)(byte)158;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS;
            p73.target_component = (byte)(byte)159;
            p73.seq = (ushort)(ushort)26847;
            p73.z = (float)2.2750195E38F;
            p73.x = (int) -761014731;
            p73.param1 = (float) -2.7088745E37F;
            p73.target_system = (byte)(byte)222;
            ADV_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.throttle == (ushort)(ushort)10277);
                Debug.Assert(pack.climb == (float)1.6657847E38F);
                Debug.Assert(pack.alt == (float)1.2279377E38F);
                Debug.Assert(pack.groundspeed == (float)1.9327834E38F);
                Debug.Assert(pack.airspeed == (float)3.1852372E38F);
                Debug.Assert(pack.heading == (short)(short) -19392);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.climb = (float)1.6657847E38F;
            p74.heading = (short)(short) -19392;
            p74.airspeed = (float)3.1852372E38F;
            p74.groundspeed = (float)1.9327834E38F;
            p74.throttle = (ushort)(ushort)10277;
            p74.alt = (float)1.2279377E38F;
            ADV_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (int)10721857);
                Debug.Assert(pack.param4 == (float)1.4364789E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)48);
                Debug.Assert(pack.param3 == (float)2.2090925E38F);
                Debug.Assert(pack.current == (byte)(byte)247);
                Debug.Assert(pack.param1 == (float) -1.7420807E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_NAV_ROI);
                Debug.Assert(pack.param2 == (float) -3.2152016E38F);
                Debug.Assert(pack.y == (int) -1311302091);
                Debug.Assert(pack.target_component == (byte)(byte)149);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.z == (float) -1.8338241E38F);
                Debug.Assert(pack.target_system == (byte)(byte)190);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.autocontinue = (byte)(byte)48;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p75.target_system = (byte)(byte)190;
            p75.param2 = (float) -3.2152016E38F;
            p75.z = (float) -1.8338241E38F;
            p75.param4 = (float)1.4364789E38F;
            p75.param1 = (float) -1.7420807E38F;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_ROI;
            p75.x = (int)10721857;
            p75.target_component = (byte)(byte)149;
            p75.param3 = (float)2.2090925E38F;
            p75.current = (byte)(byte)247;
            p75.y = (int) -1311302091;
            ADV_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param6 == (float) -1.0022293E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)13);
                Debug.Assert(pack.param5 == (float)8.539146E37F);
                Debug.Assert(pack.target_system == (byte)(byte)190);
                Debug.Assert(pack.param2 == (float) -1.9736778E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_REVERSE);
                Debug.Assert(pack.param4 == (float)2.4975794E38F);
                Debug.Assert(pack.target_component == (byte)(byte)165);
                Debug.Assert(pack.param7 == (float)8.636493E37F);
                Debug.Assert(pack.param3 == (float) -1.5313299E38F);
                Debug.Assert(pack.param1 == (float) -7.4358574E37F);
            };
            GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.param2 = (float) -1.9736778E38F;
            p76.confirmation = (byte)(byte)13;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_REVERSE;
            p76.target_system = (byte)(byte)190;
            p76.param6 = (float) -1.0022293E38F;
            p76.param7 = (float)8.636493E37F;
            p76.param1 = (float) -7.4358574E37F;
            p76.param4 = (float)2.4975794E38F;
            p76.param5 = (float)8.539146E37F;
            p76.target_component = (byte)(byte)165;
            p76.param3 = (float) -1.5313299E38F;
            CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -1349966123);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_CONDITION_LAST);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)79);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)71);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_IN_PROGRESS);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)249);
            };
            GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.target_component_SET((byte)(byte)71, PH) ;
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_CONDITION_LAST;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_IN_PROGRESS;
            p77.result_param2_SET((int) -1349966123, PH) ;
            p77.target_system_SET((byte)(byte)249, PH) ;
            p77.progress_SET((byte)(byte)79, PH) ;
            CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)3.9462716E37F);
                Debug.Assert(pack.roll == (float)1.6616635E38F);
                Debug.Assert(pack.pitch == (float)1.0495075E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1930552669U);
                Debug.Assert(pack.mode_switch == (byte)(byte)197);
                Debug.Assert(pack.thrust == (float) -1.3616943E38F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)224);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.manual_override_switch = (byte)(byte)224;
            p81.thrust = (float) -1.3616943E38F;
            p81.roll = (float)1.6616635E38F;
            p81.mode_switch = (byte)(byte)197;
            p81.pitch = (float)1.0495075E38F;
            p81.time_boot_ms = (uint)1930552669U;
            p81.yaw = (float)3.9462716E37F;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.thrust == (float) -3.7742933E37F);
                Debug.Assert(pack.type_mask == (byte)(byte)255);
                Debug.Assert(pack.time_boot_ms == (uint)3017427590U);
                Debug.Assert(pack.target_system == (byte)(byte)55);
                Debug.Assert(pack.target_component == (byte)(byte)155);
                Debug.Assert(pack.body_roll_rate == (float) -3.1950375E38F);
                Debug.Assert(pack.body_yaw_rate == (float)1.3222946E38F);
                Debug.Assert(pack.body_pitch_rate == (float) -1.2547177E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.3389411E38F, 1.1009267E38F, -2.8003122E38F, 3.0817261E38F}));
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_yaw_rate = (float)1.3222946E38F;
            p82.body_roll_rate = (float) -3.1950375E38F;
            p82.target_component = (byte)(byte)155;
            p82.type_mask = (byte)(byte)255;
            p82.time_boot_ms = (uint)3017427590U;
            p82.target_system = (byte)(byte)55;
            p82.thrust = (float) -3.7742933E37F;
            p82.body_pitch_rate = (float) -1.2547177E38F;
            p82.q_SET(new float[] {-2.3389411E38F, 1.1009267E38F, -2.8003122E38F, 3.0817261E38F}, 0) ;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (byte)(byte)142);
                Debug.Assert(pack.body_yaw_rate == (float) -3.3235967E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4266184014U);
                Debug.Assert(pack.body_roll_rate == (float)1.3143933E38F);
                Debug.Assert(pack.body_pitch_rate == (float)7.1910867E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.9124435E38F, -9.162561E36F, -2.2637242E38F, 2.7303685E38F}));
                Debug.Assert(pack.thrust == (float) -1.4719414E38F);
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_roll_rate = (float)1.3143933E38F;
            p83.q_SET(new float[] {2.9124435E38F, -9.162561E36F, -2.2637242E38F, 2.7303685E38F}, 0) ;
            p83.body_pitch_rate = (float)7.1910867E37F;
            p83.type_mask = (byte)(byte)142;
            p83.time_boot_ms = (uint)4266184014U;
            p83.body_yaw_rate = (float) -3.3235967E38F;
            p83.thrust = (float) -1.4719414E38F;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afx == (float)1.6352321E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)29064);
                Debug.Assert(pack.target_system == (byte)(byte)135);
                Debug.Assert(pack.afy == (float) -8.633538E35F);
                Debug.Assert(pack.yaw_rate == (float) -2.2930345E38F);
                Debug.Assert(pack.y == (float) -2.2850968E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.vy == (float)1.769241E38F);
                Debug.Assert(pack.afz == (float) -7.978698E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3952002032U);
                Debug.Assert(pack.vx == (float)1.6160118E38F);
                Debug.Assert(pack.x == (float) -4.421751E37F);
                Debug.Assert(pack.vz == (float)2.3101133E38F);
                Debug.Assert(pack.yaw == (float) -2.5686822E38F);
                Debug.Assert(pack.z == (float) -2.6992013E38F);
                Debug.Assert(pack.target_component == (byte)(byte)83);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.afy = (float) -8.633538E35F;
            p84.afx = (float)1.6352321E38F;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p84.afz = (float) -7.978698E37F;
            p84.vy = (float)1.769241E38F;
            p84.x = (float) -4.421751E37F;
            p84.yaw = (float) -2.5686822E38F;
            p84.y = (float) -2.2850968E38F;
            p84.target_component = (byte)(byte)83;
            p84.type_mask = (ushort)(ushort)29064;
            p84.yaw_rate = (float) -2.2930345E38F;
            p84.z = (float) -2.6992013E38F;
            p84.vz = (float)2.3101133E38F;
            p84.vx = (float)1.6160118E38F;
            p84.time_boot_ms = (uint)3952002032U;
            p84.target_system = (byte)(byte)135;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float) -1.0566291E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.target_component == (byte)(byte)255);
                Debug.Assert(pack.vx == (float) -2.9628361E38F);
                Debug.Assert(pack.time_boot_ms == (uint)696887601U);
                Debug.Assert(pack.target_system == (byte)(byte)34);
                Debug.Assert(pack.yaw == (float) -3.1462027E38F);
                Debug.Assert(pack.vy == (float) -1.7941353E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)20040);
                Debug.Assert(pack.afz == (float)2.6258206E38F);
                Debug.Assert(pack.vz == (float)9.803991E37F);
                Debug.Assert(pack.lon_int == (int) -318877320);
                Debug.Assert(pack.afx == (float)4.0652863E37F);
                Debug.Assert(pack.yaw_rate == (float) -2.971207E38F);
                Debug.Assert(pack.lat_int == (int)1835056427);
                Debug.Assert(pack.afy == (float)3.4004565E38F);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.yaw_rate = (float) -2.971207E38F;
            p86.afz = (float)2.6258206E38F;
            p86.lat_int = (int)1835056427;
            p86.target_component = (byte)(byte)255;
            p86.vy = (float) -1.7941353E38F;
            p86.alt = (float) -1.0566291E38F;
            p86.vx = (float) -2.9628361E38F;
            p86.vz = (float)9.803991E37F;
            p86.lon_int = (int) -318877320;
            p86.type_mask = (ushort)(ushort)20040;
            p86.afy = (float)3.4004565E38F;
            p86.time_boot_ms = (uint)696887601U;
            p86.afx = (float)4.0652863E37F;
            p86.yaw = (float) -3.1462027E38F;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p86.target_system = (byte)(byte)34;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afy == (float) -1.1472943E37F);
                Debug.Assert(pack.vz == (float)2.1597194E38F);
                Debug.Assert(pack.lat_int == (int)419508925);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.lon_int == (int) -647521391);
                Debug.Assert(pack.time_boot_ms == (uint)3488822535U);
                Debug.Assert(pack.afz == (float) -3.152006E38F);
                Debug.Assert(pack.yaw == (float) -1.4908484E38F);
                Debug.Assert(pack.vx == (float) -1.6466573E38F);
                Debug.Assert(pack.vy == (float) -1.7699978E38F);
                Debug.Assert(pack.alt == (float) -1.0756004E38F);
                Debug.Assert(pack.yaw_rate == (float) -2.9978742E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)36986);
                Debug.Assert(pack.afx == (float)3.0243703E38F);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.afy = (float) -1.1472943E37F;
            p87.lon_int = (int) -647521391;
            p87.vx = (float) -1.6466573E38F;
            p87.afx = (float)3.0243703E38F;
            p87.alt = (float) -1.0756004E38F;
            p87.vy = (float) -1.7699978E38F;
            p87.type_mask = (ushort)(ushort)36986;
            p87.afz = (float) -3.152006E38F;
            p87.time_boot_ms = (uint)3488822535U;
            p87.yaw = (float) -1.4908484E38F;
            p87.vz = (float)2.1597194E38F;
            p87.lat_int = (int)419508925;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p87.yaw_rate = (float) -2.9978742E38F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)1.7035023E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2417433039U);
                Debug.Assert(pack.pitch == (float) -2.063049E38F);
                Debug.Assert(pack.roll == (float) -1.3569803E38F);
                Debug.Assert(pack.x == (float) -2.5021119E38F);
                Debug.Assert(pack.z == (float)2.3686567E38F);
                Debug.Assert(pack.y == (float)3.0389E37F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.z = (float)2.3686567E38F;
            p89.pitch = (float) -2.063049E38F;
            p89.yaw = (float)1.7035023E38F;
            p89.y = (float)3.0389E37F;
            p89.x = (float) -2.5021119E38F;
            p89.time_boot_ms = (uint)2417433039U;
            p89.roll = (float) -1.3569803E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short) -1341);
                Debug.Assert(pack.yawspeed == (float)1.3862906E38F);
                Debug.Assert(pack.vx == (short)(short)26480);
                Debug.Assert(pack.lat == (int) -1013033209);
                Debug.Assert(pack.yacc == (short)(short)26835);
                Debug.Assert(pack.pitchspeed == (float)1.5159439E38F);
                Debug.Assert(pack.vy == (short)(short)2021);
                Debug.Assert(pack.time_usec == (ulong)1695413506880303480L);
                Debug.Assert(pack.pitch == (float)4.270375E37F);
                Debug.Assert(pack.zacc == (short)(short)28876);
                Debug.Assert(pack.alt == (int)1317711172);
                Debug.Assert(pack.yaw == (float) -1.566778E38F);
                Debug.Assert(pack.roll == (float) -1.4377247E38F);
                Debug.Assert(pack.lon == (int) -365157338);
                Debug.Assert(pack.rollspeed == (float)5.9410007E37F);
                Debug.Assert(pack.vz == (short)(short)30658);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.yawspeed = (float)1.3862906E38F;
            p90.yaw = (float) -1.566778E38F;
            p90.vx = (short)(short)26480;
            p90.pitchspeed = (float)1.5159439E38F;
            p90.time_usec = (ulong)1695413506880303480L;
            p90.xacc = (short)(short) -1341;
            p90.yacc = (short)(short)26835;
            p90.lon = (int) -365157338;
            p90.pitch = (float)4.270375E37F;
            p90.zacc = (short)(short)28876;
            p90.rollspeed = (float)5.9410007E37F;
            p90.vz = (short)(short)30658;
            p90.alt = (int)1317711172;
            p90.lat = (int) -1013033209;
            p90.vy = (short)(short)2021;
            p90.roll = (float) -1.4377247E38F;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aux2 == (float)3.2795068E38F);
                Debug.Assert(pack.yaw_rudder == (float)1.0048652E38F);
                Debug.Assert(pack.roll_ailerons == (float)9.567827E37F);
                Debug.Assert(pack.time_usec == (ulong)7764458627841850280L);
                Debug.Assert(pack.throttle == (float) -2.1566882E38F);
                Debug.Assert(pack.pitch_elevator == (float)6.482531E37F);
                Debug.Assert(pack.aux3 == (float)6.0990747E37F);
                Debug.Assert(pack.aux4 == (float)3.670643E36F);
                Debug.Assert(pack.aux1 == (float)6.5067126E37F);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED);
                Debug.Assert(pack.nav_mode == (byte)(byte)198);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.aux3 = (float)6.0990747E37F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED;
            p91.time_usec = (ulong)7764458627841850280L;
            p91.aux1 = (float)6.5067126E37F;
            p91.nav_mode = (byte)(byte)198;
            p91.pitch_elevator = (float)6.482531E37F;
            p91.aux4 = (float)3.670643E36F;
            p91.roll_ailerons = (float)9.567827E37F;
            p91.throttle = (float) -2.1566882E38F;
            p91.yaw_rudder = (float)1.0048652E38F;
            p91.aux2 = (float)3.2795068E38F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)42803);
                Debug.Assert(pack.rssi == (byte)(byte)213);
                Debug.Assert(pack.time_usec == (ulong)7272920062378903447L);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)35880);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)57609);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)62865);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)4003);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)44044);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)28712);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)24687);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)57071);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)16617);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)42040);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)8269);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.rssi = (byte)(byte)213;
            p92.chan12_raw = (ushort)(ushort)57071;
            p92.time_usec = (ulong)7272920062378903447L;
            p92.chan5_raw = (ushort)(ushort)44044;
            p92.chan1_raw = (ushort)(ushort)42803;
            p92.chan11_raw = (ushort)(ushort)62865;
            p92.chan10_raw = (ushort)(ushort)24687;
            p92.chan4_raw = (ushort)(ushort)57609;
            p92.chan2_raw = (ushort)(ushort)4003;
            p92.chan3_raw = (ushort)(ushort)8269;
            p92.chan9_raw = (ushort)(ushort)28712;
            p92.chan8_raw = (ushort)(ushort)16617;
            p92.chan6_raw = (ushort)(ushort)35880;
            p92.chan7_raw = (ushort)(ushort)42040;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)2108541111504062976L);
                Debug.Assert(pack.flags == (ulong)2841538874369335741L);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.2209489E38F, -1.8232576E38F, 1.7039628E37F, 3.3257937E38F, -2.9777407E38F, 1.4018142E38F, 3.0457642E38F, 1.2349274E38F, -1.7334623E38F, 2.9533428E38F, -2.1186642E38F, -1.0068909E38F, 2.6159824E38F, -8.516627E37F, 2.7966216E38F, -9.655955E37F}));
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED;
            p93.time_usec = (ulong)2108541111504062976L;
            p93.flags = (ulong)2841538874369335741L;
            p93.controls_SET(new float[] {-1.2209489E38F, -1.8232576E38F, 1.7039628E37F, 3.3257937E38F, -2.9777407E38F, 1.4018142E38F, 3.0457642E38F, 1.2349274E38F, -1.7334623E38F, 2.9533428E38F, -2.1186642E38F, -1.0068909E38F, 2.6159824E38F, -8.516627E37F, 2.7966216E38F, -9.655955E37F}, 0) ;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.quality == (byte)(byte)194);
                Debug.Assert(pack.time_usec == (ulong)3927762197109102431L);
                Debug.Assert(pack.ground_distance == (float)1.8942097E38F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)1.3412202E38F);
                Debug.Assert(pack.flow_comp_m_x == (float)1.0996869E38F);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -2.0285146E38F);
                Debug.Assert(pack.flow_comp_m_y == (float) -1.2895876E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)40);
                Debug.Assert(pack.flow_y == (short)(short) -9851);
                Debug.Assert(pack.flow_x == (short)(short)2211);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_comp_m_y = (float) -1.2895876E38F;
            p100.flow_rate_x_SET((float) -2.0285146E38F, PH) ;
            p100.quality = (byte)(byte)194;
            p100.flow_rate_y_SET((float)1.3412202E38F, PH) ;
            p100.flow_y = (short)(short) -9851;
            p100.time_usec = (ulong)3927762197109102431L;
            p100.ground_distance = (float)1.8942097E38F;
            p100.sensor_id = (byte)(byte)40;
            p100.flow_x = (short)(short)2211;
            p100.flow_comp_m_x = (float)1.0996869E38F;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)6.871718E37F);
                Debug.Assert(pack.yaw == (float)2.6471386E38F);
                Debug.Assert(pack.roll == (float) -2.096126E38F);
                Debug.Assert(pack.y == (float)7.184704E37F);
                Debug.Assert(pack.pitch == (float)1.06537E38F);
                Debug.Assert(pack.x == (float) -2.2039927E38F);
                Debug.Assert(pack.usec == (ulong)6473480304862000720L);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.z = (float)6.871718E37F;
            p101.usec = (ulong)6473480304862000720L;
            p101.roll = (float) -2.096126E38F;
            p101.y = (float)7.184704E37F;
            p101.pitch = (float)1.06537E38F;
            p101.x = (float) -2.2039927E38F;
            p101.yaw = (float)2.6471386E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)1.9769027E38F);
                Debug.Assert(pack.z == (float) -1.6029082E38F);
                Debug.Assert(pack.y == (float)1.696301E38F);
                Debug.Assert(pack.x == (float) -1.1753703E37F);
                Debug.Assert(pack.yaw == (float)1.9362896E38F);
                Debug.Assert(pack.roll == (float)2.6101013E38F);
                Debug.Assert(pack.usec == (ulong)4206705437699898464L);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.z = (float) -1.6029082E38F;
            p102.usec = (ulong)4206705437699898464L;
            p102.x = (float) -1.1753703E37F;
            p102.pitch = (float)1.9769027E38F;
            p102.roll = (float)2.6101013E38F;
            p102.yaw = (float)1.9362896E38F;
            p102.y = (float)1.696301E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)7298466968412549352L);
                Debug.Assert(pack.z == (float)1.5645596E38F);
                Debug.Assert(pack.x == (float) -2.5486704E38F);
                Debug.Assert(pack.y == (float)8.870495E37F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.x = (float) -2.5486704E38F;
            p103.z = (float)1.5645596E38F;
            p103.usec = (ulong)7298466968412549352L;
            p103.y = (float)8.870495E37F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)2.5491304E38F);
                Debug.Assert(pack.roll == (float) -8.2266604E37F);
                Debug.Assert(pack.pitch == (float)1.7802277E38F);
                Debug.Assert(pack.z == (float)7.277989E37F);
                Debug.Assert(pack.yaw == (float) -1.3300697E38F);
                Debug.Assert(pack.usec == (ulong)4073094302345415876L);
                Debug.Assert(pack.x == (float) -2.204842E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.x = (float) -2.204842E38F;
            p104.roll = (float) -8.2266604E37F;
            p104.z = (float)7.277989E37F;
            p104.usec = (ulong)4073094302345415876L;
            p104.y = (float)2.5491304E38F;
            p104.yaw = (float) -1.3300697E38F;
            p104.pitch = (float)1.7802277E38F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fields_updated == (ushort)(ushort)56929);
                Debug.Assert(pack.xmag == (float) -1.9290496E38F);
                Debug.Assert(pack.zacc == (float)7.600534E37F);
                Debug.Assert(pack.zgyro == (float)2.336483E38F);
                Debug.Assert(pack.ygyro == (float) -5.2108203E37F);
                Debug.Assert(pack.time_usec == (ulong)8698154283868005540L);
                Debug.Assert(pack.abs_pressure == (float) -3.092088E38F);
                Debug.Assert(pack.pressure_alt == (float)2.4517192E38F);
                Debug.Assert(pack.yacc == (float)2.86966E38F);
                Debug.Assert(pack.xacc == (float)1.5608325E38F);
                Debug.Assert(pack.diff_pressure == (float) -9.497634E37F);
                Debug.Assert(pack.zmag == (float) -2.95856E38F);
                Debug.Assert(pack.temperature == (float) -3.24621E38F);
                Debug.Assert(pack.ymag == (float) -1.0018507E38F);
                Debug.Assert(pack.xgyro == (float) -2.3997109E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.zacc = (float)7.600534E37F;
            p105.time_usec = (ulong)8698154283868005540L;
            p105.yacc = (float)2.86966E38F;
            p105.temperature = (float) -3.24621E38F;
            p105.ygyro = (float) -5.2108203E37F;
            p105.pressure_alt = (float)2.4517192E38F;
            p105.fields_updated = (ushort)(ushort)56929;
            p105.zgyro = (float)2.336483E38F;
            p105.zmag = (float) -2.95856E38F;
            p105.abs_pressure = (float) -3.092088E38F;
            p105.ymag = (float) -1.0018507E38F;
            p105.xgyro = (float) -2.3997109E38F;
            p105.xacc = (float)1.5608325E38F;
            p105.xmag = (float) -1.9290496E38F;
            p105.diff_pressure = (float) -9.497634E37F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_x == (float) -1.1713231E38F);
                Debug.Assert(pack.integrated_xgyro == (float) -1.980248E38F);
                Debug.Assert(pack.integration_time_us == (uint)1605722653U);
                Debug.Assert(pack.integrated_zgyro == (float)1.8586802E38F);
                Debug.Assert(pack.integrated_ygyro == (float) -9.219139E37F);
                Debug.Assert(pack.time_usec == (ulong)2531693466099153226L);
                Debug.Assert(pack.temperature == (short)(short) -19070);
                Debug.Assert(pack.time_delta_distance_us == (uint)3416029817U);
                Debug.Assert(pack.distance == (float) -1.3640524E38F);
                Debug.Assert(pack.integrated_y == (float) -3.9304186E37F);
                Debug.Assert(pack.quality == (byte)(byte)142);
                Debug.Assert(pack.sensor_id == (byte)(byte)124);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_y = (float) -3.9304186E37F;
            p106.integrated_xgyro = (float) -1.980248E38F;
            p106.time_delta_distance_us = (uint)3416029817U;
            p106.temperature = (short)(short) -19070;
            p106.quality = (byte)(byte)142;
            p106.time_usec = (ulong)2531693466099153226L;
            p106.integrated_x = (float) -1.1713231E38F;
            p106.distance = (float) -1.3640524E38F;
            p106.integration_time_us = (uint)1605722653U;
            p106.sensor_id = (byte)(byte)124;
            p106.integrated_ygyro = (float) -9.219139E37F;
            p106.integrated_zgyro = (float)1.8586802E38F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pressure_alt == (float)2.2163358E38F);
                Debug.Assert(pack.xmag == (float)8.353308E37F);
                Debug.Assert(pack.zgyro == (float)1.2364721E38F);
                Debug.Assert(pack.zacc == (float)2.5348674E38F);
                Debug.Assert(pack.yacc == (float) -2.516551E38F);
                Debug.Assert(pack.xgyro == (float) -2.5827422E38F);
                Debug.Assert(pack.zmag == (float) -2.9949339E38F);
                Debug.Assert(pack.ymag == (float)3.4017903E38F);
                Debug.Assert(pack.time_usec == (ulong)7039374304440116889L);
                Debug.Assert(pack.temperature == (float)1.6075995E38F);
                Debug.Assert(pack.ygyro == (float)1.8066807E37F);
                Debug.Assert(pack.diff_pressure == (float) -1.5723446E38F);
                Debug.Assert(pack.fields_updated == (uint)2028879383U);
                Debug.Assert(pack.xacc == (float) -1.6448354E38F);
                Debug.Assert(pack.abs_pressure == (float)2.2120645E38F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.abs_pressure = (float)2.2120645E38F;
            p107.xmag = (float)8.353308E37F;
            p107.fields_updated = (uint)2028879383U;
            p107.ymag = (float)3.4017903E38F;
            p107.zmag = (float) -2.9949339E38F;
            p107.pressure_alt = (float)2.2163358E38F;
            p107.ygyro = (float)1.8066807E37F;
            p107.time_usec = (ulong)7039374304440116889L;
            p107.xacc = (float) -1.6448354E38F;
            p107.xgyro = (float) -2.5827422E38F;
            p107.zacc = (float)2.5348674E38F;
            p107.diff_pressure = (float) -1.5723446E38F;
            p107.zgyro = (float)1.2364721E38F;
            p107.yacc = (float) -2.516551E38F;
            p107.temperature = (float)1.6075995E38F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ve == (float) -3.3597521E38F);
                Debug.Assert(pack.zgyro == (float)5.051772E37F);
                Debug.Assert(pack.alt == (float) -8.647679E37F);
                Debug.Assert(pack.q4 == (float)2.59656E38F);
                Debug.Assert(pack.vd == (float)2.9637438E38F);
                Debug.Assert(pack.q3 == (float)2.3446936E38F);
                Debug.Assert(pack.xacc == (float)1.9400612E38F);
                Debug.Assert(pack.zacc == (float) -2.606889E38F);
                Debug.Assert(pack.yaw == (float) -3.1148982E38F);
                Debug.Assert(pack.lat == (float) -3.12934E37F);
                Debug.Assert(pack.std_dev_horz == (float)1.5172072E38F);
                Debug.Assert(pack.xgyro == (float)3.2980689E38F);
                Debug.Assert(pack.q1 == (float) -1.743832E38F);
                Debug.Assert(pack.std_dev_vert == (float)8.604304E37F);
                Debug.Assert(pack.q2 == (float)3.259828E38F);
                Debug.Assert(pack.pitch == (float) -2.2076916E37F);
                Debug.Assert(pack.lon == (float)2.725694E38F);
                Debug.Assert(pack.vn == (float) -2.2007893E38F);
                Debug.Assert(pack.roll == (float)3.3664494E38F);
                Debug.Assert(pack.yacc == (float) -1.9832598E38F);
                Debug.Assert(pack.ygyro == (float)4.0390423E37F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float) -1.743832E38F;
            p108.std_dev_vert = (float)8.604304E37F;
            p108.q2 = (float)3.259828E38F;
            p108.xacc = (float)1.9400612E38F;
            p108.std_dev_horz = (float)1.5172072E38F;
            p108.yaw = (float) -3.1148982E38F;
            p108.yacc = (float) -1.9832598E38F;
            p108.q3 = (float)2.3446936E38F;
            p108.ve = (float) -3.3597521E38F;
            p108.zacc = (float) -2.606889E38F;
            p108.roll = (float)3.3664494E38F;
            p108.pitch = (float) -2.2076916E37F;
            p108.vd = (float)2.9637438E38F;
            p108.zgyro = (float)5.051772E37F;
            p108.ygyro = (float)4.0390423E37F;
            p108.q4 = (float)2.59656E38F;
            p108.vn = (float) -2.2007893E38F;
            p108.lon = (float)2.725694E38F;
            p108.alt = (float) -8.647679E37F;
            p108.xgyro = (float)3.2980689E38F;
            p108.lat = (float) -3.12934E37F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.remnoise == (byte)(byte)228);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)11136);
                Debug.Assert(pack.rssi == (byte)(byte)166);
                Debug.Assert(pack.txbuf == (byte)(byte)171);
                Debug.Assert(pack.noise == (byte)(byte)188);
                Debug.Assert(pack.remrssi == (byte)(byte)255);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)17804);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.fixed_ = (ushort)(ushort)17804;
            p109.rssi = (byte)(byte)166;
            p109.remrssi = (byte)(byte)255;
            p109.remnoise = (byte)(byte)228;
            p109.rxerrors = (ushort)(ushort)11136;
            p109.noise = (byte)(byte)188;
            p109.txbuf = (byte)(byte)171;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)107);
                Debug.Assert(pack.target_component == (byte)(byte)179);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)208, (byte)46, (byte)178, (byte)186, (byte)74, (byte)145, (byte)96, (byte)5, (byte)196, (byte)144, (byte)24, (byte)27, (byte)108, (byte)249, (byte)5, (byte)25, (byte)33, (byte)152, (byte)30, (byte)155, (byte)91, (byte)180, (byte)234, (byte)15, (byte)190, (byte)180, (byte)4, (byte)103, (byte)228, (byte)107, (byte)187, (byte)156, (byte)18, (byte)45, (byte)50, (byte)98, (byte)102, (byte)51, (byte)244, (byte)197, (byte)115, (byte)183, (byte)133, (byte)84, (byte)97, (byte)4, (byte)54, (byte)30, (byte)2, (byte)239, (byte)191, (byte)108, (byte)121, (byte)254, (byte)133, (byte)32, (byte)209, (byte)46, (byte)178, (byte)187, (byte)78, (byte)148, (byte)115, (byte)125, (byte)63, (byte)13, (byte)159, (byte)41, (byte)17, (byte)149, (byte)251, (byte)10, (byte)28, (byte)9, (byte)218, (byte)127, (byte)54, (byte)38, (byte)101, (byte)22, (byte)141, (byte)118, (byte)226, (byte)112, (byte)216, (byte)64, (byte)162, (byte)255, (byte)229, (byte)5, (byte)206, (byte)19, (byte)149, (byte)54, (byte)86, (byte)37, (byte)221, (byte)174, (byte)14, (byte)149, (byte)83, (byte)149, (byte)199, (byte)237, (byte)177, (byte)226, (byte)73, (byte)252, (byte)23, (byte)1, (byte)93, (byte)5, (byte)72, (byte)64, (byte)128, (byte)176, (byte)74, (byte)44, (byte)206, (byte)15, (byte)212, (byte)215, (byte)44, (byte)96, (byte)30, (byte)157, (byte)130, (byte)208, (byte)149, (byte)161, (byte)71, (byte)230, (byte)39, (byte)215, (byte)138, (byte)28, (byte)121, (byte)190, (byte)159, (byte)154, (byte)144, (byte)18, (byte)15, (byte)120, (byte)178, (byte)46, (byte)228, (byte)21, (byte)179, (byte)196, (byte)209, (byte)104, (byte)209, (byte)201, (byte)69, (byte)95, (byte)5, (byte)174, (byte)77, (byte)23, (byte)163, (byte)39, (byte)45, (byte)72, (byte)13, (byte)137, (byte)96, (byte)87, (byte)91, (byte)40, (byte)245, (byte)219, (byte)85, (byte)248, (byte)131, (byte)120, (byte)234, (byte)70, (byte)242, (byte)6, (byte)120, (byte)196, (byte)191, (byte)20, (byte)124, (byte)48, (byte)72, (byte)188, (byte)101, (byte)248, (byte)114, (byte)149, (byte)213, (byte)214, (byte)117, (byte)17, (byte)190, (byte)202, (byte)29, (byte)237, (byte)105, (byte)8, (byte)83, (byte)42, (byte)52, (byte)196, (byte)144, (byte)94, (byte)194, (byte)181, (byte)193, (byte)54, (byte)103, (byte)207, (byte)174, (byte)76, (byte)49, (byte)36, (byte)189, (byte)26, (byte)137, (byte)111, (byte)212, (byte)202, (byte)249, (byte)228, (byte)74, (byte)250, (byte)85, (byte)141, (byte)78, (byte)93, (byte)216, (byte)15, (byte)56, (byte)11, (byte)250, (byte)49, (byte)144, (byte)107, (byte)32, (byte)183, (byte)48, (byte)234, (byte)29, (byte)113, (byte)92, (byte)20, (byte)76, (byte)238, (byte)115}));
                Debug.Assert(pack.target_system == (byte)(byte)82);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.payload_SET(new byte[] {(byte)208, (byte)46, (byte)178, (byte)186, (byte)74, (byte)145, (byte)96, (byte)5, (byte)196, (byte)144, (byte)24, (byte)27, (byte)108, (byte)249, (byte)5, (byte)25, (byte)33, (byte)152, (byte)30, (byte)155, (byte)91, (byte)180, (byte)234, (byte)15, (byte)190, (byte)180, (byte)4, (byte)103, (byte)228, (byte)107, (byte)187, (byte)156, (byte)18, (byte)45, (byte)50, (byte)98, (byte)102, (byte)51, (byte)244, (byte)197, (byte)115, (byte)183, (byte)133, (byte)84, (byte)97, (byte)4, (byte)54, (byte)30, (byte)2, (byte)239, (byte)191, (byte)108, (byte)121, (byte)254, (byte)133, (byte)32, (byte)209, (byte)46, (byte)178, (byte)187, (byte)78, (byte)148, (byte)115, (byte)125, (byte)63, (byte)13, (byte)159, (byte)41, (byte)17, (byte)149, (byte)251, (byte)10, (byte)28, (byte)9, (byte)218, (byte)127, (byte)54, (byte)38, (byte)101, (byte)22, (byte)141, (byte)118, (byte)226, (byte)112, (byte)216, (byte)64, (byte)162, (byte)255, (byte)229, (byte)5, (byte)206, (byte)19, (byte)149, (byte)54, (byte)86, (byte)37, (byte)221, (byte)174, (byte)14, (byte)149, (byte)83, (byte)149, (byte)199, (byte)237, (byte)177, (byte)226, (byte)73, (byte)252, (byte)23, (byte)1, (byte)93, (byte)5, (byte)72, (byte)64, (byte)128, (byte)176, (byte)74, (byte)44, (byte)206, (byte)15, (byte)212, (byte)215, (byte)44, (byte)96, (byte)30, (byte)157, (byte)130, (byte)208, (byte)149, (byte)161, (byte)71, (byte)230, (byte)39, (byte)215, (byte)138, (byte)28, (byte)121, (byte)190, (byte)159, (byte)154, (byte)144, (byte)18, (byte)15, (byte)120, (byte)178, (byte)46, (byte)228, (byte)21, (byte)179, (byte)196, (byte)209, (byte)104, (byte)209, (byte)201, (byte)69, (byte)95, (byte)5, (byte)174, (byte)77, (byte)23, (byte)163, (byte)39, (byte)45, (byte)72, (byte)13, (byte)137, (byte)96, (byte)87, (byte)91, (byte)40, (byte)245, (byte)219, (byte)85, (byte)248, (byte)131, (byte)120, (byte)234, (byte)70, (byte)242, (byte)6, (byte)120, (byte)196, (byte)191, (byte)20, (byte)124, (byte)48, (byte)72, (byte)188, (byte)101, (byte)248, (byte)114, (byte)149, (byte)213, (byte)214, (byte)117, (byte)17, (byte)190, (byte)202, (byte)29, (byte)237, (byte)105, (byte)8, (byte)83, (byte)42, (byte)52, (byte)196, (byte)144, (byte)94, (byte)194, (byte)181, (byte)193, (byte)54, (byte)103, (byte)207, (byte)174, (byte)76, (byte)49, (byte)36, (byte)189, (byte)26, (byte)137, (byte)111, (byte)212, (byte)202, (byte)249, (byte)228, (byte)74, (byte)250, (byte)85, (byte)141, (byte)78, (byte)93, (byte)216, (byte)15, (byte)56, (byte)11, (byte)250, (byte)49, (byte)144, (byte)107, (byte)32, (byte)183, (byte)48, (byte)234, (byte)29, (byte)113, (byte)92, (byte)20, (byte)76, (byte)238, (byte)115}, 0) ;
            p110.target_component = (byte)(byte)179;
            p110.target_network = (byte)(byte)107;
            p110.target_system = (byte)(byte)82;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long)3532464193430306099L);
                Debug.Assert(pack.tc1 == (long)6857854500636478603L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long)3532464193430306099L;
            p111.tc1 = (long)6857854500636478603L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)347188814049046323L);
                Debug.Assert(pack.seq == (uint)3853918767U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)347188814049046323L;
            p112.seq = (uint)3853918767U;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == (byte)(byte)222);
                Debug.Assert(pack.vel == (ushort)(ushort)36563);
                Debug.Assert(pack.vn == (short)(short) -20733);
                Debug.Assert(pack.cog == (ushort)(ushort)30758);
                Debug.Assert(pack.time_usec == (ulong)6746644657154058459L);
                Debug.Assert(pack.vd == (short)(short) -21999);
                Debug.Assert(pack.eph == (ushort)(ushort)28483);
                Debug.Assert(pack.lat == (int)289287573);
                Debug.Assert(pack.lon == (int)1259715511);
                Debug.Assert(pack.alt == (int) -1420691898);
                Debug.Assert(pack.satellites_visible == (byte)(byte)234);
                Debug.Assert(pack.epv == (ushort)(ushort)1030);
                Debug.Assert(pack.ve == (short)(short) -31569);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.vn = (short)(short) -20733;
            p113.ve = (short)(short) -31569;
            p113.satellites_visible = (byte)(byte)234;
            p113.vel = (ushort)(ushort)36563;
            p113.alt = (int) -1420691898;
            p113.eph = (ushort)(ushort)28483;
            p113.fix_type = (byte)(byte)222;
            p113.lat = (int)289287573;
            p113.lon = (int)1259715511;
            p113.time_usec = (ulong)6746644657154058459L;
            p113.vd = (short)(short) -21999;
            p113.epv = (ushort)(ushort)1030;
            p113.cog = (ushort)(ushort)30758;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.distance == (float) -1.3473419E38F);
                Debug.Assert(pack.time_usec == (ulong)2639853886548884735L);
                Debug.Assert(pack.sensor_id == (byte)(byte)80);
                Debug.Assert(pack.temperature == (short)(short)31524);
                Debug.Assert(pack.integration_time_us == (uint)3682235171U);
                Debug.Assert(pack.time_delta_distance_us == (uint)1726676198U);
                Debug.Assert(pack.integrated_y == (float) -1.3860633E38F);
                Debug.Assert(pack.quality == (byte)(byte)136);
                Debug.Assert(pack.integrated_zgyro == (float)1.0088902E38F);
                Debug.Assert(pack.integrated_xgyro == (float) -2.2633979E38F);
                Debug.Assert(pack.integrated_ygyro == (float)6.8174766E37F);
                Debug.Assert(pack.integrated_x == (float)1.5875489E38F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.sensor_id = (byte)(byte)80;
            p114.integrated_x = (float)1.5875489E38F;
            p114.integrated_y = (float) -1.3860633E38F;
            p114.time_delta_distance_us = (uint)1726676198U;
            p114.distance = (float) -1.3473419E38F;
            p114.integrated_zgyro = (float)1.0088902E38F;
            p114.integrated_ygyro = (float)6.8174766E37F;
            p114.quality = (byte)(byte)136;
            p114.temperature = (short)(short)31524;
            p114.integrated_xgyro = (float) -2.2633979E38F;
            p114.integration_time_us = (uint)3682235171U;
            p114.time_usec = (ulong)2639853886548884735L;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)20117);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {2.1394778E38F, 1.2611792E38F, 3.166119E38F, 8.3695125E37F}));
                Debug.Assert(pack.lat == (int) -1156695514);
                Debug.Assert(pack.lon == (int) -861775147);
                Debug.Assert(pack.rollspeed == (float) -2.3831774E38F);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)22829);
                Debug.Assert(pack.yacc == (short)(short) -30027);
                Debug.Assert(pack.vz == (short)(short) -31422);
                Debug.Assert(pack.xacc == (short)(short)19886);
                Debug.Assert(pack.alt == (int) -1236913409);
                Debug.Assert(pack.pitchspeed == (float) -4.7841544E37F);
                Debug.Assert(pack.vy == (short)(short)9042);
                Debug.Assert(pack.time_usec == (ulong)6021217058006053856L);
                Debug.Assert(pack.yawspeed == (float) -2.0650811E38F);
                Debug.Assert(pack.vx == (short)(short) -24134);
                Debug.Assert(pack.zacc == (short)(short) -7637);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.vy = (short)(short)9042;
            p115.yacc = (short)(short) -30027;
            p115.pitchspeed = (float) -4.7841544E37F;
            p115.lat = (int) -1156695514;
            p115.true_airspeed = (ushort)(ushort)22829;
            p115.vx = (short)(short) -24134;
            p115.time_usec = (ulong)6021217058006053856L;
            p115.rollspeed = (float) -2.3831774E38F;
            p115.ind_airspeed = (ushort)(ushort)20117;
            p115.attitude_quaternion_SET(new float[] {2.1394778E38F, 1.2611792E38F, 3.166119E38F, 8.3695125E37F}, 0) ;
            p115.vz = (short)(short) -31422;
            p115.lon = (int) -861775147;
            p115.xacc = (short)(short)19886;
            p115.yawspeed = (float) -2.0650811E38F;
            p115.alt = (int) -1236913409;
            p115.zacc = (short)(short) -7637;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short)6072);
                Debug.Assert(pack.zgyro == (short)(short) -30753);
                Debug.Assert(pack.yacc == (short)(short) -5739);
                Debug.Assert(pack.zmag == (short)(short) -7735);
                Debug.Assert(pack.xgyro == (short)(short) -14247);
                Debug.Assert(pack.ygyro == (short)(short)7046);
                Debug.Assert(pack.time_boot_ms == (uint)544734936U);
                Debug.Assert(pack.ymag == (short)(short) -2939);
                Debug.Assert(pack.xacc == (short)(short)30064);
                Debug.Assert(pack.xmag == (short)(short) -12486);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.yacc = (short)(short) -5739;
            p116.time_boot_ms = (uint)544734936U;
            p116.xgyro = (short)(short) -14247;
            p116.zgyro = (short)(short) -30753;
            p116.xmag = (short)(short) -12486;
            p116.xacc = (short)(short)30064;
            p116.ygyro = (short)(short)7046;
            p116.zacc = (short)(short)6072;
            p116.ymag = (short)(short) -2939;
            p116.zmag = (short)(short) -7735;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)164);
                Debug.Assert(pack.start == (ushort)(ushort)61705);
                Debug.Assert(pack.end == (ushort)(ushort)6344);
                Debug.Assert(pack.target_component == (byte)(byte)74);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)6344;
            p117.start = (ushort)(ushort)61705;
            p117.target_system = (byte)(byte)164;
            p117.target_component = (byte)(byte)74;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.size == (uint)955363261U);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)13793);
                Debug.Assert(pack.num_logs == (ushort)(ushort)60697);
                Debug.Assert(pack.time_utc == (uint)2400567882U);
                Debug.Assert(pack.id == (ushort)(ushort)6833);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)6833;
            p118.last_log_num = (ushort)(ushort)13793;
            p118.time_utc = (uint)2400567882U;
            p118.num_logs = (ushort)(ushort)60697;
            p118.size = (uint)955363261U;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)43012);
                Debug.Assert(pack.target_system == (byte)(byte)72);
                Debug.Assert(pack.count == (uint)1010282443U);
                Debug.Assert(pack.target_component == (byte)(byte)91);
                Debug.Assert(pack.ofs == (uint)3301554237U);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.count = (uint)1010282443U;
            p119.target_component = (byte)(byte)91;
            p119.ofs = (uint)3301554237U;
            p119.target_system = (byte)(byte)72;
            p119.id = (ushort)(ushort)43012;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)17, (byte)124, (byte)245, (byte)251, (byte)118, (byte)16, (byte)233, (byte)144, (byte)74, (byte)58, (byte)27, (byte)185, (byte)130, (byte)78, (byte)205, (byte)91, (byte)203, (byte)190, (byte)191, (byte)14, (byte)237, (byte)217, (byte)224, (byte)214, (byte)164, (byte)80, (byte)237, (byte)74, (byte)146, (byte)239, (byte)134, (byte)234, (byte)218, (byte)110, (byte)150, (byte)199, (byte)6, (byte)29, (byte)248, (byte)9, (byte)118, (byte)95, (byte)94, (byte)237, (byte)251, (byte)54, (byte)183, (byte)45, (byte)50, (byte)82, (byte)85, (byte)101, (byte)134, (byte)250, (byte)234, (byte)18, (byte)143, (byte)12, (byte)17, (byte)221, (byte)91, (byte)0, (byte)145, (byte)21, (byte)46, (byte)163, (byte)161, (byte)6, (byte)132, (byte)114, (byte)104, (byte)93, (byte)138, (byte)118, (byte)45, (byte)47, (byte)21, (byte)248, (byte)80, (byte)13, (byte)67, (byte)57, (byte)88, (byte)246, (byte)250, (byte)159, (byte)142, (byte)54, (byte)69, (byte)44}));
                Debug.Assert(pack.ofs == (uint)532212202U);
                Debug.Assert(pack.count == (byte)(byte)78);
                Debug.Assert(pack.id == (ushort)(ushort)37663);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.count = (byte)(byte)78;
            p120.data__SET(new byte[] {(byte)17, (byte)124, (byte)245, (byte)251, (byte)118, (byte)16, (byte)233, (byte)144, (byte)74, (byte)58, (byte)27, (byte)185, (byte)130, (byte)78, (byte)205, (byte)91, (byte)203, (byte)190, (byte)191, (byte)14, (byte)237, (byte)217, (byte)224, (byte)214, (byte)164, (byte)80, (byte)237, (byte)74, (byte)146, (byte)239, (byte)134, (byte)234, (byte)218, (byte)110, (byte)150, (byte)199, (byte)6, (byte)29, (byte)248, (byte)9, (byte)118, (byte)95, (byte)94, (byte)237, (byte)251, (byte)54, (byte)183, (byte)45, (byte)50, (byte)82, (byte)85, (byte)101, (byte)134, (byte)250, (byte)234, (byte)18, (byte)143, (byte)12, (byte)17, (byte)221, (byte)91, (byte)0, (byte)145, (byte)21, (byte)46, (byte)163, (byte)161, (byte)6, (byte)132, (byte)114, (byte)104, (byte)93, (byte)138, (byte)118, (byte)45, (byte)47, (byte)21, (byte)248, (byte)80, (byte)13, (byte)67, (byte)57, (byte)88, (byte)246, (byte)250, (byte)159, (byte)142, (byte)54, (byte)69, (byte)44}, 0) ;
            p120.id = (ushort)(ushort)37663;
            p120.ofs = (uint)532212202U;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)141);
                Debug.Assert(pack.target_component == (byte)(byte)112);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)112;
            p121.target_system = (byte)(byte)141;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)187);
                Debug.Assert(pack.target_component == (byte)(byte)155);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)187;
            p122.target_component = (byte)(byte)155;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)25, (byte)15, (byte)198, (byte)171, (byte)233, (byte)169, (byte)162, (byte)120, (byte)58, (byte)152, (byte)253, (byte)147, (byte)4, (byte)244, (byte)112, (byte)112, (byte)33, (byte)133, (byte)151, (byte)36, (byte)218, (byte)241, (byte)199, (byte)207, (byte)24, (byte)136, (byte)200, (byte)203, (byte)165, (byte)250, (byte)139, (byte)222, (byte)11, (byte)55, (byte)84, (byte)214, (byte)82, (byte)61, (byte)50, (byte)252, (byte)121, (byte)23, (byte)20, (byte)103, (byte)13, (byte)221, (byte)11, (byte)58, (byte)23, (byte)204, (byte)112, (byte)64, (byte)14, (byte)97, (byte)23, (byte)1, (byte)100, (byte)230, (byte)137, (byte)217, (byte)237, (byte)93, (byte)226, (byte)1, (byte)4, (byte)32, (byte)151, (byte)228, (byte)148, (byte)166, (byte)166, (byte)63, (byte)50, (byte)6, (byte)193, (byte)5, (byte)74, (byte)166, (byte)42, (byte)227, (byte)33, (byte)226, (byte)154, (byte)225, (byte)65, (byte)220, (byte)152, (byte)202, (byte)191, (byte)162, (byte)180, (byte)87, (byte)149, (byte)25, (byte)198, (byte)76, (byte)118, (byte)171, (byte)6, (byte)188, (byte)94, (byte)15, (byte)234, (byte)213, (byte)107, (byte)88, (byte)187, (byte)21, (byte)88, (byte)248}));
                Debug.Assert(pack.target_component == (byte)(byte)67);
                Debug.Assert(pack.len == (byte)(byte)153);
                Debug.Assert(pack.target_system == (byte)(byte)182);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.data__SET(new byte[] {(byte)25, (byte)15, (byte)198, (byte)171, (byte)233, (byte)169, (byte)162, (byte)120, (byte)58, (byte)152, (byte)253, (byte)147, (byte)4, (byte)244, (byte)112, (byte)112, (byte)33, (byte)133, (byte)151, (byte)36, (byte)218, (byte)241, (byte)199, (byte)207, (byte)24, (byte)136, (byte)200, (byte)203, (byte)165, (byte)250, (byte)139, (byte)222, (byte)11, (byte)55, (byte)84, (byte)214, (byte)82, (byte)61, (byte)50, (byte)252, (byte)121, (byte)23, (byte)20, (byte)103, (byte)13, (byte)221, (byte)11, (byte)58, (byte)23, (byte)204, (byte)112, (byte)64, (byte)14, (byte)97, (byte)23, (byte)1, (byte)100, (byte)230, (byte)137, (byte)217, (byte)237, (byte)93, (byte)226, (byte)1, (byte)4, (byte)32, (byte)151, (byte)228, (byte)148, (byte)166, (byte)166, (byte)63, (byte)50, (byte)6, (byte)193, (byte)5, (byte)74, (byte)166, (byte)42, (byte)227, (byte)33, (byte)226, (byte)154, (byte)225, (byte)65, (byte)220, (byte)152, (byte)202, (byte)191, (byte)162, (byte)180, (byte)87, (byte)149, (byte)25, (byte)198, (byte)76, (byte)118, (byte)171, (byte)6, (byte)188, (byte)94, (byte)15, (byte)234, (byte)213, (byte)107, (byte)88, (byte)187, (byte)21, (byte)88, (byte)248}, 0) ;
            p123.target_system = (byte)(byte)182;
            p123.len = (byte)(byte)153;
            p123.target_component = (byte)(byte)67;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)71);
                Debug.Assert(pack.cog == (ushort)(ushort)64211);
                Debug.Assert(pack.lat == (int)1175756961);
                Debug.Assert(pack.dgps_age == (uint)1763924331U);
                Debug.Assert(pack.epv == (ushort)(ushort)21285);
                Debug.Assert(pack.vel == (ushort)(ushort)11641);
                Debug.Assert(pack.lon == (int) -875704882);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.alt == (int) -1752613510);
                Debug.Assert(pack.dgps_numch == (byte)(byte)58);
                Debug.Assert(pack.eph == (ushort)(ushort)9228);
                Debug.Assert(pack.time_usec == (ulong)5948245311491662685L);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)5948245311491662685L;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p124.vel = (ushort)(ushort)11641;
            p124.eph = (ushort)(ushort)9228;
            p124.lon = (int) -875704882;
            p124.cog = (ushort)(ushort)64211;
            p124.lat = (int)1175756961;
            p124.alt = (int) -1752613510;
            p124.dgps_age = (uint)1763924331U;
            p124.epv = (ushort)(ushort)21285;
            p124.satellites_visible = (byte)(byte)71;
            p124.dgps_numch = (byte)(byte)58;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)40256);
                Debug.Assert(pack.Vservo == (ushort)(ushort)5943);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID;
            p125.Vcc = (ushort)(ushort)40256;
            p125.Vservo = (ushort)(ushort)5943;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
                Debug.Assert(pack.count == (byte)(byte)248);
                Debug.Assert(pack.timeout == (ushort)(ushort)30772);
                Debug.Assert(pack.baudrate == (uint)1853600945U);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)34, (byte)67, (byte)255, (byte)191, (byte)40, (byte)222, (byte)87, (byte)245, (byte)18, (byte)250, (byte)127, (byte)196, (byte)207, (byte)206, (byte)57, (byte)32, (byte)134, (byte)148, (byte)212, (byte)248, (byte)240, (byte)174, (byte)68, (byte)140, (byte)189, (byte)164, (byte)138, (byte)84, (byte)245, (byte)174, (byte)14, (byte)199, (byte)237, (byte)12, (byte)86, (byte)155, (byte)185, (byte)186, (byte)130, (byte)62, (byte)221, (byte)12, (byte)134, (byte)40, (byte)85, (byte)22, (byte)37, (byte)138, (byte)78, (byte)133, (byte)104, (byte)35, (byte)68, (byte)64, (byte)139, (byte)68, (byte)164, (byte)48, (byte)167, (byte)237, (byte)53, (byte)75, (byte)47, (byte)162, (byte)91, (byte)216, (byte)52, (byte)135, (byte)84, (byte)150}));
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.baudrate = (uint)1853600945U;
            p126.count = (byte)(byte)248;
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING;
            p126.timeout = (ushort)(ushort)30772;
            p126.data__SET(new byte[] {(byte)34, (byte)67, (byte)255, (byte)191, (byte)40, (byte)222, (byte)87, (byte)245, (byte)18, (byte)250, (byte)127, (byte)196, (byte)207, (byte)206, (byte)57, (byte)32, (byte)134, (byte)148, (byte)212, (byte)248, (byte)240, (byte)174, (byte)68, (byte)140, (byte)189, (byte)164, (byte)138, (byte)84, (byte)245, (byte)174, (byte)14, (byte)199, (byte)237, (byte)12, (byte)86, (byte)155, (byte)185, (byte)186, (byte)130, (byte)62, (byte)221, (byte)12, (byte)134, (byte)40, (byte)85, (byte)22, (byte)37, (byte)138, (byte)78, (byte)133, (byte)104, (byte)35, (byte)68, (byte)64, (byte)139, (byte)68, (byte)164, (byte)48, (byte)167, (byte)237, (byte)53, (byte)75, (byte)47, (byte)162, (byte)91, (byte)216, (byte)52, (byte)135, (byte)84, (byte)150}, 0) ;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tow == (uint)1551153130U);
                Debug.Assert(pack.wn == (ushort)(ushort)7214);
                Debug.Assert(pack.time_last_baseline_ms == (uint)929248872U);
                Debug.Assert(pack.accuracy == (uint)756596738U);
                Debug.Assert(pack.nsats == (byte)(byte)219);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)96);
                Debug.Assert(pack.rtk_health == (byte)(byte)136);
                Debug.Assert(pack.baseline_b_mm == (int) -1271517560);
                Debug.Assert(pack.baseline_c_mm == (int)879015083);
                Debug.Assert(pack.baseline_a_mm == (int) -1978098177);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)232);
                Debug.Assert(pack.rtk_rate == (byte)(byte)79);
                Debug.Assert(pack.iar_num_hypotheses == (int)1501827832);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)929248872U;
            p127.baseline_b_mm = (int) -1271517560;
            p127.rtk_health = (byte)(byte)136;
            p127.rtk_rate = (byte)(byte)79;
            p127.baseline_c_mm = (int)879015083;
            p127.tow = (uint)1551153130U;
            p127.nsats = (byte)(byte)219;
            p127.wn = (ushort)(ushort)7214;
            p127.accuracy = (uint)756596738U;
            p127.rtk_receiver_id = (byte)(byte)232;
            p127.baseline_a_mm = (int) -1978098177;
            p127.iar_num_hypotheses = (int)1501827832;
            p127.baseline_coords_type = (byte)(byte)96;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rtk_rate == (byte)(byte)181);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)21);
                Debug.Assert(pack.wn == (ushort)(ushort)11883);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)167);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2064028866U);
                Debug.Assert(pack.baseline_a_mm == (int)1408979072);
                Debug.Assert(pack.baseline_b_mm == (int) -1717137420);
                Debug.Assert(pack.accuracy == (uint)1818858095U);
                Debug.Assert(pack.rtk_health == (byte)(byte)99);
                Debug.Assert(pack.tow == (uint)806491720U);
                Debug.Assert(pack.baseline_c_mm == (int)1628357394);
                Debug.Assert(pack.nsats == (byte)(byte)67);
                Debug.Assert(pack.iar_num_hypotheses == (int)2102684685);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.tow = (uint)806491720U;
            p128.baseline_coords_type = (byte)(byte)21;
            p128.iar_num_hypotheses = (int)2102684685;
            p128.baseline_a_mm = (int)1408979072;
            p128.time_last_baseline_ms = (uint)2064028866U;
            p128.rtk_receiver_id = (byte)(byte)167;
            p128.baseline_b_mm = (int) -1717137420;
            p128.rtk_health = (byte)(byte)99;
            p128.wn = (ushort)(ushort)11883;
            p128.nsats = (byte)(byte)67;
            p128.baseline_c_mm = (int)1628357394;
            p128.accuracy = (uint)1818858095U;
            p128.rtk_rate = (byte)(byte)181;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short)3976);
                Debug.Assert(pack.yacc == (short)(short)8132);
                Debug.Assert(pack.xmag == (short)(short)18676);
                Debug.Assert(pack.ymag == (short)(short) -21268);
                Debug.Assert(pack.xgyro == (short)(short)17842);
                Debug.Assert(pack.xacc == (short)(short) -20396);
                Debug.Assert(pack.zgyro == (short)(short)25482);
                Debug.Assert(pack.zmag == (short)(short) -21805);
                Debug.Assert(pack.zacc == (short)(short) -14018);
                Debug.Assert(pack.time_boot_ms == (uint)687468792U);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.yacc = (short)(short)8132;
            p129.xmag = (short)(short)18676;
            p129.time_boot_ms = (uint)687468792U;
            p129.zmag = (short)(short) -21805;
            p129.xgyro = (short)(short)17842;
            p129.ygyro = (short)(short)3976;
            p129.zacc = (short)(short) -14018;
            p129.ymag = (short)(short) -21268;
            p129.xacc = (short)(short) -20396;
            p129.zgyro = (short)(short)25482;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.height == (ushort)(ushort)57926);
                Debug.Assert(pack.width == (ushort)(ushort)33183);
                Debug.Assert(pack.payload == (byte)(byte)201);
                Debug.Assert(pack.packets == (ushort)(ushort)55985);
                Debug.Assert(pack.type == (byte)(byte)29);
                Debug.Assert(pack.size == (uint)2917512890U);
                Debug.Assert(pack.jpg_quality == (byte)(byte)21);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.packets = (ushort)(ushort)55985;
            p130.jpg_quality = (byte)(byte)21;
            p130.width = (ushort)(ushort)33183;
            p130.size = (uint)2917512890U;
            p130.payload = (byte)(byte)201;
            p130.height = (ushort)(ushort)57926;
            p130.type = (byte)(byte)29;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)110, (byte)193, (byte)117, (byte)61, (byte)93, (byte)167, (byte)129, (byte)182, (byte)108, (byte)43, (byte)36, (byte)202, (byte)61, (byte)226, (byte)244, (byte)56, (byte)68, (byte)224, (byte)228, (byte)137, (byte)243, (byte)41, (byte)32, (byte)167, (byte)0, (byte)200, (byte)149, (byte)115, (byte)81, (byte)86, (byte)38, (byte)40, (byte)168, (byte)238, (byte)198, (byte)27, (byte)169, (byte)180, (byte)135, (byte)204, (byte)228, (byte)51, (byte)144, (byte)0, (byte)31, (byte)23, (byte)96, (byte)165, (byte)14, (byte)163, (byte)233, (byte)40, (byte)229, (byte)190, (byte)96, (byte)171, (byte)122, (byte)113, (byte)153, (byte)255, (byte)226, (byte)170, (byte)32, (byte)173, (byte)151, (byte)149, (byte)112, (byte)205, (byte)210, (byte)221, (byte)224, (byte)65, (byte)119, (byte)3, (byte)240, (byte)63, (byte)44, (byte)253, (byte)204, (byte)155, (byte)192, (byte)255, (byte)55, (byte)215, (byte)73, (byte)112, (byte)14, (byte)148, (byte)235, (byte)168, (byte)40, (byte)87, (byte)100, (byte)237, (byte)65, (byte)119, (byte)5, (byte)104, (byte)199, (byte)247, (byte)25, (byte)37, (byte)131, (byte)246, (byte)115, (byte)156, (byte)254, (byte)75, (byte)207, (byte)99, (byte)108, (byte)94, (byte)111, (byte)155, (byte)199, (byte)215, (byte)134, (byte)118, (byte)231, (byte)110, (byte)62, (byte)210, (byte)210, (byte)91, (byte)146, (byte)87, (byte)212, (byte)134, (byte)99, (byte)186, (byte)159, (byte)72, (byte)9, (byte)243, (byte)58, (byte)91, (byte)116, (byte)30, (byte)46, (byte)125, (byte)153, (byte)94, (byte)126, (byte)56, (byte)104, (byte)77, (byte)169, (byte)96, (byte)88, (byte)130, (byte)200, (byte)14, (byte)224, (byte)216, (byte)165, (byte)103, (byte)255, (byte)254, (byte)115, (byte)241, (byte)80, (byte)232, (byte)235, (byte)201, (byte)32, (byte)104, (byte)72, (byte)202, (byte)89, (byte)171, (byte)206, (byte)144, (byte)222, (byte)57, (byte)233, (byte)190, (byte)78, (byte)119, (byte)111, (byte)30, (byte)207, (byte)41, (byte)98, (byte)111, (byte)6, (byte)51, (byte)172, (byte)234, (byte)186, (byte)168, (byte)185, (byte)207, (byte)71, (byte)208, (byte)235, (byte)169, (byte)221, (byte)123, (byte)121, (byte)184, (byte)173, (byte)242, (byte)235, (byte)191, (byte)76, (byte)121, (byte)103, (byte)202, (byte)229, (byte)217, (byte)0, (byte)195, (byte)50, (byte)26, (byte)78, (byte)140, (byte)207, (byte)85, (byte)60, (byte)1, (byte)83, (byte)43, (byte)34, (byte)90, (byte)230, (byte)9, (byte)109, (byte)154, (byte)11, (byte)6, (byte)127, (byte)81, (byte)250, (byte)171, (byte)11, (byte)245, (byte)254, (byte)244, (byte)213, (byte)96, (byte)131, (byte)244, (byte)13, (byte)66, (byte)148, (byte)148, (byte)12, (byte)32, (byte)51, (byte)16, (byte)144, (byte)225, (byte)139}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)5120);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)5120;
            p131.data__SET(new byte[] {(byte)110, (byte)193, (byte)117, (byte)61, (byte)93, (byte)167, (byte)129, (byte)182, (byte)108, (byte)43, (byte)36, (byte)202, (byte)61, (byte)226, (byte)244, (byte)56, (byte)68, (byte)224, (byte)228, (byte)137, (byte)243, (byte)41, (byte)32, (byte)167, (byte)0, (byte)200, (byte)149, (byte)115, (byte)81, (byte)86, (byte)38, (byte)40, (byte)168, (byte)238, (byte)198, (byte)27, (byte)169, (byte)180, (byte)135, (byte)204, (byte)228, (byte)51, (byte)144, (byte)0, (byte)31, (byte)23, (byte)96, (byte)165, (byte)14, (byte)163, (byte)233, (byte)40, (byte)229, (byte)190, (byte)96, (byte)171, (byte)122, (byte)113, (byte)153, (byte)255, (byte)226, (byte)170, (byte)32, (byte)173, (byte)151, (byte)149, (byte)112, (byte)205, (byte)210, (byte)221, (byte)224, (byte)65, (byte)119, (byte)3, (byte)240, (byte)63, (byte)44, (byte)253, (byte)204, (byte)155, (byte)192, (byte)255, (byte)55, (byte)215, (byte)73, (byte)112, (byte)14, (byte)148, (byte)235, (byte)168, (byte)40, (byte)87, (byte)100, (byte)237, (byte)65, (byte)119, (byte)5, (byte)104, (byte)199, (byte)247, (byte)25, (byte)37, (byte)131, (byte)246, (byte)115, (byte)156, (byte)254, (byte)75, (byte)207, (byte)99, (byte)108, (byte)94, (byte)111, (byte)155, (byte)199, (byte)215, (byte)134, (byte)118, (byte)231, (byte)110, (byte)62, (byte)210, (byte)210, (byte)91, (byte)146, (byte)87, (byte)212, (byte)134, (byte)99, (byte)186, (byte)159, (byte)72, (byte)9, (byte)243, (byte)58, (byte)91, (byte)116, (byte)30, (byte)46, (byte)125, (byte)153, (byte)94, (byte)126, (byte)56, (byte)104, (byte)77, (byte)169, (byte)96, (byte)88, (byte)130, (byte)200, (byte)14, (byte)224, (byte)216, (byte)165, (byte)103, (byte)255, (byte)254, (byte)115, (byte)241, (byte)80, (byte)232, (byte)235, (byte)201, (byte)32, (byte)104, (byte)72, (byte)202, (byte)89, (byte)171, (byte)206, (byte)144, (byte)222, (byte)57, (byte)233, (byte)190, (byte)78, (byte)119, (byte)111, (byte)30, (byte)207, (byte)41, (byte)98, (byte)111, (byte)6, (byte)51, (byte)172, (byte)234, (byte)186, (byte)168, (byte)185, (byte)207, (byte)71, (byte)208, (byte)235, (byte)169, (byte)221, (byte)123, (byte)121, (byte)184, (byte)173, (byte)242, (byte)235, (byte)191, (byte)76, (byte)121, (byte)103, (byte)202, (byte)229, (byte)217, (byte)0, (byte)195, (byte)50, (byte)26, (byte)78, (byte)140, (byte)207, (byte)85, (byte)60, (byte)1, (byte)83, (byte)43, (byte)34, (byte)90, (byte)230, (byte)9, (byte)109, (byte)154, (byte)11, (byte)6, (byte)127, (byte)81, (byte)250, (byte)171, (byte)11, (byte)245, (byte)254, (byte)244, (byte)213, (byte)96, (byte)131, (byte)244, (byte)13, (byte)66, (byte)148, (byte)148, (byte)12, (byte)32, (byte)51, (byte)16, (byte)144, (byte)225, (byte)139}, 0) ;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_distance == (ushort)(ushort)33499);
                Debug.Assert(pack.id == (byte)(byte)78);
                Debug.Assert(pack.time_boot_ms == (uint)52965157U);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
                Debug.Assert(pack.current_distance == (ushort)(ushort)25228);
                Debug.Assert(pack.covariance == (byte)(byte)18);
                Debug.Assert(pack.max_distance == (ushort)(ushort)182);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p132.current_distance = (ushort)(ushort)25228;
            p132.covariance = (byte)(byte)18;
            p132.id = (byte)(byte)78;
            p132.min_distance = (ushort)(ushort)33499;
            p132.time_boot_ms = (uint)52965157U;
            p132.max_distance = (ushort)(ushort)182;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)52472);
                Debug.Assert(pack.lon == (int)699380716);
                Debug.Assert(pack.lat == (int) -310540157);
                Debug.Assert(pack.mask == (ulong)2441150400370003250L);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.grid_spacing = (ushort)(ushort)52472;
            p133.lon = (int)699380716;
            p133.lat = (int) -310540157;
            p133.mask = (ulong)2441150400370003250L;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short)7568, (short) -23928, (short) -24391, (short) -7136, (short)6153, (short)29558, (short) -3861, (short)27170, (short)4898, (short)16572, (short)31021, (short) -30101, (short)24943, (short)25177, (short) -31781, (short) -12316}));
                Debug.Assert(pack.gridbit == (byte)(byte)194);
                Debug.Assert(pack.lat == (int) -1850769157);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)45816);
                Debug.Assert(pack.lon == (int) -2009996260);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -1850769157;
            p134.data__SET(new short[] {(short)7568, (short) -23928, (short) -24391, (short) -7136, (short)6153, (short)29558, (short) -3861, (short)27170, (short)4898, (short)16572, (short)31021, (short) -30101, (short)24943, (short)25177, (short) -31781, (short) -12316}, 0) ;
            p134.lon = (int) -2009996260;
            p134.gridbit = (byte)(byte)194;
            p134.grid_spacing = (ushort)(ushort)45816;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1254401594);
                Debug.Assert(pack.lon == (int)1774418858);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)1254401594;
            p135.lon = (int)1774418858;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -993711103);
                Debug.Assert(pack.terrain_height == (float) -5.1235304E37F);
                Debug.Assert(pack.current_height == (float)2.0373893E38F);
                Debug.Assert(pack.loaded == (ushort)(ushort)16418);
                Debug.Assert(pack.pending == (ushort)(ushort)52417);
                Debug.Assert(pack.lon == (int)1601811049);
                Debug.Assert(pack.spacing == (ushort)(ushort)23239);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.pending = (ushort)(ushort)52417;
            p136.terrain_height = (float) -5.1235304E37F;
            p136.current_height = (float)2.0373893E38F;
            p136.loaded = (ushort)(ushort)16418;
            p136.lon = (int)1601811049;
            p136.lat = (int) -993711103;
            p136.spacing = (ushort)(ushort)23239;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -22899);
                Debug.Assert(pack.press_abs == (float)2.7000232E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1439822227U);
                Debug.Assert(pack.press_diff == (float)2.3352594E38F);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_diff = (float)2.3352594E38F;
            p137.time_boot_ms = (uint)1439822227U;
            p137.press_abs = (float)2.7000232E38F;
            p137.temperature = (short)(short) -22899;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)1.809891E38F);
                Debug.Assert(pack.y == (float)9.207246E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {4.6320764E37F, 1.23098404E36F, -6.653234E37F, 3.199389E38F}));
                Debug.Assert(pack.x == (float) -1.7136446E38F);
                Debug.Assert(pack.time_usec == (ulong)3342313533923459886L);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.q_SET(new float[] {4.6320764E37F, 1.23098404E36F, -6.653234E37F, 3.199389E38F}, 0) ;
            p138.z = (float)1.809891E38F;
            p138.time_usec = (ulong)3342313533923459886L;
            p138.x = (float) -1.7136446E38F;
            p138.y = (float)9.207246E37F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)168);
                Debug.Assert(pack.group_mlx == (byte)(byte)76);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.5669851E38F, 9.828884E36F, 2.9191213E38F, 3.6137354E37F, -4.307546E37F, -2.6260699E38F, 7.909321E37F, 1.1926654E38F}));
                Debug.Assert(pack.target_system == (byte)(byte)65);
                Debug.Assert(pack.time_usec == (ulong)965119735150544474L);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)965119735150544474L;
            p139.group_mlx = (byte)(byte)76;
            p139.target_component = (byte)(byte)168;
            p139.target_system = (byte)(byte)65;
            p139.controls_SET(new float[] {1.5669851E38F, 9.828884E36F, 2.9191213E38F, 3.6137354E37F, -4.307546E37F, -2.6260699E38F, 7.909321E37F, 1.1926654E38F}, 0) ;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)58);
                Debug.Assert(pack.time_usec == (ulong)8702726400154195408L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.0270869E38F, 2.7123532E38F, 1.6493422E38F, 2.7472607E38F, 3.1401152E38F, -1.0034952E38F, -1.3973036E38F, 2.1982057E38F}));
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)58;
            p140.controls_SET(new float[] {-1.0270869E38F, 2.7123532E38F, 1.6493422E38F, 2.7472607E38F, 3.1401152E38F, -1.0034952E38F, -1.3973036E38F, 2.1982057E38F}, 0) ;
            p140.time_usec = (ulong)8702726400154195408L;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_terrain == (float) -2.1484609E38F);
                Debug.Assert(pack.altitude_amsl == (float)1.2830212E38F);
                Debug.Assert(pack.altitude_monotonic == (float) -2.033383E38F);
                Debug.Assert(pack.altitude_relative == (float) -2.8875671E38F);
                Debug.Assert(pack.bottom_clearance == (float) -2.5757247E38F);
                Debug.Assert(pack.time_usec == (ulong)381432641270009342L);
                Debug.Assert(pack.altitude_local == (float)2.4533568E38F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_local = (float)2.4533568E38F;
            p141.time_usec = (ulong)381432641270009342L;
            p141.altitude_amsl = (float)1.2830212E38F;
            p141.altitude_terrain = (float) -2.1484609E38F;
            p141.bottom_clearance = (float) -2.5757247E38F;
            p141.altitude_monotonic = (float) -2.033383E38F;
            p141.altitude_relative = (float) -2.8875671E38F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)19, (byte)71, (byte)92, (byte)97, (byte)202, (byte)74, (byte)104, (byte)174, (byte)195, (byte)141, (byte)110, (byte)184, (byte)27, (byte)120, (byte)220, (byte)123, (byte)108, (byte)98, (byte)21, (byte)165, (byte)113, (byte)75, (byte)121, (byte)88, (byte)113, (byte)148, (byte)70, (byte)28, (byte)202, (byte)51, (byte)147, (byte)31, (byte)243, (byte)89, (byte)225, (byte)49, (byte)58, (byte)129, (byte)99, (byte)201, (byte)3, (byte)197, (byte)136, (byte)27, (byte)81, (byte)117, (byte)24, (byte)93, (byte)204, (byte)134, (byte)0, (byte)95, (byte)182, (byte)32, (byte)135, (byte)153, (byte)218, (byte)46, (byte)112, (byte)25, (byte)219, (byte)45, (byte)152, (byte)185, (byte)70, (byte)28, (byte)242, (byte)138, (byte)247, (byte)156, (byte)13, (byte)152, (byte)126, (byte)100, (byte)181, (byte)175, (byte)72, (byte)98, (byte)201, (byte)79, (byte)123, (byte)39, (byte)39, (byte)119, (byte)86, (byte)250, (byte)67, (byte)47, (byte)39, (byte)8, (byte)93, (byte)97, (byte)152, (byte)153, (byte)96, (byte)13, (byte)174, (byte)132, (byte)76, (byte)41, (byte)251, (byte)33, (byte)102, (byte)95, (byte)214, (byte)207, (byte)62, (byte)57, (byte)114, (byte)80, (byte)100, (byte)28, (byte)210, (byte)116, (byte)126, (byte)158, (byte)249, (byte)101, (byte)55, (byte)40}));
                Debug.Assert(pack.uri_type == (byte)(byte)112);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)97, (byte)85, (byte)110, (byte)203, (byte)73, (byte)49, (byte)195, (byte)255, (byte)17, (byte)186, (byte)99, (byte)15, (byte)205, (byte)159, (byte)77, (byte)175, (byte)222, (byte)143, (byte)119, (byte)161, (byte)205, (byte)229, (byte)121, (byte)215, (byte)5, (byte)63, (byte)133, (byte)151, (byte)251, (byte)78, (byte)136, (byte)29, (byte)221, (byte)52, (byte)8, (byte)77, (byte)38, (byte)45, (byte)105, (byte)107, (byte)117, (byte)16, (byte)158, (byte)166, (byte)241, (byte)182, (byte)222, (byte)175, (byte)64, (byte)67, (byte)50, (byte)240, (byte)29, (byte)214, (byte)219, (byte)114, (byte)65, (byte)153, (byte)167, (byte)249, (byte)115, (byte)248, (byte)173, (byte)146, (byte)76, (byte)207, (byte)252, (byte)212, (byte)5, (byte)117, (byte)135, (byte)166, (byte)234, (byte)51, (byte)14, (byte)74, (byte)59, (byte)170, (byte)82, (byte)205, (byte)134, (byte)147, (byte)156, (byte)142, (byte)63, (byte)61, (byte)175, (byte)50, (byte)250, (byte)217, (byte)214, (byte)216, (byte)238, (byte)96, (byte)184, (byte)59, (byte)82, (byte)233, (byte)135, (byte)250, (byte)150, (byte)10, (byte)84, (byte)122, (byte)151, (byte)87, (byte)73, (byte)145, (byte)144, (byte)104, (byte)84, (byte)183, (byte)203, (byte)47, (byte)49, (byte)88, (byte)126, (byte)184, (byte)228, (byte)112}));
                Debug.Assert(pack.request_id == (byte)(byte)171);
                Debug.Assert(pack.transfer_type == (byte)(byte)197);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.uri_SET(new byte[] {(byte)97, (byte)85, (byte)110, (byte)203, (byte)73, (byte)49, (byte)195, (byte)255, (byte)17, (byte)186, (byte)99, (byte)15, (byte)205, (byte)159, (byte)77, (byte)175, (byte)222, (byte)143, (byte)119, (byte)161, (byte)205, (byte)229, (byte)121, (byte)215, (byte)5, (byte)63, (byte)133, (byte)151, (byte)251, (byte)78, (byte)136, (byte)29, (byte)221, (byte)52, (byte)8, (byte)77, (byte)38, (byte)45, (byte)105, (byte)107, (byte)117, (byte)16, (byte)158, (byte)166, (byte)241, (byte)182, (byte)222, (byte)175, (byte)64, (byte)67, (byte)50, (byte)240, (byte)29, (byte)214, (byte)219, (byte)114, (byte)65, (byte)153, (byte)167, (byte)249, (byte)115, (byte)248, (byte)173, (byte)146, (byte)76, (byte)207, (byte)252, (byte)212, (byte)5, (byte)117, (byte)135, (byte)166, (byte)234, (byte)51, (byte)14, (byte)74, (byte)59, (byte)170, (byte)82, (byte)205, (byte)134, (byte)147, (byte)156, (byte)142, (byte)63, (byte)61, (byte)175, (byte)50, (byte)250, (byte)217, (byte)214, (byte)216, (byte)238, (byte)96, (byte)184, (byte)59, (byte)82, (byte)233, (byte)135, (byte)250, (byte)150, (byte)10, (byte)84, (byte)122, (byte)151, (byte)87, (byte)73, (byte)145, (byte)144, (byte)104, (byte)84, (byte)183, (byte)203, (byte)47, (byte)49, (byte)88, (byte)126, (byte)184, (byte)228, (byte)112}, 0) ;
            p142.storage_SET(new byte[] {(byte)19, (byte)71, (byte)92, (byte)97, (byte)202, (byte)74, (byte)104, (byte)174, (byte)195, (byte)141, (byte)110, (byte)184, (byte)27, (byte)120, (byte)220, (byte)123, (byte)108, (byte)98, (byte)21, (byte)165, (byte)113, (byte)75, (byte)121, (byte)88, (byte)113, (byte)148, (byte)70, (byte)28, (byte)202, (byte)51, (byte)147, (byte)31, (byte)243, (byte)89, (byte)225, (byte)49, (byte)58, (byte)129, (byte)99, (byte)201, (byte)3, (byte)197, (byte)136, (byte)27, (byte)81, (byte)117, (byte)24, (byte)93, (byte)204, (byte)134, (byte)0, (byte)95, (byte)182, (byte)32, (byte)135, (byte)153, (byte)218, (byte)46, (byte)112, (byte)25, (byte)219, (byte)45, (byte)152, (byte)185, (byte)70, (byte)28, (byte)242, (byte)138, (byte)247, (byte)156, (byte)13, (byte)152, (byte)126, (byte)100, (byte)181, (byte)175, (byte)72, (byte)98, (byte)201, (byte)79, (byte)123, (byte)39, (byte)39, (byte)119, (byte)86, (byte)250, (byte)67, (byte)47, (byte)39, (byte)8, (byte)93, (byte)97, (byte)152, (byte)153, (byte)96, (byte)13, (byte)174, (byte)132, (byte)76, (byte)41, (byte)251, (byte)33, (byte)102, (byte)95, (byte)214, (byte)207, (byte)62, (byte)57, (byte)114, (byte)80, (byte)100, (byte)28, (byte)210, (byte)116, (byte)126, (byte)158, (byte)249, (byte)101, (byte)55, (byte)40}, 0) ;
            p142.uri_type = (byte)(byte)112;
            p142.transfer_type = (byte)(byte)197;
            p142.request_id = (byte)(byte)171;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float)1.3561538E37F);
                Debug.Assert(pack.press_diff == (float) -4.8458997E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3152176241U);
                Debug.Assert(pack.temperature == (short)(short)15027);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.temperature = (short)(short)15027;
            p143.press_diff = (float) -4.8458997E37F;
            p143.press_abs = (float)1.3561538E37F;
            p143.time_boot_ms = (uint)3152176241U;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-2.5430189E38F, -3.0775727E37F, -1.6073124E38F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {1.9844742E38F, -1.3590753E38F, 6.3273816E37F}));
                Debug.Assert(pack.custom_state == (ulong)6239306461560525151L);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {1.5288718E37F, -2.6808133E38F, 1.1799589E38F}));
                Debug.Assert(pack.lon == (int)1157420717);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {5.721389E37F, -6.16363E37F, 2.094234E38F}));
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {2.5851945E38F, -9.640717E37F, -2.5238597E38F, 2.3282072E37F}));
                Debug.Assert(pack.timestamp == (ulong)6184092182541583102L);
                Debug.Assert(pack.alt == (float) -3.386771E38F);
                Debug.Assert(pack.est_capabilities == (byte)(byte)5);
                Debug.Assert(pack.lat == (int) -2144378016);
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.vel_SET(new float[] {1.9844742E38F, -1.3590753E38F, 6.3273816E37F}, 0) ;
            p144.est_capabilities = (byte)(byte)5;
            p144.acc_SET(new float[] {1.5288718E37F, -2.6808133E38F, 1.1799589E38F}, 0) ;
            p144.position_cov_SET(new float[] {5.721389E37F, -6.16363E37F, 2.094234E38F}, 0) ;
            p144.timestamp = (ulong)6184092182541583102L;
            p144.rates_SET(new float[] {-2.5430189E38F, -3.0775727E37F, -1.6073124E38F}, 0) ;
            p144.attitude_q_SET(new float[] {2.5851945E38F, -9.640717E37F, -2.5238597E38F, 2.3282072E37F}, 0) ;
            p144.lat = (int) -2144378016;
            p144.lon = (int)1157420717;
            p144.custom_state = (ulong)6239306461560525151L;
            p144.alt = (float) -3.386771E38F;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y_pos == (float)2.1326503E38F);
                Debug.Assert(pack.y_vel == (float)7.6879703E37F);
                Debug.Assert(pack.x_acc == (float)2.8035821E38F);
                Debug.Assert(pack.x_pos == (float)8.4761346E37F);
                Debug.Assert(pack.pitch_rate == (float) -5.3034384E37F);
                Debug.Assert(pack.roll_rate == (float)1.8770843E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-2.889374E37F, -2.2798793E38F, -4.235283E37F}));
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.5426678E38F, -2.8723896E38F, -9.604974E37F, -2.0948316E38F}));
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-1.13209E38F, 1.6264875E38F, 3.307088E38F}));
                Debug.Assert(pack.airspeed == (float) -8.768271E37F);
                Debug.Assert(pack.z_pos == (float)2.2239153E38F);
                Debug.Assert(pack.y_acc == (float) -2.541982E38F);
                Debug.Assert(pack.x_vel == (float)1.9762186E37F);
                Debug.Assert(pack.time_usec == (ulong)7785966191619884571L);
                Debug.Assert(pack.yaw_rate == (float) -3.0961297E38F);
                Debug.Assert(pack.z_acc == (float)3.1015017E38F);
                Debug.Assert(pack.z_vel == (float)1.2286721E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.x_vel = (float)1.9762186E37F;
            p146.x_acc = (float)2.8035821E38F;
            p146.y_acc = (float) -2.541982E38F;
            p146.x_pos = (float)8.4761346E37F;
            p146.yaw_rate = (float) -3.0961297E38F;
            p146.time_usec = (ulong)7785966191619884571L;
            p146.z_pos = (float)2.2239153E38F;
            p146.y_pos = (float)2.1326503E38F;
            p146.pitch_rate = (float) -5.3034384E37F;
            p146.z_acc = (float)3.1015017E38F;
            p146.airspeed = (float) -8.768271E37F;
            p146.q_SET(new float[] {2.5426678E38F, -2.8723896E38F, -9.604974E37F, -2.0948316E38F}, 0) ;
            p146.z_vel = (float)1.2286721E38F;
            p146.y_vel = (float)7.6879703E37F;
            p146.pos_variance_SET(new float[] {-1.13209E38F, 1.6264875E38F, 3.307088E38F}, 0) ;
            p146.roll_rate = (float)1.8770843E38F;
            p146.vel_variance_SET(new float[] {-2.889374E37F, -2.2798793E38F, -4.235283E37F}, 0) ;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_consumed == (int)727463512);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
                Debug.Assert(pack.energy_consumed == (int)981127411);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)43818, (ushort)5608, (ushort)30781, (ushort)33769, (ushort)15535, (ushort)42578, (ushort)23795, (ushort)38870, (ushort)45298, (ushort)28030}));
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
                Debug.Assert(pack.id == (byte)(byte)238);
                Debug.Assert(pack.current_battery == (short)(short) -14997);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)32);
                Debug.Assert(pack.temperature == (short)(short)3377);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            p147.energy_consumed = (int)981127411;
            p147.battery_remaining = (sbyte)(sbyte)32;
            p147.id = (byte)(byte)238;
            p147.current_battery = (short)(short) -14997;
            p147.temperature = (short)(short)3377;
            p147.voltages_SET(new ushort[] {(ushort)43818, (ushort)5608, (ushort)30781, (ushort)33769, (ushort)15535, (ushort)42578, (ushort)23795, (ushort)38870, (ushort)45298, (ushort)28030}, 0) ;
            p147.current_consumed = (int)727463512;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)95, (byte)133, (byte)35, (byte)82, (byte)59, (byte)212, (byte)162, (byte)20}));
                Debug.Assert(pack.flight_sw_version == (uint)3818102463U);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)200, (byte)70, (byte)13, (byte)169, (byte)152, (byte)206, (byte)156, (byte)166, (byte)9, (byte)100, (byte)64, (byte)187, (byte)158, (byte)48, (byte)159, (byte)66, (byte)238, (byte)56}));
                Debug.Assert(pack.os_sw_version == (uint)12389770U);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)34852);
                Debug.Assert(pack.uid == (ulong)4327912577033666669L);
                Debug.Assert(pack.middleware_sw_version == (uint)3146899794U);
                Debug.Assert(pack.board_version == (uint)2380739805U);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)43, (byte)29, (byte)231, (byte)126, (byte)63, (byte)12, (byte)67, (byte)125}));
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)142, (byte)247, (byte)44, (byte)231, (byte)132, (byte)141, (byte)241, (byte)116}));
                Debug.Assert(pack.product_id == (ushort)(ushort)1408);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.middleware_custom_version_SET(new byte[] {(byte)142, (byte)247, (byte)44, (byte)231, (byte)132, (byte)141, (byte)241, (byte)116}, 0) ;
            p148.uid2_SET(new byte[] {(byte)200, (byte)70, (byte)13, (byte)169, (byte)152, (byte)206, (byte)156, (byte)166, (byte)9, (byte)100, (byte)64, (byte)187, (byte)158, (byte)48, (byte)159, (byte)66, (byte)238, (byte)56}, 0, PH) ;
            p148.product_id = (ushort)(ushort)1408;
            p148.board_version = (uint)2380739805U;
            p148.os_sw_version = (uint)12389770U;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION;
            p148.uid = (ulong)4327912577033666669L;
            p148.vendor_id = (ushort)(ushort)34852;
            p148.middleware_sw_version = (uint)3146899794U;
            p148.flight_sw_version = (uint)3818102463U;
            p148.os_custom_version_SET(new byte[] {(byte)95, (byte)133, (byte)35, (byte)82, (byte)59, (byte)212, (byte)162, (byte)20}, 0) ;
            p148.flight_custom_version_SET(new byte[] {(byte)43, (byte)29, (byte)231, (byte)126, (byte)63, (byte)12, (byte)67, (byte)125}, 0) ;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.distance == (float) -3.4074795E37F);
                Debug.Assert(pack.size_y == (float) -9.634744E37F);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-1.4494906E38F, 7.995518E37F, 3.2203972E38F, 2.2064154E38F}));
                Debug.Assert(pack.angle_x == (float)2.531648E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)230);
                Debug.Assert(pack.angle_y == (float) -1.0862503E38F);
                Debug.Assert(pack.target_num == (byte)(byte)56);
                Debug.Assert(pack.x_TRY(ph) == (float) -3.0319629E38F);
                Debug.Assert(pack.time_usec == (ulong)9100719835828623033L);
                Debug.Assert(pack.y_TRY(ph) == (float) -1.2932418E38F);
                Debug.Assert(pack.size_x == (float) -2.459286E38F);
                Debug.Assert(pack.z_TRY(ph) == (float) -7.88285E37F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.angle_y = (float) -1.0862503E38F;
            p149.time_usec = (ulong)9100719835828623033L;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            p149.y_SET((float) -1.2932418E38F, PH) ;
            p149.z_SET((float) -7.88285E37F, PH) ;
            p149.position_valid_SET((byte)(byte)230, PH) ;
            p149.distance = (float) -3.4074795E37F;
            p149.angle_x = (float)2.531648E38F;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p149.x_SET((float) -3.0319629E38F, PH) ;
            p149.size_x = (float) -2.459286E38F;
            p149.target_num = (byte)(byte)56;
            p149.q_SET(new float[] {-1.4494906E38F, 7.995518E37F, 3.2203972E38F, 2.2064154E38F}, 0, PH) ;
            p149.size_y = (float) -9.634744E37F;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_0Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.v1 == (byte)(byte)53);
                Debug.Assert(pack.ar_u16.SequenceEqual(new ushort[] {(ushort)59454, (ushort)53055, (ushort)50711, (ushort)48328}));
                Debug.Assert(pack.ar_u8.SequenceEqual(new byte[] {(byte)57, (byte)210, (byte)45, (byte)25}));
                Debug.Assert(pack.ar_i8.SequenceEqual(new sbyte[] {(sbyte) - 89, (sbyte) - 117, (sbyte)98, (sbyte) - 113}));
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {2848420170U, 1853811498U, 720204025U, 2393696533U}));
            };
            GroundControl.ARRAY_TEST_0 p150 = CommunicationChannel.new_ARRAY_TEST_0();
            PH.setPack(p150);
            p150.ar_u32_SET(new uint[] {2848420170U, 1853811498U, 720204025U, 2393696533U}, 0) ;
            p150.v1 = (byte)(byte)53;
            p150.ar_u8_SET(new byte[] {(byte)57, (byte)210, (byte)45, (byte)25}, 0) ;
            p150.ar_i8_SET(new sbyte[] {(sbyte) - 89, (sbyte) - 117, (sbyte)98, (sbyte) - 113}, 0) ;
            p150.ar_u16_SET(new ushort[] {(ushort)59454, (ushort)53055, (ushort)50711, (ushort)48328}, 0) ;
            CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_1Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {1136916355U, 2537566362U, 3679412766U, 2857136988U}));
            };
            GroundControl.ARRAY_TEST_1 p151 = CommunicationChannel.new_ARRAY_TEST_1();
            PH.setPack(p151);
            p151.ar_u32_SET(new uint[] {1136916355U, 2537566362U, 3679412766U, 2857136988U}, 0) ;
            CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.v == (byte)(byte)155);
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {983422160U, 1929253664U, 1663583576U, 2821924341U}));
            };
            GroundControl.ARRAY_TEST_3 p153 = CommunicationChannel.new_ARRAY_TEST_3();
            PH.setPack(p153);
            p153.v = (byte)(byte)155;
            p153.ar_u32_SET(new uint[] {983422160U, 1929253664U, 1663583576U, 2821924341U}, 0) ;
            CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_4Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.v == (byte)(byte)22);
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {930090188U, 4008695544U, 2292780448U, 389679308U}));
            };
            GroundControl.ARRAY_TEST_4 p154 = CommunicationChannel.new_ARRAY_TEST_4();
            PH.setPack(p154);
            p154.ar_u32_SET(new uint[] {930090188U, 4008695544U, 2292780448U, 389679308U}, 0) ;
            p154.v = (byte)(byte)22;
            CommunicationChannel.instance.send(p154);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_5Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.c2_LEN(ph) == 1);
                Debug.Assert(pack.c2_TRY(ph).Equals("f"));
                Debug.Assert(pack.c1_LEN(ph) == 1);
                Debug.Assert(pack.c1_TRY(ph).Equals("m"));
            };
            GroundControl.ARRAY_TEST_5 p155 = CommunicationChannel.new_ARRAY_TEST_5();
            PH.setPack(p155);
            p155.c1_SET("m", PH) ;
            p155.c2_SET("f", PH) ;
            CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_6Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_c_LEN(ph) == 20);
                Debug.Assert(pack.ar_c_TRY(ph).Equals("bfhvfvmkzbltisancelt"));
                Debug.Assert(pack.ar_i32.SequenceEqual(new int[] {-872446286, 1094942470}));
                Debug.Assert(pack.ar_u8.SequenceEqual(new byte[] {(byte)130, (byte)209}));
                Debug.Assert(pack.ar_i16.SequenceEqual(new short[] {(short) -436, (short) -8087}));
                Debug.Assert(pack.v3 == (uint)3351297837U);
                Debug.Assert(pack.ar_i8.SequenceEqual(new sbyte[] {(sbyte) - 61, (sbyte)58}));
                Debug.Assert(pack.ar_f.SequenceEqual(new float[] {1.963799E38F, -3.3542538E38F}));
                Debug.Assert(pack.v2 == (ushort)(ushort)60545);
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {1908301886U, 2861444833U}));
                Debug.Assert(pack.v1 == (byte)(byte)239);
                Debug.Assert(pack.ar_u16.SequenceEqual(new ushort[] {(ushort)13101, (ushort)5678}));
                Debug.Assert(pack.ar_d.SequenceEqual(new double[] {-1.6371881256415932E307, 1.029110340892298E307}));
            };
            GroundControl.ARRAY_TEST_6 p156 = CommunicationChannel.new_ARRAY_TEST_6();
            PH.setPack(p156);
            p156.ar_f_SET(new float[] {1.963799E38F, -3.3542538E38F}, 0) ;
            p156.v3 = (uint)3351297837U;
            p156.v2 = (ushort)(ushort)60545;
            p156.ar_i8_SET(new sbyte[] {(sbyte) - 61, (sbyte)58}, 0) ;
            p156.ar_u16_SET(new ushort[] {(ushort)13101, (ushort)5678}, 0) ;
            p156.ar_u8_SET(new byte[] {(byte)130, (byte)209}, 0) ;
            p156.v1 = (byte)(byte)239;
            p156.ar_c_SET("bfhvfvmkzbltisancelt", PH) ;
            p156.ar_u32_SET(new uint[] {1908301886U, 2861444833U}, 0) ;
            p156.ar_d_SET(new double[] {-1.6371881256415932E307, 1.029110340892298E307}, 0) ;
            p156.ar_i32_SET(new int[] {-872446286, 1094942470}, 0) ;
            p156.ar_i16_SET(new short[] {(short) -436, (short) -8087}, 0) ;
            CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_7Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_i8.SequenceEqual(new sbyte[] {(sbyte)53, (sbyte) - 109}));
                Debug.Assert(pack.ar_u8.SequenceEqual(new byte[] {(byte)144, (byte)104}));
                Debug.Assert(pack.ar_u16.SequenceEqual(new ushort[] {(ushort)19985, (ushort)27077}));
                Debug.Assert(pack.ar_i32.SequenceEqual(new int[] {-21691590, -924159525}));
                Debug.Assert(pack.ar_d.SequenceEqual(new double[] {3.504045529637445E307, -1.2591189865117702E308}));
                Debug.Assert(pack.ar_c_LEN(ph) == 19);
                Debug.Assert(pack.ar_c_TRY(ph).Equals("ovuqyIfVxyfmlAyxbnw"));
                Debug.Assert(pack.ar_i16.SequenceEqual(new short[] {(short) -10726, (short)14167}));
                Debug.Assert(pack.ar_f.SequenceEqual(new float[] {3.427733E37F, 2.0008408E38F}));
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {2446213615U, 4097869644U}));
            };
            GroundControl.ARRAY_TEST_7 p157 = CommunicationChannel.new_ARRAY_TEST_7();
            PH.setPack(p157);
            p157.ar_i8_SET(new sbyte[] {(sbyte)53, (sbyte) - 109}, 0) ;
            p157.ar_u8_SET(new byte[] {(byte)144, (byte)104}, 0) ;
            p157.ar_u16_SET(new ushort[] {(ushort)19985, (ushort)27077}, 0) ;
            p157.ar_f_SET(new float[] {3.427733E37F, 2.0008408E38F}, 0) ;
            p157.ar_u32_SET(new uint[] {2446213615U, 4097869644U}, 0) ;
            p157.ar_i32_SET(new int[] {-21691590, -924159525}, 0) ;
            p157.ar_d_SET(new double[] {3.504045529637445E307, -1.2591189865117702E308}, 0) ;
            p157.ar_i16_SET(new short[] {(short) -10726, (short)14167}, 0) ;
            p157.ar_c_SET("ovuqyIfVxyfmlAyxbnw", PH) ;
            CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_8Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_u16.SequenceEqual(new ushort[] {(ushort)35209, (ushort)33814}));
                Debug.Assert(pack.ar_d.SequenceEqual(new double[] {4.118636555643864E307, -2.7357791750154884E307}));
                Debug.Assert(pack.v3 == (uint)1085644350U);
            };
            GroundControl.ARRAY_TEST_8 p158 = CommunicationChannel.new_ARRAY_TEST_8();
            PH.setPack(p158);
            p158.ar_u16_SET(new ushort[] {(ushort)35209, (ushort)33814}, 0) ;
            p158.ar_d_SET(new double[] {4.118636555643864E307, -2.7357791750154884E307}, 0) ;
            p158.v3 = (uint)1085644350U;
            CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_vert_accuracy == (float) -7.390943E37F);
                Debug.Assert(pack.pos_vert_ratio == (float)7.5320934E37F);
                Debug.Assert(pack.hagl_ratio == (float)3.6342625E37F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS);
                Debug.Assert(pack.vel_ratio == (float)2.289666E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)2.272421E38F);
                Debug.Assert(pack.time_usec == (ulong)5627446513935229502L);
                Debug.Assert(pack.mag_ratio == (float) -2.122237E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float)1.9251674E38F);
                Debug.Assert(pack.tas_ratio == (float)2.7881324E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)5627446513935229502L;
            p230.mag_ratio = (float) -2.122237E38F;
            p230.pos_vert_ratio = (float)7.5320934E37F;
            p230.pos_vert_accuracy = (float) -7.390943E37F;
            p230.pos_horiz_accuracy = (float)2.272421E38F;
            p230.hagl_ratio = (float)3.6342625E37F;
            p230.pos_horiz_ratio = (float)1.9251674E38F;
            p230.vel_ratio = (float)2.289666E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS;
            p230.tas_ratio = (float)2.7881324E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vert_accuracy == (float) -1.4907648E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -1.4054759E38F);
                Debug.Assert(pack.wind_y == (float) -2.209535E38F);
                Debug.Assert(pack.wind_x == (float)1.296658E37F);
                Debug.Assert(pack.var_vert == (float) -1.3141646E38F);
                Debug.Assert(pack.var_horiz == (float) -1.9828185E38F);
                Debug.Assert(pack.time_usec == (ulong)7957607323465404111L);
                Debug.Assert(pack.wind_alt == (float)3.0912625E38F);
                Debug.Assert(pack.wind_z == (float)1.3572953E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.vert_accuracy = (float) -1.4907648E38F;
            p231.wind_x = (float)1.296658E37F;
            p231.wind_z = (float)1.3572953E38F;
            p231.horiz_accuracy = (float) -1.4054759E38F;
            p231.var_vert = (float) -1.3141646E38F;
            p231.wind_alt = (float)3.0912625E38F;
            p231.var_horiz = (float) -1.9828185E38F;
            p231.wind_y = (float) -2.209535E38F;
            p231.time_usec = (ulong)7957607323465404111L;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vert_accuracy == (float) -4.358616E37F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)170);
                Debug.Assert(pack.ve == (float) -2.0690709E38F);
                Debug.Assert(pack.time_week_ms == (uint)24801301U);
                Debug.Assert(pack.lon == (int) -777351690);
                Debug.Assert(pack.vd == (float)1.9266829E37F);
                Debug.Assert(pack.alt == (float) -1.1965869E38F);
                Debug.Assert(pack.lat == (int)1034409258);
                Debug.Assert(pack.vn == (float) -1.6542435E38F);
                Debug.Assert(pack.vdop == (float)3.3274583E38F);
                Debug.Assert(pack.time_usec == (ulong)997127794980739740L);
                Debug.Assert(pack.time_week == (ushort)(ushort)23869);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY);
                Debug.Assert(pack.fix_type == (byte)(byte)16);
                Debug.Assert(pack.gps_id == (byte)(byte)154);
                Debug.Assert(pack.horiz_accuracy == (float) -3.0370507E38F);
                Debug.Assert(pack.speed_accuracy == (float) -5.5280813E37F);
                Debug.Assert(pack.hdop == (float)5.5544383E37F);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.vert_accuracy = (float) -4.358616E37F;
            p232.vn = (float) -1.6542435E38F;
            p232.gps_id = (byte)(byte)154;
            p232.horiz_accuracy = (float) -3.0370507E38F;
            p232.fix_type = (byte)(byte)16;
            p232.lon = (int) -777351690;
            p232.satellites_visible = (byte)(byte)170;
            p232.time_week = (ushort)(ushort)23869;
            p232.alt = (float) -1.1965869E38F;
            p232.time_usec = (ulong)997127794980739740L;
            p232.lat = (int)1034409258;
            p232.vd = (float)1.9266829E37F;
            p232.vdop = (float)3.3274583E38F;
            p232.speed_accuracy = (float) -5.5280813E37F;
            p232.hdop = (float)5.5544383E37F;
            p232.ve = (float) -2.0690709E38F;
            p232.time_week_ms = (uint)24801301U;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (byte)(byte)196);
                Debug.Assert(pack.len == (byte)(byte)217);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)132, (byte)115, (byte)241, (byte)170, (byte)66, (byte)140, (byte)203, (byte)127, (byte)172, (byte)28, (byte)1, (byte)249, (byte)134, (byte)236, (byte)15, (byte)73, (byte)78, (byte)161, (byte)185, (byte)251, (byte)196, (byte)72, (byte)16, (byte)97, (byte)234, (byte)225, (byte)228, (byte)184, (byte)183, (byte)62, (byte)168, (byte)88, (byte)72, (byte)241, (byte)136, (byte)227, (byte)62, (byte)101, (byte)15, (byte)26, (byte)75, (byte)0, (byte)158, (byte)141, (byte)197, (byte)18, (byte)95, (byte)228, (byte)94, (byte)97, (byte)236, (byte)122, (byte)231, (byte)112, (byte)172, (byte)233, (byte)167, (byte)160, (byte)229, (byte)49, (byte)104, (byte)162, (byte)174, (byte)143, (byte)108, (byte)218, (byte)199, (byte)154, (byte)115, (byte)203, (byte)174, (byte)243, (byte)238, (byte)98, (byte)58, (byte)237, (byte)72, (byte)217, (byte)169, (byte)255, (byte)63, (byte)13, (byte)212, (byte)163, (byte)249, (byte)188, (byte)182, (byte)246, (byte)35, (byte)98, (byte)115, (byte)120, (byte)26, (byte)4, (byte)77, (byte)50, (byte)210, (byte)234, (byte)122, (byte)93, (byte)34, (byte)110, (byte)19, (byte)225, (byte)93, (byte)35, (byte)165, (byte)173, (byte)148, (byte)234, (byte)109, (byte)20, (byte)214, (byte)17, (byte)172, (byte)42, (byte)31, (byte)91, (byte)83, (byte)22, (byte)171, (byte)172, (byte)32, (byte)84, (byte)251, (byte)39, (byte)128, (byte)206, (byte)94, (byte)123, (byte)122, (byte)119, (byte)11, (byte)59, (byte)91, (byte)149, (byte)12, (byte)226, (byte)7, (byte)148, (byte)159, (byte)73, (byte)85, (byte)95, (byte)233, (byte)21, (byte)144, (byte)243, (byte)168, (byte)126, (byte)49, (byte)46, (byte)190, (byte)208, (byte)215, (byte)246, (byte)193, (byte)29, (byte)58, (byte)235, (byte)172, (byte)32, (byte)94, (byte)215, (byte)174, (byte)164, (byte)226, (byte)90, (byte)88, (byte)91, (byte)237, (byte)142, (byte)28, (byte)189, (byte)209, (byte)248, (byte)174, (byte)154, (byte)58, (byte)127}));
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)196;
            p233.data__SET(new byte[] {(byte)132, (byte)115, (byte)241, (byte)170, (byte)66, (byte)140, (byte)203, (byte)127, (byte)172, (byte)28, (byte)1, (byte)249, (byte)134, (byte)236, (byte)15, (byte)73, (byte)78, (byte)161, (byte)185, (byte)251, (byte)196, (byte)72, (byte)16, (byte)97, (byte)234, (byte)225, (byte)228, (byte)184, (byte)183, (byte)62, (byte)168, (byte)88, (byte)72, (byte)241, (byte)136, (byte)227, (byte)62, (byte)101, (byte)15, (byte)26, (byte)75, (byte)0, (byte)158, (byte)141, (byte)197, (byte)18, (byte)95, (byte)228, (byte)94, (byte)97, (byte)236, (byte)122, (byte)231, (byte)112, (byte)172, (byte)233, (byte)167, (byte)160, (byte)229, (byte)49, (byte)104, (byte)162, (byte)174, (byte)143, (byte)108, (byte)218, (byte)199, (byte)154, (byte)115, (byte)203, (byte)174, (byte)243, (byte)238, (byte)98, (byte)58, (byte)237, (byte)72, (byte)217, (byte)169, (byte)255, (byte)63, (byte)13, (byte)212, (byte)163, (byte)249, (byte)188, (byte)182, (byte)246, (byte)35, (byte)98, (byte)115, (byte)120, (byte)26, (byte)4, (byte)77, (byte)50, (byte)210, (byte)234, (byte)122, (byte)93, (byte)34, (byte)110, (byte)19, (byte)225, (byte)93, (byte)35, (byte)165, (byte)173, (byte)148, (byte)234, (byte)109, (byte)20, (byte)214, (byte)17, (byte)172, (byte)42, (byte)31, (byte)91, (byte)83, (byte)22, (byte)171, (byte)172, (byte)32, (byte)84, (byte)251, (byte)39, (byte)128, (byte)206, (byte)94, (byte)123, (byte)122, (byte)119, (byte)11, (byte)59, (byte)91, (byte)149, (byte)12, (byte)226, (byte)7, (byte)148, (byte)159, (byte)73, (byte)85, (byte)95, (byte)233, (byte)21, (byte)144, (byte)243, (byte)168, (byte)126, (byte)49, (byte)46, (byte)190, (byte)208, (byte)215, (byte)246, (byte)193, (byte)29, (byte)58, (byte)235, (byte)172, (byte)32, (byte)94, (byte)215, (byte)174, (byte)164, (byte)226, (byte)90, (byte)88, (byte)91, (byte)237, (byte)142, (byte)28, (byte)189, (byte)209, (byte)248, (byte)174, (byte)154, (byte)58, (byte)127}, 0) ;
            p233.len = (byte)(byte)217;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gps_nsat == (byte)(byte)103);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)58);
                Debug.Assert(pack.failsafe == (byte)(byte)150);
                Debug.Assert(pack.airspeed == (byte)(byte)113);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.latitude == (int) -1674777077);
                Debug.Assert(pack.longitude == (int)390697728);
                Debug.Assert(pack.groundspeed == (byte)(byte)113);
                Debug.Assert(pack.heading == (ushort)(ushort)56346);
                Debug.Assert(pack.custom_mode == (uint)4060162876U);
                Debug.Assert(pack.battery_remaining == (byte)(byte)35);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)37571);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)115);
                Debug.Assert(pack.altitude_sp == (short)(short) -3827);
                Debug.Assert(pack.roll == (short)(short)20228);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
                Debug.Assert(pack.altitude_amsl == (short)(short) -14731);
                Debug.Assert(pack.wp_num == (byte)(byte)210);
                Debug.Assert(pack.heading_sp == (short)(short)2976);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 87);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)116);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)102);
                Debug.Assert(pack.pitch == (short)(short)6773);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.heading_sp = (short)(short)2976;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED;
            p234.altitude_sp = (short)(short) -3827;
            p234.gps_nsat = (byte)(byte)103;
            p234.wp_num = (byte)(byte)210;
            p234.climb_rate = (sbyte)(sbyte) - 87;
            p234.pitch = (short)(short)6773;
            p234.temperature = (sbyte)(sbyte)115;
            p234.latitude = (int) -1674777077;
            p234.airspeed = (byte)(byte)113;
            p234.custom_mode = (uint)4060162876U;
            p234.roll = (short)(short)20228;
            p234.battery_remaining = (byte)(byte)35;
            p234.throttle = (sbyte)(sbyte)102;
            p234.airspeed_sp = (byte)(byte)116;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            p234.altitude_amsl = (short)(short) -14731;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p234.failsafe = (byte)(byte)150;
            p234.longitude = (int)390697728;
            p234.heading = (ushort)(ushort)56346;
            p234.temperature_air = (sbyte)(sbyte)58;
            p234.wp_distance = (ushort)(ushort)37571;
            p234.groundspeed = (byte)(byte)113;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_2 == (uint)2680014777U);
                Debug.Assert(pack.vibration_x == (float) -6.0927352E35F);
                Debug.Assert(pack.clipping_0 == (uint)1453413846U);
                Debug.Assert(pack.clipping_1 == (uint)2778313163U);
                Debug.Assert(pack.time_usec == (ulong)6035776258147180424L);
                Debug.Assert(pack.vibration_z == (float)2.8338398E37F);
                Debug.Assert(pack.vibration_y == (float) -9.128734E37F);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_1 = (uint)2778313163U;
            p241.time_usec = (ulong)6035776258147180424L;
            p241.clipping_2 = (uint)2680014777U;
            p241.vibration_y = (float) -9.128734E37F;
            p241.vibration_z = (float)2.8338398E37F;
            p241.clipping_0 = (uint)1453413846U;
            p241.vibration_x = (float) -6.0927352E35F;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.1407923E38F, -2.2110705E38F, 2.5464377E38F, 2.2361286E38F}));
                Debug.Assert(pack.latitude == (int) -1560182446);
                Debug.Assert(pack.approach_x == (float) -1.1568871E38F);
                Debug.Assert(pack.approach_z == (float)3.163E38F);
                Debug.Assert(pack.x == (float)3.3279256E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)5627236114901476443L);
                Debug.Assert(pack.z == (float)1.6302896E38F);
                Debug.Assert(pack.approach_y == (float) -1.2658371E38F);
                Debug.Assert(pack.longitude == (int) -93517240);
                Debug.Assert(pack.y == (float) -1.9584715E38F);
                Debug.Assert(pack.altitude == (int) -414245400);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int) -1560182446;
            p242.altitude = (int) -414245400;
            p242.z = (float)1.6302896E38F;
            p242.q_SET(new float[] {-1.1407923E38F, -2.2110705E38F, 2.5464377E38F, 2.2361286E38F}, 0) ;
            p242.approach_x = (float) -1.1568871E38F;
            p242.longitude = (int) -93517240;
            p242.approach_y = (float) -1.2658371E38F;
            p242.approach_z = (float)3.163E38F;
            p242.y = (float) -1.9584715E38F;
            p242.time_usec_SET((ulong)5627236114901476443L, PH) ;
            p242.x = (float)3.3279256E38F;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2230095248715559655L);
                Debug.Assert(pack.target_system == (byte)(byte)160);
                Debug.Assert(pack.approach_z == (float)3.1921618E38F);
                Debug.Assert(pack.longitude == (int)269805636);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.025278E38F, 2.456534E38F, -3.813616E37F, 2.624989E38F}));
                Debug.Assert(pack.latitude == (int)1904128932);
                Debug.Assert(pack.z == (float)2.2164698E37F);
                Debug.Assert(pack.x == (float)1.2711757E38F);
                Debug.Assert(pack.altitude == (int) -1159237781);
                Debug.Assert(pack.y == (float) -2.2704744E38F);
                Debug.Assert(pack.approach_y == (float) -2.967428E38F);
                Debug.Assert(pack.approach_x == (float)5.485779E37F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)160;
            p243.approach_y = (float) -2.967428E38F;
            p243.altitude = (int) -1159237781;
            p243.time_usec_SET((ulong)2230095248715559655L, PH) ;
            p243.y = (float) -2.2704744E38F;
            p243.q_SET(new float[] {-2.025278E38F, 2.456534E38F, -3.813616E37F, 2.624989E38F}, 0) ;
            p243.z = (float)2.2164698E37F;
            p243.approach_x = (float)5.485779E37F;
            p243.x = (float)1.2711757E38F;
            p243.approach_z = (float)3.1921618E38F;
            p243.latitude = (int)1904128932;
            p243.longitude = (int)269805636;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int)1394297153);
                Debug.Assert(pack.message_id == (ushort)(ushort)29979);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int)1394297153;
            p244.message_id = (ushort)(ushort)29979;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_FW;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)14200);
                Debug.Assert(pack.altitude == (int) -1944835273);
                Debug.Assert(pack.ICAO_address == (uint)1011960392U);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY);
                Debug.Assert(pack.squawk == (ushort)(ushort)39441);
                Debug.Assert(pack.ver_velocity == (short)(short) -24801);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.tslc == (byte)(byte)131);
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV);
                Debug.Assert(pack.heading == (ushort)(ushort)51498);
                Debug.Assert(pack.lon == (int) -1744462296);
                Debug.Assert(pack.lat == (int) -1145096663);
                Debug.Assert(pack.callsign_LEN(ph) == 7);
                Debug.Assert(pack.callsign_TRY(ph).Equals("Jibgjui"));
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.heading = (ushort)(ushort)51498;
            p246.tslc = (byte)(byte)131;
            p246.hor_velocity = (ushort)(ushort)14200;
            p246.altitude = (int) -1944835273;
            p246.ver_velocity = (short)(short) -24801;
            p246.callsign_SET("Jibgjui", PH) ;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY;
            p246.ICAO_address = (uint)1011960392U;
            p246.squawk = (ushort)(ushort)39441;
            p246.lon = (int) -1744462296;
            p246.lat = (int) -1145096663;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (uint)1555903260U);
                Debug.Assert(pack.horizontal_minimum_delta == (float) -3.1746958E38F);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.altitude_minimum_delta == (float) -4.2344943E37F);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
                Debug.Assert(pack.time_to_minimum_delta == (float) -1.1449214E38F);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.altitude_minimum_delta = (float) -4.2344943E37F;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW;
            p247.id = (uint)1555903260U;
            p247.horizontal_minimum_delta = (float) -3.1746958E38F;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.time_to_minimum_delta = (float) -1.1449214E38F;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)204, (byte)41, (byte)200, (byte)149, (byte)81, (byte)47, (byte)171, (byte)245, (byte)91, (byte)33, (byte)55, (byte)84, (byte)121, (byte)180, (byte)123, (byte)42, (byte)177, (byte)98, (byte)181, (byte)211, (byte)181, (byte)102, (byte)147, (byte)24, (byte)214, (byte)148, (byte)56, (byte)52, (byte)11, (byte)143, (byte)255, (byte)250, (byte)52, (byte)0, (byte)117, (byte)120, (byte)211, (byte)228, (byte)233, (byte)251, (byte)19, (byte)31, (byte)134, (byte)76, (byte)97, (byte)11, (byte)243, (byte)177, (byte)131, (byte)30, (byte)115, (byte)93, (byte)235, (byte)67, (byte)197, (byte)23, (byte)57, (byte)48, (byte)0, (byte)50, (byte)136, (byte)208, (byte)130, (byte)160, (byte)218, (byte)179, (byte)81, (byte)148, (byte)227, (byte)99, (byte)195, (byte)82, (byte)152, (byte)187, (byte)189, (byte)29, (byte)29, (byte)8, (byte)69, (byte)227, (byte)246, (byte)0, (byte)44, (byte)25, (byte)191, (byte)161, (byte)124, (byte)180, (byte)189, (byte)212, (byte)155, (byte)70, (byte)77, (byte)113, (byte)12, (byte)105, (byte)59, (byte)179, (byte)26, (byte)64, (byte)139, (byte)227, (byte)42, (byte)145, (byte)125, (byte)78, (byte)176, (byte)229, (byte)190, (byte)104, (byte)195, (byte)50, (byte)238, (byte)125, (byte)155, (byte)118, (byte)250, (byte)56, (byte)135, (byte)205, (byte)228, (byte)120, (byte)59, (byte)133, (byte)125, (byte)84, (byte)152, (byte)32, (byte)136, (byte)222, (byte)71, (byte)101, (byte)220, (byte)85, (byte)44, (byte)56, (byte)67, (byte)159, (byte)13, (byte)14, (byte)119, (byte)106, (byte)179, (byte)109, (byte)41, (byte)111, (byte)62, (byte)144, (byte)221, (byte)4, (byte)165, (byte)169, (byte)251, (byte)191, (byte)181, (byte)154, (byte)144, (byte)236, (byte)205, (byte)194, (byte)126, (byte)216, (byte)16, (byte)48, (byte)89, (byte)100, (byte)251, (byte)83, (byte)145, (byte)35, (byte)75, (byte)30, (byte)167, (byte)19, (byte)210, (byte)210, (byte)133, (byte)160, (byte)232, (byte)50, (byte)48, (byte)153, (byte)102, (byte)116, (byte)177, (byte)233, (byte)156, (byte)159, (byte)143, (byte)106, (byte)182, (byte)110, (byte)132, (byte)140, (byte)147, (byte)184, (byte)51, (byte)241, (byte)167, (byte)204, (byte)109, (byte)223, (byte)78, (byte)245, (byte)40, (byte)137, (byte)0, (byte)148, (byte)254, (byte)10, (byte)1, (byte)212, (byte)221, (byte)94, (byte)121, (byte)69, (byte)15, (byte)192, (byte)97, (byte)218, (byte)64, (byte)103, (byte)171, (byte)117, (byte)242, (byte)234, (byte)35, (byte)249, (byte)203, (byte)131, (byte)131, (byte)46, (byte)142, (byte)128, (byte)18, (byte)42, (byte)172, (byte)91, (byte)41, (byte)204, (byte)84, (byte)103, (byte)0, (byte)167, (byte)113, (byte)254, (byte)225, (byte)43, (byte)80}));
                Debug.Assert(pack.target_component == (byte)(byte)29);
                Debug.Assert(pack.target_network == (byte)(byte)10);
                Debug.Assert(pack.target_system == (byte)(byte)43);
                Debug.Assert(pack.message_type == (ushort)(ushort)174);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)10;
            p248.target_system = (byte)(byte)43;
            p248.message_type = (ushort)(ushort)174;
            p248.payload_SET(new byte[] {(byte)204, (byte)41, (byte)200, (byte)149, (byte)81, (byte)47, (byte)171, (byte)245, (byte)91, (byte)33, (byte)55, (byte)84, (byte)121, (byte)180, (byte)123, (byte)42, (byte)177, (byte)98, (byte)181, (byte)211, (byte)181, (byte)102, (byte)147, (byte)24, (byte)214, (byte)148, (byte)56, (byte)52, (byte)11, (byte)143, (byte)255, (byte)250, (byte)52, (byte)0, (byte)117, (byte)120, (byte)211, (byte)228, (byte)233, (byte)251, (byte)19, (byte)31, (byte)134, (byte)76, (byte)97, (byte)11, (byte)243, (byte)177, (byte)131, (byte)30, (byte)115, (byte)93, (byte)235, (byte)67, (byte)197, (byte)23, (byte)57, (byte)48, (byte)0, (byte)50, (byte)136, (byte)208, (byte)130, (byte)160, (byte)218, (byte)179, (byte)81, (byte)148, (byte)227, (byte)99, (byte)195, (byte)82, (byte)152, (byte)187, (byte)189, (byte)29, (byte)29, (byte)8, (byte)69, (byte)227, (byte)246, (byte)0, (byte)44, (byte)25, (byte)191, (byte)161, (byte)124, (byte)180, (byte)189, (byte)212, (byte)155, (byte)70, (byte)77, (byte)113, (byte)12, (byte)105, (byte)59, (byte)179, (byte)26, (byte)64, (byte)139, (byte)227, (byte)42, (byte)145, (byte)125, (byte)78, (byte)176, (byte)229, (byte)190, (byte)104, (byte)195, (byte)50, (byte)238, (byte)125, (byte)155, (byte)118, (byte)250, (byte)56, (byte)135, (byte)205, (byte)228, (byte)120, (byte)59, (byte)133, (byte)125, (byte)84, (byte)152, (byte)32, (byte)136, (byte)222, (byte)71, (byte)101, (byte)220, (byte)85, (byte)44, (byte)56, (byte)67, (byte)159, (byte)13, (byte)14, (byte)119, (byte)106, (byte)179, (byte)109, (byte)41, (byte)111, (byte)62, (byte)144, (byte)221, (byte)4, (byte)165, (byte)169, (byte)251, (byte)191, (byte)181, (byte)154, (byte)144, (byte)236, (byte)205, (byte)194, (byte)126, (byte)216, (byte)16, (byte)48, (byte)89, (byte)100, (byte)251, (byte)83, (byte)145, (byte)35, (byte)75, (byte)30, (byte)167, (byte)19, (byte)210, (byte)210, (byte)133, (byte)160, (byte)232, (byte)50, (byte)48, (byte)153, (byte)102, (byte)116, (byte)177, (byte)233, (byte)156, (byte)159, (byte)143, (byte)106, (byte)182, (byte)110, (byte)132, (byte)140, (byte)147, (byte)184, (byte)51, (byte)241, (byte)167, (byte)204, (byte)109, (byte)223, (byte)78, (byte)245, (byte)40, (byte)137, (byte)0, (byte)148, (byte)254, (byte)10, (byte)1, (byte)212, (byte)221, (byte)94, (byte)121, (byte)69, (byte)15, (byte)192, (byte)97, (byte)218, (byte)64, (byte)103, (byte)171, (byte)117, (byte)242, (byte)234, (byte)35, (byte)249, (byte)203, (byte)131, (byte)131, (byte)46, (byte)142, (byte)128, (byte)18, (byte)42, (byte)172, (byte)91, (byte)41, (byte)204, (byte)84, (byte)103, (byte)0, (byte)167, (byte)113, (byte)254, (byte)225, (byte)43, (byte)80}, 0) ;
            p248.target_component = (byte)(byte)29;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)70, (sbyte) - 22, (sbyte) - 46, (sbyte) - 48, (sbyte) - 78, (sbyte)41, (sbyte) - 39, (sbyte)113, (sbyte) - 106, (sbyte) - 93, (sbyte) - 42, (sbyte) - 38, (sbyte)46, (sbyte) - 113, (sbyte) - 21, (sbyte)64, (sbyte)111, (sbyte)112, (sbyte) - 30, (sbyte)81, (sbyte) - 93, (sbyte) - 69, (sbyte)45, (sbyte)94, (sbyte) - 58, (sbyte)92, (sbyte)4, (sbyte)31, (sbyte)78, (sbyte) - 1, (sbyte) - 51, (sbyte)95}));
                Debug.Assert(pack.type == (byte)(byte)162);
                Debug.Assert(pack.address == (ushort)(ushort)64979);
                Debug.Assert(pack.ver == (byte)(byte)18);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.ver = (byte)(byte)18;
            p249.type = (byte)(byte)162;
            p249.value_SET(new sbyte[] {(sbyte)70, (sbyte) - 22, (sbyte) - 46, (sbyte) - 48, (sbyte) - 78, (sbyte)41, (sbyte) - 39, (sbyte)113, (sbyte) - 106, (sbyte) - 93, (sbyte) - 42, (sbyte) - 38, (sbyte)46, (sbyte) - 113, (sbyte) - 21, (sbyte)64, (sbyte)111, (sbyte)112, (sbyte) - 30, (sbyte)81, (sbyte) - 93, (sbyte) - 69, (sbyte)45, (sbyte)94, (sbyte) - 58, (sbyte)92, (sbyte)4, (sbyte)31, (sbyte)78, (sbyte) - 1, (sbyte) - 51, (sbyte)95}, 0) ;
            p249.address = (ushort)(ushort)64979;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -1.1019008E38F);
                Debug.Assert(pack.y == (float)2.2609806E37F);
                Debug.Assert(pack.x == (float)3.1836927E38F);
                Debug.Assert(pack.time_usec == (ulong)1562652923857246154L);
                Debug.Assert(pack.name_LEN(ph) == 9);
                Debug.Assert(pack.name_TRY(ph).Equals("vovkEpvVx"));
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.z = (float) -1.1019008E38F;
            p250.y = (float)2.2609806E37F;
            p250.time_usec = (ulong)1562652923857246154L;
            p250.x = (float)3.1836927E38F;
            p250.name_SET("vovkEpvVx", PH) ;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float) -2.3545782E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3775166240U);
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("xgwjfjxeHQ"));
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.name_SET("xgwjfjxeHQ", PH) ;
            p251.value = (float) -2.3545782E38F;
            p251.time_boot_ms = (uint)3775166240U;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (int)2123015643);
                Debug.Assert(pack.name_LEN(ph) == 5);
                Debug.Assert(pack.name_TRY(ph).Equals("tyvca"));
                Debug.Assert(pack.time_boot_ms == (uint)2203359334U);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.value = (int)2123015643;
            p252.name_SET("tyvca", PH) ;
            p252.time_boot_ms = (uint)2203359334U;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 27);
                Debug.Assert(pack.text_TRY(ph).Equals("orobloyxedWcZrahmwmcnpzwhve"));
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_ERROR);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("orobloyxedWcZrahmwmcnpzwhve", PH) ;
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_ERROR;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float) -1.6697867E38F);
                Debug.Assert(pack.time_boot_ms == (uint)199846636U);
                Debug.Assert(pack.ind == (byte)(byte)255);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.ind = (byte)(byte)255;
            p254.value = (float) -1.6697867E38F;
            p254.time_boot_ms = (uint)199846636U;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)83);
                Debug.Assert(pack.initial_timestamp == (ulong)6600792301318780890L);
                Debug.Assert(pack.target_system == (byte)(byte)75);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)246, (byte)239, (byte)220, (byte)88, (byte)86, (byte)124, (byte)48, (byte)117, (byte)37, (byte)177, (byte)59, (byte)88, (byte)139, (byte)142, (byte)128, (byte)253, (byte)150, (byte)58, (byte)206, (byte)119, (byte)198, (byte)64, (byte)3, (byte)76, (byte)88, (byte)43, (byte)20, (byte)76, (byte)23, (byte)114, (byte)50, (byte)31}));
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.secret_key_SET(new byte[] {(byte)246, (byte)239, (byte)220, (byte)88, (byte)86, (byte)124, (byte)48, (byte)117, (byte)37, (byte)177, (byte)59, (byte)88, (byte)139, (byte)142, (byte)128, (byte)253, (byte)150, (byte)58, (byte)206, (byte)119, (byte)198, (byte)64, (byte)3, (byte)76, (byte)88, (byte)43, (byte)20, (byte)76, (byte)23, (byte)114, (byte)50, (byte)31}, 0) ;
            p256.target_component = (byte)(byte)83;
            p256.initial_timestamp = (ulong)6600792301318780890L;
            p256.target_system = (byte)(byte)75;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_change_ms == (uint)1395002543U);
                Debug.Assert(pack.state == (byte)(byte)76);
                Debug.Assert(pack.time_boot_ms == (uint)162458382U);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.last_change_ms = (uint)1395002543U;
            p257.time_boot_ms = (uint)162458382U;
            p257.state = (byte)(byte)76;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tune_LEN(ph) == 4);
                Debug.Assert(pack.tune_TRY(ph).Equals("ctQv"));
                Debug.Assert(pack.target_component == (byte)(byte)147);
                Debug.Assert(pack.target_system == (byte)(byte)201);
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)201;
            p258.tune_SET("ctQv", PH) ;
            p258.target_component = (byte)(byte)147;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)20523);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 118);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("cawarvsKgczqrfhlhabfJeslpaSKboohsfcsxifuwohbkpixaljefrymfwbqbbywixwqnwoIySsFzoafgxxrhzoilacixoMbWtDcdcxscxdCprwywgtncw"));
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)108, (byte)51, (byte)21, (byte)7, (byte)231, (byte)251, (byte)241, (byte)34, (byte)83, (byte)158, (byte)74, (byte)4, (byte)96, (byte)198, (byte)104, (byte)221, (byte)181, (byte)144, (byte)230, (byte)22, (byte)26, (byte)219, (byte)157, (byte)122, (byte)134, (byte)75, (byte)50, (byte)208, (byte)239, (byte)131, (byte)203, (byte)122}));
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)55079);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
                Debug.Assert(pack.lens_id == (byte)(byte)239);
                Debug.Assert(pack.time_boot_ms == (uint)2921816547U);
                Debug.Assert(pack.focal_length == (float) -3.3943349E38F);
                Debug.Assert(pack.firmware_version == (uint)2586484170U);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)33, (byte)69, (byte)17, (byte)216, (byte)223, (byte)250, (byte)120, (byte)121, (byte)141, (byte)89, (byte)85, (byte)42, (byte)24, (byte)21, (byte)70, (byte)64, (byte)106, (byte)253, (byte)195, (byte)163, (byte)36, (byte)91, (byte)143, (byte)69, (byte)56, (byte)27, (byte)149, (byte)157, (byte)75, (byte)28, (byte)251, (byte)114}));
                Debug.Assert(pack.sensor_size_h == (float) -2.8339889E38F);
                Debug.Assert(pack.sensor_size_v == (float) -1.4943777E38F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)4625);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.resolution_h = (ushort)(ushort)4625;
            p259.firmware_version = (uint)2586484170U;
            p259.resolution_v = (ushort)(ushort)20523;
            p259.model_name_SET(new byte[] {(byte)33, (byte)69, (byte)17, (byte)216, (byte)223, (byte)250, (byte)120, (byte)121, (byte)141, (byte)89, (byte)85, (byte)42, (byte)24, (byte)21, (byte)70, (byte)64, (byte)106, (byte)253, (byte)195, (byte)163, (byte)36, (byte)91, (byte)143, (byte)69, (byte)56, (byte)27, (byte)149, (byte)157, (byte)75, (byte)28, (byte)251, (byte)114}, 0) ;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE;
            p259.sensor_size_v = (float) -1.4943777E38F;
            p259.cam_definition_uri_SET("cawarvsKgczqrfhlhabfJeslpaSKboohsfcsxifuwohbkpixaljefrymfwbqbbywixwqnwoIySsFzoafgxxrhzoilacixoMbWtDcdcxscxdCprwywgtncw", PH) ;
            p259.focal_length = (float) -3.3943349E38F;
            p259.time_boot_ms = (uint)2921816547U;
            p259.lens_id = (byte)(byte)239;
            p259.sensor_size_h = (float) -2.8339889E38F;
            p259.vendor_name_SET(new byte[] {(byte)108, (byte)51, (byte)21, (byte)7, (byte)231, (byte)251, (byte)241, (byte)34, (byte)83, (byte)158, (byte)74, (byte)4, (byte)96, (byte)198, (byte)104, (byte)221, (byte)181, (byte)144, (byte)230, (byte)22, (byte)26, (byte)219, (byte)157, (byte)122, (byte)134, (byte)75, (byte)50, (byte)208, (byte)239, (byte)131, (byte)203, (byte)122}, 0) ;
            p259.cam_definition_version = (ushort)(ushort)55079;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE);
                Debug.Assert(pack.time_boot_ms == (uint)952461492U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE;
            p260.time_boot_ms = (uint)952461492U;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.status == (byte)(byte)120);
                Debug.Assert(pack.storage_count == (byte)(byte)214);
                Debug.Assert(pack.write_speed == (float)2.8234424E38F);
                Debug.Assert(pack.available_capacity == (float) -1.1395548E38F);
                Debug.Assert(pack.total_capacity == (float) -2.32091E38F);
                Debug.Assert(pack.read_speed == (float)2.1434994E38F);
                Debug.Assert(pack.time_boot_ms == (uint)328425622U);
                Debug.Assert(pack.storage_id == (byte)(byte)221);
                Debug.Assert(pack.used_capacity == (float) -2.9222393E38F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.total_capacity = (float) -2.32091E38F;
            p261.write_speed = (float)2.8234424E38F;
            p261.read_speed = (float)2.1434994E38F;
            p261.available_capacity = (float) -1.1395548E38F;
            p261.storage_id = (byte)(byte)221;
            p261.time_boot_ms = (uint)328425622U;
            p261.storage_count = (byte)(byte)214;
            p261.status = (byte)(byte)120;
            p261.used_capacity = (float) -2.9222393E38F;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.video_status == (byte)(byte)146);
                Debug.Assert(pack.image_status == (byte)(byte)139);
                Debug.Assert(pack.available_capacity == (float) -3.3176773E38F);
                Debug.Assert(pack.image_interval == (float)2.93204E38F);
                Debug.Assert(pack.time_boot_ms == (uint)240432574U);
                Debug.Assert(pack.recording_time_ms == (uint)1621592194U);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.recording_time_ms = (uint)1621592194U;
            p262.video_status = (byte)(byte)146;
            p262.time_boot_ms = (uint)240432574U;
            p262.image_interval = (float)2.93204E38F;
            p262.image_status = (byte)(byte)139;
            p262.available_capacity = (float) -3.3176773E38F;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2704722984U);
                Debug.Assert(pack.alt == (int)593548456);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.2632512E38F, 7.055799E37F, -8.3781406E37F, -3.1351985E38F}));
                Debug.Assert(pack.time_utc == (ulong)8378011988636936641L);
                Debug.Assert(pack.file_url_LEN(ph) == 60);
                Debug.Assert(pack.file_url_TRY(ph).Equals("zdxkoUlpmtqbbmheFeNnokysfmqcfoXepckkajmqnuXcdjnmYfddobWZxsaa"));
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 24);
                Debug.Assert(pack.lat == (int)944318972);
                Debug.Assert(pack.image_index == (int)2060793717);
                Debug.Assert(pack.camera_id == (byte)(byte)202);
                Debug.Assert(pack.lon == (int) -188604785);
                Debug.Assert(pack.relative_alt == (int) -763458125);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.file_url_SET("zdxkoUlpmtqbbmheFeNnokysfmqcfoXepckkajmqnuXcdjnmYfddobWZxsaa", PH) ;
            p263.lon = (int) -188604785;
            p263.time_utc = (ulong)8378011988636936641L;
            p263.image_index = (int)2060793717;
            p263.alt = (int)593548456;
            p263.time_boot_ms = (uint)2704722984U;
            p263.q_SET(new float[] {-2.2632512E38F, 7.055799E37F, -8.3781406E37F, -3.1351985E38F}, 0) ;
            p263.lat = (int)944318972;
            p263.camera_id = (byte)(byte)202;
            p263.capture_result = (sbyte)(sbyte) - 24;
            p263.relative_alt = (int) -763458125;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.takeoff_time_utc == (ulong)9087036116889148252L);
                Debug.Assert(pack.arming_time_utc == (ulong)3898437911883287827L);
                Debug.Assert(pack.flight_uuid == (ulong)5993852643510104630L);
                Debug.Assert(pack.time_boot_ms == (uint)244751504U);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)244751504U;
            p264.takeoff_time_utc = (ulong)9087036116889148252L;
            p264.flight_uuid = (ulong)5993852643510104630L;
            p264.arming_time_utc = (ulong)3898437911883287827L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)3.3390288E38F);
                Debug.Assert(pack.pitch == (float) -2.6209538E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2905836494U);
                Debug.Assert(pack.yaw == (float) -2.8533507E38F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.pitch = (float) -2.6209538E38F;
            p265.yaw = (float) -2.8533507E38F;
            p265.time_boot_ms = (uint)2905836494U;
            p265.roll = (float)3.3390288E38F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)7316);
                Debug.Assert(pack.length == (byte)(byte)8);
                Debug.Assert(pack.target_system == (byte)(byte)239);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)231, (byte)201, (byte)152, (byte)22, (byte)59, (byte)106, (byte)88, (byte)53, (byte)69, (byte)60, (byte)15, (byte)84, (byte)175, (byte)112, (byte)124, (byte)147, (byte)101, (byte)136, (byte)45, (byte)159, (byte)193, (byte)219, (byte)60, (byte)246, (byte)181, (byte)173, (byte)244, (byte)215, (byte)105, (byte)66, (byte)144, (byte)242, (byte)241, (byte)60, (byte)63, (byte)108, (byte)202, (byte)248, (byte)88, (byte)116, (byte)181, (byte)184, (byte)92, (byte)92, (byte)51, (byte)14, (byte)21, (byte)254, (byte)185, (byte)252, (byte)244, (byte)30, (byte)131, (byte)80, (byte)215, (byte)196, (byte)27, (byte)89, (byte)223, (byte)183, (byte)250, (byte)23, (byte)122, (byte)209, (byte)157, (byte)52, (byte)249, (byte)76, (byte)1, (byte)223, (byte)47, (byte)214, (byte)100, (byte)74, (byte)255, (byte)228, (byte)124, (byte)176, (byte)28, (byte)162, (byte)2, (byte)127, (byte)252, (byte)140, (byte)16, (byte)61, (byte)10, (byte)125, (byte)9, (byte)44, (byte)222, (byte)16, (byte)148, (byte)22, (byte)226, (byte)96, (byte)59, (byte)180, (byte)218, (byte)76, (byte)79, (byte)57, (byte)170, (byte)83, (byte)3, (byte)95, (byte)189, (byte)53, (byte)226, (byte)232, (byte)94, (byte)242, (byte)0, (byte)178, (byte)58, (byte)31, (byte)119, (byte)226, (byte)180, (byte)237, (byte)243, (byte)143, (byte)102, (byte)100, (byte)143, (byte)199, (byte)84, (byte)152, (byte)57, (byte)64, (byte)231, (byte)16, (byte)139, (byte)27, (byte)245, (byte)229, (byte)30, (byte)172, (byte)111, (byte)1, (byte)6, (byte)126, (byte)78, (byte)237, (byte)191, (byte)63, (byte)230, (byte)113, (byte)94, (byte)199, (byte)102, (byte)84, (byte)142, (byte)62, (byte)140, (byte)245, (byte)156, (byte)176, (byte)225, (byte)208, (byte)69, (byte)85, (byte)8, (byte)238, (byte)95, (byte)26, (byte)108, (byte)117, (byte)108, (byte)201, (byte)137, (byte)144, (byte)113, (byte)138, (byte)75, (byte)136, (byte)58, (byte)146, (byte)116, (byte)230, (byte)9, (byte)218, (byte)72, (byte)236, (byte)90, (byte)230, (byte)192, (byte)231, (byte)39, (byte)155, (byte)207, (byte)67, (byte)216, (byte)62, (byte)52, (byte)147, (byte)42, (byte)107, (byte)181, (byte)206, (byte)204, (byte)103, (byte)146, (byte)20, (byte)90, (byte)201, (byte)218, (byte)219, (byte)81, (byte)34, (byte)126, (byte)48, (byte)37, (byte)21, (byte)4, (byte)5, (byte)244, (byte)191, (byte)88, (byte)138, (byte)8, (byte)243, (byte)211, (byte)208, (byte)97, (byte)138, (byte)146, (byte)173, (byte)115, (byte)237, (byte)72, (byte)129, (byte)178, (byte)16, (byte)150, (byte)105, (byte)11, (byte)190, (byte)58, (byte)154, (byte)111, (byte)251, (byte)20, (byte)40, (byte)84, (byte)190, (byte)143, (byte)42, (byte)11}));
                Debug.Assert(pack.target_component == (byte)(byte)163);
                Debug.Assert(pack.first_message_offset == (byte)(byte)167);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.first_message_offset = (byte)(byte)167;
            p266.target_system = (byte)(byte)239;
            p266.target_component = (byte)(byte)163;
            p266.sequence = (ushort)(ushort)7316;
            p266.length = (byte)(byte)8;
            p266.data__SET(new byte[] {(byte)231, (byte)201, (byte)152, (byte)22, (byte)59, (byte)106, (byte)88, (byte)53, (byte)69, (byte)60, (byte)15, (byte)84, (byte)175, (byte)112, (byte)124, (byte)147, (byte)101, (byte)136, (byte)45, (byte)159, (byte)193, (byte)219, (byte)60, (byte)246, (byte)181, (byte)173, (byte)244, (byte)215, (byte)105, (byte)66, (byte)144, (byte)242, (byte)241, (byte)60, (byte)63, (byte)108, (byte)202, (byte)248, (byte)88, (byte)116, (byte)181, (byte)184, (byte)92, (byte)92, (byte)51, (byte)14, (byte)21, (byte)254, (byte)185, (byte)252, (byte)244, (byte)30, (byte)131, (byte)80, (byte)215, (byte)196, (byte)27, (byte)89, (byte)223, (byte)183, (byte)250, (byte)23, (byte)122, (byte)209, (byte)157, (byte)52, (byte)249, (byte)76, (byte)1, (byte)223, (byte)47, (byte)214, (byte)100, (byte)74, (byte)255, (byte)228, (byte)124, (byte)176, (byte)28, (byte)162, (byte)2, (byte)127, (byte)252, (byte)140, (byte)16, (byte)61, (byte)10, (byte)125, (byte)9, (byte)44, (byte)222, (byte)16, (byte)148, (byte)22, (byte)226, (byte)96, (byte)59, (byte)180, (byte)218, (byte)76, (byte)79, (byte)57, (byte)170, (byte)83, (byte)3, (byte)95, (byte)189, (byte)53, (byte)226, (byte)232, (byte)94, (byte)242, (byte)0, (byte)178, (byte)58, (byte)31, (byte)119, (byte)226, (byte)180, (byte)237, (byte)243, (byte)143, (byte)102, (byte)100, (byte)143, (byte)199, (byte)84, (byte)152, (byte)57, (byte)64, (byte)231, (byte)16, (byte)139, (byte)27, (byte)245, (byte)229, (byte)30, (byte)172, (byte)111, (byte)1, (byte)6, (byte)126, (byte)78, (byte)237, (byte)191, (byte)63, (byte)230, (byte)113, (byte)94, (byte)199, (byte)102, (byte)84, (byte)142, (byte)62, (byte)140, (byte)245, (byte)156, (byte)176, (byte)225, (byte)208, (byte)69, (byte)85, (byte)8, (byte)238, (byte)95, (byte)26, (byte)108, (byte)117, (byte)108, (byte)201, (byte)137, (byte)144, (byte)113, (byte)138, (byte)75, (byte)136, (byte)58, (byte)146, (byte)116, (byte)230, (byte)9, (byte)218, (byte)72, (byte)236, (byte)90, (byte)230, (byte)192, (byte)231, (byte)39, (byte)155, (byte)207, (byte)67, (byte)216, (byte)62, (byte)52, (byte)147, (byte)42, (byte)107, (byte)181, (byte)206, (byte)204, (byte)103, (byte)146, (byte)20, (byte)90, (byte)201, (byte)218, (byte)219, (byte)81, (byte)34, (byte)126, (byte)48, (byte)37, (byte)21, (byte)4, (byte)5, (byte)244, (byte)191, (byte)88, (byte)138, (byte)8, (byte)243, (byte)211, (byte)208, (byte)97, (byte)138, (byte)146, (byte)173, (byte)115, (byte)237, (byte)72, (byte)129, (byte)178, (byte)16, (byte)150, (byte)105, (byte)11, (byte)190, (byte)58, (byte)154, (byte)111, (byte)251, (byte)20, (byte)40, (byte)84, (byte)190, (byte)143, (byte)42, (byte)11}, 0) ;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)18704);
                Debug.Assert(pack.length == (byte)(byte)234);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)125, (byte)222, (byte)146, (byte)153, (byte)74, (byte)85, (byte)36, (byte)78, (byte)114, (byte)196, (byte)137, (byte)67, (byte)149, (byte)184, (byte)190, (byte)235, (byte)198, (byte)223, (byte)135, (byte)205, (byte)181, (byte)76, (byte)152, (byte)105, (byte)47, (byte)63, (byte)220, (byte)218, (byte)51, (byte)217, (byte)38, (byte)182, (byte)188, (byte)177, (byte)103, (byte)25, (byte)175, (byte)106, (byte)42, (byte)103, (byte)106, (byte)25, (byte)149, (byte)216, (byte)168, (byte)171, (byte)138, (byte)120, (byte)175, (byte)202, (byte)190, (byte)13, (byte)144, (byte)107, (byte)54, (byte)36, (byte)236, (byte)90, (byte)104, (byte)147, (byte)75, (byte)209, (byte)85, (byte)53, (byte)244, (byte)25, (byte)39, (byte)51, (byte)133, (byte)184, (byte)143, (byte)49, (byte)19, (byte)160, (byte)84, (byte)215, (byte)115, (byte)62, (byte)95, (byte)132, (byte)212, (byte)54, (byte)83, (byte)235, (byte)72, (byte)86, (byte)229, (byte)202, (byte)111, (byte)107, (byte)134, (byte)93, (byte)76, (byte)133, (byte)72, (byte)156, (byte)37, (byte)143, (byte)231, (byte)250, (byte)208, (byte)117, (byte)157, (byte)96, (byte)243, (byte)205, (byte)173, (byte)102, (byte)178, (byte)162, (byte)131, (byte)224, (byte)111, (byte)244, (byte)175, (byte)134, (byte)12, (byte)33, (byte)115, (byte)43, (byte)27, (byte)185, (byte)76, (byte)210, (byte)241, (byte)210, (byte)179, (byte)217, (byte)242, (byte)125, (byte)104, (byte)169, (byte)219, (byte)243, (byte)221, (byte)120, (byte)230, (byte)139, (byte)138, (byte)150, (byte)89, (byte)173, (byte)56, (byte)82, (byte)98, (byte)23, (byte)43, (byte)57, (byte)105, (byte)122, (byte)240, (byte)241, (byte)232, (byte)48, (byte)220, (byte)78, (byte)135, (byte)156, (byte)94, (byte)81, (byte)164, (byte)195, (byte)47, (byte)213, (byte)34, (byte)212, (byte)53, (byte)157, (byte)73, (byte)202, (byte)51, (byte)164, (byte)141, (byte)150, (byte)20, (byte)90, (byte)174, (byte)154, (byte)59, (byte)103, (byte)231, (byte)13, (byte)209, (byte)113, (byte)158, (byte)83, (byte)201, (byte)54, (byte)229, (byte)240, (byte)43, (byte)50, (byte)4, (byte)132, (byte)74, (byte)194, (byte)41, (byte)122, (byte)15, (byte)43, (byte)161, (byte)106, (byte)58, (byte)191, (byte)138, (byte)63, (byte)58, (byte)110, (byte)252, (byte)255, (byte)177, (byte)214, (byte)12, (byte)174, (byte)172, (byte)63, (byte)58, (byte)250, (byte)235, (byte)68, (byte)20, (byte)5, (byte)192, (byte)79, (byte)209, (byte)72, (byte)201, (byte)107, (byte)203, (byte)219, (byte)204, (byte)134, (byte)229, (byte)88, (byte)131, (byte)29, (byte)119, (byte)140, (byte)223, (byte)52, (byte)103, (byte)78, (byte)50, (byte)154, (byte)8, (byte)91, (byte)171, (byte)182, (byte)31}));
                Debug.Assert(pack.target_component == (byte)(byte)224);
                Debug.Assert(pack.first_message_offset == (byte)(byte)109);
                Debug.Assert(pack.target_system == (byte)(byte)165);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.data__SET(new byte[] {(byte)125, (byte)222, (byte)146, (byte)153, (byte)74, (byte)85, (byte)36, (byte)78, (byte)114, (byte)196, (byte)137, (byte)67, (byte)149, (byte)184, (byte)190, (byte)235, (byte)198, (byte)223, (byte)135, (byte)205, (byte)181, (byte)76, (byte)152, (byte)105, (byte)47, (byte)63, (byte)220, (byte)218, (byte)51, (byte)217, (byte)38, (byte)182, (byte)188, (byte)177, (byte)103, (byte)25, (byte)175, (byte)106, (byte)42, (byte)103, (byte)106, (byte)25, (byte)149, (byte)216, (byte)168, (byte)171, (byte)138, (byte)120, (byte)175, (byte)202, (byte)190, (byte)13, (byte)144, (byte)107, (byte)54, (byte)36, (byte)236, (byte)90, (byte)104, (byte)147, (byte)75, (byte)209, (byte)85, (byte)53, (byte)244, (byte)25, (byte)39, (byte)51, (byte)133, (byte)184, (byte)143, (byte)49, (byte)19, (byte)160, (byte)84, (byte)215, (byte)115, (byte)62, (byte)95, (byte)132, (byte)212, (byte)54, (byte)83, (byte)235, (byte)72, (byte)86, (byte)229, (byte)202, (byte)111, (byte)107, (byte)134, (byte)93, (byte)76, (byte)133, (byte)72, (byte)156, (byte)37, (byte)143, (byte)231, (byte)250, (byte)208, (byte)117, (byte)157, (byte)96, (byte)243, (byte)205, (byte)173, (byte)102, (byte)178, (byte)162, (byte)131, (byte)224, (byte)111, (byte)244, (byte)175, (byte)134, (byte)12, (byte)33, (byte)115, (byte)43, (byte)27, (byte)185, (byte)76, (byte)210, (byte)241, (byte)210, (byte)179, (byte)217, (byte)242, (byte)125, (byte)104, (byte)169, (byte)219, (byte)243, (byte)221, (byte)120, (byte)230, (byte)139, (byte)138, (byte)150, (byte)89, (byte)173, (byte)56, (byte)82, (byte)98, (byte)23, (byte)43, (byte)57, (byte)105, (byte)122, (byte)240, (byte)241, (byte)232, (byte)48, (byte)220, (byte)78, (byte)135, (byte)156, (byte)94, (byte)81, (byte)164, (byte)195, (byte)47, (byte)213, (byte)34, (byte)212, (byte)53, (byte)157, (byte)73, (byte)202, (byte)51, (byte)164, (byte)141, (byte)150, (byte)20, (byte)90, (byte)174, (byte)154, (byte)59, (byte)103, (byte)231, (byte)13, (byte)209, (byte)113, (byte)158, (byte)83, (byte)201, (byte)54, (byte)229, (byte)240, (byte)43, (byte)50, (byte)4, (byte)132, (byte)74, (byte)194, (byte)41, (byte)122, (byte)15, (byte)43, (byte)161, (byte)106, (byte)58, (byte)191, (byte)138, (byte)63, (byte)58, (byte)110, (byte)252, (byte)255, (byte)177, (byte)214, (byte)12, (byte)174, (byte)172, (byte)63, (byte)58, (byte)250, (byte)235, (byte)68, (byte)20, (byte)5, (byte)192, (byte)79, (byte)209, (byte)72, (byte)201, (byte)107, (byte)203, (byte)219, (byte)204, (byte)134, (byte)229, (byte)88, (byte)131, (byte)29, (byte)119, (byte)140, (byte)223, (byte)52, (byte)103, (byte)78, (byte)50, (byte)154, (byte)8, (byte)91, (byte)171, (byte)182, (byte)31}, 0) ;
            p267.target_component = (byte)(byte)224;
            p267.target_system = (byte)(byte)165;
            p267.length = (byte)(byte)234;
            p267.sequence = (ushort)(ushort)18704;
            p267.first_message_offset = (byte)(byte)109;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)18668);
                Debug.Assert(pack.target_component == (byte)(byte)34);
                Debug.Assert(pack.target_system == (byte)(byte)21);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.sequence = (ushort)(ushort)18668;
            p268.target_system = (byte)(byte)21;
            p268.target_component = (byte)(byte)34;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.framerate == (float)1.4240716E38F);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)17511);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)64894);
                Debug.Assert(pack.rotation == (ushort)(ushort)34865);
                Debug.Assert(pack.bitrate == (uint)3296184931U);
                Debug.Assert(pack.status == (byte)(byte)125);
                Debug.Assert(pack.camera_id == (byte)(byte)17);
                Debug.Assert(pack.uri_LEN(ph) == 157);
                Debug.Assert(pack.uri_TRY(ph).Equals("uZltcmizqhuptsgmkifuttffovsypiuqfcfgtpvyAwvseTtknHtetdbsedfuyMfxwFyyrmbscHejueokcnbnrbkrhhvusbizctauvsMtcddumsokbvRvaxqtmKbnjiDgqctqzuqbwfnkrtrlccbjhlvutfcze"));
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.resolution_h = (ushort)(ushort)64894;
            p269.rotation = (ushort)(ushort)34865;
            p269.uri_SET("uZltcmizqhuptsgmkifuttffovsypiuqfcfgtpvyAwvseTtknHtetdbsedfuyMfxwFyyrmbscHejueokcnbnrbkrhhvusbizctauvsMtcddumsokbvRvaxqtmKbnjiDgqctqzuqbwfnkrtrlccbjhlvutfcze", PH) ;
            p269.framerate = (float)1.4240716E38F;
            p269.bitrate = (uint)3296184931U;
            p269.camera_id = (byte)(byte)17;
            p269.status = (byte)(byte)125;
            p269.resolution_v = (ushort)(ushort)17511;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_LEN(ph) == 78);
                Debug.Assert(pack.uri_TRY(ph).Equals("mjzrvhqmwqmhcqyghkyYrkcwyehsxawepkwleyiiagsgQbuumXklLaalmjVggqyhqoeRwzuplublxv"));
                Debug.Assert(pack.target_system == (byte)(byte)224);
                Debug.Assert(pack.rotation == (ushort)(ushort)1410);
                Debug.Assert(pack.framerate == (float) -7.123795E37F);
                Debug.Assert(pack.target_component == (byte)(byte)253);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)31276);
                Debug.Assert(pack.camera_id == (byte)(byte)10);
                Debug.Assert(pack.bitrate == (uint)19859708U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)18791);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.rotation = (ushort)(ushort)1410;
            p270.resolution_v = (ushort)(ushort)18791;
            p270.target_component = (byte)(byte)253;
            p270.target_system = (byte)(byte)224;
            p270.uri_SET("mjzrvhqmwqmhcqyghkyYrkcwyehsxawepkwleyiiagsgQbuumXklLaalmjVggqyhqoeRwzuplublxv", PH) ;
            p270.resolution_h = (ushort)(ushort)31276;
            p270.framerate = (float) -7.123795E37F;
            p270.bitrate = (uint)19859708U;
            p270.camera_id = (byte)(byte)10;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 35);
                Debug.Assert(pack.password_TRY(ph).Equals("gZYyhmwcatrmixsavudtxsvqzdlmNOalljV"));
                Debug.Assert(pack.ssid_LEN(ph) == 1);
                Debug.Assert(pack.ssid_TRY(ph).Equals("h"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("h", PH) ;
            p299.password_SET("gZYyhmwcatrmixsavudtxsvqzdlmNOalljV", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.max_version == (ushort)(ushort)53005);
                Debug.Assert(pack.version == (ushort)(ushort)12225);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)69, (byte)8, (byte)210, (byte)162, (byte)47, (byte)185, (byte)192, (byte)42}));
                Debug.Assert(pack.min_version == (ushort)(ushort)57310);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)55, (byte)251, (byte)85, (byte)198, (byte)24, (byte)155, (byte)122, (byte)70}));
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)12225;
            p300.spec_version_hash_SET(new byte[] {(byte)69, (byte)8, (byte)210, (byte)162, (byte)47, (byte)185, (byte)192, (byte)42}, 0) ;
            p300.min_version = (ushort)(ushort)57310;
            p300.max_version = (ushort)(ushort)53005;
            p300.library_version_hash_SET(new byte[] {(byte)55, (byte)251, (byte)85, (byte)198, (byte)24, (byte)155, (byte)122, (byte)70}, 0) ;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
                Debug.Assert(pack.uptime_sec == (uint)3751098083U);
                Debug.Assert(pack.time_usec == (ulong)716379740645477364L);
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)7403);
                Debug.Assert(pack.sub_mode == (byte)(byte)98);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.uptime_sec = (uint)3751098083U;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.sub_mode = (byte)(byte)98;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE;
            p310.vendor_specific_status_code = (ushort)(ushort)7403;
            p310.time_usec = (ulong)716379740645477364L;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_vcs_commit == (uint)1742281743U);
                Debug.Assert(pack.name_LEN(ph) == 64);
                Debug.Assert(pack.name_TRY(ph).Equals("ffRtiwRvgccaotbVfxytofzcvzNtnEiahxWtgutkBwvaStJuexuutpuhhjljyvzc"));
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)240, (byte)251, (byte)20, (byte)211, (byte)75, (byte)61, (byte)105, (byte)16, (byte)153, (byte)52, (byte)142, (byte)35, (byte)213, (byte)17, (byte)85, (byte)180}));
                Debug.Assert(pack.hw_version_major == (byte)(byte)94);
                Debug.Assert(pack.uptime_sec == (uint)3887784713U);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)203);
                Debug.Assert(pack.sw_version_major == (byte)(byte)104);
                Debug.Assert(pack.time_usec == (ulong)7706769311805670266L);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)5);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_unique_id_SET(new byte[] {(byte)240, (byte)251, (byte)20, (byte)211, (byte)75, (byte)61, (byte)105, (byte)16, (byte)153, (byte)52, (byte)142, (byte)35, (byte)213, (byte)17, (byte)85, (byte)180}, 0) ;
            p311.sw_version_major = (byte)(byte)104;
            p311.hw_version_minor = (byte)(byte)5;
            p311.sw_version_minor = (byte)(byte)203;
            p311.hw_version_major = (byte)(byte)94;
            p311.time_usec = (ulong)7706769311805670266L;
            p311.name_SET("ffRtiwRvgccaotbVfxytofzcvzNtnEiahxWtgutkBwvaStJuexuutpuhhjljyvzc", PH) ;
            p311.uptime_sec = (uint)3887784713U;
            p311.sw_vcs_commit = (uint)1742281743U;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)132);
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("zl"));
                Debug.Assert(pack.target_component == (byte)(byte)64);
                Debug.Assert(pack.param_index == (short)(short)21246);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)132;
            p320.target_component = (byte)(byte)64;
            p320.param_id_SET("zl", PH) ;
            p320.param_index = (short)(short)21246;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)131);
                Debug.Assert(pack.target_system == (byte)(byte)160);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)160;
            p321.target_component = (byte)(byte)131;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (ushort)(ushort)13027);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
                Debug.Assert(pack.param_count == (ushort)(ushort)59755);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ekbjpcvjiUx"));
                Debug.Assert(pack.param_value_LEN(ph) == 106);
                Debug.Assert(pack.param_value_TRY(ph).Equals("egnxiauikpgdkdloLvfuswqstFkogkbZvcmpvvoqxuigsmkvejgHrkauztdrukjmEtWAsbszvTypgloiAdijrrdxikdltpziptclOragjO"));
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_count = (ushort)(ushort)59755;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8;
            p322.param_value_SET("egnxiauikpgdkdloLvfuswqstFkogkbZvcmpvvoqxuigsmkvejgHrkauztdrukjmEtWAsbszvTypgloiAdijrrdxikdltpziptclOragjO", PH) ;
            p322.param_id_SET("ekbjpcvjiUx", PH) ;
            p322.param_index = (ushort)(ushort)13027;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
                Debug.Assert(pack.target_component == (byte)(byte)87);
                Debug.Assert(pack.param_value_LEN(ph) == 112);
                Debug.Assert(pack.param_value_TRY(ph).Equals("mvaklceohnTnfhqpskyxssdqsgoPlwnnvabxrZxspwbfOapwjlqumaftkehlgcSboikyGqtwiktcfSvvlsswxzatlyrofbiAtgcgfpovuwvrkviw"));
                Debug.Assert(pack.target_system == (byte)(byte)182);
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("LkedbjetlipqiJo"));
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_component = (byte)(byte)87;
            p323.target_system = (byte)(byte)182;
            p323.param_value_SET("mvaklceohnTnfhqpskyxssdqsgoPlwnnvabxrZxspwbfOapwjlqumaftkehlgcSboikyGqtwiktcfSvvlsswxzatlyrofbiAtgcgfpovuwvrkviw", PH) ;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            p323.param_id_SET("LkedbjetlipqiJo", PH) ;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS);
                Debug.Assert(pack.param_value_LEN(ph) == 126);
                Debug.Assert(pack.param_value_TRY(ph).Equals("DbpHfgpbjokPkhioLmepmhdwvachxmorolmzzdjtilwhgxfncuixrknfzxOWEsjsrtblrgKjpxxsLgrpnkKsneDxpywsyxeJcVxejjplrcjivqocntnigzunfpOxns"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
                Debug.Assert(pack.param_id_LEN(ph) == 13);
                Debug.Assert(pack.param_id_TRY(ph).Equals("zaklqiqbqcepc"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32;
            p324.param_id_SET("zaklqiqbqcepc", PH) ;
            p324.param_value_SET("DbpHfgpbjokPkhioLmepmhdwvachxmorolmzzdjtilwhgxfncuixrknfzxOWEsjsrtblrgKjpxxsLgrpnkKsneDxpywsyxeJcVxejjplrcjivqocntnigzunfpOxns", PH) ;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3777544254731582156L);
                Debug.Assert(pack.increment == (byte)(byte)190);
                Debug.Assert(pack.min_distance == (ushort)(ushort)10492);
                Debug.Assert(pack.max_distance == (ushort)(ushort)63362);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)34580, (ushort)19274, (ushort)53345, (ushort)9313, (ushort)36919, (ushort)40410, (ushort)10957, (ushort)16990, (ushort)26639, (ushort)31900, (ushort)18037, (ushort)29884, (ushort)61472, (ushort)51071, (ushort)14083, (ushort)24880, (ushort)6367, (ushort)55141, (ushort)56810, (ushort)27775, (ushort)39657, (ushort)40133, (ushort)19473, (ushort)25353, (ushort)11020, (ushort)58839, (ushort)40667, (ushort)39685, (ushort)47136, (ushort)21878, (ushort)48984, (ushort)39009, (ushort)46123, (ushort)59710, (ushort)3843, (ushort)7092, (ushort)21946, (ushort)46937, (ushort)44692, (ushort)42771, (ushort)48896, (ushort)63055, (ushort)65045, (ushort)45441, (ushort)59046, (ushort)54767, (ushort)18280, (ushort)19904, (ushort)6947, (ushort)25718, (ushort)44772, (ushort)52059, (ushort)46667, (ushort)37134, (ushort)39146, (ushort)19779, (ushort)27554, (ushort)12514, (ushort)7703, (ushort)59283, (ushort)12852, (ushort)30081, (ushort)34752, (ushort)15108, (ushort)61849, (ushort)22975, (ushort)13991, (ushort)18480, (ushort)56363, (ushort)47142, (ushort)20542, (ushort)29382}));
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)3777544254731582156L;
            p330.max_distance = (ushort)(ushort)63362;
            p330.increment = (byte)(byte)190;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p330.min_distance = (ushort)(ushort)10492;
            p330.distances_SET(new ushort[] {(ushort)34580, (ushort)19274, (ushort)53345, (ushort)9313, (ushort)36919, (ushort)40410, (ushort)10957, (ushort)16990, (ushort)26639, (ushort)31900, (ushort)18037, (ushort)29884, (ushort)61472, (ushort)51071, (ushort)14083, (ushort)24880, (ushort)6367, (ushort)55141, (ushort)56810, (ushort)27775, (ushort)39657, (ushort)40133, (ushort)19473, (ushort)25353, (ushort)11020, (ushort)58839, (ushort)40667, (ushort)39685, (ushort)47136, (ushort)21878, (ushort)48984, (ushort)39009, (ushort)46123, (ushort)59710, (ushort)3843, (ushort)7092, (ushort)21946, (ushort)46937, (ushort)44692, (ushort)42771, (ushort)48896, (ushort)63055, (ushort)65045, (ushort)45441, (ushort)59046, (ushort)54767, (ushort)18280, (ushort)19904, (ushort)6947, (ushort)25718, (ushort)44772, (ushort)52059, (ushort)46667, (ushort)37134, (ushort)39146, (ushort)19779, (ushort)27554, (ushort)12514, (ushort)7703, (ushort)59283, (ushort)12852, (ushort)30081, (ushort)34752, (ushort)15108, (ushort)61849, (ushort)22975, (ushort)13991, (ushort)18480, (ushort)56363, (ushort)47142, (ushort)20542, (ushort)29382}, 0) ;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
        }
    }
}
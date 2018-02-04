
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
        new class ATT_POS_MOCAP : GroundControl.ATT_POS_MOCAP
        {
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float[] q //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                get {return q_GET(new float[4], 0);}
            }
            public float[]q_GET(float[] dst_ch, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                for(int BYTE = 8, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float x //X position in meters (NED)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float y //Y position in meters (NED)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float z //Z position in meters (NED)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
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

            public void OnATT_POS_MOCAPReceive_direct(Channel src, Inside ph, ATT_POS_MOCAP pack) {OnATT_POS_MOCAPReceive(this, ph,  pack);}
            public event ATT_POS_MOCAPReceiveHandler OnATT_POS_MOCAPReceive;
            public delegate void ATT_POS_MOCAPReceiveHandler(Channel src, Inside ph, ATT_POS_MOCAP pack);
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
                    case 138:
                        if(pack == null) return new ATT_POS_MOCAP();
                        OnATT_POS_MOCAPReceive(this, ph, (ATT_POS_MOCAP) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_MISSION_FULL);
                Debug.Assert(pack.mavlink_version == (byte)(byte)96);
                Debug.Assert(pack.custom_mode == (uint)1411729035U);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_CALIBRATING);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_VTOL_RESERVED2);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_CALIBRATING;
            p0.mavlink_version = (byte)(byte)96;
            p0.custom_mode = (uint)1411729035U;
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_VTOL_RESERVED2;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_MISSION_FULL;
            ADV_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)60187);
                Debug.Assert(pack.current_battery == (short)(short)26238);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)43844);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)31996);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)41451);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)22367);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)36256);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)36802);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)125);
                Debug.Assert(pack.load == (ushort)(ushort)38051);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
            p1.errors_count1 = (ushort)(ushort)36256;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE;
            p1.drop_rate_comm = (ushort)(ushort)36802;
            p1.errors_count2 = (ushort)(ushort)31996;
            p1.voltage_battery = (ushort)(ushort)41451;
            p1.errors_count3 = (ushort)(ushort)43844;
            p1.errors_comm = (ushort)(ushort)22367;
            p1.current_battery = (short)(short)26238;
            p1.load = (ushort)(ushort)38051;
            p1.battery_remaining = (sbyte)(sbyte)125;
            p1.errors_count4 = (ushort)(ushort)60187;
            ADV_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)1956992383840547691L);
                Debug.Assert(pack.time_boot_ms == (uint)45844501U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)1956992383840547691L;
            p2.time_boot_ms = (uint)45844501U;
            ADV_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -1.908199E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.yaw == (float)2.1110751E38F);
                Debug.Assert(pack.afx == (float)2.7659941E38F);
                Debug.Assert(pack.vx == (float) -3.252622E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)62624);
                Debug.Assert(pack.vy == (float)1.9883657E36F);
                Debug.Assert(pack.time_boot_ms == (uint)1437509979U);
                Debug.Assert(pack.x == (float) -1.2893719E38F);
                Debug.Assert(pack.vz == (float) -2.8655128E38F);
                Debug.Assert(pack.y == (float)1.708561E38F);
                Debug.Assert(pack.yaw_rate == (float)1.5607491E38F);
                Debug.Assert(pack.afy == (float)1.4302215E38F);
                Debug.Assert(pack.afz == (float) -2.507065E38F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.z = (float) -1.908199E38F;
            p3.yaw_rate = (float)1.5607491E38F;
            p3.x = (float) -1.2893719E38F;
            p3.vz = (float) -2.8655128E38F;
            p3.vx = (float) -3.252622E38F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p3.y = (float)1.708561E38F;
            p3.vy = (float)1.9883657E36F;
            p3.afx = (float)2.7659941E38F;
            p3.time_boot_ms = (uint)1437509979U;
            p3.type_mask = (ushort)(ushort)62624;
            p3.afy = (float)1.4302215E38F;
            p3.afz = (float) -2.507065E38F;
            p3.yaw = (float)2.1110751E38F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)121);
                Debug.Assert(pack.seq == (uint)2428941260U);
                Debug.Assert(pack.time_usec == (ulong)5815009036843491979L);
                Debug.Assert(pack.target_component == (byte)(byte)181);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.target_system = (byte)(byte)121;
            p4.time_usec = (ulong)5815009036843491979L;
            p4.target_component = (byte)(byte)181;
            p4.seq = (uint)2428941260U;
            ADV_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.passkey_LEN(ph) == 21);
                Debug.Assert(pack.passkey_TRY(ph).Equals("ebuteucsmbiTbwbjddJqf"));
                Debug.Assert(pack.version == (byte)(byte)89);
                Debug.Assert(pack.control_request == (byte)(byte)175);
                Debug.Assert(pack.target_system == (byte)(byte)249);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.version = (byte)(byte)89;
            p5.target_system = (byte)(byte)249;
            p5.control_request = (byte)(byte)175;
            p5.passkey_SET("ebuteucsmbiTbwbjddJqf", PH) ;
            ADV_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ack == (byte)(byte)171);
                Debug.Assert(pack.control_request == (byte)(byte)240);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)136);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)136;
            p6.ack = (byte)(byte)171;
            p6.control_request = (byte)(byte)240;
            ADV_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 2);
                Debug.Assert(pack.key_TRY(ph).Equals("pl"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("pl", PH) ;
            ADV_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)198);
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_PREFLIGHT);
                Debug.Assert(pack.custom_mode == (uint)263714717U);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)198;
            p11.custom_mode = (uint)263714717U;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_PREFLIGHT;
            ADV_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short)9132);
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Uk"));
                Debug.Assert(pack.target_system == (byte)(byte)150);
                Debug.Assert(pack.target_component == (byte)(byte)33);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_index = (short)(short)9132;
            p20.param_id_SET("Uk", PH) ;
            p20.target_system = (byte)(byte)150;
            p20.target_component = (byte)(byte)33;
            ADV_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)99);
                Debug.Assert(pack.target_system == (byte)(byte)8);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)99;
            p21.target_system = (byte)(byte)8;
            ADV_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32);
                Debug.Assert(pack.param_value == (float)3.3007537E38F);
                Debug.Assert(pack.param_count == (ushort)(ushort)27426);
                Debug.Assert(pack.param_index == (ushort)(ushort)54328);
                Debug.Assert(pack.param_id_LEN(ph) == 14);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ysbkakqhluudcv"));
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_index = (ushort)(ushort)54328;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32;
            p22.param_id_SET("ysbkakqhluudcv", PH) ;
            p22.param_value = (float)3.3007537E38F;
            p22.param_count = (ushort)(ushort)27426;
            ADV_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)66);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("fvbnilfp"));
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
                Debug.Assert(pack.param_value == (float)1.1272357E38F);
                Debug.Assert(pack.target_component == (byte)(byte)43);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_id_SET("fvbnilfp", PH) ;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64;
            p23.target_component = (byte)(byte)43;
            p23.param_value = (float)1.1272357E38F;
            p23.target_system = (byte)(byte)66;
            ADV_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)1496516950U);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)3234173984U);
                Debug.Assert(pack.lat == (int)1482445225);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)2464607561U);
                Debug.Assert(pack.vel == (ushort)(ushort)15936);
                Debug.Assert(pack.time_usec == (ulong)9126517564282504155L);
                Debug.Assert(pack.satellites_visible == (byte)(byte)119);
                Debug.Assert(pack.lon == (int) -1346512843);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1423344835U);
                Debug.Assert(pack.cog == (ushort)(ushort)64290);
                Debug.Assert(pack.eph == (ushort)(ushort)24360);
                Debug.Assert(pack.alt == (int) -231460883);
                Debug.Assert(pack.epv == (ushort)(ushort)46440);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -280710328);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.vel_acc_SET((uint)2464607561U, PH) ;
            p24.v_acc_SET((uint)1423344835U, PH) ;
            p24.cog = (ushort)(ushort)64290;
            p24.time_usec = (ulong)9126517564282504155L;
            p24.satellites_visible = (byte)(byte)119;
            p24.hdg_acc_SET((uint)3234173984U, PH) ;
            p24.alt_ellipsoid_SET((int) -280710328, PH) ;
            p24.alt = (int) -231460883;
            p24.lat = (int)1482445225;
            p24.h_acc_SET((uint)1496516950U, PH) ;
            p24.lon = (int) -1346512843;
            p24.eph = (ushort)(ushort)24360;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS;
            p24.vel = (ushort)(ushort)15936;
            p24.epv = (ushort)(ushort)46440;
            ADV_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)249, (byte)234, (byte)109, (byte)183, (byte)192, (byte)131, (byte)35, (byte)197, (byte)165, (byte)100, (byte)224, (byte)55, (byte)90, (byte)61, (byte)185, (byte)161, (byte)172, (byte)87, (byte)114, (byte)132}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)153);
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)250, (byte)56, (byte)124, (byte)177, (byte)88, (byte)19, (byte)161, (byte)27, (byte)35, (byte)186, (byte)72, (byte)189, (byte)138, (byte)118, (byte)192, (byte)238, (byte)29, (byte)38, (byte)215, (byte)196}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)115, (byte)81, (byte)187, (byte)211, (byte)150, (byte)24, (byte)40, (byte)182, (byte)120, (byte)100, (byte)214, (byte)169, (byte)53, (byte)251, (byte)48, (byte)200, (byte)153, (byte)188, (byte)218, (byte)161}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)240, (byte)1, (byte)120, (byte)129, (byte)218, (byte)167, (byte)229, (byte)237, (byte)179, (byte)5, (byte)85, (byte)22, (byte)241, (byte)154, (byte)13, (byte)61, (byte)238, (byte)232, (byte)153, (byte)21}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)5, (byte)66, (byte)194, (byte)188, (byte)170, (byte)36, (byte)7, (byte)215, (byte)224, (byte)71, (byte)109, (byte)132, (byte)167, (byte)42, (byte)151, (byte)150, (byte)40, (byte)66, (byte)205, (byte)13}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_used_SET(new byte[] {(byte)5, (byte)66, (byte)194, (byte)188, (byte)170, (byte)36, (byte)7, (byte)215, (byte)224, (byte)71, (byte)109, (byte)132, (byte)167, (byte)42, (byte)151, (byte)150, (byte)40, (byte)66, (byte)205, (byte)13}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)115, (byte)81, (byte)187, (byte)211, (byte)150, (byte)24, (byte)40, (byte)182, (byte)120, (byte)100, (byte)214, (byte)169, (byte)53, (byte)251, (byte)48, (byte)200, (byte)153, (byte)188, (byte)218, (byte)161}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)250, (byte)56, (byte)124, (byte)177, (byte)88, (byte)19, (byte)161, (byte)27, (byte)35, (byte)186, (byte)72, (byte)189, (byte)138, (byte)118, (byte)192, (byte)238, (byte)29, (byte)38, (byte)215, (byte)196}, 0) ;
            p25.satellites_visible = (byte)(byte)153;
            p25.satellite_azimuth_SET(new byte[] {(byte)240, (byte)1, (byte)120, (byte)129, (byte)218, (byte)167, (byte)229, (byte)237, (byte)179, (byte)5, (byte)85, (byte)22, (byte)241, (byte)154, (byte)13, (byte)61, (byte)238, (byte)232, (byte)153, (byte)21}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)249, (byte)234, (byte)109, (byte)183, (byte)192, (byte)131, (byte)35, (byte)197, (byte)165, (byte)100, (byte)224, (byte)55, (byte)90, (byte)61, (byte)185, (byte)161, (byte)172, (byte)87, (byte)114, (byte)132}, 0) ;
            ADV_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short)23533);
                Debug.Assert(pack.xgyro == (short)(short) -177);
                Debug.Assert(pack.xmag == (short)(short) -19532);
                Debug.Assert(pack.yacc == (short)(short) -31863);
                Debug.Assert(pack.zmag == (short)(short) -26822);
                Debug.Assert(pack.zgyro == (short)(short) -968);
                Debug.Assert(pack.ygyro == (short)(short) -16707);
                Debug.Assert(pack.time_boot_ms == (uint)3818475959U);
                Debug.Assert(pack.ymag == (short)(short) -29511);
                Debug.Assert(pack.xacc == (short)(short)17945);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.yacc = (short)(short) -31863;
            p26.xgyro = (short)(short) -177;
            p26.ygyro = (short)(short) -16707;
            p26.zgyro = (short)(short) -968;
            p26.ymag = (short)(short) -29511;
            p26.xacc = (short)(short)17945;
            p26.zacc = (short)(short)23533;
            p26.time_boot_ms = (uint)3818475959U;
            p26.xmag = (short)(short) -19532;
            p26.zmag = (short)(short) -26822;
            ADV_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6086875354473160732L);
                Debug.Assert(pack.zacc == (short)(short)9866);
                Debug.Assert(pack.zgyro == (short)(short) -3915);
                Debug.Assert(pack.xacc == (short)(short) -24284);
                Debug.Assert(pack.xgyro == (short)(short) -31012);
                Debug.Assert(pack.zmag == (short)(short)29679);
                Debug.Assert(pack.ymag == (short)(short) -9711);
                Debug.Assert(pack.xmag == (short)(short) -16793);
                Debug.Assert(pack.ygyro == (short)(short) -28295);
                Debug.Assert(pack.yacc == (short)(short) -12047);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.xgyro = (short)(short) -31012;
            p27.time_usec = (ulong)6086875354473160732L;
            p27.zgyro = (short)(short) -3915;
            p27.ymag = (short)(short) -9711;
            p27.xacc = (short)(short) -24284;
            p27.xmag = (short)(short) -16793;
            p27.zmag = (short)(short)29679;
            p27.ygyro = (short)(short) -28295;
            p27.zacc = (short)(short)9866;
            p27.yacc = (short)(short) -12047;
            ADV_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff2 == (short)(short)5238);
                Debug.Assert(pack.time_usec == (ulong)4741727380683769836L);
                Debug.Assert(pack.temperature == (short)(short) -14591);
                Debug.Assert(pack.press_diff1 == (short)(short)1833);
                Debug.Assert(pack.press_abs == (short)(short)23708);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.temperature = (short)(short) -14591;
            p28.press_diff1 = (short)(short)1833;
            p28.time_usec = (ulong)4741727380683769836L;
            p28.press_diff2 = (short)(short)5238;
            p28.press_abs = (short)(short)23708;
            ADV_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float) -8.4723103E37F);
                Debug.Assert(pack.press_abs == (float) -2.2195014E38F);
                Debug.Assert(pack.temperature == (short)(short) -27700);
                Debug.Assert(pack.time_boot_ms == (uint)3536321618U);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_diff = (float) -8.4723103E37F;
            p29.press_abs = (float) -2.2195014E38F;
            p29.temperature = (short)(short) -27700;
            p29.time_boot_ms = (uint)3536321618U;
            ADV_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)208254259U);
                Debug.Assert(pack.yaw == (float) -3.2148433E38F);
                Debug.Assert(pack.yawspeed == (float)9.496017E37F);
                Debug.Assert(pack.pitchspeed == (float) -3.362341E38F);
                Debug.Assert(pack.rollspeed == (float) -3.170699E38F);
                Debug.Assert(pack.pitch == (float) -1.244951E38F);
                Debug.Assert(pack.roll == (float) -2.4226695E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.pitchspeed = (float) -3.362341E38F;
            p30.yawspeed = (float)9.496017E37F;
            p30.time_boot_ms = (uint)208254259U;
            p30.yaw = (float) -3.2148433E38F;
            p30.pitch = (float) -1.244951E38F;
            p30.roll = (float) -2.4226695E38F;
            p30.rollspeed = (float) -3.170699E38F;
            ADV_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q1 == (float) -1.7912221E38F);
                Debug.Assert(pack.q4 == (float) -2.4436294E38F);
                Debug.Assert(pack.q2 == (float) -2.190248E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4035172161U);
                Debug.Assert(pack.yawspeed == (float) -1.3579665E38F);
                Debug.Assert(pack.q3 == (float) -2.6437867E38F);
                Debug.Assert(pack.pitchspeed == (float)1.6095734E38F);
                Debug.Assert(pack.rollspeed == (float)1.7809716E38F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.rollspeed = (float)1.7809716E38F;
            p31.q4 = (float) -2.4436294E38F;
            p31.q2 = (float) -2.190248E38F;
            p31.yawspeed = (float) -1.3579665E38F;
            p31.pitchspeed = (float)1.6095734E38F;
            p31.q3 = (float) -2.6437867E38F;
            p31.q1 = (float) -1.7912221E38F;
            p31.time_boot_ms = (uint)4035172161U;
            ADV_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)6.205528E37F);
                Debug.Assert(pack.vy == (float)2.7318586E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1638390632U);
                Debug.Assert(pack.vx == (float) -1.911266E38F);
                Debug.Assert(pack.y == (float)3.007036E38F);
                Debug.Assert(pack.x == (float)9.928987E37F);
                Debug.Assert(pack.vz == (float) -2.5246909E38F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.time_boot_ms = (uint)1638390632U;
            p32.vx = (float) -1.911266E38F;
            p32.x = (float)9.928987E37F;
            p32.z = (float)6.205528E37F;
            p32.y = (float)3.007036E38F;
            p32.vy = (float)2.7318586E38F;
            p32.vz = (float) -2.5246909E38F;
            ADV_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (short)(short)7016);
                Debug.Assert(pack.lat == (int) -1105436255);
                Debug.Assert(pack.alt == (int)502963779);
                Debug.Assert(pack.vx == (short)(short) -14335);
                Debug.Assert(pack.relative_alt == (int)1746453645);
                Debug.Assert(pack.lon == (int) -1602029797);
                Debug.Assert(pack.time_boot_ms == (uint)2855995137U);
                Debug.Assert(pack.vz == (short)(short)8798);
                Debug.Assert(pack.hdg == (ushort)(ushort)56070);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.relative_alt = (int)1746453645;
            p33.hdg = (ushort)(ushort)56070;
            p33.time_boot_ms = (uint)2855995137U;
            p33.vz = (short)(short)8798;
            p33.lat = (int) -1105436255;
            p33.vx = (short)(short) -14335;
            p33.vy = (short)(short)7016;
            p33.alt = (int)502963779;
            p33.lon = (int) -1602029797;
            ADV_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan8_scaled == (short)(short)24872);
                Debug.Assert(pack.chan5_scaled == (short)(short) -8242);
                Debug.Assert(pack.chan4_scaled == (short)(short) -5514);
                Debug.Assert(pack.chan1_scaled == (short)(short)16728);
                Debug.Assert(pack.chan7_scaled == (short)(short) -23262);
                Debug.Assert(pack.chan2_scaled == (short)(short)28352);
                Debug.Assert(pack.chan3_scaled == (short)(short)21520);
                Debug.Assert(pack.rssi == (byte)(byte)0);
                Debug.Assert(pack.port == (byte)(byte)168);
                Debug.Assert(pack.time_boot_ms == (uint)3047958245U);
                Debug.Assert(pack.chan6_scaled == (short)(short) -4706);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.rssi = (byte)(byte)0;
            p34.chan1_scaled = (short)(short)16728;
            p34.chan4_scaled = (short)(short) -5514;
            p34.chan8_scaled = (short)(short)24872;
            p34.time_boot_ms = (uint)3047958245U;
            p34.chan7_scaled = (short)(short) -23262;
            p34.chan6_scaled = (short)(short) -4706;
            p34.chan2_scaled = (short)(short)28352;
            p34.port = (byte)(byte)168;
            p34.chan5_scaled = (short)(short) -8242;
            p34.chan3_scaled = (short)(short)21520;
            ADV_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)55892);
                Debug.Assert(pack.rssi == (byte)(byte)150);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)28900);
                Debug.Assert(pack.time_boot_ms == (uint)18404252U);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)10108);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)49160);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)55756);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)49825);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)37936);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)18375);
                Debug.Assert(pack.port == (byte)(byte)26);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan7_raw = (ushort)(ushort)55892;
            p35.port = (byte)(byte)26;
            p35.chan1_raw = (ushort)(ushort)18375;
            p35.chan3_raw = (ushort)(ushort)49825;
            p35.chan5_raw = (ushort)(ushort)55756;
            p35.rssi = (byte)(byte)150;
            p35.chan4_raw = (ushort)(ushort)28900;
            p35.time_boot_ms = (uint)18404252U;
            p35.chan8_raw = (ushort)(ushort)37936;
            p35.chan6_raw = (ushort)(ushort)10108;
            p35.chan2_raw = (ushort)(ushort)49160;
            ADV_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)6178);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)20179);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)45785);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)51123);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)61661);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)10501);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)2606);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)54512);
                Debug.Assert(pack.port == (byte)(byte)130);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)9431);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)21611);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)24504);
                Debug.Assert(pack.time_usec == (uint)3423467928U);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)12644);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)17284);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)51034);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)40526);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)26096);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo9_raw_SET((ushort)(ushort)51123, PH) ;
            p36.servo15_raw_SET((ushort)(ushort)40526, PH) ;
            p36.port = (byte)(byte)130;
            p36.servo6_raw = (ushort)(ushort)61661;
            p36.servo10_raw_SET((ushort)(ushort)9431, PH) ;
            p36.servo5_raw = (ushort)(ushort)51034;
            p36.time_usec = (uint)3423467928U;
            p36.servo7_raw = (ushort)(ushort)54512;
            p36.servo3_raw = (ushort)(ushort)17284;
            p36.servo12_raw_SET((ushort)(ushort)20179, PH) ;
            p36.servo8_raw = (ushort)(ushort)12644;
            p36.servo1_raw = (ushort)(ushort)6178;
            p36.servo13_raw_SET((ushort)(ushort)2606, PH) ;
            p36.servo4_raw = (ushort)(ushort)24504;
            p36.servo11_raw_SET((ushort)(ushort)10501, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)45785, PH) ;
            p36.servo14_raw_SET((ushort)(ushort)21611, PH) ;
            p36.servo2_raw = (ushort)(ushort)26096;
            ADV_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short) -24219);
                Debug.Assert(pack.target_system == (byte)(byte)85);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.start_index == (short)(short)17364);
                Debug.Assert(pack.target_component == (byte)(byte)211);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p37.end_index = (short)(short) -24219;
            p37.start_index = (short)(short)17364;
            p37.target_component = (byte)(byte)211;
            p37.target_system = (byte)(byte)85;
            ADV_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short) -14667);
                Debug.Assert(pack.target_component == (byte)(byte)186);
                Debug.Assert(pack.start_index == (short)(short) -27347);
                Debug.Assert(pack.target_system == (byte)(byte)33);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_component = (byte)(byte)186;
            p38.start_index = (short)(short) -27347;
            p38.end_index = (short)(short) -14667;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p38.target_system = (byte)(byte)33;
            ADV_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.current == (byte)(byte)19);
                Debug.Assert(pack.param3 == (float) -7.693307E37F);
                Debug.Assert(pack.param1 == (float) -6.649225E37F);
                Debug.Assert(pack.y == (float)2.3737569E38F);
                Debug.Assert(pack.target_component == (byte)(byte)135);
                Debug.Assert(pack.target_system == (byte)(byte)5);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_PANORAMA_CREATE);
                Debug.Assert(pack.param4 == (float)2.1159946E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.z == (float) -7.936267E37F);
                Debug.Assert(pack.autocontinue == (byte)(byte)199);
                Debug.Assert(pack.seq == (ushort)(ushort)60563);
                Debug.Assert(pack.param2 == (float) -2.7516772E38F);
                Debug.Assert(pack.x == (float) -2.5351894E38F);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p39.x = (float) -2.5351894E38F;
            p39.seq = (ushort)(ushort)60563;
            p39.autocontinue = (byte)(byte)199;
            p39.param4 = (float)2.1159946E38F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p39.param1 = (float) -6.649225E37F;
            p39.target_system = (byte)(byte)5;
            p39.z = (float) -7.936267E37F;
            p39.current = (byte)(byte)19;
            p39.param2 = (float) -2.7516772E38F;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_PANORAMA_CREATE;
            p39.target_component = (byte)(byte)135;
            p39.y = (float)2.3737569E38F;
            p39.param3 = (float) -7.693307E37F;
            ADV_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)61434);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)162);
                Debug.Assert(pack.target_system == (byte)(byte)56);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_component = (byte)(byte)162;
            p40.target_system = (byte)(byte)56;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p40.seq = (ushort)(ushort)61434;
            ADV_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)50317);
                Debug.Assert(pack.target_system == (byte)(byte)106);
                Debug.Assert(pack.target_component == (byte)(byte)88);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)106;
            p41.target_component = (byte)(byte)88;
            p41.seq = (ushort)(ushort)50317;
            ADV_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)5296);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)5296;
            ADV_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)7);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)71);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_component = (byte)(byte)71;
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p43.target_system = (byte)(byte)7;
            ADV_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)85);
                Debug.Assert(pack.target_component == (byte)(byte)206);
                Debug.Assert(pack.count == (ushort)(ushort)4570);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p44.count = (ushort)(ushort)4570;
            p44.target_system = (byte)(byte)85;
            p44.target_component = (byte)(byte)206;
            ADV_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)37);
                Debug.Assert(pack.target_system == (byte)(byte)16);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_component = (byte)(byte)37;
            p45.target_system = (byte)(byte)16;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            ADV_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)58202);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)58202;
            ADV_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)103);
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM7);
                Debug.Assert(pack.target_component == (byte)(byte)21);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM7;
            p47.target_component = (byte)(byte)21;
            p47.target_system = (byte)(byte)103;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            ADV_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2157390933069835158L);
                Debug.Assert(pack.target_system == (byte)(byte)220);
                Debug.Assert(pack.latitude == (int)1933960642);
                Debug.Assert(pack.longitude == (int)1954838938);
                Debug.Assert(pack.altitude == (int) -1377314463);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.time_usec_SET((ulong)2157390933069835158L, PH) ;
            p48.target_system = (byte)(byte)220;
            p48.longitude = (int)1954838938;
            p48.latitude = (int)1933960642;
            p48.altitude = (int) -1377314463;
            ADV_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int) -349353812);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3865815016734014481L);
                Debug.Assert(pack.latitude == (int)842467280);
                Debug.Assert(pack.altitude == (int) -1992491853);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.time_usec_SET((ulong)3865815016734014481L, PH) ;
            p49.longitude = (int) -349353812;
            p49.latitude = (int)842467280;
            p49.altitude = (int) -1992491853;
            ADV_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)213);
                Debug.Assert(pack.param_value_min == (float) -1.9983654E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("aveJliiZ"));
                Debug.Assert(pack.target_component == (byte)(byte)91);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)182);
                Debug.Assert(pack.param_value0 == (float) -3.3161608E38F);
                Debug.Assert(pack.param_index == (short)(short) -28488);
                Debug.Assert(pack.param_value_max == (float)2.3230539E38F);
                Debug.Assert(pack.scale == (float) -2.2411386E37F);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_component = (byte)(byte)91;
            p50.param_value_min = (float) -1.9983654E38F;
            p50.param_index = (short)(short) -28488;
            p50.parameter_rc_channel_index = (byte)(byte)182;
            p50.param_value_max = (float)2.3230539E38F;
            p50.scale = (float) -2.2411386E37F;
            p50.param_id_SET("aveJliiZ", PH) ;
            p50.target_system = (byte)(byte)213;
            p50.param_value0 = (float) -3.3161608E38F;
            ADV_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)180);
                Debug.Assert(pack.seq == (ushort)(ushort)13276);
                Debug.Assert(pack.target_system == (byte)(byte)101);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.seq = (ushort)(ushort)13276;
            p51.target_component = (byte)(byte)180;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p51.target_system = (byte)(byte)101;
            ADV_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2x == (float)3.8479714E37F);
                Debug.Assert(pack.p2z == (float)5.845593E37F);
                Debug.Assert(pack.target_component == (byte)(byte)228);
                Debug.Assert(pack.target_system == (byte)(byte)13);
                Debug.Assert(pack.p1y == (float)1.5328546E38F);
                Debug.Assert(pack.p2y == (float)4.231354E37F);
                Debug.Assert(pack.p1x == (float) -1.0461039E38F);
                Debug.Assert(pack.p1z == (float)2.1863103E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p2y = (float)4.231354E37F;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p54.target_component = (byte)(byte)228;
            p54.p1z = (float)2.1863103E38F;
            p54.p1y = (float)1.5328546E38F;
            p54.p1x = (float) -1.0461039E38F;
            p54.p2z = (float)5.845593E37F;
            p54.target_system = (byte)(byte)13;
            p54.p2x = (float)3.8479714E37F;
            ADV_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.p2z == (float)2.402476E38F);
                Debug.Assert(pack.p1z == (float)1.386975E38F);
                Debug.Assert(pack.p1x == (float) -2.007704E38F);
                Debug.Assert(pack.p2y == (float) -9.732062E37F);
                Debug.Assert(pack.p2x == (float)2.1644734E38F);
                Debug.Assert(pack.p1y == (float) -1.4581843E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p1x = (float) -2.007704E38F;
            p55.p2z = (float)2.402476E38F;
            p55.p1y = (float) -1.4581843E38F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p55.p2y = (float) -9.732062E37F;
            p55.p2x = (float)2.1644734E38F;
            p55.p1z = (float)1.386975E38F;
            ADV_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.0574156E38F, 1.7747855E38F, -1.246317E38F, -1.6953302E38F, 6.115796E37F, 6.1583795E37F, -7.085866E37F, -2.3330416E38F, -4.1695432E37F}));
                Debug.Assert(pack.rollspeed == (float)5.2694197E37F);
                Debug.Assert(pack.yawspeed == (float)1.1236277E38F);
                Debug.Assert(pack.pitchspeed == (float) -2.7224513E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.608023E38F, 7.7443934E37F, -2.2345823E37F, -5.4148243E37F}));
                Debug.Assert(pack.time_usec == (ulong)6138010271435000545L);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.q_SET(new float[] {-2.608023E38F, 7.7443934E37F, -2.2345823E37F, -5.4148243E37F}, 0) ;
            p61.pitchspeed = (float) -2.7224513E37F;
            p61.rollspeed = (float)5.2694197E37F;
            p61.covariance_SET(new float[] {-1.0574156E38F, 1.7747855E38F, -1.246317E38F, -1.6953302E38F, 6.115796E37F, 6.1583795E37F, -7.085866E37F, -2.3330416E38F, -4.1695432E37F}, 0) ;
            p61.time_usec = (ulong)6138010271435000545L;
            p61.yawspeed = (float)1.1236277E38F;
            ADV_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_pitch == (float)1.6096928E38F);
                Debug.Assert(pack.nav_roll == (float)1.1962734E38F);
                Debug.Assert(pack.target_bearing == (short)(short)19762);
                Debug.Assert(pack.nav_bearing == (short)(short) -18938);
                Debug.Assert(pack.alt_error == (float) -1.5686183E38F);
                Debug.Assert(pack.xtrack_error == (float)6.0790524E37F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)8102);
                Debug.Assert(pack.aspd_error == (float)1.7250159E38F);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_pitch = (float)1.6096928E38F;
            p62.nav_bearing = (short)(short) -18938;
            p62.alt_error = (float) -1.5686183E38F;
            p62.wp_dist = (ushort)(ushort)8102;
            p62.xtrack_error = (float)6.0790524E37F;
            p62.aspd_error = (float)1.7250159E38F;
            p62.nav_roll = (float)1.1962734E38F;
            p62.target_bearing = (short)(short)19762;
            ADV_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)223605294);
                Debug.Assert(pack.lon == (int)1918904389);
                Debug.Assert(pack.alt == (int) -344593767);
                Debug.Assert(pack.relative_alt == (int)2113765551);
                Debug.Assert(pack.vx == (float) -7.434784E37F);
                Debug.Assert(pack.time_usec == (ulong)8869698817605485871L);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.vy == (float)2.9620979E38F);
                Debug.Assert(pack.vz == (float)1.6341976E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.9083795E38F, 2.0278073E38F, -8.553875E37F, 2.3644708E38F, -2.3448985E38F, -2.6591002E37F, -1.7154288E38F, -3.4063382E37F, -1.338378E38F, -1.638921E37F, -8.585168E37F, 1.8473912E38F, -3.0833733E38F, -4.508121E37F, -1.5722069E38F, 7.9155E37F, -2.8355277E38F, -3.2347928E38F, 1.9235271E38F, -1.848111E38F, 2.0117873E38F, -3.0894233E38F, -1.2111624E38F, 2.5518184E38F, -3.39443E38F, 1.914975E38F, 2.9787434E37F, 1.159514E38F, 2.5525439E38F, -1.4951456E38F, 3.884262E37F, 1.5884072E38F, 1.3210417E38F, 1.1115471E38F, -1.3587669E38F, -1.4492263E37F}));
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.time_usec = (ulong)8869698817605485871L;
            p63.lat = (int)223605294;
            p63.lon = (int)1918904389;
            p63.relative_alt = (int)2113765551;
            p63.vz = (float)1.6341976E38F;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p63.alt = (int) -344593767;
            p63.covariance_SET(new float[] {-2.9083795E38F, 2.0278073E38F, -8.553875E37F, 2.3644708E38F, -2.3448985E38F, -2.6591002E37F, -1.7154288E38F, -3.4063382E37F, -1.338378E38F, -1.638921E37F, -8.585168E37F, 1.8473912E38F, -3.0833733E38F, -4.508121E37F, -1.5722069E38F, 7.9155E37F, -2.8355277E38F, -3.2347928E38F, 1.9235271E38F, -1.848111E38F, 2.0117873E38F, -3.0894233E38F, -1.2111624E38F, 2.5518184E38F, -3.39443E38F, 1.914975E38F, 2.9787434E37F, 1.159514E38F, 2.5525439E38F, -1.4951456E38F, 3.884262E37F, 1.5884072E38F, 1.3210417E38F, 1.1115471E38F, -1.3587669E38F, -1.4492263E37F}, 0) ;
            p63.vx = (float) -7.434784E37F;
            p63.vy = (float)2.9620979E38F;
            ADV_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ay == (float)1.2698902E37F);
                Debug.Assert(pack.time_usec == (ulong)3751251069247550068L);
                Debug.Assert(pack.vz == (float)8.037586E37F);
                Debug.Assert(pack.x == (float)3.0451182E38F);
                Debug.Assert(pack.y == (float)3.7069156E37F);
                Debug.Assert(pack.az == (float) -5.5795175E37F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.7350916E38F, -1.85115E38F, 3.0771543E38F, 6.4930316E37F, -3.2579917E36F, 1.25715E37F, 1.4175251E38F, -1.1311403E38F, -2.2290275E38F, -4.422233E37F, -2.605317E38F, -2.2058279E38F, 3.250113E38F, 1.7700144E38F, -1.7768786E38F, 1.7882846E38F, 2.777073E38F, 1.3443557E38F, 4.085667E37F, -2.8856898E38F, 2.7880058E38F, 9.046596E37F, 1.815989E38F, 5.6933074E37F, -1.5975091E38F, 3.1679661E38F, -2.3756845E38F, -1.789777E38F, 2.161218E38F, 2.9451349E38F, 3.1588143E38F, -2.1840851E38F, -2.0358195E38F, 3.335246E38F, 1.4550612E38F, -3.62769E37F, -2.5899998E38F, -1.0403774E38F, 1.250462E38F, 3.1050398E38F, 3.3056827E38F, 3.044621E38F, -3.0118484E38F, 2.133914E38F, 1.411094E37F}));
                Debug.Assert(pack.vy == (float)1.4234178E38F);
                Debug.Assert(pack.ax == (float)2.5312816E38F);
                Debug.Assert(pack.z == (float)2.094937E38F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
                Debug.Assert(pack.vx == (float)8.90532E37F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.y = (float)3.7069156E37F;
            p64.vx = (float)8.90532E37F;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            p64.az = (float) -5.5795175E37F;
            p64.vz = (float)8.037586E37F;
            p64.vy = (float)1.4234178E38F;
            p64.ax = (float)2.5312816E38F;
            p64.x = (float)3.0451182E38F;
            p64.z = (float)2.094937E38F;
            p64.covariance_SET(new float[] {-1.7350916E38F, -1.85115E38F, 3.0771543E38F, 6.4930316E37F, -3.2579917E36F, 1.25715E37F, 1.4175251E38F, -1.1311403E38F, -2.2290275E38F, -4.422233E37F, -2.605317E38F, -2.2058279E38F, 3.250113E38F, 1.7700144E38F, -1.7768786E38F, 1.7882846E38F, 2.777073E38F, 1.3443557E38F, 4.085667E37F, -2.8856898E38F, 2.7880058E38F, 9.046596E37F, 1.815989E38F, 5.6933074E37F, -1.5975091E38F, 3.1679661E38F, -2.3756845E38F, -1.789777E38F, 2.161218E38F, 2.9451349E38F, 3.1588143E38F, -2.1840851E38F, -2.0358195E38F, 3.335246E38F, 1.4550612E38F, -3.62769E37F, -2.5899998E38F, -1.0403774E38F, 1.250462E38F, 3.1050398E38F, 3.3056827E38F, 3.044621E38F, -3.0118484E38F, 2.133914E38F, 1.411094E37F}, 0) ;
            p64.ay = (float)1.2698902E37F;
            p64.time_usec = (ulong)3751251069247550068L;
            ADV_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)31926);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)58926);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)60132);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)16637);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)51908);
                Debug.Assert(pack.chancount == (byte)(byte)242);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)19876);
                Debug.Assert(pack.time_boot_ms == (uint)984327279U);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)8010);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)46137);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)16835);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)29795);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)8416);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)38040);
                Debug.Assert(pack.rssi == (byte)(byte)27);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)31635);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)51919);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)51091);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)64271);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)5367);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)7144);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan7_raw = (ushort)(ushort)64271;
            p65.chan1_raw = (ushort)(ushort)16835;
            p65.chan12_raw = (ushort)(ushort)31926;
            p65.chan5_raw = (ushort)(ushort)60132;
            p65.chan11_raw = (ushort)(ushort)46137;
            p65.rssi = (byte)(byte)27;
            p65.chan13_raw = (ushort)(ushort)7144;
            p65.chan8_raw = (ushort)(ushort)31635;
            p65.chan10_raw = (ushort)(ushort)16637;
            p65.chan14_raw = (ushort)(ushort)29795;
            p65.chan17_raw = (ushort)(ushort)51919;
            p65.chan16_raw = (ushort)(ushort)19876;
            p65.chan6_raw = (ushort)(ushort)38040;
            p65.chan3_raw = (ushort)(ushort)5367;
            p65.chan18_raw = (ushort)(ushort)8010;
            p65.chan2_raw = (ushort)(ushort)51908;
            p65.chan4_raw = (ushort)(ushort)8416;
            p65.chan15_raw = (ushort)(ushort)51091;
            p65.chancount = (byte)(byte)242;
            p65.chan9_raw = (ushort)(ushort)58926;
            p65.time_boot_ms = (uint)984327279U;
            ADV_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_stop == (byte)(byte)105);
                Debug.Assert(pack.target_system == (byte)(byte)202);
                Debug.Assert(pack.req_stream_id == (byte)(byte)122);
                Debug.Assert(pack.target_component == (byte)(byte)68);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)48334);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.req_message_rate = (ushort)(ushort)48334;
            p66.start_stop = (byte)(byte)105;
            p66.req_stream_id = (byte)(byte)122;
            p66.target_system = (byte)(byte)202;
            p66.target_component = (byte)(byte)68;
            ADV_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.on_off == (byte)(byte)244);
                Debug.Assert(pack.stream_id == (byte)(byte)241);
                Debug.Assert(pack.message_rate == (ushort)(ushort)3269);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.on_off = (byte)(byte)244;
            p67.stream_id = (byte)(byte)241;
            p67.message_rate = (ushort)(ushort)3269;
            ADV_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.buttons == (ushort)(ushort)47845);
                Debug.Assert(pack.target == (byte)(byte)217);
                Debug.Assert(pack.z == (short)(short)1256);
                Debug.Assert(pack.r == (short)(short) -32508);
                Debug.Assert(pack.y == (short)(short) -26764);
                Debug.Assert(pack.x == (short)(short)7860);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.x = (short)(short)7860;
            p69.z = (short)(short)1256;
            p69.y = (short)(short) -26764;
            p69.target = (byte)(byte)217;
            p69.r = (short)(short) -32508;
            p69.buttons = (ushort)(ushort)47845;
            ADV_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)61);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)48663);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)56174);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)11623);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)33931);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)45644);
                Debug.Assert(pack.target_system == (byte)(byte)119);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)39868);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)28912);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)15486);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan2_raw = (ushort)(ushort)39868;
            p70.chan3_raw = (ushort)(ushort)56174;
            p70.chan6_raw = (ushort)(ushort)48663;
            p70.chan5_raw = (ushort)(ushort)33931;
            p70.chan4_raw = (ushort)(ushort)45644;
            p70.target_system = (byte)(byte)119;
            p70.target_component = (byte)(byte)61;
            p70.chan7_raw = (ushort)(ushort)15486;
            p70.chan8_raw = (ushort)(ushort)28912;
            p70.chan1_raw = (ushort)(ushort)11623;
            ADV_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.y == (int) -1402160705);
                Debug.Assert(pack.param2 == (float)1.7662983E38F);
                Debug.Assert(pack.param3 == (float) -3.372448E38F);
                Debug.Assert(pack.target_system == (byte)(byte)30);
                Debug.Assert(pack.param4 == (float)1.3500301E38F);
                Debug.Assert(pack.target_component == (byte)(byte)195);
                Debug.Assert(pack.param1 == (float) -4.3089224E37F);
                Debug.Assert(pack.autocontinue == (byte)(byte)190);
                Debug.Assert(pack.x == (int)299569813);
                Debug.Assert(pack.seq == (ushort)(ushort)34641);
                Debug.Assert(pack.current == (byte)(byte)71);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.z == (float) -3.0861065E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.z = (float) -3.0861065E38F;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL;
            p73.param4 = (float)1.3500301E38F;
            p73.target_component = (byte)(byte)195;
            p73.current = (byte)(byte)71;
            p73.x = (int)299569813;
            p73.param1 = (float) -4.3089224E37F;
            p73.target_system = (byte)(byte)30;
            p73.seq = (ushort)(ushort)34641;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p73.autocontinue = (byte)(byte)190;
            p73.y = (int) -1402160705;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p73.param2 = (float)1.7662983E38F;
            p73.param3 = (float) -3.372448E38F;
            ADV_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.climb == (float) -7.024138E37F);
                Debug.Assert(pack.airspeed == (float) -1.4107109E38F);
                Debug.Assert(pack.alt == (float)1.3678227E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)52938);
                Debug.Assert(pack.groundspeed == (float)2.7740187E38F);
                Debug.Assert(pack.heading == (short)(short) -27116);
            };
            GroundControl.VFR_HUD p74 = CommunicationChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.alt = (float)1.3678227E38F;
            p74.throttle = (ushort)(ushort)52938;
            p74.heading = (short)(short) -27116;
            p74.climb = (float) -7.024138E37F;
            p74.airspeed = (float) -1.4107109E38F;
            p74.groundspeed = (float)2.7740187E38F;
            CommunicationChannel.instance.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)48);
                Debug.Assert(pack.x == (int) -1689787131);
                Debug.Assert(pack.current == (byte)(byte)60);
                Debug.Assert(pack.param4 == (float) -2.6305395E38F);
                Debug.Assert(pack.param3 == (float) -4.5515913E37F);
                Debug.Assert(pack.param1 == (float)2.3934694E38F);
                Debug.Assert(pack.param2 == (float)6.5583415E37F);
                Debug.Assert(pack.z == (float)4.815302E37F);
                Debug.Assert(pack.autocontinue == (byte)(byte)97);
                Debug.Assert(pack.y == (int)2095945111);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.target_component == (byte)(byte)223);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_RALLY_LAND);
            };
            GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.autocontinue = (byte)(byte)97;
            p75.current = (byte)(byte)60;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p75.param4 = (float) -2.6305395E38F;
            p75.x = (int) -1689787131;
            p75.target_component = (byte)(byte)223;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_RALLY_LAND;
            p75.param3 = (float) -4.5515913E37F;
            p75.y = (int)2095945111;
            p75.target_system = (byte)(byte)48;
            p75.param2 = (float)6.5583415E37F;
            p75.z = (float)4.815302E37F;
            p75.param1 = (float)2.3934694E38F;
            CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param3 == (float) -1.7115591E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)71);
                Debug.Assert(pack.param1 == (float)3.1639803E37F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT);
                Debug.Assert(pack.param2 == (float) -1.540697E38F);
                Debug.Assert(pack.param7 == (float)2.327499E38F);
                Debug.Assert(pack.param5 == (float) -1.6521936E38F);
                Debug.Assert(pack.target_system == (byte)(byte)234);
                Debug.Assert(pack.target_component == (byte)(byte)150);
                Debug.Assert(pack.param4 == (float) -9.970396E37F);
                Debug.Assert(pack.param6 == (float) -2.5704685E38F);
            };
            GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_component = (byte)(byte)150;
            p76.param1 = (float)3.1639803E37F;
            p76.param2 = (float) -1.540697E38F;
            p76.param5 = (float) -1.6521936E38F;
            p76.param7 = (float)2.327499E38F;
            p76.param3 = (float) -1.7115591E38F;
            p76.param4 = (float) -9.970396E37F;
            p76.target_system = (byte)(byte)234;
            p76.param6 = (float) -2.5704685E38F;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT;
            p76.confirmation = (byte)(byte)71;
            CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)191);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)248);
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -1450266110);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_DENIED);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)48);
            };
            GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_DENIED;
            p77.target_component_SET((byte)(byte)248, PH) ;
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
            p77.result_param2_SET((int) -1450266110, PH) ;
            p77.target_system_SET((byte)(byte)191, PH) ;
            p77.progress_SET((byte)(byte)48, PH) ;
            CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)1.6580292E38F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)167);
                Debug.Assert(pack.thrust == (float) -1.6463825E38F);
                Debug.Assert(pack.pitch == (float)1.3859779E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)159);
                Debug.Assert(pack.time_boot_ms == (uint)365247969U);
                Debug.Assert(pack.yaw == (float) -3.3820366E38F);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.mode_switch = (byte)(byte)159;
            p81.thrust = (float) -1.6463825E38F;
            p81.pitch = (float)1.3859779E38F;
            p81.roll = (float)1.6580292E38F;
            p81.yaw = (float) -3.3820366E38F;
            p81.time_boot_ms = (uint)365247969U;
            p81.manual_override_switch = (byte)(byte)167;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_pitch_rate == (float)1.5417409E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3528374270U);
                Debug.Assert(pack.type_mask == (byte)(byte)211);
                Debug.Assert(pack.target_system == (byte)(byte)135);
                Debug.Assert(pack.body_yaw_rate == (float)6.45301E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-9.57323E37F, -1.5620151E38F, -1.0880557E38F, -4.7298295E37F}));
                Debug.Assert(pack.body_roll_rate == (float)2.8049564E38F);
                Debug.Assert(pack.thrust == (float)1.9871778E38F);
                Debug.Assert(pack.target_component == (byte)(byte)31);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.target_component = (byte)(byte)31;
            p82.time_boot_ms = (uint)3528374270U;
            p82.body_yaw_rate = (float)6.45301E37F;
            p82.body_roll_rate = (float)2.8049564E38F;
            p82.thrust = (float)1.9871778E38F;
            p82.type_mask = (byte)(byte)211;
            p82.target_system = (byte)(byte)135;
            p82.body_pitch_rate = (float)1.5417409E38F;
            p82.q_SET(new float[] {-9.57323E37F, -1.5620151E38F, -1.0880557E38F, -4.7298295E37F}, 0) ;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1638580835U);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.8774674E38F, 2.729821E38F, -2.1393446E38F, -3.3974446E38F}));
                Debug.Assert(pack.body_roll_rate == (float)2.9082154E38F);
                Debug.Assert(pack.thrust == (float) -2.3623498E37F);
                Debug.Assert(pack.body_pitch_rate == (float) -7.38774E37F);
                Debug.Assert(pack.body_yaw_rate == (float)1.4993114E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)163);
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.type_mask = (byte)(byte)163;
            p83.time_boot_ms = (uint)1638580835U;
            p83.thrust = (float) -2.3623498E37F;
            p83.body_roll_rate = (float)2.9082154E38F;
            p83.q_SET(new float[] {1.8774674E38F, 2.729821E38F, -2.1393446E38F, -3.3974446E38F}, 0) ;
            p83.body_pitch_rate = (float) -7.38774E37F;
            p83.body_yaw_rate = (float)1.4993114E38F;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)1.1892919E38F);
                Debug.Assert(pack.yaw == (float) -1.7497736E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)31225);
                Debug.Assert(pack.vz == (float) -4.324567E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1637437993U);
                Debug.Assert(pack.afy == (float) -2.4472171E38F);
                Debug.Assert(pack.yaw_rate == (float)2.1708533E38F);
                Debug.Assert(pack.vx == (float)2.5976822E38F);
                Debug.Assert(pack.afz == (float) -1.4832784E38F);
                Debug.Assert(pack.z == (float)6.424178E37F);
                Debug.Assert(pack.x == (float) -1.761652E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.afx == (float)1.7730158E38F);
                Debug.Assert(pack.target_component == (byte)(byte)225);
                Debug.Assert(pack.target_system == (byte)(byte)206);
                Debug.Assert(pack.vy == (float) -2.5729727E37F);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.yaw = (float) -1.7497736E38F;
            p84.x = (float) -1.761652E38F;
            p84.type_mask = (ushort)(ushort)31225;
            p84.afy = (float) -2.4472171E38F;
            p84.vz = (float) -4.324567E37F;
            p84.y = (float)1.1892919E38F;
            p84.afx = (float)1.7730158E38F;
            p84.z = (float)6.424178E37F;
            p84.afz = (float) -1.4832784E38F;
            p84.target_component = (byte)(byte)225;
            p84.time_boot_ms = (uint)1637437993U;
            p84.vy = (float) -2.5729727E37F;
            p84.yaw_rate = (float)2.1708533E38F;
            p84.vx = (float)2.5976822E38F;
            p84.target_system = (byte)(byte)206;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)169);
                Debug.Assert(pack.alt == (float)3.1944018E38F);
                Debug.Assert(pack.afz == (float)2.6329144E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)42796);
                Debug.Assert(pack.lon_int == (int)1723150335);
                Debug.Assert(pack.time_boot_ms == (uint)1453095851U);
                Debug.Assert(pack.vy == (float) -1.8654186E38F);
                Debug.Assert(pack.vx == (float) -2.129187E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.lat_int == (int)1964645314);
                Debug.Assert(pack.yaw == (float) -3.2097566E38F);
                Debug.Assert(pack.afy == (float) -3.1343314E38F);
                Debug.Assert(pack.afx == (float)1.1263348E38F);
                Debug.Assert(pack.yaw_rate == (float)5.1725235E37F);
                Debug.Assert(pack.vz == (float) -9.28778E37F);
                Debug.Assert(pack.target_system == (byte)(byte)243);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.afx = (float)1.1263348E38F;
            p86.yaw = (float) -3.2097566E38F;
            p86.afz = (float)2.6329144E38F;
            p86.alt = (float)3.1944018E38F;
            p86.target_system = (byte)(byte)243;
            p86.vx = (float) -2.129187E38F;
            p86.yaw_rate = (float)5.1725235E37F;
            p86.vz = (float) -9.28778E37F;
            p86.time_boot_ms = (uint)1453095851U;
            p86.vy = (float) -1.8654186E38F;
            p86.type_mask = (ushort)(ushort)42796;
            p86.lat_int = (int)1964645314;
            p86.afy = (float) -3.1343314E38F;
            p86.lon_int = (int)1723150335;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p86.target_component = (byte)(byte)169;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon_int == (int)565245449);
                Debug.Assert(pack.vz == (float) -1.5510527E38F);
                Debug.Assert(pack.vx == (float) -6.2539045E37F);
                Debug.Assert(pack.afy == (float) -8.323706E37F);
                Debug.Assert(pack.yaw_rate == (float)2.998704E38F);
                Debug.Assert(pack.afx == (float)1.4070672E38F);
                Debug.Assert(pack.afz == (float) -7.8598237E37F);
                Debug.Assert(pack.yaw == (float) -5.788046E37F);
                Debug.Assert(pack.time_boot_ms == (uint)87950735U);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.lat_int == (int) -705597566);
                Debug.Assert(pack.vy == (float) -2.3355146E38F);
                Debug.Assert(pack.alt == (float)1.2058973E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)22825);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.yaw_rate = (float)2.998704E38F;
            p87.afz = (float) -7.8598237E37F;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p87.vz = (float) -1.5510527E38F;
            p87.lat_int = (int) -705597566;
            p87.vx = (float) -6.2539045E37F;
            p87.type_mask = (ushort)(ushort)22825;
            p87.alt = (float)1.2058973E38F;
            p87.time_boot_ms = (uint)87950735U;
            p87.vy = (float) -2.3355146E38F;
            p87.lon_int = (int)565245449;
            p87.afy = (float) -8.323706E37F;
            p87.yaw = (float) -5.788046E37F;
            p87.afx = (float)1.4070672E38F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)2.461108E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3011590966U);
                Debug.Assert(pack.z == (float) -8.1005074E37F);
                Debug.Assert(pack.pitch == (float)9.40551E37F);
                Debug.Assert(pack.roll == (float)3.0894343E38F);
                Debug.Assert(pack.y == (float)3.3101888E38F);
                Debug.Assert(pack.yaw == (float) -2.8407561E38F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)3011590966U;
            p89.x = (float)2.461108E38F;
            p89.z = (float) -8.1005074E37F;
            p89.yaw = (float) -2.8407561E38F;
            p89.roll = (float)3.0894343E38F;
            p89.pitch = (float)9.40551E37F;
            p89.y = (float)3.3101888E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)2.6162805E38F);
                Debug.Assert(pack.pitchspeed == (float)1.9613737E38F);
                Debug.Assert(pack.rollspeed == (float)1.9714746E38F);
                Debug.Assert(pack.xacc == (short)(short)29067);
                Debug.Assert(pack.lat == (int)916108870);
                Debug.Assert(pack.time_usec == (ulong)3230410455280904981L);
                Debug.Assert(pack.yacc == (short)(short)11182);
                Debug.Assert(pack.vz == (short)(short)17088);
                Debug.Assert(pack.lon == (int) -1038429967);
                Debug.Assert(pack.zacc == (short)(short) -3365);
                Debug.Assert(pack.vx == (short)(short)28971);
                Debug.Assert(pack.vy == (short)(short)2901);
                Debug.Assert(pack.pitch == (float) -1.7504482E38F);
                Debug.Assert(pack.yawspeed == (float)1.898721E38F);
                Debug.Assert(pack.alt == (int)1491728081);
                Debug.Assert(pack.yaw == (float) -2.3533402E38F);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.roll = (float)2.6162805E38F;
            p90.yaw = (float) -2.3533402E38F;
            p90.pitchspeed = (float)1.9613737E38F;
            p90.pitch = (float) -1.7504482E38F;
            p90.vx = (short)(short)28971;
            p90.zacc = (short)(short) -3365;
            p90.yacc = (short)(short)11182;
            p90.vy = (short)(short)2901;
            p90.yawspeed = (float)1.898721E38F;
            p90.vz = (short)(short)17088;
            p90.xacc = (short)(short)29067;
            p90.alt = (int)1491728081;
            p90.lon = (int) -1038429967;
            p90.time_usec = (ulong)3230410455280904981L;
            p90.rollspeed = (float)1.9714746E38F;
            p90.lat = (int)916108870;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_mode == (byte)(byte)138);
                Debug.Assert(pack.aux3 == (float)1.6833533E38F);
                Debug.Assert(pack.throttle == (float)8.9137245E35F);
                Debug.Assert(pack.aux4 == (float) -2.859699E38F);
                Debug.Assert(pack.yaw_rudder == (float)1.1154067E38F);
                Debug.Assert(pack.aux1 == (float)3.406571E37F);
                Debug.Assert(pack.aux2 == (float)9.284001E37F);
                Debug.Assert(pack.time_usec == (ulong)6076288258431621078L);
                Debug.Assert(pack.roll_ailerons == (float)1.1998828E38F);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED);
                Debug.Assert(pack.pitch_elevator == (float) -2.3393999E38F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.pitch_elevator = (float) -2.3393999E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED;
            p91.nav_mode = (byte)(byte)138;
            p91.aux3 = (float)1.6833533E38F;
            p91.aux1 = (float)3.406571E37F;
            p91.aux2 = (float)9.284001E37F;
            p91.time_usec = (ulong)6076288258431621078L;
            p91.roll_ailerons = (float)1.1998828E38F;
            p91.aux4 = (float) -2.859699E38F;
            p91.throttle = (float)8.9137245E35F;
            p91.yaw_rudder = (float)1.1154067E38F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)2553922032287868470L);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)37016);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)31056);
                Debug.Assert(pack.rssi == (byte)(byte)244);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)28212);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)7752);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)10430);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)3197);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)23699);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)54045);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)30619);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)42911);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)23912);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)10098);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan3_raw = (ushort)(ushort)10430;
            p92.chan5_raw = (ushort)(ushort)30619;
            p92.chan8_raw = (ushort)(ushort)42911;
            p92.chan1_raw = (ushort)(ushort)31056;
            p92.rssi = (byte)(byte)244;
            p92.chan4_raw = (ushort)(ushort)23912;
            p92.chan2_raw = (ushort)(ushort)3197;
            p92.chan11_raw = (ushort)(ushort)10098;
            p92.chan9_raw = (ushort)(ushort)37016;
            p92.chan6_raw = (ushort)(ushort)28212;
            p92.chan12_raw = (ushort)(ushort)23699;
            p92.chan10_raw = (ushort)(ushort)7752;
            p92.chan7_raw = (ushort)(ushort)54045;
            p92.time_usec = (ulong)2553922032287868470L;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-3.2571507E38F, 3.133031E38F, -6.5261695E37F, -3.3469264E38F, 2.7488052E38F, 1.0965728E38F, 1.4885307E37F, 3.0770308E38F, -2.0081514E38F, -9.178043E37F, -4.8728765E36F, -3.3029444E38F, 2.8610432E38F, -1.9642753E38F, 8.654457E37F, 1.6550179E38F}));
                Debug.Assert(pack.flags == (ulong)891414474604444698L);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED);
                Debug.Assert(pack.time_usec == (ulong)7316003298369195843L);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.controls_SET(new float[] {-3.2571507E38F, 3.133031E38F, -6.5261695E37F, -3.3469264E38F, 2.7488052E38F, 1.0965728E38F, 1.4885307E37F, 3.0770308E38F, -2.0081514E38F, -9.178043E37F, -4.8728765E36F, -3.3029444E38F, 2.8610432E38F, -1.9642753E38F, 8.654457E37F, 1.6550179E38F}, 0) ;
            p93.flags = (ulong)891414474604444698L;
            p93.time_usec = (ulong)7316003298369195843L;
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1649091083244510585L);
                Debug.Assert(pack.flow_x == (short)(short)25779);
                Debug.Assert(pack.flow_comp_m_x == (float) -1.1485906E38F);
                Debug.Assert(pack.flow_comp_m_y == (float) -1.0790737E38F);
                Debug.Assert(pack.flow_y == (short)(short) -6011);
                Debug.Assert(pack.quality == (byte)(byte)211);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -3.145356E38F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -1.1502376E38F);
                Debug.Assert(pack.ground_distance == (float)1.2345112E37F);
                Debug.Assert(pack.sensor_id == (byte)(byte)139);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.quality = (byte)(byte)211;
            p100.sensor_id = (byte)(byte)139;
            p100.flow_comp_m_x = (float) -1.1485906E38F;
            p100.flow_y = (short)(short) -6011;
            p100.flow_comp_m_y = (float) -1.0790737E38F;
            p100.flow_rate_y_SET((float) -1.1502376E38F, PH) ;
            p100.ground_distance = (float)1.2345112E37F;
            p100.time_usec = (ulong)1649091083244510585L;
            p100.flow_x = (short)(short)25779;
            p100.flow_rate_x_SET((float) -3.145356E38F, PH) ;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)1.710293E38F);
                Debug.Assert(pack.usec == (ulong)6074243367826997054L);
                Debug.Assert(pack.roll == (float) -3.0567024E37F);
                Debug.Assert(pack.x == (float) -6.4469474E37F);
                Debug.Assert(pack.y == (float)1.7684342E38F);
                Debug.Assert(pack.z == (float)8.0493607E37F);
                Debug.Assert(pack.pitch == (float) -2.0836482E38F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.yaw = (float)1.710293E38F;
            p101.z = (float)8.0493607E37F;
            p101.x = (float) -6.4469474E37F;
            p101.usec = (ulong)6074243367826997054L;
            p101.pitch = (float) -2.0836482E38F;
            p101.roll = (float) -3.0567024E37F;
            p101.y = (float)1.7684342E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)4008265953392823042L);
                Debug.Assert(pack.y == (float)2.5684972E38F);
                Debug.Assert(pack.roll == (float) -1.7893712E38F);
                Debug.Assert(pack.z == (float) -2.444658E38F);
                Debug.Assert(pack.x == (float)2.5974807E37F);
                Debug.Assert(pack.yaw == (float) -2.5368117E37F);
                Debug.Assert(pack.pitch == (float) -1.1607306E37F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.z = (float) -2.444658E38F;
            p102.usec = (ulong)4008265953392823042L;
            p102.roll = (float) -1.7893712E38F;
            p102.yaw = (float) -2.5368117E37F;
            p102.x = (float)2.5974807E37F;
            p102.y = (float)2.5684972E38F;
            p102.pitch = (float) -1.1607306E37F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)6.678695E36F);
                Debug.Assert(pack.usec == (ulong)8609901223237448742L);
                Debug.Assert(pack.x == (float)1.7892955E38F);
                Debug.Assert(pack.z == (float) -2.4418348E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.y = (float)6.678695E36F;
            p103.z = (float) -2.4418348E38F;
            p103.usec = (ulong)8609901223237448742L;
            p103.x = (float)1.7892955E38F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)2.7779077E37F);
                Debug.Assert(pack.y == (float)2.3703799E38F);
                Debug.Assert(pack.z == (float) -2.0671469E38F);
                Debug.Assert(pack.pitch == (float)2.4595693E38F);
                Debug.Assert(pack.x == (float) -1.2814361E38F);
                Debug.Assert(pack.yaw == (float) -5.6032763E37F);
                Debug.Assert(pack.usec == (ulong)2950370073427944893L);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.pitch = (float)2.4595693E38F;
            p104.x = (float) -1.2814361E38F;
            p104.z = (float) -2.0671469E38F;
            p104.roll = (float)2.7779077E37F;
            p104.yaw = (float) -5.6032763E37F;
            p104.y = (float)2.3703799E38F;
            p104.usec = (ulong)2950370073427944893L;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (float) -2.6976347E38F);
                Debug.Assert(pack.yacc == (float) -2.3978643E38F);
                Debug.Assert(pack.diff_pressure == (float)2.0902433E38F);
                Debug.Assert(pack.zacc == (float)2.896276E38F);
                Debug.Assert(pack.abs_pressure == (float)1.5356989E38F);
                Debug.Assert(pack.xacc == (float)2.5368122E38F);
                Debug.Assert(pack.time_usec == (ulong)7070053634888191817L);
                Debug.Assert(pack.ygyro == (float)6.6513024E37F);
                Debug.Assert(pack.zmag == (float) -6.7389214E37F);
                Debug.Assert(pack.zgyro == (float)2.7401586E38F);
                Debug.Assert(pack.temperature == (float) -1.022753E38F);
                Debug.Assert(pack.pressure_alt == (float) -4.1103173E37F);
                Debug.Assert(pack.xmag == (float) -1.7197595E38F);
                Debug.Assert(pack.ymag == (float)2.5564375E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)26462);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.diff_pressure = (float)2.0902433E38F;
            p105.zmag = (float) -6.7389214E37F;
            p105.temperature = (float) -1.022753E38F;
            p105.zacc = (float)2.896276E38F;
            p105.xgyro = (float) -2.6976347E38F;
            p105.xacc = (float)2.5368122E38F;
            p105.zgyro = (float)2.7401586E38F;
            p105.time_usec = (ulong)7070053634888191817L;
            p105.ygyro = (float)6.6513024E37F;
            p105.pressure_alt = (float) -4.1103173E37F;
            p105.fields_updated = (ushort)(ushort)26462;
            p105.xmag = (float) -1.7197595E38F;
            p105.abs_pressure = (float)1.5356989E38F;
            p105.yacc = (float) -2.3978643E38F;
            p105.ymag = (float)2.5564375E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_zgyro == (float)2.079618E38F);
                Debug.Assert(pack.integrated_x == (float)2.8587975E38F);
                Debug.Assert(pack.integrated_ygyro == (float)3.3374892E38F);
                Debug.Assert(pack.integration_time_us == (uint)4251442687U);
                Debug.Assert(pack.distance == (float)2.1511534E37F);
                Debug.Assert(pack.time_delta_distance_us == (uint)3309388932U);
                Debug.Assert(pack.time_usec == (ulong)92933440106651654L);
                Debug.Assert(pack.sensor_id == (byte)(byte)212);
                Debug.Assert(pack.integrated_xgyro == (float)2.50125E38F);
                Debug.Assert(pack.quality == (byte)(byte)10);
                Debug.Assert(pack.integrated_y == (float)1.6980078E38F);
                Debug.Assert(pack.temperature == (short)(short) -14264);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_y = (float)1.6980078E38F;
            p106.integration_time_us = (uint)4251442687U;
            p106.time_delta_distance_us = (uint)3309388932U;
            p106.time_usec = (ulong)92933440106651654L;
            p106.quality = (byte)(byte)10;
            p106.integrated_zgyro = (float)2.079618E38F;
            p106.temperature = (short)(short) -14264;
            p106.integrated_xgyro = (float)2.50125E38F;
            p106.distance = (float)2.1511534E37F;
            p106.sensor_id = (byte)(byte)212;
            p106.integrated_ygyro = (float)3.3374892E38F;
            p106.integrated_x = (float)2.8587975E38F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (float)1.8552032E34F);
                Debug.Assert(pack.zacc == (float)2.4093375E38F);
                Debug.Assert(pack.diff_pressure == (float) -3.1226283E37F);
                Debug.Assert(pack.abs_pressure == (float)4.8757835E36F);
                Debug.Assert(pack.temperature == (float) -8.4590974E37F);
                Debug.Assert(pack.xgyro == (float) -2.4132376E38F);
                Debug.Assert(pack.time_usec == (ulong)7691933059540381147L);
                Debug.Assert(pack.zgyro == (float)8.080085E37F);
                Debug.Assert(pack.pressure_alt == (float) -2.740173E38F);
                Debug.Assert(pack.xacc == (float) -9.746878E37F);
                Debug.Assert(pack.zmag == (float)2.1443882E38F);
                Debug.Assert(pack.ymag == (float) -6.3366405E37F);
                Debug.Assert(pack.yacc == (float)2.9609704E37F);
                Debug.Assert(pack.ygyro == (float) -2.1191126E38F);
                Debug.Assert(pack.fields_updated == (uint)2364459296U);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.abs_pressure = (float)4.8757835E36F;
            p107.temperature = (float) -8.4590974E37F;
            p107.xmag = (float)1.8552032E34F;
            p107.zacc = (float)2.4093375E38F;
            p107.yacc = (float)2.9609704E37F;
            p107.zmag = (float)2.1443882E38F;
            p107.xacc = (float) -9.746878E37F;
            p107.time_usec = (ulong)7691933059540381147L;
            p107.ygyro = (float) -2.1191126E38F;
            p107.ymag = (float) -6.3366405E37F;
            p107.fields_updated = (uint)2364459296U;
            p107.diff_pressure = (float) -3.1226283E37F;
            p107.zgyro = (float)8.080085E37F;
            p107.pressure_alt = (float) -2.740173E38F;
            p107.xgyro = (float) -2.4132376E38F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (float) -2.2400341E38F);
                Debug.Assert(pack.q1 == (float) -2.0045627E38F);
                Debug.Assert(pack.zgyro == (float)2.0422326E38F);
                Debug.Assert(pack.vd == (float)2.499688E38F);
                Debug.Assert(pack.std_dev_horz == (float)1.0488558E38F);
                Debug.Assert(pack.lat == (float) -3.0642212E38F);
                Debug.Assert(pack.zacc == (float) -2.3659818E38F);
                Debug.Assert(pack.pitch == (float) -2.9293592E38F);
                Debug.Assert(pack.ve == (float) -3.3939187E37F);
                Debug.Assert(pack.xacc == (float) -1.0184877E38F);
                Debug.Assert(pack.q4 == (float)3.2685014E38F);
                Debug.Assert(pack.vn == (float)3.06836E38F);
                Debug.Assert(pack.alt == (float) -3.129824E38F);
                Debug.Assert(pack.std_dev_vert == (float)3.696779E37F);
                Debug.Assert(pack.yacc == (float)2.3044E38F);
                Debug.Assert(pack.q3 == (float)2.4450388E38F);
                Debug.Assert(pack.roll == (float)1.6365435E38F);
                Debug.Assert(pack.lon == (float) -5.6785996E37F);
                Debug.Assert(pack.yaw == (float)4.4269817E37F);
                Debug.Assert(pack.ygyro == (float)6.419354E37F);
                Debug.Assert(pack.q2 == (float) -2.3808486E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.xacc = (float) -1.0184877E38F;
            p108.pitch = (float) -2.9293592E38F;
            p108.roll = (float)1.6365435E38F;
            p108.yaw = (float)4.4269817E37F;
            p108.lat = (float) -3.0642212E38F;
            p108.q4 = (float)3.2685014E38F;
            p108.std_dev_vert = (float)3.696779E37F;
            p108.lon = (float) -5.6785996E37F;
            p108.ve = (float) -3.3939187E37F;
            p108.q1 = (float) -2.0045627E38F;
            p108.zacc = (float) -2.3659818E38F;
            p108.std_dev_horz = (float)1.0488558E38F;
            p108.vd = (float)2.499688E38F;
            p108.alt = (float) -3.129824E38F;
            p108.q2 = (float) -2.3808486E38F;
            p108.vn = (float)3.06836E38F;
            p108.zgyro = (float)2.0422326E38F;
            p108.q3 = (float)2.4450388E38F;
            p108.ygyro = (float)6.419354E37F;
            p108.xgyro = (float) -2.2400341E38F;
            p108.yacc = (float)2.3044E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fixed_ == (ushort)(ushort)56505);
                Debug.Assert(pack.noise == (byte)(byte)141);
                Debug.Assert(pack.txbuf == (byte)(byte)168);
                Debug.Assert(pack.remnoise == (byte)(byte)80);
                Debug.Assert(pack.remrssi == (byte)(byte)186);
                Debug.Assert(pack.rssi == (byte)(byte)145);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)24920);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.remrssi = (byte)(byte)186;
            p109.remnoise = (byte)(byte)80;
            p109.rxerrors = (ushort)(ushort)24920;
            p109.noise = (byte)(byte)141;
            p109.rssi = (byte)(byte)145;
            p109.txbuf = (byte)(byte)168;
            p109.fixed_ = (ushort)(ushort)56505;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)182);
                Debug.Assert(pack.target_system == (byte)(byte)61);
                Debug.Assert(pack.target_component == (byte)(byte)214);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)129, (byte)115, (byte)221, (byte)193, (byte)46, (byte)54, (byte)141, (byte)112, (byte)178, (byte)71, (byte)228, (byte)75, (byte)58, (byte)44, (byte)170, (byte)95, (byte)123, (byte)117, (byte)197, (byte)161, (byte)136, (byte)165, (byte)61, (byte)100, (byte)198, (byte)109, (byte)171, (byte)237, (byte)111, (byte)142, (byte)163, (byte)125, (byte)204, (byte)136, (byte)201, (byte)94, (byte)166, (byte)72, (byte)157, (byte)98, (byte)229, (byte)147, (byte)122, (byte)235, (byte)204, (byte)216, (byte)255, (byte)228, (byte)144, (byte)193, (byte)165, (byte)161, (byte)105, (byte)80, (byte)127, (byte)186, (byte)14, (byte)75, (byte)173, (byte)129, (byte)220, (byte)177, (byte)220, (byte)131, (byte)119, (byte)243, (byte)82, (byte)86, (byte)101, (byte)197, (byte)242, (byte)56, (byte)44, (byte)119, (byte)175, (byte)129, (byte)123, (byte)73, (byte)204, (byte)241, (byte)100, (byte)111, (byte)48, (byte)131, (byte)101, (byte)109, (byte)113, (byte)156, (byte)207, (byte)128, (byte)194, (byte)226, (byte)195, (byte)37, (byte)230, (byte)188, (byte)40, (byte)143, (byte)182, (byte)28, (byte)72, (byte)134, (byte)131, (byte)184, (byte)84, (byte)241, (byte)75, (byte)189, (byte)60, (byte)101, (byte)82, (byte)86, (byte)146, (byte)253, (byte)166, (byte)183, (byte)251, (byte)219, (byte)55, (byte)72, (byte)20, (byte)128, (byte)122, (byte)118, (byte)177, (byte)165, (byte)110, (byte)251, (byte)236, (byte)12, (byte)61, (byte)238, (byte)156, (byte)112, (byte)96, (byte)248, (byte)102, (byte)103, (byte)195, (byte)16, (byte)116, (byte)112, (byte)220, (byte)0, (byte)137, (byte)222, (byte)118, (byte)202, (byte)176, (byte)9, (byte)73, (byte)91, (byte)35, (byte)105, (byte)229, (byte)227, (byte)182, (byte)122, (byte)40, (byte)156, (byte)192, (byte)188, (byte)196, (byte)17, (byte)13, (byte)164, (byte)96, (byte)88, (byte)86, (byte)2, (byte)158, (byte)251, (byte)12, (byte)132, (byte)27, (byte)60, (byte)183, (byte)39, (byte)105, (byte)55, (byte)245, (byte)242, (byte)31, (byte)13, (byte)199, (byte)33, (byte)235, (byte)79, (byte)36, (byte)26, (byte)187, (byte)142, (byte)251, (byte)201, (byte)26, (byte)105, (byte)209, (byte)190, (byte)170, (byte)48, (byte)191, (byte)184, (byte)173, (byte)53, (byte)129, (byte)119, (byte)216, (byte)116, (byte)107, (byte)197, (byte)150, (byte)138, (byte)119, (byte)39, (byte)45, (byte)10, (byte)113, (byte)136, (byte)89, (byte)139, (byte)42, (byte)223, (byte)193, (byte)208, (byte)80, (byte)134, (byte)114, (byte)96, (byte)33, (byte)163, (byte)105, (byte)254, (byte)146, (byte)210, (byte)234, (byte)190, (byte)70, (byte)126, (byte)100, (byte)213, (byte)203, (byte)24, (byte)58, (byte)135, (byte)24, (byte)40, (byte)68, (byte)248, (byte)17, (byte)98, (byte)246}));
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)182;
            p110.payload_SET(new byte[] {(byte)129, (byte)115, (byte)221, (byte)193, (byte)46, (byte)54, (byte)141, (byte)112, (byte)178, (byte)71, (byte)228, (byte)75, (byte)58, (byte)44, (byte)170, (byte)95, (byte)123, (byte)117, (byte)197, (byte)161, (byte)136, (byte)165, (byte)61, (byte)100, (byte)198, (byte)109, (byte)171, (byte)237, (byte)111, (byte)142, (byte)163, (byte)125, (byte)204, (byte)136, (byte)201, (byte)94, (byte)166, (byte)72, (byte)157, (byte)98, (byte)229, (byte)147, (byte)122, (byte)235, (byte)204, (byte)216, (byte)255, (byte)228, (byte)144, (byte)193, (byte)165, (byte)161, (byte)105, (byte)80, (byte)127, (byte)186, (byte)14, (byte)75, (byte)173, (byte)129, (byte)220, (byte)177, (byte)220, (byte)131, (byte)119, (byte)243, (byte)82, (byte)86, (byte)101, (byte)197, (byte)242, (byte)56, (byte)44, (byte)119, (byte)175, (byte)129, (byte)123, (byte)73, (byte)204, (byte)241, (byte)100, (byte)111, (byte)48, (byte)131, (byte)101, (byte)109, (byte)113, (byte)156, (byte)207, (byte)128, (byte)194, (byte)226, (byte)195, (byte)37, (byte)230, (byte)188, (byte)40, (byte)143, (byte)182, (byte)28, (byte)72, (byte)134, (byte)131, (byte)184, (byte)84, (byte)241, (byte)75, (byte)189, (byte)60, (byte)101, (byte)82, (byte)86, (byte)146, (byte)253, (byte)166, (byte)183, (byte)251, (byte)219, (byte)55, (byte)72, (byte)20, (byte)128, (byte)122, (byte)118, (byte)177, (byte)165, (byte)110, (byte)251, (byte)236, (byte)12, (byte)61, (byte)238, (byte)156, (byte)112, (byte)96, (byte)248, (byte)102, (byte)103, (byte)195, (byte)16, (byte)116, (byte)112, (byte)220, (byte)0, (byte)137, (byte)222, (byte)118, (byte)202, (byte)176, (byte)9, (byte)73, (byte)91, (byte)35, (byte)105, (byte)229, (byte)227, (byte)182, (byte)122, (byte)40, (byte)156, (byte)192, (byte)188, (byte)196, (byte)17, (byte)13, (byte)164, (byte)96, (byte)88, (byte)86, (byte)2, (byte)158, (byte)251, (byte)12, (byte)132, (byte)27, (byte)60, (byte)183, (byte)39, (byte)105, (byte)55, (byte)245, (byte)242, (byte)31, (byte)13, (byte)199, (byte)33, (byte)235, (byte)79, (byte)36, (byte)26, (byte)187, (byte)142, (byte)251, (byte)201, (byte)26, (byte)105, (byte)209, (byte)190, (byte)170, (byte)48, (byte)191, (byte)184, (byte)173, (byte)53, (byte)129, (byte)119, (byte)216, (byte)116, (byte)107, (byte)197, (byte)150, (byte)138, (byte)119, (byte)39, (byte)45, (byte)10, (byte)113, (byte)136, (byte)89, (byte)139, (byte)42, (byte)223, (byte)193, (byte)208, (byte)80, (byte)134, (byte)114, (byte)96, (byte)33, (byte)163, (byte)105, (byte)254, (byte)146, (byte)210, (byte)234, (byte)190, (byte)70, (byte)126, (byte)100, (byte)213, (byte)203, (byte)24, (byte)58, (byte)135, (byte)24, (byte)40, (byte)68, (byte)248, (byte)17, (byte)98, (byte)246}, 0) ;
            p110.target_component = (byte)(byte)214;
            p110.target_system = (byte)(byte)61;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long)9115888368743692344L);
                Debug.Assert(pack.tc1 == (long)9083006618193519414L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long)9115888368743692344L;
            p111.tc1 = (long)9083006618193519414L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)1612331633U);
                Debug.Assert(pack.time_usec == (ulong)7163584390224579385L);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)1612331633U;
            p112.time_usec = (ulong)7163584390224579385L;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cog == (ushort)(ushort)11598);
                Debug.Assert(pack.time_usec == (ulong)2686460328473179660L);
                Debug.Assert(pack.alt == (int) -519844365);
                Debug.Assert(pack.epv == (ushort)(ushort)10125);
                Debug.Assert(pack.lon == (int) -1878383258);
                Debug.Assert(pack.vn == (short)(short) -15074);
                Debug.Assert(pack.satellites_visible == (byte)(byte)175);
                Debug.Assert(pack.eph == (ushort)(ushort)58839);
                Debug.Assert(pack.lat == (int)891450964);
                Debug.Assert(pack.fix_type == (byte)(byte)137);
                Debug.Assert(pack.ve == (short)(short) -19776);
                Debug.Assert(pack.vel == (ushort)(ushort)60810);
                Debug.Assert(pack.vd == (short)(short)31950);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.vel = (ushort)(ushort)60810;
            p113.lat = (int)891450964;
            p113.ve = (short)(short) -19776;
            p113.alt = (int) -519844365;
            p113.cog = (ushort)(ushort)11598;
            p113.fix_type = (byte)(byte)137;
            p113.vd = (short)(short)31950;
            p113.time_usec = (ulong)2686460328473179660L;
            p113.eph = (ushort)(ushort)58839;
            p113.vn = (short)(short) -15074;
            p113.epv = (ushort)(ushort)10125;
            p113.satellites_visible = (byte)(byte)175;
            p113.lon = (int) -1878383258;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_xgyro == (float)6.9993703E37F);
                Debug.Assert(pack.time_usec == (ulong)390623326865072670L);
                Debug.Assert(pack.temperature == (short)(short)30984);
                Debug.Assert(pack.integrated_x == (float)3.1077335E38F);
                Debug.Assert(pack.integrated_zgyro == (float)2.6234118E38F);
                Debug.Assert(pack.quality == (byte)(byte)255);
                Debug.Assert(pack.distance == (float) -2.543934E38F);
                Debug.Assert(pack.integration_time_us == (uint)1906743328U);
                Debug.Assert(pack.integrated_ygyro == (float) -1.6639179E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)94);
                Debug.Assert(pack.time_delta_distance_us == (uint)3079679476U);
                Debug.Assert(pack.integrated_y == (float) -3.0286002E38F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integrated_y = (float) -3.0286002E38F;
            p114.integrated_xgyro = (float)6.9993703E37F;
            p114.temperature = (short)(short)30984;
            p114.distance = (float) -2.543934E38F;
            p114.quality = (byte)(byte)255;
            p114.integration_time_us = (uint)1906743328U;
            p114.integrated_zgyro = (float)2.6234118E38F;
            p114.time_delta_distance_us = (uint)3079679476U;
            p114.time_usec = (ulong)390623326865072670L;
            p114.integrated_x = (float)3.1077335E38F;
            p114.sensor_id = (byte)(byte)94;
            p114.integrated_ygyro = (float) -1.6639179E38F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int) -1803733227);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)42636);
                Debug.Assert(pack.zacc == (short)(short) -11631);
                Debug.Assert(pack.rollspeed == (float)7.708553E37F);
                Debug.Assert(pack.lon == (int) -476016011);
                Debug.Assert(pack.yawspeed == (float)1.4469465E37F);
                Debug.Assert(pack.vz == (short)(short) -22294);
                Debug.Assert(pack.lat == (int)174695777);
                Debug.Assert(pack.time_usec == (ulong)2175477130701592795L);
                Debug.Assert(pack.pitchspeed == (float) -9.141599E36F);
                Debug.Assert(pack.yacc == (short)(short) -26688);
                Debug.Assert(pack.vx == (short)(short) -19185);
                Debug.Assert(pack.xacc == (short)(short) -1667);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-8.431313E37F, -2.8547633E38F, -9.8306456E36F, -1.4112477E38F}));
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)39408);
                Debug.Assert(pack.vy == (short)(short) -3081);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.ind_airspeed = (ushort)(ushort)42636;
            p115.xacc = (short)(short) -1667;
            p115.true_airspeed = (ushort)(ushort)39408;
            p115.pitchspeed = (float) -9.141599E36F;
            p115.zacc = (short)(short) -11631;
            p115.alt = (int) -1803733227;
            p115.vy = (short)(short) -3081;
            p115.vx = (short)(short) -19185;
            p115.lon = (int) -476016011;
            p115.attitude_quaternion_SET(new float[] {-8.431313E37F, -2.8547633E38F, -9.8306456E36F, -1.4112477E38F}, 0) ;
            p115.yacc = (short)(short) -26688;
            p115.vz = (short)(short) -22294;
            p115.lat = (int)174695777;
            p115.time_usec = (ulong)2175477130701592795L;
            p115.rollspeed = (float)7.708553E37F;
            p115.yawspeed = (float)1.4469465E37F;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2631476911U);
                Debug.Assert(pack.xacc == (short)(short) -32309);
                Debug.Assert(pack.yacc == (short)(short) -14415);
                Debug.Assert(pack.ymag == (short)(short)11517);
                Debug.Assert(pack.zgyro == (short)(short) -20501);
                Debug.Assert(pack.zacc == (short)(short) -10599);
                Debug.Assert(pack.zmag == (short)(short) -30821);
                Debug.Assert(pack.xmag == (short)(short) -30581);
                Debug.Assert(pack.xgyro == (short)(short)4914);
                Debug.Assert(pack.ygyro == (short)(short) -18748);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)2631476911U;
            p116.ygyro = (short)(short) -18748;
            p116.xacc = (short)(short) -32309;
            p116.zgyro = (short)(short) -20501;
            p116.zmag = (short)(short) -30821;
            p116.xmag = (short)(short) -30581;
            p116.ymag = (short)(short)11517;
            p116.xgyro = (short)(short)4914;
            p116.yacc = (short)(short) -14415;
            p116.zacc = (short)(short) -10599;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)228);
                Debug.Assert(pack.target_system == (byte)(byte)57);
                Debug.Assert(pack.start == (ushort)(ushort)16978);
                Debug.Assert(pack.end == (ushort)(ushort)10994);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.start = (ushort)(ushort)16978;
            p117.end = (ushort)(ushort)10994;
            p117.target_component = (byte)(byte)228;
            p117.target_system = (byte)(byte)57;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.size == (uint)4156398480U);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)51177);
                Debug.Assert(pack.id == (ushort)(ushort)45347);
                Debug.Assert(pack.num_logs == (ushort)(ushort)41313);
                Debug.Assert(pack.time_utc == (uint)783981061U);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.time_utc = (uint)783981061U;
            p118.size = (uint)4156398480U;
            p118.id = (ushort)(ushort)45347;
            p118.last_log_num = (ushort)(ushort)51177;
            p118.num_logs = (ushort)(ushort)41313;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)201);
                Debug.Assert(pack.target_system == (byte)(byte)208);
                Debug.Assert(pack.id == (ushort)(ushort)60331);
                Debug.Assert(pack.ofs == (uint)4029839663U);
                Debug.Assert(pack.count == (uint)405288522U);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.count = (uint)405288522U;
            p119.ofs = (uint)4029839663U;
            p119.target_system = (byte)(byte)208;
            p119.id = (ushort)(ushort)60331;
            p119.target_component = (byte)(byte)201;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (byte)(byte)13);
                Debug.Assert(pack.id == (ushort)(ushort)52911);
                Debug.Assert(pack.ofs == (uint)1291897954U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)178, (byte)122, (byte)194, (byte)120, (byte)100, (byte)237, (byte)145, (byte)198, (byte)32, (byte)124, (byte)172, (byte)113, (byte)32, (byte)26, (byte)78, (byte)146, (byte)40, (byte)248, (byte)247, (byte)47, (byte)194, (byte)52, (byte)125, (byte)173, (byte)113, (byte)185, (byte)109, (byte)84, (byte)72, (byte)34, (byte)82, (byte)122, (byte)116, (byte)250, (byte)78, (byte)157, (byte)99, (byte)101, (byte)177, (byte)181, (byte)109, (byte)85, (byte)72, (byte)18, (byte)180, (byte)31, (byte)238, (byte)129, (byte)10, (byte)92, (byte)78, (byte)113, (byte)49, (byte)108, (byte)134, (byte)100, (byte)70, (byte)15, (byte)11, (byte)184, (byte)81, (byte)122, (byte)189, (byte)164, (byte)123, (byte)189, (byte)116, (byte)222, (byte)217, (byte)195, (byte)105, (byte)202, (byte)156, (byte)120, (byte)81, (byte)134, (byte)243, (byte)231, (byte)174, (byte)107, (byte)79, (byte)199, (byte)113, (byte)51, (byte)90, (byte)42, (byte)35, (byte)183, (byte)148, (byte)30}));
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.count = (byte)(byte)13;
            p120.id = (ushort)(ushort)52911;
            p120.ofs = (uint)1291897954U;
            p120.data__SET(new byte[] {(byte)178, (byte)122, (byte)194, (byte)120, (byte)100, (byte)237, (byte)145, (byte)198, (byte)32, (byte)124, (byte)172, (byte)113, (byte)32, (byte)26, (byte)78, (byte)146, (byte)40, (byte)248, (byte)247, (byte)47, (byte)194, (byte)52, (byte)125, (byte)173, (byte)113, (byte)185, (byte)109, (byte)84, (byte)72, (byte)34, (byte)82, (byte)122, (byte)116, (byte)250, (byte)78, (byte)157, (byte)99, (byte)101, (byte)177, (byte)181, (byte)109, (byte)85, (byte)72, (byte)18, (byte)180, (byte)31, (byte)238, (byte)129, (byte)10, (byte)92, (byte)78, (byte)113, (byte)49, (byte)108, (byte)134, (byte)100, (byte)70, (byte)15, (byte)11, (byte)184, (byte)81, (byte)122, (byte)189, (byte)164, (byte)123, (byte)189, (byte)116, (byte)222, (byte)217, (byte)195, (byte)105, (byte)202, (byte)156, (byte)120, (byte)81, (byte)134, (byte)243, (byte)231, (byte)174, (byte)107, (byte)79, (byte)199, (byte)113, (byte)51, (byte)90, (byte)42, (byte)35, (byte)183, (byte)148, (byte)30}, 0) ;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)237);
                Debug.Assert(pack.target_system == (byte)(byte)160);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)160;
            p121.target_component = (byte)(byte)237;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)15);
                Debug.Assert(pack.target_component == (byte)(byte)228);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)15;
            p122.target_component = (byte)(byte)228;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)87);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)103, (byte)37, (byte)198, (byte)166, (byte)100, (byte)53, (byte)23, (byte)86, (byte)200, (byte)135, (byte)105, (byte)5, (byte)67, (byte)162, (byte)118, (byte)104, (byte)159, (byte)126, (byte)3, (byte)58, (byte)185, (byte)123, (byte)6, (byte)112, (byte)131, (byte)183, (byte)241, (byte)173, (byte)199, (byte)42, (byte)15, (byte)39, (byte)214, (byte)162, (byte)71, (byte)210, (byte)211, (byte)113, (byte)75, (byte)72, (byte)167, (byte)172, (byte)213, (byte)77, (byte)151, (byte)69, (byte)69, (byte)243, (byte)183, (byte)249, (byte)110, (byte)230, (byte)40, (byte)53, (byte)230, (byte)51, (byte)77, (byte)0, (byte)215, (byte)250, (byte)41, (byte)67, (byte)1, (byte)152, (byte)142, (byte)113, (byte)235, (byte)243, (byte)100, (byte)158, (byte)46, (byte)8, (byte)34, (byte)230, (byte)157, (byte)119, (byte)108, (byte)80, (byte)27, (byte)112, (byte)176, (byte)82, (byte)180, (byte)50, (byte)150, (byte)71, (byte)192, (byte)145, (byte)43, (byte)139, (byte)130, (byte)222, (byte)24, (byte)153, (byte)216, (byte)24, (byte)29, (byte)113, (byte)181, (byte)25, (byte)48, (byte)193, (byte)150, (byte)21, (byte)193, (byte)146, (byte)114, (byte)162, (byte)110, (byte)89}));
                Debug.Assert(pack.len == (byte)(byte)77);
                Debug.Assert(pack.target_system == (byte)(byte)95);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)95;
            p123.len = (byte)(byte)77;
            p123.data__SET(new byte[] {(byte)103, (byte)37, (byte)198, (byte)166, (byte)100, (byte)53, (byte)23, (byte)86, (byte)200, (byte)135, (byte)105, (byte)5, (byte)67, (byte)162, (byte)118, (byte)104, (byte)159, (byte)126, (byte)3, (byte)58, (byte)185, (byte)123, (byte)6, (byte)112, (byte)131, (byte)183, (byte)241, (byte)173, (byte)199, (byte)42, (byte)15, (byte)39, (byte)214, (byte)162, (byte)71, (byte)210, (byte)211, (byte)113, (byte)75, (byte)72, (byte)167, (byte)172, (byte)213, (byte)77, (byte)151, (byte)69, (byte)69, (byte)243, (byte)183, (byte)249, (byte)110, (byte)230, (byte)40, (byte)53, (byte)230, (byte)51, (byte)77, (byte)0, (byte)215, (byte)250, (byte)41, (byte)67, (byte)1, (byte)152, (byte)142, (byte)113, (byte)235, (byte)243, (byte)100, (byte)158, (byte)46, (byte)8, (byte)34, (byte)230, (byte)157, (byte)119, (byte)108, (byte)80, (byte)27, (byte)112, (byte)176, (byte)82, (byte)180, (byte)50, (byte)150, (byte)71, (byte)192, (byte)145, (byte)43, (byte)139, (byte)130, (byte)222, (byte)24, (byte)153, (byte)216, (byte)24, (byte)29, (byte)113, (byte)181, (byte)25, (byte)48, (byte)193, (byte)150, (byte)21, (byte)193, (byte)146, (byte)114, (byte)162, (byte)110, (byte)89}, 0) ;
            p123.target_component = (byte)(byte)87;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.dgps_numch == (byte)(byte)96);
                Debug.Assert(pack.dgps_age == (uint)512818141U);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
                Debug.Assert(pack.epv == (ushort)(ushort)27019);
                Debug.Assert(pack.vel == (ushort)(ushort)31607);
                Debug.Assert(pack.eph == (ushort)(ushort)32319);
                Debug.Assert(pack.lon == (int) -196620221);
                Debug.Assert(pack.time_usec == (ulong)2554332017725283243L);
                Debug.Assert(pack.cog == (ushort)(ushort)12131);
                Debug.Assert(pack.lat == (int)847243854);
                Debug.Assert(pack.satellites_visible == (byte)(byte)253);
                Debug.Assert(pack.alt == (int)67825769);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)2554332017725283243L;
            p124.cog = (ushort)(ushort)12131;
            p124.epv = (ushort)(ushort)27019;
            p124.vel = (ushort)(ushort)31607;
            p124.satellites_visible = (byte)(byte)253;
            p124.lon = (int) -196620221;
            p124.alt = (int)67825769;
            p124.eph = (ushort)(ushort)32319;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT;
            p124.dgps_numch = (byte)(byte)96;
            p124.lat = (int)847243854;
            p124.dgps_age = (uint)512818141U;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)6530);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
                Debug.Assert(pack.Vservo == (ushort)(ushort)33884);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)6530;
            p125.Vservo = (ushort)(ushort)33884;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
                Debug.Assert(pack.count == (byte)(byte)255);
                Debug.Assert(pack.timeout == (ushort)(ushort)37635);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)50, (byte)119, (byte)251, (byte)242, (byte)29, (byte)93, (byte)166, (byte)172, (byte)17, (byte)112, (byte)73, (byte)34, (byte)184, (byte)16, (byte)222, (byte)4, (byte)65, (byte)72, (byte)212, (byte)45, (byte)164, (byte)22, (byte)128, (byte)175, (byte)62, (byte)26, (byte)149, (byte)138, (byte)98, (byte)133, (byte)203, (byte)218, (byte)23, (byte)255, (byte)122, (byte)181, (byte)103, (byte)235, (byte)31, (byte)243, (byte)16, (byte)61, (byte)166, (byte)215, (byte)13, (byte)167, (byte)64, (byte)117, (byte)120, (byte)240, (byte)249, (byte)88, (byte)193, (byte)123, (byte)144, (byte)173, (byte)128, (byte)141, (byte)19, (byte)153, (byte)9, (byte)67, (byte)110, (byte)233, (byte)163, (byte)206, (byte)196, (byte)52, (byte)171, (byte)143}));
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI);
                Debug.Assert(pack.baudrate == (uint)2837246062U);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.baudrate = (uint)2837246062U;
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL;
            p126.count = (byte)(byte)255;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI;
            p126.data__SET(new byte[] {(byte)50, (byte)119, (byte)251, (byte)242, (byte)29, (byte)93, (byte)166, (byte)172, (byte)17, (byte)112, (byte)73, (byte)34, (byte)184, (byte)16, (byte)222, (byte)4, (byte)65, (byte)72, (byte)212, (byte)45, (byte)164, (byte)22, (byte)128, (byte)175, (byte)62, (byte)26, (byte)149, (byte)138, (byte)98, (byte)133, (byte)203, (byte)218, (byte)23, (byte)255, (byte)122, (byte)181, (byte)103, (byte)235, (byte)31, (byte)243, (byte)16, (byte)61, (byte)166, (byte)215, (byte)13, (byte)167, (byte)64, (byte)117, (byte)120, (byte)240, (byte)249, (byte)88, (byte)193, (byte)123, (byte)144, (byte)173, (byte)128, (byte)141, (byte)19, (byte)153, (byte)9, (byte)67, (byte)110, (byte)233, (byte)163, (byte)206, (byte)196, (byte)52, (byte)171, (byte)143}, 0) ;
            p126.timeout = (ushort)(ushort)37635;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_b_mm == (int)693592735);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)206);
                Debug.Assert(pack.time_last_baseline_ms == (uint)637957315U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)0);
                Debug.Assert(pack.accuracy == (uint)4244202013U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)101);
                Debug.Assert(pack.tow == (uint)4189778453U);
                Debug.Assert(pack.rtk_health == (byte)(byte)9);
                Debug.Assert(pack.wn == (ushort)(ushort)42566);
                Debug.Assert(pack.nsats == (byte)(byte)0);
                Debug.Assert(pack.baseline_c_mm == (int)9652535);
                Debug.Assert(pack.baseline_a_mm == (int) -2024202641);
                Debug.Assert(pack.iar_num_hypotheses == (int) -1439142658);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.baseline_a_mm = (int) -2024202641;
            p127.baseline_c_mm = (int)9652535;
            p127.tow = (uint)4189778453U;
            p127.rtk_rate = (byte)(byte)101;
            p127.rtk_receiver_id = (byte)(byte)0;
            p127.time_last_baseline_ms = (uint)637957315U;
            p127.nsats = (byte)(byte)0;
            p127.baseline_b_mm = (int)693592735;
            p127.baseline_coords_type = (byte)(byte)206;
            p127.iar_num_hypotheses = (int) -1439142658;
            p127.rtk_health = (byte)(byte)9;
            p127.wn = (ushort)(ushort)42566;
            p127.accuracy = (uint)4244202013U;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)234);
                Debug.Assert(pack.wn == (ushort)(ushort)34828);
                Debug.Assert(pack.rtk_health == (byte)(byte)148);
                Debug.Assert(pack.rtk_rate == (byte)(byte)206);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)227);
                Debug.Assert(pack.tow == (uint)1281432477U);
                Debug.Assert(pack.time_last_baseline_ms == (uint)3271608152U);
                Debug.Assert(pack.iar_num_hypotheses == (int)2118525509);
                Debug.Assert(pack.baseline_a_mm == (int) -1376871490);
                Debug.Assert(pack.baseline_c_mm == (int)781725587);
                Debug.Assert(pack.nsats == (byte)(byte)200);
                Debug.Assert(pack.baseline_b_mm == (int)699318497);
                Debug.Assert(pack.accuracy == (uint)741861607U);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.baseline_c_mm = (int)781725587;
            p128.iar_num_hypotheses = (int)2118525509;
            p128.nsats = (byte)(byte)200;
            p128.rtk_rate = (byte)(byte)206;
            p128.time_last_baseline_ms = (uint)3271608152U;
            p128.baseline_a_mm = (int) -1376871490;
            p128.baseline_b_mm = (int)699318497;
            p128.rtk_health = (byte)(byte)148;
            p128.tow = (uint)1281432477U;
            p128.baseline_coords_type = (byte)(byte)234;
            p128.rtk_receiver_id = (byte)(byte)227;
            p128.wn = (ushort)(ushort)34828;
            p128.accuracy = (uint)741861607U;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short)4622);
                Debug.Assert(pack.zgyro == (short)(short)26663);
                Debug.Assert(pack.xgyro == (short)(short)861);
                Debug.Assert(pack.zmag == (short)(short)3580);
                Debug.Assert(pack.xacc == (short)(short) -18030);
                Debug.Assert(pack.time_boot_ms == (uint)1490548918U);
                Debug.Assert(pack.zacc == (short)(short)22881);
                Debug.Assert(pack.xmag == (short)(short) -5511);
                Debug.Assert(pack.yacc == (short)(short)13094);
                Debug.Assert(pack.ymag == (short)(short)32156);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.zgyro = (short)(short)26663;
            p129.ygyro = (short)(short)4622;
            p129.ymag = (short)(short)32156;
            p129.xacc = (short)(short) -18030;
            p129.xmag = (short)(short) -5511;
            p129.yacc = (short)(short)13094;
            p129.time_boot_ms = (uint)1490548918U;
            p129.zmag = (short)(short)3580;
            p129.xgyro = (short)(short)861;
            p129.zacc = (short)(short)22881;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.jpg_quality == (byte)(byte)73);
                Debug.Assert(pack.type == (byte)(byte)56);
                Debug.Assert(pack.size == (uint)3019332228U);
                Debug.Assert(pack.width == (ushort)(ushort)46780);
                Debug.Assert(pack.packets == (ushort)(ushort)28902);
                Debug.Assert(pack.payload == (byte)(byte)23);
                Debug.Assert(pack.height == (ushort)(ushort)10091);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.packets = (ushort)(ushort)28902;
            p130.payload = (byte)(byte)23;
            p130.width = (ushort)(ushort)46780;
            p130.height = (ushort)(ushort)10091;
            p130.type = (byte)(byte)56;
            p130.size = (uint)3019332228U;
            p130.jpg_quality = (byte)(byte)73;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seqnr == (ushort)(ushort)359);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)193, (byte)43, (byte)37, (byte)223, (byte)19, (byte)210, (byte)172, (byte)246, (byte)31, (byte)169, (byte)174, (byte)130, (byte)81, (byte)177, (byte)15, (byte)134, (byte)88, (byte)229, (byte)235, (byte)78, (byte)213, (byte)124, (byte)4, (byte)127, (byte)128, (byte)38, (byte)11, (byte)238, (byte)15, (byte)247, (byte)114, (byte)155, (byte)199, (byte)130, (byte)181, (byte)186, (byte)87, (byte)106, (byte)41, (byte)177, (byte)127, (byte)70, (byte)10, (byte)149, (byte)222, (byte)30, (byte)66, (byte)251, (byte)243, (byte)152, (byte)83, (byte)69, (byte)155, (byte)212, (byte)200, (byte)79, (byte)230, (byte)14, (byte)50, (byte)151, (byte)103, (byte)6, (byte)27, (byte)38, (byte)6, (byte)182, (byte)236, (byte)5, (byte)215, (byte)228, (byte)73, (byte)169, (byte)4, (byte)232, (byte)212, (byte)194, (byte)73, (byte)79, (byte)126, (byte)169, (byte)152, (byte)232, (byte)214, (byte)88, (byte)205, (byte)56, (byte)36, (byte)12, (byte)97, (byte)150, (byte)218, (byte)188, (byte)168, (byte)247, (byte)52, (byte)77, (byte)76, (byte)84, (byte)96, (byte)57, (byte)126, (byte)222, (byte)112, (byte)79, (byte)255, (byte)17, (byte)213, (byte)159, (byte)144, (byte)208, (byte)44, (byte)7, (byte)228, (byte)108, (byte)158, (byte)203, (byte)56, (byte)56, (byte)149, (byte)230, (byte)176, (byte)185, (byte)210, (byte)44, (byte)79, (byte)71, (byte)220, (byte)132, (byte)174, (byte)116, (byte)190, (byte)165, (byte)70, (byte)170, (byte)60, (byte)255, (byte)79, (byte)35, (byte)16, (byte)255, (byte)156, (byte)191, (byte)190, (byte)249, (byte)48, (byte)2, (byte)251, (byte)10, (byte)189, (byte)191, (byte)45, (byte)54, (byte)148, (byte)63, (byte)187, (byte)203, (byte)25, (byte)190, (byte)168, (byte)155, (byte)78, (byte)90, (byte)97, (byte)246, (byte)203, (byte)175, (byte)134, (byte)105, (byte)189, (byte)94, (byte)145, (byte)135, (byte)141, (byte)132, (byte)220, (byte)255, (byte)62, (byte)199, (byte)80, (byte)113, (byte)102, (byte)78, (byte)218, (byte)206, (byte)6, (byte)99, (byte)141, (byte)114, (byte)82, (byte)36, (byte)192, (byte)219, (byte)13, (byte)213, (byte)168, (byte)102, (byte)114, (byte)204, (byte)35, (byte)217, (byte)160, (byte)122, (byte)105, (byte)205, (byte)189, (byte)245, (byte)91, (byte)232, (byte)47, (byte)56, (byte)216, (byte)142, (byte)58, (byte)177, (byte)74, (byte)213, (byte)204, (byte)163, (byte)126, (byte)163, (byte)229, (byte)72, (byte)186, (byte)165, (byte)99, (byte)50, (byte)168, (byte)126, (byte)178, (byte)67, (byte)245, (byte)26, (byte)36, (byte)240, (byte)30, (byte)62, (byte)184, (byte)119, (byte)76, (byte)220, (byte)100, (byte)245, (byte)154, (byte)55, (byte)64, (byte)84, (byte)250, (byte)134, (byte)74, (byte)226, (byte)79, (byte)22, (byte)207}));
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)359;
            p131.data__SET(new byte[] {(byte)193, (byte)43, (byte)37, (byte)223, (byte)19, (byte)210, (byte)172, (byte)246, (byte)31, (byte)169, (byte)174, (byte)130, (byte)81, (byte)177, (byte)15, (byte)134, (byte)88, (byte)229, (byte)235, (byte)78, (byte)213, (byte)124, (byte)4, (byte)127, (byte)128, (byte)38, (byte)11, (byte)238, (byte)15, (byte)247, (byte)114, (byte)155, (byte)199, (byte)130, (byte)181, (byte)186, (byte)87, (byte)106, (byte)41, (byte)177, (byte)127, (byte)70, (byte)10, (byte)149, (byte)222, (byte)30, (byte)66, (byte)251, (byte)243, (byte)152, (byte)83, (byte)69, (byte)155, (byte)212, (byte)200, (byte)79, (byte)230, (byte)14, (byte)50, (byte)151, (byte)103, (byte)6, (byte)27, (byte)38, (byte)6, (byte)182, (byte)236, (byte)5, (byte)215, (byte)228, (byte)73, (byte)169, (byte)4, (byte)232, (byte)212, (byte)194, (byte)73, (byte)79, (byte)126, (byte)169, (byte)152, (byte)232, (byte)214, (byte)88, (byte)205, (byte)56, (byte)36, (byte)12, (byte)97, (byte)150, (byte)218, (byte)188, (byte)168, (byte)247, (byte)52, (byte)77, (byte)76, (byte)84, (byte)96, (byte)57, (byte)126, (byte)222, (byte)112, (byte)79, (byte)255, (byte)17, (byte)213, (byte)159, (byte)144, (byte)208, (byte)44, (byte)7, (byte)228, (byte)108, (byte)158, (byte)203, (byte)56, (byte)56, (byte)149, (byte)230, (byte)176, (byte)185, (byte)210, (byte)44, (byte)79, (byte)71, (byte)220, (byte)132, (byte)174, (byte)116, (byte)190, (byte)165, (byte)70, (byte)170, (byte)60, (byte)255, (byte)79, (byte)35, (byte)16, (byte)255, (byte)156, (byte)191, (byte)190, (byte)249, (byte)48, (byte)2, (byte)251, (byte)10, (byte)189, (byte)191, (byte)45, (byte)54, (byte)148, (byte)63, (byte)187, (byte)203, (byte)25, (byte)190, (byte)168, (byte)155, (byte)78, (byte)90, (byte)97, (byte)246, (byte)203, (byte)175, (byte)134, (byte)105, (byte)189, (byte)94, (byte)145, (byte)135, (byte)141, (byte)132, (byte)220, (byte)255, (byte)62, (byte)199, (byte)80, (byte)113, (byte)102, (byte)78, (byte)218, (byte)206, (byte)6, (byte)99, (byte)141, (byte)114, (byte)82, (byte)36, (byte)192, (byte)219, (byte)13, (byte)213, (byte)168, (byte)102, (byte)114, (byte)204, (byte)35, (byte)217, (byte)160, (byte)122, (byte)105, (byte)205, (byte)189, (byte)245, (byte)91, (byte)232, (byte)47, (byte)56, (byte)216, (byte)142, (byte)58, (byte)177, (byte)74, (byte)213, (byte)204, (byte)163, (byte)126, (byte)163, (byte)229, (byte)72, (byte)186, (byte)165, (byte)99, (byte)50, (byte)168, (byte)126, (byte)178, (byte)67, (byte)245, (byte)26, (byte)36, (byte)240, (byte)30, (byte)62, (byte)184, (byte)119, (byte)76, (byte)220, (byte)100, (byte)245, (byte)154, (byte)55, (byte)64, (byte)84, (byte)250, (byte)134, (byte)74, (byte)226, (byte)79, (byte)22, (byte)207}, 0) ;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
                Debug.Assert(pack.min_distance == (ushort)(ushort)4914);
                Debug.Assert(pack.time_boot_ms == (uint)968990071U);
                Debug.Assert(pack.covariance == (byte)(byte)111);
                Debug.Assert(pack.id == (byte)(byte)46);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_135);
                Debug.Assert(pack.current_distance == (ushort)(ushort)14911);
                Debug.Assert(pack.max_distance == (ushort)(ushort)33629);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)968990071U;
            p132.min_distance = (ushort)(ushort)4914;
            p132.covariance = (byte)(byte)111;
            p132.max_distance = (ushort)(ushort)33629;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_135;
            p132.id = (byte)(byte)46;
            p132.current_distance = (ushort)(ushort)14911;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -592572447);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)5715);
                Debug.Assert(pack.lat == (int) -519532592);
                Debug.Assert(pack.mask == (ulong)7386661522870035856L);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.mask = (ulong)7386661522870035856L;
            p133.grid_spacing = (ushort)(ushort)5715;
            p133.lon = (int) -592572447;
            p133.lat = (int) -519532592;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gridbit == (byte)(byte)36);
                Debug.Assert(pack.lat == (int)1567345534);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -20392, (short)5853, (short)11264, (short)16986, (short)3631, (short)11636, (short)1982, (short)15176, (short) -31839, (short)16105, (short) -13665, (short)6716, (short)1666, (short) -31874, (short) -6549, (short) -23583}));
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)60638);
                Debug.Assert(pack.lon == (int)386971837);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lon = (int)386971837;
            p134.data__SET(new short[] {(short) -20392, (short)5853, (short)11264, (short)16986, (short)3631, (short)11636, (short)1982, (short)15176, (short) -31839, (short)16105, (short) -13665, (short)6716, (short)1666, (short) -31874, (short) -6549, (short) -23583}, 0) ;
            p134.gridbit = (byte)(byte)36;
            p134.lat = (int)1567345534;
            p134.grid_spacing = (ushort)(ushort)60638;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)659683821);
                Debug.Assert(pack.lat == (int) -1632224033);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -1632224033;
            p135.lon = (int)659683821;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.spacing == (ushort)(ushort)5542);
                Debug.Assert(pack.terrain_height == (float)1.7201345E38F);
                Debug.Assert(pack.lat == (int) -1304429152);
                Debug.Assert(pack.pending == (ushort)(ushort)64027);
                Debug.Assert(pack.lon == (int) -1621023319);
                Debug.Assert(pack.loaded == (ushort)(ushort)29554);
                Debug.Assert(pack.current_height == (float) -1.4610002E38F);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lon = (int) -1621023319;
            p136.current_height = (float) -1.4610002E38F;
            p136.terrain_height = (float)1.7201345E38F;
            p136.lat = (int) -1304429152;
            p136.loaded = (ushort)(ushort)29554;
            p136.pending = (ushort)(ushort)64027;
            p136.spacing = (ushort)(ushort)5542;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1984282540U);
                Debug.Assert(pack.press_diff == (float)1.5212309E38F);
                Debug.Assert(pack.press_abs == (float)2.103409E38F);
                Debug.Assert(pack.temperature == (short)(short)24698);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_abs = (float)2.103409E38F;
            p137.temperature = (short)(short)24698;
            p137.time_boot_ms = (uint)1984282540U;
            p137.press_diff = (float)1.5212309E38F;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {9.5945666E36F, -7.493466E37F, -1.2412856E38F, -3.067988E38F}));
                Debug.Assert(pack.time_usec == (ulong)4201608005149836085L);
                Debug.Assert(pack.x == (float) -2.7775586E38F);
                Debug.Assert(pack.z == (float) -5.2359147E37F);
                Debug.Assert(pack.y == (float)3.4027233E38F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.z = (float) -5.2359147E37F;
            p138.q_SET(new float[] {9.5945666E36F, -7.493466E37F, -1.2412856E38F, -3.067988E38F}, 0) ;
            p138.x = (float) -2.7775586E38F;
            p138.y = (float)3.4027233E38F;
            p138.time_usec = (ulong)4201608005149836085L;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)83);
                Debug.Assert(pack.target_system == (byte)(byte)31);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.2726203E38F, 2.5050585E38F, -7.681128E37F, 1.6267285E38F, -2.5136622E37F, 4.7308786E37F, -1.8710732E38F, -5.961404E37F}));
                Debug.Assert(pack.group_mlx == (byte)(byte)37);
                Debug.Assert(pack.time_usec == (ulong)7516243543046029561L);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.group_mlx = (byte)(byte)37;
            p139.time_usec = (ulong)7516243543046029561L;
            p139.target_system = (byte)(byte)31;
            p139.target_component = (byte)(byte)83;
            p139.controls_SET(new float[] {2.2726203E38F, 2.5050585E38F, -7.681128E37F, 1.6267285E38F, -2.5136622E37F, 4.7308786E37F, -1.8710732E38F, -5.961404E37F}, 0) ;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)77);
                Debug.Assert(pack.time_usec == (ulong)3034379045263369691L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.7937883E38F, -1.8332615E38F, 1.9003705E38F, -5.9506136E37F, -2.3518174E38F, 2.302769E38F, 4.7990483E37F, 7.120844E36F}));
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)3034379045263369691L;
            p140.group_mlx = (byte)(byte)77;
            p140.controls_SET(new float[] {-1.7937883E38F, -1.8332615E38F, 1.9003705E38F, -5.9506136E37F, -2.3518174E38F, 2.302769E38F, 4.7990483E37F, 7.120844E36F}, 0) ;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_terrain == (float) -2.5622312E38F);
                Debug.Assert(pack.altitude_amsl == (float)1.6171415E38F);
                Debug.Assert(pack.altitude_local == (float)3.6102423E37F);
                Debug.Assert(pack.time_usec == (ulong)2530582752215849718L);
                Debug.Assert(pack.altitude_relative == (float)4.9047187E37F);
                Debug.Assert(pack.bottom_clearance == (float)2.4927743E38F);
                Debug.Assert(pack.altitude_monotonic == (float) -1.1178068E38F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_local = (float)3.6102423E37F;
            p141.altitude_relative = (float)4.9047187E37F;
            p141.time_usec = (ulong)2530582752215849718L;
            p141.altitude_terrain = (float) -2.5622312E38F;
            p141.altitude_amsl = (float)1.6171415E38F;
            p141.altitude_monotonic = (float) -1.1178068E38F;
            p141.bottom_clearance = (float)2.4927743E38F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_type == (byte)(byte)151);
                Debug.Assert(pack.request_id == (byte)(byte)67);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)226, (byte)97, (byte)107, (byte)155, (byte)139, (byte)115, (byte)162, (byte)239, (byte)83, (byte)92, (byte)121, (byte)99, (byte)53, (byte)231, (byte)127, (byte)245, (byte)29, (byte)217, (byte)66, (byte)231, (byte)117, (byte)195, (byte)39, (byte)212, (byte)40, (byte)151, (byte)115, (byte)128, (byte)120, (byte)231, (byte)191, (byte)48, (byte)60, (byte)61, (byte)132, (byte)119, (byte)22, (byte)242, (byte)100, (byte)50, (byte)42, (byte)243, (byte)94, (byte)254, (byte)77, (byte)3, (byte)65, (byte)138, (byte)3, (byte)82, (byte)162, (byte)106, (byte)199, (byte)217, (byte)13, (byte)131, (byte)232, (byte)198, (byte)171, (byte)121, (byte)136, (byte)109, (byte)219, (byte)247, (byte)134, (byte)79, (byte)68, (byte)28, (byte)243, (byte)16, (byte)106, (byte)138, (byte)223, (byte)99, (byte)65, (byte)230, (byte)218, (byte)242, (byte)55, (byte)68, (byte)192, (byte)81, (byte)62, (byte)68, (byte)9, (byte)179, (byte)178, (byte)174, (byte)48, (byte)119, (byte)251, (byte)185, (byte)69, (byte)135, (byte)5, (byte)225, (byte)34, (byte)23, (byte)95, (byte)172, (byte)95, (byte)218, (byte)140, (byte)215, (byte)26, (byte)80, (byte)39, (byte)243, (byte)157, (byte)232, (byte)21, (byte)42, (byte)51, (byte)66, (byte)253, (byte)186, (byte)26, (byte)81, (byte)21, (byte)48}));
                Debug.Assert(pack.transfer_type == (byte)(byte)20);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)78, (byte)68, (byte)148, (byte)225, (byte)15, (byte)54, (byte)143, (byte)221, (byte)215, (byte)21, (byte)22, (byte)59, (byte)9, (byte)233, (byte)34, (byte)25, (byte)11, (byte)252, (byte)168, (byte)23, (byte)87, (byte)181, (byte)168, (byte)239, (byte)196, (byte)121, (byte)7, (byte)24, (byte)62, (byte)169, (byte)9, (byte)114, (byte)69, (byte)131, (byte)83, (byte)183, (byte)159, (byte)36, (byte)103, (byte)218, (byte)220, (byte)243, (byte)90, (byte)96, (byte)53, (byte)122, (byte)206, (byte)31, (byte)208, (byte)240, (byte)206, (byte)33, (byte)169, (byte)88, (byte)30, (byte)238, (byte)144, (byte)248, (byte)140, (byte)26, (byte)255, (byte)87, (byte)198, (byte)64, (byte)169, (byte)189, (byte)51, (byte)168, (byte)239, (byte)223, (byte)119, (byte)230, (byte)193, (byte)87, (byte)108, (byte)55, (byte)198, (byte)171, (byte)126, (byte)124, (byte)118, (byte)111, (byte)204, (byte)96, (byte)80, (byte)94, (byte)113, (byte)94, (byte)67, (byte)53, (byte)229, (byte)114, (byte)98, (byte)141, (byte)183, (byte)105, (byte)127, (byte)207, (byte)180, (byte)131, (byte)222, (byte)133, (byte)198, (byte)252, (byte)72, (byte)33, (byte)93, (byte)154, (byte)231, (byte)85, (byte)49, (byte)155, (byte)65, (byte)98, (byte)190, (byte)19, (byte)158, (byte)58, (byte)225, (byte)51}));
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.transfer_type = (byte)(byte)20;
            p142.request_id = (byte)(byte)67;
            p142.uri_type = (byte)(byte)151;
            p142.storage_SET(new byte[] {(byte)78, (byte)68, (byte)148, (byte)225, (byte)15, (byte)54, (byte)143, (byte)221, (byte)215, (byte)21, (byte)22, (byte)59, (byte)9, (byte)233, (byte)34, (byte)25, (byte)11, (byte)252, (byte)168, (byte)23, (byte)87, (byte)181, (byte)168, (byte)239, (byte)196, (byte)121, (byte)7, (byte)24, (byte)62, (byte)169, (byte)9, (byte)114, (byte)69, (byte)131, (byte)83, (byte)183, (byte)159, (byte)36, (byte)103, (byte)218, (byte)220, (byte)243, (byte)90, (byte)96, (byte)53, (byte)122, (byte)206, (byte)31, (byte)208, (byte)240, (byte)206, (byte)33, (byte)169, (byte)88, (byte)30, (byte)238, (byte)144, (byte)248, (byte)140, (byte)26, (byte)255, (byte)87, (byte)198, (byte)64, (byte)169, (byte)189, (byte)51, (byte)168, (byte)239, (byte)223, (byte)119, (byte)230, (byte)193, (byte)87, (byte)108, (byte)55, (byte)198, (byte)171, (byte)126, (byte)124, (byte)118, (byte)111, (byte)204, (byte)96, (byte)80, (byte)94, (byte)113, (byte)94, (byte)67, (byte)53, (byte)229, (byte)114, (byte)98, (byte)141, (byte)183, (byte)105, (byte)127, (byte)207, (byte)180, (byte)131, (byte)222, (byte)133, (byte)198, (byte)252, (byte)72, (byte)33, (byte)93, (byte)154, (byte)231, (byte)85, (byte)49, (byte)155, (byte)65, (byte)98, (byte)190, (byte)19, (byte)158, (byte)58, (byte)225, (byte)51}, 0) ;
            p142.uri_SET(new byte[] {(byte)226, (byte)97, (byte)107, (byte)155, (byte)139, (byte)115, (byte)162, (byte)239, (byte)83, (byte)92, (byte)121, (byte)99, (byte)53, (byte)231, (byte)127, (byte)245, (byte)29, (byte)217, (byte)66, (byte)231, (byte)117, (byte)195, (byte)39, (byte)212, (byte)40, (byte)151, (byte)115, (byte)128, (byte)120, (byte)231, (byte)191, (byte)48, (byte)60, (byte)61, (byte)132, (byte)119, (byte)22, (byte)242, (byte)100, (byte)50, (byte)42, (byte)243, (byte)94, (byte)254, (byte)77, (byte)3, (byte)65, (byte)138, (byte)3, (byte)82, (byte)162, (byte)106, (byte)199, (byte)217, (byte)13, (byte)131, (byte)232, (byte)198, (byte)171, (byte)121, (byte)136, (byte)109, (byte)219, (byte)247, (byte)134, (byte)79, (byte)68, (byte)28, (byte)243, (byte)16, (byte)106, (byte)138, (byte)223, (byte)99, (byte)65, (byte)230, (byte)218, (byte)242, (byte)55, (byte)68, (byte)192, (byte)81, (byte)62, (byte)68, (byte)9, (byte)179, (byte)178, (byte)174, (byte)48, (byte)119, (byte)251, (byte)185, (byte)69, (byte)135, (byte)5, (byte)225, (byte)34, (byte)23, (byte)95, (byte)172, (byte)95, (byte)218, (byte)140, (byte)215, (byte)26, (byte)80, (byte)39, (byte)243, (byte)157, (byte)232, (byte)21, (byte)42, (byte)51, (byte)66, (byte)253, (byte)186, (byte)26, (byte)81, (byte)21, (byte)48}, 0) ;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)13635);
                Debug.Assert(pack.press_diff == (float) -8.933577E37F);
                Debug.Assert(pack.press_abs == (float)2.6574265E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3776805966U);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_abs = (float)2.6574265E38F;
            p143.time_boot_ms = (uint)3776805966U;
            p143.press_diff = (float) -8.933577E37F;
            p143.temperature = (short)(short)13635;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.est_capabilities == (byte)(byte)202);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {3.6446326E37F, -5.78167E37F, -3.049111E37F, 1.8374916E38F}));
                Debug.Assert(pack.lon == (int) -1582855465);
                Debug.Assert(pack.timestamp == (ulong)3601732838639705590L);
                Debug.Assert(pack.alt == (float) -1.1396179E38F);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-1.7570223E38F, -2.5826956E37F, 1.0307761E38F}));
                Debug.Assert(pack.lat == (int)1735697873);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {3.1770904E38F, 1.5940934E38F, -2.6517767E37F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-2.5434724E38F, -1.1961823E38F, 1.4358539E38F}));
                Debug.Assert(pack.custom_state == (ulong)7336167033996729872L);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {2.224574E38F, 3.0896098E37F, -1.699611E38F}));
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.attitude_q_SET(new float[] {3.6446326E37F, -5.78167E37F, -3.049111E37F, 1.8374916E38F}, 0) ;
            p144.custom_state = (ulong)7336167033996729872L;
            p144.timestamp = (ulong)3601732838639705590L;
            p144.acc_SET(new float[] {3.1770904E38F, 1.5940934E38F, -2.6517767E37F}, 0) ;
            p144.rates_SET(new float[] {-1.7570223E38F, -2.5826956E37F, 1.0307761E38F}, 0) ;
            p144.lat = (int)1735697873;
            p144.est_capabilities = (byte)(byte)202;
            p144.lon = (int) -1582855465;
            p144.vel_SET(new float[] {-2.5434724E38F, -1.1961823E38F, 1.4358539E38F}, 0) ;
            p144.position_cov_SET(new float[] {2.224574E38F, 3.0896098E37F, -1.699611E38F}, 0) ;
            p144.alt = (float) -1.1396179E38F;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y_pos == (float) -2.7072082E38F);
                Debug.Assert(pack.y_vel == (float)2.0536765E38F);
                Debug.Assert(pack.z_vel == (float)3.1206685E37F);
                Debug.Assert(pack.yaw_rate == (float) -2.5886196E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {2.0508173E38F, 7.902504E37F, -1.0041743E38F}));
                Debug.Assert(pack.x_pos == (float)1.1558396E38F);
                Debug.Assert(pack.x_acc == (float) -3.3253191E38F);
                Debug.Assert(pack.y_acc == (float) -2.6831894E37F);
                Debug.Assert(pack.z_acc == (float) -9.796202E37F);
                Debug.Assert(pack.z_pos == (float)2.3990154E38F);
                Debug.Assert(pack.pitch_rate == (float) -2.457784E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {6.849111E37F, -1.1163837E38F, -1.572107E38F}));
                Debug.Assert(pack.time_usec == (ulong)2384124161779339468L);
                Debug.Assert(pack.x_vel == (float)2.7676983E38F);
                Debug.Assert(pack.airspeed == (float)1.9166058E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {9.774359E37F, 1.4383333E38F, -2.0733738E38F, 1.4666043E37F}));
                Debug.Assert(pack.roll_rate == (float)2.018377E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.pos_variance_SET(new float[] {2.0508173E38F, 7.902504E37F, -1.0041743E38F}, 0) ;
            p146.y_acc = (float) -2.6831894E37F;
            p146.z_pos = (float)2.3990154E38F;
            p146.x_vel = (float)2.7676983E38F;
            p146.airspeed = (float)1.9166058E38F;
            p146.yaw_rate = (float) -2.5886196E38F;
            p146.pitch_rate = (float) -2.457784E38F;
            p146.z_acc = (float) -9.796202E37F;
            p146.q_SET(new float[] {9.774359E37F, 1.4383333E38F, -2.0733738E38F, 1.4666043E37F}, 0) ;
            p146.y_vel = (float)2.0536765E38F;
            p146.roll_rate = (float)2.018377E38F;
            p146.z_vel = (float)3.1206685E37F;
            p146.x_acc = (float) -3.3253191E38F;
            p146.vel_variance_SET(new float[] {6.849111E37F, -1.1163837E38F, -1.572107E38F}, 0) ;
            p146.time_usec = (ulong)2384124161779339468L;
            p146.y_pos = (float) -2.7072082E38F;
            p146.x_pos = (float)1.1558396E38F;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_consumed == (int)391440870);
                Debug.Assert(pack.energy_consumed == (int) -1142832446);
                Debug.Assert(pack.temperature == (short)(short) -16250);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)55);
                Debug.Assert(pack.current_battery == (short)(short) -9328);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)48208, (ushort)35699, (ushort)21880, (ushort)50761, (ushort)19645, (ushort)36314, (ushort)3478, (ushort)52046, (ushort)62378, (ushort)26010}));
                Debug.Assert(pack.id == (byte)(byte)239);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)239;
            p147.energy_consumed = (int) -1142832446;
            p147.voltages_SET(new ushort[] {(ushort)48208, (ushort)35699, (ushort)21880, (ushort)50761, (ushort)19645, (ushort)36314, (ushort)3478, (ushort)52046, (ushort)62378, (ushort)26010}, 0) ;
            p147.temperature = (short)(short) -16250;
            p147.current_battery = (short)(short) -9328;
            p147.battery_remaining = (sbyte)(sbyte)55;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL;
            p147.current_consumed = (int)391440870;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_sw_version == (uint)566652698U);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)198, (byte)137, (byte)68, (byte)183, (byte)95, (byte)40, (byte)11, (byte)124, (byte)117, (byte)234, (byte)160, (byte)73, (byte)243, (byte)237, (byte)123, (byte)107, (byte)65, (byte)103}));
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)88, (byte)25, (byte)215, (byte)45, (byte)234, (byte)12, (byte)95, (byte)216}));
                Debug.Assert(pack.os_sw_version == (uint)4287872145U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)149, (byte)112, (byte)223, (byte)242, (byte)76, (byte)169, (byte)204, (byte)33}));
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)101, (byte)48, (byte)249, (byte)91, (byte)154, (byte)41, (byte)28, (byte)80}));
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT);
                Debug.Assert(pack.product_id == (ushort)(ushort)19326);
                Debug.Assert(pack.middleware_sw_version == (uint)4180676966U);
                Debug.Assert(pack.board_version == (uint)851041632U);
                Debug.Assert(pack.uid == (ulong)8640934396626693325L);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)31625);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
            p148.flight_sw_version = (uint)566652698U;
            p148.flight_custom_version_SET(new byte[] {(byte)88, (byte)25, (byte)215, (byte)45, (byte)234, (byte)12, (byte)95, (byte)216}, 0) ;
            p148.uid2_SET(new byte[] {(byte)198, (byte)137, (byte)68, (byte)183, (byte)95, (byte)40, (byte)11, (byte)124, (byte)117, (byte)234, (byte)160, (byte)73, (byte)243, (byte)237, (byte)123, (byte)107, (byte)65, (byte)103}, 0, PH) ;
            p148.middleware_sw_version = (uint)4180676966U;
            p148.os_custom_version_SET(new byte[] {(byte)101, (byte)48, (byte)249, (byte)91, (byte)154, (byte)41, (byte)28, (byte)80}, 0) ;
            p148.board_version = (uint)851041632U;
            p148.vendor_id = (ushort)(ushort)31625;
            p148.product_id = (ushort)(ushort)19326;
            p148.middleware_custom_version_SET(new byte[] {(byte)149, (byte)112, (byte)223, (byte)242, (byte)76, (byte)169, (byte)204, (byte)33}, 0) ;
            p148.os_sw_version = (uint)4287872145U;
            p148.uid = (ulong)8640934396626693325L;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.size_x == (float)2.1901232E38F);
                Debug.Assert(pack.angle_y == (float) -2.0801572E37F);
                Debug.Assert(pack.x_TRY(ph) == (float) -2.2251396E38F);
                Debug.Assert(pack.distance == (float)1.7170587E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {2.770082E38F, 2.3900725E38F, -1.4220303E37F, -1.725871E38F}));
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)246);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
                Debug.Assert(pack.target_num == (byte)(byte)185);
                Debug.Assert(pack.z_TRY(ph) == (float) -5.991742E37F);
                Debug.Assert(pack.angle_x == (float) -7.173474E37F);
                Debug.Assert(pack.time_usec == (ulong)6072528249186007791L);
                Debug.Assert(pack.size_y == (float)2.0315945E38F);
                Debug.Assert(pack.y_TRY(ph) == (float) -1.0606844E38F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p149.size_x = (float)2.1901232E38F;
            p149.distance = (float)1.7170587E38F;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.q_SET(new float[] {2.770082E38F, 2.3900725E38F, -1.4220303E37F, -1.725871E38F}, 0, PH) ;
            p149.time_usec = (ulong)6072528249186007791L;
            p149.y_SET((float) -1.0606844E38F, PH) ;
            p149.angle_y = (float) -2.0801572E37F;
            p149.z_SET((float) -5.991742E37F, PH) ;
            p149.angle_x = (float) -7.173474E37F;
            p149.x_SET((float) -2.2251396E38F, PH) ;
            p149.size_y = (float)2.0315945E38F;
            p149.position_valid_SET((byte)(byte)246, PH) ;
            p149.target_num = (byte)(byte)185;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mag_ratio == (float)1.8771749E36F);
                Debug.Assert(pack.pos_vert_ratio == (float)3.0041398E38F);
                Debug.Assert(pack.tas_ratio == (float) -2.576508E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)8.1395105E37F);
                Debug.Assert(pack.vel_ratio == (float) -3.9669632E37F);
                Debug.Assert(pack.pos_horiz_ratio == (float)1.4933058E38F);
                Debug.Assert(pack.time_usec == (ulong)3194757029528808662L);
                Debug.Assert(pack.hagl_ratio == (float) -1.650282E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
                Debug.Assert(pack.pos_vert_accuracy == (float) -3.2531953E37F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.vel_ratio = (float) -3.9669632E37F;
            p230.tas_ratio = (float) -2.576508E38F;
            p230.pos_vert_ratio = (float)3.0041398E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ;
            p230.pos_horiz_accuracy = (float)8.1395105E37F;
            p230.mag_ratio = (float)1.8771749E36F;
            p230.hagl_ratio = (float) -1.650282E38F;
            p230.pos_horiz_ratio = (float)1.4933058E38F;
            p230.time_usec = (ulong)3194757029528808662L;
            p230.pos_vert_accuracy = (float) -3.2531953E37F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vert_accuracy == (float) -2.758899E37F);
                Debug.Assert(pack.time_usec == (ulong)139187151597477175L);
                Debug.Assert(pack.wind_y == (float) -4.266592E37F);
                Debug.Assert(pack.wind_alt == (float) -5.973406E37F);
                Debug.Assert(pack.wind_x == (float)2.207162E38F);
                Debug.Assert(pack.horiz_accuracy == (float)3.1778145E38F);
                Debug.Assert(pack.var_horiz == (float) -1.9273887E38F);
                Debug.Assert(pack.var_vert == (float) -1.8399634E37F);
                Debug.Assert(pack.wind_z == (float) -3.2355628E37F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.horiz_accuracy = (float)3.1778145E38F;
            p231.vert_accuracy = (float) -2.758899E37F;
            p231.wind_y = (float) -4.266592E37F;
            p231.var_vert = (float) -1.8399634E37F;
            p231.time_usec = (ulong)139187151597477175L;
            p231.wind_alt = (float) -5.973406E37F;
            p231.var_horiz = (float) -1.9273887E38F;
            p231.wind_z = (float) -3.2355628E37F;
            p231.wind_x = (float)2.207162E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.speed_accuracy == (float) -1.8375334E38F);
                Debug.Assert(pack.alt == (float) -1.7938828E38F);
                Debug.Assert(pack.horiz_accuracy == (float)1.2281353E38F);
                Debug.Assert(pack.vdop == (float)1.161251E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)32019);
                Debug.Assert(pack.vd == (float)1.6631965E38F);
                Debug.Assert(pack.time_usec == (ulong)7598351009084707938L);
                Debug.Assert(pack.lon == (int) -926971113);
                Debug.Assert(pack.gps_id == (byte)(byte)46);
                Debug.Assert(pack.vert_accuracy == (float)1.9511384E38F);
                Debug.Assert(pack.time_week_ms == (uint)2295202943U);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ);
                Debug.Assert(pack.satellites_visible == (byte)(byte)50);
                Debug.Assert(pack.fix_type == (byte)(byte)27);
                Debug.Assert(pack.lat == (int)383208734);
                Debug.Assert(pack.hdop == (float) -2.8072072E37F);
                Debug.Assert(pack.vn == (float) -1.3344517E38F);
                Debug.Assert(pack.ve == (float)3.1132917E38F);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ;
            p232.ve = (float)3.1132917E38F;
            p232.vert_accuracy = (float)1.9511384E38F;
            p232.satellites_visible = (byte)(byte)50;
            p232.vd = (float)1.6631965E38F;
            p232.lon = (int) -926971113;
            p232.gps_id = (byte)(byte)46;
            p232.time_week = (ushort)(ushort)32019;
            p232.time_usec = (ulong)7598351009084707938L;
            p232.vn = (float) -1.3344517E38F;
            p232.alt = (float) -1.7938828E38F;
            p232.vdop = (float)1.161251E38F;
            p232.time_week_ms = (uint)2295202943U;
            p232.fix_type = (byte)(byte)27;
            p232.speed_accuracy = (float) -1.8375334E38F;
            p232.lat = (int)383208734;
            p232.horiz_accuracy = (float)1.2281353E38F;
            p232.hdop = (float) -2.8072072E37F;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (byte)(byte)210);
                Debug.Assert(pack.len == (byte)(byte)114);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)5, (byte)199, (byte)5, (byte)59, (byte)16, (byte)31, (byte)218, (byte)238, (byte)51, (byte)134, (byte)35, (byte)255, (byte)68, (byte)173, (byte)77, (byte)11, (byte)63, (byte)48, (byte)73, (byte)92, (byte)115, (byte)207, (byte)57, (byte)29, (byte)143, (byte)58, (byte)49, (byte)65, (byte)241, (byte)202, (byte)200, (byte)179, (byte)195, (byte)243, (byte)7, (byte)165, (byte)60, (byte)86, (byte)141, (byte)209, (byte)100, (byte)110, (byte)165, (byte)41, (byte)34, (byte)153, (byte)226, (byte)18, (byte)22, (byte)214, (byte)26, (byte)55, (byte)94, (byte)50, (byte)183, (byte)135, (byte)77, (byte)119, (byte)127, (byte)251, (byte)164, (byte)38, (byte)77, (byte)165, (byte)118, (byte)149, (byte)204, (byte)13, (byte)136, (byte)188, (byte)34, (byte)163, (byte)160, (byte)77, (byte)104, (byte)224, (byte)101, (byte)53, (byte)187, (byte)215, (byte)13, (byte)151, (byte)192, (byte)233, (byte)160, (byte)99, (byte)156, (byte)100, (byte)7, (byte)251, (byte)17, (byte)71, (byte)22, (byte)41, (byte)220, (byte)71, (byte)101, (byte)165, (byte)71, (byte)12, (byte)109, (byte)85, (byte)57, (byte)114, (byte)52, (byte)229, (byte)85, (byte)230, (byte)145, (byte)196, (byte)219, (byte)147, (byte)94, (byte)31, (byte)164, (byte)6, (byte)188, (byte)231, (byte)78, (byte)230, (byte)139, (byte)61, (byte)52, (byte)61, (byte)231, (byte)52, (byte)137, (byte)145, (byte)138, (byte)35, (byte)108, (byte)14, (byte)242, (byte)235, (byte)110, (byte)189, (byte)68, (byte)45, (byte)15, (byte)247, (byte)181, (byte)180, (byte)23, (byte)138, (byte)92, (byte)153, (byte)139, (byte)208, (byte)156, (byte)80, (byte)184, (byte)253, (byte)212, (byte)216, (byte)52, (byte)184, (byte)30, (byte)228, (byte)204, (byte)32, (byte)205, (byte)101, (byte)199, (byte)105, (byte)152, (byte)194, (byte)93, (byte)36, (byte)149, (byte)252, (byte)237, (byte)169, (byte)184, (byte)182, (byte)50, (byte)171, (byte)77, (byte)88, (byte)35, (byte)126}));
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)210;
            p233.len = (byte)(byte)114;
            p233.data__SET(new byte[] {(byte)5, (byte)199, (byte)5, (byte)59, (byte)16, (byte)31, (byte)218, (byte)238, (byte)51, (byte)134, (byte)35, (byte)255, (byte)68, (byte)173, (byte)77, (byte)11, (byte)63, (byte)48, (byte)73, (byte)92, (byte)115, (byte)207, (byte)57, (byte)29, (byte)143, (byte)58, (byte)49, (byte)65, (byte)241, (byte)202, (byte)200, (byte)179, (byte)195, (byte)243, (byte)7, (byte)165, (byte)60, (byte)86, (byte)141, (byte)209, (byte)100, (byte)110, (byte)165, (byte)41, (byte)34, (byte)153, (byte)226, (byte)18, (byte)22, (byte)214, (byte)26, (byte)55, (byte)94, (byte)50, (byte)183, (byte)135, (byte)77, (byte)119, (byte)127, (byte)251, (byte)164, (byte)38, (byte)77, (byte)165, (byte)118, (byte)149, (byte)204, (byte)13, (byte)136, (byte)188, (byte)34, (byte)163, (byte)160, (byte)77, (byte)104, (byte)224, (byte)101, (byte)53, (byte)187, (byte)215, (byte)13, (byte)151, (byte)192, (byte)233, (byte)160, (byte)99, (byte)156, (byte)100, (byte)7, (byte)251, (byte)17, (byte)71, (byte)22, (byte)41, (byte)220, (byte)71, (byte)101, (byte)165, (byte)71, (byte)12, (byte)109, (byte)85, (byte)57, (byte)114, (byte)52, (byte)229, (byte)85, (byte)230, (byte)145, (byte)196, (byte)219, (byte)147, (byte)94, (byte)31, (byte)164, (byte)6, (byte)188, (byte)231, (byte)78, (byte)230, (byte)139, (byte)61, (byte)52, (byte)61, (byte)231, (byte)52, (byte)137, (byte)145, (byte)138, (byte)35, (byte)108, (byte)14, (byte)242, (byte)235, (byte)110, (byte)189, (byte)68, (byte)45, (byte)15, (byte)247, (byte)181, (byte)180, (byte)23, (byte)138, (byte)92, (byte)153, (byte)139, (byte)208, (byte)156, (byte)80, (byte)184, (byte)253, (byte)212, (byte)216, (byte)52, (byte)184, (byte)30, (byte)228, (byte)204, (byte)32, (byte)205, (byte)101, (byte)199, (byte)105, (byte)152, (byte)194, (byte)93, (byte)36, (byte)149, (byte)252, (byte)237, (byte)169, (byte)184, (byte)182, (byte)50, (byte)171, (byte)77, (byte)88, (byte)35, (byte)126}, 0) ;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED);
                Debug.Assert(pack.pitch == (short)(short)24058);
                Debug.Assert(pack.latitude == (int)426515807);
                Debug.Assert(pack.failsafe == (byte)(byte)100);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
                Debug.Assert(pack.gps_nsat == (byte)(byte)3);
                Debug.Assert(pack.airspeed == (byte)(byte)251);
                Debug.Assert(pack.altitude_amsl == (short)(short)30298);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
                Debug.Assert(pack.heading_sp == (short)(short)18409);
                Debug.Assert(pack.custom_mode == (uint)1059946731U);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)16403);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)66);
                Debug.Assert(pack.roll == (short)(short)22758);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte) - 125);
                Debug.Assert(pack.altitude_sp == (short)(short) -8613);
                Debug.Assert(pack.groundspeed == (byte)(byte)209);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)114);
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 34);
                Debug.Assert(pack.longitude == (int) -501917987);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 104);
                Debug.Assert(pack.heading == (ushort)(ushort)20801);
                Debug.Assert(pack.wp_num == (byte)(byte)89);
                Debug.Assert(pack.battery_remaining == (byte)(byte)180);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.pitch = (short)(short)24058;
            p234.latitude = (int)426515807;
            p234.wp_distance = (ushort)(ushort)16403;
            p234.airspeed_sp = (byte)(byte)66;
            p234.gps_nsat = (byte)(byte)3;
            p234.failsafe = (byte)(byte)100;
            p234.temperature_air = (sbyte)(sbyte) - 125;
            p234.heading = (ushort)(ushort)20801;
            p234.climb_rate = (sbyte)(sbyte)114;
            p234.temperature = (sbyte)(sbyte) - 104;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p234.wp_num = (byte)(byte)89;
            p234.altitude_sp = (short)(short) -8613;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED;
            p234.custom_mode = (uint)1059946731U;
            p234.altitude_amsl = (short)(short)30298;
            p234.battery_remaining = (byte)(byte)180;
            p234.groundspeed = (byte)(byte)209;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p234.heading_sp = (short)(short)18409;
            p234.airspeed = (byte)(byte)251;
            p234.roll = (short)(short)22758;
            p234.throttle = (sbyte)(sbyte) - 34;
            p234.longitude = (int) -501917987;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_0 == (uint)1984732874U);
                Debug.Assert(pack.vibration_x == (float) -1.9793906E38F);
                Debug.Assert(pack.vibration_y == (float)1.968372E38F);
                Debug.Assert(pack.vibration_z == (float)1.0894171E38F);
                Debug.Assert(pack.clipping_1 == (uint)534524590U);
                Debug.Assert(pack.time_usec == (ulong)5261066560884574098L);
                Debug.Assert(pack.clipping_2 == (uint)3858621437U);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)5261066560884574098L;
            p241.clipping_0 = (uint)1984732874U;
            p241.clipping_1 = (uint)534524590U;
            p241.vibration_y = (float)1.968372E38F;
            p241.vibration_x = (float) -1.9793906E38F;
            p241.vibration_z = (float)1.0894171E38F;
            p241.clipping_2 = (uint)3858621437U;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)1.09948E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3546616014772063110L);
                Debug.Assert(pack.longitude == (int) -16619345);
                Debug.Assert(pack.y == (float) -3.1788306E38F);
                Debug.Assert(pack.approach_x == (float)1.5575715E38F);
                Debug.Assert(pack.x == (float) -1.5531369E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.3234499E38F, -1.3913111E38F, 4.2299168E37F, -2.3221424E38F}));
                Debug.Assert(pack.latitude == (int) -924937136);
                Debug.Assert(pack.altitude == (int) -1384533851);
                Debug.Assert(pack.approach_z == (float) -3.1646266E37F);
                Debug.Assert(pack.approach_y == (float)2.8875123E38F);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.approach_x = (float)1.5575715E38F;
            p242.longitude = (int) -16619345;
            p242.approach_z = (float) -3.1646266E37F;
            p242.q_SET(new float[] {-1.3234499E38F, -1.3913111E38F, 4.2299168E37F, -2.3221424E38F}, 0) ;
            p242.approach_y = (float)2.8875123E38F;
            p242.latitude = (int) -924937136;
            p242.z = (float)1.09948E38F;
            p242.time_usec_SET((ulong)3546616014772063110L, PH) ;
            p242.y = (float) -3.1788306E38F;
            p242.altitude = (int) -1384533851;
            p242.x = (float) -1.5531369E38F;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)210);
                Debug.Assert(pack.approach_y == (float)3.1040127E38F);
                Debug.Assert(pack.y == (float) -1.8156418E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7278592662484160882L);
                Debug.Assert(pack.altitude == (int)1809005020);
                Debug.Assert(pack.approach_x == (float)2.4992598E38F);
                Debug.Assert(pack.z == (float) -4.8233974E37F);
                Debug.Assert(pack.longitude == (int) -309410366);
                Debug.Assert(pack.approach_z == (float) -2.1484006E38F);
                Debug.Assert(pack.latitude == (int)502369842);
                Debug.Assert(pack.x == (float)7.108236E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.4450359E37F, 8.625847E36F, -3.1269663E38F, -1.9459351E38F}));
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)210;
            p243.z = (float) -4.8233974E37F;
            p243.y = (float) -1.8156418E38F;
            p243.approach_y = (float)3.1040127E38F;
            p243.longitude = (int) -309410366;
            p243.q_SET(new float[] {-2.4450359E37F, 8.625847E36F, -3.1269663E38F, -1.9459351E38F}, 0) ;
            p243.x = (float)7.108236E37F;
            p243.approach_x = (float)2.4992598E38F;
            p243.time_usec_SET((ulong)7278592662484160882L, PH) ;
            p243.approach_z = (float) -2.1484006E38F;
            p243.altitude = (int)1809005020;
            p243.latitude = (int)502369842;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)21462);
                Debug.Assert(pack.interval_us == (int)719356209);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)21462;
            p244.interval_us = (int)719356209;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.callsign_LEN(ph) == 5);
                Debug.Assert(pack.callsign_TRY(ph).Equals("ykdki"));
                Debug.Assert(pack.ver_velocity == (short)(short)23389);
                Debug.Assert(pack.squawk == (ushort)(ushort)24089);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.lon == (int) -792047127);
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ULTRA_LIGHT);
                Debug.Assert(pack.tslc == (byte)(byte)165);
                Debug.Assert(pack.heading == (ushort)(ushort)5587);
                Debug.Assert(pack.altitude == (int) -1176721339);
                Debug.Assert(pack.lat == (int)395273537);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)36460);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY);
                Debug.Assert(pack.ICAO_address == (uint)4244742657U);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)4244742657U;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.tslc = (byte)(byte)165;
            p246.squawk = (ushort)(ushort)24089;
            p246.heading = (ushort)(ushort)5587;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ULTRA_LIGHT;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY;
            p246.hor_velocity = (ushort)(ushort)36460;
            p246.lon = (int) -792047127;
            p246.lat = (int)395273537;
            p246.callsign_SET("ykdki", PH) ;
            p246.altitude = (int) -1176721339;
            p246.ver_velocity = (short)(short)23389;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_minimum_delta == (float)1.9158395E38F);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.time_to_minimum_delta == (float)2.55471E38F);
                Debug.Assert(pack.horizontal_minimum_delta == (float) -1.4538107E38F);
                Debug.Assert(pack.id == (uint)1904106255U);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.altitude_minimum_delta = (float)1.9158395E38F;
            p247.horizontal_minimum_delta = (float) -1.4538107E38F;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.time_to_minimum_delta = (float)2.55471E38F;
            p247.id = (uint)1904106255U;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)95, (byte)149, (byte)182, (byte)103, (byte)156, (byte)58, (byte)88, (byte)94, (byte)229, (byte)18, (byte)226, (byte)44, (byte)209, (byte)68, (byte)175, (byte)155, (byte)214, (byte)141, (byte)239, (byte)109, (byte)244, (byte)55, (byte)10, (byte)113, (byte)176, (byte)62, (byte)199, (byte)35, (byte)130, (byte)11, (byte)21, (byte)216, (byte)125, (byte)80, (byte)185, (byte)23, (byte)157, (byte)55, (byte)73, (byte)86, (byte)30, (byte)81, (byte)151, (byte)51, (byte)175, (byte)74, (byte)237, (byte)6, (byte)59, (byte)111, (byte)148, (byte)175, (byte)215, (byte)229, (byte)61, (byte)52, (byte)36, (byte)78, (byte)111, (byte)24, (byte)40, (byte)32, (byte)234, (byte)182, (byte)110, (byte)95, (byte)177, (byte)144, (byte)230, (byte)154, (byte)219, (byte)41, (byte)13, (byte)89, (byte)164, (byte)229, (byte)33, (byte)126, (byte)0, (byte)26, (byte)109, (byte)124, (byte)1, (byte)35, (byte)3, (byte)209, (byte)183, (byte)170, (byte)95, (byte)153, (byte)51, (byte)71, (byte)224, (byte)154, (byte)148, (byte)227, (byte)104, (byte)150, (byte)64, (byte)123, (byte)155, (byte)142, (byte)159, (byte)47, (byte)216, (byte)128, (byte)83, (byte)27, (byte)179, (byte)3, (byte)206, (byte)160, (byte)13, (byte)81, (byte)31, (byte)122, (byte)238, (byte)8, (byte)220, (byte)205, (byte)249, (byte)75, (byte)173, (byte)150, (byte)46, (byte)49, (byte)41, (byte)105, (byte)193, (byte)42, (byte)81, (byte)255, (byte)115, (byte)12, (byte)8, (byte)43, (byte)3, (byte)243, (byte)121, (byte)192, (byte)155, (byte)157, (byte)189, (byte)93, (byte)101, (byte)69, (byte)24, (byte)195, (byte)26, (byte)163, (byte)19, (byte)179, (byte)146, (byte)85, (byte)227, (byte)92, (byte)40, (byte)49, (byte)158, (byte)88, (byte)65, (byte)143, (byte)144, (byte)190, (byte)62, (byte)26, (byte)50, (byte)226, (byte)197, (byte)140, (byte)233, (byte)165, (byte)96, (byte)52, (byte)52, (byte)162, (byte)136, (byte)63, (byte)99, (byte)31, (byte)45, (byte)138, (byte)73, (byte)17, (byte)82, (byte)120, (byte)102, (byte)1, (byte)191, (byte)62, (byte)215, (byte)135, (byte)208, (byte)119, (byte)251, (byte)64, (byte)77, (byte)146, (byte)123, (byte)212, (byte)220, (byte)39, (byte)174, (byte)65, (byte)156, (byte)149, (byte)69, (byte)34, (byte)59, (byte)25, (byte)201, (byte)84, (byte)35, (byte)17, (byte)218, (byte)193, (byte)113, (byte)122, (byte)124, (byte)11, (byte)199, (byte)59, (byte)66, (byte)195, (byte)152, (byte)37, (byte)128, (byte)84, (byte)104, (byte)113, (byte)161, (byte)6, (byte)179, (byte)96, (byte)225, (byte)195, (byte)40, (byte)138, (byte)94, (byte)227, (byte)254, (byte)10, (byte)192, (byte)91, (byte)150, (byte)207, (byte)246, (byte)159, (byte)200}));
                Debug.Assert(pack.target_network == (byte)(byte)207);
                Debug.Assert(pack.target_system == (byte)(byte)185);
                Debug.Assert(pack.message_type == (ushort)(ushort)19139);
                Debug.Assert(pack.target_component == (byte)(byte)149);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_component = (byte)(byte)149;
            p248.target_network = (byte)(byte)207;
            p248.payload_SET(new byte[] {(byte)95, (byte)149, (byte)182, (byte)103, (byte)156, (byte)58, (byte)88, (byte)94, (byte)229, (byte)18, (byte)226, (byte)44, (byte)209, (byte)68, (byte)175, (byte)155, (byte)214, (byte)141, (byte)239, (byte)109, (byte)244, (byte)55, (byte)10, (byte)113, (byte)176, (byte)62, (byte)199, (byte)35, (byte)130, (byte)11, (byte)21, (byte)216, (byte)125, (byte)80, (byte)185, (byte)23, (byte)157, (byte)55, (byte)73, (byte)86, (byte)30, (byte)81, (byte)151, (byte)51, (byte)175, (byte)74, (byte)237, (byte)6, (byte)59, (byte)111, (byte)148, (byte)175, (byte)215, (byte)229, (byte)61, (byte)52, (byte)36, (byte)78, (byte)111, (byte)24, (byte)40, (byte)32, (byte)234, (byte)182, (byte)110, (byte)95, (byte)177, (byte)144, (byte)230, (byte)154, (byte)219, (byte)41, (byte)13, (byte)89, (byte)164, (byte)229, (byte)33, (byte)126, (byte)0, (byte)26, (byte)109, (byte)124, (byte)1, (byte)35, (byte)3, (byte)209, (byte)183, (byte)170, (byte)95, (byte)153, (byte)51, (byte)71, (byte)224, (byte)154, (byte)148, (byte)227, (byte)104, (byte)150, (byte)64, (byte)123, (byte)155, (byte)142, (byte)159, (byte)47, (byte)216, (byte)128, (byte)83, (byte)27, (byte)179, (byte)3, (byte)206, (byte)160, (byte)13, (byte)81, (byte)31, (byte)122, (byte)238, (byte)8, (byte)220, (byte)205, (byte)249, (byte)75, (byte)173, (byte)150, (byte)46, (byte)49, (byte)41, (byte)105, (byte)193, (byte)42, (byte)81, (byte)255, (byte)115, (byte)12, (byte)8, (byte)43, (byte)3, (byte)243, (byte)121, (byte)192, (byte)155, (byte)157, (byte)189, (byte)93, (byte)101, (byte)69, (byte)24, (byte)195, (byte)26, (byte)163, (byte)19, (byte)179, (byte)146, (byte)85, (byte)227, (byte)92, (byte)40, (byte)49, (byte)158, (byte)88, (byte)65, (byte)143, (byte)144, (byte)190, (byte)62, (byte)26, (byte)50, (byte)226, (byte)197, (byte)140, (byte)233, (byte)165, (byte)96, (byte)52, (byte)52, (byte)162, (byte)136, (byte)63, (byte)99, (byte)31, (byte)45, (byte)138, (byte)73, (byte)17, (byte)82, (byte)120, (byte)102, (byte)1, (byte)191, (byte)62, (byte)215, (byte)135, (byte)208, (byte)119, (byte)251, (byte)64, (byte)77, (byte)146, (byte)123, (byte)212, (byte)220, (byte)39, (byte)174, (byte)65, (byte)156, (byte)149, (byte)69, (byte)34, (byte)59, (byte)25, (byte)201, (byte)84, (byte)35, (byte)17, (byte)218, (byte)193, (byte)113, (byte)122, (byte)124, (byte)11, (byte)199, (byte)59, (byte)66, (byte)195, (byte)152, (byte)37, (byte)128, (byte)84, (byte)104, (byte)113, (byte)161, (byte)6, (byte)179, (byte)96, (byte)225, (byte)195, (byte)40, (byte)138, (byte)94, (byte)227, (byte)254, (byte)10, (byte)192, (byte)91, (byte)150, (byte)207, (byte)246, (byte)159, (byte)200}, 0) ;
            p248.target_system = (byte)(byte)185;
            p248.message_type = (ushort)(ushort)19139;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)174);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)76, (sbyte)53, (sbyte)16, (sbyte)19, (sbyte)16, (sbyte)58, (sbyte)1, (sbyte)95, (sbyte) - 15, (sbyte) - 89, (sbyte)77, (sbyte)127, (sbyte)12, (sbyte) - 116, (sbyte) - 52, (sbyte)72, (sbyte)51, (sbyte) - 90, (sbyte) - 67, (sbyte) - 1, (sbyte) - 57, (sbyte)115, (sbyte) - 59, (sbyte)112, (sbyte)1, (sbyte) - 31, (sbyte)92, (sbyte)96, (sbyte) - 52, (sbyte) - 27, (sbyte)103, (sbyte) - 25}));
                Debug.Assert(pack.ver == (byte)(byte)146);
                Debug.Assert(pack.address == (ushort)(ushort)15587);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.ver = (byte)(byte)146;
            p249.address = (ushort)(ushort)15587;
            p249.type = (byte)(byte)174;
            p249.value_SET(new sbyte[] {(sbyte)76, (sbyte)53, (sbyte)16, (sbyte)19, (sbyte)16, (sbyte)58, (sbyte)1, (sbyte)95, (sbyte) - 15, (sbyte) - 89, (sbyte)77, (sbyte)127, (sbyte)12, (sbyte) - 116, (sbyte) - 52, (sbyte)72, (sbyte)51, (sbyte) - 90, (sbyte) - 67, (sbyte) - 1, (sbyte) - 57, (sbyte)115, (sbyte) - 59, (sbyte)112, (sbyte)1, (sbyte) - 31, (sbyte)92, (sbyte)96, (sbyte) - 52, (sbyte) - 27, (sbyte)103, (sbyte) - 25}, 0) ;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)2.1092162E38F);
                Debug.Assert(pack.name_LEN(ph) == 8);
                Debug.Assert(pack.name_TRY(ph).Equals("yYrbMKtl"));
                Debug.Assert(pack.y == (float) -2.7124315E38F);
                Debug.Assert(pack.time_usec == (ulong)8669094410807349403L);
                Debug.Assert(pack.x == (float) -2.1090516E38F);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.y = (float) -2.7124315E38F;
            p250.name_SET("yYrbMKtl", PH) ;
            p250.time_usec = (ulong)8669094410807349403L;
            p250.z = (float)2.1092162E38F;
            p250.x = (float) -2.1090516E38F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 3);
                Debug.Assert(pack.name_TRY(ph).Equals("qrd"));
                Debug.Assert(pack.value == (float)1.3422888E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2559123283U);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.name_SET("qrd", PH) ;
            p251.time_boot_ms = (uint)2559123283U;
            p251.value = (float)1.3422888E38F;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 9);
                Debug.Assert(pack.name_TRY(ph).Equals("eswZuzbtf"));
                Debug.Assert(pack.value == (int) -823826621);
                Debug.Assert(pack.time_boot_ms == (uint)3483015888U);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.value = (int) -823826621;
            p252.name_SET("eswZuzbtf", PH) ;
            p252.time_boot_ms = (uint)3483015888U;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 43);
                Debug.Assert(pack.text_TRY(ph).Equals("lgcrylithlqxReminSdfisydcgVdmiguymaXPrmyowb"));
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_ERROR);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("lgcrylithlqxReminSdfisydcgVdmiguymaXPrmyowb", PH) ;
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_ERROR;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float)9.699238E37F);
                Debug.Assert(pack.ind == (byte)(byte)89);
                Debug.Assert(pack.time_boot_ms == (uint)189448498U);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)189448498U;
            p254.value = (float)9.699238E37F;
            p254.ind = (byte)(byte)89;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)73);
                Debug.Assert(pack.target_component == (byte)(byte)26);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)156, (byte)76, (byte)197, (byte)89, (byte)216, (byte)15, (byte)120, (byte)27, (byte)160, (byte)17, (byte)120, (byte)43, (byte)16, (byte)67, (byte)118, (byte)172, (byte)243, (byte)133, (byte)149, (byte)192, (byte)159, (byte)203, (byte)79, (byte)105, (byte)182, (byte)129, (byte)208, (byte)65, (byte)2, (byte)124, (byte)66, (byte)216}));
                Debug.Assert(pack.initial_timestamp == (ulong)5434692606567634334L);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_component = (byte)(byte)26;
            p256.target_system = (byte)(byte)73;
            p256.initial_timestamp = (ulong)5434692606567634334L;
            p256.secret_key_SET(new byte[] {(byte)156, (byte)76, (byte)197, (byte)89, (byte)216, (byte)15, (byte)120, (byte)27, (byte)160, (byte)17, (byte)120, (byte)43, (byte)16, (byte)67, (byte)118, (byte)172, (byte)243, (byte)133, (byte)149, (byte)192, (byte)159, (byte)203, (byte)79, (byte)105, (byte)182, (byte)129, (byte)208, (byte)65, (byte)2, (byte)124, (byte)66, (byte)216}, 0) ;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4013790750U);
                Debug.Assert(pack.state == (byte)(byte)9);
                Debug.Assert(pack.last_change_ms == (uint)1649016565U);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)4013790750U;
            p257.state = (byte)(byte)9;
            p257.last_change_ms = (uint)1649016565U;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)68);
                Debug.Assert(pack.target_system == (byte)(byte)230);
                Debug.Assert(pack.tune_LEN(ph) == 30);
                Debug.Assert(pack.tune_TRY(ph).Equals("qqmptwZstgerEUrlwflzvxeuyqXnmh"));
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)230;
            p258.target_component = (byte)(byte)68;
            p258.tune_SET("qqmptwZstgerEUrlwflzvxeuyqXnmh", PH) ;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)161, (byte)113, (byte)15, (byte)159, (byte)131, (byte)6, (byte)180, (byte)116, (byte)152, (byte)199, (byte)217, (byte)5, (byte)148, (byte)217, (byte)122, (byte)171, (byte)95, (byte)63, (byte)209, (byte)155, (byte)28, (byte)182, (byte)99, (byte)108, (byte)76, (byte)235, (byte)252, (byte)91, (byte)212, (byte)161, (byte)38, (byte)75}));
                Debug.Assert(pack.focal_length == (float) -2.3817086E38F);
                Debug.Assert(pack.firmware_version == (uint)1288260004U);
                Debug.Assert(pack.lens_id == (byte)(byte)228);
                Debug.Assert(pack.sensor_size_v == (float)2.5125677E38F);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)231, (byte)180, (byte)131, (byte)224, (byte)117, (byte)88, (byte)165, (byte)140, (byte)247, (byte)74, (byte)76, (byte)136, (byte)32, (byte)107, (byte)165, (byte)37, (byte)125, (byte)86, (byte)149, (byte)49, (byte)79, (byte)82, (byte)151, (byte)185, (byte)167, (byte)136, (byte)142, (byte)172, (byte)29, (byte)48, (byte)195, (byte)31}));
                Debug.Assert(pack.sensor_size_h == (float)9.255849E37F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)48875);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)10912);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 109);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("paghknzidZvzlmuqdbbqrpknzsxezvwwKngafVxcezZfuvybGtccgNwrBusHrgjglmublqdmzrpgyioRhnvadblAforaitmzfdlqznkmzkffa"));
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES);
                Debug.Assert(pack.time_boot_ms == (uint)397752200U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)32992);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.cam_definition_version = (ushort)(ushort)10912;
            p259.resolution_v = (ushort)(ushort)32992;
            p259.time_boot_ms = (uint)397752200U;
            p259.resolution_h = (ushort)(ushort)48875;
            p259.sensor_size_v = (float)2.5125677E38F;
            p259.lens_id = (byte)(byte)228;
            p259.sensor_size_h = (float)9.255849E37F;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES;
            p259.focal_length = (float) -2.3817086E38F;
            p259.cam_definition_uri_SET("paghknzidZvzlmuqdbbqrpknzsxezvwwKngafVxcezZfuvybGtccgNwrBusHrgjglmublqdmzrpgyioRhnvadblAforaitmzfdlqznkmzkffa", PH) ;
            p259.model_name_SET(new byte[] {(byte)231, (byte)180, (byte)131, (byte)224, (byte)117, (byte)88, (byte)165, (byte)140, (byte)247, (byte)74, (byte)76, (byte)136, (byte)32, (byte)107, (byte)165, (byte)37, (byte)125, (byte)86, (byte)149, (byte)49, (byte)79, (byte)82, (byte)151, (byte)185, (byte)167, (byte)136, (byte)142, (byte)172, (byte)29, (byte)48, (byte)195, (byte)31}, 0) ;
            p259.vendor_name_SET(new byte[] {(byte)161, (byte)113, (byte)15, (byte)159, (byte)131, (byte)6, (byte)180, (byte)116, (byte)152, (byte)199, (byte)217, (byte)5, (byte)148, (byte)217, (byte)122, (byte)171, (byte)95, (byte)63, (byte)209, (byte)155, (byte)28, (byte)182, (byte)99, (byte)108, (byte)76, (byte)235, (byte)252, (byte)91, (byte)212, (byte)161, (byte)38, (byte)75}, 0) ;
            p259.firmware_version = (uint)1288260004U;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2836285296U);
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_VIDEO);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)2836285296U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_VIDEO;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.total_capacity == (float)1.1991522E38F);
                Debug.Assert(pack.used_capacity == (float)1.2922737E38F);
                Debug.Assert(pack.storage_count == (byte)(byte)124);
                Debug.Assert(pack.write_speed == (float) -2.5716382E38F);
                Debug.Assert(pack.status == (byte)(byte)206);
                Debug.Assert(pack.read_speed == (float) -1.8877228E38F);
                Debug.Assert(pack.available_capacity == (float) -2.3440326E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3834076391U);
                Debug.Assert(pack.storage_id == (byte)(byte)90);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)3834076391U;
            p261.write_speed = (float) -2.5716382E38F;
            p261.available_capacity = (float) -2.3440326E38F;
            p261.total_capacity = (float)1.1991522E38F;
            p261.storage_count = (byte)(byte)124;
            p261.read_speed = (float) -1.8877228E38F;
            p261.storage_id = (byte)(byte)90;
            p261.status = (byte)(byte)206;
            p261.used_capacity = (float)1.2922737E38F;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1561349464U);
                Debug.Assert(pack.video_status == (byte)(byte)86);
                Debug.Assert(pack.available_capacity == (float) -2.2825063E38F);
                Debug.Assert(pack.image_interval == (float)9.17401E37F);
                Debug.Assert(pack.image_status == (byte)(byte)212);
                Debug.Assert(pack.recording_time_ms == (uint)2441146324U);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.video_status = (byte)(byte)86;
            p262.image_status = (byte)(byte)212;
            p262.time_boot_ms = (uint)1561349464U;
            p262.recording_time_ms = (uint)2441146324U;
            p262.image_interval = (float)9.17401E37F;
            p262.available_capacity = (float) -2.2825063E38F;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.image_index == (int) -453319356);
                Debug.Assert(pack.camera_id == (byte)(byte)191);
                Debug.Assert(pack.alt == (int) -1713062222);
                Debug.Assert(pack.relative_alt == (int) -2081817031);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-4.5058073E36F, -2.383909E38F, 2.4889764E38F, -1.6448005E38F}));
                Debug.Assert(pack.time_utc == (ulong)8621721062850583207L);
                Debug.Assert(pack.lat == (int) -2045475602);
                Debug.Assert(pack.time_boot_ms == (uint)180750054U);
                Debug.Assert(pack.lon == (int)742769062);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)2);
                Debug.Assert(pack.file_url_LEN(ph) == 86);
                Debug.Assert(pack.file_url_TRY(ph).Equals("veElwsohLqujyzvplbuifkajfzkcnnuybkkyblqqceMpwztpozvrvtywvsofoigkfhcieyjvSdzapifwhsrfir"));
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.file_url_SET("veElwsohLqujyzvplbuifkajfzkcnnuybkkyblqqceMpwztpozvrvtywvsofoigkfhcieyjvSdzapifwhsrfir", PH) ;
            p263.q_SET(new float[] {-4.5058073E36F, -2.383909E38F, 2.4889764E38F, -1.6448005E38F}, 0) ;
            p263.lat = (int) -2045475602;
            p263.capture_result = (sbyte)(sbyte)2;
            p263.lon = (int)742769062;
            p263.relative_alt = (int) -2081817031;
            p263.alt = (int) -1713062222;
            p263.camera_id = (byte)(byte)191;
            p263.image_index = (int) -453319356;
            p263.time_boot_ms = (uint)180750054U;
            p263.time_utc = (ulong)8621721062850583207L;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_uuid == (ulong)3091479458102120761L);
                Debug.Assert(pack.time_boot_ms == (uint)273590841U);
                Debug.Assert(pack.arming_time_utc == (ulong)7488597907357646048L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)6596373392294179484L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.takeoff_time_utc = (ulong)6596373392294179484L;
            p264.time_boot_ms = (uint)273590841U;
            p264.arming_time_utc = (ulong)7488597907357646048L;
            p264.flight_uuid = (ulong)3091479458102120761L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)1.4012133E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1205863771U);
                Debug.Assert(pack.pitch == (float)1.5422359E38F);
                Debug.Assert(pack.yaw == (float) -2.4059887E37F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.roll = (float)1.4012133E38F;
            p265.time_boot_ms = (uint)1205863771U;
            p265.yaw = (float) -2.4059887E37F;
            p265.pitch = (float)1.5422359E38F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)26050);
                Debug.Assert(pack.target_component == (byte)(byte)43);
                Debug.Assert(pack.first_message_offset == (byte)(byte)15);
                Debug.Assert(pack.length == (byte)(byte)179);
                Debug.Assert(pack.target_system == (byte)(byte)157);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)42, (byte)191, (byte)164, (byte)57, (byte)137, (byte)148, (byte)150, (byte)17, (byte)45, (byte)1, (byte)213, (byte)234, (byte)123, (byte)212, (byte)153, (byte)121, (byte)165, (byte)162, (byte)211, (byte)141, (byte)148, (byte)175, (byte)137, (byte)10, (byte)120, (byte)83, (byte)153, (byte)228, (byte)8, (byte)85, (byte)118, (byte)130, (byte)169, (byte)109, (byte)66, (byte)141, (byte)44, (byte)220, (byte)113, (byte)146, (byte)62, (byte)247, (byte)226, (byte)5, (byte)140, (byte)18, (byte)146, (byte)214, (byte)88, (byte)151, (byte)218, (byte)98, (byte)244, (byte)191, (byte)5, (byte)133, (byte)71, (byte)228, (byte)83, (byte)167, (byte)25, (byte)194, (byte)7, (byte)199, (byte)45, (byte)253, (byte)223, (byte)46, (byte)2, (byte)86, (byte)100, (byte)58, (byte)144, (byte)196, (byte)199, (byte)42, (byte)45, (byte)76, (byte)212, (byte)247, (byte)81, (byte)250, (byte)189, (byte)156, (byte)104, (byte)120, (byte)14, (byte)50, (byte)57, (byte)163, (byte)68, (byte)244, (byte)127, (byte)234, (byte)139, (byte)54, (byte)126, (byte)202, (byte)41, (byte)206, (byte)132, (byte)82, (byte)115, (byte)183, (byte)241, (byte)73, (byte)93, (byte)231, (byte)13, (byte)128, (byte)116, (byte)66, (byte)24, (byte)105, (byte)120, (byte)105, (byte)127, (byte)7, (byte)131, (byte)3, (byte)118, (byte)198, (byte)110, (byte)51, (byte)178, (byte)6, (byte)65, (byte)104, (byte)213, (byte)189, (byte)107, (byte)158, (byte)4, (byte)11, (byte)61, (byte)213, (byte)39, (byte)42, (byte)51, (byte)127, (byte)222, (byte)138, (byte)41, (byte)169, (byte)157, (byte)178, (byte)187, (byte)240, (byte)31, (byte)218, (byte)209, (byte)92, (byte)248, (byte)38, (byte)84, (byte)198, (byte)91, (byte)61, (byte)107, (byte)78, (byte)67, (byte)120, (byte)4, (byte)115, (byte)133, (byte)41, (byte)15, (byte)247, (byte)219, (byte)176, (byte)32, (byte)162, (byte)150, (byte)225, (byte)251, (byte)21, (byte)240, (byte)163, (byte)89, (byte)221, (byte)94, (byte)106, (byte)238, (byte)18, (byte)0, (byte)218, (byte)195, (byte)181, (byte)106, (byte)119, (byte)42, (byte)168, (byte)10, (byte)7, (byte)149, (byte)50, (byte)207, (byte)179, (byte)205, (byte)243, (byte)79, (byte)55, (byte)29, (byte)11, (byte)187, (byte)197, (byte)30, (byte)19, (byte)104, (byte)233, (byte)167, (byte)110, (byte)1, (byte)159, (byte)150, (byte)221, (byte)240, (byte)121, (byte)172, (byte)21, (byte)40, (byte)164, (byte)169, (byte)56, (byte)39, (byte)61, (byte)157, (byte)65, (byte)179, (byte)127, (byte)25, (byte)204, (byte)186, (byte)12, (byte)170, (byte)68, (byte)220, (byte)247, (byte)142, (byte)47, (byte)207, (byte)17, (byte)36, (byte)176, (byte)180, (byte)53, (byte)214, (byte)44, (byte)210}));
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.data__SET(new byte[] {(byte)42, (byte)191, (byte)164, (byte)57, (byte)137, (byte)148, (byte)150, (byte)17, (byte)45, (byte)1, (byte)213, (byte)234, (byte)123, (byte)212, (byte)153, (byte)121, (byte)165, (byte)162, (byte)211, (byte)141, (byte)148, (byte)175, (byte)137, (byte)10, (byte)120, (byte)83, (byte)153, (byte)228, (byte)8, (byte)85, (byte)118, (byte)130, (byte)169, (byte)109, (byte)66, (byte)141, (byte)44, (byte)220, (byte)113, (byte)146, (byte)62, (byte)247, (byte)226, (byte)5, (byte)140, (byte)18, (byte)146, (byte)214, (byte)88, (byte)151, (byte)218, (byte)98, (byte)244, (byte)191, (byte)5, (byte)133, (byte)71, (byte)228, (byte)83, (byte)167, (byte)25, (byte)194, (byte)7, (byte)199, (byte)45, (byte)253, (byte)223, (byte)46, (byte)2, (byte)86, (byte)100, (byte)58, (byte)144, (byte)196, (byte)199, (byte)42, (byte)45, (byte)76, (byte)212, (byte)247, (byte)81, (byte)250, (byte)189, (byte)156, (byte)104, (byte)120, (byte)14, (byte)50, (byte)57, (byte)163, (byte)68, (byte)244, (byte)127, (byte)234, (byte)139, (byte)54, (byte)126, (byte)202, (byte)41, (byte)206, (byte)132, (byte)82, (byte)115, (byte)183, (byte)241, (byte)73, (byte)93, (byte)231, (byte)13, (byte)128, (byte)116, (byte)66, (byte)24, (byte)105, (byte)120, (byte)105, (byte)127, (byte)7, (byte)131, (byte)3, (byte)118, (byte)198, (byte)110, (byte)51, (byte)178, (byte)6, (byte)65, (byte)104, (byte)213, (byte)189, (byte)107, (byte)158, (byte)4, (byte)11, (byte)61, (byte)213, (byte)39, (byte)42, (byte)51, (byte)127, (byte)222, (byte)138, (byte)41, (byte)169, (byte)157, (byte)178, (byte)187, (byte)240, (byte)31, (byte)218, (byte)209, (byte)92, (byte)248, (byte)38, (byte)84, (byte)198, (byte)91, (byte)61, (byte)107, (byte)78, (byte)67, (byte)120, (byte)4, (byte)115, (byte)133, (byte)41, (byte)15, (byte)247, (byte)219, (byte)176, (byte)32, (byte)162, (byte)150, (byte)225, (byte)251, (byte)21, (byte)240, (byte)163, (byte)89, (byte)221, (byte)94, (byte)106, (byte)238, (byte)18, (byte)0, (byte)218, (byte)195, (byte)181, (byte)106, (byte)119, (byte)42, (byte)168, (byte)10, (byte)7, (byte)149, (byte)50, (byte)207, (byte)179, (byte)205, (byte)243, (byte)79, (byte)55, (byte)29, (byte)11, (byte)187, (byte)197, (byte)30, (byte)19, (byte)104, (byte)233, (byte)167, (byte)110, (byte)1, (byte)159, (byte)150, (byte)221, (byte)240, (byte)121, (byte)172, (byte)21, (byte)40, (byte)164, (byte)169, (byte)56, (byte)39, (byte)61, (byte)157, (byte)65, (byte)179, (byte)127, (byte)25, (byte)204, (byte)186, (byte)12, (byte)170, (byte)68, (byte)220, (byte)247, (byte)142, (byte)47, (byte)207, (byte)17, (byte)36, (byte)176, (byte)180, (byte)53, (byte)214, (byte)44, (byte)210}, 0) ;
            p266.length = (byte)(byte)179;
            p266.target_component = (byte)(byte)43;
            p266.sequence = (ushort)(ushort)26050;
            p266.first_message_offset = (byte)(byte)15;
            p266.target_system = (byte)(byte)157;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)14);
                Debug.Assert(pack.length == (byte)(byte)226);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)221, (byte)6, (byte)22, (byte)43, (byte)222, (byte)218, (byte)182, (byte)71, (byte)1, (byte)175, (byte)52, (byte)120, (byte)174, (byte)249, (byte)159, (byte)81, (byte)187, (byte)27, (byte)86, (byte)192, (byte)143, (byte)40, (byte)138, (byte)144, (byte)103, (byte)109, (byte)160, (byte)224, (byte)50, (byte)56, (byte)128, (byte)239, (byte)72, (byte)162, (byte)34, (byte)226, (byte)55, (byte)82, (byte)72, (byte)204, (byte)50, (byte)233, (byte)219, (byte)91, (byte)100, (byte)199, (byte)119, (byte)55, (byte)159, (byte)56, (byte)56, (byte)16, (byte)21, (byte)155, (byte)17, (byte)98, (byte)193, (byte)200, (byte)94, (byte)176, (byte)122, (byte)210, (byte)171, (byte)178, (byte)23, (byte)57, (byte)68, (byte)188, (byte)77, (byte)196, (byte)238, (byte)96, (byte)110, (byte)17, (byte)43, (byte)6, (byte)60, (byte)241, (byte)81, (byte)166, (byte)75, (byte)127, (byte)86, (byte)243, (byte)194, (byte)128, (byte)238, (byte)200, (byte)164, (byte)124, (byte)175, (byte)10, (byte)172, (byte)221, (byte)214, (byte)214, (byte)119, (byte)44, (byte)115, (byte)71, (byte)141, (byte)197, (byte)9, (byte)201, (byte)253, (byte)208, (byte)52, (byte)156, (byte)192, (byte)97, (byte)71, (byte)231, (byte)83, (byte)225, (byte)235, (byte)79, (byte)114, (byte)40, (byte)156, (byte)7, (byte)179, (byte)67, (byte)194, (byte)163, (byte)225, (byte)74, (byte)98, (byte)55, (byte)1, (byte)148, (byte)143, (byte)87, (byte)115, (byte)12, (byte)123, (byte)166, (byte)23, (byte)128, (byte)250, (byte)103, (byte)30, (byte)179, (byte)110, (byte)82, (byte)197, (byte)48, (byte)131, (byte)57, (byte)133, (byte)161, (byte)170, (byte)176, (byte)225, (byte)152, (byte)86, (byte)7, (byte)42, (byte)61, (byte)197, (byte)98, (byte)152, (byte)123, (byte)124, (byte)180, (byte)238, (byte)170, (byte)211, (byte)67, (byte)195, (byte)86, (byte)130, (byte)210, (byte)26, (byte)113, (byte)109, (byte)211, (byte)84, (byte)19, (byte)72, (byte)212, (byte)156, (byte)214, (byte)86, (byte)27, (byte)53, (byte)128, (byte)131, (byte)219, (byte)10, (byte)237, (byte)170, (byte)198, (byte)42, (byte)254, (byte)255, (byte)164, (byte)4, (byte)56, (byte)132, (byte)250, (byte)150, (byte)27, (byte)45, (byte)25, (byte)156, (byte)131, (byte)19, (byte)249, (byte)253, (byte)141, (byte)187, (byte)244, (byte)182, (byte)123, (byte)78, (byte)105, (byte)144, (byte)127, (byte)36, (byte)36, (byte)111, (byte)98, (byte)236, (byte)163, (byte)222, (byte)13, (byte)211, (byte)226, (byte)244, (byte)235, (byte)145, (byte)0, (byte)214, (byte)19, (byte)129, (byte)249, (byte)48, (byte)59, (byte)130, (byte)161, (byte)56, (byte)241, (byte)235, (byte)34, (byte)210, (byte)76, (byte)214, (byte)221, (byte)241}));
                Debug.Assert(pack.sequence == (ushort)(ushort)32390);
                Debug.Assert(pack.first_message_offset == (byte)(byte)82);
                Debug.Assert(pack.target_component == (byte)(byte)12);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)14;
            p267.first_message_offset = (byte)(byte)82;
            p267.sequence = (ushort)(ushort)32390;
            p267.length = (byte)(byte)226;
            p267.target_component = (byte)(byte)12;
            p267.data__SET(new byte[] {(byte)221, (byte)6, (byte)22, (byte)43, (byte)222, (byte)218, (byte)182, (byte)71, (byte)1, (byte)175, (byte)52, (byte)120, (byte)174, (byte)249, (byte)159, (byte)81, (byte)187, (byte)27, (byte)86, (byte)192, (byte)143, (byte)40, (byte)138, (byte)144, (byte)103, (byte)109, (byte)160, (byte)224, (byte)50, (byte)56, (byte)128, (byte)239, (byte)72, (byte)162, (byte)34, (byte)226, (byte)55, (byte)82, (byte)72, (byte)204, (byte)50, (byte)233, (byte)219, (byte)91, (byte)100, (byte)199, (byte)119, (byte)55, (byte)159, (byte)56, (byte)56, (byte)16, (byte)21, (byte)155, (byte)17, (byte)98, (byte)193, (byte)200, (byte)94, (byte)176, (byte)122, (byte)210, (byte)171, (byte)178, (byte)23, (byte)57, (byte)68, (byte)188, (byte)77, (byte)196, (byte)238, (byte)96, (byte)110, (byte)17, (byte)43, (byte)6, (byte)60, (byte)241, (byte)81, (byte)166, (byte)75, (byte)127, (byte)86, (byte)243, (byte)194, (byte)128, (byte)238, (byte)200, (byte)164, (byte)124, (byte)175, (byte)10, (byte)172, (byte)221, (byte)214, (byte)214, (byte)119, (byte)44, (byte)115, (byte)71, (byte)141, (byte)197, (byte)9, (byte)201, (byte)253, (byte)208, (byte)52, (byte)156, (byte)192, (byte)97, (byte)71, (byte)231, (byte)83, (byte)225, (byte)235, (byte)79, (byte)114, (byte)40, (byte)156, (byte)7, (byte)179, (byte)67, (byte)194, (byte)163, (byte)225, (byte)74, (byte)98, (byte)55, (byte)1, (byte)148, (byte)143, (byte)87, (byte)115, (byte)12, (byte)123, (byte)166, (byte)23, (byte)128, (byte)250, (byte)103, (byte)30, (byte)179, (byte)110, (byte)82, (byte)197, (byte)48, (byte)131, (byte)57, (byte)133, (byte)161, (byte)170, (byte)176, (byte)225, (byte)152, (byte)86, (byte)7, (byte)42, (byte)61, (byte)197, (byte)98, (byte)152, (byte)123, (byte)124, (byte)180, (byte)238, (byte)170, (byte)211, (byte)67, (byte)195, (byte)86, (byte)130, (byte)210, (byte)26, (byte)113, (byte)109, (byte)211, (byte)84, (byte)19, (byte)72, (byte)212, (byte)156, (byte)214, (byte)86, (byte)27, (byte)53, (byte)128, (byte)131, (byte)219, (byte)10, (byte)237, (byte)170, (byte)198, (byte)42, (byte)254, (byte)255, (byte)164, (byte)4, (byte)56, (byte)132, (byte)250, (byte)150, (byte)27, (byte)45, (byte)25, (byte)156, (byte)131, (byte)19, (byte)249, (byte)253, (byte)141, (byte)187, (byte)244, (byte)182, (byte)123, (byte)78, (byte)105, (byte)144, (byte)127, (byte)36, (byte)36, (byte)111, (byte)98, (byte)236, (byte)163, (byte)222, (byte)13, (byte)211, (byte)226, (byte)244, (byte)235, (byte)145, (byte)0, (byte)214, (byte)19, (byte)129, (byte)249, (byte)48, (byte)59, (byte)130, (byte)161, (byte)56, (byte)241, (byte)235, (byte)34, (byte)210, (byte)76, (byte)214, (byte)221, (byte)241}, 0) ;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)213);
                Debug.Assert(pack.target_system == (byte)(byte)199);
                Debug.Assert(pack.sequence == (ushort)(ushort)32558);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.sequence = (ushort)(ushort)32558;
            p268.target_component = (byte)(byte)213;
            p268.target_system = (byte)(byte)199;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)33166);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)669);
                Debug.Assert(pack.rotation == (ushort)(ushort)7482);
                Debug.Assert(pack.bitrate == (uint)839981699U);
                Debug.Assert(pack.status == (byte)(byte)105);
                Debug.Assert(pack.framerate == (float)1.4403903E38F);
                Debug.Assert(pack.camera_id == (byte)(byte)241);
                Debug.Assert(pack.uri_LEN(ph) == 216);
                Debug.Assert(pack.uri_TRY(ph).Equals("NmrhdcrflBbuLcfboEkfaqcdcecLmtyburtgndqZovcwLthysshnyuzkwWqaixyfjMdxxbzxzgatvbXsiwbiqBtveyihotclzteispqgqrfrdnlhlguabntlqgsuppwvitkrcbicyuokyyBopalyjkpugrCgossqlpblevyknmGuNvQrXvgzwhwcuqCvcQxksptkjejvmpgLndbpyjHfgxst"));
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.resolution_h = (ushort)(ushort)669;
            p269.bitrate = (uint)839981699U;
            p269.rotation = (ushort)(ushort)7482;
            p269.uri_SET("NmrhdcrflBbuLcfboEkfaqcdcecLmtyburtgndqZovcwLthysshnyuzkwWqaixyfjMdxxbzxzgatvbXsiwbiqBtveyihotclzteispqgqrfrdnlhlguabntlqgsuppwvitkrcbicyuokyyBopalyjkpugrCgossqlpblevyknmGuNvQrXvgzwhwcuqCvcQxksptkjejvmpgLndbpyjHfgxst", PH) ;
            p269.resolution_v = (ushort)(ushort)33166;
            p269.framerate = (float)1.4403903E38F;
            p269.status = (byte)(byte)105;
            p269.camera_id = (byte)(byte)241;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.framerate == (float) -2.6094515E38F);
                Debug.Assert(pack.camera_id == (byte)(byte)98);
                Debug.Assert(pack.rotation == (ushort)(ushort)11943);
                Debug.Assert(pack.target_system == (byte)(byte)249);
                Debug.Assert(pack.bitrate == (uint)3777765498U);
                Debug.Assert(pack.uri_LEN(ph) == 71);
                Debug.Assert(pack.uri_TRY(ph).Equals("aqlsttjtonjqrzDrTiaxvAaawowOdvrMjpfbaxjghmqasycepvqvfNnjjbkgyysodGjnpyc"));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)45949);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)10493);
                Debug.Assert(pack.target_component == (byte)(byte)181);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.framerate = (float) -2.6094515E38F;
            p270.target_system = (byte)(byte)249;
            p270.bitrate = (uint)3777765498U;
            p270.camera_id = (byte)(byte)98;
            p270.resolution_v = (ushort)(ushort)10493;
            p270.rotation = (ushort)(ushort)11943;
            p270.target_component = (byte)(byte)181;
            p270.uri_SET("aqlsttjtonjqrzDrTiaxvAaawowOdvrMjpfbaxjghmqasycepvqvfNnjjbkgyysodGjnpyc", PH) ;
            p270.resolution_h = (ushort)(ushort)45949;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ssid_LEN(ph) == 16);
                Debug.Assert(pack.ssid_TRY(ph).Equals("QqfkMbadBOvjWoic"));
                Debug.Assert(pack.password_LEN(ph) == 56);
                Debug.Assert(pack.password_TRY(ph).Equals("SmleejwpfrtvivxjhvtiApuknmyxvlcsftpyjgollcfnNnxtkcsghihd"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("QqfkMbadBOvjWoic", PH) ;
            p299.password_SET("SmleejwpfrtvivxjhvtiApuknmyxvlcsftpyjgollcfnNnxtkcsghihd", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (ushort)(ushort)22593);
                Debug.Assert(pack.min_version == (ushort)(ushort)11133);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)242, (byte)124, (byte)119, (byte)43, (byte)126, (byte)68, (byte)147, (byte)144}));
                Debug.Assert(pack.max_version == (ushort)(ushort)26550);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)162, (byte)52, (byte)94, (byte)25, (byte)102, (byte)196, (byte)27, (byte)34}));
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.spec_version_hash_SET(new byte[] {(byte)242, (byte)124, (byte)119, (byte)43, (byte)126, (byte)68, (byte)147, (byte)144}, 0) ;
            p300.max_version = (ushort)(ushort)26550;
            p300.library_version_hash_SET(new byte[] {(byte)162, (byte)52, (byte)94, (byte)25, (byte)102, (byte)196, (byte)27, (byte)34}, 0) ;
            p300.version = (ushort)(ushort)22593;
            p300.min_version = (ushort)(ushort)11133;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uptime_sec == (uint)2231041893U);
                Debug.Assert(pack.time_usec == (ulong)7280722621124084994L);
                Debug.Assert(pack.sub_mode == (byte)(byte)14);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)54910);
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL;
            p310.uptime_sec = (uint)2231041893U;
            p310.sub_mode = (byte)(byte)14;
            p310.vendor_specific_status_code = (ushort)(ushort)54910;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE;
            p310.time_usec = (ulong)7280722621124084994L;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hw_version_minor == (byte)(byte)219);
                Debug.Assert(pack.uptime_sec == (uint)3990499818U);
                Debug.Assert(pack.sw_version_major == (byte)(byte)164);
                Debug.Assert(pack.time_usec == (ulong)3851240043637539839L);
                Debug.Assert(pack.hw_version_major == (byte)(byte)231);
                Debug.Assert(pack.sw_vcs_commit == (uint)3616647060U);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)70);
                Debug.Assert(pack.name_LEN(ph) == 54);
                Debug.Assert(pack.name_TRY(ph).Equals("mvYfoOhnwlcosnntitsjgllTngcgdgPbqxjuztwrpvklnhaxbtzxCw"));
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)131, (byte)204, (byte)81, (byte)188, (byte)245, (byte)119, (byte)48, (byte)121, (byte)233, (byte)192, (byte)168, (byte)10, (byte)212, (byte)68, (byte)236, (byte)84}));
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_version_minor = (byte)(byte)219;
            p311.sw_vcs_commit = (uint)3616647060U;
            p311.sw_version_major = (byte)(byte)164;
            p311.time_usec = (ulong)3851240043637539839L;
            p311.name_SET("mvYfoOhnwlcosnntitsjgllTngcgdgPbqxjuztwrpvklnhaxbtzxCw", PH) ;
            p311.hw_unique_id_SET(new byte[] {(byte)131, (byte)204, (byte)81, (byte)188, (byte)245, (byte)119, (byte)48, (byte)121, (byte)233, (byte)192, (byte)168, (byte)10, (byte)212, (byte)68, (byte)236, (byte)84}, 0) ;
            p311.uptime_sec = (uint)3990499818U;
            p311.hw_version_major = (byte)(byte)231;
            p311.sw_version_minor = (byte)(byte)70;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)252);
                Debug.Assert(pack.target_component == (byte)(byte)73);
                Debug.Assert(pack.param_id_LEN(ph) == 14);
                Debug.Assert(pack.param_id_TRY(ph).Equals("hkWncwBauivwov"));
                Debug.Assert(pack.param_index == (short)(short) -957);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_index = (short)(short) -957;
            p320.param_id_SET("hkWncwBauivwov", PH) ;
            p320.target_component = (byte)(byte)73;
            p320.target_system = (byte)(byte)252;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)182);
                Debug.Assert(pack.target_component == (byte)(byte)209);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)209;
            p321.target_system = (byte)(byte)182;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("za"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
                Debug.Assert(pack.param_value_LEN(ph) == 119);
                Debug.Assert(pack.param_value_TRY(ph).Equals("tpmeybdUdvfbzsurktQfwwgyhszUwmfwflidoepbfwpliosxbbfsbspuagjqvujuyiljockfwgbcojrqmpwtshHlUNwpjavxyynpgsfusPcsmxbkQvrpknf"));
                Debug.Assert(pack.param_index == (ushort)(ushort)14068);
                Debug.Assert(pack.param_count == (ushort)(ushort)7485);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_index = (ushort)(ushort)14068;
            p322.param_value_SET("tpmeybdUdvfbzsurktQfwwgyhszUwmfwflidoepbfwpliosxbbfsbspuagjqvujuyiljockfwgbcojrqmpwtshHlUNwpjavxyynpgsfusPcsmxbkQvrpknf", PH) ;
            p322.param_id_SET("za", PH) ;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8;
            p322.param_count = (ushort)(ushort)7485;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)150);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.target_system == (byte)(byte)71);
                Debug.Assert(pack.param_value_LEN(ph) == 76);
                Debug.Assert(pack.param_value_TRY(ph).Equals("yjDuvvvdhlfXaevkpddeulrHfmISlnwyyhTkkpjcKqflfkpqysDPwkronNknboqbreoubbrosojr"));
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Feeamzrjgr"));
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_value_SET("yjDuvvvdhlfXaevkpddeulrHfmISlnwyyhTkkpjcKqflfkpqysDPwkronNknboqbreoubbrosojr", PH) ;
            p323.target_component = (byte)(byte)150;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p323.param_id_SET("Feeamzrjgr", PH) ;
            p323.target_system = (byte)(byte)71;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("haCiakzxrmqp"));
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.param_value_LEN(ph) == 74);
                Debug.Assert(pack.param_value_TRY(ph).Equals("ymcnbieivlftsoYtWvXvjbgawqpflmcaAfLyWpaaddqkWpubpcgsvZmnzOomshubdZjaBqVakf"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_value_SET("ymcnbieivlftsoYtWvXvjbgawqpflmcaAfLyWpaaddqkWpubpcgsvZmnzOomshubdZjaBqVakf", PH) ;
            p324.param_id_SET("haCiakzxrmqp", PH) ;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED;
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
                Debug.Assert(pack.max_distance == (ushort)(ushort)56714);
                Debug.Assert(pack.min_distance == (ushort)(ushort)32577);
                Debug.Assert(pack.time_usec == (ulong)9203577475784985756L);
                Debug.Assert(pack.increment == (byte)(byte)195);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)53504, (ushort)61638, (ushort)49071, (ushort)56288, (ushort)37693, (ushort)12362, (ushort)46304, (ushort)57499, (ushort)25733, (ushort)28375, (ushort)32533, (ushort)59446, (ushort)51466, (ushort)39026, (ushort)22771, (ushort)26584, (ushort)20237, (ushort)24559, (ushort)10130, (ushort)14279, (ushort)27077, (ushort)17175, (ushort)48667, (ushort)54578, (ushort)50509, (ushort)62778, (ushort)59892, (ushort)57424, (ushort)59952, (ushort)28786, (ushort)8084, (ushort)17077, (ushort)57297, (ushort)41171, (ushort)4292, (ushort)25910, (ushort)22641, (ushort)26036, (ushort)4011, (ushort)38838, (ushort)26519, (ushort)6111, (ushort)41645, (ushort)19262, (ushort)31355, (ushort)27615, (ushort)27690, (ushort)64735, (ushort)52559, (ushort)14683, (ushort)60137, (ushort)44015, (ushort)7539, (ushort)46119, (ushort)23054, (ushort)65211, (ushort)28187, (ushort)43129, (ushort)33075, (ushort)54853, (ushort)30533, (ushort)39621, (ushort)9106, (ushort)16146, (ushort)15550, (ushort)59899, (ushort)23880, (ushort)51588, (ushort)19690, (ushort)48098, (ushort)36658, (ushort)38246}));
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.distances_SET(new ushort[] {(ushort)53504, (ushort)61638, (ushort)49071, (ushort)56288, (ushort)37693, (ushort)12362, (ushort)46304, (ushort)57499, (ushort)25733, (ushort)28375, (ushort)32533, (ushort)59446, (ushort)51466, (ushort)39026, (ushort)22771, (ushort)26584, (ushort)20237, (ushort)24559, (ushort)10130, (ushort)14279, (ushort)27077, (ushort)17175, (ushort)48667, (ushort)54578, (ushort)50509, (ushort)62778, (ushort)59892, (ushort)57424, (ushort)59952, (ushort)28786, (ushort)8084, (ushort)17077, (ushort)57297, (ushort)41171, (ushort)4292, (ushort)25910, (ushort)22641, (ushort)26036, (ushort)4011, (ushort)38838, (ushort)26519, (ushort)6111, (ushort)41645, (ushort)19262, (ushort)31355, (ushort)27615, (ushort)27690, (ushort)64735, (ushort)52559, (ushort)14683, (ushort)60137, (ushort)44015, (ushort)7539, (ushort)46119, (ushort)23054, (ushort)65211, (ushort)28187, (ushort)43129, (ushort)33075, (ushort)54853, (ushort)30533, (ushort)39621, (ushort)9106, (ushort)16146, (ushort)15550, (ushort)59899, (ushort)23880, (ushort)51588, (ushort)19690, (ushort)48098, (ushort)36658, (ushort)38246}, 0) ;
            p330.max_distance = (ushort)(ushort)56714;
            p330.time_usec = (ulong)9203577475784985756L;
            p330.min_distance = (ushort)(ushort)32577;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p330.increment = (byte)(byte)195;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
        }
    }
}
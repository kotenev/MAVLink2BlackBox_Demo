
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
        new class NAV_FILTER_BIAS : GroundControl.NAV_FILTER_BIAS
        {
            public ulong usec //Timestamp (microseconds)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float accel_0 //b_f[0]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float accel_1 //b_f[1]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float accel_2 //b_f[2]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float gyro_0 //b_f[0]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float gyro_1 //b_f[1]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float gyro_2 //b_f[2]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }
        }
        new class RADIO_CALIBRATION : GroundControl.RADIO_CALIBRATION
        {
            public ushort[] aileron //Aileron setpoints: left, center, right
            {
                get {return aileron_GET(new ushort[3], 0);}
            }
            public ushort[]aileron_GET(ushort[] dst_ch, int pos)  //Aileron setpoints: left, center, right
            {
                for(int BYTE = 0, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort[] elevator //Elevator setpoints: nose down, center, nose up
            {
                get {return elevator_GET(new ushort[3], 0);}
            }
            public ushort[]elevator_GET(ushort[] dst_ch, int pos)  //Elevator setpoints: nose down, center, nose up
            {
                for(int BYTE = 6, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort[] rudder //Rudder setpoints: nose left, center, nose right
            {
                get {return rudder_GET(new ushort[3], 0);}
            }
            public ushort[]rudder_GET(ushort[] dst_ch, int pos)  //Rudder setpoints: nose left, center, nose right
            {
                for(int BYTE = 12, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort[] gyro //Tail gyro mode/gain setpoints: heading hold, rate mode
            {
                get {return gyro_GET(new ushort[2], 0);}
            }
            public ushort[]gyro_GET(ushort[] dst_ch, int pos)  //Tail gyro mode/gain setpoints: heading hold, rate mode
            {
                for(int BYTE = 18, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort[] pitch //Pitch curve setpoints (every 25%)
            {
                get {return pitch_GET(new ushort[5], 0);}
            }
            public ushort[]pitch_GET(ushort[] dst_ch, int pos)  //Pitch curve setpoints (every 25%)
            {
                for(int BYTE = 22, dst_max = pos + 5; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort[] throttle //Throttle curve setpoints (every 25%)
            {
                get {return throttle_GET(new ushort[5], 0);}
            }
            public ushort[]throttle_GET(ushort[] dst_ch, int pos)  //Throttle curve setpoints (every 25%)
            {
                for(int BYTE = 32, dst_max = pos + 5; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
        }
        new class UALBERTA_SYS_STATUS : GroundControl.UALBERTA_SYS_STATUS
        {
            public byte mode //System mode, see UALBERTA_AUTOPILOT_MODE ENUM
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte nav_mode //Navigation mode, see UALBERTA_NAV_MODE ENUM
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte pilot //Pilot mode, see UALBERTA_PILOT_MODE
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
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
            public void OnNAV_FILTER_BIASReceive_direct(Channel src, Inside ph, NAV_FILTER_BIAS pack) {OnNAV_FILTER_BIASReceive(this, ph,  pack);}
            public event NAV_FILTER_BIASReceiveHandler OnNAV_FILTER_BIASReceive;
            public delegate void NAV_FILTER_BIASReceiveHandler(Channel src, Inside ph, NAV_FILTER_BIAS pack);
            public void OnRADIO_CALIBRATIONReceive_direct(Channel src, Inside ph, RADIO_CALIBRATION pack) {OnRADIO_CALIBRATIONReceive(this, ph,  pack);}
            public event RADIO_CALIBRATIONReceiveHandler OnRADIO_CALIBRATIONReceive;
            public delegate void RADIO_CALIBRATIONReceiveHandler(Channel src, Inside ph, RADIO_CALIBRATION pack);
            public void OnUALBERTA_SYS_STATUSReceive_direct(Channel src, Inside ph, UALBERTA_SYS_STATUS pack) {OnUALBERTA_SYS_STATUSReceive(this, ph,  pack);}
            public event UALBERTA_SYS_STATUSReceiveHandler OnUALBERTA_SYS_STATUSReceive;
            public delegate void UALBERTA_SYS_STATUSReceiveHandler(Channel src, Inside ph, UALBERTA_SYS_STATUS pack);
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
                    case 220:
                        if(pack == null) return new NAV_FILTER_BIAS();
                        OnNAV_FILTER_BIASReceive(this, ph, (NAV_FILTER_BIAS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 221:
                        if(pack == null) return new RADIO_CALIBRATION();
                        OnRADIO_CALIBRATIONReceive(this, ph, (RADIO_CALIBRATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 222:
                        if(pack == null) return new UALBERTA_SYS_STATUS();
                        OnUALBERTA_SYS_STATUSReceive(this, ph, (UALBERTA_SYS_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_GROUND_ROVER);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_ACTIVE);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
                Debug.Assert(pack.custom_mode == (uint)1447610708U);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_AUTOQUAD);
                Debug.Assert(pack.mavlink_version == (byte)(byte)30);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.custom_mode = (uint)1447610708U;
            p0.mavlink_version = (byte)(byte)30;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_AUTOQUAD;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_ACTIVE;
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_GROUND_ROVER;
            SMP_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)20797);
                Debug.Assert(pack.load == (ushort)(ushort)43109);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2);
                Debug.Assert(pack.current_battery == (short)(short) -28456);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)63913);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)60502);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)13123);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)57417);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)48801);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 62);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)36035);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.errors_count3 = (ushort)(ushort)60502;
            p1.current_battery = (short)(short) -28456;
            p1.battery_remaining = (sbyte)(sbyte) - 62;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR;
            p1.errors_count2 = (ushort)(ushort)48801;
            p1.errors_count4 = (ushort)(ushort)20797;
            p1.voltage_battery = (ushort)(ushort)57417;
            p1.errors_count1 = (ushort)(ushort)13123;
            p1.load = (ushort)(ushort)43109;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION;
            p1.errors_comm = (ushort)(ushort)36035;
            p1.drop_rate_comm = (ushort)(ushort)63913;
            SMP_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)3282673456245019476L);
                Debug.Assert(pack.time_boot_ms == (uint)356370911U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)3282673456245019476L;
            p2.time_boot_ms = (uint)356370911U;
            SMP_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1590095541U);
                Debug.Assert(pack.afz == (float) -5.3360276E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)62306);
                Debug.Assert(pack.vy == (float)9.478147E37F);
                Debug.Assert(pack.yaw_rate == (float) -8.876014E37F);
                Debug.Assert(pack.y == (float) -2.8162107E38F);
                Debug.Assert(pack.vz == (float)2.571477E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.yaw == (float)9.291782E37F);
                Debug.Assert(pack.z == (float)3.9216123E37F);
                Debug.Assert(pack.afx == (float) -2.9034772E38F);
                Debug.Assert(pack.x == (float) -1.2317701E38F);
                Debug.Assert(pack.afy == (float)2.903123E38F);
                Debug.Assert(pack.vx == (float) -2.6124086E38F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.x = (float) -1.2317701E38F;
            p3.vz = (float)2.571477E38F;
            p3.y = (float) -2.8162107E38F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p3.afy = (float)2.903123E38F;
            p3.z = (float)3.9216123E37F;
            p3.type_mask = (ushort)(ushort)62306;
            p3.yaw_rate = (float) -8.876014E37F;
            p3.vy = (float)9.478147E37F;
            p3.yaw = (float)9.291782E37F;
            p3.vx = (float) -2.6124086E38F;
            p3.time_boot_ms = (uint)1590095541U;
            p3.afx = (float) -2.9034772E38F;
            p3.afz = (float) -5.3360276E37F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)58);
                Debug.Assert(pack.time_usec == (ulong)7294092155966090483L);
                Debug.Assert(pack.target_component == (byte)(byte)225);
                Debug.Assert(pack.seq == (uint)2260632142U);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.target_component = (byte)(byte)225;
            p4.target_system = (byte)(byte)58;
            p4.time_usec = (ulong)7294092155966090483L;
            p4.seq = (uint)2260632142U;
            SMP_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (byte)(byte)98);
                Debug.Assert(pack.passkey_LEN(ph) == 2);
                Debug.Assert(pack.passkey_TRY(ph).Equals("me"));
                Debug.Assert(pack.target_system == (byte)(byte)250);
                Debug.Assert(pack.control_request == (byte)(byte)41);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.version = (byte)(byte)98;
            p5.target_system = (byte)(byte)250;
            p5.passkey_SET("me", PH) ;
            p5.control_request = (byte)(byte)41;
            SMP_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ack == (byte)(byte)235);
                Debug.Assert(pack.control_request == (byte)(byte)41);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)211);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.control_request = (byte)(byte)41;
            p6.gcs_system_id = (byte)(byte)211;
            p6.ack = (byte)(byte)235;
            SMP_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 14);
                Debug.Assert(pack.key_TRY(ph).Equals("anhYxoewqasixz"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("anhYxoewqasixz", PH) ;
            SMP_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)196);
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
                Debug.Assert(pack.custom_mode == (uint)924450889U);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.custom_mode = (uint)924450889U;
            p11.target_system = (byte)(byte)196;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            SMP_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)45);
                Debug.Assert(pack.param_index == (short)(short)25966);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("cttn"));
                Debug.Assert(pack.target_system == (byte)(byte)103);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_component = (byte)(byte)45;
            p20.param_id_SET("cttn", PH) ;
            p20.param_index = (short)(short)25966;
            p20.target_system = (byte)(byte)103;
            SMP_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)79);
                Debug.Assert(pack.target_system == (byte)(byte)118);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)79;
            p21.target_system = (byte)(byte)118;
            SMP_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("pq"));
                Debug.Assert(pack.param_value == (float)1.5561942E37F);
                Debug.Assert(pack.param_count == (ushort)(ushort)16971);
                Debug.Assert(pack.param_index == (ushort)(ushort)59056);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_index = (ushort)(ushort)59056;
            p22.param_count = (ushort)(ushort)16971;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32;
            p22.param_value = (float)1.5561942E37F;
            p22.param_id_SET("pq", PH) ;
            SMP_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("vlvuU"));
                Debug.Assert(pack.param_value == (float)2.023647E38F);
                Debug.Assert(pack.target_system == (byte)(byte)144);
                Debug.Assert(pack.target_component == (byte)(byte)30);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_value = (float)2.023647E38F;
            p23.target_component = (byte)(byte)30;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64;
            p23.target_system = (byte)(byte)144;
            p23.param_id_SET("vlvuU", PH) ;
            SMP_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel == (ushort)(ushort)57467);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -814765385);
                Debug.Assert(pack.alt == (int) -670218795);
                Debug.Assert(pack.time_usec == (ulong)3655546184398826024L);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)775182189U);
                Debug.Assert(pack.cog == (ushort)(ushort)17593);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)2737783379U);
                Debug.Assert(pack.epv == (ushort)(ushort)1626);
                Debug.Assert(pack.satellites_visible == (byte)(byte)172);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)3589075U);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)2392933221U);
                Debug.Assert(pack.lat == (int)1971529526);
                Debug.Assert(pack.lon == (int) -1373827600);
                Debug.Assert(pack.eph == (ushort)(ushort)27863);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.v_acc_SET((uint)775182189U, PH) ;
            p24.satellites_visible = (byte)(byte)172;
            p24.epv = (ushort)(ushort)1626;
            p24.h_acc_SET((uint)2392933221U, PH) ;
            p24.lat = (int)1971529526;
            p24.cog = (ushort)(ushort)17593;
            p24.alt_ellipsoid_SET((int) -814765385, PH) ;
            p24.hdg_acc_SET((uint)3589075U, PH) ;
            p24.eph = (ushort)(ushort)27863;
            p24.alt = (int) -670218795;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS;
            p24.vel = (ushort)(ushort)57467;
            p24.lon = (int) -1373827600;
            p24.vel_acc_SET((uint)2737783379U, PH) ;
            p24.time_usec = (ulong)3655546184398826024L;
            SMP_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)63, (byte)194, (byte)125, (byte)93, (byte)215, (byte)60, (byte)81, (byte)182, (byte)222, (byte)113, (byte)101, (byte)164, (byte)209, (byte)224, (byte)201, (byte)191, (byte)16, (byte)93, (byte)170, (byte)26}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)114, (byte)122, (byte)1, (byte)5, (byte)154, (byte)240, (byte)37, (byte)14, (byte)131, (byte)224, (byte)135, (byte)168, (byte)166, (byte)143, (byte)50, (byte)231, (byte)121, (byte)72, (byte)0, (byte)90}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)81, (byte)112, (byte)227, (byte)207, (byte)91, (byte)207, (byte)192, (byte)154, (byte)185, (byte)211, (byte)186, (byte)46, (byte)204, (byte)132, (byte)246, (byte)92, (byte)170, (byte)191, (byte)244, (byte)70}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)212, (byte)57, (byte)103, (byte)37, (byte)89, (byte)224, (byte)104, (byte)196, (byte)161, (byte)202, (byte)239, (byte)76, (byte)176, (byte)9, (byte)213, (byte)108, (byte)136, (byte)90, (byte)78, (byte)203}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)67);
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)253, (byte)179, (byte)23, (byte)240, (byte)236, (byte)205, (byte)237, (byte)84, (byte)28, (byte)118, (byte)194, (byte)6, (byte)143, (byte)154, (byte)160, (byte)179, (byte)50, (byte)47, (byte)120, (byte)208}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellites_visible = (byte)(byte)67;
            p25.satellite_snr_SET(new byte[] {(byte)81, (byte)112, (byte)227, (byte)207, (byte)91, (byte)207, (byte)192, (byte)154, (byte)185, (byte)211, (byte)186, (byte)46, (byte)204, (byte)132, (byte)246, (byte)92, (byte)170, (byte)191, (byte)244, (byte)70}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)114, (byte)122, (byte)1, (byte)5, (byte)154, (byte)240, (byte)37, (byte)14, (byte)131, (byte)224, (byte)135, (byte)168, (byte)166, (byte)143, (byte)50, (byte)231, (byte)121, (byte)72, (byte)0, (byte)90}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)212, (byte)57, (byte)103, (byte)37, (byte)89, (byte)224, (byte)104, (byte)196, (byte)161, (byte)202, (byte)239, (byte)76, (byte)176, (byte)9, (byte)213, (byte)108, (byte)136, (byte)90, (byte)78, (byte)203}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)253, (byte)179, (byte)23, (byte)240, (byte)236, (byte)205, (byte)237, (byte)84, (byte)28, (byte)118, (byte)194, (byte)6, (byte)143, (byte)154, (byte)160, (byte)179, (byte)50, (byte)47, (byte)120, (byte)208}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)63, (byte)194, (byte)125, (byte)93, (byte)215, (byte)60, (byte)81, (byte)182, (byte)222, (byte)113, (byte)101, (byte)164, (byte)209, (byte)224, (byte)201, (byte)191, (byte)16, (byte)93, (byte)170, (byte)26}, 0) ;
            SMP_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ymag == (short)(short)9065);
                Debug.Assert(pack.ygyro == (short)(short) -23389);
                Debug.Assert(pack.xgyro == (short)(short) -16485);
                Debug.Assert(pack.xacc == (short)(short)1106);
                Debug.Assert(pack.yacc == (short)(short) -29612);
                Debug.Assert(pack.zacc == (short)(short)30102);
                Debug.Assert(pack.xmag == (short)(short) -13455);
                Debug.Assert(pack.zmag == (short)(short)26883);
                Debug.Assert(pack.zgyro == (short)(short) -4706);
                Debug.Assert(pack.time_boot_ms == (uint)4066214046U);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.xacc = (short)(short)1106;
            p26.zgyro = (short)(short) -4706;
            p26.xmag = (short)(short) -13455;
            p26.yacc = (short)(short) -29612;
            p26.xgyro = (short)(short) -16485;
            p26.ygyro = (short)(short) -23389;
            p26.ymag = (short)(short)9065;
            p26.zacc = (short)(short)30102;
            p26.time_boot_ms = (uint)4066214046U;
            p26.zmag = (short)(short)26883;
            SMP_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short) -20321);
                Debug.Assert(pack.xacc == (short)(short) -30514);
                Debug.Assert(pack.zacc == (short)(short)93);
                Debug.Assert(pack.xmag == (short)(short)29549);
                Debug.Assert(pack.time_usec == (ulong)3837360507303989507L);
                Debug.Assert(pack.yacc == (short)(short)16152);
                Debug.Assert(pack.xgyro == (short)(short) -7930);
                Debug.Assert(pack.zmag == (short)(short) -14562);
                Debug.Assert(pack.zgyro == (short)(short)13252);
                Debug.Assert(pack.ymag == (short)(short) -11865);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.xmag = (short)(short)29549;
            p27.time_usec = (ulong)3837360507303989507L;
            p27.ymag = (short)(short) -11865;
            p27.ygyro = (short)(short) -20321;
            p27.zacc = (short)(short)93;
            p27.zmag = (short)(short) -14562;
            p27.xacc = (short)(short) -30514;
            p27.yacc = (short)(short)16152;
            p27.xgyro = (short)(short) -7930;
            p27.zgyro = (short)(short)13252;
            SMP_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff1 == (short)(short)13714);
                Debug.Assert(pack.press_diff2 == (short)(short) -1040);
                Debug.Assert(pack.time_usec == (ulong)5583796650611192077L);
                Debug.Assert(pack.temperature == (short)(short)23262);
                Debug.Assert(pack.press_abs == (short)(short) -31951);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff2 = (short)(short) -1040;
            p28.time_usec = (ulong)5583796650611192077L;
            p28.temperature = (short)(short)23262;
            p28.press_diff1 = (short)(short)13714;
            p28.press_abs = (short)(short) -31951;
            SMP_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1056482411U);
                Debug.Assert(pack.press_abs == (float)1.0836729E38F);
                Debug.Assert(pack.temperature == (short)(short) -28938);
                Debug.Assert(pack.press_diff == (float) -2.7071788E38F);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_diff = (float) -2.7071788E38F;
            p29.time_boot_ms = (uint)1056482411U;
            p29.press_abs = (float)1.0836729E38F;
            p29.temperature = (short)(short) -28938;
            SMP_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)1.6712463E38F);
                Debug.Assert(pack.pitchspeed == (float)2.0192143E38F);
                Debug.Assert(pack.yawspeed == (float) -1.9885541E37F);
                Debug.Assert(pack.pitch == (float)2.8736495E38F);
                Debug.Assert(pack.roll == (float) -3.395478E38F);
                Debug.Assert(pack.yaw == (float) -1.1042469E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1307998105U);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.pitchspeed = (float)2.0192143E38F;
            p30.pitch = (float)2.8736495E38F;
            p30.yawspeed = (float) -1.9885541E37F;
            p30.rollspeed = (float)1.6712463E38F;
            p30.roll = (float) -3.395478E38F;
            p30.time_boot_ms = (uint)1307998105U;
            p30.yaw = (float) -1.1042469E38F;
            SMP_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)1.1780665E38F);
                Debug.Assert(pack.yawspeed == (float)3.0849227E38F);
                Debug.Assert(pack.q1 == (float) -2.3404268E38F);
                Debug.Assert(pack.q2 == (float)2.4836802E38F);
                Debug.Assert(pack.q4 == (float)3.4010512E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1206484781U);
                Debug.Assert(pack.pitchspeed == (float) -1.4889164E38F);
                Debug.Assert(pack.q3 == (float) -7.044862E37F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.rollspeed = (float)1.1780665E38F;
            p31.q3 = (float) -7.044862E37F;
            p31.q1 = (float) -2.3404268E38F;
            p31.q4 = (float)3.4010512E38F;
            p31.q2 = (float)2.4836802E38F;
            p31.pitchspeed = (float) -1.4889164E38F;
            p31.yawspeed = (float)3.0849227E38F;
            p31.time_boot_ms = (uint)1206484781U;
            SMP_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -2.4256651E37F);
                Debug.Assert(pack.y == (float) -1.3167338E38F);
                Debug.Assert(pack.x == (float)3.33043E38F);
                Debug.Assert(pack.vz == (float) -1.1233896E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1168144789U);
                Debug.Assert(pack.vx == (float) -2.1631135E38F);
                Debug.Assert(pack.z == (float)2.7026106E38F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.y = (float) -1.3167338E38F;
            p32.vx = (float) -2.1631135E38F;
            p32.time_boot_ms = (uint)1168144789U;
            p32.x = (float)3.33043E38F;
            p32.z = (float)2.7026106E38F;
            p32.vy = (float) -2.4256651E37F;
            p32.vz = (float) -1.1233896E38F;
            SMP_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)711187185U);
                Debug.Assert(pack.lat == (int)761136052);
                Debug.Assert(pack.relative_alt == (int)966897060);
                Debug.Assert(pack.vx == (short)(short) -24872);
                Debug.Assert(pack.hdg == (ushort)(ushort)44758);
                Debug.Assert(pack.vz == (short)(short) -29099);
                Debug.Assert(pack.alt == (int)1519488333);
                Debug.Assert(pack.vy == (short)(short) -32258);
                Debug.Assert(pack.lon == (int) -1870362272);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.lon = (int) -1870362272;
            p33.hdg = (ushort)(ushort)44758;
            p33.alt = (int)1519488333;
            p33.time_boot_ms = (uint)711187185U;
            p33.vy = (short)(short) -32258;
            p33.lat = (int)761136052;
            p33.vx = (short)(short) -24872;
            p33.vz = (short)(short) -29099;
            p33.relative_alt = (int)966897060;
            SMP_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_scaled == (short)(short) -25327);
                Debug.Assert(pack.chan1_scaled == (short)(short)11729);
                Debug.Assert(pack.chan5_scaled == (short)(short) -17556);
                Debug.Assert(pack.port == (byte)(byte)127);
                Debug.Assert(pack.chan8_scaled == (short)(short)18846);
                Debug.Assert(pack.chan4_scaled == (short)(short)12792);
                Debug.Assert(pack.chan6_scaled == (short)(short)3044);
                Debug.Assert(pack.chan2_scaled == (short)(short)26322);
                Debug.Assert(pack.time_boot_ms == (uint)544520128U);
                Debug.Assert(pack.chan3_scaled == (short)(short)2534);
                Debug.Assert(pack.rssi == (byte)(byte)183);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.port = (byte)(byte)127;
            p34.chan3_scaled = (short)(short)2534;
            p34.time_boot_ms = (uint)544520128U;
            p34.chan4_scaled = (short)(short)12792;
            p34.chan6_scaled = (short)(short)3044;
            p34.chan5_scaled = (short)(short) -17556;
            p34.chan8_scaled = (short)(short)18846;
            p34.chan2_scaled = (short)(short)26322;
            p34.chan1_scaled = (short)(short)11729;
            p34.rssi = (byte)(byte)183;
            p34.chan7_scaled = (short)(short) -25327;
            SMP_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)1311);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)6178);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)51575);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)38451);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)64433);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)56474);
                Debug.Assert(pack.port == (byte)(byte)162);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)277);
                Debug.Assert(pack.rssi == (byte)(byte)224);
                Debug.Assert(pack.time_boot_ms == (uint)500116093U);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)52592);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan1_raw = (ushort)(ushort)6178;
            p35.chan7_raw = (ushort)(ushort)277;
            p35.chan8_raw = (ushort)(ushort)52592;
            p35.chan3_raw = (ushort)(ushort)1311;
            p35.chan5_raw = (ushort)(ushort)38451;
            p35.rssi = (byte)(byte)224;
            p35.time_boot_ms = (uint)500116093U;
            p35.chan4_raw = (ushort)(ushort)64433;
            p35.chan6_raw = (ushort)(ushort)56474;
            p35.port = (byte)(byte)162;
            p35.chan2_raw = (ushort)(ushort)51575;
            SMP_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.port == (byte)(byte)63);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)6574);
                Debug.Assert(pack.time_usec == (uint)3313857496U);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)29660);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)23010);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)43544);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)59614);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)23931);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)26703);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)18000);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)26631);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)25336);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)35011);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)22511);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)32298);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)26477);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)59441);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)53455);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo16_raw_SET((ushort)(ushort)43544, PH) ;
            p36.port = (byte)(byte)63;
            p36.time_usec = (uint)3313857496U;
            p36.servo4_raw = (ushort)(ushort)25336;
            p36.servo7_raw = (ushort)(ushort)29660;
            p36.servo10_raw_SET((ushort)(ushort)35011, PH) ;
            p36.servo8_raw = (ushort)(ushort)26631;
            p36.servo5_raw = (ushort)(ushort)22511;
            p36.servo6_raw = (ushort)(ushort)53455;
            p36.servo15_raw_SET((ushort)(ushort)23010, PH) ;
            p36.servo2_raw = (ushort)(ushort)32298;
            p36.servo11_raw_SET((ushort)(ushort)59614, PH) ;
            p36.servo1_raw = (ushort)(ushort)23931;
            p36.servo3_raw = (ushort)(ushort)18000;
            p36.servo13_raw_SET((ushort)(ushort)59441, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)6574, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)26703, PH) ;
            p36.servo14_raw_SET((ushort)(ushort)26477, PH) ;
            SMP_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short)7874);
                Debug.Assert(pack.target_component == (byte)(byte)25);
                Debug.Assert(pack.target_system == (byte)(byte)197);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.end_index == (short)(short)23413);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_component = (byte)(byte)25;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p37.end_index = (short)(short)23413;
            p37.start_index = (short)(short)7874;
            p37.target_system = (byte)(byte)197;
            SMP_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short) -5679);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)26);
                Debug.Assert(pack.end_index == (short)(short)3115);
                Debug.Assert(pack.target_system == (byte)(byte)162);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.start_index = (short)(short) -5679;
            p38.target_component = (byte)(byte)26;
            p38.target_system = (byte)(byte)162;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p38.end_index = (short)(short)3115;
            SMP_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param4 == (float)2.5032864E38F);
                Debug.Assert(pack.z == (float) -1.6163365E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)178);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.x == (float)3.2415418E38F);
                Debug.Assert(pack.target_component == (byte)(byte)15);
                Debug.Assert(pack.param1 == (float)1.7046582E37F);
                Debug.Assert(pack.param3 == (float)3.2732483E38F);
                Debug.Assert(pack.target_system == (byte)(byte)90);
                Debug.Assert(pack.y == (float) -6.5647675E37F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE);
                Debug.Assert(pack.param2 == (float)2.2147359E38F);
                Debug.Assert(pack.current == (byte)(byte)62);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.seq == (ushort)(ushort)10206);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
            p39.seq = (ushort)(ushort)10206;
            p39.target_system = (byte)(byte)90;
            p39.x = (float)3.2415418E38F;
            p39.param4 = (float)2.5032864E38F;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p39.param1 = (float)1.7046582E37F;
            p39.z = (float) -1.6163365E38F;
            p39.param2 = (float)2.2147359E38F;
            p39.y = (float) -6.5647675E37F;
            p39.autocontinue = (byte)(byte)178;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p39.current = (byte)(byte)62;
            p39.target_component = (byte)(byte)15;
            p39.param3 = (float)3.2732483E38F;
            SMP_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)128);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.seq == (ushort)(ushort)59401);
                Debug.Assert(pack.target_system == (byte)(byte)60);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_system = (byte)(byte)60;
            p40.seq = (ushort)(ushort)59401;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p40.target_component = (byte)(byte)128;
            SMP_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)3);
                Debug.Assert(pack.seq == (ushort)(ushort)54316);
                Debug.Assert(pack.target_component == (byte)(byte)174);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.seq = (ushort)(ushort)54316;
            p41.target_component = (byte)(byte)174;
            p41.target_system = (byte)(byte)3;
            SMP_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)11193);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)11193;
            SMP_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)21);
                Debug.Assert(pack.target_system == (byte)(byte)38);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p43.target_component = (byte)(byte)21;
            p43.target_system = (byte)(byte)38;
            SMP_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)236);
                Debug.Assert(pack.target_system == (byte)(byte)251);
                Debug.Assert(pack.count == (ushort)(ushort)54500);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.count = (ushort)(ushort)54500;
            p44.target_component = (byte)(byte)236;
            p44.target_system = (byte)(byte)251;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            SMP_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)72);
                Debug.Assert(pack.target_system == (byte)(byte)126);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_component = (byte)(byte)72;
            p45.target_system = (byte)(byte)126;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            SMP_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)36383);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)36383;
            SMP_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)35);
                Debug.Assert(pack.target_component == (byte)(byte)157);
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p47.target_component = (byte)(byte)157;
            p47.target_system = (byte)(byte)35;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE;
            SMP_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)241);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)9022450341945973386L);
                Debug.Assert(pack.longitude == (int)1181621574);
                Debug.Assert(pack.latitude == (int) -1802532101);
                Debug.Assert(pack.altitude == (int) -177821563);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.latitude = (int) -1802532101;
            p48.longitude = (int)1181621574;
            p48.target_system = (byte)(byte)241;
            p48.time_usec_SET((ulong)9022450341945973386L, PH) ;
            p48.altitude = (int) -177821563;
            SMP_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int) -318298944);
                Debug.Assert(pack.latitude == (int) -868969259);
                Debug.Assert(pack.longitude == (int) -1277345897);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7745829134561757963L);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.longitude = (int) -1277345897;
            p49.time_usec_SET((ulong)7745829134561757963L, PH) ;
            p49.latitude = (int) -868969259;
            p49.altitude = (int) -318298944;
            SMP_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)251);
                Debug.Assert(pack.scale == (float)1.0655231E38F);
                Debug.Assert(pack.param_value0 == (float)1.9649811E37F);
                Debug.Assert(pack.param_value_min == (float)8.770537E37F);
                Debug.Assert(pack.target_component == (byte)(byte)38);
                Debug.Assert(pack.param_value_max == (float)1.3695854E38F);
                Debug.Assert(pack.param_index == (short)(short)2500);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)138);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("sbegCmcrTvv"));
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_system = (byte)(byte)251;
            p50.param_index = (short)(short)2500;
            p50.param_value0 = (float)1.9649811E37F;
            p50.parameter_rc_channel_index = (byte)(byte)138;
            p50.param_value_max = (float)1.3695854E38F;
            p50.param_value_min = (float)8.770537E37F;
            p50.param_id_SET("sbegCmcrTvv", PH) ;
            p50.scale = (float)1.0655231E38F;
            p50.target_component = (byte)(byte)38;
            SMP_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)158);
                Debug.Assert(pack.seq == (ushort)(ushort)26229);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_system == (byte)(byte)15);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_component = (byte)(byte)158;
            p51.seq = (ushort)(ushort)26229;
            p51.target_system = (byte)(byte)15;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            SMP_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2x == (float) -3.1974097E38F);
                Debug.Assert(pack.target_system == (byte)(byte)62);
                Debug.Assert(pack.p2y == (float)2.3764492E38F);
                Debug.Assert(pack.p1z == (float) -1.3734855E38F);
                Debug.Assert(pack.p2z == (float)1.3354298E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.p1x == (float) -1.196567E38F);
                Debug.Assert(pack.p1y == (float) -1.3990955E38F);
                Debug.Assert(pack.target_component == (byte)(byte)119);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1x = (float) -1.196567E38F;
            p54.p2z = (float)1.3354298E38F;
            p54.p1y = (float) -1.3990955E38F;
            p54.p1z = (float) -1.3734855E38F;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p54.target_component = (byte)(byte)119;
            p54.target_system = (byte)(byte)62;
            p54.p2y = (float)2.3764492E38F;
            p54.p2x = (float) -3.1974097E38F;
            SMP_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2z == (float) -2.18547E38F);
                Debug.Assert(pack.p2x == (float)2.094695E38F);
                Debug.Assert(pack.p1y == (float) -1.0624801E38F);
                Debug.Assert(pack.p1x == (float)3.0500677E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.p1z == (float) -1.1246413E38F);
                Debug.Assert(pack.p2y == (float)2.183211E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p1y = (float) -1.0624801E38F;
            p55.p2x = (float)2.094695E38F;
            p55.p2y = (float)2.183211E38F;
            p55.p1x = (float)3.0500677E38F;
            p55.p2z = (float) -2.18547E38F;
            p55.p1z = (float) -1.1246413E38F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            SMP_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float) -6.0726493E37F);
                Debug.Assert(pack.yawspeed == (float)2.113177E38F);
                Debug.Assert(pack.rollspeed == (float) -3.2366525E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-8.581015E37F, 9.260881E37F, 7.0352626E37F, 4.520694E37F}));
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.4057783E38F, -6.279624E37F, -2.2943362E38F, 9.749254E37F, -8.249496E36F, 1.1666969E37F, 1.7866237E38F, -2.1450078E38F, 2.4418632E38F}));
                Debug.Assert(pack.time_usec == (ulong)7866869756611398723L);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.pitchspeed = (float) -6.0726493E37F;
            p61.time_usec = (ulong)7866869756611398723L;
            p61.yawspeed = (float)2.113177E38F;
            p61.rollspeed = (float) -3.2366525E38F;
            p61.covariance_SET(new float[] {2.4057783E38F, -6.279624E37F, -2.2943362E38F, 9.749254E37F, -8.249496E36F, 1.1666969E37F, 1.7866237E38F, -2.1450078E38F, 2.4418632E38F}, 0) ;
            p61.q_SET(new float[] {-8.581015E37F, 9.260881E37F, 7.0352626E37F, 4.520694E37F}, 0) ;
            SMP_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_bearing == (short)(short) -1017);
                Debug.Assert(pack.nav_roll == (float) -5.9925246E37F);
                Debug.Assert(pack.nav_pitch == (float) -1.2734581E38F);
                Debug.Assert(pack.xtrack_error == (float)2.214745E37F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)44299);
                Debug.Assert(pack.nav_bearing == (short)(short)19534);
                Debug.Assert(pack.alt_error == (float) -1.201948E38F);
                Debug.Assert(pack.aspd_error == (float) -1.2134587E38F);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_roll = (float) -5.9925246E37F;
            p62.alt_error = (float) -1.201948E38F;
            p62.xtrack_error = (float)2.214745E37F;
            p62.wp_dist = (ushort)(ushort)44299;
            p62.aspd_error = (float) -1.2134587E38F;
            p62.nav_bearing = (short)(short)19534;
            p62.nav_pitch = (float) -1.2734581E38F;
            p62.target_bearing = (short)(short) -1017;
            SMP_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float) -2.7196819E38F);
                Debug.Assert(pack.lon == (int) -418967548);
                Debug.Assert(pack.relative_alt == (int) -1425637785);
                Debug.Assert(pack.time_usec == (ulong)5201832591092099585L);
                Debug.Assert(pack.vx == (float) -2.526897E38F);
                Debug.Assert(pack.lat == (int)776759505);
                Debug.Assert(pack.alt == (int)695337938);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {3.043559E38F, 2.4441253E38F, 1.0324653E38F, 7.113241E37F, -3.3532385E38F, -1.2443237E38F, 2.3235958E38F, -2.7234315E38F, 1.3302099E38F, 1.6031137E38F, -2.737334E38F, 2.3148707E37F, 1.8829047E38F, 3.133381E38F, -2.7188698E38F, -2.771384E38F, 1.6371742E38F, -2.482878E38F, 1.0400299E38F, 2.8779553E38F, 1.5510894E38F, -3.1552712E38F, 1.3794898E37F, 2.0063187E38F, 1.6903762E38F, 1.5451612E37F, 2.4884752E38F, -6.501213E37F, -7.2385187E37F, 1.0203351E38F, -5.002061E37F, -1.4480546E38F, 2.2536566E38F, -5.866731E37F, 8.639557E37F, -1.8345117E38F}));
                Debug.Assert(pack.vy == (float)3.2419414E38F);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.lat = (int)776759505;
            p63.covariance_SET(new float[] {3.043559E38F, 2.4441253E38F, 1.0324653E38F, 7.113241E37F, -3.3532385E38F, -1.2443237E38F, 2.3235958E38F, -2.7234315E38F, 1.3302099E38F, 1.6031137E38F, -2.737334E38F, 2.3148707E37F, 1.8829047E38F, 3.133381E38F, -2.7188698E38F, -2.771384E38F, 1.6371742E38F, -2.482878E38F, 1.0400299E38F, 2.8779553E38F, 1.5510894E38F, -3.1552712E38F, 1.3794898E37F, 2.0063187E38F, 1.6903762E38F, 1.5451612E37F, 2.4884752E38F, -6.501213E37F, -7.2385187E37F, 1.0203351E38F, -5.002061E37F, -1.4480546E38F, 2.2536566E38F, -5.866731E37F, 8.639557E37F, -1.8345117E38F}, 0) ;
            p63.time_usec = (ulong)5201832591092099585L;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            p63.lon = (int) -418967548;
            p63.alt = (int)695337938;
            p63.relative_alt = (int) -1425637785;
            p63.vx = (float) -2.526897E38F;
            p63.vz = (float) -2.7196819E38F;
            p63.vy = (float)3.2419414E38F;
            SMP_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)1.2276247E38F);
                Debug.Assert(pack.ax == (float) -2.9505606E38F);
                Debug.Assert(pack.az == (float)7.5281505E37F);
                Debug.Assert(pack.z == (float) -2.7022047E38F);
                Debug.Assert(pack.ay == (float)1.6814164E38F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
                Debug.Assert(pack.vy == (float)1.7796537E38F);
                Debug.Assert(pack.y == (float) -9.902663E37F);
                Debug.Assert(pack.time_usec == (ulong)1277713521057631660L);
                Debug.Assert(pack.vx == (float)2.3329823E38F);
                Debug.Assert(pack.vz == (float) -1.2109827E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {6.718974E37F, 3.155803E38F, -2.6464212E38F, 1.0456564E38F, 3.1035362E38F, -2.138967E38F, 1.1835769E38F, 1.8353974E38F, -9.803083E36F, -2.528154E38F, 1.7803536E38F, -1.6506199E38F, 2.7618544E37F, 2.6569403E38F, -2.1286318E38F, 2.2456994E38F, -3.1909137E38F, 2.3531147E38F, -8.741914E37F, -1.7850703E38F, -2.1622803E38F, -6.0250135E37F, -3.1751264E38F, -2.4222525E38F, 3.2422949E38F, -4.827665E37F, 4.1040956E37F, -3.3502598E38F, -1.8456086E38F, -2.972985E38F, 2.8022698E38F, 2.8238546E38F, -1.8667301E38F, -7.226956E35F, -1.9703426E38F, 1.1162124E38F, -3.3790718E38F, -2.2031124E38F, -1.6454982E38F, -2.469409E38F, 1.6197836E38F, -2.011376E38F, -3.3871281E38F, 2.172639E38F, -1.8027894E38F}));
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            p64.z = (float) -2.7022047E38F;
            p64.ax = (float) -2.9505606E38F;
            p64.y = (float) -9.902663E37F;
            p64.covariance_SET(new float[] {6.718974E37F, 3.155803E38F, -2.6464212E38F, 1.0456564E38F, 3.1035362E38F, -2.138967E38F, 1.1835769E38F, 1.8353974E38F, -9.803083E36F, -2.528154E38F, 1.7803536E38F, -1.6506199E38F, 2.7618544E37F, 2.6569403E38F, -2.1286318E38F, 2.2456994E38F, -3.1909137E38F, 2.3531147E38F, -8.741914E37F, -1.7850703E38F, -2.1622803E38F, -6.0250135E37F, -3.1751264E38F, -2.4222525E38F, 3.2422949E38F, -4.827665E37F, 4.1040956E37F, -3.3502598E38F, -1.8456086E38F, -2.972985E38F, 2.8022698E38F, 2.8238546E38F, -1.8667301E38F, -7.226956E35F, -1.9703426E38F, 1.1162124E38F, -3.3790718E38F, -2.2031124E38F, -1.6454982E38F, -2.469409E38F, 1.6197836E38F, -2.011376E38F, -3.3871281E38F, 2.172639E38F, -1.8027894E38F}, 0) ;
            p64.ay = (float)1.6814164E38F;
            p64.vz = (float) -1.2109827E38F;
            p64.az = (float)7.5281505E37F;
            p64.vy = (float)1.7796537E38F;
            p64.vx = (float)2.3329823E38F;
            p64.x = (float)1.2276247E38F;
            p64.time_usec = (ulong)1277713521057631660L;
            SMP_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)52230);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)36785);
                Debug.Assert(pack.rssi == (byte)(byte)119);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)12773);
                Debug.Assert(pack.time_boot_ms == (uint)4148235876U);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)20301);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)57636);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)23055);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)9324);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)27524);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)37109);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)64570);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)18811);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)46024);
                Debug.Assert(pack.chancount == (byte)(byte)103);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)59939);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)28842);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)24074);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)33202);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)19055);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)64420);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chancount = (byte)(byte)103;
            p65.chan12_raw = (ushort)(ushort)46024;
            p65.chan1_raw = (ushort)(ushort)24074;
            p65.chan3_raw = (ushort)(ushort)33202;
            p65.chan18_raw = (ushort)(ushort)57636;
            p65.chan4_raw = (ushort)(ushort)9324;
            p65.chan7_raw = (ushort)(ushort)36785;
            p65.chan2_raw = (ushort)(ushort)64420;
            p65.time_boot_ms = (uint)4148235876U;
            p65.chan11_raw = (ushort)(ushort)52230;
            p65.chan14_raw = (ushort)(ushort)27524;
            p65.chan15_raw = (ushort)(ushort)18811;
            p65.chan16_raw = (ushort)(ushort)28842;
            p65.chan6_raw = (ushort)(ushort)37109;
            p65.rssi = (byte)(byte)119;
            p65.chan9_raw = (ushort)(ushort)20301;
            p65.chan17_raw = (ushort)(ushort)59939;
            p65.chan10_raw = (ushort)(ushort)23055;
            p65.chan5_raw = (ushort)(ushort)19055;
            p65.chan13_raw = (ushort)(ushort)12773;
            p65.chan8_raw = (ushort)(ushort)64570;
            SMP_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.req_stream_id == (byte)(byte)94);
                Debug.Assert(pack.target_system == (byte)(byte)79);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)54455);
                Debug.Assert(pack.target_component == (byte)(byte)107);
                Debug.Assert(pack.start_stop == (byte)(byte)55);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.req_stream_id = (byte)(byte)94;
            p66.target_system = (byte)(byte)79;
            p66.req_message_rate = (ushort)(ushort)54455;
            p66.start_stop = (byte)(byte)55;
            p66.target_component = (byte)(byte)107;
            SMP_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)43235);
                Debug.Assert(pack.stream_id == (byte)(byte)169);
                Debug.Assert(pack.on_off == (byte)(byte)252);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.on_off = (byte)(byte)252;
            p67.stream_id = (byte)(byte)169;
            p67.message_rate = (ushort)(ushort)43235;
            SMP_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (short)(short)31992);
                Debug.Assert(pack.target == (byte)(byte)151);
                Debug.Assert(pack.x == (short)(short) -188);
                Debug.Assert(pack.y == (short)(short) -6018);
                Debug.Assert(pack.r == (short)(short) -15617);
                Debug.Assert(pack.buttons == (ushort)(ushort)895);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.target = (byte)(byte)151;
            p69.x = (short)(short) -188;
            p69.y = (short)(short) -6018;
            p69.z = (short)(short)31992;
            p69.buttons = (ushort)(ushort)895;
            p69.r = (short)(short) -15617;
            SMP_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)24062);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)20922);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)16916);
                Debug.Assert(pack.target_system == (byte)(byte)7);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)63627);
                Debug.Assert(pack.target_component == (byte)(byte)79);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)35597);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)40058);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)57661);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)59928);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.target_component = (byte)(byte)79;
            p70.target_system = (byte)(byte)7;
            p70.chan3_raw = (ushort)(ushort)24062;
            p70.chan2_raw = (ushort)(ushort)57661;
            p70.chan1_raw = (ushort)(ushort)40058;
            p70.chan6_raw = (ushort)(ushort)20922;
            p70.chan8_raw = (ushort)(ushort)35597;
            p70.chan7_raw = (ushort)(ushort)16916;
            p70.chan4_raw = (ushort)(ushort)59928;
            p70.chan5_raw = (ushort)(ushort)63627;
            SMP_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current == (byte)(byte)148);
                Debug.Assert(pack.param2 == (float) -1.9881412E38F);
                Debug.Assert(pack.param3 == (float)1.0337737E38F);
                Debug.Assert(pack.y == (int)476014084);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.autocontinue == (byte)(byte)32);
                Debug.Assert(pack.param1 == (float)2.0574643E38F);
                Debug.Assert(pack.x == (int)1980726139);
                Debug.Assert(pack.target_component == (byte)(byte)85);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS);
                Debug.Assert(pack.z == (float) -1.9948772E38F);
                Debug.Assert(pack.target_system == (byte)(byte)176);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.seq == (ushort)(ushort)31358);
                Debug.Assert(pack.param4 == (float)3.525555E37F);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.z = (float) -1.9948772E38F;
            p73.param3 = (float)1.0337737E38F;
            p73.autocontinue = (byte)(byte)32;
            p73.current = (byte)(byte)148;
            p73.param1 = (float)2.0574643E38F;
            p73.target_component = (byte)(byte)85;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS;
            p73.param2 = (float) -1.9881412E38F;
            p73.param4 = (float)3.525555E37F;
            p73.seq = (ushort)(ushort)31358;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p73.y = (int)476014084;
            p73.target_system = (byte)(byte)176;
            p73.x = (int)1980726139;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            SMP_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float) -3.2506818E37F);
                Debug.Assert(pack.airspeed == (float) -1.8952358E38F);
                Debug.Assert(pack.climb == (float)7.057511E37F);
                Debug.Assert(pack.heading == (short)(short)24858);
                Debug.Assert(pack.throttle == (ushort)(ushort)55289);
                Debug.Assert(pack.groundspeed == (float)2.1881268E38F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.alt = (float) -3.2506818E37F;
            p74.groundspeed = (float)2.1881268E38F;
            p74.throttle = (ushort)(ushort)55289;
            p74.climb = (float)7.057511E37F;
            p74.airspeed = (float) -1.8952358E38F;
            p74.heading = (short)(short)24858;
            SMP_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)41);
                Debug.Assert(pack.param4 == (float) -3.0842815E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_ROI);
                Debug.Assert(pack.param3 == (float)1.6879436E38F);
                Debug.Assert(pack.param2 == (float) -2.0988448E37F);
                Debug.Assert(pack.current == (byte)(byte)178);
                Debug.Assert(pack.y == (int)686005827);
                Debug.Assert(pack.param1 == (float)1.9794E38F);
                Debug.Assert(pack.z == (float)2.7164256E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)154);
                Debug.Assert(pack.x == (int)855846712);
                Debug.Assert(pack.target_component == (byte)(byte)95);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION);
            };
            GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.param1 = (float)1.9794E38F;
            p75.y = (int)686005827;
            p75.autocontinue = (byte)(byte)154;
            p75.param2 = (float) -2.0988448E37F;
            p75.z = (float)2.7164256E38F;
            p75.param3 = (float)1.6879436E38F;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_ROI;
            p75.x = (int)855846712;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p75.target_system = (byte)(byte)41;
            p75.target_component = (byte)(byte)95;
            p75.param4 = (float) -3.0842815E38F;
            p75.current = (byte)(byte)178;
            CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)129);
                Debug.Assert(pack.param6 == (float) -1.3994524E38F);
                Debug.Assert(pack.param7 == (float)1.1627384E38F);
                Debug.Assert(pack.param5 == (float) -2.2182938E38F);
                Debug.Assert(pack.param1 == (float)7.40951E36F);
                Debug.Assert(pack.param2 == (float) -2.4334677E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)88);
                Debug.Assert(pack.param3 == (float)2.760073E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_JUMP);
                Debug.Assert(pack.param4 == (float)2.3729247E38F);
                Debug.Assert(pack.target_component == (byte)(byte)76);
            };
            GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.param6 = (float) -1.3994524E38F;
            p76.param7 = (float)1.1627384E38F;
            p76.target_component = (byte)(byte)76;
            p76.param3 = (float)2.760073E38F;
            p76.param2 = (float) -2.4334677E38F;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_JUMP;
            p76.confirmation = (byte)(byte)88;
            p76.target_system = (byte)(byte)129;
            p76.param1 = (float)7.40951E36F;
            p76.param4 = (float)2.3729247E38F;
            p76.param5 = (float) -2.2182938E38F;
            CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result_param2_TRY(ph) == (int)1618140505);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)49);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)177);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_UNSUPPORTED);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)194);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_CONDITION_DELAY);
            };
            GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_CONDITION_DELAY;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_UNSUPPORTED;
            p77.target_component_SET((byte)(byte)177, PH) ;
            p77.progress_SET((byte)(byte)49, PH) ;
            p77.result_param2_SET((int)1618140505, PH) ;
            p77.target_system_SET((byte)(byte)194, PH) ;
            CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3119348303U);
                Debug.Assert(pack.yaw == (float) -1.9224009E38F);
                Debug.Assert(pack.pitch == (float) -6.2817376E37F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)215);
                Debug.Assert(pack.thrust == (float)3.325358E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)92);
                Debug.Assert(pack.roll == (float)2.3213086E37F);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.manual_override_switch = (byte)(byte)215;
            p81.yaw = (float) -1.9224009E38F;
            p81.mode_switch = (byte)(byte)92;
            p81.thrust = (float)3.325358E38F;
            p81.time_boot_ms = (uint)3119348303U;
            p81.pitch = (float) -6.2817376E37F;
            p81.roll = (float)2.3213086E37F;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)80);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.3693989E38F, -8.2302874E37F, -2.2711206E38F, -1.4393526E38F}));
                Debug.Assert(pack.body_yaw_rate == (float) -3.0741659E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3798968124U);
                Debug.Assert(pack.target_component == (byte)(byte)96);
                Debug.Assert(pack.type_mask == (byte)(byte)114);
                Debug.Assert(pack.thrust == (float)2.3511582E38F);
                Debug.Assert(pack.body_pitch_rate == (float)2.9159527E38F);
                Debug.Assert(pack.body_roll_rate == (float) -2.1326705E37F);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.thrust = (float)2.3511582E38F;
            p82.body_pitch_rate = (float)2.9159527E38F;
            p82.type_mask = (byte)(byte)114;
            p82.target_system = (byte)(byte)80;
            p82.body_yaw_rate = (float) -3.0741659E38F;
            p82.time_boot_ms = (uint)3798968124U;
            p82.body_roll_rate = (float) -2.1326705E37F;
            p82.target_component = (byte)(byte)96;
            p82.q_SET(new float[] {1.3693989E38F, -8.2302874E37F, -2.2711206E38F, -1.4393526E38F}, 0) ;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_roll_rate == (float)2.2174371E38F);
                Debug.Assert(pack.body_pitch_rate == (float)3.0035872E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3116059596U);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.3643316E37F, 4.8150402E36F, 2.3672343E37F, -2.5541663E38F}));
                Debug.Assert(pack.body_yaw_rate == (float)2.9637288E38F);
                Debug.Assert(pack.thrust == (float) -1.383089E37F);
                Debug.Assert(pack.type_mask == (byte)(byte)246);
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)3116059596U;
            p83.thrust = (float) -1.383089E37F;
            p83.q_SET(new float[] {-2.3643316E37F, 4.8150402E36F, 2.3672343E37F, -2.5541663E38F}, 0) ;
            p83.body_yaw_rate = (float)2.9637288E38F;
            p83.body_pitch_rate = (float)3.0035872E38F;
            p83.type_mask = (byte)(byte)246;
            p83.body_roll_rate = (float)2.2174371E38F;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (ushort)(ushort)9527);
                Debug.Assert(pack.vz == (float)5.697605E37F);
                Debug.Assert(pack.afy == (float) -6.870455E37F);
                Debug.Assert(pack.target_component == (byte)(byte)102);
                Debug.Assert(pack.y == (float) -4.9918696E37F);
                Debug.Assert(pack.afx == (float)2.5933758E38F);
                Debug.Assert(pack.vx == (float)2.3851663E38F);
                Debug.Assert(pack.z == (float)1.826876E38F);
                Debug.Assert(pack.yaw_rate == (float)2.0838255E38F);
                Debug.Assert(pack.target_system == (byte)(byte)125);
                Debug.Assert(pack.x == (float) -5.4068406E37F);
                Debug.Assert(pack.time_boot_ms == (uint)861078967U);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.vy == (float)7.455767E36F);
                Debug.Assert(pack.yaw == (float)3.1401048E38F);
                Debug.Assert(pack.afz == (float)5.9126723E37F);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.yaw_rate = (float)2.0838255E38F;
            p84.vz = (float)5.697605E37F;
            p84.afz = (float)5.9126723E37F;
            p84.vx = (float)2.3851663E38F;
            p84.time_boot_ms = (uint)861078967U;
            p84.target_system = (byte)(byte)125;
            p84.vy = (float)7.455767E36F;
            p84.afx = (float)2.5933758E38F;
            p84.target_component = (byte)(byte)102;
            p84.z = (float)1.826876E38F;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p84.y = (float) -4.9918696E37F;
            p84.type_mask = (ushort)(ushort)9527;
            p84.yaw = (float)3.1401048E38F;
            p84.x = (float) -5.4068406E37F;
            p84.afy = (float) -6.870455E37F;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float)2.2390334E38F);
                Debug.Assert(pack.afz == (float)3.2078661E38F);
                Debug.Assert(pack.lat_int == (int) -1841258617);
                Debug.Assert(pack.yaw == (float) -1.4622823E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)14839);
                Debug.Assert(pack.vz == (float)3.1141115E38F);
                Debug.Assert(pack.afy == (float) -6.466574E37F);
                Debug.Assert(pack.afx == (float)1.3382827E38F);
                Debug.Assert(pack.target_component == (byte)(byte)1);
                Debug.Assert(pack.time_boot_ms == (uint)7322646U);
                Debug.Assert(pack.alt == (float)4.8146526E37F);
                Debug.Assert(pack.yaw_rate == (float) -3.1948306E38F);
                Debug.Assert(pack.vx == (float) -2.2205649E38F);
                Debug.Assert(pack.lon_int == (int) -121841089);
                Debug.Assert(pack.target_system == (byte)(byte)36);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p86.target_system = (byte)(byte)36;
            p86.afx = (float)1.3382827E38F;
            p86.yaw_rate = (float) -3.1948306E38F;
            p86.vy = (float)2.2390334E38F;
            p86.lon_int = (int) -121841089;
            p86.vx = (float) -2.2205649E38F;
            p86.target_component = (byte)(byte)1;
            p86.type_mask = (ushort)(ushort)14839;
            p86.vz = (float)3.1141115E38F;
            p86.lat_int = (int) -1841258617;
            p86.afz = (float)3.2078661E38F;
            p86.yaw = (float) -1.4622823E37F;
            p86.alt = (float)4.8146526E37F;
            p86.afy = (float) -6.466574E37F;
            p86.time_boot_ms = (uint)7322646U;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (ushort)(ushort)39259);
                Debug.Assert(pack.time_boot_ms == (uint)2624607941U);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.yaw_rate == (float)1.9076653E38F);
                Debug.Assert(pack.vz == (float)1.9166587E38F);
                Debug.Assert(pack.vy == (float) -2.7811427E38F);
                Debug.Assert(pack.lat_int == (int)30236938);
                Debug.Assert(pack.lon_int == (int) -865578905);
                Debug.Assert(pack.afz == (float) -1.9201308E38F);
                Debug.Assert(pack.alt == (float)1.8146694E38F);
                Debug.Assert(pack.vx == (float) -1.92243E38F);
                Debug.Assert(pack.yaw == (float)1.6791581E38F);
                Debug.Assert(pack.afx == (float) -2.0843168E38F);
                Debug.Assert(pack.afy == (float) -3.271281E38F);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.vx = (float) -1.92243E38F;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p87.time_boot_ms = (uint)2624607941U;
            p87.lat_int = (int)30236938;
            p87.afx = (float) -2.0843168E38F;
            p87.afy = (float) -3.271281E38F;
            p87.afz = (float) -1.9201308E38F;
            p87.vy = (float) -2.7811427E38F;
            p87.yaw = (float)1.6791581E38F;
            p87.lon_int = (int) -865578905;
            p87.type_mask = (ushort)(ushort)39259;
            p87.yaw_rate = (float)1.9076653E38F;
            p87.vz = (float)1.9166587E38F;
            p87.alt = (float)1.8146694E38F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -9.03895E37F);
                Debug.Assert(pack.z == (float)1.7025602E38F);
                Debug.Assert(pack.roll == (float)2.9529769E38F);
                Debug.Assert(pack.yaw == (float) -3.5693585E37F);
                Debug.Assert(pack.x == (float) -1.710138E38F);
                Debug.Assert(pack.time_boot_ms == (uint)236692021U);
                Debug.Assert(pack.pitch == (float) -2.407228E37F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.y = (float) -9.03895E37F;
            p89.pitch = (float) -2.407228E37F;
            p89.z = (float)1.7025602E38F;
            p89.yaw = (float) -3.5693585E37F;
            p89.time_boot_ms = (uint)236692021U;
            p89.x = (float) -1.710138E38F;
            p89.roll = (float)2.9529769E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short) -29871);
                Debug.Assert(pack.yawspeed == (float) -1.8469655E38F);
                Debug.Assert(pack.xacc == (short)(short)9642);
                Debug.Assert(pack.vx == (short)(short) -2272);
                Debug.Assert(pack.alt == (int)1609502740);
                Debug.Assert(pack.vy == (short)(short) -4086);
                Debug.Assert(pack.rollspeed == (float)2.43305E37F);
                Debug.Assert(pack.zacc == (short)(short) -22875);
                Debug.Assert(pack.lat == (int)310626019);
                Debug.Assert(pack.yaw == (float)3.3608742E38F);
                Debug.Assert(pack.pitch == (float)5.533568E37F);
                Debug.Assert(pack.vz == (short)(short) -11899);
                Debug.Assert(pack.roll == (float) -2.3512055E38F);
                Debug.Assert(pack.pitchspeed == (float)7.632687E37F);
                Debug.Assert(pack.time_usec == (ulong)8125924881918468405L);
                Debug.Assert(pack.lon == (int) -490816762);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.vx = (short)(short) -2272;
            p90.pitchspeed = (float)7.632687E37F;
            p90.lat = (int)310626019;
            p90.alt = (int)1609502740;
            p90.pitch = (float)5.533568E37F;
            p90.rollspeed = (float)2.43305E37F;
            p90.xacc = (short)(short)9642;
            p90.lon = (int) -490816762;
            p90.yaw = (float)3.3608742E38F;
            p90.time_usec = (ulong)8125924881918468405L;
            p90.roll = (float) -2.3512055E38F;
            p90.zacc = (short)(short) -22875;
            p90.vy = (short)(short) -4086;
            p90.vz = (short)(short) -11899;
            p90.yawspeed = (float) -1.8469655E38F;
            p90.yacc = (short)(short) -29871;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.throttle == (float) -1.1595005E38F);
                Debug.Assert(pack.aux3 == (float)2.7948248E38F);
                Debug.Assert(pack.pitch_elevator == (float)2.4324177E38F);
                Debug.Assert(pack.yaw_rudder == (float) -2.4836168E38F);
                Debug.Assert(pack.roll_ailerons == (float) -3.2019905E38F);
                Debug.Assert(pack.aux2 == (float)2.4477946E38F);
                Debug.Assert(pack.aux4 == (float) -3.3672763E38F);
                Debug.Assert(pack.time_usec == (ulong)8262106587200923614L);
                Debug.Assert(pack.aux1 == (float) -2.7099772E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)168);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_DISARMED);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.pitch_elevator = (float)2.4324177E38F;
            p91.time_usec = (ulong)8262106587200923614L;
            p91.yaw_rudder = (float) -2.4836168E38F;
            p91.aux3 = (float)2.7948248E38F;
            p91.aux4 = (float) -3.3672763E38F;
            p91.throttle = (float) -1.1595005E38F;
            p91.aux2 = (float)2.4477946E38F;
            p91.aux1 = (float) -2.7099772E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_DISARMED;
            p91.nav_mode = (byte)(byte)168;
            p91.roll_ailerons = (float) -3.2019905E38F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)10712);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)52080);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)5624);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)3771);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)6211);
                Debug.Assert(pack.time_usec == (ulong)7265732210436887188L);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)59861);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)52681);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)22420);
                Debug.Assert(pack.rssi == (byte)(byte)45);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)15931);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)9830);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)25401);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)59065);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan4_raw = (ushort)(ushort)3771;
            p92.chan6_raw = (ushort)(ushort)5624;
            p92.chan12_raw = (ushort)(ushort)25401;
            p92.time_usec = (ulong)7265732210436887188L;
            p92.rssi = (byte)(byte)45;
            p92.chan7_raw = (ushort)(ushort)6211;
            p92.chan3_raw = (ushort)(ushort)52080;
            p92.chan1_raw = (ushort)(ushort)59065;
            p92.chan8_raw = (ushort)(ushort)59861;
            p92.chan5_raw = (ushort)(ushort)9830;
            p92.chan2_raw = (ushort)(ushort)52681;
            p92.chan9_raw = (ushort)(ushort)22420;
            p92.chan10_raw = (ushort)(ushort)10712;
            p92.chan11_raw = (ushort)(ushort)15931;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_DISARMED);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.1950975E38F, 2.5768793E38F, -2.4954907E38F, 3.9150278E37F, 2.906722E38F, -1.4903709E38F, 1.3964731E38F, 1.2753075E37F, -3.2405843E38F, 2.0525035E37F, -2.3622687E38F, -2.7948307E38F, -9.344321E36F, -2.4605242E38F, 2.2601125E38F, -6.125478E37F}));
                Debug.Assert(pack.time_usec == (ulong)7827225952986399952L);
                Debug.Assert(pack.flags == (ulong)4408727418825483628L);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)7827225952986399952L;
            p93.flags = (ulong)4408727418825483628L;
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_DISARMED;
            p93.controls_SET(new float[] {-1.1950975E38F, 2.5768793E38F, -2.4954907E38F, 3.9150278E37F, 2.906722E38F, -1.4903709E38F, 1.3964731E38F, 1.2753075E37F, -3.2405843E38F, 2.0525035E37F, -2.3622687E38F, -2.7948307E38F, -9.344321E36F, -2.4605242E38F, 2.2601125E38F, -6.125478E37F}, 0) ;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)2.3652239E38F);
                Debug.Assert(pack.flow_comp_m_y == (float) -8.607433E36F);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -1.5367091E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)4);
                Debug.Assert(pack.quality == (byte)(byte)239);
                Debug.Assert(pack.ground_distance == (float)3.0923618E38F);
                Debug.Assert(pack.flow_x == (short)(short)23864);
                Debug.Assert(pack.flow_comp_m_x == (float) -6.568657E37F);
                Debug.Assert(pack.flow_y == (short)(short)3443);
                Debug.Assert(pack.time_usec == (ulong)259962404493177777L);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_rate_x_SET((float) -1.5367091E38F, PH) ;
            p100.flow_y = (short)(short)3443;
            p100.flow_comp_m_y = (float) -8.607433E36F;
            p100.time_usec = (ulong)259962404493177777L;
            p100.flow_x = (short)(short)23864;
            p100.quality = (byte)(byte)239;
            p100.flow_comp_m_x = (float) -6.568657E37F;
            p100.ground_distance = (float)3.0923618E38F;
            p100.flow_rate_y_SET((float)2.3652239E38F, PH) ;
            p100.sensor_id = (byte)(byte)4;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -1.1685959E38F);
                Debug.Assert(pack.roll == (float)3.6948679E37F);
                Debug.Assert(pack.yaw == (float)3.2431853E38F);
                Debug.Assert(pack.y == (float)1.3477345E38F);
                Debug.Assert(pack.usec == (ulong)5107067406815029072L);
                Debug.Assert(pack.pitch == (float)3.4846741E37F);
                Debug.Assert(pack.z == (float) -2.7839327E38F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)5107067406815029072L;
            p101.pitch = (float)3.4846741E37F;
            p101.yaw = (float)3.2431853E38F;
            p101.z = (float) -2.7839327E38F;
            p101.y = (float)1.3477345E38F;
            p101.roll = (float)3.6948679E37F;
            p101.x = (float) -1.1685959E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -5.4100006E37F);
                Debug.Assert(pack.usec == (ulong)2899853163939283394L);
                Debug.Assert(pack.x == (float)1.9813912E38F);
                Debug.Assert(pack.roll == (float)9.005574E37F);
                Debug.Assert(pack.pitch == (float)1.1348957E38F);
                Debug.Assert(pack.z == (float) -9.051986E37F);
                Debug.Assert(pack.y == (float)4.2233172E37F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.yaw = (float) -5.4100006E37F;
            p102.z = (float) -9.051986E37F;
            p102.y = (float)4.2233172E37F;
            p102.pitch = (float)1.1348957E38F;
            p102.roll = (float)9.005574E37F;
            p102.usec = (ulong)2899853163939283394L;
            p102.x = (float)1.9813912E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)8750690902642274574L);
                Debug.Assert(pack.x == (float) -1.5571562E38F);
                Debug.Assert(pack.y == (float)9.120672E37F);
                Debug.Assert(pack.z == (float)3.3811586E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.y = (float)9.120672E37F;
            p103.z = (float)3.3811586E38F;
            p103.usec = (ulong)8750690902642274574L;
            p103.x = (float) -1.5571562E38F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)7.2200444E37F);
                Debug.Assert(pack.usec == (ulong)3378792099947538452L);
                Debug.Assert(pack.pitch == (float)1.2754409E38F);
                Debug.Assert(pack.y == (float)1.1193942E38F);
                Debug.Assert(pack.roll == (float) -2.1221923E38F);
                Debug.Assert(pack.x == (float) -3.2462285E38F);
                Debug.Assert(pack.yaw == (float)4.831858E37F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.x = (float) -3.2462285E38F;
            p104.yaw = (float)4.831858E37F;
            p104.y = (float)1.1193942E38F;
            p104.roll = (float) -2.1221923E38F;
            p104.z = (float)7.2200444E37F;
            p104.pitch = (float)1.2754409E38F;
            p104.usec = (ulong)3378792099947538452L;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.abs_pressure == (float) -2.2567804E37F);
                Debug.Assert(pack.pressure_alt == (float)1.6003576E38F);
                Debug.Assert(pack.xacc == (float) -2.2293307E38F);
                Debug.Assert(pack.time_usec == (ulong)3816730488293038085L);
                Debug.Assert(pack.zmag == (float)1.5321451E37F);
                Debug.Assert(pack.xgyro == (float) -2.4528907E38F);
                Debug.Assert(pack.yacc == (float)1.853102E38F);
                Debug.Assert(pack.zacc == (float)3.8283294E37F);
                Debug.Assert(pack.temperature == (float) -8.167964E37F);
                Debug.Assert(pack.xmag == (float) -3.2469815E38F);
                Debug.Assert(pack.zgyro == (float)5.562041E37F);
                Debug.Assert(pack.ymag == (float) -3.1649712E38F);
                Debug.Assert(pack.ygyro == (float)3.0065334E38F);
                Debug.Assert(pack.diff_pressure == (float)2.5954489E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)10147);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.pressure_alt = (float)1.6003576E38F;
            p105.temperature = (float) -8.167964E37F;
            p105.diff_pressure = (float)2.5954489E38F;
            p105.xacc = (float) -2.2293307E38F;
            p105.zgyro = (float)5.562041E37F;
            p105.zmag = (float)1.5321451E37F;
            p105.yacc = (float)1.853102E38F;
            p105.zacc = (float)3.8283294E37F;
            p105.xmag = (float) -3.2469815E38F;
            p105.fields_updated = (ushort)(ushort)10147;
            p105.ymag = (float) -3.1649712E38F;
            p105.abs_pressure = (float) -2.2567804E37F;
            p105.time_usec = (ulong)3816730488293038085L;
            p105.ygyro = (float)3.0065334E38F;
            p105.xgyro = (float) -2.4528907E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integration_time_us == (uint)2994107276U);
                Debug.Assert(pack.integrated_zgyro == (float) -1.9186466E38F);
                Debug.Assert(pack.integrated_ygyro == (float)6.746583E37F);
                Debug.Assert(pack.temperature == (short)(short)14621);
                Debug.Assert(pack.integrated_y == (float) -2.3257586E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)118);
                Debug.Assert(pack.time_usec == (ulong)3351579954453055796L);
                Debug.Assert(pack.integrated_xgyro == (float) -3.0546256E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)825645961U);
                Debug.Assert(pack.distance == (float)2.281074E37F);
                Debug.Assert(pack.quality == (byte)(byte)5);
                Debug.Assert(pack.integrated_x == (float) -2.0706707E38F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_x = (float) -2.0706707E38F;
            p106.integrated_y = (float) -2.3257586E38F;
            p106.integrated_xgyro = (float) -3.0546256E38F;
            p106.quality = (byte)(byte)5;
            p106.distance = (float)2.281074E37F;
            p106.integration_time_us = (uint)2994107276U;
            p106.temperature = (short)(short)14621;
            p106.integrated_ygyro = (float)6.746583E37F;
            p106.integrated_zgyro = (float) -1.9186466E38F;
            p106.time_usec = (ulong)3351579954453055796L;
            p106.sensor_id = (byte)(byte)118;
            p106.time_delta_distance_us = (uint)825645961U;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.diff_pressure == (float) -9.518003E37F);
                Debug.Assert(pack.fields_updated == (uint)1561160052U);
                Debug.Assert(pack.ygyro == (float) -1.8385852E38F);
                Debug.Assert(pack.abs_pressure == (float)2.3782085E38F);
                Debug.Assert(pack.ymag == (float) -2.7559898E38F);
                Debug.Assert(pack.xgyro == (float) -2.2903812E38F);
                Debug.Assert(pack.zacc == (float) -1.51767E38F);
                Debug.Assert(pack.yacc == (float) -1.3314546E38F);
                Debug.Assert(pack.zgyro == (float) -1.7843825E38F);
                Debug.Assert(pack.zmag == (float) -1.945107E38F);
                Debug.Assert(pack.temperature == (float) -3.3105225E38F);
                Debug.Assert(pack.pressure_alt == (float) -2.243678E38F);
                Debug.Assert(pack.time_usec == (ulong)4100654588387902826L);
                Debug.Assert(pack.xacc == (float) -3.0020159E38F);
                Debug.Assert(pack.xmag == (float) -1.4368063E38F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.abs_pressure = (float)2.3782085E38F;
            p107.temperature = (float) -3.3105225E38F;
            p107.zacc = (float) -1.51767E38F;
            p107.xacc = (float) -3.0020159E38F;
            p107.xmag = (float) -1.4368063E38F;
            p107.diff_pressure = (float) -9.518003E37F;
            p107.ymag = (float) -2.7559898E38F;
            p107.yacc = (float) -1.3314546E38F;
            p107.fields_updated = (uint)1561160052U;
            p107.zmag = (float) -1.945107E38F;
            p107.pressure_alt = (float) -2.243678E38F;
            p107.ygyro = (float) -1.8385852E38F;
            p107.zgyro = (float) -1.7843825E38F;
            p107.xgyro = (float) -2.2903812E38F;
            p107.time_usec = (ulong)4100654588387902826L;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (float)2.2687603E38F);
                Debug.Assert(pack.roll == (float) -2.9039048E38F);
                Debug.Assert(pack.std_dev_horz == (float) -1.5945257E38F);
                Debug.Assert(pack.alt == (float) -8.265339E36F);
                Debug.Assert(pack.q2 == (float) -1.2951853E38F);
                Debug.Assert(pack.vd == (float) -2.5249018E38F);
                Debug.Assert(pack.xgyro == (float)3.8298184E37F);
                Debug.Assert(pack.zgyro == (float)2.4506463E38F);
                Debug.Assert(pack.lat == (float) -2.0633417E38F);
                Debug.Assert(pack.ygyro == (float) -2.3580591E38F);
                Debug.Assert(pack.q3 == (float) -2.8731439E38F);
                Debug.Assert(pack.lon == (float) -2.321117E38F);
                Debug.Assert(pack.ve == (float)3.2376658E38F);
                Debug.Assert(pack.yaw == (float) -1.1565231E38F);
                Debug.Assert(pack.vn == (float)2.0073476E38F);
                Debug.Assert(pack.xacc == (float)7.352559E37F);
                Debug.Assert(pack.q4 == (float)3.7205943E37F);
                Debug.Assert(pack.std_dev_vert == (float)1.8056304E38F);
                Debug.Assert(pack.zacc == (float) -5.374239E37F);
                Debug.Assert(pack.q1 == (float) -1.4796022E38F);
                Debug.Assert(pack.pitch == (float) -8.24498E36F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.lat = (float) -2.0633417E38F;
            p108.std_dev_vert = (float)1.8056304E38F;
            p108.xacc = (float)7.352559E37F;
            p108.lon = (float) -2.321117E38F;
            p108.std_dev_horz = (float) -1.5945257E38F;
            p108.zgyro = (float)2.4506463E38F;
            p108.xgyro = (float)3.8298184E37F;
            p108.zacc = (float) -5.374239E37F;
            p108.q1 = (float) -1.4796022E38F;
            p108.ygyro = (float) -2.3580591E38F;
            p108.yaw = (float) -1.1565231E38F;
            p108.yacc = (float)2.2687603E38F;
            p108.q4 = (float)3.7205943E37F;
            p108.vn = (float)2.0073476E38F;
            p108.pitch = (float) -8.24498E36F;
            p108.vd = (float) -2.5249018E38F;
            p108.q2 = (float) -1.2951853E38F;
            p108.roll = (float) -2.9039048E38F;
            p108.alt = (float) -8.265339E36F;
            p108.ve = (float)3.2376658E38F;
            p108.q3 = (float) -2.8731439E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.txbuf == (byte)(byte)186);
                Debug.Assert(pack.remnoise == (byte)(byte)34);
                Debug.Assert(pack.noise == (byte)(byte)11);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)17698);
                Debug.Assert(pack.remrssi == (byte)(byte)98);
                Debug.Assert(pack.rssi == (byte)(byte)67);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)57601);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.noise = (byte)(byte)11;
            p109.rxerrors = (ushort)(ushort)57601;
            p109.remnoise = (byte)(byte)34;
            p109.fixed_ = (ushort)(ushort)17698;
            p109.remrssi = (byte)(byte)98;
            p109.rssi = (byte)(byte)67;
            p109.txbuf = (byte)(byte)186;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)12, (byte)238, (byte)215, (byte)4, (byte)173, (byte)23, (byte)218, (byte)133, (byte)246, (byte)117, (byte)12, (byte)97, (byte)67, (byte)103, (byte)33, (byte)21, (byte)75, (byte)244, (byte)81, (byte)175, (byte)167, (byte)20, (byte)115, (byte)112, (byte)209, (byte)214, (byte)176, (byte)245, (byte)194, (byte)34, (byte)231, (byte)144, (byte)83, (byte)194, (byte)235, (byte)128, (byte)124, (byte)106, (byte)22, (byte)227, (byte)147, (byte)163, (byte)81, (byte)197, (byte)255, (byte)251, (byte)24, (byte)145, (byte)139, (byte)166, (byte)54, (byte)236, (byte)28, (byte)145, (byte)242, (byte)85, (byte)110, (byte)114, (byte)40, (byte)164, (byte)197, (byte)28, (byte)163, (byte)171, (byte)164, (byte)122, (byte)249, (byte)195, (byte)179, (byte)218, (byte)205, (byte)244, (byte)109, (byte)23, (byte)35, (byte)163, (byte)38, (byte)48, (byte)180, (byte)11, (byte)60, (byte)154, (byte)77, (byte)251, (byte)54, (byte)169, (byte)130, (byte)161, (byte)95, (byte)160, (byte)111, (byte)23, (byte)49, (byte)210, (byte)173, (byte)99, (byte)196, (byte)45, (byte)192, (byte)29, (byte)158, (byte)184, (byte)115, (byte)11, (byte)193, (byte)161, (byte)214, (byte)84, (byte)126, (byte)6, (byte)254, (byte)196, (byte)254, (byte)83, (byte)139, (byte)151, (byte)183, (byte)241, (byte)1, (byte)249, (byte)103, (byte)198, (byte)245, (byte)92, (byte)195, (byte)55, (byte)61, (byte)76, (byte)139, (byte)73, (byte)234, (byte)227, (byte)90, (byte)67, (byte)145, (byte)168, (byte)156, (byte)147, (byte)15, (byte)91, (byte)170, (byte)54, (byte)159, (byte)34, (byte)173, (byte)9, (byte)82, (byte)218, (byte)168, (byte)17, (byte)148, (byte)184, (byte)203, (byte)152, (byte)30, (byte)213, (byte)207, (byte)217, (byte)192, (byte)225, (byte)176, (byte)220, (byte)15, (byte)30, (byte)255, (byte)6, (byte)199, (byte)105, (byte)148, (byte)197, (byte)29, (byte)156, (byte)170, (byte)28, (byte)62, (byte)66, (byte)71, (byte)239, (byte)246, (byte)80, (byte)223, (byte)34, (byte)89, (byte)30, (byte)238, (byte)9, (byte)142, (byte)107, (byte)33, (byte)241, (byte)34, (byte)250, (byte)10, (byte)75, (byte)240, (byte)51, (byte)59, (byte)109, (byte)68, (byte)73, (byte)159, (byte)139, (byte)132, (byte)31, (byte)226, (byte)55, (byte)79, (byte)19, (byte)115, (byte)200, (byte)212, (byte)38, (byte)162, (byte)230, (byte)84, (byte)10, (byte)59, (byte)104, (byte)237, (byte)244, (byte)120, (byte)190, (byte)194, (byte)73, (byte)110, (byte)187, (byte)59, (byte)15, (byte)96, (byte)63, (byte)235, (byte)20, (byte)38, (byte)99, (byte)41, (byte)80, (byte)47, (byte)20, (byte)227, (byte)23, (byte)191, (byte)56, (byte)40, (byte)155, (byte)252, (byte)227, (byte)254, (byte)38, (byte)135, (byte)156, (byte)153}));
                Debug.Assert(pack.target_network == (byte)(byte)95);
                Debug.Assert(pack.target_system == (byte)(byte)174);
                Debug.Assert(pack.target_component == (byte)(byte)67);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)95;
            p110.payload_SET(new byte[] {(byte)12, (byte)238, (byte)215, (byte)4, (byte)173, (byte)23, (byte)218, (byte)133, (byte)246, (byte)117, (byte)12, (byte)97, (byte)67, (byte)103, (byte)33, (byte)21, (byte)75, (byte)244, (byte)81, (byte)175, (byte)167, (byte)20, (byte)115, (byte)112, (byte)209, (byte)214, (byte)176, (byte)245, (byte)194, (byte)34, (byte)231, (byte)144, (byte)83, (byte)194, (byte)235, (byte)128, (byte)124, (byte)106, (byte)22, (byte)227, (byte)147, (byte)163, (byte)81, (byte)197, (byte)255, (byte)251, (byte)24, (byte)145, (byte)139, (byte)166, (byte)54, (byte)236, (byte)28, (byte)145, (byte)242, (byte)85, (byte)110, (byte)114, (byte)40, (byte)164, (byte)197, (byte)28, (byte)163, (byte)171, (byte)164, (byte)122, (byte)249, (byte)195, (byte)179, (byte)218, (byte)205, (byte)244, (byte)109, (byte)23, (byte)35, (byte)163, (byte)38, (byte)48, (byte)180, (byte)11, (byte)60, (byte)154, (byte)77, (byte)251, (byte)54, (byte)169, (byte)130, (byte)161, (byte)95, (byte)160, (byte)111, (byte)23, (byte)49, (byte)210, (byte)173, (byte)99, (byte)196, (byte)45, (byte)192, (byte)29, (byte)158, (byte)184, (byte)115, (byte)11, (byte)193, (byte)161, (byte)214, (byte)84, (byte)126, (byte)6, (byte)254, (byte)196, (byte)254, (byte)83, (byte)139, (byte)151, (byte)183, (byte)241, (byte)1, (byte)249, (byte)103, (byte)198, (byte)245, (byte)92, (byte)195, (byte)55, (byte)61, (byte)76, (byte)139, (byte)73, (byte)234, (byte)227, (byte)90, (byte)67, (byte)145, (byte)168, (byte)156, (byte)147, (byte)15, (byte)91, (byte)170, (byte)54, (byte)159, (byte)34, (byte)173, (byte)9, (byte)82, (byte)218, (byte)168, (byte)17, (byte)148, (byte)184, (byte)203, (byte)152, (byte)30, (byte)213, (byte)207, (byte)217, (byte)192, (byte)225, (byte)176, (byte)220, (byte)15, (byte)30, (byte)255, (byte)6, (byte)199, (byte)105, (byte)148, (byte)197, (byte)29, (byte)156, (byte)170, (byte)28, (byte)62, (byte)66, (byte)71, (byte)239, (byte)246, (byte)80, (byte)223, (byte)34, (byte)89, (byte)30, (byte)238, (byte)9, (byte)142, (byte)107, (byte)33, (byte)241, (byte)34, (byte)250, (byte)10, (byte)75, (byte)240, (byte)51, (byte)59, (byte)109, (byte)68, (byte)73, (byte)159, (byte)139, (byte)132, (byte)31, (byte)226, (byte)55, (byte)79, (byte)19, (byte)115, (byte)200, (byte)212, (byte)38, (byte)162, (byte)230, (byte)84, (byte)10, (byte)59, (byte)104, (byte)237, (byte)244, (byte)120, (byte)190, (byte)194, (byte)73, (byte)110, (byte)187, (byte)59, (byte)15, (byte)96, (byte)63, (byte)235, (byte)20, (byte)38, (byte)99, (byte)41, (byte)80, (byte)47, (byte)20, (byte)227, (byte)23, (byte)191, (byte)56, (byte)40, (byte)155, (byte)252, (byte)227, (byte)254, (byte)38, (byte)135, (byte)156, (byte)153}, 0) ;
            p110.target_system = (byte)(byte)174;
            p110.target_component = (byte)(byte)67;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long)3453394041630621839L);
                Debug.Assert(pack.tc1 == (long) -637499424501725724L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long)3453394041630621839L;
            p111.tc1 = (long) -637499424501725724L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1104310211845924139L);
                Debug.Assert(pack.seq == (uint)2159957636U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)2159957636U;
            p112.time_usec = (ulong)1104310211845924139L;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7097852115431620886L);
                Debug.Assert(pack.lat == (int) -171082174);
                Debug.Assert(pack.vn == (short)(short) -9115);
                Debug.Assert(pack.eph == (ushort)(ushort)41087);
                Debug.Assert(pack.ve == (short)(short)4047);
                Debug.Assert(pack.cog == (ushort)(ushort)22966);
                Debug.Assert(pack.fix_type == (byte)(byte)233);
                Debug.Assert(pack.epv == (ushort)(ushort)46238);
                Debug.Assert(pack.vel == (ushort)(ushort)55005);
                Debug.Assert(pack.satellites_visible == (byte)(byte)53);
                Debug.Assert(pack.vd == (short)(short)15595);
                Debug.Assert(pack.alt == (int)935263793);
                Debug.Assert(pack.lon == (int) -1328354817);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.fix_type = (byte)(byte)233;
            p113.vel = (ushort)(ushort)55005;
            p113.lat = (int) -171082174;
            p113.alt = (int)935263793;
            p113.eph = (ushort)(ushort)41087;
            p113.epv = (ushort)(ushort)46238;
            p113.satellites_visible = (byte)(byte)53;
            p113.cog = (ushort)(ushort)22966;
            p113.vd = (short)(short)15595;
            p113.lon = (int) -1328354817;
            p113.vn = (short)(short) -9115;
            p113.time_usec = (ulong)7097852115431620886L;
            p113.ve = (short)(short)4047;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_ygyro == (float) -1.8974841E38F);
                Debug.Assert(pack.integrated_y == (float) -6.2694515E37F);
                Debug.Assert(pack.integrated_zgyro == (float) -3.1656203E38F);
                Debug.Assert(pack.distance == (float)2.6602782E38F);
                Debug.Assert(pack.time_usec == (ulong)3191670213265905907L);
                Debug.Assert(pack.temperature == (short)(short)20431);
                Debug.Assert(pack.time_delta_distance_us == (uint)3313725570U);
                Debug.Assert(pack.integrated_x == (float)3.742623E37F);
                Debug.Assert(pack.quality == (byte)(byte)120);
                Debug.Assert(pack.sensor_id == (byte)(byte)142);
                Debug.Assert(pack.integration_time_us == (uint)1517760745U);
                Debug.Assert(pack.integrated_xgyro == (float)2.9581671E38F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.distance = (float)2.6602782E38F;
            p114.integrated_ygyro = (float) -1.8974841E38F;
            p114.integration_time_us = (uint)1517760745U;
            p114.temperature = (short)(short)20431;
            p114.time_usec = (ulong)3191670213265905907L;
            p114.time_delta_distance_us = (uint)3313725570U;
            p114.integrated_y = (float) -6.2694515E37F;
            p114.integrated_zgyro = (float) -3.1656203E38F;
            p114.sensor_id = (byte)(byte)142;
            p114.integrated_x = (float)3.742623E37F;
            p114.quality = (byte)(byte)120;
            p114.integrated_xgyro = (float)2.9581671E38F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)5610);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {3.3821593E38F, 1.8305995E37F, 7.240006E37F, -1.6121573E38F}));
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)39006);
                Debug.Assert(pack.alt == (int) -2056565425);
                Debug.Assert(pack.vy == (short)(short)9469);
                Debug.Assert(pack.vz == (short)(short) -9565);
                Debug.Assert(pack.lon == (int) -1557144557);
                Debug.Assert(pack.xacc == (short)(short)18510);
                Debug.Assert(pack.time_usec == (ulong)8918024997549241278L);
                Debug.Assert(pack.rollspeed == (float) -1.1650533E38F);
                Debug.Assert(pack.vx == (short)(short)3003);
                Debug.Assert(pack.zacc == (short)(short) -5450);
                Debug.Assert(pack.yacc == (short)(short)573);
                Debug.Assert(pack.yawspeed == (float)2.869256E38F);
                Debug.Assert(pack.pitchspeed == (float) -7.8222824E37F);
                Debug.Assert(pack.lat == (int)789598819);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.attitude_quaternion_SET(new float[] {3.3821593E38F, 1.8305995E37F, 7.240006E37F, -1.6121573E38F}, 0) ;
            p115.xacc = (short)(short)18510;
            p115.true_airspeed = (ushort)(ushort)5610;
            p115.ind_airspeed = (ushort)(ushort)39006;
            p115.pitchspeed = (float) -7.8222824E37F;
            p115.vy = (short)(short)9469;
            p115.vz = (short)(short) -9565;
            p115.yawspeed = (float)2.869256E38F;
            p115.alt = (int) -2056565425;
            p115.zacc = (short)(short) -5450;
            p115.vx = (short)(short)3003;
            p115.time_usec = (ulong)8918024997549241278L;
            p115.rollspeed = (float) -1.1650533E38F;
            p115.lon = (int) -1557144557;
            p115.lat = (int)789598819;
            p115.yacc = (short)(short)573;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short) -5392);
                Debug.Assert(pack.zmag == (short)(short) -5767);
                Debug.Assert(pack.xacc == (short)(short)12743);
                Debug.Assert(pack.xgyro == (short)(short)9979);
                Debug.Assert(pack.ymag == (short)(short)7548);
                Debug.Assert(pack.ygyro == (short)(short)20079);
                Debug.Assert(pack.zacc == (short)(short)7406);
                Debug.Assert(pack.time_boot_ms == (uint)1281052812U);
                Debug.Assert(pack.zgyro == (short)(short) -9403);
                Debug.Assert(pack.xmag == (short)(short)16183);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.ygyro = (short)(short)20079;
            p116.zacc = (short)(short)7406;
            p116.ymag = (short)(short)7548;
            p116.xacc = (short)(short)12743;
            p116.xmag = (short)(short)16183;
            p116.zmag = (short)(short) -5767;
            p116.zgyro = (short)(short) -9403;
            p116.yacc = (short)(short) -5392;
            p116.time_boot_ms = (uint)1281052812U;
            p116.xgyro = (short)(short)9979;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start == (ushort)(ushort)18431);
                Debug.Assert(pack.target_system == (byte)(byte)209);
                Debug.Assert(pack.target_component == (byte)(byte)148);
                Debug.Assert(pack.end == (ushort)(ushort)1493);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)209;
            p117.start = (ushort)(ushort)18431;
            p117.end = (ushort)(ushort)1493;
            p117.target_component = (byte)(byte)148;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_utc == (uint)592175244U);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)59440);
                Debug.Assert(pack.num_logs == (ushort)(ushort)39545);
                Debug.Assert(pack.size == (uint)3338872799U);
                Debug.Assert(pack.id == (ushort)(ushort)54753);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.time_utc = (uint)592175244U;
            p118.last_log_num = (ushort)(ushort)59440;
            p118.id = (ushort)(ushort)54753;
            p118.num_logs = (ushort)(ushort)39545;
            p118.size = (uint)3338872799U;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)25);
                Debug.Assert(pack.count == (uint)2210362408U);
                Debug.Assert(pack.id == (ushort)(ushort)37941);
                Debug.Assert(pack.target_system == (byte)(byte)103);
                Debug.Assert(pack.ofs == (uint)1144293514U);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.ofs = (uint)1144293514U;
            p119.count = (uint)2210362408U;
            p119.target_system = (byte)(byte)103;
            p119.target_component = (byte)(byte)25;
            p119.id = (ushort)(ushort)37941;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)13489);
                Debug.Assert(pack.ofs == (uint)1621605778U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)59, (byte)54, (byte)117, (byte)50, (byte)29, (byte)105, (byte)183, (byte)196, (byte)157, (byte)84, (byte)161, (byte)9, (byte)43, (byte)2, (byte)40, (byte)141, (byte)142, (byte)116, (byte)200, (byte)134, (byte)191, (byte)1, (byte)30, (byte)203, (byte)227, (byte)182, (byte)196, (byte)198, (byte)183, (byte)144, (byte)223, (byte)125, (byte)247, (byte)2, (byte)153, (byte)63, (byte)214, (byte)65, (byte)98, (byte)55, (byte)85, (byte)79, (byte)97, (byte)219, (byte)39, (byte)139, (byte)48, (byte)73, (byte)11, (byte)83, (byte)130, (byte)205, (byte)30, (byte)155, (byte)209, (byte)228, (byte)144, (byte)47, (byte)28, (byte)174, (byte)98, (byte)88, (byte)43, (byte)144, (byte)51, (byte)35, (byte)178, (byte)148, (byte)187, (byte)41, (byte)42, (byte)93, (byte)135, (byte)212, (byte)182, (byte)129, (byte)209, (byte)131, (byte)200, (byte)4, (byte)75, (byte)176, (byte)26, (byte)31, (byte)224, (byte)147, (byte)249, (byte)146, (byte)188, (byte)83}));
                Debug.Assert(pack.count == (byte)(byte)16);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.data__SET(new byte[] {(byte)59, (byte)54, (byte)117, (byte)50, (byte)29, (byte)105, (byte)183, (byte)196, (byte)157, (byte)84, (byte)161, (byte)9, (byte)43, (byte)2, (byte)40, (byte)141, (byte)142, (byte)116, (byte)200, (byte)134, (byte)191, (byte)1, (byte)30, (byte)203, (byte)227, (byte)182, (byte)196, (byte)198, (byte)183, (byte)144, (byte)223, (byte)125, (byte)247, (byte)2, (byte)153, (byte)63, (byte)214, (byte)65, (byte)98, (byte)55, (byte)85, (byte)79, (byte)97, (byte)219, (byte)39, (byte)139, (byte)48, (byte)73, (byte)11, (byte)83, (byte)130, (byte)205, (byte)30, (byte)155, (byte)209, (byte)228, (byte)144, (byte)47, (byte)28, (byte)174, (byte)98, (byte)88, (byte)43, (byte)144, (byte)51, (byte)35, (byte)178, (byte)148, (byte)187, (byte)41, (byte)42, (byte)93, (byte)135, (byte)212, (byte)182, (byte)129, (byte)209, (byte)131, (byte)200, (byte)4, (byte)75, (byte)176, (byte)26, (byte)31, (byte)224, (byte)147, (byte)249, (byte)146, (byte)188, (byte)83}, 0) ;
            p120.ofs = (uint)1621605778U;
            p120.id = (ushort)(ushort)13489;
            p120.count = (byte)(byte)16;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)69);
                Debug.Assert(pack.target_system == (byte)(byte)204);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)204;
            p121.target_component = (byte)(byte)69;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)202);
                Debug.Assert(pack.target_component == (byte)(byte)20);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)202;
            p122.target_component = (byte)(byte)20;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)96);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)173, (byte)6, (byte)80, (byte)39, (byte)252, (byte)171, (byte)141, (byte)59, (byte)184, (byte)158, (byte)182, (byte)26, (byte)15, (byte)68, (byte)65, (byte)162, (byte)129, (byte)254, (byte)163, (byte)144, (byte)191, (byte)58, (byte)136, (byte)213, (byte)117, (byte)13, (byte)186, (byte)248, (byte)105, (byte)25, (byte)233, (byte)239, (byte)188, (byte)182, (byte)80, (byte)99, (byte)120, (byte)168, (byte)194, (byte)105, (byte)48, (byte)119, (byte)199, (byte)166, (byte)21, (byte)165, (byte)227, (byte)115, (byte)202, (byte)109, (byte)164, (byte)153, (byte)217, (byte)47, (byte)40, (byte)9, (byte)83, (byte)65, (byte)90, (byte)115, (byte)170, (byte)27, (byte)237, (byte)180, (byte)156, (byte)78, (byte)81, (byte)173, (byte)63, (byte)79, (byte)205, (byte)187, (byte)171, (byte)28, (byte)137, (byte)247, (byte)90, (byte)14, (byte)93, (byte)220, (byte)247, (byte)57, (byte)214, (byte)178, (byte)41, (byte)194, (byte)65, (byte)74, (byte)125, (byte)6, (byte)25, (byte)152, (byte)151, (byte)231, (byte)120, (byte)246, (byte)154, (byte)70, (byte)113, (byte)114, (byte)139, (byte)161, (byte)252, (byte)250, (byte)152, (byte)249, (byte)10, (byte)165, (byte)25, (byte)108}));
                Debug.Assert(pack.target_component == (byte)(byte)158);
                Debug.Assert(pack.len == (byte)(byte)45);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.data__SET(new byte[] {(byte)173, (byte)6, (byte)80, (byte)39, (byte)252, (byte)171, (byte)141, (byte)59, (byte)184, (byte)158, (byte)182, (byte)26, (byte)15, (byte)68, (byte)65, (byte)162, (byte)129, (byte)254, (byte)163, (byte)144, (byte)191, (byte)58, (byte)136, (byte)213, (byte)117, (byte)13, (byte)186, (byte)248, (byte)105, (byte)25, (byte)233, (byte)239, (byte)188, (byte)182, (byte)80, (byte)99, (byte)120, (byte)168, (byte)194, (byte)105, (byte)48, (byte)119, (byte)199, (byte)166, (byte)21, (byte)165, (byte)227, (byte)115, (byte)202, (byte)109, (byte)164, (byte)153, (byte)217, (byte)47, (byte)40, (byte)9, (byte)83, (byte)65, (byte)90, (byte)115, (byte)170, (byte)27, (byte)237, (byte)180, (byte)156, (byte)78, (byte)81, (byte)173, (byte)63, (byte)79, (byte)205, (byte)187, (byte)171, (byte)28, (byte)137, (byte)247, (byte)90, (byte)14, (byte)93, (byte)220, (byte)247, (byte)57, (byte)214, (byte)178, (byte)41, (byte)194, (byte)65, (byte)74, (byte)125, (byte)6, (byte)25, (byte)152, (byte)151, (byte)231, (byte)120, (byte)246, (byte)154, (byte)70, (byte)113, (byte)114, (byte)139, (byte)161, (byte)252, (byte)250, (byte)152, (byte)249, (byte)10, (byte)165, (byte)25, (byte)108}, 0) ;
            p123.target_system = (byte)(byte)96;
            p123.target_component = (byte)(byte)158;
            p123.len = (byte)(byte)45;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.eph == (ushort)(ushort)14653);
                Debug.Assert(pack.time_usec == (ulong)8609420865966697922L);
                Debug.Assert(pack.dgps_age == (uint)3459589925U);
                Debug.Assert(pack.lon == (int) -1802060534);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.lat == (int)2097913753);
                Debug.Assert(pack.satellites_visible == (byte)(byte)116);
                Debug.Assert(pack.dgps_numch == (byte)(byte)65);
                Debug.Assert(pack.epv == (ushort)(ushort)25352);
                Debug.Assert(pack.vel == (ushort)(ushort)64215);
                Debug.Assert(pack.alt == (int) -1446308277);
                Debug.Assert(pack.cog == (ushort)(ushort)51760);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.dgps_age = (uint)3459589925U;
            p124.lon = (int) -1802060534;
            p124.cog = (ushort)(ushort)51760;
            p124.vel = (ushort)(ushort)64215;
            p124.alt = (int) -1446308277;
            p124.time_usec = (ulong)8609420865966697922L;
            p124.lat = (int)2097913753;
            p124.satellites_visible = (byte)(byte)116;
            p124.eph = (ushort)(ushort)14653;
            p124.dgps_numch = (byte)(byte)65;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p124.epv = (ushort)(ushort)25352;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)4697);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED);
                Debug.Assert(pack.Vservo == (ushort)(ushort)14388);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)4697;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED;
            p125.Vservo = (ushort)(ushort)14388;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)156, (byte)119, (byte)142, (byte)130, (byte)44, (byte)205, (byte)11, (byte)189, (byte)160, (byte)233, (byte)28, (byte)173, (byte)141, (byte)95, (byte)246, (byte)187, (byte)200, (byte)195, (byte)227, (byte)4, (byte)175, (byte)235, (byte)206, (byte)43, (byte)157, (byte)138, (byte)226, (byte)38, (byte)187, (byte)120, (byte)51, (byte)35, (byte)132, (byte)207, (byte)15, (byte)177, (byte)68, (byte)101, (byte)17, (byte)194, (byte)56, (byte)120, (byte)154, (byte)90, (byte)44, (byte)1, (byte)20, (byte)133, (byte)73, (byte)28, (byte)95, (byte)210, (byte)33, (byte)95, (byte)149, (byte)10, (byte)241, (byte)90, (byte)116, (byte)58, (byte)195, (byte)232, (byte)207, (byte)245, (byte)33, (byte)21, (byte)189, (byte)12, (byte)92, (byte)0}));
                Debug.Assert(pack.baudrate == (uint)2034456154U);
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
                Debug.Assert(pack.count == (byte)(byte)49);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING);
                Debug.Assert(pack.timeout == (ushort)(ushort)65521);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.data__SET(new byte[] {(byte)156, (byte)119, (byte)142, (byte)130, (byte)44, (byte)205, (byte)11, (byte)189, (byte)160, (byte)233, (byte)28, (byte)173, (byte)141, (byte)95, (byte)246, (byte)187, (byte)200, (byte)195, (byte)227, (byte)4, (byte)175, (byte)235, (byte)206, (byte)43, (byte)157, (byte)138, (byte)226, (byte)38, (byte)187, (byte)120, (byte)51, (byte)35, (byte)132, (byte)207, (byte)15, (byte)177, (byte)68, (byte)101, (byte)17, (byte)194, (byte)56, (byte)120, (byte)154, (byte)90, (byte)44, (byte)1, (byte)20, (byte)133, (byte)73, (byte)28, (byte)95, (byte)210, (byte)33, (byte)95, (byte)149, (byte)10, (byte)241, (byte)90, (byte)116, (byte)58, (byte)195, (byte)232, (byte)207, (byte)245, (byte)33, (byte)21, (byte)189, (byte)12, (byte)92, (byte)0}, 0) ;
            p126.timeout = (ushort)(ushort)65521;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING;
            p126.baudrate = (uint)2034456154U;
            p126.count = (byte)(byte)49;
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_a_mm == (int)1116764330);
                Debug.Assert(pack.wn == (ushort)(ushort)28194);
                Debug.Assert(pack.nsats == (byte)(byte)176);
                Debug.Assert(pack.accuracy == (uint)3695150150U);
                Debug.Assert(pack.tow == (uint)3007529320U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)118);
                Debug.Assert(pack.rtk_health == (byte)(byte)179);
                Debug.Assert(pack.iar_num_hypotheses == (int)369394863);
                Debug.Assert(pack.rtk_rate == (byte)(byte)154);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)49);
                Debug.Assert(pack.time_last_baseline_ms == (uint)4136933376U);
                Debug.Assert(pack.baseline_c_mm == (int)1673241637);
                Debug.Assert(pack.baseline_b_mm == (int)2093601302);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.rtk_receiver_id = (byte)(byte)118;
            p127.baseline_c_mm = (int)1673241637;
            p127.rtk_health = (byte)(byte)179;
            p127.iar_num_hypotheses = (int)369394863;
            p127.baseline_a_mm = (int)1116764330;
            p127.wn = (ushort)(ushort)28194;
            p127.baseline_b_mm = (int)2093601302;
            p127.baseline_coords_type = (byte)(byte)49;
            p127.time_last_baseline_ms = (uint)4136933376U;
            p127.accuracy = (uint)3695150150U;
            p127.nsats = (byte)(byte)176;
            p127.tow = (uint)3007529320U;
            p127.rtk_rate = (byte)(byte)154;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rtk_health == (byte)(byte)11);
                Debug.Assert(pack.nsats == (byte)(byte)12);
                Debug.Assert(pack.accuracy == (uint)248793369U);
                Debug.Assert(pack.wn == (ushort)(ushort)39680);
                Debug.Assert(pack.baseline_c_mm == (int)1191360782);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)220);
                Debug.Assert(pack.time_last_baseline_ms == (uint)4166356749U);
                Debug.Assert(pack.baseline_a_mm == (int) -381535965);
                Debug.Assert(pack.tow == (uint)2654275313U);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)252);
                Debug.Assert(pack.rtk_rate == (byte)(byte)116);
                Debug.Assert(pack.iar_num_hypotheses == (int)1018787329);
                Debug.Assert(pack.baseline_b_mm == (int)1399325183);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.accuracy = (uint)248793369U;
            p128.baseline_c_mm = (int)1191360782;
            p128.baseline_coords_type = (byte)(byte)252;
            p128.baseline_a_mm = (int) -381535965;
            p128.baseline_b_mm = (int)1399325183;
            p128.rtk_receiver_id = (byte)(byte)220;
            p128.nsats = (byte)(byte)12;
            p128.rtk_rate = (byte)(byte)116;
            p128.iar_num_hypotheses = (int)1018787329;
            p128.time_last_baseline_ms = (uint)4166356749U;
            p128.tow = (uint)2654275313U;
            p128.wn = (ushort)(ushort)39680;
            p128.rtk_health = (byte)(byte)11;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short)22840);
                Debug.Assert(pack.xmag == (short)(short) -25211);
                Debug.Assert(pack.ygyro == (short)(short)20700);
                Debug.Assert(pack.xgyro == (short)(short)23656);
                Debug.Assert(pack.time_boot_ms == (uint)3608664527U);
                Debug.Assert(pack.ymag == (short)(short)20641);
                Debug.Assert(pack.zmag == (short)(short) -22295);
                Debug.Assert(pack.xacc == (short)(short) -32346);
                Debug.Assert(pack.zgyro == (short)(short) -25640);
                Debug.Assert(pack.zacc == (short)(short) -22060);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.xacc = (short)(short) -32346;
            p129.ymag = (short)(short)20641;
            p129.zgyro = (short)(short) -25640;
            p129.xgyro = (short)(short)23656;
            p129.yacc = (short)(short)22840;
            p129.zmag = (short)(short) -22295;
            p129.xmag = (short)(short) -25211;
            p129.ygyro = (short)(short)20700;
            p129.time_boot_ms = (uint)3608664527U;
            p129.zacc = (short)(short) -22060;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.height == (ushort)(ushort)57328);
                Debug.Assert(pack.width == (ushort)(ushort)52933);
                Debug.Assert(pack.type == (byte)(byte)9);
                Debug.Assert(pack.jpg_quality == (byte)(byte)8);
                Debug.Assert(pack.size == (uint)1482661070U);
                Debug.Assert(pack.payload == (byte)(byte)151);
                Debug.Assert(pack.packets == (ushort)(ushort)22211);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.height = (ushort)(ushort)57328;
            p130.jpg_quality = (byte)(byte)8;
            p130.width = (ushort)(ushort)52933;
            p130.size = (uint)1482661070U;
            p130.packets = (ushort)(ushort)22211;
            p130.payload = (byte)(byte)151;
            p130.type = (byte)(byte)9;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)52, (byte)134, (byte)177, (byte)26, (byte)28, (byte)179, (byte)141, (byte)86, (byte)230, (byte)21, (byte)36, (byte)93, (byte)249, (byte)205, (byte)92, (byte)221, (byte)1, (byte)191, (byte)110, (byte)247, (byte)142, (byte)38, (byte)242, (byte)164, (byte)146, (byte)0, (byte)60, (byte)139, (byte)238, (byte)162, (byte)32, (byte)232, (byte)151, (byte)110, (byte)234, (byte)154, (byte)182, (byte)150, (byte)23, (byte)224, (byte)126, (byte)217, (byte)209, (byte)30, (byte)206, (byte)29, (byte)90, (byte)197, (byte)139, (byte)74, (byte)13, (byte)255, (byte)131, (byte)66, (byte)205, (byte)191, (byte)230, (byte)65, (byte)62, (byte)80, (byte)73, (byte)137, (byte)158, (byte)136, (byte)56, (byte)222, (byte)113, (byte)238, (byte)253, (byte)73, (byte)1, (byte)21, (byte)205, (byte)17, (byte)11, (byte)175, (byte)212, (byte)48, (byte)109, (byte)10, (byte)182, (byte)162, (byte)15, (byte)36, (byte)251, (byte)203, (byte)173, (byte)136, (byte)197, (byte)254, (byte)207, (byte)48, (byte)82, (byte)143, (byte)88, (byte)116, (byte)62, (byte)5, (byte)238, (byte)235, (byte)21, (byte)97, (byte)246, (byte)125, (byte)24, (byte)182, (byte)192, (byte)115, (byte)133, (byte)143, (byte)10, (byte)47, (byte)139, (byte)132, (byte)172, (byte)179, (byte)89, (byte)142, (byte)37, (byte)223, (byte)222, (byte)26, (byte)44, (byte)150, (byte)69, (byte)172, (byte)130, (byte)231, (byte)42, (byte)19, (byte)245, (byte)56, (byte)87, (byte)144, (byte)220, (byte)17, (byte)163, (byte)185, (byte)207, (byte)145, (byte)4, (byte)42, (byte)140, (byte)182, (byte)96, (byte)5, (byte)128, (byte)222, (byte)223, (byte)183, (byte)6, (byte)0, (byte)136, (byte)78, (byte)192, (byte)75, (byte)224, (byte)91, (byte)154, (byte)167, (byte)140, (byte)105, (byte)235, (byte)73, (byte)96, (byte)229, (byte)223, (byte)184, (byte)173, (byte)153, (byte)199, (byte)210, (byte)11, (byte)136, (byte)29, (byte)134, (byte)131, (byte)239, (byte)139, (byte)232, (byte)127, (byte)49, (byte)105, (byte)72, (byte)232, (byte)111, (byte)112, (byte)208, (byte)38, (byte)190, (byte)156, (byte)230, (byte)167, (byte)140, (byte)174, (byte)124, (byte)227, (byte)36, (byte)91, (byte)186, (byte)23, (byte)161, (byte)255, (byte)13, (byte)156, (byte)128, (byte)80, (byte)36, (byte)22, (byte)42, (byte)200, (byte)146, (byte)154, (byte)0, (byte)78, (byte)49, (byte)113, (byte)151, (byte)140, (byte)180, (byte)12, (byte)254, (byte)62, (byte)157, (byte)198, (byte)0, (byte)234, (byte)186, (byte)241, (byte)86, (byte)196, (byte)148, (byte)146, (byte)236, (byte)145, (byte)57, (byte)226, (byte)209, (byte)65, (byte)150, (byte)144, (byte)202, (byte)232, (byte)71, (byte)240, (byte)113, (byte)111, (byte)211, (byte)203, (byte)86, (byte)41, (byte)159, (byte)3}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)9910);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)9910;
            p131.data__SET(new byte[] {(byte)52, (byte)134, (byte)177, (byte)26, (byte)28, (byte)179, (byte)141, (byte)86, (byte)230, (byte)21, (byte)36, (byte)93, (byte)249, (byte)205, (byte)92, (byte)221, (byte)1, (byte)191, (byte)110, (byte)247, (byte)142, (byte)38, (byte)242, (byte)164, (byte)146, (byte)0, (byte)60, (byte)139, (byte)238, (byte)162, (byte)32, (byte)232, (byte)151, (byte)110, (byte)234, (byte)154, (byte)182, (byte)150, (byte)23, (byte)224, (byte)126, (byte)217, (byte)209, (byte)30, (byte)206, (byte)29, (byte)90, (byte)197, (byte)139, (byte)74, (byte)13, (byte)255, (byte)131, (byte)66, (byte)205, (byte)191, (byte)230, (byte)65, (byte)62, (byte)80, (byte)73, (byte)137, (byte)158, (byte)136, (byte)56, (byte)222, (byte)113, (byte)238, (byte)253, (byte)73, (byte)1, (byte)21, (byte)205, (byte)17, (byte)11, (byte)175, (byte)212, (byte)48, (byte)109, (byte)10, (byte)182, (byte)162, (byte)15, (byte)36, (byte)251, (byte)203, (byte)173, (byte)136, (byte)197, (byte)254, (byte)207, (byte)48, (byte)82, (byte)143, (byte)88, (byte)116, (byte)62, (byte)5, (byte)238, (byte)235, (byte)21, (byte)97, (byte)246, (byte)125, (byte)24, (byte)182, (byte)192, (byte)115, (byte)133, (byte)143, (byte)10, (byte)47, (byte)139, (byte)132, (byte)172, (byte)179, (byte)89, (byte)142, (byte)37, (byte)223, (byte)222, (byte)26, (byte)44, (byte)150, (byte)69, (byte)172, (byte)130, (byte)231, (byte)42, (byte)19, (byte)245, (byte)56, (byte)87, (byte)144, (byte)220, (byte)17, (byte)163, (byte)185, (byte)207, (byte)145, (byte)4, (byte)42, (byte)140, (byte)182, (byte)96, (byte)5, (byte)128, (byte)222, (byte)223, (byte)183, (byte)6, (byte)0, (byte)136, (byte)78, (byte)192, (byte)75, (byte)224, (byte)91, (byte)154, (byte)167, (byte)140, (byte)105, (byte)235, (byte)73, (byte)96, (byte)229, (byte)223, (byte)184, (byte)173, (byte)153, (byte)199, (byte)210, (byte)11, (byte)136, (byte)29, (byte)134, (byte)131, (byte)239, (byte)139, (byte)232, (byte)127, (byte)49, (byte)105, (byte)72, (byte)232, (byte)111, (byte)112, (byte)208, (byte)38, (byte)190, (byte)156, (byte)230, (byte)167, (byte)140, (byte)174, (byte)124, (byte)227, (byte)36, (byte)91, (byte)186, (byte)23, (byte)161, (byte)255, (byte)13, (byte)156, (byte)128, (byte)80, (byte)36, (byte)22, (byte)42, (byte)200, (byte)146, (byte)154, (byte)0, (byte)78, (byte)49, (byte)113, (byte)151, (byte)140, (byte)180, (byte)12, (byte)254, (byte)62, (byte)157, (byte)198, (byte)0, (byte)234, (byte)186, (byte)241, (byte)86, (byte)196, (byte)148, (byte)146, (byte)236, (byte)145, (byte)57, (byte)226, (byte)209, (byte)65, (byte)150, (byte)144, (byte)202, (byte)232, (byte)71, (byte)240, (byte)113, (byte)111, (byte)211, (byte)203, (byte)86, (byte)41, (byte)159, (byte)3}, 0) ;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_distance == (ushort)(ushort)39176);
                Debug.Assert(pack.time_boot_ms == (uint)569609710U);
                Debug.Assert(pack.id == (byte)(byte)79);
                Debug.Assert(pack.current_distance == (ushort)(ushort)53929);
                Debug.Assert(pack.max_distance == (ushort)(ushort)60431);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_180);
                Debug.Assert(pack.covariance == (byte)(byte)6);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.covariance = (byte)(byte)6;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_180;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p132.id = (byte)(byte)79;
            p132.current_distance = (ushort)(ushort)53929;
            p132.time_boot_ms = (uint)569609710U;
            p132.max_distance = (ushort)(ushort)60431;
            p132.min_distance = (ushort)(ushort)39176;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -790436407);
                Debug.Assert(pack.mask == (ulong)4009208401771903647L);
                Debug.Assert(pack.lon == (int) -65483339);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)37672);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.grid_spacing = (ushort)(ushort)37672;
            p133.lon = (int) -65483339;
            p133.mask = (ulong)4009208401771903647L;
            p133.lat = (int) -790436407;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1472585143);
                Debug.Assert(pack.gridbit == (byte)(byte)206);
                Debug.Assert(pack.lat == (int) -2129426277);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)12700);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -31267, (short)13979, (short) -20488, (short)29201, (short) -477, (short) -19925, (short)10603, (short)16584, (short)23486, (short) -12801, (short)27619, (short)14662, (short) -15134, (short)24229, (short)10077, (short)29365}));
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -2129426277;
            p134.grid_spacing = (ushort)(ushort)12700;
            p134.data__SET(new short[] {(short) -31267, (short)13979, (short) -20488, (short)29201, (short) -477, (short) -19925, (short)10603, (short)16584, (short)23486, (short) -12801, (short)27619, (short)14662, (short) -15134, (short)24229, (short)10077, (short)29365}, 0) ;
            p134.lon = (int) -1472585143;
            p134.gridbit = (byte)(byte)206;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1096243045);
                Debug.Assert(pack.lat == (int)639370847);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)639370847;
            p135.lon = (int) -1096243045;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.terrain_height == (float) -2.477289E38F);
                Debug.Assert(pack.loaded == (ushort)(ushort)16533);
                Debug.Assert(pack.lat == (int) -885865121);
                Debug.Assert(pack.current_height == (float)5.523861E37F);
                Debug.Assert(pack.lon == (int) -1329779804);
                Debug.Assert(pack.spacing == (ushort)(ushort)30873);
                Debug.Assert(pack.pending == (ushort)(ushort)53527);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.spacing = (ushort)(ushort)30873;
            p136.loaded = (ushort)(ushort)16533;
            p136.terrain_height = (float) -2.477289E38F;
            p136.current_height = (float)5.523861E37F;
            p136.lon = (int) -1329779804;
            p136.lat = (int) -885865121;
            p136.pending = (ushort)(ushort)53527;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float) -1.9997975E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3125312941U);
                Debug.Assert(pack.temperature == (short)(short)16597);
                Debug.Assert(pack.press_abs == (float)1.03742953E37F);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.temperature = (short)(short)16597;
            p137.time_boot_ms = (uint)3125312941U;
            p137.press_abs = (float)1.03742953E37F;
            p137.press_diff = (float) -1.9997975E38F;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -5.288614E36F);
                Debug.Assert(pack.z == (float)1.9156152E38F);
                Debug.Assert(pack.x == (float)4.8594123E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-9.213336E37F, -1.0555538E38F, -2.6479325E38F, -2.2246345E37F}));
                Debug.Assert(pack.time_usec == (ulong)3827968490149656780L);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)3827968490149656780L;
            p138.y = (float) -5.288614E36F;
            p138.z = (float)1.9156152E38F;
            p138.q_SET(new float[] {-9.213336E37F, -1.0555538E38F, -2.6479325E38F, -2.2246345E37F}, 0) ;
            p138.x = (float)4.8594123E37F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {5.166161E37F, -1.1535433E38F, -2.8568733E38F, 6.6868027E37F, -7.899159E37F, 1.827852E38F, 1.6310567E38F, 2.6373473E38F}));
                Debug.Assert(pack.target_component == (byte)(byte)33);
                Debug.Assert(pack.time_usec == (ulong)7972509067436576181L);
                Debug.Assert(pack.group_mlx == (byte)(byte)251);
                Debug.Assert(pack.target_system == (byte)(byte)71);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_system = (byte)(byte)71;
            p139.target_component = (byte)(byte)33;
            p139.controls_SET(new float[] {5.166161E37F, -1.1535433E38F, -2.8568733E38F, 6.6868027E37F, -7.899159E37F, 1.827852E38F, 1.6310567E38F, 2.6373473E38F}, 0) ;
            p139.group_mlx = (byte)(byte)251;
            p139.time_usec = (ulong)7972509067436576181L;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.9196991E38F, 1.8371247E38F, 3.1112035E37F, -3.2402155E38F, 2.4804805E38F, -2.7605144E38F, 2.2227945E38F, -9.567185E37F}));
                Debug.Assert(pack.time_usec == (ulong)750545957134119564L);
                Debug.Assert(pack.group_mlx == (byte)(byte)3);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)3;
            p140.controls_SET(new float[] {2.9196991E38F, 1.8371247E38F, 3.1112035E37F, -3.2402155E38F, 2.4804805E38F, -2.7605144E38F, 2.2227945E38F, -9.567185E37F}, 0) ;
            p140.time_usec = (ulong)750545957134119564L;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_amsl == (float) -1.404684E38F);
                Debug.Assert(pack.altitude_terrain == (float)1.6876797E38F);
                Debug.Assert(pack.time_usec == (ulong)1272060158156307024L);
                Debug.Assert(pack.altitude_monotonic == (float) -5.832619E37F);
                Debug.Assert(pack.bottom_clearance == (float) -2.2846617E38F);
                Debug.Assert(pack.altitude_relative == (float) -1.7907043E38F);
                Debug.Assert(pack.altitude_local == (float) -2.9213606E37F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_local = (float) -2.9213606E37F;
            p141.altitude_amsl = (float) -1.404684E38F;
            p141.altitude_terrain = (float)1.6876797E38F;
            p141.altitude_monotonic = (float) -5.832619E37F;
            p141.bottom_clearance = (float) -2.2846617E38F;
            p141.altitude_relative = (float) -1.7907043E38F;
            p141.time_usec = (ulong)1272060158156307024L;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_type == (byte)(byte)26);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)88, (byte)26, (byte)111, (byte)254, (byte)67, (byte)69, (byte)113, (byte)75, (byte)147, (byte)204, (byte)168, (byte)112, (byte)118, (byte)7, (byte)6, (byte)147, (byte)117, (byte)124, (byte)3, (byte)231, (byte)240, (byte)73, (byte)185, (byte)8, (byte)173, (byte)135, (byte)253, (byte)13, (byte)239, (byte)219, (byte)197, (byte)140, (byte)53, (byte)50, (byte)121, (byte)4, (byte)8, (byte)65, (byte)249, (byte)209, (byte)45, (byte)227, (byte)149, (byte)244, (byte)3, (byte)160, (byte)50, (byte)175, (byte)145, (byte)104, (byte)168, (byte)34, (byte)220, (byte)38, (byte)26, (byte)44, (byte)13, (byte)125, (byte)107, (byte)1, (byte)184, (byte)85, (byte)62, (byte)228, (byte)80, (byte)88, (byte)232, (byte)95, (byte)157, (byte)174, (byte)120, (byte)215, (byte)144, (byte)185, (byte)111, (byte)148, (byte)21, (byte)114, (byte)236, (byte)62, (byte)36, (byte)49, (byte)227, (byte)132, (byte)126, (byte)40, (byte)7, (byte)90, (byte)236, (byte)102, (byte)141, (byte)31, (byte)29, (byte)43, (byte)87, (byte)100, (byte)75, (byte)193, (byte)216, (byte)224, (byte)96, (byte)133, (byte)242, (byte)56, (byte)148, (byte)50, (byte)0, (byte)42, (byte)44, (byte)81, (byte)206, (byte)196, (byte)248, (byte)235, (byte)83, (byte)1, (byte)101, (byte)233, (byte)153, (byte)205}));
                Debug.Assert(pack.transfer_type == (byte)(byte)147);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)216, (byte)14, (byte)32, (byte)130, (byte)163, (byte)162, (byte)238, (byte)199, (byte)148, (byte)92, (byte)165, (byte)8, (byte)101, (byte)79, (byte)194, (byte)195, (byte)99, (byte)37, (byte)212, (byte)133, (byte)192, (byte)141, (byte)140, (byte)94, (byte)101, (byte)124, (byte)72, (byte)189, (byte)41, (byte)220, (byte)96, (byte)4, (byte)187, (byte)86, (byte)16, (byte)62, (byte)67, (byte)213, (byte)26, (byte)19, (byte)226, (byte)208, (byte)121, (byte)3, (byte)59, (byte)96, (byte)119, (byte)238, (byte)34, (byte)245, (byte)205, (byte)193, (byte)110, (byte)33, (byte)225, (byte)107, (byte)115, (byte)104, (byte)116, (byte)35, (byte)235, (byte)93, (byte)92, (byte)3, (byte)119, (byte)217, (byte)73, (byte)135, (byte)57, (byte)88, (byte)174, (byte)207, (byte)98, (byte)92, (byte)29, (byte)104, (byte)143, (byte)123, (byte)160, (byte)70, (byte)187, (byte)194, (byte)65, (byte)97, (byte)243, (byte)23, (byte)94, (byte)224, (byte)27, (byte)145, (byte)203, (byte)249, (byte)49, (byte)165, (byte)129, (byte)27, (byte)6, (byte)163, (byte)51, (byte)132, (byte)200, (byte)232, (byte)60, (byte)16, (byte)172, (byte)254, (byte)217, (byte)161, (byte)180, (byte)101, (byte)160, (byte)221, (byte)47, (byte)46, (byte)135, (byte)1, (byte)210, (byte)155, (byte)116, (byte)103}));
                Debug.Assert(pack.request_id == (byte)(byte)54);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.uri_type = (byte)(byte)26;
            p142.request_id = (byte)(byte)54;
            p142.transfer_type = (byte)(byte)147;
            p142.storage_SET(new byte[] {(byte)216, (byte)14, (byte)32, (byte)130, (byte)163, (byte)162, (byte)238, (byte)199, (byte)148, (byte)92, (byte)165, (byte)8, (byte)101, (byte)79, (byte)194, (byte)195, (byte)99, (byte)37, (byte)212, (byte)133, (byte)192, (byte)141, (byte)140, (byte)94, (byte)101, (byte)124, (byte)72, (byte)189, (byte)41, (byte)220, (byte)96, (byte)4, (byte)187, (byte)86, (byte)16, (byte)62, (byte)67, (byte)213, (byte)26, (byte)19, (byte)226, (byte)208, (byte)121, (byte)3, (byte)59, (byte)96, (byte)119, (byte)238, (byte)34, (byte)245, (byte)205, (byte)193, (byte)110, (byte)33, (byte)225, (byte)107, (byte)115, (byte)104, (byte)116, (byte)35, (byte)235, (byte)93, (byte)92, (byte)3, (byte)119, (byte)217, (byte)73, (byte)135, (byte)57, (byte)88, (byte)174, (byte)207, (byte)98, (byte)92, (byte)29, (byte)104, (byte)143, (byte)123, (byte)160, (byte)70, (byte)187, (byte)194, (byte)65, (byte)97, (byte)243, (byte)23, (byte)94, (byte)224, (byte)27, (byte)145, (byte)203, (byte)249, (byte)49, (byte)165, (byte)129, (byte)27, (byte)6, (byte)163, (byte)51, (byte)132, (byte)200, (byte)232, (byte)60, (byte)16, (byte)172, (byte)254, (byte)217, (byte)161, (byte)180, (byte)101, (byte)160, (byte)221, (byte)47, (byte)46, (byte)135, (byte)1, (byte)210, (byte)155, (byte)116, (byte)103}, 0) ;
            p142.uri_SET(new byte[] {(byte)88, (byte)26, (byte)111, (byte)254, (byte)67, (byte)69, (byte)113, (byte)75, (byte)147, (byte)204, (byte)168, (byte)112, (byte)118, (byte)7, (byte)6, (byte)147, (byte)117, (byte)124, (byte)3, (byte)231, (byte)240, (byte)73, (byte)185, (byte)8, (byte)173, (byte)135, (byte)253, (byte)13, (byte)239, (byte)219, (byte)197, (byte)140, (byte)53, (byte)50, (byte)121, (byte)4, (byte)8, (byte)65, (byte)249, (byte)209, (byte)45, (byte)227, (byte)149, (byte)244, (byte)3, (byte)160, (byte)50, (byte)175, (byte)145, (byte)104, (byte)168, (byte)34, (byte)220, (byte)38, (byte)26, (byte)44, (byte)13, (byte)125, (byte)107, (byte)1, (byte)184, (byte)85, (byte)62, (byte)228, (byte)80, (byte)88, (byte)232, (byte)95, (byte)157, (byte)174, (byte)120, (byte)215, (byte)144, (byte)185, (byte)111, (byte)148, (byte)21, (byte)114, (byte)236, (byte)62, (byte)36, (byte)49, (byte)227, (byte)132, (byte)126, (byte)40, (byte)7, (byte)90, (byte)236, (byte)102, (byte)141, (byte)31, (byte)29, (byte)43, (byte)87, (byte)100, (byte)75, (byte)193, (byte)216, (byte)224, (byte)96, (byte)133, (byte)242, (byte)56, (byte)148, (byte)50, (byte)0, (byte)42, (byte)44, (byte)81, (byte)206, (byte)196, (byte)248, (byte)235, (byte)83, (byte)1, (byte)101, (byte)233, (byte)153, (byte)205}, 0) ;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float)2.4780115E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1772091086U);
                Debug.Assert(pack.temperature == (short)(short) -11375);
                Debug.Assert(pack.press_abs == (float) -3.2329402E38F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.temperature = (short)(short) -11375;
            p143.time_boot_ms = (uint)1772091086U;
            p143.press_diff = (float)2.4780115E38F;
            p143.press_abs = (float) -3.2329402E38F;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {2.280763E38F, -2.0893145E38F, 1.1083171E38F}));
                Debug.Assert(pack.lon == (int) -704501816);
                Debug.Assert(pack.alt == (float) -1.8219788E38F);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-3.1054117E38F, -3.1313327E38F, 2.5611858E38F}));
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {3.310609E38F, 9.803685E37F, 2.1092787E38F, -2.383456E38F}));
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-1.0876328E38F, 2.5596656E38F, 2.3671807E38F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)236);
                Debug.Assert(pack.custom_state == (ulong)2757113185313973530L);
                Debug.Assert(pack.lat == (int)1122236441);
                Debug.Assert(pack.timestamp == (ulong)8688082779938473902L);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-1.1849151E37F, -2.8315414E38F, -2.3562958E38F}));
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.lon = (int) -704501816;
            p144.timestamp = (ulong)8688082779938473902L;
            p144.vel_SET(new float[] {-1.1849151E37F, -2.8315414E38F, -2.3562958E38F}, 0) ;
            p144.acc_SET(new float[] {-3.1054117E38F, -3.1313327E38F, 2.5611858E38F}, 0) ;
            p144.custom_state = (ulong)2757113185313973530L;
            p144.attitude_q_SET(new float[] {3.310609E38F, 9.803685E37F, 2.1092787E38F, -2.383456E38F}, 0) ;
            p144.rates_SET(new float[] {-1.0876328E38F, 2.5596656E38F, 2.3671807E38F}, 0) ;
            p144.est_capabilities = (byte)(byte)236;
            p144.lat = (int)1122236441;
            p144.position_cov_SET(new float[] {2.280763E38F, -2.0893145E38F, 1.1083171E38F}, 0) ;
            p144.alt = (float) -1.8219788E38F;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y_pos == (float)2.4468188E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-1.1757279E37F, -1.311215E38F, -2.523963E38F}));
                Debug.Assert(pack.yaw_rate == (float) -3.9671113E37F);
                Debug.Assert(pack.roll_rate == (float) -3.192105E38F);
                Debug.Assert(pack.x_pos == (float)1.5860909E38F);
                Debug.Assert(pack.y_acc == (float)1.814273E38F);
                Debug.Assert(pack.time_usec == (ulong)3012317966993929952L);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {2.3631736E36F, 1.1711402E38F, -2.5723456E38F}));
                Debug.Assert(pack.x_vel == (float) -7.299154E37F);
                Debug.Assert(pack.y_vel == (float) -2.2590974E38F);
                Debug.Assert(pack.z_acc == (float) -1.5124828E38F);
                Debug.Assert(pack.x_acc == (float)1.0678231E38F);
                Debug.Assert(pack.z_vel == (float)2.5956945E38F);
                Debug.Assert(pack.pitch_rate == (float)1.0872205E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.8589894E38F, 1.6619125E38F, 2.5189455E38F, -1.1775283E38F}));
                Debug.Assert(pack.z_pos == (float) -4.4060924E37F);
                Debug.Assert(pack.airspeed == (float) -1.6316439E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.z_acc = (float) -1.5124828E38F;
            p146.pos_variance_SET(new float[] {2.3631736E36F, 1.1711402E38F, -2.5723456E38F}, 0) ;
            p146.time_usec = (ulong)3012317966993929952L;
            p146.yaw_rate = (float) -3.9671113E37F;
            p146.y_pos = (float)2.4468188E38F;
            p146.z_vel = (float)2.5956945E38F;
            p146.z_pos = (float) -4.4060924E37F;
            p146.y_acc = (float)1.814273E38F;
            p146.pitch_rate = (float)1.0872205E38F;
            p146.vel_variance_SET(new float[] {-1.1757279E37F, -1.311215E38F, -2.523963E38F}, 0) ;
            p146.x_pos = (float)1.5860909E38F;
            p146.x_acc = (float)1.0678231E38F;
            p146.q_SET(new float[] {2.8589894E38F, 1.6619125E38F, 2.5189455E38F, -1.1775283E38F}, 0) ;
            p146.y_vel = (float) -2.2590974E38F;
            p146.x_vel = (float) -7.299154E37F;
            p146.roll_rate = (float) -3.192105E38F;
            p146.airspeed = (float) -1.6316439E38F;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_consumed == (int)1722683745);
                Debug.Assert(pack.temperature == (short)(short) -9703);
                Debug.Assert(pack.id == (byte)(byte)6);
                Debug.Assert(pack.energy_consumed == (int)765950384);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)38851, (ushort)24177, (ushort)28485, (ushort)48452, (ushort)60806, (ushort)39749, (ushort)64368, (ushort)49123, (ushort)23966, (ushort)54498}));
                Debug.Assert(pack.current_battery == (short)(short) -30008);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 125);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.voltages_SET(new ushort[] {(ushort)38851, (ushort)24177, (ushort)28485, (ushort)48452, (ushort)60806, (ushort)39749, (ushort)64368, (ushort)49123, (ushort)23966, (ushort)54498}, 0) ;
            p147.id = (byte)(byte)6;
            p147.temperature = (short)(short) -9703;
            p147.battery_remaining = (sbyte)(sbyte) - 125;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN;
            p147.energy_consumed = (int)765950384;
            p147.current_battery = (short)(short) -30008;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN;
            p147.current_consumed = (int)1722683745;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)76, (byte)218, (byte)127, (byte)41, (byte)94, (byte)172, (byte)21, (byte)191}));
                Debug.Assert(pack.board_version == (uint)4038434734U);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)46734);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)175, (byte)57, (byte)148, (byte)117, (byte)26, (byte)200, (byte)14, (byte)150, (byte)242, (byte)236, (byte)56, (byte)157, (byte)188, (byte)110, (byte)211, (byte)103, (byte)9, (byte)241}));
                Debug.Assert(pack.uid == (ulong)2633965622698956337L);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)63, (byte)27, (byte)237, (byte)146, (byte)193, (byte)171, (byte)241, (byte)87}));
                Debug.Assert(pack.product_id == (ushort)(ushort)42281);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)136, (byte)6, (byte)8, (byte)40, (byte)205, (byte)1, (byte)109, (byte)189}));
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT);
                Debug.Assert(pack.middleware_sw_version == (uint)3739070728U);
                Debug.Assert(pack.os_sw_version == (uint)4112051610U);
                Debug.Assert(pack.flight_sw_version == (uint)2069646516U);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.os_sw_version = (uint)4112051610U;
            p148.flight_sw_version = (uint)2069646516U;
            p148.uid2_SET(new byte[] {(byte)175, (byte)57, (byte)148, (byte)117, (byte)26, (byte)200, (byte)14, (byte)150, (byte)242, (byte)236, (byte)56, (byte)157, (byte)188, (byte)110, (byte)211, (byte)103, (byte)9, (byte)241}, 0, PH) ;
            p148.vendor_id = (ushort)(ushort)46734;
            p148.uid = (ulong)2633965622698956337L;
            p148.middleware_sw_version = (uint)3739070728U;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
            p148.board_version = (uint)4038434734U;
            p148.product_id = (ushort)(ushort)42281;
            p148.os_custom_version_SET(new byte[] {(byte)136, (byte)6, (byte)8, (byte)40, (byte)205, (byte)1, (byte)109, (byte)189}, 0) ;
            p148.flight_custom_version_SET(new byte[] {(byte)76, (byte)218, (byte)127, (byte)41, (byte)94, (byte)172, (byte)21, (byte)191}, 0) ;
            p148.middleware_custom_version_SET(new byte[] {(byte)63, (byte)27, (byte)237, (byte)146, (byte)193, (byte)171, (byte)241, (byte)87}, 0) ;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.angle_x == (float)2.040045E38F);
                Debug.Assert(pack.distance == (float)5.5698073E37F);
                Debug.Assert(pack.z_TRY(ph) == (float) -5.2321695E37F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {1.5281172E38F, 3.6279275E37F, 6.2103673E37F, 9.137131E37F}));
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)118);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
                Debug.Assert(pack.x_TRY(ph) == (float)2.427686E38F);
                Debug.Assert(pack.y_TRY(ph) == (float) -2.3030303E38F);
                Debug.Assert(pack.angle_y == (float)6.9012126E37F);
                Debug.Assert(pack.target_num == (byte)(byte)74);
                Debug.Assert(pack.size_y == (float) -4.8414463E36F);
                Debug.Assert(pack.size_x == (float)1.1965129E38F);
                Debug.Assert(pack.time_usec == (ulong)1455108749684431525L);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.y_SET((float) -2.3030303E38F, PH) ;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.q_SET(new float[] {1.5281172E38F, 3.6279275E37F, 6.2103673E37F, 9.137131E37F}, 0, PH) ;
            p149.time_usec = (ulong)1455108749684431525L;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p149.position_valid_SET((byte)(byte)118, PH) ;
            p149.size_x = (float)1.1965129E38F;
            p149.z_SET((float) -5.2321695E37F, PH) ;
            p149.angle_y = (float)6.9012126E37F;
            p149.distance = (float)5.5698073E37F;
            p149.angle_x = (float)2.040045E38F;
            p149.x_SET((float)2.427686E38F, PH) ;
            p149.size_y = (float) -4.8414463E36F;
            p149.target_num = (byte)(byte)74;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAV_FILTER_BIASReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.accel_2 == (float) -1.1523632E38F);
                Debug.Assert(pack.gyro_2 == (float) -3.3837053E38F);
                Debug.Assert(pack.gyro_1 == (float)2.3791545E37F);
                Debug.Assert(pack.accel_1 == (float)1.0680583E38F);
                Debug.Assert(pack.gyro_0 == (float) -1.1647769E38F);
                Debug.Assert(pack.accel_0 == (float) -2.4971534E38F);
                Debug.Assert(pack.usec == (ulong)3468643463803602005L);
            };
            GroundControl.NAV_FILTER_BIAS p220 = CommunicationChannel.new_NAV_FILTER_BIAS();
            PH.setPack(p220);
            p220.accel_1 = (float)1.0680583E38F;
            p220.gyro_1 = (float)2.3791545E37F;
            p220.usec = (ulong)3468643463803602005L;
            p220.gyro_0 = (float) -1.1647769E38F;
            p220.gyro_2 = (float) -3.3837053E38F;
            p220.accel_2 = (float) -1.1523632E38F;
            p220.accel_0 = (float) -2.4971534E38F;
            CommunicationChannel.instance.send(p220);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRADIO_CALIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aileron.SequenceEqual(new ushort[] {(ushort)44253, (ushort)40747, (ushort)42248}));
                Debug.Assert(pack.throttle.SequenceEqual(new ushort[] {(ushort)22023, (ushort)20280, (ushort)27749, (ushort)25217, (ushort)11713}));
                Debug.Assert(pack.elevator.SequenceEqual(new ushort[] {(ushort)15651, (ushort)59338, (ushort)45824}));
                Debug.Assert(pack.pitch.SequenceEqual(new ushort[] {(ushort)45179, (ushort)12162, (ushort)47152, (ushort)16225, (ushort)59356}));
                Debug.Assert(pack.gyro.SequenceEqual(new ushort[] {(ushort)1261, (ushort)12860}));
                Debug.Assert(pack.rudder.SequenceEqual(new ushort[] {(ushort)44048, (ushort)16821, (ushort)49456}));
            };
            GroundControl.RADIO_CALIBRATION p221 = CommunicationChannel.new_RADIO_CALIBRATION();
            PH.setPack(p221);
            p221.rudder_SET(new ushort[] {(ushort)44048, (ushort)16821, (ushort)49456}, 0) ;
            p221.gyro_SET(new ushort[] {(ushort)1261, (ushort)12860}, 0) ;
            p221.throttle_SET(new ushort[] {(ushort)22023, (ushort)20280, (ushort)27749, (ushort)25217, (ushort)11713}, 0) ;
            p221.aileron_SET(new ushort[] {(ushort)44253, (ushort)40747, (ushort)42248}, 0) ;
            p221.elevator_SET(new ushort[] {(ushort)15651, (ushort)59338, (ushort)45824}, 0) ;
            p221.pitch_SET(new ushort[] {(ushort)45179, (ushort)12162, (ushort)47152, (ushort)16225, (ushort)59356}, 0) ;
            CommunicationChannel.instance.send(p221);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUALBERTA_SYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pilot == (byte)(byte)170);
                Debug.Assert(pack.mode == (byte)(byte)223);
                Debug.Assert(pack.nav_mode == (byte)(byte)183);
            };
            GroundControl.UALBERTA_SYS_STATUS p222 = CommunicationChannel.new_UALBERTA_SYS_STATUS();
            PH.setPack(p222);
            p222.nav_mode = (byte)(byte)183;
            p222.pilot = (byte)(byte)170;
            p222.mode = (byte)(byte)223;
            CommunicationChannel.instance.send(p222);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1774850286664036328L);
                Debug.Assert(pack.pos_vert_ratio == (float)1.4907492E38F);
                Debug.Assert(pack.mag_ratio == (float)1.9885055E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -1.6452163E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH);
                Debug.Assert(pack.pos_vert_accuracy == (float) -1.2793376E37F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)2.667135E38F);
                Debug.Assert(pack.vel_ratio == (float)2.6029465E38F);
                Debug.Assert(pack.tas_ratio == (float) -2.4579968E38F);
                Debug.Assert(pack.hagl_ratio == (float)2.1672868E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH;
            p230.time_usec = (ulong)1774850286664036328L;
            p230.pos_vert_ratio = (float)1.4907492E38F;
            p230.mag_ratio = (float)1.9885055E38F;
            p230.vel_ratio = (float)2.6029465E38F;
            p230.tas_ratio = (float) -2.4579968E38F;
            p230.hagl_ratio = (float)2.1672868E38F;
            p230.pos_horiz_ratio = (float) -1.6452163E38F;
            p230.pos_horiz_accuracy = (float)2.667135E38F;
            p230.pos_vert_accuracy = (float) -1.2793376E37F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wind_y == (float) -2.3901436E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -3.4015146E38F);
                Debug.Assert(pack.vert_accuracy == (float)9.139372E37F);
                Debug.Assert(pack.wind_z == (float)2.2862573E38F);
                Debug.Assert(pack.var_vert == (float)1.314002E35F);
                Debug.Assert(pack.var_horiz == (float) -1.5605346E38F);
                Debug.Assert(pack.wind_x == (float) -6.3357517E37F);
                Debug.Assert(pack.wind_alt == (float) -3.3798022E37F);
                Debug.Assert(pack.time_usec == (ulong)4530997569262329712L);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.vert_accuracy = (float)9.139372E37F;
            p231.wind_alt = (float) -3.3798022E37F;
            p231.wind_z = (float)2.2862573E38F;
            p231.var_horiz = (float) -1.5605346E38F;
            p231.wind_x = (float) -6.3357517E37F;
            p231.time_usec = (ulong)4530997569262329712L;
            p231.var_vert = (float)1.314002E35F;
            p231.horiz_accuracy = (float) -3.4015146E38F;
            p231.wind_y = (float) -2.3901436E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdop == (float)2.7161663E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)214);
                Debug.Assert(pack.fix_type == (byte)(byte)183);
                Debug.Assert(pack.vert_accuracy == (float)1.0838245E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)35287);
                Debug.Assert(pack.time_usec == (ulong)3015110791897279941L);
                Debug.Assert(pack.lat == (int)56533832);
                Debug.Assert(pack.speed_accuracy == (float)4.1316326E37F);
                Debug.Assert(pack.time_week_ms == (uint)935454859U);
                Debug.Assert(pack.alt == (float) -3.1061628E38F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)240);
                Debug.Assert(pack.horiz_accuracy == (float)2.2405749E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY);
                Debug.Assert(pack.vdop == (float) -6.2601936E37F);
                Debug.Assert(pack.lon == (int)266291632);
                Debug.Assert(pack.ve == (float) -1.6836638E38F);
                Debug.Assert(pack.vn == (float) -1.6885174E37F);
                Debug.Assert(pack.vd == (float) -2.2148584E38F);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)3015110791897279941L;
            p232.time_week_ms = (uint)935454859U;
            p232.hdop = (float)2.7161663E38F;
            p232.fix_type = (byte)(byte)183;
            p232.ve = (float) -1.6836638E38F;
            p232.vn = (float) -1.6885174E37F;
            p232.horiz_accuracy = (float)2.2405749E38F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY;
            p232.vdop = (float) -6.2601936E37F;
            p232.alt = (float) -3.1061628E38F;
            p232.gps_id = (byte)(byte)214;
            p232.speed_accuracy = (float)4.1316326E37F;
            p232.vd = (float) -2.2148584E38F;
            p232.vert_accuracy = (float)1.0838245E38F;
            p232.satellites_visible = (byte)(byte)240;
            p232.lon = (int)266291632;
            p232.time_week = (ushort)(ushort)35287;
            p232.lat = (int)56533832;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)86);
                Debug.Assert(pack.flags == (byte)(byte)98);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)143, (byte)132, (byte)146, (byte)237, (byte)235, (byte)82, (byte)187, (byte)92, (byte)217, (byte)153, (byte)68, (byte)109, (byte)178, (byte)36, (byte)253, (byte)10, (byte)153, (byte)6, (byte)9, (byte)144, (byte)196, (byte)103, (byte)42, (byte)125, (byte)157, (byte)154, (byte)199, (byte)14, (byte)137, (byte)128, (byte)69, (byte)8, (byte)176, (byte)48, (byte)78, (byte)69, (byte)129, (byte)231, (byte)37, (byte)152, (byte)22, (byte)246, (byte)8, (byte)175, (byte)123, (byte)161, (byte)23, (byte)200, (byte)196, (byte)197, (byte)221, (byte)198, (byte)109, (byte)249, (byte)217, (byte)194, (byte)152, (byte)227, (byte)187, (byte)66, (byte)148, (byte)2, (byte)191, (byte)133, (byte)106, (byte)204, (byte)110, (byte)138, (byte)82, (byte)114, (byte)254, (byte)167, (byte)98, (byte)29, (byte)34, (byte)253, (byte)5, (byte)104, (byte)166, (byte)111, (byte)49, (byte)82, (byte)8, (byte)238, (byte)181, (byte)62, (byte)249, (byte)158, (byte)136, (byte)33, (byte)34, (byte)14, (byte)217, (byte)50, (byte)195, (byte)196, (byte)108, (byte)3, (byte)249, (byte)233, (byte)122, (byte)242, (byte)242, (byte)137, (byte)48, (byte)197, (byte)39, (byte)123, (byte)121, (byte)173, (byte)56, (byte)34, (byte)242, (byte)81, (byte)221, (byte)174, (byte)51, (byte)186, (byte)63, (byte)184, (byte)195, (byte)137, (byte)14, (byte)169, (byte)30, (byte)50, (byte)105, (byte)131, (byte)219, (byte)164, (byte)59, (byte)144, (byte)94, (byte)200, (byte)146, (byte)96, (byte)113, (byte)30, (byte)228, (byte)101, (byte)9, (byte)161, (byte)22, (byte)138, (byte)191, (byte)182, (byte)51, (byte)181, (byte)43, (byte)111, (byte)100, (byte)237, (byte)104, (byte)199, (byte)119, (byte)153, (byte)59, (byte)123, (byte)63, (byte)135, (byte)24, (byte)12, (byte)106, (byte)106, (byte)66, (byte)180, (byte)179, (byte)98, (byte)27, (byte)4, (byte)29, (byte)39, (byte)189, (byte)60, (byte)98, (byte)177, (byte)224, (byte)129, (byte)46, (byte)138}));
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.len = (byte)(byte)86;
            p233.data__SET(new byte[] {(byte)143, (byte)132, (byte)146, (byte)237, (byte)235, (byte)82, (byte)187, (byte)92, (byte)217, (byte)153, (byte)68, (byte)109, (byte)178, (byte)36, (byte)253, (byte)10, (byte)153, (byte)6, (byte)9, (byte)144, (byte)196, (byte)103, (byte)42, (byte)125, (byte)157, (byte)154, (byte)199, (byte)14, (byte)137, (byte)128, (byte)69, (byte)8, (byte)176, (byte)48, (byte)78, (byte)69, (byte)129, (byte)231, (byte)37, (byte)152, (byte)22, (byte)246, (byte)8, (byte)175, (byte)123, (byte)161, (byte)23, (byte)200, (byte)196, (byte)197, (byte)221, (byte)198, (byte)109, (byte)249, (byte)217, (byte)194, (byte)152, (byte)227, (byte)187, (byte)66, (byte)148, (byte)2, (byte)191, (byte)133, (byte)106, (byte)204, (byte)110, (byte)138, (byte)82, (byte)114, (byte)254, (byte)167, (byte)98, (byte)29, (byte)34, (byte)253, (byte)5, (byte)104, (byte)166, (byte)111, (byte)49, (byte)82, (byte)8, (byte)238, (byte)181, (byte)62, (byte)249, (byte)158, (byte)136, (byte)33, (byte)34, (byte)14, (byte)217, (byte)50, (byte)195, (byte)196, (byte)108, (byte)3, (byte)249, (byte)233, (byte)122, (byte)242, (byte)242, (byte)137, (byte)48, (byte)197, (byte)39, (byte)123, (byte)121, (byte)173, (byte)56, (byte)34, (byte)242, (byte)81, (byte)221, (byte)174, (byte)51, (byte)186, (byte)63, (byte)184, (byte)195, (byte)137, (byte)14, (byte)169, (byte)30, (byte)50, (byte)105, (byte)131, (byte)219, (byte)164, (byte)59, (byte)144, (byte)94, (byte)200, (byte)146, (byte)96, (byte)113, (byte)30, (byte)228, (byte)101, (byte)9, (byte)161, (byte)22, (byte)138, (byte)191, (byte)182, (byte)51, (byte)181, (byte)43, (byte)111, (byte)100, (byte)237, (byte)104, (byte)199, (byte)119, (byte)153, (byte)59, (byte)123, (byte)63, (byte)135, (byte)24, (byte)12, (byte)106, (byte)106, (byte)66, (byte)180, (byte)179, (byte)98, (byte)27, (byte)4, (byte)29, (byte)39, (byte)189, (byte)60, (byte)98, (byte)177, (byte)224, (byte)129, (byte)46, (byte)138}, 0) ;
            p233.flags = (byte)(byte)98;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.airspeed_sp == (byte)(byte)21);
                Debug.Assert(pack.failsafe == (byte)(byte)58);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)88);
                Debug.Assert(pack.gps_nsat == (byte)(byte)243);
                Debug.Assert(pack.longitude == (int)410707934);
                Debug.Assert(pack.roll == (short)(short) -28337);
                Debug.Assert(pack.battery_remaining == (byte)(byte)206);
                Debug.Assert(pack.airspeed == (byte)(byte)236);
                Debug.Assert(pack.groundspeed == (byte)(byte)181);
                Debug.Assert(pack.pitch == (short)(short)22727);
                Debug.Assert(pack.wp_num == (byte)(byte)4);
                Debug.Assert(pack.altitude_amsl == (short)(short)7153);
                Debug.Assert(pack.heading_sp == (short)(short)4617);
                Debug.Assert(pack.heading == (ushort)(ushort)52696);
                Debug.Assert(pack.altitude_sp == (short)(short)18598);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 72);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)61);
                Debug.Assert(pack.latitude == (int) -1766769380);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)60423);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 5);
                Debug.Assert(pack.custom_mode == (uint)1575399547U);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.failsafe = (byte)(byte)58;
            p234.wp_num = (byte)(byte)4;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p234.gps_nsat = (byte)(byte)243;
            p234.temperature_air = (sbyte)(sbyte)61;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED;
            p234.climb_rate = (sbyte)(sbyte) - 72;
            p234.heading_sp = (short)(short)4617;
            p234.heading = (ushort)(ushort)52696;
            p234.pitch = (short)(short)22727;
            p234.altitude_amsl = (short)(short)7153;
            p234.wp_distance = (ushort)(ushort)60423;
            p234.roll = (short)(short) -28337;
            p234.latitude = (int) -1766769380;
            p234.longitude = (int)410707934;
            p234.throttle = (sbyte)(sbyte)88;
            p234.temperature = (sbyte)(sbyte) - 5;
            p234.airspeed = (byte)(byte)236;
            p234.battery_remaining = (byte)(byte)206;
            p234.custom_mode = (uint)1575399547U;
            p234.airspeed_sp = (byte)(byte)21;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p234.groundspeed = (byte)(byte)181;
            p234.altitude_sp = (short)(short)18598;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_0 == (uint)3120676365U);
                Debug.Assert(pack.clipping_2 == (uint)1315852089U);
                Debug.Assert(pack.time_usec == (ulong)3583181304190515696L);
                Debug.Assert(pack.clipping_1 == (uint)564877955U);
                Debug.Assert(pack.vibration_y == (float) -1.2008372E38F);
                Debug.Assert(pack.vibration_x == (float)2.1216362E38F);
                Debug.Assert(pack.vibration_z == (float)3.2006912E38F);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_0 = (uint)3120676365U;
            p241.clipping_2 = (uint)1315852089U;
            p241.vibration_z = (float)3.2006912E38F;
            p241.time_usec = (ulong)3583181304190515696L;
            p241.clipping_1 = (uint)564877955U;
            p241.vibration_x = (float)2.1216362E38F;
            p241.vibration_y = (float) -1.2008372E38F;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -1.0242078E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2388541134137695032L);
                Debug.Assert(pack.approach_y == (float) -7.548087E37F);
                Debug.Assert(pack.approach_z == (float)2.2726231E38F);
                Debug.Assert(pack.altitude == (int) -818665096);
                Debug.Assert(pack.y == (float) -1.5211278E38F);
                Debug.Assert(pack.longitude == (int)703147752);
                Debug.Assert(pack.approach_x == (float)2.644665E38F);
                Debug.Assert(pack.x == (float)2.4228178E38F);
                Debug.Assert(pack.latitude == (int)1151194903);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.1745481E38F, -3.3426383E38F, -7.112759E37F, 2.754077E38F}));
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.y = (float) -1.5211278E38F;
            p242.x = (float)2.4228178E38F;
            p242.approach_z = (float)2.2726231E38F;
            p242.z = (float) -1.0242078E38F;
            p242.q_SET(new float[] {2.1745481E38F, -3.3426383E38F, -7.112759E37F, 2.754077E38F}, 0) ;
            p242.approach_x = (float)2.644665E38F;
            p242.time_usec_SET((ulong)2388541134137695032L, PH) ;
            p242.latitude = (int)1151194903;
            p242.longitude = (int)703147752;
            p242.approach_y = (float) -7.548087E37F;
            p242.altitude = (int) -818665096;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -1.6371299E38F);
                Debug.Assert(pack.x == (float) -5.845643E36F);
                Debug.Assert(pack.latitude == (int)2045086449);
                Debug.Assert(pack.approach_y == (float)2.7304898E38F);
                Debug.Assert(pack.approach_z == (float) -1.4828164E38F);
                Debug.Assert(pack.longitude == (int)359379632);
                Debug.Assert(pack.target_system == (byte)(byte)227);
                Debug.Assert(pack.altitude == (int)372442218);
                Debug.Assert(pack.z == (float)7.859048E37F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3853143949919514557L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.4904063E38F, -3.0393266E38F, -3.8126047E37F, 2.050865E38F}));
                Debug.Assert(pack.approach_x == (float) -9.796559E37F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.approach_x = (float) -9.796559E37F;
            p243.q_SET(new float[] {1.4904063E38F, -3.0393266E38F, -3.8126047E37F, 2.050865E38F}, 0) ;
            p243.approach_z = (float) -1.4828164E38F;
            p243.z = (float)7.859048E37F;
            p243.approach_y = (float)2.7304898E38F;
            p243.time_usec_SET((ulong)3853143949919514557L, PH) ;
            p243.altitude = (int)372442218;
            p243.target_system = (byte)(byte)227;
            p243.longitude = (int)359379632;
            p243.y = (float) -1.6371299E38F;
            p243.latitude = (int)2045086449;
            p243.x = (float) -5.845643E36F;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int) -1578017582);
                Debug.Assert(pack.message_id == (ushort)(ushort)1605);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int) -1578017582;
            p244.message_id = (ushort)(ushort)1605;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC;
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)19639);
                Debug.Assert(pack.altitude == (int)2136247715);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING);
                Debug.Assert(pack.lat == (int)1141791410);
                Debug.Assert(pack.lon == (int)1642307905);
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_EMERGENCY_SURFACE);
                Debug.Assert(pack.tslc == (byte)(byte)41);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.squawk == (ushort)(ushort)35535);
                Debug.Assert(pack.callsign_LEN(ph) == 5);
                Debug.Assert(pack.callsign_TRY(ph).Equals("uuiTk"));
                Debug.Assert(pack.ver_velocity == (short)(short)28923);
                Debug.Assert(pack.heading == (ushort)(ushort)45705);
                Debug.Assert(pack.ICAO_address == (uint)4185551741U);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.callsign_SET("uuiTk", PH) ;
            p246.heading = (ushort)(ushort)45705;
            p246.ICAO_address = (uint)4185551741U;
            p246.lon = (int)1642307905;
            p246.lat = (int)1141791410;
            p246.tslc = (byte)(byte)41;
            p246.hor_velocity = (ushort)(ushort)19639;
            p246.ver_velocity = (short)(short)28923;
            p246.altitude = (int)2136247715;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.squawk = (ushort)(ushort)35535;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_EMERGENCY_SURFACE;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horizontal_minimum_delta == (float)3.1508E38F);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.altitude_minimum_delta == (float)1.293692E38F);
                Debug.Assert(pack.id == (uint)177947890U);
                Debug.Assert(pack.time_to_minimum_delta == (float) -1.3008544E38F);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.horizontal_minimum_delta = (float)3.1508E38F;
            p247.id = (uint)177947890U;
            p247.time_to_minimum_delta = (float) -1.3008544E38F;
            p247.altitude_minimum_delta = (float)1.293692E38F;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_type == (ushort)(ushort)60236);
                Debug.Assert(pack.target_system == (byte)(byte)20);
                Debug.Assert(pack.target_network == (byte)(byte)69);
                Debug.Assert(pack.target_component == (byte)(byte)131);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)77, (byte)103, (byte)231, (byte)193, (byte)175, (byte)27, (byte)134, (byte)232, (byte)231, (byte)177, (byte)26, (byte)126, (byte)155, (byte)20, (byte)119, (byte)90, (byte)234, (byte)67, (byte)204, (byte)151, (byte)50, (byte)88, (byte)210, (byte)31, (byte)241, (byte)232, (byte)247, (byte)76, (byte)189, (byte)66, (byte)48, (byte)13, (byte)243, (byte)72, (byte)110, (byte)135, (byte)15, (byte)219, (byte)173, (byte)198, (byte)97, (byte)223, (byte)240, (byte)50, (byte)22, (byte)207, (byte)14, (byte)215, (byte)67, (byte)49, (byte)243, (byte)112, (byte)71, (byte)116, (byte)227, (byte)228, (byte)27, (byte)67, (byte)36, (byte)207, (byte)30, (byte)78, (byte)252, (byte)218, (byte)36, (byte)255, (byte)193, (byte)30, (byte)217, (byte)3, (byte)125, (byte)249, (byte)251, (byte)222, (byte)181, (byte)146, (byte)241, (byte)132, (byte)246, (byte)106, (byte)204, (byte)0, (byte)24, (byte)108, (byte)94, (byte)14, (byte)180, (byte)230, (byte)30, (byte)43, (byte)144, (byte)180, (byte)110, (byte)111, (byte)12, (byte)80, (byte)49, (byte)66, (byte)233, (byte)62, (byte)30, (byte)239, (byte)136, (byte)35, (byte)62, (byte)2, (byte)212, (byte)184, (byte)243, (byte)77, (byte)117, (byte)150, (byte)149, (byte)233, (byte)74, (byte)49, (byte)202, (byte)131, (byte)175, (byte)226, (byte)49, (byte)37, (byte)233, (byte)216, (byte)13, (byte)182, (byte)119, (byte)246, (byte)142, (byte)114, (byte)235, (byte)20, (byte)235, (byte)45, (byte)222, (byte)243, (byte)234, (byte)236, (byte)31, (byte)1, (byte)56, (byte)62, (byte)163, (byte)173, (byte)3, (byte)251, (byte)133, (byte)85, (byte)209, (byte)97, (byte)198, (byte)126, (byte)49, (byte)27, (byte)145, (byte)24, (byte)161, (byte)201, (byte)211, (byte)12, (byte)108, (byte)246, (byte)60, (byte)239, (byte)167, (byte)221, (byte)240, (byte)111, (byte)141, (byte)137, (byte)116, (byte)132, (byte)20, (byte)68, (byte)46, (byte)95, (byte)241, (byte)221, (byte)49, (byte)45, (byte)204, (byte)42, (byte)213, (byte)50, (byte)125, (byte)248, (byte)240, (byte)105, (byte)46, (byte)137, (byte)4, (byte)105, (byte)234, (byte)43, (byte)184, (byte)200, (byte)100, (byte)112, (byte)182, (byte)221, (byte)234, (byte)87, (byte)121, (byte)59, (byte)170, (byte)122, (byte)55, (byte)3, (byte)201, (byte)208, (byte)5, (byte)21, (byte)153, (byte)43, (byte)105, (byte)99, (byte)176, (byte)170, (byte)227, (byte)94, (byte)5, (byte)209, (byte)180, (byte)66, (byte)64, (byte)152, (byte)98, (byte)201, (byte)76, (byte)65, (byte)149, (byte)80, (byte)191, (byte)195, (byte)189, (byte)54, (byte)80, (byte)243, (byte)223, (byte)206, (byte)2, (byte)14, (byte)181, (byte)59, (byte)220, (byte)179, (byte)89, (byte)213, (byte)26}));
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.message_type = (ushort)(ushort)60236;
            p248.target_component = (byte)(byte)131;
            p248.payload_SET(new byte[] {(byte)77, (byte)103, (byte)231, (byte)193, (byte)175, (byte)27, (byte)134, (byte)232, (byte)231, (byte)177, (byte)26, (byte)126, (byte)155, (byte)20, (byte)119, (byte)90, (byte)234, (byte)67, (byte)204, (byte)151, (byte)50, (byte)88, (byte)210, (byte)31, (byte)241, (byte)232, (byte)247, (byte)76, (byte)189, (byte)66, (byte)48, (byte)13, (byte)243, (byte)72, (byte)110, (byte)135, (byte)15, (byte)219, (byte)173, (byte)198, (byte)97, (byte)223, (byte)240, (byte)50, (byte)22, (byte)207, (byte)14, (byte)215, (byte)67, (byte)49, (byte)243, (byte)112, (byte)71, (byte)116, (byte)227, (byte)228, (byte)27, (byte)67, (byte)36, (byte)207, (byte)30, (byte)78, (byte)252, (byte)218, (byte)36, (byte)255, (byte)193, (byte)30, (byte)217, (byte)3, (byte)125, (byte)249, (byte)251, (byte)222, (byte)181, (byte)146, (byte)241, (byte)132, (byte)246, (byte)106, (byte)204, (byte)0, (byte)24, (byte)108, (byte)94, (byte)14, (byte)180, (byte)230, (byte)30, (byte)43, (byte)144, (byte)180, (byte)110, (byte)111, (byte)12, (byte)80, (byte)49, (byte)66, (byte)233, (byte)62, (byte)30, (byte)239, (byte)136, (byte)35, (byte)62, (byte)2, (byte)212, (byte)184, (byte)243, (byte)77, (byte)117, (byte)150, (byte)149, (byte)233, (byte)74, (byte)49, (byte)202, (byte)131, (byte)175, (byte)226, (byte)49, (byte)37, (byte)233, (byte)216, (byte)13, (byte)182, (byte)119, (byte)246, (byte)142, (byte)114, (byte)235, (byte)20, (byte)235, (byte)45, (byte)222, (byte)243, (byte)234, (byte)236, (byte)31, (byte)1, (byte)56, (byte)62, (byte)163, (byte)173, (byte)3, (byte)251, (byte)133, (byte)85, (byte)209, (byte)97, (byte)198, (byte)126, (byte)49, (byte)27, (byte)145, (byte)24, (byte)161, (byte)201, (byte)211, (byte)12, (byte)108, (byte)246, (byte)60, (byte)239, (byte)167, (byte)221, (byte)240, (byte)111, (byte)141, (byte)137, (byte)116, (byte)132, (byte)20, (byte)68, (byte)46, (byte)95, (byte)241, (byte)221, (byte)49, (byte)45, (byte)204, (byte)42, (byte)213, (byte)50, (byte)125, (byte)248, (byte)240, (byte)105, (byte)46, (byte)137, (byte)4, (byte)105, (byte)234, (byte)43, (byte)184, (byte)200, (byte)100, (byte)112, (byte)182, (byte)221, (byte)234, (byte)87, (byte)121, (byte)59, (byte)170, (byte)122, (byte)55, (byte)3, (byte)201, (byte)208, (byte)5, (byte)21, (byte)153, (byte)43, (byte)105, (byte)99, (byte)176, (byte)170, (byte)227, (byte)94, (byte)5, (byte)209, (byte)180, (byte)66, (byte)64, (byte)152, (byte)98, (byte)201, (byte)76, (byte)65, (byte)149, (byte)80, (byte)191, (byte)195, (byte)189, (byte)54, (byte)80, (byte)243, (byte)223, (byte)206, (byte)2, (byte)14, (byte)181, (byte)59, (byte)220, (byte)179, (byte)89, (byte)213, (byte)26}, 0) ;
            p248.target_system = (byte)(byte)20;
            p248.target_network = (byte)(byte)69;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ver == (byte)(byte)1);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 46, (sbyte) - 64, (sbyte) - 96, (sbyte)69, (sbyte)11, (sbyte)86, (sbyte) - 85, (sbyte)9, (sbyte)49, (sbyte)53, (sbyte) - 120, (sbyte)69, (sbyte)113, (sbyte)35, (sbyte) - 46, (sbyte)35, (sbyte) - 95, (sbyte)118, (sbyte)76, (sbyte)25, (sbyte)114, (sbyte) - 4, (sbyte) - 124, (sbyte)30, (sbyte) - 107, (sbyte) - 83, (sbyte)106, (sbyte)5, (sbyte)83, (sbyte)8, (sbyte)59, (sbyte)0}));
                Debug.Assert(pack.address == (ushort)(ushort)18242);
                Debug.Assert(pack.type == (byte)(byte)48);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.type = (byte)(byte)48;
            p249.ver = (byte)(byte)1;
            p249.address = (ushort)(ushort)18242;
            p249.value_SET(new sbyte[] {(sbyte) - 46, (sbyte) - 64, (sbyte) - 96, (sbyte)69, (sbyte)11, (sbyte)86, (sbyte) - 85, (sbyte)9, (sbyte)49, (sbyte)53, (sbyte) - 120, (sbyte)69, (sbyte)113, (sbyte)35, (sbyte) - 46, (sbyte)35, (sbyte) - 95, (sbyte)118, (sbyte)76, (sbyte)25, (sbyte)114, (sbyte) - 4, (sbyte) - 124, (sbyte)30, (sbyte) - 107, (sbyte) - 83, (sbyte)106, (sbyte)5, (sbyte)83, (sbyte)8, (sbyte)59, (sbyte)0}, 0) ;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)5.0838757E37F);
                Debug.Assert(pack.time_usec == (ulong)8722779235680204878L);
                Debug.Assert(pack.y == (float) -2.2553756E38F);
                Debug.Assert(pack.x == (float)1.4190777E37F);
                Debug.Assert(pack.name_LEN(ph) == 9);
                Debug.Assert(pack.name_TRY(ph).Equals("qaukuocyv"));
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.x = (float)1.4190777E37F;
            p250.time_usec = (ulong)8722779235680204878L;
            p250.y = (float) -2.2553756E38F;
            p250.name_SET("qaukuocyv", PH) ;
            p250.z = (float)5.0838757E37F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3018622462U);
                Debug.Assert(pack.value == (float)3.3060727E38F);
                Debug.Assert(pack.name_LEN(ph) == 6);
                Debug.Assert(pack.name_TRY(ph).Equals("avWqnL"));
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.name_SET("avWqnL", PH) ;
            p251.time_boot_ms = (uint)3018622462U;
            p251.value = (float)3.3060727E38F;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("abjzozcbvg"));
                Debug.Assert(pack.value == (int)975499606);
                Debug.Assert(pack.time_boot_ms == (uint)1320334477U);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.name_SET("abjzozcbvg", PH) ;
            p252.time_boot_ms = (uint)1320334477U;
            p252.value = (int)975499606;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 26);
                Debug.Assert(pack.text_TRY(ph).Equals("hparFwzfllxnkhgpmgicauLdqm"));
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_NOTICE);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("hparFwzfllxnkhgpmgicauLdqm", PH) ;
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_NOTICE;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ind == (byte)(byte)208);
                Debug.Assert(pack.value == (float)1.1248316E38F);
                Debug.Assert(pack.time_boot_ms == (uint)994194813U);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.value = (float)1.1248316E38F;
            p254.ind = (byte)(byte)208;
            p254.time_boot_ms = (uint)994194813U;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.initial_timestamp == (ulong)9161635316528310411L);
                Debug.Assert(pack.target_component == (byte)(byte)1);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)207, (byte)1, (byte)102, (byte)190, (byte)1, (byte)100, (byte)228, (byte)31, (byte)115, (byte)58, (byte)67, (byte)127, (byte)160, (byte)238, (byte)100, (byte)232, (byte)93, (byte)188, (byte)108, (byte)219, (byte)146, (byte)251, (byte)123, (byte)10, (byte)46, (byte)188, (byte)61, (byte)164, (byte)198, (byte)238, (byte)110, (byte)207}));
                Debug.Assert(pack.target_system == (byte)(byte)177);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.secret_key_SET(new byte[] {(byte)207, (byte)1, (byte)102, (byte)190, (byte)1, (byte)100, (byte)228, (byte)31, (byte)115, (byte)58, (byte)67, (byte)127, (byte)160, (byte)238, (byte)100, (byte)232, (byte)93, (byte)188, (byte)108, (byte)219, (byte)146, (byte)251, (byte)123, (byte)10, (byte)46, (byte)188, (byte)61, (byte)164, (byte)198, (byte)238, (byte)110, (byte)207}, 0) ;
            p256.target_component = (byte)(byte)1;
            p256.initial_timestamp = (ulong)9161635316528310411L;
            p256.target_system = (byte)(byte)177;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.state == (byte)(byte)39);
                Debug.Assert(pack.last_change_ms == (uint)2906282848U);
                Debug.Assert(pack.time_boot_ms == (uint)3521641861U);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.state = (byte)(byte)39;
            p257.last_change_ms = (uint)2906282848U;
            p257.time_boot_ms = (uint)3521641861U;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)159);
                Debug.Assert(pack.tune_LEN(ph) == 27);
                Debug.Assert(pack.tune_TRY(ph).Equals("eemwthiqgmmtvpJdeyniznjfxsc"));
                Debug.Assert(pack.target_system == (byte)(byte)35);
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.tune_SET("eemwthiqgmmtvpJdeyniznjfxsc", PH) ;
            p258.target_component = (byte)(byte)159;
            p258.target_system = (byte)(byte)35;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.focal_length == (float)4.713428E37F);
                Debug.Assert(pack.firmware_version == (uint)1924821594U);
                Debug.Assert(pack.lens_id == (byte)(byte)93);
                Debug.Assert(pack.sensor_size_h == (float)2.9074086E38F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)20632);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
                Debug.Assert(pack.sensor_size_v == (float)8.71595E37F);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)64650);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 127);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("nzemYvfdZqaeaptrXqswDlegwmHryssdjSrcgjxafjWqvtjypcvfplgoedWogybXdDzZqjumxpaokdspxlszstshehurbWXgyrjhpwkZyxytuBxYtpsetIdlqtIFiqg"));
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)229, (byte)201, (byte)52, (byte)175, (byte)33, (byte)32, (byte)71, (byte)138, (byte)185, (byte)67, (byte)232, (byte)106, (byte)170, (byte)97, (byte)216, (byte)35, (byte)25, (byte)213, (byte)239, (byte)213, (byte)79, (byte)240, (byte)236, (byte)185, (byte)210, (byte)149, (byte)243, (byte)30, (byte)226, (byte)236, (byte)113, (byte)28}));
                Debug.Assert(pack.time_boot_ms == (uint)1002934802U);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)208, (byte)235, (byte)89, (byte)58, (byte)224, (byte)78, (byte)46, (byte)28, (byte)47, (byte)226, (byte)229, (byte)199, (byte)24, (byte)250, (byte)255, (byte)27, (byte)31, (byte)75, (byte)30, (byte)134, (byte)236, (byte)243, (byte)108, (byte)5, (byte)201, (byte)1, (byte)174, (byte)189, (byte)157, (byte)90, (byte)178, (byte)145}));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)19437);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.firmware_version = (uint)1924821594U;
            p259.resolution_h = (ushort)(ushort)20632;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO;
            p259.sensor_size_v = (float)8.71595E37F;
            p259.time_boot_ms = (uint)1002934802U;
            p259.model_name_SET(new byte[] {(byte)208, (byte)235, (byte)89, (byte)58, (byte)224, (byte)78, (byte)46, (byte)28, (byte)47, (byte)226, (byte)229, (byte)199, (byte)24, (byte)250, (byte)255, (byte)27, (byte)31, (byte)75, (byte)30, (byte)134, (byte)236, (byte)243, (byte)108, (byte)5, (byte)201, (byte)1, (byte)174, (byte)189, (byte)157, (byte)90, (byte)178, (byte)145}, 0) ;
            p259.vendor_name_SET(new byte[] {(byte)229, (byte)201, (byte)52, (byte)175, (byte)33, (byte)32, (byte)71, (byte)138, (byte)185, (byte)67, (byte)232, (byte)106, (byte)170, (byte)97, (byte)216, (byte)35, (byte)25, (byte)213, (byte)239, (byte)213, (byte)79, (byte)240, (byte)236, (byte)185, (byte)210, (byte)149, (byte)243, (byte)30, (byte)226, (byte)236, (byte)113, (byte)28}, 0) ;
            p259.cam_definition_version = (ushort)(ushort)64650;
            p259.cam_definition_uri_SET("nzemYvfdZqaeaptrXqswDlegwmHryssdjSrcgjxafjWqvtjypcvfplgoedWogybXdDzZqjumxpaokdspxlszstshehurbWXgyrjhpwkZyxytuBxYtpsetIdlqtIFiqg", PH) ;
            p259.sensor_size_h = (float)2.9074086E38F;
            p259.lens_id = (byte)(byte)93;
            p259.resolution_v = (ushort)(ushort)19437;
            p259.focal_length = (float)4.713428E37F;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
                Debug.Assert(pack.time_boot_ms == (uint)3808384292U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)3808384292U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.read_speed == (float) -9.268164E37F);
                Debug.Assert(pack.used_capacity == (float)1.2551177E38F);
                Debug.Assert(pack.total_capacity == (float) -3.2985289E38F);
                Debug.Assert(pack.available_capacity == (float)1.94397E38F);
                Debug.Assert(pack.time_boot_ms == (uint)303327449U);
                Debug.Assert(pack.status == (byte)(byte)212);
                Debug.Assert(pack.storage_id == (byte)(byte)61);
                Debug.Assert(pack.write_speed == (float) -2.7014861E38F);
                Debug.Assert(pack.storage_count == (byte)(byte)145);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.write_speed = (float) -2.7014861E38F;
            p261.available_capacity = (float)1.94397E38F;
            p261.time_boot_ms = (uint)303327449U;
            p261.storage_id = (byte)(byte)61;
            p261.storage_count = (byte)(byte)145;
            p261.status = (byte)(byte)212;
            p261.read_speed = (float) -9.268164E37F;
            p261.used_capacity = (float)1.2551177E38F;
            p261.total_capacity = (float) -3.2985289E38F;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1186888380U);
                Debug.Assert(pack.recording_time_ms == (uint)2260412570U);
                Debug.Assert(pack.video_status == (byte)(byte)237);
                Debug.Assert(pack.image_status == (byte)(byte)152);
                Debug.Assert(pack.image_interval == (float)3.163827E38F);
                Debug.Assert(pack.available_capacity == (float)7.5661354E37F);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.available_capacity = (float)7.5661354E37F;
            p262.image_interval = (float)3.163827E38F;
            p262.recording_time_ms = (uint)2260412570U;
            p262.time_boot_ms = (uint)1186888380U;
            p262.video_status = (byte)(byte)237;
            p262.image_status = (byte)(byte)152;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)254);
                Debug.Assert(pack.file_url_LEN(ph) == 18);
                Debug.Assert(pack.file_url_TRY(ph).Equals("ritjuGunvZnoZEsfZq"));
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 42);
                Debug.Assert(pack.relative_alt == (int) -491334164);
                Debug.Assert(pack.time_boot_ms == (uint)2776274196U);
                Debug.Assert(pack.lat == (int) -514049091);
                Debug.Assert(pack.image_index == (int)2048607096);
                Debug.Assert(pack.lon == (int) -482883693);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.277043E38F, 1.4152011E38F, -3.8847852E37F, -2.7285226E38F}));
                Debug.Assert(pack.alt == (int) -768833095);
                Debug.Assert(pack.time_utc == (ulong)37718377280208072L);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.lat = (int) -514049091;
            p263.file_url_SET("ritjuGunvZnoZEsfZq", PH) ;
            p263.capture_result = (sbyte)(sbyte) - 42;
            p263.time_utc = (ulong)37718377280208072L;
            p263.relative_alt = (int) -491334164;
            p263.time_boot_ms = (uint)2776274196U;
            p263.image_index = (int)2048607096;
            p263.q_SET(new float[] {2.277043E38F, 1.4152011E38F, -3.8847852E37F, -2.7285226E38F}, 0) ;
            p263.camera_id = (byte)(byte)254;
            p263.lon = (int) -482883693;
            p263.alt = (int) -768833095;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.arming_time_utc == (ulong)5950773679761906457L);
                Debug.Assert(pack.flight_uuid == (ulong)9218067090096178170L);
                Debug.Assert(pack.time_boot_ms == (uint)3517065812U);
                Debug.Assert(pack.takeoff_time_utc == (ulong)6860591657441960972L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.takeoff_time_utc = (ulong)6860591657441960972L;
            p264.arming_time_utc = (ulong)5950773679761906457L;
            p264.time_boot_ms = (uint)3517065812U;
            p264.flight_uuid = (ulong)9218067090096178170L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)3.2193307E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2816436714U);
                Debug.Assert(pack.roll == (float)6.182944E37F);
                Debug.Assert(pack.pitch == (float)3.0443532E38F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.pitch = (float)3.0443532E38F;
            p265.roll = (float)6.182944E37F;
            p265.time_boot_ms = (uint)2816436714U;
            p265.yaw = (float)3.2193307E38F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)226, (byte)2, (byte)53, (byte)44, (byte)165, (byte)33, (byte)242, (byte)35, (byte)15, (byte)68, (byte)53, (byte)254, (byte)35, (byte)59, (byte)67, (byte)177, (byte)70, (byte)44, (byte)49, (byte)168, (byte)166, (byte)235, (byte)30, (byte)19, (byte)3, (byte)19, (byte)73, (byte)101, (byte)104, (byte)79, (byte)7, (byte)78, (byte)128, (byte)40, (byte)241, (byte)95, (byte)142, (byte)77, (byte)236, (byte)97, (byte)164, (byte)68, (byte)188, (byte)214, (byte)192, (byte)18, (byte)203, (byte)85, (byte)159, (byte)77, (byte)167, (byte)86, (byte)33, (byte)189, (byte)102, (byte)64, (byte)125, (byte)87, (byte)125, (byte)104, (byte)233, (byte)209, (byte)253, (byte)64, (byte)186, (byte)120, (byte)180, (byte)190, (byte)166, (byte)80, (byte)117, (byte)115, (byte)145, (byte)39, (byte)183, (byte)99, (byte)93, (byte)32, (byte)16, (byte)202, (byte)154, (byte)196, (byte)23, (byte)240, (byte)212, (byte)44, (byte)227, (byte)54, (byte)70, (byte)79, (byte)141, (byte)13, (byte)121, (byte)152, (byte)111, (byte)206, (byte)91, (byte)210, (byte)77, (byte)23, (byte)221, (byte)228, (byte)207, (byte)74, (byte)189, (byte)55, (byte)244, (byte)112, (byte)81, (byte)103, (byte)144, (byte)59, (byte)20, (byte)16, (byte)217, (byte)189, (byte)252, (byte)62, (byte)166, (byte)4, (byte)23, (byte)138, (byte)111, (byte)125, (byte)114, (byte)142, (byte)51, (byte)93, (byte)202, (byte)211, (byte)47, (byte)249, (byte)110, (byte)100, (byte)172, (byte)209, (byte)10, (byte)248, (byte)68, (byte)196, (byte)55, (byte)237, (byte)171, (byte)187, (byte)76, (byte)155, (byte)132, (byte)170, (byte)254, (byte)93, (byte)54, (byte)147, (byte)42, (byte)56, (byte)84, (byte)12, (byte)90, (byte)192, (byte)108, (byte)42, (byte)77, (byte)8, (byte)134, (byte)216, (byte)123, (byte)66, (byte)131, (byte)8, (byte)5, (byte)159, (byte)196, (byte)123, (byte)201, (byte)211, (byte)166, (byte)195, (byte)7, (byte)218, (byte)198, (byte)180, (byte)178, (byte)218, (byte)34, (byte)88, (byte)230, (byte)229, (byte)77, (byte)172, (byte)194, (byte)234, (byte)230, (byte)40, (byte)235, (byte)232, (byte)10, (byte)44, (byte)152, (byte)81, (byte)169, (byte)191, (byte)72, (byte)7, (byte)175, (byte)0, (byte)57, (byte)255, (byte)85, (byte)148, (byte)215, (byte)193, (byte)34, (byte)106, (byte)169, (byte)236, (byte)88, (byte)138, (byte)148, (byte)69, (byte)103, (byte)2, (byte)214, (byte)71, (byte)239, (byte)83, (byte)36, (byte)65, (byte)56, (byte)245, (byte)168, (byte)26, (byte)1, (byte)52, (byte)71, (byte)48, (byte)233, (byte)170, (byte)170, (byte)96, (byte)10, (byte)148, (byte)225, (byte)26, (byte)161, (byte)87, (byte)45, (byte)60, (byte)112, (byte)158, (byte)117}));
                Debug.Assert(pack.length == (byte)(byte)176);
                Debug.Assert(pack.first_message_offset == (byte)(byte)117);
                Debug.Assert(pack.sequence == (ushort)(ushort)2224);
                Debug.Assert(pack.target_component == (byte)(byte)29);
                Debug.Assert(pack.target_system == (byte)(byte)239);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)239;
            p266.data__SET(new byte[] {(byte)226, (byte)2, (byte)53, (byte)44, (byte)165, (byte)33, (byte)242, (byte)35, (byte)15, (byte)68, (byte)53, (byte)254, (byte)35, (byte)59, (byte)67, (byte)177, (byte)70, (byte)44, (byte)49, (byte)168, (byte)166, (byte)235, (byte)30, (byte)19, (byte)3, (byte)19, (byte)73, (byte)101, (byte)104, (byte)79, (byte)7, (byte)78, (byte)128, (byte)40, (byte)241, (byte)95, (byte)142, (byte)77, (byte)236, (byte)97, (byte)164, (byte)68, (byte)188, (byte)214, (byte)192, (byte)18, (byte)203, (byte)85, (byte)159, (byte)77, (byte)167, (byte)86, (byte)33, (byte)189, (byte)102, (byte)64, (byte)125, (byte)87, (byte)125, (byte)104, (byte)233, (byte)209, (byte)253, (byte)64, (byte)186, (byte)120, (byte)180, (byte)190, (byte)166, (byte)80, (byte)117, (byte)115, (byte)145, (byte)39, (byte)183, (byte)99, (byte)93, (byte)32, (byte)16, (byte)202, (byte)154, (byte)196, (byte)23, (byte)240, (byte)212, (byte)44, (byte)227, (byte)54, (byte)70, (byte)79, (byte)141, (byte)13, (byte)121, (byte)152, (byte)111, (byte)206, (byte)91, (byte)210, (byte)77, (byte)23, (byte)221, (byte)228, (byte)207, (byte)74, (byte)189, (byte)55, (byte)244, (byte)112, (byte)81, (byte)103, (byte)144, (byte)59, (byte)20, (byte)16, (byte)217, (byte)189, (byte)252, (byte)62, (byte)166, (byte)4, (byte)23, (byte)138, (byte)111, (byte)125, (byte)114, (byte)142, (byte)51, (byte)93, (byte)202, (byte)211, (byte)47, (byte)249, (byte)110, (byte)100, (byte)172, (byte)209, (byte)10, (byte)248, (byte)68, (byte)196, (byte)55, (byte)237, (byte)171, (byte)187, (byte)76, (byte)155, (byte)132, (byte)170, (byte)254, (byte)93, (byte)54, (byte)147, (byte)42, (byte)56, (byte)84, (byte)12, (byte)90, (byte)192, (byte)108, (byte)42, (byte)77, (byte)8, (byte)134, (byte)216, (byte)123, (byte)66, (byte)131, (byte)8, (byte)5, (byte)159, (byte)196, (byte)123, (byte)201, (byte)211, (byte)166, (byte)195, (byte)7, (byte)218, (byte)198, (byte)180, (byte)178, (byte)218, (byte)34, (byte)88, (byte)230, (byte)229, (byte)77, (byte)172, (byte)194, (byte)234, (byte)230, (byte)40, (byte)235, (byte)232, (byte)10, (byte)44, (byte)152, (byte)81, (byte)169, (byte)191, (byte)72, (byte)7, (byte)175, (byte)0, (byte)57, (byte)255, (byte)85, (byte)148, (byte)215, (byte)193, (byte)34, (byte)106, (byte)169, (byte)236, (byte)88, (byte)138, (byte)148, (byte)69, (byte)103, (byte)2, (byte)214, (byte)71, (byte)239, (byte)83, (byte)36, (byte)65, (byte)56, (byte)245, (byte)168, (byte)26, (byte)1, (byte)52, (byte)71, (byte)48, (byte)233, (byte)170, (byte)170, (byte)96, (byte)10, (byte)148, (byte)225, (byte)26, (byte)161, (byte)87, (byte)45, (byte)60, (byte)112, (byte)158, (byte)117}, 0) ;
            p266.sequence = (ushort)(ushort)2224;
            p266.first_message_offset = (byte)(byte)117;
            p266.target_component = (byte)(byte)29;
            p266.length = (byte)(byte)176;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)193, (byte)187, (byte)201, (byte)245, (byte)42, (byte)143, (byte)42, (byte)50, (byte)215, (byte)186, (byte)218, (byte)166, (byte)80, (byte)95, (byte)173, (byte)122, (byte)240, (byte)170, (byte)63, (byte)205, (byte)19, (byte)6, (byte)203, (byte)108, (byte)73, (byte)53, (byte)105, (byte)121, (byte)81, (byte)23, (byte)109, (byte)182, (byte)147, (byte)63, (byte)46, (byte)147, (byte)121, (byte)163, (byte)192, (byte)52, (byte)226, (byte)124, (byte)151, (byte)86, (byte)146, (byte)89, (byte)182, (byte)250, (byte)199, (byte)242, (byte)27, (byte)104, (byte)125, (byte)127, (byte)254, (byte)182, (byte)230, (byte)165, (byte)158, (byte)237, (byte)156, (byte)168, (byte)190, (byte)48, (byte)203, (byte)63, (byte)179, (byte)119, (byte)5, (byte)221, (byte)139, (byte)96, (byte)71, (byte)143, (byte)251, (byte)60, (byte)180, (byte)255, (byte)132, (byte)230, (byte)77, (byte)6, (byte)13, (byte)105, (byte)5, (byte)74, (byte)65, (byte)85, (byte)172, (byte)231, (byte)45, (byte)165, (byte)70, (byte)204, (byte)225, (byte)146, (byte)4, (byte)29, (byte)149, (byte)146, (byte)217, (byte)179, (byte)238, (byte)158, (byte)200, (byte)204, (byte)123, (byte)154, (byte)87, (byte)141, (byte)222, (byte)241, (byte)223, (byte)199, (byte)117, (byte)194, (byte)94, (byte)158, (byte)245, (byte)83, (byte)250, (byte)105, (byte)18, (byte)152, (byte)201, (byte)52, (byte)2, (byte)163, (byte)165, (byte)111, (byte)99, (byte)149, (byte)50, (byte)86, (byte)190, (byte)170, (byte)86, (byte)144, (byte)13, (byte)56, (byte)199, (byte)136, (byte)232, (byte)90, (byte)17, (byte)47, (byte)10, (byte)177, (byte)137, (byte)98, (byte)160, (byte)39, (byte)178, (byte)83, (byte)75, (byte)168, (byte)134, (byte)110, (byte)63, (byte)20, (byte)134, (byte)232, (byte)36, (byte)144, (byte)250, (byte)144, (byte)66, (byte)98, (byte)130, (byte)154, (byte)71, (byte)217, (byte)235, (byte)52, (byte)222, (byte)6, (byte)105, (byte)34, (byte)83, (byte)253, (byte)132, (byte)110, (byte)119, (byte)62, (byte)59, (byte)104, (byte)122, (byte)71, (byte)243, (byte)10, (byte)160, (byte)63, (byte)172, (byte)82, (byte)59, (byte)104, (byte)245, (byte)222, (byte)70, (byte)162, (byte)97, (byte)125, (byte)184, (byte)100, (byte)180, (byte)91, (byte)250, (byte)185, (byte)244, (byte)100, (byte)206, (byte)144, (byte)202, (byte)253, (byte)196, (byte)220, (byte)246, (byte)0, (byte)139, (byte)56, (byte)92, (byte)25, (byte)104, (byte)244, (byte)75, (byte)233, (byte)125, (byte)173, (byte)42, (byte)71, (byte)200, (byte)219, (byte)33, (byte)189, (byte)53, (byte)31, (byte)88, (byte)64, (byte)49, (byte)251, (byte)68, (byte)63, (byte)207, (byte)237, (byte)46, (byte)250, (byte)230, (byte)217, (byte)34}));
                Debug.Assert(pack.first_message_offset == (byte)(byte)146);
                Debug.Assert(pack.target_system == (byte)(byte)156);
                Debug.Assert(pack.sequence == (ushort)(ushort)23336);
                Debug.Assert(pack.length == (byte)(byte)11);
                Debug.Assert(pack.target_component == (byte)(byte)175);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.data__SET(new byte[] {(byte)193, (byte)187, (byte)201, (byte)245, (byte)42, (byte)143, (byte)42, (byte)50, (byte)215, (byte)186, (byte)218, (byte)166, (byte)80, (byte)95, (byte)173, (byte)122, (byte)240, (byte)170, (byte)63, (byte)205, (byte)19, (byte)6, (byte)203, (byte)108, (byte)73, (byte)53, (byte)105, (byte)121, (byte)81, (byte)23, (byte)109, (byte)182, (byte)147, (byte)63, (byte)46, (byte)147, (byte)121, (byte)163, (byte)192, (byte)52, (byte)226, (byte)124, (byte)151, (byte)86, (byte)146, (byte)89, (byte)182, (byte)250, (byte)199, (byte)242, (byte)27, (byte)104, (byte)125, (byte)127, (byte)254, (byte)182, (byte)230, (byte)165, (byte)158, (byte)237, (byte)156, (byte)168, (byte)190, (byte)48, (byte)203, (byte)63, (byte)179, (byte)119, (byte)5, (byte)221, (byte)139, (byte)96, (byte)71, (byte)143, (byte)251, (byte)60, (byte)180, (byte)255, (byte)132, (byte)230, (byte)77, (byte)6, (byte)13, (byte)105, (byte)5, (byte)74, (byte)65, (byte)85, (byte)172, (byte)231, (byte)45, (byte)165, (byte)70, (byte)204, (byte)225, (byte)146, (byte)4, (byte)29, (byte)149, (byte)146, (byte)217, (byte)179, (byte)238, (byte)158, (byte)200, (byte)204, (byte)123, (byte)154, (byte)87, (byte)141, (byte)222, (byte)241, (byte)223, (byte)199, (byte)117, (byte)194, (byte)94, (byte)158, (byte)245, (byte)83, (byte)250, (byte)105, (byte)18, (byte)152, (byte)201, (byte)52, (byte)2, (byte)163, (byte)165, (byte)111, (byte)99, (byte)149, (byte)50, (byte)86, (byte)190, (byte)170, (byte)86, (byte)144, (byte)13, (byte)56, (byte)199, (byte)136, (byte)232, (byte)90, (byte)17, (byte)47, (byte)10, (byte)177, (byte)137, (byte)98, (byte)160, (byte)39, (byte)178, (byte)83, (byte)75, (byte)168, (byte)134, (byte)110, (byte)63, (byte)20, (byte)134, (byte)232, (byte)36, (byte)144, (byte)250, (byte)144, (byte)66, (byte)98, (byte)130, (byte)154, (byte)71, (byte)217, (byte)235, (byte)52, (byte)222, (byte)6, (byte)105, (byte)34, (byte)83, (byte)253, (byte)132, (byte)110, (byte)119, (byte)62, (byte)59, (byte)104, (byte)122, (byte)71, (byte)243, (byte)10, (byte)160, (byte)63, (byte)172, (byte)82, (byte)59, (byte)104, (byte)245, (byte)222, (byte)70, (byte)162, (byte)97, (byte)125, (byte)184, (byte)100, (byte)180, (byte)91, (byte)250, (byte)185, (byte)244, (byte)100, (byte)206, (byte)144, (byte)202, (byte)253, (byte)196, (byte)220, (byte)246, (byte)0, (byte)139, (byte)56, (byte)92, (byte)25, (byte)104, (byte)244, (byte)75, (byte)233, (byte)125, (byte)173, (byte)42, (byte)71, (byte)200, (byte)219, (byte)33, (byte)189, (byte)53, (byte)31, (byte)88, (byte)64, (byte)49, (byte)251, (byte)68, (byte)63, (byte)207, (byte)237, (byte)46, (byte)250, (byte)230, (byte)217, (byte)34}, 0) ;
            p267.sequence = (ushort)(ushort)23336;
            p267.target_system = (byte)(byte)156;
            p267.first_message_offset = (byte)(byte)146;
            p267.target_component = (byte)(byte)175;
            p267.length = (byte)(byte)11;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)64);
                Debug.Assert(pack.target_system == (byte)(byte)68);
                Debug.Assert(pack.sequence == (ushort)(ushort)41654);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.sequence = (ushort)(ushort)41654;
            p268.target_system = (byte)(byte)68;
            p268.target_component = (byte)(byte)64;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)46500);
                Debug.Assert(pack.framerate == (float)3.0832869E38F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)40387);
                Debug.Assert(pack.rotation == (ushort)(ushort)36052);
                Debug.Assert(pack.camera_id == (byte)(byte)129);
                Debug.Assert(pack.status == (byte)(byte)237);
                Debug.Assert(pack.bitrate == (uint)1056935990U);
                Debug.Assert(pack.uri_LEN(ph) == 24);
                Debug.Assert(pack.uri_TRY(ph).Equals("ojroEEchipibVmlsegqughkw"));
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.resolution_v = (ushort)(ushort)46500;
            p269.rotation = (ushort)(ushort)36052;
            p269.camera_id = (byte)(byte)129;
            p269.resolution_h = (ushort)(ushort)40387;
            p269.bitrate = (uint)1056935990U;
            p269.framerate = (float)3.0832869E38F;
            p269.uri_SET("ojroEEchipibVmlsegqughkw", PH) ;
            p269.status = (byte)(byte)237;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_h == (ushort)(ushort)718);
                Debug.Assert(pack.uri_LEN(ph) == 110);
                Debug.Assert(pack.uri_TRY(ph).Equals("etyfzdkbSindrxphBsHrkvgWglxxdlZedtdeiGshRuqdqwylVrbuziujdyfnESsnhcmmdvvdqaglfikptweowUmnpbrreqkanosovkehjyrIyF"));
                Debug.Assert(pack.target_system == (byte)(byte)15);
                Debug.Assert(pack.framerate == (float) -1.3447451E38F);
                Debug.Assert(pack.bitrate == (uint)3495168129U);
                Debug.Assert(pack.camera_id == (byte)(byte)254);
                Debug.Assert(pack.target_component == (byte)(byte)162);
                Debug.Assert(pack.rotation == (ushort)(ushort)14532);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)3004);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.uri_SET("etyfzdkbSindrxphBsHrkvgWglxxdlZedtdeiGshRuqdqwylVrbuziujdyfnESsnhcmmdvvdqaglfikptweowUmnpbrreqkanosovkehjyrIyF", PH) ;
            p270.rotation = (ushort)(ushort)14532;
            p270.target_system = (byte)(byte)15;
            p270.target_component = (byte)(byte)162;
            p270.framerate = (float) -1.3447451E38F;
            p270.resolution_h = (ushort)(ushort)718;
            p270.camera_id = (byte)(byte)254;
            p270.bitrate = (uint)3495168129U;
            p270.resolution_v = (ushort)(ushort)3004;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 54);
                Debug.Assert(pack.password_TRY(ph).Equals("idymivvbbsvpvmapiozvamsvdjxfwvylitavxmjvxUbsbhzvcpjOri"));
                Debug.Assert(pack.ssid_LEN(ph) == 9);
                Debug.Assert(pack.ssid_TRY(ph).Equals("pgshwpjfb"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("pgshwpjfb", PH) ;
            p299.password_SET("idymivvbbsvpvmapiozvamsvdjxfwvylitavxmjvxUbsbhzvcpjOri", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)245, (byte)79, (byte)22, (byte)162, (byte)237, (byte)214, (byte)79, (byte)26}));
                Debug.Assert(pack.max_version == (ushort)(ushort)42127);
                Debug.Assert(pack.version == (ushort)(ushort)40987);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)127, (byte)134, (byte)158, (byte)226, (byte)1, (byte)113, (byte)204, (byte)11}));
                Debug.Assert(pack.min_version == (ushort)(ushort)16493);
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.library_version_hash_SET(new byte[] {(byte)127, (byte)134, (byte)158, (byte)226, (byte)1, (byte)113, (byte)204, (byte)11}, 0) ;
            p300.max_version = (ushort)(ushort)42127;
            p300.spec_version_hash_SET(new byte[] {(byte)245, (byte)79, (byte)22, (byte)162, (byte)237, (byte)214, (byte)79, (byte)26}, 0) ;
            p300.min_version = (ushort)(ushort)16493;
            p300.version = (ushort)(ushort)40987;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
                Debug.Assert(pack.sub_mode == (byte)(byte)70);
                Debug.Assert(pack.uptime_sec == (uint)3512960572U);
                Debug.Assert(pack.time_usec == (ulong)2594100442325469199L);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)4643);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.vendor_specific_status_code = (ushort)(ushort)4643;
            p310.sub_mode = (byte)(byte)70;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            p310.uptime_sec = (uint)3512960572U;
            p310.time_usec = (ulong)2594100442325469199L;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hw_version_major == (byte)(byte)66);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)39);
                Debug.Assert(pack.sw_version_major == (byte)(byte)206);
                Debug.Assert(pack.name_LEN(ph) == 77);
                Debug.Assert(pack.name_TRY(ph).Equals("uyrtkgbgyubstwuzcqmfIskdjqdkxdzjzkqutaIbkcnHqertxwoyqqKgmPithYmviicoggxrygwjO"));
                Debug.Assert(pack.sw_vcs_commit == (uint)3098188481U);
                Debug.Assert(pack.time_usec == (ulong)6766292820217446550L);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)31);
                Debug.Assert(pack.uptime_sec == (uint)2547196777U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)178, (byte)73, (byte)54, (byte)35, (byte)80, (byte)91, (byte)180, (byte)52, (byte)78, (byte)70, (byte)17, (byte)168, (byte)86, (byte)36, (byte)32, (byte)192}));
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_unique_id_SET(new byte[] {(byte)178, (byte)73, (byte)54, (byte)35, (byte)80, (byte)91, (byte)180, (byte)52, (byte)78, (byte)70, (byte)17, (byte)168, (byte)86, (byte)36, (byte)32, (byte)192}, 0) ;
            p311.sw_version_major = (byte)(byte)206;
            p311.hw_version_minor = (byte)(byte)39;
            p311.uptime_sec = (uint)2547196777U;
            p311.name_SET("uyrtkgbgyubstwuzcqmfIskdjqdkxdzjzkqutaIbkcnHqertxwoyqqKgmPithYmviicoggxrygwjO", PH) ;
            p311.time_usec = (ulong)6766292820217446550L;
            p311.sw_version_minor = (byte)(byte)31;
            p311.hw_version_major = (byte)(byte)66;
            p311.sw_vcs_commit = (uint)3098188481U;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 6);
                Debug.Assert(pack.param_id_TRY(ph).Equals("orkbme"));
                Debug.Assert(pack.target_system == (byte)(byte)23);
                Debug.Assert(pack.param_index == (short)(short)9417);
                Debug.Assert(pack.target_component == (byte)(byte)24);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_index = (short)(short)9417;
            p320.target_component = (byte)(byte)24;
            p320.target_system = (byte)(byte)23;
            p320.param_id_SET("orkbme", PH) ;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)109);
                Debug.Assert(pack.target_component == (byte)(byte)128);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)109;
            p321.target_component = (byte)(byte)128;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 107);
                Debug.Assert(pack.param_value_TRY(ph).Equals("beabgduwkvgpvfmcenslnFebtoqtubjxzkyotyaaZvnUafmzhchgepehyyGpltKnmqkdbzShrNFgozzbxpsmdguagxzrlchgxbaesjiyvox"));
                Debug.Assert(pack.param_count == (ushort)(ushort)41321);
                Debug.Assert(pack.param_id_LEN(ph) == 14);
                Debug.Assert(pack.param_id_TRY(ph).Equals("nisuibnneGpmgn"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
                Debug.Assert(pack.param_index == (ushort)(ushort)52096);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_index = (ushort)(ushort)52096;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            p322.param_value_SET("beabgduwkvgpvfmcenslnFebtoqtubjxzkyotyaaZvnUafmzhchgepehyyGpltKnmqkdbzShrNFgozzbxpsmdguagxzrlchgxbaesjiyvox", PH) ;
            p322.param_count = (ushort)(ushort)41321;
            p322.param_id_SET("nisuibnneGpmgn", PH) ;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)171);
                Debug.Assert(pack.param_value_LEN(ph) == 20);
                Debug.Assert(pack.param_value_TRY(ph).Equals("fmvgwsrcpszassryrima"));
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("wd"));
                Debug.Assert(pack.target_system == (byte)(byte)230);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_id_SET("wd", PH) ;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p323.target_system = (byte)(byte)230;
            p323.param_value_SET("fmvgwsrcpszassryrima", PH) ;
            p323.target_component = (byte)(byte)171;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.param_value_LEN(ph) == 9);
                Debug.Assert(pack.param_value_TRY(ph).Equals("hkOcylloe"));
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS);
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("xjnlmcuhjmtqgyjy"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_value_SET("hkOcylloe", PH) ;
            p324.param_id_SET("xjnlmcuhjmtqgyjy", PH) ;
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.increment == (byte)(byte)162);
                Debug.Assert(pack.time_usec == (ulong)5937978820165640169L);
                Debug.Assert(pack.min_distance == (ushort)(ushort)48214);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)28117, (ushort)55351, (ushort)60052, (ushort)20557, (ushort)52704, (ushort)5227, (ushort)37996, (ushort)18651, (ushort)25280, (ushort)56302, (ushort)59818, (ushort)30003, (ushort)44718, (ushort)42228, (ushort)26852, (ushort)11142, (ushort)33052, (ushort)45431, (ushort)6996, (ushort)38168, (ushort)38935, (ushort)55596, (ushort)31536, (ushort)11496, (ushort)64166, (ushort)2171, (ushort)9812, (ushort)49216, (ushort)57609, (ushort)53065, (ushort)14866, (ushort)9199, (ushort)59797, (ushort)40159, (ushort)56885, (ushort)18870, (ushort)11837, (ushort)62506, (ushort)22830, (ushort)64144, (ushort)16968, (ushort)3577, (ushort)37015, (ushort)46079, (ushort)45616, (ushort)50951, (ushort)26537, (ushort)16967, (ushort)7618, (ushort)54635, (ushort)50003, (ushort)38124, (ushort)18944, (ushort)21129, (ushort)48954, (ushort)4456, (ushort)1082, (ushort)54082, (ushort)24597, (ushort)16184, (ushort)23334, (ushort)61693, (ushort)52096, (ushort)57669, (ushort)57145, (ushort)10207, (ushort)22635, (ushort)54818, (ushort)50222, (ushort)61035, (ushort)14068, (ushort)49656}));
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
                Debug.Assert(pack.max_distance == (ushort)(ushort)4878);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)5937978820165640169L;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p330.max_distance = (ushort)(ushort)4878;
            p330.min_distance = (ushort)(ushort)48214;
            p330.distances_SET(new ushort[] {(ushort)28117, (ushort)55351, (ushort)60052, (ushort)20557, (ushort)52704, (ushort)5227, (ushort)37996, (ushort)18651, (ushort)25280, (ushort)56302, (ushort)59818, (ushort)30003, (ushort)44718, (ushort)42228, (ushort)26852, (ushort)11142, (ushort)33052, (ushort)45431, (ushort)6996, (ushort)38168, (ushort)38935, (ushort)55596, (ushort)31536, (ushort)11496, (ushort)64166, (ushort)2171, (ushort)9812, (ushort)49216, (ushort)57609, (ushort)53065, (ushort)14866, (ushort)9199, (ushort)59797, (ushort)40159, (ushort)56885, (ushort)18870, (ushort)11837, (ushort)62506, (ushort)22830, (ushort)64144, (ushort)16968, (ushort)3577, (ushort)37015, (ushort)46079, (ushort)45616, (ushort)50951, (ushort)26537, (ushort)16967, (ushort)7618, (ushort)54635, (ushort)50003, (ushort)38124, (ushort)18944, (ushort)21129, (ushort)48954, (ushort)4456, (ushort)1082, (ushort)54082, (ushort)24597, (ushort)16184, (ushort)23334, (ushort)61693, (ushort)52096, (ushort)57669, (ushort)57145, (ushort)10207, (ushort)22635, (ushort)54818, (ushort)50222, (ushort)61035, (ushort)14068, (ushort)49656}, 0) ;
            p330.increment = (byte)(byte)162;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
        }
    }
}